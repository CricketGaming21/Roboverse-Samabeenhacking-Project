"""Per-drone state + kinematic blocking-command execution.

Simulator INTERNAL — mission code must never import simcore.

One SimDrone per configured drone. The rules:
- ALL state mutation happens on the sim thread: the goal_*() installers are
  invoked via run_on_sim_thread, and step(dt) runs inside the registry's
  fixed-timestep loop.
- One active Goal at a time; a new motion command preempts (fails) the old.
- Motion is kinematic: straight line at the m/s mapped from VelocityLevel by
  NAME via config.velocity_levels (TURBO fastest). The enum INT is a firmware
  P-gain divisor (lower = faster) — it is NEVER used as a speed.
- Completion and timeouts are decided in SIM time (clock), never wall time.
- This is the TRUE pose (mirrored into PyBullet). The drifting estimate that
  get_position() reports is Phase 3 — do not conflate them.
- led/flags arguments are accepted by the API and ignored by the sim.
"""

import math
import threading

import numpy as np
import pybullet as p

from pyhulax.core import (BarrierMask, CameraPitchMode, CommandResult,
                          Direction, DroneState, Obstacles, Orientation,
                          Vector3, VelocityLevel)
from pyhulax.exceptions import LowBattery, NotReady, PyhulaxError

from . import frames, sensors
from .log import get_logger

# Goal deadline (sim seconds) = estimated duration * margin + base. Kinematics
# are exact, so these only fire on real bugs (e.g. speed 0 with distance > 0).
_TIMEOUT_MARGIN = 2.0
_TIMEOUT_BASE_S = 5.0


def speed_to_mps(cfg, speed) -> float:
    """Map VelocityLevel (enum or raw int) -> m/s by NAME via config.

    The enum int is the firmware's P-gain divisor (SLOW=300 .. TURBO=50,
    lower = faster); only the NAME keys into config.velocity_levels.
    """
    try:
        name = VelocityLevel(int(speed)).name
    except ValueError:
        raise PyhulaxError(
            f"invalid speed level {speed!r} — use VelocityLevel.SLOW/MEDIUM/"
            f"ZOOM/TURBO") from None
    return float(getattr(cfg.velocity_levels, name))


def _mask_tripped(flags: Obstacles, mask: int) -> bool:
    """Does any tripped sensor fall inside the armed BarrierMask?
    (There is no UP sensor; BarrierMask.UP simply never trips.)"""
    return bool(((mask & BarrierMask.FRONT) and flags.forward)
                or ((mask & BarrierMask.BACK) and flags.back)
                or ((mask & BarrierMask.LEFT) and flags.left)
                or ((mask & BarrierMask.RIGHT) and flags.right)
                or ((mask & BarrierMask.DOWN) and flags.down))


def _flag_names(flags: Obstacles) -> str:
    names = [n for n in ("forward", "back", "left", "right", "down")
             if getattr(flags, n)]
    return ",".join(names) or "none"


class Goal:
    """One in-flight command target. done/result are consumed by the waiter."""

    __slots__ = ("kind", "target_pos", "target_yaw", "speed_mps",
                 "yaw_rate_rps", "end_time", "deadline", "exec_after",
                 "done", "result")

    def __init__(self, kind, target_pos=None, target_yaw=None, speed_mps=0.0,
                 yaw_rate_rps=0.0, end_time=None, deadline=float("inf"),
                 exec_after=0.0):
        self.kind = kind
        self.target_pos = target_pos      # np.ndarray(3) world m, or None
        self.target_yaw = target_yaw      # world rad (unwrapped), or None
        self.speed_mps = speed_mps
        self.yaw_rate_rps = yaw_rate_rps
        self.end_time = end_time          # sim time the goal must last until
        self.deadline = deadline          # sim time after which it fails
        self.exec_after = exec_after      # command latency (realistic mode)
        self.done = threading.Event()
        self.result = None

    def complete(self, success: bool, message: str) -> None:
        self.result = CommandResult(success, message)
        self.done.set()


class SimDrone:
    """State + kinematic executor for one drone (sim-thread only mutation)."""

    def __init__(self, cfg, index, spec, body_id, client, clock,
                 start_pos, start_yaw, monitor=None, landing_scorer=None):
        self.cfg = cfg
        self.index = index
        self.monitor = monitor            # CommandMonitor (thrash watchdog)
        self.landing_scorer = landing_scorer  # part-1 referee (DEPLOY)
        self.spec = spec                  # DroneSpec (ip, uwb_tag_id, ...)
        self.body_id = body_id
        self._client = client
        self._clock = clock
        self._log = get_logger(f"drone{index}", cfg)
        self._half_z = float(cfg.bodies.drone_half_extents_m[2])
        self._ground_z = float(cfg.arena.origin[2])  # flat floor top (Phase 2)

        # TRUE pose (world m / rad, yaw unwrapped). Mirrored into PyBullet.
        self.pos = np.array(start_pos, dtype=float)
        self.yaw = float(start_yaw)

        self.connected = False
        self.flying = False
        self.takeoff_frame = None         # frames.TakeoffFrame, set at takeoff
        self.battery_pct = float(cfg.drones.battery.start_pct)
        self.goal = None

        # Drifting onboard estimate (optical-flow/IMU): estimate = true pose +
        # drift_err. A SLOW, LOW-FREQUENCY, strictly BOUNDED wander — the sum
        # of a few long-period sinusoids per axis (no high-frequency jitter),
        # calibrated to the confirmed optical-flow accuracy. Reset to 0 at
        # takeoff (re-anchor) so it grows from there. UWB never sees it (§5.3).
        pd = cfg.position_drift
        self._drift_enabled = bool(pd.enabled)
        self._drift_t = 0.0               # sim seconds since takeoff
        rng = np.random.default_rng([cfg.meta.seed, 1000 + index])
        n = max(1, int(pd.n_components))
        # per-axis: amplitudes summing to the bound, random periods + dirs;
        # zero phase => drift(0) = 0 exactly (clean re-anchor at takeoff).
        self._drift_freq = 2.0 * math.pi / rng.uniform(
            pd.period_min_s, pd.period_max_s, (3, n))
        amp = rng.uniform(0.4, 1.0, (3, n))
        amp /= amp.sum(axis=1, keepdims=True)   # rows sum to 1
        # Split the horizontal budget across x and y by sqrt(2) so the
        # horizontal NORM (not just each axis) stays <= horizontal_bound_m.
        hb = pd.horizontal_bound_m / math.sqrt(2.0)
        bounds = np.array([hb, hb, pd.vertical_bound_m])
        self._drift_amp = amp * bounds[:, None] * rng.choice([-1.0, 1.0],
                                                             (3, n))
        self.drift_err = np.zeros(3)      # world metres, estimate minus truth

        # Realistic motion model (config motion.*): BEHAVIOUR ONLY — ramps,
        # tilt-to-translate, momentum/settling, latency. NOT a firmware-
        # accurate dynamics replica (the Hula has no SITL to replicate).
        # motion.realistic=False keeps the crisp snap-to-target stepper.
        m = cfg.motion
        self._realistic = bool(m.realistic)
        self._accel = float(m.accel_mps2)
        self._max_tilt = float(m.max_tilt_deg)
        self._latency = float(m.latency_s)
        self._overshoot = float(m.overshoot_frac)
        self._tilt_rate = float(m.tilt_rate_dps)
        self._arrive_tol = float(m.arrive_tol_m)
        self._arrive_speed = float(m.arrive_speed_mps)
        self._wind_on = bool(m.wind.enabled)
        self._wind_mps = float(m.wind.speed_mps)
        self._wind_rng = np.random.default_rng([cfg.meta.seed, 4000 + index])
        self._wind_vec = np.zeros(2)
        self.vel = np.zeros(3)            # world m/s (realistic mode)
        self.tilt_deg = 0.0               # body pitch; negative = nose down

        # Manual (stick) control — send_manual_control / manual_fly. The sim
        # EXECUTES the inputs faithfully and contains NO PID: the control
        # loop that picks stick values is mission code.
        self.manual_stick = np.zeros(4)   # forward, right, up, rotate
        self._stick_fresh_until = 0.0     # sim time the inputs go stale
        self._hold_anchor = None          # zero-stick station-keep point
        self._manual_timeout = float(m.manual_timeout_s)

        # Reflexes (§4.4) — both route through the SAME committing executor.
        self.barrier_mode = False         # firmware auto-avoid (stop short)
        self.avoidance_rule = None        # (Direction, distance_m, mask) or None

        # Camera (§4.5): tiltable pitch, 0 = forward .. 90 = straight down.
        self.camera_pitch_deg = float(cfg.camera.default_pitch_deg)
        self.video_enabled = False        # set_video_stream(True/False)

    # ------------------------------------------------------------------ #
    # Goal installers — SIM THREAD ONLY (called via run_on_sim_thread).
    # Validation raises NotReady/LowBattery, which propagates to the caller.
    # ------------------------------------------------------------------ #

    def goal_takeoff(self, height_cm: float, flags=0) -> Goal:
        self._require_connected()
        if self.flying:
            g = Goal("takeoff")
            g.complete(False, "already flying")
            return g
        self._require_battery("takeoff")
        # Freeze the takeoff-origin frame HERE: ground point + current heading.
        self.takeoff_frame = frames.capture_takeoff_frame(
            self.pos[0], self.pos[1], self.pos[2] - self._half_z, self.yaw)
        self.drift_err[:] = 0.0  # estimate re-anchors at takeoff
        self._drift_t = 0.0      # ...and the slow wander restarts from zero
        self.flying = True
        target = self.pos.copy()
        target[2] = self.takeoff_frame.oz + height_cm / 100.0
        # Confirmed asymmetric vertical speeds: climb 1.2 m/s.
        return self._install("takeoff", target_pos=self._clamp_z(target),
                             speed_mps=float(self.cfg.velocity_levels.climb_mps))

    def goal_land(self) -> Goal:
        # Always allowed while flying — never battery-gated.
        self._require_connected()
        self._require_flying("land")
        target = self.pos.copy()
        target[2] = self._ground_z + self._half_z
        # Confirmed asymmetric vertical speeds: descend 1.0 m/s.
        return self._install("land", target_pos=target,
                             speed_mps=float(self.cfg.velocity_levels.descent_mps))

    def goal_hover(self, duration_seconds: float) -> Goal:
        self._require_connected()
        self._require_flying("hover")
        self._require_battery("hover")
        end = self._clock.now() + max(0.0, float(duration_seconds))
        # Station-keeping speed: the realistic controller needs a nonzero
        # cap to hold position against momentum/wind (crisp mode ignores it).
        return self._install("hover", target_pos=self.pos.copy(),
                             end_time=end,
                             speed_mps=speed_to_mps(self.cfg,
                                                    VelocityLevel.MEDIUM))

    def goal_move(self, direction, distance_cm: float, speed) -> Goal:
        """BODY-relative: direction is taken from the CURRENT heading."""
        self._require_connected()
        self._require_flying("move")
        self._require_battery("move")
        vec = np.array(frames.body_direction_to_world(self.yaw, direction))
        target = self._clamp_z(self.pos + vec * (float(distance_cm) / 100.0))
        return self._install("move", target_pos=target,
                             speed_mps=speed_to_mps(self.cfg, speed))

    def goal_rotate(self, angle_degrees: float) -> Goal:
        """Positive = CCW. Yaw stays unwrapped so >360° rotations work."""
        self._require_connected()
        self._require_flying("rotate")
        self._require_battery("rotate")
        return self._install(
            "rotate", target_yaw=self.yaw + math.radians(float(angle_degrees)),
            yaw_rate_rps=math.radians(self.cfg.velocity_levels.yaw_rate_dps))

    def goal_move_to(self, x: float, y: float, z: float, speed) -> Goal:
        """TAKEOFF-ORIGIN frame, cm — fixed at takeoff, ignores current yaw."""
        self._require_connected()
        self._require_flying("move_to")
        self._require_battery("move_to")
        target = np.array(frames.takeoff_cm_to_world(self.takeoff_frame,
                                                     float(x), float(y),
                                                     float(z)))
        return self._install("move_to", target_pos=self._clamp_z(target),
                             speed_mps=speed_to_mps(self.cfg, speed))

    # ------------------------------------------------------------------ #
    # Per-step execution — SIM THREAD ONLY (registry step loop)
    # ------------------------------------------------------------------ #

    def step(self, dt: float) -> None:
        if self.flying and self.battery_pct > 0.0:
            drain = self.cfg.drones.battery.drain_pct_per_min * dt / 60.0
            self.battery_pct = max(0.0, self.battery_pct - drain)
        if self.flying and self._drift_enabled:
            # Low-frequency bounded wander: drift = sum of long-period
            # sinusoids (zero phase => drift(0)=0 at takeoff). Smooth, slow,
            # |drift| <= bound by construction (amplitudes sum to the bound).
            self._drift_t += dt
            self.drift_err = (self._drift_amp
                              * np.sin(self._drift_freq * self._drift_t)
                              ).sum(axis=1)
        if self.flying:
            self._run_reflexes()  # may preempt or stop the active goal

        g = self.goal
        if g is None:
            # Idle (between/after goals): ease the body tilt back to level at
            # the rate limit — no snapping, no rocking.
            if self.flying and self._realistic and abs(self.tilt_deg) > 1e-3:
                step = self._tilt_rate * dt
                self.tilt_deg -= math.copysign(min(step, abs(self.tilt_deg)),
                                               self.tilt_deg)
                self._mirror_pose()
            return
        now = self._clock.now()
        if now >= g.deadline:
            self.goal = None
            g.complete(False, f"{g.kind} timed out (sim-time deadline)")
            return
        if g.kind == "manual":
            self._step_manual(g, dt, now)  # always dynamic, never snaps
        elif self._realistic:
            self._step_realistic(g, dt, now)
        else:
            self._step_crisp(g, dt, now)

    def _yaw_step(self, g, dt: float) -> bool:
        """Shared rate-limited yaw tracking. Returns True if the yaw moved."""
        if g.target_yaw is None or self.yaw == g.target_yaw:
            return False
        dyaw = g.target_yaw - self.yaw
        step_yaw = g.yaw_rate_rps * dt
        if abs(dyaw) <= step_yaw:
            self.yaw = g.target_yaw
        else:
            self.yaw += math.copysign(step_yaw, dyaw)
        return True

    def _mirror_pose(self) -> None:
        p.resetBasePositionAndOrientation(
            self.body_id, self.pos.tolist(),
            p.getQuaternionFromEuler(
                [0.0, -math.radians(self.tilt_deg), self.yaw]),
            physicsClientId=self._client)

    def _finish_if_done(self, g, now: float, pos_done: bool) -> None:
        yaw_done = g.target_yaw is None or self.yaw == g.target_yaw
        time_done = g.end_time is None or now >= g.end_time
        if pos_done and yaw_done and time_done:
            self.goal = None
            # NOTE: do NOT snap tilt to 0 here — that was a frame-to-frame
            # jump (rocking). The idle-relax in step() eases it to level at
            # the rate limit; a landed drone is forced level just below.
            if g.kind == "land":
                self.flying = False
                self.vel[:] = 0.0
                self.tilt_deg = 0.0  # grounded: level (no further stepping)
                self._mirror_pose()
                if self.landing_scorer is not None:
                    self.landing_scorer.record_landing(self)  # part-1 referee
            g.complete(True, f"{g.kind} complete")

    def _step_crisp(self, g, dt: float, now: float) -> None:
        """The original snap-to-target stepper (motion.realistic=False):
        exact geometry for deterministic tests."""
        moved = False
        if g.target_pos is not None:
            delta = g.target_pos - self.pos
            dist = float(np.linalg.norm(delta))
            if dist > 0.0:
                step_len = g.speed_mps * dt
                if dist <= step_len:
                    self.pos = g.target_pos.copy()
                elif step_len > 0.0:
                    self.pos = self.pos + delta * (step_len / dist)
                moved = True
        moved |= self._yaw_step(g, dt)
        if moved:
            self._mirror_pose()
        pos_done = g.target_pos is None or bool(
            np.all(self.pos == g.target_pos))
        self._finish_if_done(g, now, pos_done)

    def _step_realistic(self, g, dt: float, now: float) -> None:
        """Behavioural motion: accel/decel ramps, tilt-to-translate, momentum
        with mild overshoot+settle, command latency, asymmetric climb/descent,
        optional gentle wind. Honest BEHAVIOUR — NOT firmware dynamics (the
        Hula has no SITL to replicate)."""
        if now < g.exec_after:
            return  # command latency: the airframe has not reacted yet
        # Desired velocity: cruise far out, sqrt-braking near the target,
        # slightly under-braked (overshoot_frac) => momentum carries it past,
        # then the same law settles it back onto the waypoint.
        vdes = np.zeros(3)
        if g.target_pos is not None:
            err = g.target_pos - self.pos
            brake = 1.0 + self._overshoot
            eh = err.copy()
            eh[2] = 0.0
            dh = float(np.linalg.norm(eh))
            # Position deadband: within arrive_tol command ZERO velocity so
            # the drone decelerates to a dead stop and holds — no limit-cycle
            # jitter, a rock-steady hover (and the tilt then eases to level).
            if dh > self._arrive_tol and g.speed_mps > 0.0:
                v_h = min(g.speed_mps,
                          math.sqrt(2.0 * self._accel * dh) * brake)
                vdes[0:2] = eh[0:2] / dh * v_h
            ez = float(err[2])
            if abs(ez) > self._arrive_tol:
                cap_v = (float(self.cfg.velocity_levels.climb_mps) if ez > 0
                         else float(self.cfg.velocity_levels.descent_mps))
                vdes[2] = math.copysign(
                    min(cap_v, math.sqrt(2.0 * self._accel * abs(ez)) * brake),
                    ez)
        moved = self._apply_velocity_dynamics(vdes, dt)
        moved |= self._yaw_step(g, dt)
        if moved or abs(self.tilt_deg) > 0.01:
            self._mirror_pose()
        pos_done = g.target_pos is None or (
            float(np.linalg.norm(g.target_pos - self.pos)) <= self._arrive_tol
            and float(np.linalg.norm(self.vel)) <= self._arrive_speed)
        self._finish_if_done(g, now, pos_done)

    def _apply_velocity_dynamics(self, vdes, dt: float) -> bool:
        """Shared ramp/tilt/wind/integration for target- AND stick-driven
        flight. Returns True if the position changed."""
        # Acceleration-limited ramp toward the desired velocity (no steps).
        dv = vdes - self.vel
        dvn = float(np.linalg.norm(dv))
        max_dv = self._accel * dt
        applied = dv if dvn <= max_dv else dv * (max_dv / dvn)
        self.vel = self.vel + applied
        # Tilt-to-translate: body pitch tracks the applied horizontal
        # acceleration along the nose (negative = nose down = accelerating),
        # levels at cruise, counter-tilts braking — but RATE-LIMITED so it
        # eases rather than snapping/rocking frame-to-frame.
        acc = applied / dt if dt > 0 else applied * 0.0
        a_fwd = float(acc[0] * math.cos(self.yaw)
                      + acc[1] * math.sin(self.yaw))
        # Deadband: only tilt for SUSTAINED translation accel. Station-keeping
        # micro-corrections (|a_fwd| << accel) leave the body level rather than
        # holding a residual tilt — a hovering drone sits flat, no wobble.
        if abs(a_fwd) < 0.15 * self._accel:
            tilt_target = 0.0
        else:
            tilt_target = max(-self._max_tilt,
                              min(self._max_tilt,
                                  -self._max_tilt * a_fwd / self._accel))
        max_step = self._tilt_rate * dt
        dtilt = tilt_target - self.tilt_deg
        self.tilt_deg += (dtilt if abs(dtilt) <= max_step
                          else math.copysign(max_step, dtilt))
        if self._wind_on and self._wind_mps > 0.0:
            # Gentle OU wind nudging the TRUE position while airborne,
            # calibrated so the stationary wind speed ~= wind_mps.
            tau_w = 5.0
            self._wind_vec += (-self._wind_vec * (dt / tau_w)
                               + self._wind_rng.normal(
                                   0.0, self._wind_mps
                                   * math.sqrt(2.0 / tau_w) * math.sqrt(dt),
                                   2))
            self.pos[0:2] += self._wind_vec * dt
        moved = bool(np.linalg.norm(self.vel) > 1e-6)
        if moved:
            self.pos = self.pos + self.vel * dt
        # never integrate through the floor
        floor = self._ground_z + self._half_z
        if self.pos[2] < floor:
            self.pos[2] = floor
            if self.vel[2] < 0.0:
                self.vel[2] = 0.0
        return moved

    def manual_frame(self, forward, right, up, rotate) -> bool:
        """One ~20 Hz joystick frame (send_manual_control). SIM THREAD ONLY.
        Returns False (never raises) when the drone cannot accept manual
        input. Installs/refreshes a persistent 'manual' goal through the SAME
        committing executor, so blocking commands and manual control never
        fight — the last command simply wins (preempting the other)."""
        if not (self.connected and self.flying):
            return False
        thr = self.cfg.drones.battery.low_threshold_pct
        if self.battery_pct < thr:
            return False
        if self.goal is None or self.goal.kind != "manual":
            g = self._install("manual")     # preempts any blocking goal
            g.deadline = float("inf")       # continuous: lives until replaced
        sticks = [max(-1.0, min(1.0, float(v)))
                  for v in (forward, right, up, rotate)]
        if (abs(sticks[0]) > 1e-3 or abs(sticks[1]) > 1e-3
                or abs(sticks[2]) > 1e-3):
            self._hold_anchor = None        # actively flying: drop the hold
        self.manual_stick[:] = sticks
        self._stick_fresh_until = self._clock.now() + self._manual_timeout
        return True

    def _step_manual(self, g, dt: float, now: float) -> None:
        """Stick-driven flight: inputs map to commanded velocity through the
        speed band and the SAME accel/tilt limits as everything else. The
        sim only EXECUTES the inputs — there is no PID in here; closing a
        loop on what the camera sees is mission code."""
        if now < g.exec_after:
            return
        if now > self._stick_fresh_until:
            f = r = u = rot = 0.0           # stale control loop: fail safe
        else:
            f, r, u, rot = (float(v) for v in self.manual_stick)
        # Barrier clamping: a stick INTO a tripped (boolean) barrier flag is
        # zeroed — braking distance keeps the hull clear of contact.
        if abs(f) > 1e-3 or abs(r) > 1e-3 or u < -1e-3:
            flags = self.sense_obstacles()
            if f > 0 and flags.forward:
                f = 0.0
            if f < 0 and flags.back:
                f = 0.0
            if r > 0 and flags.right:
                r = 0.0
            if r < 0 and flags.left:
                r = 0.0
            if u < 0 and flags.down:
                u = 0.0
        vdes = np.zeros(3)
        cap_h = float(self.cfg.velocity_levels.TURBO)  # the 0.5-1.0 band cap
        if abs(f) > 1e-3 or abs(r) > 1e-3 or abs(u) > 1e-3:
            # Body-relative: +forward = nose, +right = body right (-y FLU).
            bx, by = f * cap_h, -r * cap_h
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            vdes[0] = c * bx - s * by
            vdes[1] = s * bx + c * by
            if abs(u) > 1e-3:
                cap_v = (float(self.cfg.velocity_levels.climb_mps) if u > 0
                         else float(self.cfg.velocity_levels.descent_mps))
                vdes[2] = u * cap_v
        else:
            # Zero stick: coast to a stop per the motion model, then
            # STATION-KEEP at the stopping point (holds against wind).
            if self._hold_anchor is None:
                if float(np.linalg.norm(self.vel)) <= self._arrive_speed:
                    self._hold_anchor = self.pos.copy()
            if self._hold_anchor is not None:
                err = self._hold_anchor - self.pos
                brake = 1.0 + self._overshoot
                eh = err.copy()
                eh[2] = 0.0
                dh = float(np.linalg.norm(eh))
                if dh > 1e-9:
                    hold_cap = float(self.cfg.velocity_levels.MEDIUM)
                    v_h = min(hold_cap,
                              math.sqrt(2.0 * self._accel * dh) * brake)
                    vdes[0:2] = eh[0:2] / dh * v_h
                ez = float(err[2])
                if abs(ez) > 1e-9:
                    cap_v = (float(self.cfg.velocity_levels.climb_mps)
                             if ez > 0
                             else float(self.cfg.velocity_levels.descent_mps))
                    vdes[2] = math.copysign(
                        min(cap_v,
                            math.sqrt(2.0 * self._accel * abs(ez)) * brake),
                        ez)
        moved = self._apply_velocity_dynamics(vdes, dt)
        if abs(rot) > 1e-3:
            # Continuous yaw rate; positive = CCW, like rotate().
            self.yaw += rot * math.radians(
                self.cfg.velocity_levels.yaw_rate_dps) * dt
            moved = True
        if moved or abs(self.tilt_deg) > 0.01:
            self._mirror_pose()
        # No completion: the manual goal persists until another command
        # (blocking or a new mode) preempts it via the executor.

    # ------------------------------------------------------------------ #
    # Telemetry reads — SIM THREAD ONLY (called via run_on_sim_thread)
    # ------------------------------------------------------------------ #

    def _frame_or_provisional(self) -> frames.TakeoffFrame:
        """Before the first takeoff there is no frozen frame yet; report in a
        provisional frame anchored at the current pose (reads ~(0,0,hz))."""
        if self.takeoff_frame is not None:
            return self.takeoff_frame
        return frames.capture_takeoff_frame(
            self.pos[0], self.pos[1], self.pos[2] - self._half_z, self.yaw)

    def telemetry_position(self) -> Vector3:
        """Drifting onboard estimate, TAKEOFF-ORIGIN frame, cm (§4.3)."""
        fr = self._frame_or_provisional()
        est = self.pos + self.drift_err
        x, y, z = frames.world_to_takeoff_cm(fr, est[0], est[1], est[2])
        return Vector3(x, y, z)

    def telemetry_orientation(self) -> Orientation:
        """Degrees. Yaw = CCW from the takeoff heading, normalised [0, 360).
        Pitch/roll are 0 in the kinematic sim. (Reference to verify on the
        real drone later — relative-to-takeoff matches IMU re-zeroing.)"""
        fr = self._frame_or_provisional()
        yaw_deg = math.degrees(self.yaw - fr.psi0) % 360.0
        # pitch = the live tilt-to-translate body angle (negative = nose
        # down, accelerating); always 0.0 in crisp-motion mode.
        return Orientation(yaw=yaw_deg, pitch=round(self.tilt_deg, 2),
                           roll=0.0)

    def telemetry_altitude(self) -> float:
        """Downward ToF, cm: real ray-cast distance to whatever is below
        (floor or obstacle top), from TRUE height — never from UWB."""
        return sensors.altitude_cm(self._client, self.cfg, self)

    def telemetry_state(self) -> DroneState:
        return DroneState(
            connected=self.connected,
            position=self.telemetry_position(),
            orientation=self.telemetry_orientation(),
            altitude=self.telemetry_altitude(),
            battery=int(round(self.battery_pct)),
            obstacles=Obstacles(),   # real barrier sensors land in Phase 4
            flying=self.flying,
        )

    def arena_position(self):
        """TRUE (north, east) metres — the UWB drop-in's ground truth."""
        return frames.world_to_arena(self.cfg, self.pos[0], self.pos[1])

    # ------------------------------------------------------------------ #
    # Barrier sensors + reflexes (§4.4) — SIM THREAD ONLY
    # ------------------------------------------------------------------ #

    def sense_obstacles(self) -> Obstacles:
        """Fresh 5-direction barrier read (booleans only — coarse by design)."""
        return sensors.barrier_flags(self._client, self.cfg, self)

    def status_bitmask(self) -> int:
        """Barrier bits: 0=forward 1=back 2=left 3=right 4=down."""
        return sensors.bitmask(self.sense_obstacles())

    def set_barrier_mode(self, enabled: bool) -> None:
        self.barrier_mode = bool(enabled)

    def set_avoidance_rule(self, direction, distance_cm, barrier_mask) -> None:
        """Arm the conditional step reflex (distance_cm <= 0 disarms)."""
        if distance_cm and float(distance_cm) > 0:
            self.avoidance_rule = (Direction(int(direction)),
                                   float(distance_cm) / 100.0,
                                   int(barrier_mask))
        else:
            self.avoidance_rule = None

    # ------------------------------------------------------------------ #
    # Camera (§4.5) — SIM THREAD ONLY
    # ------------------------------------------------------------------ #

    def set_camera_pitch(self, mode, angle: float) -> float:
        """Tilt the main camera. Sim models pitch 0 (forward) .. 90 (down);
        tilting above the horizon clamps to 0. Returns the new pitch."""
        m = CameraPitchMode(int(mode))
        a = float(angle)
        if m == CameraPitchMode.DOWN_ABSOLUTE:
            pitch = a
        elif m == CameraPitchMode.UP_ABSOLUTE:
            pitch = -a  # angle 0 = straight ahead; above-horizon clamps to 0
        elif m == CameraPitchMode.DOWN_RELATIVE:
            pitch = self.camera_pitch_deg + a
        elif m == CameraPitchMode.UP_RELATIVE:
            pitch = self.camera_pitch_deg - a
        elif m == CameraPitchMode.CALIBRATE:
            pitch = float(self.cfg.camera.default_pitch_deg)
        else:
            raise PyhulaxError(f"unsupported CameraPitchMode: {mode!r}")
        self.camera_pitch_deg = min(max(pitch, 0.0), 90.0)
        return self.camera_pitch_deg

    def _run_reflexes(self) -> None:
        """Both reflexes route through the SAME committing executor — no
        separate control path. The step reflex preempts via _install (logged);
        barrier mode fails the active goal ('stopped short')."""
        g = self.goal
        barrier_watch = (self.barrier_mode and g is not None
                         and g.target_pos is not None
                         and g.kind not in ("land", "takeoff", "avoid_step"))
        if self.avoidance_rule is None and not barrier_watch:
            return
        flags = self.sense_obstacles()
        if not flags.any:
            return

        # 1) Explicit conditional reflex: fires only on an actual detection
        #    in the armed mask; one step at a time (no re-fire mid-step).
        if self.avoidance_rule is not None and (g is None
                                                or g.kind != "avoid_step"):
            direction, dist_m, mask = self.avoidance_rule
            if _mask_tripped(flags, mask):
                self._log.warning(
                    "avoidance reflex tripped (%s) -> stepping %s %.0f cm",
                    _flag_names(flags), direction.name, dist_m * 100)
                vec = np.array(frames.body_direction_to_world(self.yaw,
                                                              direction))
                self._install(
                    "avoid_step",
                    target_pos=self._clamp_z(self.pos + vec * dist_m),
                    speed_mps=speed_to_mps(self.cfg, VelocityLevel.ZOOM))
                return

        # 2) Firmware auto-avoid: stop short when a tripped sensor lies in
        #    the direction of travel (landing/takeoff are never blocked).
        g = self.goal
        if (self.barrier_mode and g is not None and g.target_pos is not None
                and g.kind not in ("land", "takeoff", "avoid_step")):
            delta = g.target_pos - self.pos
            dist = float(np.linalg.norm(delta))
            if dist > 1e-9 and self._motion_blocked(delta / dist, flags):
                self.goal = None
                self._log.warning("barrier mode: %s stopped short (%s)",
                                  g.kind, _flag_names(flags))
                g.complete(False,
                           f"{g.kind} stopped short by barrier (auto-avoid)")

    def _motion_blocked(self, unit_world, flags: Obstacles) -> bool:
        """Is a tripped sensor in the direction of travel? (world -> body)"""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        bx = c * unit_world[0] + s * unit_world[1]    # along the nose
        by = -s * unit_world[0] + c * unit_world[1]   # toward body left
        bz = unit_world[2]
        return bool((bx > 0.3 and flags.forward) or (bx < -0.3 and flags.back)
                    or (by > 0.3 and flags.left) or (by < -0.3 and flags.right)
                    or (bz < -0.3 and flags.down))

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #

    def _install(self, kind, target_pos=None, target_yaw=None, speed_mps=0.0,
                 yaw_rate_rps=0.0, end_time=None) -> Goal:
        now = self._clock.now()
        est = 0.0
        if target_pos is not None and speed_mps > 0.0:
            est += float(np.linalg.norm(target_pos - self.pos)) / speed_mps
        if target_yaw is not None and yaw_rate_rps > 0.0:
            est += abs(target_yaw - self.yaw) / yaw_rate_rps
        if end_time is not None:
            est += max(0.0, end_time - now)
        if self._realistic:
            # ramps + latency + settle take real time beyond distance/speed
            est += self._latency + 2.0
            if speed_mps > 0.0:
                est += 2.0 * speed_mps / self._accel
        g = Goal(kind, target_pos=target_pos, target_yaw=target_yaw,
                 speed_mps=speed_mps, yaw_rate_rps=yaw_rate_rps,
                 end_time=end_time,
                 deadline=now + est * _TIMEOUT_MARGIN + _TIMEOUT_BASE_S,
                 exec_after=now + (self._latency if self._realistic else 0.0))
        if self.monitor is not None and kind != "avoid_step":
            self.monitor.record_command(self.index, kind)  # reflexes exempt
        if self.goal is not None:
            old = self.goal
            self._log.warning("%s preempted by %s", old.kind, kind)
            if self.monitor is not None:
                self.monitor.record_preempt(self.index, old.kind, kind)
            old.complete(False, f"preempted by {kind}")
        self.goal = g
        return g

    def _clamp_z(self, target: np.ndarray) -> np.ndarray:
        """Keep targets between the floor and the ceiling."""
        lo = self._ground_z + self._half_z
        hi = float(self.cfg.arena.origin[2]) + self.cfg.arena.height_m \
            - self._half_z
        target[2] = min(max(target[2], lo), hi)
        return target

    def _require_connected(self) -> None:
        if not self.connected:
            raise NotReady(f"drone {self.spec.ip} is not connected")

    def _require_flying(self, what: str) -> None:
        if not self.flying:
            raise NotReady(f"{what} requires flying — call takeoff() first")

    def _require_battery(self, what: str) -> None:
        thr = self.cfg.drones.battery.low_threshold_pct
        if self.battery_pct < thr:
            raise LowBattery(f"battery {self.battery_pct:.0f}% < {thr}% — "
                             f"{what} refused (land() is still allowed)")
