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

from pyhulax.core import CommandResult, VelocityLevel
from pyhulax.exceptions import LowBattery, NotReady, PyhulaxError

from . import frames
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


class Goal:
    """One in-flight command target. done/result are consumed by the waiter."""

    __slots__ = ("kind", "target_pos", "target_yaw", "speed_mps",
                 "yaw_rate_rps", "end_time", "deadline", "done", "result")

    def __init__(self, kind, target_pos=None, target_yaw=None, speed_mps=0.0,
                 yaw_rate_rps=0.0, end_time=None, deadline=float("inf")):
        self.kind = kind
        self.target_pos = target_pos      # np.ndarray(3) world m, or None
        self.target_yaw = target_yaw      # world rad (unwrapped), or None
        self.speed_mps = speed_mps
        self.yaw_rate_rps = yaw_rate_rps
        self.end_time = end_time          # sim time the goal must last until
        self.deadline = deadline          # sim time after which it fails
        self.done = threading.Event()
        self.result = None

    def complete(self, success: bool, message: str) -> None:
        self.result = CommandResult(success, message)
        self.done.set()


class SimDrone:
    """State + kinematic executor for one drone (sim-thread only mutation)."""

    def __init__(self, cfg, index, spec, body_id, client, clock,
                 start_pos, start_yaw):
        self.cfg = cfg
        self.index = index
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
        self.flying = True
        target = self.pos.copy()
        target[2] = self.takeoff_frame.oz + height_cm / 100.0
        return self._install("takeoff", target_pos=self._clamp_z(target),
                             speed_mps=speed_to_mps(self.cfg, VelocityLevel.ZOOM))

    def goal_land(self) -> Goal:
        # Always allowed while flying — never battery-gated.
        self._require_connected()
        self._require_flying("land")
        target = self.pos.copy()
        target[2] = self._ground_z + self._half_z
        return self._install("land", target_pos=target,
                             speed_mps=speed_to_mps(self.cfg, VelocityLevel.ZOOM))

    def goal_hover(self, duration_seconds: float) -> Goal:
        self._require_connected()
        self._require_flying("hover")
        self._require_battery("hover")
        end = self._clock.now() + max(0.0, float(duration_seconds))
        return self._install("hover", target_pos=self.pos.copy(), end_time=end)

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

        g = self.goal
        if g is None:
            return
        now = self._clock.now()
        if now >= g.deadline:
            self.goal = None
            g.complete(False, f"{g.kind} timed out (sim-time deadline)")
            return

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
        if g.target_yaw is not None and self.yaw != g.target_yaw:
            dyaw = g.target_yaw - self.yaw
            step_yaw = g.yaw_rate_rps * dt
            if abs(dyaw) <= step_yaw:
                self.yaw = g.target_yaw
            else:
                self.yaw += math.copysign(step_yaw, dyaw)
            moved = True
        if moved:
            p.resetBasePositionAndOrientation(
                self.body_id, self.pos.tolist(),
                p.getQuaternionFromEuler([0.0, 0.0, self.yaw]),
                physicsClientId=self._client)

        pos_done = g.target_pos is None or bool(np.all(self.pos == g.target_pos))
        yaw_done = g.target_yaw is None or self.yaw == g.target_yaw
        time_done = g.end_time is None or now >= g.end_time
        if pos_done and yaw_done and time_done:
            self.goal = None
            if g.kind == "land":
                self.flying = False
            g.complete(True, f"{g.kind} complete")

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
        g = Goal(kind, target_pos=target_pos, target_yaw=target_yaw,
                 speed_mps=speed_mps, yaw_rate_rps=yaw_rate_rps,
                 end_time=end_time,
                 deadline=now + est * _TIMEOUT_MARGIN + _TIMEOUT_BASE_S)
        if self.goal is not None:
            old = self.goal
            self._log.warning("%s preempted by %s", old.kind, kind)
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
