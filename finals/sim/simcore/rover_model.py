"""Rover agents: convoy routing / random patrol / mixed (auto + evasive) +
unique top ArUco markers.

Simulator INTERNAL — mission code must never import simcore.

NOT mission logic: rovers are world actors the mission has to find. Motion is
advanced on the SIM THREAD only (step(dt) from the registry loop).

MOTION MODES (config rovers.motion):
- "convoy" (DEFAULT): staggered SW entry, shared trunk, per-rover authored
  branch, loiter — smooth, rate-limited turning (Phase 28). No reacting to drones.
- "patrol" (back-compat): seeded waypoint_random wander. speed_mps = 0 parks them.
- "mixed": 3 AUTONOMOUS rovers (auto_motion, smooth) carrying ids [20,21,22],
  plus 2 EVASIVE rovers carrying a separate id block [30,31]. The evasive pair
  models the human-teleoperated opponents: flee a nearby drone, seek crate
  cover, and juke (intentionally UN-smooth). One evasive rover can be
  human-driven via the SSH-safe teleop hook (set_teleop / scripts.rover_teleop).
"""

import math

import numpy as np
import pybullet as p

from . import arena, frames

_WAYPOINT_TRIES = 50
_OBSTACLE_CLEAR_M = 0.3   # keep the path this far from obstacle footprints
_SEGMENT_STEP_M = 0.2     # sampling pitch of the straight-path clearance check

_MOTION_MODES = ("convoy", "patrol", "mixed")
_PERSONALITIES = ("convoy", "patrol", "evasive")
_LOITER_MODES = ("loop", "hold")
_HOLD_BRAKE_M = 0.6       # loiter=hold: ease to a stop over this final distance

_ROVER_RADIUS_M = 0.2     # footprint radius for bounds / crate avoidance
_COVER_LOOKAHEAD_M = 1.0  # how far ahead the cover-seeking score peeks


# --------------------------------------------------------------------------- #
# Per-mode resolution (shared by world.py + registry.py)
# --------------------------------------------------------------------------- #

def resolved_marker_ids(cfg) -> list:
    """The per-rover ArUco marker ids for the configured motion mode. Mixed
    mode uses the two id blocks (autonomous + evasive) in order; otherwise
    rovers.marker_ids."""
    r = cfg.rovers
    if r.motion == "mixed":
        return [int(i) for i in (list(r.mixed.auto_ids)
                                 + list(r.mixed.evasive_ids))]
    return [int(i) for i in r.marker_ids]


def personality_for(cfg, index: int) -> str:
    """Behaviour for rover `index`: convoy | patrol | evasive. Mixed = the
    autonomous motion for the first len(auto_ids), then evasive."""
    r = cfg.rovers
    if r.motion == "mixed":
        n_auto = len(r.mixed.auto_ids)
        return r.mixed.auto_motion if index < n_auto else "evasive"
    return r.motion


def keys_to_rover_drive(keys, speed: float = 1.0):
    """Map held keys -> (v_north, v_east) for the teleop hook. Arena frame:
    W=+north, S=-north, A=-east, D=+east; opposing keys cancel; magnitude is
    clamped to `speed`. PURE function — headless/SSH-friendly, no window."""
    ks = {str(k).lower() for k in keys}
    vn = (1.0 if "w" in ks else 0.0) - (1.0 if "s" in ks else 0.0)
    ve = (1.0 if "d" in ks else 0.0) - (1.0 if "a" in ks else 0.0)
    norm = math.hypot(vn, ve)
    if norm > 1.0:
        vn, ve = vn / norm, ve / norm
    return (vn * speed, ve * speed)


def _point_segment_dist(px, py, ax, ay, bx, by) -> float:
    """Distance from point (px,py) to segment (a)->(b)."""
    abx, aby = bx - ax, by - ay
    denom = abx * abx + aby * aby
    if denom <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * abx + (py - ay) * aby) / denom
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * abx), py - (ay + t * aby))


def convoy_route(cfg, rover_index: int):
    """The full authored route for one rover, arena (north, east):
    entrance -> shared trunk (to split_index) -> its own branch.
    Pure config — used by the rover itself and by the route-clearance test."""
    cv = cfg.rovers.convoy
    if not (0 <= cv.split_index < len(cv.trunk)):
        raise ValueError(f"rovers.convoy.split_index {cv.split_index} out of "
                         f"range for a {len(cv.trunk)}-waypoint trunk")
    if rover_index >= len(cv.branches):
        raise ValueError(f"rovers.convoy.branches has {len(cv.branches)} "
                         f"entries — need one per rover (index {rover_index})")
    shared = [tuple(w) for w in cv.trunk[:cv.split_index + 1]]
    branch = [tuple(w) for w in cv.branches[rover_index]]
    if not branch:
        raise ValueError(f"rovers.convoy.branches[{rover_index}] is empty")
    return [tuple(cfg.scenario.entrance)] + shared + branch


class SimRover:
    """State + motion for one rover (sim-thread only mutation)."""

    def __init__(self, cfg, index, marker_id, body_id, client, clock,
                 start_pose, obstacles):
        self.cfg = cfg
        self.index = index
        self.marker_id = int(marker_id)   # this rover's UNIQUE ArUco id
        self.body_id = body_id
        self._client = client
        self._clock = clock
        self._half_z = float(cfg.bodies.rover_half_extents_m[2])
        pat = cfg.rovers.patrol
        if pat.mode != "waypoint_random":
            raise ValueError(f"unsupported rovers.patrol.mode: {pat.mode!r}")
        self._speed = float(pat.speed_mps)
        self._pause_s = float(pat.waypoint_pause_s)
        self._bounds_n = tuple(pat.bounds_north)
        self._bounds_e = tuple(pat.bounds_east)
        self._obstacles = obstacles       # layout.obstacles (arena frame)
        self._ground_obstacles = [o for o in obstacles if o.z0_m == 0.0]
        self._rng = np.random.default_rng([cfg.meta.seed, 3000 + index])

        wx, wy, _ = frames.arena_to_world(cfg, start_pose.north,
                                          start_pose.east, 0.0)
        self.pos = np.array([wx, wy, self._half_z])
        self.yaw = frames.heading_to_world_yaw_rad(cfg, start_pose.heading_deg)
        self._target_w = None             # world (x, y) of current waypoint
        self._pause_until = clock.now() + self._pause_s
        # Scenario staging (simcore/scenario.py): in DEPLOY the rovers wait
        # OFF-MAP (inert, invisible, not targets) until the ambush begins.
        self.in_arena = True
        self._spawn_pos = self.pos.copy()
        self._spawn_yaw = self.yaw

        self._motion = cfg.rovers.motion
        if self._motion not in _MOTION_MODES:
            raise ValueError(f"unknown rovers.motion: {self._motion!r}")
        self._personality = personality_for(cfg, index)
        if self._personality not in _PERSONALITIES:
            raise ValueError(f"unknown rover personality: "
                             f"{self._personality!r}")
        # Only convoy-personality rovers enter from the SW entrance; patrol /
        # evasive rovers enter at their layout spawn.
        self.enters_from_entrance = self._personality == "convoy"
        self._drones = []                 # bound by the registry (evasive flee)
        self._teleop = None               # (v_north, v_east) m/s, or None
        self._teleop_until = 0.0

        # Moving gimbal: the marker's facing yaw sweeps from a seeded per-rover
        # phase; a drone decodes the marker only inside the readable cone.
        gb = cfg.rovers.gimbal
        self._gimbal_on = bool(gb.enabled)
        self._gimbal_sweep = math.radians(float(gb.sweep_deg_per_s))
        self._gimbal_half = math.radians(float(gb.readable_halfangle_deg))
        self._gimbal_phase = float(np.random.default_rng(
            [cfg.meta.seed, 5000 + index]).uniform(0.0, 2.0 * math.pi))
        self._marker_link = None          # set by the registry after world build
        self._marker_tex = None           # the ArUco texture id for the marker

        if self._personality == "convoy":
            cv = cfg.rovers.convoy
            if cv.loiter not in _LOITER_MODES:
                raise ValueError(f"unknown rovers.convoy.loiter: "
                                 f"{cv.loiter!r}")
            self._convoy_speed = float(cv.speed_mps)
            self._turn_rate = math.radians(float(cv.turn_rate_dps))
            waypoints_ne = convoy_route(cfg, index)[1:]  # after the entrance
            self._route_w = [
                np.array(frames.arena_to_world(cfg, n, e, 0.0)[:2])
                for n, e in waypoints_ne]
            self._loop_from = cv.split_index + 1  # first branch wp index
            self._loiter = cv.loiter
            self._entry_time = None   # sim time this rover enters (staggered)
            self._wp_i = 0            # current route waypoint (None = holding)
        elif self._personality == "evasive":
            ev = cfg.rovers.mixed.evasive
            self._ev_speed = float(ev.speed_mps)
            self._flee_radius = float(ev.flee_radius_m)
            self._flee_gain = float(ev.flee_gain)
            self._cover_bias = float(ev.cover_bias)
            self._juke_prob = float(ev.juke_prob)
            self._ev_turn_rate = math.radians(float(ev.turn_rate_dps))
            self._ev_decision_period = float(ev.decision_period_s)
            self._ev_teleop_timeout = float(ev.teleop_timeout_s)
            self._next_decision = 0.0
            self._juke = False
            self._juke_angle = 0.0
            self._ev_target = None    # arena (n, e) roam target

    def arena_position(self):
        """TRUE (north, east) metres."""
        return frames.world_to_arena(self.cfg, self.pos[0], self.pos[1])

    def bind_drones(self, drones) -> None:
        """Give the rover a reference to the drone list (read on the sim
        thread) so an evasive rover can find the nearest pursuer. No-op for
        the other personalities."""
        self._drones = drones

    def bind_marker(self, link_index, texture_id) -> None:
        """Record this rover's marker link + ArUco texture (the registry sets
        these after world build) so the gimbal gating can show/hide it."""
        self._marker_link = link_index
        self._marker_tex = texture_id

    # ------------------------------------------------------------------ #
    # Moving-gimbal marker model (read on the sim thread)
    # ------------------------------------------------------------------ #

    def marker_yaw(self, now: float) -> float:
        """World-frame yaw (rad) the marker is FACING at sim time `now`:
        a seeded per-rover phase swept at gimbal.sweep_deg_per_s."""
        return (self._gimbal_phase + self._gimbal_sweep * now) % (2.0 * math.pi)

    def marker_readable_by(self, now: float, drone_world_xy) -> bool:
        """Can a drone at `drone_world_xy` (world x, y) DECODE this marker now?
        True when the gimbal facing is within ±readable_halfangle of the
        horizontal bearing from the rover to the drone. With the gimbal off,
        always True (the marker is a plain top fiducial)."""
        if not self._gimbal_on:
            return True
        dx = float(drone_world_xy[0]) - float(self.pos[0])
        dy = float(drone_world_xy[1]) - float(self.pos[1])
        if math.hypot(dx, dy) < 0.05:     # directly overhead: facing undefined
            return False                  # must be offset toward the facing
        bearing = math.atan2(dy, dx)
        diff = (self.marker_yaw(now) - bearing + math.pi) % (2.0 * math.pi) \
            - math.pi
        return abs(diff) <= self._gimbal_half

    # ------------------------------------------------------------------ #
    # Scenario staging — SIM THREAD ONLY (called by simcore/scenario.py)
    # ------------------------------------------------------------------ #

    def park_offmap(self, north: float, east: float) -> None:
        """Stage the rover OUTSIDE the arena (DEPLOY: inert, not a target)."""
        self.in_arena = False
        wx, wy, _ = frames.arena_to_world(self.cfg, north, east, 0.0)
        self.pos = np.array([wx, wy, self._half_z])
        self._target_w = None
        self._mirror()

    def activate(self, now: float) -> None:
        """AMBUSH begins (called by the scenario). Convoy: arm the staggered
        entry. Patrol / evasive: enter the arena at the layout spawn."""
        if self._personality == "convoy":
            self._entry_time = (now + self.index
                                * float(self.cfg.rovers.convoy.entry_stagger_s))
        else:
            self.enter_arena()
            if self._personality == "evasive":
                self._next_decision = now  # decide a heading immediately

    def enter_arena(self) -> None:
        """Teleport to the layout spawn (patrol / evasive AMBUSH entry)."""
        self.in_arena = True
        self.pos = self._spawn_pos.copy()
        self.yaw = self._spawn_yaw
        self._target_w = None
        self._pause_until = self._clock.now() + self._pause_s
        self._mirror()

    def _mirror(self) -> None:
        p.resetBasePositionAndOrientation(
            self.body_id, self.pos.tolist(),
            p.getQuaternionFromEuler([0.0, 0.0, self.yaw]),
            physicsClientId=self._client)

    # ------------------------------------------------------------------ #
    # Per-step motion — SIM THREAD ONLY
    # ------------------------------------------------------------------ #

    def step(self, dt: float) -> None:
        if self._personality == "convoy":
            self._step_convoy(dt)
            return
        if self._personality == "evasive":
            self._step_evasive(dt)
            return
        # patrol
        if not self.in_arena:
            return  # staged off-map (DEPLOY): inert by definition
        if self._speed <= 0.0:
            return  # parked (e.g. test configs)
        now = self._clock.now()
        if self._target_w is None:
            if now < self._pause_until:
                return
            self._target_w = self._pick_waypoint(now)
            return

        delta = self._target_w - self.pos[:2]
        dist = float(np.hypot(delta[0], delta[1]))
        step_len = self._speed * dt
        if dist <= step_len:
            self.pos[:2] = self._target_w
            self._target_w = None
            self._pause_until = now + self._pause_s
        else:
            self.pos[:2] += delta * (step_len / dist)
            self.yaw = math.atan2(delta[1], delta[0])  # face travel direction
        p.resetBasePositionAndOrientation(
            self.body_id, self.pos.tolist(),
            p.getQuaternionFromEuler([0.0, 0.0, self.yaw]),
            physicsClientId=self._client)

    # ------------------------------------------------------------------ #
    # Convoy driving (fixed route, deterministic, no RNG)
    # ------------------------------------------------------------------ #

    def _step_convoy(self, dt: float) -> None:
        now = self._clock.now()
        if not self.in_arena:
            if self._entry_time is None or now < self._entry_time:
                return  # not armed / waiting its staggered slot
            # Enter the arena AT the entrance (never teleport mid-arena).
            en, ee = self.cfg.scenario.entrance
            wx, wy, _ = frames.arena_to_world(self.cfg, float(en), float(ee),
                                              0.0)
            self.in_arena = True
            self.pos = np.array([wx, wy, self._half_z])
            first = self._route_w[0]
            self.yaw = math.atan2(first[1] - wy, first[0] - wx)
            self._wp_i = 0
            self._mirror()
            return
        if self._wp_i is None:
            return  # loiter=hold: eased to a gentle stop at the branch end
        target = self._route_w[self._wp_i]
        self._target_w = target
        delta = target - self.pos[:2]
        dist = float(np.hypot(delta[0], delta[1]))
        # loiter=hold: ease to a gentle stop over the final approach segment
        # (no abrupt halt) once on the last waypoint.
        speed = self._convoy_speed
        last = self._wp_i == len(self._route_w) - 1
        if last and self._loiter == "hold":
            speed = max(0.05, min(speed, speed * dist / _HOLD_BRAKE_M))
        step_len = speed * dt
        if dist <= step_len:
            self.pos[:2] = target
            nxt = self._wp_i + 1
            if nxt >= len(self._route_w):
                if self._loiter == "loop":
                    self._wp_i = self._loop_from  # seamlessly cycle the branch
                else:
                    self._wp_i = None             # parked at the end
                    self._target_w = None
            else:
                self._wp_i = nxt
        elif step_len > 0.0:
            self.pos[:2] += delta * (step_len / dist)
        # Rate-limited heading: ease the body toward the travel direction so
        # corners and the loop seam are smooth turns, never an instant spin.
        self._turn_toward(math.atan2(delta[1], delta[0]), dt, self._turn_rate)
        self._mirror()

    def _turn_toward(self, target_yaw: float, dt: float, rate: float) -> None:
        d = (target_yaw - self.yaw + math.pi) % (2 * math.pi) - math.pi
        step = rate * dt
        self.yaw += d if abs(d) <= step else math.copysign(step, d)

    # ------------------------------------------------------------------ #
    # Evasive driving (adversarial: flee + cover-seek + juke; un-smooth)
    # ------------------------------------------------------------------ #

    def set_teleop(self, v_north: float, v_east: float) -> None:
        """Headless teleop drive for this (evasive) rover — arena-frame m/s,
        fresh for teleop_timeout_s. Set by the SSH stdin hook or the dashboard;
        SIM THREAD ONLY. No window anywhere."""
        self._teleop = (float(v_north), float(v_east))
        self._teleop_until = self._clock.now() + self._ev_teleop_timeout

    def teleop_active(self, now=None) -> bool:
        now = self._clock.now() if now is None else now
        return self._teleop is not None and now <= self._teleop_until

    def _step_evasive(self, dt: float) -> None:
        if not self.in_arena:
            return
        now = self._clock.now()
        n0, e0 = self.arena_position()
        if self.teleop_active(now):
            vn, ve = self._teleop              # human drive (arena m/s)
        else:
            dn, de = self._evasive_direction(now, n0, e0)
            vn, ve = dn * self._ev_speed, de * self._ev_speed
        # integrate in arena frame, clamped to bounds + crate-avoiding (slide)
        n1 = self._clamp_n(n0 + vn * dt)
        e1 = self._clamp_e(e0 + ve * dt)
        if self._hits_crate(n1, e1):
            if not self._hits_crate(n1, e0):       # slide along north
                e1 = e0
            elif not self._hits_crate(n0, e1):     # slide along east
                n1 = n0
            else:
                n1, e1 = n0, e0                     # boxed in: hold
        wx, wy, _ = frames.arena_to_world(self.cfg, n1, e1, 0.0)
        delta_x, delta_y = wx - self.pos[0], wy - self.pos[1]
        self.pos[0], self.pos[1] = wx, wy
        # jerky heading: rate-limited at the FAST evasive turn rate (jukes make
        # it change abruptly — intentionally un-smooth, unlike the autonomous).
        if abs(delta_x) > 1e-6 or abs(delta_y) > 1e-6:
            self._turn_toward(math.atan2(delta_y, delta_x), dt,
                              self._ev_turn_rate)
        self._mirror()

    def _evasive_direction(self, now, n0, e0):
        """Unit arena (north, east) heading: flee the nearest drone (cover-
        seeking) when close, else roam; with periodic random jukes."""
        if now >= self._next_decision:
            self._next_decision = now + self._ev_decision_period
            self._juke = bool(self._rng.random() < self._juke_prob)
            self._juke_angle = float(self._rng.uniform(-math.pi / 2,
                                                       math.pi / 2))
        drone_ne, dist = self._nearest_drone(n0, e0)
        if drone_ne is not None and dist <= self._flee_radius:
            dn, de = self._flee_direction(n0, e0, drone_ne)
        else:
            dn, de = self._roam_direction(now, n0, e0)
        if self._juke and (dn or de):              # inject a random heading juke
            c, s = math.cos(self._juke_angle), math.sin(self._juke_angle)
            dn, de = c * dn - s * de, s * dn + c * de
        return dn, de

    def _nearest_drone(self, n0, e0):
        best, best_d = None, float("inf")
        for d in self._drones:
            if not getattr(d, "flying", False):
                continue
            dn_, de_ = frames.world_to_arena(self.cfg, d.pos[0], d.pos[1])
            dist = math.hypot(dn_ - n0, de_ - e0)
            if dist < best_d:
                best, best_d = (dn_, de_), dist
        return best, best_d

    def _flee_direction(self, n0, e0, drone_ne):
        away_n, away_e = n0 - drone_ne[0], e0 - drone_ne[1]
        norm = math.hypot(away_n, away_e) or 1.0
        away = (away_n / norm, away_e / norm)
        if self._cover_bias <= 0.0:
            return away
        # Among candidate headings around the away-direction, prefer the one
        # that keeps a crate between the rover's next position and the drone
        # (cover_bias) while still fleeing (flee_gain).
        best_dir, best_score = away, -1e18
        for ang in (-1.2, -0.6, 0.0, 0.6, 1.2):
            c, s = math.cos(ang), math.sin(ang)
            d = (c * away[0] - s * away[1], s * away[0] + c * away[1])
            look = (n0 + d[0] * _COVER_LOOKAHEAD_M,
                    e0 + d[1] * _COVER_LOOKAHEAD_M)
            shadow = self._cover_score(look, drone_ne)
            align = d[0] * away[0] + d[1] * away[1]   # stay near the away dir
            score = self._flee_gain * align + self._cover_bias * shadow
            if score > best_score:
                best_score, best_dir = score, d
        return best_dir

    def _cover_score(self, look, drone_ne) -> float:
        """How shadowed a lookahead point is: 1 when a crate sits on the
        segment lookahead->drone, fading to 0 at its edge."""
        best = 0.0
        for o in self._ground_obstacles:
            d = _point_segment_dist(o.north, o.east, look[0], look[1],
                                    drone_ne[0], drone_ne[1])
            reach = max(o.half_n, o.half_e) + _ROVER_RADIUS_M
            if d < reach:
                best = max(best, 1.0 - d / reach)
        return best

    def _roam_direction(self, now, n0, e0):
        if self._ev_target is None or math.hypot(
                self._ev_target[0] - n0, self._ev_target[1] - e0) < 0.4:
            self._ev_target = self._pick_roam_target(n0, e0)
        if self._ev_target is None:
            return (0.0, 0.0)
        dn, de = self._ev_target[0] - n0, self._ev_target[1] - e0
        norm = math.hypot(dn, de) or 1.0
        return (dn / norm, de / norm)

    def _pick_roam_target(self, n0, e0):
        for _ in range(_WAYPOINT_TRIES):
            n = float(self._rng.uniform(*self._bounds_n))
            e = float(self._rng.uniform(*self._bounds_e))
            if self._segment_clear(n0, e0, n, e):
                return (n, e)
        return None

    def _clamp_n(self, n):
        lo, hi = _ROVER_RADIUS_M, self.cfg.arena.length_m - _ROVER_RADIUS_M
        return max(lo, min(hi, n))

    def _clamp_e(self, e):
        lo, hi = _ROVER_RADIUS_M, self.cfg.arena.width_m - _ROVER_RADIUS_M
        return max(lo, min(hi, e))

    def _hits_crate(self, n, e) -> bool:
        return any(arena.point_rect_dist_m(n, e, o.north, o.east, o.half_n,
                                           o.half_e) < _ROVER_RADIUS_M
                   for o in self._ground_obstacles)

    # ------------------------------------------------------------------ #
    # Waypoint sampling (patrol mode)
    # ------------------------------------------------------------------ #

    def _pick_waypoint(self, now):
        cur_n, cur_e = self.arena_position()
        for _ in range(_WAYPOINT_TRIES):
            n = self._rng.uniform(*self._bounds_n)
            e = self._rng.uniform(*self._bounds_e)
            if self._segment_clear(cur_n, cur_e, n, e):
                wx, wy, _ = frames.arena_to_world(self.cfg, n, e, 0.0)
                return np.array([wx, wy])
        self._pause_until = now + self._pause_s  # boxed in: wait, retry later
        return None

    def _segment_clear(self, n0, e0, n1, e1) -> bool:
        """Sampled straight-path clearance vs obstacle footprints."""
        length = math.hypot(n1 - n0, e1 - e0)
        samples = max(2, int(length / _SEGMENT_STEP_M) + 1)
        for i in range(samples + 1):
            t = i / samples
            n, e = n0 + (n1 - n0) * t, e0 + (e1 - e0) * t
            for o in self._obstacles:
                if arena.point_rect_dist_m(n, e, o.north, o.east, o.half_n,
                                           o.half_e) < _OBSTACLE_CLEAR_M:
                    return False
        return True
