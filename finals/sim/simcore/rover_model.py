"""Rover agents: convoy routing / random patrol + unique top ArUco markers.

Simulator INTERNAL — mission code must never import simcore.

NOT mission logic: rovers are world actors the mission has to find. They
follow FIXED configured routes — no evasion, no reacting to drones. Motion
is advanced on the SIM THREAD only (step(dt) from the registry loop).

TWO MOTION MODES (config rovers.motion):
- "convoy" (DEFAULT, matches the Phase-2 image): in AMBUSH the rovers enter
  one by one from scenario.entrance (staggered entry_stagger_s apart, a
  convoy column), follow the shared trunk to split_index, then each follows
  its own authored branch, then loiters (loop the branch | hold at the end).
  Purely config-driven and deterministic — no RNG anywhere.
- "patrol" (back-compat flag): the original seeded waypoint_random wander
  within patrol bounds. speed_mps = 0 parks the rovers (handy for tests).
"""

import math

import numpy as np
import pybullet as p

from . import arena, frames

_WAYPOINT_TRIES = 50
_OBSTACLE_CLEAR_M = 0.3   # keep the path this far from obstacle footprints
_SEGMENT_STEP_M = 0.2     # sampling pitch of the straight-path clearance check

_MOTION_MODES = ("convoy", "patrol")
_LOITER_MODES = ("loop", "hold")
_HOLD_BRAKE_M = 0.6       # loiter=hold: ease to a stop over this final distance


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
    """State + patrol motion for one rover (sim-thread only mutation)."""

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

        # Convoy mode (rovers.motion: convoy): fixed authored route.
        self._motion = cfg.rovers.motion
        if self._motion not in _MOTION_MODES:
            raise ValueError(f"unknown rovers.motion: {self._motion!r}")
        if self._motion == "convoy":
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

    def arena_position(self):
        """TRUE (north, east) metres."""
        return frames.world_to_arena(self.cfg, self.pos[0], self.pos[1])

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
        entry (rover k enters k * entry_stagger_s after now). Patrol
        (back-compat): the old teleport-to-spawn."""
        if self._motion == "convoy":
            self._entry_time = (now + self.index
                                * float(self.cfg.rovers.convoy.entry_stagger_s))
        else:
            self.enter_arena()

    def enter_arena(self) -> None:
        """Teleport to the layout spawn (patrol mode's AMBUSH entry)."""
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
        if self._motion == "convoy":
            self._step_convoy(dt)
            return
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
        self._turn_toward(math.atan2(delta[1], delta[0]), dt)
        self._mirror()

    def _turn_toward(self, target_yaw: float, dt: float) -> None:
        d = (target_yaw - self.yaw + math.pi) % (2 * math.pi) - math.pi
        step = self._turn_rate * dt
        self.yaw += d if abs(d) <= step else math.copysign(step, d)

    # ------------------------------------------------------------------ #
    # Waypoint sampling (patrol mode)
    # ------------------------------------------------------------------ #

    def _pick_waypoint(self, now: float):
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
