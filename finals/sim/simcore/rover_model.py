"""Rover agents: waypoint-random patrol + unique top-face ArUco markers.

Simulator INTERNAL — mission code must never import simcore.

NOT mission logic: rovers are world actors the mission has to find. Motion is
advanced on the SIM THREAD only (step(dt) from the registry loop), paced in
sim time, seeded per rover from the one config seed.

Patrol (config.rovers.patrol, mode waypoint_random): pick a random waypoint
inside bounds_north x bounds_east whose straight-line path stays clear of the
generated obstacles (sampled-segment check — sim realism, not a planner),
drive there at speed_mps facing the direction of travel, pause
waypoint_pause_s, repeat. speed_mps = 0 parks the rovers (handy for tests).
"""

import math

import numpy as np
import pybullet as p

from . import arena, frames

_WAYPOINT_TRIES = 50
_OBSTACLE_CLEAR_M = 0.3   # keep the path this far from obstacle footprints
_SEGMENT_STEP_M = 0.2     # sampling pitch of the straight-path clearance check


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

    def enter_arena(self) -> None:
        """Bring the rover into the arena (AMBUSH begins). Phase 11 replaces
        this teleport-to-spawn with the staggered convoy entry."""
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
    # Waypoint sampling
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
