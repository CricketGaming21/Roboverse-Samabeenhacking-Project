"""Phase 2 — search + lock-on + tag (the ambush).

Persistent **vantage patrol / chokepoint overwatch** (cycle preplanned look-points and
re-observe lanes — NOT a one-pass lawnmower; moving targets need frequent re-observation).
`lock_and_tag` is a **visual servo** on the marker's pixel offset (station-keep over a
near-stationary target, velocity-match a mover), bounded & time-boxed, banking distinct ids
to the shared `MissionState`. Bubble-gating keeps each drone in its zone, with a
**commitment rule** (a started lock finishes across the boundary — lock_and_tag is bounded)
and a **mop-up** endgame (drop gating when only stragglers remain).

≤0.5 m/s everywhere (clamped at the stick); never overflies a footprint (routes + guard).
"""

from __future__ import annotations

import math
import time
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

from mission.control.uwb_loop import fly_to_uwb
from mission.perception.aruco import confirm_with_aruco, is_rover_id
from mission.planner.geometry import Rect
from mission.planner.projection import CameraIntrinsics, pixel_to_arena
from mission.world.taskboard import Track

Point = Tuple[float, float]


# --------------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------------- #
def point_in_poly(p: Point, poly: Sequence[Point]) -> bool:
    """Ray-cast point-in-polygon (arena n,e)."""
    n = len(poly)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > p[1]) != (yj > p[1])) and \
                (p[0] < (xj - xi) * (p[1] - yi) / (yj - yi + 1e-15) + xi):
            inside = not inside
        j = i
    return inside


def _alt_up(drone, alt_m: float, kp_alt: float = 0.8, climb_mps: float = 0.5) -> float:
    err = alt_m - drone.get_altitude() / 100.0
    v = max(-climb_mps, min(climb_mps, kp_alt * err))
    return max(-1.0, min(1.0, v / climb_mps if climb_mps > 0 else 0.0))


def drone_arena_xy(drone, uwb=None, tag_id: Optional[int] = None) -> Optional[Point]:
    """Drone arena (north,east) from UWB (real + sim). Falls back to the fake's truth
    attrs only when UWB has no fix — NEVER assumes the real DroneAPI exposes `.n`/`.e`."""
    if uwb is not None and tag_id is not None:
        x, y, _ = uwb.get_tag_position(tag_id)
        if x is not None and y is not None:
            return (x, y)
    n, e = getattr(drone, "n", None), getattr(drone, "e", None)
    return (n, e) if n is not None and e is not None else None


def _marker_xy(bbox, cam_xy: Optional[Point], yaw_deg: float, alt_m: float,
               gimbal_deg: float, intr: Optional[CameraIntrinsics]) -> Optional[Point]:
    """Project a marker bbox centre to the floor, given the drone's UWB camera position."""
    if intr is None or cam_xy is None:
        return cam_xy
    x, y, w, h = bbox
    try:
        return pixel_to_arena(x + w / 2, y + h / 2, cam_xy, yaw_deg, alt_m, gimbal_deg, intr)
    except ValueError:
        return cam_xy


# --------------------------------------------------------------------------- #
# lock-on + tag (visual servo)
# --------------------------------------------------------------------------- #
def lock_and_tag(drone, stream, detection, state, *, hold_frames: int = 5,
                 center_tol_px: int = 45, lock_timeout_s: float = 6.0,
                 rate_hz: float = 20.0, kp_px: float = 0.02, gimbal_deg: float = 90.0,
                 max_mps: float = 0.5, alt_m: float = 1.1,
                 intrinsics: Optional[CameraIntrinsics] = None, uwb=None,
                 tag_id: Optional[int] = None,
                 dictionary: str = "DICT_6X6_250", sleep=time.sleep,
                 should_stop: Optional[Callable[[], bool]] = None,
                 clock: Callable[[], float] = time.time, on_step=None) -> bool:
    """Servo the drone to centre `detection.marker_id` and hold `hold_frames` frames,
    then bank it. Bounded by `lock_timeout_s` (no deadlock). Returns True iff banked."""
    target_id = detection.marker_id
    if target_id is None:
        return False
    intr = intrinsics or CameraIntrinsics()
    cx, cy = intr.cx, intr.cy
    drone.set_camera_angle(_DOWN, gimbal_deg)
    dt = 1.0 / rate_hz if rate_hz > 0 else 0.05
    steps = max(1, int(lock_timeout_s * rate_hz))
    held = 0
    for _ in range(steps):
        if should_stop is not None and should_stop():
            return False
        up = _alt_up(drone, alt_m)
        frame = stream.latest_frame
        if frame is None:
            drone.send_manual_control(0.0, 0.0, up, 0.0)
            sleep(dt)
            continue
        rgb = frame.to_rgb()
        match = next((d for d in confirm_with_aruco(rgb, dictionary)
                      if d.marker_id == target_id), None)
        if match is None:
            held = 0
            drone.send_manual_control(0.0, 0.0, up, 0.0)   # lost it → hold, keep looking
            sleep(dt)
            continue
        bx, by, bw, bh = match.bbox
        ex, ey = (bx + bw / 2) - cx, (by + bh / 2) - cy   # +ex=east, +ey=north (nadir)
        if math.hypot(ex, ey) <= center_tol_px:
            held += 1
            if held >= hold_frames:
                cam_xy = drone_arena_xy(drone, uwb, tag_id)
                xy = _marker_xy(match.bbox, cam_xy, drone.get_orientation().yaw,
                                drone.get_altitude() / 100.0, gimbal_deg, intr)
                state.bank(target_id, rgb, xy, clock())
                drone.send_manual_control(0.0, 0.0, up, 0.0)
                return True
        else:
            held = 0
        fwd = max(-1.0, min(1.0, kp_px * ey))
        right = max(-1.0, min(1.0, kp_px * ex))
        drone.send_manual_control(fwd, right, up, 0.0)
        if on_step is not None:
            on_step({"phase": "lock", "offset_px": math.hypot(ex, ey), "held": held})
        sleep(dt)
    return False


# --------------------------------------------------------------------------- #
# vantage patrol
# --------------------------------------------------------------------------- #
def vantage_patrol(drone, uwb, tag_id: int, vantages: Sequence[dict],
                   on_dwell: Callable[[], bool], *, gimbal_deg: float = 90.0,
                   dwell_s: float = 1.0, alt_m: float = 1.1, guard=None, graph=None,
                   rate_hz: float = 20.0, sleep=time.sleep, on_step=None,
                   **loop_kwargs) -> bool:
    """One cycle of overwatch: fly to each vantage, tilt the gimbal, dwell while calling
    `on_dwell()` per frame. `on_dwell` returns True to stop the whole patrol (done).

    With `graph`, each vantage hop is routed **around inflated footprints** (no-overfly is
    structural in Phase 2 too — a straight hop can cut over a crate, breaching compliance)."""
    from mission.planner.geometry import plan_path
    dt = 1.0 / rate_hz if rate_hz > 0 else 0.05
    for v in vantages:
        target = (float(v["xy"][0]), float(v["xy"][1]))
        waypoints = [target]
        if graph is not None:
            cur = drone_arena_xy(drone, uwb, tag_id) or target
            path = plan_path(cur, target, graph)
            if path:
                waypoints = path[1:] if len(path) > 1 else path
        for wp in waypoints:
            fly_to_uwb(drone, uwb, tag_id, wp, alt_m=alt_m, guard=guard,
                       rate_hz=rate_hz, sleep=sleep, on_step=on_step, **loop_kwargs)
        drone.set_camera_angle(_DOWN, float(v.get("gimbal_deg", gimbal_deg)))
        dwell_steps = max(1, int(float(v.get("dwell_s", dwell_s)) * rate_hz))
        for _ in range(dwell_steps):
            if on_dwell():
                return True
            drone.send_manual_control(0.0, 0.0, _alt_up(drone, alt_m), 0.0)
            if on_step is not None:
                on_step({"phase": "dwell"})
            sleep(dt)
    return False


# --------------------------------------------------------------------------- #
# the per-drone phase-2 driver
# --------------------------------------------------------------------------- #
def phase2_search(drone, uwb, tag_id: int, vantages: Sequence[dict], stream, state,
                  taskboard, *, bubble: Optional[Sequence[Point]] = None,
                  all_ids: Optional[Sequence[int]] = None, guard=None,
                  budget_cycles: int = 6, mopup_extra_cycles: int = 2,
                  dwell_s: float = 1.0, gimbal_deg: float = 90.0, rate_hz: float = 20.0,
                  lock_timeout_s: float = 6.0, center_tol_px: int = 45,
                  kp_px: float = 0.02, hold_frames: int = 5,
                  intrinsics: Optional[CameraIntrinsics] = None, alt_m: float = 1.1,
                  graph=None, sleep=time.sleep, clock: Callable[[], float] = time.time,
                  on_step=None, **loop_kwargs) -> set:
    """Patrol vantages, lock-and-tag distinct rover ids within `bubble` (mop-up drops the
    gate near the end). Returns the set of ids THIS drone banked."""
    banked: set = set()
    flags = {"mopup": False}

    def done() -> bool:
        return all_ids is not None and len(set(all_ids) - state.tagged()) == 0

    def scan_and_lock() -> bool:
        if done():
            return True
        frame = stream.latest_frame
        if frame is None:
            return False
        cam_xy = drone_arena_xy(drone, uwb, tag_id)
        for d in confirm_with_aruco(frame.to_rgb()):
            mid = d.marker_id
            if mid is None or not is_rover_id(mid) or state.is_tagged(mid):
                continue
            xy = _marker_xy(d.bbox, cam_xy, drone.get_orientation().yaw,
                            drone.get_altitude() / 100.0, gimbal_deg, intrinsics)
            taskboard.see(Track(marker_id=mid, xy=xy if xy is not None else cam_xy,
                                t=clock()))
            if (bubble is not None and not flags["mopup"]
                    and (xy is None or not point_in_poly(xy, bubble))):
                continue                                  # gated out of our zone (logged)
            # commitment: a started lock runs to completion (bounded)
            if lock_and_tag(drone, stream, d, state, hold_frames=hold_frames,
                            center_tol_px=center_tol_px, lock_timeout_s=lock_timeout_s,
                            rate_hz=rate_hz, kp_px=kp_px, gimbal_deg=gimbal_deg,
                            intrinsics=intrinsics, uwb=uwb, tag_id=tag_id, alt_m=alt_m,
                            sleep=sleep, clock=clock, on_step=on_step):
                banked.add(mid)
            return done()                                 # one target per scan frame
        return done()

    for cycle in range(budget_cycles):
        if done():
            break
        if all_ids is not None and cycle >= budget_cycles - mopup_extra_cycles:
            flags["mopup"] = True                         # endgame: drop the gate
        if vantage_patrol(drone, uwb, tag_id, vantages, scan_and_lock,
                          gimbal_deg=gimbal_deg, dwell_s=dwell_s, alt_m=alt_m,
                          guard=guard, graph=graph, rate_hz=rate_hz, sleep=sleep,
                          on_step=on_step, **loop_kwargs):
            break
    return banked


# --------------------------------------------------------------------------- #
# P8 — adversarial evader handling
# --------------------------------------------------------------------------- #
def classify_behaviour(samples: Sequence[Point], *, erratic_turn_std_deg: float = 45.0,
                       periodic_close_frac: float = 0.25,
                       min_path_m: float = 0.3) -> str:
    """Triage a track's recent path → 'smooth' | 'periodic' | 'erratic' | 'unknown'.

    Erratic (human evader) ⇒ high turn-angle variance. Periodic (loop) ⇒ low variance and
    returns near its start. Smooth (autonomous convoy) ⇒ low variance, open path."""
    pts = [(float(p[0]), float(p[1])) for p in samples]
    if len(pts) < 3:
        return "unknown"
    turns: List[float] = []
    path_len = 0.0
    for i in range(1, len(pts) - 1):
        v1 = (pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        v2 = (pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        path_len += math.hypot(*v1)
        if math.hypot(*v1) < 1e-9 or math.hypot(*v2) < 1e-9:
            continue
        ang = math.degrees(math.atan2(v1[0] * v2[1] - v1[1] * v2[0],
                                      v1[0] * v2[0] + v1[1] * v2[1]))
        turns.append(ang)
    path_len += math.hypot(pts[-1][0] - pts[-2][0], pts[-1][1] - pts[-2][1])
    if not turns or path_len < min_path_m:
        return "unknown"
    std = float(np.std(turns))
    if std > erratic_turn_std_deg:
        return "erratic"
    closes = math.dist(pts[0], pts[-1]) < periodic_close_frac * path_len
    return "periodic" if closes else "smooth"


class ReachableSet:
    """Where an evader could be, on the free-space grid. Holding a chokepoint `cut`s the
    set; with no expansion the size is monotonically non-increasing → containment shrinks
    the search (docs/ARCHITECTURE.md). Lane-graph reachability, not probability."""

    def __init__(self, bounds: Rect,
                 footprints: Sequence[Tuple[float, float, float, float]] = (),
                 cell_size: float = 0.25):
        self.bounds = bounds
        self.cell = float(cell_size)
        self.nn = max(1, int(round((bounds.max_n - bounds.min_n) / self.cell)))
        self.ne = max(1, int(round((bounds.max_e - bounds.min_e) / self.cell)))
        self.free = np.ones((self.nn, self.ne), dtype=bool)
        for i in range(self.nn):
            for j in range(self.ne):
                cn, ce = self._center(i, j)
                for fn, fe, sn, se in footprints:
                    if abs(cn - fn) <= sn / 2 and abs(ce - fe) <= se / 2:
                        self.free[i, j] = False
                        break
        self.barrier = np.zeros((self.nn, self.ne), dtype=bool)
        self.reach = self.free.copy()
        self._anchor: Optional[Tuple[int, int]] = None

    def _center(self, i: int, j: int) -> Point:
        return (self.bounds.min_n + (i + 0.5) * self.cell,
                self.bounds.min_e + (j + 0.5) * self.cell)

    def cell_of(self, xy: Point) -> Tuple[int, int]:
        i = int((xy[0] - self.bounds.min_n) / self.cell)
        j = int((xy[1] - self.bounds.min_e) / self.cell)
        return (min(max(i, 0), self.nn - 1), min(max(j, 0), self.ne - 1))

    def _component(self, start: Tuple[int, int]) -> np.ndarray:
        from collections import deque
        passable = self.free & ~self.barrier
        out = np.zeros_like(self.free)
        if not passable[start]:
            return out
        out[start] = True
        q = deque([start])
        while q:
            i, j = q.popleft()
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ni, nj = i + di, j + dj
                if 0 <= ni < self.nn and 0 <= nj < self.ne and \
                        passable[ni, nj] and not out[ni, nj]:
                    out[ni, nj] = True
                    q.append((ni, nj))
        return out

    def seed(self, xy: Point) -> None:
        self._anchor = self.cell_of(xy)
        self.reach = self._component(self._anchor)

    def cut(self, xy: Point, radius_m: float = 0.4) -> None:
        ci, cj = self.cell_of(xy)
        r = int(math.ceil(radius_m / self.cell))
        for i in range(max(0, ci - r), min(self.nn, ci + r + 1)):
            for j in range(max(0, cj - r), min(self.ne, cj + r + 1)):
                if math.hypot(*[a - b for a, b in zip(self._center(i, j), xy)]) <= radius_m:
                    self.barrier[i, j] = True
        if self._anchor is not None:
            self.reach = self._component(self._anchor)

    def expand(self, dt: float, speed: float) -> None:
        steps = max(1, int(round(speed * dt / self.cell)))
        passable = self.free & ~self.barrier
        for _ in range(steps):
            grown = self.reach.copy()
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                grown |= np.roll(self.reach, (di, dj), axis=(0, 1))
            self.reach = grown & passable

    def size(self) -> int:
        return int(self.reach.sum())

    def contains(self, xy: Point) -> bool:
        return bool(self.reach[self.cell_of(xy)])


def plan_containment(reachable: ReachableSet, chokepoints: Sequence[Point],
                     n_blockers: int) -> List[Point]:
    """Pick up to `n_blockers` chokepoints that best bound the reachable set (those whose
    neighbourhood overlaps the reachable region most) — hold these to shrink it."""
    scored = []
    for cp in chokepoints:
        cells = 0
        ci, cj = reachable.cell_of(cp)
        r = 2
        for i in range(max(0, ci - r), min(reachable.nn, ci + r + 1)):
            for j in range(max(0, cj - r), min(reachable.ne, cj + r + 1)):
                if reachable.reach[i, j]:
                    cells += 1
        if cells > 0:
            scored.append((cells, cp))
    scored.sort(key=lambda s: -s[0])
    return [cp for _, cp in scored[:n_blockers]]


# resolved once (the fake/real enum)
def _down_mode():
    from pyhulax.core import CameraPitchMode
    return CameraPitchMode.DOWN_ABSOLUTE


_DOWN = _down_mode()
