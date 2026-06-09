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

from mission.control.uwb_loop import fly_to_uwb
from mission.perception.aruco import confirm_with_aruco, is_rover_id
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


def _marker_xy(bbox, drone, gimbal_deg: float, intr: Optional[CameraIntrinsics],
               alt_m: Optional[float]) -> Point:
    if intr is None:
        return (drone.n, drone.e)
    x, y, w, h = bbox
    a = alt_m if alt_m is not None else drone.get_altitude() / 100.0
    try:
        return pixel_to_arena(x + w / 2, y + h / 2, (drone.n, drone.e),
                              drone.get_orientation().yaw, a, gimbal_deg, intr)
    except ValueError:
        return (drone.n, drone.e)


# --------------------------------------------------------------------------- #
# lock-on + tag (visual servo)
# --------------------------------------------------------------------------- #
def lock_and_tag(drone, stream, detection, state, *, hold_frames: int = 5,
                 center_tol_px: int = 45, lock_timeout_s: float = 6.0,
                 rate_hz: float = 20.0, kp_px: float = 0.02, gimbal_deg: float = 90.0,
                 max_mps: float = 0.5, alt_m: float = 1.1,
                 intrinsics: Optional[CameraIntrinsics] = None,
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
                xy = _marker_xy(match.bbox, drone, gimbal_deg, intr, alt_m)
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
                   dwell_s: float = 1.0, alt_m: float = 1.1, guard=None,
                   rate_hz: float = 20.0, sleep=time.sleep, on_step=None,
                   **loop_kwargs) -> bool:
    """One cycle of overwatch: fly to each vantage, tilt the gimbal, dwell while calling
    `on_dwell()` per frame. `on_dwell` returns True to stop the whole patrol (done)."""
    dt = 1.0 / rate_hz if rate_hz > 0 else 0.05
    for v in vantages:
        fly_to_uwb(drone, uwb, tag_id, tuple(v["xy"]), alt_m=alt_m, guard=guard,
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
                  sleep=time.sleep, clock: Callable[[], float] = time.time,
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
        for d in confirm_with_aruco(frame.to_rgb()):
            mid = d.marker_id
            if mid is None or not is_rover_id(mid) or state.is_tagged(mid):
                continue
            xy = _marker_xy(d.bbox, drone, gimbal_deg, intrinsics, alt_m)
            taskboard.see(Track(marker_id=mid, xy=xy, t=clock()))
            if bubble is not None and not flags["mopup"] and not point_in_poly(xy, bubble):
                continue                                  # gated out of our zone (logged)
            # commitment: a started lock runs to completion (bounded)
            if lock_and_tag(drone, stream, d, state, hold_frames=hold_frames,
                            center_tol_px=center_tol_px, lock_timeout_s=lock_timeout_s,
                            rate_hz=rate_hz, kp_px=kp_px, gimbal_deg=gimbal_deg,
                            intrinsics=intrinsics, alt_m=alt_m, sleep=sleep,
                            clock=clock, on_step=on_step):
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
                          guard=guard, rate_hz=rate_hz, sleep=sleep, on_step=on_step,
                          **loop_kwargs):
            break
    return banked


# resolved once (the fake/real enum)
def _down_mode():
    from pyhulax.core import CameraPitchMode
    return CameraPitchMode.DOWN_ABSOLUTE


_DOWN = _down_mode()
