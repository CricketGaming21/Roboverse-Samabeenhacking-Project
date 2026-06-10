"""Phase 1 — deploy + land in the hoop.

`assign_pads` picks 3 of the announced valid pads for the 3 drones, minimising total
route length and avoiding crossing paths. `land_in_hoop` lands **purely on UWB** — the real
landing pads have NO ArUco markers (R1), so there is no decode/confirm: centre on the UWB
target within `hoop_tol_m`, then descend ONLY when centred-in-hoop, the descent column is
footprint-clear, and the down barrier is clear above 0.35 m. The camera stays off in Phase 1.

Count-first: never trade a landing for speed (docs/RULES_AND_CONSTRAINTS.md).
"""

from __future__ import annotations

import itertools
import math
import time
from typing import Dict, Optional, Sequence, Tuple

from mission.control.uwb_loop import fly_to_uwb
from mission.frames import arena_to_body, clamp_speed

Point = Tuple[float, float]
_CROSS_PENALTY = 1e6


# --------------------------------------------------------------------------- #
# pad assignment
# --------------------------------------------------------------------------- #
def _orient(a: Point, b: Point, c: Point) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _segments_cross(a1: Point, a2: Point, b1: Point, b2: Point) -> bool:
    """Proper segment intersection (interiors cross)."""
    d1 = _orient(b1, b2, a1)
    d2 = _orient(b1, b2, a2)
    d3 = _orient(a1, a2, b1)
    d4 = _orient(a1, a2, b2)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def _pad_xy(pad) -> Point:
    return (float(pad.north), float(pad.east))


def assign_pads(valid_pads: Sequence, drone_starts: Dict[int, Point]) -> Dict[int, object]:
    """Assign 3 (or len(drone_starts)) valid pads to drones, minimising total straight
    path length with a heavy penalty for crossing routes. Returns {tag_id: pad}."""
    tags = sorted(drone_starts)
    k = len(tags)
    if len(valid_pads) < k:
        raise ValueError(f"need ≥{k} valid pads, got {len(valid_pads)}")

    best, best_cost = None, math.inf
    for combo in itertools.permutations(valid_pads, k):
        segs = [(drone_starts[tags[i]], _pad_xy(combo[i])) for i in range(k)]
        total = sum(math.dist(s, p) for s, p in segs)
        crossings = sum(
            1 for i in range(k) for j in range(i + 1, k)
            if _segments_cross(segs[i][0], segs[i][1], segs[j][0], segs[j][1]))
        cost = total + crossings * _CROSS_PENALTY
        if cost < best_cost:
            best_cost, best = cost, combo
    return {tags[i]: best[i] for i in range(k)}


# --------------------------------------------------------------------------- #
# landing
# --------------------------------------------------------------------------- #
def _point_in_footprint(p: Point, footprints: Sequence[Tuple[float, float, float, float]],
                        margin: float = 0.0) -> bool:
    for cn, ce, sn, se in footprints:
        if abs(p[0] - cn) <= sn / 2 + margin and abs(p[1] - ce) <= se / 2 + margin:
            return True
    return False


def land_in_hoop(drone, uwb, tag_id: int, pad_xy: Point, hoop_tol_m: float, *,
                 footprints: Optional[Sequence] = None, cruise_alt_m: float = 1.10,
                 land_alt_cm: float = 12.0, descend_stick: float = 0.4,
                 rate_hz: float = 20.0, max_steps: int = 8000,
                 kp_xy: float = 0.8, kp_alt: float = 0.8, max_mps: float = 0.5,
                 climb_mps: float = 0.5, yaw_offset_deg: float = 0.0,
                 invert_forward: bool = False, invert_right: bool = False,
                 hold_on_dropout: bool = True, sleep=time.sleep,
                 on_step=None) -> bool:
    """Centre on the pad (UWB target) then descend into the hoop — **UWB-only, no ArUco**
    (R1: the real pads carry no markers). Returns True iff the final UWB position is within
    `hoop_tol_m` of the pad centre."""
    # 1) centre over the pad within the hoop tolerance
    centred = fly_to_uwb(drone, uwb, tag_id, pad_xy, alt_m=cruise_alt_m,
                         tol_m=hoop_tol_m, speed_tol_mps=max(0.1, hoop_tol_m),
                         rate_hz=rate_hz, kp_xy=kp_xy, kp_alt=kp_alt, max_mps=max_mps,
                         climb_mps=climb_mps, yaw_offset_deg=yaw_offset_deg,
                         invert_forward=invert_forward, invert_right=invert_right,
                         hold_on_dropout=hold_on_dropout, max_steps=max_steps,
                         sleep=sleep, on_step=on_step)
    if not centred:
        return False

    # 2) descent column must be footprint-clear, and nothing unexpected below
    if footprints is not None and _point_in_footprint(pad_xy, footprints):
        return False
    if drone.get_obstacles().down:           # something below at cruise → abort
        return False

    # 3) controlled descent that keeps re-centring; descend only while centred
    dt = 1.0 / rate_hz if rate_hz > 0 else 0.05
    for _ in range(max_steps):
        x, y, _t = uwb.get_tag_position(tag_id)
        alt_cm = drone.get_altitude()
        if x is None or y is None:
            drone.send_manual_control(0.0, 0.0, 0.0, 0.0)   # hold on dropout
            sleep(dt)
            continue
        err_n, err_e = pad_xy[0] - x, pad_xy[1] - y
        err = math.hypot(err_n, err_e)
        vx, vy = clamp_speed(kp_xy * err_n, kp_xy * err_e, cap=max_mps)
        yaw = math.radians(drone.get_orientation().yaw + yaw_offset_deg)
        v_fwd, v_right = arena_to_body(vx, vy, yaw)
        fwd = max(-1.0, min(1.0, v_fwd / max_mps))
        right = max(-1.0, min(1.0, v_right / max_mps))
        if invert_forward:
            fwd = -fwd
        if invert_right:
            right = -right
        down = -descend_stick if err <= hoop_tol_m else 0.0   # descend only when centred
        if alt_cm <= land_alt_cm and err <= hoop_tol_m:
            break
        drone.send_manual_control(fwd, right, down, 0.0)
        if on_step is not None:
            on_step({"phase": "descend", "alt_cm": alt_cm, "err": err})
        sleep(dt)

    drone.land()
    x, y, _t = uwb.get_tag_position(tag_id)
    if x is None or y is None:
        return False
    return math.hypot(pad_xy[0] - x, pad_xy[1] - y) <= hoop_tol_m
