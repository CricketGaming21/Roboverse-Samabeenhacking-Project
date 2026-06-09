"""`fly_to_uwb` — the committing UWB closed-loop transit controller.

Closed loop on **arena truth** (UWB, metres): read tag → arena error → body velocity
(P control, clamped **≤ 0.5 m/s**) → stick command at `rate_hz`. Altitude is held on
`get_altitude()` (ToF), never on UWB (UWB has no Z). On a `(None,None,None)` UWB sample
the loop **holds position** (zero horizontal sticks) and keeps emitting — never lurches.

Arrival uses a **UWB-derived speed** (Δpos/Δt across consecutive samples) because the sim
has no `get_velocity()`. A `guard` (P3) may filter the horizontal command; the guard never
contributes +up (altitude hold owns the vertical, and never climbs above cruise to clear
an obstacle — that is structural, handled by 2-D routing).
"""

from __future__ import annotations

import math
import time
from typing import Callable, Optional, Tuple

from mission.frames import arena_to_body, clamp_speed


def fly_to_uwb(drone, uwb, tag_id: int, target_xy_m: Tuple[float, float], *,
               alt_m: float = 1.10, tol_m: float = 0.10, speed_tol_mps: float = 0.10,
               guard=None, rate_hz: float = 20.0,
               kp_xy: float = 0.8, kp_alt: float = 0.8,
               max_mps: float = 0.5, climb_mps: float = 0.5,
               yaw_offset_deg: float = 0.0,
               invert_forward: bool = False, invert_right: bool = False,
               hold_on_dropout: bool = True, max_steps: int = 6000,
               sleep: Callable[[float], None] = time.sleep,
               on_step: Optional[Callable[[dict], None]] = None) -> bool:
    """Drive `drone` to `target_xy_m` (arena metres) using UWB tag `tag_id`.

    Returns True when arrived (within `tol_m` and slower than `speed_tol_mps`),
    False if `max_steps` is exhausted (e.g., persistent UWB dropout).
    """
    tn, te = float(target_xy_m[0]), float(target_xy_m[1])
    dt = 1.0 / rate_hz if rate_hz > 0 else 0.05
    prev_xy: Optional[Tuple[float, float]] = None
    prev_t: Optional[float] = None

    for step in range(max_steps):
        x, y, t = uwb.get_tag_position(tag_id)

        # altitude hold runs regardless of UWB (ToF is independent of UWB).
        alt_cm = drone.get_altitude()
        alt_err_m = alt_m - alt_cm / 100.0
        up_vel = max(-climb_mps, min(climb_mps, kp_alt * alt_err_m))
        up_stick = up_vel / climb_mps if climb_mps > 0 else 0.0
        up_stick = max(-1.0, min(1.0, up_stick))

        if x is None or y is None:
            # UWB dropout → HOLD horizontal, keep altitude + heartbeat. Never lurch.
            if hold_on_dropout:
                drone.send_manual_control(0.0, 0.0, up_stick, 0.0)
                if on_step is not None:
                    on_step({"step": step, "dropout": True, "fwd": 0.0,
                             "right": 0.0, "up": up_stick})
                sleep(dt)
                continue
            # if not holding, fall through treating as no error (also safe)
            x, y, t = tn, te, None

        err_n, err_e = tn - x, te - y
        err_dist = math.hypot(err_n, err_e)

        # UWB-derived speed (Δpos/Δt) — used for the arrival gate.
        uwb_speed = 0.0
        if prev_xy is not None and prev_t is not None and t is not None:
            ddt = t - prev_t
            if ddt > 1e-9:
                uwb_speed = math.hypot(x - prev_xy[0], y - prev_xy[1]) / ddt
        prev_xy, prev_t = (x, y), t

        if err_dist <= tol_m and uwb_speed <= speed_tol_mps:
            drone.send_manual_control(0.0, 0.0, up_stick, 0.0)   # settle, hold alt
            if on_step is not None:
                on_step({"step": step, "arrived": True, "err": err_dist})
            return True

        # P control in arena, clamped to the hard speed cap, then mapped to body.
        vx, vy = clamp_speed(kp_xy * err_n, kp_xy * err_e, cap=max_mps)
        yaw_rad = math.radians(drone.get_orientation().yaw + yaw_offset_deg)
        v_fwd, v_right = arena_to_body(vx, vy, yaw_rad)
        # velocity → stick (fake/real map stick·max_mps = velocity)
        fwd = max(-1.0, min(1.0, v_fwd / max_mps)) if max_mps > 0 else 0.0
        right = max(-1.0, min(1.0, v_right / max_mps)) if max_mps > 0 else 0.0
        if invert_forward:
            fwd = -fwd
        if invert_right:
            right = -right

        if guard is not None:
            obstacles = drone.get_obstacles()
            fwd, right, gup = guard.filter(fwd, right, obstacles)
            # the guard NEVER climbs; altitude hold owns +up.
            up_stick = min(up_stick, 0.0) if gup < 0 else up_stick

        drone.send_manual_control(fwd, right, up_stick, 0.0)
        if on_step is not None:
            on_step({"step": step, "fwd": fwd, "right": right, "up": up_stick,
                     "err": err_dist, "uwb_speed": uwb_speed, "xy": (x, y)})
        sleep(dt)

    return False
