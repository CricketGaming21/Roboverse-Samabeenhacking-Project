"""Frame & unit conversions — the metres/cm + arena/body spine.

Conventions (docs/ARCHITECTURE.md, docs/SDK_REFERENCE.md):
  - arena / UWB:  x = North (m), y = East (m)
  - body:         forward, right, up   (send_manual_control sticks, −1..1)
  - body→arena rotation by heading `yaw` (radians, CCW from north):
        north = forward*cos(yaw) − right*sin(yaw)
        east  = forward*sin(yaw) + right*cos(yaw)
    so at the operational locked yaw of 0, forward=+north and right=+east (identity).

The physical CCW sign and per-stick inversions are calibrated on the day via config
(`frame.yaw_offset_deg`, `frame.invert_forward`, `frame.invert_right`); this module is
the pure geometry the loop builds on.

Units: UWB is METRES, pyhulax is CENTIMETRES. Convert ONLY at these named boundaries.
"""

from __future__ import annotations

import math
from typing import Tuple

# HARD competition cap — commanded horizontal speed must never exceed this (m/s).
MAX_SPEED_MPS = 0.5


def m_to_cm(m: float) -> float:
    return m * 100.0


def cm_to_m(cm: float) -> float:
    return cm / 100.0


def body_to_arena(forward: float, right: float, yaw_rad: float) -> Tuple[float, float]:
    """Body (forward,right) → arena (north_m, east_m) at heading `yaw_rad`."""
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    north = forward * c - right * s
    east = forward * s + right * c
    return (north, east)


def arena_to_body(ex_m: float, ey_m: float, yaw_rad: float) -> Tuple[float, float]:
    """Arena error (north,east) → body (forward,right) at heading `yaw_rad`.
    Exact inverse of `body_to_arena` (rotation transpose)."""
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    forward = ex_m * c + ey_m * s
    right = -ex_m * s + ey_m * c
    return (forward, right)


def clamp_speed(vx: float, vy: float, cap: float = MAX_SPEED_MPS) -> Tuple[float, float]:
    """Scale a velocity vector so its magnitude never exceeds `cap` (preserves heading)."""
    mag = math.hypot(vx, vy)
    if mag <= cap or mag < 1e-12:
        return (vx, vy)
    scale = cap / mag
    return (vx * scale, vy * scale)
