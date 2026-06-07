"""Frame conversions: arena(m) <-> pybullet-world. ALL conversions live here.

Simulator INTERNAL — mission code must never import simcore.

Covers: arena<->world (Phase 1), the per-drone TAKEOFF-ORIGIN frame and
BODY-relative directions (Phase 2). The pyhulax-facing axes are exactly
x = RIGHT, y = FORWARD, z = UP — the internal ENU/FLU conventions below must
never leak through the API; these functions are the only crossing point.

CONVENTIONS (fixed here, used everywhere — do not redefine elsewhere):
- Arena/UWB frame: x = NORTH (m), y = EAST (m). This is what UWBParserThread
  reports and how sim_config.yaml coordinates are written.
- PyBullet world frame: right-handed ENU — +x = arena EAST, +y = arena NORTH,
  +z = UP (at arena.yaw_deg = 0 and arena.origin = [0,0,0]). arena.origin and
  arena.yaw_deg place/rotate the arena inside the world.
- heading_deg (config): 0 = facing +north; positive = CCW viewed from above
  (same sign convention as pyhulax rotate(): positive = CCW).
- A body's nose/forward axis is its local +x.
"""

import math
from dataclasses import dataclass

from pyhulax.core import Direction


def arena_yaw_world_rad(cfg) -> float:
    """Rotation of the arena axes inside the pybullet world, radians."""
    return math.radians(cfg.arena.yaw_deg)


def arena_to_world(cfg, north: float, east: float, z: float = 0.0):
    """Arena (north, east) metres -> pybullet world (x, y, z)."""
    yaw = arena_yaw_world_rad(cfg)
    c, s = math.cos(yaw), math.sin(yaw)
    bx, by = east, north  # ENU: east -> +x, north -> +y (at zero arena yaw)
    ox, oy, oz = cfg.arena.origin
    return (ox + c * bx - s * by, oy + s * bx + c * by, oz + z)


def world_to_arena(cfg, x: float, y: float):
    """Pybullet world (x, y) -> arena (north, east) metres."""
    yaw = arena_yaw_world_rad(cfg)
    c, s = math.cos(yaw), math.sin(yaw)
    ox, oy, _ = cfg.arena.origin
    dx, dy = x - ox, y - oy
    bx = c * dx + s * dy   # east
    by = -s * dx + c * dy  # north
    return (by, bx)


def heading_to_world_yaw_rad(cfg, heading_deg: float) -> float:
    """Arena heading (0 = +north, CCW+) -> body yaw about world +z.

    Body local +x is the nose; at heading 0 the nose points at arena north
    (world +y when arena.yaw_deg = 0), hence the 90° offset.
    """
    return math.radians(90.0 + heading_deg) + arena_yaw_world_rad(cfg)


# --------------------------------------------------------------------------- #
# Takeoff-origin frame (§5.2) — what move_to()/get_position() speak
# --------------------------------------------------------------------------- #
# pyhulax axes: x = RIGHT, y = FORWARD, z = UP, in CENTIMETRES, fixed at the
# pose + heading captured at takeoff. The frame does NOT rotate when the drone
# yaws afterwards. (right, forward, up) is right-handed: right x forward = up.

@dataclass(frozen=True)
class TakeoffFrame:
    """Origin (world metres; oz = ground beneath the drone) + nose yaw psi0."""
    ox: float
    oy: float
    oz: float
    psi0: float  # world yaw (rad, about +z from world +x) of the nose at takeoff


def capture_takeoff_frame(x_w: float, y_w: float, ground_z_w: float,
                          yaw_world_rad: float) -> TakeoffFrame:
    """Freeze the takeoff-origin frame at the moment of takeoff."""
    return TakeoffFrame(ox=x_w, oy=y_w, oz=ground_z_w, psi0=yaw_world_rad)


def takeoff_cm_to_world(fr: TakeoffFrame, x_cm: float, y_cm: float,
                        z_cm: float):
    """Takeoff-frame point (cm; x=right, y=forward, z=up) -> world (m)."""
    fx, fy = math.cos(fr.psi0), math.sin(fr.psi0)    # forward = nose at takeoff
    rx, ry = math.sin(fr.psi0), -math.cos(fr.psi0)   # right = forward yawed -90°
    xr, yf, zu = x_cm / 100.0, y_cm / 100.0, z_cm / 100.0
    return (fr.ox + xr * rx + yf * fx,
            fr.oy + xr * ry + yf * fy,
            fr.oz + zu)


def world_to_takeoff_cm(fr: TakeoffFrame, x_w: float, y_w: float, z_w: float):
    """World point (m) -> takeoff-frame (cm; x=right, y=forward, z=up)."""
    fx, fy = math.cos(fr.psi0), math.sin(fr.psi0)
    rx, ry = math.sin(fr.psi0), -math.cos(fr.psi0)
    dx, dy = x_w - fr.ox, y_w - fr.oy
    return ((dx * rx + dy * ry) * 100.0,
            (dx * fx + dy * fy) * 100.0,
            (z_w - fr.oz) * 100.0)


def arena_to_takeoff_cm(cfg, fr: TakeoffFrame, north: float, east: float,
                        z_m: float = 0.0):
    """Arena (north, east[, z]) metres -> a drone's takeoff frame (cm)."""
    x_w, y_w, z_w = arena_to_world(cfg, north, east, z_m)
    return world_to_takeoff_cm(fr, x_w, y_w, z_w)


def takeoff_cm_to_arena(cfg, fr: TakeoffFrame, x_cm: float, y_cm: float,
                        z_cm: float = 0.0):
    """A drone's takeoff-frame point (cm) -> arena (north, east) metres."""
    x_w, y_w, _ = takeoff_cm_to_world(fr, x_cm, y_cm, z_cm)
    return world_to_arena(cfg, x_w, y_w)


# --------------------------------------------------------------------------- #
# Body-relative directions (§4.2 move()) — relative to the CURRENT heading
# --------------------------------------------------------------------------- #
# Body frame is FLU: nose = +x, left = +y, up = +z. After any rotate(),
# FORWARD follows the new nose — callers pass the CURRENT world yaw.

_BODY_DIRS = {
    Direction.FORWARD: (1.0, 0.0, 0.0),
    Direction.BACK: (-1.0, 0.0, 0.0),
    Direction.LEFT: (0.0, 1.0, 0.0),
    Direction.RIGHT: (0.0, -1.0, 0.0),
    Direction.UP: (0.0, 0.0, 1.0),
    Direction.DOWN: (0.0, 0.0, -1.0),
}


def body_direction_to_world(yaw_world_rad: float, direction):
    """Unit vector (world) for a pyhulax Direction at the current yaw."""
    bx, by, bz = _BODY_DIRS[Direction(int(direction))]
    c, s = math.cos(yaw_world_rad), math.sin(yaw_world_rad)
    return (c * bx - s * by, s * bx + c * by, bz)
