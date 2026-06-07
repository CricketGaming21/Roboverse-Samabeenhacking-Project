"""Frame conversions: arena(m) <-> pybullet-world. ALL conversions live here.

Simulator INTERNAL — mission code must never import simcore.

Phase 1 ships the arena<->world subset needed to place bodies; the per-drone
takeoff-origin frame (cm) lands in Phase 3.

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
