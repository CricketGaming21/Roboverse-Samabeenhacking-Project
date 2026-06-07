"""Procedural room/obstacle generation (seeded) — pure geometry, no PyBullet.

Simulator INTERNAL — mission code must never import simcore.

generate(cfg) is a pure function of the config (including meta.seed): the same
config always yields an identical ArenaLayout (the determinism contract that
tests/test_phase1 checks). All coordinates here are in the ARENA frame
(north, east, metres) — world placement happens in world.py via frames.py.

Room interior spans north in [0, length_m], east in [0, width_m]; the four
walls sit just OUTSIDE that span, so configured coordinates are always
positive and inside the room.
"""

import math
from dataclasses import dataclass

import numpy as np

from .log import get_logger

_MAX_ATTEMPTS = 20_000  # rejection-sampling guard

# Clearance the rovers' spawn sampling keeps from obstacles / each other.
# Spawn-time placement margins only (patrol motion is Phase 6).
_ROVER_OBSTACLE_CLEAR_M = 0.35
_ROVER_MUTUAL_CLEAR_M = 0.6


@dataclass(frozen=True)
class WallSpec:
    """One wall as an axis-aligned box in the arena frame (centre + half sizes)."""
    north: float
    east: float
    half_n: float
    half_e: float
    half_h: float


@dataclass(frozen=True)
class ObstacleSpec:
    """One box/pillar obstacle: axis-aligned footprint + height, arena frame."""
    north: float
    east: float
    half_n: float
    half_e: float
    height_m: float


@dataclass(frozen=True)
class StartPose:
    """A ground start pose in the arena frame."""
    north: float
    east: float
    heading_deg: float


@dataclass(frozen=True)
class ArenaLayout:
    """Everything world.py needs to build the scene. Frozen => comparable."""
    length_m: float
    width_m: float
    height_m: float
    wall_thickness_m: float
    walls: tuple            # 4x WallSpec
    obstacles: tuple        # N x ObstacleSpec
    drone_starts: tuple     # 3x StartPose (from config)
    rover_starts: tuple     # 5x StartPose (seeded random within patrol bounds)


# --------------------------------------------------------------------------- #
# Geometry helpers (shared with tests)
# --------------------------------------------------------------------------- #

def rect_gap_m(an, ae, ahn, ahe, bn, be, bhn, bhe) -> float:
    """Edge-to-edge distance between two axis-aligned rects (0 if overlapping)."""
    dn = abs(an - bn) - (ahn + bhn)
    de = abs(ae - be) - (ahe + bhe)
    return math.hypot(max(dn, 0.0), max(de, 0.0))


def point_rect_dist_m(pn, pe, on, oe, ohn, ohe) -> float:
    """Distance from a point to an axis-aligned rect (0 if inside)."""
    dn = max(abs(pn - on) - ohn, 0.0)
    de = max(abs(pe - oe) - ohe, 0.0)
    return math.hypot(dn, de)


# --------------------------------------------------------------------------- #
# Generation
# --------------------------------------------------------------------------- #

def _make_walls(cfg) -> tuple:
    L, W = cfg.arena.length_m, cfg.arena.width_m
    t = cfg.arena.wall_thickness_m
    half_h = cfg.arena.height_m / 2.0
    return (
        WallSpec(L + t / 2, W / 2, t / 2, W / 2 + t, half_h),   # far-north wall
        WallSpec(-t / 2, W / 2, t / 2, W / 2 + t, half_h),      # south wall
        WallSpec(L / 2, W + t / 2, L / 2 + t, t / 2, half_h),   # far-east wall
        WallSpec(L / 2, -t / 2, L / 2 + t, t / 2, half_h),      # west wall
    )


def _place_obstacles(cfg, rng, keepout_points) -> tuple:
    """Rejection-sample obstacle boxes honouring clearance + keepclear config."""
    ob = cfg.arena.obstacles
    L, W = cfg.arena.length_m, cfg.arena.width_m
    placed = []
    attempts = 0
    while len(placed) < ob.count:
        attempts += 1
        if attempts > _MAX_ATTEMPTS:
            raise RuntimeError(
                f"arena: placed only {len(placed)}/{ob.count} obstacles after "
                f"{_MAX_ATTEMPTS} attempts — relax clearance/keepclear/size config")
        half_n = rng.uniform(ob.footprint_min_m, ob.footprint_max_m) / 2.0
        half_e = rng.uniform(ob.footprint_min_m, ob.footprint_max_m) / 2.0
        height = rng.uniform(ob.height_min_m, ob.height_max_m)
        # Keep min_clearance from the walls too, so corridors stay flyable.
        margin_n = half_n + ob.min_clearance_m
        margin_e = half_e + ob.min_clearance_m
        if L - 2 * margin_n <= 0 or W - 2 * margin_e <= 0:
            continue  # this footprint can't fit; resample
        north = rng.uniform(margin_n, L - margin_n)
        east = rng.uniform(margin_e, W - margin_e)

        if any(point_rect_dist_m(kn, ke, north, east, half_n, half_e)
               < ob.keepclear_radius_m for kn, ke in keepout_points):
            continue
        if any(rect_gap_m(north, east, half_n, half_e,
                          o.north, o.east, o.half_n, o.half_e)
               < ob.min_clearance_m for o in placed):
            continue
        placed.append(ObstacleSpec(north, east, half_n, half_e, height))
    return tuple(placed)


def _place_rovers(cfg, rng, obstacles) -> tuple:
    """Seeded random rover start poses inside the patrol bounds."""
    pat = cfg.rovers.patrol
    n_lo, n_hi = pat.bounds_north
    e_lo, e_hi = pat.bounds_east
    placed = []
    attempts = 0
    while len(placed) < cfg.rovers.count:
        attempts += 1
        if attempts > _MAX_ATTEMPTS:
            raise RuntimeError(
                f"arena: placed only {len(placed)}/{cfg.rovers.count} rovers "
                f"after {_MAX_ATTEMPTS} attempts — check patrol bounds vs obstacles")
        north = rng.uniform(n_lo, n_hi)
        east = rng.uniform(e_lo, e_hi)
        heading = rng.uniform(0.0, 360.0)
        if any(point_rect_dist_m(north, east, o.north, o.east, o.half_n, o.half_e)
               < _ROVER_OBSTACLE_CLEAR_M for o in obstacles):
            continue
        if any(math.hypot(north - r.north, east - r.east) < _ROVER_MUTUAL_CLEAR_M
               for r in placed):
            continue
        placed.append(StartPose(north, east, heading))
    return tuple(placed)


def generate(cfg) -> ArenaLayout:
    """Generate the full seeded layout. Pure function of cfg (incl. meta.seed)."""
    if cfg.arena.shape != "rectangle":
        raise ValueError(f"unsupported arena.shape: {cfg.arena.shape!r}")
    rng = np.random.default_rng(cfg.meta.seed)

    # Obstacles must keep clear of pads AND drone starts (keepclear_radius_m).
    keepout = ([(p.north, p.east) for p in cfg.pads]
               + [(d.start[0], d.start[1]) for d in cfg.drones.units])

    obstacles = _place_obstacles(cfg, rng, keepout)
    rover_starts = _place_rovers(cfg, rng, obstacles)
    drone_starts = tuple(StartPose(d.start[0], d.start[1], d.heading_deg)
                         for d in cfg.drones.units)

    layout = ArenaLayout(
        length_m=cfg.arena.length_m,
        width_m=cfg.arena.width_m,
        height_m=cfg.arena.height_m,
        wall_thickness_m=cfg.arena.wall_thickness_m,
        walls=_make_walls(cfg),
        obstacles=obstacles,
        drone_starts=drone_starts,
        rover_starts=rover_starts,
    )
    get_logger("arena", cfg).debug(
        "generated layout: seed=%s obstacles=%d rovers=%d",
        cfg.meta.seed, len(obstacles), len(rover_starts))
    return layout
