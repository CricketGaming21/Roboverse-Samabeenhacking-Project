"""Room/obstacle layout generation — pure geometry, no PyBullet.

Simulator INTERNAL — mission code must never import simcore.

TWO LAYOUT MODES (config arena.layout):
- "authored" (DEFAULT): the fixed map matching the reference images — crate
  clusters + archway from arena.authored, deterministic, no RNG. Coordinates
  are approximate starting points meant to be nudged in sim_config.yaml.
- "procedural" (RETAINED — robustness mode): the seeded rejection sampler.
  The real competition map is unknown; a mission tuned only against the
  authored map would overfit, so this mode stays working.

generate(cfg) is a pure function of the config (including meta.seed): the same
config always yields an identical ArenaLayout. All coordinates here are in the
ARENA frame (north, east, metres) — world placement happens in world.py via
frames.py. Room interior spans north in [0, length_m], east in [0, width_m].
"""

import math
from dataclasses import dataclass

import numpy as np

from . import aruco_assets
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
    """One box obstacle: axis-aligned footprint + height, arena frame.
    z0_m raises the box off the floor (the archway lintel); ground boxes
    keep the default 0."""
    north: float
    east: float
    half_n: float
    half_e: float
    height_m: float
    z0_m: float = 0.0


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


def _authored_obstacles(cfg) -> tuple:
    """The fixed authored structures from arena.authored — one ObstacleSpec per
    entry (center / size / height), data-driven, no RNG. Arch gates are two
    post entries each; the rest are solid structures."""
    au = cfg.arena.authored
    obstacles = []
    for ci, cl in enumerate(au.clusters):
        if len(cl.center) != 2 or len(cl.size) != 2:
            raise ValueError(f"authored structure {ci}: center and size must "
                             f"each be [north, east] pairs (got {cl})")
        cn, ce = float(cl.center[0]), float(cl.center[1])
        sn, se = float(cl.size[0]), float(cl.size[1])
        if sn <= 0.0 or se <= 0.0 or float(cl.height) <= 0.0:
            raise ValueError(f"authored structure {ci}: size and height must "
                             f"be positive (got {cl})")
        obstacles.append(ObstacleSpec(cn, ce, sn / 2.0, se / 2.0,
                                      float(cl.height)))
    _validate_authored(cfg, obstacles)
    return tuple(obstacles)


def _validate_authored(cfg, obstacles) -> None:
    """Guard the hand-authored coordinates: in bounds, and no GROUND box may
    overlap a pad footprint or sit on a drone start (pads must stay landable).
    Raises ValueError on a bad config rather than building a broken arena."""
    L, W = cfg.arena.length_m, cfg.arena.width_m
    pad_half = cfg.aruco.pad_marker_size_m * aruco_assets.texture_scale() / 2
    for o in obstacles:
        if not (0.0 <= o.north - o.half_n and o.north + o.half_n <= L
                and 0.0 <= o.east - o.half_e and o.east + o.half_e <= W):
            raise ValueError(f"authored obstacle out of bounds: {o}")
        if o.z0_m > 0.0:
            continue  # elevated (lintel): ground clearance not applicable
        for pad in cfg.pads:
            if rect_gap_m(o.north, o.east, o.half_n, o.half_e,
                          pad.north, pad.east, pad_half, pad_half) <= 0.0:
                raise ValueError(f"authored obstacle overlaps pad {pad.id}: "
                                 f"{o} — adjust arena.authored or pads")
        for d in cfg.drones.units:
            if point_rect_dist_m(d.start[0], d.start[1], o.north, o.east,
                                 o.half_n, o.half_e) < 0.15:
                raise ValueError(f"authored obstacle sits on drone start "
                                 f"{d.start}: {o}")


def generate(cfg) -> ArenaLayout:
    """Generate the layout for the configured arena.layout mode.

    authored => fixed obstacles (no RNG); procedural => seeded sampler.
    Rover starts are seeded random within patrol bounds in BOTH modes.
    """
    if cfg.arena.shape != "rectangle":
        raise ValueError(f"unsupported arena.shape: {cfg.arena.shape!r}")
    rng = np.random.default_rng(cfg.meta.seed)

    if cfg.arena.layout == "authored":
        obstacles = _authored_obstacles(cfg)
    elif cfg.arena.layout == "procedural":
        # Obstacles keep clear of pads AND drone starts (keepclear_radius_m).
        keepout = ([(p.north, p.east) for p in cfg.pads]
                   + [(d.start[0], d.start[1]) for d in cfg.drones.units])
        obstacles = _place_obstacles(cfg, rng, keepout)
    else:
        raise ValueError(f"unknown arena.layout: {cfg.arena.layout!r} "
                         f"(use 'authored' or 'procedural')")

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
        "generated layout (%s): seed=%s obstacles=%d rovers=%d",
        cfg.arena.layout, cfg.meta.seed, len(obstacles), len(rover_starts))
    return layout
