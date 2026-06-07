"""Phase 1 acceptance test — PyBullet world + procedural arena (headless).

Checks: body counts match config; same seed => identical layout/world,
different seed => different; the sim thread advances time at the configured
real-time factor; generation honours the clearance/keepclear config.
"""

import copy
import time

import pytest

from simcore import arena
from simcore.config import load_config
from simcore.registry import SimRegistry

CONFIG_PATH = "sim_config.yaml"


@pytest.fixture()
def cfg():
    c = load_config(CONFIG_PATH)
    # This file tests the PROCEDURAL generator (the authored fixed map has
    # its own suite in test_phase9_authored.py).
    c.arena.layout = "procedural"
    return c


@pytest.fixture()
def registry(cfg):
    reg = SimRegistry(cfg)
    yield reg
    reg.shutdown()


# --------------------------------------------------------------------------- #
# Procedural arena (pure, no PyBullet)
# --------------------------------------------------------------------------- #

def test_layout_counts(cfg):
    layout = arena.generate(cfg)
    assert len(layout.walls) == 4
    assert len(layout.obstacles) == cfg.arena.obstacles.count
    assert len(layout.drone_starts) == len(cfg.drones.units) == 3
    assert len(layout.rover_starts) == cfg.rovers.count == 5


def test_layout_honours_clearance_config(cfg):
    layout = arena.generate(cfg)
    ob = cfg.arena.obstacles
    obstacles = layout.obstacles

    for o in obstacles:
        # inside the room with the configured wall clearance
        assert o.north - o.half_n >= ob.min_clearance_m - 1e-9
        assert o.north + o.half_n <= cfg.arena.length_m - ob.min_clearance_m + 1e-9
        assert o.east - o.half_e >= ob.min_clearance_m - 1e-9
        assert o.east + o.half_e <= cfg.arena.width_m - ob.min_clearance_m + 1e-9
        assert ob.footprint_min_m - 1e-9 <= 2 * o.half_n <= ob.footprint_max_m + 1e-9
        assert ob.height_min_m - 1e-9 <= o.height_m <= ob.height_max_m + 1e-9

    # pairwise clearance between obstacles
    for i, a in enumerate(obstacles):
        for b in obstacles[i + 1:]:
            gap = arena.rect_gap_m(a.north, a.east, a.half_n, a.half_e,
                                   b.north, b.east, b.half_n, b.half_e)
            assert gap >= ob.min_clearance_m - 1e-9

    # keepclear around pads and drone starts
    keepout = ([(p.north, p.east) for p in cfg.pads]
               + [(d.start[0], d.start[1]) for d in cfg.drones.units])
    for o in obstacles:
        for kn, ke in keepout:
            d = arena.point_rect_dist_m(kn, ke, o.north, o.east,
                                        o.half_n, o.half_e)
            assert d >= ob.keepclear_radius_m - 1e-9

    # rovers spawn inside the patrol bounds
    n_lo, n_hi = cfg.rovers.patrol.bounds_north
    e_lo, e_hi = cfg.rovers.patrol.bounds_east
    for r in layout.rover_starts:
        assert n_lo <= r.north <= n_hi
        assert e_lo <= r.east <= e_hi


def test_same_seed_identical_layout(cfg):
    assert arena.generate(cfg) == arena.generate(cfg)


def test_different_seed_different_layout(cfg):
    cfg2 = copy.deepcopy(cfg)
    cfg2.meta.seed = 1234
    a = arena.generate(cfg)
    b = arena.generate(cfg2)
    assert a.obstacles != b.obstacles
    assert a.rover_starts != b.rover_starts


# --------------------------------------------------------------------------- #
# PyBullet world (booted on the sim thread)
# --------------------------------------------------------------------------- #

def test_world_body_counts(cfg, registry):
    b = registry.bodies
    assert len(b.walls) == 4
    assert len(b.obstacles) == cfg.arena.obstacles.count
    assert len(b.drones) == len(cfg.drones.units) == 3
    assert len(b.rovers) == cfg.rovers.count == 5
    assert len(b.pads) == len(cfg.pads) == 5
    expected = (1 + 4 + cfg.arena.obstacles.count + 3 + cfg.rovers.count
                + len(cfg.pads))
    assert registry.body_count() == b.total == expected


def _static_poses(snapshot):
    """Rovers patrol on their own (Phase 6+), so their live poses depend on
    when the snapshot lands; their START determinism is asserted via the
    layout. Everything else must match bit-for-bit."""
    return [s for s in snapshot if s[0] != "rover"]


def test_same_seed_identical_world(cfg):
    reg1 = SimRegistry(cfg)
    try:
        snap1 = reg1.snapshot_body_poses()
        layout1 = reg1.layout
    finally:
        reg1.shutdown()
    reg2 = SimRegistry(copy.deepcopy(cfg))
    try:
        snap2 = reg2.snapshot_body_poses()
        layout2 = reg2.layout
    finally:
        reg2.shutdown()
    assert layout1 == layout2  # includes identical rover/drone starts
    assert _static_poses(snap1) == _static_poses(snap2)


def test_different_seed_different_world(cfg):
    cfg2 = copy.deepcopy(cfg)
    cfg2.meta.seed = 1234
    reg1 = SimRegistry(cfg)
    try:
        snap1 = reg1.snapshot_body_poses()
    finally:
        reg1.shutdown()
    reg2 = SimRegistry(cfg2)
    try:
        snap2 = reg2.snapshot_body_poses()
    finally:
        reg2.shutdown()
    assert _static_poses(snap1) != _static_poses(snap2)


# --------------------------------------------------------------------------- #
# Sim thread + clock
# --------------------------------------------------------------------------- #

def test_sim_thread_advances_time(registry):
    t0 = registry.sim_time()
    time.sleep(0.6)
    delta = registry.sim_time() - t0
    # rtf = 1.0 in config: ~0.6 s of sim time; generous CI bounds
    assert 0.25 <= delta <= 1.2


def test_real_time_factor_speeds_up_sim(cfg):
    cfg2 = copy.deepcopy(cfg)
    cfg2.meta.real_time_factor = 5.0
    reg = SimRegistry(cfg2)
    try:
        t0 = reg.sim_time()
        time.sleep(0.5)
        delta = reg.sim_time() - t0
    finally:
        reg.shutdown()
    # expect ~2.5 s of sim time in 0.5 s of wall time
    assert delta >= 1.2


def test_sim_time_stops_after_shutdown(cfg):
    reg = SimRegistry(cfg)
    reg.shutdown()
    t0 = reg.sim_time()
    time.sleep(0.2)
    assert reg.sim_time() == t0
    with pytest.raises(RuntimeError):
        reg.run_on_sim_thread(lambda: None)
