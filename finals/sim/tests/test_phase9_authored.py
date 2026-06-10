"""Phase 9 acceptance test — the authored map layout (headless).

arena.layout: authored (the new DEFAULT) builds the fixed map from the
reference images: crate clusters + archway at the configured positions
(deterministic, no RNG), ~5 pads with valid/designated flags, and the drone
starts clustered at the SW entrance. The procedural generator stays working
behind the switch (its own suite lives in test_phase1_world.py).
"""

import copy
import math

import pytest

from simcore import arena
from simcore.config import load_config
from simcore.registry import SimRegistry

ENTRANCE_NE = (0.5, 0.5)  # SW corner — where the convoy will enter (Phase 10)


@pytest.fixture()
def cfg():
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0
    c.camera.use_egl = False
    c.scoring.enabled = False
    return c


def _expected_obstacle_count(cfg) -> int:
    # one obstacle per authored structure entry (gates = 2 posts each)
    return len(cfg.arena.authored.clusters)


# --------------------------------------------------------------------------- #
# Authored layout: fixed, deterministic, seed-independent
# --------------------------------------------------------------------------- #

def test_authored_is_default_and_seed_independent(cfg):
    assert cfg.arena.layout == "authored"  # the new default
    layout_a = arena.generate(cfg)
    layout_b = arena.generate(cfg)
    assert layout_a == layout_b            # fully deterministic

    cfg2 = copy.deepcopy(cfg)
    cfg2.meta.seed = 1234                  # seed must NOT move the authored map
    layout_c = arena.generate(cfg2)
    assert layout_c.obstacles == layout_a.obstacles
    assert layout_c.drone_starts == layout_a.drone_starts


def test_authored_clusters_at_configured_positions(cfg):
    layout = arena.generate(cfg)
    au = cfg.arena.authored
    assert len(layout.obstacles) == _expected_obstacle_count(cfg)

    # each authored cluster entry is ONE crate at its configured center/size/height
    for cl, o in zip(au.clusters, layout.obstacles[:len(au.clusters)]):
        assert o.north == pytest.approx(cl.center[0])
        assert o.east == pytest.approx(cl.center[1])
        assert o.half_n == pytest.approx(cl.size[0] / 2)
        assert o.half_e == pytest.approx(cl.size[1] / 2)
        assert o.height_m == pytest.approx(float(cl.height))
        assert o.z0_m == 0.0               # crates sit on the floor


def test_arch_gates_are_post_pairs_on_the_floor(cfg):
    # the 4 arch GATES are modelled as 8 solid posts (two cluster entries each),
    # all ground boxes (z0 = 0) of the gate height — no elevated lintel body.
    layout = arena.generate(cfg)
    posts = layout.obstacles[:8]           # the gate posts lead the authored list
    assert all(o.z0_m == 0.0 for o in layout.obstacles)
    assert all(o.height_m == pytest.approx(1.10) for o in posts)
    # each pair shares a northing and is ~0.9 m apart along easting (open gap)
    for a, b in zip(posts[0::2], posts[1::2]):
        assert a.north == pytest.approx(b.north)
        assert abs(a.east - b.east) == pytest.approx(0.74, abs=0.01)


def test_pads_flags_and_positions(cfg):
    pads = cfg.pads
    assert len(pads) == 5
    flags = {p.id: (p.valid, p.designated) for p in pads}
    assert flags[11] == (True, True)
    assert flags[45] == (True, True)
    assert flags[51] == (True, True)
    assert flags[67] == (False, False)     # invalid zone
    assert flags[101] == (False, False)    # invalid zone
    assert sum(1 for p in pads if p.valid) == 3   # PROVISIONAL 3 valid / 2 invalid
    assert sum(1 for p in pads if p.valid and p.designated) == 3
    assert len({p.id for p in pads}) == 5


def test_drone_starts_clustered_at_sw_entrance(cfg):
    starts = [tuple(u.start) for u in cfg.drones.units]
    # all at the SW entrance (same spot the convoy enters)
    for n, e in starts:
        assert math.dist((n, e), ENTRANCE_NE) < 1.0
    # spaced ~0.5 m apart so they can fan out to their pads
    gaps = [math.dist(a, b) for i, a in enumerate(starts)
            for b in starts[i + 1:]]
    assert all(0.4 <= g <= 0.8 for g in gaps)


def test_authored_keeps_pads_and_starts_clear(cfg):
    """The hand-authored coords must leave every pad landable and no crate
    on a drone start (the builder validates this too — double-check here)."""
    from simcore import aruco_assets
    layout = arena.generate(cfg)
    pad_half = cfg.aruco.pad_marker_size_m * aruco_assets.texture_scale() / 2
    for o in layout.obstacles:
        if o.z0_m > 0:
            continue  # elevated lintel
        for pad in cfg.pads:
            gap = arena.rect_gap_m(o.north, o.east, o.half_n, o.half_e,
                                   pad.north, pad.east, pad_half, pad_half)
            assert gap > 0.0, f"crate overlaps pad {pad.id}"
        for u in cfg.drones.units:
            d = arena.point_rect_dist_m(u.start[0], u.start[1],
                                        o.north, o.east, o.half_n, o.half_e)
            assert d >= 0.15, f"crate on drone start {u.start}"


def test_bad_authored_config_is_rejected(cfg):
    cfg2 = copy.deepcopy(cfg)
    cfg2.arena.authored.clusters[0].center = [cfg2.pads[0].north,
                                              cfg2.pads[0].east]  # on a pad
    with pytest.raises(ValueError):
        arena.generate(cfg2)
    cfg3 = copy.deepcopy(cfg)
    cfg3.arena.layout = "nonsense"
    with pytest.raises(ValueError):
        arena.generate(cfg3)


# --------------------------------------------------------------------------- #
# World build + the retained procedural mode
# --------------------------------------------------------------------------- #

def test_world_body_count_authored(cfg):
    reg = SimRegistry(cfg)
    try:
        expected = (1 + 4 + _expected_obstacle_count(cfg)
                    + len(cfg.drones.units) + cfg.rovers.count
                    + len(cfg.pads))
        assert reg.body_count() == reg.bodies.total == expected
        assert len(reg.bodies.obstacles) == _expected_obstacle_count(cfg)
        # a ground structure body sits at half its height in the world
        o0 = arena.generate(cfg).obstacles[0]
        import pybullet as p
        pos, _ = reg.run_on_sim_thread(
            lambda: p.getBasePositionAndOrientation(
                reg.bodies.obstacles[0], physicsClientId=reg.client))
        assert pos[2] == pytest.approx(o0.height_m / 2, abs=1e-6)
    finally:
        reg.shutdown()


def test_procedural_retained_behind_the_switch(cfg):
    cfg2 = copy.deepcopy(cfg)
    cfg2.arena.layout = "procedural"
    layout = arena.generate(cfg2)
    assert len(layout.obstacles) == cfg2.arena.obstacles.count
    assert all(o.z0_m == 0.0 for o in layout.obstacles)
    assert arena.generate(cfg2) == layout  # same seed => identical
    cfg3 = copy.deepcopy(cfg2)
    cfg3.meta.seed = 1234
    assert arena.generate(cfg3).obstacles != layout.obstacles  # seed matters
