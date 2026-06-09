"""Phase 31 acceptance test — real arena (fully config-driven, mission-readable)
+ the 0.5 m/s HARD speed cap (headless).

Invariants: arena loads from config and validates; the emitted ground-truth
YAML matches arena.authored exactly; NO velocity level maps above 0.5 m/s; the
public pyhulax API surface (enum names/values, signatures) is unchanged.
"""

import copy
import inspect

import pytest
import yaml

from pyhulax import DroneAPI
from pyhulax.core import VelocityLevel

from simcore import arena
from simcore.arena_truth import build_arena_truth, write_arena_truth
from simcore.config import load_config
from simcore.drone_model import level_mps, speed_to_mps


@pytest.fixture()
def cfg():
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0
    c.camera.use_egl = False
    c.scoring.enabled = False
    return c


# --------------------------------------------------------------------------- #
# Arena is fully config-driven: loads, builds, validates
# --------------------------------------------------------------------------- #

def test_authored_arena_is_data_driven_per_crate(cfg):
    au = cfg.arena.authored
    assert cfg.arena.layout == "authored"            # still the default
    # every cluster entry is a data-driven crate: center / size / height
    for cl in au.clusters:
        assert len(cl.center) == 2 and len(cl.size) == 2
        assert cl.height > 0.0
    layout = arena.generate(cfg)
    # one obstacle per crate + 3 archway parts, each at its configured footprint
    assert len(layout.obstacles) == len(au.clusters) + 3
    for cl, o in zip(au.clusters, layout.obstacles[:len(au.clusters)]):
        assert (o.north, o.east) == pytest.approx(tuple(cl.center))
        assert o.half_n == pytest.approx(cl.size[0] / 2)
        assert o.half_e == pytest.approx(cl.size[1] / 2)
        assert o.height_m == pytest.approx(cl.height)
    # dimensions come straight from config
    assert layout.length_m == cfg.arena.length_m
    assert layout.width_m == cfg.arena.width_m


def test_authored_arena_validates_bad_edits(cfg):
    # a crate dropped onto a pad is rejected (validation still fires)
    bad_pad = copy.deepcopy(cfg)
    bad_pad.arena.authored.clusters[0].center = [bad_pad.pads[0].north,
                                                 bad_pad.pads[0].east]
    with pytest.raises(ValueError):
        arena.generate(bad_pad)
    # a crate out of bounds is rejected
    oob = copy.deepcopy(cfg)
    oob.arena.authored.clusters[0].center = [cfg.arena.length_m + 5, 3.0]
    with pytest.raises(ValueError):
        arena.generate(oob)
    # a non-positive size is rejected
    bad_size = copy.deepcopy(cfg)
    bad_size.arena.authored.clusters[0].size = [0.0, 0.45]
    with pytest.raises(ValueError):
        arena.generate(bad_size)


# --------------------------------------------------------------------------- #
# Arena exposed as plain data for the mission (no simcore import needed)
# --------------------------------------------------------------------------- #

def test_emitted_arena_truth_matches_authored_config(cfg, tmp_path):
    data = build_arena_truth(cfg)
    au = cfg.arena.authored
    # arena dimensions
    assert data["arena"] == {"length_m": cfg.arena.length_m,
                             "width_m": cfg.arena.width_m,
                             "height_m": cfg.arena.height_m}
    # one crate per authored cluster, center/size/height verbatim
    assert len(data["crates"]) == len(au.clusters)
    for crate, cl in zip(data["crates"], au.clusters):
        assert crate["center"] == [cl.center[0], cl.center[1]]
        assert crate["size"] == [cl.size[0], cl.size[1]]
        assert crate["height"] == cl.height
    # archway
    assert data["archway"]["corner"] == [au.archway.corner[0],
                                         au.archway.corner[1]]
    assert data["archway"]["width_m"] == au.archway.width_m
    assert data["archway"]["height_m"] == au.archway.height_m

    # round-trips through a plain YAML file the mission loads WITHOUT simcore
    out = tmp_path / "arena_truth.yaml"
    write_arena_truth(cfg, str(out))
    reloaded = yaml.safe_load(out.read_text())
    assert reloaded == data


def test_committed_arena_truth_is_in_sync():
    # the checked-in arena_truth.yaml must match the default config's emit
    cfg = load_config("sim_config.yaml")
    with open("arena_truth.yaml") as f:
        committed = yaml.safe_load(f)
    assert committed == build_arena_truth(cfg)


def test_arena_truth_is_simcore_free_plain_data():
    # the emitted file is PLAIN data: no python object tags, safe_load-able
    # into nothing but dict/list/scalar (so the mission needs no simcore).
    with open("arena_truth.yaml") as f:
        text = f.read()
    assert "!!python" not in text                    # no pickled python objects
    data = yaml.safe_load(text)
    assert set(data) == {"arena", "crates", "archway"}

    def _plain(v):
        if isinstance(v, dict):
            return all(_plain(x) for x in v.values())
        if isinstance(v, list):
            return all(_plain(x) for x in v)
        return isinstance(v, (int, float, str, bool, type(None)))
    assert _plain(data)


# --------------------------------------------------------------------------- #
# 0.5 m/s HARD cap — no velocity level maps above it
# --------------------------------------------------------------------------- #

def test_no_velocity_level_maps_above_half_mps(cfg):
    assert cfg.velocity_levels.max_mps == 0.5
    for level in VelocityLevel:
        assert speed_to_mps(cfg, level) <= 0.5 + 1e-9, \
            f"{level.name} maps above the 0.5 m/s hard cap"
    # MEDIUM is the usable max; ZOOM/TURBO clamp down to it; SLOW is below
    assert speed_to_mps(cfg, VelocityLevel.SLOW) == 0.3
    assert speed_to_mps(cfg, VelocityLevel.MEDIUM) == 0.5
    assert speed_to_mps(cfg, VelocityLevel.ZOOM) == 0.5
    assert speed_to_mps(cfg, VelocityLevel.TURBO) == 0.5
    # the helper clamps any level name
    assert level_mps(cfg, "TURBO") == 0.5
    # raising the cap would lift the levels — clamp tracks config, not hardcoded
    hi = copy.deepcopy(cfg)
    hi.velocity_levels.max_mps = 0.9
    assert speed_to_mps(hi, VelocityLevel.ZOOM) == 0.8   # now the raw band shows


def test_vertical_and_yaw_rates_not_clamped_by_horizontal_cap(cfg):
    # the 0.5 cap is the HORIZONTAL flight cap; climb/descent/yaw are separate
    assert cfg.velocity_levels.climb_mps == 1.2
    assert cfg.velocity_levels.descent_mps == 1.0


# --------------------------------------------------------------------------- #
# Public API surface unchanged
# --------------------------------------------------------------------------- #

def test_velocity_enum_and_api_surface_unchanged():
    # the enum names/values are the firmware contract — untouched by the clamp
    assert {m.name: int(m) for m in VelocityLevel} == {
        "SLOW": 300, "MEDIUM": 200, "ZOOM": 100, "TURBO": 50}
    # speed_to_mps signature unchanged (cfg, speed)
    assert list(inspect.signature(speed_to_mps).parameters) == ["cfg", "speed"]
    # no arena/crate/speed-cap leakage onto the public surface
    import pyhulax
    import pyhulax.core
    from UWBParserThread import UWBParserThread
    public = (set(dir(pyhulax)) | set(dir(pyhulax.DroneAPI))
              | set(dir(pyhulax.core)) | set(dir(UWBParserThread)))
    leaked = [n for n in public if "arena_truth" in n.lower()
              or "max_mps" in n.lower() or "level_mps" in n.lower()]
    assert leaked == []
