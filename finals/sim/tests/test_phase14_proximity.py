"""Phase 14 acceptance test — observer-only obstacle proximity (headless).

The proximity DISTANCES are OBSERVER-ONLY ground truth from the barrier rays
(sensors.barrier_distances / DebugProbe), kept for the read-only dashboard.
The public pyhulax surface still returns only the five blocked/clear
booleans — pinned here. (Phase 20 simplified the VIZ overlay itself to
boolean directional indicators; that is covered in test_phase20_dashboard.)
"""

import dataclasses
import time

import pybullet as p
import pytest

from pyhulax import DroneAPI
from pyhulax.core import Direction, Obstacles

from simcore import frames, sensors
from simcore.config import load_config
from simcore.debug import DebugProbe
from simcore.registry import get_registry, shutdown_registry
from simcore.viz import TopDownView


@pytest.fixture()
def cfg():
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0
    c.camera.use_egl = False
    c.scoring.enabled = False
    c.motion.realistic = False  # crisp snap motion: exact geometry under test
    c.arena.layout = "procedural"  # controlled arena: we spawn the obstacle
    c.arena.obstacles.count = 0
    c.rovers.count = 0
    return c


@pytest.fixture()
def sim(cfg):
    reg = get_registry(cfg)
    yield reg
    shutdown_registry()


def _spawn_box(reg, north, east, half=0.2, height=2.0):
    def _build():
        pos = frames.arena_to_world(reg.config, north, east, height / 2)
        col = p.createCollisionShape(p.GEOM_BOX,
                                     halfExtents=[half, half, height / 2],
                                     physicsClientId=reg.client)
        return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                                 basePosition=pos,
                                 physicsClientId=reg.client)
    return reg.run_on_sim_thread(_build)


def _distances(reg, index=0):
    drone = reg.drones[index]
    return reg.run_on_sim_thread(
        lambda: sensors.barrier_distances(reg.client, reg.config, drone))


# --------------------------------------------------------------------------- #
# Observer-only distances from the actual rays (dashboard / DebugProbe)
# --------------------------------------------------------------------------- #

def test_forward_distance_and_approach(sim, cfg):
    # Box south face at north = 2.4; drone 0 flies north along east = 1.1.
    # Forward trigger ray: starts 0.10 m ahead of centre, ends at 0.6 m.
    _spawn_box(sim, north=2.6, east=1.1)
    d = DroneAPI()
    d.connect(cfg.drones.units[0].ip)
    d.takeoff(100)

    d.move_to(0, 130, 100)              # centre at n=1.9: face gap 0.5 m
    prox = _distances(sim)
    fwd = prox["forward"]
    assert fwd["range_m"] == pytest.approx(0.5, abs=1e-6)   # 0.6 - 0.1 offset
    assert fwd["distance_m"] == pytest.approx(0.40, abs=0.03)  # ray-true
    # clear directions report no hit
    for name in ("back", "left", "right", "down"):
        assert prox[name]["distance_m"] is None

    d.move_to(0, 156, 100)              # n=2.16: face gap 0.24 m -> closing in
    fwd2 = _distances(sim)["forward"]
    assert fwd2["distance_m"] == pytest.approx(0.14, abs=0.03)
    assert fwd2["distance_m"] < fwd["distance_m"]            # updates live

    # the value really is the ray's: cross-check against the probe's ray data
    ray = DebugProbe(sim).snapshot(referee_view=False)["drones"][0][
        "sensors"]["rays"]["forward"]
    assert ray["distance_m"] == pytest.approx(fwd2["distance_m"], abs=0.02)
    assert ray["hit_pos"] is not None


# --------------------------------------------------------------------------- #
# The boundary: pyhulax still booleans only
# --------------------------------------------------------------------------- #

def test_public_api_still_booleans_only(sim, cfg):
    _spawn_box(sim, north=1.0, east=1.1)   # face 0.2 m ahead: inside range
    d = DroneAPI()
    d.connect(cfg.drones.units[0].ip)
    d.takeoff(100)
    ob = d.get_obstacles()
    assert isinstance(ob, Obstacles)
    field_names = [f.name for f in dataclasses.fields(Obstacles)]
    assert field_names == ["forward", "back", "left", "right", "down"]
    assert all(isinstance(getattr(ob, f), bool) for f in field_names)
    assert ob.forward is True              # detected... but boolean ONLY

    import pyhulax
    import pyhulax.core
    public = (set(dir(pyhulax)) | set(dir(pyhulax.DroneAPI))
              | set(dir(pyhulax.core)) | set(dir(Obstacles)))
    leaked = [n for n in public
              if "proximity" in n.lower() or "distance" in n.lower()
              or "range" in n.lower()]
    assert leaked == [], f"proximity leaked into the public API: {leaked}"


# --------------------------------------------------------------------------- #
# Boolean overlay renders headless (Phase 20 simplified shape)
# --------------------------------------------------------------------------- #

def test_headless_png_with_boolean_proximity_overlay(sim, cfg, tmp_path):
    assert cfg.viz.show_proximity is True
    _spawn_box(sim, north=2.0, east=1.1)
    d = DroneAPI()
    d.connect(cfg.drones.units[0].ip)
    d.takeoff(100)
    d.move_to(0, 90, 100)                  # n=1.5: face gap 0.3 -> forward set
    view = TopDownView(sim)
    view.sample()
    prox = view._latest[0][7]
    assert prox is not None
    # the overlay sample is now BOOLEAN directional — no distance/angle
    assert set(prox) == {"forward", "back", "left", "right", "down"}
    assert all(isinstance(v, bool) for v in prox.values())
    assert prox["forward"] is True
    out = tmp_path / "prox.png"
    view.render_png(str(out))
    assert out.is_file() and out.stat().st_size > 10_000
