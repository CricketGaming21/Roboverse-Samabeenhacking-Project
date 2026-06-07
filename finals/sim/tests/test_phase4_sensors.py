"""Phase 4 acceptance test — barrier sensors + reflex avoidance (headless).

Uses a CONTROLLED arena (no random obstacles, no rovers) and spawns boxes at
known positions, so trigger ranges and reflex geometry are exact.

Coarse by design: only the five booleans are checked — there is no distance,
point cloud, map, or planner anywhere in this phase.
"""

import logging
import time

import pybullet as p
import pytest

from pyhulax import DroneAPI
from pyhulax.core import BarrierMask, Direction, Obstacles
from pyhulax.exceptions import NotReady

from simcore import frames
from simcore.config import load_config
from simcore.registry import get_registry, shutdown_registry


@pytest.fixture()
def cfg():
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0
    c.scoring.enabled = False  # referee not under test here
    c.camera.use_egl = False
    c.arena.obstacles.count = 0   # controlled arena: we spawn our own boxes
    c.rovers.count = 0
    return c


@pytest.fixture()
def sim(cfg):
    reg = get_registry(cfg)
    yield reg
    shutdown_registry()


def _connect(cfg, index=0) -> DroneAPI:
    d = DroneAPI()
    d.connect(cfg.drones.units[index].ip)
    return d


def _spawn_box(reg, north, east, half_n=0.2, half_e=0.2, height=2.0) -> int:
    """Static box at arena (north, east), grounded, built on the sim thread."""
    cfg = reg.config

    def _build():
        pos = frames.arena_to_world(cfg, north, east, height / 2)
        col = p.createCollisionShape(p.GEOM_BOX,
                                     halfExtents=[half_e, half_n, height / 2],
                                     physicsClientId=reg.client)
        return p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                                 basePosition=pos,
                                 physicsClientId=reg.client)
    return reg.run_on_sim_thread(_build)


def _north_of(reg, index=0) -> float:
    pos, _ = reg.drone_world_pose(index)
    return frames.world_to_arena(reg.config, pos[0], pos[1])[0]


def _wait_goal_done(reg, index=0, timeout_s=5.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if reg.run_on_sim_thread(lambda: reg.drones[index].goal is None):
            return
        time.sleep(0.01)
    pytest.fail("drone goal did not complete in time")


# --------------------------------------------------------------------------- #
# Flags + bitmask at the configured range
# --------------------------------------------------------------------------- #

def test_forward_flag_trips_at_configured_range(sim, cfg):
    # Box south face at north = 2.4; drone flies north along east = 1.5.
    _spawn_box(sim, north=2.6, east=1.5)
    d = _connect(cfg)
    d.takeoff(100)
    assert d.get_obstacles() == Obstacles()      # gap 1.8 m: all clear

    d.move(Direction.FORWARD, 110)               # north = 1.7 -> gap 0.7 m
    assert d.get_obstacles().forward is False    # still beyond range_m 0.6

    d.move(Direction.FORWARD, 20)                # north = 1.9 -> gap 0.5 m
    ob = d.get_obstacles()
    assert ob.forward is True                    # inside the configured range
    assert (ob.back, ob.left, ob.right, ob.down) == (False, False, False, False)
    assert d.any_obstacle() is True
    status = d.get_drone_status()
    assert status & 0b00001                      # bit 0 = forward
    assert status & 0b11110 == 0                 # no other bits


def test_min_altitude_gate_and_down_bit(sim, cfg):
    # Box very close ahead (south face at north = 1.1, gap 0.5 m from start).
    _spawn_box(sim, north=1.3, east=1.5)
    d = _connect(cfg)
    assert d.get_obstacles() == Obstacles()      # grounded: gated, all clear

    d.takeoff(25)                                # below the 0.35 m gate
    assert d.get_obstacles() == Obstacles()      # box in range, but gated
    assert d.get_drone_status() == 0

    d.move(Direction.UP, 13)                     # ~0.38 m: gate passed
    ob = d.get_obstacles()
    assert ob.forward is True                    # box now seen
    assert ob.down is True                       # floor within down range 0.4
    assert d.get_drone_status() & 0b10000        # bit 4 = down

    d.move(Direction.UP, 62)                     # ~1.0 m: floor out of range
    ob = d.get_obstacles()
    assert ob.down is False and ob.forward is True


def test_altitude_ray_reads_obstacle_top(sim, cfg):
    # Flat box (top at 0.5 m) under drone 1's path along east = 3.0.
    _spawn_box(sim, north=2.0, east=3.0, half_n=0.5, half_e=0.5, height=0.5)
    d = _connect(cfg, index=1)
    d.takeoff(150)
    assert abs(d.get_altitude() - 150.0) < 3.0   # over the floor
    d.move(Direction.FORWARD, 140)               # north = 2.0: over the box
    assert abs(d.get_altitude() - 100.0) < 3.0   # 1.5 - 0.5 box top
    d.move(Direction.FORWARD, 150)               # north = 3.5: past the box
    assert abs(d.get_altitude() - 150.0) < 3.0   # floor again


# --------------------------------------------------------------------------- #
# Reflexes — both routed through the committing executor
# --------------------------------------------------------------------------- #

def test_avoidance_reflex_steps_back_and_logs_preemption(sim, cfg, caplog):
    _spawn_box(sim, north=2.6, east=1.5)         # south face at 2.4
    d = _connect(cfg)
    d.takeoff(100)
    r = d.set_avoidance_direction(Direction.BACK, 30, BarrierMask.FRONT)
    assert r.success

    root = logging.getLogger("hulasim")
    old_propagate = root.propagate
    root.propagate = True  # let caplog's root handler see sim records
    try:
        with caplog.at_level(logging.WARNING, logger="hulasim"):
            res = d.move(Direction.FORWARD, 300)  # would reach the box
            _wait_goal_done(sim)                  # let the reflex step finish
    finally:
        root.propagate = old_propagate

    # The move was preempted by the reflex, not completed.
    assert not res.success
    assert "preempted by avoid_step" in res.message
    msgs = [rec.getMessage() for rec in caplog.records]
    assert any("avoidance reflex tripped" in m for m in msgs)
    assert any("preempted by avoid_step" in m for m in msgs)

    # Geometry: front tripped at gap ~0.6 (north ~1.8), stepped BACK 30 cm.
    north = _north_of(sim)
    assert abs(north - 1.5) < 0.05
    assert d.get_obstacles().forward is False     # step cleared the sensor


def test_avoidance_only_fires_on_actual_detection(sim, cfg):
    d = _connect(cfg)
    d.takeoff(100)
    d.set_avoidance_direction(Direction.BACK, 30, BarrierMask.FRONT)
    p0, _ = sim.drone_world_pose(0)
    r = d.move(Direction.FORWARD, 100)            # empty arena ahead
    assert r.success                              # nothing tripped, no reflex
    p1, _ = sim.drone_world_pose(0)
    assert abs((p1[1] - p0[1]) - 1.0) < 0.02


def test_barrier_mode_stops_short_of_wall(sim, cfg):
    d = _connect(cfg)
    d.takeoff(100)
    assert d.set_barrier_mode(True).success
    r = d.move(Direction.FORWARD, 1500)           # far past the wall at n=10
    assert not r.success
    assert "barrier" in r.message.lower()         # stopped short, goal failed
    north = _north_of(sim)
    assert 9.2 < north < 9.55                     # halted ~range short of wall
    hull_front = north + cfg.bodies.drone_half_extents_m[0]
    assert hull_front < 10.0                      # no penetration

    assert d.set_barrier_mode(False).success      # disable: flies right past
    r2 = d.move(Direction.BACK, 100)
    assert r2.success


def test_barrier_mode_never_blocks_landing(sim, cfg):
    d = _connect(cfg)
    d.takeoff(100)
    d.set_barrier_mode(True)
    r = d.land()                                  # down sensor must not stop it
    assert r.success
    pos, _ = sim.drone_world_pose(0)
    assert abs(pos[2] - cfg.bodies.drone_half_extents_m[2]) < 0.02


# --------------------------------------------------------------------------- #
# API edges
# --------------------------------------------------------------------------- #

def test_obstacle_api_before_connect(sim):
    d = DroneAPI()
    assert d.get_obstacles() == Obstacles()       # empty if no data
    assert d.any_obstacle() is False
    assert d.get_drone_status() is None
    with pytest.raises(NotReady):
        d.set_barrier_mode(True)
