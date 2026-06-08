"""Phase 2 acceptance test — blocking pyhulax control commands (headless).

Asserts against the TRUE world pose from the sim (registry.drone_world_pose),
NOT get_position() — the drifting estimate is Phase 3.

Frame guards: move() is body-relative to the CURRENT heading; move_to() is
the takeoff-origin frame (x=right, y=forward, z=up, cm) frozen at takeoff —
it must NOT rotate with the drone and must respect the takeoff heading.
"""

import copy
import math
import time

import pytest

from pyhulax import DroneAPI
from pyhulax.core import Direction
from pyhulax.exceptions import LowBattery, NotReady

from simcore.config import load_config
from simcore.registry import get_registry, shutdown_registry

TOL = 0.02  # m — kinematics snap to target, so this is generous


@pytest.fixture()
def cfg():
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0  # sim-time semantics identical, tests faster
    c.camera.use_egl = False        # no rendering needed in this phase
    c.scoring.enabled = False       # referee not under test here
    c.motion.realistic = False  # crisp snap motion: exact geometry under test
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


# Drone 0 starts at arena (north=0.6, east=1.1), heading 0 (facing +north)
# => world (x=1.1, y=0.6), nose along +y.
START_W = (1.1, 0.6)


# --------------------------------------------------------------------------- #
# Core flight commands
# --------------------------------------------------------------------------- #

def test_takeoff_reaches_height(sim, cfg):
    d = _connect(cfg)
    r = d.takeoff(100)
    assert r and r.success
    pos, _ = sim.drone_world_pose(0)
    assert abs(pos[2] - 1.0) < TOL  # true height ~1.0 m
    assert sim.run_on_sim_thread(lambda: sim.drones[0].flying) is True


def test_move_forward_along_heading(sim, cfg):
    d = _connect(cfg)
    d.takeoff(100)
    p0, _ = sim.drone_world_pose(0)
    r = d.move(Direction.FORWARD, 100)
    assert r.success
    p1, _ = sim.drone_world_pose(0)
    disp = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    # heading 0 = +north = world +y
    assert abs(disp[0]) < TOL and abs(disp[1] - 1.0) < TOL and abs(disp[2]) < TOL


def test_rotate_then_move_uses_new_heading(sim, cfg):
    d = _connect(cfg)
    d.takeoff(100)
    _, yaw0 = sim.drone_world_pose(0)
    r = d.rotate(90)  # +90 = CCW: north -> west
    assert r.success
    p0, yaw1 = sim.drone_world_pose(0)
    assert math.isclose(yaw1 - yaw0, math.pi / 2, abs_tol=1e-6)
    d.move(Direction.FORWARD, 100)
    p1, _ = sim.drone_world_pose(0)
    disp = (p1[0] - p0[0], p1[1] - p0[1])
    # west = world -x
    assert abs(disp[0] + 1.0) < TOL and abs(disp[1]) < TOL


def test_land_grounds_the_drone(sim, cfg):
    d = _connect(cfg)
    d.takeoff(100)
    d.move(Direction.FORWARD, 50)
    r = d.land()
    assert r.success
    pos, _ = sim.drone_world_pose(0)
    hz = cfg.bodies.drone_half_extents_m[2]
    assert abs(pos[2] - hz) < TOL
    assert sim.run_on_sim_thread(lambda: sim.drones[0].flying) is False
    with pytest.raises(NotReady):  # grounded again => motion needs takeoff
        d.move(Direction.FORWARD, 50)


def test_hover_holds_position_for_duration(sim, cfg):
    d = _connect(cfg)
    d.takeoff(100)
    p0, _ = sim.drone_world_pose(0)
    t0 = sim.sim_time()
    r = d.hover(1.0)
    assert r.success
    elapsed = sim.sim_time() - t0
    assert 1.0 <= elapsed < 2.0  # sim-time duration honoured
    p1, _ = sim.drone_world_pose(0)
    assert math.dist(p0, p1) < 1e-6


# --------------------------------------------------------------------------- #
# move_to frame correctness (the guard)
# --------------------------------------------------------------------------- #

def test_move_to_world_geometry(sim, cfg):
    """Takeoff at world (1.1, 0.6) facing north: right=east(+x), forward=
    north(+y). move_to(50,100,150)cm must land at world (1.6, 1.6, 1.5) m."""
    d = _connect(cfg)
    d.takeoff(100)
    r = d.move_to(50, 100, 150)
    assert r.success
    pos, _ = sim.drone_world_pose(0)
    assert abs(pos[0] - (START_W[0] + 0.5)) < TOL
    assert abs(pos[1] - (START_W[1] + 1.0)) < TOL
    assert abs(pos[2] - 1.5) < TOL


def test_move_to_frame_fixed_after_rotate(sim, cfg):
    """The takeoff frame must NOT rotate with the drone: yawing 90° first
    must not change where move_to(50,100,150) goes."""
    d = _connect(cfg)
    d.takeoff(100)
    d.rotate(90)
    r = d.move_to(50, 100, 150)
    assert r.success
    pos, _ = sim.drone_world_pose(0)
    assert abs(pos[0] - (START_W[0] + 0.5)) < TOL  # same target as un-rotated
    assert abs(pos[1] - (START_W[1] + 1.0)) < TOL
    assert abs(pos[2] - 1.5) < TOL


def test_move_to_respects_takeoff_heading(cfg):
    """Takeoff facing WEST (heading 90, CCW from north): forward=west,
    right=north. move_to(100,0,100) => 1 m NORTH of the start."""
    cfg2 = copy.deepcopy(cfg)
    cfg2.drones.units[0].heading_deg = 90.0
    reg = get_registry(cfg2)
    try:
        d = _connect(cfg2)
        d.takeoff(100)
        r = d.move_to(100, 0, 100)
        assert r.success
        pos, _ = reg.drone_world_pose(0)
        assert abs(pos[0] - START_W[0]) < TOL
        assert abs(pos[1] - (START_W[1] + 1.0)) < TOL
        assert abs(pos[2] - 1.0) < TOL
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Blocking semantics
# --------------------------------------------------------------------------- #

def test_nonblocking_returns_promptly_and_motion_continues(sim, cfg):
    d = _connect(cfg)
    d.takeoff(100)
    p0, _ = sim.drone_world_pose(0)
    t_wall = time.perf_counter()
    r = d.move(Direction.FORWARD, 200, blocking=False)
    call_s = time.perf_counter() - t_wall
    assert r and r.success            # accepted immediately
    assert call_s < 0.5               # returned long before the ~2 s sim move
    p_now, _ = sim.drone_world_pose(0)
    assert p_now[1] - p0[1] < 1.9     # motion not finished at return
    deadline = time.time() + 10
    while time.time() < deadline:     # ...but it continues to completion
        pos, _ = sim.drone_world_pose(0)
        if pos[1] - p0[1] >= 2.0 - TOL:
            break
        time.sleep(0.02)
    else:
        pytest.fail("non-blocking move never completed")


# --------------------------------------------------------------------------- #
# NotReady / LowBattery / battery model
# --------------------------------------------------------------------------- #

def test_not_ready(sim, cfg):
    unconnected = DroneAPI()
    with pytest.raises(NotReady):
        unconnected.takeoff(100)
    d = _connect(cfg)
    with pytest.raises(NotReady):     # connected but not flying
        d.move(Direction.FORWARD, 100)


def test_low_battery_blocks_motion_but_not_land(sim, cfg):
    d = _connect(cfg)
    d.takeoff(100)
    drone = sim.drones[0]
    sim.run_on_sim_thread(lambda: setattr(drone, "battery_pct", 5.0))
    assert d.get_battery() == 5
    with pytest.raises(LowBattery):
        d.move(Direction.FORWARD, 100)
    with pytest.raises(LowBattery):
        d.rotate(90)
    r = d.land()                      # landing must always be allowed
    assert r.success


def test_battery_drains_while_flying(sim, cfg):
    d = _connect(cfg)
    start = cfg.drones.battery.start_pct
    assert d.get_battery() == start   # grounded: no drain yet
    d.takeoff(100)
    d.hover(2.0)
    level = sim.run_on_sim_thread(lambda: sim.drones[0].battery_pct)
    assert level < start              # linear drain accumulated in sim time
    assert d.get_battery() <= start
