"""Phase 19 acceptance test — send_manual_control / manual_fly (headless).

The sim EXECUTES stick inputs through the motion model (speed band, accel/
tilt limits, barrier clamping) and contains NO PID — a constant stick yields
constant velocity, never convergence on a target. Body-relative, ~20 Hz,
distinct from (and arbitrated against) the blocking commands.
"""

import inspect
import math
import time

import numpy as np
import pybullet as p
import pytest

from pyhulax import DroneAPI
from pyhulax.core import Direction

from simcore import frames
from simcore.config import load_config
from simcore.registry import get_registry, shutdown_registry

E = inspect.Parameter.empty


def _cfg(rtf=4.0, wind=0.0):
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = rtf  # moderate: wall-paced 50 Hz frames stay fresh
    c.camera.use_egl = False
    c.scoring.enabled = False
    c.arena.layout = "procedural"
    c.arena.obstacles.count = 0
    c.rovers.count = 0
    c.motion.wind_mps = wind
    return c


def _connect(cfg, index=0) -> DroneAPI:
    d = DroneAPI()
    d.connect(cfg.drones.units[index].ip)
    return d


def _pump(d, reg, seconds_wall, **sticks):
    """Drive frames at ~50 Hz wall while sampling (t, pos, vel, tilt, yaw)."""
    drone = reg.drones[0]
    out = []
    end = time.time() + seconds_wall
    while time.time() < end:
        assert d.send_manual_control(**sticks)
        out.append(reg.run_on_sim_thread(
            lambda: (reg.clock.now(), drone.pos.copy(), drone.vel.copy(),
                     float(drone.tilt_deg), float(drone.yaw))))
        time.sleep(0.02)
    return out


# --------------------------------------------------------------------------- #
# Stick -> velocity through the band, ramped, body-relative
# --------------------------------------------------------------------------- #

def test_half_forward_ramps_to_half_band():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = _connect(cfg)
        d.takeoff(100)
        d.hover(1.0)
        samples = _pump(d, reg, 2.5, forward=0.5)     # ~10 sim s held
        speeds = [float(np.linalg.norm(v[:2])) for _t, _p, v, _ti, _y in samples]
        band = cfg.velocity_levels.TURBO
        assert speeds[0] < 0.5 * 0.5 * band           # ramped, not instant
        steady = speeds[len(speeds) // 2:]
        assert np.mean(steady) == pytest.approx(0.5 * band, rel=0.12)
        # heading north: displacement is +y (the nose direction)
        dy = samples[-1][1][1] - samples[0][1][1]
        dx = samples[-1][1][0] - samples[0][1][0]
        assert dy > 1.0 and abs(dx) < 0.2
        # tilt obeyed the limit and was active during the ramp
        tilts = [ti for _t, _p, _v, ti, _y in samples]
        assert max(abs(t) for t in tilts) <= cfg.motion.max_tilt_deg + 1e-6
        assert any(ti < -3.0 for ti in tilts[:10])
    finally:
        shutdown_registry()


def test_rotate_yaws_ccw_and_forward_is_body_relative():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = _connect(cfg)
        d.takeoff(100)
        d.hover(1.0)
        _, yaw0 = reg.drone_world_pose(0)
        # rotate stick 0.5 => ~30 deg/s CCW; hold ~12 sim s => ~+90 deg
        deadline = time.time() + 6.0
        while time.time() < deadline:
            d.send_manual_control(rotate=0.5)
            _, yaw = reg.drone_world_pose(0)
            if yaw - yaw0 >= math.pi / 2:
                break
            time.sleep(0.02)
        _, yaw1 = reg.drone_world_pose(0)
        assert yaw1 > yaw0                            # CCW = +yaw
        assert yaw1 - yaw0 >= math.pi / 3
        # now hold forward: motion follows the NEW nose
        p0, _ = reg.drone_world_pose(0)
        samples = _pump(d, reg, 1.5, forward=0.6)
        p1 = samples[-1][1]
        disp = np.array([p1[0] - p0[0], p1[1] - p0[1]])
        nose = np.array([math.cos(yaw1), math.sin(yaw1)])
        cosang = float(disp @ nose / (np.linalg.norm(disp) + 1e-9))
        assert cosang > 0.95                          # along the new nose
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Zero stick: coast to a stop, then station-keep (even in wind)
# --------------------------------------------------------------------------- #

def test_stopping_inputs_coast_then_station_keep_in_wind():
    cfg = _cfg(wind=0.15)
    reg = get_registry(cfg)
    try:
        d = _connect(cfg)
        d.takeoff(100)
        d.hover(1.0)
        moving = _pump(d, reg, 1.5, forward=0.8)      # get moving fast
        v_fly = max(float(np.linalg.norm(v[:2]))
                    for _t, _p, v, _ti, _y in moving[-10:])
        assert v_fly > 0.3                            # genuinely cruising
        # release the stick: zero frames keep arriving (the loop is alive)
        samples = _pump(d, reg, 3.0)                  # all sticks 0.0
        speeds = [float(np.linalg.norm(v[:2])) for _t, _p, v, _ti, _y in samples]
        assert max(speeds[:8]) > 0.2                  # momentum: still moving
        assert min(speeds) < 0.5 * max(speeds[:8])    # decays, no snap to zero
        assert speeds[-1] < 0.15                      # coasted to a stop
        # station-keep: once stopped, the wind cannot walk it away
        hold0 = samples[-1][1]
        more = _pump(d, reg, 2.5)
        drift = max(math.dist(s[1][:2], hold0[:2]) for s in more)
        assert drift < 0.35
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Barrier clamping: a stick into a tripped flag cannot penetrate
# --------------------------------------------------------------------------- #

def test_stick_into_tripped_barrier_is_clamped():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        # crate wall dead ahead: south face at north = 2.6
        def _box():
            pos = frames.arena_to_world(cfg, 2.8, 1.1, 1.0)
            col = p.createCollisionShape(p.GEOM_BOX,
                                         halfExtents=[0.6, 0.2, 1.0],
                                         physicsClientId=reg.client)
            return p.createMultiBody(0, col, basePosition=pos,
                                     physicsClientId=reg.client)
        reg.run_on_sim_thread(_box)
        d = _connect(cfg)
        d.takeoff(100)
        d.hover(1.0)
        _pump(d, reg, 3.0, forward=1.0)               # full stick at the wall
        pos, _ = reg.drone_world_pose(0)
        north = frames.world_to_arena(cfg, pos[0], pos[1])[0]
        hull_front = north + cfg.bodies.drone_half_extents_m[0]
        assert hull_front < 2.6                       # NO penetration
        assert north > 1.5                            # ...but it did approach
        assert d.get_obstacles().forward is True      # stopped on the flag
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Arbitration: manual and blocking commands never fight — last wins
# --------------------------------------------------------------------------- #

def test_last_command_wins_between_manual_and_blocking():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = _connect(cfg)
        d.takeoff(100)
        drone = reg.drones[0]
        assert d.send_manual_control(forward=0.3)
        assert reg.run_on_sim_thread(lambda: drone.goal.kind) == "manual"

        r = d.move(Direction.FORWARD, 50, blocking=False)  # blocking wins now
        assert r.success
        assert reg.run_on_sim_thread(lambda: drone.goal.kind) == "move"

        assert d.send_manual_control(forward=0.2)          # manual wins back
        assert reg.run_on_sim_thread(lambda: drone.goal.kind) == "manual"
        d.send_manual_control()                            # release
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Surface + boundaries
# --------------------------------------------------------------------------- #

def test_real_sdk_signatures_and_bool_semantics():
    sig = inspect.signature(DroneAPI.send_manual_control)
    params = [(n, q.default) for n, q in sig.parameters.items() if n != "self"]
    assert params == [("forward", 0.0), ("right", 0.0), ("up", 0.0),
                      ("rotate", 0.0)]
    sig = inspect.signature(DroneAPI.manual_fly)
    params = [(n, q.default) for n, q in sig.parameters.items() if n != "self"]
    assert params == [("duration_sec", E), ("forward", 0.0), ("right", 0.0),
                      ("up", 0.0), ("rotate", 0.0), ("rate_hz", 20),
                      ("on_frame", None)]
    assert DroneAPI().send_manual_control(forward=1.0) is False  # not connected

    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = _connect(cfg)
        assert d.send_manual_control(forward=1.0) is False  # not flying
        d.takeoff(100)
        frames_seen = []
        ok = d.manual_fly(0.3, forward=0.4, rate_hz=20,
                          on_frame=lambda f, r, u, ro, i, k:
                          frames_seen.append((i, k)))
        assert ok and len(frames_seen) == 6           # duration * rate frames
        assert all(k for _i, k in frames_seen)
        d.land()
        assert d.send_manual_control(forward=1.0) is False  # grounded again
        assert reg is not None
    finally:
        shutdown_registry()
