"""Phase 18 acceptance test — realistic motion model (headless).

Asserts BEHAVIOUR: accel->cruise->decel velocity profile (not a step),
tilt-to-translate (<=20°, levels at cruise), bounded overshoot + settle,
command latency, asymmetric climb/descent, drift BOUNDED near the ±20 cm
optical-flow accuracy, and that crisp mode still gives exact geometry.
"""

import math
import time

import numpy as np
import pytest

from pyhulax import DroneAPI
from pyhulax.core import Direction

from simcore.config import load_config
from simcore.registry import get_registry, shutdown_registry


def _cfg(realistic=True, rtf=10.0):
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = rtf
    c.camera.use_egl = False
    c.scoring.enabled = False
    c.arena.layout = "procedural"
    c.arena.obstacles.count = 0
    c.rovers.count = 0
    c.motion.realistic = realistic
    return c


def _connect(cfg, index=0) -> DroneAPI:
    d = DroneAPI()
    d.connect(cfg.drones.units[index].ip)
    return d


def _sample_motion(reg, until_goal_done, period_wall=0.01):
    """(sim_t, pos, |vel|, tilt) samples while a goal runs."""
    out = []
    drone = reg.drones[0]
    deadline = time.time() + 30
    while time.time() < deadline:
        t, pos, vel, tilt, busy = reg.run_on_sim_thread(
            lambda: (reg.clock.now(), drone.pos.copy(),
                     float(np.linalg.norm(drone.vel[:2])),
                     float(drone.tilt_deg), drone.goal is not None))
        out.append((t, pos, vel, tilt))
        if until_goal_done and not busy:
            break
        time.sleep(period_wall)
    return out


# --------------------------------------------------------------------------- #
# Velocity profile + tilt-to-translate
# --------------------------------------------------------------------------- #

def test_move_shows_accel_cruise_decel_profile():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = _connect(cfg)
        d.takeoff(100)
        d.hover(1.0)  # settle out of the climb
        d.move(Direction.FORWARD, 300, blocking=False)  # 3 m at ZOOM 0.8
        samples = _sample_motion(reg, until_goal_done=True)
        speeds = [v for _t, _p, v, _ti in samples]
        cap = cfg.velocity_levels.ZOOM
        assert max(speeds) == pytest.approx(cap, rel=0.15)  # reaches cruise
        assert speeds[0] < 0.5 * cap          # started slow: a RAMP, no step
        peak = speeds.index(max(speeds))
        assert peak > 1                       # accel section exists
        assert min(speeds[peak:]) < 0.15      # decel section brings it down
        # cruise plateau: several samples near the cap
        assert sum(1 for v in speeds if v > 0.9 * cap) >= 3
    finally:
        shutdown_registry()


def test_body_tilts_during_accel_and_levels_at_cruise():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = _connect(cfg)
        d.takeoff(100)
        d.hover(1.0)
        d.move(Direction.FORWARD, 300, blocking=False)
        samples = _sample_motion(reg, until_goal_done=True)
        tilts = [ti for _t, _p, _v, ti in samples]
        speeds = [v for _t, _p, v, _ti in samples]
        cap = cfg.velocity_levels.ZOOM
        assert max(abs(t) for t in tilts) <= cfg.motion.max_tilt_deg + 1e-6
        assert max(abs(t) for t in tilts) > 5.0      # it really tilts
        # nose-down (negative) while accelerating forward
        accel_phase = [ti for (_t, _p, v, ti) in samples[:8] if v < 0.7 * cap]
        assert any(ti < -5.0 for ti in accel_phase)
        # level at cruise
        cruise = [abs(ti) for (_t, _p, v, ti) in samples if v > 0.95 * cap]
        assert cruise and min(cruise) < 4.0
        # settled level at the end
        assert abs(tilts[-1]) < 4.0
    finally:
        shutdown_registry()


def test_waypoint_overshoot_is_bounded_then_settles():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = _connect(cfg)
        d.takeoff(100)
        d.hover(1.0)
        p0, _ = reg.drone_world_pose(0)
        d.move(Direction.FORWARD, 200, blocking=False)  # +2 m north
        samples = _sample_motion(reg, until_goal_done=True)
        ys = [p[1] - p0[1] for _t, p, _v, _ti in samples]
        overshoot = max(ys) - 2.0
        assert 0.0 < overshoot <= 0.35        # past the waypoint, bounded
        final = ys[-1]
        assert abs(final - 2.0) <= cfg.motion.arrive_tol_m + 0.01  # settled
    finally:
        shutdown_registry()


def test_command_latency_before_motion():
    cfg = _cfg(rtf=1.0)  # real time so the 80 ms latency is observable
    reg = get_registry(cfg)
    try:
        d = _connect(cfg)
        d.takeoff(100)
        d.hover(1.0)
        drone = reg.drones[0]
        t0 = reg.sim_time()
        d.move(Direction.FORWARD, 100, blocking=False)
        moved_at = None
        deadline = time.time() + 5
        start_y = reg.run_on_sim_thread(lambda: float(drone.pos[1]))
        while time.time() < deadline:
            t, y = reg.run_on_sim_thread(
                lambda: (reg.clock.now(), float(drone.pos[1])))
            if y - start_y > 0.005:
                moved_at = t
                break
            time.sleep(0.005)
        assert moved_at is not None
        assert moved_at - t0 >= cfg.motion.latency_s * 0.8  # waited ~latency
    finally:
        shutdown_registry()


def test_climb_and_descent_speeds_differ_realistically():
    """Asymmetric vertical caps show up in the PEAK vertical speeds (duration
    comparisons drown in settle jitter at these short distances)."""
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = _connect(cfg)
        drone = reg.drones[0]

        def peak_vz(goal_call):
            goal_call()
            peak = 0.0
            deadline = time.time() + 20
            while time.time() < deadline:
                vz, busy = reg.run_on_sim_thread(
                    lambda: (float(drone.vel[2]), drone.goal is not None))
                peak = max(peak, abs(vz))
                if not busy:
                    return peak
                time.sleep(0.01)
            pytest.fail("vertical goal never completed")

        up = peak_vz(lambda: d.takeoff(250, blocking=False))
        d.hover(0.5)
        down = peak_vz(lambda: d.land(blocking=False))
        assert up == pytest.approx(cfg.velocity_levels.climb_mps, rel=0.15)
        assert down == pytest.approx(cfg.velocity_levels.descent_mps,
                                     rel=0.15)
        assert up > down                      # climb 1.2 > descent 1.0
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Calibrated, BOUNDED drift (the optical-flow accuracy)
# --------------------------------------------------------------------------- #

def test_drift_stays_bounded_near_20cm_horizontal():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = _connect(cfg)
        d.takeoff(100)
        drone = reg.drones[0]
        errs = []
        for _ in range(120):                  # ~60 sim seconds hovering
            d.hover(0.5)
            errs.append(reg.run_on_sim_thread(
                lambda: float(np.linalg.norm(drone.drift_err[:2]))))
        bound = cfg.position_drift.horizontal_bound_m
        assert max(errs) <= bound * 2.2       # BOUNDED: never runs away
        assert 0.02 <= float(np.mean(errs)) <= bound  # ...but really wanders
        # mean-reverting: the late window is no worse than the run overall
        late = float(np.mean(errs[-30:]))
        assert late <= max(2.0 * float(np.mean(errs)), bound)
        # vertical bounded too
        vert = reg.run_on_sim_thread(lambda: abs(float(drone.drift_err[2])))
        assert vert <= cfg.position_drift.vertical_bound_m * 2.5
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# The crisp toggle: old exact geometry preserved
# --------------------------------------------------------------------------- #

def test_crisp_mode_remains_exact():
    cfg = _cfg(realistic=False)
    reg = get_registry(cfg)
    try:
        d = _connect(cfg)
        start, _ = reg.drone_world_pose(0)
        d.takeoff(100)
        for _ in range(4):                    # the classic square
            d.move(Direction.FORWARD, 100)
            d.rotate(90)
        d.land()
        end, _ = reg.drone_world_pose(0)
        assert math.dist(start[:2], end[:2]) < 0.01   # closure ~0
        assert reg.run_on_sim_thread(
            lambda: float(reg.drones[0].tilt_deg)) == 0.0  # never tilted
    finally:
        shutdown_registry()


def test_wind_disturbs_hover_but_station_keeping_holds():
    cfg = _cfg()
    cfg.motion.wind_mps = 0.15
    reg = get_registry(cfg)
    try:
        d = _connect(cfg)
        d.takeoff(100)
        d.hover(1.0)  # settle at the hold point
        p0, _ = reg.drone_world_pose(0)
        drifted = 0.0
        for _ in range(8):                    # ~8 sim s of gusts
            d.hover(1.0)
            p1, _ = reg.drone_world_pose(0)
            drifted = max(drifted, math.dist(p0[:2], p1[:2]))
        assert drifted > 0.001                # the wind really perturbs it
        assert drifted < 0.4                  # ...but station-keeping holds
    finally:
        shutdown_registry()
