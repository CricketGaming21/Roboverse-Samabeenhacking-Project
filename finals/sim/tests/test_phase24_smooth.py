"""Phase 24 acceptance test — critically-damped smooth flight + deliberate
pacing (headless).

Kills the Phase-18 wobble: no overshoot ringing, rate-limited tilt, wind off
by default, a slow low-frequency get_position drift over a SMOOTH true body
pose, no command thrash in the demo, and a slow deliberate episode.
"""

import math

import numpy as np
import pytest

from pyhulax import DroneAPI
from pyhulax.core import Direction

from scripts.scenario_demo import DEMO_SPEED, run_scenario_demo
from simcore.config import load_config
from simcore.drone_model import speed_to_mps
from simcore.registry import get_registry, shutdown_registry


def _cfg(rtf=10.0):
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = rtf
    c.camera.use_egl = False
    c.scoring.enabled = False
    c.arena.layout = "procedural"
    c.arena.obstacles.count = 0
    c.rovers.count = 0
    return c


def _connect(cfg, index=0):
    d = DroneAPI()
    d.connect(cfg.drones.units[index].ip)
    return d


def _sample(reg, until_done=True, period=0.008):
    import time
    out = []
    drone = reg.drones[0]
    deadline = time.time() + 30
    while time.time() < deadline:
        t, pos, vel, tilt, busy = reg.run_on_sim_thread(
            lambda: (reg.clock.now(), drone.pos.copy(),
                     float(np.linalg.norm(drone.vel[:2])),
                     float(drone.tilt_deg), drone.goal is not None))
        out.append((t, pos, vel, tilt))
        if until_done and not busy:
            break
        time.sleep(period)
    return out


# --------------------------------------------------------------------------- #
# Critically damped: settles with no oscillation/ringing
# --------------------------------------------------------------------------- #

def test_move_settles_without_oscillation():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = _connect(cfg)
        d.takeoff(100)
        d.hover(1.0)
        p0, _ = reg.drone_world_pose(0)
        d.move(Direction.FORWARD, 200, blocking=False)   # +2 m north
        ys = [p[1] - p0[1] for _t, p, _v, _ti in _sample(reg)]
        target = 2.0
        overshoot = max(ys) - target
        assert overshoot <= 0.06                  # essentially no overshoot
        assert min(ys) >= -0.02                   # monotonic forward (no back)
        # NO ringing: the trajectory never reverses direction once moving
        # (strictly non-decreasing within a small tolerance to the target)
        peak_i = max(range(len(ys)), key=lambda k: ys[k])
        after_peak = ys[peak_i:]
        # after the peak it may settle back a hair, but must not re-rise:
        # count sign changes of the velocity along-track -> at most 1.
        dirs = [b - a for a, b in zip(ys, ys[1:]) if abs(b - a) > 1e-4]
        sign_changes = sum(1 for a, b in zip(dirs, dirs[1:]) if a * b < 0)
        assert sign_changes <= 1                  # no oscillation/ringing
        assert abs(ys[-1] - target) <= cfg.motion.arrive_tol_m + 0.01
    finally:
        shutdown_registry()


def test_tilt_rate_is_bounded():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = _connect(cfg)
        d.takeoff(100)
        d.hover(1.0)
        d.move(Direction.FORWARD, 300, blocking=False)
        samples = _sample(reg)
        cap = cfg.motion.tilt_rate_dps
        max_rate = 0.0
        for (t0, _p0, _v0, ti0), (t1, _p1, _v1, ti1) in zip(samples,
                                                            samples[1:]):
            dt = t1 - t0
            if dt > 1e-6:
                max_rate = max(max_rate, abs(ti1 - ti0) / dt)
        assert max_rate <= cap * 1.5              # |dtilt/dt| bounded by cap
        assert max(abs(ti) for _t, _p, _v, ti in samples) <= \
            cfg.motion.max_tilt_deg + 1e-6
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Wind off by default: a hovering drone holds within a few cm
# --------------------------------------------------------------------------- #

def test_wind_off_hover_is_steady():
    cfg = _cfg()
    assert cfg.motion.wind.enabled is False       # off by default
    reg = get_registry(cfg)
    try:
        d = _connect(cfg)
        d.takeoff(100)
        d.hover(1.0)
        p0, _ = reg.drone_world_pose(0)
        worst = 0.0
        for _ in range(10):                       # ~10 sim s of hover
            d.hover(1.0)
            p1, _ = reg.drone_world_pose(0)
            worst = max(worst, math.dist(p0[:2], p1[:2]))
        assert worst < 0.03                       # steady within a few cm
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Drift: slow + low-frequency, bounded; true body pose smooth
# --------------------------------------------------------------------------- #

def test_drift_is_slow_low_frequency_and_bounded():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = _connect(cfg)
        d.takeoff(100)
        drone = reg.drones[0]
        # sample get_position drift error AND true pose at a steady cadence
        drift, truex = [], []
        import time
        for _ in range(150):                      # ~? sim s, fine cadence
            d.hover(0.4)
            e, tx = reg.run_on_sim_thread(
                lambda: (drone.drift_err.copy(), float(drone.pos[0])))
            drift.append(e)
            truex.append(tx)
        dh = np.array([np.linalg.norm(e[:2]) for e in drift])
        bound = cfg.position_drift.horizontal_bound_m
        assert dh.max() <= bound + 1e-6           # strictly bounded <= ±20 cm
        assert dh.mean() > 0.02                   # really wanders (not frozen)
        # LOW-FREQUENCY: most spectral energy of the drift is in slow bands.
        sig = np.array([e[0] for e in drift]) - np.mean([e[0] for e in drift])
        if np.any(sig):
            spec = np.abs(np.fft.rfft(sig))
            lo = spec[:max(2, len(spec) // 5)].sum()   # slowest 20% of bins
            assert lo >= 0.7 * spec.sum()         # dominated by low freq
        # TRUE body pose is SMOOTH: hovering, the true x barely moves and has
        # no high-frequency jitter (successive steps tiny).
        steps = np.abs(np.diff(truex))
        assert steps.max() < 0.01                 # no vibration of the body
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# UWB unchanged: truth + small noise, no drift
# --------------------------------------------------------------------------- #

def test_uwb_has_no_drift():
    from UWBParserThread import UWBParserThread
    cfg = _cfg()
    reg = get_registry(cfg)
    u = UWBParserThread()
    u.start()
    try:
        import time
        d = _connect(cfg)
        d.takeoff(100)
        tag = cfg.drones.units[0].uwb_tag_id
        errs = []
        for _ in range(30):
            d.hover(0.4)
            time.sleep(0.01)
            x, y, _t = u.get_tag_position(tag)
            if x is None:
                continue
            tn, te = reg.run_on_sim_thread(lambda: reg.drones[0].arena_position())
            errs.append(math.hypot(x - tn, y - te))
        assert errs and max(errs) < 6 * cfg.uwb.noise_std_m   # noise only
    finally:
        u.stop()
        u.join(timeout=2)
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Crisp mode still exact
# --------------------------------------------------------------------------- #

def test_crisp_mode_still_exact():
    cfg = _cfg()
    cfg.motion.realistic = False
    reg = get_registry(cfg)
    try:
        d = _connect(cfg)
        start, _ = reg.drone_world_pose(0)
        d.takeoff(100)
        for _ in range(4):
            d.move(Direction.FORWARD, 100)
            d.rotate(90)
        d.land()
        end, _ = reg.drone_world_pose(0)
        assert math.dist(start[:2], end[:2]) < 0.01
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Deliberate pacing: no thrash, slow demo speed, long episode, slow rovers
# --------------------------------------------------------------------------- #

def test_demo_run_has_no_thrash_warnings():
    cfg = load_config("sim_config.yaml")
    cfg.meta.real_time_factor = 12.0
    cfg.scenario.episode_seconds = 60.0
    cfg.scenario.ambush_seconds = 50.0
    cfg.scenario.deploy_timeout_s = 30.0
    cfg.scenario.ambush_trigger.mode = "on_all_landed"
    cfg.scenario.ambush_trigger.delay_s = 1.0
    result = run_scenario_demo(cfg)
    mon = result["monitor"]
    assert sum(mon["rate_warnings"].values()) == 0     # no fast re-commands
    assert sum(mon["preemptions"].values()) == 0       # no unfinished-goal kill
    assert result["final_phase"] == "done"


def test_demo_speed_is_slow_and_config_pacing():
    cfg = load_config("sim_config.yaml")
    # demo flies at <= 0.5 m/s (SLOW/MEDIUM), not the 0.8-1.0 cap
    assert speed_to_mps(cfg, DEMO_SPEED) <= 0.5
    # band cap + vertical specs unchanged
    assert cfg.velocity_levels.TURBO == 1.0
    assert cfg.velocity_levels.climb_mps == 1.2
    assert cfg.velocity_levels.descent_mps == 1.0
    # long deliberate episode, slow rovers — from config
    assert cfg.scenario.episode_seconds >= 240
    assert cfg.rovers.convoy.speed_mps <= 0.3
    # rover route/stagger/loiter are clean config keys
    cv = cfg.rovers.convoy
    assert cv.entry_stagger_s > 0 and cv.loiter in ("loop", "hold")
    assert len(cv.branches) >= cfg.rovers.count
