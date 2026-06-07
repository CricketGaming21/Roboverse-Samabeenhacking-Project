"""Phase 3 acceptance test — frames, drift, UWB drop-in, telemetry (headless).

The core asymmetry under test (§5.3):
  get_position() = takeoff-frame estimate that DRIFTS away from truth;
  UWB get_tag_position() = arena truth + Gaussian noise, NO drift.
All randomness is seeded from config, so assertions are deterministic.
"""

import copy
import math
import time

import pytest

from pyhulax import DroneAPI
from pyhulax.core import Direction, DroneState, Obstacles, Orientation, Vector3
from pyhulax.exceptions import TelemetryUnavailable
from UWBParserThread import UWBParserThread

from simcore import frames
from simcore.config import load_config
from simcore.registry import get_registry, shutdown_registry


@pytest.fixture()
def cfg():
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0
    c.scoring.enabled = False  # referee not under test here
    c.camera.use_egl = False
    return c


@pytest.fixture()
def sim(cfg):
    reg = get_registry(cfg)
    yield reg
    shutdown_registry()


@pytest.fixture()
def cfg_rt(cfg):
    """Real-time config for wall-clock UWB cadence checks."""
    c = copy.deepcopy(cfg)
    c.meta.real_time_factor = 1.0
    return c


def _connect(cfg, index=0) -> DroneAPI:
    d = DroneAPI()
    d.connect(cfg.drones.units[index].ip)
    return d


def _start_uwb(**kwargs) -> UWBParserThread:
    u = UWBParserThread(**kwargs)
    u.start()
    return u


def _stop_uwb(u: UWBParserThread) -> None:
    u.stop()
    u.join(timeout=2)


def _wait_for_sample(u: UWBParserThread, tag: int, timeout_s: float = 5.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        s = u.get_tag_position(tag)
        if s[0] is not None:
            return s
        time.sleep(0.005)
    pytest.fail(f"no UWB sample for tag {tag} within {timeout_s}s")


def _true_arena(reg, index=0):
    d = reg.drones[index]
    return reg.run_on_sim_thread(d.arena_position)


def _position_error_cm(reg, index=0) -> float:
    """|get_position estimate - true pose| in the takeoff frame, atomically."""
    d = reg.drones[index]

    def _err():
        est = d.telemetry_position()
        tx, ty, tz = frames.world_to_takeoff_cm(
            d.takeoff_frame, d.pos[0], d.pos[1], d.pos[2])
        return math.dist((est.x, est.y, est.z), (tx, ty, tz))
    return reg.run_on_sim_thread(_err)


# --------------------------------------------------------------------------- #
# UWB — tracks truth, arena frame, metres
# --------------------------------------------------------------------------- #

def test_uwb_tracks_truth_through_motion(sim, cfg):
    tag = cfg.drones.units[0].uwb_tag_id
    noise = cfg.uwb.noise_std_m
    u = _start_uwb()
    try:
        d = _connect(cfg)
        d.takeoff(100)
        _wait_for_sample(u, tag)
        errs = []
        for _ in range(10):  # hovering: samples vs truth within noise bounds
            x, y, _t = u.get_tag_position(tag)
            tn, te = _true_arena(sim)
            errs.append(math.hypot(x - tn, y - te))
            time.sleep(0.03)
        assert max(errs) < 6 * noise
        assert sum(errs) / len(errs) < 3 * noise
        d.move(Direction.FORWARD, 200)  # 2 m north
        time.sleep(0.05)                # let a refresh land
        x, y, _t = u.get_tag_position(tag)
        tn, te = _true_arena(sim)
        assert math.hypot(x - tn, y - te) < 6 * noise  # UWB followed the move
        assert abs(tn - 2.6) < 0.05                    # truth itself moved
    finally:
        _stop_uwb(u)


def test_uwb_reports_all_mapped_tags_and_no_unmapped(sim, cfg):
    u = _start_uwb()
    try:
        for unit, start in zip(cfg.drones.units,
                               [(0.6, 1.5), (0.6, 3.0), (0.6, 4.5)]):
            x, y, t = _wait_for_sample(u, unit.uwb_tag_id)
            assert abs(x - start[0]) < 0.3  # north
            assert abs(y - start[1]) < 0.3  # east
            assert t is not None
        assert u.get_tag_position(99) == (None, None, None)  # unmapped tag
    finally:
        _stop_uwb(u)


# --------------------------------------------------------------------------- #
# The drift asymmetry (§5.3)
# --------------------------------------------------------------------------- #

def test_position_drifts_while_uwb_does_not(sim, cfg):
    tag = cfg.drones.units[0].uwb_tag_id
    u = _start_uwb()
    try:
        d = _connect(cfg)
        d.takeoff(100)
        err_start = _position_error_cm(sim)
        # ~12 sim seconds of flying (drift accumulates while airborne)
        d.hover(6)
        d.move(Direction.FORWARD, 100)
        d.hover(5)
        err_end = _position_error_cm(sim)
        assert err_end > err_start          # estimate walked away from truth
        assert err_end > 1.0                # visibly (cm; seeded => stable)
        x, y, _t = _wait_for_sample(u, tag)  # UWB still within its noise band
        tn, te = _true_arena(sim)
        assert math.hypot(x - tn, y - te) < 6 * cfg.uwb.noise_std_m
    finally:
        _stop_uwb(u)


def test_drift_disabled_keeps_estimate_exact(cfg):
    cfg2 = copy.deepcopy(cfg)
    cfg2.position_drift.enabled = False
    reg = get_registry(cfg2)
    try:
        d = _connect(cfg2)
        d.takeoff(100)
        d.hover(5)
        assert _position_error_cm(reg) < 1e-6
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# UWB sample mechanics: hold, wall-clock update_time, rate, quirks
# --------------------------------------------------------------------------- #

def test_uwb_holds_sample_between_refreshes_at_wallclock_rate(cfg_rt):
    reg = get_registry(cfg_rt)
    u = _start_uwb()
    try:
        tag = cfg_rt.drones.units[0].uwb_tag_id
        _wait_for_sample(u, tag)
        samples = []
        end = time.time() + 1.0
        while time.time() < end:  # poll much faster than uwb.rate_hz
            samples.append(u.get_tag_position(tag))
            time.sleep(0.004)
        valid = [s for s in samples if s[2] is not None]
        assert valid
        # update_time is wall-clock
        assert abs(valid[-1][2] - time.time()) < 1.0
        # refreshes arrive at ~rate_hz (10 Hz) in real time
        stamps = sorted({s[2] for s in valid})
        assert 6 <= len(stamps) <= 15
        gaps = [b - a for a, b in zip(stamps, stamps[1:])]
        assert all(0.03 < g < 0.3 for g in gaps)
        # between refreshes the SAME (value, timestamp) pair is held
        by_stamp = {}
        for x, y, t in valid:
            by_stamp.setdefault(t, set()).add((x, y))
        assert all(len(vals) == 1 for vals in by_stamp.values())
    finally:
        _stop_uwb(u)
        shutdown_registry()


def test_origin_args_accepted_but_not_applied(sim, cfg):
    """Real-code quirk: x_origin/y_origin never shift the output while
    uwb.apply_origin_offset is false (the default)."""
    assert cfg.uwb.apply_origin_offset is False
    tag = cfg.drones.units[0].uwb_tag_id
    u = _start_uwb(x_origin=5.0, y_origin=-3.0)
    try:
        assert u.origin_x == 0.0 and u.origin_y == 0.0  # matches real attrs
        x, y, _t = _wait_for_sample(u, tag)
        tn, te = _true_arena(sim)
        # a 5 m / -3 m shift would blow way past the noise band
        assert math.hypot(x - tn, y - te) < 6 * cfg.uwb.noise_std_m
    finally:
        _stop_uwb(u)


def test_uwb_dropout(cfg):
    cfg2 = copy.deepcopy(cfg)
    cfg2.uwb.dropout_prob = 1.0  # every mapped tag drops every frame
    get_registry(cfg2)
    u = _start_uwb()
    try:
        time.sleep(0.3)  # several refresh periods
        tag = cfg2.drones.units[0].uwb_tag_id
        assert u.get_tag_position(tag) == (None, None, None)
    finally:
        _stop_uwb(u)
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Telemetry: get_position / get_orientation / get_altitude / get_state
# --------------------------------------------------------------------------- #

def test_telemetry_values(sim, cfg):
    d = _connect(cfg)
    p0 = d.get_position()  # grounded, pre-takeoff: provisional frame, ~origin
    assert isinstance(p0, Vector3)
    assert abs(p0.x) < 1.0 and abs(p0.y) < 1.0 and p0.z < 10.0

    d.takeoff(100)
    assert abs(d.get_altitude() - 100.0) < 3.0  # ToF cm, true height
    p1 = d.get_position()
    assert abs(p1.z - 100.0) < 5.0              # estimate ~100 cm up

    o0 = d.get_orientation()
    assert isinstance(o0, Orientation)
    assert o0.yaw < 2.0 or o0.yaw > 358.0       # at takeoff heading
    d.rotate(90)
    o1 = d.get_orientation()
    assert 88.0 < o1.yaw < 92.0                 # +90 = CCW
    assert o1.pitch == 0.0 and o1.roll == 0.0

    d.move(Direction.UP, 50)
    assert abs(d.get_altitude() - 150.0) < 3.0

    st = d.get_state()
    assert isinstance(st, DroneState)
    assert st.connected is True and st.flying is True
    assert isinstance(st.position, Vector3)
    assert isinstance(st.obstacles, Obstacles)
    assert isinstance(st.battery, int) and 0 <= st.battery <= 100


def test_telemetry_unavailable_before_connect(sim):
    d = DroneAPI()
    for call in (d.get_position, d.get_orientation, d.get_altitude,
                 d.get_state, d.get_battery):
        with pytest.raises(TelemetryUnavailable):
            call()
