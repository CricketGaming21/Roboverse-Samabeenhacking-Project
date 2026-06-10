"""P1 — config (reject unknown keys) + frames (inverse-consistent) + fly_to_uwb."""

import math

import pytest
import yaml
from pydantic import ValidationError

from mission.config import DEFAULT_CONFIG_PATH, load_config
from mission.control.uwb_loop import fly_to_uwb
from mission.frames import (arena_to_body, body_to_arena, clamp_speed, cm_to_m,
                            m_to_cm)
from mission.runtime import sdk_compat

pytestmark = pytest.mark.p1

NOSLEEP = lambda _s: None


# --------------------------------------------------------------------------- #
# frames
# --------------------------------------------------------------------------- #
def test_units():
    assert m_to_cm(1.0) == 100.0
    assert cm_to_m(100.0) == 1.0
    assert cm_to_m(m_to_cm(3.7)) == pytest.approx(3.7)


def test_frames_identity_at_yaw0():
    assert arena_to_body(1.0, 0.0, 0.0) == pytest.approx((1.0, 0.0))   # north→forward
    assert arena_to_body(0.0, 1.0, 0.0) == pytest.approx((0.0, 1.0))   # east →right


def test_frames_handworked_yaw90():
    # heading east (yaw=90°): a pure NORTH error needs forward 0, right −1 (slide left)
    f, r = arena_to_body(1.0, 0.0, math.radians(90))
    assert (f, r) == pytest.approx((0.0, -1.0), abs=1e-9)


@pytest.mark.parametrize("ex,ey,yaw", [
    (1.0, 0.0, 0.0), (0.0, 1.0, 0.4), (2.3, -1.7, 1.2), (-3.0, 4.0, -2.5),
    (0.5, 0.5, math.pi), (1.0, 1.0, math.pi / 2),
])
def test_frames_inverse_consistent(ex, ey, yaw):
    f, r = arena_to_body(ex, ey, yaw)
    n, e = body_to_arena(f, r, yaw)
    assert (n, e) == pytest.approx((ex, ey), abs=1e-9)


def test_clamp_speed():
    assert clamp_speed(0.4, 0.3) == pytest.approx((0.4, 0.3))          # under cap, kept
    vx, vy = clamp_speed(0.6, 0.8, cap=0.5)                            # mag 1.0 → 0.5
    assert math.hypot(vx, vy) == pytest.approx(0.5)
    assert (vx, vy) == pytest.approx((0.3, 0.4))                       # heading preserved


# --------------------------------------------------------------------------- #
# config
# --------------------------------------------------------------------------- #
def test_load_default_config():
    cfg = load_config()
    assert cfg.speed.max_mps == 0.5
    assert cfg.speed.cruise_alt_m == pytest.approx(1.10)
    assert len(cfg.pads) == 5
    assert cfg.aruco.pad_ids == [10, 11, 12, 13, 14]
    assert len(cfg.valid_pads()) == 4
    assert cfg.uwb.tag_ids == [0, 1, 2]


def test_unknown_key_rejected(tmp_path):
    data = yaml.safe_load(DEFAULT_CONFIG_PATH.read_text())
    data["frame"]["bogus_key"] = 1
    p = tmp_path / "c.yaml"
    p.write_text(yaml.safe_dump(data))
    with pytest.raises(ValidationError):
        load_config(p)


def test_unknown_top_level_key_rejected(tmp_path):
    data = yaml.safe_load(DEFAULT_CONFIG_PATH.read_text())
    data["surprise"] = {"x": 1}
    p = tmp_path / "c.yaml"
    p.write_text(yaml.safe_dump(data))
    with pytest.raises(ValidationError):
        load_config(p)


def test_speed_cap_enforced(tmp_path):
    data = yaml.safe_load(DEFAULT_CONFIG_PATH.read_text())
    data["speed"]["max_mps"] = 0.7                                     # over the hard cap
    p = tmp_path / "c.yaml"
    p.write_text(yaml.safe_dump(data))
    with pytest.raises(ValidationError):
        load_config(p)


# --------------------------------------------------------------------------- #
# sdk_compat
# --------------------------------------------------------------------------- #
def test_prepare_is_noop_on_sim_fake(drone):
    # FakeDroneAPI lacks set_app_mode/etc — the shim must NOT raise.
    sdk_compat.prepare_manual_control(drone, velocity_level=100)
    sdk_compat.send_heartbeat(drone)
    sdk_compat.release(drone)


def test_prepare_runs_real_path_on_real_like(make_drone):
    d = make_drone(0, real_like=True)
    sdk_compat.prepare_manual_control(d, velocity_level=100)
    assert "set_app_mode:1" in d.real_calls
    assert "arm" in d.real_calls
    assert "send_app_heartbeat" in d.real_calls
    assert "set_velocity_level:100" in d.real_calls
    sdk_compat.release(d)
    assert "stop_manual_control" in d.real_calls
    assert "disarm" in d.real_calls
    assert "disconnect" in d.real_calls


def test_sim_fake_lacks_all_realonly_methods(drone):
    # the sim DroneAPI has NONE of these → the shim's hasattr-guards are mandatory
    for name in ("set_app_mode", "send_app_heartbeat", "set_velocity_level",
                 "stop_manual_control", "arm", "disarm", "disconnect",
                 "enable_battery_failsafe"):
        assert not hasattr(drone, name)


# --------------------------------------------------------------------------- #
# fly_to_uwb
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("start,target", [
    ((0.6, 1.1), (8.5, 3.0)),
    ((1.0, 5.0), (2.0, 1.5)),
    ((5.0, 3.0), (5.0, 3.0)),       # already there
    ((9.0, 0.5), (0.5, 5.5)),       # corner to corner
    ((2.0, 2.0), (8.0, 4.5)),
])
def test_converges_from_many_poses(make_drone, uwb, start, target):
    d = make_drone(0)
    d.n, d.e = start
    d.takeoff(110)
    ok = fly_to_uwb(d, uwb, 0, target, sleep=NOSLEEP)
    assert ok is True
    x, y, _ = uwb.get_tag_position(0)
    assert math.hypot(x - target[0], y - target[1]) <= 0.10 + 1e-6


def test_never_exceeds_speed_cap(make_drone, uwb):
    d = make_drone(0)
    d.n, d.e = (0.6, 1.1)
    d.takeoff(110)
    positions = []
    fly_to_uwb(d, uwb, 0, (8.0, 4.0), sleep=NOSLEEP,
               on_step=lambda info: positions.append((d.n, d.e)))
    dt = 1.0 / 20.0
    max_speed = max(
        (math.hypot(positions[i][0] - positions[i - 1][0],
                    positions[i][1] - positions[i - 1][1]) / dt)
        for i in range(1, len(positions)))
    assert max_speed <= 0.5 + 1e-6


def test_holds_on_uwb_dropout(make_drone, uwb):
    d = make_drone(0)
    d.n, d.e = (3.0, 2.0)
    d.takeoff(110)
    uwb.set_unseen(0, True)
    n0, e0 = d.n, d.e
    ok = fly_to_uwb(d, uwb, 0, (8.0, 4.0), sleep=NOSLEEP, max_steps=80)
    assert ok is False                                    # never arrives while blind
    assert math.hypot(d.n - n0, d.e - e0) < 1e-9          # never lurched horizontally


def test_resumes_after_dropout_clears(make_drone, uwb):
    d = make_drone(0)
    d.n, d.e = (3.0, 2.0)
    d.takeoff(110)
    uwb.set_unseen(0, True)
    assert fly_to_uwb(d, uwb, 0, (6.0, 4.0), sleep=NOSLEEP, max_steps=30) is False
    uwb.set_unseen(0, False)
    assert fly_to_uwb(d, uwb, 0, (6.0, 4.0), sleep=NOSLEEP) is True


def test_altitude_holds_to_cruise(make_drone, uwb):
    d = make_drone(0)
    d.n, d.e = (2.0, 2.0)
    d.takeoff(60)                                         # start low
    fly_to_uwb(d, uwb, 0, (4.0, 2.0), alt_m=1.10, sleep=NOSLEEP)
    assert d.get_altitude() == pytest.approx(110.0, abs=2.0)


def test_converges_under_uwb_noise(make_drone, world):
    from tests.fakes import fake_uwb as fuwb
    noisy = fuwb.FakeUWBParserThread(world=world, noise_std_m=0.03, seed=5)
    d = make_drone(0)
    d.n, d.e = (1.0, 1.0)
    d.takeoff(110)
    ok = fly_to_uwb(d, uwb=noisy, tag_id=0, target_xy_m=(7.0, 4.0),
                    tol_m=0.15, speed_tol_mps=0.2, sleep=NOSLEEP)
    assert ok is True
    # true position (noise-free) should be close to target
    assert math.hypot(d.n - 7.0, d.e - 4.0) <= 0.25
