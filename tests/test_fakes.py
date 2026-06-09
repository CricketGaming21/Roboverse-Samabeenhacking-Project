"""P0 — the fake SDK substrate.

Verifies the fake mirrors the sim SUBSET, the kinematics/clamp/obstacle/camera/UWB
models behave per docs, and everything is deterministic under the seed.
"""

import math

import cv2
import numpy as np
import pytest

from tests.fakes import fake_pyhulax as fpx
from tests.fakes import fake_uwb as fuwb

pytestmark = pytest.mark.p0


# --------------------------------------------------------------------------- #
# kinematics: direction + speed clamp
# --------------------------------------------------------------------------- #
def test_forward_stick_moves_north_at_yaw0(drone):
    drone.takeoff(110)
    n0, e0 = drone.n, drone.e
    for _ in range(10):
        drone.send_manual_control(forward=1.0)
    assert drone.n > n0 + 0.1          # moved north
    assert abs(drone.e - e0) < 1e-9    # no east drift


def test_right_stick_moves_east_at_yaw0(drone):
    drone.takeoff(110)
    n0, e0 = drone.n, drone.e
    for _ in range(10):
        drone.send_manual_control(right=1.0)
    assert drone.e > e0 + 0.1
    assert abs(drone.n - n0) < 1e-9


def test_forward_stick_follows_yaw(drone):
    drone.takeoff(110)
    drone.yaw_deg = 90.0               # nose now points +east
    n0, e0 = drone.n, drone.e
    for _ in range(10):
        drone.send_manual_control(forward=1.0)
    assert drone.e > e0 + 0.1
    assert abs(drone.n - n0) < 1e-6


def test_speed_clamp_holds(drone, world):
    drone.takeoff(110)
    n0, e0 = drone.n, drone.e
    steps = 20
    for _ in range(steps):
        drone.send_manual_control(forward=1.0, right=1.0)   # would be 0.707 m/s unclamped
    travelled = math.hypot(drone.n - n0, drone.e - e0)
    cap = world.max_mps * world.dt * steps
    assert travelled <= cap + 1e-9                          # never exceeds the cap
    assert travelled == pytest.approx(cap, rel=1e-6)        # and is exactly capped
    # the unclamped diagonal would have been sqrt(2) larger:
    assert travelled < math.sqrt(2) * cap * 0.99


def test_manual_control_returns_bool(drone):
    drone.takeoff(110)
    assert drone.send_manual_control(forward=0.5) is True


def test_altitude_integrates_from_up_stick(drone):
    drone.takeoff(100)
    a0 = drone.get_altitude()
    for _ in range(10):
        drone.send_manual_control(up=1.0)
    assert drone.get_altitude() > a0
    # climb is capped at climb_mps
    assert drone.get_altitude() == pytest.approx(a0 + 100 * 0.5 * 10 * 0.05, rel=1e-6)


# --------------------------------------------------------------------------- #
# obstacles
# --------------------------------------------------------------------------- #
def test_obstacle_forward_when_crate_ahead(world):
    fpx.set_active_world(world)
    d = fpx.FakeDroneAPI(world)
    d.connect(world.ip_map[0])
    d.n, d.e, d.alt_cm, d.yaw_deg = 4.0, 3.0, 110.0, 0.0   # crate at (5.0,3.0) ahead
    obs = d.get_obstacles()
    assert obs.forward is True
    assert obs.back is False and obs.left is False and obs.right is False


def test_obstacle_clears_below_min_alt(world):
    fpx.set_active_world(world)
    d = fpx.FakeDroneAPI(world)
    d.connect(world.ip_map[0])
    d.n, d.e, d.alt_cm, d.yaw_deg = 4.0, 3.0, 20.0, 0.0    # below sensing alt
    obs = d.get_obstacles()
    assert obs.forward is False                            # horizontal sensors off
    assert obs.down is True                                # ground close


def test_obstacle_other_drone_to_the_side():
    w = fpx.FakeWorld(crates=[])                           # isolate: no crates
    fpx.set_active_world(w)
    a = fpx.FakeDroneAPI(w); a.connect(w.ip_map[0])
    b = fpx.FakeDroneAPI(w); b.connect(w.ip_map[1])
    a.n, a.e, a.alt_cm, a.yaw_deg = 4.0, 3.0, 110.0, 0.0
    b.n, b.e, b.alt_cm, b.yaw_deg = 4.0, 3.4, 110.0, 0.0   # 0.4 m east of A
    obs = a.get_obstacles()
    assert obs.right is True
    assert obs.forward is False and obs.left is False
    fpx.set_active_world(None)


def test_drone_status_bitmask(world):
    fpx.set_active_world(world)
    d = fpx.FakeDroneAPI(world); d.connect(world.ip_map[0])
    d.n, d.e, d.alt_cm, d.yaw_deg = 4.0, 3.0, 110.0, 0.0
    assert d.get_drone_status() & 0b1                      # forward bit set
    assert d.any_obstacle() is True


# --------------------------------------------------------------------------- #
# camera: real ArUco decode
# --------------------------------------------------------------------------- #
def _decode(frame_rgb):
    gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
    det = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250),
        cv2.aruco.DetectorParameters())
    _, ids, _ = det.detectMarkers(gray)
    return [] if ids is None else sorted(int(i) for i in ids.flatten())


def test_nadir_frame_decodes_known_marker(world):
    fpx.set_active_world(world)
    d = fpx.FakeDroneAPI(world); d.connect(world.ip_map[0])
    d.n, d.e, d.alt_cm = 8.5, 3.0, 110.0                   # directly over pad id 10
    d.set_camera_angle(fpx.CameraPitchMode.DOWN_ABSOLUTE, 90)
    d.set_video_stream(True)
    s = d.create_video_stream(); s.start()
    ids = _decode(s.latest_frame.to_rgb())
    assert 10 in ids


def test_offset_marker_still_decodes(world):
    fpx.set_active_world(world)
    d = fpx.FakeDroneAPI(world); d.connect(world.ip_map[0])
    d.n, d.e, d.alt_cm = 8.3, 2.85, 130.0                  # offset & higher
    d.set_camera_angle(fpx.CameraPitchMode.DOWN_ABSOLUTE, 90)
    d.set_video_stream(True)
    s = d.create_video_stream(); s.start()
    assert 10 in _decode(s.latest_frame.to_rgb())


def test_stream_none_before_start(world):
    fpx.set_active_world(world)
    d = fpx.FakeDroneAPI(world); d.connect(world.ip_map[0])
    s = d.create_video_stream()
    assert s.latest_frame is None


# --------------------------------------------------------------------------- #
# UWB
# --------------------------------------------------------------------------- #
def test_uwb_returns_truth_noiseless(world, uwb):
    d = fpx.FakeDroneAPI(world); d.connect(world.ip_map[0])
    d.n, d.e = 3.0, 2.0
    x, y, t = uwb.get_tag_position(0)
    assert x == pytest.approx(3.0) and y == pytest.approx(2.0)
    assert t == world.clock


def test_uwb_unmapped_and_unseen_return_none(world):
    u = fuwb.FakeUWBParserThread(world=world)
    fpx.FakeDroneAPI(world).connect(world.ip_map[0])
    assert u.get_tag_position(99) == (None, None, None)    # unmapped
    u.set_unseen(0, True)
    assert u.get_tag_position(0) == (None, None, None)     # forced dropout


def test_uwb_noise_is_zero_mean_and_bounded(world):
    fpx.FakeDroneAPI(world).connect(world.ip_map[0])
    world.truth_xy(0)
    u = fuwb.FakeUWBParserThread(world=world, noise_std_m=0.05, seed=7)
    xs = []
    for _ in range(4000):
        x, y, _ = u.get_tag_position(0)
        xs.append(x)
    arr = np.array(xs)
    truth_x = world.truth_xy(0)[0]
    assert abs(arr.mean() - truth_x) < 0.01                # ~zero-mean noise
    assert abs(arr.std() - 0.05) < 0.01                    # right magnitude


def test_uwb_stochastic_dropout(world):
    fpx.FakeDroneAPI(world).connect(world.ip_map[0])
    u = fuwb.FakeUWBParserThread(world=world, dropout_prob=0.3, seed=3)
    nones = sum(u.get_tag_position(0)[0] is None for _ in range(2000))
    assert 0.25 * 2000 < nones < 0.35 * 2000


def test_uwb_deterministic_under_seed(world):
    fpx.FakeDroneAPI(world).connect(world.ip_map[0])
    seq1 = [fuwb.FakeUWBParserThread(world=world, noise_std_m=0.05, seed=11)
            .get_tag_position(0) for _ in range(1)]
    a = fuwb.FakeUWBParserThread(world=world, noise_std_m=0.05, seed=11)
    b = fuwb.FakeUWBParserThread(world=world, noise_std_m=0.05, seed=11)
    seqa = [a.get_tag_position(0) for _ in range(50)]
    seqb = [b.get_tag_position(0) for _ in range(50)]
    assert seqa == seqb


# --------------------------------------------------------------------------- #
# the fake mirrors the SUBSET (forces the compat shim)
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("name", [
    "set_app_mode", "send_app_heartbeat", "set_velocity_level",
    "stop_manual_control", "arm", "disarm", "disconnect", "get_velocity",
    "get_drone_id", "enable_battery_failsafe",
])
def test_real_only_methods_absent_on_sim_fake(drone, name):
    assert not hasattr(drone, name), f"{name} must be absent (forces sdk_compat)"


@pytest.mark.parametrize("name", [
    "set_app_mode", "send_app_heartbeat", "set_velocity_level",
    "stop_manual_control", "arm", "disarm", "disconnect", "get_velocity",
])
def test_real_only_methods_present_on_real_like(make_drone, name):
    d = make_drone(0, real_like=True)
    assert hasattr(d, name)


def test_obstacles_has_no_up_field(drone):
    obs = drone.get_obstacles()
    assert not hasattr(obs, "up")
    assert set(vars(obs)) == {"forward", "back", "left", "right", "down"}


def test_connect_requires_ip(world):
    fpx.set_active_world(world)
    d = fpx.FakeDroneAPI(world)
    with pytest.raises(TypeError):
        d.connect()                                        # ip is required


def test_hover_requires_duration(drone):
    with pytest.raises(TypeError):
        drone.hover()                                      # duration is required


def test_connect_returns_truthy_command_result(world):
    fpx.set_active_world(world)
    d = fpx.FakeDroneAPI(world)
    res = d.connect(world.ip_map[0])
    assert bool(res) is True


# --------------------------------------------------------------------------- #
# imports resolve to the fakes (sys.modules wiring)
# --------------------------------------------------------------------------- #
def test_pyhulax_import_resolves_to_fake():
    import pyhulax
    from pyhulax.core import CameraPitchMode, Direction, BarrierMask
    from UWBParserThread import UWBParserThread
    assert pyhulax.DroneAPI is fpx.DroneAPI
    assert UWBParserThread is fuwb.FakeUWBParserThread
    assert int(BarrierMask.HORIZONTAL) == 1
    assert int(Direction.FORWARD) == 0 and int(CameraPitchMode.DOWN_ABSOLUTE) == 1
