"""Tests for the staged bring-up scripts (connect_check / hover_test / camera_check /
yaw_calibrate) against the fakes — motion-gating, no-drift, read-only, land-on-exit."""

import math

import pytest

from scripts import camera_check, connect_check, hover_test, yaw_calibrate
from tests.fakes import fake_pyhulax as fpx
from tests.fakes import fake_uwb as fuwb
from tests.fakes.fake_pyhulax import CameraPitchMode, Marker

NOSLEEP = lambda _s: None


def _silent(*_a, **_k):
    pass


# --------------------------------------------------------------------------- #
# connect_check — READ-ONLY
# --------------------------------------------------------------------------- #
def test_connect_check_telemetry_no_motion(world):
    uwb = fuwb.FakeUWBParserThread(world=world)
    created = []

    def mk():
        d = fpx.FakeDroneAPI(world)
        created.append(d)
        return d

    checks = connect_check.connect_and_telemetry(world.ip_map, uwb=uwb, make_api=mk,
                                                 log=_silent)
    assert all(c.passed for c in checks.values()) and len(checks) == 3
    for d in created:                                    # READ-ONLY: never moved/armed
        assert d.manual_calls == 0 and d.alt_cm == 0.0


def test_connect_check_live_uwb_prints(world):
    uwb = fuwb.FakeUWBParserThread(world=world)
    lines = []
    connect_check.live_uwb(uwb, [0, 1, 2], reads=3, sleep=NOSLEEP, log=lines.append)
    assert len(lines) == 3 and "tag0=" in lines[0]


# --------------------------------------------------------------------------- #
# hover_test — first arming test; gated; no drift; lands
# --------------------------------------------------------------------------- #
def test_hover_once_takes_off_holds_and_lands(drone):
    n0, e0 = drone.n, drone.e
    hover_test.hover_once(drone, height_cm=100, hover_s=1.0, sleep=NOSLEEP, log=_silent)
    assert drone.manual_calls == 20                      # 1.0 s × 20 Hz of station-keep
    assert (drone.n, drone.e) == (n0, e0)                # zero horizontal drift
    assert drone.get_altitude() == 0.0                   # landed at the end


def test_hover_test_refuses_without_clear_space():
    assert hover_test.main(["--ip", "10.0.0.11"]) == 1   # no --i-have-clear-space → refuse


# --------------------------------------------------------------------------- #
# camera_check — READ-ONLY ArUco on live frames
# --------------------------------------------------------------------------- #
def test_camera_check_decodes_marker_no_motion():
    w = fpx.FakeWorld(crates=[])
    w.rovers = [Marker(23, 5.0, 3.0, size_m=0.20)]
    fpx.set_active_world(w)
    d = fpx.FakeDroneAPI(w)
    d.connect(w.ip_map[0])
    d.n, d.e, d.alt_cm = 5.0, 3.0, 110.0
    d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 90)
    s = d.create_video_stream()
    seen = camera_check.scan_frames(d, s, frames=3, sleep=NOSLEEP, log=_silent)
    assert 23 in {mid for mid, _px in seen}
    assert all(px > 0 for _mid, px in seen)
    assert d.manual_calls == 0 and d.alt_cm == 110.0     # READ-ONLY: no motion
    fpx.set_active_world(None)


# --------------------------------------------------------------------------- #
# yaw_calibrate — nudge forward, report UWB delta, land
# --------------------------------------------------------------------------- #
def test_yaw_calibrate_nudge_moves_north_and_lands(world):
    d = fpx.FakeDroneAPI(world)
    d.connect(world.ip_map[0])
    d.n, d.e, d.yaw_deg = 3.0, 3.0, 0.0                   # yaw 0 → forward should be +north
    u = fuwb.FakeUWBParserThread(world=world)
    before, after = yaw_calibrate.nudge_forward(d, u, 0, secs=1.0, sleep=NOSLEEP, log=_silent)
    assert after[0] > before[0] + 0.1                    # moved +north
    assert abs(after[1] - before[1]) < 1e-6              # no east drift
    assert d.get_altitude() == 0.0                       # landed
    fpx.set_active_world(None)


def test_yaw_calibrate_refuses_without_clear_space():
    assert yaw_calibrate.main(["--ip", "10.0.0.11"]) == 1
