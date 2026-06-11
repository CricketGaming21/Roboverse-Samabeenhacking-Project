"""R2 — phase-2 search/read tuned for the MOVING GIMBAL.

The sim (and the fake here) model each rover's ArUco marker on a gimbal whose facing yaw
sweeps; a drone DECODES it only when the facing is within ±halfangle of the rover->drone
bearing (never from directly overhead). Out of the cone the camera sees the rover BODY but
cv2.aruco finds no marker. These tests cover the fake's gimbal model + the mission's
persistence/search-pitch behaviour built on top of it.
"""

import math

import pytest

from mission.perception.aruco import confirm_with_aruco
from mission.perception.detector import ClassicalRoverDetector, frame_bgr
from tests.fakes import fake_pyhulax as fpx
from tests.fakes.fake_pyhulax import Marker

pytestmark = pytest.mark.r2

DICT = "DICT_6X6_250"


# --------------------------------------------------------------------------- #
# config: search pitch + persistence knobs
# --------------------------------------------------------------------------- #
def test_config_search_pitch_and_persistence_defaults():
    from mission.config import load_config, load_real_config
    cfg = load_config()
    assert 45.0 <= cfg.camera.search_pitch_deg <= 60.0     # a MODERATE tilt, not nadir
    assert cfg.camera.use_nadir_search is False            # baseline off by default
    assert cfg.search.persist_timeout_s >= 8.0             # holds past one full gimbal sweep
    assert cfg.search.orbit_step_m <= cfg.speed.cruise_alt_m + 1e-9   # light, ≤ cruise
    # the real profile (no search section / new camera keys) still loads on the defaults
    rcfg = load_real_config()
    assert rcfg.camera.use_nadir_search is False
    assert rcfg.search.persist_timeout_s >= 8.0


def _world(rover: Marker, *, gimbal: bool = True) -> fpx.FakeWorld:
    w = fpx.FakeWorld()
    w.gimbal_enabled = gimbal
    w.pads = []
    w.rovers = [rover]
    return w


def _drone_at(w: fpx.FakeWorld, n: float, e: float, *, alt_cm: float = 110.0,
              yaw_deg: float = 0.0, pitch_down: float = 52.0):
    fpx.set_active_world(w)
    d = fpx.FakeDroneAPI(w)
    d.connect(w.ip_map[0])
    d.n, d.e, d.alt_cm, d.yaw_deg = n, e, alt_cm, yaw_deg
    d.pitch_down_deg = pitch_down
    d.set_video_stream(True)
    s = d.create_video_stream()
    s.start()
    return d, s


def _decode_ids(stream) -> set:
    return {det.marker_id for det in confirm_with_aruco(frame_bgr(stream.latest_frame), DICT)}


# --------------------------------------------------------------------------- #
# the fake's gimbal model
# --------------------------------------------------------------------------- #
def test_gimbal_off_marker_always_decodes():
    """Backward-compat: with the gimbal disabled (the default), an offset drone decodes
    the rover marker exactly as before — existing worlds are unchanged."""
    w = _world(Marker(45, 4.0, 2.0, size_m=0.20), gimbal=False)
    _d, s = _drone_at(w, 3.0, 2.0)
    assert 45 in _decode_ids(s)


def test_gimbal_out_of_cone_hides_marker_shows_body():
    """Marker faces north (yaw 0) at t=0; a drone offset SOUTH is on bearing ~180° → out of
    the ±60° cone → cv2.aruco gets nothing, but the rover BODY is visible (presence)."""
    w = _world(Marker(45, 4.0, 2.0, size_m=0.20, gimbal_phase_deg=0.0))
    d, s = _drone_at(w, 3.0, 2.0)
    assert w.marker_readable(w.rovers[0], (d.n, d.e)) is False
    assert 45 not in _decode_ids(s)                              # cannot decode out of cone
    assert ClassicalRoverDetector().detect(frame_bgr(s.latest_frame))   # body blob is seen


def test_gimbal_overhead_is_never_readable():
    """Directly overhead the facing is undefined → never readable (nadir loses the marker)."""
    w = _world(Marker(45, 4.0, 2.0, size_m=0.20))
    d, _s = _drone_at(w, 4.0, 2.0)
    assert w.marker_readable(w.rovers[0], (d.n, d.e)) is False


def test_gimbal_sweep_brings_marker_into_cone():
    """Holding a steady offset, the sweeping facing rotates into the cone → decodes."""
    w = _world(Marker(45, 4.0, 2.0, size_m=0.20, gimbal_phase_deg=0.0))
    d, s = _drone_at(w, 3.0, 2.0)
    assert 45 not in _decode_ids(s)                              # out of cone at t=0
    w.clock = 4.0                                                # 45°/s · 4 s = 180° → bearing south
    assert w.marker_readable(w.rovers[0], (d.n, d.e)) is True
    assert 45 in _decode_ids(s)                                  # now decodable
