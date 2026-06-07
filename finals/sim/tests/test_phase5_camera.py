"""Phase 5 acceptance test — camera rendering + ArUco pads (headless).

A drone above a pad with the camera pitched DOWN must yield frames on which
cv2.aruco decodes the pad's CONFIGURED id; at pitch 0 (forward) the pad
directly below must NOT be in view. All rendering goes through the sim-thread
queue. NO landing logic anywhere — that's the mission project's job.
"""

import time

import cv2
import numpy as np
import pytest

from pyhulax import DroneAPI
from pyhulax.core import CameraPitchMode, Direction

from simcore import aruco_assets
from simcore.config import load_config
from simcore.registry import get_registry, shutdown_registry

# Pad 10 sits at arena (north=3.0, east=1.5) — dead ahead of drone 0's start
# (0.6, 1.5, heading north): move_to(0, 240, z) parks the drone above it.
PAD_ID = 10


@pytest.fixture()
def cfg():
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0
    c.arena.obstacles.count = 0  # clean sight lines, deterministic views
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


def _detect_ids(cfg, rgb):
    """Decode marker ids exactly like the organiser sample (modern API)."""
    detector = cv2.aruco.ArucoDetector(
        aruco_assets.get_dictionary(cfg.aruco.dictionary),
        cv2.aruco.DetectorParameters())
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    found = [] if ids is None else [int(i) for i in ids.flatten()]
    return found, corners


def _wait_frame(stream, timeout_s=5.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        f = stream.latest_frame
        if f is not None:
            return f
        time.sleep(0.01)
    pytest.fail("no video frame within timeout")


def _stream_over_pad(cfg, d, pitch_deg=90):
    d.takeoff(150)
    d.move_to(0, 240, 150)  # above pad 10
    d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, pitch_deg)
    stream = d.create_video_stream()
    d.set_video_stream(True)
    stream.start()
    return stream


# --------------------------------------------------------------------------- #
# Marker detection from rendered frames
# --------------------------------------------------------------------------- #

def test_pad_marker_detected_when_pitched_down(sim, cfg):
    d = _connect(cfg)
    stream = _stream_over_pad(cfg, d, pitch_deg=90)
    try:
        frame = _wait_frame(stream)
        ids, corners = _detect_ids(cfg, frame.to_rgb())
        assert PAD_ID in ids, f"pad id {PAD_ID} not decoded (saw {ids})"
        quad = corners[ids.index(PAD_ID)]
        assert quad.shape == (1, 4, 2)  # exactly 4 corners
    finally:
        stream.stop()


def test_pitch_matters(sim, cfg):
    """The pad directly below is NOT in view at pitch 0, IS at pitch 90."""
    d = _connect(cfg)
    stream = _stream_over_pad(cfg, d, pitch_deg=0)  # forward-looking
    try:
        # Give the stream a few frame periods, then check: no pad in view.
        time.sleep(0.2)
        frame = _wait_frame(stream)
        ids, _ = _detect_ids(cfg, frame.to_rgb())
        assert PAD_ID not in ids

        d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 90)
        deadline = time.time() + 5.0
        while time.time() < deadline:  # wait for a post-tilt frame
            ids, _ = _detect_ids(cfg, stream.latest_frame.to_rgb())
            if PAD_ID in ids:
                break
            time.sleep(0.02)
        else:
            pytest.fail("pad never detected after pitching down")
    finally:
        stream.stop()


# --------------------------------------------------------------------------- #
# Frame format contract
# --------------------------------------------------------------------------- #

def test_frame_format_and_channel_order(sim, cfg):
    d = _connect(cfg)
    stream = _stream_over_pad(cfg, d, pitch_deg=90)
    try:
        frame = _wait_frame(stream)
        rgb, bgr = frame.to_rgb(), frame.to_bgr()
        assert rgb.shape == (cfg.camera.height, cfg.camera.width, 3)
        assert rgb.dtype == np.uint8
        assert frame.width == cfg.camera.width
        assert frame.height == cfg.camera.height
        assert np.array_equal(bgr, rgb[:, :, ::-1])  # exact channel reversal
        assert not np.array_equal(bgr, rgb)          # scene isn't channel-symmetric
        rgb[:] = 0                                   # to_rgb returns a copy:
        assert frame.to_rgb().any()                  # mutations don't leak back
    finally:
        stream.stop()


# --------------------------------------------------------------------------- #
# Camera pitch modes + threading behaviour
# --------------------------------------------------------------------------- #

def test_camera_pitch_modes(sim, cfg):
    d = _connect(cfg)
    drone = sim.drones[0]

    def pitch():
        return sim.run_on_sim_thread(lambda: drone.camera_pitch_deg)

    assert d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 50).success
    assert pitch() == 50.0
    d.set_camera_angle(CameraPitchMode.DOWN_RELATIVE, 20)
    assert pitch() == 70.0
    d.set_camera_angle(CameraPitchMode.UP_RELATIVE, 80)
    assert pitch() == 0.0                       # clamps at the horizon
    d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 120)
    assert pitch() == 90.0                      # clamps straight down
    d.set_camera_angle(CameraPitchMode.CALIBRATE)
    assert pitch() == cfg.camera.default_pitch_deg


def test_rendering_streams_during_flight(sim, cfg):
    """Frames keep flowing from the sim thread while motion goals execute —
    no threading errors, command still completes."""
    d = _connect(cfg)
    d.takeoff(100)
    stream = d.create_video_stream()
    d.set_video_stream(True)
    stream.start()
    try:
        _wait_frame(stream)
        r = d.move(Direction.FORWARD, 200, blocking=False)
        assert r.success
        frames_seen = 0
        deadline = time.time() + 1.5
        while time.time() < deadline:
            if stream.latest_frame is not None:
                frames_seen += 1
            time.sleep(0.03)
        assert frames_seen >= 5
        assert stream.fps > 0.0
        deadline = time.time() + 5.0  # the concurrent move still completes
        while time.time() < deadline:
            if sim.run_on_sim_thread(lambda: sim.drones[0].goal is None):
                break
            time.sleep(0.02)
        else:
            pytest.fail("move did not complete while streaming")
    finally:
        stream.stop()


# --------------------------------------------------------------------------- #
# Asset generation
# --------------------------------------------------------------------------- #

def test_gen_assets_pngs_decode(cfg, tmp_path):
    paths = aruco_assets.generate_all(cfg, out_dir=str(tmp_path))
    expected = {p.id for p in cfg.pads} | set(cfg.rovers.marker_ids)
    assert set(paths) == expected
    for marker_id, path in paths.items():
        img = cv2.imread(path)
        assert img is not None
        # white quiet zone present (corners are white)
        assert img[0, 0].min() >= 250
        ids, _ = _detect_ids(cfg, cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        assert ids == [marker_id]
