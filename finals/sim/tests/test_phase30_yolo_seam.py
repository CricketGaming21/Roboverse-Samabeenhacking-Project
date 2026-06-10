"""Phase 30 acceptance test — YOLO integration seam (mission-side, headless).

Proves: the example runs against the sim through the PUBLIC pyhulax VideoStream
and the RoverDetector interface; with the dummy detector it returns candidates
and the two-stage pattern is exercised end to end (YOLO-stub box -> real
cv2.aruco identity confirm); the boundary holds (NO yolo/ultralytics import in
simcore, the example imports no simcore); and the public surface is unchanged.
"""

import ast
import inspect
import pathlib
import time

import cv2
import numpy as np
import pytest

from pyhulax import DroneAPI
from pyhulax.core import CameraPitchMode

from mission_examples import rover_detection_example as ex
from simcore import frames
from simcore.config import load_config
from simcore.registry import get_registry, shutdown_registry


def _cfg():
    """A world where rovers sit PARKED in-arena from t=0 (phases=ambush +
    patrol speed 0), so a drone can be positioned over a known rover marker
    deterministically for the confirm stage."""
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 12.0
    c.camera.use_egl = False
    c.scoring.enabled = False
    c.scenario.phases = "ambush"
    c.rovers.motion = "patrol"
    c.rovers.patrol.speed_mps = 0.0
    return c


# --------------------------------------------------------------------------- #
# Stage 1 — the placeholder detector + the RoverDetector interface
# --------------------------------------------------------------------------- #

def test_placeholder_detector_implements_interface_and_returns_candidates():
    det = ex.PlaceholderRoverDetector()
    assert isinstance(det, ex.RoverDetector)
    frame = np.full((480, 640, 3), 120, np.uint8)
    cands = det.detect(frame)
    assert len(cands) >= 1
    c = cands[0]
    assert c.source == "yolo" and c.marker_id is None and not c.confirmed
    assert c.bbox[2] > 0 and c.bbox[3] > 0 and 0.0 <= c.conf <= 1.0
    # it is clearly a placeholder a YOLO model replaces
    assert "PLACEHOLDER" in inspect.getsource(ex.PlaceholderRoverDetector)


# --------------------------------------------------------------------------- #
# Stage 2 — REAL cv2.aruco confirm, and the full two-stage flow on a frame
# --------------------------------------------------------------------------- #

def _synthetic_marker_frame(marker_id=51, dict_name="DICT_7X7_1000"):
    dic = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    img = np.full((480, 640), 255, np.uint8)
    img[140:340, 220:420] = cv2.aruco.generateImageMarker(dic, marker_id, 200)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)


def test_aruco_confirm_decodes_real_id():
    confirmed = ex.confirm_with_aruco(_synthetic_marker_frame(45))
    assert [c.marker_id for c in confirmed] == [45]
    assert confirmed[0].source == "aruco" and confirmed[0].confirmed


class _OneFrameStream:
    """Minimal stand-in with the public VideoStream surface used by the seam
    (latest_frame.to_rgb()), so the two-stage flow can be unit-tested."""
    class _F:
        def __init__(self, rgb):
            self._rgb = rgb

        def to_rgb(self):
            return self._rgb
    def __init__(self, rgb):
        self.latest_frame = self._F(rgb)


def test_two_stage_flow_yolo_stub_then_aruco_confirm_with_approach():
    approached = {"called": False}

    def approach(cands):
        approached["called"] = True
        assert cands and cands[0].source == "yolo"   # given the stage-1 boxes

    stream = _OneFrameStream(_synthetic_marker_frame(11))
    res = ex.two_stage_scan(stream, ex.PlaceholderRoverDetector(),
                            approach=approach)
    assert approached["called"]                       # APPROACH ran
    assert res.candidates and res.candidates[0].marker_id is None  # STAGE 1
    assert 11 in res.confirmed_ids                    # STAGE 2 (real cv2.aruco)


# --------------------------------------------------------------------------- #
# End to end against the SIM via the public VideoStream
# --------------------------------------------------------------------------- #

def test_example_runs_end_to_end_against_sim_public_videostream():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        # a parked rover with a known marker id, at a known arena position
        rover = reg.run_on_sim_thread(
            lambda: (reg.rovers[0].marker_id, reg.rovers[0].arena_position()))
        marker_id, (n, e) = rover

        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        d.set_video_stream(True)
        stream = d.create_video_stream()
        stream.start()
        d.takeoff(100)
        # position above the rover, camera straight down (public API)
        fr = reg.run_on_sim_thread(lambda: reg.drones[0].takeoff_frame)
        x, y, _ = frames.arena_to_takeoff_cm(cfg, fr, n, e)
        d.move_to(x, y, 150)
        d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 90)

        # a real frame pulled through the public stream interface
        frame = ex.grab_frame(stream, timeout_s=3.0)
        assert frame is not None and frame.ndim == 3

        # hover-and-scan a few times so fresh frames render at the new pose
        # (a realistic "held a few frames" capture, robust to render latency)
        detector = ex.PlaceholderRoverDetector()
        res = ex.two_stage_scan(stream, detector)
        for _ in range(20):
            if marker_id in res.confirmed_ids:
                break
            time.sleep(0.15)
            res = ex.two_stage_scan(stream, detector)
        # STAGE 1 (YOLO stub) proposed candidates...
        assert res.candidates and res.candidates[0].source == "yolo"
        # ...STAGE 2 (real cv2.aruco) confirmed the actual ROVER id up close
        assert marker_id in res.confirmed_ids
        assert marker_id in ex.ROVER_ID_RANGE
        stream.stop()
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Boundary — YOLO is mission-side only; example never imports simcore
# --------------------------------------------------------------------------- #

def _import_lines(path):
    src = pathlib.Path(path).read_text()
    out = []
    for node in ast.walk(ast.parse(src)):
        if isinstance(node, ast.Import):
            out += [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            out.append(node.module or "")
    return out


def test_no_yolo_or_ultralytics_import_in_simcore():
    simcore_dir = pathlib.Path(inspect.getfile(
        __import__("simcore"))).parent
    banned = ("yolo", "ultralytics", "rknn")
    for py in simcore_dir.rglob("*.py"):
        for mod in _import_lines(py):
            low = mod.lower()
            assert not any(b in low for b in banned), \
                f"{py.name} imports a YOLO/RKNN module ({mod}) — boundary break"


def test_example_imports_pyhulax_not_simcore():
    mods = _import_lines(inspect.getfile(ex))
    assert any(m == "pyhulax" or m.startswith("pyhulax")
               for m in mods)            # uses the PUBLIC SDK
    assert not any(m == "simcore" or m.startswith("simcore.")
                   for m in mods)        # never the sim internals


def test_no_new_public_surface():
    import pyhulax
    import pyhulax.core
    from UWBParserThread import UWBParserThread
    public = (set(dir(pyhulax)) | set(dir(pyhulax.DroneAPI))
              | set(dir(pyhulax.core)) | set(dir(UWBParserThread)))
    leaked = [n for n in public if "yolo" in n.lower()
              or "rover_detect" in n.lower() or "detector" in n.lower()]
    assert leaked == []
