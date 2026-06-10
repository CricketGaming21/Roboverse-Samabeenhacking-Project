"""P5 — ArUco confirm (multi-marker, tilted, px gate, allow-list) + classical finder +
two_stage_scan seam."""

import cv2
import numpy as np
import pytest

from mission.perception.aruco import confirm_with_aruco, is_rover_id
from mission.perception.detector import (ClassicalRoverDetector, Detection,
                                         PlaceholderRoverDetector, two_stage_scan)
from tests.fakes import fake_pyhulax as fpx
from tests.fakes.fake_pyhulax import CameraPitchMode, Marker

pytestmark = pytest.mark.p5

NOSLEEP = lambda _s: None
REAL_ROVERS = [11, 45, 51, 67, 101]
SIM_ROVERS = [20, 21, 22, 23, 24]


def _looking_down_drone(world, n, e, alt_cm, pitch_down=90.0, yaw=0.0):
    fpx.set_active_world(world)
    d = fpx.FakeDroneAPI(world)
    d.connect(world.ip_map[0])
    d.n, d.e, d.alt_cm, d.yaw_deg = n, e, alt_cm, yaw
    d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, pitch_down)
    d.set_video_stream(True)
    s = d.create_video_stream()
    s.start()
    return d, s


# --------------------------------------------------------------------------- #
# id allow-list (config-driven; NO pad deny-list)
# --------------------------------------------------------------------------- #
def test_is_rover_id_is_an_allow_list():
    # real profile: id 11 is a ROVER (was wrongly excluded by the old 10–14 pad deny-list)
    assert is_rover_id(11, REAL_ROVERS) is True
    assert is_rover_id(10, REAL_ROVERS) is False         # a decoded id NOT in the list is ignored
    # sim profile: only the listed ids count
    assert is_rover_id(20, SIM_ROVERS) is True
    assert is_rover_id(11, SIM_ROVERS) is False
    assert is_rover_id(99, SIM_ROVERS) is False


def test_detector_is_dictionary_agnostic():
    # same confirm logic decodes both the sim and real dictionaries
    for dname in ("DICT_6X6_250", "DICT_7X7_1000"):
        d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dname))
        img = np.full((240, 240, 3), 255, np.uint8)
        img[20:220, 20:220] = cv2.cvtColor(
            cv2.aruco.generateImageMarker(d, 11, 200), cv2.COLOR_GRAY2BGR)
        ids = {x.marker_id for x in confirm_with_aruco(img, dname)}
        assert 11 in ids                                 # id 11 decodes in both dictionaries


# --------------------------------------------------------------------------- #
# aruco confirm
# --------------------------------------------------------------------------- #
def test_confirm_decodes_multiple_markers_in_one_frame():
    w = fpx.FakeWorld(crates=[])
    w.pads = [Marker(10, 4.0, 2.5), Marker(11, 4.0, 3.5)]
    d, s = _looking_down_drone(w, 4.0, 3.0, 150.0)
    dets = confirm_with_aruco(s.latest_frame.to_rgb())
    ids = {x.marker_id for x in dets}
    assert {10, 11} <= ids
    for x in dets:
        assert x.source == "aruco" and x.confirmed and x.conf == 1.0
        assert len(x.bbox) == 4 and x.bbox[2] > 0 and x.bbox[3] > 0
    fpx.set_active_world(None)


def test_confirm_decodes_tilted_marker():
    w = fpx.FakeWorld(crates=[])
    w.pads = [Marker(12, 5.0, 3.0)]
    d, s = _looking_down_drone(w, 4.4, 3.0, 120.0, pitch_down=63.0)
    ids = {x.marker_id for x in confirm_with_aruco(s.latest_frame.to_rgb())}
    assert 12 in ids
    fpx.set_active_world(None)


def test_min_marker_px_gate():
    w = fpx.FakeWorld(crates=[])
    w.pads = [Marker(10, 8.5, 3.0)]
    d, s = _looking_down_drone(w, 8.5, 3.0, 110.0)
    frame = s.latest_frame.to_rgb()
    assert 10 in {x.marker_id for x in confirm_with_aruco(frame, min_marker_px=0)}
    assert confirm_with_aruco(frame, min_marker_px=2000) == []   # too-big gate filters all
    fpx.set_active_world(None)


# --------------------------------------------------------------------------- #
# classical finder
# --------------------------------------------------------------------------- #
def test_classical_detector_finds_rendered_rover():
    w = fpx.FakeWorld(crates=[])
    w.rovers = [Marker(22, 5.0, 3.0, size_m=0.20)]
    d, s = _looking_down_drone(w, 5.0, 3.0, 110.0)
    boxes = ClassicalRoverDetector(min_area_px=300).detect(s.latest_frame.to_rgb())
    assert len(boxes) >= 1
    # the strongest box should be near the image centre (drone is over the rover)
    x, y, bw, bh = boxes[0].bbox
    cx, cy = x + bw / 2, y + bh / 2
    assert abs(cx - 320) < 120 and abs(cy - 240) < 120
    assert boxes[0].source == "classical" and boxes[0].marker_id is None
    fpx.set_active_world(None)


def test_placeholder_detector_returns_centre_box():
    import numpy as np
    boxes = PlaceholderRoverDetector().detect(np.zeros((480, 640, 3), np.uint8))
    assert len(boxes) == 1 and boxes[0].marker_id is None


# --------------------------------------------------------------------------- #
# two-stage seam
# --------------------------------------------------------------------------- #
def test_two_stage_scan_wires_stages_and_confirms_id():
    w = fpx.FakeWorld(crates=[])
    w.rovers = [Marker(23, 6.0, 3.0, size_m=0.20)]
    d, s = _looking_down_drone(w, 6.0, 3.0, 110.0, pitch_down=0.0)  # start looking forward

    def approach(_cands):
        d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 90)       # close in / look down

    result = two_stage_scan(s, PlaceholderRoverDetector(), approach=approach, sleep=NOSLEEP)
    assert len(result.candidates) == 1                              # stage 1 proposed a box
    assert 23 in result.confirmed_ids                               # stage 2 confirmed the id
    assert is_rover_id(23, SIM_ROVERS)
    fpx.set_active_world(None)
