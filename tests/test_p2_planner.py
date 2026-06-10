"""P2 — planner geometry (inflate / visibility graph / plan / conflict) + projection."""

import math

import cv2
import numpy as np
import pytest
import yaml

from mission.planner.arena import load_arena
from mission.planner.geometry import (Rect, build_graph, inflate, paths_conflict,
                                       plan_path, point_blocked, segment_clear)
from mission.planner.projection import (CameraIntrinsics, arena_to_pixel,
                                        pixel_to_arena)
from tests.fakes import fake_pyhulax as fpx

pytestmark = pytest.mark.p2


# --------------------------------------------------------------------------- #
# arena_truth loader — reads the sim's `structures:` key (emit_arena_truth)
# --------------------------------------------------------------------------- #
def test_load_arena_reads_structures_key(tmp_path):
    """The overhauled sim emits footprints under `structures:` (crates + arch posts).
    load_arena must read them as crate footprints so the planner routes around them."""
    p = tmp_path / "a.yaml"
    p.write_text(yaml.safe_dump({
        "arena": {"length_m": 10.0, "width_m": 6.0, "height_m": 3.0},
        "structures": [{"center": [4.7, 2.3], "size": [0.42, 0.42], "height": 1.1},
                       {"center": [7.0, 4.0], "size": [1.0, 0.42], "height": 0.34}],
        "landing_zones": [{"id": 11, "north": 4.4, "east": 1.35, "valid": True}],
    }))
    arena = load_arena(p)
    assert arena.length_m == 10.0 and arena.width_m == 6.0
    assert arena.footprint_tuples() == [(4.7, 2.3, 0.42, 0.42), (7.0, 4.0, 1.0, 0.42)]


def test_load_arena_legacy_crates_key_still_read(tmp_path):
    """Back-compat: an older arena file using `crates:` (e.g. the open-cage real file)
    still loads — empty when neither key is present."""
    p = tmp_path / "a.yaml"
    p.write_text(yaml.safe_dump({
        "arena": {"length_m": 11.0, "width_m": 11.0},
        "crates": [{"center": [5.0, 3.0], "size": [0.45, 0.45], "height": 0.6}],
    }))
    arena = load_arena(p)
    assert arena.footprint_tuples() == [(5.0, 3.0, 0.45, 0.45)]


# --------------------------------------------------------------------------- #
# inflation
# --------------------------------------------------------------------------- #
def test_inflate_expands_each_side():
    r = inflate([(5.0, 3.0, 0.9, 0.9)], 0.4)[0]
    assert (r.min_n, r.min_e, r.max_n, r.max_e) == pytest.approx((4.15, 2.15, 5.85, 3.85))


def test_point_blocked():
    rects = inflate([(5.0, 3.0, 0.9, 0.9)], 0.4)
    assert point_blocked((5.0, 3.0), rects) is True
    assert point_blocked((0.0, 0.0), rects) is False


# --------------------------------------------------------------------------- #
# segment clearance
# --------------------------------------------------------------------------- #
def test_segment_through_crate_not_clear():
    rects = inflate([(5.0, 3.0, 0.9, 0.9)], 0.4)
    assert segment_clear((3.0, 3.0), (7.0, 3.0), rects) is False   # straight through


def test_segment_around_crate_clear():
    rects = inflate([(5.0, 3.0, 0.9, 0.9)], 0.4)
    assert segment_clear((3.0, 0.5), (7.0, 0.5), rects) is True    # passes south of it


def test_segment_tangent_to_boundary_allowed():
    rects = inflate([(5.0, 3.0, 0.9, 0.9)], 0.4)               # max_e = 3.85
    assert segment_clear((3.0, 3.86), (7.0, 3.86), rects) is True


# --------------------------------------------------------------------------- #
# visibility graph + planning
# --------------------------------------------------------------------------- #
def _path_is_clear(path, inflated, bounds):
    assert path is not None
    for p in path:
        assert bounds.contains(p)
    for i in range(len(path) - 1):
        assert segment_clear(path[i], path[i + 1], inflated)


def test_plan_routes_around_single_crate():
    bounds = Rect(0, 0, 10, 6)
    inflated = inflate([(5.0, 3.0, 0.9, 0.9)], 0.4)
    g = build_graph(inflated, bounds)
    path = plan_path((2.0, 3.0), (8.0, 3.0), g)            # straight line is blocked
    _path_is_clear(path, inflated, bounds)
    assert path[0] == (2.0, 3.0) and path[-1] == (8.0, 3.0)
    assert len(path) >= 3                                  # had to detour


def test_plan_straight_when_clear():
    bounds = Rect(0, 0, 10, 6)
    inflated = inflate([(5.0, 3.0, 0.9, 0.9)], 0.4)
    g = build_graph(inflated, bounds)
    path = plan_path((1.0, 0.5), (9.0, 0.5), g)            # clear corridor
    assert path == [(1.0, 0.5), (9.0, 0.5)]


def test_plan_none_when_walled_off():
    bounds = Rect(0, 0, 10, 6)
    inflated = inflate([(5.0, 3.0, 0.2, 6.0)], 0.4)        # wall spanning full width
    g = build_graph(inflated, bounds)
    assert plan_path((1.0, 3.0), (9.0, 3.0), g) is None


def test_plan_none_when_goal_blocked():
    bounds = Rect(0, 0, 10, 6)
    inflated = inflate([(5.0, 3.0, 0.9, 0.9)], 0.4)
    g = build_graph(inflated, bounds)
    assert plan_path((1.0, 0.5), (5.0, 3.0), g) is None    # goal inside footprint


def test_plan_on_real_arena_truth():
    arena = load_arena()
    bounds = Rect(0, 0, arena.length_m, arena.width_m)
    inflated = inflate(arena.footprint_tuples(), 0.4)
    g = build_graph(inflated, bounds)
    path = plan_path((0.6, 1.1), (8.5, 3.0), g)
    _path_is_clear(path, inflated, bounds)


# --------------------------------------------------------------------------- #
# inter-drone conflict
# --------------------------------------------------------------------------- #
def test_paths_conflict_when_crossing():
    a = [(0.0, 0.0), (4.0, 4.0)]
    b = [(0.0, 4.0), (4.0, 0.0)]                           # crosses A
    assert paths_conflict(a, b, sep_m=0.8) is True


def test_paths_no_conflict_when_separated():
    a = [(0.0, 0.0), (10.0, 0.0)]
    b = [(0.0, 3.0), (10.0, 3.0)]                          # 3 m apart, parallel
    assert paths_conflict(a, b, sep_m=0.8) is False


def test_paths_conflict_when_too_close():
    a = [(0.0, 0.0), (10.0, 0.0)]
    b = [(0.0, 0.5), (10.0, 0.5)]                          # 0.5 m apart < 0.8
    assert paths_conflict(a, b, sep_m=0.8) is True


# --------------------------------------------------------------------------- #
# projection
# --------------------------------------------------------------------------- #
INTR = CameraIntrinsics(640, 480, 71.0)


@pytest.mark.parametrize("drone,yaw,alt,pitch,pt", [
    ((3.0, 2.0), 0.0, 1.5, 90.0, (3.0, 2.0)),
    ((3.0, 2.0), 0.0, 1.5, 90.0, (3.4, 2.3)),
    ((3.0, 2.0), 30.0, 1.5, 90.0, (2.7, 1.6)),
    ((3.0, 2.0), 0.0, 1.5, 60.0, (4.2, 2.0)),
    ((3.0, 2.0), 15.0, 1.8, 65.0, (4.5, 2.6)),
])
def test_projection_round_trip(drone, yaw, alt, pitch, pt):
    px = arena_to_pixel(pt, drone, yaw, alt, pitch, INTR)
    assert px is not None
    back = pixel_to_arena(px[0], px[1], drone, yaw, alt, pitch, INTR)
    assert back == pytest.approx(pt, abs=1e-6)


def test_pixel_to_arena_matches_rendered_marker(world):
    """End-to-end: render a real marker, decode its centre, back-project to its
    known arena position. Ties projection.py to the fake camera model."""
    fpx.set_active_world(world)
    d = fpx.FakeDroneAPI(world)
    d.connect(world.ip_map[0])
    d.n, d.e, d.alt_cm = 8.2, 2.8, 110.0                   # offset from pad 10 @ (8.5,3.0)
    d.set_camera_angle(fpx.CameraPitchMode.DOWN_ABSOLUTE, 90)
    d.set_video_stream(True)
    s = d.create_video_stream()
    s.start()
    frame = s.latest_frame.to_rgb()
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    det = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250),
        cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(gray)
    flat = list(ids.flatten())
    quad = corners[flat.index(10)][0]
    cu, cv = float(quad[:, 0].mean()), float(quad[:, 1].mean())
    est = pixel_to_arena(cu, cv, (d.n, d.e), d.yaw_deg, d.alt_cm / 100.0,
                         d.pitch_down_deg, INTR)
    assert est == pytest.approx((8.5, 3.0), abs=0.1)
