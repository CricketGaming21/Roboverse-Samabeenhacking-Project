"""Phase 6 acceptance test — moving rovers + unique ArUco markers (headless).

Proves: rovers patrol (advance, stay inside bounds), every rover carries its
distinct CONFIGURED marker id facing UP, and a drone at a sane scan pose
(directly above, pitched down, 1.5 m altitude) decodes that id with the
marker projecting >= scoring.min_marker_px. NO search/lock-on/scoring here —
that's Phase 7 / the mission project.
"""

import math
import time

import cv2
import pytest

from pyhulax import DroneAPI
from pyhulax.core import CameraPitchMode

from simcore import aruco_assets
from simcore.config import load_config
from simcore.registry import get_registry, shutdown_registry


def _base_cfg():
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0
    return c


@pytest.fixture()
def cfg_move():
    """Default world (obstacles in), no rendering — patrol behaviour."""
    c = _base_cfg()
    c.camera.use_egl = False
    return c


@pytest.fixture()
def cfg_scan():
    """Parked rovers + clean sight lines — scan-geometry checks."""
    c = _base_cfg()
    c.rovers.patrol.speed_mps = 0.0
    c.arena.obstacles.count = 0
    return c


def _detect(cfg, rgb):
    detector = cv2.aruco.ArucoDetector(
        aruco_assets.get_dictionary(cfg.aruco.dictionary),
        cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(
        cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
    found = [] if ids is None else [int(i) for i in ids.flatten()]
    return found, corners


def _marker_side_px(quad) -> float:
    """Longest edge of the detected marker quadrilateral, pixels."""
    pts = quad[0]
    return max(float(math.dist(pts[i], pts[(i + 1) % 4])) for i in range(4))


# --------------------------------------------------------------------------- #
# Patrol motion
# --------------------------------------------------------------------------- #

def test_rovers_advance_and_stay_in_bounds(cfg_move):
    reg = get_registry(cfg_move)
    try:
        n_lo, n_hi = cfg_move.rovers.patrol.bounds_north
        e_lo, e_hi = cfg_move.rovers.patrol.bounds_east
        tracks = [[] for _ in range(cfg_move.rovers.count)]
        for _ in range(15):              # ~15 sim seconds at rtf 10
            for track, pos in zip(tracks, reg.rover_arena_positions()):
                track.append(pos)
            time.sleep(0.1)
        for i, track in enumerate(tracks):
            path_len = sum(math.dist(a, b) for a, b in zip(track, track[1:]))
            assert path_len > 0.5, f"rover {i} barely moved ({path_len:.2f} m)"
            for n, e in track:           # never out of bounds / through walls
                assert n_lo - 0.05 <= n <= n_hi + 0.05
                assert e_lo - 0.05 <= e <= e_hi + 0.05
    finally:
        shutdown_registry()


def test_parked_rovers_hold_position(cfg_scan):
    reg = get_registry(cfg_scan)
    try:
        before = reg.rover_arena_positions()
        time.sleep(0.3)
        after = reg.rover_arena_positions()
        assert before == after           # speed 0 => parked
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Unique markers
# --------------------------------------------------------------------------- #

def test_marker_ids_distinct_and_match_config(cfg_move):
    reg = get_registry(cfg_move)
    try:
        ids = [r.marker_id for r in reg.rovers]
        assert ids == list(cfg_move.rovers.marker_ids)[:cfg_move.rovers.count]
        assert len(set(ids)) == len(ids) == 5
        assert not set(ids) & {p.id for p in cfg_move.pads}
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Scan geometry: marker >= min_marker_px AND decodes at a sane pose
# --------------------------------------------------------------------------- #

def test_drone_scans_rover_marker_above_threshold(cfg_scan):
    reg = get_registry(cfg_scan)
    stream = None
    try:
        rover = reg.rovers[0]
        rn, re_ = reg.rover_arena_positions()[0]

        d = DroneAPI()
        d.connect(cfg_scan.drones.units[0].ip)
        d.takeoff(150)
        # Drone 0 takes off at arena (0.6, 1.5) heading north => takeoff
        # frame: x(right)=east, y(forward)=north. Park the CAMERA (mounted
        # mount_offset_m ahead of centre) directly over the rover marker.
        cam_off_cm = cfg_scan.camera.mount_offset_m * 100.0
        d.move_to((re_ - 1.5) * 100.0, (rn - 0.6) * 100.0 - cam_off_cm, 150)
        d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 90)
        stream = d.create_video_stream()
        d.set_video_stream(True)
        stream.start()

        deadline = time.time() + 5.0
        while time.time() < deadline:
            frame = stream.latest_frame
            if frame is not None:
                ids, corners = _detect(cfg_scan, frame.to_rgb())
                if rover.marker_id in ids:
                    break
            time.sleep(0.02)
        else:
            pytest.fail(f"rover marker {rover.marker_id} never decoded from "
                        f"the scan pose")

        side_px = _marker_side_px(corners[ids.index(rover.marker_id)])
        assert side_px >= cfg_scan.scoring.min_marker_px, (
            f"marker projects {side_px:.0f}px < scoring gate "
            f"{cfg_scan.scoring.min_marker_px}px — scan not valid at 1.5 m")
        # And it is THIS rover's unique id, not any other configured marker.
        other_ids = set(cfg_scan.rovers.marker_ids) - {rover.marker_id}
        assert rover.marker_id in ids
        assert not other_ids & set(ids)
    finally:
        if stream is not None:
            stream.stop()
        shutdown_registry()


def test_marker_faces_up_not_visible_edge_on(cfg_scan):
    """Top markers face UP: viewed edge-on (camera at the marker's own plane
    height, looking forward) they must not decode — only a camera looking
    down onto the face reads them (asserted in the scan test above)."""
    reg = get_registry(cfg_scan)
    stream = None
    try:
        rover = reg.rovers[0]
        rn, re_ = reg.rover_arena_positions()[0]
        d = DroneAPI()
        d.connect(cfg_scan.drones.units[0].ip)
        d.takeoff(100)
        # Stand off 1.2 m south of the rover with the camera AT the marker
        # plane (rover top ~0.27 m), looking forward (north): true edge-on.
        d.move_to((re_ - 1.5) * 100.0, (rn - 0.6) * 100.0 - 120.0, 27)
        d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 0)
        stream = d.create_video_stream()
        d.set_video_stream(True)
        stream.start()
        deadline = time.time() + 5.0
        while time.time() < deadline:
            if stream.latest_frame is not None:
                break
            time.sleep(0.02)
        time.sleep(0.2)  # a couple of fresh frames at the final pose
        ids, _ = _detect(cfg_scan, stream.latest_frame.to_rgb())
        assert rover.marker_id not in ids  # top face is edge-on: unreadable
    finally:
        if stream is not None:
            stream.stop()
        shutdown_registry()
