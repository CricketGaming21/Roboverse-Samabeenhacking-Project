"""Phase 21 acceptance test — ArUco-outlined feeds, on-bank scan logging,
RoboMaster-style rover visuals (headless).

All observer-side: the sim's own cv2.aruco for the overlay/scan images, the
referee's bank event for the saves, and richer rover geometry so a
mission-side YOLO could plausibly detect it. Nothing on the pyhulax surface.
"""

import time

import cv2
import numpy as np
import pybullet as p
import pytest

from pyhulax import DroneAPI
from pyhulax.core import CameraPitchMode

from simcore import aruco_assets, camfeed
from simcore.config import load_config
from simcore.registry import get_registry, shutdown_registry


def _cfg(**over):
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0
    c.rovers.gimbal.enabled = False  # marker always faces drone (gimbal tested in phase36)
    c.arena.layout = "procedural"
    c.arena.obstacles.count = 0
    c.rovers.motion = "patrol"
    c.rovers.patrol.speed_mps = 0.0       # parked targets, deterministic
    c.scenario.phases = "ambush"          # part-2 referee judges in AMBUSH
    c.motion.realistic = False
    for k, v in over.items():
        setattr(c.logging, k, v)
    return c


def _park_camera_over_rover(reg, cfg, d, index=0):
    rn, re_ = reg.rover_arena_positions()[index]
    start = cfg.drones.units[0].start
    d.takeoff(150)
    d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 90)
    d.move_to((re_ - start[1]) * 100.0,
              (rn - start[0]) * 100.0 - cfg.camera.mount_offset_m * 100.0, 150)
    return reg.rovers[index].marker_id


# --------------------------------------------------------------------------- #
# 1. Camera feed overlay: markers outlined + id labelled
# --------------------------------------------------------------------------- #

def test_annotate_markers_outlines_and_labels():
    cfg = _cfg()
    cfg.scoring.enabled = False
    reg = get_registry(cfg)
    try:
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        rover_id = _park_camera_over_rover(reg, cfg, d)
        rgb = reg.render_camera(reg.drones[0])
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        annotated, found = camfeed.annotate_markers(cfg, bgr)
        ids = [mid for mid, _pts in found]
        assert rover_id in ids                      # the marker was detected
        assert annotated.shape == bgr.shape
        # the overlay actually changed pixels (outline + id text drawn)
        assert not np.array_equal(annotated, bgr)

        # ...and with NO marker in view, the frame is returned unchanged
        d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 0)  # look forward
        time.sleep(0.05)
        rgb2 = reg.render_camera(reg.drones[0])
        bgr2 = cv2.cvtColor(rgb2, cv2.COLOR_RGB2BGR)
        annotated2, found2 = camfeed.annotate_markers(cfg, bgr2)
        assert found2 == []
        assert np.array_equal(annotated2, bgr2)
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# 2. On-bank scan logging — once per banking event, not per frame
# --------------------------------------------------------------------------- #

def test_scan_image_saved_once_on_bank(tmp_path):
    cfg = _cfg(scans_dir=str(tmp_path / "scans"), save_scans=True)
    reg = get_registry(cfg)
    try:
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        rover_id = _park_camera_over_rover(reg, cfg, d)
        deadline = time.time() + 8.0
        while time.time() < deadline:
            if rover_id in reg.referee.banked_ids():
                break
            time.sleep(0.02)
        assert rover_id in reg.referee.banked_ids(), "marker never banked"
        time.sleep(0.6)   # keep staring well past the bank (would-be spam)

        files = sorted((tmp_path / "scans").glob("*.png"))
        mine = [f for f in files if f"id{rover_id}_" in f.name]
        assert len(mine) == 1, f"expected exactly one scan image, got {mine}"
        img = cv2.imread(str(mine[0]))
        assert img is not None and img.shape[2] == 3   # a real boxed frame
        assert "drone0" in mine[0].name                # attribution in name
    finally:
        shutdown_registry()


def test_scan_logging_can_be_disabled(tmp_path):
    cfg = _cfg(scans_dir=str(tmp_path / "scans"), save_scans=False)
    reg = get_registry(cfg)
    try:
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        rover_id = _park_camera_over_rover(reg, cfg, d)
        deadline = time.time() + 8.0
        while time.time() < deadline:
            if rover_id in reg.referee.banked_ids():
                break
            time.sleep(0.02)
        assert rover_id in reg.referee.banked_ids()
        time.sleep(0.3)
        assert not (tmp_path / "scans").exists() or not list(
            (tmp_path / "scans").glob("*.png"))        # nothing written
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# 3. Rover visual fidelity — non-trivial geometry, not a flat plane
# --------------------------------------------------------------------------- #

def test_rover_render_is_non_trivial_geometry():
    cfg = _cfg()
    cfg.scoring.enabled = False
    reg = get_registry(cfg)
    try:
        rover_id = reg.bodies.rovers[0]
        shapes = reg.run_on_sim_thread(
            lambda: p.getVisualShapeData(rover_id, physicsClientId=reg.client))
        # chassis + 4 wheels + turret + barrel + marker = many visual shapes
        assert len(shapes) >= 6, f"rover has only {len(shapes)} visual shapes"
        geom_types = {s[2] for s in shapes}
        assert p.GEOM_CYLINDER in geom_types          # wheels/barrel: not flat
        # real 3D extent (a billboard plane would be flat in z)
        lo, hi = reg.run_on_sim_thread(
            lambda: p.getAABB(rover_id, physicsClientId=reg.client))
        dims = [hi[k] - lo[k] for k in range(3)]
        assert dims[2] > 0.1                           # genuine height
        assert dims[0] > 0.1 and dims[1] > 0.1
    finally:
        shutdown_registry()


def test_rover_marker_still_decodes_on_new_visual():
    """The richer body must not break the top marker's detectability."""
    cfg = _cfg()
    cfg.scoring.enabled = False
    reg = get_registry(cfg)
    try:
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        rover_id = _park_camera_over_rover(reg, cfg, d)
        rgb = reg.render_camera(reg.drones[0])
        detector = cv2.aruco.ArucoDetector(
            aruco_assets.get_dictionary(cfg.aruco.dictionary),
            cv2.aruco.DetectorParameters())
        _c, ids, _ = detector.detectMarkers(cv2.cvtColor(rgb,
                                                         cv2.COLOR_RGB2GRAY))
        assert ids is not None and rover_id in ids.flatten()
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Headless safety
# --------------------------------------------------------------------------- #

def test_headless_safe_camfeed_no_pyhulax():
    import inspect
    src = inspect.getsource(camfeed)
    imports = [ln for ln in src.splitlines()
               if ln.lstrip().startswith(("import ", "from "))]
    assert not any("pyhulax" in ln for ln in imports)  # observer-side only
