"""Phase 26 acceptance test — MP4 target-acquisition UI + polish (headless).

A YELLOW 'DETECTED id N' box appears the moment cv2.aruco decodes a marker
in a drone's frame, turning GREEN 'ACQUIRED id N' once the referee banks it,
with a brief flash on the first bank. Real detection (not ground-truth
positions); legends/status render; fidelity holds; never p.GUI.
"""

import math

import cv2
import numpy as np
import pybullet as p
import pytest

from pyhulax import DroneAPI
from pyhulax.core import CameraPitchMode

from scripts.scenario_demo import run_scenario_demo
from simcore import camfeed
from simcore.config import load_config
from simcore.recorder import ArenaRecorder
from simcore.registry import get_registry, shutdown_registry


def _scan_cfg():
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0
    c.scoring.enabled = False
    c.arena.layout = "procedural"
    c.arena.obstacles.count = 0
    c.rovers.motion = "patrol"
    c.rovers.patrol.speed_mps = 0.0
    c.scenario.phases = "ambush"
    c.motion.realistic = False
    return c


def _yellow(img):  # DETECTED = (0,255,255) BGR
    b, g, r = img[:, :, 0], img[:, :, 1], img[:, :, 2]
    return int(np.sum((r > 180) & (g > 180) & (b < 110)))


def _green_box(img):  # ACQUIRED green ~ (60,220,80) BGR
    b, g, r = img[:, :, 0], img[:, :, 1], img[:, :, 2]
    return int(np.sum((g > 170) & (r < 130) & (b < 130)))


def _scan_pose(reg, cfg, d, index=0):
    rn, re_ = reg.rover_arena_positions()[index]
    start = cfg.drones.units[0].start
    d.takeoff(150)
    d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 90)
    d.move_to((re_ - start[1]) * 100.0,
              (rn - start[0]) * 100.0 - cfg.camera.mount_offset_m * 100.0, 150)
    return reg.rovers[index].marker_id


# --------------------------------------------------------------------------- #
# Detected (yellow) vs acquired (green), from REAL detection
# --------------------------------------------------------------------------- #

def test_detected_box_yellow_then_acquired_green():
    cfg = _scan_cfg()
    reg = get_registry(cfg)
    try:
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        rover_id = _scan_pose(reg, cfg, d)
        bgr = cv2.cvtColor(reg.render_camera(reg.drones[0]),
                           cv2.COLOR_RGB2BGR)

        # NOT yet banked -> DETECTED (yellow)
        det, found = camfeed.acquisition_overlay(cfg, bgr, banked_ids=set())
        assert rover_id in [m for m, _p in found]
        assert _yellow(det) > 20 and _green_box(det) < _yellow(det)

        # banked -> ACQUIRED (green)
        acq, _f = camfeed.acquisition_overlay(cfg, bgr, banked_ids={rover_id})
        assert _green_box(acq) > 20
        assert _green_box(acq) > _green_box(det)   # more green once acquired
    finally:
        shutdown_registry()


def test_detection_is_real_cv2_aruco_not_ground_truth():
    cfg = _scan_cfg()
    reg = get_registry(cfg)
    try:
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        rover_id = _scan_pose(reg, cfg, d)
        bgr = cv2.cvtColor(reg.render_camera(reg.drones[0]),
                           cv2.COLOR_RGB2BGR)
        # the overlay's found == the shared real detector's output
        _o, found = camfeed.acquisition_overlay(cfg, bgr, banked_ids=set())
        real = camfeed.detect_markers(cfg, bgr)
        assert sorted(m for m, _ in found) == sorted(m for m, _ in real)
        assert rover_id in [m for m, _ in real]

        # a marker NOT in view (camera forward, rover behind) is NOT boxed —
        # ground-truth drawing would show it; real detection does not.
        d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 0)
        import time
        time.sleep(0.05)
        bgr2 = cv2.cvtColor(reg.render_camera(reg.drones[0]),
                            cv2.COLOR_RGB2BGR)
        _o2, found2 = camfeed.acquisition_overlay(cfg, bgr2,
                                                  banked_ids={rover_id})
        assert rover_id not in [m for m, _ in found2]   # not detected => no box
    finally:
        shutdown_registry()


def test_first_bank_flash_then_fades():
    cfg = _scan_cfg()
    cfg.scoring.enabled = True
    reg = get_registry(cfg)
    try:
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        rover_id = _scan_pose(reg, cfg, d)
        rec = ArenaRecorder(reg, "/tmp/_p26_flash.mp4")
        # before any bank: no flash
        banked, flash = rec._acquisition_state()
        assert rover_id not in flash
        # force the bank, then the first _acquisition_state flags it flashing
        import time
        deadline = time.time() + 8.0
        while time.time() < deadline and rover_id not in reg.referee.banked_ids():
            time.sleep(0.02)
        assert rover_id in reg.referee.banked_ids()
        _b, flash = rec._acquisition_state()
        assert rover_id in flash                       # flashes on first bank
        assert rover_id in rec._banked_seen
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Polish: legend + per-drone status line render
# --------------------------------------------------------------------------- #

def test_legend_and_status_render():
    cfg = _scan_cfg()
    reg = get_registry(cfg)
    try:
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        d.takeoff(150)
        rec = ArenaRecorder(reg, "/tmp/_p26_leg.mp4")
        # status line from a probe snapshot
        from simcore.debug import DebugProbe
        snap = DebugProbe(reg).snapshot(referee_view=False)
        status = rec._status_line(snap["drones"][0])
        assert status.startswith("d0 ")
        assert ("BLOCKING" in status or "MANUAL" in status
                or status.endswith("idle"))
        # the overlay banner draws a legend (KEY) — banner taller than before
        arena = np.full((cfg.record.height, cfg.record.width, 3), 90,
                        dtype=np.uint8)
        rec._overlay(arena)
        # legend swatches add saturated colour into the banner region
        banner = arena[:110]
        assert float(banner.std()) > 0
        # a manual-mode status line shows the sticks
        d.send_manual_control(forward=0.5, right=-0.2)
        snap2 = DebugProbe(reg).snapshot(referee_view=False)
        st2 = rec._status_line(snap2["drones"][0])
        assert "MANUAL" in st2 and "F+0.5" in st2
        d.send_manual_control()
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Fidelity + GUI guard with the acquisition UI on
# --------------------------------------------------------------------------- #

def _episode_cfg():
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0
    c.record.width, c.record.height = 480, 320
    c.scenario.episode_seconds = 45.0
    c.scenario.ambush_seconds = 8.0
    c.scenario.deploy_timeout_s = 40.0
    c.scenario.ambush_trigger.mode = "on_all_landed"
    c.scenario.ambush_trigger.delay_s = 1.0
    return c


def test_acquisition_ui_does_not_perturb_the_sim(tmp_path):
    plain = run_scenario_demo(_episode_cfg())
    rec = run_scenario_demo(_episode_cfg(), record_path=str(tmp_path / "a.mp4"))
    assert rec["landing_score"] == plain["landing_score"]
    assert rec["snapshot_score"] == plain["snapshot_score"]
    assert sorted(b[0] for b in rec["banked"]) == \
        sorted(b[0] for b in plain["banked"])
    assert rec["record_frames"] > 0


def test_record_never_connects_gui(monkeypatch, tmp_path):
    real_connect = p.connect
    seen = []
    monkeypatch.setattr(p, "connect",
                        lambda mode, *a, **k: (seen.append(mode),
                                               real_connect(mode, *a, **k))[1])
    run_scenario_demo(_episode_cfg(), record_path=str(tmp_path / "g.mp4"))
    assert p.GUI not in seen and p.DIRECT in seen
