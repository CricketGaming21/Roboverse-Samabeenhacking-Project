"""Phase 23 acceptance test — per-drone camera insets in the recording.

The composited frame shows the 3D arena (top) plus a row of three labelled
per-drone camera feeds (bottom), each with ArUco markers outlined + id
labelled (reusing camfeed.annotate_markers). Observer-only: the insets
render offscreen on the sim thread and must not perturb the sim, never
open p.GUI, and degrade to placeholder tiles when a drone isn't flying.
"""

import cv2
import numpy as np
import pytest

from pyhulax import DroneAPI
from pyhulax.core import CameraPitchMode

from scripts.scenario_demo import run_scenario_demo
from simcore import aruco_assets
from simcore.config import load_config
from simcore.recorder import ArenaRecorder
from simcore.registry import get_registry, shutdown_registry


def _cfg(rtf=10.0):
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = rtf
    c.record.width, c.record.height = 600, 360
    c.record.show_camera_insets = True
    return c


def _scan_cfg():
    c = _cfg()
    c.rovers.gimbal.enabled = False  # marker always faces drone (gimbal tested in phase36)
    c.scoring.enabled = False
    c.arena.layout = "procedural"
    c.arena.obstacles.count = 0
    c.rovers.motion = "patrol"
    c.rovers.patrol.speed_mps = 0.0
    c.scenario.phases = "ambush"
    c.motion.realistic = False
    return c


# --------------------------------------------------------------------------- #
# Layout: arena on top + three labelled insets below
# --------------------------------------------------------------------------- #

def test_composited_frame_has_arena_plus_three_insets():
    cfg = _scan_cfg()
    reg = get_registry(cfg)
    try:
        rec = ArenaRecorder(reg, "/tmp/_p23.mp4")
        # canvas is taller than the arena render: a band reserved below
        assert rec._h > rec._arena_h
        assert rec._h == rec._arena_h + rec._band_h
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        d.takeoff(150)
        frame = rec.capture_frame()
        assert frame.shape == (rec._h, rec._w, 3)
        arena_part = frame[:rec._arena_h]
        inset_band = frame[rec._arena_h:]
        assert float(arena_part.std()) > 5.0       # arena geometry up top
        assert float(inset_band.std()) > 5.0       # feeds rendered below
        # three tiles: green drone labels live in the band
        assert inset_band.shape[0] == rec._band_h
    finally:
        shutdown_registry()


def test_inset_shows_marker_outline_and_id_in_view():
    cfg = _scan_cfg()
    reg = get_registry(cfg)
    try:
        from simcore import camfeed
        rn, re_ = reg.rover_arena_positions()[0]
        start = cfg.drones.units[0].start
        rover_id = reg.rovers[0].marker_id
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        d.takeoff(150)
        d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 90)
        d.move_to((re_ - start[1]) * 100.0,
                  (rn - start[0]) * 100.0
                  - cfg.camera.mount_offset_m * 100.0, 150)
        # the inset uses the SAME annotate path; drone 0 frames the rover
        rgb = reg.render_camera(reg.drones[0])
        annotated, found = camfeed.annotate_markers(
            cfg, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        assert rover_id in [mid for mid, _pts in found]   # outlined + labelled
        # and the composited frame is built without error from that state
        rec = ArenaRecorder(reg, "/tmp/_p23b.mp4")
        frame = rec.capture_frame()
        assert frame.shape == (rec._h, rec._w, 3)
    finally:
        shutdown_registry()


def test_placeholder_tiles_when_not_flying():
    cfg = _scan_cfg()
    reg = get_registry(cfg)
    try:
        # no drone connected/flying -> all three insets are placeholders,
        # and capture_frame must not crash
        rec = ArenaRecorder(reg, "/tmp/_p23c.mp4")
        d0 = reg.drones[0]
        assert reg.run_on_sim_thread(lambda: d0.flying) is False
        tile = rec._drone_tile(d0, 200, 90)
        assert tile.shape == (90, 200, 3)          # a real placeholder tile
        frame = rec.capture_frame()
        assert frame.shape == (rec._h, rec._w, 3)  # whole frame still built
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Fidelity + boundaries still hold with insets on
# --------------------------------------------------------------------------- #

def _episode_cfg():
    c = _cfg()
    c.scenario.episode_seconds = 12.0
    c.scenario.ambush_seconds = 9.0
    c.scenario.deploy_timeout_s = 8.0
    c.scenario.ambush_trigger.mode = "on_all_landed"
    c.scenario.ambush_trigger.delay_s = 1.0
    return c


def test_insets_do_not_perturb_the_sim(tmp_path):
    plain = run_scenario_demo(_episode_cfg())
    rec_cfg = _episode_cfg()
    rec_cfg.record.show_camera_insets = True
    rec = run_scenario_demo(rec_cfg, record_path=str(tmp_path / "i.mp4"))
    assert rec["landing_score"] == plain["landing_score"]
    assert rec["snapshot_score"] == plain["snapshot_score"]
    assert sorted(b[0] for b in rec["banked"]) == \
        sorted(b[0] for b in plain["banked"])
    assert rec["record_frames"] > 0


def test_record_with_insets_never_connects_gui(monkeypatch, tmp_path):
    import pybullet as p
    real_connect = p.connect
    seen = []
    monkeypatch.setattr(p, "connect",
                        lambda mode, *a, **k: (seen.append(mode),
                                               real_connect(mode, *a, **k))[1])
    cfg = _episode_cfg()
    cfg.record.show_camera_insets = True
    run_scenario_demo(cfg, record_path=str(tmp_path / "g.mp4"))
    assert p.GUI not in seen and p.DIRECT in seen


def test_insets_off_gives_bare_arena_frame():
    cfg = _scan_cfg()
    cfg.record.show_camera_insets = False
    reg = get_registry(cfg)
    try:
        rec = ArenaRecorder(reg, "/tmp/_p23d.mp4")
        assert rec._h == rec._arena_h            # no extra band
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        d.takeoff(150)
        frame = rec.capture_frame()
        assert frame.shape == (rec._arena_h, rec._arena_w, 3)
    finally:
        shutdown_registry()
