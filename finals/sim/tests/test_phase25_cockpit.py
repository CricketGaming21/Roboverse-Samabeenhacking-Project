"""Phase 25 acceptance test — MP4 cockpit: per-drone telemetry + car-style
proximity graphic + translucent walls (headless).

All observer-side (DebugProbe), never pyhulax; the scoreboard-equality
fidelity check still holds; the record path never connects p.GUI; wall
TRANSPARENCY is visual-only (collision/geometry unchanged).
"""

import math

import cv2
import numpy as np
import pybullet as p
import pytest

from pyhulax import DroneAPI

from scripts.scenario_demo import run_scenario_demo
from simcore import frames
from simcore.config import load_config
from simcore.debug import DebugProbe
from simcore.recorder import ArenaRecorder
from simcore.registry import get_registry, shutdown_registry


def _cfg():
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0
    c.scoring.enabled = False
    c.arena.layout = "procedural"
    c.arena.obstacles.count = 0
    c.rovers.count = 0
    c.motion.realistic = False
    c.record.width, c.record.height = 900, 540
    return c


def _connect(cfg, i=0):
    d = DroneAPI()
    d.connect(cfg.drones.units[i].ip)
    return d


# --------------------------------------------------------------------------- #
# Telemetry panel — fields present, values match the probe snapshot
# --------------------------------------------------------------------------- #

def test_telemetry_lines_match_probe_snapshot():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = _connect(cfg)
        d.takeoff(150)
        rec = ArenaRecorder(reg, "/tmp/_p25_a.mp4")
        snap = DebugProbe(reg).snapshot(referee_view=False)
        d0 = snap["drones"][0]
        lines = [t for t, _c in rec._telemetry_lines(d0)]
        blob = " ".join(lines)
        # every listed field is present
        assert "FLY" in blob and "batt" in blob          # state + battery
        assert "UWB" in blob and "EST" in blob            # UWB pos AND estimate
        assert "drift" in blob
        assert "spd" in blob and "hdg" in blob            # speed + heading
        assert "cam" in blob                              # camera pitch
        assert "ypr" in blob                              # yaw/pitch/roll
        assert "CMD" in blob or "STK" in blob             # command or sticks
        assert "UWB: OK" in blob or "UWB: NO FIX" in blob  # uwb indicator
        # values are the PROBE's, not invented
        n, e = d0["true"]["arena_ne_m"][:2]
        assert f"{n:5.2f},{e:5.2f}" in blob
        assert f"batt {d0['battery_pct']:.0f}%" in blob
        assert f"spd {d0['speed_mps']:.2f}" in blob
    finally:
        shutdown_registry()


def test_manual_sticks_shown_in_telemetry():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = _connect(cfg)
        d.takeoff(150)
        d.send_manual_control(forward=0.5, right=-0.2, rotate=0.3)
        rec = ArenaRecorder(reg, "/tmp/_p25_b.mp4")
        d0 = DebugProbe(reg).snapshot(referee_view=False)["drones"][0]
        blob = " ".join(t for t, _c in rec._telemetry_lines(d0))
        assert "STK" in blob and "F+0.5" in blob          # live stick inputs
        d.send_manual_control()
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Proximity graphic — boolean directional, lights red on a set flag
# --------------------------------------------------------------------------- #

def test_proximity_segment_lights_red_when_flag_set():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        rec = ArenaRecorder(reg, "/tmp/_p25_c.mp4")
        rays_clear = {n: {"blocked": False} for n in
                      ("forward", "back", "left", "right", "down")}
        rays_fwd = dict(rays_clear, forward={"blocked": True})
        h, w = 120, 120

        def render(rays):
            panel = np.full((h, w, 3), 22, dtype=np.uint8)
            rec._draw_proximity(panel, 0, 0, w, h, {"sensors": {"rays": rays}})
            return panel
        clear = render(rays_clear)
        lit = render(rays_fwd)
        # RED = (60,60,235) BGR. "lit" has strong-red pixels; "clear" does not.
        def red_count(img):
            b, g, r = img[:, :, 0], img[:, :, 1], img[:, :, 2]
            return int(np.sum((r > 180) & (g < 110) & (b < 110)))
        assert red_count(lit) > 20
        assert red_count(clear) == 0
        # the lit red is in the FORWARD (upper) half of the icon
        upper = lit[: h // 2]
        lower = lit[h // 2:]
        assert red_count(upper) > red_count(lower)
    finally:
        shutdown_registry()


def test_proximity_reflects_real_get_obstacles():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        def _box():
            pos = frames.arena_to_world(cfg, 2.8, 1.1, 1.0)
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.6, 0.2, 1.0],
                                         physicsClientId=reg.client)
            return p.createMultiBody(0, col, basePosition=pos,
                                     physicsClientId=reg.client)
        reg.run_on_sim_thread(_box)
        d = _connect(cfg)
        d.takeoff(100)
        d.move_to(0, 150, 100)                  # nose at the wall: fwd flag set
        snap = DebugProbe(reg).snapshot(referee_view=False)
        fwd = snap["drones"][0]["sensors"]["rays"]["forward"]["blocked"]
        assert fwd is True
        assert d.get_obstacles().forward is True   # mirrors the public boolean
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Translucent walls — visual alpha < 1, collision/geometry unchanged
# --------------------------------------------------------------------------- #

def test_walls_translucent_but_solid_to_physics_and_sensors():
    cfg = _cfg()
    cfg.arena.obstacles.count = 0
    reg = get_registry(cfg)
    try:
        wall = reg.bodies.walls[0]
        vis = reg.run_on_sim_thread(
            lambda: p.getVisualShapeData(wall, physicsClientId=reg.client))
        alpha = vis[0][7][3]
        assert alpha == pytest.approx(cfg.arena.wall_alpha)
        assert 0.0 < alpha < 1.0                       # semi-transparent render
        # collision shape is UNCHANGED (still solid to physics + ray sensors)
        col = reg.run_on_sim_thread(
            lambda: p.getCollisionShapeData(wall, -1,
                                            physicsClientId=reg.client))
        assert len(col) >= 1
        # the barrier sensor still detects the (translucent) wall: fly drone 0
        # north toward the far wall and confirm the forward flag trips
        d = _connect(cfg)
        d.takeoff(100)
        d.move_to(0, 900, 100)                         # n~9.6: wall face 0.3 m
        assert d.get_obstacles().forward is True       # ahead, inside range
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# UWB-OK indicator
# --------------------------------------------------------------------------- #

def test_uwb_ok_indicator_green_then_red():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = _connect(cfg)
        d.takeoff(100)
        ok = DebugProbe(reg).snapshot(referee_view=False)["drones"][0]
        assert ok["uwb_ok"] is True                    # mapped tag, no dropout
    finally:
        shutdown_registry()
    cfg2 = _cfg()
    cfg2.uwb.dropout_prob = 1.0                         # UWB fully dropping out
    reg = get_registry(cfg2)
    try:
        d = _connect(cfg2)
        d.takeoff(100)
        rec = ArenaRecorder(reg, "/tmp/_p25_d.mp4")
        d0 = DebugProbe(reg).snapshot(referee_view=False)["drones"][0]
        assert d0["uwb_ok"] is False                    # no fix
        blob = " ".join(t for t, _c in rec._telemetry_lines(d0))
        assert "NO FIX" in blob
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Layout + fidelity + GUI guard
# --------------------------------------------------------------------------- #

def test_cockpit_layout_has_camera_and_panel_rows():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = _connect(cfg)
        d.takeoff(150)
        rec = ArenaRecorder(reg, "/tmp/_p25_e.mp4")
        assert rec._band_h == rec._cam_h + rec._panel_h
        assert rec._cam_h > 0 and rec._panel_h > 0     # both rows present
        frame = rec.capture_frame()
        assert frame.shape == (rec._arena_h + rec._band_h, rec._w, 3)
        arena = frame[:rec._arena_h]
        panels = frame[rec._arena_h + rec._cam_h:]
        assert float(arena.std()) > 5.0                # arena up top
        assert float(panels.std()) > 5.0              # telemetry/prox rendered
    finally:
        shutdown_registry()


def _episode_cfg():
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0
    c.record.width, c.record.height = 480, 320
    # Generous deploy window so all three (slow MEDIUM) drones land before
    # any timeout — landings are then deterministic; a short ambush keeps the
    # rest deterministic too (the slow convoy reaches no station in time).
    c.scenario.episode_seconds = 45.0
    c.scenario.ambush_seconds = 8.0
    c.scenario.deploy_timeout_s = 40.0
    c.scenario.ambush_trigger.mode = "on_all_landed"
    c.scenario.ambush_trigger.delay_s = 1.0
    return c


def test_cockpit_does_not_perturb_the_sim(tmp_path):
    plain = run_scenario_demo(_episode_cfg())
    rec = run_scenario_demo(_episode_cfg(), record_path=str(tmp_path / "c.mp4"))
    assert rec["landing_score"] == plain["landing_score"]
    assert rec["snapshot_score"] == plain["snapshot_score"]
    assert sorted(b[0] for b in rec["banked"]) == \
        sorted(b[0] for b in plain["banked"])
    assert rec["record_frames"] > 0


def test_cockpit_record_never_connects_gui(monkeypatch, tmp_path):
    real_connect = p.connect
    seen = []
    monkeypatch.setattr(p, "connect",
                        lambda mode, *a, **k: (seen.append(mode),
                                               real_connect(mode, *a, **k))[1])
    run_scenario_demo(_episode_cfg(), record_path=str(tmp_path / "g.mp4"))
    assert p.GUI not in seen and p.DIRECT in seen
