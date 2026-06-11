"""Regression test — the camera render path forces TinyRenderer under p.GUI.

A hardware getCameraImage races the GUI render thread and HANGS the sim
thread on WSLg (the scenario_demo --gui part-2 freeze). resolve_renderer
must pick ER_TINY_RENDERER whenever the client is a p.GUI connection,
regardless of the renderer a caller hands in — and leave the headless
DIRECT+EGL path untouched. (GUI itself is not opened here: no display in
CI; the guard is verified by connection mode without a window.)
"""

import cv2
import pytest

from pyhulax import DroneAPI
from pyhulax.core import CameraPitchMode

from simcore import aruco_assets, camera
from simcore.config import load_config
from simcore.registry import get_registry, shutdown_registry


def _cfg():
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0
    c.scoring.enabled = False
    c.arena.layout = "procedural"
    c.arena.obstacles.count = 0
    c.rovers.count = 0
    c.rovers.gimbal.enabled = False  # marker always faces drone (gimbal tested in phase36)
    return c


# --------------------------------------------------------------------------- #
# resolve_renderer: keyed on connection mode, not the requested value
# --------------------------------------------------------------------------- #

def test_resolve_renderer_forces_tiny_under_gui(monkeypatch):
    import pybullet as p

    # GUI connection: ANY requested renderer is forced to software Tiny.
    monkeypatch.setattr(p, "getConnectionInfo",
                        lambda client: {"connectionMethod": p.GUI})
    assert camera.resolve_renderer(0, p.ER_BULLET_HARDWARE_OPENGL) \
        == p.ER_TINY_RENDERER
    assert camera.resolve_renderer(0, p.ER_TINY_RENDERER) \
        == p.ER_TINY_RENDERER

    # DIRECT (headless): the requested renderer passes through unchanged —
    # the EGL hardware path is not altered.
    monkeypatch.setattr(p, "getConnectionInfo",
                        lambda client: {"connectionMethod": p.DIRECT})
    assert camera.resolve_renderer(0, p.ER_BULLET_HARDWARE_OPENGL) \
        == p.ER_BULLET_HARDWARE_OPENGL
    assert camera.resolve_renderer(0, p.ER_TINY_RENDERER) \
        == p.ER_TINY_RENDERER


def test_render_path_uses_tiny_when_client_is_gui(monkeypatch):
    """Even if the registry handed a HARDWARE renderer, render_rgb must issue
    the getCameraImage with TinyRenderer once the client reports p.GUI — so a
    wrong self.renderer can never reach the hangs-on-WSLg hardware path."""
    import pybullet as p
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        d.takeoff(150)
        drone = reg.drones[0]

        captured = {}
        real = p.getCameraImage

        def _spy(*args, **kwargs):
            captured["renderer"] = kwargs.get("renderer")
            return real(*args, **kwargs)

        # Pretend the live connection is GUI and force a hardware request.
        monkeypatch.setattr(p, "getConnectionInfo",
                            lambda client: {"connectionMethod": p.GUI})
        monkeypatch.setattr(p, "getCameraImage", _spy)
        reg.run_on_sim_thread(
            lambda: camera.render_rgb(reg.client, cfg, drone,
                                      p.ER_BULLET_HARDWARE_OPENGL))
        assert captured["renderer"] == p.ER_TINY_RENDERER  # forced software
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Headless EGL path unchanged: real render still decodes a marker
# --------------------------------------------------------------------------- #

def test_headless_egl_render_still_decodes_marker():
    cfg = _cfg()
    cfg.rovers.count = 5
    cfg.rovers.motion = "patrol"
    cfg.rovers.patrol.speed_mps = 0.0
    cfg.scenario.phases = "ambush"
    cfg.motion.realistic = False
    reg = get_registry(cfg)
    try:
        import pybullet as p
        # headless connection really is DIRECT (the EGL hardware path)
        mode = reg.run_on_sim_thread(
            lambda: p.getConnectionInfo(reg.client)["connectionMethod"])
        assert mode == p.DIRECT
        rn, re_ = reg.rover_arena_positions()[0]
        start = cfg.drones.units[0].start
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        d.takeoff(150)
        d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 90)
        d.move_to((re_ - start[1]) * 100.0,
                  (rn - start[0]) * 100.0
                  - cfg.camera.mount_offset_m * 100.0, 150)
        rgb = reg.render_camera(reg.drones[0])
        assert rgb.shape == (cfg.camera.height, cfg.camera.width, 3)
        detector = cv2.aruco.ArucoDetector(
            aruco_assets.get_dictionary(cfg.aruco.dictionary),
            cv2.aruco.DetectorParameters())
        _c, ids, _ = detector.detectMarkers(
            cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
        assert ids is not None and reg.rovers[0].marker_id in ids.flatten()
    finally:
        shutdown_registry()
