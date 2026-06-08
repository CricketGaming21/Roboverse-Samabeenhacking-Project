"""Phase 13 acceptance test — camera-FOV footprint overlay (headless).

The footprint is the camera frustum projected onto the ground, built from
the SAME pose+pitch geometry as the render (frames.camera_eye_target_up).
PITCH moves/tilts it; its angular size is FIXED by the lens — no zoom
exists anywhere.
"""

import math
import time

import pytest

from pyhulax import DroneAPI
from pyhulax.core import CameraPitchMode

from simcore import frames
from simcore.config import load_config
from simcore.registry import get_registry, shutdown_registry
from simcore.viz import TopDownView


@pytest.fixture()
def cfg():
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0
    c.scoring.enabled = False
    c.motion.realistic = False  # crisp snap motion: exact geometry under test
    return c


def _centroid(quad):
    return (sum(p[0] for p in quad) / 4.0, sum(p[1] for p in quad) / 4.0)


# --------------------------------------------------------------------------- #
# Pure geometry vs the render's own pose+pitch maths
# --------------------------------------------------------------------------- #

def test_footprint_corners_match_known_pose_pitch_down(cfg):
    """Drone at world (2, 3, 1.5), nose north (yaw pi/2), camera straight
    down: the quad must be the analytic frustum/ground intersection."""
    cam = cfg.camera
    pos, yaw, z = (2.0, 3.0, 1.5), math.pi / 2, 1.5
    quad = frames.camera_ground_footprint(cfg, pos, yaw, 90.0)

    eye_x, eye_y = 2.0, 3.0 + cam.mount_offset_m  # lens sits ahead of centre
    tan_h = math.tan(math.radians(cam.h_fov_deg) / 2)
    tan_v = tan_h * cam.height / cam.width
    # pitch 90: forward=(0,0,-1), up=nose=(0,1,0), right=east=(1,0,0)
    expected = [(eye_x + sh * tan_h * z, eye_y + sv * tan_v * z)
                for sv, sh in ((-1, -1), (-1, 1), (1, 1), (1, -1))]
    for got, exp in zip(quad, expected):
        assert got == pytest.approx(exp, abs=1e-9)


def test_pitch_forward_vs_down_differ_correctly(cfg):
    pos, yaw = (2.0, 3.0, 1.5), math.pi / 2  # nose north

    down = frames.camera_ground_footprint(cfg, pos, yaw, 90.0)
    cn = _centroid(down)
    assert math.dist(cn, (2.0, 3.1)) < 0.05        # compact quad UNDER it
    assert all(math.dist(p, cn) < 1.6 for p in down)

    fwd = frames.camera_ground_footprint(cfg, pos, yaw, 0.0, max_range_m=8.0)
    cf = _centroid(fwd)
    assert cf[1] - 3.0 > 1.0                       # AHEAD of the drone (north)
    assert abs(cf[0] - 2.0) < 0.1                  # straight along the nose
    # near edge (bottom of image) hits the ground ahead at z/tan(v_half)
    tan_v = (math.tan(math.radians(cfg.camera.h_fov_deg) / 2)
             * cfg.camera.height / cfg.camera.width)
    near_y = (down is not None) and min(p[1] for p in fwd)
    assert near_y == pytest.approx(3.0 + cfg.camera.mount_offset_m
                                   + 1.5 / tan_v, abs=0.05)
    assert fwd != down


def test_angular_size_fixed_no_zoom(cfg):
    """Footprint width scales ONLY with geometry (height), never a zoom
    parameter: doubling the altitude doubles the nadir footprint width."""
    w = []
    for z in (1.5, 3.0):
        quad = frames.camera_ground_footprint(cfg, (2.0, 3.0, z),
                                              math.pi / 2, 90.0)
        w.append(max(p[0] for p in quad) - min(p[0] for p in quad))
    assert w[1] / w[0] == pytest.approx(2.0, abs=0.01)

    import pyhulax
    import pyhulax.core
    names = set(dir(pyhulax.DroneAPI)) | set(dir(pyhulax.core))
    assert not [n for n in names if "zoom" in n.lower()
                or "fov" in n.lower()]  # no zoom/FOV control anywhere public


# --------------------------------------------------------------------------- #
# Live: set_camera_angle swings the footprint; sim-thread rendering intact
# --------------------------------------------------------------------------- #

def test_set_camera_angle_changes_live_footprint(cfg):
    cfg.camera.use_egl = False
    reg = get_registry(cfg)
    try:
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        d.takeoff(150)
        drone = reg.drones[0]

        def live_footprint():
            return reg.run_on_sim_thread(
                lambda: frames.camera_ground_footprint(
                    cfg, drone.pos, drone.yaw, drone.camera_pitch_deg,
                    max_range_m=8.0))

        d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 0)
        fp0 = live_footprint()
        d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 90)
        fp90 = live_footprint()
        assert fp0 != fp90                          # the tilt swings it
        pos, _yaw = reg.drone_world_pose(0)
        assert math.dist(_centroid(fp90), pos[:2]) < 0.2   # under the drone
        assert _centroid(fp0)[1] - pos[1] > 1.0            # was ahead of it

        frame = reg.render_camera(drone)            # render path unaffected
        assert frame.shape == (cfg.camera.height, cfg.camera.width, 3)
    finally:
        shutdown_registry()


def test_headless_png_renders_with_fov_overlay(cfg, tmp_path):
    cfg.camera.use_egl = False
    assert cfg.viz.show_camera_fov is True          # overlay on by default
    reg = get_registry(cfg)
    try:
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        d.takeoff(150)
        d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 90)
        view = TopDownView(reg)
        view.sample()
        assert view._latest[0][6] is not None       # footprint sampled
        out = tmp_path / "fov.png"
        view.render_png(str(out))
        assert out.is_file() and out.stat().st_size > 10_000
    finally:
        shutdown_registry()
