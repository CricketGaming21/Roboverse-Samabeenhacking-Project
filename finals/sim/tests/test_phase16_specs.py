"""Phase 16 acceptance test — confirmed Hula HG-F09 specs baked in (headless).

Sources: HighGreat's official user manual + the pyhulax SDK. Camera FOV 71°,
programming-speed band capped at 1.0 m/s, asymmetric climb/descent 1.2/1.0,
~10 %/min battery (9-10 min endurance), 189x185x50 mm / 100 g airframe,
20° max body tilt (consumed by the Phase-18 motion model).
"""

import math
import time

import cv2
import pybullet as p
import pytest

from pyhulax import DroneAPI
from pyhulax.core import CameraPitchMode, VelocityLevel

from simcore import aruco_assets
from simcore.config import load_config
from simcore.drone_model import speed_to_mps
from simcore.registry import get_registry, shutdown_registry


@pytest.fixture()
def cfg():
    return load_config("sim_config.yaml")


# --------------------------------------------------------------------------- #
# The confirmed numbers, loaded from config
# --------------------------------------------------------------------------- #

def test_confirmed_spec_values(cfg):
    assert cfg.camera.h_fov_deg == 71.0
    assert cfg.camera.width == 640 and cfg.camera.height == 480  # "AI mode"

    v = cfg.velocity_levels
    assert (v.SLOW, v.MEDIUM, v.ZOOM, v.TURBO) == (0.3, 0.5, 0.8, 1.0)
    assert v.climb_mps == 1.2 and v.descent_mps == 1.0

    drain = cfg.drones.battery.drain_pct_per_min
    assert drain == 10.0
    endurance_min = cfg.drones.battery.start_pct / drain
    assert 9.0 <= endurance_min <= 10.0          # the confirmed flight time

    hx, hy, hz = cfg.bodies.drone_half_extents_m  # 189 x 185 x 50 mm
    assert (2 * hx, 2 * hy, 2 * hz) == (0.19, 0.185, 0.05)
    assert cfg.bodies.drone_mass_kg == 0.10

    assert cfg.motion.max_tilt_deg == 20.0       # body tilt (Phase 18 input)


def test_speed_band_capped_at_one_mps(cfg):
    for level in VelocityLevel:
        assert speed_to_mps(cfg, level) <= 1.0
    assert speed_to_mps(cfg, VelocityLevel.TURBO) == 1.0
    assert speed_to_mps(cfg, VelocityLevel.ZOOM) == 0.8


def test_detection_range_geometry_at_fov71(cfg):
    """0.15 m marker at the 40 px gate: range ~1.7 m at 640x480 / 71°
    (derived from config, not hardcoded — updates if specs change)."""
    tan_h = math.tan(math.radians(cfg.camera.h_fov_deg) / 2)
    marker = cfg.aruco.rover_marker_size_m
    gate = cfg.scoring.min_marker_px
    max_range = marker * (cfg.camera.width / 2) / (gate * tan_h)
    assert 1.6 <= max_range <= 1.8               # ~1.7 m per the spec sheet
    # the 0.30 m pad marker correspondingly reaches ~3.4 m
    assert 3.2 <= max_range * 2 <= 3.6


# --------------------------------------------------------------------------- #
# The specs flow into the live world
# --------------------------------------------------------------------------- #

def test_rendered_marker_px_matches_fov71(cfg):
    """Render a parked rover from 1.5 m and check the measured marker side
    matches the 71° projection analytically — proves the FOV reached the
    camera, not just the YAML."""
    cfg.meta.real_time_factor = 10.0
    cfg.scoring.enabled = False
    cfg.arena.layout = "procedural"
    cfg.arena.obstacles.count = 0
    cfg.rovers.motion = "patrol"
    cfg.rovers.patrol.speed_mps = 0.0
    cfg.scenario.phases = "ambush"
    reg = get_registry(cfg)
    try:
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
        detector = cv2.aruco.ArucoDetector(
            aruco_assets.get_dictionary(cfg.aruco.dictionary),
            cv2.aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(
            cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
        assert ids is not None and reg.rovers[0].marker_id in ids.flatten()
        quad = corners[list(ids.flatten()).index(reg.rovers[0].marker_id)][0]
        side = max(float(math.dist(quad[k], quad[(k + 1) % 4]))
                   for k in range(4))
        marker_total = (cfg.aruco.rover_marker_size_m
                        * aruco_assets.texture_scale())
        dist = 1.5 - (2 * cfg.bodies.rover_half_extents_m[2] + 0.002)
        tan_h = math.tan(math.radians(cfg.camera.h_fov_deg) / 2)
        expected = (cfg.aruco.rover_marker_size_m
                    * (cfg.camera.width / 2) / (dist * tan_h))
        assert side == pytest.approx(expected, abs=5.0)
        assert side >= cfg.scoring.min_marker_px  # scan still valid at 1.5 m
        assert marker_total > 0  # (texture sizing sanity)
    finally:
        shutdown_registry()


def test_takeoff_climbs_faster_than_landing_descends(cfg):
    cfg.meta.real_time_factor = 10.0
    cfg.camera.use_egl = False
    cfg.scoring.enabled = False
    reg = get_registry(cfg)
    try:
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        t0 = reg.sim_time()
        d.takeoff(120)
        t_up = reg.sim_time() - t0          # ~1.2 m at 1.2 m/s
        t1 = reg.sim_time()
        d.land()
        t_down = reg.sim_time() - t1        # ~1.2 m at 1.0 m/s
        assert t_up == pytest.approx(1.2 / cfg.velocity_levels.climb_mps,
                                     abs=0.2)
        assert t_down == pytest.approx(1.2 / cfg.velocity_levels.descent_mps,
                                       abs=0.2)
        assert t_down > t_up                # the confirmed asymmetry
    finally:
        shutdown_registry()


def test_drone_bodies_have_real_dimensions(cfg):
    cfg.meta.real_time_factor = 10.0
    cfg.camera.use_egl = False
    cfg.scoring.enabled = False
    reg = get_registry(cfg)
    try:
        bid = reg.bodies.drones[0]
        lo, hi = reg.run_on_sim_thread(
            lambda: p.getAABB(bid, physicsClientId=reg.client))
        dims = [hi[k] - lo[k] for k in range(3)]
        # AABB is slightly padded by bullet; nose along +y at heading north
        assert dims[2] == pytest.approx(0.05, abs=0.02)   # 50 mm tall
        assert dims[0] == pytest.approx(0.185, abs=0.03)  # 185 mm wide
        assert dims[1] == pytest.approx(0.19, abs=0.03)   # 189 mm long
        assert reg.drones[0]._half_z == pytest.approx(0.025)
    finally:
        shutdown_registry()
