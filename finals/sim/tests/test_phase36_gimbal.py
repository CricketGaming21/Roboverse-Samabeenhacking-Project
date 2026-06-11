"""Phase 36 acceptance — the rover MOVING-GIMBAL marker model (headless).

Each rover's ArUco marker (0.20 m) sits on a gimbal whose facing yaw sweeps
over time from a seeded per-rover phase. A drone DECODES the marker only when
the gimbal faces within +/-readable_halfangle_deg of the bearing from the
rover to that drone; outside the cone the rendered frame shows the rover body
but NO decodable marker (the marker is hidden per-drone). ON by default for
convoy AND mixed. --dump exposes each rover's marker yaw + which drones can
read it. The public pyhulax/UWB surface is untouched (gimbal is sim-internal).

NOT mission logic: this only proves the sim models the moving marker honestly,
so the mission's own cv2.aruco read can be tested against it.
"""

import math

import cv2
import numpy as np
import pytest

from pyhulax import DroneAPI
from pyhulax.core import CameraPitchMode

from simcore import aruco_assets, frames
from simcore.config import load_config
from simcore.debug import DebugProbe
from simcore.registry import get_registry, shutdown_registry


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _scan_cfg():
    """Single PARKED rover on a clean procedural floor: a drone can be placed
    over it deterministically while the gimbal stays ON (sweep frozen below
    where the geometry must be exact)."""
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0
    c.scoring.enabled = False
    c.arena.layout = "procedural"
    c.arena.obstacles.count = 0
    c.rovers.count = 1
    c.rovers.motion = "patrol"
    c.rovers.patrol.speed_mps = 0.0
    c.scenario.phases = "ambush"          # rover sits in-arena at its spawn
    c.motion.realistic = False            # crisp snap: exact geometry under test
    return c


def _camera_over(d, cfg, north, east, alt_cm=150):
    """Drive drone 0 so its (downward) camera sits over arena (north, east)."""
    start = cfg.drones.units[0].start
    d.move_to((east - start[1]) * 100.0,
              (north - start[0]) * 100.0 - cfg.camera.mount_offset_m * 100.0,
              alt_cm)


# --------------------------------------------------------------------------- #
# Config: 0.20 m marker + gimbal defaults (ON, 45 deg/s sweep, +/-60 cone)
# --------------------------------------------------------------------------- #

def test_marker_size_is_20cm_and_gimbal_defaults():
    c = load_config("sim_config.yaml")
    assert c.aruco.rover_marker_size_m == pytest.approx(0.20)
    g = c.rovers.gimbal
    assert g.enabled is True                       # ON by default
    assert g.sweep_deg_per_s == pytest.approx(45.0)
    assert g.readable_halfangle_deg == pytest.approx(60.0)


# --------------------------------------------------------------------------- #
# Facing yaw: rotates at the configured rate, seeded + per-rover distinct
# --------------------------------------------------------------------------- #

def test_marker_yaw_rotates_seeded_and_per_rover_distinct():
    c = load_config("sim_config.yaml")          # default motion = convoy
    reg = get_registry(c)
    try:
        r0 = reg.rovers[0]
        assert r0._gimbal_on is True             # default-ON in convoy mode

        # sweeps at exactly sweep_deg_per_s
        step = _wrap(r0.marker_yaw(1.0) - r0.marker_yaw(0.0))
        assert abs(step - math.radians(c.rovers.gimbal.sweep_deg_per_s)) < 1e-6

        # the t=0 facing is the seeded per-rover phase (deterministic)
        expect0 = float(np.random.default_rng(
            [c.meta.seed, 5000 + 0]).uniform(0.0, 2.0 * math.pi))
        assert abs(_wrap(r0.marker_yaw(0.0) - expect0)) < 1e-9

        # different rovers carry different phases (seeded per index)
        assert abs(_wrap(reg.rovers[0].marker_yaw(0.0)
                         - reg.rovers[1].marker_yaw(0.0))) > 1e-3
    finally:
        shutdown_registry()


def test_gimbal_default_on_in_mixed_mode():
    c = load_config("sim_config.yaml")
    c.rovers.motion = "mixed"                     # 3 auto + 2 evasive
    reg = get_registry(c)
    try:
        assert all(r._gimbal_on for r in reg.rovers)   # ON for the whole fleet
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# HONEST gating: the readable cone flips the REAL cv2.aruco decode on/off.
# Same rover, same +/-60 cone, only the viewing direction differs.
# --------------------------------------------------------------------------- #

def test_readable_cone_gates_real_cv2_aruco_decode():
    c = _scan_cfg()
    c.rovers.gimbal.enabled = True
    c.rovers.gimbal.sweep_deg_per_s = 0.0         # FREEZE facing -> exact geometry
    c.rovers.gimbal.readable_halfangle_deg = 60.0
    reg = get_registry(c)
    try:
        rover = reg.rovers[0]
        rover_id = rover.marker_id
        rn, re_ = reg.rover_arena_positions()[0]
        face = rover.marker_yaw(0.0)              # frozen world-frame facing
        detector = cv2.aruco.ArucoDetector(
            aruco_assets.get_dictionary(c.aruco.dictionary),
            cv2.aruco.DetectorParameters())

        d = DroneAPI()
        d.connect(c.drones.units[0].ip)
        d.takeoff(150)
        d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 90)

        R = 0.4   # offset so the rover stays well in frame yet sets the bearing

        # --- IN the cone: drone offset toward the marker's facing -> decodes ---
        _camera_over(d, c, rn + R * math.sin(face), re_ + R * math.cos(face))
        pose, _ = reg.drone_world_pose(0)
        assert rover.marker_readable_by(0.0, (pose[0], pose[1])) is True
        rgb = reg.render_camera(reg.drones[0])
        _c1, ids_in, _ = detector.detectMarkers(
            cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
        assert ids_in is not None and rover_id in ids_in.flatten()

        # --- OUT of the cone: drone offset AWAY -> body seen, marker hidden ---
        _camera_over(d, c, rn - R * math.sin(face), re_ - R * math.cos(face))
        pose, _ = reg.drone_world_pose(0)
        assert rover.marker_readable_by(0.0, (pose[0], pose[1])) is False
        rgb = reg.render_camera(reg.drones[0])
        _c2, ids_out, _ = detector.detectMarkers(
            cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
        flat = [] if ids_out is None else list(ids_out.flatten())
        assert rover_id not in flat               # outside the cone: no decode
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# --dump exposes the per-tick marker yaw + which drones can read each marker
# --------------------------------------------------------------------------- #

def test_dump_exposes_marker_yaw_and_readable_by():
    c = _scan_cfg()
    c.rovers.count = 3
    c.rovers.gimbal.enabled = True               # default sweep (45 deg/s)
    reg = get_registry(c)
    try:
        d = DroneAPI()
        d.connect(c.drones.units[0].ip)
        d.takeoff(150)
        probe = DebugProbe(reg)

        snap = probe.snapshot()
        drone_ids = {dd["index"] for dd in snap["drones"]}
        for rd in snap["rovers"]:
            assert isinstance(rd["marker_yaw_deg"], float)
            assert 0.0 <= rd["marker_yaw_deg"] < 360.0
            assert isinstance(rd["readable_by"], list)
            assert all(isinstance(i, int) for i in rd["readable_by"])
            assert set(rd["readable_by"]) <= drone_ids   # valid drone indices

        # the moving gimbal is visible in the dump: facing advances over time
        import time
        time.sleep(0.3)                              # ~3 sim s at rtf 10
        later = probe.snapshot()
        moved = [abs(_wrap(math.radians(b["marker_yaw_deg"])
                           - math.radians(a["marker_yaw_deg"]))) > 1e-3
                 for a, b in zip(snap["rovers"], later["rovers"])]
        assert all(moved)                            # every marker yaw rotated
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# The gimbal is sim-internal: the public pyhulax control surface is unchanged
# --------------------------------------------------------------------------- #

def test_public_pyhulax_surface_has_no_gimbal():
    import pyhulax
    assert not any("gimbal" in name.lower() for name in dir(pyhulax.DroneAPI))
