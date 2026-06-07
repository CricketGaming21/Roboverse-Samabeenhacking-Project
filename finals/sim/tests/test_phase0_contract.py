"""Phase 0 acceptance test — the §4 API surface exists with the right signatures.

Checks presence + signatures only (inspect.signature / inspect.Parameter).
NO behaviour is tested here; bodies may raise NotImplementedError.
"""

import inspect
import threading
from dataclasses import fields, is_dataclass
from enum import IntEnum, IntFlag

import pytest


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #

def assert_params(func, expected):
    """Assert a callable's parameter (name, default) pairs, excluding self.

    expected: list of (name, default) — default is inspect.Parameter.empty
    for required params.
    """
    sig = inspect.signature(func)
    params = [(n, p.default) for n, p in sig.parameters.items() if n != "self"]
    assert params == expected, (
        f"{getattr(func, '__qualname__', func)}: signature mismatch\n"
        f"  expected: {expected}\n  actual:   {params}"
    )


E = inspect.Parameter.empty


# --------------------------------------------------------------------------- #
# Imports — the public surface importable exactly as mission code does it
# --------------------------------------------------------------------------- #

def test_public_imports():
    import pyhulax
    from pyhulax import DroneAPI, Dola  # noqa: F401
    import pyhulax.core  # noqa: F401
    import pyhulax.video  # noqa: F401
    import pyhulax.exceptions  # noqa: F401
    from UWBParserThread import UWBParserThread  # noqa: F401
    assert hasattr(pyhulax, "DroneAPI")
    assert hasattr(pyhulax, "Dola")


# --------------------------------------------------------------------------- #
# §4.6 Enums
# --------------------------------------------------------------------------- #

def test_direction_enum():
    from pyhulax.core import Direction
    assert issubclass(Direction, IntEnum)
    assert Direction.FORWARD == 0
    assert Direction.BACK == 1
    assert Direction.LEFT == 2
    assert Direction.RIGHT == 3
    assert Direction.UP == 4
    assert Direction.DOWN == 5


def test_velocity_level_enum():
    from pyhulax.core import VelocityLevel
    assert issubclass(VelocityLevel, IntEnum)
    # Real firmware uses the value as a gain divisor: SLOW largest.
    assert VelocityLevel.ZOOM == 100          # contract: ZOOM = 100
    assert {"SLOW", "MEDIUM", "ZOOM", "TURBO"} <= set(VelocityLevel.__members__)
    assert VelocityLevel.SLOW == 300
    assert VelocityLevel.MEDIUM == 200
    assert VelocityLevel.TURBO == 50


def test_camera_pitch_mode_enum():
    from pyhulax.core import CameraPitchMode
    assert issubclass(CameraPitchMode, IntEnum)
    assert CameraPitchMode.UP_ABSOLUTE == 0   # contract: UP_ABSOLUTE = 0
    assert "DOWN_ABSOLUTE" in CameraPitchMode.__members__
    assert "CALIBRATE" in CameraPitchMode.__members__


def test_vision_mode_enum():
    from pyhulax.core import VisionMode
    assert VisionMode.OPTICAL_FLOW == 0
    assert VisionMode.FRONT_CAMERA == 1


def test_takeoff_flags_enum():
    from pyhulax.core import TakeoffFlags
    assert issubclass(TakeoffFlags, (IntEnum, IntFlag))
    assert TakeoffFlags.NONE == 0
    assert TakeoffFlags.RESET_YAW == 1
    assert TakeoffFlags.WITH_LOAD == 2
    # Flags must be combinable
    assert int(TakeoffFlags.RESET_YAW | TakeoffFlags.WITH_LOAD) == 3


def test_barrier_mask_enum():
    from pyhulax.core import BarrierMask
    assert issubclass(BarrierMask, (IntEnum, IntFlag))
    for name in ("FRONT", "BACK", "LEFT", "RIGHT", "UP", "DOWN"):
        assert name in BarrierMask.__members__
    assert BarrierMask.HORIZONTAL == (BarrierMask.FRONT | BarrierMask.BACK
                                      | BarrierMask.LEFT | BarrierMask.RIGHT)
    assert BarrierMask.ALL == (BarrierMask.HORIZONTAL | BarrierMask.UP
                               | BarrierMask.DOWN)


# --------------------------------------------------------------------------- #
# §4.6 Models (real dataclasses — these must WORK in Phase 0)
# --------------------------------------------------------------------------- #

def test_vector3_and_orientation():
    from pyhulax.core import Vector3, Orientation
    assert is_dataclass(Vector3) and is_dataclass(Orientation)
    v = Vector3(1.0, 2.0, 3.0)
    assert (v.x, v.y, v.z) == (1.0, 2.0, 3.0)
    o = Orientation(yaw=90.0, pitch=0.0, roll=-5.0)
    assert (o.yaw, o.pitch, o.roll) == (90.0, 0.0, -5.0)
    with pytest.raises(Exception):  # frozen
        v.x = 9.0


def test_obstacles_model():
    from pyhulax.core import Obstacles
    assert is_dataclass(Obstacles)
    empty = Obstacles()
    assert empty.any is False
    assert (empty.forward, empty.back, empty.left, empty.right, empty.down) == (
        False, False, False, False, False)
    # bits: 0=forward 1=back 2=left 3=right 4=down
    ob = Obstacles.from_bitmask(0b10101)
    assert ob.forward is True and ob.left is True and ob.down is True
    assert ob.back is False and ob.right is False
    assert ob.any is True


def test_command_result_model():
    from pyhulax.core import CommandResult
    assert is_dataclass(CommandResult)
    ok = CommandResult()
    assert ok.success is True and ok.message == ""
    assert bool(ok) is True
    assert bool(CommandResult(success=False, message="nope")) is False


def test_drone_state_model():
    from pyhulax.core import DroneState
    assert is_dataclass(DroneState)
    names = [f.name for f in fields(DroneState)]
    assert names == ["connected", "position", "orientation", "altitude",
                     "battery", "obstacles", "flying"]


def test_airesult_model():
    from pyhulax.core import AIResult, Vector3
    assert is_dataclass(AIResult)
    r = AIResult()
    assert r.success is False
    assert isinstance(r.position, Vector3)
    assert hasattr(r, "angle") and hasattr(r, "target_id")


# --------------------------------------------------------------------------- #
# §4.7 Exceptions
# --------------------------------------------------------------------------- #

def test_exception_hierarchy():
    from pyhulax.exceptions import (PyhulaxError, NotReady, LowBattery,
                                    TelemetryUnavailable)
    assert issubclass(PyhulaxError, Exception)
    for exc in (NotReady, LowBattery, TelemetryUnavailable):
        assert issubclass(exc, PyhulaxError)


# --------------------------------------------------------------------------- #
# §4.1 Dola
# --------------------------------------------------------------------------- #

def test_dola_signatures():
    from pyhulax import Dola
    assert_params(Dola.__init__, [("listen_ip", "0.0.0.0")])
    assert_params(Dola.start, [])
    assert_params(Dola.stop, [])
    assert_params(Dola.get_all_ips, [("listen_seconds", 5.0)])


# --------------------------------------------------------------------------- #
# §4.2–§4.5 DroneAPI
# --------------------------------------------------------------------------- #

def test_droneapi_control_signatures():
    from pyhulax import DroneAPI
    from pyhulax.core import TakeoffFlags, VelocityLevel

    assert_params(DroneAPI.__init__, [])
    assert_params(DroneAPI.connect, [("ip", E)])
    assert_params(DroneAPI.takeoff, [
        ("height_cm", 100), ("led", None), ("blocking", True),
        ("flags", TakeoffFlags.NONE)])
    assert_params(DroneAPI.land, [("led", None), ("blocking", True)])
    assert_params(DroneAPI.hover, [
        ("duration_seconds", E), ("led", None), ("blocking", True)])
    assert_params(DroneAPI.move, [
        ("direction", E), ("distance_cm", E), ("led", None),
        ("blocking", True), ("speed", VelocityLevel.ZOOM)])
    assert_params(DroneAPI.rotate, [
        ("angle_degrees", E), ("led", None), ("blocking", True)])
    assert_params(DroneAPI.move_to, [
        ("x", E), ("y", E), ("z", E), ("led", None),
        ("blocking", True), ("speed", VelocityLevel.ZOOM)])


def test_droneapi_telemetry_signatures():
    from pyhulax import DroneAPI
    assert_params(DroneAPI.get_state, [])
    assert_params(DroneAPI.get_position, [])
    assert_params(DroneAPI.get_orientation, [])
    assert_params(DroneAPI.get_altitude, [])
    assert_params(DroneAPI.get_battery, [])


def test_droneapi_obstacle_signatures():
    from pyhulax import DroneAPI
    from pyhulax.core import BarrierMask
    assert_params(DroneAPI.get_obstacles, [("drone_id", 0)])
    assert_params(DroneAPI.any_obstacle, [])
    assert_params(DroneAPI.get_drone_status, [("drone_id", 0)])
    assert_params(DroneAPI.set_barrier_mode, [("enabled", E)])
    assert_params(DroneAPI.set_avoidance_direction, [
        ("direction", E), ("distance_cm", 0),
        ("barrier_mask", BarrierMask.ALL), ("blocking", True)])


def test_droneapi_camera_signatures():
    from pyhulax import DroneAPI
    assert_params(DroneAPI.set_camera_angle, [("mode", E), ("angle", 0)])
    assert_params(DroneAPI.create_video_stream, [])
    assert_params(DroneAPI.set_video_stream, [("enabled", E)])


def test_droneapi_edu_stubs_present():
    """Stubbed real-SDK edu features exist on the surface (bodies may raise)."""
    from pyhulax import DroneAPI
    for name in ("curve_to", "circle", "recognize_qr", "detect_qr", "track_qr",
                 "recognize_target", "follow_line", "fire_laser", "set_clamp"):
        assert callable(getattr(DroneAPI, name)), f"missing edu stub: {name}"


# --------------------------------------------------------------------------- #
# §4.5 Video surface
# --------------------------------------------------------------------------- #

def test_video_surface():
    from pyhulax.video import VideoFrame, VideoStream, VideoDisplay

    assert isinstance(VideoFrame.width, property)
    assert isinstance(VideoFrame.height, property)
    assert_params(VideoFrame.to_rgb, [])
    assert_params(VideoFrame.to_bgr, [])

    assert_params(VideoStream.start, [])
    assert_params(VideoStream.stop, [])
    assert isinstance(VideoStream.latest_frame, property)
    assert isinstance(VideoStream.fps, property)

    assert_params(VideoDisplay.__init__, [
        ("stream", E), ("window_name", "hula"), ("show_fps", True)])
    assert_params(VideoDisplay.start, [])
    assert_params(VideoDisplay.stop, [])


# --------------------------------------------------------------------------- #
# §4.8 UWBParserThread
# --------------------------------------------------------------------------- #

def test_uwb_parser_thread_surface():
    from UWBParserThread import UWBParserThread
    assert issubclass(UWBParserThread, threading.Thread)
    assert_params(UWBParserThread.__init__, [
        ("x_origin", 0.0), ("y_origin", 0.0),
        ("serial_port", None), ("baud_rate", 921600)])
    assert_params(UWBParserThread.detect_com_port, [])
    assert_params(UWBParserThread.run, [])
    assert_params(UWBParserThread.get_tag_position, [("tag_id", E)])
    assert_params(UWBParserThread.stop, [])


# --------------------------------------------------------------------------- #
# Config wiring (Phase 0 requires sim_config.yaml -> simcore.config to work)
# --------------------------------------------------------------------------- #

def test_config_loads_yaml():
    from simcore.config import load_config, SimConfig
    cfg = load_config("sim_config.yaml")
    assert isinstance(cfg, SimConfig)
    # spot-check values flowing through from the YAML
    assert cfg.meta.seed == 42
    assert cfg.arena.length_m == 10.0
    assert len(cfg.drones.units) == 3
    assert cfg.drones.units[1].uwb_tag_id == 1
    assert cfg.drones.battery.low_threshold_pct == 10
    assert cfg.uwb.apply_origin_offset is False
    assert [p.id for p in cfg.pads] == [10, 11, 12]
    assert cfg.rovers.marker_ids == [20, 21, 22, 23, 24]
    assert cfg.scoring.hold_frames == 5
    assert cfg.aruco.dictionary == "DICT_6X6_250"
