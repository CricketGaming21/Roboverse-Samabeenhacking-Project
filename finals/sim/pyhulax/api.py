"""DroneAPI — the public control/telemetry surface of the (simulated) pyhulax SDK.

Signatures are the hard contract from HULA_SIM_BUILD_PLAN.md §4.2–§4.5.
Phase 0: every body raises NotImplementedError. Behaviour lands in Phases 2–5.

Blocking semantics (Phase 2+): blocking commands return only once the move
completes in SIM time (or times out); blocking=False returns immediately
after the goal is accepted. Motion commands raise NotReady if not
connected/flying and LowBattery below the configured threshold.
"""

from typing import Optional, Union

from .core import (
    AIResult,
    BarrierMask,
    CameraPitchMode,
    CommandResult,
    Direction,
    DroneState,
    LineColor,
    Obstacles,
    Orientation,
    TakeoffFlags,
    Vector3,
    VelocityLevel,
    VisionMode,
)
from .video import VideoStream


class DroneAPI:
    """One instance controls one drone (bound by connect(ip))."""

    def __init__(self) -> None:
        raise NotImplementedError

    # ------------------------------------------------------------------ #
    # Connection + control (blocking by default) — §4.2
    # ------------------------------------------------------------------ #

    def connect(self, ip: str) -> CommandResult:
        """Bind this instance to the sim drone whose configured IP == ip.

        Boots the shared sim world on first call.
        """
        raise NotImplementedError

    def takeoff(self, height_cm: int = 100, led=None, blocking: bool = True,
                flags: Union[TakeoffFlags, int] = TakeoffFlags.NONE) -> CommandResult:
        raise NotImplementedError

    def land(self, led=None, blocking: bool = True) -> CommandResult:
        raise NotImplementedError

    def hover(self, duration_seconds: float, led=None,
              blocking: bool = True) -> CommandResult:
        raise NotImplementedError

    def move(self, direction: Direction, distance_cm: float, led=None,
             blocking: bool = True,
             speed: Union[VelocityLevel, int] = VelocityLevel.ZOOM) -> CommandResult:
        """Relative move along the CURRENT heading (body frame).

        FORWARD = nose direction (follows the nose after any rotate()).
        """
        raise NotImplementedError

    def rotate(self, angle_degrees: float, led=None,
               blocking: bool = True) -> CommandResult:
        """Yaw by angle. Positive = CCW (left), negative = CW (right)."""
        raise NotImplementedError

    def move_to(self, x: float, y: float, z: float, led=None,
                blocking: bool = True,
                speed: Union[VelocityLevel, int] = VelocityLevel.ZOOM) -> CommandResult:
        """Straight-line flight to a point in the TAKEOFF-ORIGIN frame, in cm.

        (x = right, y = forward, z = up; relative to where this drone took
        off — the frame is fixed at takeoff and does NOT rotate with yaw.)
        """
        raise NotImplementedError

    # ------------------------------------------------------------------ #
    # Telemetry — §4.3 (each raises TelemetryUnavailable if no data yet)
    # ------------------------------------------------------------------ #

    def get_state(self) -> DroneState:
        raise NotImplementedError

    def get_position(self) -> Vector3:
        """cm, takeoff-origin frame, WITH drift (optical-flow/IMU estimate)."""
        raise NotImplementedError

    def get_orientation(self) -> Orientation:
        """Degrees (yaw, pitch, roll)."""
        raise NotImplementedError

    def get_altitude(self) -> float:
        """cm, downward ToF."""
        raise NotImplementedError

    def get_battery(self) -> int:
        """0-100."""
        raise NotImplementedError

    # ------------------------------------------------------------------ #
    # Obstacles (barrier sensors) — §4.4
    # ------------------------------------------------------------------ #

    def get_obstacles(self, drone_id: int = 0) -> Obstacles:
        """Five booleans from simulated IR/ToF barrier sensors."""
        raise NotImplementedError

    def any_obstacle(self) -> bool:
        raise NotImplementedError

    def get_drone_status(self, drone_id: int = 0) -> Optional[int]:
        """Raw status int; barrier bits: 0=forward 1=back 2=left 3=right 4=down."""
        raise NotImplementedError

    def set_barrier_mode(self, enabled: bool) -> CommandResult:
        """Enable/disable the sim's automatic reflex avoidance."""
        raise NotImplementedError

    def set_avoidance_direction(self, direction: Direction, distance_cm: int = 0,
                                barrier_mask: Union[BarrierMask, int] = BarrierMask.ALL,
                                blocking: bool = True) -> CommandResult:
        """Conditional reflex: when a sensor in barrier_mask trips, move
        `direction` by distance_cm. Only fires when an obstacle is actually
        detected."""
        raise NotImplementedError

    # ------------------------------------------------------------------ #
    # Camera + video — §4.5
    # ------------------------------------------------------------------ #

    def set_camera_angle(self, mode: CameraPitchMode, angle: int = 0) -> CommandResult:
        """Tilt the main camera. angle in degrees 0-90 (DOWN_ABSOLUTE 90 =
        straight down, UP_ABSOLUTE 0 = straight ahead)."""
        raise NotImplementedError

    def create_video_stream(self) -> VideoStream:
        raise NotImplementedError

    def set_video_stream(self, enabled: bool) -> CommandResult:
        raise NotImplementedError

    # ------------------------------------------------------------------ #
    # Edu features present in the real SDK — surface kept complete, but
    # NOT simulated (not needed for Challenge 2). Permanently stubbed.
    # ------------------------------------------------------------------ #

    def curve_to(self, x: float, y: float, z: float, velocity,
                 radius_cm: Optional[float] = None, led=None,
                 blocking: bool = True) -> CommandResult:
        raise NotImplementedError("edu feature — not simulated")

    def circle(self, radius_cm: float, velocity, clockwise: bool = True,
               led=None, blocking: bool = True) -> CommandResult:
        raise NotImplementedError("edu feature — not simulated")

    def recognize_qr(self, mode: VisionMode = VisionMode.FRONT_CAMERA,
                     timeout: float = 5.0) -> AIResult:
        raise NotImplementedError("edu feature — not simulated (use cv2.aruco on frames)")

    def detect_qr(self, timeout: float = 5.0) -> AIResult:
        raise NotImplementedError("edu feature — not simulated (use cv2.aruco on frames)")

    def track_qr(self, timeout: float = 5.0) -> AIResult:
        raise NotImplementedError("edu feature — not simulated (use cv2.aruco on frames)")

    def recognize_target(self, target) -> AIResult:
        raise NotImplementedError("edu feature — not simulated")

    def follow_line(self, color: LineColor = LineColor.BLACK, speed=None,
                    blocking: bool = True):
        raise NotImplementedError("edu feature — not simulated")

    def fire_laser(self, count: int = 1, interval: float = 0.2,
                   blocking: bool = True) -> CommandResult:
        raise NotImplementedError("edu feature — not simulated")

    def set_clamp(self, is_open: Optional[bool] = None,
                  angle: Optional[int] = None) -> CommandResult:
        raise NotImplementedError("edu feature — not simulated")

    def set_electromagnet(self, on: bool) -> CommandResult:
        raise NotImplementedError("edu feature — not simulated")
