"""DroneAPI — the public control/telemetry surface of the (simulated) pyhulax SDK.

Signatures are the hard contract from HULA_SIM_BUILD_PLAN.md §4.2–§4.5.
Implemented so far: connection + control + battery (Phase 2). Telemetry
beyond battery is Phase 3; obstacles Phase 4; camera/video Phase 5.

Blocking semantics: blocking commands return only once the move completes in
SIM time (or times out via the sim clock); blocking=False returns immediately
after the goal is accepted and motion continues on the sim thread. Motion
commands raise NotReady if not connected/flying and LowBattery below the
configured threshold (land() is never battery-gated). led/flags arguments
are accepted and ignored by the sim.
"""

from typing import Optional, Union

from . import _bridge
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
from .exceptions import NotReady, TelemetryUnavailable
from .video import VideoStream


class DroneAPI:
    """One instance controls one drone (bound by connect(ip))."""

    def __init__(self) -> None:
        self._reg = None
        self._drone = None

    def _require_connection(self):
        if self._drone is None:
            raise NotReady("not connected — call connect(ip) first")

    # ------------------------------------------------------------------ #
    # Connection + control (blocking by default) — §4.2
    # ------------------------------------------------------------------ #

    def connect(self, ip: str) -> CommandResult:
        """Bind this instance to the sim drone whose configured IP == ip.

        Boots the shared sim world on first call.
        """
        self._reg, self._drone = _bridge.connect(ip)
        return CommandResult(True, f"connected to {ip}")

    def takeoff(self, height_cm: int = 100, led=None, blocking: bool = True,
                flags: Union[TakeoffFlags, int] = TakeoffFlags.NONE) -> CommandResult:
        self._require_connection()
        return _bridge.submit(self._reg, self._drone, blocking, "takeoff",
                              height_cm=height_cm, flags=int(flags))

    def land(self, led=None, blocking: bool = True) -> CommandResult:
        self._require_connection()
        return _bridge.submit(self._reg, self._drone, blocking, "land")

    def hover(self, duration_seconds: float, led=None,
              blocking: bool = True) -> CommandResult:
        self._require_connection()
        return _bridge.submit(self._reg, self._drone, blocking, "hover",
                              duration_seconds=duration_seconds)

    def move(self, direction: Direction, distance_cm: float, led=None,
             blocking: bool = True,
             speed: Union[VelocityLevel, int] = VelocityLevel.ZOOM) -> CommandResult:
        """Relative move along the CURRENT heading (body frame).

        FORWARD = nose direction (follows the nose after any rotate()).
        """
        self._require_connection()
        return _bridge.submit(self._reg, self._drone, blocking, "move",
                              direction=direction, distance_cm=distance_cm,
                              speed=speed)

    def rotate(self, angle_degrees: float, led=None,
               blocking: bool = True) -> CommandResult:
        """Yaw by angle. Positive = CCW (left), negative = CW (right)."""
        self._require_connection()
        return _bridge.submit(self._reg, self._drone, blocking, "rotate",
                              angle_degrees=angle_degrees)

    def move_to(self, x: float, y: float, z: float, led=None,
                blocking: bool = True,
                speed: Union[VelocityLevel, int] = VelocityLevel.ZOOM) -> CommandResult:
        """Straight-line flight to a point in the TAKEOFF-ORIGIN frame, in cm.

        (x = right, y = forward, z = up; relative to where this drone took
        off — the frame is fixed at takeoff and does NOT rotate with yaw.)
        """
        self._require_connection()
        return _bridge.submit(self._reg, self._drone, blocking, "move_to",
                              x=x, y=y, z=z, speed=speed)

    # ------------------------------------------------------------------ #
    # Telemetry — §4.3 (each raises TelemetryUnavailable if no data yet)
    # ------------------------------------------------------------------ #

    def _require_telemetry(self):
        if self._drone is None:
            raise TelemetryUnavailable("no telemetry before connect()")

    def get_state(self) -> DroneState:
        self._require_telemetry()
        return _bridge.read_state(self._reg, self._drone)

    def get_position(self) -> Vector3:
        """cm, takeoff-origin frame, WITH drift (optical-flow/IMU estimate).

        Drifts away from truth over time — correct it with UWB; UWB never
        drifts. Frame is frozen at takeoff (x=right, y=forward, z=up).
        """
        self._require_telemetry()
        return _bridge.read_position(self._reg, self._drone)

    def get_orientation(self) -> Orientation:
        """Degrees (yaw, pitch, roll). Yaw = CCW from the takeoff heading."""
        self._require_telemetry()
        return _bridge.read_orientation(self._reg, self._drone)

    def get_altitude(self) -> float:
        """cm, downward ToF (true height — never from UWB)."""
        self._require_telemetry()
        return _bridge.read_altitude(self._reg, self._drone)

    def get_battery(self) -> int:
        """0-100. Linear drain while flying (config drones.battery)."""
        self._require_telemetry()
        return _bridge.read_battery(self._reg, self._drone)

    # ------------------------------------------------------------------ #
    # Obstacles (barrier sensors) — §4.4
    # ------------------------------------------------------------------ #

    def get_obstacles(self, drone_id: int = 0) -> Obstacles:
        """Five booleans from simulated IR/ToF barrier sensors (coarse by
        design — no distances). Below the min-altitude gate all report clear.
        Empty Obstacles if no data (before connect). drone_id is ignored —
        this instance is bound to one drone."""
        if self._drone is None:
            return Obstacles()
        return _bridge.read_obstacles(self._reg, self._drone)

    def any_obstacle(self) -> bool:
        if self._drone is None:
            return False
        return _bridge.read_obstacles(self._reg, self._drone).any

    def get_drone_status(self, drone_id: int = 0) -> Optional[int]:
        """Raw status int; barrier bits: 0=forward 1=back 2=left 3=right 4=down.
        None if no data (before connect). drone_id is ignored."""
        if self._drone is None:
            return None
        return _bridge.read_status(self._reg, self._drone)

    def set_barrier_mode(self, enabled: bool) -> CommandResult:
        """Enable/disable firmware auto-avoid: a move into an obstacle stops
        short (the goal fails with 'stopped short') instead of penetrating."""
        self._require_connection()
        return _bridge.set_barrier_mode(self._reg, self._drone, enabled)

    def set_avoidance_direction(self, direction: Direction, distance_cm: int = 0,
                                barrier_mask: Union[BarrierMask, int] = BarrierMask.ALL,
                                blocking: bool = True) -> CommandResult:
        """Conditional reflex: when a sensor in barrier_mask trips, move
        `direction` by distance_cm. Only fires when an obstacle is actually
        detected; distance_cm=0 disarms. The reflex step preempts whatever
        goal is active (logged as a preemption)."""
        self._require_connection()
        return _bridge.set_avoidance(self._reg, self._drone, direction,
                                     distance_cm, barrier_mask)

    # ------------------------------------------------------------------ #
    # Camera + video — §4.5
    # ------------------------------------------------------------------ #

    def set_camera_angle(self, mode: CameraPitchMode, angle: int = 0) -> CommandResult:
        """Tilt the main camera. angle in degrees 0-90 (DOWN_ABSOLUTE 90 =
        straight down, UP_ABSOLUTE 0 = straight ahead)."""
        self._require_connection()
        return _bridge.set_camera_angle(self._reg, self._drone, mode, angle)

    def create_video_stream(self) -> VideoStream:
        """Stream of real rendered frames from this drone's tiltable camera."""
        self._require_connection()
        return VideoStream(self._reg, self._drone)

    def set_video_stream(self, enabled: bool) -> CommandResult:
        self._require_connection()
        return _bridge.set_video_enabled(self._reg, self._drone, enabled)

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
