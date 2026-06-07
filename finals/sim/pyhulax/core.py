"""Core types and enums for the (simulated) pyhulax SDK.

The public surface here is the hard contract from HULA_SIM_BUILD_PLAN.md §4.6.
Enum VALUES match the real pyhulax SDK (pyhulax_complete_knowledge_base.txt):
  - Direction:      FORWARD=0 .. DOWN=5
  - VelocityLevel:  firmware uses the value as a P-gain divisor, so SLOW=300 .. TURBO=50
  - CameraPitchMode / VisionMode / TakeoffFlags / BarrierMask: real firmware values.
"""

from dataclasses import dataclass, field
from enum import IntEnum, IntFlag


# --------------------------------------------------------------------------- #
# Enums
# --------------------------------------------------------------------------- #

class Direction(IntEnum):
    """Movement direction for move() — body frame, relative to current heading."""
    FORWARD = 0
    BACK = 1
    LEFT = 2
    RIGHT = 3
    UP = 4
    DOWN = 5


class VelocityLevel(IntEnum):
    """Flight speed level. Real firmware uses the value as a gain DIVISOR,
    hence SLOW has the largest value. ZOOM=100 is the pyhulax default."""
    SLOW = 300
    MEDIUM = 200
    ZOOM = 100
    TURBO = 50


class CameraPitchMode(IntEnum):
    """Camera pitch control mode for set_camera_angle()."""
    UP_ABSOLUTE = 0
    DOWN_ABSOLUTE = 1
    CALIBRATE = 4
    UP_RELATIVE = 5
    DOWN_RELATIVE = 6


class VisionMode(IntEnum):
    """Camera selector for vision operations."""
    OPTICAL_FLOW = 0
    FRONT_CAMERA = 1


class TakeoffFlags(IntFlag):
    """Takeoff behaviour flags (combinable with |)."""
    NONE = 0
    RESET_YAW = 1
    WITH_LOAD = 2


class BarrierMask(IntFlag):
    """Obstacle sensor bitmask for set_avoidance_direction() (combinable with |)."""
    FRONT = 1
    BACK = 2
    LEFT = 4
    RIGHT = 8
    UP = 16
    DOWN = 32
    HORIZONTAL = FRONT | BACK | LEFT | RIGHT
    ALL = FRONT | BACK | LEFT | RIGHT | UP | DOWN


class LineColor(IntEnum):
    """Line colour for follow_line() (edu feature — stubbed in the sim)."""
    BLACK = 0
    WHITE = 255


# --------------------------------------------------------------------------- #
# Models
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class Vector3:
    """3D vector. Position contexts are centimetres in the takeoff-origin frame."""
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class Orientation:
    """Orientation angles in degrees."""
    yaw: float
    pitch: float
    roll: float


@dataclass(frozen=True)
class Obstacles:
    """Obstacle detection state from the 5-direction barrier sensors."""
    forward: bool = False
    back: bool = False
    left: bool = False
    right: bool = False
    down: bool = False

    @classmethod
    def from_bitmask(cls, barrier: int) -> "Obstacles":
        """Decode the status bitmask: bits 0=forward 1=back 2=left 3=right 4=down."""
        return cls(
            forward=bool(barrier & 0x01),
            back=bool(barrier & 0x02),
            left=bool(barrier & 0x04),
            right=bool(barrier & 0x08),
            down=bool(barrier & 0x10),
        )

    @property
    def any(self) -> bool:
        return self.forward or self.back or self.left or self.right or self.down


@dataclass(frozen=True)
class CommandResult:
    """Result of a DroneAPI command. Truthy on success."""
    success: bool = True
    message: str = ""

    def __bool__(self) -> bool:
        return self.success


@dataclass(frozen=True)
class DroneState:
    """Full telemetry snapshot returned by get_state()."""
    connected: bool
    position: Vector3
    orientation: Orientation
    altitude: float
    battery: int
    obstacles: Obstacles
    flying: bool


@dataclass(frozen=True)
class AIResult:
    """Result of the edu QR/target recognition features.

    Present only so the stubbed QR methods can type-return; never produced
    by the sim.
    """
    success: bool = False
    position: Vector3 = field(default=Vector3(0.0, 0.0, 0.0))
    angle: float = 0.0
    target_id: int = -1
