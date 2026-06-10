"""Fake `pyhulax` SDK — a deterministic, headless double of the PUBLIC API.

It mirrors the sim's **SUBSET** of the real SDK exactly (see docs/RECONCILIATION.md
and docs/SDK_REFERENCE.md). In particular it deliberately does **NOT** implement the
real-SDK-only methods (`set_app_mode`, `send_app_heartbeat`, `set_velocity_level`,
`stop_manual_control`, `arm`, `disarm`, `disconnect`, `get_velocity`, `get_drone_id`,
`enable_battery_failsafe`). Their *absence* is what forces the mission to route those
calls through `runtime/sdk_compat.py` `hasattr` guards. `Obstacles` has no `up` field.

The kinematic model is **time-stepped**, not wall-clock driven: each
`send_manual_control` advances a shared sim clock by `world.dt` and integrates the
pose by exactly that step, so every test is deterministic and runs without real
sleeps. UWB timestamps read this same clock (so a UWB-derived speed = Δpos/Δt is
deterministic too).

The camera renders REAL ArUco markers (`cv2.aruco.generateImageMarker`) warped onto
the floor with a correct pinhole projection, so `cv2.aruco.detectMarkers` genuinely
decodes them — perception tests exercise real OpenCV, not a stub.

Frame convention (matches docs): arena `x=North(m), y=East(m), z=Up(m)`; body
`forward,right,up`; locked-yaw body→arena rotation:
    north = forward*cos(yaw) - right*sin(yaw)
    east  = forward*sin(yaw) + right*cos(yaw)
The physical CCW sign / axis inversions are absorbed by config (`yaw_offset_deg`,
`invert_*`) on the day; the fake just needs to be self-consistent, and at the
operational locked yaw of 0 it is the identity (forward=+north, right=+east).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Callable, Dict, List, Optional, Tuple

import cv2
import numpy as np


# --------------------------------------------------------------------------- #
# Enums (pyhulax.core) — values per docs/SDK_REFERENCE.md
# --------------------------------------------------------------------------- #
class Direction(IntEnum):
    FORWARD = 0
    BACK = 1
    LEFT = 2
    RIGHT = 3
    UP = 4
    DOWN = 5


class CameraPitchMode(IntEnum):
    UP_ABSOLUTE = 0
    DOWN_ABSOLUTE = 1
    CALIBRATE = 4
    UP_RELATIVE = 5
    DOWN_RELATIVE = 6


class VelocityLevel(IntEnum):
    SLOW = 300
    MEDIUM = 200
    ZOOM = 100
    TURBO = 50


# Raw real-world speed band per level (m/s), matching the sim's velocity_levels table.
# The enum INTS above are firmware P-gain divisors (unchanged); these are the speeds the
# sim maps by NAME. ZOOM/TURBO (0.8/1.0) exceed the 0.5 cap — the fake (like the sim) HARD
# CLAMPS every level to max_mps so no VelocityLevel can ever exceed 0.5 m/s.
VELOCITY_LEVEL_RAW_MPS = {
    VelocityLevel.SLOW: 0.3,
    VelocityLevel.MEDIUM: 0.5,
    VelocityLevel.ZOOM: 0.8,
    VelocityLevel.TURBO: 1.0,
}


class BarrierMask(IntEnum):
    ALL = 0
    HORIZONTAL = 1     # the clean "never climb" mask
    VERTICAL = 2


class TakeoffFlags(IntEnum):
    NONE = 0
    RESET_YAW = 1
    WITH_LOAD = 2


class VideoResolution(IntEnum):
    LOW = 0
    MEDIUM = 1
    HIGH = 2


# --------------------------------------------------------------------------- #
# Small value types
# --------------------------------------------------------------------------- #
@dataclass
class CommandResult:
    """What `connect`/commands return. Truthy on success (does NOT raise on failure)."""
    success: bool = True
    message: str = ""

    def __bool__(self) -> bool:
        return bool(self.success)


@dataclass
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Orientation:
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0


@dataclass
class Obstacles:
    """Exactly the 5 sim fields — NO `up` (matches docs/RECONCILIATION.md §4)."""
    forward: bool = False
    back: bool = False
    left: bool = False
    right: bool = False
    down: bool = False

    @property
    def any(self) -> bool:
        return self.forward or self.back or self.left or self.right or self.down


@dataclass
class Marker:
    id: int
    n: float          # arena north (m)
    e: float          # arena east (m)
    z: float = 0.0    # height of the marker plane (m)
    size_m: float = 0.30   # printed marker side (m)
    motion: Optional[Callable[[float], Tuple[float, float]]] = None  # t -> (n,e)

    def pos_at(self, t: float) -> Tuple[float, float]:
        if self.motion is not None:
            return self.motion(t)
        return (self.n, self.e)


# --------------------------------------------------------------------------- #
# The shared world
# --------------------------------------------------------------------------- #
@dataclass
class FakeWorld:
    """Ground truth shared by the drones, the camera and the UWB fake."""
    length_m: float = 10.0          # arena North extent
    width_m: float = 6.0            # arena East extent
    # crate footprints: each (center_n, center_e, size_n, size_e)
    crates: List[Tuple[float, float, float, float]] = field(default_factory=list)
    pads: List[Marker] = field(default_factory=list)
    rovers: List[Marker] = field(default_factory=list)

    # control / kinematics
    dt: float = 0.05                # sim seconds advanced per send_manual_control
    max_mps: float = 0.5            # HARD horizontal speed cap
    climb_mps: float = 0.5          # vertical cap
    yaw_rate_dps: float = 90.0      # deg/s at full rotate stick

    # sensing
    barrier_range_m: float = 0.6    # horizontal IR/ToF range
    down_range_cm: float = 40.0     # down sensor trips this close to ground
    min_sense_alt_cm: float = 35.0  # horizontal sensors inactive below this
    climb_cmps: float = 50.0        # vertical speed for move(UP/DOWN), cm/s

    # camera intrinsics
    cam_w: int = 640
    cam_h: int = 480
    cam_hfov_deg: float = 71.0

    # onboard-estimate drift (per metre travelled), default 0 for clean tests
    drift_frac: float = 0.0

    # bookkeeping
    clock: float = 0.0
    ip_map: Dict[int, str] = field(default_factory=lambda: {0: "10.0.0.11",
                                                            1: "10.0.0.12",
                                                            2: "10.0.0.13"})
    drone_starts: Dict[int, Tuple[float, float]] = field(default_factory=lambda: {
        0: (0.6, 1.1), 1: (0.6, 0.6), 2: (1.1, 0.6)})

    _drones: Dict[int, "FakeDroneAPI"] = field(default_factory=dict)

    # -- helpers ---------------------------------------------------------- #
    def tag_for_ip(self, ip: str) -> int:
        for tag, known in self.ip_map.items():
            if known == ip:
                return tag
        raise ValueError(f"unknown drone ip {ip!r}")

    def register(self, drone: "FakeDroneAPI") -> None:
        self._drones[drone.tag_id] = drone

    def truth_xy(self, tag_id: int) -> Optional[Tuple[float, float]]:
        d = self._drones.get(tag_id)
        return None if d is None else (d.n, d.e)

    def point_in_crate(self, n: float, e: float, inflate: float = 0.0) -> bool:
        for cn, ce, sn, se in self.crates:
            if (abs(n - cn) <= sn / 2 + inflate and
                    abs(e - ce) <= se / 2 + inflate):
                return True
        return False

    def all_markers(self) -> List[Marker]:
        return list(self.pads) + list(self.rovers)

    def velocity_mps(self, level) -> float:
        """m/s for a VelocityLevel, HARD-clamped to max_mps (every level ≤ 0.5)."""
        try:
            lvl = level if isinstance(level, VelocityLevel) else VelocityLevel(int(level))
        except (ValueError, TypeError):
            return self.max_mps
        raw = VELOCITY_LEVEL_RAW_MPS.get(lvl, self.max_mps)
        return min(raw, self.max_mps)


def default_world() -> FakeWorld:
    """A standard arena: 10x6, the five config pads, two static rovers, two crates."""
    w = FakeWorld()
    w.pads = [
        Marker(10, 8.5, 3.0), Marker(11, 5.5, 4.8), Marker(12, 2.0, 4.5),
        Marker(13, 5.5, 1.2), Marker(14, 2.0, 1.5),
    ]
    w.rovers = [Marker(20, 4.0, 3.0, size_m=0.18), Marker(21, 6.0, 2.0, size_m=0.18)]
    w.crates = [(5.0, 3.0, 0.9, 0.9), (7.5, 4.2, 0.9, 0.9)]
    return w


# --------------------------------------------------------------------------- #
# Camera rendering (real ArUco, real pinhole)
# --------------------------------------------------------------------------- #
_ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)


def _signed_area(pts: np.ndarray) -> float:
    a = 0.0
    for i in range(len(pts)):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % len(pts)]
        a += x1 * y2 - x2 * y1
    return a


def _camera_basis(yaw_rad: float, pitch_down_rad: float):
    """Return (optical z_cam, image-right x_cam, image-down y_cam) in arena (n,e,up).

    pitch_down: 0 = looking forward at the horizon, +pi/2 = straight down.
    Right-handed (non-mirrored) so ArUco decodes the warped marker.
    """
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    cp, sp = math.cos(pitch_down_rad), math.sin(pitch_down_rad)
    f_h = np.array([cy, sy, 0.0])              # body forward, horizontal
    x_cam = np.array([-sy, cy, 0.0])           # body right
    z_cam = f_h * cp + np.array([0.0, 0.0, -1.0]) * sp   # optical axis (look dir)
    y_cam = np.cross(z_cam, x_cam)             # image-down (right-handed)
    return z_cam, x_cam, y_cam


def render_frame(world: FakeWorld, drone: "FakeDroneAPI") -> np.ndarray:
    """Render the drone's current camera view as an RGB uint8 ndarray with the
    visible ArUco markers warped onto the floor."""
    W, H = world.cam_w, world.cam_h
    frame = np.full((H, W, 3), 170, np.uint8)   # flat grey floor

    fx = (W / 2.0) / math.tan(math.radians(world.cam_hfov_deg) / 2.0)
    fy = fx
    cx, cy = W / 2.0, H / 2.0

    cam = np.array([drone.n, drone.e, drone.alt_cm / 100.0])
    z_cam, x_cam, y_cam = _camera_basis(math.radians(drone.yaw_deg),
                                        math.radians(drone.pitch_down_deg))

    def project(q: np.ndarray):
        d = q - cam
        zc = float(d @ z_cam)
        if zc <= 1e-6:
            return None
        u = cx + fx * float(d @ x_cam) / zc
        v = cy + fy * float(d @ y_cam) / zc
        return (u, v, zc)

    # near markers last so they overdraw far ones
    markers = sorted(world.all_markers(),
                     key=lambda m: -((m.pos_at(world.clock)[0] - drone.n) ** 2 +
                                     (m.pos_at(world.clock)[1] - drone.e) ** 2))
    for mk in markers:
        mn, me = mk.pos_at(world.clock)
        s = mk.size_m
        quiet = 0.25                       # white border fraction (each side)
        pad = s * (1.0 + 2 * quiet)        # white pad incl. quiet zone
        half = pad / 2.0
        # 4 pad corners in arena, consistent order TL,TR,BR,BL
        corners3d = [
            np.array([mn + half, me - half, mk.z]),
            np.array([mn + half, me + half, mk.z]),
            np.array([mn - half, me + half, mk.z]),
            np.array([mn - half, me - half, mk.z]),
        ]
        proj = [project(c) for c in corners3d]
        if any(p is None for p in proj):
            continue
        dst = np.array([[p[0], p[1]] for p in proj], np.float32)
        # skip if entirely off-frame
        if (dst[:, 0].max() < 0 or dst[:, 0].min() > W or
                dst[:, 1].max() < 0 or dst[:, 1].min() > H):
            continue

        side_px = 200
        marker_img = cv2.aruco.generateImageMarker(_ARUCO_DICT, int(mk.id), side_px)
        q_px = int(side_px * quiet)
        canvas = np.full((side_px + 2 * q_px, side_px + 2 * q_px), 255, np.uint8)
        canvas[q_px:q_px + side_px, q_px:q_px + side_px] = marker_img
        canvas = cv2.cvtColor(canvas, cv2.COLOR_GRAY2RGB)
        ch, cw = canvas.shape[:2]
        src = np.array([[0, 0], [cw, 0], [cw, ch], [0, ch]], np.float32)
        # match winding: a face-up marker must NOT be mirrored or ArUco can't decode it
        if _signed_area(src) * _signed_area(dst) < 0:
            dst = dst[::-1].copy()
        try:
            M = cv2.getPerspectiveTransform(src, dst)
        except cv2.error:
            continue
        warped = cv2.warpPerspective(canvas, M, (W, H), flags=cv2.INTER_LINEAR,
                                     borderMode=cv2.BORDER_CONSTANT,
                                     borderValue=(0, 0, 0))
        mask = cv2.warpPerspective(np.full((ch, cw), 255, np.uint8), M, (W, H))
        frame[mask > 0] = warped[mask > 0]
    return frame


# --------------------------------------------------------------------------- #
# Video stream
# --------------------------------------------------------------------------- #
class FakeVideoFrame:
    def __init__(self, rgb: np.ndarray):
        self._rgb = rgb

    def to_rgb(self) -> np.ndarray:
        return self._rgb


class FakeVideoStream:
    def __init__(self, drone: "FakeDroneAPI", world: FakeWorld):
        self._drone = drone
        self._world = world
        self._started = False

    def start(self) -> None:
        self._started = True

    def stop(self) -> None:
        self._started = False

    @property
    def latest_frame(self) -> Optional[FakeVideoFrame]:
        if not self._started:
            return None
        return FakeVideoFrame(render_frame(self._world, self._drone))


# --------------------------------------------------------------------------- #
# The drone — SIM SUBSET ONLY
# --------------------------------------------------------------------------- #
class FakeDroneAPI:
    """Mirrors exactly the sim's `DroneAPI` surface. Real-only methods are absent
    on purpose (see module docstring / RealLikeFakeDroneAPI)."""

    def __init__(self, world: Optional[FakeWorld] = None):
        self._world = world if world is not None else get_active_world()
        self.tag_id: int = -1
        self.connected = False
        # pose (arena truth)
        self.n = 0.0
        self.e = 0.0
        self.alt_cm = 0.0
        self.yaw_deg = 0.0
        # takeoff reference frame (frozen at takeoff)
        self._takeoff_n = 0.0
        self._takeoff_e = 0.0
        self._takeoff_yaw = 0.0
        self._dist_travelled = 0.0      # for onboard drift
        # camera
        self.pitch_down_deg = 0.0
        self.video_enabled = False
        # telemetry
        self._battery = 100
        # records (assertable in tests)
        self.barrier_mode = False
        self.avoidance_calls: List[tuple] = []
        self.manual_calls = 0

    # -- connection ------------------------------------------------------- #
    def connect(self, ip: str) -> CommandResult:
        if ip is None:
            raise TypeError("connect(ip) requires an ip")
        self.tag_id = self._world.tag_for_ip(ip)
        start = self._world.drone_starts.get(self.tag_id, (0.0, 0.0))
        self.n, self.e = float(start[0]), float(start[1])
        self.alt_cm = 0.0
        self.yaw_deg = 0.0
        self.connected = True
        self._world.register(self)
        return CommandResult(True, f"connected {ip}")

    # -- flight ----------------------------------------------------------- #
    def takeoff(self, height_cm: float = 100, blocking: bool = True,
                flags: int = TakeoffFlags.NONE) -> CommandResult:
        self.alt_cm = float(height_cm)
        self._takeoff_n, self._takeoff_e = self.n, self.e
        self._takeoff_yaw = self.yaw_deg
        self._dist_travelled = 0.0
        return CommandResult(True, "takeoff")

    def land(self, blocking: bool = True) -> CommandResult:
        self.alt_cm = 0.0
        return CommandResult(True, "land")

    def hover(self, duration_seconds: float, blocking: bool = True) -> CommandResult:
        # REQUIRES a duration (no bare hover()). Advances the clock; holds pose.
        self._world.clock += float(duration_seconds)
        return CommandResult(True, "hover")

    def move(self, direction: int, distance_cm: float,
             speed: int = VelocityLevel.ZOOM) -> CommandResult:
        d = distance_cm / 100.0
        if direction in (Direction.FORWARD, Direction.BACK,
                         Direction.RIGHT, Direction.LEFT):
            spd = self._world.velocity_mps(speed)          # ≤ max_mps (0.5), always
            if direction == Direction.FORWARD:
                self._translate(d, 0.0)
            elif direction == Direction.BACK:
                self._translate(-d, 0.0)
            elif direction == Direction.RIGHT:
                self._translate(0.0, d)
            else:
                self._translate(0.0, -d)
            self._world.clock += abs(d) / spd if spd > 0 else 0.0   # time at clamped speed
        elif direction == Direction.UP:
            self.alt_cm += distance_cm
            self._world.clock += abs(distance_cm) / self._world.climb_cmps
        elif direction == Direction.DOWN:
            self.alt_cm = max(0.0, self.alt_cm - distance_cm)
            self._world.clock += abs(distance_cm) / self._world.climb_cmps
        return CommandResult(True, "move")

    def rotate(self, angle_degrees: float) -> CommandResult:
        self.yaw_deg = (self.yaw_deg + float(angle_degrees)) % 360.0
        return CommandResult(True, "rotate")

    def move_to(self, x: float, y: float, z: float,
                speed: int = VelocityLevel.ZOOM) -> CommandResult:
        # takeoff-frame cm: x=right, y=forward, z=up. Convert to arena truth.
        fwd_m, right_m = y / 100.0, x / 100.0
        yaw = math.radians(self._takeoff_yaw)
        self.n = self._takeoff_n + fwd_m * math.cos(yaw) - right_m * math.sin(yaw)
        self.e = self._takeoff_e + fwd_m * math.sin(yaw) + right_m * math.cos(yaw)
        self.alt_cm = float(z)
        return CommandResult(True, "move_to")

    def send_manual_control(self, forward: float = 0.0, right: float = 0.0,
                            up: float = 0.0, rotate: float = 0.0) -> bool:
        self.manual_calls += 1
        f = _clamp(forward, -1.0, 1.0)
        r = _clamp(right, -1.0, 1.0)
        u = _clamp(up, -1.0, 1.0)
        rot = _clamp(rotate, -1.0, 1.0)
        # horizontal velocity with magnitude clamp to the hard cap
        vf, vr = f * self._world.max_mps, r * self._world.max_mps
        mag = math.hypot(vf, vr)
        if mag > self._world.max_mps:
            scale = self._world.max_mps / mag
            vf, vr = vf * scale, vr * scale
        dt = self._world.dt
        self._translate(vf * dt, vr * dt)
        # vertical
        self.alt_cm = max(0.0, self.alt_cm + u * self._world.climb_mps * 100.0 * dt)
        # yaw
        self.yaw_deg = (self.yaw_deg + rot * self._world.yaw_rate_dps * dt) % 360.0
        self._world.clock += dt
        return True

    def manual_fly(self, duration_sec: float, forward: float = 0.0,
                   right: float = 0.0, up: float = 0.0, rotate: float = 0.0,
                   rate_hz: float = 20, on_frame=None) -> CommandResult:
        steps = max(1, int(round(duration_sec * rate_hz)))
        for _ in range(steps):
            self.send_manual_control(forward, right, up, rotate)
            if on_frame is not None:
                on_frame()
        return CommandResult(True, "manual_fly")

    # -- internal kinematics --------------------------------------------- #
    def _translate(self, fwd_m: float, right_m: float) -> None:
        yaw = math.radians(self.yaw_deg)
        dn = fwd_m * math.cos(yaw) - right_m * math.sin(yaw)
        de = fwd_m * math.sin(yaw) + right_m * math.cos(yaw)
        self.n += dn
        self.e += de
        self._dist_travelled += math.hypot(dn, de)

    # -- telemetry -------------------------------------------------------- #
    def get_state(self):
        return {"connected": self.connected, "alt_cm": self.alt_cm}

    def get_position(self) -> Vector3:
        # onboard estimate in takeoff frame cm (x=right, y=forward, z=up) + drift
        yaw = math.radians(self._takeoff_yaw)
        dn, de = self.n - self._takeoff_n, self.e - self._takeoff_e
        fwd = dn * math.cos(yaw) + de * math.sin(yaw)
        right = -dn * math.sin(yaw) + de * math.cos(yaw)
        drift = self._dist_travelled * self._world.drift_frac
        return Vector3(x=right * 100.0 + drift * 100.0,
                       y=fwd * 100.0 + drift * 100.0, z=self.alt_cm)

    def get_orientation(self) -> Orientation:
        return Orientation(yaw=self.yaw_deg, pitch=0.0, roll=0.0)

    def get_altitude(self) -> float:
        return float(self.alt_cm)

    def get_battery(self) -> int:
        return int(self._battery)

    # -- obstacles -------------------------------------------------------- #
    def _blocked(self, body_fwd: float, body_right: float) -> bool:
        """Is something within barrier range in this body direction?"""
        if self.alt_cm < self._world.min_sense_alt_cm:
            return False
        yaw = math.radians(self.yaw_deg)
        # unit body vector -> arena
        dn = body_fwd * math.cos(yaw) - body_right * math.sin(yaw)
        de = body_fwd * math.sin(yaw) + body_right * math.cos(yaw)
        rng = self._world.barrier_range_m
        steps = 8
        for i in range(1, steps + 1):
            dist = rng * i / steps
            pn, pe = self.n + dn * dist, self.e + de * dist
            if self._world.point_in_crate(pn, pe):
                return True
        # other drones
        for tag, other in self._world._drones.items():
            if tag == self.tag_id or other.alt_cm < self._world.min_sense_alt_cm:
                continue
            rel_n, rel_e = other.n - self.n, other.e - self.e
            d = math.hypot(rel_n, rel_e)
            if d > rng or d < 1e-9:
                continue
            # projection onto body direction must be dominant & positive
            proj = (rel_n * dn + rel_e * de)
            if proj > 0 and proj >= 0.7 * d:
                return True
        return False

    def get_obstacles(self, drone_id: int = 0) -> Obstacles:
        down = self.alt_cm <= self._world.down_range_cm
        if self.alt_cm < self._world.min_sense_alt_cm:
            return Obstacles(False, False, False, False, down)
        return Obstacles(
            forward=self._blocked(1.0, 0.0),
            back=self._blocked(-1.0, 0.0),
            left=self._blocked(0.0, -1.0),
            right=self._blocked(0.0, 1.0),
            down=down,
        )

    def any_obstacle(self) -> bool:
        return self.get_obstacles().any

    def get_drone_status(self) -> int:
        o = self.get_obstacles()
        bits = 0
        bits |= int(o.forward) << 0
        bits |= int(o.back) << 1
        bits |= int(o.left) << 2
        bits |= int(o.right) << 3
        bits |= int(o.down) << 4
        return bits

    # -- avoidance helpers (record only) --------------------------------- #
    def set_barrier_mode(self, enabled: bool) -> CommandResult:
        self.barrier_mode = bool(enabled)
        return CommandResult(True, "barrier_mode")

    def set_avoidance_direction(self, direction: int, distance_cm: float = 0,
                                barrier_mask: int = BarrierMask.ALL) -> CommandResult:
        self.avoidance_calls.append((direction, distance_cm, barrier_mask))
        return CommandResult(True, "avoidance")

    # -- camera / video --------------------------------------------------- #
    def set_camera_angle(self, mode: int, angle: float = 0) -> CommandResult:
        if mode == CameraPitchMode.DOWN_ABSOLUTE:
            self.pitch_down_deg = float(angle)
        elif mode == CameraPitchMode.UP_ABSOLUTE:
            self.pitch_down_deg = -float(angle)
        elif mode == CameraPitchMode.DOWN_RELATIVE:
            self.pitch_down_deg += float(angle)
        elif mode == CameraPitchMode.UP_RELATIVE:
            self.pitch_down_deg -= float(angle)
        self.pitch_down_deg = _clamp(self.pitch_down_deg, -90.0, 90.0)
        return CommandResult(True, "camera_angle")

    def set_video_stream(self, enabled: bool) -> CommandResult:
        self.video_enabled = bool(enabled)
        return CommandResult(True, "video")

    def create_video_stream(self) -> FakeVideoStream:
        return FakeVideoStream(self, self._world)


class RealLikeFakeDroneAPI(FakeDroneAPI):
    """A fake that ALSO exposes the real-SDK-only methods, to exercise the
    `runtime/sdk_compat.py` *real* path (the hardware branch of the hasattr guards)."""

    def __init__(self, world: Optional[FakeWorld] = None):
        super().__init__(world)
        self.real_calls: List[str] = []

    def _rec(self, name: str) -> CommandResult:
        self.real_calls.append(name)
        return CommandResult(True, name)

    def set_app_mode(self, mode: int) -> CommandResult:
        return self._rec(f"set_app_mode:{mode}")

    def send_app_heartbeat(self) -> CommandResult:
        return self._rec("send_app_heartbeat")

    def set_velocity_level(self, level: int) -> CommandResult:
        return self._rec(f"set_velocity_level:{level}")

    def set_yaw_rate_level(self, level: int) -> CommandResult:
        return self._rec(f"set_yaw_rate_level:{level}")

    def stop_manual_control(self) -> CommandResult:
        return self._rec("stop_manual_control")

    def arm(self) -> CommandResult:
        return self._rec("arm")

    def disarm(self) -> CommandResult:
        return self._rec("disarm")

    def disconnect(self) -> CommandResult:
        self.connected = False
        return self._rec("disconnect")

    def enable_battery_failsafe(self, *a, **k) -> CommandResult:
        return self._rec("enable_battery_failsafe")

    def get_velocity(self) -> Vector3:
        self.real_calls.append("get_velocity")
        return Vector3(0.0, 0.0, 0.0)

    def get_drone_id(self) -> int:
        return self.tag_id


# --------------------------------------------------------------------------- #
# Discovery (pyhulax.discovery.Dola surface)
# --------------------------------------------------------------------------- #
class FakeDola:
    def __init__(self, listen_ip: str = "0.0.0.0", world: Optional[FakeWorld] = None):
        self._world = world if world is not None else get_active_world()

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def get_all_ips(self, listen_seconds: float = 0) -> Dict[int, str]:
        return dict(self._world.ip_map)

    def get_ip_by_plane_id(self, plane_id: int, listen_seconds: float = 0):
        return self._world.ip_map.get(plane_id)

    def get_ips_by_plane_ids(self, plane_ids, listen_seconds: float = 0):
        return {pid: self._world.ip_map.get(pid) for pid in plane_ids}


# --------------------------------------------------------------------------- #
# Active-world registry (so a bare `DroneAPI()` binds to the test's world)
# --------------------------------------------------------------------------- #
_ACTIVE_WORLD: Optional[FakeWorld] = None


def set_active_world(world: Optional[FakeWorld]) -> None:
    global _ACTIVE_WORLD
    _ACTIVE_WORLD = world


def get_active_world() -> FakeWorld:
    if _ACTIVE_WORLD is None:
        raise RuntimeError("no active FakeWorld; use the `world` fixture / set_active_world")
    return _ACTIVE_WORLD


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


# Public alias mirroring the real module (`from pyhulax import DroneAPI`)
DroneAPI = FakeDroneAPI
