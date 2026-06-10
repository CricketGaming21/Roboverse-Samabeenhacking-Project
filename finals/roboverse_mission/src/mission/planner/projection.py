"""Ground-plane projection — pixel ↔ arena (x_m, y_m) on the flat floor.

Pure pinhole geometry, **no depth**. A rover's approximate (x,y) comes from the
drone pose + altitude + camera tilt + intrinsics. This implements the SAME camera
model the fake renderer uses (docs camera convention), so `arena_to_pixel` and
`pixel_to_arena` are exact inverses for floor points.

Camera convention: arena (north=x, east=y, up=z); heading `yaw_deg` (CCW from north);
`cam_pitch_deg` is the DOWN pitch (0 = looking forward at the horizon, 90 = straight
down). Right-handed (non-mirrored) image axes.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np

Point = Tuple[float, float]


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int = 640
    height: int = 480
    h_fov_deg: float = 71.0

    @property
    def fx(self) -> float:
        return (self.width / 2.0) / math.tan(math.radians(self.h_fov_deg) / 2.0)

    @property
    def fy(self) -> float:
        return self.fx          # square pixels (matches the fake)

    @property
    def cx(self) -> float:
        return self.width / 2.0

    @property
    def cy(self) -> float:
        return self.height / 2.0


def _basis(yaw_rad: float, pitch_down_rad: float):
    """(z_cam optical, x_cam image-right, y_cam image-down) in arena (n,e,up)."""
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    cp, sp = math.cos(pitch_down_rad), math.sin(pitch_down_rad)
    f_h = np.array([cy, sy, 0.0])
    x_cam = np.array([-sy, cy, 0.0])
    z_cam = f_h * cp + np.array([0.0, 0.0, -1.0]) * sp
    y_cam = np.cross(z_cam, x_cam)
    return z_cam, x_cam, y_cam


def arena_to_pixel(point_xy: Point, drone_xy: Point, yaw_deg: float, alt_m: float,
                   cam_pitch_deg: float, intrinsics: CameraIntrinsics,
                   z_m: float = 0.0):
    """Project a floor (or height-z) arena point to a pixel (u, v). Returns None if
    the point is behind the camera."""
    cam = np.array([drone_xy[0], drone_xy[1], alt_m])
    z_cam, x_cam, y_cam = _basis(math.radians(yaw_deg), math.radians(cam_pitch_deg))
    d = np.array([point_xy[0], point_xy[1], z_m]) - cam
    zc = float(d @ z_cam)
    if zc <= 1e-6:
        return None
    u = intrinsics.cx + intrinsics.fx * float(d @ x_cam) / zc
    v = intrinsics.cy + intrinsics.fy * float(d @ y_cam) / zc
    return (u, v)


def pixel_to_arena(u: float, v: float, drone_xy: Point, yaw_deg: float, alt_m: float,
                   cam_pitch_deg: float, intrinsics: CameraIntrinsics,
                   floor_z_m: float = 0.0) -> Point:
    """Back-project pixel (u,v) onto the floor plane z=floor_z_m → arena (x_m, y_m).
    Raises ValueError if the ray does not hit the floor (looking up / at horizon)."""
    cam = np.array([drone_xy[0], drone_xy[1], alt_m])
    z_cam, x_cam, y_cam = _basis(math.radians(yaw_deg), math.radians(cam_pitch_deg))
    ray = ((u - intrinsics.cx) / intrinsics.fx) * x_cam \
        + ((v - intrinsics.cy) / intrinsics.fy) * y_cam + z_cam
    if abs(ray[2]) < 1e-9 or (cam[2] - floor_z_m) * ray[2] >= 0:
        raise ValueError("ray does not intersect the floor plane (camera not looking down)")
    t = (floor_z_m - cam[2]) / ray[2]
    pt = cam + t * ray
    return (float(pt[0]), float(pt[1]))
