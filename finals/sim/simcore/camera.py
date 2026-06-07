"""Per-drone tiltable camera rendering — SIM THREAD ONLY.

Simulator INTERNAL — mission code must never import simcore.

Intrinsics come from config.camera (resolution, horizontal FOV, near/far);
view geometry (drone pose composed with pitch) lives in frames.py — the one
place those compose. Rendering uses the registry's renderer (EGL hardware GL
when loaded, TinyRenderer fallback) with shadows OFF and high ambient light
so ArUco markers decode cleanly.
"""

import math

import numpy as np
import pybullet as p

from . import frames

# Even, diffuse lighting: no shadows / no speculars across the marker faces.
_LIGHT_KWARGS = dict(
    shadow=0,
    lightDirection=[0.4, 0.3, 1.0],
    lightAmbientCoeff=0.8,
    lightDiffuseCoeff=0.5,
    lightSpecularCoeff=0.0,
)


def projection_matrix(cfg):
    """Vertical-FOV projection derived from the configured horizontal FOV."""
    cam = cfg.camera
    fov_v = 2.0 * math.degrees(math.atan(
        math.tan(math.radians(cam.h_fov_deg) / 2.0) * cam.height / cam.width))
    return p.computeProjectionMatrixFOV(
        fov=fov_v, aspect=cam.width / cam.height,
        nearVal=cam.near_m, farVal=cam.far_m)


def render_rgb(client, cfg, drone, renderer) -> np.ndarray:
    """One (H, W, 3) uint8 RGB frame from this drone's camera. SIM THREAD."""
    cam = cfg.camera
    eye, target, up = frames.camera_eye_target_up(
        cfg, drone.pos, drone.yaw, drone.camera_pitch_deg)
    view = p.computeViewMatrix(eye, target, up)
    img = p.getCameraImage(
        cam.width, cam.height, viewMatrix=view,
        projectionMatrix=projection_matrix(cfg), renderer=renderer,
        flags=p.ER_NO_SEGMENTATION_MASK, physicsClientId=client,
        **_LIGHT_KWARGS)
    rgba = np.asarray(img[2], dtype=np.uint8).reshape(cam.height, cam.width, 4)
    return rgba[:, :, :3].copy()
