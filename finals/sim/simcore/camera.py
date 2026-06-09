"""Per-drone tiltable camera rendering — SIM THREAD ONLY.

Simulator INTERNAL — mission code must never import simcore.

Intrinsics come from config.camera (resolution, horizontal FOV, near/far);
view geometry (drone pose composed with pitch) lives in frames.py — the one
place those compose. Shadows OFF and high ambient light so ArUco markers
decode cleanly.

RENDERER SELECTION IS GUARDED HERE (resolve_renderer), keyed on the live
PyBullet CONNECTION MODE — not on a value a caller hands us. Under p.GUI a
hardware getCameraImage races the GUI's own render thread and HANGS the sim
thread on WSLg, so GUI mode ALWAYS uses the software TinyRenderer; headless
DIRECT+EGL keeps the hardware OpenGL renderer. Every camera-render entry
point (referee, camera windows, top-down) must go through resolve_renderer
so no path can issue a hardware render while GUI is connected.
"""

import math

import numpy as np
import pybullet as p

from . import frames


def resolve_renderer(client, requested):
    """Pick the safe getCameraImage renderer for the current connection.

    p.GUI  -> ER_TINY_RENDERER (software): hardware getCameraImage hangs the
              GUI render thread on WSLg (the part-2 freeze).
    other  -> `requested` unchanged: the headless DIRECT+EGL hardware
              renderer (or the Tiny fallback the registry already chose when
              EGL was unavailable).
    """
    if p.getConnectionInfo(client).get("connectionMethod") == p.GUI:
        return p.ER_TINY_RENDERER
    return requested

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


def render_rgb(client, cfg, drone, renderer, width=None,
               height=None) -> np.ndarray:
    """One (H, W, 3) uint8 RGB frame from this drone's camera. SIM THREAD.
    width/height override the output resolution at the SAME FOV/pose (for
    crisp high-res insets); they default to the configured AI-mode size."""
    cam = cfg.camera
    w = int(width if width is not None else cam.width)
    h = int(height if height is not None else cam.height)
    eye, target, up = frames.camera_eye_target_up(
        cfg, drone.pos, drone.yaw, drone.camera_pitch_deg)
    view = p.computeViewMatrix(eye, target, up)
    # vertical FOV preserved from the configured camera; aspect from the
    # requested dims so a higher-res inset shows the SAME view, just crisper.
    fov_v = 2.0 * math.degrees(math.atan(
        math.tan(math.radians(cam.h_fov_deg) / 2.0) * cam.height / cam.width))
    proj = p.computeProjectionMatrixFOV(
        fov=fov_v, aspect=w / h, nearVal=cam.near_m, farVal=cam.far_m)
    img = p.getCameraImage(
        w, h, viewMatrix=view, projectionMatrix=proj,
        renderer=resolve_renderer(client, renderer),  # GUI -> software
        flags=p.ER_NO_SEGMENTATION_MASK, physicsClientId=client,
        **_LIGHT_KWARGS)
    rgba = np.asarray(img[2], dtype=np.uint8).reshape(h, w, 4)
    return rgba[:, :, :3].copy()
