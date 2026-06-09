"""Observer-side camera feed: ArUco-outlined live windows + scan-image saves.

Simulator INTERNAL — for the human watching the sim. Detection here is the
sim's OWN cv2.aruco (the same the referee uses) drawn onto a copy of the
frame for visual proof; it is NOT a sensor the mission reads, and nothing
here touches the public pyhulax surface. Mission detection runs separately
on the same frames.

annotate_markers() is a pure function (headless-testable). MarkerCameraWindow
wraps a public VideoStream and shows annotated frames with cv2.imshow.
"""

import threading
import time

import cv2
import numpy as np

from . import aruco_assets
from .log import get_logger


# Acquisition-UI colours (BGR): a marker the camera DETECTS (decodable) vs
# one the referee has ACQUIRED (scored). Consistent with the cockpit key.
_DETECTED = (0, 255, 255)   # yellow
_ACQUIRED = (60, 220, 80)   # green
_FLASH = (255, 255, 255)    # brief highlight on the first bank


def detect_markers(cfg, bgr: np.ndarray):
    """The shared REAL cv2.aruco detection on a frame -> [(id, pts(4,2))].

    This is genuine detection on the rendered pixels (the same path the
    referee judges with) — NOT a box drawn from ground-truth marker poses.
    """
    detector = cv2.aruco.ArucoDetector(
        aruco_assets.get_dictionary(cfg.aruco.dictionary),
        cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(
        cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
    if ids is None:
        return []
    return [(int(mid), quad[0]) for mid, quad in zip(ids.flatten(), corners)]


def annotate_markers(cfg, bgr: np.ndarray):
    """Return a copy of a BGR frame with any detected ArUco markers OUTLINED
    and id-labelled. Returns (annotated_bgr, [(id, corners(4,2)), ...])."""
    out = bgr.copy()
    found = detect_markers(cfg, bgr)
    for mid, pts in found:
        cv2.polylines(out, [pts.astype(np.int32)], True, (0, 255, 0), 2)
        cx, cy = int(pts[:, 0].mean()), int(pts[:, 1].min()) - 8
        cv2.putText(out, f"id {mid}", (cx - 20, max(cy, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return out, found


def acquisition_overlay(cfg, bgr: np.ndarray, banked_ids=(), flash_ids=()):
    """Deliberate target-acquisition UI on a camera frame, from REAL
    cv2.aruco detection: a YELLOW 'DETECTED id N' box the moment a marker is
    decodable, turning GREEN 'ACQUIRED id N' once the referee has banked it
    (a brief white highlight while id is in flash_ids — the first bank).
    Returns (overlaid_bgr, found)."""
    out = bgr.copy()
    found = detect_markers(cfg, bgr)
    banked, flash = set(banked_ids), set(flash_ids)
    for mid, pts in found:
        acquired = mid in banked
        color = _ACQUIRED if acquired else _DETECTED
        ipts = pts.astype(np.int32)
        flashing = mid in flash
        cv2.polylines(out, [ipts], True, color, 3 if flashing else 2)
        label = f"{'ACQUIRED' if acquired else 'DETECTED'} id {mid}"
        x0, y0 = int(ipts[:, 0].min()), int(ipts[:, 1].min())
        cv2.putText(out, label, (x0, max(y0 - 6, 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
        if flashing:  # first-bank flash: a bright box around the marker
            cv2.rectangle(out, (int(ipts[:, 0].min()) - 4,
                               int(ipts[:, 1].min()) - 4),
                          (int(ipts[:, 0].max()) + 4,
                           int(ipts[:, 1].max()) + 4), _FLASH, 2)
    return out, found


def box_marker(cfg, rgb: np.ndarray, corners) -> np.ndarray:
    """BGR copy of an RGB frame with ONE marker's quad boxed in red + a
    'SCANNED id N' tag — used for the on-bank scan image."""
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    pts = np.asarray(corners, dtype=np.int32).reshape(-1, 2)
    cv2.polylines(bgr, [pts], True, (0, 0, 255), 3)
    return bgr


class MarkerCameraWindow:
    """Live cv2.imshow window of one drone's stream with ArUco overlays.

    Reads frames through the PUBLIC VideoStream API; the overlay is
    observer-side. Needs a display (WSLg); gated by viz.show_camera_windows.
    """

    def __init__(self, cfg, stream, window_name: str):
        self._cfg = cfg
        self._stream = stream
        self._window = window_name
        self._log = get_logger("camfeed", cfg)
        self._stop = threading.Event()
        self._thread = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="hula-camfeed",
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)

    def _loop(self) -> None:
        while not self._stop.is_set():
            frame = self._stream.latest_frame
            if frame is not None:
                annotated, _found = annotate_markers(self._cfg, frame.to_bgr())
                cv2.imshow(self._window, annotated)
                cv2.waitKey(1)
            time.sleep(1.0 / 30.0)
        cv2.destroyWindow(self._window)
