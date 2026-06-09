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
# one the referee has ACQUIRED (scored). HIGH-CONTRAST and bright so the box
# is unmissable at any record size.
_DETECTED = (0, 255, 255)   # bright yellow
_ACQUIRED = (0, 255, 0)     # bright green
_FLASH = (255, 255, 255)    # brief highlight on the first bank
_BOX_MIN_THICK = 3          # never thinner than this, even on small frames


def box_thickness(frame_w: int) -> int:
    """Outline thickness scaled to the frame width — bold at any resolution
    (>= _BOX_MIN_THICK)."""
    return max(_BOX_MIN_THICK, int(round(frame_w / 220.0)))


def draw_acquisition_box(img, marker_id, pts, acquired, flash=False):
    """Draw ONE high-visibility detect/acquire box on a BGR frame: a thick
    bright outline (yellow DETECTED / green ACQUIRED) + a large id label on a
    semi-opaque dark pill so it is legible against any background."""
    h, w = img.shape[:2]
    color = _ACQUIRED if acquired else _DETECTED
    thick = box_thickness(w) + (2 if flash else 0)
    ipts = pts.astype(np.int32)
    cv2.polylines(img, [ipts], True, color, thick, cv2.LINE_AA)
    if flash:  # first-bank highlight: a bright ring around the marker
        cv2.rectangle(img, (int(ipts[:, 0].min()) - 5, int(ipts[:, 1].min()) - 5),
                      (int(ipts[:, 0].max()) + 5, int(ipts[:, 1].max()) + 5),
                      _FLASH, 2, cv2.LINE_AA)
    # label on a semi-opaque pill, just above the marker
    label = f"{'ACQUIRED' if acquired else 'DETECTED'} id {marker_id}"
    fs = max(0.5, w / 1100.0)
    (tw, th), base = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, fs, 2)
    lx = int(max(2, min(ipts[:, 0].min(), w - tw - 6)))
    ly = int(ipts[:, 1].min()) - 8
    if ly - th - 4 < 0:                 # no room above -> below the marker
        ly = int(ipts[:, 1].max()) + th + 10
    x0, y0 = lx - 4, ly - th - 4
    x1, y1 = lx + tw + 4, ly + base + 2
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(w, x1), min(h, y1)
    roi = img[y0:y1, x0:x1]
    if roi.size:                        # semi-opaque dark pill behind the text
        dark = np.zeros_like(roi)
        cv2.addWeighted(dark, 0.55, roi, 0.45, 0.0, roi)
    cv2.putText(img, label, (lx, ly), cv2.FONT_HERSHEY_SIMPLEX, fs, color, 2,
                cv2.LINE_AA)


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
        draw_acquisition_box(out, mid, pts, acquired=mid in banked,
                             flash=mid in flash)
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
