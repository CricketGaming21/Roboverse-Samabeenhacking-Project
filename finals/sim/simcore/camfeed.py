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


def annotate_markers(cfg, bgr: np.ndarray):
    """Return a copy of a BGR frame with any detected ArUco markers OUTLINED
    and id-labelled. Returns (annotated_bgr, [(id, corners(4,2)), ...])."""
    out = bgr.copy()
    detector = cv2.aruco.ArucoDetector(
        aruco_assets.get_dictionary(cfg.aruco.dictionary),
        cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(
        cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
    found = []
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(out, corners, ids)
        for mid, quad in zip(ids.flatten(), corners):
            pts = quad[0]
            found.append((int(mid), pts))
            cx, cy = int(pts[:, 0].mean()), int(pts[:, 1].min()) - 8
            cv2.putText(out, f"id {int(mid)}", (cx - 20, max(cy, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
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
