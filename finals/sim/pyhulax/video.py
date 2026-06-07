"""Video surface of the (simulated) pyhulax SDK — HULA_SIM_BUILD_PLAN.md §4.5.

VideoFrame.to_rgb() returns a real rendered RGB image of what the drone's
tiltable camera sees, so cv2.aruco and any mission-side detector work on it.

No pybullet calls happen in this module: each VideoStream runs a small puller
thread that requests frames through the sim registry's sim-thread queue at
~config.camera.fps (wall clock), and only while set_video_stream(True) is on.
"""

import threading
import time
from collections import deque
from typing import Optional

import numpy as np

from .exceptions import PyhulaxError


class VideoFrame:
    """One rendered camera frame (constructed internally by VideoStream)."""

    def __init__(self, rgb: np.ndarray):
        self._rgb = rgb  # (H, W, 3) uint8, RGB

    @property
    def width(self) -> int:
        return self._rgb.shape[1]

    @property
    def height(self) -> int:
        return self._rgb.shape[0]

    def to_rgb(self) -> np.ndarray:
        """(H, W, 3) uint8, RGB — what the mission's detector consumes."""
        return self._rgb.copy()

    def to_bgr(self) -> np.ndarray:
        """(H, W, 3) uint8, BGR."""
        return self._rgb[:, :, ::-1].copy()


class VideoStream:
    """Per-drone video stream; latest_frame holds the most recent render.

    Obtain via DroneAPI.create_video_stream(); frames flow once BOTH
    set_video_stream(True) has been sent and start() has been called
    (either order works).
    """

    def __init__(self, registry=None, drone=None):
        self._reg = registry
        self._drone = drone
        self._latest: Optional[VideoFrame] = None
        self._stamps = deque(maxlen=30)
        self._stop = threading.Event()
        self._thread = None

    def start(self) -> None:
        if self._reg is None or self._drone is None:
            raise PyhulaxError("stream is not bound to a drone — obtain it "
                               "via DroneAPI.create_video_stream()")
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._pull_loop,
            name=f"hula-video-{self._drone.index}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)

    @property
    def latest_frame(self) -> Optional[VideoFrame]:
        """Most recent rendered frame for this drone (None until first)."""
        return self._latest

    @property
    def fps(self) -> float:
        """Measured frame rate over the recent window (0.0 until flowing)."""
        stamps = list(self._stamps)
        if len(stamps) < 2:
            return 0.0
        span = stamps[-1] - stamps[0]
        return (len(stamps) - 1) / span if span > 0 else 0.0

    def _pull_loop(self) -> None:
        period = 1.0 / float(self._reg.config.camera.fps)
        while not self._stop.is_set():
            if not self._reg.is_alive():
                break
            # Atomic bool read; written only on the sim thread.
            if not self._drone.video_enabled:
                time.sleep(0.02)
                continue
            t0 = time.perf_counter()
            try:
                rgb = self._reg.render_camera(self._drone)
            except (RuntimeError, TimeoutError):
                break  # sim shut down mid-render
            self._latest = VideoFrame(rgb)
            self._stamps.append(time.time())
            remaining = period - (time.perf_counter() - t0)
            if remaining > 0:
                self._stop.wait(remaining)


class VideoDisplay:
    """Optional helper that shows stream frames with cv2.imshow.

    Needs a display (WSLg works); never required by the headless sim.
    """

    def __init__(self, stream: VideoStream, window_name: str = "hula",
                 show_fps: bool = True):
        self._stream = stream
        self._window = window_name
        self._show_fps = show_fps
        self._stop = threading.Event()
        self._thread = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._show_loop,
                                        name="hula-display", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)

    def _show_loop(self) -> None:
        import cv2  # lazy: only a started display needs a GUI
        while not self._stop.is_set():
            frame = self._stream.latest_frame
            if frame is not None:
                img = frame.to_bgr()
                if self._show_fps:
                    cv2.putText(img, f"{self._stream.fps:.1f} fps", (8, 24),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow(self._window, img)
                cv2.waitKey(1)
            time.sleep(1.0 / 30.0)
        cv2.destroyWindow(self._window)
