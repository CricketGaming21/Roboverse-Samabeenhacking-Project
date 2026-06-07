"""Video surface of the (simulated) pyhulax SDK — HULA_SIM_BUILD_PLAN.md §4.5.

VideoFrame.to_rgb() must return a real rendered RGB image of what the drone's
tiltable camera sees, so cv2.aruco and any mission-side detector work on it.
Implemented in the camera phase; Phase 0 ships signatures only.
"""

from typing import Optional

import numpy as np


class VideoFrame:
    """One rendered camera frame."""

    @property
    def width(self) -> int:
        raise NotImplementedError

    @property
    def height(self) -> int:
        raise NotImplementedError

    def to_rgb(self) -> np.ndarray:
        """(H, W, 3) uint8, RGB — what the mission's detector consumes."""
        raise NotImplementedError

    def to_bgr(self) -> np.ndarray:
        """(H, W, 3) uint8, BGR."""
        raise NotImplementedError


class VideoStream:
    """Per-drone video stream; latest_frame holds the most recent render."""

    def start(self) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    @property
    def latest_frame(self) -> Optional[VideoFrame]:
        """Most recent rendered frame for this drone (None until first frame)."""
        raise NotImplementedError

    @property
    def fps(self) -> float:
        raise NotImplementedError


class VideoDisplay:
    """Optional helper that shows stream frames with cv2.imshow."""

    def __init__(self, stream: VideoStream, window_name: str = "hula",
                 show_fps: bool = True):
        raise NotImplementedError

    def start(self) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError
