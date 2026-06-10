"""Rover detection seam — mirrors `reference/provided_code/rover_detection_example.py`.

Detect-wide / identify-narrow: Stage 1 (`RoverDetector.detect`) proposes rover-shaped
candidates from afar (no id); Stage 2 (`confirm_with_aruco`, in `aruco.py`) decodes the
real id up close — only a confirmed id scores. `ClassicalRoverDetector` is the PRIMARY,
training-free finder (contour/contrast/motion); an optional YOLO subclass plugs into the
same seam without touching the flow. Keeps detection behind ONE interface so ArUco ↔ YOLO
is a swap (docs/ARCHITECTURE.md).
"""

from __future__ import annotations

import abc
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class Detection:
    """One detected target. `marker_id` is None for a Stage-1 candidate (a rover-shaped
    blob, no identity yet) and set once ArUco confirms it."""
    bbox: Tuple[int, int, int, int]      # (x, y, w, h) in frame pixels
    conf: float                          # detector confidence 0..1
    marker_id: Optional[int] = None      # decoded id (None until confirmed)
    source: str = "yolo"                 # "yolo"/"classical" (stage 1) | "aruco" (stage 2)

    @property
    def confirmed(self) -> bool:
        return self.marker_id is not None


# --------------------------------------------------------------------------- #
# Stage 1 — the finder seam
# --------------------------------------------------------------------------- #
class RoverDetector(abc.ABC):
    @abc.abstractmethod
    def detect(self, image_bgr: np.ndarray) -> List[Detection]:
        ...


class PlaceholderRoverDetector(RoverDetector):
    """Trivial stand-in (one centre box) so the pipeline runs without a model —
    mirrors the provided example. Swap for a real model behind the same seam."""

    def __init__(self, conf: float = 0.5, box_frac: float = 0.34):
        self._conf = float(conf)
        self._box_frac = float(box_frac)

    def detect(self, image_bgr: np.ndarray) -> List[Detection]:
        h, w = image_bgr.shape[:2]
        bw, bh = int(w * self._box_frac), int(h * self._box_frac)
        return [Detection(bbox=((w - bw) // 2, (h - bh) // 2, bw, bh),
                          conf=self._conf, marker_id=None, source="yolo")]


class ClassicalRoverDetector(RoverDetector):
    """PRIMARY finder — training-free. Finds high-contrast structured blobs (the rover's
    marked top stands out from the flat floor) via thresholding + contours, with an
    optional motion gate (frame differencing) for moving targets."""

    def __init__(self, min_area_px: int = 400, bright_thresh: int = 210,
                 dark_thresh: int = 70, close_ksize: int = 15,
                 use_motion: bool = False, motion_thresh: int = 18):
        self.min_area_px = int(min_area_px)
        self.bright_thresh = int(bright_thresh)
        self.dark_thresh = int(dark_thresh)
        self.close_ksize = int(close_ksize)
        self.use_motion = bool(use_motion)
        self.motion_thresh = int(motion_thresh)
        self._prev_gray: Optional[np.ndarray] = None

    def detect(self, image_bgr: np.ndarray) -> List[Detection]:
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        # a marked target is much brighter (white pad) or darker (marker) than the floor
        mask = ((gray >= self.bright_thresh) | (gray <= self.dark_thresh)).astype(np.uint8) * 255
        if self.use_motion and self._prev_gray is not None:
            motion = (cv2.absdiff(gray, self._prev_gray) >= self.motion_thresh).astype(np.uint8) * 255
            motion = cv2.dilate(motion, np.ones((self.close_ksize, self.close_ksize), np.uint8))
            mask = cv2.bitwise_and(mask, motion)
        self._prev_gray = gray
        k = np.ones((self.close_ksize, self.close_ksize), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out: List[Detection] = []
        frame_area = float(image_bgr.shape[0] * image_bgr.shape[1])
        for c in contours:
            x, y, w, h = cv2.boundingRect(c)
            area = w * h
            if area < self.min_area_px:
                continue
            out.append(Detection(bbox=(int(x), int(y), int(w), int(h)),
                                 conf=min(1.0, area / frame_area * 6.0),
                                 marker_id=None, source="classical"))
        out.sort(key=lambda d: -d.conf)
        return out


# --------------------------------------------------------------------------- #
# the two-stage flow
# --------------------------------------------------------------------------- #
@dataclass
class ScanResult:
    candidates: List[Detection] = field(default_factory=list)   # stage 1
    confirmed: List[Detection] = field(default_factory=list)    # stage 2 (ArUco)
    frame_shape: Optional[Tuple[int, int, int]] = None

    @property
    def confirmed_ids(self) -> List[int]:
        return [d.marker_id for d in self.confirmed if d.marker_id is not None]


def frame_bgr(frame) -> np.ndarray:
    """One BGR ndarray for ALL cv2/ArUco/evidence use. Real SDK frames expose a native
    BGR `.image`; the sim's expose `.to_rgb()` (convert). Routing everything through this is
    what keeps real proof-snapshots from coming out colour-swapped."""
    img = getattr(frame, "image", None)
    if img is not None:
        return img                                          # real SDK: already BGR
    return cv2.cvtColor(frame.to_rgb(), cv2.COLOR_RGB2BGR)   # sim: RGB -> BGR


def grab_frame(stream, timeout_s: float = 2.0,
               sleep: Callable[[float], None] = time.sleep) -> Optional[np.ndarray]:
    """Pull the latest frame from a public pyhulax VideoStream as a BGR ndarray."""
    deadline = time.time() + timeout_s
    while True:
        frame = stream.latest_frame
        if frame is not None:
            return frame_bgr(frame)
        if time.time() >= deadline:
            return None
        sleep(0.02)


def two_stage_scan(stream, detector: RoverDetector,
                   approach: Optional[Callable[[List[Detection]], None]] = None,
                   dictionary: str = "DICT_6X6_250",
                   sleep: Callable[[float], None] = time.sleep) -> ScanResult:
    """One pass: grab → Stage-1 detect → (optional) approach + re-grab → Stage-2 ArUco."""
    from mission.perception.aruco import confirm_with_aruco       # lazy: avoid import cycle

    frame = grab_frame(stream, sleep=sleep)
    if frame is None:
        return ScanResult()
    candidates = detector.detect(frame)
    if approach is not None and candidates:
        approach(candidates)
        fresh = grab_frame(stream, sleep=sleep)
        if fresh is not None:
            frame = fresh
    confirmed = confirm_with_aruco(frame, dictionary)
    return ScanResult(candidates=candidates, confirmed=confirmed,
                      frame_shape=tuple(frame.shape))
