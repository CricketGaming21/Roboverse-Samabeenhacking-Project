"""ArUco confirmation — the identity stage that actually scores (HARD invariant #5).

Real `cv2.aruco` on the same frames the referee scores on. Multi-marker per frame.
**Pads = ids 10–14; ANY other decoded id = a rover** — opponent rovers carry DIFFERENT
ids from the autonomous ones, so we NEVER hard-filter to 20–24. De-dup is by id, post-read.
"""

from __future__ import annotations

from typing import FrozenSet, List, Tuple

import cv2
import numpy as np

from mission.perception.detector import Detection

ARUCO_DICT_DEFAULT = "DICT_6X6_250"
PAD_IDS: FrozenSet[int] = frozenset({10, 11, 12, 13, 14})


def is_pad_id(marker_id: int) -> bool:
    return marker_id in PAD_IDS


def is_rover_id(marker_id: int) -> bool:
    """Any decoded id that is NOT a pad is a rover (opponent rovers differ from 20–24)."""
    return marker_id not in PAD_IDS


def _detector(dictionary: str):
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    return cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())


def confirm_with_aruco(frame_rgb: np.ndarray, dictionary: str = ARUCO_DICT_DEFAULT,
                       min_marker_px: int = 0) -> List[Detection]:
    """Decode every visible marker → a confirmed `Detection` (id + bbox) per marker.
    A marker whose larger pixel side is below `min_marker_px` is dropped (too far to
    trust)."""
    gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
    corners, ids, _ = _detector(dictionary).detectMarkers(gray)
    out: List[Detection] = []
    if ids is None:
        return out
    for mid, quad in zip(ids.flatten(), corners):
        x, y, w, h = cv2.boundingRect(quad[0].astype(np.int32))
        if min_marker_px and max(w, h) < min_marker_px:
            continue
        out.append(Detection(bbox=(int(x), int(y), int(w), int(h)), conf=1.0,
                             marker_id=int(mid), source="aruco"))
    return out


def split_pads_rovers(detections: List[Detection]) -> Tuple[List[Detection], List[Detection]]:
    """Partition confirmed detections into (pads, rovers) by id."""
    pads = [d for d in detections if d.marker_id is not None and is_pad_id(d.marker_id)]
    rovers = [d for d in detections if d.marker_id is not None and is_rover_id(d.marker_id)]
    return pads, rovers
