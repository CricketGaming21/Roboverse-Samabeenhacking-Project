"""ArUco confirmation — the identity stage that actually scores (HARD invariant #5).

Real `cv2.aruco` on the same frames the referee scores on. Multi-marker per frame.
**ALLOW-LIST by config `aruco.rover_ids`** — bank only those ids, distinct/de-duped. (No
pad deny-list: a real rover is id 11, which sat inside the old 10–14 pad range and was wrongly
excluded; real pads carry no markers anyway — R1.) The detector is dictionary-agnostic.
"""

from __future__ import annotations

from typing import Iterable, List

import cv2
import numpy as np

from mission.perception.detector import Detection

ARUCO_DICT_DEFAULT = "DICT_6X6_250"


def is_rover_id(marker_id: int, rover_ids: Iterable[int]) -> bool:
    """True iff `marker_id` is in the configured allow-list (`aruco.rover_ids`)."""
    return marker_id in set(rover_ids)


def _detector(dictionary: str):
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    return cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())


def confirm_with_aruco(image_bgr: np.ndarray, dictionary: str = ARUCO_DICT_DEFAULT,
                       min_marker_px: int = 0) -> List[Detection]:
    """Decode every visible marker → a confirmed `Detection` (id + bbox) per marker, given a
    BGR ndarray (use `detector.frame_bgr` on the frame). A marker whose larger pixel side is
    below `min_marker_px` is dropped (too far to trust)."""
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
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
