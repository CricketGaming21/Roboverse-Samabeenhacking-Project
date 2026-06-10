"""YOLO integration SEAM — the two-stage rover-detection example (MISSION side).

This is where the mission's YOLO model plugs in. It runs entirely against the
**public pyhulax API** (`DroneAPI` + its `VideoStream`) and `cv2`/`numpy` — it
imports **no `simcore`**. On the real day the identical code runs against the
real SDK; here it runs against the sim (the path-based `import pyhulax` swap).

THE DETECTION SPLIT (see docs/DETECTION.md):
  - **YOLO is MISSION code** — it finds a rover-shaped blob *from afar* (no id).
    It is NOT in the sim core. Below it is a clearly-marked PLACEHOLDER you swap
    for your real model (ultralytics / RKNN) without touching anything else.
  - **ArUco is REAL in the sim** — the sim renders RoboMaster-style rovers with
    a unique `cv2.aruco` marker on top (and the same dictionary on the landing
    pads); the referee scores on `cv2.aruco`, and the mission confirms identity
    with the SAME `cv2.aruco` on the SAME rendered frames.

THE TWO-STAGE FLOW this example demonstrates:
    1. STAGE 1 — YOLO (placeholder) scans a frame and proposes candidate rover
       boxes *from a distance*  ->  `RoverDetector.detect(frame) -> [Detection]`.
    2. APPROACH — the mission flies/tilts toward a candidate to get it big and
       centred (here: camera straight down + a `move_to`; your mission uses UWB).
    3. STAGE 2 — REAL `cv2.aruco` decodes the marker id up close  ->  identity
       *confirmed*. Only a confirmed id counts as a capture.

Swapping in YOLO: subclass `RoverDetector` (or replace `PlaceholderRoverDetector`)
so `detect()` runs your model and returns one `Detection` per rover. Nothing
else in the flow changes — `two_stage_scan()` already wires stage-1 -> approach
-> stage-2.

Run it:  python -m mission_examples.rover_detection_example
"""

import abc
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

import cv2
import numpy as np

from pyhulax import DroneAPI
from pyhulax.core import CameraPitchMode

# The organiser ArUco dictionary — the sim renders the rover markers from it,
# scores on it, and the mission confirms on it (keep them identical). Landing
# pads carry NO marker (coordinate-only).
ARUCO_DICT = "DICT_7X7_1000"

# Rover marker ids the sim assigns (config rovers.marker_ids).
ROVER_ID_RANGE = frozenset({11, 45, 51, 67, 101})


@dataclass
class Detection:
    """One detected target. `marker_id` is None for a YOLO candidate (a
    rover-shaped blob with no identity yet) and set once ArUco confirms it."""
    bbox: Tuple[int, int, int, int]      # (x, y, w, h) in frame pixels
    conf: float                          # detector confidence 0..1
    marker_id: Optional[int] = None      # decoded id (None until confirmed)
    source: str = "yolo"                 # "yolo" (stage 1) | "aruco" (stage 2)

    @property
    def confirmed(self) -> bool:
        return self.marker_id is not None


# --------------------------------------------------------------------------- #
# STAGE 1 — the YOLO seam (mission code; placeholder shipped here)
# --------------------------------------------------------------------------- #

class RoverDetector(abc.ABC):
    """Interface the mission implements with YOLO. `detect()` takes one RGB
    frame and returns a candidate `Detection` per rover it sees (no id yet —
    identity is confirmed later by ArUco)."""

    @abc.abstractmethod
    def detect(self, frame_rgb: np.ndarray) -> List[Detection]:
        ...


class PlaceholderRoverDetector(RoverDetector):
    """# PLACEHOLDER — plug your YOLO model here.

    A trivial stand-in so the pipeline runs and is testable WITHOUT a real
    model: it proposes a single centre box (as if "a rover is roughly ahead").
    Replace `detect()` with your real inference, e.g.::

        from ultralytics import YOLO
        class YoloRoverDetector(RoverDetector):
            def __init__(self, weights): self.model = YOLO(weights)
            def detect(self, frame_rgb):
                res = self.model(frame_rgb[:, :, ::-1])      # YOLO wants BGR
                return [Detection(bbox=tuple(map(int, b.xywh[0])),
                                  conf=float(b.conf), source="yolo")
                        for b in res[0].boxes]

    Nothing else in this file changes — `two_stage_scan()` still drives the
    stage-1 -> approach -> stage-2 (ArUco) flow.
    """

    def __init__(self, conf: float = 0.5, box_frac: float = 0.34):
        self._conf = float(conf)
        self._box_frac = float(box_frac)

    def detect(self, frame_rgb: np.ndarray) -> List[Detection]:
        h, w = frame_rgb.shape[:2]
        bw, bh = int(w * self._box_frac), int(h * self._box_frac)
        # PLACEHOLDER: a centre box. A real YOLO returns one box per rover it
        # actually sees (and an empty list when none are in view).
        return [Detection(bbox=((w - bw) // 2, (h - bh) // 2, bw, bh),
                          conf=self._conf, marker_id=None, source="yolo")]


# --------------------------------------------------------------------------- #
# STAGE 2 — REAL cv2.aruco identity confirm (the same path the sim scores on)
# --------------------------------------------------------------------------- #

def _aruco_detector(dictionary: str = ARUCO_DICT):
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    return cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())


def confirm_with_aruco(frame_rgb: np.ndarray,
                       dictionary: str = ARUCO_DICT) -> List[Detection]:
    """Run REAL `cv2.aruco` on the frame and return a confirmed `Detection`
    (with a decoded `marker_id`) per visible marker. This is the identity
    confirmation — the sim's referee uses the same detector on the same
    frames, so a confirm here means a capture there."""
    gray = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2GRAY)
    corners, ids, _ = _aruco_detector(dictionary).detectMarkers(gray)
    confirmed: List[Detection] = []
    if ids is not None:
        for mid, quad in zip(ids.flatten(), corners):
            x, y, w, h = cv2.boundingRect(quad[0].astype(np.int32))
            confirmed.append(Detection(bbox=(int(x), int(y), int(w), int(h)),
                                       conf=1.0, marker_id=int(mid),
                                       source="aruco"))
    return confirmed


# --------------------------------------------------------------------------- #
# The seam: stage-1 (YOLO) -> approach -> stage-2 (ArUco), on live frames
# --------------------------------------------------------------------------- #

def grab_frame(stream, timeout_s: float = 2.0) -> Optional[np.ndarray]:
    """Pull the latest RGB frame from a public pyhulax VideoStream (waits for
    the first frame up to timeout_s)."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        frame = stream.latest_frame
        if frame is not None:
            return frame.to_rgb()
        time.sleep(0.02)
    return None


@dataclass
class ScanResult:
    candidates: List[Detection] = field(default_factory=list)  # stage 1 (YOLO)
    confirmed: List[Detection] = field(default_factory=list)   # stage 2 (ArUco)
    frame_shape: Optional[Tuple[int, int, int]] = None

    @property
    def confirmed_ids(self) -> List[int]:
        return [d.marker_id for d in self.confirmed]


def two_stage_scan(stream, detector: RoverDetector,
                   approach: Optional[Callable[[List[Detection]], None]] = None,
                   dictionary: str = ARUCO_DICT) -> ScanResult:
    """One pass of the mission's detection seam:

      1. grab a frame from the public stream,
      2. STAGE 1 — `detector.detect()` (YOLO/placeholder) proposes candidates,
      3. APPROACH — if given a candidate, run `approach(candidates)` so the
         mission closes in (move_to / camera tilt) and re-grab the frame,
      4. STAGE 2 — `confirm_with_aruco()` decodes the real id up close.
    """
    frame = grab_frame(stream)
    if frame is None:
        return ScanResult()
    candidates = detector.detect(frame)                       # STAGE 1 (YOLO)
    if approach is not None and candidates:
        approach(candidates)                                  # APPROACH
        fresh = grab_frame(stream)         # re-grab after closing in
        if fresh is not None:
            frame = fresh
    confirmed = confirm_with_aruco(frame, dictionary)         # STAGE 2 (ArUco)
    return ScanResult(candidates=candidates, confirmed=confirmed,
                      frame_shape=tuple(frame.shape))


# --------------------------------------------------------------------------- #
# Demo runner — fly the sim and show the seam end to end
# --------------------------------------------------------------------------- #

# A downward-looking search sweep (takeoff-frame cm: x=right, y=forward, z=up).
# Tuned for the default sim_config.yaml; the mission uses UWB-derived targets.
_SWEEP_ALT_CM = 210
_SWEEP_X_CM = (150, 0, 300)
_SWEEP_Y_CM = (200, 350, 500, 650, 800)


def _make_approach(drone: DroneAPI) -> Callable[[List[Detection]], None]:
    """The mission's 'close in on a candidate' step. Here: ensure the camera
    looks straight down at the marker. A real mission would `move_to` toward
    the candidate's UWB-resolved position; the seam is identical."""
    def approach(candidates: List[Detection]) -> None:
        drone.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 90)
    return approach


def main(argv=None) -> int:
    import argparse
    ap = argparse.ArgumentParser(
        description="Two-stage rover-detection seam (YOLO stub -> cv2.aruco "
                    "confirm) against the sim, via the public pyhulax API.")
    ap.add_argument("--ip", default="10.0.0.11",
                    help="drone IP (default 10.0.0.11 = first sim drone)")
    args = ap.parse_args(argv)

    detector = PlaceholderRoverDetector()
    print("Two-stage rover detection seam")
    print("  STAGE 1 = YOLO (PLACEHOLDER here — swap in your model)")
    print("  STAGE 2 = REAL cv2.aruco identity confirm "
          f"({ARUCO_DICT})\n")

    d = DroneAPI()
    # connect() boots the shared sim world (or binds the real drone).
    d.connect(args.ip)
    d.set_video_stream(True)
    stream = d.create_video_stream()
    stream.start()
    try:
        d.takeoff(100)
        d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 90)
        approach = _make_approach(d)
        for y in _SWEEP_Y_CM:
            for x in _SWEEP_X_CM:
                d.move_to(x, y, _SWEEP_ALT_CM)
                result = two_stage_scan(stream, detector, approach=approach)
                yolo = result.candidates
                stage1 = (f"{len(yolo)} candidate(s) (conf "
                          f"{yolo[0].conf:.2f}, id={yolo[0].marker_id})"
                          if yolo else "0 candidates")
                print(f"sweep (x={x:>4} y={y:>4} cm):  STAGE1 YOLO -> {stage1}")
                if result.confirmed:
                    for c in result.confirmed:
                        kind = "ROVER" if c.marker_id in ROVER_ID_RANGE else "marker"
                        print(f"    STAGE2 cv2.aruco CONFIRMED {kind} id "
                              f"{c.marker_id}  bbox={c.bbox}")
                    print("\nSeam demonstrated: YOLO (placeholder) proposed a "
                          "candidate, the drone approached, and REAL cv2.aruco "
                          "confirmed the id. Swap PlaceholderRoverDetector for "
                          "your YOLO model to find rovers from afar.")
                    return 0
        print("\nNo marker decoded during the sweep (rovers are off-map until "
              "AMBUSH; pads are visible during DEPLOY). The seam still ran "
              "end to end — STAGE 1 proposed candidates each step.")
        return 0
    finally:
        stream.stop()
        d.land()


if __name__ == "__main__":
    raise SystemExit(main())
