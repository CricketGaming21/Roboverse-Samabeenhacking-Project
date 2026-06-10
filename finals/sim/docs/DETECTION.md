# Detection stack — what's REAL in the sim vs what's MISSION code

This sim is built so the **mission's detection code runs unchanged** against
the sim or the real drone. To keep that promise, the SIM/MISSION boundary is
explicit about which detector lives where.

## The split (read this first)

| Stage | Lives in | Real in the sim? | Notes |
|-------|----------|------------------|-------|
| **ArUco** (`cv2.aruco`) | sim **and** mission | **YES — real** | The sim renders a unique `cv2.aruco` marker on every rover (and on the landing pads) and runs `cv2.aruco` on the rendered frames to **score** captures and draw the on-screen detect→acquire boxes. The mission runs the **same** `cv2.aruco` on the **same** frames to confirm identity. |
| **YOLO** (object detection) | **mission only** | n/a — not in the sim core | YOLO is the mission's *find-a-rover-from-afar* stage. It is **not** imported into `simcore`. The sim's job is to render **detectable RoboMaster-style rovers** and **serve frames**; the mission runs YOLO on those frames. |

In one line: **ArUco is real in the sim; YOLO is mission code run on the sim's
frames.** The sim provides detectable rovers + real frames + a documented seam.

## Why this split

- **ArUco is the scoring truth.** Each rover carries a unique marker on top and
  each pad carries one; "capturing a rover" == decoding its marker. So the sim
  *must* run real `cv2.aruco` (for the referee and the recorder/dashboard
  overlays), and the mission confirms with the identical detector — no
  sim-vs-mission detector mismatch.
- **YOLO is a contingency / approach aid, and it's the team's own model.**
  Putting YOLO in the sim core would (a) couple the simulator to a heavy
  mission dependency (ultralytics/RKNN) and (b) blur the boundary that's held
  for the whole build. The sim instead renders rovers with **real geometry**
  (not flat billboards — see `simcore/world.py`) so a mission-side YOLO has
  something plausible to detect, and serves the frames over the public API.

## The two-stage flow (find → approach → confirm)

The intended mission pattern, demonstrated end to end in
`mission_examples/rover_detection_example.py`:

1. **STAGE 1 — YOLO (mission):** scan a camera frame, propose candidate
   rover boxes *from a distance*. No identity yet.
2. **APPROACH (mission):** fly/tilt toward a candidate (UWB-guided `move_to` +
   camera down) so the marker is large and centred.
3. **STAGE 2 — ArUco (real, shared):** `cv2.aruco` decodes the marker id up
   close. Only a **confirmed id** counts as a capture — and because the sim's
   referee uses the same detector on the same frames, a confirm here is a score
   there.

## The seam (where your YOLO plugs in)

`mission_examples/rover_detection_example.py` defines the interface and ships a
runnable placeholder:

```python
class RoverDetector(abc.ABC):
    @abc.abstractmethod
    def detect(self, frame_rgb) -> list[Detection]: ...   # one box per rover

class PlaceholderRoverDetector(RoverDetector):
    # PLACEHOLDER — plug your YOLO model here.
    def detect(self, frame_rgb): ...   # trivial centre box, so the pipeline runs
```

Swap the placeholder for your model — nothing else changes:

```python
from ultralytics import YOLO
class YoloRoverDetector(RoverDetector):
    def __init__(self, weights): self.model = YOLO(weights)
    def detect(self, frame_rgb):
        res = self.model(frame_rgb[:, :, ::-1])          # YOLO wants BGR
        return [Detection(bbox=tuple(map(int, b.xywh[0])),
                          conf=float(b.conf), source="yolo")
                for b in res[0].boxes]
```

`two_stage_scan(stream, detector, approach=...)` already wires **stage-1 →
approach → stage-2 (real `cv2.aruco`)**, pulling frames from the public
`VideoStream` (`create_video_stream()` → `latest_frame.to_rgb()`).

## How the frames reach you (public API only)

```python
from pyhulax import DroneAPI
from pyhulax.core import CameraPitchMode

d = DroneAPI(); d.connect(ip)            # boots the sim world (or binds real drone)
d.set_video_stream(True)
stream = d.create_video_stream(); stream.start()
d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 90)   # look down at rover tops
rgb = stream.latest_frame.to_rgb()       # (H, W, 3) uint8 RGB — feed to YOLO/ArUco
```

The example imports **`pyhulax` + `cv2`/`numpy` only — never `simcore`**. That
import boundary IS the sim↔real swap: the same mission code runs against the
real SDK on the day.

## Markers used

- Dictionary: **`DICT_7X7_1000`** (config `aruco.dictionary`).
- **Rover** marker ids: `11, 45, 51, 67, 101` (config `rovers.marker_ids` —
  autonomous `11/45/51`, evasive `67/101`).
- **Landing zones carry NO ArUco marker** — they are coordinate-only floor
  markings; the referee detects ONLY rover markers (during AMBUSH).

## Run the example

```bash
python -m mission_examples.rover_detection_example          # against the sim
```

It prints the placeholder YOLO candidates each sweep step and the real
`cv2.aruco` confirmation when a marker comes into view — showing exactly where
your YOLO model drops in.
