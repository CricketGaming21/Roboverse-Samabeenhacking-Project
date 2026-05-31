"""
barrel_tracker.py — v3: barrel queue, fly-toward lock-on, robust dedup.
"""

import math
import time
import json
import os
import threading
from dataclasses import dataclass, field, asdict
from typing import List
import config as C


class LockOn:
    """
    Manages a QUEUE of barrel sightings.
    When one lock-on completes, the next in queue is served.
    Cooldown prevents re-triggering on same area.
    """
    def __init__(self):
        self._lock = threading.Lock()
        self.active = False
        self.yaw_offset = 0.0
        self.class_name = ""
        self.confidence = 0.0
        self.bbox = [0, 0, 0, 0]
        self.bbox_height = 0
        self.timestamp = 0.0
        self._last_cleared_time = 0.0
        # Queue for second barrel when two are spotted simultaneously
        self._queue = []

    def set_sighting(self, cls, conf, bbox):
        if time.monotonic() - self._last_cleared_time < C.LOCKON_COOLDOWN_S:
            return
        offset_px = (bbox[0] + bbox[2]) / 2.0 - C.CAM_CX
        yaw_off = math.degrees(math.atan(offset_px / C.CAM_FX))
        # Reject barrels at extreme angles — they cause the drone to
        # turn 70°+, lose heading, and circle. The barrel will appear
        # at a better angle as the drone flies past it.
        if abs(yaw_off) > 50.0:
            return
        entry = {
            "yaw_offset": yaw_off,
            "class_name": cls,
            "confidence": conf,
            "bbox": list(bbox),
            "bbox_height": bbox[3] - bbox[1],
        }
        with self._lock:
            if not self.active:
                self._apply(entry)
            else:
                # Queue it — will be served after current lock-on completes
                self._queue.append(entry)

    def _apply(self, entry):
        """Set the active lock-on from an entry dict."""
        self.active = True
        self.yaw_offset = entry["yaw_offset"]
        self.class_name = entry["class_name"]
        self.confidence = entry["confidence"]
        self.bbox = entry["bbox"]
        self.bbox_height = entry["bbox_height"]
        self.timestamp = time.monotonic()
        self._frozen_pos = None  # will be set by main.py when lock-on starts

    def set_frozen_position(self, n, e, alt):
        """Freeze the barrel detection position so all try_log calls
        during fly-toward use the SAME position — prevents dedup failure."""
        with self._lock:
            self._frozen_pos = (n, e, alt)

    def get_frozen_position(self):
        with self._lock:
            return self._frozen_pos
        self.timestamp = time.monotonic()

    def get(self):
        with self._lock:
            if not self.active:
                return None
            return {
                "yaw_offset": self.yaw_offset,
                "class_name": self.class_name,
                "confidence": self.confidence,
                "bbox": self.bbox,
                "bbox_height": self.bbox_height,
            }

    def clear(self):
        with self._lock:
            self.active = False
            self._last_cleared_time = time.monotonic()
            self._queue.clear()

    def in_cooldown(self):
        return time.monotonic() - self._last_cleared_time < C.LOCKON_COOLDOWN_S

    def queue_size(self):
        with self._lock:
            return len(self._queue)

    def clear_queue(self):
        with self._lock:
            self._queue.clear()

    def get_latest_bbox(self):
        """Called by the flight loop to get the LATEST bbox from ongoing YOLO frames
        during the fly-toward phase, so the drone can track the barrel in real time."""
        with self._lock:
            if not self.active:
                return None
            return list(self.bbox), self.bbox_height

    def update_bbox(self, bbox):
        """Called by YOLO callback to update the bbox during fly-toward phase."""
        with self._lock:
            if self.active:
                self.bbox = list(bbox)
                self.bbox_height = bbox[3] - bbox[1]
                offset_px = (bbox[0] + bbox[2]) / 2.0 - C.CAM_CX
                self.yaw_offset = math.degrees(math.atan(offset_px / C.CAM_FX))


@dataclass
class Detection:
    class_name: str
    north:      float
    east:       float
    altitude:   float
    confidence: float
    frames:     int  = 1
    confirmed:  bool = False
    det_id:     int  = 0
    timestamp:  float = field(default_factory=time.time)


class BarrelTracker:
    def __init__(self):
        self.pending:   List[Detection] = []
        self.confirmed: List[Detection] = []
        self._id = 0
        self._lock = threading.Lock()
        self._last_confirm_time = {}

    def try_log(self, cls, n, e, alt, conf):
        with self._lock:
            # Time cooldown per class
            if time.monotonic() - self._last_confirm_time.get(cls, 0) < C.CONFIRM_COOLDOWN_S:
                return False

            # Check pending — increment frame count
            for p in self.pending:
                if p.class_name == cls and math.hypot(p.north - n, p.east - e) < C.BARREL_SEP_M:
                    p.frames += 1
                    # Update position to average (improves accuracy)
                    p.north = (p.north * (p.frames-1) + n) / p.frames
                    p.east  = (p.east  * (p.frames-1) + e) / p.frames
                    if p.frames >= C.CONFIRM_FRAMES and not p.confirmed:
                        p.confirmed = True
                        p.det_id = self._id
                        self._id += 1
                        self.confirmed.append(p)
                        self._last_confirm_time[cls] = time.monotonic()
                        self._announce(p)
                        return True
                    return False

            # Check not already confirmed
            for c in self.confirmed:
                if c.class_name == cls and math.hypot(c.north - n, c.east - e) < C.BARREL_SEP_M:
                    return False

            self.pending.append(Detection(
                class_name=cls, north=round(n, 1), east=round(e, 1),
                altitude=round(alt, 1), confidence=round(conf, 2)))
            return False

    def has_nearby(self, cls, n, e):
        with self._lock:
            for c in self.confirmed:
                if c.class_name == cls and math.hypot(c.north - n, c.east - e) < C.BARREL_SEP_M:
                    return True
            for p in self.pending:
                if p.confirmed and p.class_name == cls and math.hypot(p.north - n, p.east - e) < C.BARREL_SEP_M:
                    return True
        return False

    def _announce(self, d):
        icon = "🟡" if "yellow" in d.class_name else "🔴"
        pts = 50 if "yellow" in d.class_name else 100
        print(f"\n  {icon} BARREL #{d.det_id} CONFIRMED: {d.class_name}")
        print(f"     Barrel pos: N={d.north}  E={d.east}  Alt={d.altitude}m")
        print(f"     +{pts}pts → Score: {self.score()}pts\n")

    def count(self, cls):
        return sum(1 for d in self.confirmed if d.class_name == cls)

    def score(self):
        return self.count("yellow_barrel") * 50 + self.count("red_barrel") * 100

    def qualifies(self):
        return self.count("yellow_barrel") >= 1 and self.count("red_barrel") >= 1

    def save(self, path="barrel_detections.json"):
        with self._lock:
            data = {
                "time": time.strftime("%Y-%m-%d %H:%M:%S"),
                "score": self.score(),
                "qualifies": self.qualifies(),
                "yellow": self.count("yellow_barrel"),
                "red": self.count("red_barrel"),
                "detections": [asdict(d) for d in self.confirmed],
            }
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
        print(f"  💾 Saved → {os.path.abspath(path)}")

    def reset_pending(self):
        with self._lock:
            self.pending.clear()


def _bbox_height(bbox):
    return bbox[3] - bbox[1]

def _bbox_center_x(bbox):
    return (bbox[0] + bbox[2]) / 2.0

def _bbox_centered(bbox):
    return abs(_bbox_center_x(bbox) - C.CAM_CX) < C.LOCKON_CENTER_TOL


def make_detection_callback(tracker, lockon, get_position):
    def callback(detections, annotated_image, context):
        pos = get_position()
        if pos is None:
            return
        n, e, alt = pos

        # If lock-on is active, update bbox and use FROZEN position for logging
        if lockon.active:
            frozen = lockon.get_frozen_position()
            log_n, log_e, log_alt = frozen if frozen else (n, e, alt)
            for d in detections:
                if d["class_name"] == lockon.class_name:
                    lockon.update_bbox(d["bbox"])
                    # Try to confirm using FROZEN position (not moving drone pos)
                    if d["confidence"] >= C.CONFIRM_CONF and _bbox_centered(d["bbox"]):
                        tracker.try_log(d["class_name"], log_n, log_e, log_alt, d["confidence"])
            return  # Don't trigger new lock-ons while one is active

        confirm_candidates = []
        lockon_candidates = []

        for d in detections:
            bbox = d["bbox"]
            conf = d["confidence"]
            cls  = d["class_name"]

            if _bbox_height(bbox) < C.MIN_BBOX_H_PX:
                continue
            if tracker.has_nearby(cls, n, e):
                continue

            if conf >= C.CONFIRM_CONF and _bbox_centered(bbox):
                tracker.try_log(cls, n, e, alt, conf)
                continue

            lockon_candidates.append(d)

        # Queue ALL viable candidates (sorted by bbox size, largest first)
        if not lockon.in_cooldown() and lockon_candidates:
            sorted_candidates = sorted(lockon_candidates,
                                       key=lambda d: _bbox_height(d["bbox"]),
                                       reverse=True)
            # First one becomes active lock-on
            first = sorted_candidates[0]
            lockon.set_sighting(first["class_name"], first["confidence"], first["bbox"])
            # Remaining go into queue
            for cand in sorted_candidates[1:]:
                lockon.set_sighting(cand["class_name"], cand["confidence"], cand["bbox"])

    return callback
