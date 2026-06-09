"""TaskBoard — live rover tracks + per-drone role/target assignments (lock-guarded).

The shared object that, with the Coordinator, IS the "smart comms": shared memory, no
network protocol (docs/ARCHITECTURE.md). Tracks de-dup by id (estimating velocity from
consecutive sightings); assignments hold each drone's current role + target.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple

Point = Tuple[float, float]


class Role(str, Enum):
    SWEEP = "SWEEP"     # patrol vantages / search the belief
    TAG = "TAG"         # go confirm-and-tag a specific target
    BLOCK = "BLOCK"     # hold a chokepoint (containment, P8)
    COVER = "COVER"     # watch a crate shadow / stale region


@dataclass
class Assignment:
    role: Role
    target: Optional[Point] = None


@dataclass
class Track:
    marker_id: Optional[int]            # None until confirmed
    xy: Point
    t: float
    velocity: Point = (0.0, 0.0)
    behaviour: str = "unknown"          # smooth/periodic/erratic (P8 triage)
    last_seen: float = 0.0


class TaskBoard:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._tracks: Dict[object, Track] = {}
        self._assignments: Dict[int, Assignment] = {}
        self._anon = 0

    def see(self, track: Track) -> None:
        """Record/refresh a sighting. Same id → update + estimate velocity from Δ."""
        with self._lock:
            if track.marker_id is not None:
                key: object = ("id", track.marker_id)
            else:
                self._anon += 1
                key = ("anon", self._anon)
            prev = self._tracks.get(key)
            if prev is not None:
                dt = track.t - prev.t
                if dt > 1e-6:
                    track.velocity = ((track.xy[0] - prev.xy[0]) / dt,
                                      (track.xy[1] - prev.xy[1]) / dt)
            track.last_seen = track.t
            self._tracks[key] = track

    def tracks(self) -> List[Track]:
        with self._lock:
            return list(self._tracks.values())

    def track_for(self, marker_id: int) -> Optional[Track]:
        with self._lock:
            return self._tracks.get(("id", marker_id))

    def assign(self, tag_id: int, role: Role, target: Optional[Point] = None) -> None:
        with self._lock:
            self._assignments[tag_id] = Assignment(role, target)

    def assignment(self, tag_id: int) -> Optional[Assignment]:
        with self._lock:
            return self._assignments.get(tag_id)

    def assignments(self) -> Dict[int, Assignment]:
        with self._lock:
            return dict(self._assignments)
