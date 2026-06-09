"""Coordinator — assigns a role + target to each drone from the shared world model.

This + the shared TaskBoard/MissionState/BeliefGrid IS the swarm's "smart comms":
shared memory, no network. Base policy (P6): TAG confirmed-but-untagged rovers with the
nearest free drone; the rest SWEEP toward the belief argmax. Evader containment/secure-
autonomous-first hooks land in P8.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from mission.world.taskboard import Assignment, Role, TaskBoard

Point = Tuple[float, float]


@dataclass
class DroneView:
    tag_id: int
    xy: Point
    priority: int = 0
    busy_locked: bool = False        # mid lock-on → don't re-task (commitment rule, P7)


class Coordinator:
    def __init__(self, *, secure_autonomous_first: bool = True):
        self.secure_autonomous_first = secure_autonomous_first

    def step(self, state, taskboard: TaskBoard, belief,
             drones: Sequence[DroneView]) -> Dict[int, Assignment]:
        tagged = state.tagged()
        # confirmed-but-untagged rover tracks are the actionable targets
        open_tracks = [t for t in taskboard.tracks()
                       if t.marker_id is not None and t.marker_id not in tagged]
        # prefer smooth/periodic (autonomous-like) before erratic (evaders) if configured
        if self.secure_autonomous_first:
            open_tracks.sort(key=lambda t: (t.behaviour == "erratic", t.marker_id))
        else:
            open_tracks.sort(key=lambda t: t.marker_id)

        out: Dict[int, Assignment] = {}
        free: List[DroneView] = [d for d in drones if not d.busy_locked]
        # a locked drone keeps whatever it was doing (commitment) — leave it untouched
        region = belief.argmax_region() if belief is not None else None

        for track in open_tracks:
            if not free:
                break
            d = min(free, key=lambda dv: _dist(dv.xy, track.xy))
            out[d.tag_id] = Assignment(Role.TAG, track.xy)
            free.remove(d)

        for d in free:
            out[d.tag_id] = Assignment(Role.SWEEP, region)

        for tag, a in out.items():
            taskboard.assign(tag, a.role, a.target)
        return out


def _dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])
