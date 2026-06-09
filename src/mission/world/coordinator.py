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
        self.tick = 0

    def step(self, state, taskboard: TaskBoard, belief, drones: Sequence[DroneView], *,
             chokepoints: Optional[Sequence[Point]] = None) -> Dict[int, Assignment]:
        self.tick += 1
        tagged = state.tagged()
        open_tracks = [t for t in taskboard.tracks()
                       if t.marker_id is not None and t.marker_id not in tagged]
        # triage: erratic = human evader; everything else = autonomous-like (secure first)
        autonomous = sorted((t for t in open_tracks if t.behaviour != "erratic"),
                            key=lambda t: t.marker_id)
        evaders = sorted((t for t in open_tracks if t.behaviour == "erratic"),
                         key=lambda t: t.marker_id)

        out: Dict[int, Assignment] = {}
        free: List[DroneView] = [d for d in drones if not d.busy_locked]
        region = belief.argmax_region() if belief is not None else None

        # Phase A — secure the predictable autonomous tags FIRST
        for track in autonomous:
            if not free:
                break
            d = min(free, key=lambda dv: _dist(dv.xy, track.xy))
            out[d.tag_id] = Assignment(Role.TAG, track.xy)
            free.remove(d)

        # Phase B — only commit to evaders once no untagged autonomous remain
        commit_evaders = evaders and free and \
            (not self.secure_autonomous_first or not autonomous)
        if commit_evaders:
            ev_region = region if region is not None else evaders[0].xy
            pursuer = min(free, key=lambda dv: _dist(dv.xy, ev_region))
            out[pursuer.tag_id] = Assignment(Role.TAG, ev_region)       # chase the belief
            free.remove(pursuer)
            if chokepoints and free:                                    # cooperative containment
                rot = self.tick % len(chokepoints)
                ordered = list(chokepoints[rot:]) + list(chokepoints[:rot])
                for d, cp in zip(list(free), ordered):                  # rotate to avoid deadlock
                    out[d.tag_id] = Assignment(Role.BLOCK, cp)
                    free.remove(d)

        for d in free:
            out[d.tag_id] = Assignment(Role.SWEEP, region)

        for tag, a in out.items():
            taskboard.assign(tag, a.role, a.target)
        return out


def _dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])
