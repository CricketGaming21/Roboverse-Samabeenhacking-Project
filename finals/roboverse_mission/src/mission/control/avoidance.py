"""Reactive avoidance — the boolean IR/ToF backstop wrapping every command.

The planner routes in 2-D around inflated footprints (no-overfly is structural). This
guard is the last line: it filters each horizontal command against the 5 barrier
booleans — **stop the blocked travel axis, slide toward the open lateral side, and
NEVER command +up** (HARD invariant #3: flying over a footprint voids scores). When
there is no lateral escape it reports `boxed()` so the executor backtracks + reroutes.

Plus inter-drone `separation`: a lower-priority drone yields to a nearer higher-priority one.
"""

from __future__ import annotations

import math
from typing import Iterable, Optional, Tuple


def _hint_to_sign(hint) -> int:
    if hint in ("right", 1, 1.0):
        return 1
    if hint in ("left", -1, -1.0):
        return -1
    return 0


class ReactiveGuard:
    """Filters (cmd_fwd, cmd_right) against barrier booleans. Stateful: commits to a
    slide side until the blocked axis clears (anti-oscillation hysteresis)."""

    def __init__(self, min_slide: float = 0.5):
        self.min_slide = float(min_slide)
        self._slide_side = 0          # committed lateral side: +1 right(east) / -1 left(west)
        self._boxed = False

    def boxed(self) -> bool:
        """True if the LAST filter() found the travel axis blocked with no lateral escape."""
        return self._boxed

    def _choose_side(self, obstacles, hint) -> int:
        """Pick an OPEN lateral side: honour the committed side, then the hint, then east."""
        right_open = not obstacles.right
        left_open = not obstacles.left
        if self._slide_side > 0 and right_open:
            return 1
        if self._slide_side < 0 and left_open:
            return -1
        order = []
        h = _hint_to_sign(hint)
        if h > 0:
            order = [1, -1]
        elif h < 0:
            order = [-1, 1]
        else:
            order = [1, -1]           # default bias: slide east
        for s in order:
            if s > 0 and right_open:
                self._slide_side = 1
                return 1
            if s < 0 and left_open:
                self._slide_side = -1
                return -1
        self._slide_side = 0
        return 0

    def filter(self, cmd_fwd: float, cmd_right: float, obstacles, *,
               open_side_hint=None) -> Tuple[float, float, float]:
        """Return (fwd, right, up). `up` is ALWAYS 0.0 — the guard never climbs."""
        fwd, right = float(cmd_fwd), float(cmd_right)
        self._boxed = False

        # 1) zero a lateral command that drives into a blocked side
        if right > 0 and obstacles.right:
            right = 0.0
        if right < 0 and obstacles.left:
            right = 0.0

        # 2) blocked in the direction of travel → stop that axis, slide laterally
        blocked_fwd = (fwd > 0 and obstacles.forward) or (fwd < 0 and obstacles.back)
        if blocked_fwd:
            mag = abs(cmd_fwd)
            fwd = 0.0
            side = self._choose_side(obstacles, open_side_hint)
            if side == 0:
                self._boxed = True    # surrounded: caller must backtrack + reroute
                right = 0.0
            else:
                slide = side * max(mag, self.min_slide)
                right = max(-1.0, min(1.0, slide))
        else:
            self._slide_side = 0      # clear the commit once the path ahead is open

        return (fwd, right, 0.0)      # NEVER +up


def separation(my_xy: Tuple[float, float],
               others: Iterable[Tuple[Tuple[float, float], float]],
               my_priority: float, sep_m: float = 0.8) -> bool:
    """Right-of-way: return True if THIS drone should YIELD — i.e., a higher-priority
    drone is within `sep_m`. `others` is an iterable of (other_xy, other_priority).
    Higher priority number = right of way; the lower-priority drone yields."""
    for oxy, oprio in others:
        if math.dist(my_xy, oxy) < sep_m and oprio > my_priority:
            return True
    return False
