"""BeliefGrid — occupancy probability over free arena cells for each un-tagged target.

Drives "search where it likely is" instead of a fixed sweep (docs/ARCHITECTURE.md):
  - `observe(cells)` collapses belief where a camera just looked (target not there),
  - `diffuse(dt, rover_speed)` spreads belief through free space over time (lanes),
  - `spike(xy)` adds mass at a sighting,
  - `argmax_region()` returns the most-likely place to look next.
`chokepoints()` extracts narrow passages from the crate map (containment, P8). Lock-guarded.
"""

from __future__ import annotations

import threading
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

from mission.planner.geometry import Rect

Point = Tuple[float, float]


class BeliefGrid:
    def __init__(self, bounds: Rect,
                 footprints: Sequence[Tuple[float, float, float, float]] = (),
                 cell_size: float = 0.25, inflate: float = 0.0):
        self._lock = threading.Lock()
        self.bounds = bounds
        self.cell = float(cell_size)
        self.footprints = list(footprints)
        self.inflate = float(inflate)
        self.nn = max(1, int(round((bounds.max_n - bounds.min_n) / self.cell)))
        self.ne = max(1, int(round((bounds.max_e - bounds.min_e) / self.cell)))
        self.free = np.ones((self.nn, self.ne), dtype=bool)
        for i in range(self.nn):
            for j in range(self.ne):
                if self._in_footprint(self._center(i, j)):
                    self.free[i, j] = False
        self.grid = np.where(self.free, 1.0, 0.0)
        self._normalize()

    # -- geometry --------------------------------------------------------- #
    def _center(self, i: int, j: int) -> Point:
        return (self.bounds.min_n + (i + 0.5) * self.cell,
                self.bounds.min_e + (j + 0.5) * self.cell)

    def _in_footprint(self, p: Point) -> bool:
        for cn, ce, sn, se in self.footprints:
            if (abs(p[0] - cn) <= sn / 2 + self.inflate and
                    abs(p[1] - ce) <= se / 2 + self.inflate):
                return True
        return False

    def cell_of(self, xy: Point) -> Tuple[int, int]:
        i = int((xy[0] - self.bounds.min_n) / self.cell)
        j = int((xy[1] - self.bounds.min_e) / self.cell)
        return (min(max(i, 0), self.nn - 1), min(max(j, 0), self.ne - 1))

    def cells_in_radius(self, xy: Point, radius_m: float) -> List[Tuple[int, int]]:
        r = int(np.ceil(radius_m / self.cell))
        ci, cj = self.cell_of(xy)
        out = []
        for i in range(max(0, ci - r), min(self.nn, ci + r + 1)):
            for j in range(max(0, cj - r), min(self.ne, cj + r + 1)):
                cn, ce = self._center(i, j)
                if (cn - xy[0]) ** 2 + (ce - xy[1]) ** 2 <= radius_m ** 2:
                    out.append((i, j))
        return out

    # -- belief updates --------------------------------------------------- #
    def _normalize(self) -> None:
        s = self.grid.sum()
        if s > 0:
            self.grid /= s

    def observe(self, cells: Iterable[Tuple[int, int]]) -> None:
        """Collapse belief where a camera observed (target not there) → renormalize."""
        with self._lock:
            for i, j in cells:
                if 0 <= i < self.nn and 0 <= j < self.ne:
                    self.grid[i, j] = 0.0
            self._normalize()

    def diffuse(self, dt: float, rover_speed: float) -> None:
        """Spread belief through free space (lanes). More speed×time → more spread."""
        steps = max(1, int(round(rover_speed * dt / self.cell)))
        with self._lock:
            for _ in range(steps):
                g = self.grid
                acc = g.copy()
                wt = np.ones_like(g)
                for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    shifted = np.roll(g, (di, dj), axis=(0, 1))
                    fmask = np.roll(self.free, (di, dj), axis=(0, 1)) & self.free
                    acc += np.where(fmask, shifted * 0.25, 0.0)
                    wt += np.where(fmask, 0.25, 0.0)
                g = acc / wt
                g[~self.free] = 0.0
                self.grid = g
            self._normalize()

    def spike(self, xy: Point, mass: float = 1.0) -> None:
        """Inject probability mass at a sighting location → becomes the argmax."""
        with self._lock:
            i, j = self.cell_of(xy)
            if self.free[i, j]:
                self.grid[i, j] += mass
            self._normalize()

    def belief_at(self, xy: Point) -> float:
        with self._lock:
            i, j = self.cell_of(xy)
            return float(self.grid[i, j])

    def argmax_region(self) -> Point:
        with self._lock:
            idx = int(np.argmax(self.grid))
            i, j = divmod(idx, self.ne)
            return self._center(i, j)

    def snapshot(self) -> np.ndarray:
        with self._lock:
            return self.grid.copy()

    # -- chokepoint / lane graph ----------------------------------------- #
    def chokepoints(self, lane_max: float = 1.6) -> List[Point]:
        """Narrow passages between footprint pairs (and footprint↔wall gaps)."""
        rects = [Rect(cn - sn / 2, ce - se / 2, cn + sn / 2, ce + se / 2)
                 for cn, ce, sn, se in self.footprints]
        pts: List[Point] = []
        # between pairs of footprints
        for a in range(len(rects)):
            for b in range(a + 1, len(rects)):
                ra, rb = rects[a], rects[b]
                # corridor running east (gap in north), needs east overlap
                e_lo, e_hi = max(ra.min_e, rb.min_e), min(ra.max_e, rb.max_e)
                if e_hi > e_lo:
                    if ra.max_n < rb.min_n:
                        gap = rb.min_n - ra.max_n
                        if 0 < gap < lane_max:
                            pts.append(((ra.max_n + rb.min_n) / 2, (e_lo + e_hi) / 2))
                    elif rb.max_n < ra.min_n:
                        gap = ra.min_n - rb.max_n
                        if 0 < gap < lane_max:
                            pts.append(((rb.max_n + ra.min_n) / 2, (e_lo + e_hi) / 2))
                # corridor running north (gap in east), needs north overlap
                n_lo, n_hi = max(ra.min_n, rb.min_n), min(ra.max_n, rb.max_n)
                if n_hi > n_lo:
                    if ra.max_e < rb.min_e:
                        gap = rb.min_e - ra.max_e
                        if 0 < gap < lane_max:
                            pts.append(((n_lo + n_hi) / 2, (ra.max_e + rb.min_e) / 2))
                    elif rb.max_e < ra.min_e:
                        gap = ra.min_e - rb.max_e
                        if 0 < gap < lane_max:
                            pts.append(((n_lo + n_hi) / 2, (rb.max_e + ra.min_e) / 2))
        return pts
