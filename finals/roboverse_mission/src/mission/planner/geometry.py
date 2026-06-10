"""Offline routing geometry — inflate footprints, build a visibility graph, plan a
footprint-clear path, and check inter-drone path conflicts.

Everything is 2-D in the arena frame (north, east) metres. **No-overfly is structural:**
routes live entirely in inflated free space, so the reactive guard (P3) is only a
backstop. Paths NEVER intersect an inflated footprint and NEVER leave bounds.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

Point = Tuple[float, float]


@dataclass(frozen=True)
class Rect:
    min_n: float
    min_e: float
    max_n: float
    max_e: float

    def contains(self, p: Point, strict: bool = False, eps: float = 0.0) -> bool:
        if strict:
            return (self.min_n + eps < p[0] < self.max_n - eps and
                    self.min_e + eps < p[1] < self.max_e - eps)
        return (self.min_n - eps <= p[0] <= self.max_n + eps and
                self.min_e - eps <= p[1] <= self.max_e + eps)

    def corners(self) -> List[Point]:
        return [(self.min_n, self.min_e), (self.min_n, self.max_e),
                (self.max_n, self.min_e), (self.max_n, self.max_e)]

    @property
    def center(self) -> Point:
        return ((self.min_n + self.max_n) / 2, (self.min_e + self.max_e) / 2)


def inflate(footprints: Sequence[Tuple[float, float, float, float]],
            radius_m: float) -> List[Rect]:
    """Expand each (cn,ce,sn,se) footprint by `radius_m` on every side → list of Rect."""
    out = []
    for cn, ce, sn, se in footprints:
        hn, he = sn / 2 + radius_m, se / 2 + radius_m
        out.append(Rect(cn - hn, ce - he, cn + hn, ce + he))
    return out


def _segment_hits_rect(p: Point, q: Point, r: Rect, eps: float = 5e-3) -> bool:
    """True if segment p→q passes through the (slightly shrunk) interior of `r`.
    Boundary-tangent segments are allowed (the eps shrink)."""
    xmin, xmax = r.min_n + eps, r.max_n - eps
    ymin, ymax = r.min_e + eps, r.max_e - eps
    if xmax <= xmin or ymax <= ymin:          # rect smaller than 2*eps → ignore
        return False
    dn, de = q[0] - p[0], q[1] - p[1]
    t0, t1 = 0.0, 1.0
    for pk, qk in ((-dn, p[0] - xmin), (dn, xmax - p[0]),
                   (-de, p[1] - ymin), (de, ymax - p[1])):
        if abs(pk) < 1e-15:
            if qk < 0:
                return False                  # parallel and outside this slab
        else:
            rr = qk / pk
            if pk < 0:
                if rr > t1:
                    return False
                if rr > t0:
                    t0 = rr
            else:
                if rr < t0:
                    return False
                if rr < t1:
                    t1 = rr
    return t0 < t1 - 1e-12                     # positive-length interior overlap


def segment_clear(p: Point, q: Point, inflated: Sequence[Rect]) -> bool:
    """True if the segment p→q avoids every inflated footprint interior."""
    return not any(_segment_hits_rect(p, q, r) for r in inflated)


def point_blocked(p: Point, inflated: Sequence[Rect]) -> bool:
    """True if p is strictly inside any inflated footprint (a no-go point)."""
    return any(r.contains(p, strict=True, eps=1e-6) for r in inflated)


@dataclass
class VisibilityGraph:
    nodes: List[Point]
    inflated: List[Rect]
    bounds: Rect
    adj: Dict[int, List[Tuple[int, float]]] = field(default_factory=dict)

    def _visible(self, a: Point, b: Point) -> bool:
        return segment_clear(a, b, self.inflated)


def build_graph(inflated: Sequence[Rect], bounds: Rect,
                corner_push: float = 0.02) -> VisibilityGraph:
    """Visibility graph over inflated-footprint corners (pushed just outside their
    rect, dropped if inside another footprint or out of bounds)."""
    inflated = list(inflated)
    nodes: List[Point] = []
    for r in inflated:
        cx, cy = r.center
        for c in r.corners():
            dn = corner_push if c[0] >= cx else -corner_push
            de = corner_push if c[1] >= cy else -corner_push
            node = (c[0] + dn, c[1] + de)
            if not bounds.contains(node):
                continue
            if point_blocked(node, inflated):
                continue
            nodes.append(node)
    g = VisibilityGraph(nodes=nodes, inflated=inflated, bounds=bounds)
    for i in range(len(nodes)):
        g.adj.setdefault(i, [])
        for j in range(i + 1, len(nodes)):
            if segment_clear(nodes[i], nodes[j], inflated):
                d = math.dist(nodes[i], nodes[j])
                g.adj[i].append((j, d))
                g.adj.setdefault(j, []).append((i, d))
    return g


def plan_path(start: Point, goal: Point,
              graph: VisibilityGraph) -> Optional[List[Point]]:
    """A* over the visibility graph (start/goal added as temp nodes). Returns a
    footprint-clear waypoint list start..goal, or None if start/goal is blocked,
    out of bounds, or no route exists."""
    if not graph.bounds.contains(start) or not graph.bounds.contains(goal):
        return None
    if point_blocked(start, graph.inflated) or point_blocked(goal, graph.inflated):
        return None
    if segment_clear(start, goal, graph.inflated):
        return [start, goal] if start != goal else [start]

    pts = list(graph.nodes) + [start, goal]
    s_idx, g_idx = len(pts) - 2, len(pts) - 1
    adj: Dict[int, List[Tuple[int, float]]] = {i: list(v) for i, v in graph.adj.items()}
    for i in range(len(pts)):
        adj.setdefault(i, [])
    # connect start & goal to every visible graph node (and to each other, already checked)
    for temp in (s_idx, g_idx):
        for j in range(len(graph.nodes)):
            if segment_clear(pts[temp], pts[j], graph.inflated):
                d = math.dist(pts[temp], pts[j])
                adj[temp].append((j, d))
                adj[j].append((temp, d))

    # A*
    def h(i: int) -> float:
        return math.dist(pts[i], goal)

    dist = {s_idx: 0.0}
    prev: Dict[int, int] = {}
    pq = [(h(s_idx), s_idx)]
    visited = set()
    while pq:
        _, u = heapq.heappop(pq)
        if u in visited:
            continue
        visited.add(u)
        if u == g_idx:
            break
        for v, w in adj[u]:
            nd = dist[u] + w
            if nd < dist.get(v, math.inf):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd + h(v), v))
    if g_idx not in dist:
        return None
    path = [g_idx]
    while path[-1] != s_idx:
        path.append(prev[path[-1]])
    path.reverse()
    return [pts[i] for i in path]


# --------------------------------------------------------------------------- #
# inter-drone conflict
# --------------------------------------------------------------------------- #
def _seg_seg_dist(a1: Point, a2: Point, b1: Point, b2: Point) -> float:
    """Minimum distance between two 2-D segments."""
    def sub(p, q):
        return (p[0] - q[0], p[1] - q[1])

    def dot(p, q):
        return p[0] * q[0] + p[1] * q[1]

    d1 = sub(a2, a1)
    d2 = sub(b2, b1)
    r = sub(a1, b1)
    a = dot(d1, d1)
    e = dot(d2, d2)
    f = dot(d2, r)
    if a < 1e-15 and e < 1e-15:
        return math.dist(a1, b1)
    if a < 1e-15:
        s = 0.0
        t = min(1.0, max(0.0, f / e))
    else:
        c = dot(d1, r)
        if e < 1e-15:
            t = 0.0
            s = min(1.0, max(0.0, -c / a))
        else:
            b = dot(d1, d2)
            denom = a * e - b * b
            s = min(1.0, max(0.0, (b * f - c * e) / denom)) if denom > 1e-15 else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = min(1.0, max(0.0, -c / a))
            elif t > 1.0:
                t = 1.0
                s = min(1.0, max(0.0, (b - c) / a))
    cp1 = (a1[0] + d1[0] * s, a1[1] + d1[1] * s)
    cp2 = (b1[0] + d2[0] * t, b1[1] + d2[1] * t)
    return math.dist(cp1, cp2)


def paths_conflict(path_a: Sequence[Point], path_b: Sequence[Point],
                   sep_m: float) -> bool:
    """True if any segment of A comes within `sep_m` of any segment of B
    (crossing → distance 0 → conflict)."""
    for i in range(len(path_a) - 1):
        for j in range(len(path_b) - 1):
            if _seg_seg_dist(path_a[i], path_a[i + 1],
                             path_b[j], path_b[j + 1]) < sep_m:
                return True
    return False
