"""mission_plan.yaml — the planner→mission contract (docs/MISSION_PLAN_SCHEMA.md).

Authored in the Planner GUI (P9), consumed by the mission (P4 pad+route, P7 vantages).
Loaded + validated here: unknown keys are rejected (pydantic `extra="forbid"`) AND the
geometric rules are enforced — every route segment is footprint-clear + in bounds, each
vantage lies inside its (footprint-clear) bubble, bubbles are pairwise disjoint, and
pad_ids are valid. A bad plan raises `PlanValidationError`, not a silent fly-into-a-wall.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import yaml
from pydantic import BaseModel, ConfigDict, conlist

from mission.planner.geometry import (Rect, inflate, point_blocked, segment_clear)

Point = Tuple[float, float]


class PlanValidationError(ValueError):
    pass


class _B(BaseModel):
    model_config = ConfigDict(extra="forbid")


class PlanCrate(_B):
    center: conlist(float, min_length=2, max_length=2)
    size: conlist(float, min_length=2, max_length=2)
    height_m: float


class PlanArena(_B):
    length_m: float
    width_m: float
    inflate_m: float
    crates: List[PlanCrate] = []


class Vantage(_B):
    xy: conlist(float, min_length=2, max_length=2)
    look_yaw_deg: float
    gimbal_deg: float
    dwell_s: float


class Phase1(_B):
    pad_id: int
    route_m: List[conlist(float, min_length=2, max_length=2)]


class Phase2(_B):
    bubble: List[conlist(float, min_length=2, max_length=2)]
    vantages: List[Vantage]


class PlanDrone(_B):
    tag_id: int
    color: str
    phase1: Phase1
    phase2: Phase2


class PlanMeta(_B):
    hoop_tol_m: float
    cruise_alt_m: float


class MissionPlan(_B):
    version: int
    arena: PlanArena
    drones: List[PlanDrone]
    meta: PlanMeta

    # -- consumer helpers (P4 / P7) -------------------------------------- #
    def drone(self, tag_id: int) -> PlanDrone:
        for d in self.drones:
            if d.tag_id == tag_id:
                return d
        raise KeyError(tag_id)

    def pad_assignment(self) -> Dict[int, int]:
        return {d.tag_id: d.phase1.pad_id for d in self.drones}

    def route(self, tag_id: int) -> List[Point]:
        return [(p[0], p[1]) for p in self.drone(tag_id).phase1.route_m]

    def bubble(self, tag_id: int) -> List[Point]:
        return [(p[0], p[1]) for p in self.drone(tag_id).phase2.bubble]

    def vantages(self, tag_id: int) -> List[dict]:
        return [{"xy": (v.xy[0], v.xy[1]), "look_yaw_deg": v.look_yaw_deg,
                 "gimbal_deg": v.gimbal_deg, "dwell_s": v.dwell_s}
                for v in self.drone(tag_id).phase2.vantages]


def _point_in_poly(p: Point, poly: Sequence[Point]) -> bool:
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > p[1]) != (yj > p[1])) and \
                (p[0] < (xj - xi) * (p[1] - yi) / (yj - yi + 1e-15) + xi):
            inside = not inside
        j = i
    return inside


def _bbox(poly: Sequence[Point]) -> Tuple[float, float, float, float]:
    ns = [p[0] for p in poly]
    es = [p[1] for p in poly]
    return (min(ns), min(es), max(ns), max(es))


def _bboxes_overlap(a: Sequence[Point], b: Sequence[Point], buffer: float) -> bool:
    an0, ae0, an1, ae1 = _bbox(a)
    bn0, be0, bn1, be1 = _bbox(b)
    separated = (an1 + buffer <= bn0 or bn1 + buffer <= an0 or
                 ae1 + buffer <= be0 or be1 + buffer <= ae0)
    return not separated


def validate_plan(plan: MissionPlan, *, valid_pad_ids: Optional[Sequence[int]] = None,
                  bubble_buffer_m: float = 0.1) -> MissionPlan:
    a = plan.arena
    bounds = Rect(0.0, 0.0, a.length_m, a.width_m)
    inflated = inflate([(c.center[0], c.center[1], c.size[0], c.size[1])
                        for c in a.crates], a.inflate_m)
    errors: List[str] = []

    if plan.version != 1:
        errors.append(f"version must be 1, got {plan.version}")
    tags = [d.tag_id for d in plan.drones]
    if len(set(tags)) != len(tags):
        errors.append("duplicate tag_id")
    pads = [d.phase1.pad_id for d in plan.drones]
    if len(set(pads)) != len(pads):
        errors.append("duplicate pad_id (two drones on one pad)")

    for d in plan.drones:
        if valid_pad_ids is not None and d.phase1.pad_id not in set(valid_pad_ids):
            errors.append(f"drone {d.tag_id}: pad_id {d.phase1.pad_id} is not a valid pad")
        route = [(p[0], p[1]) for p in d.phase1.route_m]
        for p in route:
            if not bounds.contains(p):
                errors.append(f"drone {d.tag_id}: route point {p} out of bounds")
        for i in range(len(route) - 1):
            if not segment_clear(route[i], route[i + 1], inflated):
                errors.append(f"drone {d.tag_id}: route {route[i]}->{route[i + 1]} "
                              f"crosses an inflated footprint")
        bubble = [(p[0], p[1]) for p in d.phase2.bubble]
        for v in d.phase2.vantages:
            xy = (v.xy[0], v.xy[1])
            if not bounds.contains(xy):
                errors.append(f"drone {d.tag_id}: vantage {xy} out of bounds")
            elif not _point_in_poly(xy, bubble):
                errors.append(f"drone {d.tag_id}: vantage {xy} outside its bubble")
            if point_blocked(xy, inflated):
                errors.append(f"drone {d.tag_id}: vantage {xy} on a footprint")

    bubs = [[(p[0], p[1]) for p in d.phase2.bubble] for d in plan.drones]
    for i in range(len(bubs)):
        for j in range(i + 1, len(bubs)):
            if _bboxes_overlap(bubs[i], bubs[j], bubble_buffer_m):
                errors.append(f"bubbles of drones {plan.drones[i].tag_id} & "
                              f"{plan.drones[j].tag_id} overlap (need {bubble_buffer_m} m buffer)")

    if errors:
        raise PlanValidationError("; ".join(errors))
    return plan


def load_mission_plan(path, *, valid_pad_ids: Optional[Sequence[int]] = None,
                      bubble_buffer_m: float = 0.1) -> MissionPlan:
    data = yaml.safe_load(Path(path).read_text())
    plan = MissionPlan(**data)            # pydantic: rejects unknown keys / bad shapes
    return validate_plan(plan, valid_pad_ids=valid_pad_ids, bubble_buffer_m=bubble_buffer_m)
