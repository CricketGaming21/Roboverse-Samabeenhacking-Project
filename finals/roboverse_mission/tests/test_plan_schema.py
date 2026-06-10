"""P9 — the mission_plan export contract. The committed example must validate against the
schema AND be consumable by the mission (P4 pad/route + P7 vantages). Tampering must raise."""

import copy
from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError

from mission.config import load_config
from mission.mission.phase2_search import point_in_poly
from mission.mission.plan import (MissionPlan, PlanValidationError,
                                  load_mission_plan, validate_plan)
from mission.planner.geometry import Rect, inflate, segment_clear

pytestmark = pytest.mark.p9

_ROOT = Path(__file__).resolve().parent.parent
EXAMPLE = _ROOT / "examples" / "mission_plan.example.yaml"
GUI = _ROOT / "planner_gui" / "index.html"


def _valid_pad_ids():
    return [p.id for p in load_config().valid_pads()]


def _raw():
    return yaml.safe_load(EXAMPLE.read_text())


# --------------------------------------------------------------------------- #
# the example validates and is consumable
# --------------------------------------------------------------------------- #
def test_example_loads_and_validates():
    plan = load_mission_plan(EXAMPLE, valid_pad_ids=_valid_pad_ids())
    assert plan.version == 1
    assert len(plan.drones) == 3


def test_p4_consumes_pad_and_route():
    plan = load_mission_plan(EXAMPLE, valid_pad_ids=_valid_pad_ids())
    valid = set(_valid_pad_ids())
    assert set(plan.pad_assignment().values()) <= valid           # P4: pads are valid
    arena = plan.arena
    inflated = inflate([(c.center[0], c.center[1], c.size[0], c.size[1])
                        for c in arena.crates], arena.inflate_m)
    for d in plan.drones:
        route = plan.route(d.tag_id)
        assert len(route) >= 2
        for i in range(len(route) - 1):                            # route is footprint-clear
            assert segment_clear(route[i], route[i + 1], inflated)


def test_p7_consumes_vantages_inside_bubbles():
    plan = load_mission_plan(EXAMPLE, valid_pad_ids=_valid_pad_ids())
    for d in plan.drones:
        bubble = plan.bubble(d.tag_id)
        vantages = plan.vantages(d.tag_id)
        assert vantages, "each drone needs vantages for P7"
        for v in vantages:                                         # shape vantage_patrol expects
            assert set(v) >= {"xy", "gimbal_deg", "dwell_s"}
            assert len(v["xy"]) == 2
            assert point_in_poly(v["xy"], bubble)                  # P7 bubble-gate consumes this


# --------------------------------------------------------------------------- #
# tampering is rejected (honest validator)
# --------------------------------------------------------------------------- #
def test_unknown_key_rejected(tmp_path):
    data = _raw()
    data["drones"][0]["surprise"] = 1
    p = tmp_path / "p.yaml"; p.write_text(yaml.safe_dump(data))
    with pytest.raises(ValidationError):
        load_mission_plan(p, valid_pad_ids=_valid_pad_ids())


def test_route_through_footprint_rejected(tmp_path):
    data = _raw()
    data["drones"][0]["phase1"]["route_m"] = [[0.6, 1.0], [8.0, 3.0]]   # cuts the crate
    p = tmp_path / "p.yaml"; p.write_text(yaml.safe_dump(data))
    with pytest.raises(PlanValidationError):
        load_mission_plan(p, valid_pad_ids=_valid_pad_ids())


def test_vantage_outside_bubble_rejected(tmp_path):
    data = _raw()
    data["drones"][0]["phase2"]["vantages"][0]["xy"] = [5.0, 5.0]       # not in drone-0's bubble
    p = tmp_path / "p.yaml"; p.write_text(yaml.safe_dump(data))
    with pytest.raises(PlanValidationError):
        load_mission_plan(p, valid_pad_ids=_valid_pad_ids())


def test_invalid_pad_rejected(tmp_path):
    data = _raw()
    data["drones"][0]["phase1"]["pad_id"] = 13                         # 13 is announced invalid
    p = tmp_path / "p.yaml"; p.write_text(yaml.safe_dump(data))
    with pytest.raises(PlanValidationError):
        load_mission_plan(p, valid_pad_ids=_valid_pad_ids())


def test_overlapping_bubbles_rejected(tmp_path):
    data = _raw()
    data["drones"][1]["phase2"]["bubble"] = [[0, 0], [10, 0], [10, 2], [0, 2]]  # overlaps drone 0
    p = tmp_path / "p.yaml"; p.write_text(yaml.safe_dump(data))
    with pytest.raises(PlanValidationError):
        load_mission_plan(p, valid_pad_ids=_valid_pad_ids())


# --------------------------------------------------------------------------- #
# the GUI exists and exports this contract
# --------------------------------------------------------------------------- #
def test_planner_gui_exists_and_targets_the_contract():
    assert GUI.exists()
    html = GUI.read_text()
    assert "<canvas" in html
    assert "version: 1" in html          # exports the schema version
    assert "route_m" in html and "vantages" in html and "bubble" in html
