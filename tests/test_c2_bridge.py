"""P10 — C2 bridge: valid JSON snapshot, over-footprint + other alarms, scoreboard,
evidence-bundle export, and operator overrides reaching the coordinator."""

import json
from pathlib import Path

import numpy as np
import pytest

from mission.planner.geometry import Rect
from mission.runtime.c2_bridge import C2Bridge
from mission.world.belief_grid import BeliefGrid
from mission.world.coordinator import Coordinator, DroneView
from mission.world.mission_state import MissionState
from mission.world.taskboard import Role, TaskBoard, Track

pytestmark = pytest.mark.p10

_ROOT = Path(__file__).resolve().parent.parent
C2_HTML = _ROOT / "c2" / "index.html"
FOOTPRINTS = [(5.0, 3.0, 0.9, 0.9)]


def _bridge(coord=None):
    st, tb = MissionState(), TaskBoard()
    belief = BeliefGrid(Rect(0, 0, 10, 6), footprints=FOOTPRINTS, cell_size=0.5)
    bridge = C2Bridge(st, tb, belief, coord, footprints=FOOTPRINTS,
                      zones=[[(0, 0), (10, 0), (10, 2), (0, 2)]],
                      all_ids=[20, 21, 22, 23, 24])
    return bridge, st, tb, belief


def _drones(over=False, batt=(90, 90, 90), uwb=(True, True, True)):
    xy1 = (5.0, 3.0) if over else (5.0, 1.0)               # (5,3) is inside the footprint
    return [{"tag_id": 0, "xy": (2.0, 1.0), "alt_m": 1.1, "battery_pct": batt[0],
             "state": "SEARCH", "uwb_ok": uwb[0]},
            {"tag_id": 1, "xy": xy1, "alt_m": 1.1, "battery_pct": batt[1],
             "state": "SEARCH", "uwb_ok": uwb[1]},
            {"tag_id": 2, "xy": (8.0, 5.0), "alt_m": 1.1, "battery_pct": batt[2],
             "state": "LANDED", "uwb_ok": uwb[2]}]


# --------------------------------------------------------------------------- #
# snapshot / JSON
# --------------------------------------------------------------------------- #
def test_snapshot_is_valid_json_with_expected_keys():
    bridge, st, tb, _ = _bridge()
    tb.see(Track(22, (3.0, 3.0), 1.0, behaviour="smooth"))
    txt = bridge.to_json(_drones(), clock=12.5)
    s = json.loads(txt)                                   # round-trips → valid JSON
    assert set(s) >= {"clock", "drones", "footprints", "zones", "tracks", "belief",
                      "scoreboard", "alarms"}
    assert s["clock"] == 12.5
    assert len(s["drones"]) == 3
    assert s["belief"] and isinstance(s["belief"][0], list)   # heatmap matrix


# --------------------------------------------------------------------------- #
# alarms
# --------------------------------------------------------------------------- #
def test_over_footprint_alarm_fires_on_violating_pose():
    bridge, *_ = _bridge()
    alarms = bridge.alarms(_drones(over=True))
    over = [a for a in alarms if a["type"] == "over_footprint"]
    assert over and over[0]["tag_id"] == 1 and over[0]["severity"] == "critical"
    # and the clean pose does NOT trip it
    assert not any(a["type"] == "over_footprint" for a in bridge.alarms(_drones(over=False)))


def test_low_battery_and_uwb_dropout_alarms():
    bridge, *_ = _bridge()
    alarms = bridge.alarms(_drones(batt=(90, 15, 90), uwb=(True, True, False)))
    types = {a["type"] for a in alarms}
    assert "low_battery" in types and "uwb_dropout" in types


def test_near_collision_alarm():
    bridge, *_ = _bridge()
    drones = _drones()
    drones[0]["xy"] = (5.0, 1.0)
    drones[1]["xy"] = (5.2, 1.0)                           # 0.2 m apart < 0.6
    assert any(a["type"] == "near_collision" for a in bridge.alarms(drones))


# --------------------------------------------------------------------------- #
# scoreboard
# --------------------------------------------------------------------------- #
def test_scoreboard_reflects_state():
    bridge, st, tb, _ = _bridge()
    st.bank(22, None, (3, 3), 1.0)
    sc = bridge.scoreboard(_drones())
    assert sc["tagged_ids"] == [22]
    assert sc["outstanding_ids"] == [20, 21, 23, 24]
    assert sc["landings"] == 1                             # drone 2 state == LANDED


# --------------------------------------------------------------------------- #
# evidence export
# --------------------------------------------------------------------------- #
def test_evidence_export_produces_bundle(tmp_path):
    bridge, st, tb, _ = _bridge()
    frame = np.full((48, 48, 3), 120, np.uint8)
    st.bank(22, frame, (3.0, 3.0), 1.0)
    st.bank(23, frame, (6.0, 2.0), 2.0)
    manifest = bridge.export_evidence(tmp_path, drones=_drones(), clock=5.0)
    assert manifest["count"] == 2
    assert Path(manifest["results_sheet"]).exists()
    assert (tmp_path / "rover_22.png").exists() and (tmp_path / "rover_23.png").exists()
    assert Path(manifest["map"]).exists()
    assert "22" in Path(manifest["results_sheet"]).read_text(encoding="utf-8")


# --------------------------------------------------------------------------- #
# operator overrides reach the coordinator
# --------------------------------------------------------------------------- #
def _coord_drones():
    return [DroneView(0, (1.0, 1.0)), DroneView(1, (2.0, 2.0)), DroneView(2, (8.0, 5.0))]


def test_retask_override_reaches_coordinator():
    coord = Coordinator()
    bridge, st, tb, belief = _bridge(coord)
    ack = bridge.override({"type": "retask", "tag_id": 1, "role": "BLOCK", "target": [5.0, 3.0]})
    assert ack["ok"] is True
    out = coord.step(st, tb, belief, _coord_drones())
    assert out[1].role == Role.BLOCK and out[1].target == (5.0, 3.0)


def test_hold_all_and_resume_override():
    coord = Coordinator()
    bridge, st, tb, belief = _bridge(coord)
    bridge.override({"type": "hold_all"})
    out = coord.step(st, tb, belief, _coord_drones())
    assert all(a.role == Role.HOLD for a in out.values())
    bridge.override({"type": "resume"})
    out2 = coord.step(st, tb, belief, _coord_drones())
    assert all(a.role != Role.HOLD for a in out2.values())


def test_unknown_override_is_rejected_gracefully():
    coord = Coordinator()
    bridge, *_ = _bridge(coord)
    assert bridge.override({"type": "nonsense"})["ok"] is False


# --------------------------------------------------------------------------- #
# the console exists and targets the contract
# --------------------------------------------------------------------------- #
def test_c2_console_exists_and_reads_the_snapshot():
    assert C2_HTML.exists()
    html = C2_HTML.read_text(encoding="utf-8")
    assert "<canvas" in html
    assert "over_footprint" in html and "scoreboard" in html and "alarms" in html
    assert "state.json" in html          # polls the bridge snapshot
