"""P8 — adversarial evader handling: behaviour triage, reachable-set containment,
and coordinator hooks (secure-autonomous-first, pursuit roles, time-box rotation)."""

import math

import pytest

from mission.mission.phase2_search import (ReachableSet, classify_behaviour,
                                           plan_containment)
from mission.planner.geometry import Rect
from mission.world.belief_grid import BeliefGrid
from mission.world.coordinator import Coordinator, DroneView
from mission.world.mission_state import MissionState
from mission.world.taskboard import Role, TaskBoard, Track

pytestmark = pytest.mark.p8


# --------------------------------------------------------------------------- #
# behaviour triage
# --------------------------------------------------------------------------- #
def test_classify_smooth_path():
    pts = [(i * 0.5, 0.0) for i in range(8)]
    assert classify_behaviour(pts) == "smooth"


def test_classify_periodic_loop():
    pts = [(1 + math.cos(math.radians(a)), math.sin(math.radians(a)))
           for a in range(0, 340, 30)]
    assert classify_behaviour(pts) == "periodic"


def test_classify_erratic_zigzag():
    pts = [(0, 0), (0.5, 0.6), (1.0, 0.0), (1.5, 0.6), (2.0, 0.0), (2.5, 0.6), (3.0, 0.0)]
    assert classify_behaviour(pts) == "erratic"


# --------------------------------------------------------------------------- #
# reachable set / containment
# --------------------------------------------------------------------------- #
def _walled_map():
    # a wall across north=5 with a single gap at east≈3 (chokepoint)
    return [(5.0, 1.25, 0.6, 2.5), (5.0, 4.75, 0.6, 2.5)]


def test_containment_monotonically_shrinks():
    rs = ReachableSet(Rect(0, 0, 10, 6), _walled_map(), cell_size=0.25)
    rs.seed((2.0, 3.0))                                  # evader in the south room
    sizes = [rs.size()]
    for cp in [(5.0, 3.0), (8.0, 1.0), (2.0, 5.0)]:      # hold chokepoints one by one
        rs.cut(cp, radius_m=0.6)
        sizes.append(rs.size())
    assert all(sizes[i + 1] <= sizes[i] for i in range(len(sizes) - 1))   # monotonic
    assert sizes[1] < sizes[0]                           # the gap cut strictly shrank it


def test_cutting_gap_isolates_far_room():
    rs = ReachableSet(Rect(0, 0, 10, 6), _walled_map(), cell_size=0.25)
    rs.seed((2.0, 3.0))
    assert rs.contains((8.0, 3.0)) is True               # north reachable via the gap
    rs.cut((5.0, 3.0), radius_m=0.6)
    assert rs.contains((8.0, 3.0)) is False              # gap held → north cut off
    assert rs.contains((2.0, 3.0)) is True               # evader's room still reachable


def test_plan_containment_picks_bordering_chokepoints():
    rs = ReachableSet(Rect(0, 0, 10, 6), _walled_map(), cell_size=0.25)
    rs.seed((2.0, 3.0))                                  # south room reachable
    picks = plan_containment(rs, [(5.0, 3.0), (9.5, 5.5)], n_blockers=1)
    assert picks == [(5.0, 3.0)]                         # the gap borders the reachable set


# --------------------------------------------------------------------------- #
# coordinator: secure autonomous first, then evader
# --------------------------------------------------------------------------- #
def _scene():
    st, tb = MissionState(), TaskBoard()
    g = BeliefGrid(Rect(0, 0, 10, 6), cell_size=0.5)
    tb.see(Track(20, (2.0, 1.0), 1.0, behaviour="smooth"))
    tb.see(Track(21, (4.0, 5.0), 1.0, behaviour="periodic"))
    tb.see(Track(22, (8.0, 3.0), 1.0, behaviour="smooth"))
    tb.see(Track(99, (5.0, 3.0), 1.0, behaviour="erratic"))      # the evader (different id)
    drones = [DroneView(0, (1.0, 1.0)), DroneView(1, (4.0, 5.0)), DroneView(2, (8.0, 3.0))]
    return st, tb, g, drones


def test_secures_three_autonomous_before_committing_to_evader():
    st, tb, g, drones = _scene()
    coord = Coordinator()
    out = coord.step(st, tb, g, drones, chokepoints=[(5.0, 2.0), (5.0, 4.0)])
    assert all(a.role == Role.TAG for a in out.values())          # all chase autonomous
    assert all(a.role != Role.BLOCK for a in out.values())        # no containment yet
    assert (5.0, 3.0) not in {a.target for a in out.values()}     # evader not pursued


def test_commits_to_evader_after_autonomous_secured():
    st, tb, g, drones = _scene()
    coord = Coordinator()
    coord.step(st, tb, g, drones)                                 # tick 1: secure autonomous
    for mid in (20, 21, 22):
        st.bank(mid, None, None, 1.0)
    g.spike((5.0, 3.0))                                           # belief peaks on the evader
    out = coord.step(st, tb, g, drones, chokepoints=[(5.0, 2.0), (5.0, 4.0)])
    roles = [a.role for a in out.values()]
    assert Role.TAG in roles                                      # a pursuer chases the belief
    assert Role.BLOCK in roles                                    # others hold chokepoints


def test_time_box_rotation_prevents_deadlock():
    st, tb, g, drones = _scene()
    for mid in (20, 21, 22):
        st.bank(mid, None, None, 1.0)                             # only the evader remains
    g.spike((5.0, 3.0))
    coord = Coordinator()
    chokes = [(5.0, 2.0), (5.0, 4.0), (3.0, 3.0)]
    a = coord.step(st, tb, g, drones, chokepoints=chokes)
    b = coord.step(st, tb, g, drones, chokepoints=chokes)
    blocks_a = {x.target for x in a.values() if x.role == Role.BLOCK}
    blocks_b = {x.target for x in b.values() if x.role == Role.BLOCK}
    assert blocks_a and blocks_b
    assert blocks_a != blocks_b                                   # rotation breaks standoffs
