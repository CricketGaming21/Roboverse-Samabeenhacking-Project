"""P6 — shared world model: MissionState de-dup, TaskBoard tracks/assignments,
BeliefGrid collapse/diffuse/spike/chokepoints, Coordinator role assignment."""

import math

import numpy as np
import pytest

from mission.planner.geometry import Rect
from mission.world.belief_grid import BeliefGrid
from mission.world.coordinator import Coordinator, DroneView
from mission.world.mission_state import MissionState
from mission.world.taskboard import Role, TaskBoard, Track

pytestmark = pytest.mark.p6


# --------------------------------------------------------------------------- #
# MissionState — de-dup by id
# --------------------------------------------------------------------------- #
def test_mission_state_dedups_by_id():
    st = MissionState()
    assert st.bank(20, None, (1.0, 2.0), 1.0) is True
    assert st.bank(20, None, (1.1, 2.1), 2.0) is False     # same id → de-dup
    assert st.tagged() == {20}
    assert st.count() == 1


def test_mission_state_remaining():
    st = MissionState()
    st.bank(20, None, (0, 0), 1.0)
    st.bank(21, None, (0, 0), 1.0)
    assert st.remaining([20, 21, 22, 23, 24]) == {22, 23, 24}


# --------------------------------------------------------------------------- #
# TaskBoard
# --------------------------------------------------------------------------- #
def test_taskboard_dedups_and_estimates_velocity():
    tb = TaskBoard()
    tb.see(Track(marker_id=23, xy=(1.0, 1.0), t=0.0))
    tb.see(Track(marker_id=23, xy=(1.5, 1.0), t=1.0))      # moved +0.5 north in 1 s
    tracks = tb.tracks()
    assert len(tracks) == 1
    assert tracks[0].velocity == pytest.approx((0.5, 0.0))


def test_taskboard_assignments():
    tb = TaskBoard()
    tb.assign(1, Role.TAG, (5.0, 3.0))
    a = tb.assignment(1)
    assert a.role == Role.TAG and a.target == (5.0, 3.0)
    assert tb.assignment(2) is None


# --------------------------------------------------------------------------- #
# BeliefGrid
# --------------------------------------------------------------------------- #
def test_belief_excludes_footprint_cells():
    g = BeliefGrid(Rect(0, 0, 10, 6), footprints=[(5.0, 3.0, 1.0, 1.0)], cell_size=0.25)
    assert g.belief_at((5.0, 3.0)) == 0.0                  # inside footprint → no belief
    assert g.snapshot().sum() == pytest.approx(1.0)


def test_belief_collapses_where_observed():
    g = BeliefGrid(Rect(0, 0, 10, 6), cell_size=0.5)
    cells = g.cells_in_radius((5.0, 3.0), 0.6)
    g.observe(cells)
    assert g.belief_at((5.0, 3.0)) == 0.0
    assert g.snapshot().sum() == pytest.approx(1.0)        # renormalized


def test_belief_spike_becomes_argmax():
    g = BeliefGrid(Rect(0, 0, 10, 6), cell_size=0.5)
    g.spike((7.5, 1.5))
    assert g.argmax_region() == pytest.approx((7.5, 1.5), abs=0.5)


def test_belief_diffuses_over_time():
    g = BeliefGrid(Rect(0, 0, 10, 6), cell_size=0.25)
    g.spike((5.0, 3.0))
    peak0 = g.belief_at((5.0, 3.0))
    neigh0 = g.belief_at((5.5, 3.0))
    g.diffuse(dt=1.0, rover_speed=0.5)
    assert g.belief_at((5.0, 3.0)) < peak0                 # peak spreads out
    assert g.belief_at((5.5, 3.0)) > neigh0                # neighbour gains mass


def test_chokepoints_from_known_map():
    g = BeliefGrid(Rect(0, 0, 10, 6),
                   footprints=[(5.0, 2.5, 0.45, 0.45), (5.0, 3.5, 0.45, 0.45)])
    cps = g.chokepoints()
    assert any(math.hypot(p[0] - 5.0, p[1] - 3.0) < 0.2 for p in cps)


# --------------------------------------------------------------------------- #
# Coordinator
# --------------------------------------------------------------------------- #
def _scenario():
    st = MissionState()
    tb = TaskBoard()
    g = BeliefGrid(Rect(0, 0, 10, 6), cell_size=0.5)
    drones = [DroneView(0, (1.0, 1.0)), DroneView(1, (4.5, 3.0)), DroneView(2, (8.0, 5.0))]
    return st, tb, g, drones


def test_coordinator_assigns_tag_to_nearest_and_sweep_to_rest():
    st, tb, g, drones = _scenario()
    tb.see(Track(marker_id=23, xy=(5.0, 3.0), t=1.0))      # confirmed, untagged
    out = Coordinator().step(st, tb, g, drones)
    assert out[1].role == Role.TAG and out[1].target == (5.0, 3.0)   # drone 1 nearest
    assert out[0].role == Role.SWEEP and out[2].role == Role.SWEEP
    # mirrored into the taskboard
    assert tb.assignment(1).role == Role.TAG


def test_coordinator_skips_already_tagged():
    st, tb, g, drones = _scenario()
    tb.see(Track(marker_id=23, xy=(5.0, 3.0), t=1.0))
    st.bank(23, None, (5.0, 3.0), 1.0)                     # already scored
    out = Coordinator().step(st, tb, g, drones)
    assert all(a.role == Role.SWEEP for a in out.values())


def test_coordinator_leaves_locked_drone_untouched():
    st, tb, g, drones = _scenario()
    drones[1] = DroneView(1, (4.5, 3.0), busy_locked=True)  # mid lock-on (commitment)
    tb.see(Track(marker_id=23, xy=(5.0, 3.0), t=1.0))
    out = Coordinator().step(st, tb, g, drones)
    assert 1 not in out                                    # not re-tasked
    assert out[0].role == Role.TAG or out[2].role == Role.TAG   # TAG goes to a free drone
