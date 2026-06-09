"""P3 — reactive guard (never +up, stop blocked axis, slide open side, boxed) +
separation + a plan-then-guard integration (surprise obstacle, no footprint entry)."""

import itertools
import math

import pytest

from mission.control.avoidance import ReactiveGuard, separation
from mission.control.uwb_loop import fly_to_uwb
from tests.fakes import fake_pyhulax as fpx
from tests.fakes.fake_pyhulax import Obstacles

pytestmark = pytest.mark.p3

NOSLEEP = lambda _s: None


def obs(forward=False, back=False, left=False, right=False, down=False):
    return Obstacles(forward, back, left, right, down)


# --------------------------------------------------------------------------- #
# never +up — the cardinal invariant
# --------------------------------------------------------------------------- #
def test_guard_never_emits_up():
    g = ReactiveGuard()
    cmds = [-1.0, -0.3, 0.0, 0.4, 1.0]
    for f, fb, l, r, d in itertools.product([False, True], repeat=5):
        for cf, cr in itertools.product(cmds, cmds):
            _, _, up = g.filter(cf, cr, obs(f, fb, l, r, d))
            assert up <= 0.0
            assert up == 0.0


# --------------------------------------------------------------------------- #
# stop the blocked travel axis
# --------------------------------------------------------------------------- #
def test_stops_forward_into_obstacle():
    g = ReactiveGuard()
    fwd, right, _ = g.filter(1.0, 0.0, obs(forward=True))
    assert fwd == 0.0


def test_stops_back_into_obstacle():
    g = ReactiveGuard()
    fwd, _, _ = g.filter(-1.0, 0.0, obs(back=True))
    assert fwd == 0.0


def test_zeroes_lateral_into_blocked_side():
    g = ReactiveGuard()
    _, right, _ = g.filter(0.0, 1.0, obs(right=True))     # commanding east into a wall
    assert right == 0.0
    g2 = ReactiveGuard()
    _, right2, _ = g2.filter(0.0, -1.0, obs(left=True))   # commanding west into a wall
    assert right2 == 0.0


def test_passes_through_when_clear():
    g = ReactiveGuard()
    assert g.filter(0.8, -0.2, obs()) == (0.8, -0.2, 0.0)


# --------------------------------------------------------------------------- #
# slide toward the open side
# --------------------------------------------------------------------------- #
def test_slides_left_when_right_blocked():
    g = ReactiveGuard()
    fwd, right, _ = g.filter(1.0, 0.0, obs(forward=True, right=True))  # only left open
    assert fwd == 0.0 and right < 0.0
    assert g.boxed() is False


def test_slides_right_when_left_blocked():
    g = ReactiveGuard()
    fwd, right, _ = g.filter(1.0, 0.0, obs(forward=True, left=True))   # only right open
    assert fwd == 0.0 and right > 0.0


def test_hint_chooses_side_when_both_open():
    g = ReactiveGuard()
    _, right, _ = g.filter(1.0, 0.0, obs(forward=True), open_side_hint="left")
    assert right < 0.0


def test_reports_boxed_when_surrounded():
    g = ReactiveGuard()
    fwd, right, up = g.filter(1.0, 0.0, obs(forward=True, left=True, right=True))
    assert (fwd, right, up) == (0.0, 0.0, 0.0)
    assert g.boxed() is True


# --------------------------------------------------------------------------- #
# separation / right-of-way
# --------------------------------------------------------------------------- #
def test_lower_priority_yields():
    others = [((0.3, 0.0), 5)]                            # higher-priority neighbour, near
    assert separation((0.0, 0.0), others, my_priority=1, sep_m=0.8) is True


def test_higher_priority_does_not_yield():
    others = [((0.3, 0.0), 1)]                            # lower-priority neighbour
    assert separation((0.0, 0.0), others, my_priority=5, sep_m=0.8) is False


def test_no_yield_when_out_of_range():
    others = [((3.0, 0.0), 9)]                            # higher priority but far
    assert separation((0.0, 0.0), others, my_priority=1, sep_m=0.8) is False


# --------------------------------------------------------------------------- #
# integration: surprise obstacle on the path — reach goal, never enter footprint
# --------------------------------------------------------------------------- #
def test_guarded_flight_avoids_surprise_obstacle():
    # a lone surprise crate the planner never knew about, dead ahead of the route
    w = fpx.FakeWorld(crates=[(5.0, 1.0, 0.6, 0.6)])      # footprint n[4.7,5.3] e[0.7,1.3]
    fpx.set_active_world(w)
    d = fpx.FakeDroneAPI(w)
    d.connect(w.ip_map[0])
    d.n, d.e = 3.0, 1.0
    d.takeoff(110)
    from tests.fakes import fake_uwb as fuwb
    u = fuwb.FakeUWBParserThread(world=w)
    guard = ReactiveGuard()

    traj, ups = [], []

    def rec(info):
        traj.append((d.n, d.e))
        ups.append(info.get("up", 0.0))

    ok = fly_to_uwb(d, u, 0, (7.0, 2.5), guard=guard, sleep=NOSLEEP,
                    max_steps=4000, on_step=rec)
    assert ok is True
    # never inside the surprise footprint (strict interior)
    for n, e in traj:
        assert not (4.7 < n < 5.3 and 0.7 < e < 1.3), f"entered footprint at {(n, e)}"
    # the guard never climbed
    assert max(ups) <= 0.0
    fpx.set_active_world(None)
