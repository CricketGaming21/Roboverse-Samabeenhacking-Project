"""P7 — phase-2 search: lock_and_tag (servo/hold/time-box/interrupt), bubble-gate +
mop-up, and 3 drones tagging all distinct ids with no double-count / no overfly / ≤0.5 m/s."""

import math

import pytest

from mission.config import load_config
from mission.mission.phase2_search import (lock_and_tag, phase2_search,
                                           point_in_poly)
from mission.mission.worker import DroneWorker, WorkerState
from mission.control.avoidance import ReactiveGuard
from mission.perception.detector import Detection
from mission.planner.projection import CameraIntrinsics
from mission.world.mission_state import MissionState
from mission.world.taskboard import TaskBoard
from tests.fakes import fake_pyhulax as fpx
from tests.fakes import fake_uwb as fuwb
from tests.fakes.fake_pyhulax import CameraPitchMode, Marker

pytestmark = pytest.mark.p7

NOSLEEP = lambda _s: None
INTR = CameraIntrinsics(640, 480, 71.0)


def _loop(cx, cy, R=0.12, w=0.25):
    return lambda t: (cx + R * math.sin(w * t), cy + R * math.cos(w * t))


def _drone_over(world, n, e, alt_cm=110.0):
    fpx.set_active_world(world)
    d = fpx.FakeDroneAPI(world)
    d.connect(world.ip_map[0])
    d.n, d.e, d.alt_cm = n, e, alt_cm
    d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 90)
    d.set_video_stream(True)
    s = d.create_video_stream()
    s.start()
    return d, s


# --------------------------------------------------------------------------- #
# lock_and_tag
# --------------------------------------------------------------------------- #
def test_lock_and_tag_banks_stationary_rover():
    w = fpx.FakeWorld(crates=[])
    w.rovers = [Marker(20, 5.0, 3.0, size_m=0.20)]
    d, s = _drone_over(w, 5.0, 3.0)
    st = MissionState()
    ok = lock_and_tag(d, s, Detection((0, 0, 9, 9), 1.0, marker_id=20), st,
                      intrinsics=INTR, sleep=NOSLEEP)
    assert ok is True and st.is_tagged(20)
    fpx.set_active_world(None)


def test_lock_and_tag_velocity_matches_moving_rover():
    w = fpx.FakeWorld(crates=[])
    w.rovers = [Marker(20, 5.0, 3.0, size_m=0.20, motion=_loop(5.0, 3.0))]
    d, s = _drone_over(w, 5.0, 3.0)
    st = MissionState()
    ok = lock_and_tag(d, s, Detection((0, 0, 9, 9), 1.0, marker_id=20), st,
                      intrinsics=INTR, sleep=NOSLEEP)
    assert ok is True and st.is_tagged(20)
    fpx.set_active_world(None)


def test_lock_and_tag_is_time_boxed_when_target_absent():
    w = fpx.FakeWorld(crates=[])
    w.rovers = [Marker(20, 9.5, 5.5, size_m=0.20)]          # far outside the nadir FOV
    d, s = _drone_over(w, 2.0, 1.0)
    st = MissionState()
    ok = lock_and_tag(d, s, Detection((0, 0, 9, 9), 1.0, marker_id=20), st,
                      intrinsics=INTR, lock_timeout_s=0.5, sleep=NOSLEEP)
    assert ok is False and not st.is_tagged(20)            # bounded, no deadlock
    fpx.set_active_world(None)


def test_lock_and_tag_is_interruptible():
    w = fpx.FakeWorld(crates=[])
    w.rovers = [Marker(20, 5.0, 3.0, size_m=0.20)]
    d, s = _drone_over(w, 5.0, 3.0)
    st = MissionState()
    ok = lock_and_tag(d, s, Detection((0, 0, 9, 9), 1.0, marker_id=20), st,
                      intrinsics=INTR, should_stop=lambda: True, sleep=NOSLEEP)
    assert ok is False
    fpx.set_active_world(None)


# --------------------------------------------------------------------------- #
# bubble-gate + mop-up
# --------------------------------------------------------------------------- #
def test_point_in_poly():
    box = [(0, 0), (10, 0), (10, 2), (0, 2)]
    assert point_in_poly((5, 1), box) is True
    assert point_in_poly((5, 3), box) is False


def test_bubble_gate_logs_but_does_not_tag_out_of_zone():
    w = fpx.FakeWorld(crates=[])
    w.rovers = [Marker(21, 5.0, 3.0, size_m=0.20)]          # at e=3, OUTSIDE the e≤2 bubble
    fpx.set_active_world(w)
    d = fpx.FakeDroneAPI(w)
    d.connect(w.ip_map[0])
    d.takeoff(110)
    u = fuwb.FakeUWBParserThread(world=w)
    s = d.create_video_stream(); d.set_video_stream(True); s.start()
    st, tb = MissionState(), TaskBoard()
    bubble = [(0, 0), (10, 0), (10, 2), (0, 2)]
    banked = phase2_search(d, u, 0, [{"xy": (5.0, 3.0), "gimbal_deg": 90, "dwell_s": 0.6}],
                           s, st, tb, bubble=bubble, all_ids=[21], guard=None,
                           budget_cycles=1, mopup_extra_cycles=0, intrinsics=INTR,
                           sleep=NOSLEEP)
    assert 21 not in banked and not st.is_tagged(21)        # gated out of zone
    assert tb.track_for(21) is not None                    # but the sighting was logged
    fpx.set_active_world(None)


def test_mopup_drops_the_gate():
    w = fpx.FakeWorld(crates=[])
    w.rovers = [Marker(21, 5.0, 3.0, size_m=0.20)]
    fpx.set_active_world(w)
    d = fpx.FakeDroneAPI(w)
    d.connect(w.ip_map[0])
    d.takeoff(110)
    u = fuwb.FakeUWBParserThread(world=w)
    s = d.create_video_stream(); d.set_video_stream(True); s.start()
    st, tb = MissionState(), TaskBoard()
    bubble = [(0, 0), (10, 0), (10, 2), (0, 2)]
    banked = phase2_search(d, u, 0, [{"xy": (5.0, 3.0), "gimbal_deg": 90, "dwell_s": 0.6}],
                           s, st, tb, bubble=bubble, all_ids=[21], guard=None,
                           budget_cycles=1, mopup_extra_cycles=1, intrinsics=INTR,
                           sleep=NOSLEEP)
    assert 21 in banked and st.is_tagged(21)               # mop-up endgame tags it
    fpx.set_active_world(None)


# --------------------------------------------------------------------------- #
# 3 drones tag all distinct ids
# --------------------------------------------------------------------------- #
def _phase2_world():
    w = fpx.FakeWorld(crates=[(5.0, 5.5, 0.45, 0.45)])     # one crate in a corner
    w.rovers = [Marker(20, 2.0, 1.0, size_m=0.2, motion=_loop(2.0, 1.0)),
                Marker(21, 4.0, 1.0, size_m=0.2, motion=_loop(4.0, 1.0)),
                Marker(22, 3.0, 3.0, size_m=0.2, motion=_loop(3.0, 3.0)),
                Marker(23, 6.0, 3.0, size_m=0.2, motion=_loop(6.0, 3.0)),
                Marker(24, 8.0, 4.5, size_m=0.2, motion=_loop(8.0, 4.5))]
    w.pads = []
    w.drone_starts = {0: (0.6, 1.0), 1: (0.6, 3.0), 2: (0.6, 4.5)}
    return w


def test_three_drones_tag_all_distinct_ids():
    cfg = load_config()
    w = _phase2_world()
    fpx.set_active_world(w)
    footprints = [(5.0, 5.5, 0.45, 0.45)]
    u = fuwb.FakeUWBParserThread(world=w)
    st, tb = MissionState(), TaskBoard()
    all_ids = [20, 21, 22, 23, 24]
    vantages = {
        0: [{"xy": (2.0, 1.0), "gimbal_deg": 90}, {"xy": (4.0, 1.0), "gimbal_deg": 90}],
        1: [{"xy": (3.0, 3.0), "gimbal_deg": 90}, {"xy": (6.0, 3.0), "gimbal_deg": 90}],
        2: [{"xy": (8.0, 4.5), "gimbal_deg": 90}],
    }
    bubbles = {0: [(0, 0), (10, 0), (10, 2), (0, 2)],
               1: [(0, 2), (10, 2), (10, 4), (0, 4)],
               2: [(0, 4), (10, 4), (10, 6), (0, 6)]}

    workers = []
    for tag in (0, 1, 2):
        d = fpx.FakeDroneAPI(w)
        d.connect(w.ip_map[tag])
        s = d.create_video_stream(); d.set_video_stream(True); s.start()
        worker = DroneWorker(d, u, tag, cfg, guard=ReactiveGuard(),
                             footprints=footprints, priority=tag, sleep=NOSLEEP)
        worker.run_phase2(vantages[tag], s, st, tb, bubble=bubbles[tag], all_ids=all_ids,
                          intrinsics=INTR, budget_cycles=2, mopup_extra_cycles=0,
                          dwell_s=0.6)
        workers.append(worker)

    assert st.tagged() == set(all_ids)
    assert st.count() == 5                                  # no double-count
    dt = 1.0 / cfg.speed.ctrl_rate_hz
    for wkr in workers:
        for i in range(1, len(wkr.trace)):
            step = math.hypot(wkr.trace[i][0] - wkr.trace[i - 1][0],
                              wkr.trace[i][1] - wkr.trace[i - 1][1])
            assert step <= 0.5 * dt + 1e-6                  # ≤0.5 m/s throughout
        for p in wkr.trace:
            assert not (abs(p[0] - 5.0) < 0.225 and abs(p[1] - 5.5) < 0.225)  # no overfly
    fpx.set_active_world(None)


def test_worker_phase2_states():
    cfg = load_config()
    w = _phase2_world()
    fpx.set_active_world(w)
    d = fpx.FakeDroneAPI(w)
    d.connect(w.ip_map[0])
    s = d.create_video_stream(); d.set_video_stream(True); s.start()
    u = fuwb.FakeUWBParserThread(world=w)
    st, tb = MissionState(), TaskBoard()
    worker = DroneWorker(d, u, 0, cfg, sleep=NOSLEEP)
    worker.run_phase2([{"xy": (2.0, 1.0), "gimbal_deg": 90}], s, st, tb,
                      bubble=[(0, 0), (10, 0), (10, 2), (0, 2)], all_ids=[20],
                      intrinsics=INTR, budget_cycles=1, dwell_s=0.6)
    assert WorkerState.RELAUNCH in worker.history
    assert WorkerState.SEARCH in worker.history
    assert worker.history[-1] == WorkerState.DONE
    fpx.set_active_world(None)
