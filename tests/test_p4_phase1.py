"""P4 — pad assignment + land-in-hoop + the per-drone worker (3-of-5 deploy)."""

import math

import pytest

from mission.config import PadCfg, load_config
from mission.mission.phase1_land import assign_pads, land_in_hoop
from mission.mission.worker import DroneWorker, WorkerState
from mission.planner.geometry import Rect, build_graph, inflate
from tests.fakes import fake_pyhulax as fpx
from tests.fakes import fake_uwb as fuwb
from tests.fakes.fake_pyhulax import Marker

pytestmark = pytest.mark.p4

NOSLEEP = lambda _s: None


def _in_footprint(p, footprints):
    for cn, ce, sn, se in footprints:
        if abs(p[0] - cn) < sn / 2 and abs(p[1] - ce) < se / 2:
            return True
    return False


# --------------------------------------------------------------------------- #
# pad assignment
# --------------------------------------------------------------------------- #
def test_assign_pads_picks_correct_three():
    valid = [PadCfg(id=14, north=2.0, east=1.0, valid=True),
             PadCfg(id=12, north=2.0, east=5.0, valid=True),
             PadCfg(id=10, north=8.5, east=3.0, valid=True)]
    starts = {0: (0.6, 1.0), 1: (0.6, 3.0), 2: (0.6, 5.0)}
    out = assign_pads(valid, starts)
    assert {t: p.id for t, p in out.items()} == {0: 14, 1: 10, 2: 12}


def test_assign_only_uses_valid_pads_and_is_non_crossing():
    valid = [PadCfg(id=14, north=2.0, east=1.0, valid=True),
             PadCfg(id=12, north=2.0, east=5.0, valid=True),
             PadCfg(id=10, north=8.5, east=1.0, valid=True),
             PadCfg(id=11, north=8.5, east=5.0, valid=True)]
    starts = {0: (0.6, 1.0), 1: (0.6, 3.0), 2: (0.6, 5.0)}
    out = assign_pads(valid, starts)
    assert len({p.id for p in out.values()}) == 3            # distinct
    segs = [(starts[t], (p.north, p.east)) for t, p in out.items()]
    from mission.mission.phase1_land import _segments_cross
    for i in range(len(segs)):
        for j in range(i + 1, len(segs)):
            assert not _segments_cross(segs[i][0], segs[i][1],
                                       segs[j][0], segs[j][1])


def test_assign_raises_when_too_few_pads():
    valid = [PadCfg(id=14, north=2.0, east=1.0, valid=True)]
    with pytest.raises(ValueError):
        assign_pads(valid, {0: (0, 0), 1: (1, 1), 2: (2, 2)})


# --------------------------------------------------------------------------- #
# land_in_hoop
# --------------------------------------------------------------------------- #
def test_land_in_hoop_centres_and_lands_uwb_only(monkeypatch):
    # R1: Phase-1 lands on UWB ONLY — if any ArUco decode were attempted it would raise,
    # proving the landing path never touches cv2.aruco and the camera is never needed.
    import cv2

    def _boom(*a, **k):
        raise AssertionError("Phase-1 landing must not call cv2.aruco (UWB-only)")
    monkeypatch.setattr(cv2.aruco, "ArucoDetector", _boom)
    monkeypatch.setattr(cv2.aruco, "detectMarkers", _boom, raising=False)

    w = fpx.FakeWorld(crates=[])
    w.pads = [Marker(10, 8.5, 3.0)]
    fpx.set_active_world(w)
    d = fpx.FakeDroneAPI(w)
    d.connect(w.ip_map[0])
    d.n, d.e = 8.0, 2.6                                      # offset from the pad
    d.takeoff(110)
    u = fuwb.FakeUWBParserThread(world=w)
    ok = land_in_hoop(d, u, 0, (8.5, 3.0), hoop_tol_m=0.15, footprints=[], sleep=NOSLEEP)
    assert ok is True
    x, y, _ = u.get_tag_position(0)
    assert math.hypot(x - 8.5, y - 3.0) <= 0.15
    assert d.get_altitude() == 0.0                          # actually landed
    assert d.video_enabled is False                         # camera stayed off in Phase 1
    fpx.set_active_world(None)


def test_land_aborts_if_pad_over_footprint():
    w = fpx.FakeWorld(crates=[])
    fpx.set_active_world(w)
    d = fpx.FakeDroneAPI(w)
    d.connect(w.ip_map[0])
    d.n, d.e = 4.8, 3.0
    d.takeoff(110)
    u = fuwb.FakeUWBParserThread(world=w)
    # the pad sits on a crate footprint → descent column blocked → must refuse
    ok = land_in_hoop(d, u, 0, (5.0, 3.0), hoop_tol_m=0.15,
                      footprints=[(5.0, 3.0, 0.9, 0.9)], sleep=NOSLEEP)
    assert ok is False
    fpx.set_active_world(None)


# --------------------------------------------------------------------------- #
# worker: 3 drones deploy + land
# --------------------------------------------------------------------------- #
def _phase1_world():
    w = fpx.FakeWorld(crates=[(5.0, 3.0, 0.9, 0.9)])
    w.pads = [Marker(14, 2.0, 1.0), Marker(12, 2.0, 5.0),
              Marker(10, 8.5, 1.0), Marker(11, 8.5, 5.0)]
    w.drone_starts = {0: (0.6, 1.0), 1: (0.6, 3.0), 2: (0.6, 5.0)}
    w.rovers = []
    return w


def test_three_drones_deploy_and_land_in_hoops():
    cfg = load_config()
    w = _phase1_world()
    fpx.set_active_world(w)
    footprints = [(5.0, 3.0, 0.9, 0.9)]
    graph = build_graph(inflate(footprints, 0.4), Rect(0, 0, 10, 6))
    valid = [PadCfg(id=14, north=2.0, east=1.0, valid=True),
             PadCfg(id=12, north=2.0, east=5.0, valid=True),
             PadCfg(id=10, north=8.5, east=1.0, valid=True),
             PadCfg(id=11, north=8.5, east=5.0, valid=True)]
    assignment = assign_pads(valid, w.drone_starts)
    u = fuwb.FakeUWBParserThread(world=w)

    landed = 0
    for tag in (0, 1, 2):
        d = fpx.FakeDroneAPI(w)
        d.connect(w.ip_map[tag])
        worker = DroneWorker(d, u, tag, cfg, graph=graph, footprints=footprints,
                             priority=tag, sleep=NOSLEEP)
        pad = assignment[tag]
        ok = worker.run_phase1((pad.north, pad.east))
        assert ok is True, f"drone {tag} failed: {worker.error}"
        assert worker.history[-1] == WorkerState.LANDED
        # landed within the hoop
        x, y, _ = u.get_tag_position(tag)
        assert math.hypot(x - pad.north, y - pad.east) <= cfg.landing.hoop_tol_m
        # never overflew a footprint
        for p in worker.trace:
            assert not _in_footprint(p, footprints), f"drone {tag} overflew {p}"
        landed += 1
    assert landed == 3
    fpx.set_active_world(None)


def test_worker_walks_phase1_states():
    cfg = load_config()
    w = _phase1_world()
    fpx.set_active_world(w)
    d = fpx.FakeDroneAPI(w)
    d.connect(w.ip_map[0])
    u = fuwb.FakeUWBParserThread(world=w)
    worker = DroneWorker(d, u, 0, cfg, footprints=[(5.0, 3.0, 0.9, 0.9)], sleep=NOSLEEP)
    worker.run_phase1((2.0, 1.0))
    assert worker.history[:4] == [WorkerState.INIT, WorkerState.TAKEOFF,
                                  WorkerState.GO_TO_PAD, WorkerState.LAND_HOOP]
    assert worker.history[-1] == WorkerState.LANDED
    assert d.video_enabled is False        # R1: camera stays OFF through Phase 1
    fpx.set_active_world(None)
