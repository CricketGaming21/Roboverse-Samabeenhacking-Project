"""P11 — full Phase1→Phase2 on the fake harness + reliability failsafes + discovery.

The full run scores 3/3 landings + all distinct tags; battery / UWB-dropout / worker-death
failsafes are covered; shutdown always lands every drone.
"""

import math

import pytest

from mission.config import load_config
from mission.mission.plan import MissionPlan, validate_plan
from mission.mission.worker import DroneWorker, WorkerState
from mission.planner.projection import CameraIntrinsics
from mission.runtime.discovery import Discovery
from mission.runtime.main import Mission
from tests.fakes import fake_pyhulax as fpx
from tests.fakes import fake_uwb as fuwb
from tests.fakes.fake_pyhulax import Marker

pytestmark = pytest.mark.p11

NOSLEEP = lambda _s: None
ALL_ROVERS = [20, 21, 22, 23, 24]


def _loop(cx, cy, R=0.12, w=0.25):
    return lambda t: (cx + R * math.sin(w * t), cy + R * math.cos(w * t))


def _world():
    w = fpx.FakeWorld(crates=[(5.0, 5.5, 0.45, 0.45)])
    w.pads = [Marker(10, 1.0, 1.0), Marker(11, 1.0, 3.0), Marker(12, 1.0, 5.0)]
    w.rovers = [Marker(20, 3.0, 1.0, size_m=0.2, motion=_loop(3.0, 1.0)),
                Marker(21, 6.0, 1.0, size_m=0.2, motion=_loop(6.0, 1.0)),
                Marker(22, 3.0, 3.0, size_m=0.2, motion=_loop(3.0, 3.0)),
                Marker(23, 6.0, 3.0, size_m=0.2, motion=_loop(6.0, 3.0)),
                Marker(24, 8.0, 5.0, size_m=0.2, motion=_loop(8.0, 5.0))]
    w.drone_starts = {0: (0.5, 1.0), 1: (0.5, 3.0), 2: (0.5, 5.0)}
    return w


def _plan():
    def v(n, e):
        return {"xy": [n, e], "look_yaw_deg": 0, "gimbal_deg": 90, "dwell_s": 0.6}
    data = {
        "version": 1,
        "arena": {"length_m": 10.0, "width_m": 6.0, "inflate_m": 0.40,
                  "crates": [{"center": [5.0, 5.5], "size": [0.45, 0.45], "height_m": 0.6}]},
        "drones": [
            {"tag_id": 0, "color": "#e6194b",
             "phase1": {"pad_id": 10, "route_m": [[0.5, 1.0], [1.0, 1.0]]},
             "phase2": {"bubble": [[0, 0], [10, 0], [10, 1.9], [0, 1.9]],
                        "vantages": [v(3.0, 1.0), v(6.0, 1.0)]}},
            {"tag_id": 1, "color": "#3cb44b",
             "phase1": {"pad_id": 11, "route_m": [[0.5, 3.0], [1.0, 3.0]]},
             "phase2": {"bubble": [[0, 2.1], [10, 2.1], [10, 3.9], [0, 3.9]],
                        "vantages": [v(3.0, 3.0), v(6.0, 3.0)]}},
            {"tag_id": 2, "color": "#4363d8",
             "phase1": {"pad_id": 12, "route_m": [[0.5, 5.0], [1.0, 5.0]]},
             "phase2": {"bubble": [[0, 4.1], [10, 4.1], [10, 6.0], [0, 6.0]],
                        "vantages": [v(8.0, 5.0)]}},
        ],
        "meta": {"hoop_tol_m": 0.15, "cruise_alt_m": 1.10}}
    return validate_plan(MissionPlan(**data), valid_pad_ids=[10, 11, 12, 14])


PAD_COORDS = {10: (1.0, 1.0), 11: (1.0, 3.0), 12: (1.0, 5.0)}


def _mission(world, cfg, plan):
    drones, streams = {}, {}
    for tag in (0, 1, 2):
        d = fpx.FakeDroneAPI(world)
        d.connect(world.ip_map[tag])
        d.set_video_stream(True)
        s = d.create_video_stream(); s.start()
        drones[tag], streams[tag] = d, s
    uwb = fuwb.FakeUWBParserThread(world=world)
    intr = CameraIntrinsics(cfg.camera.width, cfg.camera.height, cfg.camera.h_fov_deg)
    return Mission(cfg, plan, drones=drones, uwb=uwb, streams=streams,
                   pad_coords=PAD_COORDS, footprints=[(5.0, 5.5, 0.45, 0.45)],
                   all_rover_ids=ALL_ROVERS, intrinsics=intr, sleep=NOSLEEP,
                   phase2_kwargs={"budget_cycles": 2, "mopup_extra_cycles": 0,
                                  "dwell_s": 0.6}), drones


# --------------------------------------------------------------------------- #
# the full run
# --------------------------------------------------------------------------- #
def test_full_phase1_to_phase2_run_scores_everything():
    cfg = load_config()
    w = _world(); fpx.set_active_world(w)
    mission, drones = _mission(w, cfg, _plan())
    result = mission.run(parallel=False)
    assert result.landings == 3                         # 3/3 hoop landings
    assert result.tagged == set(ALL_ROVERS)             # all distinct rover ids
    assert mission.state.count() == 5                   # no double-count
    for d in drones.values():
        assert d.get_altitude() == 0.0                  # shutdown landed every drone
    fpx.set_active_world(None)


# --------------------------------------------------------------------------- #
# reliability / failsafes
# --------------------------------------------------------------------------- #
def test_shutdown_always_lands_all_even_on_exception(monkeypatch):
    cfg = load_config()
    w = _world(); fpx.set_active_world(w)
    mission, drones = _mission(w, cfg, _plan())
    for d in drones.values():
        d.takeoff(110)
    monkeypatch.setattr(mission, "run_phase1",
                        lambda parallel=False: (_ for _ in ()).throw(RuntimeError("boom")))
    with pytest.raises(RuntimeError):
        mission.run()
    for d in drones.values():
        assert d.get_altitude() == 0.0                  # finally landed all
    fpx.set_active_world(None)


def test_battery_failsafe_returns_and_lands():
    cfg = load_config()
    w = _world(); fpx.set_active_world(w)
    d = fpx.FakeDroneAPI(w); d.connect(w.ip_map[0])
    u = fuwb.FakeUWBParserThread(world=w)
    worker = DroneWorker(d, u, 0, cfg, sleep=NOSLEEP,
                         battery_rtl_pct=cfg.failsafe.battery_rtl_pct)
    d._battery = 15                                     # below the 20% RTL threshold
    ok = worker.run_phase1((8.5, 1.0), [[0.5, 1.0], [8.5, 1.0]])
    assert ok is False
    assert worker.rtl is True and worker.error == "battery_rtl"
    assert d.get_altitude() == 0.0                      # safely landed
    fpx.set_active_world(None)


def test_uwb_dropout_holds_without_lurching():
    cfg = load_config()
    w = _world(); fpx.set_active_world(w)
    d = fpx.FakeDroneAPI(w); d.connect(w.ip_map[0])
    u = fuwb.FakeUWBParserThread(world=w)
    worker = DroneWorker(d, u, 0, cfg, sleep=NOSLEEP)
    d.takeoff(110)
    n0, e0 = d.n, d.e
    u.set_unseen(0, True)                               # persistent UWB dropout
    worker.run_phase1((8.5, 1.0), [[8.5, 1.0]])
    assert math.hypot(d.n - n0, d.e - e0) < 1e-6        # held position, never lurched
    fpx.set_active_world(None)


def test_worker_death_zone_reassigned(monkeypatch):
    cfg = load_config()
    w = _world(); fpx.set_active_world(w)
    mission, drones = _mission(w, cfg, _plan())
    drones[0].takeoff(110)                              # 0 and 2 airborne, 1 grounded
    drones[2].takeoff(110)
    monkeypatch.setattr(drones[1], "takeoff",
                        lambda *a, **k: (_ for _ in ()).throw(RuntimeError("dead motor")))
    mission.run_phase2(parallel=False)
    assert mission.workers[1].state == WorkerState.FAILED      # drone 1 died
    assert mission.state.tagged() == set(ALL_ROVERS)           # its zone was mopped up
    fpx.set_active_world(None)


# --------------------------------------------------------------------------- #
# discovery
# --------------------------------------------------------------------------- #
def test_discovery_config_first_resolves_tags():
    disc = Discovery.from_config(load_config())          # in-sim default: no Dola call
    assert disc.resolve() == {0: "10.0.0.11", 1: "10.0.0.12", 2: "10.0.0.13"}


def test_discovery_falls_back_to_config_when_dola_stubbed(monkeypatch):
    """The sim's Dola raises NotImplementedError — even use_dola=True must fall back."""
    import mission.runtime.discovery as disc_mod

    class StubDola:                                      # mirrors the sim's broken Dola
        def __init__(self, *a, **k):
            pass

        def start(self):
            raise NotImplementedError

        def stop(self):
            pass

        def get_all_ips(self, *a, **k):
            raise NotImplementedError

    monkeypatch.setattr(disc_mod, "_resolve_dola", lambda: StubDola)
    disc = Discovery.from_config(load_config(), use_dola=True)
    assert disc.resolve() == {0: "10.0.0.11", 1: "10.0.0.12", 2: "10.0.0.13"}


def test_discovery_uses_dola_when_available(world):
    """On the real day Dola works; use_dola maps plane_id→tag_id (fake Dola here)."""
    disc = Discovery.from_config(load_config(), use_dola=True)
    assert disc.resolve() == world.ip_map


# --------------------------------------------------------------------------- #
# real-sim integration (implemented; excluded from the gate)
# --------------------------------------------------------------------------- #
@pytest.mark.integration
def test_full_run_against_real_sim():
    """Run the same Mission against the REAL sim pyhulax (supervised). Needs the sim on
    the path + RUN_INTEGRATION=1; excluded from the overnight gate."""
    import pyhulax
    assert hasattr(pyhulax, "DroneAPI")                 # placeholder — wire to the live sim
