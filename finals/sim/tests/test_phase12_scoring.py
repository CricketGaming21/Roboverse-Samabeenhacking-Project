"""Phase 12 acceptance test — two-part scoring (headless).

Part 1 (DEPLOY): landing-accuracy referee — valid+designated pad within
tolerance, one drone per pad, attempts recorded with distance/time.
Part 2 (AMBUSH): the existing snapshot gate, ROVER ids only, never pads,
never outside AMBUSH. Nothing on the public pyhulax/UWB surface.
"""

import time

import pytest

from pyhulax import DroneAPI

from simcore.config import load_config
from simcore.registry import get_registry, shutdown_registry
from simcore.scoring import BankedID, format_combined_scoreboard

TOL = 0.30  # scoring.landing.tolerance_m (asserted against config below)


def _cfg(**overrides):
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0
    c.camera.use_egl = False
    c.scoring.enabled = False  # part-1 scorer must work without the camera referee
    c.motion.realistic = False  # crisp snap motion: exact geometry under test
    for key, value in overrides.items():
        setattr(c.scoring.landing, key, value)
    return c


def _connect(cfg, index=0) -> DroneAPI:
    d = DroneAPI()
    d.connect(cfg.drones.units[index].ip)
    return d


def _land_at(d, cfg, index, north, east):
    """Canned hop: fly drone <index> to an arena point and land there.

    Re-takeoffs re-anchor the takeoff-origin frame at the current spot, so
    the move_to target is computed against the LIVE frame, not the config
    start (simcore.frames is fine here — tests are sim-internal)."""
    from simcore import frames
    reg, drone = d._reg, d._drone
    if not reg.run_on_sim_thread(lambda: drone.flying):
        d.takeoff(100)
    fr = reg.run_on_sim_thread(lambda: drone.takeoff_frame)
    x_cm, y_cm, _ = frames.arena_to_takeoff_cm(cfg, fr, north, east)
    d.move_to(x_cm, y_cm, 100)
    return d.land()


# --------------------------------------------------------------------------- #
# Part 1 — landing accuracy
# --------------------------------------------------------------------------- #

def test_landing_on_designated_pad_scores_with_distance_and_time():
    cfg = _cfg()
    assert cfg.scoring.landing.tolerance_m == TOL
    reg = get_registry(cfg)
    try:
        pad10 = cfg.pads[0]
        d0 = _connect(cfg, 0)
        _land_at(d0, cfg, 0, pad10.north, pad10.east)  # dead centre
        t_land = reg.sim_time()
        assert reg.landing_scorer.score() == 1
        r = reg.landing_scorer.results()[0]
        assert (r.drone_index, r.pad_id, r.scored) == (0, 10, True)
        assert r.pad_valid and r.pad_designated
        assert r.error_m < 0.02                       # landed dead centre
        assert 0 < r.sim_time <= t_land               # time-to-land recorded

        pad11 = cfg.pads[1]
        d1 = _connect(cfg, 1)
        _land_at(d1, cfg, 1, pad11.north, pad11.east + 0.20)  # 20 cm off
        r1 = next(r for r in reg.landing_scorer.results()
                  if r.drone_index == 1)
        assert r1.pad_id == 11 and r1.scored
        assert r1.error_m == pytest.approx(0.20, abs=0.03)  # correct distance
        assert reg.landing_scorer.score() == 2
    finally:
        shutdown_registry()


def test_invalid_nondesignated_and_offpad_score_nothing():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        d = _connect(cfg, 0)
        _land_at(d, cfg, 0, 5.5, 1.2)     # pad 13: the INVALID decoy
        _land_at(d, cfg, 0, 2.0, 1.5)     # pad 14: valid but NOT designated
        _land_at(d, cfg, 0, 7.0, 1.5)     # open floor: outside any tolerance
        assert reg.landing_scorer.score() == 0
        attempts = reg.landing_scorer.all_attempts()
        assert len(attempts) == 3         # every attempt recorded, none scored
        assert [a.pad_id for a in attempts[:2]] == [13, 14]
        assert attempts[0].pad_valid is False
        assert attempts[1].pad_valid and not attempts[1].pad_designated
        assert attempts[2].error_m > cfg.scoring.landing.tolerance_m
        assert all(not a.scored for a in attempts)
    finally:
        shutdown_registry()


def test_one_drone_per_pad_and_one_pad_per_drone():
    cfg = _cfg()
    reg = get_registry(cfg)
    try:
        pad10 = cfg.pads[0]
        d0 = _connect(cfg, 0)
        _land_at(d0, cfg, 0, pad10.north, pad10.east)
        assert reg.landing_scorer.score() == 1

        d1 = _connect(cfg, 1)             # second drone, same pad: claimed
        _land_at(d1, cfg, 1, pad10.north + 0.1, pad10.east)
        assert reg.landing_scorer.score() == 1
        assert 1 not in {r.drone_index for r in reg.landing_scorer.results()}

        pad11 = cfg.pads[1]               # ...but a fresh pad scores
        _land_at(d1, cfg, 1, pad11.north, pad11.east)
        assert reg.landing_scorer.score() == 2

        _land_at(d0, cfg, 0, cfg.pads[2].north, cfg.pads[2].east)
        assert reg.landing_scorer.score() == 2  # one pad per drone: no double
    finally:
        shutdown_registry()


def test_fixed_assignment_mode():
    cfg = _cfg(assignment="fixed")
    cfg.drones.units[0].pad_id = 11
    reg = get_registry(cfg)
    try:
        d0 = _connect(cfg, 0)
        pad10, pad11 = cfg.pads[0], cfg.pads[1]
        _land_at(d0, cfg, 0, pad10.north, pad10.east)  # perfect — wrong pad
        assert reg.landing_scorer.score() == 0         # assigned pad 11
        _land_at(d0, cfg, 0, pad11.north, pad11.east)
        assert reg.landing_scorer.score() == 1
        assert reg.landing_scorer.results()[0].pad_id == 11
    finally:
        shutdown_registry()


def test_landings_do_not_score_outside_deploy():
    cfg = _cfg()
    cfg.scenario.phases = "ambush"        # episode starts beyond DEPLOY
    cfg.rovers.motion = "patrol"
    cfg.rovers.patrol.speed_mps = 0.0
    reg = get_registry(cfg)
    try:
        pad10 = cfg.pads[0]
        d = _connect(cfg, 0)
        _land_at(d, cfg, 0, pad10.north, pad10.east)   # perfect landing...
        assert reg.landing_scorer.score() == 0         # ...but not DEPLOY
        assert reg.landing_scorer.all_attempts() == []  # not even recorded
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Part 2 — rover snapshots only, AMBUSH only
# --------------------------------------------------------------------------- #

def test_snapshots_gate_by_phase_and_never_score_pads():
    cfg = load_config("sim_config.yaml")
    cfg.meta.real_time_factor = 10.0
    cfg.arena.layout = "procedural"       # clean sight lines
    cfg.arena.obstacles.count = 0
    cfg.rovers.motion = "patrol"
    cfg.rovers.patrol.speed_mps = 0.0     # parked targets
    cfg.motion.realistic = False          # crisp: exact hover geometry
    cfg.scenario.ambush_trigger.mode = "manual_key"
    cfg.scenario.ambush_trigger.delay_s = 0.2
    reg = get_registry(cfg)
    try:
        from pyhulax.core import CameraPitchMode
        pad10 = cfg.pads[0]
        start = cfg.drones.units[0].start
        d = _connect(cfg, 0)
        d.takeoff(150)
        d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 90)
        d.move_to((pad10.east - start[1]) * 100.0,
                  (pad10.north - start[0]) * 100.0 - 10.0, 150)
        time.sleep(0.6)                   # ~6 sim s staring at pad 10...
        assert reg.scenario.phase == "deploy"
        assert reg.referee.banked_ids() == set()   # DEPLOY: judging is off

        reg.scenario.request_manual_trigger()      # operator starts AMBUSH
        deadline = time.time() + 3.0
        while time.time() < deadline and reg.scenario.phase != "ambush":
            time.sleep(0.01)
        time.sleep(0.5)                   # ~5 sim s staring at pad 10 again
        assert reg.referee.banked_ids() == set()   # pads are NOT targets

        rn, re_ = reg.rover_arena_positions()[0]   # now a real target
        rover_id = reg.rovers[0].marker_id
        d.move_to((re_ - start[1]) * 100.0,
                  (rn - start[0]) * 100.0 - 10.0, 150)
        deadline = time.time() + 8.0
        while time.time() < deadline:
            if rover_id in reg.referee.banked_ids():
                break
            time.sleep(0.02)
        assert reg.referee.banked_ids() == {rover_id}  # rover scored, pads never
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Combined scoreboard + surface freeze
# --------------------------------------------------------------------------- #

def test_combined_scoreboard_lists_both_parts():
    cfg = _cfg()
    cfg.scoring.enabled = True
    reg = get_registry(cfg)
    try:
        pad10 = cfg.pads[0]
        d = _connect(cfg, 0)
        _land_at(d, cfg, 0, pad10.north, pad10.east + 0.1)
        rover_id = reg.rovers[0].marker_id

        def _inject():  # white-box: one banked rover snapshot
            with reg.referee._lock:
                reg.referee._banked[rover_id] = BankedID(rover_id, 2, 42.0)
        reg.run_on_sim_thread(_inject)

        board = format_combined_scoreboard(reg)
        assert "PART 1" in board and "PART 2" in board
        assert "drone 0 -> pad 10" in board
        assert "10.0cm" in board          # the 10 cm landing error
        assert f"id  {rover_id}" in board
        assert "drone 2" in board
    finally:
        shutdown_registry()


def test_nothing_added_to_public_surface():
    import pyhulax
    import pyhulax.core
    from UWBParserThread import UWBParserThread
    public = (set(dir(pyhulax)) | set(dir(pyhulax.DroneAPI))
              | set(dir(pyhulax.core)) | set(dir(UWBParserThread)))
    leaked = [n for n in public if "scor" in n.lower()
              or "referee" in n.lower() or "landing" in n.lower()]
    assert leaked == [], f"scoring leaked into the public surface: {leaked}"