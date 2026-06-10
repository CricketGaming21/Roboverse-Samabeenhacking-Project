"""Phase 34 acceptance test — inter-drone obstacle sensing + hoop / distinct-id
scoring + 8-min stage clock + drone-count knob (headless).

All sim/scenario/scoring-side; the public pyhulax/UWB surface is unchanged
(get_obstacles() still returns the five booleans — the world simply now also
contains the other drones as obstacle sources).
"""

import math
import time

import pytest

from pyhulax import DroneAPI

from scripts import scenario_demo as demo
from simcore import frames, rover_model
from simcore.config import load_config
from simcore.registry import get_registry, shutdown_registry


def _cfg():
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0
    c.camera.use_egl = False
    return c


# --------------------------------------------------------------------------- #
# Inter-drone obstacle sensing (drones are obstacles to each other)
# --------------------------------------------------------------------------- #

def test_second_drone_within_ir_range_trips_the_barrier_flag():
    c = _cfg()
    c.scoring.enabled = False
    c.arena.layout = "procedural"
    c.arena.obstacles.count = 0
    c.rovers.count = 0
    c.motion.realistic = False
    reg = get_registry(c)
    try:
        d0 = DroneAPI(); d0.connect(c.drones.units[0].ip); d0.takeoff(110)
        d1 = DroneAPI(); d1.connect(c.drones.units[1].ip); d1.takeoff(110)
        fr0 = reg.run_on_sim_thread(lambda: reg.drones[0].takeoff_frame)
        fr1 = reg.run_on_sim_thread(lambda: reg.drones[1].takeoff_frame)
        rng = c.barrier_sensors.range_m.forward

        # d0 at (3,3) facing +north; d1 just inside the IR range, dead ahead
        x, y, _ = frames.arena_to_takeoff_cm(c, fr0, 3.0, 3.0); d0.move_to(x, y, 110)
        x, y, _ = frames.arena_to_takeoff_cm(c, fr1, 3.0 + rng * 0.7, 3.0)
        d1.move_to(x, y, 110)
        time.sleep(0.3)
        assert d0.get_obstacles().forward is True       # trips on the other drone

        # move d1 well clear -> the flag clears
        x, y, _ = frames.arena_to_takeoff_cm(c, fr1, 8.0, 5.0); d1.move_to(x, y, 110)
        time.sleep(0.3)
        assert d0.get_obstacles().forward is False
        # public surface unchanged: still exactly the five booleans, no distances
        obs = d0.get_obstacles()
        for name in ("forward", "back", "left", "right", "down"):
            assert isinstance(getattr(obs, name), bool)
        assert not hasattr(obs, "distance_m")
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Hoop scoring: land inside the hoop -> scores; outside -> doesn't
# --------------------------------------------------------------------------- #

def _land_at(d, reg, cfg, north, east):
    drone = reg.drones[0]
    if not reg.run_on_sim_thread(lambda: drone.flying):
        d.takeoff(100)
    fr = reg.run_on_sim_thread(lambda: drone.takeoff_frame)
    x, y, _ = frames.arena_to_takeoff_cm(cfg, fr, north, east)
    d.move_to(x, y, 100)
    return d.land()


def test_hoop_scoring_inside_scores_outside_does_not():
    c = _cfg()
    c.scoring.enabled = False        # part-1 scorer works without the referee
    c.motion.realistic = False
    c.scoring.landing.hoop_radius_m = 0.25      # the configured hoop
    reg = get_registry(c)
    try:
        pad = c.pads[0]              # a valid+designated pad
        assert pad.valid and pad.designated
        # inside the hoop (0.15 m < 0.25) -> scores
        d0 = DroneAPI(); d0.connect(c.drones.units[0].ip)
        _land_at(d0, reg, c, pad.north, pad.east + 0.15)
        assert reg.landing_scorer.score() == 1
        r = reg.landing_scorer.results()[0]
        assert r.pad_id == pad.id and r.error_m == pytest.approx(0.15, abs=0.03)

        # another drone outside the hoop (0.40 m > 0.25) -> no score
        d1 = DroneAPI(); d1.connect(c.drones.units[1].ip)
        _land_at(d1, reg, c, pad.north, pad.east + 0.40)
        assert reg.landing_scorer.score() == 1               # unchanged
        miss = [a for a in reg.landing_scorer.attempts
                if a.drone_index == 1][-1]
        assert miss.scored is False and miss.error_m > c.scoring.landing.hoop_radius_m
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Distinct-id scoring across ALL 5 rovers (both id blocks)
# --------------------------------------------------------------------------- #

def test_distinct_id_scoring_counts_both_id_blocks():
    c = _cfg()
    c.rovers.motion = "mixed"
    c.scenario.phases = "ambush"
    c.scoring.enabled = True
    # freeze the evasive rovers so one can be scanned stably (no flee/roam)
    c.rovers.mixed.evasive.speed_mps = 0.0
    c.rovers.mixed.evasive.flee_radius_m = 0.0
    reg = get_registry(c)
    try:
        # the referee accepts BOTH blocks as part-2 targets (5 distinct ids)
        assert reg.referee._targets == {11, 45, 51, 67, 101}

        # scan an EVASIVE rover (id 67) up close -> it banks (the block counts)
        ev = reg.rovers[3]
        assert ev.marker_id == 67
        rn, re_ = reg.run_on_sim_thread(ev.arena_position)
        d = DroneAPI(); d.connect(c.drones.units[0].ip); d.takeoff(150)
        from pyhulax.core import CameraPitchMode
        fr = reg.run_on_sim_thread(lambda: reg.drones[0].takeoff_frame)
        x, y, _ = frames.arena_to_takeoff_cm(c, fr, rn, re_)
        d.move_to(x, y, 150)
        d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 90)
        deadline = time.time() + 8.0
        while time.time() < deadline and 67 not in reg.referee.banked_ids():
            time.sleep(0.05)
        assert 67 in reg.referee.banked_ids()       # an evasive id scored
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# 8-min stage clock
# --------------------------------------------------------------------------- #

def test_stage_budget_reflects_eight_minutes():
    c = load_config("sim_config.yaml")
    # ~8 min per stage (PROVISIONAL); episode caps the whole combined run
    assert c.scenario.deploy_timeout_s == pytest.approx(480.0)
    assert c.scenario.ambush_seconds == pytest.approx(480.0)
    assert c.scenario.episode_seconds >= 480.0


def test_episode_honours_the_cap():
    c = _cfg()
    c.scoring.enabled = False
    c.scenario.episode_seconds = 3.0          # a tight cap for the test
    c.scenario.ambush_seconds = 30.0          # so the EPISODE cap is what ends it
    c.scenario.ambush_trigger.mode = "timed"
    c.scenario.ambush_trigger.delay_s = 0.5
    reg = get_registry(c)
    try:
        deadline = time.time() + 6.0
        while time.time() < deadline and reg.scenario.phase != "done":
            time.sleep(0.02)
        assert reg.scenario.phase == "done"                  # reached the cap
        assert reg.sim_time() >= c.scenario.episode_seconds - 0.5
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Drone-count knob: the scenario runs with 1 OR 3 drones
# --------------------------------------------------------------------------- #

def _demo_cfg(n_drones):
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0
    c.scenario.episode_seconds = 45.0
    c.scenario.ambush_seconds = 35.0
    c.scenario.deploy_timeout_s = 25.0
    c.scenario.ambush_trigger.mode = "on_all_landed"
    c.scenario.ambush_trigger.delay_s = 1.0
    c.drones.units = c.drones.units[:n_drones]    # the count knob
    return c


def test_scenario_runs_with_three_drones():
    result = demo.run_scenario_demo(_demo_cfg(3))
    assert result["final_phase"] == "done"
    assert result["landing_score"] >= 1


def test_scenario_runs_with_one_drone():
    cfg = _demo_cfg(1)
    assert len(cfg.drones.units) == 1
    result = demo.run_scenario_demo(cfg)
    assert result["final_phase"] == "done"        # ran end to end with 1 drone
    assert result["landing_score"] >= 1           # the single drone landed
