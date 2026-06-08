"""Phase 10 acceptance test — two-phase scenario controller (headless).

DEPLOY: rovers parked OFF-MAP, inert, not targets. AMBUSH: rovers active.
DONE: at episode_seconds / ambush window end / all rovers scanned. The
trigger is scenario-owned (all three modes), never mission-called — the
pyhulax/UWB public surface gains nothing.
"""

import time

import pytest

from pyhulax import DroneAPI

from simcore.config import load_config
from simcore.registry import get_registry, shutdown_registry
from simcore.scoring import BankedID


def _cfg(**scenario_overrides):
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 10.0
    c.camera.use_egl = False
    c.scoring.enabled = False
    c.rovers.motion = "patrol"  # scenario mechanics under test here;
    c.motion.realistic = False  # crisp snap motion: exact geometry under test
    # convoy routing has its own suite in test_phase11_convoy.py
    for key, value in scenario_overrides.items():
        if key in ("mode", "delay_s"):
            setattr(c.scenario.ambush_trigger, key, value)
        else:
            setattr(c.scenario, key, value)
    return c


def _wait_phase(reg, phase, timeout_s=5.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if reg.scenario.phase == phase:
            return True
        time.sleep(0.01)
    return False


def _land_drone(cfg, index):
    d = DroneAPI()
    d.connect(cfg.drones.units[index].ip)
    d.takeoff(60)
    d.land()
    return d


# --------------------------------------------------------------------------- #
# DEPLOY: rovers off-map, inert, not targets
# --------------------------------------------------------------------------- #

def test_deploy_rovers_offmap_and_inert():
    cfg = _cfg()  # defaults: phases=both, on_all_landed
    reg = get_registry(cfg)
    try:
        assert reg.scenario.phase == "deploy"
        before = reg.rover_arena_positions()
        assert all(n < 0.0 for n, _e in before)   # outside the south wall
        assert all(not r.in_arena for r in reg.rovers)
        time.sleep(0.4)                           # ~4 sim seconds
        assert reg.rover_arena_positions() == before  # inert: zero motion
        assert reg.scenario.phase == "deploy"     # no drones flew: no trigger
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# The three trigger modes
# --------------------------------------------------------------------------- #

def test_on_all_landed_trigger():
    cfg = _cfg(mode="on_all_landed", delay_s=0.5)
    reg = get_registry(cfg)
    try:
        _land_drone(cfg, 0)
        _land_drone(cfg, 1)
        time.sleep(0.3)                           # 3 sim s >> delay_s
        assert reg.scenario.phase == "deploy"     # 2 of 3 landed: no trigger
        _land_drone(cfg, 2)
        assert _wait_phase(reg, "ambush"), "all landed + delay must trigger"
        assert all(r.in_arena for r in reg.rovers)
        assert all(n > 0.0 for n, _e in reg.rover_arena_positions())
        p0 = reg.rover_arena_positions()
        time.sleep(0.3)                           # placeholder patrol moves
        p1 = reg.rover_arena_positions()
        assert p0 != p1
    finally:
        shutdown_registry()


def test_timed_trigger():
    cfg = _cfg(mode="timed", delay_s=2.0)
    reg = get_registry(cfg)
    try:
        assert reg.scenario.phase == "deploy"
        assert _wait_phase(reg, "ambush", timeout_s=3.0)
        assert reg.sim_time() >= 2.0              # entered at +delay_s
    finally:
        shutdown_registry()


def test_manual_key_trigger():
    cfg = _cfg(mode="manual_key", delay_s=0.5)
    reg = get_registry(cfg)
    try:
        time.sleep(0.3)                           # 3 sim s: nothing happens
        assert reg.scenario.phase == "deploy"
        reg.scenario.request_manual_trigger()     # the --live 'a' keypress
        assert _wait_phase(reg, "ambush")
    finally:
        shutdown_registry()


def test_deploy_timeout_forces_ambush():
    cfg = _cfg(mode="on_all_landed", deploy_timeout_s=1.5)
    reg = get_registry(cfg)
    try:
        assert _wait_phase(reg, "ambush", timeout_s=3.0)  # nobody landed
        assert reg.sim_time() >= 1.5              # the convoy comes anyway
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# DONE conditions + phases modes
# --------------------------------------------------------------------------- #

def test_episode_runs_configured_duration_then_done():
    cfg = _cfg(mode="timed", delay_s=0.5, episode_seconds=3.0,
               ambush_seconds=100.0)
    reg = get_registry(cfg)
    try:
        # observing AMBUSH at all proves it did NOT end on part 1 (the phase
        # may legitimately advance to done between polls on this short episode)
        assert _wait_phase(reg, "ambush")
        assert _wait_phase(reg, "done", timeout_s=4.0)
        assert reg.sim_time() >= 3.0              # full configured length
        frozen = reg.rover_arena_positions()
        time.sleep(0.2)
        assert reg.rover_arena_positions() == frozen  # DONE: rovers stop
    finally:
        shutdown_registry()


def test_ambush_window_ends_episode():
    cfg = _cfg(mode="timed", delay_s=0.5, ambush_seconds=1.5,
               episode_seconds=100.0)
    reg = get_registry(cfg)
    try:
        assert _wait_phase(reg, "ambush")
        assert _wait_phase(reg, "done", timeout_s=3.0)
        assert reg.sim_time() >= 2.0              # delay + ambush window
    finally:
        shutdown_registry()


def test_all_rovers_scanned_ends_episode():
    cfg = _cfg(mode="timed", delay_s=0.2)
    cfg.scoring.enabled = True                    # need the referee's bank
    reg = get_registry(cfg)
    try:
        assert _wait_phase(reg, "ambush")

        def _inject():  # white-box: pretend every rover id was banked
            with reg.referee._lock:
                for r in reg.rovers:
                    reg.referee._banked[r.marker_id] = BankedID(
                        r.marker_id, 0, reg.clock.now())
        reg.run_on_sim_thread(_inject)
        assert _wait_phase(reg, "done", timeout_s=3.0)
        assert reg.sim_time() < 100.0             # ended early, not by timer
    finally:
        shutdown_registry()


def test_phases_ambush_only_starts_active():
    cfg = _cfg(phases="ambush")
    reg = get_registry(cfg)
    try:
        assert reg.scenario.phase == "ambush"
        assert all(r.in_arena for r in reg.rovers)
        p0 = reg.rover_arena_positions()
        time.sleep(0.3)
        assert reg.rover_arena_positions() != p0  # moving from the start
    finally:
        shutdown_registry()


def test_phases_deploy_only_never_ambushes():
    cfg = _cfg(phases="deploy", deploy_timeout_s=1.5)
    reg = get_registry(cfg)
    try:
        # The invariant is NEVER ambush (the short episode may already be
        # done by the first read under load — that's fine).
        assert reg.scenario.phase in ("deploy", "done")
        assert _wait_phase(reg, "done", timeout_s=4.0)
        assert all(not r.in_arena for r in reg.rovers)  # never entered
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# The mission can never start the ambush
# --------------------------------------------------------------------------- #

def test_no_ambush_api_on_the_public_surface():
    import pyhulax
    import pyhulax.core
    import pyhulax.video
    from UWBParserThread import UWBParserThread
    public = (set(dir(pyhulax)) | set(dir(pyhulax.DroneAPI))
              | set(dir(pyhulax.core)) | set(dir(pyhulax.video))
              | set(dir(UWBParserThread)))
    leaked = [n for n in public
              if "ambush" in n.lower() or "scenario" in n.lower()
              or "convoy" in n.lower()]
    assert leaked == [], f"scenario control leaked into pyhulax: {leaked}"


def test_bad_scenario_config_rejected():
    cfg = _cfg(phases="forever")
    with pytest.raises(ValueError):
        get_registry(cfg)
    shutdown_registry()  # clear any partial singleton
    cfg = _cfg(mode="psychic")
    with pytest.raises(ValueError):
        get_registry(cfg)
    shutdown_registry()
