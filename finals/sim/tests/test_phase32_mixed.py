"""Phase 32 acceptance test — mixed convoy (3 autonomous + 2 evasive) + the
SSH-safe teleop hook (headless).

The evasive pair models the human-teleoperated opponents: flee a nearby drone,
seek crate cover, and juke (intentionally un-smooth). The 3 autonomous rovers
keep the Phase-28 smooth rate-limited turning. Teleop drives one evasive rover
from terminal stdin — never a GUI window. Public API surface unchanged.
"""

import inspect
import math
import time

import numpy as np
import pytest

from pyhulax import DroneAPI

from simcore import arena, frames, rover_model
from simcore.config import load_config
from simcore.registry import get_registry, shutdown_registry


def _mixed_cfg(juke=0.0):
    c = load_config("sim_config.yaml")
    c.meta.real_time_factor = 12.0
    c.camera.use_egl = False
    c.scoring.enabled = False
    c.rovers.motion = "mixed"
    c.scenario.phases = "ambush"          # rovers active from boot
    c.rovers.mixed.evasive.juke_prob = juke
    return c


# --------------------------------------------------------------------------- #
# Spawn: 3 autonomous (ids 20-22) + 2 evasive (ids 30-31)
# --------------------------------------------------------------------------- #

def test_mixed_spawns_3_auto_2_evasive_with_id_blocks():
    cfg = _mixed_cfg()
    assert rover_model.resolved_marker_ids(cfg) == [20, 21, 22, 30, 31]
    reg = get_registry(cfg)
    try:
        plan = reg.run_on_sim_thread(
            lambda: [(r.marker_id, r._personality) for r in reg.rovers])
        assert plan == [(20, "convoy"), (21, "convoy"), (22, "convoy"),
                        (30, "evasive"), (31, "evasive")]
        # the two id blocks are disjoint and the evasive ids are the [30,31] block
        auto = {m for m, p in plan if p == "convoy"}
        evasive = {m for m, p in plan if p == "evasive"}
        assert auto == {20, 21, 22} and evasive == {30, 31}
        assert auto.isdisjoint(evasive)
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Evasive: flees a nearby drone
# --------------------------------------------------------------------------- #

def test_evasive_flees_a_nearby_drone():
    cfg = _mixed_cfg(juke=0.0)
    reg = get_registry(cfg)
    try:
        d = DroneAPI()
        d.connect(cfg.drones.units[0].ip)
        d.takeoff(110)
        # hover the drone at a fixed, crate-clear point, then HOLD station
        fr = reg.run_on_sim_thread(lambda: reg.drones[0].takeoff_frame)
        hover_ne = (5.0, 1.0)
        x, y, _ = frames.arena_to_takeoff_cm(cfg, fr, *hover_ne)
        d.move_to(x, y, 110)                   # blocking: settles at the point
        ev = reg.rovers[3]                     # first evasive rover

        def place_rover():                     # 0.7 m away -> within flee_radius
            wx, wy, _ = frames.arena_to_world(cfg, 5.0, 1.7, 0.0)
            ev.pos[:] = [wx, wy, ev._half_z]
            ev.in_arena = True
            ev._next_decision = 0.0

        def separation():
            rn, re_ = ev.arena_position()
            dn, de_ = frames.world_to_arena(cfg, reg.drones[0].pos[0],
                                            reg.drones[0].pos[1])
            return math.hypot(rn - dn, re_ - de_)

        reg.run_on_sim_thread(place_rover)
        s0 = reg.run_on_sim_thread(separation)
        assert s0 < cfg.rovers.mixed.evasive.flee_radius_m   # starts inside
        time.sleep(1.2)
        s1 = reg.run_on_sim_thread(separation)
        # the drone held station; the rover put distance between them (fled)
        assert s1 > s0 + 0.2, f"evasive rover did not flee (sep {s0:.2f}->{s1:.2f})"
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Cover-seeking: prefers crate-shadowed flee headings (deterministic)
# --------------------------------------------------------------------------- #

def test_cover_bias_prefers_crate_shadowed_routes():
    cfg = _mixed_cfg()
    reg = get_registry(cfg)
    try:
        ev = reg.rovers[3]
        # a single synthetic crate offset from the straight-away axis
        crate = arena.ObstacleSpec(north=4.0, east=3.5, half_n=0.225,
                                   half_e=0.225, height_m=1.0)
        ev._ground_obstacles = [crate]
        rover_ne = (5.0, 3.0)
        drone_ne = (2.0, 3.0)                  # due south -> away is +north
        away = (1.0, 0.0)

        # pure flee (no cover bias) -> the straight-away direction
        ev._flee_gain, ev._cover_bias = 1.0, 0.0
        d_flee = ev._flee_direction(*rover_ne, drone_ne)
        assert d_flee == pytest.approx(away, abs=1e-6)

        # cover-dominant -> the most crate-shadowed candidate, which here is
        # NOT straight away (the crate is off the away-axis)
        ev._flee_gain, ev._cover_bias = 0.0, 1.0
        d_cover = ev._flee_direction(*rover_ne, drone_ne)
        look_cover = (rover_ne[0] + d_cover[0] * rover_model._COVER_LOOKAHEAD_M,
                      rover_ne[1] + d_cover[1] * rover_model._COVER_LOOKAHEAD_M)
        look_away = (rover_ne[0] + away[0] * rover_model._COVER_LOOKAHEAD_M,
                     rover_ne[1] + away[1] * rover_model._COVER_LOOKAHEAD_M)
        score_cover = ev._cover_score(look_cover, drone_ne)
        score_away = ev._cover_score(look_away, drone_ne)
        assert score_cover > score_away          # genuinely chose more cover
        assert d_cover != pytest.approx(away, abs=1e-3)
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# Teleop: headless, no window; drives one evasive rover
# --------------------------------------------------------------------------- #

def test_keys_to_rover_drive_mapping():
    assert rover_model.keys_to_rover_drive({"w"}, 0.4) == (0.4, 0.0)   # +north
    assert rover_model.keys_to_rover_drive({"s"}, 0.4) == (-0.4, 0.0)  # -north
    assert rover_model.keys_to_rover_drive({"d"}, 0.4) == (0.0, 0.4)   # +east
    assert rover_model.keys_to_rover_drive({"a"}, 0.4) == (0.0, -0.4)  # -east
    assert rover_model.keys_to_rover_drive({"w", "s"}, 0.4) == (0.0, 0.0)  # cancel
    assert rover_model.keys_to_rover_drive(set(), 0.4) == (0.0, 0.0)   # stop
    # diagonal is clamped to the speed magnitude
    vn, ve = rover_model.keys_to_rover_drive({"w", "d"}, 0.4)
    assert math.hypot(vn, ve) == pytest.approx(0.4, abs=1e-9)


def test_teleop_drives_evasive_rover_headlessly_no_window():
    from scripts.rover_teleop import RoverTeleop, read_key_loop
    import scripts.rover_teleop as mod
    # the teleop module opens NO window (no cv2.imshow / no p.GUI / no pybullet)
    src = inspect.getsource(mod)
    assert "imshow" not in src and "p.GUI" not in src
    imports = [ln for ln in src.splitlines()
               if ln.lstrip().startswith(("import ", "from "))]
    assert not any("pybullet" in ln for ln in imports)

    cfg = _mixed_cfg(juke=0.0)
    reg = get_registry(cfg)
    try:
        teleop = RoverTeleop(reg)
        assert teleop.rover_index == 3         # first evasive (auto block = 3)
        ev = reg.rovers[teleop.rover_index]

        def place():                           # a crate-clear lane (low north)
            wx, wy, _ = frames.arena_to_world(cfg, 1.0, 1.0, 0.0)
            ev.pos[:] = [wx, wy, ev._half_z]
            ev.in_arena = True

        reg.run_on_sim_thread(place)
        n0, e0 = reg.run_on_sim_thread(ev.arena_position)
        # human holds 'd' (drive +east): a real teleop loop re-sends the key,
        # keeping the drive fresh (teleop_timeout_s). Applied headlessly.
        drive = teleop.apply_keys({"d"})
        assert drive[1] > 0 and drive[0] == 0.0
        assert reg.run_on_sim_thread(lambda: ev.teleop_active())
        for _ in range(12):
            teleop.apply_keys({"d"})
            time.sleep(0.05)
        n1, e1 = reg.run_on_sim_thread(ev.arena_position)
        assert e1 > e0 + 0.2                    # drove east under teleop
        assert abs(n1 - n0) < 0.2               # not north/south (teleop, not AI)
    finally:
        shutdown_registry()


# --------------------------------------------------------------------------- #
# The 3 autonomous rovers stay SMOOTH (Phase-28) in mixed mode
# --------------------------------------------------------------------------- #

def test_autonomous_rovers_stay_smooth_in_mixed():
    cfg = _mixed_cfg()
    cfg.rovers.convoy.speed_mps = 0.5
    cfg.rovers.convoy.entry_stagger_s = 0.0
    speed = cfg.rovers.convoy.speed_mps
    reg = get_registry(cfg)
    try:
        # sample autonomous rover 0 finely (atomic sim-time + pose snapshots)
        track, deadline = [], time.time() + 7.0
        while time.time() < deadline:
            t, pos = reg.run_on_sim_thread(
                lambda: (reg.clock.now(),
                         (float(reg.rovers[0].pos[0]),
                          float(reg.rovers[0].pos[1]), float(reg.rovers[0].yaw),
                          bool(reg.rovers[0].in_arena))))
            track.append((t, pos))
            time.sleep(0.01)
        seen, samples = set(), []
        for t, pos in track:
            if not pos[3] or t in seen:
                continue
            seen.add(t)
            samples.append((t, pos))
        samples = samples[20:]                  # drop the entry transient
        assert len(samples) > 40

        turns = []
        max_turn = math.radians(cfg.rovers.convoy.turn_rate_dps)
        for (t0, p0), (t1, p1) in zip(samples, samples[1:]):
            dt = t1 - t0
            if dt <= 0:
                continue
            step = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            assert step < 0.25, f"autonomous rover position snap {step:.2f} m"
            dyaw = (p1[2] - p0[2] + math.pi) % (2 * math.pi) - math.pi
            turns.append(dyaw)
            # heading is RATE-LIMITED (smooth), never an instant spin
            assert abs(dyaw) <= max_turn * dt + 0.05
        # NO OSCILLATION: heading does not rapidly reverse
        reversals = sum(1 for a, b in zip(turns, turns[1:])
                        if a * b < 0 and abs(a) > 0.08 and abs(b) > 0.08)
        assert reversals <= max(3, len(turns) // 15)
    finally:
        shutdown_registry()
