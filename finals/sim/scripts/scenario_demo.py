"""Full two-phase scenario demo: watch DEPLOY -> AMBUSH -> DONE end to end.

ASSEMBLY ONLY — no new sim capability, nothing on the pyhulax surface, and
explicitly NOT the mission: every flight below is a FIXED canned script that
exercises the world. There is no search strategy, no coordination, and no
camera-driven control anywhere in this file — the waypoints, stations and
pitch keyframes are constants, and the "lock-on" is a SCRIPTED stand-in
timed to a rover pass computed from CONFIG geometry (entry stagger + route
length / speed — fully deterministic), never from sensing. The real lock-on
PID lives in the mission project.

  1. PART 1: the 3 drones launch from the SW entrance and fly canned hops to
     the 3 designated pads, landing (part-1 scorer records accuracy + time).
  2. The scenario advances to AMBUSH per scenario.ambush_trigger; the convoy
     enters and winds its authored loops.
  3. PART 2: the drones take off to FIXED observation hover points over the
     convoy lanes and HOLD position. When the scripted pass time of "their"
     rover arrives, they ease position ~25 cm toward its approach AND tilt
     the camera smoothly over ~2 s to keep it framed, then ease back — the
     gradual lock-on demonstration. The snapshot referee banks rover ids.
  4. The combined scoreboard + thrash report print at the end.

Runs the configured scenario.episode_seconds. Headless by default;
--gui/--live/--debug/--dump apply, as do viz.show_camera_fov /
show_proximity / show_camera_windows.

Usage (from the project root): python -m scripts.scenario_demo [--rtf 5]
"""

import argparse
import math
import os
import sys
import threading
import time

from pyhulax import DroneAPI
from pyhulax.core import CameraPitchMode, Direction

from simcore import frames, rover_model
from simcore.config import load_config
from simcore.debug import start_debug_loop
from simcore.registry import get_registry, shutdown_registry
from simcore.scoring import format_combined_scoreboard
from simcore.viz import TopDownView

# ---- THE CANNED SCRIPT (fixed data — edit coordinates, never add logic) ---- #
DEPLOY_PAD_INDEX = (0, 1, 2)        # drone i lands on designated pad i
# Fixed observation hover points — each sits EXACTLY on a convoy-branch
# waypoint so the rover pass times are deterministic config geometry.
OBSERVE_STATIONS = ((6.5, 0.9), (7.0, 5.2), (2.2, 5.2))
LOCKON_TARGETS = (0, 2, 3)          # drone i scripts against this rover
# Gradual lock-on keyframes: pitch eases down toward the approaching rover
# and back as it passes underneath; position eases out-and-back in sync.
LOCKON_PITCH_SEQ = (90, 84, 77, 70, 64, 70, 77, 84, 90)
LOCKON_EASE_M = 0.25
LOCKON_STEP_S = 0.15                # per keyframe (~2 s total incl. eases)
LOCKON_LEAD_S = 1.0                 # start this long before the pass
OBSERVE_ALT_CM = 150


def _goto(d, cfg, north, east, z_cm=OBSERVE_ALT_CM):
    """move_to an arena point via the drone's LIVE takeoff frame (re-takeoffs
    re-anchor the frame, so the config start is not usable here)."""
    reg, drone = d._reg, d._drone
    fr = reg.run_on_sim_thread(lambda: drone.takeoff_frame)
    x_cm, y_cm, _ = frames.arena_to_takeoff_cm(cfg, fr, north, east)
    d.move_to(x_cm, y_cm, z_cm)


# ---- deterministic pass times from CONFIG (no sensing anywhere) ----------- #

def _station_on_route(cfg, rover_idx, station):
    route = rover_model.convoy_route(cfg, rover_idx)
    for idx, wp in enumerate(route):
        if math.dist(wp, station) < 1e-6:
            return route, idx
    raise ValueError(f"station {station} must sit exactly on rover "
                     f"{rover_idx}'s configured route")


def rover_pass_times(cfg, rover_idx, station, ambush_t0, horizon_s):
    """Sim times rover_idx drives over `station`, from config geometry only:
    staggered entry + cumulative route distance / speed, then once per
    branch-loop cycle. Deterministic — this is what makes the scripted
    lock-on canned rather than tracking."""
    route, idx = _station_on_route(cfg, rover_idx, station)
    cv = cfg.rovers.convoy
    speed = float(cv.speed_mps)
    cum = sum(math.dist(route[k], route[k + 1]) for k in range(idx))
    branch = route[cv.split_index + 2:]
    cycle = (sum(math.dist(branch[k], branch[k + 1])
                 for k in range(len(branch) - 1))
             + math.dist(branch[-1], branch[0]))  # loiter=loop closes it
    first = ambush_t0 + rover_idx * float(cv.entry_stagger_s) + cum / speed
    times, t = [], first
    while t <= ambush_t0 + horizon_s:
        times.append(t)
        t += cycle / speed
    return times


def _approach_dir(cfg, rover_idx, station):
    """Unit vector from the station back toward where the rover comes from."""
    route, idx = _station_on_route(cfg, rover_idx, station)
    prev = route[idx - 1]
    dn, de = prev[0] - station[0], prev[1] - station[1]
    length = math.hypot(dn, de)
    return (dn / length, de / length)


# ---- the canned flights ---------------------------------------------------- #

def _fly_deploy(cfg, i, errors):
    """PART-1 canned hop: entrance -> designated pad -> land."""
    try:
        pad = cfg.pads[DEPLOY_PAD_INDEX[i]]
        d = DroneAPI()
        d.connect(cfg.drones.units[i].ip)
        d.takeoff(150)
        _goto(d, cfg, pad.north, pad.east)
        d.hover(0.5)
        if i == 0:  # deliberate thrash demo so the report has content
            d.move(Direction.FORWARD, 60, blocking=False)
            d.move(Direction.BACK, 30, blocking=False)
            d.hover(0.5)
        d.land()
    except Exception as e:  # surfaced by the director
        errors.append(e)


def _fly_observe(cfg, i, stop_evt, errors):
    """PART-2: hold a FIXED hover point; run the scripted gradual lock-on at
    each precomputed rover pass. Stationary observer — not a search."""
    try:
        d = DroneAPI()
        d.connect(cfg.drones.units[i].ip)
        reg = d._reg
        station = OBSERVE_STATIONS[i]
        rover_idx = LOCKON_TARGETS[i]
        adir = _approach_dir(cfg, rover_idx, station)
        t0 = reg.scenario.ambush_started_at
        if t0 is None:
            t0 = reg.sim_time()
        passes = rover_pass_times(cfg, rover_idx, station, t0,
                                  horizon_s=cfg.scenario.episode_seconds)
        d.takeoff(OBSERVE_ALT_CM)
        d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 90)
        _goto(d, cfg, *station)
        next_pass = 0
        while not stop_evt.is_set():
            now = reg.sim_time()
            while (next_pass < len(passes)
                   and passes[next_pass] - LOCKON_LEAD_S < now - 0.3):
                next_pass += 1  # window already missed: skip it
            if next_pass >= len(passes):
                d.hover(0.5)    # HOLD: no pass left, keep observing
                continue
            if now < passes[next_pass] - LOCKON_LEAD_S:
                d.hover(0.3)    # HOLD the station until the scripted window
                continue
            # Scripted GRADUAL lock-on: pitch + position ease together over
            # ~2 s toward the rover's approach, then back. Fixed keyframes.
            steps = len(LOCKON_PITCH_SEQ)
            for k, pitch in enumerate(LOCKON_PITCH_SEQ):
                if stop_evt.is_set():
                    break
                d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, pitch)
                ease = LOCKON_EASE_M * math.sin(math.pi * k / (steps - 1))
                _goto(d, cfg, station[0] + adir[0] * ease,
                      station[1] + adir[1] * ease)
                d.hover(LOCKON_STEP_S)
            next_pass += 1
        d.land()
    except Exception as e:
        errors.append(e)


def _track_world(reg, rover_samples, drone_samples, done_evt):
    """Observer: rover + drone state over the whole run (for the tests)."""
    while not done_evt.is_set() and reg.is_alive():
        try:
            t, ph = reg.sim_time(), reg.scenario.phase
            rover_samples.append((t, ph, reg.rover_arena_positions(),
                                  [r.in_arena for r in reg.rovers]))
            drone_samples.append((t, ph, reg.run_on_sim_thread(
                lambda: [(*d.arena_position(), float(d.pos[2]),
                          float(d.camera_pitch_deg)) for d in reg.drones])))
        except RuntimeError:
            break
        time.sleep(0.02)


def _direct(cfg, reg, done_evt, errors):
    """The director: part 1 -> wait for AMBUSH -> part 2 -> DONE."""
    rtf = max(cfg.meta.real_time_factor, 1e-9)
    try:
        part1 = [threading.Thread(target=_fly_deploy, args=(cfg, i, errors),
                                  daemon=True) for i in range(3)]
        for t in part1:
            t.start()
        for t in part1:
            t.join(timeout=cfg.scenario.deploy_timeout_s / rtf + 60)
        deadline = time.time() + cfg.scenario.episode_seconds / rtf + 60
        while (reg.scenario.phase == "deploy" and time.time() < deadline):
            time.sleep(0.02)               # the SCENARIO advances itself

        stop = threading.Event()
        part2 = [threading.Thread(target=_fly_observe,
                                  args=(cfg, i, stop, errors), daemon=True)
                 for i in range(3)]
        for t in part2:
            t.start()
        while reg.scenario.phase != "done" and time.time() < deadline:
            time.sleep(0.05)
        stop.set()
        for t in part2:
            t.join(timeout=30)
    finally:
        done_evt.set()


def run_scenario_demo(cfg, verbose=False, gui=False, live=False, debug=False,
                      dump_path=None, topdown_png=None) -> dict:
    """Run the whole canned scenario; returns both scores + run telemetry."""
    reg = get_registry(cfg, gui=gui)
    view = TopDownView(reg)
    stop_debug = None
    cams = []
    rover_samples = []
    drone_samples = []
    done_evt = threading.Event()
    errors = []
    try:
        if debug or dump_path:
            stop_debug = start_debug_loop(reg, print_text=debug,
                                          dump_path=dump_path)
        if cfg.viz.show_camera_windows:
            from scripts.run_sim import open_camera_windows
            from simcore.log import get_logger
            cams = open_camera_windows(cfg, get_logger("demo", cfg))
        view.start_sampling()
        threading.Thread(target=_track_world,
                         args=(reg, rover_samples, drone_samples, done_evt),
                         daemon=True).start()
        director = threading.Thread(target=_direct,
                                    args=(cfg, reg, done_evt, errors),
                                    daemon=True)
        director.start()
        if live:
            view.run_live(until=done_evt.is_set)
        rtf = max(cfg.meta.real_time_factor, 1e-9)
        director.join(timeout=cfg.scenario.episode_seconds / rtf + 180)
        if errors:
            raise errors[0]
        time.sleep(0.3)  # let the referees flush
        banked = reg.referee.banked() if reg.referee else []
        result = {
            "landing_score": reg.landing_scorer.score(),
            "landings": [(r.drone_index, r.pad_id, r.error_m, r.sim_time)
                         for r in reg.landing_scorer.results()],
            "snapshot_score": len(banked),
            "banked": [(b.marker_id, b.drone_index, b.sim_time)
                       for b in banked],
            "monitor": reg.monitor.snapshot(),
            "sim_time": reg.sim_time(),
            "final_phase": reg.scenario.phase,
            "rover_samples": list(rover_samples),
            "drone_samples": list(drone_samples),
        }
        if topdown_png:
            view.sample()
            view.render_png(topdown_png)
        if verbose:
            print()
            print(format_combined_scoreboard(reg))
            print(reg.monitor.format_report())
            print(f"episode: {result['sim_time']:.1f}s sim "
                  f"(configured {cfg.scenario.episode_seconds:.0f}s) — "
                  f"final phase: {result['final_phase']}")
        return result
    finally:
        if cams:
            from scripts.run_sim import close_camera_windows
            close_camera_windows(cams)
        if stop_debug:
            stop_debug()
        view.stop()
        shutdown_registry()


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(
        description="Full two-phase scenario demo (canned flight, not a "
                    "mission)")
    ap.add_argument("--rtf", type=float, default=None,
                    help="override meta.real_time_factor for this run")
    ap.add_argument("--gui", action="store_true",
                    help="PyBullet 3D window instead of headless DIRECT")
    ap.add_argument("--live", action="store_true",
                    help="watch on the live top-down view")
    ap.add_argument("--debug", action="store_true",
                    help="print world snapshots once per sim second")
    ap.add_argument("--dump", metavar="PATH", default=None,
                    help="append per-tick JSON snapshots to PATH")
    ap.add_argument("--topdown", metavar="PNG", default=None,
                    help="save the final top-down view")
    args = ap.parse_args(argv)

    cfg = load_config()
    if args.rtf:
        cfg.meta.real_time_factor = args.rtf

    result = run_scenario_demo(cfg, verbose=True, gui=args.gui,
                               live=args.live, debug=args.debug,
                               dump_path=args.dump, topdown_png=args.topdown)
    ok = result["landing_score"] >= 1 and result["snapshot_score"] >= 1
    print("SCENARIO DEMO COMPLETE" if ok else
          f"SCENARIO DEMO INCOMPLETE (landings={result['landing_score']}, "
          f"snapshots={result['snapshot_score']})")
    if args.gui:
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0 if ok else 1)
    if not ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
