"""Full two-phase scenario demo: watch DEPLOY -> AMBUSH -> DONE end to end.

ASSEMBLY ONLY — no new sim capability, nothing on the pyhulax surface, and
explicitly NOT the mission: every flight below is a FIXED canned script that
exercises the world. There is no search strategy, no coordination, and no
camera-driven control anywhere in this file — the pitch sweep is a scripted
stand-in for a tilt, the waypoints are constants. The real logic lives in
the mission project.

  1. PART 1: the 3 drones launch from the SW entrance and fly canned hops to
     the 3 designated pads, landing (part-1 scorer records accuracy + time).
  2. The scenario advances to AMBUSH per scenario.ambush_trigger; the convoy
     enters and runs its authored routes.
  3. PART 2: the drones take off again and hold fixed observation stations
     over the convoy lanes, sweeping camera pitch, while the snapshot
     referee banks distinct rover ids.
  4. The combined scoreboard + thrash report print at the end.

Runs the configured scenario.episode_seconds. Headless by default;
--gui/--live/--debug/--dump apply, as do viz.show_camera_fov /
show_proximity / show_camera_windows.

Usage (from the project root): python -m scripts.scenario_demo [--rtf 5]
"""

import argparse
import os
import sys
import threading
import time

from pyhulax import DroneAPI
from pyhulax.core import CameraPitchMode, Direction

from simcore import frames
from simcore.config import load_config
from simcore.debug import start_debug_loop
from simcore.registry import get_registry, shutdown_registry
from simcore.scoring import format_combined_scoreboard
from simcore.viz import TopDownView

# ---- THE CANNED SCRIPT (fixed data — edit coordinates, never add logic) ---- #
DEPLOY_PAD_INDEX = (0, 1, 2)        # drone i lands on designated pad i
OBSERVE_STATIONS = (                # drone i holds these fixed arena points,
    ((6.0, 1.0), (7.5, 2.2)),       # chosen over the convoy's branch lanes
    ((7.0, 5.0), (8.0, 3.6)),
    ((3.5, 4.5), (4.0, 4.4)),
)
OBSERVE_PITCH_SWEEP = (90, 70, 90)  # scripted tilt stand-in (degrees down)
OBSERVE_ALT_CM = 150
_HOVER_S = 2.0


def _goto(d, cfg, north, east, z_cm=OBSERVE_ALT_CM):
    """move_to an arena point via the drone's LIVE takeoff frame (re-takeoffs
    re-anchor the frame, so the config start is not usable here)."""
    reg, drone = d._reg, d._drone
    fr = reg.run_on_sim_thread(lambda: drone.takeoff_frame)
    x_cm, y_cm, _ = frames.arena_to_takeoff_cm(cfg, fr, north, east)
    d.move_to(x_cm, y_cm, z_cm)


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
    """PART-2 canned circuit: hold fixed stations, sweep pitch, repeat."""
    try:
        d = DroneAPI()
        d.connect(cfg.drones.units[i].ip)
        d.takeoff(OBSERVE_ALT_CM)
        d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 90)
        while not stop_evt.is_set():
            for north, east in OBSERVE_STATIONS[i]:
                if stop_evt.is_set():
                    break
                _goto(d, cfg, north, east)
                for pitch in OBSERVE_PITCH_SWEEP:  # scripted tilt, not lock-on
                    if stop_evt.is_set():
                        break
                    d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, pitch)
                    d.hover(_HOVER_S)
        d.land()
    except Exception as e:
        errors.append(e)


def _track_rovers(reg, samples, done_evt):
    """Observer: rover positions + phase over the whole run (for the tests)."""
    while not done_evt.is_set() and reg.is_alive():
        try:
            samples.append((reg.sim_time(), reg.scenario.phase,
                            reg.rover_arena_positions(),
                            [r.in_arena for r in reg.rovers]))
        except RuntimeError:
            break
        time.sleep(0.05)


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
    samples = []
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
        threading.Thread(target=_track_rovers,
                         args=(reg, samples, done_evt), daemon=True).start()
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
            "rover_samples": list(samples),
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
