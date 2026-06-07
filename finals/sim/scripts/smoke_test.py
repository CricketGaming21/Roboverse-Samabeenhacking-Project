"""Full smoke run: 3 drones fly a CANNED pattern over the pads, the referee
banks what their cameras actually see, and a scoreboard + thrash report print.

PROVES THE SIM RUNS END TO END. This is NOT a mission: the pattern is a fixed
per-drone waypoint hop over its own pad (no search strategy, no lock-on, no
swarm intelligence — those live in the mission project). Drone 0 also fires
two deliberately-too-fast commands so the thrash report has content.

Usage (from the project root):
    python -m scripts.smoke_test [--rtf 5] [--topdown PNG]
                                 [--live] [--gui] [--debug] [--dump PATH]
--live watches the canned flight scoring on the top-down 2D view (the flight
runs in a background thread; the window owns the main thread). --gui boots
the world in PyBullet's interactive 3D window instead of headless DIRECT.
"""

import argparse
import os
import sys
import threading
import time

from pyhulax import DroneAPI
from pyhulax.core import CameraPitchMode, Direction

from simcore.config import load_config
from simcore.debug import start_debug_loop
from simcore.registry import get_registry, shutdown_registry
from simcore.viz import TopDownView


def _fly_canned_pattern(cfg, verbose: bool = False) -> None:
    """The fixed per-drone waypoint hop. NOT a mission."""
    for i, unit in enumerate(cfg.drones.units):
        pad = cfg.pads[i % len(cfg.pads)]
        if verbose:
            print(f"drone {i} ({unit.ip}): takeoff -> pad {pad.id} "
                  f"at ({pad.north}, {pad.east}) -> hover -> land")
        d = DroneAPI()
        d.connect(unit.ip)
        d.takeoff(150)
        d.set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 90)
        # Canned hop: takeoff frame at heading 0 => x=right=east,
        # y=forward=north; park the camera over the pad centre.
        x_cm = (pad.east - unit.start[1]) * 100.0
        y_cm = ((pad.north - unit.start[0])
                - cfg.camera.mount_offset_m) * 100.0
        d.move_to(x_cm, y_cm, 150)
        d.hover(1.2)  # > hold_frames at camera fps: a stable read
        if i == 0:
            # Deliberate thrash demo: re-command far faster than
            # monitor.max_cmd_rate_hz allows.
            d.move(Direction.FORWARD, 80, blocking=False)
            d.move(Direction.BACK, 40, blocking=False)
            d.hover(0.5)
        d.land()


def run_smoke(cfg, verbose: bool = False, topdown_png: str = None,
              gui: bool = False, live: bool = False, debug: bool = False,
              dump_path: str = None) -> dict:
    """Run the canned 3-drone pattern; returns score/banked/monitor data."""
    reg = get_registry(cfg, gui=gui)
    view = TopDownView(reg)
    stop_debug = None
    try:
        if debug or dump_path:
            stop_debug = start_debug_loop(reg, print_text=debug,
                                          dump_path=dump_path)
        view.start_sampling()
        if live:
            # The matplotlib window must own the MAIN thread, so the canned
            # flight runs in the background while we watch it score.
            done = threading.Event()
            errors = []

            def _bg():
                try:
                    _fly_canned_pattern(cfg, verbose)
                except Exception as e:  # surfaced after the view closes
                    errors.append(e)
                finally:
                    done.set()
            flight = threading.Thread(target=_bg, name="smoke-flight",
                                      daemon=True)
            flight.start()
            view.run_live(until=done.is_set)  # False (no display) is fine:
            flight.join(timeout=600)          # the flight finishes regardless
            if errors:
                raise errors[0]
        else:
            _fly_canned_pattern(cfg, verbose)
        time.sleep(0.3)  # let the referee flush its last frames
        banked = reg.referee.banked() if reg.referee else []
        result = {
            "score": len(banked),
            "banked": [(b.marker_id, b.drone_index, b.sim_time)
                       for b in banked],
            "monitor": reg.monitor.snapshot(),
            "sim_time": reg.sim_time(),
        }
        if topdown_png:
            view.sample()
            view.render_png(topdown_png)
        if verbose:
            print()
            if reg.referee is not None:
                print(reg.referee.format_scoreboard())
            print(reg.monitor.format_report())
            print(f"sim time: {result['sim_time']:.1f}s")
        return result
    finally:
        if stop_debug:
            stop_debug()
        view.stop()
        shutdown_registry()


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description="Hula sim full smoke run")
    ap.add_argument("--rtf", type=float, default=None,
                    help="override meta.real_time_factor for this run")
    ap.add_argument("--topdown", metavar="PNG", default=None,
                    help="also save the top-down view at the end")
    ap.add_argument("--live", action="store_true",
                    help="watch the flight scoring on the live top-down view")
    ap.add_argument("--gui", action="store_true",
                    help="boot with PyBullet's interactive 3D window instead "
                         "of headless DIRECT")
    ap.add_argument("--debug", action="store_true",
                    help="print a read-only world snapshot once per sim second")
    ap.add_argument("--dump", metavar="PATH", default=None,
                    help="append per-tick JSON snapshots (JSON Lines) to PATH")
    args = ap.parse_args(argv)

    cfg = load_config()
    if args.rtf:
        cfg.meta.real_time_factor = args.rtf
    if not cfg.scoring.enabled:
        print("warning: scoring.enabled is false — no referee, score will be 0")

    result = run_smoke(cfg, verbose=True, topdown_png=args.topdown,
                       gui=args.gui, live=args.live, debug=args.debug,
                       dump_path=args.dump)
    preempts = sum(result["monitor"]["preemptions"].values())
    ok = result["score"] > 0 and preempts > 0
    if ok:
        print("SMOKE TEST PASSED")
    else:
        print(f"SMOKE TEST FAILED (score={result['score']}, "
              f"preemptions={preempts})")
    if args.gui:
        # See run_sim: the GUI client crashes interpreter teardown; output
        # is flushed and the sim is shut down — exit hard with the verdict.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0 if ok else 1)
    if not ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
