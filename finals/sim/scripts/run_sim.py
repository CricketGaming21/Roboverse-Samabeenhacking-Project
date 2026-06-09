"""Boot the sim world and let it run; optional live 2D / 3D / debug views.

Usage (from the project root):
    python -m scripts.run_sim --seconds 2 --topdown out.png
    python -m scripts.run_sim --live --seconds 30        # top-down 2D window
    python -m scripts.run_sim --gui --seconds 30         # PyBullet 3D window
    python -m scripts.run_sim --debug --dump run.jsonl   # introspection

The live 2D window is gated by config viz.enabled; --topdown always works
(headless-safe Agg render). --gui boots the world with p.connect(p.GUI)
instead of the headless DIRECT+EGL default — orbit/pan/zoom via WSLg.
"""

import argparse
import os
import sys
import time

from simcore.config import load_config
from simcore.debug import start_debug_loop
from simcore.log import get_logger
from simcore.registry import SimRegistry
from simcore.viz import TopDownView


def open_camera_windows(cfg, log):
    """viz.show_camera_windows debug view: one cv2 window per drone showing
    its live frame with detected ArUco markers OUTLINED + id labelled
    (observer-side overlay — the sim's own cv2.aruco, not a mission sensor)."""
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        log.warning("show_camera_windows: no display; skipping")
        return []
    from pyhulax import DroneAPI
    from simcore.camfeed import MarkerCameraWindow
    pairs = []
    for unit in cfg.drones.units:
        d = DroneAPI()
        d.connect(unit.ip)
        d.set_video_stream(True)
        stream = d.create_video_stream()
        stream.start()
        display = MarkerCameraWindow(cfg, stream, window_name=f"hula {unit.ip}")
        display.start()
        pairs.append((stream, display))
    return pairs


def close_camera_windows(pairs):
    for stream, display in pairs:
        display.stop()
        stream.stop()


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(
        description="Boot and step the Hula sim world",
        epilog="NOTE: run_sim observes a STATIC-DRONE world — nothing flies, "
               "so the score stays 0 by design. Use `python -m "
               "scripts.smoke_test` to see the scored canned flight.")
    ap.add_argument("--config", default=None,
                    help="config YAML (default: $HULA_SIM_CONFIG or ./sim_config.yaml)")
    ap.add_argument("--seconds", type=float, default=2.0,
                    help="wall-clock seconds to let the sim run (default 2)")
    ap.add_argument("--topdown", metavar="PNG", default=None,
                    help="save the top-down 2D view to this PNG at the end")
    ap.add_argument("--live", action="store_true",
                    help="show the live top-down view (needs viz.enabled)")
    ap.add_argument("--gui", action="store_true",
                    help="boot with PyBullet's interactive 3D window instead "
                         "of headless DIRECT (orbit/pan/zoom)")
    ap.add_argument("--debug", action="store_true",
                    help="print a read-only world snapshot once per sim second")
    ap.add_argument("--dump", metavar="PATH", default=None,
                    help="append per-tick JSON snapshots (JSON Lines) to PATH")
    ap.add_argument("--dashboard", action="store_true",
                    help="live read-only per-drone command/telemetry console")
    ap.add_argument("--record", metavar="PATH.mp4", default=None,
                    help="record an offscreen 3D MP4 of the run "
                         "(headless EGL; no GUI window)")
    ap.add_argument("--hq", action="store_true",
                    help="high-quality recording at 1920x1080 (heavier)")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    if args.hq:
        cfg.record.width, cfg.record.height = 1920, 1080
    log = get_logger("run_sim", cfg)
    reg = SimRegistry(cfg, gui=args.gui)
    view = TopDownView(reg)
    stop_debug = None
    dash = None
    recorder = None
    cams = []
    try:
        b = reg.bodies
        log.info("running: floor=1 walls=%d obstacles=%d drones=%d rovers=%d "
                 "pads=%d referee=%s", len(b.walls), len(b.obstacles),
                 len(b.drones), len(b.rovers), len(b.pads),
                 "on" if reg.referee else "off")
        log.info("note: run_sim observes a static-drone world (score stays "
                 "0); use smoke_test to see the scored canned flight")
        log.info("scenario: phases=%s trigger=%s episode=%.0fs phase=%s",
                 cfg.scenario.phases, cfg.scenario.ambush_trigger.mode,
                 cfg.scenario.episode_seconds, reg.scenario.phase)
        if args.debug or args.dump:
            stop_debug = start_debug_loop(reg, print_text=args.debug,
                                          dump_path=args.dump)
        if args.dashboard:
            from scripts.dashboard import CommandDashboard
            dash = CommandDashboard(reg)
            dash.start()
        if args.record:
            from simcore.recorder import ArenaRecorder
            recorder = ArenaRecorder(reg, args.record)
            recorder.start()
        if cfg.viz.show_camera_windows:
            cams = open_camera_windows(cfg, log)
        view.start_sampling()
        if args.live and cfg.viz.enabled:
            if not view.run_live(args.seconds):
                log.warning("no display; use --topdown for a PNG — running "
                            "headless for %.1fs instead", args.seconds)
                time.sleep(max(0.0, args.seconds))
        else:
            if args.live:
                log.warning("--live requested but viz.enabled is false")
            time.sleep(max(0.0, args.seconds))
        log.info("sim time %.2fs (real_time_factor %.2f) — scenario phase: %s",
                 reg.sim_time(), cfg.meta.real_time_factor,
                 reg.scenario.phase)
        if args.topdown:
            view.sample()
            view.render_png(args.topdown)
        from simcore.scoring import format_combined_scoreboard
        print(format_combined_scoreboard(reg))
        print(reg.monitor.format_report())
    finally:
        if recorder:
            recorder.stop()
        if dash:
            dash.stop()
        close_camera_windows(cams)
        if stop_debug:
            stop_debug()
        view.stop()
        reg.shutdown()
    if args.gui:
        # The GUI client is left connected (closing it from a worker thread
        # segfaults under WSLg) and crashes interpreter teardown; all output
        # is flushed and the sim is shut down — exit hard with success.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)


if __name__ == "__main__":
    main()
