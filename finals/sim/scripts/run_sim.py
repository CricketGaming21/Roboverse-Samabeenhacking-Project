"""Boot the sim world and let it run; optional live/top-down 2D view.

Usage (from the project root):
    python -m scripts.run_sim --seconds 2 --topdown out.png
    python -m scripts.run_sim --live --seconds 30
The live window is gated by config viz.enabled; --topdown always works
(headless-safe Agg render).
"""

import argparse
import time

from simcore.config import load_config
from simcore.log import get_logger
from simcore.registry import SimRegistry
from simcore.viz import TopDownView


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description="Boot and step the Hula sim world")
    ap.add_argument("--config", default=None,
                    help="config YAML (default: $HULA_SIM_CONFIG or ./sim_config.yaml)")
    ap.add_argument("--seconds", type=float, default=2.0,
                    help="wall-clock seconds to let the sim run (default 2)")
    ap.add_argument("--topdown", metavar="PNG", default=None,
                    help="save the top-down 2D view to this PNG at the end")
    ap.add_argument("--live", action="store_true",
                    help="show the live top-down view (needs viz.enabled)")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    log = get_logger("run_sim", cfg)
    reg = SimRegistry(cfg)
    view = TopDownView(reg)
    try:
        b = reg.bodies
        log.info("running: floor=1 walls=%d obstacles=%d drones=%d rovers=%d "
                 "pads=%d referee=%s", len(b.walls), len(b.obstacles),
                 len(b.drones), len(b.rovers), len(b.pads),
                 "on" if reg.referee else "off")
        view.start_sampling()
        if args.live and cfg.viz.enabled:
            view.run_live(args.seconds)
        else:
            if args.live:
                log.warning("--live requested but viz.enabled is false")
            time.sleep(max(0.0, args.seconds))
        log.info("sim time %.2fs (real_time_factor %.2f)",
                 reg.sim_time(), cfg.meta.real_time_factor)
        if args.topdown:
            view.sample()
            view.render_png(args.topdown)
        if reg.referee is not None:
            print(reg.referee.format_scoreboard())
        print(reg.monitor.format_report())
    finally:
        view.stop()
        reg.shutdown()


if __name__ == "__main__":
    main()
