"""Boot the sim world from config and let it step; optional top-down screenshot.

Phase 1 CLI (the live view and keep-alive options arrive in Phase 7).
Usage (from the project root):
    python -m scripts.run_sim --seconds 2 --topdown out.png
"""

import argparse
import time

from simcore.config import load_config
from simcore.log import get_logger
from simcore.registry import SimRegistry


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description="Boot and step the Hula sim world")
    ap.add_argument("--config", default=None,
                    help="config YAML (default: $HULA_SIM_CONFIG or ./sim_config.yaml)")
    ap.add_argument("--seconds", type=float, default=2.0,
                    help="wall-clock seconds to let the sim run (default 2)")
    ap.add_argument("--topdown", metavar="PNG", default=None,
                    help="save a top-down screenshot of the arena to this file")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    log = get_logger("run_sim", cfg)
    reg = SimRegistry(cfg)
    try:
        b = reg.bodies
        log.info("running: floor=1 walls=%d obstacles=%d drones=%d rovers=%d",
                 len(b.walls), len(b.obstacles), len(b.drones), len(b.rovers))
        time.sleep(max(0.0, args.seconds))
        log.info("sim time advanced to %.2fs (real_time_factor %.2f)",
                 reg.sim_time(), cfg.meta.real_time_factor)
        if args.topdown:
            reg.save_topdown_png(args.topdown)
    finally:
        reg.shutdown()


if __name__ == "__main__":
    main()
