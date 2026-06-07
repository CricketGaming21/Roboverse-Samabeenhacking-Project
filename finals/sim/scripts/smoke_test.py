"""Full smoke run: 3 drones fly a CANNED pattern over the pads, the referee
banks what their cameras actually see, and a scoreboard + thrash report print.

PROVES THE SIM RUNS END TO END. This is NOT a mission: the pattern is a fixed
per-drone waypoint hop over its own pad (no search strategy, no lock-on, no
swarm intelligence — those live in the mission project). Drone 0 also fires
two deliberately-too-fast commands so the thrash report has content.

Usage (from the project root): python -m scripts.smoke_test [--rtf 5]
"""

import argparse
import sys
import time

from pyhulax import DroneAPI
from pyhulax.core import CameraPitchMode, Direction

from simcore.config import load_config
from simcore.registry import get_registry, shutdown_registry
from simcore.viz import TopDownView


def run_smoke(cfg, verbose: bool = False, topdown_png: str = None) -> dict:
    """Run the canned 3-drone pattern; returns score/banked/monitor data."""
    reg = get_registry(cfg)
    view = TopDownView(reg)
    try:
        view.start_sampling()
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
        view.stop()
        shutdown_registry()


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description="Hula sim full smoke run")
    ap.add_argument("--rtf", type=float, default=None,
                    help="override meta.real_time_factor for this run")
    ap.add_argument("--topdown", metavar="PNG", default=None,
                    help="also save the top-down view at the end")
    args = ap.parse_args(argv)

    cfg = load_config()
    if args.rtf:
        cfg.meta.real_time_factor = args.rtf
    if not cfg.scoring.enabled:
        print("warning: scoring.enabled is false — no referee, score will be 0")

    result = run_smoke(cfg, verbose=True, topdown_png=args.topdown)
    preempts = sum(result["monitor"]["preemptions"].values())
    if result["score"] > 0 and preempts > 0:
        print("SMOKE TEST PASSED")
    else:
        print(f"SMOKE TEST FAILED (score={result['score']}, "
              f"preemptions={preempts})")
        sys.exit(1)


if __name__ == "__main__":
    main()
