"""Live 3D-only viewer — a p.GUI window with ALL camera rendering DISABLED.

FREEZE-PROOF BY DESIGN. The WSLg deadlock is an offscreen getCameraImage
racing the GUI's render thread. This mode renders NO cameras at all — no
referee scanning, no camera insets, no drone-camera renders (the registry is
built with cameras_enabled=False) — so it cannot hit that path. The 3D world
still draws in the GUI window; you watch and fly there.

HONEST CONSTRAINT (the whole point): this live mode shows the **3D world
only, NOT the FPV camera**. Showing a live camera feed here would reintroduce
the freeze. For camera / FPV / scan review, use `--record` (the offscreen MP4
cockpit). Two separate modes by design:
  - live_view  : interactive 3D + keyboard flying, no cameras, freeze-proof.
  - --record   : full FPV cockpit (cameras, scans, telemetry), offscreen.

Keyboard (see scripts/keyboardcontrol.py): WASD move, R/F up/down, Q/E yaw,
arrows tilt the (off-screen) camera. Drives the real send_manual_control.

Usage (needs a display; e.g. WSLg): python -m scripts.live_view --drone 0
"""

import argparse

from pyhulax import DroneAPI

from simcore.config import load_config
from simcore.log import get_logger


def live_registry(cfg, gui: bool = True):
    """Create the world for the live 3D-only viewer: a p.GUI window with
    cameras_enabled=False (no getCameraImage anywhere -> freeze-proof).
    gui is overridable so headless tests can verify the camera-disabled
    guard without opening a window."""
    from simcore.registry import get_registry
    return get_registry(cfg, gui=gui, cameras_enabled=False)


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(
        description="Live 3D-only viewer (no cameras, freeze-proof) + "
                    "keyboard flying. FPV/camera review = --record.")
    ap.add_argument("--config", default=None)
    ap.add_argument("--drone", type=int, default=0,
                    help="index of the drone to pilot with the keyboard")
    ap.add_argument("--seconds", type=float, default=None,
                    help="auto-exit after this many wall seconds (default: "
                         "until the window closes)")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    log = get_logger("live_view", cfg)
    from simcore.registry import shutdown_registry
    reg = live_registry(cfg, gui=True)
    try:
        log.info("live 3D-only viewer: cameras OFF (freeze-proof). Fly with "
                 "WASD / R-F / Q-E / arrows. FPV review = --record.")
        unit = cfg.drones.units[args.drone]
        d = DroneAPI()
        d.connect(unit.ip)
        d.takeoff(cfg.drones.takeoff_height_cm)

        import time
        from scripts.keyboardcontrol import run_keyboard_loop
        end = None if args.seconds is None else time.time() + args.seconds
        run_keyboard_loop(
            reg, d, rate_hz=20,
            stop_predicate=(None if end is None
                            else (lambda: time.time() >= end)))
    finally:
        shutdown_registry()


if __name__ == "__main__":
    main()
