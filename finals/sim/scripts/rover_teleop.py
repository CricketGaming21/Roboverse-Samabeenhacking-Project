"""SSH-friendly teleop for ONE evasive rover — terminal stdin keys, NO window.

The mixed convoy (rovers.motion: mixed) has 2 EVASIVE rovers modelling the
human-teleoperated opponents. This hook lets a human drive one of them live
over SSH: it reads single keystrokes from the terminal (raw stdin, cbreak
mode) and maps them to the rover's drive — there is NO GUI window anywhere
(headless/WSLg-safe; the freeze-proof rule holds). It can also be driven
programmatically (the web dashboard could call apply_keys/apply_drive).

Keys: W/A/S/D drive (north/west/south/east, arena frame), SPACE or X stop,
Q quit. Drive is applied via the registry on the sim thread; it stays fresh
for evasive.teleop_timeout_s, so holding nothing coasts to a stop.

Usage (in the SAME process/run as a mixed-convoy sim, e.g. wired by a flag,
or standalone for a quick drive):
    python -m scripts.rover_teleop          # boots a mixed sim and drives rover
"""

import select
import sys

from simcore.config import load_config
from simcore.rover_model import keys_to_rover_drive, personality_for
from simcore.registry import get_registry, shutdown_registry


class RoverTeleop:
    """Headless driver for one evasive rover (no window). apply_keys/apply_drive
    are pure-ish hooks the stdin loop OR the web dashboard can call."""

    def __init__(self, registry, rover_index: int = None):
        self._reg = registry
        cfg = registry.config
        ev = cfg.rovers.mixed.evasive
        self._speed = float(ev.speed_mps)
        if rover_index is None:
            # default target: the configured evasive rover (auto block + index)
            n_auto = len(cfg.rovers.mixed.auto_ids)
            rover_index = n_auto + int(ev.teleop_index)
        self.rover_index = rover_index
        self._rover = registry.rovers[rover_index]
        if personality_for(cfg, rover_index) != "evasive":
            raise ValueError(f"rover {rover_index} is not an evasive rover — "
                             f"teleop drives one of the evasive block")

    def apply_drive(self, v_north: float, v_east: float):
        """Push an arena-frame (m/s) drive to the rover on the sim thread."""
        self._reg.run_on_sim_thread(
            lambda: self._rover.set_teleop(v_north, v_east))
        return (v_north, v_east)

    def apply_keys(self, keys):
        """Map held keys -> drive and apply it (headless; no window)."""
        vn, ve = keys_to_rover_drive(keys, self._speed)
        return self.apply_drive(vn, ve)

    def stop(self):
        return self.apply_drive(0.0, 0.0)


def read_key_loop(teleop: RoverTeleop) -> None:
    """Raw-stdin key loop (SSH terminal; needs a TTY). No window opened."""
    if not sys.stdin.isatty():
        print("rover_teleop: stdin is not a TTY — use apply_keys() "
              "programmatically (e.g. from the web dashboard).")
        return
    import termios
    import tty
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    print("teleop: W/A/S/D drive, SPACE/X stop, Q quit (no window)")
    try:
        tty.setcbreak(fd)
        while True:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                ch = sys.stdin.read(1).lower()
                if ch == "q":
                    break
                if ch in (" ", "x"):
                    teleop.stop()
                elif ch in ("w", "a", "s", "d"):
                    teleop.apply_keys({ch})
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        teleop.stop()


def main(argv=None) -> None:
    import argparse
    ap = argparse.ArgumentParser(
        description="SSH-friendly teleop for one evasive rover (stdin keys, "
                    "no window).")
    ap.add_argument("--config", default=None)
    args = ap.parse_args(argv)
    cfg = load_config(args.config)
    cfg.rovers.motion = "mixed"
    cfg.scenario.phases = "ambush"        # rovers active immediately
    reg = get_registry(cfg)
    try:
        teleop = RoverTeleop(reg)
        read_key_loop(teleop)
    finally:
        shutdown_registry()


if __name__ == "__main__":
    main()
