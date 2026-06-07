"""Smoke test: connect -> takeoff -> fly a square -> land. PROVES THE SIM RUNS.

This is NOT a mission — no detection, no strategy; just the public pyhulax
control surface driven end to end against the booted world.
Usage (from the project root): python -m scripts.smoke_test
"""

import math
import sys

from pyhulax import DroneAPI
from pyhulax.core import Direction

from simcore.config import load_config
from simcore.registry import get_registry, shutdown_registry


def main() -> None:
    cfg = load_config()
    reg = get_registry(cfg)
    ip = cfg.drones.units[0].ip

    drone = DroneAPI()
    print(f"connecting to {ip} ...")
    drone.connect(ip)
    try:
        start_pos, _ = reg.drone_world_pose(0)

        r = drone.takeoff(100)
        print(f"takeoff: {r.message}")
        for leg in range(1, 5):
            r = drone.move(Direction.FORWARD, 100)
            print(f"  leg {leg}: {r.message}")
            r = drone.rotate(90)
            print(f"  leg {leg}: {r.message}")
        r = drone.land()
        print(f"land: {r.message}")

        end_pos, _ = reg.drone_world_pose(0)
        closure_cm = math.dist(start_pos[:2], end_pos[:2]) * 100.0
        print(f"battery: {drone.get_battery()}%   "
              f"sim time: {reg.sim_time():.1f}s   "
              f"square closure error: {closure_cm:.1f} cm")
        if not r or closure_cm > 5.0:
            print("SMOKE TEST FAILED")
            sys.exit(1)
        print("SMOKE TEST PASSED")
    finally:
        shutdown_registry()


if __name__ == "__main__":
    main()
