#!/usr/bin/env python3
"""hover_test.py --ip <ip> --i-have-clear-space — ONE drone: takeoff 1.0 m, hover 5 s, land.

The first real-ARMING test. Works at home (no UWB needed). MOTION is gated behind
--i-have-clear-space. Station-keeps with zero manual sticks (heartbeat-like, never > 0.5 m/s
since there is no horizontal input). Lands in a `finally` (and on Ctrl-C).

Public API only; real-only init via sdk_compat (guarded).
"""

from __future__ import annotations

import os
import sys
import time

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from typing import Callable                                         # noqa: E402

from mission.runtime import sdk_compat                             # noqa: E402


def hover_once(drone, *, height_cm: int = 100, hover_s: float = 5.0, rate_hz: int = 20,
               sleep: Callable = time.sleep, log: Callable = print) -> None:
    """Takeoff → station-keep (zero sticks) for hover_s → land. Caller has already done
    `prepare_manual_control` and the clear-space gate. Horizontal sticks stay 0 (no drift)."""
    log(f"  takeoff {height_cm} cm…")
    drone.takeoff(height_cm)
    steps = max(1, int(hover_s * rate_hz))
    for _ in range(steps):
        drone.send_manual_control(0.0, 0.0, 0.0, 0.0)   # station-keep; NEVER any horizontal/up
        sleep(1.0 / rate_hz)
    log("  landing…")
    drone.land()


def main(argv=None) -> int:
    import argparse

    import pyhulax

    ap = argparse.ArgumentParser(description="ONE-drone takeoff/hover/land (real-arming test).")
    ap.add_argument("--ip", required=True, help="drone IP")
    ap.add_argument("--i-have-clear-space", action="store_true", dest="clear",
                    help="REQUIRED to arm/move — confirm the area is clear")
    ap.add_argument("--height-cm", type=int, default=100)
    ap.add_argument("--seconds", type=float, default=5.0)
    args = ap.parse_args(argv)

    if not args.clear:
        print("REFUSING to arm/move: pass --i-have-clear-space once the area is clear.")
        return 1

    d = pyhulax.DroneAPI()
    d.connect(args.ip)
    sdk_compat.prepare_manual_control(d, velocity_level=None)   # arms (guarded) — real flight
    try:
        hover_once(d, height_cm=args.height_cm, hover_s=args.seconds)
    except KeyboardInterrupt:
        print("\nCtrl-C — landing.")
    finally:
        try:
            d.land()
        except Exception:
            pass
        sdk_compat.release(d)
    print("hover_test done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
