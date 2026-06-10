#!/usr/bin/env python3
"""yaw_calibrate.py --ip <ip> --i-have-clear-space — see which way "forward" moves.

Cage only (needs UWB). Takeoff, send a small +forward `send_manual_control` for ~2 s, print
the UWB position before/after so you can set `frame.yaw_offset_deg` / `invert_forward` /
`invert_right` in config (no code change). MOTION gated behind --i-have-clear-space; gentle
stick (the SDK clamps to ≤ 0.5 m/s); lands in a `finally`.
"""

from __future__ import annotations

import os
import sys
import time

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (_ROOT, os.path.join(_ROOT, "src")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from typing import Callable, Optional, Tuple                       # noqa: E402

from mission.runtime import sdk_compat                             # noqa: E402


def nudge_forward(drone, uwb, tag_id: int, *, height_cm: int = 100, fwd_stick: float = 0.3,
                  secs: float = 2.0, rate_hz: int = 20, sleep: Callable = time.sleep,
                  log: Callable = print) -> Tuple[Optional[Tuple], Optional[Tuple]]:
    """Takeoff → small +forward for `secs` → land. Prints UWB before/after + the delta.
    `fwd_stick` is small; the SDK clamps the resulting speed to ≤ 0.5 m/s."""
    drone.takeoff(height_cm)
    bx, by, _bt = uwb.get_tag_position(tag_id)
    steps = max(1, int(secs * rate_hz))
    for _ in range(steps):
        drone.send_manual_control(float(fwd_stick), 0.0, 0.0, 0.0)   # +forward only (gentle)
        sleep(1.0 / rate_hz)
    ax, ay, _at = uwb.get_tag_position(tag_id)
    drone.land()
    if bx is not None and ax is not None:
        dn, de = ax - bx, ay - by
        log(f"  UWB before ({bx:.2f}, {by:.2f}) -> after ({ax:.2f}, {ay:.2f})")
        log(f"  +forward moved: north {dn:+.2f} m, east {de:+.2f} m")
        log("  → set frame.yaw_offset_deg / invert_forward / invert_right so +forward = +north")
    else:
        log("  UWB had no fix — check the cage/origin/tag id")
    return (bx, by), (ax, ay)


def main(argv=None) -> int:
    import argparse

    import pyhulax
    from UWBParserThread import UWBParserThread

    from mission.config import load_config

    ap = argparse.ArgumentParser(description="Cage yaw/sign calibration nudge.")
    ap.add_argument("--ip", required=True, help="drone IP")
    ap.add_argument("--tag", type=int, default=0, help="this drone's UWB tag id")
    ap.add_argument("--i-have-clear-space", action="store_true", dest="clear")
    ap.add_argument("--stick", type=float, default=0.3)
    ap.add_argument("--seconds", type=float, default=2.0)
    args = ap.parse_args(argv)

    if not args.clear:
        print("REFUSING to arm/move: pass --i-have-clear-space once the area is clear.")
        return 1

    cfg = load_config()
    d = pyhulax.DroneAPI()
    d.connect(args.ip)
    sdk_compat.prepare_manual_control(d, velocity_level=None)
    uwb = UWBParserThread(x_origin=cfg.uwb.origin_x, y_origin=cfg.uwb.origin_y)
    uwb.start()
    try:
        nudge_forward(d, uwb, args.tag, fwd_stick=args.stick, secs=args.seconds)
    except KeyboardInterrupt:
        print("\nCtrl-C — landing.")
    finally:
        try:
            d.land()
        except Exception:
            pass
        try:
            uwb.stop()
        except Exception:
            pass
        sdk_compat.release(d)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
