#!/usr/bin/env python3
"""connect_check.py — READ-ONLY discover/connect + telemetry + LIVE UWB. No motion.

Verifies connection, the per-cage UWB origin (do the UWB positions match where the drones
physically sit?), and the ip↔tag pairing.

    python scripts/connect_check.py --real          # Dola-discover all 3 (cage origin from config)
    python scripts/connect_check.py --ip 10.0.0.11  # one drone by IP
    python scripts/connect_check.py                 # sim/default: config IPs

Public API only; real-only init via sdk_compat (telemetry-only, never arms).
"""

from __future__ import annotations

from typing import Callable, Dict

from mission.runtime import sdk_compat
from scripts.hardware_check import check_drone


def connect_and_telemetry(ips: Dict[int, str], *, uwb, make_api: Callable,
                          log: Callable = print) -> Dict[int, object]:
    """Connect each {tag: ip} and print READ-ONLY telemetry. Returns {tag: DroneCheck}."""
    out = {}
    for tag, ip in sorted(ips.items()):
        out[tag] = check_drone(make_api(), ip, tag, uwb, log=log)
    return out


def live_uwb(uwb, tags, *, reads: int = 10, sleep, log: Callable = print) -> None:
    """Stream live UWB tag positions (read-only) so the cage origin can be eyeballed."""
    for i in range(max(1, reads)):
        cells = []
        for t in sorted(tags):
            x, y, _t = uwb.get_tag_position(t)
            cells.append(f"tag{t}=({x:.2f},{y:.2f})" if x is not None else f"tag{t}=NONE")
        log(f"  UWB[{i:02d}]  " + "   ".join(cells))
        sleep(0.5)


def main(argv=None) -> int:
    import argparse

    import pyhulax
    from UWBParserThread import UWBParserThread

    from mission.config import load_config, load_real_config
    from mission.runtime.discovery import Discovery

    ap = argparse.ArgumentParser(description="READ-ONLY connect + telemetry + live UWB "
                                             "(no motion).")
    ap.add_argument("--real", action="store_true", help="real profile + Dola discovery")
    ap.add_argument("--ip", default=None, help="check one drone by IP")
    ap.add_argument("--reads", type=int, default=10, help="live-UWB samples to print")
    args = ap.parse_args(argv)

    cfg = load_real_config() if args.real else load_config()
    if args.ip is not None:
        rev = {ip: t for t, ip in cfg.ip_for_tag().items()}
        ips = {rev.get(args.ip, 0): args.ip}
    elif args.real:
        ips = Discovery.from_config(cfg, use_dola=True).resolve_ordered(log=print)
    else:
        ips = Discovery.from_config(cfg, use_dola=cfg.use_dola()).resolve()

    uwb = UWBParserThread(x_origin=cfg.uwb.origin_x, y_origin=cfg.uwb.origin_y)
    uwb.start()
    print("connect_check — READ-ONLY (no takeoff, no arming, no motion)\n")
    drones = {}
    try:
        import time
        checks = connect_and_telemetry(ips, uwb=uwb,
                                       make_api=lambda: pyhulax.DroneAPI(), log=print)
        drones = checks
        print("\nlive UWB (do these match where the drones physically sit?):")
        live_uwb(uwb, list(ips), reads=args.reads, sleep=time.sleep, log=print)
    finally:
        try:
            uwb.stop()
        except Exception:
            pass
    n_pass = sum(1 for c in drones.values() if c.passed)
    print(f"\n══ {n_pass}/{len(drones)} drone(s) PASS ══")
    return 0 if (drones and n_pass == len(drones)) else 1


if __name__ == "__main__":
    raise SystemExit(main())
