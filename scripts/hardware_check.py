#!/usr/bin/env python3
"""hardware_check.py — READ-ONLY hardware bring-up check (rung 1 of docs/SIM_VS_REAL.md).

**NO takeoff, NO arming, NO movement.** For each configured drone it connects, reads
`get_battery`, `get_position`, `get_orientation`, `get_altitude`, `get_obstacles`, and the
UWB `get_tag_position(tag_id)`, and prints a clear **PASS/FAIL** per drone:
  - connected?  - telemetry non-null?  - UWB returning real coords?

Uses ONLY the public pyhulax API. Real-only init is routed through
`runtime/sdk_compat.prepare_telemetry` (guarded, **telemetry-only — never arms**); teardown
through `sdk_compat.release`. Nothing here commands motion.

Run (REAL hardware — drop the sim PYTHONPATH/HULA_SIM_* so `import pyhulax` is the real SDK):
    python scripts/hardware_check.py                 # all configured drones
    python scripts/hardware_check.py --drone 10.0.0.11   # one at a time
Against the SIM:
    PYTHONPATH=~/codes/finals/sim:src HULA_SIM_CONFIG=~/codes/finals/sim/sim_config.yaml \
    python scripts/hardware_check.py
Exit code 0 iff every checked drone PASSes.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, List, Optional, Tuple

from mission.runtime import sdk_compat


@dataclass
class DroneCheck:
    ip: str
    tag_id: int
    connected: bool = False
    battery: Any = None
    position: Any = None
    orientation: Any = None
    altitude: Any = None
    obstacles: Any = None
    uwb: Tuple = (None, None, None)
    error: Optional[str] = None

    @property
    def telemetry_ok(self) -> bool:
        return all(v is not None for v in (self.battery, self.position,
                                           self.orientation, self.altitude,
                                           self.obstacles))

    @property
    def uwb_ok(self) -> bool:
        return self.uwb[0] is not None and self.uwb[1] is not None

    @property
    def passed(self) -> bool:
        return self.connected and self.telemetry_ok and self.uwb_ok


def _read(fn: Callable):
    """Read one telemetry value; return None if it raises (real SDK can raise before data)."""
    try:
        return fn()
    except Exception:
        return None


def check_drone(drone, ip: str, tag_id: int, uwb, *, log: Callable = print) -> DroneCheck:
    """Connect, do guarded telemetry-only init (NO arm), read everything, return a result.
    Commands NO motion."""
    res = DroneCheck(ip=ip, tag_id=tag_id)
    try:
        res.connected = bool(drone.connect(ip))
    except Exception as exc:
        res.error = repr(exc)
        log(_format(res))
        return res

    sdk_compat.prepare_telemetry(drone)            # guarded; NEVER arms / no motion

    res.battery = _read(drone.get_battery)
    res.position = _read(drone.get_position)
    res.orientation = _read(drone.get_orientation)
    res.altitude = _read(drone.get_altitude)
    res.obstacles = _read(drone.get_obstacles)
    try:
        res.uwb = tuple(uwb.get_tag_position(tag_id))
    except Exception as exc:
        res.error = repr(exc)
        res.uwb = (None, None, None)

    log(_format(res))
    return res


def run_check(cfg, *, uwb, make_api: Callable, only_ip: Optional[str] = None,
              log: Callable = print) -> List[DroneCheck]:
    """Check every configured drone (or just `only_ip`). `make_api()` yields a fresh
    DroneAPI per drone; each is released (guarded) after its read."""
    units = [u for u in cfg.drones if only_ip is None or u.ip == only_ip]
    if not units:
        raise SystemExit(f"no configured drone with ip {only_ip!r} "
                         f"(configured: {[u.ip for u in cfg.drones]})")
    results: List[DroneCheck] = []
    for u in units:
        drone = make_api()
        try:
            results.append(check_drone(drone, u.ip, u.tag_id, uwb, log=log))
        finally:
            try:
                sdk_compat.release(drone)          # guarded teardown (disconnect on real)
            except Exception:
                pass
    return results


def _format(res: DroneCheck) -> str:
    head = f"── tag {res.tag_id} @ {res.ip} " + "─" * max(0, 34 - len(res.ip))
    lines = [head]
    if not res.connected:
        lines.append(f"  connect            : FAIL  {res.error or ''}")
        lines.append("  ── RESULT: FAIL (not connected)")
        return "\n".join(lines)
    lines.append("  connect            : OK")
    b = res.battery
    lines.append(f"  battery            : {b} %" if b is not None
                 else "  battery            : FAIL (None)")
    p = res.position
    lines.append(f"  position (onboard) : x={p.x:.1f} y={p.y:.1f} z={p.z:.1f} cm"
                 if p is not None else "  position (onboard) : FAIL (None)")
    o = res.orientation
    lines.append(f"  orientation        : yaw={o.yaw:.1f} pitch={o.pitch:.1f} "
                 f"roll={o.roll:.1f} deg" if o is not None
                 else "  orientation        : FAIL (None)")
    a = res.altitude
    lines.append(f"  altitude (ToF)     : {a:.1f} cm" if a is not None
                 else "  altitude (ToF)     : FAIL (None)")
    ob = res.obstacles
    if ob is not None:
        flags = "".join(k[0].upper() if getattr(ob, k, False) else "-"
                        for k in ("forward", "back", "left", "right", "down"))
        lines.append(f"  obstacles [F B L R D]: {' '.join(flags)}")
    else:
        lines.append("  obstacles          : FAIL (None)")
    x, y, _t = res.uwb
    lines.append(f"  UWB tag {res.tag_id}          : ({x:.3f}, {y:.3f}) m" if x is not None
                 else f"  UWB tag {res.tag_id}          : FAIL (None,None,None) — tag not seen")
    lines.append(f"  ── RESULT: {'PASS' if res.passed else 'FAIL'}")
    return "\n".join(lines)


def main(argv=None) -> int:
    import argparse

    import pyhulax
    from UWBParserThread import UWBParserThread

    from mission.config import load_config

    ap = argparse.ArgumentParser(description="READ-ONLY drone bring-up check "
                                             "(no takeoff/arm/motion).")
    ap.add_argument("--drone", default=None, metavar="IP",
                    help="check only this drone IP (default: all configured)")
    args = ap.parse_args(argv)

    cfg = load_config()
    print("hardware_check — READ-ONLY (no takeoff, no arming, no motion)\n")
    uwb = UWBParserThread()
    uwb.start()
    try:
        results = run_check(cfg, uwb=uwb, make_api=lambda: pyhulax.DroneAPI(),
                            only_ip=args.drone)
    finally:
        try:
            uwb.stop()
        except Exception:
            pass

    n_pass = sum(r.passed for r in results)
    print(f"\n══ SUMMARY: {n_pass}/{len(results)} drone(s) PASS ══")
    for r in results:
        if not r.passed:
            why = []
            if not r.connected:
                why.append("not connected")
            elif not r.telemetry_ok:
                why.append("telemetry null")
            if not r.uwb_ok:
                why.append("UWB no fix")
            print(f"   FAIL tag {r.tag_id} @ {r.ip}: {', '.join(why)}")
    return 0 if (results and n_pass == len(results)) else 1


if __name__ == "__main__":
    raise SystemExit(main())
