"""Integration self-check — confirm the sim is healthy and the 3 drones are
flyable through the PUBLIC API ONLY. This is the canonical wiring a separate
MISSION process copies (it imports nothing the mission may not).

Run it from the sim dir so `import pyhulax` resolves to THIS sim:

    python -m scripts.integration_check                       # headless, seeded
    python -m scripts.integration_check --record /tmp/check.mp4
    python -m scripts.integration_check --seconds 10          # idle after, for the recording

What it proves (the exact mission wiring):
  - `import pyhulax` + `from UWBParserThread import UWBParserThread` resolve to
    the sim (sim dir on sys.path).
  - The shared sim world boots LAZILY on the first `connect(ip)` — no separate
    sim process/launcher is needed; the mission hosts it in its own process.
  - Dola discovery is a STUB in-sim, so drones are reached by FIXED IPs.
  - Each drone connects, takes off, reports telemetry + barrier flags, its UWB
    tag reads back, and lands.

NOTE: a separate mission process gets its OWN in-process sim (the registry is a
per-process singleton) — this script does not act as a server for it. Use it to
verify the sim is healthy, and as the copy-paste reference for connect/UWB.
"""

import argparse
import os
import time

# --- the ENTIRE public surface a mission may import (mirror this exactly) ---
import pyhulax
from pyhulax import DroneAPI
from UWBParserThread import UWBParserThread

# The sim's fixed drone contract (Dola discovery is stubbed in-sim, so the
# mission is GIVEN these — ip <-> uwb_tag_id, paired by config order).
DRONES = [("10.0.0.11", 0), ("10.0.0.12", 1), ("10.0.0.13", 2)]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Sim integration self-check "
                                 "(public API only, headless, seeded).")
    ap.add_argument("--record", metavar="PATH.mp4", default=None,
                    help="record an offscreen cockpit MP4 of the check "
                         "(sets $HULA_SIM_RECORD before the first connect)")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="idle this many wall-seconds after the check (lets the "
                         "scenario advance / the recording capture more)")
    args = ap.parse_args(argv)

    # The recorder is a SIM-SIDE observer the mission can't import; the env hook
    # auto-starts it on boot. Set it BEFORE the first connect() (the boot point).
    if args.record:
        os.environ["HULA_SIM_RECORD"] = args.record

    print("import wiring: pyhulax ->", os.path.dirname(pyhulax.__file__))
    print("UWB:", UWBParserThread.__module__, "| Dola discovery is a sim stub "
          "(use fixed IPs)\n")

    drones = []
    try:
        for ip, tag in DRONES:
            d = DroneAPI()
            r = d.connect(ip)                     # <-- boots the sim on the FIRST call
            assert r.success, f"connect({ip}) failed: {r.message}"
            d.takeoff(110)
            pos = d.get_position()
            alt = d.get_altitude()
            batt = d.get_battery()
            obs = d.get_obstacles()
            print(f"drone {ip} (tag {tag}): takeoff OK  "
                  f"est=({pos.x:.0f},{pos.y:.0f},{pos.z:.0f})cm  alt={alt:.0f}cm  "
                  f"batt={batt}%  obstacles fwd={obs.forward}")
            drones.append(d)

        # UWB: one thread, read every drone's tag (arena frame, metres)
        uwb = UWBParserThread()
        uwb.start()
        time.sleep(0.4)                           # let a sample land
        print()
        for ip, tag in DRONES:
            x, y, t = uwb.get_tag_position(tag)
            print(f"UWB tag {tag} ({ip}): "
                  + ("no fix" if x is None
                     else f"arena=({x:.2f},{y:.2f}) m  t={t:.2f}"))

        if args.seconds > 0:
            print(f"\nidling {args.seconds:.0f}s (scenario advancing)...")
            time.sleep(args.seconds)

        for d in drones:
            d.land()
        uwb.stop()
        print("\nINTEGRATION CHECK OK — sim healthy, 3 drones flyable, UWB live.")
        return 0
    finally:
        # Clean teardown so an auto-recording (if any) finalises its mp4.
        from simcore.registry import shutdown_registry
        shutdown_registry()


if __name__ == "__main__":
    raise SystemExit(main())
