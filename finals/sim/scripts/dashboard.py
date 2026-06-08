"""Concurrent command dashboard — a live read-only per-drone console panel.

Observer-only: reads simcore.DebugProbe.snapshot() and renders it as text.
It MUTATES NOTHING and never touches pyhulax — it is a window onto the sim
for the human, not a control or sensing path the mission could use.

Per drone it shows: scenario phase; telemetry (UWB pos, get_position +
drift error, altitude, heading, battery); the current move_to goal +
progress in blocking mode OR the live send_manual_control stick inputs in
manual mode; the five barrier flags as on/off indicators; and a scan-status
line (markers banked by this drone, with sim-time).

render_dashboard(snapshot) is a pure function (headless-testable). The
CommandDashboard class wraps a DebugProbe and refreshes the console on an
interval; opened by the --dashboard flag alongside the 3D sim.
"""

import threading
import time

from simcore.debug import DebugProbe

_CLEAR = "\033[2J\033[H"  # ANSI clear + home


def _fmt_stick(stick) -> str:
    return "  ".join(f"{k[:3].upper()}{stick[k]:+.2f}"
                     for k in ("forward", "right", "up", "rotate"))


def _fmt_command(d) -> str:
    g = d["goal"]
    if g is None:
        return "CMD     idle"
    if d["mode"] == "manual":
        return "MANUAL  " + _fmt_stick(g["stick"])
    tgt = g.get("target_world")
    where = (f"->({tgt[0]:.2f},{tgt[1]:.2f},{tgt[2]:.2f})"
             if tgt is not None else "")
    rem = g.get("remaining_m")
    prog = f" | {rem:.2f} m to go" if rem is not None else ""
    return f"CMD     {g['kind']}{where}{prog}"


def _fmt_barriers(sensors) -> str:
    rays = sensors["rays"]
    cells = []
    for key, ltr in (("forward", "F"), ("back", "B"), ("left", "L"),
                     ("right", "R"), ("down", "D")):
        on = bool(rays[key]["blocked"])
        cells.append(f"{ltr}{'[X]' if on else '[ ]'}")
    return "BARRIER " + " ".join(cells)


def _fmt_scan(snapshot, drone_index) -> str:
    ref = snapshot.get("referee", {})
    banked = ref.get("banked", {})
    mine = sorted((int(mid), info["sim_time"])
                  for mid, info in banked.items()
                  if info.get("drone") == drone_index)
    if not mine:
        return "SCAN    (none banked by this drone)"
    ids = ", ".join(f"{mid}@{t:.1f}s" for mid, t in mine)
    return f"SCAN    banked [{ids}]"


def render_dashboard(snapshot) -> str:
    """Pure text render of one DebugProbe snapshot (no side effects)."""
    phase = snapshot.get("referee", {})
    score = phase.get("score")
    head = (f"=== HULA COMMAND DASHBOARD ===   t={snapshot['sim_time']:6.1f}s"
            f"   distinct rover ids: {score if score is not None else '-'}")
    lines = [head]
    for d in snapshot["drones"]:
        n, e = d["true"]["arena_ne_m"][:2]
        est = d["estimate"]
        drift_cm = est["drift_error_m"] * 100.0
        alt = d["sensors"]["altitude_cm"]
        hdg = d["orientation_deg"]["yaw"]
        state = "FLY" if d["flying"] else ("RDY" if d["connected"] else "OFF")
        lines.append("")
        lines.append(f" drone {d['index']} ({d['ip']})  {state}  "
                     f"batt {d['battery_pct']:5.1f}%  [{d['mode']}]")
        lines.append(f"   UWB(n,e) {n:6.2f},{e:6.2f} m   "
                     f"est {est['takeoff_cm'][0]:6.1f},"
                     f"{est['takeoff_cm'][1]:6.1f} cm "
                     f"(drift {drift_cm:4.1f} cm)   "
                     f"alt {alt:5.0f} cm  hdg {hdg:5.1f} deg")
        lines.append("   " + _fmt_command(d))
        lines.append("   " + _fmt_barriers(d["sensors"]))
        lines.append("   " + _fmt_scan(snapshot, d["index"]))
    return "\n".join(lines)


class CommandDashboard:
    """Live console dashboard refreshed from a DebugProbe. Read-only."""

    def __init__(self, registry, refresh_hz: float = 4.0):
        self._reg = registry
        self._probe = DebugProbe(registry)
        self._period = 1.0 / max(float(refresh_hz), 0.1)
        self._stop = threading.Event()
        self._thread = None

    def snapshot_text(self) -> str:
        """One rendered panel from a fresh snapshot (for tests / one-shot)."""
        return render_dashboard(self._probe.snapshot())

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop,
                                        name="hula-dashboard", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3)

    def _loop(self) -> None:
        while not self._stop.is_set() and self._reg.is_alive():
            try:
                text = render_dashboard(self._probe.snapshot())
            except (RuntimeError, TimeoutError):
                break
            print(_CLEAR + text, flush=True)
            self._stop.wait(self._period)
