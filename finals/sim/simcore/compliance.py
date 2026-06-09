"""Compliance flagging — the brief's score-invalidation rules, made VISIBLE.

Simulator INTERNAL — observer-only. The Finals brief invalidates a run if a
drone flies OVER an obstacle or above the altitude cap. Rather than silently
killing the run, the sim FLAGS it: a flagged event the operator can SEE (in
--record + --dashboard) and a logs/compliance record — so you fix your nav,
not lose the run blind. This mutates NOTHING in the world; it only reads drone
pose + the arena footprints.

`check(cfg, drone, obstacles)` is a pure per-drone flag (used by DebugProbe so
both renderers can show it). `ComplianceMonitor.step(now)` (called on the sim
thread from the registry loop) logs violation onset/clear transitions.
"""

from pathlib import Path

from . import arena
from .log import get_logger


def check(cfg, drone, obstacles) -> dict:
    """Per-drone compliance flags (pure geometry, observer-only). FLAGS the
    no-fly-over-obstacle + altitude-cap rules; a flagged dict, never a crash.

    obstacles = the arena layout obstacles (arena frame); only GROUND crates
    (z0_m == 0) count as no-fly footprints — the elevated lintel does not.
    """
    comp = cfg.compliance
    airborne = bool(drone.flying)
    enabled = bool(comp.enabled)
    n, e = drone.arena_position()
    alt_m = float(drone.pos[2]) - float(cfg.arena.origin[2])
    margin = float(comp.margin_m)

    over_obstacle = False
    if airborne and enabled:
        for o in obstacles:
            if o.z0_m != 0.0:
                continue  # elevated (lintel) — not a ground crate footprint
            if arena.point_rect_dist_m(n, e, o.north, o.east,
                                       o.half_n, o.half_e) <= margin:
                over_obstacle = True
                break
    over_altitude = bool(airborne and enabled
                         and alt_m > float(comp.max_altitude_m))
    reasons = []
    if over_obstacle:
        reasons.append("over_crate")
    if over_altitude:
        reasons.append("altitude_cap")
    return {
        "enabled": enabled,
        "airborne": airborne,
        "altitude_m": round(alt_m, 3),
        "over_obstacle": over_obstacle,
        "over_altitude": over_altitude,
        "violation": bool(over_obstacle or over_altitude),
        "reasons": reasons,
    }


class ComplianceMonitor:
    """Logs compliance VIOLATION onset/clear transitions to logs/compliance.

    Stepped on the SIM THREAD from the registry loop (cheap geometry, throttled
    to check_period_s; file writes only on a transition — rare). Observer-only.
    The live FLAG for the renderers comes from `check()` via DebugProbe; this
    just keeps the durable record."""

    def __init__(self, registry):
        self._reg = registry
        self._cfg = registry.config
        self._period = float(self._cfg.compliance.check_period_s)
        self._next = 0.0
        self._violating = {}          # drone index -> currently violating?
        self._log = get_logger("compliance", self._cfg)
        self._fh = None

    def step(self, now: float) -> None:
        if not self._cfg.compliance.enabled or now < self._next:
            return
        self._next = now + self._period
        for d in self._reg.drones:
            flags = check(self._cfg, d, self._reg.layout.obstacles)
            was = self._violating.get(d.index, False)
            if flags["violation"] and not was:
                self._event(now, d.index, flags, "VIOLATION")
            elif was and not flags["violation"]:
                self._event(now, d.index, flags, "cleared")
            self._violating[d.index] = flags["violation"]

    def _event(self, now, index, flags, kind) -> None:
        reasons = ",".join(flags["reasons"]) or "ok"
        msg = (f"t={now:7.2f}s drone {index} {kind}: {reasons} "
               f"(alt {flags['altitude_m']:.2f} m)")
        self._log.warning(msg)
        try:
            if self._fh is None:
                d = Path(self._cfg.compliance.log_dir)
                d.mkdir(parents=True, exist_ok=True)
                self._fh = open(d / "compliance.log", "a")
            self._fh.write(msg + "\n")
            self._fh.flush()
        except OSError:
            pass  # logging must never break the sim

    def close(self) -> None:
        if self._fh is not None:
            self._fh.close()
            self._fh = None
