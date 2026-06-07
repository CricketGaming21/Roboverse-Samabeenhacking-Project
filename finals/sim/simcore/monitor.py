"""Per-drone command-rate watchdog + thrash (preemption) monitor.

Simulator INTERNAL — mission code must never import simcore.

Catches the qualifier failure mode: reactive commands overwriting each other
faster than they can complete. record_command/record_preempt are called from
the goal installer on the SIM THREAD; intervals are measured in SIM time.
Reflex 'avoid_step' goals are sim-internal and not rate-counted (their
preemptions ARE counted — a reflex interrupting a move is still a preempt).
"""

import threading
from collections import deque

from .log import get_logger


class CommandMonitor:
    """Counts commands, too-fast re-commands, and goal preemptions."""

    def __init__(self, cfg, clock):
        self._log = get_logger("monitor", cfg)
        self._clock = clock
        self._min_interval = 1.0 / float(cfg.monitor.max_cmd_rate_hz)
        self._warn_preempt = bool(cfg.monitor.warn_on_preempt)
        self._lock = threading.Lock()
        self._last_cmd = {}      # drone_index -> sim time of last command
        self._recent = {}        # drone_index -> deque[(sim_time, kind)]
        self.commands = {}       # drone_index -> total commands
        self.rate_warnings = {}  # drone_index -> too-fast re-command count
        self.preemptions = {}    # drone_index -> unfinished-goal preempts

    def record_command(self, drone_index: int, kind: str) -> None:
        now = self._clock.now()
        with self._lock:
            self.commands[drone_index] = self.commands.get(drone_index, 0) + 1
            self._recent.setdefault(
                drone_index, deque(maxlen=20)).append((now, kind))
            last = self._last_cmd.get(drone_index)
            self._last_cmd[drone_index] = now
            too_fast = last is not None and (now - last) < self._min_interval
            if too_fast:
                self.rate_warnings[drone_index] = \
                    self.rate_warnings.get(drone_index, 0) + 1
        if too_fast:
            self._log.warning(
                "thrash: drone %d re-commanded (%s) after %.3fs sim — faster "
                "than max_cmd_rate_hz allows (min interval %.2fs)",
                drone_index, kind, now - last, self._min_interval)

    def record_preempt(self, drone_index: int, old_kind: str,
                       new_kind: str) -> None:
        with self._lock:
            self.preemptions[drone_index] = \
                self.preemptions.get(drone_index, 0) + 1
        if self._warn_preempt:
            self._log.warning(
                "thrash: drone %d goal '%s' preempted by '%s' before it "
                "finished", drone_index, old_kind, new_kind)

    def recent_commands(self, drone_index: int) -> list:
        """[(sim_time, kind)] for the most recent commands (read-only)."""
        with self._lock:
            return list(self._recent.get(drone_index, ()))

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "commands": dict(self.commands),
                "rate_warnings": dict(self.rate_warnings),
                "preemptions": dict(self.preemptions),
            }

    def format_report(self) -> str:
        snap = self.snapshot()
        drones = sorted(set(snap["commands"]) | set(snap["rate_warnings"])
                        | set(snap["preemptions"]))
        rows = ["THRASH REPORT"]
        if not drones:
            rows.append("  (no commands recorded)")
        for i in drones:
            rows.append(
                f"  drone {i}: {snap['commands'].get(i, 0)} commands, "
                f"{snap['rate_warnings'].get(i, 0)} rate warnings, "
                f"{snap['preemptions'].get(i, 0)} preemptions")
        return "\n".join(rows)
