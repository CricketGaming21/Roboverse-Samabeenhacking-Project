"""Sim time + real-time factor.

Simulator INTERNAL — mission code must never import simcore.

SimClock tracks SIM time: it only advances when the sim thread executes a
physics step (advance(dt)). The real-time factor lives in config and is
honoured by the registry's pacing loop, not here — the clock just counts.
Blocking-command completion and timeouts (Phase 2) must use this clock, never
wall time, so faster-than-real runs still behave.
"""

import threading


class SimClock:
    """Monotonic sim-time counter. One writer (the sim thread), many readers."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._now_s = 0.0

    def advance(self, dt_s: float) -> None:
        """Advance sim time by one physics step. Sim thread only."""
        with self._lock:
            self._now_s += dt_s

    def now(self) -> float:
        """Current sim time in seconds since world boot. Any thread."""
        with self._lock:
            return self._now_s
