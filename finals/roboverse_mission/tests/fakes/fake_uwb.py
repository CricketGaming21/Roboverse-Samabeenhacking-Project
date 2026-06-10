"""Fake `UWBParserThread` — deterministic, world-coupled, no serial/network.

Mirrors the real/sim `UWBParserThread` surface used by the mission:
    UWBParserThread(x_origin=0, y_origin=0, serial_port=None, baud_rate=921600)
    .start() / .stop() / .get_tag_position(tag_id) -> (x_m, y_m, t)

Differences that keep tests deterministic (and that match the real *contract*):
  - No background sampling thread on wall-clock: `get_tag_position` reads the
    shared `FakeWorld` truth *synchronously* and stamps it with the sim clock
    (`world.clock`), so a UWB-derived speed (Δpos/Δt) is reproducible.
  - Truth (metres, arena x=North/y=East) + optional Gaussian noise (seeded rng).
  - Unmapped tag, stochastic dropout, or a forced-unseen tag -> (None, None, None).
"""

from __future__ import annotations

import threading
from typing import Optional, Set, Tuple

import numpy as np

from .fake_pyhulax import FakeWorld, get_active_world


class FakeUWBParserThread(threading.Thread):
    def __init__(self, x_origin: float = 0.0, y_origin: float = 0.0,
                 serial_port=None, baud_rate: int = 921600, *,
                 world: Optional[FakeWorld] = None,
                 noise_std_m: float = 0.0, dropout_prob: float = 0.0,
                 seed: int = 1234) -> None:
        super().__init__(daemon=True)
        self.serial_port = serial_port or "/dev/ttySIM_UWB"
        self.baud_rate = baud_rate
        # Per-cage origin: the latest real UWBParserThread ADDS it to the reported position.
        # Default 0.0 → identical to before; non-zero shifts the fix into the cage frame.
        self.origin_x = float(x_origin)
        self.origin_y = float(y_origin)
        self._world = world if world is not None else get_active_world()
        self.noise_std_m = float(noise_std_m)
        self.dropout_prob = float(dropout_prob)
        self._rng = np.random.default_rng(seed)
        self._force_unseen: Set[int] = set()
        self.running = False

    # -- thread surface (no-op; reads are synchronous) -------------------- #
    def run(self) -> None:
        self.running = True

    def start(self) -> None:          # don't actually spin a wall-clock thread
        self.running = True

    def stop(self) -> None:
        self.running = False

    # -- test controls ---------------------------------------------------- #
    def set_unseen(self, tag_id: int, unseen: bool = True) -> None:
        if unseen:
            self._force_unseen.add(tag_id)
        else:
            self._force_unseen.discard(tag_id)

    # -- the real API ----------------------------------------------------- #
    def get_tag_position(self, tag_id: int) -> Tuple:
        if tag_id in self._force_unseen:
            return (None, None, None)
        truth = self._world.truth_xy(tag_id)
        if truth is None:
            return (None, None, None)
        if self.dropout_prob > 0.0 and self._rng.random() < self.dropout_prob:
            return (None, None, None)
        nx, ny = (self._rng.normal(0.0, self.noise_std_m, 2)
                  if self.noise_std_m > 0.0 else (0.0, 0.0))
        return (truth[0] + float(nx) + self.origin_x,
                truth[1] + float(ny) + self.origin_y, self._world.clock)
