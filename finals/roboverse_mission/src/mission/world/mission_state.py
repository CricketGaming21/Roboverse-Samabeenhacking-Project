"""MissionState — the lock-guarded tagged-id set + per-id capture evidence.

This is the source of truth for "what have we scored". De-dup is **by ArUco id** (HARD
invariant #5): banking the same id twice keeps a single tagged entry. Drives the
"done?" check and the rubric/evidence export (P10).
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Optional, Set, Tuple


@dataclass
class Evidence:
    marker_id: int
    frame: Any                       # annotated RGB ndarray (or None)
    xy: Optional[Tuple[float, float]]
    t: float


class MissionState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._evidence: Dict[int, Evidence] = {}

    def bank(self, marker_id: int, frame: Any, xy: Optional[Tuple[float, float]],
             t: float) -> bool:
        """Record a confirmed capture. Returns True if this was a NEW id, False if it
        was already tagged (de-dup by id — first capture wins)."""
        with self._lock:
            if marker_id in self._evidence:
                return False
            self._evidence[marker_id] = Evidence(int(marker_id), frame, xy, float(t))
            return True

    def is_tagged(self, marker_id: int) -> bool:
        with self._lock:
            return marker_id in self._evidence

    def tagged(self) -> Set[int]:
        with self._lock:
            return set(self._evidence)

    def remaining(self, all_ids: Iterable[int]) -> Set[int]:
        with self._lock:
            return set(all_ids) - set(self._evidence)

    def evidence(self) -> Dict[int, Evidence]:
        with self._lock:
            return dict(self._evidence)

    def count(self) -> int:
        with self._lock:
            return len(self._evidence)
