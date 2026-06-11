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
    frame: Any                       # annotated BGR ndarray (or None)
    xy: Optional[Tuple[float, float]]
    t: float
    drone_id: Optional[int] = None   # which drone banked it (R4 — caption + provenance)
    path: Optional[str] = None        # on-disk annotated capture (logs/evidence/rover_<id>.png)


class MissionState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._evidence: Dict[int, Evidence] = {}

    def bank(self, marker_id: int, frame: Any, xy: Optional[Tuple[float, float]],
             t: float, *, drone_id: Optional[int] = None,
             path: Optional[str] = None) -> bool:
        """Record a confirmed capture. Returns True if this was a NEW id, False if it
        was already tagged (de-dup by id — first capture wins). `drone_id`/`path` carry
        the R4 provenance (who banked it + where the annotated PNG lives)."""
        with self._lock:
            if marker_id in self._evidence:
                return False
            self._evidence[marker_id] = Evidence(
                int(marker_id), frame, xy, float(t),
                drone_id=None if drone_id is None else int(drone_id),
                path=None if path is None else str(path))
            return True

    def set_path(self, marker_id: int, path: Optional[str]) -> None:
        """Attach the on-disk annotated-capture path to an already-banked id (the PNG is
        written only once the bank de-dup confirms this id is NEW)."""
        with self._lock:
            e = self._evidence.get(marker_id)
            if e is not None:
                e.path = None if path is None else str(path)

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
