"""Referee: scores DISTINCT ArUco ids actually seen by the drones' cameras.

Simulator INTERNAL — mission code must never import simcore. The mission
stays pure pyhulax: it never calls a scoring API; the referee renders its own
frames and judges them with its own cv2.aruco (plan §7).

THE GATE — an id is banked only when, on ONE drone, a detection passes ALL of:
  - apparent marker side >= scoring.min_marker_px        (close/clear enough)
  - fully inside the frame by >= scoring.frame_margin_px (not clipped)
  - held for >= scoring.hold_frames CONSECUTIVE referee frames (stable read)
Score = number of distinct ids banked, deduplicated globally across all
drones (first drone to complete a hold gets the attribution). The same gate
scores pad ids and rover ids.

scoring.mode: "auto" (default) banks automatically per the gate;
"explicit" only banks on the INTERNAL hook register_capture(drone_index)
(not part of the public pyhulax surface).

The referee runs in its own thread, paced at config.camera.fps in SIM time;
renders go through the sim-thread queue, detection happens here. Only flying
drones are rendered/judged.
"""

import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np

from . import aruco_assets
from .log import get_logger


@dataclass(frozen=True)
class BankedID:
    marker_id: int
    drone_index: int
    sim_time: float


class Referee:
    """Owns the scoring state + the judging thread for one sim world."""

    def __init__(self, registry):
        self._reg = registry
        cfg = registry.config
        self._cfg = cfg
        self._log = get_logger("referee", cfg)
        self._detector = cv2.aruco.ArucoDetector(
            aruco_assets.get_dictionary(cfg.aruco.dictionary),
            cv2.aruco.DetectorParameters())
        self._min_px = float(cfg.scoring.min_marker_px)
        self._margin_px = float(cfg.scoring.frame_margin_px)
        self._hold_frames = int(cfg.scoring.hold_frames)
        self._auto = cfg.scoring.mode == "auto"
        if cfg.scoring.mode not in ("auto", "explicit"):
            raise ValueError(f"unknown scoring.mode: {cfg.scoring.mode!r}")

        self._lock = threading.Lock()
        self._banked = {}        # marker_id -> BankedID (insertion ordered)
        self._holds = {}         # drone_index -> {marker_id: consecutive}
        self._last_valid = {}    # drone_index -> set of gate-passing ids
        self._stop = threading.Event()
        self._thread = None

    # ------------------------------------------------------------------ #
    # Public (sim-side) reads
    # ------------------------------------------------------------------ #

    def score(self) -> int:
        with self._lock:
            return len(self._banked)

    def banked_ids(self) -> set:
        with self._lock:
            return set(self._banked)

    def banked(self) -> list:
        """BankedID records sorted by the sim time they were banked."""
        with self._lock:
            return sorted(self._banked.values(), key=lambda b: b.sim_time)

    def format_scoreboard(self) -> str:
        rows = [f"SCOREBOARD  score={self.score()} (distinct marker ids)"]
        for b in self.banked():
            rows.append(f"  id {b.marker_id:3d}  by drone {b.drone_index}"
                        f"  at t={b.sim_time:6.1f}s")
        return "\n".join(rows)

    def register_capture(self, drone_index: int) -> list:
        """INTERNAL explicit-capture hook (scoring.mode == "explicit"):
        bank whatever gate-passing ids this drone saw on its latest frame.
        Never exposed on the public pyhulax surface."""
        now = self._reg.sim_time()
        banked_now = []
        with self._lock:
            for mid in self._last_valid.get(drone_index, set()):
                if mid not in self._banked:
                    self._banked[mid] = BankedID(mid, drone_index, now)
                    banked_now.append(mid)
        for mid in banked_now:
            self._log.info("capture: id %d banked by drone %d", mid,
                           drone_index)
        return banked_now

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop,
                                        name="hula-referee", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)

    # ------------------------------------------------------------------ #
    # Judging
    # ------------------------------------------------------------------ #

    def _gate_ok(self, pts: np.ndarray) -> bool:
        """Instantaneous gate: size + fully-inside-with-margin."""
        side = max(float(np.linalg.norm(pts[i] - pts[(i + 1) % 4]))
                   for i in range(4))
        if side < self._min_px:
            return False
        w, h = self._cfg.camera.width, self._cfg.camera.height
        return bool(pts[:, 0].min() >= self._margin_px
                    and pts[:, 0].max() <= w - 1 - self._margin_px
                    and pts[:, 1].min() >= self._margin_px
                    and pts[:, 1].max() <= h - 1 - self._margin_px)

    def _judge(self, drone_index: int, detections) -> list:
        """One referee frame for one drone. detections: [(id, pts(4,2))].
        Returns ids banked by this frame."""
        valid = {mid for mid, pts in detections if self._gate_ok(pts)}
        now = self._reg.sim_time()
        banked_now = []
        with self._lock:
            holds = self._holds.setdefault(drone_index, {})
            for mid in list(holds):
                if mid not in valid:
                    del holds[mid]  # consecutive run broken
            for mid in valid:
                holds[mid] = holds.get(mid, 0) + 1
                if (self._auto and holds[mid] >= self._hold_frames
                        and mid not in self._banked):
                    self._banked[mid] = BankedID(mid, drone_index, now)
                    banked_now.append(mid)
            self._last_valid[drone_index] = valid
        for mid in banked_now:
            self._log.info("banked id %d (drone %d, t=%.1fs)", mid,
                           drone_index, now)
        return banked_now

    def _judge_frame(self, drone_index: int, rgb: np.ndarray) -> list:
        corners, ids, _ = self._detector.detectMarkers(
            cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
        detections = [] if ids is None else [
            (int(mid), quad[0]) for mid, quad in zip(ids.flatten(), corners)]
        return self._judge(drone_index, detections)

    def _loop(self) -> None:
        reg = self._reg
        period_sim = 1.0 / float(self._cfg.camera.fps)
        poll_s = max(0.001, min(
            period_sim / max(self._cfg.meta.real_time_factor, 1e-9) / 4.0,
            0.05))
        next_due = reg.sim_time()
        while not self._stop.is_set():
            if not reg.is_alive():
                break
            now = reg.sim_time()
            if now < next_due:
                time.sleep(poll_s)
                continue
            for i, drone in enumerate(reg.drones):
                if not drone.flying:  # grounded camera judges nothing
                    with self._lock:
                        self._holds.pop(i, None)
                        self._last_valid.pop(i, None)
                    continue
                try:
                    rgb = reg.render_camera(drone)
                except (RuntimeError, TimeoutError):
                    return  # sim shut down
                self._judge_frame(i, rgb)
            next_due += period_sim
            if next_due < reg.sim_time():  # fell behind: resync
                next_due = reg.sim_time() + period_sim
