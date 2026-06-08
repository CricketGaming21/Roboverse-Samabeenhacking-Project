"""TWO referees — the sim scores both stages; mission code never calls either.

Simulator INTERNAL — mission code must never import simcore.

PART 1 — LandingScorer (DEPLOY): when a drone's land() completes during
DEPLOY, the horizontal distance from the touchdown point to its candidate
pad centre (arena frame) is measured. It scores ONLY if that pad is
valid AND designated AND within scoring.landing.tolerance_m; one drone per
pad (assignment: nearest_unclaimed | fixed via drones.units[].pad_id).
Landing on the invalid decoy, a non-designated spare, or off-pad records an
attempt but scores nothing. (Judging validity FROM THE MARKER is mission
work — the sim only consults the config flag.)

PART 2 — Referee (AMBUSH): renders each flying drone's camera and judges
with its own cv2.aruco. THE GATE — an id is banked only when, on ONE drone,
a detection passes ALL of:
  - apparent marker side >= scoring.min_marker_px        (close/clear enough)
  - fully inside the frame by >= scoring.frame_margin_px (not clipped)
  - held for >= scoring.hold_frames CONSECUTIVE referee frames (stable read)
Deduplicated globally across drones. Targets are the ROVER marker ids ONLY
— pads are never part-2 targets — and judging runs ONLY during AMBUSH.
scoring.mode auto banks per the gate; "explicit" banks via the INTERNAL
register_capture hook (never on the public pyhulax surface).
"""

import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from . import aruco_assets
from .log import get_logger
from .scenario import AMBUSH, DEPLOY


@dataclass(frozen=True)
class BankedID:
    marker_id: int
    drone_index: int
    sim_time: float


@dataclass(frozen=True)
class LandingResult:
    """One DEPLOY landing attempt, scored or not."""
    drone_index: int
    pad_id: int          # candidate pad (-1 if none was assignable)
    pad_valid: bool
    pad_designated: bool
    error_m: float       # horizontal touchdown error to that pad's centre
    sim_time: float      # time-to-land (sim seconds since episode start)
    scored: bool


class LandingScorer:
    """Part-1 referee: landing accuracy during DEPLOY (no thread — the
    drone's land() completion calls record_landing on the sim thread)."""

    def __init__(self, registry):
        self._reg = registry
        cfg = registry.config.scoring.landing
        if cfg.assignment not in ("nearest_unclaimed", "fixed"):
            raise ValueError(
                f"unknown scoring.landing.assignment: {cfg.assignment!r}")
        self._cfg = cfg
        self._log = get_logger("landing", registry.config)
        self._lock = threading.Lock()
        self.attempts = []     # every DEPLOY landing, scored or not
        self._scored = {}      # drone_index -> LandingResult (first success)
        self._claimed = set()  # pad ids already won by a drone

    # ---- SIM THREAD (from the drone's land-goal completion) ----------- #

    def record_landing(self, drone) -> None:
        scn = self._reg.scenario
        if scn is None or scn.phase != DEPLOY:
            return  # part 1 only scores during DEPLOY
        now = self._reg.clock.now()
        north, east = drone.arena_position()
        pad = self._candidate_pad(drone, north, east)
        if pad is None:
            result = LandingResult(drone.index, -1, False, False,
                                   float("inf"), now, False)
        else:
            err = math.hypot(north - pad.north, east - pad.east)
            ok = (pad.valid and pad.designated
                  and err <= self._cfg.tolerance_m
                  and pad.id not in self._claimed
                  and drone.index not in self._scored)
            result = LandingResult(drone.index, pad.id, pad.valid,
                                   pad.designated, err, now, ok)
        with self._lock:
            self.attempts.append(result)
            if result.scored:
                self._scored[drone.index] = result
                self._claimed.add(result.pad_id)
        self._log.info(
            "drone %d landed at (%.2f, %.2f) t=%.1fs -> pad %s err %.0fcm "
            "%s", drone.index, north, east, now,
            result.pad_id, result.error_m * 100,
            "SCORED" if result.scored else "no score")

    def _candidate_pad(self, drone, north, east):
        pads = self._reg.config.pads
        if self._cfg.assignment == "fixed":
            target = drone.spec.pad_id
            return next((p for p in pads if p.id == target), None)
        unclaimed = [p for p in pads if p.id not in self._claimed]
        if not unclaimed:
            return None
        return min(unclaimed,
                   key=lambda p: math.hypot(north - p.north, east - p.east))

    # ---- reads (any thread) ------------------------------------------ #

    def score(self) -> int:
        with self._lock:
            return len(self._scored)

    def results(self) -> list:
        """Successful landings, sorted by time-to-land."""
        with self._lock:
            return sorted(self._scored.values(), key=lambda r: r.sim_time)

    def all_attempts(self) -> list:
        with self._lock:
            return list(self.attempts)

    def claimed_ids(self) -> set:
        with self._lock:
            return set(self._claimed)


def format_combined_scoreboard(registry) -> str:
    """Both stages on one board: part-1 landings + part-2 rover snapshots."""
    rows = ["SCOREBOARD"]
    ls = registry.landing_scorer
    if ls is not None:
        weighted = registry.config.scoring.landing.time_weighted
        rows.append(f" PART 1 — landings: {ls.score()} scored"
                    + (" (time-weighted)" if weighted else ""))
        for r in ls.results():
            rows.append(f"   drone {r.drone_index} -> pad {r.pad_id}"
                        f"  err {r.error_m * 100:5.1f}cm"
                        f"  t={r.sim_time:6.1f}s")
        for r in ls.all_attempts():
            if not r.scored:
                why = ("invalid pad" if not r.pad_valid
                       else "not designated" if not r.pad_designated
                       else "off target / pad taken")
                rows.append(f"   drone {r.drone_index} -> pad {r.pad_id}"
                            f"  err {r.error_m * 100:5.1f}cm  NO SCORE"
                            f" ({why})")
    ref = registry.referee
    if ref is not None:
        rows.append(f" PART 2 — snapshots: {ref.score()} distinct rover ids")
        for b in ref.banked():
            rows.append(f"   id {b.marker_id:3d}  by drone {b.drone_index}"
                        f"  at t={b.sim_time:6.1f}s")
    return "\n".join(rows)


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
        # Part-2 targets: ROVER markers only — pads never count.
        self._targets = set(cfg.rovers.marker_ids[:cfg.rovers.count])

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
            (int(mid), quad[0]) for mid, quad in zip(ids.flatten(), corners)
            if int(mid) in self._targets]  # rover ids only; pads filtered out
        banked_now = self._judge(drone_index, detections)
        # Deliberate scan logging: fire ONCE on the banking event (banked_now
        # is non-empty only when an id is first banked — never per frame).
        if banked_now:
            quads = dict(detections)
            for mid in banked_now:
                self._save_scan(drone_index, mid, rgb, quads.get(mid))
        return banked_now

    def _save_scan(self, drone_index: int, marker_id: int, rgb, quad) -> None:
        """Write logs/scans/<...>.png with the just-banked marker boxed + a
        log line — visual proof of the scan. Observer-side; gated by config.
        Failures never disrupt scoring."""
        side = (max(float(np.linalg.norm(quad[k] - quad[(k + 1) % 4]))
                    for k in range(4)) if quad is not None else 0.0)
        now = self._reg.sim_time()
        self._log.info("SCAN id %d by drone %d  %.0f px  t=%.1fs", marker_id,
                       drone_index, side, now)
        if not self._cfg.logging.save_scans:
            return
        try:
            from . import camfeed
            out_dir = Path(self._cfg.logging.scans_dir)
            out_dir.mkdir(parents=True, exist_ok=True)
            bgr = camfeed.box_marker(self._cfg, rgb, quad) if quad is not None \
                else cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            cv2.putText(bgr, f"SCANNED id {marker_id}  drone {drone_index}"
                        f"  t={now:.1f}s", (8, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            path = out_dir / (f"scan_id{marker_id}_drone{drone_index}"
                              f"_t{now:06.1f}.png")
            cv2.imwrite(str(path), bgr)
        except Exception as e:  # logging must never break the referee
            self._log.warning("scan image save failed for id %d: %s",
                              marker_id, e)

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
            scn = reg.scenario
            if scn is None or scn.phase != AMBUSH:
                # Part 2 judges ONLY during AMBUSH: no rendering, no holds.
                with self._lock:
                    self._holds.clear()
                    self._last_valid.clear()
                next_due = now + period_sim
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
