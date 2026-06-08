"""Offscreen MP4 recording of the REAL headless run.

Simulator INTERNAL — observer-only. A PASSIVE observer inside the one real
run: it renders extra offscreen frames of the live world (the same guarded
EGL path the drone cameras use — never p.GUI) and writes them to video. It
does NOT re-simulate and does NOT change sim behaviour; same seed -> same
run -> same video, and the overlaid scoreboard equals the run's printed one.

A background thread samples by SIM TIME (a frame every 1/record.fps sim
seconds, not every physics tick), renders the third-person arena view via
registry.render_arena, draws an info overlay, and writes through
cv2.VideoWriter (mp4v) — falling back to a PNG sequence if the codec is
unavailable. Off unless a path is given (no --record => zero overhead).
"""

import threading
import time
from pathlib import Path

import cv2
import numpy as np

from .log import get_logger

_PHASE_LABEL = {"deploy": "DEPLOY", "ambush": "AMBUSH", "done": "DONE"}


class ArenaRecorder:
    """Captures the live run to PATH.mp4 (or a PNG sequence fallback)."""

    def __init__(self, registry, path: str):
        self._reg = registry
        self._cfg = registry.config
        self._rc = self._cfg.record
        self._path = Path(path)
        self._log = get_logger("recorder", self._cfg)
        self._w, self._h = int(self._rc.width), int(self._rc.height)
        self._writer = None          # cv2.VideoWriter, or None for PNG mode
        self._png_dir = None         # set in PNG-fallback mode
        self.frames_written = 0
        self.last_frame = None       # most recent overlaid BGR frame
        self._stop = threading.Event()
        self._thread = None

    # ------------------------------------------------------------------ #
    # One frame: third-person arena render + info overlay (BGR)
    # ------------------------------------------------------------------ #

    def capture_frame(self):
        """Render + overlay one frame now; returns BGR ndarray or None."""
        rgb = self._reg.render_arena(self._w, self._h)
        if rgb is None:
            return None
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        self._overlay(bgr)
        return bgr

    def _overlay(self, bgr) -> None:
        reg = self._reg
        scn = getattr(reg, "scenario", None)
        phase = _PHASE_LABEL.get(scn.phase if scn else "", "?")
        landings = reg.landing_scorer.score() if reg.landing_scorer else 0
        rovers = reg.referee.score() if reg.referee else 0
        banked = (sorted(reg.referee.banked_ids()) if reg.referee else [])
        line1 = f"{phase}   t={reg.sim_time():6.1f}s"
        line2 = (f"P1 landings {landings}   P2 rovers {rovers}   "
                 f"score {landings + rovers}")
        line3 = "ids " + (",".join(str(i) for i in banked) or "-")
        cv2.rectangle(bgr, (0, 0), (self._w, 86), (0, 0, 0), -1)
        cv2.putText(bgr, line1, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (255, 255, 255), 2)
        cv2.putText(bgr, line2, (12, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (180, 255, 180), 2)
        cv2.putText(bgr, line3, (12, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (180, 220, 255), 1)

    # ------------------------------------------------------------------ #
    # Writer (mp4v, PNG-sequence fallback)
    # ------------------------------------------------------------------ #

    def _open_writer(self) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(self._path), fourcc, float(self._rc.fps),
                                 (self._w, self._h))
        if writer.isOpened():
            self._writer = writer
            self._log.info("recording -> %s (%dx%d @ %d fps, mp4v)",
                           self._path, self._w, self._h, self._rc.fps)
        else:
            self._png_dir = self._path.with_suffix("")
            self._png_dir = Path(str(self._png_dir) + "_frames")
            self._png_dir.mkdir(parents=True, exist_ok=True)
            self._log.warning("mp4v codec unavailable — writing PNG sequence "
                              "to %s/", self._png_dir)

    def _write(self, bgr) -> None:
        if self._writer is not None:
            self._writer.write(bgr)
        else:
            cv2.imwrite(str(self._png_dir / f"frame_{self.frames_written:06d}"
                            ".png"), bgr)
        self.frames_written += 1
        self.last_frame = bgr

    # ------------------------------------------------------------------ #
    # Lifecycle — sim-time-paced background capture
    # ------------------------------------------------------------------ #

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._open_writer()
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop,
                                        name="hula-recorder", daemon=True)
        self._thread.start()

    def stop(self) -> int:
        """Stop capture, flush the writer; returns frames written."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=10)
        if self._writer is not None:
            self._writer.release()
            self._writer = None
        self._log.info("recording finished: %d frames -> %s",
                       self.frames_written,
                       self._png_dir or self._path)
        return self.frames_written

    def _loop(self) -> None:
        period_sim = 1.0 / float(self._rc.fps)
        next_due = self._reg.sim_time()
        while not self._stop.is_set():
            if not self._reg.is_alive():
                break
            now = self._reg.sim_time()
            if now < next_due:
                # poll faster than the frame period (scaled by real-time fac)
                time.sleep(min(period_sim
                               / max(self._cfg.meta.real_time_factor, 1e-9)
                               / 4.0, 0.02))
                continue
            frame = self.capture_frame()
            if frame is None:
                break
            self._write(frame)
            next_due += period_sim
            if next_due < self._reg.sim_time():  # fell behind: resync
                next_due = self._reg.sim_time() + period_sim
