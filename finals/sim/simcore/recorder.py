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

from . import camfeed
from .debug import DebugProbe
from .log import get_logger

_PHASE_LABEL = {"deploy": "DEPLOY", "ambush": "AMBUSH", "done": "DONE"}
_GREEN = (90, 230, 120)
_AMBER = (40, 190, 240)
_RED = (60, 60, 235)
_GREY = (170, 170, 170)
_CAM_H_FRAC = 0.42        # camera feed fraction of a cockpit column
_FLASH_SECONDS = 1.5      # how long an id's first-bank flash lasts (sim s)
_PROX_DIRS = (("forward", 0, -1), ("back", 0, 1),   # (flag, dx, dy) on icon
              ("left", -1, 0), ("right", 1, 0))


class ArenaRecorder:
    """Captures the live run to PATH.mp4 (or a PNG sequence fallback)."""

    def __init__(self, registry, path: str):
        self._reg = registry
        self._cfg = registry.config
        self._rc = self._cfg.record
        self._path = Path(path)
        self._log = get_logger("recorder", self._cfg)
        self._arena_w = int(self._rc.width)
        self._arena_h = int(self._rc.height)
        # Per-drone COCKPIT COLUMN below the arena view: camera feed +
        # telemetry panel + car-style proximity graphic, stacked. The band
        # grows when the panels are on (Phase 25); the canvas is taller than
        # the arena render either way (Phase 23).
        self._insets = bool(self._rc.show_camera_insets)
        self._telemetry = bool(self._rc.show_telemetry)
        self._proximity = bool(self._rc.show_proximity)
        self._cam_h = self._arena_h // 4 if self._insets else 0
        # camera tile + a panel region (telemetry/proximity) below it
        panel_h = (self._arena_h * 5 // 16) if (
            self._insets and (self._telemetry or self._proximity)) else 0
        self._panel_h = panel_h
        self._band_h = self._cam_h + panel_h
        self._probe = DebugProbe(registry) if self._band_h else None
        self._w = self._arena_w
        self._h = self._arena_h + self._band_h
        self._writer = None          # cv2.VideoWriter, or None for PNG mode
        self._png_dir = None         # set in PNG-fallback mode
        self.frames_written = 0
        self.last_frame = None       # most recent overlaid BGR frame
        # Acquisition UI state: which ids the referee has banked (acquired),
        # and a brief flash window on each id's FIRST bank.
        self._banked_seen = set()
        self._flash = {}             # marker_id -> sim time first seen banked
        self._stop = threading.Event()
        self._thread = None

    # ------------------------------------------------------------------ #
    # One frame: third-person arena render + info overlay (BGR)
    # ------------------------------------------------------------------ #

    def capture_frame(self):
        """Render + overlay one composited frame now; returns BGR or None.

        Top: the third-person arena view with the info overlay. Bottom (when
        record.show_camera_insets): a row of the three per-drone camera feeds
        — the SAME offscreen frames the referee judges — each with detected
        ArUco markers outlined + id labelled (camfeed.annotate_markers)."""
        rgb = self._reg.render_arena(self._arena_w, self._arena_h)
        if rgb is None:
            return None
        arena = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        self._overlay(arena)
        if not self._insets:
            return arena
        canvas = np.zeros((self._h, self._w, 3), dtype=np.uint8)
        canvas[:self._arena_h] = arena
        canvas[self._arena_h:] = self._cockpit_band()
        return canvas

    def _acquisition_state(self):
        """(banked_ids, flash_ids) for the acquisition UI; tracks each id's
        first bank to flash it briefly. SIM-time based."""
        ref = self._reg.referee
        banked = set(ref.banked_ids()) if ref is not None else set()
        now = self._reg.sim_time()
        for mid in banked - self._banked_seen:   # first time seen banked
            self._flash[mid] = now
        self._banked_seen |= banked
        flash = {mid for mid, t in self._flash.items()
                 if now - t < _FLASH_SECONDS}
        return banked, flash

    def _cockpit_band(self):
        """The bottom band: a per-drone COLUMN — camera feed (with the
        target-acquisition UI) + telemetry panel + proximity graphic.
        Read-only DebugProbe snapshot feeds the panels (referee_view=False)."""
        drones = self._reg.drones
        snap = (self._probe.snapshot(referee_view=False)
                if self._probe is not None else {"drones": []})
        dsnaps = {d["index"]: d for d in snap.get("drones", [])}
        banked, flash = self._acquisition_state()
        col_w = self._w // 3
        band = np.full((self._band_h, self._w, 3), 30, dtype=np.uint8)
        for i in range(3):
            x0 = i * col_w
            x1 = self._w if i == 2 else x0 + col_w
            col = self._drone_column(
                drones[i] if i < len(drones) else None,
                dsnaps.get(i), x1 - x0, banked, flash)
            band[:, x0:x1] = col
            cv2.rectangle(band, (x0, 0), (x1 - 1, self._band_h - 1),
                          (90, 90, 90), 1)
        return band

    def _drone_column(self, drone, dsnap, w, banked=(), flash=()):
        """One cockpit column: camera tile on top, then telemetry + proximity
        panel below. Robust to a missing/idle drone (placeholder camera)."""
        col = np.full((self._band_h, w, 3), 30, dtype=np.uint8)
        col[:self._cam_h] = self._drone_tile(
            drone, w, self._cam_h, banked, flash, self._status_line(dsnap))
        if self._panel_h:
            col[self._cam_h:] = self._drone_panel(dsnap, w, self._panel_h)
        return col

    def _status_line(self, dsnap):
        """Compact per-drone status: mode + current goal or stick inputs."""
        if dsnap is None:
            return None
        mode = dsnap.get("mode", "idle")
        g = dsnap.get("goal")
        if mode == "manual" and g and "stick" in g:
            s = g["stick"]
            return (f"d{dsnap['index']} MANUAL "
                    f"F{s['forward']:+.1f} R{s['right']:+.1f}")
        if mode == "blocking" and g:
            return f"d{dsnap['index']} BLOCKING {g['kind']}"
        return f"d{dsnap['index']} idle"

    def _drone_panel(self, dsnap, w, h):
        """Telemetry text (left) + car-style proximity graphic (right)."""
        panel = np.full((h, w, 3), 22, dtype=np.uint8)
        if dsnap is None:
            return panel
        prox_w = int(w * 0.32) if self._proximity else 0
        if self._telemetry:
            y = 16
            for text, color in self._telemetry_lines(dsnap):
                cv2.putText(panel, text, (6, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.36, color, 1, cv2.LINE_AA)
                y += 16
        if self._proximity:
            self._draw_proximity(panel, w - prox_w, 0, prox_w, h, dsnap)
        return panel

    def _telemetry_lines(self, d):
        """The per-drone telemetry fields, as (text, color) rows."""
        n, e = d["true"]["arena_ne_m"][:2]
        est = d["estimate"]["takeoff_cm"]
        drift = d["estimate"]["drift_error_m"] * 100.0
        o = d["orientation_deg"]
        course = d["course_deg"]
        course_s = f"{course:.0f}" if course is not None else "--"
        g = d["goal"]
        if d["mode"] == "manual" and g and "stick" in g:
            s = g["stick"]
            cmd = (f"STK F{s['forward']:+.1f} R{s['right']:+.1f} "
                   f"U{s['up']:+.1f} Y{s['rotate']:+.1f}")
        elif g:
            cmd = f"CMD {g['kind']}"
        else:
            cmd = "CMD idle"
        batt_c = _GREEN if d["battery_pct"] > 25 else _AMBER
        uwb_c = _GREEN if d["uwb_ok"] else _RED
        comp = d.get("compliance", {})
        rows = []
        if comp.get("violation"):
            why = "OVER CRATE" if comp.get("over_obstacle") else "ALT CAP"
            rows.append((f"!! VIOLATION: {why}", _RED))
        rows += [
            (f"d{d['index']} {'FLY' if d['flying'] else 'GND'}  "
             f"batt {d['battery_pct']:.0f}%", batt_c),
            (f"UWB n,e {n:5.2f},{e:5.2f} m", _GREY),
            (f"EST cm {est[0]:5.0f},{est[1]:5.0f} (drift {drift:3.0f}cm)",
             _GREY),
            (f"spd {d['speed_mps']:.2f} m/s  hdg {o['yaw']:.0f}", _GREY),
            (f"course {course_s}  cam {d['camera_pitch_deg']:.0f}", _GREY),
            (f"ypr {o['yaw']:.0f}/{o['pitch']:.0f}/{o['roll']:.0f}", _GREY),
            (cmd, _GREY),
            ("UWB: OK" if d["uwb_ok"] else "UWB: NO FIX", uwb_c),
        ]
        return rows

    def _draw_proximity(self, panel, x0, y0, w, h, d):
        """Car-style parking-sensor graphic: a drone icon with the five
        barrier directions (fwd/back/left/right + a down dot) lighting RED
        when that boolean flag is set — mirrors get_obstacles() exactly."""
        rays = d["sensors"]["rays"]
        cx, cy = x0 + w // 2, y0 + h // 2
        r = min(w, h) // 2 - 10
        seg = max(6, r // 3)
        cv2.circle(panel, (cx, cy), max(4, seg // 2), (200, 200, 200), 1)
        for name, dx, dy in _PROX_DIRS:
            lit = bool(rays.get(name, {}).get("blocked"))
            px, py = cx + dx * r, cy + dy * r
            cv2.rectangle(panel, (px - seg // 2, py - seg // 2),
                          (px + seg // 2, py + seg // 2),
                          _RED if lit else (70, 70, 70),
                          -1 if lit else 1)
        down_lit = bool(rays.get("down", {}).get("blocked"))
        cv2.circle(panel, (cx, cy), max(3, seg // 3),
                   _RED if down_lit else (70, 70, 70), -1 if down_lit else 1)
        cv2.putText(panel, "PROX", (x0 + 4, y0 + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, _GREY, 1, cv2.LINE_AA)

    def _drone_tile(self, drone, w, h, banked=(), flash=(), status=None):
        """One inset: the drone's live frame with the target-ACQUISITION UI
        (yellow DETECTED -> green ACQUIRED boxes, real cv2.aruco), or a
        placeholder when it isn't flying (DEPLOY pre-takeoff). Never crashes.
        The label is a compact per-drone status line (mode + goal/sticks)."""
        tile = np.full((h, w, 3), 40, dtype=np.uint8)
        flying = drone is not None and getattr(drone, "flying", False)
        if flying:
            # render the inset camera at a HIGHER internal resolution (4:3)
            # so the marker + detect/acquire box stay crisp after downscale.
            ih = int(self._rc.inset_render_h)
            rgb = self._reg.render_camera(drone, width=ih * 4 // 3, height=ih)
            if rgb is not None:
                overlaid, _found = camfeed.acquisition_overlay(
                    self._cfg, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                    banked_ids=banked, flash_ids=flash)
                tile = cv2.resize(overlaid, (w, h))
                label = status or f"d{drone.index}"
            else:
                label = (status or f"d{drone.index}") + " (no frame)"
        else:
            cv2.putText(tile, "idle (pre-takeoff)", (10, h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1)
            label = status or (f"d{drone.index}" if drone is not None else "-")
        cv2.rectangle(tile, (0, 0), (w, 20), (0, 0, 0), -1)
        cv2.putText(tile, label, (6, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    _GREEN, 1, cv2.LINE_AA)
        return tile

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
        cv2.rectangle(bgr, (0, 0), (self._w, 110), (0, 0, 0), -1)
        cv2.putText(bgr, line1, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (255, 255, 255), 2)
        cv2.putText(bgr, line2, (12, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    _GREEN, 2)
        cv2.putText(bgr, line3, (12, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (180, 220, 255), 1)
        self._draw_legend(bgr, 12, 100)

    def _draw_legend(self, bgr, x, y) -> None:
        """A compact colour key so the cockpit is self-describing."""
        items = (("scored", _GREEN), ("detected", (0, 255, 255)),
                 ("warn", _AMBER), ("blocked", _RED))
        cv2.putText(bgr, "KEY", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    _GREY, 1, cv2.LINE_AA)
        cx = x + 52
        for text, color in items:
            cv2.rectangle(bgr, (cx, y - 9), (cx + 12, y + 1), color, -1)
            cv2.putText(bgr, text, (cx + 16, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.42, _GREY, 1, cv2.LINE_AA)
            cx += 30 + 9 * len(text)

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
