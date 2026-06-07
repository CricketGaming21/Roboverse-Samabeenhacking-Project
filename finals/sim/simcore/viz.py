"""Top-down 2D arena view (matplotlib) — headless-safe.

Simulator INTERNAL — mission code must never import simcore.

Shows drones + their paths, rovers, pads, approximate covered area, banked
marker ids (green) and the current score. render_png() draws straight onto an
Agg canvas (never needs a display — CI/batch safe); run_live() lazily imports
pyplot and must run on the MAIN thread. config.viz.enabled gates the
auto/live view in run_sim; explicit render_png calls always work.
"""

import math
import threading
import time

import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle

from . import frames
from .log import get_logger

_CELL_M = 0.25          # coverage grid pitch
_PATH_MIN_STEP_M = 0.02  # path decimation
_DRONE_COLORS = ("tab:blue", "tab:orange", "tab:purple")


class TopDownView:
    """Samples world state (thread-safe via the registry) and draws it."""

    def __init__(self, registry):
        self._reg = registry
        cfg = registry.config
        self._cfg = cfg
        self._log = get_logger("viz", cfg)
        self._lock = threading.Lock()
        self._paths = [[] for _ in registry.drones]
        self._latest = [None] * len(registry.drones)
        self._rovers = []
        self._rover_ids = [r.marker_id for r in registry.rovers]
        self._sim_time = 0.0
        nn = int(math.ceil(cfg.arena.length_m / _CELL_M))
        ne = int(math.ceil(cfg.arena.width_m / _CELL_M))
        self._covered = np.zeros((nn, ne), dtype=bool)
        self._stop = threading.Event()
        self._thread = None

    # ------------------------------------------------------------------ #
    # Sampling
    # ------------------------------------------------------------------ #

    def sample(self) -> None:
        """Pull one state snapshot (callable from any thread)."""
        reg, cfg = self._reg, self._cfg

        def _read():
            ds = []
            for d in reg.drones:
                n, e = d.arena_position()
                n2, e2 = frames.world_to_arena(
                    cfg, d.pos[0] + math.cos(d.yaw) * 0.5,
                    d.pos[1] + math.sin(d.yaw) * 0.5)
                ds.append((n, e, n2 - n, e2 - e, d.flying, float(d.pos[2])))
            return ds, [r.arena_position() for r in reg.rovers], \
                reg.clock.now()
        try:
            drones, rovers, now = reg.run_on_sim_thread(_read)
        except (RuntimeError, TimeoutError):
            return
        with self._lock:
            self._rovers = rovers
            self._sim_time = now
            for i, st in enumerate(drones):
                self._latest[i] = st
                n, e, _, _, flying, alt = st
                if not flying:
                    continue
                path = self._paths[i]
                if not path or math.dist(path[-1], (n, e)) > _PATH_MIN_STEP_M:
                    path.append((n, e))
                self._mark_covered(n, e, max(0.3, 0.5 * alt))

    def _mark_covered(self, n, e, radius_m) -> None:
        nn, ne = self._covered.shape
        lo_n = max(0, int((n - radius_m) / _CELL_M))
        hi_n = min(nn - 1, int((n + radius_m) / _CELL_M))
        lo_e = max(0, int((e - radius_m) / _CELL_M))
        hi_e = min(ne - 1, int((e + radius_m) / _CELL_M))
        for gi in range(lo_n, hi_n + 1):
            for gj in range(lo_e, hi_e + 1):
                cn, ce = (gi + 0.5) * _CELL_M, (gj + 0.5) * _CELL_M
                if math.hypot(cn - n, ce - e) <= radius_m:
                    self._covered[gi, gj] = True

    def start_sampling(self) -> None:
        """Background sampler at config.viz.fps (wall clock)."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._sample_loop,
                                        name="hula-viz", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)

    def _sample_loop(self) -> None:
        period = 1.0 / float(self._cfg.viz.fps)
        while not self._stop.is_set() and self._reg.is_alive():
            self.sample()
            self._stop.wait(period)

    # ------------------------------------------------------------------ #
    # Drawing
    # ------------------------------------------------------------------ #

    def _draw(self, ax) -> None:
        cfg = self._cfg
        L, W = cfg.arena.length_m, cfg.arena.width_m
        with self._lock:
            covered = self._covered.copy()
            paths = [list(p) for p in self._paths]
            latest = list(self._latest)
            rovers = list(self._rovers)
            sim_t = self._sim_time
        banked = (self._reg.referee.banked_ids()
                  if self._reg.referee is not None else set())
        score = len(banked)

        ax.clear()
        ax.set_xlim(-0.4, W + 0.4)
        ax.set_ylim(-0.4, L + 0.4)
        ax.set_aspect("equal")
        ax.set_xlabel("east (m)")
        ax.set_ylabel("north (m)")
        ax.add_patch(Rectangle((0, 0), W, L, fill=False, ec="black", lw=2))

        gi, gj = np.nonzero(covered)
        if gi.size:
            ax.scatter((gj + 0.5) * _CELL_M, (gi + 0.5) * _CELL_M,
                       marker="s", s=42, c="palegreen", alpha=0.5, zorder=0)

        for o in self._reg.layout.obstacles:
            ax.add_patch(Rectangle((o.east - o.half_e, o.north - o.half_n),
                                   2 * o.half_e, 2 * o.half_n,
                                   fc="peru", ec="saddlebrown", zorder=2))

        half = cfg.aruco.pad_marker_size_m / 2
        for pad in cfg.pads:
            fc = "limegreen" if pad.id in banked else "white"
            ax.add_patch(Rectangle((pad.east - half, pad.north - half),
                                   2 * half, 2 * half, fc=fc, ec="black",
                                   zorder=3))
            ax.annotate(str(pad.id), (pad.east, pad.north + half + 0.06),
                        ha="center", fontsize=8, zorder=3)

        for (n, e), rid in zip(rovers, self._rover_ids):
            color = "limegreen" if rid in banked else "crimson"
            ax.plot(e, n, "s", color=color, ms=9, zorder=4)
            ax.annotate(str(rid), (e, n + 0.14), ha="center", fontsize=8,
                        zorder=4)

        for i, (path, st) in enumerate(zip(paths, latest)):
            c = _DRONE_COLORS[i % len(_DRONE_COLORS)]
            if len(path) > 1:
                ax.plot([p[1] for p in path], [p[0] for p in path],
                        color=c, lw=1.2, alpha=0.8, zorder=5)
            if st is not None:
                n, e, dn, de, flying, _alt = st
                rot = -math.degrees(math.atan2(de, dn))  # 0 = pointing north
                ax.plot(e, n, marker=(3, 0, rot), ms=12, color=c, zorder=6)
                ax.annotate(f"d{i}", (e + 0.12, n + 0.12), color=c,
                            fontsize=9, zorder=6)

        ids_txt = ",".join(str(i) for i in sorted(banked)) or "-"
        ax.set_title(f"score {score}   banked [{ids_txt}]   "
                     f"t={sim_t:.1f}s sim")

    def render_png(self, path: str) -> None:
        """Draw the current state to a PNG — never needs a display."""
        fig = Figure(figsize=(6, 9))
        FigureCanvasAgg(fig)
        self._draw(fig.add_subplot(111))
        fig.savefig(path, dpi=110, bbox_inches="tight")
        self._log.info("top-down view saved to %s", path)

    def run_live(self, duration_s=None) -> None:
        """Interactive updating window. MAIN THREAD ONLY (matplotlib GUI)."""
        try:
            import matplotlib.pyplot as plt
            plt.ion()
            fig, ax = plt.subplots(figsize=(6, 9))
        except Exception as e:
            self._log.warning("live view unavailable (%s); running headless",
                              e)
            if duration_s:
                time.sleep(duration_s)
            return
        period = 1.0 / float(self._cfg.viz.fps)
        end = None if duration_s is None else time.time() + duration_s
        try:
            while ((end is None or time.time() < end)
                   and plt.fignum_exists(fig.number)
                   and self._reg.is_alive()):
                self._draw(ax)
                plt.pause(max(0.01, period))
        except KeyboardInterrupt:
            pass
        finally:
            plt.ioff()
            plt.close(fig)
