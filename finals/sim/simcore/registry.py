"""The shared world: sim thread + command queue + lazily-initialised singleton.

Simulator INTERNAL — mission code must never import simcore.

THE ONE-THREAD RULE: every pybullet.* call happens on the sim thread owned by
SimRegistry. Anything else (pyhulax facade, UWB thread, tests, scripts) hands
work over via run_on_sim_thread() and waits for the result. Mixing threads
silently corrupts PyBullet state or segfaults.

The step loop is fixed-timestep (physics.dt_s of SIM time per step) and paces
itself against the wall clock scaled by meta.real_time_factor: rtf 2.0 runs
the sim twice as fast as real time. If rendering or load makes it fall behind,
it catches up by stepping multiple times per iteration (bounded by
physics.max_catchup_steps) and re-anchors if hopelessly behind.
"""

import pkgutil
import queue
import threading
import time

import pybullet as p

from . import (arena, camera, compliance, drone_model, frames, monitor,
               rover_model, scenario, scoring, world)
from .clock import SimClock
from .config import SimConfig, load_config
from .log import get_logger

_BOOT_TIMEOUT_S = 30.0
_CALL_TIMEOUT_S = 10.0


class SimRegistry:
    """Owns the PyBullet client, the world, the clock, and the sim thread.

    Constructing one boots the world (on the sim thread) and starts stepping.
    """

    def __init__(self, config: SimConfig = None, gui: bool = False,
                 cameras_enabled: bool = True) -> None:
        self.config = config if config is not None else load_config()
        self.gui = bool(gui)         # fixed at boot: p.GUI window vs DIRECT
        # cameras_enabled=False is the FREEZE-PROOF live-3D mode: NO
        # getCameraImage anywhere (no referee scanning, no render_camera, no
        # render_arena) so a p.GUI window can never deadlock against an
        # offscreen render. The 3D world still renders in the GUI itself.
        self.cameras_enabled = bool(cameras_enabled)
        self._log = get_logger("registry", self.config)
        self.clock = SimClock()
        self.client = None           # pybullet client id (set on sim thread)
        self.layout = None           # ArenaLayout (set during boot)
        self.bodies = None           # WorldBodies (set during boot)
        self.drones = []             # [SimDrone] (set during boot)
        self.rovers = []             # [SimRover] (set during boot)
        self.monitor = monitor.CommandMonitor(self.config, self.clock)
        self.referee = None          # part-2 referee (after boot, if enabled)
        self.landing_scorer = scoring.LandingScorer(self)  # part-1 referee
        self.compliance = None       # compliance flag logger (set during boot)
        self._auto_recorder = None   # $HULA_SIM_RECORD integration hook
        self._marker_applied = {}    # rover index -> last-applied marker texture (gimbal)
        self.scenario = None         # episode state machine (set during boot)
        self.renderer = p.ER_TINY_RENDERER  # upgraded if the EGL plugin loads
        self._egl_plugin = -1
        self._calls = queue.Queue()  # (fn, result_box, done_event)
        self._stop = threading.Event()
        self._boot_done = threading.Event()
        self._boot_error = None
        self._thread = threading.Thread(target=self._sim_loop,
                                        name="hula-sim", daemon=True)
        self._thread.start()
        if not self._boot_done.wait(_BOOT_TIMEOUT_S):
            raise RuntimeError("sim world failed to boot within "
                               f"{_BOOT_TIMEOUT_S}s")
        if self._boot_error is not None:
            raise self._boot_error
        # The referee renders + scans camera frames; it MUST NOT run in the
        # cameras-disabled live-3D mode (that is the whole freeze-proof point).
        if self.config.scoring.enabled and self.cameras_enabled:
            self.referee = scoring.Referee(self)
            self.referee.start()
        elif self.config.scoring.enabled:
            self._log.info("cameras disabled (live-3D): referee/scanning OFF")
        self._maybe_start_autorecord()

    def _maybe_start_autorecord(self) -> None:
        """Opt-in integration hook: if $HULA_SIM_RECORD names a path, auto-start
        the offscreen cockpit recorder when the world boots — so a MISSION
        process (which boots the sim in-process via connect() and cannot import
        the sim's recorder) still gets an MP4 of its run. Observer-only; OFF
        unless the env var is set; never under p.GUI / cameras-disabled."""
        import os
        path = os.environ.get("HULA_SIM_RECORD")
        if not path or not self.cameras_enabled or self.gui:
            return
        import atexit

        from .recorder import ArenaRecorder
        self._auto_recorder = ArenaRecorder(self, path)
        self._auto_recorder.start()
        atexit.register(self._auto_recorder.stop)  # finalise mp4 on plain exit
        self._log.info("HULA_SIM_RECORD set -> auto-recording this run to %s",
                       path)

    # ------------------------------------------------------------------ #
    # Cross-thread access
    # ------------------------------------------------------------------ #

    def run_on_sim_thread(self, fn, timeout: float = _CALL_TIMEOUT_S):
        """Execute fn() on the sim thread and return its result.

        This is the only legal way to touch PyBullet from outside the sim
        thread. Exceptions raised by fn propagate to the caller.
        """
        if threading.current_thread() is self._thread:
            return fn()  # already on the sim thread (re-entrant call)
        if self._stop.is_set() or not self._thread.is_alive():
            raise RuntimeError("sim thread is not running")
        box = {}
        done = threading.Event()
        self._calls.put((fn, box, done))
        if not done.wait(timeout):
            raise TimeoutError(f"sim-thread call timed out after {timeout}s")
        if "error" in box:
            raise box["error"]
        return box.get("result")

    def sim_time(self) -> float:
        """Sim time in seconds since boot (advances at real_time_factor)."""
        return self.clock.now()

    def is_alive(self) -> bool:
        """True while the sim thread is running and accepting work."""
        return self._thread.is_alive() and not self._stop.is_set()

    def drone_by_ip(self, ip: str):
        """The SimDrone configured with this IP, or None."""
        for d in self.drones:
            if d.spec.ip == ip:
                return d
        return None

    def drone_world_pose(self, index: int):
        """((x, y, z) world metres, yaw rad) TRUE pose — for tests/viz only."""
        d = self.drones[index]
        return self.run_on_sim_thread(lambda: (tuple(d.pos), float(d.yaw)))

    def render_camera(self, drone, width=None, height=None):
        """One (H, W, 3) uint8 RGB frame from a drone's tiltable camera,
        rendered on the sim thread with the active renderer (EGL or Tiny).
        width/height override the resolution (same camera FOV/pose) so the
        recorder can render crisp HIGH-RES insets while the referee keeps the
        default AI-mode 640x480. Returns None when cameras are disabled
        (live-3D mode) — no getCameraImage, so it cannot freeze a p.GUI."""
        if not self.cameras_enabled:
            return None

        def _render():
            self._apply_gimbal_gating(drone)   # show/hide markers for THIS drone
            return camera.render_rgb(self.client, self.config, drone,
                                     self.renderer, width=width, height=height)
        return self.run_on_sim_thread(_render, timeout=30)

    def _apply_gimbal_gating(self, drone) -> None:
        """Before rendering `drone`'s camera, set each rover's marker texture to
        the ArUco fiducial when the gimbal is readable BY THIS DRONE, else to a
        blank texture (the camera then sees the body, no decodable marker).
        SIM THREAD ONLY. No-op when the gimbal is disabled."""
        if not self.config.rovers.gimbal.enabled:
            return
        now = self.clock.now()
        dxy = (float(drone.pos[0]), float(drone.pos[1]))
        for r in self.rovers:
            if r._marker_link is None:
                continue
            tex = (r._marker_tex if r.marker_readable_by(now, dxy)
                   else self.bodies.blank_marker)
            if self._marker_applied.get(r.index) != tex:
                p.changeVisualShape(r.body_id, r._marker_link,
                                    textureUniqueId=tex,
                                    physicsClientId=self.client)
                self._marker_applied[r.index] = tex

    def render_arena(self, width: int = None, height: int = None):
        """One (H, W, 3) uint8 RGB frame of a fixed angled-overhead
        third-person view framing the WHOLE arena — offscreen, on the sim
        thread, through the guarded renderer (EGL headless; never p.GUI).
        Passive observer for the recorder; renders the live world, no
        re-sim. Returns None if the sim has shut down or cameras are disabled
        (live-3D mode — no getCameraImage, so it cannot freeze a p.GUI)."""
        if not self.cameras_enabled:
            return None
        import math

        import numpy as np
        rc = self.config.record
        w = int(width if width is not None else rc.width)
        h = int(height if height is not None else rc.height)

        def _render():
            L, W = self.config.arena.length_m, self.config.arena.width_m
            tx, ty, _ = frames.arena_to_world(self.config, L / 2, W / 2, 0.0)
            # Eye stands off behind the south edge, elevated, looking at the
            # arena centre at the configured elevation angle.
            ang = math.radians(rc.camera_angle_deg)
            standoff = rc.camera_height_m / max(math.tan(ang), 0.1)
            ex, ey, _ = frames.arena_to_world(
                self.config, L / 2 - standoff, W / 2, 0.0)
            view = p.computeViewMatrix(
                cameraEyePosition=[ex, ey, rc.camera_height_m],
                cameraTargetPosition=[tx, ty, 0.3],
                cameraUpVector=[0.0, 0.0, 1.0])
            proj = p.computeProjectionMatrixFOV(
                fov=rc.fov_deg, aspect=w / h, nearVal=0.1,
                farVal=rc.camera_height_m + math.hypot(L, W) + 10.0)
            img = p.getCameraImage(
                w, h, viewMatrix=view, projectionMatrix=proj,
                renderer=camera.resolve_renderer(self.client, self.renderer),
                physicsClientId=self.client)  # GUI -> software (never here)
            rgba = np.asarray(img[2], dtype=np.uint8).reshape(h, w, 4)
            return rgba[:, :, :3].copy()
        try:
            return self.run_on_sim_thread(_render, timeout=30)
        except (RuntimeError, TimeoutError):
            return None

    def uwb_truth(self):
        """[(uwb_tag_id, north, east), ...] TRUE arena positions, one atomic
        sim-thread read — the UWB drop-in samples this each refresh."""
        return self.run_on_sim_thread(
            lambda: [(d.spec.uwb_tag_id, *d.arena_position())
                     for d in self.drones])

    def rover_arena_positions(self):
        """[(north, east), ...] TRUE rover positions — for tests/viz only."""
        return self.run_on_sim_thread(
            lambda: [r.arena_position() for r in self.rovers])

    def body_count(self) -> int:
        """Total bodies in the PyBullet world (queried on the sim thread)."""
        return self.run_on_sim_thread(
            lambda: p.getNumBodies(physicsClientId=self.client))

    def snapshot_body_poses(self):
        """[(kind, position, orientation), ...] for every body — for tests/viz."""
        def _snap():
            out = []
            b = self.bodies
            for kind, ids in (("floor", (b.floor,)), ("wall", b.walls),
                              ("obstacle", b.obstacles), ("drone", b.drones),
                              ("rover", b.rovers), ("pad", b.pads)):
                for bid in ids:
                    pos, orn = p.getBasePositionAndOrientation(
                        bid, physicsClientId=self.client)
                    out.append((kind, pos, orn))
            return out
        return self.run_on_sim_thread(_snap)

    def save_topdown_png(self, path: str, width: int = 900,
                         height: int = 900) -> None:
        """Render a straight-down view of the arena to a PNG (headless-safe).

        Minimal Phase 1 screenshot; the live top-down view is Phase 7.
        Image up = arena north, image right = arena east.
        """
        import math

        import numpy as np
        from PIL import Image

        cfg = self.config

        def _render():
            L, W = cfg.arena.length_m, cfg.arena.width_m
            t = cfg.arena.wall_thickness_m
            cx, cy, _ = frames.arena_to_world(cfg, L / 2, W / 2, 0.0)
            radius = math.hypot(L / 2 + t, W / 2 + t) + 0.3
            fov_deg = 60.0
            aspect = width / height
            # Camera altitude so the whole room fits in the narrower axis.
            alt = radius / (math.tan(math.radians(fov_deg / 2))
                            * min(1.0, aspect))
            view = p.computeViewMatrix(
                cameraEyePosition=[cx, cy, alt],
                cameraTargetPosition=[cx, cy, 0.0],
                cameraUpVector=[0.0, 1.0, 0.0])
            proj = p.computeProjectionMatrixFOV(
                fov=fov_deg, aspect=aspect, nearVal=0.1, farVal=alt + 10.0)
            img = p.getCameraImage(
                width, height, viewMatrix=view, projectionMatrix=proj,
                renderer=camera.resolve_renderer(self.client, self.renderer),
                physicsClientId=self.client)  # GUI -> software (no hang)
            rgba = np.asarray(img[2], dtype=np.uint8).reshape(height, width, 4)
            return rgba[:, :, :3].copy()
        rgb = self.run_on_sim_thread(_render, timeout=60)
        Image.fromarray(rgb).save(path)
        self._log.info("top-down screenshot saved to %s", path)

    def shutdown(self) -> None:
        """Stop the referee + sim thread and disconnect PyBullet. Idempotent."""
        if self._auto_recorder is not None:
            self._auto_recorder.stop()
        if self.referee is not None:
            self.referee.stop()
        if self.compliance is not None:
            self.compliance.close()
        self._stop.set()
        self._thread.join(timeout=10)
        if self._thread.is_alive():
            self._log.warning("sim thread did not exit cleanly")

    # ------------------------------------------------------------------ #
    # Sim thread
    # ------------------------------------------------------------------ #

    def _boot(self) -> None:
        cfg = self.config
        if self.gui:
            # Interactive 3D window (orbit/pan/zoom). The GUI provides its
            # own hardware GL, so the EGL plugin is skipped — EGL belongs to
            # the headless DIRECT default, which stays byte-for-byte as-is.
            self.client = p.connect(p.GUI)
            if self.client < 0:
                raise RuntimeError("pybullet GUI connect failed — no usable "
                                   "display? run without --gui")
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0,
                                       physicsClientId=self.client)
            cx, cy, _ = frames.arena_to_world(cfg, cfg.arena.length_m / 2,
                                              cfg.arena.width_m / 2, 0.0)
            p.resetDebugVisualizerCamera(
                cameraDistance=0.9 * max(cfg.arena.length_m,
                                         cfg.arena.width_m),
                cameraYaw=-90.0, cameraPitch=-55.0,
                cameraTargetPosition=[cx, cy, 0.5],
                physicsClientId=self.client)
            # Default field stays Tiny for GUI; the AUTHORITATIVE guard is
            # camera.resolve_renderer (keyed on the live connection mode), so
            # no camera-render path can issue a hardware getCameraImage while
            # GUI is connected (it races the GUI render thread and hangs/
            # segfaults on WSLg/D3D12). GUI is a human-speed viewing mode.
            self.renderer = p.ER_TINY_RENDERER
        else:
            self.client = p.connect(p.DIRECT)
            if self.client < 0:
                raise RuntimeError("pybullet DIRECT connect failed")
            self._load_egl()
        p.setGravity(0.0, 0.0, cfg.physics.gravity_mps2,
                     physicsClientId=self.client)
        p.setTimeStep(cfg.physics.dt_s, physicsClientId=self.client)
        self.layout = arena.generate(cfg)
        self.bodies = world.build(self.client, cfg, self.layout)
        hz = cfg.bodies.drone_half_extents_m[2]
        self.drones = [
            drone_model.SimDrone(
                cfg=cfg, index=i, spec=spec, body_id=bid, client=self.client,
                clock=self.clock,
                start_pos=frames.arena_to_world(cfg, pose.north, pose.east, hz),
                start_yaw=frames.heading_to_world_yaw_rad(cfg, pose.heading_deg),
                monitor=self.monitor, landing_scorer=self.landing_scorer)
            for i, (spec, pose, bid) in enumerate(
                zip(cfg.drones.units, self.layout.drone_starts,
                    self.bodies.drones))
        ]
        rover_ids = rover_model.resolved_marker_ids(cfg)
        self.rovers = [
            rover_model.SimRover(
                cfg=cfg, index=i, marker_id=rover_ids[i],
                body_id=bid, client=self.client, clock=self.clock,
                start_pose=pose, obstacles=self.layout.obstacles)
            for i, (pose, bid) in enumerate(
                zip(self.layout.rover_starts, self.bodies.rovers))
        ]
        for r, (link, tex) in zip(self.rovers, self.bodies.rover_markers):
            r.bind_drones(self.drones)     # evasive rovers flee the nearest drone
            r.bind_marker(link, tex)       # gimbal show/hide of its marker
        self._marker_applied = {}          # rover index -> last-applied texture id
        self.compliance = compliance.ComplianceMonitor(self)
        self.scenario = scenario.Scenario(self)
        self.scenario.on_boot()
        self._log.info(
            "world booted: seed=%s rtf=%.2f bodies=%d "
            "(walls=4 obstacles=%d drones=%d rovers=%d) renderer=%s",
            cfg.meta.seed, cfg.meta.real_time_factor, self.bodies.total,
            len(self.bodies.obstacles), len(self.bodies.drones),
            len(self.bodies.rovers),
            "GUI/GL" if self.gui
            else ("EGL/GPU" if self._egl_plugin >= 0 else "TinyRenderer"))

    def _load_egl(self) -> None:
        """Load the EGL hardware-render plugin by RESOLVED FILE PATH (§3).

        The bare name string fails with 'cannot open shared object file'.
        On WSL2, GL_RENDERER='D3D12 (NVIDIA ...)' IS hardware acceleration.
        Falls back to TinyRenderer (slow CPU) with a warning if unavailable.
        """
        if not self.config.camera.use_egl:
            self._log.info("EGL disabled by config; using TinyRenderer")
            return
        try:
            egl = pkgutil.get_loader("eglRenderer")
            if egl is not None:
                self._egl_plugin = p.loadPlugin(
                    egl.get_filename(), "_eglRendererPlugin",
                    physicsClientId=self.client)
        except Exception as e:  # never let render setup kill the sim
            self._log.warning("EGL plugin load raised: %s", e)
            self._egl_plugin = -1
        if self._egl_plugin >= 0:
            self.renderer = p.ER_BULLET_HARDWARE_OPENGL
        else:
            self._log.warning(
                "EGL renderer unavailable — falling back to TinyRenderer "
                "(slow CPU rendering)")

    def _drain_calls(self) -> None:
        while True:
            try:
                fn, box, done = self._calls.get_nowait()
            except queue.Empty:
                return
            try:
                box["result"] = fn()
            except BaseException as e:  # propagate to the caller, keep stepping
                box["error"] = e
            done.set()

    def _fail_pending_calls(self) -> None:
        while True:
            try:
                _, box, done = self._calls.get_nowait()
            except queue.Empty:
                return
            box["error"] = RuntimeError("sim shut down")
            done.set()

    def _sim_loop(self) -> None:
        try:
            self._boot()
        except BaseException as e:
            self._boot_error = e
            self._boot_done.set()
            if self.client is not None and self.client >= 0:
                p.disconnect(physicsClientId=self.client)
            return
        self._boot_done.set()

        dt = self.config.physics.dt_s
        rtf = self.config.meta.real_time_factor
        max_catchup = self.config.physics.max_catchup_steps
        wall_per_step = dt / rtf
        anchor = time.perf_counter()  # wall instant where sim step 0 is due
        steps = 0
        lag_warned = False  # first lag is a warning, repeats go to debug

        while not self._stop.is_set():
            self._drain_calls()
            behind = int((time.perf_counter() - anchor) / wall_per_step) - steps
            if behind <= 0:
                # Ahead of schedule: sleep until the next step is due
                # (capped so we stay responsive to calls and stop).
                due = anchor + (steps + 1) * wall_per_step
                delay = due - time.perf_counter()
                if delay > 0:
                    self._stop.wait(min(delay, 0.05))
                continue
            if behind > max_catchup * 10:
                log_fn = self._log.debug if lag_warned else self._log.warning
                log_fn("sim thread fell %d steps behind; re-anchoring (heavy "
                       "rendering or real_time_factor %.2f too high for this "
                       "machine — sim-time semantics are unaffected)",
                       behind, rtf)
                lag_warned = True
                anchor = time.perf_counter() - steps * wall_per_step
                continue
            for _ in range(min(behind, max_catchup)):
                p.stepSimulation(physicsClientId=self.client)
                self.clock.advance(dt)
                for d in self.drones:
                    d.step(dt)
                if self.scenario.rovers_active():
                    for r in self.rovers:
                        r.step(dt)
                self.scenario.step(self.clock.now())
                if self.compliance is not None:
                    self.compliance.step(self.clock.now())
                steps += 1

        self._fail_pending_calls()
        if self._egl_plugin >= 0:
            p.unloadPlugin(self._egl_plugin, physicsClientId=self.client)
        if not self.gui:
            p.disconnect(physicsClientId=self.client)
        # GUI clients are deliberately left for process exit to reap:
        # tearing the GUI window down via p.disconnect from a worker thread
        # segfaults (observed on WSLg/D3D12), and --gui is only used by
        # short-lived CLI runs.


# --------------------------------------------------------------------------- #
# Lazily-initialised shared singleton (§5.4) — what pyhulax/_bridge and the
# UWB drop-in attach to, so they all describe the same world.
# --------------------------------------------------------------------------- #

_registry = None
_registry_lock = threading.Lock()


def get_registry(config: SimConfig = None, gui: bool = False,
                 cameras_enabled: bool = True) -> SimRegistry:
    """Return the shared world, creating it from config on first use.

    gui / cameras_enabled only apply at creation time (the connection mode is
    fixed at p.connect; cameras_enabled=False is the freeze-proof live-3D
    mode); they are ignored when the world already exists.
    """
    global _registry
    with _registry_lock:
        if _registry is None:
            _registry = SimRegistry(config, gui=gui,
                                    cameras_enabled=cameras_enabled)
        return _registry


def shutdown_registry() -> None:
    """Tear down the shared world (tests / interpreter exit)."""
    global _registry
    with _registry_lock:
        if _registry is not None:
            _registry.shutdown()
            _registry = None
