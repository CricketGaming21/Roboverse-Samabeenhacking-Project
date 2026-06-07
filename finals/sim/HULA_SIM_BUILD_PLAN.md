# RoboVerse 2026 — Challenge 2 Hula Swarm Simulator
## Build specification for Claude Code

> **You are building a simulator, not a drone-control mission.**
> This project produces a *simulated world* plus a *drop-in replacement for the `pyhulax` SDK and the UWB tag library*. A separate project (written elsewhere) will `import pyhulax` and `from UWBParserThread import UWBParserThread` and fly the mission. That mission code must run **unmodified** against this sim and against the real hardware. **API fidelity to the real SDK is the number-one requirement.** When in doubt, match the contract in §4 exactly.

---

## 1. How to use this document with Claude Code

Work **phase by phase** (§6). Do not skip ahead. For each phase:

1. Implement only that phase's scope.
2. Make its acceptance test (in `tests/`) pass, headless.
3. Run the phase's manual check command and confirm the printed result.
4. Commit with the convention in §8, then **stop and report** what you built and the test output before starting the next phase.

Suggested first prompt to Claude Code after dropping in this file:

> "Read `HULA_SIM_BUILD_PLAN.md` in full, then read the reference files listed in §3. Do **Phase 0** only: scaffold the repo, create `CLAUDE.md`, write the full interface stubs from §4 (signatures + types, raising `NotImplementedError` in bodies), and make `pytest tests/test_phase0_contract.py` pass (it only checks that the API surface imports and has the right signatures). Then stop and show me the tree and the test output."

Then, for each subsequent phase: *"Do Phase N only. Make its test pass headless, run its manual check, commit, then stop and report."*

Rules of engagement for the whole build:
- **Never** add mission logic (landing routine, search pattern, lock-on, swarm strategy, YOLO). If a phase tempts you toward it, stop — that belongs to the other project.
- **Never** change a signature in §4 to make implementation easier. The signature is the contract.
- Keep every phase independently runnable and committed. Small commits.
- Prefer clarity over cleverness; this is test infrastructure, not flight code.

---

## 2. Scope

### In scope (this project builds all of it)
- A PyBullet world: procedurally generated room (configurable size/seed), walls, box/pillar obstacles, ArUco-textured landing pads at **configured** coordinates, 3 drones, 5 rovers.
- A drop-in `pyhulax` package: API-compatible `DroneAPI`, `Dola` discovery, video stream, types/enums, exceptions (§4).
- A drop-in `UWBParserThread` that reports arena (north, east) position from sim ground truth.
- Two coordinate frames modelled honestly: UWB arena frame (metres, ground truth + noise) and each drone's takeoff-origin frame (cm, with injected drift) — see §5.
- Rovers that move on their own (configurable patrol), each wearing a **unique** ArUco marker on top.
- A **referee/scorer** (sim-side) that counts distinct ArUco IDs successfully scanned (§7).
- A live top-down 2D view (drones, paths, rovers, pads, coverage, banked IDs) and a command-rate / thrash monitor.
- A smoke test (connect → takeoff → fly a square → land) that proves the sim runs — *not* a mission.
- Config surface (single source of truth) + README.

### Out of scope (the other project builds these — do not write them here)
- Landing approach / precision-landing logic.
- Search / coverage strategy.
- Target lock-on control law.
- Swarm coordination / deconfliction strategy.
- The real YOLO model or any mission-side detector.

The sim must boot and run its smoke test with **zero** mission code present.

---

## 3. Before you start — read these (they define the real API you must mimic)

In the repo / provided materials:
- `pyhulax_complete_knowledge_base.txt` — the real SDK reference (control, telemetry, obstacles, camera, video, types). **Primary source of truth for §4.**
- `dola.py` — real `Dola` discovery listener (UDP 8668 → `{plane_id: ip}`).
- `huladola.py` — real usage example (connect to many drones, pull video streams).
- `UWBParserThread.py` — real UWB serial parser. Your sim version must match its constructor and `get_tag_position` signature.
- `UWBParserThread_Core_Documentation.pdf` — UWB API notes.

If any detail below conflicts with these files, the files win for **signatures**; this doc wins for **sim behaviour**.

### Environment
**The dev environment is already built and validated** — WSL2 + Ubuntu 22.04 on the RTX 2080 Ti desktop, Python 3.10, GPU visible (`nvidia-smi`), PyBullet EGL hardware rendering confirmed (~hundreds of FPS at 640×480). Build here, not in the VMware VM. Specifics:
- **venv:** `~/sim-venv` (activate via the `simvenv` alias). Run **all** commands from the project root (§5.5) so `import pyhulax` resolves to the sim — see the swap note in §5.5.
- **Dependencies** — already frozen in the venv: `pybullet==3.2.7`, `opencv-contrib-python==4.13.0.92` (provides `cv2.aruco`), `numpy==2.2.6`, `scipy==1.15.3`, `pillow==12.2.0`, on Python 3.10. **Still to add** to `requirements.txt` and install: `pyyaml` (config), `matplotlib` (top-down view), `pytest` (tests). (`pygame` optional if preferred over matplotlib.)
- **Validated GPU render path — use exactly this in the camera phase.** Load the EGL plugin by its **resolved file path** (the bare name string `p.loadPlugin("eglRendererPlugin")` fails with "cannot open shared object file"):
  ```python
  import pybullet as p, pkgutil
  p.connect(p.DIRECT)
  egl = pkgutil.get_loader("eglRenderer")
  plugin = p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")   # plugin id >= 0 == loaded
  # render frames with the hardware OpenGL renderer:
  p.getCameraImage(w, h, renderer=p.ER_BULLET_HARDWARE_OPENGL)
  ```
  On WSL2 the renderer string reports `GL_RENDERER = D3D12 (NVIDIA GeForce RTX 2080 Ti)` — **this is expected and IS hardware acceleration** (WSLg routes OpenGL → DirectX 12 → the NVIDIA card). Do **not** mistake the `D3D12`/`Microsoft` string for software (`llvmpipe`) and "fix" it.
  - **Fallback only if EGL ever refuses the GPU on another machine** (e.g. the 4070 laptop): `p.connect(p.GUI)` renders through WSLg's display, also GPU-accelerated. Not needed on the desktop — EGL works there. Keep the headless EGL path as the default; it's also what any future CI needs.
- **WSL hygiene:** keep the repo in the WSL native filesystem (it already is, at `~/codes`), never under `/mnt/c/...`. matplotlib and `cv2.imshow` windows display fine via WSLg, no extra setup.

---

## 4. The interface contract (THE SEAM — implement exactly)

The drop-in package is named **`pyhulax`** so mission code does `import pyhulax` unchanged. Module layout:

```
pyhulax/
  __init__.py        # exports DroneAPI, Dola
  core.py            # enums + models (Direction, VelocityLevel, ... Vector3, Obstacles, ...)
  video.py           # VideoStream, VideoFrame, VideoDisplay
  exceptions.py      # PyhulaxError, NotReady, LowBattery, TelemetryUnavailable
  _bridge.py         # INTERNAL: routes DroneAPI calls to the sim registry (not part of the public API)
```

> Mission code only ever touches the public names below. Everything in `pyhulax/_bridge.py` and the whole `simcore/` package (§ layout in 5) is internal plumbing.

### 4.1 Discovery — `Dola` (in `pyhulax/__init__.py` or `pyhulax/discovery.py`)
```python
class Dola:
    def __init__(self, listen_ip: str = "0.0.0.0") -> None: ...
    def start(self) -> None: ...
    def stop(self) -> None: ...
    def get_all_ips(self, listen_seconds: float = 5.0) -> dict[int, str]:
        """Return {plane_id: ip} for all drones the sim is configured with.
        In sim, returns the configured fake drones immediately (ignores listen_seconds
        beyond a short fake delay)."""
```

### 4.2 `DroneAPI` — connection + control (blocking by default)
```python
class DroneAPI:
    def __init__(self) -> None: ...
    def connect(self, ip: str) -> CommandResult:
        """Bind this instance to the sim drone whose configured IP == ip.
        Boots the shared sim world on first call (§5.4)."""

    def takeoff(self, height_cm: int = 100, led=None,
                blocking: bool = True, flags: "TakeoffFlags|int" = TakeoffFlags.NONE) -> CommandResult: ...
    def land(self, led=None, blocking: bool = True) -> CommandResult: ...
    def hover(self, duration_seconds: float, led=None, blocking: bool = True) -> CommandResult: ...

    def move(self, direction: "Direction", distance_cm: float, led=None,
             blocking: bool = True, speed: "VelocityLevel|int" = VelocityLevel.ZOOM) -> CommandResult:
        """Relative move along the CURRENT heading (body frame). FORWARD = nose direction."""

    def rotate(self, angle_degrees: float, led=None, blocking: bool = True) -> CommandResult:
        """Yaw by angle. Positive = CCW (left), negative = CW (right)."""

    def move_to(self, x: float, y: float, z: float, led=None,
                blocking: bool = True, speed: "VelocityLevel|int" = VelocityLevel.ZOOM) -> CommandResult:
        """Straight-line flight to a point in the TAKEOFF-ORIGIN frame, in cm
        (x = right, y = forward, z = up; relative to where this drone took off — §5.2)."""
```
Raises `NotReady` if not connected/flying, `LowBattery` if battery below threshold (configurable; default e.g. 10%). Blocking commands return only once the move completes (in **sim time**) or times out; `blocking=False` returns immediately after the goal is accepted.

*Stub these to keep the surface complete but raise `NotImplementedError` (not needed for Challenge 2):* `curve(...)`, `circle(...)`, `recognize_qr(...)`, `detect_qr(...)`, `track_qr(...)`, `recognize_target(...)`, `follow_line(...)`, `fire_laser(...)`, gripper/clamp. They exist in the real SDK; we don't simulate edu features.

### 4.3 `DroneAPI` — telemetry
```python
    def get_state(self) -> "DroneState": ...
    def get_position(self) -> "Vector3":        # cm, takeoff-origin frame, WITH drift (§5.3)
        ...
    def get_orientation(self) -> "Orientation": # degrees (yaw, pitch, roll)
        ...
    def get_altitude(self) -> float:            # cm, downward ToF
        ...
    def get_battery(self) -> int:               # 0-100
        ...
```
Each raises `TelemetryUnavailable` if no data yet (e.g. before connect).

### 4.4 `DroneAPI` — obstacles (barrier sensors)
```python
    def get_obstacles(self, drone_id: int = 0) -> "Obstacles":
        """Five booleans from simulated IR/ToF barrier sensors (§ Phase 4)."""
    def any_obstacle(self) -> bool: ...
    def get_drone_status(self, drone_id: int = 0) -> "int|None":
        """Raw status int; barrier bits: 0=forward 1=back 2=left 3=right 4=down."""
    def set_barrier_mode(self, enabled: bool) -> CommandResult:
        """Enable/disable the sim's automatic reflex avoidance."""
    def set_avoidance_direction(self, direction: "Direction", distance_cm: int = 0,
                                barrier_mask: "BarrierMask|int" = BarrierMask.ALL,
                                blocking: bool = True) -> CommandResult:
        """Conditional reflex: when a sensor in barrier_mask trips, move `direction` by distance_cm.
        Only fires when an obstacle is actually detected."""
```

### 4.5 `DroneAPI` — camera + video
```python
    def set_camera_angle(self, mode: "CameraPitchMode", angle: int = 0) -> CommandResult:
        """Tilt the main camera. angle in degrees 0-90 (DOWN_ABSOLUTE 90 = straight down,
        UP_ABSOLUTE 0 = straight ahead)."""
    def create_video_stream(self) -> "VideoStream": ...
    def set_video_stream(self, enabled: bool) -> CommandResult: ...
```

`pyhulax/video.py`:
```python
class VideoFrame:
    @property
    def width(self) -> int: ...
    @property
    def height(self) -> int: ...
    def to_rgb(self) -> "np.ndarray":   # (H, W, 3) uint8, RGB  <-- what the mission's detector consumes
    def to_bgr(self) -> "np.ndarray":   # (H, W, 3) uint8, BGR

class VideoStream:
    def start(self) -> None: ...
    def stop(self) -> None: ...
    @property
    def latest_frame(self) -> "VideoFrame|None":   # most recent rendered frame for this drone
    @property
    def fps(self) -> float: ...

class VideoDisplay:        # optional helper; may show frames with cv2.imshow
    def __init__(self, stream: "VideoStream", window_name: str = "hula", show_fps: bool = True): ...
    def start(self) -> None: ...
    def stop(self) -> None: ...
```
The frame returned must be a real rendered RGB image of what that drone's (tiltable) camera sees, so `cv2.aruco.detectMarkers` and any mission-side detector work on it.

### 4.6 Types & enums — `pyhulax/core.py`
```python
from enum import IntEnum
from dataclasses import dataclass

class Direction(IntEnum):     FORWARD=...; BACK=...; LEFT=...; RIGHT=...; UP=...; DOWN=...
class VelocityLevel(IntEnum): SLOW=...; MEDIUM=...; ZOOM=100; TURBO=...
class CameraPitchMode(IntEnum): UP_ABSOLUTE=0; DOWN_ABSOLUTE=...; CALIBRATE=...
class VisionMode(IntEnum):    OPTICAL_FLOW=0; FRONT_CAMERA=1
class TakeoffFlags(IntEnum):  NONE=0; RESET_YAW=1; WITH_LOAD=2
class BarrierMask(IntEnum):
    FRONT=...; BACK=...; LEFT=...; RIGHT=...; UP=...; DOWN=...
    HORIZONTAL=...  # FRONT|BACK|LEFT|RIGHT
    ALL=...

@dataclass(frozen=True)
class Vector3:    x: float; y: float; z: float            # cm in DroneAPI contexts
@dataclass(frozen=True)
class Orientation: yaw: float; pitch: float; roll: float  # degrees

@dataclass(frozen=True)
class Obstacles:
    forward: bool=False; back: bool=False; left: bool=False; right: bool=False; down: bool=False
    @classmethod
    def from_bitmask(cls, barrier: int) -> "Obstacles": ...   # bits 0..4 as in §4.4
    @property
    def any(self) -> bool: ...

@dataclass(frozen=True)
class CommandResult:
    success: bool = True
    message: str = ""
    def __bool__(self) -> bool: return self.success

@dataclass(frozen=True)
class DroneState:
    connected: bool
    position: Vector3
    orientation: Orientation
    altitude: float
    battery: int
    obstacles: Obstacles
    flying: bool
```
(Provide `AIResult` as a small dataclass too — `success`, `position: Vector3`, `angle: float`, `target_id: int` — only so the stubbed QR methods can type-return; never produced by the sim.)

### 4.7 Exceptions — `pyhulax/exceptions.py`
```python
class PyhulaxError(Exception): ...
class NotReady(PyhulaxError): ...
class LowBattery(PyhulaxError): ...
class TelemetryUnavailable(PyhulaxError): ...
```

### 4.8 UWB drop-in — `UWBParserThread.py` (top-level import, matches the real file)
```python
import threading
class UWBParserThread(threading.Thread):
    def __init__(self, x_origin: float = 0.0, y_origin: float = 0.0,
                 serial_port=None, baud_rate: int = 921600) -> None: ...
    def detect_com_port(self): ...                # sim: return a fake port string, no hardware
    def run(self) -> None: ...                    # sim: pull ground truth from the registry, apply noise
    def get_tag_position(self, tag_id: int):      # -> (x, y, update_time) in METRES, or (None, None, None)
        ...
    def stop(self) -> None: ...
```
**Behaviour — match the real device and the provided `UWBParserThread.py` exactly:**
- **Multiple tags, keyed by integer id** (`block_id` in the real frame), one tag per drone. The `tag_id → drone` mapping comes from config (`drones[].uwb_tag_id`). An unknown/unmapped `tag_id` returns `(None, None, None)`.
- **Return is `(pos_x, pos_y, update_time)`** — `pos_x`/`pos_y` are arena coordinates in **metres** (real device sends mm /1000). **X-Y only, no Z** — UWB gives no altitude; altitude comes from `get_altitude` (ToF). Value = arena ground truth **+ Gaussian noise** (`uwb.noise_std_m`).
- **`update_time` is a wall-clock `time.time()`** captured when the sample was produced (so a mission freshness check `time.time() - update_time` behaves like hardware). Samples refresh at `uwb.rate_hz` (sim time); between refreshes the **same** value + timestamp are held (the tag dict is replaced each frame, exactly as the real `parse_data`).
- **No drift.** Unlike `get_position` (which drifts), UWB is truth + noise only — that asymmetry is the whole point of having UWB.
- **Origin-offset fidelity quirk:** the provided real code *accepts* `x_origin`/`y_origin` but **never applies them** (`origin_x`/`origin_y` stay `0.0` and aren't added to the returned coords). Reproduce that by default (`uwb.apply_origin_offset: false`) so sim output == hardware output. Only apply the offset if that flag is set true.
- **Optional dropout** (`uwb.dropout_prob`, default 0): when set, a *mapped* tag may briefly return `(None, None, None)` to mimic NLOS/occlusion, so mission code that must tolerate gaps gets exercised.

---

## 5. Coordinate frames & the runtime model (read before Phase 2)

### 5.1 UWB arena frame
A fixed 2D frame for the whole arena. Origin at a configured arena point; **x = north (m), y = east (m)**. This is ground truth (plus small noise) and is what `UWBParserThread.get_tag_position` reports.

### 5.2 Takeoff-origin frame (per drone)
Created when a drone takes off. Origin = that drone's takeoff position and takeoff heading. **x = right, y = forward, z = up, in cm.** This frame is **fixed at takeoff** (it does not rotate when the drone yaws). `move_to(x,y,z)` targets and `get_position()` reports are in this frame. `move(Direction, dist)` is **body-relative to current heading** (FORWARD follows the nose after any `rotate`). Keep these two behaviours distinct — this is the most common place to introduce a bug.

> Assumption to verify against the real drone later, but implement this way for now: `move` is body/current-heading relative; `move_to`/`get_position` are in the fixed takeoff frame.

### 5.3 Drift
`get_position()` (optical-flow/IMU estimate) must **drift** slowly relative to true pose (configurable random-walk). `get_tag_position()` (UWB) must **not** drift (true + small noise only). This is deliberate: it lets the mission's UWB-correction loop be tested. Maintain, per drone: true pose (PyBullet), a drifting estimate (for `get_position`), and the arena-frame truth (for UWB).

### 5.4 Process & threading model
- **Single process.** The mission imports `pyhulax` (this package) and runs in the same process as the sim.
- A **background sim thread** owns the PyBullet client and calls `stepSimulation()` at a fixed rate scaled by a configurable real-time factor (`>1` runs faster than real time for quick tests; `1.0` = real time). Rovers move, physics advances, and telemetry/drift update on this thread.
- **All PyBullet calls happen on the sim thread.** PyBullet is not thread-safe. The `DroneAPI` / `UWBParserThread` implementations (called from the mission thread, possibly several) push commands onto a thread-safe queue; the sim thread executes them and sets a completion `Event`. Camera renders (`getCameraImage`) also go through this queue.
- **Blocking commands:** the implementation enqueues a goal, then waits on the completion event (with a sim-time timeout). The sim thread drives the drone toward the goal each step (simple kinematic motion at the speed implied by `VelocityLevel`), and signals completion on arrival. `blocking=False` returns a `CommandResult` immediately and the motion continues in the background.
- **The shared world is a lazily-initialised singleton** (`simcore/registry.py`) created from config on the first `connect()` or first UWB read. Both `pyhulax` and `UWBParserThread` attach to this one registry, so they describe the same world.

### 5.5 Repository layout (full)
The simulator lives **inside the existing repo** at `~/codes/finals/sim/` (repo `Roboverse-Samabeenhacking-Project`, branch `finals`, auto-push hook on every commit). `finals/sim/` is the project root below; the internals package is named **`simcore`** (not `sim`, to avoid a confusing `finals/sim/sim/`).

```
finals/sim/                # PROJECT ROOT — run all commands from here
  pyhulax/                 # DROP-IN SDK (public API only — §4)
    __init__.py            # exports DroneAPI, Dola
    core.py
    video.py
    exceptions.py
    _bridge.py             # internal: DroneAPI -> simcore registry
  UWBParserThread.py       # DROP-IN UWB provider (top-level, matches real file)
  simcore/                 # SIMULATOR INTERNALS (mission never imports these)
    __init__.py
    registry.py            # the world singleton + command queue + sim thread
    config.py              # Config dataclass (all tunables, defaults) + YAML/env override
    arena.py               # procedural room/obstacle/pad generation (seeded)
    world.py               # builds the PyBullet scene from arena + config
    drone_model.py         # per-drone state, kinematic command execution, drift
    rover_model.py         # rover agents + patrol motion + their markers
    camera.py              # per-drone tiltable camera render (EGL), intrinsics
    sensors.py             # 5-direction barrier ray-casts, ToF altitude
    frames.py              # arena(m) <-> takeoff(cm) <-> pybullet-world transforms
    aruco_assets.py        # generate ArUco marker PNGs (pads + rovers) with quiet zone
    scoring.py             # referee: unique ArUco IDs scanned (§7)
    monitor.py             # command-rate / thrash monitor
    viz.py                 # top-down 2D view (matplotlib/pygame), optional
    clock.py               # sim time + real-time factor
    log.py                 # sim logger (console + file)
  assets/                  # generated marker PNGs, rover billboard texture, floor texture
  scripts/
    __init__.py
    run_sim.py             # boot world from config; optional live view; idle/keep-alive
    smoke_test.py          # connect -> takeoff -> square -> land (PROVES SIM RUNS)
    gen_assets.py          # (re)generate ArUco PNGs into assets/
  tests/
    test_phase0_contract.py ... test_phase7_smoke.py
  sim_config.yaml          # user-editable overrides (arena size, pads, rovers, noise, ...)
  requirements.txt
  README.md
  CLAUDE.md                # created in Phase 0 (sim-building session context)
  HULA_SIM_BUILD_PLAN.md   # this document
```

**The sim ↔ real swap is purely the import path — no code change.** Because `pyhulax/` and `UWBParserThread.py` sit in the project root, running any command from `~/codes/finals/sim` with `python -m ...` (which puts the project root on `sys.path`) makes `import pyhulax` / `from UWBParserThread import UWBParserThread` resolve to the **sim** versions. On the real C2 you instead `pip install` the real `pyhulax` and use the organisers' real `UWBParserThread.py`, with `finals/sim` **not** on the path — the mission code is byte-identical either way. Do **not** `pip install -e` the sim's `pyhulax` (it would shadow the real one); keep it path-based.

---

## 6. Phase plan

Each phase: **Deliverable → Files touched → Acceptance test (headless) → Manual check → Commit.** Stop and report after each.

### Phase 0 — Scaffolding & contract stubs
- **Deliverable:** repo tree (§5.5), `requirements.txt`, `CLAUDE.md` (§ content below), and the **full §4 API surface as stubs** — every class/method/enum/dataclass present with correct signatures; method bodies raise `NotImplementedError` (except trivial dataclasses/enums which are real).
- **Acceptance test:** `tests/test_phase0_contract.py` imports `pyhulax`, `pyhulax.core`, `pyhulax.video`, `UWBParserThread`, and asserts the presence + signatures (use `inspect.signature`) of every public symbol in §4. No behaviour tested.
- **Manual check:** `python -c "import pyhulax, UWBParserThread; print('surface ok')"`.
- **Commit:** `chore: scaffold sim repo and pyhulax/UWB API stubs`.

`CLAUDE.md` content to create (condensed operating rules):
```
# Hula Swarm Simulator — Claude Code context (sim-building session)
PURPOSE: build the SIM only (simulated world + drop-in pyhulax + UWB). NOT mission code.
LIVES AT: ~/codes/finals/sim/  (repo branch: finals; auto-push hook on every commit).
RUN EVERYTHING from ~/codes/finals/sim with `python -m ...` so `import pyhulax`
  resolves to the sim package in this folder (that path-based import IS the sim/real swap).
VENV: ~/sim-venv  (alias: simvenv).
GOLDEN RULE: pyhulax/UWB public API must match the real SDK (see HULA_SIM_BUILD_PLAN.md §4) exactly.
DO: work phase by phase; make each phase's test pass headless; commit; STOP and report.
DON'T: write landing/search/lock-on/swarm/YOLO logic. Don't change §4 signatures.
RENDER (camera phase): PyBullet DIRECT + EGL plugin loaded BY RESOLVED FILE PATH
  (pkgutil.get_loader('eglRenderer').get_filename(), "_eglRendererPlugin"); render with
  renderer=p.ER_BULLET_HARDWARE_OPENGL. On WSL2 GL_RENDERER='D3D12 (NVIDIA ...)' is GPU — normal.
ALL PyBullet calls on the sim thread (simcore/registry.py). It is NOT thread-safe.
CONFIG: simcore/config.py defaults, overridable by sim_config.yaml / $HULA_SIM_CONFIG.
TEST: python -m pytest tests/      SMOKE: python -m scripts.smoke_test   (run from ~/codes/finals/sim)
COMMIT FORMAT: feat/fix/docs/chore: what + why  (auto-push hook handles the push)
```

### Phase 1 — PyBullet world + procedural arena
- **Deliverable:** `config.py`, `arena.py`, `world.py`, `registry.py` (world boot + sim thread + step loop + EGL), `clock.py`, `log.py`. Procedurally generate a seeded room: floor, four walls sized to config, N box/pillar obstacles (count/size/clearance from config), 3 drone bodies at configured start poses, 5 rover bodies at configured/ random positions. Drones/rovers are simple collision shapes for now (no camera/sensors yet). Real-time factor honoured.
- **Acceptance test:** boot world from a fixed seed; assert body counts (walls, obstacles==config, drones==3, rovers==5); assert two runs with the same seed are identical and different seeds differ; assert the sim thread advances time.
- **Manual check:** `python -m scripts.run_sim --seconds 2 --topdown out.png` saves a top-down screenshot of the generated arena.
- **Commit:** `feat: procedural arena + pybullet world with stepping sim thread`.

### Phase 2 — Drone model + blocking command execution
- **Deliverable:** `drone_model.py`, `_bridge.py`, real implementations of `connect`, `takeoff`, `land`, `hover`, `move`, `rotate`, `move_to`, and the command-queue/round-trip in `registry.py`. Kinematic motion at speeds mapped from `VelocityLevel` (define the m/s mapping in config). Blocking semantics per §5.4. Battery model (simple linear drain) feeding `get_battery`. Enforce `NotReady`/`LowBattery`.
- **Acceptance test:** connect one drone; `takeoff(100)`; assert true height ≈ 1.0 m. `move(FORWARD, 100)`; assert displaced ~1 m along heading. `rotate(90)` then `move(FORWARD, 100)`; assert moved along the new heading (proves body-relative move). `move_to(0,0,150)` returns toward takeoff-frame origin column at 1.5 m. `land()`; assert grounded. Test `blocking=False` returns promptly while motion continues.
- **Manual check:** `python -m scripts.smoke_test` (square pattern) completes without error.
- **Commit:** `feat: kinematic drone model with blocking pyhulax command execution`.

### Phase 3 — Frames, UWB drop-in, telemetry
- **Deliverable:** `frames.py` (arena(m) ↔ takeoff(cm) ↔ world transforms, configurable arena origin/orientation), drift model, `UWBParserThread.py` per §4.8 (reads registry ground truth; `tag_id→drone` map from config; Gaussian noise; sample refresh at `uwb.rate_hz` with wall-clock `update_time` held between refreshes; optional dropout; origin-offset quirk matched), and real `get_position` (drifting, cm), `get_orientation`, `get_altitude` (ToF cm), `get_state`.
- **Acceptance test:** start `UWBParserThread`; `get_tag_position(tag)` returns arena (north,east) in metres matching ground truth within noise bounds; move the drone and assert UWB tracks it; assert `get_position()` (estimate) diverges from UWB-derived truth over time (drift present) while UWB stays within noise; assert `update_time` is a wall-clock timestamp that advances at ~`uwb.rate_hz` and the (value, timestamp) pair is held constant between refreshes; assert `x_origin`/`y_origin` do **not** shift the returned coords while `apply_origin_offset` is false (matches the provided code); assert an unmapped `tag_id` returns `(None,None,None)`.
- **Manual check:** print side-by-side UWB vs `get_position` for 10 s of a moving drone; drift visibly accumulates, UWB does not.
- **Commit:** `feat: arena/takeoff frames, drift model, UWB drop-in, telemetry`.

### Phase 4 — Barrier sensors + reactive reflexes
- **Deliverable:** `sensors.py` — `rayTestBatch` in the five directions (forward/back/left/right/down) from each drone against obstacles/walls, with per-direction trigger range from config (short, IR/ToF-like). Implement `get_obstacles`, `any_obstacle`, `get_drone_status` (bitmask), `set_barrier_mode` (sim auto-stops/reflex-avoids when enabled), `set_avoidance_direction` (conditional reflex). Respect a minimum-altitude gate (~0.35 m) like the real sensors. **No depth image, no point cloud, no planner.**
- **Acceptance test:** spawn an obstacle directly ahead; fly toward it; assert `get_obstacles().forward` becomes `True` at the configured range and the bitmask bit 0 sets. With `set_avoidance_direction(BACK, 30, BarrierMask.FRONT)` active, assert the drone steps back when the front trips. With `set_barrier_mode(True)`, assert a forward move into a wall stops short instead of penetrating. Assert below the altitude gate, sensors report clear.
- **Manual check:** drive a drone down a corridor of boxes with `set_barrier_mode(True)`; it doesn't pass through anything.
- **Commit:** `feat: 5-direction barrier sensors and reflex avoidance (no depth/planner)`.

### Phase 5 — Camera rendering + ArUco pads
- **Deliverable:** `aruco_assets.py` + `gen_assets.py` (generate ArUco PNGs with a white quiet-zone border; dictionary from config, default `DICT_6X6_250` to match the organiser's sample; use the modern `cv2.aruco.ArucoDetector(dict, DetectorParameters())` API on grayscale, as the organiser sample does), `camera.py` (per-drone camera via `getCameraImage` through the sim thread; intrinsics from config: resolution default 640×480, H-FOV default ~70°; pitch controlled by `set_camera_angle`, 0°=forward … 90°=down), and `VideoStream`/`VideoFrame`/`set_video_stream`/`create_video_stream`. Texture the landing pads with ArUco markers at the **configured pad coordinates + IDs**. Even, diffuse lighting so markers read cleanly.
- **Acceptance test:** position a drone above a pad, camera pitched down; pull `latest_frame.to_rgb()`; run `cv2.aruco.detectMarkers`; assert the pad's configured ID is detected with 4 corners. Assert `to_rgb` is `(H,W,3)` uint8 RGB and `to_bgr` differs in channel order. Assert changing `set_camera_angle` changes the view (forward vs down see different things).
- **Manual check:** `gen_assets.py` writes PNGs; a script saves one rendered down-view frame with the detected marker drawn on it.
- **Commit:** `feat: tiltable camera rendering + ArUco landing pads (real cv2.aruco detection)`.

### Phase 6 — Rovers + unique ArUco markers
- **Deliverable:** `rover_model.py` — 5 rovers moving on configurable patrol (waypoint loiter / random walk within arena, speed from config), each carrying a **unique** ArUco marker on its top face (one ID per rover, from config) plus a RoboMaster billboard texture for visual realism. Markers must be detectable from a drone camera looking down/forward.
- **Acceptance test:** assert rover positions change over time and stay within arena bounds; place a drone so a rover is in view; assert that rover's unique marker ID is detected by `cv2.aruco`; assert all 5 IDs are distinct and configured.
- **Manual check:** save a frame with a rover marker detected; print rover tracks over 10 s.
- **Commit:** `feat: moving rovers with unique ArUco markers and RoboMaster texture`.

### Phase 7 — Scoring, top-down view, thrash monitor, full smoke run
- **Deliverable:** `scoring.py` (referee per §7), `monitor.py` (command-rate / thrash), `viz.py` (live top-down: drones+paths, rovers, pads, covered area, banked marker IDs, current score), wired into `registry.py` and `run_sim.py` (toggleable, headless-safe). Extend `smoke_test.py` to fly all 3 drones over some markers and print a final scoreboard.
- **Acceptance test:** drive a drone to hold a rover marker in view past the validity gate; assert the scorer banks that ID once and not twice; assert a second drive-by of the same ID does not increase the score; assert re-issuing motion commands faster than the threshold raises a thrash warning/count. Top-down renders to PNG headless.
- **Manual check:** `python -m scripts.run_sim --live` shows the top-down updating; the extended smoke run ends with a non-zero unique-ID score and a thrash report.
- **Commit:** `feat: referee scoring, top-down view, thrash monitor, full smoke run`.

### Phase 8 — Polish & docs
- **Deliverable:** finalise `README.md` (install, how to run, how the mission project imports it, config reference), ensure all config is surfaced and documented, ensure full headless run works on a fresh venv, tidy logging.
- **Acceptance test:** `pytest tests/` all green on a clean venv install from `requirements.txt`.
- **Commit:** `docs: README, config reference, final polish`.

---

## 7. Scoring (referee) — definition

The sim is the **referee**. It scores **automatically from what each drone's camera actually sees**, so mission code stays pure pyhulax (it never calls a scoring API).

- Every tick, run `cv2.aruco.detectMarkers` on each drone's current rendered frame.
- A marker **ID counts as scanned** (scored once, deduplicated globally across all drones) when it passes the **validity gate**:
  - apparent marker side length ≥ `min_marker_px` (close/clear enough),
  - fully inside the frame with a margin ≥ `frame_margin_px`,
  - held for ≥ `hold_frames` consecutive frames by the same drone (a stable read, not one lucky frame).
- **Score = count of distinct marker IDs scanned.** Each rover's ID counts once. The same gate scores Phase 1 pad-marker reads.
- Track per-drone attribution and the time each ID was banked. Expose a final scoreboard (IDs banked, by which drone, at what sim time) and surface banked IDs live in the top-down view.
- All gate parameters live in config (the real formula isn't published — keep them tunable).

**Optional explicit-capture mode** (config flag, default off): instead of sustained-visibility, score only when a deliberate capture is registered. Provide an *internal* referee hook `registry.referee.register_capture(drone_id)` for this; do **not** add it to the public pyhulax surface. Default behaviour is automatic sustained-visibility scoring.

> Note: this referee uses its own `cv2.aruco` purely to judge. The mission project will run its **own** detection on the same frames for its lock-on — that's separate and not this project's concern. (This supersedes the earlier idea of a synthetic segmentation-mask detection feed; we don't need it now that markers are real.)

---

## 8. Conventions & gotchas

**Commits:** `feat/fix/docs/chore: what changed and why`. Commit after each phase passes. A post-commit hook auto-pushes — don't push manually.

**Determinism:** seed every RNG (arena, rover paths, sensor/UWB noise, drift) from a single config seed so tests are reproducible. Same seed ⇒ identical run.

**Gotchas to get right (these are the usual failure points):**
- **One PyBullet thread.** Every `pybullet.*` call (step, ray tests, camera) on the sim thread only; facade enqueues and waits. Mixing threads silently corrupts state or segfaults.
- **EGL for headless camera.** Without the EGL plugin, `getCameraImage` falls back to slow CPU rendering. Load it (§3).
- **ArUco legibility.** Generate markers with a white quiet-zone border; texture pads/rovers at adequate pixel resolution; keep lighting even (avoid specular/over-dark) or detection fails. Verify detection in the phase test, not by eye.
- **Frame conventions.** `move` = body/current-heading; `move_to`/`get_position` = fixed takeoff frame (cm). UWB = arena frame (m), no Z. Don't conflate. Centralise all conversions in `frames.py`.
- **Drift vs noise.** `get_position` drifts; UWB only has noise. If your UWB also drifts, the mission's correction logic can't be tested — keep them separate.
- **Sim time, not wall time.** Blocking-command completion and timeouts use sim time scaled by the real-time factor, so faster-than-real runs still behave.
- **Headless safety.** `run_sim`/viz must work with the live view disabled (for CI / batch). Never require a display for the core sim or camera.
- **No mission leakage.** Keep `simcore/` and `_bridge.py` out of the public API. The only things the mission imports are `pyhulax` and `UWBParserThread`.

---

## 9. Definition of done (whole project)

- A program that does **only** `import pyhulax` + `from UWBParserThread import UWBParserThread`, connects to 3 drones, takes off, flies, reads UWB + telemetry + barrier flags, pulls camera frames with detectable ArUco markers, and is scored on unique markers seen — all without any mission/strategy code present.
- `pytest tests/` green on a clean install; `smoke_test.py` runs headless and prints a scoreboard.
- The same `import pyhulax` mission code could be pointed at the real SDK by swapping which `pyhulax` is on the path — no mission changes required.
