# Hula Swarm Simulator

A drop-in **`pyhulax`** SDK + **`UWBParserThread`** + PyBullet world for
developing the RoboVerse 2026 Challenge 2 mission code without hardware:
3 simulated HULA drones, a procedurally generated room, ArUco landing pads at
configured coordinates, 5 patrolling ArUco-tagged rovers, barrier sensors, a
referee that scores what the drone cameras actually see, a thrash monitor,
and a live top-down view.

**Status: complete — Phases 0–8 (`HULA_SIM_BUILD_PLAN.md`) + the part-2
scenario, Phases 9–15 (`SIM_UPDATE_PART2.md`): authored map, two-phase
DEPLOY→AMBUSH episode, rover convoy, two-part scoring, FOV/proximity views,
and `python -m scripts.scenario_demo` to watch the whole thing.**

## Install

```bash
cd ~/codes/finals/sim          # PROJECT ROOT — everything runs from here
source ~/sim-venv/bin/activate # alias: simvenv  (Python 3.10)
pip install -r requirements.txt
```

`requirements.txt` is fully pinned; a fresh venv + `pip install -r
requirements.txt` is all the setup there is.

## Running — always from `~/codes/finals/sim`, venv active, via `python -m`

```bash
python -m pytest                                  # full headless test suite
python -m scripts.smoke_test                      # 3-drone end-to-end run + scoreboard
python -m scripts.smoke_test --rtf 5              # same, 5x faster than real time
python -m scripts.run_sim --seconds 10 --topdown out.png   # idle world + 2D view PNG
python -m scripts.run_sim --live --seconds 30     # live top-down window (WSLg)
python -m scripts.gen_assets                      # (re)generate marker PNGs
```

Running from this folder matters: `python -m ...` puts the project root on
`sys.path`, which is what makes `import pyhulax` resolve to the sim.

## How the mission project uses this (the sim ↔ real swap)

Mission code only ever writes:

```python
import pyhulax                  # DroneAPI, Dola
from pyhulax.core import Direction, VelocityLevel, CameraPitchMode, BarrierMask
from pyhulax.video import VideoStream
from UWBParserThread import UWBParserThread
```

Run the mission from (or with `PYTHONPATH` including) `~/codes/finals/sim`
and those imports resolve to the **sim**; the shared world boots on the first
`DroneAPI().connect(ip)` or first UWB read. On the real C2 laptop, install the
real `pyhulax`, use the organisers' real `UWBParserThread.py`, and keep
`finals/sim` **off** the path. **The swap is the import path — never a code
change.** Do not `pip install -e` the sim's `pyhulax` (it would shadow the
real one); keep it path-based. Mission code must never import `simcore` or
`pyhulax._bridge` — those are sim internals.

Configured drone IPs (sim_config.yaml): `10.0.0.11/.12/.13` with UWB tags
0/1/2.

## Coordinate frames (the #1 source of bugs — see `simcore/frames.py`)

| Frame | Units | Meaning |
|---|---|---|
| UWB / arena | metres | `get_tag_position` → (x=NORTH, y=EAST), no Z, truth + noise, **no drift** |
| Takeoff-origin | cm | `move_to` / `get_position` → x=right, y=forward, z=up, **frozen at takeoff** (does not rotate with yaw); `get_position` **drifts** — correct it with UWB |
| Body | cm | `move(Direction, d)` is relative to the **current** heading (FORWARD = nose, after any `rotate`) |

Altitude comes from `get_altitude()` (downward ToF, cm) — never from UWB.
`rotate(+deg)` = CCW.

## Config reference (`sim_config.yaml` overrides `simcore/config.py` defaults; `$HULA_SIM_CONFIG` points elsewhere)

```
meta.seed                       one seed drives arena gen, rover paths, UWB noise, drift
meta.real_time_factor           1.0 = real time; >1 runs the sim faster (sim-time semantics unchanged)
arena.length_m/width_m/height_m room size: north extent / east extent / ceiling
arena.wall_thickness_m          wall slab thickness
arena.origin / arena.yaw_deg    where + how the arena sits in the pybullet world
arena.obstacles.count           number of random box/pillar obstacles
arena.obstacles.footprint_*     obstacle footprint side min/max (m)
arena.obstacles.height_*        obstacle height min/max (m)
arena.obstacles.min_clearance_m gap kept between obstacles (and to walls)
arena.obstacles.keepclear_radius_m  no obstacles within this radius of pads/drone starts
velocity_levels.SLOW/MEDIUM/ZOOM/TURBO  m/s per VelocityLevel NAME (the enum int is a P-gain, never a speed)
velocity_levels.yaw_rate_dps    rotate() angular speed
drones.units[]                  per drone: ip, uwb_tag_id, start [north,east], heading_deg (0=north, CCW+)
drones.takeoff_height_cm        default takeoff height
drones.battery.start_pct        battery at boot
drones.battery.drain_pct_per_min  linear drain while flying (sim time)
drones.battery.low_threshold_pct  below this, motion commands raise LowBattery (land always allowed)
uwb.rate_hz                     sample refresh rate (sim time); value+timestamp held between refreshes
uwb.noise_std_m                 Gaussian noise on x,y (truth + noise, no drift)
uwb.dropout_prob                per-frame chance a mapped tag returns (None,None,None)
uwb.apply_origin_offset         false = real-hardware quirk (x/y_origin accepted but ignored)
uwb.x_origin / uwb.y_origin     only applied if apply_origin_offset is true
position_drift.enabled          drift on get_position() on/off
position_drift.random_walk_std_mps  drift random-walk std (error std grows as std*sqrt(t))
barrier_sensors.range_m.*       per-direction trigger ray length (forward/back/left/right/down)
barrier_sensors.min_altitude_m  below this ToF altitude all barriers report clear
camera.width/height             frame resolution
camera.h_fov_deg                horizontal FOV (vertical derived)
camera.near_m/far_m             clip planes
camera.default_pitch_deg        boot camera pitch (0=forward, 90=down); CALIBRATE returns here
camera.fps                      video stream + referee frame rate
camera.use_egl                  load the EGL hardware-GL plugin (TinyRenderer fallback)
camera.mount_offset_m           lens sits this far ahead of the drone centre
aruco.dictionary                cv2.aruco dictionary name (default DICT_6X6_250)
aruco.pad_marker_size_m         printed marker side on landing pads
aruco.rover_marker_size_m       marker side on each rover's top face
pads[]                          per pad: ArUco id + arena position {id, north, east}
rovers.count                    number of rovers
rovers.marker_ids               unique ArUco id per rover (disjoint from pad ids)
rovers.billboard_texture        rover body texture (visual only; generated if missing)
rovers.patrol.mode              waypoint_random (only supported mode)
rovers.patrol.speed_mps         rover speed (0 parks them — handy in tests)
rovers.patrol.waypoint_pause_s  pause at each waypoint
rovers.patrol.bounds_north/east rovers stay inside these arena bounds
scoring.enabled                 run the referee thread with the world
scoring.mode                    auto = sustained-visibility; explicit = internal capture hook
scoring.min_marker_px           gate: apparent marker side must reach this
scoring.frame_margin_px         gate: marker fully inside the frame by this margin
scoring.hold_frames             gate: consecutive frames one drone must hold a valid read
viz.enabled                     gates the LIVE top-down window (PNG render always works)
viz.fps                         live view / sampler rate
viz.show_camera_windows         debug: cv2.imshow per-drone frames (never in headless)
monitor.max_cmd_rate_hz         warn when a drone is re-commanded faster than this (sim time)
monitor.warn_on_preempt         warn when a new command preempts an unfinished goal
logging.level / logging.file    sim log level and file (logs/ is gitignored)
```

Defaults that exist only in `simcore/config.py` (override by adding the key
to the YAML): `physics.dt_s` (1/240), `physics.gravity_mps2`,
`physics.max_catchup_steps`, `bodies.drone_half_extents_m`,
`bodies.rover_half_extents_m`.

## Scoring (the referee)

The sim referee renders each flying drone's camera and runs its **own**
`cv2.aruco` — mission code never calls a scoring API. An id is banked once,
globally, when one drone holds a read that is large enough
(`min_marker_px`), fully in frame (`frame_margin_px`), for `hold_frames`
consecutive frames. Score = distinct ids banked; the scoreboard records which
drone banked each id and when (sim time). The same gate scores pads and
rovers.

## Watching the sim (three views) + debugging

- **Top-down 2D** — `python -m scripts.run_sim --live` or
  `python -m scripts.smoke_test --live` (the canned flight scoring in real
  time): map with drones+paths, rovers, pads, coverage, banked ids, score.
  Needs a GUI matplotlib backend (e.g. `sudo apt install python3-tk`).
- **Per-drone cameras** — set `viz.show_camera_windows: true` and run
  `run_sim`: one cv2 window per drone with the live frames detection sees
  (or attach `pyhulax.video.VideoDisplay` to any stream yourself).
- **3D world** — add `--gui` to `run_sim`/`smoke_test`: PyBullet's
  interactive window (orbit/pan/zoom via WSLg) showing wall/obstacle
  heights, drones at altitude, rovers. Headless DIRECT+EGL stays the
  default; `--gui` only changes the connection mode at boot.

`simcore/debug.py` (`DebugProbe`, plus `--debug` / `--dump file.jsonl` on
both scripts) is a READ-ONLY sim-internal introspection tool for tests and
debug scripts: per-tick drone pose in all frames (true vs drifting
estimate), goals, barrier rays, rover waypoints, why each marker id is/isn't
scoring, and command-rate stats — as a plain dict / JSON Lines. It never
mutates anything and is **never imported by mission code** (it is not
reachable through pyhulax).

## Notes

- Everything is headless-safe: tests, smoke run, and `--topdown` PNGs need no
  display. Only `--live` and `VideoDisplay` open windows (WSLg).
- The EGL banner (`GL_RENDERER = D3D12 (NVIDIA ...)`) printed at boot **is**
  hardware acceleration on WSL2 — not a bug, don't "fix" it.
- Determinism: one config seed drives every RNG; same seed ⇒ identical arena,
  rover paths, noise and drift sequences.
- Sim time vs wall time: blocking commands and timeouts run on **sim time**
  scaled by `meta.real_time_factor`, so fast runs behave identically.
- Logs go to the console and `logs/sim.log` (gitignored).
