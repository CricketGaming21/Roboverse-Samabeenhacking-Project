# PHASE_PLAN.md — the autonomous spine (same cadence as the sim build)

Phases run **in order**. Each has an **objective gate** (a `pytest -m pN` selection that must pass,
then the full suite). Auto-advance on green; stop + `BLOCKED.md` on a hard block (see CLAUDE.md).
Everything below is testable against the **fake SDK substrate** (`tests/README.md`) — no sim, no
hardware needed overnight. Signatures are the **contract**; match them so phases compose.

Frame convention used throughout: **arena/UWB** `x=North(m), y=East(m)`; **body** `forward,right,up`;
drones lock heading at takeoff (one `YAW_OFFSET`), so body↔arena is a fixed rotation. `send_manual_control`
sticks are −1..1 (`+forward,+right,+up,+rotate=CCW`). pyhulax distances are **cm**; UWB is **m**.

---

## P0 — Test substrate (fake SDK)  ·  gate: `make gate-p0`
**Goal:** a deterministic, headless fake of the public API so every later phase is TDD-able.
**Files:** `tests/fakes/fake_pyhulax.py`, `tests/fakes/fake_uwb.py`, `tests/conftest.py`,
`tests/test_fakes.py`. Implement exactly the surface in `docs/SDK_REFERENCE.md`:
- `FakeDroneAPI` — **mirror the sim's SUBSET exactly** (do NOT add `set_app_mode/send_app_heartbeat/
  set_velocity_level/stop_manual_control/arm/disarm/disconnect/get_velocity/get_drone_id/
  enable_battery_failsafe` — their absence is what forces the compat shim): `connect(ip)` (required ip,
  returns truthy CommandResult), `takeoff(height_cm)`, `land`, `hover(duration_seconds)`,
  `send_manual_control(forward,right,up,rotate)->bool` (kinematic: stick·cap → velocity, integrate
  x,y,alt; clamp to a configurable speed cap), `manual_fly`, `set_camera_angle(mode,angle)`,
  `set_barrier_mode`, `set_avoidance_direction`, `set_video_stream`, `create_video_stream()`
  → `FakeVideoStream(.start/.stop/.latest_frame)`, `get_position()->Vector3` (cm, onboard estimate
  with optional bounded drift), `get_orientation()->Orientation`, `get_altitude()->float`,
  `get_battery()->int`, `get_obstacles(drone_id=0)->Obstacles(forward,back,left,
  right,down)` (**no `up`**), `any_obstacle()`, `get_drone_status()` (raw bitmask). (No `get_velocity`.)
- A shared **FakeWorld**: arena rect + crate footprints + pad markers (ids 10–14) + rover positions
  (each a marker id) → drives `get_obstacles` (boolean if a footprint/drone is within range in a
  direction) and the camera. `FakeVideoStream.latest_frame.to_rgb()` returns an RGB ndarray that
  **renders the real ArUco markers in view** (`cv2.aruco.generateImageMarker`) at the correct pixel
  position/size given drone pose + camera tilt + FOV, so `cv2.aruco` genuinely decodes them.
- `FakeUWBParserThread`: `get_tag_position(tag_id)->(x_m,y_m,t)` truth + optional Gaussian noise +
  optional `(None,None,None)` dropout. Same method names as the real `UWBParserThread`.
- `conftest.py`: install fakes into `sys.modules` so the mission's `import pyhulax` /
  `from UWBParserThread import UWBParserThread` resolve to the fakes during tests. Global seed fixture.
The fake mirrors the sim's SUBSET (no `up` on Obstacles; NO `set_app_mode/heartbeat/set_velocity_level/arm/disconnect/get_velocity`), so unguarded real-only calls fail (forces the compat shim). `connect(ip)` needs an ip; `hover(duration_seconds)` needs a duration.
**Gate criteria:** sticks move position the right way; speed clamp holds; `get_obstacles` returns
booleans consistent with the world; a rendered frame with a known marker decodes to that id via
`cv2.aruco`; UWB returns truth±noise and the documented `(None,None,None)`. Deterministic under seed.

## P1 — Config + frames + UWB control loop  ·  gate: `make gate-p1`
**Goal:** the metres/cm + arena/body machinery and `fly_to_uwb`.
**Files:** `src/mission/config.py` (pydantic load of `config/mission_config.yaml`, **reject unknown
keys**), `src/mission/frames.py`, `src/mission/control/uwb_loop.py`, `src/mission/runtime/sdk_compat.py`.
- `sdk_compat.py`: `prepare_manual_control(drone)` calls `set_app_mode(1)`/`send_app_heartbeat()`/`set_velocity_level(...)` **only via `hasattr`** (no-op on sim, real on hardware); `release(drone)` guards `stop_manual_control()`/`disconnect()`. The mission's manual-control init goes through this.
- `frames.py`: `arena_to_body(ex_m, ey_m, yaw)->(fwd,right)`, `body_to_arena(...)`, `m_to_cm/cm_to_m`,
  `clamp_speed(vx,vy, cap=0.5)`. Sign/axis/yaw-zero are config constants (calibrated on the day).
- `uwb_loop.py`: `fly_to_uwb(drone, uwb, tag_id, target_xy_m, alt_m=1.1, tol_m=…, *, guard=None,
  rate_hz=20) -> bool`. Closed loop: read UWB → arena error → body velocity (P/PI, **≤0.5 m/s**) →
  altitude hold on `get_altitude()` → arrive within tol & speed. **Hold position on `(None,None,None)`**.
  Keep emitting at `rate_hz` (manual-control is heartbeat-like). `guard` (P3) may filter the command. **Arrival 'speed < tol' uses UWB-derived speed (Δpos/Δt) — the sim has NO `get_velocity()`.** Use `connect(ip)` (explicit ip) + `sdk_compat.prepare_manual_control` before takeoff; station-keep with `send_manual_control()` zeros (NOT `hover()`).
**Gate criteria:** transforms are inverse-consistent and match hand-worked cases; controller converges
a fake drone from many start poses to within tol; never exceeds 0.5 m/s; holds on UWB dropout;
unknown config keys raise.

## P2 — Crate map + planner geometry + projection  ·  gate: `make gate-p2`
**Goal:** the offline routing math (no GUI yet) + pixel→arena projection.
**Files:** `src/mission/planner/geometry.py`, `src/mission/planner/projection.py`, `src/mission/planner/arena.py` (load `config/arena_truth.yaml` via `yaml.safe_load` — the sim emits it; Discord coords on the day; **never import simcore**).
- `geometry.py`: `inflate(footprints, radius_m)`, `build_graph(inflated, bounds)` (visibility graph
  or grid), `plan_path(start_xy, goal_xy, graph) -> [waypoints]`, `segment_clear(p, q, inflated)->bool`,
  `paths_conflict(pathA, pathB, sep_m)->bool`.
- `projection.py`: `pixel_to_arena(u, v, drone_xy, yaw, alt_m, cam_pitch_deg, intrinsics) -> (x_m,y_m)`
  on the flat floor (geometry, **no depth**).
**Gate criteria:** inflation correct; planned paths **never** intersect an inflated footprint or leave
bounds; known maps give expected connectivity/None-when-blocked; projection matches synthetic ground
truth within tolerance; conflict check flags crossing/too-close paths.

## P3 — Reactive avoidance guard + plan-then-guard  ·  gate: `make gate-p3`
**Goal:** the boolean backstop wrapping every command; inter-drone separation.
**Files:** `src/mission/control/avoidance.py`.
- `class ReactiveGuard`: `filter(cmd_fwd, cmd_right, obstacles, *, open_side_hint=None) ->
  (fwd, right, up)` — zero the travel-direction axis if blocked, slide toward the open lateral side,
  **never return up>0**; expose `boxed()` when no lateral escape (caller backtracks + reroutes).
- `separation(my_xy, others_xy, my_priority, sep_m) -> yield: bool` (lower priority yields).
- Integrate into `fly_to_uwb` via its `guard=`.
**Gate criteria:** guard never emits +up; stops the blocked axis; picks the open side; reports `boxed`
when surrounded; lower-priority drone yields; an integration test: a fake drone with a surprise
obstacle on its planned path still reaches goal **without entering any footprint**.

## P4 — Phase 1: deploy + land in hoop  ·  gate: `make gate-p4`
**Goal:** per-drone state machine through landing; dynamic 3-of-5 assignment.
**Files:** `src/mission/mission/phase1_land.py`, `src/mission/mission/worker.py` (states INIT, TAKEOFF,
GO_TO_PAD, LAND_HOOP).
- `assign_pads(valid_pads, drone_starts) -> {tag_id: pad}` (pick 3, minimise total path + avoid
  crossing). `land_in_hoop(drone, uwb, tag_id, pad_xy, hoop_tol_m)` — centre on UWB; **decode-to-confirm**
  the pad ArUco if visible (optional, defensive); descend only when centred-in-hoop AND the descent
  column is footprint-clear AND `down` clear above 0.35 m.
**Gate criteria:** 3 fake drones each route to their assigned pad (paths footprint-clear) and land
within `hoop_tol_m`; assignment selects the correct 3; no drone overflies a footprint.

## P5 — Perception: ArUco + detector seam + two-stage  ·  gate: `make gate-p5`
**Goal:** detection, behind the public API, mirroring `reference/provided_code/rover_detection_example.py`.
**Files:** `src/mission/perception/aruco.py`, `src/mission/perception/detector.py`.
- `aruco.py`: `confirm_with_aruco(frame_rgb, dict="DICT_6X6_250") -> [Detection]` (RGB→GRAY;
  multi-marker per frame; returns id + bbox). Pads 10–14 vs rovers (**any other id**).
- `detector.py`: `Detection` dataclass + `RoverDetector` ABC + `ClassicalRoverDetector` (PRIMARY,
  training-free: colour/contour/motion) + `PlaceholderRoverDetector` + `two_stage_scan(stream,
  detector, approach=None) -> ScanResult` (YOLO/classical find → approach → ArUco confirm). Optional
  YOLO subclass behind the same seam (do not require ultralytics).
**Gate criteria:** synthetic frames (incl. several markers in one frame, tilted, near the 40-px gate)
decode to the right ids; pad/rover split correct; classical detector returns plausible boxes on a
rendered rover; `two_stage_scan` wires the stages and returns confirmed ids.

## P6 — Shared world model  ·  gate: `make gate-p6`
**Goal:** the lock-guarded belief-and-tasking brain.
**Files:** `src/mission/world/mission_state.py` (tagged-set + evidence), `world/taskboard.py`
(sightings + per-drone assignments), `world/belief_grid.py`, `world/coordinator.py`.
- `MissionState.bank(id, frame, xy, t)`, `.tagged() -> set`, `.remaining(all_ids)`, `.evidence()`.
- `TaskBoard.see(track)`, `.tracks()`, `.assign(tag_id, role, target)`, `.assignment(tag_id)`.
- `BeliefGrid`: occupancy field over free cells; `.observe(footprint_cells)` (collapse), `.diffuse(dt,
  rover_speed)` (spread along lanes), `.spike(xy)` (sighting), `.argmax_region()`. Chokepoint/lane
  graph extracted from the crate map (`.chokepoints()`).
- `Coordinator.step(state, taskboard, belief, drones)` → role/target per drone.
**Gate criteria:** de-dup by id; belief collapses where observed and diffuses over time; sighting spikes;
chokepoint graph extracted from a known map; coordinator assigns sane roles on scripted scenarios.

## P7 — Phase 2: search + lock-on + tag  ·  gate: `make gate-p7`
**Goal:** persistent **vantage patrol / chokepoint overwatch** + lock-on; tag distinct ids.
**Files:** `src/mission/mission/phase2_search.py`, extend `worker.py` (RELAUNCH, SEARCH, HOME,
CONVERGE).
- `vantage_patrol(drone, uwb, vantages)` (cycle preplanned look-points from `mission_plan.yaml`, dwell,
  re-observe lanes — **not** a one-pass lawnmower). `lock_and_tag(drone, uwb, detection)` — PID on
  marker pixel-offset → `send_manual_control` (+camera tilt/yaw), **bounded & interruptible**, velocity-
  match a moving rover for the 5-frame hold, time-boxed, bank to `MissionState`. Bubble-gating with
  **commitment rule** (finish a started lock across a boundary) + **mop-up** endgame (drop gating when
  only evaders remain).
**Gate criteria:** with scripted autonomous rovers on loops, 3 fake drones tag all **distinct** ids
within a budget; no double-count; bubble-gate + commitment + mop-up behave; locks are time-boxed
(no deadlock); never overflies a footprint; ≤0.5 m/s throughout.

## P8 — Adversarial evader handling  ·  gate: `make gate-p8`
**Goal:** the playbook for the 2 human-piloted evaders, driven by P6.
**Files:** `src/mission/mission/phase2_search.py` (evader logic) + coordinator hooks.
- Behaviour triage (smooth/periodic vs erratic/reactive); **secure the 3 autonomous tags first**;
  belief-grid pursuit (go to argmax region); cooperative **containment** (hold chokepoints to shrink
  the evader's reachable set on the lane graph); bait/flush; watch-the-cover (stale crate shadows).
**Gate criteria:** against a scripted *evasive* fake rover, containment monotonically shrinks its
reachable set; roles assigned for pursuit; the 3 easy tags precede evader commitment; time-box rotation
prevents deadlock. (Full realism needs the sim's evasive/teleop mode — that's an `integration` test.)

## P9 — Mission Planner GUI  ·  gate: `make gate-p9`
**Goal:** single-file browser tool: author Phase-1 routes + Phase-2 vantages, export the contract.
**Files:** `planner_gui/index.html` (canvas; no build step), `tests/test_plan_schema.py`.
- Load arena/crates/pads (paste or default to the sim map); draw inflated footprints; place/drag
  Phase-1 waypoints per drone with **live validation** (red on footprint/bounds breach) + auto-route
  (visibility-graph) + inter-path conflict highlight; place Phase-2 vantages (pos + look-yaw + tilt +
  dwell) with **camera-footprint + coverage** overlay; export `mission_plan.yaml` + a PNG.
**Gate criteria (headless):** the committed `examples/mission_plan.example.yaml` validates against
`docs/MISSION_PLAN_SCHEMA.md` and **loads cleanly in the mission** (P4 assign + P7 vantages consume it).
(The GUI's interactivity is verified by a human; the gate locks the export contract.)

## P10 — C2 operator console  ·  gate: `make gate-p10`
**Goal:** live operator UI (browser), reading the public API + world model.
**Files:** `c2/index.html` (+ a thin `src/mission/runtime/c2_bridge.py` serving state as JSON),
`tests/test_c2_bridge.py`.
- Live top-down (drones, footprints, paths/zones, rover tracks, **belief heatmap**), 3 camera tiles
  with detection boxes, **rubric scoreboard** (landings; distinct ids with capture thumbnails;
  outstanding ids; clock), **alarms** (low battery, UWB dropout, near-collision, **drone over a crate
  footprint** = no-fly violation), pre-flight checklist, one-click **evidence export** (annotated
  images + results sheet + map snapshot), and an **optional override layer** (re-task/designate/recall/
  hold-all) that injects into the coordinator.
**Gate criteria:** bridge serves valid JSON of mission state from the fake harness; evidence export
produces the bundle; override commands reach the coordinator (unit-tested); the over-footprint alarm
fires on a synthetic violating pose.

## P11 — Full integration + reliability + sim handoff  ·  gate: `make gate-p11`
**Goal:** end-to-end on the fake harness; failsafes; the swap doc.
**Files:** `src/mission/runtime/discovery.py` (`try: from pyhulax.discovery import Dola` `except ImportError: from dola import Dola`; `get_all_ips()->{plane_id: ip}`; map `plane_id`↔UWB `tag_id`; fixed config ips work in-sim),
`src/mission/runtime/main.py` (spawn `DroneWorker`s, monitor, **reassign a dead drone's zone**,
shutdown lands all in `finally`), `docs/SIM_VS_REAL.md` (the import-path swap + the on-the-day
calibration checklist), `tests/test_integration_fake.py`.
**Gate criteria:** a full Phase1→Phase2 run on the fake harness scores 3/3 landings + all distinct tags;
battery/UWB-dropout/worker-death failsafes covered; shutdown lands all; all prior gates still green.

---

### Notes for the runner
- Tag each test with its phase marker (`@pytest.mark.pN`). The `integration` marker = real sim only;
  it is **excluded** from every gate and from the overnight loop — implement those tests but expect a
  human to run them supervised.
- Keep `examples/mission_plan.example.yaml` valid as the schema evolves (P4/P7/P9 all depend on it).
- `[SYNC-WITH-SIM]` values in `config/mission_config.yaml` (arena, ids, frame) mirror the sim; a human
  re-syncs them after sim updates. Don't invent arena geometry — read it from config.
