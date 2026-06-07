# Hula Sim — Update: realism pass (real specs, motion, manual control, dashboard)
## Build spec for Claude Code (continues SIM_UPDATE_PART2.md; Phases 16–21)

> Continues the sim after the two-phase scenario work (Phases 9–15). Same rules: work **phase by
> phase**, make each phase's test pass headless, commit (§8 of the original build plan), then **STOP
> and report**. Existing tests stay green, DIRECT+EGL stays the default, all PyBullet on the sim
> thread, config adds go in BOTH `sim_config.yaml` AND `simcore/config.py` (loader rejects unknown keys).

---

## 0. The SIM / MISSION boundary (unchanged — still the whole point)

This update makes the sim **more realistic and more observable**. It still builds only the **world,
the canned demo, and the views** — never mission intelligence.

- **Sim builds:** realistic flight motion, both control interfaces (`move_to` + `send_manual_control`),
  faithful drift/telemetry, the rover convoy, the dashboard + overlays + camera feeds + scan logging,
  and a canned demo that exercises all of it.
- **Mission builds (other chat, NOT here):** the reactive barrier-flag avoidance strategy, the lock-on
  PID, YOLO detection/approach, search/coverage, swarm coordination. The sim **visualizes** these; it
  never contains them. The demo's flight is a **fixed script**, not a strategy.

Two mechanisms that are easy to conflate, kept separate (per the agreed design):
- **Obstacle avoidance = reactive, boolean.** `get_obstacles()` returns five blocked/clear flags
  (fwd/back/left/right/down) — no distance, no angle, no PID possible. Mission logic reacts to the flags.
- **Target lock-on = PID.** Continuous error (marker/YOLO pixel offset from image-centre) → `send_manual_control`
  stick inputs + gradual `set_camera_angle` pitch. This is mission code; the sim provides the frames
  (the error source) and both control paths.

---

## 1. Real Hula HG-F09 specs (confirmed — bake these in)

From HighGreat's official user manual + the pyhulax SDK. These replace placeholders:

| Spec | Value |
|---|---|
| Camera FOV | **71°** (`camera.h_fov_deg: 71`) |
| Video resolutions | 640×480 (LOW / "AI mode", default for detection), 1280×720 (MED, default video), 1920×1080 (HIGH) |
| Optical-flow position accuracy | horizontal **±20 cm**, vertical ±10 cm → drift magnitude target |
| Programming-mode speed | **0.5–1.0 m/s** (cap `velocity_levels` at ~1.0; SLOW 0.3 / MED 0.5 / ZOOM 0.8 / TURBO 1.0) |
| Climb / descent speed | 1.2 / 1.0 m/s (asymmetric vertical) |
| Max tilt angle | 20° (tilt-to-translate) |
| Weight / dimensions / wheelbase | 100 g / 189×185×50 mm / 128 mm → drone body ≈ 0.19 m, mass 0.10 kg |
| Flight time | 9–10 min → battery drain ≈ 10 %/min |
| Max altitude / comms | 10 m / 50 m |
| IR barrier range | not published (adjustable knob) → keep `barrier_sensors.range_m` a measured/config value |

Detection range at the real 71° FOV (0.15 m marker, 40 px gate): ~1.7 m @ 640×480, ~3.4 m @ 720p,
~5.0 m @ 1080p (geometric; real-world ~20–40% less).

---

## 2. Phase plan

Each phase: **Deliverable → Files → Acceptance test (headless) → Manual check → Commit → STOP.**

### Phase 16 — Real specs into config (foundation, quick)
- **Deliverable:** update config to the confirmed specs above — `camera.h_fov_deg: 71`; `velocity_levels`
  to the 0.5–1.0 band (SLOW 0.3 / MEDIUM 0.5 / ZOOM 0.8 / TURBO 1.0); add `climb_mps: 1.2` /
  `descent_mps: 1.0`; battery drain ≈ 10 %/min (≈9–10 min endurance); drone body dimensions ≈
  0.19×0.18×0.05 m and mass 0.10 kg; add `camera.max_tilt_deg: 20` is N/A (that's airframe tilt — put
  `motion.max_tilt_deg: 20` for Phase 18). Keep `camera` resolution default 640×480 (AI mode) with a
  comment that 720p/1080p extend ArUco range at higher latency. Note FOV/IR-range are "measured —
  confirm on real drone" even though FOV is now sourced.
- **Files:** `sim_config.yaml`, `simcore/config.py` (any new keys), update the body-size constants in `world.py`.
- **Acceptance test:** config loads with the new values; the geometry/scan tests still pass with FOV 71
  (the detection-range expectation updates accordingly); drone bodies are the new size; existing tests green.
- **Manual check:** `run_sim --topdown out.png` still renders; print the loaded FOV / speeds / endurance.
- **Commit:** `feat: bake in confirmed Hula HG-F09 specs (71° FOV, 0.5-1.0 m/s, ~10min battery, real dimensions)`.

### Phase 17 — Rover route fidelity + stationary observers + gradual lock-on (the main thing)
- **Deliverable:**
  - **Rover routes:** make the convoy trace the reference image's two winding loops faithfully (longer,
    smoother branch paths than the rough sketch), still entering staggered from the SW entrance, still
    threading between the crates, then looping to loiter. Tune the branch waypoints so the paths read as
    the orange/yellow loops, not short straight hops.
  - **Stationary observers (demo):** in the canned demo, after landing the drones take off to fixed
    **observation hover points** and largely hold position while the rovers move (matching the image
    where drones hover and rovers drive). They are NOT flying search patterns.
  - **Gradual lock-on stand-in (demo):** when a rover passes near a hovering drone, the demo performs a
    **scripted gradual lock-on** — easing the drone's position a little AND tilting `set_camera_angle`
    smoothly to keep the rover framed, over ~1–2 s (not a snap). This is a canned demonstration of the
    behaviour, not real tracking; assert it's scripted.
- **Files:** `sim_config.yaml` (`rovers.convoy` branch waypoints, demo observation points/lock-on script),
  `rover_model.py`, `scripts/scenario_demo.py`, `config.py`.
- **Acceptance test:** rovers follow the lengthened loop routes (still staggered, still no obstacle
  intersection, still in bounds); in the demo the drones reach their hover points and hold (bounded
  position) during AMBUSH; the scripted lock-on changes BOTH drone position and camera pitch gradually
  over time toward a rover (assert pitch and position move together, smoothly, and it's a fixed script).
- **Manual check:** `scenario_demo --gui` — convoy winds the loops while drones hover and the camera
  visibly tilts to follow a passing rover.
- **Commit:** `feat: faithful convoy loop routes, stationary observer demo, gradual scripted lock-on (pos+pitch)`.

### Phase 18 — Realistic motion model (the honest "not full SITL" version)
- **Deliverable:** replace snap-to-target kinematics with a **realistic motion model** on the same
  command interface: acceleration/deceleration ramps to/from the configured speed (no instant velocity),
  **tilt-to-translate** (the body pitches up to `motion.max_tilt_deg` 20° to accelerate, levels to
  cruise, counter-tilts to stop), momentum + mild **overshoot/settling** at waypoints, a small command
  **latency**, asymmetric climb/descent (1.2/1.0), and **drift calibrated** so `get_position` wanders to
  ~±20 cm horizontal (the optical-flow accuracy), not unbounded. Optional gentle **wind disturbance**
  (`motion.wind`). Make the realism tunable and toggleable so tests can still use crisp motion where
  needed. NOTE: this models *behaviour* (momentum, settling, drift) — it is NOT a firmware-accurate
  dynamics replica (the Hula has no SITL); say so in a comment.
- **Files:** `drone_model.py`, `config.py` (`motion.*`: accel, max_tilt_deg, latency_s, settle, wind),
  keep `frames.py` as the conversion point.
- **Acceptance test:** a move shows accel→cruise→decel (velocity profile, not a step); the body tilts
  during acceleration and levels at cruise (≤20°); waypoint arrival shows bounded overshoot then settle;
  climb≠descent speed; over a long hover/flight `get_position` drift stays bounded near ±20 cm while UWB
  stays in its noise band (the asymmetry holds); crisp-motion mode still passes the old geometric tests;
  square-closure still ~0 with realism off. Existing tests green (adjust any that assumed instant motion,
  honestly, to assert the profile instead).
- **Manual check:** `--dump` a move; offline-plot the velocity/tilt profile showing ramp + settle.
- **Commit:** `feat: realistic motion model (accel/decel, tilt-to-translate, settling, latency, calibrated ±20cm drift)`.

### Phase 19 — `send_manual_control` (the lock-on PID's control path)
- **Deliverable:** implement `send_manual_control(forward, right, up, rotate)` (−1..+1 stick inputs) and
  `manual_fly(duration, ...)` against the motion model — continuous real-time control at ~20 Hz
  (body-relative; positive forward = nose, positive rotate = CCW), distinct from the blocking commands.
  Inputs map to commanded velocity via the configured speed band and obey the same tilt/accel limits and
  barrier clamping (a stick input toward a tripped barrier is clamped). This is the interface a mission
  PID lock-on drives; the sim just executes the inputs faithfully — it contains no PID itself.
- **Files:** `drone_model.py`, `_bridge.py`/`api.py` (wire the real method), `config.py`.
- **Acceptance test:** a held `forward=+0.5` produces steady forward motion at ~0.5× the speed band;
  `rotate` yaws CCW; inputs are body-relative (after a yaw, forward follows the nose); a stick input into
  a tripped barrier is clamped (no penetration); manual control and blocking commands don't fight (last
  command wins via the executor); stopping inputs → the drone coasts to a stop per the motion model.
- **Manual check:** a short script driving `send_manual_control` in a circle; `--gui`/`--dump` shows smooth continuous flight.
- **Commit:** `feat: send_manual_control / manual_fly continuous stick control (lock-on PID control path)`.

### Phase 20 — Concurrent command dashboard (+ keep the boolean proximity overlay)
- **Deliverable:**
  - A **concurrent command dashboard** (separate window or rich console, opened by a `--dashboard` flag
    alongside the 3D sim), refreshed live from `DebugProbe.snapshot()`, showing **per drone**: scenario
    phase; telemetry (UWB pos, get_position + drift error, altitude, heading, battery); current command
    /goal + progress, OR the live `send_manual_control` stick inputs (fwd/right/up/rotate) when in manual
    mode; the **five barrier flags** (fwd/back/left/right/down) as on/off indicators; and a **scan status**
    line (markers banked, by this drone, with sim-time). Read-only, `simcore` only, never `pyhulax`.
  - **Keep the existing proximity overlay** in the 3D/top-down view, but as **plain boolean directional
    indicators** (the direction's marker lights up when that flag is set) — NO wedge angle, NO `cone_deg`,
    no invented geometry. It shows what's in range directionally; the dashboard shows the numbers.
- **Files:** `scripts/` (dashboard runner, `--dashboard`), `simcore/debug.py` (expose what the dashboard
  needs; it already gathers most), `viz.py` (simplify the proximity overlay to boolean directional), `config.py`.
- **Acceptance test:** the dashboard renders a per-drone snapshot with the fields above and updates over
  ticks; it shows `move_to` goal in blocking mode and stick inputs in manual mode; the barrier indicators
  reflect the five flags; it's read-only (a probe-invariance test) and reads no `pyhulax`; the overlay is
  boolean directional (no angle param anywhere); headless runs unaffected when `--dashboard` is off.
- **Manual check:** `scenario_demo --gui --dashboard` — the 3D window plus a live per-drone panel; fly
  toward a crate and watch the forward indicator light in both.
- **Commit:** `feat: concurrent command dashboard (telemetry/commands/flags/scan) + boolean directional proximity overlay`.

### Phase 21 — Deliberate scan logging + camera feeds + rover visuals for YOLO
- **Deliverable:**
  - **Camera feeds:** ensure `viz.show_camera_windows: true` opens each drone's live frame during the
    scenario/demo, with detected ArUco markers **outlined + id labelled** on the frame (so you see what
    the drone sees).
  - **Deliberate scan logging:** when the referee **banks** a marker, save that frame to disk
    (`logs/scans/`) with the marker boxed + a log line (drone, id, px size, sim-time) — visual proof of a
    scan, not just a score bump. Throttled/deduped so it saves the banking frame, not every frame.
  - **Rover visual fidelity for YOLO:** improve the rover appearance so it reads as a RoboMaster-style
    ground robot (better than a flat billboard) — enough that a YOLO model could plausibly detect it on
    the rendered frame. (YOLO itself is mission code run on the frames; this just makes the rovers
    detectable. Note in a comment that the real model is validated on real footage / may need sim fine-tuning.)
- **Files:** `viz.py` (camera windows + outline), `scoring.py`/`camera.py` (on-bank frame save + log),
  `rover_model.py` (rover visual), `config.py` (`logging.scans`, toggles).
- **Acceptance test:** camera windows draw the marker outline + id when present; on a bank, a frame file
  is written with the marker boxed and a log line recorded (and only on the banking event, not spammed);
  rover render produces a recognisable robot silhouette (basic check: non-trivial geometry/texture, not a
  flat plane); headless runs still work (windows/saves gated by flags).
- **Manual check:** `scenario_demo --gui` with camera windows — watch a marker get outlined and a saved
  scan image + log line appear when it banks.
- **Commit:** `feat: ArUco-outlined camera feeds, on-bank scan image logging, RoboMaster-style rover visuals for YOLO`.

---

## 3. Confirm / measure on the real drone (short)

- **IR barrier range** — adjustable knob on the drone; set it and measure, then drop into
  `barrier_sensors.range_m`. (Range only; no angle.)
- **UWB noise** — the organisers' external system; sim assumes ~5 cm in `uwb.noise_std_m`; confirm on the day.
- **FOV** — manual says 71°; trust but verify with one marker-at-known-distance shot.
- **Optical-flow drift** — calibrated to ±20 cm horizontal; adjust `position_drift` if the real drone differs.

## 4. Reference

Same `reference/` materials as Part 2 (the brief + images). The confirmed specs above come from
HighGreat's official Hula user manual and the pyhulax SDK.
