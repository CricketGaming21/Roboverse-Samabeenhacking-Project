# Hula Sim — Update: physics rework, MP4 cockpit UI, manual-control groundwork
## Build spec for Claude Code (continues SIM_UPDATE_PART4.md; Phases 24–27)

> Same rules: phase by phase, tests pass headless, commit (§8), STOP and report. Existing tests stay
> green, DIRECT+EGL default, all PyBullet on the sim thread, config in BOTH `sim_config.yaml` AND
> `simcore/config.py` (loader rejects unknown keys). Observer/viz and physics tuning only — the public
> pyhulax/UWB surface stays frozen and untouched; scoring/scenario logic unchanged unless stated.

---

## 0. Context

The realism pass (Phase 18) tuned the motion model too aggressively → the drones are visibly **wobbly**
(underdamped oscillation + wind + tilt rocking + command thrash). We went headless+record (Phase 22–23)
to beat the WSLg GUI freeze, which means the live dashboard/camera views are gone — they need to move
**into the MP4**. This update: (24) rework the physics to smooth+stable+slower+deliberate; (25–26) turn
the MP4 into a rich multi-panel cockpit; (27) lay groundwork for a live 3D-only manual-control viewer.

**Fidelity reminder:** the pyhulax/UWB public surface is the contract that makes mission code transfer —
do not add to it or change its semantics. The physics is a *behaviour model*; the goal is realistic,
smooth behaviour, not a firmware replica.

---

## 1. Phase plan

### Phase 24 — Physics rework + deliberate pacing (PRIORITY)
Diagnose first, then fix. **Read `drone_model.py` (the Phase-18 `_step_realistic` / `_apply_velocity_dynamics`)
and report what produces the wobble before changing anything.** The likely contributors: an underdamped
position controller (overshoot → oscillation), the OU wind disturbance, tilt-to-translate body rocking,
high-frequency drift, and command thrash.

- **Deliverable — kill the FAST wobble, keep the SLOW realism:**
  - **Critically-damped motion:** the drone eases to its target and settles **without oscillating**
    (damping ratio ≈ 1.0, `overshoot_frac` ≈ 0 / removed). No ringing, no back-and-forth.
  - **Rate-limited tilt:** body/camera tilt changes **smoothly** (cap on deg/s) — no snapping or rocking.
  - **Wind OFF by default** (`motion.wind.enabled: false`); when enabled it must be gentle (a hovering
    drone should not visibly sway). 
  - **Drift = slow + low-frequency:** the realistic ±20 cm optical-flow wander stays, but as a **slow**
    drift (seconds timescale), NOT high-frequency jitter. `get_position` wanders ≤ ±20 cm; the rendered
    body moves **smoothly** (no vibration). Keep UWB as truth+small-noise, no drift.
  - **Fix command thrash:** the demo's part-1 flight (`scenario_demo._fly_deploy`) issues move/hover
    commands ~0.004 s apart on some drones (monitor warns). Make ALL part-1 drones **wait for each
    blocking command to complete** before issuing the next (match the non-thrashing drones). No thrash
    warnings in a clean demo run.
- **Deliverable — slower & more deliberate:**
  - Fly the **demo** at real programming-mode speed: **0.3–0.5 m/s** (use SLOW/MEDIUM, not the 0.8–1.0
    cap). Keep the band cap at 1.0 and climb 1.2 / descent 1.0 (real specs, unchanged).
  - **Longer episode:** raise `scenario.episode_seconds` (e.g., 240–300 s) so every system is observable.
  - **Slower rovers:** lower `rovers.convoy.speed_mps` (~0.2–0.3 m/s) and make sure speed/stagger/routes/
    loiter are all clean, documented config keys.
- **Files:** `drone_model.py` (damping, tilt rate-limit, wind default, low-freq drift), `scenario_demo.py`
  (command pacing + slower demo speed), `sim_config.yaml` + `config.py` (episode length, rover speed,
  motion damping/tilt-rate/wind keys).
- **Acceptance test (headless):** a move settles **monotonically or with ≤1 small overshoot, no
  oscillation** (assert the position trace doesn't cross/ring); tilt rate is bounded (|Δtilt/dt| ≤ cap);
  with wind off a hovering drone stays within a few cm; `get_position` drift is low-frequency and ≤ ±20 cm
  while the **true body pose is smooth** (assert no high-frequency content in the true trajectory);
  **no thrash warnings** in a demo run; demo speed ≤ 0.5 m/s; episode length and rover speed come from
  config and rovers are slower. Crisp-motion mode still exact (square-closure ~0). Existing tests green.
- **Manual check:** `scenario_demo --record run.mp4` — drones fly **smoothly and slowly**, hover steadily
  (no wobble), rovers crawl, the run is long and deliberate.
- **Commit:** `fix: rework motion model to critically-damped smooth flight (no wobble) + slower deliberate pacing`.

### Phase 25 — MP4 cockpit: per-drone telemetry + proximity UI + translucent walls
- **Deliverable:**
  - **Translucent walls:** render the arena walls semi-transparent (alpha) so the third-person view sees
    objects behind them. (Render/visual only — collision/geometry unchanged.)
  - **Per-drone telemetry panel** in the recording (from `DebugProbe.snapshot()`): speed, position (UWB
    **and** the drifting estimate), heading / direction of travel, camera pitch angle, yaw/pitch/roll, the
    manual stick inputs (fwd/right/up/rotate), a **UWB-OK indicator** (green good-fix / amber-red
    no-fix), and battery.
  - **Per-drone car-style proximity graphic:** a small drone icon with the five directional segments
    (fwd/back/left/right/down) lighting **red when that barrier flag is set** (boolean — mirrors
    `get_obstacles()` exactly, no distance/angle). Parking-sensor style.
  - **Layout:** arena view + scoreboard on top (Phase 22), and **per drone a column** below: camera feed
    (Phase 23) + telemetry panel + proximity graphic. Compose with cv2 on the existing offscreen canvas;
    keep it readable (labels, spacing).
- **Files:** `simcore/recorder.py` (panels + layout), `simcore/debug.py` (expose any missing fields:
  speed, heading, ypr, uwb-ok), `world.py`/`viz.py` (translucent wall material), `config.py` (panel toggles).
- **Acceptance test (headless):** the composited frame contains, per drone, a telemetry panel with the
  listed fields and a proximity graphic; telemetry values match the probe snapshot; proximity segments
  reflect the five flags (lit when set); walls render translucent (alpha in the visual); the
  scoreboard-equality **fidelity** check still holds (panels don't perturb the sim); the path never
  connects `p.GUI`; existing tests green.
- **Manual check:** `scenario_demo --record run.mp4` — arena on top, and each drone's camera + live
  telemetry + parking-sensor graphic below; fly a drone near a crate and watch its forward segment light.
- **Commit:** `feat: MP4 cockpit — per-drone telemetry panels + car-style proximity UI + translucent walls`.

### Phase 26 — MP4 target-acquisition UI + maximally-informative polish
- **Deliverable:**
  - **Deliberate acquisition UI** on each drone's camera feed: a **yellow "DETECTED id N"** box the moment
    `cv2.aruco` finds a marker in that drone's frame (reuse `camfeed.annotate_markers` — real detection on
    the rendered frame, not a fake overlay), turning **green "ACQUIRED id N"** when the referee banks it;
    a brief highlight/flash on the first bank. So the video shows detection happening AND the moment it scores.
  - **Polish for information density:** clear legends/keys, consistent colour coding (e.g., green=good/
    scored, amber=warning, red=blocked/bad), a frame timestamp, phase banner, and a compact per-drone
    status line (mode: idle/blocking/manual, current goal or sticks). Maximise clarity without clutter.
- **Files:** `simcore/recorder.py` (acquisition box state machine + polish), reuse `camfeed.annotate_markers`,
  `config.py` (toggles).
- **Acceptance test (headless):** a detection box draws (yellow) when a marker is in a drone's view; it
  shows acquired (green) when that marker is banked; the detection is the real `cv2.aruco` path; legends/
  status render; fidelity (scoreboard equality) holds; no `p.GUI`; existing tests green.
- **Manual check:** `scenario_demo --record run.mp4` — watch a rover's marker get a yellow DETECTED box in
  a drone's feed, then turn green ACQUIRED as it banks; the frame is clearly legible end to end.
- **Commit:** `feat: deliberate target-acquisition UI (detected->acquired) + maximally-informative recording polish`.

### Phase 27 — Live 3D-only viewer + keyboard-control groundwork
- **Deliverable:**
  - **Live 3D-only viewer:** a mode (e.g., `--view3d` / `scripts/live_view.py`) that opens the PyBullet
    `p.GUI` 3D window with **ALL camera rendering disabled** — no referee scanning, no camera insets, no
    drone-camera renders — so it **cannot** hit the `getCameraImage`-vs-GUI freeze. Third-person view of
    the world for watching/flying. (Part 1 ran fine in GUI before camera renders started; this keeps the
    sim permanently in that safe subset.)
  - **`scripts/keyboardcontrol.py`:** drive a selected drone via **`send_manual_control`** from the
    keyboard — e.g. WASD = forward/back/left/right, R/F = up/down, Q/E = yaw, arrow keys = camera tilt —
    at ~20 Hz, in the live 3D-only view. Groundwork for manual piloting; uses the Phase-19 control path.
  - **HONEST CONSTRAINT (document it):** this live mode shows the **3D world only, NOT the FPV camera** —
    showing a live camera feed here would reintroduce the freeze. For camera/FPV review, use `--record`.
    Two separate modes by design.
- **Files:** `scripts/live_view.py` + `scripts/keyboardcontrol.py`, a flag/guard ensuring camera rendering
  is fully disabled in this mode, `config.py` (key bindings / view options).
- **Acceptance test (headless, no window opened):** in the live-3D mode, **no camera-render path is
  active** (assert referee/insets/drone-camera rendering are disabled — so it can't freeze);
  `keyboardcontrol` maps each key to the correct `send_manual_control` stick values (pure-function test:
  key → (forward,right,up,rotate)/camera-tilt); the control path is the real `send_manual_control` (no new
  surface); existing tests green. (The GUI window itself isn't opened in tests — no display in CI — the
  guard is proven by mode/state, like the Phase-21b GUI-render test.)
- **Manual check (on a machine with a display, or to attempt on WSLg):** `live_view` opens a 3D window and
  `keyboardcontrol` flies a drone with the keyboard, no camera feeds, no freeze.
- **Commit:** `feat: live 3D-only viewer (camera rendering disabled, freeze-proof) + keyboardcontrol groundwork (send_manual_control)`.

---

## 2. Notes
- Physics goal restated: **smooth, critically-damped, slow, deliberate** flight that *behaves* like a real
  drone; the realistic **slow** ±20 cm drift stays (mission must handle it), the **fast wobble** goes.
- The recording stays the primary way to watch on WSLg; the live 3D-only viewer is for manual flying and is
  freeze-proof because it renders no cameras. FPV/camera review = `--record`.
- Public pyhulax/UWB surface is frozen throughout. Real specs (FOV 71°, ±20 cm drift, 0.5–1.0 m/s, ~10 min,
  dimensions, DICT_6X6_250) are confirmed and already in config — unchanged here.
- Confirm on the real drone: IR barrier range (knob), UWB noise (organisers'), FOV sanity check.
