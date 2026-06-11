# PROGRESS — updated by Claude Code at every phase boundary

> Append one block per phase. Keep newest at top. A human waking up can see status in 10 seconds.

| Phase | Status | Tests (gate / full) | Commit | Notes |
|------|--------|---------------------|--------|-------|
| P0   | ✅ GREEN | 41 / 41            | 54444c9 | fake SDK substrate |
| P1   | ✅ GREEN | 26 / 67            | 6b12d91 | config + frames + fly_to_uwb |
| P2   | ✅ GREEN | 19 / 86            | d29f1ec | planner geometry + projection |
| P3   | ✅ GREEN | 13 / 99            | de673b8 | reactive avoidance guard |
| P4   | ✅ GREEN | 7 / 106            | 75f1195 | phase-1 deploy + land in hoop |
| P5   | ✅ GREEN | 8 / 114            | bc766fd | perception: aruco + detector seam |
| P6   | ✅ GREEN | 12 / 126           | 2973a80 | shared world model |
| P7   | ✅ GREEN | 9 / 135            | d7aaac1 | phase-2 search + lock-on + tag |
| P8   | ✅ GREEN | 9 / 144            | 77e2a0b | adversarial evader handling |
| P9   | ✅ GREEN | 9 / 153            | 759bd7f | planner GUI export contract |
| P10  | ✅ GREEN | 10 / 163           | f0cd72d | C2 operator console |
| P11  | ✅ GREEN | 7 / 170 (+1 skip)  | (this) | full integration + reliability |

**ALL PHASES GREEN — full suite: 174 passed, 1 skipped (the real-sim `integration` test).**

## R2 — Phase-2 search/read tuned for the MOVING GIMBAL (persistence) — fake gate green (221 passed, 1 skipped)
The sim models each rover's marker on a gimbal: its facing yaw sweeps (45°/s, 8 s period) and a
drone DECODES it only within ±60° of the rover→drone bearing — **never from directly overhead**.
Out of the cone the camera sees the rover BODY but cv2.aruco finds no marker. Last run the mission's
OWN tally lagged the referee (**2/5**) because lock-on needed 5 *consecutive* decodes but `lock_timeout`
(6 s) was **shorter than the 8 s sweep** — it gave up before the cone returned. R2 closes that.
TDD, one commit per change, full fake suite green throughout; **Phase-1 landing path untouched** (still
3/3 ≤3 cm); **public pyhulax surface unchanged**.
- **Fake gimbal+body model** (commit `cf60560`): the fake mirrors the sim — sweeping marker yaw +
  readable cone (overhead→False), and a bright BODY blob when out of cone. OFF by default so the 211
  existing always-decodable tests are unchanged; R2 tests opt in. New `r2` pytest marker.
- **Config** (commit `585528e`): `camera.search_pitch_deg` (52, a MODERATE forward tilt held for search
  AND read — never nadir) + `camera.use_nadir_search` (90° baseline fallback). New `search:` section —
  `persist_timeout_s` (9 s, past one sweep), `orbit_step_m`, `max_orbits`, `presence_min_area_px`. All
  defaulted → real profile loads unchanged.
- **Persistence + held search pitch** (commit `66d3545`): when a drone sees a rover BODY but can't decode
  (out of cone), `persist_and_read` holds on it at the search pitch — never steepening to nadir — keeping
  it framed until the sweeping gimbal brings the marker into the cone, then banks. The target stays ACTIVE
  across ticks (a seen-but-unread rover is not dropped). A **light lateral orbit** (≤ `orbit_step_m`, no
  overfly, never +up) breaks a stuck hold after the timeout — a fallback, not the primary move.
  Bank-and-release on an already-banked id. **Projection uses the live camera pitch each frame.** Opt-in
  via a `presence` detector → p7/p11 (nadir) unchanged. Multi-bearing stays a free bonus (3 disjoint
  zones) — no pincer logic here (that's P8).
- **Live wiring** (commit `f409bd4`): `search_pitch_deg()` / `r2_phase2_kwargs()` build the Phase-2 kwargs
  (held pitch + body-presence detector + persist/orbit budget) from config; the runner and the real-sim
  integration both use them.

**Real-sim convoy integration (gimbal ON, `python -m mission.runtime.main`, sim scorers authoritative):**
- LandingScorer **3/3 in-hoop** (pads 11/45/51; mission-side ≤5.7 cm).
- **Mission's OWN tally: 5/5 distinct rover ids** `[11,45,51,67,101]` — up from **2/5 pre-R2**; the sim
  ended the ambush early (`phase DONE … score 5` at t=99 s). Referee banked all 5 independently.
- **0 compliance flags.** No errors; drone 2 hit the battery-RTL failsafe mid-Phase-2 (persistence costs
  battery) and landed safely — the other two still covered all 5 (count-first conservatism held).

## Sim re-sync to the overhauled hula_sim (SIM profile only — real-day profile untouched) — fake gate green (211 passed, 1 skipped)
The `hula_sim` was overhauled (new landing zones + arena geometry), so the mission's planner inputs
were stale and the real-sim convoy landed in empty space. Re-synced the **sim-testing inputs only**;
`mission_real.yaml` values are untouched (real coords arrive from Discord on the day). TDD, one
commit per change, full fake suite green throughout.
- **`load_arena` reads the sim's new `structures:` key** (crates + arch posts) with a `crates:`
  fallback — one loader for both (commit `2d5b2f3`). Tests cover both keys.
- **Regenerated `config/arena_truth.yaml`** from the sim's `emit_arena_truth` — new `structures` +
  `landing_zones`, arena 10×6 (commit `cf56f09`). `test_plan_on_real_arena_truth` retargets to pad 51.
- **Sim profile pads mirror the sim's landing zones**: 11→(4.40,1.35) 45→(7.85,1.30) 51→(4.40,4.40)
  valid+designated; 67/101 invalid — clearly commented as **SIM** coords (real = Discord, NOT copied
  here). `test_p1` valid-pad count 4→3; example plan pad_ids → 11,45,51 (commit `d624fd7`).
- **`mission_real.yaml` pads marked `# [FROM DISCORD ON THE DAY]` placeholder** — comment only,
  values unchanged (the real-config test pins them) (commit `b9c0908`).
- **Compliant pad approach** (commit `3d0fffa`): the sim places designated pad 11 only **0.4 m from
  an arch post**, so `plan_path(goal=pad)` was inflation-blocked and the route fell back to a straight
  line that **overflew the raw post** — a compliance flag on Phase-1 ingress AND Phase-2 egress.
  `plan_path` now admits an endpoint inside the full inflated bubble but clear of a reduced `approach`
  set: its own incident segments use the reduced margin (a short, raw-clear final approach) while every
  interior segment stays on full inflation; an endpoint inside the raw obstacle is still refused. New
  `planner.pad_approach_inflate_m` knob (default 0.20 → real/legacy profiles load unchanged). TDD:
  ingress, egress, raw-blocked-still-none.

**Real-sim convoy integration (RUN_INTEGRATION=1 + a full `python -m mission.runtime.main` convoy run;
sim scorers are authoritative):**
- LandingScorer **3/3 in-hoop** (drones → pads 11/45/51, ≤3 cm sim error).
- **0 compliance flags** (sim's own compliance referee — was **2** `over_crate` flags at pad 11 before
  the planner fix; both gone after).
- **Score 5 — all 5 distinct rover ids banked** by the sim referee (11,45,51,67,101); the sim ended the
  ambush early at t=181 s (`phase DONE … score 5`).
- ⚠️ **Observation (not a re-sync regression, flagged for review):** the mission's OWN tally recorded
  only 2/5 distinct (`[51,67]`). The drones flew the captures (referee scored all 5) but the mission's
  `lock_and_tag` multi-frame hold is stricter than the referee's single-scan bank, so its self-belief
  lagged. This is a Phase-2 perception/lock-on tuning matter, orthogonal to the sim re-sync — left for
  a separate session.

## Real-hardware enablement (UWB cage) — sim path kept working, fake gate green (203 passed)
Made the mission flyable on real HULA hardware while leaving the sim path untouched (it still
loads `mission_config.yaml` + the sim `pyhulax`). TDD, one commit per layer:
- **`config/mission_real.yaml`** (loaded by `--real`; never edits the sim config): the 5 real pads
  (`{id:{x,y}}` → internal list), `designated_pads:[11,51,101]`, `uwb.origin_x|y` (per-cage origin),
  `discovery.use_dola:true`, **tag-only** `drones`, safety (≤0.5 m/s, 1.1 m, hoop 0.20 m, battery RTL,
  hold-on-dropout). `load_real_config()` translates it; schema extensions are backward-compatible
  (uwb origin defaults, optional drone ip/start, optional `discovery`). Open-cage `arena_real.yaml`.
- **UWB cage origin**: `UWBParserThread(x_origin, y_origin)` started from config; the fake applies it
  (origin 0 unchanged) so it's tested.
- **Discovery** `resolve_ordered()` — Dola-discover then map drones to the configured tag_ids **in
  order** (deck slide-6), logged; config fallback when Dola is stubbed/absent.
- **`--real` entrypoint** (`python -m mission.runtime.main --real`): Dola-ordered discover+connect,
  UWB cage origin, **starts read from UWB**, assign the 3 designated pads (nearest, no crossing), fly
  ≤0.5 m/s at ~1.1 m UWB-only landing, basic Phase-2; **finally lands every drone**, Ctrl-C →
  abort-and-land, per-drone status lines (connected / UWB / target pad / state). Config-driven arena path.
- **Staged bring-up scripts** (motion gated behind `--i-have-clear-space`, land-in-finally, public-API
  only): `connect_check.py` (read-only telemetry + live UWB; validates the cage origin & ip↔tag),
  `hover_test.py` (first arming), `camera_check.py` (live ArUco on real frames), `yaw_calibrate.py`
  (forward-nudge → set `frame.yaw_offset_deg`/`invert_*`, no code change). Each runs as
  `python scripts/<x>.py` (self-bootstraps `src` onto the path).
- **Landing robustness**: the success gate now averages independent UWB samples (a single 5 cm read
  flipped pass/fail even when centred); the report counts from `worker.landed_ok`.
- **Verified**: full fake suite 203 passed; `connect_check`/`--real` wiring tested on the fakes; the
  SIM run (no `--real`) still lands **3/3 SCORED** (sim LandingScorer, ~2–4 cm). The live `--real` path
  needs the real cage (the sim's `Dola` is stubbed + the real config has no IPs), so it's fake-verified
  here and climbs the `docs/SIM_VS_REAL.md` ladder on the day.

## Refinements (docs/REFINEMENTS.md) — one at a time, human review between each
### R1 — Phase-1 ArUco removal ✅ (committed `809bd79`)
Roster check: live sim `rovers.motion: convoy` (baseline). Phase 1 now lands **purely on UWB** — all
ArUco decode/confirm removed from `land_in_hoop` + the `LAND_HOOP` path (dropped `confirm_pad`/`stream`/
`pad_id`/`_decode_ids`/`confirm_pad_aruco`); the camera is **off in Phase 1** (stream created but
`set_video_stream(True)` deferred to Phase-2 start in `worker.run_phase2`). Phase-2 pad-id exclusion
(skip 10–14) **kept**. Fake gate: 174 passed, 1 skipped (new tests: Phase-1 lands with `cv2.aruco`
monkeypatched to raise → proves UWB-only; `video_enabled` False through Phase 1, True at Phase-2 start).
**Supervised sim verify (`/tmp/r1.mp4`, cycles=1): still 3/3 in-hoop (sim LandingScorer pads 12/11/10 @
1–2 cm SCORED); compliance 0 violations; no UWB/connect errors.** Stopped for review before R2.

## Integration session vs the LIVE sim (~/codes/finals/sim, in-process) — `python -m mission.runtime.main`
Ran the mission end-to-end against the real `pyhulax` + PyBullet sim (booted in-process by
`connect()`). **Live results (authoritative sim referees): Phase 1 = 3/3 landings SCORED in-hoop
(0.30 m), pads 12/11/10 @ 2–3 cm; Phase 2 = 4/5 convoy rovers tagged per single run; 0 compliance
violations; no UWB/connect issues.** Across 3 runs the *missing* rover varied (runs banked
{22,20,23}, {22,23,20,24}, {22,20,24,21}) so the **union is all 5** — the single-run 4/5 is a
coverage/timing limit of the general overwatch grid (the mission can't know the convoy routes), not
a bug. Recording at `/tmp/mission_run.mp4` (HULA_SIM_RECORD).
Reconciliation fixes made this session (TDD, fake gate stays green):
- **Discovery** is config-first (`config.drones` map) — the sim's `Dola` raises `NotImplementedError`;
  Dola is opt-in for the real day with a transparent fallback (`runtime/discovery.py`).
- **Fake speed** encodes the sim's raw VelocityLevel band (ZOOM 0.8/TURBO 1.0) but hard-clamps every
  level to 0.5 m/s; enum ints unchanged.
- **sdk_compat** now also guards `arm`/`disarm`; verified nothing in `src/` calls real-only methods unguarded.
- **Real bug the sim exposed:** mission read arena position from fake-only `drone.n/.e` (fatal on the real
  `DroneAPI`) — now reads **UWB** everywhere (`worker.py`, `phase2_search.py`), with a fake-only fallback.
- **fly_to_uwb** gained a position-dwell arrival fallback (the real 10 Hz + 5 cm UWB makes the instantaneous
  speed gate too noisy).
- **Phase-2 compliance fix:** vantage hops now route around inflated footprints (a straight hop cut over a
  crate → `over_crate` + ToF-altitude-hold climbed over the crate top → `altitude_cap` breach). After the fix:
  **0 violations.**
- **lock_and_tag** now banks on the referee's gate (marker ≥40 px AND fully in-frame, held 5 frames) instead
  of tight centering, so the mission's own belief matches the sim scorer.
Remaining gap: rover id 21 not always caught in a short run — a **coverage** limitation of the general
overwatch grid (the mission can't know the convoy routes), not a control/compliance bug. `@integration`
test (`RUN_INTEGRATION=1 PYTHONPATH=…/sim pytest -m integration`) drives a short real-sim episode and
asserts 3/3 landings via the sim's authoritative LandingScorer.

## Reference-source note (read once)
The authoritative `reference/pyhulax_knowledge_base.txt` and `reference/brief/Finals_brief.pdf`
are **NOT present** in this repo (only `reference/provided_code/` is vendored, and the `brief/`
+ `sim/` dirs hold just `.gitkeep`). SDK signatures were therefore verified against
`docs/SDK_REFERENCE.md` + `docs/RECONCILIATION.md` (which state they were reconciled against the
real sim and the KB) and the vendored `provided_code/` (`UWBParserThread.py`, `dola.py`,
`rover_detection_example.py`). No disagreements were found between those and what the docs assert.
**[TO CONFIRM]** when the KB/brief PDFs are dropped in: re-verify the enum integer values
(`Direction`, `CameraPitchMode`, `VelocityLevel`, `BarrierMask`) and the exact `connect`/`hover`
signatures — the fake encodes the doc's values.

## Log

### P11 — Full integration + reliability + sim handoff  ✅
Built `src/mission/runtime/discovery.py` (`Discovery`: `pyhulax.discovery.Dola` → fallback
`dola.py`, `plane_id`↔`tag_id` map, `fixed_ips` short-circuit for the sim — no network),
`src/mission/runtime/main.py` (`Mission`: spawns a `DroneWorker` per drone, runs Phase 1 → Phase 2
sequentially-deterministic or `parallel=True` threaded, **reassigns a dead drone's zone** via a
mop-up pass, and **lands every drone in `run()`'s `finally`**), and `docs/SIM_VS_REAL.md` (the
import-path swap + on-the-day calibration checklist). Added a **battery RTL failsafe** to
`DroneWorker` (`FailsafeAbort` raised from `_on_step` when `get_battery() ≤ threshold` → safe land,
`rtl=True`). Tests: a full Phase1→Phase2 run on the fake harness scores **3/3 landings + all 5
distinct tags, no double-count, every drone landed on shutdown**; battery RTL lands safely;
persistent UWB dropout holds without lurching; a sabotaged (dead) drone's zone is reassigned and
still fully tagged; `run()` lands all even when a phase raises; discovery resolves fixed IPs and via
the (fake) Dola. The real-sim end-to-end test is implemented + `@pytest.mark.integration` (skipped
in the gate; run supervised with `RUN_INTEGRATION=1`).

### P10 — C2 operator console  ✅
Built `src/mission/runtime/c2_bridge.py` (`C2Bridge`: `snapshot`/`to_json` of drones + footprints +
zones + rover tracks + coarsened belief heatmap + rubric scoreboard + alarms — all
JSON-serializable from the public world model only; `alarms` surfaces low-battery, UWB-dropout,
near-collision and the **over-footprint NO-FLY** violation; `export_evidence` writes one annotated
PNG per tagged id + a results.csv + a map.json bundle; `override` routes operator commands into the
Coordinator) and a single-file `c2/index.html` (live top-down map with belief heatmap, camera
tiles, scoreboard, alarm panel with the over-footprint highlight, pre-flight checklist, evidence
export + override buttons; polls `state.json` with an embedded-sample fallback). Added operator
override support to `Coordinator` (`force`/`hold_all`/`clear_all`, applied in `step`). Tests:
snapshot is valid JSON, the over-footprint alarm fires on a synthetic violating pose (and not on a
clean one), battery/UWB/near-collision alarms fire, scoreboard reflects state, the evidence bundle
is produced, and retask/hold-all/resume overrides reach the coordinator.

### P9 — Mission Planner GUI export contract  ✅
Built `src/mission/mission/plan.py` (pydantic `MissionPlan` loader, `extra="forbid"`; `validate_plan`
enforces the geometric rules — every route segment footprint-clear + in bounds, vantages inside
their footprint-clear bubble, bubbles pairwise disjoint with a buffer, pad_ids valid — raising
`PlanValidationError`; consumer helpers `pad_assignment`/`route`/`bubble`/`vantages`) and a single-
file `planner_gui/index.html` (canvas authoring of routes + vantages with live red-on-invalid
validation, inter-path conflict highlight, and `mission_plan.yaml`/PNG export matching the schema).
**Fixed the committed `examples/mission_plan.example.yaml`** — the scaffold's route 0 crossed the
inflated crate (invalid); re-authored all three drones with footprint-clear routes to their pads,
vantages inside disjoint east-band bubbles, and valid pad ids (PHASE_PLAN authorizes maintaining
this fixture). Tests: the example loads + validates + is consumed by P4 (pads valid, routes clear)
and P7 (vantages in bubbles, correct dict shape); unknown keys, a crate-crossing route, an out-of-
bubble vantage, an invalid pad (13), and overlapping bubbles each raise.

### P8 — Adversarial evader handling  ✅
Added evader logic to `phase2_search.py`: `classify_behaviour` triages a track's recent path into
smooth/periodic/erratic from turn-angle variance + loop-closure; `ReachableSet` models where an
evader can be on the free-space grid (`seed`/`expand`/`cut`/`size`) so holding a chokepoint
(`cut`) is **monotonically non-increasing** by construction (the connected component from the
anchor can only shrink as barriers are added); `plan_containment` picks the chokepoints bordering
the reachable set. Extended `Coordinator` with behaviour-aware tasking: **secure the predictable
autonomous tags first** (no evader commitment while untagged autonomous tracks remain), then a
pursuer TAGs the belief argmax while others BLOCK chokepoints, with **tick-based rotation** of the
block assignments to break standoffs. Tests cover triage, monotonic containment + room isolation,
the autonomous-before-evader ordering, pursuit/containment role assignment, and rotation. Full
teleop realism is an `integration` test (needs the sim's evasive mode) — excluded from the gate.

### P7 — Phase 2: search + lock-on + tag  ✅
Built `src/mission/mission/phase2_search.py`: `lock_and_tag` is a visual servo on the marker's
pixel offset (camera nadir, image axes == body axes at locked yaw), holds `hold_frames` consecutive
centred frames then banks the id — bounded by `lock_timeout_s` (no deadlock), `should_stop`-
interruptible, and it velocity-matches a slow mover. `vantage_patrol` cycles preplanned look-points
(fly→tilt→dwell-scan, not a lawnmower). `phase2_search` drives the cycle with **bubble-gating**
(engage only rovers whose projected xy is in the drone's zone — others are logged to the taskboard),
a **commitment rule** (a started lock runs to completion regardless of the boundary), and a
**mop-up** endgame (drop the gate in the last cycles). Extended `worker.py` with phase-2 states
(RELAUNCH/SEARCH/CONVERGE/HOME/DONE) and `run_phase2`. Tests: servo banks a stationary + a moving
rover, lock is time-boxed and interruptible, bubble-gate logs-but-doesn't-tag out-of-zone, mop-up
tags it, and **3 drones tag all 5 distinct ids with no double-count, no footprint overfly, ≤0.5 m/s
throughout** (rover marker xy via `projection.pixel_to_arena`, de-dup via shared `MissionState`).

### P6 — Shared world model  ✅
Built `world/mission_state.py` (lock-guarded tagged-id set + evidence, **de-dup by id**),
`world/taskboard.py` (`Track` + `Role`/`Assignment`; `see` merges by id and estimates velocity
from Δ), `world/belief_grid.py` (numpy occupancy over free cells: `observe` collapses + renorms,
`diffuse` spreads through free neighbours, `spike` injects a sighting, `argmax_region`, and
`chokepoints` extracts narrow gaps from the crate map), and `world/coordinator.py`
(`Coordinator.step` TAGs confirmed-untagged rovers with the nearest free drone, SWEEPs the rest
to the belief argmax, and leaves `busy_locked` drones alone — the P7 commitment hook). All shared
objects are `threading.Lock`-guarded. Tests cover de-dup, velocity estimation, belief
collapse/diffuse/spike, chokepoint extraction, and coordinator role logic incl. already-tagged
and locked-drone cases.

### P5 — Perception: ArUco + detector seam + two-stage  ✅
Built `src/mission/perception/detector.py` (`Detection` dataclass, `RoverDetector` ABC,
`PlaceholderRoverDetector` + PRIMARY `ClassicalRoverDetector` (contrast/contour + optional motion
gate), `ScanResult`, `two_stage_scan` — mirrors `reference/provided_code/rover_detection_example.py`)
and `src/mission/perception/aruco.py` (`confirm_with_aruco` multi-marker decode with a
`min_marker_px` gate, `is_pad_id`/`is_rover_id` where **pads=10–14 and ANY other id is a rover**,
`split_pads_rovers`). The import cycle (two_stage→aruco→Detection) is broken by a lazy import inside
`two_stage_scan`. Tests decode multiple + tilted markers off rendered frames, prove the px gate, the
classical finder boxes a rendered rover near frame-centre, and `two_stage_scan` runs find→approach→
confirm end-to-end returning the rover id.

### P4 — Phase 1: deploy + land in hoop  ✅
Built `src/mission/mission/phase1_land.py` (`assign_pads` brute-forces drone→pad permutations
minimising total route length with a heavy crossing penalty; `land_in_hoop` centres on UWB to the
hoop tolerance, gates the descent on footprint-clear column + clear `down` sensor, optionally
decode-confirms the pad ArUco, then descends only while centred and lands) and
`src/mission/mission/worker.py` (`DroneWorker` FSM INIT→TAKEOFF→GO_TO_PAD→LAND_HOOP→LANDED, plans
its route via the visibility graph, records a trace, lands on any exception so one drone never
freezes the others). Tests: assignment selects the correct 3 valid pads with no crossing; landing
refuses when the pad sits on a footprint; 3 drones each route + land within `hoop_tol_m=0.15` and
**no trace point ever enters a raw footprint**.

### P3 — Reactive avoidance guard + plan-then-guard  ✅
Built `src/mission/control/avoidance.py`: `ReactiveGuard.filter(cmd_fwd, cmd_right, obstacles, *,
open_side_hint=None) -> (fwd, right, up)` with `up` hard-wired to **0.0** (never climbs — HARD
invariant #3), stops the blocked travel axis, slides toward the open lateral side with
anti-oscillation **commit hysteresis**, and reports `boxed()` when surrounded. `separation(my_xy,
others, my_priority, sep_m)` implements right-of-way (lower priority yields). Wired into
`fly_to_uwb` via `guard=`, passing a goal-aware `open_side_hint` (sign of the body-right velocity)
so the slide naturally heads toward the goal. An exhaustive test sweeps all 2^5 obstacle combos ×
command grid asserting `up == 0`; the integration test flies a guarded drone past a *surprise*
crate (unknown to the planner) and proves it reaches the goal, never enters the footprint interior,
and never commands +up.

### P2 — Crate map + planner geometry + projection  ✅
Built `src/mission/planner/arena.py` (loads `config/arena_truth.yaml` with `yaml.safe_load`,
**no simcore** — HARD invariant #10), `src/mission/planner/geometry.py` (`inflate` →
axis-aligned `Rect`s; `segment_clear` via Liang-Barsky against slightly-shrunk rects so
boundary-tangent edges are allowed but interior crossings are rejected; `build_graph`
visibility graph over pushed-out footprint corners; `plan_path` A* with start/goal as temp
nodes; `paths_conflict` via segment-segment distance), and `src/mission/planner/projection.py`
(pinhole `arena_to_pixel`/`pixel_to_arena`, exact inverses, **no depth**). Tests prove paths
never cross an inflated footprint or leave bounds, `None` when goal is blocked / arena is walled
off, the real `arena_truth` routes cleanly, conflict detection flags crossing/too-close paths,
projection round-trips to 1e-6, and `pixel_to_arena` recovers a **rendered** marker's known arena
position to within 0.1 m (ties projection.py to the fake camera model — they share the pinhole).

### P1 — Config + frames + UWB control loop  ✅
Built `src/mission/config.py` (pydantic, every section `extra="forbid"` → unknown keys raise;
a validator enforces `speed.max_mps ∈ (0, 0.5]` as the HARD cap), `src/mission/frames.py`
(`arena_to_body`/`body_to_arena` as exact-inverse rotations, `m_to_cm`/`cm_to_m`, `clamp_speed`),
`src/mission/runtime/sdk_compat.py` (`prepare_manual_control`/`release` route real-only calls
through `hasattr` — no-op on the sim fake, real on `RealLikeFakeDroneAPI`), and
`src/mission/control/uwb_loop.py::fly_to_uwb` (P control on UWB arena error → body sticks, hard
clamp ≤0.5 m/s, ToF altitude hold, **hold-on-dropout**, UWB-derived arrival speed Δpos/Δt). The
loop is deterministic in tests via an injectable `sleep` (no real waits) and the time-stepped
fake. Converges from 5 start/target poses incl. corner-to-corner and under 3 cm UWB noise; a
test proves per-step speed never exceeds 0.5 m/s and another proves a horizontal freeze on UWB
dropout. Frame transforms are inverse-consistent and match hand-worked yaw-0 / yaw-90 cases.

### P0 — Test substrate (fake SDK)  ✅
Built `tests/fakes/fake_pyhulax.py`, `tests/fakes/fake_uwb.py`, `tests/conftest.py`,
`tests/test_fakes.py`. The fake mirrors the sim's **SUBSET** exactly: `connect(ip)` (required ip,
truthy `CommandResult`), `takeoff/land/hover(duration)/move/rotate/move_to/send_manual_control/
manual_fly/get_state/get_position/get_orientation/get_altitude/get_battery/get_obstacles/
any_obstacle/get_drone_status/set_barrier_mode/set_avoidance_direction/set_camera_angle/
create_video_stream/set_video_stream`. The real-SDK-only methods (`set_app_mode`,
`send_app_heartbeat`, `set_velocity_level`, `stop_manual_control`, `arm/disarm/disconnect`,
`get_velocity`, `get_drone_id`, `enable_battery_failsafe`) are **deliberately absent** — a
parametrized test asserts their absence, which is what will force the P1 `sdk_compat` shim.
`RealLikeFakeDroneAPI` adds them so the shim's hardware path is also testable. `Obstacles` has
no `up` field (asserted).

Kinematics are **time-stepped** (each `send_manual_control` advances a shared sim clock by
`world.dt=0.05` and integrates exactly that step) so tests are deterministic with no real sleeps;
the horizontal speed is hard-clamped to `max_mps=0.5` (resultant magnitude, asserted). UWB stamps
its samples with the same sim clock so a Δpos/Δt speed estimate (P1) is reproducible.

The camera renders **real** `cv2.aruco` markers via a correct pinhole projection +
`warpPerspective`. One subtlety hit and fixed: the floor-corner→canvas-corner correspondence
produced an orientation-reversing homography (a *mirrored* marker, which `cv2.aruco` refuses to
decode). Added a winding-match (`_signed_area`) that flips the destination quad when needed — now
nadir and offset frames genuinely decode to the right ids.

Integration tests are skipped unless `RUN_INTEGRATION=1` (documented mechanism, not weakening).
`conftest.py` also prepends `src/` to `sys.path` (the `mission` package is not pip-installed),
seeds `random`+`numpy`, and installs the fakes into `sys.modules` so `import pyhulax` /
`from UWBParserThread import UWBParserThread` resolve to them.
