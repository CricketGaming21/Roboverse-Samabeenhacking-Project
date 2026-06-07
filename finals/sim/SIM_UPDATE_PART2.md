# Hula Sim — Update: two-phase scenario, rover convoy, observability
## Build spec for Claude Code (continues HULA_SIM_BUILD_PLAN.md; Phases 9–15)

> This adds the **part-2 world** and a **canned demo** of the full scenario to the existing sim.
> Same rules as the original plan: work **phase by phase**, make each phase's test pass headless,
> commit (§8 of the build plan), then **STOP and report**. The existing 83 tests must stay green,
> the public `pyhulax` / `UWBParserThread` surface stays frozen, DIRECT+EGL stays the default,
> all PyBullet calls stay on the sim thread, config adds go in BOTH `sim_config.yaml` AND the
> `simcore/config.py` dataclasses (the loader rejects unknown keys, so YAML-only additions fail).

---

## 0. The SIM / MISSION boundary (read first — this is the whole point)

Everything here builds the **world and a demo**, never the mission. Concretely:

**This update BUILDS (sim):**
- A two-phase episode (DEPLOY → AMBUSH) that controls **when rovers enter and move**.
- Rover **convoy** motion: enter from one entrance, run a shared trunk, split to branches, loiter.
- **Crate-cluster** obstacles that look like the real arena.
- A longer, configurable episode so it doesn't end on part 1.
- **Two-part scoring**: a landing-accuracy referee (part 1) + the existing snapshot referee (part 2).
- **Views**: per-drone camera windows, a camera-FOV footprint that moves as the camera tilts, and a
  car-sensor-style obstacle-proximity display.
- A **canned demo harness** that launches → lands (part 1) → takes off → scans the convoy (part 2),
  so you can watch the whole thing.

**This update does NOT build (mission — the other chat):**
- `phase1_land` / `phase2_search` real logic, search/coverage strategy, target lock-on.
- The lock-on that tilts the camera is **mission** code; this update only **visualizes** the tilt.
- The canned demo's flight path is a **dumb scripted path to exercise the world**, not a strategy.

> A test that gives the rovers a "smart evade" or the demo drones a "real search" has crossed the
> line. Rovers follow fixed configured routes; the demo follows a fixed scripted path.

**How the ambush starts (important):** the rover convoy entry is triggered by the **scenario**, not
by the mission — because on the real day the organisers launch the convoy, the drones don't summon
it. So the trigger is scenario-controlled (config: auto when all drones have landed, or after a
timer, or an operator keypress in `--live`). **Mission code must never call a sim API to start the
ambush** — that wouldn't exist on hardware. The drones just fly; the rovers appear on the scenario's
own schedule.

---

## 1. What each of your requests maps to

| Your ask | Phase |
|---|---|
| Rovers convoy in from one entrance, then split to routes (not random back-and-forth) | 10 |
| Rovers shouldn't move during part 1; sim ends too fast / want it longer | 9 |
| Obstacles like the screenshots (crate clusters) | 11 |
| Two-part scoring (land accurately on pads vs snapshot rovers) | 12 |
| Camera view + a marker showing camera FOV, FOV changes during lock-on | 13 |
| Car-sensor obstacle-avoidance view (direction + proximity near a wall) | 14 |
| Split the sim into part 1 (land) and part 2 (rovers move) | 9 + 15 |
| Drones take off from pads to await/detect the convoy (the big missing piece) | 9 (world) + 15 (demo); real logic = mission |

---

## 2. Arena facts (from the brief + images — use these, don't re-interpret the pictures)

- The arena is one rectangular room. **One entrance on a single side** (left/bottom-left in the
  images). Put it in config (`scenario.entrance`).
- **Part 1:** 3 HULA launch from the C2 side (near the entrance) and **fan out** to designated
  landing pads scattered across the room. Several pads exist; **3 are designated/valid** targets.
- **Part 2:** the 5-rover convoy **enters from the entrance**, follows a shared route then **splits**
  into separate winding routes, spreads across the arena, and **loiters**. Drones (already landed)
  take off and scan them.
- **Obstacles** look like **clustered crates/boxes of varying height** (some single, some stacked,
  grouped), scattered through the room, with an **archway/tunnel** in one corner.
- **Layout has two modes (config `arena.layout`):**
  - **`authored` (new DEFAULT for watching/demoing):** a fixed layout that matches the reference
    images — pads (valid + invalid) and crate clusters at the approximate positions in Phase 11.
  - **`procedural` (retained):** the existing seeded random generator in crate-cluster style — used
    for robustness testing, because the real competition map is unknown and will NOT match this
    illustration. **Do not delete it.** A mission tuned only against the authored map would overfit.
- The authored coordinates are **approximate** (read off a low-res top-down) — Mark adjusts them.
- The reference images + `RoboVerse_Finals.pdf` are in `reference/` for context; the actionable
  numbers all live in `sim_config.yaml`.

---

## 3. Config additions (add to sim_config.yaml AND simcore/config.py dataclasses)

Sketch — Claude Code finalises names to match the existing config style:

```yaml
scenario:
  phases: both              # deploy | ambush | both
  episode_seconds: 180      # total run length (was effectively ~part-1-only/short)
  deploy_timeout_s: 90      # max time allowed for part 1
  ambush_seconds: 120       # how long the convoy phase runs
  entrance: [0.5, 0.5]      # arena-frame (north,east) where drones launch / convoy enters
  ambush_trigger:
    mode: on_all_landed     # on_all_landed | timed | manual_key
    delay_s: 3.0            # extra delay after the trigger before the convoy enters

rovers:
  # convoy REPLACES waypoint_random patrol when scenario.phases includes ambush
  convoy:
    entry_stagger_s: 2.0    # gap between successive rovers entering
    speed_mps: 0.4
    trunk: [[1.0,0.8],[2.5,1.5],[4.0,2.0]]     # shared path from the entrance (north,east)
    split_index: 2          # after this trunk waypoint, each rover takes its own branch
    branches:               # one per rover (5), each a waypoint list after the split
      - [[5.0,1.0],[7.0,1.0]]
      - [[5.0,3.0],[8.0,4.0]]
      - [[6.0,5.0],[8.5,5.0]]
      - [[4.0,4.0],[3.0,5.0]]
      - [[5.5,2.5],[8.0,2.0]]
    loiter: loop            # loop the branch | hold at end | small random within sub-area

arena:
  layout: authored          # authored (matches the images, DEFAULT) | procedural (seeded random)
  # --- authored layout (used when layout: authored) — APPROXIMATE, adjust to taste ---
  # arena is 10 (north) x 6 (east); north = image bottom->top, east = image left->right
  authored:
    clusters:               # crate clusters at image positions (north,east); boxes of varying height
      - { center: [5.0, 3.0], boxes: 5, height_m: [0.4, 1.2] }   # central L/plus of crates
      - { center: [7.5, 4.2], boxes: 2, height_m: [0.6, 1.0] }   # upper-right pair
      - { center: [5.0, 5.2], boxes: 2, height_m: [0.4, 0.9] }   # right edge
      - { center: [3.0, 3.5], boxes: 2, height_m: [0.5, 1.1] }   # bottom-center
    archway: { corner: [9.0, 5.3] }     # archway/tunnel in the top-right corner
  # --- procedural layout (used when layout: procedural) — crate-cluster style, seeded ---
  obstacles:
    style: clusters         # clusters (new look) | scatter (old behaviour)
    cluster_count: 4
    boxes_per_cluster: [2, 5]
    box_height_m: [0.3, 2.0]
    cluster_spread_m: 0.6
    archway: true           # add one archway/tunnel feature in a corner

# Pads now carry valid/designated flags and match the image layout (~5 pads, 3 designated "Land").
# A drone scores part 1 only by landing on a VALID + DESIGNATED pad (see Phase 12).
pads:
  - { id: 10, north: 8.5, east: 3.0, valid: true,  designated: true }   # top "Land"
  - { id: 11, north: 5.5, east: 4.8, valid: true,  designated: true }   # right "Land"
  - { id: 12, north: 2.0, east: 4.5, valid: true,  designated: true }   # bottom-right "Land"
  - { id: 13, north: 5.5, east: 1.2, valid: false, designated: false }  # left-mid INVALID (red marker)
  - { id: 14, north: 2.0, east: 1.5, valid: true,  designated: false }  # bottom-left valid, not chosen

scoring:
  landing:                  # part-1 referee
    tolerance_m: 0.30       # within this of a designated pad center = successful landing
    assignment: nearest_unclaimed   # nearest_unclaimed | fixed (drones.units[].pad_id)
    time_weighted: true

viz:
  show_camera_fov: true     # draw each drone's camera footprint/cone (top-down + 3D)
  show_proximity: true      # car-sensor obstacle display per drone
  # show_camera_windows already exists
```

---

## 4. Phase plan

Each phase: **Deliverable → Files → Acceptance test (headless) → Manual check → Commit → STOP.**

### Phase 9 — Scenario controller + two-phase episode + longer run
- **Deliverable:** `simcore/scenario.py` — an episode state machine `DEPLOY → AMBUSH → DONE`.
  In DEPLOY: pads active, **rovers are NOT in the arena** (parked off-map at the entrance, inert,
  not rendered as moving targets). Transition to AMBUSH on `scenario.ambush_trigger` (default
  `on_all_landed` — detected from ground-truth drone landedness — plus `delay_s`); also support
  `timed` and `manual_key` (a keypress in `--live`). In AMBUSH: rovers enter and move (Phase 10
  gives them routes; for Phase 9 a placeholder "rovers active" flag is fine). DONE at
  `episode_seconds` or when all rovers scanned. Episode length driven by config (default 180 s),
  replacing the short run. The scenario **never flies drones** and exposes **no API the mission
  calls** to start the ambush.
- **Files:** `scenario.py`, `registry.py` (own/step the scenario on the sim thread), `rover_model.py`
  (gate motion/visibility on phase), `config.py` (+scenario block), `run_sim.py`/`smoke_test.py` (phase-aware).
- **Acceptance test:** in DEPLOY rovers are absent/inert (no motion, not counted as targets); the
  trigger advances to AMBUSH (test all three modes — landing-based via simulated landings, timed,
  and manual); rovers become active only in AMBUSH; the episode runs the configured duration and
  ends at DONE; existing tests still green.
- **Manual check:** `python -m scripts.run_sim --seconds 20` logs the phase transitions
  (DEPLOY→AMBUSH→…) and runs the full configured length without ending on part 1.
- **Commit:** `feat: two-phase scenario controller (deploy/ambush) with configurable episode length`.

### Phase 10 — Rover convoy routing
- **Deliverable:** replace `waypoint_random` patrol with **convoy** behaviour from `rovers.convoy`:
  all 5 rovers **enter from the entrance staggered** by `entry_stagger_s`, follow the shared `trunk`
  to `split_index`, then each follows its own `branch`, then **loiters** per `loiter`. Speed from
  config; stay on route; advanced on the sim thread; only active in AMBUSH. Keep `style: scatter`/
  random patrol available behind a flag for back-compat.
- **Files:** `rover_model.py`, `config.py` (+convoy block), maybe a small `routes.py` helper.
- **Acceptance test:** rovers enter from `entrance` (not teleport mid-arena), traverse the trunk in
  order then diverge onto distinct branches, remain on/near their route within tolerance, enter
  staggered (rover k starts ~`k*entry_stagger_s` after AMBUSH begins), and loiter at the end; all 5
  stay within arena bounds; determinism holds for a fixed seed.
- **Manual check:** `--dump run.jsonl` over an ambush run; offline-confirm each rover's track follows
  trunk-then-branch and they entered staggered.
- **Commit:** `feat: rover convoy routing (staggered entry, shared trunk, per-rover branches, loiter)`.

### Phase 11 — Authored map layout (matches the images) + crate-cluster obstacles
- **Deliverable:** `arena.py` gains a **layout switch** (`arena.layout`):
  - **`authored` (DEFAULT):** build the **fixed layout from `arena.authored` + `pads`** so the scene
    matches the reference images — crate clusters at the configured `clusters` anchors (each a group
    of `boxes` of varying `height_m` within a small spread), the `archway` in the configured corner,
    and the ~5 pads at their configured positions. Coordinates in `sim_config.yaml` are **approximate
    starting points** (10 north × 6 east; north = image bottom→top, east = left→right) — easy to nudge.
  - **`procedural` (RETAINED — do not remove):** the existing seeded rejection sampler, now in
    crate-cluster style (`cluster_count` clusters of `boxes_per_cluster`, varying `box_height_m`,
    within `cluster_spread_m`), honouring `min_clearance`/`keepclear_radius`/wall margins and staying
    clear of pads, drone starts, and the convoy route. This is the **unknown-map** mode for robustness.
  - Both build via the same `world.py` body-creation path; the authored archway is a static arch/
    tunnel body the drones pass under or around.
- **Files:** `arena.py` (layout switch + authored builder + cluster sampler), `world.py` (clusters +
  archway), `config.py` (+`arena.layout`, +`arena.authored`, +expanded `pads` with valid/designated).
- **Acceptance test:** with `layout: authored`, obstacles/pads land at the configured positions
  (deterministic, no RNG), the archway is present, and clearances around pads/starts still hold; with
  `layout: procedural`, clusters generate with configured counts/heights, all clearances + the route
  corridor are respected, and same-seed ⇒ identical / different-seed ⇒ different; body-count math
  checks out in both; pads expose `valid`/`designated` flags. Existing tests still green.
- **Manual check:** `python -m scripts.run_sim --seconds 2 --topdown out.png` (or `--gui`) with
  `layout: authored` shows a scene resembling the Phase-1 image (clusters + archway + scattered pads);
  flipping to `procedural` shows a different seeded crate-cluster arena.
- **Commit:** `feat: authored map layout matching reference images (default) + procedural crate-cluster fallback`.

### Phase 12 — Two-part scoring
- **Deliverable:** split scoring to match the two stages.
  - **Part-1 landing referee** (new): when a drone's `land()` completes in DEPLOY, measure horizontal
    distance from its touchdown point to the nearest pad center (arena frame). It scores **only if that
    pad is `valid: true` AND `designated: true`** and within `scoring.landing.tolerance_m` (dedup: one
    drone per pad, `assignment` mode). Landing on an **invalid** pad (e.g. id 13) or a non-designated
    pad scores nothing — that models choosing the wrong zone. Record per-drone: pad id, valid?,
    distance error (cm), time-to-land. Time-weighted score. (Deciding validity *from the marker* is
    mission work; the sim just exposes the flag for scoring.)
  - **Part-2 snapshot referee** (existing): keep the distinct-ArUco-id gate, but it now scores **rover**
    markers during AMBUSH (pads are no longer the part-2 target). Time-weighted.
  - Combined scoreboard prints both: part-1 landings (pad, error, time) and part-2 snapshots (distinct
    rover ids, time). Keep `scoring.mode` (auto) and the internal `register_capture` hook.
- **Files:** `scoring.py` (or add `landing_scorer.py`), `scenario.py` (tell the scorer the phase),
  `config.py` (+scoring.landing).
- **Acceptance test:** a drone landing within tolerance of a designated pad scores part 1 with the
  right distance/time; landing off-pad (outside tolerance, or on an invalid pad) does **not** score;
  two drones can't both claim one pad; part-2 rover snapshots score exactly as the existing gate
  (size/in-frame/hold, deduped across drones) and pads do **not** count as part-2 targets.
- **Manual check:** the demo (Phase 15) ends with a combined scoreboard: 3 landings with errors + N
  distinct rover ids.
- **Commit:** `feat: two-part scoring — landing-accuracy referee (part 1) + rover-snapshot referee (part 2)`.

### Phase 13 — Camera-FOV view + live camera windows
- **Deliverable:** show what each camera sees and where it points.
  - Per-drone **camera footprint**: project the camera frustum (from drone pose + current
    `set_camera_angle` pitch + the fixed FOV from `config.camera`) onto the ground as a polygon, drawn
    on the top-down view; draw the frustum in `--gui` 3D. It **moves and tilts live** as the drone
    moves and as pitch changes — so when the mission (or the demo) tilts the camera during lock-on,
    the footprint visibly swings. Toggle via `viz.show_camera_fov`.
  - Ensure `viz.show_camera_windows: true` opens the per-drone live frames during the scenario run.
  - **NOTE on "FOV":** the Hula changes camera **pitch**, not zoom — the footprint reflects pitch/
    pointing, and its size is fixed by the real lens FOV. (If true optical zoom is wanted, that's not
    a Hula capability — flag it, don't fake a shrinking footprint.)
- **Files:** `viz.py`, `frames.py` (footprint projection — one place), `camera.py` (expose current
  pitch/intrinsics to viz), `config.py` (+viz.show_camera_fov).
- **Acceptance test:** the footprint polygon matches the camera pose+pitch geometry (assert corners
  against a known pose); changing `set_camera_angle` changes the footprint (forward vs down differ);
  rendering stays on the sim thread; headless PNG still works with the overlay.
- **Manual check:** `--gui` (or `--live`) with a scripted pitch change shows the footprint swinging;
  camera windows show the live frames.
- **Commit:** `feat: camera-FOV footprint overlay (top-down + 3D) and live camera windows`.

### Phase 14 — Obstacle-proximity (car-sensor) view
- **Deliverable:** a per-drone **parking-sensor-style** display: around each drone icon, segments for
  the five barrier directions (forward/back/left/right + a down indicator) that light up by
  **proximity** — green (clear) → amber → red (close). Use the **actual ray distance** to the hit
  (from the existing barrier ray-cast / DebugProbe), so it shows direction **and** proximity, with a
  short label (e.g. `FWD 0.3 m`). Drawn on the top-down (and a compact HUD in `--gui` if practical).
  Toggle via `viz.show_proximity`.
  - **Critical:** this proximity is **observer-only ground truth** (the sim knows the ray distance).
    The **mission still only receives the blocked/clear booleans** from `get_obstacles()` — do NOT
    add distance to the pyhulax surface. The view reads `simcore`/DebugProbe, never `pyhulax`.
- **Files:** `viz.py`, reuse `sensors.barrier_rays` / `debug.py` for the distances, `config.py`
  (+viz.show_proximity).
- **Acceptance test:** with an obstacle at a known distance ahead, the forward segment shows the
  correct band/colour and distance; clear directions read green; the indicator updates as the drone
  approaches; assert the proximity comes from ray data and that `get_obstacles()` still returns only
  booleans (no distance leaked to the public API).
- **Manual check:** `--live`/`--gui` flying a drone toward a wall — the forward sensor goes
  amber→red with the closing distance, like a reversing camera.
- **Commit:** `feat: car-sensor obstacle-proximity overlay (observer-only; API still booleans)`.

### Phase 15 — Two-phase demo harness (watch the whole scenario)
- **Deliverable:** a scenario demo (extend `smoke_test.py` or add `scripts/scenario_demo.py`) that
  runs the **full canned scenario** end to end so you can watch it:
  1. **Part 1:** 3 drones launch from the entrance/C2 side and fly **canned** paths to 3 designated
     pads and land (part-1 scorer records accuracy + time).
  2. Scenario advances to **AMBUSH** (per the trigger); the **convoy enters** and runs its routes.
  3. **Part 2:** the 3 drones take off and fly a **canned** search/observation path (tilting the
     camera as a scripted lock-on stand-in) while the convoy moves; the part-2 scorer banks distinct
     rover ids.
  4. Prints the **combined scoreboard** (landings + snapshots) + the thrash report.
  Runs the full `episode_seconds`. Works headless; `--gui`/`--live`/`--debug`/`--dump` all apply.
  The flight is **scripted to exercise the world**, explicitly **not** a search strategy.
- **Files:** `scripts/scenario_demo.py` (or `smoke_test.py`), reuse scenario/scoring/viz; no new sim
  capability — this just drives the pieces.
- **Acceptance test:** the demo runs both phases headless, ends with ≥1 successful landing scored in
  part 1 AND ≥1 distinct rover id scored in part 2, rovers only moved during AMBUSH, and the run
  lasted the configured duration. No search/strategy logic — assert the path is a fixed script.
- **Manual check:** `python -m scripts.scenario_demo --gui` (real-time): watch launch → land → convoy
  enters → drones take off and scan → combined scoreboard.
- **Commit:** `feat: full two-phase scenario demo (deploy + ambush) with combined scoreboard`.

---

## 5. Confirm before / during (mostly config — not code)

- **"Camera FOV during lock-on" = pitch, not zoom.** Confirm you mean the camera *tilts* to track the
  target (supported, visualized in Phase 13). True optical zoom isn't a Hula feature; say so if you
  expected a zoom and we'll decide whether a non-physical debug zoom is worth it.
- **Ambush trigger:** default is "start the convoy once all 3 drones have landed (+delay)." Confirm
  that fits how you imagine running it, or pick `timed`/`manual_key`. (It must not be mission-called.)
- **One continuous episode** (land → ambush → scan) is the default. If the real event resets between
  the two stages, we can also run `scenario.phases: deploy` and `: ambush` separately.
- **Convoy routes + entrance + pad coords** are placeholders in `sim_config.yaml`; drop in the real
  geometry when known — config edit, no code change.
- **Landing tolerance** (`scoring.landing.tolerance_m`, default 0.30 m) and **drone→pad assignment**
  (nearest-unclaimed vs fixed) — adjust to the real scoring once known.

---

## 6. Reference materials to place in `finals/sim/reference/`

Drop these so Claude Code can consult them (the spec already carries the actionable facts as text, so
this is for context/curiosity, not a dependency):
- `RoboVerse_Finals.pdf` — the updated brief (authoritative scenario + scoring).
- `Phase_1_task.png` — landing layout: drones fan from the entrance to scattered pads; crate clusters; archway.
- `Phase_2_task.png` — convoy: enters from one side, shared route then splits into winding branches.
- `real_life_3d_image_of_maze.PNG` — obstacle look (box/crate clusters in a truss-frame room).
- `context.png` — scenario/objective text.

(The existing `pyhulax_complete_knowledge_base.txt`, `UWBParserThread.py`, `dola.py`, `huladola.py`
stay where they are.)
