# Hula Sim — Update to the REAL RoboVerse Finals brief
## Build spec for Claude Code (continues SIM_UPDATE_PART6.md; Phases 31–34)

> Driven by the official **Finals_brief.pdf**. Same invariants, non-negotiable:
> - **Do NOT change the public pyhulax / UWBParserThread API surface or signatures** — mission code must
>   still transfer by import-path swap. All changes are sim/observer/scenario-side.
> - Keep **determinism** via `meta.seed`; keep **all tests green** (add tests for new behaviour);
>   **phase-by-phase, commit per change.**
> - Keep `--record` (offscreen) and `live_view` as the safe WSLg paths — **no `--gui` / `--dashboard`
>   freeze regressions.**
> - **Don't touch mission code.**

### Facts the brief established (apply throughout)
- HULA max speed **0.5 m/s** (hard). Recommended flight height **1.1 m**; crates ~**1.2 m**; **no flying
  over obstacles → score invalidated.** ~**8 min** per stage; **no re-assessment on crash**.
- Stage-1 (deploy): land 3 drones on **provided** valid coordinates (3 of 5), scored **landings-in-hoop +
  time**. Stage-2 (ambush): **5 RoboMaster rovers**, **2 human-teleoperated by the opponent team**, 3
  autonomous; scored **distinct ArUco IDs + time**. (Pre-U: 44% + 44% + bonus.)
- **OPEN — confirm before relying on:** drone count (brief logistics say **1** HULA for Pre-U but Challenge
  2 says launch **3** — make drone count a CONFIG KNOB so both work); ArUco dictionary (assume
  `DICT_6X6_250`, confirm); crate footprints / landing coords / cage dimensions / rover speed / hoop radius
  (**all come from Discord — not in the brief; parameterise, fill in later, estimate meanwhile**).

---

## Phase 31 — Real arena (fully config-driven) + 0.5 m/s hard cap
- **Arena → config-driven, ready for the Discord numbers.** In `sim_config.yaml` `arena.authored`, make
  dimensions, crate clusters, and the archway **entirely data-driven**: `length_m`, `width_m`,
  `authored.clusters` (each crate `center [x,y]`, `size [w,d]`, `height`), `archway`. Drop in the real
  Discord coordinates when available; **meanwhile use a best estimate from the brief images** (do NOT
  hard-bake guesses as if confirmed — keep them clearly marked as provisional defaults). Keep
  `layout: authored` default. Validate authored coords (a malformed edit raises, as today).
- **Expose the arena as plain data for the mission side.** Real teams are *given* crate coordinates, so
  emit the authored arena to a documented **plain-data file** (e.g. `crates.yaml` / `arena_truth.yaml`) the
  mission can load **without importing `simcore`** (it's external ground-truth data, like the Discord file
  — NOT a sim import). Keep the boundary: the mission reads a data file, not sim internals.
- **0.5 m/s hard cap.** Enforce the HULA cap in the `VelocityLevel`→m/s mapping: add
  `motion.velocity_levels.max_mps: 0.5` and **clamp every level so none exceeds 0.5** (MEDIUM = 0.5 is the
  usable max; ZOOM/TURBO clamp to 0.5). **Leave the `VelocityLevel` enum names/values and the public API
  unchanged** — only the mapped speed clamps.
- **Tests:** arena loads from config and validates; the emitted `crates.yaml` matches `arena.authored`; **no
  velocity level maps above 0.5 m/s** (assert the mapping); existing green; **API surface unchanged.**
- **Commit:** `feat: real arena (config-driven, mission-readable) + 0.5 m/s hard speed cap`.

### Phase 32 — Mixed convoy: 3 autonomous (smooth) + 2 evasive (adversarial) + teleop
- **`rovers.motion: mixed`.** Spawn **3 autonomous** rovers (routes/patrol, **keep the Phase-28 smooth
  rate-limited turning**), marker ids e.g. `[20,21,22]`; and **2 evasive** rovers with a **separate marker
  block** e.g. `[30,31]` (mirrors the opponent-controlled rovers carrying their own ids).
- **Evasive behaviour model (the sim's stand-in for the human opponent):** each evasive rover, when the
  nearest drone is within `flee_radius_m`, **flees** (gain `flee_gain`), **prefers paths that keep a crate
  between itself and the pursuer** (`cover_bias`, uses the arena footprints), and injects **randomized
  jukes / heading changes** (`juke_prob`). All tunable. This is intentionally *un*-smooth — distinct from
  the Phase-28 autonomous rovers, which stay smooth.
- **Teleop hook (test against a real adversary).** Add an **SSH-friendly** way to human-drive **one**
  evasive rover live: **terminal stdin keys in the SSH session, or via the web dashboard** — **never a GUI
  window** (headless-safe; no WSLg freeze). Keyboard maps to the rover's drive (e.g. WASD + stop).
- **Tests:** mixed mode spawns 3 auto + 2 evasive with the two id blocks; an evasive rover **flees a nearby
  drone** (assert separation grows / heading reacts within `flee_radius_m`); `cover_bias` prefers
  crate-shadowed routes (seeded/qualitative); the **teleop hook accepts input headlessly without opening a
  window**; the **autonomous rovers still pass the Phase-28 smoothness assertions**; existing green; **API
  unchanged.**
- **Commit:** `feat: mixed convoy — 3 autonomous + 2 evasive (cover-seeking, juking) rovers + SSH-safe teleop`.

### Phase 33 — No-fly-over-obstacle + altitude-cap compliance flagging
- **The score-invalidation rule, made visible in-sim.** Add a compliance checker that **FLAGS** (logs +
  surfaces in `--record` and `--dashboard`) whenever an **airborne** drone's **horizontal position is over a
  crate footprint**, or its **altitude exceeds a cap**. It's a **scored/flagged event, NOT a crash** — so
  you can SEE violations and fix your nav, not lose the run silently. Config:
  `compliance.max_altitude_m`, footprints from `arena.authored`, optional small margin.
- **Surface it.** A visible per-drone **VIOLATION** indicator in the cockpit MP4 + web dashboard, and a
  `logs/compliance` record. Put the flag in the `DebugProbe` snapshot so both renderers can show it.
- **Tests:** an airborne drone positioned over a crate footprint **raises a flagged event (not a crash)**;
  exceeding `max_altitude_m` flags; flying at ~1.1 m over clear floor **does not** flag; the flag appears in
  the debug snapshot (so dashboard/record can render it); existing green; **API unchanged.**
- **Commit:** `feat: no-fly-over-obstacle + altitude-cap compliance flagging (logged + surfaced, not a crash)`.

### Phase 34 — Inter-drone obstacle sensing + hoop / distinct-ID scoring + 8-min clock
- **Drones are obstacles to each other.** Make the obstacle query include the **other drones**, so
  `get_obstacles()` (fwd/back/left/right/down booleans) **trips on a nearby drone** at the same realistic IR
  range as crates. Enables testing **3-drone separation at a common ~1.1 m height** (no altitude layering).
  **API unchanged** — the world simply now contains the other drones as obstacle sources.
- **Scoring → match the brief.** Stage-1 **"inside the hoop"**: parameterise `scoring.landing.hoop_radius_m`
  (value **TBD — config knob**); a landing scores **iff** within the hoop of a chosen valid pad. Stage-2:
  **distinct ArUco ids across ALL 5 rovers** (both the `[20,21,22]` and `[30,31]` blocks count).
- **8-min stage clock.** Set the stage/episode budget to reflect the real ~**8 min** per stage
  (`scenario.deploy_timeout` / `episode_seconds` as config knobs).
- **Drone count knob (the 1-vs-3 hedge).** Ensure the number of drones is a clean config value so the sim
  runs the scenario with **1 or 3** drones depending on what the organisers confirm.
- **Tests:** a second drone within IR range **trips the correct barrier flag** on the first (and clears when
  apart); **hoop scoring** (inside → scores, outside → doesn't) at the configured radius; **distinct-id
  scoring** counts both blocks (5 distinct); the episode honours the **8-min** cap; the scenario runs with
  **drones = 1** and **drones = 3**; existing green; **API unchanged.**
- **Commit:** `feat: inter-drone obstacle sensing + hoop/distinct-id scoring + 8-min stage clock + drone-count knob`.

---

## Notes
- **No API drift.** Every change here is arena/scenario/observer/scoring-side. If a change seems to need a
  new pyhulax/UWB method or signature, stop — it doesn't.
- **Provisional vs confirmed.** Crate/landing coords, cage size, rover speed, hoop radius, drone count, and
  the ArUco dict are **provisional** until Discord/organisers confirm. Keep them as clearly-labelled config
  defaults; never present an estimate as ground truth.
- **Two rover personalities, on purpose.** Autonomous = smooth (Phase-28); evasive = adversarial/juking
  (Phase-32). Don't let the Phase-32 evasive work regress the Phase-28 smoothness of the autonomous three.
- **Safe paths only.** `--record` + `live_view` stay the WSLg-safe options; the web `--dashboard` stays
  offscreen. No regressions to the freeze-proofing.
