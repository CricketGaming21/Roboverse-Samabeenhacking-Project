# CLAUDE.md — read this first, every session

You are building the **MISSION CODE** for the BrainHack 2026 **RoboVerse Drone Challenge,
Challenge 2 (Pre-University)**: 3 Highgreat **HULA** drones, two phases —
`phase1_land` (deploy + land in hoops) and `phase2_search` (ambush: tag the rover convoy).

The code runs **only against the public `pyhulax` + `UWBParserThread` API** (plus
`cv2`/`numpy`/stdlib). It is developed against a faithful simulator and deploys on the real
drones **by swapping the import path** — so the public-API boundary is sacred.

This repo is **separate from the simulator repo**. For the unattended test run you do **not**
need the sim or hardware: tests run against a fake SDK substrate (see `tests/README.md`).

---

## HARD INVARIANTS — never violate. Enforce in code AND cover with a test.
A regression here is worse than an unfinished feature. These come from the official brief
(`reference/brief/Finals_brief.pdf`) and the SDK (`reference/pyhulax_knowledge_base.txt`).

1. **Public API only.** Import `pyhulax` and `UWBParserThread` (+ cv2/numpy/std). **NEVER import
   `simcore`.** **NEVER trigger the ambush** (no such public call exists; the organisers start it).
2. **Speed ≤ 0.5 m/s** commanded, everywhere. Hard rule. Clamp at the controller.
3. **NEVER fly over an obstacle footprint, at any height.** Cruise ≈ **1.1 m**. The reactive
   avoidance layer must **never command +up to clear an obstacle** — flying over = scores voided.
   Routes are planned in 2-D around inflated footprints; the guard resolves **laterally only**.
4. **Units: UWB is METRES, pyhulax is CENTIMETRES.** Mixing them is a 100× bug that flies a drone
   into a wall. Convert at well-named boundaries; test the conversion.
5. **Distinct-ID tagging.** De-dup by ArUco id (post-read). **Opponent rovers carry DIFFERENT ids
   from the autonomous ones** — do **NOT** hard-filter to 20–24. **Pads = ids 10–14; any other
   decoded marker is a rover.**
6. **Crash = no re-assessment.** Conservatism wins: generous clearances, crawl near obstacles,
   battery/UWB/worker failsafes, and **shutdown always lands every drone** (in a `finally`).
7. **Determinism.** Seed everything. **No network in unit tests.** No flaky tests.
8. **Don't trust onboard position for arena accuracy.** No QR mat → `get_position()`/`move_to`/
   `create_flight_controller` use the drifting onboard estimate. Arena truth = UWB
   (`UWBParserThread`). We fly our own UWB loop (`fly_to_uwb`).

9. **The sim implements a SUBSET of the SDK.** Call only the sim surface directly; wrap real-SDK-only
   init (`set_app_mode`, `send_app_heartbeat`, `set_velocity_level`, `stop_manual_control`, `arm/disarm`,
   `disconnect`, `enable_battery_failsafe`) behind `hasattr` guards in `runtime/sdk_compat.py` so ONE
   codebase runs on sim AND real. `connect(ip)` needs an ip; `hover(duration_seconds)` needs a duration
   (station-keep with `send_manual_control()` zeros); `Obstacles` has **no `up`**; there is **no
   `get_velocity()`** (estimate speed from UWB Δ). Details + the full diff: `docs/RECONCILIATION.md`.
10. **Planner input = `config/arena_truth.yaml`** (the sim emits it via `emit_arena_truth`; on the day the
   Discord coords play the same role). Load it with `yaml.safe_load` — **never** read crate geometry from
   `simcore`.

## DO NOT (anti-patterns from the team manual)
- No MAVSDK; don't port `kolomee.py`. No RealSense/depth; `getDepthAndDetect.py` is Challenge-1.
- No RKNN/`rknndecoder.py` (C2 is x86, zero NPU speedup). Optional YOLO runs via Ultralytics/ONNX.
- No altitude-layer deconfliction (illegal — everyone ~1.1 m). Use disjoint zones + lateral UWB
  separation + right-of-way.
- Don't hold a marker "lock" while sleeping/flying. Don't let an exception escape a worker thread.
- Don't burn the Phase-2 clock chasing one evader before the 3 autonomous tags are secured.

---

## AUTONOMOUS RUN PROTOCOL (this is an UNATTENDED overnight run)
Full details + the kickoff prompt: `docs/CLAUDE_CODE_KICKOFF.md`. The phase specs + objective
gates: `docs/PHASE_PLAN.md`. Summary loop:

```
for phase in P0..P11 (in order):
    read docs/PHASE_PLAN.md §<phase>
    implement it (TDD: write the phase's tests, then the code)
    attempt up to 4 times:
        run the gate:  make gate-<phase>      # = pytest -m <phase>  AND  full pytest
        if PASS: break
        else: read failures, fix, retry
    if PASS:
        update PROGRESS.md (status, test counts, one-paragraph log)
        git add -A && git commit -m "<phase>: <summary> (gate green)"
        continue to next phase
    else (still failing after 4 tries):
        write BLOCKED.md (exact failing test, hypotheses, what you tried)
        git add -A && git commit -m "<phase>: WIP — BLOCKED, see BLOCKED.md"
        STOP. Do not start later phases on a broken foundation.
```

### Unattended guardrails — absolute
- **NEVER make a test pass by weakening it.** Do not edit, delete, `skip`, `xfail`, loosen an
  assertion, or `pass`-stub a test/gate to go green. If the code can't satisfy an honest test,
  that's a BLOCK — stop and write BLOCKED.md. (This is the single most important overnight rule.)
- **NEVER weaken a HARD INVARIANT** to make something pass.
- **One commit per green phase.** Conventional message. Never commit red (except the single
  `WIP — BLOCKED` commit when you stop).
- **Stay in scope:** only the current phase's files. Don't refactor earlier green phases unless a
  gate forces it (and then keep their tests green).
- **If a phase truly needs the real sim** (only the `integration`-marked tests do), SKIP those
  tests in the overnight loop (they're excluded from the gates) and note it in PROGRESS.md.
- **Leave it green and explained.** A human wakes up to PROGRESS.md and a clean `git log`.

---

## Where things are
- `docs/RECONCILIATION.md` — mission-vs-sim diff (READ THIS: the sim is a subset; honour it).
- `docs/PHASE_PLAN.md` — the spine: each phase's goal, files+signatures, gate command, gate criteria.
- `docs/ARCHITECTURE.md` — the system (frames, control stack, perception, world model, C2).
- `docs/SDK_REFERENCE.md` — verified pyhulax signatures + semantics + gotchas. Trust this over memory.
- `docs/RULES_AND_CONSTRAINTS.md` — competition rules that bind the code + the `[TO CONFIRM]` list.
- `docs/MISSION_PLAN_SCHEMA.md` — the `mission_plan.yaml` contract (planner → mission).
- `reference/` — AUTHORITATIVE sources. **Authority order:** Finals brief > pyhulax KB > provided
  code > sim files > `to_consider.md` > older PDF. (`reference/README.md` indexes them.)
- `config/mission_config.yaml` — tunables; `[SYNC-WITH-SIM]` / `[TO CONFIRM]` markers flag values to
  verify. `examples/mission_plan.example.yaml` — a schema-valid plan fixture.
- `src/mission/` — the package. `tests/` — fake-SDK substrate + suite. `Makefile` — gates.
- `planner_gui/`, `c2/` — browser tools (Phases 9, 10), single-file, no build step.

## Test discipline
TDD. Gate per phase = `make gate-pN` (the phase's tagged tests **and** the full regression). The fake
SDK lets ~everything be tested headless and deterministically. Real-sim/hardware checks are
`@pytest.mark.integration` and are run **supervised**, not in this loop.
