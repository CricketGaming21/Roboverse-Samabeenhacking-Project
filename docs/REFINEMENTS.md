# REFINEMENTS.md — Phase-2 refinement plan (R1 → R4; R5 conditional)

Post-integration refinements to the mission code. **Same discipline as the build:** TDD against the
fake substrate, each refinement has its own tests **and** keeps the full suite green, one conventional
commit per refinement, **never weaken a HARD INVARIANT or change the public `pyhulax` surface**. Do them
**in order, one at a time**, and **STOP after each for a human to review the recording** before the next.

## Invariants that still hold (from CLAUDE.md / RECONCILIATION.md — do not regress)
- ≤ **0.5 m/s** commanded; cruise ~**1.1 m**; **never `+up` to clear an obstacle, never overfly a footprint**
  — avoidance is **lateral-only**. Tilting the camera is NOT descending or overflying.
- UWB is **metres**, pyhulax is **cm** — convert at named boundaries. Read arena position from **UWB**, never
  from fake-only attrs.
- **Distinct-id de-dup.** **Your `MissionState` IS the score** (no referee on the real day).
- The sim implements a **SUBSET** of the SDK — real-only calls stay `hasattr`-guarded via `runtime/sdk_compat.py`.
- **Detection is ArUco-only for now.** **YOLO is a real-day drop-in behind the `RoverDetector` seam**, selected
  by a config flag — the seam stays intact so day-of YOLO is a config flip + a model file, zero rewrite.
- Shutdown always lands every drone in a `finally`.

## Verifying each phase (sim tools — see SIM_MANUAL)
- `--record run.mp4` → cockpit MP4 (watch behaviour). Transfer per the manual: `scp` to PC-B or
  `start run.mp4` from a Windows PowerShell (`explorer.exe` is unreliable on this box).
- `--dump run.jsonl` → per-tick trace (rover tracks, scoring, violations) — the **why** behind a number.
- `--dashboard` → live web cockpit on PC-B (Tailscale `:8080` or SSH tunnel).
- `logs/compliance` → no-fly/altitude-cap events (must stay empty).
- The mission's own evidence folder (R4) → one annotated capture per banked id.
- Integration run command (supervised, per phase):
  ```
  PYTHONPATH=~/codes/finals/sim HULA_SIM_CONFIG=~/codes/finals/sim/sim_config.yaml \
  HULA_SIM_RECORD=/tmp/<phase>.mp4 python -m mission.runtime.main
  ```

---

## R1 — Phase-1 ArUco removal
**Why:** the real landing pads have **no** ArUco markers — Phase 1 must land on UWB coordinates alone.
**First, verify the roster** we integrate against (manual says default may be `mixed`; the hand-off said
`convoy`): `grep -nE "motion:|marker_ids|hoop_radius_m|tolerance_m" ~/codes/finals/sim/sim_config.yaml` and
report `rovers.motion`. **Keep it `convoy`** for this baseline.
**Scope (Phase-1 ONLY — do NOT touch the landing control that lands at 1–4 cm):**
- Remove all ArUco decode/confirm from Phase 1 (`mission/phase1_land.py`, the `LAND_HOOP` path in
  `worker.py`). Land purely on the UWB target + hoop-gate: UWB-centred within `hoop_tol_m`, descent column
  footprint-clear (from `arena_truth`), down-barrier clear above 0.35 m. Drop `confirm_pad_aruco` from the
  Phase-1 path.
- Camera stays off/unstreamed in Phase 1 (landing is UWB-only); enabled at Phase-2 start.
- **KEEP the Phase-2 pad-id exclusion (skip ids 10–14)** — the sim still renders pad markers, so
  "any non-pad id = rover" must still exclude pads.
**Gate (fakes):** Phase-1 lands in-hoop with no ArUco available (assert no aruco call in the Phase-1 path);
Phase-2 exclusion/de-dup unchanged; full suite green.
**Verify (sim):** integration run still **3/3 in-hoop**; `logs/compliance` clean.

---

## R2 — Variable search camera
**Why:** a true 90° nadir is a microscope (sees only straight down); a forward tilt throws the footprint
ahead so the drone spots rovers approaching from a distance. But a tilted/distant marker is too small to
decode (640×480 reads ArUco only to ~1.7 m), so the camera angle becomes part of the control loop.
**Scope (Phase-2 only):**
- Config `camera.search_pitch_deg` (~**50–55°**, tunable; ideally altitude-aware). Set it at Phase-2 start.
- **Steepen toward nadir as range-to-target shrinks** on approach (so the marker grows/squares up for
  ArUco); **revert to search pitch** when no target / after release (ties into R4's bank-and-release).
- The ground-plane **projection must use the *live* pitch each frame**, not assume 90°.
- The drone holds ~**1.1 m** and routes **laterally** the whole time — only the camera angle changes.
- **Approach distance:** close to within ~1.5 m (the 640×480 ArUco range) before expecting a read. (Optional
  lever, NOT default: a sim-side bump to `camera.height: 720` extends ArUco range at a latency cost.)
- **Keep the nadir/no-tilt baseline selectable** by config (a fallback you don't delete).
**Gate (fakes):** camera pitch varies with range-to-target; projection correct at non-nadir angles; reverts
to search pitch with no target; nadir baseline still selectable; full suite green.
**Verify (sim):** `--record` shows the tilt steepening on approach; `logs/compliance` **zero** violations.

---

## R3 — ArUco-only detection + YOLO seam stub (no body-finder)
**Why:** the marker is the score and it's already in the sim — no body-finder is needed to score (you banked
4/5 with none). YOLO is the real-day long-range finder; we keep its seam but leave the backend empty for now.
**Scope:**
- **Detection now = ArUco only.** Find rovers by detecting their markers in-frame (the sim renders them),
  exclude pad ids 10–14, bank distinct rover ids. No motion detector, no body-finder active.
- **Keep the `RoverDetector` seam intact** with a `YoloRoverDetector(best.pt)` stub (loads the model if
  present, else a clear error) selectable by config `detector.backend: aruco | yolo` (**default `aruco`**).
- **Decision (override if you disagree):** the P5 `ClassicalRoverDetector` is **kept but disabled** behind
  the flag (a no-YOLO fallback for the day) — not deleted. Remove any half-built/unused detector cruft.
- On the real day: drop `best.pt` in, set `detector.backend: yolo` — zero code change.
**Gate (fakes):** detection path is ArUco-only when `backend: aruco`; pad ids excluded; seam swappable to the
YOLO stub by config; `YoloRoverDetector` raises clearly if no model; full suite green.
**Verify (sim):** integration run banks distinct rover ids via ArUco only. (R3 is small — mostly confirming
marker-only detection + a clean seam/flag.)

---

## R4 — Banking: scan-while-transit + bank-and-release + evidence capture  ← the big one
**Why:** on the real day there's no referee — **`MissionState` is your score**. Last run the camera *saw*
rovers it never *banked* (banking only happened during deliberate locks). This closes that gap and produces
the **judge deliverable** (annotated ArUco capture per rover).

**4a — Scan-while-transit banking.** Bank **any** ArUco read that passes the gate — **in-frame, ≥40 px, held
5 consecutive frames by the same drone** — **during transit and vantage patrol, not only during a dwell-lock.**
De-dup by id (unchanged). `MissionState` holds `{id: (evidence_path, arena_xy, t, drone_id)}`.

**4b — Bank-and-release (your new requirement).** The instant a read passes the gate and is banked, **release
the lock immediately and move on — do NOT keep dwelling/locking on an already-banked rover.** If a drone sees
an **already-banked id**, ignore it (no lock, no re-bank, no dwell). On release, **return the camera to the
search pitch** (R2) and resume patrol / proceed to the next target. Banking no longer depends on a long lock;
`lock_and_tag` is kept only to *centre* a marginal/edge read enough to clear the gate.

**4c — Evidence capture (your new requirement).** On each successful bank, save an **annotated screenshot** to
an evidence folder (`logs/evidence/`):
- The camera frame with the **detection box drawn** (high-contrast outline on the marker), a **label pill
  "ID <n>"**, and a small caption: `drone <k> · t=<sim_s> · (<north>, <east>) m`.
- One file per banked id (e.g. `logs/evidence/rover_<id>.png`); capture the **first qualifying** frame, and
  optionally replace it only if a later frame is clearly sharper/larger. Don't overwrite a good capture with a
  worse one.
- Also produce a **visually clean gallery** `logs/evidence/index.html` — a contact-sheet/grid of cards (one
  per banked id: thumbnail + ID + caption), dark theme, readable — the printable "inform the judge" artifact.
  Build/append it on each bank or regenerate at mission end. (May also surface in the C2 console.)

**Gate (fakes):**
- A drone banks a qualifying read **while transiting** between vantages (no dwell-lock required).
- **Bank-and-release:** the lock is released immediately after a bank; an already-banked id triggers **no**
  re-lock/re-bank/dwell; camera returns to search pitch on release.
- Each bank writes **one** annotated PNG (box + ID + caption) to `logs/evidence/`; the gallery lists all
  banked ids; **no double-count**; the ≥40 px + 5-frame + in-frame gate is still enforced.
**Verify (sim):** integration run → **single-run coverage (target 5/5, run it 3–4× — repeatable, not just
union)**; `logs/evidence/` has one clean annotated capture per banked id; `index.html` opens and shows them;
`logs/compliance` clean; drones visibly **don't loiter** on already-banked rovers.

---

## R5 — CONDITIONAL: sighting → intercept at chokepoints  (build ONLY if needed)
**Build this only if, after R4, single-run convoy coverage is *still* < 5/5 repeatably AND `--dump` shows
drones genuinely can't reach rovers in time (not lazy banking).** R4 alone may already get you to 5/5.
**If built:** feed rover sightings into the world-model tracks; the coordinator **vectors a drone to
intercept a predictable rover at its next chokepoint instead of chasing** (a stern chase never closes at
0.5 m/s); **tune vantages toward chokepoints** (gaps rovers must physically cross — these transfer to the
real arena), NOT the sim's specific convoy branch waypoints.
**Verify:** repeatable single-run 5/5 on convoy + the **intercept pattern in `--dump`** (a drone holds at a
gap while a rover's track approaches, then banks) + parking-not-chasing in `--record`.
**Note:** R5 helps the *predictable* convoy; the 2 human evaders stay a maximise-the-odds problem regardless.

---

## After R4 (or R5): flip to the evaders
Set `rovers.motion: mixed` (3 autonomous [20,21,22] + 2 evasive [30,31] + teleop), re-run, and pressure-test
the evaders — including hand-driving one with `rover_teleop` over SSH. The "any non-pad id = rover" rule +
the P8 logic already handle the [30,31] block. Expect single-run numbers to drop on the evaders — that's the
honest adversarial reality; count-first scoring means autonomous-3 + as-many-evaders-as-possible still scores.
