# reference/ — authoritative source material (Claude Code: read, don't modify)

**Authority order (resolve conflicts top-down):**
1. **`brief/Finals_brief.pdf`** — the official Finals brief. Overrides everything.
2. **`pyhulax_knowledge_base.txt`** — the SDK truth (crawl of pyhulax.xenops.ae). Verify all signatures here.
3. **`provided_code/`** — the real provided code; the exact API + the detection seam to mirror. (`UWBParserThread.py`, `dola.py`, `huladola.py`, `rover_detection_example.py` are **already vendored here** from the repo — confirm they match your uploads.)
4. **`sim/`** — the simulator we develop against; the parity source for arena/ids/frame.
5. **`to_consider.md`** — the team's living manual (rich context; NOT authoritative over 1–4).
6. **`brief/RoboVerse_Finals.pdf`** — the OLDER deck; **superseded** by Finals_brief.pdf. Context only.

## Drop these files in (from the chat uploads), exactly here:

```
reference/
├── pyhulax_knowledge_base.txt          <- pyhulax_complete_knowledge_base.txt   (SDK signatures truth)
├── to_consider.md                      <- to_consider.md                        (team manual; context)
├── Potential_Detection_Targets.txt     <- Potential_Detection_Targets.txt       (ArUco sample code)
├── provided_code/
│   ├── UWBParserThread.py              <- UWBParserThread.py                     (our UWB API + quirks)
│   ├── dola.py                         <- dola.py                               (discovery; port 8668)
│   ├── huladola.py                     <- huladola.py                           (multi-drone-from-one-PC pattern)
│   ├── rover_detection_example.py      <- rover_detection_example.py            (the RoverDetector SEAM to mirror)
│   └── __init__.py                     <- __init__.py                           (mission-examples boundary note)
├── brief/
│   ├── Finals_brief.pdf                <- Finals_brief.pdf                       (AUTHORITATIVE — newest)
│   ├── RoboVerse_Finals.pdf            <- RoboVerse_Finals.pdf                   (older, superseded — context)
│   ├── Phase_1_task.png                <- Phase_1_task.png                       (landing-map image)
│   └── Phase_2_task.png                <- Phase_2_task.png                       (convoy/ambush image)
└── sim/
    ├── sim_config.yaml                 <- sim_config.yaml                        (PARITY: arena/ids/frame/speeds)
    ├── SIM_MANUAL.md                   <- SIM_MANUAL.md                          (how to run/record the sim)
    └── DETECTION.md                    <- DETECTION.md                          (sim/mission detection split)
```

Notes for Claude Code:
- `provided_code/rover_detection_example.py` defines `Detection` + `RoverDetector` + `two_stage_scan`
  — your `src/mission/perception/detector.py` MIRRORS this interface (same names) so day-of code swaps.
- `sim/sim_config.yaml` is the source for `[SYNC-WITH-SIM]` values in `config/mission_config.yaml`
  (arena dims, pad ids/coords, rover id blocks, speeds). The sim is being updated (real map, 0.5 m/s
  cap, evasive+teleop rovers, no-fly enforcement, hoop) — a human re-syncs config after each sim update.
- The real sim is NOT in this repo. Overnight tests use the fake substrate (`tests/README.md`); any
  `@pytest.mark.integration` test that needs the real sim is excluded from the gates.

## Already wired from the sim (no action needed)
- `config/arena_truth.yaml` — the sim's emitted ground-truth crates/arena (the planner loads this; the
  Discord coordinate file plays the same role on the day). Re-emit + re-copy after any sim arena edit:
  `python -m scripts.emit_arena_truth` in the sim repo.
- `docs/RECONCILIATION.md` — the mission-vs-sim diff. **The sim is a SUBSET of the SDK** — the mission
  guards real-only calls via `runtime/sdk_compat.py`. Read it before P0.
