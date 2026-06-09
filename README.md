# RoboVerse 2026 — Challenge 2 Mission Code (Pre-University)

Mission code for 3 Highgreat **HULA** drones: `phase1_land` (deploy + land in hoops) and
`phase2_search` (ambush: tag the rover convoy). Runs on the **C2** against the **public pyhulax +
UWBParserThread** API; developed against a simulator, deployable on the real drones by swapping the
import path. **This repo is separate from the sim repo.**

## Quickstart
```bash
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
# 1) drop the authoritative files into reference/  (see reference/README.md)
git init && git add -A && git commit -m "scaffold: mission repo (pre-P0)"
make test            # pre-P0: trivial / no tests — fine
```
Then open this folder in **Claude Code** and paste the kickoff prompt from
`docs/CLAUDE_CODE_KICKOFF.md`. It will build P0→P11 autonomously, committing per green phase.

## Read order
`CLAUDE.md` → `docs/PHASE_PLAN.md` → `docs/ARCHITECTURE.md` → `docs/SDK_REFERENCE.md` →
`docs/RULES_AND_CONSTRAINTS.md`. Authoritative sources live in `reference/` (authority order in its README).

## Layout
- `src/mission/` — package (control, planner, perception, world, mission, runtime)
- `tests/` — fake-SDK substrate + suite (headless, deterministic); `Makefile` gates (`make gate-pN`)
- `config/` — `mission_config.yaml` (tunables; `[SYNC-WITH-SIM]`/`[TO CONFIRM]`)
- `planner_gui/` (P9) + `c2/` (P10) — single-file browser tools
- `PROGRESS.md` — Claude Code's per-phase status log
