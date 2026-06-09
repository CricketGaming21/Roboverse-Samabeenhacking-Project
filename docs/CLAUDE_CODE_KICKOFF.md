# CLAUDE_CODE_KICKOFF.md — how to start the unattended run

## Before you start Claude Code (human, ~5 min)
1. Drop the reference files into `reference/` exactly as `reference/README.md` specifies.
2. Create the env and confirm the suite is wired:
   ```bash
   python -m venv .venv && source .venv/bin/activate
   pip install -r requirements.txt
   git init && git add -A && git commit -m "scaffold: mission repo (pre-P0)"
   python -m pytest        # expect "no tests ran" or trivial — that's fine pre-P0
   ```
3. Skim `config/mission_config.yaml`; the `[SYNC-WITH-SIM]` values should match your current sim
   (`reference/sim/sim_config.yaml`). They can stay as placeholders for the overnight logic run —
   real numbers are confirmed on the day.
4. Launch Claude Code in the repo root (it auto-reads `CLAUDE.md`).

## The kickoff prompt (paste into Claude Code)

> You are continuing the RoboVerse Challenge-2 **mission code** build. Read `CLAUDE.md`,
> `docs/PHASE_PLAN.md`, `docs/ARCHITECTURE.md`, `docs/SDK_REFERENCE.md`,
> `docs/RULES_AND_CONSTRAINTS.md`, and the `reference/` index — then verify every SDK signature
> against `reference/pyhulax_knowledge_base.txt` and every rule against
> `reference/brief/Finals_brief.pdf`; if anything I wrote disagrees with those, trust those and note
> it in PROGRESS.md.
>
> This is an **unattended overnight run**. Work `docs/PHASE_PLAN.md` from **P0 to P11 in order**,
> TDD. For each phase: write its tests, implement, then run `make gate-p<N>` (the phase's tagged
> tests **and** the full regression). Retry up to 4 times. **On green:** update `PROGRESS.md`, then
> `git commit` with a conventional message, then move to the next phase **automatically — do not wait
> for me.** **On a hard block after 4 tries:** write `BLOCKED.md` (the exact failing test, your
> hypotheses, what you tried), commit it as `WIP — BLOCKED`, and **STOP** (do not start later phases).
>
> **Absolute rule:** never make a test pass by weakening, skipping, xfailing, deleting, or
> `pass`-stubbing it, and never weaken a HARD INVARIANT (CLAUDE.md). If honest code can't pass an
> honest test, that's a BLOCK — stop and explain. Everything runs against the fake SDK substrate
> (`tests/README.md`); `@pytest.mark.integration` tests need the real sim — implement them but they
> are excluded from the gates, so skip them tonight and note it.
>
> Keep each phase in scope, seed for determinism, no network in tests, and make sure the safety
> invariants (no `+up` in avoidance, ≤0.5 m/s, no path over a footprint, distinct-id de-dup) are
> enforced **and** covered by a test. Start with P0 now and keep going phase-by-phase until you finish
> P11 or hit a block.

## What the human will see on waking
- `git log --oneline` = one green commit per completed phase (newest = furthest reached).
- `PROGRESS.md` = a table + a short paragraph per phase, including any `[TO CONFIRM]` you hit.
- If stopped: `BLOCKED.md` at the top of the tree with the exact blocker.
- `make test` reproduces the green suite.

## Resuming after a block / after the human fixes something
Re-launch with: *"Read PROGRESS.md and BLOCKED.md, resume the autonomous P0–P11 loop from the first
unfinished phase under the same protocol and guardrails."*
