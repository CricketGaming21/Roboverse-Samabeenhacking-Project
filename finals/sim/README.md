# Hula Swarm Simulator

Drop-in `pyhulax` SDK + `UWBParserThread` + PyBullet world for developing the
RoboVerse 2026 Challenge 2 mission code without hardware.

Mission code does `import pyhulax` / `from UWBParserThread import UWBParserThread`
unchanged; running from this folder puts the sim versions on the path — that
path-based import IS the sim/real swap (see `HULA_SIM_BUILD_PLAN.md` §5.5).

**Status: Phase 0 (scaffolding + API contract stubs).** Behaviour lands phase by
phase per `HULA_SIM_BUILD_PLAN.md` §6. Full docs in Phase 8.

```bash
# from ~/codes/finals/sim, venv ~/sim-venv (alias: simvenv)
pip install -r requirements.txt
python -m pytest tests/
```
