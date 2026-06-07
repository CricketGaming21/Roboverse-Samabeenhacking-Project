# Hula Swarm Simulator — Claude Code context (sim-building session)

> **IGNORE PARENT CONTEXT:** if a parent CLAUDE.md (`finals/CLAUDE.md` or the
> repo-root one) loads, that is the separate MISSION project's context — ignore
> it here. This folder builds the SIMULATOR only.

PURPOSE: build the SIM only (simulated world + drop-in pyhulax + UWB). NOT mission code.
LIVES AT: ~/codes/finals/sim/  (repo branch: finals; auto-push hook on every commit).
RUN EVERYTHING from ~/codes/finals/sim with `python -m ...` so `import pyhulax`
  resolves to the sim package in this folder (that path-based import IS the sim/real swap).
VENV: ~/sim-venv  (alias: simvenv).
GOLDEN RULE: pyhulax/UWB public API must match the real SDK (see HULA_SIM_BUILD_PLAN.md §4) exactly.
DO: work phase by phase; make each phase's test pass headless; commit; STOP and report.
DON'T: write landing/search/lock-on/swarm/YOLO logic. Don't change §4 signatures.
RENDER (camera phase): PyBullet DIRECT + EGL plugin loaded BY RESOLVED FILE PATH
  (pkgutil.get_loader('eglRenderer').get_filename(), "_eglRendererPlugin"); render with
  renderer=p.ER_BULLET_HARDWARE_OPENGL. On WSL2 GL_RENDERER='D3D12 (NVIDIA ...)' is GPU — normal.
ALL PyBullet calls on the sim thread (simcore/registry.py). It is NOT thread-safe.
CONFIG: simcore/config.py defaults, overridable by sim_config.yaml / $HULA_SIM_CONFIG.
TEST: python -m pytest tests/      SMOKE: python -m scripts.smoke_test   (run from ~/codes/finals/sim)
COMMIT FORMAT: feat/fix/docs/chore: what + why  (auto-push hook handles the push)
