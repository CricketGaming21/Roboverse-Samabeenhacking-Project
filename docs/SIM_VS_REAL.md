# SIM_VS_REAL.md — the sim→real swap + the bring-up ladder

The mission code is **identical** in sim and on hardware. Only two things change on the day:
1. which `pyhulax` / `UWBParserThread` is importable, and
2. the on-the-day-calibrated values in `config/mission_config.yaml`.

> The bring-up **ladder** below was assembled from the mission's own components (it wasn't handed
> to us verbatim) — adjust the rungs to the cage/marshal rules on the day. The ordering principle is
> fixed: **read-only before any motion, one drone before three, low/short before full mission.**

## The swap (no code edits) — make `import pyhulax` resolve to the right SDK
`import pyhulax` and `from UWBParserThread import UWBParserThread` are resolved by whatever is on the
path — **nothing in the mission pins them**:
- **Fake substrate (unit tests):** `tests/conftest.py` installs the fakes into `sys.modules`. Nothing else.
- **Against the sim:** put the `hula_sim` repo on the path and point the sim config at itself:
  `PYTHONPATH=~/codes/finals/sim:src HULA_SIM_CONFIG=~/codes/finals/sim/sim_config.yaml …`
  (`HULA_SIM_RECORD=/tmp/run.mp4` optionally records). `runtime/discovery.py` resolves IP↔tag from
  `config.drones` (the sim's `Dola` raises `NotImplementedError`).
- **On real hardware — DROP the sim env:** unset / don't pass **`PYTHONPATH=…/sim`** and all
  **`HULA_SIM_*`** vars, so `import pyhulax` resolves to the **installed real SDK** (not the sim
  drop-in). Install the real `pyhulax` (with `[video]`) + the standalone `dola.py` + the serial
  `UWBParserThread.py`. Keep `src` importable (`PYTHONPATH=src` or `pip install -e .`).
  `runtime/discovery.py` falls back to `from dola import Dola`; pass `HULA_USE_DOLA=1` to broadcast-
  discover live IPs, else it uses the config map.

Because the sim is a **SUBSET** of the real SDK, every real-only init/teardown call (`set_app_mode`,
`send_app_heartbeat`, `set_velocity_level`, `stop_manual_control`, `disconnect`,
`enable_battery_failsafe`, `arm`/`disarm`) is routed through `runtime/sdk_compat.py` `hasattr`
guards — no-ops on the sim, real on hardware. **Never call them directly.** The read-only bring-up
init uses `sdk_compat.prepare_telemetry` (app-mode + heartbeat, **never arms**).

## The bring-up ladder (climb in order — each rung GATES the next)
- [ ] **0 · Env swap.** Real SDK importable (above); `python -c "import pyhulax; print(pyhulax.__file__)"`
      points at the **real** package, not `…/sim/pyhulax`. UWB serial detected.
- [ ] **1 · READ-ONLY hardware check — no motion.** `python scripts/hardware_check.py`
      (or `--drone <ip>` one at a time). **Gate: every drone PASS** — connected, telemetry non-null
      (battery/position/orientation/altitude/obstacles), and UWB returns real coords for its tag.
      Nothing takes off or arms. Fix wiring/UWB/IP-map before climbing.
- [ ] **2 · Frame calibration (first powered motion — caged/low, marshal-approved).** Through
      `sdk_compat.prepare_manual_control`, send a small `+forward` then `+right` stick and watch UWB.
      Set **`frame.yaw_offset_deg`** so body-forward = +North, and **`frame.invert_forward` /
      `invert_right`** so the sticks aren't mirrored. **Gate: +forward → +North, +right → +East in UWB.**
- [ ] **3 · Single-drone `fly_to_uwb`.** Fly one drone to a near UWB waypoint at ~**1.1 m**.
      **Gate:** converges within `speed.arrive_tol_m`, **never > 0.5 m/s**, holds position on a UWB
      dropout, no `+up` to clear anything. Tune `speed.kp_xy`/`kp_alt` gently here.
- [ ] **4 · Single-drone Phase-1 land (UWB-only — R1).** Land one drone on a sample pad by UWB alone
      (no ArUco). **Gate: lands within `landing.hoop_tol_m`** of the pad centre; set `hoop_tol_m` from
      the measured hoop radius minus a UWB-jitter margin.
- [ ] **5 · Single-drone Phase-2 lock-on + tag.** Over a sample rover/marker: confirm `aruco.dictionary`
      (`DICT_6X6_250`) and `camera.h_fov_deg`; the drone banks a **distinct id** (≥40 px, in-frame, 5
      frames), **releases**, and routes **laterally** (no overfly). **Gate: one clean bank + release.**
- [ ] **6 · Full 3-drone mission.** `python -m mission.runtime.main` (Phase 1 → Phase 2, `parallel`).
      **Gate: 3/3 land in-hoop, distinct rover ids tagged, `logs/compliance` empty (no over-crate /
      altitude-cap), shutdown lands every drone.**

## On-the-day calibration values (edit `config/mission_config.yaml` only)
Feed these as you climb the ladder (rung in brackets):
- [ ] **`frame.yaw_offset_deg`, `invert_forward`, `invert_right`** — rung 2.
- [ ] **`speed.kp_xy` / `kp_alt`** — rung 3 (start gentle).
- [ ] **`landing.hoop_tol_m`** — rung 4 (measured hoop radius − UWB margin).
- [ ] **`aruco.dictionary`, `camera.h_fov_deg`, `camera.search_gimbal_deg` / `read_gimbal_deg`** — rung 5.
- [ ] **`pads:`** (id + north/east + `valid`/`designated`) — the announced Discord coordinates.
- [ ] **`drones:`** (ip ↔ `tag_id` ↔ `start`) + **`uwb.tag_ids`** — confirm which tag is which drone.
- [ ] **`failsafe.battery_rtl_pct` / `phase2_budget_s`** — from the briefing's loiter window.
- [ ] **`config/arena_truth.yaml`** — paste the real crate map (planner input; re-validate routes in the GUI).

## What stays fake-only until the sim updates (see docs/RECONCILIATION.md ⛔)
The current sim models 5 autonomous convoy rovers (ids 20–24, no evasion). The **2 human-piloted
evaders** + **different opponent ids** are exercised with `rovers.motion: mixed` (`[30,31]`); their
realism checks are `@pytest.mark.integration`. The "any non-pad id = rover" rule + P8 logic already
handle the `[30,31]` block.
