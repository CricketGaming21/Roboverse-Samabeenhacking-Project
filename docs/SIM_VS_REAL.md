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

## The real-hardware profile
`config/mission_real.yaml` (loaded by `--real`; **never edit `mission_config.yaml`**) bakes the real
pads, sets `discovery.use_dola: true`, lists `drones` as **tag-only** (IPs Dola-discovered; starts read
from UWB at runtime), and carries the safety values (≤0.5 m/s, ~1.1 m cruise, `hoop_tol_m: 0.20`, battery
RTL, hold-on-dropout). **Set `uwb.origin_x|origin_y` per assigned cage** (Cage1 0/0, Cage2 5.5/0, Cage3
5.5/5.5) — the real `UWBParserThread(x_origin, y_origin)` applies it so UWB and the pad coords share one
frame. The frame transform stays parametric via `frame.yaw_offset_deg`/`invert_*` (rung 2 — no code change).

## The bring-up ladder (climb in order — each rung GATES the next; all run with the real env)
- [ ] **0 · Env swap.** Real SDK importable; `python -c "import pyhulax; print(pyhulax.__file__)"` points
      at the **real** package, not `…/sim/pyhulax`. UWB serial detected. Scripts run as
      `python scripts/<x>.py` (they bootstrap `src` onto the path themselves).
- [ ] **1 · READ-ONLY connect + telemetry + live UWB — no motion.**
      `python scripts/connect_check.py --real` (or `--ip <ip>` for one; `scripts/hardware_check.py` is the
      PASS/FAIL variant). **Gate:** every drone PASS, ip↔tag pairing as logged, and the **live UWB matches
      where each drone physically sits** (this validates `uwb.origin_*`). Nothing arms.
- [ ] **2 · First ARMING — hover (clear space, no UWB needed).**
      `python scripts/hover_test.py --ip <ip> --i-have-clear-space` → takeoff 1.0 m, hover 5 s, land.
      **Gate:** arms, holds, lands cleanly (works at home).
- [ ] **3 · Camera / marker check (read-only).** `python scripts/camera_check.py --ip <ip>` → live
      `cv2.aruco` on the real feed. **Gate:** the 20 cm markers decode and read ≥40 px at the search range.
- [ ] **4 · Frame calibration (cage — first UWB motion).**
      `python scripts/yaw_calibrate.py --ip <ip> --i-have-clear-space` → nudges +forward, prints the UWB
      delta. Set **`frame.yaw_offset_deg` / `invert_forward` / `invert_right`** so **+forward → +North,
      +right → +East**. (Gentle stick; SDK clamps ≤0.5 m/s; lands in `finally`.)
- [ ] **5 · Single-drone UWB land + lock-on.** A `fly_to_uwb` to a pad at ~1.1 m, land **UWB-only**
      (R1, no ArUco) within `landing.hoop_tol_m`; then a Phase-2 lock that banks a **distinct id** and
      **releases**, routing **laterally** (no overfly). Tune `speed.kp_xy`/`kp_alt` gently.
- [ ] **6 · Full 3-drone mission.** `python -m mission.runtime.main --real` — Dola-discover + connect,
      UWB with the cage origin, assign the 3 `designated_pads` (nearest, no crossing), fly each ≤0.5 m/s
      at ~1.1 m, land in-hoop, then basic Phase-2. **Gate: 3/3 land in-hoop, shutdown lands every drone**
      (`finally`), Ctrl-C → abort-and-land. Per-drone status: connected / UWB pos / target pad / state.

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
