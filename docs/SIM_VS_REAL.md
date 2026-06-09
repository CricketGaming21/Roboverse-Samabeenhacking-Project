# SIM_VS_REAL.md — the import-path swap + on-the-day calibration checklist

The mission code is **identical** in sim and on hardware. Only two things change on the day:
1. which `pyhulax` / `UWBParserThread` is on the `PYTHONPATH`, and
2. the on-the-day-calibrated values in `config/mission_config.yaml`.

## The swap (no code edits)
- **Tests / this repo's overnight run:** `tests/conftest.py` installs the fake `pyhulax` +
  `UWBParserThread` into `sys.modules`. Nothing else needed.
- **Against the sim:** put the `hula_sim` repo's drop-in `pyhulax/` package and `UWBParserThread.py`
  on the path (e.g. `PYTHONPATH=/path/to/hula_sim`). `runtime/discovery.py` resolves
  `pyhulax.discovery.Dola`; fixed config IPs (`10.0.0.11/12/13`) also work.
- **On hardware:** install the real `pyhulax` (with `[video]`) + the standalone `dola.py` +
  `UWBParserThread.py` (serial). `runtime/discovery.py` falls back to `from dola import Dola`.

Because the sim is a **SUBSET** of the real SDK, every real-only init/teardown call
(`set_app_mode`, `send_app_heartbeat`, `set_velocity_level`, `stop_manual_control`, `disconnect`,
`enable_battery_failsafe`, `arm`/`disarm`) is already routed through `runtime/sdk_compat.py`
`hasattr` guards — no-ops on the sim, real on hardware. **Never call them directly.**

## On-the-day calibration checklist (edit `config/mission_config.yaml` only)
- [ ] **`frame.yaw_offset_deg`** — nudge a drone forward, watch UWB; set so body-forward = +North.
- [ ] **`frame.invert_forward` / `invert_right`** — verify a +forward / +right stick moves the
      drone the expected way in the UWB frame; flip if mirrored.
- [ ] **`speed.kp_xy` / `kp_alt`** — tune the UWB loop on the real drone (start gentle).
- [ ] **`landing.hoop_tol_m`** — measure the sample pad's hoop radius minus a UWB-jitter margin.
- [ ] **`aruco.dictionary`** — confirm on the sample pad (`DICT_6X6_250` expected).
- [ ] **`camera.h_fov_deg`** + `search_gimbal_deg` / `read_gimbal_deg` — verify projection + that
      a marker is decodable at search altitude.
- [ ] **`pads:`** — replace with the announced Discord coordinates + valid/invalid flags.
- [ ] **`uwb.tag_ids`** + discovery `plane_id`↔`tag_id` map — confirm which tag is which drone.
- [ ] **`failsafe.battery_rtl_pct` / `phase2_budget_s`** — set from the briefing's loiter window.
- [ ] **Arena truth** — re-emit / paste the real crate map into `config/arena_truth.yaml`
      (planner input; the Planner GUI re-validates routes against it).

## What stays fake-only until the sim updates (see docs/RECONCILIATION.md ⛔)
The current sim models 5 autonomous convoy rovers (ids 20–24, no evasion). The **2 human-piloted
evaders** + **different opponent ids** + **no-fly enforcement** + a **hoop** knob are validated
against the fake substrate until the sim update lands; their realism checks are `@pytest.mark.integration`.

## Run order on the day
1. Power drones; confirm UWB tags live; `discovery.resolve()` → `{tag_id: ip}`.
2. Author/freeze `mission_plan.yaml` in the Planner GUI (or reuse a validated one).
3. Launch the C2 console; run the pre-flight checklist.
4. Start `runtime/main.Mission(...).run(parallel=True)` — Phase 1 lands, Phase 2 tags,
   shutdown lands all in `finally`.
