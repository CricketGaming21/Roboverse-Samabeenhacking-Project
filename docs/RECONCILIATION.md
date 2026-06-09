# RECONCILIATION.md — mission scaffold vs the real sim (finals/sim @ branch finals)

Done by reading the cloned sim: `pyhulax/{core,api,video,discovery}.py`, `simcore/frames.py`,
`sim_config.yaml`, `arena_truth.yaml`. Verdict: **the architecture and the frame/unit/sign model
match the sim.** A handful of concrete divergences must be honoured — they're folded into the docs/
config below and summarised here.

## ✅ Confirmed identical (no change needed)
- **Frames/units/signs.** Arena/UWB `x=North(m), y=East(m)`; `+yaw` CCW from north; `heading 0 = +north`
  → at locked yaw, body **forward=+north, right=+east**. `send_manual_control(forward,right,up,rotate)`
  sticks **−1..+1**, body-relative, **+forward=nose, +right=body-right, +up=climb, +rotate=CCW**, no
  `buttons`. `move_to/get_position` = takeoff-origin **cm**, `x=right,y=forward,z=up`, frozen at takeoff
  (no yaw rotation), `get_position` **drifts** (correct with UWB). `get_altitude` = ToF cm. Nadir camera
  (`DOWN_ABSOLUTE,90`): image-up = nose. (Source: `simcore/frames.py`, `pyhulax/api.py`.)
- **UWB.** `UWBParserThread.get_tag_position(tag_id) -> (x_m, y_m, t)`, `(None,None,None)` if unseen.
- **Video.** `VideoStream.latest_frame` (property) → `VideoFrame.to_rgb() -> np.ndarray` (RGB). Frames
  render real ArUco markers → `cv2.aruco` decodes them.
- **Config parity.** Arena 10×6; pads ids 10–14 at the coords in `config/mission_config.yaml`;
  `velocity_levels.max_mps: 0.5` (matches our cap); camera 640×480, FOV 71°, `DICT_6X6_250`; drones
  tag_ids 0/1/2 @ ips 10.0.0.11/12/13; UWB 10 Hz, noise 0.05. **Ambush is scenario-owned — no API to
  trigger it (good).**

## ⚠️ Divergences — HONOUR THESE (folded into the docs/config)
1. **The sim implements a SUBSET of the SDK.** The sim's `DroneAPI` is exactly:
   `connect, takeoff, land, hover, move, rotate, move_to, send_manual_control, manual_fly, get_state,
   get_position, get_orientation, get_altitude, get_battery, get_obstacles, any_obstacle,
   get_drone_status, set_barrier_mode, set_avoidance_direction, set_camera_angle, create_video_stream,
   set_video_stream` (+ stubbed edu: curve_to, circle, recognize_*, follow_line, fire_laser, set_clamp,
   set_electromagnet).
   **NOT present** (real-SDK only): `set_app_mode, send_app_heartbeat, set_velocity_level,
   set_yaw_rate_level, stop_manual_control, arm, disarm, disconnect, robust_connect, get_velocity,
   get_drone_id, get_flight_data, set_video_resolution, enable_battery_failsafe`.
   → **The mission must call only the sim surface directly, and wrap real-SDK-only init behind
   `hasattr` guards** so ONE codebase runs on both. Put this in `src/mission/runtime/sdk_compat.py`:
   `prepare_manual_control(drone)` calls `set_app_mode(1)`, `send_app_heartbeat()`, `set_velocity_level(...)`
   **only if those attrs exist** (no-op on the sim, real on hardware); `release(drone)` calls
   `stop_manual_control()`/`disconnect()` if present. **Never call `set_app_mode`/heartbeat/`arm`
   unguarded** — it AttributeErrors on the sim.
2. **`connect(ip)` is REQUIRED-arg and returns `CommandResult`** (no `timeout`, no default ip, does not
   raise on failure, no `disconnect`). Call `connect(ip)` with an explicit ip; don't rely on `connect()`
   raising.
3. **`hover(duration_seconds)` REQUIRES a duration.** Do **not** call bare `hover()` to station-keep —
   station-keep with `send_manual_control()` (zero sticks). (`stop_manual_control()` is absent too.)
4. **`Obstacles` has exactly 5 fields: `forward, back, left, right, down` — NO `up`.** Helpers:
   `Obstacles.any` (property) and `DroneAPI.any_obstacle()` (method); `get_drone_status()` returns the
   raw bitmask (bits 0=fwd,1=back,2=left,3=right,4=down). Fix anywhere that listed `up`.
5. **No `get_velocity()`.** The arrival test in `fly_to_uwb` must estimate speed from **consecutive UWB
   samples** (Δpos/Δt), not `get_velocity()`. No `get_drone_id()`/`get_flight_data()` either — don't depend.
6. **No `enable_battery_failsafe()`.** Battery RTL is **mission-side**: watch `get_battery()` and trigger
   RETURN_AND_LAND at the config threshold. (`get_battery()` exists.)
7. **Discovery import differs.** Sim provides `pyhulax.discovery.Dola` (config-backed, same method names:
   `get_all_ips()->{plane_id: ip}`, etc.); the real day uses the standalone `dola.py` (UDP 8668).
   Abstract it in `runtime/discovery.py`: try `from pyhulax.discovery import Dola` else `from dola import
   Dola`; or just use the fixed config IPs in the sim. Map `plane_id` (control) ↔ UWB `tag_id` (position).
8. **Built-in reactive helpers exist** (`set_barrier_mode(True)` = move-into-obstacle "stops short" instead
   of penetrating; `set_avoidance_direction(dir, dist, BarrierMask.HORIZONTAL)`). Optional extra safety on
   top of our planner+guard — and `BarrierMask.HORIZONTAL` is the clean "never climb" mask if we use it.

## ✅ Planner input is solved by the sim
The sim emits **`arena_truth.yaml`** (plain `{arena, crates[center,size,height], archway}`, loadable with
`yaml.safe_load`, **no simcore**) via `python -m scripts.emit_arena_truth`. The mission's planner reads
THIS (now vendored at `config/arena_truth.yaml`); on the day the Discord coordinate file plays the same
role. Re-emit after any `sim_config.yaml` arena edit and re-copy.

## ⛔ Sim does NOT yet model (so these stay fake-tested until you update the sim)
The current sim is **5 autonomous convoy rovers, ids 20–24, no evasion** (`rovers.motion: convoy`).
NOT yet present: the **2 human-piloted/teleop evaders**, **different opponent marker ids**, **no-fly-over
enforcement/flagging**, and a **hoop** knob (landing still scores at `tolerance_m: 0.30`). So:
- P8 (adversarial) and the no-fly-over violation alarm are validated against **fakes** only until your
  sim update lands (the sim-update prompt from earlier covers exactly these).
- The mission's "**any non-pad id is a rover**" rule already handles different opponent ids — but the
  current sim only exercises 20–24.
- Integration Phase-2 against today's sim tests the **autonomous-coverage** path, not the pincer/evasion.
