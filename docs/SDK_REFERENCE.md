# SDK_REFERENCE.md — the API surface, RECONCILED against the real sim (finals/sim)

Verified against `pyhulax/{core,api,video,discovery}.py` + `simcore/frames.py` in the cloned sim, and
the KB (`reference/pyhulax_knowledge_base.txt`). See `docs/RECONCILIATION.md` for the full diff.

## ⚠️ The sim implements a SUBSET — call ONLY these directly
Sim `DroneAPI` methods (safe to call directly; present on sim AND real):
`connect(ip)`, `takeoff(height_cm=100, blocking=True, flags=TakeoffFlags.NONE)`, `land(blocking=True)`,
`hover(duration_seconds, blocking=True)`, `move(direction, distance_cm, speed=ZOOM)`,
`rotate(angle_degrees)` (+=CCW), `move_to(x,y,z, speed=ZOOM)`,
`send_manual_control(forward,right,up,rotate)->bool`, `manual_fly(duration_sec, forward,right,up,rotate,
rate_hz=20, on_frame=None)`, `get_state()`, `get_position()->Vector3`, `get_orientation()->Orientation`,
`get_altitude()->float`, `get_battery()->int`, `get_obstacles(drone_id=0)->Obstacles`,
`any_obstacle()->bool`, `get_drone_status()->int|None`, `set_barrier_mode(enabled)`,
`set_avoidance_direction(direction, distance_cm=0, barrier_mask=BarrierMask.ALL)`,
`set_camera_angle(mode, angle=0)`, `create_video_stream()->VideoStream`, `set_video_stream(enabled)`.

## ⛔ Real-SDK-only — NOT on the sim. NEVER call unguarded; wrap in `hasattr` via `runtime/sdk_compat.py`
`set_app_mode`, `send_app_heartbeat`, `set_velocity_level`, `set_yaw_rate_level`, `stop_manual_control`,
`arm`, `disarm`, `disconnect`, `robust_connect`, `get_velocity`, `get_drone_id`, `get_flight_data`,
`set_video_resolution`, `enable_battery_failsafe`.
→ `sdk_compat.prepare_manual_control(drone)`: `if hasattr(drone,'set_app_mode'): drone.set_app_mode(1)`;
likewise `send_app_heartbeat()` / `set_velocity_level(...)`. No-op on sim, real on hardware. One codebase.

## Key semantics (identical on sim + real)
- **Frames:** arena/UWB `x=North(m), y=East(m)`; `+yaw` CCW from north; **heading 0 ⇒ body forward=+north,
  right=+east** (lock yaw → arena error maps straight to forward/right). `move_to/get_position` =
  **takeoff-origin cm**, `x=right,y=forward,z=up`, frozen at takeoff (no yaw rotation). **`get_position`
  DRIFTS** (optical-flow/IMU) — correct with UWB; UWB never drifts. `get_altitude` = downward ToF cm.
- **`send_manual_control(forward,right,up,rotate)`** — sticks **−1..+1**, body-relative, **+forward=nose,
  +right=body-right, +up=climb, +rotate=CCW**, no `buttons`. ~20 Hz; empty call hover; stale (>~1 s) frames
  coast→hold (so keep emitting). Returns bool, never raises.
- **`connect(ip)`** — REQUIRED ip, returns `CommandResult`, does not raise on failure (no `disconnect`).
- **`hover(duration_seconds)`** — needs a duration. **Station-keep with `send_manual_control()` (zeros),
  not `hover()`.**
- **`Obstacles`** — exactly `forward, back, left, right, down` (**NO `up`**); `.any` property; clear below
  ~0.35 m. `get_drone_status()` raw bits: 0=fwd 1=back 2=left 3=right 4=down. Barrier range ~0.6 m
  horizontal / 0.4 m down (IR range is adjustable on the real drone — confirm on the day).
- **No `get_velocity()`** → estimate speed from consecutive UWB samples (Δpos/Δt) for the arrival test.
- **No `enable_battery_failsafe()`** → mission-side battery RTL on `get_battery()` ≤ threshold.
- **Camera:** `set_camera_angle(CameraPitchMode.DOWN_ABSOLUTE, 90)` = straight down; `UP_ABSOLUTE, 0` =
  ahead. Nadir image-up = nose. `h_fov_deg` + `mount_offset_m` drive projection. 640×480 default. Built-in
  vision is QR-only — use `cv2.aruco` for markers.
- **Enums** (`pyhulax.core`): `Direction(FORWARD=0..DOWN=5)`, `VelocityLevel(SLOW=300,MEDIUM=200,ZOOM=100,
  TURBO=50` = P-gain divisor, not m/s; sim clamps all to 0.5`)`, `CameraPitchMode(UP_ABSOLUTE=0,
  DOWN_ABSOLUTE=1,CALIBRATE=4,UP_RELATIVE=5,DOWN_RELATIVE=6)`, `BarrierMask(...,HORIZONTAL,ALL)`
  (`HORIZONTAL` = the clean "never climb" mask), `TakeoffFlags(NONE,RESET_YAW,WITH_LOAD)`.
- **Optional firmware safety:** `set_barrier_mode(True)` makes a `move()/move_to` into an obstacle "stop
  short" (goal fails) instead of penetrating — a useful net on top of our planner+guard.

## UWB — `reference/provided_code/UWBParserThread.py`
`UWBParserThread(...).start()`; `get_tag_position(tag_id) -> (x_m, y_m, t)` in **metres**, `(None,None,
None)` if unseen (table briefly empties each frame → normal → **hold position**). Serial USB @ 921600;
`x_origin/y_origin` not applied.

## Video — `pyhulax/video.py`
`d.set_video_stream(True)`; `s = d.create_video_stream(); s.start()`; `f = s.latest_frame` (property,
`VideoFrame|None`); `f.to_rgb() -> np.ndarray` (**RGB** → `cv2.cvtColor(f, COLOR_RGB2GRAY)` for aruco).

## Discovery — `pyhulax/discovery.py` (sim) vs `dola.py` (real, UDP 8668)
Both expose `Dola` with `get_all_ips()->{plane_id: ip}`, `get_ip_by_plane_id`, etc. `runtime/discovery.py`
should `try: from pyhulax.discovery import Dola` `except: from dola import Dola`. Map `plane_id`↔`tag_id`.
The 3 assessment drones are cage-provided → discover fresh; in the sim, fixed ips 10.0.0.11/12/13 also work.

## cv2.aruco — `DICT_6X6_250`
`getPredefinedDictionary(cv2.aruco.DICT_6X6_250)` → `ArucoDetector` → `detectMarkers(gray)` (all markers
in one pass). `opencv-contrib-python` ≥4.7. **Pads = 10–14; ANY other decoded id = a rover** (opponent
rovers carry different ids). Confirm the dictionary on the sample pad on the day.
