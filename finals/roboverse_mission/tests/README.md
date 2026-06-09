# tests/ — the fake-SDK substrate + conventions (built in P0, used by every later phase)

The mission imports `pyhulax` and `from UWBParserThread import UWBParserThread`. For the **unattended
overnight run there is no sim/hardware** — `tests/conftest.py` installs **fakes** into `sys.modules`
so those imports resolve to test doubles. This makes ~all logic testable headless + deterministic.

## conftest.py must
- Before mission imports, do `sys.modules["pyhulax"] = <fake module>` and provide a fake
  `UWBParserThread` module, so `import pyhulax` / `from UWBParserThread import UWBParserThread` hit the
  fakes. Expose `pyhulax.DroneAPI`, `pyhulax.core` (enums `Direction`, `CameraPitchMode`,
  `VelocityLevel`, `BarrierMask`, `VideoResolution`), `pyhulax.video` if referenced.
- Provide a session **seed** fixture (seed `random`, `numpy`) for determinism.
- Provide a `world` fixture (a `FakeWorld` with arena + crates + pads + rovers) and a
  `drone(world, tag_id)` fixture.

## FakeWorld + FakeDroneAPI (the kinematic + camera model)
- World holds: arena rect; crate footprints; pad markers (ids 10–14 at coords); rover markers (each an
  id + position + optional scripted motion path, incl. an "evasive" mover for P8).
- `send_manual_control(fwd,right,up,rotate)`: velocity = stick · `max_mps` (clamp), integrate (x,y) in
  the arena frame via the drone's yaw, integrate altitude from `up`. Honour the **0.5 cap**.
- `get_obstacles(...)`: boolean per direction = a crate footprint OR another drone within the barrier
  range in that body direction; clear below 0.35 m.
- `get_altitude()`: integrated altitude (cm). `get_position()`: onboard estimate (may add bounded
  drift, distinct from UWB truth). `get_orientation().yaw`: current heading.
- `FakeVideoStream.latest_frame.to_rgb()`: render an RGB frame that **draws the real ArUco markers in
  view** with `cv2.aruco.generateImageMarker`, placed at the pixel position/scale implied by drone
  pose + `set_camera_angle` tilt + FOV + altitude. So `cv2.aruco.detectMarkers` genuinely decodes them
  → real perception tests (single + multiple + tilted + near-40px-gate).
- `FakeUWBParserThread.get_tag_position(tag_id)`: truth (m) + optional Gaussian noise + a configurable
  `(None,None,None)` dropout. Same method names/signature as the real class.


## CRITICAL: the fake mirrors the sim's SUBSET (so tests catch real-only calls)
`FakeDroneAPI` implements **exactly the sim surface** (see docs/RECONCILIATION.md §1) and **does NOT**
implement `set_app_mode/send_app_heartbeat/set_velocity_level/stop_manual_control/arm/disarm/disconnect/
get_velocity/get_drone_id/enable_battery_failsafe`. So mission code that calls those unguarded will
**fail** the fake — which is the point: it forces the `runtime/sdk_compat.py` hasattr-guards. `Obstacles`
has **no `up`** field. `connect(ip)` requires an ip and returns a truthy CommandResult; `hover` needs a
duration. Provide a fixture that toggles a "real-like" fake (extra methods present) so the compat shim's
real path is also covered.

## Conventions
- Tag every test with its phase marker (`@pytest.mark.p3`, ...). `@pytest.mark.integration` = needs the
  **real sim** → excluded from all gates and the overnight loop (implement, but a human runs them).
- **No network. No sleeps longer than necessary** (drive the kinematic model by stepping, not wall
  time, where possible). Deterministic under the seed. Keep tests fast (the suite runs every gate).
- Put shared synthetic-frame / marker-rendering helpers in `tests/fakes/` so perception + lock-on
  tests reuse them.
