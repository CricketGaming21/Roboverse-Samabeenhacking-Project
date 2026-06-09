# PROGRESS — updated by Claude Code at every phase boundary

> Append one block per phase. Keep newest at top. A human waking up can see status in 10 seconds.

| Phase | Status | Tests (gate / full) | Commit | Notes |
|------|--------|---------------------|--------|-------|
| P0   | ✅ GREEN | 41 / 41            | (this) | fake SDK substrate |
| P1   | TODO   | – / –               | –      | not started |

## Reference-source note (read once)
The authoritative `reference/pyhulax_knowledge_base.txt` and `reference/brief/Finals_brief.pdf`
are **NOT present** in this repo (only `reference/provided_code/` is vendored, and the `brief/`
+ `sim/` dirs hold just `.gitkeep`). SDK signatures were therefore verified against
`docs/SDK_REFERENCE.md` + `docs/RECONCILIATION.md` (which state they were reconciled against the
real sim and the KB) and the vendored `provided_code/` (`UWBParserThread.py`, `dola.py`,
`rover_detection_example.py`). No disagreements were found between those and what the docs assert.
**[TO CONFIRM]** when the KB/brief PDFs are dropped in: re-verify the enum integer values
(`Direction`, `CameraPitchMode`, `VelocityLevel`, `BarrierMask`) and the exact `connect`/`hover`
signatures — the fake encodes the doc's values.

## Log

### P0 — Test substrate (fake SDK)  ✅
Built `tests/fakes/fake_pyhulax.py`, `tests/fakes/fake_uwb.py`, `tests/conftest.py`,
`tests/test_fakes.py`. The fake mirrors the sim's **SUBSET** exactly: `connect(ip)` (required ip,
truthy `CommandResult`), `takeoff/land/hover(duration)/move/rotate/move_to/send_manual_control/
manual_fly/get_state/get_position/get_orientation/get_altitude/get_battery/get_obstacles/
any_obstacle/get_drone_status/set_barrier_mode/set_avoidance_direction/set_camera_angle/
create_video_stream/set_video_stream`. The real-SDK-only methods (`set_app_mode`,
`send_app_heartbeat`, `set_velocity_level`, `stop_manual_control`, `arm/disarm/disconnect`,
`get_velocity`, `get_drone_id`, `enable_battery_failsafe`) are **deliberately absent** — a
parametrized test asserts their absence, which is what will force the P1 `sdk_compat` shim.
`RealLikeFakeDroneAPI` adds them so the shim's hardware path is also testable. `Obstacles` has
no `up` field (asserted).

Kinematics are **time-stepped** (each `send_manual_control` advances a shared sim clock by
`world.dt=0.05` and integrates exactly that step) so tests are deterministic with no real sleeps;
the horizontal speed is hard-clamped to `max_mps=0.5` (resultant magnitude, asserted). UWB stamps
its samples with the same sim clock so a Δpos/Δt speed estimate (P1) is reproducible.

The camera renders **real** `cv2.aruco` markers via a correct pinhole projection +
`warpPerspective`. One subtlety hit and fixed: the floor-corner→canvas-corner correspondence
produced an orientation-reversing homography (a *mirrored* marker, which `cv2.aruco` refuses to
decode). Added a winding-match (`_signed_area`) that flips the destination quad when needed — now
nadir and offset frames genuinely decode to the right ids.

Integration tests are skipped unless `RUN_INTEGRATION=1` (documented mechanism, not weakening).
`conftest.py` also prepends `src/` to `sys.path` (the `mission` package is not pip-installed),
seeds `random`+`numpy`, and installs the fakes into `sys.modules` so `import pyhulax` /
`from UWBParserThread import UWBParserThread` resolve to them.
