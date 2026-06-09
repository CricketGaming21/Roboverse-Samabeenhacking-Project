# PROGRESS — updated by Claude Code at every phase boundary

> Append one block per phase. Keep newest at top. A human waking up can see status in 10 seconds.

| Phase | Status | Tests (gate / full) | Commit | Notes |
|------|--------|---------------------|--------|-------|
| P0   | ✅ GREEN | 41 / 41            | 54444c9 | fake SDK substrate |
| P1   | ✅ GREEN | 26 / 67            | 6b12d91 | config + frames + fly_to_uwb |
| P2   | ✅ GREEN | 19 / 86            | d29f1ec | planner geometry + projection |
| P3   | ✅ GREEN | 13 / 99            | (this) | reactive avoidance guard |
| P4   | TODO   | – / –               | –      | not started |

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

### P3 — Reactive avoidance guard + plan-then-guard  ✅
Built `src/mission/control/avoidance.py`: `ReactiveGuard.filter(cmd_fwd, cmd_right, obstacles, *,
open_side_hint=None) -> (fwd, right, up)` with `up` hard-wired to **0.0** (never climbs — HARD
invariant #3), stops the blocked travel axis, slides toward the open lateral side with
anti-oscillation **commit hysteresis**, and reports `boxed()` when surrounded. `separation(my_xy,
others, my_priority, sep_m)` implements right-of-way (lower priority yields). Wired into
`fly_to_uwb` via `guard=`, passing a goal-aware `open_side_hint` (sign of the body-right velocity)
so the slide naturally heads toward the goal. An exhaustive test sweeps all 2^5 obstacle combos ×
command grid asserting `up == 0`; the integration test flies a guarded drone past a *surprise*
crate (unknown to the planner) and proves it reaches the goal, never enters the footprint interior,
and never commands +up.

### P2 — Crate map + planner geometry + projection  ✅
Built `src/mission/planner/arena.py` (loads `config/arena_truth.yaml` with `yaml.safe_load`,
**no simcore** — HARD invariant #10), `src/mission/planner/geometry.py` (`inflate` →
axis-aligned `Rect`s; `segment_clear` via Liang-Barsky against slightly-shrunk rects so
boundary-tangent edges are allowed but interior crossings are rejected; `build_graph`
visibility graph over pushed-out footprint corners; `plan_path` A* with start/goal as temp
nodes; `paths_conflict` via segment-segment distance), and `src/mission/planner/projection.py`
(pinhole `arena_to_pixel`/`pixel_to_arena`, exact inverses, **no depth**). Tests prove paths
never cross an inflated footprint or leave bounds, `None` when goal is blocked / arena is walled
off, the real `arena_truth` routes cleanly, conflict detection flags crossing/too-close paths,
projection round-trips to 1e-6, and `pixel_to_arena` recovers a **rendered** marker's known arena
position to within 0.1 m (ties projection.py to the fake camera model — they share the pinhole).

### P1 — Config + frames + UWB control loop  ✅
Built `src/mission/config.py` (pydantic, every section `extra="forbid"` → unknown keys raise;
a validator enforces `speed.max_mps ∈ (0, 0.5]` as the HARD cap), `src/mission/frames.py`
(`arena_to_body`/`body_to_arena` as exact-inverse rotations, `m_to_cm`/`cm_to_m`, `clamp_speed`),
`src/mission/runtime/sdk_compat.py` (`prepare_manual_control`/`release` route real-only calls
through `hasattr` — no-op on the sim fake, real on `RealLikeFakeDroneAPI`), and
`src/mission/control/uwb_loop.py::fly_to_uwb` (P control on UWB arena error → body sticks, hard
clamp ≤0.5 m/s, ToF altitude hold, **hold-on-dropout**, UWB-derived arrival speed Δpos/Δt). The
loop is deterministic in tests via an injectable `sleep` (no real waits) and the time-stepped
fake. Converges from 5 start/target poses incl. corner-to-corner and under 3 cm UWB noise; a
test proves per-step speed never exceeds 0.5 m/s and another proves a horizontal freeze on UWB
dropout. Frame transforms are inverse-consistent and match hand-worked yaw-0 / yaw-90 cases.

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
