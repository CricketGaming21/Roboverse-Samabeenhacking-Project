# RoboVerse 2026 Finals — Claude Context (Challenge 2)

> Context bridge for the **mission-code** repo. Read this before any task.
> This file reflects the *corrected* understanding of the Hula platform after a
> full read of the pyhulax SDK, the UWB parser, and the Finals briefing. Where it
> contradicts older notes or the qualifier, **this file wins.**

## Competition
- RoboVerse 2026 Finals — **Pre-University category**.
- We do **Challenge 2 only** (the mapping drone / Challenge 1 is University-only — not us).
- Prize: 1st $1800 / 2nd $1300 / 3rd $900.

## Challenge 2 — two phases
### Phase 1 — Landing
- Launch **3 HULA drones** from the C2 terminal.
- Land accurately on **3 valid landing zones**, each marked with an **ArUco marker**.
- Landing-zone coordinates are **provided by the organisers** (from the mapping step). Treat them as a known input — we do not have to discover them by exploring.
- Scored on **landing accuracy + speed**.

### Phase 2 — Search and scan
- **5 RoboMaster ground rovers** loiter in the arena for a period.
- The 3 HULA drones search for them and **capture them on camera**.
- **Each rover carries a unique ArUco marker on top** (confirmed for the real event). So "snapshot the rover" = **read its ArUco marker** — detection is ArUco-first, not object-detection-first.
- Scored on **number of distinct rovers successfully captured + time**.

## Finals brief — confirmed facts + what the sim now models (Phases 31–34)
> From the official Finals brief; the `hula_sim` now models all of this, so mission
> code is developed + tested against it. Items marked **PROVISIONAL** come from
> Discord/organisers (not yet in the brief) — **parameterise, never hard-bake.**

- **0.5 m/s HARD speed cap.** Every `VelocityLevel` maps to **≤ 0.5 m/s** (MEDIUM is
  the usable max; ZOOM/TURBO clamp down to it). The enum names/values and the public
  API are unchanged — only the mapped speed clamps. Recommended flight height
  **1.1 m**. Plan for slow, deliberate flight.
- **No flying OVER obstacles + an altitude cap → score INVALIDATED.** Crates are
  ~1.2 m. **Route AROUND crate footprints** and stay under the altitude cap (~2.0 m
  PROVISIONAL). The sim FLAGS a violation — visible in `--record` / `--dashboard`,
  in the `DebugProbe` snapshot (`compliance`), and `logs/compliance` — a *flagged*
  event, **not a crash**, so you can see it and fix the nav (no silent run loss).
- **Arena coords are GIVEN as a data file.** The sim emits **`arena_truth.yaml`**
  (arena dims + every crate `center/size/height` + archway) — plain data the mission
  loads with `yaml.safe_load`, **never importing `simcore`** (it mirrors the
  organisers' Discord coordinate file). PROVISIONAL until Discord; regenerate from
  the sim with `python -m scripts.emit_arena_truth`.
- **Drones are obstacles to each other.** `get_obstacles()` (the same five booleans)
  trips on a nearby drone at the same IR range as crates. Keep 3-drone separation at
  a common ~1.1 m height (no altitude layering needed — just watch the barrier flags).
- **Phase-2 rovers: 5 total = 3 AUTONOMOUS + 2 HUMAN-TELEOPERATED opponents.** The
  opponents carry a **separate ArUco id block** (sim: `[30,31]` vs autonomous
  `[20,21,22]`) and are adversarial — they **flee** the nearest drone, seek **crate
  cover**, and **juke**. Detection/lock-on must handle evasive targets; **all 5
  distinct ids count** for Stage 2.
- **Stage-1 scoring = land INSIDE THE HOOP** of a chosen valid pad (`hoop_radius_m`,
  PROVISIONAL) + time. **Stage-2 = distinct ArUco ids across all 5 rovers** + time.
- **~8 min per stage**; **no re-assessment on crash.** Budget search/coverage to fit.
- **Drone count is OPEN — design for 1 OR 3.** Brief logistics say **1** HULA for
  Pre-U; Challenge 2 says launch **3**. Keep drone count a **config knob** — never
  hardcode 3 (the sim runs the scenario with both).
- **ArUco dictionary:** assume **`DICT_6X6_250`** (confirm at the briefing).

## THE PLATFORM REALITY (read carefully — this is what changed)
The HULA (HG-Fly F09-lite) is a small indoor edu-drone driven by the **pyhulax** SDK. It is **not** a PX4/MAVSDK platform and **not** a depth-camera platform.

### Control — pyhulax, command-based and BLOCKING
- `takeoff(height_cm=100)`, `land()`, `hover(seconds)`
- `move(Direction, distance_cm, speed=VelocityLevel.ZOOM)` — relative step along the **current heading** (body frame)
- `rotate(angle_degrees)` — +CCW / −CW yaw
- `move_to(x, y, z)` — straight line to a point in the **takeoff-origin frame, in cm** (x right, y forward, z up)
- There is **no MAVSDK-style velocity offboard.** Commands block until done (or `blocking=False`).

### Position — two frames, do not conflate
- **UWB (arena frame):** `UWBParserThread.get_tag_position(tag_id) -> (x, y, t)` in **metres**, **X-Y only (no Z)**, one tag per drone. This is the arena-absolute, drift-free truth (with small noise). Origin/orientation set by the anchors.
- **Onboard estimate (takeoff frame):** `get_position()` returns cm in the takeoff-origin frame and **drifts** (optical-flow/IMU). Use UWB to correct it.
- **Altitude** comes from the downward **ToF**: `get_altitude()` (cm). **Never from UWB.**
- Other telemetry: `get_orientation()` (yaw/pitch/roll), `get_battery()`, `get_state()`.

### Obstacles — barrier flags only, reactive avoidance
- Sensing is coarse **IR/ToF barrier sensors in 5 directions**: `get_obstacles() -> Obstacles(forward, back, left, right, down)` (booleans), `any_obstacle()`, `get_drone_status()` (bitmask: 0 fwd,1 back,2 left,3 right,4 down).
- Built-in reactive avoidance: `set_barrier_mode(enabled)` (firmware auto-avoid) and `set_avoidance_direction(direction, distance_cm, barrier_mask)` (detect → step away).
- **There is NO depth image, NO point cloud, NO lidar, NO map.** Avoidance is anti-bump reflex + conservative coverage. Sensors need a minimum altitude (~0.35 m) to work.

### Camera — monocular, tiltable
- One main **front camera with controllable pitch**: `set_camera_angle(CameraPitchMode, angle)` (0° = forward … 90° = straight down), plus a downward optical-flow camera. `VisionMode.OPTICAL_FLOW` / `FRONT_CAMERA`.
- Video: `create_video_stream()` → `set_video_stream(True)` → `latest_frame.to_rgb()` (RGB `np.ndarray`). Run `cv2.aruco` on these frames.
- pyhulax also has built-in QR/digit/arrow recognition, but those target HG's own fiducials — **we use our own `cv2.aruco`**, not pyhulax's QR features.

## What does NOT carry over from the qualifier (do not reuse)
- **Depth → point cloud → RRT\* / GlobalMapper / PointCloudPlanner / depth_to_xy_map** — depends on a depth camera the Hula does not have. **Challenge 1 only. Do not port.**
- **RealSense / pyrealsense2** code — that's the mapping drone. Not on the Hula.
- **MAVSDK / PX4 / offboard velocity / Gazebo** — wrong control stack for the Hula.
These files exist in `reference/` for history; they are **not applicable to Challenge 2**.

## What DOES carry over (reusable thinking)
- **Committing-waypoint executor:** hold a single target until reached, replan only on real events. (Our qualifier failure was reactive velocity commands overwriting each other every 0.1 s — do not repeat that.)
- **Detection → lock-on → capture** pattern, and the overall mission structure.
- **Config-driven values** (no hardcoding).

## Hardware
- 3× Highgreat HULA drones, one UWB tag each (arena north-east position).
- C2 Terminal: Windows laptop hosting an Ubuntu 22.04 VM; pyhulax connects to the drones over Wi-Fi (use `Dola` discovery to find drone IPs, then `DroneAPI().connect(ip)`).

## Detection & scoring
- **Primary: ArUco** (`cv2.aruco`) on the front/down camera frames — pad markers (Phase 1) and unique rover markers (Phase 2).
- A "successful capture" = marker detected, ID decoded, and a stable/close enough read (size + in-frame + held a few frames).
- **YOLO / RKNN is a contingency only** — if any target turns out to be a bare robot. The YOLO subgroup trains it separately; the NPU (~50 fps via RKNN) is the deployment path if needed. Keep detection behind one interface so ArUco ↔ YOLO is a swap.

## We develop against a SIMULATOR first
- A separate **`hula_sim`** project provides a drop-in `pyhulax` package, a drop-in `UWBParserThread`, and a PyBullet world (room, ArUco pads at given coords, 5 moving ArUco-tagged rovers, barrier sensors, scoring, top-down view).
- **Mission code imports `pyhulax` / `UWBParserThread` unchanged** and runs against the sim for development, then against the real SDK on the day. The swap is just which `pyhulax` is on the path.
- **Mission code must never depend on anything sim-specific** — only the public pyhulax/UWB API.
- The sim now models the full Finals brief (Phases 31–34 above): the **0.5 m/s cap**, the **mixed convoy** (3 autonomous + 2 evasive/teleoperated rovers), **compliance flagging** (no-fly-over-crate + altitude cap), **inter-drone barrier sensing**, **hoop / distinct-id scoring**, and the **8-min stage clock**. Arena ground truth is the **`arena_truth.yaml`** data file (loaded as plain data, not via `simcore`). The sim ships a YOLO seam (`mission_examples/rover_detection_example.py`) showing where a mission-side YOLO plugs in behind the ArUco-first detector.

## Coordinate-frame & gotcha checklist
- `move` = body / current heading. `move_to` + `get_position` = fixed takeoff frame (cm). UWB = arena frame (m), **no Z**.
- Altitude = ToF (`get_altitude`), never UWB.
- `get_position` drifts; UWB does not. Correct the estimate with UWB.
- Avoidance is reactive barrier flags only — never assume a map or a planned route around an unseen obstacle.

## Mission repo layout
```
finals/
  mission/        phase1_land.py, phase2_search.py, mission_runner.py (swarm orchestration)
  control/        hula_control.py (pyhulax wrapper), uwb_handler.py (UWB + correction),
                  avoidance.py (reactive barrier-flag avoidance)
  detection/      aruco.py (PRIMARY, cv2.aruco), lockon.py (centering/approach),
                  snapshot.py (capture + dedup), detector.py (YOLO/RKNN — contingency)
  utils/          config.py (ALL tunables — read before hardcoding), logger.py
  reference/      qualifier_code/ (mostly NOT applicable — see above), sample_code/,
                  project_details/, learning_materials/, hardware_docs/
  weights/        YOLO files (only if a bare-robot fallback is needed; gitignored)
  claude_debug/   logging + run scripts
```

## Rules for Claude Code — read before touching anything
1. Read this file (and the root CLAUDE.md if present) before starting any task.
2. **Detection is ArUco-first.** Don't reach for YOLO unless a target is confirmed bare.
3. **No depth / point-cloud / RRT\* / RealSense / MAVSDK code** in Challenge 2 — those don't run on the Hula. Don't port qualifier avoidance.
4. Read `reference/` before writing code, but treat the depth/RealSense/qualifier-planner material as historical, not a template.
5. All tunable values go in `utils/config.py` only — no hardcoding.
6. Test against the `hula_sim` simulator; keep mission code on the public pyhulax/UWB API only.
7. New code goes in the correct subfolder above.
8. Commit format: `feat/fix/docs/chore: what changed and why`. After a working change: `git add -A && git commit -m "..."` (auto-push hook handles the push).
