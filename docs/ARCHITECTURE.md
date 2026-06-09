# ARCHITECTURE.md

Mission code for 3 HULA drones, Challenge 2. Runs on the **C2** (one process) against the **public
pyhulax + UWBParserThread** API. Thread-per-drone; everything coordinates through one shared,
lock-guarded world model. Sim-developed; real-deployable by import-path swap.

## Frames (the spine)
- **Arena / UWB (truth):** `x=North(m), y=East(m)`, drift-free, ~5 cm noise, 10 Hz, occasional
  `(None,None,None)`. Source: `UWBParserThread.get_tag_position(tag_id)`.
- **Body (commands):** `forward,right,up`; `send_manual_control` sticks −1..1. We **lock yaw at
  takeoff** (one `YAW_OFFSET`), so arena↔body is a fixed rotation; arena error maps directly to
  forward/right. Yaw is then a *framing* DOF for Phase-2 lock-on only.
- **Onboard estimate (do NOT trust for arena accuracy):** `get_position()`/`move_to` drift (no QR
  mat). We fly our own UWB loop instead.
- **Units:** UWB metres, pyhulax centimetres. Convert at named boundaries.

## Control stack (plan-then-guard)
1. **Offline planner** (`planner/geometry.py`): on the **provided** crate map, inflate footprints
   (~0.4 m), build a visibility graph, route each drone C2→pad and the Phase-2 vantage loops. Paths
   live entirely in inflated free space, so **no-overfly is structural** and nothing nominally trips
   the reactive layer. Authored offline in the **Planner GUI** (P9) → frozen `mission_plan.yaml`.
2. **UWB loop** (`control/uwb_loop.py`, `fly_to_uwb`): closed-loop on UWB error → body velocity,
   **≤0.5 m/s**, altitude held ~1.1 m, hold-on-dropout, ~20 Hz.
3. **Reactive guard** (`control/avoidance.py`): the 5 IR booleans filter every command — stop the
   travel axis, slide to the open side, **never +up**; backtrack+reroute if boxed. Backstop only.
4. **Inter-drone:** disjoint zones (primary) + lateral UWB separation + right-of-way (no altitude
   layers — illegal). Other drones are moving obstacles in the planner + the forward barrier.

## Perception (`perception/`)
Detect-wide / identify-narrow, behind the `RoverDetector` seam (mirrors
`reference/provided_code/rover_detection_example.py`):
- **Stage 1 (find):** `ClassicalRoverDetector` PRIMARY (colour/contour/motion — training-free, works
  day-one; at ~1.1 m the rover fills much of the frame). Optional self-trained **YOLO** behind the
  same seam (Ultralytics/ONNX on the C2 — **never RKNN**), bonus only.
- **Stage 2 (confirm = the score):** real `cv2.aruco` `DICT_6X6_250` on the same frames; multi-marker
  per frame. **Pads = 10–14; any other id = rover. Opponent rovers have DIFFERENT ids — never filter
  to 20–24.** Tagging = reading the top marker's id (a photo alone doesn't score).
- Ground-plane **projection** (`planner/projection.py`) gives a rover's approximate (x,y) from
  pose+altitude+intrinsics — geometry, no depth.

## Shared world model (`world/`) — the brain
One lock-guarded belief-and-tasking state powering search, evaders, and the C2:
- **MissionState:** tagged-id set + per-id evidence (annotated frame, xy, time). Drives "done" + rubric.
- **TaskBoard:** live tracks for every rover seen (proj xy, velocity, behaviour class, last-seen) +
  per-drone role/target assignments.
- **BeliefGrid:** occupancy probability over free cells per un-tagged target — collapses where
  observed, **diffuses** along lanes over time, spikes on sightings. Drives "search where it likely
  is," not a fixed sweep.
- **Chokepoint/lane graph:** from the crate map; drives evader **containment** (hold cut-points to
  shrink the reachable set).
- **Coordinator:** assigns roles (SWEEP / TAG / BLOCK / COVER) from the belief + tracks. This shared
  object + the coordinator **is** the "smart comms" — shared memory, no network protocol.

## Phase strategies
- **Phase 1 (deploy):** dynamic 3-of-5 pad assignment (from given coords) → planned route → land
  **inside the hoop** (UWB-centred, hoop-gated descent, optional decode-to-confirm). Count-first.
- **Phase 2 (ambush):** **persistent vantage patrol / chokepoint overwatch** (not lawnmower — moving
  targets need *frequent re-observation*, and rovers are confined to the same lanes we watch). Lock-on
  is **station-keep over a near-stationary target** (5-frame hold ≈ 0.17 s), bounded/interruptible,
  velocity-matched for movers, time-boxed. **Evaders (2, human-piloted):** triage → secure the 3
  autonomous tags first → belief-grid pursuit + cooperative containment + bait/flush; honest ceiling
  (a skilled human may deny the 5th — count-first means 4/5 still scores well).

## Concurrency & reliability
Thread-per-drone `DroneWorker` (blocking is fine inside a worker); one `Lock` per shared object, held
briefly, never while flying/sleeping. One drone failing must not freeze the others (wrap the loop,
recover/land). Battery + UWB-dropout + worker-death failsafes; **shutdown always lands all drones**.
GIL note: 3 video+detection streams are the real bottleneck → small/classical detector, throttle FPS,
per-worker model or a detection queue.

## Tools (browser, single-file, shared render core)
- **Planner GUI** (P9): offline route + vantage authoring → `mission_plan.yaml` (the contract) + PNG.
- **C2 console** (P10): live map + belief heatmap + camera tiles + rubric/evidence + alarms (incl. the
  no-fly over-footprint alarm) + optional operator overrides.

## Sim ↔ real
Identical mission code; the only change on the day is the `pyhulax` import target and the
on-the-day-calibrated config (`YAW_OFFSET`, stick scale, hoop tolerance, ids, arena). See
`docs/SIM_VS_REAL.md` (created P11).
