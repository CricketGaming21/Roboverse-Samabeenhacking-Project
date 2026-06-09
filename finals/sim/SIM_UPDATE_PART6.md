# Hula Sim — Update: visibility polish, web dashboard, YOLO seam
## Build spec for Claude Code (continues SIM_UPDATE_PART5.md; Phases 28–30)

> Same rules: phase by phase, tests pass headless, commit (§8), STOP and report. Existing tests stay
> green, DIRECT+EGL default, all PyBullet on the sim thread, config in BOTH `sim_config.yaml` AND
> `simcore/config.py` (loader rejects unknown keys). Observer/viz + tooling only — the public
> pyhulax/UWB surface stays frozen; scoring/scenario logic unchanged unless stated.

These are the **last polish phases before mission coding.** Context: the operator works **remotely over
SSH** (PC-B → VM via Tailscale), so any **window** opens on the desktop's screen, not theirs — that's why
the dashboard must be **web-based** (browser, served over HTTP), not a native window.

---

## Phase 28 — Sharper detection box (HQ) + realistic rover loiter
- **Deliverable A — make the detection/acquisition box clearly visible:**
  - **Bolder box regardless of resolution:** thicker outline (≥3–4 px scaled to tile size), high-contrast
    colours (bright yellow DETECTED / bright green ACQUIRED), a larger id label with a semi-opaque
    background pill so it's legible against any frame. The box must be unmissable at the normal record size.
  - **Higher-resolution camera insets / recording:** render the per-drone camera insets at a higher
    internal resolution so the box + marker are crisp, and allow a high-quality record mode
    (`record.width/height` settable up to 1920×1080, e.g. a `--hq` flag or config) — but keep a sane
    default (HQ is heavier on file size/time). The insets should look sharp even at the default frame size.
- **Deliverable B — realistic rover loiter (fix the "spastic" end behaviour):**
  - The rovers convoy in nicely but go erratic toward the end — the **loiter / end-of-branch** behaviour
    snaps or oscillates. Make loiter a **smooth, continuous, realistic** motion: the rover eases around its
    loop path repeatedly with **no teleport/snap at the loop seam** and **no oscillation**, at the slow
    convoy speed — like a vehicle calmly patrolling, similar to the real day. (If `loiter: loop`, smooth
    the wrap-around; if `hold`, ease to a gentle stop. No discontinuities.)
- **Files:** `simcore/recorder.py` / `camfeed.py` (box thickness/colour/label, inset resolution),
  `config.py` + `sim_config.yaml` (record HQ size, box style), `rover_model.py` (smooth loiter).
- **Acceptance test (headless):** the box outline thickness/contrast meet a minimum (assert line width +
  colour values); the id label renders with its background; insets render at the configured (higher)
  resolution; HQ record size is honoured. Rover loiter is **smooth**: assert the loiter path has no
  position discontinuity above a small threshold between frames (no snap), velocity stays ~convoy speed,
  and there's no oscillation (no rapid direction reversals). Existing tests green.
- **Manual check:** `scenario_demo --record run.mp4` (or `--hq`) — the DETECTED/ACQUIRED box is clearly
  visible; rovers crawl their loops smoothly to the end with no spastic jitter.
- **Commit:** `feat: high-visibility detection box + HQ record option + smooth realistic rover loiter`.

### Phase 29 — `--gui` → 3D-only safe view + graphical WEB dashboard (SSH-friendly)
- **Deliverable A — fix `--gui` to the 3D-only safe view:** make `--gui` run the **camera-disabled
  3D-only mode** (the Phase-27 `live_view` behaviour: `cameras_enabled=False` — no referee scanning, no
  insets, no drone-camera renders) so it shows "just the 3D space" and **cannot** hit the
  `getCameraImage`-vs-GUI freeze. (It still opens a window on the host's display — fine at the desktop;
  not visible over SSH, which is expected — that's what the web dashboard below is for.) The old
  full-GUI-with-cameras path is removed/redirected, not left as a freeze trap.
- **Deliverable B — graphical web dashboard (the SSH-friendly monitor):** a `--dashboard` flag (now
  **web-based**) starts a small **HTTP server** in the headless run that serves a **live graphical
  dashboard** viewable in a browser on PC-B:
  - **Server:** binds `0.0.0.0` on a config port (`dashboard.port`, default 8080); a lightweight Flask or
    stdlib `http.server`. Started alongside `scenario_demo` / `run_sim`; off unless `--dashboard`.
  - **`GET /`** → an HTML/JS page that polls a JSON endpoint (~3–5 Hz) and renders, **per drone**, a
    graphical panel: telemetry (speed, UWB pos **and** drifting estimate, heading, camera pitch,
    yaw/pitch/roll, manual sticks or current command, battery), a **UWB-OK** badge, and a **car-style
    proximity widget** (the five directional segments, drawn in SVG/canvas, lit when the barrier flag is
    set — boolean only). Plus the scoreboard/phase banner.
  - **`GET /api/state`** → JSON from `DebugProbe.snapshot()` (read-only). 
  - **Optional but recommended — live visuals over HTTP:** `GET /api/arena.jpg` and
    `GET /api/camera/<i>.jpg` serving the **latest offscreen-rendered** arena + per-drone frames (EGL,
    sim thread — same path as the recorder, NO `p.GUI`), embedded in the page and refreshed periodically,
    so the operator gets a near-live 3D arena + camera feeds **in the browser over SSH**.
  - **Boundary:** reads `simcore`/`DebugProbe` only; renders offscreen only; does **not** perturb the sim
    (fidelity holds) and adds nothing to the public pyhulax/UWB surface.
  - **Access (document in README):** from PC-B either browse `http://<tailscale-ip>:<port>` directly, or
    SSH-forward: `ssh -L <port>:localhost:<port> drone@<host> -p 2222` then `http://localhost:<port>`.
- **Files:** `scripts/webdash.py` (server + HTML/JS), `scripts/scenario_demo.py` + `run_sim.py`
  (`--gui` → 3D-only, `--dashboard` → web), `simcore/debug.py` (JSON-serialise the snapshot),
  `registry.py` (reuse offscreen arena/camera render for the image endpoints), `config.py` + `sim_config.yaml`
  (`dashboard.port`, toggles), README (access instructions).
- **Acceptance test (headless, no window/browser):** `--gui` mode disables all camera rendering (assert
  no camera-render path active → freeze-proof, like the Phase-21b/27 guards); the dashboard server starts
  and `GET /` returns the page and `GET /api/state` returns JSON with the per-drone telemetry + proximity
  fields matching `DebugProbe.snapshot()`; the image endpoints (if built) return a valid JPEG from an
  offscreen render; the server is read-only (a probe-invariance / scoreboard-equality check still holds);
  nothing connects `p.GUI`; nothing added to the public surface. Existing tests green.
- **Manual check:** `scenario_demo --dashboard` on the VM, then open `http://<tailscale-ip>:8080` (or the
  SSH-forwarded localhost) in the PC-B browser — live per-drone telemetry + proximity widgets update as the
  run progresses (and, if built, the arena + camera images refresh).
- **Commit:** `feat: --gui as freeze-proof 3D-only view + graphical web dashboard (SSH/browser, offscreen, no freeze)`.

### Phase 30 — YOLO integration seam (mission-side) + detection-stack docs
- **Deliverable:** a clearly-labelled **YOLO integration seam** so the operator can drop their model in,
  **without** putting YOLO in the sim core (boundary intact):
  - An example/harness (e.g. `mission_examples/rover_detection_example.py`) that, against the **public
    pyhulax API**, pulls a drone's latest camera frame (`create_video_stream` → `latest_frame.to_rgb()`),
    passes it to a **`RoverDetector` interface** (`detect(frame) -> list[Detection]` with id/bbox/conf),
    and shows the intended **two-stage flow**: YOLO finds a rover from afar → approach → `cv2.aruco`
    confirms identity up close. Ship a **dummy/placeholder detector** (clearly marked `# PLACEHOLDER —
    plug your YOLO model here`, e.g. returning a trivial/centre box) so the pipeline runs and is testable
    **without** a real model; the operator swaps in their YOLO (ultralytics etc.) on the mission side.
  - **Docs (`docs/DETECTION.md` or README):** state the split plainly — **ArUco is REAL in the sim**
    (`cv2.aruco` runs on rendered frames for scoring + the detect→acquire boxes; mission runs the same on
    the same frames); **YOLO is MISSION code** run on the sim's frames; the sim's job is to **render
    detectable RoboMaster-style rovers + serve frames** (Phase-21 visuals); the example shows the seam.
  - **Boundary:** the seam imports the **public** pyhulax API only; YOLO/ultralytics is **NOT** imported
    into `simcore`; the placeholder is obviously a placeholder.
- **Files:** `mission_examples/rover_detection_example.py` (seam + dummy detector + `RoverDetector`
  interface), `docs/DETECTION.md`, optionally a tiny `simcore`-free helper for frame access if useful.
- **Acceptance test (headless):** the example pulls a frame via the public VideoStream and calls the
  detector interface; with the dummy detector it returns detections and the example runs end to end; the
  two-stage pattern is exercised (YOLO-stub box → ArUco confirm via the real `cv2.aruco`); assert
  **no YOLO/ultralytics import in `simcore`** (boundary); the public surface is unchanged. Existing tests green.
- **Manual check:** run the example against the sim — it prints detections from the placeholder and the
  ArUco confirmation, demonstrating where the real YOLO plugs in.
- **Commit:** `feat: YOLO integration seam (mission-side placeholder + RoverDetector interface) + detection-stack docs`.

---

## Notes
- Web dashboard is freeze-immune **by construction**: headless DIRECT+EGL process, all rendering offscreen,
  HTTP serves data — no `p.GUI` window anywhere. It reaches PC-B because it's a network service, not a window.
- `--gui` (now 3D-only) and `live_view` are for when physically at the desktop; over SSH, use `--record`
  (cockpit MP4) and `--dashboard` (web).
- YOLO stays mission-side — the sim provides detectable rovers + real frames + a documented seam. ArUco is
  real in the sim. This preserves the SIM/MISSION boundary that's held for 27 phases.
- Public pyhulax/UWB surface frozen throughout. Real specs unchanged (FOV 71°, ±20 cm drift, 0.5–1.0 m/s, etc.).
