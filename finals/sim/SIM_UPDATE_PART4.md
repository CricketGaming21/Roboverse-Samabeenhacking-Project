# Hula Sim — Update: offscreen run recording (faithful MP4, no live window)
## Build spec for Claude Code (continues SIM_UPDATE_PART3.md; Phases 22–23)

> Adds a **recording** of the real run: a watchable MP4 captured offscreen, so the live-GUI freeze on
> WSLg is irrelevant. Same rules: phase by phase, tests pass headless, commit (§8), STOP and report.
> Existing tests stay green, DIRECT+EGL default, all PyBullet on the sim thread, config in BOTH
> `sim_config.yaml` AND `simcore/config.py` (loader rejects unknown keys).

---

## 0. Why this is faithful (the whole point) and why it can't freeze

The interactive `p.GUI` window is what hangs on WSLg, because a live window and an offscreen
`getCameraImage` contend for the display pipeline. This tool **opens no window**. It captures the run
the way the drone cameras and referee already render — **offscreen via EGL (`ER_BULLET_HARDWARE_OPENGL`)
in DIRECT mode** — which is the exact path all 170 tests use and which never freezes.

Fidelity guarantees (must hold, and be tested):
- The recorder runs **inside the real headless `scenario_demo` run** — the same process, the same tick
  loop, the same physics, the same `cv2.aruco` scoring. It is a **passive observer** that renders extra
  offscreen frames of the live world; it does **NOT** re-simulate and does **NOT** change sim behaviour.
- There is **only one simulation**. The video shows that run. The video's final scoreboard **equals** the
  run's printed scoreboard (assert it).
- Deterministic: same seed → same run → same video.
- Observer-only: reads `simcore` state, renders offscreen; adds **nothing** to the public `pyhulax`/UWB
  surface and does not alter scoring/scenario/convoy logic.

So this is a recording of what actually happened, captured by the real renderer — not a redraw, not a replay-from-seed.

---

## 1. Phase plan

### Phase 22 — Offscreen 3D arena recording → MP4 (the fidelity-critical core)
- **Deliverable:** a `--record PATH.mp4` flag on `scripts/scenario_demo.py` (and `run_sim.py`) that, during
  the **real headless run**, captures a **third-person 3D view of the whole arena** each video frame and
  writes an MP4.
  - **Camera:** a fixed angled-overhead pose framing the entire arena (sees all 3 drones, the convoy, pads,
    obstacles, archway), via offscreen `getCameraImage` with `ER_BULLET_HARDWARE_OPENGL` (EGL), issued
    **on the sim thread** through the existing registry queue (same as the drone cameras). Pose/resolution
    in config (`record.camera_height_m`, `record.camera_angle_deg`, `record.width`, `record.height`).
  - **Frame rate:** capture at a target video fps (config `record.fps`, default 30) by sampling **sim-time**
    (render a frame every `1/fps` sim-seconds), NOT every physics tick — keeps render cost and file size sane.
  - **Info overlay (drawn on each frame with cv2):** scenario phase (DEPLOY/AMBUSH/DONE), sim-time,
    Part-1 landings scored, Part-2 distinct rover ids banked, and the running score. So the video is
    self-describing.
  - **Writer:** `cv2.VideoWriter` (mp4v) is fine; if the codec is unavailable, fall back to a PNG
    sequence in a folder + an ffmpeg/imageio stitch (or just leave the PNG sequence). Whatever's robust headless.
  - **NO `p.GUI` anywhere in this path** — offscreen EGL only. Off by default (no `--record` → zero overhead).
- **Files:** `simcore/recorder.py` (the capture+overlay+writer), `registry.py` (offscreen third-person
  render entry on the sim thread, reusing the guarded renderer path), `scripts/scenario_demo.py` +
  `run_sim.py` (`--record` flag), `config.py` (+`record` block), `sim_config.yaml`.
- **Acceptance test (headless):**
  - a short `--record` run writes a video file (or PNG sequence) with roughly `episode_seconds × fps`
    frames; frames are real renders (non-blank, contain arena geometry — assert non-trivial pixel variance).
  - **Fidelity:** the scoreboard the recorder overlays/finishes with **equals** the run's printed
    scoreboard; a recorded run and a plain headless run with the same seed produce the **same** scoreboard
    (the recorder doesn't perturb the sim).
  - the record path uses DIRECT+EGL and **never connects `p.GUI`** (assert the connection mode / that no
    GUI client is created).
  - recording is gated by the flag: without `--record`, no capture, existing tests unaffected and green.
- **Manual check:** `python -m scripts.scenario_demo --record run.mp4` → produces `run.mp4`; the full
  two-phase run is watchable (drones fan out and land, convoy enters and winds the loops, score climbs).
- **Commit:** `feat: offscreen 3D arena recording to MP4 (faithful capture of the real headless run; no GUI window)`.

### Phase 23 — Composite per-drone camera feeds into the recording
- **Deliverable:** extend the recorder so the MP4 also shows **what each drone sees** — composite the three
  per-drone camera frames (the same frames the referee judges) as a **row of labelled insets** along the
  bottom (or side) of the 3D arena frame, each with detected **ArUco markers outlined + id labelled**
  (reuse Phase-21 `annotate_markers`). So one video shows: the 3D arena (what's happening) + each drone's
  live view (what it sees) + the overlay (the score). Insets toggle via `record.show_camera_insets` (default on).
  - Camera frames captured offscreen on the sim thread (same guarded EGL path); resized for the insets.
  - Keep it robust: if a drone isn't flying yet (DEPLOY pre-takeoff), show a placeholder tile, not a crash.
- **Files:** `simcore/recorder.py` (compositing + insets), reuse `camfeed.annotate_markers`, `config.py`
  (+`record.show_camera_insets`).
- **Acceptance test (headless):** the composited frame contains the 3D view plus three labelled camera
  insets; an inset shows a marker outline + id when that drone has a marker in view (drive a known pose);
  placeholder tiles render when a drone isn't flying; the scoreboard-equality fidelity check still holds;
  no `p.GUI`; existing tests green.
- **Manual check:** `scenario_demo --record run.mp4` → the video shows the arena up top and the three drone
  feeds below, with a rover's marker getting outlined in a drone's inset as it scans.
- **Commit:** `feat: composite per-drone ArUco-annotated camera feeds into the recording`.

---

## 2. Notes
- This is purely additive observability. The sim, scoring, convoy, control, and public API are untouched.
- It works over a remote connection (no display server, no window) — you generate `run.mp4` on the VM and
  download/scp it to watch, or play it wherever.
- It doubles as a debugging tool for the mission phase: record any mission run and review exactly what happened.
- The live `--gui`/`--dashboard` paths still exist for anyone on a native display; they are simply not the
  way to watch on this WSLg box. Recording is.
