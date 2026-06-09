# RULES_AND_CONSTRAINTS.md — what binds the code (from Finals_brief.pdf; brief wins)

## Hard rules (enforce in code + test)
- **HULA ≤ 0.5 m/s** commanded, everywhere (Slide 6, "Other rules").
- **Strictly no flying over obstacles, any height. Recommended ≈ 1.1 m. Violation → scores
  invalidated** (Slide 6). Route around footprints; avoidance never climbs.
- **Land INSIDE the hoop** (Slide 9 "landings within hoop"). Tolerance = measured hoop radius minus a
  UWB-jitter margin. `[TO CONFIRM]` hoop size (measure the sample pad).
- **Tagging = read the rover's TOP ArUco id.** A photo alone does not score. Print/annotate the
  output + inform the judge (Slide 7).
- **5 rovers: 3 autonomous + 2 human-piloted by opposing teams** (Slide 7, 14). The 2 actively evade.
  **They carry DIFFERENT marker ids — never hard-filter to 20–24. Pads = 10–14; any other id = rover.**
- **Crash = no re-assessment, for any reason** (Slide 18). **Marshal-violation = 1 h penalty.**
  → conservatism + failsafes + always-land-on-shutdown.
- **Phase-1 cap = 8 min** (Slide 6). Phase-2 loiter window `[TO CONFIRM]`.

## Scoring (Pre-University, Slide 9) — COUNT first, then TIMING
- Landings within hoop + timing = **44%**; ArUco detections + timing = **44%**; CUAS showcase bonus
  **4%**; concept explanation **8%**. Priority is in sequence (count, then time).
- ⇒ **Never trade a landing or a tag for speed.** Maximise count; timing only breaks ties at equal
  count. "Fast" here = clean routing + minimal dwell + true 3-way parallelism + stop the instant done.
- ⇒ The **12% bonus is nearly free** — do the CUAS Tech Showcase task and write the concept doc.

## Environment / scope
- C2 = Windows laptop + Ubuntu 22.04 VM; all 3 streams + detection on this one machine.
- Libraries: pyhulax + dola + UWBParserThread + OpenCV (+ optional Ultralytics). **No ROS2, no MAVSDK,
  no RKNN, no RealSense/depth.** UWB = metres; pyhulax = cm.
- Bench logistics (Pre-Uni): **1 HULA drone** + sample landing pad + sample ArUco pad; the **3
  assessment drones are cage-provided**. ⇒ 3-drone coordination can't be fully bench-rehearsed →
  each drone independently robust + disjoint zones; validate coordination in the **sim**.

## `[TO CONFIRM]` at briefing / first test slot (carry into PROGRESS.md)
1. **Coordinate frame/units of the Discord valid-pad coords** vs the UWB frame (origin, axes).
2. **Hoop radius** (measure sample pad) → landing tolerance.
3. Is an **obstacle/crate map provided** for Pre-Uni Ch2? (Mark says yes → drives the planner.)
4. Phase-2 **loiter length**; rover motion (continuous vs pausing); Phase 1→2 continuous?; end-of-
   mission landing location; run start/stop trigger.
5. Are the 5 ids pre-announced or "any 5 distinct"? Are valid/invalid pad ids announced (yes — Slide 5).
6. **Is operator (manual) input allowed/penalised?** (We build autonomous regardless; overrides are
   optional.) Marker + rover physical size. Robot access to train YOLO?
7. `send_manual_control` units/scale + required rate; `YAW_OFFSET`; is `get_position()` arena or onboard?
