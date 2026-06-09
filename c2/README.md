# c2/ — Operator Console (Phase 10, single-file browser tool + thin JSON bridge)
`index.html` renders live mission state served by `src/mission/runtime/c2_bridge.py` (reads the public
API + the shared world model; no simcore). Panels: live top-down (drones, footprints, paths/zones,
rover tracks, **belief heatmap**), 3 camera tiles w/ detection boxes, **rubric scoreboard** (landings;
distinct tagged ids + capture thumbnails; outstanding ids; clock), **alarms** (battery, UWB dropout,
near-collision, **drone over a crate footprint = no-fly violation**), pre-flight checklist, one-click
**evidence export** (annotated images + results sheet + map snapshot for the judge), and an **optional
override layer** (re-task / designate target / recall / hold-all) that injects into the coordinator.
Overrides are optional — the mission must score fully if operator input is disallowed. Reuses the
planner's render core.
