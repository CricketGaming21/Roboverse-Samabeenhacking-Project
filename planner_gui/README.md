# planner_gui/ — Mission Planner (Phase 9, single-file browser tool, no build step)
`index.html` (HTML5 canvas + vanilla JS). Loads arena/crates/pads (paste, or default to the sim map);
authors **Phase-1 routes** per drone (place/drag waypoints, live red-on-violation validation,
visibility-graph auto-route, inter-path conflict highlight) and **Phase-2 vantages** (pos + look-yaw +
gimbal + dwell, with camera-footprint + bubble-coverage overlay); exports `mission_plan.yaml`
(see docs/MISSION_PLAN_SCHEMA.md) + a PNG. No localStorage — state in memory, export via download.
The geometry/render core is shared with the C2 console (P10). Gate = the exported file validates +
loads in the mission (see tests/test_plan_schema.py).
