# MISSION_PLAN_SCHEMA.md — the planner → mission contract (`mission_plan.yaml`)

Authored in the Planner GUI (P9), consumed by the mission (P4 assignment, P7 vantage patrol). Units:
**metres** (arena frame, x=North, y=East), degrees for angles. Validated on load (reject unknown keys).

```yaml
version: 1
arena:                      # [SYNC-WITH-SIM] mirror of the map used to plan
  length_m: 10.0            # North extent
  width_m: 6.0              # East extent
  crates:                   # provided footprints (rectangles), for reference/validation
    - { center: [5.0, 3.0], size: [0.9, 0.9], height_m: 1.2 }
  inflate_m: 0.40           # safety radius used when these routes were validated

drones:                     # one block per drone; tag_id ties to UWB + config
  - tag_id: 0
    color: "#e6194b"        # for the GUI/C2
    phase1:
      pad_id: 10            # which pad this drone lands on (must be a valid id)
      route_m: [[0.6,1.1],[2.0,1.5],[8.5,3.0]]   # waypoint chain C2 -> pad, footprint-clear
    phase2:
      bubble:               # this drone's search zone (polygon, arena m)
        - [0.5,0.5]
        - [5.0,0.5]
        - [5.0,3.0]
        - [0.5,3.0]
      vantages:             # persistent overwatch points (cycled, not a lawnmower)
        - { xy: [2.0,1.5], look_yaw_deg: 0,  gimbal_deg: 60, dwell_s: 2.0 }
        - { xy: [4.0,2.2], look_yaw_deg: 90, gimbal_deg: 75, dwell_s: 2.0 }
  - tag_id: 1
    # ...
  - tag_id: 2
    # ...

meta:
  hoop_tol_m: 0.15          # [TO CONFIRM] measured hoop radius minus UWB margin
  cruise_alt_m: 1.10
```

Rules the loader enforces: every `phase1.route_m` segment is clear of inflated crates and inside
bounds; `pad_id` is one of the announced valid ids; vantages lie inside the drone's `bubble`; bubbles
are pairwise disjoint (with a buffer). `examples/mission_plan.example.yaml` is the canonical fixture.
