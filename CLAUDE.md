# PX4 Drone Project — Claude Master Context

## Repo overview
- GitHub: git@github.com:CricketGaming21/px4-drone-project.git
- `main` branch   → Qualifier code (complete, referenceable)
- `finals` branch → Finals code (active development)

## IMPORTANT: how to reference qualifier code
The qualifier code is accessible two ways:
1. Switch branch:  git checkout main  (to browse all qualifier files)
2. Stay on finals: read finals/reference/qualifier_code/  (copied snapshots)
Never modify qualifier files. Use them for reference and reuse only.

## Qualifier environment (main branch)
- PX4 SITL + Gazebo Harmonic (simulation only)
- MAVSDK-python over MAVLink UDP port 14540
- Drone: x500_depth with depth camera
- Ubuntu 22.04 in VMware VM

## Key qualifier scripts (in finals/reference/qualifier_code/)
- avoid.py                main avoidance navigation loop
- AvoidancePlanner.py     planner orchestration class
- RRTStarPlanner.py       RRT* path planning
- VelocityPlanner.py      velocity-based planning
- PointCloudPlanner.py    depth point cloud planning
- GlobalMapper.py         occupancy map builder
- Detector.py             YOLO detection wrapper
- drone_control.py        low level MAVSDK control
- get_depth.py            depth camera reader
- depth_receiver.py       depth data receiver

## Debug infrastructure (qualifier)
- claude_debug/run_with_log.sh      run scripts with full logging
- claude_debug/telemetry_logger.py  MAVLink telemetry to log files
- claude_debug/logs/                all runtime logs (gitignored)

## Git workflow
- Check branch before editing:  git branch
- Quick save:                   save
- Proper commit:                push
- Switch to finals:             finals  (alias)
- Switch to qualifier:          qualifier  (alias)
- Auto-push hook active:        every commit auto-pushes to GitHub

## Start a Claude Code session
```bash
cd ~/Desktop/codes
claude
```
Say: "Read CLAUDE.md, then read finals/CLAUDE.md. Tell me what branch
we are on and summarise the full project before we start."
