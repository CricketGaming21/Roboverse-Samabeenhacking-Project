# PX4 Drone Project — Claude Context

## Environment
- Ubuntu 22.04 in VMware
- PX4 SITL + Gazebo Harmonic
- MAVSDK-python (UDP port 14540)
- Python 3, all scripts in ~/Desktop/codes/

## Key files
- avoid.py — main obstacle avoidance script
- avoid_with_detect.py — avoidance with YOLO detection
- AvoidancePlanner.py — planner class
- RRTStarPlanner.py — RRT* path planner
- VelocityPlanner.py — velocity-based planner
- PointCloudPlanner.py / PointCloudPlanner_new.py — depth-based planning
- GlobalMapper.py / GlobalMapper_new.py — occupancy map
- Detector.py — YOLO object detection wrapper
- drone_control.py / drone_control_new.py — low level control
- get_depth.py — depth camera reader
- get_video.py — camera stream
- imu.py / imutest.py — IMU data
- telemetry_logger.py — MAVLink telemetry to log files

## Debug logs (claude_debug/logs/)
- telemetry.log — live MAVLink telemetry (position, velocity, attitude, GPS, battery)
- events.log — flight mode changes, arm/disarm, health, connection events
- px4_console.log — raw PX4 SITL console output
- gz_topics.log — Gazebo sensor snapshots (IMU, sim time)
- *_HHMMSS.log — timestamped output from each script run

## How to run scripts
- Normal: python3 avoid.py
- With full logging: ./claude_debug/run_with_log.sh avoid.py
- Full debug session: ./claude_debug/start_debug_session.sh

## Start simulation
- cd ~ && ./start_px4.sh (choose vehicle 1=x500_vision or 2=x500_depth)
- MAVLink heartbeat on udpin://0.0.0.0:14540

## When debugging, always check:
1. logs/events.log — for connection/mode/health issues
2. logs/telemetry.log — for position/velocity anomalies
3. logs/<scriptname>_*.log — for Python exceptions and print output
4. logs/px4_console.log — for PX4 internal errors
