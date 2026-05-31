#!/bin/bash
# Logs Gazebo topics to files — run after PX4 SITL is up
# Captures: depth camera, IMU, world stats

LOG_DIR=~/Desktop/codes/claude_debug/logs
mkdir -p "$LOG_DIR"

echo "Starting Gazebo topic logging..."

# Log IMU data (10 msgs, repeat every 5s — lightweight)
while true; do
    TIMESTAMP=$(date +"%H:%M:%S")
    echo "[$TIMESTAMP] --- GZ TOPIC SNAPSHOT ---" >> "$LOG_DIR/gz_topics.log"
    
    # World stats (sim time, real time factor)
    gz topic -e -n 1 -t /world/default/stats 2>/dev/null \
        | grep -E "sim_time|real_time_factor" \
        >> "$LOG_DIR/gz_topics.log" 2>&1
    
    # IMU snapshot
    echo "[$TIMESTAMP] IMU:" >> "$LOG_DIR/gz_topics.log"
    gz topic -e -n 1 -t /world/default/model/x500_vision_0/link/base_link/sensor/imu_sensor/imu 2>/dev/null \
        | head -20 >> "$LOG_DIR/gz_topics.log" 2>&1

    sleep 5
done


