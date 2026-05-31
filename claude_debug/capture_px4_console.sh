
#!/bin/bash
# Captures PX4 SITL console output to a log file
# Run this in place of your normal ./start_px4.sh

LOG_DIR=~/Desktop/codes/claude_debug/logs
mkdir -p "$LOG_DIR"
PX4_LOG="$LOG_DIR/px4_console.log"

echo "Starting PX4 SITL — console output going to $PX4_LOG"
echo "=== PX4 Session $(date) ===" > "$PX4_LOG"

cd ~
./start_px4.sh 2>&1 | tee -a "$PX4_LOG"


