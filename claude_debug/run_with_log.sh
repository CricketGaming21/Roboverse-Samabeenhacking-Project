#!/bin/bash
# Usage: ./run_with_log.sh avoid.py
# Runs your script and tees ALL output (stdout + stderr) to a timestamped log

LOG_DIR=~/Desktop/codes/claude_debug/logs
mkdir -p "$LOG_DIR"

SCRIPT="$1"
BASENAME=$(basename "$SCRIPT" .py)
TIMESTAMP=$(date +"%H%M%S")
LOGFILE="$LOG_DIR/${BASENAME}_${TIMESTAMP}.log"

echo "========================================" | tee "$LOGFILE"
echo " Running: $SCRIPT" | tee -a "$LOGFILE"
echo " Log: $LOGFILE" | tee -a "$LOGFILE"
echo " Started: $(date)" | tee -a "$LOGFILE"
echo "========================================" | tee -a "$LOGFILE"

# Run from the codes directory, capture everything
cd ~/Desktop/codes
python3 "$SCRIPT" 2>&1 | tee -a "$LOGFILE"

EXIT_CODE=${PIPESTATUS[0]}
echo "" | tee -a "$LOGFILE"
echo "========================================" | tee -a "$LOGFILE"
echo " Finished: $(date)  Exit code: $EXIT_CODE" | tee -a "$LOGFILE"
echo "========================================" | tee -a "$LOGFILE"


