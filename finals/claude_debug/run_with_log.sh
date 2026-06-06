#!/bin/bash
# Usage: ./finals/claude_debug/run_with_log.sh finals/mission/phase1_land.py
LOG_DIR="$(dirname "$0")/logs"
mkdir -p "$LOG_DIR"

SCRIPT="$1"
BASENAME=$(basename "$SCRIPT" .py)
TIMESTAMP=$(date +"%H%M%S")
LOGFILE="$LOG_DIR/${BASENAME}_${TIMESTAMP}.log"

echo "========================================" | tee "$LOGFILE"
echo " Running:  $SCRIPT"                       | tee -a "$LOGFILE"
echo " Log:      $LOGFILE"                      | tee -a "$LOGFILE"
echo " Started:  $(date)"                       | tee -a "$LOGFILE"
echo "========================================" | tee -a "$LOGFILE"

cd ~/Desktop/codes
python3 "$SCRIPT" 2>&1 | tee -a "$LOGFILE"

EXIT_CODE=${PIPESTATUS[0]}
echo ""                                          | tee -a "$LOGFILE"
echo "========================================"  | tee -a "$LOGFILE"
echo " Finished: $(date)  Exit: $EXIT_CODE"      | tee -a "$LOGFILE"
echo "========================================"  | tee -a "$LOGFILE"
