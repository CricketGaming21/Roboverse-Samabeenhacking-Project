#!/bin/bash
# ============================================================
# start_debug_session.sh
# One command to start your full PX4 debug environment.
# Opens a tmux session with 4 panes:
#   [0] PX4 SITL (with logging)
#   [1] Telemetry logger
#   [2] Gazebo topic logger
#   [3] Claude Code (your AI debug partner)
# ============================================================

SESSION="px4_debug"
CODES_DIR=~/Desktop/codes
DEBUG_DIR=~/Desktop/codes/claude_debug

# Kill any existing session
tmux kill-session -t $SESSION 2>/dev/null

tmux new-session -d -s $SESSION -x 220 -y 50

# Pane 0 (top-left): PX4 SITL
tmux rename-window -t $SESSION "PX4 Debug"
tmux send-keys -t $SESSION "echo '=== PANE 0: PX4 SITL ===' && $DEBUG_DIR/capture_px4_console.sh" Enter

# Pane 1 (top-right): Telemetry logger
tmux split-window -t $SESSION -h
tmux send-keys -t $SESSION "echo '=== PANE 1: TELEMETRY LOGGER ===' && sleep 15 && python3 $DEBUG_DIR/telemetry_logger.py" Enter

# Pane 2 (bottom-left): Gazebo topic logger
tmux select-pane -t $SESSION:0.0
tmux split-window -t $SESSION -v
tmux send-keys -t $SESSION "echo '=== PANE 2: GZ TOPIC LOGGER ===' && sleep 20 && $DEBUG_DIR/gz_topic_logger.sh" Enter

# Pane 3 (bottom-right): Claude Code
tmux select-pane -t $SESSION:0.1
tmux split-window -t $SESSION -v
tmux send-keys -t $SESSION "echo '=== PANE 3: CLAUDE CODE ===' && cd $CODES_DIR && claude" Enter

# Focus on Claude pane
tmux select-pane -t $SESSION:0.3

tmux attach-session -t $SESSION

