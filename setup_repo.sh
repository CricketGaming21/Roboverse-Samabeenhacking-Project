#!/bin/bash
# =============================================================
# setup_repo.sh — RoboVerse Repo Setup
# Run ONCE in your Ubuntu VM.
#
# What this does:
#   1. Keeps qualifier code on main branch (readable, referenceable)
#   2. Creates finals/ subfolder on a new finals branch
#   3. Writes all CLAUDE.md, config, debug, gitignore files
#   4. Sets up auto-push git hook and bash aliases
#   5. Writes a Claude Code organise script for the reference dump
#   6. Commits and pushes everything to GitHub
#
# Usage:
#   chmod +x setup_repo.sh
#   ./setup_repo.sh
# =============================================================

set -e

CODES_DIR=~/Desktop/codes

# ─── Banner ───────────────────────────────────────────────
echo ""
echo "======================================================"
echo "  RoboVerse Repo Setup"
echo "======================================================"
echo ""
read -p "Enter your GitHub username: " GITHUB_USERNAME
echo ""
echo "  GitHub user : $GITHUB_USERNAME"
echo "  Project dir : $CODES_DIR"
echo ""
read -p "Press Enter to start or Ctrl+C to cancel..."

cd "$CODES_DIR"

# ─── 1. Commit everything on main (qualifier stays intact) ─
echo ""
echo "[1/8] Saving qualifier code on main branch..."
git checkout main 2>/dev/null || git checkout -b main
git add -A
git commit -m "chore: save qualifier code - accessible for finals reference" 2>/dev/null \
    || echo "      Nothing new to commit on main, continuing."
git push origin main 2>/dev/null || true
echo "      Qualifier code saved on main branch."

# ─── 2. Create finals branch from main ────────────────────
echo ""
echo "[2/8] Creating finals branch..."
git checkout -b finals 2>/dev/null || git checkout finals
git push -u origin finals 2>/dev/null || true
echo "      Finals branch ready."
echo "      Qualifier code still fully readable at: git checkout main"

# ─── 3. Create folder structure ───────────────────────────
echo ""
echo "[3/8] Creating folder structure..."

# Qualifier reference snapshot (copies, not originals — originals stay on main)
mkdir -p "$CODES_DIR/qualifier"

# Finals folders
mkdir -p "$CODES_DIR/finals/mission"
mkdir -p "$CODES_DIR/finals/control"
mkdir -p "$CODES_DIR/finals/detection"
mkdir -p "$CODES_DIR/finals/utils"
mkdir -p "$CODES_DIR/finals/weights"
mkdir -p "$CODES_DIR/finals/claude_debug/logs"

# Reference folders (dump everything here first, Claude Code sorts it)
mkdir -p "$CODES_DIR/finals/reference/project_details"
mkdir -p "$CODES_DIR/finals/reference/sample_code"
mkdir -p "$CODES_DIR/finals/reference/learning_materials"
mkdir -p "$CODES_DIR/finals/reference/hardware_docs"
mkdir -p "$CODES_DIR/finals/reference/qualifier_code"
mkdir -p "$CODES_DIR/finals/reference/dump"

# Copy qualifier .py files into reference so Claude can see them on finals branch
echo "      Copying qualifier scripts into finals/reference/qualifier_code/..."
cp "$CODES_DIR"/*.py "$CODES_DIR/finals/reference/qualifier_code/" 2>/dev/null || true

# Keep empty folders tracked
for dir in mission control detection weights \
    reference/project_details reference/sample_code \
    reference/learning_materials reference/hardware_docs \
    reference/dump; do
    touch "$CODES_DIR/finals/$dir/.gitkeep"
done

echo "      Folders created."

# ─── 4. Write .gitignore ──────────────────────────────────
echo ""
echo "[4/8] Writing .gitignore..."

cat > "$CODES_DIR/.gitignore" << 'EOF'
# ── Python ────────────────────────────────────────────────
__pycache__/
*.pyc
*.pyo
*.pyd

# ── YOLO / ML weights (too large for GitHub) ──────────────
*.pt
*.pth
*.onnx
*.rknn
*.engine
*.tflite
*.weights
finals/weights/

# ── Images / video / captures ─────────────────────────────
captured_images/
detected_images/
detections/
confirmed/
confirmed_barrels/
dataset_raw/
*.jpg
*.jpeg
*.png
*.mp4
*.avi
*.bag

# ── Runtime logs (never commit) ───────────────────────────
claude_debug/logs/
finals/claude_debug/logs/
*.log

# ── Runtime JSON logs ─────────────────────────────────────
barrel_detections.json
nav_status.json
canister_log.txt

# ── Jupyter ───────────────────────────────────────────────
.ipynb_checkpoints/

# ── Secrets ───────────────────────────────────────────────
.env
*.env

# ── OS ────────────────────────────────────────────────────
.DS_Store
Thumbs.db
EOF

echo "      .gitignore written."

# ─── 5. Write root CLAUDE.md ──────────────────────────────
echo ""
echo "[5/8] Writing CLAUDE.md files..."

cat > "$CODES_DIR/CLAUDE.md" << EOF
# PX4 Drone Project — Claude Master Context

## Repo overview
- GitHub: git@github.com:${GITHUB_USERNAME}/px4-drone-project.git
- \`main\` branch   → Qualifier code (complete, referenceable)
- \`finals\` branch → Finals code (active development)

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
\`\`\`bash
cd ~/Desktop/codes
claude
\`\`\`
Say: "Read CLAUDE.md, then read finals/CLAUDE.md. Tell me what branch
we are on and summarise the full project before we start."
EOF

# ─── 6. Write finals/CLAUDE.md ────────────────────────────
cat > "$CODES_DIR/finals/CLAUDE.md" << 'EOF'
# RoboVerse Finals — Claude Context

## Competition
RoboVerse 2026 Finals — Pre-University Category
Challenge 2 ONLY (we do not do Challenge 1)
Prize: 1st $1800 / 2nd $1300 / 3rd $900

## Challenge 2 — two phases

### Phase 1: Landing
- Launch 3x HULA drones from C2 terminal
- Land accurately on 3 chosen valid landing zones
- Landing zones marked by Aruco markers in arena
- Scored on: accuracy of landing + speed

### Phase 2: Search and snapshot
- 5x RoboMaster ground robots loiter in arena
- 3 HULA drones search and photograph the robots
- Scored on: successful snapshots + time taken

## How this differs from the qualifier
| Qualifier                  | Finals                          |
|----------------------------|---------------------------------|
| PX4 SITL simulation        | Real physical hardware          |
| MAVSDK-python              | pyhulax library                 |
| Single drone               | 3x HULA drones (swarm)          |
| GNSS/simulated GPS         | UWB positioning                 |
| Runs inside Ubuntu VM      | Runs on Windows C2 terminal     |
| Gazebo depth camera        | Physical onboard camera         |
| MAVLink UDP port 14540     | pyhulax SDK                     |

## Hardware
- 3x Highgreat HULA drones
- C2 Terminal: Windows laptop with Ubuntu 22.04 VM
- UWB tag on each drone (gives north-east position in arena)
- Onboard camera on each HULA drone

## Key libraries
- pyhulax         HULA drone control and camera access
- UWB library     drone UWB positioning (provided by organisers)
- OpenCV          image processing
- YOLO / RKNN     object detection (NPU accelerated ~50fps)

## Full folder map (read this carefully)
\`\`\`
finals/
  mission/                    YOUR ACTIVE MISSION CODE
    phase1_land.py            Phase 1: landing on zones
    phase2_search.py          Phase 2: search and snapshot

  control/                    DRONE CONTROL LAYER
    hula_control.py           pyhulax wrapper class
    uwb_handler.py            UWB position polling and correction

  detection/                  VISION AND DETECTION
    detector.py               YOLO/RKNN detection wrapper
    snapshot.py               robot snapshot capture logic

  utils/                      SHARED HELPERS
    config.py                 ALL tunable params — read first
    logger.py                 logging to terminal and file

  reference/                  READ ONLY — never modify these
    qualifier_code/           full qualifier scripts for reference
    project_details/          competition brief, rules, scoring
    sample_code/              organiser-provided reference code
    learning_materials/       PDFs from qualifier workshops
    hardware_docs/            hardware manuals and specs
    dump/                     drop new files here to be organised

  weights/                    YOLO model files (gitignored)
  claude_debug/               debug scripts and logs
    run_with_log.sh           run any script with full logging
    organise_dump.sh          sorts files from dump into correct folders
    logs/                     runtime logs (gitignored)
\`\`\`

## Rules for Claude Code — read before touching anything
1. Read this file AND root CLAUDE.md before starting any task
2. Read finals/reference/ folders before writing any code
3. Read finals/utils/config.py before hardcoding ANY value
4. NEVER modify files inside finals/reference/
5. ALL tunable values go in finals/utils/config.py only
6. ALL new code goes in the correct subfolder above
7. Reference qualifier code in finals/reference/qualifier_code/ for reusable patterns
8. Commit format: "feat/fix/docs/chore: what changed and why"
9. After every working change: git add -A && git commit -m "..."
   (auto-push hook sends it to GitHub automatically)

## Reference files — read these before writing any code
- reference/qualifier_code/     full qualifier scripts
- reference/project_details/    competition brief (RoboVerse.pptx)
- reference/sample_code/        organiser code (kolomee.py, getDepth.py etc.)
- reference/learning_materials/ qualifier workshop PDFs
- reference/hardware_docs/      hardware specs

## Adding new reference files
1. Drop files into: finals/reference/dump/
2. Run: git add -A && git commit -m "docs: add reference files to dump"
3. In Claude Code say: "organise the reference dump"
   Claude Code will move files to the right subfolder and update this file.

## Debug workflow
\`\`\`bash
cd ~/Desktop/codes
git checkout finals
./finals/claude_debug/run_with_log.sh finals/mission/phase1_land.py
\`\`\`

## How to start every Claude Code session
\`\`\`bash
cd ~/Desktop/codes
git checkout finals
claude
\`\`\`
Then say:
"Read CLAUDE.md and finals/CLAUDE.md. List every file in
finals/reference/ and tell me what you understand about the
project before we start anything."
EOF

echo "      CLAUDE.md files written."

# ─── 7. Write utils files ─────────────────────────────────
cat > "$CODES_DIR/finals/utils/config.py" << 'EOF'
# =============================================================
# config.py — ALL tunable parameters live here
# Claude Code: always read this before hardcoding any value
# =============================================================

# ── Swarm ─────────────────────────────────────────────────
NUM_DRONES              = 3

# ── Flight ────────────────────────────────────────────────
TAKEOFF_ALTITUDE_M      = 1.5
FLIGHT_SPEED_MS         = 1.0
LANDING_DESCENT_SPEED   = 0.3
HOVER_ALTITUDE_M        = 2.0

# ── UWB positioning ───────────────────────────────────────
UWB_POLL_RATE_HZ        = 10
POSITION_TOLERANCE_M    = 0.3       # acceptable error at landing
VELOCITY_CORRECTION_KP  = 0.5      # proportional gain for correction

# ── Arena ─────────────────────────────────────────────────
LANDING_ZONE_RADIUS_M   = 0.5
NUM_LANDING_ZONES       = 3
SEARCH_GRID_SPACING_M   = 1.5

# ── Detection ─────────────────────────────────────────────
MODEL_PATH              = "weights/best.pt"
CONFIDENCE_THRESHOLD    = 0.6
SNAPSHOT_COOLDOWN_S     = 2.0
TARGET_CLASS_ID         = 0         # RoboMaster ground robot

# ── Aruco ─────────────────────────────────────────────────
ARUCO_DICT              = "DICT_4X4_50"
VALID_MARKER_IDS        = []        # fill after competition briefing

# ── Timeouts ──────────────────────────────────────────────
CONNECTION_TIMEOUT_S    = 10
LANDING_TIMEOUT_S       = 30
MISSION_TIMEOUT_S       = 300
EOF

cat > "$CODES_DIR/finals/utils/logger.py" << 'EOF'
# =============================================================
# logger.py — shared logging utility
# Writes to both terminal and a timestamped log file
# =============================================================
import logging
import os
from datetime import datetime

LOG_DIR = os.path.join(os.path.dirname(__file__), "../claude_debug/logs")
os.makedirs(LOG_DIR, exist_ok=True)

def get_logger(name: str) -> logging.Logger:
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)
    if logger.handlers:
        return logger

    fmt = "[%(asctime)s] %(levelname)s %(name)s: %(message)s"

    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    ch.setFormatter(logging.Formatter(fmt, "%H:%M:%S"))

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    fh = logging.FileHandler(os.path.join(LOG_DIR, f"{name}_{timestamp}.log"))
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(logging.Formatter(fmt))

    logger.addHandler(ch)
    logger.addHandler(fh)
    return logger
EOF

# ─── 8. Write debug and organise scripts ──────────────────
echo ""
echo "[6/8] Writing debug and organise scripts..."

cat > "$CODES_DIR/finals/claude_debug/run_with_log.sh" << 'EOF'
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
EOF

# This is the script Claude Code runs to sort the dump folder
cat > "$CODES_DIR/finals/claude_debug/organise_dump.sh" << 'EOF'
#!/bin/bash
# =============================================================
# organise_dump.sh
# Run by Claude Code to sort files from reference/dump/
# into the correct reference subfolders.
# Claude Code reads each file, determines what it is,
# moves it, and updates finals/CLAUDE.md accordingly.
# =============================================================

DUMP="$HOME/Desktop/codes/finals/reference/dump"
REF="$HOME/Desktop/codes/finals/reference"

echo "Files currently in dump:"
echo "─────────────────────────"
ls -la "$DUMP"
echo ""
echo "Claude Code: please read each file above, determine what it is,"
echo "and move it to the correct subfolder:"
echo "  $REF/project_details/     → competition briefs, rules, scoring"
echo "  $REF/sample_code/         → organiser-provided code files"
echo "  $REF/learning_materials/  → tutorial PDFs, workshop slides"
echo "  $REF/hardware_docs/       → hardware manuals and specs"
echo ""
echo "After moving all files, update finals/CLAUDE.md to list"
echo "every file now in each reference subfolder."
EOF

chmod +x "$CODES_DIR/finals/claude_debug/run_with_log.sh"
chmod +x "$CODES_DIR/finals/claude_debug/organise_dump.sh"

echo "      Debug scripts written."

# ─── Git hook: auto-push on every commit ──────────────────
echo ""
echo "[7/8] Setting up git auto-push hook..."

cat > "$CODES_DIR/.git/hooks/post-commit" << 'EOF'
#!/bin/bash
BRANCH=$(git branch --show-current)
echo "Auto-pushing $BRANCH to GitHub..."
git push origin "$BRANCH" --quiet
echo "Pushed."
EOF

chmod +x "$CODES_DIR/.git/hooks/post-commit"

# ─── Bash aliases ─────────────────────────────────────────
echo ""
echo "[8/8] Adding bash aliases..."

if ! grep -q "alias save=" ~/.bashrc; then
cat >> ~/.bashrc << 'EOF'

# ── RoboVerse project shortcuts ───────────────────────────
alias save='cd ~/Desktop/codes && git add -A && git commit -m "wip: checkpoint $(date +"%H:%M")"'
alias push='cd ~/Desktop/codes && git add -A && git commit -m "$(read -p "Commit message: " m && echo $m)"'
alias finals='cd ~/Desktop/codes && git checkout finals'
alias qualifier='cd ~/Desktop/codes && git checkout main'
alias logs='tail -f ~/Desktop/codes/finals/claude_debug/logs/*.log 2>/dev/null || echo "No logs yet - run a script first"'
alias dumpcheck='ls -la ~/Desktop/codes/finals/reference/dump/'
EOF
fi

source ~/.bashrc 2>/dev/null || true
echo "      Aliases added."

# ─── Final commit and push ─────────────────────────────────
echo ""
echo "Committing and pushing everything to GitHub..."
cd "$CODES_DIR"
git add -A
git commit -m "feat: full repo setup - finals branch, CLAUDE.md, debug scripts, qualifier reference"
# hook fires and auto-pushes

echo ""
echo "======================================================"
echo "  Setup complete!"
echo "======================================================"
echo ""
echo "  Branches:"
echo "    main    → qualifier code (intact, referenceable)"
echo "    finals  → finals development (active)"
echo ""
echo "  Shortcuts:"
echo "    save       → quick checkpoint commit + auto-push"
echo "    push       → commit with your own message + auto-push"
echo "    finals     → switch to finals branch"
echo "    qualifier  → switch to main branch"
echo "    logs       → watch live debug logs"
echo "    dumpcheck  → see what's in the reference dump folder"
echo ""
echo "  Next steps:"
echo "  ─────────────────────────────────────────────────────"
echo "  1. Copy your reference files into:"
echo "     ~/Desktop/codes/finals/reference/dump/"
echo ""
echo "  2. Run: save"
echo ""
echo "  3. Open Claude Code:"
echo "     cd ~/Desktop/codes && claude"
echo ""
echo "  4. Say exactly this to Claude Code:"
echo "     'Read CLAUDE.md and finals/CLAUDE.md."
echo "      Then run finals/claude_debug/organise_dump.sh"
echo "      and sort every file in the dump into the right"
echo "      reference subfolder. Update finals/CLAUDE.md"
echo "      to list every file and what it contains.'"
echo "======================================================"
