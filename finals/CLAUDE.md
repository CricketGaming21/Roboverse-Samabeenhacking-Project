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
