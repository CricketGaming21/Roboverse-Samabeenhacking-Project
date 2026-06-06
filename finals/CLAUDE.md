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

### reference/project_details/ — competition briefs, rules, scoring
- `Potential Detection Targets.txt` — organiser hints on likely detection targets:
  fiducial markers (Aruco / QR / AprilTag). Includes sample OpenCV Aruco detection
  code with depth-to-3D-position conversion (pixel + depth → X,Y,Z metres).

### reference/sample_code/ — organiser-provided code
**hula_swarm/** — HULA drone swarm control and video
- `dola.py` — "Dola" discovery listener: listens on UDP for HULA broadcast packets
  and maintains a table of discovered aircraft IPs on the network.
- `huladola.py` — reference for connecting to multiple HULA drones via pyhulax,
  using Dola to find all drone IPs and pulling all video streams onto one computer
  for multi-drone detection. (pyhulax docs: https://pyhulax.xenops.ae)

**flight_control/** — drone flight control with UWB
- `kolomee.py` — MAVSDK offboard velocity control (VelocityNedYaw) using UWB
  position fed in via ROS2 PoseStamped. P-gain navigation to waypoints with
  velocity limits. Key pattern for UWB-corrected position flight.

**uwb/** — UWB positioning
- `UWBParserThread.py` — threaded serial parser for the UWB tag (921600 baud,
  auto-detects USB COM port). Maintains `{tag_id: (x, y, update_time)}` with
  thread-safe access and configurable arena origin offset.

**yolo_to_rknn/** — converting YOLO models for the NPU
- `convertyolotoonnx.py` — minimal YOLO → ONNX export (opset 12, static shapes).
- `convertyolotoonnx_2.py` — YOLO → ONNX export with full RKNN-compat flags
  explained (static input, simplify, FP32, imgsz 640).
- `convertrknn.py` — ONNX → RKNN build/export for rk3588 with step-by-step
  error checks; notes how to enable INT8 quantization.
- `convertrknn2.py` — compact ONNX → RKNN variant; shows 0-1 input normalisation
  via mean 0 / std 255 and optional w8a8 INT8.

**rknn_detection/** — running YOLO on the NPU
- `getDepthAndDetect.py` — full pipeline: RealSense RGB+depth, RKNNLite YOLO
  inference, depth-aligned bounding boxes → 3D object position.
- `rknndecoder.py` — YOLOv11 RKNN output decoder: sigmoid, xywh→xyxy, NMS,
  detection drawing helpers.
- `testrknn_with_display.py` — standalone RKNN model test on a single image
  with post-processing and display; good first check after converting a model.

**realsense/** — RealSense depth camera samples (from qualifier hardware)
- `getRGB.py` — capture and display the RGB stream.
- `getDepth.py` — read depth value at image centre and visualise colorised depth.
- `getInfra.py` — capture left/right infrared streams.
- `getSyncDepthColor.py` — align depth to colour stream for synced frames.
- `getDepthPointCloud.py` — generate a point cloud from depth + colour.
- `getDepthAndDetect.py` — same RGB+depth+RKNN detection pipeline as
  rknn_detection/ copy.
- `generateTopDown.py` — build a top-down occupancy grid from depth data with
  a downward-facing camera (camera frame → north-east grid).
- `rknndecoder.py` — duplicate of rknn_detection/rknndecoder.py.

### reference/learning_materials/ — qualifier workshop PDFs
- `LearningMaterial1.pdf`
- `LearningMaterial2.pdf`
- `LearningMaterial3.pdf`
- `Supplmentary_LearningMaterial1 (1).pdf`
- `Supplmentary_LearningMaterial2 (1).pdf`

### reference/hardware_docs/ — hardware manuals and specs
- `UWBParserThread_Core_Documentation.pdf` — documentation for the UWB parser
  thread / UWB tag serial interface (pairs with sample_code/uwb/UWBParserThread.py).

### reference/qualifier_code/ — full qualifier scripts (see root CLAUDE.md)
Key files: avoid.py, AvoidancePlanner.py, RRTStarPlanner.py, VelocityPlanner.py,
PointCloudPlanner.py, GlobalMapper.py, Detector.py, drone_control.py,
get_depth.py, depth_receiver.py, plus ~40 more utility/test scripts.

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
