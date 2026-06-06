"""
config.py — v10. Corner-safe navigation parameters.
"""

# ── Arena ──────────────────────────────────────────────────────────────────
GRID_CELL_SIZE_M    = 4.0
GRID_ORIGIN_N       = -2.0
GRID_ORIGIN_E       = -2.0
GRID_END_N          = 34.0
GRID_END_E          = 38.0

# ── Flight ─────────────────────────────────────────────────────────────────
TAKEOFF_ALT_M       = 3.0
ALT_SEARCH_M        = 3.0        # pass 1 altitude
MAX_SPEED           = 1.0        # m/s — max in open space
MIN_SPEED           = 0.3        # m/s — min (at obstacle threshold)
TURN_YAW_RATE       = 60.0       # deg/s
CONTROL_DT_S        = 0.1

# ── Obstacle Avoidance ─────────────────────────────────────────────────────
SAFE_DIST_M         = 2.0        # center blocked → turn
CRITICAL_DIST_M     = 0.8        # reverse while turning if any wall this close
EMERGENCY_DIST_M    = 0.5        # all-blocked reverse
WALL_PROXIMITY_M    = 2.0        # push away starts at 2m (was 1.8 — too late for corners)
CORRIDOR_WIDTH      = 5.0        # both sides < this → corridor mode
CENTERING_KP        = 6.0        # corridor centering gain

# ── Depth ROI ──────────────────────────────────────────────────────────────
ROI_TOP_FRAC        = 0.25
ROI_BOT_FRAC        = 0.70

# ── Altitude PID ───────────────────────────────────────────────────────────
ALT_KP = 1.0;  ALT_KI = 0.02;  ALT_KD = 0.3

# ── Stuck Detection ────────────────────────────────────────────────────────
STUCK_TIMEOUT_S     = 5.0
STUCK_DIST_M        = 0.4
BACKTRACK_TURN_S    = 2.0
BACKTRACK_PUSH_S    = 1.5
BACKTRACK_COOLDOWN_S = 8.0

# ── YOLO ───────────────────────────────────────────────────────────────────
YOLO_MODEL          = "best.pt"
SPOT_CONF           = 0.4
CONFIRM_CONF        = 0.75
CONFIRM_FRAMES      = 2
YOLO_HZ             = 8
DETECTOR_ENABLED    = True

# ── Lock-On ────────────────────────────────────────────────────────────────
MIN_BBOX_H_PX       = 40
TARGET_BBOX_H_PX    = 80
LOCKON_CENTER_TOL   = 100
LOCKON_MAX_TIME_S   = 8.0
LOCKON_COOLDOWN_S   = 3.0
APPROACH_BARREL_SPD = 0.5
LOCKON_ALT_YELLOW   = 2.2        # was 1.5 — caused 4.7m plunge crash
LOCKON_ALT_RED      = 3.0        # stay at 3m for elevated red barrels

# ── Barrel Dedup ───────────────────────────────────────────────────────────
BARREL_SEP_M        = 4.0
CONFIRM_COOLDOWN_S  = 3.0

# ── Topics ─────────────────────────────────────────────────────────────────
DEPTH_TOPIC = "/depth_camera"
IMAGE_TOPIC = "/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image"
CAM_FX = 433.0;  CAM_CX = 320.0;  CAM_HEIGHT = 480;  CAM_WIDTH = 640

# ── Timing ─────────────────────────────────────────────────────────────────
MISSION_TIMEOUT_S   = 540
