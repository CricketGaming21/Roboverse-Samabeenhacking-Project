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
