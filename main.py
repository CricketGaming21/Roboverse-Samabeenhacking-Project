"""
main.py — RoboVerse 2026 Qualifier (v9)
=========================================
GOAL-PURSUIT NAVIGATION — replaces wall-following entirely.

WHY: Wall-following has two fatal flaws in this arena:
  1. Misses corridor openings (no reason to turn into them)
  2. Picks wrong wall and goes backwards (no sense of direction)

HOW: The drone always has a DESTINATION — the nearest unseen grid cell.
It steers TOWARD that cell using proportional yaw control.
When obstacles block the path, depth camera avoidance steers around them.
Once clear, the goal vector pulls the drone back on course.

This is the "Goal + Avoidance" hybrid from LM3 p.4:
  - Goal Vector gives PURPOSE (always heading somewhere useful)
  - Avoidance Vector gives SAFETY (depth camera dodges walls)
  - Resultant = drone navigates corridors toward unexplored territory

Camera-cone visibility was removed — it marked cells through walls.
Now only cells the drone physically flies near are marked as visited,
ensuring every corridor and chamber is actually explored.
"""

import asyncio
import math
import time
import threading

import numpy as np
import cv2
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

from minimal_autonomy import MinimalAutonomy
import minimal_autonomy as _ma
import config as C
from barrel_tracker import BarrelTracker, LockOn, make_detection_callback

_ma.TAKEOFF_ALTITUDE_M = C.TAKEOFF_ALT_M


def _norm_angle(a):
    """Normalize angle to [-180, 180]."""
    while a > 180: a -= 360
    while a < -180: a += 360
    return a


# ══════════════════════════════════════════════════════════════
#  COVERAGE GRID — tracks which cells the drone has physically visited
# ══════════════════════════════════════════════════════════════
class CoverageGrid:
    """
    Divides the arena into cells and tracks which have been visited.
    Only marks cells within physical proximity (~6m) of the drone.
    No camera-cone projection — it can't see through walls.
    Selects the nearest unvisited cell as the next exploration goal.
    """

    def __init__(self):
        self.n_min = C.GRID_ORIGIN_N
        self.n_max = C.GRID_END_N
        self.e_min = C.GRID_ORIGIN_E
        self.e_max = C.GRID_END_E
        self.cell  = C.GRID_CELL_SIZE_M

        self.rows = int((self.n_max - self.n_min) / self.cell)
        self.cols = int((self.e_max - self.e_min) / self.cell)
        self.total = self.rows * self.cols

        self._seen = set()              # (row, col) of visited cells
        self._goal = None               # current goal (north, east)
        self._goal_time = 0.0           # when goal was selected
        self._goal_cell = None          # (row, col) of goal cell
        self._attempts = {}             # (row,col) → count of timeouts

    def cell_id(self, n, e):
        row = int((n - self.n_min) / self.cell)
        col = int((e - self.e_min) / self.cell)
        row = max(0, min(self.rows - 1, row))
        col = max(0, min(self.cols - 1, col))
        return (row, col)

    def cell_center(self, row, col):
        n = self.n_min + (row + 0.5) * self.cell
        e = self.e_min + (col + 0.5) * self.cell
        return (n, e)

    def mark_seen(self, drone_n, drone_e, drone_yaw):
        """Mark cells the drone is physically near as visited.

        NO camera-cone projection — it can't see through walls.
        Only marks cells within 1.5 cell widths (~6m) of the drone.
        This ensures the drone MUST fly through every corridor and
        chamber for real coverage.
        """
        cr, cc = self.cell_id(drone_n, drone_e)
        mark_radius = 1  # mark current cell + immediate neighbors

        for dr in range(-mark_radius, mark_radius + 1):
            for dc in range(-mark_radius, mark_radius + 1):
                nr, nc = cr + dr, cc + dc
                if 0 <= nr < self.rows and 0 <= nc < self.cols:
                    cn, ce = self.cell_center(nr, nc)
                    dist = math.hypot(cn - drone_n, ce - drone_e)
                    if dist < self.cell * 1.5:  # ~6m
                        self._seen.add((nr, nc))

    def coverage_pct(self):
        return 100.0 * len(self._seen) / max(1, self.total)

    def select_goal(self, drone_n, drone_e, drone_yaw):
        """
        Pick the nearest unvisited cell as the exploration goal.
        Bias toward cells in the current heading direction.
        Give up on cells after 2 failed attempts (probably behind a wall).
        """
        now = time.monotonic()

        # Keep current goal ONLY if it's still roughly ahead
        # If goal is >90° off heading, the drone changed direction
        # (exited a corridor, backtracked, etc) — pick a new one
        if (self._goal is not None and
                self._goal_cell not in self._seen and
                now - self._goal_time < 30.0):
            # Check if goal is still roughly ahead
            gn, ge = self._goal
            bearing = math.degrees(
                math.atan2(ge - drone_e, gn - drone_n))
            angle_off = abs(_norm_angle(bearing - drone_yaw))
            if angle_off < 90.0:
                return self._goal
            # Goal is behind us — fall through to reselect

        # Current goal timed out or was force-reselected — record failure
        if self._goal_cell is not None and self._goal_cell not in self._seen:
            self._attempts[self._goal_cell] = \
                self._attempts.get(self._goal_cell, 0) + 1
            tries = self._attempts[self._goal_cell]
            if tries >= 2:
                self._seen.add(self._goal_cell)

        best = None
        best_score = float('inf')

        for r in range(self.rows):
            for c in range(self.cols):
                if (r, c) in self._seen:
                    continue
                cn, ce = self.cell_center(r, c)
                dist = math.hypot(cn - drone_n, ce - drone_e)

                # Heading bias: cells in front cost less
                bearing = math.degrees(
                    math.atan2(ce - drone_e, cn - drone_n))
                angle_off = abs(_norm_angle(bearing - drone_yaw))
                heading_penalty = angle_off / 180.0

                score = dist + heading_penalty * 4.0

                if score < best_score:
                    best_score = score
                    best = (cn, ce)
                    self._goal_cell = (r, c)

        self._goal = best
        self._goal_time = now
        return best

    def force_reselect(self):
        """Force goal reselection (after backtrack, etc)."""
        self._goal = None

    def all_seen(self):
        return len(self._seen) >= self.total


# ══════════════════════════════════════════════════════════════
#  MAIN COMPETITION AUTONOMY
# ══════════════════════════════════════════════════════════════
class CompetitionAutonomy(MinimalAutonomy):

    def __init__(self):
        super().__init__(depth_topic=C.DEPTH_TOPIC)

        self.SAFE_DISTANCE_M     = C.SAFE_DIST_M
        self.CRITICAL_DISTANCE_M = C.CRITICAL_DIST_M
        self.FORWARD_SPEED_M_S   = C.MAX_SPEED
        self.YAW_RATE_DEG_S      = C.TURN_YAW_RATE
        self.ROI_TOP_FRACTION    = C.ROI_TOP_FRAC
        self.ROI_BOTTOM_FRACTION = C.ROI_BOT_FRAC

        # Position + yaw
        self._north = 0.0; self._east = 0.0
        self._down = 0.0;  self._yaw = 0.0
        self._pos_ready = False

        # Systems
        self.grid    = CoverageGrid()
        self.tracker = BarrelTracker()
        self.lockon  = LockOn()
        self.detector = None

        # Altitude PID
        self._alt_i = 0.0; self._alt_prev = 0.0; self._alt_t = None

        # YOLO
        self._last_yolo_t = 0.0
        self._gz_node = None
        self._mission_start = 0.0

        # Stuck detection (fixed reference point)
        self._stuck_ref = None
        self._stuck_time = time.monotonic()
        self._last_backtrack = 0.0

        # Turn commitment (prevents oscillation at obstacles)
        self._turn_committed = 0        # +1 right, -1 left, 0 none
        self._turn_commit_until = 0.0

        # Wall-follow escape: activates after 3s of continuous obstacle
        self._obstacle_since = 0.0      # time when obstacle mode started
        self._in_obstacle = False

        # Yaw commitment: prevents sudden 180° reversals
        self._committed_yaw = None      # heading we're committed to
        self._yaw_commit_until = 0.0    # when commitment expires

        # Lock-on attempt tracking (prevents repeated locks on same barrel)
        self._lockon_attempts = []   # [(time, north, east)]

        # Latest RGB frame for confirmation screenshots
        self._latest_rgb = None
        self._rgb_lock = threading.Lock()

    # ── Telemetry ─────────────────────────────────────────────
    async def _track_state(self):
        async def _pos():
            try:
                async for pv in self.drone.telemetry.position_velocity_ned():
                    if not self.running: break
                    self._north = pv.position.north_m
                    self._east  = pv.position.east_m
                    self._down  = pv.position.down_m
                    self._pos_ready = True
            except asyncio.CancelledError: pass

        async def _yaw():
            try:
                async for att in self.drone.telemetry.attitude_euler():
                    if not self.running: break
                    self._yaw = att.yaw_deg
            except asyncio.CancelledError: pass

        try:
            await asyncio.gather(_pos(), _yaw())
        except asyncio.CancelledError: pass

    def _get_position(self):
        if not self._pos_ready: return None
        return (self._north, self._east, -self._down)

    async def _set_telemetry_rates(self):
        try:
            await self.drone.telemetry.set_rate_position_velocity_ned(20.0)
            await self.drone.telemetry.set_rate_attitude_euler(20.0)
            print("  ✅ Telemetry rates → 20 Hz")
        except Exception as e:
            print(f"  ⚠ Telemetry rate set failed: {e}")

    # ── YOLO camera feed ──────────────────────────────────────
    def _start_camera_feed(self):
        if not C.DETECTOR_ENABLED: return
        from Detector import Detector
        self.detector = Detector(
            model_path=C.YOLO_MODEL,
            confidence_threshold=C.SPOT_CONF,
            callback=make_detection_callback(
                self.tracker, self.lockon, self._get_position),
            num_workers=1, device="cpu",
            save_dir="./detections", enable_display=True,
            display_window_name="Barrel Detection",
        )
        # Warm-up to prevent cold-model miss
        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        self.detector.submit_image(dummy, context={"warmup": True})
        print("  ✅ Detector warm-up done")

        self._gz_node = Node()
        def camera_cb(msg: Image):
            now = time.monotonic()
            if now - self._last_yolo_t < 1.0 / C.YOLO_HZ: return
            self._last_yolo_t = now
            try:
                frame = np.frombuffer(msg.data, dtype=np.uint8)
                frame = frame.reshape((msg.height, msg.width, 3))
                bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                # Store for confirmation screenshots
                with self._rgb_lock:
                    self._latest_rgb = bgr.copy()
                self.detector.submit_image(bgr, context={"timestamp": now})
            except Exception as e:
                print(f"  [Cam] {e}")
        if self._gz_node.subscribe(Image, C.IMAGE_TOPIC, camera_cb):
            print(f"  ✅ YOLO: {C.IMAGE_TOPIC}")
        else:
            print(f"  ❌ Camera failed")

    # ── Altitude PID ──────────────────────────────────────────
    def _alt_vz(self, target_alt):
        alt = -self._down
        now = time.monotonic()
        dt = (now - self._alt_t) if self._alt_t else C.CONTROL_DT_S
        self._alt_t = now
        err = target_alt - alt
        self._alt_i += err * dt
        d = (err - self._alt_prev) / max(dt, 1e-3)
        self._alt_prev = err
        return float(-np.clip(C.ALT_KP*err + C.ALT_KI*self._alt_i + C.ALT_KD*d,
                              -1.0, 1.0))

    def _reset_alt(self):
        self._alt_i = 0.0; self._alt_prev = 0.0; self._alt_t = None

    # ── Speed — center clearance only ──────────────────────
    def _calc_speed(self, center):
        """Speed based on what's AHEAD. Side walls handled by yaw."""
        return max(C.MIN_SPEED, min(C.MAX_SPEED, center * 0.5))

    # ── Stuck detection (fixed reference) ─────────────────────
    def _check_stuck(self):
        pos = (self._north, self._east)
        if self._stuck_ref is None:
            self._stuck_ref = pos
        moved = math.hypot(pos[0] - self._stuck_ref[0],
                           pos[1] - self._stuck_ref[1])
        if moved > C.STUCK_DIST_M:
            self._stuck_ref = pos
            self._stuck_time = time.monotonic()
        return (time.monotonic() - self._stuck_time) > C.STUCK_TIMEOUT_S

    def _reset_stuck(self):
        self._stuck_ref = (self._north, self._east)
        self._stuck_time = time.monotonic()

    # ── Heading-referenced yaw helper ─────────────────────────
    async def _yaw_to_heading(self, target_deg, target_alt, tol=5.0):
        for _ in range(80):
            err = _norm_angle(target_deg - self._yaw)
            if abs(err) < tol: break
            rate = C.TURN_YAW_RATE if err > 0 else -C.TURN_YAW_RATE
            down = self._alt_vz(target_alt)
            await self.set_body_velocity(0.0, 0.0, down, rate)
            await asyncio.sleep(C.CONTROL_DT_S)
            if self.lockon.active: return

    # ══════════════════════════════════════════════════════════
    #  GOAL-PURSUIT NAVIGATION
    #  The core algorithm replacing wall-following.
    # ══════════════════════════════════════════════════════════
    def _navigate(self, left, center, right, goal_n, goal_e, target_alt):
        """
        Steer toward goal while avoiding obstacles.

        Returns (forward_speed, yaw_rate, down_speed, action_label)

        Decision priority:
        1. ALL BLOCKED     → reverse (emergency)
        2. CENTER BLOCKED  → turn toward open side biased by goal
        3. CORRIDOR        → center between walls + gentle goal pull
        4. NEAR WALL       → push away from close wall + goal pursuit
        5. CLEAR           → proportional yaw toward goal
        """
        down = self._alt_vz(target_alt)
        now = time.monotonic()

        # Bearing and yaw error to goal
        bearing = math.degrees(
            math.atan2(goal_e - self._east, goal_n - self._north))
        yaw_err = _norm_angle(bearing - self._yaw)

        # ── 1. ALL BLOCKED → fast reverse ─────────────────────
        if (left < C.EMERGENCY_DIST_M and
            center < C.EMERGENCY_DIST_M and
            right < C.EMERGENCY_DIST_M):
            self._turn_committed = 0
            return -0.6, 0.0, down, "REVERSE"

        # Track how long we've been in obstacle modes
        in_trouble = center < C.SAFE_DIST_M or min(left, right) < C.CRITICAL_DIST_M
        if in_trouble:
            if not self._in_obstacle:
                self._in_obstacle = True
                self._obstacle_since = now
        else:
            self._in_obstacle = False

        # ── 1a. WALL-FOLLOW ESCAPE ──────────────────────────
        # If stuck in obstacle modes for 3+ seconds, stop turning in place
        # and SLIDE along the wall until center clears.
        if self._in_obstacle and (now - self._obstacle_since) > 3.0:
            # Pick the more open side and slide along it
            if right > left:
                # Wall on left, slide right-ish
                yaw_rate = C.TURN_YAW_RATE * 0.3
            else:
                # Wall on right, slide left-ish
                yaw_rate = -C.TURN_YAW_RATE * 0.3
            # Always move forward slowly — sliding, not spinning
            fwd = C.MIN_SPEED if center > C.CRITICAL_DIST_M else -C.MIN_SPEED
            return fwd, yaw_rate, down, "WALL_SLIDE"

        # ── 1b. ANY SIDE WALL DANGEROUSLY CLOSE → push away ─
        if min(left, right) < C.CRITICAL_DIST_M:
            push = C.TURN_YAW_RATE if left < right else -C.TURN_YAW_RATE
            return -0.3, push, down, "CORNER_ESCAPE"

        # ── 2. CENTER BLOCKED → turn toward open side ─────────
        if center < C.SAFE_DIST_M:

            # If already committed to a turn, hold it
            if self._turn_committed != 0 and now < self._turn_commit_until:
                turn = C.TURN_YAW_RATE * self._turn_committed
                # Reverse while turning if any wall is very close
                fwd = -0.3 if min(left, center, right) < C.CRITICAL_DIST_M else 0.0
                return fwd, turn, down, "OBSTACLE"

            # Score each side: clearance + goal alignment bonus
            right_score = right + (2.0 if yaw_err > 0 else 0.0)
            left_score  = left  + (2.0 if yaw_err < 0 else 0.0)

            if right_score > left_score + 0.3:
                direction = 1
            elif left_score > right_score + 0.3:
                direction = -1
            else:
                direction = 1 if yaw_err >= 0 else -1

            # 1.5s = 90° at 60 deg/s
            self._turn_committed = direction
            self._turn_commit_until = now + 1.5

            turn = C.TURN_YAW_RATE * direction
            fwd = -0.3 if min(left, center, right) < C.CRITICAL_DIST_M else 0.0
            return fwd, turn, down, "OBSTACLE"

        # Clear turn commitment when center opens up
        self._turn_committed = 0

        # ── 3. CORRIDOR → center between walls, fly THROUGH ───
        if left < C.CORRIDOR_WIDTH and right < C.CORRIDOR_WIDTH:
            imbalance = right - left
            centering = float(np.clip(
                imbalance * C.CENTERING_KP,
                -C.TURN_YAW_RATE * 0.3,
                 C.TURN_YAW_RATE * 0.3))

            # KEY FIX: if goal is behind us (> 60°), suppress goal pull
            # and fly STRAIGHT through corridor. Don't try to U-turn
            # inside a corridor — fly to the exit, then turn.
            if abs(yaw_err) > 60:
                goal_pull = 0.0   # just fly straight, walls guide us
            else:
                goal_pull = float(np.clip(
                    yaw_err * 0.5,
                    -C.TURN_YAW_RATE * 0.15,
                     C.TURN_YAW_RATE * 0.15))

            yaw_rate = centering + goal_pull
            spd = self._calc_speed(center)
            return spd, yaw_rate, down, "CORRIDOR"

        # ── 4a. GOAL FAR OFF → turn toward goal ────────────────
        # BUT: if we're committed to a heading, don't reverse
        if self._committed_yaw is not None and now < self._yaw_commit_until:
            commit_err = abs(_norm_angle(bearing - self._committed_yaw))
            if commit_err > 90:
                # Goal requires >90° turn from committed heading
                # Keep flying committed direction instead of reversing
                goal_yaw = float(np.clip(
                    _norm_angle(self._committed_yaw - self._yaw) * 1.5,
                    -C.TURN_YAW_RATE * 0.5,
                     C.TURN_YAW_RATE * 0.5))
                spd = self._calc_speed(center)
                return spd, goal_yaw, down, "COMMITTED"

        if abs(yaw_err) > 100:
            turn = C.TURN_YAW_RATE if yaw_err > 0 else -C.TURN_YAW_RATE
            # Set new yaw commitment when we start a big turn
            self._committed_yaw = bearing
            self._yaw_commit_until = now + 4.0
            return C.MIN_SPEED, turn, down, "TURN_TO_GOAL"

        # Set/refresh yaw commitment when flying normally
        if abs(yaw_err) < 30:
            self._committed_yaw = self._yaw
            self._yaw_commit_until = now + 4.0

        # ── 4b. NEAR WALL → push away + goal pursuit ─────────
        #    One side is close (< WALL_PROXIMITY_M) but center is clear.
        wall_push = 0.0
        near_wall = False
        if right < C.WALL_PROXIMITY_M:
            proximity = 1.0 - (right / C.WALL_PROXIMITY_M)
            wall_push = -C.TURN_YAW_RATE * 0.5 * proximity
            near_wall = True
        elif left < C.WALL_PROXIMITY_M:
            proximity = 1.0 - (left / C.WALL_PROXIMITY_M)
            wall_push = C.TURN_YAW_RATE * 0.5 * proximity
            near_wall = True

        # ── 5. CLEAR → head toward goal ──────────────────────
        # Proportional yaw correction toward goal bearing
        goal_yaw = float(np.clip(
            yaw_err * 1.5,
            -C.TURN_YAW_RATE,
             C.TURN_YAW_RATE))

        # Combine goal steering with wall push-away
        yaw_rate = goal_yaw + wall_push

        # Clamp combined rate
        yaw_rate = float(np.clip(yaw_rate,
                                 -C.TURN_YAW_RATE,
                                  C.TURN_YAW_RATE))

        # Slow down when turning sharply OR near a wall
        turn_factor = max(0.3, 1.0 - abs(yaw_err) / 90.0)
        wall_factor = 0.7 if near_wall else 1.0
        spd = self._calc_speed(center) * turn_factor * wall_factor

        label = "NEAR_WALL" if near_wall else "GOAL"
        return spd, yaw_rate, down, label

    # ══════════════════════════════════════════════════════════
    #  LOCK-ON: approach barrel with live bbox tracking
    # ══════════════════════════════════════════════════════════
    def _save_confirmation_screenshot(self, barrel_id):
        """Save camera frame as proof when barrel is confirmed."""
        try:
            import os as _os
            _os.makedirs("confirmed", exist_ok=True)
            with self._rgb_lock:
                if self._latest_rgb is not None:
                    path = f"confirmed/barrel_{barrel_id}_{int(time.time())}.jpg"
                    cv2.imwrite(path, self._latest_rgb)
                    print(f"     📸 Screenshot saved: {path}")
        except Exception:
            pass

    async def _execute_lockon(self, target_alt):
        sighting = self.lockon.get()
        if sighting is None:
            self.lockon.clear(); return

        cls = sighting["class_name"]
        icon = "🟡" if "yellow" in cls else "🔴"

        # Skip if barrel is too far off-center — extreme angles cause circling
        # The drone turns 74°, loses its heading, YOLO re-detects → loop
        if abs(sighting["yaw_offset"]) > 50.0:
            print(f"\n  {icon} {cls} at {sighting['yaw_offset']:+.0f}° "
                  f"— too far off-center, skipping")
            self.lockon.clear(); return

        # Skip if already confirmed nearby
        if self.tracker.has_nearby(cls, self._north, self._east):
            print(f"\n  {icon} {cls} already confirmed nearby — skipping")
            self.lockon.clear(); return

        self.lockon.set_frozen_position(self._north, self._east, -self._down)
        score_before = self.tracker.score()
        barrel_count_before = len(self.tracker.confirmed)
        print(f"\n  {icon} LOCK-ON: {cls} "
              f"(yaw={sighting['yaw_offset']:+.1f}°)")

        deadline = time.monotonic() + C.LOCKON_MAX_TIME_S

        def _safe():
            depth = self.receiver.get_frame()
            if depth is None: return True
            l, c, r = self.compute_clearances(depth)
            return min(l, c, r) >= C.EMERGENCY_DIST_M

        # ══════════════════════════════════════════════════════
        # PHASE 1: YAW — simple timed rotation by offset angle
        #          No bbox tracking, no jitter — just turn the
        #          calculated amount and stop.
        # ══════════════════════════════════════════════════════
        yaw_offset = sighting["yaw_offset"]
        yaw_time = abs(yaw_offset) / C.TURN_YAW_RATE + 0.2  # slight overshoot buffer
        yaw_time = min(yaw_time, 2.0)
        rate = C.TURN_YAW_RATE if yaw_offset > 0 else -C.TURN_YAW_RATE
        print(f"     Phase 1: Yaw {yaw_offset:+.0f}° ({yaw_time:.1f}s)")

        t_end = time.monotonic() + yaw_time
        while time.monotonic() < t_end:
            if not _safe():
                print(f"     ⚠ Wall — abort")
                self.lockon.clear(); return
            if self.tracker.score() > score_before:
                print(f"     ✅ Confirmed during yaw!")
                self._save_confirmation_screenshot(barrel_count_before)
                self.lockon.clear(); self._reset_stuck(); return
            down = self._alt_vz(target_alt)
            await self.set_body_velocity(0.0, 0.0, down, rate)
            await asyncio.sleep(C.CONTROL_DT_S)

        # ══════════════════════════════════════════════════════
        # PHASE 1b: HOLD STILL — 1 second, zero movement
        #           Let YOLO get clean frames of the barrel
        # ══════════════════════════════════════════════════════
        print(f"     Phase 1b: Hold still 1.0s — looking at barrel")
        t_end = time.monotonic() + 1.0
        while time.monotonic() < t_end:
            if self.tracker.score() > score_before:
                print(f"     ✅ Confirmed during hold!")
                self._save_confirmation_screenshot(barrel_count_before)
                self.lockon.clear(); self._reset_stuck(); return
            down = self._alt_vz(target_alt)
            await self.set_body_velocity(0.0, 0.0, down, 0.0)
            await asyncio.sleep(C.CONTROL_DT_S)

        # ══════════════════════════════════════════════════════
        # PHASE 2: ALTITUDE — adjust if barrel not fully in frame
        #          NO yaw, NO forward — altitude only
        # ══════════════════════════════════════════════════════
        adj_alt = float(-self._down)
        bbox_data = self.lockon.get_latest_bbox()
        if bbox_data:
            bbox, _ = bbox_data
            bbox_top = bbox[1]
            bbox_bot = bbox[3]
            frame_h = C.CAM_HEIGHT
            needs_adjust = bbox_top < frame_h * 0.05 or bbox_bot > frame_h * 0.95

            if needs_adjust:
                print(f"     Phase 2: Adjusting altitude...")
                for _ in range(20):
                    if time.monotonic() >= deadline: break
                    bbox_data = self.lockon.get_latest_bbox()
                    if bbox_data:
                        bbox, _ = bbox_data
                        top_ok = bbox[1] > frame_h * 0.05
                        bot_ok = bbox[3] < frame_h * 0.95
                        if top_ok and bot_ok:
                            print(f"     ✅ Barrel fully in frame at {adj_alt:.1f}m")
                            break
                        if not bot_ok: adj_alt -= 0.15
                        elif not top_ok: adj_alt += 0.15
                        adj_alt = max(1.5, min(5.0, adj_alt))
                    down = self._alt_vz(adj_alt)
                    await self.set_body_velocity(0.0, 0.0, down, 0.0)
                    await asyncio.sleep(C.CONTROL_DT_S)

        # ══════════════════════════════════════════════════════
        # PHASE 3: APPROACH — move toward barrel if too far
        # ══════════════════════════════════════════════════════
        bbox_data = self.lockon.get_latest_bbox()
        needs_approach = True
        if bbox_data:
            _, bbox_h = bbox_data
            if bbox_h >= C.TARGET_BBOX_H_PX:
                needs_approach = False

        if needs_approach:
            print(f"     Phase 3: Approaching...")
            lost_frames = 0
            approach_start = time.monotonic()
            while (time.monotonic() < deadline and
                   time.monotonic() - approach_start < 4.0):
                if self.tracker.score() > score_before:
                    print(f"     ✅ Confirmed during approach!")
                    self._save_confirmation_screenshot(barrel_count_before)
                    break
                if not _safe():
                    print(f"     ⚠ Obstacle — stop"); break
                bbox_data = self.lockon.get_latest_bbox()
                if bbox_data:
                    lost_frames = 0
                    bbox, bbox_h = bbox_data
                    if bbox_h >= C.TARGET_BBOX_H_PX:
                        print(f"     ✅ Close enough"); break
                    cx = (bbox[0] + bbox[2]) / 2.0
                    norm_off = (cx - C.CAM_CX) / C.CAM_CX
                    yaw_rate = float(np.clip(norm_off * 25.0, -30.0, 30.0))
                    fwd = C.APPROACH_BARREL_SPD
                    down = self._alt_vz(adj_alt)
                    await self.set_body_velocity(fwd, 0.0, down, yaw_rate)
                else:
                    lost_frames += 1
                    if lost_frames > 15:
                        print(f"     ⚠ Lost barrel — abort"); break
                    down = self._alt_vz(adj_alt)
                    await self.set_body_velocity(0.3, 0.0, down, 0.0)
                await asyncio.sleep(C.CONTROL_DT_S)

        # ══════════════════════════════════════════════════════
        # PHASE 4: HOVER — final confirmation
        # ══════════════════════════════════════════════════════
        hover = min(1.5, max(0.5, deadline - time.monotonic()))
        print(f"     Phase 4: Hover {hover:.1f}s")
        t_end = time.monotonic() + hover
        while time.monotonic() < t_end:
            down = self._alt_vz(adj_alt)
            await self.set_body_velocity(0.0, 0.0, down, 0.0)
            await asyncio.sleep(C.CONTROL_DT_S)
            if self.tracker.score() > score_before:
                print(f"     ✅ Confirmed!")
                self._save_confirmation_screenshot(barrel_count_before)
                break

        self.lockon.clear()
        self._reset_stuck()

        # Return to search altitude
        print(f"     ↑ Returning to {target_alt}m")
        for _ in range(15):
            if abs(-self._down - target_alt) < 0.3: break
            down = self._alt_vz(target_alt)
            await self.set_body_velocity(0.0, 0.0, down, 0.0)
            await asyncio.sleep(C.CONTROL_DT_S)

        print(f"     ▶ Resuming exploration")
        self.grid.force_reselect()
        self._committed_yaw = None

    # ── Backtrack (when stuck) ────────────────────────────────
    async def _do_backtrack(self, target_alt):
        if time.monotonic() - self._last_backtrack < C.BACKTRACK_COOLDOWN_S:
            return
        self._last_backtrack = time.monotonic()
        print(f"  ⚠️ STUCK — backtrack + reselect goal")

        # Reverse
        for _ in range(10):
            down = self._alt_vz(target_alt)
            await self.set_body_velocity(-0.4, 0.0, down, 0.0)
            await asyncio.sleep(C.CONTROL_DT_S)

        # Turn toward a random-ish direction (avoid repeating same path)
        turn_dir = 1 if (int(time.monotonic() * 10) % 2 == 0) else -1
        for _ in range(int(C.BACKTRACK_TURN_S / C.CONTROL_DT_S)):
            down = self._alt_vz(target_alt)
            await self.set_body_velocity(0.0, 0.0, down,
                                         C.TURN_YAW_RATE * turn_dir)
            await asyncio.sleep(C.CONTROL_DT_S)

        # Push forward
        for _ in range(int(C.BACKTRACK_PUSH_S / C.CONTROL_DT_S)):
            down = self._alt_vz(target_alt)
            depth = self.receiver.get_frame()
            if depth is not None:
                _, ctr, _ = self.compute_clearances(depth)
                if ctr < C.CRITICAL_DIST_M: break
            await self.set_body_velocity(0.5, 0.0, down, 0.0)
            await asyncio.sleep(C.CONTROL_DT_S)

        self._reset_stuck()
        self.grid.force_reselect()
        self._committed_yaw = None

    # ══════════════════════════════════════════════════════════
    #  SEARCH PASS
    # ══════════════════════════════════════════════════════════
    async def _search_pass(self, target_alt, pass_name):
        print(f"\n{'=' * 55}")
        print(f"  {pass_name} @ {target_alt}m")
        print(f"  Strategy: GOAL PURSUIT + DEPTH AVOIDANCE")
        print(f"  Grid: {self.grid.rows}x{self.grid.cols} cells "
              f"({self.grid.total} total)")
        print(f"{'=' * 55}")

        self._reset_alt()
        self._reset_stuck()
        self._last_backtrack = 0.0
        pass_start = time.monotonic()
        last_status = time.monotonic()

        # Climb/descend to target altitude
        for _ in range(30):
            down = self._alt_vz(target_alt)
            await self.set_body_velocity(0.0, 0.0, down, 0.0)
            await asyncio.sleep(C.CONTROL_DT_S)

        while self.running:
            mission_t = time.monotonic() - self._mission_start
            if mission_t > C.MISSION_TIMEOUT_S:
                print(f"  ⏱ TIMEOUT ({mission_t:.0f}s)")
                return "timeout"

            # Read depth
            depth = self.receiver.get_frame()
            if depth is None:
                await self.set_body_velocity(0.0, 0.0, 0.0, 0.0)
                await asyncio.sleep(C.CONTROL_DT_S); continue

            left, center, right = self.compute_clearances(depth)

            # Position guard
            if not self._pos_ready:
                await self.set_body_velocity(0.0, 0.0, 0.0, 0.0)
                await asyncio.sleep(C.CONTROL_DT_S); continue

            # ── Update coverage grid ─────────────────────────
            self.grid.mark_seen(self._north, self._east, self._yaw)

            # ── Check if coverage complete ────────────────────
            if self.grid.all_seen():
                print(f"  ✅ 100% coverage reached!")
                return "complete"

            # ── Lock-on (barrel detected) ─────────────────────
            if self.lockon.active:
                await self._execute_lockon(target_alt)
                continue

            # ── Stuck detection ───────────────────────────────
            if self._check_stuck():
                await self._do_backtrack(target_alt)
                continue

            # ── Select goal ───────────────────────────────────
            goal = self.grid.select_goal(
                self._north, self._east, self._yaw)

            if goal is None:
                print(f"  ✅ All cells explored!")
                return "complete"

            goal_n, goal_e = goal
            goal_dist = math.hypot(goal_n - self._north,
                                   goal_e - self._east)

            # ── Navigate toward goal ──────────────────────────
            fwd, yaw_rate, down, action = self._navigate(
                left, center, right, goal_n, goal_e, target_alt)

            await self.set_body_velocity(fwd, 0.0, down, yaw_rate)
            await asyncio.sleep(C.CONTROL_DT_S)

            # ── Status print + status file for monitor.py ────
            if time.monotonic() - last_status > 5.0:
                bonus_remaining = max(0.0, 300.0 - mission_t)
                bearing = math.degrees(math.atan2(
                    goal_e - self._east, goal_n - self._north))
                yaw_err = _norm_angle(bearing - self._yaw)
                print(f"  [{mission_t:.0f}s | bonus in "
                      f"{bonus_remaining:.0f}s] {action:<10} "
                      f"L={left:.1f} C={center:.1f} R={right:.1f}  "
                      f"N={self._north:.1f} E={self._east:.1f}  "
                      f"→ goal({goal_n:.0f},{goal_e:.0f}) "
                      f"d={goal_dist:.0f}m err={yaw_err:+.0f}°  "
                      f"cov={self.grid.coverage_pct():.0f}%  "
                      f"score={self.tracker.score()}")
                last_status = time.monotonic()

            # Write status file every tick for monitor.py
            try:
                import json as _json
                _status = {
                    "north": round(self._north, 1),
                    "east":  round(self._east,  1),
                    "alt":   round(-self._down,  1),
                    "yaw":   round(self._yaw,    1),
                    "action": action,
                    "left":   round(left,   1),
                    "center": round(center, 1),
                    "right":  round(right,  1),
                    "coverage": round(self.grid.coverage_pct(), 1),
                    "score": self.tracker.score(),
                    "mission_t": round(mission_t, 0),
                    "goal_n": round(goal_n, 1),
                    "goal_e": round(goal_e, 1),
                    "goal_dist": round(goal_dist, 1),
                    "yellow": self.tracker.count("yellow_barrel"),
                    "red":    self.tracker.count("red_barrel"),
                }
                with open("nav_status.json", "w") as _f:
                    _json.dump(_status, _f)
            except Exception:
                pass

        return "stopped"

    # ── Task loop ─────────────────────────────────────────────
    async def task_loop(self):
        print("\n" + "=" * 55)
        print("  ROBOVERSE 2026 — GOAL PURSUIT NAV v9")
        print("=" * 55 + "\n")

        self._mission_start = time.monotonic()
        print("  Waiting for position...")
        while not self._pos_ready and self.running:
            await asyncio.sleep(0.1)
        if not self.running: return
        print(f"  Position: N={self._north:.1f} E={self._east:.1f}")

        # Pass 1: 3.0m — catches both barrel types
        result = await self._search_pass(
            C.ALT_SEARCH_M, "PASS 1 — ALL BARRELS")

        # Pass 2: 1.5m — yellow ground barrels if time allows
        elapsed = time.monotonic() - self._mission_start
        if (result != "timeout" and elapsed < 420 and
                self.tracker.count("yellow_barrel") < 2):
            # Reset grid for second pass at different altitude
            self.grid = CoverageGrid()
            await self._search_pass(1.5, "PASS 2 — YELLOW (LOW)")

        # Final report
        total = time.monotonic() - self._mission_start
        y = self.tracker.count("yellow_barrel")
        r = self.tracker.count("red_barrel")
        bonus = max(0, int((300 - total) / 30)) * 20 if total < 300 else 0
        print(f"\n{'=' * 55}\n  DONE ({total/60:.1f}min)\n{'=' * 55}")
        print(f"  Yellow: {y}x50={y*50}  Red: {r}x100={r*100}  "
              f"Bonus: +{bonus}")
        print(f"  TOTAL: {self.tracker.score()+bonus}")
        print(f"  Qualifies: "
              f"{'YES' if self.tracker.qualifies() else 'NO'}  "
              f"Coverage: {self.grid.coverage_pct():.0f}%"
              f"\n{'=' * 55}\n")

    # ── Run ───────────────────────────────────────────────────
    async def run(self):
        print("RoboVerse 2026 — Goal Pursuit Nav v9")
        print(f"  Detector: {'ON' if C.DETECTOR_ENABLED else 'OFF'}")
        print(f"  Alt: {C.ALT_SEARCH_M}m  Speed: {C.MAX_SPEED}m/s")
        print(f"  Grid: {self.grid.rows}x{self.grid.cols} "
              f"({self.grid.total} cells)")

        state_task = None
        try:
            await self.connect()
            await self._set_telemetry_rates()
            await self.arm_and_takeoff()
            await self.start_offboard()
            state_task = asyncio.create_task(self._track_state())
            self._start_camera_feed()
            await self.task_loop()
        except asyncio.CancelledError:
            print("Cancelled"); raise
        finally:
            self.tracker.save()
            if self.detector: self.detector.stop()
            if state_task:
                state_task.cancel()
                try: await state_task
                except asyncio.CancelledError: pass
            await self.shutdown()


async def main():
    c = CompetitionAutonomy()
    try: await c.run()
    except KeyboardInterrupt:
        c.stop(); await c.shutdown()

if __name__ == "__main__":
    asyncio.run(main())
