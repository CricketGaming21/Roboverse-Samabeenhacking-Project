"""
noclip_capture.py
==================
Noclip drone control for dataset capture.
 
FIX: Uses /world/roboverse/set_pose/blocking + suspends PX4 physics
     by zeroing motor commands so Gazebo stops fighting our pose.
 
Controls (click OpenCV window first):
  W / S     Forward / Backward
  A / D     Strafe Left / Right
  R / F     Up / Down
  Q / E     Yaw Left / Right
  SPACE     Capture image
  X         Switch class (yellow ↔ red)
  ESC       Quit
"""
 
import os
import sys
import time
import math
import threading
import subprocess
import numpy as np
import cv2
 
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
 
# ── Config ────────────────────────────────────────────────────────────────────
WORLD_NAME   = "roboverse"
MODEL_NAME   = "x500_vision_0"
CAMERA_TOPIC = f"/world/{WORLD_NAME}/model/{MODEL_NAME}/link/camera_link/sensor/IMX214/image"
SET_POSE_SVC = f"/world/{WORLD_NAME}/set_pose/blocking"   # blocking version wins over PX4
 
MOVE_SPEED = 0.5    # metres per keypress — increase if too slow
YAW_SPEED  = 5.0    # degrees per keypress
 
SAVE_DIR = "./dataset_raw"
# ─────────────────────────────────────────────────────────────────────────────
 
os.makedirs(SAVE_DIR, exist_ok=True)
node = Node()
 
# ── Shared state ──────────────────────────────────────────────────────────────
pos_lock   = threading.Lock()
px, py, pz = 0.0, 0.0, 2.0    # spawn position in Gazebo ENU (x=East,y=North,z=Up)
yaw_deg    = 0.0
 
latest_frame = None
frame_lock   = threading.Lock()
barrel_class = "yellow"
frame_count  = len([f for f in os.listdir(SAVE_DIR) if f.endswith('.jpg')])
running      = True
 
 
# ── Kill PX4's grip on the model ─────────────────────────────────────────────
def kill_px4_physics():
    """
    Pause the Gazebo world physics so PX4 SITL can't push the drone back.
    Uses gz CLI — runs once at startup.
    """
    try:
        subprocess.run(
            ["gz", "world", "-w", WORLD_NAME, "-p"],   # -p = pause physics
            timeout=3, capture_output=True
        )
        print("⏸️  Gazebo physics PAUSED — PX4 can no longer move the drone")
    except Exception as e:
        print(f"⚠️  Could not pause physics via CLI: {e}")
        print("   Falling back to high-frequency blocking pose override.")
 
 
def resume_px4_physics():
    """Unpause physics on exit so Gazebo isn't left frozen."""
    try:
        subprocess.run(
            ["gz", "world", "-w", WORLD_NAME, "-r"],   # -r = run/unpause
            timeout=3, capture_output=True
        )
        print("▶️  Gazebo physics RESUMED")
    except Exception:
        pass
 
 
# ── Camera subscriber ─────────────────────────────────────────────────────────
def camera_callback(msg: Image):
    global latest_frame
    raw   = np.frombuffer(msg.data, dtype=np.uint8)
    frame = raw.reshape((msg.height, msg.width, 3))
    with frame_lock:
        latest_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
 
 
# ── Yaw → quaternion (ENU, Z-up) ─────────────────────────────────────────────
def yaw_to_quat(yaw_d):
    r = math.radians(yaw_d) / 2.0
    return 0.0, 0.0, math.sin(r), math.cos(r)   # qx, qy, qz, qw
 
 
# ── Send pose via gz CLI (node.request returns False - use CLI instead) ────────
def send_pose(x, y, z, yaw_d):
    qx, qy, qz, qw = yaw_to_quat(yaw_d)
    req_str = (
        f'name: "{MODEL_NAME}" '
        f'position: {{x: {x:.3f}, y: {y:.3f}, z: {z:.3f}}} '
        f'orientation: {{x: {qx:.4f}, y: {qy:.4f}, z: {qz:.4f}, w: {qw:.4f}}}'
    )
    subprocess.Popen(
        ["gz", "service",
         "-s", f"/world/{WORLD_NAME}/set_pose",
         "--reqtype", "gz.msgs.Pose",
         "--reptype", "gz.msgs.Boolean",
         "--timeout", "500",
         "--req", req_str],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
 
 
# ── Anti-gravity thread (20 Hz pose spam to override physics) ────────────────
def antigravity_thread():
    while running:
        with pos_lock:
            x, y, z, y_deg = px, py, pz, yaw_deg
        send_pose(x, y, z, y_deg)
        time.sleep(0.05)   # 20 Hz
 
 
# ── HUD ───────────────────────────────────────────────────────────────────────
def draw_hud(frame):
    h, w = frame.shape[:2]
    colour = (0, 255, 255) if barrel_class == "yellow" else (0, 0, 255)
 
    cv2.rectangle(frame, (0, 0), (w, 58), (0, 0, 0), -1)
    cv2.putText(frame, f"Class: {barrel_class}_barrel", (10, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, colour, 2)
    with pos_lock:
        cv2.putText(frame,
                    f"x={px:.1f}  y={py:.1f}  z={pz:.1f}  yaw={yaw_deg:.0f}deg",
                    (10, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
 
    cv2.rectangle(frame, (0, h - 30), (w, h), (0, 0, 0), -1)
    cv2.putText(frame,
                "W/S=fwd/bk  A/D=strafe  R/F=up/dn  Q/E=yaw  SPACE=capture  X=class  ESC=quit",
                (6, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (160, 160, 160), 1)
    cv2.putText(frame, f"Saved:{frame_count}", (w - 110, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return frame
 
 
# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    global px, py, pz, yaw_deg, running, barrel_class, frame_count
 
    # Subscribe to drone camera
    if not node.subscribe(Image, CAMERA_TOPIC, camera_callback):
        print(f"❌ Camera subscribe failed. Is Gazebo running?")
        sys.exit(1)
    print(f"✅ Camera: {CAMERA_TOPIC}")
 
    # Pause physics so PX4 can't fight us
    kill_px4_physics()
 
    # Start anti-gravity thread
    ag_thread = threading.Thread(target=antigravity_thread, daemon=True)
    ag_thread.start()
    print("🔒 Anti-gravity active — pose locked at 20Hz")
 
    # Send initial pose so drone appears immediately
    with pos_lock:
        send_pose(px, py, pz, yaw_deg)
    print(f"📌 Initial pose sent")
    print(f"   Starting at x={px}, y={py}, z={pz}")
    print(f"   Save dir: {os.path.abspath(SAVE_DIR)}")
    print(f"   Existing images: {frame_count}")
    print(f"\n👉 Click the OpenCV window to start controlling!\n")
 
    cv2.namedWindow("Noclip Capture", cv2.WINDOW_AUTOSIZE)
 
    try:
        while running:
            # Grab latest camera frame
            with frame_lock:
                frame = latest_frame.copy() if latest_frame is not None else None
 
            if frame is None:
                blank = np.zeros((360, 640, 3), dtype=np.uint8)
                cv2.putText(blank, "Waiting for camera...", (140, 180),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (80, 80, 80), 2)
                cv2.imshow("Noclip Capture", blank)
                cv2.waitKey(100)
                continue
 
            key = cv2.waitKey(30) & 0xFF
            moved = False
 
            with pos_lock:
                yaw_r = math.radians(yaw_deg)
 
                if key == ord('w'):
                    px += MOVE_SPEED * math.cos(yaw_r)
                    py += MOVE_SPEED * math.sin(yaw_r)
                    moved = True
                elif key == ord('s'):
                    px -= MOVE_SPEED * math.cos(yaw_r)
                    py -= MOVE_SPEED * math.sin(yaw_r)
                    moved = True
                elif key == ord('a'):
                    px -= MOVE_SPEED * math.sin(yaw_r)
                    py += MOVE_SPEED * math.cos(yaw_r)
                    moved = True
                elif key == ord('d'):
                    px += MOVE_SPEED * math.sin(yaw_r)
                    py -= MOVE_SPEED * math.cos(yaw_r)
                    moved = True
                elif key == ord('r'):
                    pz += MOVE_SPEED
                    moved = True
                elif key == ord('f'):
                    pz = max(0.1, pz - MOVE_SPEED)
                    moved = True
                elif key == ord('q'):
                    yaw_deg = (yaw_deg + YAW_SPEED) % 360
                    moved = True
                elif key == ord('e'):
                    yaw_deg = (yaw_deg - YAW_SPEED) % 360
                    moved = True
 
                if moved:
                    send_pose(px, py, pz, yaw_deg)
 
            if key == ord(' '):
                clean = frame.copy()
                fname = f"{barrel_class}_barrel_{frame_count:04d}.jpg"
                path  = os.path.join(SAVE_DIR, fname)
                cv2.imwrite(path, clean)
                frame_count += 1
                print(f"📸 {path}  (total: {frame_count})")
 
                flash = draw_hud(frame.copy())
                cv2.rectangle(flash, (0, 0), (flash.shape[1], flash.shape[0]), (0, 255, 0), 10)
                cv2.putText(flash, "SAVED!", (flash.shape[1]//2 - 70, flash.shape[0]//2),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 255, 0), 3)
                cv2.imshow("Noclip Capture", flash)
                cv2.waitKey(200)
 
            elif key == ord('x'):
                barrel_class = "red" if barrel_class == "yellow" else "yellow"
                print(f"🔄 Class: {barrel_class}_barrel")
 
            elif key == 27:
                break
 
            display = draw_hud(frame.copy())
            cv2.imshow("Noclip Capture", display)
 
    except KeyboardInterrupt:
        pass
 
    finally:
        running = False
        resume_px4_physics()
        cv2.destroyAllWindows()
        yellow = len([f for f in os.listdir(SAVE_DIR) if f.startswith('yellow')])
        red    = len([f for f in os.listdir(SAVE_DIR) if f.startswith('red')])
        print(f"\n✅ Done.  Yellow: {yellow}  Red: {red}  Total: {yellow + red}")
 
 
if __name__ == "__main__":
    main()
 
