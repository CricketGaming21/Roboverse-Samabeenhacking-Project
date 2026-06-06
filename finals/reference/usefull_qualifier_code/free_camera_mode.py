"""
free_camera_mode.py
====================
Freezes the drone in place in Gazebo so you can roam the free camera
freely and capture barrel images for your dataset.

HOW TO USE:
-----------
1. Start Gazebo + PX4 SITL as normal (do NOT arm or takeoff)
2. Run this script: python3 free_camera_mode.py
3. In Gazebo:
   - Orbit:  Left-click drag
   - Pan:    Middle-click drag (or Shift + left-click drag)
   - Zoom:   Scroll wheel
4. Press SPACE in the terminal to capture the current drone camera frame
   Press Q to quit and restore the drone

WHAT IT DOES:
-------------
- Pins the drone to a fixed pose so PX4 can't move it
- Subscribes to the drone camera so you can still capture frames
- Lets you roam Gazebo's free camera independently
"""

import time
import sys
import os
import threading
import numpy as np
import cv2

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_v_pb2 import Pose_V

# ── Config ─────────────────────────────────────────────────────────────────
WORLD_NAME   = "roboverse"
MODEL_NAME   = "x500_vision_0"
CAMERA_TOPIC = f"/world/{WORLD_NAME}/model/{MODEL_NAME}/link/camera_link/sensor/IMX214/image"
SET_POSE_SVC = f"/world/{WORLD_NAME}/set_pose"

# Where to freeze the drone (metres, Gazebo ENU frame: x=East, y=North, z=Up)
# Adjust if your drone spawns elsewhere — read from Gazebo Entity Inspector
FREEZE_X = 0.0
FREEZE_Y = 0.0
FREEZE_Z = 0.15   # just above ground so it doesn't clip

# Output folder for captured images
SAVE_DIR     = "./dataset_raw"
BARREL_CLASS = "yellow"   # change to "red" when shooting red barrels
# ───────────────────────────────────────────────────────────────────────────

os.makedirs(SAVE_DIR, exist_ok=True)
node = Node()

latest_frame = None
frame_lock   = threading.Lock()
frame_count  = 0
keep_pinning = True


# ── Camera subscriber ───────────────────────────────────────────────────────
def camera_callback(msg: Image):
    global latest_frame
    raw   = np.frombuffer(msg.data, dtype=np.uint8)
    frame = raw.reshape((msg.height, msg.width, 3))
    bgr   = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    with frame_lock:
        latest_frame = bgr


# ── Pose-pinning thread ─────────────────────────────────────────────────────
def pin_drone():
    """
    Repeatedly calls /world/<world>/set_pose to lock the drone in place.
    PX4 SITL will try to move the model; we override it at ~10 Hz.
    """
    req = Pose()
    req.name = MODEL_NAME
    req.position.x = FREEZE_X
    req.position.y = FREEZE_Y
    req.position.z = FREEZE_Z
    # Identity quaternion — keep drone level
    req.orientation.w = 1.0
    req.orientation.x = 0.0
    req.orientation.y = 0.0
    req.orientation.z = 0.0

    print(f"📌 Pinning {MODEL_NAME} at ({FREEZE_X}, {FREEZE_Y}, {FREEZE_Z})...")

    while keep_pinning:
        rep = Boolean()
        timeout = 500   # ms
        node.request(SET_POSE_SVC, req, Boolean, Boolean, timeout, rep)
        time.sleep(0.1)   # 10 Hz override

    print("📌 Pose pinning stopped.")


# ── Main ────────────────────────────────────────────────────────────────────
def main():
    global keep_pinning, frame_count, BARREL_CLASS

    # Subscribe to drone camera
    if node.subscribe(Image, CAMERA_TOPIC, camera_callback):
        print(f"✅ Subscribed to drone camera: {CAMERA_TOPIC}")
    else:
        print(f"❌ Could not subscribe to {CAMERA_TOPIC}. Is Gazebo running?")
        sys.exit(1)

    # Start pinning thread
    pin_thread = threading.Thread(target=pin_drone, daemon=True)
    pin_thread.start()

    print("\n" + "="*55)
    print("  FREE CAMERA MODE ACTIVE")
    print("="*55)
    print(f"  Capturing class : {BARREL_CLASS}_barrel")
    print(f"  Save directory  : {os.path.abspath(SAVE_DIR)}")
    print("-"*55)
    print("  SPACE  → capture current drone-camera frame")
    print("  C      → toggle barrel class (yellow / red)")
    print("  Q      → quit")
    print("="*55)
    print("\nMove the Gazebo free camera around the barrels now.\n")

    try:
        while True:
            key = input("Command (SPACE=capture, C=class, Q=quit): ").strip().lower()

            if key == "" or key == " ":
                with frame_lock:
                    frame = latest_frame.copy() if latest_frame is not None else None
                if frame is None:
                    print("  ⚠️  No frame received yet — is the camera topic publishing?")
                    continue
                fname = f"{BARREL_CLASS}_barrel_{frame_count:04d}.jpg"
                path  = os.path.join(SAVE_DIR, fname)
                cv2.imwrite(path, frame)
                frame_count += 1
                print(f"  📸 Saved: {path}  (total: {frame_count})")

            elif key == "c":
                BARREL_CLASS = "red" if BARREL_CLASS == "yellow" else "yellow"
                print(f"  🔄 Switched to: {BARREL_CLASS}_barrel")

            elif key == "q":
                break

            else:
                print("  Unknown command. Use SPACE, C, or Q.")

    except KeyboardInterrupt:
        pass

    finally:
        keep_pinning = False
        pin_thread.join(timeout=2)
        cv2.destroyAllWindows()
        print(f"\n✅ Done. {frame_count} images saved to {os.path.abspath(SAVE_DIR)}/")


if __name__ == "__main__":
    main()