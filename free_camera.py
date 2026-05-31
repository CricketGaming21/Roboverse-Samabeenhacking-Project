"""
free_camera.py
==============
Detaches the Gazebo GUI camera from the drone and lets you freely
move the viewpoint using keyboard controls.

How it works
------------
Gazebo Harmonic exposes a GUI camera-pose topic:
  /gui/camera/pose
Publishing a gz.msgs10.Pose message to this topic immediately
repositions the camera and breaks the "follow" lock.

Controls (press in the terminal window)
---------------------------------------
  W / S        — move forward / backward  (X axis)
  A / D        — strafe left / right       (Y axis)
  Q / E        — move up / down            (Z axis)
  Arrow Up/Dn  — pitch up / down
  Arrow Lt/Rt  — yaw left / right
  R            — reset to default overview position
  ESC / Ctrl-C — quit

Requirements
------------
  pip install gz-transport13 gz-msgs10   (or whatever version is installed)
  The script must run on the same machine as Gazebo (or with GZ_IP set).

Usage
-----
  python free_camera.py
"""

import sys
import math
import time
import threading

# ── Gazebo transport ──────────────────────────────────────────────────────────
try:
    from gz.transport13 import Node
    from gz.msgs10.pose_pb2 import Pose
    from gz.msgs10.quaternion_pb2 import Quaternion
    from gz.msgs10.vector3d_pb2 import Vector3d
except ImportError:
    print("[ERROR] gz-transport / gz-msgs Python bindings not found.")
    print("        Make sure you are running inside the PX4 VM or have")
    print("        the gz-transport13 Python package installed.")
    sys.exit(1)

# ── Optional: keyboard input (falls back to simple input() if not available) ──
try:
    import tty
    import termios
    HAS_TTY = True
except ImportError:
    HAS_TTY = False  # Windows fallback


# ─────────────────────────────────────────────────────────────────────────────
# Camera state
# ─────────────────────────────────────────────────────────────────────────────
# Start position: 8 m back, 5 m up, looking slightly downward at the origin
cam = {
    "x":   0.0,   # metres  (forward = +X in Gazebo world frame)
    "y":  -8.0,   # metres  (left    = +Y)
    "z":   5.0,   # metres  (up      = +Z)
    "yaw":  1.5708,  # radians  (facing +X, i.e. 90° from default Gazebo Y-forward)
    "pitch": -0.3,   # radians  (slight downward tilt)
}

# Movement step sizes
MOVE_STEP  = 0.5   # metres per key press
ANGLE_STEP = 0.05  # radians per key press

# Gazebo GUI camera topic
CAMERA_TOPIC = "/gui/camera/pose"


# ─────────────────────────────────────────────────────────────────────────────
# Quaternion helper
# ─────────────────────────────────────────────────────────────────────────────
def euler_to_quat(roll: float, pitch: float, yaw: float):
    """Convert roll/pitch/yaw (radians) → (qx, qy, qz, qw)."""
    cr, sr = math.cos(roll / 2),  math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2),   math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,   # qx
        cr * sp * cy + sr * cp * sy,   # qy
        cr * cp * sy - sr * sp * cy,   # qz
        cr * cp * cy + sr * sp * sy,   # qw
    )


# ─────────────────────────────────────────────────────────────────────────────
# Publish camera pose
# ─────────────────────────────────────────────────────────────────────────────
def publish_camera(node: Node, publisher):
    qx, qy, qz, qw = euler_to_quat(0.0, cam["pitch"], cam["yaw"])

    msg = Pose()
    msg.position.CopyFrom(Vector3d(x=cam["x"], y=cam["y"], z=cam["z"]))
    msg.orientation.CopyFrom(Quaternion(x=qx, y=qy, z=qz, w=qw))

    publisher.publish(msg)


# ─────────────────────────────────────────────────────────────────────────────
# Move camera in the direction it is facing (forward/strafe)
# ─────────────────────────────────────────────────────────────────────────────
def move_local(dx: float, dy: float):
    """Move dx forward and dy left relative to current yaw."""
    yaw = cam["yaw"]
    cam["x"] += dx * math.cos(yaw) - dy * math.sin(yaw)
    cam["y"] += dx * math.sin(yaw) + dy * math.cos(yaw)


# ─────────────────────────────────────────────────────────────────────────────
# Key → action mapping
# ─────────────────────────────────────────────────────────────────────────────
def handle_key(ch: str):
    """Return False to quit, True to continue."""
    if ch in ("w", "W"):
        move_local(MOVE_STEP, 0)
    elif ch in ("s", "S"):
        move_local(-MOVE_STEP, 0)
    elif ch in ("a", "A"):
        move_local(0, MOVE_STEP)
    elif ch in ("d", "D"):
        move_local(0, -MOVE_STEP)
    elif ch in ("q", "Q"):
        cam["z"] += MOVE_STEP
    elif ch in ("e", "E"):
        cam["z"] -= MOVE_STEP
    elif ch == "UP":
        cam["pitch"] = max(-math.pi / 2 + 0.01, cam["pitch"] - ANGLE_STEP)
    elif ch == "DOWN":
        cam["pitch"] = min(math.pi / 2 - 0.01, cam["pitch"] + ANGLE_STEP)
    elif ch == "LEFT":
        cam["yaw"] += ANGLE_STEP
    elif ch == "RIGHT":
        cam["yaw"] -= ANGLE_STEP
    elif ch in ("r", "R"):
        cam.update({"x": 0.0, "y": -8.0, "z": 5.0,
                    "yaw": 1.5708, "pitch": -0.3})
    elif ch in ("\x1b", "\x03"):   # ESC or Ctrl-C
        return False
    return True


# ─────────────────────────────────────────────────────────────────────────────
# Read a single key (Linux/macOS)
# ─────────────────────────────────────────────────────────────────────────────
def read_key_unix():
    """Block until one key is pressed; decode arrow keys."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            ch2 = sys.stdin.read(1)
            if ch2 == "[":
                ch3 = sys.stdin.read(1)
                return {"A": "UP", "B": "DOWN",
                        "C": "RIGHT", "D": "LEFT"}.get(ch3, "")
            return "\x1b"
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ─────────────────────────────────────────────────────────────────────────────
# Fallback: simple line-based input (Windows / no tty)
# ─────────────────────────────────────────────────────────────────────────────
def read_key_fallback():
    line = input("Key (wasd/qe/arrows=UDLR/r/ESC): ").strip()
    return line[0] if line else ""


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────
def main():
    node = Node()

    print("[INFO] Advertising on", CAMERA_TOPIC)
    publisher = node.advertise(CAMERA_TOPIC, Pose)
    if not publisher:
        print("[ERROR] Could not advertise camera pose topic.")
        print("        Is Gazebo Harmonic running?")
        sys.exit(1)

    # Give Gazebo a moment to register the publisher
    time.sleep(0.5)

    # Immediately publish to break the follow-cam lock
    publish_camera(node, publisher)
    print("[INFO] Camera detached from drone.")
    print()
    print("  W/S        — forward / backward")
    print("  A/D        — strafe left / right")
    print("  Q/E        — up / down")
    print("  Arrow keys — pitch / yaw")
    print("  R          — reset view")
    print("  ESC        — quit")
    print()

    read_key = read_key_unix if HAS_TTY else read_key_fallback

    while True:
        key = read_key()
        if not handle_key(key):
            print("\n[INFO] Exiting free camera control.")
            break
        publish_camera(node, publisher)

        # Print current position for reference
        print(f"\r  pos=({cam['x']:+.1f}, {cam['y']:+.1f}, {cam['z']:+.1f})  "
              f"yaw={math.degrees(cam['yaw']):+.1f}°  "
              f"pitch={math.degrees(cam['pitch']):+.1f}°   ",
              end="", flush=True)


if __name__ == "__main__":
    main()
