"""
save_picture.py
================
Dataset capture tool. Click the OpenCV window and use keys to capture.
 
Controls:
  1 / 2 / 3    Switch class (yellow_barrel / red_barrel / null)
  SPACE        Capture current frame
  ESC          Quit
"""
 
import os
import time
import threading
import numpy as np
import cv2
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
 
# ── Config ────────────────────────────────────────────────────────────────────
CAMERA_TOPIC = "/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image"
SAVE_DIR     = "./dataset_raw"
# ─────────────────────────────────────────────────────────────────────────────
 
CLASSES = ['yellow_barrel', 'red_barrel', 'null']
CLASS_COLOURS = {
    'yellow_barrel': (0, 220, 220),
    'red_barrel':    (50,  50, 255),
    'null':          (160, 160, 160),
}
 
# Create folders
for cls in CLASSES:
    os.makedirs(os.path.join(SAVE_DIR, cls), exist_ok=True)
 
# Count existing images so we never overwrite
def count_existing(cls):
    d = os.path.join(SAVE_DIR, cls)
    return len([f for f in os.listdir(d) if f.endswith('.jpg')])
 
counts = {cls: count_existing(cls) for cls in CLASSES}
 
# ── Shared state ──────────────────────────────────────────────────────────────
latest_frame = None
frame_lock   = threading.Lock()
active_class = 'yellow_barrel'
 
 
# ── Camera callback ───────────────────────────────────────────────────────────
def image_callback(msg: Image):
    global latest_frame
    raw   = np.frombuffer(msg.data, dtype=np.uint8)
    frame = raw.reshape((msg.height, msg.width, 3))
    with frame_lock:
        latest_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
 
 
# ── HUD ───────────────────────────────────────────────────────────────────────
def draw_hud(frame, flash=False):
    h, w  = frame.shape[:2]
    out   = frame.copy()
    col   = CLASS_COLOURS[active_class]
 
    # Flash green border on capture
    if flash:
        cv2.rectangle(out, (0, 0), (w, h), (0, 255, 0), 16)
 
    # Top bar background
    cv2.rectangle(out, (0, 0), (w, 68), (15, 15, 15), -1)
 
    # Active class pill
    cv2.rectangle(out, (8, 7), (240, 44), col, -1)
    cv2.putText(out, active_class.upper(), (14, 33),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 2, cv2.LINE_AA)
 
    # Count for active class
    cv2.putText(out, f"captured: {counts[active_class]}", (250, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (200, 200, 200), 1, cv2.LINE_AA)
    cv2.putText(out, f"total: {sum(counts.values())}", (250, 48),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (130, 130, 130), 1, cv2.LINE_AA)
 
    # Per-class counts top right
    for i, cls in enumerate(CLASSES):
        c   = CLASS_COLOURS[cls]
        mrk = " ◀" if cls == active_class else ""
        cv2.putText(out, f"{cls}: {counts[cls]}{mrk}",
                    (w - 235, 20 + i * 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    c if cls == active_class else (100, 100, 100),
                    1, cv2.LINE_AA)
 
    # Bottom bar
    cv2.rectangle(out, (0, h - 30), (w, h), (15, 15, 15), -1)
    cv2.putText(out, "1=yellow  2=red  3=null  |  P=capture  ESC=quit",
                (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.42, (120, 120, 120), 1, cv2.LINE_AA)
 
    # SAVED flash text
    if flash:
        cv2.putText(out, "SAVED!", (w // 2 - 75, h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.8, (0, 255, 0), 4, cv2.LINE_AA)
 
    return out
 
 
# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    global active_class
 
    node = Node()
    if not node.subscribe(Image, CAMERA_TOPIC, image_callback):
        print(f"❌ Could not subscribe to {CAMERA_TOPIC}. Is Gazebo running?")
        return
 
    print(f"✅ Camera live.")
    print(f"📁 Saving to: {os.path.abspath(SAVE_DIR)}/")
    print(f"\n   Existing images: { {cls: counts[cls] for cls in CLASSES} }")
    print(f"\n   Click the OpenCV window then:")
    print(f"   1=yellow  2=red  3=null  P=capture  ESC=quit\n")
 
    cv2.namedWindow("Dataset Capture", cv2.WINDOW_AUTOSIZE)
    flash_until = 0.0
 
    while True:
        with frame_lock:
            frame = latest_frame.copy() if latest_frame is not None else None
 
        if frame is None:
            blank = np.zeros((360, 640, 3), dtype=np.uint8)
            cv2.putText(blank, "Waiting for camera...", (140, 180),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (80, 80, 80), 2)
            cv2.imshow("Dataset Capture", blank)
            key = cv2.waitKey(30) & 0xFF
        else:
            flash   = time.time() < flash_until
            display = draw_hud(frame, flash=flash)
            cv2.imshow("Dataset Capture", display)
            key = cv2.waitKey(30) & 0xFF
 
        # ── Key handling ──────────────────────────────────────────────────────
        if key == ord('1'):
            active_class = 'yellow_barrel'
            print(f"  🔷 Class → yellow_barrel")
 
        elif key == ord('2'):
            active_class = 'red_barrel'
            print(f"  🔴 Class → red_barrel")
 
        elif key == ord('3'):
            active_class = 'null'
            print(f"  ⬜ Class → null")
 
        elif key == ord('p') and frame is not None:
            idx   = counts[active_class]
            fname = f"{idx:05d}.jpg"
            path  = os.path.join(SAVE_DIR, active_class, fname)
            cv2.imwrite(path, frame)   # save clean frame, no HUD
            counts[active_class] += 1
            flash_until = time.time() + 0.3
            print(f"  📸 [{active_class}] {fname}  "
                  f"(class total: {counts[active_class]}  overall: {sum(counts.values())})")
 
        elif key == 27:   # ESC
            break
 
    cv2.destroyAllWindows()
    print(f"\n✅ Session done:")
    for cls in CLASSES:
        print(f"   {cls:<16} {counts[cls]} images")
    print(f"   {'TOTAL':<16} {sum(counts.values())} images")
    print(f"\n   Saved to: {os.path.abspath(SAVE_DIR)}/")
 
 
if __name__ == "__main__":
    main()
