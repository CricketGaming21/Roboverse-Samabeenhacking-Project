"""
barrel_detect.py
=================
Live barrel detection while the drone is flying.

Architecture (3 independent threads so nothing blocks anything):

  [Gazebo camera topic]
         │
         ▼
  Thread 1 – Camera callback   →  raw_frame_queue (maxsize=1, always latest)
         │
         ▼
  Thread 2 – YOLO Inference    →  display_queue   (maxsize=1, always latest)
         │
         ▼
  Main thread – OpenCV display  (handles waitKey so the GUI stays alive)

  maxsize=1 on every queue means we ALWAYS show the newest frame and never
  build up a backlog — essential for a real-time feel.

Usage:
    python3 barrel_detect.py
    python3 barrel_detect.py --model /path/to/best.pt --conf 0.4 --device cuda:0

Controls (click the OpenCV window):
    Q / ESC   Quit
    S         Save current annotated frame to ./detections/
    +  /  -   Raise / lower confidence threshold by 0.05
"""

import argparse
import os
import time
import queue
import threading
import numpy as np
import cv2
from ultralytics import YOLO
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

# ── Topic — change if your drone/world differs ─────────────────────────────────
CAMERA_TOPIC  = "/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image"

DEFAULT_MODEL = "best.pt"
DEFAULT_CONF  = 0.35
SAVE_DIR      = "./detections"
WIN_NAME      = "Barrel Detector – best.pt  |  Q/ESC=quit  S=save  +/-=conf"

# ── Shared queues ──────────────────────────────────────────────────────────────
raw_queue  = queue.Queue(maxsize=1)   # raw BGR frames from camera
det_queue  = queue.Queue(maxsize=1)   # annotated BGR frames from YOLO

# ── Mutable conf (thread-safe via a list so it's mutable from main) ────────────
_conf   = [DEFAULT_CONF]
_stop   = threading.Event()


# ══════════════════════════════════════════════════════════════════════════════
#  THREAD 1  —  Camera callback (called by Gazebo transport, not our thread)
# ══════════════════════════════════════════════════════════════════════════════
def _camera_cb(msg: Image):
    try:
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        bgr   = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        # Non-blocking put: drop old frame, insert new one
        try:
            raw_queue.put_nowait(bgr)
        except queue.Full:
            try:
                raw_queue.get_nowait()   # discard stale frame
            except queue.Empty:
                pass
            raw_queue.put_nowait(bgr)
    except Exception as e:
        print(f"[CAM] Decode error: {e}")


# ══════════════════════════════════════════════════════════════════════════════
#  THREAD 2  —  YOLO inference
# ══════════════════════════════════════════════════════════════════════════════
def inference_thread(model: YOLO):
    fps_t      = time.monotonic()
    fps_frames = 0
    fps_val    = 0.0

    while not _stop.is_set():
        try:
            frame = raw_queue.get(timeout=0.5)
        except queue.Empty:
            continue

        conf = _conf[0]

        # Run detection
        results   = model(frame, conf=conf, verbose=False)
        annotated = results[0].plot()          # draws boxes + labels on a copy

        # Count per class for HUD
        det_counts = {}
        for box in results[0].boxes:
            cls_name = model.names[int(box.cls[0])]
            det_counts[cls_name] = det_counts.get(cls_name, 0) + 1

        # FPS (of inference, not camera)
        fps_frames += 1
        if fps_frames >= 10:
            fps_val    = fps_frames / (time.monotonic() - fps_t)
            fps_t      = time.monotonic()
            fps_frames = 0

        # Build HUD on the annotated frame
        h, w = annotated.shape[:2]

        # Top bar
        cv2.rectangle(annotated, (0, 0), (w, 44), (20, 20, 20), -1)

        cv2.putText(annotated, f"FPS {fps_val:4.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 120), 2)

        cv2.putText(annotated, f"Conf {conf:.2f}", (145, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 200, 0), 2)

        if det_counts:
            summary = "   ".join(f"{k}: {v}" for k, v in det_counts.items())
            colour  = (60, 255, 60)
        else:
            summary = "No barrels detected"
            colour  = (120, 120, 220)

        cv2.putText(annotated, summary, (310, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2)

        # Push to display queue (drop old if display is slow)
        try:
            det_queue.put_nowait((annotated, det_counts, fps_val))
        except queue.Full:
            try:
                det_queue.get_nowait()
            except queue.Empty:
                pass
            det_queue.put_nowait((annotated, det_counts, fps_val))


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN THREAD  —  OpenCV display  (waitKey MUST run on the main thread)
# ══════════════════════════════════════════════════════════════════════════════
def display_loop():
    os.makedirs(SAVE_DIR, exist_ok=True)
    save_count   = 0
    last_display = None   # keep showing last frame if inference is momentarily busy

    cv2.namedWindow(WIN_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN_NAME, 960, 540)

    while not _stop.is_set():
        # Pull latest annotated frame (non-blocking)
        try:
            last_display, _, _ = det_queue.get_nowait()
        except queue.Empty:
            pass   # keep showing previous frame — no stutter

        if last_display is not None:
            cv2.imshow(WIN_NAME, last_display)
        else:
            # Waiting screen before first frame arrives
            blank = np.zeros((540, 960, 3), dtype=np.uint8)
            cv2.putText(blank, "Waiting for camera feed ...", (200, 270),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.1, (100, 200, 255), 2)
            cv2.putText(blank, "Make sure the simulation is running.", (180, 320),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (160, 160, 160), 1)
            cv2.imshow(WIN_NAME, blank)

        key = cv2.waitKey(15) & 0xFF   # 15 ms → ~66 fps display cap

        if key in (ord('q'), 27):      # Q or ESC
            print("[INFO] Quit requested.")
            _stop.set()
            break

        elif key == ord('s') and last_display is not None:
            save_count += 1
            ts   = int(time.time())
            path = os.path.join(SAVE_DIR, f"detection_{save_count:04d}_{ts}.jpg")
            cv2.imwrite(path, last_display)
            print(f"[SAVE] Saved → {path}")

        elif key in (ord('+'), ord('=')):
            _conf[0] = min(0.95, round(_conf[0] + 0.05, 2))
            print(f"[CONF] → {_conf[0]:.2f}")

        elif key == ord('-'):
            _conf[0] = max(0.05, round(_conf[0] - 0.05, 2))
            print(f"[CONF] → {_conf[0]:.2f}")

    cv2.destroyAllWindows()


# ══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(description="Live barrel detection from Gazebo drone camera")
    parser.add_argument("--model",  default=DEFAULT_MODEL, help="Path to YOLO weights (default: best.pt)")
    parser.add_argument("--conf",   default=DEFAULT_CONF,  type=float, help="Confidence threshold (default: 0.35)")
    parser.add_argument("--topic",  default=CAMERA_TOPIC,  help="Gazebo image topic")
    parser.add_argument("--device", default="cpu",         help="Inference device: cpu or cuda:0")
    args = parser.parse_args()

    _conf[0] = args.conf

    # Load YOLO
    print(f"[MODEL] Loading {args.model} on {args.device} ...")
    model = YOLO(args.model)
    model.to(args.device)
    print(f"[MODEL] Classes: {list(model.names.values())}")
    print(f"[MODEL] Confidence threshold: {_conf[0]:.2f}")

    # Subscribe to Gazebo camera
    node = Node()
    ok   = node.subscribe(Image, args.topic, _camera_cb)
    if not ok:
        print(f"\n[ERROR] Could not subscribe to:\n  {args.topic}")
        print("Check the topic with:  gz topic -l | grep image")
        return
    print(f"[CAM]   Subscribed to {args.topic}")
    print(f"\nControls: Q/ESC=quit  S=save  +/-=confidence\n")

    # Start inference thread
    inf_t = threading.Thread(target=inference_thread, args=(model,), daemon=True)
    inf_t.start()

    # Display loop runs on main thread (required by OpenCV)
    try:
        display_loop()
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted.")
        _stop.set()

    inf_t.join(timeout=3)
    print("[INFO] Done.")


if __name__ == "__main__":
    main()
