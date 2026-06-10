#!/usr/bin/env python3
"""
live_view.py  —  HULA HG-F09  Live Camera Feed + Screenshot Tool
=================================================================
Streams the drone's front camera to your PC without arming or
taking off.  The drone stays flat on the ground the entire time.

Usage:
    python live_view.py                      # uses default IP 192.168.100.1
    python live_view.py 192.168.100.1        # explicit IP (single drone)
    python live_view.py 192.168.100.2        # different drone in the swarm

Controls inside the window:
    SPACE  →  Save screenshot  (./screenshots/screenshot_YYYYMMDD_HHMMSS.jpg)
    Q      →  Quit cleanly

Requirements:
    pip install "pyhulax[video]"   (PyAV + OpenCV pulled in automatically)

Notes:
    • Your laptop MUST be connected to the drone's WiFi before running.
    • For the cage: plug the Ethernet cable from the cage WiFi router into
      your laptop (set IP back to DHCP first if you were using Ethernet
      to the mapping drone).
    • The drone does NOT need to be armed or airborne.
"""

import sys
import os
import time
import cv2
from datetime import datetime

# ── Configuration ─────────────────────────────────────────────────────────────
DRONE_IP    = sys.argv[1] if len(sys.argv) > 1 else "192.168.100.1"
SAVE_DIR    = "screenshots"
WINDOW_NAME = "HULA Live Feed  |  SPACE = Screenshot    Q = Quit"

# Overlay text settings
OVERLAY_TEXT   = "SPACE = Screenshot  |  Q = Quit"
OVERLAY_FONT   = cv2.FONT_HERSHEY_SIMPLEX
OVERLAY_SCALE  = 0.60
OVERLAY_COLOUR = (0, 255, 0)   # bright green (BGR)
OVERLAY_THICK  = 2

# Flash message shown after a screenshot is saved  (displayed for N frames)
FLASH_FRAMES   = 30   # ~1 second at 30 fps


# ── Helpers ───────────────────────────────────────────────────────────────────

def ensure_save_dir() -> None:
    """Create the screenshots directory if it does not exist."""
    os.makedirs(SAVE_DIR, exist_ok=True)
    print(f"[INFO] Screenshots folder: {os.path.abspath(SAVE_DIR)}")


def save_screenshot(image) -> str | None:
    """
    Write *image* (BGR numpy array) to SAVE_DIR with a timestamp filename.
    Returns the saved path on success, None on failure.
    """
    ts       = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    filename = os.path.join(SAVE_DIR, f"screenshot_{ts}.jpg")
    ok       = cv2.imwrite(filename, image)
    if ok:
        print(f"[SCREENSHOT] ✓  Saved: {filename}")
        return filename
    else:
        print(f"[SCREENSHOT] ✗  Failed to write: {filename}")
        return None


def draw_overlay(frame_bgr, flash_msg: str | None = None):
    """
    Draw the control-hint overlay (and optional flash message) directly onto
    a copy of *frame_bgr*.  Returns the annotated copy without modifying the
    original.
    """
    out = frame_bgr.copy()
    h, w = out.shape[:2]

    # ── bottom-left hint ──
    cv2.putText(
        out, OVERLAY_TEXT,
        (10, h - 12),
        OVERLAY_FONT, OVERLAY_SCALE,
        OVERLAY_COLOUR, OVERLAY_THICK, cv2.LINE_AA,
    )

    # ── optional flash message (top-centre, yellow) ──
    if flash_msg:
        (tw, _), _ = cv2.getTextSize(flash_msg, OVERLAY_FONT, OVERLAY_SCALE + 0.1, 2)
        cx = max(0, (w - tw) // 2)
        cv2.putText(
            out, flash_msg,
            (cx, 32),
            OVERLAY_FONT, OVERLAY_SCALE + 0.1,
            (0, 220, 255),   # yellow-ish (BGR)
            2, cv2.LINE_AA,
        )

    return out


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    ensure_save_dir()

    # ── Step 1: Connect to drone (no flight commands) ─────────────────────
    from pyhulax import DroneAPI
    drone = DroneAPI()

    print(f"[INFO] Connecting to HULA drone at {DRONE_IP} ...")
    connected = drone.robust_connect(DRONE_IP)
    if not connected:
        print(
            "[ERROR] Could not reach the drone.\n"
            "        • Is your laptop on the drone's WiFi network?\n"
            f"        • Is the drone IP correct? (using: {DRONE_IP})\n"
            "        • Is the drone powered on?"
        )
        sys.exit(1)
    print("[INFO] Connected.")

    # ── Step 2: Enable RTP video stream on the drone side ─────────────────
    print("[INFO] Enabling video stream on drone ...")
    drone.set_video_stream(True)
    time.sleep(0.5)     # brief pause — drone needs a moment to start the stream

    # ── Step 3: Open the stream and display loop ───────────────────────────
    from pyhulax.video import VideoStreamSimple

    stream = VideoStreamSimple(DRONE_IP)
    print(f"[INFO] Stream started. Window: '{WINDOW_NAME}'")
    print(f"[INFO] Press SPACE to take a screenshot.  Press Q to quit.\n")

    flash_msg       = None
    flash_countdown = 0

    try:
        for frame in stream:
            # ── build display frame with overlay ──
            if flash_countdown > 0:
                display = draw_overlay(frame.image, flash_msg)
                flash_countdown -= 1
            else:
                flash_msg = None
                display   = draw_overlay(frame.image)

            cv2.imshow(WINDOW_NAME, display)

            # ── key handling ──
            key = cv2.waitKey(1) & 0xFF

            if key in (ord('q'), ord('Q'), 27):   # Q or ESC
                print("[INFO] Quit requested.")
                break

            elif key == 32:   # SPACE bar
                saved = save_screenshot(frame.image)
                if saved:
                    fname         = os.path.basename(saved)
                    flash_msg     = f"Saved: {fname}"
                    flash_countdown = FLASH_FRAMES
                else:
                    flash_msg     = "Save FAILED — check console"
                    flash_countdown = FLASH_FRAMES

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C — stopping ...")

    finally:
        # ── clean shutdown ─────────────────────────────────────────────────
        print("[INFO] Stopping stream ...")
        stream.close()
        drone.set_video_stream(False)
        drone.disconnect()
        cv2.destroyAllWindows()
        print("[INFO] Done.  Goodbye.")


if __name__ == "__main__":
    main()
