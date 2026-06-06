import time
import numpy as np
import cv2
import os
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

# ── Config ──────────────────────────────────────────────────────────────────
MAX_PHOTOS = 2000
save_dir   = "captured_images"
os.makedirs(save_dir, exist_ok=True)

# ── Shared state ─────────────────────────────────────────────────────────────
latest_frame = None
photo_count  = 400

def image_callback(msg: Image):
    global latest_frame
    frame = np.frombuffer(msg.data, dtype=np.uint8)
    frame = frame.reshape((msg.height, msg.width, 3))
    latest_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

def main():
    global photo_count

    node = Node()
    image_topic = "/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image"

    if node.subscribe(Image, image_topic, image_callback):
        print(f"Subscribed to {image_topic}")
        print("📸 Press SPACE to take a photo | Q to quit")
        print(f"Max photos: {MAX_PHOTOS}\n")
    else:
        print("Failed to subscribe. Is Gazebo running?")
        return

    while True:
        if latest_frame is not None:
            # Show live feed with photo count overlay
            display = latest_frame.copy()
            cv2.putText(display, f"Photos: {photo_count}/{MAX_PHOTOS}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(display, "SPACE=Capture  Q=Quit",
                        (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.imshow("Drone Camera", display)

        key = cv2.waitKey(1) & 0xFF

        # SPACE to capture
        if key == ord(' '):
            if latest_frame is None:
                print("No frame received yet, wait a moment...")
            elif photo_count >= MAX_PHOTOS:
                print(f"Maximum of {MAX_PHOTOS} photos reached!")
            else:
                filename = os.path.join(save_dir, f"photo_{photo_count+1:04d}.jpg")
                cv2.imwrite(filename, latest_frame)
                photo_count += 1
                print(f"📸 Saved: {filename}  ({photo_count}/{MAX_PHOTOS})")

        # Q to quit
        elif key == ord('q'):
            print(f"\nDone. {photo_count} photos saved to '{save_dir}/'")
            break

        # Auto-quit when max photos reached
        if photo_count >= MAX_PHOTOS:
            print(f"\n✅ {MAX_PHOTOS} photos captured! Saved to '{save_dir}/'")
            break

        time.sleep(0.01)

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
