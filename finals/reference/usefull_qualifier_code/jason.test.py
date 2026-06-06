#!/usr/bin/env python3
"""
=============================================================
BRAINHACK 2026 - ROBOVERSE FLIGHT CHALLENGE
QUALIFIER RUN - COMPLETE COMPETITION FILE
=============================================================
HOW TO RUN:
1. Start the simulator with ./start_px4.sh (x500_vision, roboverse world)
2. Open a NEW terminal
3. Navigate to this file: cd ~/Desktop/codes
4. Run: python3 qualifier_run.py
"""
import asyncio
import numpy as np
import time
import math
import threading
import cv2
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from depth_receiver import DepthReceiver
from drone_control import Drone
from AvoidancePlanner import AvoidancePlanner
from get_position_with_task import SharedState, position_monitor_task
from Detector import Detector

# =============================================================
# CONFIGURATION - Adjust these if needed
# =============================================================
DEPTH_TOPIC = "/depth_camera"
CAMERA_TOPIC = "/world/roboverse/model/x500_vision_0/link/camera_link/sensor/IMX214/image"
MODEL_PATH = "yolov10n.pt"
DETECTION_CONFIDENCE = 0.5

TAKEOFF_ALTITUDE_M = 2.0 
FLIGHT_DOWN = -2.0 
LOOP_HZ = 10.0 

LANE_WIDTH_M = 3.0 
NUM_LANES = 6 
LANE_LENGTH_M = 15.0 

SAFE_DISTANCE_M = 3.5 
CRITICAL_DISTANCE_M = 1.2 
STEP_SIZE_M = 1.5 

K = np.array([[433.0, 0.0, 320.0],
              [ 0.0, 433.0, 240.0],
              [ 0.0, 0.0, 1.0]])

# =============================================================
# CANISTER LOG
# =============================================================
class CanisterLog:
    def __init__(self):
        self.detections = []
        self.lock = threading.Lock()

    def add(self, north, east, down, class_name, confidence, photo_path):
        with self.lock:
            entry = {
                "time": time.strftime("%H:%M:%S"),
                "north": north,
                "east": east,
                "down": down,
                "class": class_name,
                "conf": confidence,
                "photo": photo_path
            }
            self.detections.append(entry)
            print(f"\n*** CANISTER DETECTED ***")
            print(f" Class: {class_name} ({confidence:.0%} confidence)")
            print(f" Position: N={north:.2f}m E={east:.2f}m")
            print(f" Photo: {photo_path}")
            print(f"*************************\n")

    def save_report(self, filename="canister_log.txt"):
        with self.lock:
            with open(filename, "w") as f:
                f.write("=" * 50 + "\n")
                f.write(" BRAINHACK 2026 - CANISTER DETECTION REPORT\n")
                f.write("=" * 50 + "\n\n")
                f.write(f"Total canisters found: {len(self.detections)}\n\n")
                for i, d in enumerate(self.detections):
                    f.write(f"Canister #{i+1}\n")
                    f.write(f" Time: {d['time']}\n")
                    f.write(f" Class: {d['class']} ({d['conf']:.0%})\n")
                    f.write(f" North: {d['north']:.3f} m\n")
                    f.write(f" East: {d['east']:.3f} m\n")
                    f.write(f" Photo: {d['photo']}\n\n")
            print(f"\nReport saved to {filename}")
            print(f"Total canisters found: {len(self.detections)}")

# =============================================================
# LAWNMOWER WAYPOINT GENERATOR
# =============================================================
def generate_lawnmower_waypoints(num_lanes, lane_length, lane_width):
    waypoints = []
    for lane in range(num_lanes):
        east = lane * lane_width
        if lane % 2 == 0:
            waypoints.append((0.0, east))
            waypoints.append((lane_length, east))
        else:
            waypoints.append((lane_length, east))
            waypoints.append((0.0, east))
    return waypoints

# =============================================================
# MAIN DRONE NAVIGATION CLASS
# =============================================================
class QualifierRun:
    def __init__(self):
        self.running = True
        self.pose = {"north": 0.0, "east": 0.0, "down": FLIGHT_DOWN, "yaw": 0.0, "yaw_deg": 0.0}
        self.target_yaw_deg = 0.0
        self.yaw_tolerance = 8.0
        self.waypoints = generate_lawnmower_waypoints(NUM_LANES, LANE_LENGTH_M, LANE_WIDTH_M)
        self.current_waypoint_idx = 0
        self.waypoint_tolerance_m = 2.0 
        self.stuck_counter = 0
        self.stuck_threshold = 30 
        self.last_north = 0.0
        self.last_east = 0.0
        self.movement_minimum = 0.3 
        self.canister_log = CanisterLog()
        self.position_state = SharedState()
        self.receiver = DepthReceiver(DEPTH_TOPIC)
        self.planner = AvoidancePlanner(K=K, width=640, height=480, safe_distance=SAFE_DISTANCE_M, critical_distance=CRITICAL_DISTANCE_M)
        self.drone = Drone()
        self.detector = Detector(model_path=MODEL_PATH, confidence_threshold=DETECTION_CONFIDENCE, callback=self.on_detection, num_workers=1, device="cpu", save_dir="./detections", enable_display=True, display_window_name="Canister Detector")
        self.gz_node = Node()
        
        subscribed = self.gz_node.subscribe(Image, CAMERA_TOPIC, self._camera_callback)
        if subscribed:
            print(f"Camera subscribed: {CAMERA_TOPIC}")
        else:
            print(f"WARNING: Could not subscribe to camera topic: {CAMERA_TOPIC}")
            print("Detection will not work but navigation will continue.")

    def _camera_callback(self, msg: Image):
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8)
            frame = frame.reshape((msg.height, msg.width, 3))
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            self.detector.submit_image(frame_bgr, context={"timestamp": time.time(), "north": self.pose["north"], "east": self.pose["east"], "down": self.pose["down"]})
        except Exception as e:
            pass 

    def on_detection(self, detections, annotated_image, context):
        for det in detections:
            self.canister_log.add(north=context.get("north", 0.0), east=context.get("east", 0.0), down=context.get("down", 0.0), class_name=det["class_name"], confidence=det["confidence"], photo_path=context.get("saved_path", "unknown"))

    async def update_pose(self):
        if self.position_state.latest_position is None:
            return 
        self.pose["north"] = self.position_state.latest_position.north_m
        self.pose["east"] = self.position_state.latest_position.east_m
        self.pose["down"] = self.position_state.latest_position.down_m
        self.pose["yaw_deg"] = self.position_state.latest_yaw or 0.0
        self.pose["yaw"] = math.radians(self.pose["yaw_deg"])

    def distance_to_waypoint(self, wp_north, wp_east):
        dn = wp_north - self.pose["north"]
        de = wp_east - self.pose["east"]
        return math.sqrt(dn*dn + de*de)

    def _get_current_waypoint(self):
        if self.current_waypoint_idx < len(self.waypoints):
            return self.waypoints[self.current_waypoint_idx]
        return None

    def advance_waypoint(self):
        self.current_waypoint_idx += 1
        if self.current_waypoint_idx < len(self.waypoints):
            wp = self.waypoints[self.current_waypoint_idx]
            print(f"\nAdvancing to waypoint {self.current_waypoint_idx+1}/{len(self.waypoints)}: N={wp[0]:.1f} E={wp[1]:.1f}")
        else:
            print("\nAll waypoints complete! Search finished.")

    def _goal_vector_to_waypoint(self, wp_north, wp_east):
        dn = wp_north - self.pose["north"]
        de = wp_east - self.pose["east"]
        dist = math.sqrt(dn*dn + de*de) + 1e-6
        return dn/dist, de/dist

    def check_if_stuck(self):
        moved = math.sqrt((self.pose["north"] - self.last_north)**2 + (self.pose["east"] - self.last_east)**2)
        if moved < self.movement_minimum:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
            self.last_north = self.pose["north"]
            self.last_east = self.pose["east"]
        return self.stuck_counter >= self.stuck_threshold

    def _yaw_error(self, target, current):
        error = target - current
        while error > 180: error -= 360
        while error < -180: error += 360
        return error

    async def _align_yaw(self):
        current_yaw = await self.drone.get_yaw()
        err = self._yaw_error(self.target_yaw_deg, current_yaw)
        if abs(err) > self.yaw_tolerance:
            await self.drone.rotate_to_yaw(self.target_yaw_deg)

    def _compute_target_yaw(self, goal_north, goal_east):
        dn = goal_north - self.pose["north"]
        de = goal_east - self.pose["east"]
        if abs(dn) < 0.5 and abs(de) < 0.5:
            return self.target_yaw_deg 
        bearing = math.degrees(math.atan2(de, dn))
        snapped = round(bearing / 90.0) * 90.0
        return snapped

    async def run(self):
        print("\n" + "="*55)
        print(" BRAINHACK 2026 - QUALIFIER RUN STARTING")
        print("="*55)
        print("Connecting to drone...")
        await self.drone.connect()
        await asyncio.sleep(3)

        print("Starting position monitor...")
        stop_event = asyncio.Event()
        self.monitor_task = asyncio.create_task(position_monitor_task(self.drone, self.position_state, stop_event))

        print("Waiting for position data...")
        for _ in range(50):
            if self.position_state.latest_position is not None:
                break
            await asyncio.sleep(0.1)

        print("Arming and taking off...")
        await self.drone.arm_and_takeoff()
        await self.update_pose()
        
        print("Aligning to North...")
        await self.drone.rotate_to_yaw(0.0)
        self.target_yaw_deg = 0.0
        print("\nBeginning search pattern...\n")

        try:
            while self.running:
                t_start = time.monotonic()
                await self.update_pose()
                
                wp = self._get_current_waypoint()
                if wp is None:
                    print("Search pattern complete!")
                    break
                wp_north, wp_east = wp
                
                dist_to_wp = self.distance_to_waypoint(wp_north, wp_east)
                if dist_to_wp < self.waypoint_tolerance_m:
                    self.advance_waypoint()
                    wp = self._get_current_waypoint()
                    if wp is None:
                        break
                    wp_north, wp_east = wp

                self.target_yaw_deg = self._compute_target_yaw(wp_north, wp_east)
                depth_frame = self.receiver.get_frame()
                
                if depth_frame is None:
                    await self.drone.send_velocity(0, 0, 0, self.target_yaw_deg)
                    await asyncio.sleep(0.1)
                    continue

                avoid_north, avoid_east, avoid_down, info = self.planner.compute_position_setpoint_ned(depth_frame, self.pose, step_size=STEP_SIZE_M)
                c = info["clearance"]

                goal_dn, goal_de = self._goal_vector_to_waypoint(wp_north, wp_east)
                avoid_dn = avoid_north - self.pose["north"]
                avoid_de = avoid_east - self.pose["east"]
                avoid_norm = math.sqrt(avoid_dn**2 + avoid_de**2) + 1e-6
                avoid_dn /= avoid_norm
                avoid_de /= avoid_norm

                if info["blocked"]:
                    blend_dn = avoid_dn
                    blend_de = avoid_de
                else:
                    blend_dn = 0.4 * avoid_dn + 0.6 * goal_dn
                    blend_de = 0.4 * avoid_de + 0.6 * goal_de
                    norm = math.sqrt(blend_dn**2 + blend_de**2) + 1e-6
                    blend_dn /= norm
                    blend_de /= norm

                target_north = self.pose["north"] + STEP_SIZE_M * blend_dn
                target_east = self.pose["east"] + STEP_SIZE_M * blend_de
                target_down = FLIGHT_DOWN

                if self.check_if_stuck():
                    print("Stuck detected! Rotating to find new direction...")
                    self.stuck_counter = 0
                    new_yaw = self.target_yaw_deg + 90
                    if new_yaw > 180: new_yaw -= 360
                    self.target_yaw_deg = new_yaw
                    await self.drone.rotate_to_yaw(self.target_yaw_deg)
                    await asyncio.sleep(1.0)
                    continue

                if info["blocked"]:
                    await self.drone.send_velocity(0, 0, 0, self.target_yaw_deg)
                    if c["left"] > c["right"]:
                        turn = self.target_yaw_deg - 90 
                    else:
                        turn = self.target_yaw_deg + 90 
                    if turn > 180: turn -= 360
                    if turn < -180: turn += 360
                    self.target_yaw_deg = turn
                    await self.drone.rotate_to_yaw(self.target_yaw_deg)
                    await asyncio.sleep(0.5)
                else:
                    await self._align_yaw()
                    await self.drone.send_position_setpoint(north=target_north, east=target_east, down=target_down, yaw_deg=self.target_yaw_deg)

                elapsed = time.monotonic() - t_start
                sleep_t = max(0, (1.0 / LOOP_HZ) - elapsed)
                await asyncio.sleep(sleep_t)

        except asyncio.CancelledError:
            print("\nNavigation cancelled.")
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received.")
        finally:
            await self._shutdown()

    async def _shutdown(self):
        print("\nShutting down...")
        self.running = False
        try:
            self.detector.stop()
        except Exception:
            pass
        try:
            await self.drone.send_velocity(0, 0, 0, self.target_yaw_deg)
            await asyncio.sleep(1.0)
            await self.drone.land()
        except Exception as e:
            print(f"Landing error: {e}")
        self.canister_log.save_report("canister_log.txt")
        print("\nDone. Check canister_log.txt for results.")

    def stop(self):
        self.running = False

async def main():
    nav = QualifierRun()
    task = asyncio.create_task(nav.run())
    try:
        await task
    except KeyboardInterrupt:
        print("\nStopping...")
        nav.stop()
        task.cancel()
        try:
            await task
        except Exception:
            pass

if __name__ == "__main__":
    print("\nBRAINHACK 2026 - RoboVerse Qualifier")
    print("Press Ctrl+C at any time to stop safely and land.\n")
    asyncio.run(main())
