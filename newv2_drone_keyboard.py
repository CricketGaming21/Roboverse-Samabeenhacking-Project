"""
drone_keyboard.py
==================
Game-style drone controller — keys behave exactly like a video game.
Press W = fly forward, release W = stop instantly. No drift.

Uses pynput for true key press/release events + OpenCV for the HUD.

Controls:
  T           Arm + Takeoff
  W / S       Forward / Backward
  A / D       Strafe Left / Right
  Q / E       Yaw Left / Right
  SPACE       Climb
  C           Descend
  L           Land
  ESC         Emergency stop + quit

Install deps if needed:
  pip install mavsdk pynput opencv-python
"""

import asyncio
import threading
import time
import numpy as np
import cv2
from pynput import keyboard as kb
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

# ── Config ────────────────────────────────────────────────────────────────────
MAVSDK_ADDRESS   = "udp://:14540"
TAKEOFF_ALTITUDE = 2.5   # metres

SPEED_XY = 2.0    # m/s horizontal
SPEED_Z  = 1.0    # m/s vertical
YAW_RATE = 40.0   # deg/s
# ─────────────────────────────────────────────────────────────────────────────

# ── Held keys set (pynput gives true press + release) ────────────────────────
held      = set()
held_lock = threading.Lock()

# ── Shared state ──────────────────────────────────────────────────────────────
class State:
    north:   float = 0.0
    east:    float = 0.0
    alt:     float = 0.0
    yaw_deg: float = 0.0
    battery: float = 0.0
    mode:    str   = "UNKNOWN"
    armed:   bool  = False

    do_takeoff:  bool = False
    do_land:     bool = False
    do_estop:    bool = False
    offboard_on: bool = False
    running:     bool = True

st      = State()
st_lock = threading.Lock()


# ── Key normalisation ─────────────────────────────────────────────────────────
def _norm(key):
    try:
        return key.char.lower()
    except AttributeError:
        return str(key)


# ── pynput callbacks ──────────────────────────────────────────────────────────
def on_press(key):
    k = _norm(key)
    with held_lock:
        held.add(k)

    with st_lock:
        if k == 't' and not st.offboard_on:
            st.do_takeoff = True
        elif k == 'l':
            st.do_land = True
        elif k == 'Key.esc':
            st.do_estop = True


def on_release(key):
    k = _norm(key)
    with held_lock:
        held.discard(k)


# ── Velocity from held keys ───────────────────────────────────────────────────
def compute_velocity():
    with held_lock:
        h = set(held)

    vx, vy, vz, yaw = 0.0, 0.0, 0.0, 0.0

    if 'w' in h: vx += SPEED_XY
    if 's' in h: vx -= SPEED_XY
    if 'd' in h: vy += SPEED_XY
    if 'a' in h: vy -= SPEED_XY
    if 'Key.space' in h: vz -= SPEED_Z   # climb (NED: negative = up)
    if 'c' in h:         vz += SPEED_Z   # descend
    if 'e' in h: yaw += YAW_RATE
    if 'q' in h: yaw -= YAW_RATE

    return vx, vy, vz, yaw


# ── OpenCV HUD ────────────────────────────────────────────────────────────────
def make_hud():
    W, H = 520, 420
    img = np.zeros((H, W, 3), dtype=np.uint8)
    img[:] = (30, 30, 30)

    def txt(msg, x, y, scale=0.55, color=(220, 220, 220), bold=False):
        thickness = 2 if bold else 1
        cv2.putText(img, msg, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, color, thickness, cv2.LINE_AA)

    # Title
    txt("DRONE CONTROLLER", 140, 32, 0.7, (100, 220, 255), bold=True)
    cv2.line(img, (10, 42), (W - 10, 42), (80, 80, 80), 1)

    # Telemetry
    with st_lock:
        north   = st.north
        east    = st.east
        alt     = st.alt
        yaw_deg = st.yaw_deg
        battery = st.battery
        mode    = st.mode
        armed   = st.armed
        offboard= st.offboard_on

    txt("TELEMETRY", 14, 68, 0.5, (150, 150, 150))
    txt(f"North  : {north:+.2f} m",  14, 92)
    txt(f"East   : {east:+.2f} m",   14, 114)
    txt(f"Alt    : {alt:.2f} m",      14, 136)
    txt(f"Yaw    : {yaw_deg:.1f} deg",14, 158)

    # Battery bar
    bat_pct = max(0.0, min(1.0, battery / 100.0))
    bar_w   = int(220 * bat_pct)
    bar_col = (0, 200, 80) if bat_pct > 0.4 else (0, 165, 255) if bat_pct > 0.2 else (0, 50, 220)
    cv2.rectangle(img, (14, 172), (234, 188), (70, 70, 70), -1)
    cv2.rectangle(img, (14, 172), (14 + bar_w, 188), bar_col, -1)
    txt(f"Battery: {battery:.0f}%", 14, 200)

    # Status badges
    arm_col  = (0, 200, 80)   if armed   else (0, 50, 200)
    arm_lbl  = "ARMED"        if armed   else "DISARMED"
    off_col  = (0, 200, 80)   if offboard else (0, 50, 200)
    off_lbl  = "OFFBOARD ON"  if offboard else "OFFBOARD OFF"

    cv2.rectangle(img, (14, 210),  (120, 232), arm_col, -1)
    txt(arm_lbl, 18, 227, 0.45, (255, 255, 255), bold=True)

    cv2.rectangle(img, (130, 210), (280, 232), off_col, -1)
    txt(off_lbl, 134, 227, 0.45, (255, 255, 255), bold=True)

    txt(f"Mode: {mode}", 14, 252, 0.5, (180, 180, 180))

    cv2.line(img, (10, 262), (W - 10, 262), (80, 80, 80), 1)

    # Key indicators
    txt("KEYS", 14, 283, 0.5, (150, 150, 150))

    with held_lock:
        h = set(held)

    def key_badge(label, key_str, x, y):
        active = key_str in h
        col    = (0, 200, 80) if active else (70, 70, 70)
        cv2.rectangle(img, (x, y), (x + 46, y + 24), col, -1)
        txt(label, x + 6, y + 17, 0.45, (255, 255, 255), bold=active)

    key_badge("W",     'w',         200, 290)
    key_badge("S",     's',         200, 318)
    key_badge("A",     'a',         150, 318)
    key_badge("D",     'd',         250, 318)
    key_badge("Q",     'q',         150, 290)
    key_badge("E",     'e',         250, 290)
    key_badge("SPC",   'Key.space', 14,  318)
    key_badge("C",     'c',         64,  318)
    key_badge("T",     't',         320, 290)
    key_badge("L",     'l',         320, 318)
    key_badge("ESC",   'Key.esc',   380, 290)

    cv2.line(img, (10, 352), (W - 10, 352), (80, 80, 80), 1)

    # Controls legend
    txt("W/S Forward/Back  A/D Strafe  Q/E Yaw", 14, 372, 0.42, (140, 140, 140))
    txt("SPACE Climb  C Descend  T Takeoff  L Land  ESC Stop", 14, 392, 0.42, (140, 140, 140))

    vx, vy, vz, yaw = compute_velocity()
    txt(f"CMD  fwd={vx:+.1f}  rgt={vy:+.1f}  dwn={vz:+.1f}  yaw={yaw:+.1f}", 14, 412, 0.42, (100, 220, 255))

    return img


def cv_thread():
    cv2.namedWindow("Drone Controller", cv2.WINDOW_AUTOSIZE)
    while st.running:
        cv2.imshow("Drone Controller", make_hud())
        key = cv2.waitKey(30) & 0xFF
        if key == 27:   # ESC via OpenCV as backup
            with st_lock:
                st.do_estop = True
            break
    cv2.destroyAllWindows()
    with st_lock:
        st.running = False


# ── Telemetry loop ────────────────────────────────────────────────────────────
async def telemetry_loop(drone: System):
    async def pos():
        async for p in drone.telemetry.position_velocity_ned():
            with st_lock:
                st.north = p.position.north_m
                st.east  = p.position.east_m
                st.alt   = -p.position.down_m

    async def att():
        async for a in drone.telemetry.attitude_euler():
            with st_lock:
                st.yaw_deg = a.yaw_deg

    async def bat():
        async for b in drone.telemetry.battery():
            with st_lock:
                st.battery = b.remaining_percent

    async def arm():
        async for a in drone.telemetry.armed():
            with st_lock:
                st.armed = a

    async def mode():
        async for m in drone.telemetry.flight_mode():
            with st_lock:
                st.mode = str(m).replace("FlightMode.", "")

    await asyncio.gather(pos(), att(), bat(), arm(), mode())


# ── Main control loop ─────────────────────────────────────────────────────────
async def control_loop(drone: System):
    dt = 0.05   # 20 Hz

    while st.running:

        # ── Takeoff ──────────────────────────────────────────────────────────
        if st.do_takeoff:
            with st_lock:
                st.do_takeoff = False

            print("[CTL] Arming...")
            try:
                await drone.param.set_param_int("COM_ARM_WO_GPS", 1)
                await drone.action.arm()
            except Exception as e:
                print(f"[CTL] Arm failed: {e}")
                await asyncio.sleep(dt)
                continue

            print(f"[CTL] Taking off to {TAKEOFF_ALTITUDE}m...")
            await drone.action.takeoff()

            # Wait for altitude — with 10s timeout so we never get stuck
            print("[CTL] Waiting for altitude...")
            t0 = time.time()
            async for pos in drone.telemetry.position_velocity_ned():
                alt = -pos.position.down_m
                print(f"\r[CTL] Alt: {alt:.2f} / {TAKEOFF_ALTITUDE:.2f}m", end="", flush=True)
                if alt >= TAKEOFF_ALTITUDE - 0.5:
                    print("\n[CTL] Target altitude reached.")
                    break
                if time.time() - t0 > 10:
                    print("\n[CTL] Altitude timeout — starting offboard anyway.")
                    break

            # Start offboard
            print("[CTL] Starting offboard mode...")
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
            try:
                await drone.offboard.start()
                with st_lock:
                    st.offboard_on = True
                print("[CTL] Offboard active. Fly!")
            except OffboardError as e:
                print(f"[CTL] Offboard start failed: {e}")

        # ── Land ─────────────────────────────────────────────────────────────
        if st.do_land:
            with st_lock:
                st.do_land     = False
                st.offboard_on = False
            print("[CTL] Landing...")
            try:
                await drone.offboard.stop()
            except Exception:
                pass
            await drone.action.land()

        # ── Emergency stop ────────────────────────────────────────────────────
        if st.do_estop:
            print("[CTL] EMERGENCY STOP")
            try:
                await drone.offboard.stop()
            except Exception:
                pass
            try:
                await drone.action.disarm()
            except Exception:
                pass
            with st_lock:
                st.running = False
            break

        # ── Send velocity ─────────────────────────────────────────────────────
        if st.offboard_on:
            vx, vy, vz, yaw = compute_velocity()
            try:
                await drone.offboard.set_velocity_body(
                    VelocityBodyYawspeed(
                        forward_m_s    = vx,
                        right_m_s      = vy,
                        down_m_s       = vz,
                        yawspeed_deg_s = yaw,
                    )
                )
            except OffboardError as e:
                print(f"[WARN] Offboard error: {e}")

        await asyncio.sleep(dt)

    try:
        await drone.offboard.stop()
    except Exception:
        pass


# ── Entry point ───────────────────────────────────────────────────────────────
async def main():
    drone = System()
    print(f"Connecting to {MAVSDK_ADDRESS}...")
    await drone.connect(system_address=MAVSDK_ADDRESS)

    async for s in drone.core.connection_state():
        if s.is_connected:
            print("Connected!")
            break

    print("Waiting for position estimate...")
    async for h in drone.telemetry.health():
        if h.is_local_position_ok:
            print("Ready. Press T in the HUD window to takeoff.")
            break

    # Start pynput listener
    listener = kb.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    # Start HUD
    threading.Thread(target=cv_thread, daemon=True).start()

    try:
        await asyncio.gather(
            telemetry_loop(drone),
            control_loop(drone),
        )
    except asyncio.CancelledError:
        pass

    listener.stop()
    print("Done.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nInterrupted.")
