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
  pip install mavsdk pynput
"""
 
import asyncio
import threading
import numpy as np
import cv2
from pynput import keyboard as kb
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
 
# ── Config ────────────────────────────────────────────────────────────────────
MAVSDK_ADDRESS   = "udpin://0.0.0.0:14540"
TAKEOFF_ALTITUDE = 2.5   # metres
 
SPEED_XY = 2.0    # m/s horizontal
SPEED_Z  = 1.0    # m/s vertical
YAW_RATE = 40.0   # deg/s
# ─────────────────────────────────────────────────────────────────────────────
 
# ── Held keys set (pynput gives true press + release) ─────────────────────────
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
 
    do_takeoff: bool = False
    do_land:    bool = False
    do_estop:   bool = False
    offboard_on: bool = False
    running:    bool = True
 
st = State()
st_lock = threading.Lock()
 
 
# ── Key normalisation ─────────────────────────────────────────────────────────
def _norm(key):
    """Return a comparable string for any pynput key."""
    try:
        return key.char.lower()          # regular keys: 'w', 'a', etc.
    except AttributeError:
        return str(key)                  # special keys: 'Key.space', etc.
 
 
# ── pynput callbacks (fire on actual press / release) ─────────────────────────
def on_press(key):
    k = _norm(key)
    with held_lock:
        held.add(k)
 
    # Single-shot actions
    if k == 't':
        with st_lock: st.do_takeoff = True
    elif k == 'l':
        with st_lock: st.do_land = True
    elif k == 'Key.esc':
        with st_lock: st.do_estop = True
        return False   # stop listener
 
 
def on_release(key):
    k = _norm(key)
    with held_lock:
        held.discard(k)
 
 
# ── Compute velocity from currently held keys ─────────────────────────────────
def compute_velocity():
    with held_lock:
        keys = set(held)
 
    vx = vy = vz = yaw = 0.0
    if 'w' in keys:         vx += SPEED_XY
    if 's' in keys:         vx -= SPEED_XY
    if 'd' in keys:         vy += SPEED_XY
    if 'a' in keys:         vy -= SPEED_XY
    if 'Key.space' in keys: vz -= SPEED_Z    # climb (NED: neg = up)
    if 'c' in keys:         vz += SPEED_Z    # descend
    if 'q' in keys:         yaw -= YAW_RATE
    if 'e' in keys:         yaw += YAW_RATE
    return vx, vy, vz, yaw
 
 
# ── HUD ───────────────────────────────────────────────────────────────────────
def make_hud():
    H, W = 460, 520
    img = np.zeros((H, W, 3), dtype=np.uint8)
 
    def box(x1, y1, x2, y2, col=(30, 30, 30)):
        cv2.rectangle(img, (x1, y1), (x2, y2), col, -1)
 
    def txt(text, x, y, scale=0.55, col=(200, 200, 200), bold=False):
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, col, 2 if bold else 1, cv2.LINE_AA)
 
    # ── Title ──
    box(0, 0, W, 40, (20, 20, 60))
    txt("DRONE CONTROLLER", 10, 28, 0.72, (255, 255, 255), bold=True)
    armed_col = (0, 200, 0) if st.armed else (0, 0, 180)
    box(W-155, 7, W-5, 33, armed_col)
    txt(f"{'ARMED' if st.armed else 'DISARMED'}  {st.mode}", W-150, 26, 0.45, (255,255,255))
 
    # ── Telemetry ──
    box(6, 48, W-6, 178, (22, 22, 22))
    txt("TELEMETRY", 14, 68, 0.5, (80, 180, 255), bold=True)
    txt(f"North  : {st.north:+8.2f} m",   14,  92)
    txt(f"East   : {st.east:+8.2f} m",    14, 114)
    txt(f"Alt    : {st.alt:+8.2f} m",     14, 136)
    txt(f"Yaw    : {st.yaw_deg:+7.1f} deg", 14, 158)
 
    # Battery bar
    batt = max(0.0, min(1.0, st.battery))
    bc = (0,200,0) if batt > 0.4 else (0,165,255) if batt > 0.2 else (0,0,255)
    box(W//2+6, 58, W-14, 80, (55,55,55))
    box(W//2+6, 58, W//2+6+int((W//2-20)*batt), 80, bc)
    txt(f"Batt: {batt*100:.0f}%", W//2+10, 76, 0.45, (255,255,255))
 
    # ── Live velocity ──
    vx, vy, vz, yaw = compute_velocity()
    moving = any([vx, vy, vz, yaw])
    box(6, 186, W-6, 276, (22, 22, 22))
    txt("VELOCITY  (release key = 0 instantly)", 14, 206, 0.45,
        (80, 255, 160) if moving else (80, 180, 255), bold=True)
 
    def vel_bar(label, val, max_val, x, y, bar_w=180):
        txt(f"{label}: {val:+.1f}", x, y)
        filled = int(bar_w * abs(val) / max_val)
        col = (0, 200, 80) if val >= 0 else (0, 100, 255)
        box(x+110, y-12, x+110+bar_w, y+2, (50,50,50))
        if filled:
            box(x+110, y-12, x+110+filled, y+2, col)
 
    vel_bar("Fwd  ", vx,  SPEED_XY,  14, 228)
    vel_bar("Right", vy,  SPEED_XY,  14, 250)
    vel_bar("Down ", vz,  SPEED_Z,   14, 272)
 
    # ── Key indicators ──
    box(6, 284, W-6, 400, (18, 18, 18))
    txt("KEYS", 14, 304, 0.5, (200, 200, 80), bold=True)
 
    def key_badge(label, held_key, x, y, w=52, h=28):
        with held_lock:
            active = held_key in held
        bg  = (0, 160, 60) if active else (50, 50, 50)
        col = (255, 255, 255) if active else (160, 160, 160)
        box(x, y, x+w, y+h, bg)
        cv2.rectangle(img, (x,y), (x+w, y+h), (80,80,80), 1)
        tx = x + (w - len(label)*8) // 2
        txt(label, tx, y+19, 0.45, col)
 
    # WASD cluster
    key_badge("W",     'w',         210, 318)
    key_badge("A",     'a',         154, 348)
    key_badge("S",     's',         210, 348)
    key_badge("D",     'd',         266, 348)
    key_badge("Q",     'q',          98, 318)
    key_badge("E",     'e',         322, 318)
    key_badge("SPC",   'Key.space',  14, 318, w=70)
    key_badge("C",     'c',          14, 348)
    key_badge("T",     't',         390, 318)
    key_badge("L",     'l',         390, 348)
 
    # Label the clusters
    txt("Yaw", 98, 314, 0.35, (120,120,120))
    txt("Move", 196, 314, 0.35, (120,120,120))
    txt("Up/Dn", 14, 314, 0.35, (120,120,120))
    txt("Cmd", 390, 314, 0.35, (120,120,120))
 
    # ── Status bar ──
    if st.offboard_on:
        box(6, 408, W-6, H-6, (0, 60, 0))
        txt("OFFBOARD ACTIVE — flying", W//2-90, H-16, 0.5, (0,255,100))
    else:
        box(6, 408, W-6, H-6, (50, 0, 0))
        txt("STANDBY — press T to arm & takeoff", 14, H-16, 0.45, (180,80,80))
 
    return img
 
 
# ── OpenCV display thread ─────────────────────────────────────────────────────
def cv_thread():
    cv2.namedWindow("Drone Controller", cv2.WINDOW_AUTOSIZE)
    while st.running:
        cv2.imshow("Drone Controller", make_hud())
        cv2.waitKey(30)   # just refresh — keys handled by pynput
    cv2.destroyAllWindows()
 
 
# ── Telemetry ─────────────────────────────────────────────────────────────────
async def telemetry_loop(drone: System):
    async def pos():
        async for p in drone.telemetry.position_velocity_ned():
            st.north = p.position.north_m
            st.east  = p.position.east_m
            st.alt   = -p.position.down_m
    async def att():
        async for a in drone.telemetry.attitude_euler():
            st.yaw_deg = a.yaw_deg
    async def bat():
        async for b in drone.telemetry.battery():
            st.battery = b.remaining_percent
    async def arm():
        async for a in drone.telemetry.armed():
            st.armed = a
    async def mode():
        async for m in drone.telemetry.flight_mode():
            st.mode = str(m).replace("FlightMode.", "")
 
    await asyncio.gather(pos(), att(), bat(), arm(), mode())
 
 
# ── Control loop ──────────────────────────────────────────────────────────────
async def control_loop(drone: System):
    dt = 0.05   # 20 Hz
 
    while st.running:
        if st.do_takeoff:
            st.do_takeoff = False
            print("[CTL] Arming...")
            await drone.action.arm()
            print(f"[CTL] Taking off to {TAKEOFF_ALTITUDE}m...")
            await drone.action.takeoff()
            async for p in drone.telemetry.position_velocity_ned():
                if -p.position.down_m >= TAKEOFF_ALTITUDE - 0.3:
                    break
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,0))
            await drone.offboard.start()
            st.offboard_on = True
            print("[CTL] Offboard active!")
 
        if st.do_land:
            st.do_land     = False
            st.offboard_on = False
            print("[CTL] Landing...")
            try: await drone.offboard.stop()
            except: pass
            await drone.action.land()
 
        if st.do_estop:
            print("[CTL] EMERGENCY STOP")
            try: await drone.offboard.stop()
            except: pass
            try: await drone.action.disarm()
            except: pass
            st.running = False
            break
 
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
                print(f"[WARN] {e}")
 
        await asyncio.sleep(dt)
 
    try: await drone.offboard.stop()
    except: pass
 
 
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
 
    # Start pynput listener (global — works even without window focus)
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