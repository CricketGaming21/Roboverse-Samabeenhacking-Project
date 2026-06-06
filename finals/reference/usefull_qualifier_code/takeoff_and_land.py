import asyncio
import time
from mavsdk import System
from mavsdk.telemetry import LandedState

async def run():
    # 1. Initialize the drone
    drone = System()
    print("🔌 Connecting to PX4 SITL...")
    # Using the correct udpin connection for your VM
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    # 2. Wait for the connection to establish
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Connected to drone!")
            break

    # 3. The Patient Health Check (ADMIN RULE: Only check Home Position)
    print("Waiting for drone to settle... (Did you set ekf_origin and bump it in Gazebo?)")
    timeout = 30
    start_time = time.time()
    is_ready = False

    while time.time() - start_time < timeout:
        async for health in drone.telemetry.health():
            # We ONLY look at the local home position, ignoring GPS entirely
            if health.is_home_position_ok:
                is_ready = True
                break
        if is_ready:
            break
        await asyncio.sleep(1)

    if not is_ready:
        print("❌ Health check timed out. Make sure you ran 'commander set_ekf_origin...' and bumped the drone.")
        return

    print("✅ System healthy! Executing Flight Plan...")

    try:
        # 4. Takeoff Sequence
        print("-- Arming Motors")
        await drone.action.arm()

        print("🚀 Taking off...")
        await drone.action.takeoff()

        # Give the drone 8 seconds to climb to its target altitude
        await asyncio.sleep(8)

        # 5. Hover
        print("⏳ Hovering for 5 seconds...")
        await asyncio.sleep(5)

        # 6. Landing Sequence
        print("🛬 Landing...")
        await drone.action.land()

        # 7. Confirm Touchdown
        async for landed in drone.telemetry.landed_state():
            if landed == LandedState.ON_GROUND:
                print("✅ Landed successfully. Mission Complete.")
                break

    except Exception as e:
        print(f"❌ Flight failed with error: {e}")

if __name__ == "__main__":
    # Run the asynchronous event loop
    asyncio.run(run()) 
    
