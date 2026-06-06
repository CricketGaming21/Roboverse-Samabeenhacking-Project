import asyncio
from mavsdk import System

async def run():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected")
            break

    print("Checking health values...")
    async for health in drone.telemetry.health():
        print(f"global_pos_ok={health.is_global_position_ok} | home_pos_ok={health.is_home_position_ok} | local_pos_ok={health.is_local_position_ok} | accel_ok={health.is_accelerometer_calibration_ok} | heading={health.is_armable}")
        await asyncio.sleep(1)

asyncio.run(run())