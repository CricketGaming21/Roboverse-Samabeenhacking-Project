#!/usr/bin/env python3
"""
telemetry_logger.py
Connects to PX4 SITL and streams all telemetry to a rolling log file.
Claude Code reads this file for live debug context.
Run this BEFORE your main drone script, keep it running throughout.
"""

import asyncio
import json
import os
from datetime import datetime
from mavsdk import System

LOG_DIR = os.path.expanduser("~/Desktop/codes/claude_debug/logs")
TELEM_LOG = os.path.join(LOG_DIR, "telemetry.log")
EVENT_LOG  = os.path.join(LOG_DIR, "events.log")

os.makedirs(LOG_DIR, exist_ok=True)

def ts():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]

def write(path, msg):
    line = f"[{ts()}] {msg}\n"
    with open(path, "a") as f:
        f.write(line)
    print(line, end="")  # also mirrors to terminal

async def stream_position(drone):
    async for pos in drone.telemetry.position():
        write(TELEM_LOG, f"POSITION  lat={pos.latitude_deg:.6f} lon={pos.longitude_deg:.6f} alt={pos.relative_altitude_m:.2f}m")

async def stream_velocity(drone):
    async for vel in drone.telemetry.velocity_ned():
        write(TELEM_LOG, f"VELOCITY  N={vel.north_m_s:.2f} E={vel.east_m_s:.2f} D={vel.down_m_s:.2f} m/s")

async def stream_attitude(drone):
    async for att in drone.telemetry.attitude_euler():
        write(TELEM_LOG, f"ATTITUDE  roll={att.roll_deg:.1f} pitch={att.pitch_deg:.1f} yaw={att.yaw_deg:.1f} deg")

async def stream_battery(drone):
    async for bat in drone.telemetry.battery():
        write(TELEM_LOG, f"BATTERY   {bat.remaining_percent*100:.0f}%  {bat.voltage_v:.2f}V")

async def stream_flight_mode(drone):
    async for mode in drone.telemetry.flight_mode():
        write(EVENT_LOG, f"FLIGHTMODE  {mode}")

async def stream_health(drone):
    async for health in drone.telemetry.health():
        status = {
            "gps_ok": health.is_global_position_ok,
            "home_ok": health.is_home_position_ok,
            "accel_ok": health.is_accelerometer_calibration_ok,
            "gyro_ok": health.is_gyrometer_calibration_ok,
            "mag_ok": health.is_magnetometer_calibration_ok,
        }
        write(EVENT_LOG, f"HEALTH    {json.dumps(status)}")

async def stream_landed_state(drone):
    async for state in drone.telemetry.landed_state():
        write(EVENT_LOG, f"LANDED_STATE  {state}")

async def stream_armed(drone):
    async for armed in drone.telemetry.armed():
        write(EVENT_LOG, f"ARMED  {armed}")

async def stream_gps(drone):
    async for gps in drone.telemetry.gps_info():
        write(TELEM_LOG, f"GPS  fix={gps.fix_type}  sats={gps.num_satellites}")

async def run():
    # Clear old logs at start of each session
    for f in [TELEM_LOG, EVENT_LOG]:
        with open(f, "w") as log:
            log.write(f"=== Session started {datetime.now().isoformat()} ===\n")

    drone = System()
    write(EVENT_LOG, "Connecting to PX4 SITL on udp://:14540 ...")
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            write(EVENT_LOG, "CONNECTED to PX4 SITL")
            break

    write(EVENT_LOG, "Telemetry logger running — streaming all data to logs/")

    # Run all streams concurrently
    await asyncio.gather(
        stream_position(drone),
        stream_velocity(drone),
        stream_attitude(drone),
        stream_battery(drone),
        stream_flight_mode(drone),
        stream_health(drone),
        stream_landed_state(drone),
        stream_armed(drone),
        stream_gps(drone),
    )

if __name__ == "__main__":
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        write(EVENT_LOG, "Telemetry logger stopped by user.")


