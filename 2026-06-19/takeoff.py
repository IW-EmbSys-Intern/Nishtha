import asyncio
from mavsdk import System

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Connecting...")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to PX4")
            break

    print("Waiting for GPS...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("GPS ready")
            break

    print("Arming...")
    await drone.action.arm()

    print("Taking off...")
    await drone.action.takeoff()

    await asyncio.sleep(10)

    print("Done")

asyncio.run(run())
