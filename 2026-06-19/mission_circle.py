import asyncio
import math
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected!")
            break

    await drone.action.arm()

    await drone.action.takeoff()
    await asyncio.sleep(8)

    print("Moving forward 40m...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(40.0, 0.0, -10.0, 0.0)
    )

    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard start failed: {e}")
        return

    await asyncio.sleep(10)

    print("Starting circle...")

    radius = 15.0
    altitude = -10.0
    center_north = 40.0
    center_east = 0.0

    steps = 60
    for i in range(steps):
        angle = 2 * math.pi * i / steps

        north = center_north + radius * math.cos(angle)
        east = center_east + radius * math.sin(angle)

        await drone.offboard.set_position_ned(
            PositionNedYaw(north, east, altitude, 0.0)
        )

        await asyncio.sleep(0.2)

    print("Returning to launch...")
    await drone.offboard.stop()

    await drone.action.return_to_launch()

    await asyncio.sleep(15)
    await drone.action.land()

    print("Mission complete")


if __name__ == "__main__":
    asyncio.run(run())
