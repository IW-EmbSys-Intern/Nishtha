import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

async def run():

    drone = System()

    # Connect to PX4 SITL
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to PX4!")
            break

    # Wait for global position
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("Global position OK")
            break

    # Arm
    print("Arming...")
    await drone.action.arm()
    # Takeoff
    print("Taking off...")
    await drone.action.takeoff()
    await asyncio.sleep(10)

    # Start offboard (needed for movement control)
    print("Starting offboard mode...")

    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0))

    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard failed: {e}")
        return

    # STEP 1: Move forward 40m (North direction)
    print("Moving forward 40m...")
    await drone.offboard.set_position_ned(PositionNedYaw(40.0, 0.0, -5.0, 0.0))
    await asyncio.sleep(10)

    # STEP 2: Square 20x20m

    print("Flying square...")

    points = [
        (40, 0),
        (40, 20),
        (20, 20),
        (20, 0),
        (40, 0)
    ]

    for x, y in points:
        await drone.offboard.set_position_ned(PositionNedYaw(x, y, -5.0, 0.0))
        await asyncio.sleep(5)

    # Stop offboard
    await drone.offboard.stop()

    # RTL
    print("Returning to launch...")
    await drone.action.return_to_launch()

    await asyncio.sleep(10)

    # Land
    print("Landing...")
    await drone.action.land()

    print("Mission complete!")


if __name__ == "__main__":
    asyncio.run(run())
