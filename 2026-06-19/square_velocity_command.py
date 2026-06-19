import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw


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

    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
    )

    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard start failed: {e}")
        return

    speed = 2.0
    side_time = 10 #10*2=20 meters

    print("Forward")
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(speed, 0.0, 0.0, 0.0)
    )
    await asyncio.sleep(side_time)

    print("Right")
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, speed, 0.0, 0.0)
    )
    await asyncio.sleep(side_time)

    print("Backward")
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(-speed, 0.0, 0.0, 0.0)
    )
    await asyncio.sleep(side_time)

    print("Left")
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, -speed, 0.0, 0.0)
    )
    await asyncio.sleep(side_time)

    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
    )

    await drone.offboard.stop()

    print("RTL")
    await drone.action.return_to_launch()
    await asyncio.sleep(10)
    await drone.action.land()


if __name__ == "__main__":
    asyncio.run(run())
