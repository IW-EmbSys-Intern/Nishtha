import asyncio
import math
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw

TAKEOFF_ALT = -10.0   # 10m altitude (NED)
FORWARD_SPEED = 14.0    # m/s
CIRCLE_RADIUS = 20.0   # meters
CIRCLE_SPEED = 2.0     # m/s


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # Wait for connection
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    print("Connected")

    # Arm
    await drone.action.arm()

    # Start offboard with zero velocity
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
    )
    await drone.offboard.start()

    print("Taking off...")
    for _ in range(50):
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, -1.5, 0.0)
        )
        await asyncio.sleep(0.1)

    # Hold altitude
    for _ in range(20):
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
        )
        await asyncio.sleep(0.1)

    print("Moving forward 40m...")

    duration = 40 / FORWARD_SPEED  # time = distance / speed

    steps = int(duration * 10)

    for _ in range(steps):
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(FORWARD_SPEED, 0.0, 0.0, 0.0)
        )
        await asyncio.sleep(0.1)

    print("Flying circle...")

    omega = CIRCLE_SPEED / CIRCLE_RADIUS

    steps = int(2 * math.pi / (omega * 0.1))  # full circle

    for i in range(steps):
        angle = omega * i * 0.1

        vx = -CIRCLE_SPEED * math.sin(angle)
        vy = CIRCLE_SPEED * math.cos(angle)

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(vx, vy, 0.0, 0.0)
        )
        await asyncio.sleep(0.1)

    print("RTL initiated")

    await drone.offboard.stop()
    await drone.action.return_to_launch()

    # Optional safety landing (PX4 handles it in RTL)
    await asyncio.sleep(5)

    print("Mission complete")


if __name__ == "__main__":
    asyncio.run(run())
