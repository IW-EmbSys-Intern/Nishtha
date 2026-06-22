import asyncio
import math
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, VelocityNedYaw, OffboardError
from mavsdk.telemetry import LandedState


TAKEOFF_ALT = 20.0


# ---------------- CONNECTION ----------------
async def wait_connected(drone):
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected")
            return


# ---------------- GET POSITION ----------------
async def get_pos(drone):
    async for pos in drone.telemetry.position_velocity_ned():
        return pos.position.north_m, pos.position.east_m, pos.position.down_m


# ---------------- SAFE OFFBOARD START ----------------
async def start_offboard(drone, n, e, d):
    # PX4 requires streaming setpoints before starting offboard
    for _ in range(10):
        await drone.offboard.set_position_ned(
            PositionNedYaw(n, e, d, 0.0)
        )
        await asyncio.sleep(0.05)

    try:
        await drone.offboard.start()
        print("Offboard started")
    except OffboardError as e:
        print(f"Offboard start failed: {e._result.result}")
        await drone.action.disarm()
        raise


# ---------------- SMOOTH VELOCITY WAYPOINT ----------------
async def goto_waypoint(drone, target, max_speed):
    while True:
        pos = await get_pos(drone)

        cn, ce, cd = pos
        tn, te, td = target

        dn, de, dd = tn - cn, te - ce, td - cd
        dist = math.sqrt(dn**2 + de**2 + dd**2)

        # reached waypoint
        if dist < 0.8:
            await drone.offboard.set_position_ned(
                PositionNedYaw(tn, te, td, 0.0)
            )
            return

        # slow down near target (smooth deceleration)
        slow_radius = 15.0
        speed = max(0.5, min(max_speed, max_speed * dist / slow_radius))

        vn = speed * dn / dist
        ve = speed * de / dist
        vd = speed * dd / dist

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(vn, ve, vd, 0.0)
        )

        await asyncio.sleep(0.1)


# ---------------- MAIN ----------------
async def main():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    await wait_connected(drone)

    # ARM
    await drone.action.arm()

    # TAKEOFF MODE FIRST
    print("Taking off...")
    await drone.action.takeoff()
    await asyncio.sleep(5)

    # home position
    n0, e0, d0 = await get_pos(drone)

    target_alt = d0 - TAKEOFF_ALT

    # define waypoints
    wp1 = (n0 + 50, e0, target_alt)
    wp2 = (n0 + 50, e0 + 50, target_alt)
    wp3 = (n0, e0 + 60, target_alt)

    # WAIT STABLE TAKEOFF
    await asyncio.sleep(2)

    # START OFFBOARD
    await start_offboard(drone, n0, e0, target_alt)

    # ---------------- FLIGHT SEQUENCE ----------------
    print("WP1 @ 5 m/s")
    await goto_waypoint(drone, wp1, 5.0)
    await asyncio.sleep(2)

    print("WP2 @ 10 m/s")
    await goto_waypoint(drone, wp2, 10.0)
    await asyncio.sleep(2)

    print("WP3 @ 15 m/s")
    await goto_waypoint(drone, wp3, 15.0)
    await asyncio.sleep(2)

    # ---------------- LAND ----------------
    print("Landing...")
    await drone.offboard.stop()

    await drone.action.land()

    async for state in drone.telemetry.landed_state():
        if state == LandedState.ON_GROUND:
            break

    await drone.action.disarm()
    print("Done")


if __name__ == "__main__":
    asyncio.run(main())
