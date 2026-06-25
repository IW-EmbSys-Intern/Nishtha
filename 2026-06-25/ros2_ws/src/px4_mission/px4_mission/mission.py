#!/usr/bin/env python3
import asyncio
import math
import rclpy
from rclpy.node import Node
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, VelocityNedYaw, OffboardError
from mavsdk.telemetry import LandedState

TAKEOFF_ALT = 20.0


class MavsdkRos2MissionNode(Node):

    def __init__(self):
        super().__init__('mavsdk_ros2_mission_node')
        self.get_logger().info("MAVSDK ROS 2 Wrapper Node initialized. Spawning mission worker...")

        # Initialize the native MAVSDK System object
        self.drone = System()

        # Spin off the main mission async loop cleanly inside the ROS 2 executor thread space
        asyncio.ensure_future(self.run_mission_sequence())

    # ---------------- TELEMETRY & CONNECTION UTILITIES ----------------
    async def wait_connected(self):
        """Asynchronously monitor connection updates until system locks handshake link."""
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("MAVLink link securely established with vehicle simulator.")
                return

    async def get_pos(self):
        """Fetch a single snapshot of the current local vehicle NED positions."""
        async for pos in self.drone.telemetry.position_velocity_ned():
            return pos.position.north_m, pos.position.east_m, pos.position.down_m

    # ---------------- FLIGHT COMMAND SEQUENCES ----------------
    async def start_offboard(self, n, e, d):
        """Initialize and change tracking pipelines directly into Offboard mode control paths."""
        # Pre-stream heartbeats explicitly mandated by PX4 safety state validation blocks
        for _ in range(10):
            await self.drone.offboard.set_position_ned(PositionNedYaw(n, e, d, 0.0))
            await asyncio.sleep(0.05)

        try:
            await self.drone.offboard.start()
            self.get_logger().info("System successfully initialized and tracking under OFFBOARD control mode.")
        except OffboardError as err:
            self.get_logger().error(f"OFFBOARD transition request rejected by flight controller: {err._result.result}")
            await self.drone.action.disarm()
            raise

    async def goto_waypoint(self, target, max_speed):
        """Dynamically tracks spatial waypoints applying a clean profile deceleration loop."""
        while True:
            # Refresh coordinates
            pos = await self.get_pos()
            cn, ce, cd = pos
            tn, te, td = target

            # Compute current spatial error parameters
            dn, de, dd = tn - cn, te - ce, td - cd
            dist = math.sqrt(dn**2 + de**2 + dd**2)

            # Check target error deadband threshold matrix constraints
            if dist < 0.8:
                await self.drone.offboard.set_position_ned(PositionNedYaw(tn, te, td, 0.0))
                return

            # Apply deceleration profile tracking calculations
            slow_radius = 15.0
            speed = max(0.5, min(max_speed, max_speed * dist / slow_radius))

            # Transform raw tracking speeds cleanly into scalar velocity structures
            vn = speed * dn / dist
            ve = speed * de / dist
            vd = speed * dd / dist

            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, vd, 0.0))
            await asyncio.sleep(0.1)

    # ---------------- MASTER MISSION FLOW RUNNER ----------------
    async def run_mission_sequence(self):
        """Asynchronous worker executing sequentially step by step."""
        self.get_logger().info("Connecting to simulation network environment target link...")
        await self.drone.connect(system_address="udpin://0.0.0.0:14540")

        # Establish base link
        await self.wait_connected()

        # Secure main system engine locks
        self.get_logger().info("Sending vehicle engine master ARM instruction payload...")
        await self.drone.action.arm()

        # Transition directly to automated local Takeoff sequence profile track rules
        self.get_logger().info("Sending automated TAKEOFF flight configuration mode profile updates...")
        await self.drone.action.takeoff()
        await asyncio.sleep(5)

        # Map absolute origin coordinates dynamically to assign tracking points accurately
        n0, e0, d0 = await self.get_pos()
        target_alt = d0 - TAKEOFF_ALT

        # Map dynamic target coordinates seamlessly derived relative to home origin spatial data
        wp1 = (n0 + 50.0, e0, target_alt)
        wp2 = (n0 + 50.0, e0 + 50.0, target_alt)
        wp3 = (n0, e0 + 60.0, target_alt)

        # Allow stabilization period buffer window gaps
        await asyncio.sleep(2)

        # Activate offboard vector control engines
        await self.start_offboard(n0, e0, target_alt)

        # Execute Waypoint Track Run Sequences
        self.get_logger().info("Executing Waypoint Path Run: Target Location WP1 -> Speed Limit 5.0m/s")
        await self.goto_waypoint(wp1, 5.0)
        await asyncio.sleep(2)

        self.get_logger().info("Executing Waypoint Path Run: Target Location WP2 -> Speed Limit 10.0m/s")
        await self.goto_waypoint(wp2, 10.0)
        await asyncio.sleep(2)

        self.get_logger().info("Executing Waypoint Path Run: Target Location WP3 -> Speed Limit 15.0m/s")
        await self.goto_waypoint(wp3, 15.0)
        await asyncio.sleep(2)

        # Initiate Landing profiles cleanly disarming internal motors safely upon contact
        self.get_logger().info("Route profile steps finalized. Shutting down OFFBOARD loop controls...")
        await self.drone.offboard.stop()

        self.get_logger().info("Sending automated vehicle autonomous LAND command configuration track...")
        await self.drone.action.land()

        # FIXED SYNTAX BUG: Using correct 'async for state in ...' iterator
        async for state in self.drone.telemetry.landed_state():
            if state == LandedState.ON_GROUND:
                self.get_logger().info("Ground contact sensor touchdown switches verified.")
                break

        await self.drone.action.disarm()
        self.get_logger().info("Mission completely finalized. Standby engine modes active.")


def main(args=None):
    rclpy.init(args=args)
    node = MavsdkRos2MissionNode()
    
    # Custom interleaved execution runner mapping asyncio and rclpy inside a unified thread layer
    try:
        loop = asyncio.get_event_loop()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.001)
            # Use loop run_until_complete to advance the asyncio event loop cycles
            loop.run_until_complete(asyncio.sleep(0.001))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
