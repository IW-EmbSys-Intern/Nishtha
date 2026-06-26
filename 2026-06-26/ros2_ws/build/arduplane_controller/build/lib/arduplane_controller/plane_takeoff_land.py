import rclpy
from rclpy.node import Node
import time
from pymavlink import mavutil

class PlanePymavlinkTakeoff(Node):
    def __init__(self):
        super().__init__('plane_pymavlink_takeoff_node')
        self.get_logger().info("=====================================")
        self.get_logger().info("ArduPlane Native Takeoff Mode Script")
        self.get_logger().info("=====================================")

        # 1. Establish direct MAVLink connection
        # Update port '14550' to match your SITL/hardware available endpoint
        connection_string = 'udp:127.0.0.1:14550'
        self.get_logger().info(f"Connecting to Autopilot on {connection_string}...")
        
        self.master = mavutil.mavlink_connection(connection_string)
        self.master.wait_heartbeat()
        self.get_logger().info("Heartbeat detected! System online.")

        # 2. Execute the required ArduPlane mode sequence
        self.execute_takeoff_sequence()

    def send_raw_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        """Helper to stream COMMAND_LONG MAVLink packets."""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            command,
            0, # Confirmation flag
            param1, param2, param3, param4, param5, param6, param7
        )

    def execute_takeoff_sequence(self):
        # Step 1: Force MANUAL Mode (ID: 0) to prepare ground status safely
        self.get_logger().info("Forcing MANUAL mode (Mode ID: 0)...")
        self.master.set_mode(0) 
        time.sleep(2.0)

        # Step 2: Request Motor Arming
        self.get_logger().info("Sending direct Arm command...")
        self.send_raw_command(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            param1=1.0 # 1.0 = ARM
        )
        
        # Give ArduPlane internal safety states time to settle as 'Armed on Ground'
        self.get_logger().info("Waiting 3 seconds for arming to stabilize...")
        time.sleep(3.0)

        # Step 3: Switch Directly into TAKEOFF Mode (ID: 13)
        # Switching into Mode 13 tells ArduPlane to apply throttle and climb autonomously.
        self.get_logger().info("Switching flight mode directly to TAKEOFF (Mode ID: 13)...")
        self.master.set_mode(13) 

        self.get_logger().info("=====================================================")
        self.get_logger().info("SUCCESS: Mode changed to TAKEOFF via Pymavlink!")
        self.get_logger().info("=====================================================")

def main(args=None):
    rclpy.init(args=args)
    node = PlanePymavlinkTakeoff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
