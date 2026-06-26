import rclpy
from rclpy.node import Node
import time
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import OverrideRCIn

class PlaneMavrosTakeoff(Node):
    def __init__(self):
        super().__init__('plane_mavros_takeoff_node')
        self.get_logger().info("=====================================")
        self.get_logger().info("ArduPlane MAVROS Direct Mode Takeoff")
        self.get_logger().info("=====================================")

        # 1. Initialize MAVROS Service Clients & Publishers
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

        # 2. Wait for MAVROS endpoints
        self.get_logger().info("Connecting to MAVROS service endpoints...")
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /mavros/set_mode...')
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /mavros/cmd/arming...')

        # 3. Execute the direct mode transition sequence
        self.execute_direct_takeoff()

    def change_flight_mode(self, mode_name):
        """Changes flight mode via MAVROS."""
        req = SetMode.Request()
        req.custom_mode = mode_name
        self.get_logger().info(f"Requesting mode transition to: {mode_name}")
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None and future.result().mode_sent:
            self.get_logger().info(f"Successfully entered {mode_name} mode.")
            return True
        else:
            self.get_logger().error(f"Autopilot REJECTED {mode_name} mode!")
            return False

    def arm_vehicle(self, arm_state: bool):
        """Arms or disarms the aircraft."""
        req = CommandBool.Request()
        req.value = arm_state
        action = "ARM" if arm_state else "DISARM"
        self.get_logger().info(f"Sending standard {action} request...")
        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None and future.result().success:
            self.get_logger().info(f"Autopilot successfully {action}ED.")
            return True
        else:
            self.get_logger().error(f"Autopilot REJECTED {action} request!")
            return False

    def execute_direct_takeoff(self):
        """Direct Mode-Switching Sequence."""
        
        # Step 1: Force MANUAL mode to establish a safe ground state
        if not self.change_flight_mode("MANUAL"):
            return
        time.sleep(2.0)
        
        # Step 2: Arm the motors on the ground
        if not self.arm_vehicle(True):
            self.get_logger().error("Takeoff sequence blocked: Check your global EKF/GPS lock!")
            return
            
        # Give the autopilot safety registers time to register the armed ground state
        self.get_logger().info("Armed. Holding 3 seconds for state stabilization...")
        
        # Clear steering stick locks by outputting a neutral virtual RC pulse
        rc_msg = OverrideRCIn()
        rc_msg.channels = [1500, 1500, 1000, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        
        for _ in range(30):
            self.rc_pub.publish(rc_msg)
            time.sleep(0.1)
        
        # Step 3: Switch directly into TAKEOFF mode as requested
        if not self.change_flight_mode("TAKEOFF"):
            return

        self.get_logger().info("=====================================================")
        self.get_logger().info("SUCCESS: ArduPlane running native TAKEOFF mode!")
        self.get_logger().info("=====================================================")

def main(args=None):
    rclpy.init(args=args)
    node = PlaneMavrosTakeoff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
