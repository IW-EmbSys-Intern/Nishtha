import rclpy
from rclpy.node import Node

from mavros_msgs.srv import CommandBool, SetMode, CommandTOL


class TakeoffNode(Node):

    def __init__(self):
        super().__init__('takeoff_node')

        # Create service clients
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        # Timer to run logic repeatedly
        self.timer = self.create_timer(2.0, self.run)

        self.stage = 0

        self.get_logger().info("Takeoff node started")

    def run(self):

        # STEP 1: Set mode (optional but safer)
        if self.stage == 0:
            self.get_logger().info("Setting mode to OFFBOARD/POSCTL")

            req = SetMode.Request()
            req.custom_mode = "POSCTL"

            self.mode_client.call_async(req)

            self.stage = 1


        # STEP 2: ARM drone
        elif self.stage == 1:
            self.get_logger().info("Arming drone")

            req = CommandBool.Request()
            req.value = True

            self.arm_client.call_async(req)

            self.stage = 2


        # STEP 3: TAKEOFF
        elif self.stage == 2:
            self.get_logger().info("Taking off")

            req = CommandTOL.Request()
            req.altitude = 10.0

            self.takeoff_client.call_async(req)

            self.stage = 3


def main(args=None):
    rclpy.init(args=args)
    node = TakeoffNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
