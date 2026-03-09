import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Safety Configuration

MAX_LINEAR = 0.22       # m/s (Turtlebot3 Burger limit)
MAX_ANGULAR = 2.5       # rad/s
CMD_TIMEOUT = 1.0       # seconds before stopping robot


class ActuationNode(Node):

    def __init__(self):
        super().__init__('actuation_node')

        # Subscriber receiving safe velocity commands
        self.subscription = self.create_subscription(
            Twist,
            '/safe_cmd',
            self.safe_callback,
            10
        )

        # Publisher sending bounded commands to robot
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.last_cmd_time = self.get_clock().now()
        self.robot_stopped = False
        self.cmd_counter = 0
        self.timeout_timer = self.create_timer(0.1, self.check_timeout)
        self.rate_timer = self.create_timer(1.0, self.report_rate)

        self.get_logger().info("Actuation node started with safety bounds and timeout protection.")

    # Utility: Clamp with detection

    def clamp_with_flag(self, value, max_value):
        if value > max_value:
            return max_value, True
        elif value < -max_value:
            return -max_value, True
        return value, False

    # Main callback

    def safe_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()
        self.robot_stopped = False
        self.cmd_counter += 1

        bounded_msg = Twist()

        # Clamping linear velocity
        bounded_linear, lin_saturated = self.clamp_with_flag(msg.linear.x, MAX_LINEAR)

        # Clamping angular velocity
        bounded_angular, ang_saturated = self.clamp_with_flag(msg.angular.z, MAX_ANGULAR)

        bounded_msg.linear.x = bounded_linear
        bounded_msg.angular.z = bounded_angular

        if lin_saturated or ang_saturated:
            self.get_logger().warn(
                "Actuation safety clamp triggered "
                f"(requested lin={msg.linear.x:.3f}, ang={msg.angular.z:.3f})"
            )

        self.get_logger().debug(
            f"Cmd received | lin={msg.linear.x:.3f}, ang={msg.angular.z:.3f} "
            f"→ bounded | lin={bounded_linear:.3f}, ang={bounded_angular:.3f}"
        )

        self.publisher.publish(bounded_msg)

    # Timeout safety

    def check_timeout(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9

        if elapsed > CMD_TIMEOUT and not self.robot_stopped:
            stop_msg = Twist()
            self.publisher.publish(stop_msg)

            self.get_logger().warn(
                f"Command timeout ({elapsed:.2f}s). Stopping robot."
            )

            self.robot_stopped = True

    # Command rate monitoring

    def report_rate(self):
        self.get_logger().info(f"Command rate: {self.cmd_counter} Hz")
        self.cmd_counter = 0


def main(args=None):
    rclpy.init(args=args)

    node = ActuationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()