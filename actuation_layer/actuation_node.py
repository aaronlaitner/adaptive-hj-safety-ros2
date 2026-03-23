import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import csv
import os
import math


# Safety Configuration
MAX_LINEAR = 0.22       # m/s (Turtlebot3 Burger limit)
MAX_ANGULAR = 2.5       # rad/s
CMD_TIMEOUT = 1.0       # seconds before stopping robot


class ActuationNode(Node):

    def __init__(self):
        super().__init__('actuation_node')

        self.subscription = self.create_subscription(
            Twist,
            '/safe_cmd',
            self.safe_callback,
            10
        )

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_cmd_time = self.get_clock().now()
        self.robot_stopped = False
        self.cmd_counter = 0

        self.timeout_timer = self.create_timer(0.1, self.check_timeout)
        self.rate_timer = self.create_timer(1.0, self.report_rate)

        # Logging setup
        log_dir = "/workspaces/ros2_ws/src/adaptive-hj-safety-ros2/logs"
        os.makedirs(log_dir, exist_ok=True)

        self.log_path = os.path.join(log_dir, "cmd_state_log.csv")

        self.log_file = open(self.log_path, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)

        # CSV header
        self.csv_writer.writerow([
            "time",
            "cmd_linear",
            "cmd_angular",
            "x",
            "y",
            "theta"
        ])

        self.log_file.flush()

        self.get_logger().info(
            "Actuation node started with safety bounds, timeout protection, and command-state logging."
        )
        self.get_logger().info(f"Logging data to {self.log_path}")

    # Utility: Clamp with detection
    def clamp_with_flag(self, value, max_value):
        if value > max_value:
            return max_value, True
        elif value < -max_value:
            return -max_value, True
        return value, False

    # Odometry callback (robot state)
    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)

        self.theta = math.atan2(siny, cosy)

    def safe_callback(self, msg):

        self.last_cmd_time = self.get_clock().now()
        self.robot_stopped = False
        self.cmd_counter += 1

        bounded_msg = Twist()

        # Clamp linear velocity
        bounded_linear, lin_saturated = self.clamp_with_flag(
            msg.linear.x, MAX_LINEAR
        )

        # Clamp angular velocity
        bounded_angular, ang_saturated = self.clamp_with_flag(
            msg.angular.z, MAX_ANGULAR
        )

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

        # Log command + robot state
        timestamp = self.get_clock().now().nanoseconds / 1e9

        self.csv_writer.writerow([
            timestamp,
            bounded_linear,
            bounded_angular,
            self.x,
            self.y,
            self.theta
        ])

        self.log_file.flush()

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

        self.get_logger().info(
            f"Command rate: {self.cmd_counter} Hz"
        )

        self.cmd_counter = 0


def main(args=None):

    rclpy.init(args=args)

    node = ActuationNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.get_logger().info("Shutting down actuation node.")
    node.log_file.close()
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()