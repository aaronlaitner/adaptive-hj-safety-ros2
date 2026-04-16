import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import csv
import os
import math

# SAFETY CONFIG
MAX_LINEAR = 0.22
MAX_ANGULAR = 2.5
CMD_TIMEOUT = 1.0

# MISMATCH THRESHOLDS
ERROR_THRESHOLD_X = 0.05
ERROR_THRESHOLD_Y = 0.05
ERROR_THRESHOLD_THETA = 0.2

# ADAPTIVE RESPONSE CONFIG
MAX_ERROR_FOR_NORMALIZATION = 0.2
MIN_SCALE = 0.1


class ActuationNode(Node):

    def __init__(self):
        super().__init__('actuation_node')

        # Mode switching
        self.mode = "adaptive"  # "baseline" or "adaptive"

        # Safety scaling
        self.safety_scale = 1.0

        self.subscription = self.create_subscription(
            Twist, '/safe_cmd', self.safe_callback, 10
        )

        self.publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_theta = 0.0

        self.prev_time = None

        self.last_cmd_time = self.get_clock().now()
        self.robot_stopped = False
        self.cmd_counter = 0

        self.timeout_timer = self.create_timer(0.1, self.check_timeout)
        self.rate_timer = self.create_timer(1.0, self.report_rate)

        # LOGGING SETUP
        log_dir = "/workspaces/ros2_ws/src/adaptive-hj-safety-ros2/logs"
        os.makedirs(log_dir, exist_ok=True)

        self.log_path = os.path.join(log_dir, "cmd_state_log.csv")
        self.log_file = open(self.log_path, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)

        self.csv_writer.writerow([
            "time",
            "cmd_linear",
            "cmd_angular",
            "x", "y", "theta",
            "expected_x", "expected_y", "expected_theta",
            "error_x", "error_y", "error_theta",
            "mismatch_flag",
            "mismatch_score",
            "confidence",
            "mode",
            "correction_linear",
            "correction_angular",
            "safety_scale"
        ])

        self.log_file.flush()

        self.get_logger().info("Final Actuation Node (Week 4+5+6) Started")

    def clamp_with_flag(self, value, max_value):
        if value > max_value:
            return max_value, True
        elif value < -max_value:
            return -max_value, True
        return value, False

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

        # Clamp commands
        bounded_linear, _ = self.clamp_with_flag(msg.linear.x, MAX_LINEAR)
        bounded_angular, _ = self.clamp_with_flag(msg.angular.z, MAX_ANGULAR)

        bounded_msg = Twist()
        bounded_msg.linear.x = bounded_linear
        bounded_msg.angular.z = bounded_angular

        timestamp = self.get_clock().now().nanoseconds / 1e9

        dt = 0.0 if self.prev_time is None else timestamp - self.prev_time
        self.prev_time = timestamp

        # EXPECTED MODEL
        expected_x = self.prev_x + bounded_linear * math.cos(self.prev_theta) * dt
        expected_y = self.prev_y + bounded_linear * math.sin(self.prev_theta) * dt
        expected_theta = self.prev_theta + bounded_angular * dt

        # ERRORS
        error_x = self.x - expected_x
        error_y = self.y - expected_y
        error_theta = self.theta - expected_theta

        # MISMATCH DETECTION
        mismatch = (
            abs(error_x) > ERROR_THRESHOLD_X or
            abs(error_y) > ERROR_THRESHOLD_Y or
            abs(error_theta) > ERROR_THRESHOLD_THETA
        )

        # MISMATCH SCORE 
        mismatch_score = (
            0.4 * abs(error_x) +
            0.4 * abs(error_y) +
            0.2 * abs(error_theta)
        )

        mismatch_score = min(mismatch_score / MAX_ERROR_FOR_NORMALIZATION, 1.0)
        confidence = 1.0 - mismatch_score

        # Safety filter adaptation
        if mismatch_score > 0.5:
            self.safety_scale = 0.5
        elif mismatch_score > 0.2:
            self.safety_scale = 0.7
        else:
            self.safety_scale = 1.0

        # Active compensation
        Kp_pos = 0.5
        Kp_theta = 1.0

        position_error = math.sqrt(error_x**2 + error_y**2)

        correction_linear = Kp_pos * position_error
        correction_angular = Kp_theta * error_theta

        scale = max(1.0 - mismatch_score, MIN_SCALE)

        safe_msg = Twist()
        safe_msg.linear.x = (bounded_linear * scale + correction_linear) * self.safety_scale
        safe_msg.angular.z = (bounded_angular * scale + correction_angular) * self.safety_scale

        # MODE SWITCH
        if self.mode == "baseline":
            self.publisher.publish(bounded_msg)

        elif self.mode == "adaptive":
            if mismatch:
                self.publisher.publish(safe_msg)
            else:
                self.publisher.publish(bounded_msg)

        # LOGGING
        self.csv_writer.writerow([
            timestamp,
            bounded_linear,
            bounded_angular,
            self.x,
            self.y,
            self.theta,
            expected_x,
            expected_y,
            expected_theta,
            error_x,
            error_y,
            error_theta,
            int(mismatch),
            mismatch_score,
            confidence,
            self.mode,
            correction_linear,
            correction_angular,
            self.safety_scale
        ])

        self.log_file.flush()

        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_theta = self.theta

    def check_timeout(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9

        if elapsed > CMD_TIMEOUT and not self.robot_stopped:
            self.publisher.publish(Twist())
            self.robot_stopped = True

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
    finally:
        node.log_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()