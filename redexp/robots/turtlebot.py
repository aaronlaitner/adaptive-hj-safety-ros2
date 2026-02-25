import rclpy
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import TransformStamped
import tf2_ros as tf
from time import sleep
import numpy as np
from threading import Lock
from redexp.brts.turtlebot_brt import (
    grid,
    turtlebot_2_no_model_mismatch,
    turtlebot_2_model_mismatch,
)

from redexp.config.turtlebot import (
    TASC_7001_X_BOUNDARY_LOWER,
    TASC_7001_X_BOUNDARY_UPPER,
    TASC_7001_Y_BOUNDARY_LOWER,
    TASC_7001_Y_BOUNDARY_UPPER,
    RADIUS,
    OBSTACLE_RADIUS,
)

VICON_TOPIC = "/vicon/ml_turtlebot_2/turtlebot_2"
VALUE_TOPIC = "/turtlebot/value"

ROTATION_OFFSET = -np.pi / 32
X_OFFSET = +0.0
Y_OFFSET = +0.0

DEBUG = False


class Turtlebot(Node):
    def __init__(self, goal_location, goal_r, model_mismatch) -> None:
        self.state = np.array([0.0, 0.0, 0.0])
        self.mutex = Lock()

        if model_mismatch:
            self.brt = np.load(
                "./redexp/brts/turtlebot_2_brt_speed_06_wMax_06_dstb.npy"
            )
            self.dyn = turtlebot_2_model_mismatch
        else:
            self.brt = np.load(
                "./redexp/brts/turtlebot_2_brt_speed_06_wMax_11_dstb.npy"
            )
            self.dyn = turtlebot_2_no_model_mismatch

        self.true_brt = np.load(
            "./redexp/brts/turtlebot_2_brt_speed_06_wMax_11_dstb.npy"
        )
        self.grid = grid

        self.goal_location = goal_location
        self.goal_r = goal_r

        # Ros2 infra
        super().__init__("turlebot_controller_node")
        self.pub = self.create_publisher(Twist, "/cmd_vel_mux/input/teleop",1)
        self.vicon_sub = self.create_subscription(
            TransformStamped,
            VICON_TOPIC,
            self.update_state,
            2**24
        )

        self.get_logger().info("Initialized turtlebot_controller_node")

    def update_state(self, ts_msg):
        with self.mutex:
            self.state = update_state(ts_msg)

    def get_state(self):
        with self.mutex:
            return self.state

    def set_action(self, action):
        if not self.in_bounds():
            print("TURTLEBOT2 OUT OF BOUNDS")
            vel_cmd = Twist()
        elif self.reach_goal():
            print("TURTLEBOT2 REACHED GOAL")
            vel_cmd = Twist()
        elif self.near_obs():
            print("TURTLEBOT2 TOO CLOSE TO OBSTACLE")
            vel_cmd = Twist()
        else:
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.6
            vel_cmd.angular.z = action[0]

        if DEBUG:
            value = grid.get_value(self.brt, self.get_state())
            print(f"DEBUG: {action=} {value=}")
        else:
            # add delay
            sleep(np.random.rand() / 4)
            self.pub.publish(vel_cmd)

    def in_bounds(self):
        (
            x,
            y,
            _,
        ) = self.get_state()
        return (
            TASC_7001_X_BOUNDARY_LOWER <= x <= TASC_7001_X_BOUNDARY_UPPER
            and TASC_7001_Y_BOUNDARY_LOWER <= y <= TASC_7001_Y_BOUNDARY_UPPER
        )

    def near_obs(self):
        state = self.get_state()
        reached_goal = np.linalg.norm(state[:2]) < (RADIUS + OBSTACLE_RADIUS)
        return reached_goal

    def reach_goal(self):
        state = self.get_state()
        reached_goal = (
            np.linalg.norm(state[:2] - self.goal_location) < self.dyn.r + self.goal_r
        )
        return reached_goal

    def get_brt_value(self):
        state = self.get_state()
        value = grid.get_value(self.true_brt, state)
        return value

class TurtleBotMonitor(Node):
    def __init__(self):
        super().__init__("turtlebot_monitor_node")
        self.get_logger().info("Initialized turtlebot_monitor_node")

        self.grid = grid
        self.brt = np.load("./redexp/brts/turtlebot_2_brt_speed_06_wMax_11_dstb.npy")

        self.sub = self.create_subscription(
            TransformStamped,
            VICON_TOPIC,
            self.monitor_update_state,
            2**24,
        )

    def calculate_heading(pose):
        x = pose.rotation.x
        y = pose.rotation.y
        z = pose.rotation.z
        w = pose.rotation.w
        quaternion = (x, y, z, w)

        # add offset to make yaw=0 face the computers
        rotation_quaternion = tf.transformations.quaternion_from_euler(
            0, 0, ROTATION_OFFSET
        )

        quaternion = tf.transformations.quaternion_multiply(rotation_quaternion, quaternion)

        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        return yaw

    def update_state(self, ts_msg):
        pose = ts_msg.transform
        x = pose.translation.x
        y = pose.translation.y

        x += X_OFFSET
        y += Y_OFFSET
        # theta off-set done in heading calculation
        theta = self.calculate_heading(pose)

        return np.array([x, y, theta])
    
    def monitor_update_state(self, ts_msg):
        state = self.update_state(ts_msg)
        value = self.grid.get_value(
            self.brt,
            state,
        )
        self.get_logger().debug(f"turtlebot2 state: {state}\n\
                                value = {value}")

def main(args=None):
    try:
        rclpy.init(args=args)
        monitor_node = TurtleBotMonitor()
        rclpy.spin(monitor_node)
    except ROSInterruptException:
        monitor_node.get_logger.info("Shutdown")
        rclpy.shutdown()

if __name__ == "__main__":
    main()
