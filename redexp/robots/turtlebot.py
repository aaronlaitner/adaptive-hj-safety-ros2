from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.msg import ModelStates
from tf_transformations import (
    quaternion_from_euler, 
    quaternion_multiply, 
    euler_from_quaternion)
from time import sleep
import numpy as np

from multiprocessing import (
    Process,
    Array
)
from ctypes import c_float

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
GAZEBO_STATE_TOPIC = "/gazebo/model_states"
VALUE_TOPIC = "/turtlebot/value"

ROTATION_OFFSET = -np.pi / 32
X_OFFSET = +0.0
Y_OFFSET = +0.0

DEBUG = False

current_file_path = Path(__file__).resolve()
project_folder = current_file_path.parents[2]

def state_from_tf_msg(ts_msg):
    pose = ts_msg.transform
    x = pose.translation.x
    y = pose.translation.y

    x += X_OFFSET
    y += Y_OFFSET
    # theta off-set done in heading calculation
    theta = calculate_heading(pose)

    return np.array([x, y, theta])

def calculate_heading(pose):
    x = pose.rotation.x
    y = pose.rotation.y
    z = pose.rotation.z
    w = pose.rotation.w
    quaternion = (x, y, z, w)

    # add offset to make yaw=0 face the computers
    rotation_quaternion = quaternion_from_euler(
        0, 0, ROTATION_OFFSET
    )

    quaternion = quaternion_multiply(rotation_quaternion, quaternion)

    roll, pitch, yaw = euler_from_quaternion(quaternion)
    return yaw

def state_from_gazebo_msg(msg):
    try:
        id = msg.name.index("tb3_burger") # should be the same name in launch file's spawner
    except:
        return np.array([0.0, 0.0, 0.0])
    x = msg.pose[id].position.x
    y = msg.pose[id].position.y
    _, _, theta = euler_from_quaternion([
        msg.pose[id].orientation.x,
        msg.pose[id].orientation.y,
        msg.pose[id].orientation.z,
        msg.pose[id].orientation.w,
    ])
    return np.array([x, y, theta])

class Turtlebot():
    def __init__(self, goal_location, goal_r, model_mismatch, use_gazebo) -> None:
        self.state = np.array([0.0, 0.0, 0.0])

        if model_mismatch:
            self.brt = np.load(
                project_folder / "redexp/brts/turtlebot_2_brt_speed_06_wMax_06_dstb.npy"
            )
            self.dyn = turtlebot_2_model_mismatch
        else:
            self.brt = np.load(
                project_folder / "redexp/brts/turtlebot_2_brt_speed_06_wMax_11_dstb.npy"
            )
            self.dyn = turtlebot_2_no_model_mismatch

        self.true_brt = np.load(
            project_folder / "redexp/brts/turtlebot_2_brt_speed_06_wMax_11_dstb.npy"
        )
        self.grid = grid

        self.goal_location = goal_location
        self.goal_r = goal_r

        # ros2 node IPC
        self.control_shm = Array(c_float, [0.0, 0.0], lock=True)
        self.state_shm = Array(c_float, [0.0, 0.0, 0.0], lock=True)

        # start the controller node
        self.spin_subprocess = Process(
            target=self._spin_controller, 
            args=(use_gazebo,))
        self.spin_subprocess.start()
        
    def __del__(self):
        # effort to clean up the controller process
        # __del__ is not guaranteed
        # RAII is not feasible/clunky in Python
        # if it lives as an orphan process, should be fine (?)
        if self.spin_subprocess.is_alive():
            self.spin_subprocess.join()

    def _spin_controller(self, use_gazebo):
        try:
            rclpy.init()
            controller_node = TurtlebotController(
                self.control_shm, 
                self.state_shm, 
                use_gazebo)
            rclpy.spin(controller_node)
        except ROSInterruptException:
            controller_node.get_logger.info("Controller shutdown")
            rclpy.shutdown()

    def get_state(self):
        state = self.state_shm[:]
        return state

    def set_action(self, action):
        state = self.get_state()
        if not self.in_bounds(state):
            print("TURTLEBOT OUT OF BOUNDS")
            self.control_shm[:] = [0.0, 0.0]
        elif self.reach_goal(state):
            print("TURTLEBOT REACHED GOAL")
            self.control_shm[:] = [0.0, 0.0]
        elif self._near_obs(state):
            print("TURTLEBOT TOO CLOSE TO OBSTACLE")
            self.control_shm[:] = [0.0, 0.0]
        else:
            # linear velocity is constant
            self.control_shm[:] = [0.2, action[0]]

        if DEBUG:
            value = grid.get_value(self.brt, state)
            print(f"DEBUG: {action=} {value=} {state=}")

    def in_bounds(self, state=None):
        if state is None:
            state = self.get_state()
        x, y, _ = state
        return (
            TASC_7001_X_BOUNDARY_LOWER <= x <= TASC_7001_X_BOUNDARY_UPPER
            and TASC_7001_Y_BOUNDARY_LOWER <= y <= TASC_7001_Y_BOUNDARY_UPPER
        )

    def _near_obs(self, state=None):
        if state is None:
            state = self.get_state()
        reached_goal = np.linalg.norm(state[:2]) < (RADIUS + OBSTACLE_RADIUS)
        return reached_goal

    def reach_goal(self, state=None):
        if state is None:
            state = self.get_state()
        reached_goal = (
            np.linalg.norm(state[:2] - self.goal_location) < self.dyn.r + self.goal_r
        )
        return reached_goal

    def get_brt_value(self, state=None):
        if state is None:
            state = self.get_state()
        value = grid.get_value(self.true_brt, state)
        return value

class TurtlebotController(Node):
    def __init__(self, control_shm, state_shm, use_gazebo=False):
        # Ros2 infra
        super().__init__("turlebot_controller_node")

        self.control_shm = control_shm
        self.state_shm = state_shm
        self.stopped = False

        # periodically publish the control
        hz = 50.0
        self.create_timer(1.0 / hz, self.control_timer_callback)
        
        # get states from topic of choice,
        # publish to corresponding topic
        if not use_gazebo:
            self.get_logger().info("Turtlebot controller using VICON")
            self.create_subscription(
                TransformStamped,
                VICON_TOPIC,
                self.update_state,
                10
            )
            self._get_state = state_from_tf_msg

            self.pub = self.create_publisher(Twist, "/cmd_vel_mux/input/teleop", 1)
        else:
            self.get_logger().info("Turtlebot controller using Gazebo")
            self.create_subscription(
                ModelStates,
                GAZEBO_STATE_TOPIC,
                self.update_state,
                10
            )
            self._get_state = state_from_gazebo_msg

            self.pub = self.create_publisher(Twist, "/cmd_vel", 1)

        self.get_logger().info("Initialized turtlebot_controller_node")
    
    def update_state(self, msg):
        self.state_shm[:] = self._get_state(msg)
        # self.get_logger().info(f"Robot state: {self.state_shm[:]}")
    
    def control_timer_callback(self):
        action = self.control_shm[:]

        # to allow teleop when robot needs to be manually moved
        should_stop = action == [0.0, 0.0, 0.0]
        if should_stop and self.stopped:
            return
        self.stopped = should_stop

        # # debug
        # if (np.random.random() < 0.01):
        #     self.get_logger().info(f"Action: {action}")
        
        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]
        vel_cmd.angular.z = action[1]

        self.pub.publish(vel_cmd)

class TurtlebotMonitor(Node):
    def __init__(self):
        super().__init__("turtlebot_monitor_node")
        self.declare_parameter("use_gazebo", True)

        self.get_logger().info("Initialized turtlebot_monitor_node")

        self.grid = grid
        self.brt = np.load(project_folder / "redexp/brts/turtlebot_2_brt_speed_06_wMax_11_dstb.npy")

        use_gazebo = self.get_parameter("use_gazebo").get_parameter_value().bool_value
        if not use_gazebo:
            self.get_logger().info("Turtlebot monitor using VICON")
            self.create_subscription(
                TransformStamped,
                VICON_TOPIC,
                self.log_state,
                10
            )
            self._get_state = state_from_tf_msg
        else:
            self.get_logger().info("Turtlebot monitor using Gazebo")
            self.create_subscription(
                ModelStates,
                GAZEBO_STATE_TOPIC,
                self.log_state,
                10
            )
            self._get_state = state_from_gazebo_msg

        self.get_logger().info("Initialized turtlebot_monitor_node")
    
    def log_state(self, msg):
        state = self._get_state(msg)
        value = self.grid.get_value(
            self.brt,
            state,
        )
        self.get_logger().info(f"turtlebot2 state: {state}\n\
                                value = {value}")

def main(args=None):
    try:
        rclpy.init(args=args)
        monitor_node = TurtlebotMonitor()
        rclpy.spin(monitor_node)
    except ROSInterruptException:
        monitor_node.get_logger.info("Monitor shutdown")
        rclpy.shutdown()

if __name__ == "__main__":
    main()
