import os

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import (
    Twist, 
    TransformStamped,
    PoseWithCovarianceStamped,
)
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

from redexp.brts.export import BRT_CONFIG

from redexp.config.turtlebot import TB_CONFIG

# TOPICS
VICON_TOPIC = "/vicon/ml_turtlebot_2/turtlebot_2"
GAZEBO_STATE_TOPIC = "/gazebo/model_states"
VALUE_TOPIC = "/turtlebot/value"

DEBUG = False

# VICON LEGACY
ROTATION_OFFSET = -np.pi / 32
X_OFFSET = +0.0
Y_OFFSET = +0.0

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
# end of VICON LEGACY

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

def state_from_amcl_msg(msg):
    state = msg.pose.pose
    x = state.position.x
    y = state.position.y
    _, _, theta = euler_from_quaternion([
        state.orientation.x,
        state.orientation.y,
        state.orientation.z,
        state.orientation.w,
    ])
    return np.array([x, y, theta])

# Organized arguments
VICON_PARAMS = {
    'state_msg_type': TransformStamped,
    'state_topic': VICON_TOPIC,
    'state_msg_fn': state_from_tf_msg,
    'ctrl_pub_topic': "/cmd_vel_mux/input/teleop",
}

GAZEBO_PARAMS = {
    'state_msg_type': ModelStates,
    'state_topic': GAZEBO_STATE_TOPIC,
    'state_msg_fn': state_from_gazebo_msg,
    'ctrl_pub_topic': "/cmd_vel",
}

# AMCL params
IRL_PARAMS = {
    'state_msg_type': PoseWithCovarianceStamped,
    'state_topic': "/amcl_pose",
    'state_msg_fn': state_from_amcl_msg,
    'ctrl_pub_topic': "/cmd_vel",
}

param_map = {
    "default": IRL_PARAMS,
    "vicon": VICON_PARAMS,
    "gazebo": GAZEBO_PARAMS,
    "irl": IRL_PARAMS,
}
# end of organized arguments

class Turtlebot():
    def __init__(self, goal_location, goal_r, model_mismatch, env_type, robot_type) -> None:
        self.state = np.array([0.0, 0.0, 0.0])
        
        self.tb_cfg = TB_CONFIG.get(robot_type, TB_CONFIG["default"])
        brt_cfg = BRT_CONFIG.get(robot_type, BRT_CONFIG["default"])

        share_folder = get_package_share_directory("safe_rl_py")
        if model_mismatch:
            self.brt = np.load(
                os.path.join(share_folder, "brts", brt_cfg['brt_mismatch_file'])
            )
            self.dyn = brt_cfg['dyn_mistmatch']
        else:
            self.brt = np.load(
                os.path.join(share_folder, "brts", brt_cfg['brt_no_mismatch_file'])
            )
            self.dyn = brt_cfg['dyn_no_mistmatch']

        self.true_brt = np.load(
            os.path.join(share_folder, "brts", brt_cfg['brt_no_mismatch_file'])
        )
        self.grid = brt_cfg['grid']

        self.goal_location = goal_location
        self.goal_r = goal_r

        # ros2 node IPC
        self.control_shm = Array(c_float, [0.0, 0.0], lock=True)
        self.state_shm = Array(c_float, [0.0, 0.0, 0.0], lock=True)

        # start the controller node
        self.spin_subprocess = Process(
            target=self._spin_controller, 
            args=(env_type,))
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
            self.control_shm[:] = [self.tb_cfg['SPEED'], action[0]]

        if DEBUG:
            value = self.grid.get_value(self.brt, state)
            print(f"DEBUG: {action=} {value=} {state=}")

    def stop(self):
        self.control_shm[:] = [0.0, 0.0]

    def in_bounds(self, state=None):
        if state is None:
            state = self.get_state()
        x, y, _ = state
        return (
            self.tb_cfg['X_BOUNDARY_LOWER'] <= x <= self.tb_cfg['X_BOUNDARY_UPPER']
            and self.tb_cfg['Y_BOUNDARY_LOWER'] <= y <= self.tb_cfg['Y_BOUNDARY_UPPER']
        )

    def _near_obs(self, state=None):
        if state is None:
            state = self.get_state()
        reached_goal = np.linalg.norm(state[:2]) < (
            self.tb_cfg['RADIUS'] + self.tb_cfg['OBSTACLE_RADIUS'])
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
        value = self.grid.get_value(self.true_brt, state)
        return value

class TurtlebotController(Node):
    def __init__(self, control_shm, state_shm, env_type="default"):
        # Ros2 infra
        super().__init__("turlebot_controller_node")

        self.control_shm = control_shm
        self.state_shm = state_shm
        self.stopped = False

        # periodically publish the control
        hz = 20.0
        self.ctrl_timer = self.create_timer(1.0 / hz, self.control_timer_callback)
        
        # get states from topic of choice,
        # publish to corresponding topic
        env_params = param_map.get(env_type)

        self.get_logger().info(f"Turtlebot controller using: {env_type=}. Environment params:\n{env_params}")
        self.create_subscription(
            env_params['state_msg_type'],
            env_params['state_topic'],
            self.update_state,
            10
        )
        self._get_state = env_params['state_msg_fn']

        self.pub = self.create_publisher(Twist, env_params['ctrl_pub_topic'], 10)

        self.get_logger().info("Initialized turtlebot_controller_node")
    
    def update_state(self, msg):
        self.state_shm[:] = self._get_state(msg)
        # self.get_logger().info(f"Robot state: {self.state_shm[:]}")
    
    def control_timer_callback(self):
        action = self.control_shm[:]

        # to allow teleop when robot needs to be manually moved
        should_stop = action == [0.0, 0.0]
        if should_stop and self.stopped:
            return
        self.stopped = should_stop

        # # debug
        # if (np.random.random() < 0.01):
        #     self.get_logger().info(f"Action: {action}")
        
        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]
        vel_cmd.angular.z = action[1]

        # sleep(np.random.rand() / 4) # may not need this since localization
        self.pub.publish(vel_cmd)

class TurtlebotMonitor(Node):
    def __init__(self):
        super().__init__("turtlebot_monitor_node")
        self.declare_parameter("env_type", "default")
        self.declare_parameter("robot_type", "default")

        env_type = self.get_parameter("env_type").get_parameter_value().string_value
        robot_type = self.get_parameter("robot_type").get_parameter_value().string_value

        self.get_logger().info("Initialized turtlebot_monitor_node")

        
        brt_cfg = BRT_CONFIG.get(robot_type, BRT_CONFIG["default"])
        self.grid = brt_cfg['grid']

        share_folder = get_package_share_directory("safe_rl_py")
        self.brt = np.load(
            os.path.join(share_folder, "brts", brt_cfg['brt_no_mismatch_file'])
        )

        
        env_params = param_map.get(env_type)

        self.get_logger().info(f"Turtlebot monitor using: {env_type=}")
        self.create_subscription(
            env_params['state_msg_type'],
            env_params['state_topic'],
            self.log_state,
            10
        )
        self._get_state = env_params['state_msg_fn']

        self.get_logger().info("Initialized turtlebot_monitor_node")
    
    def log_state(self, msg):
        state = self._get_state(msg)
        value = self.grid.get_value(
            self.brt,
            state,
        )
        self.get_logger().info(f"turtlebot state: {state}\n\
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
