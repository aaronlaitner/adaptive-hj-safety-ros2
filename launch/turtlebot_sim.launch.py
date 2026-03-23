import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_PARAMS_FILES = {
    "map_file": os.path.join(
        get_package_share_directory("safe_rl_py"),
        "maps",
        "pillar_room.yaml",
    ),
    "world_file": os.path.join(
        get_package_share_directory("safe_rl_py"), "worlds", "pillar_room_big.world"
    ),
}


def launch_setup(context, *args, **kwargs):
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    tb3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")
    nav2_bringup_launch_dir = os.path.join(nav2_bringup_dir, "launch")
    map_file = LaunchConfiguration("map_file").perform(context)

    world = LaunchConfiguration("world")

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[{"use_sim_time": True}, {"yaml_filename": map_file}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[
            {"use_sim_time": True},
        ],
        #arguments=["-d" + rviz_settings_file],
        output={"both": "log"},
    )

    urdf = os.path.join(tb3_gazebo_dir, "urdf", "turtlebot3_burger.urdf")
    with open(urdf, "r") as infp:
        robot_description = infp.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True, "robot_description": robot_description}],
    )

    gazebo_server = ExecuteProcess(
        cmd=[
            "gzserver",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            world,
        ],
        cwd=[nav2_bringup_launch_dir],
        output="screen",
    )

    gazebo_client = ExecuteProcess(
        cmd=["gzclient"],
        cwd=[nav2_bringup_launch_dir],
        output="screen",
    )

    pose = {
        "x": LaunchConfiguration("x_pose", default="-1.5"),
        "y": LaunchConfiguration("y_pose", default="2.5"),
        "z": LaunchConfiguration("z_pose", default="0.01"),
        "R": LaunchConfiguration("roll", default="0.0"),
        "P": LaunchConfiguration("pitch", default="0.0"),
        "Y": LaunchConfiguration("yaw", default="0.0"),
    }

    gazebo_spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity",
            "tb3_burger",
            "-file",
            os.path.join(tb3_gazebo_dir, "models", "turtlebot3_burger", "model.sdf"),
            "-x",
            pose["x"],
            "-y",
            pose["y"],
            "-z",
            pose["z"],
            "-R",
            pose["R"],
            "-P",
            pose["P"],
            "-Y",
            pose["Y"],
        ],
    )

    return [
        gazebo_server,
        gazebo_client,
        gazebo_spawner,
        robot_state_publisher,
        # map_server_node,
        #rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_file",
                default_value=DEFAULT_PARAMS_FILES["map_file"],
                description="Map file to use",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=DEFAULT_PARAMS_FILES["world_file"],
                description="Full path to world model file to load",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
