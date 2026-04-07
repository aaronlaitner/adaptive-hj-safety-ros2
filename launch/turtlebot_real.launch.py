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
    "nav2_params_file": os.path.join(
        get_package_share_directory("safe_rl_py"),
        "params",
        "nav2_params.yaml",
    ),
    "map_file": os.path.join(
        get_package_share_directory("safe_rl_py"),
        "maps",
        "dam_good_map.yaml",
    ),
    "rviz_settings": os.path.join(
        get_package_share_directory("safe_rl_py"),
        "rviz",
        "sim_srl_settings.rviz",
    ),
}


def launch_setup(context, *args, **kwargs):
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    tb3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")
    nav2_bringup_launch_dir = os.path.join(nav2_bringup_dir, "launch")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    map_file = LaunchConfiguration("map_file").perform(context)
    rviz_settings_file = LaunchConfiguration("rviz_settings_file").perform(context)


    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[{"use_sim_time": False}, {"yaml_filename": map_file}],
    )

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[
            {"use_sim_time": False},
            {"autostart": True},
            {"node_names": ["map_server", "amcl",]},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[
            {"use_sim_time": False},
        ],
        arguments=["-d" + rviz_settings_file],
        output={"both": "log"},
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'set_initial_pose': True,
            'initial_pose_x': 0.0,
            'initial_pose_y': -2.0,
            'initial_pose_yaw': 0.0,
            'transform_tolerance': 0.6, # <--- Add this (increased from default 0.1)
            'global_frame_id': 'map',
            'odom_frame_id': 'odom',
            'base_frame_id': 'base_link',
            'update_min_d': 0.1,        # Update filter after 10cm movement
            'update_min_a': 0.1         # Update filter after ~11 deg rotation
        }]
    )

    urdf = os.path.join(tb3_gazebo_dir, "urdf", "turtlebot3_burger.urdf")
    with open(urdf, "r") as infp:
        robot_description = infp.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": False, "robot_description": robot_description}],
    )

    return [
        robot_state_publisher,
        map_server_node,
        lifecycle_manager_node,
        amcl_node,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=DEFAULT_PARAMS_FILES["nav2_params_file"],
                description="Nav2 parameters file to use",
            ),
            DeclareLaunchArgument(
                "map_file",
                default_value=DEFAULT_PARAMS_FILES["map_file"],
                description="Map file to use",
            ),
            DeclareLaunchArgument(
                "rviz_settings_file",
                default_value=DEFAULT_PARAMS_FILES["rviz_settings"],
                description="Rviz settings file to use",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
