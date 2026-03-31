import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Define the package name and get its share directory
    pkg_name = 'adaptive-hj-safety-ros2'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Pass the world you want to learn in. You can create your own world file using Gazebo and place it in the 'worlds' folder of your package.
    world_file = os.path.join(pkg_share, 'worlds', 'pillar_room.world') # Start with normal pillar room.
    
    # Launch Gazebo (The Environment)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('adaptive-hj-safety-ros2'), 'launch', 'turtlebot_sim.launch.py')
        ]),
        launch_arguments={'world': world_file}.items()
    )

    # Launch SLAM (The Logic)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items() # Critical for Gazebo!
    )

    # Launch RViz with your saved config (The Visualization)
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam_view.rviz') # Will provide the file for you, but feel free to customize it.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        gazebo,
        slam,
        rviz
    ])