import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    

    # Not sure how it would help us, gotta read about it more. 
    # You can copy the default file from the slam_toolbox package to your own 'config' folder
    slam_config_file = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'config',
        'mapper_params_online_async.yaml'
    )

    
    # Exports the SLAM map and pose estimates to ROS topics, which can be visualized in RViz2 or used by other nodes.
    start_sync_slam_toolbox_node = Node(
        parameters=[
            slam_config_file,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node', # Switched from async to sync
        name='slam_toolbox',
        output='screen'
    )

    # For custom RViz configuration, you can create a .rviz file in your package and specify it here. Otherwise, it will use the default RViz configuration.
    # rviz_config_dir = os.path.join(get_package_share_directory('adaptive-hj-safety-ros2'), 'rviz', 'slam.rviz')
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
        # arguments=['-d', rviz_config_dir]
    )


    # For online SLAM, keep False for gazebo clock. Need to counteract with real robot time.
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        start_sync_slam_toolbox_node,  # <--- Fixed this line
        start_rviz
    ])