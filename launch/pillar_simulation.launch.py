import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Setup paths
    pkg_safe_rl = get_package_share_directory('safe_rl_py')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Path to your pillar world file
    world_path = os.path.join(pkg_safe_rl, 'worlds', 'pillar_room.world')
    
    # 2. Declare Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # 3. Include Gazebo Launch
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # 4. Robot State Publisher (Required for LiDAR and TF)
    turtlebot3_gen_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 5. Spawn the Robot
    spawn_turtlebot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_burger',
            '-file', os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models', 'turtlebot3_burger', 'model.sdf'),
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )

    return LaunchDescription([
        gzserver_cmd,
        gzclient_cmd,
        turtlebot3_gen_launch,
        spawn_turtlebot_cmd
    ])