import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # 1. FIXED: Point to the package that actually contains your 'worlds' folder
    pkg_safe_rl = get_package_share_directory('safe_rl_py') 
    
    # 2. Get the official TurtleBot3 package path
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # 3. Load your circular room world from the correct package
    world_file = LaunchConfiguration('world', default=os.path.join(pkg_safe_rl, 'worlds', 'circular_room.world'))

    # 4. Point to the official TurtleBot3 Burger model
    robot_model_path = os.path.join(pkg_tb3_gazebo, 'models', 'turtlebot3_burger', 'model.sdf') 

    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file}.items()
    )

    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_burger', 
            '-file', robot_model_path,
            '-x', '1.5', '-y', '0.0', '-z', '0.2'
        ],
        output='screen'
    )

    return LaunchDescription([
        start_gazebo_cmd,
        spawn_robot_cmd
    ])