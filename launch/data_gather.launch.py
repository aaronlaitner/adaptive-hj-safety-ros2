import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    pkg_path = get_package_share_directory('safe_rl_py')
    
    # Paths
    map_config_path = os.path.join(pkg_path, 'maps', 'dam_good_map.yaml')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Optional: Path to a saved RViz config file
    # If you save your RViz config as 'data_collect.rviz' in your rviz folder:
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'data_collect.rviz')

    # 1. Robot State Publisher (The "Model")
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}]
    )

    # 2. Map & Localization
    map_server_node = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        parameters=[{'yaml_filename': map_config_path}]
    )
    amcl_node = Node(
        package='nav2_amcl', executable='amcl', name='amcl',
        parameters=[{'use_sim_time': use_sim_time, 'set_initial_pose': True,
                     'initial_pose_x': -1.931, 'initial_pose_y': -0.295, 'initial_pose_yaw': 0.066}]
    )
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
        parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}]
    )

    # 3. Data Flow (Proxy & Collector)
    odom_proxy_node = Node(package='safe_rl_py', executable='odom_proxy', name='odom_proxy')
    nn_data_collector_node = Node(package='safe_rl_py', executable='nn_data_collector', name='nn_data_collector', output='screen')

    # 4. RViz (Configured)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path], # Load the saved view
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        node_robot_state_publisher,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        odom_proxy_node,
        nn_data_collector_node,
        rviz_node
    ])