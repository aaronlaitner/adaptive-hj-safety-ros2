import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # This dynamically finds where your map is installed on your GS66
    map_config_path = os.path.join(
        get_package_share_directory('safe_rl_py'),
        'maps',
        'dam_good_map.yaml'
    )

    # # Path to your robot's URDF/Xacro file
    # pkg_path = get_package_share_directory('safe_rl_py')
    # xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file)

    # # The Robot State Publisher Node
    # params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    # node_robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[params]
    # )

    # 1. Map Server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        # Use the variable we defined above!
        parameters=[{'yaml_filename': map_config_path}, 
                    {'use_sim_time': use_sim_time}]
    )

    # 2. AMCL Node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
        'use_sim_time': use_sim_time,
        'set_initial_pose': True,
        'initial_pose_x': -1.931,
        'initial_pose_y': -0.295,
        'initial_pose_yaw': 0.066,
        'transform_tolerance': 0.6, # <--- Add this (increased from default 0.1)
        'global_frame_id': 'map',
        'odom_frame_id': 'odom',
        'base_frame_id': 'base_link',
        'update_min_d': 0.1,        # Update filter after 10cm movement
        'update_min_a': 0.1         # Update filter after ~11 deg rotation
    }]
    )

    # 3. Lifecycle Manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        rviz_node,
        #node_robot_state_publisher
    ])