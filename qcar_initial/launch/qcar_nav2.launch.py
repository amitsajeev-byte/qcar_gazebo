import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('qcar_initial')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    map_file = os.path.join(pkg, 'maps', 'qcar_map.yaml')
    nav2_params = os.path.join(pkg, 'config', 'nav2', 'nav2_params.yaml')

    return LaunchDescription([
        # Static odom->base transform since drive_controller publishes no real odometry
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_static',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base'],
            parameters=[{'use_sim_time': True}]
        ),

        # cmd_vel -> drive_controller/steering_controller converter
        Node(
            package='qcar_initial',
            executable='cmd_vel_to_drive.py',
            name='cmd_vel_to_drive',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': nav2_params,
                'map': map_file
            }.items()
        ),
    ])