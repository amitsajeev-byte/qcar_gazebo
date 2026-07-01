import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('qcar_updated')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    map_file = os.path.join(pkg, 'maps', 'qcar_map.yaml')
    nav2_params = os.path.join(pkg, 'config', 'nav2', 'nav2_params.yaml')

    launch_sim = LaunchConfiguration('launch_sim')

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_sim',
            default_value='true',
            description='Bring up Gazebo + robot_state_publisher + controllers as well'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'qcar_updated.launch.py')
            ),
            condition=IfCondition(launch_sim)
        ),

        # cmd_vel -> drive_controller/steering_controller converter
        Node(
            package='qcar_initial',
            executable='cmd_vel_to_drive.py',
            name='cmd_vel_to_drive',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # Real odometry TF from p3d plugin
        Node(
            package='qcar_initial',
            executable='odom_to_tf.py',
            name='odom_to_tf',
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