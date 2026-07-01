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
    cartographer_config_dir = os.path.join(pkg, 'config', 'cartographer')
    configuration_basename = 'qcar_2d.lua'

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

        # cmd_vel -> drive_controller/steering_controller converter, so the
        # robot can be driven (e.g. via qcar_teleop_twist.py) while mapping.
        # Note: odom_to_tf.py is intentionally NOT launched here - Cartographer
        # itself broadcasts the odom->base transform (provide_odom_frame=true
        # in qcar_2d.lua); running both would fight over the same TF edge.
        Node(
            package='qcar_updated',
            executable='cmd_vel_to_drive.py',
            name='cmd_vel_to_drive',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
            ]
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-resolution', '0.05']
        ),
    ])
