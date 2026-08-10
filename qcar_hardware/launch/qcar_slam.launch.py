import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('qcar_hardware')
    cartographer_config_dir = os.path.join(pkg, 'config', 'cartographer')
    configuration_basename = 'qcar_2d.lua'

    return LaunchDescription([
        # robot_state_publisher + joint_state_publisher + rviz2 (qcar.rviz).
        # Supplies the base->lidar static transform Cartographer needs to
        # project /scan into its tracking_frame ("base").
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'qcar_visualize.launch.py')
            )
        ),

        # qcar_2d.lua has use_odometry=false and provide_odom_frame=true -
        # Cartographer computes the entire map->odom->base TF chain itself
        # from /scan alone (pure LiDAR scan-matching SLAM). No /odom or TF
        # needed from qcar_bridge.py/qcar_relay_node.py for this to work -
        # unlike the Gazebo-sim version of this launch (qcar_updated's
        # qcar_slam.launch.py), there's no drive-plugin odom broadcaster to
        # conflict with here, so no disable_odom_tf equivalent is needed.
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
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
            arguments=['-resolution', '0.05']
        ),
    ])
