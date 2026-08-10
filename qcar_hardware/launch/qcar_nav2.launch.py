import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = get_package_share_directory('qcar_hardware')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    map_file = os.path.join(pkg, 'maps', 'qcar_map.yaml')
    nav2_params = os.path.join(pkg, 'config', 'nav2', 'nav2_params.yaml')
    bt_dir = os.path.join(pkg, 'config', 'nav2', 'behavior_trees')

    # Same BT override as qcar_updated's sim version - replan only on an
    # invalid path/updated goal (not unconditionally every second) and no
    # <Spin> recovery, since this Ackermann car can't rotate in place.
    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params,
        param_rewrites={
            'default_nav_to_pose_bt_xml': os.path.join(
                bt_dir, 'navigate_to_pose_w_replanning_and_recovery.xml'),
            'default_nav_through_poses_bt_xml': os.path.join(
                bt_dir, 'navigate_through_poses_w_replanning_and_recovery.xml'),
        },
        convert_types=True
    )

    return LaunchDescription([
        # robot_state_publisher + joint_state_publisher + rviz2 (qcar.rviz).
        # No Gazebo, no Cartographer here - Phase 3 uses AMCL against the
        # static map saved in Phase 2, not live SLAM.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'qcar_visualize.launch.py')
            )
        ),

        # Small delay so robot_state_publisher's TF is up before AMCL starts
        # looking up odom->base at its first scan callback.
        TimerAction(
            period=2.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': 'false',
                        'params_file': configured_nav2_params,
                        'map': map_file
                    }.items()
                ),
            ])
    ])
