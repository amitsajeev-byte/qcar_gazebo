import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

# No-Gazebo sim-vs-hardware-map comparison mode: Gazebo's world (worlds/myworld.world) is fixed
# geometry unrelated to any map we capture from the real QCar, so its simulated laser can't see a
# captured map's walls - useless for comparing planner/controller reactions against a real map.
# This launch instead runs a pure kinematic "fake robot" (fake_odom_node.py, cmd_vel integrated
# into odom, no physics/collision) plus map_server + navigation stack against nav2_params_compare
# .yaml, which adds a static_layer to the local costmap so MPPI still reacts to the real map's
# walls despite there being no live sensor. A static map->odom identity transform stands in for
# AMCL, since there's no live scan for a localizer to correct against.
#
# See CHANGELOG.md 2026-09-01 and TUNING.md for context.


def generate_launch_description():
    pkg = get_package_share_directory('qcar_updated')
    xacro_file = os.path.join(pkg, 'urdf', 'qcar_model.xacro')
    maps_dir = os.path.join(pkg, 'maps')
    nav2_params = os.path.join(pkg, 'config', 'nav2', 'nav2_params_compare.yaml')
    bt_dir = os.path.join(pkg, 'config', 'nav2', 'behavior_trees')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_yaml = LaunchConfiguration('map')

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

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' disable_odom_tf:=true']),
        value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(maps_dir, 'qcar_map_hardware.yaml'),
            description='Map yaml to compare planner/controller behavior against'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': False}]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        # Stands in for AMCL: no live sensor here for a localizer to correct against, so treat
        # map and odom as coincident.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),
        Node(
            package='qcar_updated',
            executable='fake_odom_node.py',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': False, 'yaml_filename': map_yaml}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server']
            }]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'autostart': 'true',
                'params_file': configured_nav2_params
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg, 'rviz', 'qcar.rviz')],
            parameters=[{'use_sim_time': False}]
        ),
    ])
