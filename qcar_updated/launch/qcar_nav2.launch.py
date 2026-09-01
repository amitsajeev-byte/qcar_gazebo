import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.actions import TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = get_package_share_directory('qcar_updated')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    maps_dir = os.path.join(pkg, 'maps')
    nav2_params = os.path.join(pkg, 'config', 'nav2', 'nav2_params.yaml')
    bt_dir = os.path.join(pkg, 'config', 'nav2', 'behavior_trees')

    # Overrides the placeholder default_nav_{to_pose,through_poses}_bt_xml keys in
    # nav2_params.yaml with the local BT trees in config/nav2/behavior_trees/ - replan only on
    # an invalid path/updated goal (not unconditionally every second) and no <Spin> recovery,
    # since this Ackermann car can't rotate in place. See the comments in those XML files.
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

    launch_sim = LaunchConfiguration('launch_sim')
    map_override = LaunchConfiguration('map_override')
    # map_override: run sim against a hardware-captured map (or vice versa) for side-by-side
    # comparison, bypassing the normal sim/hardware map selection below. Empty (default) keeps
    # the normal behavior.
    map_file = PythonExpression([
        "'", map_override, "' if '", map_override, "' != '' else ('",
        os.path.join(maps_dir, 'qcar_map_sim.yaml'),
        "' if '", launch_sim, "' == 'true' else '",
        os.path.join(maps_dir, 'qcar_map_hardware.yaml'), "')"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_sim',
            default_value='false',
            description='Bring up Gazebo + robot_state_publisher + controllers as well'
        ),

        DeclareLaunchArgument(
            'map_override',
            default_value='',
            description='Optional map yaml path to use instead of the normal sim/hardware pick '
                        '- e.g. run sim against a hardware-captured map for comparison'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'qcar_updated.launch.py')
            ),
            condition=IfCondition(launch_sim)
        ),

        # real-hardware equivalent when launch_sim:=false
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'qcar_visualize.launch.py')
            ),
            condition=UnlessCondition(launch_sim)
        ),

        # Delay everything below by 3 seconds
        TimerAction(
            period=3.0,
            actions=[

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': launch_sim,
                        'params_file': configured_nav2_params,
                        'map': map_file
                    }.items()
                ),
            ])
    ])