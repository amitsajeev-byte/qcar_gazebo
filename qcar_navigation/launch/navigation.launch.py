import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('qcar_navigation')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    slam = LaunchConfiguration('slam')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    world_arg = DeclareLaunchArgument(
        'world', default_value=os.path.join(pkg_share, 'worlds', 'myworld.world'))
    map_arg = DeclareLaunchArgument(
        'map', default_value=os.path.join(pkg_share, 'maps', 'qcar_map.yaml'),
        description='Full path to the map yaml file to load for localization')
    params_file_arg = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'))
    slam_arg = DeclareLaunchArgument(
        'slam', default_value='False',
        description='Run slam_toolbox instead of AMCL localization against a known map. '
                    'nav2_bringup evaluates this as a raw Python expression, so it must be '
                    'the capitalized literal True/False, not lowercase.')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'gui': gui,
            'world': world,
        }.items(),
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'params_file': params_file,
            'slam': slam,
            'autostart': 'true',
            # nav2_bringup's default composed bringup (all servers loaded as
            # components into one nav2_container) has been observed to stall
            # indefinitely with no error when localization and navigation try
            # to load into the container back-to-back. Standalone nodes avoid
            # that shared-container race.
            'use_composition': 'False',
        }.items(),
    )

    nav2_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'rviz_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz_config': os.path.join(pkg_share, 'rviz', 'nav2.rviz'),
        }.items(),
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        use_sim_time_arg,
        gui_arg,
        rviz_arg,
        world_arg,
        map_arg,
        params_file_arg,
        slam_arg,
        gazebo,
        nav2_bringup,
        nav2_rviz,
    ])