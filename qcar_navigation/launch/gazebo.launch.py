import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('qcar_navigation')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'worlds', 'qcar_world.sdf'),
        description='Full path to the Gazebo world (SDF) to load')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock')

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch the Gazebo GUI client (gzclient)')

    xacro_file = os.path.join(pkg_share, 'urdf', 'qcar2.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    # Classic Gazebo's gazebo_ros package resolves "package://" URIs in the
    # URDF (meshes, etc.) automatically via its GazeboRosPaths plugin - unlike
    # gz-sim, there's no GZ_SIM_RESOURCE_PATH env var to set up here.

    # gzserver must be started with libgazebo_ros_factory.so (provides the
    # /spawn_entity, /delete_entity services used below) and
    # libgazebo_ros_init.so (provides simulation clock / control services).
    # gazebo_ros's own launch files already wire these in.
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': world,
            'use_sim_time': use_sim_time,
            'extra_gazebo_args': '--ros-args -r /tf:=tf -r /tf_static:=tf_static',
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(gui),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )

    # ros_gz_sim's "create" node -> gazebo_ros's spawn_entity.py. It takes
    # -topic (the robot_description topic) instead of -file, and -entity
    # instead of -name.
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'qcar2',
            '-z', '0.05',
        ],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        gui_arg,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_robot,
    ])