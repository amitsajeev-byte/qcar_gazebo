import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg = get_package_share_directory('qcar_gazebo')
    xacro_file = os.path.join(pkg, 'urdf', 'qcar_model.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    install_share = os.path.realpath(os.path.join(pkg, '..'))
    model_path = ':'.join([
        os.path.join(pkg, 'models'),
        install_share,
        os.environ.get('GAZEBO_MODEL_PATH', '')
    ])

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose',
                 os.path.join(pkg, 'worlds', 'myworld.world'),
                 '-s', 'libgazebo_ros_factory.so',
                 '-s', 'libgazebo_ros_init.so'],
            additional_env={
                'GAZEBO_MODEL_DATABASE_URI': '',
                'GAZEBO_MODEL_PATH': model_path,
            },
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'source_list': ['/joint_states']
            }]
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'qcar'],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('/diff_drive_controller/odom', '/odom'),
                ('/diff_drive_controller/cmd_vel_unstamped', '/cmd_vel')
            ],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['steering_controller'],
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg, 'rviz', 'qcar.rviz')],
            parameters=[{'use_sim_time': True}]
        )
    ])