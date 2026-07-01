# Changelog

## 2026-07-01 - Navigation/mapping bug fixes

### Fixed
- **Controller namespace mismatch**: removed `<robotNamespace>/qcar</robotNamespace>` from the
  `gazebo_ros2_control` plugin in `urdf/qcar_model.xacro`. It put `controller_manager` under
  `/qcar/...` while the spawner nodes and `cmd_vel_to_drive.py`/`qcar_teleop.py` addressed the
  unnamespaced topics/services, so controllers never activated and drive commands went nowhere.
- **Hard dependency on unused `realsense2_description`**: removed the unconditional
  `xacro:include` of `_d435.urdf.xacro` and its commented-out `sensor_d435` usage from
  `urdf/qcar_model.xacro`; the D435 macro was never actually instantiated.
- **Missing package dependencies**: `package.xml`/`CMakeLists.txt` only declared `rclcpp`
  (unused), `xacro`, `robot_state_publisher`. Added the actually-used runtime deps: `rclpy`,
  `geometry_msgs`, `std_msgs`, `nav_msgs`, `tf2_ros`, `joint_state_publisher`,
  `controller_manager`, `gazebo_ros`, `gazebo_ros2_control`, `nav2_bringup`, `cartographer_ros`,
  `rviz2`. Dropped the unused `rclcpp` dependency.
- **Unrealistic link inertials**: every link (chassis, hubs, wheels, lidar) shared the same
  placeholder `mass="0.1"` / `inertia diag(0.001)`. Replaced with values computed from each
  mesh's actual bounding box (chassis 2.4 kg, hubs 0.05 kg, wheels 0.2 kg, lidar 0.15 kg, with
  proper box/cylinder inertia tensors), so Gazebo dynamics are physically plausible.
- **Wrong wheel-speed gain** in `scripts/cmd_vel_to_drive.py`: `linear.x * 20.0` didn't match the
  measured wheel radius (~0.033 m from `QCarWheel.stl`); the robot ran ~35% slower than
  commanded. Fixed to `linear.x / WHEEL_RADIUS`.
- **Non-Ackermann steering conversion** in `scripts/cmd_vel_to_drive.py`: replaced the fixed-gain
  `angular.z * 0.2` with a proper bicycle-model inversion,
  `atan2(WHEELBASE * angular.z, linear.x)`, using the actual wheelbase (0.25725 m).
- **Inconsistent `Twist.angular.z` semantics** in `scripts/qcar_teleop_twist.py`: it was
  publishing a literal steering angle into a field that's supposed to carry yaw rate. Now
  derives yaw rate from the desired steering angle via the same bicycle model so it round-trips
  correctly through `cmd_vel_to_drive.py`.
- **No noise on simulated odometry**: `gazebo_ros_p3d`'s `gaussian_noise` was `0.0`, making
  `/odom` perfect ground truth and defeating the purpose of AMCL's odometry correction. Set to
  `0.01`.
- **`qcar_teleop_twist.py` not installed**: added it to `install(PROGRAMS ...)` in
  `CMakeLists.txt` alongside the other scripts.
- **No combined bring-up**: `qcar_slam.launch.py` and `qcar_nav2.launch.py` each now include
  `qcar_updated.launch.py` (gated by a `launch_sim` launch argument) so mapping/navigation can be
  started with a single `ros2 launch` instead of manually launching multiple files. Added
  `cmd_vel_to_drive.py` to `qcar_slam.launch.py` so the robot can be driven while mapping;
  deliberately did **not** add `odom_to_tf.py` there since Cartographer already publishes the
  `odom->base` transform itself and a second broadcaster would conflict.

### Removed
- Dead ROS1-style `<transmission>` blocks in `urdf/qcar_model.xacro` (ignored by
  `gazebo_ros2_control`, which only reads the `<ros2_control>` block).
- Vestigial commented-out blocks: `ackermann_drive` Gazebo plugin, per-wheel friction
  parameters, and the `sensor_d435` macro invocation.
