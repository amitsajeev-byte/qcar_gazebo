# Changelog

## 2026-07-05 - Migrate from JointGroup controllers to `ackermann_steering_controller`

Replaced the hand-rolled drive stack (two independent `ros2_control` controllers plus a
manual bicycle-model conversion script) with `ackermann_steering_controller`, which does
Ackermann inverse kinematics and wheel odometry natively.

### Changed
- **Controller manager config** (`config/qcar_controllers.yaml`): removed `drive_controller`
  (`velocity_controllers/JointGroupVelocityController`) and `steering_controller`
  (`position_controllers/JointGroupPositionController`); added a single
  `ackermann_steering_controller` (`ackermann_steering_controller/AckermannSteeringController`)
  configured with `front_steering: true`, the measured wheelbase/track/radius values, and
  `enable_odom_tf: true` so it publishes its own `odom -> base` transform.
- **`launch/qcar_updated.launch.py`**: replaced the `drive_controller`/`steering_controller`
  spawner nodes with one `ackermann_steering_controller` spawner; added a `disable_odom_tf`
  launch argument (default `false`) that, when true, spawns the controller with an extra
  `-p config/qcar_controllers_slam_override.yaml` so it doesn't broadcast `odom -> base` (used
  during SLAM, see below); added two `topic_tools relay` nodes to bridge the controller's private
  topics onto the topics the rest of the stack expects: `~/odometry -> /odom` and
  `~/tf_odometry -> /tf` (this controller version publishes its TF to a private topic instead of
  directly to `/tf`); added `--ros-args --log-level ackermann_steering_controller:=error` to the
  `gazebo` process to silence a benign per-message deprecation warning from the unstamped
  `Twist` reference topic (the node exposes no runtime logger service in Humble, so this has to
  be set at process launch).
- **`launch/qcar_nav2.launch.py`** / **`launch/qcar_slam.launch.py`**: replaced the
  `cmd_vel_to_drive.py` node with a `topic_tools relay` from `/cmd_vel` to
  `/ackermann_steering_controller/reference_unstamped`. `qcar_slam.launch.py` now includes
  `qcar_updated.launch.py` with `disable_odom_tf:=true`, since Cartographer
  (`provide_odom_frame: true` in `qcar_2d.lua`) must be the sole `odom -> base` broadcaster.
- **`urdf/qcar_model.xacro`**: removed the `gazebo_ros_p3d` ground-truth odometry plugin
  (`/odom` is now the controller's real wheel odometry, not a perfect ground-truth feed). Fixed
  a latent bug on `base_wheelrl_joint`: its `<origin>` is yawed 180 deg for mesh mirroring but it
  shared the same `<axis>` as `base_wheelrl_joint`'s counterpart, silently inverting its
  effective rotation sense; the old `JointGroupVelocityController` setup masked this by
  negating one side's command in `cmd_vel_to_drive.py`, but a real per-joint controller needs it
  fixed at the source, so the axis was flipped to `0 -1 0` instead.
- **`package.xml`**: removed `velocity_controllers`, `position_controllers`, `std_msgs`,
  `nav_msgs`, `tf2_ros` (nothing left depends on them); added `ackermann_steering_controller`
  and `topic_tools`.
- **`CMakeLists.txt`**: removed the deleted scripts from `install(PROGRAMS ...)`.

### Removed
- `scripts/cmd_vel_to_drive.py` - superseded by the controller's native `Twist` reference input
  (via the `cmd_vel_relay` topic relay).
- `scripts/odom_to_tf.py` - superseded by the controller's own `odom -> base` TF broadcast (via
  the `odom_tf_relay` topic relay).
- `scripts/qcar_teleop.py` - published directly to `/drive_controller/commands` and
  `/steering_controller/commands`, both retired by this migration. `scripts/qcar_teleop_twist.py`
  (publishes plain `Twist` to `/cmd_vel`) already supersedes it and needs no changes.

### Fixed
- **Launch argument name collision**: the first attempt at the `disable_odom_tf` argument above
  was named `slam`, which collides with `nav2_bringup`'s own `slam` launch argument (used
  internally by `bringup_launch.py` for its own localization-vs-SLAM branching). Because
  `qcar_nav2.launch.py` includes `qcar_updated.launch.py` before including nav2's bringup, the
  lowercase `default_value='false'` won the shared launch-context lookup, and nav2's
  `PythonExpression(['not ', slam])` then evaluated `eval("not false")` - a `NameError`, since
  Python needs the capitalized `False`. This crashed the whole launch and took the nav2
  component container down with it (`exit code -6`). Renamed the argument to `disable_odom_tf`.

### Known limitations
- Wheel odometry now has real (if a bit high) dead-reckoning drift during turns, most likely
  from unmodeled wheel/ground slip - `urdf/qcar_model.xacro` has no `<gazebo><surface><friction>`
  tuning on the wheel collisions at all (bare ODE defaults). AMCL tolerates some odometry error
  by design (it corrects against the map via laser scan matching), but this may need attention
  if navigation accuracy is unsatisfactory. The controller's own odometry math was verified
  correct against the upstream `ros2_controllers` source for this version.
