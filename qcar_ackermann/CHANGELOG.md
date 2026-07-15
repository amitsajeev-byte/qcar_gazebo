# Changelog

## 2026-07-13 - Migrate from `ackermann_steering_controller` to `libgazebo_ros_ackermann_drive`, fix "not moving" / "not reaching goal"

Replaced the `ros2_control`-based `ackermann_steering_controller` (and its `controller_manager`
spawners and `topic_tools relay` bridges) with the `gazebo_ros_ackermann_drive` Gazebo plugin,
declared directly in `urdf/qcar_model.xacro`'s `<gazebo>` block - same approach already used by
the sibling package `qcar_navigation`. The plugin subscribes to `/cmd_vel` and publishes `/odom`
and the `odom -> base` TF natively, so no relay nodes or controller manager are needed.

### Changed
- **`urdf/qcar_model.xacro`**: removed the `<ros2_control>` block and its `libgazebo_ros2_control`
  plugin; added the `gazebo_ros_ackermann_drive` plugin block (front/rear/steering joint mapping,
  `max_steer`/`max_speed`, PID gains, odometry/TF publishing flags). Replaced each wheel's
  `<collision>` (previously the visual STL mesh) with a `<cylinder radius="0.036"
  length="0.0245"/>` - the plugin's wheel-radius auto-detection only supports cylinder/sphere
  collisions, and a mesh collision silently reads `wheel_radius_=0`, poisoning the velocity PID
  entirely (this is why the robot didn't move at all after the initial migration). Added
  `<mu1>1.0</mu1><mu2>1.0</mu2>` friction to all 4 wheel links (previously zero friction
  anywhere, causing 100% wheel slip with zero real motion even once the radius bug was fixed).
  Bumped wheel rotational inertia from `ixx=izz=0.0000645, iyy=0.0001089` to
  `ixx=izz=0.0012, iyy=0.0022` - the original tiny inertia caused an intermittent
  `Ogre::AxisAlignedBox::setExtents` assertion crash in gzserver under normal steering PID load.
- **`worlds/myworld.world`**: fixed the ground plane's `<friction><ode><mu>100</mu><mu2>50</mu2>`
  down to `mu=1 mu2=1` - the original absurd value, combined with a now-working wheel contact,
  caused a physics blow-up.
- **`left/right_steering_pid_gain`**: initially set to `0.02 0 0.002` (copied from
  `qcar_navigation`'s equivalent fix), but that value was tuned for a robot with zero wheel
  friction. Once wheel friction was added (above), turning the front hubs against a gripping,
  rolling tire needed real torque that `kp=0.02` couldn't supply - measured via direct `tf2`
  lookups on `base -> hubfl`/`hubfr` that the hub only reached ~1 deg of an ~15 deg commanded
  steering angle. This silently made the car's real turning radius ~6x larger than commanded,
  causing Nav2's path-tracking controller to be unable to follow any planned turn accurately.
  Raised to `3.0 0 0.1`, re-verified to converge to the correct Ackermann inner/outer wheel angle
  split for a given commanded turning radius.
- **`launch/qcar_ackermann.launch.py`**: removed the `joint_state_broadcaster`/
  `ackermann_steering_controller` spawner nodes and the `odom_relay`/`odom_tf_relay`
  `topic_tools relay` nodes (no longer needed - the plugin publishes `/odom` and `odom -> base`
  directly). `robot_description` is now built via a `Command(['xacro ', ...])` substitution
  instead of `xacro.process_file(...).toxml()` at launch-description-generation time.
- **`launch/qcar_nav2.launch.py`** / **`launch/qcar_slam.launch.py`**: removed the `cmd_vel_relay`
  `topic_tools relay` node (the plugin subscribes to `/cmd_vel` directly, no bridging needed).
- **`package.xml`**: removed `controller_manager`, `joint_state_broadcaster`,
  `ackermann_steering_controller`, `topic_tools`, `gazebo_ros2_control`; added `gazebo_plugins`.

### Removed
- `config/qcar_controllers.yaml`, `config/qcar_controllers_slam_override.yaml` - `ros2_control`
  controller manager configs, no longer used.

### Known limitations
- **`disable_odom_tf` launch argument is currently a no-op.** It's still declared in
  `qcar_ackermann.launch.py` and passed through to `xacro`, and `qcar_slam.launch.py` still passes
  `disable_odom_tf:=true` expecting it to suppress the drive plugin's own `odom -> base`
  broadcast during SLAM (so Cartographer is the sole broadcaster). But
  `urdf/qcar_model.xacro` never reads a `disable_odom_tf` property anywhere, and the plugin's
  `publish_odom_tf` is hardcoded `true` - so in SLAM mode both Cartographer and the plugin now
  publish `odom -> base`, fighting over the same TF edge. Needs an `xacro:arg`-gated
  `publish_odom_tf` value if SLAM mode is exercised again.
- **`/joint_states` is self-referential and always reports zero.** `joint_state_publisher` in
  `qcar_ackermann.launch.py` is configured with `source_list: ['/joint_states']` - its own output
  topic - so it just republishes its own default zero state rather than real wheel angles. Doesn't
  affect physics or Nav2 (the plugin publishes real wheel/steering TF separately via
  `publish_wheel_tf`), but any tooling that reads `/joint_states` directly for wheel angles will
  see stale zeros; use TF (`base -> wheelXX`/`hubXX`) instead.
- End-to-end verified via a real Nav2 run (headless gzserver + AMCL + planner + controller,
  goal requiring both straight driving and a turn) - `bt_navigator` logged `Reached the goal!` /
  `Goal succeeded`. See `TUNING.md` for further accuracy-tuning parameters if goal-reaching
  precision still isn't satisfactory for a given map/goal.

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
- **`launch/qcar_ackermann.launch.py`**: replaced the `drive_controller`/`steering_controller`
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
  `qcar_ackermann.launch.py` with `disable_odom_tf:=true`, since Cartographer
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
  `qcar_nav2.launch.py` includes `qcar_ackermann.launch.py` before including nav2's bringup, the
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
