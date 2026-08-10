# qcar_initial

ROS 2 / Gazebo (classic) simulation, SLAM, and Nav2 navigation stack for the QCar
Ackermann-steered robot, driven directly by the `libgazebo_ros_ackermann_drive.so` Gazebo plugin.

## Prerequisites

- **ROS 2 Humble on Ubuntu 22.04 (Jammy)**, or another combination where Gazebo **classic**
  (`gazebo11`) is still packaged. This stack uses the classic `gazebo_ros`/`gazebo_plugins`
  plugins (`libgazebo_ros_factory.so`, `libgazebo_ros_camera.so`, `libgazebo_ros_ray_sensor.so`,
  `libgazebo_ros_ackermann_drive.so`) and the `gazebo`/`spawn_entity.py` executables — none of
  which exist for newer distros/OSes that ship only the new Gazebo (Harmonic/Ionic, `gz-sim`). In
  particular, **ROS 2 Jazzy on Ubuntu 24.04 (Noble) will not work**: `gazebo11` has no apt
  candidate there, and `rosdep install` will silently fail to resolve `gazebo_ros`/`gazebo_plugins`
  either, since those aren't released for Noble. Run this package in an Ubuntu 22.04 + ROS 2
  Humble environment (native, VM, or container) if your host is on a newer distro.
- `gazebo_ros`, `gazebo_plugins`
- `robot_state_publisher`, `joint_state_publisher`
- `xacro`
- `cartographer_ros` (for mapping)
- `nav2_bringup` (for navigation)
- `rviz2`

Install any missing packages with `rosdep`:

```bash
cd ~/ws/tmp/ros_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
cd ~/ws/tmp/ros_ws
colcon build --packages-select qcar_initial
source install/setup.bash
```

## Package layout

| Path | Purpose |
|---|---|
| `urdf/qcar_model.xacro` | Robot description: chassis, Ackermann hubs/wheels, lidar, cameras, and the `gazebo_ros_ackermann_drive` plugin block (drive kinematics, PID gains, odometry) |
| `launch/qcar_initial.launch.py` | Core sim bring-up: Gazebo, robot spawn, `robot_state_publisher`, `joint_state_publisher`, RViz |
| `launch/qcar_slam.launch.py` | Mapping: Cartographer SLAM (includes the sim by default) |
| `launch/qcar_nav2.launch.py` | Navigation: Nav2 stack against a saved map (includes the sim by default) |
| `config/cartographer/qcar_2d.lua` | Cartographer SLAM parameters |
| `config/nav2/nav2_params.yaml` | Nav2 stack parameters (AMCL, costmaps, planner, controller) - see `config/nav2/PARAMS_REFERENCE.md` for a full per-parameter explanation, or `TUNING.md` for the accuracy-tuning subset |
| `config/nav2/behavior_trees/*.xml` | Custom copies of nav2's stock BT trees, wired in via `RewrittenYaml` in `qcar_nav2.launch.py` - replan only on an invalid/updated path (not unconditionally every second) and no `<Spin>` recovery, since this Ackermann car can't rotate in place |
| `src/critics/early_commit_critic.cpp`, `include/qcar_initial/critics/early_commit_critic.hpp` | Custom MPPI critic plugin (`EarlyCommitCritic`) built into this package - see "Custom MPPI critic" below |
| `maps/qcar_map.yaml` / `.pgm` | Saved occupancy grid map used by `qcar_nav2.launch.py` |
| `scripts/qcar_teleop_twist.py` | Keyboard teleop publishing `/cmd_vel` |
| `worlds/*.world` | Gazebo worlds (`myworld.world` is used by default; others available for manual swap) |
| `TUNING.md` | Navigation accuracy tuning reference (physics, MPPI controller, AMCL, costmap parameters) |

## Quick start: simulation only

```bash
ros2 launch qcar_initial qcar_initial.launch.py
```

Brings up Gazebo (`myworld.world`), spawns the robot, and starts `robot_state_publisher`,
`joint_state_publisher`, and RViz. Nothing drives the robot yet — use the teleop script below, or
one of the mapping/navigation launch files, which start their own driving pipeline.

### How the drive plugin is wired into the rest of the stack

The `gazebo_ros_ackermann_drive` plugin (declared directly in the `<gazebo>` block of
`urdf/qcar_model.xacro`) subscribes to `/cmd_vel` (`geometry_msgs/Twist`) and publishes
`/odom` (`nav_msgs/Odometry`) and the `odom -> base` TF transform directly — no topic relays or
`ros2_control` controller manager are needed. It also broadcasts wheel/steering-hub TF
(`publish_wheel_tf: true`) so RViz can visualize the wheels turning.

## Mapping (SLAM)

1. Launch SLAM (this also brings up the simulation):

   ```bash
   ros2 launch qcar_initial qcar_slam.launch.py
   ```

   This starts Gazebo + the robot (unless disabled, see [Launch arguments](#launch-arguments))
   with `disable_odom_tf:=true`, and the two Cartographer nodes (`cartographer_node` publishing
   the `map -> odom -> base` TF, and `cartographer_occupancy_grid_node` publishing `/map`).

2. Drive the robot around the environment to build up the map. In a second terminal:

   ```bash
   ros2 run qcar_initial qcar_teleop_twist.py
   ```

   Keys: `w`/`s` forward/backward, `a`/`d` steer left/right, `z` centre steering, `x` stop and
   centre, `q`/`e` increase/decrease speed.

3. Watch progress in RViz (`Fixed Frame: map`, `LaserScan` and map display already configured in
   `rviz/qcar.rviz`).

4. Once the map looks complete, save it with `nav2_map_server`'s saver (subscribes to `/map`):

   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/ws/tmp/ros_ws/src/qcar_initial/maps/qcar_map
   ```

   This overwrites `maps/qcar_map.pgm` and `maps/qcar_map.yaml`, which is what
   `qcar_nav2.launch.py` loads by default.

**Note:** `disable_odom_tf:=true` is *intended* to make `qcar_initial.launch.py` suppress the
drive plugin's own `odom -> base` broadcast during SLAM, since Cartographer already provides it
(`provide_odom_frame: true` in `config/cartographer/qcar_2d.lua`) and the two would otherwise
fight over the same TF edge. **This is currently a known-broken no-op** — the launch argument is
declared and passed through to `xacro`, but `urdf/qcar_model.xacro` never reads it, and the
plugin's `publish_odom_tf` is hardcoded `true`. See Troubleshooting below.

## Navigation (Nav2 + AMCL)

1. Make sure `maps/qcar_map.yaml` is the map you want to navigate in (see Mapping above to
   (re)generate it).

2. Launch navigation (this also brings up the simulation):

   ```bash
   ros2 launch qcar_initial qcar_nav2.launch.py
   ```

   This starts Gazebo + the robot (the drive plugin publishes `odom -> base` directly, since
   nothing else does in this mode) and the full Nav2 `bringup_launch.py` (map server, AMCL,
   planner, controller, behavior server, BT navigator) configured with
   `config/nav2/nav2_params.yaml`.

3. In RViz, set the robot's starting pose with **2D Pose Estimate** if it doesn't match the
   AMCL default (`x=0, y=0, yaw=0`, configured in `nav2_params.yaml`), and let AMCL converge for
   a few seconds (drive a little to help it disambiguate) before trusting the localization.

4. Send a goal with **2D Nav Goal** in RViz, or via the `navigate_to_pose` action, and the robot
   will plan (via `nav2_smac_planner::SmacPlannerHybrid`, Reeds-Shepp motion model) and drive there
   using `nav2_mppi_controller::MPPIController` (`FollowPath`, see Custom MPPI critic below).

You can also drive manually at any time with `ros2 run qcar_initial qcar_teleop_twist.py`
(publishes to `/cmd_vel`, same as Nav2's controller output).

### Custom MPPI critic: `EarlyCommitCritic`

`FollowPath.critics` includes a custom critic plugin built into this package
(`src/critics/early_commit_critic.cpp`, exported via `qcar_critics.xml`/`package.xml`), on top of
the stock MPPI critics (`ConstraintCritic`, `CostCritic`, `GoalCritic`, `GoalAngleCritic`,
`PathAlignCritic`, `PathFollowCritic`, `PathAngleCritic`). It scores only the first
`early_time_steps` of each sampled trajectory against the bearing to a near-term path point, with
no distance/angle gating - unlike the stock path critics, it always pushes MPPI to start turning
toward the path immediately rather than driving straight into a curve before reacting to it. Gated
by `active_path_points` so it stays out of the way once the robot has made real progress along the
route. See `CHANGELOG.md` 2026-07-18 (9) for why it was needed and 2026-07-19 (1) for a later
direction-awareness fix (reversing/K-turn segments).

Building this plugin requires `xtensor`/`xsimd` compile definitions to match
`nav2_mppi_controller`'s own build exactly (see the comment block at the top of `CMakeLists.txt`) -
a mismatch here is a silent ABI mismatch that crashes `nav2_container` on the first control cycle
that touches vectorized critic math, not a compile error.

## Launch arguments

Both `qcar_slam.launch.py` and `qcar_nav2.launch.py` accept:

| Argument | Default | Description |
|---|---|---|
| `launch_sim` | `true` | Whether to include `qcar_initial.launch.py` (Gazebo + robot). Set to `false` if the simulation is already running (e.g. started separately, or you're driving real hardware) to avoid spawning a second instance. |

`qcar_initial.launch.py` itself accepts:

| Argument | Default | Description |
|---|---|---|
| `disable_odom_tf` | `false` | Intended to suppress the drive plugin's own `odom -> base` TF broadcast (for SLAM, where Cartographer should be the sole broadcaster). **Currently a no-op** — see the note in Mapping above and Troubleshooting below. |

Example:

```bash
ros2 launch qcar_initial qcar_slam.launch.py launch_sim:=false
```

## Key robot parameters

These are derived from the STL meshes and joint origins in `urdf/qcar_model.xacro`, and (except
where noted) auto-computed by the drive plugin from that geometry at spawn time rather than
stated in any config file:

| Parameter | Value | Source |
|---|---|---|
| Wheel radius | 0.033 m (mesh), 0.036 m (collision cylinder) | `models/qcar/QCarWheel.stl` bounding box (measured directly); collision geometry is padded +3mm for reliable ground contact — see `TUNING.md` |
| Wheelbase | 0.25725 m | Sum of front/rear hub joint x-offsets (0.12960 + 0.12765) |
| Front wheel track | 0.1118 m | `base_hubfl_joint`/`base_hubfr_joint` y-offsets (2 x 0.05590) |
| Rear wheel track | 0.1122 m | `base_wheelrl_joint`/`base_wheelrr_joint` y-offsets (2 x 0.05610) |
| Max steering angle | ±0.5236 rad (30°) | `base_hubfl_joint` / `base_hubfr_joint` limits, and the plugin's `max_steer` |
| Min turning radius | ~0.45 m | Derived: wheelbase / tan(max steering angle) |
| Lidar range | 0.15 - 12.0 m | Lidar sensor plugin, matches `nav2_params.yaml` |

The `gazebo_ros_ackermann_drive` plugin auto-computes wheelbase/track/wheel-radius from the URDF
joint poses and collision geometry at spawn time — these aren't separately configured anywhere,
unlike the old `ackermann_steering_controller` setup which needed them stated explicitly in a
YAML config.

Nav2 speed limits (`config/nav2/nav2_params.yaml`): max linear velocity 0.3 m/s
(`velocity_smoother`, `FollowPath.desired_linear_vel`), max angular velocity 1.0 rad/s.

## Topics and frames reference

| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Driving input; subscribed to directly by the drive plugin |
| `/odom` | `nav_msgs/Odometry` | Wheel-based odometry published directly by the drive plugin (real dead reckoning, not ground truth) |
| `/scan` | `sensor_msgs/LaserScan` | From the simulated lidar |
| `/map` | `nav_msgs/OccupancyGrid` | From Cartographer (mapping) or the map server (navigation) |
| `/joint_states` | `sensor_msgs/JointState` | Published by `joint_state_publisher`; **not** driven by the plugin's actual wheel motion (see Troubleshooting) — wheel/steering-hub TF instead comes directly from the plugin's own `publish_wheel_tf` |

TF tree: `map -> odom -> base -> {lidar, hubfl, hubfr, wheelfl, wheelfr, wheelrl, wheelrr,
camera_*}`. During SLAM, Cartographer *should* own both `map -> odom` and `odom -> base` (see the
`disable_odom_tf` caveat above — currently both Cartographer and the drive plugin publish
`odom -> base`). During navigation, AMCL owns `map -> odom` and the drive plugin owns
`odom -> base`.

## Troubleshooting

- **Robot doesn't move**: check the Gazebo terminal for `[gazebo_ros_ackermann_drive]: Subscribed
  to [/cmd_vel]` at startup — if that line is missing, the plugin failed to load (check for URDF
  parse errors just above it). If it's present but the car still doesn't move, check wheel
  collision geometry, friction, and ground-plane friction in `urdf/qcar_model.xacro` /
  `worlds/myworld.world` — see `TUNING.md` and the 2026-07-13 entry in `CHANGELOG.md` for the bug
  pattern this plugin is prone to (mesh-shaped wheel collisions read as zero radius, zero
  friction anywhere, razor-thin wheel/ground clearance).
- **Robot moves but barely turns / doesn't reach Nav2 goals accurately**: check the steering PID
  gain (`left/right_steering_pid_gain` in the plugin block) — too weak relative to wheel/ground
  friction silently produces a much larger real turning radius than commanded. See `TUNING.md`
  for the full accuracy-tuning parameter reference.
- **No `/scan` data**: check `gazebo_ros_ray_sensor` plugin output in the Gazebo terminal for
  plugin load errors.
- **No `/odom` or `/tf` (`odom -> base`) data**: check the Gazebo terminal for `[gazebo_ros_ackermann_drive]: Advertise odometry on [/odom]` / `Publishing odom transforms between [odom] and [base]` at startup — these confirm the plugin itself loaded successfully. If they're missing, check for a URDF parse failure earlier in the log.
- **`/joint_states` shows all-zero positions while the robot is visibly driving**: expected with
  the current launch wiring — `joint_state_publisher`'s `source_list` is configured to
  `['/joint_states']` (self-referential), so it just republishes its own default zero state rather
  than reflecting real wheel motion. This doesn't affect physics or navigation; the plugin
  broadcasts the real wheel/steering TF separately via `publish_wheel_tf`. Only relevant if you're
  trying to read wheel angles from `/joint_states` directly (e.g. for a custom debug tool) — use
  `tf2_echo base <wheel_or_hub_link>` instead.
- **AMCL doesn't localize / lidar points seem to jump or drift while driving**: first verify the
  initial pose (2D Pose Estimate) matches the robot's actual position in the map, and give AMCL a
  few seconds and a bit of driving to converge (check `/amcl_pose`'s covariance shrinking).
  Wheel odometry has real dead-reckoning drift (unlike the old ground-truth-based `/odom`) — some
  AMCL correction on every scan match is expected, not a bug.
- **Weird/inconsistent behavior across repeated test launches (duplicate TF, jittery joints)**:
  make sure no processes from a previous run are still alive before relaunching -
  `ps aux | grep -E "gazebo|gzserver|gzclient|rviz2|robot_state_publisher|joint_state_publisher"`
  should show nothing before a fresh `ros2 launch`. A `joint_state_publisher` (or any other node)
  left running from an earlier, not-fully-killed launch will silently double-publish and corrupt
  the TF tree.
- **`ros2 launch` crashes immediately with `NameError: name 'false' is not defined`**: this means
  a launch argument named `slam` somewhere is colliding with `nav2_bringup`'s own `slam`
  argument (see the 2026-07-05 entry in `CHANGELOG.md`) — don't reintroduce a top-level argument
  literally named `slam` in these launch files.
- **gzserver crashes a few seconds into driving (`Ogre::AxisAlignedBox::setExtents` assertion)**:
  numerical instability from wheel rotational inertia being too small relative to the plugin's PID
  gains. See `TUNING.md` (wheel inertia / steering PID gain) and the 2026-07-13 entry in
  `CHANGELOG.md`.

See `CHANGELOG.md` for a history of fixes applied to this package, and `TUNING.md` for a
navigation-accuracy parameter reference.

## Relation to `qcar_hardware`

This package is simulation-only (Gazebo classic). The sibling `qcar_hardware` package
(`../qcar_hardware/`) covers bring-up on the physical QCar 2 - Quanser HAL/PAL reference material,
hardware-test scripts, and `hardware_integration_reference.md` documenting the actual sensor/motor
API. The physical QCar's onboard compute runs ROS 2 Dashing on Ubuntu 18.04, not Humble, so this
package's Nav2/MPPI stack (including the custom critic above) cannot run natively on it and keeps
running here, on the dev PC. `qcar_hardware` instead adds a small native bridge on the QCar itself
(`qcar_hw_bridge.py` / `qcar_lidar_bridge.py`) that subscribes to `/cmd_vel` from this stack over
the network and publishes real `/odom`/`/imu`/`/scan` back - a Humble Docker container on the QCar
was the original plan but was dropped for storage reasons (32GB eMMC, already shared with the
onboard Dashing/Melodic installs). Not yet verified end-to-end on hardware; see `qcar_hardware`'s
reference doc for the architecture, the open Dashing↔Humble wire-compatibility risk, and what still
needs calibrating (throttle-to-speed gain in particular).
