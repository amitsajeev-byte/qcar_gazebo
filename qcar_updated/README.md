# qcar_updated

ROS 2 / Gazebo (classic) simulation, SLAM, and Nav2 navigation stack for the QCar
Ackermann-steered robot, driven by `ackermann_steering_controller`.

## Prerequisites

- **ROS 2 Humble on Ubuntu 22.04 (Jammy)**, or another combination where Gazebo **classic**
  (`gazebo11`) is still packaged. This stack uses the classic `gazebo_ros`/`gazebo_ros2_control`
  plugins (`libgazebo_ros_factory.so`, `libgazebo_ros_camera.so`, `libgazebo_ros_ray_sensor.so`,
  `libgazebo_ros2_control.so`) and the `gazebo`/`spawn_entity.py` executables — none of which
  exist for newer distros/OSes that ship only the new Gazebo (Harmonic/Ionic, `gz-sim`). In
  particular, **ROS 2 Jazzy on Ubuntu 24.04 (Noble) will not work**: `gazebo11` has no apt
  candidate there, and `rosdep install` will fail to resolve `gazebo_ros2_control` (and silently
  can't resolve `gazebo_ros`/`gazebo_plugins` either, since those aren't released for Noble). Run
  this package in an Ubuntu 22.04 + ROS 2 Humble environment (native, VM, or container) if your
  host is on a newer distro.
- `gazebo_ros`, `gazebo_ros2_control`
- `robot_state_publisher`, `joint_state_publisher`, `controller_manager`, `joint_state_broadcaster`
- `ackermann_steering_controller` (`ros-humble-ackermann-steering-controller`)
- `topic_tools`
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
colcon build --packages-select qcar_updated
source install/setup.bash
```

## Package layout

| Path | Purpose |
|---|---|
| `urdf/qcar_model.xacro` | Robot description: chassis, Ackermann hubs/wheels, lidar, cameras, `ros2_control` interfaces |
| `launch/qcar_updated.launch.py` | Core sim bring-up: Gazebo, robot spawn, `robot_state_publisher`, controller, topic relays, RViz |
| `launch/qcar_slam.launch.py` | Mapping: Cartographer SLAM (includes the sim by default) |
| `launch/qcar_nav2.launch.py` | Navigation: Nav2 stack against a saved map (includes the sim by default) |
| `config/qcar_controllers.yaml` | `ros2_control` controller manager config (`ackermann_steering_controller`) |
| `config/qcar_controllers_slam_override.yaml` | Param override loaded on top of the above during SLAM, to disable the controller's own `odom -> base` TF broadcast |
| `config/cartographer/qcar_2d.lua` | Cartographer SLAM parameters |
| `config/nav2/nav2_params.yaml` | Nav2 stack parameters (AMCL, costmaps, planner, controller) |
| `maps/qcar_map.yaml` / `.pgm` | Saved occupancy grid map used by `qcar_nav2.launch.py` |
| `scripts/qcar_teleop_twist.py` | Keyboard teleop publishing `/cmd_vel` |
| `worlds/*.world` | Gazebo worlds (`myworld.world` is used by default; others available for manual swap) |

## Quick start: simulation only

```bash
ros2 launch qcar_updated qcar_updated.launch.py
```

Brings up Gazebo (`myworld.world`), spawns the robot, starts `robot_state_publisher`,
`joint_state_publisher`, `joint_state_broadcaster`/`ackermann_steering_controller`, the topic
relays described below, and RViz. Nothing drives the robot yet — use the teleop script below, or
one of the mapping/navigation launch files, which start their own driving pipeline.

### How the controller is wired into the rest of the stack

`ackermann_steering_controller` takes a `geometry_msgs/Twist` reference and publishes its own
odometry, but on private topics under its own name, and the rest of this stack (Nav2, AMCL,
Cartographer, RViz) expects the conventional `/cmd_vel`, `/odom`, and `/tf`. `qcar_updated.launch.py`
bridges this with three always-running `topic_tools relay` nodes/launch wiring:

| Relay | From | To |
|---|---|---|
| `odom_relay` | `/ackermann_steering_controller/odometry` | `/odom` |
| `odom_tf_relay` | `/ackermann_steering_controller/tf_odometry` | `/tf` |
| `cmd_vel_relay` (in `qcar_nav2.launch.py` / `qcar_slam.launch.py`) | `/cmd_vel` | `/ackermann_steering_controller/reference_unstamped` |

`odom_tf_relay` is skipped when `disable_odom_tf:=true` (used during SLAM — see below).

## Mapping (SLAM)

1. Launch SLAM (this also brings up the simulation):

   ```bash
   ros2 launch qcar_updated qcar_slam.launch.py
   ```

   This starts Gazebo + the robot (unless disabled, see [Launch arguments](#launch-arguments))
   with `disable_odom_tf:=true`, the `cmd_vel_relay`, and the two Cartographer nodes
   (`cartographer_node` publishing the `map -> odom -> base` TF, and
   `cartographer_occupancy_grid_node` publishing `/map`).

2. Drive the robot around the environment to build up the map. In a second terminal:

   ```bash
   ros2 run qcar_updated qcar_teleop_twist.py
   ```

   Keys: `w`/`s` forward/backward, `a`/`d` steer left/right, `z` centre steering, `x` stop and
   centre, `q`/`e` increase/decrease speed.

3. Watch progress in RViz (`Fixed Frame: map`, `LaserScan` and map display already configured in
   `rviz/qcar.rviz`).

4. Once the map looks complete, save it with `nav2_map_server`'s saver (subscribes to `/map`):

   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/ws/tmp/ros_ws/src/qcar_updated/maps/qcar_map
   ```

   This overwrites `maps/qcar_map.pgm` and `maps/qcar_map.yaml`, which is what
   `qcar_nav2.launch.py` loads by default.

**Note:** `disable_odom_tf:=true` is passed to `qcar_updated.launch.py` during SLAM so the
controller does **not** broadcast `odom -> base` itself — Cartographer already does
(`provide_odom_frame: true` in `config/cartographer/qcar_2d.lua`); running both would fight over
the same TF edge.

## Navigation (Nav2 + AMCL)

1. Make sure `maps/qcar_map.yaml` is the map you want to navigate in (see Mapping above to
   (re)generate it).

2. Launch navigation (this also brings up the simulation):

   ```bash
   ros2 launch qcar_updated qcar_nav2.launch.py
   ```

   This starts Gazebo + the robot (with `odom_tf_relay` active, since nothing else publishes
   `odom -> base` in this mode), the `cmd_vel_relay`, and the full Nav2 `bringup_launch.py` (map
   server, AMCL, planner, controller, behavior server, BT navigator) configured with
   `config/nav2/nav2_params.yaml`.

3. In RViz, set the robot's starting pose with **2D Pose Estimate** if it doesn't match the
   AMCL default (`x=0, y=0, yaw=0`, configured in `nav2_params.yaml`), and let AMCL converge for
   a few seconds (drive a little to help it disambiguate) before trusting the localization.

4. Send a goal with **2D Nav Goal** in RViz, or via the `navigate_to_pose` action, and the robot
   will plan and drive there using `nav2_regulated_pure_pursuit_controller`.

You can also drive manually at any time with `ros2 run qcar_updated qcar_teleop_twist.py`
(publishes to `/cmd_vel`, same as Nav2's controller output).

## Launch arguments

Both `qcar_slam.launch.py` and `qcar_nav2.launch.py` accept:

| Argument | Default | Description |
|---|---|---|
| `launch_sim` | `true` | Whether to include `qcar_updated.launch.py` (Gazebo + robot + controller). Set to `false` if the simulation is already running (e.g. started separately, or you're driving real hardware) to avoid spawning a second instance. |

`qcar_updated.launch.py` itself accepts:

| Argument | Default | Description |
|---|---|---|
| `disable_odom_tf` | `false` | Spawns the controller with `config/qcar_controllers_slam_override.yaml` applied, disabling its `odom -> base` TF broadcast. Set automatically to `true` by `qcar_slam.launch.py`; leave `false` for `qcar_nav2.launch.py` and standalone sim use. |

Example:

```bash
ros2 launch qcar_updated qcar_slam.launch.py launch_sim:=false
```

## Key robot parameters

These are configured in `config/qcar_controllers.yaml` for `ackermann_steering_controller`, and
were derived from the STL meshes and joint origins in `urdf/qcar_model.xacro`:

| Parameter | Value | Source |
|---|---|---|
| Wheel radius (`front_wheels_radius` / `rear_wheels_radius`) | 0.033 m | `models/qcar/QCarWheel.stl` bounding box (measured directly) |
| Wheelbase | 0.25725 m | Sum of front/rear hub joint x-offsets (0.12960 + 0.12765) |
| Front wheel track | 0.1118 m | `base_hubfl_joint`/`base_hubfr_joint` y-offsets (2 x 0.05590) |
| Rear wheel track | 0.1122 m | `base_wheelrl_joint`/`base_wheelrr_joint` y-offsets (2 x 0.05610) |
| Max steering angle | ±0.5236 rad (30°) | `base_hubfl_joint` / `base_hubfr_joint` limits |
| Lidar range | 0.15 - 12.0 m | Lidar sensor plugin, matches `nav2_params.yaml` |

Nav2 speed limits (`config/nav2/nav2_params.yaml`): max linear velocity 0.3 m/s
(`velocity_smoother`, `FollowPath.desired_linear_vel`), max angular velocity 1.0 rad/s.

## Topics and frames reference

| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Driving input; relayed to the controller's reference topic |
| `/ackermann_steering_controller/reference_unstamped` | `geometry_msgs/Twist` | Controller's native driving input (deprecated in favor of the stamped `~/reference`, but still functional) |
| `/ackermann_steering_controller/odometry` | `nav_msgs/Odometry` | Controller's native wheel odometry output, relayed to `/odom` |
| `/odom` | `nav_msgs/Odometry` | Wheel-based odometry from `ackermann_steering_controller` (real dead reckoning, not ground truth — see Known limitations in `CHANGELOG.md`) |
| `/scan` | `sensor_msgs/LaserScan` | From the simulated lidar |
| `/map` | `nav_msgs/OccupancyGrid` | From Cartographer (mapping) or the map server (navigation) |
| `/joint_states` | `sensor_msgs/JointState` | Aggregated joint states |

TF tree: `map -> odom -> base -> {lidar, hubfl, hubfr, wheelfl, wheelfr, wheelrl, wheelrr,
camera_*}`. During SLAM, Cartographer owns both `map -> odom` and `odom -> base`. During
navigation, AMCL owns `map -> odom` and `ackermann_steering_controller` (via `odom_tf_relay`)
owns `odom -> base`.

## Troubleshooting

- **Robot doesn't move**: confirm controllers are actually active:
  `ros2 control list_controllers` should show `joint_state_broadcaster` and
  `ackermann_steering_controller` both `active`, under the (unnamespaced) default
  `/controller_manager`.
- **No `/scan` data**: check `gazebo_ros_ray_sensor` plugin output in the Gazebo terminal for
  plugin load errors.
- **No `/odom` or `/tf` (`odom -> base`) data**: check that `odom_relay`/`odom_tf_relay` are
  running (`ros2 node list | grep relay`) and that `/ackermann_steering_controller/odometry` /
  `/ackermann_steering_controller/tf_odometry` are actually being published — if not, check
  `ros2 control list_controllers` for the controller's own activation state first.
- **AMCL doesn't localize / lidar points seem to jump or drift while driving**: first verify the
  initial pose (2D Pose Estimate) matches the robot's actual position in the map, and give AMCL a
  few seconds and a bit of driving to converge (check `/amcl_pose`'s covariance shrinking).
  Wheel odometry now has real dead-reckoning drift (unlike the old ground-truth-based `/odom`) —
  some AMCL correction on every scan match is expected, not a bug.
- **Weird/inconsistent behavior across repeated test launches (duplicate TF, jittery joints)**:
  make sure no processes from a previous run are still alive before relaunching -
  `ps aux | grep -E "gazebo|gzserver|gzclient|rviz2|robot_state_publisher|joint_state_publisher|topic_tools|controller_manager"`
  should show nothing before a fresh `ros2 launch`. A `joint_state_publisher` (or any other node)
  left running from an earlier, not-fully-killed launch will silently double-publish and corrupt
  the TF tree.
- **`ros2 launch` crashes immediately with `NameError: name 'false' is not defined`**: this means
  a launch argument named `slam` somewhere is colliding with `nav2_bringup`'s own `slam`
  argument (see the 2026-07-05 entry in `CHANGELOG.md`) — don't reintroduce a top-level argument
  literally named `slam` in these launch files.

See `CHANGELOG.md` for a history of fixes applied to this package.
