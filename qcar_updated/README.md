# qcar_updated

ROS 2 / Gazebo (classic) simulation, SLAM, and Nav2 navigation stack for the QCar
Ackermann-steered robot.

## Prerequisites

- **ROS 2 Humble on Ubuntu 22.04 (Jammy)**, or another combination where Gazebo **classic**
  (`gazebo11`) is still packaged. This stack uses the classic `gazebo_ros`/`gazebo_ros2_control`
  plugins (`libgazebo_ros_factory.so`, `libgazebo_ros_camera.so`, `libgazebo_ros_ray_sensor.so`,
  `libgazebo_ros_p3d.so`, `libgazebo_ros2_control.so`) and the `gazebo`/`spawn_entity.py`
  executables — none of which exist for newer distros/OSes that ship only the new Gazebo
  (Harmonic/Ionic, `gz-sim`). In particular, **ROS 2 Jazzy on Ubuntu 24.04 (Noble) will not
  work**: `gazebo11` has no apt candidate there, and `rosdep install` will fail to resolve
  `gazebo_ros2_control` (and silently can't resolve `gazebo_ros`/`gazebo_plugins` either, since
  those aren't released for Noble). Run this package in an Ubuntu 22.04 + ROS 2 Humble
  environment (native, VM, or container) if your host is on a newer distro.
- `gazebo_ros`, `gazebo_ros2_control`
- `robot_state_publisher`, `joint_state_publisher`, `controller_manager`
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
| `launch/qcar_updated.launch.py` | Core sim bring-up: Gazebo, robot spawn, `robot_state_publisher`, controllers, RViz |
| `launch/qcar_slam.launch.py` | Mapping: Cartographer SLAM (includes the sim by default) |
| `launch/qcar_nav2.launch.py` | Navigation: Nav2 stack against a saved map (includes the sim by default) |
| `config/qcar_controllers.yaml` | `ros2_control` controller manager config (drive + steering controllers) |
| `config/cartographer/qcar_2d.lua` | Cartographer SLAM parameters |
| `config/nav2/nav2_params.yaml` | Nav2 stack parameters (AMCL, costmaps, planner, controller) |
| `maps/qcar_map.yaml` / `.pgm` | Saved occupancy grid map used by `qcar_nav2.launch.py` |
| `scripts/cmd_vel_to_drive.py` | Converts `/cmd_vel` (Twist) into wheel/steering commands using the vehicle's bicycle model |
| `scripts/odom_to_tf.py` | Publishes the `odom -> base` TF from the simulated odometry topic |
| `scripts/qcar_teleop_twist.py` | Keyboard teleop publishing `/cmd_vel` (recommended, works with Nav2/SLAM) |
| `scripts/qcar_teleop.py` | Keyboard teleop publishing raw wheel/steering commands directly (bypasses `cmd_vel_to_drive.py`) |
| `worlds/*.world` | Gazebo worlds (`myworld.world` is used by default; others available for manual swap) |

## Quick start: simulation only

```bash
ros2 launch qcar_updated qcar_updated.launch.py
```

Brings up Gazebo (`myworld.world`), spawns the robot, starts `robot_state_publisher`,
`joint_state_publisher`, the `joint_state_broadcaster`/`drive_controller`/`steering_controller`,
and RViz. Nothing drives the robot yet — use one of the teleop scripts below, or one of the
mapping/navigation launch files, which start their own driving pipeline.

## Mapping (SLAM)

1. Launch SLAM (this also brings up the simulation):

   ```bash
   ros2 launch qcar_updated qcar_slam.launch.py
   ```

   This starts Gazebo + the robot (unless disabled, see [Launch arguments](#launch-arguments)),
   `cmd_vel_to_drive.py`, and the two Cartographer nodes (`cartographer_node` publishing the
   `map -> odom -> base` TF, and `cartographer_occupancy_grid_node` publishing `/map`).

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

**Note:** `odom_to_tf.py` is intentionally *not* launched during SLAM — Cartographer already
publishes the `odom -> base` transform itself (`provide_odom_frame: true` in
`config/cartographer/qcar_2d.lua`); running both would fight over the same TF edge.

## Navigation (Nav2 + AMCL)

1. Make sure `maps/qcar_map.yaml` is the map you want to navigate in (see Mapping above to
   (re)generate it).

2. Launch navigation (this also brings up the simulation):

   ```bash
   ros2 launch qcar_updated qcar_nav2.launch.py
   ```

   This starts Gazebo + the robot, `cmd_vel_to_drive.py`, `odom_to_tf.py` (publishes
   `odom -> base` from the simulated odometry, since nothing else does in this mode), and the
   full Nav2 `bringup_launch.py` (map server, AMCL, planner, controller, behavior server,
   BT navigator) configured with `config/nav2/nav2_params.yaml`.

3. In RViz, set the robot's starting pose with **2D Pose Estimate** if it doesn't match the
   AMCL default (`x=0, y=0, yaw=0`, configured in `nav2_params.yaml`).

4. Send a goal with **2D Nav Goal** in RViz, or via the `navigate_to_pose` action, and the robot
   will plan and drive there using `nav2_regulated_pure_pursuit_controller`.

You can also drive manually at any time with `ros2 run qcar_updated qcar_teleop_twist.py`
(publishes to `/cmd_vel`, same as Nav2's controller output).

## Launch arguments

Both `qcar_slam.launch.py` and `qcar_nav2.launch.py` accept:

| Argument | Default | Description |
|---|---|---|
| `launch_sim` | `true` | Whether to include `qcar_updated.launch.py` (Gazebo + robot + controllers). Set to `false` if the simulation is already running (e.g. started separately, or you're driving real hardware) to avoid spawning a second instance. |

Example:

```bash
ros2 launch qcar_updated qcar_slam.launch.py launch_sim:=false
```

## Key robot parameters

These are used by `scripts/cmd_vel_to_drive.py` and `scripts/qcar_teleop_twist.py` to convert
between `Twist` commands and wheel/steering commands (bicycle model), and were derived from the
STL meshes and joint origins in `urdf/qcar_model.xacro`:

| Parameter | Value | Source |
|---|---|---|
| Wheel radius | 0.033 m | `models/qcar/QCarWheel.stl` bounding box |
| Wheelbase | 0.25725 m | Sum of front/rear hub joint x-offsets (0.12960 + 0.12765) |
| Max steering angle | ±0.5236 rad (30°) | `base_hubfl_joint` / `base_hubfr_joint` limits |
| Lidar range | 0.15 - 12.0 m | Lidar sensor plugin, matches `nav2_params.yaml` |

Nav2 speed limits (`config/nav2/nav2_params.yaml`): max linear velocity 0.3 m/s
(`velocity_smoother`, `FollowPath.desired_linear_vel`), max angular velocity 1.0 rad/s.

## Topics and frames reference

| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Driving input; consumed by `cmd_vel_to_drive.py` |
| `/drive_controller/commands` | `std_msgs/Float64MultiArray` | Rear wheel velocity commands (rad/s) |
| `/steering_controller/commands` | `std_msgs/Float64MultiArray` | Front hub position commands (rad) |
| `/odom` | `nav_msgs/Odometry` | From the `gazebo_ros_p3d` plugin (with small Gaussian noise) |
| `/scan` | `sensor_msgs/LaserScan` | From the simulated lidar |
| `/map` | `nav_msgs/OccupancyGrid` | From Cartographer (mapping) or the map server (navigation) |
| `/joint_states` | `sensor_msgs/JointState` | Aggregated joint states |

TF tree: `map -> odom -> base -> {lidar, hubfl, hubfr, wheelfl, wheelfr, wheelrl, wheelrr,
camera_*}`. During SLAM, Cartographer owns `map -> odom` and `odom -> base`. During navigation,
AMCL owns `map -> odom` and `odom_to_tf.py` owns `odom -> base`.

## Troubleshooting

- **Robot doesn't move**: confirm controllers are actually active:
  `ros2 control list_controllers` should show `joint_state_broadcaster`, `drive_controller`, and
  `steering_controller` all `active`, all under the (unnamespaced) default `/controller_manager`.
- **No `/scan` or `/odom` data**: check `gazebo_ros_ray_sensor` / `gazebo_ros_p3d` plugin output
  in the Gazebo terminal for plugin load errors.
- **AMCL doesn't localize**: verify the initial pose (2D Pose Estimate) roughly matches the
  robot's actual position in the map, and that `/scan` is publishing.

See `CHANGELOG.md` for a history of fixes applied to this package.
