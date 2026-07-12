# qcar_navigation

Gazebo simulation, SLAM mapping, and Nav2 autonomous navigation for the Quanser QCar2.

Built for classic Gazebo 11 (`gazebo_ros`), not the newer Gazebo Sim (`gz sim` / `ros_gz`)
stack. The vehicle is simulated with a real Ackermann drivetrain (front hubs steer, all
four wheels driven) via the `libgazebo_ros_ackermann_drive.so` plugin, and carries a 2D
LiDAR + IMU.

## Prerequisites

- ROS 2 Humble
- `gazebo_ros`, `gazebo_plugins`, `xacro`, `robot_state_publisher`
- `slam_toolbox` (mapping)
- `nav2_bringup` (navigation)

## Build

```bash
cd ~/ws/robotics/ros2/qcar_ws
colcon build --packages-select qcar_navigation --symlink-install
source install/setup.bash    # or setup.zsh
```

## 1. Simulation only

Spawns the QCar2 in Gazebo with the drive/sensor plugins running, no SLAM or Nav2:

```bash
ros2 launch qcar_navigation gazebo.launch.py
```

Drive it manually:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Useful arguments (all launch files below accept these too):

| Argument        | Default                      | Description                        |
|-----------------|-------------------------------|-------------------------------------|
| `world`         | `worlds/qcar_world.sdf`       | Full path to the SDF world to load |
| `gui`           | `true`                        | Show the Gazebo GUI client         |
| `use_sim_time`  | `true`                        | Use the Gazebo clock               |

## 2. Mapping (SLAM)

Runs Gazebo + `slam_toolbox` (online async) + RViz so you can drive around and build a map:

```bash
ros2 launch qcar_navigation mapping.launch.py
```

Drive the robot around the world with `teleop_twist_keyboard` until the map looks complete
in RViz, then save it:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

Extra argument: `rviz` (default `true`) — set `rviz:=false` to skip launching RViz.

## 3. Navigation

Runs Gazebo + Nav2 bringup (planner, controller, costmaps, BT navigator, etc.), localized
against a saved map:

```bash
ros2 launch qcar_navigation navigation.launch.py map:=/home/you/my_map.yaml
```

By default this loads the sample map shipped in `maps/my_map.yaml`. In RViz, use
**2D Pose Estimate** to set the robot's initial pose, then **2D Goal Pose** to send it
somewhere.

Extra arguments:

| Argument      | Default                        | Description                                          |
|---------------|----------------------------------|-------------------------------------------------------|
| `map`         | `maps/my_map.yaml`             | Map to localize against                              |
| `params_file` | `config/nav2_params.yaml`        | Nav2 parameter file                                   |
| `slam`        | `false`                          | `true` runs `slam_toolbox` instead of AMCL (SLAM+Nav2 concurrently, for unknown environments) |
| `rviz`        | `true`                           | Launch RViz with the Nav2 display config             |

## Package layout

```
urdf/qcar2.urdf.xacro   QCar2 model: Ackermann drive, ray lidar, IMU, joint states
worlds/                 SDF worlds, loaded directly by classic `gazebo`/`gzserver`
meshes/                 QCar2 STL meshes
maps/                   Sample pre-built map (my_map.yaml / .pgm)
config/mapper_params_online_async.yaml   slam_toolbox parameters
config/nav2_params.yaml          Nav2 parameters (MPPI controller tuned for Ackermann steering)
rviz/nav2.rviz          RViz display config
launch/gazebo.launch.py      Spawns the robot with its drive/sensor plugins (used by both launch files below)
launch/mapping.launch.py     gazebo.launch.py + slam_toolbox + RViz
launch/navigation.launch.py  gazebo.launch.py + nav2_bringup + RViz
```

## Notes / known quirks

- `libgazebo_ros_ackermann_drive.so` subscribes on a plain `/cmd_vel` (no bridge/remap
  needed) and auto-computes wheelbase/track/wheel radius from the joint poses and
  collision geometry in the URDF.
- Classic Gazebo doesn't use gz-sim's opt-in "system" plugins — physics, sensors, and IMU
  are built into `gzserver`/`gzclient` directly, so the world SDF files don't need to
  declare them.
