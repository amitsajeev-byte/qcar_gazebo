# QCar hardware terminal commands

Reference sheet of every terminal command used to bring up, test, and shut
down the real-hardware QCar stack. Active testing currently happens in
**`qcar_updated`** (not `qcar_hardware`) - see that package's own
`README.md`/`CHANGELOG.md` for why. Commands below target `qcar_updated`;
swap the package name if working in `qcar_hardware` instead.

QCar IP may change per DHCP session - `192.168.123.16` below is this
session's address, confirm before use (`ping <ip>` or check the QCar's own
network settings if connection fails).

## One-time build + deploy (after any local code change)

```bash
cd ~/humble_ws
colcon build --symlink-install --packages-select qcar_updated
source /opt/ros/humble/setup.bash
source ~/humble_ws/install/setup.bash

# push onboard bridge files to the QCar - only needed if qcar_onboard/*.py changed
scp ~/humble_ws/src/qcar_updated/qcar_onboard/*.py nvidia@192.168.123.16:~/ros2_amit/src/onboard_bridge/
```

## Terminal 1 (QCar motor bridge - needs sudo for PYTHONPATH + hardware access)

```bash
ssh nvidia@192.168.123.16
sudo -E env PYTHONPATH=/home/nvidia/Documents/python python3 ~/ros2_amit/src/onboard_bridge/qcar_bridge.py
```
(sudo password: `nvidia`)

## Terminal 2 (QCar LiDAR node - no sudo needed)

```bash
ssh nvidia@192.168.123.16
PYTHONPATH=/home/nvidia/Documents/python python3 ~/ros2_amit/src/onboard_bridge/qcar_lidar_node.py
```

## Measuring qcar_clock_offset (dev PC, repeat each session)

```bash
for i in 1 2 3 4 5; do
  t1=$(date +%s.%N); remote=$(ssh nvidia@192.168.123.16 'date +%s.%N'); t2=$(date +%s.%N)
  python3 -c "print(($remote) - (($t1 + $t2) / 2))"
done
```
Average the 5 printed values (should cluster tightly - large spread means
redo it) and use in place of `0.50` below.

## Terminal 3 (dev PC relay node)

```bash
source /opt/ros/humble/setup.bash
source ~/humble_ws/install/setup.bash
export ROS_DOMAIN_ID=77
ros2 run qcar_updated qcar_relay_node.py --ros-args -p qcar_ip:=192.168.123.16 -p qcar_clock_offset:=0.50
```

## Terminal 4 (dev PC) - SLAM (real hardware path, no Gazebo)

```bash
source /opt/ros/humble/setup.bash
source ~/humble_ws/install/setup.bash
export ROS_DOMAIN_ID=77
ros2 launch qcar_updated qcar_slam.launch.py launch_sim:=false
```

## Terminal 4 alt (dev PC) - Nav2 (real hardware path, no Gazebo)

```bash
source /opt/ros/humble/setup.bash
source ~/humble_ws/install/setup.bash
export ROS_DOMAIN_ID=77
ros2 launch qcar_updated qcar_nav2.launch.py launch_sim:=false
```

## Terminal 5 (dev PC) - teleop

```bash
source /opt/ros/humble/setup.bash
source ~/humble_ws/install/setup.bash
export ROS_DOMAIN_ID=77
ros2 run qcar_updated qcar_teleop_twist.py
```

## Save a map (dev PC, while qcar_slam.launch.py is up)

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=77
ros2 run nav2_map_server map_saver_cli -f ~/humble_ws/src/qcar_updated/maps/qcar_map
```

## AMCL global localization (skip needing a precisely marked start pose)

```bash
export ROS_DOMAIN_ID=77
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
```
Drive a short distance afterward before trusting localization/sending a goal.

## Useful live-diagnostic commands (dev PC, `ROS_DOMAIN_ID=77` set)

```bash
ros2 node list
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic echo /odom --field pose.pose.position --once
ros2 topic echo /amcl_pose --field pose.pose.pose
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo base lidar
```

## Shutdown, in order

```bash
# dev PC: Ctrl+C teleop / nav2 launch / relay node, in that order

# QCar side:
ssh nvidia@192.168.123.16 'pkill -f qcar_lidar_node.py'
ssh nvidia@192.168.123.16 'echo nvidia | sudo -S pkill -f qcar_bridge.py'
ssh nvidia@192.168.123.16 'sudo ss -tlnp | grep -E "5555|5556"'   # should print nothing
```
