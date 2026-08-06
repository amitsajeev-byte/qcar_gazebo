#!/usr/bin/env python3
'''qcar_teleop_twist.py

Keyboard teleop publishing /cmd_vel (geometry_msgs/Twist). Run this on the
dev PC - the same /cmd_vel is consumed by whichever drive is currently up:
Gazebo's gazebo_ros_ackermann_drive plugin in simulation, or
qcar_hardware/qcar_onboard/qcar_bridge.py listening on the QCar's onboard
ROS2 Dashing for real hardware. No topic/type change needed either way; for
real hardware, ROS_DOMAIN_ID must match on both machines and both must be
reachable over the same network (see hardware_integration_reference.md for
the untested Dashing<->Humble wire-compatibility risk this depends on).
'''
import math
import os
import sys
import tty
import termios

# Pin the RMW implementation explicitly rather than trusting whatever each
# machine's shell environment happens to default to - Dashing and Humble both
# default to rmw_fastrtps_cpp, but if either machine's environment overrides
# this differently (e.g. a cyclonedds install), the two ends would silently
# never discover each other. Must be set before rclpy is imported. This does
# NOT by itself guarantee Dashing<->Humble discovery/serialization actually
# works across that many releases - still verify with a plain talker/listener
# test first, per hardware_integration_reference.md.
os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Must match urdf/qcar_model.xacro hub joint origins (0.12960 + 0.12765 m).
WHEELBASE = 0.25725

MSG = """
QCar Twist Teleop
-----------------
Driving:
   w : forward
   s : backward
   a : steer left
   d : steer right
   z : steer centre
   x : stop & centre

Speed control:
   q : increase speed
   e : decrease speed

CTRL+C to quit
"""

class QCarTwistTeleop(Node):
    def __init__(self):
        super().__init__('qcar_teleop_twist')

        self.drive_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.speed = 0.2  # m/s
        self.steer_angle = 0.4  # desired steering angle, radians
        self.current_drive = 0.0
        self.current_steer_angle = 0.0

        print(MSG)
        print(f'Current speed: {self.speed:.2f} m/s')
        print(f"Publishing /cmd_vel on ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0 (default)')} "
              f"- confirm this matches the QCar's onboard qcar_hw_bridge.py if driving real hardware.")

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key

    def publish_drive(self, speed, steer_angle):
        # cmd_vel.angular.z is a yaw rate, not a steering angle: invert the
        # bicycle model so the downstream consumer recovers steer_angle - either
        # Gazebo's gazebo_ros_ackermann_drive plugin (sim) or
        # qcar_hardware/qcar_onboard/qcar_bridge.py's cmd_vel_callback() on
        # real hardware, which applies the exact inverse of this formula.
        yaw_rate = 0.0
        if abs(speed) > 1e-3:
            yaw_rate = speed * math.tan(steer_angle) / WHEELBASE
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = yaw_rate
        self.drive_pub.publish(msg)

    def run(self):
        while rclpy.ok():
            key = self.get_key()

            if key == 'w':
                self.current_drive = self.speed
                print(f'Forward  | speed: {self.speed:.2f} m/s')
            elif key == 's':
                self.current_drive = -self.speed
                print(f'Backward | speed: {self.speed:.2f} m/s')
            elif key == 'a':
                self.current_steer_angle = self.steer_angle
                print(f'Steer Left  | angle: {self.steer_angle:.2f} rad')
            elif key == 'd':
                self.current_steer_angle = -self.steer_angle
                print(f'Steer Right | angle: {self.steer_angle:.2f} rad')
            elif key == 'z':
                self.current_steer_angle = 0.0
                print('Steer Centre')
            elif key == 'x':
                self.current_drive = 0.0
                self.current_steer_angle = 0.0
                print('Stop & Centre')
            elif key == 'q':
                self.speed = min(self.speed + 0.05, 1.0)
                print(f'Speed increased: {self.speed:.2f} m/s')
            elif key == 'e':
                self.speed = max(self.speed - 0.05, 0.05)
                print(f'Speed decreased: {self.speed:.2f} m/s')
            elif key == '\x03':
                break

            self.publish_drive(self.current_drive, self.current_steer_angle)

def main():
    rclpy.init()
    node = QCarTwistTeleop()
    try:
        node.run()
    except Exception as e:
        print(e)
    finally:
        node.publish_drive(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()