#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import sys
import tty
import termios

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
        self.steer_pub = self.create_publisher(
            Float64MultiArray,
            '/steering_controller/commands',
            10
        )

        self.speed = 0.2  # m/s
        self.steer_angle = 0.4  # radians
        self.current_drive = 0.0
        self.current_steer = 0.0

        print(MSG)
        print(f'Current speed: {self.speed:.2f} m/s')

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key

    def publish_drive(self, speed):
        msg = Twist()
        msg.linear.x = speed
        self.drive_pub.publish(msg)

    def publish_steer(self, angle):
        msg = Float64MultiArray()
        msg.data = [angle, angle]
        self.steer_pub.publish(msg)

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
                self.current_steer = self.steer_angle
                print(f'Steer Left  | angle: {self.steer_angle:.2f} rad')
            elif key == 'd':
                self.current_steer = -self.steer_angle
                print(f'Steer Right | angle: {self.steer_angle:.2f} rad')
            elif key == 'z':
                self.current_steer = 0.0
                print('Steer Centre')
            elif key == 'x':
                self.current_drive = 0.0
                self.current_steer = 0.0
                print('Stop & Centre')
            elif key == 'q':
                self.speed = min(self.speed + 0.05, 1.0)
                print(f'Speed increased: {self.speed:.2f} m/s')
            elif key == 'e':
                self.speed = max(self.speed - 0.05, 0.05)
                print(f'Speed decreased: {self.speed:.2f} m/s')
            elif key == '\x03':
                break

            self.publish_drive(self.current_drive)
            self.publish_steer(self.current_steer)

def main():
    rclpy.init()
    node = QCarTwistTeleop()
    try:
        node.run()
    except Exception as e:
        print(e)
    finally:
        node.publish_drive(0.0)
        node.publish_steer(0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
