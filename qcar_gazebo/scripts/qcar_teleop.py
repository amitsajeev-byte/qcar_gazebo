#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import sys
import tty
import termios

MSG = """
QCar Keyboard Teleop
--------------------
Driving:
   w : forward
   s : backward
   a : steer left
   d : steer right
   x : stop & centre
   z : steer centre

Speed control:
   q : increase speed
   e : decrease speed

CTRL+C to quit
"""

class QCarTeleop(Node):
    def __init__(self):
        super().__init__('qcar_teleop')

        self.drive_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.steer_pub = self.create_publisher(
            Float64MultiArray,
            '/steering_controller/commands',
            10
        )

        self.speed = 2.0
        self.steer_angle = 0.4
        self.current_drive = 0.0
        self.current_steer = 0.0

        print(MSG)
        print(f'Current speed: {self.speed:.1f} rad/s')

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
                print(f'Forward  | speed: {self.speed:.1f}')
            elif key == 's':
                self.current_drive = -self.speed
                print(f'Backward | speed: {self.speed:.1f}')
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
                self.speed = min(self.speed + 0.5, 10.0)
                print(f'Speed increased: {self.speed:.1f}')
            elif key == 'e':
                self.speed = max(self.speed - 0.5, 0.5)
                print(f'Speed decreased: {self.speed:.1f}')
            elif key == '\x03':  # CTRL+C
                break

            self.publish_drive(self.current_drive)
            self.publish_steer(self.current_steer)


def main():
    rclpy.init()
    node = QCarTeleop()
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
