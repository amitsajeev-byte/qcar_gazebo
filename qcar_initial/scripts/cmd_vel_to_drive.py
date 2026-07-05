#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

# Measured from models/qcar/QCarWheel.stl (0.033 m radius) and the front/rear
# hub joint origins in urdf/qcar_model.xacro (0.12960 + 0.12765 m wheelbase).
WHEEL_RADIUS = 0.033
WHEELBASE = 0.25725
MAX_STEER = 0.5236

class CmdVelToDrive(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_drive')

        self.drive_pub = self.create_publisher(
            Float64MultiArray,
            '/drive_controller/commands',
            10
        )
        self.steer_pub = self.create_publisher(
            Float64MultiArray,
            '/steering_controller/commands',
            10
        )
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback,
            10
        )
        self.get_logger().info('cmd_vel_to_drive started')

    def callback(self, msg):
        # Convert linear.x (m/s) to wheel angular speed (rad/s)
        speed = msg.linear.x / WHEEL_RADIUS
        drive_msg = Float64MultiArray()
        drive_msg.data = [-speed, speed]
        self.drive_pub.publish(drive_msg)

        # Bicycle-model inversion: steering_angle = atan(wheelbase * yaw_rate / linear_vel).
        # With no forward speed a car-like steering axle can't produce a meaningful angle.
        if abs(msg.linear.x) < 0.01:
            steer = 0.0
        else:
            steer = math.atan2(WHEELBASE * msg.angular.z, msg.linear.x)
            steer = max(-MAX_STEER, min(MAX_STEER, steer))
        steer_msg = Float64MultiArray()
        steer_msg.data = [steer, steer]
        self.steer_pub.publish(steer_msg)

def main():
    rclpy.init()
    node = CmdVelToDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()