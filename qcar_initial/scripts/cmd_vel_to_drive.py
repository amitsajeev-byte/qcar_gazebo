#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

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
        # Convert linear.x to wheel speed
        speed = msg.linear.x * 20.0
        drive_msg = Float64MultiArray()
        drive_msg.data = [-speed, speed]
        self.drive_pub.publish(drive_msg)

        # Convert angular.z to steering angle
        if abs(msg.angular.z) < 0.01:
            steer = 0.0
        else:
            steer = msg.angular.z * 0.2
            steer = max(-0.5236, min(0.5236, steer))
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