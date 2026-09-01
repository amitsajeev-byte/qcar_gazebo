#!/usr/bin/env python3
# Pure kinematic "fake robot" for comparing nav2 planner/controller behavior against a
# hardware-captured map without Gazebo, whose fixed world geometry doesn't match any real map we
# capture. Integrates /cmd_vel (unicycle model: linear.x, angular.z) into a pose at a fixed rate
# and publishes odom + the odom->base TF - no physics, no collision, no sensor noise. Assumes
# map->odom is supplied separately (a static identity broadcaster in this comparison setup), since
# there is no live sensor here for a localizer to correct against.
import math

import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class FakeOdomNode(Node):
    def __init__(self):
        super().__init__('fake_odom_node')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        rate_hz = self.get_parameter('rate_hz').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.x = self.get_parameter('start_x').value
        self.y = self.get_parameter('start_y').value
        self.yaw = self.get_parameter('start_yaw').value
        self.v = 0.0
        self.wz = 0.0

        self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_time = self.get_clock().now()
        self.create_timer(1.0 / rate_hz, self._on_timer)

    def _on_cmd_vel(self, msg: Twist):
        self.v = msg.linear.x
        self.wz = msg.angular.z

    def _on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0:
            return

        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw = math.atan2(
            math.sin(self.yaw + self.wz * dt), math.cos(self.yaw + self.wz * dt))

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = FakeOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
