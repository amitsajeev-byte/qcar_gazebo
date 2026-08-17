#!/usr/bin/env python3
'''drive_and_log.py <target_distance_m> <speed_m_s> [tail_seconds]

Simple drive-and-log test: publishes /cmd_vel at speed until /odom-derived
distance reaches target, then publishes zero and keeps logging for
tail_seconds to capture the coast-out. Logs every /odom sample (time,
distance, velocity, drive/coast phase) to a CSV in the current directory.
Exits automatically after the tail window - no bells and whistles. This was
the primary tool used for the 2026-08-17 THROTTLE_GAIN calibration sweep
(see qcar_bridge.py's THROTTLE_GAIN comment and the
qcar_updated_stop_distance_investigation project memory) - kept simpler
than check_stop_latency.py deliberately, per the user's request.
'''
import csv
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class DriveAndLog(Node):

    def __init__(self, target_distance, speed, tail_seconds):
        super().__init__('drive_and_log')
        self.target_distance = target_distance
        self.speed = speed
        self.tail_seconds = tail_seconds

        self.start_xy = None
        self.commanding = True
        self.stop_time = None
        self.done = False

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_timer(1.0 / 20.0, self.publish_cmd)

        log_name = 'drive_log_%d.csv' % int(time.time())
        self.log_file = open(log_name, 'w', newline='')
        self.writer = csv.writer(self.log_file)
        self.writer.writerow(['t', 'dist_m', 'v_m_s', 'phase'])
        self.get_logger().info('driving %.2fm @ %.2fm/s, logging to %s' % (
            target_distance, speed, log_name))

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.speed if self.commanding else 0.0
        self.cmd_pub.publish(msg)

    def odom_callback(self, msg):
        now = time.time()
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if self.start_xy is None:
            self.start_xy = (x, y)
        dist = math.hypot(x - self.start_xy[0], y - self.start_xy[1])
        v = msg.twist.twist.linear.x

        if self.commanding and dist >= self.target_distance:
            self.commanding = False
            self.stop_time = now
            self.get_logger().info('<<< STOP COMMANDED at t=%.3f dist=%.4fm' % (now, dist))

        phase = 'drive' if self.stop_time is None else 'coast'
        self.writer.writerow(['%.4f' % now, '%.4f' % dist, '%.4f' % v, phase])

        if self.stop_time is not None and (now - self.stop_time) >= self.tail_seconds:
            self.get_logger().info('done (%.1fs tail elapsed), exiting' % self.tail_seconds)
            self.log_file.flush()
            self.done = True


def main():
    if len(sys.argv) < 3:
        print('usage: drive_and_log.py <target_distance_m> <speed_m_s> [tail_seconds]')
        sys.exit(1)
    target_distance = float(sys.argv[1])
    speed = float(sys.argv[2])
    tail_seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 15.0

    rclpy.init()
    node = DriveAndLog(target_distance, speed, tail_seconds)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.log_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
