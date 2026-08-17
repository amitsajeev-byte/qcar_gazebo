#!/usr/bin/env python3
'''check_stop_latency.py

Diagnostic tool for the real-hardware stop-distance investigation. Runs a
single drive-to-distance-then-stop test and, from that one run, reports two
things at once (no need for a separate calibration pass):

1. Cruise-phase velocity: mean of the real encoder-derived /odom velocity
   during the drive phase, skipping an initial WARMUP_SKIP_S to let the
   throttle rate-limiter settle - this is the real measured speed for a
   given commanded speed, i.e. exactly what THROTTLE_GAIN calibration needs
   (compare against `speed` to get the correction ratio).
2. Stop latency: the exact moment the stop was commanded (time + distance)
   vs. the moment the car's own odometry shows it actually stopped (velocity
   below a threshold for a debounce window, to reject a single noisy
   near-zero sample mid-coast) - replaces eyeballing "does it stop the
   instant the command prints" with precise numbers.

Usage:
    ros2 run qcar_updated check_stop_latency.py <target_distance_m> <speed_m_s> [stop_threshold_m_s] [debounce_s]

Defaults: stop_threshold=0.02 m/s, debounce=0.3s.

Every /odom sample is logged (receipt time, distance-from-start, velocity,
drive/coast phase) to a CSV in the current directory. Exits automatically
shortly after printing the report - does NOT keep running indefinitely. A
first version did (reasoning: "still commanding zero, safe"), but that left
a zombie process fighting a separately-run teleop node's /cmd_vel commands
with its own zero-velocity publishes, silently blocking all driving until
someone noticed the leftover process (confirmed on hardware 2026-08-17).
'''
import csv
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Skip velocity samples from the drive-phase cruise average until this much
# time has passed since the first /odom sample - lets THROTTLE_RATE_LIMIT's
# ramp-up settle before treating readings as steady-state cruise speed.
WARMUP_SKIP_S = 0.5


class StopLatencyChecker(Node):

    def __init__(self, target_distance, speed, stop_threshold, debounce):
        super().__init__('check_stop_latency')
        self.target_distance = target_distance
        self.speed = speed
        self.stop_threshold = stop_threshold
        self.debounce = debounce

        self.start_xy = None
        self.first_motion_time = None
        self.commanding = True
        self.stop_commanded_time = None
        self.stop_commanded_distance = None
        self.below_threshold_since = None
        self.actual_stop_time = None
        self.cruise_velocities = []
        self.done = False

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_timer(1.0 / 20.0, self.publish_cmd)

        log_name = 'stop_latency_%d.csv' % int(time.time())
        self.log_file = open(log_name, 'w', newline='')
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(['recv_time', 'dist_m', 'v_m_s', 'phase'])
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
        distance = math.hypot(x - self.start_xy[0], y - self.start_xy[1])
        v = msg.twist.twist.linear.x

        # Warmup skip is measured from when the car actually starts moving,
        # not from script start - a real static-friction deadband can delay
        # motion by ~2s after the command is first sent, and counting that
        # stationary period as part of the cruise-phase window silently
        # drags the reported average down (confirmed on hardware 2026-08-17).
        if self.first_motion_time is None and abs(v) > 0.01:
            self.first_motion_time = now
        if (self.commanding and self.first_motion_time is not None
                and (now - self.first_motion_time) >= WARMUP_SKIP_S):
            self.cruise_velocities.append(v)

        if self.commanding and distance >= self.target_distance:
            self.commanding = False
            self.stop_commanded_time = now
            self.stop_commanded_distance = distance
            self.get_logger().info(
                '<<< STOP COMMANDED at t=%.3f dist=%.4fm' % (now, distance))

        phase = 'drive' if self.stop_commanded_time is None else 'coast'
        self.log_writer.writerow(['%.4f' % now, '%.4f' % distance, '%.4f' % v, phase])

        if self.stop_commanded_time is not None and self.actual_stop_time is None:
            if abs(v) <= self.stop_threshold:
                if self.below_threshold_since is None:
                    self.below_threshold_since = now
                elif now - self.below_threshold_since >= self.debounce:
                    self.actual_stop_time = self.below_threshold_since
                    self.report(distance)
            else:
                self.below_threshold_since = None

    def report(self, actual_stop_distance):
        latency = self.actual_stop_time - self.stop_commanded_time
        extra_dist = actual_stop_distance - self.stop_commanded_distance
        self.get_logger().info('=' * 60)
        if self.cruise_velocities:
            measured_v = sum(self.cruise_velocities) / len(self.cruise_velocities)
            ratio = measured_v / self.speed if self.speed else float('nan')
            self.get_logger().info(
                'CRUISE VELOCITY : commanded=%.3fm/s  measured=%.3fm/s  '
                'ratio=%.3fx  (n=%d samples after %.1fs warmup)' % (
                    self.speed, measured_v, ratio, len(self.cruise_velocities), WARMUP_SKIP_S))
            self.get_logger().info(
                '  -> to correct THROTTLE_GAIN, multiply its current value by %.3f' % (
                    1.0 / ratio if ratio else float('nan')))
        else:
            self.get_logger().warn('CRUISE VELOCITY : no samples after warmup - target_distance too small?')
        self.get_logger().info('STOP COMMANDED  : t=%.3f  dist=%.4fm' % (
            self.stop_commanded_time, self.stop_commanded_distance))
        self.get_logger().info('ACTUALLY STOPPED: t=%.3f  dist=%.4fm' % (
            self.actual_stop_time, actual_stop_distance))
        self.get_logger().info('STOP LATENCY    : %.3f s' % latency)
        self.get_logger().info('EXTRA DISTANCE  : %.4f m' % extra_dist)
        self.get_logger().info('=' * 60)
        self.log_file.flush()
        self.done = True


def main():
    if len(sys.argv) < 3:
        print('usage: check_stop_latency.py <target_distance_m> <speed_m_s> '
              '[stop_threshold_m_s] [debounce_s]')
        sys.exit(1)
    target_distance = float(sys.argv[1])
    speed = float(sys.argv[2])
    stop_threshold = float(sys.argv[3]) if len(sys.argv) > 3 else 0.02
    debounce = float(sys.argv[4]) if len(sys.argv) > 4 else 0.3

    rclpy.init()
    node = StopLatencyChecker(target_distance, speed, stop_threshold, debounce)
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
