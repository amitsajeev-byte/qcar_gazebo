#!/usr/bin/env python3
'''monitor_goal_timing.py

Instruments a single Nav2 /navigate_to_pose goal to answer: exactly when
did the goal get accepted, when did the REAL robot start moving (from
/odom, encoder ground truth via qcar_bridge.py), when did the RVIZ-DISPLAYED
pose (map->base TF, what a human watching RViz actually sees) start
moving, when did the goal get reached, and when did each of those stop.
Logs every sample (both odom-frame and map-frame pose) to a CSV, and
prints a summary table with timestamps + pose at each key event, plus the
position jump (if any) at the moment RViz's displayed pose first "catches
up" after a period of being stale.

Usage: just run it, then send a Nav2 goal (RViz "2D Goal Pose" or
`ros2 action send_goal`) while it's running. Exits automatically ~5s after
detecting goal SUCCEEDED, or after a 90s safety timeout if no goal ever
completes.
'''
import csv
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatusArray
from tf2_ros import Buffer, TransformListener

# Matches bt_navigator's actual publisher QoS for the action status topic
# (RELIABLE + TRANSIENT_LOCAL) - the rclpy default subscription QoS (plain
# int depth) leaves durability at VOLATILE, which should be spec-compatible
# with a TRANSIENT_LOCAL publisher in theory, but delivered zero messages
# in practice (confirmed on hardware 2026-08-17, twice) - matching exactly
# is the safe fix.
STATUS_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

VEL_THRESHOLD = 0.02       # m/s - "real motion" cutoff, matches earlier scripts
POS_THRESHOLD = 0.03       # m - "rviz pose changed" cutoff
STOP_SETTLE_S = 0.3        # must stay below threshold this long to call it "stopped"
SAFETY_TIMEOUT_S = 90.0

# action_msgs/msg/GoalStatus status codes
STATUS_ACCEPTED = 1
STATUS_EXECUTING = 2
STATUS_SUCCEEDED = 4


class GoalTimingMonitor(Node):

    def __init__(self):
        super().__init__('monitor_goal_timing')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                  self.status_cb, STATUS_QOS)
        self.create_timer(1.0 / 20.0, self.poll_map_pose)

        log_name = 'goal_timing_%d.csv' % int(time.time())
        self.log_file = open(log_name, 'w', newline='')
        self.writer = csv.writer(self.log_file)
        self.writer.writerow(['t', 'source', 'x', 'y', 'yaw_or_v'])

        self.goal_sent_t = None
        self.goal_reached_t = None
        self.real_start_t = None
        self.real_stop_t = None
        self.rviz_start_t = None
        self.rviz_stop_t = None
        self.rviz_start_pose = None
        self.rviz_stop_pose = None
        self.real_start_pose = None
        self.done = False
        self.done_at = None

        self.rviz_pose_at_goal_sent = None
        self.below_vel_since = None
        self.rviz_still_since = None
        self.rviz_last_pose = None
        self.known_goal_ids = set()
        self.tracked_goal_id = None

    def odom_cb(self, msg):
        now = time.time()
        v = msg.twist.twist.linear.x
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.writer.writerow(['%.4f' % now, 'odom', '%.4f' % x, '%.4f' % y, '%.4f' % v])

        if self.goal_sent_t is None:
            return

        # Real motion start (after goal sent, before goal reached)
        if self.real_start_t is None and self.goal_reached_t is None:
            if abs(v) > VEL_THRESHOLD:
                self.real_start_t = now
                self.real_start_pose = (x, y)
                self.get_logger().info('REAL START at t=%.3f (%.3fs after goal) pose=(%.3f,%.3f)' % (
                    now, now - self.goal_sent_t, x, y))

        # Real stop, debounced, only after goal reached
        if self.goal_reached_t is not None and self.real_stop_t is None:
            if abs(v) <= VEL_THRESHOLD:
                if self.below_vel_since is None:
                    self.below_vel_since = now
                elif now - self.below_vel_since >= STOP_SETTLE_S:
                    self.real_stop_t = self.below_vel_since
                    self.get_logger().info('REAL STOP at t=%.3f (%.3fs after goal reached)' % (
                        self.real_stop_t, self.real_stop_t - self.goal_reached_t))
            else:
                self.below_vel_since = None

    def poll_map_pose(self):
        now = time.time()
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base', rclpy.time.Time())
        except Exception:
            return
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w
        import math
        yaw = 2.0 * math.atan2(qz, qw)
        self.writer.writerow(['%.4f' % now, 'map_base', '%.4f' % x, '%.4f' % y, '%.4f' % yaw])

        if self.goal_sent_t is None:
            self.rviz_last_pose = (x, y)
            return

        if self.rviz_pose_at_goal_sent is None:
            self.rviz_pose_at_goal_sent = (x, y)

        # RViz-displayed motion start (after goal sent, before goal reached)
        if self.rviz_start_t is None and self.goal_reached_t is None:
            dx = x - self.rviz_pose_at_goal_sent[0]
            dy = y - self.rviz_pose_at_goal_sent[1]
            if (dx * dx + dy * dy) ** 0.5 > POS_THRESHOLD:
                self.rviz_start_t = now
                self.rviz_start_pose = (x, y)
                self.get_logger().info('RVIZ START at t=%.3f (%.3fs after goal) pose=(%.3f,%.3f)' % (
                    now, now - self.goal_sent_t, x, y))

        # RViz-displayed stop, debounced, only after goal reached
        if self.goal_reached_t is not None and self.rviz_stop_t is None:
            if self.rviz_last_pose is not None:
                dx = x - self.rviz_last_pose[0]
                dy = y - self.rviz_last_pose[1]
                moved = (dx * dx + dy * dy) ** 0.5 > 0.005  # per-tick noise floor
            else:
                moved = True
            if not moved:
                if self.rviz_still_since is None:
                    self.rviz_still_since = now
                elif now - self.rviz_still_since >= STOP_SETTLE_S:
                    self.rviz_stop_t = self.rviz_still_since
                    self.rviz_stop_pose = (x, y)
                    self.get_logger().info('RVIZ STOP at t=%.3f (%.3fs after goal reached) pose=(%.3f,%.3f)' % (
                        self.rviz_stop_t, self.rviz_stop_t - self.goal_reached_t, x, y))
            else:
                self.rviz_still_since = None

        self.rviz_last_pose = (x, y)

        if self.report_ready() and self.done_at is None:
            self.done_at = now
        if self.done_at is not None and now - self.done_at >= 5.0:
            self.report()
            self.done = True

    def status_cb(self, msg):
        # Track by goal UUID rather than waiting for a specific status code -
        # confirmed on hardware 2026-08-17 that a goal can go straight from
        # "not present" to EXECUTING(2) in the time between two status
        # publishes, skipping a separately-observable ACCEPTED(1) update
        # entirely. "Goal sent" is instead the first time we see a goal_id
        # we've never seen before (after the initial snapshot of whatever
        # was already retained/completed at startup, which doesn't count).
        now = time.time()
        if not self.known_goal_ids:
            for s in msg.status_list:
                self.known_goal_ids.add(bytes(s.goal_info.goal_id.uuid))
            self.get_logger().info('seeded %d pre-existing goal id(s), waiting for a new one' % len(
                self.known_goal_ids))
            return

        for s in msg.status_list:
            gid = bytes(s.goal_info.goal_id.uuid)
            if gid not in self.known_goal_ids:
                self.known_goal_ids.add(gid)
                if self.goal_sent_t is None:
                    self.goal_sent_t = now
                    self.tracked_goal_id = gid
                    self.get_logger().info('GOAL SENT (new goal_id first seen) at t=%.3f' % now)
            if (gid == self.tracked_goal_id and s.status == STATUS_SUCCEEDED
                    and self.goal_reached_t is None and self.goal_sent_t is not None):
                self.goal_reached_t = now
                self.get_logger().info('GOAL REACHED at t=%.3f (%.3fs after goal sent)' % (
                    now, now - self.goal_sent_t))

    def report_ready(self):
        return (self.goal_sent_t and self.real_start_t and self.rviz_start_t
                and self.goal_reached_t and self.real_stop_t and self.rviz_stop_t)

    def report(self):
        self.get_logger().info('=' * 70)
        self.get_logger().info('GOAL SENT      : t=%.3f' % self.goal_sent_t)
        self.get_logger().info('REAL START     : t=%.3f  (+%.3fs)  pose=(%.3f,%.3f)' % (
            self.real_start_t, self.real_start_t - self.goal_sent_t, *self.real_start_pose))
        self.get_logger().info('RVIZ START     : t=%.3f  (+%.3fs)  pose=(%.3f,%.3f)' % (
            self.rviz_start_t, self.rviz_start_t - self.goal_sent_t, *self.rviz_start_pose))
        self.get_logger().info('START GAP (rviz - real): %.3fs' % (self.rviz_start_t - self.real_start_t))
        self.get_logger().info('GOAL REACHED   : t=%.3f  (+%.3fs since sent)' % (
            self.goal_reached_t, self.goal_reached_t - self.goal_sent_t))
        self.get_logger().info('REAL STOP      : t=%.3f  (+%.3fs since goal reached)' % (
            self.real_stop_t, self.real_stop_t - self.goal_reached_t))
        self.get_logger().info('RVIZ STOP      : t=%.3f  (+%.3fs since goal reached)  pose=(%.3f,%.3f)' % (
            self.rviz_stop_t, self.rviz_stop_t - self.goal_reached_t, *self.rviz_stop_pose))
        self.get_logger().info('STOP GAP (rviz - real): %.3fs' % (self.rviz_stop_t - self.real_stop_t))
        if self.rviz_stop_pose and self.rviz_start_pose:
            import math
            dx = self.rviz_stop_pose[0] - self.rviz_start_pose[0]
            dy = self.rviz_stop_pose[1] - self.rviz_start_pose[1]
            self.get_logger().info('RVIZ net displacement start->stop: %.3fm' % math.hypot(dx, dy))
        self.get_logger().info('=' * 70)
        self.log_file.flush()


def wait_for_discovery(node, topic, timeout_s=10.0):
    '''Blocks until at least one publisher on `topic` is discovered, plus a
    small settle margin - a subscription created and immediately assumed
    "ready" can miss the goal's ACCEPTED transition entirely if DDS
    discovery with the remote publisher (bt_navigator, running in a
    different process) hasn't finished matching yet, especially since this
    topic's TRANSIENT_LOCAL history only retains the latest status, not the
    full transition history - confirmed missing on first attempt,
    2026-08-17.'''
    start = time.time()
    while time.time() - start < timeout_s:
        infos = node.get_publishers_info_by_topic(topic)
        if infos:
            time.sleep(1.0)  # settle margin for the match to fully complete
            return True
        rclpy.spin_once(node, timeout_sec=0.2)
    return False


def main():
    rclpy.init()
    node = GoalTimingMonitor()
    if not wait_for_discovery(node, '/navigate_to_pose/_action/status'):
        node.get_logger().error('never discovered a publisher for the action status topic - '
                                 'is Nav2 actually running?')
    node.get_logger().info('discovery complete - send the goal now')
    start = time.time()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.time() - start > SAFETY_TIMEOUT_S:
                node.get_logger().warn('safety timeout, exiting without full report')
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.log_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
