#!/usr/bin/env python3
'''qcar_relay_node.py

Bridges /cmd_vel -> the QCar's onboard motor TCP server
(qcar_onboard/qcar_bridge.py) and the QCar's LiDAR TCP stream
(qcar_onboard/qcar_lidar_node.py) -> /scan.

Runs on the dev PC (Humble) - the only thing on this side of the link that
still speaks ROS2. Native ROS2 pub/sub between the QCar's onboard Dashing and
this machine's Humble doesn't interoperate (confirmed on hardware: raw UDP
passes fine in both directions on both arbitrary and DDS-discovery ports,
but Fast-RTPS discovery itself never completes - a protocol-version issue,
not a network one). This node exists to bridge across that gap over plain
TCP sockets instead. See hardware_integration_reference.md for the full
story.

    ros2 run qcar_hardware qcar_relay_node.py --ros-args -p qcar_ip:=<ip>
'''
import json
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

MOTOR_PORT = 5555
LIDAR_PORT = 5556
RECONNECT_DELAY = 2.0  # seconds
SOCKET_TIMEOUT = 2.0  # seconds - bounds how long a dead connection takes to notice


class QCarRelayNode(Node):

    def __init__(self):
        super().__init__('qcar_relay_node')

        self.declare_parameter('qcar_ip', '')
        self.declare_parameter('frame_id', 'lidar')
        qcar_ip = self.get_parameter('qcar_ip').value
        if not qcar_ip:
            raise RuntimeError(
                'qcar_ip parameter is required, e.g.: ros2 run qcar_hardware '
                'qcar_relay_node.py --ros-args -p qcar_ip:=172.24.0.66')
        self.qcar_ip = qcar_ip
        self.frame_id = self.get_parameter('frame_id').value

        self._motor_sock = None
        self._motor_lock = threading.Lock()
        self._stop = False

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        self._motor_thread = threading.Thread(target=self._motor_connection_loop, daemon=True)
        self._lidar_thread = threading.Thread(target=self._lidar_connection_loop, daemon=True)
        self._motor_thread.start()
        self._lidar_thread.start()

        self.get_logger().info('qcar_relay_node connecting to %s (motor:%d, lidar:%d)'
                                % (qcar_ip, MOTOR_PORT, LIDAR_PORT))

    # --- /cmd_vel -> QCar motor bridge ---

    def cmd_vel_callback(self, msg):
        line = (json.dumps({'linear_x': msg.linear.x, 'angular_z': msg.angular.z}) + '\n')
        with self._motor_lock:
            if self._motor_sock is None:
                return
            try:
                self._motor_sock.sendall(line.encode('utf-8'))
            except OSError:
                self._close_motor_socket()

    def _motor_connection_loop(self):
        while not self._stop:
            try:
                sock = socket.create_connection((self.qcar_ip, MOTOR_PORT), timeout=5.0)
                sock.settimeout(SOCKET_TIMEOUT)
                with self._motor_lock:
                    self._motor_sock = sock
                self.get_logger().info('connected to QCar motor bridge')
                # No data expected from the QCar on this connection - just
                # block here (with periodic timeouts to notice self._stop)
                # so cmd_vel_callback stops sending the moment it drops.
                while not self._stop:
                    try:
                        data = sock.recv(1)
                        if data == b'':
                            break
                    except socket.timeout:
                        continue
            except OSError:
                pass
            self._close_motor_socket()
            if not self._stop:
                self.get_logger().warn('QCar motor bridge disconnected, retrying...')
                time.sleep(RECONNECT_DELAY)

    def _close_motor_socket(self):
        with self._motor_lock:
            if self._motor_sock is not None:
                try:
                    self._motor_sock.close()
                except OSError:
                    pass
                self._motor_sock = None

    # --- QCar LiDAR bridge -> /scan ---

    def _lidar_connection_loop(self):
        while not self._stop:
            try:
                sock = socket.create_connection((self.qcar_ip, LIDAR_PORT), timeout=5.0)
                sock.settimeout(SOCKET_TIMEOUT)
                self.get_logger().info('connected to QCar LiDAR bridge')
                buf = b''
                while not self._stop:
                    try:
                        chunk = sock.recv(65536)
                    except socket.timeout:
                        continue
                    if chunk == b'':
                        break
                    buf += chunk
                    while b'\n' in buf:
                        line, buf = buf.split(b'\n', 1)
                        if line.strip():
                            self._publish_scan(line)
            except OSError:
                pass
            if not self._stop:
                self.get_logger().warn('QCar LiDAR bridge disconnected, retrying...')
                time.sleep(RECONNECT_DELAY)

    def _publish_scan(self, line):
        try:
            data = json.loads(line)
        except ValueError:
            return
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min = data['angle_min']
        scan.angle_max = data['angle_max']
        scan.angle_increment = data['angle_increment']
        scan.range_min = data['range_min']
        scan.range_max = data['range_max']
        scan.ranges = [float(r) for r in data['ranges']]
        self.scan_pub.publish(scan)

    def destroy_node(self):
        self._stop = True
        self._close_motor_socket()
        super().destroy_node()


def main():
    rclpy.init()
    node = QCarRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
