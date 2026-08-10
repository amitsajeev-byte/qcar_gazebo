#!/usr/bin/env python3
'''qcar_lidar_node.py

Publishes real /scan data from the QCar's onboard RPLidar A2, over a plain
TCP socket rather than ROS2 - see qcar_bridge.py's docstring for why native
ROS2 pub/sub between this QCar's Dashing and the dev PC's Humble doesn't
work. Runs directly on the QCar's onboard Dashing, as a separate process
from qcar_bridge.py on purpose (a LiDAR issue must not be able to take down
drive control, and vice versa):

    python3 qcar_lidar_node.py

scripts/qcar_relay_node.py on the dev PC connects to this, reads
newline-delimited JSON scan messages, and republishes them as a real
sensor_msgs/LaserScan on /scan for the rest of the Humble-side stack.

Wire format, one JSON object per line:
    {"t": <QCar time.time() at capture>, "angle_min": <rad>, "angle_max": <rad>,
     "angle_increment": <rad>, "range_min": <m>, "range_max": <m>, "ranges": [<m>, ...]}

The "t" field is this QCar's own time.time() at capture, not a receipt-time
stamp - see qcar_bridge.py's module docstring for why (confirmed on
hardware 2026-08-06: receipt-time stamping on two independently-latent TCP
connections caused AMCL to pair scans with stale poses, producing a
coherent rigid offset between the live scan and the map).
'''
import json
import math
import signal
import socket
import time

from pal.products.qcar import QCarLidar, IS_PHYSICAL_QCAR

# SIGTERM (pkill/kill's default signal) is NOT auto-converted to a catchable
# KeyboardInterrupt the way SIGINT (Ctrl+C) is - without this, `pkill -f
# qcar_lidar_node.py` skips the `finally: lidar.terminate()` cleanup below
# entirely, leaving the physical LiDAR motor spinning. Confirmed on hardware
# 2026-08-03: exactly this happened, recovered via a one-off
# `QCarLidar().terminate()` since QCar2_hardware_stop.py also touches
# QCar() and would have disturbed a concurrently-running qcar_bridge.py.
def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt()


signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)

NUM_MEASUREMENTS = 360
SCAN_RATE = 10.0  # Hz
LISTEN_PORT = 5556

# Matches the range limits already configured in
# qcar_updated/config/nav2/nav2_params.yaml.
RANGE_MIN = 0.15
RANGE_MAX = 12.0


def read_scan(lidar):
    capture_time = time.time()
    lidar.read()

    # myLidar.angles/distances are parallel arrays of raw samples - not
    # guaranteed already gridded to a fixed angle_min/angle_increment.
    # LaserScan requires a uniform grid, so bin each raw sample onto one
    # instead of assuming the HAL delivers them pre-gridded in order.
    #
    # The HAL's raw angle convention needs both a direction flip (CW ->
    # CCW, to match ROS's LaserScan/REP-103 convention - matches Quanser's
    # own reference script needing ax.set_theta_direction(-1) to display
    # correctly) AND a +90deg rotational offset between the LiDAR's own
    # zero-reference and the vehicle's actual front. Both confirmed on
    # hardware 2026-08-03: with only the direction flip applied, an object
    # placed at the vehicle's physical right (should read -90deg) showed up
    # at the back (+-180deg) in RViz - a quarter-turn error on top of the
    # flip, not a further mirror. -angle + pi/2 corrects both at once.
    increment = (2.0 * math.pi) / NUM_MEASUREMENTS
    ranges = [float('inf')] * NUM_MEASUREMENTS
    for angle, dist in zip(lidar.angles, lidar.distances):
        target = -angle + math.pi / 2
        normalized = math.atan2(math.sin(target), math.cos(target))
        idx = int(round((normalized + math.pi) / increment)) % NUM_MEASUREMENTS
        if RANGE_MIN <= dist <= RANGE_MAX:
            ranges[idx] = float(dist)

    return {
        't': capture_time,
        'angle_min': -math.pi,
        'angle_max': math.pi,
        'angle_increment': increment,
        'range_min': RANGE_MIN,
        'range_max': RANGE_MAX,
        'ranges': ranges,
    }


def main():
    if not IS_PHYSICAL_QCAR:
        import qlabs_setup
        qlabs_setup.setup()

    lidar = QCarLidar(
        numMeasurements=NUM_MEASUREMENTS,
        rangingDistanceMode=2,
        interpolationMode=0,
    )

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', LISTEN_PORT))
    server.listen(1)
    print('qcar_lidar_node listening on port %d (waiting for dev PC relay). '
          'Angle convention (front=0deg, direction, sample ordering) is NOT '
          'yet verified against a real obstacle placement, see '
          'hardware_integration_reference.md' % LISTEN_PORT)

    period = 1.0 / SCAN_RATE

    try:
        while True:
            conn, addr = server.accept()
            print('dev PC relay connected from', addr)
            try:
                while True:
                    start = time.time()
                    scan = read_scan(lidar)
                    line = (json.dumps(scan) + '\n').encode('utf-8')
                    conn.sendall(line)

                    elapsed = time.time() - start
                    if elapsed < period:
                        time.sleep(period - elapsed)
            except (BrokenPipeError, ConnectionResetError):
                print('dev PC relay disconnected')
            finally:
                conn.close()
    except KeyboardInterrupt:
        pass
    finally:
        lidar.terminate()
        server.close()


if __name__ == '__main__':
    main()
