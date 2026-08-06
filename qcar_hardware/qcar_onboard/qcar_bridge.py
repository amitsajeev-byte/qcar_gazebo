#!/usr/bin/env python3
'''qcar_bridge.py

Minimal QCar drive bridge. Runs directly on the QCar's onboard Dashing:

    python3 qcar_bridge.py

No ROS2 involved on this side at all - native ROS2 pub/sub between this
QCar's Dashing and the dev PC's Humble was tested and found to NOT
interoperate (confirmed: raw UDP passes fine in both directions on both
arbitrary and DDS-discovery ports, but Fast-RTPS discovery itself never
completes - a protocol-version issue, not a network one). Instead, this
listens on a plain TCP socket for newline-delimited JSON drive commands from
scripts/qcar_relay_node.py on the dev PC (the only thing that still speaks
ROS2 - it bridges this socket to /cmd_vel on the Humble side).

Wire format, one JSON object per line:
    {"linear_x": <float m/s equivalent>, "angular_z": <float rad/s>}

See hardware_integration_reference.md for the HAL API this wraps and why
the ROS2-to-ROS2 approach was dropped.
'''
import json
import math
import signal
import socket
import time

from pal.products.qcar import QCar, IS_PHYSICAL_QCAR

# SIGTERM (pkill/kill's default signal) is NOT auto-converted to a catchable
# KeyboardInterrupt the way SIGINT (Ctrl+C) is - without this, a plain
# `pkill -f qcar_bridge.py` would skip the cleanup in main()'s `finally`
# block entirely (car.write(0,0,...) + car.terminate()), potentially leaving
# the motor drive in an uncertain state. See qcar_lidar_node.py for the same
# fix and the hardware incident (LiDAR left spinning after a plain pkill)
# that surfaced this, 2026-08-03.
def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt()


signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)

# Wheelbase (m): matches urdf/qcar_model.xacro hub joint origins and
# scripts/qcar_teleop_twist.py.
WHEELBASE = 0.25725

# Steering *command* range accepted by the HAL is -0.5 to 0.5 rad - distinct
# from the vehicle's physical max steering angle (0.5236 rad / 30 deg).
MAX_STEER_CMD = 0.5

# PWM duty-cycle safety limits (documents/user_manual_troubleshooting.pdf):
# saturate to +/-30% magnitude, rate-limited to 100% duty-cycle change per
# second, to avoid a battery-brownout-triggered shutdown.
THROTTLE_LIMIT = 0.3
THROTTLE_RATE_LIMIT = 1.0  # duty-cycle fraction per second

# cmd_vel's linear.x is m/s; the HAL's throttle is a PWM duty-cycle fraction
# (unitless) - there's no documented conversion between the two anywhere in
# the Quanser manuals. This is an UNCALIBRATED starting guess (1/3.0, from
# the documented 3 m/s rated max speed in
# documents/user_manual_customizing_the_qcar.pdf), not a tuned value -
# calibrate against real encoder-measured speed before trusting closed-loop
# velocity control. See hardware_integration_reference.md.
THROTTLE_GAIN = 1.0 / 3.0

# If no command arrives for this long (or the socket disconnects), stop the
# car. Protects against a dropped WiFi link or a crashed dev-PC relay.
CMD_TIMEOUT = 0.5  # seconds

READ_RATE = 50.0  # Hz
LISTEN_PORT = 5555


def compute_steering(linear_x, angular_z):
    # A car-like steering axle can't produce a meaningful angle at a
    # standstill, so hold steering at 0 rather than divide by ~0.
    if abs(linear_x) < 1e-2:
        return 0.0
    steer = math.atan(WHEELBASE * angular_z / linear_x)
    return max(-MAX_STEER_CMD, min(MAX_STEER_CMD, steer))


def main():
    if not IS_PHYSICAL_QCAR:
        import qlabs_setup
        qlabs_setup.setup()

    car = QCar(readMode=1, frequency=READ_RATE)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', LISTEN_PORT))
    server.listen(1)
    print('qcar_bridge listening on port %d (waiting for dev PC relay)' % LISTEN_PORT)

    target_linear = 0.0
    target_steering = 0.0
    current_throttle = 0.0
    last_cmd_time = time.time()
    last_loop_time = time.time()
    period = 1.0 / READ_RATE

    try:
        while True:
            conn, addr = server.accept()
            conn.settimeout(period)
            print('dev PC relay connected from', addr)
            buf = b''
            last_cmd_time = time.time()

            try:
                while True:
                    # Drain any newline-delimited JSON commands available
                    # without blocking the fixed-rate control loop below.
                    try:
                        chunk = conn.recv(4096)
                        if chunk == b'':
                            print('dev PC relay disconnected')
                            break
                        buf += chunk
                        while b'\n' in buf:
                            line, buf = buf.split(b'\n', 1)
                            if not line.strip():
                                continue
                            try:
                                cmd = json.loads(line)
                                target_linear = float(cmd['linear_x'])
                                angular_z = float(cmd['angular_z'])
                                target_steering = compute_steering(target_linear, angular_z)
                                last_cmd_time = time.time()
                            except (ValueError, KeyError):
                                print('bad command line, ignoring:', line)
                    except socket.timeout:
                        pass  # no new data this cycle - fall through to control loop

                    # Pace the actual hardware read/write to READ_RATE
                    # regardless of how fast commands arrive over the
                    # socket - a burst of queued messages shouldn't turn
                    # into a burst of car.read()/write() calls.
                    now = time.time()
                    dt = now - last_loop_time
                    if dt < period:
                        continue
                    last_loop_time = now

                    # Watchdog: stop driving if no command has arrived recently.
                    if now - last_cmd_time > CMD_TIMEOUT:
                        target_linear = 0.0
                        target_steering = 0.0

                    desired_throttle = target_linear * THROTTLE_GAIN
                    desired_throttle = max(-THROTTLE_LIMIT, min(THROTTLE_LIMIT, desired_throttle))
                    max_step = THROTTLE_RATE_LIMIT * dt
                    delta = max(-max_step, min(max_step, desired_throttle - current_throttle))
                    current_throttle += delta

                    leds = [0, 0, 0, 0, 0, 0, 1, 1]
                    if target_steering > 0.15:
                        leds[0] = 1
                        leds[2] = 1
                    elif target_steering < -0.15:
                        leds[1] = 1
                        leds[3] = 1
                    if current_throttle < 0:
                        leds[5] = 1

                    car.read()
                    car.write(current_throttle, target_steering, leds)
            finally:
                conn.close()
                # Lost the relay connection - stop immediately rather than
                # waiting out CMD_TIMEOUT on the next accept() loop.
                target_linear = 0.0
                target_steering = 0.0
                current_throttle = 0.0
                car.write(0.0, 0.0, [0, 0, 0, 0, 0, 0, 0, 0])
    except KeyboardInterrupt:
        pass
    finally:
        try:
            car.write(0.0, 0.0, [0, 0, 0, 0, 0, 0, 0, 0])
        except Exception:
            pass
        car.terminate()
        server.close()


if __name__ == '__main__':
    main()
