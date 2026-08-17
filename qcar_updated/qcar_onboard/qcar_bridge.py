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

Wire format, one JSON object per line, one direction each way on the same
connection (TCP is full-duplex - no separate port needed):
    dev PC -> QCar: {"linear_x": <float m/s equivalent>, "angular_z": <float rad/s>}
    QCar -> dev PC: {"t": <QCar time.time() at capture>, "x": <m>, "y": <m>,
                      "yaw": <rad>, "v": <m/s>, "yaw_rate": <rad/s>}

The odometry direction was added for Phase 3 (Nav2/AMCL) - AMCL looks up a
live odom->base TF at every scan callback to track motion between
localization corrections, which Phase 2 (Cartographer, pure LiDAR
scan-matching) never needed. This is open-loop dead reckoning: there's no
separate steering-angle feedback sensor in this HAL (confirmed by grepping
every reference script), so the yaw-rate integration below uses the
last-*commanded* steering angle, not a measured one - accuracy depends on
the vehicle's steering mechanical center actually matching steering=0.0.
See hardware_integration_reference.md and README.md's "Known risks" for the
steering-trim caveat this depends on.

The "t" field is this QCar's own time.time() at the moment of capture, not
a receipt-time stamp - confirmed on hardware 2026-08-06 that timestamping
at receipt time on the dev PC side (independent, uncorrelated latency on
this TCP connection vs the LiDAR one) caused AMCL to pair scans with
poses from a slightly different real-world instant, producing a
coherent rigid offset between the live scan and the map ("the LiDAR dots
move... maintaining the shape... the skeleton of the map is moving" - the
user's own precise description). qcar_relay_node.py converts this into a
dev-PC-clock-equivalent timestamp using a measured clock offset - see that
file's docstring for the full explanation and the offset measurement
procedure.

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

# Compensates for this vehicle's steering mechanical center being offset
# from steering=0.0 (wheels sit visibly left of straight when commanded to
# 0 - confirmed on hardware 2026-08-03/06). No adjustable physical linkage
# is available on this chassis, so corrected in software instead.
#
# Calibrated iteratively on hardware, 2026-08-06, first via an interactive
# static test (motor off, held candidate angles, visually judged against
# straight - -0.05 too left, -0.09 "almost correct"), then refined via a
# sequence of real straight-line driving tests (drive N meters at
# angular.z=0, measure actual lateral offset). Distance-based measurement
# is far more precise than eyeballing a stationary wheel angle - prefer it
# for any further refinement. History (some noise expected - offset also
# depends on how precisely the vehicle was aimed at the start of each
# test, not purely the trim value):
#   trim     distance   offset        direction needed
#   -0.09     4.5m      0.1m right    less negative (toward 0)
#   -0.0678   ?         drifted left  more negative (overshot the above)
#   -0.085    5.3m      0.2m right    less negative
#   -0.08     ~4.4m      no visible drift reported - ACCEPTED
# Accepted as "almost accurate" (user's call, 2026-08-06), not a perfect
# zero - if drift reappears or matters more for a future use case (e.g.
# tighter Nav2 tolerances), re-run the driving-test procedure above rather
# than adjusting from static/visual judgment alone.
STEERING_TRIM = -0.08

# PWM duty-cycle safety limits (documents/user_manual_troubleshooting.pdf):
# saturate to +/-30% magnitude, rate-limited to 100% duty-cycle change per
# second, to avoid a battery-brownout-triggered shutdown.
THROTTLE_LIMIT = 0.3
THROTTLE_RATE_LIMIT = 1.0  # duty-cycle fraction per second

# cmd_vel's linear.x is m/s; the HAL's throttle is a PWM duty-cycle fraction
# (unitless) - there's no documented conversion between the two anywhere in
# the Quanser manuals. The original guess (a flat THROTTLE_GAIN=1/3.0
# multiplier) was never calibrated and made real speed run ~2.6-2.7x
# commanded (0.3 m/s command -> ~0.8 m/s real). A naive linear correction
# (just dividing that gain down) was tried and reverted the same day: it
# pushed normal teleop speeds below the vehicle's static-friction deadband,
# so the car stopped responding to teleop entirely.
#
# Root cause: not a single linear relationship. There's a real stiction
# floor below which NO commanded speed produces motion at all, and once
# past it the real speed is very sensitive to small duty changes (a
# stick-slip signature - static friction exceeds kinetic friction, so the
# instant the car breaks free, the same duty suddenly has much more net
# force to accelerate with). Calibrated on hardware 2026-08-17, car on the
# ground with real load, by empirically sweeping fixed duty values and
# separately validating a fitted formula against 4 different commanded
# speeds end to end (2m drive-and-stop test each):
#   commanded  ->  measured real cruise speed
#   0.2 m/s        0.24-0.99x reliable - right at the stiction edge, can
#                  fail to move at all depending on run-to-run friction
#                  noise. Treat anything below ~0.25-0.3 m/s as unreliable.
#   0.3 m/s        0.31 m/s (ratio 1.05)
#   0.4 m/s        0.42 m/s (ratio 1.05)
#   0.5 m/s        0.50 m/s (ratio 1.00)
# THROTTLE_DEADBAND/THROTTLE_GAIN below implement
# duty = THROTTLE_DEADBAND + THROTTLE_GAIN * |speed|, fitted and validated
# for the 0.3-0.5 m/s range above. Not validated above 0.5 m/s - re-check
# with scripts/drive_and_log.py or scripts/check_stop_latency.py before
# trusting a higher command. See qcar_updated_stop_distance_investigation
# project memory for the full sweep/validation data.
THROTTLE_DEADBAND = 0.04
THROTTLE_GAIN = 0.0667

# Tried active braking (brief reverse-throttle pulse proportional to
# residual speed when a stop is commanded) to counter drivetrain coast -
# REJECTED, not just untuned. It's a proportional loop on velocity sign
# with no deadband and one-cycle-stale (~20ms) feedback: if the pulse
# overshoots past zero into real reverse motion, the next cycle reads a
# negative v and "corrects" with a forward push, which can itself
# overshoot - a genuine undamped hunting oscillation, not a bad gain
# value. Confirmed on hardware 2026-08-12 (twice - it was reverted once
# already before this comment, then re-tried and reproduced the same
# result): car visibly lurched forward/back repeatedly and fast, had to
# be killed manually. Do not re-add this without a real
# deadband/hysteresis or a one-shot (non-continuous) brake pulse design -
# retuning BRAKE_GAIN alone will not fix it. The car coasting for a fairly
# fixed ~2.4-2.55s after a stop command (0.60-1.20m extra travel across
# 0.2-0.5 m/s cruise, scaling with speed - see THROTTLE_GAIN comment above
# for the validated numbers) is the accepted, safe behavior for now - a
# separate, still-open problem from throttle gain calibration.

# If no command arrives for this long (or the socket disconnects), stop the
# car. Protects against a dropped WiFi link or a crashed dev-PC relay.
CMD_TIMEOUT = 0.5  # seconds

READ_RATE = 50.0  # Hz
LISTEN_PORT = 5555

# Encoder counts -> distance (documents/user_manual_system_hardware.pdf,
# derived from the drive motor's gear ratio and wheel radius):
# distance(m) = encoderCounts * (1/2880) * 0.01977
METERS_PER_COUNT = 0.01977 / 2880.0


def compute_steering(linear_x, angular_z):
    # Returns the KINEMATIC (untrimmed) steering angle - the real physical
    # wheel angle the bicycle model wants, used for odometry and the LED
    # turn-indicator logic. STEERING_TRIM must NOT be folded in here: an
    # earlier version did, and it corrupted the odometry - target_steering
    # was being reused as the "real physical angle" input to
    # Odometry.update(), so trimming it there made the dead-reckoning math
    # believe the wheels were deflected by the trim amount even while
    # driving arrow-straight (trim's whole purpose is to make the REAL
    # wheels read ~0 despite a nonzero servo command - counting that same
    # command as a real deflection is exactly backwards). Confirmed on
    # hardware 2026-08-06: with trim folded in here, a straight-line drive
    # test produced a fictitious ~30-degree odometry curve. See apply_trim()
    # below for where STEERING_TRIM actually belongs - only at the
    # car.write() call site, never upstream of it.
    #
    # A car-like steering axle can't produce a meaningful angle at a
    # standstill, so hold steering at 0 rather than divide by ~0.
    if abs(linear_x) < 1e-2:
        steer = 0.0
    else:
        steer = math.atan(WHEELBASE * angular_z / linear_x)
    return max(-MAX_STEER_CMD, min(MAX_STEER_CMD, steer))


def apply_trim(kinematic_steering):
    '''Converts a real/kinematic steering angle into the servo command that
    actually achieves it on this specific vehicle - only call this right
    before car.write(), never before odometry or anything else that cares
    about the real physical wheel angle.'''
    return max(-MAX_STEER_CMD, min(MAX_STEER_CMD, kinematic_steering + STEERING_TRIM))


class Odometry:
    '''Bicycle-model dead reckoning from encoder deltas + last-commanded
    steering angle. Reset at the start of each new relay connection - odom
    is a purely local/relative frame by ROS convention, doesn't need to
    persist across reconnects.'''

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_encoder = None

    def update(self, encoder_counts, steering, dt):
        if self.last_encoder is None:
            self.last_encoder = encoder_counts
        delta_counts = encoder_counts - self.last_encoder
        self.last_encoder = encoder_counts

        distance = delta_counts * METERS_PER_COUNT
        v = distance / dt
        yaw_rate = v * math.tan(steering) / WHEELBASE

        self.yaw += yaw_rate * dt
        self.x += distance * math.cos(self.yaw)
        self.y += distance * math.sin(self.yaw)

        return {'x': self.x, 'y': self.y, 'yaw': self.yaw, 'v': v, 'yaw_rate': yaw_rate}


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
            odom = Odometry()

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
                            except (ValueError, KeyError, TypeError):
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

                    if abs(target_linear) < 1e-3:
                        desired_throttle = 0.0
                    else:
                        desired_throttle = math.copysign(
                            THROTTLE_DEADBAND + THROTTLE_GAIN * abs(target_linear), target_linear)
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
                    car.write(current_throttle, apply_trim(target_steering), leds)

                    # Odometry uses the untrimmed/kinematic angle - see
                    # compute_steering()'s docstring for why.
                    odom_data = odom.update(float(car.motorEncoder[0]), target_steering, dt)
                    # Capture-time timestamp, not send-time - see this file's
                    # module docstring for why this matters.
                    odom_data['t'] = time.time()
                    try:
                        conn.sendall((json.dumps(odom_data) + '\n').encode('utf-8'))
                    except OSError:
                        print('dev PC relay disconnected (send failed)')
                        break
            finally:
                conn.close()
                # Lost the relay connection - stop immediately rather than
                # waiting out CMD_TIMEOUT on the next accept() loop.
                target_linear = 0.0
                target_steering = 0.0
                current_throttle = 0.0
                car.write(0.0, apply_trim(0.0), [0, 0, 0, 0, 0, 0, 0, 0])
    except KeyboardInterrupt:
        pass
    finally:
        try:
            car.write(0.0, apply_trim(0.0), [0, 0, 0, 0, 0, 0, 0, 0])
        except Exception:
            pass
        car.terminate()
        server.close()


if __name__ == '__main__':
    main()
