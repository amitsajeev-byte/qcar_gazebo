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

# Compensates for two independent, real effects: (1) this vehicle's
# steering mechanical center being offset from steering=0.0 (wheels sit
# visibly left of straight when commanded to 0), and (2) a real steering
# GAIN error - the physical wheel turns more than the commanded angle for
# any nonzero command, not just a zero-point offset. No adjustable
# physical linkage is available on this chassis and no factory steering
# calibration curve is documented anywhere in the Quanser manuals (only
# the raw servo command range, -0.5 to 0.5 rad, and the physical max wheel
# angle, +/-0.5236 rad - checked user_manual_system_hardware.pdf and the
# QCar HAL source directly; the HAL's own steeringBias parameter is left
# at its default 0 in this file's QCar() construction, so nothing at the
# firmware level is fighting this software correction), so both are
# corrected in software instead, calibrated empirically like
# THROTTLE_GAIN/THROTTLE_DEADBAND below.
#
# 2026-08-06: STEERING_TRIM alone (no gain term) iteratively calibrated -
# first an interactive static test (motor off, held candidate angles,
# visually judged against straight), then refined via real straight-line
# driving tests (drive N meters at angular.z=0, measure actual lateral
# offset). -0.08 accepted that day as "almost accurate", no gain term
# considered/tested yet.
#
# 2026-08-24/25: revisited after the vehicle appeared to turn less
# accurately on hardware than in sim. -0.08 no longer held (drift had
# grown to ~18cm over just 1m, not the ~cm-level residual expected) -
# initial re-investigation was badly confounded by a nut physically caught
# under a wheel and a near-depleted battery (0.3 m/s command was sitting
# right at/below the current static-friction floor - see THROTTLE_GAIN
# history below for the same stiction-floor phenomenon), producing
# apparent stalls/nonlinearity that were artifacts, not real steering
# behavior - see qcar_steering_gain_nonlinearity_investigation project
# memory for the full retracted dataset, kept for the record but not to be
# reused. Clean re-test 2026-08-25 (nut removed, fresh battery, 0.4 m/s
# for stiction headroom, user re-aiming to a fixed start position/heading
# every run) via a 7-point 1m-arc sweep, each point back-solved for the
# real wheel angle implied by the measured lateral deviation:
#   cmd (rad)   measured      implied real angle (rad)
#   0.00        21.5cm L      +0.112
#   -0.05       10cm L        +0.052
#   -0.08       3.5cm L       +0.018
#   -0.085      1cm L         +0.005
#   -0.09       dead straight  0.000
#   -0.10       4cm R         -0.021
#   -0.12       12cm R        -0.062
# Clean, monotonic, no stalls/sign-flips/asymmetry - unlike the retracted
# 2026-08-24 data. Least-squares fit: implied = 1.39*cmd + 0.119, R²=0.977
# (the two largest-magnitude points sit slightly above the line - mild
# hint of extra curvature out there, not investigated further). Zero-point
# anchored to the direct -0.09 "dead straight" measurement (more precise
# than the regression's own intercept-derived zero-crossing, -0.0857,
# since the largest-magnitude points pull that estimate off slightly) -
# see apply_trim() below for how STEERING_GAIN and STEERING_TRIM combine.
# If drift or turning inaccuracy reappears, re-run this exact sweep
# (script: steering_angle_test.py, currently only in a scratch location,
# not committed to the repo - recreate from the project memory above if
# needed) rather than adjusting from a single straight-line test alone,
# since a single test can't distinguish a gain error from an offset error.
#
# Trim refined same day (2026-08-25) via 3 repeated trials each at -0.08,
# -0.085, -0.09 to average out human placement/measurement noise:
#   -0.08: 3.3, 3.2, 0.5 cm L (avg 2.33 L) | -0.085: 0, 0, 1 cm L (avg 0.33 L)
#   -0.09: 1, 0.5, 1 cm R (avg 0.83 R)
# Zero-crossing of the averaged points: -0.087 (this narrow a span isn't
# enough data to also re-fit STEERING_GAIN - kept at 1.39 from the wider
# sweep above).
STEERING_GAIN = 1.39
STEERING_TRIM = -0.087

# PWM duty-cycle safety limits (documents/user_manual_troubleshooting.pdf):
# saturate to +/-30% magnitude, rate-limited to 100% duty-cycle change per
# second, to avoid a battery-brownout-triggered shutdown.
THROTTLE_LIMIT = 0.3
THROTTLE_RATE_LIMIT = 1.0  # duty-cycle fraction per second

# FLAG 2026-09-02: added after a live hardware observation - during Nav2 retry/orientation-
# correction cycles (esp. the MPPI cusp-negotiation "searching" behavior documented in the
# qcar_updated_mppi_cusp_freeze_investigation memory), the commanded steering angle was seen
# oscillating hard left/right, spontaneously, cycle to cycle. Unlike throttle above, steering had
# NO rate limit at all before this - target_steering went straight from compute_steering() to
# car.write() every ~50ms with nothing damping a full-range reversal. Repeated full-swing
# reversals under load are a real mechanical stress risk (encoder/servo wear), independent of
# whatever is driving the oscillation upstream (Nav2/MPPI) - this is a hardware-boundary backstop,
# not a fix for the upstream cause. Value is a first guess (full +/-MAX_STEER_CMD swing, 1.0 rad,
# in 0.5s) mirroring THROTTLE_RATE_LIMIT's rough time constant above - NOT yet validated on
# hardware. Bench-test with wheels off the ground before trusting this on a real drive; if normal
# path-following now feels sluggish/late on real turns, raise this rather than removing it.
STEERING_RATE_LIMIT = 2.0  # rad per second

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
# project memory for the full sweep/validation data. FORWARD ONLY - see
# REVERSE_DEADBAND/REVERSE_GAIN below for why reverse can't reuse these.
THROTTLE_DEADBAND = 0.04
# 2026-09-01: nudged 0.0667 -> 0.070 - after adding REVERSE_DEADBAND/REVERSE_GAIN below, live
# comparison at the same 0.3 m/s command showed reverse cruising noticeably faster than forward
# (~0.40-0.44 m/s vs ~0.30-0.31 m/s). Small increase here alongside a REVERSE_GAIN reduction to
# bring the two closer together, at the user's request - not a full recalibration, a deliberate
# small symmetric nudge.
THROTTLE_GAIN = 0.070

# 2026-09-01: reverse silently reused THROTTLE_DEADBAND/THROTTLE_GAIN above (both
# forward-only-calibrated, 2026-08-17) until today - never actually validated for reverse, and
# turned out not to hold. Real symptom: "reverse takes great effort" - live-tested via
# scripts/drive_and_log.py (production cmd_vel pipeline, not a raw duty script) across many
# trials, forward vs reverse at the SAME commanded speed:
#   - Forward: consistently clean, every single trial (0.30 m/s -> ~0.30-0.31 m/s cruise,
#     0.45 m/s -> ~0.37-0.42 m/s cruise) - a stable, repeatable relationship.
#   - Reverse: highly inconsistent at identical commanded values - sometimes a clean cruise
#     close to forward's ratio, sometimes a 17-22s dead stall before suddenly breaking free to
#     near-forward speed, sometimes no stall but capped at roughly half forward's real speed the
#     whole way. This bimodal/noisy behavior (not a stable function of duty) is itself evidence
#     of a real mechanical inconsistency in this direction, not just an uncalibrated gain -
#     REVERSE_DEADBAND/REVERSE_GAIN below reduce how OFTEN/how badly this shows up, they don't
#     eliminate it. STALL_TIMEOUT below remains the real backstop for the residual risk.
#
# Fitted from the real (non-obstacle-contaminated) trial data - duty computed via the shared
# formula above, matched against each commanded speed's distance/time-averaged real speed
# (deliberately not just cherry-picked cruise-phase speed, so the stall episodes pull the fit
# toward giving reverse more duty rather than less - erring toward fewer stalls over precise
# speed-tracking when commanded speed is low):
#   duty=0.060 (0.30 m/s cmd): trials averaged ~0.111 m/s and ~0.0 m/s (total stall)
#   duty=0.070 (0.45 m/s cmd): trials averaged ~0.108, ~0.177, ~0.200 m/s
# Rounded from the resulting fit rather than kept at spurious precision, given how much
# trial-to-trial noise underlies it. Re-derive from a fresh set of drive_and_log.py trials
# (confirm clear space first, every time - see qcar_hardware_confirm_clear_space_before_driving
# project memory) if this stops matching real behavior, same as THROTTLE_GAIN's own history.
REVERSE_DEADBAND = 0.055
# 2026-09-01: nudged 0.09 -> 0.075 -> 0.065 -> 0.056 -> 0.04 across four rounds, each retested
# live at a 0.3 m/s command via drive_and_log.py peak cruise speed vs forward's peak at the same
# command (battery-dependent - see note below):
#   GAIN=0.09   -> peak ~0.42-0.44 m/s  (vs forward ~0.29 m/s, same battery)
#   GAIN=0.075  -> peak ~0.42 m/s
#   GAIN=0.065  -> peak ~0.35 m/s
#   GAIN=0.056  -> peak ~0.43 m/s AFTER a battery swap (vs forward ~0.33 m/s, same fresh
#     battery) - duty is a FRACTION of battery voltage, so a fresher/higher-voltage battery
#     delivers more real speed for the same duty, both directions - this shifted the whole
#     curve, not just reverse, and re-confirmed reverse still ran hotter than forward
#     proportionally either way (~1.2-1.3x).
#   GAIN=0.04   -> interpolated target for the fresh battery, aiming to match forward's ~0.33
#     m/s peak
# REVERSE_DEADBAND dominates the duty at this commanded speed (0.055 of ~0.065-0.078 total), so
# GAIN has outsized leverage on the remainder - small GAIN changes move peak speed a lot more
# here than the same change would for forward's much smaller deadband fraction, and the whole
# fit is battery-voltage-dependent same as THROTTLE_GAIN's own history. Deliberate iterative
# nudges, not a full re-fit each time - re-test with drive_and_log.py (confirm clear space
# first) after any further change, or after a battery swap, before trusting this value.
REVERSE_GAIN = 0.04

# Precautions from documents/user_manual_system_hardware.pdf and
# documents/user_manual_power.pdf that this file previously read the underlying HAL buffers for
# (motorCurrent, batteryVoltage) but never actually watched - added 2026-09-01 after a real
# "reverse takes great effort" hardware symptom raised the question of whether the FPGA's own
# overcurrent protection was intermittently tripping. Worth the arithmetic once: at our
# THROTTLE_LIMIT=0.3 duty cap and a nominal ~12V battery, applied voltage tops out at ~3.6V -
# comfortably under the 5V stalled-motor-damage caution below, so that specific risk already has
# real margin. But locked-rotor (v=0) current isn't bounded by that same margin - back-EMF is
# zero at a true stall, so current is just V/R = 3.6V / 0.470ohm (Table 7's terminal resistance)
# =~ 7.7A, comfortably past the FPGA's 5A/8s tier and not far from its 10A/2s tier. So a genuine
# stiction-floor stall (the exact "straining, not moving" symptom under investigation) is a
# real candidate for tripping the FPGA's protection even though we're nowhere near max duty.
#
# documents/user_manual_system_hardware.pdf, "Drive Motor and Steering Servo": onboard
# overcurrent protection trips (motor forced to Neutral/coast mode) if current sustains 5A for
# 8s, 10A for 2s, or 15A for 0.5s. Per documents/user_manual_troubleshooting.pdf ("e. The drive
# motor does not function/respond to commands"): recovery requires restarting this whole
# process (closing/reopening the HIL device), not just re-sending commands - and the QCar's own
# LCD is the authoritative way to confirm this happened ("Overcurrent" message). The checks
# below only print an early warning on this process's own console (nothing else can see this
# from the dev PC side right now) - they do not and cannot override the FPGA's own protection.
OVERCURRENT_TIERS = [(5.0, 8.0), (10.0, 2.0), (15.0, 0.5)]  # (amps, sustained seconds)
OVERCURRENT_WARN_FRACTION = 0.7  # warn at 70% of the FPGA's own sustained-time trip threshold

# documents/user_manual_power.pdf, "Low-battery and auto-shutdown": QCar shows a 'LOW BAT' LCD
# warning below 10.5V and auto-shuts-down below 10.0V (attempts a normal shutdown first, then
# force-disconnects power if that fails). Independent of anything in this script - just also
# printed here for visibility, same reasoning as the overcurrent tiers above.
BATTERY_WARN_VOLTAGE = 10.5
BATTERY_SHUTDOWN_VOLTAGE = 10.0

# documents/user_manual_system_hardware.pdf, Drive Motor section: "Holding the motor in a
# stalled position for a prolonged period at applied voltages of over 5V can result in
# permanent damage." CMD_TIMEOUT above only protects against a STALE command (nothing arriving)
# - it does nothing for a continuously-refreshed nonzero command that's failing to actually
# move the vehicle (e.g. commanded duty sitting right at/below the real stiction floor - exactly
# the symptom under investigation 2026-09-01). This is a software-side backstop for that specific
# gap: if throttle has been commanded above STALL_THROTTLE_THRESHOLD for STALL_TIMEOUT seconds
# straight with no corresponding encoder motion, cut the throttle. Deliberately conservative
# (encoder epsilon well above quadrature noise, generous timeout) - this is a safety backstop,
# not a substitute for actually fixing the underlying stiction/gain mismatch if one exists.
# 2026-09-01: originally 0.08 - a real bug, not just a conservative margin. desired_throttle =
# THROTTLE_DEADBAND + THROTTLE_GAIN*|speed| only reaches ~0.053-0.073 across the ENTIRE
# validated 0.2-0.5 m/s speed range (0.3 m/s -> ~0.06), so 0.08 could never fire at any normal
# teleop speed - confirmed live: held reverse 3-4s with the encoder never moving at all
# (odom_monitor.py showed a flat 0.000) and this check stayed silent throughout. Fixed to sit
# just above THROTTLE_DEADBAND itself instead of guessing a round number unrelated to the
# actual duty formula.
STALL_THROTTLE_THRESHOLD = 0.045  # duty fraction - above the deadband, i.e. "genuinely trying to drive"
STALL_ENCODER_EPS = 20  # encoder counts/cycle (~0.007 m/s at 50Hz) - treat as "not moving" below this
STALL_TIMEOUT = 3.0  # seconds of continuous stall before cutting throttle

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
    about the real physical wheel angle. Inverts the measured
    implied_real_angle = STEERING_GAIN*cmd + (real angle at cmd=0) relationship
    by pre-dividing by the gain, so the real wheel ends up at
    kinematic_steering, not just at kinematic_steering + a flat offset.'''
    return max(-MAX_STEER_CMD, min(MAX_STEER_CMD, kinematic_steering / STEERING_GAIN + STEERING_TRIM))


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

    # readMode=0 (immediate I/O) - was 1 (task-based I/O), which runs a
    # background acquisition task into a 100-sample ring buffer
    # (frequency*2) from the moment this object is constructed, independent
    # of when car.read() actually starts being called. Since this object is
    # created before the listening socket even opens, and car.read() isn't
    # called until a relay connects, the buffer fills and starts
    # overwriting during that gap - once reads begin at the matched 50Hz
    # rate, they get permanently stuck ~100 samples (~2.0s) behind
    # real-time, since the backlog never drains. Confirmed on hardware
    # 2026-08-18: this exactly explains the "~2.1s stiction delay" and
    # "~2.4s stop latency" found in earlier THROTTLE_GAIN/stop-latency
    # investigation - both are the same fixed reporting lag, not real
    # vehicle physics. Immediate I/O reads current hardware state directly,
    # no task/buffer involved. See qcar_updated_stop_distance_investigation
    # project memory for the full history.
    car = QCar(readMode=0, frequency=READ_RATE)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', LISTEN_PORT))
    server.listen(1)
    print('qcar_bridge listening on port %d (waiting for dev PC relay)' % LISTEN_PORT)

    target_linear = 0.0
    target_steering = 0.0
    current_throttle = 0.0
    current_steering = 0.0
    last_cmd_time = time.time()
    last_loop_time = time.time()
    period = 1.0 / READ_RATE

    # State for the precaution checks documented above (OVERCURRENT_TIERS/STALL_* /
    # BATTERY_*_VOLTAGE) - reset per relay connection alongside odom, since a fresh connection
    # is a natural "start clean" point same as the odom frame already resetting there.
    last_stall_encoder = None
    stall_start_time = None
    overcurrent_tier_start = [None] * len(OVERCURRENT_TIERS)
    overcurrent_tier_warned = [False] * len(OVERCURRENT_TIERS)
    battery_warn_state = None  # None / 'warn' / 'shutdown' - avoid re-printing every cycle

    try:
        while True:
            conn, addr = server.accept()
            conn.settimeout(period)
            print('dev PC relay connected from', addr)
            buf = b''
            last_cmd_time = time.time()
            odom = Odometry()
            last_stall_encoder = None
            stall_start_time = None
            overcurrent_tier_start = [None] * len(OVERCURRENT_TIERS)
            overcurrent_tier_warned = [False] * len(OVERCURRENT_TIERS)
            battery_warn_state = None

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
                    except OSError:
                        # Abrupt disconnect (e.g. the relay process killed rather than closed
                        # cleanly) raises ConnectionResetError here instead of the b'' empty-recv
                        # case above - previously unhandled, crashed the whole process (confirmed
                        # on hardware 2026-09-01: killing a stale relay process while a new one
                        # started up hit exactly this, took the bridge down until manually
                        # restarted). Mirrors the send-side OSError handling below.
                        print('dev PC relay disconnected (recv failed)')
                        break

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

                    car.read()

                    # Stall backstop (see STALL_* comment above) - based on the PREVIOUS cycle's
                    # write, since car.read() reflects state up to now, before this cycle's
                    # car.write() happens further down.
                    now_encoder = float(car.motorEncoder[0])
                    if last_stall_encoder is None:
                        last_stall_encoder = now_encoder
                    moving = abs(now_encoder - last_stall_encoder) > STALL_ENCODER_EPS
                    last_stall_encoder = now_encoder

                    if abs(current_throttle) > STALL_THROTTLE_THRESHOLD and not moving:
                        if stall_start_time is None:
                            stall_start_time = now
                        elif now - stall_start_time > STALL_TIMEOUT:
                            print('STALL: throttle %.2f commanded for >%.1fs with no encoder '
                                  'motion - cutting throttle (see user_manual_system_hardware.pdf '
                                  'stalled-motor caution)' % (current_throttle, STALL_TIMEOUT))
                            target_linear = 0.0
                            current_throttle = 0.0
                            stall_start_time = None
                    else:
                        stall_start_time = None

                    # Overcurrent early warning - mirrors the FPGA's own tiers, see
                    # OVERCURRENT_TIERS comment above. Console-only, cannot override the FPGA.
                    current_amps = abs(float(car.motorCurrent))
                    for i, (amps_thresh, seconds_thresh) in enumerate(OVERCURRENT_TIERS):
                        if current_amps >= amps_thresh:
                            if overcurrent_tier_start[i] is None:
                                overcurrent_tier_start[i] = now
                                overcurrent_tier_warned[i] = False
                            elif (not overcurrent_tier_warned[i] and now - overcurrent_tier_start[i]
                                    > seconds_thresh * OVERCURRENT_WARN_FRACTION):
                                print('WARNING: motor current %.1fA approaching the %.0fA/%.1fs '
                                      'overcurrent tier - FPGA may force Neutral mode soon '
                                      '(check QCar LCD)' % (current_amps, amps_thresh, seconds_thresh))
                                overcurrent_tier_warned[i] = True
                        else:
                            overcurrent_tier_start[i] = None
                            overcurrent_tier_warned[i] = False

                    battery_v = float(car.batteryVoltage)
                    if battery_v < BATTERY_SHUTDOWN_VOLTAGE and battery_warn_state != 'shutdown':
                        print('WARNING: battery %.2fV below the %.1fV auto-shutdown threshold' %
                              (battery_v, BATTERY_SHUTDOWN_VOLTAGE))
                        battery_warn_state = 'shutdown'
                    elif (BATTERY_SHUTDOWN_VOLTAGE <= battery_v < BATTERY_WARN_VOLTAGE
                            and battery_warn_state is None):
                        print('WARNING: battery %.2fV below the %.1fV LOW BAT threshold' %
                              (battery_v, BATTERY_WARN_VOLTAGE))
                        battery_warn_state = 'warn'
                    elif battery_v >= BATTERY_WARN_VOLTAGE:
                        battery_warn_state = None

                    if abs(target_linear) < 1e-3:
                        desired_throttle = 0.0
                    elif target_linear > 0:
                        desired_throttle = THROTTLE_DEADBAND + THROTTLE_GAIN * target_linear
                    else:
                        desired_throttle = -(REVERSE_DEADBAND + REVERSE_GAIN * abs(target_linear))
                    desired_throttle = max(-THROTTLE_LIMIT, min(THROTTLE_LIMIT, desired_throttle))
                    max_step = THROTTLE_RATE_LIMIT * dt
                    delta = max(-max_step, min(max_step, desired_throttle - current_throttle))
                    current_throttle += delta

                    # STEERING_RATE_LIMIT backstop (see FLAG comment above) - same pattern as
                    # throttle above, applied to the kinematic (untrimmed) angle so a hard
                    # left/right flip in target_steering gets damped into a bounded sweep instead
                    # of an instant full-range servo reversal.
                    max_steer_step = STEERING_RATE_LIMIT * dt
                    steer_delta = max(-max_steer_step, min(max_steer_step, target_steering - current_steering))
                    current_steering += steer_delta

                    leds = [0, 0, 0, 0, 0, 0, 1, 1]
                    if current_steering > 0.15:
                        leds[0] = 1
                        leds[2] = 1
                    elif current_steering < -0.15:
                        leds[1] = 1
                        leds[3] = 1
                    if current_throttle < 0:
                        leds[5] = 1

                    car.write(current_throttle, apply_trim(current_steering), leds)

                    # Odometry uses the untrimmed/kinematic angle - see compute_steering()'s
                    # docstring for why - and current_steering specifically (not target_steering)
                    # now that STEERING_RATE_LIMIT above means the two can genuinely differ: using
                    # the instantaneous target here would tell odometry the wheel turned further/
                    # faster than what was actually sent to the servo this cycle.
                    odom_data = odom.update(float(car.motorEncoder[0]), current_steering, dt)
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
                current_steering = 0.0
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
