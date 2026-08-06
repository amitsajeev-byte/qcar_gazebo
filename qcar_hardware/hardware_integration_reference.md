# QCar 2 Hardware Integration Reference

Notes derived from `references/` — the Quanser-provided Python hardware test
scripts and `QCar2_Hardware Tests.docx` lab guide extracted from the physical
QCar 2. This documents the actual HAL/PAL API surface and deployment model
observed in those files, for use when writing real ROS2 nodes against the
hardware (see [[qcar_hardware_integration_overview]] in memory for the
OS/ROS-version context these fit into).

## Deployment model (from the docx lab guide)

Two independent hardware-test tracks exist, MATLAB/Simulink and Python — both
work on top of the same underlying HAL. The Python track is the relevant one
for this project:

1. Power on with a charged battery, ping the QCar 2 to confirm connectivity
   (IP shown on the onboard LCD screen).
2. Deploy Python scripts to the QCar 2 target and run them there directly
   (copy + run over SSH/PuTTY — matches the scp/PYTHONPATH workflow already
   noted in memory, no cross-compile step).
3. There is also a **Simulink real-time (RT) model** deployment path
   (`Qcar2_harware_initial_test.rt-linux_qcar2`, a compiled aarch64 ELF binary
   in `references/`) used only once, as an initial system-level smoke test
   before touching any Python/ROS work — deployed via Simulink's "Run on
   Target" with `Target URI = tcpip://192.168.2.xxx:17000` and model args
   `-d /tmp -uri tcpip://192.168.2.xxx:17001` (`xxx` = the LCD-displayed
   octet). It exercises all subsystems (motors, LEDs, sensors) through an
   on-LCD menu navigated with the vehicle's three left buttons — not
   something this project's ROS2 code needs to interact with, just a
   go/no-go hardware health check.

## `pal.products.qcar` — core vehicle I/O

The primary class is `QCar` (`pal/products/qcar.py`), used as a context
manager:

```python
with QCar(readMode=1, frequency=sampleRate) as myCar:
    myCar.read()
    myCar.write(throttle, steering, LEDs)
```

- **`read()`** populates buffered sensor attributes on the instance —
  observed in use: `batteryVoltage`, `motorCurrent`, `motorEncoder`,
  `motorTach`, `accelerometer`, `gyroscope`. (The docstring in
  `QCar2_hardware_test_basic_io.py` also references a `motorTach` and notes
  "see the QCar class definition for other sensor buffers" — i.e. this list
  from the test script is illustrative, not exhaustive.)
- **`write(throttle, steering, LEDs)`** — `throttle`/`steering` are floats
  (radians for steering; the test script drives both as small sinusoids,
  ±0.1 throttle / ±0.3 steering, well inside the ±0.5 rad steering /
  ±30% PWM safety limits already recorded in memory). `LEDs` is an 8-element
  array (`np.array([...])`, 0/1 per element) — indices observed in use:
  `0,2` = left turn indicators, `1,3` = right turn indicators, `5` = reverse
  lamp, `6,7` = always-on (headlamps/tail lamps in the test script's default).
- **Positive steering → left turn** (front wheels steer left, counterclockwise
  rotation per the docx) — a sign-convention worth checking against this
  project's URDF axis direction (see the wheel-axis bug already found in
  [[qcar_ackermann_migration]] — the same left/right sign confusion pattern
  is worth double-checking here too, this time between HAL sign convention
  and the URDF's, not within the URDF itself).
- **`myCar.terminate()`** must be called on shutdown to release the DAQ —
  `QCar2_hardware_stop.py` exists specifically as a standalone recovery
  script (`QCar().terminate()` + `QCarLidar().terminate()`) for when a prior
  run crashed without cleanup and left the motor drive or LIDAR still active.
  Any ROS2 node wrapping this HAL needs the equivalent cleanup in its
  shutdown path (`try/finally` or a destructor), not just relying on process
  exit.
- **`IS_PHYSICAL_QCAR`** flag — imported from `pal.products.qcar` in every
  script. When false (i.e. running against Quanser's QLabs simulator instead
  of real hardware), scripts additionally call `qlabs_setup.setup()`. This is
  the flag to branch on if this project ever needs code that runs unmodified
  against both QLabs and the physical car.

## `pal.products.qcar.QCarLidar` — RPLidar A2

```python
myLidar = QCarLidar(
    numMeasurements=1000,
    rangingDistanceMode=2,
    interpolationMode=0,
)
myLidar.read()
# myLidar.angles, myLidar.distances — parallel arrays, polar form
myLidar.terminate()
```

- 0° is the front of the vehicle; **scan direction is counterclockwise**
  (`ax.set_theta_direction(-1)` in the test script — needed to get a polar
  plot to visually match, implying the raw angle data increases clockwise
  and must be flipped for display... actually confirm sign convention
  empirically before wiring into a `/scan` publisher, don't assume from the
  plotting code alone).
- No ROS2 message translation happens in this HAL layer at all — a real
  `/scan` (`sensor_msgs/LaserScan`) publisher node will need to be written
  from scratch around `myLidar.read()`, converting `angles`/`distances` into
  `angle_min/max/increment` + `ranges[]`.

## `pal.products.qcar.QCarCameras` — 4x CSI cameras

```python
cameras = QCarCameras(enableBack=True, enableFront=True,
                       enableLeft=True, enableRight=True)
flags = cameras.readAll()          # per-camera success flags
cameras.csi[i].imageData           # per-camera image buffer
cameras.terminate()
```

- Camera order/ID convention (confirmed in the docx, matches
  `QCar2_hardware_test_csi_cameras_probe.py`'s loop): **0 = right, 1 = rear,
  2 = front, 3 = left**. Easy to get backwards — worth hardcoding a
  named-constant mapping rather than raw indices anywhere this is consumed.
- **CSI images cannot be displayed/forwarded remotely via X11 forwarding or
  RDP** (explicitly called out in the docx) — the only supported way to view
  them off-device is Quanser's own `pal.utilities.probe` pub/sub pair:
  `Probe(ip=ipHost)` runs on the QCar and pushes frames over the network,
  `Observer()` runs on the remote/dev machine and receives+displays them.
  This is Quanser's own transport, **not ROS2** — if this project wants CSI
  frames in ROS2 (e.g. as `sensor_msgs/Image` for use in Nav2/perception), a
  bridging node needs to be written; the `Probe`/`Observer` pair is only a
  manual-viewing convenience, not something to route real image data through
  in production. This confirms the CSI-camera-needs-Argus/L4T-passthrough
  concern raised earlier for the Docker plan — worth checking during Phase 0
  whether these 4 cameras are the CSI type referenced here.

## `pal.products.qcar.QCarRealSense` — Intel RealSense

```python
with QCarRealSense(mode='IR') as myCam:            # or mode='RGB, Depth'
    myCam.read_IR()      # -> imageBufferIRLeft, imageBufferIRRight
    myCam.read_RGB()      # -> imageBufferRGB
    myCam.read_depth(dataMode='PX')  # -> imageBufferDepthPX (per-pixel units)
```

- Mode string (`'IR'` vs `'RGB, Depth'`) determines which `read_*` buffers are
  valid — the two test scripts each only initialize one mode and only call
  the matching read method(s). Calling e.g. `read_RGB()` on an `'IR'`-mode
  instance is untested by these scripts and shouldn't be assumed to work.
- `dataMode='PX'` on `read_depth` returns depth already scaled for direct
  pixel display (test script divides by a `max_distance` in meters purely for
  `cv2.imshow` normalization) — needs checking against the HAL docs for
  whether other `dataMode` values return raw sensor units instead, if this
  project needs metric depth for costmap/perception use rather than display.
- Unlike CSI, this is a standard USB RealSense — no CSI/Argus L4T dependency,
  lower risk for the Docker container plan.

## `pal.utilities.gamepad.LogitechF710`

```python
gpad = LogitechF710()
new = gpad.read()   # True if new data arrived this poll
gpad.leftJoystickX/Y, gpad.rightJoystickX/Y, gpad.trigger
gpad.buttonA/B/X/Y, gpad.buttonLeft/Right (=LB/RB)
gpad.up/down/left/right   # D-pad
gpad.terminate()
```

- Explicitly noted in the script's docstring: **button index/mapping may
  differ between Windows and Linux** for the same physical gamepad — don't
  port button-mapping constants across platforms without re-verifying on the
  QCar's actual Ubuntu 18.04 environment.
- Only useful for manual teleop/testing on this project, not something the
  autonomous stack depends on — but a reasonable stand-in for
  `qcar_teleop_twist.py` when testing directly on hardware without a network
  link to a dev-PC joystick.

## Common patterns across all scripts (worth carrying into any ROS2 wrapper)

- Every hardware handle (`QCar`, `QCarLidar`, `QCarCameras`, `QCarRealSense`,
  `LogitechF710`) has an explicit `.terminate()` and every long-running
  script wraps its main loop in `try/except KeyboardInterrupt/finally:
  <handle>.terminate()`. A ROS2 node wrapping any of these should call
  `.terminate()` from its shutdown/destructor path, not rely on the process
  just exiting — matches why `QCar2_hardware_stop.py` exists as a standalone
  recovery script in the first place.
- Sample-rate loops are hand-rolled with `time.time()`/`time.sleep()`
  (e.g. gamepad script computes `sleep_time` to hit a fixed `sampleRate`)
  rather than any ROS2 timer — expected, since these are pre-ROS2 Quanser
  reference scripts, but means the actual polling-loop timing behavior needs
  re-implementing with `rclpy` timers when wrapped as a node, not copy-pasted
  as-is.

## Deployment plan: TCP relay (native ROS2-to-ROS2 confirmed broken)

A Humble Docker container on the QCar's Jetson TX2 was considered first
(specifically to avoid a Dashing↔Humble ROS2 bridge), but was **ruled out: the
QCar's 32GB eMMC is too limited** for a Humble container image + workspace
alongside the existing native Dashing/Melodic installs. Not being pursued.

The next approach tried was native ROS2-to-ROS2: `qcar_bridge.py`/
`qcar_lidar_node.py` as real `rclpy` nodes on the QCar's Dashing, publishing/
subscribing topics the dev PC's Humble side would use directly.
**This was tested on real hardware (2026-08-03) and does not work:**

- `ros2 run demo_nodes_cpp talker` (QCar, Dashing, `ROS_DOMAIN_ID=77`) never
  reached `ros2 run demo_nodes_py listener` (dev PC, Humble, same domain ID),
  reproducibly across 3 separate attempts, talker confirmed actively
  publishing throughout each one.
- Before concluding this was a protocol issue, the network was ruled out
  first: `nc -u` (plain UDP, no ROS2 involved) passed cleanly in both
  directions, on an arbitrary port (15001/15002) *and* on the exact Fast-RTPS
  metatraffic port Dashing's talker actually bound (26650, matches the
  standard `7400 + 250*domain_id` formula for `ROS_DOMAIN_ID=77`, confirmed
  via `ss -ulnp` on the QCar). No firewall on either host either - the
  QCar's `iptables` is empty/ACCEPT, and the dev PC's `ufw` reports
  `ENABLED=no`.
- An explicit unicast-peers `FASTRTPS_DEFAULT_PROFILES_FILE` XML profile
  (bypassing multicast SPDP entirely, listing both hosts' IPs directly under
  `<rtps><builtin><initialPeersList>`) was tried too, applied identically on
  both sides. Still no discovery.

Conclusion: the transport is fine, but Fast-RTPS discovery itself doesn't
complete between Dashing's (2019) and Humble's (2022) bundled versions - a
genuine protocol-version incompatibility, not a network/firewall/multicast
problem. **Don't re-attempt native ROS2 pub/sub between these two specific
distros without a new reason to believe something changed** (e.g. if either
side's RMW gets upgraded independently of the distro).

**Current architecture**: keep the full Nav2/MPPI stack on the dev PC
(Humble) exactly as-is, and bridge to the QCar over a plain TCP socket
instead of ROS2. Two independent scripts on the QCar's onboard Dashing -
`qcar_onboard/qcar_bridge.py` and `qcar_onboard/qcar_lidar_node.py` - each
run their own TCP server (ports 5555 and 5556) speaking a tiny
newline-delimited-JSON protocol, with **no `rclpy`/ROS2 involved on the QCar
side at all**. On the dev PC, `scripts/qcar_relay_node.py` is a normal ROS2
Humble node (built via this package's `colcon build`, unlike the `qcar_onboard/`
scripts) that connects to both as a TCP client: subscribes `/cmd_vel` and
forwards it over the socket to `qcar_bridge.py`, and republishes
`qcar_lidar_node.py`'s stream as `sensor_msgs/LaserScan` on `/scan`. Kept as
two separate onboard processes on purpose - a LiDAR issue must not be able
to take down drive control, and vice versa. `qcar_onboard/` stays deliberately
**not** part of this package's `colcon build`; the files get copied onto the
QCar (scp/WinSCP) and run directly there as plain scripts (`python3
qcar_bridge.py`, `python3 qcar_lidar_node.py`), no launch file, matching
Quanser's own documented workflow. No `/odom`, `/imu`, or TF yet - those get
added in Phase 2 (SLAM) once plain teleop + LiDAR visualization is proven
safe on real hardware. Camera is explicitly out of scope for this project
(LiDAR-only), not deferred.

`qcar_relay_node.py` runs its QCar-motor and QCar-LiDAR TCP connections on
two background threads (each with its own reconnect-with-2s-delay loop,
independent of the other), separate from the `rclpy` executor thread that
handles the `/cmd_vel` subscription callback and `/scan` publishing calls -
`rclpy` publishers are thread-safe for this. `qcar_bridge.py`'s control loop
explicitly paces `car.read()`/`car.write()` to `READ_RATE` (50Hz) regardless
of how fast JSON command lines arrive over the socket, so a burst of queued
commands can't turn into a burst of uncontrolled-rate hardware writes.

`qcar_lidar_node.py` bins raw `angles`/`distances` samples from the HAL onto
a uniform `LaserScan`-shaped grid before sending (the HAL doesn't guarantee
pre-gridded output). The angle convention (front=0deg, scan direction,
sample ordering) was carried over unverified from the earlier ROS2-based
design through several iterations of this file - **verified and fixed
against a real obstacle placement on hardware, 2026-08-03** (see "Confirmed
on hardware" section below for the full story). `qcar_relay_node.py` just
passes the JSON fields straight through into the `LaserScan` message without
altering them, so the correction has to live in `qcar_lidar_node.py` itself.

`qcar_bridge.py` includes two fixes found during a static review before this
hardware access existed, carried forward unchanged into the TCP version:
- **Command watchdog**: the very first version of this bridge had no timeout
  on `/cmd_vel` - if the topic went stale (network drop, a crashed publisher,
  or simply an idle `qcar_teleop_twist.py` terminal, which only publishes on
  keypress) the car would keep driving at the last received command forever.
  Fixed with a 0.5s `CMD_TIMEOUT` that zeroes throttle/steering if no message
  has arrived recently.
- **Reverse-steering bug**: the original inversion used
  `atan2(WHEELBASE * angular_z, linear_x)`, which is only equivalent to the
  intended `atan(WHEELBASE * angular_z / linear_x)` for `linear_x > 0`. For
  negative `linear_x` (reverse) with any nonzero `angular_z`, `atan2` lands
  near ±π instead of near 0, which after clamping saturates steering to full
  lock (±0.5 rad) for any small reverse-turn command instead of a small
  angle. Fixed by using `atan(...)` directly, guarded by the existing
  near-zero-`linear_x` check.

## Confirmed on hardware, 2026-08-03: first end-to-end TCP relay test

Full chain tested live: `qcar_teleop`-style `ros2 topic pub /cmd_vel` (dev PC)
→ `qcar_relay_node.py` → TCP → `qcar_bridge.py`/`qcar_lidar_node.py` (QCar) →
real hardware, and back for `/scan`.

- **`/scan` fully validated end-to-end.** Steady ~10Hz on the dev PC's real
  `/scan` topic, correct `frame_id: lidar`, plausible indoor ranges
  (~4.1m). The full relay pipeline (HAL → JSON over TCP →
  `sensor_msgs/LaserScan`) works as designed.
- **Steering confirmed working correctly** - commanded `angular.z: 0.1` with
  `linear.x: 0.1` produced visible steering deflection, and it returned to
  center on its own once publishing stopped (the 0.5s command watchdog
  firing as intended).
- **`THROTTLE_GAIN = 1/3` is too conservative to move the car from a
  standstill at low `cmd_vel` values.** `linear.x: 0.1` → 3.3% duty cycle
  produced *no* wheel rotation; `linear.x: 0.3` → 10% duty cycle (matching
  the amplitude used in the known-working `QCar2_hardware_test_basic_io.py`
  direct-hardware test) *did* spin the wheels. Since steering and throttle
  go through the same `car.write()` call, this rules out "command didn't
  arrive" - it's specifically that 3.3% duty is below the threshold needed
  to overcome the drivetrain's static friction from rest. Real calibration
  (not just a documented gap) still needed before trusting `cmd_vel` values
  Nav2 would actually send.
- **SIGTERM cleanup bug found and fixed.** `pkill -f qcar_lidar_node.py` (to
  stop testing) killed the process without running its `finally:
  lidar.terminate()` cleanup - Python does not auto-convert SIGTERM into a
  catchable exception the way it does SIGINT (Ctrl+C). Left the physical
  LiDAR motor spinning after the process was gone and the port no longer
  listening. Recovered via a one-off `QCarLidar().terminate()` rather than
  the vendor's `QCar2_hardware_stop.py`, since that script also touches
  `QCar()` and would have disturbed the concurrently-running `qcar_bridge.py`'s
  open motor handle. Both `qcar_bridge.py` and `qcar_lidar_node.py` now
  install a `SIGTERM` handler that raises `KeyboardInterrupt`, routing
  through the same existing cleanup path regardless of which signal stops
  them. Fixed and redeployed to the QCar (hash-verified matching). The
  running `qcar_bridge.py` instance was then cleanly stopped with `SIGINT`
  (which it already handled correctly even pre-fix) and restarted from the
  corrected file - confirmed reconnected by `qcar_relay_node.py`'s log.

## Confirmed on hardware, 2026-08-03: Phase 1 (teleop) complete

With the fixed `qcar_bridge.py` running and `qcar_relay_node.py` bridging
it, the dev PC's own `qcar_hardware` colcon install turned out to be stale
(built before this session's source edits - confirmed via `diff` against
the installed copy of `qcar_teleop_twist.py`, which was missing the
`RMW_IMPLEMENTATION` pin and other changes). Rebuilt with
`colcon build --packages-select qcar_hardware --symlink-install`, which
also fixes this going forward (symlinked install, no more copy-goes-stale
risk). `qcar_relay_node.py` wasn't in the installed executables at all yet
either pre-rebuild, though this didn't block testing since it was being run
directly via `python3` throughout.

Also worth remembering: `qcar_relay_node.py` was launched with
`ROS_DOMAIN_ID=77` explicitly - any dev-PC-side ROS2 process meant to reach
it (`qcar_teleop_twist.py`, later Nav2/SLAM) needs the same domain ID set,
or its publishers/subscribers simply won't discover the relay node at all
(different `ROS_DOMAIN_ID` values are a full DDS-level network partition,
not a soft filter).

**`ros2 run qcar_hardware qcar_teleop_twist.py` (dev PC, `ROS_DOMAIN_ID=77`)
successfully drove the real QCar's motor and steering through the full TCP
relay chain.** Phase 1 is complete.

Two more real, source-grounded facts used by `qcar_bridge.py` worth keeping
in mind:
- **Throttle is a PWM duty-cycle fraction, not m/s** (confirmed in
  `documents/user_manual_troubleshooting.pdf`: saturate to ±0.3 magnitude,
  rate-limit to 100% duty/s). There's no documented linear formula from
  `cmd_vel`'s `linear.x` (m/s) to duty fraction anywhere in the Quanser
  manuals - `qcar_bridge.py`'s `THROTTLE_GAIN` constant is an
  **uncalibrated starting guess** (`1/3.0`, from the documented 3 m/s rated
  max speed in `user_manual_customizing_the_qcar.pdf`), not a tuned value -
  now with a hardware data point above showing it's too conservative at low
  `cmd_vel` magnitudes specifically. (This got accidentally dropped
  entirely - `linear_x` fed straight into the throttle clamp with no gain -
  in an early draft of the TCP-relay rewrite; caught and restored before
  deployment.) Calibrate by driving at a few fixed duty values and measuring
  real speed via the encoder before trusting closed-loop nav2 velocity
  commands on hardware.
- **Encoder-counts-to-distance conversion is documented exactly**:
  `documents/user_manual_system_hardware.pdf` gives
  `distance(m) = encoderCounts * (1/2880) * 0.01977` (derived from the drive
  motor's gear ratio and wheel radius) - not used yet since `qcar_bridge.py`
  doesn't publish `/odom`, but this is the formula to use when odometry gets
  added back. There's no separate steering-angle feedback in this HAL
  (confirmed by grepping every reference script), so any odometry built on
  this will inherently be open-loop for yaw, using the last-*commanded*
  steering angle rather than a measured one.

When odometry/TF get added back, note this repo's own `qcar_gazebo.launch.py`
has a `disable_odom_tf` argument specifically so Cartographer can be the sole
`odom`→`base` publisher - a hardware bridge that also broadcasts that
transform unconditionally would fight Cartographer over the same TF edge.
Worth an equivalent toggle (or just not broadcasting it from the bridge at
all) when that day comes.

Cameras are not part of this project's plan (LiDAR-only per the architecture
decision in `README.md`) - kept here only as a note in case that ever
changes: the `references/` scripts confirm the 4 onboard cameras
(`QCarCameras`, `enableFront/Back/Left/Right`) are the **CSI** type, not USB,
and the docx lab guide states CSI frames can't be forwarded over X11/RDP -
only via Quanser's own `Probe`/`Observer` network transport (not ROS2). CSI
capture on Jetson normally goes through NVIDIA's proprietary Argus stack,
tied to the host's specific JetPack/L4T version.

## Confirmed on hardware, 2026-08-03: RViz visualization + LiDAR angle-convention fix

`launch/qcar_visualize.launch.py` was added for viewing the real robot +
live `/scan` without needing Gazebo or `/odom`: just
`robot_state_publisher` (parses `urdf/qcar_model.xacro`, no
`disable_odom_tf` arg needed - grepped the whole `urdf/` tree, that arg
isn't actually referenced anywhere in the xacro despite
`qcar_gazebo.launch.py` passing it, so it's a harmless unused leftover) +
`joint_state_publisher` (feeds default/zero joint values so
`robot_state_publisher` can compute the full TF tree, including all the
*fixed*-joint sensor links like `base → lidar` - it doesn't need real
encoder feedback for that) + `rviz2`. `rviz/qcar.rviz` needed **zero
changes** - it already had `Fixed Frame: base`, a `RobotModel` display on
`/robot_description`, and a `LaserScan` display on `/scan` from back when
this repo was sim-only. Using `base` rather than `odom`/`map` as the fixed
frame is what makes this work without any odometry from the QCar at all -
`robot_state_publisher` supplies every sensor link as a static-from-`base`
transform straight from the URDF's fixed joints.

### LiDAR angle-convention bug: two stacked errors, not one

First look after bringing this up: robot model and scan dots both
rendered, but an object physically in front of the car appeared on the
wrong side. This caveat had been carried unverified through every version
of the LiDAR bridge in this repo (ROS2-native, then TCP) - here's what it
actually took to nail down, as a worked example for next time something
like this comes up:

1. **First hypothesis (wrong on its own): pure CW-vs-CCW mirror.** Matches
   Quanser's own `QCar2_hardware_test_rp_lidar_a2.py` needing
   `ax.set_theta_direction(-1)` to plot correctly - reasonable prior that
   the raw HAL angle just increases the opposite rotational direction from
   ROS's CCW `LaserScan` convention (REP-103). Fix applied: negate the
   angle before wrapping (`atan2(-sin(a), cos(a))` instead of
   `atan2(sin(a), cos(a))`).
2. **Result after that fix: the error didn't go away, it mirrored** - an
   object physically on the car's right now showed up at the *back* of the
   rendered model, not the left (where the pre-fix error had put it) and
   not the right (where it should be). A pure direction flip cannot produce
   that on its own - flipping direction leaves the front/back axis alone
   and only swaps left/right, so "right showed up at back" (a quarter-turn,
   not a mirror) means a *second*, independent error was stacked on top:
   roughly a 90° rotational offset between the LiDAR's own zero-reference
   and the vehicle's actual front.
3. **How it was actually solved**: rather than stacking another guess on
   top of an already-wrong mental model, used one precise, repeatable data
   point - object placed at a *known* physical bearing (dead abeam, the
   car's right side, which in the vehicle body frame per
   `documents/user_manual_system_hardware.pdf`'s `{B}` convention is -90°:
   `x` forward, `y` left, so right is `-y`) - and solved directly for what
   correction the *current* (already-negated) code's output needed to
   match it, rather than re-deriving from raw HAL semantics blind. Current
   code showed that known -90° object at +-180° (back); the fix is
   whatever rotates +-180° to -90°, i.e. +90°. Final formula:
   `target = -angle + pi/2`, then wrap `target` into `(-pi, pi]` via
   `atan2(sin(target), cos(target))`.
4. **Verified correct afterward**: right stays right, front stays front,
   confirmed by the user directly against the rendered robot model (not
   raw screen left/right, which depends on which way the RViz camera
   happens to be facing - using the model's own rendered orientation as
   ground truth is what made this verification actually trustworthy,
   versus the first, looser "seemed like it was on the left" report that
   helped find the bug but wasn't precise enough to fix it correctly in
   one step).

Redeployed and hash-verified per the usual process each iteration. This is
now a **closed, confirmed-correct** finding, not an open caveat - if this
ever regresses (e.g. a different physical LiDAR unit, a repositioned
mount), rederive from a real obstacle test the same way, don't assume the
old `+ pi/2` constant still applies.

### Shutdown quirk noticed while closing out this session

`rviz2` segfaulted (SIGSEGV) on shutdown when the `qcar_visualize.launch.py`
process group got `SIGINT` - a known, harmless Qt/OpenGL exit-path quirk in
rviz2, not specific to this project. `ros2 launch`'s own shutdown handling
still works fine; just confirm the process is actually gone afterward
(it can be picked up by `apport`, Ubuntu's crash reporter, which is also
safe to kill once rviz2 itself is confirmed dead).
