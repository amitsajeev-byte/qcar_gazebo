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

## Docker deployment plan

Decision (see conversation history in `qcar_updated`): rather than downgrading
`qcar_updated`'s Humble/Nav2/MPPI stack to run natively on the QCar's onboard
ROS2 Dashing (infeasible — Nav2/MPPI/Smac didn't exist for Dashing), run a
Humble container directly on the QCar's Jetson TX2, alongside the untouched
native Dashing/Melodic install. Native Dashing is never touched; the goal is
to never need a Dashing↔Humble bridge at all.

### Phase 0 — verify on the physical QCar before writing any Dockerfile

- [ ] **JetPack/L4T version**: `cat /etc/nv_tegra_release` (or
  `apt list --installed | grep nvidia-l4t-core`). Determines whether any
  L4T-matched base image is needed at all (see CSI finding below).
- [ ] **HAL/PAL Python ABI**: check whether `quanser-devices` /
  `quanser-hardware` / `quanser-communications` (the `pal.products.qcar` /
  `pal.utilities.*` packages documented above) ship as pure-Python or contain
  compiled `.so` files tied to the onboard Python 3.6 ABI
  (`pip show -f quanser-devices`, inspect `site-packages` for `.so` files and
  their tags). This decides whether the HAL wrapper node can run *inside* the
  Humble (Python 3.10) container, or must stay native and bridge over
  localhost.
- [ ] **Free eMMC space**: `df -h /` — 32GB total already carries two ROS
  distros; point Docker's `data-root` at external storage if tight.
- [ ] **Docker present**: `docker --version` — plain `apt install docker.io`
  works fine on Ubuntu 18.04 arm64 regardless of L4T, as long as no GPU
  passthrough is needed (see below).

### CSI camera finding — confirms the L4T/Argus risk

The `references/` scripts confirm the 4 onboard cameras used in hardware
tests (`QCarCameras`, `enableFront/Back/Left/Right`) are the **CSI** type, not
USB — and the docx lab guide explicitly states CSI frames cannot be forwarded
over X11/RDP, only via Quanser's own `Probe`/`Observer` network transport.
CSI capture on Jetson normally goes through NVIDIA's proprietary
`nvarguscamerasrc`/Argus stack, which is tied to the host's specific
L4T/JetPack version — this is the one piece of the stack that genuinely needs
an L4T-matched container (e.g. an `nvcr.io/nvidia/l4t-base` variant matching
the JetPack version found in Phase 0), not a generic arm64 Ubuntu 22.04 image.
The RealSense (USB, standard UVC/depth stack) and the RPLidar A2 (USB serial)
carry none of this risk and work in any generic arm64 container.

### Container architecture (branches on Phase 0 results)

- **If** HAL/PAL is pure-Python (or Python-3.10-compatible) **and** the CSI
  cameras aren't needed for the first pass (e.g. relying on the RealSense for
  perception instead): a plain `arm64v8/ros:humble-ros-base` (or
  `osrf/ros:humble-desktop`) container works with no NVIDIA-specific tooling
  at all. This is the simplest path and should be tried first.
- **If** HAL/PAL is Python-3.6-ABI-locked, or CSI cameras are required: keep
  the hardware-facing HAL node native on the host (still Dashing/Python 3.6,
  unchanged), and put only the Nav2/MPPI/planning side in the Humble
  container, talking over `--network host` via a small bridge (plain
  socket/shared-memory relay, not a ROS2-to-ROS2 bridge — the native side
  isn't a ROS2 node in this branch). CSI frames specifically may need to stay
  on the native/L4T-matched side regardless, per the finding above, and get
  relayed into the container as plain image buffers rather than trying to
  run Argus inside a generic container.

### Build steps once architecture is chosen

- [ ] Write a `Dockerfile` installing this project's actual deps:
  `nav2_bringup`, `nav2_mppi_controller`, `nav2_smac_planner`,
  `nav2_costmap_2d`, `cartographer_ros`, `xacro`, `robot_state_publisher`,
  matching `xtensor`/`xsimd` versions, and build the custom `qcar_critics`
  plugin from `qcar_updated` inside the image.
- [ ] Drop `gazebo_ros`/`gazebo_plugins`/`rviz2` from the hardware image —
  sim/visualization-only; run rviz2 remotely from the dev PC instead, pointed
  at the container's ROS2 graph over the network.
- [ ] Map real device nodes in explicitly (`--device=/dev/ttyUSB0`, etc.)
  rather than blanket `--privileged` where possible. Motor/LIDAR access
  needing `sudo` natively (already noted in memory) likely means running the
  container as root or granting specific `--cap-add` capabilities.
- [ ] Replace `qcar_updated.launch.py`'s Gazebo spawn with a
  `qcar_hardware.launch.py` that starts `robot_state_publisher` + the HAL
  wrapper node(s) instead; `qcar_nav2.launch.py`/`qcar_slam.launch.py` should
  need little to no change since they only depend on topics
  (`/cmd_vel`, `/odom`, `/scan`, `/tf`), not on Gazebo.
- [ ] Wire real wheel-encoder odometry (via the HAL `motorEncoder`/
  `motorTach` buffers documented above) into `/odom`, replacing the Gazebo
  Ackermann plugin's simulated odometry.

### Test order

Bring the container up standalone (`ros2 topic list`) → verify HAL wrapper
publishes real sensor data → low-speed teleop via `qcar_teleop_twist.py` or
`LogitechF710` (open area, PWM/steering safety limits already in place) →
only then bring in Nav2/MPPI and send an autonomous goal.
