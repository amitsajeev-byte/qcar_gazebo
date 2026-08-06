# qcar_hardware

Target home for the full real-hardware QCar stack (teleop → SLAM → Nav2),
migrated incrementally from `qcar_updated` (the Gazebo-based end-to-end
simulation reference). This is the basic architecture for the project going
forward — update this file as the plan or design decisions change.

## Architecture

Two machines, connected over WiFi on the same network:

- **Dev PC** (this repo, ROS2 Humble, Ubuntu 22.04) — runs the "brain":
  teleop, SLAM, Nav2, visualization.
- **QCar onboard computer** (Jetson TX2, ROS2 Dashing, Ubuntu 18.04) — runs
  two independent scripts, deliberately separate processes so a LiDAR issue
  can't take down drive control and vice versa:
  - `qcar_bridge.py` — drives the real motor + steering via the Quanser HAL
    (`QCar.write()`)
  - `qcar_lidar_node.py` — reads the RPLidar A2 via the HAL

**The link between the two machines is a plain TCP socket, not ROS2.**
Native ROS2 pub/sub between the QCar's Dashing and the dev PC's Humble was
tested directly on hardware (2026-08-03) and does not interoperate — see
"Known risks" below for what was actually confirmed. So `qcar_bridge.py` and
`qcar_lidar_node.py` each run a plain TCP server (ports 5555 and 5556) with a
tiny newline-delimited-JSON protocol, no `rclpy` involved on the QCar side at
all. On the dev PC, `scripts/qcar_relay_node.py` is a normal ROS2 node that
connects to both as a client: subscribes `/cmd_vel` and forwards it to
`qcar_bridge.py`, and republishes `qcar_lidar_node.py`'s stream as a real
`sensor_msgs/LaserScan` on `/scan`. It's the only thing on the dev-PC side
that talks to the QCar directly — the rest of the stack (teleop, SLAM, Nav2)
just uses normal ROS2 topics on the Humble side, unaware that a socket relay
is involved at all.

No camera integration — this project only uses the LiDAR, cameras aren't
part of the plan.

**Onboard script lifecycle**: developed and iterated on in this repo's
`qcar_onboard/` folder, deliberately kept out of this package's
`colcon build`/`CMakeLists.txt` — neither script runs on the dev PC or
through this package's ROS2 build graph. Once a version is ready to test,
it's copied onto the QCar's onboard filesystem and run directly there as a
plain script (`python3 qcar_bridge.py`, `python3 qcar_lidar_node.py` —
no launch file, just two terminals/processes), matching Quanser's own
documented workflow — no cross-compilation, no `colcon build` on the QCar
for these scripts. Once stable and permanently deployed, they become plain
files living on the QCar with no version control there (decided
2026-08-03 — simplest option, matches how Quanser's own reference scripts
are deployed), and get removed from this repo's `qcar_onboard/` folder once
they're serving their purpose live on the robot rather than being actively
iterated on here. `scripts/qcar_relay_node.py`, by contrast, is permanent
dev-PC-side code, built via this package's normal `colcon build` like
`qcar_teleop_twist.py`.

## Migration plan

1. **Phase 1 — Teleop. ✅ Confirmed working end-to-end on real hardware,
   2026-08-03.** Keyboard teleop (`scripts/qcar_teleop_twist.py` on the dev
   PC) reliably drives the real QCar through `scripts/qcar_relay_node.py` →
   `qcar_bridge.py`, and `qcar_lidar_node.py` → `/scan` was validated
   separately in the same session (see "Validation log" below for the full
   story, including two real bugs found and fixed along the way).
2. **Phase 2 — SLAM (next up)**. Migrate `qcar_updated`'s Cartographer-based
   SLAM stack. `/scan` is already available from Phase 1; requires extending
   `qcar_bridge.py` + `qcar_relay_node.py` to also carry `/odom` (encoder
   dead-reckoning) and `/tf`.
3. **Phase 3 — Nav2**. Migrate `qcar_updated`'s Nav2/MPPI stack, verify
   autonomous goal-reaching on real hardware.

Each phase should be confirmed working end-to-end on the physical QCar
before starting the next.

## Validation log

Dated, append-only record of what's actually been confirmed on real
hardware (as opposed to design intent above) — keep adding entries here as
later phases get tested, don't rewrite history.

**2026-08-03 — Phase 1 (teleop) end-to-end, first full pass.**
- Direct hardware sanity check (bypassing any bridge): Quanser's own
  reference scripts at
  `~/DCode-v05_Qcars/vendor/examples/sdcs/qcar2/hardware/hardware_tests/` on
  the QCar confirmed the LiDAR (real scan data, ~1.5s spin-up) and the
  motor/steering/IMU/encoder/battery (`QCar2_hardware_test_basic_io.py`, 5s
  run, no errors) all work, independent of any network/bridge question.
- Native ROS2-to-ROS2 between the QCar's Dashing and the dev PC's Humble
  was tried and **confirmed not to work** (`demo_nodes_cpp`/`demo_nodes_py`
  talker/listener, reproducible across attempts) — network layer
  systematically ruled out first (raw UDP fine both directions, exact DDS
  port fine, no firewall either side, unicast-peers FastRTPS profile still
  didn't help). Root-caused to a Fast-RTPS protocol-version gap between
  Dashing (2019) and Humble (2022). This is why the architecture uses a
  plain TCP relay instead — see "Known risks" below for the full test
  trail.
- Rebuilt `qcar_bridge.py`/`qcar_lidar_node.py` as plain TCP servers (no
  ROS2 on the QCar side) plus the new `qcar_relay_node.py` on the dev PC.
  User deployed both onboard scripts to `~/ros2_amit/src/onboard_bridge` on
  the QCar (their own colcon workspace layout, separate from this repo's
  `qcar_onboard/` staging folder) — verified byte-for-byte via SHA256 and
  syntax-valid on the QCar's actual Python 3.6.9 before first run. Found and
  fixed the directory being root-owned with no write access for `nvidia`
  (`chown -R nvidia:nvidia` across the whole `~/ros2_amit` tree).
- First live test: `/scan` confirmed flowing end-to-end (~10Hz, real ~4.1m
  indoor ranges, correct `frame_id`). `cmd_vel`-driven steering confirmed
  working, including the 0.5s command watchdog correctly re-centering it
  when publishing stopped. Drive **didn't** move at `linear.x: 0.1`
  (3.3% duty cycle, below the drivetrain's static-friction threshold from
  rest) but did at `linear.x: 0.3` (10% duty, matching the known-working
  direct-hardware-test amplitude) — confirms `THROTTLE_GAIN` needs real
  calibration, now backed by an actual data point instead of just a
  documented guess.
- Found and fixed a real bug mid-session: `pkill`'s default SIGTERM isn't
  auto-converted to a catchable exception in Python the way SIGINT is, so
  stopping `qcar_lidar_node.py` with a plain `pkill` skipped its
  `finally: lidar.terminate()` cleanup and left the physical LiDAR motor
  spinning after the process was already gone. Fixed in both onboard
  scripts (SIGTERM handler routes through the same cleanup path as
  Ctrl+C), redeployed, hash-verified.
- Also found the dev PC's own `qcar_hardware` colcon install was stale
  (pre-dated same-session source edits) — rebuilt with `--symlink-install`
  so this can't recur going forward.
- **Result: full teleop chain (keyboard → `qcar_teleop_twist.py` →
  `qcar_relay_node.py` → TCP → `qcar_bridge.py` → real motor/steering)
  confirmed working.**

**2026-08-03 (continued) — RViz visualization + LiDAR angle-convention bug,
found and fixed.**
- Added `launch/qcar_visualize.launch.py`: `robot_state_publisher` +
  `joint_state_publisher` + `rviz2` (no Gazebo), reusing the existing
  `rviz/qcar.rviz` config as-is — it already had `Fixed Frame: base`, a
  `RobotModel` display on `/robot_description`, and a `LaserScan` display on
  `/scan` from when this repo was sim-only, so no config changes were
  needed. Works without `/odom`/`/tf` from the QCar since `base` (not
  `odom`/`map`) is the fixed frame — `robot_state_publisher` supplies
  `base → lidar` (and the other sensor links) as fixed transforms from the
  URDF on its own.
- First look: robot model and LiDAR dots both rendered, but an object
  placed in front of the car showed up on the wrong side.
- **Root cause turned out to be two stacked errors, found in two rounds**:
  (1) the HAL's raw LiDAR angle data increases clockwise, opposite ROS's
  counterclockwise `LaserScan` convention (REP-103) — confirmed by matching
  Quanser's own reference script needing `ax.set_theta_direction(-1)` to
  display correctly; (2) *on top of that*, there's a +90° rotational offset
  between the LiDAR's own zero-reference and the vehicle's actual front.
  The first fix (flipping direction only) exactly mirrored the error
  left-to-right instead of fixing it — a giveaway that a second, rotational
  error was still present, since a pure mirror doesn't explain "physical
  right showing up at the back." Solved by testing systematically: placed
  an object at a known physical position (vehicle's right side) and read
  off exactly what correction the *current* code's output needed, rather
  than re-guessing. Final fix: `target = -angle + pi/2` before wrapping to
  `(-pi, pi]`. Confirmed correct afterward (right stays right, front stays
  front).
- Session closed out cleanly: all processes stopped with `SIGINT` (not
  `pkill`, given the SIGTERM lesson from earlier) in order — dev PC relay
  node, then the visualization launch (rviz2 segfaulted on shutdown, a
  known harmless Qt/OpenGL exit quirk, cleaned up separately), then the
  QCar's LiDAR node, then its motor bridge — with both QCar ports (5555,
  5556) confirmed released at the end. QCar left powered on, nothing left
  running on either machine.

**Later same day — resumed session, throttle calibration deferred, steering
trim finding.**
- QCar still reachable at the same IP (DHCP lease persisted), confirmed
  clean state (no leftover processes) before bringing Phase 1 back up in
  the same order as before (motor bridge → LiDAR node → relay node →
  visualization) - all reconnected cleanly, `/scan` confirmed flowing again.
- Attempted to start `THROTTLE_GAIN` calibration (drive at fixed duty
  values, measure real speed via encoder) but **the QCar was elevated with
  wheels free-spinning** - deferred, since free-spin speed doesn't match
  real loaded/on-ground speed and would calibrate to the wrong value.
  Needs the car on the ground with clear runway space to do properly -
  still an open item.
- **New finding: the steering servo's mechanical center doesn't match
  `steering=0.0`'s commanded center** - wheels sit turned left even while
  actively running and commanded straight (not just an unpowered/no-signal
  drift). User will correct this physically (Traxxas-style adjustable
  steering linkage/turnbuckle expected on this chassis, per
  `documents/user_manual_customizing_the_qcar.pdf`) rather than a software
  offset workaround - keeps `steering=0.0` meaning true-straight for
  Phase 2's dead-reckoning odometry math, which will assume exactly that.
  **Worth re-confirming this is fixed before trusting any odometry.**
- Shutdown mid-session had one hiccup: user ran the documented SIGINT
  sequence themselves but the motor bridge (`qcar_bridge.py`) was still
  listening on port 5555 afterward - the `sudo kill -INT` step on that one
  process either wasn't run or didn't take signal-first-attempt. Finished
  from here, confirmed both ports released after. Worth remembering when
  following the shutdown steps manually: always re-check
  `sudo ss -tlnp | grep -E "5555|5556"` shows nothing, don't assume the
  sequence worked just because it was run.

## Known risks / open items

- **Steering mechanical center is offset from `steering=0.0`'s commanded
  center** - wheels sit visibly left of straight even when actively
  commanded to 0. Confirmed 2026-08-03, not yet fixed - user plans a
  physical linkage adjustment. Re-verify straight before trusting Phase 2
  odometry, which assumes commanded steering angle reflects the real one.
- **Dashing↔Humble native ROS2 pub/sub does not interoperate — confirmed on
  hardware, 2026-08-03.** Tested directly: `ros2 run demo_nodes_cpp talker`
  (QCar, Dashing) never reached `demo_nodes_py listener` (dev PC, Humble),
  reproducibly, across multiple domain-ID-matched attempts. Systematically
  ruled out the network as the cause first — raw UDP unicast was confirmed
  working in both directions, on both arbitrary ports and the exact
  Fast-RTPS discovery port (26650 for `ROS_DOMAIN_ID=77`) — and an explicit
  unicast-peers FastRTPS XML profile (bypassing multicast SPDP entirely)
  still didn't work. That points at a genuine Fast-RTPS protocol-version
  incompatibility between Dashing's (2019) and Humble's (2022) bundled
  versions, not a firewall or multicast-filtering issue. This is *why* the
  architecture above uses a plain TCP relay instead of native ROS2-to-ROS2 —
  not a workaround for an unconfirmed risk, but a fix for a confirmed one.
- **`qcar_bridge.py`'s `THROTTLE_GAIN` is uncalibrated** — needs real-world
  calibration against encoder-measured speed before trusting closed-loop
  velocity control (Phase 3+). Confirmed on hardware 2026-08-03 that the
  current guess (`1/3`) is too conservative at low `cmd_vel` values: 3.3%
  duty cycle produced no wheel rotation, 10% did. A proper calibration
  attempt was made later the same day but the QCar was elevated with wheels
  free-spinning at the time — deferred, since free-spin speed doesn't match
  real loaded/on-ground speed and would calibrate to the wrong value. Needs
  the car on the ground with clear runway space to do properly.
- **The relay protocol has no reconnection guarantees beyond a fixed retry
  loop** — `qcar_relay_node.py` will keep retrying both TCP connections
  every 2s if dropped, and `qcar_bridge.py` zeroes the motor immediately on
  disconnect (not just via the 0.5s command-staleness watchdog), but this
  hasn't been stress-tested against a flaky WiFi link yet.
- **Onboard scripts must be stopped with a signal Python treats as
  catchable (SIGINT/Ctrl+C), not a plain `pkill`/`kill` (SIGTERM)** — fixed
  2026-08-03 after a plain `pkill` on `qcar_lidar_node.py` skipped its
  cleanup and left the physical LiDAR spinning; both onboard scripts now
  handle SIGTERM the same as SIGINT, but this is worth remembering as an
  operational habit regardless (any future onboard script added here should
  do the same). See "Stopping everything" below for the full shutdown
  order.
- Gazebo sim assets (`worlds/`, `models/`, `rviz/`, `urdf/`,
  `launch/qcar_gazebo.launch.py`) are being kept indefinitely alongside the
  hardware code, for local regression-testing of migrated logic before
  trying it on real hardware (decided 2026-08-03) — this package is
  dual-purpose (sim + hardware) by design, not transitional.

## Stopping everything (real hardware)

Full teardown order, all via `SIGINT`/Ctrl+C — **never a plain `pkill`
without `-INT`**, see "Known risks" above:

1. Dev PC: `qcar_relay_node.py`, then the `qcar_visualize.launch.py`
   process group (rviz2 may segfault on exit - harmless Qt/OpenGL quirk,
   just confirm the process is actually gone after).
2. QCar: `qcar_lidar_node.py`, then `qcar_bridge.py` (needs `sudo`, same as
   starting it).
3. Confirm both TCP ports released on the QCar
   (`sudo ss -tlnp | grep -E "5555|5556"` should show nothing) before
   considering the car safely idle.

## Directory structure

- `scripts/` — runs on the dev PC, built via this package's `colcon build`
  (currently: `qcar_teleop_twist.py`, `qcar_relay_node.py`)
- `launch/qcar_gazebo.launch.py` — Gazebo simulation, kept for regression
  testing
- `launch/qcar_visualize.launch.py` — real-hardware visualization
  (`robot_state_publisher` + `joint_state_publisher` + `rviz2`, no Gazebo);
  reuses `rviz/qcar.rviz` as-is
- `urdf/`, `worlds/`, `models/`, `rviz/` — shared between sim and hardware
  visualization (the URDF and RViz config don't care which launch file
  brought them up)
- `qcar_onboard/` — staging area for code that gets deployed to and run on
  the QCar's onboard computer directly; **not** part of this package's ROS2
  build graph (currently: `qcar_bridge.py`, `qcar_lidar_node.py`). Deployed
  copy lives at `~/ros2_amit/src/onboard_bridge` on the QCar itself (the
  user's own colcon workspace layout there — unrelated name to this
  folder, don't confuse the two)
- `documents/` — Quanser hardware manuals (PDF)
- `hardware_integration_reference.md` — Quanser HAL/PAL API surface, safety
  limits, and the detailed reasoning behind bridge design decisions and bug
  fixes
