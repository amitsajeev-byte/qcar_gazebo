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
2. **Phase 2 — SLAM. ✅ Confirmed working end-to-end on real hardware,
   2026-08-06.** Migrated `qcar_updated`'s Cartographer-based SLAM stack via
   `launch/qcar_slam.launch.py` + `config/cartographer/qcar_2d.lua` (copied
   from `qcar_updated` unchanged). Turned out **not** to need `/odom`/`/tf`
   extension after all — `qcar_2d.lua` has `use_odometry = false` and
   `provide_odom_frame = true`, so Cartographer computes the whole
   `map→odom→base` chain itself from `/scan` alone (pure LiDAR
   scan-matching SLAM), same as it does in the Gazebo sim version. Only
   needed a working `/scan` (already had it from Phase 1) and the
   `base→lidar` static transform (`robot_state_publisher`, via
   `qcar_visualize.launch.py`, which `qcar_slam.launch.py` includes). First
   real map saved to `~/qcar_map.pgm`/`.yaml` — see "Validation log".
3. **Phase 3 — Nav2. Plumbing built and confirmed bringing up cleanly on
   real hardware, 2026-08-06 — autonomous goal-reaching not yet attempted.**
   Migrated `qcar_updated`'s Nav2/MPPI stack via `launch/qcar_nav2.launch.py`
   + `config/nav2/nav2_params.yaml` (adapted: `use_sim_time: false`
   throughout, `EarlyCommitCritic` deliberately left out for this first
   pass — see "Known risks") + `config/nav2/behavior_trees/*.xml` (copied
   unchanged) + `maps/qcar_map.yaml` (Phase 2's saved map). Unlike Phase 2,
   this **did** need real odometry — AMCL looks up a live `odom→base` TF at
   every scan callback, which Cartographer's pure-SLAM approach never
   required. Added wheel-odometry dead reckoning to `qcar_bridge.py`
   (encoder-based, sent back over the same TCP connection cmd_vel arrives
   on) and `/odom` + TF publishing to `qcar_relay_node.py`, tested
   separately on hardware first (real motion confirmed: `linear.x: 0.3` for
   3s produced `/odom` reading `x: 1.25m`, `v: ~0.37 m/s`) before building
   Nav2 on top of it. Full `bringup_launch.py` stack (AMCL, both costmaps,
   planner, MPPI controller, behavior/bt_navigator/waypoint_follower/
   velocity_smoother) came up and reached "Managed nodes are active" for
   both lifecycle managers on the first attempt, no crashes. **Deliberately
   did not send a real navigation goal yet** — the odometry this all
   depends on is only as accurate as the steering trim, which was unfixed
   at the time. That's now resolved (see "Known risks" — software
   `STEERING_TRIM` calibrated via real driving tests, 2026-08-06) — a real
   navigation goal test is the next step whenever that's picked back up.

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

**Later same day — Phase 2 (SLAM) tested live on real hardware, confirmed
working, first map saved.**
- Brought up the full chain (motor bridge, LiDAR node, relay node,
  `qcar_slam.launch.py`) with the QCar grounded (not elevated) in a closed
  space. `cartographer_node` inserted its first submap within a second of
  starting and kept building from there - confirmed via both the log output
  and `ros2 topic echo /map --field info`, map size actively growing over
  time (real occupancy grid data, not just a placeholder).
- Found and fixed a real bug hit only when actually using `ros2 run`
  instead of invoking scripts directly: `qcar_relay_node.py` was missing
  its executable bit (created without `+x`), so it silently didn't show up
  in `ros2 pkg executables qcar_hardware` - `chmod +x` fixed it immediately,
  no rebuild needed since `--symlink-install` points straight at the
  source file. See "Known risks" below.
- Drove the QCar around with `qcar_teleop_twist.py` to build out the map,
  then saved it with `nav2_map_server`'s `map_saver_cli`
  (`ros2 run nav2_map_server map_saver_cli -f ~/qcar_map --ros-args -p
  save_map_timeout:=5.0`) - produced `qcar_map.pgm` + `qcar_map.yaml`
  (147×145 cells @ 5cm/pixel, ≈7.35m × 7.25m mapped). **First real map from
  the physical QCar** - this becomes the static map Phase 3 (Nav2) will
  use.
- **Result: Phase 2 (SLAM) confirmed working end-to-end on real hardware.**

**Later same day — replaced the first map with a corrected pass** (moved
into `maps/` per the user's request, then overwritten in place with a
better/larger map, 155×167 cells, after the first save was found to be
wrong - `map_saver_cli` overwrites cleanly when pointed at the same `-f`
path while `qcar_slam.launch.py` is still up).

**Later same day — Phase 3 (Nav2) plumbing built and confirmed bringing up
cleanly on real hardware, autonomous goal-reaching not yet attempted.**
- Realized partway through planning this that AMCL (unlike Cartographer)
  needs a live `odom→base` TF, since it looks that up at every scan
  callback to track motion between localization corrections - Phase 2
  never needed this because Cartographer computes its own odometry
  internally from `/scan` scan-matching alone. This meant the wheel-
  odometry extension originally scoped for Phase 2 (then found
  unnecessary there) was needed after all, just one phase later than
  planned.
- Checked in on the steering trim status before building on odometry that
  depends on it - **still not physically fixed** as of this session.
  Proceeded with building the odometry code anyway (useful regardless of
  when the trim gets fixed) but treated live-tested accuracy as
  provisional, and deliberately stopped short of sending a real navigation
  goal once Nav2 was up - see below.
- Added encoder-based dead reckoning to `qcar_bridge.py` (same formula
  already documented: `distance(m) = encoderCounts * (1/2880) * 0.01977`),
  sent back to the dev PC over the *same* TCP connection `/cmd_vel` arrives
  on (TCP is full-duplex, no new port needed) - `qcar_relay_node.py`
  reads it and publishes `/odom` + broadcasts `odom→base` TF. Tested this
  in isolation before touching Nav2 at all: `/odom` published at ~45Hz,
  all zeros at rest: correct; commanded `linear.x: 0.3` for 3s produced
  `x: 1.25m`, `v: ~0.37 m/s`, `y` staying at 0 (straight line, no steering
  commanded) - real, plausible numbers, pipeline confirmed working
  end-to-end before layering Nav2 on top.
- Given the custom `EarlyCommitCritic` MPPI plugin decision from earlier
  (start simpler, add later), copied `qcar_updated`'s `nav2_params.yaml`
  with `EarlyCommitCritic` removed from the critics list and its tuned
  parameters preserved as a comment block (not deleted - real, hard-won
  live-tested values, for whenever the plugin migration happens) alongside
  a `use_sim_time: true` → `false` sweep across all 13 occurrences.
  `behavior_trees/*.xml` (the "no `<Spin>` recovery, replan only when
  needed" customization) copied unchanged - applies regardless of
  hardware/sim or the critic decision.
- `launch/qcar_nav2.launch.py` (`qcar_visualize.launch.py` include for
  `robot_state_publisher`/TF, then `nav2_bringup`'s `bringup_launch.py`
  with our map/params, no Gazebo) brought the **entire** Nav2 stack up on
  the first attempt with the full onboard chain running (motor bridge +
  LiDAR node + relay node): both lifecycle managers
  (localization/navigation) reached "Managed nodes are active", every
  server (controller, smoother, planner, behavior, bt_navigator,
  waypoint_follower, velocity_smoother) activated cleanly, all expected
  topics present (`/amcl_pose`, `/particle_cloud`, `/plan`, both
  costmaps), no crashes, stable/quiescent once settled - no crash-loop.
  Global costmap sized itself to 155×167 - correctly matching the saved
  map.
- **Deliberately stopped here without sending a navigation goal.** The
  localization/planning this all depends on is only as accurate as the
  dead-reckoning odometry, which is only as accurate as the steering trim
  - still open at the time. Sending a real autonomous goal then would have
  meant navigating on known-uncorrected drift. See below for how this got
  resolved the same day.

**Later same day — steering trim fixed in software, calibrated via real
driving tests.**
- Physical linkage adjustment turned out not to be available on this
  chassis ("cannot be corrected manually") - pivoted to a software fix:
  `STEERING_TRIM`, a constant added to `qcar_bridge.py`, applied only at
  the `car.write()` call site.
- Calibrated interactively: started with a static test (motor off, held
  fixed candidate angles, judged by eye against straight) to get roughly
  close (-0.05 too left, -0.09 close), then refined with real straight-line
  driving tests (drive N meters, physically measure lateral offset) for
  actual precision - static visual judgment alone isn't precise enough.
  Iterated through -0.09 (0.1m right drift over 4.5m), a computed
  correction to -0.0678 that overshot into a left drift, -0.085 (0.2m
  right drift over 5.3m - noisier, likely partly due to imprecise vehicle
  aiming at test start, not purely the trim value), and settled on -0.08
  after a ~4.4m drive showed no reported drift. Accepted as "almost
  accurate" (user's call), not a mathematically perfect zero.
- **Caught a real bug in the first implementation along the way**: trim
  was initially folded into the same steering value used for both the real
  servo command *and* the odometry's dead-reckoning math. Since trim's
  whole purpose is making the real wheels read ~0 despite a nonzero servo
  command, feeding that same trimmed value into the odometry made it
  believe the wheels were physically deflected by the trim amount even
  while driving dead straight - a live straight-line test briefly produced
  a fictitious ~30-degree fake curve in `/odom` before this was caught.
  Fixed by keeping the untrimmed/kinematic steering angle for odometry and
  only computing the trimmed servo command separately, right before
  `car.write()` (see `qcar_bridge.py`'s `compute_steering()` vs.
  `apply_trim()`).
- **Result: steering trim resolved, `/odom` now correctly reads `y: 0.0`
  for real straight-line driving.** Phase 3 is unblocked - a real
  navigation goal test is the next step.

**Later same day — first real Nav2 goal test (run independently, not via
this session) surfaced a new issue: sudden AMCL localization jumps
(LiDAR points visibly snapping in RViz) and path deviation on both
straight and curved segments.**
- Ruled out the already-known `EarlyCommitCritic` gap as the primary cause
  - that would only explain curve-specific deviation, not straight-segment
  deviation too.
- Working theory: `qcar_bridge.py`'s dead-reckoning assumes the steering
  servo reaches a commanded angle instantly, but
  `documents/user_manual_system_hardware.pdf` documents a real steering
  time constant of τ=0.16s. Under continuous MPPI control (which samples
  small steering corrections even on nominally "straight" segments via
  `vx_std`/`wz_std` noise), this unmodeled lag accumulates real heading
  error continuously, not just during obvious turns - AMCL then has to
  correct that accumulated error in a sudden jump when it re-localizes
  against the real LiDAR scan.
- First mitigation applied: raised AMCL's `alpha1-5` in
  `config/nav2/nav2_params.yaml` from `0.4` to `0.6` (the sim version had
  already gone `0.2` → `0.4` for this exact symptom category - "map->odom
  TF jumps... during maneuvering" - so this is a continuation of the same
  tuning direction, not a new idea). **Reverted at the user's request**
  before re-testing - see below for what actually turned out to be wrong.

**Later same day — real root cause found via video review + the user's own
precise diagnosis ("transform issue"), fixed via corrected timestamps, not
AMCL tuning.**
- The AMCL-alpha and servo-lag theories above were both superseded once the
  user provided a screen recording of an actual goal run and described the
  symptom precisely: the LiDAR scan's *shape* stayed internally coherent
  (not random noise/divergence) but the whole scan was rigidly offset from
  the static map, tracking the robot's motion - "it feels like the map is
  staying still, but the skeleton of the map is moving... the robot follows
  the dotted LiDAR map... not the original map." Confirmed by extracting
  frames from the video (OpenCV) and comparing scan-to-map alignment
  directly: near-perfect at the start, badly diverged mid-maneuver
  (diagonal streaks of scan points cutting through open space, nowhere
  near any wall), partially recovering later. A coherent rigid offset that
  tracks with motion, rather than random noise, is the signature of a
  *timing* problem, not a spatial one - which is exactly what the user's
  own "transform issue" instinct was pointing at.
- Also checked qcar_updated's own `CHANGELOG.md` for a matching sim-side
  history (a very close-sounding "robot deviates mid-curve, corrects by
  curve end" entry, 2026-08-02) - the user confirmed this was a different
  issue, so that lead wasn't pursued further, but it's worth knowing that
  entry exists if a genuine goal-shape-dependent MPPI limitation shows up
  again later.
- **Actual root cause**: `qcar_relay_node.py` was stamping `/scan` and
  `/odom` using the dev PC's *receipt* time (`self.get_clock().now()`),
  not the QCar's actual *capture* time. `/scan` and `/odom` travel over
  two independent TCP connections with independent, uncorrelated network
  latency (this WiFi link's jitter was already documented, 87-300ms+) -
  so a scan that happened to arrive a bit later than usual got paired by
  AMCL against an odom-derived pose that had already moved on, producing
  exactly the "shape preserved, whole thing rigidly shifted" pattern.
  Confirmed the QCar and dev PC clocks are meaningfully unsynchronized
  (~0.50s offset, measured via a round-trip-corrected `date`+SSH check,
  averaged over 5 trials, tightly clustered) - not accounted for anywhere
  before this.
- **Fix**: `qcar_bridge.py` and `qcar_lidar_node.py` now include their own
  QCar-side capture timestamp (`time.time()`) in every message instead of
  leaving the relay to stamp at receipt time. `qcar_relay_node.py` gained
  a `qcar_clock_offset` parameter and converts each QCar timestamp into a
  dev-PC-clock-equivalent stamp before publishing - see that file's
  module docstring for the measurement procedure (must be re-measured each
  session; offset can drift if either machine reboots or its own NTP
  client re-syncs). **Deliberately not fixed via system-level NTP/chrony**
  even though that was tried first (an `allow` rule was added to the dev
  PC's `chrony.conf` then reverted) - the QCar is shared lab hardware, and
  leaving a persistent system config pointing at one user's personal dev
  PC as a time source isn't good practice for infrastructure other people
  also use. The software-side correction is fully self-contained to this
  package's own code and touches no shared system state.
- Verified after the fix: `/scan`/`/odom` timestamps land close to "now"
  minus genuine one-way network latency (not clock-skew-inflated), TF
  lookups (`odom`→`base`) resolve cleanly with no extrapolation/repeated-
  data warnings. **Not yet re-tested with a real Nav2 goal** - next step
  whenever Nav2 testing resumes, with AMCL's `alpha1-5` back at the
  original `0.4` (the timestamp fix is the real fix; the alpha tuning was
  chasing the wrong cause).

## Known risks / open items

- **Steering mechanical center offset - fixed in software, 2026-08-06.**
  No adjustable physical linkage turned out to be available on this
  chassis, so this was corrected with a `STEERING_TRIM` constant in
  `qcar_bridge.py` (`-0.08` rad) instead of a mechanical adjustment.
  Calibrated iteratively on real hardware: an interactive static test
  (motor off, held candidate angles, judged by eye) got close, then a
  sequence of real straight-line driving tests (drive N meters, measure
  actual lateral offset) refined it further - see the constant's own
  comment in `qcar_bridge.py` for the full numeric history. Accepted as
  "almost accurate" (user's call), not a mathematically perfect zero.
  Applying the trim surfaced a real bug along the way: the first
  implementation folded the trim into the *same* value used for both the
  real servo command and the odometry's dead-reckoning math, so the
  odometry started believing the trimmed servo command was a real physical
  wheel deflection - a straight-line test briefly showed a fictitious
  ~30-degree fake curve before this was caught and fixed (trim now only
  applied at the `car.write()` call site, never upstream of the odometry).
  If drift reappears or matters more for a future use case (tighter Nav2
  tolerances, etc.), re-run the driving-test calibration procedure rather
  than adjusting from static/visual judgment alone.
- **`EarlyCommitCritic` (custom MPPI plugin) deliberately left out of
  Phase 3's first pass** - see `config/nav2/nav2_params.yaml`'s comment
  block for the full reasoning and the preserved tuned parameter values.
  It exists to stop MPPI settling into a near-straight local optimum
  instead of committing to a path's initial curvature - if the robot is
  seen cutting corners or drifting off newly-started curves during real
  goal-reaching tests, this is the most likely reason, and the fix is
  migrating the plugin from `qcar_updated` (C++ build with a documented
  ABI-sensitive xtensor/xsimd compile-flag requirement - see that
  package's `CMakeLists.txt`).
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
- **New dev-PC scripts under `scripts/` need `chmod +x` before they'll show
  up in `ros2 pkg executables`/`ros2 run`** — with `--symlink-install`, the
  installed "binary" is a symlink straight to the source file, so the
  source itself needs the executable bit, not just correct install rules in
  `CMakeLists.txt`. `qcar_relay_node.py` was missing it (created without
  `+x`) and silently worked anyway during development because it was always
  invoked directly as `python3 scripts/qcar_relay_node.py`, not
  `ros2 run` — only surfaced when actually trying `ros2 run`. Fixed
  2026-08-03.

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
- `launch/qcar_slam.launch.py` — Phase 2, real-hardware SLAM: includes
  `qcar_visualize.launch.py` and layers `cartographer_node` +
  `cartographer_occupancy_grid_node` on top, using
  `config/cartographer/qcar_2d.lua`
- `launch/qcar_nav2.launch.py` — Phase 3, real-hardware Nav2: includes
  `qcar_visualize.launch.py`, then `nav2_bringup`'s `bringup_launch.py`
  with `maps/qcar_map.yaml` + `config/nav2/nav2_params.yaml`
  (`use_sim_time: false`, BT XML overrides via `RewrittenYaml`), no Gazebo,
  no Cartographer (AMCL against the static map instead of live SLAM)
- `urdf/`, `worlds/`, `models/`, `rviz/` — shared between sim and hardware
  visualization (the URDF and RViz config don't care which launch file
  brought them up)
- `config/cartographer/qcar_2d.lua` — copied unchanged from `qcar_updated`,
  shared between the sim and hardware SLAM launches
- `config/nav2/nav2_params.yaml` — adapted from `qcar_updated`
  (`use_sim_time: false`, `EarlyCommitCritic` removed from the critics list
  with its tuned values preserved as a comment - see "Known risks");
  `config/nav2/behavior_trees/*.xml` copied unchanged
- `maps/qcar_map.pgm` + `maps/qcar_map.yaml` — real map built from the
  physical QCar via Phase 2, saved with `nav2_map_server`'s
  `map_saver_cli`. This is the static map Phase 3 (Nav2) will load.
  Replaced 2026-08-06 with a better/corrected pass (155×167 cells @
  5cm/pixel) after the first save (147×145) was found to be wrong — if
  this needs replacing again, just re-run `map_saver_cli` with the same
  `-f` path while `qcar_slam.launch.py` is up, it overwrites in place.
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
