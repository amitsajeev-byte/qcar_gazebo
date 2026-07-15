# `nav2_params.yaml` parameter reference

Per-parameter explanation of every value in `nav2_params.yaml`, organized by the same top-level
sections as the YAML file. For which of these are worth adjusting to improve navigation
accuracy specifically, see `../../TUNING.md` instead - this doc is a full reference, not a
tuning guide.

## `amcl` (localization)

- `set_initial_pose: true` / `initial_pose: {x,y,z,yaw}` - seeds AMCL with a starting pose
  (0,0,0,0) at startup instead of requiring a manual "2D Pose Estimate" click in RViz first.
- `alpha1`-`alpha5` (`0.2` each) - odometry motion noise coefficients (see `TUNING.md` §5).
  Higher = particles spread out more per unit of motion, correcting faster from odometry error
  but with noisier pose estimates.
- `base_frame_id: "base"` - the robot body TF frame AMCL tracks.
- `beam_skip_distance` / `beam_skip_error_threshold` / `beam_skip_threshold` / `do_beamskip`
  (`false`) - an optimization that skips individual laser beams that disagree with the map
  across most particles (likely a dynamic obstacle, not a mapping error), so they don't wrongly
  penalize otherwise-good particles. Disabled here, so every beam is scored normally.
- `global_frame_id: "map"` - the frame AMCL publishes `map -> odom` into.
- `lambda_short: 0.1` - parameter of the laser model's "short reading" exponential distribution
  (models unexpected obstacles closer than the map predicts, e.g. someone walking in front of the
  lidar).
- `laser_likelihood_max_dist: 2.0` - max distance (m) used when pre-computing the likelihood
  field for scan matching; caps how far a laser hit's "distance to nearest map obstacle" is
  looked up.
- `laser_max_range: 12.0` / `laser_min_range: 0.15` - laser scan range bounds AMCL considers,
  matching the actual lidar sensor.
- `laser_model_type: "likelihood_field"` - which laser sensor model to use for scan matching
  (the other option is `"beam"`, an older/slower ray-casting model). Likelihood field is the
  standard modern choice.
- `max_beams: 60` - number of laser beams actually used per scan for matching (subsampled from
  the full scan for speed).
- `max_particles: 2000` / `min_particles: 500` - particle filter population bounds; more
  particles = better pose-hypothesis coverage but more CPU per update.
- `odom_frame_id: "odom"` - the frame AMCL reads to get the current odometry delta.
- `pf_err: 0.05` / `pf_z: 0.99` - KLD-sampling parameters controlling adaptive particle count
  (95% confidence that estimated distribution error is within 5% - together these set the
  confidence bound used to decide how many particles are actually needed each cycle, between
  `min_particles` and `max_particles`).
- `recovery_alpha_fast: 0.0` / `recovery_alpha_slow: 0.0` - weights for AMCL's automatic
  "kidnapped robot" recovery (injecting random particles when average particle weight drops
  suddenly). Disabled here (both 0) - if the robot ever gets picked up/teleported in map frame,
  AMCL won't self-recover; you'd need to send `/reinitialize_global_localization` or a manual
  pose estimate.
- `resample_interval: 1` - resample the particle filter every N updates (1 = every update).
- `robot_model_type: "nav2_amcl::DifferentialMotionModel"` - correct as-is for this Ackermann
  car; see `TUNING.md` §5 for the full reasoning (both shipped AMCL motion models take the same
  odometry delta and alpha params - "differential" just means non-holonomic, which fits a car
  fine; "omni" adds a strafe noise term for robots that can actually move sideways, which this
  car cannot).
- `save_pose_rate: 0.5` - how often (Hz) the last known pose is saved to the parameter server for
  restart persistence.
- `sigma_hit: 0.2` - standard deviation (m) of the Gaussian "hit" component of the laser model -
  how much measurement noise is expected on a beam that correctly hits a mapped obstacle.
- `tf_broadcast: true` - whether AMCL actually publishes the `map -> odom` transform (vs. just
  computing pose internally).
- `transform_tolerance: 1.0` - how far into the future (s) the published TF is post-dated,
  giving consumers a buffer against small timing jitter.
- `update_min_a` (`0.2` rad) / `update_min_d` (`0.25` m) - minimum rotation/translation since the
  last update before AMCL bothers re-localizing against a new scan.
- `z_hit` (`0.5`) / `z_short` (`0.05`) / `z_max` (`0.05`) / `z_rand` (`0.5`) - mixture weights of
  the laser model's four components (hit-obstacle, unexpected-short-reading, max-range-reading,
  random-noise); must sum to ~1. Here it's a 50/50 blend of "trust clean hits" and "assume a lot
  of random noise," with very little weight on short-reading or max-range explanations.
- `scan_topic: scan` - which topic AMCL subscribes to for lidar data.

## `bt_navigator` (behavior tree executor - the top-level "run the nav mission" node)

- `global_frame: map` / `robot_base_frame: base` / `odom_topic: /odom` - frames/topic the BT
  navigator needs to track mission progress.
- `bt_loop_duration: 10` (ms) - how often the behavior tree ticks.
- `default_server_timeout: 20` (s) - timeout waiting for an action server (planner, controller,
  etc.) to respond.
- `wait_for_service_timeout: 1000` (ms) - timeout waiting for BT-node-internal service calls
  (e.g. clear costmap).
- `navigate_to_pose` / `navigate_through_poses` - which navigator plugin handles each action type
  (both are the stock Nav2 ones here).
- `plugin_lib_names` - the full catalog of behavior-tree leaf-node plugins made available to
  whatever BT XML file is loaded (default trees, since no custom XML is configured here). Each
  entry registers one BT node type (e.g. `nav2_spin_action_bt_node` = the `<Spin>` leaf,
  `nav2_recovery_node_bt_node` = the `<RecoveryNode>` control node). Not individually tunable -
  they just need to match whatever nodes your BT XML actually references.

## `controller_server` (local path-following)

- `controller_frequency: 20.0` (Hz) - how often `FollowPath` computes a new `cmd_vel`.
- `min_x_velocity_threshold` / `min_theta_velocity_threshold` (`0.001`) - below this, velocity is
  treated as zero (avoids sending negligible/noisy commands).
- `min_y_velocity_threshold: 0.5` - same idea for lateral velocity; irrelevant here since this is
  a non-holonomic car and `y` velocity is always 0.
- `failure_tolerance: 0.3` (s) - how long the controller can fail to produce a valid command
  before the whole `FollowPath` action reports failure (triggering a recovery behavior).
- `progress_checker_plugin` / `goal_checker_plugins` / `controller_plugins` - names the plugin
  instances used below.
- `progress_checker` (`SimpleProgressChecker`): `required_linear_distance: 0.5` /
  `required_angular_distance: 0.1` - the robot must move at least this much within a time window
  (elsewhere-configured internal default) or it's considered "stuck," triggering
  failure/recovery.
- `general_goal_checker` (`SimpleGoalChecker`): `stateful: true` (once within tolerance, stays
  "reached" even if it drifts back out - avoids flapping), `xy_goal_tolerance: 0.1` m,
  `yaw_goal_tolerance: 0.7` rad (~40°) - see `TUNING.md` §3.
- `FollowPath` (`RegulatedPurePursuitController`) - see `TUNING.md` §2 for the accuracy-relevant
  ones; the rest: `lookahead_time: 1.5` s (an alternate way lookahead distance can scale with
  speed, only used if `use_velocity_scaled_lookahead_dist: true`, which is `false` here so this
  is inert), `rotate_to_heading_angular_vel: 1.0` (only relevant if `use_rotate_to_heading: true`,
  `false` here since the car can't rotate in place), `transform_tolerance: 0.5` s (TF lookup
  slack), `min_approach_linear_velocity: 0.05` / `approach_velocity_scaling_dist: 0.6` (slows the
  car down over the last 0.6m before the goal, down to a floor of 0.05 m/s),
  `use_collision_detection: true` / `max_allowed_time_to_collision_up_to_carrot: 1.0` (aborts/
  replans if the planned arc to the lookahead point would hit an obstacle within 1s),
  `allow_reversing: false` (car will not back up to correct a path), `rotate_to_heading_min_angle:
  0.785` rad (~45°, inert while `use_rotate_to_heading` is false), `max_robot_pose_search_dist:
  10.0` m (how far along the path RPP searches to find the robot's closest point, for very long
  paths).

## `smoother_server`

- `smoother_plugins: ["simple_smoother"]`, `SimpleSmoother`: `tolerance: 1.0e-10` (convergence
  threshold - how small a change counts as "smoothing done"), `max_its: 1000` (iteration cap),
  `do_refinement: true` (runs an extra pass to reduce residual curvature/kinks). Post-processes
  the raw grid-planner path into something less jagged before handing it to the controller.

## `planner_server` (global path planning)

- `expected_planner_frequency: 20.0` (Hz) - used only to warn if planning is taking longer than
  expected, not an actual rate limiter.
- `GridBased` (`NavfnPlanner`): `tolerance: 0.5` m (accepts a planned path ending within this
  distance of the exact goal if the goal cell itself is unreachable/occupied), `use_astar: false`
  (uses Dijkstra instead of A* - slightly slower but doesn't need a heuristic; rarely matters at
  this map size), `allow_unknown: true` (path can cross unexplored/unknown costmap cells, not
  just known-free ones). This planner doesn't know about the car's turning-radius limit - see
  `TUNING.md` §4.

## `local_costmap` / `global_costmap`

- `update_frequency` / `publish_frequency` - local: 5.0/2.0 Hz, global: 1.0/1.0 Hz. Local updates
  faster since it's the immediate-vicinity map used for live obstacle avoidance; global updates
  slower since it covers the whole map and changes less often.
- `global_frame` - local costmap uses `odom` (rolls with the robot, for reactive avoidance robust
  to localization jumps), global costmap uses `map` (fixed, for long-range planning).
- `robot_base_frame: base` - both track the same robot frame.
- `rolling_window: true` / `width: 3` / `height: 3` (local only) - the local costmap is a 3x3m
  window centered on and moving with the robot, rather than a fixed full-map grid.
- `resolution: 0.05` (m/cell, both) - matches the map's resolution.
- `robot_radius: 0.15` (both) - circular footprint approximation - see `TUNING.md`'s "Known
  mismatches" for why this is worth revisiting.
- `plugins` - which costmap layers are stacked: local = `voxel_layer` + `inflation_layer`; global
  = `static_layer` + `obstacle_layer` + `inflation_layer`.
- `inflation_layer`: `cost_scaling_factor: 3.0` / `inflation_radius: 0.55` - how obstacle cost
  decays with distance (higher scaling factor = sharper falloff, so the "keep away" zone shrinks
  faster near the true obstacle edge); `inflation_radius` is how far (m) the cost gradient
  extends at all.
- `voxel_layer` (local only, `nav2_costmap_2d::VoxelLayer`) - a 3D-aware obstacle layer (tracks
  obstacles in a Z-column per cell, not just 2D) built from live sensor data. `enabled: true`,
  `publish_voxel_map: true` (for RViz debug viz), `origin_z: 0.0` / `z_resolution: 0.05` /
  `z_voxels: 16` (voxel grid covers 0 to 0.8m height in 16 slices), `max_obstacle_height: 2.0` m
  (ignore returns above this), `mark_threshold: 0` (min number of marked voxels in a column
  before that 2D cell counts as occupied).
- `obstacle_layer` (global only, `nav2_costmap_2d::ObstacleLayer`) - the simpler 2D equivalent
  used for the static/global map.
- Both layers' `scan:` observation source block: `topic: /scan`, `max_obstacle_height: 2.0`,
  `clearing: true` (scan can clear previously-marked free cells) / `marking: true` (scan can mark
  new obstacles), `data_type: "LaserScan"`, `raytrace_max_range: 3.0` / `raytrace_min_range: 0.0`
  (how far a beam's empty-space raytrace clears cells), `obstacle_max_range: 2.5` /
  `obstacle_min_range: 0.0` (how far a beam's actual hit marks a cell as occupied - note this is
  less than `raytrace_max_range` and the lidar's own 12m range, so obstacles beyond 2.5m won't be
  marked into the costmap even though they're visible in `/scan`).
- `static_layer` (global only): `plugin: "nav2_costmap_2d::StaticLayer"`,
  `map_subscribe_transient_local: true` - loads the saved occupancy grid as the base layer;
  `transient_local` QoS means it gets the map even if `/map` was published before this node
  subscribed (map server uses a latched-style QoS).
- `track_unknown_space: true` (global only) - unexplored cells stay "unknown" rather than being
  assumed free, which matters for `allow_unknown` in the planner above.
- `always_send_full_costmap: true` (both) - sends the entire costmap on every update rather than
  incremental diffs; simpler/more robust, more bandwidth (irrelevant at this map size).

## `map_server` / `map_saver`

- `map_server`: `yaml_filename: ""` - left blank because `qcar_nav2.launch.py` passes the actual
  map path as a launch argument override at runtime.
- `map_saver`: `save_map_timeout: 5.0` s (how long `map_saver_cli` waits for a `/map` message),
  `free_thresh_default: 0.25` / `occupied_thresh_default: 0.65` (probability thresholds for
  classifying a cell as free/occupied when saving - matches `maps/qcar_map.yaml`'s own
  thresholds), `map_subscribe_transient_local: true` (same QoS reasoning as above).

## `behavior_server` (recovery behaviors)

- `costmap_topic` / `footprint_topic` - where recovery behaviors get live obstacle/footprint data
  for their own collision checks.
- `cycle_frequency: 10.0` (Hz) - internal control loop rate for behaviors.
- `behavior_plugins` - the five registered recoveries: `spin`, `backup`, `drive_on_heading`,
  `wait`, `assisted_teleop`. Note `spin` is still registered as *available* here, even though
  in-place rotation is physically impossible for this Ackermann vehicle - whether it's actually
  invoked at runtime depends on which behavior-tree XML is loaded (not controlled by this file),
  so check that separately if `spin` shows up during real recovery attempts.
- `global_frame: odom` / `robot_base_frame: base` - frames behaviors operate in.
- `transform_timeout: 0.1` (s) - TF lookup timeout.
- `simulate_ahead_time: 2.0` (s) - how far ahead behaviors simulate their own motion for
  collision checking before executing.
- `max_rotational_vel: 1.0` / `min_rotational_vel: 0.4` / `rotational_acc_lim: 3.2` - limits
  specifically for the `Spin` behavior (and any other behavior needing pure rotation) - same
  physical-feasibility caveat as above.

## `waypoint_follower`

- `loop_rate: 20` (Hz) - control loop rate while executing a waypoint mission.
- `stop_on_failure: false` - if one waypoint fails, continue on to the next rather than aborting
  the whole mission.
- `waypoint_task_executor_plugin: "wait_at_waypoint"`, `WaitAtWaypoint`: `enabled: true`,
  `waypoint_pause_duration: 200` ms (pause at each waypoint before continuing) - only relevant if
  you use the waypoint-following action, not plain single-goal `navigate_to_pose`.

## `velocity_smoother`

- `smoothing_frequency: 20.0` (Hz) - rate at which raw controller `cmd_vel` gets smoothed before
  being republished.
- `scale_velocities: false` - if true, would proportionally scale down all velocity components
  together when one hits a limit (keeps commanded curvature intact); false means each component
  is independently clamped.
- `feedback: "OPEN_LOOP"` - smooths based on the last commanded velocity, not actual measured
  velocity (the alternative, `CLOSED_LOOP`, reads back from `odom_topic` below).
- `max_velocity: [0.3, 0.0, 1.0]` / `min_velocity: [-0.3, 0.0, -1.0]` - [x, y, theta] velocity
  bounds; y is 0 since this is non-holonomic.
- `max_accel: [1.5, 0.0, 2.0]` / `max_decel: [-1.5, 0.0, -2.0]` - acceleration/deceleration
  limits per axis.
- `odom_topic: "odom"` / `odom_duration: 0.1` - only used in `CLOSED_LOOP` feedback mode
  (currently inert since feedback is `OPEN_LOOP`).
- `deadband_velocity: [0.0, 0.0, 0.0]` - velocities below this magnitude get zeroed instead of
  passed through (none configured here).
- `velocity_timeout: 1.0` (s) - if no new `cmd_vel` arrives within this window, publishes zero
  velocity as a safety stop.

## `lifecycle_manager`

- `autostart: true` - automatically transitions all listed nodes through their lifecycle states
  (configure -> activate) on launch, rather than requiring a manual service call.
- `node_names` - the ordered list of managed nodes brought up together: `map_server`, `amcl`,
  `controller_server`, `smoother_server`, `planner_server`, `behavior_server`, `bt_navigator`,
  `waypoint_follower`, `velocity_smoother`.

## Worth a second look

- `allow_reversing: false` in `FollowPath` - confirm this is intentional; earlier migration
  history for the sibling `qcar_navigation` package referenced reversing being enabled for
  collision recovery.
- `spin` is registered in `behavior_server.behavior_plugins` despite being physically infeasible
  for this car - check whatever BT XML is actually active to see if it's really excluded from
  the runtime recovery tree (this params file alone doesn't control that).
