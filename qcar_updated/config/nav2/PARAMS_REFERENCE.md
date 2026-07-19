# `nav2_params.yaml` parameter reference

Per-parameter explanation of every value in `nav2_params.yaml`, organized by the same top-level
sections as the YAML file. For which of these are worth adjusting to improve navigation
accuracy specifically, see `../../TUNING.md` instead - this doc is a full reference, not a
tuning guide.

## `amcl` (localization)

- `set_initial_pose: true` / `initial_pose: {x,y,z,yaw}` - seeds AMCL with a starting pose
  (0,0,0,0) at startup instead of requiring a manual "2D Pose Estimate" click in RViz first.
- `alpha1`-`alpha5` (`0.4` each, raised from `0.2` as of 2026-07-15 (2)) - odometry motion noise
  coefficients (see `TUNING.md` §5). Higher = particles spread out more per unit of motion,
  correcting faster from odometry error but with noisier pose estimates. Raised after live-
  measuring `map -> odom` TF jumps of up to 7cm/2.3deg in a single 0.2s step during goal-approach
  reorientation maneuvers (K-turns with reversing) - real wheel odometry drift during tight/
  reversing motion apparently exceeds what `0.2` assumed, so AMCL underestimated uncertainty
  between scan-match updates and had to correct in large, sudden jumps.
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
- `update_min_a` (`0.1` rad) / `update_min_d` (`0.15` m) - minimum rotation/translation since the
  last update before AMCL bothers re-localizing against a new scan. Lowered from `0.2`/`0.25` as
  of 2026-07-15 (2), paired with the `alpha1`-`alpha5` increase above - re-localizes more often
  (smaller corrections each time) rather than accumulating more drift between updates and
  correcting in a single larger jump.
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
- `progress_checker` (`SimpleProgressChecker`): `required_linear_distance: 0.1` m /
  `movement_time_allowance: 30.0` s - the robot must move at least this much linear distance
  within this time window or it's considered "stuck," triggering failure/recovery.
  `required_angular_distance: 0.1` is declared but **not implemented** by this Humble version of
  `SimpleProgressChecker` (added in a later nav2 release) - progress is judged by linear
  displacement alone; see the inline comment in `nav2_params.yaml` and `CHANGELOG.md`'s
  2026-07-14 (3) entry.
- `general_goal_checker` (`SimpleGoalChecker`): `stateful: true` (once within tolerance, stays
  "reached" even if it drifts back out - avoids flapping), `xy_goal_tolerance: 0.12` m (widened
  slightly from `0.1` on 2026-07-15 (3) - see `TUNING.md` §3), `yaw_goal_tolerance: 0.3` rad
  (~17°) - see `TUNING.md` §3.
- `FollowPath` (`MPPIController` as of 2026-07-14 (9), replacing `RegulatedPurePursuitController`
  - see `TUNING.md` §2 for the full rationale and the accuracy-relevant parameters): `time_steps:
  56` / `model_dt: 0.05` / `batch_size: 2000` (core sampling parameters - trajectory horizon
  length, timestep size, and number of candidate trajectories sampled per control cycle.
  `batch_size` raised to `2500`/`3500` and live-tested on 2026-07-19 (4) as a fix for path-
  tracking looseness (see `PathAlignCritic.cost_weight` below for the critic-weight attempt that
  didn't work) - genuinely tightened tracking, reproducibly, but the extra CPU cost degraded
  control-loop timing enough near the goal that one otherwise-normal run took 4.5 minutes to
  converge instead of ~30s; reverted to `2000` - see `CHANGELOG.md` 2026-07-19 (4) for the full
  trial data),
  `vx_std` / `vy_std` / `wz_std: 0.2 / 0.0 / 0.4` (sampling noise standard deviation per axis -
  `vy_std: 0.0` since this is non-holonomic; these are the plugin's shipped defaults. Briefly
  lowered to `0.15`/`0.3` on 2026-07-18 after the user recorded and shared video evidence of the
  robot deviating from the planned path mid-trip - tighter sampling meant less "wobble," but
  restored to the defaults on 2026-07-19 (2) after live-testing traced a *different, worse* bug to
  the tightening: MPPI samples each cycle's candidate trajectories as noise added to the
  *previous* cycle's control sequence (`Optimizer::shiftControlSequence()`,
  `nav2_mppi_controller/src/optimizer.cpp`), so once that sequence collapsed toward near-zero
  velocity on a demanding goal, tighter noise made it measurably harder to sample anything far
  enough from "near zero" to escape - a self-reinforcing freeze with no way out short of a full
  `Optimizer::reset()`. If wobble reappears, prefer tuning `PathAlignCritic`/`PathFollowCritic`
  over re-tightening these - see `CHANGELOG.md` 2026-07-19 (2)), `vx_max` /
  `vx_min: 0.3 / -0.2` (forward/reverse speed limits), `vy_max: 0.0` (no lateral motion),
  `wz_max: 1.9` (max angular velocity considered), `iteration_count: 2` (raised from the default
  `1` on 2026-07-19 (2), alongside the sampling-noise restoration above - a second optimization
  pass per control cycle lets MPPI refine away from a bad warm-started sequence within the same
  cycle rather than relying on next-cycle sampling luck; doubles per-cycle optimizer CPU cost,
  live-verified to still fit within `controller_frequency`'s 50ms budget, though it does produce
  occasional "Control loop missed its desired rate" warnings under load), `prune_distance: 1.7` m
  (how far ahead along the path is considered for cost evaluation), `transform_tolerance: 0.1` s
  (TF lookup slack), `temperature: 0.3` / `gamma: 0.015` (MPPI's own softmax-weighting and
  control-cost trade-off parameters - core to the algorithm, not typically hand-tuned without
  deeper MPPI familiarity), `motion_model: "Ackermann"` / `AckermannConstraints.min_turning_r:
  0.5` (this vehicle's true hard turning-radius limit - see `TUNING.md` §2 for why this replaces
  RPP's kinematic assumptions, and `planner_server.GridBased.minimum_turning_radius` below for why
  this is deliberately *not* kept equal to the planner's own minimum anymore), `visualize: true`
  (enabled 2026-07-18 (4) at user request - publishes two debug-only topics for RViz,
  `/trajectories` (every sampled candidate trajectory plus the chosen optimal one, each control
  cycle) and `transformed_global_plan` (the local, controller-side portion of the plan MPPI is
  actively tracking); adds marker-array publishing overhead, so turn back off for normal
  (non-debugging) runs), `enforce_path_inversion: true` / `inversion_xy_tolerance: 0.2` /
  `inversion_yaw_tolerance: 0.4` (dedicated cusp/reversal handling - without
  `enforce_path_inversion`, MPPI does not correctly treat a path with a direction change as
  "drive to the cusp, then continue past it," which was the actual cause of the robot not
  following the planned path at all right after the initial MPPI switch - see `TUNING.md` §2 and
  `CHANGELOG.md`'s 2026-07-14 (10) entry), `critics: [...]` (which cost functions score each
  candidate trajectory - see `TUNING.md` §2 for the list and what each does; now includes the
  custom `EarlyCommitCritic` alongside the stock nav2 critics - see below).

  **Per-critic parameter overrides** (defaults verified directly against the Humble source,
  `github.com/ros-navigation/navigation2` humble branch `nav2_mppi_controller/src/critics/`,
  rather than guessed, since the local headers only expose parameter names, not default values):
  - `EarlyCommitCritic` - a custom critic defined in *this* package (`src/critics/
    early_commit_critic.cpp`, class `mppi::critics::EarlyCommitCritic`, deliberately declared in
    namespace `mppi::critics` rather than `qcar_updated::critics` - `CriticManager::getFullName()`
    hardcodes that prefix when resolving names from the `critics:` list, so any other namespace
    is unreachable regardless of how it's exported via pluginlib), added 2026-07-18 (9) - see
    `CHANGELOG.md` for the full investigation. Scores only the first `early_time_steps` (`10`) of
    each sampled trajectory against the bearing to the path point `offset_from_furthest` (`3`)
    indices ahead of current progress, with no `threshold_to_consider`/`max_angle_to_furthest`
    gate (unlike stock `PathAngleCritic`) - makes MPPI commit to the path's initial curvature from
    the very first control cycle instead of settling into a near-straight local optimum that never
    corrects. `active_path_points: 8` (lowered from `15` on 2026-07-19 (5)) gates it off once the
    robot has progressed past the first 8 path points - live-tested with no such gate at all and
    it broke ordinary path-following for the rest of the trip, since this critic's fixed near-term
    target stops being a meaningful signal once real progress has been made. Lowered from the
    original `15` after a safety-relevant report (robot turning before the planned curve, risking
    obstacle contact): on a goal whose path ran straight for ~1m before curving, the curve started
    around path point 15-16 - inside the old 15-point window - so this highest-weighted, ungated
    critic reached toward the curve well before the vehicle got there (live-measured lateral
    position up to 3x the planned value at matching points). `8` measurably reduced this (~1.9x at
    the same point) without regressing the original curve-at-trip-start case. See `CHANGELOG.md`
    2026-07-19 (5). `forward_preference: false` (added 2026-07-19) - a
    `SmacPlannerHybrid` Reeds-Shepp path can require a reverse (K-turn) segment anywhere a sharp
    reorientation is needed, not just at trip start; at `false` the critic scores yaw against
    whichever of (bearing-to-target, bearing-to-target + 180deg) is closer - the same "reversing
    allowed" convention as `nav2_mppi_controller`'s own `utils::posePointAngle()` - instead of
    assuming forward-only travel and fighting a correct reverse maneuver, which was live-confirmed
    (via a `/plan` dump mid-stall) to freeze the robot again later in a trip, the same way the
    ungated version once froze it at trip start. See `CHANGELOG.md` 2026-07-19 (1). `cost_weight:
    15.0`.
  - `CostCritic.consider_footprint: true` - defaults to `false` upstream; collision/obstacle cost
    was otherwise evaluated at the robot's center point only, not its actual elongated footprint,
    a real bug and a plausible direct cause of corner-cutting near obstacles for a car-shaped
    robot. See `CHANGELOG.md` 2026-07-14 (11).
  - `GoalCritic.cost_weight: 8.0` - raised from the default `5.0`; final position was reported
    inaccurate (outside the `xy_goal_tolerance`) even when nav2 considered the goal "reached."
    Strengthens convergence to the exact goal point within `GoalCritic`'s `1.4m`
    `threshold_to_consider`, relative to `PathAlignCritic`'s competing weight of `10.0` over the
    same final approach.
  - `GoalAngleCritic.cost_weight: 10.0` - raised from the default `3.0` on 2026-07-18 (5); fixes
    "robot doesn't move when given a new goal that's mostly a reorientation" (e.g. re-sending the
    same position with a different final yaw shortly after reaching a goal). Root cause: `PathAlign`/
    `PathFollow`/`PathAngle` Critic all gate on the robot's *current* distance to the goal, not
    progress along the plan - when a new goal sits at (near) the robot's current position, that
    distance is ~0 from the first control cycle, disabling all three for the entire maneuver, and
    combined with `ConstraintCritic`'s turning-radius penalty, the default-weight `GoalAngleCritic`
    alone was too weak to pull MPPI out of a near-zero-velocity local optimum (an Ackermann vehicle
    cannot change heading at zero linear velocity). `30.0` was tried first and fixed the
    reorientation case but regressed a normal goal's final approach; `10.0` was live-tested clean
    on both. Converts a permanent hang into an eventual, automatic (`progress_checker` + recovery)
    convergence, rather than a fully clean instant fix - the underlying critic-gating gap is still
    present.
  - `PathFollowCritic.threshold_to_consider: 0.15` (lowered from `0.5` on 2026-07-18 (5)) - shrinks
    the window, near the end of a normal (non-reorientation) approach, where the planned path stops
    being enforced before goal-focused critics take over. Does **not** fix the reorientation-in-
    place bug above - confirmed via live testing this has zero effect there, since that bug's gate
    is already ~0 regardless of the threshold's value.
  - `PathFollowCritic.offset_from_furthest: 3` (lowered from the default `6`) - this critic pulls
    each sampled trajectory's endpoint toward the path point this many indices ahead of current
    progress; too far ahead can land past the start of an upcoming curve, pulling the executed
    heading toward that curve before the robot has physically reached it (reported live as
    "turning early").
  - `PathAngleCritic.offset_from_furthest: 2` (lowered from the default `4`) - same reasoning and
    mechanism as `PathFollowCritic` above, but for heading rather than position: steers the
    robot's heading toward the path point this many indices ahead, the more direct cause of
    visibly "turning early" into a curve.
  - `PathAlignCritic.offset_from_furthest: 3` (lowered from the default `20` on 2026-07-18 (3)) -
    works differently here than in `PathFollowCritic`/`PathAngleCritic` above: it's a gate, not a
    look-ahead target (`if (path_segments_count < offset_from_furthest_) return;`), so
    `PathAlignCritic` (the highest-weighted critic in use, at `10.0`) was completely inactive for
    the first `offset_from_furthest` path points of every trip - if a curve starts early in the
    planned path, the strongest path-adherence critic simply wasn't running yet during that
    segment.
  - `PathAlignCritic.threshold_to_consider: 0.15` (lowered from `0.5` on 2026-07-18 (5)) - same
    reasoning and same "does not fix the reorientation-in-place bug" caveat as
    `PathFollowCritic.threshold_to_consider` above.
  - `PathAlignCritic.cost_weight: 10.0` - the stock default, left explicit after a 2026-07-19 (3)
    investigation into corner-cutting (the robot tracking a looser curve than the plan calls for,
    a real side effect of the wider `vx_std`/`wz_std` restored above). Raising this weight (`12.0`,
    `14.0`) was live-tested and did **not** produce a reliable fix - results didn't order sensibly
    with the weight (higher values overshot the *other* direction and stalled, and a repeat run at
    the default swung wider still than either), indicating run-to-run MPPI sampling variance at
    this wider noise setting, not this weight, dominates the outcome. See `CHANGELOG.md`
    2026-07-19 (3).

## `smoother_server`

- `smoother_plugins: ["simple_smoother"]`, `SimpleSmoother`: `tolerance: 1.0e-10` (convergence
  threshold - how small a change counts as "smoothing done"), `max_its: 1000` (iteration cap),
  `do_refinement: true` (runs an extra pass to reduce residual curvature/kinks). Post-processes
  the raw grid-planner path into something less jagged before handing it to the controller.

## `planner_server` (global path planning)

- `expected_planner_frequency: 20.0` (Hz) - used only to warn if planning is taking longer than
  expected, not an actual rate limiter.
- `GridBased` (`nav2_smac_planner/SmacPlannerHybrid`, replacing the original `NavfnPlanner` as of
  2026-07-14 - see `CHANGELOG.md`): a search-based planner that's kinematically aware of this
  Ackermann car's turning-radius limit. `downsample_costmap: false` / `downsampling_factor: 1` (no
  costmap downsampling - fine at this map's small size, would trade search speed for resolution
  on a larger map), `tolerance: 0.5` m (same meaning as before), `allow_unknown: true` (same),
  `max_iterations: 1000000` / `max_on_approach_iterations: 1000` (search effort caps),
  `max_planning_time: 5.0` s (hard wall-clock cap per planning attempt),
  `motion_model_for_search: "REEDS_SHEPP"` (allows reverse/K-turn segments, matching
  `vx_min < 0` in `FollowPath` - required for this vehicle per user need. This setting has a
  documented history of instability when paired with `RegulatedPurePursuitController`'s
  lookahead-point path following in this Humble nav2 version - tried `"DUBIN"` (forward-only, no
  cusps) as a stable workaround, then switched the *controller* to `MPPIController` instead
  (see `TUNING.md` §2), which doesn't share RPP's failure mode, allowing `"REEDS_SHEPP"` to be
  restored - see `TUNING.md` §4 and `CHANGELOG.md`'s 2026-07-14 entries for the full history),
  `angle_quantization_bins: 72` (5deg heading resolution in the search space),
  `analytic_expansion_ratio: 3.5` / `analytic_expansion_max_length: 3.0` (tuning for the
  shortcut/analytic-expansion optimization that tries a direct Reeds-Shepp curve to the goal
  before falling back to full graph search), `minimum_turning_radius: 0.7` m (padded over the
  car's true ~0.445m physical minimum - see `TUNING.md` §4 for the derivation. Raised from `0.5`
  on 2026-07-19 (2) - `0.5` exactly matched `FollowPath.AckermannConstraints.min_turning_r` (the
  controller's own hard limit), which sounds safe but isn't: the planner was then free to produce
  curves at exactly the tightest radius MPPI is allowed to drive, leaving MPPI's randomly sampled
  candidate trajectories zero margin - live-confirmed via a circle fit on a real `/plan` dump
  (fitted radius `0.49999999m`, dead on the minimum) that the robot froze on. `0.7` here vs `0.5`
  in the controller gives every planned curve real slack before it's anywhere near what MPPI
  actually treats as too tight - see `CHANGELOG.md` 2026-07-19 (2)),
  `reverse_penalty: 4.0` (raised from `2.0` - discourages planning a reverse segment unless the
  goal orientation truly requires one, minimizing unnecessary reversing) /
  `non_straight_penalty: 1.2` / `change_penalty: 3.0` / `cost_penalty: 2.0` /
  `retrospective_penalty: 0.025` (search cost weights shaping path preference - see `TUNING.md`
  §4 for which to adjust for which symptom; `change_penalty` raised from the default `0.0` on
  2026-07-18 (7) - `node_hybrid.cpp`'s own comment on this term is literally "penalizes
  wiggling," applied whenever consecutive motion primitives change turning direction. At `0.0`
  there was no cost at all for the planner inserting a short, unnecessary direction-changing
  segment - observed live as a small reversal immediately followed by forward motion, right at
  the start of nearly every plan whose initial heading doesn't already match the path tangent,
  which was driving a "stuck at path start" symptom that turned out to be a planner issue, not an
  MPPI tuning one), `lookup_table_size: 20.0` (size of the precomputed
  motion-primitive distance heuristic table), `cache_obstacle_heuristic: false` (recompute the
  obstacle heuristic fresh each planning call rather than caching between calls - safer given
  costmaps update from live sensor data), `smooth_path: true` (planner's own internal smoothing -
  see the `smoother_server` note above, since this package's BT trees don't separately invoke it).

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
- `footprint: [[0.22,0.09],[0.22,-0.09],[-0.22,-0.09],[-0.22,0.09]]` (both, as of 2026-07-14 (11))
  - real polygon matching the car's measured body extent (`models/qcar/QCarBody.stl`: 0.409m long
  x 0.156m wide), replacing the earlier `robot_radius: 0.15` circular approximation. Now directly
  load-bearing for `controller_server.FollowPath.CostCritic.consider_footprint: true` and for
  `planner_server.GridBased`'s own footprint-aware collision checking during planning.
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

- `spin` is registered in `behavior_server.behavior_plugins` despite being physically infeasible
  for this car - check whatever BT XML is actually active to see if it's really excluded from
  the runtime recovery tree (this params file alone doesn't control that).
