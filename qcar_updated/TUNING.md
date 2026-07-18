# Navigation accuracy tuning reference

Parameters worth adjusting if Nav2 goal-reaching accuracy needs improvement, organized by
layer. See `CHANGELOG.md` / the 2026-07-13 fixes for the bugs already root-caused and fixed in
this stack (wheel collision shape, friction, steering PID gain) before this list was compiled -
this doc assumes those are already applied.

## Known mismatches (fixed)

- ~~Costmap footprint is a circle~~ **Fixed 2026-07-14 (11)**: `local_costmap`/`global_costmap`
  now use a real polygon `footprint`, measured directly from `models/qcar/QCarBody.stl` (0.409m
  long x 0.156m wide, rounded outward with a small margin) instead of `robot_radius: 0.15`. This
  became directly load-bearing once `CostCritic.consider_footprint: true` was enabled in §2 below
  - footprint-aware collision checking against an undersized circle wouldn't have actually
    protected the car's real front/rear overhangs.

## 1. Physics / drive plugin - `urdf/qcar_model.xacro` (`gazebo_ros_ackermann_drive` block, ~line 305)

| Param | Current | Effect |
|---|---|---|
| `left/right_steering_pid_gain` | `3.0 0 0.1` | How well actual steer angle tracks commanded. Too low -> undershoot; too high -> oscillation/instability. |
| `linear_velocity_pid_gain` | `0.3 0 0.005` | Wheel-speed tracking accuracy - same trade-off. |
| Wheel collision `radius` (~line 142) | `0.036` | Feeds the plugin's `wheel_radius_` used to convert wheel spin -> odom linear velocity. A mismatch creates a **systematic odometry scale error** that accumulates over a whole path (padded +3mm over the mesh's true 0.033m for ground contact - worth testing a smaller pad if traction is solid). |
| `mu1`/`mu2` on wheels | `1.0` | Traction. Lower -> more slip/skid (less accurate); unrealistically high can cause its own instability. |
| `max_steer` | `0.5236` (30°) | Real hardware limit -> sets true min turning radius (~0.45m). Nav2's planner doesn't know this limit, so paths can ask for tighter turns than physically possible. |
| Wheel inertia (`ixx/iyy/izz`) | `0.0012/0.0022/0.0012` | Too low -> numerically unstable at higher PID gains (hit an `Ogre::AxisAlignedBox` crash once at the old, tinier value); too high -> sluggish response. |

## 2. Path following - `FollowPath` / MPPIController - `config/nav2/nav2_params.yaml:148`

As of 2026-07-14 (9) this is `nav2_mppi_controller::MPPIController`, not
`RegulatedPurePursuitController`. History: RPP was tuned extensively (lookahead distances, speed
regulation, replanning rate, `progress_checker` patience) trying to stabilize its handling of
path cusps (reverse/direction-change points) for goals needing a large final-heading change with
`allow_reversing: true`. It never became reliable - one goal would succeed, a geometrically
similar one would fail after 7+ minutes of retries. RPP's specific failure mode is tied to its
lookahead-point ("carrot point") pure-pursuit geometry, which has no robust handling for a large
lookahead radius straddling both sides of a cusp. MPPI is architecturally different: each control
cycle it samples many candidate trajectories and scores them against cost critics, rather than
tracking one lookahead point - it doesn't have RPP's specific failure mode, and natively supports
an Ackermann motion model with a real minimum-turning-radius constraint (verified via
`/opt/ros/humble/include/nav2_mppi_controller/motion_models.hpp`).

**Only structural parameters are set explicitly** (motion model, velocity limits, turning radius,
which critics to load) - per-critic cost weights are left at nav2's own built-in defaults, since
this package doesn't have local access to the critics' `.cpp` source to verify non-default weight
values with confidence. Retune individual critic weights only after testing these defaults live.

| Param | Current | Effect |
|---|---|---|
| `motion_model` | `"Ackermann"` | Selects `AckermannMotionModel`, which enforces `AckermannConstraints.min_turning_r` as a hard constraint on sampled trajectories (verified string present in the compiled `.so` and declared in the header). |
| `AckermannConstraints.min_turning_r` | `0.5` m | Keep in sync with `planner_server.GridBased.minimum_turning_radius` below - both represent the same physical vehicle limit. |
| `vx_max` / `vx_min` | `0.3` / `-0.2` | Forward/reverse speed limits. Reverse magnitude kept lower than forward since the drive plugin's PID (`urdf/qcar_model.xacro`, see §1) was only tuned/verified for forward speeds up to 0.3 m/s. |
| `vx_std` / `wz_std` | `0.15` / `0.3` | Lowered from the plugin's shipped defaults (`0.2`/`0.4`, verified against `nav2_mppi_controller/src/optimizer.cpp` on GitHub) on 2026-07-18, after the user shared video evidence of mid-trip path deviation. Controls how widely sampled candidate trajectories spread each cycle - tighter sampling = less wobble around the plan. Live-tested: max path deviation on the standard test goal dropped `0.447m -> 0.375m`, and final position accuracy improved sharply (`0.139m -> 0.054m`) - but total travel time increased (`122s -> 172s`), a real trade-off (tighter sampling explores less, converges more cautiously). If travel time becomes the primary complaint, raise these back toward the defaults before touching anything else. |
| `vy_max` / `vy_std` | `0.0` | Zero - this is a non-holonomic vehicle, no lateral motion. |
| `wz_max` | `1.9` | Max angular velocity considered during sampling. |
| `batch_size` / `time_steps` / `model_dt` | `2000` / `56` / `0.05` | Core MPPI sampling parameters - more samples/longer horizon = better trajectory quality but more CPU per control cycle. Reduce `batch_size` first if `controller_frequency: 20.0` (above) starts getting missed under load. |
| `enforce_path_inversion` | `true` | **Critical for this vehicle.** MPPI has dedicated logic for a path containing a cusp/reversal (as `SmacPlannerHybrid`'s Reeds-Shepp K-turns produce) - without this, it defaults to `false` and does not correctly treat the path as "drive to the cusp, then continue past it." Confirmed (after the initial MPPI switch) to be the actual cause of the robot not following the planned path at all, not a critic-weight issue - see `CHANGELOG.md`'s 2026-07-14 (10) entry. If reversing is ever disabled again (`vx_min >= 0`, `motion_model_for_search: "DUBIN"`), this can safely go back to `false` since there'd be no cusps to handle. |
| `inversion_xy_tolerance` / `inversion_yaw_tolerance` | `0.2` m / `0.4` rad (~23°) | Position/heading window within which the robot is considered to have reached the cusp, so MPPI advances to the path segment beyond it. Tighter values demand a more precise stop at the cusp before continuing (more correct-looking but slower); looser values move on sooner (faster but the "K-turn" may look less clean). |
| `critics` | `["ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic", "PathAlignCritic", "PathFollowCritic", "PathAngleCritic"]` | Which cost functions score each candidate trajectory. `PreferForwardCritic` was **removed** 2026-07-15 (3) - its source (fetched from GitHub, not guessed) showed it penalizes any reverse velocity for as long as the robot is >0.5m from goal, while `PathFollowCritic` (before its own fix below) stopped enforcing the plan inside 1.4m - the 0.5-1.4m gap let `PreferForwardCritic` actively fight a planned reversal with nothing defending it. Live-tested: fixed a K-turn goal that previously failed catastrophically (2.2m/177deg error) down to 0.145m/0.22deg. Default `cost_weight`s of the remaining critics, verified directly against the Humble source (`nav2_mppi_controller/src/critics/*.cpp` on GitHub): `ConstraintCritic` 4.0, `CostCritic` 3.81, `GoalCritic` 5.0, `GoalAngleCritic` 3.0, `PathAlignCritic` 10.0, `PathFollowCritic` 5.0, `PathAngleCritic` 2.0. |
| `CostCritic.consider_footprint` | `true` | Defaults to **false** - was a real bug in the original MPPI config (docs claimed `true`, the yaml never set it), meaning collision/obstacle cost was evaluated at the robot's center point only, not its actual elongated footprint. A plausible direct cause of corner-cutting near obstacles. **Depends on the costmap's own footprint being accurate** - see the circular-footprint mismatch flagged at the top of this doc; footprint-aware collision checking against an undersized circular approximation won't fully protect the car's real overhangs. |
| `GoalCritic.cost_weight` | `8.0` | Raised from the default `5.0` - final position was reported inaccurate even when nav2 considered the goal "reached." Live-tested alongside the `PreferForwardCritic` removal - contributed to convergence within 0.14-0.20m across two test goals (down from >0.6m before both fixes). |
| `GoalAngleCritic.cost_weight` | `10.0` | Raised from the default `3.0` on 2026-07-18 (5) - fixes the robot not moving on a new goal that's mainly a reorientation (same/near position, new final yaw). Root cause: `PathAlignCritic`/`PathFollowCritic`/`PathAngleCritic`'s `threshold_to_consider` gates on distance from the robot's **current** pose to the goal (`utils::withinPositionGoalTolerance`), which is ~0 for this scenario from the first control cycle - disabling all three for the entire maneuver, leaving `GoalAngleCritic` as the only active guidance against `ConstraintCritic`'s turning-radius penalty. Live-tested: measurably improves recovery (often converges directly, otherwise unstuck by `progress_checker`'s retry loop within ~30-90s) with no regression on normal short goals (~5-11s, same as baseline). **Not a complete fix** - still occasionally fails outright on retry-budget exhaustion in live testing, since MPPI's sampling is stochastic and the underlying critic-gating gap is unchanged. `30.0` converges faster/more directly but was live-verified to regress a separate normal goal's final approach - reverted. See `CHANGELOG.md` 2026-07-18 (5). |
| `PathFollowCritic.threshold_to_consider` / `PathAlignCritic.threshold_to_consider` | `0.15` | Lowered from the defaults `1.4` / `0.5` - closes the "dead zone" described above so path-following guidance stays active closer to the final approach. **Does not fix** the reorientation-in-place bug described in the `GoalAngleCritic` row above - live-verified to have zero effect on that case, since the gate is based on current-position-to-goal distance, which is already ~0 for that scenario regardless of this value. |
| `PathFollowCritic.offset_from_furthest` / `PathAngleCritic.offset_from_furthest` | `3` / `2` | Lowered from the defaults `6` / `4` on 2026-07-18 (2) - user reported (and shared video of) the robot turning early, before an upcoming curve in the planned path actually starts. Confirmed via the real scoring code (`path_follow_critic.cpp` / `path_angle_critic.cpp` on GitHub): both critics target a path point this many *indices* ahead of the robot's current furthest-reached progress - `PathAngleCritic` steers heading toward it, `PathFollowCritic` pulls trajectory endpoints toward it. A target that far ahead can land past the start of a curve, pulling the executed heading toward it prematurely. |
| `PathAlignCritic.offset_from_furthest` | `3` | Lowered from the default `20` on 2026-07-18 (3), after the fix above alone didn't resolve the reported symptom. Works completely differently here than in the other two critics - it's a **gate**, not a look-ahead target (`path_align_critic.cpp`): `PathAlignCritic` (the highest-weighted critic in use, at `10.0`) is entirely inactive for the first `offset_from_furthest` path points of every trip. At the default `20`, the strongest path-adherence critic simply wasn't running yet for an early curve. Live-tested on the same isolated forward-curve goal as the fix above: max path deviation tightened `0.123m -> 0.091m -> 0.083m` across the two fix rounds, with every sampled point staying under 9cm from the path. **User-confirmed fixed** against real RViz recordings (the green line in both shared videos was the actual `/plan` topic) - see `CHANGELOG.md`'s 2026-07-18 (3) entry. |

If MPPI still doesn't reliably handle reversing near a goal after `enforce_path_inversion: true`,
that would indicate the problem is genuinely in the planned path (SmacPlannerHybrid, §4) rather
than the controller - a good diagnostic split RPP couldn't offer, since RPP's own instability was
always a confound.

**Confirmed as of 2026-07-18 (6): this does happen** - a path with a reverse-then-forward
(cusp) segment can get the robot stuck in a `Failed to make progress` / replan loop that never
reaches the goal. Not fixed yet - see `CHANGELOG.md` 2026-07-18 (6) for the full investigation
(live-reproduced, ruled out (5)'s changes as the cause, root-cause candidate identified in
`path_handler.cpp`'s `isWithinInversionTolerances` combined with `SmacPlannerHybrid` inserting a
fresh small corrective reversal on nearly every replan). `inversion_xy_tolerance` /
`inversion_yaw_tolerance` (both under `FollowPath` above, defaults `0.2`m / `0.4`rad) are the
most likely next levers, but loosening them live only partially helped, so nothing's been
committed to the YAML for this yet.

## 3. Goal checker - `nav2_params.yaml:127`

| Param | Current | Effect |
|---|---|---|
| `xy_goal_tolerance` | `0.12` | Widened slightly from `0.1` on 2026-07-15 (3) - a live-tested K-turn goal converged to 0.145m and cycled through repeated abort/retries instead of latching "reached" at the old, stricter tolerance. Tighter = more accurate stop, but harder to satisfy (may cause hunting/oscillation near the goal for a car that can't easily fine-adjust position). |
| `yaw_goal_tolerance` | `0.3` (~17°) | A real, meaningful tolerance - final heading matters for this deployment (confirmed by the user), so don't loosen this drastically as a way to avoid goal-approach oscillation; fix the actual path-following behavior instead (see §2/§4). |

## 4. Global planner - `GridBased` / SmacPlannerHybrid - `nav2_params.yaml:190`

As of 2026-07-14 this is `nav2_smac_planner/SmacPlannerHybrid`, not the original
`nav2_navfn_planner/NavfnPlanner` - switched because `NavfnPlanner` is curvature- and
heading-agnostic (plans paths with turns tighter than the car can physically execute, and has no
concept of arrival orientation at all), which was the root cause of a goal-approach oscillation
where `RegulatedPurePursuitController` kept trying to improvise a final-heading correction
reactively. See `CHANGELOG.md`'s 2026-07-14 entries for the full diagnosis history.

**`motion_model_for_search` is `"REEDS_SHEPP"` (reversing allowed).** History: this was the
original choice, found unstable when paired with `RegulatedPurePursuitController` (RPP doesn't
handle path cusps robustly in this Humble nav2 version - confirmed via live testing), switched to
`"DUBIN"` (forward-only, no cusps) as a stable workaround, then switched back once the local
controller itself was replaced with MPPI (§2 above) - MPPI's trajectory-sampling approach doesn't
share RPP's cusp/carrot-point failure mode, so `"REEDS_SHEPP"` should now be followable reliably.
`reverse_penalty` (below) is still raised from its default to prefer forward-only routes when
possible, purely to minimize unnecessary reversing (less wear, shorter/simpler paths) - not as an
instability workaround anymore.

| Param | Current | Effect |
|---|---|---|
| `motion_model_for_search` | `"REEDS_SHEPP"` | Allows reverse/K-turn segments in the planned path. `"DUBIN"` (forward-only, no cusps) remains the fallback if instability somehow returns with MPPI too - see the history above and in `CHANGELOG.md`. |
| `minimum_turning_radius` | `0.5` m | Padded slightly over the car's true ~0.445m minimum (wheelbase / tan(max steer)) so the planner never asks for a curve tighter than the steering PID can actually deliver. Lower cautiously if `max_steer` or the steering PID response is later retuned to allow tighter turns. |
| `reverse_penalty` | `4.0` | Raised from `2.0` - makes the planner prefer a forward-only route even if longer, only actually planning a reverse segment when the goal orientation truly can't be reached forward-only. Fewer cusps encountered in practice = fewer chances to hit the residual instability. Raise further if oscillation returns and you'd rather the car take a longer forward route than reverse at all in most cases. |
| `non_straight_penalty` / `change_penalty` | `1.2` / `0.0` | Discourage unnecessary curvature / direction changes in the planned path, for smoother routes. |
| `cost_penalty` | `2.0` | Weight on costmap cost in the search - higher pushes the path further from inflated obstacle zones. |
| `smooth_path` | `true` | Planner's own internal path smoothing - the BT trees in this package don't separately invoke `smoother_server`, so this is the only smoothing actually applied. |
| `max_planning_time` | `5.0` s | Hard cap per planning attempt - Hybrid-A* is more expensive than NavfnPlanner's Dijkstra/A*, worth watching if replanning starts feeling sluggish on a larger map. |
| `angle_quantization_bins` | `72` | Heading resolution of the search space (5 deg per bin) - finer bins = more precise final heading but slower search. |

If cusp/reversing behavior is still unreliable after the MPPI switch (§2), that now points at the
planned path itself rather than path-following - inspect `/plan` during a large-heading-change
goal to check whether `SmacPlannerHybrid` is producing a sane K-turn geometry, before assuming
it's a controller problem again.

## 5. AMCL - `nav2_params.yaml:10-44`

`robot_model_type: "nav2_amcl::DifferentialMotionModel"` (line 35) is correct as-is and does
**not** need to change. Checked directly against the `nav2_amcl` source
(`/opt/ros/humble/include/nav2_amcl/motion_model/*.hpp`): both shipped motion models
(`DifferentialMotionModel`, `OmniMotionModel`) consume the same raw odometry pose delta and the
same 5 `alpha` noise params - the model name doesn't refer to the vehicle's real drivetrain, it
refers to how the delta gets decomposed into particle noise. "Differential" = rotate-translate-
rotate, i.e. non-holonomic (can't move sideways without also turning/driving) - true for an
Ackermann car exactly as much as a diff-drive robot. "Omni" adds an explicit lateral *strafe*
noise term for robots that can genuinely move sideways (mecanum/omni wheels), which this car
cannot do - switching to it would inject particle-spread hypotheses for motion the robot can
never produce, which can hurt localization efficiency rather than help it. There is no
Ackermann-specific motion model in stock Nav2; `DifferentialMotionModel` is the standard choice
for any non-holonomic robot, car-like or not.

The actual accuracy levers here:

| Param | Current | Effect |
|---|---|---|
| `alpha1-alpha5` | `0.2` each | Motion-model noise the filter expects per unit of travel/rotation. If real odometry is noisier than this (e.g. wheel slip), particles under-spread and localization overconfidently drifts. Raising these lets AMCL correct faster from odometry error at the cost of noisier pose estimates. |
| `update_min_d` / `update_min_a` | `0.25 / 0.2` | How far the robot must move before AMCL re-localizes against the scan. Smaller = more frequent corrections (better accuracy, more CPU). |
| `laser_max_range`, `z_hit`, `z_rand`, `sigma_hit` | `12.0 / 0.5 / 0.5 / 0.2` | Scan-matching confidence model - `z_hit` vs `z_rand` balance affects how much AMCL trusts clean vs noisy lidar returns. |

## 6. Costmap - `nav2_params.yaml:187-269`

| Param | Current | Effect |
|---|---|---|
| `inflation_radius` / `cost_scaling_factor` | `0.55 / 3.0` | Controls how far obstacles push the planned path away - directly shapes how tight a path the planner will attempt near walls. |
| `footprint` | `[[0.22,0.09],[0.22,-0.09],[-0.22,-0.09],[-0.22,0.09]]` | Real polygon matching the car's measured body extent, replacing the old `robot_radius: 0.15` circle as of 2026-07-14 (11) - see "Known mismatches" above. If this is ever revised (new mesh, added bumpers/sensors changing the real extent), remeasure from the STL rather than eyeballing it, and keep `local_costmap`/`global_costmap` in sync. |

## Suggested debugging order

1. Fix the footprint (cheap, structural, no physics retuning needed).
2. Watch whether errors concentrate at tight turns (-> tune `lookahead_dist` /
   `regulated_linear_scaling_min_radius`, or consider Smac Hybrid-A*) vs. general position drift
   over long paths (-> check AMCL `alpha` params and the wheel-radius odometry scale first).
