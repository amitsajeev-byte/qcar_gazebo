# Navigation accuracy tuning reference

Parameters worth adjusting if Nav2 goal-reaching accuracy needs improvement, organized by
layer. See `CHANGELOG.md` / the 2026-07-13 fixes for the bugs already root-caused and fixed in
this stack (wheel collision shape, friction, steering PID gain) before this list was compiled -
this doc assumes those are already applied.

## Known mismatches worth fixing first

- **Costmap footprint is a circle** (`robot_radius: 0.15`, `nav2_params.yaml:190`/`226`) but the
  car is elongated (wheelbase 0.257m + overhangs, roughly 0.36m long by ~0.11m wide per the
  URDF). A circle either clips corners (if sized for the width) or blocks passable narrow gaps
  (if sized for the length). Swapping to a proper `footprint: [[x,y], ...]` polygon matching the
  chassis is usually a real accuracy win for car-shaped robots, especially mid-turn.

## 1. Physics / drive plugin - `urdf/qcar_model.xacro` (`gazebo_ros_ackermann_drive` block, ~line 305)

| Param | Current | Effect |
|---|---|---|
| `left/right_steering_pid_gain` | `3.0 0 0.1` | How well actual steer angle tracks commanded. Too low -> undershoot; too high -> oscillation/instability. |
| `linear_velocity_pid_gain` | `0.3 0 0.005` | Wheel-speed tracking accuracy - same trade-off. |
| Wheel collision `radius` (~line 142) | `0.036` | Feeds the plugin's `wheel_radius_` used to convert wheel spin -> odom linear velocity. A mismatch creates a **systematic odometry scale error** that accumulates over a whole path (padded +3mm over the mesh's true 0.033m for ground contact - worth testing a smaller pad if traction is solid). |
| `mu1`/`mu2` on wheels | `1.0` | Traction. Lower -> more slip/skid (less accurate); unrealistically high can cause its own instability. |
| `max_steer` | `0.5236` (30°) | Real hardware limit -> sets true min turning radius (~0.45m). Nav2's planner doesn't know this limit, so paths can ask for tighter turns than physically possible. |
| Wheel inertia (`ixx/iyy/izz`) | `0.0012/0.0022/0.0012` | Too low -> numerically unstable at higher PID gains (hit an `Ogre::AxisAlignedBox` crash once at the old, tinier value); too high -> sluggish response. |

## 2. Path following - `FollowPath` / RegulatedPurePursuitController - `config/nav2/nav2_params.yaml:132`

| Param | Current | Effect |
|---|---|---|
| `desired_linear_vel` | `0.3` | Slower = more accurate tracking through turns, generally. |
| `lookahead_dist` / `min_/max_lookahead_dist` | `0.6 / 0.3 / 0.9` | Larger lookahead -> smoother but corner-cutting; smaller -> tighter tracking but jerkier, more oscillation risk. Usually the highest-leverage knob for "cuts corners" vs "hugs the path." |
| `regulated_linear_scaling_min_radius` / `_min_speed` | `0.9 / 0.25` | Slows the car automatically as path curvature tightens below this radius. Consider tying `min_radius` closer to the car's actual ~0.45m min turning radius. |
| `use_cost_regulated_linear_velocity_scaling` | `false` | Enabling slows the car near obstacles/costmap inflation - can reduce clipping. |
| `max_angular_accel` | `2.0` | If lower than what the steering PID can actually achieve, commands get rate-limited and lag the plan; if higher, may ask for steering-angle rates the PID can't track in time. |
| `use_fixed_curvature_lookahead` | `false` | `min_turning_radius` is **not available** as a param in this Humble build (checked via `strings` on the compiled `.so` - added in a later nav2 release), so this and curvature-lookahead tuning won't behave like newer nav2 docs describe. |

## 3. Goal checker - `nav2_params.yaml:127`

| Param | Current | Effect |
|---|---|---|
| `xy_goal_tolerance` | `0.1` | Tighter = more accurate stop, but harder to satisfy (may cause hunting/oscillation near the goal for a car that can't easily fine-adjust position). |
| `yaw_goal_tolerance` | `0.7` (~40°) | Quite loose currently - tightening forces closer final heading match but is harder for a car-like robot with no in-place rotation. |

## 4. Global planner - `GridBased` / NavfnPlanner - `nav2_params.yaml:172`

NavfnPlanner is curvature-agnostic - it will plan paths with turns tighter than the car can
physically execute, and RPP just clamps/deviates when it hits that limit. If accuracy problems
concentrate around tight corners specifically, the real fix is swapping to `nav2_smac_planner`'s
Hybrid-A* (Ackermann-aware, respects min turning radius) rather than tuning RPP further - a
bigger change, but worth knowing it's on the table.

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
| `robot_radius` -> footprint | `0.15` (circle) | See "Known mismatches" above - biggest single accuracy lever if paths are clipping corners or avoiding passable gaps. |

## Suggested debugging order

1. Fix the footprint (cheap, structural, no physics retuning needed).
2. Watch whether errors concentrate at tight turns (-> tune `lookahead_dist` /
   `regulated_linear_scaling_min_radius`, or consider Smac Hybrid-A*) vs. general position drift
   over long paths (-> check AMCL `alpha` params and the wheel-radius odometry scale first).
