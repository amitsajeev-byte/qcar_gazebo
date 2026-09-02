# Changelog

## 2026-09-01 - Reverse gain tuning parked: REVERSE_DEADBAND=0.055, REVERSE_GAIN=0.04 (fresh battery)

Closing entry for today's reverse-gain investigation (see the two entries below for the fuller
history). Final values after iterative live tuning against a fresh battery, each round retested
via `scripts/drive_and_log.py` (0.3 m/s, 2.5m, confirmed clear space each time):
`REVERSE_DEADBAND=0.055` (unchanged from the original fit), `REVERSE_GAIN` walked
0.09 -> 0.075 -> 0.065 -> 0.056 -> 0.04. Also nudged `THROTTLE_GAIN` (forward) up slightly,
0.0667 -> 0.070, at the same time to help the two converge rather than only cutting reverse.

**Final matched result**: forward peak ~0.34 m/s, reverse peak ~0.34 m/s, both clean (no stall)
at a 0.3 m/s command, both taking ~10s for the same 2.5m distance - the closest forward/reverse
match all session.

**Important caveat, not resolved**: mid-session, forward itself hit the exact same "long dead
stall (31s) then sudden clean breakaway" pattern that reverse had been showing all day - at a
command (0.3 m/s) and gain forward had run cleanly at a dozen+ times before. This is strong
evidence the underlying issue is a real, direction-independent mechanical stiction/inconsistency
(something intermittently catching, in either direction), not a pure forward/reverse calibration
gap - reverse likely just showed it more often today because its lower duty margin made the same
underlying stall condition easier to trigger. Also confirmed the whole duty->speed relationship
is battery-voltage-dependent (a battery swap mid-session shifted both forward's AND reverse's
peak speed at the same duty, proportionally) - re-verify these constants after any future
battery change, don't assume they hold.

**Still outstanding**: the physical hands-on check (power off, spin wheels by hand, compare
forward/reverse rotation for binding/roughness) was requested repeatedly this session and never
completed. Do this before fully trusting reverse for a demo/K-turn scenario, especially under
time pressure - see qcar_reverse_gain_asymmetry_investigation project memory for the full
history and this open question.

## 2026-09-01 - `qcar_bridge.py` crash fix: unhandled ConnectionResetError on abrupt relay disconnect

Found on hardware: after a battery swap + QCar reboot, killed a stale `qcar_relay_node.py`
instance (still holding the old pre-reboot clock offset) to relaunch it with a fresh one. The
abrupt disconnect raised `ConnectionResetError` at `conn.recv()`, which was unhandled - crashed
the whole `qcar_bridge.py` process (traceback, process exit) rather than just logging the
disconnect and waiting for a new connection like a clean disconnect (empty `recv()`) already did.
Added an `except OSError` alongside the existing `except socket.timeout`, mirroring the
send-side `OSError` handling that already existed a few lines below - logs and breaks to the
outer accept() loop instead of crashing. Redeployed and hash-verified.

## 2026-09-01 - `qcar_bridge.py` gained separate REVERSE_DEADBAND/REVERSE_GAIN

Live-tested "reverse takes great effort" via `scripts/drive_and_log.py` (production cmd_vel
pipeline) across many forward/reverse trials at matched commanded speeds, LiDAR on and off,
before and after a QCar reboot. Forward was clean and consistent every time (0.30 m/s -> ~0.30
m/s, 0.45 m/s -> ~0.37-0.42 m/s). Reverse was not: at identical commanded values it variously
produced a clean near-forward cruise, a 17-22s dead stall before suddenly breaking free to
near-forward speed, or a steady but roughly-half-forward-speed cruise with no stall at all -
different behavior on nominally identical trials, not a stable function of duty. That
inconsistency is itself evidence this is at least partly a real mechanical asymmetry, not purely
a calibration gap - REVERSE_DEADBAND/REVERSE_GAIN reduce how often/how badly it shows up, they
don't eliminate the underlying inconsistency. A physical hands-on check (spin the wheels by hand,
power off, compare forward/reverse rotation) was requested repeatedly this session and is still
outstanding - worth doing before considering this fully resolved.

Fitted REVERSE_DEADBAND=0.055, REVERSE_GAIN=0.09 (vs forward's 0.04/0.0667) from the
distance/time-averaged real speed at each tested duty, deliberately including the stall episodes
in the average (biases the fit toward giving reverse more duty, i.e. toward fewer stalls, rather
than toward precisely matching commanded speed on the trials that happened to go well). Wired
into qcar_bridge.py's throttle formula by sign of target_linear - previously reverse silently
reused the forward-only-calibrated THROTTLE_DEADBAND/THROTTLE_GAIN pair (2026-08-17), never
actually validated for reverse.

**Safety incident during this investigation**: an early `calibrate_throttle.py` duty sweep
(later deleted from the calibration approach in favor of production-pipeline drive_and_log.py
tests) was run without confirming clear space first - the vehicle drove into an object at an
untested/unexpectedly-fast duty value, requiring the user to lift it to stop it. See
qcar_hardware_confirm_clear_space_before_driving project memory - clear-space confirmation before
any driving action is now a hard rule, and `calibrate_throttle.py` (if still present) has a
code-enforced distance-budget abort as a backstop, not just the verbal-ask convention.

Also this session: `qcar_bridge.py` gained active checks for previously-unflagged HAL
precautions (see immediately below), and `qcar_lidar_node.py` got a documented note on a real
sudo-requirement discrepancy against `commands.md`.

## 2026-09-01 - `qcar_bridge.py` gained active checks for previously-unflagged HAL precautions

Prompted by a live hardware symptom ("reverse takes great effort") and a full audit of every
Quanser manual (`documents/*.pdf`) for anything code-relevant that wasn't already reflected in
`qcar_bridge.py`. `car.read()` already buffered `motorCurrent`/`batteryVoltage` every cycle but
nothing ever looked at them.

- **Overcurrent early warning**: `user_manual_system_hardware.pdf`'s FPGA overcurrent tiers
  (5A/8s, 10A/2s, 15A/0.5s -> forced Neutral mode, requires restarting this process per
  `user_manual_troubleshooting.pdf`'s "drive motor does not function/respond" entry) were
  invisible from the dev-PC side entirely. Worth the arithmetic: at our `THROTTLE_LIMIT=0.3` cap
  and ~12V battery, applied voltage is only ~3.6V (comfortable margin under the 5V
  stalled-motor-damage caution below), but a true stall has zero back-EMF, so locked-rotor
  current is just V/R = 3.6V / 0.470ohm (Table 7's terminal resistance) =~ 7.7A - past the 5A/8s
  tier even at our safe duty cap. Directly relevant to the reverse-effort symptom under
  investigation: a genuine stiction-floor stall could plausibly trip this without ever
  approaching max duty. Added `OVERCURRENT_TIERS` tracking that prints a console warning at 70%
  of each tier's sustained-time threshold - cannot override the FPGA, purely for visibility (the
  QCar's own LCD "Overcurrent" message remains the authoritative confirmation).
- **Stall backstop**: `user_manual_system_hardware.pdf`'s "holding the motor in a stalled
  position... over 5V can result in permanent damage" caution had no corresponding protection -
  `CMD_TIMEOUT` only catches a *stale* command (nothing arriving), not a continuously-refreshed
  command that's failing to actually move the vehicle. Added `STALL_THROTTLE_THRESHOLD`/
  `STALL_ENCODER_EPS`/`STALL_TIMEOUT`: if throttle stays above the stall threshold for 3s
  straight with no corresponding encoder motion, cut it and print why. Conservative by design -
  a safety backstop, not a fix for any underlying stiction/gain mismatch.
- **Battery voltage warnings**: `user_manual_power.pdf`'s documented 10.5V "LOW BAT" / 10.0V
  auto-shutdown thresholds now get a one-shot console warning each (`BATTERY_WARN_VOLTAGE`/
  `BATTERY_SHUTDOWN_VOLTAGE`) instead of being silently invisible outside the QCar's own LCD.
- All three reset per relay connection alongside the existing `Odometry()` reset. Moved
  `car.read()` earlier in the loop (was previously called right before `car.write()`) so these
  checks see fresh encoder/current/voltage data before the throttle decision each cycle -
  removed the now-duplicate later `car.read()` call; no other timing/ordering change.
- **`qcar_lidar_node.py`**: flagged a real discrepancy found during the same audit -
  `user_manual_troubleshooting.pdf` says LIDAR apps need `sudo`, this project's `commands.md`
  documents running this script *without* it. Both are apparently true in practice on this unit;
  left as a documented note (try sudo first if this script ever fails to open the LIDAR device
  with a permissions-looking error), not changed, since it currently works as-is.
- Everything else in the manuals (ESD/grounding, LiPo storage/charging/fire safety, header
  current limits, waterproofing, TX2 fan) is physical-handling only - nothing for code to check.
- Not yet deployed to the QCar or hardware-tested - `qcar_updated`'s copy only so far.

## 2026-09-01 - Promoted a fresh hardware map, backing up the previous one

Re-scanned the hardware map (`qcar_slam.launch.py`), `map_saver_cli`'d it, then promoted it to
the active `maps/qcar_map_hardware.yaml`/`.pgm`, backing up the prior version first as
`qcar_map_hardware_20260901.{yaml,pgm}`. This became the map used for the rest of today's nav2
hardware work, including the reverse-gain investigation entries above.

## 2026-09-01 - No-Gazebo kinematic sim-vs-hardware-map comparison mode

Wanted to compare MPPI/planner behavior against the real hardware-captured map without Gazebo's
mismatched world geometry getting in the way. Added `launch/qcar_nav2_kinematic_compare.launch.py`
(brings up nav2 + the real hardware map without Gazebo), `qcar_onboard/fake_odom_node.py`
(publishes a kinematic-integrated `/odom` + TF from `/cmd_vel` so nav2 has something to close the
loop on without a simulator or real robot), and `config/nav2/nav2_params_compare.yaml` (adds a
`static_layer` to the local costmap, which the normal hardware params don't need since real LiDAR
already provides it). Built but not deeply exercised yet this session - most of the day's actual
comparison work ended up happening directly on hardware instead.

## 2026-09-01 - RViz map display fix: QoS durability mismatch caused "No map received"

RViz showed a transform/map error, then explicit "No map received" warnings. Root cause:
`map_server` publishes `/map` with `TRANSIENT_LOCAL` durability (the standard pattern for a
latched, published-once topic), but `rviz/qcar.rviz`'s saved Map display config had `Durability
Policy` set to `Volatile` - a durability mismatch means a late-subscribing RViz never receives
the one-shot map message at all, regardless of `map_server` being healthy. Fixed by changing the
Map display's `Durability Policy` to `Transient Local` in `qcar.rviz`.

## 2026-08-31 - `config/nav2/PARAMS_REFERENCE.md` refreshed - same staleness as `TUNING.md`, same day

Companion doc to `TUNING.md` (also last touched 2026-07-20), same audit method: checked every
claimed value against the live `nav2_params.yaml`/BT XML rather than just adding new sections.

- **`smoother_server` section was entirely wrong** - still described `simple_smoother` as the
  active plugin with no mention `cusp_straightener` (custom, added 2026-08-25/26) exists at all.
  Rewritten: `cusp_straightener` documented as the active plugin (`straight_distance`,
  `extend_distance` including the 2026-08-29 curvature bug/fix, `min_lead_distance`);
  `simple_smoother` reframed as dead config, not the active one.
- **`bt_navigator.plugin_lib_names` claimed "default trees, since no custom XML is configured
  here"** - false: `launch/qcar_nav2.launch.py` overrides `default_bt_xml_filename` to this
  package's own `navigate_to_pose_w_replanning_and_recovery.xml`. Added that fact plus
  `nav2_smooth_path_action_bt_node`'s addition and the `qcar_bt_nodes`/`IsPathValidDebounced`
  tried-and-reverted note.
- **`PathFollowCritic`/`PathAngleCritic.offset_from_furthest`**: same `3`/`2` vs. live `6`/`4`
  discrepancy found in `TUNING.md` yesterday - flagged identically here (no changelog entry
  documents the reversion).
- **`amcl.recovery_alpha_fast`/`recovery_alpha_slow`**: doc claimed `0.0`/`0.0` (disabled); live
  values are `0.1`/`0.001` (nav2's own defaults) - this one was simply wrong, not a later drift.
- **`planner_server.smooth_path`/`tolerance` comments**: same "only smoothing applied" error as
  `TUNING.md`, fixed; `tolerance`'s comment was a near-content-free placeholder ("same meaning as
  before") - replaced with the real mechanism and the 2026-08-29 false-success history.
- Added `PathDeviationCritic` tried-and-reverted note, resolved the long-standing "worth a second
  look" `spin`-behavior question (answer was already sitting in the `bt_navigator` section once
  the custom BT XML fact was added), and the same "last comprehensively checked" marker pattern
  as `TUNING.md`.

## 2026-08-31 - `TUNING.md` refreshed - was last updated 2026-07-20, had drifted from the live config

Audited every "Current" value the doc claims against the live `nav2_params.yaml`/BT XML rather
than just appending new sections. Found and fixed real drift, not just missing history:

- **§4 `smooth_path` row was flatly wrong**: claimed the planner's internal smoothing is "the only
  smoothing actually applied" - false since 2026-08-25, when `smoother_server`'s custom
  `cusp_straightener` was wired into the BT via `<SmoothPath>`. This was wrong even before the
  doc's own 2026-07-20 date, since the mid-route-cusp fix that made this necessary happened later.
- **§2's `PathFollowCritic`/`PathAngleCritic.offset_from_furthest`**: doc claimed `3`/`2`
  (set 2026-07-18), but the live yaml has `6`/`4` (the stock defaults). No changelog entry
  documents this reversion - flagged as an unexplained discrepancy rather than guessed at, since
  a 2026-08-02 re-test found "no meaningful change" between the two on a different goal, which
  isn't the same as confirming which one is actually live and why.
- **§5 AMCL `alpha1-5`/`update_min_d`/`update_min_a`**: doc claimed the original `0.2`/`0.25`/`0.2`
  defaults; live values are `0.4`/`0.15`/`0.1` (raised/lowered 2026-07-15 (2), a change that
  predates this doc's own last edit and was simply never reflected in it).
- **§4's mid-route-cusp-freeze and inversion-tolerance-widening notes** said "not yet fixed" -
  updated to point at the actual fix (§2a) rather than leaving stale open-problem framing.

Added a new **§2a "Path smoothing"** section for the entire `cusp_straightener` smoother subsystem
(didn't exist when this doc was last touched): `straight_distance`, `extend_distance` (including
today's curve-continuing-extension bug and fix), `min_lead_distance`. Also added: `IsPathValidDebounced`
tried-and-reverted note (§2), `PathDeviationCritic` tried-and-reverted note (§2 critics row),
`GridBased.tolerance`/`max_iterations` row with the false-success lesson (§4). Added a "last
comprehensively checked" marker at the top of the doc so future staleness is easier to catch at a
glance instead of silently accumulating for another month+.

## 2026-08-29 - Fixed the real bug: extension curvature was far too tight, robot deviated off-path and got stuck

User caught the actual problem after live testing: the curve-continuing extension's curvature was
"too large... not in line with the incoming curve" - the vehicle deviated significantly from the
planned path right at the K-turn vertex and got stuck, since it ended up outside what MPPI/the
costmap considered a valid position relative to the plan. Root cause: the earlier same-day fix
(raising the `ds` guard to 0.02m) was NOT sufficient - the underlying problem wasn't a
near-duplicate point, it was that a **two-point (single-segment) curvature estimate is inherently
dominated by the planner's coarse angular quantization** (`angle_quantization_bins: 72`, 5deg
steps). Even one quantization step over a perfectly normal segment length implies an
unrealistically tight radius - e.g. 5deg over a 0.05m segment implies a 0.57m radius, tighter than
the vehicle's own configured 1.0m minimum turning radius. So the extension was routinely curving
far more sharply than the incoming path actually did, exactly matching the reported symptom.

**Real fix**: estimate curvature over a **~0.15m lookback window** of the actual incoming path
(averaging the yaw change across several planner segments, stopping at the previous cusp or path
start if closer) instead of the single adjacent segment. The quantization noise is bounded to
~5deg total regardless of window length, so averaging over more real path distance makes that
error proportionally much smaller relative to the curve's true accumulated turning - this is what
makes the extension continue the SAME curve rather than grafting on a different, tighter one.
Below a 0.05m window (not enough path available, e.g. right after path start or the previous
cusp), falls back to straight rather than trusting an unreliable short estimate.

**Verified in sim** on the K-turn reproduction goal: 0 `Failed to make progress` events (the
earlier buggy-curvature run had 2 - this was the deviate-off-path/stuck symptom), goal succeeded
in ~37s (vs ~99s with the buggy curvature). Also directly observed the `min_lead_distance` skip
firing for the first time this session: `cusp at idx=1: REVERSE-entering but only 0.071m ahead
(< 0.300m) - robot already at/past it, skipped` - confirming both of today's smoother fixes are
now working together correctly. Not yet a multi-trial timing comparison, not yet hardware-tested.

## 2026-08-29 - Code review of today's `cusp_straightener` changes: two robustness fixes

User asked to revisit the day's smoother changes (curve-continuing `extend_distance` +
`min_lead_distance`) for outliers. Found and fixed two real edge cases in
`cusp_straightener_smoother.cpp`:

1. **Curvature estimate could blow up for closely-spaced points.** The incoming-curvature
   estimate (`kappa = dyaw/ds`) only guarded against literal division-by-zero (`ds > 1e-6`), not
   against noise amplification - with `angle_quantization_bins: 72` (up to ~2.5deg of quantization
   jitter between "straight" poses), a small-but-nonzero `ds` combined with ordinary yaw noise
   could produce a wildly exaggerated curvature, making the extension arc spiral instead of gently
   continuing the incoming curve. Raised the guard to a physically meaningful minimum baseline
   distance (0.02m) - below that, falls back to straight (`kappa=0`) rather than trusting a
   noise-dominated two-point estimate.

2. **`extend_distance <= 0` produced a duplicate point exactly at the cusp.** With
   `extend_distance_` at or below zero (misconfiguration, or deliberately disabled), the old code
   still ran one sub-step of zero arc length, pushing a point identical to the cusp itself right
   next to it - a zero-length segment that could confuse downstream consumers expecting distinct
   points. Not triggered by the current default (0.15m), but a real landmine if ever changed.
   Extension sub-loop now skipped entirely when `extend_distance_ <= 0`.

Also noted, not changed (confirmed intentional, matches the `min_lead_distance` fix's own
rationale): a cusp skipped for being too close to the robot (`lead_distance < min_lead_distance`)
now gets NO treatment at all, including the plain `straight_distance` blend that previously always
ran regardless of proximity - this is a deliberate consequence of the fix (an already-reached cusp
shouldn't be reshaped at all, not just not-extended), not an oversight.

Rebuilt clean. No live nav2 instance was running at review time, so not re-tested live this pass -
the next launch will use the rebuilt `libqcar_smoothers.so` automatically.

## 2026-08-29 - `cusp_straightener` no longer re-extends a K-turn cusp the robot has already reached

User's report: the corner-extension (`extend_distance`, curve-continuing as of earlier today) was
visibly "getting added wherever there's a break in the curve" - not at spurious non-K-turn cusps
(the forward/reverse classification was already confirmed correct via `forward-entering, skipped`
log lines), but repeatedly on the SAME real K-turn cusp, once per replan, all the way until the
robot had essentially already reached it. Confirmed directly in a fresh log
(`nav2_curveext_test_1787995046.log`): one real K-turn's cusp got the extend+straighten treatment
re-applied on 8 separate replans as the robot approached it, cusp index shrinking from idx=94 down
to idx=1 in the replanned path (path length 119 -> 30 points) - by idx=1 the robot is essentially
already at the cusp, already mid-maneuver, and the smoother was still reshaping the path right
under it each time instead of a one-time approach adjustment.

**Fix**: added `min_lead_distance` (default 0.3m) to `CuspStraightenerSmoother`. Computed the arc
length from the replanned path's start (`original[0]`, the robot's live pose) to each cusp; only
apply the extend+straighten treatment when the cusp is still at least `min_lead_distance` ahead -
otherwise skip it (new log line: `REVERSE-entering but only X.XXXm ahead (< 0.300m) - robot
already at/past it, skipped`), leaving that portion of the path untouched. Declared as a normal
smoother parameter (`nav2_params.yaml`, `cusp_straightener.min_lead_distance: 0.3`).

Rebuilt, sim-tested on the K-turn reproduction goal twice: both succeeded, no regression. The
specific `skipped` log line wasn't captured in either trial - the first completed before any
replan's cusp got that close (nondeterministic sim/replan timing vs. the original bad run), the
second started from a position already past the maneuver (reused robot pose from the first
goal's end). The arc-length logic itself is straightforward and the parameter is confirmed loaded
correctly (`CuspStraightenerSmoother instantiated with ... min_lead_distance 0.300`) - correct by
code review, but the skip branch specifically hasn't been directly observed firing yet. Live
testing (the user's own session, left running with this fix loaded) is the next real check.

## 2026-08-29 - Reverted `GridBased.tolerance` back to 0.5: widening it caused silent false successes

The `tolerance: 0.5 -> 1.0` change (previous entry below) fixed the near-wall planning failure it
targeted, but the user reported the real cost in live testing: goals now sometimes reported
`Goal succeeded` while the robot ended up noticeably far from the requested goal and in the wrong
orientation. Root cause is inherent to the mechanism itself, not a bug: when `tolerance` lets the
planner substitute the closest feasible node for an unreachable exact goal, that substituted pose
- not the original requested goal - becomes the plan's actual endpoint. `FollowPath`'s goal
checker only ever compares against the plan it was given (`path.poses.back()`), and nothing
downstream in the stock `NavigateToPose` behavior tree rechecks the final pose against the
original requested goal. So widening `tolerance` doesn't just rescue tight-but-real goals - it
silently accepts "close enough, possibly wrong-facing" as success for ANY goal that's marginally
infeasible, up to the full tolerance radius away, with no indication in the result that a
substitution happened. For a demo/report, a silent wrong-pose "success" is worse than an honest
failure - failures are visible and diagnosable, false successes actively misreport what happened.

**Reverted**: `nav2_params.yaml`, `planner_server.GridBased.tolerance: 1.0 -> 0.5` (back to the
originally-tuned value). The near-wall goal this was fixing (`0.947435, 4.57429`, ~0.15-0.20m
from a wall) will fail outright again rather than silently succeed at the wrong pose - the correct
fix for that specific goal, if it's a real target, is to ensure genuine clearance there (see the
"Diagnosed a planner failure" entries the same day), not to relax the acceptance criterion
globally. No live nav2 instance was running at the time of this revert, so only the config file
needed updating.

## 2026-08-29 - Fixed near-wall goal planning failures: `GridBased.tolerance` 0.5 -> 1.0

Two goals sent minutes apart to nearly the same map location (~0.15-0.20m from a wall, tighter
than the vehicle's 0.22m footprint half-length) both failed identically: `SmacPlannerHybrid`
logged `failed to create plan, exceeded maximum iterations` on all 6 `NavigateRecovery` retries,
ending in `Goal failed` (see the two "Diagnosed a planner failure" investigations this same day).
Since both failures were real, intended targets (not misclicks), investigated whether nav2's
`tolerance` parameter (already 0.5m in `GridBased`) could be made to rescue near-goal-infeasible
cases like this instead of failing outright.

Confirmed against the actual `nav2_smac_planner` search loop (`a_star.cpp`, humble branch,
GitHub): the search loop is `while (iterations < max_iterations && !queue.empty())`; on exit
(either condition), it checks `if (_best_heuristic_node.first < getToleranceHeuristic())` and
backtracks to the closest node it ever reached within `tolerance`, rather than failing outright -
so `tolerance` genuinely is meant to rescue exactly this situation (goal itself infeasible, but a
nearby feasible pose exists), even on iteration-budget exhaustion, not just a clean queue-empty
exit. In practice `tolerance: 0.5` wasn't enough for this particular tight corner.

**Tested live** (`ros2 param set /planner_server GridBased.tolerance 1.0` on the already-running
node, no restart needed): re-sent the exact goal that had just failed (`0.947435, 4.57429`,
orientation `(0,0,-0.999905,0.0137576)`) - this time `planner_server` found a plan immediately
(no more "exceeded maximum iterations"), `smoother_server` applied the curve-continuing
`extend_distance` correctly (1 reverse cusp), `FollowPath` aborted/retried twice on "Failed to
make progress" (separate, already-characterized controller behavior near tight goals, not a
planning failure) then succeeded: `Reached the goal! Goal succeeded`.

**Changed**: `nav2_params.yaml`, `planner_server.GridBased.tolerance: 0.5 -> 1.0`. Sim-validated
on this one case only - not yet a multi-trial comparison, not yet hardware-tested. Widening
tolerance to 1.0m means the planner will now also settle for a pose up to 1m from any requested
goal when the exact goal is infeasible, which is a real behavior change worth knowing about: for
goals that *are* freely reachable, this has no effect (the planner still reaches the exact pose
when it can); it only changes behavior for goals that were failing outright before.

## 2026-08-29 - `cusp_straightener`'s `extend_distance` now continues the incoming curve instead of cutting straight

Changed the corner-push mechanism (`extend_distance`, added 2026-08-26) in
`src/smoothers/cusp_straightener_smoother.cpp`. Previously it pushed the cusp point
`extend_distance` further along a straight line at the cusp's own heading before the reverse
blend began - a hard "cut straight right at the corner" even when the vehicle was still curving
on approach. Now it estimates the incoming leg's curvature at the cusp (from the yaw change over
its last segment: `kappa = dyaw/ds`) and extends along a constant-curvature arc with that same
curvature instead, generated as several sub-poses (0.03m steps) so the curve is actually visible
to downstream consumers rather than implied by one endpoint. Degrades to the old straight-line
behavior automatically when the incoming leg was already straight (`kappa ~ 0`). The subsequent
reverse-leg straight-run blend (`straight_distance_`, unchanged in concept) is now anchored to the
extension's own end heading (`ext_yaw = cusp_yaw + kappa * extend_distance`) rather than the
original cusp heading, since that's the direction the vehicle is actually facing once it reaches
the end of the extension - required for the reverse leg to still point the right way, not new
behavior on its own.

Rebuilt (`colcon build --packages-select qcar_updated`), then smoke-tested in sim on the standard
K-turn reproduction goal (`4.28, 6.35, -1.5708`): goal succeeded (~99s, comparable to prior trial
ranges), the new curvature-extension code path ran on 8 reverse-cusp events across the run (several
replans) with no NaN/crash - just the usual transient `Failed to make progress` controller retries
already characterized as normal. Straight_distance/extend_distance values (0.21/0.15) left
unchanged; only the shape of the extension changed. Not yet re-validated on hardware.

## 2026-08-29 - Diagnosed a planner failure: goal placed too close to a wall exhausts the iteration budget

Sim run after the config revert (see entries below): a goal sent via RViz at map coords
(0.89, 4.61) failed after cycling through all 6 `NavigateRecovery` retries, each attempt logging
`GridBased: failed to create plan, exceeded maximum iterations` from `planner_server`, interleaved
with the `RoundRobin` recovery actions (clear costmaps, wait, backup), ending in
`bt_navigator: Goal failed`.

Root cause, confirmed by sampling `qcar_map_sim.pgm` directly (resolution 0.05m/px,
origin [-7.17, -8.70]): the goal pose sits only **~0.20m** from the nearest occupied (wall) cell.
The robot's footprint (`nav2_params.yaml`, `local_costmap`/`global_costmap.footprint`) is a
0.44m x 0.18m rectangle (`[[0.22, 0.09], [0.22, -0.09], [-0.22, -0.09], [-0.22, 0.09]]`), so no
collision-free full-footprint pose exists at or near that goal - `SmacPlannerHybrid` (Hybrid-A*,
`GridBased`) burns through its entire iteration budget (`max_iterations: 1000000`) searching for
one and never finds it, on every one of the 6 retries, before the whole `NavigateToPose` goal is
aborted.

This is a **goal-placement issue, not a code/config regression** - `planner_server`, footprint, and
inflation config are untouched by anything reverted or restored this session. Practical takeaway
for future goal selection (sim or hardware): keep RViz 2D Goal Pose clicks at least ~0.3-0.4m
clear of any wall/obstacle, given this vehicle's footprint and the planner's collision-checking
against it. Same underlying mechanism (iteration-budget exhaustion, not time-budget - see the
planner search-time investigation referenced in the qcar_updated_mppi_cusp_freeze_investigation
memory) as previously characterized for hard/long goals; this is the first time it was traced to
insufficient wall clearance specifically.

## 2026-08-29 - Nav2 sim smoke test after the `simple_smoother` restore: clean bring-up, no rebuild needed

Verified the working tree launches correctly after the config revert above. `nav2_params.yaml` is
symlink-installed (`install/qcar_updated/share/.../nav2_params.yaml -> src/.../nav2_params.yaml`),
so the `simple_smoother` restoration took effect immediately with no rebuild. Confirmed all
compiled libraries (`libqcar_smoothers.so`, `libqcar_bt_nodes.so`, `libqcar_critics.so`) were
already built after their newest source/header edits from 2026-08-26 - nothing stale, nothing to
rebuild. Sim launch (`qcar_nav2.launch.py launch_sim:=true`) came up clean: `bt_navigator`
connected with bond, both `lifecycle_manager_localization`/`_navigation` reported managed nodes
active, `smoother_server` configured `cusp_straightener` (straight_distance 0.210,
extend_distance 0.150) with the restored `simple_smoother` block present but correctly unused (not
in `smoother_plugins`). Only log warning was a harmless RViz GLSL shader link message. Stack was
torn down cleanly afterward (`SIGTERM` then `SIGKILL` for the one process, `nav2_container`, that
didn't exit on `SIGTERM` alone).

## 2026-08-29 - Partial revert of the 2026-08-26 cleanup pass: dead `simple_smoother` config restored

User asked to revert "the cleanup pass" since they suspected it as a possible cause of the
persistent-invalid-path hardware failure investigated on 2026-08-26. Checked: the cleanup
pass (previous entry below) only touched comments and removed the unused `simple_smoother` block
- no active behavior changed, and this was already confirmed via a post-cleanup sim trial at the
time. The actual disaster-run investigation points at `qcar_clock_offset`/`IsPathValidDebounced`
(already reverted separately - see the "IsPathValidDebounced + max_cusps" entry below), not this
pass. No git commit existed from before the cleanup pass to restore exact prior comment text from
(none of today's/yesterday's nav2 work has been committed yet), so a byte-for-byte comment revert
wasn't possible without fabricating prose. Landed on a partial, honest revert instead: restored
the dead `simple_smoother` block in `nav2_params.yaml`'s `smoother_server` (still unused - no
`<SmoothPath>` call for it in the BT - but matches the project's own "don't proactively remove
dead config" rule, which the cleanup pass arguably violated). Condensed comments left as they are
- they're accurate for the current state and rewriting them from memory risked introducing new
inaccuracies for no functional benefit. Current live config (BT, critics, smoother selection) is
otherwise unchanged and should be functionally identical to the state before the cleanup pass.

## 2026-08-26 - Multiple real-hardware maps scanned; stale variants left in maps/, needs manual cleanup

Re-scanned the hardware map several times today (fresh SLAM sessions via `qcar_slam.launch.py`,
each `map_saver_cli`'d to a new file, then promoted to the active `qcar_map_hardware.yaml` by
copying in and backing up the previous one). Active map as of end of day: originally
`qcar_map_hardware_latest`/`_v2`'s content, promoted last. **`maps/` now has several stale
variants sitting alongside it that were never cleaned up**: `qcar_map_hardware_new.{pgm,yaml}`,
`_latest.{pgm,yaml}`, `_v2.{pgm,yaml}`, `_prev.{pgm,yaml}`, `_prev2.{pgm,yaml}`,
`_prev3.{pgm,yaml}` - only `qcar_map_hardware.yaml` itself (and `qcar_map_sim.yaml`, untouched)
are actually referenced by any launch file. Left in place deliberately (not proactively deleted -
see the qcar_updated_dont_proactively_cleanup_dead_config memory) since one of the `_prevN`
backups may still be wanted as a fallback, but this is real clutter worth a manual pass to decide
what to keep before it gets confusing.

## 2026-08-26 - Code cleanup pass: dead config removed, verbose historical comments condensed

Requested cleanup of the whole stack once the cusp-freeze fix (smoother + extend_distance) was
locked in. Two categories of change, comment-only/dead-code-only - no behavior change, confirmed
via a post-cleanup sim rebuild + trial matching pre-cleanup numbers (40.1s/0 recoveries on the
K-turn reproduction goal, vs. the already-validated 38.3-48.1s range).

**Removed** (genuinely dead, self-documented as unused): the `simple_smoother` block in
`smoother_server` (`nav2_params.yaml`) - never wired into any BT's `<SmoothPath>` call, and its
own comment already said so.

**Condensed** (trimmed multi-experiment trial-and-error narratives that are now also captured in
this CHANGELOG/the project memory, kept the load-bearing "why" + final value + a pointer to full
history): `nav2_params.yaml` shrank 750→633 lines - `yaw_goal_tolerance`, the `FollowPath`/MPPI
intro comment, `batch_size`, `vx_std`/`wz_std`, the `PreferForwardCritic`-removal rationale,
`EarlyCommitCritic.active_path_points`, `GoalAngleCritic.cost_weight` (31→14 lines, the worst
offender), `PathAlignCritic.cost_weight`, the `smoother_server`/`cusp_straightener` block, and
`minimum_turning_radius`. Also condensed: `navigate_to_pose_w_replanning_and_recovery.xml`'s
header comment (69→~20 lines, kept the conclusion of the SingleTrigger/Fallback debugging saga,
cut the blow-by-blow), `cusp_straightener_smoother.hpp`'s doc comment (54→~17 lines), one
redundant inline comment in `cusp_straightener_smoother.cpp`, and `early_commit_critic.hpp`
(40→~24 lines, an older critic from a prior session, also in scope).

Left alone: `qcar_bridge.py`'s steering/throttle calibration comments (real measured data tables
needed to reproduce the calibration, not narrative bloat) and all scripts in `scripts/`/
`qcar_onboard/` (no dead code or debug leftovers found).

## 2026-08-26 - IsPathValidDebounced + max_cusps: reverted ahead of video capture

Both `IsPathValidDebounced` (see full entry below) and the `cusp_straightener.max_cusps` cap
(see the further-below entry) were reverted the same day they were added, after a real-hardware
run on a newly-scanned map produced a disaster path: the robot never executed its reverse
segment at all.

**What actually happened** (log-verified): the local costmap reported the current path
persistently invalid - not a single-tick blip, but confirmed invalid on 2 consecutive checks,
every single RateController cycle (~1.5-2.6s), for the entire run. Each confirmed-invalid
triggered `IsPathValidDebounced` to fail correctly (exactly as designed) and force a replan; each
replan aborted the in-progress `FollowPath` action and handed the controller a brand-new path.
The robot was never given an uninterrupted window to execute the reverse segment - the same
"interrupting a maneuver, however triggered, makes it worse" failure mode from the original
2026-08-25 diagnosis, just triggered by a new mechanism this time. `max_cusps` was not the actual
cause in this run (every replan came back at 2 cusps, well under the cap) but was reverted
alongside it to get back to a single, fully-known-good state rather than debug two things at once
under time pressure.

**Most likely root cause, not fully confirmed**: `qcar_clock_offset` was still at its default
0.0 (unmeasured) on the relay for this session - confirmed via the relay's own startup warning.
A quick re-measurement after a battery swap earlier in the session found a real, non-trivial
offset (~0.15s after discarding a noisier first SSH sample). If `/scan`/`/odom` timestamps are
off by that much, the local costmap can genuinely misplace obstacles relative to the robot's true
current pose, which would explain a REAL (not falsely-triggered) persistent invalidity on an
otherwise-fine path. Applied `qcar_clock_offset:=0.152` to the relay as a fix and had one goal in
progress with it when the decision was made to revert everything instead of keep debugging live,
so this fix is not yet confirmed to resolve the issue.

**Reverted**: `IsPathValidDebounced` back to stock `IsPathValid` in
`navigate_to_pose_w_replanning_and_recovery.xml`; `qcar_bt_nodes` removed from
`bt_navigator.plugin_lib_names`; `max_cusps` rejection logic removed from
`cusp_straightener_smoother.cpp`/`.hpp` and its param removed from `nav2_params.yaml`. All source
files kept (`src/bt/is_path_valid_debounced_condition.cpp`), just deselected. Active config is
back to exactly what was validated earlier today in sim (mean 43.9s/0 recoveries) and on hardware
(2/2 goals succeeded). The `qcar_clock_offset:=0.152` fix on the relay was left in place (it's a
launch parameter, not a code change, and is very likely a genuine improvement regardless of
today's BT-node revert) - re-measure at the start of any future session, since it drifts.

## 2026-08-26 - IsPathValidDebounced: fixes false-positive replans, locked in

User's diagnosis, prompted by the `is_path_valid` service mechanism explanation earlier today: the
stock `IsPathValid` BT condition (`nav2_behavior_tree`) declares the whole path invalid - and
triggers a full replan - the instant a SINGLE tick's check against the live local costmap fails,
with no debounce. A single frame of sensor noise, a transient TF jitter, or a momentary
self-detection artifact can flip one path pose "invalid" for exactly one tick even though the
path is still genuinely fine - forcing an unnecessary replan, and per today's earlier
investigation, an unnecessary replan from a mid-maneuver pose is exactly what produces the
messiest, highest-cusp-count plans.

**Implemented** `IsPathValidDebouncedCondition` (`src/bt/is_path_valid_debounced_condition.cpp`),
a new custom BT condition node - loaded via BT.CPP's own plugin mechanism
(`BT_REGISTER_NODES`/`bt_navigator.plugin_lib_names`, distinct from the pluginlib mechanism used
by the smoother/critic plugins). Otherwise an exact copy of stock `IsPathValidCondition`'s service
call (verified against `is_path_valid_condition.cpp` on GitHub, humble branch - same
`/is_path_valid` service, same request/response types), but only reports the path invalid after
`consecutive_failures_required` (2) ticks in a row report invalid; a single valid response resets
the counter immediately. Since the check is already rate-limited to 1Hz by the existing
`RateController`, this means a real, persistent obstacle is still caught within ~2s, while a
single-frame blip is fully absorbed.

**Build gotcha hit and fixed**: the plugin loaded but `bt_navigator` failed with `can't find
symbol [BT_RegisterNodesFromPlugin]` - `BT_REGISTER_NODES`'s `BTCPP_EXPORT` macro expands to
nothing unless the target has `BT_PLUGIN_EXPORT` defined
(`target_compile_definitions(qcar_bt_nodes PRIVATE BT_PLUGIN_EXPORT)`), so the registration
entry point wasn't actually exported from the `.so`. Fixed, confirmed via `nm -D` showing the
symbol as exported (`T`) before retesting.

**3 trials on the K-turn reproduction goal**: 38.2s, 38.3s, 40.4s - mean 39.0s, **0 recoveries in
every trial**, confirmed actively absorbing 2-3 false-positive blips per trial (logged directly:
`IsPathValidDebounced: reported invalid (N/2 consecutive) - riding it out`). Beats the
already-locked-in baseline (45.2s, 38.3s, 48.1s, mean 43.9s) on both mean and variance. **Locked
in**: `IsPathValidDebounced` replaces `IsPathValid` in
`navigate_to_pose_w_replanning_and_recovery.xml`, `qcar_bt_nodes` added to
`bt_navigator.plugin_lib_names`.

**Logging gap found and fixed** (same day, real-hardware trial): the node only logged the
absorbed ("riding it out") case, not the confirmed-invalid case that actually triggers a replan -
made a real replan sequence look like it "just happened" with no visible cause when tracing a
hardware run. Added a log line for the confirmed-invalid/replanning-now branch too.

Also added `cusp_straightener.max_cusps` (default 4): real hardware produced a 16-cusp plan for a
goal that should've been simple - `smooth()` now rejects any plan over this count outright
(returns false), forcing a retry via the existing RecoveryNode machinery instead of trying to
straighten an excessively complex plan. A strict `max_cusps=2` test confirmed the rejection path
fires correctly (observed a real 2-cusp plan get invalidated by IsPathValidDebounced, replan to
4 cusps, rejected, retry to 3 cusps, rejected, retry to 2 cusps, accepted) but caused more
replan/reject churn than worth it for a video-capture run - reverted to 4 for actual use.

## 2026-08-26 - PathDeviationCritic: implemented the deferred deviation-correction idea, reverted

Implemented the deviation-based path-seeking idea deferred earlier today (see
`qcar_updated_mppi_cusp_freeze_investigation` memory's "deferred idea" section): a new
`PathDeviationCritic` (`src/critics/path_deviation_critic.cpp`, same pattern as
`EarlyCommitCritic`) that, once the robot is more than `deviation_threshold` from the path,
pulls sampled trajectories toward a point `lookahead_offset` points further along the path
instead of the nearest one - "keep making progress" rather than "insist on an exact rejoin".
Zero-cost and inactive whenever the robot is within `deviation_threshold` of the path, so it
should never affect on-track behavior.

**3 trials on the established K-turn reproduction goal (`4.28, 6.35, -1.5708`), clean relaunch
each**: 112.8s, 71.7s, 38.4s - mean 74.3s, 0 recoveries in any trial. Compared against the
already-locked-in config's own validated numbers (45.2s, 38.3s, 48.1s, mean 43.9s): worse mean
AND much wider variance (38-113s vs 38-48s). Never caused an outright recovery, but the extra
critic appears to be fighting `PathAlignCritic` rather than helping - net effect is slower, less
consistent runs, not faster/safer ones.

**Reverted before moving to video capture**: removed `PathDeviationCritic` from
`controller_server.FollowPath.critics` in `nav2_params.yaml` (code and param block both kept,
deselected - same pattern as the smoother's reverted v1). Active config is back to exactly what
was validated earlier today (`cusp_straightener` with `extend_distance`, no `PathDeviationCritic`).
Do not re-enable without a different design (e.g. a smaller weight, a different lookahead
mechanism, or reducing `PathAlignCritic`'s own weight conditionally instead of adding a competing
critic) and a fresh multi-trial comparison.

## 2026-08-26 - Real-hardware validation of the fixed+extended cusp_straightener smoother

Ran nav2 on real hardware (map `qcar_map_hardware.yaml`) with the locked-in `cusp_straightener`
config (`straight_distance: 0.21`, `extend_distance: 0.15`) active, output captured to a log file
this time so the smoother's `RCLCPP_INFO` diagnostics (cusp count/classification per `smooth()`
call) were available for real analysis instead of CSV forensics. User sent two goals manually via
rviz while nav2 ran clean (no gazebo/rviz/stray processes beforehand).

**Goal 1** `(0,0) -> (5.15,-0.64)`: 50.1s, **succeeded, 0 recoveries**. 7 `smooth()` calls; cusp
counts across them: 1, 1, 1, 1, 8, 8, 0 (final approach was cusp-free).
**Goal 2** `(4.87,-0.57) -> (-1.92,-0.14)`: 96.8s, **succeeded, 0 recoveries**. 8 `smooth()` calls;
cusp counts: 5, 5, 5, 5, 9, 10, 5, 3 (settled to 3 for the final ~60s of uninterrupted execution).

Both results are a good sign for the fix holding up on hardware - no recoveries needed even
though several replans produced far more complex paths (up to 10 cusps) than anything validated
in the sim trials above (max ~3 cusps seen there).

**Investigated whether the replan pattern itself was a problem, corrected an initial
misdiagnosis**: the first few `smooth()` calls in Goal 1 landed close together (944.071, 944.110,
945.150, 946.200 - epoch seconds), which initially looked like an unthrottled/noisy replan burst.
Re-reading `navigate_to_pose_w_replanning_and_recovery.xml` showed this is not a bug: `IsPathValid`
is already gated by a `RateController hz="1.0"` (deliberately tuned back on 2026-07-19, full
history in that file's own header comment). The timestamps are fully explained by that design:
`944.071` is the unconditional `InitialComputePathToPose` (runs once, outside the
RateController); `944.110` (+0.04s) is the RateController's own first tick, which BT.CPP fires
immediately before settling into its steady interval; `945.150`/`946.200` are then exactly 1s
apart, textbook steady-state behavior. The ~15.6s gap before the next real replan (961.800) means
`IsPathValid` kept passing for ~15 consecutive 1Hz checks with nothing to do. **No change made** -
an initial plan to add a second `RateController` around `IsPathValid` was correctly identified as
redundant before implementing it, since one already exists and is already tuned/documented there.

**Banked for future investigation, not acted on today**: why real-hardware replans occasionally
produce much higher cusp counts (8-10) than anything seen in the sim reproduction (max ~3). No
validated root cause - most likely `SmacPlannerHybrid` responding to genuinely more awkward
real goal/environment geometry, but confirming that (and any fix, e.g. retuning
`reverse_penalty`/`change_penalty`) needs the same relaunch-clean, multi-trial validation process
used throughout this investigation, not a same-day change. Revisit with a full session's time
budget, alongside the deferred goal-direction-seeking idea below.

## 2026-08-26 - Cusp-freeze reproducibility trials + a real bug found in the v2 smoother

Ran 3 relaunch-fresh trials each of baseline (no smoother) and v2 `cusp_straightener`, same
corridor goal (`4.28, 6.35, -1.5708`), to check reproducibility and get a fair comparison.

**Baseline (no smoother)**: 81.3s/1 recovery, 81.1s/1 recovery, 168.1s/4 recoveries. High
run-to-run variance confirmed - same config/goal/map, ~2x time and 4x recoveries between the best
and worst trial. Consistent with the MPPI-stochastic-freeze diagnosis, not a deterministic
planning defect. The previously-reported left-steering spike during the freeze was NOT
consistently present: 2/3 trials showed no anomaly at all (only a legitimate final-orientation
correction at the very end), 1/3 showed brief (single-sample, ~50ms) left flickers during the
freeze - much smaller than the ~0.5s sustained spike seen earlier with v2 active.

**v2 smoother**: 96.6s/2 recoveries, 101.4s/2 recoveries, then a severe outlier -
**432.8s/12 recoveries**, ~75% of the run stagnant, including a failed `BackUp` recovery action.
v2's mean (~210s) and worst case (433s) are both roughly 2x and 2.5x worse than baseline's,
respectively - this run of trials does NOT support v2 as an improvement.

**Bug found** (`cusp_straightener_smoother.cpp`): the smoother detected cusps once at the top of
`smooth()`, then processed each cusp **in place** on the same `path.poses` array it was mutating.
If two cusps end up close together - plausible on a path replanned from a robot pose mid-maneuver
(e.g. reverse briefly, then immediately forward again), which is exactly what happens during a
recovery cycle - an earlier cusp's blend (which rewrites poses up to `straight_distance_` past it)
could corrupt the position/orientation data a later, nearby cusp reads to classify itself
(forward- vs reverse-entering) and compute its own straight-line target. A plausible direct cause
of the 432.8s outlier, which had 12 replans (12 fresh `smooth()` calls, each on a path shaped by
wherever the robot happened to be mid-recovery) - far more opportunity for a multi-cusp,
close-together path than the clean single-cusp initial plan. **Fixed**: `smooth()` now snapshots
`path.poses` into a read-only `original` vector before any modification: every geometric decision
(cusp detection, forward/reverse classification, and the blend's interpolation targets) reads
from `original`, only ever writing results into the live `path`. Also added `RCLCPP_INFO` logging
of cusp count and each cusp's classification, so this is directly visible in the nav2 log on
future runs instead of needing separate CSV forensics.

Also fixed: `log_goal.py` (scratch, not committed) previously logged only the *first* `/plan`
message, so a trial with multiple replans (like the 432.8s outlier) left every subsequent
replanned path unrecorded - no way to check what shape those paths actually had. Now logs every
`/plan` message it receives, each to its own sequence-numbered CSV file.

**Re-tested with the fix in place, 3 trials**: 59.3s/0 recoveries, 97.1s/2, 103.7s/2 - mean 86.7s
/ 1.3 recoveries, worst case 103.7s. This clearly beats baseline on every axis (mean 110.2s/2
recoveries, worst case 168.1s) and is nowhere near the old buggy v2's 432.8s outlier. Both
trial 2 and trial 3 logged genuine multi-cusp replans (2 cusps each) handled correctly by the
fixed classification.

**User-proposed further improvement: extend the corner.** Sketch: instead of the incoming and
outgoing legs of a K-turn meeting at one precise geometric point (a sharp vertex the path - and
MPPI's cusp-arrival gating - must converge on exactly), extend each leg *past* the natural
corner so they overlap/cross, giving a wider margin around the transition rather than a single
tight point. Implemented as a new `extend_distance` parameter (default 0.15m) on
`CuspStraightenerSmoother`: at each REVERSE-entering cusp, a new point is inserted
`extend_distance` past the original cusp position, continuing straight in the *incoming* leg's
own heading (as if the vehicle kept driving a little further before reversing); the existing
`straight_distance` blend then starts from this new, pushed-out point instead of the original
cusp, so the reverse leg has to travel back past the incoming leg's overshoot to rejoin the
original downstream curve - producing the crossed/overlapping corner from the sketch rather than
a single sharp vertex. Required rewriting `smooth()` to build a new output path (inserting a
point changes the path's length, unlike the previous in-place-only modification) - still reads
only from the `original` read-only snapshot per cusp, so the multi-cusp-corruption fix above is
preserved.

**Tested, 3 trials**: 45.2s/0 recoveries, 38.3s/0, 48.1s/0 - mean 43.9s, **zero recoveries in
every trial**. This roughly halves fixed-v2's own mean (86.7s) and eliminates recovery triggers
entirely across the full set, on top of already beating baseline. **This is now the locked-in
config**: `extend_distance: 0.15` added alongside `straight_distance: 0.21` in
`cusp_straightener`'s params; `smoother_plugins: ["cusp_straightener"]` and both `<SmoothPath>`
BT calls stay active.

## 2026-08-25 - MPPI freeze at path cusps in tight spaces: diagnosed, one fix attempt reverted

User reported nav2 in tight real-hardware spaces producing a "chaotic curve/reverse-curve/curve"
maneuver when a goal required turning around. Reproduced in sim by adding a narrow (~2.4m) dead-end
corridor to the SLAM map (`qcar_map_sim.yaml`, re-mapped) and sending a goal deep inside it with a
reversed final heading. Built `log_goal.py` (scratch, not committed) to log `/plan` and `/odom` to
CSV for a NavigateToPose goal, and a direction-reversal detector to count cusps in each.

**Diagnosis**: the *planner* (`SmacPlannerHybrid`) is fine - the logged `/plan` had exactly one
clean cusp for this corridor goal. The *executed* `/odom` trajectory showed the robot frozen
(creeping only a few cm) for ~34s, then another ~29s, right at the cusp point, before finally
breaking through - not a planning problem, an MPPI execution-side freeze at the cusp transition.
Read `nav2_mppi_controller`'s actual `path_handler.cpp` source (humble branch, fetched from
GitHub): MPPI only sees the plan truncated up to the cusp pose until `isWithinInversionTolerances()`
passes, which requires the robot to be within `inversion_xy_tolerance`/`inversion_yaw_tolerance` of
that pose *simultaneously* - structurally the same "must precisely reach a pose requiring a big
heading change" situation that caused the separate, already-documented final-goal reorientation
freeze (fixed there via `GoalAngleCritic.cost_weight` 3.0->10.0), but with no equivalent fix ever
applied to the cusp-transition case.

**Fix attempted, reverted**: widened `inversion_xy_tolerance`/`inversion_yaw_tolerance` from the
stock 0.2/0.4 to 0.4/0.7, on the theory that loosening the pose-lock would let MPPI advance past
the cusp sooner. Live-tested (same logged corridor goal): **worse**, not better - the single freeze
at the cusp was replaced by 6+ `controller_server: Failed to make progress` / progress_checker
aborts starting well before the robot even reached the cusp, with `/odom` showing it oscillating
over a wider area without net progress. Reverted both values to the stock 0.2/0.4. The underlying
diagnosis may still be right, but this specific fix wasn't - see `nav2_params.yaml`'s
`inversion_xy_tolerance` comment for the full note on what NOT to re-try without independently
testing each value and logging (not just watching) each candidate.

**Second fix attempted, also reverted**: user proposed replanning as soon as the robot is detected
stationary rather than on a fixed timer (distinct from - and not the same failure mode as - the
already-reverted "replan every N seconds unconditionally" from 2026-07-15/07-19, since this is
gated on actually being stuck). Tested the cheap version: lowered
`controller_server.progress_checker.movement_time_allowance` from 30.0 to 8.0s, to make the
existing stuck-detection -> outer-recovery-loop -> fresh-replan chain trigger ~4x sooner. Also
worse: `Failed to make progress` fired every ~8-13s as designed, but cycling through recovery this
much more often meant the round-robin's `BackUp` action itself failed at least once ("backup
failed", plausibly no clear room for a 0.3m backup in this ~2.4m corridor) - something never seen
at the stock 30s setting. Never succeeded within a 110s test window. Reverted to 30.0.

Taken together, both same-day fix attempts - one loosening the cusp-arrival pose lock, one
shortening the stuck-detection window - made the corridor case worse, not better, despite
targeting the freeze from different angles. Circumstantial evidence the freeze needs real,
uninterrupted time to self-resolve (the untouched 30s/stock-tolerance baseline DID eventually
succeed, ~34s+~29s freeze then done) - interrupting or shortcutting it, however triggered, seems
to prevent whatever convergence is actually happening from completing, rather than speeding it up.
Next investigation should look at why interruption itself seems harmful, not just at detecting or
escaping the freeze faster.

**Third fix attempted, also reverted**: user identified a distinct, real mechanism separate from
cusp count or steering calibration - a front-steered vehicle's steered wheels are the
self-correcting LEADING axle driving forward but the jackknife-prone TRAILING axle in reverse
(same reason backing up a car, or pushing a shopping cart backward by its front casters, is
inherently harder to track than pulling/driving forward). User proposed a real driving technique
for this: back up roughly straight first to establish stable directional tracking, then ease into
the curve, rather than starting to curve immediately from a standing start in reverse. Checked
`SmacPlannerHybrid`'s actual `SearchInfo` struct (nav2_smac_planner source) for an existing
"minimum straight run after a cusp" parameter - none exists. Implemented as a new custom
`nav2_core::Smoother` plugin, `CuspStraightenerSmoother`
(`src/smoothers/cusp_straightener_smoother.cpp` + `.hpp`, `qcar_smoothers.xml`, wired via
`smoother_server` + a new `<SmoothPath>` BT call after every `ComputePathToPose`): detects cusps
in the planned path and re-projects the next `straight_distance` (default 0.21m, ~half this
vehicle's length) of arc length after each one onto a straight line continuing the cusp pose's
heading, before letting the original curve resume.

Live-tested (same logged corridor goal): **worse** again - took even longer to resolve than the
untouched baseline, still unresolved at the end of a 108s test window. Directly verified the
plugin's own logic against the real logged cusp (via a standalone script calling the
`smooth_path` action directly) and confirmed it works exactly as designed - the mechanism itself
isn't buggy. Working theory for why it still made things worse: it introduces a new sharp kink
where the straightened segment rejoins the original curve, and the vehicle is still in the same
unstable trailing-axle reverse configuration when it hits that new discontinuity - delaying the
hard transition rather than removing it. Reverted `smoother_server`'s active plugin to
`simple_smoother` (itself still unused/dead config, unchanged from before) and removed the
`<SmoothPath>` BT calls; the `CuspStraightenerSmoother` plugin code is left in place
(`qcar_smoothers.xml`/CMakeLists/package.xml all still build it) in case a future gradual-blend
version is worth trying instead of the abrupt kink this version used.

Three same-day fix attempts now, targeting the freeze from three different angles (looser
cusp-arrival tolerance, faster stuck-detection, straightening the reverse approach) - all three
made the reproduction case worse, none better than the untouched baseline. Strengthens the
existing hypothesis that the freeze needs real, uninterrupted time/space to resolve and that
active intervention - of several different kinds tried so far - tends to interfere with whatever
is actually happening rather than help it.

## 2026-08-25 - Steering GAIN calibration added, not just zero-point trim

`STEERING_TRIM` alone only ever corrected the zero-point offset (wheels not pointing straight at
a 0 command). Real hardware behavior "not turning as well as sim" traced to a second, separate
error: a genuine steering GAIN excess - the real wheel turns further than the kinematic model's
commanded angle implies for any nonzero command, not just at zero. Calibrated empirically (no
factory steering conversion curve exists in the Quanser manuals or HAL source - checked directly)
via a clean 7-point 1m-arc sweep on 2026-08-25: `implied_real_angle = 1.39*cmd + 0.119`, R²=0.977.
An earlier 2026-08-24 attempt at this same investigation produced apparently contradictory
"nonlinear/asymmetric" results - retracted, caused by a nut physically caught under a wheel and a
near-depleted battery (0.3 m/s sitting right at the current static-friction floor), not a real
steering property. See `qcar_steering_gain_nonlinearity_investigation` project memory for the full
history.

`STEERING_TRIM` updated from `-0.08` to `-0.09` (anchored to a direct "dead straight" measurement
at that exact value), then refined to **`-0.087`** via 3 repeated trials each at -0.08/-0.085/-0.09
to average out human placement/measurement noise. New `STEERING_GAIN = 1.39` added; `apply_trim()`
now divides by it before adding the trim, so `car.write()` receives a command that produces the
actually-intended real wheel angle instead of just `kinematic_steering + a flat offset`.
`compute_steering()` and odometry remain untouched (still use the true kinematic angle), per the
existing documented constraint that trim/gain correction must stay downstream of anything that
cares about the real physical angle.

End-to-end validation (kinematic +0.15 rad, 1m arc @ 0.4 m/s): residual error between intended and
measured-real steering angle dropped from ~39% (no correction) to ~5.6% (`STEERING_GAIN=1.39`,
`STEERING_TRIM=-0.087`).

## 2026-08-24 - Nav2 sim goals silently no-op: use_sim_time hardcoded false in qcar_nav2.launch.py

Running `qcar_nav2.launch.py launch_sim:=true` accepted RViz goals and immediately reported
"Goal succeeded" without the robot ever moving. `controller_server` logged
`Transform data too old when converting from map to odom` with a ~700,000s gap between the
data timestamp (wall-clock epoch, e.g. `1787571765s`) and the transform timestamp (sim time,
e.g. `756s`) - the nav2 stack was on wall clock while Gazebo/robot_state_publisher/controllers
were on sim clock (per `qcar_updated.launch.py`'s `use_sim_time: True`), so tf lookups failed
and the controller aborted instantly on every goal.

**Root cause**: `qcar_nav2.launch.py` passed `'use_sim_time': 'False'` unconditionally to
`nav2_bringup`'s `bringup_launch.py`, regardless of the `launch_sim` argument.

**Fix**: pass `launch_sim` itself as the `use_sim_time` launch argument, so it tracks whether
Gazebo is actually running.

Same hardcoded-`use_sim_time: False` bug also existed in both Cartographer nodes in
`qcar_slam.launch.py`, unrelated to `launch_sim` - fixed the same way (tied to `launch_sim`).

## 2026-08-24 - Split hardware/sim maps

`qcar_map.yaml` was built from real-hardware Cartographer SLAM but was being used unconditionally
by `qcar_nav2.launch.py` for both real hardware and Gazebo sim (`myworld.world`) runs, even though
the two environments have different geometry - AMCL localization against the wrong map would be
broken/nonsensical in sim regardless of the `use_sim_time` fix above.

Renamed `maps/qcar_map.yaml`/`.pgm` -> `maps/qcar_map_hardware.yaml`/`.pgm`. `qcar_nav2.launch.py`
now picks `maps/qcar_map_sim.yaml` or `maps/qcar_map_hardware.yaml` based on the `launch_sim`
argument. `qcar_map_sim.yaml` still needs to be generated (via `qcar_slam.launch.py
launch_sim:=true` + teleop in Gazebo, then `map_saver_cli`) before sim nav2 runs will work.

## 2026-08-18 - Stop-latency/stiction-delay root cause found: QCar HAL readMode=1 buffering bug, RESOLVED

**Resolves the 2026-08-17 investigation below.** The ~2.0-2.1s "stiction delay" and ~2.4-2.55s "stop
latency" characterized yesterday were NOT real vehicle physics - they were an artifact of
`QCar(readMode=1, ...)` in `qcar_bridge.py`.

**Root cause, confirmed by reading the actual HAL source
(`/home/nvidia/Documents/python/pal/products/qcar.py` on the QCar)**: `readMode=1` ("task based
I/O") starts a background acquisition task at object construction time, filling a ring buffer of
`frequency*2` samples (100 samples at our 50Hz = exactly 2.0s) with `BufferOverflowMode.OVERWRITE_ON_OVERFLOW`.
`car.read()` dequeues one buffered sample at a time via `task_read()`. Since the `QCar` object is
constructed *before* the listening socket even opens, and `car.read()` isn't called until a relay
connects, the buffer fills and starts cycling during that gap. Once reads begin at the matched 50Hz
rate, they get permanently stuck ~100 samples (~2.0s) behind real-time - the backlog never drains
because consumption matches production rate exactly. This produced a stable, fixed reporting lag
that looked exactly like a physical stiction/coasting delay in every measurement, but wasn't one.

**How this was found**: the user directly, repeatedly observed the real robot start/stop moving the
instant a command was given/removed during teleop - flatly contradicting the ~2s delay measured via
`drive_and_log.py`. Initial hypotheses (duty-magnitude dependence, cold-start-vs-warm-restart) were
tested and disproven with real hardware data. The user's physical argument - "an encoder is a
real-time sensor, it can't have a built-in 2-second lag" - was the correct one, and prompted actually
reading the HAL source rather than continuing to trust the `Odometry` class's own internal
consistency (which is real, but doesn't rule out the *input* to that class being stale).

**Fix**: `qcar_bridge.py` now constructs `QCar(readMode=0, frequency=READ_RATE)` - immediate I/O,
reads current hardware state directly, no background task/buffer at all.

**Validated by re-running the full THROTTLE_GAIN calibration suite** (2m @ 0.3/0.4/0.5 m/s, 0.3 m/s
@ 1m/2m/3m) on a fresh battery, same duty formula (`duty = 0.04 + 0.0667*|speed|`) unchanged:

| Test | Stiction delay (before -> after) | Extra distance (before -> after) | Stop latency (before -> after) |
|---|---|---|---|
| 2m @ 0.3 m/s | 2.09s -> 0.117s | 0.520m -> 0.114m | 2.37s -> 0.598s |
| 2m @ 0.4 m/s | 2.14s -> 0.001s | 0.790m -> 0.114m | 2.40s -> 0.501s |
| 2m @ 0.5 m/s | 2.14s -> 0.116s | 0.923m -> 0.154m | 2.55s -> 0.550s |
| 1m @ 0.3 m/s | 2.14s -> 0.120s | 0.641m -> 0.118m | 2.44s -> 0.572s |
| 3m @ 0.3 m/s | 2.08s -> 0.088s | 0.583m -> 0.104m | 2.35s -> 0.509s |

Stiction delay is now essentially gone (0.001-0.12s). Extra distance is now small and roughly
constant (~0.10-0.15m) regardless of speed or target distance, rather than scaling with speed as
previously found - that scaling relationship was itself an artifact of the lag, not a real vehicle
property. The remaining small residual (~0.5-0.6s stop latency, ~0.1-0.15m extra distance) checks
out physically: `extra_distance / stop_latency` (average coast velocity) comes out to ~54-61% of
cruise velocity in 4 of 5 tests - consistent with an ordinary, roughly linear velocity decay to zero
over that latency window (50% expected for perfectly linear decay). This residual is genuine, small,
and fully explained - not a separate mystery to chase further.

**Also fixed while investigating**: the `[DBG]` print (added 2026-08-17 for exactly this
investigation) was firing at the full ~50Hz control-loop rate, making the terminal impossible to
read live - rate-limited to 10Hz (`DEBUG_PRINT_PERIOD = 0.1`) in `qcar_bridge.py`.

**IMPORTANT - open follow-up, not yet done**: 0.2 m/s remains unreliable (right at the real
static-friction deadband, separate from this bug - sometimes doesn't move at all even after this
fix). More importantly, **the 2026-08-17 Nav2 display-lag investigation's "~0.56s RViz start gap"
finding was measured during the SAME session this buffering bug was active** - that investigation's
conclusion (a "cosmetic pipeline/rendering latency, not a real navigation problem") needs to be
re-verified with the fix in place before being trusted, since the same stale-encoder-data mechanism
could plausibly have contributed to or entirely explained that gap too. See
`qcar_updated_stop_distance_investigation` and `qcar_updated_nav2_display_lag_investigation` project
memory.

## 2026-08-17 - Stop-latency root-cause dig started, interrupted by dead QCar battery

**`qcar_bridge.py` currently has a temporary `[DBG]` print re-added to the
control loop** (logs `target_linear`, `current_throttle`, and real encoder
`v` together, QCar-local clock, no network involved) - NOT the clean state
described in the THROTTLE_GAIN entry below. Left in deliberately to resume
this diagnostic next session; remove once resolved.

Prompted by the settled-distance table (below) showing a consistent
increase in extra distance across speeds while stop latency itself stayed
flat (~2.4-2.55s regardless of commanded speed) - re-examined the raw
coast-phase velocity trace instead of assuming simple momentum/friction
coasting. Found the coast is NOT smooth decay: velocity stays flat/even
slightly elevated for ~2.0s after "stop commanded," then crashes to zero in
~0.3-0.4s. Checked the Quanser HAL manuals for a documented drive-motor
ramp/filter that could explain this - found none (only the steering servo
has a documented time constant, 0.16s, nothing for the drive motor). This
points at a command-delivery lag in our own software/relay pipeline rather
than physical drivetrain coasting - a different conclusion than this
investigation has carried until now.

Was mid-way through capturing QCar-local `current_throttle`-vs-`v` data to
determine whether the lag is in the real motor's hardware response or in
our own software not delivering the stop command promptly, when **the
QCar's battery died**, killing the SSH connection. Also hit and fixed a
real gotcha along the way: piping a long-running process through `tee`
block-buffers Python's stdout instead of line-buffering it, so nothing
reaches the log file until the buffer fills or the process exits - fix is
`python3 -u` (unbuffered) on the QCar-side launch command.

See `qcar_updated_stop_distance_investigation` project memory for the full
writeup and the exact next-session resume steps.

## 2026-08-17 - "LiDAR dots shift with the robot" during Nav2 goals - root-caused, resolution paused

Added `scripts/monitor_goal_timing.py` (wired into `CMakeLists.txt`) to
instrument a single Nav2 goal end to end: exact timestamps + pose for goal
sent, real motion start (from `/odom`, encoder ground truth), RViz-displayed
motion start (from the `map->base` TF - what a human watching RViz actually
sees), goal reached, real stop, and RViz-displayed stop. Two real bugs found
and fixed while building it: (1) the default rclpy subscription QoS
(VOLATILE durability) delivered zero messages against
`/navigate_to_pose/_action/status`'s actual TRANSIENT_LOCAL publisher QoS -
fixed by matching the QoS profile explicitly; (2) detecting "goal sent" via
watching for status code ACCEPTED(1) missed goals that transition straight
to EXECUTING(2) between two status publishes - fixed by tracking goal UUIDs
instead (first time a never-before-seen goal_id appears = goal sent).

**Root cause, measured**: the user's precise report ("real robot starts
immediately, RViz shows it starting late, LiDAR dots visibly shift/catch up,
same thing happens in reverse at the end") is actually TWO separate,
unrelated phenomena:
1. **Start of goal**: RViz's displayed motion lags the real robot by ~0.56s
   (measured: real start +2.145s after goal sent, RViz start +2.705s - a
   0.559s gap). This is a genuine but modest pipeline latency (network
   transit + AMCL scan-matching + RViz's own subscribe/render overhead) -
   NOT a TF-jump bug (map->odom was independently confirmed to update in
   small smooth increments, not sudden discontinuities) and NOT fixable by
   delaying the robot's motion (a constant downstream processing lag doesn't
   shrink by delaying the triggering event - it just shifts both events
   later by the same amount). Nav2's actual controller consumes the same
   `/odom`/`/amcl_pose`/`/tf` topics directly, without RViz's additional
   render overhead, so real navigation decisions are less stale than what's
   visually displayed - this is a cosmetic-only issue.
2. **End of goal**: the real robot kept moving for 2.238s after "goal
   reached" was declared - matching, independently, the ~2.4-2.55s
   stop-latency/coast behavior already characterized in the THROTTLE_GAIN
   calibration work below (open-loop `drive_and_log.py` tests, which never
   go through Nav2's controller at all, showed the same ~2.4-2.5s number).
   This strongly indicates genuine drivetrain momentum/coast, not a Nav2
   controller-tuning gap or display bug.

**Resolution status**: paused here for this session, decision deferred.
Options identified but not yet acted on: (a) tune MPPI's `GoalCritic`/
deceleration-near-goal behavior so commanded velocity is already lower by
the time the robot reaches `xy_goal_tolerance`, which would proportionally
shrink (not eliminate) the coast per the already-established
speed-scales-linearly-with-coast-distance relationship - lower risk than
touching `qcar_bridge.py` directly; (b) accept the coast as a known,
budgeted-for vehicle characteristic (the existing documented stance).
Active braking remains explicitly off the table (see the safety-incident
history below). See `qcar_updated_nav2_display_lag_investigation` project
memory for the full writeup, including the exact timing table from the
live hardware test.

**Also re-confirmed working**: `qcar_clock_offset` (currently `0.17`,
measured fresh this session) must be passed explicitly on every
`qcar_relay_node.py` launch - forgetting it (which happened once this
session, an oversight in a command given to the user) silently defaults to
`0.0` and reintroduces the exact scan/odom timestamp-drift bug this
parameter exists to fix.

## 2026-08-17 - THROTTLE_GAIN calibration finalized and validated across 0.2-0.5 m/s

Final result of the day's calibration work (supersedes the two earlier
same-day attempts logged below). `qcar_bridge.py`'s throttle mapping is now
`duty = THROTTLE_DEADBAND + THROTTLE_GAIN * |speed|` with
`THROTTLE_DEADBAND = 0.04`, `THROTTLE_GAIN = 0.0667`, fitted from an
empirical duty sweep (0.05 -> no motion, 0.06-0.061 -> ~0.30-0.33 m/s,
0.065 -> ~0.39 m/s, 0.085 -> runs away to ~0.85-0.9 m/s - a stick-slip
signature, not a clean linear range) and validated end-to-end with 2m
drive-and-stop tests at 4 commanded speeds:

| commanded | measured cruise | ratio |
|---|---|---|
| 0.2 m/s | unreliable - moved once, didn't move at all on an earlier nearby duty | - |
| 0.3 m/s | 0.31 m/s | 1.05 |
| 0.4 m/s | 0.42 m/s | 1.05 |
| 0.5 m/s | 0.50 m/s | 1.00 |

0.3-0.5 m/s now tracks commanded speed within ~5%, versus the original
`1/3` gain's ~2.6-2.7x overshoot. Below ~0.25-0.3 m/s the real vehicle sits
right at its static-friction deadband and can fail to move at all,
depending on run-to-run friction noise - not something duty calibration
can fix, a hard physical limit of this vehicle at this floor condition.

Also cleaned up as part of finalizing this: removed the leftover `[DBG]`
print statement in `qcar_bridge.py`'s control loop (temporary debug from
2026-08-12, no longer needed), fixed a latent typo bug in
`qcar_teleop_twist.py` (`current_steer_angle_angle` was never actually
read - `current_steer_angle` was the real attribute name used elsewhere in
the file), and raised the teleop default speed from 0.2 to 0.3 m/s since
0.2 m/s is now known to sit at the unreliable deadband edge.

**Still open, unaffected by this fix**: the ~2.4-2.55s stop latency /
coasting behavior after a stop command is commanded is a separate problem
- extra distance still scales with cruise speed (0.60m at 0.2 m/s up to
1.20m at 0.5 m/s). See `qcar_updated_stop_distance_investigation` project
memory for the full sweep/validation data and current status.

## 2026-08-17 - Fixed check_stop_latency.py's cruise-velocity warmup skip

The warmup-skip window for the cruise-phase velocity average was measured
from script start, not from when the car actually started moving. With the
real static-friction deadband found this session (motion can be delayed
~2s after the command is first sent), this silently mixed several seconds
of near-zero pre-motion samples into the "cruise velocity" average -
exactly what made the earlier 0.3m@0.25m/s test report an unreliable 0.097
m/s. Fixed: the warmup timer now starts from the first `/odom` sample where
`|v| > 0.01`, not from node startup.

## 2026-08-17 - THROTTLE_GAIN replaced with a deadband-compensated mapping

Second calibration attempt, addressing why the first (a flat linear gain
correction) broke teleop: added `THROTTLE_DEADBAND = 0.085` (duty floor to
reliably break static friction) alongside a much shallower `THROTTLE_GAIN =
0.021` (duty per m/s above that floor, from the earlier duty=0.10 -> ~0.8
m/s clean measurement). `qcar_bridge.py`'s control loop now computes
`throttle = sign(v) * (THROTTLE_DEADBAND + THROTTLE_GAIN * |v|)` for any
nonzero commanded speed instead of a flat multiplier. Deployed to the QCar;
not yet validated on hardware - this is a two-point fit, expect to refine
after real testing (ideally a longer clean run with `check_stop_latency.py`
once there's more runway than the current ~2-3m).

## 2026-08-17 - check_stop_latency.py no longer runs forever after its report

Found the hard way: the script's original design kept spinning/publishing
`linear.x=0.0` indefinitely after printing its report ("still commanding
zero, safe" was the reasoning). A finished test process was left running in
the background, and its continuous zero-velocity publishes fought a
separately-run `qcar_teleop_twist.py` node's commands on the same
`/cmd_vel` topic - teleop looked like it was "responding but not moving"
(briefly registering a nonzero command, then getting stomped back to zero
within one control cycle), which cost real time to diagnose. Fixed: the
node now sets a `done` flag once its report prints and `main()` exits the
spin loop right after, so the process exits cleanly instead of lingering.

## 2026-08-17 - THROTTLE_GAIN calibration attempted and reverted same day - deadband found

First real calibration attempt: lowered `THROTTLE_GAIN` from `1/3` to
`(1/3)/2.67` based on the earlier-measured ~2.67x commanded-vs-real speed
ratio. Deployed and tested on hardware via `check_stop_latency.py`, which
surfaced a real static-friction deadband: at the old gain, commanded speeds
below ~0.25 m/s (~8% duty) produced zero motion, and even 0.25 m/s sat
motionless for ~2.1s before breaking stiction. Lowering the gain pushes
normal teleop speeds below that same duty threshold, so teleop stopped
responding entirely - reverted `THROTTLE_GAIN` back to `1/3` same day to
restore teleop.

**Conclusion**: the gain and the deadband aren't independent - a linear
gain correction alone can't fix commanded-vs-real speed without also
addressing the static-friction dead zone (e.g. a deadband-compensating
offset added to the throttle command). Next calibration attempt should
tackle both together. See `qcar_updated_stop_distance_investigation`
project memory for full context.

## 2026-08-17 - Added check_stop_latency.py, a combined cruise-speed + stop-latency checker

New node `scripts/check_stop_latency.py` (wired into `CMakeLists.txt`, run via
`ros2 run qcar_updated check_stop_latency.py <target_distance_m> <speed_m_s>
[stop_threshold_m_s] [debounce_s]`). One drive-to-distance-then-stop test now
answers two questions at once instead of needing a separate calibration pass:

1. **Cruise velocity**: mean of the real encoder-derived `/odom` velocity
   during the drive phase (skipping an initial 0.5s warmup for the throttle
   rate-limiter to settle) - directly gives the commanded-vs-measured speed
   ratio needed to correct `THROTTLE_GAIN`, printed as a ready-to-use
   multiplier.
2. **Stop latency**: exact wall-clock time + distance at the moment the stop
   is commanded, vs. the moment `/odom` velocity stays below a small
   threshold for a debounce window (default 0.02 m/s / 0.3s, guards against
   a single noisy near-zero sample mid-coast) - replaces eyeballing "does it
   stop the instant the command prints" with precise numbers.

Every `/odom` sample is also logged to a CSV. Built to support the ongoing
real-hardware stop-distance overshoot investigation (see
`qcar_updated_stop_distance_investigation` project memory).

## 2026-08-02 - Investigated "robot deviates mid-curve, corrects by curve end" on a large-reorientation goal - five parameters tested, none fixed it

User reported a specific symptom on a real goal from their own session (`(-0.0005, -2.014, 180deg)`
from the origin, log: `Setting goal pose: Frame:map, Position(-0.000466228, -2.01385, 0),
Orientation(0, 0, 1, 6.12323e-17) = Angle: 3.14159`): the robot turns more than the curve needs,
deviates from the plan, then corrects back by the time the curve/maneuver completes - not a goal
failure, but a real tracking-accuracy complaint. User did their own hands-on testing across four
parameters first (steering PID `kd`, `PathFollowCritic`/`PathAngleCritic.offset_from_furthest`,
`PathAlignCritic.cost_weight`) and reported no meaningful change each time - this entry reproduces
their exact goal directly and tests a fifth parameter with a repeatable measurement harness, to get
past "no difference" as a subjective impression and into actual numbers.

### Reproduction and measurement method
Isolated headless test (separate `ROS_DOMAIN_ID`/`GAZEBO_MASTER_URI` from the user's own running
session, never interfered with it), the user's exact goal, a Python `/odom` + `/plan` subscriber
capturing every distinct plan snapshot with a timestamp (not just the latest, which was a real bug
in the first pass of this investigation - a mid-maneuver replan was being compared against
early-trajectory samples, producing meaningless numbers). Metric: nearest-point distance from each
`/odom` sample to whichever `/plan` snapshot was active at that moment.

### What's actually happening, measured
- The planned path itself is a single smooth, continuously-curving forward arc (radius of
  curvature ~constant via a 3-point circle fit at every plan point) - not a K-turn, not a kinked
  planner artifact. One small 2-point wiggle right at the very start (residual of the
  known/accepted `change_penalty` case), nothing else.
- Position deviation from the plan rises smoothly to a real, repeatable peak of **0.44m**
  roughly 60-70% of the way through the maneuver (not at the very start), then converges back to
  ~0.11-0.13m as the robot settles near the goal - this is the real mechanism behind "deviates,
  then corrects."
- At peak deviation, the robot was ~0.9-1.0m from the goal position - within a plausible default
  gating range for goal-proximity critics, which is what motivated the `GoalAngleCritic` test
  below.
- A signed heading-vs-local-path-tangent measurement was attempted but abandoned as unreliable
  here: the nearest-plan-point tangent becomes a poor reference exactly when deviation is largest
  (the point genuinely closest to a badly-off-track position isn't necessarily "the point the
  robot should be heading toward"), so no confident over-turn-vs-under-turn directional claim is
  made in this entry - only the deviation magnitude, which is unambiguous.

### Five parameters tested, all live-verified via the same harness, none fixed it
1. **`left/right_steering_pid_gain` kd, 0.1 -> 0.005** (user's own test, physical wheel-tracking
   damping) - no meaningful change. Reverted to `3.0 0 0.1` (see this file's `urdf/qcar_model.xacro`
   history for why that value is load-bearing - lower kp risks reintroducing a real, previously-
   fixed under-steering bug).
2. **`PathFollowCritic`/`PathAngleCritic.offset_from_furthest`, 6/4 and 36/24** (user's own test,
   look-ahead target distance) - no meaningful change at either value.
3. **`PathAlignCritic.cost_weight`, 10.0 -> 7.0** (user's own test, path-adherence strength,
   untested direction - only raising it had been tried before, on 2026-07-19 (3)) - no meaningful
   change.
4. **`GoalAngleCritic.threshold_to_consider`, added explicit `0.5`** (this entry, hypothesis: this
   critic competing with path-following critics too far from the goal): peak deviation `0.44m ->
   0.41m` - within noise, not a real fix. Reverted.
5. **`FollowPath.wz_std`, 0.4 -> 0.3 alone** (this entry, hypothesis: MPPI's own sampling breadth,
   not any single critic's weighting, is the dominant factor - motivated directly by 1-4 all
   failing): peak deviation `0.44m -> 0.39m` - a small, likely-within-noise reduction (~12%,
   comparable to other single-trial differences seen elsewhere in this file), not a clear fix. No
   freeze regression this time (this exact value caused a freeze when first tried on 2026-07-18,
   but `iteration_count` is now `2`, not `1`, which is the fix that made this safe to retest).
   Reverted since the improvement isn't clearly beyond noise on one trial.

### Synthesis - this looks goal-shape-dependent, not a single-parameter bug
A separate, earlier test this session on a milder goal (`(1.5, 1.5, 90deg)`, a moderate curve with
real net translation, nowhere near `minimum_turning_radius`) showed much smaller absolute
deviation than this goal's 0.4m peak. The common factor across the five failed attempts above,
plus that contrast, points at the goal's own geometry - a large heading change (~180deg) over a
short net translation (~2m), close to the vehicle's kinematic limits - as the dominant driver of
deviation magnitude, not any single tunable critic weight or gain. This is consistent with, and
reinforces, the open trade-off already documented in this file under `FollowPath.PathAlignCritic`
and `vx_std`/`wz_std`: path-tracking looseness under the current sampling configuration is a real,
harder-than-single-parameter-tuning-suggests limitation for this specific class of goal.

### Not yet tried (next steps if pursued further)
- Lowering `vx_std` and `wz_std` **together** (not `wz_std` alone, as tried here) - the
  2026-07-18 entry found a real deviation improvement doing this, at a documented, real cost
  (+41% travel time on that trial). Never re-verified against a freeze regression with the current
  `iteration_count: 2`.
- Accepting this as a known, goal-shape-dependent limitation rather than continuing to search for
  a single fixing parameter, given five independent attempts across both the critic layer and the
  physical PID layer all converged on the same "no meaningful change" result.

## 2026-07-20 - User-tuned fix for goal accuracy: `minimum_turning_radius` 0.7 -> 1.0 paired with `yaw_goal_tolerance` 0.3 -> 0.5

User made these two changes directly (hands-on investigation, per their own request in the prior
session to be pointed at the relevant parameters rather than have further automated tuning done
for them) and confirmed the robot now navigates to goals "almost accurately" - a clear, real-world
improvement over the prior committed config.

### Changed
- **`config/nav2/nav2_params.yaml`**: `planner_server.GridBased.minimum_turning_radius` raised
  from `0.7` to `1.0`.
- **`config/nav2/nav2_params.yaml`**: `controller_server.FollowPath.general_goal_checker.
  yaw_goal_tolerance` widened from `0.3` to `0.5`.

### Reconciling with the earlier, contradictory finding
2026-07-19 (2)/(3) documented live-testing `minimum_turning_radius: 1.0` on its own (with
`yaw_goal_tolerance` still at `0.3`) and finding it made a K-turn reproduction goal *worse* - the
same reversal/loop-path shape as `0.7`, just scaled up to a wider swing. That result isn't wrong,
but it was only ever tested as a single-parameter change. This entry changes both parameters
together, and the combination behaves differently: the working hypothesis is that a looser final-
heading tolerance means `general_goal_checker` no longer demands the tight exact-heading match
that was pushing the planner toward the extreme loop-and-reverse maneuver in the first place -
loosening `yaw_goal_tolerance` removes the pressure that made a larger `minimum_turning_radius`
counterproductive on its own. Not independently re-verified by Claude with a repeat-trial harness
(this was the user's own direct hands-on testing, not the multi-trial methodology used earlier in
this session for MPPI-noise-sensitive parameters) - if goal accuracy regresses on some other goal
shape, that's the first thing to check.

### Known limitation
- No quantitative multi-trial data (position/heading error, deviation) was captured for this
  change, unlike several other entries in this file - it's a real, user-confirmed qualitative
  improvement ("almost accurate"), not yet backed by the same rigorous measurement used for e.g.
  the 2026-07-19 (4) batch_size investigation. If further tuning is done on top of this, consider
  building the same repeatable trial harness before drawing conclusions, given how much MPPI/
  planner run-to-run variance has bitten single-run comparisons earlier in this project.

## 2026-07-19 (5) - Root-caused and fixed "robot turns before the curve" - `EarlyCommitCritic` window too wide

User reported the robot turning before the planned curve actually starts and deviating enough
from the plan to risk hitting obstacles - a safety-relevant report, investigated directly rather
than guessed at.

### Reproduction
A goal whose path runs straight for ~1m before curving (curve starts around path point 15-16,
confirmed via a `/plan` dump) - deliberately different from prior reproduction goals, which all
curved immediately from the start. Traced `/odom` from the exact moment the goal was sent (a
combined concurrent goal-send + trace, after discovering a gap between sending a goal and starting
a trace let a fast-resolving goal finish before the trace even began) and compared against the
plan at matching x-positions:
- x~0.5 (still deep in the straight section): actual y=0.027 vs planned y=0.029 - on-path.
- x~0.86 (plan is still straight - curve doesn't start until x~1.07): actual y=0.084 vs planned
  y=0.048 - already curving.
- x~1.0: actual y=0.169 vs planned y=0.055 - over 3x the planned lateral offset.

### Root cause
`EarlyCommitCritic.active_path_points: 15` - this critic (weight 15.0, the highest of any critic
in `FollowPath.critics`) has no distance/urgency gating at all, unlike stock `PathAngleCritic`'s
`max_angle_to_furthest`. It pulls every sampled trajectory toward the bearing to a near-term
look-ahead path point for the entire first 15 path points of any trip, regardless of whether a
curve is actually imminent. This reproduction goal's curve happens to start right around point
15-16 - squarely inside that window - so the critic kept reaching toward the upcoming curve well
before the vehicle had actually traveled far enough to be at its start.

### Fix
Lowered `active_path_points` from 15 to 8, shrinking the window in which this critic can reach
into a not-yet-current curve. Live-verified on the same reproduction: at the same x~1.0 comparison
point, actual y dropped to ~1.9x the planned value (down from ~3x) - a real, measured reduction,
though not full elimination (see "Known residual" below). **Checked for regression** against the
original case this critic exists for - a curve starting at literal trip start, `(0.6, 0.8,
90deg)` - before adopting: still completes cleanly, `Reached the goal!` / `Goal succeeded`, no
change in that behavior.

### Known residual
The deviation is reduced, not eliminated. `PathFollowCritic.offset_from_furthest: 3` and
`PathAngleCritic.offset_from_furthest: 2` (both below) are NOT gated by `active_path_points` -
they look ahead a fixed number of path points for the *entire* trip, so some early-anticipation
effect from those two remains by design of the fix already applied in 2026-07-18 (2)/(3) for a
different symptom ("turning early into a curve" - see those `TUNING.md` rows). If premature
turning is still visible near a real obstacle after this fix, the next lever is lowering those
two offsets further, trading against their own original purpose (pulling the trajectory promptly
into a curve that's genuinely starting).

### Methodology note
Hit two harness bugs while building the reproduction, both fixed for future use: (1) `/initialpose`
only resets AMCL's belief, not the robot's physical pose in Gazebo - reusing a stack across
multiple goals without a full relaunch left the robot at its previous goal's position, silently
invalidating the next trace; always relaunch the isolated stack between trials needing a clean
start. (2) an `rclpy` `ActionClient.send_goal_async()` call made outside an active spin loop can
silently never reach the server - confirmed via `controller_server` never logging "Received a
goal" at all. Reverted to the reliable pattern: concurrent `ros2 action send_goal` CLI (server
communication, separately proven reliable all session) + a separate Python `/odom` subscriber
launched in the same shell call with no gap, rather than a single combined rclpy script.

## 2026-07-19 (4) - Rigorous path-tracking-accuracy follow-up to (3): real improvement, real cost, reverted

User confirmed (1)/(2) resolved the "stuck" issue and asked for the remaining path-tracking
looseness from (3) to be fine-tuned properly, rather than left as an open question. (3) had
already shown that single-run "bump a critic weight" experiments were unreliable due to MPPI's own
sampling variance - this entry does it properly: a repeatable trial harness
(`/tmp/.../scratchpad/deviation_study/`, not committed - the method matters here, not the scripts)
with an objective metric (nearest-point distance from each `/odom` sample to the `/plan` polyline:
max/mean/p90 deviation) and **2 trials per configuration**, not 1, on the same reproduction goal
as (2)/(3).

### Baseline (batch_size 2000, current committed config)
2 trials: max deviation `0.43m` / `0.36m`, mean `0.17m` / `0.15m`, p90 `0.41m` / `0.33m`. Tight
enough agreement between trials to trust this as a real baseline, not noise.

### Candidate: raise `batch_size` (more samples/cycle = lower sampling variance, unlike a critic
weight - a principled choice after (3) showed weight-tuning was dominated by noise)
- **`3500`**: 2 trials, both markedly tighter - mean `0.14m` / `0.11m`, p90 `0.25m` / `0.26m`.
  Reproducible improvement. But `controller_server`'s 50ms/cycle budget (`controller_frequency:
  20.0`) started getting missed heavily: 41 and 48 "Control loop missed its desired rate" warnings
  per trial, vs 1 per trial at the 2000 baseline.
- **`2500`** (a gentler step): 2 trials, nearly as tight as `3500` - mean `0.11m` / `0.10m`, p90
  `0.26m` / `0.25m` - with far fewer missed-rate warnings (8, 10). Looked like the right Pareto
  point on the tracking-accuracy metric alone.
- **Checked the full goal outcome, not just the 20s tracking window, before adopting `2500`** -
  good instinct: one of the two trials took **4.5 minutes** to finally report `Goal succeeded`
  (vs ~30s at the `2000` baseline), cycling through 7+ `progress_checker` recoveries while
  oscillating just outside tolerance near the goal, before eventually converging. The extra
  per-cycle CPU cost that tightened mid-path tracking also degraded control-loop timing badly
  enough, specifically during final-approach fine correction, to make convergence dramatically
  slower and less reliable near the goal - a cost the 20s trace alone didn't reveal.

### Reverted to `batch_size: 2000`
The mid-path tracking improvement is real and reproducible, but not worth the demonstrated risk to
final-goal convergence reliability. Documented the full trial data directly in
`nav2_params.yaml`'s `batch_size` comment so this isn't retried blindly - if pursued again,
budget for it explicitly (e.g. lower `time_steps`/`iteration_count` to make room, on a platform
with headroom to spare for measurement) and check full goal-completion time across several trials,
not just early-window tracking accuracy, before adopting.

### Net status
Path-tracking looseness (from (2)'s freeze fix widening `vx_std`/`wz_std`) remains a real, open,
lower-priority trade-off - genuinely harder to fix within the current CPU budget than it first
looked, not a quick parameter tweak. The core stuck/frozen bug from (1)/(2) remains fixed and
unaffected by any of this round's experiments (all reverted).

## 2026-07-19 (3) - Investigated "robot doesn't turn as much as the planned curve" - inconclusive, left at default

User confirmed (2) below resolved the "stuck" issue, with one remaining complaint: the robot
doesn't turn as much as the planned curve calls for. Live-confirmed via a `/plan` vs `/odom`
comparison on the same reproduction goal as (2): at a point where the plan requires `x~0.495`
(mid-arc), the actual trajectory was already at `x~0.348` - cutting the corner by about 0.15m.
This is a real, expected trade against (2)'s fix: restoring `vx_std`/`wz_std` to their wider
defaults (needed to stop MPPI's warm-start collapse) also loosens how tightly it tracks the plan's
exact shape.

Tried compensating by raising `PathAlignCritic.cost_weight` (the primary path-adherence critic,
at the stock default `10.0`, not previously overridden):
- `14.0`: live-tested **worse**, not better - the trajectory overshot the *other* direction
  (`x~0.84-0.89` against a planned `~0.49` at the same y) and stalled oscillating there instead of
  cutting the corner.
- `12.0`: live-tested with the **same** overshoot-and-oscillate numbers as `14.0` - suspicious,
  since the pattern should shift with the weight if the weight were the real lever.
- A dense-sampled rerun at the **10.0 default** (as a control) swung even wider than either
  (`peak x~1.11`) on a different run of the identical goal and config.

**Conclusion**: MPPI's own sampling stochasticity produces enough run-to-run variance at the wider
`vx_std`/`wz_std` from (2) that a single-run "bump this weight" experiment isn't reliable evidence
in either direction - the three tested weights produced results that don't order sensibly by
weight, which is the signature of chasing noise rather than a real effect. Reverted to the
`10.0` default (left explicit in `nav2_params.yaml` with this investigation documented) rather
than commit to a value with no solid evidence behind it.

**Not resolved, and deliberately not chased further this round.** If tighter path-tracking is
worth pursuing: (a) run multiple trials per candidate weight, not one, before drawing conclusions,
given the demonstrated variance; or (b) address it via `vx_std`/`wz_std`/`iteration_count`
directly rather than a path-adherence critic, accepting some retuning of (2)'s freeze-vs-wobble
trade-off. The core "stuck/frozen" bug from (1)/(2) remains fixed and unaffected by this.

## 2026-07-19 (2) - Four stacked bugs behind "still not fixed": custom BT trees, turning-radius margin, MPPI warm-start collapse

User reported (1) below was "still not fixed" and provided a fresh full startup-through-failure
log: goal `(0.009, -2.004, ~180deg)` from the origin - 2m mostly in -y, with a large final
reorientation. Reproduced exactly in isolated headless testing. Investigation surfaced three more
distinct, stacked bugs beyond (1), each masking the next until fixed:

### Bug 2: blind periodic replanning was still tearing up long/slow maneuvers
`config/nav2/behavior_trees/*.xml` (rewritten by `qcar_nav2.launch.py`'s `RewrittenYaml` override)
already replans on a fixed timer (0.2Hz/0.15Hz, lowered from stock 1Hz/0.333Hz on 2026-07-15) -
enough for short goals, not enough for this one: still hit `Failed to make progress` /
`Aborting handle` every ~30s, replanned every 5s in between, indefinitely.

**First fix attempt made things far worse.** Tried "replan only if the path becomes invalid"
(this file already documents why a prior attempt at this was reverted: `IsPathValid` returns
SUCCESS on the empty/default `{path}` blackboard entry before `ComputePathToPose` ever runs).
Added an unconditional initial `ComputePathToPose`, gated with nav2_behavior_tree's
`SingleTrigger` decorator inside a `<Fallback>`/`<AlwaysSuccess>` wrapper to make it run once.
Live-tested: `/plan` was publishing at **~33Hz** - continuous replanning, far worse than before.
Root cause, confirmed via BT.CPP v3's actual source
(`BehaviorTree.CPP/src/controls/fallback_node.cpp`, tag 3.8.6): a plain `<Fallback>` calls
`resetChildren()` (halting every child back to IDLE) whenever it resolves to SUCCESS - and
`SingleTrigger` re-arms its own one-shot flag whenever ITS status is IDLE at tick-start
(`single_trigger_node.cpp`). So every time the Fallback succeeded, it silently reset
`SingleTrigger` back to "fire again," and `PipelineSequence` re-ticks that whole branch every
~10ms regardless.

**Fixed** by dropping `SingleTrigger`/`Fallback` and using a plain `<Sequence>` instead (confirmed
via `sequence_node.cpp`): unlike `Fallback`/`PipelineSequence`, a plain `Sequence` keeps its child
index as *persistent state* across ticks and only resets on the whole Sequence's own failure or
full completion - so `InitialComputePathToPose` (child 0) runs exactly once, then is skipped on
every later tick for as long as the inner `PipelineSequence` (child 1: the replan-gate +
`FollowPath`) keeps returning RUNNING. Re-arms correctly on a genuinely new attempt because
`RecoveryNode::tick()` calls `haltChild(0)` before every retry, and `Sequence::halt()` resets the
child index. `RateController` restored to stock 1.0Hz, since it now only gates a cheap
`IsPathValid` check, not a real replan. Applied to both `navigate_to_pose_...` and
`navigate_through_poses_...` trees. **Live-verified**: zero unwanted `/plan` republishes across a
30s+ window (previously one every 5s) - this bug is genuinely fixed, though (per the next two
bugs) fixing it alone did not make the reported goal succeed.

### Bug 3: planner and controller shared the exact same minimum turning radius - zero margin
With replanning-interruption eliminated, the goal still froze - but now demonstrably *not* from
being cut off: `cmd_vel` genuinely near-zero for the full uninterrupted window. Dumped the very
first `/plan` (clean, no cusp) and fit a circle through its early curve: **radius 0.49999999m**,
i.e. exactly `planner_server.GridBased.minimum_turning_radius` (0.5), which was deliberately kept
equal to `FollowPath.AckermannConstraints.min_turning_r` (also 0.5, the controller's real hard
limit) per this file's own prior comment ("keep these two in sync"). In hindsight, keeping them
equal is the problem: the planner is then free to produce curves at *exactly* the tightest radius
MPPI is allowed to drive, leaving MPPI's randomly sampled candidate trajectories zero margin - any
sample noisier than perfect (normal, given `vx_std`/`wz_std`) curves slightly tighter than 0.5m
and gets hit by `ConstraintCritic`'s turning-radius penalty, so MPPI kept preferring straighter,
cheaper trajectories over commitment.

**Fixed** by raising `planner_server.GridBased.minimum_turning_radius` to `0.7`, while leaving
`FollowPath.AckermannConstraints.min_turning_r` at the vehicle's true `0.5` limit - every planned
curve now has real slack before it's anywhere near what MPPI actually penalizes.

### Bug 4: MPPI's own warm-start made a stall self-reinforcing
Even with bugs 2 and 3 fixed, the same freeze-and-recover pattern persisted: a burst of real
progress right after each recovery-triggered `Optimizer reset`, then frozen again for the rest of
that ~30s window. Read `nav2_mppi_controller/src/optimizer.cpp`: each control cycle samples
candidate trajectories as noise added to the *previous* cycle's control sequence
(`shiftControlSequence()` warm-starts the next cycle from the last one) - so once that sequence
collapses toward near-zero velocity, every subsequent cycle keeps sampling near that same
near-zero point, with no escape mechanism short of a full `Optimizer::reset()` (which only happens
when `controller_server` accepts a brand new `FollowPath` goal, i.e. after a `progress_checker`
recovery). `iteration_count: 1` (one refinement pass per cycle) and `vx_std`/`wz_std` tightened to
0.15/0.3 on 2026-07-18 (to fix a *different* bug: path wobble) made this worse - tighter sampling
noise makes it harder to sample anything far enough from "near zero" to escape.

**Fixed** by restoring `vx_std`/`wz_std` to the plugin's shipped defaults (0.2/0.4) and raising
`iteration_count` from 1 to 2 (a second refinement pass per cycle, so MPPI can correct a bad
warm-start within the same cycle rather than relying on next-cycle sampling luck alone). This is a
genuine trade against the earlier wobble fix, not free - documented in `nav2_params.yaml` so a
future wobble regression isn't "fixed" by re-tightening these and reintroducing this freeze.

### Live-verified, full reproduction
Same exact goal as the user's log (`(0.009, -2.004, ~180deg)` from the origin), same isolated
headless method:
- **Before any of bugs 2-4 fixed**: robot crawls to ~x=0.3-0.6 in bursts between 30s
  `Failed to make progress` cycles, never gets anywhere near the goal.
- **After all fixes**: robot reaches the goal's vicinity (within ~0.2m) by **t=30s** (previously
  still near the origin at t=150s+), and `controller_server: Reached the goal!` /
  `bt_navigator: Goal succeeded` fire cleanly.
- **Regression check**: a separate short/normal goal (no large reorientation) from a live,
  already-driving robot position completed in ~11s with zero `Failed to make progress` cycles -
  some `Control loop missed its desired rate of 20Hz` warnings from `iteration_count: 2`'s doubled
  per-cycle CPU cost, not enough to affect the outcome.

## 2026-07-19 (1) - `EarlyCommitCritic` fought a legitimate reverse (K-turn) segment - made it direction-aware

User reported a specific failure rhythm: for a goal at an angle to the robot's current heading,
the planned path curves immediately, the robot moves forward a bit without steering, deviates
from the path, the path gets replanned, and the cycle repeats until the goal fails.

### First lead, investigated and ruled out
`bt_navigator.default_nav_to_pose_bt_xml` (empty in `nav2_params.yaml`) initially looked like the
cause - nav2's stock default tree unconditionally replans at 1Hz with no path-validity check,
which matches the reported rhythm closely. Reading `config/nav2/behavior_trees/` (already
overridden by `qcar_nav2.launch.py` via `RewrittenYaml`) showed this was a dead end: this package
already lowered the replan rate to 0.2Hz (every 5s) on 2026-07-15 for exactly this class of
symptom, and its detailed comment documents a real nav2-Humble bug already found and reverted -
switching to "replan only if path becomes invalid" makes `IsPathValid` evaluate the
default-constructed empty `{path}` blackboard entry as trivially valid *before*
`ComputePathToPose` ever runs, so the planner is never called and `FollowPath` gets an empty path
forever. Not reattempted.

### Live reproduction
Isolated headless test (separate `ROS_DOMAIN_ID`/`GAZEBO_MASTER_URI`), goal `(0.6, 0.8, 90deg)`
from the origin - reproduced exactly: robot advanced to roughly `(0.61, 0.19)`, yaw ~22deg, then
froze (`cmd_vel` ~0.003 m/s / ~0.007 rad/s - noise-level) for 30s, hit
`controller_server: Failed to make progress` / `Aborting handle`, recovered, and repeated. Dumping
`/plan` mid-stall showed why: `SmacPlannerHybrid`'s replanned path from that position was a
Reeds-Shepp K-turn - a short *reverse* segment (3 poses moving away from the goal) before curving
forward into it, exactly what `motion_model_for_search: "REEDS_SHEPP"` and
`change_penalty`/`minimum_turning_radius` are supposed to produce for a sharp reorientation this
close to the goal.

### Root cause
`EarlyCommitCritic` (added in (9) below) always scored trajectory yaw against the bearing *toward*
the near-term target, assuming forward travel. For the K-turn's reverse segment, the physically
correct yaw points *away* from that bearing (~180deg off) - the critic scored the correct reverse
maneuver as maximally wrong, fighting it directly. Combined with `ConstraintCritic`'s turning-radius
penalty (already discouraging any `wz` without matching `vx`), MPPI's cheapest option became
near-zero velocity - the same class of frozen local optimum (9) fixed at trip start, recurring
later in a trip wherever the path needs a reverse.

### Fix
`nav2_mppi_controller/tools/utils.hpp`'s own `posePointAngle()` already has this exact allowance
(`forward_preference` parameter: "if reversing direction is valid", return the smaller of the
diff to the target bearing or to `bearing + pi`). Added the same behavior to `EarlyCommitCritic`:
a new `forward_preference` param (default `false`, matching this vehicle's `AckermannConstraints`
always allowing reverse), and the per-timestep cost is now
`min(|yaw - bearing|, |yaw - (bearing + pi)|)` instead of always `|yaw - bearing|` -  scores
"pointing along the near-term path in either direction" rather than assuming forward-only, letting
`ConstraintCritic`/`PathFollowCritic`/etc. actually decide whether forward or reverse is cheaper.

### Live-verified
Same reproduction goal, rebuilt critic, isolated test: robot reached within ~0.12m/~4deg of
`(0.6, 0.8, 90deg)` within 10s (previously frozen indefinitely at that position) -
`controller_server: Reached the goal!` / `bt_navigator: Goal succeeded`. One
`progress_checker`-triggered recovery cycle (`backup`) still occurred during final-approach
convergence near the goal - matches the pre-existing, separately-documented limitation from (9)
and the (6)/(7) history, not the bug this fixes.

## 2026-07-18 (9) - Code-level fix: custom MPPI critic so the robot commits to a path that curves from the start

Follow-up to (7)/(8). User confirmed (8)'s `/joint_states` fix was real but didn't fix the
underlying "robot doesn't move" symptom, and explicitly asked for a code-level fix rather than
further parameter tuning - correctly, per the investigation in (7): nav2's own `PathAngleCritic`
provides no heading guidance at all for moderate initial direction mismatches (gated by
`max_angle_to_furthest`, default ~69deg, and it only scores the *average* heading error across
the whole rollout, not the near-term approach), and live A/B testing (disabling
`ConstraintCritic`, loosening `PathAngleCritic.max_angle_to_furthest`) failed to fix it or made
things worse. No existing critic, tuned any way, closes this gap - it needed new code.

### What was built
A new custom MPPI critic, `EarlyCommitCritic`, added directly to this package:
- **`include/qcar_updated/critics/early_commit_critic.hpp`** / **`src/critics/early_commit_critic.cpp`**
  - Scores only the first `early_time_steps` steps of each sampled trajectory against the bearing
    to a near-term path point (`offset_from_furthest` indices ahead), with no
    `threshold_to_consider`/`max_angle_to_furthest` gating - so it always pushes MPPI to start
    turning toward the path immediately.
  - Gated on `active_path_points`: only scores while `furthest_reached_path_point` is still small
    (default 15). Live-tested first *without* this gate - it fixed the stuck-at-start case but
    broke ordinary path-following for the rest of every trip, since a fixed near-term target
    stops being meaningful once the robot has made real progress and starts fighting the normal
    path/goal critics. Re-added the gate and both cases work.
  - Must live in `namespace mppi::critics` (not `qcar_updated::critics`):
    `nav2_mppi_controller`'s `CriticManager::getFullName()`
    (`nav2_mppi_controller/src/critic_manager.cpp`) hardcodes the `"mppi::critics::"` prefix when
    resolving names from the `critics: [...]` list, so a plugin in any other namespace is simply
    unreachable regardless of pluginlib export.
- **`qcar_critics.xml`** - pluginlib plugin description, exported via
  `<nav2_mppi_controller plugin="...">` in `package.xml` (matching the tag nav2_mppi_controller's
  own `package.xml` uses for its own critics, not the incorrect `<mppi_core>` guess tried first).
- **`CMakeLists.txt`** / **`package.xml`** - added the build target and
  `nav2_mppi_controller`/`nav2_costmap_2d`/`rclcpp`/`rclcpp_lifecycle`/`pluginlib`/`xtensor`/`xsimd`
  dependencies.
- **`config/nav2/nav2_params.yaml`**: added `"EarlyCommitCritic"` to `FollowPath.critics`, with
  `offset_from_furthest: 3`, `early_time_steps: 10`, `active_path_points: 15`, `cost_weight: 15.0`.

### A real crash, root-caused and fixed along the way
First working build crashed `nav2_container` (SIGSEGV, taking down `controller_server`,
`planner_server`, everything in that process) on literally the first control cycle of any goal.
Bisected by adding numbered `RCLCPP_INFO_ONCE` checkpoints through the function and rebuilding
between each: every individual step (path lookup, trajectory shape reads, bearing computation,
the per-timestep loop, even a trivial hardcoded `data.costs += xt::pow(ones*weight, power)`)
reported success right up to the crash - pointing away from ordinary logic bugs.

Root cause: `nav2_mppi_controller`'s own `CMakeLists.txt`
(github.com/ros-navigation/navigation2, humble branch) sets `add_definitions(-DXTENSOR_ENABLE_XSIMD)`
/ `add_definitions(-DXTENSOR_USE_XSIMD)` globally for its whole build. `XTENSOR_USE_XSIMD`
changes `xt::xtensor<float,N>`'s actual memory layout/alignment at compile time. This package's
`CMakeLists.txt` didn't define it, so this plugin's translation unit and nav2_mppi_controller's
precompiled binary disagreed on the layout of the "same" C++ type - a silent ABI mismatch. Simple
operations (assignment, `+= 0.0f`) happened not to touch the affected code paths and looked fine;
`xt::pow` (a vectorized xsimd-accelerated operation) did, and corrupted memory crossing the
plugin boundary via the shared `CriticData` reference.

Fixed by adding the identical `add_definitions(-DXTENSOR_ENABLE_XSIMD)` /
`add_definitions(-DXTENSOR_USE_XSIMD)`, `find_package(xsimd REQUIRED)`, and
`target_link_libraries(qcar_critics xtensor::optimize xtensor::use_xsimd)` to this package's
`CMakeLists.txt`, matching nav2_mppi_controller's own build exactly. A full clean rebuild
(`rm -rf build/qcar_updated install/qcar_updated`) confirmed no stale object files carried the
old ABI forward. Live-verified with the real critic logic restored: no crash across multiple
clean test runs.

### Live-verified
Isolated headless testing (separate `ROS_DOMAIN_ID`/`GAZEBO_MASTER_URI` from any locally running
session, per this session's established practice of never interfering with the user's own
running stack):
- **Before**: the reproduction goal (an immediate ~90deg curve from a standing start) left the
  robot's pose essentially frozen (sub-cm movement) for the full test window, matching the user's
  video and log evidence.
- **After**: the same goal reliably produces substantial, correct movement toward the goal -
  reaching within a few cm of the target position and closing most of the heading error, where it
  previously never moved at all. A short direct-approach goal (no curve-from-start) still
  completes correctly with the `active_path_points` gate in place, confirming no regression to
  ordinary path-following.

### Known limitation
Some runs still hit one `progress_checker`-triggered recovery cycle (backup, then a clean
replan/completion) before finally converging, rather than a single clean approach. This matches
the separate, still-open `progress_checker`/replan-timing behavior already documented in (6)/(7)
- not a new issue introduced here, and not something this critic was meant to address (the fatal
"never moves" case is fixed; a slower-than-ideal-but-successful final approach is not the same
class of problem).

## 2026-07-18 (8) - Fix real code bug: `/joint_states` was fake/static, not read from physics

User pushed back hard that (7) still wasn't right and this "doesn't seem like a parameter issue"
at all, with a new screen recording showing the robot motionless in RViz for the full ~80s clip
even though `/plan` was a clean, single-direction curve (no wiggle, no cusp - ruling out both (6)
and (7)'s mechanisms). That pushed the investigation off parameters entirely and into the
mechanical/plugin layer, and there was a real bug there.

### Root cause (source-verified, then live-proven)
Read `gazebo_ros_pkgs/gazebo_plugins/src/gazebo_ros_ackermann_drive.cpp`: this plugin does **not**
publish `sensor_msgs/JointState` at all (confirmed - no such publisher in the source; it only
publishes odometry/TF and an optional scalar steer angle). Checked `urdf/qcar_model.xacro` and
`launch/qcar_updated.launch.py`: there was no Gazebo plugin publishing real joint states either -
the *only* thing publishing `/joint_states` was the standalone ROS `joint_state_publisher` node,
which has no hardware feedback and no GUI, so per its own design it publishes a single static
(all-zero) position for every joint once and never updates it.

This looked exactly like "the steering physically can't move": every diagnostic reading
`/joint_states` (including my own, earlier in this investigation) saw `base_hubfl_joint` /
`base_hubfr_joint` pinned at exactly `0.0` forever, regardless of commanded velocity.

**Live-proven to be a false reading, not real physics**, by bypassing the broken topic entirely
and reading the `base`->`hubfl` TF transform directly (published by `gazebo_ros_ackermann_drive`
itself from the real physics engine): with a sustained raw `/cmd_vel` command, this real TF
showed the hub angle climbing smoothly from 0deg to a stable ~24deg over about 1.5s and holding -
correct, working steering. `/joint_states` for the same joint, at the same time, still read
`0.0`. The mechanism was never broken; the topic reporting it was lying, and RViz's rendered
robot model (driven by `/joint_states` via `robot_state_publisher`) would have shown the wheels
as frozen even while the vehicle was physically steering - which is almost certainly what the
user was seeing and rightly didn't believe was a tuning issue.

### Changed
- **`urdf/qcar_model.xacro`**: added a `libgazebo_ros_joint_state_publisher.so` Gazebo plugin
  listing all 6 non-fixed joints (`base_hubfl_joint`, `base_hubfr_joint`, `hubfl_wheelfl_joint`,
  `hubfr_wheelfr_joint`, `base_wheelrl_joint`, `base_wheelrr_joint`), publishing real physics-read
  positions/velocities at 30Hz.
- **`launch/qcar_updated.launch.py`**: removed the standalone `joint_state_publisher` node - it
  had no real data source and would otherwise still contend for the same topic with the new
  plugin.

### Live-verified
Re-ran the exact same sustained-command test after the fix: `/joint_states` for
`base_hubfl_joint`/`base_hubfr_joint` now tracks the real angle in real time (matching the TF
measurement, converging to ~0.42/0.34 rad), and all 6 joints - including the continuously-spinning
wheel joints, previously entirely absent from any real feed - report live, physically accurate
positions and velocities.

### Scope of this fix - what it does and doesn't explain
This is a real, confirmed, non-parameter code bug, now fixed. It explains why the robot's wheels
would have visually appeared frozen in RViz regardless of what was actually happening physically.
It does **not** fully explain why the robot's actual body pose stays motionless in nav2-driven
runs like the one in the user's video: live-testing (with the topic bug now understood, reading
real TF throughout) showed that during an actual MPPI-driven `NavigateToPose` goal requiring an
immediate curve, the commanded steering angle is real and initially substantial, but decays
smoothly back to ~0deg over roughly 15-20s and *stays* there for the rest of the run - meaning
MPPI itself is not committing to sustained turning for this goal shape, separately from anything
mechanical. That remaining behavior is the same class of MPPI local-optimum issue investigated in
prior entries this session, still not resolved by parameter tuning attempted so far. Given the
user's now-repeated and correct instinct that tuning alone hasn't been enough, the honest next
step if this remains unacceptable is a code-level change to MPPI's critic gating or a different
control strategy for this goal shape - not another parameter sweep.

## 2026-07-18 (7) - Fix (partial): robot doesn't move at all when the path starts with a turn

Follow-up to (6). User clarified this isn't the mid-route cusp case from (6) but specifically:
the robot doesn't move when the *first* thing the planned path needs is a turn (i.e. the robot's
starting heading doesn't already match the path's initial tangent) - and pushed back that this
didn't look like an MPPI/controller tuning problem. That redirected the investigation to the
planner, and they were right.

### Root cause (source-verified)
Live-dumped `/plan` for a goal directly behind the robot's start heading (matching the user's
log: start ~0deg, goal needing ~180deg) and found `SmacPlannerHybrid` inserting a small
**reversal** as literally the first segment of the path - drive back ~0.09m, then immediately
forward into the real route - even though a reverse was never needed for this goal. Confirmed via
live `cmd_vel`/pose logging that the robot was not actually frozen (`cmd_vel` showed real,
nonzero commands) but the small reversal-then-forward at the start meant net displacement stayed
near zero, and this repeated on every replan, matching the "doesn't move" symptom.

Read `nav2_smac_planner/src/node_hybrid.cpp`'s `getTraversalCost()`: `change_penalty` is applied
whenever consecutive motion primitives change turning direction, and the function's own comment
literally says "penalizes wiggling." It defaults to `0.0` (confirmed against
`smac_planner_hybrid.cpp`'s parameter declarations - `0.0` was already nav2's actual default, not
something misconfigured in this repo) - so the planner has **zero** cost for inserting this kind
of unnecessary direction-changing wiggle. This is a planner-level issue, not something any amount
of `FollowPath`/MPPI critic tuning could fix, matching the user's instinct.

### Changed
- **`config/nav2/nav2_params.yaml`**: `planner_server.GridBased.change_penalty` raised from `0.0`
  to `3.0`.

### Live-verified
- Before: `/plan` for the reproduction goal started with a ~0.09m reversal before the real route;
  robot's pose stayed pinned at ~(0.087, 0.000) for the full 56s test window (yaw crept only
  ~0.6deg) despite continuous nonzero `cmd_vel`.
- After: `/plan` for the identical goal/start is a single continuous forward arc from the first
  point - no reversal. Live re-ran the full `NavigateToPose` goal: the robot is no longer frozen
  at the start and makes substantial, sustained real progress (confirmed via TF pose over 80s+,
  e.g. `(0.0,0.0) -> (2.2,-0.5) -> (4.3,-3.3)` and continuing to move/turn), which it never did at
  all before this change.

### Known limitation - does not fully solve goal-reaching for large heading changes
The specific reported symptom ("doesn't move") is fixed. But live-testing surfaced a related new
issue: for a goal needing a large (~180deg) heading change, `change_penalty: 3.0` combined with
the already-raised `reverse_penalty: 4.0` (see 2026-07-15) now pushes `SmacPlannerHybrid` toward
a long, wide forward-only loop instead of a short reverse - and on the live test run, the robot
followed that loop out several meters past the direct goal, then stalled again (this time
rotating in place for tens of seconds without translating) before the test was stopped. Not yet
resolved - see (6) for the related, still-open mid-route cusp investigation. Both point at the
same underlying tension: `reverse_penalty` and `change_penalty` were raised (independently, on
different dates) specifically to discourage the planner from using reversal/direction-change
segments, but this vehicle sometimes genuinely needs them, and nav2 Humble's Reeds-Shepp handling
in this stack remains fragile for exactly those cases (consistent with the "known-fragile
combination" noted in `planner_server.GridBased`'s own comments, predating this session).

## 2026-07-18 (6) - Investigated (not fixed): robot gets stuck when the path has a reverse-then-forward segment

User reported the robot getting stuck when the planned path includes a reversal followed by
continuing forward, with a screen recording (RViz top, terminal bottom) as evidence. **No changes
committed to `nav2_params.yaml` for this entry** - investigated live via an isolated reproduction
(separate `ROS_DOMAIN_ID` and `GAZEBO_MASTER_URI` from the user's own running session, to avoid
interfering with it) but did not reach a confident fix within the investigation, at the user's
choice to pause here rather than keep iterating live. Documenting findings so the investigation
isn't lost.

### Reproduced
Live, repeatedly, using the existing K-turn benchmark goal `(2.0, 1.0, 179deg)` from a clean
headless sim: `controller_server` cycles through repeated `Failed to make progress` ->
`Aborting handle` -> recovery (clear costmaps / wait / backup) -> full replan, without ever
reaching the goal (multiple attempts run 60-150s+ without success).

### Ruled out as the cause
Reverted all of this session's earlier (5) changes live
(`GoalAngleCritic.cost_weight` back to `3.0`, `PathFollowCritic`/`PathAlignCritic.
threshold_to_consider` back to `0.5`) and reproduced the identical failure - **this is a
pre-existing issue, not a regression from (5)'s tuning.**

### Root cause candidate (source-verified, not yet confirmed as complete)
Read `nav2_mppi_controller/src/path_handler.cpp`: MPPI only ever tracks the path up to the first
reversal/cusp (`utils::removePosesAfterFirstInversion`) until the robot satisfies
`PathHandler::isWithinInversionTolerances` - **both** `inversion_xy_tolerance` (default `0.2`m)
**and** `inversion_yaw_tolerance` (default `0.4`rad) simultaneously against the cusp pose - only
then is the rest of the path released. Live-dumping `/plan` across several replan cycles showed
`SmacPlannerHybrid` frequently inserting a *new*, small corrective reversal right at the start of
each freshly (re)computed plan (not only the "real" K-turn deeper in the route) - plausibly
because the robot's actual heading at replan time rarely matches the path tangent the planner
assumes exactly. These small wiggles cover very little net linear distance, so
`progress_checker.movement_time_allowance` (`30.0`s) times out, the BT
(`navigate_to_pose_w_replanning_and_recovery.xml`) runs its recovery round-robin, and then
replans from scratch - which tends to produce another small wiggle at the new start point,
repeating the cycle. This dovetails with a previously-documented, only-partially-fixed issue
already described in that BT XML's own comments (replanning interrupting an in-progress K-turn
before it could finish) - today's finding is a related but distinct manifestation: the *retry
after failure* path re-triggers the same problem, not just a too-fast periodic replan.

### Tried live, did not fully resolve
Loosened `inversion_xy_tolerance` / `inversion_yaw_tolerance` to `0.4`m / `0.8`rad live - the
robot visibly made more net progress per attempt (advanced further from the start point across
replans) but still did not reliably reach the goal within the test budget used. Not committed to
the YAML since it's an incomplete fix and the tradeoffs (a looser cusp handoff) weren't fully
characterized.

### Next steps (not yet attempted)
- Investigate `SmacPlannerHybrid` tuning to stop it generating start-of-plan corrective
  reversals in the first place (more root-cause than loosening the MPPI-side tolerances) -
  candidates: `analytic_expansion_ratio`, `angle_quantization_bins`, `cost_penalty`, or how it's
  seeding the search from the robot's current heading.
- Consider a BT-level change so a `Failed to make progress` abort near a cusp doesn't force a
  full replan-from-scratch (which re-triggers a new wiggle) - e.g. distinguishing "stuck at a
  cusp, keep trying the same plan a bit longer" from "genuinely stuck, replan."

## 2026-07-18 (5) - Fix "robot doesn't move on a new goal that's mainly a reorientation"

User reported the robot not moving when given a new nav goal shortly after reaching a previous
one, particularly when the new goal mainly needs a heading correction (e.g. re-sending
essentially the same position with a different final yaw). Reproduced live and repeatedly:
`cmd_vel` converges to near-zero and stays there, heading frozen tens of degrees short of target,
for 60s+ with no progress.

### Root cause
Read `utils::withinPositionGoalTolerance` (`nav2_mppi_controller/tools/utils.hpp`, used by
`PathAlignCritic`, `PathFollowCritic`, and `PathAngleCritic`'s `threshold_to_consider` gate): it
measures distance from the **robot's current pose** to the goal, not progress along the plan.
When a new goal sits at (or very near) the robot's current position - which a "same position,
new orientation" goal does by definition - that distance is ~0 from the very first control
cycle, so all three of MPPI's path-adherence critics are disabled for the *entire* maneuver, not
just the final approach. (Confirmed via `ComputePathToPose`: `SmacPlannerHybrid` does plan a real,
sensible loop-back maneuver up to ~0.2m from the goal for this case - the plan is fine, nothing is
enforcing it.) That leaves only `GoalAngleCritic` (default weight `3.0`) to drive the
reorientation, competing against `ConstraintCritic`'s turning-radius penalty (which discourages
high angular rate at low forward speed - exactly what a tight reorientation needs) under MPPI's
per-timestep-independent noise sampling. In practice this combination frequently converges to a
near-zero-velocity local optimum, and an Ackermann vehicle physically cannot change heading at
zero linear velocity - so nothing ever kicks it out.

### Changed
- **`config/nav2/nav2_params.yaml`**: `controller_server.FollowPath.GoalAngleCritic.cost_weight`
  raised from the default `3.0` to `10.0`, to strengthen the only critic still active in this
  near-goal regime.
- (Also corrected the reasoning previously logged in (5)'s predecessor edits to
  `PathFollowCritic`/`PathAlignCritic.threshold_to_consider` - see `TUNING.md` - that change does
  *not* fix this bug, since the gate is based on current-position-to-goal distance which starts at
  ~0 for this scenario regardless of the threshold's value. It remains in place because it's
  independently useful for a different, narrower "dead zone" during normal final approach.)

### Live-verified
Reproduced the failure repeatedly with a dedicated script (send goal A, wait for completion, wait
3s, send goal B at the same resulting position with yaw +90deg) against a clean, freshly-launched
headless sim each time - not a one-off. With the fix:
- A short, normal (non-reorientation) goal completes in the same ~5-11s as before the change - no
  regression on ordinary path-following.
- The reorientation-only goal no longer hangs indefinitely. It either converges directly or is
  unstuck by `controller_server`'s existing `progress_checker` + recovery-behavior retry loop
  (each retry calls `Optimizer::reset()`, giving MPPI a fresh independent attempt).

### Known limitation - not a complete, deterministic fix
This is a genuine improvement, not a guarantee. Because the underlying critic-gating gap
(described above) is unchanged, and MPPI's sampling is stochastic, the reorientation case still
sometimes takes one or more progress-checker retry cycles (observed range: instant up to ~90s
across repeated live trials) to resolve, and in at least one trial it exhausted the behavior
tree's retry budget and failed outright even with this fix applied. Tried `cost_weight: 30.0`
first - it did make the reorientation case converge faster and more directly, but live-testing
also showed it degrading a separate, normal long-distance goal's final approach (oscillation, a
`progress_checker` failure that a `10.0`-weighted run of the same goal did not trigger) - reverted
for that reason. Separately tried lowering `progress_checker.movement_time_allowance` from `30.0`
to `8.0` (faster retries) - live-tested, made the BT exhaust its retry count before recovering on
one trial (outright `Goal failed`), so left at `30.0`. If more reliable behavior is needed, the
next step would be a code-level fix (e.g. a dedicated in-place-reorientation behavior, or making
these critics' `threshold_to_consider` path-progress-based instead of goal-distance-based) rather
than further YAML tuning.

## 2026-07-18 (4) - Enable MPPI debug visualization topics for RViz

User asked which topic shows the "local" plan alongside `/plan` (the global path from
`planner_server`). Checked `nav2_mppi_controller/src/trajectory_visualizer.cpp` on GitHub: MPPI
publishes `/trajectories` (`visualization_msgs/MarkerArray` - every sampled candidate trajectory
each control cycle plus the chosen optimal one) and `transformed_global_plan`
(`nav_msgs/Path` - the local, controller-side portion of the plan MPPI is actively tracking,
distinct from `/plan`'s full raw global output) - but both are gated behind `FollowPath.
visualize`, which was `false`.

### Changed
- **`config/nav2/nav2_params.yaml`**: `controller_server.FollowPath.visualize` set to `true`.

### Live-verified
Confirmed both topics actually publish real data once enabled (not just advertised): `ros2 topic
list` showed both `/trajectories` and `/transformed_global_plan`, and `ros2 topic echo --once` on
each returned valid messages during an active goal (a populated `MarkerArray` under the
"Candidate Trajectories" namespace, and a `Path` with a valid `odom`-frame header respectively).

### Known limitation
- Adds publishing/marker-array overhead on every control cycle - fine for debugging, but should
  be turned back off (`visualize: false`) for normal, non-debugging runs if CPU/bandwidth ever
  becomes a concern (not observed to be a problem in headless testing, but never measured under
  load with RViz also subscribed and rendering).

## 2026-07-18 (3) - Fix `PathAlignCritic`'s much larger blind spot: inactive for the first 20 path points of every trip

Follow-up to (2). User reported the "turning early" symptom was still present after the
`PathAngle`/`PathFollowCritic` `offset_from_furthest` fix, with a second video as evidence. Read
`PathAlignCritic`'s actual scoring code (`path_align_critic.cpp`, `humble` branch) - its own
`offset_from_furthest` (still at the default `20`, never touched in (2)) works completely
differently than the other two critics' parameter of the same name: it's a gate, not a
look-ahead target - `if (path_segments_count < offset_from_furthest_) return;` means
`PathAlignCritic` (the highest-weighted critic in use, at `10.0`) is entirely inactive for the
first 20 path points of every trip. If a curve starts early in the plan, the strongest
path-adherence critic simply isn't running yet during that segment.

### Changed
- **`config/nav2/nav2_params.yaml`**: `FollowPath.PathAlignCritic.offset_from_furthest` lowered
  from the default `20` to `3`.

### Live-verified result
Reran the same isolated forward-curve goal (1.5, 1.5, 90deg, no reversal) used in (2) for a clean
before/after: max path deviation `0.123m -> 0.091m -> 0.083m` across the (2) and (3) fixes
respectively - measurably, consistently tighter each round. Captured the full raw per-sample
heading-lead trace this time (not just aggregate stats): every single sample stayed under 9cm
from the path. The "lead" values follow a sawtooth pattern - growing to 20-32deg then snapping
back near zero, repeating roughly every 5s, a cadence matching the planner's replanning cycle
(`max_planning_time: 5.0` s) rather than a persistent tracking failure. Some heading lead while
banking into an ongoing curve is normal anticipatory steering, not inherently a bug - a vehicle
tracking a curve well naturally points somewhat ahead of the exact instantaneous local tangent.

### Confirmed resolved by the user
Both videos were actually RViz (not Gazebo's motion-trail feature as Claude initially assumed),
and the green line in both was the real `/plan` topic (the planner's output), not a ground-truth
trail - correcting an earlier misreading of the visual evidence. With that corrected
understanding, the frames examined during this investigation (robot visibly offset from the
green `/path` line, e.g. cutting to the inside of a curve) were a genuine plan-vs-actual
comparison after all, not the ambiguous ground-truth-trail view Claude thought they were. User
confirmed directly: **the turning-early issue is resolved.** This closes out the (2)/(3)
`offset_from_furthest` investigation as an actual fix, not just a favorable-looking synthetic
measurement - the live-tested numbers (0.375m -> 0.091m -> 0.083m max deviation) and the user's
real-world confirmation now agree.

## 2026-07-18 (2) - Fix "turning early": `offset_from_furthest` was targeting a path point past the curve start

Follow-up to the entry below. User gave a precise symptom: the robot turns before the curve in
the planned path actually begins - anticipating turns too early rather than tracking the path's
actual local shape. Read the real scoring code for the two critics that could plausibly cause
this (`path_angle_critic.cpp`, `path_follow_critic.cpp`, both from
`ros-navigation/navigation2`'s `humble` branch): both target a path point `offset_from_furthest`
*indices* ahead of wherever the robot has currently progressed to along the path -
`PathAngleCritic` steers the robot's heading toward that point, `PathFollowCritic` pulls each
sampled trajectory's endpoint toward it. A target that far ahead can land past the start of an
upcoming curve, pulling the executed heading toward the curve's direction before the robot has
physically reached it - exactly the reported symptom.

### Changed
- **`config/nav2/nav2_params.yaml`**: `FollowPath.PathAngleCritic.offset_from_furthest` lowered
  from the default `4` to `2`; `FollowPath.PathFollowCritic.offset_from_furthest` lowered from
  the default `6` to `3`.

### Live-verified result
Built a new measurement (heading-lead vs. the planned path's local tangent direction, computed
from the closest path point at each sample) - not something previously tracked, so no "before"
baseline exists for direct comparison. The standard K-turn benchmark goal (2.0, 1.0, 179deg) is
unsuitable for this specific metric: it involves a reversal, during which the robot's heading is
*correctly* ~180deg from the path's direction-of-travel tangent (that's normal reversing
kinematics, not a bug), which swamped the measurement with irrelevant large values on a first
attempt. Retested with a goal requiring only a forward curve (1.5, 1.5, 90deg, no reversal) to
isolate the actual symptom: heading-lead vs. local path tangent came out to median `6.9deg` /
p90 `25.6deg` / max `33.2deg`, with `0.123m` max path deviation and the goal succeeding within
`0.088m` of the true target - a tight result, though not provably an improvement over the old
`offset_from_furthest` values without a same-goal "before" run.

### Known limitation
- No direct before/after comparison exists for this specific fix - the live test confirms the
  post-fix behavior is reasonably tight, not that it's better than the prior config on an
  apples-to-apples basis. If turning-early is still visible after this, lower
  `offset_from_furthest` further (e.g. `2`/`1`); if the car instead starts reacting *late* to
  real curves/corners (undershooting turns), raise back toward the defaults (`4`/`6`).
- The reversal-goal heading-lead measurement (median 37deg, max 179deg) was discarded as
  contaminated, but the underlying test infrastructure (`verify_early_turn.py` in scratch) is
  reusable if a future session wants to separate "genuine early-turning" from "expected reversal
  heading" more rigorously (e.g. by detecting the path's cusp index and excluding samples near it
  rather than excluding the whole goal).

## 2026-07-18 - Reduce MPPI sampling noise: sharper final accuracy, modestly tighter path-following

User reported the robot deviates from the planned path mid-trip and shared a screen recording as
evidence (`~/Videos/Screencasts/Screencast from 07-18-2026 09:46:18 AM.webm`). Extracted frames
via OpenCV (no `ffmpeg`/`ffprobe` available, only `libav*-dev` runtime libs - `cv2.VideoCapture`
worked directly, though frame-accurate seeking via `CAP_PROP_POS_FRAMES` failed on this VP9-encoded
webm and had to fall back to sequential `.read()` calls). The recording is Gazebo's own top-down
view with its built-in motion-trail feature (ground-truth trail of the robot's actual movement),
not an RViz overlay with the nav2-planned path drawn in - so it confirmed the *character* of the
motion (a jerky, tight period roughly mid-trip) but didn't provide pixel-comparable deviation
data against the plan. That already matched what live testing had measured and flagged as an
open issue in the (3) entry above (0.31-0.45m max deviation), so addressed it directly rather
than re-deriving the same fact from the video.

Checked `nav2_mppi_controller/src/optimizer.cpp` on GitHub for the real default values of
`vx_std`/`wz_std` (0.2/0.4) rather than assume the existing config was already at a sensible
baseline - confirmed the config was sitting exactly at the shipped defaults. Rather than raise
`PathAlignCritic`'s weight further (already the highest-weighted critic in use, at 10.0), lowered
the sampling noise instead - a different lever that controls how widely MPPI's candidate
trajectories spread each cycle, independent of critic weighting.

### Changed
- **`config/nav2/nav2_params.yaml`**: `FollowPath.vx_std` lowered `0.2` -> `0.15`; `wz_std`
  lowered `0.4` -> `0.3`.

### Live-verified result
Reran the standard test goal (2.0, 1.0, 179deg from origin) used throughout this investigation:
- Max path deviation during travel: `0.447m -> 0.375m` (modest improvement).
- Final position error: `0.139m -> 0.054m` (sharp improvement - well within the `0.12m`
  `xy_goal_tolerance` from (3), no more borderline latching issue).
- Final yaw error: `2.67deg -> 3.37deg` (essentially unchanged, still well within `0.3` rad
  tolerance).
- **Total travel time: `122.4s -> 172.4s`** - a real trade-off, not a pure win. Tighter sampling
  explores fewer candidate trajectories' worth of "boldness," converging more cautiously. Two
  `Failed to make progress` recoveries still occurred during this run (down from typically 3-4+
  in earlier tests), each adding to the elapsed time.

### Known limitation
- The travel-time increase (+41%) was not weighed against user priorities before making this
  change - if speed matters more than the last few cm/deg of accuracy for this deployment,
  `vx_std`/`wz_std` should be raised back toward the defaults, or partially split the difference
  (e.g. `0.17`/`0.35`) rather than kept at the current values. Ask before assuming accuracy
  should always win that trade-off.
- Path deviation improved only modestly (0.447m -> 0.375m still isn't tight) - if mid-trip
  deviation specifically is still the primary complaint after this, the video evidence suggests
  looking at the jerky/tight-turning period specifically (possibly still curvature/`ConstraintCritic`
  related) rather than assuming general noise reduction alone will fully resolve it.

## 2026-07-15 (3) - Found and fixed the actual cause: `PreferForwardCritic` was fighting the planned reversal

User visually confirmed (in RViz) the actual mechanism behind the K-turn failures: the plan
itself correctly shows a reverse segment, but the robot doesn't execute it - "the path shows a
reverse, but the robot is not moving on the planned path." This matched a precise hypothesis from
reading the real MPPI critic source (not guessed): `PreferForwardCritic` penalizes any reverse
velocity (`cost_weight` 5.0) for as long as the robot is more than its `threshold_to_consider`
(default 0.5m) from the goal, while `PathFollowCritic` - the critic that enforces sticking to the
planned path - stops applying inside *its own* `threshold_to_consider` (default 1.4m). That
leaves a 0.5m-1.4m "dead zone" around the goal where nothing enforces the planned path, but
`PreferForwardCritic` is still actively fighting any reverse motion. A K-turn's reversal segment
plausibly falls in exactly that zone.

### Changed
- **`config/nav2/nav2_params.yaml`**: removed `PreferForwardCritic` from `FollowPath.critics`
  entirely - its entire purpose (discourage reversing) is fundamentally at odds with this
  vehicle's confirmed requirement for real reversing capability, and is redundant with
  `planner_server.GridBased.reverse_penalty`, which already discourages *unnecessary* reversing
  at the planning level without fighting a reverse the plan has already decided is needed.
- **`config/nav2/nav2_params.yaml`**: `FollowPath.PathFollowCritic.threshold_to_consider` lowered
  from the default `1.4` to `0.5`, matching `PathAlignCritic`/`GoalAngleCritic` - closes the dead
  zone by keeping path-following guidance active up to the same final-approach boundary the other
  critics already use.
- **`config/nav2/nav2_params.yaml`**: `general_goal_checker.xy_goal_tolerance` widened slightly
  from `0.1` to `0.12` - see the live-test result below for why.

### Live-verified result (this one actually worked)
Reran both test goals used throughout this investigation:
- Goal (2.0, 1.0, 179deg) from origin: **succeeded**, yaw error improved from 6.97deg to
  **2.67deg**, position error 0.202m -> 0.139m. (Path deviation during travel went up, 0.313m ->
  0.447m - a real trade-off, not a pure win: removing `PreferForwardCritic` gives the planner's
  reverse decision more control, so the *pursuit* logic doesn't fight it, but it doesn't
  necessarily improve mid-path tightness.)
- Return-trip goal (0, 0, 0deg), the one that previously failed catastrophically (2.2m position
  error, 177deg yaw error, effectively never attempting the reversal): converged to **0.145m /
  0.22deg** of the true goal - a massive improvement, though it cycled through repeated
  `progress_checker`-triggered abort/retries instead of latching "reached," since 0.145m is just
  outside the old `0.1` `xy_goal_tolerance`. That's what prompted the tolerance widening above.

### Known limitation
- The `xy_goal_tolerance: 0.12` change was not independently re-verified after being added (made
  after the live tests above, based on their measured result) - if 0.145m convergence isn't
  representative and the car sometimes settles further out, this tolerance may need to go back
  down and the remaining gap addressed differently (e.g. `movement_time_allowance` or
  `GoalCritic.cost_weight` further).
- Path deviation during travel (0.3-0.45m measured) is still fairly loose - if that specifically
  is still a complaint once goal-reaching itself is confirmed reliable, `PathAlignCritic`'s weight
  (already the highest at 10.0) or `ConstraintCritic` may need attention next.

## 2026-07-15 (2) - AMCL tuning for localization jitter: real but incomplete fix; ruled out physics as the cause of K-turn failures

User reported the local map/lidar points visibly "drift" during the goal-approach reorientation
maneuver, plus continued goal-reaching failures. Live-measured the `map`->`odom` TF directly
(0.2s sampling) during a goal requiring a K-turn: confirmed real jumps up to 7.2cm / 2.3deg in a
single step, roughly every 5-10s - this is what renders as lidar points "jumping" in the map
frame, since every scan point's map-frame position depends on the current `map`->`odom`
correction. Likely cause: real wheel odometry drifts faster during tight/reversing motion (more
slip) than `alpha1-5: 0.2` assumed, so AMCL underestimated uncertainty between scan-match updates
and corrected in large, sudden jumps to catch up.

### Changed
- **`config/nav2/nav2_params.yaml`**: `amcl.alpha1-5` raised `0.2` -> `0.4` (all five); `amcl.
  update_min_d`/`update_min_a` lowered `0.25`/`0.2` -> `0.15`/`0.1` (re-localize more often, in
  smaller increments, rather than accumulating more drift between corrections).

### Live-verified result (honest, not a full fix)
Reran the identical live `map`->`odom` measurement after this change: jump *count* increased
(14 -> 72) while jump *magnitude* decreased (max 7.2cm -> max 5.6cm, mostly 1-3cm). This is a
real, measured change in the *character* of the jitter (more frequent, smaller corrections
instead of fewer, larger ones) - but it is **not clearly a visual improvement** (constant small
jitter could look worse than occasional bigger snaps, not better) and it **did not fix the
underlying goal-reaching failure** - the same test goal still timed out after 120s, ending at
(2.76, 0.06, 36deg) against a target of (2.0, 1.0, 179deg). Keeping this change since smaller,
more frequent corrections are the more theoretically sound direction and it's a plausible partial
improvement, but not claiming it resolves the reported symptom.

### Major finding: physics-level reversing is NOT the problem
Given RPP tuning, MPPI's controller swap, footprint/critic-weight fixes, and now AMCL tuning have
all failed to reliably fix K-turn-requiring goals, tested whether the vehicle's actual reversing
capability is itself unstable - bypassing all of nav2, publishing raw `/cmd_vel` directly to the
drive plugin and measuring `map`->`base` TF:
- **Pure reverse** (`linear.x: -0.15`, no steering) over 0.75m: lateral drift only 2mm, yaw drift
  0.01deg - essentially perfect.
- **Reverse + steering simultaneously** (`linear.x: -0.15`, `angular.z: 0.5`, the actual motion a
  K-turn segment needs) over ~5.5s of active turning: smooth, monotonic yaw change from ~0deg to
  ~93deg, no jumps, no instability.

**This rules out the drive plugin/vehicle physics as the cause of the K-turn failures.** The
robot can genuinely execute a clean, stable reverse-and-steer maneuver when commanded directly.
The remaining problem is confirmed to be entirely within the nav2 planning/execution stack
(`SmacPlannerHybrid`'s path geometry at the cusp, or `MPPIController`'s execution of it, or their
interaction) - not something a lower-level (URDF/PID/friction) fix would address.

### Known limitation / next steps
- Root cause of the remaining K-turn goal-reaching failures is still not found. Suggested next
  investigation (not yet done): inspect the actual `/plan` path geometry for a failing goal
  directly (not just aggregate success/failure) to see whether `SmacPlannerHybrid` is producing a
  sensible K-turn shape at all for these cases, before assuming the problem is in `MPPIController`
  execution again.
- Given the same class of goal (~180deg heading change) has now failed under four different
  configurations, it may be worth reconsidering whether every such goal is achievable at this
  vehicle's `minimum_turning_radius: 0.5` m in the available map space, rather than continuing to
  treat each failure as a newly-introduced bug.

## 2026-07-15 (1) - Live-verified: (11)'s fixes are a real but partial improvement, not a full fix

User reported "still not accurate" after (11) with no further detail. Rather than guess again or
ask for another log, ran a live headless test directly (gzserver + full nav2 stack, no display) -
the first time in this whole tuning thread that Claude verified behavior first-hand instead of
relying on the user's terminal output. Wrote a verification script using a `map`->`base` TF
lookup (not raw `/odom`, which is in a different, drifting frame not directly comparable to a
`map`-frame goal - an earlier version of the test script made this mistake and produced a
misleading number) to measure true position/heading error against the goal, and subscribed to
`/plan` to measure maximum deviation from the planned path during travel.

**Results, using a `navigate_to_pose` goal with a ~179deg final-heading mismatch from (0,0,0)
to (2.0, 1.0)**: succeeded, but not cleanly - position error 0.202m (target tolerance is 0.1m),
yaw error 6.97deg (within the 0.3 rad/~17deg tolerance), max path deviation 0.313m during travel,
took 91.6s. **A second goal** (return trip, similarly large heading mismatch) from the resulting
position **timed out completely after 180s**, with `Failed to make progress` still firing
repeatedly (~30s cadence, matching `movement_time_allowance`) - the same failure signature RPP
had, now reproduced with MPPI. Conclusion: (9)'s controller swap and (10)/(11)'s structural fixes
measurably improved things (this exact failure pattern used to be near-total for goals like this)
but did not eliminate it - there's still a real, unresolved failure mode for at least some
goal/position combinations requiring a tight reverse maneuver.

**Isolated one contributing factor via live A/B testing** (using `ros2 param set` on the running
`controller_server` node - `CostCritic`'s `cost_weight` and related params are dynamically
reconfigurable, no relaunch needed): temporarily set
`CostCritic.consider_footprint` back to `false` and reran the exact failing second goal. Result
was closer (position error 0.435m vs 0.696m, yaw error 20deg vs 80deg) but still timed out -
`consider_footprint: true` (from (11)) is a genuine contributing factor to failures in tight
maneuvering space (a stricter, more accurate footprint is harder to keep collision-free during a
K-turn than the old undersized circle was), but it is not the sole cause. The (11) fix is being
kept (footprint-aware collision checking against the car's real shape is correct behavior, and
made the first goal notably more accurate), but this trade-off - it makes tight reversals harder
to execute - is now a known, understood cost of that correctness, not a hidden regression.

### Not changed this round
No config changes were made - this was a diagnostic-only session establishing ground truth via
direct testing before further tuning, to avoid another round of unverified guesses. `nav2_params.yaml`
is unchanged from (11) (the live `ros2 param set` used for the A/B test was memory-only on the
already-running test node, not persisted).

### Suggested next steps (not yet done)
- The remaining "some goals succeed, some time out with repeated Failed to make progress" pattern
  now looks less like a pure algorithm/controller bug (MPPI measurably outperforms RPP here) and
  more like a genuine kinematic/spatial difficulty: this vehicle's `minimum_turning_radius: 0.5`m
  is large relative to some parts of this map, and a Reeds-Shepp K-turn reversal may need more
  clear space than is available at some goal/heading combinations. Worth checking whether the
  specific goals that fail are in tighter map regions than the ones that succeed, rather than
  continuing to treat every failure as the same bug.
- If corner-cutting/path-deviation (0.2-0.35m observed) is still the primary complaint on goals
  that DO succeed, `PathAlignCritic`/`PathFollowCritic` weight increases are the next lever (see
  `TUNING.md` §2) - not yet tried, since this round prioritized establishing ground truth over
  further blind tuning.
- Consider testing with a smaller `minimum_turning_radius` only if the steering PID
  (`urdf/qcar_model.xacro`, `TUNING.md` §1) is confirmed able to actually deliver a tighter turn
  reliably - artificially shrinking this without that confirmation would trade one failure mode
  for another.

## 2026-07-14 (11) - Fix accuracy: footprint-blind collision checking + verified critic weights + real footprint polygon

Follow-up to (10). User reported "still not accurate": path followed loosely (cutting corners),
final position off by more than `xy_goal_tolerance` even when reported "reached," and some goals
still failing after many retries. Rather than guess at critic `cost_weight` values again, fetched
the actual Humble source for every critic in use directly from GitHub
(`ros-navigation/navigation2`, `humble` branch,
`nav2_mppi_controller/src/critics/*.cpp`) to get real default values instead of estimating - this
surfaced a genuine bug, not just an undertuned default.

**Root cause found**: `CostCritic.consider_footprint` defaults to `false`. The (9)/(10) MPPI
config never set it (despite `PARAMS_REFERENCE.md` incorrectly documenting it as `true`), so
collision/obstacle cost was being evaluated at the robot's center point only, not its actual
elongated footprint - a direct, plausible cause of corner-cutting near obstacles for a car-shaped
robot. Enabling it exposed a second, older latent issue: `local_costmap`/`global_costmap` were
still using `robot_radius: 0.15` (a circular approximation, flagged as a known mismatch as far
back as the first `TUNING.md` draft but never fixed) - footprint-aware collision checking against
an undersized circle wouldn't actually protect the car's real front/rear overhangs. Fixed both
together, plus raised `GoalCritic.cost_weight` for the reported final-position inaccuracy.

### Changed
- **`config/nav2/nav2_params.yaml`**: `controller_server.FollowPath.CostCritic.consider_footprint`
  set to `true` (was silently `false`).
- **`config/nav2/nav2_params.yaml`**: `controller_server.FollowPath.GoalCritic.cost_weight` raised
  from the verified default `5.0` to `8.0` - strengthens convergence to the exact goal point
  within `GoalCritic`'s 1.4m `threshold_to_consider`, addressing the reported position inaccuracy.
- **`config/nav2/nav2_params.yaml`**: `local_costmap`/`global_costmap`'s `robot_radius: 0.15`
  replaced with a real polygon `footprint`, measured directly from `models/qcar/QCarBody.stl`
  (X -0.2006 to 0.2082, Y -0.0806 to 0.0754 - 0.409m long x 0.156m wide, rounded outward with a
  small margin): `[[0.22, 0.09], [0.22, -0.09], [-0.22, -0.09], [-0.22, 0.09]]`.
- **`TUNING.md`** / **`config/nav2/PARAMS_REFERENCE.md`**: updated with the verified real default
  `cost_weight` for every critic in use (`ConstraintCritic` 4.0, `CostCritic` 3.81, `GoalCritic`
  5.0, `GoalAngleCritic` 3.0, `PathAlignCritic` 10.0, `PathFollowCritic` 5.0, `PathAngleCritic`
  2.0, `PreferForwardCritic` 5.0), each critic's `threshold_to_consider`, and the footprint fix.
  The "Known mismatches" section marked resolved rather than removed, to preserve the history.

### Known limitation
- Not live-tested by Claude - same reasoning as (9)/(10). This round is grounded in verified
  source (not guessed defaults), which is a step up in confidence, but `GoalCritic.cost_weight:
  8.0` specifically is still an estimate of the *right* value, not something derived from
  evidence - if position accuracy is still off, raise it further; if the car now ignores the path
  in the final approach to beeline for the goal, it went too far. The "goal failed after many
  retries" symptom may or may not be resolved by this round - if it persists, get a fresh
  terminal log rather than assuming it's the same root cause as before.

## 2026-07-14 (10) - Fix MPPI not following the planned path: missing `enforce_path_inversion`

Follow-up to (9). User reported the robot wasn't following the planned path at all after the
MPPI switch. Rather than guess at critic weights, checked `nav2_mppi_controller`'s path-handling
header directly
(`/opt/ros/humble/include/nav2_mppi_controller/tools/path_handler.hpp`) and found
`enforce_path_inversion_{false}` - a dedicated mechanism for handling a path that contains a
cusp/reversal (exactly what `SmacPlannerHybrid`'s Reeds-Shepp K-turns produce), defaulting to
off. Without it, MPPI does not correctly treat the path as "drive to the cusp, then continue past
it" - this was left unset in the (9) config, which only set structural motion/velocity
parameters and left everything else at defaults. This is very likely the actual cause of "not
following the path" (a structural gap, not a critic-weight tuning issue), though not yet
confirmed via a live test with logs.

### Changed
- **`config/nav2/nav2_params.yaml`**: added `controller_server.FollowPath.enforce_path_inversion:
  true`, `inversion_xy_tolerance: 0.2`, `inversion_yaw_tolerance: 0.4` (the tolerance values are
  the plugin's own defaults, made explicit rather than left implicit).
- **`TUNING.md`** / **`config/nav2/PARAMS_REFERENCE.md`**: documented these three parameters as
  critical for this vehicle's reversing use case, and noted `PathAlignCritic`/`PathFollowCritic`
  weight tuning as the next lever if path-hugging is still imperfect after this fix (as opposed
  to structurally broken).

### Known limitation
- Not live-tested by Claude - same reasoning as (9), the user's own session was active. This is a
  narrower, more targeted fix than (9) (one specific missing parameter, found via source
  inspection rather than broad guessing), but still needs a real goal test to confirm the robot
  now tracks the planned path, especially through a cusp/reversal segment.

## 2026-07-14 (9) - Replace local controller with MPPI: RPP's cusp instability was never reliably tunable away

Follow-up to (8). User shared a fresh log after the cusp-stabilizing `FollowPath` tuning: goal 1
succeeded (after one retry, ~85s), but goal 2 - shorter distance, same ~180deg heading mismatch
pattern - failed completely after cycling through repeated aborts/recoveries for **7.6 minutes**
before `bt_navigator` gave up ("Goal failed"). This is not a fixed problem, it's an intermittent
one: the (8) mitigations (tighter lookahead, slower cusp approach, higher `reverse_penalty`)
reduce the failure rate but don't eliminate it, and an unreliable navigation stack (sometimes
takes 85s, sometimes fails after 7+ minutes) isn't a usable outcome.

Rather than continue tuning `RegulatedPurePursuitController` parameters against a failure mode
that's inherent to its algorithm (a single lookahead/"carrot" point tracked along the path, which
has no robust handling for a large lookahead radius straddling both sides of a cusp), swapped the
local controller entirely to `nav2_mppi_controller::MPPIController`. MPPI samples many candidate
trajectories each control cycle and scores them against cost critics, rather than tracking one
point on the path - it doesn't share RPP's specific failure mode, and (confirmed via
`/opt/ros/humble/include/nav2_mppi_controller/motion_models.hpp`) natively supports an
`AckermannMotionModel` with a real minimum-turning-radius hard constraint, which is a better
kinematic fit for this vehicle than RPP's curvature-based regulation ever was.

### Changed
- **`config/nav2/nav2_params.yaml`**: `controller_server.FollowPath.plugin` changed from
  `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` to
  `nav2_mppi_controller::MPPIController`. Configured with `motion_model: "Ackermann"` and
  `AckermannConstraints.min_turning_r: 0.5` (kept in sync with
  `planner_server.GridBased.minimum_turning_radius`), `vx_max/vx_min: 0.3/-0.2` (forward/reverse
  speed limits), `vy_max: 0.0` (non-holonomic), `wz_max: 1.9`, core sampling parameters
  (`batch_size: 2000`, `time_steps: 56`, `model_dt: 0.05`), and the standard critic set
  (`ConstraintCritic`, `CostCritic`, `GoalCritic`, `GoalAngleCritic`, `PathAlignCritic`,
  `PathFollowCritic`, `PathAngleCritic`, `PreferForwardCritic`). All the RPP-specific parameters
  from (8) (`lookahead_dist`, `regulated_linear_scaling_*`, etc.) are removed - they don't apply
  to MPPI.
- **`package.xml`**: added `nav2_mppi_controller` exec_depend.
- **`TUNING.md`** / **`config/nav2/PARAMS_REFERENCE.md`**: §2 (path following) rewritten for MPPI;
  §4 (planner) and the `progress_checker`/`FollowPath` sections updated to remove stale
  RPP-specific mitigation language now that the controller itself has changed.

### Verification performed (structural, not a live goal test)
Since local access to the critics' `.cpp` source isn't available (only headers), per-critic cost
weights were deliberately left at nav2's own built-in defaults rather than guessed at - only
structural/load-bearing parameters are set explicitly. Verified before handing off:
- `nav2_mppi_controller::MPPIController` matches the registered pluginlib class exactly
  (`/opt/ros/humble/share/nav2_mppi_controller/mppic.xml`).
- All 8 critic short names match `critics.xml`'s registered classes, and
  `critic_manager.hpp::getFullName()` confirms short names (e.g. `"ConstraintCritic"`) are
  expanded to the full `mppi::critics::X` form internally - the config doesn't need the
  namespaced form.
- The literal string `"Ackermann"` (capital A, matching `AckermannMotionModel`'s selection logic)
  is present in the compiled `.so`, and `AckermannConstraints.min_turning_r` matches the actual
  parameter path declared in `motion_models.hpp`.
- `nav2_params.yaml` parses as valid YAML with the expected structure (checked directly with
  Python's `yaml` module).

### Known limitation
- **Not live-tested by Claude at all** - the user has their own `qcar_nav2.launch.py` session
  running with a real display, so no test launch was attempted in this session to avoid
  disrupting it. This is the biggest structural change of this entire tuning session (new
  controller algorithm, not a parameter adjustment) and rests on header/pluginlib verification,
  not observed behavior. Needs a full relaunch and goal test, ideally repeating a goal similar to
  the one that previously failed after 7+ minutes, before trusting this is actually fixed.
- Per-critic cost weights are untuned (nav2 defaults). If paths look reasonable but the car
  behaves oddly in some specific way (e.g. too cautious, cuts corners, prefers reverse too much),
  the fix is very likely a specific critic's `cost_weight`, not the structural parameters above -
  see `TUNING.md` §2 for which critic governs which behavior.

## 2026-07-14 (8) - Re-enable reversing with cusp-stabilizing `FollowPath` tuning

Follow-up to (7). User rejected the forward-only Dubins workaround: reversing is a real
requirement for this vehicle, not optional. Before reverting blindly, checked
`RegulatedPurePursuitController`'s header
(`/opt/ros/humble/include/nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp`)
directly - it does have cusp-detection (`findVelocitySignChange`, "checks for the cusp position...
robot distance from the cusp"), used internally to regulate speed near a cusp. So this isn't a
complete blind spot in the controller, just evidently not enough margin at the lookahead/speed
settings that were in use when the instability was observed. Re-enabled reversing paired with a
meaningfully more conservative configuration, rather than just flipping the same settings back.

### Changed
- **`config/nav2/nav2_params.yaml`**: `planner_server.GridBased.motion_model_for_search` back to
  `"REEDS_SHEPP"`; `reverse_penalty` raised `2.0` -> `4.0` (prefer forward-only routes, only
  reverse when the goal orientation truly requires it - fewer cusps encountered in practice).
- **`config/nav2/nav2_params.yaml`**: `controller_server.FollowPath.allow_reversing` back to
  `true`.
- **`config/nav2/nav2_params.yaml`**: `FollowPath.lookahead_dist` / `min_lookahead_dist` /
  `max_lookahead_dist` reduced `0.6/0.3/0.9` -> `0.35/0.15/0.5` - a smaller carrot-point search
  radius is less likely to straddle both sides of a cusp and flip direction as the robot's exact
  position fluctuates.
- **`config/nav2/nav2_params.yaml`**: `FollowPath.regulated_linear_scaling_min_radius` /
  `_min_speed` changed `0.9/0.25` -> `1.2/0.1` - slows the car down harder and earlier approaching
  tight curvature (a cusp is effectively infinite curvature), giving RPP's internal
  `findVelocitySignChange` cusp regulation more time/control authority to settle the direction
  decision before the robot drifts enough to flip it again.
- **`TUNING.md`** / **`config/nav2/PARAMS_REFERENCE.md`**: updated to describe `"REEDS_SHEPP"` +
  the new conservative `FollowPath` values, with the full back-and-forth history preserved so a
  future session understands why these specific values were chosen (not just their current state).

### Known limitation
- **This is a mitigation, not a guaranteed fix.** The underlying cusp-following behavior in this
  Humble nav2 version's `RegulatedPurePursuitController` is still what it is - these changes
  reduce the risk (slower, tighter-lookahead approach gives the existing cusp regulation more
  margin; higher `reverse_penalty` means fewer cusps are planned at all) without eliminating the
  architectural gap. Not independently re-verified by Claude - needs a real goal test, ideally
  with the user watching the sim directly again during any cusp/reversal moment, not just the
  logs. If oscillation returns, `TUNING.md` §4 has the next levers to try (`reverse_penalty`
  further, then `desired_linear_vel`) before falling back to `"DUBIN"` again.

## 2026-07-14 (7) - Switch planner to forward-only Dubins paths: `RegulatedPurePursuitController` can't handle cusps in this nav2 version

Follow-up to (6). The rate-slowing fix did not help either: user shared a fresh log showing
`Failed to make progress` still firing roughly every ~30s (matching `movement_time_allowance`
almost exactly) even at the slower 5s replan rate - ruling out replanning frequency as the actual
bottleneck (a much less frequently changing path still didn't let the robot accumulate 10cm of
net movement in 25-30 seconds). Asked the user to watch the sim directly during a stall window
rather than the logs: confirmed the robot was actively wiggling (forward/back) the whole time,
not frozen - ruling out a collision-detection stall and confirming a genuine path-following
instability.

Conclusion: `SmacPlannerHybrid` with `motion_model_for_search: "REEDS_SHEPP"` was correctly
producing K-turn paths with a reverse segment for large final-heading mismatches, but
`RegulatedPurePursuitController` in this Humble nav2 version doesn't have robust handling for
path cusps (a direction-reversal point) - the lookahead/carrot-point search becomes unstable
right at the cusp, causing indefinite oscillation with no net progress. This is a controller-level
limitation, not something fixable via replanning rate, `progress_checker` patience, or
`yaw_goal_tolerance` - all three of those were legitimate fixes for their own specific symptoms
((3), (6), and the reverted (4) respectively) but none of them address an unstable cusp.

### Changed
- **`config/nav2/nav2_params.yaml`**: `planner_server.GridBased.motion_model_for_search` changed
  from `"REEDS_SHEPP"` to `"DUBIN"` - forward-only path search, no cusps. The car will now take a
  wider forward loop instead of a tight reverse K-turn when a large final-heading change is
  needed, trading path efficiency for stability.
- **`config/nav2/nav2_params.yaml`**: `controller_server.FollowPath.allow_reversing` changed from
  `true` back to `false`, to match the forward-only planner - a forward-only plan with a
  controller still free to independently decide to reverse was part of what caused the
  instability.
- **`TUNING.md`** / **`config/nav2/PARAMS_REFERENCE.md`**: updated the global-planner sections to
  describe `"DUBIN"` instead of `"REEDS_SHEPP"`, and to note `reverse_penalty` is now unused.

### Known limitation
- Not independently re-verified by Claude - based on the user's shared logs and direct
  observation of the sim, not a fresh live test in this session. Needs a relaunch and a goal test
  to confirm the oscillation is actually gone and that the resulting forward-loop paths are
  acceptable in practice (not, e.g., looping through a space that's too tight for the wider
  maneuver in some parts of the map).
- If Reeds-Shepp/reversing is ever revisited (e.g. after a nav2 upgrade with better cusp
  handling), re-enable both `motion_model_for_search: "REEDS_SHEPP"` and `allow_reversing: true`
  together and retest live - don't re-enable only one.

## 2026-07-14 (6) - Slow replanning again now that `progress_checker` won't misinterpret the wait

Follow-up to (5). User shared a fresh terminal log after the Hybrid-A* switch: "improved
slightly" but still oscillating. The log showed `[controller_server]: Passing new path to
controller.` firing roughly once per second, continuously - confirming the `RateController` is
still at the stock `1.0` hz - and `Failed to make progress` firing again almost exactly 30s after
the prior one (matching `movement_time_allowance: 30.0` from (3) precisely, confirming that fix
is working as configured). The robot covered under 0.1m of net displacement across that entire
30-second window: not slow progress, genuinely stuck. Conclusion: even with SmacPlannerHybrid
now producing a correct K-turn path, a fresh replan arriving roughly every second interrupts that
maneuver before it can ever complete, forcing it to restart repeatedly.

This is the same fix attempted once before in (2) and reverted - but (2) was reverted because it
interacted badly with the *old*, impatient `progress_checker` (10s default allowance, 0.5m
required), where each abort then had to wait up to 5s for the next scheduled path, turning quick
retries into multi-minute stalls. `progress_checker` is patient now (3), so that interaction
should no longer apply - reapplying the rate change on that basis.

### Changed
- **`config/nav2/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`**: `RateController
  hz` lowered from `1.0` to `0.2` again (replan every 5s).
- **`config/nav2/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml`**:
  `RateController hz` lowered from `0.333` to `0.15` again (replan roughly every 7s), for
  consistency.
- Both files' header comments updated with the full back-and-forth history (tried in (2), reverted,
  now reapplied for a different, verified-fixed reason) so a future session doesn't re-revert this
  blindly without checking whether the (3) `progress_checker` fix is still in place first.

### Known limitation
- Not independently re-verified by Claude - based on the user's shared log and the same
  reasoning already validated for (2)'s original attempt, not a fresh live test in this session.
  If oscillation still persists after this, the next thing to check is whether SmacPlannerHybrid's
  plan is actually consistent between consecutive replans (compare two consecutive `path` messages
  on `/plan` a few seconds apart during the goal-approach phase) - if the planned K-turn geometry
  itself is changing meaningfully call to call (not just timing), slowing the rate further won't
  help and the search parameters (`reverse_penalty`, `retrospective_penalty`,
  `angle_quantization_bins`) may need adjustment instead.

## 2026-07-14 (5) - Real fix for accurate final heading: switch global planner to Hybrid-A*

Follow-up to (4) - user rejected the "loosen yaw_goal_tolerance to accept any angle" workaround:
final orientation genuinely matters for their use case, "irrespective of any angle... goal
completed" is not correct behavior. That workaround was masking the problem, not fixing it, so
reverted it and addressed the actual architectural cause instead (see (1)-(4) above for the full
diagnosis trail): `NavfnPlanner` has no concept of heading at all, so `FollowPath` was always
handed a path whose arrival tangent could be arbitrarily different from the requested goal
orientation, leaving `RegulatedPurePursuitController` to improvise a correction reactively -
that reactive improvisation, not any single tunable parameter, was the real source of the
oscillation. Tuning `progress_checker`, replanning rate, or `yaw_goal_tolerance` could only ever
mask or reduce the symptom, not fix the mismatch between what the planner produces and what the
controller is being asked to achieve.

### Changed
- **`config/nav2/nav2_params.yaml`**: `planner_server.GridBased.plugin` switched from
  `nav2_navfn_planner/NavfnPlanner` to `nav2_smac_planner/SmacPlannerHybrid` - kinematically
  aware of this car's real minimum turning radius, and with
  `motion_model_for_search: "REEDS_SHEPP"` can plan an explicit reverse/K-turn segment into the
  path itself when the goal orientation requires one (matching `allow_reversing: true` already
  set in `FollowPath`), instead of leaving the controller to improvise. Configured with
  `minimum_turning_radius: 0.5` (padded over the car's true ~0.445m physical minimum - wheelbase
  0.25725m / tan(30deg max steer)); full parameter list and rationale in `TUNING.md` §4 and
  `config/nav2/PARAMS_REFERENCE.md`.
- **`config/nav2/nav2_params.yaml`**: `general_goal_checker.yaw_goal_tolerance` reverted from the
  (4) workaround value of `2.9` rad back down to `0.3` rad - a real, meaningful tolerance, now
  achievable without oscillation because the path itself should arrive close to the correct
  heading rather than needing a large post-hoc correction.
- **`package.xml`**: added `nav2_smac_planner` exec_depend.
- **`TUNING.md`** / **`config/nav2/PARAMS_REFERENCE.md`**: updated the global-planner sections to
  describe `SmacPlannerHybrid` and its parameters instead of the retired `NavfnPlanner`.

### Known limitation
- Not independently re-verified by Claude - this is a bigger structural change than the previous
  parameter-only tuning attempts in this session, and has not yet been tested against a real
  goal. `max_planning_time: 5.0`s and `angle_quantization_bins: 72` are reasonable starting
  values for this map's small size, not empirically tuned - watch planner latency and path
  quality on the first few real tests. If `0.3` rad `yaw_goal_tolerance` turns out too tight
  (oscillation returns) or too loose (imprecise stops), that's the first knob to revisit before
  touching the planner config again.

## 2026-07-14 (4) - Sidestep goal-approach oscillation: loosen `yaw_goal_tolerance` drastically

Follow-up to (3) - the `progress_checker` fix reduced abort-driven stalling but the user reported
the robot still oscillates trying to correct final heading. Asked the user whether precise final
heading actually matters for their use case; they confirmed only goal *position* matters, heading
is fine "roughly." Given that, the pragmatic fix is to stop the goal checker from ever demanding
a tight heading match, rather than continuing to chase RegulatedPurePursuitController's inherent
difficulty converging a large final-heading mismatch without an explicit K-turn plan from the
global planner (NavfnPlanner doesn't produce one - see the (1)/(2)/(3) entries above and
`TUNING.md` SS4 for the architectural root of this: the real fix if heading precision is ever
needed is `nav2_smac_planner`'s Hybrid-A*, a bigger change than parameter tuning can achieve).

### Changed
- **`config/nav2/nav2_params.yaml`**: `controller_server.general_goal_checker.yaw_goal_tolerance`
  raised from `1.0` to `2.9` rad (~166 deg) - accepts essentially any arrival heading except one
  almost exactly reversed from the requested orientation, so the oscillating correction maneuver
  should now rarely/never trigger at all. Added an inline comment pointing back to this rationale
  and to `TUNING.md` SS4 for the proper fix if heading precision becomes a requirement later.

### Known limitation
- Not independently re-verified by Claude - based on the user's own test goal, confirmed
  requirement (position-only), and the accumulated diagnosis from the (1)-(3) entries above, not
  a fresh live test in this session. Needs a relaunch and one more goal test to confirm the
  oscillation is actually gone, not just less frequent.

## 2026-07-14 (3) - Real fix for goal-approach oscillation: `progress_checker` was aborting mid-maneuver

Follow-up to the entry below (2). The replanning-rate slowdown did not fix the oscillation - user
shared the live launch terminal output and it showed the true cause: repeated
`[controller_server]: Failed to make progress` / `[follow_path] [ActionServer] Aborting handle`
cycling for 3+ minutes through `wait`/`backup` recovery behaviors. Checked
`nav2_controller::SimpleProgressChecker`'s header
(`/opt/ros/humble/include/nav2_controller/plugins/simple_progress_checker.hpp`) directly: this
Humble-era version only stores a single `radius_` member and judges progress by **linear
displacement alone** - `required_angular_distance` (already set in this file) is not implemented
until a later nav2 release and is silently ignored here. During a heading-correction maneuver
near the goal (especially with `allow_reversing: true`, which does small forward/reverse
micro-moves), net linear displacement stays under the old `required_linear_distance: 0.5` within
the default ~10s `movement_time_allowance`, so the checker judged the robot "stuck" and aborted
the maneuver before it could complete - repeatedly, since every retry hit the same wall.

### Changed
- **`config/nav2/nav2_params.yaml`**: `controller_server.progress_checker.required_linear_distance`
  lowered from `0.5` to `0.1`; added `movement_time_allowance: 30.0` explicitly (previously
  relying on the plugin's default, undocumented in this file). Added an inline comment
  explaining the `required_angular_distance` no-op gotcha for future reference.
- **Reverted the (2) replanning-rate change** in both
  `config/nav2/behavior_trees/navigate_{to_pose,through_poses}_w_replanning_and_recovery.xml` -
  `RateController hz` back to the stock `1.0`/`0.333`. That slowdown wasn't the fix and made the
  actual problem worse: each "Failed to make progress" abort/retry now had to wait up to 5s for
  the next scheduled path before attempting to move again, stretching what should be a quick
  retry into part of that observed 3+ minute stall. `<Spin>` remains removed from recovery in
  both files (that part was correct and unrelated to this issue).

### Known limitation
- Not independently re-verified by Claude - the user is testing directly via their own running
  `qcar_nav2.launch.py` session and sharing terminal output; this fix is based on that shared log
  plus direct inspection of the `SimpleProgressChecker` header, not a fresh live test in this
  session. Needs a relaunch to pick up the new params and one more goal test to confirm.

## 2026-07-14 (2) - Reduce replanning rate to fix repeated oscillation with `allow_reversing: true`

Follow-up to the entry below. User set `allow_reversing: true` in `nav2_params.yaml` (their own
change, expected to let the controller use a reverse maneuver to correct final heading in tight
spaces) and reported the robot now oscillates *multiple times* - forward swings, then reverse
attempts - before finally settling, worse than before. Root cause: `allow_reversing` didn't
remove the actual problem (see the entry below - `bt_navigator`'s `RateController` still
recomputes the global path every 1s, continuously, even while the robot is stationary and just
correcting its heading near the goal). It just gave the controller a second degree of freedom
(forward vs. reverse) to flip-flop on each time a fresh 1Hz replan nudged the path's implied
approach angle, instead of committing to one clean correction.

### Changed
- **`config/nav2/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`**: `RateController
  hz` lowered from `1.0` to `0.2` (replan every 5s instead of every 1s).
- **`config/nav2/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml`**: `RateController
  hz` lowered from `0.333` to `0.15` (replan roughly every 7s instead of every 3s), for
  consistency with the above.

Deliberately a rate change only, not a restructuring of the replan-trigger logic - see the
"Tried and reverted" note in the entry below for why a conditional (`IsPathValid`-based)
restructuring is considered too risky to reattempt right now given a real bug already found in
that approach in this nav2 version.

### Known limitation
- Not re-verified live in this session - the user has their own `qcar_nav2.launch.py` instance
  running with a real display/GUI while this change was made, so it was rebuilt but not
  relaunched to avoid disrupting their active session. Needs a restart of that launch to pick up
  the new behavior tree files, and a fresh goal test to confirm the oscillation is actually
  reduced.

## 2026-07-14 - Fix goal-approach orientation-correction wobble, remove infeasible `<Spin>` recovery

User reported: robot reaches the goal *position* but then swings away from it and back while
correcting its final *orientation* - not a smooth in-place-feeling correction. Root cause:
`qcar_nav2.launch.py` never set a custom behavior tree, so `bt_navigator` used its compiled-in
default (`navigate_to_pose_w_replanning_and_recovery.xml`), which replans the entire global path
every second, continuously, for the whole navigation - including after the robot has already
reached the goal position but before its heading satisfies `yaw_goal_tolerance`. Since
`NavfnPlanner` has no concept of the robot's current heading, and `use_rotate_to_heading: false`
(correctly, since this Ackermann car can't rotate in place), each fresh 1Hz replan near the goal
could demand a different arrival heading, forcing the car to swing out and back repeatedly to
chase it.

### Changed
- **`config/nav2/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`** /
  **`navigate_through_poses_w_replanning_and_recovery.xml`** (new): local copies of the stock
  nav2-shipped default trees, identical except `<Spin>` removed from `RecoveryActions` - a pure
  in-place rotation is physically impossible for this car (steering has no effect at zero linear
  velocity), so it was a guaranteed no-op recovery attempt that just burned its timeout each time
  it came up in the round-robin.
- **`launch/qcar_nav2.launch.py`**: wraps `nav2_params.yaml` in `RewrittenYaml` to set
  `default_nav_to_pose_bt_xml` / `default_nav_through_poses_bt_xml` to the two new local files.
- **`config/nav2/nav2_params.yaml`**: added the placeholder `default_nav_to_pose_bt_xml: ""` /
  `default_nav_through_poses_bt_xml: ""` keys under `bt_navigator` (required for `RewrittenYaml`
  to have something to overwrite - it only rewrites existing keys, doesn't add new ones). Raised
  `general_goal_checker.yaw_goal_tolerance` from `0.7` to `1.0` rad, to reduce how often a
  correction maneuver triggers at all.
- **`package.xml`**: added `nav2_common` (imported directly by `qcar_nav2.launch.py` for
  `RewrittenYaml`; was previously only a transitive dependency via `nav2_bringup`).

### Tried and reverted (documented in case this is revisited)
- Attempted swapping to nav2's stock "replan only if the path becomes invalid" alternate tree
  (`navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml`) instead of just
  removing `<Spin>`, to eliminate the continuous 1Hz replanning at its source rather than just
  tolerating its effect via a looser yaw tolerance. **This has a real bug** in this nav2/Humble
  version: `IsPathValid` returns SUCCESS on the default-constructed/empty `{path}` blackboard
  entry before `ComputePathToPose` has ever run, so the `Fallback` wrapping them skips
  `ComputePathToPose` entirely and `FollowPath` receives an empty path forever - confirmed via a
  live headless test (`Resulting plan has 0 poses in it`, `Controller patience exceeded`
  repeating until all recovery retries were exhausted, `Goal failed`). Reverted to the stock
  continuous-replanning structure (proven working end-to-end in this package) with only `<Spin>`
  removed.

### Known limitation
- The final BT + `yaw_goal_tolerance` combination above was not re-verified live end-to-end after
  the revert - the local test sandbox degraded after many repeated heavy launches in one session
  (stale FastRTPS shared-memory segments, DDS daemon losing track of nodes) to the point of
  timing out on basic `ros2` CLI commands, unrelated to the config itself. Verify with a real
  `ros2 launch qcar_updated qcar_nav2.launch.py` + goal send before trusting this fully.

## 2026-07-13 - Migrate from `ackermann_steering_controller` to `libgazebo_ros_ackermann_drive`, fix "not moving" / "not reaching goal"

Replaced the `ros2_control`-based `ackermann_steering_controller` (and its `controller_manager`
spawners and `topic_tools relay` bridges) with the `gazebo_ros_ackermann_drive` Gazebo plugin,
declared directly in `urdf/qcar_model.xacro`'s `<gazebo>` block - same approach already used by
the sibling package `qcar_navigation`. The plugin subscribes to `/cmd_vel` and publishes `/odom`
and the `odom -> base` TF natively, so no relay nodes or controller manager are needed.

### Changed
- **`urdf/qcar_model.xacro`**: removed the `<ros2_control>` block and its `libgazebo_ros2_control`
  plugin; added the `gazebo_ros_ackermann_drive` plugin block (front/rear/steering joint mapping,
  `max_steer`/`max_speed`, PID gains, odometry/TF publishing flags). Replaced each wheel's
  `<collision>` (previously the visual STL mesh) with a `<cylinder radius="0.036"
  length="0.0245"/>` - the plugin's wheel-radius auto-detection only supports cylinder/sphere
  collisions, and a mesh collision silently reads `wheel_radius_=0`, poisoning the velocity PID
  entirely (this is why the robot didn't move at all after the initial migration). Added
  `<mu1>1.0</mu1><mu2>1.0</mu2>` friction to all 4 wheel links (previously zero friction
  anywhere, causing 100% wheel slip with zero real motion even once the radius bug was fixed).
  Bumped wheel rotational inertia from `ixx=izz=0.0000645, iyy=0.0001089` to
  `ixx=izz=0.0012, iyy=0.0022` - the original tiny inertia caused an intermittent
  `Ogre::AxisAlignedBox::setExtents` assertion crash in gzserver under normal steering PID load.
- **`worlds/myworld.world`**: fixed the ground plane's `<friction><ode><mu>100</mu><mu2>50</mu2>`
  down to `mu=1 mu2=1` - the original absurd value, combined with a now-working wheel contact,
  caused a physics blow-up.
- **`left/right_steering_pid_gain`**: initially set to `0.02 0 0.002` (copied from
  `qcar_navigation`'s equivalent fix), but that value was tuned for a robot with zero wheel
  friction. Once wheel friction was added (above), turning the front hubs against a gripping,
  rolling tire needed real torque that `kp=0.02` couldn't supply - measured via direct `tf2`
  lookups on `base -> hubfl`/`hubfr` that the hub only reached ~1 deg of an ~15 deg commanded
  steering angle. This silently made the car's real turning radius ~6x larger than commanded,
  causing Nav2's path-tracking controller to be unable to follow any planned turn accurately.
  Raised to `3.0 0 0.1`, re-verified to converge to the correct Ackermann inner/outer wheel angle
  split for a given commanded turning radius.
- **`launch/qcar_updated.launch.py`**: removed the `joint_state_broadcaster`/
  `ackermann_steering_controller` spawner nodes and the `odom_relay`/`odom_tf_relay`
  `topic_tools relay` nodes (no longer needed - the plugin publishes `/odom` and `odom -> base`
  directly). `robot_description` is now built via a `Command(['xacro ', ...])` substitution
  instead of `xacro.process_file(...).toxml()` at launch-description-generation time.
- **`launch/qcar_nav2.launch.py`** / **`launch/qcar_slam.launch.py`**: removed the `cmd_vel_relay`
  `topic_tools relay` node (the plugin subscribes to `/cmd_vel` directly, no bridging needed).
- **`package.xml`**: removed `controller_manager`, `joint_state_broadcaster`,
  `ackermann_steering_controller`, `topic_tools`, `gazebo_ros2_control`; added `gazebo_plugins`.

### Removed
- `config/qcar_controllers.yaml`, `config/qcar_controllers_slam_override.yaml` - `ros2_control`
  controller manager configs, no longer used.

### Known limitations
- **`disable_odom_tf` launch argument is currently a no-op.** It's still declared in
  `qcar_updated.launch.py` and passed through to `xacro`, and `qcar_slam.launch.py` still passes
  `disable_odom_tf:=true` expecting it to suppress the drive plugin's own `odom -> base`
  broadcast during SLAM (so Cartographer is the sole broadcaster). But
  `urdf/qcar_model.xacro` never reads a `disable_odom_tf` property anywhere, and the plugin's
  `publish_odom_tf` is hardcoded `true` - so in SLAM mode both Cartographer and the plugin now
  publish `odom -> base`, fighting over the same TF edge. Needs an `xacro:arg`-gated
  `publish_odom_tf` value if SLAM mode is exercised again.
- **`/joint_states` is self-referential and always reports zero.** `joint_state_publisher` in
  `qcar_updated.launch.py` is configured with `source_list: ['/joint_states']` - its own output
  topic - so it just republishes its own default zero state rather than real wheel angles. Doesn't
  affect physics or Nav2 (the plugin publishes real wheel/steering TF separately via
  `publish_wheel_tf`), but any tooling that reads `/joint_states` directly for wheel angles will
  see stale zeros; use TF (`base -> wheelXX`/`hubXX`) instead.
- End-to-end verified via a real Nav2 run (headless gzserver + AMCL + planner + controller,
  goal requiring both straight driving and a turn) - `bt_navigator` logged `Reached the goal!` /
  `Goal succeeded`. See `TUNING.md` for further accuracy-tuning parameters if goal-reaching
  precision still isn't satisfactory for a given map/goal.

## 2026-07-05 - Migrate from JointGroup controllers to `ackermann_steering_controller`

Replaced the hand-rolled drive stack (two independent `ros2_control` controllers plus a
manual bicycle-model conversion script) with `ackermann_steering_controller`, which does
Ackermann inverse kinematics and wheel odometry natively.

### Changed
- **Controller manager config** (`config/qcar_controllers.yaml`): removed `drive_controller`
  (`velocity_controllers/JointGroupVelocityController`) and `steering_controller`
  (`position_controllers/JointGroupPositionController`); added a single
  `ackermann_steering_controller` (`ackermann_steering_controller/AckermannSteeringController`)
  configured with `front_steering: true`, the measured wheelbase/track/radius values, and
  `enable_odom_tf: true` so it publishes its own `odom -> base` transform.
- **`launch/qcar_updated.launch.py`**: replaced the `drive_controller`/`steering_controller`
  spawner nodes with one `ackermann_steering_controller` spawner; added a `disable_odom_tf`
  launch argument (default `false`) that, when true, spawns the controller with an extra
  `-p config/qcar_controllers_slam_override.yaml` so it doesn't broadcast `odom -> base` (used
  during SLAM, see below); added two `topic_tools relay` nodes to bridge the controller's private
  topics onto the topics the rest of the stack expects: `~/odometry -> /odom` and
  `~/tf_odometry -> /tf` (this controller version publishes its TF to a private topic instead of
  directly to `/tf`); added `--ros-args --log-level ackermann_steering_controller:=error` to the
  `gazebo` process to silence a benign per-message deprecation warning from the unstamped
  `Twist` reference topic (the node exposes no runtime logger service in Humble, so this has to
  be set at process launch).
- **`launch/qcar_nav2.launch.py`** / **`launch/qcar_slam.launch.py`**: replaced the
  `cmd_vel_to_drive.py` node with a `topic_tools relay` from `/cmd_vel` to
  `/ackermann_steering_controller/reference_unstamped`. `qcar_slam.launch.py` now includes
  `qcar_updated.launch.py` with `disable_odom_tf:=true`, since Cartographer
  (`provide_odom_frame: true` in `qcar_2d.lua`) must be the sole `odom -> base` broadcaster.
- **`urdf/qcar_model.xacro`**: removed the `gazebo_ros_p3d` ground-truth odometry plugin
  (`/odom` is now the controller's real wheel odometry, not a perfect ground-truth feed). Fixed
  a latent bug on `base_wheelrl_joint`: its `<origin>` is yawed 180 deg for mesh mirroring but it
  shared the same `<axis>` as `base_wheelrl_joint`'s counterpart, silently inverting its
  effective rotation sense; the old `JointGroupVelocityController` setup masked this by
  negating one side's command in `cmd_vel_to_drive.py`, but a real per-joint controller needs it
  fixed at the source, so the axis was flipped to `0 -1 0` instead.
- **`package.xml`**: removed `velocity_controllers`, `position_controllers`, `std_msgs`,
  `nav_msgs`, `tf2_ros` (nothing left depends on them); added `ackermann_steering_controller`
  and `topic_tools`.
- **`CMakeLists.txt`**: removed the deleted scripts from `install(PROGRAMS ...)`.

### Removed
- `scripts/cmd_vel_to_drive.py` - superseded by the controller's native `Twist` reference input
  (via the `cmd_vel_relay` topic relay).
- `scripts/odom_to_tf.py` - superseded by the controller's own `odom -> base` TF broadcast (via
  the `odom_tf_relay` topic relay).
- `scripts/qcar_teleop.py` - published directly to `/drive_controller/commands` and
  `/steering_controller/commands`, both retired by this migration. `scripts/qcar_teleop_twist.py`
  (publishes plain `Twist` to `/cmd_vel`) already supersedes it and needs no changes.

### Fixed
- **Launch argument name collision**: the first attempt at the `disable_odom_tf` argument above
  was named `slam`, which collides with `nav2_bringup`'s own `slam` launch argument (used
  internally by `bringup_launch.py` for its own localization-vs-SLAM branching). Because
  `qcar_nav2.launch.py` includes `qcar_updated.launch.py` before including nav2's bringup, the
  lowercase `default_value='false'` won the shared launch-context lookup, and nav2's
  `PythonExpression(['not ', slam])` then evaluated `eval("not false")` - a `NameError`, since
  Python needs the capitalized `False`. This crashed the whole launch and took the nav2
  component container down with it (`exit code -6`). Renamed the argument to `disable_odom_tf`.

### Known limitations
- Wheel odometry now has real (if a bit high) dead-reckoning drift during turns, most likely
  from unmodeled wheel/ground slip - `urdf/qcar_model.xacro` has no `<gazebo><surface><friction>`
  tuning on the wheel collisions at all (bare ODE defaults). AMCL tolerates some odometry error
  by design (it corrects against the map via laser scan matching), but this may need attention
  if navigation accuracy is unsatisfactory. The controller's own odometry math was verified
  correct against the upstream `ros2_controllers` source for this version.
