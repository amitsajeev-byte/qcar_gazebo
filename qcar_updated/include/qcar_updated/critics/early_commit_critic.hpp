// Custom MPPI critic for qcar_updated.
//
// Why this exists (see CHANGELOG.md 2026-07-18 (9) for the full investigation): for a goal whose
// planned path curves immediately from the robot's current position, MPPI repeatedly failed to
// commit to the turn - commanded steering decayed back toward 0deg within ~15-20s of a goal
// being sent, and stayed there, leaving the robot essentially stationary. Live-testing ruled out
// ConstraintCritic's turning-radius penalty as the sole cause (disabling it did not help) and
// showed nav2's own PathAngleCritic provides no heading guidance at all for moderate initial
// direction mismatches, since it's gated by max_angle_to_furthest (default ~69 degrees) and only
// scores the *average* heading error across the whole rollout, not the near-term approach.
//
// EarlyCommitCritic scores only the first `early_time_steps` of each sampled trajectory against
// the bearing to a near-term path point, with no threshold_to_consider/max_angle_to_furthest
// gating - so it always pushes MPPI to start turning toward the path immediately, rather than
// leaving that decision to critics that are inactive or too diffuse this early in a trajectory.
//
// It IS gated on path progress though (active_path_points): an early live test with no such gate
// fixed the "stuck at trip start" case but broke ordinary path-following for the rest of the
// trip - this critic's near-term bearing target is only a meaningful signal right at the start of
// a path, and actively fights the normal path/goal critics once the robot has made real
// progress. Scoring stops once furthest_reached_path_point passes active_path_points.
//
// Direction-aware as of CHANGELOG.md 2026-07-19: originally always compared trajectory yaw to the
// bearing TOWARD the near-term target, which is only correct for a forward approach. Live-testing
// a goal requiring a sharp mid-trip reorientation found the robot freezing again, further into
// the trip than the original bug - SmacPlannerHybrid's replanned path from that position was a
// Reeds-Shepp K-turn (a short reverse segment before curving forward into the goal, confirmed via
// dumping /plan), and this critic was scoring that correct reverse segment as maximally wrong
// (heading ~180deg from the "forward" bearing it assumed), fighting the exact maneuver needed and
// pinning the batch back into a near-zero-velocity optimum. Fixed the same way nav2's own
// utils::posePointAngle() handles it (see nav2_mppi_controller/tools/utils.hpp): when
// forward_preference is false, score against whichever of (bearing, bearing + pi) is closer,
// rather than assuming forward travel - letting other critics (ConstraintCritic, PathFollow, etc)
// decide whether forward or reverse is actually cheaper.
//
// Lives in namespace mppi::critics, not qcar_updated::critics: nav2_mppi_controller's
// CriticManager::getFullName() hardcodes the "mppi::critics::" prefix when resolving names from
// the critics: [...] list (see nav2_mppi_controller/src/critic_manager.cpp), so a plugin in any
// other namespace is simply unreachable from the critics list, regardless of how it's exported
// via pluginlib.
#ifndef QCAR_UPDATED__CRITICS__EARLY_COMMIT_CRITIC_HPP_
#define QCAR_UPDATED__CRITICS__EARLY_COMMIT_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

class EarlyCommitCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  size_t offset_from_furthest_{3};
  size_t early_time_steps_{10};
  size_t active_path_points_{15};
  bool forward_preference_{false};

  unsigned int power_{1};
  float weight_{0};
};

}  // namespace mppi::critics

#endif  // QCAR_UPDATED__CRITICS__EARLY_COMMIT_CRITIC_HPP_
