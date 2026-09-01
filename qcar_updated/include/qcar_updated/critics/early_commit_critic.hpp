// Custom MPPI critic for qcar_updated.
//
// For a goal whose planned path curves immediately from the robot's current position, MPPI
// repeatedly failed to commit to the turn - nav2's own PathAngleCritic gives no heading guidance
// for moderate initial direction mismatches (gated by max_angle_to_furthest, and scores average
// heading error across the whole rollout rather than the near-term approach). EarlyCommitCritic
// scores only the first `early_time_steps` of each sampled trajectory against the bearing to a
// near-term path point, ungated by threshold_to_consider/max_angle_to_furthest, so it always
// pushes MPPI to start turning toward the path immediately.
//
// Gated on path progress (active_path_points): this critic's near-term bearing target is only
// meaningful right at the start of a path and fights the normal path/goal critics once real
// progress has been made, so scoring stops once furthest_reached_path_point passes
// active_path_points.
//
// Direction-aware: scores against whichever of (bearing, bearing + pi) is closer rather than
// assuming forward travel, matching nav2's own utils::posePointAngle() behavior
// (forward_preference=false) - otherwise this critic scores a correct Reeds-Shepp reverse segment
// as maximally wrong and fights the exact K-turn maneuver needed.
//
// Lives in namespace mppi::critics, not qcar_updated::critics: nav2_mppi_controller's
// CriticManager::getFullName() hardcodes the "mppi::critics::" prefix when resolving names from
// the critics: [...] list, so a plugin in any other namespace is unreachable regardless of how
// it's exported via pluginlib.
//
// Full investigation history: CHANGELOG.md 2026-07-18/19.
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
