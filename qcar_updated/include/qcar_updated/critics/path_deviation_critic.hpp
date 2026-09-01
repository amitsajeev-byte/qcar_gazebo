// Custom MPPI critic for qcar_updated.
//
// User-proposed correction (deferred 2026-08-26, implemented same day once time allowed): when
// the robot deviates from the planned path during a reverse/corner maneuver, nav2's own
// PathAlignCritic/PathFollowCritic/PathAngleCritic all pull every sampled trajectory back toward
// the NEAREST path point - forcing an exact rejoin. If deviation is large, that costs time/heading
// budget the robot may not have left before the goal. This critic instead pulls trajectories
// toward a point `lookahead_offset` further along the path (a "keep making progress" target,
// not "snap back exactly") - but ONLY once the robot is more than `deviation_threshold` away from
// the path, so it stays completely inactive (zero cost contribution) while normal path-following
// is working, exactly like every other trial run today.
//
// Deliberately does not touch PathAlignCritic's own weight/behavior - this is a purely additive
// critic (same as EarlyCommitCritic's pattern) so all of today's already-validated on-path
// behavior is unaffected; it only ever competes with the path-align pull once genuinely deviated.
#ifndef QCAR_UPDATED__CRITICS__PATH_DEVIATION_CRITIC_HPP_
#define QCAR_UPDATED__CRITICS__PATH_DEVIATION_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

class PathDeviationCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  double deviation_threshold_{0.3};
  size_t lookahead_offset_{15};

  unsigned int power_{1};
  float weight_{0};
};

}  // namespace mppi::critics

#endif  // QCAR_UPDATED__CRITICS__PATH_DEVIATION_CRITIC_HPP_
