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
// Gated on real distance to the near-term target (max_lead_distance), not path-point count:
// originally gated by active_path_points (a fixed count of path points from trip start), but
// CHANGELOG.md 2026-07-19 (5) found and only partially fixed a real bug with that approach - a
// path that runs straight for a while before curving could still have its curve fall inside the
// point-count window, so this critic kept reaching toward that not-yet-current curve well before
// the robot had actually traveled there ("robot turns before the curve", live-measured up to 3x
// the planned lateral offset; lowering the window 15->8 reduced but did not eliminate it).
// Root-caused further 2026-09-02: a path-point count is a poor proxy for physical distance -
// replaced with a direct Euclidean distance check against the same near-term target this critic
// already scores against, mirroring cusp_straightener_smoother's already-proven
// min_lead_distance pattern. This one mechanism naturally covers both cases this critic needs to
// handle: at genuine trip start with an immediately-curving path, distance-to-target is ~0, so it
// fires right away (same as before); further down a path where the near-term target is still
// physically far off, it stays suppressed until the robot actually closes that distance - fixing
// the 2026-07-19 (5) residual without needing two separate critics or an arbitrary point count.
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
  float max_lead_distance_{0.3f};
  bool forward_preference_{false};

  unsigned int power_{1};
  float weight_{0};
};

}  // namespace mppi::critics

#endif  // QCAR_UPDATED__CRITICS__EARLY_COMMIT_CRITIC_HPP_
