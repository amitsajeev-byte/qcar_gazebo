// Custom nav2_core::Smoother plugin for qcar_updated.
//
// A front-steered vehicle's steered wheels are the leading (self-correcting) axle going
// forward, but the trailing (jackknife-prone) axle in reverse - curving immediately from a
// standing start in reverse is the hardest sub-case of a K-turn. Real drivers back up roughly
// straight first, then ease into the turn. This plugin does that as path post-processing: at
// each REVERSE-entering cusp in a planned path (forward-entering cusps and all forward segments
// are left untouched), it re-projects the next `straight_distance` of arc length onto a straight
// line continuing the cusp's own heading, blending back to the original curve over that same
// distance (weight 1.0 at the cusp -> 0.0 at the blend end, not a hard cutover).
//
// `extend_distance`: the cusp point itself is also pushed `extend_distance` further, continuing
// the incoming leg's own curvature rather than cutting straight - so the incoming and outgoing
// legs overlap/cross instead of meeting at one precise vertex, and the vehicle keeps curving
// through the extension the way it was already curving on approach instead of snapping straight
// right at the corner. The curvature is estimated over a ~0.15m lookback window of the incoming
// path (averaging the yaw change across several planner segments), not just the single adjacent
// segment - a two-point estimate is dominated by the planner's coarse angular quantization
// (angle_quantization_bins: 72, 5deg steps), which makes the extension curve far more sharply
// than the incoming path actually does (found and fixed 2026-08-29, see CHANGELOG.md). Degrades
// to a straight-line push-out when the incoming leg was already straight (curvature ~ 0). MPPI's
// cusp-arrival gating (isWithinInversionTolerances(), path_handler.cpp) needs to converge on that
// vertex exactly before revealing the path beyond it, and widening the geometric target eases
// that.
//
// `min_lead_distance`: a replanned path starts at the robot's live pose, so the SAME physical
// K-turn cusp keeps reappearing near the front of the path on every replan while the robot
// approaches/executes it (nav2 replans roughly every second while active). Only apply the
// extend+straighten treatment when the cusp is still at least this far ahead of the robot's
// current position - once the robot is essentially already at/into the cusp, it's not an
// upcoming maneuver to prepare for anymore, and re-extending it would just reshape the path
// under the robot on every replan tick instead of a one-time approach adjustment.
//
// A max_cusps rejection cap (reject and force a retry above N cusps) was tried and reverted
// 2026-08-26 - not the cause of a same-day hardware issue, but reverted alongside
// IsPathValidDebounced to get back to the last fully-validated config before video capture. See
// CHANGELOG.md 2026-08-26.
//
// Full derivation/trial history (v1 hard-cutover regression, a multi-cusp corruption bug,
// extend_distance trial results): CHANGELOG.md 2026-08-25/26 and the
// qcar_updated_mppi_cusp_freeze_investigation memory.
#ifndef QCAR_UPDATED__SMOOTHERS__CUSP_STRAIGHTENER_SMOOTHER_HPP_
#define QCAR_UPDATED__SMOOTHERS__CUSP_STRAIGHTENER_SMOOTHER_HPP_

#include <memory>
#include <string>

#include "nav2_core/smoother.hpp"

namespace qcar_updated::smoothers
{

class CuspStraightenerSmoother : public nav2_core::Smoother
{
public:
  CuspStraightenerSmoother() = default;
  ~CuspStraightenerSmoother() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
    std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub) override;

  void cleanup() override {}
  void activate() override {}
  void deactivate() override {}

  bool smooth(nav_msgs::msg::Path & path, const rclcpp::Duration & max_time) override;

protected:
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("CuspStraightenerSmoother")};
  double straight_distance_{0.21};
  double extend_distance_{0.15};
  double min_lead_distance_{0.3};
};

}  // namespace qcar_updated::smoothers

#endif  // QCAR_UPDATED__SMOOTHERS__CUSP_STRAIGHTENER_SMOOTHER_HPP_
