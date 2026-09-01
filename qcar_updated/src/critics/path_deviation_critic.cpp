#include "qcar_updated/critics/path_deviation_critic.hpp"

#include <algorithm>
#include <cmath>

#include <xtensor/xview.hpp>

namespace mppi::critics
{

void PathDeviationCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(deviation_threshold_, "deviation_threshold", 0.3);
  getParam(lookahead_offset_, "lookahead_offset", 15);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 10.0);

  RCLCPP_INFO(
    logger_,
    "PathDeviationCritic instantiated with %u power, %f weight, deviation_threshold %f, "
    "lookahead_offset %zu",
    power_, weight_, deviation_threshold_, lookahead_offset_);
}

void PathDeviationCritic::score(CriticData & data)
{
  if (!enabled_ || data.path.x.shape(0) < 2) {
    return;
  }

  utils::setPathFurthestPointIfNotSet(data);
  const size_t path_size = data.path.x.shape(0) - 1;
  const size_t idx = std::min(*data.furthest_reached_path_point, path_size);

  // How far the robot's actual current pose (not a sampled trajectory) sits from the path point
  // it has already progressed to - the same "furthest reached" index nav2's own path-following
  // critics use, so this stays consistent with what they consider "on track."
  const double px = data.path.x(idx);
  const double py = data.path.y(idx);
  const double rx = data.state.pose.pose.position.x;
  const double ry = data.state.pose.pose.position.y;
  const double deviation = std::hypot(rx - px, ry - py);

  if (deviation < deviation_threshold_) {
    return;
  }

  const size_t target_idx = std::min(idx + lookahead_offset_, path_size);
  const float target_x = data.path.x(target_idx);
  const float target_y = data.path.y(target_idx);

  const auto last_x = xt::view(data.trajectories.x, xt::all(), -1);
  const auto last_y = xt::view(data.trajectories.y, xt::all(), -1);
  const auto dx = last_x - target_x;
  const auto dy = last_y - target_y;
  const auto dists = xt::sqrt(dx * dx + dy * dy);

  data.costs += xt::pow(dists * weight_, power_);
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PathDeviationCritic,
  mppi::critics::CriticFunction)
