// Custom BT condition node for qcar_updated.
//
// User-observed 2026-08-26: stock IsPathValid (nav2_behavior_tree) declares the whole path
// invalid, triggering a full replan, the instant a SINGLE tick's check against the live local
// costmap fails - no debounce. On real hardware, a single frame of LiDAR noise, a transient TF
// jitter, or a momentary self-detection artifact can flip one path pose to "invalid" for exactly
// one tick even though the path is still genuinely fine, forcing an unnecessary replan (and, per
// today's investigation, an unnecessary replan from a mid-maneuver pose is what produces the
// messiest, highest-cusp-count plans).
//
// This is otherwise an exact copy of stock IsPathValidCondition's service-call logic (same
// /is_path_valid service, same request/response types - verified against
// nav2_behavior_tree/plugins/condition/is_path_valid_condition.cpp on GitHub, humble branch)
// with one addition: only reports the path invalid (BT FAILURE, triggering a real replan) after
// `consecutive_failures_required` ticks in a row report invalid - a single valid response resets
// the counter to 0 immediately. Since this whole check is already rate-limited to 1Hz by the
// RateController wrapping it in the BT XML, consecutive_failures_required=2 means a transient
// single-frame blip is fully absorbed, while a real, persistent obstacle is still caught within
// ~2 seconds - not meaningfully slower than stock for the case that actually matters.
#ifndef QCAR_UPDATED__BT__IS_PATH_VALID_DEBOUNCED_CONDITION_HPP_
#define QCAR_UPDATED__BT__IS_PATH_VALID_DEBOUNCED_CONDITION_HPP_

#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"

namespace qcar_updated::bt
{

class IsPathValidDebouncedCondition : public BT::ConditionNode
{
public:
  IsPathValidDebouncedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsPathValidDebouncedCondition() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Path to Check"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout"),
      BT::InputPort<int>(
        "consecutive_failures_required", 2,
        "Number of consecutive invalid checks required before actually reporting invalid"),
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<nav2_msgs::srv::IsPathValid>::SharedPtr client_;
  std::chrono::milliseconds server_timeout_;
  int consecutive_invalid_count_{0};
};

}  // namespace qcar_updated::bt

#endif  // QCAR_UPDATED__BT__IS_PATH_VALID_DEBOUNCED_CONDITION_HPP_
