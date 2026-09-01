#include "qcar_updated/bt/is_path_valid_debounced_condition.hpp"

#include <chrono>
#include <memory>
#include <string>

namespace qcar_updated::bt
{

IsPathValidDebouncedCondition::IsPathValidDebouncedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = node_->create_client<nav2_msgs::srv::IsPathValid>("is_path_valid");

  server_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
  getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
}

BT::NodeStatus IsPathValidDebouncedCondition::tick()
{
  nav_msgs::msg::Path path;
  getInput("path", path);

  int required = 2;
  getInput("consecutive_failures_required", required);

  auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();
  request->path = path;
  auto result = client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, result, server_timeout_) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    if (result.get()->is_valid) {
      consecutive_invalid_count_ = 0;
      return BT::NodeStatus::SUCCESS;
    }

    ++consecutive_invalid_count_;
    if (consecutive_invalid_count_ >= required) {
      RCLCPP_INFO(
        node_->get_logger(),
        "IsPathValidDebounced: confirmed invalid after %d/%d consecutive checks - replanning now",
        consecutive_invalid_count_, required);
      consecutive_invalid_count_ = 0;
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "IsPathValidDebounced: reported invalid (%d/%d consecutive) - riding it out, "
      "not replanning yet",
      consecutive_invalid_count_, required);
    return BT::NodeStatus::SUCCESS;
  }

  // Service call itself failed/timed out - fail safe, same as stock IsPathValid.
  return BT::NodeStatus::FAILURE;
}

}  // namespace qcar_updated::bt

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<qcar_updated::bt::IsPathValidDebouncedCondition>("IsPathValidDebounced");
}
