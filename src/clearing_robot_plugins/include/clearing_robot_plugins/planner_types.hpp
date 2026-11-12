#pragma once
#include "clearing_robot_plugins/model.hpp"
#include <functional>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
namespace clearing_robot_plugins
{
using Node = rclcpp_lifecycle::LifecycleNode;
using Path = nav_msgs::msg::Path;
using Validator = std::function<bool(const Pose &, const Pose &)>;
} // namespace clearing_robot_plugins
