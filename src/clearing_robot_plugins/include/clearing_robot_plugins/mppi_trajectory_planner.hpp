#pragma once
#include "clearing_robot_plugins/motion_limits.hpp"
#include "clearing_robot_plugins/planner_types.hpp"
#include "clearing_robot_plugins/scene.hpp"
#include <atomic>
#include <multimap_nav_core/trajectory_planner.hpp>
#include <multimap_nav_utils/control/jerk_limited_axis.hpp>
#include <random>
#include <std_msgs/msg/string.hpp>
namespace clearing_robot_plugins
{
class MPPITrajectory : public multimap_nav_core::TrajectoryPlannerBase
{
public:
  void configure(const Node::WeakPtr &, std::string) override;
  void activate() override { active_ = true; }
  void deactivate() override { active_ = false; }
  void cleanup() override { scene_.reset(); }
  void shutdown() override
  {
    deactivate();
    cleanup();
  }
  void setLocalPlan(const Path &) override;
  void setLocalPlan(const Path &, bool) override;
  ControlResult computeControl(const geometry_msgs::msg::PoseStamped &,
                               const geometry_msgs::msg::Twist &) override;
  geometry_msgs::msg::TwistStamped
  computeVelocityCommands(const geometry_msgs::msg::PoseStamped &,
                          const geometry_msgs::msg::Twist &) override;
  multimap_nav_msgs::msg::Trajectory computeTrajectory(const geometry_msgs::msg::PoseStamped &,
                                                       const geometry_msgs::msg::Twist &) override;
  bool isGoalReached() override { return goal_reached_; }

private:
  struct Command
  {
    double v{0}, w{0};
  };
  Node::WeakPtr node_;
  std::shared_ptr<Scene> scene_;
  std::mutex mutex_;
  Path path_;
  bool active_{false}, ends_at_goal_{true};
  std::atomic<bool> goal_reached_{false};
  int samples_{48}, steps_{30};
  double dt_{.2}, frequency_{8}, max_v_{.45}, max_w_{.18};
  double max_a_{.25}, max_aw_{.15}, jerk_{.6}, angular_jerk_{.5}, temperature_{1.0};
  std::vector<Command> warm_;
  MotionLimits limits_;
  bool guard_seen_{false};double guard_v_{0},guard_w_{0},guard_a_{0},guard_aw_{0},guard_stamp_{-1};
  uint64_t guard_epoch_{0},control_epoch_{0};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr guard_sub_;
  double last_control_time_{-1};
  std::mt19937 random_{20261027};
  multimap_nav_utils::JerkLimitedAxis linear_, angular_;
};
} // namespace clearing_robot_plugins
