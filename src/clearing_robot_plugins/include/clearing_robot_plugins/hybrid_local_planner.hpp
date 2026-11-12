#pragma once
#include "clearing_robot_plugins/body_path.hpp"
#include "clearing_robot_plugins/planner_types.hpp"
#include "clearing_robot_plugins/scene.hpp"
#include <multimap_nav_core/local_planner_base.hpp>
namespace clearing_robot_plugins
{
class HybridLocalPlanner : public multimap_nav_core::LocalPlannerBase
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
  void setGlobalPlan(const Path &) override;
  Path computeLocalPath(const geometry_msgs::msg::PoseStamped &) override;
  bool lastPlanReachesGoal() const override { return ends_at_goal_; }

private:
  Node::WeakPtr node_;
  std::shared_ptr<Scene> scene_;
  std::mutex mutex_;
  Path global_;
  std::vector<Pose> cached_local_;
  Pose cached_goal_;
  bool cached_reaches_goal_{false};
  bool active_{false}, ends_at_goal_{false};
  double horizon_{12.0}, timeout_{1.5};
  double clearance_{.4};
  int expansions_{5000};
  BodySearchOptions body_options_;
  rclcpp::Time cached_stamp_{0, 0, RCL_ROS_TIME};
};
} // namespace clearing_robot_plugins
