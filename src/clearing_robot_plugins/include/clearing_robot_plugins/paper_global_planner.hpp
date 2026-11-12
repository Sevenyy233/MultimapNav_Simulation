#pragma once
#include "clearing_robot_plugins/body_path.hpp"
#include "clearing_robot_plugins/planner_types.hpp"
#include "clearing_robot_plugins/scene.hpp"
#include <multimap_nav_core/global_planner_base.hpp>
namespace clearing_robot_plugins
{
std::vector<Pose> paperPlan(const TerrainMap &, Pose, Pose, const Validator &,
                            const std::function<bool()> &cancel = {}, double max_seconds = 3.0);
std::vector<Pose> filterNodes(const std::vector<Pose> &, const Validator &);
std::vector<Pose> bezierPath(const std::vector<Pose> &, const Validator &, double resolution = .2);

class PaperGlobalPlanner : public multimap_nav_core::GlobalPlannerBase
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
  Path createPlan(const geometry_msgs::msg::PoseStamped &,
                  const geometry_msgs::msg::PoseStamped &) override;

protected:
  bool validateSmoothedPath(const Path &) const override;

private:
  Node::WeakPtr node_;
  std::shared_ptr<Scene> scene_;
  bool active_{false};
  double timeout_{3.0};
  BodySearchOptions body_options_;
  std::mutex cache_mutex_;
  std::vector<Pose> cached_path_;
};
} // namespace clearing_robot_plugins
