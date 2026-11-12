#pragma once
#include "clearing_robot_plugins/planner_types.hpp"
#include "clearing_robot_plugins/scene.hpp"
namespace clearing_robot_plugins
{
struct BodySearchOptions
{
  double min_radius{2.5}, step{.65}, timeout{7.}, clearance{0.};
  double reverse_weight{1.08}, switch_cost{4.}, steering_change_cost{2.};
  int max_expansions{12000};
  bool allow_spin{false};
};
int motionDirection(const Pose &, const Pose &);
bool kinematicPath(const std::vector<Pose> &, double min_radius);
std::vector<Pose> connectBodyPath(const Pose &, const Pose &, const Validator &,
                                  const BodySearchOptions &);
std::vector<Pose> searchBodyPath(const Scene &, const Pose &, const Pose &,
                                 const std::shared_ptr<const LiveData> &, bool dynamic,
                                 const BodySearchOptions &,
                                 const std::function<bool()> &cancel = {});
} // namespace clearing_robot_plugins
