#include "clearing_robot_plugins/hybrid_local_planner.hpp"
#include "clearing_robot_plugins/planner_utils.hpp"
#include <algorithm>
#include <limits>
#include <pluginlib/class_list_macros.hpp>
#include <queue>
#include <unordered_map>
namespace clearing_robot_plugins
{
using detail::distance;
using detail::expired;
using detail::message;
using detail::positions;
void HybridLocalPlanner::configure(const Node::WeakPtr &p, std::string name)
{
  node_ = p;
  auto n = p.lock();
  scene_ = Scene::create(p);
  cached_local_.clear();
  body_options_.min_radius = parameter(n, "motion.planning_turn_radius", 3.5);
  horizon_ = parameter(n, name + ".lookahead", 12.0);
  timeout_ = parameter(n, name + ".timeout", 1.5);
  expansions_ = parameter(n, name + ".max_expansions", 5000);
  clearance_ = parameter(n, name + ".planning_clearance", .35);
  body_options_.clearance = clearance_;
  body_options_.timeout = timeout_;
  body_options_.max_expansions = expansions_;
  if (!std::isfinite(clearance_) || clearance_ < 0)
    throw std::runtime_error("Invalid local planning clearance");
}
void HybridLocalPlanner::setGlobalPlan(const Path &path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  global_ = path;
}
Path HybridLocalPlanner::computeLocalPath(const geometry_msgs::msg::PoseStamped &current)
{
  auto n = node_.lock();
  Path source;
  std::vector<Pose> previous;
  Pose previous_goal;
  bool previous_reaches_goal;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    source = global_;
    previous = cached_local_;
    previous_goal = cached_goal_;
    previous_reaches_goal = cached_reaches_goal_;
  }
  if (!active_ || !n || source.poses.empty())
  {
    throw std::runtime_error("Local planner has no route");
  }
  if (!scene_->healthy())
  {
    RCLCPP_WARN_THROTTLE(n->get_logger(), *n->get_clock(), 2000, "Navigation hold: %s",
                         scene_->healthReason().c_str());
    // Health is an independent motion gate in the follower and governor.
    // Keep an unchanged certified spatial route through a temporary outage;
    // replacing it with repeated one-pose actions can strand the follower in
    // its hold episode even after sensor recovery.
    if (previous.size() > 1 && distance(previous_goal, fromMsg(source.poses.back().pose)) < .05 &&
        std::abs(wrap(previous_goal.yaw - fromMsg(source.poses.back().pose).yaw)) < .05)
    {
      ends_at_goal_ = previous_reaches_goal;
      return message(previous, cached_stamp_);
    }
    ends_at_goal_ = false;
    return message({fromMsg(current.pose)}, n->now());
  }
  auto reference = positions(source);
  Pose start = fromMsg(current.pose);
  auto live = scene_->snapshot();
  auto remember = [&](const std::vector<Pose> &path, bool reaches_goal, bool current_prefix = false)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // A measured start pose is only an output connector. Caching it again
      // would accumulate a trail of previous measurements at each update.
      cached_local_.assign(path.begin() + (current_prefix ? 1 : 0), path.end());
      cached_goal_ = reference.back();
      cached_reaches_goal_ = reaches_goal;
      cached_stamp_ = n->now();
    }
    ends_at_goal_ = reaches_goal;
    return message(path, n->now());
  };
  if (!previous.empty() && distance(previous_goal, reference.back()) < .05 &&
      std::abs(wrap(previous_goal.yaw - reference.back().yaw)) < .05)
  {
    size_t closest = 0;
    for (size_t i = 1; i < previous.size(); ++i)
      if (distance(start, previous[i]) < distance(start, previous[closest]))
        closest = i;
    // Preserve a nearby unfinished pivot even if a later moving pose is
    // spatially closer. Orientation is part of path progress for this body.
    for (size_t i = 0; i < closest && i + 1 < previous.size(); ++i)
      if (distance(previous[i], previous[i + 1]) < 1e-6 &&
          std::abs(wrap(previous[i + 1].yaw - previous[i].yaw)) > .12 &&
          distance(start, previous[i]) < 1. &&
          std::abs(wrap(start.yaw - previous[i + 1].yaw)) >= .12)
      {
        closest = i;
        break;
      }
    const double body_error =
        distance(start, previous[closest]) +
        scene_->model.radius * std::abs(wrap(start.yaw - previous[closest].yaw));
    if (body_error < .35 && (previous_reaches_goal || distance(start, previous.back()) > 2.))
    {
      if (closest + 1 < previous.size())
      {
        const Eigen::Vector2d ahead(previous[closest + 1].x - previous[closest].x,
                                    previous[closest + 1].y - previous[closest].y);
        const Eigen::Vector2d behind(previous[closest].x - start.x, previous[closest].y - start.y);
        if (ahead.norm() > .03 && behind.dot(ahead) < 0)
          ++closest;
      }
      std::vector<Pose> remaining{start};
      remaining.insert(remaining.end(), previous.begin() + closest, previous.end());
      bool clear = true;
      const bool start_buffer = scene_->safe(start, live, true, clearance_);
      const bool end_buffer = scene_->safe(remaining.back(), live, true, clearance_);
      for (size_t i = 1; i < remaining.size(); ++i)
      {
        double factor = 1.;
        if (!start_buffer)
          factor = std::min(factor, std::clamp(distance(remaining[i - 1], start) / 1.5, 0., 1.));
        if (!end_buffer)
          factor =
              std::min(factor, std::clamp(distance(remaining[i], remaining.back()) / 1.5, 0., 1.));
        if (!scene_->swept(remaining[i - 1], remaining[i], live, true, clearance_ * factor))
        {
          clear = false;
          break;
        }
      }
      if (clear)
      {
        ends_at_goal_ = previous_reaches_goal;
        // Keep the action's stamp stable while the certified route is valid.
        // The follower projects progress itself; periodic replans must not
        // cancel a moving action just to crop already travelled vertices.
        return message(previous, cached_stamp_);
      }
    }
  }
  size_t nearest = 0;
  double best = INFINITY;
  for (size_t i = 0; i < reference.size(); ++i)
  {
    double d = distance(start, reference[i]);
    if (d < best)
    {
      best = d;
      nearest = i;
    }
  }
  std::vector<Pose> segment{start};
  size_t terminal = nearest;
  double length = 0;
  for (size_t i = nearest; i < reference.size(); ++i)
  {
    length += distance(segment.back(), reference[i]);
    segment.push_back(reference[i]);
    terminal = i;
    if (length >= horizon_)
    {
      break;
    }
  }
  // A long arm can intersect an obstacle even when the horizon's center is
  // free. Move the intermediate goal forward to a body-clear reference pose.
  while (!scene_->safe(segment.back(), live) && terminal + 1 < reference.size() &&
         length < horizon_ + 6.)
  {
    ++terminal;
    length += distance(segment.back(), reference[terminal]);
    segment.push_back(reference[terminal]);
  }
  double tail = 0;
  for (size_t i = terminal + 1; i < reference.size(); ++i)
    tail += distance(reference[i - 1], reference[i]);
  // Finish at the requested terminal orientation when the remaining tail is
  // shorter than the vehicle. Stopping a long arm just before the goal can
  // introduce a much tighter, unnecessary intermediate pivot.
  if (tail < std::min(6., scene_->model.radius) && scene_->safe(reference.back(), live))
    for (size_t i = terminal + 1; i < reference.size(); ++i)
      segment.push_back(reference[i]);
  ends_at_goal_ = distance(segment.back(), reference.back()) < .05;
  Pose target = segment.back();
  if (!scene_->safe(target, live))
    throw std::runtime_error("Hybrid A*: local target's full body is occupied");
  const bool start_buffer = scene_->safe(start, live, true, clearance_);
  const bool target_buffer = scene_->safe(target, live, true, clearance_);
  auto local_valid = [&](const Pose &a, const Pose &b)
  {
    // Reserve tracking room along the corridor, tapering it at the exact
    // start/goal. Extra clearance is a planning preference; it must not reject
    // an otherwise collision-free requested goal near a static surface.
    const double angle = wrap(b.yaw - a.yaw);
    const int steps =
        std::max(1, int(std::ceil((distance(a, b) + scene_->model.radius * std::abs(angle)) /
                                  scene_->collision_resolution)));
    for (int i = 0; i <= steps; ++i)
    {
      const double fraction = double(i) / steps;
      Pose next = a;
      next.x += fraction * (b.x - a.x);
      next.y += fraction * (b.y - a.y);
      next.z += fraction * (b.z - a.z);
      next.yaw = wrap(a.yaw + fraction * angle);
      double factor = 1.;
      if (!start_buffer)
        factor = std::min(factor, std::clamp(distance(next, start) / 1.5, 0., 1.));
      if (!target_buffer)
        factor = std::min(factor, std::clamp(distance(next, target) / 1.5, 0., 1.));
      const double padding = clearance_ * factor;
      if (!scene_->safe(next, live, true, padding))
        return false;
    }
    return true;
  };
  bool clear = true;
  for (size_t i = 1; i < segment.size(); ++i)
  {
    if (!local_valid(segment[i - 1], segment[i]))
    {
      clear = false;
      break;
    }
  }
  if (clear && kinematicPath(segment, body_options_.min_radius))
  {
    return remember(segment, ends_at_goal_);
  }
  auto route = searchBodyPath(*scene_, start, target, live, true, body_options_,
                              [&] { return isCancelRequested(); });
  if (route.empty())
    throw std::runtime_error(
        "No full-body forward/reverse path within curvature and search limits");
  return remember(route, ends_at_goal_);
}

} // namespace clearing_robot_plugins
PLUGINLIB_EXPORT_CLASS(clearing_robot_plugins::HybridLocalPlanner,
                       multimap_nav_core::LocalPlannerBase)
