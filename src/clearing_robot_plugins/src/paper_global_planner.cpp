#include "clearing_robot_plugins/paper_global_planner.hpp"
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
std::vector<Pose> paperPlan(const TerrainMap &map, Pose start, Pose goal, const Validator &valid,
                            const std::function<bool()> &cancel, double seconds)
{
  int s = map.index(start.x, start.y), g = map.index(goal.x, goal.y),
      size = map.width * map.height_cells;
  if (s < 0 || g < 0)
  {
    return {};
  }
  struct Front
  {
    std::vector<double> cost;
    std::vector<int> parent;
    std::vector<uint8_t> closed;
    std::vector<int> open;
    int current;
  };
  auto make = [&](int seed)
  {
    Front f;
    f.cost.assign(size, INFINITY);
    f.parent.assign(size, -1);
    f.closed.assign(size, 0);
    f.cost[seed] = 0;
    f.open.push_back(seed);
    f.current = seed;
    return f;
  };
  Front a = make(s), b = make(g);
  int meet = -1;
  auto beginning = std::chrono::steady_clock::now();
  auto cell = [&](int k)
  {
    Pose p;
    auto xy = map.center(k);
    p.x = xy.x();
    p.y = xy.y();
    p.z = map.elevations[k] + .01;
    return p;
  };
  auto expand = [&](Front &f, Front &other, bool reverse)
  {
    int best = -1;
    double value = INFINITY;
    auto anchor = cell(other.current);
    for (size_t i = 0; i < f.open.size(); ++i)
    {
      auto p = cell(f.open[i]);
      double h = distance(p, anchor);
      // Paper equations (3),(5)-(8). Saturation only avoids overflow; this is
      // weighted search and does not claim an admissible/optimal heuristic.
      double score = f.cost[f.open[i]] + std::exp(std::min(60., h)) * h;
      if (score < value)
      {
        value = score;
        best = int(i);
      }
    }
    if (best < 0)
    {
      return;
    }
    int u = f.open[best];
    f.open.erase(f.open.begin() + best);
    f.closed[u] = 1;
    f.current = u;
    if (std::isfinite(other.cost[u]))
    {
      meet = u;
      return;
    }
    for (int dy = -1; dy <= 1; ++dy)
    {
      for (int dx = -1; dx <= 1; ++dx)
      {
        if (dx == 0 && dy == 0)
        {
          continue;
        }
        int x = u % map.width + dx, y = u / map.width + dy;
        if (x < 0 || y < 0 || x >= map.width || y >= map.height_cells)
        {
          continue;
        }
        int v = y * map.width + x;
        if (f.closed[v] || !std::isfinite(map.elevations[v]))
        {
          continue;
        }
        auto p = cell(u), q = cell(v);
        double heading = std::atan2(q.y - p.y, q.x - p.x) + (reverse ? M_PI : 0.);
        p.yaw = q.yaw = wrap(heading);
        if (!valid(reverse ? q : p, reverse ? p : q))
        {
          continue;
        }
        // Equation (4): 3D slope distance, not planar distance alone.
        double cost = f.cost[u] + distance(p, q);
        if (cost + 1e-9 < f.cost[v])
        {
          if (!std::isfinite(f.cost[v]))
          {
            f.open.push_back(v);
          }
          f.cost[v] = cost;
          f.parent[v] = u;
        }
      }
    }
  };
  while (!a.open.empty() && !b.open.empty() && meet < 0)
  {
    if ((cancel && cancel()) || expired(beginning, seconds))
    {
      return {};
    }
    expand(a, b, false);
    if (meet < 0)
    {
      expand(b, a, true);
    }
  }
  if (meet < 0)
  {
    return {};
  }
  std::vector<int> keys;
  for (int u = meet; u != -1; u = a.parent[u])
  {
    keys.push_back(u);
  }
  std::reverse(keys.begin(), keys.end());
  for (int u = b.parent[meet]; u != -1; u = b.parent[u])
  {
    keys.push_back(u);
  }
  std::vector<Pose> out;
  out.push_back(start);
  // Replace the endpoint cell centres with the exact poses. Appending both
  // creates sub-cell connectors whose arbitrary direction can turn a six-metre
  // arm through a large angle immediately before the goal.
  for (size_t i = 1; i + 1 < keys.size(); ++i)
  {
    auto p = cell(keys[i]);
    if (distance(out.back(), p) > .05)
    {
      out.push_back(p);
    }
  }
  if (distance(out.back(), goal) > .01)
  {
    out.push_back(goal);
  }
  for (size_t i = 0; i + 1 < out.size(); ++i)
  {
    out[i].yaw = std::atan2(out[i + 1].y - out[i].y, out[i + 1].x - out[i].x);
  }
  out.front().yaw = start.yaw;
  out.back().yaw = goal.yaw;
  for (size_t i = 1; i < out.size(); ++i)
  {
    if (!valid(out[i - 1], out[i]))
    {
      // Eight-neighbour A* has no heading state. Its reconstructed corner
      // headings can be infeasible for a long body even in an otherwise clear
      // corridor. Try a forward straight connection with checked rotations at
      // BOTH ends; never return the invalid grid headings as an executable path.
      Pose departure = start, arrival = goal;
      departure.yaw = arrival.yaw = std::atan2(goal.y - start.y, goal.x - start.x);
      if (valid(start, departure) && valid(departure, arrival) && valid(arrival, goal))
      {
        return {start, departure, arrival, goal};
      }
      return {};
    }
  }
  return out;
}
std::vector<Pose> filterNodes(const std::vector<Pose> &path, const Validator &valid)
{
  if (path.size() < 3)
  {
    return path;
  }
  std::vector<Pose> out{path.front()};
  size_t i = 0;
  while (i + 1 < path.size())
  {
    size_t far = i + 1;
    for (size_t j = i + 2; j < path.size(); ++j)
    {
      auto a = out.back(), b = path[j];
      double heading = std::atan2(b.y - a.y, b.x - a.x);
      a.yaw = out.back().yaw;
      b.yaw = j + 1 == path.size() ? path.back().yaw : heading;
      if (!valid(a, b))
      {
        break;
      }
      far = j;
    }
    Pose next = path[far];
    if (far + 1 < path.size())
    {
      next.yaw = std::atan2(path[far + 1].y - next.y, path[far + 1].x - next.x);
    }
    out.push_back(next);
    i = far;
  }
  return out;
}
std::vector<Pose> bezierPath(const std::vector<Pose> &path, const Validator &valid,
                             double resolution)
{
  if (path.size() < 3)
  {
    return path;
  }
  std::vector<Pose> out{path.front()};
  auto vector = [](const Pose &p) { return Eigen::Vector3d(p.x, p.y, p.z); };
  for (size_t i = 1; i + 1 < path.size(); ++i)
  {
    auto a = vector(path[i - 1]), b = vector(path[i]), c = vector(path[i + 1]);
    // Paper section 2.4: midpoints and eighth-points define cubic control points.
    Eigen::Vector3d p0 = (a + b) / 2, p1 = b + (a - b) / 8, p2 = b + (c - b) / 8, p3 = (b + c) / 2;
    int count = std::max(3, int(std::ceil(((b - a).norm() + (c - b).norm()) / resolution)));
    for (int j = 0; j <= count; ++j)
    {
      double t = double(j) / count, u = 1 - t;
      Eigen::Vector3d point =
          u * u * u * p0 + 3 * u * u * t * p1 + 3 * u * t * t * p2 + t * t * t * p3;
      Eigen::Vector3d tangent =
          3 * u * u * (p1 - p0) + 6 * u * t * (p2 - p1) + 3 * t * t * (p3 - p2);
      Pose p;
      p.x = point.x();
      p.y = point.y();
      p.z = point.z();
      p.yaw = std::atan2(tangent.y(), tangent.x());
      if (!valid(out.back(), p))
      {
        return path;
      }
      out.push_back(p);
    }
  }
  if (!valid(out.back(), path.back()))
  {
    return path;
  }
  out.push_back(path.back());
  return out;
}
void PaperGlobalPlanner::configure(const Node::WeakPtr &p, std::string name)
{
  node_ = p;
  auto n = p.lock();
  scene_ = Scene::create(p);
  timeout_ = parameter(n, name + ".timeout", 3.0);
  cached_path_.clear();
  body_options_.min_radius = parameter(n, "motion.planning_turn_radius", 3.5);
  body_options_.timeout = parameter(n, name + ".body_search_timeout", 7.0);
}
Path PaperGlobalPlanner::createPlan(const geometry_msgs::msg::PoseStamped &a,
                                    const geometry_msgs::msg::PoseStamped &b)
{
  auto n = node_.lock();
  if (!active_ || !n)
  {
    throw std::runtime_error("Global planner inactive");
  }
  Pose start = fromMsg(a.pose), goal = fromMsg(b.pose);
  if (!scene_->map->project(start, scene_->max_pitch, scene_->max_roll, scene_->max_step) ||
      !scene_->map->project(goal, scene_->max_pitch, scene_->max_roll, scene_->max_step))
  {
    throw std::runtime_error("Goal/start lack traversable support");
  }
  auto snapshot = scene_->snapshot();
  Validator valid = [&](const Pose &x, const Pose &y)
  { return scene_->swept(x, y, snapshot, true); };
  if (distance(start, goal) < .25 && std::abs(wrap(goal.yaw - start.yaw)) < .03 &&
      valid(start, goal))
  {
    return message({start, goal}, n->now());
  }
  if (!scene_->safe(start, snapshot, true))
    throw std::runtime_error("Start full body intersects static geometry");
  if (!scene_->safe(goal, snapshot, true))
    throw std::runtime_error(
        "Goal full body intersects static geometry; choose another pose/orientation");
  bool body_path = true;
  auto raw = connectBodyPath(start, goal, valid, body_options_);
  if (raw.empty())
  {
    raw = paperPlan(
        *scene_->map, start, goal, valid, [&] { return isCancelRequested(); }, timeout_);
    body_path = false;
    if (!raw.empty())
    {
      raw = bezierPath(filterNodes(raw, valid), valid);
      if (!kinematicPath(raw, body_options_.min_radius))
        raw.clear();
    }
    if (raw.empty())
    {
      raw = searchBodyPath(*scene_, start, goal, snapshot, true, body_options_,
                           [&] { return isCancelRequested(); });
      body_path = true;
    }
  }
  if (raw.empty())
  {
    std::vector<Pose> previous;
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      previous = cached_path_;
    }
    if (!isCancelRequested() && !previous.empty() && distance(previous.back(), goal) < .05 &&
        std::abs(wrap(previous.back().yaw - goal.yaw)) < .05)
    {
      size_t nearest = 0;
      for (size_t i = 1; i < previous.size(); ++i)
        if (distance(start, previous[i]) < distance(start, previous[nearest]))
          nearest = i;
      std::vector<Pose> remaining{start};
      remaining.insert(remaining.end(), previous.begin() + nearest, previous.end());
      bool clear = true;
      for (size_t i = 1; i < remaining.size(); ++i)
        if (!valid(remaining[i - 1], remaining[i]))
        {
          clear = false;
          break;
        }
      if (clear)
      {
        RCLCPP_WARN(n->get_logger(), "Grid heading reconstruction failed; retaining fully "
                                     "revalidated remaining global path");
        return message(remaining, n->now());
      }
    }
    throw std::runtime_error("Improved A*: no body-clear slope-constrained path");
  }
  auto filtered = raw;
  auto smoothed = raw;
  RCLCPP_INFO(n->get_logger(), "Body-feasible global route: %s",
              body_path ? "analytic/SE2 repair" : "paper A* + Bezier");
  RCLCPP_INFO(n->get_logger(), "Paper bidirectional A*: %zu nodes -> %zu keys -> %zu samples",
              raw.size(), filtered.size(), smoothed.size());
  for (auto &q : smoothed)
  {
    scene_->map->project(q, scene_->max_pitch, scene_->max_roll, scene_->max_step);
  }
  // The framework measures deviation against path vertices. Keep sampling
  // dense after shortcutting so a long two-point segment is reusable while the
  // vehicle moves along it; pivots retain their explicit zero-length edge.
  std::vector<Pose> dense{smoothed.front()};
  for (size_t i = 1; i < smoothed.size(); ++i)
  {
    const Pose &a = smoothed[i - 1], &b = smoothed[i];
    const int count = std::max(1, int(std::ceil(distance(a, b) / .30)));
    for (int j = 1; j <= count; ++j)
    {
      const double fraction = double(j) / count;
      Pose p = b;
      p.x = a.x + fraction * (b.x - a.x);
      p.y = a.y + fraction * (b.y - a.y);
      p.z = a.z + fraction * (b.z - a.z);
      p.yaw = wrap(a.yaw + fraction * wrap(b.yaw - a.yaw));
      if (!scene_->map->project(p, scene_->max_pitch, scene_->max_roll, scene_->max_step) ||
          !valid(dense.back(), p))
        throw std::runtime_error("Dense global path failed full-body validation");
      dense.push_back(p);
    }
  }
  {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cached_path_ = dense;
  }
  return message(dense, n->now());
}
bool PaperGlobalPlanner::validateSmoothedPath(const Path &path) const
{
  if (!scene_ || path.poses.empty())
  {
    return false;
  }
  auto p = positions(path);
  auto live = scene_->snapshot();
  for (size_t i = 1; i < p.size(); ++i)
  {
    if (!scene_->swept(p[i - 1], p[i], live, true))
    {
      return false;
    }
  }
  return true;
}
} // namespace clearing_robot_plugins
PLUGINLIB_EXPORT_CLASS(clearing_robot_plugins::PaperGlobalPlanner,
                       multimap_nav_core::GlobalPlannerBase)
