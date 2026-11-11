#include "clearing_robot_plugins/body_path.hpp"
#include "clearing_robot_plugins/planner_utils.hpp"
#include <algorithm>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <queue>
#include <unordered_map>

namespace clearing_robot_plugins
{
using detail::distance;
int motionDirection(const Pose &a, const Pose &b)
{
  return (b.x - a.x) * std::cos(a.yaw) + (b.y - a.y) * std::sin(a.yaw) < 0 ? -1 : 1;
}
bool kinematicPath(const std::vector<Pose> &path, double min_radius)
{
  for (size_t i = 1; i < path.size(); ++i)
  {
    const auto &a = path[i - 1], &b = path[i];
    double ds = distance(a, b), angle = wrap(b.yaw - a.yaw);
    if (ds < 1e-5)
    {
      if (std::abs(angle) > .02)
        return false;
      continue;
    }
    const double bearing = std::atan2(b.y - a.y, b.x - a.x);
    if (std::abs(std::sin(bearing - a.yaw - angle / 2)) > .12 ||
        std::abs(angle) > ds / min_radius + .025)
      return false;
  }
  return true;
}
std::vector<Pose> connectBodyPath(const Pose &a, const Pose &b, const Validator &valid,
                                  const BodySearchOptions &o)
{
  const double d = distance(a, b);
  if (d < .03 && (std::abs(wrap(b.yaw - a.yaw)) < .03 || o.allow_spin) && valid(a, b))
    return {a, b};
  std::vector<Pose> best;
  double best_cost = INFINITY;
  auto accept_curve = [&](const std::vector<double> &segments, const auto &interpolate)
  {
    double total = 0;
    for (double s : segments)
      total += std::abs(s);
    if (!std::isfinite(total) || total < 1e-7 || total > best_cost)
      return;
    std::vector<Pose> path{a};
    double prefix = 0, cost = 0;
    int last_gear = 0;
    bool clear = true;
    for (double length : segments)
    {
      if (std::abs(length) < 1e-7)
        continue;
      int gear = length < 0 ? -1 : 1;
      if (last_gear && last_gear != gear)
        cost += o.switch_cost;
      int steps = std::max(1, int(std::ceil(std::abs(length) / .10)));
      for (int j = 1; j <= steps; ++j)
      {
        double f = (prefix + std::abs(length) * j / steps) / total;
        Pose p = interpolate(f);
        p.z = a.z + f * (b.z - a.z);
        if (f > 1 - 1e-9)
          p = b;
        if (!valid(path.back(), p))
        {
          clear = false;
          break;
        }
        cost += distance(path.back(), p) * (gear < 0 ? o.reverse_weight : 1.) +
                .5 * std::abs(wrap(p.yaw - path.back().yaw));
        path.push_back(p);
      }
      if (!clear)
        break;
      prefix += std::abs(length);
      last_gear = gear;
    }
    if (clear && cost < best_cost && kinematicPath(path, o.min_radius))
    {
      best_cost = cost;
      best = std::move(path);
    }
  };
  // OMPL supplies analytic car kinematics only. Every state and edge is still
  // checked against this robot's complete 3D model, terrain, and selected map.
  namespace ob = ompl::base;
  auto reeds = std::make_shared<ob::ReedsSheppStateSpace>(o.min_radius);
  ob::ScopedState<ob::SE2StateSpace> rs_a(reeds), rs_b(reeds), rs_p(reeds);
  rs_a->setXY(a.x, a.y);
  rs_a->setYaw(a.yaw);
  rs_b->setXY(b.x, b.y);
  rs_b->setYaw(b.yaw);
  auto rs_word = reeds->reedsShepp(rs_a.get(), rs_b.get());
  bool rs_first = false;
  std::vector<double> segments;
  for (double v : rs_word.length_)
    segments.push_back(v * o.min_radius);
  accept_curve(segments,
               [&](double f)
               {
                 reeds->interpolate(rs_a.get(), rs_b.get(), f, rs_first, rs_word, rs_p.get());
                 Pose p = a;
                 p.x = rs_p->getX();
                 p.y = rs_p->getY();
                 p.yaw = rs_p->getYaw();
                 return p;
               });
  for (int gear : {1, -1})
  {
    auto dubins = std::make_shared<ob::DubinsStateSpace>(o.min_radius, false);
    ob::ScopedState<ob::SE2StateSpace> from(dubins), to(dubins), point(dubins);
    const Pose &first = gear > 0 ? a : b, &last = gear > 0 ? b : a;
    from->setXY(first.x, first.y);
    from->setYaw(first.yaw);
    to->setXY(last.x, last.y);
    to->setYaw(last.yaw);
    auto word = dubins->dubins(from.get(), to.get());
    bool first_time = false;
    segments.clear();
    if (gear > 0)
      for (double v : word.length_)
        segments.push_back(v * o.min_radius);
    else
      for (int i = 2; i >= 0; --i)
        segments.push_back(-word.length_[i] * o.min_radius);
    accept_curve(segments,
                 [&](double f)
                 {
                   dubins->interpolate(from.get(), to.get(), gear > 0 ? f : 1 - f, first_time, word,
                                       point.get());
                   Pose p = a;
                   p.x = point->getX();
                   p.y = point->getY();
                   p.yaw = point->getYaw();
                   return p;
                 });
  }
  for (int gear : {1, -1})
    for (double scale : {.5, 1., 1.6})
    {
      const double length = std::max(.2, d * scale);
      const Eigen::Vector2d p0(a.x, a.y), p3(b.x, b.y),
          p1 = p0 + gear * length * Eigen::Vector2d(std::cos(a.yaw), std::sin(a.yaw)),
          p2 = p3 - gear * length * Eigen::Vector2d(std::cos(b.yaw), std::sin(b.yaw));
      const int count = std::max(8, int(std::ceil((d + 2 * length) / .12)));
      std::vector<Pose> path{a};
      bool clear = true;
      double cost = 0;
      for (int i = 1; i <= count; ++i)
      {
        double t = double(i) / count, u = 1 - t;
        Eigen::Vector2d point =
            u * u * u * p0 + 3 * u * u * t * p1 + 3 * u * t * t * p2 + t * t * t * p3;
        Eigen::Vector2d v = 3 * u * u * (p1 - p0) + 6 * u * t * (p2 - p1) + 3 * t * t * (p3 - p2);
        Eigen::Vector2d acc = 6 * u * (p2 - 2 * p1 + p0) + 6 * t * (p3 - 2 * p2 + p1);
        const double speed = v.norm();
        if (speed < 1e-6 || std::abs(v.x() * acc.y() - v.y() * acc.x()) / std::pow(speed, 3) >
                                1 / o.min_radius + 1e-6)
        {
          clear = false;
          break;
        }
        Pose p = a;
        p.x = point.x();
        p.y = point.y();
        p.z = a.z + t * (b.z - a.z);
        p.yaw = wrap(std::atan2(v.y(), v.x()) + (gear < 0 ? M_PI : 0));
        if (i == count)
          p = b;
        // Reject a cubic that loops past a cusp or reverses its tangent.
        if (motionDirection(path.back(), p) != gear || !valid(path.back(), p))
        {
          clear = false;
          break;
        }
        cost += distance(path.back(), p) * (gear < 0 ? o.reverse_weight : 1.) +
                .5 * std::abs(wrap(p.yaw - path.back().yaw));
        path.push_back(p);
      }
      if (clear && cost < best_cost)
      {
        best_cost = cost;
        best = std::move(path);
      }
    }
  // Parallel chassis poses with lateral displacement need a real manoeuvre,
  // not a forbidden sideways edge or an in-place quarter turn. A smooth
  // curvature S followed by a signed straight leg supplies that connection.
  if (std::abs(wrap(b.yaw - a.yaw)) < .03)
  {
    const double c = std::cos(a.yaw), sn = std::sin(a.yaw);
    const double lateral = -(b.x - a.x) * sn + (b.y - a.y) * c;
    if (std::abs(lateral) > .05)
      for (int gear : {1, -1})
        for (double amplitude : {.5, .8, 1.1, 1.4, 1.65, 1.9, 2.1})
        {
          constexpr int integration_samples = 300;
          double iy = 0;
          for (int i = 0; i < integration_samples; ++i)
          {
            double u = (i + .5) / integration_samples;
            iy += std::sin(amplitude * std::pow(std::sin(M_PI * u), 2)) / integration_samples;
          }
          if (iy < 1e-6)
            continue;
          const double length = std::abs(lateral) / iy;
          if (amplitude * M_PI / length > 1 / o.min_radius || length > 50)
            continue;
          const int count = std::max(30, int(std::ceil(length / .10)));
          double exact_y = 0;
          for (int i = 0; i < count; ++i)
          {
            double u = (i + .5) / count;
            exact_y += std::sin(amplitude * std::pow(std::sin(M_PI * u), 2)) / count;
          }
          const double ds = std::abs(lateral) / exact_y / count;
          std::vector<Pose> path{a};
          bool clear = true;
          double cost = 0;
          for (int i = 1; i <= count; ++i)
          {
            double u = (i - .5) / count;
            double heading = a.yaw + std::copysign(1., lateral) * gear * amplitude *
                                         std::pow(std::sin(M_PI * u), 2);
            Pose point = path.back();
            point.x += gear * ds * std::cos(heading);
            point.y += gear * ds * std::sin(heading);
            point.yaw = wrap(a.yaw + std::copysign(1., lateral) * gear * amplitude *
                                         std::pow(std::sin(M_PI * double(i) / count), 2));
            if (!valid(path.back(), point))
            {
              clear = false;
              break;
            }
            cost += ds * (gear < 0 ? o.reverse_weight : 1.);
            path.push_back(point);
          }
          if (!clear)
            continue;
          const Pose end = path.back();
          const int tail_count = std::max(1, int(std::ceil(distance(end, b) / .12)));
          const int tail_gear = motionDirection(end, b);
          if (tail_gear != gear)
            cost += o.switch_cost;
          for (int i = 1; i <= tail_count; ++i)
          {
            Pose point = b;
            point.x = end.x + (b.x - end.x) * i / tail_count;
            point.y = end.y + (b.y - end.y) * i / tail_count;
            point.z = end.z + (b.z - end.z) * i / tail_count;
            if (!valid(path.back(), point))
            {
              clear = false;
              break;
            }
            cost += distance(path.back(), point) * (tail_gear < 0 ? o.reverse_weight : 1.);
            path.push_back(point);
          }
          if (clear && cost < best_cost)
          {
            best_cost = cost;
            best = std::move(path);
          }
        }
  }
  return best;
}

std::vector<Pose> searchBodyPath(const Scene &scene, const Pose &start, const Pose &goal,
                                 const std::shared_ptr<const LiveData> &live, bool dynamic,
                                 const BodySearchOptions &o, const std::function<bool()> &cancel)
{
  const bool start_buffer = scene.safe(start, live, dynamic, o.clearance);
  const bool goal_buffer = scene.safe(goal, live, dynamic, o.clearance);
  Validator valid = [&](const Pose &a, const Pose &b)
  {
    double factor = 1.;
    if (!start_buffer)
      factor = std::min(factor,
                        std::clamp(std::min(distance(a, start), distance(b, start)) / 1.5, 0., 1.));
    if (!goal_buffer)
      factor = std::min(factor,
                        std::clamp(std::min(distance(a, goal), distance(b, goal)) / 1.5, 0., 1.));
    return scene.swept(a, b, live, dynamic, o.clearance * factor);
  };
  if (!scene.safe(start, live, dynamic) || !scene.safe(goal, live, dynamic))
    return {};
  if (auto direct = connectBodyPath(start, goal, valid, o); !direct.empty())
    return direct;
  const auto &map = *scene.map;
  std::vector<uint8_t> blocked(map.elevations.size(), 0);
  const double radius = scene.model.boxes.front().size.head<2>().minCoeff() / 2 + scene.margin;
  auto mark = [&](const Cloud &cloud)
  {
    const int reach = int(std::ceil(radius / map.resolution));
    for (const auto &p : cloud)
    {
      double ground;
      if (!map.height(p.x, p.y, ground) || p.z < ground + .15 || p.z > ground + 1.8)
        continue;
      int k = map.index(p.x, p.y);
      if (k < 0)
        continue;
      for (int y = -reach; y <= reach; ++y)
        for (int x = -reach; x <= reach; ++x)
        {
          if (std::hypot(x, y) * map.resolution > radius)
            continue;
          int xx = k % map.width + x, yy = k / map.width + y;
          if (xx >= 0 && yy >= 0 && xx < map.width && yy < map.height_cells)
            blocked[yy * map.width + xx] = 1;
        }
    }
  };
  mark(*map.obstacles);
  if (dynamic)
    mark(*live->cloud);
  using Queue = std::pair<double, int>;
  std::priority_queue<Queue, std::vector<Queue>, std::greater<Queue>> frontier;
  std::vector<double> field(blocked.size(), INFINITY);
  int target = map.index(goal.x, goal.y);
  if (target < 0)
    return {};
  field[target] = 0;
  frontier.push({0, target});
  while (!frontier.empty())
  {
    auto [c, k] = frontier.top();
    frontier.pop();
    if (c > field[k] + 1e-9)
      continue;
    for (int y = -1; y <= 1; ++y)
      for (int x = -1; x <= 1; ++x)
      {
        if (!x && !y)
          continue;
        int xx = k % map.width + x, yy = k / map.width + y;
        if (xx < 0 || yy < 0 || xx >= map.width || yy >= map.height_cells)
          continue;
        int v = yy * map.width + xx;
        if (blocked[v] || !std::isfinite(map.elevations[v]))
          continue;
        double next = c + map.resolution * std::hypot(x, y);
        if (next < field[v])
        {
          field[v] = next;
          frontier.push({next, v});
        }
      }
  }
  auto heuristic = [&](const Pose &p)
  {
    int k = map.index(p.x, p.y);
    return std::max(distance(p, goal),
                    k >= 0 && std::isfinite(field[k]) ? field[k] : distance(p, goal));
  };
  struct Entry
  {
    Pose p;
    int parent, gear, steer;
    double cost;
    std::vector<Pose> edge;
  };
  std::vector<Entry> nodes{{start, -1, 0, 0, 0, {}}};
  auto key = [](const Pose &p, int gear, int steer)
  {
    int64_t x = std::llround(p.x / .35) + 10000, y = std::llround(p.y / .35) + 10000;
    int64_t a = int(std::floor((wrap(p.yaw) + M_PI) / (2 * M_PI) * 72)) % 72;
    return (((x * 20000 + y) * 72 + a) * 3 + (gear + 1)) * 5 + (steer + 2);
  };
  std::unordered_map<int64_t, double> visited;
  frontier.push({heuristic(start), 0});
  auto begun = std::chrono::steady_clock::now();
  for (int count = 0;
       !frontier.empty() && count < o.max_expansions && !detail::expired(begun, o.timeout); ++count)
  {
    if (cancel && cancel())
      return {};
    int id = frontier.top().second;
    frontier.pop();
    const Entry u = nodes[id];
    if (count % 24 == 0 || distance(u.p, goal) < 1.)
    {
      auto tail = connectBodyPath(u.p, goal, valid, o);
      if (!tail.empty())
      {
        std::vector<int> chain;
        for (int i = id; i >= 0; i = nodes[i].parent)
          chain.push_back(i);
        std::reverse(chain.begin(), chain.end());
        std::vector<Pose> out{start};
        for (int i : chain)
          out.insert(out.end(), nodes[i].edge.begin(), nodes[i].edge.end());
        out.insert(out.end(), tail.begin() + 1, tail.end());
        return out;
      }
    }
    for (int gear : {1, -1})
      for (int steer : {0, -1, 1, -2, 2})
      {
        const double curvature = double(steer) / (2 * o.min_radius);
        std::vector<Pose> edge;
        Pose last = u.p;
        bool clear = true;
        int samples = std::max(
            2, int(std::ceil((o.step + scene.model.radius * o.step * std::abs(curvature)) / .15)));
        for (int i = 1; i <= samples; ++i)
        {
          double ds = gear * o.step * i / samples;
          Pose p = u.p;
          p.yaw = wrap(u.p.yaw + ds * curvature);
          if (std::abs(curvature) < 1e-9)
          {
            p.x += ds * std::cos(u.p.yaw);
            p.y += ds * std::sin(u.p.yaw);
          }
          else
          {
            p.x += (std::sin(p.yaw) - std::sin(u.p.yaw)) / curvature;
            p.y -= (std::cos(p.yaw) - std::cos(u.p.yaw)) / curvature;
          }
          if (!scene.map->project(p, scene.max_pitch, scene.max_roll, scene.max_step) ||
              !valid(last, p))
          {
            clear = false;
            break;
          }
          edge.push_back(p);
          last = p;
        }
        if (!clear)
          continue;
        double cost = u.cost + o.step * (gear < 0 ? o.reverse_weight : 1.) + .25 * std::abs(steer) +
                      (u.gear && gear != u.gear ? o.switch_cost : 0.) +
                      o.steering_change_cost * std::abs(steer - u.steer) * .1;
        auto k = key(last, gear, steer);
        if (visited.count(k) && visited[k] <= cost)
          continue;
        visited[k] = cost;
        int v = nodes.size();
        nodes.push_back({last, id, gear, steer, cost, std::move(edge)});
        frontier.push({cost + 2. * heuristic(last) + .4 * std::abs(wrap(last.yaw - goal.yaw)), v});
      }
  }
  return {};
}
} // namespace clearing_robot_plugins
