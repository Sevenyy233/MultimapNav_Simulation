#include "clearing_robot_plugins/mppi_trajectory_planner.hpp"
#include "clearing_robot_plugins/body_path.hpp"
#include "clearing_robot_plugins/planner_utils.hpp"
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <pluginlib/class_list_macros.hpp>
namespace clearing_robot_plugins
{
using detail::distance;
namespace
{
struct Reference
{
  Pose nearest, look, terminal;
  int gear{1};
  double lateral{0}, remaining{0}, curvature{0};
  bool final{true};
  size_t terminal_index{0};
};
Reference referenceAt(const std::vector<Pose> &path, const Pose &p, bool ends)
{
  Reference r;
  r.nearest = r.look = r.terminal = path.back();
  r.final = ends;
  size_t nearest = 0;
  double fraction = 0, best = INFINITY;
  for (size_t i = 0; i + 1 < path.size(); ++i)
  {
    Eigen::Vector2d a(path[i].x, path[i].y),
        d(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
    if (d.norm() < .03)
      continue;
    double f = std::clamp((Eigen::Vector2d(p.x, p.y) - a).dot(d) / d.squaredNorm(), 0., 1.);
    const double heading = wrap(path[i].yaw + f * wrap(path[i + 1].yaw - path[i].yaw));
    double cost = (Eigen::Vector2d(p.x, p.y) - a - f * d).squaredNorm() +
                  .03 * std::pow(wrap(p.yaw - heading), 2);
    if (cost < best)
    {
      best = cost;
      nearest = i;
      fraction = f;
      r.nearest = path[i];
      r.nearest.x += f * d.x();
      r.nearest.y += f * d.y();
      r.nearest.yaw = heading;
    }
  }
  r.lateral = distance(p, r.nearest);
  r.gear = motionDirection(path[nearest], path[std::min(nearest + 1, path.size() - 1)]);
  size_t last = path.size() - 1;
  for (size_t i = nearest + 1; i + 1 < path.size(); ++i)
    if (distance(path[i], path[i + 1]) > .03 && motionDirection(path[i], path[i + 1]) != r.gear)
    {
      last = i;
      r.final = false;
      break;
    }
  r.terminal = path[last];
  r.terminal_index = last;
  double length = 0;
  r.look = r.terminal;
  Pose previous = r.nearest;
  for (size_t i = nearest + 1; i <= last; ++i)
  {
    const double edge = distance(previous, path[i]);
    if (length < 1.8 && edge > .01)
      r.curvature = std::max(r.curvature, std::abs(wrap(path[i].yaw - previous.yaw)) / edge);
    if (length < 1.8 && length + edge >= 1.8)
    {
      const double f = (1.8 - length) / edge;
      r.look = path[i];
      r.look.x = previous.x + f * (path[i].x - previous.x);
      r.look.y = previous.y + f * (path[i].y - previous.y);
      r.look.yaw = wrap(previous.yaw + f * wrap(path[i].yaw - previous.yaw));
    }
    length += edge;
    previous = path[i];
  }
  if (r.final && length < 1.8)
  {
    const double extension = 1.8 - length;
    r.look = r.terminal;
    r.look.x += r.gear * extension * std::cos(r.terminal.yaw);
    r.look.y += r.gear * extension * std::sin(r.terminal.yaw);
  }
  r.remaining = length;
  (void)fraction;
  return r;
}
Pose integrate(Pose p, double v, double w, double dt)
{
  p.x += v * std::cos(p.yaw + w * dt / 2) * dt;
  p.y += v * std::sin(p.yaw + w * dt / 2) * dt;
  p.yaw = wrap(p.yaw + w * dt);
  return p;
}
} // namespace
void MPPITrajectory::configure(const Node::WeakPtr &p, std::string name)
{
  node_ = p;
  auto n = p.lock();
  scene_ = Scene::create(p);
  limits_ = MotionLimits::load(n, scene_->model);
  limits_.forward =
      std::min(limits_.forward, parameter(n, name + ".max_linear_velocity", limits_.forward));
  limits_.yaw_rate =
      std::min(limits_.yaw_rate, parameter(n, name + ".max_angular_velocity", limits_.yaw_rate));
  samples_ = parameter(n, name + ".samples", 40);
  steps_ = parameter(n, name + ".steps", 24);
  dt_ = parameter(n, name + ".dt", .25);
  temperature_ = parameter(n, name + ".temperature", .3);
  frequency_ = parameter(n, "controller_frequency", 8.0);
  if (samples_ < 4 || steps_ < 5 || dt_ <= 0 || temperature_ <= 0 || frequency_ <= 0)
    throw std::runtime_error("Invalid MPPI configuration");
  guard_sub_=n->create_subscription<std_msgs::msg::String>("/clearing/safety_status",10,
      [this](std_msgs::msg::String::ConstSharedPtr m){
        try{auto d=YAML::Load(m->data);if(d["command_protocol"].as<int>()<3)return;
          std::lock_guard<std::mutex> lock(mutex_);
          guard_v_=d["command"][0].as<double>();guard_w_=d["command"][1].as<double>();
          guard_a_=d["command_acceleration"][0].as<double>();guard_aw_=d["command_acceleration"][1].as<double>();
          guard_stamp_=d["commit_stamp"].as<double>();
          guard_epoch_=d["reset_sequence"].as<uint64_t>();guard_seen_=true;
        }catch(const YAML::Exception&){}
      });
  warm_.assign(steps_, {});
  last_control_time_ = -1;
}
void MPPITrajectory::setLocalPlan(const Path &p) { setLocalPlan(p, true); }
void MPPITrajectory::setLocalPlan(const Path &p, bool ends)
{
  std::lock_guard<std::mutex> lock(mutex_);
  path_ = p;
  ends_at_goal_ = ends;
  goal_reached_ = false;
}
multimap_nav_core::TrajectoryPlannerBase::ControlResult
MPPITrajectory::computeControl(const geometry_msgs::msg::PoseStamped &current,
                               const geometry_msgs::msg::Twist &velocity)
{
  auto n = node_.lock();
  ControlResult result;
  result.trajectory.header.frame_id = "map";
  result.trajectory.header.stamp = n->now();
  std::vector<Pose> route;
  bool ends;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    route = detail::positions(path_);
    ends = ends_at_goal_;
  }
  Pose start = fromMsg(current.pose);
  auto live = scene_->snapshot();
  double now = n->now().seconds();
  double command_dt = last_control_time_ < 0 ? 1 / frequency_ :
      std::clamp(now - last_control_time_, .01, .25);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if(!guard_seen_)throw std::runtime_error("Shared committed command history unavailable");
    if(last_control_time_<0 || now-last_control_time_>1. || guard_epoch_!=control_epoch_ ||
        std::abs(linear_.velocity-guard_v_)+std::abs(angular_.velocity-guard_w_)>1e-6){
      // Measured twist contains slip/estimator noise and is not the previous
      // commanded velocity. The common guard owns that history for all methods.
      linear_={guard_v_,guard_a_};angular_={guard_w_,guard_aw_};
    }
    control_epoch_=guard_epoch_;
    // A hold/brake may have committed a newer command than this planner's
    // preceding calculation. Bound the increment from that executed sample,
    // not from time spent searching while the guard was stopping the robot.
    if(guard_stamp_>=0)
      command_dt=std::clamp(now-guard_stamp_,1/frequency_,.25);
  }
  last_control_time_ = now;
  auto append = [&](const Pose &p, double v, double w, double a, double aw, int step)
  {
    multimap_nav_msgs::msg::TrajectoryPoint point;
    point.pose = toMsg(p);
    point.velocity.linear.x = v;
    point.velocity.angular.z = w;
    point.acceleration.linear.x = a;
    point.acceleration.angular.z = aw;
    double t = step * dt_;
    point.time_from_start.sec = int(t);
    point.time_from_start.nanosec = uint32_t((t - int(t)) * 1e9);
    result.trajectory.points.push_back(point);
  };
  append(start, velocity.linear.x, velocity.angular.z, linear_.acceleration, angular_.acceleration,
         0);
  auto controlled_stop = [&]()
  {
    linear_.advance(0, limits_.acceleration, limits_.jerk, command_dt);
    angular_.advance(0, limits_.yaw_acceleration, limits_.yaw_jerk, command_dt);
    result.command.linear.x = linear_.velocity;
    result.command.angular.z = angular_.velocity;
    append(integrate(start, linear_.velocity, angular_.velocity, dt_), linear_.velocity,
           angular_.velocity, linear_.acceleration, angular_.acceleration, 1);
  };
  if (!active_ || route.size() < 2 || !scene_->healthy())
  {
    controlled_stop();
    return result;
  }
  auto current_reference = referenceAt(route, start, ends);
  if (!current_reference.final &&
      distance(start, current_reference.terminal) <
          std::clamp(.3 * distance(route.front(), current_reference.terminal), .035, .12) &&
      std::abs(velocity.linear.x) < .035 && std::abs(linear_.velocity) < .035)
  {
    const auto cut = current_reference.terminal_index;
    if (cut > 0)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (path_.poses.size() == route.size())
        path_.poses.erase(path_.poses.begin(), path_.poses.begin() + cut);
      route.erase(route.begin(), route.begin() + cut);
      current_reference = referenceAt(route, start, ends);
    }
  }
  const double goal_distance = distance(start, route.back());
  if (ends && goal_distance < .25 && std::abs(wrap(start.yaw - route.back().yaw)) < .16)
  {
    controlled_stop();
    if (std::abs(velocity.linear.x) < .025 && std::abs(velocity.angular.z) < .025 &&
        std::abs(linear_.velocity) < .01 && std::abs(angular_.velocity) < .01)
    {
      goal_reached_ = true;
      result.trajectory.reaches_goal = true;
    }
    return result;
  }
  auto nominal = [&](const Pose &p)
  {
    auto r = referenceAt(route, p, ends);
    Command u;
    double remaining = r.remaining;
    const double terminal_distance = distance(p, r.terminal);
    int gear = r.gear;
    if (r.final && terminal_distance < .55 && remaining < .1)
    {
      const double along =
          (r.terminal.x - p.x) * std::cos(p.yaw) + (r.terminal.y - p.y) * std::sin(p.yaw);
      gear = along < 0 ? -1 : 1;
      remaining = terminal_distance;
    }
    double speed_cap = gear < 0 ? limits_.reverse : limits_.forward;
    if (r.curvature > 1e-5)
      speed_cap = std::min({speed_cap, limits_.yaw_rate / r.curvature,
                            std::sqrt(limits_.lateral_acceleration / r.curvature)});
    u.v = gear * std::min(speed_cap, std::sqrt(2 * limits_.acceleration *
                                               std::max(0., remaining - (r.final ? .10 : .02))));
    if (r.final && terminal_distance < .25 && std::abs(wrap(p.yaw - r.terminal.yaw)) < .14)
      u.v = 0;
    const double bearing = std::atan2(r.look.y - p.y, r.look.x - p.x);
    double heading_error = wrap(r.look.yaw - p.yaw);
    if (std::abs(heading_error) > 1.)
      u.v *= .3;
    const double tracking_angle = std::abs(wrap(r.nearest.yaw - p.yaw));
    const double factor =
        std::min(1., std::min(.10 / std::max(.10, r.lateral), .05 / std::max(.05, tracking_angle)));
    u.v *= std::max(.25, factor);
    u.w = 2 * u.v * std::sin(bearing - p.yaw) / std::max(.6, distance(p, r.look)) +
          .20 * wrap(r.nearest.yaw - p.yaw);
    const double bound = std::min(limits_.yaw_rate, std::abs(u.v) / limits_.min_radius);
    u.w = std::clamp(u.w, -bound, bound);
    if (r.lateral < .04 && std::abs(heading_error) < .02)
      u.w = 0;
    return u;
  };
  std::vector<Command> nominal_sequence(steps_);
  Pose seed_pose = start;
  multimap_nav_utils::JerkLimitedAxis seed_v{velocity.linear.x, linear_.acceleration},
      seed_w{velocity.angular.z, angular_.acceleration};
  for (int t = 0; t < steps_; ++t)
  {
    nominal_sequence[t] = nominal(seed_pose);
    seed_v.advance(nominal_sequence[t].v, limits_.acceleration, limits_.jerk, dt_);
    seed_w.advance(nominal_sequence[t].w, limits_.yaw_acceleration, limits_.yaw_jerk, dt_);
    seed_pose = integrate(seed_pose, seed_v.velocity, seed_w.velocity, dt_);
  }
  const double prediction_a = linear_.acceleration, prediction_aw = angular_.acceleration;
  auto rollout = [&](const std::vector<Command> &inputs, bool store) -> double
  {
    Pose p = start;
    double cost = 0, previous_w = velocity.angular.z;
    multimap_nav_utils::JerkLimitedAxis va{velocity.linear.x, prediction_a},
        wa{velocity.angular.z, prediction_aw};
    for (int t = 0; t < steps_; ++t)
    {
      const double previous_a = va.acceleration, previous_aw = wa.acceleration;
      va.advance(inputs[t].v, limits_.acceleration, limits_.jerk, dt_);
      wa.advance(inputs[t].w, limits_.yaw_acceleration, limits_.yaw_jerk, dt_);
      if (!limits_.admissible(va.velocity, wa.velocity, va.acceleration, wa.acceleration))
        return INFINITY;
      Pose next = integrate(p, va.velocity, wa.velocity, dt_);
      if (!scene_->map->project(next, scene_->max_pitch, scene_->max_roll, scene_->max_step) ||
          !scene_->swept(p, next, live))
        return INFINITY;
      auto r = referenceAt(route, next, ends);
      Command desired = nominal(next);
      cost += dt_ * (30 * r.lateral * r.lateral + 8 * std::pow(wrap(next.yaw - r.nearest.yaw), 2) +
                     4 * std::pow(va.velocity - desired.v, 2) + 4 * wa.velocity * wa.velocity +
                     8 * wa.acceleration * wa.acceleration +
                     2 * std::pow((wa.acceleration - previous_aw) / dt_, 2) +
                     .2 * std::pow((va.acceleration - previous_a) / dt_, 2));
      if (previous_w * wa.velocity < 0 && std::abs(previous_w) > .025 &&
          std::abs(wa.velocity) > .025)
        cost += 5;
      if (scene_->model.collides(next, *live->cloud, *live->tree, scene_->margin + .35) ||
          scene_->staticCollision(next, .35))
        cost += dt_ * .5;
      if (store)
        append(next, va.velocity, wa.velocity, va.acceleration, wa.acceleration, t + 1);
      previous_w = wa.velocity;
      p = next;
    }
    return cost + 15 * distance(p, current_reference.terminal) +
           8 * std::pow(wrap(p.yaw - current_reference.terminal.yaw), 2);
  };
  std::vector<std::vector<Command>> sequences;
  std::vector<double> costs;
  double best = INFINITY;
  int best_id = -1;
  std::normal_distribution<double> nv(0, .04), nw(0, .015);
  for (int k = 0; k < samples_; ++k)
  {
    auto sequence = nominal_sequence;
    double ev = 0, ew = 0;
    for (int t = 0; t < steps_; ++t)
    {
      if (k == 1)
      {
        sequence[t] = {};
        continue;
      }
      if (k == 2)
      {
        sequence[t].v = .7 * sequence[t].v + .3 * warm_[t].v;
        sequence[t].w = .7 * sequence[t].w + .3 * warm_[t].w;
      }
      if (k > 2)
      {
        ev = .85 * ev + .527 * nv(random_);
        ew = .85 * ew + .527 * nw(random_);
        sequence[t].v += ev;
        sequence[t].w += ew;
      }
      sequence[t].v = std::clamp(sequence[t].v, -limits_.reverse, limits_.forward);
      const double bound = std::min(limits_.yaw_rate, std::abs(sequence[t].v) / limits_.min_radius);
      sequence[t].w = std::clamp(sequence[t].w, -bound, bound);
    }
    double cost = rollout(sequence, false);
    sequences.push_back(std::move(sequence));
    costs.push_back(cost);
    if (cost < best)
    {
      best = cost;
      best_id = k;
    }
  }
  if (best_id < 0)
  {
    controlled_stop();
    return result;
  }
  std::vector<Command> averaged(steps_);
  double sum = 0;
  for (int k = 0; k < samples_; ++k)
  {
    if (!std::isfinite(costs[k]))
      continue;
    double weight = std::exp(std::max(-60., -(costs[k] - best) / temperature_));
    sum += weight;
    for (int t = 0; t < steps_; ++t)
    {
      averaged[t].v += weight * sequences[k][t].v;
      averaged[t].w += weight * sequences[k][t].w;
    }
  }
  for (auto &u : averaged)
  {
    u.v /= sum;
    u.w /= sum;
  }
  if (!std::isfinite(rollout(averaged, false)))
    averaged = sequences[best_id];
  const auto committed_v = linear_, committed_w = angular_;
  linear_.advance(averaged.front().v, limits_.acceleration, limits_.jerk, command_dt);
  angular_.advance(averaged.front().w, limits_.yaw_acceleration, limits_.yaw_jerk, command_dt);
  // Certify the real command plus its entire jerk-limited braking envelope.
  Pose p = start;
  auto va = linear_, wa = angular_;
  bool safe = true;
  if (std::abs(velocity.linear.x) > std::abs(va.velocity))
    va.velocity = velocity.linear.x;
  if (std::abs(velocity.angular.z) > std::abs(wa.velocity))
    wa.velocity = velocity.angular.z;
  const double age = std::clamp(now - rclcpp::Time(current.header.stamp).seconds(), 0., 1.);
  const int reaction = int(std::ceil((.25 + age) / .1));
  for (int i = 0; i < 80; ++i)
  {
    va.advance(i < reaction ? linear_.velocity : 0., limits_.acceleration, limits_.jerk, .1);
    wa.advance(i < reaction ? angular_.velocity : 0., limits_.yaw_acceleration, limits_.yaw_jerk,
               .1);
    auto next = integrate(p, va.velocity, wa.velocity, .1);
    if (!scene_->swept(p, next, live))
    {
      safe = false;
      break;
    }
    p = next;
  }
  if (!safe)
  {
    linear_ = committed_v;
    angular_ = committed_w;
    controlled_stop();
    return result;
  }
  result.command.linear.x = linear_.velocity;
  result.command.angular.z = angular_.velocity;
  if (!std::isfinite(rollout(averaged, true)))
  {
    result.trajectory.points.resize(1);
    linear_ = committed_v;
    angular_ = committed_w;
    controlled_stop();
    return result;
  }
  for (int t = 0; t + 1 < steps_; ++t)
    warm_[t] = averaged[t + 1];
  warm_.back() = averaged.back();
  return result;
}
geometry_msgs::msg::TwistStamped
MPPITrajectory::computeVelocityCommands(const geometry_msgs::msg::PoseStamped &p,
                                        const geometry_msgs::msg::Twist &v)
{
  auto r = computeControl(p, v);
  geometry_msgs::msg::TwistStamped out;
  out.header = r.trajectory.header;
  out.twist = r.command;
  return out;
}
multimap_nav_msgs::msg::Trajectory
MPPITrajectory::computeTrajectory(const geometry_msgs::msg::PoseStamped &p,
                                  const geometry_msgs::msg::Twist &v)
{
  return computeControl(p, v).trajectory;
}
} // namespace clearing_robot_plugins
PLUGINLIB_EXPORT_CLASS(clearing_robot_plugins::MPPITrajectory,
                       multimap_nav_core::TrajectoryPlannerBase)
