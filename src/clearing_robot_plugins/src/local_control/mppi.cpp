#include "clearing_robot/local_control/mppi.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <octomap_msgs/conversions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace clearing_robot
{
namespace
{
constexpr double pi = 3.141592653589793;
constexpr double front = 6.05;
constexpr double rear = -3.45;
constexpr double half_width = 1.75;
constexpr double max_slope = 0.30;
constexpr double max_step = 0.25;
constexpr double av = 0.30;
constexpr double aw = 0.20;
constexpr double control_dt = 0.10;

/** @brief 返回规范化到 [-π,π] 的角度差。 */
double wrap(double value) {return std::remainder(value, 2.0 * pi);}

/** @brief 找到当前位姿在路线前向段上的最近样点。 */
std::size_t nearest(const nav_msgs::msg::Path & path, const State & state)
{
  std::size_t selected = 0;
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const auto & pose = path.poses[i].pose;
    const double distance = std::hypot(pose.position.x - state.x, pose.position.y - state.y);
    const double cost = distance + 0.5 * (1.0 - std::cos(wrap(tf2::getYaw(pose.orientation) - state.yaw)));
    if (cost < best) {
      selected = i;
      best = cost;
    }
  }
  return selected;
}

/** @brief 沿离散路径前进指定距离（米），返回最近前视样点。 */
std::size_t lookahead(const nav_msgs::msg::Path & path, std::size_t index, double distance)
{
  while (index + 1 < path.poses.size() && distance > 0.0) {
    const auto & a = path.poses[index].pose.position;
    const auto & b = path.poses[index + 1].pose.position;
    distance -= std::hypot(b.x - a.x, b.y - a.y);
    ++index;
  }
  return index;
}

/** @brief 将横纵坐标从车体系旋转、平移到地图系。 */
void bodyPoint(const State & s, double x, double y, double & map_x, double & map_y)
{
  map_x = s.x + std::cos(s.yaw) * x - std::sin(s.yaw) * y;
  map_y = s.y + std::sin(s.yaw) * x + std::cos(s.yaw) * y;
}

/** @brief 以最大绝对物理量归一化并截断。 */
float scale(double value, double limit)
{
  return static_cast<float>(std::clamp(value / limit, -1.0, 1.0));
}
}  // namespace

LocalScene::LocalScene(const clearing_robot_plugins::msg::CompoundLocalMap & message)
: header(message.header)
{
  if (header.frame_id != "map" || message.terrain.header.frame_id != header.frame_id ||
    message.occupancy.header.frame_id != header.frame_id ||
    !grid_map::GridMapRosConverter::fromMessage(message.terrain, terrain_) ||
    !terrain_.exists("elevation") || !terrain_.exists("slope"))
  {
    return;
  }
  std::unique_ptr<octomap::AbstractOcTree> abstract(octomap_msgs::msgToMap(message.occupancy));
  const auto * tree = dynamic_cast<const octomap::OcTree *>(abstract.get());
  if (!tree) {
    return;
  }
  const auto size = terrain_.getSize();
  obstacles_ = grid_map::Matrix::Zero(size(0), size(1));
  for (auto leaf = tree->begin_leafs(); leaf != tree->end_leafs(); ++leaf) {
    if (!tree->isNodeOccupied(*leaf)) {
      continue;
    }
    const double side = leaf.getSize();
    const double stride = std::min(side, terrain_.getResolution() * 0.75);
    for (double dx = -0.5 * side + 0.5 * stride; dx < 0.5 * side; dx += stride) {
      for (double dy = -0.5 * side + 0.5 * stride; dy < 0.5 * side; dy += stride) {
        const double x = leaf.getX() + dx;
        const double y = leaf.getY() + dy;
        double height = 0.0;
        double slope = 0.0;
        grid_map::Index index;
        // 生成端已剥离支承地形；控制端只按车体净高裁剪，不再丢弃低矮障碍。
        const bool has_ground = ground(x, y, height, slope);
        if (terrain_.getIndex(grid_map::Position(x, y), index) &&
          (!has_ground ||
          (leaf.getZ() + 0.5 * side > height && leaf.getZ() - 0.5 * side < height + 4.30)))
        {
          obstacles_(index(0), index(1)) = 1.0;
        }
      }
    }
  }
  valid = true;
}

bool LocalScene::ground(double x, double y, double & elevation, double & slope) const
{
  grid_map::Index index;
  if (!terrain_.getIndex(grid_map::Position(x, y), index)) {
    return false;
  }
  elevation = terrain_.at("elevation", index);
  slope = terrain_.at("slope", index);
  return std::isfinite(elevation) && std::isfinite(slope);
}

bool LocalScene::occupied(double x, double y) const
{
  grid_map::Index index;
  return terrain_.getIndex(grid_map::Position(x, y), index) &&
         obstacles_(index(0), index(1)) > 0.5;
}

bool LocalScene::traversable(const State & state, double margin) const
{
  if (!valid) {
    return false;
  }
  double center = 0.0;
  double slope = 0.0;
  if (!ground(state.x, state.y, center, slope) || slope > max_slope) {
    return false;
  }
  const double step = terrain_.getResolution() * 0.9;
  const int nx = static_cast<int>(std::ceil((front - rear + 2.0 * margin) / step));
  const int ny = static_cast<int>(std::ceil((2.0 * half_width + 2.0 * margin) / step));
  const double cosine = std::cos(state.yaw);
  const double sine = std::sin(state.yaw);
  for (int i = 0; i <= nx; ++i) {
    const double x = rear - margin + i * (front - rear + 2.0 * margin) / nx;
    for (int j = 0; j <= ny; ++j) {
      const double y = -half_width - margin + j * (2.0 * half_width + 2.0 * margin) / ny;
      const double map_x = state.x + cosine * x - sine * y;
      const double map_y = state.y + sine * x + cosine * y;
      double elevation = 0.0;
      if (!ground(map_x, map_y, elevation, slope) || slope > max_slope ||
        std::abs(elevation - center) > max_step + std::tan(max_slope) * std::hypot(x, y) ||
        occupied(map_x, map_y))
      {
        return false;
      }
    }
  }
  return true;
}

bool LocalScene::collision(const State & state) const
{
  if (!valid) {
    return false;
  }
  const double step = terrain_.getResolution() * 0.8;
  for (double x = rear; x <= front + 1e-6; x += step) {
    for (double y = -half_width; y <= half_width + 1e-6; y += step) {
      double map_x = 0.0;
      double map_y = 0.0;
      bodyPoint(state, x, y, map_x, map_y);
      if (occupied(map_x, map_y)) {
        return true;
      }
    }
  }
  return false;
}

double LocalScene::range(const State & state, double relative_bearing) const
{
  const double angle = state.yaw + relative_bearing;
  for (double distance = 0.5; distance < 12.0; distance += 0.5) {
    const double x = state.x + std::cos(angle) * distance;
    const double y = state.y + std::sin(angle) * distance;
    double height = 0.0;
    double slope = 0.0;
    if (!ground(x, y, height, slope) || slope > max_slope || occupied(x, y)) {
      return distance;
    }
  }
  return 12.0;
}

State Mppi::state(const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Twist & velocity)
{
  return {pose.position.x, pose.position.y, pose.position.z,
    tf2::getYaw(pose.orientation), velocity.linear.x, velocity.angular.z, velocity.linear.y};
}

Action Mppi::normalize(const geometry_msgs::msg::Twist & twist)
{
  return {scale(2.0 * twist.linear.x - v_min - v_max, v_max - v_min), scale(twist.angular.z, w_max)};
}

geometry_msgs::msg::Twist Mppi::decode(const Action & action)
{
  geometry_msgs::msg::Twist value;
  value.linear.x = v_min + 0.5 * (v_max - v_min) * (1.0 + std::clamp(action[0], -1.0f, 1.0f));
  value.angular.z = w_max * std::clamp(action[1], -1.0f, 1.0f);
  return value;
}

nav_msgs::msg::Path Mppi::localPath(
  const LocalScene & scene, const State & current, const nav_msgs::msg::Path & global,
  bool & ends_at_goal) const
{
  ends_at_goal = false;
  nav_msgs::msg::Path result;
  if (!scene.valid || global.poses.empty() || global.header.frame_id != scene.header.frame_id) {
    return result;
  }
  result.header = scene.header;
  const auto first = nearest(global, current);
  const auto last = lookahead(global, first, 12.0);
  const std::array<double, 5> offsets = {0.0, 1.5, -1.5, 2.8, -2.8};
  for (const double offset : offsets) {
    result.poses.assign(global.poses.begin() + first, global.poses.begin() + last + 1);
    const double count = static_cast<double>(std::max<std::size_t>(1, result.poses.size() - 1));
    for (std::size_t i = 0; i < result.poses.size(); ++i) {
      auto & pose = result.poses[i];
      const double yaw = tf2::getYaw(pose.pose.orientation);
      const double phase = pi * static_cast<double>(i) / count;
      const double shift = offset * std::sin(phase) * std::sin(phase);
      pose.pose.position.x -= std::sin(yaw) * shift;
      pose.pose.position.y += std::cos(yaw) * shift;
      pose.header = result.header;
    }
    bool feasible = true;
    for (std::size_t i = 0; i < result.poses.size(); ++i) {
      auto candidate = state(result.poses[i].pose, {});
      if (i + 1 < result.poses.size()) {
        const auto & next = result.poses[i + 1].pose.position;
        candidate.yaw = std::atan2(next.y - candidate.y, next.x - candidate.x);
        tf2::Quaternion heading;
        heading.setRPY(0.0, 0.0, candidate.yaw);
        result.poses[i].pose.orientation = tf2::toMsg(heading);
      } else if (i > 0) {
        candidate.yaw = tf2::getYaw(result.poses[i - 1].pose.orientation);
      }
      if (!scene.traversable(candidate)) {
        feasible = false;
        break;
      }
    }
    if (feasible) {
      ends_at_goal = last + 1 == global.poses.size();
      return result;
    }
  }
  // 障碍物远端可能封住整段；允许跟踪入口前的已验证部分，随后重新规划。
  result.poses.clear();
  for (std::size_t i = first; i <= last; ++i) {
    auto candidate = global.poses[i];
    candidate.header = result.header;
    if (!scene.traversable(state(candidate.pose, {}))) {
      break;
    }
    result.poses.push_back(candidate);
  }
  ends_at_goal = !result.poses.empty() && first + result.poses.size() == global.poses.size();
  if (result.poses.size() < 2 && !ends_at_goal) {
    result.poses.clear();
  }
  return result;
}

geometry_msgs::msg::Twist Mppi::cruiseTarget(const geometry_msgs::msg::Twist & request)
{
  geometry_msgs::msg::Twist value;
  if (request.linear.x <= 0.0) {
    return value;
  }
  value.linear.x = std::clamp(request.linear.x, v_min, v_max);
  const double turn = std::min(w_max, curvature_max * value.linear.x);
  value.angular.z = std::clamp(request.angular.z, -turn, turn);
  return value;
}

State Mppi::advance(State state, const geometry_msgs::msg::Twist & command, double dt, double traction)
{
  const double v = state.v + std::clamp(traction * command.linear.x - state.v, -av * dt, av * dt);
  const double w = state.w + std::clamp(command.angular.z - state.w, -aw * dt, aw * dt);
  const double yaw = state.yaw + 0.25 * dt * (state.w + w);
  state.x += dt * (0.5 * (state.v + v) * std::cos(yaw) - state.lateral * std::sin(yaw));
  state.y += dt * (0.5 * (state.v + v) * std::sin(yaw) + state.lateral * std::cos(yaw));
  state.lateral *= std::exp(-dt / 0.6);
  state.yaw = wrap(state.yaw + 0.5 * dt * (state.w + w));
  state.v = v;
  state.w = w;
  return state;
}

bool Mppi::canBrake(
  const LocalScene & scene, State state, const geometry_msgs::msg::Twist & command)
{
  for (int i = 0; i < 75; ++i) {
    auto target = command;
    if (i >= 8) {
      // 为指令传输和加加速度过渡留出余量，再以保守减速度估计停车扫掠。
      const double remaining_v = std::max(0.0, std::abs(state.v) - av * 0.60 * control_dt);
      target.linear.x = std::copysign(remaining_v, state.v);
      const double remaining_w = std::max(0.0, std::abs(state.w) - aw * 0.60 * control_dt);
      target.angular.z = std::copysign(remaining_w, state.w);
    }
    state = advance(state, target, control_dt, 1.0);
    if (!scene.traversable(state)) {
      return false;
    }
    if (i >= 8 && std::abs(state.v) < 0.001 && std::abs(state.w) < 0.001 &&
      std::abs(state.lateral) < 0.01)
    {
      return true;
    }
  }
  return false;
}

geometry_msgs::msg::Twist Mppi::shield(
  const LocalScene & scene, const State & state, const geometry_msgs::msg::Twist & applied,
  const geometry_msgs::msg::Twist & requested, const geometry_msgs::msg::Twist & prior) const
{
  if (prior.linear.x <= 0.0 || requested.linear.x <= 0.0) {
    return {};
  }
  const auto bounded_request = cruiseTarget(requested);
  State initial = state;
  if (applied.linear.x > initial.v) {
    initial.v = applied.linear.x;
  }
  if (canBrake(scene, initial, bounded_request)) {
    return bounded_request;
  }
  if (canBrake(scene, initial, prior)) {
    return prior;
  }
  return {};
}

geometry_msgs::msg::Twist Mppi::command(
  const LocalScene & scene, const State & current, const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Twist & applied, bool ends_at_goal, const char ** stop_reason) const
{
  const auto stopped = [stop_reason](const char * reason) {
    if (stop_reason) {
      *stop_reason = reason;
    }
    return geometry_msgs::msg::Twist{};
  };
  if (stop_reason) {
    *stop_reason = nullptr;
  }
  if (path.poses.empty()) {
    return stopped("MPPI 无局部路径");
  }
  if (!scene.traversable(current)) {
    return stopped("MPPI 当前车体包络不可通行");
  }
  const auto & end = path.poses.back().pose.position;
  if (ends_at_goal && std::hypot(end.x - current.x, end.y - current.y) <= goal_tolerance) {
    return stopped("MPPI 已到路径终点");
  }
  const auto start = nearest(path, current);
  const auto & target = path.poses[lookahead(path, start, 2.5)].pose.position;
  const double heading = wrap(std::atan2(target.y - current.y, target.x - current.x) - current.yaw);
  double slope = 0.0;
  double elevation = 0.0;
  if (!scene.ground(current.x, current.y, elevation, slope)) {
    return stopped("MPPI 当前地面无有效支承数据");
  }
  const double traction = applied.linear.x > 0.1 ?
    std::clamp(current.v / applied.linear.x, 0.55, 1.0) : 1.0;
  const double goal_distance = std::hypot(end.x - current.x, end.y - current.y);
  geometry_msgs::msg::Twist nominal;
  nominal.linear.x = v_max - (v_max - v_min) * std::clamp(slope / max_slope, 0.0, 1.0);
  if (ends_at_goal) {
    nominal.linear.x = std::clamp(0.40 * goal_distance, v_min, nominal.linear.x);
  }
  nominal.angular.z = std::clamp(0.45 * heading, -w_max, w_max);

  constexpr std::size_t horizon = 16;
  constexpr std::size_t samples = 28;
  constexpr double dt = 0.25;
  constexpr double temperature = 0.8;
  using Sequence = std::array<geometry_msgs::msg::Twist, horizon>;
  Sequence baseline;
  baseline.fill(nominal);
  std::array<geometry_msgs::msg::Twist, samples> sampled_first{};
  std::array<double, samples> cost{};
  cost.fill(std::numeric_limits<double>::infinity());
  std::mt19937 random(74);
  std::normal_distribution<double> linear_noise(0.0, 0.14);
  std::normal_distribution<double> angular_noise(0.0, 0.07);
  for (std::size_t sample = 0; sample < samples; ++sample) {
    auto state = current;
    auto previous = applied;
    double total = 0.0;
    double linear_offset = 0.0;
    double angular_offset = 0.0;
    bool feasible = true;
    for (std::size_t i = 0; i < horizon; ++i) {
      auto proposal = baseline[i];
      if (sample > 0) {
        linear_offset = 0.70 * linear_offset + linear_noise(random);
        angular_offset = 0.70 * angular_offset + angular_noise(random);
        proposal.linear.x += linear_offset;
        proposal.angular.z += angular_offset;
      }
      const auto control = cruiseTarget(proposal);
      if (i == 0) {
        sampled_first[sample] = control;
      }
      state = advance(state, control, dt, traction);
      if (!scene.traversable(state)) {
        feasible = false;
        break;
      }
      const auto closest = nearest(path, state);
      const auto & reference = path.poses[closest].pose;
      const auto & lead = path.poses[lookahead(path, closest, 1.6)].pose.position;
      const double yaw_error = wrap(std::atan2(lead.y - state.y, lead.x - state.x) - state.yaw);
      const double tracking = std::hypot(reference.position.x - state.x, reference.position.y - state.y);
      double ground = 0.0;
      double terrain_slope = 0.0;
      scene.ground(state.x, state.y, ground, terrain_slope);
      total += dt * (2.2 * tracking * tracking + 1.5 * (1.0 - std::cos(yaw_error)) +
        0.7 * terrain_slope / max_slope + 0.25 * std::abs(control.angular.z) / w_max -
        1.1 * state.v / v_max + 0.45 * std::abs(control.angular.z - previous.angular.z) / w_max);
      previous = control;
    }
    if (feasible) {
      cost[sample] = total;
    }
  }
  const auto best = std::min_element(cost.begin(), cost.end());
  if (!std::isfinite(*best)) {
    return stopped("MPPI 无可行采样轨迹");
  }
  geometry_msgs::msg::Twist mean;
  double total_weight = 0.0;
  for (std::size_t i = 0; i < samples; ++i) {
    const double weight = std::exp(-(cost[i] - *best) / temperature);
    mean.linear.x += weight * sampled_first[i].linear.x;
    mean.angular.z += weight * sampled_first[i].angular.z;
    total_weight += weight;
  }
  mean.linear.x /= total_weight;
  mean.angular.z /= total_weight;
  const auto command = shield(scene, current, applied, mean,
      sampled_first[static_cast<std::size_t>(best - cost.begin())]);
  return command.linear.x > 0.0 ? command : stopped("MPPI 制动扫掠不可行或未生成前进指令");
}

Observation Mppi::observation(
  const LocalScene & scene, const geometry_msgs::msg::Pose & pose,
  const geometry_msgs::msg::Twist & velocity, const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Twist & applied, bool feedback_valid) const
{
  Observation obs{};
  if (path.poses.empty()) {
    return obs;
  }
  const auto current = state(pose, velocity);
  const auto first = nearest(path, current);
  const auto & lead = path.poses[lookahead(path, first, 2.5)].pose;
  const double cosine = std::cos(current.yaw);
  const double sine = std::sin(current.yaw);
  const double dx = lead.position.x - current.x;
  const double dy = lead.position.y - current.y;
  obs[0] = scale(cosine * dx + sine * dy, 12.0);
  obs[1] = scale(-sine * dx + cosine * dy, 12.0);
  const double yaw_error = wrap(tf2::getYaw(lead.orientation) - current.yaw);
  obs[2] = static_cast<float>(std::sin(yaw_error));
  obs[3] = static_cast<float>(std::cos(yaw_error));
  obs[4] = scale(current.v, v_max);
  obs[5] = scale(current.w, w_max);
  obs[6] = scale(applied.linear.x, v_max);
  obs[7] = scale(applied.angular.z, w_max);
  for (int i = 0; i < 8; ++i) {
    obs[8 + i] = static_cast<float>(scene.range(current, i * pi / 4.0) / 12.0);
  }
  const double gx = path.poses.back().pose.position.x - current.x;
  const double gy = path.poses.back().pose.position.y - current.y;
  obs[16] = scale(cosine * gx + sine * gy, 20.0);
  obs[17] = scale(-sine * gx + cosine * gy, 20.0);
  obs[18] = scale(std::hypot(gx, gy), 20.0);
  obs[19] = 1.0f;
  obs[20] = feedback_valid ? scale(applied.linear.x - current.v, v_max) : 0.0f;
  obs[21] = feedback_valid ? scale(applied.angular.z - current.w, w_max) : 0.0f;
  obs[22] = scale(velocity.linear.y, v_max);
  tf2::Quaternion rotation;
  tf2::fromMsg(pose.orientation, rotation);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
  obs[23] = scale(roll, max_slope);
  obs[24] = scale(pitch, max_slope);
  double ground = 0.0;
  double slope = 0.0;
  if (scene.ground(current.x, current.y, ground, slope)) {
    obs[25] = scale(slope, max_slope);
    double next = 0.0;
    if (scene.ground(current.x + cosine, current.y + sine, next, slope)) {
      obs[26] = scale(next - ground, max_step);
    }
  }
  obs[27] = feedback_valid ? 1.0f : 0.0f;
  return obs;
}

}  // namespace clearing_robot
