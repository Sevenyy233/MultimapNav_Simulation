#pragma once
#include "clearing_robot_plugins/planner_types.hpp"
#include <chrono>
namespace clearing_robot_plugins::detail
{
inline double distance(const Pose &a, const Pose &b)
{
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}
inline Path message(const std::vector<Pose> &points, const rclcpp::Time &now)
{
  Path result;
  result.header.frame_id = "map";
  result.header.stamp = now;
  for (const auto &p : points)
  {
    geometry_msgs::msg::PoseStamped m;
    m.header = result.header;
    m.pose = toMsg(p);
    result.poses.push_back(m);
  }
  return result;
}
inline std::vector<Pose> positions(const Path &p)
{
  std::vector<Pose> out;
  for (const auto &m : p.poses)
  {
    out.push_back(fromMsg(m.pose));
  }
  return out;
}
inline bool expired(const std::chrono::steady_clock::time_point &t, double seconds)
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count() > seconds;
}
} // namespace clearing_robot_plugins::detail
