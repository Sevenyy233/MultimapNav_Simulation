#pragma once
#include "clearing_robot_plugins/scene.hpp"
namespace clearing_robot_plugins
{
struct MotionLimits
{
  double forward{.35}, reverse{.20}, yaw_rate{.12}, acceleration{.18}, yaw_acceleration{.08};
  double jerk{.4}, yaw_jerk{.15}, min_radius{2.5}, lateral_acceleration{.08};
  double physical_separation{2.8}, separation{3.276}, wheel_radius{.48}, wheel_speed{8.},
      side_force{37500.};
  double mass{29208.}, yaw_inertia{106681.18};
  template <class N> static MotionLimits load(const std::shared_ptr<N> &n, const RobotModel &model)
  {
    MotionLimits l;
    l.mass = model.mass;
    l.yaw_inertia = model.yaw_inertia;
    l.forward = parameter(n, "motion.max_forward_speed", l.forward);
    l.reverse = parameter(n, "motion.max_reverse_speed", l.reverse);
    l.yaw_rate = parameter(n, "motion.max_yaw_rate", l.yaw_rate);
    l.acceleration = parameter(n, "motion.max_acceleration", l.acceleration);
    l.yaw_acceleration = parameter(n, "motion.max_yaw_acceleration", l.yaw_acceleration);
    l.jerk = parameter(n, "motion.linear_jerk", l.jerk);
    l.yaw_jerk = parameter(n, "motion.angular_jerk", l.yaw_jerk);
    l.min_radius = parameter(n, "motion.min_turn_radius", l.min_radius);
    l.lateral_acceleration =
        parameter(n, "motion.max_lateral_acceleration", l.lateral_acceleration);
    for (double v : {l.forward, l.reverse, l.yaw_rate, l.acceleration, l.yaw_acceleration, l.jerk,
                     l.yaw_jerk, l.min_radius, l.lateral_acceleration})
      if (!std::isfinite(v) || v <= 0)
        throw std::runtime_error("Invalid chassis motion limit");
    if (l.mass * l.acceleration / 2 + l.yaw_inertia * l.yaw_acceleration / l.physical_separation >
        l.side_force)
      throw std::runtime_error(
          "Acceleration/yaw acceleration exceed conservative track force budget");
    return l;
  }
  bool admissible(double v, double w, double a, double aw, bool stationary_turn = false) const
  {
    if (!std::isfinite(v + w + a + aw) || v > forward + 1e-5 || v < -reverse - 1e-5 ||
        std::abs(w) > yaw_rate + 1e-5 || std::abs(a) > acceleration + 1e-5 ||
        std::abs(aw) > yaw_acceleration + 1e-5 || std::abs(v * w) > lateral_acceleration + 1e-5)
      return false;
    if (!stationary_turn && std::abs(w) > (std::abs(v) + .04) / min_radius + .01)
      return false;
    double force = mass * a / 2, moment = yaw_inertia * aw / physical_separation;
    return std::max(std::abs(force - moment), std::abs(force + moment)) <= side_force &&
           std::max(std::abs(v - w * separation / 2), std::abs(v + w * separation / 2)) /
                   wheel_radius <=
               wheel_speed;
  }
};
} // namespace clearing_robot_plugins
