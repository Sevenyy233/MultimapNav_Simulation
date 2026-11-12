#pragma once

#include <cmath>
#include <stdexcept>

#include <tf2/LinearMath/Transform.h>

namespace multimap_nav_simulation
{

struct PoseVelocity
{
  tf2::Vector3 linear{0.0, 0.0, 0.0};
  tf2::Vector3 angular{0.0, 0.0, 0.0};
  bool valid{false};
};

// Differentiate the estimated base pose, expressing twist in the current base
// frame as required by nav_msgs/Odometry. No commanded or physical truth input.
class PoseVelocityEstimator
{
public:
  explicit PoseVelocityEstimator(double filter_time_constant = 0.2, double max_interval = 0.5)
  : time_constant_(filter_time_constant), max_interval_(max_interval)
  {
    if (!std::isfinite(time_constant_) || time_constant_ < 0.0 ||
      !std::isfinite(max_interval_) || max_interval_ <= 0.0)
    {
      throw std::invalid_argument("Invalid pose velocity estimator parameters");
    }
  }

  PoseVelocity update(const tf2::Transform & pose, double stamp)
  {
    const double dt = stamp - previous_stamp_;
    if (!initialized_ || !std::isfinite(dt) || dt <= 0.0 || dt > max_interval_) {
      previous_pose_ = pose;
      previous_stamp_ = stamp;
      initialized_ = true;
      filtered_ = PoseVelocity{};
      return filtered_;
    }

    const auto world_linear = (pose.getOrigin() - previous_pose_.getOrigin()) / dt;
    auto delta = pose.getRotation() * previous_pose_.getRotation().inverse();
    delta.normalize();
    if (delta.w() < 0.0) {delta.setValue(-delta.x(), -delta.y(), -delta.z(), -delta.w());}
    const tf2::Vector3 vector(delta.x(), delta.y(), delta.z());
    const double sine = vector.length();
    const auto world_angular = sine > 1e-9 ?
      vector * (2.0 * std::atan2(sine, delta.w()) / (sine * dt)) : vector * (2.0 / dt);
    const auto world_to_base = pose.getBasis().transpose();
    const auto linear = world_to_base * world_linear;
    const auto angular = world_to_base * world_angular;
    const double alpha = time_constant_ == 0.0 ? 1.0 : -std::expm1(-dt / time_constant_);
    if (filtered_.valid) {
      filtered_.linear += (linear - filtered_.linear) * alpha;
      filtered_.angular += (angular - filtered_.angular) * alpha;
    } else {
      filtered_.linear = linear;
      filtered_.angular = angular;
      filtered_.valid = true;
    }
    previous_pose_ = pose;
    previous_stamp_ = stamp;
    return filtered_;
  }

private:
  double time_constant_;
  double max_interval_;
  bool initialized_{false};
  double previous_stamp_{0.0};
  tf2::Transform previous_pose_{tf2::Transform::getIdentity()};
  PoseVelocity filtered_;
};

}  // namespace multimap_nav_simulation
