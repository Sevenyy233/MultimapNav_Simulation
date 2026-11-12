#include "clearing_robot_plugins/plugins.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace clearing_robot_plugins
{
void SafeRecovery::configure(const Node::WeakPtr &parent, std::string name)
{
  node_ = parent;
  auto n = parent.lock();
  scene_ = Scene::create(parent);
  target_ = parameter(n, name + (spin_ ? ".angle" : ".distance"), spin_ ? .26 : .6);
  speed_ = parameter(n, name + ".speed", spin_ ? .06 : .10);
  if (target_ <= 0 || speed_ <= 0 || (spin_ && target_ > .52))
  {
    throw std::runtime_error("Invalid conservative recovery motion");
  }
  publisher_ = n->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}
void SafeRecovery::activate() { publisher_->on_activate(); }
void SafeRecovery::deactivate()
{
  onCancel();
  if (publisher_ && publisher_->is_activated())
  {
    publisher_->on_deactivate();
  }
}
void SafeRecovery::cleanup()
{
  deactivate();
  scene_.reset();
  publisher_.reset();
}
void SafeRecovery::shutdown() { cleanup(); }
multimap_nav_core::RecoveryRequirements SafeRecovery::requirements() const
{
  // The server's generic circular envelope is only a scene/freshness precheck.
  // This CUSTOM capability retains the full oriented 3D swept-box checks below.
  return {multimap_nav_core::RecoveryMotion::CUSTOM, 0., .15, .25};
}
void SafeRecovery::onRun()
{
  auto n = node_.lock();
  if (!scene_->healthy())
  {
    throw std::runtime_error("Recovery requires healthy localization and five sensors");
  }
  start_ = scene_->currentPose();
  direction_ = 1;
  Pose target = start_;
  if (spin_)
  {
    if (std::max(std::abs(start_.pitch), std::abs(start_.roll)) > .10)
    {
      throw std::runtime_error("No recovery spin on tilted ground");
    }
    target.yaw += target_;
  }
  else
  {
    target.x -= target_ * std::cos(start_.yaw);
    target.y -= target_ * std::sin(start_.yaw);
  }
  auto live = scene_->snapshot();
  if (!scene_->swept(start_, target, live))
  {
    if (!spin_)
    {
      throw std::runtime_error("Rear swept body corridor is blocked");
    }
    target.yaw = start_.yaw - target_;
    direction_ = -1;
    if (!scene_->swept(start_, target, live))
    {
      throw std::runtime_error("Both rotation swept volumes are blocked");
    }
  }
  started_ = n->now().seconds();
  running_ = true;
}
multimap_nav_core::RecoveryStatus SafeRecovery::onCycleUpdate()
{
  using Status = multimap_nav_core::RecoveryStatus;
  auto n = node_.lock();
  if (!running_ || !n || !scene_->healthy() || n->now().seconds() - started_ > 25)
  {
    onCancel();
    return Status::FAILURE;
  }
  Pose current;
  try
  {
    current = scene_->currentPose();
  }
  catch (const tf2::TransformException &)
  {
    onCancel();
    return Status::FAILURE;
  }
  double progress = spin_ ? direction_ * wrap(current.yaw - start_.yaw)
                          : -((current.x - start_.x) * std::cos(start_.yaw) +
                              (current.y - start_.y) * std::sin(start_.yaw));
  if (progress >= target_ - (spin_ ? .015 : .03))
  {
    onCancel();
    return Status::SUCCESS;
  }
  Pose target = current;
  double remaining = target_ - progress;
  if (spin_)
  {
    target.yaw += direction_ * remaining;
  }
  else
  {
    target.x -= remaining * std::cos(start_.yaw);
    target.y -= remaining * std::sin(start_.yaw);
  }
  if (!scene_->swept(current, target, scene_->snapshot()))
  {
    onCancel();
    return Status::FAILURE;
  }
  geometry_msgs::msg::Twist command;
  if (spin_)
  {
    command.angular.z = direction_ * std::min(speed_, std::max(.025, remaining * .5));
  }
  else
  {
    command.linear.x = -std::min(speed_, std::max(.025, remaining * .5));
  }
  publisher_->publish(command);
  return Status::RUNNING;
}
void SafeRecovery::onCancel()
{
  running_ = false;
  if (publisher_ && publisher_->is_activated())
  {
    publisher_->publish(geometry_msgs::msg::Twist{});
  }
}
} // namespace clearing_robot_plugins
PLUGINLIB_EXPORT_CLASS(clearing_robot_plugins::BackUp, multimap_nav_core::RecoveryBehaviorBase)
PLUGINLIB_EXPORT_CLASS(clearing_robot_plugins::Spin, multimap_nav_core::RecoveryBehaviorBase)
