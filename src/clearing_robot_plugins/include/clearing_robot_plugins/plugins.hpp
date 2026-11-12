#pragma once
// Compatibility umbrella; each planning plugin has its own header and source.
#include "clearing_robot_plugins/hybrid_local_planner.hpp"
#include "clearing_robot_plugins/mppi_trajectory_planner.hpp"
#include "clearing_robot_plugins/paper_global_planner.hpp"
#include <multimap_nav_core/localization_base.hpp>
#include <multimap_nav_core/recovery_behavior_base.hpp>
namespace clearing_robot_plugins
{
class MapCorrectedFastLio : public multimap_nav_core::LocalizationBase
{
public:
  MapCorrectedFastLio();
  ~MapCorrectedFastLio() override;
  void configure(const Node::WeakPtr &, std::string) override;
  void activate() override;
  void deactivate() override;
  void cleanup() override;
  void shutdown() override;
  void setInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped &) override;
  bool getCurrentPose(geometry_msgs::msg::PoseWithCovarianceStamped &) const override;
  bool getCurrentOdometry(nav_msgs::msg::Odometry &) const override;
  bool isLocalized() const override;
  double getHealthScore() const override;

private:
  struct Impl;
  std::shared_ptr<Impl> state_;
};
class SafeRecovery : public multimap_nav_core::RecoveryBehaviorBase
{
public:
  void configure(const Node::WeakPtr &, std::string) override;
  void activate() override;
  void deactivate() override;
  void cleanup() override;
  void shutdown() override;
  void onRun() override;
  multimap_nav_core::RecoveryStatus onCycleUpdate() override;
  void onCancel() override;
  multimap_nav_core::RecoveryRequirements requirements() const override;

protected:
  bool spin_{false};

private:
  Node::WeakPtr node_;
  std::shared_ptr<Scene> scene_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  Pose start_;
  double target_{.6}, speed_{.1}, started_{0}, direction_{1};
  bool running_{false};
};
class BackUp : public SafeRecovery
{
};
class Spin : public SafeRecovery
{
public:
  void configure(const Node::WeakPtr &p, std::string n) override
  {
    spin_ = true;
    SafeRecovery::configure(p, n);
  }
};
} // namespace clearing_robot_plugins
