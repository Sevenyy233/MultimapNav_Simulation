#pragma once
#include "clearing_robot_plugins/scene.hpp"
#include <multimap_nav_core/map_plugin_base.hpp>
#include <pluginlib/class_loader.hpp>

namespace clearing_robot_plugins
{
// Retain the framework's TMPU Costmap2D implementation while excluding ceilings
// from ground-plane fitting. Full original geometry remains in every Scene.
class Costmap2DAdapter : public multimap_nav_core::MapPluginBase
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, std::string) override;
  void activate() override { delegate_->activate(); }
  void deactivate() override { delegate_->deactivate(); }
  void cleanup() override;
  void shutdown() override { cleanup(); }
  void updateMapFromCloud(const sensor_msgs::msg::PointCloud2::SharedPtr) override;
  void updateMapFromOctomap(const octomap_msgs::msg::Octomap::SharedPtr m) override
  {
    delegate_->updateMapFromOctomap(m);
  }

private:
  std::unique_ptr<pluginlib::ClassLoader<multimap_nav_core::MapPluginBase>> loader_;
  multimap_nav_core::MapPluginBase::Ptr delegate_;
  std::shared_ptr<TerrainMap> support_;
  double height_{4.9};
};
} // namespace clearing_robot_plugins
