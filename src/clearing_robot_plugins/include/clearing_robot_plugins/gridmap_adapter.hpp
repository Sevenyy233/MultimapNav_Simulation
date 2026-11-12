#pragma once
#include "clearing_robot_plugins/scene.hpp"
#include <multimap_nav_core/map_plugin_base.hpp>
#include <pluginlib/class_loader.hpp>

namespace clearing_robot_plugins
{
// Keep the framework GridMap representation, removing points below the actual
// support surface so floor thickness cannot be mistaken for a terrain step.
class GridMapAdapter : public multimap_nav_core::MapPluginBase
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
};
} // namespace clearing_robot_plugins
