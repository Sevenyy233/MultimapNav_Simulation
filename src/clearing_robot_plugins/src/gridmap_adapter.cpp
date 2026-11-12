#include "clearing_robot_plugins/gridmap_adapter.hpp"
#include "clearing_robot_plugins/map_adapter_utils.hpp"
#include <limits>
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.hpp>

namespace clearing_robot_plugins
{
void GridMapAdapter::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                               std::string name)
{
  auto n = parent.lock();
  support_ = std::make_shared<TerrainMap>(parameter(n, "scene.map_file", std::string("")),
                                          parameter(n, "scene.support_file", std::string("")),
                                          parameter(n, "scene.terrain_resolution", .5));
  loader_ = std::make_unique<pluginlib::ClassLoader<multimap_nav_core::MapPluginBase>>(
      "multimap_nav_core", "multimap_nav_core::MapPluginBase");
  delegate_ = loader_->createSharedInstance("multimap_nav_plugins::GridMapPlugin");
  delegate_->configure(parent, std::move(name));
}
void GridMapAdapter::updateMapFromCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  Cloud raw;
  pcl::fromROSMsg(*msg, raw);
  Cloud filtered = terrainBand(raw, *support_, std::numeric_limits<double>::infinity());
  if (filtered.empty())
    throw std::runtime_error("GridMap has no usable above-ground points");
  auto output = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(filtered, *output);
  output->header = msg->header;
  delegate_->updateMapFromCloud(output);
}
void GridMapAdapter::cleanup()
{
  if (delegate_)
    delegate_->cleanup();
  delegate_.reset();
  loader_.reset();
  support_.reset();
}
} // namespace clearing_robot_plugins
PLUGINLIB_EXPORT_CLASS(clearing_robot_plugins::GridMapAdapter, multimap_nav_core::MapPluginBase)
