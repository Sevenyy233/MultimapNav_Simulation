#include "clearing_robot_plugins/costmap2d_adapter.hpp"
#include "clearing_robot_plugins/map_adapter_utils.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.hpp>

namespace clearing_robot_plugins
{
void Costmap2DAdapter::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                                 std::string name)
{
  auto n = parent.lock();
  height_ = parameter(n, "scene.robot_height", 4.9);
  support_ = std::make_shared<TerrainMap>(parameter(n, "scene.map_file", std::string("")),
                                          parameter(n, "scene.support_file", std::string("")),
                                          parameter(n, "scene.terrain_resolution", .5));
  loader_ = std::make_unique<pluginlib::ClassLoader<multimap_nav_core::MapPluginBase>>(
      "multimap_nav_core", "multimap_nav_core::MapPluginBase");
  delegate_ = loader_->createSharedInstance("multimap_nav_plugins::Costmap2DMapPlugin");
  delegate_->configure(parent, std::move(name));
}
void Costmap2DAdapter::updateMapFromCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  Cloud raw;
  pcl::fromROSMsg(*msg, raw);
  Cloud filtered = terrainBand(raw, *support_, height_);
  if (filtered.empty())
    throw std::runtime_error("Costmap2D has no usable near-ground points");
  auto output = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(filtered, *output);
  output->header = msg->header;
  delegate_->updateMapFromCloud(output);
}
void Costmap2DAdapter::cleanup()
{
  if (delegate_)
    delegate_->cleanup();
  delegate_.reset();
  loader_.reset();
  support_.reset();
}
} // namespace clearing_robot_plugins
PLUGINLIB_EXPORT_CLASS(clearing_robot_plugins::Costmap2DAdapter, multimap_nav_core::MapPluginBase)
