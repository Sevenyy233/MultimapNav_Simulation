#pragma once
#include "clearing_robot_plugins/map_geometry.hpp"
#include "clearing_robot_plugins/model.hpp"
#include <mutex>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace clearing_robot_plugins
{
template <typename T, typename N>
T parameter(const std::shared_ptr<N> &node, const std::string &key, const T &value)
{
  if (!node->has_parameter(key))
  {
    node->template declare_parameter<T>(key, value);
  }
  return node->get_parameter(key).template get_value<T>();
}

struct LiveData
{
  Cloud::Ptr cloud{new Cloud};
  std::shared_ptr<pcl::KdTreeFLANN<Point>> tree{new pcl::KdTreeFLANN<Point>};
  double stamp{-1e9};
};
class Scene : public std::enable_shared_from_this<Scene>
{
public:
  using Node = rclcpp_lifecycle::LifecycleNode;
  static std::shared_ptr<Scene> create(const Node::WeakPtr &node);
  std::shared_ptr<const TerrainMap> map;
  RobotModel model;
  double margin{.20}, collision_resolution{.15}, max_pitch{.21}, max_roll{.02}, max_step{.12};
  std::shared_ptr<const LiveData> snapshot() const;
  bool safe(Pose p, const std::shared_ptr<const LiveData> &data, bool dynamic = true,
            double extra_margin = 0.0) const;
  bool swept(const Pose &a, const Pose &b, const std::shared_ptr<const LiveData> &data,
             bool dynamic = true, double extra_margin = 0.0) const;
  bool healthy() const;
  std::string healthReason() const;
  Pose currentPose() const;
  bool staticCollision(const Pose &, double extra_margin = 0.) const;

private:
  explicit Scene(const Node::WeakPtr &node);
  void connect();
  void cloud(const sensor_msgs::msg::PointCloud2 &cloud);
  void acceptMap(std::shared_ptr<const MapGeometry>, uint64_t signature);
  void publishMapStatus();
  bool sameMap(uint64_t signature) const;
  Node::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_sub_, sensor_sub_;
  mutable std::mutex mutex_;
  std::shared_ptr<const LiveData> live_{new LiveData};
  bool localization_ok_{false}, sensors_ok_{false}, arm_ok_{false};
  double health_time_{-1e9}, sensor_time_{-1e9}, joint_time_{-1e9};
  double timeout_{1.0}, arm_tolerance_{.06};
  std::string map_mode_, map_source_, map_topic_;
  uint64_t map_signature_{0};
  std::shared_ptr<const MapGeometry> map_geometry_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr map_status_pub_;
};
} // namespace clearing_robot_plugins
