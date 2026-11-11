#pragma once
#include "clearing_robot_plugins/model.hpp"
#include <grid_map_msgs/msg/grid_map.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <octomap_msgs/msg/octomap.hpp>

namespace clearing_robot_plugins
{
// Immutable collision geometry decoded from the selected framework map topic.
// OctoMap keeps each leaf's 3D size. Planar maps retain occupied/unknown columns.
class MapGeometry
{
public:
  explicit MapGeometry(bool planar = false) : planar_(planar) {}
  static std::shared_ptr<MapGeometry> fromOctomap(const octomap_msgs::msg::Octomap &,
                                                  const TerrainMap &);
  static std::shared_ptr<MapGeometry> fromGridmap(const grid_map_msgs::msg::GridMap &);
  static std::shared_ptr<MapGeometry> fromCostmap(const nav_msgs::msg::OccupancyGrid &);
  static std::shared_ptr<MapGeometry> fromFile(const TerrainMap &, const std::string &mode,
                                               double resolution, double robot_height);
  static uint64_t fingerprint(const octomap_msgs::msg::Octomap &);
  static uint64_t fingerprint(const grid_map_msgs::msg::GridMap &);
  static uint64_t fingerprint(const nav_msgs::msg::OccupancyGrid &);
  bool collides(const RobotModel &, const Pose &, double margin) const;
  size_t size() const { return centers_->size(); }

private:
  void add(const Eigen::Vector3d &center, const Eigen::Vector3d &half_size);
  void finish();
  bool planar_;
  double largest_half_diagonal_{0};
  Cloud::Ptr centers_{new Cloud};
  std::vector<Eigen::Vector3d> half_sizes_;
  pcl::KdTreeFLANN<Point> tree_;
};
} // namespace clearing_robot_plugins
