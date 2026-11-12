#pragma once
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

namespace clearing_robot_plugins
{
using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
struct Pose
{
  double x{0}, y{0}, z{0}, roll{0}, pitch{0}, yaw{0};
  Eigen::Isometry3d transform() const;
};
double wrap(double angle);
double yawOf(const geometry_msgs::msg::Quaternion &q);
Pose fromMsg(const geometry_msgs::msg::Pose &p);
geometry_msgs::msg::Pose toMsg(const Pose &p);
struct Box
{
  std::string name;
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d size{Eigen::Vector3d::Ones()};
};
class RobotModel
{
public:
  explicit RobotModel(const std::string &path);
  RobotModel() = default;
  std::vector<Box> boxes;
  std::vector<double> stow;
  double radius{0};
  double mass{29208.0}, yaw_inertia{106681.18};
  bool contains(const Eigen::Vector3d &point, double margin = 0) const;
  bool collides(const Pose &, const Cloud &, const pcl::KdTreeFLANN<Point> &, double margin) const;
  visualization_msgs::msg::MarkerArray markers(const std::string &frame, double margin = 0) const;
};
// The paper assumes one elevation layer. Obstacles retain their full 3D shape.
class TerrainMap
{
public:
  TerrainMap(const std::string &cloud, const std::string &support, double resolution,
             double clearance = .15);
  TerrainMap(int width, int height, double resolution, double x0, double y0);
  int width{0}, height_cells{0};
  double resolution{.5}, x0{0}, y0{0};
  std::vector<double> elevations;
  Cloud::Ptr obstacles{new Cloud};
  std::shared_ptr<pcl::KdTreeFLANN<Point>> obstacle_tree;
  bool height(double x, double y, double &z) const;
  bool project(Pose &, double max_pitch, double max_roll, double max_step) const;
  int index(double x, double y) const;
  Eigen::Vector2d center(int index) const;
  bool ground(const Eigen::Vector3d &point, double clearance) const;
};
} // namespace clearing_robot_plugins
