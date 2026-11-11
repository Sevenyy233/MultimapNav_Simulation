#include "clearing_robot_plugins/model.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <pcl/io/pcd_io.h>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace clearing_robot_plugins
{
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
double yawOf(const geometry_msgs::msg::Quaternion &q)
{
  return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}
Eigen::Isometry3d Pose::transform() const
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(x, y, z);
  t.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
                   .toRotationMatrix();
  return t;
}
Pose fromMsg(const geometry_msgs::msg::Pose &m)
{
  Pose p;
  p.x = m.position.x;
  p.y = m.position.y;
  p.z = m.position.z;
  Eigen::Quaterniond q(m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z);
  if (q.norm() > 1e-8)
  {
    auto r = q.normalized().toRotationMatrix();
    p.pitch = std::asin(std::clamp(-r(2, 0), -1.0, 1.0));
    p.roll = std::atan2(r(2, 1), r(2, 2));
    p.yaw = std::atan2(r(1, 0), r(0, 0));
  }
  return p;
}
geometry_msgs::msg::Pose toMsg(const Pose &p)
{
  geometry_msgs::msg::Pose m;
  m.position.x = p.x;
  m.position.y = p.y;
  m.position.z = p.z;
  Eigen::Quaterniond q(p.transform().linear());
  m.orientation.x = q.x();
  m.orientation.y = q.y();
  m.orientation.z = q.z();
  m.orientation.w = q.w();
  return m;
}
RobotModel::RobotModel(const std::string &path)
{
  auto data = YAML::LoadFile(path);
  stow = data["stowed_joints"].as<std::vector<double>>();
  if (data["dynamics"])
  {
    mass = data["dynamics"]["mass_kg"].as<double>();
    yaw_inertia = data["dynamics"]["yaw_inertia_kg_m2"].as<double>();
    if (!std::isfinite(mass) || mass <= 0 || !std::isfinite(yaw_inertia) || yaw_inertia <= 0)
      throw std::runtime_error("Invalid stowed rigid-body mass/inertia");
  }
  if (stow.size() != 7)
  {
    throw std::runtime_error("Expected seven stowed arm joints");
  }
  for (const auto &item : data["boxes"])
  {
    Box b;
    b.name = item["name"].as<std::string>();
    auto c = item["center"].as<std::vector<double>>();
    auto s = item["size"].as<std::vector<double>>();
    auto q = item["quaternion"].as<std::vector<double>>();
    if (c.size() != 3 || s.size() != 3 || q.size() != 4)
    {
      throw std::runtime_error("Invalid collision box");
    }
    b.pose.translation() = Eigen::Vector3d(c[0], c[1], c[2]);
    b.size = Eigen::Vector3d(s[0], s[1], s[2]);
    Eigen::Quaterniond rotation(q[3], q[0], q[1], q[2]);
    if (!b.size.allFinite() || b.size.minCoeff() <= 0 || !b.pose.translation().allFinite() ||
        !rotation.coeffs().allFinite() || rotation.norm() < 1e-8)
    {
      throw std::runtime_error("Nonfinite collision geometry");
    }
    b.pose.linear() = rotation.normalized().toRotationMatrix();
    boxes.push_back(b);
    radius = std::max(radius, b.pose.translation().head<2>().norm() + b.size.norm() / 2);
  }
  if (boxes.empty())
  {
    throw std::runtime_error("Empty collision model");
  }
}
bool RobotModel::contains(const Eigen::Vector3d &p, double margin) const
{
  for (const auto &b : boxes)
  {
    auto local = b.pose.inverse() * p;
    if ((local.cwiseAbs().array() <= (b.size.array() / 2 + margin)).all())
    {
      return true;
    }
  }
  return false;
}
bool RobotModel::collides(const Pose &p, const Cloud &cloud, const pcl::KdTreeFLANN<Point> &tree,
                          double margin) const
{
  if (cloud.empty())
  {
    return false;
  }
  auto base = p.transform();
  std::vector<int> ids;
  std::vector<float> distances;
  for (const auto &b : boxes)
  {
    auto t = base * b.pose;
    Eigen::Vector3d half = b.size / 2 + Eigen::Vector3d::Constant(margin);
    Point c;
    c.x = t.translation().x();
    c.y = t.translation().y();
    c.z = t.translation().z();
    if (tree.radiusSearch(c, half.norm(), ids, distances) <= 0)
    {
      continue;
    }
    auto inverse = t.inverse();
    for (int i : ids)
    {
      const auto &point = cloud[i];
      auto local = inverse * Eigen::Vector3d(point.x, point.y, point.z);
      if ((local.cwiseAbs().array() <= half.array()).all())
      {
        return true;
      }
    }
  }
  return false;
}
visualization_msgs::msg::MarkerArray RobotModel::markers(const std::string &frame,
                                                         double margin) const
{
  visualization_msgs::msg::MarkerArray result;
  int id = 0;
  for (const auto &b : boxes)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame;
    m.ns = "collision_model";
    m.id = id++;
    m.type = m.CUBE;
    m.action = m.ADD;
    m.pose.position.x = b.pose.translation().x();
    m.pose.position.y = b.pose.translation().y();
    m.pose.position.z = b.pose.translation().z();
    Eigen::Quaterniond q(b.pose.linear());
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    m.pose.orientation.w = q.w();
    m.scale.x = b.size.x() + 2 * margin;
    m.scale.y = b.size.y() + 2 * margin;
    m.scale.z = b.size.z() + 2 * margin;
    m.color.r = id > 4 ? .95 : .15;
    m.color.g = id > 4 ? .55 : .65;
    m.color.b = id > 4 ? .12 : .85;
    m.color.a = .32;
    result.markers.push_back(m);
  }
  return result;
}
TerrainMap::TerrainMap(int w, int h, double r, double x, double y)
    : width(w), height_cells(h), resolution(r), x0(x), y0(y),
      elevations(w * h, std::numeric_limits<double>::quiet_NaN()),
      obstacle_tree(new pcl::KdTreeFLANN<Point>)
{
}
TerrainMap::TerrainMap(const std::string &path, const std::string &support, double r,
                       double clearance)
    : resolution(r)
{
  Cloud cloud, ground_cloud;
  if (pcl::io::loadPCDFile(path, cloud) < 0 || pcl::io::loadPCDFile(support, ground_cloud) < 0 ||
      ground_cloud.empty())
  {
    throw std::runtime_error("Cannot load navigation/support PCD");
  }
  double xmax = -1e9, ymax = -1e9;
  x0 = 1e9;
  y0 = 1e9;
  for (const auto &p : ground_cloud)
  {
    x0 = std::min(x0, double(p.x));
    y0 = std::min(y0, double(p.y));
    xmax = std::max(xmax, double(p.x));
    ymax = std::max(ymax, double(p.y));
  }
  x0 = std::floor(x0 / r) * r;
  y0 = std::floor(y0 / r) * r;
  width = int(std::ceil((xmax - x0) / r)) + 1;
  height_cells = int(std::ceil((ymax - y0) / r)) + 1;
  if (width <= 0 || height_cells <= 0 || int64_t(width) * height_cells > 10000000)
  {
    throw std::runtime_error("Invalid terrain extent");
  }
  elevations.assign(width * height_cells, std::numeric_limits<double>::quiet_NaN());
  for (const auto &p : ground_cloud)
  {
    int k = index(p.x, p.y);
    if (k < 0)
    {
      continue;
    }
    if (!std::isfinite(elevations[k]) || p.z < elevations[k])
    {
      elevations[k] = p.z;
    }
  }
  for (const auto &p : cloud)
  {
    if (!std::isfinite(p.x + p.y + p.z))
    {
      continue;
    }
    if (!ground(Eigen::Vector3d(p.x, p.y, p.z), clearance))
    {
      obstacles->push_back(p);
    }
  }
  obstacle_tree = std::make_shared<pcl::KdTreeFLANN<Point>>();
  if (!obstacles->empty())
  {
    obstacle_tree->setInputCloud(obstacles);
  }
}
int TerrainMap::index(double x, double y) const
{
  int ix = int(std::floor((x - x0) / resolution)), iy = int(std::floor((y - y0) / resolution));
  return ix < 0 || iy < 0 || ix >= width || iy >= height_cells ? -1 : iy * width + ix;
}
Eigen::Vector2d TerrainMap::center(int k) const
{
  return {x0 + (k % width + .5) * resolution, y0 + (k / width + .5) * resolution};
}
bool TerrainMap::height(double x, double y, double &z) const
{
  int k = index(x, y);
  if (k < 0 || !std::isfinite(elevations[k]))
  {
    return false;
  }
  z = elevations[k];
  return true;
}
bool TerrainMap::ground(const Eigen::Vector3d &p, double clearance) const
{
  double h;
  return height(p.x(), p.y(), h) && p.z() <= h + clearance;
}
bool TerrainMap::project(Pose &p, double max_pitch, double max_roll, double max_step) const
{
  double h, hf, hb, hl, hr;
  double c = std::cos(p.yaw), s = std::sin(p.yaw);
  if (!height(p.x, p.y, h) || !height(p.x + 2 * c, p.y + 2 * s, hf) ||
      !height(p.x - 2 * c, p.y - 2 * s, hb) || !height(p.x - 1.4 * s, p.y + 1.4 * c, hl) ||
      !height(p.x + 1.4 * s, p.y - 1.4 * c, hr))
  {
    return false;
  }
  p.z = h + .01;
  p.pitch = -std::atan2(hf - hb, 4.0);
  p.roll = std::atan2(hl - hr, 2.8);
  if (std::abs(p.pitch) > max_pitch || std::abs(p.roll) > max_roll)
  {
    return false;
  }
  for (double a : {-2., 0., 2.})
  {
    for (double b : {-1.4, 1.4})
    {
      double sample;
      if (!height(p.x + a * c - b * s, p.y + a * s + b * c, sample))
      {
        return false;
      }
      if (std::abs(sample - (h - a * std::tan(p.pitch) + b * std::tan(p.roll))) > max_step)
      {
        return false;
      }
    }
  }
  return true;
}
} // namespace clearing_robot_plugins
