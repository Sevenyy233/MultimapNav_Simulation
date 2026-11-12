#include "clearing_robot_plugins/map_geometry.hpp"
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <set>
#include <stdexcept>

namespace clearing_robot_plugins
{
namespace
{
bool intersects(const Eigen::Isometry3d &a, const Eigen::Vector3d &ah, const Eigen::Vector3d &bc,
                const Eigen::Vector3d &bh)
{
  const Eigen::Matrix3d r = a.linear().transpose();
  const Eigen::Matrix3d ar = r.cwiseAbs().array() + 1e-9;
  const Eigen::Vector3d t = r * (bc - a.translation());
  for (int i = 0; i < 3; ++i)
    if (std::abs(t[i]) > ah[i] + ar.row(i).dot(bh))
      return false;
  for (int j = 0; j < 3; ++j)
    if (std::abs(t.dot(r.col(j))) > bh[j] + ah.dot(ar.col(j)))
      return false;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
    {
      const int u = (i + 1) % 3, v = (i + 2) % 3;
      const int b = (j + 1) % 3, c = (j + 2) % 3;
      if (std::abs(t[v] * r(u, j) - t[u] * r(v, j)) >
          ah[u] * ar(v, j) + ah[v] * ar(u, j) + bh[b] * ar(i, c) + bh[c] * ar(i, b))
        return false;
    }
  return true;
}
uint64_t bytes(uint64_t h, const void *data, size_t size)
{
  const auto *p = static_cast<const unsigned char *>(data);
  for (size_t i = 0; i < size; ++i)
    h = (h ^ p[i]) * 1099511628211ULL;
  return h;
}
template <typename T> uint64_t value(uint64_t h, const T &v) { return bytes(h, &v, sizeof(v)); }
uint64_t poseHash(uint64_t h, const geometry_msgs::msg::Pose &p)
{
  for (double v : {p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y,
                   p.orientation.z, p.orientation.w})
    h = value(h, v);
  return h;
}
constexpr uint64_t seed = 1469598103934665603ULL;
} // namespace

void MapGeometry::add(const Eigen::Vector3d &center, const Eigen::Vector3d &half)
{
  Point p;
  p.x = center.x();
  p.y = center.y();
  p.z = planar_ ? 0. : center.z();
  centers_->push_back(p);
  half_sizes_.push_back(half);
  largest_half_diagonal_ =
      std::max(largest_half_diagonal_, planar_ ? half.head<2>().norm() : half.norm());
}
void MapGeometry::finish()
{
  if (!centers_->empty())
    tree_.setInputCloud(centers_);
}
std::shared_ptr<MapGeometry> MapGeometry::fromOctomap(const octomap_msgs::msg::Octomap &msg,
                                                      const TerrainMap &support)
{
  std::unique_ptr<octomap::AbstractOcTree> raw(octomap_msgs::msgToMap(msg));
  auto *tree = dynamic_cast<octomap::OcTree *>(raw.get());
  if (!tree)
    throw std::runtime_error("Selected OctoMap must contain an OcTree");
  auto result = std::make_shared<MapGeometry>();
  for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it)
  {
    if (!tree->isNodeOccupied(*it))
      continue;
    Eigen::Vector3d p(it.getX(), it.getY(), it.getZ());
    const double half = it.getSize() / 2.;
    // Ground is handled by chassis support/slope constraints. Keep occupied
    // volume above it, including large/pruned leaves rather than point centers.
    if (support.ground(p + Eigen::Vector3d(0, 0, half), .15))
      continue;
    result->add(p, Eigen::Vector3d::Constant(half));
  }
  result->finish();
  return result;
}
std::shared_ptr<MapGeometry> MapGeometry::fromGridmap(const grid_map_msgs::msg::GridMap &msg)
{
  grid_map::GridMap grid;
  if (!grid_map::GridMapRosConverter::fromMessage(msg, grid) || !grid.exists("obstacle"))
    throw std::runtime_error("Selected GridMap lacks the obstacle layer");
  auto result = std::make_shared<MapGeometry>(true);
  const double half = grid.getResolution() / 2.;
  for (grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it)
  {
    const float occupied = grid.at("obstacle", *it);
    if (std::isfinite(occupied) && occupied < .5)
      continue;
    grid_map::Position p;
    if (grid.getPosition(*it, p))
      result->add({p.x(), p.y(), 0}, {half, half, 10000});
  }
  result->finish();
  return result;
}
std::shared_ptr<MapGeometry> MapGeometry::fromCostmap(const nav_msgs::msg::OccupancyGrid &msg)
{
  if (msg.info.resolution <= 0 || msg.data.size() != size_t(msg.info.width) * msg.info.height)
    throw std::runtime_error("Malformed selected Costmap2D");
  auto result = std::make_shared<MapGeometry>(true);
  const double r = msg.info.resolution, angle = yawOf(msg.info.origin.orientation);
  const double c = std::cos(angle), s = std::sin(angle);
  if (std::abs(angle) > 1e-6)
    throw std::runtime_error("Selected Costmap2D must be axis aligned in map");
  for (size_t k = 0; k < msg.data.size(); ++k)
  {
    if (msg.data[k] >= 0 && msg.data[k] < 50)
      continue;
    const double x = (k % msg.info.width + .5) * r, y = (k / msg.info.width + .5) * r;
    result->add(
        {msg.info.origin.position.x + c * x - s * y, msg.info.origin.position.y + s * x + c * y, 0},
        {r / 2, r / 2, 10000});
  }
  result->finish();
  return result;
}
std::shared_ptr<MapGeometry> MapGeometry::fromFile(const TerrainMap &map, const std::string &mode,
                                                   double r, double height)
{
  if (mode == "octomap")
  {
    octomap::OcTree tree(r);
    for (const auto &p : *map.obstacles)
      tree.updateNode(octomap::point3d(p.x, p.y, float(p.z - r / 2)), true);
    octomap_msgs::msg::Octomap msg;
    if (!octomap_msgs::binaryMapToMsg(tree, msg))
      throw std::runtime_error("Cannot encode file map");
    return fromOctomap(msg, map);
  }
  auto result = std::make_shared<MapGeometry>(true);
  std::set<std::pair<int, int>> cells;
  for (const auto &p : *map.obstacles)
  {
    double z;
    if (map.height(p.x, p.y, z) && p.z - z > height)
      continue;
    cells.emplace(int(std::floor(p.x / r)), int(std::floor(p.y / r)));
  }
  for (const auto &cell : cells)
    result->add({(cell.first + .5) * r, (cell.second + .5) * r, 0}, {r / 2, r / 2, 10000});
  result->finish();
  return result;
}
bool MapGeometry::collides(const RobotModel &model, const Pose &pose, double margin) const
{
  if (centers_->empty())
    return false;
  std::vector<int> ids;
  std::vector<float> distances;
  const auto base = pose.transform();
  for (const auto &box : model.boxes)
  {
    const Eigen::Isometry3d t = base * box.pose;
    const Eigen::Vector3d half = box.size / 2 + Eigen::Vector3d::Constant(margin);
    Point q;
    q.x = t.translation().x();
    q.y = t.translation().y();
    q.z = planar_ ? 0. : t.translation().z();
    const Eigen::Vector3d extent = t.linear().cwiseAbs() * half;
    const double radius =
        (planar_ ? extent.head<2>().norm() : half.norm()) + largest_half_diagonal_;
    tree_.radiusSearch(q, radius, ids, distances);
    for (int index : ids)
    {
      const auto &p = (*centers_)[index];
      const Eigen::Vector3d center(p.x, p.y, planar_ ? t.translation().z() : p.z);
      if (intersects(t, half, center, half_sizes_[index]))
        return true;
    }
  }
  return false;
}
uint64_t MapGeometry::fingerprint(const octomap_msgs::msg::Octomap &m)
{
  return bytes(value(value(seed, m.resolution), m.binary), m.data.data(), m.data.size());
}
uint64_t MapGeometry::fingerprint(const nav_msgs::msg::OccupancyGrid &m)
{
  auto h = poseHash(value(value(value(seed, m.info.resolution), m.info.width), m.info.height),
                    m.info.origin);
  return bytes(h, m.data.data(), m.data.size());
}
uint64_t MapGeometry::fingerprint(const grid_map_msgs::msg::GridMap &m)
{
  auto h = poseHash(value(value(value(seed, m.info.resolution), m.info.length_x), m.info.length_y),
                    m.info.pose);
  h = value(value(h, m.outer_start_index), m.inner_start_index);
  for (const auto &layer : m.layers)
    h = bytes(h, layer.data(), layer.size());
  for (const auto &layer : m.data)
    h = bytes(h, layer.data.data(), layer.data.size() * sizeof(float));
  return h;
}
} // namespace clearing_robot_plugins
