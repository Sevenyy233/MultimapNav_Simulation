#include "clearing_robot_plugins/scene.hpp"
#include <algorithm>
#include <cmath>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stdexcept>
#include <tf2_eigen/tf2_eigen.hpp>

namespace clearing_robot_plugins
{
std::shared_ptr<Scene> Scene::create(const Node::WeakPtr &node)
{
  auto scene = std::shared_ptr<Scene>(new Scene(node));
  scene->connect();
  return scene;
}
Scene::Scene(const Node::WeakPtr &parent) : node_(parent)
{
  auto node = parent.lock();
  if (!node)
  {
    throw std::runtime_error("Missing scene parent");
  }
  model = RobotModel(parameter(node, "scene.robot_model_file", std::string("")));
  margin = parameter(node, "scene.collision_margin", .20);
  collision_resolution = parameter(node, "scene.collision_resolution", .15);
  max_pitch = parameter(node, "scene.max_pitch", .21);
  max_roll = parameter(node, "scene.max_roll", .02);
  max_step = parameter(node, "scene.max_step", .12);
  timeout_ = parameter(node, "scene.sensor_timeout", 1.0);
  arm_tolerance_ = parameter(node, "scene.arm_tolerance", .06);
  double resolution = parameter(node, "scene.terrain_resolution", .5);
  if (margin < 0 || collision_resolution <= 0 || resolution <= 0 || timeout_ <= 0 ||
      max_pitch <= 0 || max_roll <= 0 || max_step < 0 || arm_tolerance_ <= 0)
  {
    throw std::runtime_error("Invalid scene parameters");
  }
  for (double value : {margin, collision_resolution, resolution, timeout_, max_pitch, max_roll,
                       max_step, arm_tolerance_})
  {
    if (!std::isfinite(value))
      throw std::runtime_error("Nonfinite scene parameter");
  }
  map = std::make_shared<TerrainMap>(parameter(node, "scene.map_file", std::string("")),
                                     parameter(node, "scene.support_file", std::string("")),
                                     resolution);
  map_source_ = parameter(node, "scene.map_source", std::string("topic"));
  map_mode_ = parameter(node, "scene.map_mode", std::string("auto"));
  if (map_mode_ == "auto")
  {
    for (const std::string key :
         {"planner_plugin_namespace", "local_planner_plugin_namespace",
          "trajectory_planner_plugin_namespace", "recovery_plugin_namespace"})
    {
      if (!node->has_parameter(key))
        continue;
      const std::string prefix = node->get_parameter(key).as_string();
      const std::string selected = parameter(node, prefix + ".map_mode", std::string(""));
      if (!selected.empty())
      {
        map_mode_ = selected;
        break;
      }
    }
  }
  if (map_mode_ != "octomap" && map_mode_ != "gridmap" && map_mode_ != "costmap2d")
    throw std::runtime_error("Scene requires the selected clearing map profile before configure");
  map_topic_ =
      map_mode_ == "octomap" ? "/octomap_map" : (map_mode_ == "gridmap" ? "/grid_map" : "/costmap");
  if (map_source_ == "file")
    map_geometry_ =
        MapGeometry::fromFile(*map, map_mode_, parameter(node, "scene.occupancy_resolution", .2),
                              parameter(node, "scene.robot_height", 4.9));
  else if (map_source_ != "topic")
    throw std::runtime_error("scene.map_source must be topic or file");
  buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
}
void Scene::connect()
{
  auto node = node_.lock();
  auto weak = weak_from_this();
  map_status_pub_ = node->create_publisher<std_msgs::msg::String>(
      "/clearing/map_backend", rclcpp::QoS(1).reliable().transient_local());
  map_status_pub_->on_activate();
  publishMapStatus();
  if (map_source_ == "topic")
  {
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    if (map_mode_ == "octomap")
      octomap_sub_ = node->create_subscription<octomap_msgs::msg::Octomap>(
          map_topic_, qos,
          [weak](octomap_msgs::msg::Octomap::ConstSharedPtr m)
          {
            if (auto self = weak.lock())
            {
              const auto signature = MapGeometry::fingerprint(*m);
              if (m->header.frame_id != "map" || self->sameMap(signature))
                return;
              try
              {
                self->acceptMap(MapGeometry::fromOctomap(*m, *self->map), signature);
              }
              catch (const std::exception &e)
              {
                if (auto n = self->node_.lock())
                  RCLCPP_ERROR(n->get_logger(), "OctoMap collision data rejected: %s", e.what());
              }
            }
          });
    else if (map_mode_ == "gridmap")
      gridmap_sub_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
          map_topic_, qos,
          [weak](grid_map_msgs::msg::GridMap::ConstSharedPtr m)
          {
            if (auto self = weak.lock())
            {
              const auto signature = MapGeometry::fingerprint(*m);
              if (m->header.frame_id != "map" || self->sameMap(signature))
                return;
              try
              {
                self->acceptMap(MapGeometry::fromGridmap(*m), signature);
              }
              catch (const std::exception &e)
              {
                if (auto n = self->node_.lock())
                  RCLCPP_ERROR(n->get_logger(), "GridMap collision data rejected: %s", e.what());
              }
            }
          });
    else
      costmap_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
          map_topic_, qos,
          [weak](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m)
          {
            if (auto self = weak.lock())
            {
              const auto signature = MapGeometry::fingerprint(*m);
              if (m->header.frame_id != "map" || self->sameMap(signature))
                return;
              try
              {
                self->acceptMap(MapGeometry::fromCostmap(*m), signature);
              }
              catch (const std::exception &e)
              {
                if (auto n = self->node_.lock())
                  RCLCPP_ERROR(n->get_logger(), "Costmap2D collision data rejected: %s", e.what());
              }
            }
          });
  }
  cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/clearing/obstacles", rclcpp::SensorDataQoS().keep_last(1),
      [weak](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
      {
        if (auto self = weak.lock())
        {
          self->cloud(*msg);
        }
      });
  health_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      "/clearing/localization_healthy", 10,
      [weak](std_msgs::msg::Bool::ConstSharedPtr msg)
      {
        if (auto self = weak.lock())
        {
          auto n = self->node_.lock();
          if (!n)
          {
            return;
          }
          std::lock_guard<std::mutex> lock(self->mutex_);
          self->localization_ok_ = msg->data;
          self->health_time_ = n->now().seconds();
        }
      });
  sensor_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      "/clearing/sensors_healthy", 10,
      [weak](std_msgs::msg::Bool::ConstSharedPtr msg)
      {
        if (auto self = weak.lock())
        {
          auto n = self->node_.lock();
          if (!n)
          {
            return;
          }
          std::lock_guard<std::mutex> lock(self->mutex_);
          self->sensors_ok_ = msg->data;
          self->sensor_time_ = n->now().seconds();
        }
      });
  joint_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [weak](sensor_msgs::msg::JointState::ConstSharedPtr msg)
      {
        if (auto self = weak.lock())
        {
          auto n = self->node_.lock();
          if (!n)
          {
            return;
          }
          bool good = true;
          for (size_t i = 0; i < self->model.stow.size(); ++i)
          {
            auto it =
                std::find(msg->name.begin(), msg->name.end(), "joint" + std::to_string(i + 1));
            size_t k = it - msg->name.begin();
            if (k >= msg->position.size() || !std::isfinite(msg->position[k]) ||
                std::abs(msg->position[k] - self->model.stow[i]) > self->arm_tolerance_)
            {
              good = false;
              break;
            }
          }
          std::lock_guard<std::mutex> lock(self->mutex_);
          self->arm_ok_ = good;
          self->joint_time_ = n->now().seconds();
        }
      });
}
void Scene::cloud(const sensor_msgs::msg::PointCloud2 &msg)
{
  if (msg.header.frame_id != "map")
  {
    return;
  }
  Cloud input;
  pcl::fromROSMsg(msg, input);
  auto next = std::make_shared<LiveData>();
  auto filtered = Cloud::Ptr(new Cloud);
  for (const auto &p : input)
  {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
    {
      continue;
    }
    filtered->push_back(p);
  }
  if (!filtered->empty())
  {
    pcl::VoxelGrid<Point> grid;
    grid.setInputCloud(filtered);
    grid.setLeafSize(.20, .20, .20);
    grid.filter(*next->cloud);
    next->tree->setInputCloud(next->cloud);
  }
  next->stamp = rclcpp::Time(msg.header.stamp).seconds();
  std::lock_guard<std::mutex> lock(mutex_);
  live_ = next;
}
std::shared_ptr<const LiveData> Scene::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return live_;
}
bool Scene::sameMap(uint64_t signature) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return map_geometry_ && signature == map_signature_;
}
void Scene::acceptMap(std::shared_ptr<const MapGeometry> geometry, uint64_t signature)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    map_geometry_ = std::move(geometry);
    map_signature_ = signature;
  }
  publishMapStatus();
}
void Scene::publishMapStatus()
{
  auto n = node_.lock();
  if (!n)
    return;
  std::lock_guard<std::mutex> lock(mutex_);
  std_msgs::msg::String status;
  status.data = std::string("{\"node\":\"") + n->get_name() + "\",\"mode\":\"" + map_mode_ +
                "\",\"source\":\"" + map_source_ + "\",\"topic\":\"" + map_topic_ +
                "\",\"ready\":" + (map_geometry_ ? "true" : "false") +
                ",\"occupied_cells\":" + std::to_string(map_geometry_ ? map_geometry_->size() : 0) +
                "}";
  map_status_pub_->publish(status);
  RCLCPP_INFO(n->get_logger(), "Clearing map backend: %s", status.data.c_str());
}
bool Scene::staticCollision(const Pose &p, double extra_margin) const
{
  std::shared_ptr<const MapGeometry> geometry;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    geometry = map_geometry_;
  }
  return !geometry || geometry->collides(model, p, margin + extra_margin) ||
         model.collides(p, *map->obstacles, *map->obstacle_tree, margin + extra_margin);
}
bool Scene::safe(Pose p, const std::shared_ptr<const LiveData> &data, bool dynamic,
                 double extra_margin) const
{
  if (!map->project(p, max_pitch, max_roll, max_step))
  {
    return false;
  }
  if (staticCollision(p, extra_margin))
  {
    return false;
  }
  return !dynamic || !data || !model.collides(p, *data->cloud, *data->tree, margin + extra_margin);
}
bool Scene::swept(const Pose &a, const Pose &b, const std::shared_ptr<const LiveData> &data,
                  bool dynamic, double extra_margin) const
{
  const double angle = wrap(b.yaw - a.yaw);
  const double distance = std::hypot(b.x - a.x, b.y - a.y) + model.radius * std::abs(angle);
  int count = std::max(1, int(std::ceil(distance / collision_resolution)));
  for (int i = 0; i <= count; ++i)
  {
    double t = double(i) / count;
    Pose p;
    p.x = a.x + (b.x - a.x) * t;
    p.y = a.y + (b.y - a.y) * t;
    p.z = a.z + (b.z - a.z) * t;
    p.yaw = wrap(a.yaw + angle * t);
    if (!safe(p, data, dynamic, extra_margin))
    {
      return false;
    }
  }
  return true;
}
std::string Scene::healthReason() const
{
  auto node = node_.lock();
  if (!node)
  {
    return "parent expired";
  }
  double now = node->now().seconds();
  std::lock_guard<std::mutex> lock(mutex_);
  auto fresh = [&](double stamp) { return now - stamp >= -.1 && now - stamp < timeout_; };
  if (!map_geometry_)
    return "selected " + map_mode_ + " map has not arrived on " + map_topic_;
  if (!localization_ok_ || !fresh(health_time_))
  {
    return "localization not healthy / heartbeat age " + std::to_string(now - health_time_);
  }
  if (!sensors_ok_ || !fresh(sensor_time_))
  {
    return "five-sensor coverage not fresh / heartbeat age " + std::to_string(now - sensor_time_);
  }
  if (!arm_ok_ || !fresh(joint_time_))
  {
    return "arm not stowed / joint age " + std::to_string(now - joint_time_);
  }
  if (!fresh(live_->stamp))
  {
    return "collision observation not fresh / cloud age " + std::to_string(now - live_->stamp);
  }
  return {};
}
bool Scene::healthy() const { return healthReason().empty(); }
Pose Scene::currentPose() const
{
  auto t = buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
  geometry_msgs::msg::Pose p;
  p.position.x = t.transform.translation.x;
  p.position.y = t.transform.translation.y;
  p.position.z = t.transform.translation.z;
  p.orientation = t.transform.rotation;
  return fromMsg(p);
}
} // namespace clearing_robot_plugins
