#include "clearing_robot_plugins/model.hpp"
#include <array>
#include <deque>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <map>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace clearing_robot_plugins
{
class Perception : public rclcpp::Node
{
public:
  Perception()
      : Node("clearing_perception"), model_(declare_parameter("robot_model_file", std::string(""))),
        mapping_only_(declare_parameter("mapping_only", false)),
        map_(mapping_only_
                 ? nullptr
                 : std::make_unique<TerrainMap>(declare_parameter("map_file", std::string("")),
                                                declare_parameter("support_file", std::string("")),
                                                declare_parameter("scene.terrain_resolution", .5))),
        margin_(declare_parameter("scene.collision_margin", .20)),
        obstacle_keep_time_(declare_parameter("scene.dynamic_obstacle_keep_time", 10.0)),
        obstacle_keep_radius_(declare_parameter("scene.obstacle_keep_radius", model_.radius + 6.0)),
        neighbor_radius_(declare_parameter("scene.obstacle_neighbor_radius", .45)),
        min_neighbors_(declare_parameter("scene.obstacle_min_neighbors", 3))
  {
    if (!std::isfinite(margin_) || margin_ < 0 || !std::isfinite(obstacle_keep_time_) ||
        obstacle_keep_time_ < 3.0 || !std::isfinite(obstacle_keep_radius_) ||
        obstacle_keep_radius_ < model_.radius + 3.0 || !std::isfinite(neighbor_radius_) ||
        neighbor_radius_ <= 0 || min_neighbors_ < 1)
    {
      throw std::runtime_error("Invalid collision marker margin");
    }
    buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    filtered_primary_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(
        "/clearing/lidar_fused", rclcpp::SensorDataQoS().keep_last(2));
    for (int i = 0; i < 5; ++i)
    {
      primary_[i] = create_subscription<livox_ros_driver2::msg::CustomMsg>(
          "/livox/lidar_" + std::to_string(i + 1), rclcpp::SensorDataQoS().keep_last(2),
          [this, i](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr m)
          {
            raw_[i] = m;
            publishPrimary();
          });

      if (mapping_only_)
        continue;
      subscriptions_[i] = create_subscription<sensor_msgs::msg::PointCloud2>(
          "/livox/lidar_" + std::to_string(i + 1) + "_PointCloud2",
          rclcpp::SensorDataQoS().keep_last(3),
          [this, i](sensor_msgs::msg::PointCloud2::ConstSharedPtr m)
          {
            clouds_[i].push_back(m);
            while (clouds_[i].size() > 12)
            {
              clouds_[i].pop_front();
            }
          });
    }
    if (!mapping_only_)
    {
      odom_ = create_subscription<nav_msgs::msg::Odometry>(
          "/odom", rclcpp::SensorDataQoS(),
          [this](nav_msgs::msg::Odometry::ConstSharedPtr m)
          { odom_stamp_ = rclcpp::Time(m->header.stamp).seconds(); });
      output_ = create_publisher<sensor_msgs::msg::PointCloud2>("/clearing/points",
                                                                rclcpp::SensorDataQoS());
      obstacles_ = create_publisher<sensor_msgs::msg::PointCloud2>("/clearing/obstacles",
                                                                   rclcpp::SensorDataQoS());
      health_ = create_publisher<std_msgs::msg::Bool>("/clearing/sensors_healthy", 10);
    }
    markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/clearing/collision_model", rclcpp::QoS(1).transient_local());
    map_timer_=create_wall_timer(std::chrono::milliseconds(10),[this]{drainMapFrames();});
    timer_ =
        rclcpp::create_timer(get_node_base_interface(), get_node_timers_interface(), get_clock(),
                             rclcpp::Duration::from_seconds(.1), [this] { update(); });
  }

private:
  using Rays=std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>>;
  struct PendingFrame{Cloud cloud;std_msgs::msg::Header header;Rays rays;};
  std::deque<PendingFrame> pending_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  void publishPrimary()
  {
    double oldest = INFINITY, newest = -1;
    for (const auto &m : raw_)
    {
      if (!m)
        return;
      double t = rclcpp::Time(m->header.stamp).seconds();
      oldest = std::min(oldest, t);
      newest = std::max(newest, t);
    }
    if (newest - oldest > .05 || oldest <= primary_stamp_ + .00001)
      return;
    livox_ros_driver2::msg::CustomMsg output;
    output.header.frame_id = "imu_link";
    for (const auto &m : raw_)
      if (rclcpp::Time(m->header.stamp).seconds() >= newest - .00001)
        output.header.stamp = m->header.stamp;
    output.timebase = uint64_t(rclcpp::Time(output.header.stamp).nanoseconds());
    output.points.reserve(15000);
    size_t total = 0;
    for (size_t i = 0; i < raw_.size(); ++i)
    {
      const auto &m = raw_[i];
      Eigen::Isometry3d base_sensor, imu_sensor;
      try
      {
        base_sensor = tf2::transformToEigen(
            buffer_->lookupTransform("base_link", m->header.frame_id, tf2::TimePointZero)
                .transform);
        imu_sensor = tf2::transformToEigen(
            buffer_->lookupTransform("imu_link", m->header.frame_id, tf2::TimePointZero).transform);
      }
      catch (const tf2::TransformException &)
      {
        return;
      }
      total += m->points.size();
      for (const auto &point : m->points)
      {
        Eigen::Vector3d p(point.x, point.y, point.z);
        if (!p.allFinite() || p.norm() < .15 || p.norm() > 80 ||
            model_.contains(base_sensor * p, .10))
          continue;
        Eigen::Vector3d transformed = imu_sensor * p;
        auto result = point;
        result.x = transformed.x();
        result.y = transformed.y();
        result.z = transformed.z();
        result.line = uint8_t(i);
        result.offset_time = 0;
        output.points.push_back(result);
      }
    }
    output.point_num = uint32_t(output.points.size());
    if (output.point_num > 150)
    {
      filtered_primary_->publish(output);
      primary_stamp_ = oldest;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                         "FAST-LIO five-lidar IMU-frame input: %u / %zu points", output.point_num,
                         total);
  }

  void update()
  {
    markers_->publish(model_.markers("base_link", margin_));
    if (mapping_only_)
      return;
    std_msgs::msg::Bool health;
    health.data = true;
    for (const auto &queue : clouds_)
    {
      if (queue.empty())
      {
        health.data = false;
        break;
      }
      double age = now().seconds() - rclcpp::Time(queue.back()->header.stamp).seconds();
      if (age < -.1 || age > .7)
      {
        health.data = false;
        break;
      }
    }
    health_->publish(health);
    if (!health.data || odom_stamp_ < 0)
    {
      return;
    }
    geometry_msgs::msg::TransformStamped reference_message;
    try
    {
      reference_message = buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
    }
    catch (const tf2::TransformException &)
    {
      return;
    }
    std::array<sensor_msgs::msg::PointCloud2::ConstSharedPtr, 5> selected;
    double stamp = rclcpp::Time(reference_message.header.stamp).seconds();
    if (now().seconds() - stamp > .6)
    {
      return;
    }
    for (int i = 0; i < 5; ++i)
    {
      for (auto it = clouds_[i].rbegin(); it != clouds_[i].rend(); ++it)
      {
        if (rclcpp::Time((*it)->header.stamp).seconds() <= stamp + .002)
        {
          selected[i] = *it;
          break;
        }
      }
      if (!selected[i] || stamp - rclcpp::Time(selected[i]->header.stamp).seconds() > .3)
      {
        return;
      }
    }
    if (stamp <= published_)
    {
      return;
    }
    Eigen::Isometry3d reference = tf2::transformToEigen(reference_message.transform);
    auto combined = Cloud::Ptr(new Cloud);
    std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>> rays;
    for (const auto &msg : selected)
    {
      Eigen::Isometry3d sensor, acquisition_base_sensor;
      try
      {
        sensor = tf2::transformToEigen(
            buffer_->lookupTransform("odom", msg->header.frame_id, msg->header.stamp).transform);
        acquisition_base_sensor = tf2::transformToEigen(
            buffer_->lookupTransform("base_link", msg->header.frame_id, msg->header.stamp)
                .transform);
      }
      catch (const tf2::TransformException &)
      {
        return;
      }
      auto transform = reference.inverse() * sensor;
      Cloud input;
      pcl::fromROSMsg(*msg, input);
      for (const auto &p : input)
      {
        Eigen::Vector3d raw(p.x, p.y, p.z);
        if (!raw.allFinite() || raw.norm() < .15 || raw.norm() > 80)
        {
          continue;
        }
        // Remove self returns in the body frame AT ACQUISITION. Reprojecting a
        // past arm/bucket return into the latest body frame first can move it
        // outside today's silhouette and create a persistent phantom obstacle.
        if (model_.contains(acquisition_base_sensor * raw, .10))
        {
          continue;
        }
        Eigen::Vector3d body = transform * raw;
        Point q;
        q.x = body.x();
        q.y = body.y();
        q.z = body.z();
        combined->push_back(q);
        rays.emplace_back(transform.translation(),body);
      }
    }
    if (combined->empty())
    {
      return;
    }
    Cloud filtered;
    pcl::VoxelGrid<Point> grid;
    grid.setInputCloud(combined);
    grid.setLeafSize(.20, .20, .20);
    grid.filter(filtered);
    sensor_msgs::msg::PointCloud2 result;
    pcl::toROSMsg(filtered, result);
    result.header.frame_id = "base_link";
    result.header.stamp = reference_message.header.stamp;
    output_->publish(result);
    published_ = stamp;
    pending_.push_back({std::move(filtered),result.header,std::move(rays)});
    while(pending_.size()>12)pending_.pop_front();
    drainMapFrames();
  }
  void drainMapFrames(){
    if(mapping_only_||pending_.empty())return;
    for(auto it=pending_.begin();it!=pending_.end();){
      if(now().seconds()-rclcpp::Time(it->header.stamp).seconds()>.75)it=pending_.erase(it);else ++it;
    }
    for(auto it=pending_.rbegin();it!=pending_.rend();++it){
      if(!buffer_->canTransform("map","base_link",it->header.stamp))continue;
      PendingFrame frame=std::move(*it);
      const auto stamp=rclcpp::Time(frame.header.stamp);
      while(!pending_.empty()&&rclcpp::Time(pending_.front().header.stamp)<=stamp)pending_.pop_front();
      mapFrame(frame);return;
    }
  }
  void mapFrame(const PendingFrame& frame){
    try
    {
      auto map_base = tf2::transformToEigen(
          buffer_->lookupTransform("map", "base_link", frame.header.stamp).transform);
      // Ray clearing removes old occupied voxels only where a measured beam
      // actually passed. Missing returns/blind sectors do not clear occupancy.
      Eigen::Vector3d lower=Eigen::Vector3d::Constant(INFINITY),upper=Eigen::Vector3d::Constant(-INFINITY);
      for(const auto &cell:remembered_){
        Eigen::Vector3d q(cell.first[0]*.2,cell.first[1]*.2,cell.first[2]*.2);
        lower=lower.cwiseMin(q);upper=upper.cwiseMax((q.array()+.2).matrix());
      }
      for(const auto &ray:frame.rays){
        if(remembered_.empty())break;
        const Eigen::Vector3d origin=map_base*ray.first,end=map_base*ray.second;
        const double distance=(end-origin).norm();
        if(distance<.4)continue;
        // Clip the measured ray to the remembered obstacle AABB. This avoids
        // walking every ground/static ray over the entire sensor range.
        const Eigen::Vector3d direction=(end-origin)/distance;
        double lo=0.,hi=distance-.3;
        for(int axis=0;axis<3;++axis){
          if(std::abs(direction[axis])<1e-10){if(origin[axis]<lower[axis]||origin[axis]>upper[axis])hi=-1.;}
          else{double a=(lower[axis]-origin[axis])/direction[axis],b=(upper[axis]-origin[axis])/direction[axis];
            if(a>b)std::swap(a,b);lo=std::max(lo,a);hi=std::min(hi,b);}
        }
        if(hi<lo)continue;
        const int count=std::min(800,int(std::ceil((hi-lo)/.1))+1);
        for(int i=0;i<count;++i){
          Eigen::Vector3d q=origin+direction*std::min(hi,lo+double(i)*.1);
          remembered_.erase({int(std::floor(q.x()/.20)),int(std::floor(q.y()/.20)),int(std::floor(q.z()/.20))});
        }
      }
      std::vector<int> nearest(1);
      std::vector<float> squared_distance(1);
      auto candidates = Cloud::Ptr(new Cloud);
      Cloud known_obstacles;
      for (const auto &p : frame.cloud)
      {
        Eigen::Vector3d global = map_base * Eigen::Vector3d(p.x, p.y, p.z);
        if (!map_->ground(global, .15))
        {
          Point q;
          q.x = global.x();
          q.y = global.y();
          q.z = global.z();
          // Known static surfaces already have a persistent full-resolution
          // collision map. Remember only additional observations, including
          // objects temporarily hidden by the vehicle during a turn.
          if (!map_->obstacles->empty() &&
              map_->obstacle_tree->nearestKSearch(q, 1, nearest, squared_distance) > 0 &&
              squared_distance[0] < .09){known_obstacles.push_back(q);continue;}
          candidates->push_back(q);
        }
      }
      if (!candidates->empty())
      {
        pcl::KdTreeFLANN<Point> observation_tree;
        observation_tree.setInputCloud(candidates);
        for (const auto &q : *candidates)
        {
          // A single asynchronous ray outlier must not become a ten-second
          // solid obstacle. Require spatial support before adding memory.
          if (observation_tree.radiusSearch(q, neighbor_radius_, nearest, squared_distance,
                                            min_neighbors_) < min_neighbors_)
            continue;
          std::array<int, 3> key{int(std::floor(q.x / .20)), int(std::floor(q.y / .20)),
                                 int(std::floor(q.z / .20))};
          remembered_[key] = {q, rclcpp::Time(frame.header.stamp).seconds()};
        }
      }
      Cloud obstacle;
      for (auto it = remembered_.begin(); it != remembered_.end();)
      {
        const double age = rclcpp::Time(frame.header.stamp).seconds() - it->second.second;

        // An unobserved dynamic cell expires even near the body. Freshness loss
        // still stops the guard; an expired cell is not proof of free space.
        if (age < -.1 || age > obstacle_keep_time_)
          it = remembered_.erase(it);
        else
        {
          obstacle.push_back(it->second.first);
          ++it;
        }
      }
      obstacle+=known_obstacles;
      sensor_msgs::msg::PointCloud2 observation;
      pcl::toROSMsg(obstacle, observation);
      observation.header = frame.header;
      observation.header.frame_id = "map";
      obstacles_->publish(observation);
    }
    catch (const tf2::TransformException &)
    {
    }

  }
  RobotModel model_;
  bool mapping_only_;
  std::array<rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr, 5> primary_;
  std::array<livox_ros_driver2::msg::CustomMsg::ConstSharedPtr, 5> raw_;
  double primary_stamp_{-1};
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr filtered_primary_;
  std::unique_ptr<TerrainMap> map_;
  double margin_;
  double obstacle_keep_time_;
  double obstacle_keep_radius_;
  double neighbor_radius_;
  int min_neighbors_;
  std::map<std::array<int, 3>, std::pair<Point, double>> remembered_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  std::array<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr, 5> subscriptions_;
  std::array<std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr>, 5> clouds_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_, obstacles_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr health_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::TimerBase::SharedPtr timer_;
  double odom_stamp_{-1}, published_{-1};
};
} // namespace clearing_robot_plugins
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<clearing_robot_plugins::Perception>());
  rclcpp::shutdown();
}
