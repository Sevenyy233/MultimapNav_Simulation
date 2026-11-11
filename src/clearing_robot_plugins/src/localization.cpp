#include "clearing_robot_plugins/plugins.hpp"
#include <deque>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/create_timer.hpp>
#include <std_msgs/msg/string.hpp>

namespace clearing_robot_plugins
{
namespace
{
Cloud::Ptr downsample(const Cloud::ConstPtr &cloud, double leaf)
{
  auto out = Cloud::Ptr(new Cloud);
  pcl::VoxelGrid<Point> filter;
  filter.setInputCloud(cloud);
  filter.setLeafSize(leaf, leaf, leaf);
  filter.filter(*out);
  return out;
}
bool validPose(const geometry_msgs::msg::Pose &p)
{
  const auto &q = p.orientation;
  return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
         std::isfinite(p.position.z) && std::isfinite(q.x) && std::isfinite(q.y) &&
         std::isfinite(q.z) && std::isfinite(q.w) &&
         Eigen::Quaterniond(q.w, q.x, q.y, q.z).norm() > 1e-8;
}
} // namespace
struct MapCorrectedFastLio::Impl : public std::enable_shared_from_this<Impl>
{
  Node::WeakPtr node;
  std::mutex mutex, registration_mutex;
  std::atomic<bool> active{false};
  std::deque<nav_msgs::msg::Odometry> history;
  Eigen::Isometry3d correction{Eigen::Isometry3d::Identity()},
      initial{Eigen::Isometry3d::Identity()};
  bool seeded{false}, matched{false}, pending_seed{false};
  uint64_t initialization_epoch{0};
  double last_match{-1e9}, last_attempt{-1e9}, score{INFINITY};
  std::string rejection{"waiting for initial pose / scans"};
  double interval{.6}, max_age{3.0}, threshold{.25}, max_shift{.8}, max_angle{.20};
  Cloud::Ptr target{new Cloud};
  pcl::KdTreeFLANN<Point> tree;
  pcl::NormalDistributionsTransform<Point, Point> ndt;
  pcl::IterativeClosestPoint<Point, Point> icp;
  rclcpp::CallbackGroup::SharedPtr registration_group, odom_group;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr health_pub;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr status_pub;
  rclcpp::TimerBase::SharedPtr timer;
  bool healthyLocked(double now) const
  {
    return active && matched && !history.empty() &&
           now - rclcpp::Time(history.back().header.stamp).seconds() < .75 &&
           now - rclcpp::Time(history.back().header.stamp).seconds() >= -.1 &&
           now - last_match < max_age;
  }
  bool odomAtLocked(double stamp, Eigen::Isometry3d &t)
  {
    if (history.empty())
    {
      return false;
    }
    auto it = std::lower_bound(history.begin(), history.end(), stamp,
                               [](const auto &m, double s)
                               { return rclcpp::Time(m.header.stamp).seconds() < s; });
    if (it == history.end())
    {
      if (stamp - rclcpp::Time(history.back().header.stamp).seconds() > .03)
      {
        return false;
      }
      t = fromMsg(history.back().pose.pose).transform();
      return true;
    }
    if (it == history.begin())
    {
      if (rclcpp::Time(it->header.stamp).seconds() - stamp > .03)
      {
        return false;
      }
      t = fromMsg(it->pose.pose).transform();
      return true;
    }
    auto before = std::prev(it);
    double a = rclcpp::Time(before->header.stamp).seconds(),
           b = rclcpp::Time(it->header.stamp).seconds();
    double u = b > a ? (stamp - a) / (b - a) : 0;
    auto first = fromMsg(before->pose.pose).transform(),
         second = fromMsg(it->pose.pose).transform();
    t = Eigen::Isometry3d::Identity();
    t.translation() = (1 - u) * first.translation() + u * second.translation();
    t.linear() = Eigen::Quaterniond(first.linear())
                     .slerp(u, Eigen::Quaterniond(second.linear()))
                     .toRotationMatrix();
    return true;
  }
  void odom(const nav_msgs::msg::Odometry &m)
  {
    if (!active || m.header.frame_id != "odom" || m.child_frame_id != "base_link" ||
        !validPose(m.pose.pose))
    {
      return;
    }
    auto p = fromMsg(m.pose.pose).transform();
    if (!p.matrix().allFinite())
    {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex);
    if (!history.empty() &&
        rclcpp::Time(m.header.stamp) <= rclcpp::Time(history.back().header.stamp))
    {
      if (rclcpp::Time(history.back().header.stamp).seconds() -
              rclcpp::Time(m.header.stamp).seconds() <
          .2)
      {
        return;
      }
      history.clear();
      matched = false;
      seeded = false;
      pending_seed = true;
    }
    history.push_back(m);
    while (history.size() > 400)
    {
      history.pop_front();
    }
    if (pending_seed)
    {
      correction = initial * p.inverse();
      seeded = true;
      matched = false;
      pending_seed = false;
      last_attempt = -1e9;
    }
  }
  void scan(const sensor_msgs::msg::PointCloud2 &msg)
  {
    auto n = node.lock();
    if (!n || !active || msg.header.frame_id != "base_link")
    {
      return;
    }
    std::unique_lock<std::mutex> work(registration_mutex, std::try_to_lock);
    if (!work.owns_lock())
    {
      return;
    }
    const double stamp = rclcpp::Time(msg.header.stamp).seconds();
    auto reject = [&](const std::string &reason)
    {
      std::lock_guard<std::mutex> lock(mutex);
      rejection = reason;
    };
    Eigen::Isometry3d odom_base, guess;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (!seeded || stamp - last_attempt < interval || !odomAtLocked(stamp, odom_base) ||
          n->now().seconds() - stamp > 1.0)
      {
        return;
      }
      last_attempt = stamp;
      guess = correction * odom_base;
    }
    auto input = Cloud::Ptr(new Cloud);
    pcl::fromROSMsg(msg, *input);
    input = downsample(input, .30);
    if (input->size() < 150)
    {
      return;
    }
    Cloud aligned;
    ndt.setInputSource(input);
    ndt.align(aligned, guess.matrix().cast<float>());
    if (!ndt.hasConverged())
    {
      reject("NDT did not converge");
      return;
    }
    Eigen::Matrix4f result = ndt.getFinalTransformation();
    double fitness = ndt.getFitnessScore();
    icp.setInputSource(input);
    icp.align(aligned, result);
    if (icp.hasConverged() && icp.getFitnessScore() < fitness)
    {
      result = icp.getFinalTransformation();
      fitness = icp.getFitnessScore();
    }
    if (!result.allFinite() || !std::isfinite(fitness) || fitness > threshold)
    {
      reject("registration fitness rejected: " + std::to_string(fitness));
      return;
    }
    int hits = 0;
    std::vector<int> ids(1);
    std::vector<float> distances(1);
    for (const auto &p : *input)
    {
      Eigen::Vector4f q = result * Eigen::Vector4f(p.x, p.y, p.z, 1);
      Point sample;
      sample.x = q.x();
      sample.y = q.y();
      sample.z = q.z();
      if (tree.nearestKSearch(sample, 1, ids, distances) > 0 && distances[0] < .49)
      {
        ++hits;
      }
    }
    if (double(hits) / input->size() < .55)
    {
      reject("map overlap rejected: " + std::to_string(double(hits) / input->size()));
      return;
    }
    Eigen::Isometry3d measured = Eigen::Isometry3d::Identity();
    measured.matrix() = result.cast<double>();
    Eigen::Isometry3d next = measured * odom_base.inverse();
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (matched)
      {
        auto delta = correction.inverse() * next;
        if (delta.translation().norm() > max_shift ||
            Eigen::AngleAxisd(delta.linear()).angle() > max_angle)
        {
          rejection = "correction jump rejected: " + std::to_string(delta.translation().norm()) +
                      " m, " + std::to_string(Eigen::AngleAxisd(delta.linear()).angle()) + " rad";
          return;
        }
        correction.translation() = .5 * correction.translation() + .5 * next.translation();
        correction.linear() = Eigen::Quaterniond(correction.linear())
                                  .slerp(.5, Eigen::Quaterniond(next.linear()))
                                  .normalized()
                                  .toRotationMatrix();
      }
      else
      {
        correction = next;
      }
      matched = true;
      score = fitness;
      last_match = stamp;
      rejection = "accepted";
    }
    RCLCPP_INFO_THROTTLE(n->get_logger(), *n->get_clock(), 5000,
                         "FAST-LIO map correction accepted: fitness %.4f, overlap %.2f", fitness,
                         double(hits) / input->size());
  }
  void status()
  {
    auto n = node.lock();
    if (!n || !active)
    {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex);
    std_msgs::msg::Bool b;
    b.data = healthyLocked(n->now().seconds());
    health_pub->publish(b);
    std_msgs::msg::String s;
    s.data = std::string("{\"healthy\":") + (b.data ? "true" : "false") +
             ",\"initialization_epoch\":" + std::to_string(initialization_epoch) +
             ",\"fitness\":" + (std::isfinite(score) ? std::to_string(score) : "null") +
             ",\"match_age\":" + std::to_string(n->now().seconds() - last_match) +
             ",\"reason\":\"" + rejection + "\"}";
    status_pub->publish(s);
  }
};
MapCorrectedFastLio::MapCorrectedFastLio() = default;
MapCorrectedFastLio::~MapCorrectedFastLio() { deactivate(); }
void MapCorrectedFastLio::configure(const Node::WeakPtr &p, std::string name)
{
  auto n = p.lock();
  state_ = std::make_shared<Impl>();
  auto &s = *state_;
  s.node = p;
  Cloud::Ptr raw(new Cloud);
  if (pcl::io::loadPCDFile(parameter(n, "scene.map_file", std::string("")), *raw) < 0)
  {
    throw std::runtime_error("Missing localization prior");
  }
  s.target = downsample(raw, .35);
  s.tree.setInputCloud(s.target);
  s.interval = parameter(n, name + ".registration_interval", .6);
  s.max_age = parameter(n, name + ".maximum_match_age", 3.0);
  s.threshold = parameter(n, name + ".fitness_threshold", .25);
  s.max_shift = parameter(n, name + ".max_correction_translation", .8);
  s.max_angle = parameter(n, name + ".max_correction_angle", .20);
  for (double value : {s.interval, s.max_age, s.threshold, s.max_shift, s.max_angle})
  {
    if (!std::isfinite(value) || value <= 0)
      throw std::runtime_error("Invalid map registration parameters");
  }
  s.ndt.setResolution(1.0);
  s.ndt.setStepSize(.15);
  s.ndt.setTransformationEpsilon(.01);
  s.ndt.setMaximumIterations(20);
  s.ndt.setInputTarget(s.target);
  s.icp.setInputTarget(s.target);
  s.icp.setMaximumIterations(12);
  s.icp.setMaxCorrespondenceDistance(.7);
  s.icp.setTransformationEpsilon(1e-5);
  s.registration_group = n->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  s.odom_group = n->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  s.health_pub = n->create_publisher<std_msgs::msg::Bool>("/clearing/localization_healthy", 10);
  s.status_pub = n->create_publisher<std_msgs::msg::String>("/clearing/localization_status", 10);
}
void MapCorrectedFastLio::activate()
{
  auto s = state_;
  auto n = s->node.lock();
  s->active = true;
  s->health_pub->on_activate();
  s->status_pub->on_activate();
  std::weak_ptr<Impl> weak = s;
  rclcpp::SubscriptionOptions a, b;
  a.callback_group = s->odom_group;
  b.callback_group = s->registration_group;
  s->odom_sub = n->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS(),
      [weak](nav_msgs::msg::Odometry::ConstSharedPtr m)
      {
        if (auto p = weak.lock())
        {
          p->odom(*m);
        }
      },
      a);
  s->cloud_sub = n->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/clearing/points", rclcpp::SensorDataQoS().keep_last(1),
      [weak](sensor_msgs::msg::PointCloud2::ConstSharedPtr m)
      {
        if (auto p = weak.lock())
        {
          p->scan(*m);
        }
      },
      b);
  s->timer = rclcpp::create_timer(
      n->get_node_base_interface(), n->get_node_timers_interface(), n->get_clock(),
      rclcpp::Duration::from_seconds(.2),
      [weak]
      {
        if (auto p = weak.lock())
        {
          p->status();
        }
      },
      s->odom_group);
}
void MapCorrectedFastLio::deactivate()
{
  if (!state_)
  {
    return;
  }
  auto s = state_;
  s->active = false;
  if (s->timer)
  {
    s->timer->cancel();
  }
  s->odom_sub.reset();
  s->cloud_sub.reset();
  std::lock_guard<std::mutex> work(s->registration_mutex);
  if (s->health_pub->is_activated())
  {
    s->health_pub->on_deactivate();
    s->status_pub->on_deactivate();
  }
}
void MapCorrectedFastLio::cleanup()
{
  deactivate();
  state_.reset();
}
void MapCorrectedFastLio::shutdown() { cleanup(); }
void MapCorrectedFastLio::setInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped &p)
{
  if (!validPose(p.pose.pose))
  {
    throw std::runtime_error("Initial pose must contain finite coordinates and a valid quaternion");
  }
  if (p.header.frame_id != "map" && !p.header.frame_id.empty())
  {
    throw std::runtime_error("Initial pose must be in map");
  }
  auto s = state_;
  std::lock_guard<std::mutex> work(s->registration_mutex);
  std::lock_guard<std::mutex> lock(s->mutex);
  ++s->initialization_epoch;
  s->initial = fromMsg(p.pose.pose).transform();
  s->pending_seed = true;
  s->matched = false;
  if (!s->history.empty())
  {
    s->correction = s->initial * fromMsg(s->history.back().pose.pose).transform().inverse();
    s->seeded = true;
    s->pending_seed = false;
    s->last_attempt = -1e9;
  }
}
bool MapCorrectedFastLio::getCurrentOdometry(nav_msgs::msg::Odometry &output) const
{
  auto s = state_;
  if (!s)
  {
    return false;
  }
  auto n = s->node.lock();
  if (!n)
  {
    return false;
  }
  std::lock_guard<std::mutex> lock(s->mutex);
  if (!s->healthyLocked(n->now().seconds()))
  {
    return false;
  }
  output = s->history.back();
  auto t = s->correction * fromMsg(output.pose.pose).transform();
  geometry_msgs::msg::Pose q;
  q.position.x = t.translation().x();
  q.position.y = t.translation().y();
  q.position.z = t.translation().z();
  Eigen::Quaterniond r(t.linear());
  q.orientation.x = r.x();
  q.orientation.y = r.y();
  q.orientation.z = r.z();
  q.orientation.w = r.w();
  output.pose.pose = q;
  output.header.frame_id = "map";
  for (int i = 0; i < 6; ++i)
  {
    output.pose.covariance[i * 7] = std::max(.01, s->score);
  }
  return true;
}
bool MapCorrectedFastLio::getCurrentPose(geometry_msgs::msg::PoseWithCovarianceStamped &p) const
{
  nav_msgs::msg::Odometry o;
  if (!getCurrentOdometry(o))
  {
    return false;
  }
  p.header = o.header;
  p.pose = o.pose;
  return true;
}
bool MapCorrectedFastLio::isLocalized() const
{
  nav_msgs::msg::Odometry o;
  return getCurrentOdometry(o);
}
double MapCorrectedFastLio::getHealthScore() const
{
  auto s = state_;
  if (!s)
  {
    return 0;
  }
  auto n = s->node.lock();
  if (!n)
  {
    return 0;
  }
  std::lock_guard<std::mutex> lock(s->mutex);
  return s->healthyLocked(n->now().seconds()) ? std::clamp(1 - s->score / s->threshold, 0., 1.) : 0;
}
} // namespace clearing_robot_plugins
PLUGINLIB_EXPORT_CLASS(clearing_robot_plugins::MapCorrectedFastLio,
                       multimap_nav_core::LocalizationBase)
