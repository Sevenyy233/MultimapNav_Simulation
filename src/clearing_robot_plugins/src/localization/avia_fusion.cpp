#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <stdexcept>
#include <vector>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace clearing_robot
{

/** @brief 将同一仿真时段的五路 Avia 扫描变换至 IMU，保留跨扫描时间差。 */
class AviaFusion : public rclcpp::Node
{
  using Scan = livox_ros_driver2::msg::CustomMsg;
  using Point = livox_ros_driver2::msg::CustomPoint;
  static constexpr std::size_t count = 5;

public:
  /** @brief 配置实际仿真雷达话题和允许的同步误差（秒）。 */
  AviaFusion()
  : Node("clearing_avia_fusion"), buffer_(get_clock()), listener_(buffer_)
  {
    target_ = declare_parameter("target_frame", "imu_link");
    const double skew = declare_parameter("max_skew", 0.040);
    if (!std::isfinite(skew) || skew <= 0.0 || skew >= 0.1) {
      throw std::invalid_argument("max_skew 取值需在 0 至 0.1 秒之间");
    }
    skew_ns_ = static_cast<int64_t>(skew * 1e9);
    const double max_range = declare_parameter("max_range", 55.0);
    if (!std::isfinite(max_range) || max_range <= 0.30) {
      throw std::invalid_argument("max_range 必须大于雷达盲区 0.30 m");
    }
    max_range_sq_ = max_range * max_range;
    output_ = create_publisher<Scan>("/fast_lio/lidar", rclcpp::QoS(10).reliable());
    for (std::size_t i = 0; i < count; ++i) {
      const std::string suffix = std::to_string(i + 1);
      subscriptions_[i] = create_subscription<Scan>("/livox/lidar_" + suffix,
        rclcpp::SensorDataQoS().keep_last(12),
        [this, i, suffix](Scan::ConstSharedPtr scan) {
          if (scan->header.frame_id != "rslidar_" + suffix || scan->points.empty() ||
            rclcpp::Time(scan->header.stamp).nanoseconds() <= last_stamp_)
          {
            return;
          }
          auto & queue = queues_[i];
          if (queue.size() == 12) {
            queue.pop_front();
          }
          queue.push_back(scan);
          synchronize();
        });
    }
  }

private:
  /** @brief 对队首做有界近似同步，每个消息只用于一个融合帧。 */
  void synchronize()
  {
    while (std::all_of(queues_.begin(), queues_.end(), [](const auto & queue) {return !queue.empty();})) {
      std::array<Scan::ConstSharedPtr, count> scans;
      std::array<int64_t, count> stamps{};
      for (std::size_t i = 0; i < count; ++i) {
        scans[i] = queues_[i].front();
        stamps[i] = rclcpp::Time(scans[i]->header.stamp).nanoseconds();
      }
      const auto bounds = std::minmax_element(stamps.begin(), stamps.end());
      if (*bounds.second - *bounds.first > skew_ns_) {
        queues_[static_cast<std::size_t>(bounds.first - stamps.begin())].pop_front();
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "五路 Avia 仿真时间戳未在窗口配齐");
        continue;
      }
      for (auto & queue : queues_) {
        queue.pop_front();
      }
      if (*bounds.first > last_stamp_ && publish(scans, stamps, *bounds.first)) {
        last_stamp_ = *bounds.second;
      }
    }
  }

  /** @brief 在各扫描时间查询外参、拼接并以纳秒偏移排序。 */
  bool publish(const std::array<Scan::ConstSharedPtr, count> & scans,
    const std::array<int64_t, count> & stamps, int64_t earliest)
  {
    std::array<tf2::Transform, count> transforms;
    tf2::Transform base_from_imu;
    try {
      const auto base = buffer_.lookupTransform("base_link", target_, tf2::TimePointZero);
      tf2::fromMsg(base.transform, base_from_imu);
      for (std::size_t i = 0; i < count; ++i) {
        const auto tf = buffer_.lookupTransform(target_, scans[i]->header.frame_id,
          scans[i]->header.stamp);
        tf2::fromMsg(tf.transform, transforms[i]);
      }
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Avia 外参不可用: %s", error.what());
      return false;
    }
    Scan merged;
    merged.header.frame_id = target_;
    merged.header.stamp = rclcpp::Time(earliest, RCL_ROS_TIME);
    merged.timebase = static_cast<uint64_t>(earliest);
    std::size_t points = 0;
    for (const auto & scan : scans) {
      points += scan->points.size();
    }
    merged.points.reserve(points);
    for (std::size_t i = 0; i < count; ++i) {
      const double sample_offset = static_cast<double>(stamps[i] - earliest);
      for (const auto & raw : scans[i]->points) {
        const double range_sq = raw.x * raw.x + raw.y * raw.y + raw.z * raw.z;
        if (!std::isfinite(range_sq) || range_sq < 0.30 * 0.30 || range_sq > max_range_sq_ ||
          raw.line > 5)
        {
          continue;
        }
        const auto coordinate = transforms[i] * tf2::Vector3(raw.x, raw.y, raw.z);
        // 移除低位收拢机械臂和车体反射。静态地图仍保留当前包络内的障碍。
        const auto body = base_from_imu * coordinate;
        if (body.x() >= -3.48 && body.x() <= 6.08 && std::abs(body.y()) <= 1.78 &&
          body.z() >= 0.20 && body.z() <= 4.33)
        {
          continue;
        }
        Point point = raw;
        point.x = static_cast<float>(coordinate.x());
        point.y = static_cast<float>(coordinate.y());
        point.z = static_cast<float>(coordinate.z());
        // Gazebo 光束来自冻结场景。厂商插件中的单点 offset 是 CPU 测时。
        point.offset_time = static_cast<uint32_t>(sample_offset);
        merged.points.push_back(point);
      }
    }
    std::stable_sort(merged.points.begin(), merged.points.end(),
      [](const Point & a, const Point & b) {return a.offset_time < b.offset_time;});
    merged.point_num = static_cast<uint32_t>(merged.points.size());
    if (merged.points.empty()) {
      return false;
    }
    output_->publish(merged);
    return true;
  }

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::string target_;
  int64_t skew_ns_{};
  double max_range_sq_{};
  int64_t last_stamp_{-1};
  std::array<std::deque<Scan::ConstSharedPtr>, count> queues_;
  std::array<rclcpp::Subscription<Scan>::SharedPtr, count> subscriptions_;
  rclcpp::Publisher<Scan>::SharedPtr output_;
};

}  // namespace clearing_robot

/** @brief 执行清舱机器人仿真多雷达同步节点。 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<clearing_robot::AviaFusion>());
  rclcpp::shutdown();
  return 0;
}
