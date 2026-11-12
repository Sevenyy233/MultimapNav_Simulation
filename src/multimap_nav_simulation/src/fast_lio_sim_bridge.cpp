#include <cmath>
#include <chrono>
#include <deque>
#include <stdexcept>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "multimap_nav_simulation/pose_velocity_estimator.hpp"

// Adapt simulation messages and FAST-LIO's fixed frame names in our own package.
// No wheel states or ground-truth odometry are subscribed to here.
class FastLioSimBridge : public rclcpp::Node
{
public:
  FastLioSimBridge() : Node("fast_lio_sim_bridge")
  {
    const auto input = declare_parameter("lidar_topic", "/livox/lidar_3");
    lidar_frame_ = declare_parameter("lidar_frame", "rslidar_3");
    velocity_estimator_ = multimap_nav_simulation::PoseVelocityEstimator(
      declare_parameter("twist_filter_time_constant", 0.2),
      declare_parameter("twist_max_interval", 0.5));
    tf_wait_timeout_ = declare_parameter("tf_wait_timeout", 1.0);
    const auto queue_size = declare_parameter("tf_queue_size", 50);
    if (!std::isfinite(tf_wait_timeout_) || tf_wait_timeout_ <= 0.0 || queue_size < 1) {
      throw std::invalid_argument("Invalid TF wait timeout or queue capacity");
    }
    tf_queue_size_ = static_cast<std::size_t>(queue_size);
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_);
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    std::vector<geometry_msgs::msg::TransformStamped> aliases(2);
    for (auto & alias : aliases) {
      alias.header.stamp = now();
      alias.transform.rotation.w = 1.0;
    }
    aliases[0].header.frame_id = "odom";
    aliases[0].child_frame_id = "camera_init";
    aliases[1].header.frame_id = "imu_link";
    aliases[1].child_frame_id = "body";
    static_broadcaster_->sendTransform(aliases);

    lidar_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>("/fast_lio/lidar", 10);
    lidar_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input, rclcpp::SensorDataQoS(),
      [this](livox_ros_driver2::msg::CustomMsg::UniquePtr scan) {
        if (scan->header.frame_id != lidar_frame_ || scan->points.empty()) {return;}
        // The vendor ray plugin samples a frozen Gazebo scene. Its per-point
        // offsets measure wall-clock CPU work, NOT acquisition in simulation
        // time. All points in this instantaneous frame share header.stamp.
        const rclcpp::Time stamp(scan->header.stamp);
        scan->timebase = static_cast<uint64_t>(stamp.nanoseconds());
        scan->point_num = static_cast<uint32_t>(scan->points.size());
        for (auto & point : scan->points) {point.offset_time = 0;}
        lidar_pub_->publish(std::move(scan));
      });
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/fast_lio/odometry", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {receive_odometry(std::move(msg));});
    tf_retry_timer_ = create_wall_timer(std::chrono::milliseconds(5), [this] {flush_odometry();});
  }

private:
  struct PendingOdometry
  {
    nav_msgs::msg::Odometry::ConstSharedPtr message;
    std::chrono::steady_clock::time_point received;
  };

  void receive_odometry(nav_msgs::msg::Odometry::ConstSharedPtr input)
  {
    if (input->header.frame_id != "camera_init" || input->child_frame_id != "body") {return;}
    const double stamp = rclcpp::Time(input->header.stamp).seconds();
    if (stamp <= last_received_stamp_) {
      if (last_received_stamp_ - stamp > 0.5) {
        pending_odometry_.clear();  // A new clock epoch must not publish old queued states.
      } else {
        return;  // Duplicate/out-of-order packets are not new measurements.
      }
    }
    last_received_stamp_ = stamp;
    pending_odometry_.push_back({std::move(input), std::chrono::steady_clock::now()});
    if (pending_odometry_.size() > tf_queue_size_) {
      pending_odometry_.pop_front();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "TF wait queue full; discarded oldest odometry without changing its timestamp");
    }
    flush_odometry();
  }

  void flush_odometry()
  {
    while (!pending_odometry_.empty()) {
      const auto & pending = pending_odometry_.front();
      geometry_msgs::msg::TransformStamped base_imu_msg;
      try {
        // Wait for bracketing joint transforms at the actual measurement time.
        // Looking up the latest transform and restamping it would fabricate timing.
        base_imu_msg = buffer_->lookupTransform(
          "base_link", "imu_link", pending.message->header.stamp);
      } catch (const tf2::TransformException & error) {
        const double age = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - pending.received).count();
        if (age < tf_wait_timeout_) {return;}
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "TF wait expired after %.3f s; discarded odometry: %s", age, error.what());
        pending_odometry_.pop_front();
        continue;
      }
      publish_odometry(*pending.message, base_imu_msg);
      pending_odometry_.pop_front();
    }
  }

  void publish_odometry(const nav_msgs::msg::Odometry & input,
                        const geometry_msgs::msg::TransformStamped & base_imu_msg)
  {
    const auto & p = input.pose.pose.position;
    const auto & q = input.pose.pose.orientation;
    const auto & t = base_imu_msg.transform;
    for (const double value : {p.x, p.y, p.z, q.x, q.y, q.z, q.w}) {
      if (!std::isfinite(value)) {return;}
    }
    tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
    if (rotation.length2() < 1e-12) {return;}
    rotation.normalize();
    const tf2::Transform odom_imu(rotation, tf2::Vector3(p.x, p.y, p.z));
    const tf2::Transform base_imu(
      tf2::Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
      tf2::Vector3(t.translation.x, t.translation.y, t.translation.z));
    const auto odom_base = odom_imu * base_imu.inverse();
    nav_msgs::msg::Odometry output;
    output.header = input.header;
    output.header.frame_id = "odom";
    output.child_frame_id = "base_link";
    output.pose.pose.position.x = odom_base.getOrigin().x();
    output.pose.pose.position.y = odom_base.getOrigin().y();
    output.pose.pose.position.z = odom_base.getOrigin().z();
    output.pose.pose.orientation.x = odom_base.getRotation().x();
    output.pose.pose.orientation.y = odom_base.getRotation().y();
    output.pose.pose.orientation.z = odom_base.getRotation().z();
    output.pose.pose.orientation.w = odom_base.getRotation().w();
    const auto velocity = velocity_estimator_.update(
      odom_base, rclcpp::Time(input.header.stamp).seconds());
    if (velocity.valid) {
      output.twist.twist.linear.x = velocity.linear.x();
      output.twist.twist.linear.y = velocity.linear.y();
      output.twist.twist.linear.z = velocity.linear.z();
      output.twist.twist.angular.x = velocity.angular.x();
      output.twist.twist.angular.y = velocity.angular.y();
      output.twist.twist.angular.z = velocity.angular.z();
    }
    // The pose lever-arm covariance and derivative covariance are not modeled.
    // Do not present either estimate as a precisely calibrated measurement.
    for (int axis = 0; axis < 6; ++axis) {
      output.pose.covariance[axis * 7] = 1e6;
      output.twist.covariance[axis * 7] = 1e6;
    }
    odom_pub_->publish(output);
    geometry_msgs::msg::TransformStamped transform;
    transform.header = output.header;
    transform.child_frame_id = output.child_frame_id;
    transform.transform.translation.x = output.pose.pose.position.x;
    transform.transform.translation.y = output.pose.pose.position.y;
    transform.transform.translation.z = output.pose.pose.position.z;
    transform.transform.rotation = output.pose.pose.orientation;
    broadcaster_->sendTransform(transform);
  }

  std::deque<PendingOdometry> pending_odometry_;
  std::size_t tf_queue_size_{50};
  double tf_wait_timeout_{1.0};
  double last_received_stamp_{-1.0};
  rclcpp::TimerBase::SharedPtr tf_retry_timer_;
  std::string lidar_frame_;
  multimap_nav_simulation::PoseVelocityEstimator velocity_estimator_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_pub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastLioSimBridge>());
  rclcpp::shutdown();
  return 0;
}
