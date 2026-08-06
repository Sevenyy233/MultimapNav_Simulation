#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class OdomTFBroadcaster : public rclcpp::Node {
public:
  OdomTFBroadcaster() : Node("odom_tf_broadcaster") {
    // 订阅话题: 默认 /odom（gazebo_ros_p3d 地面真值里程计）
    const std::string odom_topic = declare_parameter("odom_topic", "odom");
    // 父坐标系固定为 odom：p3d 消息的 header.frame_id 是 world，
    // 而仿真中 odom 原点与 world 原点重合，因此真值位姿可直接映射到 odom 系
    parent_frame_id_ = declare_parameter("parent_frame_id", "odom");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10, std::bind(&OdomTFBroadcaster::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "订阅里程计话题: %s, 发布 TF: %s -> <child_frame_id>",
      odom_topic.c_str(), parent_frame_id_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = parent_frame_id_;
    t.child_frame_id = msg->child_frame_id;
    //t.child_frame_id = "camera_init";
    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;
    t.transform.rotation = msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);
  }

  std::string parent_frame_id_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
 
