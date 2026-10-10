#include <cmath>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class OdomTFBroadcaster : public rclcpp::Node {
public:
  OdomTFBroadcaster() : Node("odom_tf_broadcaster") {
    const auto input=declare_parameter("odom_topic","/odom");
    const auto output=declare_parameter("output_odom_topic","/truth/odometry");
    parent_=declare_parameter("parent_frame_id","odom");
    source_=declare_parameter("source_frame_id","world");
    broadcaster_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    publisher_=create_publisher<nav_msgs::msg::Odometry>(output,rclcpp::QoS(10).reliable());
    subscription_=create_subscription<nav_msgs::msg::Odometry>(input,rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg){receive(msg);});
    RCLCPP_WARN(get_logger(),"Simulation truth: %s -> %s; %s and %s origins must coincide; full pose retained, world twist rotated to body.",
      input.c_str(),output.c_str(),source_.c_str(),parent_.c_str());
  }
private:
  void receive(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if(!msg || msg->header.frame_id!=source_ || msg->child_frame_id.empty()) {return;}
    const auto & p=msg->pose.pose.position;const auto & q=msg->pose.pose.orientation;
    const auto & v=msg->twist.twist;
    for(double x:{p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.linear.x,v.linear.y,v.linear.z,
      v.angular.x,v.angular.y,v.angular.z}) {if(!std::isfinite(x)){return;}}
    tf2::Quaternion rotation(q.x,q.y,q.z,q.w);
    if(rotation.length2()<1e-12){return;}rotation.normalize();
    auto output=*msg;output.header.frame_id=parent_;
    output.pose.pose.orientation.x=rotation.x();output.pose.pose.orientation.y=rotation.y();
    output.pose.pose.orientation.z=rotation.z();output.pose.pose.orientation.w=rotation.w();
    const auto R=tf2::Matrix3x3(rotation).transpose();
    const auto linear=R*tf2::Vector3(v.linear.x,v.linear.y,v.linear.z);
    const auto angular=R*tf2::Vector3(v.angular.x,v.angular.y,v.angular.z);
    output.twist.twist.linear.x=linear.x();output.twist.twist.linear.y=linear.y();output.twist.twist.linear.z=linear.z();
    output.twist.twist.angular.x=angular.x();output.twist.twist.angular.y=angular.y();output.twist.twist.angular.z=angular.z();
    for(int row=0;row<6;++row){for(int col=0;col<6;++col){
      double value=0.0;
      for(int a=0;a<3;++a){for(int b=0;b<3;++b){
        value+=R[row%3][a]*msg->twist.covariance[(row/3*3+a)*6+col/3*3+b]*R[col%3][b];
      }}output.twist.covariance[row*6+col]=value;
    }}
    publisher_->publish(output);
    geometry_msgs::msg::TransformStamped transform;
    transform.header=output.header;transform.child_frame_id=output.child_frame_id;
    transform.transform.translation.x=p.x;transform.transform.translation.y=p.y;transform.transform.translation.z=p.z;
    transform.transform.rotation=output.pose.pose.orientation;broadcaster_->sendTransform(transform);
  }
  std::string parent_,source_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};
int main(int argc,char ** argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<OdomTFBroadcaster>());rclcpp::shutdown();}
