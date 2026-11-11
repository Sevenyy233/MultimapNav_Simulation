#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "multimap_nav_simulation/body_velocity_pi.hpp"
using namespace std::chrono_literals;
class BodyVelocityController : public rclcpp::Node {
public:
  BodyVelocityController():Node("tracked_body_velocity_controller") {
    const auto frequency=declare_parameter("control_frequency",50.);
    command_timeout_=declare_parameter("command_timeout",.5);
    odom_timeout_=declare_parameter("odom_timeout",.5);imu_timeout_=declare_parameter("imu_timeout",.2);
    linear_=axis("linear",.3,2.);angular_=axis("angular",.7,1.2);
    guard_=multimap_nav_simulation::VelocityFeedbackGuard(
      declare_parameter("feedback_max_linear_speed",3.),declare_parameter("feedback_max_angular_speed",3.));
    if(!std::isfinite(frequency)||frequency<=0.)throw std::invalid_argument("Invalid controller frequency");
    buffer_=std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_=std::make_unique<tf2_ros::TransformListener>(*buffer_);
    output_=create_publisher<geometry_msgs::msg::Twist>("/actuation/cmd_vel",10);
    status_=create_publisher<std_msgs::msg::String>("/actuation/velocity_status",20);
    reference_=create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10,[this](geometry_msgs::msg::Twist::ConstSharedPtr msg){
      if(!std::isfinite(msg->linear.x)||!std::isfinite(msg->angular.z))return;
      desired_=*msg;last_command_=Clock::now();has_command_=true;
    });
    odom_=create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS().keep_last(100),[this](nav_msgs::msg::Odometry::ConstSharedPtr msg){
      last_odom_=Clock::now();has_odom_=true;
      if(msg->child_frame_id!="base_link"){guard_.invalidate("odometry_frame_invalid");return;}
      const auto & v=msg->twist.twist.linear;
      guard_.observeLinear(v.x,v.y,v.z);
      if(std::isfinite(v.x))measured_linear_=v.x;
    });
    imu_=create_subscription<sensor_msgs::msg::Imu>("/livox/imu",rclcpp::SensorDataQoS().keep_last(1000),[this](sensor_msgs::msg::Imu::ConstSharedPtr msg){
      const auto & w=msg->angular_velocity;
      imu_stamp_=rclcpp::Time(msg->header.stamp).seconds();
      if(!guard_.observeAngular(w.x,w.y,w.z)){has_imu_=false;return;}
      try {
        const auto transform=buffer_->lookupTransform("base_link",msg->header.frame_id,tf2::TimePointZero);
        tf2::Quaternion rotation;tf2::fromMsg(transform.transform.rotation,rotation);
        measured_angular_=(tf2::Matrix3x3(rotation)*tf2::Vector3(w.x,w.y,w.z)).z();
        imu_stamp_=rclcpp::Time(msg->header.stamp).seconds();last_imu_=Clock::now();has_imu_=true;
      }catch(const tf2::TransformException&){}
    });
    timer_=create_wall_timer(std::chrono::nanoseconds(static_cast<int64_t>(1e9/frequency)),[this]{step();});
  }
private:
  using Clock=std::chrono::steady_clock;
  multimap_nav_simulation::VelocityPI axis(const std::string & name,double integral,double limit) {
    return {declare_parameter(name+".kp",1.),declare_parameter(name+".ki",2.),
      declare_parameter(name+".integral_limit",integral),declare_parameter(name+".command_limit",limit)};
  }
  void step() {
    const auto now=Clock::now();
    auto age=[&](auto time){return std::chrono::duration<double>(now-time).count();};
    const bool timestamps_fresh=has_odom_&&has_imu_&&age(last_odom_)<=odom_timeout_&&age(last_imu_)<=imu_timeout_;
    const bool fresh=timestamps_fresh&&guard_.usable();
    const bool command_fresh=has_command_&&age(last_command_)<=command_timeout_;
    geometry_msgs::msg::Twist output;
    const double v=command_fresh?desired_.linear.x:0.,w=command_fresh?desired_.angular.z:0.;
    double dt=last_step_stamp_<0.?0.:imu_stamp_-last_step_stamp_;last_step_stamp_=imu_stamp_;
    if(fresh&&dt>=0.&&dt<=.2) {
      output.linear.x=linear_.update(v,measured_linear_,dt);
      output.angular.z=angular_.update(w,measured_angular_,dt);
    }else {linear_.reset();angular_.reset();}
    output_->publish(output);
    if(imu_stamp_-last_status_>=.1) {
      std::ostringstream text;text.precision(12);
      text<<"{\"stamp\":"<<imu_stamp_<<",\"feedback_fresh\":"<<(fresh?"true":"false")
          <<",\"timestamps_fresh\":"<<(timestamps_fresh?"true":"false")
          <<",\"feedback_fault_latched\":"<<(guard_.usable()?"false":"true")
          <<",\"feedback_fault_reason\":\""<<guard_.reason()<<"\""
          <<",\"reference\":["<<v<<','<<w<<"],\"measured\":["<<measured_linear_<<','<<measured_angular_
          <<"],\"actuator\":["<<output.linear.x<<','<<output.angular.z<<"]}";
      std_msgs::msg::String msg;msg.data=text.str();status_->publish(msg);last_status_=imu_stamp_;
    }
  }
  multimap_nav_simulation::VelocityPI linear_,angular_;
  multimap_nav_simulation::VelocityFeedbackGuard guard_;
  double command_timeout_{.5},odom_timeout_{.5},imu_timeout_{.2},measured_linear_{0.},measured_angular_{0.};
  double imu_stamp_{0.},last_step_stamp_{-1.},last_status_{0.};
  bool has_command_{false},has_odom_{false},has_imu_{false};geometry_msgs::msg::Twist desired_;
  Clock::time_point last_command_{},last_odom_{},last_imu_{};
  std::unique_ptr<tf2_ros::Buffer> buffer_;std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr reference_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char**argv) {
  rclcpp::init(argc,argv);auto node=std::make_shared<BodyVelocityController>();
  rclcpp::spin(node);rclcpp::shutdown();
}
