#include <atomic>
#include <cmath>
#include <functional>
#include <sstream>
#include <string>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

namespace gazebo {
class ControlledObstacle : public ModelPlugin {
public:
  void Load(physics::ModelPtr model,sdf::ElementPtr sdf) override {
    model_=std::move(model);node_=gazebo_ros::Node::Get(sdf);
    mode_=sdf->Get<std::string>("mode");
    delay_=sdf->Get<double>("delay");duration_=sdf->Get<double>("duration");speed_=sdf->Get<double>("speed");
    start_=node_->create_subscription<std_msgs::msg::Float64>("/evaluation/sequence_start_time",rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Float64::ConstSharedPtr msg){if(start_time_.load()<0)start_time_.store(msg->data);});
    state_=node_->create_publisher<std_msgs::msg::String>("/evaluation/obstacle_state",20);
    connection_=event::Events::ConnectWorldUpdateBegin(std::bind(&ControlledObstacle::update,this,std::placeholders::_1));
  }
private:
  void update(const common::UpdateInfo & info) {
    const double now=info.simTime.Double(),start=start_time_.load();
    auto pose=model_->WorldPose();
    pose.Pos().Set(0.,0.,-3.);
    if(start>=0){
      const double elapsed=now-start-delay_;
      if(mode_=="crossing"&&elapsed>=0&&elapsed<=16.)pose.Pos().Set(0.,-2.+speed_*elapsed,.4);
      if((mode_=="temporary"||mode_=="permanent")&&elapsed>=0&&
          (mode_=="permanent"||elapsed<duration_))pose.Pos().Set(0.,0.,.5);
    }
    model_->SetWorldPose(pose);
    if(now-last_publish_<.1)return;
    std::ostringstream json;json.precision(12);
    json << "{\"stamp\":" << now << ",\"mode\":\"" << mode_ << "\",\"sequence_start\":" << start
         << ",\"x\":" << pose.Pos().X() << ",\"y\":" << pose.Pos().Y() << ",\"z\":" << pose.Pos().Z() << '}';
    std_msgs::msg::String msg;msg.data=json.str();state_->publish(msg);last_publish_=now;
  }
  physics::ModelPtr model_;gazebo_ros::Node::SharedPtr node_;event::ConnectionPtr connection_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr start_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_;
  std::atomic<double> start_time_{-1.};
  std::string mode_;double delay_{6.},duration_{4.},speed_{.25},last_publish_{0.};
};
GZ_REGISTER_MODEL_PLUGIN(ControlledObstacle)
}
