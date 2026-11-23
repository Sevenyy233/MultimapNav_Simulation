#include "clearing_robot/local_control/controller.hpp"
#include "clearing_robot_plugins/srv/training_control.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>

namespace clearing_robot
{

/** @brief 单独训练图中唯一的底盘命令来源，使用导航同一控制实现。 */
class TrainingControl : public rclcpp::Node
{
public:
  /** @brief 建立已确认 LIO 位姿、地图、动作服务和失联停车。 */
  TrainingControl() : Node("clearing_training_control"), inputs_(*this)
  {
    auto qos = rclcpp::QoS(10).reliable();
    localization_ = create_subscription<nav_msgs::msg::Odometry>("/localization/odometry", qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {pose_ = message;});
    health_ = create_subscription<std_msgs::msg::Bool>("/localization/healthy", qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        healthy_ = message->data;
        health_stamp_ = now().seconds();
        if (!healthy_) {
          cancelCommand();
        }
      });
    contacts_ = create_subscription<std_msgs::msg::Bool>("/clearing/contact_collision", qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {
        if (message->data && now().seconds() >= accept_contacts_from_) {
          contact_collision_ = true;
          cancelCommand();
        }
      });
    reset_ = create_subscription<std_msgs::msg::Empty>("/clearing/estimator_reset", 1,
      [this](std_msgs::msg::Empty::ConstSharedPtr) {
        contact_collision_ = false;
        reset_stamp_ = now().seconds();
        accept_contacts_from_ = reset_stamp_ + 0.55;
        pose_.reset();
        healthy_ = false;
        cancelCommand();
      });
    command_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    service_ = create_service<Service>("/clearing/training_control",
      [this](const std::shared_ptr<Service::Request> request,
        std::shared_ptr<Service::Response> response) {handle(*request, *response);});
    watchdog_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
        if (moving_ && (std::chrono::steady_clock::now() > expiry_ || !healthy_)) {
          cancelCommand();
        }
      });
  }

private:
  using Service = clearing_robot_plugins::srv::TrainingControl;
  using Steady = std::chrono::steady_clock;
  struct Prepared
  {
    std::shared_ptr<const LocalScene> scene;
    Service::Response observation;
    geometry_msgs::msg::Twist applied;
    geometry_msgs::msg::Twist prior;
  };

  /** @brief 发布零速并使所有待应用的旧样本失效。 */
  void cancelCommand()
  {
    pending_.reset();
    moving_ = false;
    command_->publish(geometry_msgs::msg::Twist{});
  }

  /** @brief 确认定位和复合地图时效，准备观测与共享 MPPI 先验。 */
  void observe(const nav_msgs::msg::Path & route, Service::Response & response)
  {
    pending_.reset();
    const auto time = now();
    bool feedback_valid = false;
    geometry_msgs::msg::Twist applied;
    const auto scene = inputs_.snapshot(time, applied, feedback_valid);
    if (!healthy_ || time.seconds() - health_stamp_ > 0.5 || health_stamp_ <= reset_stamp_) {
      response.reason = "定位健康状态未恢复";
      return;
    }
    if (!pose_ || pose_->header.frame_id != "map") {
      response.reason = "地图坐标系定位位姿未到达";
      return;
    }
    const double pose_stamp = rclcpp::Time(pose_->header.stamp).seconds();
    if (pose_stamp <= reset_stamp_ || time.seconds() - pose_stamp > 0.8) {
      response.reason = "定位位姿未跨过复位时间或已过期";
      return;
    }
    if (!scene || rclcpp::Time(scene->header.stamp).seconds() <= reset_stamp_) {
      response.reason = "复位后的滚动地图未到达或已过期";
      return;
    }
    if (!feedback_valid) {
      response.reason = "履带速度反馈未到达或已过期";
      return;
    }
    response.header = pose_->header;
    response.pose = pose_->pose.pose;
    response.velocity = pose_->twist.twist;
    response.command = applied;
    const auto state = Mppi::state(response.pose, response.velocity);
    response.collision = contact_collision_ || scene->collision(state);
    response.success = true;
    if (response.collision) {
      response.reason = contact_collision_ ? "Gazebo 接触碰撞" : "局部地图占据与车体相交";
      return;
    }
    if (route.poses.empty()) {
      return;
    }
    bool ends_at_goal = false;
    const auto local = mppi_.localPath(*scene, state, route, ends_at_goal);
    if (local.poses.empty()) {
      response.reason = "未找到可行局部路径";
      return;
    }
    const char * prior_stop_reason = nullptr;
    const auto prior = mppi_.command(*scene, state, local, applied, ends_at_goal, &prior_stop_reason);
    if (prior_stop_reason) {
      response.reason = prior_stop_reason;
    }
    response.path_valid = true;
    response.prior = Mppi::normalize(prior);
    response.observation = mppi_.observation(*scene, response.pose, response.velocity,
      local, applied, feedback_valid);
    pending_ = std::make_unique<Prepared>(Prepared{scene, response, applied, prior});
  }

  /** @brief 应用与待处理观测一一对应的有限动作。 */
  void handle(const Service::Request & request, Service::Response & response)
  {
    if (request.operation == Service::Request::STOP) {
      cancelCommand();
      response.success = true;
      return;
    }
    if (request.operation == Service::Request::OBSERVE) {
      observe(request.path, response);
      return;
    }
    if (request.operation == Service::Request::APPLY && contact_collision_) {
      cancelCommand();
      response.success = true;
      response.collision = true;
      response.reason = "接触碰撞，动作未执行";
      return;
    }
    if (request.operation != Service::Request::APPLY || !pending_ ||
      request.observation_stamp != pending_->observation.header.stamp ||
      !std::isfinite(request.action[0]) || !std::isfinite(request.action[1]) ||
      !healthy_ || (now() - rclcpp::Time(pending_->observation.header.stamp)).seconds() > 0.50)
    {
      cancelCommand();
      response.reason = "动作与可用的近期观测不匹配";
      return;
    }
    auto prepared = std::move(pending_);
    response = prepared->observation;
    const auto state = Mppi::state(response.pose, response.velocity);
    response.command = mppi_.shield(*prepared->scene, state, prepared->applied,
      Mppi::decode(request.action), prepared->prior);
    command_->publish(response.command);
    moving_ = true;
    expiry_ = Steady::now() + std::chrono::milliseconds(1100);
  }

  ControlInputs inputs_;
  Mppi mppi_;
  bool healthy_{false};
  double health_stamp_{-1.0};
  bool moving_{false};
  bool contact_collision_{false};
  double accept_contacts_from_{-1.0};
  double reset_stamp_{-1.0};
  Steady::time_point expiry_;
  nav_msgs::msg::Odometry::ConstSharedPtr pose_;
  std::unique_ptr<Prepared> pending_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localization_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr contacts_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_;
  rclcpp::Service<Service>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

}  // namespace clearing_robot

/** @brief 在训练模式运行控制服务；保持 ROS 仿真时钟自然推进。 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<clearing_robot::TrainingControl>());
  rclcpp::shutdown();
  return 0;
}
