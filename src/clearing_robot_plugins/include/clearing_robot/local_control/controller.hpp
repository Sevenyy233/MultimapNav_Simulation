#pragma once

#include <memory>
#include <mutex>

#include <multimap_nav_core/local_controller_base.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "clearing_robot/local_control/mppi.hpp"

namespace clearing_robot
{

/** @brief 地图和执行反馈缓存，由统一控制插件和训练服务分别订阅。 */
class ControlInputs
{
public:
  /** @brief 订阅整车扫掠所需的内部感知窗口和底盘执行反馈。 */
  template<typename NodeT>
  explicit ControlInputs(NodeT & node)
  {
    map_sub_ = node.template create_subscription<clearing_robot_plugins::msg::CompoundLocalMap>(
      "/clearing/control_map", rclcpp::QoS(1).reliable().transient_local(),
      [this, logger = node.get_logger()](clearing_robot_plugins::msg::CompoundLocalMap::ConstSharedPtr message) {
        try {
          auto scene = std::make_shared<LocalScene>(*message);
          std::lock_guard<std::mutex> lock(mutex_);
          scene_ = std::move(scene);
        } catch (const std::exception & error) {
          RCLCPP_ERROR(logger, "复合地图解析失败: %s", error.what());
          std::lock_guard<std::mutex> lock(mutex_);
          scene_.reset();
        }
      });
    applied_sub_ = node.template create_subscription<geometry_msgs::msg::TwistStamped>(
      "/clearing/cmd_vel_applied", rclcpp::QoS(1).best_effort(),
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        applied_ = message->twist;
        applied_time_ = rclcpp::Time(message->header.stamp).seconds();
      });
  }

  /** @brief 获取同一快照，丢弃过期或未来的地图和执行反馈。 */
  std::shared_ptr<const LocalScene> snapshot(
    const rclcpp::Time & time, geometry_msgs::msg::Twist & applied, bool & feedback_valid) const;

private:
  mutable std::mutex mutex_;
  std::shared_ptr<const LocalScene> scene_;
  geometry_msgs::msg::Twist applied_;
  double applied_time_{-1.0};
  rclcpp::Subscription<clearing_robot_plugins::msg::CompoundLocalMap>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr applied_sub_;
};

/** @brief MultiMapNav 统一 MPPI 控制器；路径更新与速度调用使用一致缓存。 */
class MppiController : public multimap_nav_core::LocalControllerBase
{
public:
  /** @brief 加载以 name 为前缀的控制器参数并初始化订阅。 */
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name) override;
  /** @brief 启用局部规划与速度生成。 */
  void activate() override;
  /** @brief 停止控制并删除当前目标。 */
  void deactivate() override;
  /** @brief 清除订阅和模型。 */
  void cleanup() override;
  /** @brief 释放控制器资源。 */
  void shutdown() override;
  /** @brief 保存规划服务器发布的全局参考。 */
  void setGlobalPlan(const nav_msgs::msg::Path & global_path) override;
  /** @brief 计算当前滚动局部规划，地图不健康时返回空路径。 */
  nav_msgs::msg::Path computeLocalPath(const geometry_msgs::msg::PoseStamped & pose) override;
  /** @brief 返回上次规划是否抵达全局终点。 */
  bool lastPlanReachesGoal() const override;
  /** @brief 更新速度控制器的参考路径。 */
  void setLocalPlan(const nav_msgs::msg::Path & path, bool ends_at_goal) override;
  /** @brief 计算带 ROS 时间戳的速度。 */
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity) override;
  /** @brief 查询控制器是否已在滚动路径的最终目标容差内。 */
  bool isGoalReached() override;

protected:
  /** @brief 传统插件直接返回 MPPI；部署插件允许 ONNX 策略改变当前命令。 */
  virtual geometry_msgs::msg::Twist selectCommand(
    const LocalScene & scene, const geometry_msgs::msg::Pose & pose,
    const geometry_msgs::msg::Twist & velocity, const geometry_msgs::msg::Twist & applied,
    const geometry_msgs::msg::Twist & prior, const nav_msgs::msg::Path & path, bool feedback_valid);

  Mppi mppi_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

private:
  std::unique_ptr<ControlInputs> inputs_;
  mutable std::mutex mutex_;
  bool active_{false};
  bool last_plan_reaches_goal_{false};
  bool local_ends_at_goal_{false};
  bool reached_{false};
  nav_msgs::msg::Path global_;
  nav_msgs::msg::Path local_;
};

/** @brief 用 ONNX Runtime 推理融合策略，同时复用传统 MPPI 和制动检查。 */
class DrlController : public MppiController
{
public:
  /** @brief 初始化未加载的部署策略。 */
  DrlController();
  /** @brief 释放 ONNX 会话。 */
  ~DrlController() override;
  /** @brief 加载参数 onnx_model；模型和接口不合法时配置失败。 */
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name) override;
  /** @brief 释放模型和控制订阅。 */
  void cleanup() override;

protected:
  /** @brief 推理动作并执行同一地图制动约束。 */
  geometry_msgs::msg::Twist selectCommand(
    const LocalScene & scene, const geometry_msgs::msg::Pose & pose,
    const geometry_msgs::msg::Twist & velocity, const geometry_msgs::msg::Twist & applied,
    const geometry_msgs::msg::Twist & prior, const nav_msgs::msg::Path & path, bool feedback_valid) override;

private:
  struct Policy;
  std::unique_ptr<Policy> policy_;
};

}  // namespace clearing_robot
