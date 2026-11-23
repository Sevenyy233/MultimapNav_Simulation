#pragma once

#include <array>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <grid_map_core/GridMap.hpp>
#include <nav_msgs/msg/path.hpp>
#include <octomap/OcTree.h>
#include "clearing_robot_plugins/msg/compound_local_map.hpp"

namespace clearing_robot
{

using Action = std::array<float, 2>;
using Observation = std::array<float, 28>;

struct State
{
  double x{}, y{}, z{}, yaw{}, v{}, w{}, lateral{};
};

/** @brief 将复合地图缓存为地形栅格和障碍投影供整车扫掠检查。 */
class LocalScene
{
public:
  /** @brief 解码同步地图，包括障碍在支承地面以上的高度。 */
  explicit LocalScene(const clearing_robot_plugins::msg::CompoundLocalMap & message);

  /** @brief 读取网格支承面。 @return 有效时返回米制高程和坡度弧度。 */
  bool ground(double x, double y, double & elevation, double & slope) const;
  /** @brief 判断带安全余量的运输姿态包络是否可通过。 */
  bool traversable(const State & state, double margin = 0.20) const;
  /** @brief 检测物理包络与占据相交，供训练回合记录。 */
  bool collision(const State & state) const;
  /** @brief 返回沿机体方位的最近障碍或未知距离，范围 0 至 12 m。 */
  double range(const State & state, double relative_bearing) const;

  std_msgs::msg::Header header;
  bool valid{false};

private:
  bool occupied(double x, double y) const;
  grid_map::GridMap terrain_;
  grid_map::Matrix obstacles_;
};

/** @brief 清舱机器人滚动路径与 MPPI；传统、训练先验和部署共用。 */
class Mppi
{
public:
  static constexpr double v_min = 0.50;  // m/s，非零巡航目标下限
  static constexpr double v_max = 1.00;  // m/s
  static constexpr double w_max = 0.12;
  static constexpr double goal_tolerance = 0.45;
  static constexpr double curvature_max = 1.0 / 4.2;

  /** @brief 从地图系位姿与机体速度构造平面履带运动状态。 */
  static State state(const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::Twist & velocity);
  /** @brief 将速度命令编码到 [-1,1]。 */
  static Action normalize(const geometry_msgs::msg::Twist & twist);
  /** @brief 将策略动作解码到 m/s、rad/s。 */
  static geometry_msgs::msg::Twist decode(const Action & action);
  /** @brief 截取并验证全局路线的滚动段，必要时尝试有界侧向绕行。 */
  nav_msgs::msg::Path localPath(
    const LocalScene & scene, const State & current, const nav_msgs::msg::Path & global,
    bool & ends_at_goal) const;
  /** @brief 基于同一动力学和代价对路径做路径积分控制。
   * @param stop_reason 可选输出，指向零速先验的诊断文字；正常行驶时为空。
   */
  geometry_msgs::msg::Twist command(
    const LocalScene & scene, const State & current, const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Twist & applied, bool ends_at_goal,
    const char ** stop_reason = nullptr) const;
  /** @brief 约束并检查策略请求和紧急制动扫掠，或使用已求解的 MPPI 先验。 */
  geometry_msgs::msg::Twist shield(
    const LocalScene & scene, const State & state, const geometry_msgs::msg::Twist & applied,
    const geometry_msgs::msg::Twist & requested, const geometry_msgs::msg::Twist & prior) const;
  /** @brief 生成训练和 ONNX 共用的固定顺序无量纲观测。 */
  Observation observation(
    const LocalScene & scene, const geometry_msgs::msg::Pose & pose,
    const geometry_msgs::msg::Twist & velocity, const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Twist & applied, bool feedback_valid) const;

private:
  /** @brief 约束非零巡航目标和转向曲率，保留零速停车。 */
  static geometry_msgs::msg::Twist cruiseTarget(const geometry_msgs::msg::Twist & request);
  /** @brief 按速度、角速度、加速度和估计滑移积分一个时间片。 */
  static State advance(
    State state, const geometry_msgs::msg::Twist & command, double dt, double traction);
  /** @brief 检查指令维持和随后制动期间的整车扫掠。 */
  static bool canBrake(
    const LocalScene & scene, State state, const geometry_msgs::msg::Twist & command);
};

}  // namespace clearing_robot
