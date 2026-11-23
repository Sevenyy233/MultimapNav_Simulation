#ifndef CLEARING_ROBOT_NAVIGATION_MODE_PANEL_HPP_
#define CLEARING_ROBOT_NAVIGATION_MODE_PANEL_HPP_

#include <chrono>
#include <future>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>

class QComboBox;
class QLabel;
class QPushButton;
class QTimer;
namespace clearing_robot {
/** @brief 在不改变相机的情况下选择导航地图并同步可见图层。 */
class NavigationModePanel : public rviz_common::Panel {
  Q_OBJECT
public:
  /** @brief 创建控制面板。 */
  explicit NavigationModePanel(QWidget * parent = nullptr);
  /** @brief 连接模式状态及参数服务。 */
  void onInitialize() override;
private Q_SLOTS:
  /** @brief 异步请求模式参数。 */
  void apply();
  /** @brief 在 GUI 线程同步模式与显示状态。 */
  void updateState();
private:
  struct State { std::mutex mutex; std::deque<std::string> modes; };
  bool applyMode(const std::string & mode);
  std::shared_ptr<State> state_ = std::make_shared<State>();
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future_;
  std::chrono::steady_clock::time_point request_started_;
  int64_t request_id_ = 0;
  bool pending_ = false, switching_ = false;
  bool saved_local_ = true, saved_global_path_ = true, saved_local_path_ = true;
  QComboBox * choice_;
  QPushButton * apply_;
  QLabel * status_;
  QTimer * timer_;
};
}
#endif
