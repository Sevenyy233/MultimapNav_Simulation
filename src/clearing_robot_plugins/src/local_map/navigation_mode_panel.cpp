#include "clearing_robot/local_map/navigation_mode_panel.hpp"
#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace clearing_robot {
NavigationModePanel::NavigationModePanel(QWidget * parent) : Panel(parent) {
  auto * layout = new QVBoxLayout(this);
  auto * row = new QHBoxLayout;
  choice_ = new QComboBox(this);
  for (const auto * name : {"auto", "costmap2d", "gridmap", "octomap"}) choice_->addItem(name);
  apply_ = new QPushButton("Apply",this);
  row->addWidget(choice_); row->addWidget(apply_); layout->addLayout(row);
  status_ = new QLabel("Waiting for navigation mode",this); status_->setWordWrap(true); layout->addWidget(status_);
  connect(apply_, &QPushButton::clicked, this, &NavigationModePanel::apply);
  timer_ = new QTimer(this); connect(timer_, &QTimer::timeout, this, &NavigationModePanel::updateState);
}
void NavigationModePanel::onInitialize() {
  const auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  auto state = state_;
  subscription_ = node->create_subscription<std_msgs::msg::String>(
    "/clearing/active_map_mode", rclcpp::QoS(1).reliable().transient_local(),
    [state](std_msgs::msg::String::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(state->mutex);
      if (state->modes.empty() || state->modes.back() != msg->data) state->modes.push_back(msg->data);
    });
  client_ = node->create_client<rcl_interfaces::srv::SetParameters>("/clearing_navigation_modes/set_parameters");
  timer_->start(150);
}
void NavigationModePanel::apply() {
  if (!client_ || !client_->service_is_ready()) { status_->setText("Mode service unavailable"); return; }
  auto req = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  rcl_interfaces::msg::Parameter param;
  param.name="map_mode"; param.value.type=rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  param.value.string_value=choice_->currentText().toStdString(); req->parameters.push_back(param);
  auto result = client_->async_send_request(req);
  future_ = result.future.share(); request_id_ = result.request_id;
  request_started_ = std::chrono::steady_clock::now(); pending_=true; apply_->setEnabled(false);
  status_->setText("Requesting mode change…");
}
bool NavigationModePanel::applyMode(const std::string & mode) {
  if (mode != "" && mode != "costmap2d" && mode != "gridmap" && mode != "octomap") return true;
  auto * group = getDisplayContext()->getRootDisplayGroup();
  if (!group) return false;
  rviz_common::Display * cost=nullptr,* grid=nullptr,* oct=nullptr,* local=nullptr,* global_path=nullptr,* local_path=nullptr;
  for (int i=0;i<group->numDisplays();++i) {
    auto * d=group->getDisplayAt(i); if (!d) continue;
    const auto name=d->getName();
    if (name=="Global Costmap") cost=d; else if (name=="Global GridMap") grid=d;
    else if (name=="Global OctoMap") oct=d; else if (name=="Local Terrain and Occupancy") local=d;
    else if (name=="Global Path") global_path=d; else if (name=="Local Path") local_path=d;
  }
  if (!cost || !grid || !oct || !local || !global_path || !local_path) return false;
  if (mode.empty() && !switching_) {
    saved_local_=local->isEnabled(); saved_global_path_=global_path->isEnabled(); saved_local_path_=local_path->isEnabled(); switching_=true;
    local->setEnabled(false); global_path->setEnabled(false); local_path->setEnabled(false);
  }
  cost->setEnabled(mode=="costmap2d"); grid->setEnabled(mode=="gridmap"); oct->setEnabled(mode=="octomap");
  if (!mode.empty() && switching_) {
    local->setEnabled(saved_local_); global_path->setEnabled(saved_global_path_); local_path->setEnabled(saved_local_path_); switching_=false;
  }
  return true;
}
void NavigationModePanel::updateState() {
  // 顺序处理状态，避免切换过快时遗漏空状态及旧地图清理。
  for (;;) {
    std::string mode;
    { std::lock_guard<std::mutex> lock(state_->mutex); if (state_->modes.empty()) break; mode=state_->modes.front(); }
    if (!applyMode(mode)) break;
    { std::lock_guard<std::mutex> lock(state_->mutex); state_->modes.pop_front(); }
    status_->setText(mode.empty() ? "Switch in progress" : QString("Active map: %1").arg(QString::fromStdString(mode)));
  }
  if (!pending_) return;
  if (future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    auto result=future_.get(); pending_=false; apply_->setEnabled(true);
    if (result->results.empty() || !result->results[0].successful) {
      const QString reason=result->results.empty() ? "No response" : QString::fromStdString(result->results[0].reason);
      status_->setText("Mode rejected: "+reason);
    }
  } else if (std::chrono::steady_clock::now()-request_started_ > std::chrono::seconds(10)) {
    client_->remove_pending_request(request_id_); pending_=false; apply_->setEnabled(true); status_->setText("Mode request timed out");
  }
}
}
PLUGINLIB_EXPORT_CLASS(clearing_robot::NavigationModePanel,rviz_common::Panel)
