#include "clearing_robot/local_control/controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <pluginlib/class_list_macros.hpp>
#ifdef CLEARING_HAS_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>
#endif

namespace clearing_robot
{

std::shared_ptr<const LocalScene> ControlInputs::snapshot(
  const rclcpp::Time & time, geometry_msgs::msg::Twist & applied, bool & feedback_valid) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const double feedback_age = time.seconds() - applied_time_;
  feedback_valid = applied_time_ >= 0.0 && feedback_age >= 0.0 && feedback_age <= 0.5;
  applied = feedback_valid ? applied_ : geometry_msgs::msg::Twist{};
  if (!scene_ || !scene_->valid) {
    return {};
  }
  const double age = (time - rclcpp::Time(scene_->header.stamp, time.get_clock_type())).seconds();
  return age < 0.0 || age > 1.5 ? nullptr : scene_;
}

void MppiController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("控制器节点已释放");
  }
  node_ = parent;
  inputs_ = std::make_unique<ControlInputs>(*node);
}

void MppiController::activate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = true;
}

void MppiController::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  global_ = {};
  local_ = {};
  reached_ = false;
  local_ends_at_goal_ = false;
  last_plan_reaches_goal_ = false;
}

void MppiController::cleanup()
{
  deactivate();
  inputs_.reset();
  node_.reset();
}

void MppiController::shutdown() {cleanup();}

void MppiController::setGlobalPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  global_ = path;
}

nav_msgs::msg::Path MppiController::computeLocalPath(const geometry_msgs::msg::PoseStamped & pose)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto node = node_.lock();
  last_plan_reaches_goal_ = false;
  if (!node || !active_ || isCancelRequested()) {
    return {};
  }
  geometry_msgs::msg::Twist applied;
  bool feedback_valid = false;
  const auto scene = inputs_->snapshot(node->now(), applied, feedback_valid);
  if (!scene || pose.header.frame_id != scene->header.frame_id) {
    return {};
  }
  return mppi_.localPath(*scene, Mppi::state(pose.pose, {}), global_, last_plan_reaches_goal_);
}

bool MppiController::lastPlanReachesGoal() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_plan_reaches_goal_;
}

void MppiController::setLocalPlan(const nav_msgs::msg::Path & path, bool ends_at_goal)
{
  std::lock_guard<std::mutex> lock(mutex_);
  local_ = path;
  local_ends_at_goal_ = ends_at_goal;
  reached_ = false;
}

geometry_msgs::msg::TwistStamped MppiController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity)
{
  std::lock_guard<std::mutex> lock(mutex_);
  geometry_msgs::msg::TwistStamped command;
  auto node = node_.lock();
  if (!node) {
    return command;
  }
  command.header.frame_id = "base_link";
  command.header.stamp = node->now();
  if (!active_ || local_.poses.empty() || isCancelRequested()) {
    return command;
  }
  geometry_msgs::msg::Twist applied;
  bool feedback_valid = false;
  const auto scene = inputs_->snapshot(node->now(), applied, feedback_valid);
  if (!scene || pose.header.frame_id != scene->header.frame_id || !feedback_valid) {
    return command;
  }
  const auto state = Mppi::state(pose.pose, velocity);
  const auto & goal = local_.poses.back().pose.position;
  reached_ = local_ends_at_goal_ &&
    std::hypot(state.x - goal.x, state.y - goal.y) <= Mppi::goal_tolerance &&
    std::abs(state.z - goal.z) <= 0.50 && std::abs(state.v) <= 0.06 && std::abs(state.w) <= 0.04;
  if (reached_ || !scene->traversable(state) || (local_ends_at_goal_ &&
    std::hypot(state.x - goal.x, state.y - goal.y) <= Mppi::goal_tolerance))
  {
    return command;
  }
  const auto prior = mppi_.command(*scene, state, local_, applied, local_ends_at_goal_);
  command.twist = selectCommand(*scene, pose.pose, velocity, applied, prior, local_, feedback_valid);
  return command;
}

bool MppiController::isGoalReached()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return reached_;
}

geometry_msgs::msg::Twist MppiController::selectCommand(
  const LocalScene &, const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Twist &,
  const geometry_msgs::msg::Twist &, const geometry_msgs::msg::Twist & prior,
  const nav_msgs::msg::Path &, bool)
{
  return prior;
}

struct DrlController::Policy
{
#ifdef CLEARING_HAS_ONNXRUNTIME
  Ort::Env environment{ORT_LOGGING_LEVEL_WARNING, "clearing_robot"};
  Ort::Session session{nullptr};
  bool base_only{false};

  /** @brief 检查模型张量名、类型和形状，排除不兼容的部署模型。 */
  void checkTensor(
    const char * name, const char * expected, const Ort::TypeInfo & info, int64_t width)
  {
    const auto tensor = info.GetTensorTypeAndShapeInfo();
    if (std::string(name) != expected || tensor.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
      tensor.GetShape() != std::vector<int64_t>{1, width})
    {
      throw std::runtime_error(std::string("ONNX 张量接口错误: ") + expected);
    }
  }

  /** @brief 加载 CPU 会话并检查合同。 */
  explicit Policy(const std::string & path)
  {
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    options.SetInterOpNumThreads(1);
    session = Ort::Session(environment, path.c_str(), options);
    if (session.GetInputCount() != 2 || session.GetOutputCount() != 1) {
      throw std::runtime_error("ONNX 模型需要 observation、prior 输入与 action 输出");
    }
    Ort::AllocatorWithDefaultOptions allocator;
    const auto metadata = session.GetModelMetadata();
    const auto mode = metadata.LookupCustomMetadataMapAllocated("observation_variant", allocator);
    base_only = mode && std::string(mode.get()) == "base";
    checkTensor(session.GetInputNameAllocated(0, allocator).get(), "observation",
      session.GetInputTypeInfo(0), 28);
    checkTensor(session.GetInputNameAllocated(1, allocator).get(), "prior",
      session.GetInputTypeInfo(1), 2);
    checkTensor(session.GetOutputNameAllocated(0, allocator).get(), "action",
      session.GetOutputTypeInfo(0), 2);
  }

  /** @brief 运行一个样本，返回双曲正切压缩后的动作。 */
  Action run(Observation observation, Action prior)
  {
    if (base_only) {
      std::fill(observation.begin() + 20, observation.end(), 0.0f);
    }
    auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    const std::array<int64_t, 2> obs_shape{1, 28};
    const std::array<int64_t, 2> action_shape{1, 2};
    std::array<Ort::Value, 2> inputs{
      Ort::Value::CreateTensor<float>(memory, observation.data(), observation.size(),
        obs_shape.data(), obs_shape.size()),
      Ort::Value::CreateTensor<float>(memory, prior.data(), prior.size(),
        action_shape.data(), action_shape.size())};
    const std::array<const char *, 2> input_names{"observation", "prior"};
    const std::array<const char *, 1> output_names{"action"};
    auto outputs = session.Run(Ort::RunOptions{nullptr}, input_names.data(), inputs.data(),
      inputs.size(), output_names.data(), output_names.size());
    const auto * action = outputs.front().GetTensorData<float>();
    if (!std::isfinite(action[0]) || !std::isfinite(action[1])) {
      throw std::runtime_error("ONNX 输出不是有限值");
    }
    return {action[0], action[1]};
  }
#endif
};

DrlController::DrlController() = default;
DrlController::~DrlController() = default;

void DrlController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name)
{
#ifndef CLEARING_HAS_ONNXRUNTIME
  (void)parent;
  (void)name;
  throw std::runtime_error("本构建缺少 ONNX Runtime C++ SDK；配置 ONNXRUNTIME_ROOT 后重新构建");
#else
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("控制器节点已释放");
  }
  const std::string key = name + ".onnx_model";
  if (!node->has_parameter(key)) {
    node->declare_parameter<std::string>(key, "");
  }
  const auto model = node->get_parameter(key).as_string();
  if (model.empty()) {
    throw std::runtime_error("DRL 部署需要 onnx_model 参数");
  }
  policy_ = std::make_unique<Policy>(model);
  MppiController::configure(parent, std::move(name));
#endif
}

void DrlController::cleanup()
{
  MppiController::cleanup();
  policy_.reset();
}

geometry_msgs::msg::Twist DrlController::selectCommand(
  const LocalScene & scene, const geometry_msgs::msg::Pose & pose,
  const geometry_msgs::msg::Twist & velocity, const geometry_msgs::msg::Twist & applied,
  const geometry_msgs::msg::Twist & prior, const nav_msgs::msg::Path & path, bool feedback_valid)
{
#ifdef CLEARING_HAS_ONNXRUNTIME
  try {
    const auto action = policy_->run(mppi_.observation(scene, pose, velocity, path, applied,
      feedback_valid), Mppi::normalize(prior));
    return mppi_.shield(scene, Mppi::state(pose, velocity), applied, Mppi::decode(action), prior);
  } catch (const std::exception & error) {
    if (auto node = node_.lock()) {
      RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
        "策略推理异常，回退到已校验 MPPI: %s", error.what());
    }
  }
#else
  (void)scene;
  (void)pose;
  (void)velocity;
  (void)applied;
  (void)path;
  (void)feedback_valid;
#endif
  return prior;
}

}  // namespace clearing_robot

PLUGINLIB_EXPORT_CLASS(clearing_robot::MppiController, multimap_nav_core::LocalControllerBase)
PLUGINLIB_EXPORT_CLASS(clearing_robot::DrlController, multimap_nav_core::LocalControllerBase)
