#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <limits>
#include <memory>
#include <multimap_nav_core/global_planner_base.hpp>
#include <multimap_nav_msgs/msg/terrain_costmap2_d.hpp>
#include <mutex>
#include <octomap_msgs/msg/octomap.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unordered_map>
#include <vector>

namespace clearing_robot
{
namespace
{
constexpr double pi = 3.14159265358979323846;
/** @brief 计算角度的主值，输入输出为弧度。 */
double wrap(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}
struct State
{
  double x, y, z, yaw;
};
struct Key
{
  int x, y, yaw;
  bool operator==(const Key &o) const
  {
    return x == o.x && y == o.y && yaw == o.yaw;
  }
};
struct Hash
{
  size_t operator()(const Key &k) const
  {
    return std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1) ^ (std::hash<int>()(k.yaw) << 2);
  }
};
}  // namespace
/** @brief 共享前向圆弧运动模型和完整车体扫描的混合 A*。 */
class AStarBase : public multimap_nav_core::GlobalPlannerBase
{
 public:
  /** @brief 创建固定地图适配模式的 A*。 */
  explicit AStarBase(int mode) : mode_(mode) {}
  /** @brief 由生命周期节点加载米制车体、地图及搜索参数。 */
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name) override
  {
    node_ = parent.lock();
    if (!node_)
      throw std::runtime_error("planner node expired");
    name_ = std::move(name);
    auto param = [&](const std::string &key, auto value)
    {
      const auto n = name_ + "." + key;
      if (!node_->has_parameter(n))
        node_->declare_parameter(n, value);
      return node_->get_parameter(n).get_value<decltype(value)>();
    };
    topic_ = param("map_topic", std::string(mode_ == 0   ? "/terrain_costmap"
                                            : mode_ == 1 ? "/grid_map"
                                                         : "/octomap_map"));
    resolution_ = param("search_resolution", 0.35);
    step_ = param("primitive_length", 0.7);
    radius_ = param("min_turn_radius", 2.0);
    length_ = param("robot_length", 9.5);
    offset_ = param("robot_x_offset", 1.3);
    width_ = param("robot_width", 3.5);
    height_ = param("robot_height", 4.3);
    max_step_ = param("max_step_height", 0.22);
    max_slope_ = param("max_slope", 0.6);
    velocity_ = param("nominal_speed", 0.35);
    planning_timeout_ = param("planning_timeout", 8.0);
    lateral_accel_ = param("max_lateral_acceleration", 0.4);
    yaw_bins_ = static_cast<int>(param("yaw_bins", int64_t(32)));
    max_nodes_ = static_cast<int>(param("max_nodes", int64_t(150000)));
    if (resolution_ <= 0 || step_ <= 0 || radius_ <= 0 || length_ <= 0 || width_ <= 0 ||
        height_ <= 0 || max_step_ <= 0 || max_slope_ <= 0 || max_slope_ >= pi / 2 ||
        velocity_ <= 0 || lateral_accel_ <= 0 || yaw_bins_ < 8 || max_nodes_ <= 0 ||
        !std::isfinite(planning_timeout_) || planning_timeout_ <= 0.0)
      throw std::runtime_error("invalid A* parameters");
  }
  /** @brief 订阅选择器装配的静态地图，读取最近一张完整地图。 */
  void activate() override
  {
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    if (mode_ == 0)
      cost_sub_ = node_->create_subscription<multimap_nav_msgs::msg::TerrainCostmap2D>(
          topic_, qos,
          [this](const multimap_nav_msgs::msg::TerrainCostmap2D::SharedPtr msg)
          {
            std::lock_guard<std::mutex> l(map_mutex_);
            latest_cost_ = msg;
          });
    if (mode_ == 1)
      grid_sub_ = node_->create_subscription<grid_map_msgs::msg::GridMap>(
          topic_, qos,
          [this](const grid_map_msgs::msg::GridMap::SharedPtr msg)
          {
            auto m = std::make_shared<grid_map::GridMap>();
            if (grid_map::GridMapRosConverter::fromMessage(*msg, *m))
            {
              std::lock_guard<std::mutex> l(map_mutex_);
              latest_grid_ = m;
            }
          });
    if (mode_ == 2)
      oct_sub_ = node_->create_subscription<octomap_msgs::msg::Octomap>(
          topic_, qos,
          [this](const octomap_msgs::msg::Octomap::SharedPtr msg)
          {
            std::unique_ptr<octomap::AbstractOcTree> raw(octomap_msgs::msgToMap(*msg));
            auto *p = dynamic_cast<octomap::OcTree *>(raw.get());
            if (!p)
              return;
            raw.release();
            std::lock_guard<std::mutex> l(map_mutex_);
            latest_oct_.reset(p);
            latest_oct_frame_ = msg->header.frame_id;
          });
  }
  /** @brief 停止接收地图更新。 */
  void deactivate() override
  {
    cost_sub_.reset();
    grid_sub_.reset();
    oct_sub_.reset();
  }
  /** @brief 释放订阅和地图对象。 */
  void cleanup() override
  {
    deactivate();
    std::lock_guard<std::mutex> operation(operation_mutex_);
    std::lock_guard<std::mutex> maps(map_mutex_);
    cost_.reset();
    grid_.reset();
    oct_.reset();
    oct_frame_.clear();
    latest_cost_.reset();
    latest_grid_.reset();
    latest_oct_.reset();
    latest_oct_frame_.clear();
  }
  /** @brief 关闭规划插件并清理资源。 */
  void shutdown() override
  {
    cleanup();
  }
  /** @brief 在同一地图坐标系中搜索前向几何路径，失败返回空路径。 */
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,
                                 const geometry_msgs::msg::PoseStamped &goal) override
  {
    std::lock_guard<std::mutex> operation(operation_mutex_);
    snapshotMap();
    deadline_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(planning_timeout_));
    struct ResetDeadline {
      std::chrono::steady_clock::time_point & deadline;
      ~ResetDeadline() { deadline = std::chrono::steady_clock::time_point::max(); }
    } reset_deadline{deadline_};
    nav_msgs::msg::Path path;
    path.header = start.header;
    const std::string frame = mode_ == 0 && cost_   ? cost_->header.frame_id
                              : mode_ == 1 && grid_ ? grid_->getFrameId()
                              : mode_ == 2 && oct_  ? oct_frame_
                                                    : "";
    if (frame.empty() || start.header.frame_id != frame || goal.header.frame_id != frame)
      return path;
    State begin{start.pose.position.x, start.pose.position.y, start.pose.position.z,
                tf2::getYaw(start.pose.orientation)};
    State target{goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
                 tf2::getYaw(goal.pose.orientation)};
    if (!std::isfinite(begin.x) || !std::isfinite(begin.y) || !std::isfinite(begin.z) ||
        !std::isfinite(begin.yaw) || !std::isfinite(target.x) || !std::isfinite(target.y) ||
        !std::isfinite(target.z) || !std::isfinite(target.yaw) || !valid(begin))
      return path;
    auto heuristic = [&](const State &state)
    {
      const double distance = std::hypot(target.x - state.x, target.y - state.y);
      const double rotation = radius_ * std::abs(wrap(target.yaw - state.yaw));
      return std::max(distance, rotation) / velocity_;
    };
    struct Node
    {
      State s;
      double g;
      int parent;
      double curvature;
    };
    struct Item
    {
      double f;
      int i;
      bool operator<(const Item &o) const
      {
        return f > o.f;
      }
    };
    std::vector<Node> nodes{{begin, 0, -1, 0.0}};
    std::priority_queue<Item> open;
    open.push({heuristic(begin), 0});
    std::unordered_map<Key, double, Hash> best;
    best[key(begin)] = 0;
    int finish = -1;
    std::vector<State> final_edge;
    const double direct_distance = std::hypot(target.x - begin.x, target.y - begin.y);
    if (direct_distance > 1e-6 && connect(begin, target, final_edge))
    {
      double length = 0.0;
      State previous = begin;
      for (const auto &point : final_edge)
      {
        length += std::hypot(point.x - previous.x, point.y - previous.y);
        previous = point;
      }
      // 明显绕圈的 Dubins 连接交由栅格搜索，避免为了末端航向多行驶一圈。
      if (length <= 1.10 * direct_distance + 0.25)
        finish = 0;
    }
    while (finish < 0 && !open.empty() && static_cast<int>(nodes.size()) < max_nodes_)
    {
      if (interrupted())
        return path;
      const auto top = open.top();
      open.pop();
      const Node current = nodes[top.i];
      if (current.g > best[key(current.s)] + 1e-8)
        continue;
      const double dist = std::hypot(target.x - current.s.x, target.y - current.s.y);
      if (dist <= step_ * 2.0 && connect(current.s, target, final_edge))
      {
        finish = top.i;
        break;
      }
      for (double curvature : {-1.0 / radius_, 0.0, 1.0 / radius_})
      {
        State next;
        std::vector<State> edge;
        if (!advance(current.s, curvature, step_, next, edge))
          continue;
        const double speed =
            std::min(velocity_, std::sqrt(lateral_accel_ / std::max(std::abs(curvature), 1e-9)));
        double grade = std::abs(next.z - current.s.z) / step_;
        const double bending = 0.06 * radius_ * std::abs(curvature);
        const double switching = 0.10 * radius_ * std::abs(curvature - current.curvature);
        const double g = current.g + step_ / speed * (1.0 + grade + bending + switching);
        const Key k = key(next);
        auto it = best.find(k);
        if (it != best.end() && it->second <= g)
          continue;
        best[k] = g;
        const int index = static_cast<int>(nodes.size());
        nodes.push_back({next, g, top.i, curvature});
        open.push({g + heuristic(next), index});
      }
    }
    if (finish < 0)
      return path;
    std::vector<int> chain;
    for (int i = finish; i >= 0; i = nodes[i].parent)
      chain.push_back(i);
    std::reverse(chain.begin(), chain.end());
    auto add = [&](const State &s)
    {
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = s.x;
      p.pose.position.y = s.y;
      p.pose.position.z = s.z;
      tf2::Quaternion q;
      q.setRPY(0, 0, s.yaw);
      p.pose.orientation.x = q.x();
      p.pose.orientation.y = q.y();
      p.pose.orientation.z = q.z();
      p.pose.orientation.w = q.w();
      path.poses.push_back(p);
    };
    add(nodes[chain.front()].s);
    for (size_t j = 1; j < chain.size(); ++j)
    {
      const auto &a = nodes[chain[j - 1]].s;
      const auto &b = nodes[chain[j]].s;
      const double curvature = wrap(b.yaw - a.yaw) / step_;
      State end;
      std::vector<State> edge;
      if (!advance(a, curvature, step_, end, edge))
        return nav_msgs::msg::Path();
      for (const auto &s : edge)
        add(s);
    }
    for (const auto &s : final_edge)
      add(s);
    return path;
  }

 protected:
  /** @brief 对平滑结果和缓存路径重做运动学及扫掠碰撞检查。 */
  bool validateSmoothedPath(const nav_msgs::msg::Path &path) const override
  {
    std::lock_guard<std::mutex> operation(operation_mutex_);
    snapshotMap();
    const std::string frame = mode_ == 0 && cost_   ? cost_->header.frame_id
                              : mode_ == 1 && grid_ ? grid_->getFrameId()
                              : mode_ == 2 && oct_  ? oct_frame_
                                                    : "";
    if (path.poses.empty() || frame.empty() || path.header.frame_id != frame)
      return false;
    const auto state = [](const geometry_msgs::msg::PoseStamped &p)
    {
      return State{p.pose.position.x, p.pose.position.y, p.pose.position.z,
                   tf2::getYaw(p.pose.orientation)};
    };
    State a = state(path.poses.front());
    if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z) ||
        !std::isfinite(a.yaw) || !valid(a))
      return false;
    for (size_t i = 1; i < path.poses.size(); ++i)
    {
      if (interrupted()) return false;
      const State b = state(path.poses[i]);
      if (!std::isfinite(b.x) || !std::isfinite(b.y) || !std::isfinite(b.z) ||
          !std::isfinite(b.yaw))
        return false;
      const double chord = std::hypot(b.x - a.x, b.y - a.y), angle = wrap(b.yaw - a.yaw);
      if (chord < 1e-8)
      {
        if (std::abs(angle) > 1e-4 || std::abs(b.z - a.z) > max_step_)
          return false;
        continue;
      }
      const double arc = std::abs(angle) < 1e-8
                             ? chord
                             : chord * std::abs(angle) / (2 * std::sin(std::abs(angle) / 2));
      const double curvature = angle / arc;
      if (std::abs(curvature) > 1 / radius_ + 1e-3)
        return false;
      State end;
      std::vector<State> edge;
      if (!advance(a, curvature, arc, end, edge) || std::hypot(end.x - b.x, end.y - b.y) > 0.02 ||
          std::abs(end.z - b.z) > max_step_)
        return false;
      a = end;
    }
    return true;
  }

 private:
  /** @brief 复制最近发布的不可变地图指针；调用方必须持有操作锁。 */
  void snapshotMap() const
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    cost_ = latest_cost_;
    grid_ = latest_grid_;
    oct_ = latest_oct_;
    oct_frame_ = latest_oct_frame_;
  }
  /** @brief 检查当前目标的取消请求及规划墙钟期限。 */
  bool interrupted() const
  {
    return isCancelRequested() || std::chrono::steady_clock::now() >= deadline_;
  }
  /** @brief 把位姿离散为 x、y 平面网格与航向编号。 */
  Key key(const State &s) const
  {
    int yaw = static_cast<int>(std::floor((wrap(s.yaw) + pi) * yaw_bins_ / (2 * pi))) % yaw_bins_;
    return {static_cast<int>(std::floor(s.x / resolution_)),
            static_cast<int>(std::floor(s.y / resolution_)), yaw};
  }
  /** @brief 按选中的地图格式查询支撑面高程、坡度及障碍。 */
  bool surface(double x, double y, double ref, double &z, double &slope) const
  {
    slope = 0;
    if (mode_ == 0)
    {
      const auto &m = *cost_;
      const auto &info = m.grid.info;
      if (info.resolution <= 0 || std::abs(tf2::getYaw(info.origin.orientation)) > 1e-6)
        return false;
      const int ix = static_cast<int>(std::floor((x - info.origin.position.x) / info.resolution));
      const int iy = static_cast<int>(std::floor((y - info.origin.position.y) / info.resolution));
      if (ix < 0 || iy < 0 || ix >= static_cast<int>(info.width) ||
          iy >= static_cast<int>(info.height))
        return false;
      const size_t i = static_cast<size_t>(iy) * info.width + ix;
      if (i >= m.grid.data.size() || i >= m.plane_height.size() || i >= m.plane_slope.size() ||
          m.grid.data[i] < 0 || m.grid.data[i] >= 50)
        return false;
      z = m.plane_height[i];
      slope = m.plane_slope[i];
      return std::isfinite(z) && std::isfinite(slope) && slope <= max_slope_;
    }
    if (mode_ == 1)
    {
      grid_map::Index i;
      if (!grid_->getIndex(grid_map::Position(x, y), i) || !grid_->exists("elevation"))
        return false;
      z = grid_->at("elevation", i);
      if (!std::isfinite(z))
        return false;
      if (grid_->exists("slope"))
      {
        slope = grid_->at("slope", i);
        if (!std::isfinite(slope) || slope > max_slope_)
          return false;
      }
      if (grid_->exists("obstacle"))
      {
        const double v = grid_->at("obstacle", i);
        if (!std::isfinite(v) || v > 0.5)
          return false;
      }
      if (grid_->exists("traversability"))
      {
        const double v = grid_->at("traversability", i);
        if (!std::isfinite(v) || v < 0.3)
          return false;
      }
      return true;
    }
    const double r = oct_->getResolution();
    double best = std::numeric_limits<double>::infinity();
    bool found = false;
    for (double h = ref - max_step_ - r; h <= ref + max_step_ + r; h += r * 0.5)
    {
      const auto *n = oct_->search(x, y, h);
      if (!n || !oct_->isNodeOccupied(n))
        continue;
      const double top = std::floor(h / r) * r + r;
      const double d = std::abs(top - ref);
      if (d < best)
      {
        best = d;
        z = top;
        found = true;
      }
    }
    if (!found || best > max_step_ + r)
      return false;
    for (double h = z + r * 0.5; h < z + height_; h += r * 0.5)
    {
      const auto *n = oct_->search(x, y, h);
      if (n && oct_->isNodeOccupied(n))
        return false;
    }
    return true;
  }
  /** @brief 获取扫掠检查沿车体和路径的采样间距（米）。 */
  double collisionSpacing() const
  {
    const double map_resolution = mode_ == 0   ? cost_->grid.info.resolution
                                  : mode_ == 1 ? grid_->getResolution()
                                               : oct_->getResolution();
    return std::min(resolution_, std::max(0.005, map_resolution * 0.5));
  }
  /** @brief 检查车体完整包络的支承与占据，并投影中心高程。 */
  bool valid(State &s) const
  {
    if (interrupted()) return false;
    double z, slope;
    if (!surface(s.x, s.y, s.z, z, slope))
      return false;
    // 保持占据与车体包络的单元检查。中心高度插值避免用半格距离
    // 除以连续坡面上的整格高度变化，造成陡坡或台阶的误判。
    if (mode_ == 1)
    {
      const double interpolated = grid_->atPosition(
          "elevation", grid_map::Position(s.x, s.y), grid_map::InterpolationMethods::INTER_LINEAR);
      if (std::isfinite(interpolated))
        z = interpolated;
    }
    if (std::abs(z - s.z) > max_step_ + (mode_ == 2 ? oct_->getResolution() : 0.0))
      return false;
    const double spacing = collisionSpacing();
    const int nx = std::max(1, static_cast<int>(std::ceil(length_ / spacing)));
    const int ny = std::max(1, static_cast<int>(std::ceil(width_ / spacing)));
    for (int i = 0; i <= nx; ++i)
    {
      for (int j = 0; j <= ny; ++j)
      {
        const double dx = offset_ + length_ * (static_cast<double>(i) / nx - 0.5),
                     dy = width_ * (static_cast<double>(j) / ny - 0.5);
        const double x = s.x + dx * std::cos(s.yaw) - dy * std::sin(s.yaw),
                     y = s.y + dx * std::sin(s.yaw) + dy * std::cos(s.yaw);
        double other, local_slope;
        if (!surface(x, y, z, other, local_slope))
          return false;
        const double d = std::hypot(dx, dy);
        if (std::abs(other - z) > max_step_ + std::tan(max_slope_) * d)
          return false;
      }
    }
    s.z = z;
    return true;
  }
  /** @brief 积分定曲率运动边并检查整个车体扫掠，返回可行样点。 */
  bool advance(const State &from, double curvature, double distance, State &end,
               std::vector<State> &edge) const
  {
    edge.clear();
    State previous = from;
    const int count = std::max(1, static_cast<int>(std::ceil(distance / collisionSpacing())));
    for (int i = 1; i <= count; ++i)
    {
      if (interrupted()) return false;
      const double d = distance * i / count, angle = from.yaw + curvature * d;
      State s = from;
      if (std::abs(curvature) < 1e-9)
      {
        s.x += d * std::cos(from.yaw);
        s.y += d * std::sin(from.yaw);
      }
      else
      {
        s.x += (std::sin(angle) - std::sin(from.yaw)) / curvature;
        s.y += (-std::cos(angle) + std::cos(from.yaw)) / curvature;
      }
      s.yaw = wrap(angle);
      s.z = previous.z;
      if (!valid(s) || std::abs(s.z - previous.z) > max_step_ ||
          std::atan2(std::abs(s.z - previous.z), distance / count) > max_slope_)
        return false;
      edge.push_back(s);
      previous = s;
    }
    end = previous;
    return true;
  }
  /** @brief 用前向 Dubins 曲线连接精确终点并逐段重新碰撞检查。 */
  bool connect(const State &a, const State &b, std::vector<State> &edge) const
  {
    const double dx = b.x - a.x, dy = b.y - a.y, distance = std::hypot(dx, dy);
    auto mod = [](double x)
    {
      x = std::fmod(x, 2 * pi);
      return x < 0 ? x + 2 * pi : x;
    };
    if (distance < 1e-6 && std::abs(wrap(a.yaw - b.yaw)) < 1e-4)
    {
      edge.clear();
      return std::abs(a.z - b.z) <= max_step_;
    }
    const double theta = std::atan2(dy, dx), d = distance / radius_;
    const double alpha = mod(a.yaw - theta), beta = mod(b.yaw - theta);
    const double sa = std::sin(alpha), sb = std::sin(beta), ca = std::cos(alpha),
                 cb = std::cos(beta);
    struct Candidate
    {
      double t, p, q;
      int first, last;
    };
    std::vector<Candidate> options;
    auto add = [&](double p2, double t, double q, int first, int last)
    {
      if (p2 >= 0)
        options.push_back({mod(t), std::sqrt(p2), mod(q), first, last});
    };
    {
      const double p2 = 2 + d * d - 2 * std::cos(alpha - beta) + 2 * d * (sa - sb);
      const double v = std::atan2(cb - ca, d + sa - sb);
      add(p2, -alpha + v, beta - v, 1, 1);
    }
    {
      const double p2 = 2 + d * d - 2 * std::cos(alpha - beta) + 2 * d * (-sa + sb);
      const double v = std::atan2(ca - cb, d - sa + sb);
      add(p2, alpha - v, -beta + v, -1, -1);
    }
    {
      const double p2 = -2 + d * d + 2 * std::cos(alpha - beta) + 2 * d * (sa + sb);
      if (p2 >= 0)
      {
        const double v = std::atan2(-ca - cb, d + sa + sb) - std::atan2(-2.0, std::sqrt(p2));
        add(p2, -alpha + v, -beta + v, 1, -1);
      }
    }
    {
      const double p2 = d * d - 2 + 2 * std::cos(alpha - beta) - 2 * d * (sa + sb);
      if (p2 >= 0)
      {
        const double v = std::atan2(ca + cb, d - sa - sb) - std::atan2(2.0, std::sqrt(p2));
        add(p2, alpha - v, beta - v, -1, 1);
      }
    }
    std::sort(options.begin(), options.end(),
              [](const Candidate &x, const Candidate &y)
              { return x.t + x.p + x.q < y.t + y.p + y.q; });
    for (const auto &c : options)
    {
      if (interrupted()) return false;
      State current = a, next;
      std::vector<State> result, part;
      bool ok = true;
      for (const auto &segment : {std::pair<double, double>{c.first / radius_, c.t * radius_},
                                  {0.0, c.p * radius_},
                                  {c.last / radius_, c.q * radius_}})
      {
        if (segment.second < 1e-8)
          continue;
        if (!advance(current, segment.first, segment.second, next, part))
        {
          ok = false;
          break;
        }
        result.insert(result.end(), part.begin(), part.end());
        current = next;
      }
      if (ok && std::hypot(current.x - b.x, current.y - b.y) < 1e-4 &&
          std::abs(wrap(current.yaw - b.yaw)) < 1e-4 && std::abs(current.z - b.z) <= max_step_)
      {
        edge = std::move(result);
        return true;
      }
    }
    return false;
  }

  int mode_;
  std::string name_, topic_;
  mutable std::string oct_frame_;
  double resolution_, step_, radius_, length_, offset_, width_, height_, max_step_, max_slope_,
      velocity_, lateral_accel_, planning_timeout_;
  int yaw_bins_, max_nodes_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  // 操作锁串行化搜索和校验；地图回调只需短暂取得地图锁。
  mutable std::mutex operation_mutex_, map_mutex_;
  mutable std::shared_ptr<const multimap_nav_msgs::msg::TerrainCostmap2D> cost_;
  mutable std::shared_ptr<const grid_map::GridMap> grid_;
  mutable std::shared_ptr<const octomap::OcTree> oct_;
  std::shared_ptr<const multimap_nav_msgs::msg::TerrainCostmap2D> latest_cost_;
  std::shared_ptr<const grid_map::GridMap> latest_grid_;
  std::shared_ptr<const octomap::OcTree> latest_oct_;
  std::string latest_oct_frame_;
  mutable std::chrono::steady_clock::time_point deadline_ = std::chrono::steady_clock::time_point::max();
  rclcpp::Subscription<multimap_nav_msgs::msg::TerrainCostmap2D>::SharedPtr cost_sub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr oct_sub_;
};
class CostmapAStar : public AStarBase
{
 public:
  CostmapAStar() : AStarBase(0) {}
};
class GridMapAStar : public AStarBase
{
 public:
  GridMapAStar() : AStarBase(1) {}
};
class OctomapAStar : public AStarBase
{
 public:
  OctomapAStar() : AStarBase(2) {}
};
}  // namespace clearing_robot
PLUGINLIB_EXPORT_CLASS(clearing_robot::CostmapAStar, multimap_nav_core::GlobalPlannerBase)
PLUGINLIB_EXPORT_CLASS(clearing_robot::GridMapAStar, multimap_nav_core::GlobalPlannerBase)
PLUGINLIB_EXPORT_CLASS(clearing_robot::OctomapAStar, multimap_nav_core::GlobalPlannerBase)
