#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <deque>
#include <cmath>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <string>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <vector>

#include "clearing_robot_plugins/msg/compound_local_map.hpp"

namespace clearing_robot {
/** @brief 判断整个体素是否位于以机器人为中心的公开 10 米立方窗口。 */
bool voxelWithinWindow(double x, double y, double z, double size,
                       double robot_x, double robot_y, double bottom) {
  const double h = size * 0.5, tolerance = 1e-7;
  return x - h >= robot_x - 5.0 - tolerance &&
         x + h <= robot_x + 5.0 + tolerance &&
         y - h >= robot_y - 5.0 - tolerance &&
         y + h <= robot_y + 5.0 + tolerance && z - h >= bottom - tolerance &&
         z + h <= bottom + 10.0 + tolerance;
}
/** @brief 在 map 坐标系构建跟随机器人的局部地形及体素地图。 */
class CompoundLocalMapNode : public rclcpp::Node {
public:
  /** @brief 加载先验 PCD、地图滑窗和点云时效配置。 */
  CompoundLocalMapNode()
      : Node("compound_local_map"), buffer_(get_clock()), listener_(buffer_) {
    frame_ = declare_parameter<std::string>("map_frame", "map");
    base_ = declare_parameter<std::string>("base_frame", "base_link");
    const auto input =
        declare_parameter<std::string>("points_topic", "/points_raw");
    const auto pcd = declare_parameter<std::string>("prior_pcd", "");
    length_ = declare_parameter<double>("length", 44.0);
    resolution_ = declare_parameter<double>("resolution", 0.2);
    voxel_ = declare_parameter<double>("voxel_resolution", 0.2);
    control_resolution_ = declare_parameter<double>("control_resolution", 0.35);
    control_voxel_ =
        declare_parameter<double>("control_voxel_resolution", 0.30);
    bottom_offset_ = declare_parameter<double>("base_bottom_offset", -0.01);
    ground_clearance_ = declare_parameter<double>("ground_clearance", 0.10);
    ttl_ = declare_parameter<double>("dynamic_ttl", 2.0);
    const double rate = declare_parameter<double>("publish_rate", 2.0);
    max_scan_age_ = declare_parameter<double>("max_scan_age", 1.0);
    if (length_ <= 0 || resolution_ <= 0 || voxel_ <= 0 || ttl_ <= 0 ||
        rate <= 0 || max_scan_age_ <= 0 || ground_clearance_ < 0 ||
        !std::isfinite(bottom_offset_) || control_resolution_ <= 0 ||
        control_voxel_ <= 0 ||
        (std::abs(resolution_ - 0.2) > 1e-6 &&
         std::abs(resolution_ - 0.25) > 1e-6)) {
      throw std::invalid_argument(
          "local map dimensions, TTL and publish rate must be positive");
    }
    if (!pcd.empty() && pcl::io::loadPCDFile(pcd, prior_) < 0) {
      throw std::runtime_error("cannot load prior_pcd: " + pcd);
    }
    pub_ = create_publisher<clearing_robot_plugins::msg::CompoundLocalMap>(
        "/clearing/local_map", rclcpp::QoS(1).reliable().transient_local());
    control_pub_ =
        create_publisher<clearing_robot_plugins::msg::CompoundLocalMap>(
            "/clearing/control_map",
            rclcpp::QoS(1).reliable().transient_local());
    terrain_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(
        "/clearing/local_map/terrain", 1);
    occupancy_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
        "/clearing/local_map/occupancy", 1);
    health_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/localization/healthy", rclcpp::QoS(1).reliable(),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          healthy_ = msg->data;
          health_time_ = std::chrono::steady_clock::now();
          if (!healthy_)
            scans_.clear();
        });
    reset_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/clearing/estimator_reset", rclcpp::QoS(1).reliable(),
        [this](const std_msgs::msg::Empty::SharedPtr) {
          scans_.clear();
          healthy_ = false;
        });
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input, rclcpp::SensorDataQoS(),
        std::bind(&CompoundLocalMapNode::cloud, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate),
                               std::bind(&CompoundLocalMapNode::publish, this));
  }

private:
  struct Scan {
    rclcpp::Time stamp;
    pcl::PointCloud<pcl::PointXYZ> points;
  };
  /** @brief 使用点云对应时间的 TF 入库，仅缓存健康定位时的扫描。 */
  void cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    if (!healthy_ || std::chrono::steady_clock::now() - health_time_ >
                         std::chrono::seconds(1)) {
      scans_.clear();
      return;
    }
    try {
      auto tf = buffer_.lookupTransform(frame_, msg->header.frame_id,
                                        msg->header.stamp,
                                        rclcpp::Duration::from_seconds(0.1));
      sensor_msgs::msg::PointCloud2 transformed;
      tf2::doTransform(*msg, transformed, tf);
      Scan scan{rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()),
                {}};
      pcl::fromROSMsg(transformed, scan.points);
      scans_.push_back(std::move(scan));
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "cloud TF unavailable: %s", e.what());
    }
  }
  /** @brief 从同一帧融合数据构建指定宽度的地图；公开体素限制在 10 m 立方体内。
   */
  bool buildMap(double side, bool public_window, double x, double y,
                double bottom, const rclcpp::Time &stamp,
                clearing_robot_plugins::msg::CompoundLocalMap &out,
                std::vector<double> &live_clearances) {
    const double map_resolution =
        public_window ? this->resolution_ : control_resolution_;
    const double voxel_size = public_window ? this->voxel_ : control_voxel_;
    grid_map::GridMap map({"elevation", "slope"});
    map.setFrameId(frame_);
    map.setTimestamp(stamp.nanoseconds());
    map.setGeometry(grid_map::Length(side, side), map_resolution,
                    grid_map::Position(x, y));
    // 候选最低点并非已经证实的地面；先收集，再检查邻域支撑及与机器人底部的连通性。
    grid_map::Matrix candidates(map.getSize()(0), map.getSize()(1));
    candidates.setConstant(NAN);
    auto gather = [&](const pcl::PointCloud<pcl::PointXYZ> &cloud) {
      for (const auto &p : cloud) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        grid_map::Index i;
        if (!map.getIndex(grid_map::Position(p.x, p.y), i)) continue;
        auto &z = candidates(i(0), i(1));
        if (!std::isfinite(z) || p.z < z) z = p.z;
      }
    };
    gather(prior_);
    const grid_map::Matrix prior_candidates = candidates;
    for (const auto &scan : scans_) {
      for (const auto &p : scan.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        grid_map::Index i;
        if (!map.getIndex(grid_map::Position(p.x, p.y), i) ||
            std::isfinite(prior_candidates(i(0), i(1)))) continue;
        auto &z = candidates(i(0), i(1));
        if (!std::isfinite(z) || p.z < z) z = p.z;
      }
    }
    const int rows = candidates.rows(), cols = candidates.cols();
    auto inside = [&](int r, int c) {return r >= 0 && c >= 0 && r < rows && c < cols;};
    // 局部拟合平面与候选高程一致时才接受地面，避免孤立障碍被当成支撑面。
    Eigen::ArrayXXi supported = Eigen::ArrayXXi::Zero(rows, cols);
    grid_map::Matrix plane_r = grid_map::Matrix::Zero(rows, cols);
    grid_map::Matrix plane_c = grid_map::Matrix::Zero(rows, cols);
    grid_map::Matrix plane_z = grid_map::Matrix::Constant(rows, cols, NAN);
    for (int r = 0; r < rows; ++r) for (int c = 0; c < cols; ++c) {
      const double z = candidates(r,c);
      if (!std::isfinite(z)) continue;
      Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
      Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
      int n = 0;
      // 不使用中心点拟合，防止孤立物体把拟合面拉向自身。
      for (int dr=-2; dr<=2; ++dr) for (int dc=-2; dc<=2; ++dc) {
        if ((dr == 0 && dc == 0) || !inside(r+dr,c+dc)) continue;
        const double h = candidates(r+dr,c+dc);
        if (!std::isfinite(h)) continue;
        Eigen::Vector3d a(dr*map_resolution, dc*map_resolution, 1.0);
        normal.noalias() += a*a.transpose(); rhs.noalias() += a*h; ++n;
      }
      if (n < 3 || std::abs(normal.determinant()) < 1e-9) continue;
      const Eigen::Vector3d plane = normal.ldlt().solve(rhs);
      if (plane.allFinite() && std::abs(plane.z()-z) <= 0.08) {
        supported(r,c)=1;
        plane_r(r,c)=plane.x(); plane_c(r,c)=plane.y(); plane_z(r,c)=plane.z();
      }
    }
    // 起点由机器人底部附近的连续支撑确定；未知、障碍或断崖不作可通行连通地面。
    Eigen::ArrayXXi connected = Eigen::ArrayXXi::Zero(rows, cols);
    std::deque<grid_map::Index> queue;
    double closest = std::numeric_limits<double>::infinity();
    grid_map::Index seed;
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
      const auto i=*it; if (!supported(i(0),i(1))) continue;
      grid_map::Position p; map.getPosition(i,p);
      const double d=(p-grid_map::Position(x,y)).norm();
      if (d <= 1.0 && std::abs(candidates(i(0),i(1))-bottom) <= 0.10 && d < closest) {
        closest=d; seed=i;
      }
    }
    if (std::isfinite(closest)) { connected(seed(0),seed(1))=1; queue.push_back(seed); }
    while (!queue.empty()) {
      const auto i=queue.front(); queue.pop_front();
      // 先验点间距可大于显示格。允许隔一格寻找支撑，缺样格本身仍为未知。
      for (int dr=-2; dr<=2; ++dr) for (int dc=-2; dc<=2; ++dc) {
        if ((dr == 0 && dc == 0) || dr*dr + dc*dc > 5 ||
            !inside(i(0)+dr,i(1)+dc)) continue;
        const int r=i(0)+dr, c=i(1)+dc;
        if (!supported(r,c) || connected(r,c)) continue;
        const double run=map_resolution*std::hypot(dr,dc);
        if (std::abs(candidates(r,c)-candidates(i(0),i(1))) > 0.08 + std::tan(0.30)*run) continue;
        connected(r,c)=1; queue.emplace_back(r,c);
      }
    }
    // 深坑底面与可通行区域不连通，但稳定的低位支撑仍须保留用于描述坑洼。
    for (int r=0; r<rows; ++r) for (int c=0; c<cols; ++c) {
      if (std::isfinite(closest) && supported(r,c) && (connected(r,c) || candidates(r,c) < bottom-0.35))
        map["elevation"](r,c)=candidates(r,c);
    }
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
      const auto index = *it;
      const auto z = map.at("elevation", index);
      if (!std::isfinite(z))
        continue;
      grid_map::Position pos;
      map.getPosition(index, pos);
      double max_gradient = 0;
      int count = 0;
      for (const auto &delta : {grid_map::Position(map_resolution, 0),
                                grid_map::Position(-map_resolution, 0),
                                grid_map::Position(0, map_resolution),
                                grid_map::Position(0, -map_resolution)}) {
        grid_map::Index neighbor;
        if (!map.getIndex(pos + delta, neighbor))
          continue;
        const auto other = map.at("elevation", neighbor);
        if (!std::isfinite(other))
          continue;
        max_gradient =
            std::max(max_gradient,
                     std::abs(static_cast<double>(other - z)) / map_resolution);
        ++count;
      }
      if (count)
        map.at("slope", index) = std::atan(max_gradient);
    }
    // 仅使用已验证、与底盘连通且由先验支持的平坦地面估计本帧正向残差。
    // 每格最小值消除密集点的权重偏置；空间覆盖或离散程度不足时保留原阈值。
    if (!public_window) {
      live_clearances.clear();
      live_clearances.reserve(scans_.size());
      for (const auto &scan : scans_) {
        grid_map::Matrix minima = grid_map::Matrix::Constant(rows, cols, NAN);
        for (const auto &p : scan.points) {
          if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
              std::hypot(p.x-x, p.y-y) > 7.0) continue;
          grid_map::Index i;
          if (!map.getIndex(grid_map::Position(p.x,p.y), i)) continue;
          const int r=i(0), c=i(1);
          if (!connected(r,c) || !std::isfinite(prior_candidates(r,c)) ||
              !std::isfinite(map["slope"](r,c)) || map["slope"](r,c) > 0.12 ||
              !std::isfinite(plane_z(r,c))) continue;
          grid_map::Position center; map.getPosition(i,center);
          const double expected=plane_z(r,c) + plane_r(r,c)*(center.x()-p.x) +
              plane_c(r,c)*(center.y()-p.y);
          const double residual=p.z-expected;
          if (!std::isfinite(residual) || residual < -0.06 || residual > 0.30) continue;
          auto &minimum=minima(r,c);
          if (!std::isfinite(minimum) || residual<minimum) minimum=residual;
        }
        std::vector<double> values;
        int quadrants[4]={0,0,0,0};
        for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
          const auto i=*it; const double value=minima(i(0),i(1));
          if (!std::isfinite(value)) continue;
          grid_map::Position p; map.getPosition(i,p);
          ++quadrants[(p.x()>=x ? 1 : 0) + (p.y()>=y ? 2 : 0)];
          values.push_back(value);
        }
        int covered=0; for (const int n : quadrants) if (n>=2) ++covered;
        double clearance=ground_clearance_;
        if (values.size()>=12 && covered>=3) {
          std::sort(values.begin(),values.end());
          const auto n=values.size();
          const double median=n%2 ? values[n/2] : 0.5*(values[n/2-1]+values[n/2]);
          const double spread=values[(3*n)/4]-values[n/4];
          if (median>0.0 && median<=0.12 && spread<=0.05) {
            const double upper=values[(9*(n-1))/10];
            clearance=std::max(ground_clearance_,std::min(0.14,upper+0.025));
          }
        }
        live_clearances.push_back(clearance);
      }
    }
    octomap::OcTree tree(voxel_size);
    auto occupancy = [&](const pcl::PointCloud<pcl::PointXYZ> &cloud, double clearance) {
      for (const auto &p : cloud) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
          continue;
        grid_map::Index i;
        if (!map.getIndex(grid_map::Position(p.x, p.y), i))
          continue;
        const auto ground = map.at("elevation", i);
        if (std::isfinite(ground)) {
          grid_map::Position center; map.getPosition(i, center);
          // grid_map 两个索引沿地图 X/Y 反向增长。
          const double expected = plane_z(i(0),i(1)) +
              plane_r(i(0),i(1))*(center.x()-p.x) +
              plane_c(i(0),i(1))*(center.y()-p.y);
          if (std::isfinite(expected) && p.z <= expected + clearance) continue;
        }
        octomap::OcTreeKey key;
        if (!tree.coordToKeyChecked(octomap::point3d(p.x, p.y, p.z), key))
          continue;
        const auto center = tree.keyToCoord(key);
        // 未知高程不假定为空地；只以已验证支撑面过滤地面点。
        if (public_window &&
            !voxelWithinWindow(center.x(), center.y(), center.z(), voxel_size,
                               x, y, bottom))
          continue;
        tree.updateNode(key, true);
      }
    };
    occupancy(prior_, ground_clearance_);
    for (std::size_t i=0; i<scans_.size(); ++i)
      occupancy(scans_[i].points, i<live_clearances.size() ? live_clearances[i] : ground_clearance_);
    tree.updateInnerOccupancy();
    out.header.frame_id = frame_;
    out.header.stamp = stamp;
    out.terrain = *grid_map::GridMapRosConverter::toMessage(map);
    if (!octomap_msgs::binaryMapToMsg(tree, out.occupancy))
      return false;
    out.occupancy.header = out.header;
    return true;
  }
  /** @brief 以最近有效扫描时间发布公开 10 m 窗口和宽范围控制地图。 */
  void publish() {
    if (!healthy_ || std::chrono::steady_clock::now() - health_time_ >
                         std::chrono::seconds(1)) {
      scans_.clear();
      return;
    }
    const auto current = now();
    scans_.erase(std::remove_if(scans_.begin(), scans_.end(),
                                [&](const Scan &s) {
                                  const double age =
                                      (current - s.stamp).seconds();
                                  return age > ttl_ || age < 0;
                                }),
                 scans_.end());
    if (scans_.empty() ||
        (current - scans_.back().stamp).seconds() > max_scan_age_) {
      scans_.clear();
      return;
    }
    const auto stamp = scans_.back().stamp;
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = buffer_.lookupTransform(frame_, base_, stamp,
                                   rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException &e) {
      scans_.clear();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "timestamped base TF unavailable: %s", e.what());
      return;
    }
    const double x = tf.transform.translation.x, y = tf.transform.translation.y;
    const auto &q = tf.transform.rotation;
    // base_link 的底部偏移沿车体 Z 轴，经当前姿态变换到 map 高度。
    const double bottom =
        tf.transform.translation.z +
        bottom_offset_ * (1.0 - 2.0 * (q.x * q.x + q.y * q.y));

    clearing_robot_plugins::msg::CompoundLocalMap control, visible;
    std::vector<double> live_clearances;
    if (!buildMap(length_, false, x, y, bottom, stamp, control, live_clearances) ||
        !buildMap(10.0, true, x, y, bottom, stamp, visible, live_clearances)) {
      RCLCPP_ERROR(get_logger(), "failed to serialize local octomap");
      return;
    }
    control_pub_->publish(control);
    pub_->publish(visible);
    terrain_pub_->publish(visible.terrain);
    occupancy_pub_->publish(visible.occupancy);
  }

  std::string frame_, base_;
  double length_, resolution_, voxel_, ttl_, max_scan_age_, bottom_offset_,
      ground_clearance_, control_resolution_, control_voxel_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  pcl::PointCloud<pcl::PointXYZ> prior_;
  std::vector<Scan> scans_;
  rclcpp::Publisher<clearing_robot_plugins::msg::CompoundLocalMap>::SharedPtr
      pub_,
      control_pub_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr terrain_pub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr occupancy_pub_;
  bool healthy_ = false;
  std::chrono::steady_clock::time_point health_time_{};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr health_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace clearing_robot

/** @brief 启动独立复合地图 ROS 2 节点。 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<clearing_robot::CompoundLocalMapNode>());
  rclcpp::shutdown();
  return 0;
}
