#include "clearing_robot_plugins/map_adapter_utils.hpp"
#include "clearing_robot_plugins/motion_limits.hpp"
#include "clearing_robot_plugins/planner_utils.hpp"
#include "clearing_robot_plugins/plugins.hpp"
#include <algorithm>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <gtest/gtest.h>
#include <sstream>
#include <iomanip>
#include <multimap_nav_utils/selection/navigation_mode_analyzer.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <thread>

using namespace clearing_robot_plugins;

TEST(ClearingMapModes, FloorThicknessAndCeilingAreNotGroundRoughness)
{
  TerrainMap support(80, 80, .5, -20, -20);
  std::fill(support.elevations.begin(), support.elevations.end(), 0.);
  Cloud input;
  for (float z : {-.5f, 0.f, 3.f, 20.f})
  {
    Point p;
    p.x = 0;
    p.y = 0;
    p.z = z;
    input.push_back(p);
  }
  const auto costmap = terrainBand(input, support, 4.9);
  ASSERT_EQ(costmap.size(), 2U);
  EXPECT_FLOAT_EQ(costmap[0].z, 0);
  EXPECT_FLOAT_EQ(costmap[1].z, 3);
  const auto gridmap = terrainBand(input, support, std::numeric_limits<double>::infinity());
  ASSERT_EQ(gridmap.size(), 3U);
  EXPECT_FLOAT_EQ(gridmap.back().z, 20);
}

RobotModel sampleModel()
{
  RobotModel model;
  model.radius = 6;
  Box chassis;
  chassis.name = "chassis";
  chassis.size = Eigen::Vector3d(4, 3, 1);
  chassis.pose.translation() = Eigen::Vector3d(0, 0, .6);
  model.boxes.push_back(chassis);
  Box arm;
  arm.name = "arm";
  arm.size = Eigen::Vector3d(4, .8, .8);
  arm.pose.translation() = Eigen::Vector3d(4, 0, 3);
  model.boxes.push_back(arm);
  return model;
}
TEST(ClearingMapModes, OctomapRetainsHeightAndVoxelVolume)
{
  TerrainMap support(80, 80, .5, -20, -20);
  std::fill(support.elevations.begin(), support.elevations.end(), 0.);
  RobotModel robot;
  Box body;
  body.size = {1, 1, 1};
  body.pose.translation() = Eigen::Vector3d(0, 0, .6);
  robot.boxes.push_back(body);
  octomap::OcTree ceiling(.2);
  ceiling.updateNode(octomap::point3d(0, 0, 2.4), true);
  ceiling.updateNode(octomap::point3d(0, 0, -.1), true);
  octomap_msgs::msg::Octomap msg;
  ASSERT_TRUE(octomap_msgs::binaryMapToMsg(ceiling, msg));
  const auto clear = MapGeometry::fromOctomap(msg, support);
  EXPECT_EQ(clear->size(), 1U);
  EXPECT_FALSE(clear->collides(robot, Pose{}, 0.));
  octomap::OcTree edge(.4);
  edge.updateNode(octomap::point3d(.65, 0, .65), true);
  ASSERT_TRUE(octomap_msgs::binaryMapToMsg(edge, msg));
  const auto occupied = MapGeometry::fromOctomap(msg, support);
  EXPECT_FALSE(robot.contains({.6, .2, .6}));
  EXPECT_TRUE(occupied->collides(robot, Pose{}, 0.))
      << "Voxel corner overlaps even though its center is outside the robot";
}
TEST(ClearingMapModes, PlanarModesRemainConservativeAndBlockUnknown)
{
  RobotModel robot;
  Box body;
  body.size = {1, 1, 1};
  body.pose.translation() = Eigen::Vector3d(0, 0, .6);
  robot.boxes.push_back(body);
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.resolution = .4;
  msg.info.width = 1;
  msg.info.height = 1;
  msg.info.origin.position.x = -.2;
  msg.info.origin.position.y = -.2;
  msg.info.origin.orientation.w = 1.;
  msg.data = {100};
  EXPECT_TRUE(MapGeometry::fromCostmap(msg)->collides(robot, Pose{}, 0.));
  msg.data = {-1};
  EXPECT_TRUE(MapGeometry::fromCostmap(msg)->collides(robot, Pose{}, 0.));
  msg.data = {0};
  EXPECT_FALSE(MapGeometry::fromCostmap(msg)->collides(robot, Pose{}, 0.));
  grid_map::GridMap grid({"obstacle"});
  grid.setFrameId("map");
  grid.setGeometry({2, 2}, .25);
  grid["obstacle"].setZero();
  grid.atPosition("obstacle", {0, 0}) = 1.;
  const auto encoded = grid_map::GridMapRosConverter::toMessage(grid);
  EXPECT_TRUE(MapGeometry::fromGridmap(*encoded)->collides(robot, Pose{}, 0.));
}
TEST(ClearingMapModes, AutomaticSelectionUsesSceneFeatures)
{
  multimap_nav_utils::AnalyzerConfig config;
  std::string error;
  ASSERT_TRUE(multimap_nav_utils::loadAnalyzerConfigFromYaml(
      std::string(CLEARING_ASSET_DIR) + "/config/modes.yaml", config, error))
      << error;
  multimap_nav_utils::NavigationModeAnalyzer analyzer(config);
  multimap_nav_utils::SceneFeatures flat;
  flat.flat_ratio = 1.;
  EXPECT_EQ(analyzer.selectAutomatically(flat).selected_mode, "costmap2d");
  auto slope = flat;
  slope.flat_ratio = .2;
  slope.slope_ratio = .8;
  slope.requires_elevation_surface = true;
  EXPECT_EQ(analyzer.selectAutomatically(slope).selected_mode, "gridmap");
  auto multi = flat;
  multi.flat_ratio = .1;
  multi.multilayer_ratio = .9;
  multi.requires_multilayer_topology = true;
  EXPECT_EQ(analyzer.selectAutomatically(multi).selected_mode, "octomap");
}
TEST(ClearingGeometry, MarkersAndCollisionUseIdenticalBoxes)
{
  auto model = sampleModel();
  auto markers = model.markers("base_link", .2);
  ASSERT_EQ(markers.markers.size(), model.boxes.size());
  for (size_t i = 0; i < model.boxes.size(); ++i)
  {
    EXPECT_NEAR(markers.markers[i].scale.x, model.boxes[i].size.x() + .4, 1e-9);
    EXPECT_NEAR(markers.markers[i].pose.position.z, model.boxes[i].pose.translation().z(), 1e-9);
  }
  EXPECT_TRUE(model.contains(Eigen::Vector3d(5, 0, 3)));
  EXPECT_FALSE(model.contains(Eigen::Vector3d(5, 0, .5)));
  Cloud::Ptr cloud(new Cloud);
  Point p;
  p.x = 5;
  p.y = 0;
  p.z = 3;
  cloud->push_back(p);
  pcl::KdTreeFLANN<Point> tree;
  tree.setInputCloud(cloud);
  EXPECT_TRUE(model.collides(Pose{}, *cloud, tree, .0));
  Pose rotated;
  rotated.yaw = M_PI_2;
  EXPECT_FALSE(model.collides(rotated, *cloud, tree, .0));
}
TEST(ClearingTerrain, ClimbingAndCrossSlopeAreConstrained)
{
  TerrainMap map(80, 80, .5, -20, -20);
  Pose p;
  for (int k = 0; k < int(map.elevations.size()); ++k)
  {
    map.elevations[k] = .1 * map.center(k).x();
  }
  EXPECT_TRUE(map.project(p, .21, .17, .12));
  for (int k = 0; k < int(map.elevations.size()); ++k)
  {
    map.elevations[k] = .4 * map.center(k).x();
  }
  EXPECT_FALSE(map.project(p, .21, .17, .12));
  for (int k = 0; k < int(map.elevations.size()); ++k)
  {
    map.elevations[k] = .3 * map.center(k).y();
  }
  EXPECT_FALSE(map.project(p, .21, .17, .12));
}
TEST(ClearingPaper, BidirectionalSearchFilteringAndBezierRemainObstacleFree)
{
  TerrainMap map(80, 80, .5, -20, -20);
  std::fill(map.elevations.begin(), map.elevations.end(), 0);
  Validator valid = [&](const Pose &a, const Pose &b)
  {
    int count = std::max(1, int(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / .1)));
    for (int i = 0; i <= count; ++i)
    {
      double t = double(i) / count, x = a.x + t * (b.x - a.x), y = a.y + t * (b.y - a.y);
      if (std::abs(x) < .8 && std::abs(y) < 3)
      {
        return false;
      }
    }
    return true;
  };
  Pose start, goal;
  start.x = -8;
  goal.x = 8;
  auto path = paperPlan(map, start, goal, valid);
  ASSERT_GT(path.size(), 2U);
  auto filtered = filterNodes(path, valid);
  auto smooth = bezierPath(filtered, valid);
  ASSERT_GE(smooth.size(), 2U);
  for (size_t i = 1; i < smooth.size(); ++i)
  {
    EXPECT_TRUE(valid(smooth[i - 1], smooth[i]));
  }
  EXPECT_NEAR(smooth.front().x, -8, 1e-9);
  EXPECT_NEAR(smooth.back().x, 8, 1e-9);
}
TEST(ClearingGeometry, EntireRotationalSweepIsChecked)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  std::string root = CLEARING_ASSET_DIR;
  options.parameter_overrides(
      {rclcpp::Parameter("scene.map_source", "file"),
       rclcpp::Parameter("scene.map_mode", "octomap"),
       rclcpp::Parameter("scene.robot_model_file", root + "/config/robot_model.yaml"),
       rclcpp::Parameter("scene.map_file", root + "/maps/clearing_enclosed.pcd"),
       rclcpp::Parameter("scene.support_file", root + "/maps/clearing_support.pcd")});
  auto node = std::make_shared<Node>("swept_geometry_test", options);
  auto scene = Scene::create(node);
  Pose start, goal;
  start.x = 1.5;
  start.y = .7;
  start.yaw = .4;
  goal.x = 10;
  goal.y = 6;
  Validator static_clear = [&](const Pose &a, const Pose &b)
  { return scene->swept(a, b, scene->snapshot(), false); };
  EXPECT_TRUE(static_clear(start, goal));
  auto replanned = paperPlan(*scene->map, start, goal, static_clear);
  EXPECT_FALSE(replanned.empty())
      << "Sub-cell endpoint connectors must not create artificial arm rotations";
  PaperGlobalPlanner planner;
  planner.configure(node, "paper_regression");
  planner.activate();
  geometry_msgs::msg::PoseStamped initial_pose, requested_goal, moved_pose;
  initial_pose.pose = toMsg(Pose{});
  requested_goal.pose = toMsg(goal);
  auto initial_path = planner.createPlan(initial_pose, requested_goal);
  ASSERT_GT(initial_path.poses.size(), 20U);
  Pose moved;
  moved.x = 8.5;
  moved.y = 4.5;
  moved.yaw = .75;
  moved_pose.pose = toMsg(moved);
  auto retained = planner.createPlan(moved_pose, requested_goal);
  ASSERT_FALSE(retained.poses.empty());
  EXPECT_NEAR(retained.poses.back().pose.position.x, goal.x, 1e-9);
  EXPECT_NEAR(retained.poses.back().pose.position.y, goal.y, 1e-9);
  planner.deactivate();
  planner.cleanup();
  scene->model = sampleModel();
  auto map = std::make_shared<TerrainMap>(80, 80, .5, -20, -20);
  std::fill(map->elevations.begin(), map->elevations.end(), 0);
  scene->map = map;
  auto live = std::make_shared<LiveData>();
  Point obstacle;
  obstacle.x = 3;
  obstacle.y = 3;
  obstacle.z = 3;
  live->cloud->push_back(obstacle);
  live->tree->setInputCloud(live->cloud);
  Pose a, b;
  b.yaw = M_PI_2;
  EXPECT_TRUE(scene->safe(a, live));
  EXPECT_TRUE(scene->safe(b, live));
  EXPECT_FALSE(scene->swept(a, b, live));
  scene.reset();
  node.reset();
  rclcpp::shutdown();
}

// Exercise the real controller with fresh sensor/arm heartbeats and the
// full map/body checks. A short terminal overshoot must use bounded reverse
// motion, not turn the long body around to face the waypoint behind it.
TEST(ClearingControl, TerminalOvershootReturnsWithoutHalfTurn)
{
  rclcpp::init(0, nullptr);
  const std::string root = CLEARING_ASSET_DIR;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("use_sim_time", true), rclcpp::Parameter("scene.map_source", "file"),
       rclcpp::Parameter("scene.map_mode", "octomap"),
       rclcpp::Parameter("scene.robot_model_file", root + "/config/robot_model.yaml"),
       rclcpp::Parameter("scene.map_file", root + "/maps/clearing_enclosed.pcd"),
       rclcpp::Parameter("scene.support_file", root + "/maps/clearing_support.pcd")});
  auto node = std::make_shared<Node>("terminal_overshoot_test", options);
  EXPECT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()), RCL_RET_OK);
  EXPECT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(), 1000000000LL),
            RCL_RET_OK);
  auto inputs = std::make_shared<rclcpp::Node>("terminal_sensor_fixture");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(inputs);
  auto localization =
      inputs->create_publisher<std_msgs::msg::Bool>("/clearing/localization_healthy", 10);
  auto sensors = inputs->create_publisher<std_msgs::msg::Bool>("/clearing/sensors_healthy", 10);
  auto joints = inputs->create_publisher<sensor_msgs::msg::JointState>("/joint_states",
                                                                       rclcpp::SensorDataQoS());
  auto obstacles = inputs->create_publisher<sensor_msgs::msg::PointCloud2>("/clearing/obstacles",
                                                                           rclcpp::SensorDataQoS());
  const RobotModel model(root + "/config/robot_model.yaml");
  sensor_msgs::msg::JointState arm;
  arm.position = model.stow;
  for (size_t i = 0; i < model.stow.size(); ++i)
    arm.name.push_back("joint" + std::to_string(i + 1));
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(Cloud{}, cloud);
  cloud.header.frame_id = "map";
  auto committed=inputs->create_publisher<std_msgs::msg::String>("/clearing/safety_status",10);
  geometry_msgs::msg::Twist submitted;double submitted_a=0.,submitted_aw=0.;
  auto update = [&]()
  {
    cloud.header.stamp = node->now();
    std_msgs::msg::Bool healthy;
    healthy.data = true;
    localization->publish(healthy);
    sensors->publish(healthy);
    joints->publish(arm);
    obstacles->publish(cloud);
    std_msgs::msg::String status;std::ostringstream json;
    json<<std::setprecision(17)<<"{\"command_protocol\":3,\"reset_sequence\":0,\"command\":["
        <<submitted.linear.x<<","<<submitted.angular.z<<"],\"command_acceleration\":["
        <<submitted_a<<","<<submitted_aw<<"],\"commit_stamp\":"<<node->now().seconds()<<"}";
    status.data=json.str();committed->publish(status);
    executor.spin_some();
  };
  {
    MPPITrajectory controller;
    controller.configure(node, "terminal_mppi");
    controller.activate();
    for (int i = 0; i < 30; ++i)
    {
      update();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    Pose pose, goal;
    pose.x = .5;
    Path route;
    route.header.frame_id = "map";
    for (const auto &p : {pose, goal})
    {
      geometry_msgs::msg::PoseStamped entry;
      entry.pose = toMsg(p);
      route.poses.push_back(entry);
    }
    controller.setLocalPlan(route, true);
    geometry_msgs::msg::Twist velocity;
    double max_yaw = 0.;
    for (int i = 0; i < 80 && !controller.isGoalReached(); ++i)
    {
      EXPECT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(),
                                          1000000000LL + int64_t(i) * 125000000LL),
                RCL_RET_OK);
      update();
      geometry_msgs::msg::PoseStamped current;
      current.header.stamp = node->now();
      current.pose = toMsg(pose);
      const auto result = controller.computeControl(current, velocity);
      submitted_a=(result.command.linear.x-velocity.linear.x)/.125;
      submitted_aw=(result.command.angular.z-velocity.angular.z)/.125;
      submitted=result.command;velocity = result.command;
      EXPECT_GE(velocity.linear.x, -.200001);
      pose.x += velocity.linear.x * std::cos(pose.yaw) * .125;
      pose.y += velocity.linear.x * std::sin(pose.yaw) * .125;
      pose.yaw = wrap(pose.yaw + velocity.angular.z * .125);
      max_yaw = std::max(max_yaw, std::abs(pose.yaw));
    }
    EXPECT_TRUE(controller.isGoalReached());
    EXPECT_LT(std::hypot(pose.x, pose.y), .25);
    EXPECT_LT(max_yaw, .16);
    controller.deactivate();
    controller.cleanup();
  }
  executor.remove_node(node->get_node_base_interface());
  executor.remove_node(inputs);
  node.reset();
  inputs.reset();
  rclcpp::shutdown();
}

TEST(ClearingBodyPath, ReverseKeepsBodyHeadingAndCurvature)
{
  Pose start, goal;
  goal.x = -5;
  Validator clear = [](const Pose &, const Pose &) { return true; };
  BodySearchOptions options;
  const auto path = connectBodyPath(start, goal, clear, options);
  ASSERT_GT(path.size(), 10U);
  EXPECT_TRUE(kinematicPath(path, options.min_radius));
  for (size_t i = 1; i < path.size(); ++i)
  {
    EXPECT_EQ(motionDirection(path[i - 1], path[i]), -1);
    EXPECT_NEAR(wrap(path[i].yaw), 0, 1e-6);
  }
  goal.x = 0;
  goal.yaw = M_PI;
  const auto manoeuvre = connectBodyPath(start, goal, clear, options);
  ASSERT_GT(manoeuvre.size(), 10U);
  EXPECT_TRUE(kinematicPath(manoeuvre, options.min_radius));
  double travel = 0;
  for (size_t i = 1; i < manoeuvre.size(); ++i)
    travel += detail::distance(manoeuvre[i - 1], manoeuvre[i]);
  EXPECT_GT(travel, 2.) << "A heading-only request needs a travelling manoeuvre, not a pivot";
}
TEST(ClearingDynamics, CoupledChassisLimitsRejectImpossibleMotion)
{
  MotionLimits limits;
  EXPECT_TRUE(limits.admissible(-.2, -.06, -.15, .05));
  EXPECT_FALSE(limits.admissible(.35, .4, 0, 0));
  EXPECT_FALSE(limits.admissible(0, .12, 0, 0));
  EXPECT_FALSE(limits.admissible(.35, .1, 1, 0));
  EXPECT_FALSE(limits.admissible(.1, .03, 0, 1));
}
TEST(ClearingBodyPath, DefaultEnclosureOpenPosesHaveFullBodyRoutes)
{
  rclcpp::init(0, nullptr);
  const std::string root = CLEARING_ASSET_DIR;
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("scene.map_source", "file"),
       rclcpp::Parameter("scene.map_mode", "octomap"),
       rclcpp::Parameter("scene.robot_model_file", root + "/config/robot_model.yaml"),
       rclcpp::Parameter("scene.map_file", root + "/maps/clearing_enclosed.pcd"),
       rclcpp::Parameter("scene.support_file", root + "/maps/clearing_support.pcd")});
  auto node = std::make_shared<Node>("open_pose_regression", options);
  auto scene = Scene::create(node);
  PaperGlobalPlanner planner;
  planner.configure(node, "paper_regression_open");
  planner.activate();
  geometry_msgs::msg::PoseStamped start, goal;
  start.pose = toMsg(Pose{});
  for (auto xy : std::vector<std::pair<double, double>>{
           {-8, -6}, {-8, 0}, {-8, 6}, {0, -8}, {0, 8}, {6, -6}, {8, 0}, {8, 6}, {10, 6}})
  {
    Pose target;
    target.x = xy.first;
    target.y = xy.second;
    goal.pose = toMsg(target);
    ASSERT_TRUE(scene->safe(target, scene->snapshot(), false));
    auto path = planner.createPlan(start, goal);
    ASSERT_FALSE(path.poses.empty()) << xy.first << "," << xy.second;
    auto points = detail::positions(path);
    EXPECT_TRUE(kinematicPath(points, 2.5));
    for (size_t i = 1; i < points.size(); ++i)
      EXPECT_TRUE(scene->swept(points[i - 1], points[i], scene->snapshot(), false));
  }
  planner.deactivate();
  planner.cleanup();
  scene.reset();
  node.reset();
  rclcpp::shutdown();
}
