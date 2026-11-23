# 滚动复合地图消息和独立节点。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CompoundLocalMap.msg"
  "srv/TrainingControl.srv"
  "srv/ResetEstimator.srv"
  DEPENDENCIES std_msgs grid_map_msgs octomap_msgs nav_msgs geometry_msgs builtin_interfaces)
add_executable(clearing_local_map src/local_map/compound_local_map_node.cpp)
target_include_directories(clearing_local_map PRIVATE ${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})
ament_target_dependencies(clearing_local_map rclcpp sensor_msgs tf2_ros tf2_sensor_msgs
  pcl_conversions grid_map_core grid_map_ros grid_map_msgs octomap_msgs)
target_link_libraries(clearing_local_map ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
rosidl_get_typesupport_target(clearing_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(clearing_local_map "${clearing_typesupport}")
install(TARGETS clearing_local_map DESTINATION lib/${PROJECT_NAME})

# 三种地图共用带航向的前向圆弧 A* 搜索。
add_library(clearing_astar_planners SHARED src/global_planning/astar.cpp)
target_include_directories(clearing_astar_planners PRIVATE ${OCTOMAP_INCLUDE_DIRS})
ament_target_dependencies(clearing_astar_planners rclcpp rclcpp_lifecycle pluginlib
  multimap_nav_core multimap_nav_msgs grid_map_core grid_map_ros grid_map_msgs octomap_msgs tf2 tf2_geometry_msgs)
target_link_libraries(clearing_astar_planners ${OCTOMAP_LIBRARIES})
pluginlib_export_plugin_description_file(multimap_nav_core plugins_xml/global_planners.xml)
install(TARGETS clearing_astar_planners LIBRARY DESTINATION lib)

# RViz 复合消息显示。
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
add_library(clearing_compound_rviz SHARED src/local_map/compound_display.cpp include/clearing_robot/local_map/compound_display.hpp)
set_target_properties(clearing_compound_rviz PROPERTIES AUTOMOC ON)
ament_target_dependencies(clearing_compound_rviz rviz_common rviz_rendering pluginlib rclcpp
  grid_map_core grid_map_ros grid_map_msgs octomap_msgs)
target_include_directories(clearing_compound_rviz PRIVATE include ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(clearing_compound_rviz Qt5::Widgets ${OCTOMAP_LIBRARIES} "${clearing_typesupport}")
pluginlib_export_plugin_description_file(rviz_common plugins_xml/compound_display.xml)
install(TARGETS clearing_compound_rviz LIBRARY DESTINATION lib)

# Add rcl_interfaces to package.xml (depend) and CMake dependencies.
find_package(rcl_interfaces REQUIRED)
add_library(clearing_navigation_mode_panel SHARED
  src/local_map/navigation_mode_panel.cpp
  include/clearing_robot/local_map/navigation_mode_panel.hpp)
set_target_properties(clearing_navigation_mode_panel PROPERTIES AUTOMOC ON)
target_include_directories(clearing_navigation_mode_panel PRIVATE include)
ament_target_dependencies(clearing_navigation_mode_panel rviz_common pluginlib rclcpp rcl_interfaces std_msgs)
target_link_libraries(clearing_navigation_mode_panel Qt5::Widgets)
pluginlib_export_plugin_description_file(rviz_common plugins_xml/navigation_mode_panel.xml)
install(TARGETS clearing_navigation_mode_panel LIBRARY DESTINATION lib)
