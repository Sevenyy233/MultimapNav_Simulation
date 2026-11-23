cmake_minimum_required(VERSION 3.16)
project(clearing_robot_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
foreach(pkg IN ITEMS
    builtin_interfaces geometry_msgs grid_map_core grid_map_msgs grid_map_ros livox_ros_driver2
    multimap_nav_core multimap_nav_msgs nav_msgs octomap_msgs pcl_conversions pluginlib
    rclcpp rclcpp_lifecycle rosidl_default_generators sensor_msgs std_msgs tf2 tf2_ros
    tf2_geometry_msgs tf2_sensor_msgs)
  find_package(${pkg} REQUIRED)
endforeach()
find_package(PCL REQUIRED COMPONENTS common io)
find_package(octomap REQUIRED)
include(cmake/maps.cmake)

add_library(clearing_local_control SHARED src/local_control/mppi.cpp src/local_control/controller.cpp)
target_include_directories(clearing_local_control PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>" "$<INSTALL_INTERFACE:include>"
  ${OCTOMAP_INCLUDE_DIRS})
ament_target_dependencies(clearing_local_control geometry_msgs grid_map_core grid_map_ros
  multimap_nav_core nav_msgs octomap_msgs pluginlib rclcpp rclcpp_lifecycle tf2 tf2_geometry_msgs)
target_link_libraries(clearing_local_control "${clearing_typesupport}" ${OCTOMAP_LIBRARIES})

set(ONNXRUNTIME_ROOT "" CACHE PATH "ONNX Runtime C++ SDK")
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS "${ONNXRUNTIME_ROOT}" "$ENV{ONNXRUNTIME_ROOT}" PATH_SUFFIXES include include/onnxruntime)
find_library(ONNXRUNTIME_LIBRARY NAMES onnxruntime
  HINTS "${ONNXRUNTIME_ROOT}" "$ENV{ONNXRUNTIME_ROOT}" PATH_SUFFIXES lib lib64)
if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY)
  target_include_directories(clearing_local_control SYSTEM PRIVATE "${ONNXRUNTIME_INCLUDE_DIR}")
  target_link_libraries(clearing_local_control "${ONNXRUNTIME_LIBRARY}")
  get_filename_component(onnxruntime_library_dir "${ONNXRUNTIME_LIBRARY}" DIRECTORY)
  set_property(TARGET clearing_local_control APPEND PROPERTY INSTALL_RPATH "${onnxruntime_library_dir}")
  target_compile_definitions(clearing_local_control PRIVATE CLEARING_HAS_ONNXRUNTIME)
else()
  message(WARNING "ONNX Runtime C++ SDK unavailable; MPPI and training services build, DRL deployment requires ONNXRUNTIME_ROOT")
endif()

add_executable(clearing_training_control src/local_control/training_control.cpp)
target_link_libraries(clearing_training_control clearing_local_control "${clearing_typesupport}")
ament_target_dependencies(clearing_training_control rclcpp std_msgs)
add_executable(clearing_avia_fusion src/localization/avia_fusion.cpp)
ament_target_dependencies(clearing_avia_fusion livox_ros_driver2 rclcpp tf2 tf2_ros tf2_geometry_msgs)

pluginlib_export_plugin_description_file(multimap_nav_core local_controller_plugins.xml)
install(TARGETS clearing_local_control DESTINATION lib)
install(TARGETS clearing_training_control clearing_avia_fusion DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/estimator_supervisor.py scripts/navigation_modes.py DESTINATION lib/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
