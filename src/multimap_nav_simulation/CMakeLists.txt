cmake_minimum_required(VERSION 3.8)
project(multimap_nav_simulation)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)
# Load Gazebo before the driver's PCL/VTK exports to avoid duplicate JsonCpp
# imported aliases in the Jammy CMake packages.
find_package(livox_ros_driver2 REQUIRED)
add_library(multimap_obstacle_motion SHARED src/path_obstacle_motion.cpp)
target_include_directories(multimap_obstacle_motion PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_directories(multimap_obstacle_motion PRIVATE ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(multimap_obstacle_motion ${GAZEBO_LIBRARIES})
install(TARGETS multimap_obstacle_motion LIBRARY DESTINATION lib)

add_library(multimap_contact_audit SHARED src/contact_audit.cpp)
target_include_directories(multimap_contact_audit PRIVATE include ${GAZEBO_INCLUDE_DIRS})
target_link_directories(multimap_contact_audit PRIVATE ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(multimap_contact_audit ${GAZEBO_LIBRARIES})
ament_target_dependencies(multimap_contact_audit rclcpp gazebo_ros std_msgs)
install(TARGETS multimap_contact_audit LIBRARY DESTINATION lib)
add_library(multimap_controlled_obstacle SHARED src/controlled_obstacle.cpp)
target_include_directories(multimap_controlled_obstacle PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_directories(multimap_controlled_obstacle PRIVATE ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(multimap_controlled_obstacle ${GAZEBO_LIBRARIES})
ament_target_dependencies(multimap_controlled_obstacle rclcpp gazebo_ros std_msgs)
install(TARGETS multimap_controlled_obstacle LIBRARY DESTINATION lib)

add_executable(body_velocity_controller src/body_velocity_controller.cpp)
target_include_directories(body_velocity_controller PRIVATE include)
ament_target_dependencies(body_velocity_controller rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs)
add_executable(odom_tf_broadcaster src/odom_tf_broadcaster.cpp)
ament_target_dependencies(odom_tf_broadcaster rclcpp nav_msgs tf2 tf2_ros geometry_msgs)
add_executable(fast_lio_sim_bridge src/fast_lio_sim_bridge.cpp)
target_include_directories(fast_lio_sim_bridge PRIVATE include)
ament_target_dependencies(fast_lio_sim_bridge
  rclcpp nav_msgs sensor_msgs std_msgs tf2 tf2_ros tf2_sensor_msgs geometry_msgs livox_ros_driver2)

install(TARGETS
  odom_tf_broadcaster
  fast_lio_sim_bridge
  body_velocity_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/initialize_clearing_arm.py scripts/wait_for_gazebo_clock.py
  scripts/clearing_tracks_drive.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 3DModel config docs launch meshes models pcd robot scripts worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# Optional historical derived clouds may be absent in the current workspace.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/pcd_modified")
  install(DIRECTORY pcd_modified DESTINATION share/${PROJECT_NAME})
endif()

install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_body_velocity_pi test/test_body_velocity_pi.cpp)
  target_include_directories(test_body_velocity_pi PRIVATE include)
  ament_add_gtest(test_contact_classification test/test_contact_classification.cpp)
  target_include_directories(test_contact_classification PRIVATE include)
  ament_add_gtest(test_pose_velocity_estimator test/test_pose_velocity_estimator.cpp)
  target_include_directories(test_pose_velocity_estimator PRIVATE include)
  ament_target_dependencies(test_pose_velocity_estimator tf2)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME map_suite_consistency
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/validate_map_suite.py
      --package-dir ${CMAKE_CURRENT_SOURCE_DIR}
  )
  set_tests_properties(map_suite_consistency PROPERTIES TIMEOUT 60)
  add_test(NAME clearing_robot_primitives_consistency
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_clearing_robot_primitives.py
  )
  set_tests_properties(clearing_robot_primitives_consistency PROPERTIES TIMEOUT 60)
  add_test(NAME robot_layout_consistency
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_robot_layout.py
  )
  set_tests_properties(robot_layout_consistency PROPERTIES TIMEOUT 60)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
