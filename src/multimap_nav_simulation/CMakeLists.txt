cmake_minimum_required(VERSION 3.8)
project(multimap_nav_simulation)

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
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(gazebo REQUIRED)
add_library(multimap_obstacle_motion SHARED src/path_obstacle_motion.cpp)
target_include_directories(multimap_obstacle_motion PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_directories(multimap_obstacle_motion PRIVATE ${GAZEBO_LIBRARY_DIRS})
target_link_libraries(multimap_obstacle_motion ${GAZEBO_LIBRARIES})
install(TARGETS multimap_obstacle_motion LIBRARY DESTINATION lib)

add_executable(odom_tf_broadcaster src/odom_tf_broadcaster.cpp)
ament_target_dependencies(odom_tf_broadcaster rclcpp nav_msgs tf2 tf2_ros geometry_msgs)

install(TARGETS
  odom_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/initialize_clearing_arm.py scripts/wait_for_gazebo_clock.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 3DModel config docs launch meshes models pcd pcd_modified robot scripts worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME map_suite_consistency
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/validate_map_suite.py
      --package-dir ${CMAKE_CURRENT_SOURCE_DIR}
  )
  set_tests_properties(map_suite_consistency PROPERTIES TIMEOUT 60)
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
