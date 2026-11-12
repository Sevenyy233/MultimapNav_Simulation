#!/usr/bin/env bash
# Build only this workspace over the system Humble installation.
set -eo pipefail
package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$(cd -- "$package_dir/../.." && pwd)"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
unset PYTHONPATH LD_LIBRARY_PATH GAZEBO_PLUGIN_PATH GAZEBO_MODEL_PATH GAZEBO_RESOURCE_PATH
source /opt/ros/humble/setup.bash
# The clearing-robot package implements the sibling framework's plugin APIs.
# Use local_setup to avoid replaying old simulation overlay chains.
framework_dir="$(cd -- "$workspace_dir/../MultiMapNav" && pwd)"
if [[ -f "$framework_dir/install/local_setup.bash" ]]; then
  source "$framework_dir/install/local_setup.bash"
fi
driver_dir="$workspace_dir/src/livox_ros_driver2"
if [[ ! -f "$driver_dir/package_ROS2.xml" ]]; then
  echo "Missing dependency: clone https://github.com/Livox-SDK/livox_ros_driver2.git into $driver_dir" >&2
  exit 1
fi
# Upstream build.sh performs this ROS2 manifest selection but also deletes the
# entire build/install trees. Select the manifest without that destructive step.
if ! cmp -s "$driver_dir/package_ROS2.xml" "$driver_dir/package.xml"; then
  cp "$driver_dir/package_ROS2.xml" "$driver_dir/package.xml"
fi
cd "$workspace_dir"
export CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
colcon build --base-paths src --parallel-workers 1 --event-handlers console_cohesion+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DROS_EDITION=ROS2 -DDISTRO_ROS=humble \
  "-DCMAKE_PROJECT_INCLUDE=$package_dir/cmake/vendor_compat.cmake" "$@"
