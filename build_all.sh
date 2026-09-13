#!/usr/bin/env bash
# Build every component used by the real FAST-LIO + SCAN + AR stack.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/noetic/setup.bash
set -u

cmake -S "$ROOT/third_party/Livox-SDK2" -B "$ROOT/.build/livox-sdk2" -DCMAKE_BUILD_TYPE=Release
cmake --build "$ROOT/.build/livox-sdk2" --target livox_lidar_sdk_static -- -j2

if [ ! -e "$ROOT/2D/FAST-LIO/src/CMakeLists.txt" ]; then
  ln -s /opt/ros/noetic/share/catkin/cmake/toplevel.cmake "$ROOT/2D/FAST-LIO/src/CMakeLists.txt"
fi
cp -f "$ROOT/2D/FAST-LIO/src/livox_ros_driver2/package_ROS1.xml" \
  "$ROOT/2D/FAST-LIO/src/livox_ros_driver2/package.xml"
(cd "$ROOT/2D/FAST-LIO" && catkin_make -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE=Release -j2)

if [ ! -e "$ROOT/2D/SENSOR-SCAN/src/CMakeLists.txt" ]; then
  ln -s /opt/ros/noetic/share/catkin/cmake/toplevel.cmake "$ROOT/2D/SENSOR-SCAN/src/CMakeLists.txt"
fi
(cd "$ROOT/2D/SENSOR-SCAN" && catkin_make -DCMAKE_BUILD_TYPE=Release -j2)

# Build the optional elevation:=true branch as a separate workspace.  The
# default 2-D launch remains elevation:=false and does not start these nodes.
ELEVATION_ROOT="$ROOT/2D/go2-scan/algorithms/elevation_mapping"
ELEVATION_WS="$ROOT/2D/ELEVATION-MAPPING" \
  bash "$ELEVATION_ROOT/go2_integration/build_elevation_ws.sh"

SCAN="$ROOT/2D/go2-scan/algorithms/local_planning/scan_planner"
if [ ! -e "$SCAN/src/CMakeLists.txt" ]; then
  ln -s /opt/ros/noetic/share/catkin/cmake/toplevel.cmake "$SCAN/src/CMakeLists.txt"
fi
(cd "$SCAN" && catkin_make -DCMAKE_BUILD_TYPE=Release -j2)

cmake -S "$ROOT/2D/go2-scan/integration/go2_motion" \
  -B "$ROOT/2D/go2-scan/integration/go2_motion/build" \
  -DUNITREE_SDK_ROOT="$ROOT/third_party/unitree_sdk2" -DCMAKE_BUILD_TYPE=Release
cmake --build "$ROOT/2D/go2-scan/integration/go2_motion/build" -- -j2

echo '[BUILD] FAST-LIO, Livox driver, sensor scan, optional elevation mapping, SCAN and Go2 motion bridge built successfully'
