#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/noetic/setup.bash

cd "$ROOT_DIR/src/livox_ros_driver2"
cp -f package_ROS1.xml package.xml

cd "$ROOT_DIR"
catkin_make -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE=Release -j2
