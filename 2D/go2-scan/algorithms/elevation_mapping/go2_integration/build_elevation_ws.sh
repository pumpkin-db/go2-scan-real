#!/bin/bash
# Build only the upstream packages required by the Go2 elevation mode.
set -eo pipefail

integration_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
elevation_root="$(cd "$integration_dir/.." && pwd)"
workspace="${ELEVATION_WS:-$HOME/Go2/2D/ELEVATION-MAPPING}"
source_dir="$workspace/src"

source /opt/ros/noetic/setup.bash
set -u
mkdir -p "$source_dir"
[ -e "$source_dir/CMakeLists.txt" ] || ln -s /opt/ros/noetic/share/catkin/cmake/toplevel.cmake "$source_dir/CMakeLists.txt"

link_package() {
  local name="$1" target="$2"
  if [ -L "$source_dir/$name" ]; then
    [ "$(readlink -f "$source_dir/$name")" = "$(readlink -f "$target")" ] || {
      echo "[FATAL] $source_dir/$name points to an unexpected package" >&2
      exit 2
    }
  elif [ -e "$source_dir/$name" ]; then
    echo "[FATAL] refusing to replace $source_dir/$name" >&2
    exit 2
  else
    ln -s "$target" "$source_dir/$name"
  fi
}

deps="$elevation_root/dependencies"
for package in grid_map_core grid_map_ros grid_map_filters grid_map_visualization; do
  rospack find "$package" >/dev/null || {
    echo "[FATAL] missing ROS Noetic package: $package" >&2
    exit 2
  }
done
link_package kindr "$deps/kindr"
link_package kindr_ros "$deps/kindr_ros/kindr_ros"
link_package message_logger "$deps/message_logger"
link_package elevation_mapping "$elevation_root/elevation_mapping"
link_package go2_elevation "$integration_dir"

catkin_make -C "$workspace" -DCMAKE_BUILD_TYPE=Release -DCATKIN_ENABLE_TESTING=OFF -j4
echo "[READY] source $workspace/devel/setup.bash"
