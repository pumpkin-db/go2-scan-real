#!/usr/bin/env bash
# Offline validation. Does not start ROS nodes, the lidar, or robot motion.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
fail() { echo "[FAIL] $*" >&2; exit 42; }

[ "$(uname -m)" = aarch64 ] || fail 'architecture is not aarch64'
[ -r /opt/ros/noetic/setup.bash ] || fail 'ROS Noetic is missing'
[ -x "$ROOT/.venv/ariadne/bin/python" ] || fail 'AR Python environment is missing'
[ -f "$ROOT/config/local/board.json" ] || fail 'board configuration is missing'
[ "$(stat -c %a "$ROOT/config/local/board.json")" = 600 ] || fail 'board configuration must have mode 0600'
[ -f "$ROOT/.build/livox-sdk2/sdk_core/liblivox_lidar_sdk_static.a" ] || fail 'local Livox-SDK2 library is missing'
[ -x "$ROOT/2D/FAST-LIO/devel/lib/fast_lio/fastlio_mapping" ] || fail 'FAST-LIO is not built'
[ -x "$ROOT/2D/FAST-LIO/devel/lib/livox_ros_driver2/livox_ros_driver2_node" ] || fail 'Livox driver is not built'
[ -x "$ROOT/2D/SENSOR-SCAN/devel/lib/sensor_scan_generation/sensorScanGeneration" ] || fail 'sensor scan package is not built'
[ -x "$ROOT/2D/go2-scan/algorithms/local_planning/scan_planner/devel/lib/scan_planner/scan_planner_node" ] || fail 'SCAN is not built'
[ -x "$ROOT/2D/go2-scan/integration/go2_motion/build/cmd_vel_bridge" ] || fail 'motion bridge is not built'

mkdir -p "$ROOT/.cache/matplotlib"
MPLCONFIGDIR="$ROOT/.cache/matplotlib" PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages \
  "$ROOT/.venv/ariadne/bin/python" -c 'import torch,numpy,scipy,skimage,matplotlib,rospy; assert torch.__version__.startswith("2.3.1")'
bash -n "$ROOT/setup_nx.sh" "$ROOT/build_all.sh" \
  "$ROOT/2D/go2-scan/real/launch_fastlio_NX.sh" \
  "$ROOT/2D/go2-scan/real/launch_fastlio_for_scan-planner_NX.sh"
python3 -m py_compile \
  "$ROOT/tools/configure_board.py" \
  "$ROOT/2D/go2-scan/real/sync_mid360_clock.py"

ROS_IP=127.0.0.1 "$ROOT/2D/go2-scan/real/launch_fastlio_NX.sh" motion:=false --check >/dev/null
echo '[PASS] self-contained real stack preflight passed; no hardware was started'
