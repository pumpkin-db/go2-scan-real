#!/bin/bash
# NX MID360 -> FAST-LIO -> SCAN; select ARiADNE or TARE for exploration.
# motion:=false is the default. Data comes directly from MID360;
# pre-start clock synchronization uses the board's existing PTP master.
# Optional vp:=0.2 sets TARE's XY viewpoint spacing in metres (default 0.5).
# Optional extend:=true enables TARE's straight-line waypoint extension (default false).
# TARE defaults to path:=true (forces extend=false); path:=false selects single-goal mode.
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
go2_root="$(cd "$script_dir/.." && pwd)"
fastlio_ws="${FASTLIO_WS:-$HOME/Go2/2D/FAST-LIO}"
fastlio_config="${FASTLIO_CONFIG:-$fastlio_ws/src/FAST_LIO/config/mid360.yaml}"
driver_config="${LIVOX_CONFIG:-$fastlio_ws/src/livox_ros_driver2/config/MID360_config.json}"
scan_ws="${SCAN:-$go2_root/algorithms/local_planning/scan_planner}"
sensor_scan_ws="${SENSOR_SCAN_WS:-$HOME/Go2/2D/SENSOR-SCAN}"
elevation_ws="${ELEVATION_WS:-$HOME/Go2/2D/ELEVATION-MAPPING}"
ariadne="${ARIADNE:-$go2_root/algorithms/global_planning/ariadne}"
tare_ws="${TARE_WS:-$go2_root/algorithms/global_planning/tare_planner}"
motion_ws="$go2_root/integration/go2_motion"
log_dir="$script_dir/logs/fastlio_navigation"

navigation=auto
exploration=tare
motion=false
record_bag=false
rviz=false
check=false
elevation=false
continuous=false
vp=0.5
extend=false
path=true
path_explicit=false
for argument in "$@"; do
  case "$argument" in
    navigation:=auto) navigation=auto;;
    navigation:=manual) navigation=manual;;
    exploration:=ariadne) exploration=ariadne;;
    exploration:=tare) exploration=tare;;
    exploration:=*) echo "[FATAL] exploration must be ariadne or tare: $argument" >&2; exit 2;;
    motion:=true) motion=true;;
    motion:=false) motion=false;;
    motion:=*) echo "[FATAL] invalid motion argument: $argument" >&2; exit 2;;
    record_bag:=true) record_bag=true;;
    record_bag:=false) record_bag=false;;
    rviz:=true) rviz=true;;
    rviz:=false) rviz=false;;
    rviz:=*) echo "[FATAL] invalid rviz argument: $argument" >&2; exit 2;;
    elevation:=true) elevation=true;;
    elevation:=false) elevation=false;;
    elevation:=*) echo "[FATAL] invalid elevation argument: $argument" >&2; exit 2;;
    continuous:=true) continuous=true;;
    continuous:=false) continuous=false;;
    continuous:=*) echo "[FATAL] invalid continuous argument: $argument" >&2; exit 2;;
    vp:=*) vp="${argument#vp:=}";;
    extend:=true) extend=true;;
    extend:=false) extend=false;;
    extend:=*) echo "[FATAL] extend must be true or false: $argument" >&2; exit 2;;
    path:=true) path=true; path_explicit=true;;
    path:=false) path=false; path_explicit=true;;
    path:=*) echo "[FATAL] path must be true or false: $argument" >&2; exit 2;;
    config:=*) fastlio_config="${argument#config:=}";;
    driver:=*) echo '[FATAL] FAST-LIO正式入口固定使用NX本地驱动，不接受driver参数' >&2; exit 2;;
    --check) check=true;;
    *) echo "[FATAL] unknown argument: $argument" >&2; exit 2;;
  esac
done

# Do not change manual/AR interfaces merely because TARE's default changed.
if [ "$path_explicit" = false ] && { [ "$navigation" != auto ] || [ "$exploration" != tare ]; }; then
  path=false
fi

if [ "$path" = true ]; then
  [ "$navigation" = auto ] && [ "$exploration" = tare ] || {
    echo '[FATAL] path:=true requires navigation:=auto exploration:=tare' >&2; exit 2;
  }
  if [ "$extend" = true ]; then
    echo '[PARAM] path mode follows the selected route; overriding extend:=true to false'
  fi
  extend=false
fi

python3 - "$vp" <<'PY'
import math
import sys
try:
    spacing = float(sys.argv[1])
    assert math.isfinite(spacing) and spacing > 0
except (ValueError, AssertionError):
    sys.exit('[FATAL] vp must be a positive finite number in metres')
PY

fail() { echo "[FATAL] $*" >&2; exit 42; }
tare_enabled=false
[ "$navigation" != auto ] || [ "$exploration" != tare ] || tare_enabled=true
mode=prone
[ "$motion" = true ] && mode=stand
[ -r "$fastlio_ws/devel/setup.bash" ] || fail "FAST-LIO workspace has not been built: $fastlio_ws"
[ -r "$scan_ws/devel/setup.bash" ] || fail "SCAN official workspace has not been built: $scan_ws"
[ "$tare_enabled" = false ] || [ -r "$tare_ws/devel/setup.bash" ] || fail "TARE workspace has not been built: $tare_ws"
[ -r "$fastlio_config" ] || fail "missing FAST-LIO config: $fastlio_config"
[ -r "$driver_config" ] || fail "missing Livox driver config: $driver_config"
[ -r "$sensor_scan_ws/devel/setup.bash" ] || fail "sensor scan workspace has not been built: $sensor_scan_ws"
[ "$elevation" = false ] || [ -r "$elevation_ws/devel/setup.bash" ] || fail "elevation workspace has not been built: $elevation_ws"
[ "$navigation" = auto ] || [ "$navigation" = manual ] || fail "invalid navigation mode: $navigation"

export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
source /opt/ros/noetic/setup.bash
source "$fastlio_ws/devel/setup.bash"
[ "$elevation" = false ] || source "$elevation_ws/devel/setup.bash" --extend
[ "$tare_enabled" = false ] || source "$tare_ws/devel/setup.bash" --extend
source "$scan_ws/devel/setup.bash" --extend
source "$sensor_scan_ws/devel/setup.bash" --extend
export PYTHONPATH="$scan_ws/devel/lib/python3/dist-packages:${PYTHONPATH:-}"
export ROS_PACKAGE_PATH="$go2_root:$go2_root/integration:$ariadne/src:$scan_ws/src:$sensor_scan_ws/src:$ROS_PACKAGE_PATH"
unset ROS_HOSTNAME
export ROS_MASTER_URI="http://localhost:11311"
if [ -z "${ROS_IP:-}" ]; then
  export ROS_IP="$(ip -4 addr show wlan0 2>/dev/null | sed -n 's/.*inet \([0-9.]*\)\/.*/\1/p' | head -1)"
  [ -n "$ROS_IP" ] || export ROS_IP="$(ip -4 route get 1.1.1.1 | sed -n 's/.* src \([^ ]*\).*/\1/p' | head -1)"
fi
[ -n "${ROS_IP:-}" ] || fail 'cannot determine NX ROS_IP'

[ "$(rospack find fast_lio)" = "$fastlio_ws/src/FAST_LIO" ] || fail 'wrong fast_lio package resolved'
[ "$(rospack find scan_planner)" = "$scan_ws/src/planner/plan_manage" ] || fail 'wrong scan_planner package resolved'
[ "$elevation" = false ] || [ "$(rospack find go2_elevation)" = "$go2_root/algorithms/elevation_mapping/go2_integration" ] || fail 'wrong go2_elevation package resolved'
[ -x "$fastlio_ws/devel/lib/fast_lio/fastlio_mapping" ] || fail 'missing FAST-LIO executable'
[ -x "$fastlio_ws/devel/lib/livox_ros_driver2/livox_ros_driver2_node" ] || fail 'missing Livox driver executable'
[ -x "$scan_ws/devel/lib/scan_planner/scan_planner_node" ] || fail 'missing SCAN executable'
[ "$elevation" = false ] || [ -x "$elevation_ws/devel/lib/elevation_mapping/elevation_mapping" ] || fail 'missing elevation_mapping executable'
if [ "$tare_enabled" = true ]; then
  [ "$(rospack find tare_planner)" = "$tare_ws/src/tare_planner" ] || fail 'wrong tare_planner package resolved'
  for executable in tare_planner/tare_planner_node terrain_analysis/terrainAnalysis terrain_analysis_ext/terrainAnalysisExt; do
    [ -x "$tare_ws/devel/lib/$executable" ] || fail "missing TARE executable: $executable"
  done
fi
if [ "$motion" = true ]; then
  [ -x "$motion_ws/build/go2_standup" ] || fail 'missing RecoveryStand helper'
  [ -x "$motion_ws/build/cmd_vel_bridge" ] || fail 'missing Go2 cmd_vel bridge'
fi

echo "[FAST-LIO NX] navigation=$navigation exploration=$exploration motion=$motion mode=$mode elevation=$elevation continuous=$continuous ROS_IP=$ROS_IP"
echo '[PARAM] FAST-LIO input/odom=10Hz; SCAN cloud=5Hz; max_v=.55 max_w=.80'
if [ "$tare_enabled" = true ]; then
  echo "[PARAM] TARE indoor 1Hz; vp=${vp}m (50x50); lookahead=4m, extend=${extend}; path=${path}; SCAN Z=current body Z"
elif [ "$elevation" = true ]; then
  echo '[PARAM] elevation mode: official local elevation map drives AR; global display map=0.2m/0.5Hz/cap100000 cells'
else
  echo '[PARAM] default 2-D mode: AR mapping Z is locked; SCAN samples current body Z once per new goal'
fi
echo '[GUI] keep using ~/Go2/2D/launch_real_rviz.sh; legacy topics are preserved'

if [ "$check" = true ]; then
  roslaunch --nodes "$script_dir/fastlio_stack_NX.launch" \
    fastlio_config:="$fastlio_config" driver_config:="$driver_config" \
    navigation_mode:="$navigation" mode:="$mode" rviz:="$rviz" elevation:="$elevation" continuous:="$continuous" exploration:="$exploration" vp:="$vp" extend:="$extend" path:="$path"
  exit 0
fi

mkdir -p "$log_dir"
exec 9>"$log_dir/launcher.lock"
flock -n 9 || fail 'another FAST-LIO navigation launcher owns the NX stack'
if pgrep -f '[l]ivox_ros_driver2_node|[f]astlio_mapping|[p]ointlio_mapping|[s]can_planner_node|[c]losed_loop_controller|[c]md_vel_bridge|[r]l_planner.py|[o]ctomap_server_node|[f]astlio_output_adapter.py|[s]can_cloud_accumulator.py|[e]levation_mapping|[g]o2_elevation_products|[g]o2_elevation_visualization|[t]are_planner_node|[t]errainAnalysis|[t]are_input_bridge.py|[t]are_scan_bridge.py' >/dev/null; then
  fail 'a local LiDAR/SLAM/navigation process is already active; stop its owning launcher first'
fi

# NX receives LiDAR directly, but the board is MID360's PTP master.
# Align the idle board system/PHC before acquisition; never rewrite ROS stamps.
timeout 60 python3 "$script_dir/sync_mid360_clock.py" 2>&1 \
  | tee "$log_dir/clock.log" \
  || fail 'NX -> board -> PHC -> MID360 clock synchronization failed'

# FAST-LIO fixes its map origin at startup. For motion=true, issue one
# RecoveryStand command and wait for that command helper to finish before
# consuming LiDAR/IMU. Do not infer posture from SportModeState.mode here;
# the initial prone/stand height is selected solely from the motion argument.
if [ "$motion" = true ]; then
  "$motion_ws/build/go2_standup" eth10 2>&1 | tee "$log_dir/stand.log" \
    || fail 'RecoveryStand command failed'
fi

pids=()
cleaned=false
cleanup() {
  [ "$cleaned" = false ] || return 0
  cleaned=true
  trap - EXIT INT TERM
  set +e
  for ((index=${#pids[@]}-1; index>=0; index--)); do kill -INT -- "-${pids[index]}" 2>/dev/null; done
  for ((round=0; round<40; round++)); do
    alive=false
    for pid in "${pids[@]}"; do kill -0 -- "-$pid" 2>/dev/null && alive=true; done
    [ "$alive" = true ] || break
    sleep 0.2
  done
  for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null; done
  echo '[FAST-LIO NX] stopped only processes owned by this launcher; board driver was not touched'
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
start() {
  local logfile="$1"
  shift
  setsid "$@" >"$log_dir/$logfile" 2>&1 &
  pids+=("$!")
}
start_console() {
  local logfile="$1"
  shift
  setsid "$@" > >(tee -a "$log_dir/$logfile") 2>&1 &
  pids+=("$!")
}

if ! timeout 2 rosparam list >/dev/null 2>&1; then
  start roscore.log roscore
  for ((round=0; round<30; round++)); do timeout 1 rosparam list >/dev/null 2>&1 && break; sleep 0.2; done
  timeout 2 rosparam list >/dev/null 2>&1 || fail 'roscore did not start'
fi
[ "$(rosparam get /use_sim_time 2>/dev/null || echo false)" != true ] || fail 'live FAST-LIO cannot use simulation time'

start stack.log roslaunch "$script_dir/fastlio_stack_NX.launch" \
  fastlio_config:="$fastlio_config" driver_config:="$driver_config" \
  navigation_mode:="$navigation" mode:="$mode" rviz:="$rviz" elevation:="$elevation" continuous:="$continuous" exploration:="$exploration" vp:="$vp" extend:="$extend" path:="$path"
stack_pid="${pids[-1]}"
wait_topic() {
  local topic="$1"
  local attempts="${2:-20}"
  for ((attempt=0; attempt<attempts; attempt++)); do
    timeout 3 rostopic echo -n1 --noarr "$topic" >/dev/null 2>&1 && return 0
    kill -0 "$stack_pid" 2>/dev/null || return 1
  done
  return 1
}

wait_topic /livox/lidar 15 || fail 'no MID360 raw cloud; inspect stack.log'
wait_topic /livox/imu 10 || fail 'no MID360 IMU; inspect stack.log'
wait_topic /Odometry 20 || fail 'no FAST-LIO odometry; inspect stack.log'
wait_topic /LIO/clouds_lidar 20 || fail 'FAST-LIO adapter did not publish SCAN cloud'
wait_topic /LIO/odom_vehicle 10 || fail 'FAST-LIO adapter did not publish vehicle odometry'
wait_topic /grid_map/occupancy 30 || fail 'SCAN did not publish its official occupancy map'
if [ "$tare_enabled" = false ]; then
  wait_topic /projected_map 30 || fail 'display map did not publish /projected_map'
fi
if [ "$tare_enabled" = true ]; then
  wait_topic /tare/registered_scan 20 || fail 'TARE same-stamp input is missing'
  wait_topic /tare/terrain_map 30 || fail 'official TARE local terrain map is missing'
  wait_topic /tare/terrain_map_ext 30 || fail 'official TARE extended terrain map is missing'
fi
if [ "$elevation" = true ]; then
  wait_topic /local_elevation_map 30 || fail 'elevation mode did not publish /local_elevation_map'
  wait_topic /global_elevation_map 30 || fail 'elevation mode did not publish /global_elevation_map'
fi
python3 "$script_dir/check_odom_health.py" _topic:=/Odometry _duration:=8.0 \
  _min_rate:=8.0 _max_gap:=0.2 _max_age:=0.5 _max_future:=0.05 _min_stamp_progress:=0.9 \
  || fail 'FAST-LIO odometry health check failed'

start_console navigation_status.log python3 "$script_dir/navigation_status_monitor.py" _exploration:="$exploration"

echo "[READY] FAST-LIO、SCAN和所选地图已就绪；exploration=$exploration"
if [ "$navigation" = auto ]; then
  if [ "$exploration" = tare ]; then
    echo '[READY] TARE室内探索已选择（1Hz）；AR建图和决策均关闭'
  else
    echo '[READY] AR自动探索决策已启用（1.5Hz）'
  fi
else
  echo '[READY] AR仅建图；请在RViz使用2D Nav Goal发送SCAN手动目标'
fi

if [ "$motion" = true ]; then
  start cmd_vel_bridge.log "$motion_ws/build/cmd_vel_bridge" _interface:=eth10 _cmd_timeout_s:=0.5
else
  echo '[SAFE] motion=false：未调用RecoveryStand，未启动Unitree运动桥'
fi
rosparam set /ariadne/execution_ready true
rosparam set /exploration/execution_ready true
if [ "$tare_enabled" = true ]; then
  start tare_start.log rostopic pub -l /tare/start_exploration std_msgs/Bool 'data: true'
fi
if [ "$record_bag" = true ]; then
  start rosbag.log rosbag record -O "$log_dir/fastlio_$(date +%Y%m%d_%H%M%S).bag" \
    /livox/lidar /livox/imu /Odometry /cloud_registered /LIO/clouds_lidar \
    /LIO/odom_vehicle /grid_map/occupancy /projected_map /cmd_vel /tf /tf_static
  if [ "$elevation" = true ]; then
    start elevation_rosbag.log rosbag record -O "$log_dir/elevation_$(date +%Y%m%d_%H%M%S).bag" \
      /local_elevation_map /local_elevation_cloud /local_traversability_map /global_elevation_map
  fi
fi

while kill -0 "$stack_pid" 2>/dev/null; do
  for pid in "${pids[@]}"; do kill -0 "$pid" 2>/dev/null || fail "owned child $pid exited; inspect $log_dir"; done
  sleep 1
done
fail 'FAST-LIO navigation stack exited'
