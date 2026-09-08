#!/bin/bash
# NX MID360 -> FAST-LIO -> official SCAN parameters -> AR map/automatic exploration.
# motion:=false is the default.  Real runs always synchronize clocks before acquisition.
set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
go2_root="$(cd "$script_dir/.." && pwd)"
fastlio_ws="${FASTLIO_WS:-$HOME/Go2/2D/FAST-LIO}"
fastlio_config="${FASTLIO_CONFIG:-$fastlio_ws/src/FAST_LIO/config/mid360.yaml}"
driver_config="${LIVOX_CONFIG:-$fastlio_ws/src/livox_ros_driver2/config/MID360_config.json}"
scan_ws="${SCAN:-$go2_root/algorithms/local_planning/scan_planner}"
sensor_scan_ws="${SENSOR_SCAN_WS:-$HOME/Go2/2D/SENSOR-SCAN}"
ariadne="${ARIADNE:-$go2_root/algorithms/global_planning/ariadne}"
motion_ws="$go2_root/integration/go2_motion"
log_dir="$script_dir/logs/fastlio_navigation"

navigation=auto
motion=false
record_bag=false
rviz=false
check=false
for argument in "$@"; do
  case "$argument" in
    navigation:=auto) navigation=auto;;
    navigation:=manual) navigation=manual;;
    motion:=true) motion=true;;
    motion:=false) motion=false;;
    motion:=*) echo "[FATAL] invalid motion argument: $argument" >&2; exit 2;;
    record_bag:=true) record_bag=true;;
    record_bag:=false) record_bag=false;;
    rviz:=true) rviz=true;;
    rviz:=false) rviz=false;;
    rviz:=*) echo "[FATAL] invalid rviz argument: $argument" >&2; exit 2;;
    config:=*) fastlio_config="${argument#config:=}";;
    driver:=*) echo '[FATAL] FAST-LIO正式入口固定使用NX本地驱动，不接受driver参数' >&2; exit 2;;
    --check) check=true;;
    *) echo "[FATAL] unknown argument: $argument" >&2; exit 2;;
  esac
done

fail() { echo "[FATAL] $*" >&2; exit 42; }
mode=prone
[ "$motion" = true ] && mode=stand
[ -r "$fastlio_ws/devel/setup.bash" ] || fail "FAST-LIO workspace has not been built: $fastlio_ws"
[ -r "$scan_ws/devel/setup.bash" ] || fail "SCAN official workspace has not been built: $scan_ws"
[ -r "$fastlio_config" ] || fail "missing FAST-LIO config: $fastlio_config"
[ -r "$driver_config" ] || fail "missing Livox driver config: $driver_config"
[ -r "$sensor_scan_ws/devel/setup.bash" ] || fail "sensor scan workspace has not been built: $sensor_scan_ws"
[ "$navigation" = auto ] || [ "$navigation" = manual ] || fail "invalid navigation mode: $navigation"

export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
source /opt/ros/noetic/setup.bash
source "$fastlio_ws/devel/setup.bash"
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
[ -x "$fastlio_ws/devel/lib/fast_lio/fastlio_mapping" ] || fail 'missing FAST-LIO executable'
[ -x "$fastlio_ws/devel/lib/livox_ros_driver2/livox_ros_driver2_node" ] || fail 'missing Livox driver executable'
[ -x "$scan_ws/devel/lib/scan_planner/scan_planner_node" ] || fail 'missing SCAN executable'
if [ "$motion" = true ]; then
  [ -x "$motion_ws/build/go2_standup" ] || fail 'missing RecoveryStand helper'
  [ -x "$motion_ws/build/cmd_vel_bridge" ] || fail 'missing Go2 cmd_vel bridge'
fi

echo "[FAST-LIO NX] navigation=$navigation motion=$motion mode=$mode ROS_IP=$ROS_IP"
echo '[PARAM] FAST-LIO input/odom=10Hz; SCAN=current phase-1 parameters; AR map=2Hz, replan=1.5Hz, resolution=0.1m, range=6m'
echo '[PARAM] AR obstacle projection=0.2-0.8m above ground; AR voxel requires two input frames; SCAN cloud is not height-sliced'
echo '[GUI] keep using ~/Go2/2D/launch_real_rviz.sh; legacy topics are preserved'

if [ "$check" = true ]; then
  roslaunch --nodes "$script_dir/fastlio_stack_NX.launch" \
    fastlio_config:="$fastlio_config" driver_config:="$driver_config" \
    navigation_mode:="$navigation" mode:="$mode" rviz:="$rviz"
  exit 0
fi

mkdir -p "$log_dir"
exec 9>"$log_dir/launcher.lock"
flock -n 9 || fail 'another FAST-LIO navigation launcher owns the NX stack'
if pgrep -f '[l]ivox_ros_driver2_node|[f]astlio_mapping|[p]ointlio_mapping|[s]can_planner_node|[c]losed_loop_controller|[c]md_vel_bridge|[r]l_planner.py|[o]ctomap_server_node|[f]astlio_output_adapter.py|[c]md_vel_safety_gate.py|[s]can_cloud_accumulator.py' >/dev/null; then
  fail 'a local LiDAR/SLAM/navigation process is already active; stop its owning launcher first'
fi

# Mandatory real-run order: synchronize before roscore/driver/FAST-LIO starts.
timeout 60 python3 "$script_dir/sync_mid360_clock.py" | tee "$log_dir/clock.log" \
  || fail 'NX -> 3908 -> PHC -> MID360 clock synchronization failed'

# FAST-LIO fixes its map origin at startup. For motion=true, issue one
# RecoveryStand command and wait for that command helper to finish before
# consuming LiDAR/IMU. Do not infer posture from SportModeState.mode here;
# the fixed height model is selected solely from the motion argument.
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

if ! timeout 2 rosparam list >/dev/null 2>&1; then
  start roscore.log roscore
  for ((round=0; round<30; round++)); do timeout 1 rosparam list >/dev/null 2>&1 && break; sleep 0.2; done
  timeout 2 rosparam list >/dev/null 2>&1 || fail 'roscore did not start'
fi
[ "$(rosparam get /use_sim_time 2>/dev/null || echo false)" != true ] || fail 'live FAST-LIO cannot use simulation time'

start stack.log roslaunch "$script_dir/fastlio_stack_NX.launch" \
  fastlio_config:="$fastlio_config" driver_config:="$driver_config" \
  navigation_mode:="$navigation" mode:="$mode" rviz:="$rviz"
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
wait_topic /projected_map 30 || fail 'AR map did not publish /projected_map'
python3 "$script_dir/check_odom_health.py" _topic:=/Odometry _duration:=8.0 \
  _min_rate:=8.0 _max_gap:=0.2 _max_age:=0.5 _max_future:=0.05 _min_stamp_progress:=0.9 \
  || fail 'FAST-LIO odometry health check failed'

echo '[READY] FAST-LIO、SCAN和AR地图已就绪；GUI接口保持不变'
if [ "$navigation" = auto ]; then
  echo '[READY] AR自动探索决策已启用（1.5Hz）'
else
  echo '[READY] AR仅建图；请在RViz使用2D Nav Goal发送SCAN手动目标'
fi

if [ "$motion" = true ]; then
  start cmd_vel_bridge.log "$motion_ws/build/cmd_vel_bridge" _interface:=eth10 _auto_stand:=false \
    _disable_avoid:=false _cmd_timeout_s:=0.5 _max_linear_speed:=0.5 _max_angular_speed:=0.7
else
  echo '[SAFE] motion=false：未调用RecoveryStand，未启动Unitree运动桥'
fi
if [ "$record_bag" = true ]; then
  start rosbag.log rosbag record -O "$log_dir/fastlio_$(date +%Y%m%d_%H%M%S).bag" \
    /livox/lidar /livox/imu /Odometry /cloud_registered /LIO/clouds_lidar \
    /LIO/odom_vehicle /grid_map/occupancy /projected_map /cmd_vel /tf /tf_static
fi

while kill -0 "$stack_pid" 2>/dev/null; do
  for pid in "${pids[@]}"; do kill -0 "$pid" 2>/dev/null || fail "owned child $pid exited; inspect $log_dir"; done
  sleep 1
done
fail 'FAST-LIO navigation stack exited'
