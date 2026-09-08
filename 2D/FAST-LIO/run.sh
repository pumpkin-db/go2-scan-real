#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RVIZ="true"
DRIVER="true"

for arg in "$@"; do
  case "$arg" in
    rviz:=true|rviz:=false) RVIZ="${arg#rviz:=}" ;;
    driver:=true|driver:=false) DRIVER="${arg#driver:=}" ;;
    *) echo "Unknown argument: $arg" >&2; exit 2 ;;
  esac
done

source /opt/ros/noetic/setup.bash
source "$ROOT_DIR/devel/setup.bash"

children=()
cleanup() {
  trap - INT TERM HUP EXIT

  for pid in "${children[@]:-}"; do
    kill -INT "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup INT TERM HUP EXIT

echo "[FAST-LIO] official=7cc4175 driver2=13eb05e rviz=$RVIZ driver=$DRIVER"
echo "[FAST-LIO] native outputs: /Odometry /cloud_registered /cloud_registered_body /path"

if [[ "$DRIVER" == "true" ]]; then
  roslaunch livox_ros_driver2 msg_MID360.launch &
  children+=("$!")
  sleep 2
fi

roslaunch fast_lio mapping_nx_mid360.launch rviz:="$RVIZ" &
children+=("$!")
wait
