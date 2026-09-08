#!/bin/bash
# 一键打开 RViz 连 NX 的 real 链，接收实时数据。
# 前置：NX 已运行 `bash ~/Go2/2D/go2-scan/real/launch_real.sh`（算法链已起）。
# 用法：bash launch_real_rviz.sh [NX_IP]
#   NX_IP 默认取手机热点地址 10.92.117.228（热点可能变，变了就传参，如 10.92.117.230）
set -e

# 注意顺序：必须先 source noetic，再设非默认的复合 ROS_PACKAGE_PATH。
# 否则 source 会把 ROS_PACKAGE_PATH 整个重置成 /opt/ros/noetic/share，
# 导致 go2_description/livox 网格找不到、go2_robot 报 error。
source /opt/ros/noetic/setup.bash

NX_IP="${1:-10.92.117.228}"
GP=/home/pumpkin-db/Go2/2D/go2-scan
rviz_source="$GP/algorithms/local_planning/scan_planner/src/planner/plan_manage/launch/default.rviz"
[ -r "$rviz_source" ] || { echo "[FATAL] 找不到RViz配置: $rviz_source" >&2; exit 2; }

# 保留GUI电脑现有的AR/SCAN布局，在临时副本中删除单帧
# /registered_scan，并把 /scan_map 强制设为反射强度彩虹。
runtime_dir="${XDG_RUNTIME_DIR:-/tmp}"
rviz_runtime="$(mktemp "$runtime_dir/go2-real-rviz.XXXXXX.rviz")"
cleanup() { rm -f "$rviz_runtime"; }
trap cleanup EXIT
python3 - "$rviz_source" "$rviz_runtime" <<'PY'
import sys
import yaml

source_path, output_path = sys.argv[1:]
with open(source_path, encoding='utf-8') as stream:
    config = yaml.safe_load(stream)

def is_registered_scan(value):
    return (isinstance(value, dict) and
            (value.get('Topic') == '/registered_scan' or
             value.get('Name') == 'registered_scan'))

def remove_registered_scan(value):
    if isinstance(value, dict):
        for child in value.values():
            remove_registered_scan(child)
    elif isinstance(value, list):
        value[:] = [child for child in value if not is_registered_scan(child)]
        for child in value:
            remove_registered_scan(child)

remove_registered_scan(config)

matches = []
def visit(value):
    if isinstance(value, dict):
        if value.get('Topic') == '/scan_map' or value.get('Name') == 'scan_map':
            matches.append(value)
        for child in value.values():
            visit(child)
    elif isinstance(value, list):
        for child in value:
            visit(child)
visit(config)

if matches:
    scan_map = matches[0]
else:
    scan_map = {
        'Alpha': 1,
        'Class': 'rviz/PointCloud2',
        'Decay Time': 0,
        'Name': 'scan_map',
        'Position Transformer': 'XYZ',
        'Queue Size': 2,
        'Selectable': True,
        'Size (Pixels)': 2,
        'Size (m)': 0.05,
        'Style': 'Flat Squares',
        'Topic': '/scan_map',
        'Unreliable': False,
        'Use Fixed Frame': True,
    }
    config['Visualization Manager']['Displays'].append(scan_map)

scan_map.update({
    'Autocompute Intensity Bounds': False,
    'Autocompute Value Bounds': {'Max Value': 255, 'Min Value': 0, 'Value': False},
    'Channel Name': 'intensity',
    'Color Transformer': 'Intensity',
    'Enabled': True,
    'Invert Rainbow': False,
    'Max Intensity': 255,
    'Min Intensity': 0,
    'Topic': '/scan_map',
    'Use rainbow': True,
    'Value': True,
})

with open(output_path, 'w', encoding='utf-8') as stream:
    yaml.safe_dump(config, stream, allow_unicode=True, sort_keys=False)
print('[launch_real_rviz] removed /registered_scan; scan_map: Intensity/intensity, Rainbow, 0-255')
PY

# 连到 NX master；本机 ROS_IP 取 wlo1（与 NX 同网段）
export ROS_MASTER_URI="http://$NX_IP:11311"
export ROS_IP="$(ip -4 addr show wlo1 2>/dev/null | sed -n 's/.*inet \([0-9.]*\)\/.*/\1/p' | head -1)"

# 网格包路径：go2_description + livox_laser_simulation（缺了 go2_robot 会报 error）
export ROS_PACKAGE_PATH=$GP/algorithms/local_planning/scan_planner/src/simulator/Utils:$GP/simulation/cmu_env/src:$ROS_PACKAGE_PATH

echo "[launch_real_rviz] ROS_MASTER_URI=$ROS_MASTER_URI"
echo "[launch_real_rviz] ROS_IP=$ROS_IP"
echo "[launch_real_rviz] ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH"
echo "[launch_real_rviz] 打开 RViz: 隐藏单帧registered_scan + scan_map强度彩虹"
rosrun rviz rviz -d "$rviz_runtime"
