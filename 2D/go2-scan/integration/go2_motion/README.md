# cmd_vel_bridge — ROS1 /cmd_vel → Go2 CycloneDDS 直通桥（第一轮安全版）

从 `~/cmd_vel_bridge` 迁移。绕开 ROS2/ros1_bridge/Fast-DDS，把 ROS1 `/cmd_vel`
(geometry_msgs/Twist) 经 `unitree_sdk2` + CycloneDDS 直发 Go2 `/api/sport/request`。

## 第一轮正式启动流程（人工，勿自动）
```
1) Go2 上电 → 等 2-3 分钟自动站立（或人工/原生确认已站稳）
2) 确认 NX eth10(192.168.123.18) ping 通 Go2(192.168.123.161)
3) bash real/launch_fastlio_for_scan-planner_NX.sh motion:=true
4) 检查 scan_map/projected_map 正常、SCAN 开始规划
5) 最后人工单独启动 cmd_vel_bridge
```

## 第一轮 bridge 参数（默认已安全）
```bash
~/Go2/2D/go2-scan/integration/go2_motion/build/cmd_vel_bridge \
  _interface:=eth10 _auto_stand:=false _disable_avoid:=false _max_linear_speed:=0.5
```

- `_auto_stand:=false`：默认**不自动站立**（狗已先站好；需要时 `_auto_stand:=true` 会 RecoveryStand）。
- `_disable_avoid:=false`：默认**保持 Go2 原生避障开启**，不调用 `SwitchSet(false)`。
- `_max_linear_speed:=0.5`：平面速度模长硬限 0.5 m/s（超按比例缩放保持方向）。
- `_max_angular_speed:=0.5`：angular.z 硬限 0.5 rad/s（SCAN 上限 kMaxVYawLimit=1.0）。

## 安全确认后再考虑
```bash
_max_linear_speed:=0.75
```
（升速前先确认原生避障开启下运动稳定。）

## 编译（沿用旧工程方式）
```bash
cd integration/go2_motion && mkdir -p build && cd build && cmake .. && make -j2
```
默认链接仓库内 `third_party/unitree_sdk2`（含 CycloneDDS ddsc/ddscxx）；也可用
`UNITREE_SDK_ROOT` 显式覆盖 SDK 路径。
