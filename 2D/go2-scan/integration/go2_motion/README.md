# cmd_vel_bridge — ROS1 /cmd_vel → Go2 CycloneDDS 直通桥

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
  _interface:=eth10 _cmd_timeout_s:=0.5
```

- 站立由正式启动脚本在启动传感器前统一处理，运动桥不再重复处理姿态。
- 运动桥不修改 Go2 原生避障状态。
- SCAN 的速度原样传给 `SportClient::Move()`，不再设置第二套速度死区。
- 运动桥不再二次裁剪线速度和角速度，原样转发SCAN命令；速度上限统一由SCAN配置负责。
- 只保留命令超时、非有限值拒绝、明确零速和退出时`StopMove()`。

## 编译（沿用旧工程方式）
```bash
cd integration/go2_motion && mkdir -p build && cd build && cmake .. && make -j2
```
默认链接仓库内 `third_party/unitree_sdk2`（含 CycloneDDS ddsc/ddscxx）；也可用
`UNITREE_SDK_ROOT` 显式覆盖 SDK 路径。
