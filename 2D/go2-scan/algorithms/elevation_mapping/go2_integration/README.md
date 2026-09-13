# Go2 高程地图集成目录

本目录集中保存 Go2 项目为平地、斜坡和连续起伏地形增加的高程处理代码。

## 目录约定

- `config/`：MID360、FAST-LIO 和高程图参数。
- `launch/`：独立验证入口；验证完成前不改生产启动链。
- `scripts/`：ROS 话题、位姿协方差和地图语义适配器。
- `src/`：确有性能需求时才加入的 C++ 节点。
- `test/`：rosbag 回放和无运动验证工具。

仓库根目录其余内容来自官方 `ANYbotics/elevation_mapping`。优先通过本目录适配，避免直接修改官方核心算法；必须修改上游代码时，要记录原因和差异。

## 当前目标

1. 使用 FAST-LIO 当前帧点云和同步位姿建立机器人中心局部高程图。
2. 根据局部坡度、粗糙度和高度连续性生成二维可通行地图。
3. 让 AR 使用可通行地图，不再根据 FAST-LIO 绝对 Z 移动整张 OctoMap 高度切片。
4. 连续坡面验证稳定后再评估离散台阶。

## 当前状态

- 官方核心：`ANYbotics/elevation_mapping`，`master@f4b082c64a3e660980da53b33c7936a8f2a2ea22`。
- 已接入 NX FAST-LIO、AR 和 SCAN；官方核心代码保持不改，Go2 适配集中在本目录。
- `~/Go2/build_all.sh` 会建立并编译独立工作空间 `~/Go2/2D/ELEVATION-MAPPING`。

## 两种运行模式

默认模式不启动高程算法，完全使用固定高度：

```bash
cd ~/Go2/2D/go2-scan/real
./launch_fastlio_NX.sh motion:=false
```

- `motion:=false`：固定地面 `z=-0.310m`。
- `motion:=true`：固定地面 `z=-0.565m`。
- 已删除原先按 FAST-LIO Z 每 8cm 移动高度切片的逻辑。

只有显式传入 `elevation:=true` 才启动高程模式：

```bash
./launch_fastlio_NX.sh motion:=false elevation:=true
```

- `/local_elevation_map`：12m × 12m、0.1m 分辨率、2Hz 的局部高程/可通行图，供 AR 和路径高度使用。
- `/global_elevation_map`：0.2m 稀疏点云、0.5Hz、最多 100000 格，只供 RViz 观察全局地形。
- `/projected_map`：由局部可通行图累计成 AR 仍然使用的二维 OccupancyGrid。
- SCAN 自己的三维占据图、碰撞检查和 `/scan_map` 均不改变。

GUI 电脑需要 `ros-noetic-grid-map-rviz-plugin`。运行原入口即可：

```bash
~/Go2/2D/launch_real_rviz.sh
```

RViz 中新增 `local_elevation_map`（官方 GridMap 显示）和
`global_elevation_map`（按 elevation 彩虹着色）。固定高度模式下这两个显示无消息，
不影响原有 AR 蓝格、二维占据图和 SCAN 显示。
