# 2026-09-27 栅格前沿兜底验证

## 已接入内容

rl_planner.py的continuous_frontier_waypoint：原有搜索返回None且没有RL提议时，调用frontier_grid_fallback.py；历史/正常策略有结果时完全不调用。
机器人出发位置采用实际里程计；OccupancyGrid原点按格子角点处理，候选输出格子中心。
采用已安装scikit-image MCP_Geometric的四邻接搜索：仅值0已知空闲可通行；未知和占据不可穿越。官方依据：https://scikit-image.org/docs/stable/api/skimage.graph.html#skimage.graph.MCP_Geometric 。
近的前沿簇优先（按1m距离段，同段数量优先）；只选择空闲路径能到的前沿邻域候选，按实际路径截取中间目标。真实launch的next_waypoint_threshold=4m，测试已同步4m。没有新增状态机、超时、节点或永久屏蔽。
复用/ariadne_goal_bridge/target_min_clearance和target_min_robot_distance，当前0.4m/0.8m。端点障碍距离用EDT减半格对角线保守估计；路径通过性仍由SCAN检查，未把0.4m作为全路径硬膨胀。

## 测试发现并修复的问题

初版纯最短路径在门框附近贴墙，连续执行第一次有目标、第二次返回空；因此没有停留在单帧出点即通过。
改为带障碍距离软代价的MCP路径：1+clearance/max(障碍边界距离,半格)，保持硬通行约束不变，避免选择始终贴墙而无法截取安全中间点的路径。多步测试重新通过。
集成测试初版导入fixture类后删除名称导致5个NameError，修复测试导入方式；非生产算法故障。

## 验证结果

- test_frontier_grid.py：14项通过。封闭墙不发对侧点、有门绕行、近不可达换远可达、未知墙不可穿、占据/未知/越界起点无错误连通、斜角不穿、净空不足不发、失败位置排除、无前沿无目标、多步无循环、0.8m提前到达、多原点坐标、门关闭/打开即时重算；另100张固定随机地图与独立四邻接BFS核对路径连通、逐步相邻及精确矩形障碍端点净空。
- test_grid_integration.py：6项通过。抽取执行真实Runner.continuous_frontier_waypoint和run函数；实际桥choose验证新目标不被挪动；策略/历史优先；封闭墙；沿用桥参数；pending禁止新决策、匹配成功解除等待。未实例化完整RL网络/节点。
- test_history_return.py：13项通过；test_directional_exploration.py：13项通过。共46项，100随机地图包含在其中1项。
- git diff --check通过。真实运行环境完整导入rl_planner模块另行核验，不启动Runner或硬件。
- 5次合成空地图选择耗时：200×200格中位36.9ms/最大43.1ms；500×500格中位223.2ms/最大227.2ms（0.1m格）。仅单机微基准，不代表实际地图/长期CPU。仅在原搜索失败时执行；未做缓存。

## 重现

在/home/unitree/Go2下，使用.venv/ariadne/bin/python，PYTHONPATH加入/opt/ros/noetic/lib/python3/dist-packages及2D/go2-scan/algorithms/local_planning/scan_planner/devel/lib/python3/dist-packages。
分别运行integration/go2_bridge/scripts下上述四个test文件。

## 证据边界与恢复

通过的是合成地图、真实选点函数及消息状态单元/集成测试；机器人按理想运动推进，0.8m测试模拟提前停在目标前0.79m。没有真实SCAN路径执行、实际狗动力学、现场地图回放或探索覆盖率验收。因此不宣称任意环境探索完备、必定不卡或目标必定可通行。
历史轨迹仍优先，可跨越旧地图的障碍记录，这种模式仍由SCAN用当前局部障碍判断；本次“封闭墙不跨越”性质只针对新栅格兜底，不针对历史链。
utils背景label=0旧问题未在本次修改，可能在进入兜底前影响节点更新；本轮不扩大修改范围。
旧test_goal_feedback和test_frontier_fallback仍有此前已记录的接口不匹配，未计入通过数量。
备份：Go2/backups/ar_grid_before_20260927_1513/rl_planner.py。回退该文件即可移除接入，独立模块和测试可保留。无需重编SCAN，重启导航加载；本轮没有启停机器人/GUI，没有推送。
