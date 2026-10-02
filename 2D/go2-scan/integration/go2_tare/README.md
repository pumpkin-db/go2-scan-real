# NX室内二维探索：TARE / ARiADNE + SCAN

## 当前入口与可选参考路径（2026-10-01）

```bash
./launch_fastlio_NX.sh motion:=true                  # 默认参考路径，vp=0.5，extend=false
./launch_fastlio_NX.sh motion:=true path:=false      # 切回单目标
./launch_fastlio_NX.sh motion:=true path:=true       # TARE路线段作为SCAN参考
./launch_fastlio_NX.sh motion:=false path:=true      # 不启动狗运动，仅看数据/目标
./launch_fastlio_NX.sh motion:=true vp:=0.2          # 自选视点间距
./launch_fastlio_NX.sh motion:=true path:=false extend:=true # 单目标模式可选恢复直线延伸
```

`path`在auto+TARE下默认true（2026-10-02更新），manual/AR原接口不变；path:=false切回单目标。true时强制关闭extend，避免把路线终点直线推到墙后。SCAN参考路径模式为3、单目标为1，速度限制0.55m/s、0.8rad/s，到达仍XY<0.6m。不是三维楼梯模式，路径高度固定为当前狗体Z，不使用TARE传感器Z控制狗。

TARE在原有GetLookAheadPoint选择分支中导出`/tare/execution_path`（nav_msgs/Path，map），仅包含当前位置至所选前瞻点的顺序路径段，保留拐角和末段插值；局部前向/反向、环路和全局回程分支分别按实际选择导出，不直接转发整条TSP环路。此输出仅在path=true启用，不改变视点收益、方向选择、覆盖或全局探索算法。

桥接在path=true时只接收该路径，不同时消费`/tare/way_point`。输出`/initial_path`（world，当前map/world为恒等），唯一stamp与SCAN GoalFeedback匹配；活动任务期间只缓存最新意图，不替换正在执行的任务。成功或失败后重新从最新狗位置截去缓存路径已走前缀，保留剩余拐角并交接下一段。距离<=0.6m的空操作、空路径、非有限坐标不发。路径终点XY与本次任务目标相同；SCAN可能按已有占据终点缩短逻辑修正有效终点，其反馈仍按有效终点判定。

path=false仍走原单目标桥`/tare/way_point -> /move_base_simple/goal`，TARE保持规划，桥等待SCAN终态后交接最新目标，未启用算法内部等待。此前文档中“已默认使用/initial_path、前瞻3m、TARE内部暂停”是早期接入状态，以此节和当前源码为准：现为前瞻4m、vp默认0.5、动态清除true/阈值1、extend默认false。

GUI继续使用原`launch_real_rviz.sh`，TARE/01目标路径已有`/initial_path`显示，无需更换GUI脚本；绿色为桥交给SCAN的路径，原生局部/全局路线和SCAN轨迹保留。路径是优化参考，不是强制轨道，也不会自动修复TARE陈旧障碍或错误路线。

验证：TARE实际编译通过；旧/新桥离线测试、路径方向/拐角/插值测试通过；提取新旧实际GetLookAheadPoint代码比较600组路线夹具，选点/返回值/LOS不变且导出终点一致。独立ROS11338仅运行真实SCAN+新桥，用合成位姿验证ACTIVE、等待、SUCCEEDED和FAILED后交接，未运行雷达/控制器/Unitree。尚未进行实体运动验收，不能声称已验证实际绕门或动态障碍恢复。

## 启动

NX（正式入口名称是fastlio，不是fatlio）：
```bash
cd ~/Go2/2D/go2-scan/real
bash launch_fastlio_NX.sh motion:=true                    # 默认TARE
bash launch_fastlio_NX.sh exploration:=ariadne motion:=true
bash launch_fastlio_NX.sh motion:=false                   # 不起立、不启运动桥；按prone高度
bash launch_fastlio_NX.sh --check                         # 只展开，不启动硬件
```
GUI：
```bash
bash ~/Go2/2D/launch_real_rviz.sh
# 自动读取NX /exploration/algorithm；也可明确指定：
bash ~/Go2/2D/launch_real_rviz.sh exploration:=tare
```
先启动NX，再启动GUI；切换算法后重开RViz。NX不可达时GUI显示默认TARE并提示。manual入口仍展示AR地图但不运行探索决策。motion=false的prone参数不是站立验收替代品。

## 官方来源与室内配置

官方仓库 https://github.com/caochao39/tare_planner ，melodic-noetic，44500592b86138257273e0cab264e6a847ccefc7。官方明确提供indoor及Matterport室内运行方法。本接入加载原始indoor.yaml（1Hz规划、单层候选观察点），不改算法源码。ARM OR-Tools9.8.3296和官方terrain_analysis/terrain_analysis_ext已编译。

为适配SCAN局部终点接口，在tare_fastlio.launch覆盖：
- kAutoStart=false：由正式脚本数据链就绪后发送开始消息。
- kExtendWayPoint=false：取消官方方向跟随器所用的8m/3.5m向前延长，避免延长点超出SCAN约10m宽局部地图或越过转角。
- kLookAheadDistance=3m：沿TARE局部路径选前视目标；它不是严格的最大目标距离限幅。
- kRushHomeDist=3m；kAtHomeDistThreshold=1m：与SCAN现有0.6m到达阈值及LiDAR/body参考点偏移配套，避免SCAN停了但TARE要求继续靠近至0.5m。
- 官方探索完成返航保留。terrain模块vehicleHeight stand=.565/prone=.310（障碍相对地面高度范围）。

## 数据链和二维高度

FAST-LIO /mid360_points + /quad_0/lidar_pose（map系，同时间戳）
→ tare_input_bridge不超过5Hz
→ /tare/registered_scan、/tare/state_estimation、/tare/state_estimation_at_scan
→ 官方terrain模块 /tare/terrain_map、/tare/terrain_map_ext
→ TARE /tare/way_point
→ tare_scan_bridge /initial_path（world；map/world恒等变换）
→ SCAN规划及closed_loop_controller /cmd_vel
→ motion=true才启动Unitree运动桥。

地形intensity是相对地面高度/代价，不是反射强度。TARE目标保留XYZ传到SCAN，但SCAN当前use_path_height=false，收到新目标时以当前狗体Z规划，按XY到达。不是跨层上楼方案。TARE保持原始重力对齐坐标，不新增Z锁定。
实体TARE模式启用单目标反馈握手：TARE发布目标后暂停后续自主目标发布，SCAN经目标桥回报SUCCEEDED或FAILED后才释放，随后TARE从最新规划产生不同的下一目标；同一XY位置0.5m内不会直接重发。TARE的地形/覆盖/路径规划循环不暂停。原生手动reset仍会发布近身停止点，但当前SCAN目标桥会忽略近身点，不能将它当作已验证的SCAN急停。不应用AR目标筛选。SCAN原起点占据失败未在此修改，换探索算法不改变碰撞判定。如果SCAN没有终态反馈，TARE按要求保持等待，不自动超时换目标。

## 显示与互斥

TARE模式不启动AR决策、cloud_range_filter、OctoMap和投影地图；仍发布恒等map/world TF。AR和manual原地图链保持。
GUI与NX rviz:=true提供独立TARE组：累计观测图、局部地形、前沿/未覆盖前沿、选中观察点、全局/局部路径、原生目标、送往SCAN的目标路径、探索子空间和局部范围默认显示；扩展地形、所有候选点、规划表面、状态文字标签默认关闭，可按需勾选。原有Planning/Mapping/狗模型保留，AR组在TARE模式关闭。
当前实际配置中，TARE目标是紫色半径0.15m；终端监视器每3秒汇总TARE规划/等待反馈和SCAN执行状态。原先自制的`/tare/display/region_status`空区域显示已删除：它不是官方RViz项目，源自官方`grid_world_marker`但初期/无区域时会是空Marker。官方`Exploring subspaces`仍保留。原先`/sensor_coverage_planner/exploration_path`仅广告发布者，实际发布被源码注释，因此从RViz移除。
话题来源：`/tare/way_point`、`/sensor_coverage_planner/{global_path,local_path,tare_visualizer/exploring_subspaces,tare_visualizer/local_planning_horizon,planner_cloud}`是TARE原生；`/tare/terrain_map{,_ext}`是官方terrain模块经本接入重映射；`/tare/observed_map`、`/tare/display/{frontiers,uncovered_frontiers,selected_viewpoints,viewpoints}`是本接入显示分支，后四者转发TARE原生点云并保留最后快照；`/initial_path`由TARE→SCAN桥生成。局部/前沿等原生数据为空时，对应显示也可能为空，不等于RViz配置错误。
原生目标位于传感器高度；实际SCAN目标看Planning/goal_point（狗体高度）。Goal sent to SCAN显示桥发送的参考路径，Z会被SCAN二维处理重置。

## 验证和限制

本轮TARE/AR/manual启动展开通过；默认TARE图无AR/OctoMap，保留唯一SCAN控制链。正式参数展开确认室内覆盖值和use_path_height=false；GUI三模式配置生成验证通过。
隔离ROS11330运行官方TARE、两种terrain模块、输入桥、目标桥和真实scan_planner_node；31组同帧数据/地形输出；明确注入(2,1,.60912)目标后SCAN采用body Z=.34803，模拟位姿距目标.5m返回REACHED。没有Unitree或运动控制器。静态场景中TARE原生仅返回起点，不宣称已验证原生长距离自主探索。首轮夹具遗漏全局body_pose_topic导致INVALID_OR_NOT_READY，补齐与正式图相同参数后通过，未为通过测试改变生产逻辑。
尚未验证本轮真实楼道、CPU负载、实际GUI渲染和实体运动；不能保证旧起点占据问题自动消失。

## 备份

NX ~/Go2/backups/tare_complete_20260928_211928：本轮旧文件及检查/测试日志；按相对目录覆盖旧文件可恢复前一轮接入状态。GUI ~/Go2/backups/tare_complete_20260928_211928/launch_real_rviz.sh。新tare_rviz.py与rviz/tare.rviz可在回退后保留（旧入口不使用）。前轮完整接入备份tare_integration_20260928_205248。
