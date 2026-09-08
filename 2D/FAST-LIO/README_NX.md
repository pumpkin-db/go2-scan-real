# NX 独立 FAST-LIO

这是独立验证工作区，不接入 AR、SCAN、模型狗、运动控制或旧的话题适配器。

- FAST-LIO 官方提交：`7cc4175de6f8ba2edf34bab02a42195b141027e9`
- livox_ros_driver2 提交：`13eb05e4e6dd7a765b934d0c5fd6236676a57b49`（与 NX 已安装 SDK2 匹配）
- 对 FAST-LIO 的代码改动仅为把 `livox_ros_driver` 消息依赖替换成 MID360 使用的 `livox_ros_driver2`。
- 雷达 IP：`192.168.123.201`；NX 接收 IP：`192.168.123.30`。
- 输出保持 FAST-LIO 原生坐标与话题，不增加 132 度旋转。
- PCD 自动保存已关闭。
- driver2 对 NX 已安装 SDK2 有一行兼容修改：仅移除当前 SDK 中不存在的 MID360s 枚举分支；MID360 分支不变。

编译：

```bash
cd ~/Go2/2D/FAST-LIO
bash build.sh
```

运行：

```bash
cd ~/Go2/2D/FAST-LIO
bash run.sh rviz:=true driver:=true
```

如果雷达驱动已由其他进程启动：

```bash
bash run.sh rviz:=true driver:=false
```
