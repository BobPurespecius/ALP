# Rao 2025 独立复现实验入口

> 当前状态：`rao_2025_full_reproduction.py` 是历史算法原型，不是论文完整复现，也不再作为新架构的实现基础。它不会启动或修改 PX4、Gazebo、EGO/EGOv2、door/stair 链路，但其结果只能用于诊断旧实现。

新的论文复现先执行第一阶段协议校验：

```bash
RAO_PROFILE=simulation ./run_rao_2025_stage1.sh
RAO_PROFILE=comparison ./run_rao_2025_stage1.sh
```

协议、论文参数来源、简单重规划契约和统计口径见 `docs/rao_2025_stage1_protocol.md`。后续模块化节点完成前，不应把下方旧入口的启动成功称为论文复现成功。

场景与目标阶段入口为：

```bash
RAO_PROFILE=simulation RAO_DURATION=30 ./run_rao_2025_stage2.sh
RAO_PROFILE=comparison RAO_DURATION=30 ./run_rao_2025_stage2.sh
```

该阶段只发布确定性障碍物几何、bicycle target 真值和 Hybrid A* 目标路径，仍不启动 UAV planner。

阶段 2 的协议节点是唯一配置来源；scene 和 target 只消费同一 manifest，并校验 profile/SHA。目标按论文式 (46) 积分 8 状态 bicycle model，搜索、碰撞检查和 20 Hz 执行共用同一个固定步长 primitive 积分器。waypoint、wheelbase、clearance、搜索离散和权重是论文未公开的复现选择，不是论文参数。

当前验证：12 项测试通过；simulation/comparison 均通过连续两轮闭环和 60 秒 ROS 运行；实测采样频率分别为 `20.0010 Hz`、`20.0067 Hz`，碰撞探针分别检查 300/400 帧，障碍碰撞样本均为 0。`RAO_RVIZ=true` 已验证 RViz 1.14.26 能加载场景、目标 Path、Odometry 和 heading marker。

这仍只是完整复现的阶段 1-2。CV-EKF/FOV sharing、每机前端、凸多面体 corridor、MINCO/L-BFGS、绝对时间互避、评估器和 Zhou baseline 尚未完成，当前数据不得与论文 Table 1 作算法结果比较。

旧原型入口：

在宿主机运行：

```bash
cd /home/bob/ALP/guidance/ros_ws
./run_rao_2025_reproduction.sh
```

脚本复用运行中的 `px4_noetic_dev` 容器。默认打开 RViz；无图形环境可运行：

```bash
RAO_RVIZ=false RAO_DURATION=60 ./run_rao_2025_reproduction.sh
```

结果 CSV 默认写入容器内 `/tmp/rao_2025_reproduction.csv`，可通过 `RAO_CSV_PATH` 修改。该旧原型固定的是论文 4.1 节 `65 x 30 x 4 m/140` 障碍物场景；Table 1 来自 4.2 节 `50 x 30 x 5 m/155` 障碍物场景，因此这份 CSV 不能与 Table 1 直接对比。

完整算法参数在 `config/rao_2025_full.yaml`，ROS 话题统一位于 `/rao_2025`，因此不会与现有 `/object_odom` 或 planner 话题冲突。

旧 full harness 含 bicycle target、CV-EKF、beam search 和 quintic penalty 等近似，但它缺少真实 Hybrid A* 目标规划、论文凸多面体走廊、位置/yaw 两条 MINCO、分布式绝对时间轨迹交换及 Zhou baseline，并存在执行后安全投影。不得继续在该单节点上调权重来冒充论文算法。
