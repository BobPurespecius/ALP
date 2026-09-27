# Rao et al. 2025 FOV 论文复现方案

更新时间：2026-08-11

本文是后续实现的唯一工作基线。它把论文明确内容、项目复现约定、实现阶段和验收条件分开记录，避免把旧原型的近似实现继续堆叠成“完整复现”。

## 1. 目标与边界

目标是独立复现 Rao et al. 2025 的多 UAV 目标跟踪框架，并用 ROS/RViz 展示运行状态。入口必须独立于现有 door/stair、EGO/EGOv2 和 `/object_odom` 链路；允许复用 `px4_noetic_dev` 容器，但不能修改现有启动架构。

当前 `rao_2025_full_reproduction.py` 只保留为历史诊断原型。它的轴对齐 corridor、固定时间 quintic、集中式顺序规划、解析目标运动和执行后安全投影都不符合论文完整实现，后续不再在此节点上增加补丁。

## 2. 论文事实

论文明确给出的算法链路为：

1. 自主车辆使用 bicycle model，并使用 kinodynamic Hybrid A* 避障。
2. 每个 UAV 使用 constant-velocity EKF 预测目标位置、速度和 yaw。
3. 前端以等间隔预测时刻扩展加速度 motion primitives，启发式项使用 Pontryagin minimum principle。
4. 通过椭球膨胀和切平面构造凸多面体 safe flight corridor。
5. 遮挡 UAV 使用可见队友报告的目标全局坐标继续规划。
6. 后端使用 L-BFGS 优化统一 penalty objective。
7. 使用两条 MINCO 轨迹，分别表示平移和 yaw 变化。
8. yaw penalty 约束机体朝向与速度方向一致，yaw 还有最大角速度约束。
9. reciprocal avoidance 使用其他 UAV 的绝对时间轨迹作为约束输入。

### 论文场景

| profile | 论文位置 | 场景 | 障碍数 | 用途 |
|---|---|---|---:|---|
| `simulation` | 4.1 | `65 x 30 x 4 m` | 140 | Figure 5-7 方法演示 |
| `comparison` | 4.2 | `50 x 30 x 5 m` | 155 | Figure 8-10、Table 1 对比 |

共同公开参数：

- 初始 UAV：`[0.75, 1.4, 1]`、`[-1.5, 0, 1]`、`[0.75, -1.4, 1]`。
- FOV：`85° x 72°`。
- UAV 最大速度、加速度、jerk：`3 m/s`、`6 m/s^2`、`20 m/s^3`。
- 目标最大速度：`2.2 m/s`。
- 立方体宽度：`0.3-0.8 m`；高度：`2-3 m`。
- 环形障碍直径：`0.5-0.7 m`；相对 x 正方向倾角：`±0.5 rad`。

Table 1 参考值为：

```text
Ours:  3-vis=57.6%, 2-vis=32.2%, 1-vis=10.2%, 0-vis=0%
Zhou:  3-vis=10.8%, 2-vis=8.2%,  1-vis=67.2%, 0-vis=13.8%
```

只有 `comparison` profile 的结果允许与 Table 1 比较；`simulation` 结果只能用于 Figure 5-7 对应的开发诊断。

## 3. 公开参数与复现约定

论文没有公布以下内容：障碍物随机种子、立方体/环形障碍数量、环壁厚度、目标初态和路线、wheelbase、EKF 噪声、预测周期和 horizon、重规划频率、运行时长、编队向量、相机距离限制、目标函数权重。

这些值全部放在 `rao_2025_protocol.yaml` 的 `reproduction` 区域，并在 manifest 中标记为未公开参数。它们使本项目可确定性重跑，但不代表论文作者的私有设置。

当前复现约定：主种子 `2025`，按 profile 和功能流派生 SHA-256 稳定种子；控制/统计频率 `20 Hz`；预测间隔 `0.3 s`、8 个预测点；评估时长 `60 s`；环形障碍比例和厚度显式记录。

环形障碍采用“环面竖直、环轴位于 xy 平面、倾角绕 z 轴相对 +x”的三维解释。这与论文图示和 `±0.5 rad relative to +x` 一致，但论文没有给出完整几何定义，因此 manifest 将其明确登记为本地复现约定。

## 4. 简单重规划逻辑

每架 UAV 只有一个固定周期重规划循环：

```text
每 0.5 s 到期
  若上一次规划仍在运行：跳过本次 tick
  否则：用当前执行状态和同一时刻目标预测，完成一次规划
  成功：下一控制 tick 原子替换轨迹
  失败：继续旧轨迹，旧轨迹过期后 hold
```

目标观测更新不直接触发重规划，不允许同一 UAV 并行多个 planner，不做执行后轨迹平移或碰撞投影。碰撞监视器只能记录事实，不能修改状态来掩盖规划失败。

## 5. 目标架构

保持独立 ROS namespace `/rao_2025`，按职责拆分：

```text
rao_scene_node          场景、障碍物和固定随机种子
rao_target_node         bicycle 目标和 Hybrid A* 目标路线
rao_sensor_uav1..3      各自 FOV、LOS、观测噪声
rao_target_fusion       CV-EKF 与 FOV sharing
rao_planner_uav1..3     每架 UAV 独立前端、corridor、MINCO/L-BFGS
rao_simulator           执行轨迹和动力学状态
rao_visualizer          RViz marker、Path、FOV、LOS、corridor
rao_evaluator           统一指标和结果文件
rao_protocol            固定 manifest 和运行契约
```

所有节点读取同一个 manifest。planner 之间只交换带绝对时间戳的轨迹，不共享集中式内部状态。

## 6. 分阶段实现顺序

### 阶段 1：协议层（已完成）

- 固定论文公开参数和两个 profile。
- 固定随机种子派生规则、采样时钟和统计分母。
- 固定简单重规划契约。
- 发布 latched `/rao_2025/protocol/ready` 和 `/rao_2025/protocol/manifest`。
- 测试论文参数漂移会失败，Table 1 不会误用 4.1 场景。

入口：`run_rao_2025_stage1.sh`。

### 阶段 2：场景与目标（本次完成）

- 按 profile 生成精确尺寸场景。
- 实现立方体和有倾角、有孔洞厚度的环形障碍。
- 实现论文 8 状态 bicycle target。
- 实现目标 kinodynamic Hybrid A*，目标路线不能再由正弦函数代替。
- 固定目标真值话题和时间基准。

入口：`run_rao_2025_stage2.sh`。阶段 2 发布 `/rao_2025/scene/geometry`、`/rao_2025/scene/markers`、`/rao_2025/target/ground_truth`、`/rao_2025/target/path` 和 `/rao_2025/target/state`。目标 planner 初始规划当前段，并只预计算一个下一段结果；路段结束时原子切换。规划失败直接报错，不使用直线或真值 fallback。

实现约束：协议节点是唯一配置/manifest 来源；所有时间使用 ROS time；搜索、扫掠碰撞检查、终点预测和执行共用 20 Hz 固定步长 primitive 积分器。论文式 (46) 按 `dot(a_tgn)=v_tg^2*kappa_tg` 积分。位置进入容差时还需存在可进入新离散 key 的合法后继，避免把下一段置于搜索无法推进的状态。

### 阶段 3：感知与共享

- 每架 UAV 使用执行位姿计算水平/垂直 FOV 和 LOS。
- 只有真实可见 UAV 发布观测。
- CV-EKF 接收本机或队友的全局观测；共享不改变 visible 统计。
- 记录 bearing、FOV margin、LOS clearance、观测来源和轨迹年龄。

### 阶段 4：前端

- 按预测时间间隔生成三维加速度 motion primitives。
- 实现 Eq. (5)-(8) 的 Pontryagin heuristic。
- 每个预测时刻构造相对编队目标和可见区域 `W_n`。
- 实现点云/椭球膨胀/切平面形成 Eq. (9) 凸多面体 corridor。

### 阶段 5：后端

- 复用仓库已有 MINCO 和 LBFGS-Lite 结构，不再用固定时间 quintic 近似。
- 独立优化平移和 yaw 两条轨迹及时间分配。
- 按论文实现 `Lst/Lrp/Lds/Lag/Lyaw/Lfb/Loa/Lra`。
- 采样时使用绝对预测时间，动态约束、corridor、障碍和 reciprocal 约束不能由执行层补救。

### 阶段 6：运行与可视化

- 每架 UAV 独立规划并交换绝对时间轨迹。
- simulator 只执行规划结果并记录执行状态。
- RViz 显示目标真值/估计、三机 yaw、FOV 锥、LOS、预测点、前端路径、corridor、规划轨迹和执行轨迹。
- 同一时刻 RViz 状态与 CSV 指标必须来自同一执行采样。

### 阶段 7：对比与验收

- 使用同一 comparison 场景、目标和随机种子实现 Zhou baseline。
- 批量重复运行并报告均值、方差和失败次数。
- 不把单次启动成功或短时 smoke test 写成论文结果。

## 7. 验收指标

每次运行至少输出：

```text
3/2/1/0-vis 百分比
障碍物碰撞样本数、UAV 间碰撞样本数
EKF 误差、目标跟踪距离误差
编队角误差、FOV margin、LOS clearance
planner latency、trajectory age
optimizer success、各 penalty 分项
```

最低验收条件：无 0-visible（论文 Ours 目标）、无障碍物和机间碰撞、无执行层隐藏修正、规划延迟可控、统计分母完整，并能在 RViz 中对应到运行数据。

## 8. 当前验证与阻塞

已完成：12 项协议/模型单元测试；4.1/4.2 manifest 生成；两个 profile 的连续两轮目标闭环；容器内真实 ROS stage-2 定时运行；RViz 1.14.26/OpenGL 4.1 配置加载。执行采样结果为 simulation `300` 帧、`20.0010 Hz`、零障碍碰撞，comparison `400` 帧、`20.0067 Hz`、零障碍碰撞；两个 profile 的 60 秒运行均正常结束。

尚未完成：阶段 3-7 算法实现、UAV 动力学与感知闭环、完整指标 CSV、Zhou baseline 和多随机种子论文场景回归。因此阶段 1-2 通过不等于论文已完整复现，也不能拿当前 target-only 运行与 Table 1 比较。

当前容器 `px4_noetic_dev` 可运行 stage 1/2。运行：

```bash
RAO_PROFILE=simulation ./run_rao_2025_stage1.sh
RAO_PROFILE=comparison ./run_rao_2025_stage1.sh
RAO_PROFILE=simulation RAO_DURATION=60 RAO_RVIZ=true ./run_rao_2025_stage2.sh
RAO_PROFILE=comparison RAO_DURATION=60 RAO_RVIZ=true ./run_rao_2025_stage2.sh
```

所有后续结果必须附运行命令、profile、manifest 路径、CSV 路径和实际统计，不能只报告“节点启动”。
