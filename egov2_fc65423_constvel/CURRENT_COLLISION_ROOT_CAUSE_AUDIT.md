# ALP 当前碰撞根因源码级审计

审计日期：2026-09-02  
审计范围：仅 `/home/bob/ALP/egov2_fc65423_constvel` 及其 `ros_ws`。未进入 `/home/bob/RRCT`。  
约束：未修改代码、参数、场景；未运行新实验；结论只使用当前源码、当前 launch 参数、两轮指定运行的原始日志/CSV，以及轨迹 commit→publish→activate→command 生命周期。

## 0. 结论摘要

**IS_CURRENT_COLLISION_PRIMARILY_PLANNER_PROBLEM: YES**

最小可证实层级不是笼统的“QP 不稳定”，而是：

1. **candidate selection / final dynamic safety checking 缺少 hard gate**。SIDE 候选只需比 nominal 提升 `0.10 m`，不要求达到规划器自己的 `1.1 m` 动态中心距契约；NOMINAL 即使已预测到 `0.17–0.43 m` 也仍可提交。源码中存在 `checkMovingObjSafety()`，但当前工程没有任何调用点。运行参数又是 `enable_post_checks=false`，且现有 post-check 分支本身也不检查动态障碍物。
2. **safe trajectory persistence/lifecycle 的保护域不完整**。它只在“本周期已触发风险、最终仍选 NOMINAL”时保留 previous safe；一个“相对 nominal 更好但绝对仍低于 1.1 m”的 SIDE 会覆盖 previous safe，非风险分支也会直接用新 NOMINAL 覆盖安全 SIDE。Run2/C2 中，`2.039773 m` 的 SIDE_PLUS 激活后仅约 2 ms 就被 `1.837297 m` NOMINAL 覆盖；随后进入风险时已失去该安全轨迹。
3. **repair→MINCO 病态初始化真实存在，并使后端失败具有因果作用，但不是三次碰撞的统一根因**。Run1/C2 前的 A* repair 样本把 49 个 raw 点简化为 25 点并塞进约 `0.667 s` 局部时间窗，初始 `v=23.38 m/s, a=694.55 m/s², jerk=54710 m/s³`；统一 retiming 到 27.10 s 后，SCP 第 0 次线性化仍出现巨大梯度、OSQP max-iter/primal-infeasible，最终 `P_TRUST_TOO_SMALL`。这直接导致 Run1 临撞前没有新安全 replacement。Run2 两次碰撞则在更早已有安全候选，却被后续选择/生命周期覆盖，因此 solver failure 只属加剧因素。
4. **不是执行跟踪主因**。碰撞时 `traj_server` 发出的 command 本身已经进入动态圆柱：重算命令点到真实圆柱表面的净距约为 Run1/C2 `-0.012 m`、Run2/A1 `-0.009 m`、Run2/C2 `-0.069 m`。控制器跟踪误差不能解释“安全 command 被跟坏”；command 已不安全。
5. **未发现 2 s 预测窗口内的动态预测错位**。场景发布的 Path 直接由同一个真实正弦运动函数生成，使用绝对 ROS 时间；日志持续显示 `valid_predictions=10/10`。但规划器只检查前 `2.0 s`，而候选常为 3.8–6.2 s，超出窗口的尾段没有动态安全证明，这是明确的设计限制。

三案分类：

- Run1/C2：**TYPE A — NO_SAFE_CANDIDATE_GENERATED**（严格指碰撞前决定性周期：previous safe 已过期/失效，PLUS/MINUS 失败，NOMINAL 明知低于 1.1 m 仍提交）。
- Run2/A1：**TYPE B — SAFE_CANDIDATE_GENERATED_BUT_NOT_SELECTED**（安全 SIDE_MINUS 已生成、选择、激活，但后续选择规则用不安全 SIDE/NOMINAL 覆盖）。
- Run2/C2：**TYPE B — SAFE_CANDIDATE_GENERATED_BUT_NOT_SELECTED**（安全 SIDE_PLUS 已生成、选择、激活，随后非风险 NOMINAL 立即覆盖；此后低于 1.1 m 的 SIDE 仍被称为成功并提交）。

## 1. PROJECT MAP

### 1.1 实际源码区

当前 `ros_ws/src` 同时存在：

- 实际本轮 launcher 使用的 EGO-Planner-v2：`ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner`
- 另一套旧的 `ros_ws/src/ego-planner`：其中提供本轮仍被 launch 使用的 `so3_control`、`so3_quadrotor_simulator` 等仿真包，但其旧 planner 不是本轮 candidate/SCP 主规划器。
- `multi_uav_formation`：本轮场景、动态障碍、指标记录和总 launch。
- `target_tracking`：另有 Rao/FOV 复现节点；本轮 native launch 的 target state 由 `multi_uav_formation/scripts/target_state_coordinator.py` 协调，而非由此包的独立规划器产生 EGO candidate。

EGO-Planner-v2 实际相关包：

- `plan_manage`：FSM、planner manager、traj_server、launch。
- `path_searching`：本轮 SIDE local repair 使用的 dynamic A*。
- `traj_opt`：MINCO 接口、原 EGO 优化、SIDE hard-corridor SCP、OSQP adapter。
- `plan_env`：grid map、dynamic object predictor。
- `traj_utils`：多项式轨迹消息和轨迹基础类型。
- `swarm_bridge`：多机轨迹/里程计桥接。
- `drone_detect`：其他感知链；不是本轮 `/dynamic/prediction_i` 的来源。

### 1.2 A–K 责任节点

| 问题 | 当前源码确认 |
|---|---|
| A. 谁触发 replanning | `/drone_i_ego_planner_node` 中 `EGOReplanFSM::execFSMCallback()` 和 safety timer/FSM 状态机；10 ms timer 在 `ego_replan_fsm.cpp:90,140`，最终通过 `callReboundReplan()`，见 `ego_replan_fsm.cpp:606-618`。 |
| B. 谁生成 candidate trajectory | 同一节点内 `EGOPlannerManager::reboundReplan()`，入口 `planner_manager.cpp:1141`；nominal、SIDE_PLUS、SIDE_MINUS 均在这里组织，优化调用在 `planner_manager.cpp:3442-3445`。 |
| C. MINCO 在哪里 | `traj_opt/include/optimizer/poly_traj_utils.hpp` 的 `MinJerkOpt`；manager 在 `computeInitState()` 中 `reset/generate`（`planner_manager.cpp:233-404`），repair 后再次 `generate`（`2555-2636,2865-2888`）。 |
| D. SCP/QP 在哪里 | `PolyTrajOptimizer::runCandidateHardCorridorSCP()`，`poly_traj_optimizer.cpp:194+`；QP 调用 `SCPOptimizer::solve()`，`poly_traj_optimizer.cpp:1759`；OSQP adapter 在 `scp_optimizer.cpp:51-200`。 |
| E. SIDE_PLUS/MINUS | `computeInitState()` 中 `side_dir=(target-start).cross(UnitZ())`，`planner_manager.cpp:311-326,381-395`。当前定义 `side>0` 为 PLUS/physical RIGHT；偏好只决定顺序，两边均求解，`planner_manager.cpp:3728-3740`。 |
| F. A* repair | `planner_manager.cpp:1931-2925` 调 `path_searching::AStar::AstarSearch()`；A* 实现在 `path_searching/src/dyn_a_star.cpp:91-245`。 |
| G. trajectory 发布 | FSM 构造 `traj_utils/PolyTraj` 并发布，`ego_replan_fsm.cpp:646-654`。 |
| H. traj_server 转 position command | `traj_server.cpp:47-89` 接收/激活多项式；`223-283` 用 `(ROS now-start_time)` 求 pos/vel/acc/jerk；`192-218` 发布 `PositionCommand`。 |
| I. controller/simulator | `simulator.xml:71-99` 启动 SO3 simulator 和 SO3 control。`so3_control_nodelet.cpp` 消费 pos_cmd/odom，`SO3Control.cpp:70-73` 使用反馈+前馈；`quadrotor_simulator_so3.cpp` 积分动力学并发布 visual_slam odom。fake poscmd launch 分支被注释。 |
| J. dynamic actual/predicted | actual 和 prediction 都来自 `native_egov2_rviz_scene.py::moving_obstacle_state()`（`108-126`）。actual pose 在 `317-405` 发布；future Path 在 `406-423` 用同一函数生成。`ObjPredictor` 订阅 `/dynamic/pose_i` 与 `/dynamic/prediction_i`（`obj_predictor.cpp:147-190`），绝对时间插值（`17-100`）。 |
| K. collision/metrics | scene node 用 odom 和真实障碍状态写 `_trajectory.csv`，`native_egov2_rviz_scene.py:627-703`；离线 episode/unsafe 统计见 `analyze_speed50_repeatability.py:16-44,69-84`。 |

### 1.3 经源码验证的运行数据流

```text
scene moving_obstacle_state(t = ROS_now - motion_start)
  ├─ /dynamic/pose_i              (actual pose)
  └─ /dynamic/prediction_i Path   (absolute ROS timestamps, same sinusoid)
                 ↓
ObjPredictor absolute-time interpolation
                 ↓
EGOReplanFSM risk/safety trigger
                 ↓
EGOPlannerManager nominal + SIDE_PLUS/RIGHT + SIDE_MINUS/LEFT
                 ↓
SIDE static backoff → local A* repair/rejoin → semantic-v2
                 ↓
MINCO P/T seed → hard-corridor SCP → OSQP
                 ↓
relative-clearance candidate selection / incomplete persistence
                 ↓
setLocalTrajFromOpt COMMITTED
                 ↓
PolyTraj PUBLISHED
                 ↓
traj_server ACTIVATED → PositionCommand
                 ↓
SO3 controller → SO3Command → quadrotor simulator
                 ↓
/drone_i_visual_slam/odom
                 ↓
scene metric: odom point vs actual moving cylinder
```

## 2. CURRENT SOURCE STATE

### 2.1 Git 状态

- 分支：`main...origin/main [ahead 2]`。
- HEAD：`8cef241 Finalize EGOv2 sinusoidal gradient avoidance`；其前一提交 `fc65423 Fix EGOv2 sinusoidal moving obstacle prediction`；远端基线 `5c4ea81`。
- workspace 非干净。与本审计核心直接相关的未提交修改包括：
  - `plan_manage/include/plan_manage/planner_manager.h`
  - `plan_manage/src/planner_manager.cpp`（相对索引约 `+3312/-11`）
  - `traj_opt/include/optimizer/poly_traj_optimizer.h`
  - `traj_opt/src/poly_traj_optimizer.cpp`（约 `+3207/-80`）
  - `advanced_param.xml`、`run_in_sim.launch`、CMake/package 文件
  - `multi_uav_formation/launch/native_egov2_rviz.launch`
  - `multi_uav_formation/scripts/native_egov2_rviz_scene.py`
- `path_searching` 在此 Git 视图中整体显示为 untracked，因此不能用 Git diff 单独证明 A* 修复历史；本报告只陈述当前文件内容。

### 2.2 已描述修改逐项核验

| 描述 | 判定 | 当前源码证据 |
|---|---|---|
| preferred_side 仅决定顺序，PLUS/MINUS 都求解 | **CONFIRMED_FROM_SOURCE** | `planner_manager.cpp:3728-3740`。 |
| A* start/end index bug 修复 | **CONFIRMED_FROM_SOURCE** | start 调整检查 `start_idx`，end 调整检查 `end_idx`，`dyn_a_star.cpp:91-141`。 |
| 首个 SUCCESS 不再无条件阻止 downstream rejoin | **CONFIRMED_FROM_SOURCE** | 保存 first success，但只有 `raw_path_usable()` 才 break；最多三次 rejoin，`planner_manager.cpp:2159-2281`。 |
| semantic-v2 替换 whole-path strict half-plane | **CONFIRMED_FROM_SOURCE** | 唯一 checker 为 `side_semantic_valid`，`2055-2157`；raw、repaired/native repaired 都调用同一 checker（`2165-2174,2755-2759`）。未发现另一个实际 hard semantic gate。 |
| PLUS=RIGHT，MINUS=LEFT | **CONFIRMED_FROM_SOURCE** | normal/tangent 使用 `tangent.cross(UnitZ())`，`planner_manager.cpp:1464-1489,2092-2104`；`side>0` 以正 lateral 判定，`2111-2140`。 |
| semantic_eps=0.05 | **CONFIRMED_FROM_SOURCE** | `planner_manager.cpp:2055`。 |
| 权重 `1/(d+0.1)` 且 cap 10 | **CONFIRMED_FROM_SOURCE** | `planner_manager.cpp:2113-2125`。 |
| dominant_eps=0.05、correct ratio>=0.60 | **CONFIRMED_FROM_SOURCE** | `planner_manager.cpp:2056-2057,2135-2141`。 |
| semantic 数据不足 non-blocking fallback | **CONFIRMED_FROM_SOURCE** | 多个不足分支返回 true，`2061-2071,2083-2101,2127-2134`。 |
| safe trajectory persistence | **CONFIRMED_FROM_SOURCE，但保护不完整** | validator `606-688`；只在 triggered 分支调用 `1432-1434`，且只有 selected=NOMINAL 时 retain，`3877-3895`。 |
| cross-cycle warm start | **CONFIRMED_FROM_SOURCE** | `buildWarmStartFromAccepted()` 附近 `planner_manager.cpp:500-603`；只缓存实际 committed 轨迹 `4066-4076`。运行日志中也有 `trajectory-warm-start`/`accepted-state-cache`。 |

### 2.3 旧逻辑残留审计

- 未发现旧的 whole-path strict semantic hard gate 与 semantic-v2 并存。
- 发现比 semantic 残留更关键的 contract split：`checkMovingObjSafety()` 定义在 `poly_traj_optimizer.cpp:5274-5311`，全工程 `rg` 仅找到声明和定义，没有调用点。
- `SCP_FINAL_OK` 的 final log 只重新计算 dynamic risk 作为诊断；`result.success` 不会因低于 1.1 m 被清零，见 `planner_manager.cpp:3442-3497`。

## 3. RUNTIME DATAFLOW / CURRENT CONFIG

两轮 launcher 都由 `multi_uav_formation/launch/native_egov2_rviz.launch` 启动三架 EGO planner、traj_server、SO3 控制/仿真、local sensing、scene 和 target coordinator。

Run1 launcher 参数（Run2 同配置）关键值：

- `manager/max_vel=3.0`：launcher line 116
- `manager/max_acc=6.0`：line 114
- `manager/max_jer=22.0`：line 115
- `optimization/moving_obj_clearance=1.1`：line 151
- `prediction/moving_obj_prediction_horizon=2.0`：line 169
- `manager/risk_sample_dt=0.1`：line 119
- `manager/candidate_min_clearance_improvement=0.1`：line 104
- `manager/enable_candidate_acceptance=False`：line 108
- `manager/enable_post_checks=False`：line 110

对应 launch 默认/传递位置：`advanced_param.xml:75,81,83,103,105-106,233-247,276,295-302`；`run_in_sim.launch:19-24,42-46,57-101`。

## 4. COLLISION METRIC DEFINITION

### 4.1 Offline/scene collision 公式

源码：`native_egov2_rviz_scene.py:627-641`。

对 UAV odom 中心点 `p=(x,y,z)` 与圆柱 `(cx,cy,r,zmin,zmax)`：

```text
radial = sqrt((x-cx)^2 + (y-cy)^2) - r_obstacle
vertical = zmin-z (z<zmin), z-zmax (z>zmax), else 0

if radial <= 0 and vertical > 0: clearance = vertical
if vertical <= 0 and radial > 0: clearance = radial
else: clearance = sqrt(max(0,radial)^2 + vertical^2)
```

本案 UAV 均位于圆柱高度范围内，因此简化为：

```text
c_offline = max(0, horizontal_center_distance - r_obstacle)
```

结论：

- 使用 **UAV center 到 obstacle surface**，不是 surface-to-surface。
- obstacle radius `0.28 m` 被减去。
- UAV radius **没有计入**。
- penetration 被 clamp 为 0，CSV 无法显示负穿透深度。
- collision 条件是 `clearance <= 0`，即 UAV center 进入/接触物理圆柱。
- unsafe 条件是 `clearance < 0.5 m`，见 `analyze_speed50_repeatability.py:79`；它是近障样本，不等同 collision。

### 4.2 Planner dynamic distance

源码：`planner_manager.cpp:691-747`。

```text
d_planner(t) = || p_traj(t) - p_obstacle(prediction_epoch + t) ||_2
trigger_distance = moving_obj_clearance(1.1) + margin(0.15)
trigger iff min d_planner < 1.25
```

- 是 3D center-to-center 距离。
- 没有显式减 obstacle radius，也没有 UAV radius。
- 只采样 `min(trajectory prefix, prediction_horizon=2.0 s)`，步长 `0.1 s`。
- optimizer moving cost 同样使用中心距 1.1 m hinge 和绝对查询时间 `t_now_+t`，`poly_traj_optimizer.cpp:4173-4238`。
- 因此 CSV 的 `0` 不能解释为 planner 的 `d_planner=0`。圆柱半径 0.28 m 时，物理碰撞通常对应 planner center distance 约 `<=0.28 m`。

### 4.3 时间基准

- scene actual：`ROS now - motion_start`，`native_egov2_rviz_scene.py:321-324,649-653,680-681`。
- scene prediction：Path 每个 pose 的 stamp 为 `ROS stamp + tau`，`406-423`。
- predictor：保留 pose 绝对 stamp，按给定绝对时间线性插值，`obj_predictor.cpp:17-100`。
- planner：`prediction_epoch + trajectory_relative_time`，`planner_manager.cpp:708-735`。
- traj_server：`ROS now - message.start_time`，`traj_server.cpp:234-255`。
- CSV `time_s`：`ROS now - tracking_start`，`native_egov2_rviz_scene.py:659-665,693-704`。

## 5. RUN1 C2 AUTOPSY

数据源：

- `semantic_v2_repeatability_20260902/alp_repeat_1/alp_repeat.launcher.log`
- `semantic_v2_repeatability_20260902/alp_repeat_1/alp_repeat_trajectory.csv`
- visibility CSV/summary 只用于 FOV，不提供 planner provenance。
- 此目录没有 rosbag，也没有分离的 planner/semantic/QP log；所有 ROS 节点日志均复用 launcher log。

### 5.1 Episode 边界

- UAV：CSV `uav_id=1`，planner `drone_id=0`
- obstacle：id 3，`dynamic_gate_C2`
- CSV collision start：`62.363205 s`
- CSV collision end：`62.393796 s`
- CSV 因 clamp 两点均为 0；用同一 scene 正弦模型恢复未截断 radial 后，最深时刻为约 `62.393796 s`，穿透约 `-0.0298 m`。
- 对齐绝对 ROS 时间约：`1788322562.033759` 至 `1788322562.064350`。

### 5.2 T-4 s 生命周期

关键周期（launcher 行号）：

| ROS time | cycle/trajectory | 结果 |
|---|---|---|
| 2558.0916 | traj 210 | NOMINAL，预测中心距 `1.453161`，selected/committed/published。launcher `31314`。 |
| 2558.8925 | traj 211 | NOMINAL，预测 `1.651513`，duration `2.062421`；selected `31955`，traj_server 激活 `32431`。 |
| 2560.004–0.632 | risk cycle 52 | previous traj211 被 validator 判 `1.9973 m` 且 retain；PLUS 的 A*/MINCO/SCP 失败，MINUS 无静态可行 SIDE。`32970,32984-32988`。 |
| 2560.955 左右 | traj211 | 按 start+duration 已到期；不能覆盖 2562.034 的 collision。 |
| 2561.0239 | traj212 | NOMINAL，预测 `0.404789`，仍 selected；launcher `33343`，traj_server receive `33472`。 |
| 2561.2296 | traj213 | NOMINAL，预测 `0.428120`；`33416,33476`。 |
| 2561.4762 | traj214 | NOMINAL，预测 `0.408651`；`33519,33790`。 |
| 2561.7306 | traj215 | NOMINAL，预测 `0.423935`；`33611,33794`。 |
| 2562.0138 | cycle 62 | traj215 被 validator 明确判 `DYNAMIC clearance=0.430551 safety=1.1`；PLUS/MINUS 都失败。`33683,33686,33693-33694`。 |
| 2562.0143 | traj216 | 仍选择 NOMINAL，预测 `0.429684`，commit/publish，`33695-33702`；traj_server receive/activate `33799`。碰撞在其激活约 19 ms 后开始。 |

### 5.3 Candidate provenance

- NOMINAL：成功生成，但 planner 自己预测低于 1.1 m，绝对不安全。
- SIDE_PLUS：在 2560.023 的代表性 repair 中，A* `raw=49, simplified=25, path=5.848 m`，semantic valid、被 accept；MINCO 初值极端，retime 后 SCP 在 iteration 0 无法得到步长，最终 `P_TRUST_TOO_SMALL`。
- SIDE_MINUS：多周期为 `NO_STATIC_FEASIBLE_SIDE`，或 dynamics trust retry failure。
- previous safe：traj211 曾有效并被多次 retain/republish，但在 collision 约 1.08 s 前到期；后续 traj212–216 均低于 1.1 m。
- selected→committed→published→activated：traj216 四阶段均有日志；不存在 TYPE C 激活丢失。

### 5.4 Command/odom

碰撞附近 command：

- `1788322561.965096`，traj215，重算 command surface clearance `+0.1558 m`。
- `1788322562.074711`，traj216，重算 command surface clearance `-0.0120 m`。

故 command 本身穿柱；不是安全轨迹因 odom tracking error 才撞。

### 5.5 分类

**RUN1_C2_CLASS: TYPE_A**

CLAIM：碰撞前决定性周期没有可继续使用的安全 candidate。  
EVIDENCE：traj211 已到期；cycle 62 明确把当前 traj215 判低于 1.1；PLUS/MINUS 失败；新 NOMINAL 只有 0.429684 并被激活。  
COUNTER-EVIDENCE：T-3.14 s 曾生成并激活 traj211，且当时满足 1.1，但 duration 不覆盖 collision。  
CONFIDENCE：高。

## 6. RUN2 A1 AUTOPSY

### 6.1 Episode 边界

- UAV1 / planner drone0 / obstacle0 `dynamic_gate_A1`
- CSV start `18.731012 s`
- CSV end `18.859149 s`
- 恢复未截断 clearance 的最深时刻约 `18.793146 s`，穿透约 `-0.0171 m`
- 绝对 ROS 时间约 `1788322608.548195–1788322608.676332`

### 6.2 轨迹生命周期

| ROS time | trajectory | 结果 |
|---|---|---|
| 2605.73436 | SIDE_MINUS candidate | `SCP_FINAL_OK`，final center distance `1.343666`，duration `6.212057`；launcher `6623`。 |
| 2605.73445/2605.73505 | traj55 | selected `6635`，traj_server activated `6885`。若不被覆盖，其 duration 可覆盖 collision。 |
| 2606.04268 | SIDE_MINUS | `SCP_FINAL_OK`，但只有 `0.938002 < 1.1`；仍被 selected 为 traj56，`6821,6833`，activated `6890`。 |
| 2606.32338 | SIDE_MINUS | `SCP_FINAL_OK`，`0.627528`；相对 nominal `0.564746` 仅提升约 `0.063 < 0.10`，最终 selected NOMINAL traj57，`7022,7033`，activated `7865`。 |
| 2606.60818 | SIDE_MINUS | `SCP_FINAL_OK`，`0.580846`，仍不满足 1.1；之后 SIDE 多次失败。 |
| 2608.44082 | traj65 | NOMINAL，预测 `0.406031`，selected/activated，`8354,9296`。 |
| 2608.548–.676 | collision | traj65 command 正在执行。 |
| 2608.71428 | traj66 | collision 后新 NOMINAL，预测 `0.431952`，`8420,9301`。 |

### 6.3 决定性证据

- 安全 traj55 不只是 solver 中间结果：它经过 selected、commit、publish、traj_server receive/activate。
- 它不是因为服务器未切换而丢失；它被后续 traj56、traj57 正常 supersede。
- 后续 SIDE `SCP_FINAL_OK` 低于 1.1，证明 `SCP_FINAL_OK` 不是动态安全证明。
- selection gate 只看 `candidate >= nominal + 0.10`，源码 `planner_manager.cpp:3718-3722`；没有 `candidate>=1.1`。
- persistence 不能保护一个已被“不安全 SIDE”覆盖的安全 trajectory；进入 traj57 周期时 safe cache 已变成后来的 committed 状态。

### 6.4 Command/odom

traj65 command 重算：

- `2608.460700` surface `+0.0445 m`
- `2608.560846` surface `-0.0091 m`
- `2608.671009` surface `+0.0065 m`

CSV odom 与同时间邻近 command 的位置误差约 `0.014–0.037 m`，command 自身已碰撞。

### 6.5 分类

**RUN2_A1_CLASS: TYPE_B**

CLAIM：安全 candidate 已生成并激活，但后续选择/持久化没有保留它。  
EVIDENCE：traj55 `1.343666>1.1`、duration 覆盖 collision；后续被 `<1.1` SIDE/NOMINAL supersede。  
COUNTER-EVIDENCE：碰撞瞬间 active traj65 不是 traj55；因此若只观察最后一个 cycle 会误判为 TYPE A。完整 provenance 表明安全解先存在后丢失。  
CONFIDENCE：高。

## 7. RUN2 C2 AUTOPSY

### 7.1 Episode 边界

- UAV2 / planner drone1 / obstacle3 `dynamic_gate_C2`
- CSV start `60.592346 s`
- CSV end `60.859129 s`
- 恢复未截断 clearance 的最深时刻约 `60.725580 s`，穿透约 `-0.0997 m`
- 绝对 ROS 时间约 `1788322650.414473–1788322650.681256`

### 7.2 轨迹生命周期

| ROS time | trajectory | 结果 |
|---|---|---|
| 2647.35459 | SIDE_PLUS candidate | `SCP_FINAL_OK`，center distance `2.039773`，duration `5.734941`；launcher `33859`。 |
| 2647.47588/2647.47674 | traj204 | selected `34002`；traj_server activated `34747`。duration 可覆盖 collision。 |
| 2647.47782/2647.47880 | traj205 | 约 2 ms 后非风险 NOMINAL `1.837297` 被 selected/activated，`34025,34749`，立即覆盖 traj204。此分支不调用 previous-safe validator。 |
| 2650.11323 | SIDE_PLUS | `SCP_FINAL_OK`，但 dynamic clearance 仅 `0.274587`；仍因相对 nominal 有提升而 selected 为 traj212，`36251,36266`，activated `36331`。 |
| 2650.38723 | traj213 | PLUS/MINUS 失败，previous 已不安全，NOMINAL `0.253240` selected；`36372`，activated `36727`。 |
| 2650.41447 | collision start | traj213 已激活约 26 ms。 |
| 2650.65094 | traj214 | collision 中又 selected NOMINAL `0.274536`，`36529`，activated `36731`。 |

### 7.3 生命周期诊断中的 stale metadata

`last_candidate_kind_` 在 non-triggered 分支只有连续 3 次后才 reset 为 NOMINAL，`planner_manager.cpp:1413-1426`，却不会在每次 NOMINAL commit 时立即更新；实际 active kind 另存于 `active_candidate_kind_`。因此 risk log 的 `previous=SIDE_PLUS` 可滞后于真实 active traj205=NOMINAL。该问题主要污染 hysteresis/诊断语义，不是 command 激活错误；traj_server 日志证明真实 traj205 已切换。

### 7.4 Command/odom

- `2650.353478` traj212 command surface `+0.1172 m`
- `2650.453722` traj213 `-0.0090 m`
- `2650.553893` traj213 `-0.0694 m`
- `2650.664257` traj214 `-0.0196 m`

碰撞窗 odom-command 误差约 `0.035–0.123 m`，但 command 穿透最高约 `0.069 m`，且 planner 自己的预测中心距已仅 `0.253–0.275 m`；执行误差不是首要因果层。

### 7.5 分类

**RUN2_C2_CLASS: TYPE_B**

CLAIM：安全 traj204 已生成、选择、激活，却被非风险 NOMINAL 立即覆盖；后续 final/selection 又允许绝对不安全 candidate。  
EVIDENCE：`2.039773 m` SIDE_PLUS → activate；2 ms 后 `1.837297 m` NOMINAL → activate；临撞前 SIDE `0.274587 m` 仍 `SCP_FINAL_OK`/selected。  
COUNTER-EVIDENCE：traj205 的 `1.837297 m` 在其生成时也高于 1.1；但它没有持续到 collision，且覆盖动作使 safe persistence 对 traj204 失去保护。  
CONFIDENCE：高。

## 8. SUCCESS vs FAILURE CANDIDATE COMPARISON

同运行 A/B 代表样本：

| 样本 | frontend / P,T | initial dynamics | first QP/SCP | final |
|---|---|---|---|---|
| Run2 A1 SUCCESS SIDE_MINUS，2605.732–.734 | DIRECT_ACCEPT；3 pieces，init duration `5.448230`；path `5.1045 m`；无 A*；warm-start 日志为 `NO_ACCEPTED_STATE` | `v=1.539, a=0.954, j=2.370`，无需 retime | 数值正常并收敛 | `SCP_FINAL_OK`, duration `6.212057`, `v=1.143,a=.859,j=2.408`, dynamic `1.343666` |
| Run2 A1 SUCCESS但不安全，2606.040–.043 | DIRECT_ACCEPT；duration `5.452791`; path `5.2937 m` | `v=1.602,a=.985,j=2.506` | 收敛 | `SCP_FINAL_OK`, dynamic `0.938002<1.1` |
| Run1 C2 FAIL SIDE_PLUS，2560.023–.632 | A* raw49→simplified25，local interval `0.250–0.917 s`，path `5.848 m`；rebuilt `piece_num=28`；原 duration `2.0 s` | `v=23.376,a=694.553,j=54710.36`；retime scale `13.548`→`27.096 s`；24 个中间 piece 仅 `0.376339 s`，尾部 `6.774 s` | iteration0 gradient-T norm `359284`；OSQP max_iter 后 primal_infeasible；probe 显示去掉 P trust 可解，反复扩大 P trust仍无 production step | `P_TRUST_TOO_SMALL`, final no step, jerk `25.644>22`, static_violation `2.5966` |
| Run1 C2 FAIL SIDE_MINUS，2560.265–.276（另一机同障碍的对照） | DIRECT_ACCEPT，原 duration `2.0 s` | `v=4.696,a=7.903,j=94.978`; retime `1.628`→`3.2566 s` | dynamics trust retry exhausted | final trajectory仍接近限值 `v=3.033,j=22.234`，`DYNAMICS_TRUST_RETRY_EXHAUSTED` |

共同点/差异：

- 成功例的空间几何和 duration 相容，初始 dynamics 已接近正常物理范围。
- 病态失败例不是单一 OSQP 偶发错误；进入 QP 前 P/T 已产生非常高阶导数和极不均匀 piece partition。
- uniform retiming 把速度/加速度/jerk 缩回约束附近，但不会改变 piece 数、空间折线密度和大量局部短段，也不会自动解除 Local-SFC/static/side/dynamics 线性化之间的冲突。
- 不是所有 failure 都由高 jerk 解释；一些 direct SIDE 也会 trust-exhaust。但最严重的 A* repair 样本有清晰的 repair→MINCO conditioning 因果链。

## 9. REPAIR→MINCO INITIALIZATION AUDIT

### 9.1 当前实现

repair 不是简单沿用一个旧 `T` 向量：

- 保留 collision interval 外的旧 junction。
- A* simplified 内点在 `[collision_start, collision_end]` **按点索引等比例分配时间**，`planner_manager.cpp:2555-2603`。
- 新 `piece_num=guide_points-1`，新 duration 为相邻 guide time 差，`2605-2636`。
- head/tail pos/vel/acc 来自原 `astar_base_traj` 两端，`2627-2635`。
- topology/P/T 数量在 SCP 入口有明确维度检查，`poly_traj_optimizer.cpp:235-276`。
- repair 后统一 retiming 使用 `max(v/vmax,sqrt(a/amax),cbrt(j/jmax))`，`planner_manager.cpp:2962-3049`。

因此“piece_num 改变后 T 没同步” **未证实**；当前源码会同步数量。

### 9.2 已确认的病态机制

等“点索引”分时而非按弧长分时，把大量 A* 转折/回插点塞入短 collision window。Run1 代表例：

```text
local window: 0.6667 s
simplified path points: 25
local path length: 5.848 m
average local piece time ≈ 0.0278 s before later reconstruction effects
MINCO initial: v 23.38, a 694.55, jerk 54710
```

这满足用户提出的可疑模式，且数值比先前线索更严重。源码层原因定位到 `planner_manager.cpp:2588-2595` 的 index-proportional time assignment，而不是 A* search error 本身。

### 9.3 判断

**PATH_TO_MINCO_INITIALIZATION_PATHOLOGICAL: YES**

但其对三案的 collision 因果强度为 **PARTIAL**：Run1/C2 是缺少 replacement 的直接来源之一；Run2/A1、C2 的首要证据是已有安全 candidate 被覆盖。

## 10. SCP/QP/TRUST AUDIT

- OSQP 配置：scaling=10、adaptive rho、eps=1e-3、max_iter=2000，`scp_optimizer.cpp:146-195`。
- 失败例 iteration0：gradient 极大、约束 661 rows；OSQP 从 max_iter 到 primal-infeasible-inaccurate。
- probe `NO_P_TRUST status=solved` 说明不是直接证明“真实约束无解”；更准确是当前 trust-constrained linearized QP 与病态 seed/多约束组合不相容。
- trust loop扩大 P radius后仍无 production step，最终状态名 `P_TRUST_TOO_SMALL` 与实际动作命名有一定语义混乱；不能把字符串当根因。
- `candidate.success` 在每次 `CandidateResult` 新对象/默认值上使用，且 final 由 optimize 返回值重写；未发现 OSQP failure 后 success 残留 true 的证据。
- trust retry 中 `current_points/current_durations` 只有接受 step 才成为新基点；失败例 `max_abs_delta_P=0,max_abs_delta_T=0`，没有观察到失败 step 污染后续基点。

**SOLVER_FAILURE_IS_CAUSALLY_LINKED_TO_COLLISION: PARTIAL**

- Run1/C2：YES，临撞前 PLUS/MINUS failure 使 replacement 空缺。
- Run2/A1：PARTIAL，后半段 solver failure 加剧恢复失败，但安全 traj55 先被选择规则丢弃。
- Run2/C2：PARTIAL/次要，最清楚的首因是 traj204 被非风险 NOMINAL 立即覆盖和低于 1.1 的 candidate 可通过 final/selection。

## 11. TRAJECTORY SELECTION/LIFECYCLE AUDIT

### 11.1 已确认正确

- PLUS/MINUS result 为独立 `CandidateResult`，每个 cycle 局部新建并分别传入 `optimize_side`；未发现 result 结构互相覆盖。
- 两侧都尝试，preferred 只控制顺序。
- final dynamic risk 在 SCP 后重新计算并写入 candidate，`planner_manager.cpp:3471-3497,3510+`。
- 只缓存真正 committed 的轨迹，failed/unselected candidate 不覆盖 accepted cache，`4066-4076`。
- commit→publish→traj_server receive/activate 在三案均连续可见；没有 TYPE C。

### 11.2 已确认问题

1. **相对 improvement 代替绝对安全**：`candidate_is_good` 仅要求 `candidate >= nominal+0.1`，`3718-3722`。
2. **NOMINAL 无动态 hard rejection**：即使 `0.17–0.43 m` 仍 commit。
3. **final checker contract 不闭合**：`checkMovingObjSafety` 无调用；post-check 无 dynamic 项，且 runtime false。
4. **persistence 分支过窄**：只在 triggered+selected NOMINAL；不保护 non-triggered overwrite，也不阻止 `<1.1` SIDE overwrite。
5. **日志/切换历史 stale**：`last_candidate_kind_` 不是每次 NOMINAL commit 都更新，可能影响 hysteresis 的 previous kind 和日志；真实 active metadata 是另一变量。
6. **超 horizon 尾段无证明**：2 s 后 candidate 尾段不检查。

### 11.3 generation/start_time/traj_server

- planner 每次成功 commit 生成新的 start_time/generation/traj_id，并记录 SUPERSEDED/ACCEPTED。
- traj_server 不做 generation monotonic rejection；它无条件接受合法 PolyTraj message 并替换当前轨迹，`traj_server.cpp:47-89`。但本两轮 planner 自身消息的 receive 顺序与 publish 一致，未发现旧消息把新轨迹回滚。
- “out of order or duplicated” 日志来自 swarm trajectory 接收/bridge（`ego_replan_fsm.cpp:1744` / `traj2odom_node.cpp:48`），不是 traj_server 拒绝本机 command trajectory。

## 12. DYNAMIC PREDICTION AUDIT

### 12.1 预测来源与一致性

- actual 和 future prediction 共用 `moving_obstacle_state()`，`native_egov2_rviz_scene.py:108-126,317-423`。
- prediction Path 使用 absolute stamp。
- predictor 对 timestamp 单调过滤、线性插值，超出 Path 后 clamp 末端，`obj_predictor.cpp:17-100`。
- 两轮日志周期性显示 `pose_histories=0/10, valid_predictions=10/10`，说明使用 Path，而非 constant-velocity fallback。
- fallback 仅在无 Path 时由 pose history 建 10 s 线性外推，`obj_predictor.cpp:259-318`；本案未走。

### 12.2 限制/潜在问题

- Path horizon 2 s，planner 风险、moving cost、unused hard checker都截断到 2 s。
- 候选 duration 大于 2 s；超出 horizon 的 `evaluateConstVel` 虽会 clamp endpoint，但 planner 主风险循环根本不查询该尾段。
- 如果某个 obstacle prediction 缺失，risk 会跳过该 obstacle；若全部缺失 risk invalid，当前上层按 non-blocking NOMINAL 处理。这是 fail-open 设计限制，但本两轮 `10/10`，不是三案证据。
- predictor callback 和 planner timer 可能并发访问 prediction vector；源码未见锁。ROS spinner 线程模型需结合 node spinner 才能证明 race，本轮没有异常/NaN 日志，分类为 LIKELY_RISK/NOT_ENOUGH_EVIDENCE，而非已证 bug。

**DYNAMIC_PREDICTION_MISMATCH_FOUND: NO（在当前 2 s 窗口及现有工件中）**  
**DYNAMIC_PREDICTION_IS_PRIMARY_CAUSE: NO**

## 13. COMMAND/ODOM EXECUTION AUDIT

### 13.1 执行链

`PolyTraj` → traj_server 按 ROS time 求导 → `PositionCommand` → SO3 controller → simulator → visual_slam odom。三案 collision 窗均有 continuous traj-server command，未见 heartbeat loss/hold。

### 13.2 命令轨迹重算

用 launcher 中 `traj-server-command` position、scene 同一真实正弦函数和障碍 radius 0.28 重算：

| case | active command traj | 最小观测 command surface clearance |
|---|---:|---:|
| Run1/C2 | 216 | `-0.0120 m` |
| Run2/A1 | 65 | `-0.0091 m` |
| Run2/C2 | 213/214 | `-0.0694 m` |

Run2/C2 tracking error相对较大，但 command 已经穿柱，且 planner predicted center distance 只约 0.25 m。因此：

**EXECUTION_TRACKING_IS_PRIMARY_CAUSE: NO**

执行误差可能改变穿透深度/episode 样本数，但不是碰撞存在与否的首因。

## 14. LATENT CODE ISSUES

按用户列出的 30 项逐项分类：

| # | 项目 | 分类 | 证据/说明 |
|---:|---|---|---|
| 1 | PLUS/MINUS state 独立 | CONFIRMED_OK | 局部独立 CandidateResult；两边分别 optimize。 |
| 2 | candidate result 每轮 reset | CONFIRMED_OK | `reboundReplan` 局部对象，未见 static 复用。 |
| 3 | obstacle_id stale | LIKELY_BUG | `last_candidate_obstacle_id_` 与 actual active metadata 分离；non-trigger NOMINAL 不立即同步。 |
| 4 | conflict_time stale | CONFIRMED_OK | 每次 `evaluateDynamicRisk` 重算；retime 后有 risk-sync。 |
| 5 | semantic fallback 过多 | DESIGN_LIMITATION | 多个数据不足分支 fail-open；日志中确有 fallback。 |
| 6 | fallback 接受 opposite topology | LIKELY_BUG | 数据不足直接 true，理论上可接受明显 opposite；本三案未证明它造成碰撞。 |
| 7 | SCP 后重算 final risk | CONFIRMED_OK | `3449-3497`。但只诊断，不 hard reject。 |
| 8 | nominal/SIDE risk 时间基准一致 | CONFIRMED_OK | 都用同 cycle `planning_prediction_epoch`。 |
| 9 | prediction absolute time | CONFIRMED_OK | scene stamps、predictor storage、planner query 对齐。 |
| 10 | A* reset/pool lifecycle | CONFIRMED_OK | 每 search `rounds_++` 且清 openSet；round tag隔离旧 node state，`dyn_a_star.cpp:144-180,225-245`。 |
| 11 | A* SUCCESS path stale | CONFIRMED_OK | success 时重写 `gridPath_`; 只有 SUCCESS 后读取。 |
| 12 | rejoin indexing | CONFIRMED_OK | 当前三次 bounded rejoin，first success 不阻断 unusable path，`2188-2281`。 |
| 13 | Local-SFC segment indexing | NOT_ENOUGH_EVIDENCE | 维度/进度日志一致，未发现越界；算法复杂，缺单元工件证明全部 segment 正确。 |
| 14 | repaired geometry 与 T 数量 | CONFIRMED_OK | `guide_points-1 == durations.size`，SCP 入口再校验。 |
| 15 | warm-start P/T topology | CONFIRMED_OK | piece partition mismatch 会拒绝 counterfactual；production cache slice 保留匹配 partition。 |
| 16 | accepted cache 使用旧 obstacle context | DESIGN_LIMITATION | cache 保存 obstacle id/kind，但 warm seed 本质只复用轨迹状态；本轮多数日志 `NO_ACCEPTED_STATE`，无三案直接证据。 |
| 17 | generation/start_time 唯一 | CONFIRMED_OK | 三案新 commit 均唯一；retain 保持原 start_time 防 time rewind。 |
| 18 | global update破坏 local metadata | NOT_ENOUGH_EVIDENCE | 当前三案无 metadata 回滚；未做新实验覆盖全部 global update 路径。 |
| 19 | traj_server 接受旧 generation | CONFIRMED_BUG | message 没有 generation gate，callback 无条件替换；本两轮未触发回滚。 |
| 20 | stale trajectory 产生 position command | NOT_ENOUGH_EVIDENCE | 当前 command trajectory_id 与最近 receive 一致；架构上因 #19 有风险。 |
| 21 | final checker检查 dynamic obstacle | CONFIRMED_BUG | `checkMovingObjSafety` 无调用；manager post-check无 dynamic。 |
| 22 | final checker采样时间空洞 | CONFIRMED_BUG | 实际无 authoritative final dynamic checker；已有 risk 只 0.1 s/2 s。 |
| 23 | prediction horizon 覆盖完整 candidate | CONFIRMED_BUG / DESIGN_LIMITATION | 明确不覆盖，2 s vs 3.8–6.2+ s。 |
| 24 | 超 horizon 处理 | DESIGN_LIMITATION | risk/gradient直接停止；predictor自身查询则 clamp末端。 |
| 25 | safety checker失败默认安全 | CONFIRMED_BUG | prediction无效/NaN被 skip；semantic不足返回 true；dynamic hard checker未调用。 |
| 26 | NaN/Inf 进入 score | CONFIRMED_OK（有限） | candidate risk valid/allFinite gate避免 NaN score；未见 NaN日志。 |
| 27 | OSQP failure 后 success 残留 | CONFIRMED_OK | optimize返回值覆盖 result.success；三案 failure均 final_success=0。 |
| 28 | trust retry P/T 恢复 | CONFIRMED_OK（日志覆盖） | failure例 delta P/T=0，基点未被失败 step覆盖。 |
| 29 | failed candidate覆盖 previous safe | CONFIRMED_OK | failed/unselected 不进 committed cache；但“成功却不安全”的 candidate 会覆盖，属 selection bug。 |
| 30 | 多线程 race | NOT_ENOUGH_EVIDENCE | predictor共享向量未见锁；无现有日志证明 race实际发生。 |

## 15. ROOT CAUSE

### 15.1 主根因

**CLAIM**  
三次当前碰撞的共同首要 planner defect 是“动态安全只作为 risk/score/diagnostic，而没有成为 commit 前的绝对 hard contract”；Run2 另有 persistence/lifecycle 覆盖域不完整。

**EVIDENCE**

1. 源码 `candidate_is_good` 只比较 relative improvement，`planner_manager.cpp:3718-3722`。
2. final `SCP_FINAL_OK` 可对应 `0.938002`、`0.627528`、`0.580846`、`0.274587`，都低于 1.1。
3. NOMINAL 预测 `0.253–0.430 m` 仍 selected/commit/publish/activate。
4. `checkMovingObjSafety` 无调用；post checks runtime false且没有 dynamic step。
5. Run2/A1 安全 traj55 被不安全 traj56/57覆盖；Run2/C2安全 traj204被下一非风险 traj205立即覆盖。
6. 三案 command 自身重算均进入真实圆柱。

**COUNTER-EVIDENCE**

- Run1/C2 的 SIDE backend 确有严重 solver failure，不能把所有责任只归 selection。
- 2 s 内 prediction 源与 actual 同模型，未显示 prediction mismatch。
- Run2/C2 odom tracking error较大，但 command本身不安全。

**CONFIDENCE：高。**

### 15.2 最终责任判断

```text
IS_CURRENT_COLLISION_PRIMARILY_PLANNER_PROBLEM: YES
  primary layers:
    1) final dynamic safety checking / candidate acceptance
    2) candidate selection and safe-trajectory persistence lifecycle
    3) Run1-specific repair→MINCO initialization / SCP availability

EXECUTION_TRACKING_IS_PRIMARY_CAUSE: NO
DYNAMIC_PREDICTION_IS_PRIMARY_CAUSE: NO
TRAJECTORY_LIFECYCLE_IS_PRIMARY_CAUSE: YES (Run2 A1/C2); NO as sole explanation for Run1 C2
SOLVER_FAILURE_IS_CAUSALLY_LINKED_TO_COLLISION: PARTIAL
```

## 16. MINIMAL NEXT ACTION

本轮按要求不改代码、不调参。下一轮若授权修复，最小验证顺序应是：

1. 先把“commit 前所有 candidate（含 NOMINAL/SIDE）必须通过同一 absolute-time dynamic hard checker”作为独立修复目标，并证明低于 1.1 的 `SCP_FINAL_OK` 不再可 commit。
2. 再把 previous-safe 保护扩展到 non-trigger overwrite 和“相对改善但绝对不安全”的 SIDE；用 Run2/A1 traj55、Run2/C2 traj204 的生命周期做回归断言。
3. 单独处理 repair 点的按弧长/时窗一致性及短 piece 病态，不能用放宽 QP/trust 参数替代根因修复。
4. 扩展 prediction/hard-check 覆盖完整可执行轨迹或明确终端保守策略，并保存 bag/候选系数以便逐条复算。

这四项是后续建议，不是本轮已执行变更。

## 17. REQUIRED FINAL FIELDS

```text
PROJECT_STRUCTURE_UNDERSTOOD: YES
THREE_COLLISION_EPISODES_RECONSTRUCTED: YES

RUN1_C2_CLASS: TYPE_A
RUN2_A1_CLASS: TYPE_B
RUN2_C2_CLASS: TYPE_B

SAFE_CANDIDATE_EXISTED_BEFORE_COLLISION_RUN1_C2: YES
  note: traj211满足当前1.1m契约，但在碰撞约1.08s前已过期，碰撞决定性周期不可用
SAFE_CANDIDATE_EXISTED_BEFORE_COLLISION_RUN2_A1: YES
  note: traj55, 1.343666m, duration覆盖collision
SAFE_CANDIDATE_EXISTED_BEFORE_COLLISION_RUN2_C2: YES
  note: traj204, 2.039773m, duration覆盖collision

SAFE_CANDIDATE_ACTIVATED_BEFORE_COLLISION_RUN1_C2: YES_BUT_EXPIRED_BEFORE_COLLISION
SAFE_CANDIDATE_ACTIVATED_BEFORE_COLLISION_RUN2_A1: YES_BUT_SUPERSEDED
SAFE_CANDIDATE_ACTIVATED_BEFORE_COLLISION_RUN2_C2: YES_BUT_SUPERSEDED

SOLVER_FAILURE_CAUSALLY_LINKED_TO_COLLISION: PARTIAL
PATH_TO_MINCO_INITIALIZATION_PATHOLOGICAL: YES
DYNAMIC_PREDICTION_MISMATCH_FOUND: NO

EXECUTION_TRACKING_IS_PRIMARY_CAUSE: NO
DYNAMIC_PREDICTION_IS_PRIMARY_CAUSE: NO
TRAJECTORY_LIFECYCLE_IS_PRIMARY_CAUSE: YES_FOR_RUN2_A1_AND_C2; NO_AS_SOLE_CAUSE_FOR_RUN1_C2
```
