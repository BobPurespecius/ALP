# DYNAMIC SAFETY CONTRACT AND CLOSURE AUDIT

审计日期：2026-09-01  
工程：`/home/bob/ALP/egov2_fc65423_constvel`  
范围：只读源码、场景、launch、已有运行日志和离线评估器；本轮未修改生产代码、参数或算法，也未重跑仿真。

## 1. Phase A — 场景是否改变

### 证据

- 当前场景：`ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest.json`
- 当前 SHA-256：`cb6b4925201a10bd3bcaf723a2b34fae95be8bbc0a029670942a37e1cf9440b4`
- Git commit `8cef241`（2026-08-20）中的同一路径文件 SHA-256 相同。
- 当前场景文件未出现在 `git diff` 中。
- 当前 commit 中的 `native_egov2_rviz.launch` 已包含与当前使用一致的密集 `target_waypoints` 前缀和 `target_speed=0.928`。当前 launch 的未提交差异是算法/运行控制项增加，不是场景 JSON 或 obstacle 9 运动参数修改。

### obstacle 6 / 9 参数

两者在当前文件及历史 commit 中一致：

| obstacle | base center (m) | radius (m) | axis | amplitude (m) | period (s) | phase (rad) |
|---|---:|---:|---|---:|---:|---:|
| 6 | (-24.0, 1.0) | 0.28 | (0, 1) | 1.2 | 7.0 | 0.0 |
| 9 | (29.0, 1.2) | 0.28 | (0, 1) | 1.2 | 6.0 | 5.76 |

运动定义为：

`c(t) = base + axis_unit * amplitude * sin(2*pi*t/period + phase)`。

历史 2026-08-28 与当前 2026-09-01 日志中的 UAV 初始状态一致：

- drone 0: `(-35.2, -1.0, 1.5)`
- drone 1: `(-35.2,  0.0, 1.5)`
- drone 2: `(-35.2,  1.0, 1.5)`

场景没有随机运动种子；moving-obstacle 运动由上述确定性正弦公式给出。target route 的实际 RViz/ALP 输入来自 launch 的密集 `target_waypoints`，而 JSON 中是较粗的 `targetTracking.waypointsENU`；两者在历史运行中均保持同一已知路线前缀，未发现路线改变或 launch 用另一 scene 覆盖的证据。

**结论**

```text
SCENE_CHANGED: NO
OBSTACLE9_CHANGED: NO
TARGET_ROUTE_CHANGED: NO (历史 roslaunch 文本对长参数有截断，完整值以 committed launch 与当前 launch 对照为准)
IMPORTANT_DIFFERENCES: 未发现场景/障碍运动/初始状态变化；当前 launch 的差异是 planner 功能和诊断开关。
```

因此，历史“无碰撞”不能用场景变化解释，更可能来自运行时序、轨迹选择、执行误差或当时未触发同一 obstacle-9 encounter。

## 2. Phase B — 当前 dynamic safety contract

### 2.1 几何定义

规划器动态距离统一采用 UAV 轨迹点与预测 obstacle **中心**的三维欧氏距离：

```text
d_planner(t) = || p_uav(t) - c_obstacle(t_query) ||_2
```

其中 `t_query = prediction_epoch + t_local`。规划器没有从该距离中减去 obstacle radius `0.28 m`、UAV radius、tracking-error allowance 或 prediction uncertainty。

离线 evaluator 使用圆柱表面 clearance（在障碍高度范围内为径向距离减 cylinder radius）：

```text
c_offline(t) = distance_to_cylinder_surface(p_uav(t), obstacle(t))
             = radial_distance - radius       (同高时)
```

离线 evaluator 也没有减 UAV radius。故 `d_planner` 与 `c_offline` 不是同一数值；在同高、径向最近情形近似满足 `d_planner = c_offline + 0.28 m`，但离开圆柱高度区间时还受 3D/capsule 几何影响。

### 2.2 各层实际实现

| 层 | 源码/数据 | 距离与半径 | 阈值 | 采样/范围 | 作用 |
|---|---|---|---|---|---|
| movingObjGradCostP | `traj_opt/src/poly_traj_optimizer.cpp:~4173` | center distance；不减任何半径 | `moving_obj_clearance_=1.1 m` | Native MINCO constraint-point lattice；非 goal 时通常前 `2/3`，受 prediction horizon 限制 | 软 quadratic hinge cost，触发距离不足时加 cost/gradient |
| `evaluateDynamicRisk` | `plan_manage/src/planner_manager.cpp:691-741` | center distance；不减半径 | trigger=`1.1+0.15=1.25 m` | `risk_sample_dt_=0.10 s`；最多 `2.0 s`；非 goal 前 `2/3` | 风险触发、候选相对改善统计，不是最终安全判定 |
| `checkMovingObjSafety` | `traj_opt/src/poly_traj_optimizer.cpp:5274-5308` | center distance；不减半径 | `distance >= 1.1 m` | `0.03 s`；最多 `2.0 s` | 设计上是硬 checker，但当前源码无调用点 |
| Hard Corridor SCP dynamic rows | `appendDynamicsConstraints()` | 无 moving-obstacle row | 无 | 仅 v/a/j rows | 动态障碍仍是 soft objective，不是 OSQP separation inequality |
| SIDE/SCP final path | `planner_manager.cpp:3271-3320` | 重新调用 `evaluateDynamicRisk` 只记录 risk | 无绝对 `1.1 m` acceptance | 同 risk sampling | final 主要检查 corridor/static/v/a/j |
| Candidate selection | `planner_manager.cpp:3350-3360` | 使用 `risk.min_distance - nominal.min_distance` | `candidate_min_clearance_gain_`，当前 `0.10 m` | 候选评估范围同 risk | 只要求相对改善，不要求绝对 dynamic clearance |
| KEEP_PREVIOUS_SAFE | `planner_manager.cpp:~650-680` | center distance；不减半径 | `< moving_obj_clearance_` 时 invalidate | remaining trajectory；同 risk 的 horizon/sampling | 比新 candidate 更严格，但只用于 persistence fallback |
| Offline collision | `native_egov2_rviz_scene.py:646-657`, `analyze_native_egov2_metrics.py` | cylinder surface clearance；减 obstacle radius，不减 UAV radius | collision `<=0` | collector odometry sample | 物理/几何结果统计 |
| Offline unsafe | `analyze_native_egov2_metrics.py`, `collision_repeat_20260901` | 同 surface clearance | unsafe `<0.5 m`（部分报告写 `<=0.5`，以脚本实际比较为准） | collector sample，episode 由时间 gap 合并 | 统计指标，不是 planner contract |

### 2.3 prediction time contract

当前链路使用同一类 absolute ROS time：

1. `native_egov2_rviz_scene.py` 发布带 absolute stamp 的 `/dynamic/prediction_i` path；每个 pose stamp 为 `stamp + tau`。
2. `ObjPredictor` 以 pose header stamp 保存样本并按 absolute time 插值。
3. manager 为 candidate batch 固定一个 `planning_prediction_epoch`，并查询 `epoch + local_t`。
4. moving-object soft cost、risk、SIDE 诊断、persistence validation 使用同一 epoch 语义。
5. TrajServer 使用 planner 发布的 `start_time`，按 `now - start_time` 重建并采样同一 polynomial。

未发现当前生产链中把 local time 误当 absolute time 的直接代码证据；问题不是一个简单的 prediction timestamp 单位错误。

### 2.4 为什么 `command clearance ≈ +0.006 m` 仍被接受

该 `+0.006 m` 来自离线 cylinder **surface** clearance；规划器日志里的 `dynamic_clearance` 是 **center distance**，两者不是同一量。除此之外，新 candidate 的接受条件（源码 `3350-3360`）只要求相对 nominal 改善至少约 `0.10 m`，没有要求 candidate 的绝对 center distance 达到 `1.1 m`。nominal 也可以走 selection/commit 路径，且其 risk 只用于统计和触发。

因此在 obstacle 9 encounter 中，当前代码合法接受的实际合同是：

- polynomial 的 static/corridor/v/a/j checks 通过；
- 若是 SIDE candidate，dynamic risk 相对 nominal 有足够改善；或触发后保留/提交 nominal；
- 没有 universal moving-obstacle final hard check；
- 没有 tracking-error 或 prediction-uncertainty safety budget。

这解释了“命令轨迹表面 clearance 仅 6 mm 仍 final accepted”，并不表示代码认为 6 mm 是满足 `1.1 m` 动态合同的安全距离。

**合同结论**

```text
DYNAMIC_SAFETY_CONTRACT: OTHER
INTENDED_OPTIMIZER_THRESHOLD: center distance >= 1.1 m
EFFECTIVE_FINAL_ACCEPTANCE: no universal absolute moving-obstacle threshold;
                            SIDE uses relative gain, nominal may be committed
FINAL_ACCEPTANCE_THRESHOLD: candidate_clearance_gain >= 0.10 m (SIDE acceptance)
SCP_THRESHOLD: no moving-obstacle hard row
CHECK_MOVING_OBJ_SAFETY_THRESHOLD: center distance >= 1.1 m (function unused)
OFFLINE_UNSAFE_THRESHOLD: surface clearance < 0.5 m (statistical metric)
CLEARANCE_DEFINITION_CONSISTENT: NO
CONTRACT_ENFORCEMENT_BUG: YES
```

## 3. Phase C — end-to-end closure audit

沿 `prediction -> risk -> candidate -> SCP -> final checker -> selection -> persistence -> publish -> TrajServer -> controller -> actual evaluator` 追踪结果如下。

| 路径点 | 当前行为 | 分类 | 严重度 |
|---|---|---|---|
| Dynamic prediction | absolute stamped paths，manager 固定 prediction epoch；当前链路时间基准一致 | — | — |
| Risk trigger | 1.25 m center-distance trigger，0.10 s/2 s/前 2/3 sampling；仅触发 recovery | CONTRACT_GAP | Major |
| Candidate generation/SCP | moving obstacle 只有 native soft cost/gradient；Hard Corridor OSQP 没有动态 separation row | CONTRACT_GAP | Major |
| SCP final checker | corridor、Local SFC、native static、v/a/j；没有调用 moving-object hard checker | BUG | Critical |
| `checkMovingObjSafety()` | 函数存在但 `rg` 全工程只找到 declaration/definition，无 call site | BUG | Critical |
| Selection | `candidate_clearance_gain >= 0.10 m`，非绝对 `>=1.1 m`；nominal 也可提交 | CONTRACT_GAP / BUG | Critical |
| Persistence | KEEP_PREVIOUS_SAFE 会按当前 prediction 重验 dynamic center distance `<1.1 m`；但仅短 horizon/离散采样，且这不是新 candidate final check | PARTIAL / ROBUSTNESS_GAP | Major |
| Runtime FSM safety timer | `ego_replan_fsm.cpp:395-590` 的 `dangerous` 只检查 inflated static occupancy 与 swarm trajectory；没有 moving-obstacle distance check | BUG | Critical |
| Map input to FSM | scene publisher `publish_map()` 的 cloud 由 `static_points` 构成；moving obstacles 单独发布 pose/prediction/markers，不进入该 occupancy cloud | CONTRACT_GAP | Critical |
| Publish | `setLocalTrajFromOpt` 后 `polyTraj2ROSMsg` 直接序列化相同 durations/coefs/start time | — | — |
| TrajServer | `traj_server.cpp:47-89` 复建同一 polynomial；`cmdCallback` 用 `now-start_time` 采样并发布 | — | — |
| Controller execution | command 轨迹与实际 odometry 存在误差；历史严格对齐 obstacle-9 encounter 的 P95 约 `0.301 m`、max 约 `0.321 m` | ROBUSTNESS_GAP | Critical |
| Execution safety budget | planner 没有把 tracking error、prediction uncertainty 或 obstacle/UAV body radius 计入 dynamic budget | CONTRACT_GAP / ROBUSTNESS_GAP | Critical |
| Trajectory switching | 新轨迹以 `start_time=now` 激活；没有显式 swept transition dynamic check。位置/速度/加速度从 odom 构造以保持连续，但误差包络未纳入安全合同 | ROBUSTNESS_GAP | Major |
| Discrete/finite horizon | risk/checker 使用离散采样，通常最多 2 s；采样点之间及 horizon 之后没有连续时间动态保证 | ROBUSTNESS_GAP | Major |
| Offline evaluator | surface geometry、`<=0` collision、`<0.5` unsafe 与 planner center-distance 口径不同 | DIAGNOSTIC_ONLY + CONTRACT_GAP | Major |

### 3.1 当前 obstacle-9 碰撞时仍被“合法满足”的合同

collision 发生时，现有实现仍可能同时满足：

1. 选中 polynomial 的 static occupancy、SIDE corridor、Local SFC 以及 v/a/j 检查；
2. SIDE candidate 相对 nominal 的 sampled center-distance improvement；
3. publish/TrajServer polynomial 系数和时间段一致。

但没有被强制满足的项是：

- 绝对 moving-obstacle center distance `>=1.1 m`；
- cylinder surface clearance `>0` 或 `>=0.5 m`；
- UAV body radius + tracking-error + prediction-uncertainty 后的 robust clearance；
- continuous-time between-sample dynamic separation。

这正是 command surface clearance `+0.006 m` 可以通过 final acceptance、而 odometry surface clearance 达到 `-0.1828 m` 的闭环缺口。

### 3.2 warm-start 影响范围

已有 warm-start 代码只改变 MINCO 初始化（accepted P/T/polynomial 的 shifted state）；源码路径仍经过既有 SCP、post static/dynamics/swarm checks 和 selection。未发现 warm-start 直接绕过这些检查的证据。但由于 moving-obstacle final checker 本身未接入，fresh 与 warm 都没有受到绝对动态安全硬合同保护。

## 4. 最终结论

```text
SCENE_CHANGED: NO
OBSTACLE9_CHANGED: NO
TARGET_ROUTE_CHANGED: NO

WHY_PREVIOUS_ZERO_COLLISION_POSSIBLE:
  场景没有改变；历史差异更可能来自运行时序、candidate selection、实际执行误差和
  当时不同的 encounter/trajectory timing，而非 obstacle 9 几何或运动参数变化。

CURRENT_DYNAMIC_SAFETY_CONTRACT:
  规划器 soft/risk/persistence 使用 UAV-to-obstacle-center distance，意向阈值 1.1 m；
  SIDE acceptance 实际只要求相对 nominal gain，nominal 无 universal absolute dynamic check；
  offline evaluator 则使用减 obstacle radius 的 surface clearance。

WHY_6MM_TRAJECTORY_IS_ACCEPTED:
  6 mm 是 offline surface clearance，不是 planner center distance；且 final selection 没有
  强制 1.1 m absolute moving-obstacle threshold，只有 relative-gain/其它已实现约束。

CONTRACT_ENFORCEMENT_BUG: YES

TOP_UNCLOSED_CODE_PATHS:
  1. checkMovingObjSafety() 未被调用，final candidate/commit 没有 moving-obstacle hard check。
  2. FSM checkCollisionCallback() 只查 static occupancy/swarm，moving obstacle 不在 occupancy cloud。
  3. planner 没有把 tracking-error、prediction uncertainty、body radius 纳入 dynamic safety budget。
  4. selection 以 relative clearance gain 而非 absolute safety contract 接受 candidate/nominal。
  5. offline surface-clearance 与 planner center-distance 的合同/诊断口径不一致。

FIRST_BLOCKER_TO_FIX:
  先统一并真正执行 moving-obstacle final acceptance contract（至少让最终提交路径经过同一
  明确定义的动态安全 checker；本报告不实施修复）。

SECOND_BLOCKER:
  将执行 tracking error / prediction uncertainty / body geometry 纳入安全预算，并覆盖 runtime
  trajectory switching 与 continuous-time/采样间隙风险。

DO_NOT_FIX_YET:
  不应先调场景、SIDE/A*/SFC/SCP/OSQP 参数，也不应把 0.5 m offline unsafe 指标直接当作
  当前 planner 已执行的安全合同。
```

## 5. Audit status

```text
READ_ONLY_AUDIT: COMPLETE
PRODUCTION_CODE_CHANGED: NO
PARAMETERS_CHANGED: NO
ALGORITHM_CHANGED: NO
RERUN_PERFORMED: NO
```
