# ALP 源码审计报告（只读）

工作区：`/home/bob/ALP/egov2_fc65423_constvel`
本轮**未修改任何源码**；只读审计 + 本报告。
RRCT_ACCESSED: NO
RRCT_CHANGED: NO

审计依据：当前 production 源码调用链 + `runs/<RUN_ID>/roslaunch_argv.txt` + runtime 日志自证。
未采用旧报告/注释/README/设计文档作为结论依据（注释仅用于定位）。

---

## 1. 当前真实 planning cycle（nominal detection → 最终 selection）

主函数：`EGOPlannerManager::reboundReplan()`（`plan_manage/src/planner_manager.cpp`）

```
reboundReplan()
  ├─ planning_prediction_epoch = local_activation_time_        ← 本批次唯一时间原点
  ├─ target_at_prediction_epoch = targetPositionAt(epoch)
  │
  ├─ [A] raw LOS truth 扫描（独立于 J_vis / FOV / BODY 风险）
  │     for sample in [0, raw_los_interval_scan_horizon] step visibility_sample_dt_
  │         observer = nominal_visibility_trajectory.getPos(min(t, total_duration))
  │         target   = target_at_prediction_epoch + object_vel * relative_time
  │         ├─ 静态: ploy_traj_opt_->queryStaticLosClearance(observer, target,
  │         │           clearance, witness)
  │         │   命中条件: query_valid && witness.valid && clearance <= visibility_occlusion_margin_
  │         └─ 动态: 对每个 hasPrediction(id) 的障碍
  │              radius = 0.5*max(scale.x,scale.y) + visibility_occlusion_margin_
  │              segmentIntersectsVerticalCylinder(observer,target,center,radius,scale.z())
  │         记录 raw_los_occlusion_enter_rel_ / _exit_rel_ / _interval_observed_
  │
  ├─ [B] current LOS 判定（无条件执行，观测者 = 当前 odom = start_pt）
  │     current_raw_los_blocked_ / current_raw_los_blocker_id_ / _dynamic_
  │     若 blocked：raw_los_occlusion_enter_rel_ = 0.0（现在就恢复）
  │
  ├─ [C] nominal MINCO 求解（LBFGS 软代价）
  │
  ├─ [D] conflict descriptor 组装（BODY 与 LOS 证据分开写）
  │     ├─ static LOS witness → reason_mask |= CONFLICT_LOS_OCCLUSION
  │     ├─ static body support → reason_mask |= CONFLICT_BODY_SAFETY
  │     ├─ dynamic BODY risk   → reason_mask |= CONFLICT_BODY_SAFETY
  │     └─ dynamic LOS         → 若 raw_dynamic_los_time < 现有 los_conflict_time 则替换
  │
  ├─ [E] 是否为该冲突生成 SIDE（门）：
  │     enable_risk_triggered_candidates_ 为真
  │     且 visibility_topology_trigger（由 BODY 描述符或 raw LOS 描述符给出）
  │
  ├─ [F] frame 选择：
  │     body_hard_conflict = (reason_mask & BODY_SAFETY) && (static_body_support_active
  │                           || risk.min_distance < absolute_dynamic_threshold)
  │     observation_frame_for_candidate = los_observation_geometry_valid && !body_hard_conflict
  │     use_observation_frame = observation_frame_for_candidate   ← 可被静态不可行降级
  │
  ├─ [G] 两侧候选：attempt_side(+1) / attempt_side(-1)  → SIDE_PLUS / SIDE_MINUS
  │     每侧内部：seed → 静态预检 → (必要时) A* 修复 → MINCO/SCP → 动态/集群/局部 SFC 校验
  │
  └─ [H] finalizeCapturedCandidates() → 排序选择 → commit → publish
        （team 通道开启时经 capture_output/deferred + team transaction）
```

**runtime 证据**：`runs/20260919_205929_346132/roslaunch_argv.txt` 含
`enable_risk_triggered_candidates:=true`、`enable_joint_pt_optimization:=true`；
启动自证 `[team-vis-opt-config] enabled=1 joint_pt=1 runtime_ready=1 scene_status=OK`。

---

## 2. 多威胁同时出现时的真实数据流

### 2.1 三种威胁各自的检测点与真实数学条件

| 威胁 | 检测位置（函数/变量） | 真实触发条件 |
|---|---|---|
| **STATIC BODY** | `checkTrajectoryStaticSafety()` + `static_body_support_active` / `static_body_conflict_time` | 名义轨迹在某采样时刻与静态占据栅格碰撞；`static_body_support_active` 为真即写入 BODY 描述符 |
| **DYNAMIC BODY** | `evaluateDynamicRisk()` → `DynamicRiskInfo`（`dynamic_risk_at_epoch`）| 对每个 `hasPrediction(id)` 的障碍：`distance = ‖p_traj(t) − p_obs(t_world)‖`；`hard_collision = distance < hard_clearance`；`triggered = min_distance < trigger_distance`。`hard_clearance = dynamicHardClearanceForObject(id) = dynamic_body_radius_(0.384) + 0.5*max(scale)`；`trigger_distance = getMovingObjClearance() + margin` |
| **LOS OCCLUSION（静态）** | `queryStaticLosClearance()`（`static_los_geometry_`）+ raw LOS 扫描 | `clearance <= visibility_occlusion_margin_` 且段与圆柱相交；witness 给出 `primitive_index / primitive_type / primitive_center / effective_radius / hit_point` |
| **LOS OCCLUSION（动态）** | raw LOS 扫描内的 `segmentIntersectsVerticalCylinder()` | `radius = 0.5*max(scale.x,scale.y) + visibility_occlusion_margin_`，段与垂直圆柱相交即判遮挡 |

**执行/预测视野**（三处都用同一构造）：

```
check_end = nominal_duration;  if (!touch_goal) check_end *= 2/3;
check_end = clamp(check_end, 0, getMovingObjPredictionHorizon());
```

`raw_los_interval_scan_horizon = max(visibility_authority_horizon, getMovingObjPredictionHorizon())`。

### 2.2 是否构造 active conflict list

**不构造 list。** 真实数据结构（`planner_manager.h`）：

```cpp
enum { CONFLICT_NONE = 0, CONFLICT_BODY_SAFETY = 1, CONFLICT_LOS_OCCLUSION = 2 };

struct ConflictDescriptor {
  bool valid;  unsigned int reason_mask;
  ConflictObstacleMotion obstacle_motion;  int obstacle_identity;  int primitive_type;
  // BODY 与 LOS 各一个独立 witness（不是 vector）
  double body_conflict_time, los_conflict_time;
  int body_obstacle_identity, los_obstacle_identity;
  ConflictObstacleMotion body_obstacle_motion, los_obstacle_motion;
  Eigen::Vector3d body_obstacle_position, los_obstacle_position;
  Eigen::Vector3d observer_position, target_position, obstacle_position;
  Eigen::Vector3d frame_forward, frame_right;
  Eigen::Vector3d clearance_witness;  double clearance, radius;
  double first_risk_world_time, risk_interval_start, risk_interval_end;
};
```

```cpp
struct DynamicRiskInfo {          // BODY 侧只有"单个"最近障碍 witness
  bool valid, triggered, hard_collision;
  int obstacle_id;                // 单个 id
  double min_distance, hard_clearance, trigger_distance, conflict_time;
  Eigen::Vector3d trajectory_position, obstacle_position;
};
```

结论：
- **1 个 primary BODY witness + 1 个 LOS witness**，靠 `reason_mask` 两位表达共存；
- 有 `obstacle_motion`（STATIC/DYNAMIC/UNKNOWN）+ `blocker id` + `risk_interval_start/end`；
- **没有 vector<Conflict>，没有 active conflict list**；
- 多个静态障碍、多个动态障碍、多个 LOS blocker 同时存在时，**只保留各自一个代表**：动态 BODY 取 `min_distance` 最小的那个（`if (!risk.valid || distance < risk.min_distance)`）；LOS 取时间最早的那个。

### 2.3 primary conflict 的选择代码

动态 LOS 覆盖静态 LOS（**earliest-time tie-break**）：

```cpp
const bool replace_los =
    (conflict.reason_mask & CONFLICT_LOS_OCCLUSION) == 0 ||
    raw_dynamic_los_time < conflict.los_conflict_time;
```

BODY 优先于 LOS —— 由 `body_hard_conflict` 判定与 frame 选择实现，且有显式日志语义：

```
"Keep the hard BODY witness authoritative when the same planning
 cycle also carries a visibility trigger. The LOS sample is separate
 evidence and must not move the safety SIDE window earlier or later."
```

因此：
- **BODY 与 LOS 不比较时间先后，优先级是 BODY > LOS**（frame 层面）；
- 同为 LOS 时按 **earliest `los_conflict_time`** 替换；
- 同为 BODY 时按 **minimum distance** 保留；
- **没有 risk score、没有 static/dynamic 之间的显式 tie-break**（静态 BODY 与动态 BODY 都写进同一个 BODY 槽，后写者覆盖）。

---

## 3. 5 个 threat 时到底生成几条 trajectory

**固定 3 条：NOMINAL + SIDE_PLUS + SIDE_MINUS。**

```cpp
result.kind = side > 0 ? CandidateKind::SIDE_PLUS : CandidateKind::SIDE_MINUS;
...
const int order[2] = {preferred_side_valid ? preferred_side : +1,
                      preferred_side_valid ? -preferred_side : -1};
for (const int side : order) { ... attempt_side(side, false); }
```

- 侧向只有 `+1 / -1` 两个值，**不是 2×M，也不是 2^M**；5 个 threat 仍然是 1 NOMINAL + 2 SIDE = **3 条**；
- `preferred_side` 只影响**尝试顺序**（注释明写 "Preferred side is only an attempt-order hint"），不影响数量；
- NOMINAL 始终作为候选保留（`nominal_result`）。

### 3.1 L/R frame 是否 threat-specific

**两个 threat 使用两套不同 frame，但同一条候选只能用一个。**

BODY 侧向 frame（`log_side_space`）：

```cpp
Eigen::Vector3d side_direction =
    (local_target_pt - start_pt).cross(Eigen::Vector3d::UnitZ());   // 运动切向 × e_z
side_direction.z() = 0.0;  side_direction.normalize();
```

LOS 观察 frame（`observation-topology-plane` 段）：

```cpp
const Eigen::Vector2d axis = blocker_xy - target_xy;
const double distance = axis.norm();
const double radius = max(0.0, conflict.radius);
if (distance > radius + 1e-6) {
  const Eigen::Vector2d e = axis / distance;
  const Eigen::Vector2d n_right(e.y(), -e.x());
  const double alpha = asin(min(1.0, radius / distance));
  const Eigen::Vector2d m2 = -sin(alpha)*e + side*cos(alpha)*n_right;
  const Eigen::Vector3d observation_normal(m2.x(), m2.y(), 0.0);
  const Eigen::Vector3d observation_point(conflict.target_position.x(),
                                          conflict.target_position.y(), 0.0);
}
```

**关键结论：同一个 `side` 符号被同时用于两套 frame，但二者不会同时生效于同一条候选**：

```cpp
const bool observation_frame_for_candidate =
    los_observation_geometry_valid && !body_hard_conflict;
bool use_observation_frame = observation_frame_for_candidate;   // 之后还可降级为 false
```

且存在静态不可行降级：

```cpp
use_observation_frame = false;   // reason=OBSERVATION_SEED_STATIC_INFEASIBLE
                                 // action=PATH_FRAME_SIDE
```

所以：
- **能表达 `s_body=+1` 而 `s_los=-1` 吗？不能。** 一条候选只有一个 `side`；
- 但 BODY 与 LOS **不是被"共享一个全局 side"统一处理**，而是**按优先级二选一**：有 BODY hard 冲突就用 BODY/path frame 建 seed，LOS 观察面作为**独立的 event-scoped Local-SFC 平面**另外附加；
- 若只有 LOS，则用 observation frame 建 seed。

---

## 4. A* 的真实职责与触发条件

### 4.1 触发条件（不是"检测到静态障碍就叫"）

A* 由 **SIDE guide 静态预检失败**触发：

```cpp
if (enable_side_local_astar_repair_ && astar_base_valid && grid_map_ && ...)
    → A* 修复
```

输入包含：`astar_base_offset`、`anchor_start`、`anchor_goal`、
`collision_start / collision_end`（`side-static-aware` 给出的冲突窗口），
以及下游 rejoin 候选（日志 `rejoin_attempt=` / `rejoin_index=`）。

日志实证：`[side-astar-repair] type=MINUS trigger_reason=STATIC_INFEASIBLE
base_offset=0.700 collision_start=0.618939 collision_end=1.856816 anchor_start=(...) anchor_goal=(...)`

**因此理解 A 正确**：A* 是 **LEFT/RIGHT SIDE guide 被静态地图阻塞时的局部修补器**，
不是"静态威胁处理器"；`trigger_reason` 说明它是被 side guide 的静态不可行性驱动的。

### 4.2 admission checks

```cpp
const bool astar_accept = astar_base_valid && path_static_free &&
                          path_side_valid && repaired_side_valid &&
                          local_sfc_build_valid;
```

即：修复路径静态自由 + 仍满足 side 语义 + 局部 SFC 可构建。日志同时打印
`static_free= / static_reason= / dynamic_valid= / side_semantic_valid= / result=ASTAR_REPAIR_ACCEPT`。

### 4.3 为什么 A* 不直接处理 dynamic / LOS

- A* 的 state 只有空间坐标（`grid_map_` 静态占据栅格），**没有时间维**；
- 动态障碍**不进入** A* occupancy（其位置由 `obj_predictor_->evaluateConstVel(id, world_time)` 在风险/校验层按世界时间求值，未栅格化进 A*）；
- **不存在 space-time A***（源码无 arrival-time 维度的搜索）；
- 动态碰撞由 `evaluateDynamicRisk()` / `validateExecutionTrajectory()` 负责；
- LOS 既**没有**被 rasterize 成 occupancy，也**没有**作为 A* 禁飞区：它只以
  `LocalSfcPlane{source=LOS_OBSERVATION_SIDE}` 的形式存在，供后端采样校验使用（见第 5 节）。

---

## 5. LOS observation plane 是否真的进入 MINCO-SCP-OSQP

### 5.1 公式：确实就是那一套

已在第 3.1 节给出源码，与 `e = (C−T)/‖·‖`、`n_R=(e_y,−e_x)`、`α=asin(min(1,r/d))`、
`m_s = −sinα·e + s·cosα·n_R`、`m_s^T(p_xy−T_xy) ≥ 0` 完全一致。

### 5.2 字段与区间

```cpp
struct LocalSfcPlane {
  int source{STATIC_COLLISION_CORRIDOR};   // 或 LOS_OBSERVATION_SIDE
  int blocker_index, blocker_type, side_sign;
  unsigned long geometry_revision;
  Eigen::Vector3d normal, point, guide;
  double clearance{0.0};
  double active_start{0.0}, active_end{0.0};
  bool world_time_anchored{false};
  double world_anchor_time{0.0};
};
```

- `normal = observation_normal = (m2.x, m2.y, 0)` → **z 分量为 0**；
- `point = (target_position.x, target_position.y, 0)`；
- `clearance = 0.0`；
- `active_start/active_end` = **真实遮挡区间** `[enter, exit]`（遮挡区间已知时），
  未知时才退回 `reached_side` 分支；
- `world_time_anchored = true`，`world_anchor_time = planning_prediction_epoch`。

deadline 语义：`los_side_reach_deadline = los_occlusion_enter_rel`（即 `t_side_reach ≤ t_enter`），
在源码与日志 `LOS_SIDE_REACH_DEADLINE=` 中均存在。

world time：区间以**候选 activation 为相对零点**，`world_anchor_time` 记录该零点；
activation 变化时经 `rebaseWorldTimeAnchoredPlane()` 做
`t_rel_new = (t_rel_old + anchor_old) − anchor_new`（**不是**按 `T_new/T_old` 比例缩放）。

### 5.3 决定性结论：**这条硬行当前没有进入 SCP/OSQP**

`candidate_local_sfc_planes_` 在整个 `poly_traj_optimizer.cpp` 里**只有 1 个读取点**：

```cpp
double PolyTrajOptimizer::evaluateCandidateLocalSfcMaxViolation(
    const poly_traj::Trajectory &traj) const      // 25 个采样点的后验违规检查
{ ... for (const LocalSfcPlane &plane : candidate_local_sfc_planes_) ... }
```

- `runCandidateHardCorridorSCP()` 内**完全不读** `candidate_local_sfc_planes_`（实测 grep 为空）；
- `LOS_OBSERVATION_SIDE` 这个枚举值在 optimizer 源码中**出现 0 次** —— optimizer **从不按 source 区分平面**；
- `evaluateCandidateLocalSfcMaxViolation` 在生产里只有 2 个调用点，且两处都在
  **feasible-initializer fallback** 分支（对 `side_init_traj` / `fallback_traj` 做后验检查，阈值 `<= 1e-3`），
  **不在 SCP 迭代内**。

因此 LOS 观察面在 production 中是
**"后验采样式否决过滤器"（25 点 + 最终阶段密集采样）**，
而非 SCP/OSQP 硬约束行。

### 5.4 runtime flag

`enable_candidate_hard_corridor_scp`：launch 默认 `false`，最新三次 run 的真实 argv 均为 `false`。
即便置 true，按 5.3 它也只影响 `runCandidateHardCorridorSCP` 是否被调用，
而该函数不消费这些平面，所以**LOS 平面仍不会变成硬行**。

---

## 6. 当前实际 visibility objective 全部公式

### 6.1 Local 方向性可见性代价（`poly_traj_optimizer.cpp`）

调用链：`optimizeTrajectoryWithinBudget` → `addDirectionalVisibilityGradCost2CT(gradT, cost)`
（`poly_traj_optimizer.cpp:4740`，无开关短路）。

启用谓词：

```cpp
bool directionalVisibilityCostEnabled(tracking_configured, guidance_active, weight) {
  return tracking_configured && guidance_active && std::isfinite(weight) && weight > 0.0;
}
```

权重：`weight_visibility_`，launch `optimization/weight_visibility` 默认 **20.0**。

代价合成：

```cpp
result.weighted_cost = result.static_los_cost + result.dynamic_los_cost + result.fov_cost;
// 每项形如  weight_visibility_ * risk.value * risk.value
// 梯度       factor = 2*weight_visibility_*risk.value*risk.derivative_clearance
//            gradient_position = factor * gradient_observer
//            gradient_time     = gradient_position·v + gradient_target_position·object_v
```

即 `J_vis = w_vis (R_static² + R_dynamic² + R_fov²)`，`R_*` 由下面的 `directionalClearanceRisk` 给出。
**梯度同时对 P 与 T 传播**（`gradient_time` / `gradient_previous_time`）。

### 6.2 deep-risk continuation（`tracking_visibility_geometry.h::directionalClearanceRisk`）

```cpp
if (clearance >= outer_clearance) return 0;                 // R = 0
if (clearance <= inner_clearance) {
  const double d = (inner_clearance - clearance) / width;
  const double one_minus_attenuation = -std::expm1(-deep_beta * d);
  value = 1.0 + deep_kappa * (d - one_minus_attenuation / deep_beta);
  derivative_clearance = -deep_kappa * one_minus_attenuation / width;
} else {
  const double u = (outer_clearance - clearance) / width;
  value = u*u*(3.0 - 2.0*u);
  derivative_clearance = -6.0*u*(1.0-u)/width;
}
```

与题述一致，且 **deep 区 `dR/dc = −κ(1−exp(−βd))/width`，当 d→∞ 时趋于常数 −κ/width，不再趋 0**。
参数来源：`directional_visibility_deep_risk_kappa_`（launch 默认 1.0）、
`directional_visibility_deep_risk_beta_`（默认 2.0）。
呼叫处：静态 LOS `directionalClearanceRisk(clearance, static_los_margin_, static_los_margin_+directional_static_risk_transition_, κ, β)`；
FOV 用 `directionalClearanceRisk(horizontal_margin, 0, 0.25*horizontal_half)`（垂直同理）。

### 6.3 Team 指标（`team_visibility_optimizer.cpp`）

```cpp
double TeamVisibilityOptimizer::q2(const std::array<double,3> &v) {
  return v0*v1 + v0*v2 + v1*v2 - 2.0*v0*v1*v2;          // Q2
}
double TeamVisibilityOptimizer::blackout(const std::array<double,3> &v) {
  return (1.0-v0)*(1.0-v1)*(1.0-v2);                    // b0
}
```

`b_K2 = 1 − Q2` 由 `k2_cost` 派生；blackout 连续项由 `blackout_cost` 提供。
**但 runtime 权重为 0**：启动自证 `stage2_k2_continuity=0 stage2_blackout=0 blackout_weight=0.000000`。
K2 的**离散**统计（`binary_k2 / binary_all3 / binary_none / longest_k2_loss / longest_blackout /
binary_weakest_camera`）由 `count<2`、`count==0` 的游程积分给出，属 **metrics**，用于采纳判断与 telemetry。

### 6.4 multiview / pairwise direction quality（`encirclement_geometry.h::multiviewQuality`）

```cpp
MultiviewPolicy { double bad{25.0*M_PI/180.0}, good{60.0*M_PI/180.0}; }   // 25° / 60°
for i in 0..2:
  j = (i+1)%3
  wrapped = atan2(sin(θi−θj), cos(θi−θj));  delta = |wrapped|
  u = clamp((delta − bad)/(good − bad), 0, 1)
  quality = u*u*(3−2u)
  result.quality += quality/3;   result.penalty += (1−quality)²/3
```

`Q_dir = Σ quality/3`，`J_dir = Σ (1−quality)²/3`。与题述一致（`bad=25°`、`good=60°`）。

**是否进入 objective：没有。** team objective 实际表达式为：

```cpp
metrics.objective = params_.k2_weight * metrics.k2_cost
                  + params_.accumulated_weight * metrics.accumulated_cost
                  + params_.deviation_weight * metrics.deviation_cost
                  + params_.jerk_weight * metrics.jerk_cost
                  + params_.encirclement_gap_weight * metrics.encirclement_gap_cost;
```

`multiview_quality` / `multiview_cost` **不在其中** —— 它们只进 telemetry
（`[multiview-joint-telemetry] Q_dir_before/after`、`[multiview-proposal]`）。

**顺带发现**：`deviation_weight * deviation_cost` 在 `metrics.objective` 里**出现两次**
（一次作为 `metrics.weighted_deviation`，一次被重复写成
`params_.deviation_weight * metrics.deviation_cost`），即该项被**双倍计入**。本轮只报告不修改。

### 6.5 circular gap（`encirclement_geometry.h`）

```cpp
encirclementAngles: θ_i = wrapToPi(phi0 + 2π·slot_i/3)
circularGapGeometry: bearings = atan2(rel.y, rel.x)，排序后 gaps[j] = θ_(j+1) − θ_j (+2π for j=2)
encirclementGeometryCost(relative, theta_min, theta_max, grad):
  for j in 0..2:
    hi = max(0, gaps[j] − theta_max)
    lo = max(0, theta_min − gaps[j])
    cost += hi*hi + lo*lo
    grad: (i==a ? −2 : +2) * (hi − lo) * (−rel_i.y, rel_i.x, 0)/‖rel_i.head<2>‖²
```

与题述 `J_gap = Σ_j ([g_j−θ_max]_+² + [θ_min−g_j]_+²)` **一致**。

实际值（`multi_uav_topology_coordinator.cpp`）：

```
gap_max_deg  = 170.0   (param /encirclement_geometry/max_circular_gap_deg)
theta_min    = 25.0°   (param preferred_view_angle_deg) → team_optimizer_params_.theta_min
theta_max    = 170.0°  → team_optimizer_params_.theta_gap_max
```

**进入 optimizer：是**，Local 与 Team 两侧都进：
- Local：`poly_traj_optimizer.cpp:3469` `costp += encirclement_local_gap_weight_ * gap_cost`，
  `encirclement_local_gap_weight` 默认 **100.0**；
- Team：`/encirclement_geometry/joint_gap_weight` 默认 **4.0**，进 `metrics.objective`。

**它是 cost，不是 metric。** 另有 metric 用途：`topology_coordinator_core.cpp:198`
把同一函数结果累加进 `metrics.geometry_cost` 供 selector 排序。

### 6.6 汇总：当前"可见性最大化"的真实层次

```
Local 单机连续 cost：
    J_vis = w_vis(R_static² + R_dynamic² + R_fov²)          w_vis = 20.0   [启用]
  + J_gap(local) = encirclement_local_gap_weight * gap_cost  w = 100.0      [启用]

Team/joint cost（仅当 joint_pt 打开）：
    J_team = 0.20*J_k2 + 4.0*J_acc + 1.0*J_dev(×2 重复计入) + 1e-5*J_jerk
           + 4.0*J_gap(team)                                              [启用]
    K2-continuity / blackout 连续项：权重 0                                [未启用]
    multiview Q_dir / J_dir：不进 objective                                 [仅 telemetry]

candidate selection metric：
    binary_k2 / binary_all3 / binary_none / longest_k2_loss /
    longest_blackout / binary_weakest_camera / mean_visible / geometry_cost  [selector 用]
```

---

## 7. 合围 seed 的真实公式

`cooperative_viewpoint_core.h`：

```cpp
std::array<double,3> encirclementAngles(phi0, slot_for_uav) {
  result[uav] = wrapToPi(phi0 + 2.0*M_PI*slot_for_uav[uav]/3.0);   // 120° 等分
}
std::array<Eigen::Vector3d,3> encirclementReferencePositions(target_position, phi0, slot) {
  const auto bearings = encirclementAngles(phi0, slot);
  for uav:
    const double radius = offsets_[uav].head<2>().norm();          // R*_i = ‖offset_i‖
    result[uav] = target_position + Vector3d(radius*cos(bearings[uav]),
                                             radius*sin(bearings[uav]),
                                             offsets_[uav].z());   // z*_i = offset_i.z()
}
```

| 符号 | 真实来源 |
|---|---|
| `phi_0` | **初始三机方位角的圆均值**（`estimateInitialPhi0`：`estimate = wrapToPi(bearing_uav − 2π·slot_uav/3)`，再取 `atan2(Σsin, Σcos)`）。不是目标航向、不是世界 x 轴。无 odom 时用 `offsets_` |
| `slot_i` | 启动时在 3! 个排列中枚举，选 `initialBearingDisplacement` 最小且 `evaluateEncirclement` 有效的那个；**此后固定**（`slot_for_uav_ = best_assignment`） |
| `R*` | `offsets_[uav].head<2>().norm()`（每机各自的编队半径） |
| `z*` | `offsets_[uav].z()` |
| target velocity | **不进入** seed 位置公式（只用于 `evaluateEncirclement` 的评价） |
| E/O/S role | **ALP 没有角色概念**（源码中无 role/slot-role 映射；slot 只由几何位移决定） |

**seed 性质：soft reference。** 证据：
- 对外发布为 `[cooperative-soft-reference]`，`referencePositions()` 只产出参考点；
- planner 侧通过 tracking 类 cost 消费（下节），**没有任何 hard constraint 引用它**；
- `validateEncirclementLimits()` 只在**构造期**校验参数合法性（抛异常），不是每周期硬约束。

seed 进入 Local optimizer 的真实公式（`poly_traj_optimizer.cpp`）：

```cpp
const Eigen::Vector3d q = p - object_p_ - object_v_t * t;
const Eigen::Vector3d seed = object_rotation * relative_tracking_p_;
const double radius = q.head<2>().norm();
const double desired_radius = seed.head<2>().norm();
band_error(e,width) = copysign(max(0.0, |e|-width), e)
er = band_error(radius - desired_radius, observation_radius_band_);   // 0.35
ez = band_error(q.z() - seed.z(),        observation_height_band_);    // 0.20
costp += wei_tracking_*(er*er + ez*ez);
```

与题述 `e_r = band(r−r*, 0.35)`、`e_z = band(q_z−seed_z, 0.20)` 一致。

---

## 8. bearing recovery 是否存在

**存在，且当前启用。**

```cpp
if (encirclement_bearing_recovery_weight_ > 0.0 && radius > 1e-8 && seed.head<2>().norm() > 1e-8) {
  const double phi      = std::atan2(q.y(), q.x());
  const double phi_star = std::atan2(seed.y(), seed.x());
  double e_phi = phi - phi_star;  while (>π) −=2π;  while (<−π) +=2π;
  const double rho = clamp(risk_state.risk, 0, 1);
  const double angular_band = elasticAngularSlack(rho, true,
        elastic_delta_phi_min_, elastic_delta_phi_max_, encirclement_base_angular_slack_);
  const double e_phi_eff = angularDeadbandViolation(e_phi, angular_band);
  if (e_phi_eff > 0.0) {
    const double sign_phi = e_phi > 0.0 ? 1.0 : −1.0;
    const Eigen::Vector2d dphi_dq(−q.y()/(radius*radius), q.x()/(radius*radius));
    const double w_phi = encirclement_bearing_recovery_weight_ *
                         (in_execution_prefix ? prefix_bearing_recovery_boost_ : 1.0);
    bearing_cost   = w_phi * e_phi_eff * e_phi_eff;
    bearing_grad_p.head<2>() = w_phi * (2.0*sign_phi*e_phi_eff) * dphi_dq;
  }
}
costp        += wei_tracking_*(er*er+ez*ez) + bearing_cost;
gradp        += wei_tracking_*radial_height_grad + bearing_grad_p;
gradt        += (wei_tracking_*radial_height_grad + bearing_grad_p).dot(v-object_v_t);
grad_prev_t  += (wei_tracking_*radial_height_grad + bearing_grad_p).dot(-object_v_t);
```

| 问题 | 结论 |
|---|---|
| weight | `encirclement_bearing_recovery_weight` launch 默认 **0.08**（头文件成员默认 0.30，被 launch 覆盖为 0.08） |
| prefix boost | `prefix_bearing_recovery_boost_`，默认 2.0 |
| prefix 判据 | **true relative time**：`in_execution_prefix = prefix_execution_span_ > 0 && elapsed_t <= prefix_execution_span_ + 1e-9`，`elapsed_t` 由调用点传 `execution_prefix_elapsed + step*j`。不是 sample index、不是 trajectory fraction |
| cost/gradient 权重一致 | **一致**。`bearing_cost` 与 `bearing_grad_p` 都只乘一次 `w_phi`；最外层 `wei_tracking_` 只作用于 `radial_height_grad`，**bearing 项没有被再乘一次** |
| 是否重复乘 `wei_tracking_` | **否** |
| 是否启用 | **是**（`encirclement_bearing_recovery_weight_ = 0.08 > 0`，且 launch 对三机都设了该参数） |

注意 `angular_band` 的**取值范围**：`elasticAngularSlack(rho, true, elastic_delta_phi_min_(0.0),
elastic_delta_phi_max_(45°), encirclement_base_angular_slack_(20°))` —— 题述的
`band_phi(rho) = 20° + 25°·rho` 与参数集（min 0 / max 45° / base 20°）**不完全等价**，
真实取值由 `elasticAngularSlack` 实现决定，不是简单的 20+25ρ。

---

## 9. Team Joint P/T/Ψ 的真实 runtime 状态

### 9.1 真实 argv（canonical runner）

`runs/20260919_205929_346132/roslaunch_argv.txt`：

```
enable_team_visibility_optimizer:=true
enable_joint_topology_coordination:=true
enable_joint_pt_optimization:=true
```

启动自证：

```
[team-vis-opt-config] enabled=1 joint_pt=1 runtime_ready=1 scene_status=OK horizon=1.500
```

**结论：Team Joint P/T 当前 runtime enabled 且已构造。**

### 9.2 变量语义

| 符号 | 真实含义 | 源码位置 |
|---|---|---|
| **P** | 三条候选 MINCO 的**内点位置**（`DecisionLayout::position_dim[drone]`） | `team_visibility_optimizer.cpp` `DecisionLayout` |
| **T / tau** | 各段**时长**（`time_dim`，虚拟时间 `virtual_times` + `RealT2VirtualT`） | 同上 |
| **Ψ**（原设计） | **相机 yaw 联合优化** | **已从 production 源码移除**：`enable_joint_yaw_optimization` / `team_yaw_trust_radius` / `optimize_yaw` 在 `ros_ws/src` 全域 **0 命中**（`*.cpp/*.h/*.launch/*.xml`），launch 与 runner 参数源也没有这两项。`run_on.sh` 内注释即写明 "Joint yaw 已退出 production；执行 yaw 始终由 target-facing 语义派生"。因此**当前不存在 Ψ 决策维度** |
| **φ** | **UAV 绕目标的方位角**（`atan2(q.y,q.x)`，见第 7/8 节），与相机 yaw 是两个不同的量 | `poly_traj_optimizer.cpp` |

> 修正：本报告早期版本按旧 argv（`runs/20260919_171153_221343` 里确有 `enable_joint_yaw_optimization:=false`、
> `team_yaw_trust_radius:=0.10`）写成"Ψ implemented but not enabled"。以**当前源码**为准，
> 该链路已被整体删除，正确表述是 **"Ψ 不存在于当前 production"**。最新 run 的真实 argv
> 也**不含**任何 yaw 项（`grep joint_yaw|yaw_trust` 命中 0）。

### 9.3 完整调用链

```
Local candidate (per-drone bundle)
  → topology_bundle 订阅 (bundleCallback)
  → TopologyCoordinatorCore::select()  → selection.joint
  → enumerateTeamTopologyTuples()      → max 27 tuples
  → attemptTeamOptimization(tuple_selection, early_legacy, /*early_joint_primary=*/true)
      → team_optimizer_->setEffectiveHorizon(H_eval)
      → team_optimizer_->optimize(inputs, activation_time, environment, warm_start)
  → proposal → planner ACK → commit → common activation
```

决策维度（`DecisionLayout`，`team_visibility_optimizer.cpp`）：
`position_dim + time_dim`（**无 yaw_dim** —— Ψ 链路已移除）。

日志实证（本次 run）：`EARLY_JOINT_OPT_ATTEMPT=464`、`EARLY_JOINT_OPT_FAIL=464`、
`TEAM_VIS_OPT_ATTEMPT=0`、`multiview-proposal=0`、`TRANSACTION_JOINT_SUCCESS=2`、
`TRANSACTION_ACTIVATED=0`、`TRANSACTION_JOINT_REJECTED=524`。

即：**attempt 真实发生，但 464 次全部失败**，主导原因 `NO_PRIMARY_JOINT_PROPOSAL_WITHIN_BUDGET`
（该字符串在 `EARLY_JOINT_FALLBACK_TO_LOCAL` 日志处，含义是 tuple 枚举在 `joint_latency_.budget()`
内没有产出可采纳的 primary proposal）。

第一条真实失败原因（日志直接给出）：

```
[EARLY_JOINT_OPT_ATTEMPT] ... H_CONFIGURED=2.000 H_COMMON=1.967 H_EVAL=1.967
                          horizon_clamped_to_common_prefix=1
[TEAM_VIS_OPT_FALLBACK] generation=3 hypothesis=0 reason=INITIAL_METRICS_INVALID
                        objective_before=inf objective_after=inf
```

`INITIAL_METRICS_INVALID` 由 `result.reason = "INITIAL_" + validation_reason;` 拼出
（`validation_reason` 初值 `"METRICS_INVALID"`），即在
`if (!result.before.valid || !validateHardConstraints(...))` 处就返回了。
`result.before.valid` 为 false 的判据是：

```cpp
metrics.valid = std::isfinite(metrics.objective) && (gradient == nullptr || gradient->allFinite());
```

而 `metrics.objective` 变 inf 的一条明确路径是 `encirclementGeometryCost` 在
`!geometry.finite` 时返回 `inf`，其上游是：

```cpp
const auto geometry = circularGapGeometry(relative);
if (!geometry.finite) return TeamVisibilityMetrics();     // ← 直接产出 invalid
```

`circularGapGeometry` 要求三机相对目标的水平距离平方 `> 1e-12` 且全部有限。

**J_vis 与 T/Ψ 的关系**：`H_EVAL` 已被正确 clamp 到公共前缀（`H_CONFIGURED=2.000 → H_EVAL=1.967`），
说明 horizon 端到端统一**在 runtime 生效**。

---

## 9.4 符号总表（第 3/5/6/7/8 节所有公式中的符号）

| 符号 | 含义 | 真实来源 / 值 |
|---|---|---|
| `p` | 轨迹采样点位置（世界系） | MINCO 系数 × `beta0` |
| `v` | 轨迹采样点速度 | MINCO 系数 × `beta1` |
| `q` | `p − object_p_ − object_v_t·t`（相对目标的**预测后**位置） | `trackingGradCostP` |
| `seed` | `object_rotation * relative_tracking_p_`（编队种子偏移） | 同上 |
| `radius` | `‖q.head<2>()‖` | 同上 |
| `desired_radius` | `‖seed.head<2>()‖` = 该机编队半径 `R*` | 同上 |
| `er` | `band_error(radius − desired_radius, observation_radius_band_)` | `observation_radius_band_ = 0.35` |
| `ez` | `band_error(q.z() − seed.z(), observation_height_band_)` | `observation_height_band_ = 0.20` |
| `band_error(e,w)` | `copysign(max(0, \|e\|−w), e)`（死区带） | 同上 |
| `φ` | `atan2(q.y(), q.x())`：UAV 绕目标方位 | 同上 |
| `φ*` | `atan2(seed.y(), seed.x())`：种子方位 | 同上 |
| `e_phi` | `wrapToPi(φ − φ*)` | 同上 |
| `ρ` | `clamp(risk_state.risk, 0, 1)` | `elastic_risk_profile_` |
| `angular_band` | `elasticAngularSlack(ρ, true, min=0, max=45°, base=20°)` | **不是** 20°+25°ρ |
| `e_phi_eff` | `angularDeadbandViolation(e_phi, angular_band)` | 同上 |
| `w_phi` | `encirclement_bearing_recovery_weight_ × (prefix ? boost : 1)` | `0.08` × `2.0` |
| `dphi_dq` | `(−q.y()/radius², q.x()/radius²)` | 同上 |
| `W` / `wei_tracking_` | tracking 权重 | `optimization/weight_tracking`（launch 100.0） |
| `c` / `clearance` | 观察者→目标线段与障碍的最近余隙 | `querySegmentClearance` |
| `c_in` | `inner_clearance` | 静态 LOS：`static_los_margin_`；FOV：`0` |
| `c_out` | `outer_clearance` | 静态 LOS：`static_los_margin_ + directional_static_risk_transition_`；FOV：`0.25×half` |
| `width` | `c_out − c_in` | `directionalClearanceRisk` |
| `κ` / `β` | deep-risk 参数 | `directional_visibility_deep_risk_kappa_ = 1.0`、`_beta_ = 2.0` |
| `R_static/R_dynamic/R_fov` | 三类 deep-risk 风险值 | `directionalClearanceRisk(...).value` |
| `w_vis` / `weight_visibility_` | 方向性可见性权重 | `optimization/weight_visibility` 默认 **20.0** |
| `v_i` | 第 i 机的二值可见性 ∈ {0,1} | `binary_values[sample][drone]` |
| `Q2` | `v0v1+v0v2+v1v2 − 2v0v1v2` | `TeamVisibilityOptimizer::q2` |
| `b0` | `(1−v0)(1−v1)(1−v2)` | `TeamVisibilityOptimizer::blackout` |
| `θ_i` | `atan2(q_i.y, q_i.x)`：第 i 机方位 | `circularGapGeometry` |
| `gaps[j]` | 排序后相邻方位差（末项 +2π） | 同上 |
| `θ_min` | 最小允许相邻间隔 | `preferred_view_angle_deg` = **25°** |
| `θ_max` / `theta_gap_max` | 最大允许空扇区 | `max_circular_gap_deg` = **170°** |
| `J_gap` | `Σ_j ([g_j−θ_max]_+² + [θ_min−g_j]_+²)` | `encirclementGeometryCost` |
| `w_gap_local` | Local 空扇区权重 | `encirclement_local_gap_weight` 默认 **100.0** |
| `w_gap_team` | Team 空扇区权重 | `/encirclement_geometry/joint_gap_weight` 默认 **4.0** |
| `δ_ij` | `\|wrapToPi(θ_i − θ_j)\|` | `multiviewQuality` |
| `bad` / `good` | 多视角间隔下/上界 | `MultiviewPolicy{bad=25°, good=60°}` |
| `u_ij` | `clamp((δ_ij−bad)/(good−bad), 0, 1)` | 同上 |
| `q_ij` | `u²(3−2u)`（smoothstep） | 同上 |
| `Q_dir` | `Σ q_ij / 3` | 同上（**仅 telemetry**） |
| `J_dir` | `Σ (1−q_ij)² / 3` | 同上（**仅 telemetry**） |
| `phi0` | 合围相位基准 | 初始三机方位角的**圆均值** |
| `slot_i` | 第 i 机的 120° 槽位索引 | 3! 排列中按最小初始方位位移选定后固定 |
| `R*_i` | 第 i 机编队半径 | `offsets_[i].head<2>().norm()` |
| `z*_i` | 第 i 机编队高度 | `offsets_[i].z()` |
| `H_configured` | 配置的 joint 评估视野 | `team_optimizer_params_.horizon`（launch 2.000） |
| `H_common` | 三机真实公共已验证前缀 | `current_available_end − evaluation_start` |
| `H_eval` | 实际使用的视野 | `min(H_configured, H_common)`（实测 1.967） |
| `m_s` | LOS 观察半空间法向 | `−sinα·e + s·cosα·n_R` |
| `e` / `n_R` | 目标→blocker 单位向量 / 其右法向 | `n_R = (e_y, −e_x)` |
| `α` | 阴影视半角 | `asin(min(1, r/d))`，`r = conflict.radius` |

**补充证据（本轮追加核实）**：

- `enable_side_local_astar_repair_` 默认 **true**（`nh.param("manager/enable_side_local_astar_repair",
  enable_side_local_astar_repair_, true)`），且 launch 链中**未覆盖**该参数 ⇒ A* 修复通道**可用**。
- `LocalSfcPlane::source` 的读取者**全部在 planner_manager 内**（`:2803` 计数、`:7942` A* 重建时保留
  LOS 平面、`:8301` MINCO 输入前计数），**optimizer 侧 0 命中** ⇒ 再次佐证第 5.3 节结论。
- LOS 平面会被显式送进 optimizer：`ploy_traj_opt_->setCandidateLocalSfc(active_local_sfc)`
  之前有 `stage=MINCO_INPUT` 的 LOS 平面计数日志 —— 即**"送进去了，但求解器不消费"**，
  与本报告结论一致（送进去 ≠ 成为硬行）。

---

## 10. 每个模块的实际作用

| 模块 | 实际作用 |
|---|---|
| `planner_manager.cpp::reboundReplan` | 单个 planning cycle 总控：raw LOS 扫描、conflict 描述符、SIDE 生成、finalize |
| `DynamicRiskInfo` / `evaluateDynamicRisk` | 动态 BODY 硬/软判定（`hard_clearance = body + obstacle`） |
| `queryStaticLosClearance` / `StaticLosWitness` | 静态 LOS 遮挡检测与 witness |
| `ConflictDescriptor` | 统一承载 BODY/LOS 两类证据 + `reason_mask`；**不是 conflict list** |
| SIDE 生成（`attempt_side`） | 固定 +1/−1 两侧，产出 `SIDE_PLUS/SIDE_MINUS` |
| `side-static-aware` / `side-astar-repair` | SIDE guide 的静态可行性预检与 A* 局部修复 |
| `LocalSfcPlane`（LOS_OBSERVATION_SIDE） | 观察半空间；在 production 中只作**后验采样否决器** |
| `candidate_local_sfc_planes_` | 唯一读取者 `evaluateCandidateLocalSfcMaxViolation()`（25 采样点） |
| `addDirectionalVisibilityGradCost2CT` | Local 连续可见性软代价（含 deep-risk），**启用** |
| `encirclementGeometryCost` / `multiviewQuality` | Local+Team 的空扇区 cost（启用）与多视角质量（**仅 telemetry**） |
| `TeamVisibilityOptimizer::optimize` | Team Joint P/T(+Ψ) 联合优化；当前 attempt 全失败于 `INITIAL_METRICS_INVALID` |
| `cooperative_viewpoint_core` | 合围 seed（`phi0` 圆均值 + slot 排列 + 每机 R/z 偏移），**soft reference** |
| `circularGapGeometry` | 环形间隙几何；`!finite` 时使 team metrics 失效（失败链起点） |

---

## 11. implemented vs runtime-active 对照表

| 能力 | implemented | reachable | runtime enabled | 实际生效 |
|---|---|---|---|---|
| Local 方向性可见性 cost（含 deep-risk） | YES | YES | YES（`w_vis=20.0`） | **YES** |
| Local circular gap cost | YES | YES | YES（`w=100.0`） | **YES** |
| bearing recovery（φ 偏差） | YES | YES | YES（`w=0.08`，boost 2.0） | **YES** |
| Team Joint P/T | YES | YES | YES（`joint_pt=1`） | **attempt 464 次全部失败** |
| Team Joint Ψ/yaw | **NO —— 链路已从源码删除** | — | — | **不存在**（`enable_joint_yaw_optimization`/`team_yaw_trust_radius`/`optimize_yaw` 在 `ros_ws/src` 全域 0 命中；最新 run 真实 argv 无 yaw 项） |
| Team circular gap cost | YES | YES | YES（`joint_gap_weight=4.0`） | 因 optimize 未成功而**未进入被采纳的解** |
| K2-continuity 连续 cost | YES | YES | **NO**（`k2_continuity_weight=0.0`） | NO |
| blackout 连续 cost | YES | YES | **NO**（`blackout_weight=0.0`） | NO |
| multiview Q_dir / J_dir | YES | YES | 计算但**不进 objective** | **仅 telemetry** |
| binary K2/All3/blackout/weakest metric | YES | YES | YES | **selector 使用** |
| LOS observation plane 作 SCP 硬行 | **NO**（SCP 不读该成员） | — | — | **NO** |
| `enable_candidate_hard_corridor_scp` | YES | YES | **NO**（真实 argv `false`） | NO |
| A* side repair | YES | YES | 由 `enable_side_local_astar_repair_` 控制 | 触发于 SIDE guide 静态不可行 |
| space-time A* | **NO** | — | — | — |

---

## 12. 与给定理解 A–G 的逐条核对

| 编号 | 给定理解 | 审计结论 |
|---|---|---|
| **A** | A* 不是"静态威胁处理器"，而是 LEFT/RIGHT SIDE guide 被静态地图阻塞时的局部修补器 | **正确**。入口条件 `enable_side_local_astar_repair_ && astar_base_valid && grid_map_`，`trigger_reason=STATIC_INFEASIBLE`；输入含 `collision_start/end` 与 rejoin 候选 |
| **B** | 多个静态障碍不会各生成一套 L/R；A* 在整个 local static occupancy 中一次处理 | **正确**。L/R 固定两条；A* 在 `grid_map_` 上做一次搜索，`astar_accept` 统一验收 |
| **C** | 当前多 threat 通常不会生成 2M 或 2^M 条轨迹 | **正确**。固定 `NOMINAL + SIDE_PLUS + SIDE_MINUS = 3` 条；`order[2]` 只是尝试顺序 |
| **D** | BODY 和 LOS 的 LEFT/RIGHT 参考 frame 不同 | **正确**。BODY：`(local_target_pt − start_pt) × e_z`；LOS：`e=(C−T)/‖·‖`、`n_R=(e_y,−e_x)`、`m_s=−sinα·e+s·cosα·n_R` |
| **E** | Local visibility cost 与 raw LOS topology authority 是两套机制 | **正确**。gate 是 `visibility_topology_trigger`，只由 BODY 描述符或 raw LOS 描述符给出；`soft-visibility-continuous-only` 日志明确 `action=NO_TOPOLOGY_AUTHORITY`；`J_vis ≠ 0` 不会创造 SIDE |
| **F** | 120° 是 seed/reference，不是 hard constraint | **正确**。`encirclementAngles = phi0 + 2π·slot/3` 只产出 reference；`validateEncirclementLimits` 仅构造期校验参数合法性；gap 约束是 soft cost（`theta_min=25°`、`theta_max=170°`），硬门只出现在 optimizer 内部的几何可行性检查上 |
| **G** | J_vis 越小越好；Q2/Q_dir 越大越好 | **需分段**。`J_vis = w·R²`、`gap_cost = Σ(hi²+lo²)`、`J_dir = Σ(1−q)²` 均为 cost（越小越好）；`Q2`、`Q_dir = Σq/3`、`binary_k2/all3/weakest` 为 quality（越大越好）。**符号方向与代码一致** |

### 12.1 与理解不一致 / 需要修正的地方

1. **"LOS plane 进入 MINCO-SCP-OSQP"不成立**：它在 production 中只是后验采样否决器（25 点）。
2. **BODY 与 LOS 不能各自独立选 side**：一条候选只有一个 `side`；优先级是"BODY hard 冲突 → 用 BODY frame，LOS 平面另行附加"，否则用 observation frame。无法表达 `s_body=+1` 且 `s_los=−1`。
3. **无 active conflict list**：BODY 一个 witness、LOS 一个 witness，`reason_mask` 两位。
4. **`metrics.objective` 里 `deviation_weight*deviation_cost` 被重复计入两次**（源码级，未修改）。
5. **multiview 不进 objective**：`Q_dir`/`J_dir` 仅 telemetry；当前 team objective 只含 K2/accumulated/deviation/jerk/gap 五项。
6. **K2-continuity 与 blackout 连续项权重为 0**，当前不参与优化。
7. **bearing 的角隙不是 20°+25°ρ**：真实由 `elasticAngularSlack(rho, true, min=0, max=45°, base=20°)` 决定。
8. **Team Joint 当前是"已启用但 100% 失败"**：464/464 attempt 失败，首因 `INITIAL_METRICS_INVALID`，其上游是 `circularGapGeometry` 非有限使 `metrics.valid=false`。不是"没开"。
9. **Ψ/yaw 链路已从 production 源码整体删除**（不是"实现了但没开"）：`enable_joint_yaw_optimization`、`team_yaw_trust_radius`、`optimize_yaw` 在 `ros_ws/src` 全域 0 命中；launch 与参数源均无此二项；最新 run 真实 argv 不含任何 yaw 项。执行 yaw 由 target-facing 语义派生。
10. **LOS 平面确实被送进 optimizer，但求解器不消费它**：`setCandidateLocalSfc(active_local_sfc)` 之前有 `stage=MINCO_INPUT` 的 LOS 计数日志，A* 重建时也专门保留 LOS 平面（`:7942`）——所以问题不是"没传进去"，而是"传进去后没有任何 hard-row 构造读它"。

---

## 13. 最终可信流程图

```
reboundReplan()
  │
  ├─ epoch = local_activation_time_           (唯一时间原点)
  │
  ├─ raw LOS 扫描 [0, max(authority_horizon, prediction_horizon)]
  │    ├─ static : queryStaticLosClearance ≤ visibility_occlusion_margin_
  │    └─ dynamic: segmentIntersectsVerticalCylinder(r = 0.5*max(scale)+margin)
  │    → raw_los_occlusion_enter_rel_ / exit_rel_ / observed_
  │
  ├─ current LOS（观测者 = 当前 odom，无条件执行）
  │    → current_raw_los_blocked_  ⇒ enter_rel_ = 0.0
  │
  ├─ nominal MINCO (LBFGS 软代价: tracking + directional-visibility(w=20) + gap(w=100) + bearing(w=0.08))
  │
  ├─ conflict descriptor
  │    reason_mask ∈ {BODY_SAFETY, LOS_OCCLUSION}
  │    BODY 槽：static body support 或 dynamic risk(min_distance 最小者)
  │    LOS  槽：static LOS witness；dynamic LOS 若更早则替换(earliest-time)
  │    → 1 BODY witness + 1 LOS witness（无 list）
  │
  ├─ SIDE 权限门：risk_candidates 开启 且 (BODY 描述符 或 raw LOS 描述符)
  │
  ├─ frame 选择
  │    body_hard_conflict ? BODY/path frame : LOS observation frame
  │    （LOS observation frame 若静态不可行 → 降级为 path frame）
  │
  ├─ 固定两条侧向候选
  │    attempt_side(+1) → SIDE_PLUS
  │    attempt_side(-1) → SIDE_MINUS
  │       ├─ seed（side × kCandidateSideOffset × side_direction，制导窗 kGuidanceWindowSeconds=0.45）
  │       ├─ 静态预检 (side-static-aware)
  │       │    └─ 静态不可行 → A* 局部修复 (astar_accept = base_valid
  │       │         && path_static_free && path_side_valid
  │       │         && repaired_side_valid && local_sfc_build_valid)
  │       ├─ LOS 观察面附加为 LocalSfcPlane{LOS_OBSERVATION_SIDE}
  │       │    normal = m_s, point = (T_x,T_y,0), clearance = 0
  │       │    active = 真实遮挡区间, world_time_anchored = true
  │       │    ✗ 不进入 SCP 硬行 → 仅后验 25 点采样
  │       ├─ MINCO / (可选) hard-corridor SCP  ← 真实 argv 为 false
  │       └─ 动态 / 集群 / 局部 SFC 校验
  │
  ├─ NOMINAL + SIDE_PLUS + SIDE_MINUS  → CandidateSetOutput（含 capture_output 路径）
  │
  ├─ finalizeCapturedCandidates()
  │    ├─ 逐候选：handoff / dynamics / static(rolling bounded) / Local-SFC(retimed) / execution trajectory
  │    ├─ 排序：safety_class → visibility(selector metrics) → geometry
  │    └─ commit → setLocalTrajFromOpt → publish
  │
  └─ [team 通道开启时] bundle → topology coordinator
       → TopologyCoordinatorCore::select
       → enumerateTeamTopologyTuples (≤27)
       → attemptTeamOptimization(early_joint_primary=true)
            setEffectiveHorizon(H_eval = min(H_configured, H_common))
            team_optimizer_->optimize(P, T, yaw)
              ← 当前在 !result.before.valid 处返回
                reason = "INITIAL_" + "METRICS_INVALID"
                上游：circularGapGeometry 非有限 ⇒ metrics.valid = false
       → 若成功：proposal → ACK → commit → common activation
```

---

## 14. 本次审计使用的 runtime 证据清单

| 证据 | 路径 |
|---|---|
| 真实 argv | `runs/20260919_205929_346132/roslaunch_argv.txt` |
| Team Joint 启动自证 | 同目录 `roslaunch_stdout.log` → `[team-vis-opt-config]` |
| attempt/成功/失败计数 | 同上 → `EARLY_JOINT_OPT_ATTEMPT=464` / `_FAIL=464` / `TRANSACTION_JOINT_SUCCESS=2` / `TRANSACTION_ACTIVATED=0` |
| 失败首因 | 同上 → `[TEAM_VIS_OPT_FALLBACK] reason=INITIAL_METRICS_INVALID objective_before=inf` |
| horizon 统一生效 | 同上 → `H_CONFIGURED=2.000 H_COMMON=1.967 H_EVAL=1.967 horizon_clamped_to_common_prefix=1` |
| hard-SCP flag | 同上 → `enable_candidate_hard_corridor_scp:=false` |

本轮**未运行任何仿真**；以上 runtime 证据来自既有 runs 目录（只读）。
