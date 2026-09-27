# Feedback 062 - Static Occlusion / Unified Observation Topology Design Audit

日期：2026-09-14  
项目：`/home/bob/ALP/egov2_fc65423_constvel`  
性质：只读源码审计与方案设计  

## 0. 范围与状态

本轮只读取当前 ALP 工作树、Feedback59-61、当前 planner/traj-opt/formation 源码和已有测试/运行证据。没有修改 production source、message、CMake、launch、参数或 RViz 配置；没有启动新的 ROS/Gazebo/RViz/FULL run，也没有运行 OFF/A-B。工作树中本轮没有新增生产修改。

事实基线仍是 Feedback61 的健康架构：

```text
committed validated Local prefix
 -> common future frontier
 -> background Joint future tail
 -> hard preflight / CAS / proposal / 3ACK
 -> scheduled team activation
```

本报告不重新评价 Feedback61 的 atomic activation、Local no-op 或 RViz authority 修复；这里只审计静态 LOS 遮挡、SIDE、A*、Local-SFC、MINCO/Elastic 和 Early Joint 的职责边界。

## 1. 结论摘要

当前系统并非完全没有静态 LOS 触发：`sharedVisibilityTopologyTrigger()` 已将 elastic/directional static LOS、dynamic LOS 和 FOV risk 统一作为 topology trigger 的连续证据。`planner_manager.cpp:5750-5815` 还把第一个 shared-risk trajectory sample 写入 `risk.conflict_time`，因此静态 LOS 风险可以提前进入现有 NOMINAL/LEFT/RIGHT candidate pipeline。

但这条链不是“静态遮挡物驱动的观察拓扑”。SIDE 的参考方向仍由 trajectory tangent 或 start-to-local-target 方向构造：`planner_manager.cpp:5872-5913`、`buildFreshMovingInitializer():712-722`、`PolyTrajOptimizer::setCandidateSideBias():7417-7443`。静态 LOS witness 只在 optimizer diagnostics 中作为 primitive index 使用；manager 丢弃 witness，`DynamicRiskInfo` 仍是 dynamic obstacle schema，SIDE 仍读取 `risk.obstacle_position` 和 `risk.obstacle_id`。

更关键的是，Local-SFC 只在 SIDE 初始参考静态不可行并且 A* repair 成功时构造。`planner_manager.cpp:6373-6378` 的 A* 入口要求 `astar_base_valid`，而 `LocalSfcPlane` 的 boundary scan 位于 repair path 之后 `planner_manager.cpp:6950-7070`。静态 LOS 被遮挡但 UAV body path 不撞 map 时，SIDE 可以是 static-free，A* 不启动，Local-SFC 为空，MINCO/SCP 只受到 SIDE/J_vis/Elastic 等软项影响。没有 observation-side hard/semi-hard corridor。

因此用户观察的“静态柱不挡 UAV 路径但挡 LOS 时 A* 常不启动、plane_count=0、UAV 可能选错侧并左右摆动”与当前源码语义一致，但需要精确表述为：这是现有模块职责的结果和语义缺口，不是现有 collision A* 本身的异常。

```text
CURRENT_STATIC_OCCLUSION_PIPELINE:
MINCO/Elastic static LOS risk
 -> shared visibility trigger
 -> existing NOMINAL/SIDE_PLUS/SIDE_MINUS dispatch
 -> path-tangent side seed
 -> static precheck
 -> A* only if side seed collides with inflated map
 -> Local-SFC only for repaired A* path
 -> MINCO/SCP with soft side/visibility costs
 -> final hard preflight
 -> local candidate bundle / optional Joint tuple
```

## 2. Required audit answers

```text
STATIC_LOS_CAN_TRIGGER_SIDE: PARTIAL
STATIC_LOS_TRIGGER_USES_REAL_OCCLUDER_IDENTITY: NO
STATIC_LOS_TRIGGER_STILL_DEPENDS_ON_DYNAMIC_CONFLICT_STATE: PARTIAL

CURRENT_SIDE_REFERENCE_FRAME: PATH_TANGENT / START_TO_LOCAL_TARGET LEFT-RIGHT
SIDE_IS_TRUE_STATIC_OCCLUSION_TOPOLOGY: NO

LOCAL_SFC_REQUIRES_ASTAR_REPAIR: YES (current implementation)
STATIC_LOS_ONLY_CAN_CREATE_LOCAL_SFC: NO
OBSERVATION_SIDE_HAS_HARD_OR_SEMIHARD_CORRIDOR_WITHOUT_ASTAR: NO

SOFT_VISIBILITY_WITHOUT_TOPOLOGY_AUTHORITY_GAP: CONFIRMED

GLOBAL_TOPOLOGY_STRATEGY_RECOMMENDED: YES, GLOBAL AVAILABLE + LOCAL ACTIVATION
```

“PARTIAL” 的含义是：static LOS risk 已经能进入 shared trigger，并在有有效 nominal trajectory、valid diagnostics、`enable_risk_triggered_candidates_` 和 `obj_predictor_` 时把 `risk.triggered` 置为 true；但这不是独立于 dynamic risk 的静态遮挡入口。`evaluateDynamicRisk()` 在没有 objects 时返回 `valid=true`，但在 predictor 存在而 prediction 缺失时直接返回 invalid（`planner_manager.cpp:4061-4070`）。SIDE candidate 仍要求 `trial_risk.valid`、`nominal_result.risk.valid` 和绝对 dynamic clearance（`6290-6338`）。因此静态 LOS 能触发的前提仍被 dynamic-risk container 和 candidate pipeline 所约束。

## 3. 真实调用链与断点

### 3.1 Static LOS risk -> trigger

`PolyTrajOptimizer::directionalVisibilityGradCostP()` 对静态 LOS 调用 `StaticLosGeometry::querySegmentClearance()`，得到 clearance、梯度和 `static_witness`，随后转换为 directional risk/cost。Elastic path 在 `elasticTrackingRisk()` 中同样计算 static LOS risk；diagnostics 保存 `elastic_static_los_risk_max`、`directional_static_los_cost_max` 和 first-support sample。

`sharedVisibilityTopologyTrigger()` 只检查六个 risk/cost support 是否为正数（`local_visibility_preference.h:32-53`），不检查 blocker identity。manager 在 `planner_manager.cpp:5750-5812` 调用它，并将 first-support time/position/target 写入 `DynamicRiskInfo` 的字段。

### 3.2 Trigger -> SIDE

manager 在 `planner_manager.cpp:5773-5815` 仍先调用 `evaluateDynamicRisk()`，然后把 visibility trigger 映射为 `risk.triggered=true`。随后 `planner_manager.cpp:5872-5913` 用 nominal velocity tangent 构造 `local_side_direction = tangent x ez`，用 `risk.obstacle_position - risk.trajectory_position` 决定 preferred side，再用 `local_target_pt-start_pt` 的横向方向把 escape direction 映射为 PLUS/MINUS。

当 visibility witness 生效时，`risk.obstacle_position` 实际被写成 shared-risk sample 的 target position（`5798-5801`），不是 static primitive 的 cylinder/wall center、ray hit point 或 surface normal。若 dynamic predictor 同时存在，`risk.obstacle_id` 和 position 仍由距离最近的 dynamic object 语义填充（`4061-4111`）。这就是 hidden coupling：trigger 名义上支持 static LOS，但 SIDE 的 obstacle reference 仍不是静态遮挡物。

### 3.3 SIDE -> A*

SIDE seed 由 `buildFreshMovingInitializer()` 生成。其 side displacement 为：

```text
side_dir = normalize((local_target - start) x ez)
q(u) = start + u(local_target-start)
       + side_bias * side_offset * sin(pi*u) * side_dir
```

这是 path-left/right，不是 occluder-left/right。SIDE seed 先以不同 offset 做 static precheck（`6312-6323`）。只有 backoff 失败且 `astar_base_valid && grid_map_ && side_a_star_` 时才进入 `planner_manager.cpp:6373-6380` 的 local A* repair。`astar_base_valid` 的选择还依赖 trial dynamic validity（`6290-6305`），虽在 visibility trigger 分支使用绝对 dynamic clearance 而不要求相对改善。

### 3.4 A* -> Local-SFC

A* repair 产生 raw path、static occupancy audit、side semantic audit、simplified path 和 rejoin interval。Local-SFC boundary scan 随后在 `planner_manager.cpp:6950-7070` 对 simplified A* guide 的每个内部点扫描 inflated grid，生成 `LocalSfcPlane{normal, point, clearance, active_start, active_end}`。这些 planes 由 `PolyTrajOptimizer::setCandidateLocalSfc()` 传入 SCP，且在 optimizer 中作为 active time half-space 进行 violation 检查和 QP/SCP 约束。

如果 SIDE seed 本身 static-free，repair 分支不执行，`local_sfc_planes` 保持空；这不是 `SFC_BUILD_FAILED`，而是当前实现意义上的 `SFC_NOT_REQUIRED`。已有 Feedback2/49 证据也确认 zero-plane handoff 是合法 collision corridor 语义。

### 3.5 Local-SFC -> MINCO/SCP -> Joint

直接 SIDE 候选没有 Local-SFC hard corridor；optimizer 仍可使用 candidate side bias、candidate side region、candidate preservation/corridor cost，以及 static LOS MINCO cost。`candidateSideRegionViolation()`、`candidateSideCorridorViolation()` 和 `candidatePreservationGradCostP()` 都返回 soft cost/gradient；真正的 sparse hard/semi-hard geometric rows 只有 `candidate_local_sfc_planes_`。

早期 Joint 只接收已标记 `joint_seed_valid`、`joint_seed_static_constructible`、`joint_seed_local_sfc_valid` 的 N/L/R descriptor，并枚举最多 27 个三机 tuple（`team_planning_context.h:208-261`、`multi_uav_topology_coordinator.cpp:2386-2483`）。descriptor 目前携带 candidate kind/hypothesis/generation/dynamic clearance/predicted camera time，没有 blocker identity 或 side reference frame。Joint 因而可以组合现有 N/L/R，但不能知道 L/R 是同一个 static occluder 的哪一侧。

## 4. SIDE 左右几何语义

### 当前方案 A：path tangent side

当前 SIDE 的数学语义是：以 `t = normalize(local_target-start)` 或 nominal trajectory tangent 为前向，`n = t x ez` 为横向基；PLUS/MINUS 是沿 `+n/-n` 的 sinusoidal lateral offset。A* repair 后的 side normal 也从 path segment tangent 构造（`planner_manager.cpp:6470-6480`、`6980-6995`）。

这对“动态冲突时从当前运动方向旁边绕开”是自然的，也最容易复用现有 seed/A*/MINCO。但对静态遮挡不充分：例如 path 沿 +X，柱子位于 observer-target LOS 的 +Y 侧，而 target 在柱子 +X 方向；path-left/right 与绕柱的 observation-left/right 可能重合，也可能在 target/observer 相对几何变化时相反。path side 只表示轨迹偏移，不保证减少 cylinder-shadow crossing。

### 推荐方案 B：target-occluder observation side

对 cylinder/wall 先得到 blocker 的 XY witness。令 `c` 为 blocker reference/hit point，`t` 为同一风险 sample 的 target，构造：

```text
d_obs = normalize((t_xy - c_xy))
n_obs = perp(d_obs) = (-d_obs.y, d_obs.x)
side_sign(p) = sign((p_xy - c_xy) dot n_obs)
```

两条观察拓扑是 `side_sign=+1` 和 `side_sign=-1`。对于 cylinder，半空间/软走廊应约束 observer 在 blocker 的一侧并在 occlusion core 期间保持该符号；对于 oriented wall，先在 wall local frame 选出与 target-facing face 相切的两个 side half-space。若 `t_xy-c_xy` 退化，使用 ray hit normal 的水平切向；再退化才回退到 path tangent。

方案 B 更符合恢复 LOS 的物理意义，也能把同一 blocker identity 传给 A* same-topology repair 和 Joint tuple。方案 A 更稳定、改动小，适合作为 fallback；最小实现应先保留 A 作为没有 witness 的 fallback，而不是把 A 宣称为静态 occlusion semantics。

## 5. 当前 visibility authority 表

| Mechanism | Trigger | 改变变量 | 能否改变 topology | 能否保持 side | Hard/soft | Local/team | 无 body collision 时 |
|---|---|---|---|---|---|---|---|
| shared risk support | elastic/directional static LOS、dynamic LOS、FOV > numerical zero | dispatch | 间接，触发 N/L/R 枚举 | 否 | trigger only | Local | 可以触发，但需现有 predictor/pipeline |
| SIDE initializer | risk trigger 或 dynamic candidate | MINCO seed P | 可以给出 L/R 初值 | 只在 tangent frame | seed/soft | Local | 可以 |
| candidate side bias/region | SIDE candidate optimization | interior P | 不决定拓扑 | 软保持 | soft | Local | 可以 |
| J_vis / static LOS MINCO | static LOS clearance | P/T gradient | 不能可靠跨越遮挡 shadow | 否 | soft | Local | 可以，但不保证绕到正确侧 |
| Elastic static LOS | LOS clearance risk band | P/T gradient/support | 不能 | 否 | soft | Local | 可以 |
| collision A* | inflated body occupancy on SIDE path | guide path | 可以修 body homotopy | 只保持 path-side audit | geometric repair | Local | 仅在 SIDE body path static-infeasible 时 |
| Local-SFC | A* guide boundary scan | half-space rows/time interval | 不创造 topology，只约束 repair topology | 是 | hard/semi-hard | Local | 当前 static LOS-only 不生成 |
| candidate visibility ranking | binary candidate trace | candidate choice | 可以在 executable candidates 中切换 | 不提供 corridor | ranking | Local | 可以，但 candidate 本身可能缺少 occlusion corridor |
| TopologyCoordinatorCore | executable N/L/R tuples | team tuple | 可以组合 topology | 只比较 candidate kind | hard-safe + ranking | Team | 可以，但 blocker semantics 未携带 |
| TeamVisibilityOptimizer | P/T/yaw on fixed tail | future P/T/yaw | 不应跨 selected side | 当前无 blocker-side plane | objective + hard safety | Team | 可以，受 1.5 s horizon |
| final preflight | static/dynamic/swarm/current revision | admission | 只否决不安全解 | 不保持观察侧 | hard | Local/team | 是 |

## 6. “visibility 知道错了，但改不掉”是否成立

成立，断点是：

```text
static LOS witness/risk
 -> shared trigger
 -> path-tangent SIDE seed
 -> no body collision
 -> no A*
 -> no Local-SFC plane
 -> MINCO/SCP only soft J_vis/Elastic/side costs
 -> next rolling cycle may choose another tangent-side basin
```

前两箭头有真实代码支持；`directionalVisibilityGradCostP()` 甚至计算 primitive witness，但 manager 没有将其作为 blocker descriptor 传给 SIDE。中间三箭头也由 A* 和 Local-SFC 的条件直接支持。最后的左右翻转不是每周期必然发生，因为 accepted-state warm start、same-obstacle bookkeeping 和 coordinator history 会降低一部分翻转；但它们不是 observation-side hard authority。`TopologyCoordinatorCore` 的 `rapid_reversal` 在 `topology_coordinator_core.cpp:684-695` 只是诊断结果，不能阻止选择；history tie 保持只适用于相同 kinds/hypothesis 的覆盖条件（`607-625`），不等价于 blocker-side intent。

因此：J_vis 能把轨迹梯度推离当前 LOS 风险，但不能单独证明 trajectory 已从柱子另一侧绕出，更不能防止后续 replan 回到另一侧。这个 gap 可以解释“软成本存在但轨迹仍贴近错误侧/左右摆动”，但不能仅凭源码断言每个微停顿都由该 gap 造成。

## 7. Static blocker identity 的现状和最小 API

`StaticLosGeometry::querySegmentClearance()` 已有可选 `int *cylinder_index` 输出（`static_los_geometry.h:170-184`），并在 cylinders/walls 上返回统一 primitive index（walls 为 cylinder count + wall index）。因此“完全没有 witness”不准确；当前真正缺的是：

1. manager-facing API 没有请求 witness：`EGOPlannerManager::queryStaticLosClearance()` 调用时传 `nullptr`（`planner_manager.cpp:7513` 附近）。
2. `querySoftPenalty()` 同样丢弃 witness。
3. index 没有 primitive name/type、ray parameter/hit point、surface normal、earliest/maximum-risk time。
4. `OptimizationDiagnostics` 的 `static_witness`/`directional_grad_max_static_witness` 是 diagnostics，不是 candidate/topology input。

最小只读设计建议增加 `StaticLosWitness`：`valid, primitive_index, primitive_name/type, ray_s, hit_point, horizontal_normal, clearance, trajectory_time, target_time`。保留现有 `querySegmentClearance()` ABI 兼容 wrapper；新增 query overload 返回 witness。对 topology 激活，优先选：

```text
primary = earliest future blocker with risk support
fallback = worst clearance / maximum risk blocker
```

不要对场景中所有柱子枚举 L/R。每个 UAV 每个 topology cycle 只保留一个 primary blocker，必要时保留一个 competing blocker；三机 tuple 仍上限 27 或按 blocker-group 合并。

## 8. Observation Corridor 是否复用 Local-SFC

推荐复用 half-space 数学和 SCP handoff，但扩展语义，不复制第二套 SFC：

```text
LocalTopologyConstraint
  source = STATIC_COLLISION_CORRIDOR | STATIC_OCCLUSION_SIDE
  blocker_id / blocker_generation
  normal, point, clearance
  active_start, active_end
  side_sign / topology_token
```

具体建议：

- `LocalSfcPlane` 可以向后兼容地增加 `source` 和 blocker metadata；现有 collision planes 的数值语义不变。
- observation side plane 不应扫描整个 map。它应从 blocker geometry、target ray 和 current selected side 生成，只作用于 occlusion core 的 `[t_enter, t_exit]`。
- `t_enter` 取 first risk-support/first predicted LOS-degradation event；`t_exit` 取 candidate/nominal LOS recovery 后的 event，若没有 recovery 则取 candidate evaluation end。两者由 risk trace/clearance trace 决定，不添加固定秒数阈值。
- cylinder 的 side plane 应约束 signed lateral side，且保留足够 target-facing clearance；wall 使用 wall local frame 的 side face/tangent。
- A* body repair 仍只负责 static occupancy；observation constraint 不能被误解为 dynamic SFC，也不能把动态障碍硬塞进 Local-SFC。
- 约束只在 blocker core active interval 生效，并在 blocker identity/generation 改变或 LOS recovery 后退出，避免把整条 trajectory 锁死。

这既可由 MINCO/SCP 作为 hard/semi-hard half-space 检查，也可在 candidate preflight 中验证 side sign/LOS recovery；不需要新增 global planner、joint A* 或新 dynamic SFC。

## 9. Global topology 的正确含义

推荐：`GLOBAL_AVAILABLE + LOCAL ACTIVATION`。

统一 topology manager 可以被每个 planner cycle 调用，但只有以下事件存在时才生成 N/L/R：

- predicted static LOS blocker with positive support and valid witness；
- dynamic conflict witness；
- meaningful spatial FOV conflict that can be expressed as a position/topology relation。

不存在 risk 时只生成 NOMINAL。不能对森林中所有柱子永久枚举 L/R，因为那会产生 `O(number_of_obstacles * 2)` local seeds，再在三机组合中导致 candidate explosion，并把无关柱子引入 side history。

推荐的候选限流：每 UAV 选择 earliest-risk blocker 作为 primary；若 primary 与第二 blocker 的风险区间互斥，可在后续 cycle 再激活第二 blocker；同一 blocker 的 L/R/N 保持最多 3 个 local candidates；Joint 仍只组合每机当前 constructible candidates。

## 10. TopologyIntent persistence

当前 `last_candidate_kind_`/`active_candidate_kind_` 和 coordinator history 不是完整 intent。建议的语义对象：

```text
TopologyIntent {
  blocker identity + geometry generation
  side (N/L/R)
  first/last risk time
  creation planning generation
  active side constraint interval
  exit reason
}
```

创建条件：primary blocker witness + positive risk support + selected side candidate passes hard checks。延续条件：same blocker identity/generation、risk interval仍重叠、side corridor/LOS recovery尚未退出。退出条件：LOS risk support消失且 candidate trace恢复、blocker prediction/map generation变化、target geometry使 side frame退化、或 trajectory 已越过 blocker shadow exit。强制换 side 仅允许 hard safety invalidation、side corridor infeasible、或新 blocker 具有更早/更强 hard conflict；普通 Q_dir/geometry improvement 不足以换 side。

这不是固定“锁 1 秒”。它是 event/geometry-based persistence。hard safety 可以覆盖 intent，但必须记录 team/local reason；新 blocker 出现时创建新 intent，并显式结束旧 intent。该对象应作为 candidate metadata 传给 Early Joint，而不是建立新的 global state machine。

## 11. Topology horizon 与 Joint

当前 team trajectory optimization 使用约 1.5 s horizon（`TeamVisibilityOptimizerParams::horizon`，coordinator `team_visibility_horizon`），适合 rolling P/T/yaw，但不一定覆盖“进入 shadow -> 绕过 blocker -> LOS recovery -> exit”。

建议区分两个概念：

```text
trajectory optimization horizon = 当前 MINCO/SCP / Joint tail 的短滚动窗口
topology evaluation horizon = 事件驱动到 blocker shadow exit/LOS recovery 的窗口
```

topology horizon 不应拍脑袋固定 3 s/5 s；应由 blocker geometry、predicted target motion、current speed 和 risk trace 计算到 `t_exit`，并受已有 execution coverage / deadline 限制。MINCO 仍只优化当前可执行 tail；topology evaluator 只负责 side semantics 和 candidate ranking，必要时把 intent/side constraint 延续到下一 cycle。

Early Joint 不需要重写：每架本地输出 `(kind, blocker_id, side, intent_generation, joint_seed)`，Joint 组合 `(sigma0,sigma1,sigma2)`，TeamVisibilityOptimizer 在各自 topology constraint 内优化 P/T/yaw。若当前 Joint 允许 continuous P/T 走出 selected side，observation corridor 的 active half-space 会阻止跨 side；若 corridor 不可行，candidate 应在 local hard preflight 前被拒绝，而不是让 Joint 事后修复。

## 12. 当前 Joint objective / selector authority 审计

当前 `TeamVisibilityOptimizer::evaluate()` 的 objective 在 `team_visibility_optimizer.cpp:1290-1307` 为：

```text
J = multiview_weight * multiview_cost
  + k2_weight * k2_cost
  + accumulated_weight * accumulated_cost
  + k2_continuity_weight * k2_continuity_cost
  + blackout_weight * blackout_cost
  + deviation_weight * deviation_cost
  + jerk_weight * jerk_cost
  + yaw_prior_weight * yaw_prior_cost
  + diversity_weight * diversity_cost
  + encirclement_gap_weight * encirclement_gap_cost
  + target_joint_weight * recovery_target_cost
```

coordinator 默认/运行时参数读取位于 `multi_uav_topology_coordinator.cpp:302-395`：horizon 默认 1.5、sample dt 默认 0.10、`k2_weight=.20`、`accumulated_weight=4.0`、deviation 1.0、jerk `1e-5`、diversity `.15`、multiview 1.0、joint gap 4.0；blackout 根据 runtime enable flag，且 protection sum 可能按现有代码缩放。实际运行参数以 launch/parameter server 日志为准，本轮没有修改或覆盖它们。

当前 authority 分三层：

1. optimizer step acceptance：`visibilityContinuityAcceptance()` 保护 binary camera-time/K2/none/longest losses；`cameraTimeAcceptance()` 在旧逻辑中仍可由 target/geometry/measurable benefit 绕过 binary plateau（`team_visibility_optimizer.cpp:582-596`）。
2. optimizer final success：还要求 final hard constraints、accepted iteration、measurable benefit 和 nonzero trajectory change（`2014-2030`）。
3. coordinator actual proposal/adoption：Feedback61 已加入 `meaningfulBinaryTeamBenefit()` 作为实际 adoption gate；但是 blocker/side semantic 仍不在 objective/selector 中。

因此当前系统已修复 Feedback60 的 binary plateau actual adoption，但仍存在一个更底层语义事实：All3/weakest camera 是 ranking/continuity 项，不是 static blocker side authority；Joint 只能在 local candidates 已带正确 topology 时放大其效果。

## 13. 微停顿与蛇形审计

### terminal velocity

`buildFreshMovingInitializer()` 只有 `mission_end=true` 才把 terminal velocity 设为 zero；普通 `computeInitState()` 明确传 `mission_end=false`（`planner_manager.cpp:858-874`），因此不能把所有 normal trajectory 终点 v=0 归因于 fresh initializer。仍需从 trajectory CSV/JSONL 逐事件检查 `v_end`，因为 local target velocity、retiming、feasible initializer fallback 和 successor replacement 都可能产生局部速度下降。

### topology flip

当前 candidate pipeline 有 accepted-state warm start 和 same-obstacle metadata，但 static occlusion 没有 blocker identity/side intent。coordinator `rapid_reversal` 只记录是否在 `rapid_reversal_window` 内回到上一 topology，未作为 veto。因此 topology chatter 是有结构支持的高概率嫌疑，但仅靠源码不能把所有左右摆动都判为单一根因。

### cross-plan smoothness

单条 MINCO 本身有 smooth P/V/A/J；跨 plan 的 continuity 由 warm start、head-state checks、candidate preservation 和 lifecycle contract 分散保证。每次 side candidate 在短 guidance window 内重新生成且没有 observation corridor 时，跨-plan side reference 可能变化，即使每条 polynomial 单独平滑。结论：

```text
MICRO_STOP_PRIMARY_SUSPECT: PARTIAL
TOPOLOGY_CHATTER_PRIMARY_SUSPECT: STRONG POSSIBILITY, not event-level proven
CROSS_PLAN_SMOOTHNESS_GAP_SUPPORTED: PARTIAL
```

需要的下一轮 telemetry 是：每个 activation 的 `v_end/a_end`、blocker id/side intent、candidate source、side switch event、command speed dip 和 successor overlap；本轮不补跑仿真。

## 14. 修改前/修改后调用图

### CURRENT

```text
static/dynamic/FOV soft risk
  -> sharedVisibilityTopologyTrigger
  -> DynamicRiskInfo-shaped witness (dynamic schema)
  -> path-tangent SIDE_PLUS/SIDE_MINUS seed
  -> static body precheck
       -> free: no A*, no Local-SFC, soft J_vis/Elastic + MINCO
       -> colliding: local A* repair
            -> static path audit
            -> LocalSfcPlane boundary scan
            -> MINCO/SCP
  -> candidate visibility ranking
  -> local bundle N/L/R
  -> Early Joint tuple + TeamVisibilityOptimizer P/T/yaw
  -> final hard preflight / CAS / 3ACK
```

### PROPOSED (design only, not implemented this round)

```text
static collision / static LOS / dynamic conflict / spatial FOV event
  -> Unified BlockerDescriptor (identity, witness, interval, frame)
  -> local NOMINAL / OBS_LEFT / OBS_RIGHT topology candidates
       -> path body collision? same-topology local A*
       -> STATIC_COLLISION_CORRIDOR and/or STATIC_OCCLUSION_SIDE constraints
       -> MINCO/SCP in topology corridor
       -> hard preflight
  -> candidate metadata (blocker, side, intent generation)
  -> Early Joint tuple (sigma_i + blocker/side semantics)
  -> TeamVisibilityOptimizer P/T/yaw constrained by active topology planes
  -> committed-prefix/CAS/3ACK unchanged
```

不动的模块：dynamic prediction、existing body collision A* implementation、MINCO/SCP solver、final preflight、committed-prefix lifecycle、CAS/3ACK protocol。只扩展 blocker/constraint semantics 和 trigger-to-topology data path。

## 15. 分阶段实施方案

### 方案 1：最小修改版（优先验证语义）

涉及文件/函数：

- `static_los_geometry.h::querySegmentClearance`：新增 witness overload，保留旧接口；
- `poly_traj_optimizer.h/cpp`：在 diagnostics/candidate export 中保留 witness、hit point、primitive type；
- `planner_manager.h/cpp`：将 witness 传入 `CandidateResult`/`TopologyCandidate`，并让 SIDE frame 在有 witness 时使用 observation side，缺失时回退 path side；
- `LocalSfcPlane`：只增加 `source`/`blocker_id` metadata，不改变 collision plane 数值；
- candidate preflight：对 observation-side candidate 增加 signed-side trace 检查，仍由现有 MINCO/SCP 和 final hard checks 决定执行。

数学变化：仅把 `side_dir` 从 path tangent fallback 扩展为 blocker-target `n_obs`，增加事件驱动 active interval；不改变 J_vis 权重、horizon、安全阈值或 A*。

风险：witness frame 在 target/observer 共线退化时需要稳定 fallback；对墙体必须正确处理 local frame。测试需求：cylinder target-side fixture、far-side fixture、no-witness fallback、static-free path with observation-side metadata、candidate source/side persistence contract。复杂度低，最可能的回归是 side sign 反转或 active interval 边界错误。

### 方案 2：推荐完整版本

在方案 1 上增加：

- `UnifiedBlockerDescriptor`：统一 static collision/static occlusion/dynamic witness/FOV spatial event 的 identity、frame、interval、source；
- `TopologyIntent`：event-driven persistence/exit；
- `LocalTopologyConstraint` 或向后兼容扩展的 `LocalSfcPlane` source semantics；
- static LOS-only observation corridor；
- same-topology A* repair：只有 body collision 才调用 A*，observation corridor 与 A* guide 共享 side token；
- Early Joint seed descriptor 携带 blocker id/side/intent generation；
- TeamVisibilityOptimizer 在 selected topology corridor 内优化，禁止跨 side；
- event-driven topology horizon 到 blocker shadow exit，MINCO rolling horizon 不变。

测试需求：静态 LOS-only 无 body collision、静态 LOS+body collision、动态 conflict、FOV spatial risk、blocker replacement、intent exit、Joint same-side tuple、corridor infeasible、Local no-op/stale/CAS regression、跨 plan side switch 和速度连续性。复杂度中等，回归重点是误锁 side、target motion 使旧 intent 失效、多个 blocker 的 candidate explosion。

### 方案 3：不推荐的大改

重写为 global visibility graph / 全森林柱子枚举 / 3-UAV joint A* / dynamic SFC / 长期全局 topology state machine。它会重复已有 A*、SFC、MINCO、Joint/CAS 职责，增加候选组合和 lifecycle 风险，且违反当前 committed-prefix 架构的最小侵入原则。

## 16. 最小修改优先级

如果只允许先改三处：

1. **Static LOS witness identity**：让 first/earliest blocker、hit point、primitive type 从 geometry/optimizer diagnostics 到 `CandidateResult`/Topology seed 可见；没有这一步，SIDE 仍会把 target/dynamic obstacle 当遮挡物。
2. **Observation-side frame + active interval**：有 witness 时按 target-occluder geometry 生成 L/R，path tangent 仅作退化 fallback；记录同 blocker side semantics。
3. **Observation corridor reuse**：扩展现有 `LocalSfcPlane` source/interval，在 static LOS-only 情况给 MINCO/SCP 一个只覆盖 occlusion core 的 side half-space；A* 仍仅用于 body collision repair。

TopologyIntent 和 Joint metadata 是第二阶段，不能在没有 witness/corridor 的情况下先用 selector 或权重弥补。

## 17. 对十个关键问题的明确回答

1. **静态柱只遮 LOS、不挡 UAV 路径时有无真正绕行 topology？** 有 N/L/R candidate label 和软 SIDE seed，但没有 blocker-referenced observation topology；因此严格答案是没有完整的 observation topology。
2. **为什么 A* 不启动时 Local-SFC 常为 0？** 因为当前 Local-SFC 是 A* static collision corridor，不是 LOS corridor；这是现有设计语义，不是 zero-plane 失败。
3. **J_vis 能否独立从柱另一侧绕出？** 不能保证。它提供 static LOS clearance gradient/soft cost，可能改变 P/T，但没有 side sign、blocker identity 或 topology-preserving hard/semi-hard constraint。
4. **现有 SIDE 能否复用？** 能复用 candidate lifecycle、MINCO/SCP、A* handoff 和 Joint tuple；需要把 reference frame 从单一 path tangent 扩展为 blocker geometry frame，并保留 fallback。
5. **Observation Corridor 复用 SFC 还是新抽象？** 复用现有 half-space/SCP 数学；推荐扩展 `LocalSfcPlane` source semantics 或轻量 `LocalTopologyConstraint` wrapper，不复制第二套 solver。
6. **全局策略是 always-on 还是 available+activation？** global available + local activation；只对 predicted primary blocker 生成 N/L/R。
7. **如何避免森林 candidate explosion？** 每 UAV 每 cycle 选 earliest-risk/worst-risk primary blocker，最多一个 competing blocker；N/L/R 上限和 Joint 27-tuple 机制保留。
8. **如何避免 L/R 翻转蛇形？** blocker identity + side intent + event-driven exit；同 blocker risk 未退出时保持 side，只有 hard safety/infeasible/new stronger blocker 才换 side。
9. **如何与 Early Joint 结合？** 每架 candidate 携带 blocker/side/intent metadata，Joint 仍组合 `(sigma0,sigma1,sigma2)`，只需把 same-topology constraint 传入 TeamVisibilityOptimizer；不需要 Joint A*。
10. **最小修改三处？** witness identity、observation side frame/interval、observation half-space corridor。

## 18. 事实边界

Feedback59-61 的 run-level visibility 数字不能证明每一次 static LOS event 都走错侧；本轮也没有启动新仿真来填补这一缺口。源码能确认的是职责和 authority 缺口：static risk trigger 已存在，static blocker identity 没有进入 SIDE，observation corridor 不存在，J_vis/Elastic 不能独立授予 topology authority。要把设计问题进一步量化，下一轮应在已有生产 telemetry 中增加 blocker/side/intent/LOS-recovery event，不应先调权重或 horizon。

```text
CURRENT_ARCHITECTURE_HEALTH: PRESERVED
PRODUCTION_SOURCE_CHANGED: NO
LAUNCH_PARAM_CHANGED: NO
SIMULATION_RUN: NO
FULL_ON_RUN_COUNT: 0
NEW_GLOBAL_ASTAR_ADDED: NO
THREE_UAV_JOINT_ASTAR_ADDED: NO
DYNAMIC_SFC_ADDED: NO
SAFETY_THRESHOLD_RELAXED: NO
LOCAL_PLANNER_FROZEN_FOR_JOINT: NO
UNIFIED_TRANSACTION_BARRIER_REINTRODUCED: NO

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
REPORT_FILE: /home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_62.md
```

最终判断：当前 ALP 已有“visibility risk 可以触发现有 SIDE 候选”的入口，但还没有“static LOS blocker 驱动、带观察侧语义和 corridor authority 的统一 observation topology”。最小正确方向是把 blocker identity 和 observation frame 上移到现有 SIDE/A*/Local-SFC/MINCO/Joint 数据链，而不是增加新的全局规划器或修改现有安全/Joint 生命周期。
