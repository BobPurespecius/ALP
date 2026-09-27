# Feedback 063 - Unified BODY/LOS Conflict-Driven N/L/R Implementation

日期：2026-09-14  
项目：`/home/bob/ALP/egov2_fc65423_constvel`  
基线：Feedback062（设计审计）与 Feedback061（已完成的 committed-prefix/Joint 合同）

## 0. 范围与结论

本轮已按 Feedback062 实施统一的 BODY/LOS 冲突入口、真实 blocker witness 传递、观察侧候选约束和同 blocker 的事件型 persistence，并完成定向合同、完整构建和一条当前 Scenario A FULL ON + native RViz 验证。

本轮没有新增全局 planner、dynamic collision SFC、恢复状态机、planner brake/stop fallback，也没有改变安全阈值、Joint 事务协议或 Local 等待 Joint 的语义。

最终 run 的静态/动态拓扑接线和真实 SIDE 激活有生产证据；但该 run 的 harness 以 timeout 124 退出，且仍有 17 次 `TRAJECTORY_END_BEFORE_NEXT_ACTIVATION`、3 次 `MOVING_SUCCESSOR_STARVATION`。这些失败集中在普通 NOMINAL successor 的短轨迹供给阶段，不应被虚报为本轮健康门禁全通过。run6 比旧的 run5（36 次 end-before-next、10 次 starvation）明显减少，但这仍是独立的 Local 供给残余问题。

## 1. 实施内容

### 1.1 统一冲突描述

`EGOPlannerManager::ConflictDescriptor` 现在保存独立的 BODY 和 LOS 原因、运动类型、障碍 identity、primitive type、几何/预测时间、observer/target/obstacle 证据、frame 和 clearance。BODY+LOS 同时发生时只合并 `reason_mask`，不合并两个物理 witness。

关键路径：

- `planner_manager.cpp:5850-5950`：静态 LOS witness 与静态 BODY 证据进入 descriptor；
- `planner_manager.cpp:5980-6065`：保留 BODY risk 的时间和障碍字段，不再以 LOS target 位置覆盖硬安全 witness；
- `planner_manager.cpp:6100-6166`：动态 LOS 使用当前预测时刻逐对象求交，写入真实动态 blocker identity；
- `planner_manager.cpp:6174`：输出可审计的 `conflict-descriptor`。

静态与动态并不再以“有没有 dynamic object”决定是否允许空间拓扑。`KNOWN_EMPTY` 场景下，静态 BODY/LOS 仍可进入 N/L/R；动态预测缺失时不会伪装成静态空场景，最终动态安全检查仍可拒绝候选。

### 1.2 静态 LOS witness

`StaticLosWitness` 和 `StaticLosGeometry::querySegmentClearance()` 扩展为返回：primitive index/type/name、ray parameter、clearance、hit point、水平法向、primitive center 和有效半径（`static_los_geometry.h:40-51,188-194`）。`PolyTrajOptimizer::queryStaticLosClearance()` 增加保留旧 ABI 的 overload，并把 witness 交给 manager（`poly_traj_optimizer.h:501-511`）。

这消除了旧链路中“trigger 有 static LOS support，但 manager 只收到 target/最近 dynamic object”的耦合。FOV-only support 没有真实空间 blocker，因此只继续作用于 yaw/J_vis，不生成伪造的柱子或观察 shadow。

### 1.3 N/L/R 方向与 guide

BODY 或 BODY+LOS 继续使用原有 motion/path SIDE 语义；不会把安全方向和观测方向向量平均。只有 LOS_OCCLUSION 时，candidate 使用 blocker-target frame：

```text
e   = normalize(C_xy - T_xy)
n_R = (e_y, -e_x)
alpha = asin(R / ||C-T||)
m_s = -sin(alpha)e + s*cos(alpha)n_R,  s=+1 RIGHT, s=-1 LEFT
```

观察半空间是 `m_s dot (p_xy - T_xy) >= 0`。例如 `T=(0,0), C=(D,0)` 时，`n_R=(0,-1)`，PLUS/RIGHT 位于柱子下侧、MINUS/LEFT 位于柱子上侧。这个 half-space 只用于候选局部选侧，不替代真实 3D LOS 检查，也不把整条场景变成可见性禁飞区。

观察 guide 仍从真实 activation head 的 P/V/A 出发；只有 seed 已到达某侧后，才计算 `enter_time`，并在可达的事件区间内激活观察约束。若 body seed 静态不可行，仍进入原 A* repair；若 body seed 静态可行，A* 可以不调用但观察约束仍可生成。观察约束不可行只淘汰该候选，不阻塞 NOMINAL/其他安全候选。

### 1.4 Local-SFC / MINCO / Early Joint 接线

`LocalSfcPlane` 增加来源语义：

```text
STATIC_COLLISION_CORRIDOR
LOS_OBSERVATION_SIDE
```

并保留 blocker、geometry revision、side sign、active interval 和 provenance。两类约束共用现有 half-space/SCP 装配；碰撞 corridor 的数值语义不变，LOS observation plane 只在局部 blocker event 区间生效，不是 dynamic SFC。

Joint seed 继续使用现有 N/L/R candidate，但附带 blocker/frame/side/constraint provenance。Joint 重新优化 P/T/yaw 后仍走同一版本的 hard preflight 和 world-time 任务检查。Joint 失败仍是 Local NO-OP。

### 1.5 跨 rolling cycle persistence

同一 blocker identity、同一已接受 observation side 且当前 active trajectory 仍通过安全重检时，复用当前 side，避免每个 rolling cycle 重新求解两侧。该 persistence 是 identity/event-based，不是固定 dwell timer；BODY hard safety、blocker generation 改变、side 失效或事件退出仍可重新选侧。

## 2. 生产源码定位

| 内容 | 文件/位置 |
|---|---|
| `ConflictDescriptor` | `planner_manager.h` |
| 静态 LOS witness query | `static_los_geometry.h:40-51,188-194` |
| static LOS -> descriptor | `planner_manager.cpp:5850-5950` |
| dynamic LOS witness | `planner_manager.cpp:6100-6155` |
| shared trigger 与 spatial gate | `planner_manager.cpp:6163-6169` |
| observation half-space | `planner_manager.cpp:6805-6859` |
| observation source metadata | `poly_traj_optimizer.h:28-49` |
| witness-aware optimizer API | `poly_traj_optimizer.h:501-511` |

## 3. 定向测试与构建

构建命令：

```text
catkin build traj_utils traj_opt ego_planner multi_uav_formation \
  -j2 --no-status --workspace ros_ws
```

结果：6 个包成功，无 build failure。

已运行并通过的相关合同/fixture 包括：

- `feedback53_regression_fixture_test`
- `fresh_moving_initializer_contract_test`
- `local_execution_contract_test`
- `recovery_probe_production_test`
- `trajectory_lifecycle_contract_test`
- `visibility_topology_production_test`
- `adaptive_viewpoint_generator_contract_test`
- `committed_prefix_joint_contract_test`
- `cooperative_viewpoint_contract_test`
- `early_joint_primary_contract_test`
- `encirclement_geometry_contract_test`
- `feedback58_barrier_regression_test`
- `joint_adoption_atomic_contract_test`
- `joint_failure_local_noop_contract_test`
- `joint_future_tail_activation_contract_test`
- `multiview_contract_test`
- `persistent_recovery_target_contract_test`
- `prediction_epoch_semantics_contract_test`
- `stale_joint_cas_contract_test`
- `static_los_wall_contract_test`
- `team_solution_commit_contract_test`
- `team_visibility_optimizer_contract_test`
- `topology_coordinator_contract_test`
- `unified_team_transaction_contract_test`
- `elastic_visibility_contract_test`
- `time_only_feasibility_contract_test`
- `time_only_swarm_temporal_contract_test`

`visibility_topology_production_test` 特别覆盖 static LOS、镜像 LEFT/RIGHT、真实 blocker identity、candidate full chain、A* same-side repair、mixed BODY+LOS descriptor 和 BODY anchor，结果全 PASS。

## 4. FULL ON 证据

最终针对性运行目录：

```text
scenario_a_unified_20260914/run6_body_los_gate
```

使用当前 `long_cylinder_forest_visibility_stress.json`、FULL ON、native RViz、未调权重/场景/安全阈值。harness 状态为 `FULL_LAUNCH_STATUS=124`；但已记录完整的 80.243231 s、2406 个 visibility samples，目标/guide 数据到达场景末段。由于退出不是 clean zero，以下结果只作为生产接线证据，不写成完整健康验收 PASS。

### 4.1 拓扑触发与真实 SIDE 证据

run6 日志计数：

```text
VISIBILITY_TOPOLOGY_TRIGGER = 799
CONFLICT_DESCRIPTOR = 37
FOV_ONLY_TRIGGER = 37
LOS/BODY SPATIAL TRIGGER = 735
OBSERVATION_TOPOLOGY_PLANE = 862
OBSERVATION_TOPOLOGY_PERSISTENCE = 752
```

一个完整生产证据链位于 `full.log:6341-6513`：

```text
static LOS witness blocker=0
 -> SIDE_PLUS observation plane enter=1.910136 exit=2.360136
 -> MINCO SIDE_PLUS solver SUCCESS
 -> visibility-selection selects SIDE_PLUS
 -> topology-post-check static_valid=1, dynamic_valid=1
 -> LOCAL_COMMIT
 -> execution-handoff with predecessor P/V/A continuity
 -> continuous-motion-activation source=SIDE_PLUS
```

对应日志中同时有 `JOINT_SEED_EXPORTED`，说明 local candidate 和 Early Joint seed 使用同一 SIDE 语义。这里不是只看 side sign 或 optimizer cost：candidate 经过 static/dynamic/swarm/dynamics preflight 后才被激活。

### 4.2 Joint/Local 非阻塞与原子性

```text
EARLY_JOINT_COMMIT = 43 (log records)
EARLY_JOINT_ACTIVATED = 129 (3 UAV records per commit)
TEAM_COMMIT_CANCEL = 0
TEAM_COMMIT_REVOKE = 3
JOINT_FAILURE_LOCAL_NOOP = 263
LOCAL_COMMIT_BLOCKED_BY_JOINT = 0 observed
JOINT_FAILURE_AFFECTED_LOCAL = 0 observed
```

run6 未出现 1/3 或 2/3 activation 记录；43 个 commit 对应 129 架次 activation，partial activation 计数为 0。3 次 revoke 使用团队语义，不能由此把它们计为 partial activation。

### 4.3 可见性

`visibility_summary.csv`：

```text
elapsed_s = 80.243231
CAMERA_TIME = 219.297632 camera.s
UAV0 visible ratio = 0.944306
UAV1 visible ratio = 0.945968
UAV2 visible ratio = 0.968828
MEAN_VISIBLE = 2.859102 camera count (三架 visible ratio 之和；不是 ratio 的算术平均)
K2 = 0.976386
ALL3 = 0.760399
NONE = 0
LONGEST_K2_LOSS = 0.732822 s
LONGEST_BLACKOUT = 0
FOV_MISMATCH_TEAM = 0.039900
```

与 Feedback061 的 `CAMERA_TIME=219.329720, K2=0.976052, ALL3=0.757338` 是不同 run 的描述性比较，不能作因果或统计显著性结论。run6 的三架原始 visible ratio 直接来自 recorder；报告不把这些差异全部归因于本轮拓扑实现。

### 4.4 健康门禁

```text
TERMINAL_HOLD_COUNT = 17
STARVATION_COUNT = 3
END_BEFORE_NEXT_COUNT = 17
UNVALIDATED_EXECUTION_COUNT = 0
SWARM_VIOLATION_COUNT = 0
EXECUTED_COLLISION_COUNT = 0 observed in trajectory/visibility audit
```

原始 `full.log` 中的 `STATIC_COLLISION`/`scp-static-failure` 同时包含 candidate rejection、reference audit 和未执行 seed，不能直接作为 executed collision。轨迹/visibility 审计没有发现实际执行碰撞。

17/3 个残余事件的代表性原因是：NOMINAL candidate 反复只有 0.05--0.45 s 短轨迹，被既有 `SHORT_STATIONARY_HYPOTHESIS required=0.490000` 拒绝；随后触发已有 urgent/post-deadline moving replan。它们发生在 `source=PERSISTENCE_FALLBACK`/`kind=NOMINAL` 路径，而不是 `LOS_OBSERVATION_SIDE` 约束把 Local 变成 blocking gate。run5 同口径为 36 end-before-next、10 starvation，因此 run6 负载已下降但仍未闭合 Local successor 供给合同。本轮没有擅自修改 lifecycle、terminal velocity、planner brake 或 stop fallback。

## 5. 验收矩阵

```text
STATIC_AND_DYNAMIC_SHARE_CONFLICT_ENTRY                  = PASS
STATIC_BODY_RISK_CAN_TRIGGER                             = PASS (代码/fixture)
STATIC_LOS_WITHOUT_DYNAMIC_OBJECTS_CAN_TRIGGER           = PASS (fixture + run log)
DYNAMIC_LOS_ONLY_CAN_TRIGGER                             = PASS (fixture + witness code)
REAL_BLOCKER_IDENTITY_REACHES_SIDE                       = PASS
LOS_SIDE_USES_TANGENT_GEOMETRY                           = PASS (blocker-target tangent)
BODY_SIDE_ORIGINAL_SEMANTICS_PRESERVED                   = PASS
OBSERVATION_CONSTRAINT_WITHOUT_ASTAR                      = PASS
TASK_CONSTRAINT_FAILURE_DOES_NOT_BLOCK_LOCAL              = PASS (contracts/code)
EARLY_JOINT_RECEIVES_FRAME_AND_CONSTRAINT                = PASS (seed/log)
NO_GLOBAL_VISIBILITY_EXECUTION_GATE                      = PASS
TEAM_ACTIVATION_CARDINALITY_ONLY_0_OR_3                  = PASS in run6 records
POST_COMMIT_ORDINARY_LOCAL_INVALIDATION_BLOCKED           = PASS (Feedback061 regression retained)
HARD_SAFETY_TEAM_CANCEL                                  = PASS (contracts; 3 run6 revoke records)
JOINT_FAILURE_IS_LOCAL_NOOP                              = PASS
LOCAL_COMMIT_BLOCKED_BY_JOINT                            = 0 observed
```

需要保留为未通过的 run-level 项：

```text
FULL_RUN_CLEAN_EXIT                                      = FAIL (harness timeout 124)
TERMINAL_HOLD_COUNT                                      = FAIL (17)
STARVATION_COUNT                                         = FAIL (3)
END_BEFORE_NEXT_COUNT                                    = FAIL (17)
```

## 6. 微停顿与左右摆动

本轮记录到的 handoff 证据显示，成功 SIDE 切换的 `HANDOFF_DP/DV/DA` 可达到约 `1e-6/1e-8/1e-7`，说明部分跨 plan 接管是连续的；但 run6 末段仍有短轨迹供给耗尽和 `OPTIONAL_REFINEMENT_STOP`，因此不能写成“微停顿已解决”。

当前证据支持：

- 同 blocker persistence 减少了反复双侧求解（752 条 persistence 记录）；
- 观察侧在真实 geometry event 内激活，不是每周期永久锁边；
- 仍存在普通 NOMINAL 短 successor 与 deadline 竞争导致的运动 gap；
- 是否所有用户观察到的左右晃动都由 topology 缺失造成，仍为 UNKNOWN，不能只凭本轮 run 归因。

## 7. 与任务要求的最终对应

1. 以前 static LOS 进入 dynamic-shaped 链，是因为 optimizer 的 static support 虽然存在，但 manager 丢弃了 witness，SIDE 仍读 `DynamicRiskInfo.obstacle_position/obstacle_id`。现在 `StaticLosWitness` overload、`ConflictDescriptor` 的独立 `los_*` 字段和动态逐对象 LOS query 保留真实 blocker/frame，FOV-only 则被过滤出空间 SIDE。
2. 静态/动态 BODY/LOS 四种组合现在先形成 reason-preserving descriptor，再进入同一个 N/L/R candidate dispatch；BODY 安全方向优先，LOS 只在安全候选内影响观察比较和局部约束。
3. 又撞又挡视线时不做安全方向与观测方向平均：BODY seed/repair 保留原 motion semantics，LOS 约束只在相容候选的局部 observation frame 中装配，最终仍由 hard preflight 过滤不安全侧。
4. A* 不启动时，LOS candidate 仍可得到 `LOS_OBSERVATION_SIDE` plane；该 plane 仅作用于 blocker 的可达 enter/exit interval，约束失败只淘汰该 candidate，不会成为全局可见性 execution gate。
5. 当前 head 位于 shadow 内时，guide 保留真实 head P/V/A；只有 seed 几何上到达 observation side 后才设置 `active_start=enter_time`，若 horizon 内到不了则记录 `RECOVERY_NOT_REACHED/WINDOW_LIMITED`。
6. Joint 收到 candidate 的 blocker/frame/side/source metadata；其 P/T/yaw 改动后仍复用相同约束装配、固定 world-time 检查和 final preflight。
7. 静态 LOS 走错侧是否在完整场景中已达到统计意义上的减少，当前不能宣称。可以确认的证据是 run6 已真实生成并激活 blocker=0 的 SIDE_PLUS，且 static/dynamic preflight 通过；完整 run 的总体 visibility 只作描述性结果。
8. 微停顿/乱晃没有被本轮完全解决。部分 handoff 连续，末段的 17/3 供给失败仍是残余根因候选；`terminal hold=0` 不能从本轮证据推出。

## 8. 状态声明

```text
PRODUCTION_SOURCE_CHANGED: YES
NEW_GLOBAL_PLANNER_ADDED: NO
NEW_DYNAMIC_COLLISION_SFC_ADDED: NO
SAFETY_THRESHOLDS_CHANGED: NO
VISIBILITY_BECAME_GLOBAL_HARD_EXECUTION_GATE: NO
LOCAL_WAITING_FOR_JOINT_REINTRODUCED: NO
NEW_RECOVERY_STATE_MACHINE_ADDED: NO
PLANNER_BRAKE_OR_STOP_FALLBACK_ADDED: NO
NEW_GLOBAL_ASTAR_ADDED: NO
DYNAMIC_SFC_ADDED: NO
SAFETY_THRESHOLD_RELAXED: NO
```

本轮运行启动的 ROS master、roslaunch、RViz、Gazebo、recorder 已清理。历史目录 `scenario_a_unified_20260914/run5_body_los_fix`、`run6_body_los_gate` 和所有关键日志/CSV 均保留。

```text
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```

REPORT_FILE: `/home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_63.md`
