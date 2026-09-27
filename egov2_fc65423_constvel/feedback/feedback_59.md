# Feedback 059 - Committed Prefix + Background Joint Future Tail 事实重建与闭环验证

日期：2026-09-14  
项目：`/home/bob/ALP/egov2_fc65423_constvel`  
场景：`long_cylinder_forest_visibility_stress.json`，FULL ON + native RViz

## 0. 结论

本轮没有依据上下文压缩后的摘要继续修改算法，而是从当前工作树、Feedback56-58、进程、原始 launcher/CSV/JSONL、PID metadata 和二进制 hash 重建事实。上一位 Codex 在运行尚未结束时宣称完成，时间点上是不成立的；但保留下来的同一 run 原始证据事后证明任务确实正常跑完。因此“当时已经完成”的表述不可信，而最终 run 本身是完整的。

当前生产架构不是 Feedback58 blocking barrier，而是：Local current-revision hard preflight 后立即正常 commit；Coordinator 只读取 committed、safety-validated schedules，找 common future frontier，在后台优化 Joint tail；CAS stale、optimizer failure、无 tuple 或 proposal failure只丢弃 Joint，不改变 Local authority。完整 FULL run 中有 62 个去重 solution ID 闭合 optimizer/preflight -> proposal -> 3ACK -> commit -> 三机 actual activation -> optimized yaw command execution。

```text
PREVIOUS_CODEX_CONTEXT_COMPRESSION_STATE_RECONSTRUCTED: YES
PREVIOUS_CLAIMED_FULL_RUN_VERIFIED: YES
PREVIOUS_CLAIMED_62_JOINT_ACTIVATIONS_VERIFIED: YES
LAST_RUN_STATUS: COMPLETE
CURRENT_ARCHITECTURE_ACTUALLY_IMPLEMENTED: COMMITTED_PREFIX_FUTURE_TAIL
```

## 1. 工作树与运行身份

工作树原本已有大量未提交生产修改和历史证据，本轮未 reset、覆盖或清理。Git HEAD 是 `8cef24143c8a8a23bae77dd0e6cf5395c7dc6d2c`；FULL run 与本轮最终 build 的关键二进制 hash一致：

```text
multi_uav_topology_coordinator:
  d26f62127559e774de6af9f32195a28167d0e2376146470e023d0256525a54fc
ego_planner_node:
  3b3ca3b30410854389f26c7602ad662cfc4b0b380addc9de57d8c1c75687970c
scene SHA256:
  430fce52ee31c4a4ea13607e39346e0f04f1af1e3da31eb426a44a61786c4cb0
```

最后运行身份：

```text
LAST_FULL_RUN_ID: committed_prefix_run1
LAST_FULL_RUN_DIRECTORY: unified_team_transaction_validation_20260913/committed_prefix_run1
LAST_FULL_RUN_ACTUAL_START: 2026-09-13 21:38:44.263139 +0800
LAST_FULL_RUN_ACTUAL_END: 2026-09-13 21:40:04.467680 +0800
LAST_FULL_RUN_ROUTE_COMPLETE: YES
LAST_FULL_RUN_PROCESS_EXITED: YES
LAST_FULL_RUN_EVIDENCE_CONSISTENT: YES
LAST_FULL_RUN_STATUS: COMPLETE
```

交叉证据：`run_meta.json` 的 stop reason 为 `target_replay_normally_completed`、exit code 0、route duration `80.204540745 s`；trajectory/visibility CSV 覆盖 mission end，target 到达 `(36,0,1.5)`；launcher 正常关闭全部 nodes/master；所有 recorder 文件停止增长。分析脚本显式读取上述目录，没有混入旧 run。

## 2. 62 条 Joint actual activation 审计

审计不把 candidate、seed、proposal、ACK 或 scheduled-only 当实际执行。每个 solution 必须同时具备 optimizer success、joint full preflight、proposal、权威 commit reason `ALL_THREE_ACKED_CAS_SETTLED`、commit、三架 safety-validated activated polynomial identity，以及 optimized yaw payload 与三机实际 command yaw/yaw-rate 数值匹配。

```text
CLAIMED_OLD_62_ACTIVATIONS_VALID: YES
ACTUAL_COMPLETE_JOINT_COUNT: 62
JOINT_OPTIMIZED_YAW_EXECUTION_COUNT: 62
PARTIAL_ACTIVATION_IDS: 19,37,68
```

完整 IDs：

```text
3,4,5,7,8,9,11,12,13,14,15,16,17,22,23,24,25,26,27,28,29,30,31,
32,33,36,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,
56,57,58,59,60,61,62,63,64,66,67,69,70,71,73,74,75,76
```

旧分析器只报 60，是因为并发 ROS stdout 使两个 textual 3ACK/COMMIT 行交织损坏；旧的 traj-server 文本 yaw 统计也只找到 55。结构化 `team_solutions.jsonl`、activated polynomial identities 和 command-level yaw matching 修正后是 62，不是重复三机日志计数。

## 3. 当前生产 authority

```text
Local:
  candidate ready -> current-revision full hard preflight
  -> setLocalTrajFromOpt -> local_first_safe_direct publish/commit
  -> LOCAL_DIRECT_COMMIT local_gate_by_joint=0

Joint:
  committed validated schedules only
  -> common future frontier t_f
  -> sample committed P/V/A at t_f
  -> optimize [t_f,t_f+H]
  -> team hard preflight
  -> CAS owner tuple
  -> proposal / 3ACK / scheduled common activation
```

Target snapshot epoch是 prediction source epoch；frontier/activation是未来执行时刻。目标按 `world_time - target_snapshot_epoch` 外推。动态预测验证 identity 以及 `[valid_from, valid_to]` 覆盖 Joint horizon，不再要求 epoch 数值等于 frontier。三架异步 local generation允许不同，CAS记录各自 owner revision/trajectory tuple。

```text
FEEDBACK58_BARRIER_REMOVED: YES
LOCAL_FIRST_SAFE_IMMEDIATE_COMMIT_AUTHORITY_RESTORED: YES
JOINT_CAN_BLOCK_LOCAL: NO
JOINT_FAILURE_IS_LOCAL_NOOP: YES
TARGET_SNAPSHOT_AND_FRONTIER_SEMANTICS_SEPARATED: YES
LOCAL_GENERATIONS_MUST_MATCH: NO
JOINT_FRONTIER_USES_COMMITTED_TRAJECTORIES_ONLY: YES
STALE_JOINT_DISCARDED_WITHOUT_LOCAL_IMPACT: YES
LOCAL_COMMIT_BLOCKED_BY_JOINT_COUNT: 0
JOINT_FAILURE_AFFECTED_LOCAL_COUNT: 0
```

## 4. Deterministic contracts 与 build

测试环境中默认 shell 曾把另一 catkin workspace 放在当前 ALP 前面，导致 `libplan_env.so` ABI 错配和 fixture segfault。本轮仅用 `/opt/ros/noetic` 加当前项目 `ros_ws/devel` 的隔离环境验证，没有读取外部 workspace。

最终构建命令：

```text
catkin build traj_utils traj_opt ego_planner multi_uav_formation \
  -j2 --no-status --workspace ros_ws
```

结果：6 个实际构建包（含依赖）全部成功、无 warning。`ego_planner` CTest 5/5、`multi_uav_formation` CTest 20/20、traj-opt 三项 gradient/temporal contract 全部通过。`recovery_probe_production_test` 使用 initialized known-free synthetic GridMap 后通过，未改变 production unknown-map fail-safe 语义。ROS team bundle lifecycle fixture也通过。

六个新合同逐项 PASS：

```text
committed_prefix_joint_contract_test
joint_failure_local_noop_contract_test
feedback58_barrier_regression_test
stale_joint_cas_contract_test
joint_future_tail_activation_contract_test
prediction_epoch_semantics_contract_test
```

关键输出：

```text
JOINT_FAILURE_IS_LOCAL_NOOP = PASS
LOCAL_COMMIT_BLOCKED_BY_TEAM_ALIGNMENT = NO
PREDECLARED_ALIGNMENT_CAN_BLOCK_LOCAL = NO
STALE_JOINT_DISCARDED_WITHOUT_LOCAL_IMPACT = PASS
JOINT_FUTURE_TAIL_ACTIVATION = PASS
TEAM_BUNDLE_LIFECYCLE_ROS_PASS committed_prefix=1 joint_failure_local_noop=1
```

## 5. FULL ON 健康、Joint 与任务指标

本轮没有再启动第二条仿真：现存 run 完整、运行二进制与最终生产二进制 hash一致、健康门禁全零且已有 62 条真实 Joint future-tail execution，已经满足“一旦首条 healthy + real Joint execution成立立即停止”。

```text
TERMINAL_HOLD_COUNT: 0
STARVATION_COUNT: 0
END_BEFORE_NEXT_COUNT: 0
UNVALIDATED_COUNT: 0
COLLISION_COUNT: 0
SWARM_VIOLATION_COUNT: 0

JOINT_FRONTIER_OPPORTUNITY_COUNT: 190
JOINT_ATTEMPT_COUNT: 179
JOINT_SUCCESS_COUNT: 76
JOINT_CAS_STALE_COUNT: 11
JOINT_PROPOSAL_COUNT: 76
JOINT_3ACK_COUNT: 65
JOINT_ACTIVATION_COUNT: 62
```

可见性与协同：

```text
CAMERA_TIME: 219.297632 camera.s
MEAN_VISIBLE: 2.736043
K2: 0.976386
ALL3: 0.760399
NONE: 0.000743
Q_DIR: 0.950894
HIGH_QUALITY_DIRECTIONAL_RATIO: 0.893969
HIGH_QUALITY_MULTI3_RATIO: 0.689431
ENCIRCLEMENT: 0.545846
SAME_SEMICIRCLE_RATIO: 0.311512
TEAM_REFERENCE_COVERAGE_RATIO: 0.677548
```

## 6. RViz 轨迹混乱与障碍审计

RViz 的 red `optimal_list` 是最新 scheduled committed polynomial，不等同于此刻 active execution；Local 与 Joint 共用同一 red topic/color/namespace和固定 marker IDs，而 odom Path 累积 actual history。rapid supersede 时缺少 source/revision/active-vs-scheduled 可视身份，造成多条未来计划、历史和实际路径的语义混淆。raw A* 使用独立 topic，不经过 executable marker/commit。

离线有限高圆柱检测确实找到 UAV0 local trajectory 131 的完整计划后缀与柱 9 相交：首次几何命中 local `t=2.468035 s`。但该 trajectory 实际只执行到 local `t=0.349233 s` 即被 successor 替换；实际执行前缀不相交，逐 odom sample collision count为 0。也就是说目视红线穿柱不是虚构，但它是后来被替换的 scheduled future suffix，并未成为实际碰撞。

```text
RVIZ_TRAJECTORY_CONFUSION_ROOT_CAUSE:
  scheduled committed Local/Joint polynomials share one red marker identity;
  odom accumulates actual history; superseded future suffix lacks explicit
  active/source/revision identity and may remain visually attributable to flight

ACTUAL_EXECUTED_OBSTACLE_COLLISION_OBSERVED: NO
CANDIDATE_OR_HISTORY_MARKER_COLLISION_OBSERVED: YES
RAW_ASTAR_EXECUTED: NO
JOINT_SEED_DIRECTLY_EXECUTED: NO
FINAL_PREFLIGHT_BYPASSED: NO
```

审计产物：`unified_team_transaction_validation_20260913/analysis/transaction_audit.json`。分析器现在同时输出完整 marker/polyline 命中、首次命中局部时间、actual executed local end 和 executed-prefix intersection，避免再把未来未执行后缀误报成实际飞行碰撞。

## 7. 禁止项与最终状态

```text
LOCAL_FIRST_SAFE_BLOCKED_BY_TEAM_BARRIER: NO
UNIFIED_TRANSACTION_IS_LOCAL_COMMIT_GATE: NO
HANDOFF_LEASE_ADDED: NO
TAKEOVER_RESERVATION_ADDED: NO
LOCAL_PLANNER_FROZEN_FOR_JOINT: NO
POST_JOINT_REBASE_ADDED: NO
NEW_GLOBAL_ASTAR_ADDED: NO
THREE_UAV_JOINT_ASTAR_ADDED: NO
DYNAMIC_SFC_ADDED: NO
SAFETY_THRESHOLD_RELAXED: NO
DYNAMIC_CLEARANCE_1P1_CHANGED: NO
PLANNER_BRAKE_REINTRODUCED: NO
STOP_FALLBACK_REINTRODUCED: NO
OLD_SUFFIX_EXTENDED: NO
STRICT_25_170_REINTRODUCED_AS_HARD: NO

BUILD_PASS: YES
UNIT_TEST_PASS: YES
GRADIENT_TEST_PASS: YES
CONTRACT_TEST_PASS: YES
FULL_ON_RUN_COMPLETE: YES

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
REPORT_FILE:
/home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_59.md
```

上一位 Codex 在当时尚未结束时提前宣称 FULL run 已完成，但该同一 run 后来确实正常完整结束；当前 Joint 的任何失败路径已经是 Local no-op；真实执行链已经从三机 committed validated prefix 的 common future frontier，经 Joint tail、preflight、CAS、proposal/3ACK，闭合到三机共同 activation 和 optimized yaw 实际执行。
