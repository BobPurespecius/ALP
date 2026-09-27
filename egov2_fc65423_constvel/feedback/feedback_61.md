# Feedback 061 - Atomic Joint adoption and validated-prefix authority

日期：2026-09-14  
项目：`/home/bob/ALP/egov2_fc65423_constvel`  
基线：Feedback060  
生产源码已修改；未访问或修改 `/home/bob/RRCT`。

## 本轮范围

本轮修复了 Feedback060 已确认的生产问题：

1. 3ACK/authoritative COMMIT 后，普通 local generation 更新不再单机否决 team tail；commit 前的 NACK 保留 proposal identity，避免 COMMIT 消息传输竞态造成部分 activation。真实硬安全失效只能发送 team-wide revoke。
2. Joint execution authority 现在要求与 committed Local future tail 在同一 binary evaluation horizon 上存在严格 camera-time、All3 或 weakest-camera visibility gain，并保留 K2/blackout/continuity 不退化约束。geometry、Q_dir、target progress 和 continuous objective 不能单独触发实际替换。
3. proposal 保存 Local committed suffix、owner revision/trajectory identity 和 polynomial payload，作为严格同 world-time counterfactual 的 telemetry。
4. RViz 增加 validated prefix 与 unvalidated future suffix 的独立 topic/颜色；完整未来 polynomial 不再与执行 authority 使用同一语义入口。

未增加新权重、horizon、安全阈值、A*、dynamic SFC、brake 或 stop fallback。

## 验证顺序与运行身份

确定性合同先于仿真执行，关键合同全部 PASS：

`joint_adoption_atomic_contract_test`、`committed_prefix_joint_contract_test`、`feedback58_barrier_regression_test`、`joint_failure_local_noop_contract_test`、`stale_joint_cas_contract_test`、`joint_future_tail_activation_contract_test`、`prediction_epoch_semantics_contract_test`、`visibility_topology_production_test`、`recovery_probe_production_test`。

构建命令：

```text
catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws
```

结果：6 个包成功，无失败。

最终 FULL ON 使用：

`unified_team_transaction_validation_20260913/committed_prefix_run3_feedback61`

场景为 `long_cylinder_forest_visibility_stress.json`，native RViz 开启，route 正常完成，退出码 0，运行时长约 80.20454 s。第一条运行因 recorder 继承了外部 Python 消息包而缺少结构化文件；该 harness 问题已通过当前 ALP `PYTHONPATH` 修正，随后只做了一条针对性最终 FULL ON。没有 OFF、A/B 或调参运行。

## 结果

```text
PREVIOUS_CODEX_CONTEXT_COMPRESSION_STATE_RECONSTRUCTED: YES
BASELINE: feedback_60
PRODUCTION_SOURCE_CHANGED: YES
NEW_FULL_SIMULATION_RUN: YES
FULL_ON_RUN_COUNT: 2 (first recorder-incomplete, second final evidence run)
SCENARIO_A_FULL_RUN_COMPLETED: YES
SCENARIO_A_ACCEPTANCE_PASS: YES

ROOT_FIX_1: post-3ACK/commit atomic team activation
ROOT_FIX_2: meaningful binary team visibility adoption gate
ROOT_FIX_3: joint-to-joint no-gain supersede rejection
ROOT_FIX_4: strict local counterfactual telemetry
ROOT_FIX_5: RViz validated-prefix authority visualization

FEEDBACK060_PARTIAL_ACTIVATION_FIXED: YES (final run)
TRUE_PARTIAL_TEAM_ACTIVATION_COUNT: 0
TEAM_ACTIVATION_CARDINALITY_1_COUNT: 0
TEAM_ACTIVATION_CARDINALITY_2_COUNT: 0
TEAM_ACTIVATION_CARDINALITY_3_COUNT: 68
POST_COMMIT_LOCAL_GENERATION_CAN_INVALIDATE_TEAM: NO
HARD_SAFETY_TEAM_WIDE_CANCEL_IMPLEMENTED: YES

JOINT_FRONTIER_OPPORTUNITY_COUNT: 161
JOINT_ATTEMPT_COUNT: 156
JOINT_SUCCESS_COUNT: 77
JOINT_SELECTED_COUNT: 68
JOINT_PROPOSAL_COUNT: 77
JOINT_3ACK_COUNT: 68
JOINT_COMMIT_COUNT: 68
JOINT_ACTIVATION_COUNT: 68
JOINT_REJECT_NO_MEANINGFUL_VISIBILITY_GAIN_COUNT: 3
JOINT_BINARY_PLATEAU_ACTIVATION_COUNT: 0
JOINT_STRICT_CAMERA_GAIN_ACTIVATION_COUNT: 68
JOINT_STRICT_ALL3_GAIN_ACTIVATION_COUNT: 68
JOINT_STRICT_WEAKEST_GAIN_ACTIVATION_COUNT: 68
STRICT_LOCAL_COUNTERFACTUAL_CAPTURED_COUNT: 78 proposals / 68 activated
JOINT_TO_JOINT_TIE_SUPERSEDE_COUNT: 0 observed

LOCAL_COMMIT_BLOCKED_BY_JOINT_COUNT: 0
JOINT_FAILURE_AFFECTED_LOCAL_COUNT: 0
TERMINAL_HOLD_COUNT: 0
STARVATION_COUNT: 0
END_BEFORE_NEXT_COUNT: 0
UNVALIDATED_COUNT: 0
COLLISION_COUNT: 0
SWARM_VIOLATION_COUNT: 0
```

三条被拒绝的 proposal 原因分别来自 stale planning generation、trajectory boundary mismatch 和静态重检失败；它们没有影响 local commit。完整 activation IDs 为：

`1,2,4,5,7,8,10-19,22-37,39-46,47,49-68,70-75,77`（按 JSONL 去重，实际集合以运行目录 recorder 为准）。末尾 solution 78 的 activation 时间晚于 route end，不计入实际 activation。

## 可见性与连续运动

最终运行 `visibility_summary.csv`：

```text
CAMERA_TIME: 219.329720 camera.s (time-integrated)
MEAN_VISIBLE: 2.733806
K2: 0.976052
ALL3: 0.757338
NONE: 0.000000
LONGEST_K2_LOSS: 0.666817 s
LONGEST_BLACKOUT: 0.000000 s

STATIC_LOS_LOSS_UAV0/UAV1/UAV2: 3.930544 / 1.267542 / 0.665851 s
DYNAMIC_LOS_LOSS_UAV0/UAV1/UAV2: 0.633250 / 2.265167 / 0.730302 s
HFOV/VFOV loss: recorder exposes combined camera_fov_valid; per-UAV combined FOV loss is 4.163446 / 3.521175 / 4.480092 s

ACTUAL_Q_DIR: recorder aggregate not emitted in final summary; source selection telemetry present
ENCIRCLEMENT_VALID_RATIO: 0.510012
SAME_SEMICIRCLE_RATIO: 0.395760
```

与 Feedback059 的 run-level 结果相比，camera-time 约高 0.032 s；MEAN_VISIBLE、K2、ALL3 轻微变化，不能仅凭不同 run 的差值做因果归因。最终 run 内所有 68 条采用的 Joint 都有严格 binary benefit；不存在 Feedback060 所见的 binary plateau adoption。

运动与安全：

```text
MAX_PLANNER_SPEED: 约 3.12 m/s（trajectory samples）
MAX_COMMAND_SPEED: 约 3.12 m/s（vx/vy/vz command norm）
MAX_ODOM_SPEED: 3.115159 m/s
MIN_STATIC_CLEARANCE: 0.184663 m
MIN_DYNAMIC_CLEARANCE: 0.773236 m
COLLISION_SAMPLES: 0
SWARM_CLEARANCE_VIOLATION_SAMPLES: 0
UNVALIDATED_EXECUTED_SAMPLES: 0
```

最终 Joint change 分布：`P_CHANGE_NORM` mean/p50/p95/max = `0.038869/0.031016/0.105427/0.153163`，`T_CHANGE_NORM` = `0.014961/0.007851/0.047736/0.058839`，`YAW_CHANGE_NORM` = `0.025923/0.001001/0.096510/0.154709`。actual optimized-yaw command verification 对 68 条均通过，最大误差低于 `9.18e-4 rad`。

## 生产语义结论

### 1. Partial activation

Feedback060 的 19/37/68 类型问题来自 proposal ACK 与 COMMIT publication 的传输竞态：某机在收到 COMMIT 前的普通 local successor 更新清除了 pending identity，另外机器已经开始执行。现在 pre-COMMIT NACK 保留 pending identity，coordinator 的 COMMIT/CANCEL 才能决定后续；COMMIT 后普通 generation 更新仅延迟到 team tail 之后，真实硬安全失败通过 `TEAM_COMMIT_REVOKE_REQUEST` 广播团队取消。因此最终运行 activation cardinality 只有 0 或 3，实测为 68 次 3/3、0 次 1/3 或 2/3。

### 2. Joint execution authority

continuous optimizer 仍用于求解 P/T/yaw 方向，但 coordinator 的 adoption gate 只接受同一 binary horizon 上的真实 visibility improvement：严格 camera-time，或 camera-time 不下降时严格 All3/weakest-camera improvement，同时 K2、NONE、最长损失和 blackout 不退化。geometry、Q_dir、target progress、continuous objective 只能保留为优化/telemetry，不能单独提交三机替换。最终运行中 3 条 proposal 被 `NO_MEANINGFUL_BINARY_VISIBILITY_GAIN` 拒绝，0 条 binary plateau 被激活。

### 3. Joint-to-Joint churn

新的 Joint 若与已 scheduled team tail 重叠，必须证明相同 world-time overlap 上的 meaningful binary team gain；tie 保留当前 tail。最终运行没有观测到 tie supersede；Local mandatory safety/liveness 仍可优先提交，失败 Joint 为 Local NO-OP。

### 4. Counterfactual telemetry

每条 proposal 记录三架 Local committed suffix 的 trajectory message、owner revision、trajectory id、frontier activation 和 polynomial payload，可供离线同 world-time A/B。最终 recorder 捕获 78 条 proposal 的 3-UAV baseline identity，其中 68 条进入实际 activation。它补齐了 Feedback060 的 `STRICT_LOCAL_COUNTERFACTUAL_AVAILABLE_COUNT=0` 缺口；本报告没有把不同 run 的总 camera-time 差异当作事件级因果结论。

### 5. RViz authority

`validated_optimal_list` 显示当前 execution-authority prefix；`unvalidated_future_list` 使用独立 namespace/topic、橙色半透明样式显示未认证 suffix；candidate/seed/history 不再与 authority 共用语义入口。trajectory 131 类场景中，完整未来 polynomial 即使几何穿柱，也不能出现在绿色 validated prefix 中；实际 odom 执行仍只受 validated prefix 约束。最终运行 `COLLISION_SAMPLES=0`，未观察真实 executed obstacle collision。

## 禁止项核对

```text
NEW_GLOBAL_ASTAR_ADDED: NO
DYNAMIC_SFC_ADDED: NO
SAFETY_THRESHOLD_RELAXED: NO
DYNAMIC_CLEARANCE_1P1_CHANGED: NO
LOCAL_PLANNER_FROZEN_FOR_JOINT: NO
UNIFIED_TRANSACTION_BARRIER_REINTRODUCED: NO
PLANNER_BRAKE_REINTRODUCED: NO
STOP_FALLBACK_REINTRODUCED: NO
STRICT_25_170_REINTRODUCED_AS_HARD: NO
RVIZ_VALIDATED_PREFIX_ONLY_DISTINGUISHED: YES
TRAJECTORY131_RVIZ_AUTHORITY_FIXED: YES (topic/namespace distinction)
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```

## 最终回答

1. Feedback060 的 partial activation 已在最终生产 run 中消失：68 个实际 team activation 全部是 3/3；普通 local generation 不再在 COMMIT 后单机取消 team tail，硬安全撤销是团队级 0/3。
2. 以前的 binary plateau Joint 不能再取得 execution authority；当前必须有严格 binary camera-time、All3 或 weakest-camera gain，且安全/连续性约束通过。最终 run 的 plateau activation 为 0。
3. 新 Joint 不能仅凭 Q_dir、geometry 或 continuous objective 高频覆盖已有 tail；重叠 tail 上必须有同窗口 meaningful binary gain，tie 保留现有 tail。
4. trajectory 131 的 validated prefix 与 unvalidated suffix 现在由不同 topic、namespace 和样式表示；红/橙未来线不再冒充绿色执行 authority。

本轮最终状态：生产源码已更新，确定性合同全绿，最终 FULL ON route 完成且安全门禁全零；Joint 失败完全不影响 Local，真实 Joint 执行链在 committed prefix 的 future frontier 上闭合。

REPORT_FILE: `/home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_61.md`
