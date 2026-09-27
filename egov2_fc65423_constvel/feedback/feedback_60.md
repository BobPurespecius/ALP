# Feedback 060 — Joint 协同可见性因果审计

日期：2026-09-14  
审计性质：只读离线分析  
基线 run：`unified_team_transaction_validation_20260913/committed_prefix_run1`  

## 范围与可信度

本轮没有修改 production source，没有调参，没有启动 ROS/Gazebo/RViz，也没有启动新的 FULL ON。只读取当前 ALP 工作树、`feedback_56.md`–`feedback_59.md`、run 的 JSONL/CSV/launcher log 和当前源码文本。新增的唯一代码是只读分析脚本：

`joint_visibility_causal_audit_20260914/analyze_joint_visibility_causality.py`

输出目录：

`joint_visibility_causal_audit_20260914/analysis/`

上一轮声称的 `committed_prefix_run1` 经过 `run_meta.json`、launcher、CSV、recorder 交叉核对，确实是 COMPLETE：mission `1789306724.263139`–`1789306804.4676797`，route stop reason 为 `target_replay_normally_completed`，exit code 0。当前没有 ROS、RViz、Gazebo、planner 或 recorder 进程。

## 事件级产物

已生成：

- `analysis/joint_solution_event_table.csv`：62 条完整 Joint + 3 条 partial 的逐 solution 事件表；
- `analysis/joint_counterfactual_metrics.csv`：严格 Local A/B 可用性；
- `analysis/joint_post_activation_windows.csv`：实际 activation 后 1.5/3/5 s 与 until-next 窗口；
- `analysis/joint_classification.json`、`partial_activation_audit.json`、`trajectory131_static_audit.json`、`summary.json`。

`team_solutions.jsonl` 的 `team_solution_id` 与 launcher 的 `binary-camera-time generation` 不是同一编号；脚本按 `coordination_generation` 关联二值 acceptance telemetry，并按 `team_solution_id` 关联真实 activation。

## 62 条 Joint 的计划阶段结果

`team_solutions.jsonl` 与 launcher 的 `[binary-camera-time]` 对齐结果：

```text
ACTUAL_COMPLETE_JOINT_COUNT: 62
STRICT_LOCAL_COUNTERFACTUAL_AVAILABLE_COUNT: 0
STRICT_LOCAL_COUNTERFACTUAL_UNAVAILABLE_COUNT: 62

JOINT_PREDICTED_BETTER_CAMERA_COUNT: 8
JOINT_PREDICTED_EQUAL_CAMERA_COUNT: 53
JOINT_PREDICTED_WORSE_CAMERA_COUNT: 0
UNKNOWN_BINARY_ASSOCIATION_COUNT: 1
JOINT_PREDICTED_BETTER_ALL3_COUNT: 7
JOINT_PREDICTED_BETTER_K2_COUNT: 2
```

严格 Local counterfactual 为 0，不是因为所有 Local 都相同，而是 recorder 没有为每个 Joint solution 保存完整、可重放的 Local polynomial/tail 与同一 world-time 输入。`team_solutions` 的 before/after 是 optimizer reference metrics，不等价于“关闭 Joint 后会执行的 Local”。因此不能用它证明 62 次 Joint 相对于 Local 的因果收益。

8 条有二值 camera-time 提升的 solution 是真实的计划阶段提升；53 条是二值 camera-time/All3/K2 数值平台上的 tie；1 条的二值字段在 solution/generation 对齐处 telemetry 不完整，不能分类为 worse。不能把 `JOINT_SUCCESS_COUNT=76` 当成 visibility benefit：成功只表示 optimizer/preflight/transaction 链闭合。

代表性 source-level 证据是 [multi_uav_topology_coordinator.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/src/multi_uav_topology_coordinator.cpp:1725)：先计算 `actual_before/actual_after`，然后 `actual_benefit` 可由 `targetProgress`、`measurableBenefit` 或 geometry progress 满足；`cameraTimeAcceptance` 只要求 continuity，不要求严格 camera-time 提升（:1732–1734）。[team_visibility_optimizer.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/src/team_visibility_optimizer.cpp:582) 明确允许 binary camera plateau 上的 target/geometry/measurable benefit 通过。当前 launch 参数中 `team_camera_time_epsilon_improve=1e-6`、`team_minimum_trajectory_change=1e-5`，因此平台 tie 很容易成为可提交 Joint。

结论：

```text
IS_JOINT_SELECTION_WRONG_AT_DECISION_TIME: PARTIAL
IS_JOINT_ADOPTION_TOO_AGGRESSIVE: YES (strong evidence)
```

“wrong”不能对所有 62 条成立，因为缺少 Local A/B；但 53/62 的实际提交没有二值 visibility gain，说明 adoption authority 接受了大量 tie/非 camera benefit。8 条严格 camera gain 说明 optimizer 并非完全选错。

## 实际执行后的窗口

按真实 `trajectory.csv` 的 `team_solution_id` 和 `visibility.csv` 的 mission-relative time 关联，62 条完整 activation 的平均 Joint 标记执行时长为约 `0.3064 s`，p50 `0.2802 s`，p95 `0.3667 s`；相邻完整 Joint activation 间隔均值约 `1.2306 s`，p50 `0.6455 s`，最小 `0.4246 s`。因此系统频繁在 1.5 s Joint evaluation horizon 尚未完整发挥前就被后续 rolling decision 覆盖。

实际窗口均值（后续规划会混入，不能当严格因果结果）：

| 窗口 | mean visible | All3 fraction | K2 fraction | None fraction |
|---|---:|---:|---:|---:|
| 1.5 s | 2.7694 | 0.7962 | 0.9753 | 0.0022 |
| 3.0 s | 2.8035 | 0.8213 | 0.9833 | 0.0011 |
| 5.0 s | 2.7877 | 0.8062 | 0.9822 | 0.0006 |
| until next complete Joint | 2.7458 | 0.7686 | 0.9780 | 0.0008 |

这些数字显示短窗口并没有证明长期损失，也没有足够 telemetry 证明 1.5/3/5 s 的 degradation 是由某一 Joint 引起。故：

```text
SHORT_TERM_GAIN_LONG_TERM_LOSS_COUNT: UNKNOWN
IS_PREDICTION_EXECUTION_GAP_PRIMARY: UNKNOWN / insufficient paired prediction telemetry
IS_RECEDING_HORIZON_MYOPIA_PRIMARY: PARTIAL / strong frequency evidence, causal effect unproven
```

不能把 Feedback57 的 229.517925 camera-s 与 Feedback59 的 219.297632 camera-s 直接归因于 Joint；两次 run 不是同一随机/状态轨迹。Feedback59 run-level 值为：camera-time `219.297632`、mean-visible `2.736043`、K2 `0.976386`、All3 `0.760399`、NONE `0.000743`。它们只能作为 run-level association。

## 哪架 UAV 失去可见性

整条 run 的 visibility CSV 中不可见样本数为：UAV1 `321`、UAV2 `195`、UAV3 `118`。这是全局统计，不是 post-Joint 严格归因；现有 recorder 没有保存“Joint 后唯一不可见 UAV + 原因”的事件级字段。因此：

```text
POST_JOINT_UNIQUE_INVISIBLE_UAV0: UNKNOWN
POST_JOINT_UNIQUE_INVISIBLE_UAV1: UNKNOWN
POST_JOINT_UNIQUE_INVISIBLE_UAV2: UNKNOWN
PRIMARY_POST_JOINT_VISIBILITY_FAILURE_CAUSE: UNKNOWN
```

源码上，Joint continuous objective 同时包含 accumulated visibility、K2/K2-cont、blackout、deviation、jerk、multiview/Q_dir 与 encirclement 项；binary All3 不是唯一主导项。当前实现没有一个“weakest third camera 必须严格改善”的独立 acceptance authority。这个结构可以解释为什么整体 All3 仍明显低于 Feedback57，但不能从当前 run 单独证明具体的 UAV/静态 LOS/FOV 因果链。

## Partial activation 19 / 37 / 68

这三条不能按完整 atomic activation 计数：

```text
PARTIAL_19_ROOT_CAUSE:
  UAV0/UAV1 ACK、commit、EARLY_JOINT_ACTIVATED；UAV2 随后发送
  LOCAL_GENERATION_INVALIDATED:LOCAL_SUCCESSOR_SUPERSEDED_GENERATION，未完成 activation。

PARTIAL_37_ROOT_CAUSE:
  UAV1/UAV2 actual activation；UAV0 在 activation 前被
  LOCAL_GENERATION_INVALIDATED:LOCAL_SUCCESSOR_SUPERSEDED_GENERATION 拒绝。

PARTIAL_68_ROOT_CAUSE:
  UAV0/UAV1 actual activation；UAV2 同样在 ACK-send 阶段被
  LOCAL_GENERATION_INVALIDATED:LOCAL_SUCCESSOR_SUPERSEDED_GENERATION 拒绝。
```

三条都有 proposal、`EARLY_JOINT_3ACK`/commit 日志，但最终只看到两架 `EARLY_JOINT_ACTIVATED`。因此：

```text
TRUE_PARTIAL_TEAM_ACTIVATION_OCCURRED: YES (CRITICAL)
```

这与“62 条 complete chain”并不矛盾：62 是三架都完成 activation 的去重集合；19/37/68 被单独排除。该事实表明 proposal/3ACK 之后仍允许每机本地 generation invalidation 在共同 activation 前制造部分采用，属于协议/日志 authority 风险。本轮按要求只诊断，不修复。

## Trajectory 131 与 RViz 红轨迹

既有离线几何审计确认：UAV0 trajectory 131 的完整 scheduled polynomial suffix 与 cylinder 9 相交，首次几何命中 local `t=2.468035 s`；实际执行只到 local `t=0.349233 s`，实际 executed prefix 不相交，online collision samples 为 0。

当前 planner 明确采用 validated-prefix 语义：

- [planner_manager.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:1970) 的 `setLocalTrajFromOpt()` 先调用 `validateCommittedPrefixUntil(activation, ...)`，再调用完整 `validateExecutionTrajectory(traj, ...)`；
- [planner_manager.cpp](/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:4505) 的 retry 注释明确写着：只有 terminal local trajectories 要求 full-trajectory static-map safety；普通 execution 使用 executable-prefix contract；
- [planner_manager.cpp](/home/bob/ALP/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp:1868) 附近把 suffix invalidation 转换为 `execution_safe_until_` 的 prefix certificate。

因此当前最符合证据的是：

```text
TRAJECTORY_131_FULL_SUFFIX_INTERSECTS_STATIC_OBSTACLE: YES
TRAJECTORY_131_EXECUTED_PREFIX_INTERSECTS: NO
TRAJECTORY_131_SAFETY_AUTHORITY_EXPLANATION: VALIDATED_PREFIX_ONLY_BUT_RVIZ_SHOWS_UNVALIDATED_SUFFIX
ACTUAL_EXECUTED_OBSTACLE_COLLISION_OBSERVED: NO
CANDIDATE_OR_HISTORY_MARKER_COLLISION_OBSERVED: YES
```

这不是“实际碰撞被 collision counter 掩盖”，而是 authority/visualization 不一致：RViz recorder 将完整 scheduled polynomial 作为红色 optimal list 显示，而执行 authority 只覆盖 safe prefix；旧/被 supersede 的 future suffix 没有足够清晰的 active/source/revision 可视身份。用户看到的穿柱红线主要是未执行的未来 suffix，不是实际 odom 碰撞。当前证据不足以把它升级为 `PLANNER_FULL_TRAJECTORY_STATIC_SAFETY_BUG`，但它是明确的可视化和 telemetry 缺口。

## 根因排序

### ROOT_CAUSE_1 — CONFIRMED：Joint adoption 对 binary plateau 过宽

证据：53/62 条完整 activation 在二值 camera-time、All3、K2 上没有严格提升；源码允许 target/geometry/objective/multiview benefit 在 camera plateau 上通过。它解释了大量 Joint activation，但不能单独解释 run-level camera-time 比 Feedback57 低多少。

### ROOT_CAUSE_2 — STRONG EVIDENCE：receding-horizon 高频覆盖

证据：Joint horizon 1.5 s，而完整 Joint 平均约 0.306 s 就被新的 trajectory marker/rolling decision 替换；activation interval p50 约 0.646 s。系统会持续追逐短时可接受解。它支持 horizon myopia/over-adoption，但现有 recorder 缺少固定 world-time 的 no-Joint A/B，不能宣称已证明 3–5 s causal loss。

### ROOT_CAUSE_3 — CONFIRMED CRITICAL：3 ACK/commit 后的 partial adoption

证据：IDs 19/37/68 各只有两架最终 `EARLY_JOINT_ACTIVATED`，另一架被 `LOCAL_GENERATION_INVALIDATED` 拒绝。它不是 Feedback58 local barrier，也没有造成本 run 的 hold/starvation，但它破坏了“team solution atomic activation”的协议假设，可能污染联合观察质量。

### ROOT_CAUSE_4 — CONFIRMED AUTHORITY/VISUALIZATION MISMATCH：validated prefix 与整条 polynomial 显示不一致

证据：trajectory 131 full suffix 几何穿柱，executed prefix 不穿柱；源码把 prefix certificate 作为执行 authority，但 RViz 红 marker 展示 full scheduled polynomial。它解释用户目视的穿柱红线，不能解释实际 collision counter 为 0。

### ROOT_CAUSE_5 — UNKNOWN：prediction→execution gap

当前 run 没有按 solution 保存 predicted per-UAV binary trace、actual execution trace、target/obstacle snapshot 的成对记录；只能知道全局 visibility，不能判断每一条 Joint 是预测错、yaw/timing 错还是后续 rolling 混入。

## 明确回答五问

1. 62 条里，至少 8 条在采用当下的 production binary before/after 有严格 camera-time 提升；53 条是 binary plateau；由于 Local counterfactual 缺失，不能说“大多数比 Local baseline 更好”。
2. 整体 visibility 较低不能从 run-level 差值直接归因于 62 条 Joint。最强可见证据是 plateau adoption + 高频 horizon 覆盖；prediction gap 与长期退化仍 UNKNOWN。
3. 是。53/62 的二值 visibility 没有提升，且 Joint 平均约 0.306 s 即被新决策覆盖，构成明确 over-adoption evidence。
4. 全局不可见样本最多的是 UAV1，其次 UAV2/UAV3；但具体 post-Joint 唯一第三架及 STATIC_LOS/DYNAMIC_LOS/HFOV/VFOV 原因，当前 telemetry 不足，不能伪造结论。
5. trajectory 131 是“validated prefix authority 但 RViz 显示未验证 suffix”，不是已观察到的实际执行碰撞；完整 suffix 仍确实穿柱，必须把它视为可视化/authority 语义缺口。

## 最终状态

```text
PRODUCTION_SOURCE_CHANGED: NO
NEW_FULL_SIMULATION_RUN: NO
FULL_ON_RUN_COUNT: 0

IS_JOINT_SELECTION_WRONG_AT_DECISION_TIME: PARTIAL
IS_PREDICTION_EXECUTION_GAP_PRIMARY: UNKNOWN
IS_RECEDING_HORIZON_MYOPIA_PRIMARY: PARTIAL
IS_JOINT_ADOPTION_TOO_AGGRESSIVE: YES

LOCAL_COMMIT_BLOCKED_BY_JOINT_COUNT: 0
JOINT_FAILURE_AFFECTED_LOCAL_COUNT: 0
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
JOINT_ACTIVATION_COUNT: 62 complete / 3 partial

CAMERA_TIME: 219.297632
MEAN_VISIBLE: 2.736043
K2: 0.976386
ALL3: 0.760399
Q_DIR: 0.950894
ENCIRCLEMENT: 0.545846

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```

本轮没有提出或实施修复。下一轮若要闭合因果链，必须先增加每个 solution 的 Local counterfactual polynomial/hash、Joint predicted binary per-UAV trace、固定 world-time actual windows、以及 3 ACK 后每架 activation 的不可篡改事件身份；在此之前继续调 visibility 权重或 horizon 都没有可审计依据。
