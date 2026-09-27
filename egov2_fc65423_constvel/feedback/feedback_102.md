# Feedback 102 —— 每批一次的 Team visibility guide-point reserve

## 1. 范围与实现

承接 Feedback100/101。本轮只替换 Feedback101 的 Local Team reserve 实现；
Team T/PT-SCP、required margin、trigger、repair candidate 选择、activation lead、P/T trust、
RealizedTeamValidator 与 PREPARE/READY/COMMIT 均未改。

旧版 `addTeamVisibilityReserveGradCost2CT()` 在每次 L-BFGS 目标函数求值、每个时间样本上调用
`visibilitySampleAt()`，其中包含完整 static/dynamic LOS、FOV、range 与
`composeVisibilityMargin()`。这条调用已删除。现在：

1. 收到最新 Team schedule，仅当本机是 declared repair candidate 且契约 active 时保留
   `[t_cross,t_recover]`；单点 crossing 合法。无风险立即清空 reserve。
2. 每次普通 Local rolling 构造 nominal `initMJO` 后，**仅一次**调用
   `prepareTeamVisibilityReserveGuides(initMJO.getTraj())`。采样 crossing 的首点、中点、末点，
   去重和截断至 seed 真实时域，至多 3 次 `visibilitySampleAt()`，全部发生在 L-BFGS 之外。
3. 只接受 margin 不足、limiter 为 STATIC/DYNAMIC_LOS 且梯度有限非零的样本。
   沿已有 margin 的位置梯度，用既有 SIDE 0.7 m 偏移尺度生成 guide；遇到静态占据、
   SIDE region 或 Local-SFC 平面冲突时按 0.7/0.35/0.175 m 回退，仍不合法则跳过。
   FOV/RANGE 不生成 guide。
4. L-BFGS 内 reserve 仅求 `Σ w_reserve ||p(t_k)-p_ref_k||²` 与普通 MINCO
   系数/时间梯度；仍复用 `manager/team_reserve_weight`，没有新增权重或阈值。
   SIDE 候选使用原有 topology/SFC 权威，不施加 nominal guide；Team realization
   重锚定时清空 guide。下一轮 rolling 用最新 schedule 重算，没有持久状态机。
5. 修正一个实测中发现的时间基准错误：nominal initializer 的世界时间原点是
   `planning_prediction_epoch`，不能沿用上一条执行轨迹的 `start_time`。

逐点日志为 `[TEAM_RESERVE_GUIDE]`；每批有 `[TEAM_RESERVE_GUIDE_SUMMARY]`，
每次 reserve-active Local solve 有 `[TEAM_RESERVE_GUIDE_RESULT]`，记录 guide_count、
guide_cost、reserve_active、solve_success；原 `[TEAM_RESERVE_TRACE]` 保留。

源码位置：`planner_manager.cpp` 的 schedule callback、normal Local rolling 初始化，
`poly_traj_optimizer.cpp` 的 `prepareTeamVisibilityReserveGuides()` 与
`addTeamVisibilityReserveGradCost2CT()`。现有独立的 directional Local visibility 代价仍按原逻辑运行；
下文“FULL_VISIBILITY_EVAL_INSIDE_LBFGS=0”专指本轮替换的 **Team reserve 完整四分量查询**。

## 2. 构建与正式运行

最终版本执行 `catkin build -j2 --no-status`：25/25 packages succeeded，0 warning。
`git diff --check`：PASS。

相同命令运行两轮：

```bash
./run_on.sh --ablation full --headless --timeout 200 \
  --scenario /home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_visibility_stress.json
```

| run | FINAL_EXIT_CODE | BOOT-12 | reserve guide 批次 | 终态 trial | T/PT PASS | ADOPTED |
|---|---:|---|---:|---:|---:|---:|
| `20260923_124859_252874` | 0 | YES | 43 | 4 | T=1 / PT=0 | 1 |
| `20260923_125315_270760` | 0 | YES | 3 | 0 | 0 | 0 |

两轮的日志、运行参数、visibility summary 均在各自 `runs/<RUN_ID>/` 下。
第二轮有 active reserve 和 3 个 guide 批次，但无 `[RELAY_LATENCY]` 终态样本；
因此投递时延统计只能由第一轮的 4 个终态样本支持。

## 3. 时延与效果

统计按日志值排序取下标 `floor(q*(n-1))`；以下 p50/p95/max 单位为 ms，除非另注明。

| 指标 | run 124859 | run 125315 | 合并 |
|---|---:|---:|---:|
| guide points / rolling，p50/p95/max | 1 / 2 / 3 (n=43) | 1 / 1 / 1 (n=3) | 1 / 2 / 3 (n=46) |
| guide points / solve，p50/p95/max | — | — | 0 / 2 / 3 (n=113；SIDE/越界含 0；26 次非零) |
| guide cost / solve，p50/p95/max | 0 / 19.366 / 27.983 | 0 / 5.193 / 7.057 | 0 / 19.366 / 27.983 |
| detect→receive，p50/p95/max | 19.724 / 33.731 / 34.780 (n=4) | 无终态样本 | 同左 |
| Local planning，p50/p95/max | 1.877 / 50.593 / 3620.584 | 1.247 / 5.177 / 135.394 | 1.608 / 44.829 / 3620.584 |

**时延判断**：有终态 trial 的第一轮，detect→receive 已从 Feedback101 的
307–1155 ms 回到 Feedback100 的几十毫秒量级；但只有 4 个样本，第二轮没有样本，
无法证明稳定性。Local planning 第一轮发生 3.62 s 的尾部故障，故“整体时延已恢复”不能成立。

Feedback101 的 `[TEAM_RESERVE_TRACE]` 口径下，本轮四个 forecast→SCP seed decay 为
`-1.810239 / -3.904100 / -4.609868 / -1.877747`，全为负值。不过这个字段的
forecast 是 **M2 limiting margin**，seed 是各节点自身 window minimum，不能当作
逐 UAV 同一量的严格配对，更不能由此证明 guide 因果效果。用第一轮各 UAV 自己的
forecast margin 与 `TEAM_REPAIR_ELIGIBILITY.seed_window_min` 配对，能确认的 candidate 样本
同时出现 `+0.608`、`+1.008`（恶化）和 `-4.677`、`-0.094`（改善）；
**逐 UAV 的恶化尚未稳定抑制**。第二轮没有 SCP seed 终态样本。

第一轮 contract 2 的 drone1 有真实 `TEAM_SCP_T_PASS`：
`margin 0.297642 → 0.995232`，`dt=0.397297 s`，`dp=0`，
`M2_after=0.995232`；`TEAM_REFINEMENT_RESULT=ADOPTED`，
随后同一 team_solution_id=98 在三个 UAV 上出现 `stage=ACTIVATION`。
这是一次非零 T 修复及实现的 M2 改善，但不能单独归因于 guide reserve。

## 4. 安全与活性

| 指标 | run 124859 | run 125315 |
|---|---:|---:|
| TRUE_CONSTRAINT_INFEASIBILITY / SOLVER_INFEASIBLE | 3 | 0 |
| MIN_BUDGET_APPLIED | **18655** | 0 |
| TERMINAL_HOLD_ENTER | **6** | 0 |
| MOVING_SUCCESSOR_STARVATION | **2** | 0 |
| PVA_MISMATCH / PARTIAL_TEAM_ACTIVATION / UNVALIDATED | 0 / 0 / 0 | 0 / 0 / 0 |
| 明确 collision / swarm violation 日志 | 0 / 0 | 0 / 0 |
| uav1/2/3 visibility ratio | 0.883796 / 0.970370 / 0.971759 | 0.924025 / 1.000000 / 0.981520 |
| 最长 k2 loss / blackout (s) | **1.468141 / 0.951563** | 0 / 0 |

第一轮的 3.620584 s Local planning 尾值发生在
`1790139020.262`，随即出现 successor starvation 和高频 current-state restart。
最后一次 `[TEAM_RESERVE_CLEARED]` 在 `1790138996.487`，相隔约 23.8 s。
因此无法把后段故障直接归因于 guide 采样，但也不能排除轨迹状态的间接影响；
本轮没有足够证据宣布活性安全恢复。第二轮未复现。
硬安全日志未见碰撞、swarm 违规、未验证执行、PVA mismatch 或部分 Team 激活。

## 5. 遗留问题与结论

1. **活性回退未闭环**：第一轮 18655 次 MIN_BUDGET_APPLIED、6 次 terminal hold、
   2 次 successor starvation，超过 Feedback101 最坏的 5115 / 6 / 0。
   触发点已离开 reserve 窗口；需要针对 Local planning 长尾与 restart 链做独立定位。
2. **逐 UAV forecast→seed 恶化仍有正样本**；guide 点虽然产生且 cost 非零，
   目前只能证明它进入了 Local 目标函数，无法证明普遍抑制 baseline 恶化。
3. **投递时延样本有限**：一轮 n=4 为几十毫秒，另一轮无终态 trial。
4. `[TEAM_RESERVE_TRACE]` 仍主要在 trial 判定路径产生，且旧字段
   `forecast_margin` 与 `scp_seed_margin` 不保证同一 UAV 语义。

```text
OLD_EXPENSIVE_RESERVE_REMOVED: YES
GUIDE_POINT_RESERVE_IMPLEMENTED: YES
FULL_VISIBILITY_EVAL_INSIDE_LBFGS: 0  # Team reserve 的 visibilitySampleAt/composeVisibilityMargin 调用
GUIDE_POINTS_PER_SOLVE_P50_P95_MAX: 0 / 2 / 3  # n=113；非零 solve 为 1 / 3 / 3，n=26
DETECT_TO_RECEIVE_P50_P95_MAX: 19.724 / 33.731 / 34.780 ms  # n=4，第二轮无终态样本
LOCAL_PLANNING_LATENCY_P50_P95_MAX: 1.608 / 44.829 / 3620.584 ms  # 两轮合并
FORECAST_TO_SEED_DECAY: -1.810239 / -3.904100 / -4.609868 / -1.877747  # 旧 TRACE 口径；逐 UAV 样本有正有负
TRUE_INFEASIBLE_COUNT: 3  # 两轮合计，无法证明相对 Feedback101 下降
RESERVE_RECOVERED_BEFORE_SCP: 0 CONFIRMED  # 一次 seed 已高于 required，但 guide 因果未证实
NONZERO_TEAM_REFINEMENT_ADOPTED: 1
REALIZED_M2_IMPROVEMENT_COUNT: 1
MIN_BUDGET_APPLIED: 18655  # 第一轮 18655，第二轮 0
TERMINAL_HOLD_ENTER: 6  # 第一轮 6，第二轮 0
SAFETY_REGRESSION: YES  # 硬安全计数为 0；活性与可见性长尾恶化
GUIDE_POINT_RESERVE_EFFECTIVE: NO  # 实现与有样本时延达标；整体活性及逐 UAV 恶化未达标
CURRENT_MAIN_BOTTLENECK: Local planning 长尾及其 current-state-restart 连锁；直接原因尚未定位
```
