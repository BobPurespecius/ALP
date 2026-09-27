# Feedback 114 — K3 executed recovery 锁存与单次 FULL ON 复核

日期：2026-09-25。工作区：`/home/bob/ALP/egov2_fc65423_constvel`。本轮只修 Feedback113 已定位的 recovery observation 漏记；production 源码在正式仿真前已完成修改和构建，仿真后未再改动。没有运行 OFF，也没有第二次 FULL。

## 修复与离线验证

旧 checker 仅在 planner rolling 时读取每机最新 odometry，并用半开 event window 过滤。若窗口内可见样本被随后窗口外的样本覆盖，恢复事实便丢失。修改了 [event state 与 odom state](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/include/plan_manage/planner_manager.h) 和 [planner_manager.cpp](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp)：复用既有 `recovered`/`recovery_world_time`，增加实际轨迹 ID；本机 odom 回调收到有有效 header stamp 的样本时立即调用同一个 K3 checker，只锁存 visibility evidence，不作候选选择或提交。本机订阅队列由 1 增至有界 32，以便 planner 忙时保留短暂排队的 odom 样本。checker 保留 world time、revision、M2 优先权、已激活轨迹身份和统一 Local range/static LOS/dynamic LOS/FOV 判断；证据一旦锁存，本 event 内不会被后来的样本清除。`[K3_RECOVERY_EVIDENCE]` 和 event result 现在记录样本时刻、窗口、轨迹 ID 和最后 odom stamp。

[Feedback113 离线 replay](artifacts/feedback114_recovery_replay.json) 使用其保存的实际 visibility/trajectory CSV：event 4 在 `1790334586.518644` 取得窗口内可见证据，原 `EVENT_EXPIRED` 应改判 `RECOVERY_OBSERVED`。旧的 6 个 recovery 可见事实保留；窗口外首次可见不倒灌；6 个 M2 preempt result 没有被改判。历史 M2 合约的逐样本内部状态并未保存在 CSV，因此后项同时依据未改动的 M2 source 分支核对。[replay 脚本](artifacts/replay_feedback114_recovery.py) 与本次 [执行审计脚本](artifacts/analyze_feedback114_final.py) 已保留。

`git diff --check` 通过；`catkin build -j2 --no-status` 为 **25/25 packages succeeded，package warnings 0**。

## 唯一一次 FULL ON

使用 Feedback113 同一场景 `natural_team_stress_dense_38_targeted_k2_v6.json`，canonical runner 参数为 `--ablation full --headless --timeout 240 --boot-timeout 180 --k3-repair on --k3-escalation on`。RUN_ID：`20260925_200523_419199`。BOOT-12 到达，`FINAL_EXIT_CODE=0`；runner 登记的 28 个自有进程全部退出，强杀 0、残留 0、外部进程被杀 0。**退出码只表明 runner 完成，不代表安全/活性验收通过**，下文记录了 terminal hold。

按 ROS 节点身份区分同名 event ID，本 run 有 25 个 Local K3 event、8 条 recovery evidence、8 个 `RECOVERY_OBSERVED`、7 个 `M2_PREEMPTED`、10 个 `MARGIN_RECOVERED`，无 `EVENT_EXPIRED`。仍有 3 个 `PROGRESS_BUT_NOT_RECOVERED` failure stage，均因 margin 提前恢复而关闭；其 event 关闭时间早于预测窗口起点，不能当成窗口内 executed recovery 漏记。

| UAV/event | 锁存 world time | 已激活轨迹 ID | 执行 CSV 首个后续可见延迟 | 关闭结果 |
|---|---:|---:|---:|---|
| UAV1 / 2 | 1790337939.251861 | 33 | 0.089 s | RECOVERY_OBSERVED |
| UAV1 / 4 | 1790337942.482151 | 44 | 0.055 s | RECOVERY_OBSERVED |
| UAV1 / 6 | 1790337956.742251 | 96 | 0.059 s | RECOVERY_OBSERVED |
| UAV1 / 7 | 1790337962.501644 | 105 | 0.044 s | RECOVERY_OBSERVED |
| UAV1 / 8 | 1790337970.821946 | 128 | 0.046 s | RECOVERY_OBSERVED |
| UAV1 / 11 | 1790337995.521632 | 213 | 0.047 s | RECOVERY_OBSERVED |
| UAV2 / 2 | 1790338005.282987 | 263 | 0.019 s | RECOVERY_OBSERVED |
| UAV2 / 3 | 1790338005.403185 | 264 | 0.032 s | RECOVERY_OBSERVED |

所有证据的世界时间落在其当轮记录的窗口内，轨迹 ID 与最近执行轨迹记录一致，执行 visibility CSV 在随后 19–89 ms 内变为可见。**7/8 条在证据 timestamp 最近的独立 visibility CSV sample 尚未可见**；这两个观测流存在可见性切换时差，不能把上表称为同一时刻逐样本完全一致，也没有证据表明持续性 false positive。UAV1/event 4 对应轨迹 44，锁存后于 `1790337942.760577` 以 `binary_k3_recovered=1`、`RECOVERY_OBSERVED` 关闭；执行 CSV 在 `1790337942.537086` 开始可见，轨迹 CSV 在 `1790337942.538688` 仍为 ID 44 且 `safety_validated=True`。

按 **event 实际存续时间与初始记录窗口的交集** 审计，未发现“窗口内 blocked→visible，但最终 `binary_k3_recovered=0`”的事件，确认漏记数为 0。部分后续 rolling 窗口会延长而没有逐轮 end telemetry；因此 0 是现有证据可确认的计数，不能声称已穷尽所有中间窗口状态。[逐事件 JSON](artifacts/feedback114_final_metrics.json) 保存了时间、关闭原因与这一限制。

## 可见性与安全结果

| 指标 | Feedback113 | 本 run | 描述 |
|---|---:|---:|---|
| ALL3 | 0.903748 | **0.889646** | 下降 0.014102 |
| K2 | 0.997871 | **0.990626** | 下降 0.007245 |
| K3 loss 总时长 | 7.531 s | **8.633 s** | 增加 1.102 s |
| 最长 ALL3 loss | 1.766540 s | **1.370782 s** | 缩短 0.396 s |
| 最长 K2 loss | 0.166836 s | **0.749216 s** | 增加 0.582 s |
| 已记录 recovery | 6 | **8** | 单次 run 方差下只作描述 |

本次 2347 个 visibility sample；K3 loss 时长按相邻样本左端状态零阶积分。单次 run **没有证明 K3 指标改善**。

静态/动态接触、swarm violation、PVA mismatch、partial Team activation、successor starvation 均为 0；最小静态/动态净空为 **0.247660 / 0.309117 m**，最小机间距 **1.322543 m**。但 UAV1 的执行轨迹 CSV 出现 **75 条 `TERMINAL_HOLD`、`safety_validated=False` 样本**（轨迹 103 的 74 条，轨迹 364 的 1 条）；日志有 **2 次 `TERMINAL_HOLD_ENTER`**、2 次 EXIT 和 **57 次 `MIN_BUDGET_APPLIED`**。这使 `UNVALIDATED_EXECUTED=75`，相对 Feedback113 的 0 构成安全/活性回退，即使没有几何接触。最早失败发生于 `1790337959.879148`：UAV1 轨迹 103 到达 end time `1790337959.871622`，traj server 报 `TRAJECTORY_END_BEFORE_NEXT_ACTIVATION`、`run_fail=1`，随后约 2.480 s 的 terminal hold。现有证据证明 successor coverage 断档，不能仅凭本次 run 把它归因于 K3 latch 修改。

## 收尾

```text
PRODUCTION_CODE_CHANGED: YES  # 仅 K3 recovery observation / telemetry 与本机 odom 队列
BUILD_PASS: YES  # 25/25
RECOVERY_EVIDENCE_LATCH_IMPLEMENTED: YES
FEEDBACK113_EVENT4_REPLAY_PASS: YES
PREVIOUS_RECOVERY_EVENTS_PRESERVED: 6/6 historical visible facts
OUT_OF_WINDOW_FALSE_RECOVERY_COUNT: 0 (offline replay)
M2_PREEMPT_FALSE_RECOVERY_COUNT: 0 (offline replay)

FULL_ON_RUN_COMPLETED: YES
FULL_RUN_COUNT_THIS_TASK: 1
RUN_ID: 20260925_200523_419199
K3_LOCAL_EVENTS: 25
K3_RECOVERY_EVIDENCE_COUNT: 8
K3_RECOVERED_EVENTS: 8
MISSED_EXECUTED_K3_RECOVERY_COUNT: 0 confirmed in event lifetime × initial logged window
PROGRESS_BUT_NOT_RECOVERED: 3  # all MARGIN_RECOVERED before predicted window
EVENT_EXPIRED: 0
M2_PREEMPTED_K3: 7

EXECUTED_ALL3: 0.889646
EXECUTED_K2: 0.990626
K3_LOSS_TOTAL_TIME: 8.632627 s
LONGEST_ALL3_LOSS: 1.370782 s
LONGEST_K2_LOSS: 0.749216 s
FEEDBACK113_ALL3: 0.903748
FEEDBACK113_K2: 0.997871
FEEDBACK113_K3_LOSS_TOTAL_TIME: 7.531 s
FEEDBACK113_LONGEST_ALL3_LOSS: 1.766540 s
FEEDBACK113_LONGEST_K2_LOSS: 0.166836 s

STATIC_CONTACT: 0
DYNAMIC_CONTACT: 0
SWARM_VIOLATION: 0
PVA_MISMATCH: 0
UNVALIDATED_EXECUTED: 75  # TERMINAL_HOLD source, safety_validated=False
PARTIAL_TEAM_ACTIVATION: 0
STARVATION: 0  # no SUCCESSOR_STARVATION event; coverage nevertheless expired
TERMINAL_HOLD: 2 enters / 75 executed samples
MIN_BUDGET_APPLIED: 57
MIN_STATIC_CLEARANCE: 0.247660 m
MIN_DYNAMIC_CLEARANCE: 0.309117 m
MIN_SWARM_SEPARATION: 1.322543 m

K3_RECOVERY_OBSERVATION_BUG_FIXED: YES for Feedback113 failure mode; exhaustive extended-window proof unavailable
K3_METRIC_IMPROVED: NO
SAFETY_REGRESSION: YES  # terminal hold and unvalidated execution
FIRST_CAUSAL_FAILURE: UAV1 trajectory 103 expired at 1790337959.871622 before successor activation; terminal hold entered at 1790337959.879148
NEXT_STEP: independently audit successor coverage / terminal hold; retain event-window telemetry for exhaustive missed-recovery audit; no further change in this task
REPEATED_FULL_SIMULATION_USED: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```

证据：[run 退出状态](../runs/20260925_200523_419199/exit_status.txt)、[进程清理](../runs/20260925_200523_419199/process_status.txt)、[K3 节点日志](../runs/20260925_200523_419199/feedback114_k3_node_events.log)、[lifecycle 摘录](../runs/20260925_200523_419199/feedback114_lifecycle_excerpt.log)、[执行 visibility](../runs/20260925_200523_419199/visibility.csv)、[执行轨迹](../runs/20260925_200523_419199/visibility_trajectory.csv)、[压缩完整 stdout](../runs/20260925_200523_419199/roslaunch_stdout.log.gz)、[压缩 ROS 日志](../runs/20260925_200523_419199/ros_log.tar.gz)。原始重复日志已在核对后压缩；run 目录约束于持久盘，未使用 tmpfs 作为 ROS_LOG_DIR。
