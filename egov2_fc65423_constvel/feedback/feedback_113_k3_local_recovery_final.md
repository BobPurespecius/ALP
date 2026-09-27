# Feedback 113 — K3 Local recovery 最终 FULL ON 审计

日期：2026-09-25。工作区：`/home/bob/ALP/egov2_fc65423_constvel`。本任务只执行了一次有效 FULL ON；没有跑 OFF 或第二次 ON。正式运行后没有再改 production 源码。

## 结论

Local 已独立产生 17 个 K3 风险事件，Team K3 proposal 在生产路径中降为遥测，M2 仍有优先权。N/L/R、C3/D3、提交和执行激活均实际发生。可见性、硬安全和进程清理结果合格，但 K3 链**尚未闭环**：第一个非 M2 终止失败事件中，执行轨迹已在窗口末端恢复 K3 可见性，Local recovery checker 却漏记了窗口内恢复，将其关闭为 `PROGRESS_BUT_NOT_RECOVERED`。本轮据此停止，不补丁、不重跑。

## 源码审计与本轮修改

- `LocalVisibilitySample` 是 K3 风险、候选 C3/K2/D3 和 binary 可见性的共同几何 authority。target、动态障碍和 Local sample 都按 world time 求值；peer 使用该时刻的执行轨迹，yaw 从有时间戳的 odometry 推进。17 条 `[K3_WORLD_TIME_AUDIT]` 的 target anchor error 最大为 **0 m**。
- 本机 Local margin crossing 创建事件，不要求 Team failure。M2 active contract 在 Local event 更新、激活确认和候选 dispatch 前抢占 K3；本轮 6 个事件以 `M2_PREEMPTED` 关闭。M2 Team 代码路径未改。
- coordinator 的 K3 记录为 `TELEMETRY_ONLY`；planner 对 `repair_k=3` schedule 不接受 Team proposal。旧的 write-only Team hint 已移除，tail/probe 第二套 authority 的静态扫描没有命中。本轮有 26 条 `[RELAY_K3_EVENT]` telemetry。
- C3/D3/K2 比较要求 baseline 与候选覆盖同一完整时间窗。新增的最小 production 补齐是：K2 baseline 窗口无效时，K3 recovery 不能选择 PLUS/MINUS 侧候选；NOMINAL 仍遵循原 Local rolling 路径。本轮出现 1 个 `[K3_K2_GATE_UNAVAILABLE]` 周期。
- FOV/RANGE 在 Local event 和统一 binary/margin 中有评估；**N/L/R 执行 dispatch 仍要求 raw BODY/STATIC/DYNAMIC LOS 几何 witness**。本轮有 1 个 RANGE limiter event 没有 raw LOS witness，因而没有 side dispatch，之后按 margin 恢复关闭。该事件没有导致硬安全问题；若产品要求 FOV/RANGE-only 也必须产生 N/L/R，这仍是明确的设计边界。
- 第一个真实 recovery 链缺口来自 `checkK3EscalationRecovery()`：它只检查调用时最新的 fresh odometry，并要求 sample stamp 落在 event window。窗口内较早的可见 odometry 样本若在下一次滚动前被更晚的样本覆盖，会被窗口上界过滤。这个问题在 §5 的 event 4 中实际出现。

## 离线 replay、检查与构建

重放了 Feedback111/112 保存的 4 个 run 的 visibility、K3 event 与 executed window 数据；结果保存在 [离线 replay 汇总](artifacts/feedback113_offline_replay.json)。Feedback112 报告中的历史 margin replay 为 39/40 个 binary loss 可被提前检出、P50 lead 2.13–2.47 s；其逐事件中间数据未保存在工作区，本轮引用该报告，不声称从原始中间数据重新计算了 39/40。最终 run 另与其 visibility CSV 做逐样本对账。

目标 production 文件和离线分析脚本 `git diff --check` 通过。`catkin build -j2 --no-status`：**25/25 packages succeeded，package warnings 0**。catkin 打印了缓存与当前 `CMAKE_PREFIX_PATH` 不同的环境提示，但最终 build summary 无 warning、无失败。

## 唯一正式 FULL ON

命令：

```bash
./scripts/run_alp_full_on.sh --ablation full --headless --timeout 240 --boot-timeout 180 \
  --scenario /home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/scenes/natural_team_stress_dense_38_targeted_k2_v6.json \
  --k3-repair on --k3-escalation on
```

RUN_ID：`20260925_190928_365841`。BOOT-12 成功，`FINAL_EXIT_CODE=0`。ROS `ROS_LOG_DIR` 指向本 run 下的 `ros_log/`，挂载为 ext4 持久盘；`/dev/shm`/tmpfs 未作为日志目录。swap 使用量在运行前后约 1.2 GB，没有随着 run log 增长；日志不是写入 swap 的目标路径。运行后完整 stdout 已 gzip 压缩，CSV、manifest、状态文件、excerpt 和指标 JSON 保留。

runner 登记 29 个自有进程；`CLEANUP_STATUS=OK`、强杀 0、剩余自有进程 0、杀死外部进程 0。

## K3 结果

| 项目 | 结果 |
|---|---:|
| Local K3 events | **17**（UAV1/2/3：9 / 4 / 4） |
| 正 lead margin crossing | **8/17**；正 lead P50 0.10 s、max 0.20 s；4 个同刻，5 个没有预测 binary loss |
| N/L/R dispatch | **13**（STATIC 11、DYNAMIC 2） |
| progress selected / committed / activated | **13 / 13 / 12 个事件**；按 rolling 候选计 **24 / 24 / 23 次** |
| selected side | SIDE_PLUS 7，SIDE_MINUS 6，NOMINAL 11 |
| C3/D3 选择比较 | 24 次；C3 增加 10、持平 14；D3 改善 23、持平 1；所有被选比较的 K2=1.0 |
| binary recovery checker 观测 | **6 个事件** |
| recovery false positive | **0/6**：每个日志 recovery 都在 50 ms 内匹配到同 UAV executed-visible 样本 |
| recovery false negative | **至少 1**：event 4 的窗口内 executed K3 已恢复，Local 未记录 |
| SIDE_BOTH_FAILED | **49 个候选比较周期，分布在 9 个事件** |
| M2_PREEMPTED_K3 | **6 个事件** |
| K2 baseline window unavailable | 1 个周期；侧候选按 fail-closed 门禁处理 |

event result 关闭原因：M2_PREEMPTED 6、RECOVERY_OBSERVED 6、MARGIN_RECOVERED 4、EVENT_EXPIRED 1。failure stage 中有 3 个 `PROGRESS_BUT_NOT_RECOVERED` 和 2 个 `NO_SIDE_DISPATCH`；event 4 是最早的非 M2、未被 Local 正确闭合的事件。

## 执行可见性与安全

全程 2348 个 visibility samples，覆盖 78.233 s：

| 指标 | 结果 |
|---|---:|
| Executed ALL3 / K2 | **0.903748 / 0.997871**（K2 不是 100%） |
| K3 loss 总时长（相邻 sample 零阶积分） | **7.531 s** |
| 最长 ALL3 / K2 loss | **1.766540 / 0.166836 s** |
| UAV1 / UAV2 / UAV3 visibility ratio | **0.942504 / 0.985945 / 0.973169** |
| static / dynamic contact samples（clearance ≤ 0） | **0 / 0** |
| 最小 static / dynamic clearance | **0.245478 / 0.313696 m** |
| swarm violation / 最小 swarm separation | **0 / 1.437044 m** |
| PVA mismatch / unvalidated execution / partial Team activation | **0 / 0 / 0** |
| moving successor starvation / terminal hold / MIN_BUDGET_APPLIED | **0 / 0 / 0** |

## FIRST_CAUSAL_FAILURE

最早的非 M2 失败是 **event 4 / UAV1 / STATIC**。Local event 在 `1790334585.100` crossing，预测 binary loss 在 `1790334585.300`，lead 0.20 s。此前 SIDE 两侧共有 4 次都失败；随后选中 NOMINAL，比较值为 `K2=1.0`、`C3 1.000→1.000`、`D3 0.017016→0.017015`。trajectory 44 在 `1790334586.481693` 激活。

event window 为 `[1790334585.100, 1790334586.600)`。executed CSV 在窗口内有 45 个样本：UAV1 可见 8 个，不可见 37 个；K3 在 `1790334586.518644 / .551427 / .585134` 连续恢复 3 个样本，均早于窗口结束。最后一个 blocked 样本在 `1790334586.485937`。但 `[K3_EVENT_RESULT]` 在 `1790334586.915709` 记录 `binary_k3_recovered=0`、`close_reason=EVENT_EXPIRED`；trajectory 44 的 activation 确认行出现在窗口结束之后的 `1790334586.652182`。

**判断**：这是 `PROGRESS_BUT_NOT_RECOVERED` 的 false negative。source 只读最新 odometry；若 callback 下一次运行时最新 sample 已晚于窗口上界，checker 会拒绝它，而没有检查此前已经落在窗口内的恢复样本。精确的 checker odometry stamp 没有日志记录，所以“被窗口上界过滤”是由源码和时间戳共同支持的因果解释，尚无那一个 stamp 的直接遥测证明。此证据不足以把首要问题归为 deep-shadow side 几何；优先应修正/观测恢复 sample 的窗口消费语义，再验证。

## 收尾字段

```text
SOURCE_AUDIT_COMPLETE: YES
OFFLINE_REPLAY_COMPLETE: YES  # 历史逐事件 39/40 数字引用 Feedback112；原始 margin 中间数据未保留
PRODUCTION_CHAIN_CLEANED: PARTIAL  # 单一 Local authority 已建立；执行恢复 sample 被漏读
BUILD_PASS: YES  # 25/25，package warnings 0

LOCAL_AUTONOMOUS_K3: YES
TEAM_FAILURE_REQUIRED_FOR_K3: NO
UNIFIED_K3_VISIBILITY_AUTHORITY: YES
M2_TEAM_PATH_PRESERVED: YES
TEAM_K3_PROPOSALS_EXECUTABLE: 0  # coordinator emitted telemetry only; Local rejects repair_k=3

FULL_ON_RUN_COMPLETED: YES
FULL_RUN_COUNT_THIS_TASK: 1
RUN_ID: 20260925_190928_365841

LOCAL_K3_EVENTS: 17
PRE_BINARY_TRIGGER_EVENTS: 8/17 positive lead; P50 0.10 s, max 0.20 s
K3_PROGRESS_SELECTED: 13 events / 24 candidate selections
K3_PROGRESS_COMMITTED: 13 events / 24 candidate commits
K3_PROGRESS_ACTIVATED: 12 events / 23 trajectory activations
K3_RECOVERED_EVENTS: 6 logged; +1 executed-truth recovery missed at event 4
K3_SIDE_BOTH_FAILED: 49 cycles across 9 events
M2_PREEMPTED_K3: 6

FALSE_K3_RECOVERY_PREDICTION_COUNT: 0 false positives / 6 observed recoveries
MISSED_EXECUTED_K3_RECOVERY_COUNT: at least 1

EXECUTED_ALL3: 0.903748
EXECUTED_K2: 0.997871
K3_LOSS_TOTAL_TIME: 7.531 s
LONGEST_ALL3_LOSS: 1.766540 s
LONGEST_K2_LOSS: 0.166836 s

STATIC_CONTACT: 0
DYNAMIC_CONTACT: 0
SWARM_VIOLATION: 0
PVA_MISMATCH: 0
UNVALIDATED_EXECUTED: 0
PARTIAL_TEAM_ACTIVATION: 0
STARVATION: 0
TERMINAL_HOLD: 0

K3_LOCAL_CHAIN_CLOSED: NO  # event 4 executed recovery was not consumed before event expiry
SAFETY_REGRESSION: NO  # hard safety counters remained zero
FIRST_CAUSAL_FAILURE: PROGRESS_BUT_NOT_RECOVERED classification; actual issue is missed in-window executed recovery sample
SIDE_GEOMETRY_NEXT_STEP_JUSTIFIED: NO  # recovery observation must be fixed/verified before geometry attribution

REPEATED_FULL_SIMULATION_USED: NO
RRCT_ACCESSED: NO  # no RRCT files read or written
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO  # this ALP run: own-process cleanup 29/29
```

## 证据

- [最终 run 指标 JSON](artifacts/feedback113_final_metrics.json)
- [K3 事件筛选 excerpt](../runs/20260925_190928_365841/feedback113_k3_excerpt.log)
- [run manifest](../runs/20260925_190928_365841/run_manifest.txt)、[resolved parameters](../runs/20260925_190928_365841/resolved_params.txt)、[exit status](../runs/20260925_190928_365841/exit_status.txt)、[process cleanup](../runs/20260925_190928_365841/process_status.txt)
- [压缩 roslaunch stdout](../runs/20260925_190928_365841/roslaunch_stdout.log.gz)、[压缩 runner console](../runs/20260925_190928_365841/runner_console.log.gz)、[visibility CSV](../runs/20260925_190928_365841/visibility.csv)、[executed trajectory CSV](../runs/20260925_190928_365841/visibility_trajectory.csv)、[full metrics](../runs/20260925_190928_365841/native_metrics_full.json)
- [历史 offline replay](artifacts/feedback113_offline_replay.json)、[Feedback112 authority/replay report](feedback_112.md)
