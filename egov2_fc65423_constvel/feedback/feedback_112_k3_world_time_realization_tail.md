# Feedback 112 — K3 世界时间、realization 失败升级与恢复尾段

日期：2026-09-25。工作区仅为 `/home/bob/ALP/egov2_fc65423_constvel`。当前 dirty worktree 是源码权威。场景固定为 `natural_team_stress_dense_38_targeted_k2_v6.json`，FULL、headless、timeout 240 s、boot timeout 180 s。用户最后明确要求不再运行 OFF；本文使用此前已完成的两组 OFF/ON，不新增基线仿真。

## 1. Feedback111 遗留问题与当前调用链审计

Feedback111 发现三处缺口：候选目标位置相对 planning epoch 传播，与候选真正执行起点不一致；K3 Team proposal 在 Local realization / hard preflight 失败后没有 Local escalation；有 D3 改善的深阴影事件会在原始二值 loss window 后失去授权。旧日志还记录了候选判断 C3=1、实际静态遮挡约 1 s 的案例；这是待复核的预测错误，不能仅凭本轮零误差诊断宣称已消失。最终 ON 的 event 2 仍有一个较短的可比误判，见 §3。

当前实际链路为：coordinator 的 `assessK3Relay` → `updateK3Contract` → `TeamReferenceSchedule` → Team SCP / Local realization → `abortTeamProposal` → `TopologyEscalation` → planner 的 `topologyEscalationCallback` → raw static/dynamic LOS witness 授权 N/L/R → `finalizeCapturedCandidates` 的 hard preflight、K2 门、C3/D3 比较、原 `betterTeamVisibilityWithinNearBestK2` → first-safe commit → 轨迹 start time 到达后确认 activation → `checkK3EscalationRecovery`。代码入口分别在 [coordinator](../ros_ws/src/multi_uav_formation/src/multi_uav_topology_coordinator.cpp)、[planner manager](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp)、[Local 原比较器](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/include/plan_manage/local_visibility_preference.h)、[几何权威](../ros_ws/src/multi_uav_formation/include/multi_uav_formation/tracking_visibility_geometry.h) 和 [TopologyEscalation.msg](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_utils/msg/TopologyEscalation.msg)。runner 的 `--k3-escalation` 仅控制 Local 消费，coordinator 在 OFF 中仍可发布 K3 escalation；因此 OFF 的 source 计数不等于 Local event 数。

时间口径核对如下。候选局部时刻 `t_local` 的世界时刻为 `t_world = candidate_start_world + t_local`；target 从冻结的**原始几何快照** `target_position + target_velocity × max(0, t_world - target_prediction_epoch)` 传播。候选本机轨迹取 `t_local`；peer 取 `executionAt(t_world)` 并以 `t_world - peer.start_time` 采样，缺轨迹时 odom fallback 也传播到 `t_world`；dynamic obstacle 的 `evaluateConstVel` 直接使用 `t_world`。`fillCandidateBinaryVisibility` 同样从 trajectory start + sample offset 计算 target / dynamic。原先 `evaluateCandidateVisibility` 的隐式相对时间和 smoothed `object_p_` 与二值权威使用的 raw target 几何快照均已对齐；FOV/yaw 仍仅在 binary/相机检查中，按本轮边界未重写。

`evaluateCandidateVisibility`、`fillCandidateBinaryVisibility`、`evaluateVisibilityMarginAt` / `visibilitySampleAt` / `composeVisibilityMargin` 的调用位置已核实。C3/D3 仅在 Local candidate finalize 后计算；本轮没有把完整 visibility evaluator 加进 L-BFGS inner loop，没有改原比较器对普通候选的路径。[TeamReferenceSchedule.msg](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_utils/msg/TeamReferenceSchedule.msg) 的 K3 evidence 继续沿用现有字段；runner/launch 参数链保持 `--k3-escalation on|off` → `manager/k3_local_escalation_enabled`。

## 2. Production 修改

1. `CandidateSetOutput` 保留与该 candidate batch 同源的 raw `visibility_target_position/velocity/epoch`。`evaluateCandidateVisibility` 接受明确 target epoch；baseline、N/L/R candidate、已提交轨迹重新评价均按上述世界时刻采样。[`K3_TIME_ALIGNMENT`] 对同一 `t_world` 输出 candidate 和 authority 目标位置及误差。初始诊断配对 ON 曾有 P95 **0.235685 m**、最大 **0.322545 m**；改后两组 ON 为 **0 / 0 m**（最终组 66 个诊断样本）。这个对齐度证明两条预测路径共享快照，不直接证明实际执行相机真值无误。
2. `abortTeamProposal` 只对 `TEAM_SCP_FAILED`、`LOCAL_HARD_PREFLIGHT_FAILED` 和源码中明确可继续 Local 搜索的 realization 失败分类考虑 K3 escalation。发布前复核 `repair_k=3`、contract/target/window/required margin/detect evidence/revision、task lineage、bundle freshness 和当前 M2 预测；expired、identity/stale、M2 preemption 不沿用旧 hint。只发布 N/L/R 授权，失败的 Team trajectory 不被采用。`[K3_ESCALATION_SOURCE]` 记录 source、abort reason、发布与拒绝原因。首组 OFF 有 1 次 `M2_PREEMPTION` 拒绝，属于正确排除，不计为合法 K3 漏发。
3. 复用既有 K3 event 与 8 s hint freshness：primary window 到期且同 target/revision/raw blocker、K2 健康、已提交轨迹实际到达 activation 且 `C3_after>C3_before` 或 `D3_after<D3_before`，才继续滚动授权；无 progress、blocker 丢失、M2 抢占、stale、无安全候选即关闭。没有固定加长原 forecast window、没有新 FSM 或 margin ladder。运行结果见下文：代码路径存在，本轮最终 ON **没有任何已确认激活的 progress**，所以 tail enter 与 tail 恢复均为 0，效果尚未被验证。

最后一次 production 修改后 `git diff --check` 通过，`catkin build -j2 --no-status` **25/25** 成功，包含 `traj_utils`、`traj_opt`、`ego_planner`、`multi_uav_formation`。本轮未改 ROS msg、SCP trust、required margin、安全阈值、controller、M2 主路径、SIDE 几何或 C3/D3 排序。

## 3. FULL 运行与 executed truth

所有四个正式 run 都达到 BOOT-12、`FINAL_EXIT_CODE=0`，runner 登记的 29 个自有进程均退出，强杀 0、外部进程 0。A 组在 target 对齐修复后、同 K3 contract continuation gate 最后修复前完成；B 组在最终代码下完成。初始 target 错位诊断 pair `144345_142844 / 144845_145113` 不作为正式性能结论。

| 指标 | A OFF `145659_150538` | A ON `150201_152994` | B OFF `151131_158119` | B ON `151633_160309` |
|---|---:|---:|---:|---:|
| ALL3 / K3 | 0.902429 | 0.903322 | 0.881551 | **0.916560** |
| K2 | 0.990200 | 0.993186 | **0.994461** | 0.990209 |
| mean visible | 2.892629 | 2.896508 | 2.876012 | 2.906769 |
| accumulated camera visible time, UAV·s | 226.206 | 226.600 | 224.890 | 227.516 |
| K3 loss total, s（采样间隔积分） | 7.629 | 7.569 | 9.262 | 6.527 |
| longest ALL3 loss, s（canonical） | 1.627 | 1.500 | 1.967 | 1.399 |
| longest K2 loss, s（canonical） | 0.766 | 0.533 | 0.428 | **0.767** |
| UAV1 / UAV2 / UAV3 visible ratio | .9374 / .9859 / .9693 | .9395 / .9889 / .9681 | .9416 / .9851 / .9493 | .9472 / .9872 / .9723 |
| blackout samples | 0 | 0 | 0 | 0 |
| K3 contracts / consumed Local events | 25 / 0 | 30 / 20 | 35 / 0 | 23 / 17 |
| K3 source: SCP / realization, published | 5 / 8 | 13 / 7 | 14 / 17 | 10 / 7 |
| candidate better / telemetry selected / confirmed activated | 0 / 0 / 0 | 25 / 1 / 1 | 0 / 0 / 0 | 32 / 2 / **0** |
| K3 event `RECOVERY_OBSERVED` | 0 | 1 | 0 | 4 |
| K3 event SIDE both failed | 0 | 56 | 0 | 33 |

B 组 ON 的 4 次 `RECOVERY_OBSERVED` 没有已激活 K3 progress，**不能归因于 Local escalation**。更严格地对齐 executed 窗口：event 2、6、12 的 target 在各自整个 0.1 s 窗口中均有 **3/3 不可见样本**，仅 event 13 在其窗口内实际可见。这里的 `RECOVERY_OBSERVED` 是 planner 用轨迹预测所得，不是 executed-truth 计数。B 组 `progress_selected=2` 是旧遥测命名：event 3 的 SIDE_PLUS 确实改善 C3 0→0.6、D3 2.414→0.326，但新事件在 activation 前取代它；event 10 的 SIDE_PLUS 仅在可用候选之间 D3 较低，**相对已执行 baseline 反而 3.339→3.490**，不满足 recovery tail progress gate，也没有确认 activation。按严格意义，最终 ON 是 **1 次改善候选提交、0 次已激活改善**。

Feedback111 记录过候选 C3=1 而 executed 仍长时遮挡的两次诊断。本轮**B ON event 2 是 1 次残留的窗口级误判**：NOMINAL candidate 50 预测 C3=1，traj 33 在世界时刻 `1790320608.591957` 激活，contract 3 的窗口为 `1790320608.600–0608.700`。实际 UAV1 在 `0608.614/0608.643/0608.675` 三个连续样本 `static_los_clear=0`、`visible=0`，到约 `0608.707` 才恢复；轨迹记录确认为 traj 33。该候选的 C3 样本落在窗口后缘 `0608.691`，说明即便 target snapshot 内部误差为 0，短窗口的采样相位、预测目标相对 executed 真值和静态 LOS 边界仍足以漏掉前半段遮挡。A ON 的近似 C3=1 事件中，event 8 候选在窗口前已被新轨迹取代，其余可比窗口实际可见；故 A=0、B=1。B event 2 的 `RECOVERY_OBSERVED` 是 planner 预测事件闭合，**不能算作窗口内 executed K3 恢复**。本轮这次误判约 0.07 s，短于 Feedback111 约 1 s 的案例；事件条件不同，不能据此归因“修复让误判缩短”。相关 candidate/commit/activation 行在完整压缩日志，实际样本在 `visibility.csv` 与 `visibility_trajectory.csv`。

### 最终 ON 的 17 个 K3 event

表中时刻为 target 开始后的相对秒；`side fail` 是 event 绑定的双侧失败次数。`selected/activated` 只计 K3 progress telemetry，普通 NOMINAL 仍可提交、激活（如 event 2 的 traj 33）；仅 event 3 的 progress selected 是对 baseline 的真实改善。source 栏 `P` = Local hard preflight failure，`S` = Team SCP failure；所有 17 条均 `escalation_published=1`。`recovery=1` 表示 planner event 预测到二值恢复，不表示 executed 真值或该 event 的 Local commit 导致恢复。没有 K3 progress 提交的 event 在 `SIDE、C3/D3` 栏为 `—`，每 cycle 的 NOMINAL/PLUS/MINUS 与 baseline 数值在 K3 excerpt 保留。

| event / contract | target / window s | detect margin / source | side fail | SIDE、C3/D3（若提交） | selected / activated | recovery / close |
|---|---|---|---:|---|---:|---|
| 1 / 2 | 0 / 8.51–8.71 | −3.391 / P | 1 | — | 0 / 0 | 0 / superseded |
| 2 / 3 | 0 / 8.71–8.81 | −1.228 / S | 2 | NOMINAL C3=1；traj 33 激活，executed 3 样本仍静态遮挡 | 0 / 0 | 1 / 预测闭合，窗口误判 |
| 3 / 4 | 0 / 10.71–11.21 | −5.962 / P | 1 | PLUS: C3 0→.6, D3 2.414→.326；commit traj 36 | 1 / 0 | 0 / superseded before activation |
| 4 / 5 | 0 / 11.11–11.41 | −4.358 / S | 1 | — | 0 / 0 | 0 / contract replaced |
| 5 / 9 | 0 / 11.51–11.91 | −7.135 / S | 1 | — | 0 / 0 | 0 / superseded |
| 6 / 10 | 0 / 11.81–11.91 | −2.464 / S | 2 | — | 0 / 0 | 1 / 预测闭合，executed 窗口 3/3 不可见 |
| 7 / 11 | 0 / 25.01–25.11 | −1.103 / S | 4 | — | 0 / 0 | 0 / contract replaced |
| 8 / 16 | 0 / 64.01–64.21 | −3.438 / S | 2 | — | 0 / 0 | 0 / superseded |
| 9 / 17 | 0 / 64.21–64.71 | −6.178 / S | 2 | — | 0 / 0 | 0 / superseded |
| 10 / 18 | 0 / 64.21–65.01 | −6.131 / S | 1 | PLUS: C3 0→0, D3 3.339→3.490；commit traj 231 | 1 / 0 | 0 / superseded before activation |
| 11 / 19 | 0 / 64.51–65.01 | −6.455 / S | 2 | — | 0 / 0 | 0 / superseded |
| 12 / 20 | 0 / 65.01–65.11 | −1.510 / S | 2 | — | 0 / 0 | 1 / 预测闭合，executed 窗口 3/3 不可见 |
| 13 / 21 | 0 / 67.11–67.81 | −3.298 / P | 1 | — | 0 / 0 | 1 / observed |
| 14 / 22 | 2 / 70.61–71.51 | −4.340 / P | 6 | — | 0 / 0 | 0 / contract replaced |
| 15 / 24 | 2 / 71.31–71.91 | −5.722 / P | 1 | — | 0 / 0 | 0 / superseded |
| 16 / 25 | 2 / 71.51–72.11 | −6.350 / P | 3 | — | 0 / 0 | 0 / superseded |
| 17 / 27 | 2 / 73.21–73.31 | −1.045 / P | 1 | — | 0 / 0 | 0 / contract replaced |

逐事件 detect、source、SIDE 候选和结果的原始行保存在 [最终 ON K3 excerpt](../runs/20260925_151633_160309/feedback112_k3_excerpt.log)，所有日志保存在同目录 `roslaunch_stdout.log.gz`；[最终 pair 机器可读汇总](artifacts/feedback112_pair2_metrics.json) 含 17 个 event 的原字段。[分析脚本](artifacts/analyze_feedback112.py) 可复算。`K3_RECOVERY_TAIL` 最终 ON 有 4 行 `CONTRACT_REPLACED_OR_M2`，均发生在 primary window 到期前、尚无 activated progress；`ENTER_PROGRESS_GATE=0`、tail recovery=0、no-progress tail exit=0。A 组 event 10 曾确认 traj 225 激活并使 D3 2.058→1.730，但它的旧合同检查过早关闭 event；这正是 B 组前修复 continuation gate 的原因，B 组没有重现相同激活机会，故 tail 仍未获得实测验证。

## 4. 区间归因、安全与资源

B OFF 的 ALL3 loss 区间约为 8.07–8.80、10.77–11.87、25.30–26.50、**35.04–36.10**、**56.33–56.37/56.90–57.20**、64.07–65.07、70.33–72.27、73.40–75.00 s。B ON 为 8.01–8.78、10.91–11.95、25.29–26.45、64.08–65.12、70.69–72.05、73.75–74.71 s。35/56 s 的 OFF-only 损失约占 B 组 K3 loss 差异的一半，附近没有 ON K3 event；晚段有 M2/K2 风险。两组 ON 的 ALL3 都高于各自 OFF（+0.000893、+0.035009），但 B 组 **没有已激活 K3 改善**，因此这不是 K3 Local 机制导致全局收益的因果证明。B 组 K2 从 .994461 降到 .990209，最长 K2 loss 从 .428 s 增到 .767 s；候选阶段的 K2 gate 仍在，**executed 全局 K2 未保持**。晚段 M2 风险与 K2 loss 同期，不能在无新实验下归咎 K3。

| 安全 / liveness | A OFF / ON | B OFF / ON |
|---|---:|---:|
| static contact samples | 0 / 0 | 0 / 0 |
| dynamic zero-clearance samples (`<=0`) | **5 / 0** | 0 / 0 |
| min static clearance, m | .208499 / .218588 | .219812 / .227203 |
| min dynamic clearance, m | **0 / .310175** | .298178 / .312568 |
| min swarm separation, m | 1.225323 / 1.349322 | 1.085306 / 1.330981 |
| swarm violation / unvalidated executed | 0 / 0 | 0 / 0 |
| PVA mismatch / partial Team activation | 0 / 0 | 0 / 0 |
| moving successor starvation | 2 / 1 | 0 / 0 |
| terminal hold enter / END_BEFORE_NEXT | 0 / 0 | 0 / 0 |
| MIN_BUDGET_APPLIED | 100 / 0 | 16 / 1 |

“dynamic zero-clearance”严格按 CSV `moving_clearance_m <= 0` 计数，不单凭数值断言物理碰撞。所有 ON 的静/动净空正、swarm violation 0、执行样本全为 `safety_validated=true`，无 ON hard safety 回退。B 组 K2 指标退步列为性能风险，不混同 hard safety。

资源清理：本轮 6 个 run 的重复 `runner_console.log`、临时 `ros_log/ros_home` 已删除，完整 roslaunch log 压缩；保留 CSV、exit/process status、K3 excerpt 和 compact JSON 以复核。最终 ON 从约 149 MB 压到约 17 MB。随后对**已正常退出且自有进程清零**的历史 run 做同样的保留证据压缩，没有删除任何 run 目录、CSV 或 manifest；总计压缩 35 个目录，回收 **6.291 GB**，`runs/` 从约 6.9 GB 降到 1.1 GB，磁盘可用空间约 14 GB。[逐 run 清理清单](artifacts/feedback112_cleanup_manifest.json) 记录前后字节和删除的冗余路径。build/devel 保留为当前可运行产物。结束时没有本轮仿真进程。

## 5. 结论与下一步边界

**FIRST_CAUSAL_FAILURE（对 tail 生效而言）**：最终 ON 的 progress 候选没有走到“改善提交并确认激活”。event 3 在 activation 前被新合同取代；其余多数临界窗 side 双失败或 comparator 未选，event 10 虽提交但相对 baseline D3 变差且未激活。代码中的 tail gate 因而正确保持关闭，无法宣称 tail 有实际恢复样本。先核对 K3 合同快速更新、commit→activation 保留窗口及真实 SIDE 可达性；当前证据不支持直接扩大 trust、改 SIDE offset 或降低安全门槛。**对预测保真度而言**，event 2 已证明 C3 在很短的窗口里发生采样/实际静态 LOS 分歧，event 2/6/12 的预测闭合又早于 executed 恢复；target 快照内部零误差不是 executed 真值保证。应优先审查候选样本覆盖 primary window 边界及 `checkK3EscalationRecovery` 的预测闭合与实际可见性的口径。全局 ALL3 增益的首要归因限制是 35/56 s 非 K3 窗口的运行分岔。

PRODUCTION_CODE_CHANGED: YES  
BUILD_PASS: YES（25/25，`git diff --check` 通过）  
WORLD_TIME_ALIGNMENT_FIXED: YES（预测路径同源，executed 可见性仍需足量可比样本）  
TARGET_ALIGNMENT_ERROR_P95: 0 m（B ON，n=66）  
TARGET_ALIGNMENT_ERROR_MAX: 0 m（B ON，n=66）  
FALSE_K3_RECOVERY_PREDICTION_COUNT: B ON **1 次 selected C3=1 的窗口级误判**（event 2，已激活 traj 33）；另有 **3 次 planner event 预测闭合与 executed 整窗不可见冲突**（event 2/6/12，含前述 1 次）；Feedback111 曾报告 2 次约 1 s 的 C3 误判  
REALIZATION_FAILURE_ESCALATION_IMPLEMENTED: YES  
K3_ESCALATION_FROM_REALIZATION_FAIL: A ON 7；B ON 7（全部 hard preflight）  
K3_ESCALATION_FROM_SCP_FAIL: A ON 13；B ON 10  
K3_REALIZATION_FAIL_WITHOUT_ESCALATION: A ON 0；B ON 0（合法且当前的 K3 contract）；A OFF 有 1 次 M2_PREEMPTION 正确拒绝  
RECOVERY_TAIL_IMPLEMENTED: YES  
K3_PRIMARY_WINDOW_EXPIRED_WITH_PROGRESS: B ON 0  
K3_RECOVERY_TAIL_ENTER: B ON 0  
K3_RECOVERY_TAIL_RECOVERED: B ON 0  
K3_RECOVERY_TAIL_NO_PROGRESS_EXIT: B ON 0  
K3_RECOVERY_TAIL_STALE_EXIT: B ON 4（窗口到期前合同替换 / M2 判定）  
RECOVERY_TAIL_EFFECTIVE: NO（未触发，缺实测证据）  
FULL_OFF_RUN_COMPLETED: YES（此前完成两次；收到最新指令后未再运行 OFF）  
FULL_ON_RUN_COMPLETED: YES（两次）  
OFF_RUN_ID: 20260925_145659_150538 / 20260925_151131_158119  
ON_RUN_ID: 20260925_150201_152994 / 20260925_151633_160309  
K3_CONTRACTS_OFF/ON: A 25/30；B 35/23  
K3_ESCALATION_EVENTS_OFF/ON: A 0/20；B 0/17（仅 Local consumed）  
K3_PROGRESS_SELECTED_OFF/ON: A 0/1；B 0/2 telemetry（B 严格改善 1）  
K3_PROGRESS_COMMITTED_OFF/ON: A 0/1；B 0/2 telemetry（B 严格改善 1）  
K3_PROGRESS_ACTIVATED_OFF/ON: A 0/1；B 0/0  
K3_RECOVERED_EVENTS_OFF/ON: A 0/1；B 0/4（B 为预测事件闭合，其中 3 个窗口实际整窗不可见，且与 Local 激活无因果关系）  
PROGRESS_BUT_NOT_RECOVERED_OFF/ON: A 0/1；B 0/0（只计已激活且严格改善）  
EXECUTED_ALL3_OFF: A .902429；B .881551  
EXECUTED_ALL3_ON: A .903322；B .916560  
EXECUTED_K2_OFF: A .990200；B .994461  
EXECUTED_K2_ON: A .993186；B .990209  
K3_LOSS_TOTAL_TIME_OFF/ON: A 7.629/7.569 s；B 9.262/6.527 s  
LONGEST_ALL3_LOSS_OFF/ON: A 1.627/1.500 s；B 1.967/1.399 s  
LONGEST_K2_LOSS_OFF/ON: A .766/.533 s；B .428/.767 s  
K3_SIDE_BOTH_FAILED_ON: A 56；B 33（event 绑定）  
STATIC_CONTACT_SAMPLES_OFF/ON: A 0/0；B 0/0  
DYNAMIC_CONTACT_SAMPLES_OFF/ON: A 5/0；B 0/0（`<=0` CSV 口径）  
SWARM_VIOLATION_OFF/ON: A 0/0；B 0/0  
PVA_MISMATCH_OFF/ON: A 0/0；B 0/0  
UNVALIDATED_EXECUTED_OFF/ON: A 0/0；B 0/0  
PARTIAL_TEAM_ACTIVATION_OFF/ON: A 0/0；B 0/0  
STARVATION_OFF/ON: A 2/1；B 0/0  
TERMINAL_HOLD_OFF/ON: A 0/0；B 0/0  
END_BEFORE_NEXT_OFF/ON: A 0/0；B 0/0  
EVENT_LEVEL_K3_GAIN_CONFIRMED: A ON 有 1 次激活 D3 改善；最终 B ON **NO**  
GLOBAL_EXECUTED_K3_GAIN_CONFIRMED: 观测 YES（两组 ON 均高于 OFF）；K3 机制因果 NO  
K2_PROTECTED: candidate gate YES；B 组 executed 全局 NO  
SAFETY_REGRESSION: NO（ON hard safety）  
LIVENESS_REGRESSION: NO（本轮记录）  
FIRST_CAUSAL_FAILURE: 合同取代发生在改善候选 activation 前 + 多数 SIDE 不可行；tail 无合格入口  
SIDE_GEOMETRY_NEXT_STEP_JUSTIFIED: NO（需先分离事件 churn / activation 问题）  
ADAPTIVE_TRUST_JUSTIFIED: NO  
RRCT_ACCESSED: NO  
RRCT_CHANGED: NO  
SIMULATION_LEFT_RUNNING: NO
