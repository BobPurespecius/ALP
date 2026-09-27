# Feedback 123 — Local 三接口修改与一次 FULL 尝试

日期：2026-09-26。工作区：`/home/bob/ALP/egov2_fc65423_constvel`。按最新指令，本轮修改 production、构建并启动仿真；没有编写或运行离线测试，也没有跑 OFF。仿真后没有修改 production 或再次启动仿真。

## 源码改动与构建

- 在 [planner_manager.cpp](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp) 接入保守的 A* voxel 边检查、相邻 Local-SFC corridor 连接检查、共用的 fixed-PVA MINCO timing initializer、候选多项式 hash 与当前 revision 的 `checked_until`。提交前计算证书，失败不改变 Local 提交状态。Team speculative path 仍通过原 PREPARE/COMMIT，旧 external polynomial proposal 入口被拒绝。
- 在 [poly_traj_optimizer.cpp](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/src/poly_traj_optimizer.cpp) 移除 seed-preservation tube 的 hard row/final gate，允许同一 SCP 中 infeasible seed 的受控 residual 下降，最终仍要求物理检查通过；无 trust probe 的状态改为仅在全部明确 primal infeasible 时报告 QP 不可行。
- [PolyTraj.msg](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_utils/msg/PolyTraj.msg) 与 [MINCOTraj.msg](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_utils/msg/MINCOTraj.msg) 的多项式数值改为 float64。PolyTraj 携带 payload hash、validation revision、checked_until；MINCO 机间广播携带 checked_until。发布、接收、激活三阶段均有 identity 日志；executor 在证书终点停止有效轨迹。
- [traj_server.cpp](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/traj_server.cpp) 仅按 Team 已验证的 evaluation horizon 给 speculative 轨迹认证，不把整段多项式时长当证书。Local/peer 数据结构和发布路径同步传递认证末端。

`git diff --check` 通过。`catkin build -j2 --no-status` 最终 **25/25 packages succeeded，package warnings 0**。没有执行测试。由于最新指令明确跳过测试，A–F 离线合同未验证，不能宣称 `OFFLINE_CONTRACT_CHECKS_PASS=YES`。

## 唯一一次 FULL ON 尝试

命令为指定的 `--ablation full --headless --timeout 240 --boot-timeout 180 --k3-repair on --k3-escalation on`，场景为 `natural_team_stress_dense_38_targeted_k2_v6.json`。RUN_ID：`20260926_135443_696819`。runner 仅确认到 **BOOT-07**，没有到 BOOT-12。三机 trajectory 4/5 的认证末端均为 `1790402091.744443`；traj_server 在 `1790402091.748–.752` 三次记录 `TERMINAL_HOLD_ENTER`。由于 BOOT-08 的循环仍在重试并持续写大量日志，在证据已足以判定失败时主动结束该 run，避免磁盘继续增长。因此没有 wrapper `FINAL_EXIT_CODE`、`exit_status.txt` 或正式 `process_status.txt`；**这是失败的 FULL 尝试，不是有效的 production run**。

手动关闭了此 run 的 roslaunch；按 run ID 与 launch 命令复查，相关进程为 0，没有触及其他进程。两份 stdout 已 gzip，ROS 日志经 tar 校验后仅保留归档，run 目录从约 1.8 GB 缩到 **263 MB**，磁盘可用空间约 **5.7 GB**。

## FIRST CAUSAL FAILURE

**新接入的 peer `checked_until` 递归取最小值使三机证书无法续期。** [hardCheckedUntil](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp) 对每个 peer 的已广播证书末端取 `min`。初始本机动态/static authority 约两秒是有限的；其后所有 peer 都以彼此上一轮的有限证书为上界。一次新的本机动态预测即使前移，也不能把 peer 的旧末端推后。运行中三个新的 hard-safe 候选在 `1790402091.548–.552` 已分别有 `active_end=1790402097.610/2095.294/2094.519`，但 `FINAL_CHECKED_UNTIL` 全是 `1790402091.744443`。这三个候选通过了 candidate metadata preflight，且发布/接收日志 revision 与 wire hash 对应；executor 在同一证书终点停止执行。其后 `CANDIDATE_CERTIFICATE status=UNKNOWN reason=MISSING_MAP_PREDICTION_OR_PEER`、反复 current-state restart 和 `DYNAMICS_FAIL` 才大量出现。后两者是覆盖失效后的下游现象，不能代替首因。

本轮没有在 FULL 后修改 production。下一次源码修改应打破这种循环证书依赖，同时保留缺失 peer/预测时 UNKNOWN fail-closed 和真实动态 horizon；不能把 peer 多项式的末端直接冒充已验证末端。需要可续期的、非循环的 joint/peer coverage authority 后，才有意义重新检查 SFC、timing、K3 与完整场景指标。

## 执行诊断，不作有效 FULL 验收

已保存的 `visibility.csv` 有 2349 个样本，覆盖 78.266 秒；`visibility_trajectory.csv` 有 47,637 条。三机大部分时间处于 terminal hold，因此以下比率描述失败状态，不能同 Feedback113/114 的有效 run 比较：ALL3 **0.040017**，K2 **0.059174**；ALL3 loss **75.133 s**，K2 loss **73.625 s**，blackout **71.399 s**。47,542 条执行记录为 `TERMINAL_HOLD` 且 `safety_validated=False`。静态/动态接触样本均为 0，最小净空分别 **2.284/7.291 m**；同 timestamp 的最小机间距 **1.062 m**，超过配置的 0.5 m 物理间距。日志中 PVA mismatch 与 partial Team activation 均为 0。K3 event result 3 条、recovery evidence 3 条，都发生在已失去正常执行覆盖的时期，不能当作 K3 recovery 性能证明。

日志还记录 1 次 `MOVING_SUCCESSOR_STARVATION`、3 次 `TERMINAL_HOLD_ENTER`、10,516 条 `MIN_BUDGET_APPLIED`。最后的 corridor audit 见 `ASTAR_STATIC_SAFE=6, SFC_BUILD_SUCCESS=0, SFC_BUILD_FAILED=6`，另一节点见 `3/0/3`；这说明 SFC 仍未由有效 run 验收，且不改变上述更早的证书死锁首因。

## 状态

```text
PRODUCTION_CODE_CHANGED: YES
BUILD_PASS: YES (25/25, package warnings 0)
TESTS_RUN: NO (按最新用户指令)
OFFLINE_CONTRACT_CHECKS_PASS: NOT_VERIFIED
A_CONTINUOUS_CHECKER_PASS: NOT_VERIFIED_THIS_TURN
B_SFC_PASS: NOT_VERIFIED
C_TIMING_JACOBIAN_PASS: NOT_VERIFIED
D_AUTHORITY_PASS: NOT_VERIFIED
E_EXECUTION_IDENTITY_PASS: NOT_VERIFIED_END_TO_END
F_CHECKED_UNTIL_PASS: NO (peer certificate cannot renew)
ROUTE_SFC_PIECE_CONTRACT: NOT_VERIFIED
PVA_AWARE_TIMING_ACTIVE: YES_IN_SOURCE
TIME_JACOBIANS_CORRECT: NOT_VERIFIED
SEED_TUBE_HARD_AUTHORITY_REMOVED: YES_IN_SOURCE
LOS_HARD_AUTHORITY_PRESENT: NOT_FULLY_AUDITED
TOPOLOGY_HARD_AUTHORITY_PRESENT: NOT_FULLY_AUDITED
QUALITY_CAN_KILL_SAFE_INCUMBENT: NOT_VERIFIED
FINAL_PREFLIGHT_AUTHORITATIVE: PARTIAL
COMMIT_PAYLOAD_EQUALS_VALIDATED_PAYLOAD: NOT_PROVEN_END_TO_END
CHECKED_UNTIL_TRUTHFUL: FAIL_FOR_PROGRESS (bounded but recursively nonrenewable)
LEGACY_FALLBACKS_REMOVED_OR_MERGED: PARTIAL
LEGACY_TEAM_EXTERNAL_POLYNOMIAL_AUTHORITY: DISABLED_IN_CALLBACK
FULL_ATTEMPT_COUNT_THIS_TASK: 1
FULL_RUN_COUNT_THIS_TASK: 0 EFFECTIVE (BOOT-07 only)
REPEATED_FULL_SIMULATION_USED: NO
EXECUTED_ALL3: 0.040017 (failed-attempt diagnostic)
EXECUTED_K2: 0.059174 (failed-attempt diagnostic)
ALL3_LOSS_TOTAL: 75.133 s
K2_LOSS_TOTAL: 73.625 s
BLACKOUT: 71.399 s
K3_EVENTS: 3 logged after coverage failure
K3_RECOVERED: 3 observed after coverage failure
STATIC_CONTACT: 0
DYNAMIC_CONTACT: 0
SWARM_VIOLATION: 0 confirmed against 0.5 m clearance
PVA_MISMATCH: 0 logged
UNVALIDATED_EXECUTED: 47542 TERMINAL_HOLD samples
PARTIAL_TEAM_ACTIVATION: 0 logged
STARVATION: 1 logged
TERMINAL_HOLD: 3 enters, 0 exits
ARCHITECTURE_HEALTH: FAIL
FIRST_REMAINING_CAUSAL_FAILURE: circular peer checked_until cap prevents all three Local certificates from renewing
NEXT_STEP: source repair of peer coverage renewal before another authorized FULL
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```

证据：[run manifest](../runs/20260926_135443_696819/run_manifest.txt)、[压缩完整 stdout](../runs/20260926_135443_696819/roslaunch_stdout.log.gz)、[压缩 runner console](../runs/20260926_135443_696819/runner_console.log.gz)、[压缩 ROS 日志](../runs/20260926_135443_696819/ros_log.tar.gz)、[visibility CSV](../runs/20260926_135443_696819/visibility.csv)、[执行轨迹 CSV](../runs/20260926_135443_696819/visibility_trajectory.csv)。
