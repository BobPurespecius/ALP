# Feedback 137 — SIDE seed 派生 topology region 与启动活性结果

日期：2026-09-27。唯一工作区：`/home/bob/ALP/egov2_fc65423_constvel`。本轮保留 SIDE fresh/warm/backoff/A*/Local-SFC/PVA timing、MINCO、SCP/OSQP 与 LOS soft/slack 链。没有调整 SIDE offset、权重、SCP trust、selector、K3 window 或安全阈值，没有运行 OFF 或测试套件。用户要求清理 ALP 进程后，已确认无残留并在低外部 CPU 负载下追加一次相同 FULL ON；没有访问 RRCT。

## 源码修改与接口不变量

在 [planner_manager.cpp](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp) 中，最终 `side_init_traj` 已在 A*/backoff、Local-SFC 和 PVA timing 后冻结；SIDE scope 在进入主 MINCO/SCP 前把这条多项式交给 optimizer。立即用同一 reference 计算 seed topology violation，超过 `1e-6` 或 reference 无效即 fail-closed。Local fallback、trial/final topology audit 继续调用同一 evaluator。`[SIDE_TOPOLOGY_SEED_REFERENCE]` 记录来源、PVA timed、有效性与偏离。

[poly_traj_optimizer.cpp](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/src/poly_traj_optimizer.cpp) 的 hard region 现在使用最终 seed 在归一化进度 `u` 的 signed lateral `σ_seed(u)`，下界为 `max(0, σ_seed − max(0.10, 0.25|σ_seed|))`。下界不大于 `1e-9` 时不生成 hard row；其余只生成 `σ_candidate ≥ lower` 的单侧行，不再有 hard upper row。Local/Team 的 SCP 行和 `candidateSideRegionViolation()` 均从这个冻结 reference 求值。数值符号核对：若 seed signed lateral 为 0.8 m，下界为 0.6 m，零步和向外侧 `+0.1 m` 均满足 `-JΔx ≤ 0.2`；向内 `−0.3 m` 不满足。解析 `sin(pi*u)` 仍用于原有 SIDE 初值或软形状，不再决定 hard topology 有效性。

跨进程 Team realized SIDE 校验原先也有独立正弦上下带。本轮通过 `TeamTrajectoryAck` 携带冻结 seed 多项式，coordinator 重建且校验 identity 后交给 `RealizedTeamValidator`；validator 以相同的 seed 下界和单侧规则校验 realized trajectory。Team 的合约、margin、trust、提交和激活逻辑没有改。第二轮有 4 条可解析的 0.2 s 短 SIDE seed 被 reference 接口拒绝，记录为 `SIDE_TOPOLOGY_SEED_INTERFACE_INVALID`；原因不能仅凭现有日志进一步区分，但这些候选未绕过 fail-closed 门。

`git diff --check` 通过；`catkin build -j2 --no-status` 为 **25/25 packages succeeded，package warnings 0**。catkin 仍打印缓存与当前 `CMAKE_PREFIX_PATH` 不同的环境提示；这不是 package 编译警告。正式仿真后没有再改 production 源码。

## FULL ON 尝试及有效性

三次均用 v6 场景、`--ablation full --headless --timeout 240 --boot-timeout 180 --k3-repair on --k3-escalation on`。前两次在 ALP 进程清理前，第三次在确认无残留 ROS/ALP 进程、内存充足且外部 CPU 负载回落后启动。

| RUN_ID | 退出/最后启动阶段 | 首个可证的活性问题 | 清理 |
|---|---|---|---|
| `20260927_132753_1316604` | `FINAL_EXIT_CODE=8`，BOOT-08 | UAV1 trajectory 3 在 `1790486886.527` 到期进入 terminal hold；后续 UAV0/UAV1 长时间无可用 successor，BOOT-08 等不到 UAV0 的新 trajectory | 自有 29/29 退出，强杀 0 |
| `20260927_133236_1329404` | `FINAL_EXIT_CODE=8`，BOOT-08 | `1790487177.894` 左右 UAV1/UAV2 及紧接着 UAV0 的 NOMINAL 轨迹到期进入 hold；没有有效 visibility 样本 | 自有 29/29 退出，强杀 0 |
| `20260927_134239_1338169` | `FINAL_EXIT_CODE=11`，BOOT-11 | 通过 BOOT-08/09/10 后，UAV1/UAV2 轨迹分别在 `1790487785.278/.276` 到期进入 hold。现场 `tracking_ready` 为 UAV0 true、UAV1/UAV2 false；latched `target_start_time` 未发布，目标启动门无法打开 | 自有 28/28 退出，强杀 0 |

**有效 BOOT-12 FULL run 数为 0。** 第三次在外部负载降低、ALP 进程已清理后仍发生轨迹到期，故“仅由之前残留进程或外部 CPU 负载造成”没有得到支持。运行中的求解 CPU 负担和 successor coverage 之间的精确因果尚未由本轮逐候选 deadline/activation 证据证明；不能把这些 hold 直接归因于 seed topology 改动，也不能当作无安全回退。runner 的 BOOT-11 在第三次没有收到 target start 是真实 readiness 未满足，并非一次性 topic 消息被漏采：target coordinator 的 publisher 是 latched，现场 UAV1/UAV2 readiness 为 false。

## 接口与 SCP 遥测

以下均为各**无效启动 run** 中可完整解析的日志行；日志存在 ROS 线程交织，故不是所有调用的穷举。第三次的 visibility CSV 为空，不能用它做 K3 比较。

| 指标 | run 1 | run 2 | 清理后 run 3 |
|---|---:|---:|---:|
| 最终 Local SIDE seed reference | 3,860 | 471 | 680 |
| reference 有效 | 3,860 | 467；4 条 fail-closed | 680 |
| seed violation P50 / P95 / max | 0 / 0 / **0 m** | 0 / 0 / **0 m**（可解析值） | 0 / 0 / **0 m** |
| 有效 FRESH / WARM / A* seed 数 | 3,847 / 9 / 4 | 467 / 0 / 0 | 679 / 1 / 0 |
| 各来源 seed violation max | 0 / 0 / 0 m | 0 / 未观测 / 未观测 | 0 / 0 / 未观测 |
| PVA-timed seed violation max | 0 m | 0 m（4 条无效 reference 被拒） | 0 m |
| 可解析 SIDE constrained SCP attempts / `SCP_FINAL_OK` | 3,886 / 694 | 592 / 578 | 739 / 726 |
| 有非零 SIDE hard rows 的尝试 | 3,254 | 591 | 729 |
| 成功 SCP 的 final topology violation max | 0.001785 m | 0.000945 m | 0.001141 m |
| 成功 SCP 中 violation >0.002 m | 0/694 | 0/578 | 0/726 |
| fallback region checked / accepted / region rejected | 3,860 / 3,163 / 0 | 598 / 25 / 0 | 738 / 15 / 0 |

Feedback136 的旧独立解析带有 **1,003/1,165** 个 SIDE seed 初始落在 hard band 外，seed violation P50/P95/max 为 `0.143398/1.090664/2.890938 m`。本轮最终 seed 派生下界后，所有有效且可解析的 seed violation 均为 0，包括第一轮 4 条 A* 修复 seed。旧现象与“最终 seed 经修复/retiming 后不同于解析正弦带”的源码机制一致；本轮 production 遥测证明新的 seed→SCP 接口自洽。它没有逐一重放旧 1,003 个候选，不能宣称每个旧候选的个体命运已被证明。

**SCP 可达率提升未获有效 FULL 证明。** run 1 的 `694/3886=17.9%` 与 Feedback136 `216/1165=18.5%` 分母和执行状态不同，且 run 1 大量尝试发生在终端 hold 之后；run 2/3 的高比率同样来自无目标启动的非生产分布，不能当作改善。run 1 的主要失败码已变为 `DYNAMICS_TRUST_RETRY_EXHAUSTED=3179`；旧 `P_TRUST_TOO_SMALL`、`T_TRUST_TOO_SMALL`、`P_T_TRUST_TOO_SMALL`、`TRUST_RETRY_EXHAUSTED`、`SIDE_REGION_FINAL_VIOLATION` 在这三次可解析 SCP reason 中均为 0。这只说明旧几何不一致标签消失，不能证明当前 dynamics/trust 问题可用调参解决。成功 SCP 与 fallback 的同一 reference 检查没有观察到 topology collapse；独立 blocker-relative oracle 和有效执行验证仍缺失。

## 安全、可见性与下一根因

run 1 虽保存了 2,344 个 visibility 样本，`K3=0.090870`、`K2=0.104096`、K3 loss `71.104428 s`、最长 `71.104428 s`，但这是 target/peer 长时间 hold 的**无效启动诊断**，不能与 Feedback134 的 `K3=0.891351` 或 Feedback136 的 `0.884157` 当成同口径生产效果比较。run 2/3 没有有效 visibility 样本。run 1 的执行轨迹 CSV 有 11,133 条 `TERMINAL_HOLD`/`safety_validated=False` 样本；采样静态/动态接触均为 0，最小净空 0.220612/0.846489 m。没有独立证据将标记缺席提升为所有 PVAJ commit 或 hard preflight 的连续证明。

本轮第一系统活性瓶颈是**启动阶段 successor coverage 断档：轨迹物理终点先于可执行 successor 激活，terminal hold 使 tracking readiness 或后续轨迹发布停止**。run 1 的 trajectory 3 在 `1790486886.527` 到期；相关 trajectory 4 的 RECEIVE 日志记录 activation `1790486886.577` 且 `accepted=0`，比旧轨迹到期晚约 50 ms。该时间链证明 gap 已存在；`accepted=0` 的内部拒绝原因不能由交织日志精确归属。run 3 则在目标启动前 UAV1/UAV2 同时进入 hold，直接阻断 target start。此问题需要单独的 successor/activation 审计，不在本轮 topology region 修改范围内；不应为掩盖它修改 trust、K3 selector 或安全阈值。

三个 run 的 stdout、console 和 ROS `.log*` 已压缩；保留 CSV、manifest、exit/process status 及完整可解压证据。对应目录约 150/168/115 MB，磁盘剩余约 5.6 GB；临时 runner 镜像日志已删除。最终检查未发现 ALP 仿真残留进程。

## 收尾字段

```text
PRODUCTION_CODE_CHANGED: YES
BUILD_PASS: YES (25/25, package warnings 0)
SEED_DERIVED_TOPOLOGY_REFERENCE: YES
ANALYTIC_SIN_HARD_TOPOLOGY_AUTHORITY_REMOVED: YES (sin remains for initial/soft shape)
ONE_SIDED_TOPOLOGY_HARD_ROW: YES
FULL_ON_BOOT12_COMPLETED: NO (0 valid / 3 attempts)
RUN_IDS: 20260927_132753_1316604, 20260927_133236_1329404, 20260927_134239_1338169

TOTAL_LOCAL_SIDE_SEEDS: 3860 / 471 / 680 parseable by run
SEED_TOPOLOGY_REFERENCE_VALID: 3860/3860, 467/471, 680/680; 4 invalid fail-closed
SEED_TOPOLOGY_VIOLATION_P50: 0 / 0 / 0 m
SEED_TOPOLOGY_VIOLATION_P95: 0 / 0 / 0 m
SEED_TOPOLOGY_VIOLATION_MAX: 0 / 0 / 0 m among parseable values
FRESH_SEED_VIOLATION_MAX: 0 m in all three runs
WARM_SEED_VIOLATION_MAX: 0 m where observed (run 1 and 3)
ASTAR_REPAIRED_SEED_VIOLATION_MAX: 0 m (run 1, n=4); no A* seed in run 2/3
PVA_TIMED_SEED_VIOLATION_MAX: 0 m for valid references

SCP_ATTEMPTS: 3886 / 592 / 739 parseable
SCP_FINAL_OK: 694 / 578 / 726
P_TRUST_TOO_SMALL: 0 / 0 / 0 parseable reasons
T_TRUST_TOO_SMALL: 0 / 0 / 0 parseable reasons
P_T_TRUST_TOO_SMALL: 0 / 0 / 0 parseable reasons
TRUST_RETRY_EXHAUSTED: 0 / 0 / 0 exact labels; DYNAMICS_TRUST_RETRY_EXHAUSTED=3179 in run 1
SIDE_REGION_FINAL_VIOLATION: 0 / 0 / 0 parseable reasons
FINAL_TOPOLOGY_VIOLATION_MAX: 0.001785 / 0.000945 / 0.001141 m among SCP_FINAL_OK
SIDE_TOPOLOGY_COLLAPSE_COUNT: 0/694, 0/578, 0/726 above 0.002 m
FALLBACK_REGION_REJECT: 0 / 0 / 0
FALLBACK_REGION_ACCEPT: 3163 / 25 / 15 (invalid-run distributions)

K3: NOT_VALIDATED; run 1 diagnostic 0.090870
K2: NOT_VALIDATED; run 1 diagnostic 0.104096
K3_LOSS_TOTAL: NOT_VALIDATED; run 1 diagnostic 71.104428 s
LONGEST_K3_LOSS: NOT_VALIDATED; run 1 diagnostic 71.104428 s
MEAN_VISIBLE: NOT_VALIDATED; run 1 diagnostic 1.169369
BLACKOUT: NOT_VALIDATED; run 1 diagnostic 60 samples
K3_EPISODES: NOT_VALIDATED; run 1 diagnostic 1
LOS_CAUSED_EPISODES: NOT_DETERMINED

CONTACTS: run 1 sampled static 0 / dynamic 0; run 2/3 no visibility execution CSV
PVAJ_VIOLATION_COMMITS: 0 explicit markers observed, exhaustive proof unavailable
UNVALIDATED_COMMITS: 0 explicit markers observed; UNVALIDATED_EXECUTED=11133 run 1 hold samples
SUCCESSOR_STARVATION: 53 / 91 / 64 logged mentions in invalid runs
TERMINAL_HOLD_ENTER: 88 / 190 / 184 logged mentions
PLANNER_DEADLOCK: startup liveness failure YES; permanent planner-process deadlock not established
SAFETY_ACCEPTANCE: FAIL in invalid runs due terminal hold/unvalidated execution
SAFETY_REGRESSION: NOT COMPARABLE as valid FULL runs; no causal attribution to topology change
SCP_REACHABILITY_IMPROVED: NOT_DETERMINED (no valid FULL)
PRIMARY_REMAINING_ROOT_CAUSE: successor coverage gap/terminal hold before target mission start; exact upstream scheduling or candidate-causation unresolved

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```

**明确回答：** Feedback136 的 `1003/1165` seed-band 冲突与“最终 SIDE seed 和独立解析正弦 hard region 使用两套几何真值”一致；本轮改为最终 `side_init_traj` 派生单侧下界后，已观察到的有效 seed→SCP 接口闭合、成功 SCP 的最终 topology 保持。由于三次仿真均未到 BOOT-12，**SCP 可达率提升和 K3 生产效果尚未被证明**，当前先卡在启动阶段 successor coverage/terminal hold。

证据：[逐 run 指标](artifacts/feedback137_metrics.json)、[分析脚本](artifacts/feedback137_analyze.py)、[run 1 状态](../runs/20260927_132753_1316604/exit_status.txt)、[run 2 状态](../runs/20260927_133236_1329404/exit_status.txt)、[清理后 run 3 状态](../runs/20260927_134239_1338169/exit_status.txt)。各 run 目录同时保留压缩 stdout、进程清理记录及已有 CSV。
