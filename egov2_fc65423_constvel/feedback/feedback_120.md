# Feedback 120 — A* seed timing 与 SIDE intent 接口复核

日期：2026-09-26。工作区：`/home/bob/ALP/egov2_fc65423_constvel`。本轮只改两个指定的 production 接口；完成修改、`git diff --check`、25/25 catkin 构建和离线检查后，仅运行一次 FULL ON。FULL 后未改 production、未运行 OFF、未重跑。唯一 run：`20260926_105704_628281`。

## 结论

**SIDE 语义硬拒已退出 A*→guide→corridor 链；PVAJ 一次时间缩放已执行，但 corridor SCP 仍为 0/16。** 与 Feedback119 相比，A* raw static-safe route 为 117/117（原 74/74），旧 `GUIDE_REJECTED` 硬拒为 0（原 63），17 条构造了 corridor，16 条进入同一条 MINCO-SCP 链。新 run 中一条 `intent_consistent=0` 的 SIDE_MINUS 在 `1790391468.2604` 建成 24-plane corridor，并在 `1790391468.3384` 到达 SCP 失败出口，直接证明局部反向路径不再被语义门提前杀死。Feedback119 的 63 条历史 route 没有保存完整路径坐标/候选身份，无法逐条认定“同一条”在新 run 进入 corridor；证明口径是相同场景下不一致拓扑路径已进入。

**本轮不能宣称数值初始化已打通 corridor。** 可完整解析的 timing 行中，v/a/j P50 从约 `2.997 / 23.747 / 486.246` 降至 `1.197 / 2.905 / 21.062`；但 108 条可解析的 `j_after` 有 90 条仍高于 20，最大 166.280。原因是一次 `λ` 按时间尺度估计，却按要求保留真实端点 PVA；重新生成 MINCO 后，边界导数不随 λ 缩放，故 `v∝λ⁻¹、a∝λ⁻²、j∝λ⁻³` 不再是整个新 seed 的精确恒等式。没有增加第二次 dilation 或 retry。

更早的可证 corridor 测量缺陷位于**本轮禁止修改**的连续 violation 检查。`continuousLocalSfcMaxViolation()` 在 [poly_traj_optimizer.cpp](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/src/poly_traj_optimizer.cpp#L333) 中先执行 `t_piece_start = t_piece_end`，再用 `lo - t_piece_start` 求 piece-local 时间；这会在每个 piece 上以**终点而非起点**为时间原点。确定性审计用同一 MINCO 类型得到：世界时刻 0.64 s 的直接位置 x=0.173292，正确局部多项式同为 0.173292，现有索引算法得 0.092379；对于 x≥0.14 的平面，真实违反为 0，而该算法报 0.047621。另一个源码细节是五次多项式导数的 `c(5)` 项未送入只接收四个系数的 `quarticRealRoots()`；本轮未作改动或单独归因。连续指标不能在修正前作为“真实 corridor/PVA 不可行”的完整证据。

## 两处接口修改与时间语义

- [planner_manager.cpp](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp#L11079)：A* raw route admission 只检 raw 点静态占据；guide 简化、corridor handoff 和 `astar_accept` 不再读取 path-wide sign / 60% dominance。旧分数仅记 `[SIDE_TOPOLOGY_INTENT] ... used_as_hard_gate=0`。未引入 primary-blocker 新硬门；无法稳定定义截面时记录 `UNKNOWN`。
- [planner_manager.cpp](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp#L11922)：保留长度与曲率给出的原时标，先生成 seed，计算全轨迹 v/a/j，取 `λ=max(1,v/3,√(a/6),∛(j/20))`，统一缩放全部 piece duration，并只重新生成一次同 route MINCO seed。静态 corridor 的 `active_start/end` 同比缩放；世界时刻锚定的 LOS plane 不缩放，随后 dynamic LOS / risk 在新 trajectory 时标上重新评估，下游 horizon 取新 duration。原来 `corridor_repair_duration` 实际累加的是长度，本轮改为累计 segment duration，路径长度另存遥测。未改 corridor 几何、SCP trust、OSQP、阈值、K3/M2、C3/D3 或预算。
- `[A_STAR_SEED_TIMING]` 记录缩放前后实际值。116 条 marker 中可解析 λ 109 条、完整 after v/a/j 108 条；ROS stdout 存在异步行交错，以下分位数只以可解析子集为分母，不能将缺字段的行算作未缩放。109 条可解析 λ 均大于 1；λ P50/P90/max = `2.897 / 3.379 / 5.352`。`j_before` P50/max=`486.246 / 3065.865`，`j_after` P50/max=`21.062 / 166.280`；a_before/max=`23.747 / 115.943`、a_after/max=`2.905 / 12.728`；v_before/max=`2.997 / 12.491`、v_after/max=`1.197 / 2.977`。完整 before/after duration P50 约 `4.687→12.500 s`。

## 仿真前离线验证

`git diff --check` 通过；`catkin build -j2 --no-status` 为 **25/25 succeeded、package warnings 0**。catkin 仅提示 shell 的 `CMAKE_PREFIX_PATH` 与缓存不同，最终 build summary 无 warning。运行 [MINCO 时间检查](artifacts/feedback120_timing_check.cpp) 的三段、每段 0.058926 s、路径 0.424264 m 同尺度重建（Feedback119 原日志 jerk=20731.394，但未保存完整 PVA/waypoints）得到 jerk `15449.939→20.000`、λ=`9.175553`；静态窗等比移动、LOS world anchor 保持不动。这证明一次缩放机制，不构成旧候选的逐系数精确 replay。

[SIDE admission 检查](artifacts/feedback120_side_admission_check.py) 读取 Feedback119 的 `raw_path_static_free=1 / path_side_valid=0 / GUIDE_REJECTED` 旧记录，核对新源码的 raw admission、guide 与 corridor 条件不再使用 side semantic 硬门。原 30 点 route 坐标未保留，所以离线只能重放**旧判定类别与新 admission 逻辑**。LOS hard reject 源码路径未改，新 run 末次三机遥测合计为 0。M2、C3/D3 comparator 的 production 区段无本轮改动。连续检查的反例程序为 [feedback120_continuous_audit.cpp](artifacts/feedback120_continuous_audit.cpp)；它在 FULL 后作为**分析证据**创建，没有回改 production。

## FULL ON 结果及首个失败

命令为用户指定的 `./scripts/run_alp_full_on.sh --ablation full --headless --timeout 240 --boot-timeout 180 --scenario .../natural_team_stress_dense_38_targeted_k2_v6.json --k3-repair on --k3-escalation on`。`FINAL_EXIT_CODE=0`、BOOT-12 成功；runner 29 个自有进程全部退出，强杀/残留/外部进程被杀均为 0。日志留在本 run，重复 stdout 和 ROS log 已无损压缩，run 约 55 MB。

| 项目 | Feedback119 | 本 run | 判断 |
|---|---:|---:|---|
| ASTAR_REQUIRED / RAW_PATH_SAFE | 74 / 74 | **117 / 117** | 静态安全 A* 供给增加 |
| GUIDE_REJECTED_HARD | 63 | **0** | 旧语义门已退出 |
| SIDE intent inconsistent telemetry | 旧门直接拒绝 | **31 / 117** | 至少一条不一致路径进 corridor/SCP |
| CORRIDOR_BUILD_SUCCESS | 10 | **17** | 几何构造未成为首因 |
| CORRIDOR_SCP attempt / success | 10 / 0 | **16 / 0** | 数值链未闭环 |
| QP max-iter reason | 1 | **3** | 未下降 |
| trust-exhausted/small reason | 5 | **7** | 未下降 |
| EXECUTION_DEADLINE (`side-minco` status) | 188 | **197** | 未下降；口径为本 run SIDE solve status |
| MIN_BUDGET_APPLIED | 78 | **0** | 恢复 |

16 次 corridor 失败互斥出口：`TRUE_CONSTRAINT_INFEASIBILITY=5`、`DYNAMICS_MODEL_MISMATCH_TRUST_EXHAUSTED=6`、`P_TRUST_TOO_SMALL=1`、`QP_MAX_ITER_EXHAUSTED=3`、`LOCAL_SFC_FINAL_VIOLATION=1`。这几个出口描述**当前 QP/验证链**的结果，不足以直接判为真实物理 corridor 不可行：连续 violation 计算已有确定性反例。16 次 SCP 初始 `seed_continuous_violation` P50/max 为 `6.149 / 90.947 m`。

**首个可追溯失败**在 `1790391463.8219–1463.8629`：SIDE_PLUS、obstacle 7，A* raw path 与 20-plane corridor 成功；`[A_STAR_SEED_TIMING]` 报 `j 299.711→20.965`、`λ=2.465`、duration `2.736→6.746 s`。随后连续 corridor 初始违反报 **34.382 m**，而失败出口的 sampled corridor violation 为 **0.275795 m**；同轮 337 行 QP 报 `primal_infeasible`，移除 P trust 探针也不可行，最终 `TRUE_CONSTRAINT_INFEASIBILITY`。确定性反例证明连续 violation 的时间原点错误能制造假违反；日志证明此候选还有 0.965 的 jerk 超限和有限的 sampled corridor 违反。**现有证据不能把该次 QP 无解唯一归因于错误时间原点、真实 corridor/PVA 行冲突或剩余 jerk**。优先应先修测量，使后续不可行诊断可信；这属于用户本轮明确禁止改动的区域，故没有补丁和第二次 FULL。

## 执行、可见性与硬安全

moving phase `t≤76.5 s`：ALL3=`0.901525`（Feedback119 `0.891503`），K2=`0.991285`（原 `0.994771`）；ALL3 loss 总时长/最长=`7.531772 / 1.633155 s`，K2 loss=`0.666941 / 0.666941 s`，blackout=0。K3 Local events=16、已记录 recovery=4（原 14/5）；单次不同执行轨迹，不将变化归因于任一接口。静态/动态接触=0/0，最小净空=`0.325628 / 0.391899 m`；机间最小距约 `1.047 m`，swarm violation log=0。PVA mismatch、unvalidated executed、partial Team activation、successor starvation、terminal hold 均为 0。硬安全未观察到回退；K2 与 corridor 成功率仍不达预期。

## 对六个验收问题的回答

1. PVAJ timing 确实压低初值，但 **corridor SCP 仍 0/16**；保留端点 PVA 后，一次 λ 不是严格的全轨迹动力学上界。
2. 旧 path-wide SIDE hard gate **已删除**；新 telemetry 不参与 A*、corridor、SCP 或 preflight 拒绝。
3. 原 63/74 的逐 route 身份**不可证**；新 run 有 `intent_consistent=0`、static-safe 的 route 实际进入 corridor 和 MINCO。
4. QP max-iter `1→3`、trust 类 `5→7`、deadline `188→197`，**没有下降**；MIN_BUDGET `78→0`。
5. 接触、swarm、PVA、未验证执行、partial activation 无新增；**K2 质量低于 Feedback119**，不称为整体质量提升。
6. 首个可证的诊断缺陷是既有连续 corridor 检查的 piece-local 时间原点错误；同时有残余 jerk 和真实 sampled corridor 违反。不能把所有 16 次归为真实 corridor/PVA infeasibility、dynamic/swarm、纯数值或预算中的唯一一类。

## 收尾字段

```text
PRODUCTION_CODE_CHANGED: YES  # 仅 A* seed timing 与 SIDE semantic admission 接口
BUILD_PASS: YES  # 25/25; package warnings 0
GIT_DIFF_CHECK: CLEAN

PVAJ_TIMING_INITIALIZER_ACTIVE: YES
SECOND_DILATION_OR_RETRY_ADDED: NO
PATH_WIDE_SIDE_HARD_GATE_REMOVED: YES
SIDE_TOPOLOGY_HARD_REJECT_PRESENT: NO

ASTAR_REQUIRED: 117
ASTAR_RAW_PATH_SAFE: 117
GUIDE_REJECTED_HARD: 0
SIDE_INTENT_INCONSISTENT_TELEMETRY: 31
CORRIDOR_BUILD_SUCCESS: 17

A_STAR_SEED_TIMING_COUNT: 116 markers; 109 parseable lambda; 108 complete post-rates
TIMING_DILATION_APPLIED: 109 confirmed from parseable lambda
LAMBDA_P50/P90/MAX: 2.896958 / 3.379175 / 5.351896 (n=109)
INITIAL_VEL_BEFORE_P50/MAX: 2.996802 / 12.491074
INITIAL_VEL_AFTER_P50/MAX: 1.196751 / 2.977458
INITIAL_ACC_BEFORE_P50/MAX: 23.747093 / 115.943292
INITIAL_ACC_AFTER_P50/MAX: 2.905014 / 12.728174
INITIAL_JERK_BEFORE_P50/MAX: 486.246426 / 3065.865066
INITIAL_JERK_AFTER_P50/MAX: 21.062062 / 166.280360

CORRIDOR_SCP_ATTEMPT: 16
CORRIDOR_SCP_SUCCESS: 0
CORRIDOR_SCP_INFEASIBLE: 16  # includes all unsuccessful outlet reasons
QP_MAX_ITER_EXHAUSTED: 3
TRUE_CONSTRAINT_INFEASIBILITY: 5  # current linearized QP diagnosis, not proven physical impossibility
TRUST_EXHAUSTED_OR_SMALL: 7
LOCAL_SFC_FINAL_VIOLATION: 1
LOS_HARD_REJECT_COUNT: 0

EXECUTION_DEADLINE: 197 SIDE-MINCO statuses
MIN_BUDGET_APPLIED: 0
K3_EVENTS: 16
K3_RECOVERED: 4
EXECUTED_ALL3: 0.901525
EXECUTED_K2: 0.991285
ALL3_LOSS_TOTAL: 7.531772 s
K2_LOSS_TOTAL: 0.666941 s
LONGEST_ALL3_LOSS: 1.633155 s
LONGEST_K2_LOSS: 0.666941 s
BLACKOUT: 0

STATIC_CONTACT: 0
DYNAMIC_CONTACT: 0
SWARM_VIOLATION: 0 observed logs; min moving-phase separation 1.047 m
PVA_MISMATCH: 0
UNVALIDATED_EXECUTED: 0
PARTIAL_TEAM_ACTIVATION: 0
STARVATION: 0
TERMINAL_HOLD: 0

FIRST_CAUSAL_FAILURE: first corridor SCP starts with false-prone continuous violation (wrong piece-local time origin), residual jerk and sampled corridor violation; QP primal infeasible, exact conflict set unproven
NEXT_STEP: in a separately authorized scope, correct/audit the existing continuous checker time origin and quintic derivative, then make one-pass timing PVA-consistent and re-evaluate the existing SCP; do not infer a need for new SFC or trust from this run

FULL_RUN_COUNT_THIS_TASK: 1
REPEATED_FULL_SIMULATION_USED: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```

证据：[run exit](../runs/20260926_105704_628281/exit_status.txt)、[cleanup](../runs/20260926_105704_628281/process_status.txt)、[完整 stdout](../runs/20260926_105704_628281/roslaunch_stdout.log.gz)、[ROS log](../runs/20260926_105704_628281/ros_log.tar.gz)、[visibility CSV](../runs/20260926_105704_628281/visibility.csv)、[执行轨迹 CSV](../runs/20260926_105704_628281/visibility_trajectory.csv)、[聚合指标](artifacts/feedback120_metrics.json)、[聚合脚本](artifacts/feedback120_analyze.py)、[时间检查输出](artifacts/feedback120_timing_result.txt)、[连续检查反例输出](artifacts/feedback120_continuous_result.txt)。
