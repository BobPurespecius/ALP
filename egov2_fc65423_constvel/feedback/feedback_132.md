# Feedback 132 — 解析 jerk 极值 + 规划储备修复（第二/三轮闭环）

日期：2026-09-27。工作区：`/home/bob/ALP/egov2_fc65423_constvel`。两轮 production 修改、两次 FULL（run3 无效指标但因果有效、run4 有效）。

## 本轮根因

feedback_131 的剩余第一根因在源码与日志中确认：优化器与 SafetyKernel 对动力学"不同物理"。`forEachCandidateDynamicsSample()` 只在均匀 α 网格（每 piece `cps_num_prePiece_` 个样本）上放置 PVAJ 行与验收采样；jerk 是分段二次多项式，采样之间的峰值逃逸——run2 中 `SIDE_PLUS` 预检 `max_j=22.99 > 20`，而求解器 lattice 判其可行。`side_retimed=false` 已排除 retiming。同时 kernel 自己的 jerk 检查也是采样（dt≤0.05s），同样可能漏峰（unsafe 方向）。

第二轮还暴露了第一轮冻结激活公式的错误：reserve 用了 `execution_budget_.estimate()`（峰值保持、含 Team 协调等待，实测爬到 1.07–1.10s）。run3 中 drone1 的首个 batch 冻结 activation 至 plan_start+1.20s，把 predecessor 剩余覆盖窗口吞掉，生成器只产出 0.0999s 的 stationary sliver（37k 次 SHORT_STATIONARY 拒绝、15k batch replan 风暴），drone1 停摆 → UAV2 全盲 55.3s → ALL3=24.9%。

## 代码修改

1. **解析 jerk 极值 API**（`poly_traj_utils.hpp`，镜像既有 `getMaxAccRate` 模式）：
   - `Piece::normalizeJerCoeffMat()`、`Piece::getMaxJerRate()`（RootFinder 求 |jerk|² 导数根 + 端点，闭式精确）；
   - `Piece::getJerStationaryAlphas()`（内部驻点 α，供行约束放置）；
   - `Trajectory::getMaxJerRate()`（逐段聚合）。
2. **Kernel**：`checkTrajectoryDynamics()` 的 jerk 从 dt 采样改为 `traj.getMaxJerRate()`（解析精确；vel/acc 本就解析）。
3. **Optimizer**：`forEachCandidateDynamicsSample()` 在均匀样本之外加入每段 jerk 驻点采样——QP 行、SCP 验收与 kernel 从此测同一个物理量；`predictDynamicsForStep()` 用 `jerkOpt_.get_b()` 构造临时 `Piece` 复用同一 API，保持预测模型一致。
4. **冻结激活 reserve**：`prepareFutureActivation()` 改用 `local_planning_budget_`（纯规划延迟 EWMA，上限 0.12s，不含 Team 等待），保留 deadline clamp 与 margin floor。
5. **（第三轮后半）SIDE 求解入口 reserve 重校准**：`kSideSolveCallReserve=0.01`——batch 级 50ms call reserve 是按 nominal 求解时标设置的；实测 SIDE 收敛只需 2–5ms，K3 事件常态（batch 剩余 <50ms）下侧求解在 0.047ms 被入口拒绝（934/967 次 EXECUTION_DEADLINE 的直接机制）。batch wall deadline 不动，覆盖权威不变。

## 离线验证

`/tmp/jerk_exact_test.cpp`：200 条随机五次轨迹，`getMaxJerRate()` vs 1e-5 步长暴力采样，最大相对差 **1.7e-5，PASS**；驻点 α 全部位于 (0,1) 且其 jerk 不超过全局峰值。

## build 与 run

`catkin build -j2 --no-status`：两轮均 **25/25 succeeded、Warnings: None**。

- run3 `20260927_044838_944355`：BOOT-12、exit 0，但 **ALL3=24.9%、UAV2 停摆**——reserve 公式错误的发现现场（因果有效、指标无效）。
- **run4 `20260927_050350_978639`：BOOT-12、exit 0，本项目历史最优**。

| 指标 | baseline (225042) | run2 | run4 | 判定 |
|---|---:|---:|---:|---|
| ALL3/K3 | 0.892375 | 0.882789 | **0.894989** | 超过 baseline |
| K2 | 0.994771 | 1.000000 | **0.998693** | K2 loss 0.30s |
| blackout | — | 0 | **0** | ✓ |
| mean_visible | — | 2.8828 | **2.8937** | 最优 |
| K3 loss total | 8.23s | 8.97s | **7.83s** | 最优 |
| 全盲段数 | 7 | 8 | **8**（0.43–1.37s） | 同一场景遮挡走廊 |
| K3 batch 三方全灭 | 46/112 (41%) | — | **25/120 (21%)** | 减半 |
| UAV2 radius P50 | 6.06 | 6.06 | **1.30** | 跟踪质量大幅改善 |
| DYNAMICS_FAIL（kernel 拒绝） | 92 | 46 | 64（P50 jerk=261，全是 initializer 种子被正确击杀） | 拒绝对象换了层级 |
| POST_OPT_PAYLOAD_MUTATED | — | 0 | **0** | ✓ |
| TERMINAL_HOLD_ENTER | 29 | 194 | 175 | 死区 churn 仍在 |

## 是否真正改善

**是，历史最优**：ALL3/K3、K3 loss total、mean_visible、UAV 跟踪半径全部优于 baseline 与 run2；mutation 不变量保持为零；安全性零回退（blackout 0、接触 0）。

## 剩余第一根因

**SIDE 求解的 wall-clock 饿死**：934/967 (97%) 次 SIDE 求解以 `EXECUTION_DEADLINE` 死亡——nominal 求解 + 种子流水线（A*/SFC/检查）耗尽 batch 预算后，侧求解在最后 <50ms 到达，被 `execution_call_reserve_=0.05` 拒绝入口（实测死亡耗时 0.047–0.082ms，batch 剩余 46ms > 侧求解实际需要的 2–5ms）。此问题在 baseline 已存在（130/166=78%），不是本轮新引入；第三轮的 reserve 重校准（kSideSolveCallReserve=0.01）针对它，run5 验证。

次级：覆盖 <2·margin 死区的结构性 MISS churn（175 次 TERMINAL_HOLD、~4000 次 MISSED）——无 mutation 契约下无解，restart authority ~60ms 恢复，K2 未破。

## 下一步

run5 检验：EP1 窗口侧求解是否拿到真实预算、盲段是否向 0.47s 设计窗收窄；随后按数据决定是否处理种子流水线的 deadline 感知。

```text
PRODUCTION_CODE_CHANGED: YES
BUILD_PASS: YES  # 25/25, Warnings: None
FULL_RUN_COUNT_THIS_TASK: 2  # run3(指标无效/因果有效) + run4(有效)
REPEATED_FULL_SIMULATION_USED: NO

EXACT_JERK_KERNEL: YES  # getMaxJerRate vs brute force 1.7e-5
EXACT_JERK_OPTIMIZER_ROWS: YES  # 驻点采样入行/验收/预测
FROZEN_ACTIVATION_RESERVE: local_planning_budget_ (EWMA<=0.12s)
SIDE_SOLVE_CALL_RESERVE: 0.01  # run5 检验

K3_ALL3: 0.894989  # run4, 历史最优
K2: 0.998693
K3_LOSS_TOTAL: 7.83s
BLACKOUT: 0
CONTACTS: 0
UNVALIDATED_COMMITS: 0
PVAJ_VIOLATION_COMMITS: 0
POST_OPT_PAYLOAD_MUTATED: 0
TERMINAL_HOLD_ENTER: 175  # 死区 churn, restart 快速恢复
PLANNER_DEADLOCK: 0

FIRST_CAUSAL_FAILURE: SIDE 求解被 batch 预算尾部的 50ms call reserve 拒绝入口(97% EXECUTION_DEADLINE)
NEXT_STEP: run5 验证 reserve 重校准; 之后视数据处理种子流水线 deadline 感知

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```

证据：[run4 exit](../runs/20260927_050350_978639/exit_status.txt)、[run4 visibility CSV](../runs/20260927_050350_978639/visibility.csv)、[run3 visibility CSV](../runs/20260927_044838_944355/visibility.csv)、[jerk 精确性测试](../../../../../tmp/jerk_exact_test.cpp)。
