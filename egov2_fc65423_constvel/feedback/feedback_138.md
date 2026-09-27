# Feedback 138 — 单次 FULL ON 复核

日期：2026-09-27。工作区：`/home/bob/ALP/egov2_fc65423_constvel`。本轮没有修改 production code、编译或运行 OFF；只在确认无 ALP/ROS 残留进程、宿主 CPU 负载低后启动一次 canonical FULL ON。场景为 `natural_team_stress_dense_38_targeted_k2_v6.json`，参数为 `--ablation full --headless --timeout 240 --boot-timeout 180 --k3-repair on --k3-escalation on`。

## 运行状态

RUN_ID：`20260927_155447_1343008`。BOOT-12 达到，`CORE_EXIT_CODE=0`、`FINAL_EXIT_CODE=0`。runner 登记的 29 个自有进程全部退出，强杀 0、残留 0、杀外部进程 0。原始 stdout、console 和 ROS `.log*` 已压缩；保留 CSV、manifest 和状态文件。run 目录约 188 MB，收尾磁盘可用约 5.4 GB。

## SIDE seed→SCP 接口

可解析的最终 Local SIDE seed reference 为 **674 条，674/674 有效**；seed topology violation P50/P95/max 均为 **0 m**。其中 FRESH 647、WARM 25、A* 修复 2，三类最大 violation 均为 0。可解析的 SIDE constrained SCP 为 **689 次**，`SCP_FINAL_OK=663`；531 次记录了非零 SIDE hard rows。成功 SCP 的最终 topology violation 最大 **0.001739 m**，超过 0.002 m 的成功结果为 **0/663**。fallback region 检查 687 次，接纳 14 次，因 region 拒绝 0 次。相较 Feedback136 的 216/1165 个 `SCP_FINAL_OK`，本轮可解析比例为 663/689；两轮候选分布不同，这只能说明本次生产运行中接口可达，不能把比率差归因于单一修改。

## 可见性

目标窗口的 2,348 个 visibility 样本覆盖 **78.268397 s**。ALL3/K3 为 **0.888842**，K2 为 **1.000000**，平均可见数 **2.888842**，blackout 样本 0。K3 loss 左端零阶积分 **8.694550 s**，9 段，最长 **1.963200 s**。Feedback136 对应 K3 0.884157、K2 0.994037、K3 loss 9.074259 s、最长 1.702291 s：本次总体 K3 和 K2 略好，但最长单段 K3 loss 更长。单次 run 的差值不证明稳定改善。

## 硬安全与活性：本轮不通过

目标窗口的执行轨迹 CSV 有 7,041 条 UAV 样本，`TERMINAL_HOLD` 和 `safety_validated=False` 均为 0；最小静态净空 **0.270474 m**，采样最小机间距约 **1.123339 m**。但是 UAV1 在 `t=57.833083–58.070792 s` 的 **8 条样本**与 moving obstacle **id5** 接触：CSV `moving_clearance_m=0`，执行源为 `NOMINAL` trajectory 211，且这些行的 `safety_validated=True`。

这是圆柱内部接触，不只是显示数值四舍五入。场景 id5 为半径 0.28 m、中心 x=18.8 m、沿 y 轴移动的圆柱；场景日志确认动态运动与 target start `1790495693.206115` 同步。用 CSV 位置与同一场景运动公式复算，8 个样本的中心径向穿入量约 **0.003–0.131 m**，最深在 `t=57.934019 s`；该段 UAV z 为 1.087–1.132 m，处于圆柱高度范围内。最早接触的执行时刻为 `1790495751.039198`。该证据证明执行轨迹与动态障碍发生接触；它尚不能单独判定是预测、优化、preflight、调度还是执行跟踪哪一环首次产生偏差。`safety_validated=True` 是来源标记，不能覆盖实际几何接触。

目标窗口之后，完整 274.566 s 执行 CSV 有 **16,249 条** `TERMINAL_HOLD` 且 `safety_validated=False` 的样本，第一条在 `t=81.999663 s`，晚于目标窗口。完整日志有 41 条 `TERMINAL_HOLD_ENTER`、4 条 `MOVING_SUCCESSOR_STARVATION` 提及。目标路线完成、BOOT-12 和退出码 0 都不能替代系统安全/活性验收。

## 结论

**seed→SCP topology 接口在这次有效 FULL 中闭合，K3/K2 指标较 Feedback136 略好；硬安全验收失败。** 当前首先需要审计 UAV1 trajectory 211 与 moving obstacle 5 在 `t≈57.83 s` 的动态预测、hard preflight 和实际执行时间/位置链，找出最早分歧。由于接触发生在 `NOMINAL` 轨迹上，不能仅凭本次结果把它归因于 SIDE topology region；也不能因为 `safety_validated=True` 而忽略接触。本轮不据此修改 trust、权重、selector 或安全阈值。

```text
RUN_ID: 20260927_155447_1343008
FULL_ON_BOOT12_COMPLETED: YES
FINAL_EXIT_CODE: 0
LOCAL_SIDE_SEED_REFERENCE_VALID: 674/674
SEED_TOPOLOGY_VIOLATION_P50_P95_MAX: 0 / 0 / 0 m
ASTAR_REPAIRED_SEED_VIOLATION_MAX: 0 m (n=2)
SIDE_SCP_FINAL_OK: 663/689 parseable
SIDE_FINAL_TOPOLOGY_VIOLATION_MAX: 0.001739 m
SIDE_TOPOLOGY_COLLAPSE_GT_0P002: 0/663
K3: 0.888842
K2: 1.000000
K3_LOSS_TOTAL: 8.694550 s
LONGEST_K3_LOSS: 1.963200 s
STATIC_CONTACT_SAMPLES: 0
DYNAMIC_CONTACT_SAMPLES: 8 (UAV1 vs moving obstacle 5, target window)
MAX_CENTER_PENETRATION_APPROX: 0.131 m
UNVALIDATED_EXECUTED_TARGET_WINDOW: 0
UNVALIDATED_EXECUTED_FULL_RUN: 16249 terminal-hold samples
HARD_SAFETY_ACCEPTANCE: FAIL
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```

证据：[exit status](../runs/20260927_155447_1343008/exit_status.txt)、[process cleanup](../runs/20260927_155447_1343008/process_status.txt)、[visibility CSV](../runs/20260927_155447_1343008/visibility.csv)、[execution trajectory CSV](../runs/20260927_155447_1343008/visibility_trajectory.csv)、[compressed stdout](../runs/20260927_155447_1343008/roslaunch_stdout.log.gz)、[scene](../ros_ws/src/multi_uav_formation/scenes/natural_team_stress_dense_38_targeted_k2_v6.json)。
