# Feedback 139 — K3 恢复漏斗的现存证据

日期：2026-09-27。仅审阅当前 ALP 源码和 Feedback138 的一次 FULL ON run `20260927_155447_1343008`；未改 production、未编译、未启动仿真。时间以 `visibility.csv` 的 `time_s` 为准，目标窗口约 78.268 s。旧 [Feedback134](feedback_134.md) 的候选多项式证据来自另一版诊断 run，不能移植为本 run 的 seed/final 结论。

## 结构判断

用户提出的两层区分成立。当前 [topology lower bound](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/src/poly_traj_optimizer.cpp) 在候选引导窗口内按 normalized trajectory progress 取 `max(0, signed_seed - max(0.10, 0.25*abs(signed_seed)))`；seed 尚未建立该侧（bound≤0）时不产生 hard row。该约束保存的是 blocker-relative 侧向下界，未给出最晚 LOS 恢复时刻。LOS observation rows 在同一次 SCP 中带非负 slack；[K3 comparator](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp) 在 hard preflight 和 K2 门后先比较 sampled C3，再比较 D3，未直接最小化完整窗口首次**持续**恢复时间。因此 `SIDE topology valid` 既不是 `LOS visible`，也不是 `recovery early` 的证书。

Feedback138 的 689 次可解析 SIDE SCP 中，663 次 `SCP_FINAL_OK`；本轮重读对应 `[LOCAL_SIDE_CONSTRAINED_SCP]` 行，成功者 **109/663** 的 `los_slack_max>1e-6`。这一事实证明成功求解可以留下 LOS 欠量，但日志没有为每个 slack row 提供世界时间、遮挡物 ID及完整 candidate polynomial，不能把 109 条直接归入某个 loss episode，也不能仅凭 slack 证明对应轨迹失明。

## 9 段实际 K3 loss

每段取 `visible_count<3` 的原始连续样本；loss 时长按左端零阶积分。blocker 从执行 UAV—target 射线与场景圆柱的相交复算，并与 CSV 的 STATIC/DYNAMIC LOS 标志核对：3 机 × 2 种 LOS × 2,348 样本，标志不一致数为 **0**。`SIDE/NOM/FB` 是**失明区间内执行**的源时长，按执行轨迹 CSV 的样本间隔近似积分。`ID` 是失明期间不同执行 trajectory ID 数，不是候选数。`T₀.₅` 是段后首次连续至少 0.5 s 全队 K3 可见的起点；相邻段之间的短暂可见不算持续恢复。

| 段 | t loss (s) | UAV / blocker（失明样本命中） | loss (s) | SIDE/NOM/FB (s) | ID | T₀.₅ (s) |
|---:|---:|---|---:|---:|---:|---:|
| 1 | 8.069–8.832 | U1 / STATIC 36 (23/23) | 0.764 | 0.300/0.463/0 | 3 | 8.832 |
| 2 | 10.533–11.699 | U1 / STATIC 26 (35/35) | 1.167 | 0.804/0.362/0 | 5 | 11.699 |
| 3 | 25.199–26.333 | U1 / STATIC 28 (34/34) | 1.134 | 0.831/0.303/0 | 7 | 26.333 |
| 4 | 34.966–35.868 | U3 / STATIC 6 (27/27) | 0.902 | 0.565/0.337/0 | 5 | 35.868 |
| 5 | 57.768–58.132 | U1 / DYNAMIC 5 (11/11) | 0.365 | 0/0.364/0 | 1 | 58.132 |
| 6 | 64.033–65.000 | U1 / STATIC 13 (29/29) | 0.967 | 0.800/0.167/0 | 5 | 65.000 |
| 7 | 70.869–72.833 | U3 / DYNAMIC 10 (47/59), STATIC 14 (19/59) | 1.963 | 1.429/0.534/0 | 8 | 74.833 |
| 8 | 73.233–74.033 | U2 / DYNAMIC 10 (24/24) | 0.800 | 0.400/0/0.399 | 3 | 74.833 |
| 9 | 74.199–74.833 | U3 / STATIC 37 (19/19) | 0.633 | 0.500/0.133/0 | 2 | 74.833 |

九段总 loss **8.69455 s**。六段纯静态约 **5.567 s**，两段纯动态约 **1.165 s**，一段动态/静态混合 **1.963 s**（表内显示值有舍入）。8/9 段在失明期间真实执行了 SIDE；这直接反驳“只要 SIDE 被采用就会立即恢复 LOS”。但每段内的多次 trajectory ID 切换（8/9 段至少两条）只证明 rolling 覆盖发生，**并未证明某条被覆盖候选本来会在更早时刻持续恢复**。

针对黄框所示的“绕行没有产生实际可见性收益”，进一步按失明 UAV 的样本核对：261 个 blocked 样本中 **169 个执行源为 SIDE**（SIDE_MINUS 120、SIDE_PLUS 49），80 个 NOMINAL，12 个 FEASIBLE_FALLBACK。这证明 SIDE 标签对应的实际执行仍大量处在 LOS 阴影，不能把这段 K3 loss 简单归为“没有生成或提交 SIDE”；它也不是 SIDE 对同一时刻 NOMINAL 的反事实收益估计。

| 段 / blocker | Local K3 event 创建相对 loss 起点 | 同一 event 首条已提交 SIDE 的首次实际执行相对 loss 起点 | 执行轨迹相对 blocker 侧向轴线的穿越时刻 |
|---|---:|---:|---:|
| 1 / 静态 36 | 提前 0.369 s | **晚 0.164 s** | 8.432 s |
| 2 / 静态 26 | 提前 1.666 s | 提前 1.132 s | 11.180 s |
| 3 / 静态 28 | 提前 1.537 s | 提前 1.366 s | 25.833 s |
| 4 / 静态 6 | 提前 1.212 s | 提前 1.033 s | 35.411 s |
| 5 / 动态 5 | 提前 1.611 s | 提前 1.468 s | 57.951 s |
| 6 / 静态 13 | 提前 1.969 s | 提前 1.800 s | 64.511 s |
| 7 / 动态 10、静态 14 | 提前 1.986 s | 提前 1.469 s | 动态 10：71.925 s；静态 14：72.533 s |
| 8 / 动态 10 | **晚 0.755 s** | 本 event 无失明前 SIDE | 73.513 s |
| 9 / 静态 37 | 提前 0.220 s | 提前仅 0.066 s | 74.543 s |

因此，段 1、8、9 的 K3 SIDE 执行相对 loss 起点确实**迟到或仅领先 0.066 s**；对段 2–7，单纯“触发太晚”**不足以解释全部 loss**，因为该 K3 event 的 SIDE 已提前至少 1.0 s 真正执行。6 个纯静态段的执行轨迹均在各自失明区间穿过 blocker-relative 侧向轴线：段 1/2/3/6 从正侧到负侧，段 4/9 从负侧到正侧。这个几何事实说明 SIDE 路径在换侧过程中仍穿过遮挡阴影；`SIDE_PLUS/MINUS` 标签不代表整段轨迹已经在可见的观察侧。它不能单独证明另一侧在相同硬安全条件下更优。

### SIDE 实际绕到哪里：逐段执行与 seed 日志

以下 `s` 是**执行位置**在当时 target→blocker 框架中的 signed right lateral 坐标，`c_LOS` 是执行射线到相应圆柱减去半径与 0.08 m margin 的净空；负值表示进入其 LOS 阴影。两者都按场景与执行 CSV 重算，和 `visibility.csv` 的静/动态 LOS 标志在全部 2,348×3 样本上完全一致。`side_reached_time` 是日志中 **seed** 的相对时间，不是 final/执行轨迹的证明。`[los-plane-audit]` 没记 candidate ID；表中把它按同 UAV、同 SIDE、选择日志之前 80 ms 内的最近记录配到同轮选择。此关联足以检验该轮生成的 seed 是否及时到侧，尚不足以当作逐 candidate ID 的闭合证明。

| 段 | 实际 SIDE 执行路径与 LOS 结果 | 同轮 seed/选择日志 |
|---:|---|---|
| 1 / 树 36 | ID32 `SIDE_MINUS` 从 `s=+0.56` 走到 `−0.17`，`c_LOS` 最低 **−0.38 m**；它是在 8.069 s 失明后才开始执行。 | 8.065 s 选中 MINUS，`occlusion_enter=0`、`side_reached_time=1.331 s`；当轮尚不能立即到观察侧。 |
| 2 / 树 26 | ID34–37 `SIDE_PLUS` 在 9.40–10.23 s 的 `s≈+1.20→+1.01 m`，`c_LOS=+0.77→+0.13 m`，没有把射线推离逼近的阴影。10.269 s 起换 `SIDE_MINUS`，当时仍在 `s≈+1.02 m`；ID39/40 在失明期间向轴线走，`c_LOS` 降到 **−0.46 m**，随后 NOMINAL 到 **−0.61 m**。 | 10.380 s 选中 MINUS，seed `side_reached_time=2.165 s`，而 `occlusion_enter=0`；C3 日志为 `0.133→0.267`。SIDE 名称先于实际到侧。 |
| 3 / 树 28 | ID85–87 `SIDE_PLUS` 的 `s≈+1.32→+1.28 m`，`c_LOS=+0.74→+0.32 m`；ID88 改 MINUS 时仍 `s≈+1.28 m`。后来 ID90–93 `SIDE_MINUS` 在阴影中从 `s=+1.10` 穿到 `−0.37`，最低 **−0.63 m**。 | 已有提前 SIDE，但 K3 event 在 25.058 s 被 `M2_PREEMPTED`，实际 loss 25.199 s 开始；不能只归咎于 K3 comparator。 |
| 4 / 树 6 | ID122 `SIDE_MINUS` 位于 `s≈−1.0 m`，ID123/124 已换 `SIDE_PLUS` 却仍在 `s≈−1.02→−0.91 m`。失明后 ID126/127 PLUS 才穿到正侧，`c_LOS` 最低 **−0.49 m**。 | 34.835 s 选中 PLUS，seed 到该观察侧需 `1.643 s`，`occlusion_enter=0`；C3 `0.154→0.308`。 |
| 5 / 动态 5 | SIDE_PLUS 206–208 与 SIDE_MINUS 209/210 曾在失明前执行，但整段失明 57.768–58.132 s 执行的都是 **NOMINAL 211**，`c_LOS` 最低 **−0.32 m**，并有实际机体接触。 | 57.255 s 选中 MINUS 时 `side_reached_time=1.046 s`，遮挡进入仅 `0.500 s` 后；随后被 NOMINAL 接替。 |
| 6 / 树 13 | ID226–228 `SIDE_PLUS` 期间 `s≈+1.30→+1.28 m`，`c_LOS=+1.11→+0.70 m`；ID229 换 MINUS 时仍在 `s≈+1.28 m`。NOMINAL 将射线带入阴影，ID233–235 MINUS 在阴影中横穿，最低 **−0.49 m**。 | 64.017 s 选中 MINUS，seed 到侧需 `1.790 s`，`occlusion_enter=0`；C3 `0.182→0.273`。 |
| 7 / 动态 10 + 树 14 | ID255 `SIDE_MINUS` 在 69.40–70.57 s 从 `s=−1.34` 向轴线移到 `−0.71 m`，`c_LOS=+0.79→+0.10 m`。ID257–259 仍标 MINUS，却继续进入动态 10 阴影；ID260 PLUS 横过轴线，动态 LOS 最低 **−0.26 m**，随后树 14 又遮挡。 | 70.666/70.928/71.204 s 被选的 MINUS 同轮日志均 `LOS_REACHED_SIDE=0`、`LOS_PLANE_CREATED=1`、`occlusion_enter=0`。event 于 72.593 s 被 M2 抢占。 |
| 8 / 动态 10 | 失明前已有来源为 SIDE_MINUS 的 ID263；失明中它从 `s=−0.45` 穿到 `+0.12 m`，`c_LOS` 最低 **−0.25 m**，接着执行两个 FEASIBLE_FALLBACK，直到 74.033 s 才恢复。 | 对应 K3 event 4 在 loss 开始 **0.755 s 后**才创建，不能把 ID263 当作该 event 的及时修复。 |
| 9 / 树 37 | ID268 `SIDE_PLUS` 在 74.134 s 开始执行，失明中从 `s=−0.89` 穿到 `+0.38 m`，`c_LOS` 最低 **−0.35 m**；NOMINAL 269 接续。 | 73.987 s 选中 PLUS，seed `side_reached_time=1.525 s`、`occlusion_enter=0`；失明仅在 74.199 s 后就发生。 |

这不是“SIDE 已经在正确观察侧并维持了 LOS，仍被 visibility evaluator 错判”的证据。更早的可见事实是：**多个 selected SIDE 在所选观察侧尚未可达时就有执行 authority，且 LOS plane 允许 slack；实际轨迹沿阴影轴线横穿。** 源码在已知 occlusion window 时即使 `reached_side=false` 也建立 LOS plane；该 plane 在 SCP 里是带非负 slack 的软行。段 7 的 `LOS_REACHED_SIDE=0` 却 `LOS_PLANE_CREATED=1`，段 2/4/6/9 的 `side_reached_time>occlusion_enter`，是这一接口语义的直接 run 证据。是否存在另一条同等 hard-safe、K2 不降、能避开所有阴影的路径，仍需候选多项式做反事实验证。

执行几何进一步确认，这些并非 SIDE 标签与实际遮挡无关的统计重合：各段相关圆柱的线段净 LOS clearance 最小值依次约为 `−0.38/−0.61/−0.63/−0.49/−0.32/−0.49/−0.51(静态14)、−0.26(动态10)/−0.25/−0.35 m`。这些是执行射线相对圆柱半径加 0.08 m 的余量，**不是机体与障碍物的安全净空**；段 5 的实际动态接触另见下文。

## 候选、选择、执行：能追到哪里

现存 `[K3_LOCAL_PROGRESS]` 与 CSV 允许对照选择、commit 和 trajectory ID 执行。例如 U1/段 2 的 event 3 在 `t=8.867` 建立、在失明前选过 SIDE_PLUS，失明期间执行 ID 39/40/42 的 SIDE_MINUS，又执行 ID 41/43 的 NOMINAL，最终在 11.699 s 恢复。U3/段 7 的 event 6 在 `t=68.883` 建立；ID 257–260 在长 loss 中依次执行 SIDE_MINUS、SIDE_MINUS、SIDE_MINUS、SIDE_PLUS，然后 NOMINAL 261、SIDE_PLUS 262、NOMINAL 263、SIDE_MINUS 264；episode 7 首次结束 72.833 s，但 0.5 s 持续 K3 要到段 9 之后的 74.833 s。U2/段 8 的动态 LOS loss 从 73.233 s 已发生，相关 Local event 4 到 `t=73.988` 才创建，约晚 **0.755 s**；这个 episode 不能归咎于某个已提前 dispatch 的 K3 SIDE 候选。

这些事件也显示 selector 在本 run 中实际选中了多个 SIDE，且部分 SIDE 进入执行。但 `[K3_SIDE_CANDIDATES]` 的 C3/D3 是各次 rolling 的采样窗口汇总，不给候选的完整窗口轨迹或 blocker-relative 持续恢复时间；跨 rolling 的 C3/D3 也不是同一固定窗口。故当前证据**不能**构造以下逐候选比较：PLUS seed vs MINUS seed 哪侧更早恢复、seed→SCP final 是否保留收益、未选 hard-safe final 是否更早恢复、以及执行中的某条 SIDE 是否在其**预测**恢复时刻之前被 supersede。Feedback134 证明过旧 run 的 C3 反例和六条较早恢复 final 在预测恢复前被覆盖；这只是该机制存在的历史证据，不能据此给 Feedback138 的九段贴同样原因。

两个事件还有明确的 Team 权限边界：U1/段 3 的 K3 event 4 在 `t=25.058` 被 `M2_PREEMPTED` 关闭，实际 static-28 loss 到 `t=25.199` 才开始；U3/段 7 的 K3 event 6 在 `t=72.593` 也以 `M2_PREEMPTED` 关闭，当时混合 LOS loss 尚未结束。它们不能仅凭后来选中 NOMINAL 或换侧就归咎于 K3 comparator，须与 M2 抢占后的执行轨迹分开看。

## 因果排序与下一步

1. **已证的执行层瓶颈：SIDE 观察侧到达晚于遮挡，实际轨迹横穿 shadow。** 9/9 段有具体 LOS blocker；段 2/4/6/9 同轮选择旁的 seed reach time 晚于 occlusion enter，段 7 被选 MINUS 旁的 seed 甚至未达到所选侧，8/9 段的实际 loss 又同时有 SIDE 执行。段 2/3/4/6 的提前 SIDE 先维持近乎不变的 `s`，LOS 净空持续下降，随后换侧并穿过阴影。该机制比“已开始执行 1 秒”具体；但仍缺同版本候选多项式，不能判断最早是 seed 生成、优化、选择还是执行覆盖损失了另一条可行路径。
2. **已证的覆盖现象，原因未闭合：** 8/9 段在失明期间切换执行 ID；段 7 有 8 条 ID，最长 1.963 s，且后面短暂恢复不足 0.5 s。是否过早覆盖了本可更快恢复的轨迹，目前为 `UNKNOWN`，不能把常规 rolling 自动判为错误。
3. **未在本 run 证明的候选级假设：** 25% inward tolerance 使擦边 seed 的恢复延迟、SIDE seed 本身绕出慢、C3 错选。这三者在数学上可能，Feedback134 对旧代码有部分支持，但本 run 无未选/未执行 candidate polynomial 与对应 LOS world-time 样本，不具备相同反事实检验条件。

下一步若要证实“恢复效率”首因，应在**同一次未来生产运行**按 `(drone, revision, candidate_id, trajectory_id)` 记录两侧最终 timed seed 和 SCP final polynomial（或足够的固定世界时间 LOS samples）、raw blocker ID、完整共同窗口 `T_persistent_recovery`/`L_LOS`、SafetyKernel/preflight 和执行替换时刻；用同一个几何 evaluator 离线复算。不要仅凭 SIDE label、SCP pass 或 C3/D3 汇总改 topology band、selector、trust 或 slack 权重。此处仅提出测量方向，未实施修改或仿真。

**独立硬安全阻断：** Feedback138 的段 5 期间 UAV1 的 NOMINAL trajectory 211 与 moving obstacle 5 有 8 个实际接触样本，尽管 `safety_validated=True`。因此本 run 的硬安全验收仍为 **FAIL**；K3 漏斗结论不能掩盖这项优先需要定位的接触。

```text
RUN_ID: 20260927_155447_1343008
K3_LOSS_EPISODES: 9
K3_LOSS_TOTAL: 8.694550 s
EPISODES_WITH_EXECUTED_SIDE_DURING_LOSS: 8/9
EPISODES_WITH_MULTIPLE_EXECUTED_TRAJECTORY_IDS_DURING_LOSS: 8/9
BLOCKED_AFFECTED_UAV_SAMPLES_WHILE_EXECUTING_SIDE: 169/261
K3_EVENT_SIDE_EXECUTED_AT_LEAST_1S_BEFORE_LOSS: 6/9 episodes
K3_EVENT_FIRST_SIDE_PRE_LOSS_LEAD_LT_0P2_OR_NONE: episodes 1, 8, 9
STATIC_BLOCKER_AXIS_CROSSED_DURING_LOSS: 6/6 pure-static episodes
SELECTED_SIDE_SEED_REACH_AFTER_OCCLUSION_ENTER: same-cycle log evidence in episodes 1, 2, 4, 5, 6, 9; audit log lacks candidate ID
SELECTED_SIDE_SEED_NOT_REACHED_WHILE_LOS_PLANE_CREATED: same-cycle log evidence in episode 7; audit log lacks candidate ID
SIDE_SCP_FINAL_OK: 663/689
SUCCESSFUL_SIDE_SCP_WITH_NONZERO_LOS_SLACK: 109/663
SEED_VS_FINAL_T_PERSISTENT_RECOVERY: UNKNOWN (candidate polynomial not retained)
SEED_VS_FINAL_COMPLETE_WINDOW_LOS_LOSS: UNKNOWN (candidate polynomial not retained)
BETTER_HARD_SAFE_FINAL_MISSELECTED_IN_THIS_RUN: UNKNOWN
BETTER_FINAL_SUPERSEDED_BEFORE_PREDICTED_RECOVERY_IN_THIS_RUN: UNKNOWN
TOPOLOGY_IMPLIES_FAST_LOS_RECOVERY: NO
FIRST_OBSERVED_EXECUTION_BOTTLENECK: selected SIDE not at visibility observation side before occlusion; actual path crosses blocker shadow
FIRST_CANDIDATE_LEVEL_CAUSAL_DIVERGENCE: UNDETERMINED
HARD_SAFETY_ACCEPTANCE: FAIL (UAV1 NOMINAL trajectory 211 vs moving obstacle 5)
PRODUCTION_CODE_CHANGED: NO
BUILD_RUN: NO
SIMULATION_RUN: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```

原始证据：[Feedback138](feedback_138.md)、[visibility.csv](../runs/20260927_155447_1343008/visibility.csv)、[visibility_trajectory.csv](../runs/20260927_155447_1343008/visibility_trajectory.csv)、[压缩 ROS stdout](../runs/20260927_155447_1343008/roslaunch_stdout.log.gz)；旧版候选级边界见 [Feedback134](feedback_134.md)。
