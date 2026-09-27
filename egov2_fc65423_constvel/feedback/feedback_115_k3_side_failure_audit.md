# Feedback 115 — K3 Local SIDE 绕行失败只读审计

日期：2026-09-25。工作区：`/home/bob/ALP/egov2_fc65423_constvel`。本轮只读当前源码和 Feedback111–114、既有 FULL run 的 ROS 日志、visibility CSV、执行轨迹 CSV；没有修改 production、编译或启动仿真。审计对象是 [Feedback113](feedback_113_k3_local_recovery_final.md) 的 `20260925_190928_365841` 与 [Feedback114](feedback_114_k3_recovery_observation.md) 的 `20260925_200523_419199`。两个 run 的事件数分别为 17、25；`SIDE_BOTH_FAILED` 分别记录 49、59 次，涉及 9、18 个 run 内事件。run 之间同名 event ID 属于不同实例。Feedback114 的 terminal hold/75 条未验证执行是已记录的 successor coverage 回退，本报告不把它归因于 SIDE，也不据此推断新的安全结果。

## 结论与证据边界

**首个实际候选瓶颈在 C3/D3 比较之前。** 108 条 `SIDE_BOTH_FAILED` 中，56 条至少有一侧已产生 `MINCO_OK`，只是构造阶段 `acceptance_accepted` 在 `safety_class` 分类前读取了默认 `INVALID`；其中 36 条对应的后续汇总至少有一侧通过当前 revision 的 hard preflight 和 K2 门。这个计数器不能当作 108 次双侧几何失败。剩余 52 条确实双侧未通过构造：24 条为双侧 `LOS_PLANE_SCP_FAILED`，17 条为双侧静态前端失败，11 条为静态、LOS 平面或未细分 MINCO 的混合失败。首次可归因的功能瓶颈是 **已知遮挡区间的 observation-side 平面在当前阴影内立即生效，SIDE 的固定 PVA 多项式在该平面 QP 内不可行**；其次是局部侧向种子触静态树后，现有 A* raw path 虽安全，重建的平滑 MINCO seed 又切角碰撞。不存在证据证明 C3/D3 排序、简单扩大 trust 或左右翻侧是主因。

分类来自每条失败记录前同一 ROS 节点的 `[los-plane-audit]`、`[side-minco]`、`[candidate-final-status]`、`[candidate-preinit-failure]` 和后续 `[K3_SIDE_CANDIDATES]`。逐条 108 对 PLUS/MINUS 的第一终止原因、原始日志字段见 [逐对证据 CSV](artifacts/feedback115_side_pair_first_causes.csv) 与 [逐对 JSON](artifacts/feedback115_side_pair_audit.json)。跨候选构造、final preflight 和执行的同一候选 ID 并非每条日志都完整记录；报告不把推断写成已证实的 candidate-level 结果。

## 1. K3 → N/L/R 时间链

源码顺序是：`reboundReplan()` 先优化 NOMINAL 并在同轮扫描 raw static/dynamic LOS；`task_topology_trigger = body_topology_trigger || spatial_visibility_trigger` 后生成 N/L/R、SIDE seed、LOS observation plane、必要时 A* 与 Local-SFC，再经 MINCO 和构造分类捕获候选；`finalizeCapturedCandidates()` **随后**调用 `updateLocalK3RecoveryEvent()` 创建/刷新 Local event，做 current-revision hard preflight、同窗 K2→C3→D3 比较，最后提交和等待激活确认。源码锚点：[planner_manager.cpp 8416、8930、9245、9634、9734、10040、10742、11894、12881、12977、13174、1461–1905、3274–3575](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp)，[poly_traj_optimizer.cpp 225–400、5072](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/src/poly_traj_optimizer.cpp)。同一批次中的候选可在 event 创建之前已生成，所以下表的阶段不强制单调；表中 `F` 专指**首次 PLUS 与 MINUS 尝试之后**的 `[K3_SIDE_CANDIDATES]`，`S/C/A` 是此后首个 progress selection / commit / 同轨迹 ID activation。完整世界时间、初始窗口、关闭原因及原始首轮汇总时间见 [42 事件时间 CSV](artifacts/feedback115_event_timeline_enriched.csv)。

表内 `E` 为本 run 首个 Local K3 event 之后的创建时刻；其他数字均为相对各自 event 创建的秒数。`cross` 是**预测轨迹上未来的 margin 样本**，不是已经发生的物理 crossing；`raw` 是 event 后首条 raw LOS 日志，`grant` 是首条有节流的授权日志；`P/M` 为首次 PLUS/MINUS 构造；`loss/recover` 是 visibility CSV 在 event 存续期内首次 blocked 与随后首次 visible。`—` 表示在该定义下未观测到，不能直接译为功能失败。跨窗口续期未逐轮记录 end，所以 lifetime 的 recovery 与 checker 窗口不完全同口径。

| run | UAV/e | limiter | E(s) | cross | predicted loss | raw | grant | P/M | F | S/C/A | executed loss/recover | failed |
|:--|:--|:--|--:|--:|--:|--:|--:|:--|--:|:--|:--|--:|
| 113 | 1/1 | STA | 0.00 | +1.55 | +1.75 | +0.29 | — | — | — | — | — | 0 |
| 113 | 1/2 | STA | 1.37 | +0.18 | +0.28 | +0.54 | +0.54 | +0.54/+0.54 | +0.54 | +0.83/+0.83/+1.07 | +0.30/+1.03 | 2 |
| 113 | 1/3 | STA | 2.55 | +1.21 | +1.31 | +0.15 | +0.44 | +0.15/+0.15 | +0.15 | +0.20/+0.20/+0.44 | — | 4 |
| 113 | 1/4 | STA | 4.09 | +0.07 | +0.27 | +0.25 | +0.25 | +0.25/+0.25 | +0.25 | +1.34/+1.34/+1.62 | +0.25/+1.48 | 4 |
| 113 | 1/5 | STA | 16.79 | +1.57 | +1.67 | +0.21 | +0.21 | +0.21/+0.28 | +0.34 | +0.34/+0.34/+0.51 | +1.92/— | 7 |
| 113 | 2/1 | STA | 18.10 | +1.76 | — | — | — | — | — | — | — | 0 |
| 113 | 3/1 | STA | 18.28 | +1.58 | — | — | — | — | — | — | — | 0 |
| 113 | 1/6 | STA | 19.79 | +0.07 | +0.07 | — | — | — | — | — | +0.02/— | 0 |
| 113 | 3/2 | FOV | 44.78 | +1.17 | +1.37 | +0.14 | +0.14 | +0.15/+0.15 | — | — | — | 0 |
| 113 | 1/7 | STA | 55.35 | +1.70 | — | +0.20 | +0.20 | +0.20/+0.20 | — | — | — | 1 |
| 113 | 1/8 | STA | 55.89 | +1.46 | +1.56 | +0.20 | +0.77 | +0.20/+0.20 | +0.21 | +0.21/+0.21/+0.49 | — | 4 |
| 113 | 1/9 | STA | 57.49 | +0.06 | +0.06 | +0.29 | +0.29 | +0.29/+0.29 | +0.29 | +0.55/+0.55/+0.83 | +0.08/+1.18 | 4 |
| 113 | 3/3 | DYN | 62.59 | +1.67 | — | +0.01 | +0.02 | +0.09/+0.02 | +0.10 | +1.03/+1.03/+1.29 | +1.65/+2.92 | 17 |
| 113 | 3/4 | STA | 67.48 | +0.07 | +0.07 | — | — | — | — | — | +0.02/+0.19 | 0 |
| 113 | 2/2 | DYN | 67.88 | +0.08 | +0.08 | +0.13 | +0.13 | +0.21/+0.21 | +0.21 | +0.51/+0.51/+0.77 | +0.03/+0.73 | 6 |
| 113 | 2/3 | DYN | 68.64 | +0.01 | — | — | — | — | — | — | — | 0 |
| 113 | 2/4 | RNG | 72.24 | +0.72 | +0.82 | — | — | — | — | — | — | 0 |
| 114 | 1/1 | STA | 0.00 | +1.67 | +1.77 | +0.28 | +0.28 | +0.28/+0.28 | +0.29 | +0.29/+0.29/+0.57 | — | 2 |
| 114 | 2/1 | STA | 0.34 | +1.73 | — | +0.22 | +0.22 | +0.22/+0.23 | +0.24 | — | — | 1 |
| 114 | 3/1 | STA | 0.58 | +1.39 | +1.49 | — | — | — | — | — | — | 0 |
| 114 | 1/2 | STA | 1.43 | +0.14 | +0.24 | +0.28 | +0.28 | +0.28/+0.28 | +0.29 | — | +0.21/+1.08 | 3 |
| 114 | 1/3 | STA | 2.83 | +1.24 | +1.24 | +0.11 | +0.11 | +0.11/+0.11 | +0.12 | +0.22/+0.22/+0.33 | — | 4 |
| 114 | 1/4 | STA | 4.33 | +0.04 | +0.14 | +0.16 | +0.16 | +0.16/+0.17 | +0.17 | +0.17/+0.17/+0.46 | +0.18/+1.38 | 4 |
| 114 | 1/5 | STA | 16.96 | +1.50 | +1.60 | +0.11 | +0.11 | +0.11/+0.28 | +0.37 | +0.37/+0.38/+0.48 | — | 4 |
| 114 | 1/6 | STA | 19.18 | +0.09 | +0.09 | +0.11 | +0.11 | +0.11/+0.11 | +0.12 | — | +0.02/+0.79 | 3 |
| 114 | 1/7 | FOV | 25.56 | +0.01 | +0.01 | — | — | +0.03/+0.03 | +0.04 | — | +0.03/+0.16 | 1 |
| 114 | 3/2 | FOV | 26.08 | +0.79 | +0.89 | — | — | — | — | — | — | 0 |
| 114 | 3/3 | FOV | 26.85 | +0.71 | — | — | — | — | — | — | — | 0 |
| 114 | 3/4 | FOV | 27.33 | +0.44 | — | — | — | — | — | — | +0.24/— | 0 |
| 114 | 1/8 | DYN | 33.20 | +0.07 | +0.17 | +0.25 | +0.25 | +0.25/+0.25 | +0.25 | — | +0.14/— | 2 |
| 114 | 1/9 | FOV | 35.45 | +1.82 | — | — | — | +0.14/+0.20 | — | — | — | 0 |
| 114 | 3/5 | FOV | 44.48 | +1.09 | +1.19 | +0.15 | +0.15 | +0.15/+0.16 | — | — | — | 1 |
| 114 | 1/10 | STA | 55.71 | +1.36 | — | +0.11 | +0.11 | +0.21/+0.11 | — | — | — | 1 |
| 114 | 1/11 | STA | 56.01 | +1.66 | +1.76 | +0.11 | +0.84 | +0.12/+0.11 | +0.14 | +0.20/+0.20/+0.31 | +1.71/+2.73 | 12 |
| 114 | 1/12 | FOV | 59.34 | +0.83 | +0.93 | — | — | +0.13/+0.12 | — | — | — | 1 |
| 114 | 1/13 | FOV | 59.64 | +1.43 | — | — | — | — | — | — | — | 0 |
| 114 | 3/6 | DYN | 61.85 | +1.82 | — | +0.28 | +0.28 | +0.29/+0.28 | +0.31 | +0.31/+0.31/+0.54 | — | 1 |
| 114 | 3/7 | DYN | 62.83 | +1.34 | +1.54 | +0.02 | +0.43 | +0.16/+0.02 | +0.16 | +0.22/+0.22/+0.43 | +1.51/— | 7 |
| 114 | 3/8 | DYN | 64.92 | +0.05 | +0.05 | +0.24 | +0.24 | +0.25/+0.24 | +0.26 | — | +0.02/— | 1 |
| 114 | 2/2 | DYN | 68.12 | +0.04 | +0.04 | +0.01 | +0.01 | +0.01/+0.01 | +0.02 | +0.10/+0.10/+0.25 | +0.03/+0.35 | 10 |
| 114 | 2/3 | DYN | 68.51 | +0.05 | — | — | — | +0.02/+0.02 | +0.02 | — | — | 1 |
| 114 | 2/4 | RNG | 72.10 | +0.77 | +0.87 | — | — | — | — | — | — | 0 |

**A：没有明显的 margin→event 创建迟滞。** 42 个事件的 `event_created − predicted_margin_cross` P50 为 **−0.962 s**（113：−1.172 s；114：−0.833 s），全部为负：Local 事件通常先于其预测 crossing 建立。这里不能用负值推断算法比真实 crossing 提前 0.962 s，因为 crossing 取自当时已执行基线的未来预测，随后 rolling 会重预测。

**B/D：raw witness gate 是授权条件，但 event 创建和 raw 扫描的阶段顺序损失了一次 rolling。** 连续 Local sample 可在 raw LOS 尚未达到 `clearance≤0.08 m` 时因平滑风险触发 `margin<0.20`；raw 扫描取 NOMINAL 轨迹上首次满足静态净空阈值或动态圆柱相交的样本，普通授权地平线为非 touch-goal 标称时长的 `2/3` 再截到 moving horizon，sample 步长约 0.10 s。K3 grant 还要求该快照的 raw witness world time 与 `[event.window_begin−0.5,event.window_end+0.5]` 相交，且 M2 未抢占；FOV/RANGE margin 不能独立制造 LOS descriptor。连续风险与 raw blocker 因阈值、所用轨迹和阶段不同，可能不同步；raw 和 grant 在同一次被记录的扫描中几乎同步。源码：[tracking_visibility_geometry.h 45–105、199–229](../ros_ws/src/multi_uav_formation/include/multi_uav_formation/tracking_visibility_geometry.h)，[planner_manager.cpp 7540–7710、3290–3425、8930–9040、9245–9325](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp)。

在有授权关联 raw 的 25 个事件中，`event→raw` P50 为 **0.220 s**；首条任意 raw 日志 P50 为 0.182 s（26 个事件）。同一授权扫描里最近 raw 日志→grant 日志 P50 **0.038 ms**（25 对）；首次 raw→首次 grant 的 P95 会达约 0.41 s，原因是 grant 用 `ROS_INFO_THROTTLE(1.0)`，不等于真实 dispatch 延迟。`side_trigger_count` 在两 run 为 51/56，而 grant 日志只有 29 条，因此无法恢复所有真实授权时刻。首条**有记录的** grant 到实际首次 blocked 的 P50 为 **0.025 s**（18 个可配对事件），其中 7/18 条日志已在该 CSV 的首次 blocked 之后；这个比值也受节流和不同采样流影响，不能当作全部 dispatch 的真实分布。

两个近端实证：113 UAV1/e4 在 `1790334585.034` 创建，预测 crossing `4585.100`、预测 loss `4585.300`；下一次 raw/grant 是创建后 `0.246 s`，SIDE 随即构造，实际 CSV loss 在 `+0.253 s`。114 UAV1/e4 在 `1790337941.162` 创建，预测 crossing `+0.038 s`、预测 loss `+0.138 s`；raw/grant `+0.164 s`、SIDE `+0.164/+0.168 s`、实际 loss `+0.177 s`。所以这两个 event 的**可观测**可执行 SIDE 时间确实仅余约 7–13 ms；它们随后又被 LOS 平面 QP 拒绝。113 UAV1/e8 则在 `+0.202 s` 已生成 SIDE，首条 grant 却到 `+0.767 s` 才出现，正好证明节流 grant 日志不能倒推出实际授予时刻。

**C：Feedback112 离线 2–4 s 与本次 production `lead_to_binary_loss` 不可直接比较。** [Feedback112 §2](feedback_112.md) 的 `39/40`、P50 `2.13–2.47 s` 是五个**历史执行轨迹**的 4 s 回看窗口内、2D 静态 LOS replay 对 binary loss 事件的可检出 lead，P90 受 4 s 截断；其逐事件 margin 中间样本未保存。[Feedback113](feedback_113_k3_local_recovery_final.md) 的正 lead 8/17、P50 0.10 s 和本次 event 日志的 `lead_to_binary_loss` 是**当前预测基线内首个 margin<0.20 样本到预测 binary loss 样本**，约 0.10 s 量化，含 FOV/RANGE、移动 LOS 和 rolling 重新锚定。事件创建本身常在预测 crossing 之前。不能把两者相减，声称 2–4 s 被某一段 planner 消耗。

## 2. SIDE_BOTH_FAILED 的逐侧第一终止原因

下表 `T/L/S/M` 顺序为：`T`=构造阶段 **telemetry stale**，SIDE 实际 `MINCO_OK`，`acceptance_accepted=0` 读到分类前的默认 `INVALID`；`L`=`LOS_PLANE_SCP_FAILED`，归入 SFC/LOS 平面不可行；`S`=`NO_STATIC_FEASIBLE_SIDE` 或 `SIDE_INIT_STATIC_COLLISION`，归入静态前端不可行；`M`=`side-minco success=0` 且 status/final reason 为空，归入未细分 MINCO 失败。`blocker id` 取该失败对的 `[los-plane-audit]`；空缺不是无障碍。每行是 run 内一个 event，逐对日志见上方 CSV。

| run | UAV/e | limiter | pairs | blocker id (logged) | PLUS T/L/S/M | MINUS T/L/S/M | close |
|:--|:--|:--|--:|:--|:--|:--|:--|
| 113 | 1/2 | STATIC | 2 | 36 | 0/2/0/0 | 1/1/0/0 | RECOVERY_OBSERVED |
| 113 | 1/3 | STATIC | 4 | 26 | 4/0/0/0 | 2/0/2/0 | M2_PREEMPTED |
| 113 | 1/4 | STATIC | 4 | 26 | 0/4/0/0 | 0/4/0/0 | EVENT_EXPIRED |
| 113 | 1/5 | STATIC | 7 | 28 | 6/1/0/0 | 4/1/1/1 | M2_PREEMPTED |
| 113 | 1/7 | STATIC | 1 | 13 | 1/0/0/0 | 0/0/1/0 | MARGIN_RECOVERED |
| 113 | 1/8 | STATIC | 4 | 13 | 3/1/0/0 | 4/0/0/0 | RECOVERY_OBSERVED |
| 113 | 1/9 | STATIC | 4 | 13 | 1/3/0/0 | 1/2/0/1 | RECOVERY_OBSERVED |
| 113 | 2/2 | DYNAMIC_LOS | 6 | — | 0/0/6/0 | 0/0/6/0 | RECOVERY_OBSERVED |
| 113 | 3/3 | DYNAMIC_LOS | 17 | 10,14 | 2/3/4/8 | 9/3/0/5 | M2_PREEMPTED |
| 114 | 1/1 | STATIC | 2 | 36 | 2/0/0/0 | 2/0/0/0 | M2_PREEMPTED |
| 114 | 1/2 | STATIC | 3 | 36 | 0/3/0/0 | 1/2/0/0 | RECOVERY_OBSERVED |
| 114 | 1/3 | STATIC | 4 | 26 | 4/0/0/0 | 3/0/1/0 | M2_PREEMPTED |
| 114 | 1/4 | STATIC | 4 | 26 | 0/4/0/0 | 0/4/0/0 | RECOVERY_OBSERVED |
| 114 | 1/5 | STATIC | 4 | 28 | 4/0/0/0 | 4/0/0/0 | M2_PREEMPTED |
| 114 | 1/6 | STATIC | 3 | 28 | 0/3/0/0 | 0/3/0/0 | RECOVERY_OBSERVED |
| 114 | 1/7 | FOV | 1 | — | 0/0/1/0 | 0/0/1/0 | RECOVERY_OBSERVED |
| 114 | 1/8 | DYNAMIC_LOS | 2 | 4 | 0/2/0/0 | 0/2/0/0 | RECOVERY_OBSERVED |
| 114 | 1/10 | STATIC | 1 | 13 | 1/0/0/0 | 0/0/0/1 | MARGIN_RECOVERED |
| 114 | 1/11 | STATIC | 12 | 13 | 6/5/0/1 | 8/4/0/0 | RECOVERY_OBSERVED |
| 114 | 1/12 | FOV | 1 | — | 1/0/0/0 | 1/0/0/0 | MARGIN_RECOVERED |
| 114 | 2/1 | STATIC | 1 | 36 | 1/0/0/0 | 0/1/0/0 | M2_PREEMPTED |
| 114 | 2/2 | DYNAMIC_LOS | 10 | — | 0/0/10/0 | 0/0/10/0 | RECOVERY_OBSERVED |
| 114 | 2/3 | DYNAMIC_LOS | 1 | — | 0/0/1/0 | 1/0/0/0 | RECOVERY_OBSERVED |
| 114 | 3/5 | FOV | 1 | 10 | 1/0/0/0 | 0/0/1/0 | MARGIN_RECOVERED |
| 114 | 3/6 | DYNAMIC_LOS | 1 | 10 | 1/0/0/0 | 1/0/0/0 | MARGIN_RECOVERED |
| 114 | 3/7 | DYNAMIC_LOS | 7 | 10 | 2/2/2/1 | 5/1/0/1 | M2_PREEMPTED |
| 114 | 3/8 | DYNAMIC_LOS | 1 | 14 | 0/0/0/1 | 0/1/0/0 | M2_PREEMPTED |

合计：PLUS `T40/L33/S24/M11`，MINUS `T47/L29/S23/M9`。按 108 对互斥分类：`≥1 T` **56**；真正两侧终止 **52**，其中 `L/L 24`、`S/S 17`、混合 `S/M 4、M/L 4、L/M 3`。113 的 49 对中 `T 26 / 双侧真失败 23`；114 的 59 对中 `T 30 / 双侧真失败 29`。在 56 条 `T` 对里，51 条可在 event 内匹配下一条最终候选汇总；其中 **36 条已有至少一侧 current-revision preflight+K2 有效 SIDE**，15 条仍无有效 SIDE；另 5 条 event 在能匹配汇总前关闭。后 20 条无法仅凭当前日志区分后续 hard preflight、窗口无效、候选被 deadline 跳过或其他原因。

请求的失败分类按**108 条已发生双侧尝试的 fallback**限定口径如下；没有 SIDE 尝试的 13 个 event 另见时间表（其中 12 个在 event 存续期无后续 raw LOS 日志，另 1 个有 raw 但被 M2 抢占），不能混入每侧失败分母。

| 分类 | PLUS / MINUS 第一终止次数 | 证据解释 |
|:--|:--|:--|
| `NO_RAW_WITNESS`、`NO_DISPATCH` | 0 / 0 | 108 对均已进入双侧尝试；无 SIDE 的 event 不属于此分母。 |
| `SIDE_SEED_INVALID` | 0 / 0 独立记载 | seed 的静态不可行计入下项；没有独立 `INVALID_SIDE_INIT` 首次终止记录。 |
| `RECOVERY_NOT_REACHED`、`WINDOW_LIMITED` | 0 / 0 直接终止 | 分别有 17/12 条 LOS-plane audit 状态，但不是 candidate return。 |
| `A_STAR_FAILED` | 无法从 47 次静态终止里独立拆出 | A* 搜索、handoff 和多项式重建在静态前端内；有 raw path 成功但平滑 seed 失败的实证。 |
| `STATIC_INFEASIBLE` | 24 / 23 | `NO_STATIC_FEASIBLE_SIDE` 或 `SIDE_INIT_STATIC_COLLISION`。 |
| `DYNAMIC_INFEASIBLE` | 0 / 0 可证的第一终止 | 不能据此断言后续动态门从未拒绝。 |
| `SFC_INFEASIBLE` | 33 / 29 | 终止码明确为 `LOS_PLANE_SCP_FAILED`。 |
| `MINCO_FAILED` | 11 / 9 | 空 status，无法再判定是 SFC、动力学还是预算造成。 |
| `DYNAMICS_FAILED`、`SWARM_FAILED` | 0 / 0 可证的第一终止 | 2 个 `M` 初值有 `dynamics_ok=0`，但终止仍是未细分 MINCO；后续门无法按 kind 完整归属。 |
| `HARD_PREFLIGHT_FAILED` | 无法精确计数 | 同 candidate 的构造 kind 与 rejection ID 没有完整可连接日志。 |
| `CANDIDATE_VALID_BUT_NOT_SELECTED` | 无法精确计数 | 36 条 `T` 对的后续汇总有有效 SIDE，但没有逐候选 ID 对应的最终未选原因。 |
| `OTHER` | 40 / 47 | `T` 是分类前状态滞后的计数误报，并非实际 SIDE 终止。 |

`T` 的源码机制是 `CandidateResult.safety_class` 默认 `INVALID`，`optimize_side` 在调用 `classify_candidate(plus/minus)` **之前**计算 `acceptance_accepted`；随后 `plus_good/minus_good` 又要求这个旧值，便增加 `side_both_failed_count`。但 `capture_output` 仍收集有轨迹的 SIDE，并在 `finalizeCapturedCandidates` 做真正 hard preflight。113 UAV1/e8 的一轮是直接反例：`SIDE_PLUS final_success=1 reason=MINCO_OK` 同批 `plus_good=0`，`[K3_SIDE_CANDIDATES] PLUS success=1` 且以 `D3_LOWER` 选中 SIDE_PLUS。[planner_manager.h 223](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/include/plan_manage/planner_manager.h)、[planner_manager.cpp 12585–12610、12752–12760、12881、12955–12985、13170–13205、1560–1700](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp)。这属于误导性遥测，不是自动把所有 SIDE 从最终 comparator 删掉。

`L` 的 solver 证据更明确。113 UAV1/e4 的四轮 PLUS/MINUS 均为 `LOS_PLANE_SCP_FAILED`，其 8 条 `[los-plane-scp]` 均 `qp_status=primal_infeasible`；PLUS 最大 plane violation 从 2.914 增至 3.914 m，MINUS 从 1.336 降至 0.349 m，后者即使小于当轮 trust 数值仍报告不可行。114 同一 UAV/event/blocker26 的四轮再次有 **8/8** `primal_infeasible`，没有一条有效 SIDE；114 的 checker 最终正确锁存 NOMINAL 执行恢复。这证明当轮约束 QP 无解，**不能证明单独扩大 trust 会生成 hard-safe 解**。QP 固定首尾 PVA、按平面取 25 点、内点 P trust 上界 1.5 m、最多 6 次；没有逐行冲突集日志，现有证据无法区分“固定端点/行组合矛盾”和“盒约束不足”的精确贡献。[poly_traj_optimizer.cpp 225–400](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/src/poly_traj_optimizer.cpp)。

`M` 的 20 次没有 solver 终止码：18 次初值 `local_sfc_ok=0`，2 次 `dynamics_ok=0`，但源码仍让它们进 MINCO，最后只记空 status 的 `success=0`。因此只能定为 `MINCO_FAILED_UNSPECIFIED`；现有证据无法再区分 SFC 梯度失败、迭代预算、动力学与其他 solver 原因。两侧失败对中未发现动态、swarm 或 hard preflight 作为**首次**终止原因；这不等于这些门在后续从未拒绝候选。`[moving-candidate-reject]` 有 candidate ID 却没有 kind；构造日志有 kind 却未完整带 ID，不能严谨地给 SIDE 的 downstream hard-preflight-failed 总数。

## 3. D3 渐进改善与 SIDE admission

**没有找到“`RECOVERY_NOT_REACHED` / `WINDOW_LIMITED` 直接杀掉 SIDE，然后不准进 C3/D3”的门。** 这两个状态只在 LOS 遮挡区间未知且 seed 未按时到侧时记 `LOS_PLANE_CREATED=0`；代码随后继续静态/动态检查、MINCO、capture。已知区间则即使 seed `LOS_REACHED_SIDE=0` 也创建覆盖真实 `[occlusion_enter,occlusion_exit]` 的 observation plane；当前已经遮挡时从相对时间 0 生效，不因初值未到侧而延后。[planner_manager.cpp 10078–10140、10490–10672](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp)。两 run event 内有 `RECOVERY_NOT_REACHED 17`、`WINDOW_LIMITED 12` 条（仅失败对内为 17/11），它们均不是自身的第一拒绝点。113 UAV1/e8 有 `LOS_PLANE_CREATED=0 status=RECOVERY_NOT_REACHED` 的 SIDE_MINUS，仍 `final_success=1 reason=MINCO_OK` 并参与后续比较，直接证伪该假设。

真正的前置强制是**已知遮挡区间的硬 LOS plane**。113/114 UAV1/e4 的 blocker26 在候选中记录 `LOS_OCCLUSION_ENTER_REL=0`、`LOS_PLANE_ACTIVE_START=0`，SIDE 起点仍在阴影侧；QP 在 C3/D3 计算前失败，候选没有可比较的 hard-safe 轨迹。这是“当前 cycle 尚未完全绕出，但可能逐轮 D3 改善”的候选无法进入比较的**具体路径**，机制是 QP infeasible，并非 `RECOVERY_NOT_REACHED` 文本门。当前日志也没有这些失败轨迹的同窗 C3/D3、K2 与最终 hard safety 值，因而**尚不能证明**放行某条 partial SIDE 会改善 D3 且满足 K2；不能直接绕过 QP 或把 partial progress 报成 recovery。保留 `required_margin=0.20` 与 recovery 的 executed binary 验证边界。

## 4. comparator 与执行漏斗

157 个 `[K3_SIDE_CANDIDATES]` batch（113：72；114：85）中，有 **96 个**通过最终 hard preflight 且窗口指标有效、K2 admissible 的 SIDE candidate（113：38；114：58）。按每个 SIDE kind×batch 的汇总和三位小数保守判断，至少 **49 个** kind×batch 出现相对执行基线更高 C3 或更低 D3（113：19；114：30）；这只是可证的候选数**下界**，因为同 kind 可有多个候选，而汇总的 C3 最大和 D3 最小可能属于不同 ID。真实被选中的严格 progress SIDE 为 **32 次**（113：13；114：19，涉及 6+10 个事件），提交 31 次，确认激活 26 次。不存在“更好的 SIDE 已有 hard-valid 候选而最终被 NOMINAL 错排”的可证实例；`K3_BETTER_SIDE_COMPARATOR_REJECTED=0 confirmed`，不是已穷尽所有未选候选的数学证明。比较源码确为 hard preflight→K2 不降→C3 高→D3 低→全平时既有 side preference。[planner_manager.cpp 1590–1610、1660–1800、1862–1948](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp)。

按事件的**阶段漏斗**而非互斥故障计：16/42 个事件没有任何最终有效 SIDE（113：6；114：10）；10/42 个事件有有效 SIDE 但无严格 progress SIDE 被选，其中 9 个没有比执行基线更好的 SIDE 汇总，另 1 个有效 SIDE 虽比基线某项好，NOMINAL 在同批比较仍不劣。没有事件止于“SIDE 被选却全部未提交”；3 个事件有 SIDE commit 但无 SIDE activation，均以 `M2_PREEMPTED` 关闭。13 个事件至少激活过一条 SIDE：2 个 `RECOVERY_OBSERVED`，5 个随后 M2 抢占，6 个以 `MARGIN_RECOVERED` 关闭；后 6 个没有 checker binary recovery 日志，但也不是 `EVENT_EXPIRED`。Feedback113 首个 `EVENT_EXPIRED`（UAV1/e4）在双侧 QP 失败后只激活了 NOMINAL；其执行恢复漏记已由 Feedback114 修正。单个 event 可跨多轮占据不同阶段，因此不能强行把全部 42 个事件唯一贴到 A–F 六类。

## 5. 单 blocker SIDE、A* 与自然森林

blocker 由 NOMINAL 的 raw STATIC/DYNAMIC_LOS 及 BODY conflict descriptor 选取；LOS observation normal 来自 `target→该 blocker` 的切线，PLUS/MINUS 各生成一个侧向半空间。它不会对一串树逐个选边；A* 只在侧向初值经偏移 backoff 后仍有静态碰撞且保留可用 A* base 时触发。若是 body-safe 的 LOS-only 风险、side seed 也静态安全，则可完全不进 A*；Local-SFC 对已有 A*/guide 和 LOS 平面作约束，不主动搜索第二条多障碍可见路径。进入 A* 时 raw path 可以有多转弯（实际 12–15 点），因此系统**有**“先绕 A 再绕 B”的路径表示能力，但未在所有 LOS-only 场景主动调用它。[planner_manager.cpp 9570–9650、10040–10140、10520–10670、10695–10770、10870–11450、11890–12040](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp)。

已证的双障碍交互是 114 UAV2/e2：同一 raw 预测里 `dynamic_id=10, dynamic_time=0`，随后 `static_time=2.0`、descriptor `primary_id=37, los_id=10`；N/L/R 针对 moving gate 10，PLUS/MINUS 偏移 backoff 遇静态碰撞。A* 输出 raw `path_static_free=1`、Local-SFC `handoff_allowed=1`、1–8 个 plane，但重建的平滑多项式 `rebuilt_static_free=0`，最终 `SIDE_INIT_STATIC_COLLISION`，该 event 的 10 对失败均双侧 `S/S`。113 同类 UAV2/e2 有 6 对双侧 `S/S`，raw path 安全、平滑 seed 静态碰撞。114 日志证明 gate10 与静态 LOS blocker37 同一 horizon 出现，也证明 seed 的静态失败；**碰撞检查未记具体静态树 ID**，所以不能把每次 polynomial collision 直接归因于树37。地图中树37位于 `(32.842,1.067)`，与该段约 `(31–32,0.3–0.6)` 的 A* anchor 接近，只能作为空间一致性旁证。当前 A* 已经接通；“只需把 SIDE 接上 A*”与运行事实不符。更可证的待改位置是已有 A* raw path→guide/Local-SFC→平滑 seed 的静态安全保持。

## 6. 跨 rolling side consistency

同 blocker 的选中 SIDE 标签转向共 **7 次**：113 UAV1/e8、blocker13 为 `MINUS→PLUS→MINUS→PLUS→MINUS`（4 次）；114 UAV1/e3、blocker26 有 1 次；114 UAV1/e11、blocker13 有 2 次（忽略夹着的 NOMINAL）。但 113 e8 五轮的执行基线 D3 为 `1.045→1.643→2.757→3.627→3.975`，它们的比较窗口随 rolling 移动，不能把每轮候选自己的 D3 改善（例如 `1.643→1.380`）当作跨轮单调改善再被翻侧抵消。实际执行轨迹 227–231（末条轨迹源记为 `FEASIBLE_FALLBACK`）的 UAV1 y 从约 `0.640` 到 `0.545 m`，始终在树13（y=1.710 m）的同一南侧，没有物理往返穿侧证据。源码已有同 event 全平时 `last_committed_side` 偏好以及同 blocker 已接受 side 的保留条件。[planner_manager.cpp 1770–1793、9770–9792](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp)。因此 `D3_IMPROVING_BUT_SIDE_REVERSED_COUNT=0 confirmed`；不能据 7 次标签切换就提出新的 hysteresis。

## 7. FOV/RANGE 边界及因果优先级

FOV/RANGE 被统一 Local sample 用于 event 和 binary/margin；独立 yaw/radial tracking 仍处理它们。`spatial_visibility_trigger` 只接受 BODY/STATIC/DYNAMIC_LOS descriptor 或由 raw LOS 授权的 K3 grant，故 FOV/RANGE-only 不应被自动解释为“应向左/右绕一个不存在的 blocker”。FOV limiter 与 SIDE 同时出现不矛盾：114 UAV1/e7 在无 raw LOS/grant 时仍有 SIDE 尝试，其 `[threat-dispatch] reason=BODY`、`conflict-descriptor reason_mask=1 body_id=7`，并不证明 FOV 获得了 topology authority。113/114 的 UAV2 RANGE event 都无 SIDE、无实际 binary loss、以 `MARGIN_RECOVERED` 关闭；FOV/RANGE event 无 `EVENT_EXPIRED` 证据。现在没有 FOV/RANGE-only 导致无法恢复的真实事件，暂不扩展 spatial SIDE。[planner_manager.cpp 7630–7710、9634–9739、12760–12840](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp)。

**CAUSE 1 — 已证，首个功能瓶颈。** 已知遮挡区间强制 LOS observation plane 覆盖当前阴影；至少 24 对双侧 `LOS_PLANE_SCP_FAILED`，113/114 UAV1/e4 各 4 对、同 blocker26 全部 QP `primal_infeasible`。发生在 C3/D3 和 hard preflight 前，影响至少这 24 对，且 62 个失败侧尝试为 `L`。`YES`：证明 QP 在现有行集下不可行；`NO`：尚未证明扩大 trust 可解。

**CAUSE 2 — 已证。** 单 blocker SIDE 的偏移种子与静态地图冲突，现有 A* raw path 安全而平滑 seed 再碰撞；17 对双侧 `S/S`，静态终止侧尝试 47 次，典型为两 run UAV2/e2 的 gate10。`YES`：证明路径→多项式重建丢失静态可行性；`PARTIAL`：树37与 LOS 同窗出现，但具体碰撞树 ID 未记。

**CAUSE 3 — 局部已证，非全局首因。** event 在本 cycle 的 raw 扫描之后创建，短窗口要等下一 rolling；113/114 UAV1/e4 的 raw/SIDE 几乎贴着实际 binary loss。25 个有记录授权事件 `event→raw` P50 0.220 s。`PARTIAL`：对 e4 类事件成立；节流和历史 replay 定义差异不支持“所有 2–4 s lead 都被 raw gate 吃掉”。

**CAUSE 4 — 遥测错判，非候选拒绝根因。** `acceptance_accepted` 提前读取默认 `INVALID`，56/108 对被误称双侧失败，至少 36 对最终有有效 SIDE。`YES`：证明计数不可直接用于几何调参；最终 comparator 仍收到候选。其余假设（comparator 错排、`RECOVERY_NOT_REACHED` 文本门、纯 trust 不足、左右翻侧抵消 progress）没有相应 run 证据。

## 8. 有证据支持的修改方向；本轮不实施

1. **先处理已知遮挡的 LOS plane 可达时间。** 对 `LOS_OCCLUSION_ENTER_REL=0` 且 seed 未在观察侧的情形，设计能保持静态/动态/swarm hard safety 与 K2 不降的分阶段 SIDE 候选，再以完整同窗 C3/D3 评估进度；当前 cycle 未达到观察侧不得申报 K3 recovered，`required_margin` 和执行二值验证保持原值。实现前应给 plane QP 记录具体矛盾行、seed side reach time、候选的 hard-safe/K2/D3 结果，以决定是时间约束不可达还是求解实现问题。**不是**直接让 `RECOVERY_NOT_REACHED` 越过已有门，因为该门不存在。
2. **修复已有 A*→guide/Local-SFC→平滑 seed 的静态保持。** 针对两 run UAV2/e2，保留 raw path 的静态净空/转弯点，在多项式初值重建后对同一区间复核并回馈实际碰撞树 ID。只有此处证实不能在现有 machinery 中得到安全多转弯 seed，才考虑扩展 LOS-only 多 blocker path 搜索；当前不需要新增全局 planner。
3. **消除 event 创建后的整轮授权空档，并校正遥测。** 让同一 Local rolling 的 continuous risk event 与已扫描 raw descriptor 在同一 revision/世界时间下配对；远期 crossing 仍受当前 authority horizon、raw blocker、M2 优先权和窗口相交约束，避免预测抖动扩大 topology 搜索。把 `SIDE_BOTH_FAILED` 移到分类/preflight 后定义，分别记真实构造失败、hard preflight 失败与有效但未选；记录每个 grant 而非仅节流 INFO，才能测真正延迟。无需改 C3/D3 排序、增 side hysteresis 或调 SCP trust。

## 收尾字段

```text
PRODUCTION_CODE_CHANGED: NO
SIMULATION_RUN: NO
BUILD_RUN: NO
K3_EVENTS_ANALYZED: 42 (Feedback113 17 + Feedback114 25)
SIDE_BOTH_FAILED_EVENTS: 27 run-specific events (9 + 18)
SIDE_BOTH_FAILED_CYCLES: 108 logged bilateral fallback pairs (49 + 59; not 108 genuine dual failures)

MARGIN_TO_EVENT_DELAY_P50: -0.962 s (event_created - predicted_margin_cross; n=42)
EVENT_TO_WITNESS_DELAY_P50: 0.220 s (first grant-associated raw after event; n=25 logged grants)
WITNESS_TO_DISPATCH_DELAY_P50: 0.038 ms observed same-cycle raw-to-grant log; true population P50 unobservable because grant INFO is throttled
DISPATCH_TO_BINARY_LOSS_P50: 0.025 s (first logged grant to first executed blocked in event's initial window; n=18, throttled/censored)

SIDE_PLUS_FAILURE_REASON_COUNTS: T40 / LOS_SFC33 / STATIC24 / MINCO_UNSPECIFIED11
SIDE_MINUS_FAILURE_REASON_COUNTS: T47 / LOS_SFC29 / STATIC23 / MINCO_UNSPECIFIED9
SIDE_BOTH_FAILED_FIRST_CAUSE_COUNTS: telemetry_stale56 / LOS_SFC_both24 / STATIC_both17 / mixed11
SIDE_FAILURE_TOP1: telemetry-stale construction label, 56 pairs; 36 subsequently have valid SIDE
SIDE_FAILURE_TOP2: LOS observation-plane QP infeasible, 24 genuine bilateral pairs
SIDE_FAILURE_TOP3: static seed/A* smooth reconstruction failure, 17 genuine bilateral pairs
RECOVERY_NOT_REACHED_COUNT: 17 audit statuses in K3 event lifetimes; direct admission rejections 0
WINDOW_LIMITED_COUNT: 12 audit statuses in K3 event lifetimes; direct admission rejections 0
STATIC_INFEASIBLE_COUNT: 47 side attempts within fallback pairs
DYNAMIC_INFEASIBLE_COUNT: 0 proven first-termination attempts within fallback pairs
SFC_INFEASIBLE_COUNT: 62 LOS_PLANE_SCP_FAILED side attempts within fallback pairs
MINCO_FAILED_COUNT: 20 unspecified side attempts within fallback pairs
HARD_PREFLIGHT_FAILED_COUNT: UNKNOWN for SIDE; reject log lacks candidate kind/ID join

VALID_SIDE_CANDIDATES: 96 final-preflight-and-K2-admissible candidate aggregates (38 + 58)
K3_BETTER_SIDE_CANDIDATES: >=49 side-kind/batch lower bound by logged 3-decimal C3/D3 versus baseline (19 + 30); exact candidate count unavailable
K3_BETTER_SIDE_SELECTED: 32 strict-progress SIDE selections (13 + 19)
K3_BETTER_SIDE_COMPARATOR_REJECTED: 0 confirmed; exhaustive candidate-level count unavailable
SAME_BLOCKER_SIDE_SWITCH_COUNT: 7 selected-label transitions in 3 events
PROGRESSIVE_SIDE_SWITCH_COUNT: 7 selected-label transitions; not proof of physical cancellation
D3_IMPROVING_BUT_SIDE_REVERSED_COUNT: 0 demonstrable with comparable cross-cycle windows

RAW_WITNESS_GATE_IS_BOTTLENECK: PARTIAL, proven for short-lead e4-style events; not global first cause
PARTIAL_SIDE_ADMISSION_IS_JUSTIFIED: NO as a RECOVERY_NOT_REACHED/WINDOW_LIMITED gate change; staged hard-safe candidate research is justified by QP failure
LOS_ASTAR_RECOVERY_IS_JUSTIFIED: NO as a missing connection; existing A* runs, its smooth reconstruction needs repair
SIDE_PERSISTENCE_IS_JUSTIFIED: NO
ADAPTIVE_TRUST_JUSTIFIED: NO
FIRST_CAUSAL_BOTTLENECK: known-occlusion LOS observation plane / QP infeasible before C3/D3; static A*-to-polynomial reconstruction second
RECOMMENDED_CHANGE_1: diagnose and stage LOS-side reach time while retaining all hard safety, K2 and exact recovery validation
RECOMMENDED_CHANGE_2: preserve static feasibility across existing A* raw path to Local-SFC/MINCO seed reconstruction
RECOMMENDED_CHANGE_3: same-revision event/raw descriptor pairing and truthful per-stage SIDE telemetry
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO (no simulation started this turn)
```

证据入口：[113 roslaunch stdout](../runs/20260925_190928_365841/roslaunch_stdout.log.gz)、[113 visibility](../runs/20260925_190928_365841/visibility.csv)、[113 trajectory](../runs/20260925_190928_365841/visibility_trajectory.csv)、[114 roslaunch stdout](../runs/20260925_200523_419199/roslaunch_stdout.log.gz)、[114 visibility](../runs/20260925_200523_419199/visibility.csv)、[114 trajectory](../runs/20260925_200523_419199/visibility_trajectory.csv)、[114 ROS log archive](../runs/20260925_200523_419199/ros_log.tar.gz)、[事件时间 CSV](artifacts/feedback115_event_timeline_enriched.csv)、[失败对 CSV](artifacts/feedback115_side_pair_first_causes.csv)、[去重节点日志摘录](artifacts/feedback115_filtered_events.tsv.gz)。
