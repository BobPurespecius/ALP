# Feedback 134 — 正确绕行 topology fate 调查（离线核验）

日期：2026-09-27  
工作区：`/home/bob/ALP/egov2_fc65423_constvel`  
调查模式：按“不仿真”要求，本次离线续查没有启动新的仿真或 build，也没有改源码。

工作区已有一个诊断 FULL：`20260927_103759_1243248`，manifest 记载 10:38 启动、BOOT-12、exit 0；其 candidate/LOS JSONL 在 `feedback/artifacts/feedback134/diagnostic_full/`。该运行曾使用临时 telemetry 并 build，属于既存证据；本报告离线读取，没有重跑。故不能把整个工作流表述成“从未有过仿真/build”，状态栏会区分“本次新启动”与“已有完成产物”。动态库 launch/end hash 一致，但三份源文件 hash 不一致，精确 source provenance 有限制，见下文。

审计边界：初始 shell 的 cwd 曾是 `/home/bob/RRCT`，只设置了工作目录，没有读取或写入 RRCT 仓库文件。按用户的严格定义如实记 `RRCT_ACCESSED: YES (cwd only)`、`RRCT_CHANGED: NO`。

## 结论

**唯一 PRIMARY（限于已留存、可严格配对的 witness）：2. TOPOLOGY COLLAPSE IN OPTIMIZATION。** 现存 run11 telemetry FULL 记录了 seed 与主 MINCO 前后 polynomial。严格 witness 集 W 有 11 条已经通过 initializer 的 PVA/static/dynamic/swarm 检查、且 blocker-relative 几何重放更早持续恢复并降低共同窗口 K3 loss 的 seed。10/11 在主优化输出处先失去这项净恢复收益；其中 7 条最终仍可执行但收益变差，3 条后来又因 PVA 超限被拒。candidate 35 是直观例子：initializer 持续恢复 8.727s、共同窗口 K3 loss 0.81s，主优化 final 变为 8.837s、0.86s，仍 PVA 合格。selector 拿到的是已退化轨迹，不能再选回 seed 的原几何。

**适用范围必须收窄。** run8/run10 的 19 个逐 UAV loss episode 没有保存未选 polynomial；run11 还有 2/9 个 episode 没有严格正确 topology witness。上面的 PRIMARY 是“候选级已证明第一分歧中占多数的机制”，不能声称它单独解释全部 24 个合并 episode 或 K3 总损失。对 run8/run10 还可以证实未来静态 LOS 时间没有传入 observation-plane 时窗，但没有正确方向/未选轨迹，不能证明该 bug 恰好杀掉了物理最优一侧。

次因有两项直接证据：主优化 PVA 合格的轨迹经独立 LOS slack-QP 改写后，138 对 PRE/POST payload 中 93 条变为 PVA 超限，85 条被 hard preflight 拒绝；另有一个仍 hard-safe 的 SIDE candidate 151 在 C3 comparator 中输掉。二者分别是 executability 与 selection 问题，但严格 witness 中它们都晚于主优化几何收益首次退化，故不是 PRIMARY。

## 样本、有效性和指标口径

纳入 run8 `20260927_055748_1104145`、run10 `20260927_061256_1157439`、run11 `20260927_103759_1243248`。三个 manifest 都标记 `ABLATION_MODE=full`，exit status 都是 `CORE_EXIT_CODE=0`、`FINAL_EXIT_CODE=0`、`BOOT-12`。run9 `20260927_060327_1111508` 按既有 feedback_133 的双 planner callback 静默停摆调查排除，不把它的低 ALL3 混进 topology 选择结论。

K3 episode 从 `visibility.csv` 的 `visible_count < 3` 逐样本识别；loss 秒数为每个失去 K3 的样本到下一样本的 `time_s` 差之和。相隔至多一个全可见样本的连续区间合并为一个 episode，但该全可见样本不计入 loss 秒数。

| run | raw `visible_3_ratio` | K3-loss episodes | K3 loss 总时长 |
|---|---:|---:|---:|
| run8 | 0.900767 | 7 | 7.771 s |
| run10 | 0.867178 | 9 | 10.404 s |
| run11 | 0.891351 | 8 | 8.498 s |
| 合计 | — | **24** | **26.673 s** |

feedback_133 报告的 run8/run10 `ALL3` 分别为 0.898475/0.864052，与这里 `visibility_summary.csv` 的 `visible_3_ratio` 不同，原始工件没有在该 summary 中给出二者的转换口径。本报告只用 `visibility.csv` 计算 episode 与 loss 秒数，不把两种 ALL3 值混算成同一指标，也不声称 run8 的单次高值代表稳定增益。

## 实际 blocker 几何与执行结果

复用源码中的 observation-frame 方向：`e = normalize(C_blocker - P_target)`，`n_R = (e_y, -e_x, 0)`；实际轨迹的侧向坐标 `s = n_R · (P_camera - C_blocker)`。静态净 LOS clearance 用相机到目标的 XY 线段到圆柱中心的距离减去 `radius + 0.08m`。表中 side 是实际轨迹在该 episode 最小线段 clearance 采样点的 `sign(s)`，不是 SIDE_PLUS/MINUS 标签。

`shadow exit` 是最小 clearance 之后 `|s| - (radius+0.08m)` 首次连续至少 0.3s 非负的时刻；末列是实际 affected UAV 首次恢复 binary visible、并持续可见至少 0.5s 的时刻。表中 first/persistent 两值相同是按采样精度四舍五入后的结果。线段 clearance 转正与该 binary 恢复相差不超过一个采样间隔。它们是 executed-path 量，不是未选 candidate 的反事实结果。

| run / episode | K3 loss | UAV / blocker | 实际源 / 几何 side | min shadow clearance | min LOS clearance | shadow exit | first / persistent binary |
|---|---:|---|---|---:|---:|---:|---:|
| run8 E1 7.949–8.749 | 0.835s | U1 / 36 | NOMINAL / + | -0.368m | -0.376m | 8.516 | 8.783 / 8.783 |
| run8 E2 10.415–11.786 | 1.402s | U1 / 26 | NOMINAL / - | -0.622m | -0.626m | 11.450 | 11.817 / 11.817 |
| run8 E3 24.849–26.215 | 1.400s | U1 / 28 | FEASIBLE_FALLBACK / - | -0.587m | -0.610m | 25.849 | 26.248 / 26.248 |
| run8 E4 35.549–36.449 | 0.933s | U3 / 6 | NOMINAL / - | -0.496m | -0.502m | 36.283 | 36.482 / 36.482 |
| run8 E5 63.916–65.015 | 1.133s | U1 / 13 | NOMINAL / - | -0.470m | -0.492m | 64.716 | 65.049 / 65.049 |
| run8 E6 70.715–71.882 | 1.201s | U3 / 14 | NOMINAL / - | -0.470m | -0.483m | 71.850 | 71.916 / 71.916 |
| run8 E7 73.149–73.987 | 0.867s | U3 / 37 | FEASIBLE_FALLBACK / + | -0.351m | -0.356m | 73.750 | 74.016 / 74.016 |
| run10 E1 8.018–8.784 | 0.799s | U1 / 36 | NOMINAL / - | -0.353m | -0.370m | 8.552 | 8.817 / 8.817 |
| run10 E2 10.417–11.784 | 1.401s | U1 / 26 | NOMINAL / + | -0.620m | -0.625m | 11.451 | 11.818 / 11.818 |
| run10 E3 25.052–26.350 | 1.332s | U1 / 28 | FEASIBLE_FALLBACK / + | -0.584m | -0.610m | 26.051 | 26.384 / 26.384 |
| run10 E4 34.951–35.924 | 1.001s | U3 / 6 | NOMINAL / + | -0.505m | -0.507m | 35.751 | 35.952 / 35.952 |
| run10 E7 63.784–64.950 | 1.200s | U1 / 13 | NOMINAL / + | -0.500m | -0.506m | 64.651 | 64.984 / 64.984 |
| run10 E8 70.651–72.018 | 1.400s | U3 / 14 | NOMINAL / + | -0.493m | -0.498m | 71.885 | 72.051 / 72.051 |
| run10 E9 73.350–75.884 | 2.535s | U3 / 37 | NOMINAL / - | -0.359m | -0.360m | 73.985 | 74.284 / 74.284 |
| run11 E1 8.003–8.769 | 0.799s | U1 / 36 | NOMINAL / - | -0.311m | -0.355m | 8.536 | 8.803 / 8.803 |
| run11 E2 10.478–11.904 | 1.458s | U1 / 26 | NOMINAL / + | -0.580m | -0.606m | 11.603 | 11.936 / 11.936 |
| run11 E3 25.003–26.235 | 1.269s | U1 / 28 | FEASIBLE_FALLBACK / + | -0.601m | -0.617m | 25.936 | 26.272 / 26.272 |
| run11 E4 35.478–36.368 | 0.925s | U3 / 6 | NOMINAL / + | -0.494m | -0.501m | 36.235 | 36.402 / 36.402 |
| run11 E6 63.769–64.936 | 1.200s | U1 / 13 | NOMINAL / + | -0.499m | -0.505m | 64.636 | 64.969 / 64.969 |
| run11 E7 70.236–71.402 | 1.201s | U3 / 14 | FEASIBLE_FALLBACK / + | -0.503m | -0.505m | 71.311 | 71.436 / 71.436 |
| run11 E8 73.237–74.072 | 0.865s | U3 / 37 | FEASIBLE_FALLBACK / + | -0.338m | -0.350m | 73.837 | 74.102 / 74.102 |

另外三例没有静态 blocker-only 解释：run10 E5 U1 dynamic LOS，39.684–40.251，binary persistent recovery 40.284；run10 E6 U1 FOV loss，43.984–44.084，恢复 44.120；run11 E5 U1 dynamic LOS，57.701–58.436，恢复 58.482。两个 UAV 同时失去可见性的 episode 还需区分 per-UAV 和全队恢复：run8 E7 的 U2 dynamic LOS 于 73.949 恢复、U3/static 于 74.016 恢复；run10 E9 的 U3/static 于 74.284 恢复，但 U2/dynamic 到 75.920 才恢复，故 K3 持续丢失到 75.920；run11 E8 的 U2/dynamic 于 73.935 恢复、U3/static 于 74.102 恢复。

重复事件在 observation frame 中的 side 符号会翻转。例如 blocker 36 的三次实际 winner 分别为 `+,-,-`，blocker 26 为 `-,+,+`。因此几何结果不能由候选标签、请求侧或另一 run 的符号代替。blocker ID/半径取自场景 `natural_team_stress_dense_38_targeted_k2_v6.json` 的 `obstacleData`。

## 从候选到执行：日志能证明什么

- run8/run10 的运行目录确实没有未选 candidate polynomial。另有一组既存 run11 telemetry JSONL 在 `feedback/artifacts/feedback134/diagnostic_full/`：三机 candidate 文件共 10,267 行，保存 TIMED_SEED、PRE_LOS、POST_LOS、FINAL_INPUT、比较/提交等 stage 的 polynomial pieces 与 candidate identity；不能再笼统说所有现存工件都没有候选几何。
- 用这些 JSONL 离线复算得到 `candidate_lifecycle.csv`、`20260927_103759_1243248_candidate_oracle.csv` 和 `strict_witness_funnel.csv`。完整源码溯源仍有限：`end_launch_hash_comparison.json` 显示运行期间两份动态库 hash 不变，但三个源文件的 launch/end hash 不同；临时 telemetry 曾改源再被撤回，缺少逐字节重建的 launch source snapshot。因此 trajectory payload 是实际既存运行记录，算法行号与其对应关系需结合当时源码快照理解，不能声称 source provenance 完全封闭。
- run8/run10/run11 `[side-budget-skip]` 总数分别为 **824/785/778**；run11 中 16 次落在运动期、episode audit window 去重后 12 次。跳过发生在 `optimize_side` 前，未生成 seed，几何方向无法回填，所以正确侧 deadline starvation 仍 UNKNOWN。
- `[LOS_SOFT_PLANE]` 有 slack 的记录：run8 **1315/1580**，run10 **349/1612**，run11 **1311/1562**。仅靠 slack 不能证明拓扑塌缩；run11 的 seed/final payload 对才允许在 11 个 witness 上直接算恢复收益变化。
- `[SIDE_TOPOLOGY_INTENT]` 行数为 run8 **94**、run10 **86**、run11 **113**。源码注释明确称它是 primary blocker 周围的 intent measurement，并写 `used_as_hard_gate=0`；它不是 A* admission、安全门或真实 LOS oracle。
- `[K3_SIDE_CANDIDATES]` 汇总行数为 run8 **105**、run10 **170**、run11 **106**。这些计数只用来描述生产选择流；只有 run11 JSONL 的 candidate identity 与完整 polynomial 可以参加几何 oracle。run8/run10 的选择原因行即便字段完整，也不能替代缺失的候选轨迹。

**历史静态 LOS 的 generator 时间断层：**`planner_manager.cpp:9051-9056` 前视静态扫描设置 `raw_static_los_time` 和 blocker witness；但 `raw_los_occlusion_interval_observed_` 只在动态圆柱分支 `:9092-9095` 置位，或者等当前 odom 已处遮挡才在 `:9182-9192` 设置区间。`optimize_side` 仅在 interval flag 有效时采用 LOS enter/exit（`:10162-10196`）；否则退回 `nominal_result.risk.conflict_time ±0.45s`（`:10152-10154`），并用窗口起点作 side reach deadline（`:10198-10201`）。

run8 相对时刻约 7.715s，raw detector 已报静态 blocker36 将在 0.1s 后遮挡、无 dynamic blocker；SIDE_PLUS audit 的 LOS enter/exit 仍为 -1，落入 `WINDOW_LIMITED`，无 plane；SIDE_MINUS plane 到达时间是 1.764901s，而真实 K3 loss 于 7.948906s 开始。run10 在约 7.723s 重现：静态遮挡 0.1s 后到来，PLUS 无 plane，MINUS plane 到达 1.703706s，loss 于 8.017654s 开始。相应 `[threat-dispatch]` 记录 body conflict=-1、LOS time=0.1s，plane 窗却为 [1.55,2.45]；因此是 generic nominal-risk fallback，不能说该 batch 检出了真实 BODY 冲突。

这证明 run8/run10 的未来静态 LOS 时刻没有成为 SIDE observation-plane 的时间权威，是 generation-timing 缺陷。由于这些 run 没有未选 polynomial，不能确定哪侧几何正确，也不能证明该缺陷杀掉了最快的硬安全 topology。故这 19 个历史逐 UAV loss episodes 的候选级首次分歧仍 UNKNOWN；不能和 run11 的十个 B witness 混成同一因果计数。

## 必需 fate funnel 与错误 winner funnel

计数分两层，避免把一次局部 witness 伪装成全体事件的结论：run8/run10 的逐 UAV candidate fate 缺失；run11 的 archived JSONL 能做 candidate 级复算。报告前面的 24 是将最多一个全可见样本的间隔合并后的 episode 数；按 maximal `visible_count<3` UAV runs 计为 28 个单机 episode、25 个联合 K3 事件，合并不改变 loss seconds。

严格集合 W 的纳入条件是同 batch 下 seed 持续恢复更早、共同窗口 K3 loss 更少；initializer 判定 `ABSOLUTE_SAFE`；恢复与后续 0.3s 连续可见处于 batch 的预测/证书支持范围；独立重放通过连续 PVA、真实场景 static/dynamic 圆柱 body clearance 和 swarm 距离检查。seed 仍不是最终 executable/certified candidate。较宽集合 R 从 final 候选筛选：相对实际轨迹持续恢复提前超过 0.05s、共同窗口无显著 K3 回退，再要求 certificate 覆盖恢复区间和实际 blocker/body、swarm 复核安全。R/W 会重叠，不能加总。

```text
TOTAL_K3_LOSS_EVENTS = 25 union / 28 UAV episodes (24 after one-visible-sample merge)
CORRECT_TOPOLOGY_IDENTIFIABLE = 7/9 run11 episodes; run8/run10 19 UAV episodes unresolved
CORRECT_TOPOLOGY_NOT_GENERATED = UNRESOLVED outside recorded witnesses; never count missing payload as zero
CORRECT_TOPOLOGY_SEED_GENERATED = 11 strict W seed witnesses in 6 episodes
SEED_CORRECT_BUT_FINAL_GEOMETRY_COLLAPSED = 10/11 W (10 first-B divergences)
CORRECT_FINAL_TRAJECTORY_EXISTS = 1 in strict W; broader R has 8 certified conditional recovery witnesses (cohorts overlap)
CORRECT_FINAL_DYNAMICS_REJECTED = 0 strictly correct finals; 3 correct-seed lineages later PVA-rejected after collapse
CORRECT_FINAL_STATIC_REJECTED = 0 in W; broader non-W failures are not counted as correct-topology rejects
CORRECT_FINAL_DYNAMIC_REJECTED = 0 in W
CORRECT_FINAL_SWARM_REJECTED = 0 in W
CORRECT_FINAL_CERTIFICATE_REJECTED = 0 in W; 34 UNKNOWN certificates in all-candidate audit
CORRECT_FINAL_DEADLINE_STARVED = 0 observed in W; 12 budget skips remain geometrically unresolved
CORRECT_HARD_SAFE_EXISTS = 1 in strict W; 8 in broader R (overlap; do not add)
CORRECT_HARD_SAFE_LOST_K2 = 0 observed
CORRECT_HARD_SAFE_LOST_C3_D3 = 1 (candidate 151 loses at C3)
CORRECT_HARD_SAFE_LOST_OLD_ORDER = 0 observed
CORRECT_SELECTED = 0 in strict W; 6 in broader R
CORRECT_COMMITTED = 0 in strict W; 6 in broader R
CORRECT_ACTIVATED = 0 in strict W; 6 in broader R
CORRECT_EXECUTED = 0 in strict W; 6 in broader R
CORRECT_EXECUTED_AND_RECOVERED = 0 completed payload recovery observed
```

```text
WRONG_NOMINAL_WON_BECAUSE_CORRECT_SIDE_ABSENT = 3 W cases (seed advantage collapsed in main optimizer)
WRONG_NOMINAL_WON_BECAUSE_CORRECT_SIDE_UNSAFE = 3 W cases (SIDE family later failed PVA preflight)
WRONG_NOMINAL_WON_BECAUSE_DEADLINE = 0 observed; skipped-side topology unknown
WRONG_NOMINAL_WON_BY_C3 = 0 in W
WRONG_NOMINAL_WON_BY_D3 = 0 in W after upstream collapse
WRONG_NOMINAL_WON_BY_OLD_ORDER = 0 in W
WRONG_SIDE_WON = 5 in W (4 degraded SIDE winners; 1 C3 loss to other SIDE)

OBSERVED_EXECUTED_SOURCE_AT_STATIC_SHADOW_MINIMUM:
  NOMINAL = 15 / 21
  FEASIBLE_FALLBACK = 6 / 21
  SIDE_PLUS_OR_MINUS = 0 / 21
```

`W` 是严格 seed-to-final first-divergence cohort；`R` 是更宽的 final candidate recovery screen。两者不是互斥事件漏斗，数值不可相加。上面 12 个 skipped SIDE 无 seed 几何，保持 UNKNOWN。

所有 run11 episode window 去重后有 541 条 N/L/R candidate lifecycle：352 SIDE dispatch，其中 266 到达 timing/solver；184 solver success、82 failure，62 个失败分支经 initializer retention 进入 final，20 个没有 final。第一终止 stage 互斥统计如下：

|first exit / comparator|NOMINAL|SIDE|合计|
|---|---:|---:|---:|
|PREINIT_REJECT|0|86|86|
|SOLVER_FAILED_NO_FINAL_INPUT|0|18|18|
|FINAL_INPUT_FAILED|60|2|62|
|HANDOFF_REJECT|30|17|47|
|PREFLIGHT_REJECT|14|86|100|
|CERTIFICATE_REJECT|2|32|34|
|进入 hard-safe comparator|83|111|194|

194 条 comparator inputs 中 108 selected、97 committed 且在 `visibility_trajectory.csv` 观察到执行；11 selected 未 commit 的确切原因 telemetry 不足。100 次 preflight rejection 为 85 PVA、14 static、1 dynamic、0 swarm；47 个 handoff reject 都是 `MISSED_FROZEN_ACTIVATION`，34 个 certificate UNKNOWN。表中总量描述全候选生命周期，不冒充“正确 topology”专属数量。

|production 阶段|本次可复核语义 / gate|
|---|---|
|风险识别与 N/L/R dispatch|raw LOS witness 可以触发候选；budget skip 在 SIDE optimizer 前退出，不能推断未生成分支的几何|
|seed、A*、Local-SFC、PVA timing|SIDE initializer 表达 blocker-relative 意图；A* 仅在静态局部修复需要时介入。强 W 多数不依赖 A*/static corridor，A* 日志标签也不等于全部候选均走过 A*|
|MINCO/SCP 主优化|以 seed/联合 seed 初始化并输出连续 polynomial；数值 `success` 不代表保留 blocker-relative 恢复优势；这是 10/11 W 的首个 B|
|post-solve LOS slack-QP|固定 durations 改内部位置点；其返回 true 不代表保持 PVA/static/dynamic/swarm 可行性，PRE/POST payload 证实 93/138 PVA 破坏|
|initializer retention|安全 seed 仅在主 solver failure 路径获得回退；solver success 但质量退化时不保护原 incumbent|
|frozen handoff 与 hard preflight|handoff 可因 `MISSED_FROZEN_ACTIVATION` 提前拒绝；通过后仍需依次过 PVA、static、dynamic、swarm final checks|
|certificate 与 K3 comparator|物理通过后仍需证书 coverage；selector 在共同有效窗口比较 K2/C3/D3 与顺序，candidate151 是唯一 strict-W C3 错选|
|commit、activation、execution|selected 不等于 committed；run11 97 个 commit 均能按 trajectory ID 在 executed CSV 观察到，宽 R 中 6 条又在预计恢复前被 successor 替换|

W 的首处分歧逐项如下（恢复/loss 秒数来自同窗口复算；B=主优化后净恢复质量退化，K=仍正确 hard-safe 但 comparator 输）：

|episode / candidate|首处分歧|seed → final 持续恢复|seed → final K3 loss|之后结果|
|---|---|---:|---:|---|
|u1e1 / d0:35|B|8.727 → 8.837|0.81 → 0.86|NOMINAL 34 执行|
|u1e2 / d0:77|B|11.723 → 11.963|1.27 → 1.50|NOMINAL 75 执行|
|u1e2 / d0:80|B，后 DYNAMICS_FAIL|11.774 → 10.844*|1.31 → 1.47|final jerk 315.995，preflight 拒绝|
|u1e2 / d0:83|B，后 DYNAMICS_FAIL|11.850 → 11.100*|1.11 → 1.14|final jerk 318.985，preflight 拒绝|
|u1e3 / d0:167|B|26.194 → 26.374|1.25 → 1.42|退化 SIDE_MINUS 仍被选中执行|
|u1e5 / d0:531|B|64.728 → 64.948|1.03 → 1.20|NOMINAL 530 执行|
|u1e5 / d0:534|B|64.813 → 64.953|1.08 → 1.21|NOMINAL 533 执行|
|u1e5 / d0:540|B，后 DYNAMICS_FAIL|64.897 → 64.117*|1.14 → 1.13|final jerk 292.304，preflight 拒绝|
|u3e1 / d2:151|K|36.119 → 36.099|0.80 → 0.81|hard-safe；因 C3 输给 149，150 获选|
|u3e1 / d2:154|B|36.184 → 36.324|0.84 → 0.92|退化 SIDE_PLUS 仍被选中执行|
|u3e3 / d2:406|B|73.991 → 74.251|0.93 → 1.04|SIDE_MINUS 405 执行|

`*` 这些最终轨迹的短时首次可见并不构成持续恢复；不能把它计作“correct final”。`B` 的 side position 仍可能跨 shadow axis，所以这里断言的是 blocker-relative 时空恢复收益被优化洗掉，不声称证明了严格数学 homotopy 翻转。

## 对六个问题和四选一主结论的回答

1. **正确 topology 有没有生成？** 有可复核的 witness：W 中 11 条 seed、覆盖 run11 六个 episode。不能证明所有 episode、所有 side 或全局最优都已生成。
2. **seed 正确而 final 还正确吗？** 多数不保留恢复收益：W 10/11 在主 MINCO 输出处第一次退化；7 条仍过 PVA 但更慢/总 loss 更大，3 条后续被 LOS refinement 弄成 PVA 超限。不是标签从 PLUS 变 MINUS 的判断。
3. **正确 final 为什么没 hard-safe？** 对 80/83/540，first divergence 已先发生在主优化，后续 post-solve LOS slack-QP 移动 internal points 并造成高 jerk；SafetyKernel 的拒绝是正确的。大 cohort 138 对 PRE/POST 中，93 个 POST PVA broken，85 个被 PVA preflight 拒绝、8 个先 handoff reject。LOS QP 没有 PVA/static/dynamic/swarm 约束，true return 不能继承原 PVA 证明。
4. **hard-safe 时为何没被 selector 选？** 严格 counterexample 是 candidate151：K2 相同=1；它的 C3=0.3571、D3=3.2773，selected candidate150 为 C3=0.4286、D3=3.4696，但比较/预测目标口径先让其输给 candidate149，最终150胜。151 有更早持续恢复且共同窗口 K3 loss 0.81s 对0.98s。它是明确的 bounded C3 selection miss；W 的 11 个 witness 中，selection 不是多数 first divergence。
5. **被选之后哪里丢失？** 97 个 commit 均能在实际 trajectory ID 看到执行。宽 R 集合中 6 条 hard-safe、恢复更早的 candidate 被选中/提交/执行，但都在预测持续恢复前被 successor 替换；这是 O 类可见事实，替换是否导致 K3 晚恢复尚未通过闭环反事实证明。11 个 selected-but-not-committed 的原因不能从现存 telemetry 唯一归因。严格 W 中 candidate151在 comparator 已落选，故不进入 commit。
6. **K3≈0.89 最大单一根因？** 唯一 PRIMARY 选 **2. TOPOLOGY COLLAPSE IN OPTIMIZATION**，限定于可核实候选级 witness：10/11 first B。它解释 selector 为什么后来常看到 NOMINAL/另一 SIDE，却不能量化成 run8/run10 的全部历史 loss。

```text
1 GENERATION FAILURE                     HISTORICAL STATIC-TIME BUG PROVEN; correct-side fate unresolved
2 TOPOLOGY COLLAPSE IN OPTIMIZATION      PRIMARY: 10/11 strict W first divergences
3 EXECUTABILITY / SCHEDULING FAILURE     SECONDARY: 93/138 post-LOS PVA break; skip-side correctness unknown
4 SELECTION FAILURE                      SECONDARY: candidate151 C3 counterexample

PRIMARY_ROOT_CAUSE = TOPOLOGY_COLLAPSE_IN_OPTIMIZATION (strict W scope)
```

关键因果句：**在 run11 可证的正确 recovery seed 中，真正较早恢复的 blocker-relative 几何第一次在主 MINCO 成功返回时被连续优化目标退化；后续 comparator 因此只比较退化 SIDE 与 NOMINAL，NOMINAL 可按现有 C3/D3 取得胜利。若几何尚能保留且 hard-safe（candidate151），则有一个可复现 C3 误选；但这不是多数 witness 的首因。run8/run10 静态前视 LOS 时间还存在 detector→plane 的 authority gap，不过缺少未选 payload，不能声称它就是那两次事件中物理正确侧被杀的位置。**

## 最小架构修复（建议，未实施）

修复要沿两处已证据化的模块边界收敛。首先，SIDE 的 blocker-relative 持续恢复目标必须由同一个受物理约束的 optimizer 保持；现有安全 seed 不应只在 solver false 时才被用作 fallback，数值成功也不能无条件覆盖质量更好的 feasible incumbent。其次，取消独立无物理约束的 post-solve LOS polynomial 改写，将 LOS soft rows 合并进已有 constrained refinement，仍保留 slack 与最终 SafetyKernel preflight。对未来静态 LOS，在前视 detector 已有 witness/time 时就让 generator 使用同一 world-time event interval，避免落回无关 nominal-risk 窗口。保持现有 activation/head/PVA、N/L/R、SafetyKernel 与 selector 职责，不加新 FSM、dwell 或第二套 planner。

candidate ID/hash 到 seed、final、rejection、comparison、commit/activation/execution 的紧凑 fate artifact 继续保留，便于验证上述契约；对 `side-budget-skip` 只能记未生成状态与预算，不能伪造不存在的 seed 几何。

本次离线续查没有改源码、策略、参数，也没有启动 build 或 simulation。已存在的 run11 telemetry FULL 用作候选级 evidence；因为没有策略修复对照，不能声称这次调查证明了修复收益或稳定 K3 提升。

## 状态与证据文件

- HEAD：`eb3ae0cf128bb6b4a4d07202f4fca08560f5e921`；worktree 有三个 tracked dirty source 文件：`planner_manager.h`、`planner_manager.cpp`、`poly_traj_optimizer.cpp`。没有改写或清理 dirty source；没有执行 reset/checkout/restore/clean。临时 telemetry 源码已撤回，但 archived run11 仍来自当时 build 的二进制。
- 源码交叉核对位置：[CandidateResult](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/include/plan_manage/planner_manager.h#L204)、[K3 汇总](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp#L1934)、[SIDE intent](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp#L10951)、[LOS soft-plane 遥测](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp#L12816)、[budget skip gate](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp#L13370)、[static LOS future interval](../ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp#L9051)。
- run11 candidate payloads：[candidate JSONL](artifacts/feedback134/diagnostic_full/)、[lifecycle CSV](artifacts/feedback134/candidate_lifecycle.csv)、[candidate geometry oracle](artifacts/feedback134/20260927_103759_1243248_candidate_oracle.csv)、[strict witnesses](artifacts/feedback134/strict_witness_funnel.csv)、[PVA before/after LOS](artifacts/feedback134/los_before_after.csv)。
- run8：[manifest](../runs/20260927_055748_1104145/run_manifest.txt)、[exit](../runs/20260927_055748_1104145/exit_status.txt)、[visibility](../runs/20260927_055748_1104145/visibility.csv)、[executed trajectory](../runs/20260927_055748_1104145/visibility_trajectory.csv)。
- run10：[manifest](../runs/20260927_061256_1157439/run_manifest.txt)、[exit](../runs/20260927_061256_1157439/exit_status.txt)、[visibility](../runs/20260927_061256_1157439/visibility.csv)、[executed trajectory](../runs/20260927_061256_1157439/visibility_trajectory.csv)。
- run11：[manifest](../runs/20260927_103759_1243248/run_manifest.txt)、[exit](../runs/20260927_103759_1243248/exit_status.txt)、[visibility](../runs/20260927_103759_1243248/visibility.csv)、[executed trajectory](../runs/20260927_103759_1243248/visibility_trajectory.csv)。其 cleanup 记录 `CLEANUP_REMAINING_OWNED=0`；当前没有运行中的 ROS/Gazebo simulation process。
- run9 排除依据：[feedback_133.md §26](feedback_133.md) 记录其双 planner callback 静默停摆。

```text
PRODUCTION_CODE_CHANGED: NO net production-strategy change; earlier temporary telemetry source was built for existing run11 and later withdrawn
BUILD_RUN: NO new build in this offline pass; existing run11 build log is 25/25
SIMULATION_RUN: NO new simulation in this offline pass; existing run11 FULL completed BOOT-12 / exit 0

TOTAL_K3_LOSS_EVENTS: 25 union / 28 UAV episodes; report table merges one-visible-sample gaps to 24
CORRECT_TOPOLOGY_IDENTIFIABLE: 7/9 run11 episodes; 19 historical run8/run10 UAV episodes unresolved
CORRECT_TOPOLOGY_GENERATED: 11 strict blocker-relative safe seed witnesses in W
CORRECT_FINAL_TRAJECTORY_EXISTS: 1 in strict W; 8 conditional witnesses in broader R (overlapping cohorts)
CORRECT_HARD_SAFE_EXISTS: 1 in strict W; 8 conditional witnesses in broader R (overlapping cohorts)
CORRECT_SELECTED: 0 in strict W; 6 in broader R
CORRECT_EXECUTED: 0 in strict W; 6 in broader R, all superseded before predicted recovery

PRIMARY_ROOT_CAUSE: TOPOLOGY COLLAPSE IN OPTIMIZATION (strict W scope, 10/11 first divergences)
SECONDARY_ROOT_CAUSE: STATIC LOS future-time authority gap; post-LOS PVA break; one C3 miss

GENERATION_STATUS: 11 verified seed witnesses; completeness unknown; run8/run10 static future interval defect proven, correct side attribution unknown
OPTIMIZATION_TOPOLOGY_PRESERVATION_STATUS: FAIL in W — 10/11 lost net recovery benefit at main optimizer output
EXECUTABILITY_STATUS: POST-LOS QP broke PVA in 93/138 paired outputs; final preflight rejected unsafe output
SCHEDULING_STATUS: 12 episode-window budget skips lack seed geometry; correct-side starvation unresolved
SELECTOR_STATUS: one hard-safe C3 counterexample; not primary

MINIMAL_ARCHITECTURAL_FIX: preserve blocker-relative recovery quality inside the physically constrained optimizer; merge LOS soft refinement into that contract; pass future static LOS event time to the generator

RRCT_ACCESSED: YES — INITIAL CWD ONLY; NO RRCT FILES READ OR WRITTEN
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```
