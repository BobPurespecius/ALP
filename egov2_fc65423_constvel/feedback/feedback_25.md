# Scenario A executed accumulated visible time 下降原因：只读源码与日志审计

审计日期：2026-09-10  
项目：`/home/bob/ALP/egov2_fc65423_constvel`  
范围：当前生产源码、`visibility_effect_validation_20260910/runs_final2` 的 3×OFF/3×ON CSV 与 launcher 日志。  
本轮未修改生产算法，未运行仿真，未访问 `/home/bob/RRCT`。

## 结论先行

ON 的现象不是“第三路偶发丢失”，而是系统把轨迹从大量三机同时可见，重排成更稳定的两机可见。3 次 ON executed visibility 中，`visible_count==2` 累计约 69.53 s，`visible_count==3` 约 167.37 s；OFF 中分别约 13.82 s 与 222.58 s。由此平均每次运行的 camera-time 减少约 18.18 camera·s。

最强证据是：ON 的第三路损失由水平 FOV 与静态 LOS 主导，而不是 blackout；并且日志中经常出现 `candidate_count=1`、`NO_VALID_VISIBILITY_ALTERNATIVE`、`SOURCE_CANDIDATE_NOT_ABSOLUTE_SAFE` 和 `ACTIVATION_RESERVE_EXHAUSTED`。因此问题同时包含：

1. 120° 合围/slot 参考把 UAV 保持在固定观察扇区，造成部分轨迹进入不利 FOV/静态 LOS 区域；
2. 候选生成和安全过滤先行收缩，很多时段根本没有高累计可见候选交给 27 组合选择；
3. selector 明确以 K2 near-best 为保护层，再在其中最大化 `mean_visible_count`，而不是全局最大化累计 camera-time；
4. joint optimizer 的 `J_acc` 只是 continuous visibility 的次级软项，且 acceptance 允许 K2 小幅退化（tolerance 0.02），没有直接约束 All3 或 executed camera-time；
5. ON 存在明显安全执行回归，且若干 joint/side candidate 因动态或静态安全失败而 fallback，进一步改变了实际轨迹。

## 1. 全时域 executed visibility loss decomposition

分类依据是每个 `*_visibility.csv` 的 executed `visible_uav*` 与 `range_valid/static_los_clear/dynamic_los_clear/camera_fov_valid` 字段；原因按 `range → static LOS → dynamic LOS → HFOV/VFOV` 优先级归类。累计时间为 3 次同模式的总和，约 80.17 s/次。

### ON 三次合计

| 直接原因 | 累计时间(s) | 占 ON visibility-loss time |
|---|---:|---:|
| STATIC_LOS_BLOCKED | 29.906 | 38.94% |
| HFOV_VIOLATION | 33.103 | 43.10% |
| DYNAMIC_LOS_BLOCKED | 8.087 | 10.53% |
| VFOV_VIOLATION | 5.133 | 6.68% |
| RANGE_INVALID | 0.570 | 0.74% |
| UNKNOWN | 0 | 0 |

按 UAV：

| UAV | Static LOS(s) | Dynamic LOS(s) | HFOV(s) | VFOV(s) | Range(s) | 主要第三路丢失 |
|---|---:|---:|---:|---:|---:|---|
| UAV1 | 22.800 | 2.567 | 0.541 | 2.533 | 0 | 是，尤其 static LOS |
| UAV2 | 3.135 | 3.483 | 32.467 | 1.938 | 0.503 | 是，尤其 HFOV |
| UAV3 | 3.971 | 2.037 | 0.095 | 0.662 | 0.067 | 较少 |

仅统计 `visible_count==2` 的时长，ON 三次合计由 UAV1/UAV2/UAV3 作为唯一不可见第三架的时长分别约 24.87/38.80/5.86 s；因此最常成为“第三架丢失”的是 UAV2，其次 UAV1。UAV2 的长段 HFOV 违例集中在约 59–80 s；UAV1 static LOS 事件横跨约 7–74 s，且在后半段明显增多。

按轨迹时间与最近轨迹点对齐，主要空间段为：UAV2 约 `x=20–35 m, y≈-2…+3 m` 的后半程；UAV1 约 `x=-26…+29 m, y≈-1…+3 m`；UAV3 的损失主要在 `x≈4…30 m`。这些区域与森林静态遮挡段和目标运动后段重合。CSV 没有逐样本 obstacle-id，因此不能把每个 static LOS 样本伪造为某一根具体圆柱。

OFF 的同样分类总 loss 仅约 22.222 s：static LOS 11.861 s、dynamic LOS 8.293 s、HFOV 0.104 s、VFOV 1.964 s。最显著的结构性差异是 ON 的 HFOV loss 从 0.104 s 增至 33.103 s，且 static LOS 也从 11.861 s 增至 29.906 s。

## 2. 120° 合围、候选空间和 selector

源码链为：

`cooperative_viewpoint_manager.cpp` 发布 common `phi0` hypotheses → `ego_replan_fsm.cpp` 接收 hypothesis bundle → `planner_manager.cpp` 生成 NOMINAL/SIDE candidates → `topology_coordinator_core.cpp` 只组合同 hypothesis/generation 的三机 candidate → `team_visibility_optimizer.cpp` 做 joint P/T/yaw。

120° 几何在 `poly_traj_optimizer.cpp` 的 elastic tracking 中实现：径向 band、angular deadband、height tracking；`encirclement_base_angular_slack_deg=20°`，risk 触发时通过 `elasticAngularSlack` 与 `elastic_tracking_min_weight_scale` 弹性缩放。该项是 soft cost，不是硬等角约束，但每架 UAV 仍围绕同一 slot reference 被恢复。

实际日志显示：

- `encirclement_spread_saturation_deg=60`、preferred separation=25°、team K=2；
- 多个时段 `encirclement-hypothesis-candidates ... candidate_count=1`；
- 大量 `selection_reason=NO_VALID_VISIBILITY_ALTERNATIVE`；
- 多次 `SOURCE_CANDIDATE_NOT_ABSOLUTE_SAFE`、`INITIAL_DYNAMIC_CLEARANCE_VIOLATION`、`ACTIVATION_RESERVE_EXHAUSTED`；
- 因而不是单纯“27 组合中选错”，而是很多时段在组合前就只剩 NOMINAL，甚至没有可执行 candidate。

因此对问题二的判断是：

1. 120° reference 将 UAV 保持在固定 radial/angular slot；在 Scenario A 后半段，UAV2 slot 主要落入 HFOV 不利区，UAV1 slot 多次落入静态 LOS 不利区。
2. ±15° phi0 hypotheses 存在，但 coordinator 要求三机同 hypothesis/generation；且 sample-and-hold 只有在 K2/mean-visible/all3 达到门槛时才切换，不能自由寻找每架 UAV 的独立最佳扇区。
3. 固定 slot 确实限制了 UAV 交换观察扇区；代码没有 independent angular offsets `δ_i`。
4. `NO_VALID_VISIBILITY_ALTERNATIVE` 与 `candidate_count=1` 证明“好候选缺失”在多次 loss 时段发生。
5. 在有可执行候选时，selector 并非盲选：`topology_coordinator_core.cpp::better()` 先经安全层，再用 near-best K2 集合，集合内按 `mean_visible_count → all3 → none → spread`。所以既存在候选缺失，也存在 K2 优先级改变全局选择的作用。

## 3. objective 与 gradient 优先级

`team_visibility_optimizer.cpp` 的实际公式为：

```text
q2 = v0*v1 + v0*v2 + v1*v2 - 2*v0*v1*v2
b_k2 = 1-q2
b_blackout = (1-v0)(1-v1)(1-v2)
J_acc = mean(1-(v0+v1+v2)/3)
J_K2-cont = slidingWindowSquaredCost(b_k2, width)
J_blackout = slidingWindowSquaredCost(b_blackout, width)
objective = 2.0*J_K2 + 0.5*J_acc + 2.0*J_K2-cont + 0.5*J_blackout
            + 1.0*J_deviation + 1e-5*J_jerk + yaw prior
```

`J_acc` 优化的是 continuous `v_i`，不是 executed binary `V_i`；binary trace只用于 planned longest K2/blackout 诊断。梯度中 `g_acc` 每个 UAV 系数为 `-1/(3M)`，其权重只有 0.5。

日志中的成功 joint event（on_3）直接给出：

```text
J_acc before=0.319597870 after=0.319175082
weighted J_acc=0.159798935 → 0.159587541
grad norm J_acc=0.011938174 → 0.012366294
J_K2/K2-cont/blackout 全为 0
mean_visible=2.041206 → 2.042475
P_CHANGE_NORM=0.002684434, T_CHANGE_NORM=0.001076937,
YAW_CHANGE_NORM=0.009839406
```

这说明 joint event 中累计连续可见项确实在改善，但数量级只是总 objective 的一个软项；它没有保证全场景 executed camera-time 上升。

可用 gradient 日志还显示两个优化层的尺度严重不同。local MINCO 的 `tracking_grad` 在三次 ON 的已记录样本中中位数约 536–648，最大约 6.0e3–1.43e4；local directional `J_vis` 的 sample-gradient mean 中位数约 0.26–1.77（高风险样本可很大）。成功 joint event 的 weighted `GRAD_NORM_ACC` 仅 0.01194，而 K2/K2-cont/blackout gradient 当时均为 0。它们并非完全相同的变量空间和采样集合，不能作严格同维比值，但足以证明 local encirclement/tracking restoring action 在实际规划中远强于成功 joint step 的 accumulated-visibility correction。当前日志没有单独拆出“120° angular tracking gradient”与 radial/height tracking gradient，故不能伪造该细分数值。

`measurableBenefit()` 使用 OR 语义：K2、K2-cont、blackout、mean-visible、或 total objective 任一达到门槛即可；acceptance guard 只检查 hard safety、K2 tolerance=0.02、longest K2 loss tolerance=0.20 s、blackout tolerance=0.20 s。没有 All3 或 accumulated visible-time guard。因此“mean_visible下降但 K2/continuity/total objective改善”的轨迹在逻辑上可以被接受。

结论：系统实际上是综合优化，累计可见时间只是 continuous secondary term；不是严格的全场景 accumulated-camera-time maximizer。`IS_J_ACC_TOO_WEAK=YES`（相对 K2、deviation、tracking/encirclement 与 selector 语义而言）；`IS_K2_PRIORITY_SUPPRESSING_MEAN_VISIBLE=PARTIAL/YES`（selector 明确先保护 near-best K2，且 acceptance 允许小 K2 退化）。

## 4. continuous surrogate、prediction/execution gap

visibility CSV 同时记录 planned 与 executed binary：

- ON 的 planned/executed mismatch rate：UAV1 约 0.8–1.7%，UAV2 约 0.2–2.1%，UAV3 约 0–0.6%；team mismatch 约 3.3%（on_1 summary）。
- on_1 UAV2 executed visibility ratio 0.692，而 planned ratio 0.707；on_3 UAV1 planned 0.867、executed 0.857。

因此存在 prediction/execution gap，但它解释不了约 8% 的主损失；主损失是 planned/executed 两者共同存在的 HFOV/static-LOS 下降。`team_visibility_optimizer.cpp` 在 `rebuild_yaw_after_pt`/Stage3A 路径上使用当前 P/T 重建 target-facing yaw，且 Stage3B 成功日志出现 `OPTIMIZED_YAW_VALID=1`、`OPTIMIZED_YAW_EXECUTED=1`。所以应判定：

```text
IS_CONTINUOUS_SURROGATE_MISMATCH_PRESENT: YES (小到中等)
IS_PREDICTION_EXECUTION_GAP_SIGNIFICANT: NO (存在，但非 8% 主因)
```

## 5. 安全回归审计

`feedback_24.md` 的完整 executed 统计：OFF collision samples=0；ON 平均 10.667，emergency stops 平均 7.333；ON 平均 minimum dynamic clearance=0.132947 m、minimum static clearance=0.115539 m，且 on_3 static clearance 触及 0、on_1 dynamic clearance 触及 0。

launcher 日志将实际执行来源显示为 `planner_commit`、`encirclement_local_fallback`、`persistence_fallback`、`topology_candidate_selection`、`traj_server_receive`。碰撞相关片段常伴随 `SIDE_PLUS/MINUS` 的 `STATIC_COLLISION`、`dynamic_valid=0`、retiming/dynamics-check FAIL，随后 `KEEP_PREVIOUS_SAFE` 或 persistence fallback。因此安全回归不是由某个已通过完整 safety validation 的 joint solution 单独证明，而是 ON 的 encirclement/side candidate 生成、retiming、fallback/hold 链条带来的执行轨迹问题。

`OFF_1 min_swarm_distance=0.151147 m` 但 collision_samples=0，说明这两个指标不是同一判据：min swarm distance 是轨迹样本的几何最小值，collision_samples 是 executed obstacle/inter-UAV collision 事件计数，不能互相替代。

## 6. 根因排序

### ROOT_CAUSE_1 — CONFIRMED

ON 使 UAV2 大段进入 HFOV 不利状态、UAV1 大段进入 static LOS 不利状态。证据：ON HFOV loss 33.103 s、static LOS 29.906 s，对比 OFF 的 0.104 s、11.861 s；UAV2 单独作为第三架丢失约 38.80 s。on_1/on_2/on_3 CSV 逐样本字段一致。对应 `poly_traj_optimizer.cpp` 的 encirclement radial/angular/height restoring cost 与 fixed slot reference。结果是大量 All3→2-visible，直接造成 mean_visible 和 accumulated camera-time下降。

### ROOT_CAUSE_2 — CONFIRMED

高可见 candidate 经常在 team selection 前就缺失。证据：日志中的 `candidate_count=1`、`NO_VALID_VISIBILITY_ALTERNATIVE`、`SOURCE_CANDIDATE_NOT_ABSOLUTE_SAFE`、`INITIAL_DYNAMIC_CLEARANCE_VIOLATION`、`ACTIVATION_RESERVE_EXHAUSTED`。对应 `planner_manager.cpp` candidate generation/filter 与 `topology_coordinator_core.cpp` 同 hypothesis/generation 组合要求。后续 selector 无法从不存在的 candidate 恢复 All3。

### ROOT_CAUSE_3 — CONFIRMED

selector 的优先级是 safety → near-best K2 → mean_visible → All3，而非累计 camera-time 全局最大化。对应 `team_visibility_preference.h` 与 `topology_coordinator_core.cpp::better/select`。这解释了 ON K2 从 0.982248 升至 0.984523、longest K2 loss 从 0.877114 s 降至 0.753568 s，却把 All3 从 0.923316 降至 0.687902。

### ROOT_CAUSE_4 — CONFIRMED

joint acceptance 没有 accumulated-visible/All3 guard。`measurableBenefit()` 的 OR 语义允许 continuity 或 total objective 改善覆盖 camera-time 回退；acceptance 仅保护 K2/最长 loss/blackout，并容许 K2 0.02 的退化。故它不是严格“最大累计可见时间”门控。

### ROOT_CAUSE_5 — POSSIBLE

ON safety/fallback/retiming 回归改变了实际执行轨迹，可能扩大 planned 与 executed 差异并造成额外不可见窗口。证据为 ON collision/emergency 非零及日志中的 side retiming/dynamics/static failure；但现有 CSV 没有 trajectory-source 字段、launcher 日志也没有与每个 collision sample 同时刻的唯一 source join key，因此不能把每个 collision sample 唯一归因到 NOMINAL/SIDE/joint/fallback 中某一类。

## 7. 对关键问题的明确回答

```text
IS_120_DEGREE_ENCIRCLEMENT_CAUSING_VISIBLE_TIME_LOSS: PARTIAL (有直接强相关证据，但与候选缺失/安全 fallback 共同作用)
IS_GOOD_VISIBILITY_CANDIDATE_MISSING_BEFORE_TEAM_SELECTION: YES
IS_J_ACC_TOO_WEAK: YES
IS_K2_PRIORITY_SUPPRESSING_MEAN_VISIBLE: YES/PARTIAL
IS_CONTINUOUS_SURROGATE_MISMATCH_PRESENT: YES
IS_PREDICTION_EXECUTION_GAP_SIGNIFICANT: NO（存在但不是主导）
IS_ACCEPTANCE_GATE_ALLOWING_VISIBLE_TIME_REGRESSION: YES
SAFETY_REGRESSION_ROOT_CAUSE: NOT FULLY RESOLVED；确认存在 static/dynamic executed-clearance 触底及 side retiming/fallback failure，但现有 CSV 缺少逐 collision trajectory-source join key，不能唯一归因
```

## 8. 最小修改方案（仅建议，本轮未实施）

1. 将 120° 从固定 soft slot 改成可变角度参考：保留 radial/safety prior，允许每架 UAV 有受限 `δ_i`，并以 visibility/safety 决定角度。
2. 保留 non-encirclement/fixed-reference candidate，和 120° candidate、adaptive-visibility candidate 一起进入统一安全过滤与团队竞争。
3. 在 selector 与 acceptance 中把 accumulated visible time/mean_visible 设为主目标；至少要求 executed/planned mean-visible 不回退超过小 tolerance，同时再用 K2/K2-cont/blackout 保护连续性。
4. K2/K2-cont/blackout 应作为 coverage protection constraints/lexicographic protection，而不是允许其轻微改善就覆盖大幅 camera-time 下降的总目标项。
5. 最可能恢复 All3/camera-time 的改动顺序：先允许非合围候选竞争，再允许 adaptive angular slots；随后提高 `J_acc` 与直接 camera-time guard，最后把 side/retiming safety failures 变成严格 reject/fallback，不让低 clearance 轨迹进入执行。

## 状态声明

```text
PRODUCTION_SOURCE_CHANGED: NO
SIMULATION_RUN: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```
