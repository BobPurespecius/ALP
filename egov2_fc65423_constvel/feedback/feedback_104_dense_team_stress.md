# Feedback 104 — 加密自然森林 Team stress 场景

## 目标与实现

按“让 Team 层在真实运行中产生规划活动，并争取 PT/T PASS 或 ADOPTED”筛选。本轮只扩展已有森林生成器的 natural 模式，没有修改 planner、安全条件、场景格式或 runner。

新增 `--extra-static-count`：补充树的位置从原森林的 x 范围均匀采样、从原树的 y 行分布抽样并轻微抖动；半径和高度沿用原森林取值，树间距至少 1.55 m。树位不读取 target 路线。原有 10 个动态障碍保留，只按 seed 改相位。

最终主候选由 36 棵原始静态树加 2 棵随机补充树构成，共 38 棵、10 个动态障碍。相对上一版 33 棵增加 5 棵（15.2%）；相对未稀疏的原森林增加 2 棵（5.6%）。路线仍由已有 seeded S 路径生成器产生。

## 搜索与 FULL 结果

| 候选 | 搜索结果 | FULL 结果 | 处理 |
|---|---|---|---|
| 40 棵静态树 | 10,000 个组合通过净空预筛数为 0 | 未运行 | 密度过高，淘汰 |
| 39 棵静态树 | 12,000 个组合仅 1 个通过；proxy M2 最低 −3.441，另有 +0.044 crossing | 未运行 | 深遮挡占主导，淘汰 |
| **38 棵，seed 20402929** | 15,000 个组合有 4 个通过；选中 1 个静态 limiter proxy crossing，最低 M2 −0.104；target 静态额外净空 0.0976 m，动态净空 1.023 m | **运行时 2 个 Team contract，PT 计数到 128；10 次 relay 终态均 `SOLVER_INFEASIBLE`；T/PT PASS 0、ADOPTED 0。** | **保留作 Team trigger/失败路径复现** |
| 38 棵，路线 seed 20406421、相位 seed 22412066 | 固定路线和树位扫描 2,500 组动态相位；180 组有动态 limiter proxy crossing。所选最低 M2 +0.093，静态净空 0.338 m，动态 target 净空 0.425 m | 无 runtime contract，runtime M2 未低于 trigger；同时出现 15 个 moving-clearance 非正样本、77 次 `MIN_BUDGET_APPLIED` 和 1 次 successor starvation | 淘汰，不作为推荐场景 |

主候选生成文件：[natural_team_stress_dense.json](../ros_ws/src/multi_uav_formation/scenes/natural_team_stress_dense.json)。静态路线校验为 26 个 waypoint、0 个相交，最小 target 额外净空 0.097619 m。布局图：[natural_team_stress_dense_layout.png](../runs/natural_team_stress_dense_layout.png)。

主候选 FULL run 为 `20260923_182315_592619`：BOOT-12 和三架 UAV 运动均通过，`FINAL_EXIT_CODE=0`，cleanup 正常；runner 按既定 240 s 上限结束核心进程。整段 78.20 s 轨迹的可见率为 UAV1/2/3 `95.44% / 100% / 100%`，三机同时可见率 95.44%，最长 K2 loss 和 blackout 均为 0。全段实测 static/moving clearance 最小值为 0.274914 / 0.369550 m，非正 clearance 样本为 0；`MIN_BUDGET_APPLIED`、terminal hold、successor starvation、PVA mismatch、partial activation、unvalidated execution 均为 0。

主候选运行时 limiter 是 `STATIC`。`TEAM_SCP_MODE=PT` 的输入中可见 deficit 约 0.788 和 0.059，但求解均未通过；另有更深的 static candidate deficit。10 条 `[RELAY_LATENCY]` 的 detect→receive 为 p50 30.287 ms、p95 321.696 ms、max 567.360 ms。场景 proxy 事件的时刻和 limiter 没有预测到 runtime 的两个 startup contract，因此不能把 proxy 结果视为 runtime 修复能力的证据。

相位变体 FULL run 为 `20260923_184455_597125`：同样达到 BOOT-12 且 runner 退出码为 0，但运行时没有 contract / relay / SCP 终态。可见率较高（全段三机同时可见 97.49%），moving clearance 最小为 0 m，并伴随 15 个非正样本、77 次 MIN_BUDGET 和 1 次 starvation；因此已淘汰。其场景保留为筛选证据：[natural_team_stress_dense_dynamic.json](../ros_ws/src/multi_uav_formation/scenes/natural_team_stress_dense_dynamic.json)。

## 结论

主候选达成了 runtime Team 风险触发与 refinement 活动，适合复现 contract、relay 和 PT infeasible 路径；未得到任何 T/PT PASS 或 ADOPTED 修复，不能称为可恢复 Team repair 通过样例。第二个相位候选的 LOS proxy 虽是动态且浅，但 runtime 没有触发，且出现 moving-clearance/liveness 回退，已排除。

```text
NEW_SCENE_FRAMEWORK_CREATED: NO
EXISTING_GENERATOR_REUSED: YES
EXISTING_ANALYSIS_REUSED: YES

OBSOLETE_SCENE_GATES_REMOVED: NONE
FIXED_DYNAMIC_OBSTACLE_COUNT_REQUIREMENT_REMOVED: NOT_PRESENT

SEEDS_SCREENED: 37,500 route/tree combinations across 40/39/38-tree batches;
               2,500 phase-only trials
FINAL_SCENE: /home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/scenes/natural_team_stress_dense.json
FULL_RUN_COMPLETED: YES
RUNTIME_TEAM_CONTRACTS: 2
TEAM_PT_ATTEMPT_COUNT: 128
TEAM_T_PASS: 0
TEAM_PT_PASS: 0
TEAM_REFINEMENT_ADOPTED: 0

SAFETY_REGRESSION: NO  # selected scene; alternate phase variant rejected for clearance/liveness
NATURAL_FOREST_STRUCTURE_PRESERVED: YES
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```

## 补充实验：固定路线/相位，静态树 38→44（+15.8%）

按 38 棵基线追加 6 棵，形成 44 棵静态树；保留原 38 棵树、26 个 waypoint、10 个动态障碍及所有动态相位。新增树使用 `extra_static_seed=26403752`。固定路线下筛了 2,500 个新增树 seed：989 个通过几何筛选且保留 proxy crossing，286 个相对 38 棵基线产生至少 0.05 的 proxy 降低；选取较浅的一组作 FULL。一个更早尝试的 seed 因目标路线净空变负而被筛掉，没有放宽门槛。

最终 44 棵候选的目标静态额外净空仍为 0.097619 m、动态目标净空 1.022809 m，路线验证 26 waypoint、0 相交。LOS proxy 有两个 STATIC crossing：`t=12.864 s, M2=-0.334603`（新增）和基线原有的 `t=70.296 s, M2=-0.104232`。新增场景为 [natural_team_stress_dense_44.json](../ros_ws/src/multi_uav_formation/scenes/natural_team_stress_dense_44.json)，筛选记录为 [natural_team_stress_dense_44_generation.json](../runs/natural_team_stress_dense_44_generation.json)。

正式运行：

```bash
./run_on.sh --ablation full --headless --timeout 240 \
  --scenario /home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/multi_uav_formation/scenes/natural_team_stress_dense_44.json
```

run `20260923_202423_602883` 正常到达 BOOT-12，`FINAL_EXIT_CODE=0`。完整 [runner 日志](../runs/20260923_202423_602883/runner_console.log)、[退出状态](../runs/20260923_202423_602883/exit_status.txt) 和 [轨迹/可见性复算](../runs/20260923_202423_602883/native_metrics_full.json) 已落盘。与 38 棵 FULL 主样本比较：

| 指标 | 38 棵基线 `182315` | 44 棵 `202423` |
|---|---:|---:|
| Runtime Team contracts / 最大 PT attempts | 2 / 128 | 2 / 128 |
| PT modes / relay terminal records | 10 / 10 | 9 / 9 |
| PT PASS / T PASS / ADOPTED | 0 / 0 / 0 | 0 / 0 / 0 |
| runtime limiter | 全部 STATIC | 全部 STATIC |
| 最低静态 / 动态轨迹净空 (m) | 0.274914 / 0.369550 | 0.299426 / 0.474390 |
| 非正净空、MIN_BUDGET、terminal hold、successor starvation | 0 / 0 / 0 / 0 | 0 / 0 / 0 / 0 |
| 三机同时可见率 | 95.44% | 94.81% |
| K2（至少 2 架可见） | 100% | 100% |
| detect→receive p50/p95/max (ms) | 30.287 / 321.696 / 567.360 (n=10) | 284.105 / 312.975 / 1111.635 (n=9) |
| `MOVING_ROLLING_RETRY` 原始日志行数 | 9,574 | 10,070 |

44 棵 run 的两个 contract 仍全由 STATIC 限制。浅 deficit 分别约 `0.7866` 与 `0.05391`，对应 `grad_T_norm=0`；其他 SCP 候选 deficit 为 `5.31–5.85` 和 `8.48`。9 次 Team refinement 终态均为 `SOLVER_INFEASIBLE`，另有 5 次 baseline-safe noop；没有 PT/T pass 或 ADOPTED。Team reserve 有 14 个 guide 批次、累计 11 个 guide 点，但没有转化为 Team 修复通过。

K2 另由逐样本 `visibility.csv` 复算：38 棵基线 `2347/2347=100%`，44 棵 `2349/2349=100%`。44 棵 run 中 122 个样本恰有 2 架可见、2227 个样本 3 架全可见；UAV2/UAV3 全程可见，遮挡只使 UAV1 在部分时刻不可见。因此 K2 loss/blackout 都是 0。该聚合指标没有失守，但独立的 Team M2 margin/forecast 路径仍产生了 2 个 contract；这次并非 Team trigger 没触发，而是触发后的 PT 修复没有通过。

**判断**：在路线、动态相位及原有 38 棵树固定后，加 6 棵确实让 proxy 新增一个早期静态遮挡 crossing；但 runtime contracts 与 PT attempt 数没有增加，SCP 结果仍未通过。此轮非正净空和明确活性故障计数仍为 0，可见率略降；`MOVING_ROLLING_RETRY` 原始行数约增加 5%，但没有对应 starvation/MIN_BUDGET，不能单独定性为活性回退。relay 延迟样本的中位数和最大值较高，但每组只有一次运行，不能据此把延迟恶化归因于密度。当前证据说明加密能改变 LOS 风险位置，不足以解决 STATIC limiter 下的 SCP 可行性/梯度问题。

```text
DENSITY_PLUS15_TEST_COMPLETED: YES  # 38 -> 44 static trees (+15.8%)
ROUTE_AND_DYNAMIC_PHASES_HELD_FIXED: YES
RUNTIME_TEAM_CONTRACTS: 2  # unchanged from 38-tree baseline
TEAM_PT_ATTEMPTS: 128  # unchanged
TEAM_PT_PASS: 0
TEAM_T_PASS: 0
TEAM_REFINEMENT_ADOPTED: 0
NONPOSITIVE_CLEARANCE: 0
MIN_BUDGET_APPLIED: 0
TERMINAL_HOLD_ENTER: 0
MOVING_SUCCESSOR_STARVATION: 0
```

## 复核：按实际 LOS 检查新增树是否有效

此前“+15.8%”只说明树数从 38 增至 44，不能说明新增树挡住了运行中的观测射线。收到关于新增障碍离目标较远的反馈后，使用另一组自然行分布 seed `26410610` 重新筛了 6 棵树；其中 5 棵到 target 路径的树表面净距不超过 2 m。该候选仍没有通过实际运行验证。

带 RViz 的 FULL run 为 `20260923_214955_653704`，场景是 [natural_team_stress_dense_44_actual_los.json](../ros_ws/src/multi_uav_formation/scenes/natural_team_stress_dense_44_actual_los.json)。RViz 确实由 roslaunch 启动，仿真到达 BOOT-12，runner 清理正常。把实际 UAV 轨迹与 target 轨迹按采样时刻对齐，并按场景圆柱和 0.08 m visibility margin 检查新增 6 棵树：**2348 个 visibility 时刻、3 架 UAV 的新增树 LOS 相交数为 0**。上一轮 seed 筛选曾基于不同 run `20260923_211317_618177` 预测 27 个反事实 K2-loss 样本；实际轨迹改变后，这个预测没有迁移成功。

| 指标 | 本次 RViz run |
|---|---:|
| K2（至少 2 架可见） | 2348/2348 = 100% |
| K3（三架均可见） | 2146/2348 = 91.40% |
| UAV1 / UAV2 / UAV3 visibility | 93.19% / 99.66% / 98.55% |
| 新增 6 棵树命中的实际 UAV→target LOS | 0 |
| 有效 `[TEAM_M2_MIN]` 样本 / 低于 trigger=0.30 | 168 / 3；最低 0.14309 |
| PT SCP start / PT fail / T 或 PT pass / adopted | 5 / 5 / 0 / 0 |
| `MIN_BUDGET_APPLIED` / successor starvation | 34 / 1 |

因此，K3 下降不能归因于这 6 棵新增树；可见率损失来自原有树、FOV 或动态 LOS。Team 有少量 margin crossing 和 PT 尝试，但 PT 均未通过。此 run 不能作为新增密度有效或 Team repair 成功的证据。

又用同一自然新增树生成器，对 1200 组每组增加 12 棵（总数 50）做实际轨迹反事实筛选。177 组通过 target 路径净空门槛，其中 11 组同时保持新增树到已执行 UAV 轨迹至少 0.20 m 净距；最好一组只预测 48 个单机 LOS 阻塞样本，K2-loss 仍为 0。该 50 棵候选已淘汰，未做 FULL，也未保留场景文件；筛选记录在 [natural_team_stress_dense_50_actual_track_search.json](../runs/natural_team_stress_dense_50_actual_track_search.json)。

为避免继续按树数挑选，又复用现有生成器分别搜索 20,000 个 target-route seed，树位和动态障碍数量保持不变：原 38 棵森林仅 9 个路线通过几何净空门槛，排前的多事件候选含多个较深 crossing；固定 44 棵布局只有 3 个路线通过，结果为深 crossing 或单个较深 crossing。另联合搜索 20,000 组路线/自然树 seed，只有 1 个过几何门槛，其单个 STATIC crossing 的 proxy 最低 M2 为 -1.489。筛选记录分别在 [natural_team_stress_route_seed_search_20k.json](../runs/natural_team_stress_route_seed_search_20k.json)、[natural_team_stress_dense_44_route_seed_search_20k.json](../runs/natural_team_stress_dense_44_route_seed_search_20k.json) 和 [natural_team_stress_joint_route_tree_search_20k.json](../runs/natural_team_stress_joint_route_tree_search_20k.json)。目前没有找到满足“多段浅、可恢复风险”的替代场景。

**修正结论**：用户关于“只加了很多无效障碍”的判断对本次 44 棵新增候选成立。树到 target 路线的距离只是粗筛条件；真实效果必须在运行轨迹上检查 UAV→target 视线，并验证运行后 K2/M2。当前 44 棵 run 的新增树 LOS 命中为零，故不应把它宣传为有效加密场景；50 棵和路线 seed 搜索也尚未产出合格替代方案。

## 后续：按实测 LOS 替换无效 gate

2026-09-24 的离线核对发现，旧 `targeted_los_gate_10` 在部分运行的 6204 个可见性样本中未拦截任何当时可见射线。已生成一个不增加障碍数量、直接沿末段 UAV1-target LOS 移动的候选，固定轨迹反事实新增 4.142 s K2 loss。结果和净空限制见 [Feedback 105](feedback_105_targeted_k2_gate_offline.md)。本轮没有仿真，候选尚未做 planner/Team 验证。
