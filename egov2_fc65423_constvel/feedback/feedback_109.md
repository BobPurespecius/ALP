# Feedback 109 — 复用 M2 relay 的 K3/All3 修复与 FULL 核验

## 修改前接口确认

| 接口 | 当前 dirty worktree 中的既有入口 | 本轮接法 |
|---|---|---|
| Team event | Coordinator 的 `assessPredictiveRelay()`、`updateHandoffContract()` 与 reference schedule 发布 | 先判 M2；M2 健康时才调用 `assessK3Relay()`、`updateK3Contract()` |
| schedule | `TeamReferenceSchedule` 已有 contract、candidate、critical-window、snapshot 和 activation 字段；没有目标 cardinality 字段 | 只增加 `handoff_repair_k`，0/2/3 分别表示无 relay、M2、K3 |
| Planner 接收 | `EGOPlannerManager::teamReferenceScheduleCallback()` | 仍走同一 identity/CAS、latest-wins、single-flight、ACK 路径 |
| refinement snapshot | 既有 `TeamRefinementSnapshot` | 增加 `repair_k`，K3 只有指定目标机进入 SCP |
| SCP | `PolyTrajOptimizer::runTeamContractSCP()` → `runCandidateHardCorridorSCP()` | 复用 T/PT、P/T trust、required margin、hard preflight 与 SIDE/Local-SFC |
| exact margin | SCP 内的 `teamMarginAt()` → `visibilitySampleAt()`；Planner 的 `evaluateVisibilityMarginAt()` | 保持原 visibility geometry；K3 trace 也由 `visibilitySampleAt()` 的真实 binary 结果产生 |
| final authority | `RealizedTeamValidator::validate()`，基线为同 activation/horizon 的 committed Local triple | K3 单独加入临界 K3 增加、逐样本 K2 不退、全时域 K3 不减；原 M2 comparator 不变 |

## 实现

三个 Planner 在 `VisibilityForecastTrace` 发布同一世界时间网格上的真实 binary visibility；它由已有 `visibilitySampleAt()` 的 range、static LOS、dynamic LOS、FOV 判定给出，未用连续 margin 的符号替代。Coordinator 在共同有效网格上计算 `K2=I(Σ visible_i≥2)`、`K3=I(Σ visible_i=3)` 和 `M3=min_i m_i`。现有 M2 event 始终优先。M2 健康时，K3 选未来第一段 `K2=1,K3=0` 的连续窗口；三机中唯一不可见者即 target，若身份变化便截断窗口。`M3` 仅用于风险和 SCP 梯度，不是最终采用标准。

K3 schedule 只声明一个 repair candidate。目标 Planner 复用现有 T/PT SCP、非零修改和 exact nonlinear margin/hard-safety 判断；两架 peer 不进入 SCP，也不再做无关的 Local MINCO 优化，而保留绑定的 Local topology seed，经原 hard preflight 后参与三机实现与最终校验。原 Local guide reserve 仍只由声明的 candidate 使用；没有在 Local L-BFGS 中新增完整 visibility evaluator。`TeamTargetCenteredOptimizer` 的 Q2、accumulated visibility、complementarity、encirclement hierarchy 均未修改。

`RealizedTeamValidator` 在原共同世界时间样本上计 baseline/realized binary visible count。K3 proposal 必须同时满足：临界窗口 K3 样本严格增加、全时域逐样本 K2 不下降、全时域 K3 总样本不下降，以及原有 identity、SIDE、hard safety、swarm、visibility-first 字典序条件。只有通过后才进入既有 PREPARE → 3 EXECUTOR_READY → COMMIT → common activation 事务。开关为 canonical runner 的 `--k3-repair on|off`，传给唯一 `/alp/team_k3_repair_enabled` ROS 参数；默认 OFF。

修改范围：`VisibilityForecastTrace.msg`、`TeamReferenceSchedule.msg`、Planner manager、poly traj optimizer、Coordinator、RealizedTeamValidator、canonical 参数/runner/launch 和 executed metrics 脚本。未增加 solver、事务状态机或安全/trust 参数。

## 构建与运行证据

最终改动执行 `git diff --check` 通过；`catkin build -j2 --no-status` 成功，25/25 包通过，包含 `traj_utils`、`traj_opt`、`ego_planner`、`multi_uav_formation`，消息 producer/consumer 均重编译。所有运行均由 `./run_on.sh --ablation full --headless` 进入，BOOT-12、`FINAL_EXIT_CODE=0`、runner cleanup OK。主配对场景为 `natural_team_stress_dense_38_targeted_k2_v6.json`，命令除 `--k3-repair` 外相同，`--timeout 240 --boot-timeout 180`。可见性指标均从 executed visibility CSV 重算，安全指标从 executed trajectory CSV 重算；每个 run 的 `k3_native_metrics.json` 保留统计。

| 用途 | RUN_ID | K3 | 结果 |
|---|---|---|---|
| 主配对 OFF | `20260925_003337_46046` | OFF | BOOT-12 / exit 0 |
| 主配对 ON | `20260925_002827_35055` | ON | BOOT-12 / exit 0 |
| 辅助 stress ON，最终代码 | `20260925_002351_27413` | ON | BOOT-12 / exit 0 |
| 因果诊断 stress ON，peer 收紧前 | `20260925_001704_18368` | ON | BOOT-12 / exit 0；仅用于分析 validator 拒绝 |

主配对证据：[OFF exit](../runs/20260925_003337_46046/exit_status.txt)、[OFF executed metrics](../runs/20260925_003337_46046/k3_native_metrics.json)、[ON exit](../runs/20260925_002827_35055/exit_status.txt)、[ON executed metrics](../runs/20260925_002827_35055/k3_native_metrics.json)。辅助证据：[最终 stress executed metrics](../runs/20260925_002351_27413/k3_native_metrics.json)、[诊断 stress 日志](../runs/20260925_001704_18368/runner_console.log)。每个 run 另有 `resolved_params.txt`、`run_manifest.txt`、`process_status.txt` 与原始 CSV。

## K3 event 与第一因果失败

主配对最终 ON 共 9 个 K3 event，7 次目标机 PT trial，0 次 solver PASS、0 次 validator PASS、0 次 adopted/activated。7 次 trial 均为 `TRUE_CONSTRAINT_INFEASIBILITY`，没有通过 hard margin/SCP 的 proposal 可以交给最终 validator。T-only 没有形成独立 T trial；原 AUTO 逻辑根据 gain bound 直接选 PT。event target 包括 UAV 0、1、2，说明并非固定目标或死分支。没有 K3 commit，执行 All3 的任何变化都不能归因于已采用 K3 repair。

辅助 stress 最终 ON 有 12 个 K3 event、13 次 PT trial，均无 solver PASS；11 次 `TRUE_CONSTRAINT_INFEASIBILITY`、2 次 `T_TRUST_TOO_SMALL`。两架 peer 保留绑定 Local seed 共 40 次。早一轮诊断 stress 在 contract 11 曾真实获得一次 PT solver PASS：target UAV0 的 exact margin `-1.942544→1.000000`，3 条 Local realization 全部 ACK；validator 测得 critical K3 `0→1`、whole-horizon K3 `65→63`、K2 regression `0`，以 `K3_TOTAL_VISIBILITY_WORSENED` 拒绝。故 solver PASS 本身不足以证明有效 K3 repair，拒绝点在用户要求的全时域 K3 非退化条件。该诊断没有 K3 PREPARE/COMMIT/activation。

主配对 ON event 逐事件证据（时间为仿真 world time）：

| contract | target | critical window (s) | PT trial | 首要结果 |
|---:|---:|---|---:|---|
| 2 | 0 | 1790267322.1–7322.6 | 0 | 无目标 trial |
| 4 | 0 | 1790267324.9–7325.9 | 0 | 无目标 trial |
| 5 | 0 | 1790267339.0–7339.1 | 2 | hard constraints infeasible |
| 7 | 0 | 1790267340.1–7340.3 | 1 | hard constraints infeasible |
| 8 | 2 | 1790267366.0–7366.6 | 0 | 无目标 trial |
| 9 | 0 | 1790267377.8–7377.9 | 3 | hard constraints infeasible |
| 10 | 0 | 1790267380.8–7381.7 | 0 | 无目标 trial |
| 11 | 2 | 1790267384.5–7384.9 | 1 | hard constraints infeasible |
| 13 | 1 | 1790267389.3–7389.8 | 0 | 无目标 trial |

## Executed OFF/ON 对照

| 指标 | OFF | ON |
|---|---:|---:|
| executed K2 | 0.988931 | 0.988931 |
| executed All3/K3 | 0.905917 | 0.918689 |
| mean visible UAVs | 2.894849 | 2.907620 |
| accumulated camera visible time | 226.566947 s | 227.552037 s |
| NONE | 0 | 0 |
| longest K2 loss | 0.867605 s | 0.699976 s |
| longest All3 loss | 1.338783 s | 1.399628 s |
| UAV1/2/3 visible ratio | 0.939123 / 0.985526 / 0.970200 | 0.946786 / 0.988506 / 0.972329 |
| static / dynamic contact samples | 0 / 0 | 0 / 0 |
| swarm violation samples | 0 | 0 |
| minimum static / dynamic clearance | 0.253850 / 0.069227 m | 0.213571 / 0.051725 m |
| minimum swarm separation | 1.032794 m | 1.270880 m |
| unvalidated executed samples | 0 | 0 |
| PVA mismatch / partial Team activation | 0 / 0 | 0 / 0 |
| successor starvation / terminal hold / end-before-next | 0 / 0 / 0 | 2 / 1 / 1 |
| MIN_BUDGET_APPLIED | 95 | 92 |

最终 stress ON executed K2=`1.000000`、All3=`0.913347`，min static/dynamic/swarm=`0.218001/0.574163/0.655343 m`，contacts、swarm violation、unvalidated samples、starvation、terminal hold 均为 0。其 12 个 K3 event 未产生 adoption；这些执行数值也不能作为 K3 已修复的因果证据。peer 收紧前的诊断 stress run 另有 8 个 dynamic contact 样本，且没有 K3 adoption；最终代码复跑为 0。该差异也提醒不能凭单次随机化运行宣称安全稳定性已经改善。

## 结论

K3 分支在 FULL 中真实触发，并穿过 schedule、目标 Planner 和现有 PT solver；final validator 已在一次真实 solver PASS 上证明会拒绝“修好 critical 点但损坏其他时段”的 proposal。最终主配对没有合法 K3 adoption，因此即使 ON 的 executed All3 高于 OFF `0.012771`，也不能确认 relay 式 K3 repair 的因果增益。OFF/ON executed K2 相同；hard contact、swarm violation、partial activation 都是 0，但 ON 的最小静态/动态净空较低，而且出现 OFF 没有的 2 次 successor starvation 与 1 次 terminal hold。第一 starvation 前目标机 K3 trial 已失败约 2 秒，日志显示直接触发为另一架机的动态碰撞预估使原轨迹失效；不能从这组 run 证明由 K3 trial 直接造成，但验收中的活性“不回退”未满足。多轮 v6 运行有明显波动：此前同场景 OFF 曾有 3 次 terminal hold，ON 曾为 0，故单轮配对也不足以估计稳定的因果差值。

当前 K3 第一阻塞是现有 trust/hard constraints 下的 PT 可行性；出现 PT PASS 时，全时域 K3 非退化仍是独立门槛。此时不建议把 Q3 加入 `TeamTargetCenteredOptimizer` hierarchy；先定位可修的浅 K3 事件及全时域损失来源，并复测 Local 活性，同时保持既有安全和 trust 不变。

## 机器可读摘要

```text
PRODUCTION_CODE_CHANGED: YES
BUILD_PASS: YES (25/25)
FULL_OFF_RUN_COMPLETED: YES
FULL_ON_RUN_COMPLETED: YES
K3_REPAIR_IMPLEMENTED: YES
K3_USES_EXISTING_M2_RELAY: YES
K3_USES_EXISTING_TEAM_T_PT: YES
K3_NEW_SOLVER_ADDED: NO
OLD_TEAM_VISIBILITY_OPTIMIZER_RESTORED: NO
K3_REPAIR_EVENTS: 9
K3_TRIALS: 7
K3_T_TRIALS: 0
K3_PT_TRIALS: 7
K3_SOLVER_PASS: 0
K3_T_SUCCESS: 0
K3_PT_SUCCESS: 0
K3_VALIDATOR_PASS: 0
K3_ADOPTED: 0
K3_ACTIVATED: 0
K3_VALIDATOR_REJECT: 0 (final pair; one diagnostic stress rejection)
K3_CRITICAL_TIME_BEFORE: N/A final-code proposal (diagnostic 0 samples ≈ 0.00 s)
K3_CRITICAL_TIME_AFTER: N/A final-code proposal (diagnostic 1 sample ≈ 0.03 s)
K3_TOTAL_TIME_BEFORE/AFTER: N/A (diagnostic 65/63 samples)
EXECUTED_ALL3_OFF: 0.905917
EXECUTED_ALL3_ON: 0.918689
EXECUTED_K2_OFF: 0.988931
EXECUTED_K2_ON: 0.988931
K2_REGRESSION_SAMPLES_FROM_K3: 0 diagnostic validator; no final-code adoption
STATIC_CONTACT_SAMPLES_OFF/ON: 0/0
DYNAMIC_CONTACT_SAMPLES_OFF/ON: 0/0
SWARM_VIOLATION_OFF/ON: 0/0
PARTIAL_TEAM_ACTIVATION_OFF/ON: 0/0
UNVALIDATED_EXECUTED_SAMPLES_OFF/ON: 0/0
STARVATION_OFF/ON: 0/2
TERMINAL_HOLD_OFF/ON: 0/1
END_BEFORE_NEXT_OFF/ON: 0/1
MIN_BUDGET_APPLIED_OFF/ON: 95/92
K3_GAIN_CONFIRMED: NO
K2_PROTECTED: YES (validator gate; no K3 adoption)
SAFETY_REGRESSION: NO (hard violations 0/0; minimum static/dynamic clearance lower ON)
LIVENESS_REGRESSION: YES (observed in final pair; causality unproven; run variability)
NEXT_STEP: Resolve PT infeasibility/whole-horizon K3 loss and repeat Local liveness A/B before considering Q3.
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_LEFT_RUNNING: NO
```
