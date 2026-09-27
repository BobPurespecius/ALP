# ALP Replanning Warm-Start Root Cause Review

审计范围：`/home/bob/ALP/egov2_fc65423_constvel` 当前源码与既有 launcher/rosout 日志。

本报告只做源码和离线日志审计；本轮未修改生产代码、未调参、未重新运行仿真。

## Executive conclusion

上一周期的安全 SIDE/SCP 轨迹在执行层面通常被保留，并且普通 local-replan 路径会从当前执行轨迹采样起点状态。但这不等于把上一周期 accepted MINCO 解作为下一周期的非线性 warm start。

下一周期仍会经 `computeInitState()` 重新采样 inner points、重新均匀分配 piece durations、重新施加 SIDE offset，并在 candidate/SCP 入口重新 `reset()`/`generate()`。因此高 jerk 初值的主要源码级根因是：

`POSITION_WARM_START_LOST + TIME_WARM_START_LOST`，并伴随 `TOPOLOGY_CONTINUITY_LOST`。

当前日志证明 `MODEL_ACCURATE_BUT_INFEASIBLE` 可能只是单步不可行，后续重新线性化可以恢复；不能据此证明 candidate 在现有 P/T 和硬约束下真正无解。

## 1. Candidate 31 final state

证据：`trajectory_persistence_full_run4_20260901.launcher.log`，约 lines 12863–12904。

- drone 0，`SIDE_PLUS`，obstacle 7。
- initial duration：约 `2.797799 s`。
- SCP final duration：约 `3.222658 s`。
- final max jerk：`19.257479`；max velocity/acceleration：`2.037354 / 3.215233`。
- 记录为 `SCP_FINAL_OK`。
- 随后的 `[candidate-acceptance]` 是 `accepted=0`，并且 `risk-candidate selected=NOMINAL`。

因此：

- `C31_SAFE_SELECTED = NOT CONFIRMED`；
- 不能把该事件描述为“C31 已被选中并作为下一周期安全轨迹”；
- 该日志只证明 C31 产生了 final-feasible 的 SCP 结果，但没有证明它成为执行选择。

## 2. Candidate 32 initial/final state

同一日志约 lines 12926–12989：

- drone 0，`SIDE_PLUS`，obstacle 7；与 C31 是时间上相邻的重建事件；
- initial duration：约 `2.779827 s`；
- 同一 `scp_iter=0` 有多次尝试；
- 最终 reason：`SPATIAL_TRUST_RETRY_EXHAUSTED`；
- final max jerk：`15.143421`，max velocity/acceleration：`3.015688 / 3.406243`；
- 与 C31 final 时间差约 `0.294 s`。

这说明相邻周期发生了重新初始化和不同的收敛结果，但不是用户背景中精确的“obstacle-9、C31 selected-safe → C32 failure”证据。对该精确叙述，现有日志不足。

## 3. P continuity

### Execution/start-state continuity

`ego_replan_fsm.cpp::planFromLocalTraj()`（约 line 676 起）在 local trajectory 有效时读取：

```cpp
info->traj.getPos(t_cur)
info->traj.getVel(t_cur)
info->traj.getAcc(t_cur)
```

并传给 `callReboundReplan(false, false)`。因此正常 local-replan 路径具有当前执行轨迹的 `p/v/a` 起点连续性。

### Accepted MINCO P inheritance

`planner_manager.cpp::computeInitState()` 的 previous-trajectory 分支不是复制上一 accepted polynomial 的 control points。它：

1. 根据当前时间计算 local trajectory 剩余时间；
2. 从 local/global trajectory 重新采样 inner points；
3. 依路径长度重新决定 `piece_nums`；
4. 用 `Constant(piece_nums, t_to_lc_tgt / piece_nums)` 重新生成均匀 durations；
5. SIDE candidate 再加 `side_bias * side_offset * sin(pi*u)`；
6. `initMJO.reset(...)` 后重新 `generate(innerPs, piece_dur_vec)`。

所以：

`EXECUTED_STATE_USED_AS_START = YES`（正常 local replan）

但：

`EXACT_ACCEPTED_P_INHERITANCE = NO`。

local trajectory 过期、tracking error 超阈值或回退到 global planning 时，还可能绕过当前执行轨迹起点，故 start continuity 不是所有路径都绝对保证。

## 4. T continuity

上一周期 accepted 的真实 durations/virtual-T 没有作为下一周期 candidate 的输入保存并平移。`computeInitState()` 重新均分 `piece_dur_vec`，之后 candidate 分支可能再次 retime；SCP 入口再执行：

```cpp
RealT2VirtualT(initT, current_virtual_t);
jerkOpt_.generate(current_points, current_durations);
```

因此：

- `EXACT_ACCEPTED_T_INHERITANCE = NO`；
- `TIME_WARM_START_LOST = YES`；
- 当前 candidate 内部仍然有 P/T continuation，但这是本 candidate 的 continuation，不是跨 replanning cycle 的 continuation。

## 5. Start and end boundary continuity

起点 `p/v/a`：

- 正常 local-replan：`PASS`（从当前执行轨迹采样）；
- expiry/tracking/global fallback：`PARTIAL`。

终点状态：`computeInitState()` 每周期显式设置：

```cpp
tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();
```

因此上一周期 accepted trajectory 的终端 velocity/acceleration 不会被继承，存在：

`TERMINAL_BOUNDARY_DISCONTINUITY = YES (risk)`。

旧日志没有同一 candidate 的完整 terminal PVA 和 polynomial coefficients，不能把某个 jerk 峰值唯一归因到该边界。

## 6. Topology continuity

SIDE candidate 每周期重新尝试 PLUS/MINUS、offset/backoff、A*、Local-SFC、retiming；`piece_nums` 也可根据当前距离重新计算。因而以下状态没有跨周期继承保证：

- SIDE branch（PLUS/MINUS）；
- A* waypoint/turning-point set；
- Local-SFC plane set and active interval；
- piece count and segment partition；
- retimed duration allocation。

结论：`TOPOLOGY_CONTINUITY = PARTIAL/LOST`。

## 7. MINCO reset path

`traj_opt/include/optimizer/poly_traj_utils.hpp` 中 `MinJerkOpt::reset(headState, tailState, pieceNum)` 会重新设置 `N/head/tail` 并 resize `T1/A/b/gdC`；`generate(inPs, ts)` 重新计算全部 polynomial coefficients。该类没有“shift previous accepted trajectory”或“inherit previous P/T”接口。

赋值 operator 只复制内部矩阵/系数，不构成上层跨周期 warm-start protocol。换言之：

`MINCO_RESET_REINITIALIZES_P = YES`

`MINCO_RESET_REINITIALIZES_T = YES`

这本身是正常的 candidate 构建机制；真正的缺口是上层没有传入上一 accepted 解。

## 8. SCP initialization

`traj_opt/src/poly_traj_optimizer.cpp::runCandidateHardCorridorSCP()`（约 line 194 起）当前 candidate 内使用：

- `position_dim = 3 * (piece_num - 1)`；
- `virtual_time_dim = piece_num`；
- `variable_num = position_dim + virtual_time_dim`；
- `current_points = initInnerPts`；
- `current_virtual_t = RealT2VirtualT(initT)`；
- 每次 major iteration 更新 `current_points/current_virtual_t` 后继续。

这说明当前 SIDE-SCP 是 free-time P/T SCP，而不是仅优化 P 的 fixed-time 分支。但 `initInnerPts/initT` 本身每周期来自重建，不来自上一 accepted MINCO state。

## 9. Native MINCO capability versus current SIDE-SCP

| 状态项 | 结论 |
|---|---|
| Execution continuity | YES（正常 local replan） |
| Start P/V/A continuity | PARTIAL/PASS on local replan |
| Exact accepted P inheritance | NO |
| Exact accepted T inheritance | NO |
| Free-time in current SCP | YES |
| Current-cycle P/T continuation | YES |
| Cross-cycle nonlinear warm start | NO |
| Topology continuity | PARTIAL/NO |
| Native objective/cost retained in SCP | YES（以 local gradient model 接入） |
| OSQP cross-solve warm start | NO |
| True candidate infeasibility proven | NO |

原生 MINCO 的 `costFunctionCallback()` 将 x 解析为 `[P, virtual_T]`，调用 `VirtualT2RealT()`、`jerkOpt_.generate()`、native cost/gradient、`getGrad2TP()` 与 `VirtualTGradCost()`；因此当前 SCP 并未把 gradT 或 time cost 从 candidate 内部完全删除。当前 QP Hessian 是对角/信赖域局部模型，不等同于 L-BFGS 的完整非线性 Hessian，但这是 SCP 近似形式，而不是跨周期 warm-start。

## 10. OSQP warm start versus trajectory warm start

`traj_opt/src/scp_optimizer.cpp` 设置了 `settings.warm_start = 1`，但每次 `SCPOptimizer::solve()` 都重新创建 `OSQPData`、调用 `osqp_setup()`，求解后 `osqp_cleanup()`。

因此：

- OSQP workspace 内的默认 warm-start 开关：`ON`；
- 跨 solve/candidate/cycle 的 primal/dual solution persistence：`NO`；
- 不能把它当作上一周期 trajectory/MINCO warm start。

## 11. Jerk 345 / high-jerk root cause

既有日志显示大量高 jerk 出现在新建 polynomial 的 initial stage，而不是上一 accepted polynomial 的 shifted continuation：

- `phaseA_smoke_20260830.launcher.log`：initial jerk 约 `83、129、840、49,297`；
- `trajectory_persistence_smoke3_20260901.launcher.log`：initial jerk 约 `120、479、5,904`；
- `trajectory_persistence_run3_20260901.launcher.log`：initial jerk 约 `1,115、1,431、21,079`；
- `trajectory_persistence_final_smoke_20260901.launcher.log`：initial jerk 约 `11,522–15,451`，后续 actual jerk 可降至约 `303、124`。

这些运行共同支持“短时长 + 重新均分 T + 重新生成 P/T/topology”与高 jerk seed 强相关。现有数据缺少完整 polynomial coefficients、上一周期 accepted P/T、峰值所在 piece 以及 terminal-PVA 对照，因此：

`JERK_345_SEGMENT_LEVEL_CAUSALITY = NOT PROVEN`。

不能把 345（或其他峰值）唯一归因给单一 piece、单一边界或单一 trust policy。

## 12. Accurate-but-infeasible is not automatically candidate infeasible

### Direct recovery evidence

`trust_agreement_main_20260831_115424.launcher.log` 的 candidate 2：

- iter 0：`MODEL_ACCURATE_BUT_INFEASIBLE`，actual jerk `20.152075`，predicted `19.977652`，`trust_decision=T_GROW`，intermediate trial accepted；
- 随后进入 iter 1，重新线性化并再次尝试；
- 最终 `SCP_FINAL_OK`，final jerk `9.730561`。

`trust_cleanup_main_20260831.launcher.log` 的 candidate 12：

- iter 0 actual jerk `23.854953`；
- iter 1/2/3 仍是 accurate-but-infeasible，且 intermediate accept 后 `next_action=RELINEARIZE`；
- iter 4 actual jerk `14.295607`；
- 最终 `SCP_FINAL_OK`。

这两个案例直接证明：

`MODEL_ACCURATE_BUT_INFEASIBLE` 可以只是当前 step/iterate 的不可行状态，继续 relinearization 仍可能找到可行方向。

### Constraint pattern

accurate-but-infeasible 多数是 jerk 轻微超限（约 `20.02–21.9`，也有 `23–28`）；velocity 偶尔轻微超过 3.0，acceleration 通常低于 6.0。更高的 `30+`、数百或数千 jerk 多伴随 model mismatch 或 initial-stage bad seed，不应混同为 accurate-but-infeasible。

因此当前日志足以否定“单次 accurate-but-infeasible = candidate 真无解”，但不足以对每个最终失败 candidate 给出数学意义上的局部不可行证明。

## 13. Offline previous-safe warm-start counterfactual

未执行离线 counterfactual（没有把 C31 的完整 accepted P/T/topology 注入 C32 再求解）。状态：`INCONCLUSIVE`。

因此只能给源码级判断，不能给实验性因果效应量：

- 若直接 shift accepted P/T 并保持对应 topology，理论上可避免当前的“均匀重建”跳变；
- 但具体 jerk、收敛和 candidate selection 改善幅度未被现有数据测量；
- 不应把该 counterfactual 的预期效果写成已验证结果。

## 14. Source-level root cause classification

### Primary

`POSITION_WARM_START_LOST`：上一 accepted MINCO 的 inner control points 未跨周期传递，下一周期从执行轨迹采样并重新施加 SIDE geometry。

`TIME_WARM_START_LOST`：上一 accepted real durations/virtual-T 未跨周期传递，下一周期重新均分并可能 retime。

### Secondary

`TOPOLOGY_CONTINUITY_LOST`：SIDE PLUS/MINUS、A*/Local-SFC guide、piece count 和 active intervals 每周期可重建。

`TERMINAL_BOUNDARY_DISCONTINUITY`：终端 PVA 以 local target 和零加速度重设，而非继承上一 accepted terminal state。

### Not the primary root cause

- 不是“当前 SCP 没有 free-time”：当前 SCP 已有 P/T 和 gradT 路径；
- 不是“OSQP 完全没有 warm_start”：单个新建 workspace 设置了 `warm_start=1`，只是无跨 solve 持久化；
- 不是已被证明的 candidate 数学真不可行：现有日志没有这种证明。

## 15. Final answers

1. **这些 accurate-but-infeasible 是 step 问题还是 candidate 真不可行？**

   至少一部分明确是 step-level infeasibility；candidate 2 和 12 在后续重新线性化后恢复。对最终失败者，现有日志不足以证明 true local infeasibility。

2. **jerk 是否仍为主因？**

   是当前 observed accurate-but-infeasible 的主要活跃限制；通常是接近 20 的小幅 jerk 超限，伴随少量 velocity 超限，acceleration 通常不是主因。

3. **多轮重新线性化是否能救回来？**

   能。已有 candidate 2、12 的明确恢复证据。

4. **当前 planner 是否错误地过早终止这类 candidate？**

   不能对全部 candidate 下定论。现有代码已对部分 accurate-but-infeasible 执行多轮 relinearization；但当有限 trust retry/iteration 耗尽时会终止，日志无法证明再继续一定可行。因此只能说存在“有限 continuation 终止”的风险，不能断言实现必然错误。

5. **是否有明确证据表明 long_cylinder_forest 某些 candidate 真无动力学可行解？**

   没有。当前结论必须是：

   **NO EVIDENCE OF TRUE CANDIDATE INFEASIBILITY**。

## 16. Minimal next-step recommendation (not implemented)

下一步若要恢复能力，最小范围应是建立跨周期 accepted-candidate state（P、real/virtual T、piece topology、SIDE/SFC context）的安全 shift/inheritance，并保留当前执行状态作为起点校正；不要把 `KEEP_PREVIOUS_SAFE` fallback 误当作 MINCO warm start。该建议本轮未实现，也未通过 counterfactual 验证。

## Audit status

- `PRODUCTION_CODE_CHANGED: NO`
- `SIMULATION_RERUN: NO`
- `EXACT_C31_SELECTED_SAFE_TO_C32_FAILURE: NOT ESTABLISHED`
- `MODEL_ACCURATE_BUT_INFEASIBLE == TRUE_CANDIDATE_INFEASIBILITY`: **FALSE as a general rule**
- `TRUE_CANDIDATE_INFEASIBILITY_PROVEN`: **NO**

