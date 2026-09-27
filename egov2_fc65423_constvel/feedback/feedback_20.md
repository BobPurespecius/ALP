# Feedback 20 — Feasible SIDE Preservation + Time-Only Feasibility Validation

日期：2026-09-04

## 1. 结论摘要

本轮完成了用户限定的最小修复，并在 `long_cylinder_forest_visibility_stress.json` 上完成一轮有效的完整 ALP 运行。

核心结果：

- retiming 的准入判据现与 full SCP hard rows 共用同一套逐 piece `v/a/j` lattice；旧的 manager checker 只保留为 diagnostic。
- 新增固定 `P`、只优化 `delta_tau` 的 feasibility-first correction；目标为 `0.5 ||delta_tau||^2`，`q=0`，不带 native objective gradient。
- 只有通过最新 static、authoritative dynamics、Local-SFC、dynamic risk 且重新分类为 `ABSOLUTE_SAFE` 的 initializer 才会被保存。
- full SCP refinement 数值失败时，会重新以当前时间复核该 initializer；复核仍为 `ABSOLUTE_SAFE` 才返回 `FEASIBLE_INITIALIZER_FALLBACK`。
- 最终有效运行中，24 条安全 initializer 在 full SCP refinement 失败后全部被保住；其中 5 条对应精确的 `QP_MAX_ITER_EXHAUSTED`，没有再被整体判死。
- 一条接近历史问题形态的候选，retiming 后 authoritative jerk 为 `22.251560`，time-only 仅增加 `0.003060 s`，将 lattice jerk 修正到 `22.000000`，随后 full SCP 成功并提交。
- 另一条动态距离仅 `1.188127 m` 的 SIDE_MINUS 在 full QP MAX_ITER 后由安全 initializer fallback 保住，最终以 `1.188056 m` 最新风险复核后 commit/publish/activate。
- 有效运行 task complete，动态碰撞 0、静态碰撞 0、`<0.5 m` unsafe samples 0；没有观察到中途 stop→restart→INVALID NOMINAL 直冲碰撞链。

本轮一轮运行不能证明所有随机运行都已彻底解决。现有 nonblocking emergency 合同仍提交了 21 条 `INVALID NOMINAL`；它们未进入 safe/warm cache，本轮也未导致碰撞，但仍是明确的剩余风险。

## 2. 直接依据与当前源码复核

直接阅读：

- `feedback/feedback_18.md`
- `feedback/feedback_19.md`

历史链条经当前源码复核仍成立：

1. SIDE/A* repair 生成 repaired `P/T`，retiming 后进入 `runCandidateHardCorridorSCP()`。
2. full SCP 使用逐 piece lattice 构造 velocity、acceleration、jerk hard rows。
3. 历史 `1.714893 m` SIDE_PLUS 的 outer checker 报 jerk `21.998234` PASS，但 SCP lattice 报 `22.393653`，差值 `0.393653`。
4. full QP MAX_ITER 会令旧流程的 SIDE `success=false`，之后可落入 INVALID NOMINAL nonblocking fallback。
5. 当前 zero-plane Local-SFC 语义仍存在：合法空平面走 `SFC_NOT_REQUIRED`，源码位于 `planner_manager.cpp:3056-3074`。
6. Feedback 18 的 previous-safe 最终选择 TOCTOU 在本轮修改前仍有结构风险：早期验证结果可跨越较长 SIDE 求解后继续使用。本轮增加最终使用点复核。

历史 `1.714893 m` 是动态障碍中心距离，并且只覆盖当前有限预测 horizon；本轮没有把它误称为全时域 surface clearance 或绝对安全证明。

## 3. 修改文件

本轮只修改/新增以下文件：

1. `ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/include/optimizer/poly_traj_optimizer.h`
2. `ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/src/poly_traj_optimizer.cpp`
3. `ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp`
4. `ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/CMakeLists.txt`
5. 新增 `ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_opt/test/time_only_feasibility_contract_test.cpp`

未修改：场景、SIDE offset、A*、Local-SFC 几何、MINCO 数学、full SCP trust 主逻辑、OSQP 参数、三类阈值、dynamic clearance、max jerk、controller、viewpoint/visibility 功能开关。

## 4. Authoritative dynamics contract 统一

### 4.1 共用采样 lattice

新增共享遍历函数：

- 声明：`poly_traj_optimizer.h:406-418`
- 实现：`poly_traj_optimizer.cpp:11-81`

对每个 piece `i`，使用当前 `cps_num_prePiece_`，在：

```text
s_ij = (j / cps_num_prePiece) T_i,  j=0,...,cps_num_prePiece
```

采样 `v_i(s_ij), a_i(s_ij), j_i(s_ij)`；相邻 piece 的重复起点只计算一次。因此 `N` pieces、每 piece 5 subdivisions 时总样本数为 `5N+1`。

authoritative violation 定义为：

```text
e_v = max(0, max ||v|| - v_max)
e_a = max(0, max ||a|| - a_max)
e_j = max(0, max ||j|| - j_max)
```

`evaluateCandidateDynamicsLattice()` 和 full SCP 的 `appendDynamicsConstraints()` 都调用该遍历器：

- standalone evaluator：`poly_traj_optimizer.cpp:44-81`
- SCP hard rows：`poly_traj_optimizer.cpp:1220-1284`

### 4.2 retiming 使用同一判据

manager 中：

- pre-retiming authoritative check：`planner_manager.cpp:3597-3615`
- uniform retiming 直接使用该 summary 计算 `s_v/s_a/s_j`：`planner_manager.cpp:3638-3668`
- post-retiming authoritative check：`planner_manager.cpp:3691-3756`
- 旧 `checkTrajectoryDynamics()` 只保留日志，不再决定是否已经满足 SCP hard-row contract：`planner_manager.cpp:3617-3619`

注意：现有 diagnostic `checkTrajectoryDynamics()` 使用另一种连续/细采样并含 `feasibility_tolerance`，因此个别 `[candidate-final-status]` diagnostic max jerk 可略高于 22，而 authoritative SCP lattice 仍严格通过。本轮修复的是“retiming admission 与 SCP hard rows”的合同一致性，并未把 SCP hard rows改为连续时间全局极值约束。

## 5. Fixed-P time-only feasibility correction

### 5.1 决策变量和目标

入口：

- 声明：`poly_traj_optimizer.h:330-335`
- 实现：`poly_traj_optimizer.cpp:208-416`
- manager 调用：`planner_manager.cpp:3758-3800`

决策变量只有：

```text
z = delta_tau in R^N
```

`P` 不在 decision vector 中；`fixedInnerPts` 在每次 MINCO regenerate 中保持不变。

QP 目标：

```text
min  1/2 delta_tau^T I delta_tau
```

源码使用：

```text
zero_gradient = 0
hessian = 1
```

位于 `poly_traj_optimizer.cpp:355-358`。没有 Feedback 19 中量级很大的 native objective gradient。

### 5.2 dynamics rows

对每个 authoritative lattice sample，实际使用平方范数的一阶线性化：

```text
2 v_k^T (dv/dtau) delta_tau <= v_max^2 - ||v_k||^2
2 a_k^T (da/dtau) delta_tau <= a_max^2 - ||a_k||^2
2 j_k^T (dj/dtau) delta_tau <= j_max^2 - ||j_k||^2
```

对应 `poly_traj_optimizer.cpp:309-329`。梯度先由现有 MINCO adjoint helper 计算完整 `[P,tau]` row，再仅取 `tau` tail，因此没有复制第二套 MINCO derivative。

时间 trust row 使用当前 virtual-time map 的 Jacobian：

```text
-rho_i <= f'(tau_i) delta_tau_i <= rho_i
```

对应 `poly_traj_optimizer.cpp:337-353`。time positivity 继续由 `VirtualT2RealT()` 映射保证。

最多执行 4 次 time-only QP。若 time-only QP 数值失败，才尝试小幅 uniform duration expansion；每次仍必须用同一 authoritative lattice 复核。失败时恢复原始 `T`，不会污染输入 `P/T`：`poly_traj_optimizer.cpp:375-415`。

manager 只对相对超限不超过 10% 的轻度 violation 触发该阶段；更严重者保持旧 full SCP 兼容路径，不新增 hard failure。

### 5.3 时间改变后的同步

修正 `T` 后：

- 用固定 `P` regenerate MINCO：`planner_manager.cpp:3789-3799`
- 重新 evaluate dynamic risk：`planner_manager.cpp:3804-3835`
- Local-SFC active time 按最终 duration scale 同步：`planner_manager.cpp:4018-4034`
- 不复用修正前的 dynamic distance。

## 6. Feasible initializer preservation

### 6.1 保存条件

保存前重新检查：

- authoritative v/a/j lattice：`planner_manager.cpp:4201-4205`
- static trajectory check：`planner_manager.cpp:4206-4208`
- Local-SFC max violation `<=1e-3`：`planner_manager.cpp:4209-4214`
- 以 `ros::Time::now()` 为 prediction epoch 重新 dynamic risk：`planner_manager.cpp:4215-4217`
- 重新走三类分类，并且必须为 `ABSOLUTE_SAFE`：`planner_manager.cpp:4218-4239`

只有全部满足时 `feasible_initializer_available=true`。保存的是 head/tail、固定 inner `P`、修正后 `T` 和对应 constraint/display metadata，不保存一个可能被 full SCP 后续覆盖的 optimizer 引用。

### 6.2 full refinement 失败后的回退

full `(P,tau)` SCP 仍照常运行：`planner_manager.cpp:4289-4303`。

如果 full SCP 失败且有安全 initializer，则重新构造并再次检查：

- authoritative dynamics
- static
- Local-SFC
- 当前时间 dynamic risk
- safety class `ABSOLUTE_SAFE`

代码：`planner_manager.cpp:4305-4388`。

只有该复核仍通过才返回：

```text
FEASIBLE_INITIALIZER_FALLBACK:<full-scp-status>
```

这与原有 `UNSAFE_FALLBACK` 完全不同：前者是经过完整当前合同复核的 ABS SIDE，后者是为保持仿真运行而允许的已知 INVALID NOMINAL。

最终 selection 前还会再次 rerisk/reclass，commit freshness 也保持原合同不变。

## 7. Previous-safe 最终使用点复核

新增 `revalidate_previous_safe_at_use()`：`planner_manager.cpp:1768-1794`。

每次最终保留前都重新读取 `ros::Time::now()`，计算：

```text
remaining = max(0, duration - (now - start_time))
```

若 remaining 已为 0，立即失效；否则调用 `validatePreviousRemainingTrajectory(now, ...)`，由该函数用最新 `now` 重新 slice/revalidate。

调用点：

- persistence selection：`planner_manager.cpp:5081`
- final KEEP_PREVIOUS_SAFE：`planner_manager.cpp:5168`
- post-check selection：`planner_manager.cpp:5240`
- commit failure fallback：`planner_manager.cpp:5402`

最终有效运行中：

- `[previous-safe-final-revalidation]` 494 条；
- `remaining_duration=0` 且 `final_valid=1`：0；
- `remaining_duration=0` 仍 `KEEP_PREVIOUS_SAFE`：0。

## 8. 实施中发现并修正的顺序错误

前两次工程验证目录 `alp_1`、`alp_2` 不作为有效结果：它们暴露了 full-SCP 成功路径的对象赋值顺序错误，planner 出现 `bad_alloc/length_error`。

ASAN 诊断定位到：

```text
MinJerkOpt::getTraj()
planner_manager.cpp:4456
```

原因是 `CandidateResult` 中默认构造的 `MinJerkOpt` 尚未接收 full-SCP optimizer 结果，诊断代码就先调用了 `getTraj()`；默认对象内的 piece count 未初始化，导致异常大分配。

修复为：full-SCP success 分支在任何 diagnostic 读取之前先执行：

```text
result.min_jerk_opt = ploy_traj_opt_->getMinJerkOpt();
```

当前位置：`planner_manager.cpp:4453-4466`。这只是结果安装顺序修正，没有改变 SCP 数学。修复后 Release 完整运行无 planner process death。

潜在问题：`MinJerkOpt` 默认对象的内部 `N` 未显式初始化仍是底层脆弱点，本轮没有扩大修改该通用容器。

## 9. Contract/self-test

新增 `time_only_feasibility_contract_test`，CMake 入口：`traj_opt/CMakeLists.txt:47-53`。

测试覆盖：

- fixed `P` 不变，并用 corrected `T` regenerate 后直接断言内部 junction 仍等于输入 `P`；
- 初始 mild jerk = `23 > 22`；
- 只增加 `T` 后 authoritative jerk = `22`；
- authoritative lattice pass；
- time-only 失败时原始 `T` 和 `P` 不被篡改。

Release 实测输出：

```text
TIME_ONLY_FEASIBILITY_CONTRACT=PASS
initial_max_j=23
corrected_max_j=22
initial_total_T=1.3766
corrected_total_T=1.39715
P_fixed=1
authoritative_pass=1
correction_status=QP_TIME_ONLY_SOLVED
uniform_fallback=0
failure_preserved_input=1
failed_status=FAILED_AFTER_primal_infeasible
```

## 10. Build

最终命令：

```bash
catkin build path_searching traj_opt ego_planner --no-status -j2
```

结果：5 个依赖/目标包全部成功，0 failed。只有已有 CMake/PCL capability warnings。

`git diff --check` 对本轮 tracked 修改无 whitespace error。

## 11. 单次有效真实仿真

有效结果目录：

```text
/home/bob/ALP/egov2_fc65423_constvel/feasible_side_validation_20260904/alp_3
```

配置：

- scene：`ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_visibility_stress.json`
- scene SHA256：`430fce52ee31c4a4ea13607e39346e0f04f1af1e3da31eb426a44a61786c4cb0`
- strict target replay：YES
- dynamic epoch synchronized：YES
- moving obstacle cost：ON
- risk candidates：ON
- hard corridor SCP：ON
- target-facing yaw：ON
- max jerk：22
- candidate visibility ranking：OFF
- A* visibility cost：OFF
- MINCO visibility cost：OFF
- cooperative viewpoint：OFF
- RViz：OFF
- target final stop 后稳定 `3.505 s` 自动 SIGINT shutdown

`run_meta.json` 记录 target reached、auto shutdown 均为 true。脚本的 `exit_code=120` 是该 runner 在正常 SIGINT 收尾下的记录方式，不是 planner crash。

## 12. SIDE candidate 统计

| 指标 | 结果 |
|---|---:|
| SIDE backend records | 122 |
| pre-full-SCP feasible initializer available | 70 |
| time-only triggered | 27 |
| time-only QP solved | 27/27 |
| uniform fallback used | 0 |
| mean total-duration increase | 0.006300 s |
| median total-duration increase | 0.000111 s |
| max total-duration increase | 0.069108 s |
| corrected class ABSOLUTE_SAFE | 17 |
| corrected class IMPROVED_ONLY | 1 |
| corrected class INVALID | 9 |
| full SCP `SCP_FINAL_OK` | 60 |
| full SCP refinement failures | 62 |
| safe initializer + full refinement failure | 24 |
| `FEASIBLE_INITIALIZER_FALLBACK` used | 24 |
| safe initializer lost after full failure | 0 |
| exact `QP_MAX_ITER_EXHAUSTED` | 11 |
| QP MAX_ITER with safe initializer | 5 |
| QP MAXITER safe initializer successfully preserved | 5/5 |

full refinement failure status：

| Status | Count |
|---|---:|
| DYNAMICS_MODEL_MISMATCH_TRUST_EXHAUSTED | 23 |
| TRUE_CONSTRAINT_INFEASIBILITY | 14 |
| QP_MAX_ITER_EXHAUSTED | 11 |
| SPATIAL_TRUST_RETRY_EXHAUSTED | 6 |
| DYNAMICS_TRUST_RETRY_EXHAUSTED | 3 |
| P_TRUST_TOO_SMALL | 3 |
| VELOCITY_LIMIT | 1 |
| QP_MAX_ITER（未归一化旧状态字符串） | 1 |

24 次 fallback 原因：model mismatch 11、QP max-iter exhausted 5、spatial trust 4、dynamics trust 3、velocity limit 1。fallback 当前时刻复核风险距离范围为 `1.188127–31.948554 m`，全部保持 `ABSOLUTE_SAFE`。

## 13. 关键运行证据

### 13.1 轻度 jerk gap 被 time-only 修正

日志：`alp_3.launcher.log:42658-42712`。

```text
post-retiming: max_j=22.251560, violation=0.251560, pass=0
time-only: duration 3.247731 -> 3.250791, delta=0.003060
corrected: max_v=2.911051, max_a=3.141113, max_j=22.000000
corrected risk=1.325159, class=ABSOLUTE_SAFE
full SCP: SCP_FINAL_OK
final commit risk=1.286783
```

该候选与 Feedback 19 的“几何可用、仅轻度 jerk gap”是同一类型，虽然不是历史精确 `1.714893` 实例。修正过程中 `P` 由接口和 contract test 保证完全固定。

### 13.2 full QP MAX_ITER 后仍成为 executable SIDE

日志：`alp_3.launcher.log:25042-25074`。

```text
candidate=SIDE_MINUS
full SCP=QP_MAX_ITER_EXHAUSTED
initializer recheck: static=1 dynamics=1 Local-SFC=1 risk=1.188127
class=ABSOLUTE_SAFE
fallback used=1
selection rerisk=1.188056
head_error=0.111910 <= 0.5
commit generation=138
publish + traj_server activate
```

这直接证明：full native-objective QP 的数值失败不再自动杀死一个已经可执行的安全 SIDE。

另一个 SIDE_PLUS QP_MAX 实例在 `alp_3.launcher.log:11273-11320`，以最新风险 `13.196713 m` commit 为 generation 89。

## 14. 运行结果

| Metric | alp_3 |
|---|---:|
| Task complete | YES |
| Dynamic collision episodes/samples | 0 / 0 |
| Unsafe samples `<0.5 m` | 0 |
| Minimum dynamic center/surface metric clearance | 0.554151 m |
| Static collision episodes/samples | 0 / 0 |
| Minimum static clearance | 0.145033 m |
| Tracking mean | 1.985374 m |
| Tracking P95 | 3.977343 m |
| Mean path length | 81.765812 m |
| Planning kernel median/P95/max | 0.4755 / 3.8923 / 17.803 ms |
| Epoch→commit median/P95/max | 26.4465 / 48.3334 / 599.204 ms |
| UAV visibility | 98.713% / 87.625% / 97.467% |
| All-3 / at-least-2 / none | 86.130% / 98.339% / 0.665% |
| Executed odom-FD jerk P95 mean | 55.7439 m/s^3 |

Executed jerk 是 odom finite difference，不等于 MINCO/SCP analytical or lattice jerk。

### 14.1 stop/starvation/unsafe fallback 检查

- 运行中正常飞行阶段没有 `speed <0.05 m/s` 连续 0.5 s 后再 restart 的事件；只检测到启动阶段和 final stop 段。
- planner process deaths：0。
- no-trajectory cascade：0。
- OSQP/Local-SFC dimension mismatch：0。
- optimizer NaN/Inf failure：0；日志中的 `nan` 来自 feature-disabled viewpoint diagnostic，不属于 optimizer。
- stale-head rejection：7，均未继续 publish stale candidate。
- `INVALID NOMINAL` 实际 commit：21；均记录 `stored=0 ... reason=UNSAFE_CLASS_NOT_CACHED`，没有污染 previous-safe/warm cache。
- 因而本轮没有复现 Feedback 18 的“hold→restart→INVALID NOMINAL→collision”，但 INVALID nonblocking commit 合同仍真实存在，不能据一轮结果称其已消失。

## 15. Latency 影响

与 Feedback 18/19 使用的旧同场景历史 run 做非严格参考：

| Latency | Historical | Current alp_3 | Change |
|---|---:|---:|---:|
| kernel P95 | 0.762 ms | 3.892 ms | +3.130 ms（相对 +411%） |
| epoch→commit P95 | 46.237 ms | 48.333 ms | +4.53% |
| epoch→commit max | 468.937 ms | 599.204 ms | +130.267 ms |

该历史 run 与本轮不是严格代码/target replay A/B，因此不能把所有差异因果归给本修复。更有意义的是整体 epoch→commit P95 只增加约 4.5%，但长尾仍存在：本轮 SIDE full-refinement optimization P95 约 `217.4 ms`、max `617.1 ms`，并出现 4 条 heartbeat-loss warning。stale-head gate 阻止了相应过时提交，且没有形成 hold/no-command cascade。

结论：常态规划延迟没有明显失控；同步 full SCP 长尾和 heartbeat 耦合仍是已有风险，本轮按要求未扩展线程/heartbeat 重构。

## 16. 新问题与限制

1. **单轮证据限制**：没有复现历史精确 `1.714893 m` 候选，不能声称所有相同随机事件永久解决。
2. **有限动态 horizon 未关闭**：ABSOLUTE_SAFE 仍按当前有限 prediction horizon 定义；本轮没有改变该设计。
3. **非 goal static checker 前缀语义未改变**：本轮没有扩大 static checker 范围。
4. **INVALID NOMINAL 仍可 nonblocking commit**：21 次，虽无碰撞且不写 safe cache，仍可能在别的随机运行重现风险链。
5. **full SCP 仍频繁 refinement failure**：本轮目标是区分 objective failure 与 feasibility failure，而不是消除所有 MAX_ITER/model mismatch。
6. **连续极值与 lattice 的差异**：authoritative contract 是现有 SCP lattice，不是全连续时间解析最大值。
7. **默认 MinJerkOpt 脆弱性**：实施中已修当前读写顺序，但底层默认 piece count 未初始化是潜在通用 bug。

## 17. Git diff 摘要

当前 worktree 在本轮开始前已高度 dirty，`git diff --stat` 包含大量历史修改，不能把全局统计归于本轮。本轮只触及第 3 节列出的 5 个文件；未 reset、checkout、clean、commit，也未覆盖其他修改。

本轮局部逻辑增加：

- shared authoritative dynamics lattice evaluator；
- fixed-P time-only feasibility helper；
- Local-SFC max violation evaluator；
- manager 中 retiming/risk/time synchronization；
- safe initializer preservation/revalidation fallback；
- previous-safe final-use revalidation；
- contract test；
- full-SCP success result 安装顺序修复。

## 18. Final fields

```text
AUTHORITATIVE_DYNAMICS_CONTRACT_UNIFIED:
YES

TIME_ONLY_FEASIBILITY_IMPLEMENTED:
YES

TIME_ONLY_DECISION_VARIABLE:
delta_tau

TIME_ONLY_P_FIXED:
YES

TIME_ONLY_NATIVE_OBJECTIVE_GRADIENT_USED:
NO

TIME_ONLY_Q_ZERO_OR_FEASIBILITY_OBJECTIVE:
YES

TIME_ONLY_USES_SAME_VAJ_LATTICE_AS_SCP:
YES

TIME_ONLY_TRIGGERED:
27

TIME_ONLY_SUCCESS:
27/27

TIME_ONLY_UNIFORM_FALLBACK_USED:
0

FEASIBLE_INITIALIZER_PRESERVATION_IMPLEMENTED:
YES

FEASIBLE_INITIALIZER_AVAILABLE:
70

FULL_SCP_FAILURE_WITH_FEASIBLE_INITIALIZER:
24

FEASIBLE_INITIALIZER_FALLBACK_USED:
24

FEASIBLE_INITIALIZER_LOST_AFTER_FULL_SCP_FAILURE:
0

QP_MAX_ITER_SAFE_INITIALIZERS_PRESERVED:
5/5

FEASIBLE_FALLBACK_REQUIRES_ABSOLUTE_SAFE:
YES

DYNAMIC_RISK_RECOMPUTED_AFTER_TIME_CHANGE:
YES

DYNAMIC_RISK_RECOMPUTED_BEFORE_FEASIBLE_FALLBACK:
YES

PREVIOUS_SAFE_FINAL_REVALIDATION_IMPLEMENTED:
YES

PREVIOUS_SAFE_ZERO_REMAINING_RETAINED:
NO

CONTRACT_TEST:
PASS

BUILD:
PASS

REAL_VALIDATION_RUN:
/home/bob/ALP/egov2_fc65423_constvel/feasible_side_validation_20260904/alp_3

TASK_COMPLETE:
YES

DYNAMIC_COLLISION:
0 episodes / 0 samples

STATIC_COLLISION:
0 episodes / 0 samples

UNSAFE_SAMPLES_LT_0_5M:
0

PROMISING_SIDE_GEOMETRY_KILLED_ONLY_BY_FULL_QP:
0 among the 24 candidates that had a saved ABSOLUTE_SAFE initializer

FULL_SCP_QP_MAX_ITER_STILL_OCCURRED:
YES

FULL_SCP_QP_MAX_ITER_CAN_PRESERVE_SAFE_INITIALIZER:
YES

STOP_RESTART_INVALID_NOMINAL_COLLISION_REPRODUCED:
NO

INVALID_NOMINAL_COMMITTED:
YES — 21, existing nonblocking emergency behavior; none cached as safe

NEW_TRAJECTORY_STARVATION_OR_NO_COMMAND_CASCADE:
NO

NEW_DIMENSION_ERROR:
NO

NEW_OPTIMIZER_NAN_INF:
NO

LATENCY_CLEAR_REGRESSION:
NO for epoch-to-commit P95; full-SCP long-tail remains

PRODUCTION_CODE_CHANGED:
YES

SCENE_CHANGED:
NO

CURRENT_FEEDBACK_FILE:
/home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_20.md
```

## 19. 最小后续建议

先不要再扩大到新的 SCP/OSQP 改造。最小下一步应是用完全相同代码和 scene 再做重复运行，确认：

1. QP MAX_ITER 前的安全 SIDE 是否持续被 initializer fallback 保住；
2. `INVALID NOMINAL` 21 次提交是否在别的随机时序下重新形成碰撞链；
3. full-SCP 长尾/heartbeat warning 是否会再次造成 trajectory lifecycle starvation。

若重复运行仍发生碰撞，应先对碰撞周期确认是否已经存在可用的 `FEASIBLE_INITIALIZER_FALLBACK`，再决定是处理 INVALID emergency policy，还是处理 heartbeat/长求解生命周期；不要继续通过放宽 hard rows 或修改场景来掩盖。
