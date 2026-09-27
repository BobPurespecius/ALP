# Feedback 19 — 1.714893 m SIDE_PLUS Candidate Failure Autopsy

日期：2026-09-04  
审计性质：只读源码审计与历史日志尸检  
生产代码修改：无  
新实验：无

## 1. 结论先行

目标候选已精确定位。它出现在：

- 运行目录：`/home/bob/ALP/egov2_fc65423_constvel/scene_geometry_visibility_20260902/visibility_benchmark/alp/`
- 日志：`alp.launcher.log`
- ROS 时间：`1788354686.334382` 开始规划，`1788354686.435838` 记录最终失败
- UAV：`drone_id=0`
- replan：`628`
- risk cycle：`38`
- 候选：`SIDE_PLUS`
- 动态障碍：`obstacle_id=2`，场景中为 `dynamic_gate_V3`
- SIDE offset：`0.700 m`
- 目标动态 clearance：`1.714893 m`
- 最终失败：`QP_MAX_ITER_EXHAUSTED`

核心判断：

1. 这是一条在当前静态采样检查和 2 s 动态预测范围内很有希望的绕行几何：A* 成功，raw path static-free，修复后 MINCO static-free，动态中心距离从 NOMINAL 的 `0.327638 m` 提高到 `1.714893 m`。
2. 它不是一条已经被完整证明为可执行且全时域安全的轨迹：`1.714893 m` 只覆盖当前 `evaluateDynamicRisk()` 的有限预测范围；并且 SCP 的严格逐 piece 动力学采样发现 jerk 超限 `0.393653 m/s^3`。
3. 真正把它判死的直接事件是同一 SCP outer iteration 内三次主 OSQP 求解均达到 `2000` 次迭代上限，没有一次返回可用 step。
4. 因为没有 QP step，代码没有进入 nonlinear regenerate、model-agreement、static/SFC/corridor nonlinear trial recheck；因此不能把失败归因于 model-agreement、Local-SFC 或 nonlinear recheck。
5. 最符合证据的根因是：边界化 retiming 留下一个很小但真实的严格 jerk hard-row violation，随后带有极大 objective gradient、374 个归一化 hard rows 和 57 个 trust rows 的 QP 出现 dual convergence/conditioning failure；trust shrink 没有修复，反而使 dual residual 从 `106.509` 上升到 `1096.37`。

## 2. 证据范围与候选唯一定位

### 2.1 运行工件

确认读取：

- `scene_geometry_visibility_20260902/visibility_benchmark/alp/alp.launcher.log`
- `scene_geometry_visibility_20260902/visibility_benchmark/alp/alp_trajectory.csv`
- `scene_geometry_visibility_20260902/visibility_benchmark/alp/run_meta.json`
- `ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_visibility_stress.json`
- 当前生产源码树：`ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner`

场景 SHA256：

`430fce52ee31c4a4ea13607e39346e0f04f1af1e3da31eb426a44a61786c4cb0`

### 2.2 为什么不能只用 candidate_id=42 定位

日志中 `candidate_id=42` 在较早时刻也曾用于其它候选。因此本次候选使用以下联合键定位：

`ROS time + drone_id + replan 628 + SIDE_PLUS + obstacle_id=2 + duration=14.235929 + risk_distance=1.714893`

联合键把它唯一锁定在 `alp.launcher.log:44815-44908` 附近，且与 Feedback 18 中 restart 前的最后一次 PLUS backend failure 是同一候选。

### 2.3 同一 planning cycle 上下文

规划周期开始前：

- 当前 trajectory：`traj_id=254`
- start time：`1788354684.000697136`
- duration：`2.217082 s`
- 到期时间：`1788354686.217779`
- cycle 37 虽然 remaining duration 已为 `0`，仍以 previous safe 形式被保留并重复 publish，但 traj_server 拒绝重复/乱序消息。
- cycle 38 在 `1788354686.334343` 明确记录：`local trajectory expired, replan from odom`。
- previous validator 随后判定 `EXPIRED`，因此决定性周期没有可保留的 previous ABSOLUTE_SAFE。

同周期候选：

| Candidate | 诊断动态中心距离 | backend 结果 | safety class | 最终处理 |
|---|---:|---|---|---|
| NOMINAL | 0.327638 m | nominal solver success | INVALID | emergency fallback commit |
| SIDE_PLUS | 1.714893 m | QP_MAX_ITER_EXHAUSTED | INVALID（success=false） | rejected |
| SIDE_MINUS | 1.643011 m | retimed static collision / dynamics fail | INVALID | rejected before SCP |
| previous traj 254 | invalid/expired | EXPIRED | INVALID | unavailable |

## 3. 完整生命周期

### 3.1 NOMINAL base

日志证据：`alp.launcher.log:44815-44816`。

- planning epoch：`1788354686.334382`
- nominal piece count：`5`
- fresh duration：`2.500000 s`
- head position：`(27.692908, -2.467415, 1.500474)`
- local target：`(34.197693, 1.267148, 1.499907)`
- initial max velocity：`5.645772 m/s`
- initial max acceleration：`6.972813 m/s^2`
- initial max jerk：`29.941807 m/s^3`
- nominal solver：success
- nominal 最终 duration：`4.308592 s`
- nominal dynamic center distance：`0.327638 m`

### 3.2 SIDE_PLUS seed 与 offset

日志证据：`alp.launcher.log:44818-44820`。

- requested offset：`0.700 m`
- direct offset path：static infeasible
- 最小测试 offset：`0.400 m`，仍 static collision
- SIDE region alpha samples 的最好动态距离：`0.6132 m`
- 因 direct SIDE static-infeasible，进入 A* repair

### 3.3 static collision window 与 A* repair

日志证据：`alp.launcher.log:44820-44830`。

- SIDE scope：`[0.0, 2.5] s`
- dynamic conflict window：`[0.45, 1.35] s`
- static collision interval：`[0.520833, 0.729167] s`
- repair active anchors：
  - start：`(27.866, -2.556, 1.500)`
  - goal/rejoin：`(29.793, -2.022, 1.499)`
- downstream rejoin attempt：`0`
- downstream rejoin index：`9`
- A* result：`SUCCESS`
- A* elapsed：`0.050 ms`
- raw points：`20`
- raw occupied points：`0`
- raw path static-free：`true`

### 3.4 simplification、semantic-v2 与 Local-SFC

- simplified guide count `N_g = 11`
- turning points retained：`7`
- collision-core reinsertion points：`2`
- semantic-v2：accepted via `SEMANTIC_DATA_INSUFFICIENT_FALLBACK`
- Local-SFC plane count：`0`
- Local-SFC status：`SFC_NOT_REQUIRED`
- handoff allowed：`true`
- repaired guide static-free：`true`
- reconstructed MINCO static reason：`OBS_FREE`

semantic-v2 的三个数值项在这次 fallback 日志中没有保存：

- closest-approach sign：N/A
- weighted mean `L_w`：N/A
- `R_correct`：N/A

因此不能声称本候选拥有高置信 semantic-v2 PASS。能够确认的只是：fallback 是 non-blocking，`path_side_valid=1`，后续 A*/static/SFC handoff 继续执行。

源码对应：

- A* raw/static audit：`plan_manage/src/planner_manager.cpp:2700-2746`
- simplification/turning/reinsertion：`planner_manager.cpp:2748-2880`
- Local-SFC construction/handoff：`planner_manager.cpp:2888-3054`

### 3.5 guide 到 repaired P/T

本候选的 repaired local guide：

- local A* path length：`2.107107 m`
- local repaired pieces：`10`
- local guide points：`11`
- original collision window：`0.625000 s`
- arc-length/curvature duration：`1.054840 s`
- local minimum piece time：`0.041667 s`
- local maximum piece time：`0.162539 s`
- maximum estimated curvature：`6.340507 1/m`

full reconstructed trajectory：

- full piece count：`15`
- full internal MINCO junction count `N_P = 14`
- full guide/junction position count（含 head/tail）：`16`
- A* guide 的 11 个点作为 repair interval junction 进入 full reconstruction
- 后面拼接 base trajectory 的 remaining junctions

源码在 `planner_manager.cpp:3477-3488` 从 repaired trajectory 提取：

```cpp
const int repaired_piece_num = repaired_traj.getPieceNum();
Eigen::MatrixXd repaired_inner_points =
    repaired_traj.getPositions().block(0, 1, 3, repaired_piece_num - 1);
side_init_mjo.reset(repaired_head, repaired_tail, repaired_piece_num);
side_init_mjo.generate(repaired_inner_points, repaired_traj.getDurations());
```

因此本候选进入最终 SIDE initializer 的 `P` 是 repaired trajectory 的所有内部 junction positions，不是单独再运行一次采样器。

### 3.6 arc-length/curvature 初始时间

repair-time-init 日志：

- path length：`2.107107 m`
- local piece count：`10`
- old window：`0.625000 s`
- new local duration：`1.054840 s`
- min piece T：`0.041667 s`
- max piece T：`0.162539 s`
- max curvature：`6.340507 1/m`

full reconstructed initial duration：`2.929840 s`。

full initial piece durations：

```text
[0.312500,
 0.117851, 0.058926, 0.058926, 0.162539,
 0.114932, 0.125000, 0.125000, 0.125000, 0.125000,
 0.041667, 0.062500,
 0.500000, 0.500000, 0.500000]
```

其中最短 piece 是 `0.041667 s`，位于 repair/rejoin 尾部附近；它是 retiming 后最短 physical T 的来源。

初始 analytical/coarse checker dynamics：

- max v：`10.340211 m/s`
- max a：`104.933983 m/s^2`
- max j：`2523.763106 m/s^3`

结论：原始 repaired initializer 的时间分配仍然极端激进；几何虽好，但初始连续多项式动力学是病态的。

### 3.7 uniform retiming

源码：`planner_manager.cpp:3568-3655`。

计算：

- `s_v = 3.446737`
- `s_a = 4.181985`
- `s_j = 4.858945`
- selected scale：`4.858945`

retimed duration：

`2.929840 × 4.858945 = 14.235929 s`

retimed piece durations：

```text
[1.518420,
 0.572632, 0.286316, 0.286316, 0.789766,
 0.558449, 0.607368, 0.607368, 0.607368, 0.607368,
 0.202456, 0.303684,
 2.429470, 2.429470, 2.429470]
```

retimed coarse/outer dynamics result：

- max v：`2.128076 m/s`
- max a：`4.444592 m/s^2`
- max j：`21.998234 m/s^3`
- outer status：`PASS`

这说明 uniform retiming 大幅修复了初始病态，但它把 jerk 几乎精确压在 `22` 的边界上，没有为不同采样网格、piece junction 峰值或线性化误差留下余量。

### 3.8 dynamic risk 1.714893 的阶段与口径

`1.714893 m` 出现在 retiming 完成后、SCP 开始前：

```text
[side-retiming-risk-sync]
prediction_epoch=1788354686.334382
conflict_time=2.000000
duration=14.235929
risk_valid=1
risk_distance=1.714893
```

当前源码 `planner_manager.cpp:845-901` 计算：

```cpp
relative_position = trajectory_position - obstacle_position;
distance = relative_position.norm();
```

所以它是：

`d = ||p_UAV(t) - p_obstacle(t)||`

即 center distance，不是扣除 UAV/obstacle radius 后的 surface clearance。

此外 `evaluateDynamicRisk()` 把检查范围限制为：

```cpp
check_end = min(trajectory_duration, moving_obj_prediction_horizon)
```

本候选 duration 为 `14.235929 s`，当前可靠动态预测 horizon 约为 `2 s`。因此 `1.714893 m` 只证明有限 horizon 内有较大中心距离，不能证明 14.24 s 全轨迹动态安全。

### 3.9 preserve corridor 与静态 rows

SCP 输入：

- preserve reference：retimed SIDE initializer 自身
- hard corridor radius：`0.500 m`
- hard corridor samples：`25`
- initial maximum corridor violation：`0`
- Local-SFC planes：`0`
- EGO native static rows：`0`
- reference static collision：`false`

当 current trajectory 与 preserve reference 完全重合时，源码 `poly_traj_optimizer.cpp:1332-1382` 对每个 sample 使用 3 轴正负 box rows：

`25 samples × 3 axes × 2 = 150 rows`

这解释了 `corridor_constraints=150`。

## 4. SCP/QP 精确重建

### 4.1 decision vector

piece count：`15`。

- internal position variables：`3 × (15-1) = 42`
- virtual-time variables：`15`
- total decision dimension：`57`

```text
z = [delta P (42), delta tau (15)]
```

物理 duration 使用虚拟时间映射：

`T_i = f(tau_i)`，线性 step 为 `delta T_i ≈ f'(tau_i) delta tau_i`。

### 4.2 objective 数值尺度

初始日志：

- total objective：`17246.7`
- smoothness cost：`192.65`
- region cost：`0.0440715`
- moving/static/tracking soft costs：`0`
- total gradient norm：`18076.6`
- P-gradient median：`216.505`
- P-gradient P95：`3664.85`
- P-gradient max：`5902.97`
- tau-gradient median：`2740.75`
- tau-gradient P95：`6860.67`
- tau-gradient max：`10690.4`

QP Hessian 源码 `poly_traj_optimizer.cpp:1759-1763`：

- P block diagonal：`1`
- tau block diagonal：`4`

即 unconstrained quadratic preferred step 的量级约为：

- `|delta P| ~ |q_P|`，最大约 `5903`
- `|delta tau| ~ |q_tau|/4`，最大约 `2673`

而 trust box 只有亚米级/小比例时间变化。这一巨大 objective-gradient 与 trust-box 尺度差是 QP conditioning/dual convergence 风险的重要证据。

### 4.3 hard-row family 数量

SCP 在 iteration 0 记录 hard constraints `374` 行：

| Row family | Rows | Active samples | Initial max violation | 本候选结论 |
|---|---:|---:|---:|---|
| SIDE preservation corridor | 150 | 25 | 0 | 初始可行；不是物理 bottleneck |
| Local-SFC | 0 | 0 | 0 | 不可能杀死该候选 |
| EGO native static rays | 0 | 0 | 0 | 不可能与 SFC 冲突 |
| velocity | 74 | 76 lattice points，2 个零 Jacobian endpoint row 未加入 | 0 | 非 bottleneck |
| acceleration | 74 | 同上 | 0 | 非 bottleneck |
| jerk | 76 | 76 | 0.393653 m/s^3 | 唯一已确认 hard violation |
| hard rows subtotal | 374 | — | — | 日志直接确认 |
| P trust | 42 | — | box | 另加 |
| T/virtual-time trust | 15 | — | box | 另加 |
| OSQP total rows | 431 | — | — | 374+57 |

velocity/acceleration/jerk 行数来自当前源码 `poly_traj_optimizer.cpp:905-985` 的逐 piece lattice：

- 每 piece `cps_num_prePiece=5`
- 首 piece含 start，后续 piece 跳过重复 start
- 总 lattice points：`15×5+1=76`
- v/a 的固定 boundary samples 若 Jacobian 为零，`appendLinearUpperBound()` 不追加，故各少 2 行
- jerk rows 保留 76 行

### 4.4 outer checker 与 SCP hard-row checker 的差异

outer checker：`planner_manager.cpp:1079-1100`。

- jerk 使用全局 `dt=max(0.02,min(0.05,duration/100))` 采样
- 还使用 `1 + feasibility_tolerance` 容差
- 本候选报告 `21.998234` 并 PASS

SCP checker：`poly_traj_optimizer.cpp:905-985`。

- 每个 piece 使用固定局部 lattice，包括 piece endpoint
- 严格比较 `||j||^2 <= max_jerk^2`
- 本候选捕获到 max jerk `22.393653`
- violation：`22.393653 - 22 = 0.393653 m/s^3`

因此同一 initializer 出现：

```text
outer dynamics check: PASS, max jerk 21.998234
SCP hard-row check: VIOLATION, max jerk 22.393653
```

这是本候选进入 QP 时唯一明确的物理不可行项。

如果只做 uniform time scaling，要把 `22.393653` 压至 `22`，理论最低附加 scale 为：

`s = cbrt(22.393653 / 22) ≈ 1.00593`

即只需约 `0.59%` 的全局时间增加。初始 T trust 允许每段约 `13.46%` 的相对物理时间变化，所以不能把该 QP 简单解释成“trust 太小，数学上无解”。

### 4.5 每次 OSQP 求解

本候选只有一个 SCP outer iteration：`iteration=0`。它没有完成任何有效 nonlinear trial。

| outer iter | attempt | trust P | trust T alpha | QP status | qp iter | primal residual | dual residual | rho updates | rho estimate | hard violation | accepted/rejected |
|---:|---:|---:|---:|---|---:|---:|---:|---:|---:|---|---|
| 0 | 0 | 0.250000 | 0.134585 | MAX_ITER | 2000 | 0.0385251 | 106.509 | 3 | 0.0220982 | jerk +0.393653 | no step / rejected |
| 0 | 1 | 0.125000 | 0.067293 | MAX_ITER | 2000 | 0.0393754 | 285.5 | 2 | 0.142368 | jerk +0.393653 | no step / rejected |
| 0 | 2 | 0.062500 | 0.033646 | MAX_ITER | 2000 | 0.0280235 | 1096.37 | 5 | 0.398934 | jerk +0.393653 | no step / terminal failure |

未发生或不可用字段：

- QP objective/merit after step：N/A，没有返回 step
- trial max SFC violation：N/A，没有 trial
- trial static violation：N/A，没有 trial
- trial corridor violation：N/A，没有 trial
- model error：N/A，没有 nonlinear regenerate
- model-agreement class：N/A
- accepted step：0

### 4.6 trust 的实际物理范围

初始 `trust_T alpha=0.134585`：

- minimum physical `|delta T|` bound：约 `0.02725 s`
- median：约 `0.08174 s`
- maximum：约 `0.32697 s`

最后 `trust_T alpha=0.033646`：

- minimum：约 `0.00681 s`
- median：约 `0.02044 s`
- maximum：约 `0.08174 s`

最短 retimed piece `T=0.202456 s` 对应：

- `tau=-1.97971`
- `f'(tau)=0.122134`

它不是完全冻结；初始相对物理 trust 仍为 `13.46%`。但连续 shrink 后，允许的实际时间变化迅速收窄。

### 4.7 probes、infeasibility 与 model agreement

源码 `poly_traj_optimizer.cpp:1801-1951` 只在 QP status 包含 `primal_infeasible` 时运行：

- `NO_P_TRUST`
- `NO_T_TRUST`
- `NO_PT_TRUST`

本候选三个 status 都是 `max_iter`，所以：

- `probe_no_p_tr=-1`
- `probe_no_t_tr=-1`
- `probe_no_pt_tr=-1`
- 没有 `TRUE_CONSTRAINT_INFEASIBILITY`
- 没有任何 no-trust feasibility 证明

同理，源码仅在 `qp.success` 后才 regenerate trial 并比较预测/实际 dynamics。这里没有任何成功 QP step，所以：

- no nonlinear regenerate
- no model-agreement reject
- no static/SFC/corridor nonlinear reject
- no accepted/rejected trial based on objective

最终 `QP_MAX_ITER_EXHAUSTED` 是主 QP 求解失败，不是 feasibility probe MAX_ITER。

## 5. 哪类约束杀死了它

### 5.1 Local-SFC

结论：不是。

- plane count：`0`
- hard rows：`0`
- status：`SFC_NOT_REQUIRED`
- 不存在相邻 planes、窄楔或与 native rays 方向冲突
- exact boundary scan occupancy query count 没有写入历史日志，无法恢复；但 transition/plane count 为 0

`LOCAL_SFC_OVERCONSTRAINT_PRIMARY = NO`

### 5.2 SIDE preservation corridor

结论：不是已证明的物理 bottleneck，但可能贡献数值冗余。

- radius：`0.5 m`
- reference：retimed SIDE initializer
- initial deviation：`0`
- initial violation：`0`
- 25 samples 产生 150 个 axis box rows
- 没有 trial，因此不存在“为了满足 dynamics 必须离开 corridor、却被 nonlinear checker 拒绝”的证据

这些 rows 在 current point 处成组对称且数量大，可能增加 active-set/redundancy 和 dual convergence 负担；但没有 QP matrix dump，不能声称它让可行域为空。

`SIDE_CORRIDOR_OVERCONSTRAINT_PRIMARY = NO`

### 5.3 native static separating rays

结论：不是。

- native static row count：`0`
- reference static collision：`false`
- max native static violation：`0`
- 不存在 native rays 与 Local-SFC 的重叠冲突

`NATIVE_STATIC_CONFLICT_PRIMARY = NO`

### 5.4 velocity / acceleration / jerk

- velocity：无 violation
- acceleration：无 violation
- jerk：`+0.393653 m/s^3`

jerk 是唯一明确需要 QP 修复的 physical hard-row family，但它不等同于 QP 数学无解：所需 uniform time correction 很小，且位于初始 T trust 以内。

因此：

- jerk 是直接的上游 feasibility trigger / secondary bottleneck
- OSQP numerical convergence failure 才是最终死亡事件

### 5.5 P/T trust

trust shrink 是 QP MAX_ITER 后的响应，不是第一因：

```text
MAX_ITER -> PT_SHRINK -> MAX_ITER -> PT_SHRINK -> MAX_ITER -> exhausted
```

如果 trust 太宽是主要问题，缩小 trust 通常应改善迭代稳定性；本候选 dual residual 却随 shrink 显著恶化：

`106.509 -> 285.5 -> 1096.37`

所以没有证据支持 `P_TRUST_TOO_SMALL`、`T_TRUST_TOO_SMALL` 或 model trust mismatch 是 primary。最后的 trust exhaustion 只是错误处理的终止条件。

### 5.6 numerical scaling / conditioning

当前最强证据指向 QP numerical/dual convergence：

- 每次均到 max_iter=2000
- primal residual 大约保持在 `0.028-0.039`
- dual residual 很大并随 trust shrink 恶化
- hard row norms 已单行归一化为 `1`
- P/T column max 分别约 `0.999974`、`0.731797`
- Hessian diagonal condition proxy 只有 `4`
- 但 objective gradient 最大达到 `O(10^4)`，而 trust step 只有 `O(10^-1)`
- corridor + dynamics 共 `374` 个 hard rows，另有 57 个 trust rows
- initial point 只有很小 jerk violation，其它 hard families 均可行

因此问题不是简单的“某行没归一化”，而更像：大 native objective gradient、紧 trust box、密集/重复 corridor+dynamics rows 与临界 jerk active rows共同造成 dual convergence 困难。

历史工件没有保存具体 QP `A/H/q/l/u`，所以以下不能严格计算：

- 完整矩阵 condition number
- near-dependent row singular values
- 每个 jerk row 的 dual multiplier
- 哪个具体 piece/sample 的最大 jerk row index
- OSQP 每 25 iteration residual trace

报告不把这些缺失量伪造成已知事实。

## 6. 时间初始化是否是 primary

### Claim

原始 repair→MINCO 时间初始化确实病态，但不是最终直接死亡原因；边界化 retiming 是重要 contributing factor。

### Evidence

- pre-retime：`v=10.34, a=104.93, j=2523.76`
- retime scale：`4.858945`
- post-retime outer check：`v=2.128, a=4.445, j=21.998`，PASS
- SCP exact piece lattice：`j=22.393653`，仅超 `0.393653`
- 其它 v/a/static/SFC/corridor rows 全部无 violation

### Counter-evidence

- retiming 已经把大部分动力学病态消除
- 额外约 0.59% uniform time 理论上可消除剩余 jerk exceedance
- 初始 T trust 足够容纳该量级变化
- 真正发生的终止状态是 OSQP MAX_ITER，而不是 retiming failure

### Confidence

高：`TIME_INITIALIZATION_PRIMARY = NO`。  
中高：retiming 没有按 SCP authoritative lattice 留出 interior margin，是 secondary causal precursor。

## 7. OSQP MAX_ITER 的性质

### 7.1 不是已证明 infeasible

OSQP 没有返回：

- primal infeasible
- dual infeasible
- setup failure

也没有运行 no-trust probes。因此无法声称 hard constraints 真正无解。

### 7.2 不是 model mismatch

没有成功 step，没有 nonlinear trial，model-agreement 逻辑未执行。

### 7.3 是主 QP numerical/iteration failure

源码 `traj_opt/src/scp_optimizer.cpp:146-195` 配置：

- scaling：`10`
- adaptive rho：enabled
- eps_abs/eps_rel：`1e-3`
- max_iter：`2000`
- polish：enabled

三次均耗尽 iteration。尤其 dual residual 逐次扩大，是 numerical/dual convergence failure 的直接证据。

### 7.4 Hessian time weight 4I 是否 primary

没有证据证明 `4I` 本身是 primary：

- time gradient 最大 `10690.4`，除以 4 后的 unconstrained direction 仍极大
- 物理 time trust 而非 Hessian 是 step 的主要上界
- OSQP 连一个可接受 step 都没返回，无法比较“宁愿移动 P 而不愿增加 T”的实际解

它可能影响 objective scaling，但本日志不足以把它单独定为根因。

## 8. 与最终 NOMINAL INVALID fallback 对比

| 指标 | SIDE_PLUS 1.714893 | final NOMINAL fallback |
|---|---:|---:|
| dynamic clearance type | center distance | center distance |
| dynamic clearance | 1.714893 m | 0.327638 m |
| evaluated horizon | ~2 s | ~2 s |
| static status | A* raw/rebuilt/reference checks free | final post-check passed |
| scalar static clearance | N/A | N/A |
| sampled path length | 8.4082 m | 7.503269 m |
| piece count | 15 | 5 |
| N_g | 11 | N/A（无 A* guide） |
| N_P | 14 | 4 |
| total duration | 14.235929 s | 4.308592 s |
| pre-retime max v | 10.340211 | 5.645772 fresh init；最终 N/A |
| pre-retime max a | 104.933983 | 6.972813 fresh init；最终 N/A |
| pre-retime max j | 2523.763106 | 29.941807 fresh init；最终 N/A |
| retimed/final logged max v | 2.128076 | N/A |
| retimed/final logged max a | 4.444592 | N/A |
| retimed/final logged max j | 21.998234 outer / 22.393653 SCP lattice | N/A |
| Local-SFC planes | 0 | 0 / not used |
| native static rows | 0 | nominal path not run through SIDE SCP |
| SCP status | QP_MAX_ITER_EXHAUSTED | not SIDE SCP; nominal solver success |
| formal safety class | INVALID because success=false | INVALID because dynamic distance<1.1 |
| committed | NO | YES, UNSAFE_FALLBACK |

### 为什么更安全的 PLUS 没有 commit

三类分类源码 `planner_manager.cpp:1738-1767` 首先要求：

```text
candidate.success == true
risk.valid == true
existing static/dynamics checks pass
```

PLUS 因 SCP 返回 false，`candidate.success=false`。虽然 diagnostic field 保留了 initializer risk `1.714893`，正式 result risk 没有成为可选择的成功 candidate；side-bilateral 因此显示 `plus_good=0`、`plus_clearance=-1`。

### 为什么 INVALID NOMINAL 能 commit

previous trajectory 已过期，PLUS/MINUS 都不是成功可执行 candidate。现有 nonblocking contract 明确保留 nominal emergency fallback：

- nominal solver 产生了可发布 polynomial
- nominal static/dynamics post-check 能通过
- dynamic class 是 INVALID
- selection reason：`UNSAFE_FALLBACK`
- commit freshness：`head_error=0.005689 m`，FRESH_COMMIT
- trajectory 255 最终 commit/publish

所以这里必须区分：

- solver/backend success：有没有一条可生成、可发布的 polynomial
- dynamic safety class：该 polynomial 的动态风险是否达到 1.1 m

NOMINAL 满足前者但不满足后者；PLUS 满足较好的诊断动态几何，但没有满足前者。

## 9. 根因排序

### PRIMARY — QP numerical/dual convergence failure

三次主 OSQP solve 均 MAX_ITER，且 dual residual 随 trust shrink 恶化。没有成功 step，直接触发 `QP_MAX_ITER_EXHAUSTED`。

### SECONDARY — strict jerk hard-row 与 retiming checker 的采样/余量不一致

retiming 在 outer checker 上把 jerk 压到边界，SCP authoritative lattice 仍发现 `+0.393653 m/s^3`。这是 QP 必须求解而不能直接接受 initializer 的唯一明确物理原因。

### CONTRIBUTING — objective/trust/row-system 尺度

- objective gradient `O(10^4)`
- trust step `O(10^-1)`
- 150 个 corridor rows
- 224 个 dynamics rows
- 57 个 trust rows
- row normalization 已生效，但不能消除整体 dual/active-set difficulty

### NOT PRIMARY

- Local-SFC：0 planes/0 rows
- native static rays：0 rows
- SIDE corridor physical overconstraint：initially feasible，无 trial 证据
- P/T trust too small：initial trust 足以容纳约 0.59% retime
- model agreement：未执行
- nonlinear static/dynamics recheck：未执行
- primal infeasibility：未报告、未 probe

## 10. 最小修复方向（本轮不实施）

### 排序

1. **最佳单一方向：在 full native-objective SCP 前做一个 feasibility-first 的轻量 time-only correction。** 固定已有的好 P/guide 几何，使用与 SCP hard rows 完全相同的逐 piece jerk lattice，先只调整 `tau/T` 消除小 jerk violation；该小步使用零或规范化 feasibility objective，而不是直接承受 `O(10^4)` 的 native objective gradient。
2. 若 feasibility-first 得到通过所有 authoritative hard rows 的 initializer，后续 full objective QP 若仅因 MAX_ITER 失败，不应自动丢弃已经通过最终 nonlinear/static/dynamic检查的 feasible candidate。
3. 作为更小的预处理，可让 pre-SCP retiming 使用 authoritative SCP lattice 并留下 interior jerk margin；但仅做这一项仍未必解决 full objective QP 的 numerical MAX_ITER。
4. corridor row 去重或 QP column/objective scaling 是后续可研究项；本候选没有证据支持先放宽 corridor 或删除安全约束。

### 为什么不是优先做“固定 T 先优化 P”

该候选的 P geometry 已经：

- A* static-free
- rebuilt static-free
- corridor violation 0
- Local-SFC not required
- native static violation 0

唯一 hard violation 是 jerk，而时间放大是最直接的小修正。因此更有依据的是“保持 P，先用 tau/T 消除 feasibility gap”，而不是先让 P 离开已经很好的绕行几何。

## 11. 五个核心回答

1. **这条 1.714893 m PLUS 候选是不是几何上已经很好？** 在当前 inflated-static 检查和 2 s 动态预测范围内，是一条明显有希望的候选：static-free，动态中心距离比 nominal 高 `1.387255 m`；但 semantic 使用 fallback，且 14.24 s 全时域动态安全没有被证明。
2. **第一次真正开始恶化/失败发生在哪一层？** 原始 repair MINCO 时间很病态；uniform retiming 基本修复后，SCP authoritative piece-lattice 首次发现仍有 `0.393653 m/s^3` 的 jerk hard-row violation。
3. **最终什么把它杀死？** 同一 outer iteration 的三次主 OSQP 均 `MAX_ITER=2000`，没有返回 step；主要是 numerical/dual convergence failure，临界 jerk row和大 objective-gradient/密集 rows 是关键上下文，trust shrink 只是失败后的反应。
4. **为什么 PLUS 失败而 INVALID NOMINAL commit？** PLUS 的 dynamic geometry 较好但 backend success=false，因此不是可执行 candidate；NOMINAL 虽动态 INVALID，却有成功 polynomial 且 previous 已过期，所以按 nonblocking emergency fallback 合同被提交。
5. **只允许一个最小改动，最应该改哪里？** 在不动 SIDE/A*/SFC geometry 的前提下，优先增加“固定 P、只调 tau/T 的 feasibility-first step”，使用 authoritative hard-row lattice先消除小 jerk gap，再进入现有 full objective SCP。

## 12. 最终字段

```text
TARGET_CANDIDATE_FOUND:
YES

TARGET_DYNAMIC_CLEARANCE:
1.714893 m

TARGET_DYNAMIC_CLEARANCE_TYPE:
CENTER_DISTANCE

TARGET_SIDE:
PLUS

TARGET_OFFSET_D:
0.700 m

TARGET_ASTAR_USED:
YES

TARGET_ASTAR_RAW_POINTS:
20

TARGET_G_COUNT:
11

TARGET_P_COUNT:
14

TARGET_PIECE_COUNT:
15

TARGET_LOCAL_SFC_PLANES:
0

TARGET_NATIVE_STATIC_ROWS:
0

TARGET_SIDE_CORRIDOR_ACTIVE:
YES; radius=0.500 m, samples=25, hard rows=150

TARGET_INITIAL_DURATION:
2.929840 s full reconstructed trajectory; local repaired interval=1.054840 s

TARGET_RETIMED_DURATION:
14.235929 s

TARGET_INITIAL_MAX_V:
10.340211 m/s

TARGET_INITIAL_MAX_A:
104.933983 m/s^2

TARGET_INITIAL_MAX_J:
2523.763106 m/s^3

TARGET_RETIMED_MAX_V:
2.128076 m/s

TARGET_RETIMED_MAX_A:
4.444592 m/s^2

TARGET_RETIMED_MAX_J:
21.998234 m/s by outer checker; 22.393653 m/s by SCP hard-row lattice

TARGET_SCP_OUTER_ITERS:
1 (iteration 0 only)

TARGET_OSQP_SOLVES:
3 main QP solves; no feasibility probes

TARGET_FINAL_QP_STATUS:
MAX_ITER, 2000 iterations on all three attempts

TARGET_FINAL_FAILURE_REASON:
QP_MAX_ITER_EXHAUSTED

PRIMARY_BOTTLENECK_ROW_FAMILY:
NUMERICAL

SECONDARY_BOTTLENECK:
JERK hard rows; authoritative lattice violation=0.393653 m/s^3, plus dense corridor/dynamics row system

QP_PRIMAL_INFEASIBLE:
NO — OSQP did not report it; mathematical feasibility was not formally proven because no no-trust probe ran

QP_MAX_ITER_PRIMARY:
YES

MODEL_MISMATCH_PRIMARY:
NO — model-agreement was never reached

TRUST_COLLAPSE_PRIMARY:
NO — trust shrink followed MAX_ITER and did not recover convergence

TIME_INITIALIZATION_PRIMARY:
NO — original timing was pathological, but retiming mostly repaired it; boundary-margin mismatch is secondary

LOCAL_SFC_OVERCONSTRAINT_PRIMARY:
NO

SIDE_CORRIDOR_OVERCONSTRAINT_PRIMARY:
NO

NATIVE_STATIC_CONFLICT_PRIMARY:
NO

DYNAMICS_HARD_ROWS_PRIMARY:
NO — jerk is the sole physical violation but not evidence of true infeasibility

JERK_PRIMARY:
NO — secondary/triggering bottleneck

NUMERICAL_SCALING_PRIMARY:
YES, with medium-high confidence; exact matrix condition number unavailable

PROMISING_GEOMETRY_KILLED_BY_BACKEND:
YES — within the actually evaluated static and 2 s dynamic contract

WOULD_TWO_STAGE_SCP_LIKELY_HELP:
PARTIAL — evidence favors a feasibility-first T-only stage, not a P-first geometry stage

BEST_MINIMAL_FIX_DIRECTION:
Keep the repaired P/guide fixed and run a feasibility-first tau/T correction on the exact SCP jerk lattice before the full native-objective QP; preserve a fully rechecked feasible initializer if the later objective QP only reaches MAX_ITER.

PRODUCTION_CODE_CHANGED:
NO

NEW_EXPERIMENT_RUN:
NO

CURRENT_FEEDBACK_FILE:
/home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_19.md
```

## 13. Source/Log evidence index

### Source

- Dynamic risk center-distance and horizon: `plan_manage/src/planner_manager.cpp:845-901`
- Candidate safety classification: `planner_manager.cpp:1738-1767`
- A* raw path, simplification and reinsertion: `planner_manager.cpp:2700-2880`
- Local-SFC construction/handoff: `planner_manager.cpp:2888-3054`
- repaired trajectory to MINCO P/T: `planner_manager.cpp:3477-3488`
- SIDE uniform retiming: `planner_manager.cpp:3568-3655`
- rerisk after retiming: `planner_manager.cpp:3670-3701`
- SIDE SCP handoff: `planner_manager.cpp:3836-4053`
- outer dynamics checker/tolerance: `planner_manager.cpp:1079-1100`
- emergency previous/fallback selection: `planner_manager.cpp:4721-4755`
- SCP dynamics lattice: `traj_opt/src/poly_traj_optimizer.cpp:905-985`
- hard corridor rows: `poly_traj_optimizer.cpp:1332-1382`
- Local-SFC rows: `poly_traj_optimizer.cpp:1385-1433`
- native static rows: `poly_traj_optimizer.cpp:1436-1484`
- trust rows: `poly_traj_optimizer.cpp:1618-1643`
- QP Hessian and solve: `poly_traj_optimizer.cpp:1759-1786`
- MAX_ITER/trust handling and probe gating: `poly_traj_optimizer.cpp:1801-1951`
- OSQP settings/status extraction: `traj_opt/src/scp_optimizer.cpp:119-200`

### Historical log

- expired trajectory and fresh init: `alp.launcher.log:44810-44816`
- SIDE seed/static window/A*: `alp.launcher.log:44818-44830`
- repair timing/retiming: `alp.launcher.log:44826-44883`
- risk distance 1.714893: `alp.launcher.log:44884`
- hard-row/gradient scale: `alp.launcher.log:44885-44891`
- three OSQP attempts: `alp.launcher.log:44891-44905`
- final PLUS failure: `alp.launcher.log:44906-44911`
- MINUS failure and final selection: `alp.launcher.log:44912-44943`
- NOMINAL fallback commit/publish: `alp.launcher.log:44944-44954`
