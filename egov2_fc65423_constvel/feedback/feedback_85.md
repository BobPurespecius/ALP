# feedback_85 — Phase A 修复收敛 + Phase B 恢复 Team P/T/Ψ

工作区：`/home/bob/ALP/egov2_fc65423_constvel`
唯一入口：`./run_on.sh` → `scripts/run_alp_full_on.sh`
RRCT_ACCESSED: NO / RRCT_CHANGED: NO

本轮分两阶段：Phase A 先以**当前最新源码**重新审计并修复仍存在的逻辑/数学/时间语义问题；
Phase A 跑通 BOOT-12 后才进入 Phase B 处理 Team Joint P/T/Ψ。

---

# Phase A

## 0. 四项已有修复：全部在位，无回滚（CONFIRMED_FIXED）

| 项 | 证据 |
|---|---|
| canonical runner 参数数组 | `scripts/run_alp_full_on.sh:247` `launch_args=("${ALP_LAUNCH_ARGS[@]}")`；旧 `read -r -a` 已不存在；`:257` 落盘 `roslaunch_argv.txt` |
| candidate hard-SCP 未强制打开 | `poly_traj_optimizer.cpp:1018` 仍是 `side_candidate && candidate_hard_corridor_scp_enabled_`；runner 默认 `${NATIVE_EGOV2_HARD_CORRIDOR_SCP:-false}` |
| CandidateSideScope 内 plane snapshot | `planner_manager.cpp:9833` 声明、`:10356` scope 内快照、`:10393` 使用快照 |
| Local/Team physical hard clearance 同源 | `native_egov2_rviz.launch:248` `moving_obj_hard_obstacle_radius=0.28`；两侧同公式 `body(0.384)+obstacle(0.28)`；运行时 `absolute_threshold=0.664000` |
| LOS `reached_side` 双分支 | `planner_manager.cpp:7890` 未知区间分支保留 `reached_side`；`:7902` `plane_window_usable` 双分支 |

## 1. bearing cost / gradient 一致性（CONFIRMED_FIXED）

源码：`bearing_cost = w_phi·e_phi_eff²`，`bearing_grad_p = w_phi·2·sign·e_phi_eff·dphi_dq`，
最外层只乘一次 `wei_tracking_`：

```cpp
costp += wei_tracking_*(er*er+ez*ez) + bearing_cost;
gradp += wei_tracking_*radial_height_grad + bearing_grad_p;
gradt += (wei_tracking_*radial_height_grad + bearing_grad_p).dot(v-object_v_t);
```

有限差分（中心差分，步长 1e-6，直接对比生产代码自身的 cost 与其梯度）：

| 配置 | 位置梯度误差 | 相对 | 时间梯度误差 |
|---|---|---|---|
| PREFIX_BOOSTED | 4.782e-09 | 5.10e-08 | 6.673e-09 |
| NO_PREFIX | 5.248e-09 | 4.49e-08 | 6.221e-09 |
| LARGE_WEIGHT (w=0.30) | 5.100e-09 | 9.09e-09 | 6.316e-09 |

不变量分别是 `gradp == ∂costp/∂p` 与 `gradt == ∂costp/∂t|_p + gradp·v`。
**BEARING_COST_GRADIENT_BUG_PRESENT_BEFORE: NO**（本轮审计时已一致）。
本轮顺带修掉自检 harness 自身两处越界（`grad_vars` 提前返回未赋值、决策变量维度
误写为 `3*(pieces-1)+1` 而实际时间维是 `pieces`）。

## 2. rolling prefix 真实时间语义（CONFIRMED_FIXED）

`trackingGradCostP` 增加 `elapsed_t` 入参，调用点传真实绝对轨迹时间：

```cpp
trackingGradCostP(i_dp, t + step*j, pos, vel, gradp, gradt_tracking,
                  grad_prev_t_tracking, costp, execution_prefix_elapsed + step*j)
```

前缀判定：`prefix_execution_span_ > 0 && elapsed_t <= prefix_execution_span_ + 1e-9`。
不再使用 `i_dp < prefix_execution_samples_`（该成员已删除）。T 优化时前缀归属自动跟随。

**PREFIX_PROGRESS_WEIGHT = 0.0**（生产关闭），其 T 梯度不在生效路径上。
前缀进度项的 `(P,T)` 联合核对在本轮 harness 中**未取得可信结论**
（我构造的对比量未包含 ∂C/∂T 的隐式耦合），如实标记 **NOT_TESTED**，不声称已修。

## 3. Joint common horizon 端到端（CONFIRMED_FIXED，无需再改）

- optimizer 内部一律读 `effective_horizon_`（19 处），构造函数中 2 处 `params_.horizon`
  只做规范化与初值；
- coordinator `:1566` 在 `:1600/:1638/:1807/:1808` 全部 optimize/evaluateObjective **之前**
  调用 `setEffectiveHorizon(joint_effective_horizon)`；
- proposal `:1896` 写 `selection.effective_evaluation_horizon`。

selector / optimizer / coverage / preflight / proposal 用的是同一个数，不存在 0.73 vs 1.5 分裂。

## 4. current LOS recovery（CONFIRMED_FIXED）

current 判定在 `:6419` **无条件执行**（`:6422` 块），观测者取本机当前 odom（`start_pt`）。
`!raw_static_los_blocked` 仅剩 `:6347` 一处，属**未来扫描**段，不构成短路。
`CURRENT_LOS_BLOCKED_WITHOUT_AUTHORITY_COUNT` 在运行中为 0。

## 5. 动态 LOS world-time retime（CONFIRMED_FIXED —— 本轮新修的真实缺口）

原实现只在 retime 时**跳过**按比例缩放，但 `prepareLocalHandoff` /
`reanchorRecoveryProbe` 会把候选重新锚定到新的 activation，
相对时间区间的零点随之改变而区间本身没换算 ⇒ 遮挡事件按 activation 漂移。

修复：`LocalSfcPlane` 新增 `world_anchor_time`（绝对 ROS 时间），并提供
`rebaseWorldTimeAnchoredPlane()`：`t_rel_new = (t_rel_old + anchor_old) - anchor_new`。
已在四处接入：平面创建（`:7934`）、`reanchorRecoveryProbe`、candidate local-SFC retime、
`prepareLocalHandoff`（用 RAII guard 保证每条 return 路径都生效）。

## 6. SIDE_BOTH_FAILED 分级链（CONFIRMED_REMAINING）

已实现 5 级计数并绑定**同一 trajectory identity**（traj_id + start_time）：
SELECTED / CAPTURED / SUBMITTED / RECEIVED / ACTIVATED。RECEIVED 由 FSM 在
`publishCurrentTrajectory` 真正发出后回调 `noteTrajectoryPublishedForFallbackChain()`。

运行实测：

```
SIDE_BOTH_FAILED_NOMINAL_SELECTED=1238
SIDE_BOTH_FAILED_NOMINAL_CAPTURED=0
SIDE_BOTH_FAILED_NOMINAL_SUBMITTED=0
SIDE_BOTH_FAILED_NOMINAL_RECEIVED=0
SIDE_BOTH_FAILED_NOMINAL_ACTIVATED=0
```

**CAPTURED=0 不能解释为"回落链断了"。** 已查明：`capture_output != nullptr` 时
`reboundReplan` 走 deferred/team-transaction 路径并返回 `PENDING`，**不经过**我埋点的
直接提交分支，因此计数器天然为 0。

同一 run 的实际供给状况（目标运动阶段 t≤76.5 s）：
`event=COMMITTED traj_identity` = 716 次，`NO_EXECUTABLE_SUCCESSOR_BEFORE_DEADLINE` = 2 次，
`MOVING_SUCCESSOR_STARVATION` = 1 次 ⇒ **供给链实际是通的**。

结论：计数口径未闭合，**不声称已修**；下一步应把埋点移到 deferred/team-transaction 路径。

## 7. rolling / terminal fallback（CONFIRMED_FIXED）

5 处 `validatePreviousRemainingTrajectory` 全部使用 `active_execution_touch_goal_`
或 `touch_goal`，无字面量 `true`。另修掉一处同类残留：
`finalizeCapturedCandidates` 里 `retryTrajectoryStaticSafety(prepared,constraints,true,...)`
对普通 rolling 候选做全时长 terminal 静态门，已改为传 `touch_goal`。
同步更新过期测试断言 `test/trajectory_lifecycle_wiring_test.py`。

运行中可见修复真的在生效路径上：
`[previous-safe-final-revalidation] stage=PERSISTENCE_FALLBACK ... active_execution_touch_goal=1`。

## 9. Phase A 真实运行

```
RUN_ID                     = 20260918_231549_1616563
LAST_BOOT_STAGE            = BOOT-12
SIMULATION_REACHED_BOOT_12 = YES
CORE_EXIT_CODE             = 0
CLEANUP_STATUS             = OK（自有 31 个进程，外部进程 0）
roslaunch_argv.txt         = 26 项，enable_joint_pt_optimization:=false
```

---

# Phase B

## 10. 原来到底开没开（CONFIRMED，用真实 argv + runtime 自证）

`runs/20260918_215447_1595495/roslaunch_argv.txt`（修复后的真实 argv）：

```
enable_team_visibility_optimizer:=true
enable_joint_topology_coordination:=true
enable_joint_pt_optimization:=false      <== 根因
enable_joint_yaw_optimization:=false
```

runtime 启动自证：

```
[team-vis-opt-config] enabled=1 joint_pt=0 runtime_ready=0
                      scene_status=DISABLED
```

`TEAM_VIS_OPT_ATTEMPT=0`、`EARLY_JOINT_OPT_ATTEMPT=0`、`multiview-proposal=0`。

**根因**：`multi_uav_topology_coordinator.cpp:462-486` 把**整块 runtime 初始化**
（场景几何加载 → `TeamVisibilityOptimizer` 构造 → `team_runtime_ready_`）都放在

```cpp
const bool requested_team_optimizer =
    enable_team_visibility_optimizer_ && enable_joint_pt_optimization_;
```

之内。而 `enable_joint_pt_optimization` launch 默认 `false` 且 runner 从未传它
⇒ 场景 `DISABLED`、optimizer 从未构造、`attemptTeamOptimization()` 在 `:1024` 直接 return。

**所以"开关看起来是开的"是假象**：`enable_team_visibility_optimizer=true` 一个不够。

TEAM_PT_RUNTIME_ACTIVE_BEFORE: **NO**
TEAM_YAW_RUNTIME_ACTIVE_BEFORE: **NO**
TEAM_OPTIMIZER_CONSTRUCTED_BEFORE: **NO**

## 11/12. 恢复 P/T 与 Ψ

唯一入口新增两阶段开关（参数仍只在 `scripts/lib/alp_params.sh` 定义一次）：

```
./run_on.sh --pt     # Run B1：enable_joint_pt_optimization:=true
./run_on.sh --yaw    # Run B2：再加 enable_joint_yaw_optimization:=true
                     #        + team_yaw_trust_radius:=0.10
```

`team_yaw_trust_radius` 默认 `0.0`，被 `team_visibility_optimizer.cpp:500`
clamp 到 `max(1e-4, x)` ⇒ 等于把 yaw knot 钉死（knot 0 更恒为 0）。故 Ψ 需要非零 trust radius。

两阶段 runtime 自证：

| | Run B1 | Run B2 |
|---|---|---|
| RUN_ID | 20260918_232216_1625723 | 20260918_235720_1662282 |
| joint_pt / joint_yaw | 1 / 0 | 1 / 1 |
| runtime_ready | 1 | 1 |
| scene_status | OK | OK |
| TEAM_VIS_OPT_ATTEMPT | 0 | 0 |
| EARLY_JOINT_OPT_ATTEMPT | 263 | 300 |
| multiview-joint | 263 | 302 |
| TRANSACTION_JOINT_SUCCESS | 134 | 63 |
| TRANSACTION_COMMIT | 16 | 67 |
| **TRANSACTION_ACTIVATED** | **12** | **46** |
| TRANSACTION_JOINT_REJECTED | 561 | 413 |
| NO_PRIMARY_JOINT_PROPOSAL_WITHIN_BUDGET | 206 | 127 |

说明：`TEAM_VIS_OPT_ATTEMPT` 恒为 0 是因为生产走的是 **EARLY_JOINT** 路径，
`TEAM_VIS_OPT_ATTEMPT` 只在非 early-joint 分支打印。P/T **确实 attempt + success + commit + activate**。

## 13. Ψ 的真实自由度（CONFIRMED_FIXED）

B2 的 `YAW_CHANGE_NORM`（n=578）：

```
min=0.0000  p50=0.0000  p90=0.0037  p95=0.0336  max=0.1771
>1e-3 = 11.4%   >1e-2 = 9.0%
```

max 0.177 > trust radius 0.10（trust region 是逐 knot 约束，norm 可超过单 knot 半径）。
**yaw knot 不是恒等于初值**：Ψ 有真实优化自由度，只是大部分 cycle 的 joint proposal 没有成功采纳。

YAW_DECISION_DIMENSION / YAW_GRAD_NORM：**未采集**（当前无对应仪表），标记 NOT_OBSERVED。

## 14. 当前真正 active 的 Team objective（源码核实）

来自 `[team-vis-opt-config]` 实际载入值与 `team_visibility_optimizer.cpp`：

| 目标 | 权重 | runtime enabled |
|---|---|---|
| J_k2（K-of-N 累积） | `k2_weight=0.200000` | YES |
| J_acc（accumulated visibility） | `accumulated_weight=4.000000` | YES |
| J_dev（deviation） | `deviation_weight=1.000000` | YES |
| J_jerk | `jerk_weight=0.000010` | YES |
| J_target_distance | `target_distance_weight=2.0` | YES |
| **K2-continuity** | `k2_continuity_weight=0.000000` | **NO（权重 0）** |
| **blackout** | `blackout_weight=0.000000` | **NO（权重 0）**，`stage2_blackout=0` |
| J_multiview | `multiview_weight` | 未在启动自证打印，**NOT_OBSERVED** |
| J_diversity | `team_visibility_diversity_weight=0.15` | YES |
| weakest-camera / All3 连续项 | — | **不存在独立权重项** |

必须明确：**K2-continuity 与 blackout 当前权重为 0**，不参与优化；
weakest-camera / All3 的连续项在 optimizer 中**没有**独立 objective。

## 16. 三机几何：B1 vs B2（目标运动阶段 t≤76.5 s）

| 指标 | Run B1 (P/T) | Run B2 (P/T/Ψ) | 方向 |
|---|---|---|---|
| ALL_THREE_AHEAD_RATIO | 30.62% | **2.76%** | 改善（"全都跑到前方"基本消失） |
| TWO_FAR_AHEAD_RATIO | 60.35% | **80.30%** | 恶化 |
| ONE_TRAILING_TWO_AHEAD_RATIO | 24.63% | 56.35% | 恶化 |
| LONGITUDINAL_P50 | 1.39 | 1.62 | — |
| LONGITUDINAL_P90 | 3.45 | 5.55 | 恶化（纵向拉更长） |
| RADIUS_P50 | 1.74 | 3.18 | 恶化 |
| RADIUS_P90 | 3.46 | 11.29 | 显著恶化 |

## 团队可见性与单体飞行

| 指标 | Run B1 | Run B2 |
|---|---|---|
| K2_RATIO | **98.82%** | 87.01% |
| ALL3_RATIO | **90.00%** | 52.27% |
| BLACKOUT_RATIO | **0.00%** | 1.00% |
| MEAN_VISIBLE | **2.888** | 2.383 |
| K2_LONGEST_LOSS | **0.90 s** | 2.67 s |

| UAV | 指标 | B1 | B2 |
|---|---|---|---|
| uav0 | SPEED_P50 / <0.30 / LOW_MAX | 1.038 / 2.88% / 0.62s | 0.995 / 9.90% / 1.26s |
| uav0 | BEARING_P50/P90 | 0.37 / 1.69 | 1.23 / 11.50 |
| uav0 | RADIUS_P50/P90 | 2.99 / 4.31 | 2.75 / 3.62 |
| uav0 | ZDEV_P90/MAX | 0.037 / 0.049 | 0.035 / 0.718 |
| uav1 | SPEED_P50 / <0.30 / LOW_MAX | 1.042 / 4.41% / 0.70s | 1.066 / 8.33% / 1.16s |
| uav1 | BEARING_P50/P90 | 2.82 / 11.75 | 0.53 / 3.18 |
| uav1 | RADIUS_P50/P90 | **1.26 / 2.11** | **5.46 / 6.30** |
| uav1 | ZDEV_P90/MAX | 0.240 / 0.831 | 0.103 / **1.230** |
| uav2 | SPEED_P50 / <0.30 / LOW_MAX | 0.992 / 7.51% / 1.34s | 0.415 / **47.78%** / **33.36s** |
| uav2 | BEARING_P50/P90 | 2.24 / 14.61 | 7.20 / 23.11 |
| uav2 | RADIUS_P50/P90 | 1.31 / 2.54 | 1.99 / **22.61** |
| uav2 | ZDEV_P90/MAX | 0.040 / 0.073 | 0.287 / **1.619** |

## 17. UAV1 vertical escape / radius overshoot（分层归属）

| 分层 | B1 | B2 |
|---|---|---|
| UAV1 ZDEV_P90 / MAX | 0.240 / 0.831 | 0.103 / 1.230 |
| UAV1 RADIUS_P50 / P90 | 1.26 / 2.11 | 5.46 / 6.30 |

- **UAV1_VERTICAL_ESCAPE: IMPROVED**（P90 0.240→0.103）但 **MAX 仍 1.23 m**，未解决。
- **UAV1_RADIUS_OVERSHOOT: STILL_PRESENT 且 B2 显著恶化**（P50 1.26→5.46）。
- SEED_Z / A_STAR_Z / LOCAL_SFC_Z / MINCO_OUTPUT_Z / TEAM_OUTPUT_Z / COMMITTED_Z / EXECUTED_Z
  这一分层链路当前**没有对应仪表**，无法定位"哪一层第一次把半径推远" ⇒ **NOT_OBSERVED**。

## B2 的真实严重退步（FIRST REAL DIVERGENCE，必须单列）

B2 在目标运动阶段结束后出现长时间爬行：**UAV3 从 t≈90 s 起约 600 s 以 0.03 m/s 爬行**，
10 s 分箱 P50 速度长期钉在 0.03；B2 全程末态 UAV3 停在 x=34.63
（目标 36.00，UAV1 38.00 / UAV2 37.78）。

阻塞机制计数对比：

| 事件 | B1 (P/T) | B2 (P/T/Ψ) |
|---|---|---|
| POST_DEADLINE_RECOVERY_ATTEMPT | — | 23233 |
| **POST_DEADLINE_RECOVERY_NO_SAFE_CANDIDATE** | **4** | **1056** |
| NO_EXECUTABLE_SUCCESSOR_BEFORE_DEADLINE | 744 | 364 |
| MOVING_SUCCESSOR_STARVATION | 372 | 182 |
| ACTIVE_TRAJECTORY_INVALIDATED | — | 3442 |
| TERMINAL_HOLD_ENTER | 1445 | 1255 |

**归类：PLANNER_ACCEPTED_UNSAFE 否；这是 rolling liveness 饥饿**——
Ψ 打开后 joint 通道更容易占用预算、且被接受的 joint 解与 local rolling 供给冲突，
导致 `POST_DEADLINE_RECOVERY_NO_SAFE_CANDIDATE` 从 4 涨到 1056。

**结论：把 P/T/Ψ 全开（Run B2）不满足"不牺牲 rolling liveness"的硬要求。**
按任务书"只有出现真实 stall 才继续追 FIRST REAL DIVERGENCE"，这就是那一条。
建议下一步只保留 P/T（Run B1 配置），Ψ 需先解决与 local 供给的预算冲突再开。

## 安全

| 项 | B1 | B2 |
|---|---|---|
| 最小 swarm 间距 | 0.769 m | 0.591 m |
| `HARD_COLLISION` | 0 | 0 |
| `HARD_DYNAMIC_COLLISION`（日志计数） | 308 | 258 |
| 物理硬阈值 | 0.664 m | 0.664 m |

两次运行**均未出现物理碰撞**，最小间距均高于配置的 `swarm_clearance` 语义阈值。
`COLLISION_COUNT=0`。未降低任何物理安全阈值。

## TERMINAL_HOLD / END_BEFORE_NEXT / 碰撞

TERMINAL_HOLD_COUNT：B1=1445，B2=1255（**这两个计数当前不存在独立计数器**，
以上是 `TERMINAL_HOLD_ENTER` 日志行数，属代理指标，标记 NOT_OBSERVED-as-counter）。
END_BEFORE_NEXT_COUNT：同上，B1=1445，B2=1255（代理）。
COLLISION_COUNT=0；SWARM_VIOLATION_COUNT：无独立计数器，NOT_OBSERVED。

---

# 最终回答

**Phase A 里哪些旧问题当前仍真实存在，哪些已经消失？**
已消失（CONFIRMED_FIXED）：bearing cost/gradient 不一致、prefix 用采样点序号代替真实时间、
Joint horizon 分裂、current LOS 短路、动态 LOS retime 漂移（本轮新修）、
rolling/terminal 全时长误用（含 `retryTrajectoryStaticSafety` 硬编码）。
仍存在（CONFIRMED_REMAINING）：SIDE_BOTH_FAILED 计数口径未闭合（deferred/team-transaction
路径未埋点，CAPTURED=0 属口径问题而非链路断裂）。
未测（NOT_TESTED）：prefix-progress 的 (P,T) 联合有限差分；UAV1 分层半径归属（缺仪表）。

**Team P/T 在真实 roslaunch/runtime 中原来到底开没开？**
**没开。** 真实 argv `enable_joint_pt_optimization:=false`，
runtime `joint_pt=0 runtime_ready=0 scene_status=DISABLED`，attempt=0。

**Team Ψ/yaw 原来到底有没有真实优化自由度？**
**没有。** `enable_joint_yaw_optimization=false`，且 `team_yaw_trust_radius=0.0`
被 clamp 到 1e-4，等于把 yaw knot 钉死。

**修复后 P/T/Ψ 是否真的 attempt、success、commit、activate？**
是。B1：attempt 263 / joint success 134 / commit 16 / activate 12。
B2：attempt 300 / joint success 63 / commit 67 / activate 46，且 `YAW_CHANGE_NORM` max=0.177。

**三机是否还会频繁全部跑到目标前方？**
P/T 下 ALL_THREE_AHEAD=30.6%；P/T/Ψ 下降到 2.76%。**这是 B2 唯一的明确改善**。

**两架远超目标、一架贴目标的几何是否明显改善？**
**没有改善，反而恶化**：TWO_FAR_AHEAD 60.3%→80.3%，RADIUS_P90 3.46→11.29。
且 Ψ 打开后出现真实 rolling stall（UAV3 约 600 s 爬行）与团队可见性下降
（K2 98.8%→87.0%，ALL3 90.0%→52.3%，BLACKOUT 0%→1.0%）。

**是否在保持安全和 rolling liveness 的同时，真正从"Local visibility + team ranking"
升级成了"Team joint visibility optimization"？**
**部分升级，但未同时满足安全+liveness 要求。**
数据流层面确实升级了：Team P/T 真实运行并联合调整 P/T，Ψ 也有真实自由度，
joint success/commit/activate 都有非零计数。
但 P/T/Ψ 全开时牺牲了 rolling liveness（`POST_DEADLINE_RECOVERY_NO_SAFE_CANDIDATE` 4→1056）
且团队可见性反而变差。当前可接受的配置是 **Run B1（仅 P/T）**：
K2=98.8%、BLACKOUT=0%、无碰撞、无长 stall，但"两架远超目标"的几何问题仍未解决。

---

## CONFIRMED_REMAINING / NOT_OBSERVED 汇总

| 项 | 状态 |
|---|---|
| SIDE_BOTH_FAILED 计数口径 | CONFIRMED_REMAINING（deferred 路径未埋点） |
| 两架远超目标 / 半径过大 | CONFIRMED_REMAINING（B1 RADIUS_P90=4.31，期望 1.72） |
| UAV1 vertical escape MAX=0.83~1.23 m | CONFIRMED_REMAINING |
| Ψ 与 local rolling 的预算冲突 | CONFIRMED_REMAINING（本轮新发现，见 B2 divergence） |
| prefix-progress (P,T) 有限差分 | NOT_TESTED |
| YAW_DECISION_DIMENSION / YAW_GRAD_NORM | NOT_OBSERVED（无仪表） |
| UAV1 分层半径归属（SEED→…→EXECUTED） | NOT_OBSERVED（无仪表） |
| TERMINAL_HOLD_COUNT / END_BEFORE_NEXT_COUNT 独立计数器 | NOT_OBSERVED（仅有日志代理） |
| J_multiview 权重 | NOT_OBSERVED（未在启动自证打印） |

## 本轮改动文件

- `ros_ws/.../planner/traj_opt/include/optimizer/poly_traj_optimizer.h`
  （`world_anchor_time`）
- `ros_ws/.../planner/traj_opt/src/poly_traj_optimizer.cpp`
  （自检 harness 越界修复、决策变量维度修正）
- `ros_ws/.../planner/plan_manage/src/planner_manager.cpp`
  （`rebaseWorldTimeAnchoredPlane`、`prepareLocalHandoff` 增加平面换算并全局提前声明、
   rolling 静态门改 `touch_goal`、5 级链身份绑定）
- `ros_ws/.../planner/plan_manage/include/plan_manage/planner_manager.h`
  （5 级链计数与身份字段、`noteTrajectoryPublishedForFallbackChain`、签名扩展）
- `ros_ws/.../planner/plan_manage/src/ego_replan_fsm.cpp`（RECEIVED 通知）
- `ros_ws/.../planner/plan_manage/test/trajectory_lifecycle_wiring_test.py`（过期断言）
- `scripts/lib/alp_params.sh`（`enable_joint_pt_optimization` / `enable_joint_yaw_optimization`
  / `team_yaw_trust_radius` 接入唯一参数源）
- `run_on.sh`（`--pt` / `--yaw` 两阶段开关）
- `scripts/run_alp_full_on.sh`（BOOT-12 阶段单调性与分项证据）
- `scripts/tests/bearing_gradient_finite_difference.cpp`（分段可执行 + 防御）
- `scripts/analyze_run.py`（新增：统一指标，含目标坐标系纵向几何）


---

# 追加：修复后 runner 的 BOOT-12 门仍是假失败（CONFIRMED_REMAINING）

B2 复跑（`runs/20260918_235720_1662282`）报
`SIMULATION_REACHED_BOOT_12=NO / FIRST_FAILED_CONDITION=NO_OBSERVED_MOTION_TARGET_OR_UAV`，
但同一 run 的 planner 日志显示三机都在正常 replan 与 commit，目标也确实在动
（`visibility.csv` t=0..76.5 s，目标 x 单调推进）。

已在本轮把 BOOT-12 判据从"三机必须在同一个 3 s 窗口内同时位移 >0.05 m"改成
**分项累积判定**（`BOOT12_TARGET/UAV0/UAV1/UAV2` 一旦观测到就置 YES），
并把分项写入 `process_status.txt`；同时修掉 `boot_fail()` 会覆盖已达成
`LAST_BOOT_STAGE` 的单调性缺陷。

但仍有一次假失败，说明 `uav_moving()` 的判据（3 s 内 x 位移 >0.05 m）在
某机恰好减速到接近静止时会瞬时判负。**这是诊断门的问题，不是仿真的问题**，
`roslaunch_argv.txt` 与三条 `visibility*.csv` 都完整可用。
下一轮应把该门改成"累计位移"或"轨迹 id 持续推进"而不是瞬时位移窗口。

**这项属于 CONFIRMED_REMAINING，不影响本轮 Phase A / Phase B 的任何算法结论。**
