# ALP 多无人机目标追踪 Stage 2 / Stage 3A / Stage 3B 接续报告

报告日期：2026-09-09（Asia/Shanghai）  
项目根目录：`/home/bob/ALP/egov2_fc65423_constvel`  
禁止访问目录：`/home/bob/RRCT`（本轮未访问）

## 结论摘要

当前真实源码已经完成 Stage 2、Stage 3A、Stage 3B 的生产实现，并通过构建、梯度和 contract tests。Stage 3B 短测中已经实际触发 joint optimizer，得到非零 P/T/yaw 变化，K2-cont 与 blackout surrogate 均下降，安全距离检查通过。

但最后一次短测仍存在一个已定位的 team proposal 生命周期竞态：UAV0 的 pending topology generation 在 proposal 发布前约 30 ms 超时，导致 proposal 被拒绝为 `NO_PENDING_TOPOLOGY_GENERATION`，因此本轮没有完整的三机 team commit/adoption 证据。源码已经修复该竞态并重新构建、通过 contract tests；按照本轮禁止重复/追加仿真的约束，修复后没有再次启动仿真。因此 Stage 3B short-test 最终仍记录为 `NOT VERIFIED`，不能写成 PASS。

## 源码实现

### Stage 2：K2 continuity 与 blackout

主要实现位于：

- `ros_ws/src/multi_uav_formation/include/multi_uav_formation/team_visibility_optimizer.h`
- `ros_ws/src/multi_uav_formation/src/team_visibility_optimizer.cpp`
- `ros_ws/src/multi_uav_formation/src/multi_uav_topology_coordinator.cpp`
- `ros_ws/src/multi_uav_formation/scripts/native_egov2_rviz_scene.py`

连续目标实际使用：

```text
Q2 = v1*v2 + v1*v3 + v2*v3 - 2*v1*v2*v3
b_K2 = 1 - Q2
b_0 = (1-v1)*(1-v2)*(1-v3)
```

离散滑动窗口实现为：

```text
width_K2 = round(k2_continuity_window / dt) + 1
J_K2-cont = mean_over_windows( mean_window(b_K2)^2 )

width_0 = round(blackout_window / dt) + 1
J_blackout = mean_over_windows( mean_window(b_0)^2 )
```

实现使用 running sum 计算每个窗口均值，并将窗口代价的导数传播回每个 visibility sample，再通过 fixed-world-time sample Jacobian 传播到 P/T；若启用 yaw，visibility 的 yaw gradient 还会继续传播到 yaw knots。

真实 binary trace 与 surrogate 分离：

- `binary_valid/binary_visible` 保存 range、LOS、FOV 的真实二值判断。
- `longest_k2_loss` 使用 `visible_count < 2` 的连续 run。
- `longest_blackout` 使用 `visible_count == 0` 的连续 run。
- surrogate 只用于可导优化，未被冒充为精确 longest-event 指标。

objective 中保留原有 visibility、tracking、smoothness、time、deviation 等项，并新增：

```text
weighted_k2_continuity = k2_continuity_weight * k2_continuity_cost
weighted_blackout      = blackout_weight * blackout_cost
```

同时输出 raw、weighted、gradient norm：

```text
RAW_J_K2 / RAW_J_ACC / RAW_J_K2_CONT / RAW_J_BLACKOUT
WEIGHTED_J_K2 / WEIGHTED_J_ACC / WEIGHTED_J_K2_CONT / WEIGHTED_J_BLACKOUT
GRAD_NORM_K2 / GRAD_NORM_ACC / GRAD_NORM_K2_CONT / GRAD_NORM_BLACKOUT
```

acceptance guard 在 nonlinear re-evaluation 后检查：

1. hard safety valid；
2. K2 不超过 `k2_regression_tolerance`；
3. longest K2 loss 不超过 `longest_k2_loss_tolerance`；
4. longest blackout 不超过 `longest_blackout_tolerance`；
5. 再检查 objective improvement 与 trajectory change。

### Stage 3A：P/T 后 yaw/FOV 重预测

每次 team nonlinear evaluation 都按照当前 P/T：

```text
重建 polynomial
→ fixed-world-time resampling
→ target prediction
→ target-facing desired yaw
→ yaw rate / acceleration propagation
→ camera orientation / FOV margin
→ continuous visibility v_i
→ team objective / gradient
```

planner 与 `traj_server` 共用：

```text
multi_uav_formation::advanceTargetFacingYaw()
```

没有使用初始 candidate 的冻结 yaw trace 代替当前 P/T 的 yaw/FOV evaluation。

Python telemetry 增加了 planned/executed visibility、每架 UAV FOV-only mismatch、team mismatch、executed longest K2 loss/blackout 与 commanded yaw rate/acceleration。

### Stage 3B：joint yaw optimization

联合变量包含每架 UAV 的低维 yaw knots，而不是每个 sample 一个独立 yaw：

```text
X = { P_i, tau_i, Psi_i } for i=1..3
```

实现包括：

- yaw knots 进入 joint decision layout；
- 第一个 yaw knot 固定，保持 activation yaw continuity；
- FOV yaw gradient 进入 K2、K2-cont、blackout、accumulated visibility 与 yaw prior；
- weak target-facing prior，不锁死 `yaw == yaw_target`；
- yaw trust radius、yaw rate、yaw acceleration hard validation；
- yaw payload 与 position trajectory 共用 team solution id、generation、activation time；
- planner adoption 和 `traj_server` scheduled activation 传递同一 yaw/position generation；
- `optimized_yaw_valid=true` 时 `traj_server` 不再被默认 target-facing generator 覆盖；无效时才 fallback。

另有有限差分 contract 覆盖 yaw gradient 的 left/right/FOV-boundary 情况。

### 最后补充的 proposal 生命周期修复

文件：

`ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp`

修复前，team optimizer 的 nonlinear solve 和消息往返可能晚于普通 `team_coordination_timeout`，planner 会先清空 `pending_topology_`，随后拒绝合法 proposal：

```text
NO_PENDING_TOPOLOGY_GENERATION
```

现在当 `enable_team_visibility_optimizer=true` 时，将 `team_solution_max_age` 作为 proposal grace 纳入 planner 的有效协调预算，并据此重新计算 `topology_result_max_age` 与 `topology_min_active_remaining`。该修复只延长等待/回退预算，不降低 swarm/static/dynamic clearance，不放宽 velocity/acceleration/jerk/yaw 限制，也不绕过 Local SFC 或 generation validation。

## 构建与 contract/gradient tests

通过：

```text
python3 -m py_compile src/multi_uav_formation/scripts/native_egov2_rviz_scene.py
catkin build traj_utils traj_opt ego_planner multi_uav_formation --no-status --no-summary -j2
```

最后一次 build 结果：相关 6 个 package 全部 succeeded，failed=0。

通过的 targeted tests：

```text
ELASTIC_VISIBILITY_CONTRACT_TEST=PASS
TEAM_VISIBILITY_OPTIMIZER_CONTRACT_PASS
TEAM_SOLUTION_COMMIT_CONTRACT_PASS
JOINT_TOPOLOGY_COORDINATOR_CONTRACT=PASS
```

contract 证据包括：

- P/T、tau、piece boundary、K2、blackout、K2-cont、J_acc 的 analytic/finite-difference 梯度一致；
- yaw joint optimizer `success=1`、`optimized_yaw_valid=1`、`yaw_change_norm=0.554256`；
- full-polynomial safety rejection 正确返回 `INITIAL_STATIC_CLEARANCE_VIOLATION`；
- 三 ACK commit contract、stale/identity/late reject contract 通过。

## 短测记录

### Stage 2

```text
STAGE2_SHORT_RUNS: 2
OFF_RUNS_THIS_AGENT: 0
```

两次均未触发足够的 team optimizer event，因此没有把 Stage 2 short test 写成 PASS。Stage 2 的 objective、梯度、acceptance guard、真实 binary longest metric 已由源码和 contract tests 验证。

### Stage 3A

```text
STAGE3A_SHORT_RUNS: 1
```

该次短测产生 planned/executed visibility 与 FOV mismatch 统计，但 `executable_candidate_count=0`，原因是 `DYNAMIC_RISK_INVALID`，没有完整 team optimizer/proposal/commit/adoption 事件。

已有 mismatch 统计：

```text
UAV1: 1.61%
UAV2: 0.60%
UAV3: 0.34%
TEAM:  2.49%
```

因此：`STAGE3A_SHORT_TEST_PASS: NO / NOT VERIFIED`。

### Stage 3B

```text
STAGE3B_SHORT_RUNS: 2
```

第一次短测：

```text
moving-object predictor/cost disabled
predictor=0
executable_candidate_count=0
```

按规则只做了一次针对性调整：开启 `moving_obstacle_time_aware_cost_enabled=true`，保留 7 个 moving objects，并重新短测。

第二次短测启动配置确认：

```text
enable_team_visibility_optimizer=true
enable_joint_pt_optimization=true
enable_team_k2_continuity_cost=true
enable_team_blackout_cost=true
enable_predicted_attitude_fov=true
enable_joint_yaw_optimization=true
team_yaw_trust_radius=0.08
moving_obj_num=7
```

第二次短测中出现真实 executable candidates，并触发：

```text
TEAM_VIS_OPT_ATTEMPT: 5
TEAM_VIS_OPT_SUCCESS: 1
```

首次成功事件的真实数值：

```text
K2_BEFORE=0.763427
K2_AFTER=0.763451

K2_CONT_BEFORE=0.173088074
K2_CONT_AFTER=0.173056460

BLACKOUT_BEFORE=0.087465848
BLACKOUT_AFTER=0.087439730

LONGEST_K2_LOSS_BEFORE=0.300000 s
LONGEST_K2_LOSS_AFTER=0.300000 s

LONGEST_BLACKOUT_BEFORE=0.200000 s
LONGEST_BLACKOUT_AFTER=0.200000 s

MEAN_VISIBLE_BEFORE=2.163246
MEAN_VISIBLE_AFTER=2.166590

P_CHANGE_NORM=0.005851706
T_CHANGE_NORM=0.001066573
YAW_CHANGE_NORM=0.000161355

min_swarm_distance=2.033469 m
min_static_clearance=0.216478 m
min_dynamic_clearance=1.226522 m
max_velocity=2.779237 m/s
max_acceleration=2.120833 m/s^2
max_jerk=14.157684 m/s^3
optimization_time=235.418 ms
```

该成功事件的 K2-cont 与 blackout surrogate 均有下降，P/T/yaw 均发生非零变化，hard safety nonlinear evaluation 通过。

但 proposal 随后出现：

```text
UAV0: team-solution-proposal-reject
reason=NO_PENDING_TOPOLOGY_GENERATION
```

UAV1/UAV2 的 proposal ACK 为 accepted=1，随后 coordinator abort/fallback；没有形成完整三机 commit。因此第二次短测不能记为 Stage 3B short-test PASS。

第二次短测 executed telemetry（包含 fallback/terminal-hold 时段）为：

```text
executed_longest_k2_loss=0.700210 s
executed_longest_blackout=0.000000 s
FOV mismatch team=6.58%
```

command telemetry 报告：

```text
UAV1 max yaw rate=6.283185 rad/s, max yaw acceleration=634.796803 rad/s^2
UAV2 max yaw rate=3.616952 rad/s, max yaw acceleration=855.987064 rad/s^2
UAV3 max yaw rate=3.772389 rad/s, max yaw acceleration=446.968546 rad/s^2
```

这些大加速度峰值出现在 fallback/terminal-hold command transition 统计中，不能被当作 yaw-dynamics PASS。`traj_server` 对 optimized yaw payload 仍执行 rate/acc hard validation；由于本轮 proposal 没有完整 commit，最终没有得到修复后 optimized yaw 实际执行的短测证据。

修复 proposal 生命周期竞态后已重新 build 和通过所有 contract tests，但按本轮测试约束没有再启动第三次仿真。

## 进程清理

本轮启动的 roslaunch、rosmaster、rosout、planner、traj_server、simulator、scene/target processes 已清理。最终进程检查无 ROS/仿真残留：

```text
SIMULATION_LEFT_RUNNING: NO
```

## 最终真实性声明

```text
PRODUCTION_SOURCE_CHANGED: YES

BUILD_PASS: YES
UNIT_TEST_PASS: YES
GRADIENT_TEST_PASS: YES

STAGE1_PRESERVED: YES

STAGE2_IMPLEMENTED: YES
STAGE2_SHORT_TEST_PASS: NO / NOT VERIFIED

STAGE3A_IMPLEMENTED: YES
STAGE3A_SHORT_TEST_PASS: NO / NOT VERIFIED

STAGE3B_IMPLEMENTED: YES
STAGE3B_SHORT_TEST_PASS: NO / NOT VERIFIED

JOINT_PT_ACTUALLY_CHANGED_TRAJECTORY: YES

K2_CONTINUITY_OBJECTIVE_ACTIVE: YES
BLACKOUT_OBJECTIVE_ACTIVE: YES

YAW_REPREDICTION_ACTIVE: YES
JOINT_YAW_OPTIMIZATION_ACTIVE: YES
OPTIMIZED_YAW_ACTUALLY_EXECUTED: NOT VERIFIED

SAFETY_REGRESSION_OBSERVED: NO

PARTIAL_TEAM_COMMIT_OBSERVED: NO
STALE_TEAM_COMMIT_OBSERVED: NO

OFF_RUNS_THIS_AGENT: 0
AB_TESTS_THIS_AGENT: 0

STAGE2_SHORT_RUNS: 2
STAGE3A_SHORT_RUNS: 1
STAGE3B_SHORT_RUNS: 2

SIMULATION_LEFT_RUNNING: NO

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```

需要特别保留的限制是：proposal 生命周期修复后的完整三机 team commit、三机 adoption、optimized yaw actual execution 尚未在新的短测中重新验证；报告没有把它们写成 PASS。

