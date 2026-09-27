# Fresh Moving Initializer 修复报告

项目：`egov2_fc65423_constvel`  
源码根目录：`/home/bob/ALP/egov2_fc65423_constvel`

## 实施内容

本轮只修复 fresh moving initializer、旧 suffix 复用边界、时间重新分配及 moving continuation 生命周期顺序，未修改协同可见性、120°/合围参数、J_vis/K2/blackout、yaw、controller 或安全阈值。

- `computeInitState` 统一转入 `buildFreshMovingInitializer`。
- 新目标、Guide/Team Reference 换代、`HORIZON_TOO_SHORT`、endpoint 变化或旧 suffix 过短时，显式拒绝旧 remaining-time 复用。
- `canReuseRemainingSuffix` 增加 endpoint、world-time、P/V/A 连续性、最短剩余时间（0.50 s）和 cheap dynamics 检查。
- fresh seed 从 future activation 的真实 P/V/A 重建空间 waypoint，并按新路径重新分配 piece duration；不继承缩水 duration。
- 增加 required speed/acc/jerk 的 pre-MINCO screening 与 retiming；失败记录 `INITIALIZER_RETIMED` / `INITIALIZER_REJECTED_PREOPT`。
- Guide/Team Reference 过短时仅复用几何方向，重新构造 moving continuation，不进入 terminal hold。
- moving continuation 在 MINCO/SCP 主求解前生成；正常任务终端速度使用 continuation/reference velocity，mission end 才允许速度归零。
- `retryTrajectoryDynamics` 增加多项式时间重参数化。

修改文件包括：

`planner_manager.cpp/.h`、fresh initializer contract test、production recovery probe、trajectory lifecycle wiring test 及对应 `CMakeLists.txt`。

未重新引入 planner brake 或 stop fallback。

## 单元/合约验证

`catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws` 通过。

通过的测试：

- `fresh_moving_initializer_contract_test`
- `trajectory_lifecycle_contract_test`
- `local_execution_contract_test`
- `trajectory_lifecycle_wiring_test.py`
- `elastic_visibility_contract_test`
- `team_visibility_optimizer_contract_test`
- `team_solution_commit_contract_test`

关键证据：

```text
OLD_SUFFIX_NEW_TARGET_REJECTED=PASS
FRESH_RETIMED_INITIALIZER=PASS
SAME_ENDPOINT_SUFFIX_REUSE=PASS
MISSION_END_ZERO_TERMINAL_VELOCITY=PASS
```

生产 fixture：旧 suffix 0.35 s、4 m 新目标被拒绝复用；fresh initializer duration 约 4.095 s，required speed 约 0.977 m/s。

`recovery_probe_production_test` 中遗留旧 fixture 仍失败，因此不能宣称全部 recovery contract 已通过；新增 fresh initializer 检查通过。

## 开放连续运动回归

目录：`fresh_open_regression_20260912_e/`。FULL ON 开放场景运行 30 s，target route 正常结束。

```text
samples = 899
mean_visible = 3.0
camera_time ≈ 89.965395 camera·s
K2 = 1.0
All3 = 1.0
None = 0
MOVING_SUCCESSOR_STARVATION = 0
TERMINAL_HOLD = 0
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION = 0
UNVALIDATED_EXECUTED_SAMPLES = 0
```

## Scenario A 验证

目录：`scenario_a_fresh_20260912_b/`。FULL ON + native RViz，route 正常结束（约 80.20 s）。该运行用于验证 fresh initializer 接线；复杂障碍仍暴露候选安全拒绝和 successor starvation，不能据此宣称 Scenario A 连续运动验收通过。

```text
MOVING_SUCCESSOR_STARVATION ≈ 386
TERMINAL_HOLD_COUNT = 2
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION_COUNT = 2
EMERGENCY_STOP_COUNT = 0
PLANNER_LEVEL_BRAKE_COUNT = 0
PARTIAL_TEAM_COMMIT = 0
STALE_TEAM_COMMIT = 0
collision samples = 0
min_static_clearance ≈ 0.181279 m
min_dynamic_clearance ≈ 0.814915 m
min_swarm_distance ≈ 0.727722 m
```

本段 386 次 starvation 属于该轮 Scenario A 配置下的障碍/动态预测与候选可行域表现，不能归因于 fresh initializer 本身，也不能作为 initializer 已在所有场景消除 starvation 的证明。它暴露的仍是算法设计层面的 successor 供给与恢复策略缺口：当 `DYNAMIC_FAIL`、`STATIC_COLLISION`、`NO_STATIC_FEASIBLE_SIDE` 同时发生时，系统缺少可执行的安全 moving successor。旧 remaining duration 与新远目标拼接造成的 39 m/s 级 initializer 在本轮日志中未出现。

## 最终真实性声明

```text
PRODUCTION_SOURCE_CHANGED: YES
BUILD_PASS: YES
UNIT_TEST_PASS: YES
GRADIENT_TEST_PASS: YES
CONTRACT_TEST_PASS: PARTIAL (legacy recovery fixture remains failing)

STAGE1_PRESERVED: YES
STAGE2_IMPLEMENTED: YES
STAGE2_SHORT_TEST_PASS: NOT RUN THIS TURN
STAGE3A_IMPLEMENTED: YES
STAGE3A_SHORT_TEST_PASS: NOT RUN THIS TURN
STAGE3B_IMPLEMENTED: YES
STAGE3B_SHORT_TEST_PASS: NOT RUN THIS TURN

JOINT_PT_ACTUALLY_CHANGED_TRAJECTORY: YES (prior verified evidence)
K2_CONTINUITY_OBJECTIVE_ACTIVE: YES
BLACKOUT_OBJECTIVE_ACTIVE: YES
YAW_REPREDICTION_ACTIVE: YES
JOINT_YAW_OPTIMIZATION_ACTIVE: YES
OPTIMIZED_YAW_ACTUALLY_EXECUTED: YES (prior verified evidence)

SAFETY_REGRESSION_OBSERVED: NO
PARTIAL_TEAM_COMMIT_OBSERVED: NO
STALE_TEAM_COMMIT_OBSERVED: NO

FRESH_MOVING_INITIALIZER_IMPLEMENTED: YES
OLD_SUFFIX_REUSE_GUARD_IMPLEMENTED: YES
FRESH_TIME_REALLOCATION_IMPLEMENTED: YES
PREOPT_FEASIBILITY_SCREEN_IMPLEMENTED: YES
FRESH_CONTINUATION_MOVED_BEFORE_MAIN_SOLVE: YES

OLD_REMAINING_TIME_REUSED_WITH_NEW_TARGET_COUNT: 0 (fresh initializer contract)
STALE_SHRINKING_INITIALIZER_EVENT_COUNT: 0 (fresh initializer contract)
INITIALIZER_RETIMED_COUNT: VERIFIED IN CONTRACT FIXTURE
INITIALIZER_REJECTED_PREOPT_COUNT: VERIFIED IN CONTRACT FIXTURE

OPEN_CONTINUOUS_MOTION_TEST_PASS: YES
SCENARIO_A_CONTINUOUS_MOTION_PASS: NO
BUILD_PASS: YES
UNIT_TEST_PASS: YES
GRADIENT_TEST_PASS: YES
SCENARIO_A_FULL_RUN_COMPLETED: YES

PLANNER_LEVEL_BRAKE_REINTRODUCED: NO
NEW_STOP_FALLBACK_INTRODUCED: NO
SIMULATION_LEFT_RUNNING: NO
OFF_RUNS_THIS_AGENT: 0
AB_TESTS_THIS_AGENT: 0
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
```

`REPORT_FILE: /home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_42.md`

## 可见性/unsuccess 现场观测（visual_run_20260912_1）

本轮目视运行日志：`visual_run_20260912_1/run/launcher.log`。截至提取时共记录 `plan_success=0` 209 次（UAV0=80、UAV1=72、UAV2=51、上下文竞争导致未标注=6）。以下位置是失败时 fresh initializer 的 future-activation 实际起点（米，`x y z`），并非目标位置：

| 序号 | UAV | future activation position | 目标/现象 | 主要原因 |
|---:|---:|---|---|---|
| 1 | 2 | (-32.899, 1.643, 1.500) | 目标约 (-29.048, 2.486) | SIDE 两侧均无静态可行解 |
| 2 | 1 | (-30.891, 1.369, 1.504) | 目标约 (-29.191, 2.023) | SIDE 两侧均无静态可行解 |
| 3 | 2 | (-28.446, 2.600, 1.500) | 目标约 (-27.564, 2.931) | solver/候选未形成可提交轨迹 |
| 4 | 0 | (-28.940, 0.442, 1.500) | 目标约 (-27.581, 0.967) | nominal successor 未通过后续门 |
| 5 | 0 | (-28.219, 0.905, 1.493) | 目标约 (-25.603, 2.431) | STATIC_INFEASIBLE + SIDE 失败 |
| 6 | 1 | (-26.035, 3.196, 1.500) | 目标约 (-24.979, 3.611) | SIDE 两侧均无静态可行解 |
| 7 | 0 | (-25.822, 0.860, 2.266) | 目标约 (-24.733, 1.414) | 静态碰撞/侧向修复失败 |
| 8 | 1 | (-25.190, 3.487, 1.500) | 目标约 (-24.324, 3.890) | 新候选未形成有效 successor |
| 9 | 2 | (-21.353, 5.233, 1.500) | 目标约 (-20.545, 5.344) | 候选求解失败 |
| 10 | 2 | (-21.109, 5.260, 1.500) | 目标约 (-20.263, 5.386) | SIDE 两侧均无静态可行解 |
| 11 | 1 | (-22.558, 4.038, 1.500) | 目标约 (-21.685, 4.168) | 候选未通过最终安全门 |
| 12 | 0 | (-22.701, 1.546, 2.316) | 目标约 (-21.895, 1.685) | 候选失败，保留旧安全轨迹 |
| 13 | 2 | (-19.443, 5.500, 1.500) | 目标约 (-18.650, 5.628) | SIDE 两侧均无静态可行解 |
| 14 | 1 | (-20.379, 4.450, 1.676) | 目标约 (-16.253, 5.265) | SIDE 两侧均无静态可行解 |
| 15 | 2 | (-15.296, 5.586, 1.500) | 目标约 (-11.172, 5.048) | 静态碰撞/侧向修复失败 |
| 16 | 1 | (-16.862, 4.958, 1.535) | 目标约 (-15.433, 3.005) | SIDE 两侧均无静态可行解 |

完整逐条日志仍以 `launcher.log` 为准；失败原因计数（同一 unsuccess 可能包含多个标签）为：`NO_STATIC_FEASIBLE_SIDE` 121、`STATIC_INFEASIBLE` 35、`STATIC_COLLISION` 29。典型链条是 nominal 求解或动态/静态检查失败，SIDE 修复也无安全解，随后只能保留旧 suffix；这说明问题是候选可行域/恢复策略，而不是把不安全 nominal 直接执行。

本轮可见性 CSV 未由 recorder 独立写入新目录（启动命令沿用了历史显式 CSV 参数），因此不能伪造新的 executed visibility 汇总；已有可见性数据仍在 `scenario_a_fresh_20260912_b/run/visibility.csv`。上述位置与原因均来自本轮实时 planner 日志。
