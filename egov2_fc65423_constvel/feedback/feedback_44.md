# Visibility-trigger verification and risk-support update — 2026-09-12

范围：仅 `/home/bob/ALP/egov2_fc65423_constvel`。未访问或修改 RRCT。先保存并验证 feedback_43 版本；随后将经验性的 visibility trigger 改为复用现有连续 risk support。未运行 Scenario A、OFF、A/B 或其它仿真。

## Baseline / source state

`HEAD`: `8cef24143c8a8a23bae77dd0e6cf5395c7dc6d2c`  
修改文件：`planner_manager.cpp`。本次 diff SHA256：`eb7a09a8fd81227b0ebff4b8d30e1b3b74cfd1da1f9092d914f17bc516880ec3`。  
构建 binary (`libtraj_opt.so`) SHA256：`97ded91a93a6ba685efbc192ec31e0d27786f6c9663f110c4808064d7ff37a21`。

完整目标包构建命令：

```text
catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws
```

结果：`BUILD_PASS=YES`（6/6 packages）。

## 阶段 1 验证

已检查 feedback_43 声明的代码路径：final candidate revalidation、visibility clearance-window removal、统一 lexicographic visibility selection、NOMINAL/LEFT/RIGHT dispatch。现有 contract binaries 中，多数通过：

- time-only feasibility：PASS
- time-only swarm temporal：PASS
- elastic visibility：PASS
- trajectory lifecycle：PASS
- local execution：PASS
- adaptive viewpoint：PASS
- encirclement geometry：PASS
- topology coordinator：PASS
- cooperative viewpoint：PASS
- multiview：PASS
- persistent recovery target：PASS
- static LOS wall：PASS
- team visibility optimizer：PASS
- team solution commit：PASS

发现两个既有测试/环境问题，未将其伪装为通过：

1. `fresh_moving_initializer_contract_test` 当前二进制直接退出 `139`（段错误），无测试输出；需要单独修 fixture/初始化生命周期。
2. `recovery_probe_production_test` 在当前链接环境报 `undefined symbol: fast_planner::ObjPredictor::evaluateConstVel(int,double)`，属于 fixture/link 环境问题，不是本次 trigger 修改引入的编译错误。

因此：`UNIT_TEST_PASS=NO (fresh fixture crash)`，`CONTRACT_TEST_PASS=NO (legacy recovery fixture unresolved symbol)`；不能声称 full suite pass。

## Trigger 审计与修改

feedback_43 中的：

```text
All3 < 0.80 OR K2 < 0.90 OR blackout > 0.05
```

在源码、launch 参数、注释和历史 feedback 中未找到理论边界或可复现统计校准；它们是 feedback_43 为提前生成 SIDE 而加入的经验常量。结论：

```text
ALL3_TRIGGER_PROVENANCE: feedback_43 empirical heuristic
K2_TRIGGER_PROVENANCE: feedback_43 empirical heuristic
BLACKOUT_TRIGGER_PROVENANCE: feedback_43 empirical heuristic
HEURISTIC_TRIGGER_THRESHOLD: YES
```

已删除这组三个聚合比例门，未替换成新的百分比阈值。现在对 nominal optimizer 的已有 `OptimizationDiagnostics::final` 检查：

```text
elastic_static_los_risk_max
elastic_dynamic_los_risk_max
elastic_fov_risk_max
directional_static_los_cost_max
directional_dynamic_los_cost_max
directional_fov_cost_max
```

任一有限且大于数值零（`1e-9`）即 `visibility_support_active`，并把现有 risk dispatch 置为 triggered，从而复用相同 visibility geometry/risk support 进入 NOMINAL/LEFT/RIGHT。没有新增任务状态机或新的经验 threshold；最终 binary visibility 仍由 `evaluateCandidateVisibility` 重算。

这形成：

```text
existing LOS/dynamic/FOV risk support
 -> topology exploration opportunity
 -> existing continuous J_vis/FOV/yaw optimization
 -> final binary range && static LOS && dynamic LOS && FOV evaluation
```

## 代码状态声明

```text
UNIFIED_CANDIDATE_EVALUATION_IMPLEMENTED: YES (feedback_43 path verified, full suite not green)
CONSTRUCTION_STATUS_SEPARATED_FROM_EXECUTABILITY: YES
EXECUTABILITY_SEPARATED_FROM_TASK_QUALITY: PARTIAL
EARLY_ABSOLUTE_SAFE_HAS_EXECUTION_AUTHORITY: NO
SCP_FINAL_OK_HAS_EXECUTION_AUTHORITY: NO
IMPROVED_ONLY_HAS_EXECUTION_AUTHORITY: NO
FALLBACK_SOURCE_HAS_SPECIAL_EXECUTION_AUTHORITY: NO
VISIBILITY_USES_SHARED_DEFINITION: YES
VISIBILITY_RECOMPUTED_AFTER_TRAJECTORY_REVISION: YES
CLEARANCE_WINDOW_BLOCKS_VISIBILITY_COMPETITION: NO
SEPARATE_VISIBILITY_0P03_GATE_REMAINS: NO
LEFT_RIGHT_ARE_OBSERVATION_TOPOLOGIES: YES (dispatch semantics; dedicated fixtures not run)
SIMPLE_SIDE_SEED_FAILURE_KILLS_TOPOLOGY: NOT VERIFIED
VISIBILITY_CAN_TRIGGER_LEFT_RIGHT_GENERATION: YES
VISIBILITY_CONTINUOUS_OPTIMIZATION_ACTIVE: YES
VISIBILITY_FINAL_BINARY_RECHECK_ACTIVE: YES

OLD_VISIBILITY_THRESHOLD_TRIGGER_RETAINED: NO
OLD_TRIGGER_THEORETICAL_BASIS: NO
OLD_TRIGGER_EMPIRICAL_BASIS: NO (no documented calibration found)
VISIBILITY_TRIGGER_METHOD: SHARED_RISK_SUPPORT
VISIBILITY_TRIGGER_USES_EXISTING_RISK_BOUNDS: YES
NEW_VISIBILITY_TRIGGER_THRESHOLD_ADDED: NO

TARGET_SIDE_VISIBILITY_FIXTURE_PASS: NOT RUN
MIRRORED_SIDE_FIXTURE_PASS: NOT RUN
TOPOLOGY_ASTAR_RESCUE_FIXTURE_PASS: NOT RUN
ROUGH_TRAJECTORY_FIXTURE_PASS: NOT RUN
FULL_TEST_SUITE_PASS: NO
SCENARIO_A_FULL_RUN_COMPLETED: NO
```

## Runtime / metrics

本轮没有仿真或 Scenario A，因此没有合法的 runtime funnel、trigger lead time、LEFT/RIGHT activation、camera-time、K2/All3、safety 或 starvation 数值。应记录为 `NOT RUN`，不能从历史运行倒灌。

```text
VISIBILITY_TRIGGERED_TOPOLOGY_COUNT: NOT RUN
TRIGGER_LEAD_TIME_P50/P95/MIN: NOT RUN
LEFT_ACTUAL_ACTIVATION_COUNT: NOT RUN
RIGHT_ACTUAL_ACTIVATION_COUNT: NOT RUN
NOMINAL_ACTUAL_ACTIVATION_COUNT: NOT RUN
ROUGH_SAFE_CANDIDATE_USED_COUNT: NOT RUN
MOVING_SUCCESSOR_STARVATION_COUNT: NOT RUN
TERMINAL_HOLD_COUNT: NOT RUN
ACCUMULATED_CAMERA_VISIBLE_TIME: NOT RUN
MEAN_VISIBLE: NOT RUN
K2: NOT RUN
ALL3: NOT RUN
NONE: NOT RUN
EXECUTED_ENCIRCLEMENT_RATIO: NOT RUN
ACTUAL_Q_DIR_MEAN: NOT RUN
HIGH_QUALITY_MULTI3_RATIO: NOT RUN
COLLISION_SAMPLES: NOT RUN
SWARM_CLEARANCE_VIOLATION_SAMPLES: NOT RUN
UNVALIDATED_EXECUTED_SAMPLES: NOT RUN
```

未恢复 planner brake，未新增 stop fallback；测试结束后已停止 roscore，当前无 ROS/planner/RViz/Gazebo 仿真进程。

```text
PLANNER_LEVEL_BRAKE_REINTRODUCED: NO
NEW_STOP_FALLBACK_INTRODUCED: NO
BUILD_PASS: YES
UNIT_TEST_PASS: NO
GRADIENT_TEST_PASS: YES (elastic/team visibility gradient contracts passed)
CONTRACT_TEST_PASS: NO (fresh initializer crash + recovery fixture unresolved symbol)
SIMULATION_LEFT_RUNNING: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
REPORT_FILE: /home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_44.md
```
