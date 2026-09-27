# ALP feedback_46

本轮继续验证 shared-risk-support trigger，并修复 recovery fixture 环境；未访问或修改 `/home/bob/RRCT`。未运行 OFF、A/B 或 Scenario A。

## 测试环境修复

仅修改测试/地图 fixture，不改变 unknown-map 生产安全语义：为 recovery fixture 设置明确地图分辨率、尺寸、ground height 和 inflation，并增加 `GridMap::setFree` 测试辅助入口，显式标记 bounded synthetic workspace 为 known-free。missing-reference probe 使用非零移动目标；unreachable endpoint 只要求不被标记 occupied。

## 构建与测试

```text
FULL_BUILD_COMMAND: catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws
BUILD_PASS: YES (目标包构建成功，ego_planner 重建无 warning)
UNIT_TEST_PASS: NO
GRADIENT_TEST_PASS: YES (已有 gradient contracts)
CONTRACT_TEST_PASS: NO
FULL_TEST_SUITE_PASS: NO
```

fresh initializer contract 已通过。recovery fixture 仍退出 1，失败集中在 `actual_NOMINAL_SIDE_MINCO_capture_generates_moving_paths`、`endpoint_valid_AND_production_paths_preflight_safe`、worker admission、continuous team-follow 和 missing-reference continuation。undefined symbol 已不再出现；fixture 仍未形成完整 candidate bundle，不能宣称全套通过。

## Shared-risk-support / early visibility

生产 trigger 仍使用已有连续 risk support：`elastic_static_los_risk_max`、`elastic_dynamic_los_risk_max`、`elastic_fov_risk_max`、`directional_static_los_cost_max`、`directional_dynamic_los_cost_max`、`directional_fov_cost_max`。没有新增 visibility 数字阈值、状态机、brake、stop fallback、权重或安全距离。

因 recovery production fixture 尚未全绿，本轮没有合法完成 target-side、mirror、A* rescue 或 rough trajectory fixture，也没有采集 risk-support 到 binary loss 的时间序列：

```text
TARGET_SIDE_VISIBILITY_FIXTURE_PASS: NOT RUN
MIRRORED_SIDE_FIXTURE_PASS: NOT RUN
TOPOLOGY_ASTAR_RESCUE_FIXTURE_PASS: NOT RUN
ROUGH_TRAJECTORY_FIXTURE_PASS: NOT RUN
TRIGGER_LEAD_TIME_P50/P95/MIN: NOT RUN
EARLY_VISIBILITY_TOPOLOGY_PROBLEM_FIXED: NO
```

## Candidate / revision

本轮没有足够的 production candidate trace 验证 revision 后安全、visibility、Q_dir 全部重算，也没有通过完整 trajectory 的统一执行资格验收：

```text
LEFT_RIGHT_ARE_OBSERVATION_TOPOLOGIES: NOT VERIFIED
SIMPLE_SIDE_SEED_FAILURE_KILLS_TOPOLOGY: NOT VERIFIED
UNIFIED_FINAL_PREFLIGHT_PASS: NOT VERIFIED
METADATA_RECOMPUTE_AFTER_REVISION_PASS: NOT VERIFIED
VISIBILITY_SHARED_DEFINITION_VERIFIED: NOT VERIFIED
```

## Scenario A / runtime

因 full test suite 未通过，未启动 `long_cylinder_forest_visibility_stress.json`。可见性、绕行、连续运动、协同、平滑度和安全运行指标均为 `NOT RUN`，没有从历史数据倒灌。

```text
ACCUMULATED_CAMERA_VISIBLE_TIME: NOT RUN
MEAN_VISIBLE: NOT RUN
K2: NOT RUN
ALL3: NOT RUN
NONE: NOT RUN
MOVING_SUCCESSOR_STARVATION_COUNT: NOT RUN
TERMINAL_HOLD_COUNT: NOT RUN
LEFT_ACTUAL_ACTIVATION_COUNT: NOT RUN
RIGHT_ACTUAL_ACTIVATION_COUNT: NOT RUN
ACTUAL_Q_DIR_MEAN: NOT RUN
EXECUTED_ENCIRCLEMENT_RATIO: NOT RUN
COLLISION_SAMPLES: NOT RUN
SWARM_CLEARANCE_VIOLATION_SAMPLES: NOT RUN
UNVALIDATED_EXECUTED_SAMPLES: NOT RUN
```

测试结束后已停止 roscore；没有 planner/RViz/Gazebo 残留。

```text
PLANNER_LEVEL_BRAKE_REINTRODUCED: NO
NEW_STOP_FALLBACK_INTRODUCED: NO
SCENARIO_A_FULL_RUN_COMPLETED: NO
SCENARIO_A_ACCEPTANCE_PASS: NO
SIMULATION_LEFT_RUNNING: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
REPORT_FILE: /home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_46.md
```

下一步必须继续定位 recovery fixture 为什么在 known-free 配置后仍不能形成 production candidate；在该 contract 全绿并取得提前绕行时间证据前，不运行完整 Scenario A。
