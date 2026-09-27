# ALP feedback_45

本轮按要求先验证 feedback_44 版本，未运行 OFF、A/B 或 Scenario A；未访问 `/home/bob/RRCT`。

## Baseline

```text
SOURCE_HEAD: 8cef24143c8a8a23bae77dd0e6cf5395c7dc6d2c
PLANNER_MANAGER_SHA256: f361a31a0c7873080416ebb2273f2fbc7c914c648d27a324092ccd8125b3b3fd
EGO_PLANNER_MANAGER_SO_SHA256: 80932fdc8557293b3979721dc689403cf49e3558e5bddc60b70f14b5a963c687
PLAN_ENV_SO_SHA256: d58abf955d56e1469ba53f9c8909a786b1285c019bdb4cc37b4cbaa97d519a7c
BUILD_COMMAND: catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws
```

## Tests

完整构建通过，6/6 packages succeeded，无 warning。

`fresh_moving_initializer_contract_test` 在 roscore 正常启动后通过，确认旧短 suffix + 新远目标被拒绝，fresh seed 重新分配时间，正常 moving 终端速度非零。

`recovery_probe_production_test` 的 undefined symbol 在完整工作区重建后消失，但测试仍退出 1。剩余失败不是动态链接错误，而是 fixture 依赖未初始化的合成 GridMap：日志中多处为 `static map t=0 occupancy=-1`，即 unknown occupancy，被当前安全 preflight 正确拒绝。另有连续 team-follow / missing-reference fixture 因同一生产候选未生成而失败。测试文件中的两个明显 fixture 语义问题已修正：missing-reference probe 不再把当前位置作为移动目标；未知端点只断言“不被标记 occupied”，不错误要求它是 known-free。

```text
BUILD_PASS: YES
UNIT_TEST_PASS: NO
GRADIENT_TEST_PASS: YES (已有 elastic/team visibility gradient contracts)
CONTRACT_TEST_PASS: NO
FULL_TEST_SUITE_PASS: NO
```

## Early visibility / topology verification

由于 contract suite 尚未全绿，本轮按门槛停止，没有运行四个受控 production fixture，也没有运行 Scenario A。以下项目均没有合法实测证据：

```text
TARGET_SIDE_VISIBILITY_FIXTURE_PASS: NOT RUN
MIRRORED_SIDE_FIXTURE_PASS: NOT RUN
TOPOLOGY_ASTAR_RESCUE_FIXTURE_PASS: NOT RUN
ROUGH_TRAJECTORY_FIXTURE_PASS: NOT RUN
EARLY_VISIBILITY_TOPOLOGY_PROBLEM_FIXED: NO
```

源码审计仍确认当前 trigger 已使用已有 visibility risk support 字段，而不是旧 `All3 < 0.80 OR K2 < 0.90 OR blackout > 0.05` 数字门槛；但本轮没有 production fixture 的 lead-time 或 false-positive/false-negative 测量。

```text
VISIBILITY_TRIGGER_METHOD: SHARED_RISK_SUPPORT
NEW_VISIBILITY_TRIGGER_THRESHOLD_ADDED: NO
TRIGGER_LEAD_TIME_P50/P95/MIN: NOT RUN
VISIBILITY_TRIGGERED_TOPOLOGY_COUNT: NOT RUN
LEFT_ACTUAL_ACTIVATION_COUNT: NOT RUN
RIGHT_ACTUAL_ACTIVATION_COUNT: NOT RUN
NOMINAL_ACTUAL_ACTIVATION_COUNT: NOT RUN
```

## Candidate semantics

feedback_43/44 的源码路径保留统一 final hard-check 方向；本轮未能用完整 production fixture 验证 revision metadata、shared binary visibility、A* rescue 或 rough candidate 选择，因此不把这些声明升级为实测 PASS。

```text
UNIFIED_FINAL_PREFLIGHT_PASS: NOT VERIFIED
METADATA_RECOMPUTE_AFTER_REVISION_PASS: NOT VERIFIED
VISIBILITY_SHARED_DEFINITION_VERIFIED: NOT VERIFIED
SIMPLE_SIDE_SEED_FAILURE_KILLS_TOPOLOGY: NOT VERIFIED
```

## Runtime metrics

没有仿真或 Scenario A，以下均为 `NOT RUN`，没有从历史运行倒灌：

```text
ACCUMULATED_CAMERA_VISIBLE_TIME: NOT RUN
MEAN_VISIBLE: NOT RUN
K2: NOT RUN
ALL3: NOT RUN
NONE: NOT RUN
LONGEST_K2_LOSS: NOT RUN
LONGEST_BLACKOUT: NOT RUN
ACTUAL_Q_DIR_MEAN: NOT RUN
HIGH_QUALITY_MULTI3_RATIO: NOT RUN
EXECUTED_ENCIRCLEMENT_RATIO: NOT RUN
ROUGH_SAFE_CANDIDATE_USED_COUNT: NOT RUN
MOVING_SUCCESSOR_STARVATION_COUNT: NOT RUN
TERMINAL_HOLD_COUNT: NOT RUN
TRAJECTORY_END_BEFORE_NEXT_ACTIVATION_COUNT: NOT RUN
COLLISION_SAMPLES: NOT RUN
SWARM_CLEARANCE_VIOLATION_SAMPLES: NOT RUN
UNVALIDATED_EXECUTED_SAMPLES: NOT RUN
```

`roscore` 已停止；没有 planner/RViz/Gazebo 残留。

```text
PLANNER_LEVEL_BRAKE_REINTRODUCED: NO
NEW_STOP_FALLBACK_INTRODUCED: NO
SCENARIO_A_FULL_RUN_COMPLETED: NO
SCENARIO_A_ACCEPTANCE_PASS: NO
SIMULATION_LEFT_RUNNING: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
REPORT_FILE: /home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_45.md
```

下一步应先给 recovery fixture 提供明确的 known-free map/fixture 初始化，并重新跑完整 unit/gradient/contract/production suite；在此之前不得进入 Scenario A。
