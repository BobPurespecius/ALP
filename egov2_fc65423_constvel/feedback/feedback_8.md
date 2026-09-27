# Feedback 8

## 1. 当前代码入口与范围

- 运行源码树由 `ros_ws/build/{ego_planner,path_searching,plan_env}/CMakeCache.txt` 确认为：
  `ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner`。
- 最终候选生成、三类安全分类与选择入口：
  `plan_manage/src/planner_manager.cpp:1460`，`EGOPlannerManager::reboundReplan()`。
- 最终候选分类：`planner_manager.cpp:1711`、`4261-4262`。
- 阶段一候选可见性评估：`planner_manager.cpp:912-1077`。
- 阶段一 visibility report/selection：`planner_manager.cpp:4264-4318`、`4385-4518`。
- SIDE repair A*：`path_searching/src/dyn_a_star.cpp`，由
  `planner_manager.cpp:2550-2630` 设置阶段二软代价上下文。
- 共享 static LOS 几何：
  `plan_env/include/plan_env/grid_map.h:375-407`。
- 本轮没有修改三类轨迹定义、动态安全阈值、SIDE offset/bilateral、semantic-v2、
  Local-SFC、MINCO/SCP、trust/model-agreement、OSQP、max_jer、stale-head、控制器或场景几何。

工作区在开始和结束时均保持 dirty；未执行 reset、checkout、clean 或 commit。

## 2. 阶段一实现

### 2.1 Candidate Visibility Report

评估位置严格位于最终轨迹和最终 risk 已产生之后、候选选择之前。评估对象为：

- `NOMINAL`
- `SIDE_PLUS`
- `SIDE_MINUS`
- 当前 remaining previous-active trajectory

三机状态来源：

- 当前规划 UAV 使用该 candidate 的最终 polynomial；
- 另外两机优先使用同一 absolute time 下的 active swarm trajectory；
- active trajectory 不可用时，使用不超过 1 s 的最新 odom 做常速度外推；
- 任一必要数据无效时 `visibility_valid=false`，保持原 selection，不拒绝候选。

统一时间口径：

- `candidate_start_time = planning_prediction_epoch`
- `visibility_horizon = min(candidate duration, reliable moving prediction horizon)`
- `visibility_sample_dt = 0.1 s`
- target、dynamic obstacle 与三机均使用同一 candidate-relative/absolute time。

未来 yaw 无法可靠复现 traj_server 的 yaw-rate/yaw-acc execution，因此 FOV 只标记为
`DIAGNOSTIC_UNAVAILABLE`，排序使用 range + authoritative static/dynamic LOS；没有另造简化 yaw 模型。

### 2.2 选择合同

实现顺序保持：

`ABSOLUTE_SAFE > previous ABSOLUTE_SAFE > IMPROVED_ONLY > INVALID fallback`。

Visibility 只比较与当前 selected 相同 safety class 且 risk valid 的候选：

- ABSOLUTE_SAFE clearance band：0.15 m
- IMPROVED_ONLY clearance band：0.05 m
- INVALID：不参与 visibility ranking

band 内按以下字典序：

1. none-visible 更小；
2. at-least-2 更大；
3. 最差单机 visibility 更大；
4. all-3 更大；
5. 最大连续失视时间更小；
6. 仍相同时回到 dynamic clearance。

只有主要 visibility 指标改善至少 0.03 才允许改变 topology。离线合同测试确认：

- 不同 safety class 不互相提升；
- band 外不能被 visibility 覆盖；
- INVALID 不能被提升；
- 小于 3 个百分点的差异不触发切换。

新增日志：`[candidate-visibility]`、`[visibility-selection]`。

## 3. 严格 target/dynamic replay

- Frozen scene：
  `ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_visibility_stress.json`
- Scene SHA256：
  `430fce52ee31c4a4ea13607e39346e0f04f1af1e3da31eb426a44a61786c4cb0`
- Frozen replay manifest SHA256（Stage1/Stage2 相同）：
  `a91ba5de35c86035f42d2bf8529b0edda2a9d1ed2e65bc393bf08eae05708228`
- 每段 target speed 固定为 scene speed，关闭 target coordinator 的运行时动态调速。
- dynamic obstacle motion epoch 由 `/target_tracking/target_start_time` 同步锁定，满足：
  `dynamic_motion_elapsed == target_elapsed`。
- 六轮 Stage1 与三轮 Stage2 的 target 样本均通过 frozen waypoint/time-law 校验；
  position cross-track 和 velocity contract 最大误差均小于 `1e-6` 量级。

测试 wrapper：`visibility_planning_validation_20260902/run_stage1.py`。

## 4. 阶段一 A/B 结果

数据目录：`visibility_planning_validation_20260902/stage1_ab/`。

表中为 3 轮均值 `[min,max]`；visibility/loss 为全程比例。

| Metric | Ranking OFF | Ranking ON |
|---|---:|---:|
| Task complete | 3/3 | 3/3 |
| Dynamic collision episodes | 0 [0,0] | 0 [0,0] |
| Dynamic collision samples | 0 [0,0] | 0 [0,0] |
| Unsafe samples `<0.5 m` | 60.7 [43,79] | 60.0 [20,98] |
| Static collision episodes | 0 [0,0] | 0 [0,0] |
| Tracking mean (m) | 1.582 [1.559,1.609] | 1.574 [1.532,1.608] |
| Tracking P95 (m) | 1.906 [1.817,2.049] | 1.915 [1.782,2.071] |
| Mean path length (m) | 79.932 [79.784,80.084] | 79.967 [79.822,80.170] |
| Planning latency median (ms) | 0.386 [0.384,0.387] | 0.385 [0.372,0.393] |
| Planning latency P95 (ms) | 0.839 [0.827,0.848] | 0.794 [0.776,0.812] |
| Planning latency max (ms) | 13.660 [3.296,19.135] | 76.751 [10.046,200.649] |
| UAV1 visibility | 97.58% [97.26,98.05] | 98.81% [97.63,100.00] |
| UAV2 visibility | 100.00% | 99.71% [99.13,100.00] |
| UAV3 visibility | 97.23% [96.47,97.63] | 96.81% [95.93,97.30] |
| All-3 visibility | 95.50% [94.72,96.51] | 95.90% [95.60,96.34] |
| At-least-2 visibility | 99.31% [99.00,99.75] | 99.43% [99.13,100.00] |
| At-least-1 visibility | 100.00% | 100.00% |
| None-visible | 0.00% | 0.00% |
| Static LOS loss | 1.01% [0.80,1.36] | 0.95% [0.76,1.23] |
| Dynamic LOS loss | 0.65% [0.35,0.86] | 0.52% [0.46,0.65] |
| FOV loss | 0.05% [0.00,0.14] | 0.04% [0.00,0.11] |
| Out-of-range | 0.06% [0.00,0.18] | 0.04% [0.00,0.11] |
| Executed jerk P95 (m/s³) | 46.043 [43.466,48.962] | 44.640 [40.585,48.423] |

阶段一变化：

- All-3：`+0.401 pp`
- Worst-UAV：`-0.360 pp`
- At-least-2：`+0.125 pp`
- None-visible：`+0.000 pp`
- Tracking P95：`+0.46%`
- Path length：`+0.04%`
- Latency P95：`-5.31%`
- visibility selection checks：294
- visibility 实际改变最终 kind：3 次，全部为同 safety class、全部在 clearance band 内
- INVALID promotions：0
- SIDE sequence oscillations：OFF 9，ON 9，无增加

结论：安全稳定，但 all-3/worst-UAV/at-least-2 均远未达到阶段一门槛，
`STAGE1_RESULT = INSUFFICIENT`，因此进入阶段二。

## 5. Dynamic-gates 安全回归

- Scene：`long_cylinder_forest_dynamic_gates_v2_c1_relaxed_v2.json`
- 数据：`visibility_planning_validation_20260902/c1_safety_regression/c1_on/`
- Task complete：YES
- 全局 dynamic/static collision samples：0 / 0
- C1 collision/unsafe samples：0 / 0
- C1 精确 obstacle-specific 最小 surface clearance：0.702 m
- 全局最小 static clearance：0.212 m

阶段一没有破坏 C1 relaxed-v2 的安全性。

## 6. 阶段二实现

阶段二只在 Stage1 `INSUFFICIENT` 后实现。

- 参数：`enable_astar_visibility_cost`，默认 false。
- 权重：`astar_visibility_cost_weight_m = 0.75 m`。
- 仅作用于 SIDE static repair A*；其它 A* 实例默认关闭。
- static occupancy 仍是 hard feasibility；visibility 只增加非负 soft cost。
- LOS clear：penalty 0。
- LOS blocked：使用 shared inflated-grid LOS ray 的 occupied fraction。
- LOS query invalid：penalty 0，绝不 hard reject。
- node time 使用明确的近似映射：
  `t_node = accumulated_geometric_arclength / (0.8 * max_vel)`。
- edge penalty 经过 direct repair length 归一化；整条 fully occluded path 的物理等价附加代价约为 0.75 m，
  不随 A* 点数线性爆炸。
- 没有 dynamic LOS hard constraint、MINCO cost、SCP row 或多机角度优化。

源码证据：

- shared LOS：`grid_map.h:375-407`
- A* context/penalty：`dyn_a_star.cpp:36-111`
- A* `gScore` soft addition：`dyn_a_star.cpp:328-350`
- manager target/time context：`planner_manager.cpp:2550-2554`
- `[astar-visibility-cost]` 日志：`planner_manager.cpp:2605-2625`

## 7. 阶段二结果

Smoke：PASS，无 collision、no-path cascade 或残留进程。

三轮数据：`visibility_planning_validation_20260902/stage2_runs/`。

| Metric | Stage1 only ON | Stage1 + Stage2 |
|---|---:|---:|
| Task complete | 3/3 | 3/3 |
| Dynamic collision episodes | 0 | 0 |
| Static collision episodes | 0 | 0 |
| Unsafe samples | 60.0 [20,98] | 22.0 [19,28] |
| Tracking P95 (m) | 1.915 [1.782,2.071] | 1.832 [1.798,1.865] |
| Mean path length (m) | 79.967 [79.822,80.170] | 80.060 [79.126,81.448] |
| Planning latency P95 (ms) | 0.794 [0.776,0.812] | 0.853 [0.785,0.967] |
| UAV1 visibility | 98.81% | 98.28% |
| UAV2 visibility | 99.71% | 99.61% |
| UAV3 visibility | 96.81% | 97.52% |
| All-3 visibility | 95.90% | 96.33% |
| At-least-2 visibility | 99.43% | 99.09% |
| None-visible | 0.00% | 0.00% |
| Static LOS loss | 0.95% | 1.15% |
| Dynamic LOS loss | 0.52% | 0.38% |
| Executed jerk P95 (m/s³) | 44.640 | 40.846 |

相对 Stage1：

- All-3：`+0.427 pp`
- Worst-UAV：`+0.705 pp`
- At-least-2：`-0.347 pp`
- Static LOS loss：`+0.195 pp`（变差）
- Tracking P95：`-4.31%`
- Path length：`+0.12%`
- Latency P95：`+7.32%`
- visibility switches：3 → 6；SIDE oscillations：9 → 13

逐 Gate：

| Gate | Metric | Stage1 | Stage1+2 | Delta |
|---|---|---:|---:|---:|
| V1 | All-3 | 70.67% | 75.17% | +4.50 pp |
| V1 | At-least-2 | 90.89% | 85.37% | -5.52 pp |
| V1 | Static LOS blocked | 4.44% | 7.02% | +2.58 pp |
| V2 | All-3 | 73.33% | 78.76% | +5.43 pp |
| V2 | Static LOS blocked | 8.89% | 7.08% | -1.81 pp |
| V3 | All-3 | 96.00% | 100.00% | +4.00 pp |

A* 诊断：

- soft-cost A* events/success：51 / 51
- invalid LOS queries：0
- successful path mean LOS-blocked node ratio：11.45%
- mean/max physical-equivalent penalty：0.031 / 0.272 m
- repair path mean length：Stage1 2.428 m，Stage2 2.273 m
- repair search mean/P95：Stage1 0.383/0.261 ms，Stage2 0.501/1.441 ms
- A* pool logs：Stage1 12，Stage2 15；timeout logs：Stage1 1，Stage2 2
- SCP_FINAL_OK candidate count：155

阶段二没有安全碰撞回归，但全局 all-3/worst-UAV 改善远低于 3/5 pp 门槛，
at-least-2 与 static LOS 反而变差，且 A* 搜索开销/oscillation 略增。
因此 `STAGE2_RESULT = INEFFECTIVE_OR_REGRESSIVE`。

## 8. 最终默认开关与验证

阶段一只有微小且不一致的收益；阶段二也没有可靠增益。因此：

- `enable_visibility_candidate_ranking`：保留可配置，主 launch 默认 OFF。
- `enable_astar_visibility_cost`：保留可配置，主 launch 默认 OFF。
- 不进入 MINCO/SCP visibility optimization。

验证：

- `stage1_contract_test.py`：PASS
- `stage2_contract_test.py`：PASS
- Python compile：PASS
- launch XML parse：PASS
- `catkin build path_searching ego_planner --no-status -j2`：PASS
- Stage1 OFF/ON：3+3 完整运行全部完成
- Stage2：3 次完整运行全部完成
- C1 safety regression：PASS
- 最终残留 ALP/ROS 进程：0

## 9. Final fields

VISIBILITY_CANDIDATE_REPORT_IMPLEMENTED:
YES

THREE_UAV_VISIBILITY_EVALUATED:
YES

VISIBILITY_ONLY_WITHIN_SAME_SAFETY_CLASS:
YES

VISIBILITY_CAN_OVERRIDE_CLEARANCE_OUTSIDE_BAND:
NO

VISIBILITY_CAN_PROMOTE_UNSAFE_CANDIDATE:
NO

STRICT_TARGET_TRAJECTORY_REPLAY:
YES

STAGE1_RUNS:
OFF=3 / ON=3

STAGE1_RESULT:
INSUFFICIENT

STAGE1_ALL3_VISIBILITY_CHANGE:
+0.401 percentage points

STAGE1_WORST_UAV_VISIBILITY_CHANGE:
-0.360 percentage points

STAGE1_COLLISION_REGRESSION:
NO

STAGE2_IMPLEMENTED:
YES

ASTAR_VISIBILITY_COST_IS_SOFT:
YES

STAGE2_RUNS:
Stage1+Stage2=3

STAGE2_RESULT:
INEFFECTIVE_OR_REGRESSIVE

FINAL_VISIBILITY_RANKING_DEFAULT:
OFF

FINAL_ASTAR_VISIBILITY_COST_DEFAULT:
OFF

MINCO_SCP_VISIBILITY_COST_IMPLEMENTED:
NO

NEW_HARD_GATE_ADDED:
NO

BUILD:
PASS

RUNTIME:
PASS

PREVIOUS_MAX_FEEDBACK_INDEX:
7

CURRENT_FEEDBACK_FILE:
feedback_8.md
