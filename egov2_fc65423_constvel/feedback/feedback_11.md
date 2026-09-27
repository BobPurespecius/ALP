# Feedback 11

日期：2026-09-03

工程：`/home/bob/ALP/egov2_fc65423_constvel`

## 实现范围与职责边界

本轮在 Feedback 10 的同一个 `Cooperative Viewpoint Manager` 中完成改造，没有新建第二套 manager，也没有改动 planner 的安全规划链。

当前数据流为：

`/object_odom` 真实目标 → 集中式独立视点 manager → 3 个 reference topic → 3 套独立 EGO planner → SIDE/A*/SFC/MINCO/SCP/OSQP → trajectory execution。

修改/新增的本轮相关文件：

- `ros_ws/src/multi_uav_formation/include/multi_uav_formation/cooperative_viewpoint_core.h`
- `ros_ws/src/multi_uav_formation/src/cooperative_viewpoint_manager.cpp`
- `ros_ws/src/multi_uav_formation/test/cooperative_viewpoint_contract_test.cpp`
- `ros_ws/src/multi_uav_formation/launch/native_egov2_rviz.launch`
- `independent_viewpoint_validation_20260903/independent_viewpoint_contract_test.py`
- `independent_viewpoint_validation_20260903/run_independent_viewpoint.py`
- `independent_viewpoint_validation_20260903/analyze_independent_viewpoint.py`

未修改 SIDE、semantic-v2、A*、Local-SFC、repair timing、MINCO/SCP、trust、OSQP、三类轨迹、previous-safe、stale-head、controller、yaw、dynamic safety 或场景。

源码证据：

- `cooperative_viewpoint_core.h:18-22`：状态只有 `HOLD` 和 `TRANSITION`。
- `cooperative_viewpoint_core.h:142-159`：三个独立角分别旋转各自原始 offset。
- `cooperative_viewpoint_manager.cpp:75-90`：真实目标订阅与三个 odom 输入、三个 reference 输出彼此分离。
- `ego_replan_fsm.cpp:1105-1118`：viewpoint 只替换 tracking goal/reference velocity，真实 `object_p_/object_v_/object_q_` 不变。
- `ego_replan_fsm.cpp:1198-1224`：reference 独立接收、校验和超时 fallback。
- `native_egov2_rviz.launch:24,32,34`：Stage1/Stage2/Stage3 默认均为 OFF。
- `native_egov2_rviz.launch:37`：独立视点 feature 生产默认为 OFF。

说明：core 中保留一个未被调用的 common-angle convenience overload，但运行状态、选择流程和发布流程均只使用 `std::array<double,3>`；不存在旧刚性旋转 runtime control flow。

## 单机独立视点逻辑

每架 UAV 保存独立的 `commanded_angles_[i]`、`target_angles_[i]` 和 `last_valid_angles_[i]`。

参考位置和速度为：

`q_i = p_T + Rz(phi_i) r_i^0`

`v_ref_i = v_T + phi_dot_i ez × Rz(phi_i) r_i^0`

证据：

- `cooperative_viewpoint_core.h:80-82`：原始三个 formation offsets 不变。
- `cooperative_viewpoint_core.h:142-159`：独立参考位置。
- `cooperative_viewpoint_core.h:579-583`：独立参考速度。
- `cooperative_viewpoint_core.h:635-642`：三个独立 angle 状态与唯一 active UAV。

候选只对待调整 UAV 枚举 `m*pi/4, m=0..7`，另外两架保持当前 angle。排序依次为：至少两架可见率、待调整 UAV 可见率、all-3、LOS clearance、移动距离。候选必须比当前视点提高至少 0.10。

证据：`cooperative_viewpoint_core.h:442-497`。

本轮初始 smoke 暴露并修复了一个实现缺陷：非 active UAV 的 reference point 与静态柱重合时，会错误地令 active UAV 的所有候选无效。最终代码只用待调整 UAV 的 point-static feasibility 判定该候选，同时仍用另外两架计算联合 visibility/separation。最终合同测试项 `candidate_static_scope=1` 通过；早期预修复数据没有进入正式结果。

## HOLD / TRANSITION 合同

### HOLD

- 触发使用实际 UAV odom 到真实目标的 solid-cylinder static LOS。
- static LOS 连续 blocked 至少 0.5 s 才参与选择。
- tracking error 大于 0.5 m 时不切换，记录 `PLANNER_NOT_SETTLED`。
- 多架同时 blocked 时优先 blocked duration 最长者，再比较当前预测可见率。

证据：`cooperative_viewpoint_core.h:384-440`。

### TRANSITION

- `active_uav_` 只允许一个索引。
- target angle 在 transition 开始时冻结。
- transition 中不重新搜索、不允许另一架开始、不允许反向。
- 只有 angle 到位、实际 odom 到 reference 距离不超过 0.5 m，并连续保持 0.5 s 才完成。
- active reference/static geometry 失效时回退到最近 valid angle；没有 FAILED、E-stop 或 hard shutdown。

证据：

- `cooperative_viewpoint_core.h:480-488`：冻结 transition target/direction。
- `cooperative_viewpoint_core.h:508-512`：transition 内禁止 reselection。
- `cooperative_viewpoint_core.h:514-531`：0.25 rad/s rate limit 与反转检查。
- `cooperative_viewpoint_core.h:533-568`：invalid fallback 和实际 odom 到达判定。

## 最小观察角分离

观察角使用各 UAV 旋转后 offset 的 `atan2(y,x)`，所有 pair 必须满足最小 25°。

证据：`cooperative_viewpoint_core.h:169-189`。

该约束只筛选 viewpoint reference 候选，不进入 planner static/dynamic safety checker，因此没有新增 planner hard gate。

## Authoritative Static LOS

manager 直接包含并复用：

`plan_env/include/plan_env/static_los_geometry.h`

证据：`cooperative_viewpoint_core.h:6,85-93,191-280`。

没有复制新的圆柱模型、LOS margin 或 raycast 合同。动态 LOS 和 FOV 仅记录为 diagnostic，不参与第一版选择。

## Build 与合同测试

构建命令：

`catkin build multi_uav_formation ego_planner plan_env --no-status -j2`

结果：6 个依赖包全部成功，0 failed，0 abandoned，构建本身无 warning。catkin 报告了环境 `CMAKE_PREFIX_PATH` 与 cache 不同的既有提示，但未 clean、未删除 build/devel，且不影响本次构建结果。

C++ 合同测试：

`COOPERATIVE_VIEWPOINT_CONTRACT_TEST=PASS`

覆盖：feature OFF、phi=0、独立 reference、单 active UAV、25° separation、blocked/clear 排序、candidate static scope、persistent trigger、unsettled block、target freeze、rate limit、zero reversal、actual odom arrival、stale fallback。

静态合同测试：

`INDEPENDENT_VIEWPOINT_STATIC_CONTRACT=PASS`

确认真实 target topic 未替换、三个 reference topic 保留、默认 OFF、三路实际 odom、无 global rigid control flow、无第二 planner、三套 planner 独立、Stage1/2/3 OFF。

## Smoke

最终代码 smoke 目录：

`independent_viewpoint_validation_20260903/smoke_final/`

观察到一次真实 transition：

- UAV0，`phi: 0 → pi/2`。
- 预测可见率：0.714286 → 1.000000。
- 同时 active UAV 数：1。
- 其它两架 angle 不变。
- transition 日志持续给出 `reselection_blocked=1`、`direction_reversal=0`。
- 约 2.4 s 后 active reference 的中间静态几何变为 invalid，按允许合同执行 `CANCEL_INVALID`，回到最近 valid angle；没有重选、反转或 no-trajectory cascade。
- 正常 SIGINT 结束，无残留 ALP/ROS 进程。

Smoke 证明状态机能触发和执行单机 transition；不代表正式场景中的有效性。

## Strict OFF/ON A/B

主场景：`long_cylinder_forest_visibility_stress.json`

Scene SHA256：`430fce52ee31c4a4ea13607e39346e0f04f1af1e3da31eb426a44a61786c4cb0`

最终代码 SHA256：`3bf906f0130b4d8db1f7b50012d4049412121fb164b4026ff2ca2a158bad06ea`

数据目录：`independent_viewpoint_validation_20260903/viewpoint_ab_final/`

OFF/ON 六轮使用同一 frozen replay、0.928 m/s 固定 segment speed、`dynamic_motion_elapsed == target_elapsed`、同一 scene/code hash；target state-law 最大横向误差约 `6.7e-7 m`，最大速度误差约 `6.6e-7 m/s`。

| Run | Task | Dyn ep/samples | Unsafe samples | Static ep/samples | Tracking P95 m | Mean path m | Epoch→commit P95 ms | Jerk P95 | All-3 | Worst UAV | Transitions |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| OFF1 | YES | 0 / 0 | 10 | 0 / 0 | 3.883 | 81.755 | 48.388 | 68.224 | 87.781% | 89.859% | 0 |
| OFF2 | YES | 0 / 0 | 18 | 0 / 0 | 2.048 | 80.741 | 48.678 | 56.596 | 95.472% | 97.341% | 0 |
| OFF3 | YES | 0 / 0 | 53 | 0 / 0 | 2.980 | 81.924 | 49.696 | 62.394 | 90.905% | 96.221% | 0 |
| ON1 | YES | 1 / 15 | 81 | 0 / 0 | 2.451 | 82.272 | 48.805 | 61.326 | 92.935% | 96.237% | 0 |
| ON2 | YES | 1 / 12 | 71 | 0 / 0 | 1.807 | 81.283 | 48.140 | 52.113 | 95.806% | 96.470% | 0 |
| ON3 | YES | 0 / 0 | 80 | 0 / 0 | 2.151 | 82.404 | 49.181 | 57.292 | 96.095% | 97.216% | 0 |

组均值：

| Metric | OFF mean | ON mean | Change |
|---|---:|---:|---:|
| Dynamic collision episodes | 0.000 | 0.667 | regression |
| Dynamic collision samples | 0.000 | 9.000 | regression |
| Unsafe samples | 27.000 | 77.333 | +50.333 |
| Static collision episodes | 0.000 | 0.000 | none |
| Tracking P95 | 2.970 m | 2.136 m | -28.084% |
| Mean path length | 81.473 m | 81.986 m | +0.630% |
| Epoch→commit P95 | 48.921 ms | 48.709 ms | -0.434% |
| Executed jerk P95 | 62.405 | 56.910 | -8.804% |
| UAV1 visibility | 94.487% | 97.444% | +2.957 pp |
| UAV2 visibility | 98.948% | 99.638% | +0.690 pp |
| UAV3 visibility | 96.926% | 96.990% | +0.064 pp |
| Worst-UAV visibility | 94.474% | 96.641% | +2.168 pp |
| All-3 visibility | 91.386% | 94.945% | +3.559 pp |
| At-least-2 visibility | 98.975% | 99.127% | +0.152 pp |
| At-least-1 visibility | 100.000% | 100.000% | 0 pp |
| None-visible | 0.000% | 0.000% | 0 pp |
| Static LOS loss | 1.256% | 1.046% | -16.718% relative |
| Dynamic LOS loss | 1.288% | 0.930% | -27.751% relative |

按数值计算：

- apparent All-3 visibility loss reduction：41.318%。
- apparent worst-UAV visibility loss reduction：39.225%。

但这两个数不能作为算法收益：三个正式 ON run 的 transition start/completion/cancel 均为 0，三个 `phi_i` 始终为 0。持久遮挡事件在 ON1/ON2 分别被 `NO_VALID_CANDIDATE` 阻止，ON2 另有一次 `PLANNER_NOT_SETTLED`；没有实际 viewpoint movement。

因此 ON/OFF 差异属于重复运行随机性或启用 reference stream 后的运行时差异，而不是独立 angle 转移带来的可见性改善。

两次 ON 动态碰撞也都发生在 HOLD、`phi_0=phi_1=phi_2=0`、无 transition 的情况下：

- ON1：UAV1 与 `dynamic_gate_V2`，约 51.367–51.832 s，15 samples。
- ON2：UAV3 与 `dynamic_gate_V3`，约 73.747–74.115 s，12 samples。

所以不能把碰撞因果归于某次 viewpoint transition；但按严格保留判据，ON 组碰撞高于 OFF 仍构成安全回归否决。

## 安全回归

场景：`long_cylinder_forest_dynamic_gates_v2_c1_relaxed_v2.json`

Scene SHA256：`88f4c28938dc16f20a1da18eb905a21b9f80d5dc6b78ff46c69338fe6f4cb405`

数据目录：`independent_viewpoint_validation_20260903/safety_regression_final/`

| Run | Task | Dyn ep/samples | Unsafe | Static ep/samples | Tracking P95/max m | Mean path m | All-3 | Worst UAV | Transitions |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Safety ON1 | YES | 0 / 0 | 0 | 0 / 0 | 15.443 / 17.170 | 68.382 | 32.022% | 33.194% | 0 |
| Safety ON2 | YES | 0 / 0 | 6 | 0 / 0 | 2.332 / 4.523 | 81.938 | 95.887% | 95.887% | 0 |
| Safety ON3 | YES | 0 / 0 | 0 | 0 / 0 | 4.207 / 6.592 | 84.728 | 98.438% | 98.565% | 0 |

安全回归三轮 task complete，dynamic/static collision 均为 0，满足“无新增 static collision”。但 Safety ON1 出现严重 tracking collapse：tracking P95 15.443 m、max 17.170 m、out-of-range loss 36.389%、none-visible 18.794%。该轮 manager 始终 HOLD/phi=0，并连续记录 `PLANNER_NOT_SETTLED`，正确阻止了 viewpoint switch；因此 collapse 不是由独立 viewpoint movement 造成，但它使“无 tracking collapse”的默认开启合同无法通过。

三轮 frozen target 文件使用相同 waypoint/速度合同和同步 dynamic epoch。trajectory 中 target 几何与速度误差均约 `1e-6`；Safety ON1/ON3 的 logger callback-time 推断 epoch span 分别约 0.243 s/0.496 s，超过旧分析器 0.12 s 诊断阈值。这是 CSV callback time 抖动警告，不是 target path/speed 变化；报告不把它伪装成精确 header-stamp 对齐证据。

## Viewpoint Runtime 统计

正式 ON 主场景与安全回归合计：

- requested transitions：0。
- completed transitions：0。
- completed rate：N/A（分母为 0；分析 JSON 内保守记为 0）。
- incomplete transitions：0。
- direction reversals：0。
- max simultaneous transitions：0（smoke 实际观察到 1，合同上限为 1）。
- transition reselection violations：0。
- non-active angle change：0 rad。
- mean/P95 transition duration：N/A。
- mean/P95 arrival error：N/A。
- main ON `NO_VALID_CANDIDATE` reports：2。
- main ON `PLANNER_NOT_SETTLED` reports：1。
- safety `PLANNER_NOT_SETTLED` reports：12。

“一次只移动一架、transition 目标冻结、无反转”的控制合同由 C++ 测试和 smoke 证明；正式 benchmark 没有提供 completed-transition runtime 样本，不能声称完成率达到 80%。

## 与 Feedback 10 整体刚性旋转对比

| Metric | Feedback 10 rigid | Independent formal ON | Interpretation |
|---|---:|---:|---|
| Switches/run | 12 | 0 | 数量下降，但本轮其实没有触发有效转移 |
| Reversals/run | 8 | 0 | 独立状态机消除了反转；正式样本同时也无 transition |
| Path change vs OFF | +7.357% | +0.630% | apparent overhead lower |
| Jerk P95 change vs OFF | +44.2% | -8.804% | apparent overhead lower |
| All-3 visibility | 89.722% | 94.945% | 不能归因于 independent transition |
| Worst-UAV visibility | 94.224% | 96.641% | 不能归因于 independent transition |
| Dynamic collisions/run | 0.333 | 0.667 | worse |
| Static collisions/run | 0 | 0 | equal |
| Static LOS loss | 2.092% | 1.046% | apparent lower, non-causal |
| Dynamic LOS loss | 1.413% | 0.930% | apparent lower, non-causal |

独立设计在结构上修复了 Feedback 10 的“全编队一起移动、transition 中反复改方向”问题；但本次正式场景中零 transition，不能据此证明它获得了真实 visibility benefit。

## Final Decision

保留为可配置实验功能，生产默认保持 OFF。

否决原因按优先级：

1. 主场景 ON 比 OFF 多 2 个 dynamic collision episodes / 27 collision samples。
2. 正式 ON 与安全回归共 6 轮没有任何 viewpoint transition，visibility 改善无法归因于实现。
3. completed transition rate 无样本，不能满足至少 80% 的保留合同。
4. 安全回归一轮发生严重 tracking collapse，尽管 manager 正确保持 HOLD，整体默认开启稳定性仍未证明。

不继续调参数，不增加 assignment、auction、Hungarian、联合 MINCO 或新的 SIDE/A*/MINCO visibility patch。

## 最终字段

INDEPENDENT_VIEWPOINT_ANGLES_IMPLEMENTED:
YES

GLOBAL_RIGID_ROTATION_ACTIVE:
NO

VIEWPOINT_LAYER_ONLY_CHANGES_TRACKING_REFERENCE:
YES

TRUE_TARGET_TOPIC_PRESERVED:
YES

ONE_UAV_TRANSITIONS_AT_A_TIME:
YES

MAX_SIMULTANEOUS_TRANSITIONS:
0 in formal runs; 1 observed in smoke; contract maximum = 1

HOLD_TRANSITION_ONLY:
YES

TRANSITION_RESELECTION_BLOCKED:
YES

TRANSITION_DIRECTION_REVERSALS:
0

ACTUAL_ODOM_USED_FOR_TRIGGER:
YES

ACTUAL_ODOM_USED_FOR_ARRIVAL:
YES

PLANNER_UNSETTLED_BLOCKS_VIEWPOINT_SWITCH:
YES

MIN_VIEW_ANGLE_SEPARATION:
25 deg

PAIRWISE_SEPARATION_ENFORCED:
YES

REFERENCE_RATE_LIMIT:
0.25 rad/s

COMPLEX_ASSIGNMENT_OR_SECOND_PLANNER_ADDED:
NO

PLANNER_SAFETY_PIPELINE_CHANGED:
NO

STRICT_TARGET_REPLAY:
YES for frozen p_T/v_T and dynamic epoch configuration; logger callback-time jitter caveat recorded above

OFF_RUNS:
3

ON_RUNS:
3

SAFETY_REGRESSION_RUNS:
3

TASK_COMPLETE:
Main A/B 6/6; safety regression 3/3

DYNAMIC_COLLISION_REGRESSION:
YES — OFF 0 episodes/0 samples; ON 2 episodes/27 samples

STATIC_COLLISION_REGRESSION:
NO — all formal runs 0 episodes/0 samples

ALL3_VISIBILITY_OFF:
91.386%

ALL3_VISIBILITY_ON:
94.945%

ALL3_VISIBILITY_LOSS_REDUCTION:
41.318% apparent; not attributable because transition count = 0

WORST_UAV_VISIBILITY_OFF:
94.474%

WORST_UAV_VISIBILITY_ON:
96.641%

WORST_UAV_VISIBILITY_LOSS_REDUCTION:
39.225% apparent; not attributable because transition count = 0

ATLEAST2_CHANGE:
+0.152 percentage points

TRACKING_P95_CHANGE:
-28.084%

PATH_LENGTH_CHANGE:
+0.630%

EXECUTED_JERK_P95_CHANGE:
-8.804%

VIEWPOINT_TRANSITIONS_PER_RUN:
0.0

COMPLETED_TRANSITION_RATE:
N/A — 0 requested transitions in formal runs

MEAN_TRANSITION_DURATION:
N/A

FEEDBACK10_GLOBAL_SWITCHES_PER_RUN:
12

FEEDBACK10_GLOBAL_REVERSALS_PER_RUN:
8

INDEPENDENT_SWITCHES_REDUCED:
YES, but because formal independent transitions did not trigger

INDEPENDENT_REVERSALS_ELIMINATED:
YES

FINAL_INDEPENDENT_VIEWPOINT_DEFAULT:
OFF

NEW_HARD_GATE_ADDED:
NO

BUILD:
PASS

RUNTIME:
PASS — all nine formal runs completed and shut down cleanly; retention criteria failed

PREVIOUS_MAX_FEEDBACK_INDEX:
10

CURRENT_FEEDBACK_FILE:
feedback_11.md
