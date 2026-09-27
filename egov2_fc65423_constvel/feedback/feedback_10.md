# Feedback 10

日期：2026-09-03

## 1. 结论

已实现一个集中式轻量 Cooperative Viewpoint Manager。它只根据真实目标状态生成三架 UAV 的动态 tracking reference；三套 `ego_planner` 仍各自独立完成 SIDE → A* → Local-SFC → MINCO/SCP → OSQP、安全分类、生命周期和控制执行。

实现合同和构建/运行均通过，但严格 A/B 没有显示可见性收益：ON 相比 OFF 的 all-3 visibility 下降 2.966 pp，worst-UAV visibility 下降 1.691 pp，at-least-2 下降 1.565 pp。安全回归第二轮还出现 2 个静态碰撞 episode。因此新层保留为可配置实验功能，主 launch 默认保持 OFF；不再向 SIDE/A*/MINCO 叠加新的 visibility patch。

## 2. Worktree 与范围

- 修改前已确认 worktree 很 dirty；未执行 reset、checkout、clean、commit 或格式化。
- 未进入 `/home/bob/RRCT`。
- 未修改 SIDE、A*、Local-SFC、MINCO/SCP、OSQP、three-class safety、previous-safe、stale-head、controller 或场景几何。
- Stage1 candidate ranking、Stage2 A* visibility cost、Stage3 MINCO visibility cost 均保持默认 OFF。
- 本轮生产代码变化限于 viewpoint manager、tracking reference 接口、launch/CMake/package 接入及测试工具。

## 3. 架构与职责边界

运行链为：

```text
/object_odom（真实目标）
        |
        +--> yaw / prediction / visibility evaluator / true tracking metrics
        |
        `--> cooperative_viewpoint_manager
                 |
                 +--> /cooperative_viewpoint/uav0/reference
                 +--> /cooperative_viewpoint/uav1/reference
                 `--> /cooperative_viewpoint/uav2/reference
                          |
                          `--> 三套独立 ego_planner 的 tracking goal
                                   `--> 原安全规划与控制链不变
```

Manager 不发布 PositionCommand，不生成 polynomial trajectory，也不调用 A*、SFC、MINCO、SCP 或 OSQP。`/object_odom` 仍写入 planner 的真实 `object_p_/object_v_/object_q_`；cooperative reference 只参与 tracking goal 与对应尾端参考速度。

源码证据：

- Manager 订阅真实目标：`multi_uav_formation/src/cooperative_viewpoint_manager.cpp:32`。
- 三个独立 reference topic：同文件 `:67`。
- Manager 明确记录 `trajectory_generation=0 planner_calls=0`：同文件 `:73`。
- Planner 开关默认 false、timeout 0.5 s：`plan_manage/src/ego_replan_fsm.cpp:73-76`。
- 每个 planner 按 drone id 订阅自己的 reference：同文件 `:101-107`。
- cooperative reference 只替换 tracking goal：同文件 `:1105-1116`。
- stale/invalid reference 判定：同文件 `:1218-1224`。
- 原 Stage1/2/3 默认 false：`multi_uav_formation/launch/native_egov2_rviz.launch:24,32,34`。

## 4. 候选生成与联合评价

固定 identity offsets 为：

- UAV0: `(-1.50, -0.85, 0)` m
- UAV1: `(-1.70, 0.00, 0)` m
- UAV2: `(-1.50, +0.85, 0)` m

候选为整体绕世界 z 轴旋转：`phi=m*pi/4, m=0..7`。不做 UAV-viewpoint assignment。

每个候选在 2.0 s horizon、0.1 s sample dt 上检查：

- 三个 reference point 的实心圆柱静态自由性；
- target range；
- reference-to-target static LOS；
- signed/minimum LOS clearance；
- 由此得到 `R0 / R>=2 / Rmin / R3 / C_LOS,min / C_move`。

评价使用 Feedback 9 的 authoritative solid-cylinder helper：`plan_env/static_los_geometry.h`。核心调用位于 `cooperative_viewpoint_core.h:142,148`，没有复制另一套圆柱或 LOS 合同。

字典序位于 `cooperative_viewpoint_core.h:192`。静态自由是候选有效性的前置条件，visibility 不能覆盖它。动态障碍不参与 viewpoint 安全判断，仍由 planner 负责；运行 visibility CSV 继续独立统计真实 static/dynamic LOS、FOV 和 range loss。

为避免从当前角度旋转到目标角度时 reference 穿过静态柱，另对角度 sweep 做静态点检查，见 `cooperative_viewpoint_core.h:212-237`。这仍只是 reference validity，不生成或约束飞行轨迹。

## 5. 平滑、保持与 fallback

- 最大 reference angular rate：0.5 rad/s。
- minimum hold：1.0 s。
- visibility switch improvement threshold：0.04（4 pp）。
- 当前 reference 明显失效时允许切换；否则需 hold 完成且达到 improvement threshold。
- `phi_cmd` 采用 wrap-to-pi 后按 `omega_max*dt` clip，见 `cooperative_viewpoint_core.h:319-323`。
- reference velocity 为 `v_target + phi_dot * ez cross Rz(phi) r_i0`，见同文件 `:331-343`。
- 无有效候选时优先保持仍有效的 previous；否则发布 invalid fixed fallback，让 planner 恢复原 fixed formation，见同文件 `:277-287`。
- Planner reference freshness timeout 为 0.5 s；超时、invalid、节点未启动或开关 OFF 都恢复原 fixed formation offset。

RViz 输出为三个小 sphere marker 和 target-to-reference line，不复制 trajectory marker，见 `cooperative_viewpoint_manager.cpp:125-158`。

## 6. 合同测试与 smoke

### C++ 几何/状态合同

```text
COOPERATIVE_VIEWPOINT_CONTRACT_TEST=PASS
phi0=1 distances=1 blocked_ranking=1 rate_limit=1 invalid_fallback=1
```

证明：phi=0 恢复 fixed offsets；旋转保持三机相对距离；blocked formation 不优于 clear formation；rate limit 生效；无有效候选执行 fallback。

### 静态接入合同

```text
true_target_topic_preserved=1
reference_topics_distinct=1
default_off=1
no_second_planner=1
three_independent_planners=1
reference_stale_fallback=1
true_target_state_not_replaced=1
old_visibility_stages_off_by_default=1
COOPERATIVE_VIEWPOINT_STATIC_CONTRACT=PASS
```

### Smoke

最终 smoke 约 35 s：manager 约 10 Hz，planner 无崩溃、无 no-trajectory cascade；三个 planner 均能消费 fresh reference，startup/无有效 reference 时能 fallback。未发现 stale reference 被冒充为真实目标。

## 7. Build

命令：

```text
catkin build multi_uav_formation ego_planner --no-status -j2
```

结果：6 个相关包全部成功，0 failed，0 abandoned，0 warnings。

## 8. 严格 A/B 设置

主场景：

`ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_visibility_stress.json`

SHA256：`430fce52ee31c4a4ea13607e39346e0f04f1af1e3da31eb426a44a61786c4cb0`

安全回归场景：

`ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_dynamic_gates_v2_c1_relaxed_v2.json`

SHA256：`88f4c28938dc16f20a1da18eb905a21b9f80d5dc6b78ff46c69338fe6f4cb405`

所有运行：RViz OFF、target-facing yaw ON、max_jer=22、Stage1/2/3 OFF、target final stop 后约 3 s SIGINT shutdown。OFF/ON 使用同一 frozen target replay；分析验证 max target cross-track error 小于 `7e-7 m`，target velocity error 小于 `7e-7 m/s`。

数据目录：

- `cooperative_viewpoint_validation_20260903/viewpoint_ab/`
- `cooperative_viewpoint_validation_20260903/safety_regression/`
- 汇总：`cooperative_viewpoint_validation_20260903/viewpoint_metrics.json`

## 9. Visibility 场景 OFF ×3 / ON ×3

下表为三轮均值；括号为 min-max。

| Metric | OFF | ON | Change |
|---|---:|---:|---:|
| Task complete | 3/3 | 3/3 | — |
| Dynamic collision episodes | 1.000 (1-1) | 0.333 (0-1) | 未增加 |
| Dynamic collision samples | 13.67 (12-16) | 2.00 (0-6) | -11.67 |
| Unsafe samples | 63.0 (48-85) | 28.0 (0-50) | -35.0 |
| Static collision episodes | 0 | 0 | 0 |
| Min static clearance | 0.205 m | 0.218 m | +0.013 m |
| Tracking mean | 1.659 m | 1.514 m | -8.70% |
| Tracking P95 | 2.381 m | 2.397 m | +0.67% |
| Mean path length | 81.734 m | 87.747 m | +7.36% |
| Epoch→commit P95 | 47.601 ms | 48.895 ms | +2.72% |
| UAV1 visibility | 96.136% | 94.847% | -1.289 pp |
| UAV2 visibility | 99.155% | 96.094% | -3.062 pp |
| UAV3 visibility | 97.009% | 96.828% | -0.181 pp |
| Worst-UAV visibility | 95.915% | 94.224% | -1.691 pp |
| All-3 visibility | 92.688% | 89.722% | -2.966 pp |
| At-least-2 visibility | 99.612% | 98.047% | -1.565 pp |
| At-least-1 visibility | 100% | 100% | 0 pp |
| None-visible | 0% | 0% | 0 pp |
| Static LOS loss | 1.459% | 2.092% | +43.4% relative |
| Dynamic LOS loss | 1.117% | 1.413% | +26.5% relative |
| Executed jerk P95 | 61.980 m/s^3 | 89.345 m/s^3 | +44.2% |
| Viewpoint switches | 0 | 12.0/run (10-13) | +12/run |
| Switch reversals | 0 | 8.0/run (6-9) | +8/run |

Visibility loss reduction：

- All-3 loss reduction：`-40.57%`，即 loss 从 7.312% 增至 10.278%。
- Worst-UAV loss reduction：`-41.40%`，即 loss 从 4.085% 增至 5.776%。
- At-least-2 change：`-1.565 pp`。

主场景的 dynamic safety 数字改善，但不是保留功能的充分条件；visibility、static/dynamic LOS 和 executed jerk 均恶化。

## 10. ON ×2 安全回归

| Metric | ON Run1 | ON Run2 |
|---|---:|---:|
| Task complete | YES | YES |
| Dynamic collision episodes | 1 | 1 |
| Dynamic collision samples | 11 | 9 |
| Unsafe samples | 44 | 41 |
| Static collision episodes | 0 | 2 |
| Static collision samples | 0 | 16 |
| Min static clearance | 0.231 m | 0.000 m |
| Tracking P95 | 2.150 m | 1.953 m |
| Mean path length | 84.879 m | 86.659 m |
| All-3 visibility | 95.472% | 90.528% |
| Viewpoint switches | 11 | 14 |

Run2 的 2 个静态碰撞 episode 使安全回归合同失败。本轮不继续调 viewpoint 参数，也不修改 planner/scene 来消除结果。

## 11. 结果解释与停止决定

Manager 评价的是未来 2 s 的理想 reference 几何，而不是三套 planner 最终执行轨迹。独立 planner 会因静态/动态安全、求解延迟和轨迹生命周期而晚于或偏离 reference；整体 formation rotation 还引入了额外横向运动和转向过渡。因此“reference 点 LOS 更好”没有稳定转化为“真实 odom LOS 更好”。

观察到的因果证据：

1. ON 每轮发生 10-13 次 viewpoint switch，且 6-9 次方向反转；虽然满足实现的 hold/rate-limit 合同，但对独立 planner 仍形成明显的移动 reference 负担。
2. ON path length 增加 7.36%，executed jerk P95 增加 44.2%。
3. ON 的 static LOS loss 和 dynamic LOS loss 同时增加，而不是仅 evaluator 统计波动。
4. Manager 第一版按要求不把 dynamic LOS、FOV 或 planner trajectory feasibility纳入选择；它不能保证理想 reference 的 LOS 优势在过渡轨迹中成立。
5. 安全回归出现静态碰撞，故即使主场景 dynamic collision 更少，也不能默认开启。

保留判据第 1、3、4、7 条未满足：安全回归失败，visibility loss 未减少，at-least-2 下降超过 0.5 pp，且 reference 切换/执行平滑性代价明显。因此停止，不继续向 SIDE/A*/MINCO 添加可见性补丁。

## 12. 产物

生产实现：

- `multi_uav_formation/include/multi_uav_formation/cooperative_viewpoint_core.h`
- `multi_uav_formation/src/cooperative_viewpoint_manager.cpp`
- `multi_uav_formation/test/cooperative_viewpoint_contract_test.cpp`
- `multi_uav_formation/CMakeLists.txt`
- `multi_uav_formation/package.xml`
- `multi_uav_formation/launch/native_egov2_rviz.launch`
- `plan_manage/include/plan_manage/ego_replan_fsm.h`
- `plan_manage/src/ego_replan_fsm.cpp`
- `plan_manage/launch/advanced_param.xml`
- `plan_manage/launch/run_in_sim.launch`
- `run_constvel_gradient_rviz.sh`

测试/分析：

- `cooperative_viewpoint_validation_20260903/run_viewpoint_ab.py`
- `cooperative_viewpoint_validation_20260903/analyze_viewpoint.py`
- `cooperative_viewpoint_validation_20260903/viewpoint_contract_test.py`
- `cooperative_viewpoint_validation_20260903/viewpoint_metrics.json`

## 13. Final Fields

CENTRALIZED_LIGHTWEIGHT_VIEWPOINT_LAYER_IMPLEMENTED: YES

VIEWPOINT_LAYER_ONLY_CHANGES_TRACKING_REFERENCE: YES

TRUE_TARGET_TOPIC_PRESERVED: YES

PLANNER_SAFETY_PIPELINE_UNCHANGED: YES

GLOBAL_FORMATION_ROTATION_USED: YES

COMPLEX_ASSIGNMENT_OR_SECOND_PLANNER_ADDED: NO

REFERENCE_RATE_LIMIT_IMPLEMENTED: YES

REFERENCE_FALLBACK_IMPLEMENTED: YES

STRICT_TARGET_REPLAY: YES

OFF_RUNS: 3

ON_RUNS: 3

SAFETY_REGRESSION: YES

ALL3_VISIBILITY_LOSS_REDUCTION: -40.57%

WORST_UAV_VISIBILITY_LOSS_REDUCTION: -41.40%

ATLEAST2_CHANGE: -1.565 pp

TRACKING_P95_CHANGE: +0.67%

PATH_LENGTH_CHANGE: +7.36%

VIEWPOINT_SWITCH_COUNT: 36 total; 12.0/run mean; range 10-13

FINAL_VIEWPOINT_LAYER_DEFAULT: OFF

BUILD: PASS

RUNTIME: PASS

PREVIOUS_MAX_FEEDBACK_INDEX: 9

CURRENT_FEEDBACK_FILE: feedback_10.md

