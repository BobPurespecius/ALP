# Feedback 9

## 1. 当前代码入口与修改边界

- 生产源码树：`ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner`。
- 本轮开始时 worktree 已有大量未提交修改；未执行 `reset`、`checkout`、`clean`、
  commit 或格式化。
- 未修改 SIDE bilateral、semantic-v2、A*、Local-SFC 几何、repair timing、三类
  安全阈值、dynamic clearance、P/T trust、model-agreement、OSQP、max_jer、
  stale-head、controller、target-facing yaw 或场景 JSON。
- Stage1 candidate ranking、Stage2 A* LOS cost 和 Stage3 MINCO LOS cost 的生产默认值
  均保持 OFF：
  `plan_manage/launch/advanced_param.xml:109,117,119`，
  `multi_uav_formation/launch/native_egov2_rviz.launch:24,32,34`。

本轮正式工件：

- 运行/分析：`visibility_stage3_validation_20260903/`
- 聚合指标：`visibility_stage3_validation_20260903/stage3_metrics.json`
- 唯一正式报告：`feedback/feedback_9.md`

## 2. 三个逻辑漏洞修复

### 2.1 post-check 后 rerisk / reclass

`plan_manage/src/planner_manager.cpp:4867-4900` 在所有 dynamics/static/swarm
post-check retry 完成后、commit freshness check 前：

1. 将实际 `committed_MJO` 写回 selected `CandidateResult`；
2. 使用当前 prediction epoch 对最终 polynomial 重算 dynamic risk；
3. 重新执行 static/dynamics checker 和三类 safety classification；
4. 若最终不再是 ABSOLUTE_SAFE 且 previous ABS 仍有效，保留 previous；
5. 若无 previous ABS，最终真实 class 继续进入现有 nonblocking fallback 语义，
   不会沿用旧 ABS class。

运行证据：每个完整 run 有 829–859 条
`[post-check-rerisk-reclass]`，发生在 `[commit-head-freshness]` 之前。

### 2.2 accepted cache 携带真实 safety class

- cache 新增 `safety_class`、candidate kind、obstacle context、start time、generation、
  P/T；定义见 `plan_manage/include/plan_manage/planner_manager.h:233-245`。
- warm-start 入口只接受 `ABSOLUTE_SAFE`：
  `planner_manager.cpp:653-705`。
- commit 后只有最终 `ABSOLUTE_SAFE` 写入 `[accepted-safe-cache]`：
  `planner_manager.cpp:5006-5062`。
- IMPROVED_ONLY / INVALID 只记录
  `[last-committed-warm-cache] stored=0 ... UNSAFE_CLASS_NOT_CACHED`：
  `planner_manager.cpp:5063-5069`。

六轮主测试和一轮 C1 回归中：

- `unsafe_cached_as_safe = 0`
- `unsafe_warm_cache_stored = 0`

因此 INVALID fallback 不再进入 previous-safe/warm-safe cache。

### 2.3 nominal failure 不提前 return

`planner_manager.cpp:1682-1715` 将 nominal solve failure 构造成：

- `success=false`
- safety class 保持 INVALID
- 仅使用有效 `initMJO` 作为风险定位和 SIDE seed
- 日志为
  `CONTINUE_PREVIOUS_REVALIDATION_AND_SIDE_RECOVERY`

六轮完整运行中该 continuation 每轮实际出现 138–336 次；所有运行均完成任务，
证明 nominal failure 已不再绕过 previous revalidation 和 SIDE recovery。

## 3. Static LOS contract 统一

新增共享实现：

`plan_env/include/plan_env/static_los_geometry.h:17-230`

合同为 scene JSON 中的实心竖直圆柱 primitive，而不是只有表面点的 point cloud：

- scene cylinder parsing：`static_los_geometry.h:34-98`
- 线段到实心圆柱 signed clearance：`static_los_geometry.h:110-190`
- margin-squared soft penalty 及 observer/target gradient：
  `static_los_geometry.h:193-224`

相同 helper 同时用于：

- Candidate Visibility Report static LOS：
  `plan_manage/src/planner_manager.cpp:949`
- MINCO static LOS soft cost：
  `traj_opt/src/poly_traj_optimizer.cpp:3793-3910`

运行时三架 planner 均输出：

`[static-los-contract] loaded=1 ... cylinders=39 margin=0.080 geometry=SOLID_VERTICAL_CYLINDERS`

单元测试：

- LOS 穿圆柱中心：blocked，PASS
- LOS 擦圆柱表面：按 margin 判定，PASS
- LOS 完全绕开：clear，PASS
- observer analytic gradient 对 finite difference：PASS
- 实际 scene：39 个有效圆柱，PASS

最终测试输出：

`STATIC_LOS_GEOMETRY_TEST=PASS weighted_cost=11.76 raw_cost=0.0784 clearance=-0.2 gradient=(0,-42,0)`

## 4. MINCO Static-LOS Soft Cost

### 4.1 Objective 与真实梯度

配置：

- `enable_minco_visibility_cost=false`（生产默认）
- `lambda_vis=150.0`
- `sample_dt=0.10 s`
- `m_vis=0.08 m`，复用 candidate visibility static LOS margin
- target model：当前 constant-velocity prediction，覆盖完整 candidate duration

实现位置：

- 单时刻 LOS cost/gradient：
  `traj_opt/src/poly_traj_optimizer.cpp:3793-3821`
- 全 trajectory 梯形积分、coefficient gradient、time gradient：
  `poly_traj_optimizer.cpp:3824-3910`
- 加入真实 MINCO objective、`gdC`、`gdT`：
  `poly_traj_optimizer.cpp:5367-5391`

代价为：

`J_LOS = Σ dt * lambda_vis * max(0, m_vis - c_LOS)^2`

它只改变连续轨迹优化偏好；没有 hard row、hard gate、FAILED、E-stop、
no-trajectory 分支，也不改变 safety class。

### 4.2 实际作用量

30 s smoke：

- 283 条 final objective records
- 85 条非零 LOS cost
- 66 条非零 gradient
- `J_LOS / J_total` P95 = 6.00%
- 最大 = 14.05%
- crash / no-trajectory cascade / stale-head loop = 0

三轮 ON 完整运行均值：

- records/run = 1105.3
- nonzero cost records/run = 211.3
- nonzero gradient records/run = 172.7
- blocked samples/run = 1337.7
- `J_LOS / J_total` P95 = 1.79%
- 各轮最大值范围 = 12.13%–16.64%

因此 soft cost 确实进入优化且改变梯度，但没有长期超过 25%；本轮只使用这一组
权重，未循环调参。

日志：`[minco-visibility-cost]` 包含 candidate type、sample count、blocked samples、
mean/min clearance、J_LOS、J_existing、ratio、lambda 和 gradient norm。

## 5. Strict A/B Setup

主场景：

`ros_ws/src/multi_uav_formation/scenes/long_cylinder_forest_visibility_stress.json`

SHA256：

`430fce52ee31c4a4ea13607e39346e0f04f1af1e3da31eb426a44a61786c4cb0`

共同设置：

- frozen waypoint/speed replay
- dynamic obstacle epoch 与 target start 同步
- Stage1 OFF
- Stage2 OFF
- RViz OFF
- target-facing yaw ON
- max_jer=22
- final target stable stop 约 3 s 后 shutdown

六轮 `target_replay_contract.valid=true`；最大 target cross-track、progress 和 velocity
误差均小于 `1e-4`，scene hash 全部相同。

运行：

- MINCO visibility OFF ×3：3/3 task complete
- MINCO visibility ON ×3：3/3 task complete
- C1 relaxed v2 ON ×1：task complete

## 6. OFF vs ON 结果

下表为三轮平均值；括号给三轮 min–max。

| Metric | OFF | ON |
|---|---:|---:|
| Task complete | 3/3 | 3/3 |
| Dynamic collision episodes | 0.000 | 0.333 |
| Dynamic collision samples | 0.0 | 7.0 |
| Unsafe samples `<0.5 m` | 51.3 (17–89) | 90.3 (70–105) |
| Min dynamic surface clearance | 0.233 m (0.023–0.433) | 0.058 m (0.000–0.138, CSV clamp) |
| Static collision episodes | 0 | 0 |
| Min static clearance | 0.236 m | 0.155 m |
| Tracking mean | 1.597 m | 1.623 m |
| Tracking P95 | 1.978 m | 2.140 m |
| Mean path length | 81.050 m | 81.457 m |
| Optimizer kernel P95 | 0.896 ms | 1.938 ms |
| Planning epoch→commit P95 | 47.739 ms | 48.605 ms |
| UAV1 visibility | 97.051% | 96.966% |
| UAV2 visibility | 100.000% | 99.405% |
| UAV3 visibility | 97.272% | 97.064% |
| Worst-UAV visibility | 96.995% | 96.108% |
| All-3 visibility | 94.738% | 94.293% |
| At-least-2 visibility | 99.585% | 99.141% |
| At-least-1 visibility | 100.000% | 100.000% |
| None-visible | 0.000% | 0.000% |
| Static LOS loss | 1.329% | 1.196% |
| Dynamic LOS loss | 0.563% | 0.910% |
| FOV loss | 0.000% | 0.083% |
| Out-of-range | 0.000% | 0.000% |
| Executed acceleration RMS | 1.016 m/s² | 1.019 m/s² |
| Executed jerk P95 | 57.795 m/s³ | 58.608 m/s³ |

Executed jerk 是 odometry finite difference，不等于 MINCO analytic jerk。

### 6.1 碰撞回归

OFF 三轮均无碰撞。

ON 第 3 轮：

- UAV1 / V2 (`obstacle_id=1`)
- 1 episode / 21 samples
- 最深真实 surface penetration：`-0.165 m`
- 最深时刻：trajectory CSV `time_s=51.615`

证据：

- `stage3_ab/on_3/on_3_trajectory.csv`
- `stage3_ab/on_3/on_3.launcher.log`
- `stage3_metrics.json`

这足以判定 safety regression；本轮不把单次事件夸大为已证明的唯一 solver 根因，
但按预定保留合同必须停止并保持功能默认 OFF。

### 6.2 Solver 与 latency

每轮日志状态均值：

| Status | OFF | ON |
|---|---:|---:|
| SCP_FINAL_OK | 182.7 | 168.0 |
| QP_MAX_ITER_EXHAUSTED | 33.7 | 42.0 |
| P_TRUST_TOO_SMALL | 17.7 | 26.7 |
| DYNAMICS_MODEL_MISMATCH_TRUST_EXHAUSTED | 31.3 | 43.0 |
| TRUE_CONSTRAINT_INFEASIBILITY | 2.0 | 6.0 |

结论：solver failure 状态明显增加，SCP success 减少，判定 solver regression。

两种 latency 口径必须分开：

- optimizer kernel P95：`+116.35%`，超过 25% 门槛；
- planning epoch→commit P95：`+1.81%`，未超过门槛。

## 7. Visibility Loss Reduction

定义：`(L_OFF - L_ON) / L_OFF`。

| Metric | OFF loss | ON loss | Loss reduction |
|---|---:|---:|---:|
| All-3 | 5.262% | 5.707% | **-8.46%** |
| Worst-UAV | 3.005% | 3.892% | **-29.54%** |
| At-least-2 | 0.415% | 0.859% | **-106.71%** |
| Static LOS | 1.329% | 1.196% | **+10.04%** |

Static LOS loss 确有约 10% 相对改善，但低于 25% 成功门槛；all-3、worst-UAV
和 at-least-2 均恶化。Gate-local 结果也不稳定：V1/V2 的 all-3 均值下降，
ON 第 3 轮还在 V2 发生动态碰撞。

## 8. C1 relaxed v2 安全回归

场景：

`long_cylinder_forest_dynamic_gates_v2_c1_relaxed_v2.json`

SHA256：

`88f4c28938dc16f20a1da18eb905a21b9f80d5dc6b78ff46c69338fe6f4cb405`

结果：

- Task complete：YES
- Static collision episodes：0
- Minimum static clearance：0.151 m
- C1：SAFE，min dynamic clearance 1.077 m
- C2：SAFE，min dynamic clearance 1.137 m
- A1：COLLISION，1 episode / 21 samples，最深 penetration `-0.108 m`
- A1 最深时刻：`time_s=18.284`
- Tracking P95：2.133 m
- Planning epoch→commit P95：48.473 ms

因此 C1 本身没有回归，但整个 dynamic-gates 安全回归没有通过；这是保持 Stage3
默认 OFF 的第二项独立安全证据。

## 9. 验证

- `STATIC_LOS_GEOMETRY_TEST=PASS`
- `STAGE3_CONTRACT_TEST=PASS`
- Python compile：PASS
- `git diff --check`：PASS
- `catkin build path_searching traj_opt ego_planner --no-status -j2`：PASS
- Build summary：5 packages succeeded，0 failed，0 warnings
- 30 s smoke：PASS
- Strict A/B：6/6 完整运行结束
- C1 ON safety run：完成
- 最终残留 ALP/ROS 仿真进程：0

## 10. Final Decision

这次实验回答了目标问题：连续 MINCO 轨迹上的轻量 static LOS 偏好确实有真实梯度，
也使 static LOS loss 小幅下降，但收益不稳定且没有转化为三机总体可见性改善；同时
伴随一次主场景碰撞、一次安全回归场景 A1 碰撞、solver failure 增加和 kernel latency
显著上升。

按预先定义的停止规则：

- Stage3 保留为可配置实验功能；
- 生产默认 OFF；
- Stage1/Stage2 继续默认 OFF；
- 不进入 dynamic LOS、FOV constraint、多机联合 MINCO 或更复杂 Stage4；
- 不为改善结果调场景、安全阈值或第二轮权重。

## 11. Final fields

POST_CHECK_RERISK_RECLASS_FIXED:
YES

ACCEPTED_CACHE_SAFETY_CLASS_FIXED:
YES

INVALID_FALLBACK_IN_PREVIOUS_SAFE_CACHE:
NO

NOMINAL_FAILURE_EARLY_RETURN_FIXED:
YES

STATIC_LOS_CONTRACT_UNIFIED:
YES

MINCO_STATIC_LOS_SOFT_COST_IMPLEMENTED:
YES

MINCO_VISIBILITY_COST_HAS_REAL_GRADIENT:
YES

VISIBILITY_COST_IS_SOFT_ONLY:
YES

VISIBILITY_CAN_PROMOTE_UNSAFE:
NO

STAGE1_DEFAULT:
OFF

STAGE2_DEFAULT:
OFF

STRICT_TARGET_TRAJECTORY_REPLAY:
YES

STAGE3_RUNS:
OFF=3 / ON=3

ALL3_VISIBILITY_OFF:
94.738%

ALL3_VISIBILITY_ON:
94.293%

ALL3_VISIBILITY_LOSS_REDUCTION:
-8.46%

WORST_UAV_VISIBILITY_OFF:
96.995%

WORST_UAV_VISIBILITY_ON:
96.108%

WORST_UAV_LOSS_REDUCTION:
-29.54%

STATIC_LOS_LOSS_OFF:
1.329%

STATIC_LOS_LOSS_ON:
1.196%

STATIC_LOS_LOSS_REDUCTION:
10.04%

COLLISION_REGRESSION:
YES

TRACKING_P95_CHANGE:
+8.20%

PATH_LENGTH_CHANGE:
+0.50%

PLANNING_EPOCH_COMMIT_P95_CHANGE:
+1.81%

SOLVER_REGRESSION:
YES

STAGE3_RESULT:
INEFFECTIVE_OR_REGRESSIVE

FINAL_MINCO_VISIBILITY_COST_DEFAULT:
OFF

NEW_HARD_GATE_ADDED:
NO

BUILD:
PASS

RUNTIME:
PASS

PREVIOUS_MAX_FEEDBACK_INDEX:
8

CURRENT_FEEDBACK_FILE:
feedback_9.md
