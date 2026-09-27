# SFC_ZERO_PLANE_AND_STALE_HEAD_FIX_REPORT

日期：2026-09-02

## 1. 结论

两项指定修复都已实现并通过源码、构建、smoke 与真实运行日志验证：

- 合法的 Local-SFC zero-plane 结果现在被标记为 `SFC_NOT_REQUIRED`，可以继续进入 repair/MINCO/SCP。
- stale candidate 在 commit 前以最新 odom 做 0.5 m 检查，超限轨迹不会 commit、publish、activate 或写入 accepted cache。
- 非 FOV 模式下，位置 tracking error 和 trajectory expiry 都会从 odom 重新规划。
- 三轮共拦截 11 条 stale candidate，违规 commit 数为 0。
- 原 Run1/C2 的“command 后跳、command/odom 分离超过 1 m”没有复现。

但是三轮不是无碰撞：

- Run1：A1、C2 各一次。
- Run2：A1、C2 各一次。
- Run3：C2 一次。
- 所有碰撞前最终执行轨迹均为 `NOMINAL / INVALID / UNSAFE_FALLBACK`，command 本身也进入障碍物，不属于 execution-only collision。
- 当前主要瓶颈已转移到 zero-plane handoff 之后的 repair→MINCO/SCP 恢复链：病态初值、retime 后静态碰撞以及后续 model/trust/QP failure 造成没有 `ABSOLUTE_SAFE` replacement。

## 2. Worktree 处理

启动前确认 worktree 已有大量未提交修改。执行过程中：

- 未运行 `git reset`
- 未运行 `git checkout`
- 未运行 `git clean`
- 未 commit
- 未进入 `/home/bob/RRCT`
- 未覆盖或恢复既有修改
- `git diff --check`：PASS

本轮算法源码修改限制在：

- `/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/include/plan_manage/planner_manager.h`
- `/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/planner_manager.cpp`
- `/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/include/plan_manage/ego_replan_fsm.h`
- `/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/src/ego_replan_fsm.cpp`

## 3. Local-SFC zero-plane 修复

实现位于 `planner_manager.cpp:2542-2650`。

当前语义：

- `path_static_free=true`
- `semantic_valid=true`
- 输入、切向、法向、时间窗口均有效
- obstacle-side probe 没发现需要构造的约束面
- `plane_count=0`

结果为：

```text
local_sfc_build_valid=true
status=SFC_NOT_REQUIRED
handoff_allowed=true
```

如果 probe 表明需要平面，但最后没有有效 plane，或出现：

- 非有限输入
- 无效切向/法向
- 非法 active window
- plane 朝向非法
- semantic/path 本身无效

则仍然：

```text
status=SFC_BUILD_FAILED
handoff_allowed=false
```

关键源码：

- 输入及几何检查：`planner_manager.cpp:2551`
- 判断是否确实需要 plane：`planner_manager.cpp:2583`
- required-but-empty 仍失败：`planner_manager.cpp:2629`
- `SFC_NOT_REQUIRED`/`SFC_ACTIVE` 分类：`planner_manager.cpp:2632`
- handoff 日志：`planner_manager.cpp:2642`

## 4. stale-head / odom resync 修复

### 4.1 Odom resync 与 FOV 解耦

位置误差判断现在不依赖 `enable_fov_tracking_`：

```cpp
tracking_error > tracking_error_replan_thresh_
```

velocity error 保持原 FOV 行为，避免扩大本轮修改：

```cpp
enable_fov_tracking_ && velocity_error > velocity_threshold
```

相关位置：

- EXEC_TRAJ 检查：`ego_replan_fsm.cpp:245-258`
- local trajectory expiry：`ego_replan_fsm.cpp:720-728`
- local planning head tracking error：`ego_replan_fsm.cpp:731-743`
- 从 odom 构造 planning state：`ego_replan_fsm.cpp:681-689`

最终阈值仍为原有 0.5 m，没有调参。

### 4.2 Commit freshness

检查位于最终候选确定和 post-check 完成之后、`setLocalTrajFromOpt()` 之前：

- 最终 trajectory 的 `t=0` head：`planner_manager.cpp:4333`
- 获取最新 `odom_world`：`planner_manager.cpp:4337`
- 计算 head error：`planner_manager.cpp:4353`
- stale reject：`planner_manager.cpp:4369`
- 真正 commit 在检查之后：`planner_manager.cpp:4399-4405`

失败时设置一次性 replan request，由 FSM 消费：

- request 消费及清除：`planner_manager.cpp:535-543`
- 更新 FSM odom state：`ego_replan_fsm.cpp:626-638`
- failed plan 不进入 publish：`ego_replan_fsm.cpp:641-675`
- 下一轮强制 odom replan：`ego_replan_fsm.cpp:709-718`

没有实现 slicing、re-anchor 或 latency compensation。

## 5. 构建与 smoke

构建命令：

```text
catkin build path_searching traj_opt ego_planner --no-status -j2
```

结果：

```text
BUILD: PASS
Failed packages: 0
Abandoned packages: 0
```

最终 smoke：

- 运行约 30 秒
- exit code 0
- planner 未崩溃
- 无 no-trajectory cascade
- `SFC_NOT_REQUIRED=12`
- commit freshness checks：220
- `FRESH_COMMIT=219`
- shutdown 后因无新 odom 产生一次 `STALE_HEAD_REPLAN`
- stale 后 commit violation：0
- trajectory generation 正常递增

```text
SMOKE: PASS
```

## 6. 三轮基础指标

完整统计来源：

`/home/bob/ALP/egov2_fc65423_constvel/sfc_stalehead_fix_validation_20260902/validation_metrics.json`

| Metric | Run1 | Run2 | Run3 |
|---|---:|---:|---:|
| Task complete | YES | YES | YES |
| Dynamic collision episodes | 2 | 2 | 1 |
| Dynamic collision samples | 18 | 19 | 3 |
| Unsafe samples `<0.5m` | 57 | 56 | 29 |
| Minimum dynamic surface clearance | 0.000 m | 0.000 m | 0.000 m |
| Static collision episodes | 0 | 0 | 0 |
| Minimum static clearance | 0.178 m | 0.207 m | 0.199 m |
| Tracking mean | 1.782 m | 1.697 m | 1.794 m |
| Tracking P95 | 2.656 m | 2.604 m | 2.880 m |
| Mean path length | 81.587 m | 78.689 m | 80.921 m |
| Planning latency median | 28.05 ms | 26.93 ms | 26.92 ms |
| Planning latency P95 | 49.30 ms | 48.00 ms | 47.23 ms |
| Planning latency max | 815.32 ms | 621.48 ms | 1988.87 ms |
| All-3 visibility | 81.16% | 84.33% | 83.33% |
| At-least-2 visibility | 90.78% | 90.72% | 87.71% |
| At-least-1 visibility | 92.93% | 93.25% | 93.02% |
| None-visible | 7.07% | 6.75% | 6.98% |
| Executed jerk P95 UAV1 | 76.81 | 86.56 | 74.97 |
| Executed jerk P95 UAV2 | 90.05 | 76.98 | 72.15 |
| Executed jerk P95 UAV3 | 90.70 | 60.40 | 78.40 |

Executed jerk 为 odom finite-difference，仅用于运行间比较，不等同于 analytic MINCO jerk，也不直接与 `max_jer=22` 比较。

## 7. Gate 结果

| Run | Gate | Result | Min clearance | Unsafe | Collision |
|---|---|---|---:|---:|---:|
| 1 | A1 | COLLISION | 0.000 | 39 | 12 |
| 1 | B1 | SAFE | 1.073 | 0 | 0 |
| 1 | C1 | SAFE | 0.994 | 0 | 0 |
| 1 | C2 | COLLISION | 0.000 | 18 | 6 |
| 1 | A2 | SAFE | 0.875 | 0 | 0 |
| 2 | A1 | COLLISION | 0.000 | 33 | 12 |
| 2 | B1 | SAFE | 0.726 | 0 | 0 |
| 2 | C1 | SAFE | 0.939 | 0 | 0 |
| 2 | C2 | COLLISION | 0.000 | 23 | 7 |
| 2 | A2 | SAFE | 0.973 | 0 | 0 |
| 3 | A1 | SAFE | 0.949 | 0 | 0 |
| 3 | B1 | SAFE | 0.941 | 0 | 0 |
| 3 | C1 | SAFE | 0.894 | 0 | 0 |
| 3 | C2 | COLLISION | 0.000 | 29 | 3 |
| 3 | A2 | SAFE | 1.179 | 0 | 0 |

## 8. Local-SFC 专项统计

| Metric | Run1 | Run2 | Run3 |
|---|---:|---:|---:|
| SFC_ACTIVE | 10 | 16 | 17 |
| SFC_NOT_REQUIRED | 38 | 30 | 22 |
| SFC_BUILD_FAILED | 7 | 5 | 5 |
| Zero-plane handoff allowed | 38 | 30 | 22 |
| Zero-plane → SCP_FINAL_OK | 1 | 1 | 3 |
| Zero-plane → ABSOLUTE_SAFE | 1 | 0 | 1 |

总计：

```text
SFC_NOT_REQUIRED_EVENTS=90
SFC_NOT_REQUIRED_TO_SCP_SUCCESS=5
SFC_NOT_REQUIRED_TO_ABSOLUTE_SAFE=2
```

C2 RIGHT 专项：

| Metric | Run1 | Run2 | Run3 |
|---|---:|---:|---:|
| Zero-plane C2 RIGHT handoff | 13 | 12 | 5 |
| Reached candidate final status | 1 | 3 | 2 |
| SCP_FINAL_OK | 0 | 1 | 0 |
| ABSOLUTE_SAFE | 0 | 0 | 0 |

Run2 中存在完整链：

```text
C2 RIGHT A*
→ static-free
→ semantic PASS
→ plane_count=0
→ SFC_NOT_REQUIRED
→ MINCO/SCP
→ SCP_FINAL_OK
→ IMPROVED_ONLY
```

因此 zero-plane 已不再被 Local-SFC handoff 误杀；但目前尚未产生 C2 `ABSOLUTE_SAFE`。

## 9. stale-head 专项统计

| Metric | Run1 | Run2 | Run3 |
|---|---:|---:|---:|
| Freshness checks | 833 | 847 | 848 |
| FRESH_COMMIT | 829 | 845 | 843 |
| STALE_HEAD_REPLAN | 4 | 2 | 5 |
| Max checked head error | 0.943 m | 0.843 m | 1.478 m |
| Max actually committed head error | 0.485 m | 0.482 m | 0.494 m |
| Max solve latency | 0.815 s | 0.621 s | 1.989 s |
| Stale candidate commit violation | 0 | 0 | 0 |

总计：

```text
STALE_HEAD_REPLAN_EVENTS=11
STALE_CANDIDATE_PUBLISHED_AFTER_REJECT=0
MAX_COMMIT_HEAD_ERROR_CHECKED=1.477713 m
MAX_FRESH_COMMIT_HEAD_ERROR=0.494035 m
```

所有 11 次 stale request 均被 FSM 消费并转为 odom planning state。Run3 最后一次 replan 提示受 throttle 抑制，但下一周期 candidate head 直接来自刚记录的 odom，证明 resync 生效。

## 10. Command–odom tracking

| Metric | Run1 | Run2 | Run3 |
|---|---:|---:|---:|
| Error P50 | 0.046 m | 0.042 m | 0.045 m |
| Error P95 | 0.193 m | 0.167 m | 0.177 m |
| Error max | 0.813 m | 0.535 m | 0.521 m |
| Samples >0.5 m | 1 | 3 | 3 |
| Episodes >0.5 m | 1 | 2 | 2 |
| Backward command jump >1 m | 0 | 0 | 0 |

```text
MAX_COMMAND_ODOM_ERROR=0.813009 m
```

原 Run1/C2 的特征：

```text
command 突然向后跳
+ odom 继续向前
+ error 约 1.4 m
+ command safe / odom collision
```

三轮均未复现。

## 11. 三类轨迹统计

| Metric | Run1 | Run2 | Run3 |
|---|---:|---:|---:|
| ABSOLUTE_SAFE candidates | 799 | 814 | 838 |
| IMPROVED_ONLY candidates | 6 | 11 | 7 |
| INVALID candidates | 307 | 265 | 255 |
| New ABSOLUTE_SAFE selected | 792 | 805 | 826 |
| Previous ABS retained | 57 | 47 | 56 |
| IMPROVED_ONLY selected | 4 | 5 | 2 |
| INVALID/UNSAFE_FALLBACK selected | 37 | 37 | 20 |

日志确认：

- unsafe candidate 没有覆盖重新验证有效的 previous ABS。
- stale reject 发生在 commit 和 cache 更新之前。
- rejected stale candidate 没有进入 accepted/warm-start cache。

## 12. SIDE 后端状态

| Status | Run1 | Run2 | Run3 |
|---|---:|---:|---:|
| SCP_FINAL_OK | 37 | 42 | 50 |
| QP_MAX_ITER_EXHAUSTED | 12 | 11 | 12 |
| P_TRUST_TOO_SMALL | 4 | 4 | 7 |
| SPATIAL_TRUST_RETRY_EXHAUSTED | 1 | 0 | 3 |
| DYNAMICS_MODEL_MISMATCH_TRUST_EXHAUSTED | 13 | 8 | 13 |
| DYNAMICS_TRUST_RETRY_EXHAUSTED | 3 | 6 | 4 |
| TRUE_CONSTRAINT_INFEASIBILITY | 0 | 1 | 0 |
| OTHER | 2 | 0 | 3 |

`QP_MAX_ITER` 与 `QP_MAX_ITER_EXHAUSTED` 已合并统计。本轮未据此继续调参。

## 13. 残余碰撞快速尸检

| Run/Gate | UAV | Episode | Final trajectory | Predicted center distance | Command episode min surface clearance |
|---|---:|---|---|---:|---:|
| Run1 A1 | 1 | 18.945–19.312 s | NOMINAL / INVALID fallback | 0.410 m | -0.245 m |
| Run1 C2 | 1 | 63.113–63.279 s | NOMINAL / INVALID fallback | 0.339 m | -0.180 m |
| Run2 A1 | 1 | 18.689–19.060 s | NOMINAL / INVALID fallback | 0.313 m | -0.216 m |
| Run2 C2 | 1 | 62.590–62.789 s | NOMINAL / INVALID fallback | 0.428 m | -0.095 m |
| Run3 C2 | 1 | 62.371–62.444 s | NOMINAL / INVALID fallback | 0.395 m | -0.012 m |

五次碰撞中：

- active trajectory 均为 `NOMINAL`
- safety class 均为 `INVALID`
- selection reason 均为 `UNSAFE_FALLBACK`
- command trajectory 本身均进入动态障碍
- 因而不是 command-safe / odom-only collision
- A1 碰撞附近没有 stale reject
- C2 附近虽有 stale reject，但这些候选被正确拦截；随后发布的是重新计算且 head≤0.5 m 的 fallback

残余恢复链主要表现为：

```text
SFC_NOT_REQUIRED handoff
→ repair-time-init 仍出现明显超限 v/a/jerk
→ SIDE_INIT_RETIMED_STATIC_COLLISION
或
→ MINCO/SCP
→ model-agreement / trust / QP failure
→ 无 ABSOLUTE_SAFE replacement
→ previous ABS 已失效
→ INVALID NOMINAL emergency fallback
→ command trajectory collision
```

例如碰撞附近仍观察到 repair 初值：

```text
initial_v 约 5–10 m/s
initial_a 约 20–128 m/s²
initial_jerk 约 258–2215 m/s³
```

所以当前碰撞不再由本轮两个已修问题直接造成，而是 zero-plane 放行后暴露出的下游 recovery 可行性问题。

## 14. 验证产物

- `/home/bob/ALP/egov2_fc65423_constvel/sfc_stalehead_fix_validation_20260902/validation_metrics.json`
- `/home/bob/ALP/egov2_fc65423_constvel/sfc_stalehead_fix_validation_20260902/analyze_validation.py`
- `/home/bob/ALP/egov2_fc65423_constvel/sfc_stalehead_fix_validation_20260902/alp_1/alp.launcher.log`
- `/home/bob/ALP/egov2_fc65423_constvel/sfc_stalehead_fix_validation_20260902/alp_2/alp.launcher.log`
- `/home/bob/ALP/egov2_fc65423_constvel/sfc_stalehead_fix_validation_20260902/alp_3/alp.launcher.log`

三轮配置均为：

```text
scene=long_cylinder_forest_dynamic_gates.json
RViz=false
target-facing yaw=true
max_jer=22
three-class=true
arc-length/curvature timing=true
```

## 15. 最终字段

```text
LOCAL_SFC_ZERO_PLANE_SEMANTICS_FIXED:
YES

SFC_NOT_REQUIRED_DISTINGUISHED_FROM_BUILD_FAILED:
YES

VALID_ZERO_PLANE_ASTAR_PATH_CAN_REACH_MINCO:
YES

ODOM_RESYNC_DECOUPLED_FROM_FOV:
YES
(position error and trajectory expiry; velocity-only behavior remains FOV-gated)

COMMIT_HEAD_FRESHNESS_CHECK_IMPLEMENTED:
YES

STALE_HEAD_THRESHOLD:
0.5 m

STALE_CANDIDATE_CAN_BE_PUBLISHED:
NO

COMPLEX_TRAJECTORY_REANCHOR_IMPLEMENTED:
NO

BUILD:
PASS

SMOKE:
PASS

RUN1_RESULT:
TASK_COMPLETE; A1+C2 collision; 2 episodes; 18 collision samples; static collision=0

RUN2_RESULT:
TASK_COMPLETE; A1+C2 collision; 2 episodes; 19 collision samples; static collision=0

RUN3_RESULT:
TASK_COMPLETE; C2 collision; 1 episode; 3 collision samples; static collision=0

THREE_RUNS_TASK_COMPLETE:
YES

THREE_RUNS_COLLISION_FREE:
NO

A1_COLLISION_FREE_3_OF_3:
NO
(1/3 collision-free)

C2_COLLISION_FREE_3_OF_3:
NO
(0/3 collision-free)

SFC_NOT_REQUIRED_EVENTS:
90 (Run1=38, Run2=30, Run3=22)

SFC_NOT_REQUIRED_TO_SCP_SUCCESS:
5 (Run1=1, Run2=1, Run3=3)

STALE_HEAD_REPLAN_EVENTS:
11 (Run1=4, Run2=2, Run3=5)

MAX_COMMIT_HEAD_ERROR:
1.477713 m checked and rejected;
0.494035 m maximum among actually committed trajectories

MAX_COMMAND_ODOM_ERROR:
0.813009 m

STALE_HEAD_COLLISION_RECURRED:
NO

NEW_REGRESSION_FOUND:
YES
(runtime safety regression: A1 collision in 2/3 and C2 collision in 3/3;
causal attribution to the two patches is NOT PROVEN)

CURRENT_PRIMARY_BLOCKER:
Valid A*/zero-plane repaired paths now pass Local-SFC, but the downstream
repair→MINCO initialization/retiming/SCP recovery chain frequently fails.
This leaves no ABSOLUTE_SAFE replacement and commits the nonblocking
INVALID NOMINAL fallback, whose command trajectory directly collides.
```
