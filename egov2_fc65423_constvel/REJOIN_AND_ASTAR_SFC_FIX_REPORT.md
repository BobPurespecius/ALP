# ALP A* → MINCO Geometry-Loss Fix Report

## 1. Scope and contract

本轮只修复已确认的 A*→MINCO 前端几何丢失控制流，并加入有限的 downstream rejoin 尝试。没有修改 A* 搜索算法、MINCO 数学、SCP/OSQP 数学或参数，也没有放宽 SIDE、静态、动态、动力学和 final checker 约束。

## 2. Source changes

修改集中在 `planner_manager.cpp` 的 SIDE candidate A* repair 分支：

- 原始 rejoin 保持为 `rejoin_attempt=0`，沿已有 trajectory samples 的后续方向最多增加两次 downstream rejoin，因此总尝试上限为 3。
- 每次尝试使用对应的 `collision_end` 与 `anchor_goal`，并继续使用现有 inflated occupancy 检查和现有 A* 搜索。
- 增加 `[astar-rejoin]`、`[astar-rejoin-fallback]`，并在 `[side-astar-repair]` 中记录 `rejoin_attempt/rejoin_index`。
- 修正 A*+Local-SFC 的 hand-off predicate：有效的 A* static-safe guide、SIDE semantic 和 Local SFC 不再要求未约束 MINCO rebuild 的动态/静态结果先通过，允许碰撞初值进入现有 P/T SIDE-SCP。
- 普通 nominal、DIRECT/BACKOFF SIDE candidate 仍保留历史 early-reject 路径。

## 3. Rejoin logic

代码使用 `max_rejoin_attempts = 3`。attempt 0 先执行；只有前一次没有得到可用 A* path 时，才沿现有 25 个 trajectory samples 的 downstream 方向选择后续 anchor。每个成功尝试立即采用其对应的 collision interval/goal 并停止搜索。没有改变 raw path、occupancy 或 A* success 定义。

本次 smoke 中所有观测到的 A* 尝试均为 attempt 0 成功，`[astar-rejoin-fallback]` 为 0；因此 fallback 分支已编译并可达，但本次场景没有自然触发真实 downstream fallback。

## 4. A* → Local SFC → SCP flow

当前有效放行条件为：

```text
astar_base_valid && path_static_free && path_side_valid &&
repaired_side_valid && local_sfc_build_valid
```

满足后立即构造 Local SFC 并进入现有 SIDE-SCP。未约束 MINCO 重建即使发生 static collision 或 dynamic invalid，也不再作为该分支的前端拒绝条件；其结果交由 Local SFC rows、Native static rows、SIDE corridor、v/a/j rows、free-time P/T、trust region、OSQP 和最终 nonlinear checker 联合处理。

在 retiming/constraint precheck 阶段，仅当 candidate 确实是 A* repair + Local SFC 有效分支时，才允许 collision result 继续传给 SCP。普通 SIDE/backoff/nominal 仍在原位置返回失败。

## 5. Side dynamic-valid semantics

`repaired_dynamic_valid` 现在是诊断量，不再阻断 A*+Local-SFC 到 SCP 的 hand-off。它仍会在完整 P/T SCP 和 authoritative final checker 中重新评估。这样避免把“未加约束、可能 cutting corner 的 MINCO 初值”误当成最终 candidate 可行性。

## 6. Build

执行：

```bash
cd /home/bob/ALP/egov2_fc65423_constvel/ros_ws
catkin build traj_opt ego_planner path_searching -j2 \
  --cmake-args \
  -Dosqp_DIR=/home/bob/ALP/egov2_fc65423_constvel/local_osqp_debs/extracted/opt/ros/noetic/lib/cmake/osqp
```

结果：`plan_env`, `traj_utils`, `path_searching`, `traj_opt`, `ego_planner` 全部 PASS；OSQP 配置/链接正常。

## 7. Smoke validation

日志：

- `rejoin_sfc_fix_smoke_20260901b.launcher.log`
- `rejoin_sfc_fix_smoke_20260901c.launcher.log`

配置使用 `long_cylinder_forest.json`、RViz OFF、当前 risk candidates/Hard Corridor SCP/Local A*/Local SFC 配置，未调参。第二次 smoke 的可观测结果：

| Metric | Result |
|---|---:|
| A* rejoin log records | 36（约 18 次 candidate attempt，每次两条记录） |
| attempt 0 / attempt 1 / attempt 2 | 36 / 0 / 0 |
| downstream fallback used | 0 |
| Local SFC `BUILD_SUCCESS` | 5 |
| `collision_bypassed_to_scp=1` records | 4 |
| Local SFC final records | 5 |
| SCP final `SCP_FINAL_OK` (全局 smoke) | 40 |
| Local SFC final failures | 5 |

Local SFC final failure reasons：`QP_MAX_ITER_EXHAUSTED` 2、`DYNAMICS_MODEL_MISMATCH_TRUST_EXHAUSTED` 2、`TRUE_CONSTRAINT_INFEASIBILITY` 1。它们均发生在后端 SCP/可行性层，不是前端 A* guide 被提前丢弃。

## 8. Obstacle 9 evidence

Obstacle 9 至少出现以下可复现链路：

```text
[side-local-sfc] ... planes=7 ... BUILD_SUCCESS
[side-local-sfc-control-flow] ... initial_minco_static_collision=1 collision_bypassed_to_scp=1
[side-local-sfc-control-flow] ... conflict_window_static_collision=1 collision_bypassed_to_scp=1
```

Obstacle 9 还有 `planes=8` 的 A*+SFC candidate。至少一个 obstacle-9 candidate 以 `SCP_FINAL_OK` 接受，且 `corridor_violation=0`；另有 candidate 在 QP max-iter 或 dynamics model-mismatch trust exhaustion 失败。失败 candidate 日志中的 `static_violation` 是保留的中间/失败轨迹诊断值，不能解释为 accepted trajectory 的 final collision。

因此：A* path 已到达 Local SFC，碰撞初值已真正越过前端门槛进入 SCP；本轮没有强行把后端不可行 candidate 判为成功。

## 9. Regression and safety checks

- SIDE topology/PLUS-MINUS semantic：未修改。
- Local SFC：未修改 plane 构造或 orientation 规则。
- Native static hard constraints：保留。
- SIDE hard corridor：保留。
- velocity/acceleration/jerk hard rows：保留。
- free-time MINCO P/T：保留。
- trust-region state machine：未修改。
- final static/nonlinear checker：未修改。
- target-facing yaw/camera layer：未修改。
- nominal candidate control flow：未修改。

Smoke 中无 dimension mismatch、bad array、segmentation fault、double free；未见本次改动导致的 NaN/Inf。退出后未发现 roscore/rosmaster/roslaunch/ego_planner_node/so3_control/swarm_bridge/simulator 残留进程。

退出阶段仍可见既有的 `boost::wrapexcept<boost::lock_error>` shutdown hygiene 信息；roslaunch 完成 cleanup，进程检查为空，未归因于本次 A*/SFC 改动。

## 10. Before/after failure layer

修复前：A* 找到 static-safe guide 后，MINCO cutting-corner 的初值碰撞会在 frontend 直接被拒绝，Local SFC/SCP 没有机会修复。

修复后：有效 A*+Local-SFC candidate 的碰撞初值可进入 SCP；失败已推进到可区分的后端类别（QP max-iter、dynamics model mismatch、true constraint infeasibility）。这证明本轮解决的是控制流/生命周期门槛，而不是宣称所有 candidate 都应成功。

## 11. Required final fields

```text
REJOIN_DOWNSTREAM_FALLBACK_IMPLEMENTED: YES
MAX_REJOIN_ATTEMPTS: 3
CURRENT_REJOIN_STILL_FIRST_PRIORITY: YES
ASTAR_SAFE_PATH_IMMEDIATELY_BUILDS_SFC: YES
UNCONSTRAINED_MINCO_EARLY_REJECT_REMOVED_FOR_VALID_ASTAR_SFC: YES
MINCO_CORNER_CUTTING_CAN_BYPASS_SFC: YES
SIDE_TOPOLOGY_SEMANTICS_PRESERVED: YES
SIDE_DYNAMIC_VALID_AUDIT: diagnostic only at A* hand-off; rechecked by full P/T SCP/final checker
NON_ASTAR_CANDIDATE_CONTROL_FLOW_CHANGED: NO
FREE_TIME_MINCO_PRESERVED: YES
TRUST_STATE_MACHINE_CHANGED: NO
YAW_CAMERA_LAYER_CHANGED: NO
VISIBILITY_LAYER_CHANGED: NO
FINAL_CHECKER_CHANGED: NO
BUILD: PASS
SMOKE: PASS
OBSTACLE9_REJOIN_FALLBACK_USED: NO
OBSTACLE9_ASTAR_PATH_REACHED_SFC: YES
OBSTACLE9_FINAL_COLLISION: NO for accepted final trajectories; failed intermediate diagnostics retained static_violation
NEW_PRIMARY_FAILURE_IF_ANY: QP_MAX_ITER / DYNAMICS_MODEL_MISMATCH / TRUE_CONSTRAINT_INFEASIBILITY
```

## 12. Conclusion

本轮已完成“有效 A* static-safe guide → Local SFC → 允许碰撞 MINCO 初值进入现有 SCP”的控制流修复，并通过 build 与 smoke 验证。剩余失败属于后端 QP/dynamics/真实约束层，不能再归因于 A*→MINCO 前端提前丢失几何。

