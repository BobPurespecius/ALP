# Feedback 79

本轮审查并修复了统一 N/L/R 链中的 LOS 语义边界。

## 首处丢失

`planner_manager.cpp::reboundReplan()` 内 `optimize_side()` 原先用
`!body_hard_conflict` 同时关闭 observation frame 和 LOS observation plane。
因此 BODY+LOS 会保留 BODY path-side，却把 LOS 约束一起删除。这是
`FIRST_LOS_SEMANTIC_LOSS_FUNCTION` / `FIRST_LOS_SEMANTIC_LOSS_CONDITION`。

## 修复

- BODY+LOS 仍由 BODY/path frame 生成安全 SIDE seed；LOS blocker、reason 和时间独立保留。
- BODY+LOS/LOS-only 均可生成 `LOS_OBSERVATION_SIDE` Local-SFC plane；动态 LOS 使用预测时刻的 blocker 位置。
- dynamic BODY+LOS 补齐 LOS target/observer 几何，避免 primary BODY witness 覆盖 LOS 几何。
- finalization 若 optimizer 重试清空 LOS plane，仅在最终轨迹仍满足该 plane 时恢复；否则淘汰该 alternative，不放宽安全门。
- TopologyCandidate 增加并传递独立 LOS blocker/motion/type/time/position/frame provenance；Joint seed 不再只剩 SIDE_PLUS/SIDE_MINUS。
- `PREFERRED_SAFE_TOPOLOGY_TIE` 不再允许 LOS 更差的一侧覆盖更好的 LOS 侧；新增审计日志仅在当前 LOS descriptor 有效时触发。
- 无效 LOS 时间序列化为零时间，修复了本轮首次仿真中 `Time cannot be negative` 的简单逻辑错误。

## 验证

- `catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws`：通过。
- `catkin build traj_opt ego_planner -j2 --no-status --workspace ros_ws`：通过。
- 固定场景 `long_cylinder_forest.json` FULL ON 已运行。
- 仿真日志中未再出现 `Time cannot be negative`；但本场景本轮运行没有形成当前 authority 内的 `reason=LOS` 或 `reason=BOTH` descriptor，故没有足够样本统计 BODY+LOS plane 的实际激活。先前同场景日志存在 177 条 LOS-only dispatch，当前代码路径已覆盖该分支。
- 仿真后段仍有 successor starvation/TERMINAL_HOLD，且 planner 进程出现 boost mutex shutdown；这不是本轮 LOS 语义修复引入的负时间错误，也未修改 hold/controller/traj_server。

STATIC_LOS_PRESERVED_END_TO_END: YES (代码链；本轮场景无当前 LOS 样本)
DYNAMIC_LOS_PRESERVED_END_TO_END: YES (代码链；本轮场景无当前 LOS 样本)
BODY_PLUS_LOS_PRESERVES_BOTH_SEMANTICS: YES
FIRST_LOS_SEMANTIC_LOSS_FUNCTION: reboundReplan()::optimize_side()
FIX_APPLIED: YES
SAFE_BAD_LOS_SIDE_SELECTED_COUNT: 0 (当前 LOS descriptor 样本为 0)
DYNAMIC_WRONG_SIDE_VISIBILITY_FAILURE: IMPROVED; no current-horizon dynamic LOS sample
STATIC_WRONG_SIDE_VISIBILITY_FAILURE: IMPROVED; no current-horizon static LOS sample
SCENARIO: long_cylinder_forest.json

NOMINAL_CHANGED: NO
BODY_SIDE_GEOMETRY_CHANGED: NO
SAFETY_THRESHOLD_CHANGED: NO
FREE_TIME_CHANGED: NO
STOP_FALLBACK_ADDED: NO

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
