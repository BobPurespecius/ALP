# Feedback 73 — Feedback61 N/L/R semantic restoration

本轮以 Feedback61 的 N/L/R 健康语义为基准，回查到当前
`planner_manager.cpp::EGOPlannerManager::reboundReplan()`。

## Root cause and fix

N_FIRST_BEHAVIOR_CHANGE_FUNCTION:
`EGOPlannerManager::reboundReplan()`（STEP 1: INIT）

N_FIRST_BEHAVIOR_CHANGE:
普通 hypothesis-0 调用原先可能因 `teamReferenceActive()` / generation 条件而进入 accepted warm start 或 `buildRecoveryGuideSeed()`；这会把旧 SIDE/guide/team geometry 带入 NOMINAL，偏离 Feedback61 的 authoritative P/V/A → normal local target → fresh initializer → MINCO/P/T 链。

LR_FIRST_BEHAVIOR_CHANGE_FUNCTION:
NOT_FOUND_IN_CURRENT_BODY_SIDE_DISPATCH

LR_FIRST_BEHAVIOR_CHANGE:
BODY conflict 的 SIDE 方向仍由 path/motion frame 的 lateral direction 构造；未发现 LOS observation frame 替换 BODY SIDE。`use_observation_frame` 仅在 LOS occlusion 且无 body hard conflict 时启用。

N_RESTORED_TO_FEEDBACK61_SEMANTICS: YES
BODY_LR_RESTORED_TO_ORIGINAL_SIDE_SEMANTICS: YES
LOS_ONLY_LR_ISOLATED_FROM_BODY_LR: YES
PERSISTENCE_CAN_MODIFY_NOMINAL: NO

FIX_APPLIED:
`encirclement_hypothesis_id == 0` 现在唯一表示不可污染的 NOMINAL baseline。N 跳过 accepted warm start 和 recovery guide，直接使用 authoritative P/V/A、normal local target 与 `buildFreshMovingInitializer()`；accepted warm start / recovery guide 仅保留给显式 alternative hypothesis。BODY L/R 的原 path-frame SIDE 保留，LOS-only 才添加 observation-side plane；未改变 dynamic hard threshold、FREE TIME、guide lifetime 或 Joint 架构。

## Fixed-scene evidence

SCENARIO: `long_cylinder_forest.json`（FULL ON，native RViz，当前合围配置）

本次仿真日志：`constvel_rviz.log`，在停止前取证窗口内：

- `nominal-baseline-semantics`：1262 次；所有 `nominal=1` 均为 `warm_start_allowed=0`；
- 实际选择：NOMINAL 396，SIDE_PLUS 181，SIDE_MINUS 241；
- LOS observation-plane 事件 1069；其中 87 次 observation seed 静态不可行后回退 `PATH_FRAME_SIDE`；
- `TERMINAL_HOLD_COUNT: 0`；
- `END_BEFORE_NEXT_COUNT: 0`；
- 没有出现本次运行期间的 hold/end-before-next 日志。

TERMINAL_HOLD_COUNT: 0
END_BEFORE_NEXT_COUNT: 0
UAV_LARGE_OSCILLATION: UNKNOWN（本轮只做短窗口语义验证，未作视觉量化）
MEDIUM_DENSITY_STUCK: NO（取证窗口内未出现 successor 断供）
TEAM_POSITION_SWAP_OBSERVED: UNKNOWN

日志中仍可见 `PERSISTENCE_FALLBACK`/`GUIDE_SEED_USED`，但这些来自显式 alternative、执行保活或 recovery 路径；它们不再被 hypothesis-0 NOMINAL 初始化使用。不能把这些 source 计数等同于 N 被改写。

## Scope guard

PRODUCTION_BEHAVIOR_CHANGED: YES（仅恢复 N 初始化语义）
SAFETY_THRESHOLD_CHANGED: NO
LIFECYCLE_CHANGED: NO
CONTROLLER_CHANGED: NO
JOINT_CHANGED: NO
BODY_LOS_TOPOLOGY_CHANGED: NO

PRODUCTION_SOURCE_CHANGED: YES
RRCT_ACCESSED: NO
RRCT_CHANGED: NO

