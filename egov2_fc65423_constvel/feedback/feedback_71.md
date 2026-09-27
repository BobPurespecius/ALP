# Feedback 071 — Stop / Spin / Sudden Acceleration Root-Cause Audit

本轮只读审查，固定场景为 `long_cylinder_forest.json`（10 个动态障碍、合围配置）。未启动新仿真，未修改 production 行为，未访问 RRCT。

## 结论

USER_VISIBLE_STOP_CONFIRMED: YES
POSITION_HOLD_WITH_ACTIVE_YAW_TRACKING: NO（代表 held UAV；没有持续 yaw-rate 证据）
SUDDEN_ACCEL_CONFIRMED: YES

PRIMARY_ROOT_CAUSE: 动态优化器/SCP 的动态预测覆盖范围只有 2.0 s，而 manager 的 `evaluateDynamicRisk()` 在 `touch_goal=true` 时检查完整候选时长。晚于 2 s 的真实动态冲突没有进入 moving cost/gradient 或 dynamic SCP row，候选随后被 manager 的硬动态准入拒绝。

SECONDARY_ROOT_CAUSE: 连续 `DYNAMIC_FAIL` 使 nominal 和 SIDE_PLUS 都不可执行，SIDE_MINUS 又被静态障碍拒绝，validated coverage 耗尽，因而没有 successor 接管。

TERTIARY_ROOT_CAUSE: `traj_server` 在 predecessor 到期且无替代轨迹时强制发布零线速度；下一条轨迹重新接管时要求非零速度，形成可见的停顿后加速。held UAV 的 yaw 被固定，不足以证明“原地绕圈”是 yaw 子系统造成的。

FIRST_CAUSAL_FAILURE_STAGE: DYNAMIC_REPAIR_FAILURE / FINAL_PREFLIGHT_REJECT
FIRST_CAUSAL_FAILURE_FUNCTION: `EGOPlannerManager::evaluateDynamicRisk()` 产生全时域风险结果，随后 candidate admission/final current-revision preflight 拒绝动态 clearance 小于阈值的候选；不是 traj_server 主动提前刹车。
FIRST_CAUSAL_FAILURE_CONDITION: candidate 的 manager dynamic clearance `< 1.10 m`，且没有同时满足静态、动态和动力学准入的 successor。

## 代表完整事件链

事件 UAV 为 drone 0，predecessor trajectory 62，来源 `PERSISTENCE_FALLBACK`：

```text
1789438053.684839  risk-candidate: obs=6, nominal=0.4037 m,
                   SIDE_PLUS=0.3584 m, SIDE_MINUS=FAILED (static)
1789438053.696806  SIDE_PLUS enters repair-only optimizer;
                   seed dynamic clearance=0.3148 m, required=1.1000 m
1789438053.697615  MINCO gradient audit: moving_cost=0,
                   moving_grad=0
1789438053.698191  final candidate clearance=0.3968 m;
                   candidate remains below hard threshold
1789438053.699155  predecessor 62 revalidation: EXPIRED,
                   remaining_duration=0
1789438053.699649  traj_server TERMINAL_HOLD_ENTER, trajectory 62
1789438053.699787  TRAJECTORY_END_BEFORE_NEXT_ACTIVATION
1789438053.745+  hold command keeps terminal position and zero linear
                   velocity/acceleration; odometry decays toward zero
1789438055.069246  successor 63 source timestamp (SIDE_MINUS)
1789438055.080868  first retained successor-63 odometry sample;
                   velocity begins from about 0.10 m/s and rises again
```

The predecessor validated/full end is `1789438053.695405722`; hold entry is `1789438053.699649811`, so the hold starts only after authority expires. The first retained successor sample is about 1.385 s after that validated end; a direct command recorder was not retained, so command-level delta-v is not numerically available.

## Coverage and command evidence

`encirclement_many_dynamic_20260915/full.log` contains 3 `TERMINAL_HOLD_ENTER` and 3 `TRAJECTORY_END_BEFORE_NEXT_ACTIVATION` events, all after candidate supply failure. For the representative event:

```text
predecessor_id=62
predecessor_validated_end=1789438053.695405722
full polynomial end=1789438053.695405722
next candidate ready/commit: no executable candidate before expiry
next scheduled activation: none before expiry
next actual activation: successor 63 observed at ~1789438055.069246 source time
gap from validated end to first retained successor sample: ~1.385 s
```

STOP_TRANSLATION_COMMAND_CONFIRMED: YES
HOLD_LINEAR_CMD_ZERO: YES
HOLD_YAW_ACTIVE: NO
YAW_CONTINUES_DURING_STOP: NO（held UAV）
XY_CIRCULAR_COMMAND_EXISTS: NO EVIDENCE
CONTROLLER_ONLY_CIRCLE: UNKNOWN

`traj_server.cpp` publishes terminal position with `Vector3d::Zero()` for velocity, acceleration and jerk, and yaw/yaw-rate `last_yaw_, 0.0`. Therefore the zero translation command is directly confirmed. `trajectory.csv` is odometry/state output, not a command recorder; it shows deceleration during hold and the successor recovering motion, but cannot establish a controller-only circle.

SUDDEN_ACCEL_PRIMARY_CAUSE: hold command forces linear velocity to zero, while post-deadline recovery/successor is initialized from propagated odometry P/V/A and requests nonzero velocity. Successor 63 starts around `0.107 m/s` in retained odometry and rises toward `0.4–0.5 m/s`; exact `Δv_cmd` is `NOT_DIRECTLY_RECORDED`, and `Δv_odom` is a gradual restart rather than an instantaneous controller step.

## SCP versus manager dynamic-check consistency

SCP_MANAGER_DYNAMIC_CHECK_CONSISTENT: NO
DYNAMIC_CHECK_MISMATCH_CLASS: SAMPLE/HORIZON

Manager code uses the complete duration for `touch_goal=true`; ordinary rolling additionally uses `2/3` duration and caps at the optimizer horizon. Optimizer moving cost, moving gradients, dynamic SCP rows, and `checkMovingObjSafety()` cap their dynamic sample set at `moving_obj_prediction_horizon_=2.0 s`.

Representative candidate 211 (drone 2, SIDE_PLUS, obstacle 8) demonstrates the mismatch:

```text
event time             ~1789446618.5285
duration               6.341238 -> 5.944248 s
conflict local time    ~6.2-6.4 s
seed/final clearance   0.498643 / 0.499247 m
required clearance     1.100000 m
SCP result              SCP_FINAL_OK (59 rows)
moving cost/grad        0 / 0 at init and final
```

The conflict occurs beyond the optimizer's 2 s dynamic sample horizon. `SCP_FINAL_OK` therefore means only that the covered prefix passed; it is not a proof that the complete candidate passed manager's full-time hard checker. The retained logs do not contain paired per-candidate SCP and manager prediction epochs/positions, so an epoch mismatch is only a possible secondary issue, not the primary finding.

SCP prediction epoch: logged from the candidate snapshot; manager prediction epoch: logged in `evaluateDynamicRisk()` call context. Exact paired values for candidate 211 were not retained.
SCP activation/local/world conflict time: local conflict is ~6.2–6.4 s and world time is prediction epoch plus local time; exact paired activation records were not retained.
SCP and manager distance definition: both use Euclidean center-to-center distance against constant-velocity predicted obstacle position; sample sets differ by horizon.

## Dynamic repair effectiveness

DYNAMIC_REPAIR_CHAIN_COMPLETE: NO
DYNAMIC_REPAIR_EFFECT: IMPROVED_BUT_STILL_UNSAFE / INCONSISTENT_CHECKERS
MOVING_COST_ACTIVE_ON_CONFLICT: PARTIAL (nonzero on conflicts within the 2 s window; zero for the late conflict above)
MOVING_GRAD_ACTIVE_ON_CONFLICT: PARTIAL (same horizon limitation)
P_REPAIR_EFFECTIVE: PARTIAL
T_REPAIR_EFFECTIVE: PARTIAL

Feedback 070's fix correctly wired moving terms and hard rows for the covered prefix, but no retained event shows unsafe seed -> clearance >= 1.1 m -> manager preflight PASS -> actual activation. This is why allowing repair-only seeds did not close successor supply.

## Safety-distance semantics

MOVING_OBJ_CLEARANCE_PHYSICAL_MEANING: `CENTER_TO_CENTER_THRESHOLD` (`moving_obj_clearance = 1.1 m`)
MIN_DISTANCE_PHYSICAL_MEANING: Euclidean center-to-center distance between predicted UAV center and predicted obstacle center.
Scene moving-obstacle radius is `0.28 m`. No production path was found that clearly adds this radius again to the 1.1 m threshold.
DOUBLE_INFLATION_EXISTS: NO EVIDENCE / UNKNOWN (not modified in this audit)

## Medium-density stuck event

MEDIUM_DENSITY_STUCK_PRIMARY_FAILURE: dynamic candidate rejection plus static SIDE failure. In the representative window nominal and SIDE_PLUS remain below 1.1 m; SIDE_MINUS has `NO_STATIC_FEASIBLE_SIDE`. This is a local executable-candidate shortage, followed by coverage expiry, not a server switch delay.

TEAM_POSITION_SWAP_DOWNSTREAM_OF_LOCAL_DISCONTINUITY: UNKNOWN. The retained evidence is insufficient to tie a specific team position swap to this hold event; no team architecture was changed.

COVERAGE_HOLE_COUNT: 3 (counted as `TRAJECTORY_END_BEFORE_NEXT_ACTIVATION` in the fixed-scene log)
STOP_SPIN_RESTART_EVENT_COUNT: 1 confirmed complete event chain; other holds are not claimed to have the same visual spin without command evidence.

## Required status

PRODUCTION_BEHAVIOR_CHANGED: NO
SAFETY_THRESHOLD_CHANGED: NO
LIFECYCLE_CHANGED: NO
CONTROLLER_CHANGED: NO
JOINT_CHANGED: NO
BODY_LOS_TOPOLOGY_CHANGED: NO
SCENARIO: `long_cylinder_forest.json`
RRCT_ACCESSED: NO
RRCT_CHANGED: NO

结论排序为：首先是 optimizer 与 manager 的 dynamic time-horizon 不一致；其次是由此产生的连续 dynamic rejection 和 coverage hole；最后是 terminal hold 的零速命令及 successor P/V/A 不连续造成的可见停顿和突然加速。held UAV 的原地转圈在当前数据中没有被证明为 yaw 或 XY 圆轨迹，不能作为第一根因。
