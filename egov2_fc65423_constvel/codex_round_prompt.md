# Task: apply exactly 5 audited fixes in this workspace, then build

Workspace root (ALL edits and builds happen here):
`/home/bob/ALP/egov2_fc65423_constvel`

Planner sources:
`/home/bob/ALP/egov2_fc65423_constvel/ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/`

| File | Path relative to the planner dir |
|---|---|
| FSM | `plan_manage/src/ego_replan_fsm.cpp` |
| FSM header | `plan_manage/include/plan_manage/ego_replan_fsm.h` |
| Manager | `plan_manage/src/planner_manager.cpp` |
| Manager header | `plan_manage/include/plan_manage/planner_manager.h` |
| Optimizer | `traj_opt/src/poly_traj_optimizer.cpp` |
| Optimizer header | `traj_opt/include/optimizer/poly_traj_optimizer.h` |

**Do NOT access `/home/bob/RRCT` (forbidden).**
**Do NOT fix anything outside the 5 items below.** Do not touch Joint / Early Joint,
target route geometry (including cylinder_09), SIGSEGV, traj_server future queue,
wall LOS radius, array bounds, safety thresholds, dynamic hard clearance, FREE TIME,
3ACK/common activation, controller, BODY/LOS semantics, or the feedback-80
`active_execution_touch_goal_` logic.

Line numbers below are from the current tree and may drift by a few lines; always
locate the quoted code before editing.

---

## FIX 1 — Kill the planning busy-loop

### Confirmed defect
`plan_manage/include/plan_manage/planner_manager.h` (~line 323):

```cpp
    bool localActivationPending() const {
      return traj_.local_traj.traj_id > 0 &&
          (executed_traj_id_ != traj_.local_traj.traj_id || executed_generation_ != active_traj_generation_);
    }
```

`ego_replan_fsm.cpp` `callReboundReplan()` (~line 749) starts with:

```cpp
    // A published successor has already reserved the next execution boundary.
    // Do not overwrite it while the executor still follows its predecessor.
    if (planner_manager_->localActivationPending())
      return true;
```

So "a successor is already pending" is reported as `success == true`. Then in
`execFSMCallback()` `REPLAN_TRAJ` (~line 339):

```cpp
      const int previous_id=planner_manager_->traj_.local_traj.traj_id;
      const bool success=planFromLocalTraj(1);
      const bool successor=planner_manager_->traj_.local_traj.traj_id!=previous_id;
      changeFSMExecState(success && (successor || planner_manager_->localActivationPending())
          ? EXEC_TRAJ : REPLAN_TRAJ, "MOVING_ROLLING");
```

keeps the FSM in `REPLAN_TRAJ`, which is handled again inside the same switch
iteration, so the whole `reboundReplan()` pipeline is re-run repeatedly within one
timer callback. Measured: 206 of 330 drone-0 replans within 10 ms of the previous one.

### Required semantics
`SUCCESS / PENDING / FAILED` must be distinguishable. Minimal implementation:

1. Add to `EGOReplanFSM` (header + cpp) an enum and a member, e.g.:

```cpp
  enum class ReplanOutcome { SUCCESS, PENDING, FAILED };
```

   and keep a `ReplanOutcome last_replan_outcome_` member set by
   `callReboundReplan()`; `callReboundReplan()` itself must **not** return `true`
   for the pending case. Simplest conforming change: change its return type to
   `ReplanOutcome` and update its 4 call sites
   (`planFromLocalTraj()` x3 around lines 1187/1193/1200, `planFromGlobalTraj()`
   around line 1130, `GEN_NEW_TRAJ` around line 1130).
2. In `callReboundReplan()`, when `localActivationPending()` is true:
   - **do not** run any MINCO/candidate work (nothing after that line may execute);
   - increment a new counter `pending_replan_suppressed_count_`;
   - return `PENDING`.
3. `planFromLocalTraj()` / `planFromGlobalTraj()` must return `false` for `PENDING`
   (no new plan was produced) **but must not** trigger the
   `callReboundReplan(true, false)` / random-retry / `planFromGlobalTraj()` fallback
   chain and must not touch `force_odom_resync_next_replan_`.
4. In the `REPLAN_TRAJ` case, if the outcome was `PENDING`, go back to
   `changeFSMExecState(EXEC_TRAJ, "PENDING_SUCCESSOR")` so the state change gates the
   next attempt to the next timer tick. There must be **no** path where
   `PENDING -> success -> REPLAN_TRAJ -> callReboundReplan()` happens twice inside
   one tick.

### Counters / logs (exact names, greppable)
In `EGOReplanFSM` add members and emit a periodic summary line (e.g. throttled 5 s)
**and** an end-of-run line with this exact prefix:

```
[replan-cadence] REPLAN_TOTAL_COUNT=%lu REPLAN_INTERVAL_LT_10MS_COUNT=%lu REPLAN_INTERVAL_LT_5MS_COUNT=%lu PENDING_REPLAN_SUPPRESSED_COUNT=%lu
```

- `REPLAN_INTERVAL_LT_10MS_COUNT`: number of consecutive replan starts whose wall-time
  interval is `< 0.010 s` (measure at the point where a replan is actually started,
  i.e. the entry of `callReboundReplan()` for non-pending outcomes).
- `REPLAN_INTERVAL_LT_5MS_COUNT`: same with `< 0.005 s`.
- `PENDING_REPLAN_SUPPRESSED_COUNT`: number of suppressed pending ticks.

**Target: the two interval counters must be near zero after the fix.**

---

## FIX 2 — rolling-target progress guard: remove the 7.5 m / 7.5 s unit error

### Confirmed defect
`ego_replan_fsm.cpp` (~lines 802-834):

```cpp
        if (satrt_tracking_ &&
            (local_target_pt_ - start_pt_).norm() < 0.5)
        {
          ...
          const double extension = std::max(1.0, planning_horizen_);
          const Eigen::Vector3d original_local_target = local_target_pt_;
          local_target_pt_ += progress_velocity * extension;
          local_target_vel_ = progress_velocity;
```

`planning_horizen_` comes from `fsm/planning_horizon` = **7.5 m** and is a *distance*
lookahead used by `getLocalTarget()` (`planner_manager.cpp:901+` scans the global
trajectory until `(pos_t - start_pt).norm() >= planning_horizen`). Multiplying the
tracked velocity by it produces a 7.0 m jump (and up to 21.6 m in the logs because
`local_target_vel_` is a 5 m-lookahead prediction velocity).

### Required fix
- Keep the guard's real purpose: stop a zero-progress local target from producing a
  useless sub-second rolling trajectory. Do **not** introduce a long artificial
  prediction.
- Use a genuine **time** quantity in seconds. Use the existing target-prediction
  horizon parameter that the planner already loads — `moving_obj_prediction_horizon`
  / `ploy_traj_opt_->getMovingObjPredictionHorizon()` (2.0 s in this scene) — as the
  authoritative `prediction_horizon_time [s]`. Do not invent a new numeric constant;
  if the value is unavailable, fall back to a clearly named seconds constant.
- Rename/clarify the units in the code so the confusion cannot recur: e.g.
  `planning_horizon_distance_m` (the 7.5 m lookahead) and `prediction_horizon_time_s`.
- Optionally clamp the extended point by the target's own predicted position at
  `start_pt`-relative prediction time. Do not add new safety gates.
- Keep the extension strictly local to this cycle; do not change
  `touch_goal_`/mission-terminal semantics.

### Counters / logs (exact names)
For every guard activation log one line and maintain a periodic + end-of-run summary
with this exact prefix:

```
[rolling-target-extension] ROLLING_TARGET_EXTENSION_COUNT=%lu ROLLING_TARGET_EXTENSION_TIME_USED=%.6f EXTENSION_DISTANCE_P50=%.6f EXTENSION_DISTANCE_P95=%.6f EXTENSION_DISTANCE_MAX=%.6f MAX_LOCAL_TARGET_FORWARD_JUMP=%.6f
```

- `ROLLING_TARGET_EXTENSION_COUNT`: activations.
- `ROLLING_TARGET_EXTENSION_TIME_USED`: the seconds value actually used
  (must be ~2.0, never 7.5).
- `EXTENSION_DISTANCE_P50/P95/MAX`: percentiles of the per-event extension length
  `|extended - original|` in metres (keep a small ring buffer / vector; compute at
  print time).
- `MAX_LOCAL_TARGET_FORWARD_JUMP`: maximum per-event `|extended - original|`.

**Target: no 7.5 m extension, no >20 m forward jump.**

---

## FIX 3 — unify the target / dynamic-obstacle time base

### Confirmed defect
The raw-LOS sampler already rebases the target to the planning epoch
(`planner_manager.cpp` ~6025-6029 uses
`object_pt + object_vel * max(0.0, planning_prediction_epoch - target_state_epoch)`),
but other consumers do not, and `evaluateDynamicRisk()` is called with
`ros::Time::now()` in three places:

- `planner_manager.cpp` ~9320-9322 (`initializer_prediction_epoch`)
- `planner_manager.cpp` ~9433-9435 (`fallback_prediction_epoch`)
- `planner_manager.cpp` ~9592-9593 (`result.risk = evaluateDynamicRisk(..., ros::Time::now().toSec(), ...)`)

while NOMINAL uses `planning_prediction_epoch` (~6286-6287). `result.risk` feeds the
safety class and `candidate_clearance_gain` (~9672-9678). `object_p_` is a 0.25 s
moving average (`ego_replan_fsm.cpp` ~1413-1488), so un-rebased consumers are also
stale by up to ~0.25 s.

### Required fix
1. Establish one authoritative helper for the target state at a world time, e.g.

```cpp
Eigen::Vector3d targetPositionAt(double world_time) const;  // object_p_ + object_v_ * (world_time - object_stamp)
Eigen::Vector3d targetVelocityAt(double world_time) const;  // object_v_
```

   Use the real target timestamp (`object_stamp_`, set from `msg->header.stamp` in
   `objectCallback`, not `ros::Time::now()`) as the reference time of `object_p_`.
   Expose it from `EGOReplanFSM` to `EGOPlannerManager` through the existing target
   hand-off (do not build a new subsystem; a stored stamp + accessor is enough).
2. Every target/dynamic prediction must go through the same
   `sample_world_time = activation_or_prediction_epoch + relative_time`:
   - raw LOS sampler (`planner_manager.cpp` ~6028-6088)
   - the target used by the optimizer visibility cost
     (`poly_traj_optimizer.cpp` ~5220/5276/5326: `object_p_ + object_v_ * t`)
   - `evaluateCandidateVisibility(...)` call (`planner_manager.cpp` ~9958-9960) and
     its internal `target = target_position + target_velocity * sample_t` (~4407)
   - `evaluateDynamicRisk(...)` call sites, including the 3 above
   - the final candidate re-risk (`~9592`)
3. `evaluateDynamicRisk()` and its callees must **not** call `ros::Time::now()`
   internally. Grep the whole function and its helpers; the epoch must come from the
   caller. Within one candidate evaluation the epoch/sample time must be fixed so a
   20 ms CPU delay cannot move the obstacles.
4. Keep one and only one target entry point: no call site may use bare `object_p_`
   for a geometry/cost decision any more, except the helper itself.
5. Audit counters (exact names in one periodic + end-of-run line):

```
[timebase-audit] MIXED_TARGET_TIMEBASE_COUNT=%lu MIXED_DYNAMIC_RISK_TIMEBASE_COUNT=%lu ROS_NOW_INSIDE_DYNAMIC_RISK_COUNT=%lu
```

   - `MIXED_TARGET_TIMEBASE_COUNT`: increments whenever a target position for a
     geometry/cost/visibility decision is computed from an un-rebased target
     (should be 0 after the fix).
   - `MIXED_DYNAMIC_RISK_TIMEBASE_COUNT`: increments whenever an
     `evaluateDynamicRisk` call site passes an epoch different from the batch's
     `planning_prediction_epoch` (should be 0).
   - `ROS_NOW_INSIDE_DYNAMIC_RISK_COUNT`: increments inside `evaluateDynamicRisk`
     if it would use `ros::Time::now()` (must be 0; keep it as a guard/assert-style
     counter).
6. Emit once per planning batch a consistency line so the numbers can be compared:

```
[timebase-consistency] drone=%d epoch=%.6f RAW_LOS_TARGET=(%.6f,%.6f,%.6f) OPTIMIZER_TARGET=(%.6f,%.6f,%.6f) CANDIDATE_VIS_TARGET=(%.6f,%.6f,%.6f) DYNAMIC_RISK_TARGET=(%.6f,%.6f,%.6f)
```

   The three/four positions must agree to numerical tolerance for the same sample
   world time.

---

## FIX 4 — when both L and R fail, fall back to NOMINAL immediately

### Confirmed defect / requirement
In the topology branch of `reboundReplan()` (`planner_manager.cpp`, the
`risk-candidate` / side-selection region ~10060-10460) the candidates are
NOMINAL / SIDE_PLUS / SIDE_MINUS with validity flags `plus_good` / `minus_good`.
When both side candidates failed there is no explicit, immediate
"NOMINAL for this round" decision.

### Required fix (keep it small)
Immediately before the selection finalization (just before
`CandidateResult *selected_before_visibility = selected_result;` ~line 10119):

```cpp
        const bool side_both_failed =
            (!plus_good || !plus_result.success) && (!minus_good || !minus_result.success);
        if (side_both_failed)
        {
          ++side_both_failed_count_;
          ++side_both_failed_fallback_nominal_count_;
          selected_result = &nominal_result;
          switch_blocked = true;
          switch_allowed = false;
          ROS_WARN("[side-both-failed-fallback] drone=%d generation=%lu nominal_id=%d "
                   "L_reason=%s R_reason=%s selected=NOMINAL reason=SIDE_BOTH_FAILED_FALLBACK_NOMINAL",
                   pp_.drone_id, static_cast<unsigned long>(active_traj_generation_),
                   nominal_result.candidate_id,
                   plus_result.success ? "OK" : plus_result.failure_reason.c_str(),
                   minus_result.success ? "OK" : minus_result.failure_reason.c_str());
        }
```

Use whatever the actual failure-reason field/name is in `CandidateResult`
(inspect the struct; if there is no reason string, log the existing
`candidate_status(...)` style text instead of inventing a field).

Constraints:
- Do **not** add: no-successor behavior, `TERMINAL_HOLD`, `VELOCITY_CONTINUATION`,
  new recovery candidates, coverage-extension gates, dwell/confirm margins, brakes,
  stop fallbacks, or a new recovery FSM.
- Do not change the existing behavior when at least one side is valid.
- Counters (report in the periodic + end-of-run summary, exact names):

```
[side-both-failed] SIDE_BOTH_FAILED_COUNT=%lu SIDE_BOTH_FAILED_FALLBACK_NOMINAL_COUNT=%lu
```

  Both must be equal.

---

## FIX 5 — LOS_OBSERVATION_SIDE plane must survive the retry/rebuild lifecycle

### Confirmed defect (first loss point)
In `reboundReplan()` the LOS observation plane is pushed into
`candidate_local_sfc_planes` around line 7320 (`LocalSfcPlane::LOS_OBSERVATION_SIDE`).
Later, when the A* repair path runs, the whole vector is **overwritten**:

```cpp
        local_sfc_planes = repaired_local_sfc_planes;      // ~line 8593
```

`repaired_local_sfc_planes` contains only static collision-corridor planes, so the
semantic LOS plane is dropped before the optimizer ever sees it. Finalization then
finds `frontend_has_los_plane == false` and cannot restore it, producing
`REJECT_ALTERNATIVE_LOS_PLANE_LOST` (~line 9660). The optimizer must actually receive
the LOS plane.

### Required fix
Split the plane set by semantics and re-combine instead of overwriting:

```cpp
        // A* / collision rebuild owns collision-derived planes only.
        std::vector<LocalSfcPlane> semantic_los_planes;   // source == LOS_OBSERVATION_SIDE
        for (const auto &plane : local_sfc_planes)
          if (plane.source == LocalSfcPlane::LOS_OBSERVATION_SIDE)
            semantic_los_planes.push_back(plane);
        local_sfc_planes = repaired_local_sfc_planes;
        local_sfc_planes.insert(local_sfc_planes.end(),
                                semantic_los_planes.begin(), semantic_los_planes.end());
```

Only preserve a semantic LOS plane when it is still valid: the blocker identity is
still the current one, `reason_mask` still contains `CONFLICT_LOS_OCCLUSION`, and the
active interval is usable (`active_start <= active_end`, finite, within the
trajectory duration). Do not change LOS observation geometry, the BODY side geometry,
or the Local-SFC algorithm itself. Do not "guess-restore" at finalization — the
optimizer must see the plane from the start.

### Lifecycle trace (needed to prove the fix)
Add a per-stage trace for the first candidate that would have lost its LOS plane.
Stages, in order:

```
SIDE_CREATED, AFTER_SIDE_SEED, AFTER_ASTAR, AFTER_LOCAL_SFC_BUILD, BEFORE_MINCO,
AFTER_MINCO, AFTER_RETIME, AFTER_DYNAMICS_RETRY, AFTER_FEASIBLE_FALLBACK,
BEFORE_SCP, AFTER_SCP, BEFORE_FINALIZATION, FINAL
```

Each stage line must print exactly:

```
[los-plane-lifecycle] stage=%s candidate_id=%d candidate_type=%s reason_mask=%u blocker_id=%d
  collision_plane_count=%zu los_plane_count=%zu total_plane_count=%zu
  los_plane{source=LOS_OBSERVATION_SIDE blocker_id=%d side_sign=%d normal=(%.6f,%.6f,%.6f)
  point=(%.6f,%.6f,%.6f) clearance=%.6f active_start=%.6f active_end=%.6f}
```

(one line is fine, keep it greppable). When the LOS plane count drops from >0 to 0
between two stages, log

```
[los-plane-lifecycle] FIRST_LOS_PLANE_LOSS_FUNCTION=%s FIRST_LOS_PLANE_LOSS_CONDITION=%s
```

If the plane count never drops but the final trajectory violates it, that is a legal
candidate failure — log it as a violation, not a loss.

### Counters (periodic + end-of-run summary, exact names)

```
[los-plane-audit] LOS_PLANE_CREATED_COUNT=%lu LOS_PLANE_LOST_COUNT=%lu LOS_PLANE_VIOLATION_COUNT=%lu
```

- `LOS_PLANE_CREATED_COUNT`: LOS_OBSERVATION_SIDE planes created.
- `LOS_PLANE_LOST_COUNT`: times a semantic LOS plane was dropped by any
  retry/rebuild/retime stage (target: 0 after the fix).
- `LOS_PLANE_VIOLATION_COUNT`: times the final trajectory really violated a
  preserved LOS plane (legal failure).

If it turns out that the plane count is never lost and only the
`active_start/active_end` time mapping is wrong after retime, fix the time mapping
instead and say so explicitly in your report; do not change LOS geometry.

---

## Build

Only this command (from `/home/bob/ALP/egov2_fc65423_constvel`):

```bash
source /opt/ros/noetic/setup.bash
source /home/bob/ALP/guidance/ros_ws/devel/setup.bash
cd /home/bob/ALP/egov2_fc65423_constvel
catkin build traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws
```

Do **not** run contract tests, smoke suites, hash tests, OFF/A-B matrices or any
unrelated test target. Do not start any simulation; the operator runs the simulation.

## Report when done

Reply with a short report containing:

- the exact files and line ranges you changed, one line per fix;
- the build result (success/failure, and the first error if it failed);
- for each of the 5 fixes: what the old behavior was, what the new behavior is, and
  the exact log/counter name that proves it at runtime;
- any place where the real code differed from the description above (say so
  explicitly instead of forcing the described edit);
- `CODE_MODIFIED_THIS_TURN: YES`;
- `RRCT_ACCESSED: NO`;
- `RRCT_CHANGED: NO`.
