# Feedback 84 — Source-level logic audit after feedback 80-83

Scope: `egov2_fc65423_constvel`, read-only source audit of the code paths changed by
feedback 74-83, cross-checked against the latest runs `sim_run_103..109`.
No production code was modified. `RRCT_ACCESSED: NO`.

Evidence convention: `file:line` quotes come from the current working tree;
`sim_run_NNN.log` counts come from the run artifacts in the workspace root.

---

## 0. Verdict

The fixes reported in feedback 74-83 are present in the code and most of them
are correct in the form they were described. However five of them are only
partially effective, and the audit found **six independent logic defects** that
are currently active in the runs, plus a set of latent ones.

Highest-value items:

1. **A planning busy-loop that misreads "a successor is already pending" as
   "planning succeeded"** (active, quantifiable: 206 of 330 replans on one drone
   happened back-to-back within 10 ms).
2. **The rolling-target progress guard extends the local target by
   `planning_horizon * v_target` = 7.5 x 0.93 = 7.0 m, measured up to 21.6 m in
   the logs** - a unit confusion (time horizon used as a distance) that pushes
   the planned endpoint far past the target.
3. **A target-time-base split**: the raw-LOS code rebases the tracked target to
   the activation epoch, the optimizer cost and the side-selection visibility
   evaluator do not.
4. **`evaluateDynamicRisk` is called with `ros::Time::now()` in three places**,
   so a candidate's risk is compared against the NOMINAL risk on a different
   clock (the exact defect feedback 81 was supposed to have fixed).
5. **`TERMINAL_HOLD` still repeats** (11 in `sim_run_106`) because the
   post-deadline recovery path starts after the validated end and has no
   margin; and **JOINT coordination timed out in 381 of 381 attempts** in
   `sim_run_109`.

---

## 1. CONFIRMED active defects

### 1.1 Planning busy-loop: pending successor is reported as a successful replan

`planner_manager.h:323-326` (`localActivationPending()` is inline in the header):

```cpp
    bool localActivationPending() const {
      return traj_.local_traj.traj_id > 0 &&
          (executed_traj_id_ != traj_.local_traj.traj_id || executed_generation_ != active_traj_generation_);
    }
```

`ego_replan_fsm.cpp:749-752`:

```cpp
    // A published successor has already reserved the next execution boundary.
    // Do not overwrite it while the executor still follows its predecessor.
    if (planner_manager_->localActivationPending())
      return true;
```

`EGOReplanFSM::callReboundReplan()` therefore returns `true` without planning
anything, and `planFromLocalTraj()` (`ego_replan_fsm.cpp:1140`) propagates that
as `success`. The FSM then evaluates
`ego_replan_fsm.cpp:341-348`:

```cpp
      const int previous_id=planner_manager_->traj_.local_traj.traj_id;
      const bool success=planFromLocalTraj(1);
      const bool successor=planner_manager_->traj_.local_traj.traj_id!=previous_id;
      changeFSMExecState(success && (successor || planner_manager_->localActivationPending())
          ? EXEC_TRAJ : REPLAN_TRAJ, "MOVING_ROLLING");
```

`success == true` and `localActivationPending() == true` while `successor == false`
(re-visiting a trajectory committed on an earlier tick) keeps the FSM in
`REPLAN_TRAJ`. Because `REPLAN_TRAJ` is handled inside the same switch that just
selected it, the next loop iteration happens immediately and the full
`reboundReplan()` pipeline (candidate generation, MINCO solve, validation) runs
again on every iteration.

Measured in `sim_run_109.log` (29 s window):

```
drone 0: 330 replans, 206 of them within 10 ms of the previous one
drone 1: 248 replans, 106 within 10 ms
drone 2: 247 replans, 106 within 10 ms
```

The repeated banner/solve block is visible verbatim in the log, e.g.
`[drone 0 replan 49]` at `1789635698.629075` and `[drone 0 replan 50]` at
`1789635698.629850`.

Impact: burned CPU exactly when the planner is behind; every millisecond spent
in this loop is a millisecond of the predecessor coverage window. It also
starves the single global callback thread (see 1.6) and makes the joint window
(1.5) unreachable.

Root cause: `callReboundReplan()` has one return value for two different
outcomes. "A successor is already scheduled, do not overwrite it" must be a
distinct status so `planFromLocalTraj()` can report "no new plan produced"
without claiming failure, and the FSM should return to `EXEC_TRAJ`/wait instead
of re-entering `REPLAN_TRAJ` in the same tick.

### 1.2 Rolling-target progress guard: unit confusion, 7.5 m overshoot

`ego_replan_fsm.cpp:802-834` (added by feedback 82):

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

`planning_horizen_` is a **distance** lookahead: `getLocalTarget()` scans the
global trajectory until `(pos_t - start_pt).norm() >= planning_horizen`
(`planner_manager.cpp:901-925`), and `fsm/planning_horizon` is 7.5 in this
deployment (`sim_run_109.log:93`, `:178`). Multiplying the tracked velocity by
it produces a 7.5 m extension, i.e. 8.1 s of target motion at the scene speed
of 0.928 m/s, while the replan cadence is ~0.3 s.

Measured events (`sim_run_106.log`, 255 occurrences; 0 occurrences in 103, 105,
107, 108, 109 - it fires only while a drone is chasing the target closely):

```
distance=0.443656 extension=7.500000 original=(-26.727828,-0.894373,1.500570) extended=(-13.439591,-4.387234,1.495251)
distance=0.357599 extension=7.500000 original=(1.364754,-0.541547,1.500433)   extended=(14.610737,-0.310496,1.496019)
distance=0.488115 extension=7.500000 original=(15.947664,0.677468,1.499937)  extended=(37.512284,-0.391263,1.504185)
```

The third entry moves the planned endpoint 21.6 m, because `local_target_vel_`
is the global trajectory's velocity there, which in tracking mode is a
5 m-lookahead prediction (`ego_replan_fsm.cpp:1508-1509`,
`goal_lookahead = 5.0`) and therefore exceeds the target's own speed.

Impact: the planned endpoint is no longer the target; the UAV flies a long
segment past it, and the next rollerplan repeats the extension. That is a
plausible driver of the large loops/oscillation and of visibility loss, and it
also means the endpoint can be far outside the region whose safety was
evaluated. The guard's stated intent ("one normal rolling horizon") needs a time
basis (target velocity x ~1 s, clamped by the remaining distance to the target
and by map/goal bounds), not the 7.5 m planning lookahead used as a length.

### 1.3 Target time base split across the LOS/visibility/optimizer chain

The tracked target used by the planner is a moving average that is refreshed
only every 0.25 s (`ego_replan_fsm.cpp:1413-1488`: 10-message average, updated
when `(t_now - last_t) > 0.25`), so `object_p_` lags the true target by roughly
0.1-0.2 s.

The raw-LOS truth sampler compensates (feedback 81) - `planner_manager.cpp:6025-6029`:

```cpp
    const double target_state_epoch = t_start.toSec();
    const Eigen::Vector3d target_at_prediction_epoch =
        object_pt + object_vel * std::max(0.0, planning_prediction_epoch - target_state_epoch);
```

but the other consumers of the same target do not:

- optimizer cost: `poly_traj_optimizer.cpp:5220`, `5276`, `5326`
  (`const Eigen::Vector3d target = object_p_ + object_v_ * t;`)
- the visibility report that feeds side selection:
  `planner_manager.cpp:9958-9960` ->
  `evaluateCandidateVisibility(..., planning_prediction_epoch, object_pt, ...)`
  with `target = target_position + target_velocity * sample_t`
  (`planner_manager.cpp:4407`)
- while a sibling call site *does* rebase
  (`planner_manager.cpp:1536-1540`).

So in one planning cycle the same target is modelled at two different world
positions, offset by `(planning_prediction_epoch - target_state_epoch) * v`:
about 0.10 s * 0.93 m/s = 0.09 m by default and up to ~1 s in team-transaction
mode (`planner_manager.cpp:1757`), plus the 0.25 s averaging lag in the
un-rebased consumers - i.e. up to a few tenths of a metre of target position
disagreement between "where the LOS witness says the target is" and "where the
optimizer thinks the target is".

Impact: LOS geometry, visibility ranking and the trajectory cost are computed
against systematically different target states; side selection can prefer the
side that is correct for the wrong target position.

### 1.4 `evaluateDynamicRisk` still called on `ros::Time::now()` in the side candidate path

`planner_manager.cpp:9320-9322`, `9433-9435`, `9592-9593`:

```cpp
            const double initializer_prediction_epoch = ros::Time::now().toSec();
            feasible_initializer_risk = evaluateDynamicRisk(
                side_init_traj, initializer_prediction_epoch, touch_goal);
...
              const double fallback_prediction_epoch = ros::Time::now().toSec();
...
            result.risk = evaluateDynamicRisk(
                result.min_jerk_opt.getTraj(), ros::Time::now().toSec(), touch_goal);
```

while NOMINAL uses the planning epoch (`planner_manager.cpp:6286-6287`).
`result.risk` then drives the safety class and
`candidate_clearance_gain = result.risk.min_distance - nominal_result.risk.min_distance`
(`planner_manager.cpp:9672-9678`).

Impact: the acceptance test compares a `now()`-epoch SIDE risk against an
activation-epoch NOMINAL risk. On a 0.1 s activation margin at ~1 m/s this is a
~0.1 m clearance bias in the comparison that decides whether a side candidate is
accepted - the same class of defect feedback 81 fixed for the raw LOS witness,
just in the risk evaluator.

### 1.5 TERMINAL_HOLD still repeats; post-deadline recovery has no usable window

`sim_run_105` 0 holds, `sim_run_106` **11 holds**, `sim_run_107` 1,
`sim_run_108` 0, `sim_run_109` 1.

The `sim_run_106` hold at `drone_2 / trajectory_id=207` shows the mechanism:

```
VALIDATED_MOVING_COVERAGE_REMAINING drone=2 trajectory_id=207 now=1789633190.572041512
   t_validated_end=1789633190.577506542 t_planning_deadline=1789633190.477506...
POST_DEADLINE_RECOVERY_ATTEMPT drone=2 planning_wall_budget=0.559541
   ... old_deadline_authority=0
future-activation drone=2 activation_earliest=1789633190.672084332 t_validated_end=1789633190.577506542
local-geometry-candidate drone=2 candidate=761 safe=0 reason=HARD_DYNAMIC_COLLISION_FAIL
...
TERMINAL_HOLD_ENTER /drone_2_traj_server trajectory_id=207
```

The validated coverage had 5.5 ms left when the recovery started, and the
recovery's activation earliest was 100 ms later - so the fresh trajectory could
never activate before the predecessor's validated end, and every candidate was
rejected with `HARD_DYNAMIC_COLLISION_FAIL`. The planner ended the run in
`post-deadline recovery` and had to be rescued by the executor hold.

Impact: the feedback 80 fix correctly stopped the terminal/rolling semantic
mix-up, but the remaining failure is that validated coverage is allowed to
decay to (almost) zero before the recovery path is entered. The successor must
be requested while the recovered activation can still fit inside the validated
window.

### 1.6 Joint coordination never completes inside its own window

`sim_run_109.log`: 381 `TEAM_PENDING_START`, 381 `TEAM_PENDING_END`, and
`JOINT_FAILURE_LOCAL_NOOP reason=JOINT_TIMEOUT` (728 raw occurrences, 446 by event
tag) - i.e. every joint attempt timed out. `NO_ADMISSIBLE_JOINT_TAIL` adds 82.

Observed pending durations: `duration=0.200268 .. 0.210100`, while the
configured coordination budget is `team_coordination_timeout = 0.35`
(`sim_run_109.log:181`, param default `planner_manager.cpp:461`,
`pending_topology_.deadline = std::min(execution_safe_until_ - local_activation_margin_, now + timeout)`
at `planner_manager.cpp:3340-3345`).

Contributing causes visible in the code:

- the planner node runs every callback on **one thread** (`ego_planner_node.cpp:53`
  `ros::spin()`); the joint result callback, the peer trajectory callbacks and the
  FSM timer are serialized behind the same thread that is also executing the
  busy-loop of 1.1 and synchronous MINCO solves;
- `finalizeCapturedCandidates()` re-enters the joint wait on every batch, so a
  0.2 s budget is repeatedly started and then abandoned
  (`planner_manager.cpp:4000-4009`);
- the executor's adoption queue never received a queued/promoted entry in any of
  runs 103-109 (`traj-server-scheduled-queue`, `-promote`, `-supersede` = 0
  everywhere), so the "ordered future queue" added by feedback 77 has never been
  exercised in the current scenario.

Impact: the joint path currently consumes planner time and returns
`JOINT_FAILURE_LOCAL_NOOP` every cycle; the intended quality gain is zero while
the cost lands exactly on the local rolling deadline.

### 1.7 Target route static geometry: 1.9 mm violation and no authority

feedback 83 reported four static route violations. The route was changed since
then; the **current committed scene still violates**, by a small margin:

```
$ python3 scripts/validate_target_route_static.py scenes/long_cylinder_forest.json
{"collisions": [{"key": "9", "name": "long_forest_cylinder_09", "primitive": "cylinder",
  "residual_m": -0.0019095449580511792, "segment": 6}],
 "expanded_margin_m": 0.3, "minimum_target_static_margin_m": -0.00191, ...}
exit code 2
```

An independent recomputation (target radius 0.25 + clearance 0.05, obstacle
radius 0.43, route from `targetTracking.waypointsENU`) gives the same result and
shows how thin the whole route is:

| obstacle | min clearance (m) |
|---|---:|
| cylinder_09 | -0.0019 |
| cylinder_01 | +0.2095 |
| cylinder_15 | +0.2667 |
| cylinder_20 | +0.3031 |
| cylinder_11 | +0.4200 |

`target_state_coordinator.py:241-304` (`prepare_dynamic_safe_speeds`) evaluates
**only** `movingObstacleData`; there is no static check, and the fallback when no
speed scale is safe is
`choice = min(safe, ...) if safe else max(choices, key=lambda item: item[0])`
(`target_state_coordinator.py:294-295`) - i.e. the least-bad dynamic clearance is
committed instead of the route being rejected. Nothing in
`run_constvel_gradient_rviz.sh` invokes `validate_target_route_static.py`
(the only `exit 2` in the script is the missing-scene-file check at line 93).

Impact: the target has no physical authority that prevents a static collision.
The remaining 1.9 mm is inside the validator's own margin, and since the route's
min clearance to cylinder_09 is small, any steering/speed deviation puts the
target inside an obstacle. A static pre-pass (or at least a hard
"reject route" gate) and a validator call in the run script are the missing
pieces.

---

## 2. Latent defects (correct today, wrong if the path is exercised)

1. `traj_server.cpp`: `scheduled_traj_queue_` is cleared on exactly one path
   (`traj_server.cpp:761`, the `SCHEDULED_ACTIVE_PVA_MISMATCH` branch). The
   immediate-activation path (`:497-515`), the new-trajectory `RECEIVE` reject
   (`:409-413`), `teamSolutionCallback` (`:96-123`), the terminal hold and the
   heartbeat-stale path never clear it. Since queued entries then wait forever
   for a `scheduled_*` promotion that never comes, a stale entry can later be
   selected as "the predecessor chain" by `use_queued_predecessor`
   (`:178-198`) and reject a valid successor. No queued entry occurred in runs
   103-109, so this is latent rather than active.
2. `planner_manager.cpp:2952-2962`: `message.los_conflict_time` is exported on
   the trajectory-relative clock while `message.los_first_risk_world_time` is
   absolute. The one consumer stores it for logging only
   (`multi_uav_topology_coordinator.cpp:658`), so the mismatch is currently
   invisible.
3. `message.los_conflict_valid` (`planner_manager.cpp:2946-2947`,
   `planner_manager.h:31`) is written and then read only by a log line
   (`multi_uav_topology_coordinator.cpp:659`). The "BODY-only must not enter the
   visibility selector" guarantee is enforced by the `reason_mask` checks at
   `planner_manager.cpp:10178-10184` and `10229`, not by this flag.
4. `planner_manager.cpp:6356-6379` is unreachable: `:6315-6318` already sets
   `reason_mask |= CONFLICT_BODY_SAFETY` whenever `body_risk_triggered`, so the
   guard at `:6356-6357` never passes. The live block at `:6315-6330` fills the
   dedicated `body_obstacle_*` fields (so the BODY evidence itself is not lost),
   but the legacy primary fields `conflict.obstacle_motion / obstacle_identity /
   obstacle_position / observer_position / first_risk_world_time /
   risk_interval_start / risk_interval_end` stay at their defaults
   (`-1 / (0,0,0) / 0`), and `TopologyCandidate.msg:45-48` documents those fields
   as "the safety witness". `TopologyCandidate.msg` has no `body_*` fields, so a
   dynamic BODY-only conflict is exported with an empty primary witness; the
   current consumer happens to read only `conflict_reason_mask` and the `los_*`
   fields (`multi_uav_topology_coordinator.cpp:654-658`), so this is latent, but
   any consumer that starts reading the primary witness will read garbage.
5. `planner_manager.cpp:10280-10289` (`BODY_ONLY_VISIBILITY_TOPOLOGY_OVERRIDE`
   audit) can never fire: both `visibility_topology_switched` and the
   `PREFERRED_SAFE_TOPOLOGY_TIE` reason are only produced under
   `los_descriptor_active_for_selection`. The counter is therefore not evidence
   of anything; it was 0 in all runs, including runs where the selection reason
   was `LEXICOGRAPHIC_VISIBILITY_IMPROVEMENT` 130 times (`sim_run_109`).
6. `planner_manager.cpp:7255-7261` builds the LOS observation cone with
   `radius = conflict.radius` and `alpha = asin(radius / distance)`, but for a
   static wall `conflict.radius` is the wall's **diagonal** half-extent
   (`plan_env/include/plan_env/static_los_geometry.h:432`
   `witness->effective_radius = wall.half_size.norm();`) and `distance` is to the
   wall centre. For a long wall the half-space normal is wrong or the plane is
   skipped. `long_cylinder_forest.json` has no `wallData`, so this is dormant.
7. `planner_manager.cpp:10620-10627` (`checkCollision(int drone_id)`) indexes
   `traj_.swarm_traj[drone_id]` and samples `traj.getPos()` with no bounds check
   and no `getPieceNum()` check; the only caller currently maintains both
   invariants (`ego_replan_fsm.cpp:2383-2455`). The same pattern appears at
   `planner_manager.cpp:2179-2183` (`team_reference_.yaws[pp_.drone_id]`) while
   `planner_manager.cpp:5384` does range-check, and at
   `planner_manager.cpp:2778` (`visibility_odom_[pp_.drone_id]`) while
   `:1207`/`:5548` do range-check. `pp_.drone_id` defaults to -1
   (`planner_manager.cpp:580`).
8. `traj_opt/include/optimizer/poly_traj_utils.hpp:499-517`: `locatePieceIdx()`
   returns -1 for a 0-piece trajectory (`idx == N == 0` then `idx--`), so any
   `getPos/getVel/getAcc` on an empty `poly_traj::Trajectory` reads
   `pieces[-1]`. `ego_replan_fsm.cpp:2482-2539` (`polyTraj2ROSMsg`) calls
   `getPos(0)`, `getVel(0)`, `getAcc(0)`, `getPos(duration)` and `getPositions()`
   without a piece check (and `:2530` computes `resize(piece_num - 1)` with
   `piece_num == 0`). All three callers of `publishCurrentTrajectory` are
   currently gated on a committed/retained status, so no reachable path was
   found - but this is the only shared-header wild-read defect present in both
   binaries that crash together, and the build is `Release`
   (`plan_manage/CMakeLists.txt:4`), so assertions are compiled out.

---

## 3. Crash evidence (not localized)

`drone_<N>_ego_planner_node` and `team_target_reachability` both die with
`exit code -11` in 3 of 6 recent runs (107, 108, 109; 103, 105, 106 clean), and
the logs also contain
`terminate called after throwing an instance of 'boost::wrapexcept<boost::lock_error>'`.

What the audit could establish:

- there is no core file and no backtrace (`ulimit -c` is 0, apport captured
  nothing for these PIDs); the ROS node logs for the dead processes are absent;
- `team_target_reachability` has no spinner threads at all
  (`team_target_reachability.cpp:310-316` uses `ros::WallRate` +
  `ros::spinOnce()`), so the joint crash of two processes is not a data race
  inside that node;
- `ego_planner_node` has three threads (`ego_planner_node.cpp:53` `ros::spin()`
  plus the heartbeat and cooperative-viewpoint spinners at
  `ego_replan_fsm.cpp:111-117` and `:131-145`); the only cross-thread members
  (viewpoint fields and the hypothesis bundle) are mutex-protected
  (`ego_replan_fsm.h:130`, locks at `:1647`, `:1670`, `:1679`, `:1696`), so no
  unguarded cross-thread member was found;
- `pthread_mutex_lock: Invalid argument` (EINVAL) is the signature of locking a
  destroyed mutex, i.e. it is consistent with memory corruption or with
  exit-time teardown (`~EGOReplanFSM` stops the spinners at
  `ego_replan_fsm.cpp:12-19`), not with contention.

`CRASH_ROOT_CAUSE: NOT LOCALIZED`. The candidate list is section 2.3/2.7 plus
the two un-guarded fixed-size-array indices in 2.6. A reproducible stack needs
either an AddressSanitizer build or `ulimit -c unlimited` before the next run.

---

## 4. Verification gaps in this round

- `CATKIN_ENABLE_TESTING:BOOL=OFF` (`ros_ws/build/ego_planner/CMakeCache.txt:63`),
  so the last builds did not rebuild any test target.
  `plan_manage/test/local_sfc_boundary_scan_contract_test.cpp`,
  `trajectory_lifecycle_wiring_test.py` and `recovery_nonblocking_wiring_test.py`
  are not referenced by `CMakeLists.txt` at all.
- Of the six test binaries on disk, only two are both current and runnable
  standalone: `trajectory_lifecycle_contract_test` (10/10 PASS) and
  `local_execution_contract_test` (all PASS). `feedback53_regression_fixture_test`
  dies with
  `undefined symbol: ...validateExecutionTrajectory...TeamTrajectorySolution...`
  because it was built against an older header; `fresh_moving_initializer_contract_test`
  and `visibility_topology_production_test` block on a ROS master because they
  link `ego_planner_manager`.
- No test covers the code added in the last rounds: the 0.5 m progress guard,
  the publication-identity guard, the scheduled-future queue, or joint
  acceptance.

## 5. What is confirmed correct

- `active_execution_touch_goal_` is declared (`planner_manager.h:573`), set on
  both install paths (`planner_manager.cpp:2191` in `setLocalTrajFromOpt`, used
  by the local commit at `:3714` and the team adoption at `:3835`; and
  `:3912`), and read by `lifecycleSuccessorDue()` at `:1938-1939`. No install
  path that skips it was found.
- The raw LOS witness is created before the optimizer diagnostic gate
  (`planner_manager.cpp:6028-6088` vs `:6105-6113`) and is not gated by solver
  success (`:5814`/`:5823`).
- `reason_mask` bit semantics are consistent: LOS bits are written only in LOS
  evidence branches (`:6214`, `:6388`, `:6456`), BODY bits only in body branches
  (`:6263`, `:6318`, `:6360`); a BODY-only event cannot acquire a LOS bit.
  `[conflict-descriptor]` telemetry in `sim_run_109` shows 75 x `reason_mask=1`,
  1707 x `=2`, 119 x `=3`.
- The finalization rule that restores an LOS plane only if the final trajectory
  still satisfies it holds (`:9613-9664`), backed by the optimizer's Local-SFC
  satisfaction check and the commit-time re-validation.
- Raw LOS sampling uses the same absolute world time for the UAV and the
  obstacles (`:6034`, `:6064`); no `ros::Time::now()` inside that region.
- Static LOS detection is live at runtime (`[raw-los-truth] ... static_blocked=1`
  in `sim_run_109`) and `reason=LOS` appeared 128 times in `sim_run_109`.
- The publication-identity guard suppresses only the identical
  `trajectory_id`+`start_time` pair (`ego_replan_fsm.cpp:713-728`) and
  `TRAJECTORY_ID` is only advanced in `setLocalTraj`, so the guard cannot swallow
  a new successor.
- `traj_server`'s heartbeat and command callbacks are on separate callback
  queues but serialize on `trajectory_state_mutex_`; `executionHandoffGate()`
  has `>0.015 s` activation margin and PVA tolerances (0.02 m / 0.05 m/s /
  0.10 m/s^2) unchanged.

## 6. Suggested order of work

1. Separate "successor pending" from "replan success" in
   `callReboundReplan()` and stop the same-tick `REPLAN_TRAJ` re-entry (1.1).
2. Re-derive the rolling progress guard in time units and clamp it by the
   remaining distance to the target / map bounds (1.2).
3. Unify the target epoch: one rebased target state used by the raw LOS
   sampler, the optimizer cost and the selection visibility evaluator (1.3),
   and replace the three `ros::Time::now()` risk epochs (1.4).
4. Make the joint attempt deadline account for the actual round trip, or make
   it fully asynchronous, so it cannot burn the local rolling deadline (1.6).
5. Start post-deadline recovery while activation can still fit inside the
   validated coverage window (1.5).
6. Give the target coordinator a static pre-pass / reject authority and call the
   route validator from `run_constvel_gradient_rviz.sh` (1.7).
7. Add the cheap guards from section 2 and re-run with ASan or core dumps to
   localize the SIGSEGV.

CODE_MODIFIED_THIS_TURN: NO
RRCT_ACCESSED: NO
RRCT_CHANGED: NO
