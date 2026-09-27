# Recovery production candidate fixture root-cause and closure

Date: 2026-09-12  
Project: `/home/bob/ALP/egov2_fc65423_constvel`  
Scope: `recovery_probe_production_test` production-candidate chain only. No simulation was run. `/home/bob/RRCT` was not accessed.

## 1. Result

The failure was a test-fixture initialization defect, not a production planner defect. The synthetic voxel buffer had been marked known-free, but three independent production launch prerequisites were still missing:

1. `GridMap::getOccupancy` and `getInflateOccupancy` apply `virtual_ceil_height` before reading the voxel buffer. The fixture inherited the default `-0.1 m`, so every tested point at `z=1.2 m` was reported occupied even after `setFree()`.
2. `PolyTrajOptimizer` owns a separate `optimization/max_vel`; the fixture only set `manager/max_vel`. The optimizer default remained `-1`, so `samplePointsToCheck()` returned false before the NOMINAL precheck could produce an executable candidate.
3. The optimizer cost and clearance parameters normally supplied by the launch file also remained at their `-1` sentinels. Once point sampling was enabled, those invalid scales caused `plan_success=0` and pathological multi-thousand-second timing rather than a normal MINCO result.

A separate dynamic-obstacle subcase set `prediction/obj_num=1` but omitted `prediction/use_time_aware_moving_obj_cost=true`; consequently no `ObjPredictor` was constructed and its published update had no production consumer. This was also a fixture defect.

The continuous-motion helper retained an older assertion that a failed NOMINAL solve must preserve the exact old trajectory ID. Current production may instead commit a fully safety-validated velocity continuation. The contract now checks the actual invariant—stationary input is not selected and moving execution is preserved—and isolates subsequent independent prefix tests from the newly committed continuation.

```text
RECOVERY_ROOT_CAUSE_PRIMARY:
GridMap::getInflateOccupancy / GridMap::getOccupancy virtual-ceiling gate;
fixture virtual_ceil_height=-0.1, candidate z=1.2, therefore raw_occ=1 and inflated_occ=1

FIRST_FAILURE_STAGE_BEFORE_FIX: STAGE_06_NOMINAL_PRECHECK
FIRST_FAILURE_FUNCTION_BEFORE_FIX: GridMap::getInflateOccupancy(Eigen::Vector3d)
FIRST_FAILURE_CONDITION_BEFORE_FIX: virtual_ceil_height > -0.5 && pos.z >= virtual_ceil_height
FIRST_FAILURE_RUNTIME_VALUES: pos=(2.0,0.0,1.2), voxel=(70,50,12), virtual_ceil_height=-0.1, raw_occ=1, inflated_occ=1
WHY_KNOWN_FREE_MAP_WAS_NOT_ENOUGH: setFree() changes occupancy buffers only; virtual ceiling is an independent hard gate evaluated first
FIXTURE_BUG_OR_PRODUCTION_BUG: FIXTURE
PRODUCTION_SOURCE_CHANGED: NO
```

## 2. Sequential failure trace

The trace was continued after each fixture correction rather than stopping at the first symptom.

| Order | Last success | First failure | Function / condition | Runtime evidence | Classification / correction |
|---:|---|---|---|---|---|
| 1 | fresh moving initializer | NOMINAL static precheck | `GridMap::getInflateOccupancy`: virtual ceiling test | `z=1.2 >= -0.1`, raw/inflated occupancy both 1 | Fixture: set ceiling to the 4.0 m synthetic-map upper boundary. |
| 2 | fresh initializer and map check | NOMINAL point construction | `PolyTrajOptimizer::samplePointsToCheck`: `max_velocity <= 0` | manager limit 3.0, optimizer limit inherited `-1` | Fixture: provide optimizer-owned v/a/j limits. |
| 3 | point construction | NOMINAL optimization | launch-owned objective/clearance sentinels remained `-1` | `plan_success=0`; candidate duration grew to about 3809 s | Fixture: load the ordinary production launch weights/clearances. |
| 4 | dynamic path initially valid | dynamic update invalidation | no predictor object because enable flag was false | `obj_num=1`, predictor absent | Fixture: enable time-aware predictor for this subcase; it now reports `DYNAMIC_FAIL`. |
| 5 | moving continuation committed | legacy exact-ID assertion | contract required ID 71 even when a safe moving successor was committed | old ID 71, validated successor ID 72, max velocity 0.6 m/s | Obsolete fixture semantics: assert continuous motion, not exact identity; reset the controlled prefix before independent tests. |

No production hard gate was relaxed. Unknown remains non-free; static, dynamic, swarm, dynamics, local-SFC and P/V/A handoff checks remain enabled.

## 3. Passing fixture versus recovery fixture differential

`fresh_moving_initializer_contract_test` proves the standalone seed/retiming contract. A full `probeRecoveryTarget` additionally initializes map and optimizer consumers normally populated by ROS launch.

| Field | Passing initializer test | Recovery fixture before fix | Relevant |
|---|---|---|---|
| Manager v/a/j | explicitly initialized | initialized | No |
| Optimizer v/a/j | not needed by standalone helper | unset (`-1`) | Yes: point sampling rejected. |
| Optimizer weights/clearances | not needed | unset (`-1`) | Yes: invalid optimization scale/timing. |
| Known-free voxel buffer | not needed | explicitly populated | Necessary but insufficient. |
| Virtual ceiling | not queried | default `-0.1` | Yes: overrode known-free voxels. |
| Target/goal scale | 4 m fresh-moving case | 0.5 m per-UAV acquisition | Valid |
| Current/future PVA | explicit | active trajectory ID 11 sampled at common future epoch | Valid |
| Dynamic predictor | not used | object count set but enable gate absent in dynamic subcase | Yes for that negative update test. |
| Team worker inputs | not used | three source trajectories and current odom/stamps | Valid after local candidates exist. |

## 4. Production-runtime prerequisite audit

| Item | Production runtime has it | Fixed fixture has it | Required on exercised path |
|---|---|---|---|
| Target/object state | YES | YES, `(0,0,1.2)` | YES |
| Fresh target stamp | YES | YES, current ROS time | YES for worker admission |
| Target velocity | YES | YES, zero is valid for this static contract; nonzero continuation is covered separately | YES |
| Odom/current state stamps | YES | YES | YES |
| Consistent ROS planning time | YES | YES | YES |
| Future activation | YES | YES, about `now+0.1 s` | YES |
| Predictor object | conditional | YES in dynamic subcase | Only for dynamic obstacle validation |
| Predictor data | conditional | YES, two world-time samples published and consumed | Only for dynamic subcase |
| Known/free map | YES | YES, bounded `[-4,4] x [-4,4] x [0.2,2.5]` | YES |
| Virtual ceiling | launch supplied | YES, 4.0 m | YES |
| Inflation policy | launch supplied | YES, 0.0 for exact synthetic occupancy | YES |
| Map update/version | YES for live sensor map | No live stream; direct deterministic buffer is ready | NO for this direct synthetic map path |
| Optimizer params | launch supplied | YES | YES |
| Drone IDs / swarm size | YES | YES, IDs 0/1/2 and three peers | YES |
| Peer trajectory cache | YES | YES, three trajectory records | YES for swarm gate |
| Peer odom fallback | available | not needed | NO |
| Candidate capture | YES | YES, probe-only deferred commit | YES |
| Generation/epoch | YES | YES, generation 1 and common activation | YES |
| Worker state/publisher harness | YES | YES | YES |
| Target valid | YES | YES | YES |
| Guide valid | optional | tested present, absent and invalid | NO for local moving continuation |
| Team Reference | optional enhancement | tested present and absent | NO |
| Trajectory start versus activation | managed in production | common reanchor epoch | YES |

## 5. Fixed stage trace

Common primary case context:

```text
CASE_NAME: THREE_UAV_PRODUCTION_NOMINAL_CAPTURE_AND_WORKER_ADMISSION
drone_ids: 0,1,2
swarm_size: 3
current_positions: (1.5,0,1.2), (-0.9,2,1.2), (-1.5,-1.8,1.2)
current_velocity/current_acceleration: zero at fixture heads
target: (0,0,1.2), velocity=(0,0,0), fresh ROS stamp at worker entry
per-UAV endpoints: (2,0,1.2), (-0.4,2,1.2), (-1,-1.8,1.2)
active_source_identity: trajectory_id=11, generation=1
future_activation: common epoch approximately now+0.1 s
Guide: optional; both guide and no-guide semantics covered
Team Reference: optional; missing-reference local continuation PASS
map: 10 x 10 x 4 m, 0.1 m resolution, bounded known-free volume, ceiling 4.0 m
```

| Stage | Result | Evidence |
|---|---|---|
| STAGE_00_FIXTURE_INIT | PASS | All manager/map/optimizer/swarm prerequisites initialized. |
| STAGE_01_TARGET_VALID | PASS | Current target and worker timestamp valid. |
| STAGE_02_REFERENCE_VALID | PASS | Direct target+offset is valid; Guide/Team Reference are not hard dependencies. |
| STAGE_03_ACTIVATION_STATE_BUILT | PASS | Active ID 11 sampled/reanchored at one future epoch. |
| STAGE_04_FRESH_INITIALIZER | PASS | Each UAV: 2 pieces, 1.201950 s; max v/a/j = 0.779983 / 1.998189 / 17.276752, within 3/6/20. |
| STAGE_05_NOMINAL_MINCO_BUILT | PASS | Three `plan_success=1` results. |
| STAGE_06_NOMINAL_PRECHECK | PASS | Known/free, point sampling and dynamics checks pass. |
| STAGE_07_SIDE_DISPATCH | NOT REQUIRED | NOMINAL already succeeded; SIDE path remains available and was not altered. |
| STAGE_08_SIDE_SEED | NOT REQUIRED | Same reason. |
| STAGE_09_ASTAR | NOT REQUIRED | Same reason; barrier negative case still confirms full-path static rejection. |
| STAGE_10_LOCAL_SFC | PASS / EMPTY ALLOWED | Fixture has hard corridor-SCP disabled; local-SFC validation reports 1 for every accepted path. |
| STAGE_11_MINCO_SCP | PASS | Real MINCO candidate produced; no production optimizer shortcut. |
| STAGE_12_CANDIDATE_CAPTURE | PASS | Each UAV: generation 1, hypothesis 0, candidate ID 1, source NOMINAL, deferred commit. |
| STAGE_13_FINAL_PREFLIGHT | PASS | Reanchored duration 1.938688 s, length 0.5 m; static/dynamic/swarm/dynamics/local-SFC/handoff all equal 1. |
| STAGE_14_WORKER_ADMISSION | PASS | `TARGET_REACHABLE`, plan ID 1, reachability generation 1. |
| STAGE_15_BUNDLE_PUBLISHED | PASS | `GUIDE_CREATED` and `TARGET_ACTIVATED`; three independent guide/evidence entries present in active worker bundle. |

```text
LAST_SUCCESS_STAGE: STAGE_15_BUNDLE_PUBLISHED
FIRST_FAILURE_STAGE_AFTER_FIX: NONE
FIRST_FAILURE_REASON_AFTER_FIX: NONE
```

### Final admission values

| Admission condition | Value | Expected | Result |
|---|---:|---:|---|
| Three candidate successes | 3 | 3 | PASS |
| Nonempty trajectory | 2-piece MINCO per UAV before reanchor | nonempty | PASS |
| Reanchored duration | 1.938688 s | >= 0.3 s | PASS |
| Activation lead | about 0.1 s | future | PASS |
| Source identity | ID 11 / generation 1 per UAV | coherent | PASS |
| Target/odom freshness | current ROS time | fresh | PASS |
| Static | 1,1,1 | 1 | PASS |
| Dynamic | 1,1,1 | 1 | PASS |
| Swarm | 1,1,1 | 1 | PASS |
| Dynamics | 1,1,1 | 1 | PASS |
| Local SFC | 1,1,1 | 1 | PASS |
| Handoff | 1,1,1 | 1 | PASS |

`FIRST_ADMISSION_REJECT_CONDITION: NONE after fixture correction`.

## 6. Modified files

- `ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/test/recovery_probe_production_test.cpp`
  - supplies the missing production launch context: optimizer limits, weights and clearances, virtual ceiling, and the dynamic predictor enable flag;
  - preserves strict known-free/static/dynamic/swarm behavior.
- `ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/test/continuous_motion_contract.h`
  - aligns old exact-ID recovery assertions with current validated moving-continuation semantics;
  - resets controlled state before independent suffix contracts.

The pre-existing test-only `GridMap::setFree` helper from feedback 46 was used but not changed in this continuation. No planner, optimizer, SIDE, visibility, SFC, worker or other production source was modified.

## 7. Build and tests

Required build command:

```text
catkin build traj_utils traj_opt ego_planner multi_uav_formation -j2 --no-status --workspace ros_ws
```

Result: all requested packages and dependencies succeeded, no package failure.

`recovery_probe_production_test` now exits 0 with every assertion passing, including:

- production NOMINAL moving candidate capture for all three UAVs;
- final production team preflight;
- real dynamic predictor update rejection;
- initial and same-target worker admission;
- changed-map negative cases;
- continuous moving execution contract;
- continuous Team Reference and missing-reference continuation contracts;
- synchronous crossing and static barrier negative cases.

Additional executable regression set: 15/15 passed:

- fresh moving initializer;
- trajectory lifecycle and local execution;
- time-only feasibility and swarm temporal contracts;
- Elastic visibility/gradient contract;
- cooperative viewpoint, topology coordinator, team visibility optimizer, team solution commit;
- static LOS wall, adaptive viewpoint, encirclement geometry, persistent recovery target and multiview contracts.

Gradient evidence:

```text
production local guide position/time finite-difference: PASS
team-reference production FD max: 1.28832e-10
elastic visibility contract: PASS
```

## 8. Required final fields

```text
FRESH_INITIALIZER_PASS: YES
NOMINAL_MINCO_BUILT: YES
SIDE_DISPATCH_WORKS: NOT EXERCISED (not required by the successful NOMINAL primary case; production path unchanged)
ASTAR_PATH_AVAILABLE_IF_REQUIRED: NOT EXERCISED (not required by the successful NOMINAL primary case; production path unchanged)
LOCAL_SFC_BUILT: NOT_REQUIRED; LOCAL_SFC_GATE_PASS=YES
FINAL_PREFLIGHT_PASS: YES
WORKER_ADMISSION_PASS: YES
CANDIDATE_BUNDLE_FORMED: YES

BUILD_PASS: YES
UNIT_TEST_PASS: YES
GRADIENT_TEST_PASS: YES
CONTRACT_TEST_PASS: YES

RRCT_ACCESSED: NO
RRCT_CHANGED: NO
SIMULATION_RUN: NO
SIMULATION_LEFT_RUNNING: NO
```
