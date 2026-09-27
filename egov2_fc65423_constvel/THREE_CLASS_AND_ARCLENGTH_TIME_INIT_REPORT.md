# Three-Class Safety and Arc-Length Repair Time Initialization Report

Date: 2026-09-02  
Project: `/home/bob/ALP/egov2_fc65423_constvel`  
Scope: only dynamic safety classification/selection and A* repaired-guide time initialization.

## 1. Worktree protection

The worktree was already heavily dirty before this task. No reset, checkout, clean,
commit, formatting pass, source deletion, parameter edit, scene edit, MINCO/SCP/QP
change, or RRCT access was performed.

Files intentionally changed for this task:

- `planner_manager.h`: adds `CandidateSafetyClass` and classification fields.
- `planner_manager.cpp`: adds three-class classification/selection and repaired-guide timing.
- `three_class_arclength_unit_test.py`: read-only/offline contract test.

Generated validation artifacts:

- `three_class_arclength_validation_20260902/smoke/`
- `three_class_arclength_validation_20260902/full_runs/alp_1/`
- `three_class_arclength_validation_20260902/full_runs/alp_2/`

## 2. Three-class implementation

### Classification contract

`CandidateSafetyClass` is defined at
`planner_manager.h:48-66` with `ABSOLUTE_SAFE`, `IMPROVED_ONLY`, and `INVALID`.

Final candidate classification is implemented in `reboundReplan()` at
`planner_manager.cpp:1409-1442`:

- The absolute threshold is the runtime optimizer value returned by
  `getMovingObjClearance()` (1.1 m in these runs).
- Each successfully generated final trajectory is rechecked with the existing
  velocity/acceleration/jerk checker and static checker.
- A failed solve, invalid dynamic risk, or failed existing check is `INVALID`.
- A valid candidate at or above 1.1 m center distance is `ABSOLUTE_SAFE`.
- A SIDE candidate below 1.1 m but at least nominal + 0.10 m is
  `IMPROVED_ONLY`.
- NOMINAL below 1.1 m is `INVALID`; it cannot satisfy improvement against itself.

Nominal risk is recomputed and classified at `planner_manager.cpp:1451-1459`.
Both SIDE results are classified after their final risk recomputation at
`planner_manager.cpp:3898-3929`.

The remaining previous-active trajectory is sliced, checked for dynamics and
static safety, and now requires a valid dynamic-risk result and clearance at or
above the same absolute threshold at `planner_manager.cpp:606-710`, especially
`670-696`.

### Selection contract

Selection ranks new candidates by class before clearance at
`planner_manager.cpp:3898-3929`:

1. `ABSOLUTE_SAFE` new candidate;
2. revalidated previous `ABSOLUTE_SAFE`;
3. `IMPROVED_ONLY` SIDE candidate;
4. existing NOMINAL emergency fallback, logged as `INVALID` / `UNSAFE_FALLBACK`.

PLUS/MINUS bilateral solving, semantic-v2, and same-side hysteresis remain in
place. Hysteresis is applied only when the two SIDE candidates have the same
safety class (`planner_manager.cpp:3940-3975`), so it cannot demote an absolute
candidate below an improved-only candidate.

The previous trajectory protection is enforced at
`planner_manager.cpp:4093-4130` and again for non-triggered cycles at
`planner_manager.cpp:4139-4178`: an `IMPROVED_ONLY` or `INVALID` result cannot
overwrite a currently revalidated previous `ABSOLUTE_SAFE` trajectory.

No no-trajectory, E-stop, FAILED, or hard-block branch was added. The invalid
NOMINAL fallback continues to commit when no absolute, previous absolute, or
improved-only result exists.

### Logging

`[candidate-safety-class]` records type, final dynamic distance, nominal
distance, absolute threshold, class, and selection reason at
`planner_manager.cpp:4030-4055` and `4141-4178`.

Across the two full runs, the logs contain:

| Run | Parseable class records | ABSOLUTE_SAFE | IMPROVED_ONLY | INVALID |
|---|---:|---:|---:|---:|
| 1 | 2050 | 1625 | 10 | 415 |
| 2 | 2030 | 1716 | 4 | 310 |

Final per-cycle outcomes:

| Run | New ABSOLUTE_SAFE | Previous ABSOLUTE_SAFE retained | IMPROVED_ONLY selected | INVALID/UNSAFE_FALLBACK |
|---|---:|---:|---:|---:|
| 1 | 800 | 66 | 5 | 51 |
| 2 | 852 | 50 | 3 | 40 |

The 116 retained-previous events demonstrate that unsafe/relative-only new
results did not overwrite a revalidated absolute trajectory. The 91 invalid
NOMINAL fallback cycles demonstrate that the planner remained nonblocking.

## 3. A* repair to MINCO time initialization

The old index-ratio assignment inside the collision window was removed. No
`collision_start + ratio * (collision_end - collision_start)` assignment remains.

The repaired-only timing is implemented at `planner_manager.cpp:2616-2748`:

- Segment length is `||p[i+1]-p[i]||` (`2663-2669`).
- Internal turn angle and the requested curvature estimate are computed at
  `2670-2691`.
- Vertex speed is capped by `0.8 * max_vel` and
  `sqrt(0.8 * max_acc / (curvature + epsilon))` at `2650-2660` and `2692-2700`.
- Segment speed is the smaller adjacent vertex speed and segment duration is
  length/speed with a 0.10 m/s numerical floor and 1e-3 s positive guard at
  `2703-2726`.
- Repaired guide times are accumulated monotonically at `2728-2737`.
- The duration delta relative to the old collision window is applied to every
  downstream guide junction at `2738-2747`; expanded repair time is not
  compressed back into the old window.
- Existing head/tail PVA and `MinJerkOpt::reset/generate` are preserved at
  `2770-2781`. Existing free-time P/T SCP and later uniform retiming are unchanged.

`[repair-time-init]` logs the requested path, piece-time, curvature, and initial
dynamics fields at `planner_manager.cpp:2783-2798`.

## 4. Offline and pathological-sample verification

`python3 three_class_arclength_unit_test.py` passed. It checks all three safety
classes, positive finite durations, longer-segment allocation, curvature speed
reduction, and point-count invariance on the same straight 5.848 m path.

Synthetic known-window check:

- old window: 0.667 s;
- new arc-length/curvature duration: 2.741913 s;
- min/max piece time: 0.083333 / 1.520000 s.

Real full-run same-length sample (`alp_1/alp.launcher.log`, timestamp
`1788334763.251788007`):

- path length: 5.848042 m;
- pieces: 26;
- old window: 0.833333 s;
- new repaired duration: 3.795427 s;
- min/max piece time: 0.041667 / 0.266676 s;
- max curvature: 6.340507 1/m;
- initial max velocity: 7.778989 m/s;
- initial max acceleration: 35.518588 m/s^2;
- initial max jerk: 457.550403 m/s^3.

Compared with the historical same-path pathological observation
(`v≈23.376`, `a≈694.553`, `jerk≈54710`), velocity fell about 66.7%, acceleration
about 94.9%, and jerk about 99.2%. The initial polynomial is not always already
dynamics-feasible; this change removes the point-index time collapse and leaves
the existing retiming/SCP stages to complete feasibility handling.

Full-run repair ranges:

| Run | Repair count | Path length range | Old-window range | New-duration range | Max initial v | Max initial a | Max initial jerk |
|---|---:|---:|---:|---:|---:|---:|---:|
| 1 | 17 | 1.539-5.870 m | 0.333-3.587 s | 0.967-3.795 s | 9.121 | 104.059 | 3057.671 |
| 2 | 14 | 2.066-5.961 m | 0.417-2.507 s | 1.678-3.875 s | 9.268 | 95.190 | 2270.762 |

## 5. Build and smoke

The workspace was previously created with `catkin build`; an initial
`catkin_make` invocation refused to mix build tools before compilation began.
No build directory was removed. The correct command then passed:

`catkin build path_searching traj_opt ego_planner --no-status -j2`

Result: all requested packages and their required dependencies succeeded.
Warnings were pre-existing CMake policy/optional PCL feature warnings.

The 25 s smoke run produced 213 committed selections and 473 classification
records. It exercised `ABSOLUTE_SAFE`, `IMPROVED_ONLY`, `INVALID`, previous-safe
retention, and unsafe fallback without a planner crash or no-trajectory cascade.
The timeout return code was the expected test cutoff. Boost mutex exceptions
appeared only during forced ROS shutdown, as in prior runs.

## 6. Two complete runs

Scene: `long_cylinder_forest_dynamic_gates.json`  
Configuration: ALP moving cost ON, risk candidates ON, hard-corridor SCP ON,
target-facing yaw ON, max jerk 22, RViz OFF. Both runs stopped immediately after
the target-stop condition was stable.

| Metric | Run 1 | Run 2 |
|---|---:|---:|
| target-stop detected | YES | YES |
| wall runtime | 85.16 s | 85.28 s |
| cutoff trajectory time | 80.788 s | 80.916 s |
| dynamic collision episodes | 1 | 0 |
| dynamic collision samples | 6 | 0 |
| dynamic unsafe samples (`<0.5 m`) | 61 | 15 |
| minimum dynamic surface clearance | 0.000 m | 0.3095 m |
| static collision episodes | 0 | 0 |
| minimum static clearance | 0.1996 m | 0.1807 m |
| mean path length | 90.193 m | 80.492 m |
| tracking mean | 1.6477 m | 1.6637 m |
| tracking p95 | 2.6606 m | 3.2330 m |
| planning latency median | 0.411 ms | 0.405 ms |
| planning latency p95 | 1.230 ms | 1.561 ms |
| planning latency max | 19.779 ms | 16.693 ms |
| all-three visibility | 86.53% | 81.06% |
| at-least-one visibility | 93.26% | 93.47% |

### A1 results

- Run 1: no collision; UAV2/A1 minimum surface clearance 0.2624 m and 32 unsafe samples.
- Run 2: no collision and no A1 unsafe samples; minimum across UAVs 0.8090 m.

### C2 results

- Run 1: one UAV1/C2 collision episode, 6 collision samples, 29 unsafe samples,
  minimum surface clearance 0.0 m; episode approximately 62.342-62.509 s.
- Run 2: no collision and no C2 unsafe samples; minimum across UAVs 0.9633 m.

The remaining Run-1 C2 collision means this minimal change is not a complete
dynamic-collision solution. It does not invalidate the requested contracts:
relative-only candidates are no longer labeled safe, previous absolute-safe
trajectories are protected while revalidation succeeds, and repaired timing no
longer collapses with A* point count.

## 7. SCP/QP/trust statistics

Unique final SIDE candidate status counts from `[candidate-final-status]`:

- Run 1 (63 final statuses): 38 `SCP_FINAL_OK`, 11
  `QP_MAX_ITER_EXHAUSTED`, 3 `P_TRUST_TOO_SMALL`, 6
  `DYNAMICS_MODEL_MISMATCH_TRUST_EXHAUSTED`, 2
  `DYNAMICS_TRUST_RETRY_EXHAUSTED`, plus 3 other terminal statuses.
- Run 2 (58 final statuses): 32 `SCP_FINAL_OK`, 4
  `QP_MAX_ITER_EXHAUSTED`, 6 `P_TRUST_TOO_SMALL`, 2
  `DYNAMICS_MODEL_MISMATCH_TRUST_EXHAUSTED`, 5
  `DYNAMICS_TRUST_RETRY_EXHAUSTED`, 3
  `SPATIAL_TRUST_RETRY_EXHAUSTED`, plus 6 other terminal statuses.

No MINCO, SCP, trust, OSQP, jerk limit, A*, SFC, SIDE offset, parameter, or scene
logic was changed in response to these counts.

## 8. Final fields

THREE_CLASS_CLASSIFICATION_IMPLEMENTED: YES

RELATIVE_IMPROVEMENT_NO_LONGER_EQUALS_SAFE: YES

ABSOLUTE_SAFE_PROTECTED_FROM_UNSAFE_OVERWRITE: YES

EMERGENCY_FALLBACK_REMAINS_NONBLOCKING: YES

ARCLENGTH_TIME_ALLOCATION_IMPLEMENTED: YES

CURVATURE_LIMITED_REFERENCE_SPEED: YES

OLD_INDEX_BASED_TIME_ASSIGNMENT_REMOVED: YES

PATHOLOGICAL_INITIAL_JERK_REDUCED: YES

A1_RESULTS: Run 1 no collision, 32 unsafe samples, min 0.2624 m; Run 2 no collision/unsafe, min 0.8090 m.

C2_RESULTS: Run 1 one collision episode, 6 collision samples, 29 unsafe samples, min 0.0 m; Run 2 no collision/unsafe, min 0.9633 m.

BUILD: PASS

RUNTIME: PASS
