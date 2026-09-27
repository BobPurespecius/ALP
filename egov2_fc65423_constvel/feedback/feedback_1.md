# Run1/C2 Post-Three-Class Source-Level Autopsy

Date: 2026-09-02  
Scope: read-only audit of `three_class_arclength_validation_20260902/full_runs/alp_1/`  
Runtime planner source: `ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/`  

No code, parameter, scene, or experiment was changed or rerun. The sibling `ros_ws/src/ego-planner` tree is not the runtime source: `ros_ws/build/ego_planner/CMakeCache.txt` resolves `ego_planner_SOURCE_DIR` to the EGO-Planner-v2 tracking workspace above.

## 1. Executive conclusion

Run1/C2 is a `TYPE_E: SAFE COMMAND AT THE COLLISION INSTANT, ODOM COLLIDES` event, preceded by `TYPE_A` recovery failure.

The collision chain is:

1. C2 risk was detected about 4.68 s before physical contact, so this was not a last-moment 2 s-horizon discovery.
2. The three-class persistence logic retained trajectory 219 while it remained ABSOLUTE_SAFE.
3. At 1788334762.800350, trajectory 219 expired. In that decisive cycle, NOMINAL was INVALID (0.933 m), MINUS was not an improvement, and the correct physical-RIGHT PLUS A* path was static-free and semantic-v2 PASS but was rejected before MINCO because Local-SFC produced no planes.
4. With no ABSOLUTE_SAFE replacement, the nonblocking policy committed successive INVALID NOMINAL fallbacks 220–231.
5. A repair/SCP attempt started at epoch 1788334763.177687 from head position x=17.664 and occupied about 0.4745 s. During that time the vehicle followed trajectory 228 forward; its command reached x=18.383 by 1788334763.635713.
6. The failed recovery then committed fallback 229 with the old cycle-start state. Fallbacks 230/231 inherited that stale trajectory state. Command x consequently jumped back to 17.679 at 1788334763.745798 while odom was already about x=18.73.
7. The configured 0.5 m tracking-error threshold could not resynchronize planning to odom because the source applies it only when `use_fov_tracking=true`, while this run had `use_fov_tracking=false`.
8. At deepest collision, odom was 1.395 m from the active command. Odom penetrated C2 by 0.0404 m; the command was still 1.085 m outside C2's physical cylinder surface.

Therefore the primary cause is stale-start trajectory activation after long recovery latency, combined with disabled odom resynchronization. The primary upstream recovery-generation cause is the Local-SFC empty-plane hard rejection. Repair→MINCO pathological initialization and QP/trust failures are real secondary contributors, not the complete root cause.

## 2. Collision event reconstructed from raw data

### Metric and obstacle model

C2 is moving obstacle id 3:

- base center `(19.04, 2.72)`, radius `0.28 m`, axis `(0.6, 0.8)`, amplitude `0.9 m`, period `11.5 s`, phase `-33.259905`;
- source: `long_cylinder_forest_dynamic_gates.json:665-679`;
- state formula: `native_egov2_rviz_scene.py:108-126`.

For UAV z inside the cylinder height, the recorder uses

`clearance = max(0, hypot(uav_xy - obstacle_xy) - radius)`.

Source: `native_egov2_rviz_scene.py:626-657`. Thus CSV zero hides penetration. I recomputed signed clearance as

`signed_surface_clearance = hypot(uav_xy - C2_xy) - 0.28`.

This is center-to-cylinder-surface clearance; it does not add a UAV body radius.

### Episode

| event | CSV relative time | reconstructed ROS time | signed surface clearance |
|---|---:|---:|---:|
| collision start | 62.342278 | 1788334763.978870 | -0.021854 m |
| deepest penetration | 62.408668 | 1788334764.045260 | -0.040366 m |
| collision end sample | 62.508722 | 1788334764.145314 | -0.018516 m |

- UAV: CSV `uav_id=1`, planner `drone_id=0`.
- Obstacle: C2, `obstacle_id=3`.
- Six collision samples; observed zero-clearance interval is about 0.1664 s.
- At the minimum: UAV `(19.312233, 2.743442, 1.433112)`; C2 center `(19.224966, 2.966622)`.
- Relative XY speed at the minimum was about 1.425 m/s.

Evidence: `alp_trajectory.csv`, rows with times 62.342278–62.508722. Absolute ROS time is reconstructed rather than directly recorded: command/odom position alignment gives tracking start `1788334701.636592` (trimmed spatial RMSE about 0.025 m), and fitting the deterministic C2 state to recorded clearances gives motion-start offset 3.57645 s. Treat absolute times as approximately ±0.03 s; log-to-log intervals remain exact.

## 3. Five-second trajectory provenance

Key cycles only; repeated equivalent cycles are grouped.

| epoch / finish | active/previous | candidates and result | selection / lifecycle |
|---|---|---|---|
| 4758.486 / 4758.488 | previous 211 ABS 1.942 | NOMINAL ABS 1.864 | id212 replaces a safer ABS trajectory |
| 4759.047 / 4759.048 | previous ABS 1.592 | NOMINAL ABS 1.481 | id214; another same-class margin decrease |
| 4759.299 / 4759.456 | previous id214 ABS 1.309 | NOMINAL ABS 1.229; PLUS repair reaches MINCO/SCP but fails model/trust; MINUS fails frontend | id215 committed/activated |
| 4759.582 | previous ABS 1.157 | NOMINAL ABS 1.336 | id216 committed |
| 4759.874 / 4760.085 | previous id216 ABS 1.153 | NOMINAL INVALID 0.685; both SIDE fail | previous id216 retained |
| 4760.137 / 4760.177 | previous id216 invalidated at 0.974 | NOMINAL ABS 1.229; SIDE fail | id217 committed |
| 4760.418 / 4760.723 | previous id217 now 1.084, INVALID | NOMINAL INVALID 0.672; SIDE fail | id218 unsafe fallback |
| 4760.734 | previous id218 INVALID 0.665 | NOMINAL ABS 1.873 | id219 committed; start 4760.734607, duration 2.061875 |
| 4760.990–4762.779 | id219 | repeated NOMINAL/SIDE failures; previous revalidates around 1.87–2.09 | id219 repeatedly retained and republished |
| 4762.284 recovery | id219 retained | PLUS path 5.870 m; QP max-iter; final diagnostic dynamic distance 1.071 (<1.1) | no ABS replacement |
| 4762.800 decisive | id219 expires | NOMINAL INVALID 0.933; PLUS correct A*/semantic path rejected by Local-SFC handoff; MINUS 0.844 | id220 INVALID `UNSAFE_FALLBACK` committed/activated |
| 4762.823–4762.917 | ids220–224, all INVALID | both SIDE repeatedly fail before a safe final candidate | ids221–225 successively supersede |
| 4762.928–4763.155 | ids225–227 INVALID | NOMINAL 0.467→0.462→0.433; PLUS correct A* paths commonly rejected at handoff | ids226–228 committed/activated |
| 4763.177 / 4763.652 | id228; plan head x=17.664 | PLUS A* accepted, repair init v/a/j=7.78/35.52/457.55, QP max-iter; final distance 0.519 | id229 INVALID 0.331 committed after 0.4745 s |
| 4763.664 / 4763.721 | id229 INVALID | both SIDE preinit fail; NOMINAL 0.302 | id230 activated 4763.722325 |
| 4763.732 / 4763.784 | id230 INVALID 0.311 | both SIDE preinit fail; NOMINAL 0.258 | id231 activated 4763.784264 |
| 4763.979–4764.145 | id231 INVALID | collision occurs while next SIDE solve is still running | active command remains id231 |
| 4763.931 / 4764.265 | id231 | late PLUS repair has pathological init and fails P-trust | id232 only commits after collision ended |

The last ABSOLUTE_SAFE trajectory was id219, NOMINAL, generation 219, start `1788334760.734607458`, duration `2.061875`, generation-time distance `1.872659 m`. It expired at `1788334762.796482`, approximately 1.182 s before collision start. Remaining duration at collision was zero.

## 4. Decisive-cycle frontend failure

At `1788334762.799962`:

- NOMINAL risk: 0.933223 m, INVALID.
- PLUS alpha guide: 0.9332→1.3285 m, so the physical-RIGHT side was dynamically useful.
- PLUS A*: SUCCESS, 13 raw points, 9 simplified points, raw static bad=0, side bad=0.
- semantic-v2: closest-side PASS, weighted correct-side ratio 1.0, no fallback.
- nevertheless final repair log says `side_semantic_valid=0`, `ASTAR_REPAIR_REJECT`, followed by `NO_STATIC_FEASIBLE_SIDE`.

The source explains the contradiction:

- `planner_manager.cpp:2527-2534` initializes `repaired_side_valid` from the valid raw path.
- `planner_manager.cpp:2555-2559` only creates a Local-SFC plane when an obstacle-side probe is occupied.
- `planner_manager.cpp:2597-2601` declares an empty plane set invalid and clears `repaired_side_valid`.
- `planner_manager.cpp:2968-2978` requires both `repaired_side_valid` and `local_sfc_build_valid` for A* handoff.
- `planner_manager.cpp:3089-3092` then returns `NO_STATIC_FEASIBLE_SIDE` before MINCO/SCP.

CLAIM: a correct, static-free, semantic-v2-valid PLUS A* recovery was discarded solely because sparse Local-SFC construction yielded no plane.

EVIDENCE: launcher timestamps 1788334762.800401–1788334762.800696 and source above.

COUNTER-EVIDENCE: later cycles sometimes produced 2–3 Local-SFC planes and reached SCP; therefore this is not the only recovery failure mode.

CONFIDENCE: high. This is the primary upstream reason no ABSOLUTE_SAFE candidate existed when id219 expired.

## 5. Repair→MINCO initialization audit

Relevant C2 PLUS samples:

| epoch | path / pieces | old window → new duration | min/max piece T | initial v/a/j | downstream result |
|---|---|---|---|---|---|
| 4759.300 | 3.331 m / 17 | 2.512→1.753 before retime | 0.0417/0.1933 | 3.86/18.86/240.8 | retimed 10.482; model/trust exhausted |
| 4761.009 | 2.163 m | 0.625→1.563 | includes 0.0417 | 9.12/86.14/3057.7 | model/trust exhausted |
| 4762.284 | 5.870 m | 0.729→3.697 | finite/positive | 7.49/34.64/388.9 | QP max-iter; diagnostic distance 1.071 |
| 4763.178 | 5.848 m / 26 | 0.833→3.795 | 0.0417/0.2667 | 7.78/35.52/457.6 | QP max-iter; final distance 0.519 |
| 4763.931 | 2.290 m / 15 | 0.625→1.785 | 0.0417/0.3309 | 5.15/27.33/701.7 | retimed 11.609; still a=7.79, j=205.5; P-trust too small |

Answers:

- A. Pathological initialization near collision: YES. Values greatly exceed 3/6/22.
- B. These were genuine PLUS/RIGHT rescue candidates for C2: YES.
- C. Some frontend failures occurred without reaching MINCO; where MINCO was reached, solver/trust failures remained: YES.
- D. Did pathological init alone cause “no ABS → fallback → collision”: PARTIAL, not proven as the primary cause. At the decisive expiry cycle, the useful PLUS path was rejected before timing initialization. Also the 4762.284 and 4763.178 failed candidates' logged final dynamic distances were below 1.1, so solver success cannot be assumed to have produced an ABS trajectory.

The arc-length implementation does lengthen long paths and shifts downstream guide times (`planner_manager.cpp:2728-2747`), but the retained 0.041667 s minimum segments plus fixed endpoint derivatives still generate large derivatives at dense/high-curvature geometry. This is a confirmed remaining limitation, not the immediate physical collision mechanism.

## 6. Prediction horizon, sampling, and time alignment

### Horizon

`evaluateDynamicRisk()` checks only

`min(total_duration or 2/3 total_duration, moving_obj_prediction_horizon)`

at `planner_manager.cpp:721-725`. Runtime horizon was 2.0 s. Previous-trajectory revalidation uses this same function (`620-695`). Thus ABSOLUTE_SAFE does not prove a full 3–6 s trajectory.

For this collision, however:

- risk triggering already occurred at epoch 1788334759.299731, about 4.68 s before contact;
- id219 ended before collision rather than carrying an unchecked tail into it;
- after expiry every committed drone-0 fallback was explicitly INVALID, not falsely ABSOLUTE_SAFE.

Conclusion: `PREDICTION_HORIZON_TOO_SHORT` is a DESIGN_LIMITATION but NOT_CAUSAL here.

### Sampling

Risk dt is approximately 0.1 s (`planner_manager.cpp:730-764`). The collision interval lasted 0.166 s and relative speed was 1.425 m/s, corresponding to 0.143 m per 0.1 s. Coarse sampling can bias minima, but no near-collision trajectory crossed the 1.1 m class boundary because all were already far below it (0.933 down to 0.258). Fine reconstruction of logged id231 commands found no physical cylinder penetration. No class-changing aliasing was found.

Conclusion: no evidence of `DYNAMIC_RISK_SAMPLING_ALIASING` in this episode.

### Prediction/activation epoch

- planning epoch is captured at cycle start: `planner_manager.cpp:1191-1195`;
- risk query uses `prediction_epoch + trajectory_relative_t`: `713-760`;
- trajectory start is assigned only after optimization: `4284-4299`.

Observed mismatch:

- id229: epoch 1788334763.177687, start 1788334763.652200, delay 0.4745 s;
- id231: epoch 1788334763.732429, start 1788334763.783626, delay 0.0512 s.

This is a confirmed time-alignment bug. It did not falsely make these candidates safe—they were INVALID—but the same stale epoch participates in the stale-head activation problem.

`PREDICTION_EPOCH_MATCHES_ACTUAL_TRAJECTORY_START: NO`.

## 7. Command, activation, and odom

Trajectory 231 lifecycle:

- selected: 1788334763.783608;
- committed/start: 1788334763.783626;
- published: 1788334763.783906;
- traj_server receive/activation: 1788334763.784264;
- publish-to-activation delay: about 0.00036 s.

There is no evidence that traj_server failed to activate the selected trajectory. `traj_server.cpp:47-88` reconstructs and immediately replaces its active polynomial; `223-255` evaluates it by `now-start_time`. IDs were monotonic. The server lacks an old-id rejection check, which is a latent risk, but no stale old message caused this event.

The important discontinuity was upstream:

| ROS time | active command id | command x | odom x | position error |
|---:|---:|---:|---:|---:|
| 4763.535 | 228 | 18.135 | about 18.25 | about 0.12 m |
| 4763.636 | 228 | 18.383 | about 18.48 | about 0.10 m |
| 4763.746 | 230 | 17.679 | about 18.73 | about 1.07 m |
| 4763.846 | 231 | 17.709 | about 19.07 | about 1.38 m |
| 4763.955 | 231 | 17.814 | about 19.23 | about 1.43 m |
| 4764.045 minimum | 231 | about 17.939 | 19.312 | 1.395 m |

At deepest collision:

- odom signed C2 surface clearance: -0.0404 m;
- interpolated command signed C2 surface clearance: +1.0850 m;
- 0.01 s interpolation over the logged command window found minimum command clearance about +0.813 m, never ≤0.

Classification: `COMMAND_SAFE_BUT_ODOM_COLLIDES`.

The stale head is directly visible: the 4763.177 cycle starts at `(17.664, 2.452, 1.500)` and only commits at 4763.652. The vehicle has advanced while the optimizer runs. Subsequent warm starts inherit the newly committed trajectory, not current odom.

Source control:

- `ego_replan_fsm.cpp:699-703` uses active trajectory p/v/a as the planning head and computes odom error.
- `ego_replan_fsm.cpp:704-713` switches to odom only if `enable_fov_tracking_` is true.
- runtime log parameters: tracking-error threshold 0.5 m, but `use_fov_tracking: False`.
- the expired-trajectory replan-from-odom branch is also FOV-gated at `ego_replan_fsm.cpp:691-697`.

Thus the general tracking-error protection was effectively disabled in this non-FOV run. This is causal.

## 8. Three-class implementation audit

- ABS threshold is the runtime moving clearance 1.1 m: CONFIRMED_OK (`planner_manager.cpp:1409-1435`).
- Candidate checks and class reset each cycle: CONFIRMED_OK (`1413-1431`).
- Invalid risk cannot become ABS: CONFIRMED_OK (`1428-1434`).
- NOMINAL cannot become IMPROVED_ONLY: CONFIRMED_OK (`1437-1441`).
- PLUS/MINUS relative +0.10 applies only below absolute threshold: CONFIRMED_OK.
- Previous trajectory is sliced at elapsed time and revalidated with current predictions: CONFIRMED_OK, but only over the horizon/prefix (`620-725`).
- IMPROVED/INVALID cannot replace previous ABS: CONFIRMED_OK (`4093-4112`, `4158-4165`), observed repeatedly for id219.
- Any new ABS may replace a safer previous ABS: DESIGN_LIMITATION. Observed 1.942→1.864, 1.592→1.481, and 1.309→1.229. This is `ABSOLUTE_SAFE_MARGIN_EROSION`, but it did not directly cause collision because id219 later restored 1.873 and was protected until expiry.
- Post-check retries may mutate `committed_MJO`, but dynamic risk/class is not recomputed after dynamics/static/swarm retry (`4248-4291`): CONFIRMED_BUG_NOT_CAUSAL_HERE.
- The accepted-state cache stores every committed fallback, including INVALID trajectories (`4328-4338`), and later calls it `USED_PREVIOUS_SAFE`: CONFIRMED_BUG_NOT_CAUSAL_HERE / contributing to fallback propagation.

## 9. Prediction audit

The scene publisher creates actual state and absolute-time future Path samples from the same sinusoidal function (`native_egov2_rviz_scene.py:317-423`). Predictor samples preserve PoseStamped times and linearly interpolate (`obj_predictor.cpp:17-80`, `202-224`). With 0.5 s prediction sampling, the sinusoid interpolation-error bound for C2 is about 0.0084 m, far below the observed 1.395 m command/odom error.

No missing prediction, wrong obstacle id, reversed phase, clamp-to-end within the configured query horizon, or material C2 state mismatch was found. Dynamic prediction is not primary.

## 10. Semantic-v2, A*, rejoin, and SFC

- PLUS corresponded to intended physical RIGHT at C2 and was preferred in decisive cycles: NO_ISSUE.
- At expiry PLUS semantic-v2 passed without fallback and weighted-side ratio was 1.0: NO_ISSUE.
- PLUS A* succeeded with fresh, static-free raw points: NO_ISSUE in search itself.
- No stale A* path or rejoin-index evidence was found: NOT_CAUSAL.
- Local-SFC empty-plane rejection discarded the good path: CONFIRMED_CAUSAL upstream.
- Later accepted repair geometries contained dense 0.041667 s pieces and produced pathological MINCO derivatives: CONFIRMED contributing limitation.

## 11. Other code audit

| item | finding |
|---|---|
| stale safety class / success after QP failure | NO_ISSUE found; candidate objects reset and failed result remains unsuccessful |
| stale obstacle id / nominal risk | NO_ISSUE in this timeline; obstacle remained id3 consistently |
| active vs last candidate kind | NO causal inconsistency found |
| generation/start time uniqueness | CONFIRMED_OK in planner logs |
| traj_server old generation acceptance | LIKELY_RISK; callback has no monotonic-id guard, not exercised here |
| dynamic checker fail-open | NO_ISSUE for classification; invalid risk maps to INVALID |
| NaN/Inf entering selection | NO evidence in this episode |
| retime dynamic-risk refresh | CONFIRMED_OK before SCP (`planner_manager.cpp:3226-3257`) |
| final SIDE risk after SCP | CONFIRMED_OK (`3637`, `3686-3687`) |
| post-check dynamic-risk refresh | CONFIRMED_BUG_NOT_CAUSAL_HERE; absent after post-check mutation |
| repaired guide downstream time shift | CONFIRMED_OK (`2728-2747`) |
| final MINCO T used by risk | CONFIRMED_OK before class selection |
| warm start old obstacle context | LIKELY_RISK; cache is not safety-class tagged and unsafe fallbacks are cached |
| non-triggered ABS replacement | DESIGN_LIMITATION; permits same-class margin erosion |
| previous-safe slice | CONFIRMED_OK; expiry of id219 was correct |
| multithread race | NOT_ENOUGH_EVIDENCE |

## 12. Causal ranking

### PRIMARY

I. `TRAJECTORY_ACTIVATION_TIMING` / stale planning head: a 0.4745 s recovery cycle committed a trajectory still anchored to its old cycle-start state after the vehicle had moved substantially; later fallbacks inherited it. Tracking-error resynchronization was disabled because it is incorrectly coupled to FOV mode.

J. `COMMAND_ODOM_EXECUTION_ERROR`: the actual vehicle, not the active command, entered C2; error reached about 1.4 m. This is the immediate physical mechanism and is caused primarily by the command discontinuity above, not ordinary small tracking lag.

### SECONDARY

M. `LOCAL_SFC`: empty-plane hard rejection discarded a valid static-free, semantic-correct RIGHT A* recovery in the decisive expiry cycle.

G. `UNSAFE_FALLBACK_COMMITTED`: after id219 expired, repeated INVALID fallbacks were intentionally committed. This enabled the stale-head churn and final discontinuity.

### CONTRIBUTING

A. `REPAIR_TO_MINCO_INITIALIZATION`: pathological v/a/j persisted in genuine rescue candidates.

B. `SCP_QP_TRUST_FAILURE`: removed additional recovery opportunities, but not every failed candidate was capable of ABS safety and the decisive cycle failed before SCP.

F. `PREVIOUS_SAFE_EXPIRED`: necessary transition condition, but correct lifecycle behavior, not a bug.

E. `ABSOLUTE_SAFE_MARGIN_EROSION`: observed earlier, but id219 later restored margin and was protected.

### NOT CAUSAL HERE

C. `PREDICTION_HORIZON_TOO_SHORT`; D. `DYNAMIC_RISK_SAMPLE_DT_TOO_COARSE`; H. `DYNAMIC_PREDICTION_MISMATCH`; K. `SIDE_TOPOLOGY`; L. `A_STAR_REJOIN`.

## 13. Required final fields

COLLISION_UAV: UAV1 / planner drone_id=0

COLLISION_OBSTACLE: C2 / obstacle_id=3

COLLISION_TIME: relative 62.342278–62.508722 s; minimum at 62.408668 s; reconstructed ROS 1788334763.978870–1788334764.145314, minimum 1788334764.045260

LAST_ABSOLUTE_SAFE_TRAJECTORY: id219, generation219, NOMINAL, start=1788334760.734607458, duration=2.061875, predicted clearance=1.872659 m

LAST_ABSOLUTE_SAFE_REMAINING_AT_COLLISION: 0 s; expired about 1.182 s before collision start

ABSOLUTE_SAFE_AVAILABLE_IN_DECISIVE_CYCLE: NO

FINAL_TRAJECTORY_CLASS_BEFORE_COLLISION: INVALID / UNSAFE_FALLBACK

FINAL_TRAJECTORY_KIND: NOMINAL, trajectory_id=231

FINAL_TRAJECTORY_PREDICTED_CLEARANCE: 0.258281 m center distance over planner check

FINAL_COMMAND_ACTUALLY_COLLIDES: NO

REPAIR_TO_MINCO_INITIALIZATION_PATHOLOGICAL_NEAR_COLLISION: YES

PATHOLOGICAL_INIT_CAUSED_RECOVERY_FAILURE: PARTIAL

PREDICTION_HORIZON_CAUSAL: NO

RISK_SAMPLING_ALIASING_FOUND: NO

PREDICTION_TIME_ALIGNMENT_BUG: YES

PREDICTION_EPOCH_MATCHES_ACTUAL_TRAJECTORY_START: NO

ABSOLUTE_SAFE_MARGIN_EROSION_FOUND: YES, not causal here

PREVIOUS_SAFE_PROTECTION_WORKED_AS_DESIGNED: YES

EMERGENCY_FALLBACK_DIRECTLY_LED_TO_COLLISION: YES, through repeated stale-head activation and command discontinuity; not because id231 command was physically inside C2 at the collision instant

DYNAMIC_PREDICTION_MISMATCH_FOUND: NO

EXECUTION_TRACKING_PRIMARY: YES

OTHER_CONFIRMED_COLLISION_CAUSES: Local-SFC empty-plane rejection of a valid RIGHT A* recovery; tracking-error/expired-trajectory odom resynchronization gated by `use_fov_tracking`; unsafe fallback churn; stale planning/prediction epoch across long solve

PRIMARY_ROOT_CAUSE: stale-start trajectory activation after long recovery latency, with odom resynchronization disabled in non-FOV mode, producing a ~1.4 m command/odom split and physical C2 penetration

SECONDARY_ROOT_CAUSE: no ABSOLUTE_SAFE replacement at id219 expiry because a valid static-free semantic PLUS A* path was rejected by the Local-SFC empty-plane handoff; other repair candidates also failed backend optimization

CONTRIBUTING_FACTORS: pathological repair MINCO initialization; QP/trust failures; nonblocking INVALID fallback commits; same-class ABS margin erosion; prediction epoch tied to planning start

OTHER_CODE_BUGS_FOUND: post-check mutation without dynamic-risk/class recomputation; unsafe committed fallbacks stored in the “accepted/previous safe” warm-start cache; traj_server lacks monotonic trajectory-id rejection; full-trajectory safety is not proven beyond 2 s / 2/3 prefix

RUN1_C2_PRIMARY_CAUSE: NOT REPAIR_TO_MINCO_INITIALIZATION; primary is stale-start activation plus disabled odom resynchronization

NEXT_MINIMAL_FIX: decouple expired-trajectory and tracking-error replan-from-odom logic from `use_fov_tracking`, and revalidate/re-anchor the trajectory head against current odom immediately before commit after long planning latency. Separately, allow a static-free semantic A* guide to reach downstream repair when sparse Local-SFC legitimately yields zero planes. Advice only; no change was made.
