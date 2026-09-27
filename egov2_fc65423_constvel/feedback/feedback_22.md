# Feedback 22 — ALP Codebase-Aware Red Trajectory Roughness + Mid-Flight Stop Audit

Audit date: 2026-09-05

Scope:

- Read-only source audit of /home/bob/ALP/egov2_fc65423_constvel.
- Offline examination of the latest available launcher logs and trajectory CSV files.
- No production source, parameter, launch, RViz, or scene changes.
- No new simulation or experiment.

Runs used as primary runtime evidence:

- feasible_side_validation_20260904/alp_3
- manual_rviz_runs/c1_relaxed_v2_alp.log and c1_relaxed_v2_alp_visibility_trajectory.csv
- manual_rviz_runs/forest_alp.log and forest_alp_visibility_trajectory.csv

The first run is the canonical Feedback 20 validation. The latter two are later RViz-enabled runs and therefore provide the closest available evidence for the reported red-trajectory observation. No bag or persisted Marker point stream was present, so quantities that require the original Marker messages or complete polynomial coefficient messages are explicitly marked N/A rather than inferred.

## Executive conclusion

The two observations are only partially related.

The red forward line is not an odometry trail, A-star path, or raw control polygon. It is a visualization_msgs/Marker LINE_STRIP produced by PlanningVisualization from sparse samples of the selected, committed planner MINCO trajectory. There are five subdivisions per polynomial piece. RViz joins those sparse samples with straight chords. This creates a strong polygonal visualization component even when the underlying quintic is C2 continuous.

There is also genuine planner-side spatial roughness. Safe FEASIBLE_INITIALIZER_FALLBACK trajectories retain the fixed-P SIDE/A-star repair geometry after full SCP fails. Across the three examined runs, their executed active prefixes have materially higher curvature than SCP_FINAL trajectories. One single active fallback polynomial, alp_3 drone 1 trajectory 94, reaches approximately 19.82 1/m sampled curvature and a 34.87 degree heading change over approximately 0.1 s. This is inside one generation, not a marker-generation splice.

MINCO P/V/A continuity is not shown to have failed. The current MinJerkOpt banded system explicitly imposes position, velocity, and acceleration continuity at every piece boundary. Jerk continuity is not required. The logs do not persist all coefficients or junction states, so numerical junction residuals cannot be reconstructed from the retained artifacts; there is no source or runtime evidence of a C2 break.

Generation switching is a confirmed secondary contributor. The freshness gate checks only head position error against odometry, with a 0.5 m threshold. It reads odometry velocity but does not gate on velocity or acceleration. In the canonical run, reconstructed switches show maximum head-state jumps of about 0.303 m, 0.529 m/s, and 2.385 m/s², with a maximum heading change of 10.917 degrees. This can make the displayed trajectory change abruptly and can create an execution transient, but the RViz marker itself does not mix two generations.

No qualifying odometry stop lasting at least 0.5 s was found in the three current complete CSVs after excluding startup and final stop. However, two command-level mid-flight hold events were recovered exactly. In both, a synchronous SIDE/SCP solve blocked the same callback thread that publishes planner heartbeat. The solve exceeded the traj_server 0.5 s heartbeat timeout. traj_server then invalidated the active trajectory and published last position with exactly zero velocity, acceleration, and jerk, even though the active trajectory still had more than one second remaining. Therefore:

- stop scheduling root cause: S7, long synchronous planning/lifecycle starvation;
- direct command mechanism: S2, heartbeat-timeout hold;
- trajectory expiry was not the cause of the two recovered events.

Feedback 20's previous-safe final-use revalidation is present and working in the examined runs. No previous-safe trajectory with zero or negative remaining duration was retained. The two recovered heartbeat holds restarted with an ABSOLUTE_SAFE SIDE/SCP_FINAL trajectory and an ABSOLUTE_SAFE NOMINAL trajectory respectively, not INVALID NOMINAL.

## Codebase map

In this report, planner/ is shorthand for the active runtime tree ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/. The separate ros_ws/src/ego-planner tree exists in the repository but is not the planner tree selected by native_egov2_rviz.launch.

| Layer | Current source | Key node/class/function |
|---|---|---|
| Top launch | ros_ws/src/multi_uav_formation/launch/native_egov2_rviz.launch | Starts scene, target coordinator, three planner stacks, and RViz |
| Scene and metrics | ros_ws/src/multi_uav_formation/scripts/native_egov2_rviz_scene.py | NativeEgoV2RvizScene; map/obstacle publication, odometry and command callbacks, CSV recording |
| Constant-velocity scene wrapper | ros_ws/src/multi_uav_formation/scripts/native_egov2_constvel_scene.py | Selects the current scene implementation |
| Target coordinator | ros_ws/src/multi_uav_formation/scripts/target_state_coordinator.py | TargetStateCoordinator; waits for required UAVs, publishes object odometry/start time |
| Target wrapper | ros_ws/src/multi_uav_formation/scripts/target_state_constvel_coordinator.py | Selects the constant-velocity target setup |
| Planner launch | ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage/launch/advanced_param.xml | Starts drone_i_ego_planner_node and remaps odometry, object state, PolyTraj, heartbeat |
| FSM | planner/plan_manage/src/ego_replan_fsm.cpp | EGOReplanFSM::execFSMCallback, callReboundReplan, planFromLocalTraj, polyTraj2ROSMsg |
| Planner manager | planner/plan_manage/src/planner_manager.cpp | EGOPlannerManager::reboundReplan, dynamic-risk evaluation, bilateral candidate generation/selection/commit |
| SIDE repair A-star | planner/path_searching/include/path_searching/dyn_a_star.h and src/dyn_a_star.cpp | AStar::AstarSearch |
| A-star guide and Local-SFC | planner/plan_manage/src/planner_manager.cpp | SIDE collision-window repair, LOS simplification, turn/core reinsertion, local half-space generation |
| MINCO/SCP orchestration | planner/traj_opt/src/poly_traj_optimizer.cpp | PolyTrajOptimizer::runCandidateHardCorridorSCP and analytic sample-row construction |
| OSQP adapter | planner/traj_opt/src/scp_optimizer.cpp | SCPOptimizer::solve |
| MINCO math | planner/traj_opt/include/optimizer/poly_traj_utils.hpp | MinJerkOpt::reset, generate, getTraj, getInitConstraintPoints, getGrad2TP |
| Polynomial message/data | planner/traj_utils | PolyTraj message and trajectory utilities |
| Trajectory server | planner/plan_manage/src/traj_server.cpp | trajectoryCallback, heartbeatCallback, publish_cmd, cmdCallback |
| Position controller | ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/uav_simulator/so3_control/src/so3_control_nodelet.cpp | position_cmd_callback, odom_callback |
| Quadrotor simulator | ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/uav_simulator/so3_quadrotor_simulator/src/quadrotor_simulator_so3.cpp | SO3 callback, simulation step, odometry publication |
| Planner visualization | planner/traj_utils/src/planning_visualization.cpp | displayOptimalList, displayMarkerList |
| RViz configuration | ros_ws/src/multi_uav_formation/config/native_egov2.rviz | UAV 1/2/3 trajectory Marker displays on optimal_list |

## Confirmed runtime chain

The current chain is:

native_egov2_rviz.launch
→ native_egov2_constvel_scene.py / NativeEgoV2RvizScene
→ target_state_constvel_coordinator.py / TargetStateCoordinator
→ drone_i_ego_planner_node
→ EGOReplanFSM::execFSMCallback
→ EGOReplanFSM::callReboundReplan
→ EGOPlannerManager::reboundReplan
→ nominal MinJerkOpt trajectory
→ EGOPlannerManager::evaluateDynamicRisk
→ bilateral SIDE_PLUS and SIDE_MINUS candidate attempts
→ SIDE geometric seed
→ static collision-window scan
→ AStar::AstarSearch when repair is required
→ greedy bounded LOS simplification
→ greater-than-20-degree turn-point reinsertion
→ at most two collision-core raw-point reinsertions
→ Local-SFC half-space construction or SFC_NOT_REQUIRED
→ rebuilt MINCO internal P and piece T
→ authoritative-lattice retiming
→ fixed-P time-only feasibility correction when required
→ safe initializer classification/cache
→ full MINCO-SCP-OSQP refinement
→ final safety classification
→ bilateral candidate selection
→ final-use previous-safe revalidation/fallback selection
→ EGOPlannerManager::setLocalTrajFromOpt
→ EGOReplanFSM::polyTraj2ROSMsg
→ PolyTraj publication
→ traj_server trajectoryCallback and polynomial reconstruction
→ traj_server cmdCallback
→ quadrotor_msgs/PositionCommand
→ SO3ControlNodelet
→ SO3 command
→ QuadrotorSimulatorSO3
→ odometry
→ NativeEgoV2RvizScene metrics/CSV and RViz displays.

The selected planner trajectory is also sampled by MinJerkOpt::getInitConstraintPoints and passed through PlanningVisualization::displayOptimalList to the red optimal_list marker.

CODEBASE_RUNTIME_CHAIN_CONFIRMED: YES

## SIDE, A-star, Local-SFC, MINCO, and SCP findings

### Dynamic-risk and SIDE behavior

The current dynamic-risk trigger uses:

- moving-object clearance: 1.1 m;
- candidate trigger margin: 0.15 m;
- resulting trigger distance: 1.25 m.

The SIDE base offset is 0.7 m, with successive attempts corresponding to 0.7, 0.6, 0.5, and 0.4 m. The preferred side changes ordering only. Both PLUS and MINUS are still attempted.

The lateral direction is formed from the nominal start-to-goal tangent and world Z using the current cross-product convention in planner_manager.cpp. Static collision is checked on the SIDE candidate. Repair is localized to its collision window.

### A-star simplification and repair reconstruction

The current implementation calls AStar::AstarSearch from dyn_a_star.cpp. The raw path is simplified with:

- bounded greedy line-of-sight pruning;
- reinsertion of raw path turning points exceeding 20 degrees;
- at most two extra raw points near the collision core;
- restoration in original raw-path order.

The reconstructed guide combines original pre-window junctions, a repair anchor, the simplified A-star guide, a rejoin anchor, and original post-window junctions.

Aggregate repair geometry:

| Run | Repairs/guides | Mean raw points | Mean simplified points | Mean turn points | Mean core reinserts | Median/min repair piece T | Typical guide curvature |
|---|---:|---:|---:|---:|---:|---:|---:|
| alp_3 | 50 | 17.60 | 12.66 | 8.66 | 2.00 | 0.041667 s | median 6.3405 1/m |
| c1_relaxed_v2 | 49 | 21.18 | 13.20 | 9.29 | 1.96 | 0.041667 s | P50 6.3405 1/m; max 10 1/m |
| forest | 43 | 18.84 | 12.74 | 8.74 | 2.00 | 0.041667 s | median/P95 6.3405 1/m |

The turn and core reinsertion stages can produce closely spaced guide points. The rebuilt timing contains pieces as short as 0.041667 s in the aggregate data. Fixed-P time-only correction changes duration only and therefore cannot remove a sharp spatial P configuration.

ASTAR_GUIDE_SHORT_SEGMENT_CONTRIBUTION: YES

### Local-SFC behavior

Local-SFC is the current lightweight local half-space scheme, not a full FIRI/ellipsoid corridor. A plane stores its normal, boundary point, clearance, and active time interval. A valid build with no necessary planes returns SFC_NOT_REQUIRED and is permitted to continue; it is not treated as SFC_BUILD_FAILED.

Zero-plane outcomes are common in the examined repairs: 32/50 in alp_3 and approximately two thirds in the two RViz runs. This is expected when the repaired trajectory is already static-free and is not itself evidence of an SFC failure.

### MINCO and SCP state

The full SCP state remains:

x = [vec(P); tau]

where P contains 3(N-1) internal-position variables and tau contains N unconstrained virtual-time variables. Physical positive durations are computed as T = f(tau). The QP step dimension is 4N-3.

The code obtains directional sample derivatives through MINCO analytic adjoint propagation. mincoSampleGradientWrtX invokes the MINCO gradient machinery rather than explicitly materializing a complete sample Jacobian.

The current hard rows include:

- SIDE preservation corridor;
- Local-SFC;
- native EGO static separating rays;
- sampled velocity, acceleration, and jerk constraints;
- P trust region;
- physical-time-scaled tau/T trust region.

Moving-obstacle separation is still not an explicit full-SCP hard row. It enters the native time-aware cost and is checked again by manager-side dynamic risk/safety classification.

The QP Hessian is a diagonal step regularizer with weight 1 for P components and 4 for tau components. It is not the Hessian of the complete native EGO objective. The linear term uses the current native-objective gradient.

## Red forward trajectory identity

### RViz display and topic

native_egov2.rviz defines three Marker displays named UAV 1 trajectory, UAV 2 trajectory, and UAV 3 trajectory. Their topics are:

- /drone_0_ego_planner_node/optimal_list
- /drone_1_ego_planner_node/optimal_list
- /drone_2_ego_planner_node/optimal_list

RED_TRAJECTORY_TOPIC: /drone_i_ego_planner_node/optimal_list

RED_TRAJECTORY_PUBLISHER: drone_i_ego_planner_node through PlanningVisualization::displayOptimalList

RED_TRAJECTORY_SOURCE_FILE: ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/traj_utils/src/planning_visualization.cpp; data prepared in plan_manage/src/planner_manager.cpp

### Marker semantics

displayOptimalList uses red RGBA (1, 0, 0, 1) and calls displayMarkerList. Each update publishes:

- a visualization_msgs/Marker SPHERE_LIST with id 0;
- a visualization_msgs/Marker LINE_STRIP with id 1000.

No namespace is assigned, so the namespace is the empty string. The action is ADD. No lifetime is assigned, so lifetime is zero/infinite. The frame is world.

Because topic, namespace, type, and id are fixed, a new ADD replaces the previous marker content. The implementation does not append points to an existing Marker and does not concatenate two trajectory generations. It also does not explicitly DELETE the previous marker. Consequently the latest marker can remain visible indefinitely if no replacement is published, including after traj_server enters a hold.

MARKER_TYPE: visualization_msgs/Marker; red LINE_STRIP id 1000 plus red SPHERE_LIST id 0

MARKER_REPLACE_OR_APPEND: REPLACE/OVERWRITE by fixed namespace and id; not append

MARKER_GENERATION_MIXING: NO

Old marker last point connected to new marker first point: NO

Marker lifetime: zero/infinite

### Exact data source

The marker points are the selected committed MinJerkOpt trajectory's constraint/sample points. planner_manager.cpp recomputes the displayed points from the committed MJO after final post-checks. MinJerkOpt::getInitConstraintPoints uses constraint_points_perPiece = 5, samples each piece at five subdivisions, and removes the duplicate piece-junction sample.

For N pieces the marker therefore has N×5+1 vertices. These vertices lie on the selected polynomial. RViz then joins adjacent vertices by straight LINE_STRIP chords.

The marker is not:

- an odometry history;
- a past-plus-future path;
- an A-star raw path;
- an A-star simplified path;
- a SIDE-only seed independent of selection;
- a traj_server continuously updated remaining trajectory;
- PositionCommand history.

RED_TRAJECTORY_DATA_SOURCE: PLANNER_TRAJECTORY — sparse samples of the selected committed active MINCO polynomial

RED_TRAJECTORY_IS_ACTIVE_POLYTRAJ: PARTIAL — same selected polynomial and exact sample vertices at commit, but a sparse persistent planner snapshot rather than a live traj_server remaining-trajectory rendering

At the marker vertices, marker-to-PolyTraj position error is zero by construction apart from serialization/floating-point roundoff: both come from the same committed MinJerkOpt. Between vertices, the LINE_STRIP is a straight chord and generally differs from the polynomial curve. The original Marker point streams were not retained, so an independent numerical maximum chord error cannot be reconstructed.

MARKER_TO_POLYTRAJ_ERROR: 0 at published vertices by construction; between-vertex chord error N/A because Marker messages were not persisted

Command-to-active-PolyTraj position error is also zero by construction apart from floating-point roundoff: traj_server reconstructs and evaluates the exact fifth-order coefficients carried by PolyTraj.

### Why it looks jagged

The visual severity is a combination of two effects:

1. Visualization: only five subdivisions per polynomial piece are connected by straight segments. High-curvature and short-piece regions become visibly polygonal.
2. Planner geometry: some fallback polynomials genuinely contain high-curvature repair geometry. Sparse chords make those bends look even sharper.

The marker is therefore not simply false, but it is not a faithful dense rendering of the smooth polynomial between its vertices.

VISUALIZATION_ARTIFACT_CONFIRMED: YES — chordal sampling exaggerates roughness

PLANNER_TRAJECTORY_GEOMETRY_ROUGHNESS_CONFIRMED: YES

## Single-generation MINCO continuity and curvature

### Continuity

MinJerkOpt::generate constructs and solves the banded system with:

- head PVA constraints;
- internal position constraints;
- position, velocity, and acceleration continuity at each junction;
- tail PVA constraints.

This makes each generated trajectory C2 at its internal junctions to numerical solver precision. Jerk may jump at a piece boundary and is not required to be continuous.

Complete polynomial coefficients and junction states were not persisted in the examined logs/CSV files. Therefore a numerical junction-residual audit cannot be honestly reconstructed offline.

MINCO_P_CONTINUITY_MAX_ERROR: N/A — coefficients/junction states not retained; source algebra explicitly imposes zero residual

MINCO_V_CONTINUITY_MAX_ERROR: N/A — coefficients/junction states not retained; source algebra explicitly imposes zero residual

MINCO_A_CONTINUITY_MAX_ERROR: N/A — coefficients/junction states not retained; source algebra explicitly imposes zero residual

MINCO_PVA_CONTINUITY_LOSS_CONFIRMED: NO

JERK_JUMP_IS_MINCO_CONTINUITY_BUG: NO

### Three concrete rough fallback generations

| Case | Generation | Selection | N / internal P / marker G | Repair | Local-SFC | Time-only | Full SCP | Authoritative max v/a/j |
|---|---|---|---|---|---|---|---|---|
| R1 | alp_3 drone 1 id 94 | SIDE_PLUS, ABSOLUTE_SAFE, FEASIBLE_INITIALIZER_FALLBACK | 14 / 13 / 71 | raw 11, simplified 10, turns 6, reinserts 2; min T 0.091268 s | 2 planes | yes; jerk 22.000064→22.000000 | 57.479 ms, DYNAMICS_MODEL_MISMATCH_TRUST_EXHAUSTED | 3.013597 / 4.405128 / 22.000000 |
| R2 | c1 drone 1 id 105 | SIDE_PLUS, ABSOLUTE_SAFE, FEASIBLE_INITIALIZER_FALLBACK | 16 / 15 / 81 | raw 22, simplified 14, turns 10, reinserts 2; min T 0.041667 s | 0, SFC_NOT_REQUIRED | yes; tiny duration increase | 118.667 ms, QP_MAX_ITER_EXHAUSTED | 1.791003 / 2.991118 / 21.360356 |
| R3 | forest drone 1 id 91 | SIDE_PLUS, ABSOLUTE_SAFE, FEASIBLE_INITIALIZER_FALLBACK | 17 / 16 / 86 | raw 20, simplified 15, turns 11, reinserts 2; min T 0.058926 s | 0, SFC_NOT_REQUIRED | yes | 207.245 ms, DYNAMICS_MODEL_MISMATCH_TRUST_EXHAUSTED | 1.311854 / 2.894339 / 22.010813 |

R1 is the strongest recovered proof of single-polynomial spatial roughness. Within trajectory id 94, without crossing a generation boundary:

- maximum sampled curvature: 19.8194 1/m;
- maximum heading change over approximately 0.1 s: 34.8736 degrees.

The following ids in the same run reach lower but still visible maxima:

- id 95: curvature 3.515 1/m, heading change 5.79 degrees;
- id 96: curvature 5.780 1/m, heading change 9.46 degrees.

SINGLE_POLYNOMIAL_HIGH_CURVATURE_CONFIRMED: YES

The observed shape is not explained by a loss of P/V/A continuity. A C2 polynomial can have high curvature, especially where velocity is small or the fixed spatial guide bends sharply.

## Feedback 20 fallback smoothness comparison

Feedback 20 correctly preserves an ABSOLUTE_SAFE feasible initializer when full refinement fails. Source inspection confirms that:

- fixed-P time-only feasibility correction changes T but not P;
- the saved initializer is restored when refinement fails and remains safely classifiable;
- the displayed and committed trajectory is then the saved initializer.

This preserves safety but can preserve sharper A-star/SIDE repair geometry that a successful P-changing full SCP could have smoothed.

Aggregate active-command-prefix geometry across alp_3, c1_relaxed_v2, and forest, after rejecting corrupted numeric outliers:

| Metric | FEASIBLE_INITIALIZER_FALLBACK | SCP_FINAL |
|---|---:|---:|
| Curvature samples | 214 | 362 |
| Mean curvature | 0.9913 1/m | 0.5335 1/m |
| Curvature P50 | 0.5039 1/m | 0.3689 1/m |
| Curvature P95 | 2.9336 1/m | 1.6552 1/m |
| Maximum curvature | 19.8194 1/m | 2.9847 1/m |
| Heading-change samples | 137 | 213 |
| Mean heading change | 4.3058 degrees | 3.4551 degrees |
| Heading-change P50 | 3.1786 degrees | 2.8151 degrees |
| Heading-change P95 | 9.3965 degrees | 8.5322 degrees |
| Maximum heading change | 34.8736 degrees | 16.0291 degrees |

The canonical alp_3 difference is stronger:

- fallback curvature P50/P95/max: 0.8943 / 4.2735 / 19.8194 1/m;
- SCP_FINAL curvature P50/P95/max: 0.3415 / 1.1463 / 2.7945 1/m;
- fallback heading P50/P95/max: 3.149 / 13.815 / 34.874 degrees;
- SCP_FINAL heading P50/P95/max: 2.586 / 5.572 / 8.276 degrees.

The difference is primarily spatial. Executed acceleration and finite-difference jerk do not show a consistent fallback penalty:

- aggregate command acceleration P95: fallback 2.006 versus SCP_FINAL 2.051 m/s²;
- aggregate command acceleration-difference jerk P95: fallback 10.145 versus SCP_FINAL 11.117 m/s³.

This is consistent with Feedback 20 preserving authoritative v/a/j feasibility while allowing a spatially sharper safe initializer.

FEASIBLE_INITIALIZER_FALLBACK_ROUGHER_THAN_SCP_FINAL: YES

FALLBACK_VS_SCP_CURVATURE: fallback mean/P95/max 0.9913/2.9336/19.8194 1/m versus SCP_FINAL 0.5335/1.6552/2.9847 1/m

FALLBACK_VS_SCP_HEADING_CHANGE: fallback mean/P95/max 4.3058/9.3965/34.8736 degrees versus SCP_FINAL 3.4551/8.5322/16.0291 degrees

The comparison is observational rather than a matched candidate A/B experiment, but the preservation mechanism is directly confirmed in source.

## Generation switching audit

### Fresh-head gate

The current final commit freshness check obtains candidate head position and current odometry. It also obtains odometry velocity for surrounding logic, but the actual freshness decision is only:

norm(candidate position at t=0 minus odometry position) <= 0.5 m.

There is no velocity-error gate and no acceleration-error gate.

FRESH_HEAD_POSITION_GATE: 0.5 m

FRESH_HEAD_VELOCITY_GATE: NONE

FRESH_HEAD_ACCELERATION_GATE: NONE

In alp_3, accepted final head-position errors, n=827:

- P50: 0.070118 m;
- P95: 0.206346 m;
- max: 0.491038 m.

Seven stale candidates were rejected:

- P50: 0.575728 m;
- P95: 0.689769 m;
- max: 0.690029 m.

### Reconstructed switch state jumps

Using the retained 10 Hz PositionCommand stream, the old trajectory state was locally propagated to the first command of the new generation. Meaningful switches require both sides to have speed at least 0.3 m/s. For n=777 switches:

HEAD_POSITION_JUMP_P50_P95_MAX: 0.028085 / 0.079983 / 0.302531 m

HEAD_VELOCITY_JUMP_P50_P95_MAX: 0.009174 / 0.076323 / 0.529323 m/s

HEAD_ACCELERATION_JUMP_P50_P95_MAX: 0.063402 / 0.583962 / 2.385230 m/s²

MAX_SWITCH_HEADING_CHANGE: 10.917 degrees

Selected transition highlights:

| Transition | Count | Notable maximum |
|---|---:|---|
| NOMINAL→NOMINAL | 703 | heading 10.917 degrees |
| NOMINAL→SIDE_PLUS | 19 | position 0.1993 m; heading 3.455 degrees |
| SIDE_PLUS→NOMINAL | 19 | acceleration 2.385 m/s²; heading 9.630 degrees |
| SIDE_PLUS→SIDE_PLUS | 11 | heading 6.313 degrees |
| SIDE_MINUS→SIDE_MINUS | 9 | heading 6.821 degrees |
| SIDE_MINUS→NOMINAL | 8 | acceleration 1.608 m/s²; heading 9.510 degrees |
| NOMINAL→SIDE_MINUS | 7 | velocity 0.529 m/s; heading 5.995 degrees |

ROUGHNESS_AT_GENERATION_SWITCH_CONFIRMED: YES

This is a secondary source of abrupt visual or execution changes. It is not marker generation mixing: every marker update replaces the complete preceding marker.

## Marker, command, and odometry alignment

The source relationship is:

- marker vertices versus selected planner polynomial: exact samples, zero positional deviation by construction;
- PositionCommand versus active PolyTraj: exact coefficient evaluation, zero model deviation by construction;
- odometry versus PositionCommand: controller/simulator tracking difference.

Two rough fallback segments were aligned using the retained CSV:

| Case | Samples | Odom-command position P50/P95/max | Odom-command velocity P50/P95/max |
|---|---:|---:|---:|
| alp_3 drone 1 id 94 | 12 | 0.0776 / 0.1147 / 0.1204 m | 0.199 / 0.308 / 0.328 m/s |
| c1 drone 1 id 105 | 11 | 0.1355 / 0.1542 / 0.1569 m | 0.3022 / 0.3842 / 0.4224 m/s |

The controller has tracking error, but the red line is produced upstream from the planner polynomial. Controller behavior therefore cannot be the primary cause of red marker jaggedness.

## Mid-flight stop audit

### Automatic stop detection

The current complete CSVs were scanned for:

- primary stop: odometry speed below 0.10 m/s continuously for at least 0.5 s;
- strict stop: odometry speed below 0.05 m/s continuously for at least 0.5 s.

Startup, the period before formal target motion, final target stop, and shutdown were excluded.

MID_FLIGHT_STOP_EVENTS: 0 qualifying odometry events in the three current complete runs; 2 shorter command-level heartbeat holds recovered from logs/CSV

STRICT_LT_0_05_STOP_EVENTS: 0

The absence of a qualifying 0.5 s odometry stop does not erase the command-level fault. The two captured holds were shorter than the requested odometry-event duration and vehicle inertia prevented speed from crossing the threshold, but their exact zero-command mechanism is recoverable.

### Heartbeat and callback threading

EGOReplanFSM creates a 0.01 s FSM timer. execFSMCallback publishes heartbeat once at callback entry, then can synchronously execute the complete replan, bilateral SIDE attempts, time-only correction, and full SCP/OSQP solve before returning.

HEARTBEAT_PUBLISH_LOCATION: EGOReplanFSM::execFSMCallback entry in plan_manage/src/ego_replan_fsm.cpp

The heartbeat does not run independently from the long synchronous planning callback.

traj_server stores the heartbeat timestamp. In cmdCallback, if heartbeat age exceeds 0.5 s it:

- reports planner heartbeat loss;
- sets receive_traj_ = false;
- publishes one command at the last position;
- sets command velocity, acceleration, and jerk to exactly zero.

HEARTBEAT_TIMEOUT: 0.5 s

After this invalidation, a new PolyTraj message is required to set receive_traj_ true and resume trajectory sampling.

### Recovered command-level hold H1

Run: feasible_side_validation_20260904/alp_3

Vehicle: drone 1

Timeline:

- active trajectory id 139 started at 1788524800.940816641;
- trajectory duration: 2.290345 s;
- same planner entered a SIDE_MINUS candidate/full-SCP path;
- full planning/optimization episode: 567.849 ms, ending with P_TRUST_TOO_SMALL;
- heartbeat-loss errors appeared at approximately 1788524801.6346 and 1788524801.7146;
- at 1788524801.714670, traj_server published id 139 last position with exactly zero velocity and acceleration;
- active trajectory still had approximately 1.516 s remaining;
- next nonzero command, id 140, arrived at 1788524802.014710;
- zero-command gap: approximately 0.300 s;
- restart: SIDE_PLUS, ABSOLUTE_SAFE, SCP_FINAL_OK, duration 23.198828 s.

Classification:

- PRIMARY: S7 long synchronous SIDE/SCP planning starved heartbeat;
- EXECUTION MECHANISM: S2 traj_server heartbeat-timeout hold;
- NOT trajectory expiry;
- NOT INVALID NOMINAL restart.

### Recovered command-level hold H2

Run: manual_rviz_runs/c1_relaxed_v2_alp

Vehicle: drone 0

Timeline:

- active trajectory id 44 started at 1788527338.301087856;
- trajectory duration: 2.240076 s;
- SIDE_PLUS full-SCP planning took 608.575 ms and ended with P_TRUST_TOO_SMALL;
- previous-safe final revalidation succeeded with approximately 1.354 s remaining;
- despite the valid remaining trajectory, the same callback had already starved heartbeat;
- heartbeat timeout and exact zero command occurred at approximately 1788527339.187872;
- next nonzero command, id 45, arrived at 1788527339.288273;
- zero-command gap: approximately 0.100 s;
- restart: NOMINAL, ABSOLUTE_SAFE, dynamic clearance approximately 3.550 m.

Classification:

- PRIMARY: S7 long synchronous planning/lifecycle starvation;
- EXECUTION MECHANISM: S2 heartbeat-timeout hold;
- NOT trajectory expiry;
- NOT previous-safe remaining-duration TOCTOU;
- NOT INVALID NOMINAL restart.

### Planning latency and warnings

Heartbeat-loss warnings in the examined runs:

- alp_3: 4;
- c1_relaxed_v2: 4;
- forest: 2;
- total: 10.

Maximum logged full-SCP times:

- alp_3: 617.093 ms;
- c1_relaxed_v2: 608.575 ms;
- forest: 449.105 ms.

The canonical Feedback 20 run has approximately 217.4 ms full-SCP P95 and 617.1 ms maximum. A bilateral cycle can include PLUS, MINUS, retiming, time-only feasibility, and multiple full-SCP attempts, so total callback occupancy can exceed the 0.5 s heartbeat timeout even when a still-valid active trajectory exists.

HEARTBEAT_LOSS_AROUND_STOP: YES — confirmed around both recovered command-level holds

PLANNER_BLOCKED_DURING_STOP: YES — synchronous candidate solve occupied the heartbeat callback thread

MAX_PLANNING_TIME_AROUND_STOP: 608.575 ms in H2; 567.849 ms in H1

ACTIVE_TRAJECTORY_EXPIRED_DURING_STOP: NO

POSITION_COMMAND_ZERO_DURING_STOP: YES

TRAJ_SERVER_HOLD: YES

### Trajectory expiry and controller behavior

Source inspection shows a separate design risk:

- if trajectory time exceeds its duration, traj_server silently stops publishing commands;
- it does not publish a continuous explicit terminal hold stream;
- SO3ControlNodelet retains and republishes the previous desired state when no new PositionCommand arrives.

This can produce an expiry/last-desired hold in other circumstances. However, no local-trajectory-expired warning was present around the two recovered current holds, and their active trajectories still had substantial remaining time. It is therefore not the confirmed primary mechanism here.

S1 finite-trajectory-end wait: DESIGN_RISK, not confirmed for current holds

S2 heartbeat timeout: CONFIRMED direct mechanism

S3 expired trajectory/no new command: DESIGN_RISK, not confirmed for current holds

S4 polynomial contains long near-zero-speed segment: NOT observed in qualifying current events

S5 controller/simulator failure: NOT supported

S6 rejection/no commit: contributing risk, not direct mechanism in H1/H2

S7 synchronous planning lifecycle starvation: CONFIRMED root cause of H1/H2

S8 other: no stronger current mechanism found

STOP_PRIMARY_TYPE: S7

STOP_DIRECT_EXECUTION_MECHANISM: S2 — heartbeat timeout publishes last-position zero-PVAJ hold and sets receive_traj_ false

## Feedback 20 previous-safe revalidation

The final-use previous-safe revalidation is present at all relevant selection/commit fallback points. It recomputes remaining duration near actual use rather than relying solely on the cycle-start result.

Current-run evidence:

| Run | Final revalidations | Minimum retained remaining duration | Zero/negative retained | Expired invalidations |
|---|---:|---:|---:|---:|
| alp_3 | 494 | 1.526338 s | 0 | 73 |
| c1_relaxed_v2 | 149 | 1.236632 s | 0 | 51 |
| forest | 265 | 1.761167 s | 0 | 110 |

PREVIOUS_SAFE_ZERO_REMAINING_RETAIN_REAPPEARED: NO

Feedback 18's old previous-safe zero-remaining TOCTOU is therefore not the current primary stop cause.

## INVALID NOMINAL after holds

The canonical Feedback 20 run still contains INVALID NOMINAL commits elsewhere, so that emergency policy remains a real lifecycle/safety concern. It is not the restart path for the two recovered heartbeat holds:

- H1 restarted with SIDE_PLUS, ABSOLUTE_SAFE, SCP_FINAL_OK;
- H2 restarted with NOMINAL, ABSOLUTE_SAFE.

INVALID_NOMINAL_AFTER_STOP: NO — for the recovered current heartbeat holds

FEASIBLE_INITIALIZER_AFTER_STOP: NO — H1 restarted with SCP_FINAL; H2 with safe NOMINAL

No current evidence supports the Feedback 18 chain stop → odometry replan → SIDE failure → INVALID NOMINAL as the explanation for these captured holds.

## Root-cause ranking

### A. RED_FORWARD_TRAJECTORY_NOT_SMOOTH

PRIMARY — CONFIRMED, visualization layer:

- PlanningVisualization publishes only five subdivisions per piece as a straight LINE_STRIP.
- High-curvature polynomial sections are rendered as visibly angular chords.
- Source: planning_visualization.cpp and MinJerkOpt::getInitConstraintPoints.
- Runtime evidence: rough cases have 71/81/86 marker vertices for 14/16/17-piece trajectories, and one single polynomial reaches 19.8194 1/m curvature.

PRIMARY — CONFIRMED, planner geometry layer:

- FEASIBLE_INITIALIZER_FALLBACK preserves fixed-P SIDE/A-star repair geometry after full SCP fails.
- Source: planner_manager.cpp candidate preservation/restore path and poly_traj_optimizer.cpp.
- Runtime evidence: fallback curvature mean/P95/max 0.9913/2.9336/19.8194 versus SCP_FINAL 0.5335/1.6552/2.9847 1/m.

SECONDARY — CONFIRMED:

- Generation switching can have small position error but materially larger velocity/acceleration/direction mismatch because final freshness checks position only.
- Runtime maximum: 0.529 m/s velocity jump, 2.385 m/s² acceleration jump, 10.917 degree heading change.

CONTRIBUTING:

- short repair pieces and concentrated A-star guide turns;
- fixed-P time-only correction cannot smooth P geometry;
- infinite marker lifetime can leave a stale planned red line visible after traj_server is no longer executing it.

NOT_CAUSAL:

- marker append or cross-generation point mixing;
- loss of MINCO P/V/A continuity;
- jerk discontinuity by itself;
- controller or simulator as the source of the red marker.

PLANNER_GEOMETRY_PRIMARY_FOR_RED_ROUGHNESS: YES — primary source of genuine single-polynomial curvature

GENERATION_SWITCH_PRIMARY_FOR_RED_ROUGHNESS: NO — confirmed secondary contributor

VISUALIZATION_PRIMARY_FOR_RED_ROUGHNESS: YES — primary amplifier of the visibly polygonal red line

CONTROLLER_PRIMARY_FOR_RED_ROUGHNESS: NO

RED_TRAJECTORY_ROOT_CAUSE: sparse five-subdivision-per-piece LINE_STRIP chord rendering of a selected MINCO trajectory that can itself be spatially sharp when Feedback 20 executes a safe fixed-P A-star/SIDE initializer after full SCP failure; generation-state mismatch is secondary, not marker mixing

### B. MID_FLIGHT_STOP

PRIMARY — CONFIRMED:

- S7 long synchronous SIDE/time-only/SCP work runs inside the same FSM callback path that publishes heartbeat.
- 567.849 ms and 608.575 ms captured episodes exceed the 0.5 s traj_server timeout.

DIRECT EXECUTION MECHANISM — CONFIRMED:

- S2 traj_server heartbeat timeout sets receive_traj_ false and publishes last position with zero velocity, acceleration, and jerk.
- Both recovered events occurred while the active polynomial still had more than one second remaining.

SECONDARY:

- recovery requires a new PolyTraj to re-enable receive_traj_;
- bilateral candidate latency and retry structure increase callback occupancy;
- position-only freshness and candidate rejection can lengthen recovery in less favorable cycles.

CONTRIBUTING DESIGN RISKS:

- silent no-command behavior after normal trajectory expiry;
- controller retention of the last desired state when commands stop;
- INVALID NOMINAL remains possible elsewhere, though not after the two current holds.

NOT_CAUSAL FOR THE RECOVERED EVENTS:

- active trajectory expiry;
- previous-safe zero-remaining retention;
- a planned long zero-speed polynomial;
- controller/simulator failure.

LIFECYCLE_STARVATION_PRIMARY_FOR_STOP: YES

HEARTBEAT_PRIMARY_FOR_STOP: NO — heartbeat timeout is the direct mechanism; synchronous planning starvation is the root cause

CONTROLLER_PRIMARY_FOR_STOP: NO

MID_FLIGHT_STOP_ROOT_CAUSE: synchronous bilateral SIDE/SCP planning starves the planner heartbeat for longer than traj_server's 0.5 s watchdog, causing traj_server to invalidate a still-unexpired active trajectory and issue an exact zero-PVAJ hold

## Relationship between the two phenomena

ARE_TWO_PHENOMENA_CAUSALLY_LINKED: PARTIAL

They share an upstream candidate lifecycle:

- a difficult SIDE/A-star case can create a sharp feasible initializer;
- full SCP failure preserves that initializer as a fallback, increasing visible spatial roughness;
- a long full SCP attempt in the same synchronous callback can starve heartbeat and trigger a hold.

Their direct mechanisms are different:

- red roughness: sparse chord visualization plus genuine high-curvature planner geometry;
- stop: heartbeat watchdog hold caused by lifecycle starvation.

A rough fallback trajectory does not itself command a stop, and a heartbeat hold does not create the marker's internal corners. The infinite-lifetime marker can, however, remain visible during a hold and make the two appear visually simultaneous.

## Minimal fix directions — not implemented

MINIMAL_FIX_DIRECTION_RED:

1. Visualization-only first step: publish an adaptively or densely sampled active-polynomial LINE_STRIP using a maximum time step, chord length, or chord-error/curvature criterion. Keep it separate from sparse constraint-point visualization.
2. Give the active marker a finite lifetime or explicitly clear/replace it on traj_server hold so a stale future plan is not presented as currently executable.
3. Preserve Feedback 20 safety fallback, but place a bounded geometry-quality gate or safety-constrained P-only smoothing step before executing a fallback whose curvature, heading change, or guide segment length exceeds a threshold. Retiming alone cannot solve spatial sharpness.
4. Extend final commit freshness from position-only to velocity and acceleration compatibility, or perform a short C2-compatible transition/blend. Do not relax safety classification.

MINIMAL_FIX_DIRECTION_STOP:

1. Move planner heartbeat to an independent timer/callback queue/thread that cannot be blocked by synchronous replanning.
2. Do not invalidate a still-unexpired validated active trajectory solely because planner heartbeat is temporarily late. Continue it until its own expiry, with a separate planner-death policy.
3. Put a hard wall-time budget around bilateral SIDE/full-SCP refinement, or run refinement asynchronously while retaining the last final-use-revalidated safe trajectory.
4. On real trajectory expiry, publish an explicit terminal hold command continuously instead of silently stopping PositionCommand; add a clear controller command-watchdog policy.

## Required result fields

CODEBASE_RUNTIME_CHAIN_CONFIRMED: YES

RED_TRAJECTORY_TOPIC: /drone_i_ego_planner_node/optimal_list

RED_TRAJECTORY_PUBLISHER: drone_i_ego_planner_node via PlanningVisualization::displayOptimalList

RED_TRAJECTORY_SOURCE_FILE: planner/traj_utils/src/planning_visualization.cpp; points supplied by planner/plan_manage/src/planner_manager.cpp

RED_TRAJECTORY_DATA_SOURCE: PLANNER_TRAJECTORY — sparse selected committed MINCO samples

RED_TRAJECTORY_IS_ACTIVE_POLYTRAJ: PARTIAL

MARKER_TYPE: visualization_msgs/Marker LINE_STRIP id 1000 plus SPHERE_LIST id 0

MARKER_REPLACE_OR_APPEND: REPLACE/OVERWRITE

MARKER_GENERATION_MIXING: NO

MARKER_TO_POLYTRAJ_ERROR: 0 at vertices by construction; between-vertex chord error N/A because Marker messages were not retained

MINCO_P_CONTINUITY_MAX_ERROR: N/A — coefficient/junction artifacts not retained; source enforces zero residual

MINCO_V_CONTINUITY_MAX_ERROR: N/A — coefficient/junction artifacts not retained; source enforces zero residual

MINCO_A_CONTINUITY_MAX_ERROR: N/A — coefficient/junction artifacts not retained; source enforces zero residual

SINGLE_POLYNOMIAL_HIGH_CURVATURE_CONFIRMED: YES

ROUGHNESS_AT_GENERATION_SWITCH_CONFIRMED: YES

HEAD_POSITION_JUMP_P50_P95_MAX: 0.028085 / 0.079983 / 0.302531 m

HEAD_VELOCITY_JUMP_P50_P95_MAX: 0.009174 / 0.076323 / 0.529323 m/s

HEAD_ACCELERATION_JUMP_P50_P95_MAX: 0.063402 / 0.583962 / 2.385230 m/s²

MAX_SWITCH_HEADING_CHANGE: 10.917 degrees

ASTAR_GUIDE_SHORT_SEGMENT_CONTRIBUTION: YES

FEASIBLE_INITIALIZER_FALLBACK_ROUGHER_THAN_SCP_FINAL: YES

FALLBACK_VS_SCP_CURVATURE: mean/P95/max 0.9913/2.9336/19.8194 versus 0.5335/1.6552/2.9847 1/m

FALLBACK_VS_SCP_HEADING_CHANGE: mean/P95/max 4.3058/9.3965/34.8736 versus 3.4551/8.5322/16.0291 degrees

MID_FLIGHT_STOP_EVENTS: 0 qualifying >=0.5 s odometry stops; 2 shorter exact command-level heartbeat holds

STRICT_LT_0_05_STOP_EVENTS: 0

STOP_PRIMARY_TYPE: S7

STOP_DIRECT_EXECUTION_MECHANISM: S2

HEARTBEAT_PUBLISH_LOCATION: EGOReplanFSM::execFSMCallback entry

HEARTBEAT_TIMEOUT: 0.5 s

HEARTBEAT_LOSS_AROUND_STOP: YES

PLANNER_BLOCKED_DURING_STOP: YES

MAX_PLANNING_TIME_AROUND_STOP: 608.575 ms

ACTIVE_TRAJECTORY_EXPIRED_DURING_STOP: NO

POSITION_COMMAND_ZERO_DURING_STOP: YES

TRAJ_SERVER_HOLD: YES

INVALID_NOMINAL_AFTER_STOP: NO for the two recovered current holds

FEASIBLE_INITIALIZER_AFTER_STOP: NO for the two recovered current holds

PREVIOUS_SAFE_ZERO_REMAINING_RETAIN_REAPPEARED: NO

PLANNER_GEOMETRY_PRIMARY_FOR_RED_ROUGHNESS: YES

GENERATION_SWITCH_PRIMARY_FOR_RED_ROUGHNESS: NO

VISUALIZATION_PRIMARY_FOR_RED_ROUGHNESS: YES

CONTROLLER_PRIMARY_FOR_RED_ROUGHNESS: NO

LIFECYCLE_STARVATION_PRIMARY_FOR_STOP: YES

HEARTBEAT_PRIMARY_FOR_STOP: NO — direct mechanism only

CONTROLLER_PRIMARY_FOR_STOP: NO

RED_TRAJECTORY_ROOT_CAUSE: sparse chord visualization plus genuinely sharp fixed-P fallback geometry; position-only generation freshness is secondary

MID_FLIGHT_STOP_ROOT_CAUSE: long synchronous SIDE/SCP callback blocks heartbeat, then traj_server's 0.5 s watchdog issues a zero-PVAJ hold despite an unexpired active trajectory

ARE_TWO_PHENOMENA_CAUSALLY_LINKED: PARTIAL

MINIMAL_FIX_DIRECTION_RED: dense/adaptive active-polynomial visualization; stale-marker clearing; safety-constrained fallback geometry-quality/smoothing step; velocity/acceleration-aware commit continuity

MINIMAL_FIX_DIRECTION_STOP: independent heartbeat; continue still-valid active trajectory through transient planner stalls; bounded/asynchronous SCP; explicit expiry hold stream

PRODUCTION_CODE_CHANGED: NO

SCENE_CHANGED: NO

NEW_EXPERIMENT_RUN: NO

CURRENT_FEEDBACK_FILE: /home/bob/ALP/egov2_fc65423_constvel/feedback/feedback_22.md

## Evidence limits

The following were not retained by the current logging setup:

- original RViz Marker point messages;
- all published PolyTraj coefficient messages;
- per-junction polynomial P/V/A state dumps;
- ROS bag data.

Therefore the marker chord-error maximum and numeric C2 junction residuals are N/A. These missing artifacts do not prevent the source-level identity of the red marker, the single-generation curvature evidence from the command stream, the generation-switch statistics, or the two heartbeat zero-command timelines from being established.
