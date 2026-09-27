uint8 STATE_PROPOSAL=0
uint8 STATE_ABORT=1

uint8 STATE_REALIZATION_READY=2

uint8 state
bool valid
uint64 team_reference_id
uint64 task_reference_generation   # CooperativeTaskReference.generation（阶段 B identity 链）
uint64 coordination_generation
uint64 target_prediction_revision
time target_snapshot_epoch
uint64 dynamic_prediction_identity
time dynamic_prediction_valid_from
time dynamic_prediction_valid_to
uint64 static_map_revision
uint64 visibility_model_version
int32 hypothesis_id
uint64 encirclement_generation
time snapshot_stamp
time evaluation_start_world
time activation_time
float64 prediction_horizon
float64 execution_horizon
uint32 knots_per_member
string reason

int16[] drone_ids
uint64[] planning_generations
int32[] source_candidate_ids
uint8[] topology_kinds
uint64[] frontier_owner_revisions
int32[] frontier_owner_trajectory_ids

# Flattened in drone-major order.  Each member has knots_per_member entries.
float64[] knot_world_times
float64[] knot_radii
float64[] knot_bearings
float64[] knot_heights
# Compatibility field: reference phase shifts only.  These values never own
# or modify Local MINCO piece durations.
float64[] time_shifts

float64 objective_before
float64 objective_after
float64 j_k2_raw
float64 j_acc_raw
float64 j_comp_raw
float64 j_enc_raw
float64 j_reg_raw
float64 j_k2_weighted
float64 j_acc_weighted
float64 j_comp_weighted
float64 j_enc_weighted
float64 j_reg_weighted
float64 complementarity_before
float64 complementarity_after
float64 pair_angle_p50
float64 pair_angle_p90
float64 simultaneous_los_loss_before
float64 simultaneous_los_loss_after
uint32 optimization_iterations
float64 optimization_time_ms

# Predictive Visibility Relay contract.  These fields are immutable for the
# lifetime of team_reference_id and use absolute world time.  They do not
# grant execution authority; the resulting Local polynomials still pass the
# existing realized validator and PREPARE/READY/COMMIT chain.
bool handoff_contract_active
uint8 handoff_repair_k   # 0=none, 2=existing M2, 3=K3 binary All3 repair
uint64 handoff_contract_id
uint64 handoff_world_revision
int16 handoff_outgoing_uav
int16 handoff_incoming_uav
int16 handoff_stable_observer_uav
time handoff_acquire_by_world_time
time handoff_preserve_until_world_time
float64 handoff_required_overlap
float64 handoff_required_visibility_margin
time handoff_contract_created_time
time handoff_contract_expire_time
float64 handoff_m2_before
float64 handoff_m2_min
time handoff_m2_min_time

# Immutable creation-time owners and the current direct-successor frontier.
# They make persistence across Local rolling explicit without rebasing the
# contract's original audit identity.
uint64[] handoff_baseline_owner_revisions
int32[] handoff_baseline_owner_trajectory_ids
uint64[] handoff_expected_lineage_revisions
int32[] handoff_expected_lineage_trajectory_ids

# Feedback098: the UAV that actually produced the M2 minimum at the critical
# world time.  Explicitly propagated so the refinement targets a named UAV
# instead of re-deriving it downstream from stale execution state.
int16 handoff_limiting_uav
float64 handoff_limiting_margin
time handoff_critical_world_time
time handoff_forecast_created_world_time

# Feedback100: M2 only DETECTS the risk; it no longer decides the single UAV
# that gets repaired.  These fields carry the repair target set (the two
# channels whose restoration can raise M2) and the per-UAV forecast margins at
# the critical instant, so every node derives the same repair priority without
# adding a protocol round trip.
int16[] handoff_repair_candidates
float64[] handoff_repair_candidate_margins
int16 handoff_repair_priority_uav
float64[] handoff_forecast_margins
time handoff_t_cross
time handoff_t_recover
time handoff_t_star

# Feedback100 pipeline anchors (monotonic wall clock, same host as the node
# that consumes them) so the Planner can report the full forecast->activation
# latency chain and INTERVENTION_SLACK without a new protocol.
float64 handoff_detect_wall
float64 handoff_request_wall

# K3 Local escalation detect binding(2026-09-25 第二轮):contract 创建时刻的
# 事件证据,供 abortTeamProposal 原样转写进 TopologyEscalation。
# detect_limiter 为 ContinuousVisibilitySample::Limiter(0 none/1 static/2 dynamic/3 fov/4 range),
# detect_k2/detect_k3 为评估窗口内二值 K2/all-3 样本占比(M2 合同为 0)。
uint8 handoff_detect_limiter
float64 handoff_detect_k2
float64 handoff_detect_k3
