uint8 PHASE_REALIZATION=0
uint8 PHASE_EXECUTOR_READY=1
uint8 PHASE_EXECUTION_REVOKE=2

uint8 phase
uint64 team_solution_id
uint64 task_reference_generation   # 阶段 B：realization 所用的 CooperativeTaskReference.generation
uint64 coordination_generation
int16 drone_id
uint64 planning_generation
int32 source_candidate_id
time stamp
bool accepted
string reason

# A positive ACK is also the Local Planner's realization certificate.  The
# coordinator never fabricates or modifies this polynomial; it only commits
# the three identity-matched Local results atomically.
uint64 team_reference_id
uint64 target_prediction_revision
time target_snapshot_epoch
uint64 dynamic_prediction_identity
uint64 static_map_revision
uint64 visibility_model_version
time evaluation_start_world
time activation_time
float64 execution_horizon
uint8 topology_kind
uint64 frontier_owner_revision
int32 frontier_owner_trajectory_id
bool realization_valid
traj_utils/MINCOTraj realized_trajectory
float64 realization_latency_ms

# Exact discrete SIDE certificate applied by the Local realization and, when
# active, by the extra Team P/T SCP.  The coordinator/realized validator must
# consume this certificate instead of reconstructing a different region from
# the historical joint seed.
bool side_contract_active
geometry_msgs/Point side_contract_origin
geometry_msgs/Vector3 side_contract_direction
int8 side_contract_sign
float64 side_contract_offset
float64 side_contract_conflict_progress
float64 side_contract_guidance_window
# Frozen, PVA-timed SIDE seed defining the one-sided topology lower bound.
traj_utils/MINCOTraj side_contract_seed_trajectory
