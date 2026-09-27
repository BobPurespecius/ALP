int16 drone_id
int32 traj_id
time start_time

uint8 order
float64[] coef_x
float64[] coef_y
float64[] coef_z
float64[] duration

# The executor verifies this hash over the exact double coefficients,
# durations, trajectory id and activation timestamp before admission.
uint64 payload_hash
uint64 validation_revision
float64 checked_until

# Optional optimized yaw trajectory transported with the same position
# trajectory generation. Empty arrays mean the executor must use its
# target-facing fallback generator.
bool optimized_yaw_valid
float64[] yaw_sample_times
float64[] yaw_samples

# Executed source identity travels atomically with position/yaw, including scheduled activation.
string trajectory_source
uint64 generation
uint64 team_solution_id
bool team_prepare_only   # 阶段 G：true = 仅 PREPARE（executor 入 prepared 存储，不自动激活）
uint64 encirclement_generation
int32 viewpoint_hypothesis_id
bool safety_validated

# Lifecycle evidence belongs to the same executable payload.
time candidate_ready_time
time commit_time
float64 pending_duration
time validation_time
time validation_expiry
string lifecycle_state

# Soft reference followed while producing this payload; independent of execution source.
uint64 team_reference_id
string team_reference_reason

# ---------------------------------------------------------------------------
# Feedback093 explicit trajectory lineage contract.
#
# expected_predecessor_id is the identity of the trajectory this payload was
# built to continue.  The executor MUST resolve exactly this identity; it must
# never infer a predecessor from queue position, scheduled slot or active slot.
# A payload whose expected predecessor cannot be resolved is rejected with an
# explicit reason instead of being compared against some unrelated trajectory.
#
#   0 = bootstrap (no predecessor: first payload of a run, or odometry restart)
#
# lineage_class is the producer class of THIS payload:
#   "LOCAL" = guaranteed rolling successor of the Local Planner
#   "TEAM"  = speculative Team replacement (never authoritative before ACTIVATE)
# ---------------------------------------------------------------------------
int32 expected_predecessor_id
uint64 expected_predecessor_generation
string lineage_class
