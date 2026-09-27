int16 drone_id
uint64 planning_generation
uint64 transaction_id
time planning_epoch
time bundle_stamp
time target_prediction_epoch
time dynamic_prediction_epoch
# Prediction source identity is independent from the future execution time.
uint64 target_snapshot_identity
time target_snapshot_epoch
uint64 dynamic_prediction_identity
time dynamic_prediction_valid_from
time dynamic_prediction_valid_to
time joint_frontier_time
uint64 static_map_revision
uint64 visibility_model_version
uint64 expected_execution_generation
int32 expected_trajectory_id
# Immutable predecessor identity sampled by this transaction.  The local
# reserve is a value, not a separately committed predecessor.
uint64 boundary_source_revision
int32 boundary_source_trajectory_id
time committed_coverage_start
bool committed_safety_validated
traj_utils/MINCOTraj committed_trajectory
time earliest_activation
time validated_end
time evaluation_start_time
float64 requested_evaluation_horizon
bool outer_loop_evaluation
uint32 execution_history_size
float64 execution_history_median
geometry_msgs/Point target_position
geometry_msgs/Vector3 target_velocity
int32 local_selected_candidate_id
int32 local_selected_hypothesis_id
uint8 local_selected_kind
traj_utils/TopologyCandidate[] candidates

# Feedback098: this drone's own time-indexed visibility forecast for its
# current Local baseline trajectory.  The Coordinator fuses three of these
# into the same-world-time M2 instead of extrapolating execution polynomials.
traj_utils/VisibilityForecastTrace visibility_forecast
