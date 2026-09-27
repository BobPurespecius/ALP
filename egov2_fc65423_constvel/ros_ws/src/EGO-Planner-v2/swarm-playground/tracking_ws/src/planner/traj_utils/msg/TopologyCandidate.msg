uint8 KIND_NOMINAL=0
uint8 KIND_SIDE_PLUS=1
uint8 KIND_SIDE_MINUS=2

uint8 SAFETY_INVALID=0
uint8 SAFETY_IMPROVED_ONLY=1
uint8 SAFETY_ABSOLUTE_SAFE=2

int32 candidate_id
int32 encirclement_hypothesis_id
float64 encirclement_phi0
uint64 encirclement_generation
uint8 candidate_kind
uint8 safety_class
bool success
bool static_valid
bool dynamics_valid
bool swarm_valid
bool astar_used
bool local_sfc_used
bool feasible_initializer_fallback
int8 side_sign

# Physical conflict provenance.  These fields are metadata only; hard safety
# remains enforced by the existing current-revision preflight.
bool conflict_valid
uint8 conflict_reason_mask
uint8 conflict_obstacle_motion
int32 conflict_obstacle_identity
int32 conflict_primitive_type
string conflict_obstacle_name
uint64 conflict_geometry_revision
time conflict_first_risk_world_time
time conflict_risk_interval_start
time conflict_risk_interval_end
geometry_msgs/Point conflict_observer_position
geometry_msgs/Point conflict_target_position
geometry_msgs/Point conflict_obstacle_position
geometry_msgs/Vector3 conflict_frame_forward
geometry_msgs/Vector3 conflict_frame_right
geometry_msgs/Point conflict_clearance_witness
float64 conflict_clearance
float64 conflict_radius

# Explicit LOS provenance is retained separately when the primary witness is
# BODY (BODY+LOS).  The primary conflict fields above remain the safety
# witness; these fields prevent Joint/Team serialization from collapsing the
# observation blocker into a generic BODY side.
bool los_conflict_valid
uint8 los_obstacle_motion
int32 los_obstacle_identity
int32 los_primitive_type
time los_conflict_time
time los_first_risk_world_time
geometry_msgs/Point los_obstacle_position
geometry_msgs/Vector3 los_frame_right

# Pre-final topology seed used only by EARLY_JOINT_PRIMARY.  It is an
# initializer/corridor contract, never an executable trajectory certificate.
bool joint_seed_valid
bool joint_seed_static_constructible
bool joint_seed_local_sfc_valid
float64 joint_seed_construction_ms
time joint_seed_start_time
float64 joint_seed_dynamic_clearance
traj_utils/MINCOTraj joint_seed
bool joint_seed_binary_visibility_valid
float64[] joint_seed_visibility_sample_offsets
uint8[] joint_seed_self_visibility_samples
geometry_msgs/Vector3[] joint_seed_local_sfc_normals
geometry_msgs/Point[] joint_seed_local_sfc_points
float64[] joint_seed_local_sfc_clearances
float64[] joint_seed_local_sfc_start_ratios
float64[] joint_seed_local_sfc_end_ratios
bool[] joint_seed_local_sfc_world_time_anchored

time trajectory_start_time
float64 duration
float64 evaluation_horizon
float64 native_cost
float64 dynamic_clearance
float64 swarm_clearance
float64 max_velocity
float64 max_acceleration
float64 max_jerk
float64 tracking_score
float64 radial_deviation
float64 angular_deviation_deg

float64 uav1_visibility
float64 uav2_visibility
float64 uav3_visibility
float64 all3_visibility
float64 atleast2_visibility
float64 none_visibility
float64 atleast_k_visibility
float64 mean_visible_count
float64 min_pairwise_view_angle_deg
float64 mean_pairwise_view_angle_deg
float64 diversity_score
float64 team_utility

# Candidate-self LOS/range visibility on the planner's authoritative
# visibility lattice.  The coordinator aligns these offsets in global ROS
# time and combines the three self traces into all-3/K-of-N statistics.
bool binary_visibility_valid
float64[] visibility_sample_offsets
uint8[] self_visibility_samples

# Predicted target-facing yaw trace on the same candidate-local time axis.
# Stage-1 team P/T refinement treats this trace as frozen input.  It is not an
# optimized yaw trajectory and does not change traj_server's fallback yaw.
float64[] predicted_yaw_sample_offsets
float64[] predicted_yaw_samples

# Sparse Local-SFC contract retained from SIDE/A* repair.  Active times are
# stored as normalized progress so a small joint duration update preserves
# association with the repaired path interval.
geometry_msgs/Vector3[] local_sfc_normals
geometry_msgs/Point[] local_sfc_points
float64[] local_sfc_clearances
float64[] local_sfc_start_ratios
float64[] local_sfc_end_ratios
# 本轮修复（动态 LOS world-time 语义）：与上面数组一一对应，标记该平面的时间
# 区间是否绑定在世界时间遮挡事件上。true 时下游（team optimizer 的 retime 与
# coverage 校验）不得按 new_duration/old_duration 统一缩放它。
bool[] local_sfc_world_time_anchored

traj_utils/MINCOTraj trajectory
