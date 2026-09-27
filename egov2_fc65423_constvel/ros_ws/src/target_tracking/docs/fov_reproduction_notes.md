# Rao 2025 Reproduction Notes

This package is a first executable FOV-based baseline for later comparison with EGO-Planner-v2.

I did not find an exact public GitHub implementation for:

`Rao K, Yan H C, Yang P H, Wang M, Lv Y. Multi-UAV trajectory planning with field-of-view sharing mechanism in cluttered environments: application to target tracking. Science China Information Sciences, 2025. DOI: 10.1007/s11432-024-4394-6`

The implementation here follows the paper structure from the local PDF and notes:

1. Target prediction with fixed time intervals.
2. Desired tracking point `p_dp = target + R(yaw) * relative_tracking`.
3. Field-of-view sharing: if one UAV is occluded but a teammate still sees the target, the visible UAV reports the target global coordinate on the shared object odometry topic, and the occluded UAV continues planning from that shared target state.
4. Yaw follows the UAV velocity direction in FOV mode, matching the paper's fixed-camera yaw penalty `psi_v = atan2(v_y, v_x)`.
5. EGO-v2 back-end optimization keeps the nominal tracking point and adds distance-band, fixed-height, angular-separation, and yaw costs.
6. The older candidate tracking-point selector is retained behind `fov_enable_candidate_goal` / `enable_candidate_goal`, but it is disabled by default because it is an engineering fallback rather than the default Rao reproduction path.

There is no fixed observer/bypasser role allocation in Rao et al. 2025. All UAVs run the same loop: report the target global coordinate when visible, predict the target state, search a feasible tracking point, construct the safety constraint used by the planner, and optimize the trajectory. The three UAVs only differ by their configured relative tracking points and angular distribution around the target.

The standalone prototype node is C++:

`target_tracking/fov_tracking_planner_node`

It follows the same ROS integration layer as the current EGO-v2 tracking setup: it subscribes to `/object_odom` and `/drone_{id}_visual_slam/odom`, then publishes `quadrotor_msgs/PositionCommand` on `/drone_{id}_planning/pos_cmd`.

For platform experiments, FOV tracking is now implemented inside EGO-Planner-v2 instead of using the standalone direct-command node. By default, the EGO FSM keeps the configured relative tracking point, and the EGO optimizer adds distance-band, fixed-height, angular-separation, and yaw costs. This keeps EGO's MINCO smoothing, dynamic feasibility, and obstacle handling.

The current FOV model is geometric, not image-based. `fov_half_angle_deg` defines a virtual camera viewing cone used by the optimizer's yaw penalty and the run-analysis visibility metric.

The Python script is kept only as a readable prototype/reference.

This is not yet a bit-for-bit reproduction of the full paper. The main missing pieces are:

1. Full kinodynamic A* over motion primitives.
2. Polyhedral safe flight corridor construction.
3. Full multi-segment MINCO optimization with all penalty gradients.
4. Direct integration with cluttered-map ESDF/occupancy maps.

Standalone launch:

`roslaunch target_tracking fov_tracking_planner.launch`

Platform launch through the existing tracking stack, using EGO with FOV tracking enabled:

`roslaunch multi_uav_formation target_tracking_platform_single.launch planner_backend:=fov`

Batch entry remains the existing script:

`./batch_run_target_tracking.sh --planner fov`
