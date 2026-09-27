#include "optimizer/scp_optimizer.h"
#ifndef _POLY_TRAJ_OPTIMIZER_H_
#define _POLY_TRAJ_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <plan_env/grid_map.h>
#include <plan_env/obj_predictor.h>
#include <plan_env/static_los_geometry.h>
#include <ros/ros.h>
#include <cstdint>
#include <limits>
#include <functional>
#include <string>
#include "optimizer/lbfgs.hpp"
#include <traj_utils/plan_container.hpp>
#include <traj_utils/ablation_runtime.h>
#include "poly_traj_utils.hpp"
#include "scp_optimizer.h"

// Feedback098: the visibility-margin authority is shared with the Coordinator
// through the Planner-side forecast.  Only a forward declaration is needed in
// this header; the definition lives in the translation unit that already
// includes the visibility geometry.
namespace multi_uav_formation
{
  struct ContinuousVisibilitySample;
}

namespace ego_planner
{

  // Sparse local half-space reused by the candidate SCP path.  The geometry
  // follows the native static ConstraintPoints convention:
  //   normal.dot(position - point) >= clearance.
  // The row is active only on [active_start, active_end] in trajectory time.
  struct LocalSfcPlane
  {
    enum Source
    {
      STATIC_COLLISION_CORRIDOR = 0,
      LOS_OBSERVATION_SIDE = 1
    };
    int source{STATIC_COLLISION_CORRIDOR};
    int blocker_index{-1};
    int blocker_type{0};
    int side_sign{0};
    unsigned long geometry_revision{0};
    Eigen::Vector3d normal{Eigen::Vector3d::Zero()};
    Eigen::Vector3d point{Eigen::Vector3d::Zero()};
    // Retained only for static Local-SFC association diagnostics.  These
    // fields do not participate in the constraint values.
    Eigen::Vector3d guide{Eigen::Vector3d::Zero()};
    int source_segment{-1};
    int source_guide_index{-1};
    // STATIC planes bind to one emitted MINCO piece. Their active interval is
    // expressed in that piece's normalized local coordinate, never a total
    // duration ratio. LOS planes retain world-time semantics below.
    int piece_id{-1};
    double piece_u_begin{0.0};
    double piece_u_end{1.0};
    double clearance{0.0};
    double active_start{0.0};
    double active_end{0.0};
    // 本轮修复（动态 LOS world-time 语义）：true 表示该平面的时间区间绑定在
    // 世界时间事件上（例如某个动态 blocker 的真实遮挡区间），因此它绝不能
    // 随候选轨迹 retime 做统一比例缩放——缩放会改变遮挡事件对应的轨迹位置；
    // false（默认）表示区间由静态路径进度决定，按比例缩放是正确语义。
    bool world_time_anchored{false};
    // 本轮修复：该平面时间区间的"世界时间锚点"（绝对 ROS 时间）。
    // active_start/active_end 表达的是候选相对时间，其零点就是这个锚点。
    // 当候选被 re-anchor 到新的 activation 时，必须做
    //     t_rel_new = (t_rel_old + anchor_old) - anchor_new
    // 才能保持"遮挡发生在世界时刻 t_w"这一语义不漂移。
    double world_anchor_time{0.0};
  };

  class ConstraintPoints
  {
  public:
    friend struct TeamReferenceContract;
    int cp_size; // deformation points
    Eigen::MatrixXd points;
    std::vector<std::vector<Eigen::Vector3d>> base_point; // The point at the statrt of the direction vector (collision point)
    std::vector<std::vector<Eigen::Vector3d>> direction;  // Direction vector, must be normalized.
    std::vector<bool> flag_temp;                          // A flag that used in many places. Initialize it everytime before using it.

    void resize_cp(const int size_set)
    {
      cp_size = size_set;

      base_point.clear();
      direction.clear();
      flag_temp.clear();

      points.resize(3, size_set);
      base_point.resize(cp_size);
      direction.resize(cp_size);
      flag_temp.resize(cp_size);
    }

    void segment(ConstraintPoints &buf, const int start, const int end)
    {
      if (start < 0 || end >= cp_size || points.rows() != 3)
      {
        ROS_ERROR("Wrong segment index! start=%d, end=%d", start, end);
        return;
      }

      buf.resize_cp(end - start + 1);
      buf.points = points.block(0, start, 3, end - start + 1);
      buf.cp_size = end - start + 1;
      for (int i = start; i <= end; i++)
      {
        buf.base_point[i - start] = base_point[i];
        buf.direction[i - start] = direction[i];
      }
    }

    static inline int two_thirds_id(Eigen::MatrixXd &points, const bool touch_goal)
    {
      return touch_goal ? points.cols() - 1 : points.cols() - 1 - (points.cols() - 2) / 3;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  class PolyTrajOptimizer
  {

  public:
    friend struct TeamReferenceContract;
    struct CostGradientSnapshot
    {
      double smoothness_cost{0.0};
      double static_cost{0.0};
      double swarm_cost{0.0};
      double moving_cost{0.0};
      double side_cost{0.0};
      double side_region_cost{0.0};
      double preserve_cost{0.0};
      double tracking_cost{0.0};
      double feasibility_cost{0.0};
      double variance_cost{0.0};
      double time_cost{0.0};
      double visibility_cost{0.0};
      double visibility_raw_cost{0.0};
      double directional_visibility_integral{0.0};
      double total_cost{0.0};
      double static_grad_norm{0.0};
      double moving_grad_norm{0.0};
      double side_grad_norm{0.0};
      double side_region_grad_norm{0.0};
      double preserve_grad_norm{0.0};
      double tracking_grad_norm{0.0};
      double smoothness_grad_norm{0.0};
      double variance_grad_norm{0.0};
      double visibility_grad_norm{0.0};
      double total_grad_norm{0.0};
      double moving_risk_time{-1.0};
      double moving_risk_distance{-1.0};
      double moving_risk_weight{1.0};
      double moving_risk_cost{0.0};
      int visibility_sample_count{0};
      int visibility_blocked_samples{0};
      double visibility_mean_clearance{std::numeric_limits<double>::infinity()};
      double visibility_min_clearance{std::numeric_limits<double>::infinity()};
      int directional_visibility_sample_count{0};
      int directional_visibility_activations{0};
      int static_los_visibility_grad_active_samples{0};
      int dynamic_los_visibility_grad_active_samples{0};
      int fov_visibility_grad_active_samples{0};
      double directional_visibility_cost_mean{0.0};
      double directional_visibility_cost_max{0.0};
      double directional_static_los_cost_mean{0.0};
      double directional_static_los_cost_max{0.0};
      double directional_dynamic_los_cost_mean{0.0};
      double directional_dynamic_los_cost_max{0.0};
      double directional_fov_cost_mean{0.0};
      double directional_fov_cost_max{0.0};
      // Earliest point at which the already-authoritative shared visibility
      // support becomes nonzero.  Candidate topology construction needs this
      // time witness; a collision predictor's unrelated closest-approach time
      // is not a valid proxy for an LOS/FOV event.
      double visibility_support_first_trajectory_time{
          std::numeric_limits<double>::infinity()};
      Eigen::Vector3d visibility_support_first_position{
          Eigen::Vector3d::Zero()};
      Eigen::Vector3d visibility_support_first_target{
          Eigen::Vector3d::Zero()};
      double directional_visibility_grad_mean{0.0};
      double directional_visibility_grad_max{0.0};
      double directional_visibility_cost_ratio{0.0};
      double directional_visibility_grad_ratio{0.0};
      double directional_visibility_compute_ms{0.0};
      double directional_static_compute_ms{0.0};
      double directional_dynamic_compute_ms{0.0};
      double directional_fov_compute_ms{0.0};
      int static_witness_switch_count{0};
      int dynamic_witness_switch_count{0};
      double witness_switch_grad_p95{0.0};
      double witness_switch_grad_max{0.0};
      int directional_grad_max_piece{-1};
      int directional_grad_max_sample{-1};
      double directional_grad_max_local_time{0.0};
      double directional_grad_max_trajectory_time{0.0};
      double directional_grad_max_global_time{0.0};
      Eigen::Vector3d directional_grad_max_position{Eigen::Vector3d::Zero()};
      Eigen::Vector3d directional_grad_max_target{Eigen::Vector3d::Zero()};
      double directional_grad_max_yaw{0.0};
      double directional_grad_max_static_clearance{std::numeric_limits<double>::infinity()};
      double directional_grad_max_dynamic_clearance{std::numeric_limits<double>::infinity()};
      double directional_grad_max_horizontal_margin{std::numeric_limits<double>::infinity()};
      double directional_grad_max_vertical_margin{std::numeric_limits<double>::infinity()};
      double directional_grad_max_static_risk{0.0};
      double directional_grad_max_dynamic_risk{0.0};
      double directional_grad_max_fov_risk{0.0};
      double directional_grad_max_static_component{0.0};
      double directional_grad_max_dynamic_component{0.0};
      double directional_grad_max_fov_component{0.0};
      double directional_grad_max_static_time_derivative{0.0};
      double directional_grad_max_dynamic_time_derivative{0.0};
      double directional_grad_max_fov_time_derivative{0.0};
      int directional_grad_max_static_witness{-1};
      int directional_grad_max_dynamic_witness{-1};
    };

    struct OptimizationDiagnostics
    {
      bool valid{false};
      int evaluations{0};
      CostGradientSnapshot initial;
      CostGradientSnapshot final;
    };

    struct CandidateDynamicsSummary
    {
      bool valid{false};
      int sample_count{0};
      int velocity_constraint_count{0};
      int acceleration_constraint_count{0};
      int jerk_constraint_count{0};
      double max_vel_value{0.0};
      double max_acc_value{0.0};
      double max_jerk_value{0.0};
      double max_vel_violation{0.0};
      double max_acc_violation{0.0};
      double max_jerk_violation{0.0};

      double maxViolation() const
      {
        return std::max(max_vel_violation,
                        std::max(max_acc_violation, max_jerk_violation));
      }
    };

  private:
    GridMap::Ptr grid_map_;
    AStar::Ptr a_star_;
    poly_traj::MinJerkOpt jerkOpt_;
    SwarmTrajData *swarm_trajs_{NULL}; // Can not use shared_ptr and no need to free
    fast_planner::ObjPredictor::Ptr moving_objs_;
    StaticLosGeometry static_los_geometry_;
    ConstraintPoints cps_;
    // PtsChk_t pts_check_;

    int drone_id_;
    int cps_num_prePiece_;   // number of distinctive constraint points each piece
    int variable_num_;       // optimization variables
    int piece_num_;          // poly traj piece numbers
    int iter_num_;           // iteration of the solver
    int last_lbfgs_status_{lbfgs::LBFGSERR_UNKNOWNERROR};
    double min_ellip_dist2_; // min trajectory distance in swarm
    bool touch_goal_;
    bool satrt_tracking_;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    /* optimization parameters */
    double wei_obs_, wei_obs_soft_;                               // obstacle weight
    double wei_swarm_;                                            // swarm weight
    double wei_tracking_;                                         // tracking weight
    /* 合围模式下的软方位恢复权重，以及真实执行前缀内的加强倍数。
     * 参考方位来自 cooperative seed；纯 soft cost，hard safety 优先。 */
    double encirclement_bearing_recovery_weight_{0.30};
    double prefix_bearing_recovery_boost_{2.0};
    double prefix_execution_span_{0.0};      // [s] 真实执行前缀长度
    double encirclement_base_angular_slack_{0.3490658503988659};
    bool encirclement_tracking_configured_{false};
    bool encirclement_tracking_active_{false};
    double observation_radius_band_{0.35};
    double encirclement_max_gap_{170.0*M_PI/180.0};
    double encirclement_min_gap_{25.0*M_PI/180.0};
    double execution_deadline_{std::numeric_limits<double>::infinity()}, execution_call_reserve_{0.05};
    double encirclement_local_gap_weight_{100.0};
    double observation_height_band_{0.20};
    bool encirclement_visibility_guidance_active_{false};
    traj_utils::AblationConfig ablation_config_;
    bool formal_ablation_mode_{false};
    double directional_static_risk_transition_{0.80};
    double directional_moving_risk_transition_{0.80};
    double directional_visibility_deep_risk_kappa_{1.0};
    double directional_visibility_deep_risk_beta_{2.0};
    double wei_feas_;                                             // feasibility weight
    double wei_sqrvar_;                                           // squared variance weight
    double wei_time_;                                             // time weight
    double obs_clearance_, obs_clearance_soft_, swarm_clearance_; // safe distance
    double max_vel_, max_acc_, max_jer_;                          // dynamic limits
    bool use_time_aware_moving_obj_cost_{false};
    double moving_obj_clearance_{1.1};
    // Physical body/obstacle contact distance.  The preferred 1.1 m margin
    // remains the soft avoidance target; this value is used only by hard
    // dynamic feasibility checks and is derived from live geometry.
    // Production iris horizontal collision footprint (rotor outer extent).
    double moving_obj_hard_body_radius_{0.384};
    // Feedback126 unified safety kernel: the checker-side feasibility
    // tolerance, shared so generation and validation use one PVAJ definition.
    double feasibility_tolerance_{0.0};
    // Feedback126 §5 pairwise ownership: peers this drone must actively
    // avoid (deterministic ID-order owner tie-break).  Empty vector means
    // "no ownership information" and hard rows fall back to all peers.
    std::vector<int> swarm_adjuster_peers_;
    double moving_obj_lambda_{0.5};
    double moving_obj_prediction_horizon_{2.0};
    double moving_obj_time_decay_tau_{1.8};
    double moving_obj_max_grad_{3.0};
    double directional_visibility_sample_dt_{0.10};
    double weight_visibility_{20.0};
    /* 阶段 D：LOCAL_VIS 真实 runtime 证据（mutable：在 const cost 采样函数内累加）。
     * FULL 下必须 >0；NO_LOCAL_VIS 下门关闭，必须保持 0。 */
    mutable unsigned long local_vis_active_sample_count_{0};
    mutable unsigned long local_vis_nonzero_cost_count_{0};
    mutable unsigned long local_vis_nonzero_grad_count_{0};
    mutable double local_vis_last_log_time_s_{0.0};
    /* 2026-09-22 恢复:P/T 联合 hard SCP 参数与状态(默认关闭)。 */
    bool candidate_hard_corridor_scp_enabled_{false};
    double candidate_hard_corridor_scp_radius_{0.5};
    double candidate_hard_corridor_scp_trust_region_{0.25};
    double candidate_hard_corridor_scp_time_trust_region_{0.15};
    int candidate_hard_corridor_scp_max_iterations_{8};
    int candidate_hard_corridor_scp_samples_{25};
    double elastic_moving_risk_margin_{0.80};
    std::uint64_t scp_candidate_sequence_{0};
    double static_los_margin_{0.08};
    std::string static_los_scene_file_;
    double candidate_preserve_weight_{2.0};
    bool candidate_region_constraint_enabled_{true};
    double candidate_region_weight_{2.0};
    std::string candidate_region_type_{"quadratic"};
    bool prediction_epoch_override_enabled_{false};
    double prediction_epoch_override_{0.0};
    bool candidate_side_bias_enabled_{false};
    bool candidate_side_region_enabled_{false};
    Eigen::Vector3d candidate_side_origin_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d candidate_side_direction_{Eigen::Vector3d::Zero()};
    double candidate_side_sign_{0.0};
    double candidate_side_offset_{0.0};
    double candidate_side_weight_{2.0};
    double candidate_side_conflict_progress_{0.5};
    double candidate_side_guidance_window_{1.0};
    // Frozen after SIDE repair and PVA timing. This exact polynomial is the
    // sole geometry authority for the one-sided topology boundary.
    bool candidate_side_topology_reference_valid_{false};
    poly_traj::Trajectory candidate_side_topology_reference_;
    bool candidate_preserve_reference_valid_{false};
    poly_traj::Trajectory candidate_preserve_reference_;
    bool candidate_risk_window_valid_{false};
    double candidate_risk_conflict_time_{0.0};
    double candidate_risk_half_window_{0.0};
    // Predictive Visibility Relay is an event-triggered refinement of an
    // already feasible Local realization.  It is deliberately independent of
    // the historical candidate-hard-SCP switch above.
    bool team_spatiotemporal_scp_enabled_{true};
    double team_scp_p_trust_{0.30};
    double team_scp_t_trust_ratio_{0.15};
    int team_scp_max_iterations_{6};
    double team_scp_soft_weight_{10.0};
    double team_scp_t_gain_ratio_{0.80};
    double team_visibility_range_smoothing_{0.25};
    double team_visibility_los_smoothing_{0.15};
    bool enable_fov_costs_{false};
    bool fov_lock_tracking_height_;
    double fov_min_target_distance_, fov_max_target_distance_;
    double fov_distance_weight_, fov_height_weight_, fov_target_height_;
    double fov_angular_weight_, fov_angular_min_angle_;
    double fov_yaw_weight_, fov_half_angle_, fov_yaw_speed_min_;
    double fov_soft_scale_{1.0};
    Eigen::Vector3d relative_tracking_p_;
    Eigen::Vector3d object_p_;
    Eigen::Vector3d object_v_;
    Eigen::Quaterniond object_q_;
    bool visibility_yaw_valid_{false};
    double visibility_yaw_{0.0};
    double visibility_yaw_rate_{0.0};
    double visibility_camera_hfov_{85.0 * M_PI / 180.0};
    double visibility_camera_vfov_{54.53644224813771 * M_PI / 180.0};
    double visibility_camera_min_range_{0.20};
    double visibility_camera_max_range_{8.0};
    unsigned long visibility_context_generation_{0};
    int visibility_context_hypothesis_id_{0};

    double t_now_{0.0};
    OptimizationDiagnostics optimization_diagnostics_;
    CostGradientSnapshot current_cost_snapshot_;
    double current_static_grad_sq_{0.0};
    double current_moving_grad_sq_{0.0};
    double current_side_grad_sq_{0.0};
    double current_side_region_grad_sq_{0.0};
    double current_preserve_grad_sq_{0.0};
    double current_tracking_grad_sq_{0.0};
    double current_smoothness_grad_sq_{0.0};
    double current_variance_grad_sq_{0.0};
    double current_visibility_grad_sq_{0.0};
    bool gradient_audit_enabled_{false};
    std::string gradient_audit_candidate_;
    int gradient_audit_obstacle_id_{-1};
    bool gradient_audit_middle_logged_{false};
    std::string visibility_cost_candidate_{"NOMINAL"};
    std::string last_candidate_final_status_reason_;
    std::vector<LocalSfcPlane> candidate_local_sfc_planes_;
  public:
    PolyTrajOptimizer() {}
    ~PolyTrajOptimizer() {}

    enum CHK_RET
    {
      OBS_FREE,
      ERR,
      FINISH
    };

    enum class TeamSCPMode
    {
      AUTO = 0,
      TEAM_T_ONLY = 1,
      TEAM_PT = 2
    };

    /* Feedback098 — one sample of the Planner-side visibility forecast.
     * Limiter: 0=INVALID 1=STATIC 2=DYNAMIC_LOS 3=FOV 4=RANGE. */
    struct VisibilityTraceSample
    {
      double world_time{0.0};
      double margin{0.0};
      int limiter{0};
      bool valid{false};
    };
    struct BinaryVisibilityTraceSample
    {
      double world_time{0.0};
      bool visible{false};
    };

    struct TeamVisibilityContract
    {
      bool active{false};
      std::uint8_t repair_k{0};
      std::uint64_t contract_id{0};
      double activation_world_time{0.0};
      double acquire_by_world_time{0.0};
      double preserve_until_world_time{0.0};
      double contract_expire_world_time{0.0};
      double required_margin{0.0};
      int outgoing_uav{-1};
      int incoming_uav{-1};
      int stable_observer_uav{-1};
      TeamSCPMode requested_mode{TeamSCPMode::AUTO};
    };

    /* Geometry is sampled once from the Local initializer before L-BFGS;
     * objective evaluations use only fixed position guide points. */
    struct TeamVisibilityReserve
    {
      bool active{false};
      std::uint64_t contract_id{0};
      double activation_world_time{0.0};
      double reserve_begin_world_time{0.0};
      double reserve_end_world_time{0.0};
      double soft_margin{0.0};
      double weight{0.0};
    };
    struct TeamReserveGuide
    {
      double world_time{0.0};
      Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    };
    void setTeamVisibilityReserve(const TeamVisibilityReserve &reserve);
    void clearTeamVisibilityReserve(void);
    void setTeamVisibilityReserveActivation(const double activation_world_time);
    void prepareTeamVisibilityReserveGuides(const poly_traj::Trajectory &seed);
    bool teamVisibilityReserveActive(void) const
    {
      return team_visibility_reserve_.active;
    }
    double teamVisibilityReserveCost(void) const
    {
      return team_reserve_cost_;
    }
    int teamVisibilityReserveActiveSamples(void) const
    {
      return team_reserve_active_samples_;
    }
    int teamVisibilityReserveGuideCount(void) const
    {
      return static_cast<int>(team_reserve_guides_.size());
    }

    struct TeamSCPResult
    {
      bool attempted{false};
      bool success{false};
      TeamSCPMode mode{TeamSCPMode::AUTO};
      std::string reason;
      int visibility_rows{0};
      double margin_before{-std::numeric_limits<double>::infinity()};
      double margin_after{-std::numeric_limits<double>::infinity()};
      double linear_margin_prediction{-std::numeric_limits<double>::infinity()};
      double model_agreement{std::numeric_limits<double>::infinity()};
      double delta_p_norm{0.0};
      double delta_virtual_t_norm{0.0};
      double delta_real_t_norm{0.0};
      double max_velocity_before{0.0};
      double max_acceleration_before{0.0};
      double max_jerk_before{0.0};
      double max_velocity_after{0.0};
      double max_acceleration_after{0.0};
      double max_jerk_after{0.0};
      // Missing-gradient evidence: which world time and which visibility
      // component made the linearization unusable.  Diagnostic only; it never
      // fabricates a gradient for a binary/discrete component.
      double linearization_invalid_world_time{-1.0};
      std::string linearization_invalid_component{"NONE"};
    };

    /* set variables */
    void setParam(ros::NodeHandle &nh);
    void setEnvironment(const GridMap::Ptr &map);
    void setEnvironment(const GridMap::Ptr &map, const fast_planner::ObjPredictor::Ptr &moving_objs);
    void setControlPoints(const Eigen::MatrixXd &points);
    void setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr);
    void setDroneId(const int drone_id);
    void setIfTouchGoal(const bool touch_goal);
    void setObject(const Eigen::Vector3d &object_pt, const Eigen::Vector3d &object_vel, const Eigen::Quaterniond &object_q);
    void setRelativeTrackingP(const Eigen::Vector3d &rela_track_p);
    /* 阶段 C 接口拆分：soft encirclement reference 与 directional visibility
     * guidance 是两种语义，不再共用一个由 encirclement_generation>0 间接驱动
     * 的开关。
     *   - setSoftEncirclementReference：软合围参考（径向/高度带 + 弱方位恢复）
     *     是否生效 = launch 配置(encirclement_tracking_configured_) && enabled
     *     （enabled = production tracking mode / cooperative task 有效）。
     *   - setDirectionalVisibilityGuidance：Local MINCO directional J_vis 是否
     *     生效 = launch 配置 && ablation_config_.local_visibility && 有效
     *     visibility context（由调用方判定并传入）。绝不允许再由 Team 的
     *     encirclement_generation 决定 Local visibility objective 是否存在。 */
    void setSoftEncirclementReference(const bool enabled);
    void setDirectionalVisibilityGuidance(const bool enabled);
    bool isEncirclementTrackingConfigured() const { return encirclement_tracking_configured_; }
    void setVisibilityYawState(const bool valid, const double yaw,
                               const double yaw_rate,
                               const double horizontal_fov,
                               const double vertical_fov,
                               const double min_range,
                               const double max_range);
    void setVisibilityContextIdentity(const unsigned long generation,
                                      const int hypothesis_id);
    void setFovSoftScale(const double scale);
    /* 本轮修复 1/2：合围软方位恢复的权重与前缀加强。
     * span[s] 为真实执行前缀长度（按积分点的真实绝对时间判定，不再换算成
     * 采样点个数，因此 sample_time_step_seconds 仅保留用于日志/兼容）。 */
    void setEncirclementBearingRecovery(const double weight,
                                        const double prefix_boost,
                                        const double prefix_span_seconds,
                                        const double sample_time_step_seconds);
    void setConstraintPoints(ConstraintPoints cps);
    void setStartTracking(const bool satrt_tracking);
    /* =========================================================================
     * 本轮修复的数学一致性自检出口（只用于有限差分核对，不参与生产决策）
     *
     * 目的：证明 trackingGradCostP 返回的 cost 与 gradient 是同一个数学目标的
     * 真实导数——方位恢复项修复前多乘了一个 wei_tracking_。
     *   bearingGradientFiniteDifferenceCheck() 对每个控制点分量与每个时间自由度
     *   做中心差分，与解析梯度逐项比较并返回最大误差。
     *   configureTrackingGradientTest() 建立只读的测试环境。
     * 二者都不做求解、不改变生产语义。
     * ========================================================================= */
    struct TrackingGradientCheck
    {
      bool valid{false};
      std::string reason;
      double cost{0.0};
      double position_error_max{0.0};
      double position_error_relative_max{0.0};
      double time_error_max{0.0};
      double time_error_relative_max{0.0};
      double prefix_span{0.0};
      double weight{0.0};
      double wei_tracking{0.0};
      double boost{1.0};
      int pieces{0};
      int samples{0};
      int prefix_points{0};
      // 逐项诊断（只用于有限差分核对输出）
      double numeric_position[3]{0.0, 0.0, 0.0};
      double analytic_position[3]{0.0, 0.0, 0.0};
      double numeric_time[3]{0.0, 0.0, 0.0};
      double analytic_time[3]{0.0, 0.0, 0.0};
      int worst_position_index{0};
      int worst_time_index{0};
    };
    void configureTrackingGradientTest(const Eigen::Vector3d &object_p,
                                       const Eigen::Vector3d &object_v,
                                       const Eigen::Vector3d &relative_tracking,
                                       const double wei_tracking,
                                       const int cps_num_pre_piece);
    TrackingGradientCheck bearingGradientFiniteDifferenceCheck(
        const double bearing_weight, const double prefix_span_seconds,
        const double prefix_boost, const double perturbation);
    /* 单点解析梯度探针：按生产调用顺序重建轨迹/约束点后，取指定约束点的
     * 解析 cost 与 grad（用于把有限差分差异定位到具体积分点）。 */
    bool trackingGradientProbeAtPoint(
        const Eigen::Matrix3d &head, const Eigen::Matrix3d &tail,
        const Eigen::MatrixXd &inner_points, const Eigen::VectorXd &durations,
        const int cps_num_pre_piece, const int point_index, double &cost,
        Eigen::Vector3d &grad_p, double &grad_t, double &grad_prev_t,
        double &point_time);
    /* 按生产调用顺序重建轨迹与约束点（供有限差分使用，保证 analytic 与
     * numeric 在完全相同的 p/v 点上取值）。 */
    void rebuildTrackingGradientState(const Eigen::Matrix3d &head,
                                      const Eigen::Matrix3d &tail,
                                      const Eigen::MatrixXd &inner_points,
                                      const Eigen::VectorXd &durations,
                                      const int cps_num_pre_piece);
    /* 取当前 cps_ 中第 point_index 个约束点上的解析 cost/grad。 */
    bool trackingGradientAtPoint(const int point_index, double &cost,
                                 Eigen::Vector3d &grad_p, double &grad_t,
                                 double &grad_prev_t, double &point_time);
    /* 只计算第 point_index 个约束点在给定位置/速度/时刻上的 tracking 代价
     * （不返回梯度）。用于对 cost↔gradient 一致性做中心差分：
     *   gradp 必须等于 ∂costp/∂p（同一 p、v、t 点上）。
     * 这正是"方位项权重被多乘一次"会破坏的不变量。 */
    bool trackingCostAtPoint(const int point_index, const Eigen::Vector3d &p,
                             const Eigen::Vector3d &v, const double t,
                             double &cost);
    void setMovingObjPredictionEpoch(const double epoch);
    void clearMovingObjPredictionEpoch(void);
    void setMovingObjPredictionHorizon(const double horizon);
    void setCandidateSideBias(const Eigen::Vector3d &start,
                              const Eigen::Vector3d &end,
                              const int side,
                              const double offset,
                              const double conflict_progress = 0.5,
                              const double guidance_window = 1.0,
                              const bool enable_region_constraint = false);
    void setCandidateSideBiasReference(const Eigen::Vector3d &start,
                                       const Eigen::Vector3d &end,
                                       const Eigen::Vector3d &reference_direction,
                                       const int side,
                                       const double offset,
                                       const double conflict_progress = 0.5,
                                       const double guidance_window = 1.0,
                                       const bool enable_region_constraint = false);
    void setCandidatePreservationReference(const poly_traj::Trajectory &reference);
    bool setCandidateSideTopologyReference(const poly_traj::Trajectory &reference);
    void setCandidateRiskWindow(double conflict_time, double half_window);
    void setCandidateLocalSfc(const std::vector<LocalSfcPlane> &planes);
    void clearCandidateLocalSfc(void);
    void clearCandidateSideBias(void);
    void setTeamVisibilityContract(const TeamVisibilityContract &contract);
    void clearTeamVisibilityContract(void);
    /* Reusable visibility-margin authority.  visibilitySampleAt() is the
     * single implementation used by both the Team-SCP seed evaluation
     * (teamMarginAt) and the Planner-side forecast below, so the Coordinator
     * and the Local Planner cannot drift apart on the same quantity. */
    multi_uav_formation::ContinuousVisibilitySample visibilitySampleAt(
        const poly_traj::Trajectory &trajectory, const double trajectory_time,
        const double activation_world_time) const;
    /* Time-indexed absolute-world-time forecast of the Planner's own current
     * Local baseline.  Never extrapolates past the trajectory's own end. */
    bool evaluateVisibilityForecast(
        const poly_traj::Trajectory &trajectory,
        const double activation_world_time, const double horizon,
        const double sample_dt,
        std::vector<VisibilityTraceSample> &trace) const;
    bool evaluateBinaryVisibilityForecast(
        const poly_traj::Trajectory &trajectory,
        double activation_world_time, double horizon, double sample_dt,
        std::vector<BinaryVisibilityTraceSample> &trace) const;
    /* Single world-time margin query on the same authority. */
    bool evaluateVisibilityMarginAt(
        const poly_traj::Trajectory &trajectory,
        const double activation_world_time, const double world_time,
        double &margin, int &limiter) const;

    bool runTeamContractSCP(
        const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
        const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
        Eigen::MatrixXd &optimal_points, double &final_cost,
        TeamSCPResult &result);
    bool teamSpatiotemporalSCPEnabled() const
    {
      return team_spatiotemporal_scp_enabled_;
    }
    // Feedback100: read-only access to the trust scales the Team-SCP itself
    // uses, so a repair trial can normalize its P and T deformation by the SAME
    // scales.  This exists only to make two candidate trials comparable; it
    // changes no tuning value and no solver behaviour.
    void teamSCPTrustScale(double &position_trust, double &time_trust) const
    {
      position_trust = std::max(1.0e-9, team_scp_p_trust_);
      time_trust = std::max(1.0e-9, team_scp_t_trust_ratio_);
    }
    void setGradientAuditContext(const std::string &candidate_type, int obstacle_id);
    void clearGradientAuditContext(void);
    bool queryStaticLosClearance(const Eigen::Vector3d &observer,
                                 const Eigen::Vector3d &target,
                                 double &clearance,
                                 Eigen::Vector3d *gradient_observer = nullptr,
                                 Eigen::Vector3d *gradient_target = nullptr) const;
    bool queryStaticLosClearance(const Eigen::Vector3d &observer,
                                 const Eigen::Vector3d &target,
                                 double &clearance,
                                 StaticLosWitness &witness,
                                 Eigen::Vector3d *gradient_observer = nullptr,
                                 Eigen::Vector3d *gradient_target = nullptr) const;
    /* K3 统一 Local visibility sample 复用的平滑/阈值常量(只读;同一常量,
     * 不新增第二套阈值)。 */
    double visibilityLosSmoothing() const { return team_visibility_los_smoothing_; }
    double visibilityRangeSmoothing() const { return team_visibility_range_smoothing_; }
    double visibilityStaticLosMargin() const { return static_los_margin_; }
    void evaluateCandidateSideRegion(const poly_traj::Trajectory &traj,
                                     double &rms_violation,
                                     double &max_violation) const;
    bool checkMovingObjSafety(const poly_traj::Trajectory &traj,
                              const double prediction_start_time,
                              std::string &reason) const;
    bool evaluateCandidateDynamicsLattice(
        const poly_traj::Trajectory &traj,
        CandidateDynamicsSummary &summary) const;

    /* =====================================================================
     * Feedback126 unified physical safety kernel (§2/§3).
     *
     * One physical predicate per constraint family, shared by the generation
     * layer (LBFGS acceptance gate + SCP hard rows) and the validation layer
     * (candidate classify / final preflight).  No second threshold, no
     * trigger-margin gate, no soft-buffer acceptance may declare a candidate
     * safe or unsafe any more.
     *
     * SWARM:  elliptical dxy^2 + 0.25*dz^2 >= swarm_clearance_^2 against the
     *         broadcast peer polynomial over the same authority horizon used
     *         by validation, polynomial-then-hold@end peer model, world time
     *         anchored at the batch prediction epoch.
     * PVAJ:   the same sampled max v/a/j as the manager checker, with the
     *         same limit × (1 + feasibility_tolerance) definition.
     * DYNAMIC: per-object hard clearance (body radius + that object's live
     *         radius), never the fleet-max approximation.
     * STATIC: the inflated map itself (finelyCheck / checker already share
     *         it); the corridor rows certify the subset that was built.
     * ===================================================================== */
    struct UnifiedSwarmFinding
    {
      bool violated{false};
      int peer_drone_id{-1};
      double conflict_time{0.0};
      double ellip_dist2{0.0};
    };
    /* Pairwise-adjuster ownership (§5).  Peers listed here are the ones this
     * drone must actively avoid with hard rows / constrained solves (this
     * drone loses the deterministic ID-order ownership tie).  Peers NOT in
     * the list are owned by this drone for their pair: their committed
     * trajectory is the fixed reference, generation must not add opposite
     * avoidance rows for them.  Validation always checks every peer. */
    void setSwarmAdjusterPeers(const std::vector<int> &peer_ids)
    {
      swarm_adjuster_peers_ = peer_ids;
    }
    const std::vector<int> &swarmAdjusterPeers(void) const
    {
      return swarm_adjuster_peers_;
    }
    bool swarmPhysicalViolation(const poly_traj::Trajectory &traj,
                                const double start_time, const bool touch_goal,
                                UnifiedSwarmFinding *finding = nullptr) const;
    bool pvaPhysicalViolation(const poly_traj::Trajectory &traj, double *max_v,
                              double *max_a, double *max_j) const;
    bool dynamicPhysicalViolation(const poly_traj::Trajectory &traj,
                                  const double start_time,
                                  int *violating_object = nullptr) const;
    double movingObjHardClearanceForObject(const int object_id) const;
    double evaluateCandidateLocalSfcMaxViolation(
        const poly_traj::Trajectory &traj) const;

    /* Feedback119(连续 corridor violation):一个 corridor 平面对五次多项式
     * 的标量函数 f(t)=a^T·p(t)-b 在其 active 区间上的最大违反可以解析求得:
     * 逐段检查区间端点与 f'(t)=0 的实根,不需要任何暴力采样。
     * worst_per_plane 非空时同时输出每个 STATIC corridor 平面的最坏违反点
     * (全局轨迹时间),供 SCP active-set 只追加真实违反点 t*。 */
    struct CorridorViolationPoint
    {
      size_t plane_index{0};
      double time{0.0};
      double violation{0.0};
    };
    double continuousLocalSfcMaxViolation(
        const poly_traj::Trajectory &traj,
        std::vector<CorridorViolationPoint> *worst_per_plane) const;
    double continuousLocalSfcMaxViolation(
        const poly_traj::Trajectory &traj,
        const std::vector<LocalSfcPlane> &planes,
        std::vector<CorridorViolationPoint> *worst_per_plane) const;
    /* Return all verified real roots of a quartic on the closed interval. */
    static std::vector<double> quarticRealRoots(
        const Eigen::Matrix<double, 5, 1> &c, double lo, double hi);

    /* Feedback117(authority 整合):LOS observation plane 是 visibility
     * authority 的 semi-hard slack 行。本函数现在对任何结果都返回 true——
     * 残余 violation 只进 LosSoftPlaneTelemetry,绝不 reject 候选;
     * 执行安全由 collision/SFC/动力学硬检查与 current-revision preflight
     * 独占。函数永远不拥有 safety reject authority。 */
    bool enforceCandidateLosPlanesSCP(
        const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
        Eigen::MatrixXd &optimal_points);

    /* 2026-09-22 恢复(核心语义回归):P/T 联合 hard SCP。SIDE 候选在
     * candidate_hard_corridor_scp_enabled_ 打开时用它替代 LBFGS:同一
     * free-time MINCO 参数化,SCP 层加 corridor/SFC/static/v-a-j/动态 body
     * 硬行与 P/T 双信任盒。恢复自 09-18 快照;默认关闭。
     * Feedback117:本函数的 Local-SFC 硬行集合只包含
     * STATIC_COLLISION_CORRIDOR 源平面;LOS_OBSERVATION_SIDE 平面被
     * 显式排除(由 enforceCandidateLosPlanesSCP 的 slack QP 独占),
     * 杜绝"LOS 行在第二条路径里仍然是硬约束"的重复 authority。
     * Feedback119(corridor 合同):A*-repair SIDE 的 MINCO seed 只是数值
     * 初值,允许切角进入本函数。Local-SFC 硬行改为 active-set 协议:
     * 初始每平面仅少量必要采样点(端点+中点+seed 连续违反 argmax),之后
     * 每轮 SCP 用五次多项式连续 violation 检查把真实违反点 t* 追加进下一
     * 轮行集;trial 验收对 corridor 项改为单调不恶化,绝对干净由
     * LOCAL_SFC_FINAL_VIOLATION(连续 ≤2e-3)在出口强制。禁止恢复
     * 每 plane×25 均匀采样的暴力行扩张。 */
    bool runCandidateHardCorridorSCP(
        const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
        const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
        Eigen::MatrixXd &optimal_points, double &final_cost);

    TeamVisibilityContract team_visibility_contract_;
    /* Feedback101 reserve state.  Refreshed on every schedule and re-anchored
     * to the current Local trajectory start before each Local optimization;
     * no persistent state machine is added. */
    TeamVisibilityReserve team_visibility_reserve_;
    std::vector<TeamReserveGuide> team_reserve_guides_;
    mutable double team_reserve_cost_{0.0};
    mutable int team_reserve_active_samples_{0};
    TeamSCPResult team_scp_result_;
    // Index (0=STATIC,1=DYNAMIC_LOS,2=FOV,3=RANGE;-1=none) of the visibility
    // component that had no usable continuous representation at the last
    // teamMarginAt() sample.  Diagnostic only: it lets a failed linearization
    // name the missing evidence instead of reporting a generic reason.
    mutable int team_margin_invalid_component_{-1};
    bool team_contract_scp_active_{false};

    /* helper functions */
    inline const ConstraintPoints &getControlPoints(void) { return cps_; }
    inline double getMovingObjPredictionStartTime(void) const { return t_now_; }
    inline double getMovingObjPredictionHorizon(void) const { return moving_obj_prediction_horizon_; }
    inline double getMovingObjClearance(void) const { return moving_obj_clearance_; }
    double getMovingObjHardClearance(void) const;
    inline const std::string &getCandidateRegionType(void) const
    {
      return candidate_region_type_;
    }
    inline const poly_traj::MinJerkOpt &getMinJerkOpt(void) { return jerkOpt_; }
    inline int get_cps_num_prePiece_(void) { return cps_num_prePiece_; }
    inline double get_swarm_clearance_(void) { return swarm_clearance_; }
    inline double getFeasibilityWeight(void) const { return wei_feas_; }
    inline void setFeasibilityWeight(const double weight) { wei_feas_ = weight; }
    inline double getTimeWeight(void) const { return wei_time_; }
    inline void setTimeWeight(const double weight) { wei_time_ = weight; }
    inline double getObstacleWeight(void) const { return wei_obs_; }
    inline void setObstacleWeight(const double weight) { wei_obs_ = weight; }
    inline double getObstacleSoftWeight(void) const { return wei_obs_soft_; }
    inline void setObstacleSoftWeight(const double weight) { wei_obs_soft_ = weight; }
    inline double getMovingObjWeight(void) const { return moving_obj_lambda_; }
    inline void setMovingObjWeight(const double weight) { moving_obj_lambda_ = weight; }
    inline double getSwarmWeight(void) const { return wei_swarm_; }
    inline void setSwarmWeight(const double weight) { wei_swarm_ = weight; }
    inline const OptimizationDiagnostics &getLastOptimizationDiagnostics(void) const
    {
      return optimization_diagnostics_;
    }
    inline const std::vector<LocalSfcPlane> &getCandidateLocalSfc(void) const
    {
      return candidate_local_sfc_planes_;
    }
    /* Feedback119:最近一次 runCandidateHardCorridorSCP 的 corridor 合同
     * 遥测。由 manager 在 optimizeTrajectory 之后立即读取,用于
     * CORRIDOR_SCP_INFEASIBLE / CONTINUOUS_CORRIDOR_VIOLATION 计数;
     * Team 路径(runTeamContractSCP)的遥测会被后续调用覆盖,manager 只在
     * 自己的调用点之后读取,不构成跨路径共享状态。 */
    struct CorridorScpTelemetry
    {
      bool attempted{false};
      bool success{false};
      int iterations{0};
      int active_point_count{0};
      int added_point_count{0};
      double final_continuous_violation{0.0};
      double final_sampled_violation{0.0};
      std::string reason;
    };
    inline const CorridorScpTelemetry &lastCorridorScpTelemetry(void) const
    {
      return last_corridor_scp_telemetry_;
    }
    inline double getObsClearance(void) const { return obs_clearance_; }
    inline const std::string &getLastCandidateFinalStatusReason(void) const
    {
      return last_candidate_final_status_reason_;
    }
    inline int getLastLBFGSStatus(void) const
    {
      return last_lbfgs_status_;
    }

    /* main planning API */
    struct ExecutionDeadlineExceeded {};
    void executionCheckpoint() const {
      if(!executionBudgetAvailable()) throw ExecutionDeadlineExceeded();
    }
    void setExecutionDeadline(double deadline,double call_reserve) {
      execution_deadline_=deadline;execution_call_reserve_=call_reserve;
    }
    bool executionBudgetAvailable() const {
      return ros::WallTime::now().toSec()+execution_call_reserve_<execution_deadline_;
    }
    SCPQPSolveResult solveExecutionQP(const Eigen::VectorXd &g,const Eigen::MatrixXd &A,
        const Eigen::VectorXd &lo,const Eigen::VectorXd &hi,const Eigen::VectorXd &h) const {
      if(!executionBudgetAvailable()) { SCPQPSolveResult result;result.status_text="EXECUTION_DEADLINE";return result; }
      return SCPOptimizer::solve(g,A,lo,hi,h,std::isfinite(execution_deadline_)?
          std::max(0.001,execution_deadline_-ros::WallTime::now().toSec()-0.01):0.0);
    }
    SCPQPSolveResult solveExecutionQP(const Eigen::VectorXd &g,const Eigen::MatrixXd &A,
        const Eigen::VectorXd &lo,const Eigen::VectorXd &hi,double h) const {
      return solveExecutionQP(g,A,lo,hi,Eigen::VectorXd::Constant(g.size(),h));
    }
    /* LOS observation-plane semi-hard QP(2026-09-25 第六轮):前 soft_rows 行
     * 允许非负 slack ξ_k(m_s·Δp + ξ_k >= bound,ξ_k >= 0),代价
     * w1·Σξ_k + 0.5·w2·Σξ_k²;其余行保持硬。ξ=0 时与原 hard QP 完全一致。
     * 返回的 step 仍只含原 n 个决策变量;slack 统计写入 los_slack 统计结构。
     * w1=0/w2<=0 退化为硬 QP。 */
    SCPQPSolveResult solveExecutionQPWithSlack(
        const Eigen::VectorXd &g, const Eigen::MatrixXd &A,
        const Eigen::VectorXd &lo, const Eigen::VectorXd &hi,
        const Eigen::VectorXd &hessian_diagonal, int soft_rows, double slack_w1,
        double slack_w2, double &slack_max, double &slack_sum) const;
    /* 最近一次 enforceCandidateLosPlanesSCP 的 slack 遥测(Feedback116)。 */
    struct LosSoftPlaneTelemetry
    {
      bool attempted{false};
      bool qp_success{false};
      bool satisfied{false};
      int iteration_count{0};
      int sample_count{0};
      int slack_nonzero_count{0};
      double slack_max{0.0};
      double slack_mean{0.0};
      double slack_cost{0.0};
      std::string qp_status;
    };
    LosSoftPlaneTelemetry last_los_soft_plane_telemetry_;
    CorridorScpTelemetry last_corridor_scp_telemetry_;
    double los_observation_slack_weight_{1.0e3};
    const LosSoftPlaneTelemetry &lastLosSoftPlaneTelemetry() const
    {
      return last_los_soft_plane_telemetry_;
    }
    bool optimizeTrajectoryWithinBudget(const Eigen::MatrixXd &iniState,const Eigen::MatrixXd &finState,
        const Eigen::MatrixXd &initInnerPts,const Eigen::VectorXd &initT,
        Eigen::MatrixXd &optimal_points,double &final_cost);
    bool optimizeTrajectory(const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
                            const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
                            Eigen::MatrixXd &optimal_points, double &final_cost);
    bool computePointsToCheck(poly_traj::Trajectory &traj, int id_end, PtsChk_t &pts_check, bool full_trajectory);
    static bool samplePointsToCheck(poly_traj::Trajectory &traj,int id_end,PtsChk_t &pts_check,
        bool full_trajectory,double resolution,double max_velocity,int samples_per_piece);

    std::vector<std::pair<int, int>> finelyCheckConstraintPointsOnly(Eigen::MatrixXd &init_points);

    /* check collision and set {p,v} pairs to constraint points */
    CHK_RET finelyCheckAndSetConstraintPoints(std::vector<std::pair<int, int>> &segments,
                                              const poly_traj::MinJerkOpt &pt_data,
                                              const bool flag_first_init /*= true*/,
                                              std::string *reason = nullptr);

    bool roughlyCheckConstraintPoints(void);

    /* multi-topo support */
    std::vector<ConstraintPoints> distinctiveTrajs(vector<std::pair<int, int>> segments);

  private:
    using CandidateDynamicsSampleVisitor = std::function<void(
        int, int, int, double, double, const Eigen::Vector3d &,
        const Eigen::Vector3d &, const Eigen::Vector3d &)>;

    bool forEachCandidateDynamicsSample(
        const poly_traj::Trajectory &traj,
        const CandidateDynamicsSampleVisitor &visitor) const;
    static double realTimeJacobianAt(const double virtual_t);
    Eigen::RowVectorXd mincoSampleGradientWrtDecision(
        const int piece_index, const double piece_time,
        const Eigen::Vector3d &grad_sample, const int derivative_order,
        const Eigen::VectorXd &ds_dT,
        const Eigen::VectorXd &current_virtual_t);

    /* callbacks by the L-BFGS optimizer */
    static double costFunctionWithinBudget(void *func_data, const double *x, double *grad, const int n);
    static double costFunctionCallback(void *func_data, const double *x, double *grad, const int n);

    static int earlyExitCallback(void *func_data, const double *x, const double *g,
                                 const double fx, const double xnorm, const double gnorm,
                                 const double step, int n, int k, int ls);


    /* mappings between real world time and unconstrained virtual time */
    template <typename EIGENVEC>
    void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

    template <typename EIGENVEC>
    void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

    template <typename EIGENVEC, typename EIGENVECGD>
    void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,
                          const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                          double &costT);

    /* gradient and cost evaluation functions */
    template <typename EIGENVEC>
    void initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost);

    template <typename EIGENVEC>
    void addPVAGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K);

    struct LiveVisibilityCost
    {
      bool valid{false};
      double weighted_cost{0.0};
      double static_los_cost{0.0};
      double dynamic_los_cost{0.0};
      double fov_cost{0.0};
      Eigen::Vector3d gradient_position{Eigen::Vector3d::Zero()};
      Eigen::Vector3d static_gradient_position{Eigen::Vector3d::Zero()};
      Eigen::Vector3d dynamic_gradient_position{Eigen::Vector3d::Zero()};
      Eigen::Vector3d fov_gradient_position{Eigen::Vector3d::Zero()};
      double gradient_time{0.0};
      double gradient_previous_time{0.0};
      double static_time_derivative{0.0};
      double dynamic_time_derivative{0.0};
      double fov_time_derivative{0.0};
      double static_clearance{std::numeric_limits<double>::infinity()};
      double dynamic_clearance{std::numeric_limits<double>::infinity()};
      double horizontal_fov_margin{std::numeric_limits<double>::infinity()};
      double vertical_fov_margin{std::numeric_limits<double>::infinity()};
      double static_risk{0.0};
      double dynamic_risk{0.0};
      double fov_risk{0.0};
      int static_witness{-1};
      int dynamic_witness{-1};
      double static_compute_ms{0.0};
      double dynamic_compute_ms{0.0};
      double fov_compute_ms{0.0};
      bool static_los_gradient_active{false};
      bool dynamic_los_gradient_active{false};
      bool fov_gradient_active{false};
    };

    LiveVisibilityCost directionalVisibilityGradCostP(
        const double t, const Eigen::Vector3d &p,
        const Eigen::Vector3d &v, const bool predicted_yaw_valid,
        const double predicted_yaw, const double predicted_yaw_rate) const;

    template <typename EIGENVEC>
    void addDirectionalVisibilityGradCost2CT(EIGENVEC &gdT,
                                             double &visibility_cost);
    /* Fixed guide points: no visibility or LOS query in this cost path. */
    template <typename EIGENVEC>
    void addTeamVisibilityReserveGradCost2CT(EIGENVEC &gdT,
                                             double &reserve_cost);


    void logDirectionalVisibilityCost(void) const;

    bool obstacleGradCostP(const int i_dp,
                           const Eigen::Vector3d &p,
                           Eigen::Vector3d &gradp,
                           double &costp);

    bool swarmGradCostP(const int i_dp,
                        const double t,
                        const Eigen::Vector3d &p,
                        const Eigen::Vector3d &v,
                        Eigen::Vector3d &gradp,
                        double &gradt,
                        double &grad_prev_t,
                        double &costp);

    bool movingObjGradCostP(const int i_dp,
                            const double t,
                            const Eigen::Vector3d &p,
                            const Eigen::Vector3d &v,
                            Eigen::Vector3d &gradp,
                            double &gradt,
                            double &grad_prev_t,
                            double &costp);

    bool candidateSideGradCostP(const double progress,
                                const Eigen::Vector3d &p,
                                Eigen::Vector3d &gradp,
                                double &costp);

    bool candidateSideRegionGradCostP(const double progress,
                                      const Eigen::Vector3d &p,
                                      Eigen::Vector3d &gradp,
                                      double &costp) const;
    bool candidatePreservationGradCostP(const double progress,
                                        const Eigen::Vector3d &p,
                                        Eigen::Vector3d &gradp,
                                        double &costp) const;

    double candidateSideRegionViolation(const double progress,
                                        const Eigen::Vector3d &p) const;
    double candidateSideTopologyLowerBound(const double progress) const;
    /* 本轮修复（prefix 真实时间语义）：elapsed_t 是该积分点的真实绝对轨迹
     * 时间（前面所有 piece 的真实时长之和 + 该 piece 内局部时间），用于判定
     * "是否属于执行前缀"。t 仍是同一积分点的世界相对时间，用于目标/障碍预测。
     * 二者只在 retime 后不等价时才有区别，因此保留两个独立入参。 */
    bool trackingGradCostP(const int i_dp,
                           const double t,
                           const Eigen::Vector3d &p,
                           const Eigen::Vector3d &v,
                           Eigen::Vector3d &gradp,
                           double &gradt,
                           double &grad_prev_t,
                           double &costp,
                           const double elapsed_t = -1.0);

    double fovObstacleSafetyScale(const Eigen::Vector3d &p) const;

    bool angularGradCostP(const int i_dp,
                          const double t,
                          const Eigen::Vector3d &p,
                          const Eigen::Vector3d &v,
                          Eigen::Vector3d &gradp,
                          double &gradt,
                          double &grad_prev_t,
                          double &costp);

    bool yawGradCostPV(const int i_dp,
                       const double t,
                       const Eigen::Vector3d &p,
                       const Eigen::Vector3d &v,
                       const Eigen::Vector3d &a,
                       Eigen::Vector3d &gradp,
                       Eigen::Vector3d &gradv,
                       double &gradt,
                       double &grad_prev_t,
                       double &costp);

    bool feasibilityGradCostV(const Eigen::Vector3d &v,
                              Eigen::Vector3d &gradv,
                              double &costv);

    bool feasibilityGradCostA(const Eigen::Vector3d &a,
                              Eigen::Vector3d &grada,
                              double &costa);

    bool feasibilityGradCostJ(const Eigen::Vector3d &j,
                              Eigen::Vector3d &gradj,
                              double &costj);

    void distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                           Eigen::MatrixXd &gdp,
                                           double &var);

    void lengthVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                      const int n,
                                      Eigen::MatrixXd &gdp,
                                      double &var);

  public:
    friend struct TeamReferenceContract;
    typedef unique_ptr<PolyTrajOptimizer> Ptr;
  };

} // namespace ego_planner
#endif
