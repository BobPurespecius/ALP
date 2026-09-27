#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <optimizer/poly_traj_optimizer.h>
#include <traj_utils/DataDisp.h>
#include <plan_env/grid_map.h>
#include <plan_env/obj_predictor.h>
#include <traj_utils/plan_container.hpp>
#include <traj_utils/trajectory_lifecycle.h>
#include <traj_utils/ablation_runtime.h>
#include <traj_utils/TopologyCandidateBundle.h>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/TopologyCandidate.h>
#include <traj_utils/TopologyCoordination.h>
#include <traj_utils/TeamTrajectoryAck.h>
#include <traj_utils/TeamTrajectorySolution.h>
#include <traj_utils/TeamReferenceSchedule.h>
#include <traj_utils/CooperativeTaskReference.h>
#include <traj_utils/TopologyEscalation.h>
#include <multi_uav_formation/tracking_visibility_geometry.h>
#include <multi_uav_formation/team_planning_context.h>
#include <plan_manage/local_execution_contract.h>
#include <ros/ros.h>
#include <cstdint>
#include <traj_utils/planning_visualization.h>
#include <optimizer/poly_traj_utils.hpp>
#include <string>
#include <limits>
#include <array>
#include <deque>
#include <map>
#include <nav_msgs/Odometry.h>

namespace ego_planner
{

  // K3 统一 Local visibility sample(2026-09-25 第三轮 authority 清理)。
  // K3 检测、C3、D3、binary 恢复判定共用这唯一原语:binary = range ∧ static LOS
  // ∧ dynamic LOS ∧ FOV(与 executed truth 同口径);margin/risks 由同一份分量经
  // composeVisibilityMargin 合成。函数体只复用既有查询权威,不新增第二套
  // LOS/FOV/range 几何。
  struct LocalVisibilitySample
  {
    bool valid{false};
    bool range_visible{false};
    bool static_los_visible{false};
    bool dynamic_los_visible{false};
    bool fov_visible{false};
    bool binary_visible{false};
    // margin/risks/limiter 复用 tracking_visibility_geometry.h 的既有类型。
    multi_uav_formation::ContinuousVisibilitySample continuous;
  };

  // K3 窗口指标(C3/K2/D3)来自同一 LocalVisibilitySample 序列。
  struct LocalK3WindowMetrics
  {
    bool valid{false};
    int sample_count{0};
    int margin_sample_count{0};
    double c3{0.0};
    double k2{0.0};
    double d3{0.0};
  };

  // 唯一的 target 世界时间锚定公式:target(t_world) =
  // target_at_epoch + v · (t_world − target_epoch)。
  inline Eigen::Vector3d localK3TargetAtWorldTime(
      const Eigen::Vector3d &target_at_epoch,
      const Eigen::Vector3d &target_velocity, const double target_epoch,
      const double world_time)
  {
    return target_at_epoch + target_velocity *
        std::max(0.0, world_time - target_epoch);
  }

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class EGOPlannerManager
  {
    // SECTION stable
  public:
    enum ConflictReasonMask
    {
      CONFLICT_NONE = 0,
      CONFLICT_BODY_SAFETY = 1,
      CONFLICT_LOS_OCCLUSION = 2
    };

    enum class ConflictObstacleMotion
    {
      UNKNOWN = 0,
      STATIC = 1,
      DYNAMIC = 2
    };

    struct ConflictDescriptor
    {
      bool valid{false};
      unsigned int reason_mask{CONFLICT_NONE};
      ConflictObstacleMotion obstacle_motion{ConflictObstacleMotion::UNKNOWN};
      int obstacle_identity{-1};
      int primitive_type{0};
      std::string obstacle_name;
      unsigned long geometry_revision{0};
      double source_snapshot_time{0.0};
      double first_risk_world_time{0.0};
      double risk_interval_start{0.0};
      double risk_interval_end{0.0};
      // BODY and LOS evidence are intentionally kept separate.  The legacy
      // fields below describe the primary topology witness, while these
      // fields preserve both causes when one event is simultaneously a body
      // safety conflict and a line-of-sight occlusion.
      double body_conflict_time{-1.0};
      double los_conflict_time{-1.0};
      double body_first_risk_world_time{-1.0};
      double los_first_risk_world_time{-1.0};
      int body_obstacle_identity{-1};
      int los_obstacle_identity{-1};
      ConflictObstacleMotion body_obstacle_motion{ConflictObstacleMotion::UNKNOWN};
      ConflictObstacleMotion los_obstacle_motion{ConflictObstacleMotion::UNKNOWN};
      int los_primitive_type{0};
      Eigen::Vector3d body_obstacle_position{Eigen::Vector3d::Zero()};
      Eigen::Vector3d los_obstacle_position{Eigen::Vector3d::Zero()};
      Eigen::Vector3d observer_position{Eigen::Vector3d::Zero()};
      Eigen::Vector3d target_position{Eigen::Vector3d::Zero()};
      Eigen::Vector3d obstacle_position{Eigen::Vector3d::Zero()};
      Eigen::Vector3d obstacle_geometry{Eigen::Vector3d::Zero()};
      Eigen::Vector3d frame_forward{Eigen::Vector3d::Zero()};
      Eigen::Vector3d frame_right{Eigen::Vector3d::Zero()};
      Eigen::Vector3d clearance_witness{Eigen::Vector3d::Zero()};
      double clearance{std::numeric_limits<double>::infinity()};
      double radius{0.0};
    };

    struct DynamicRiskInfo
    {
      bool valid{false};
      bool triggered{false};
      // `triggered` denotes violation of the preferred avoidance margin;
      // hard_collision is reserved for actual body/obstacle overlap.
      bool hard_collision{false};
      int obstacle_id{-1};
      double min_distance{std::numeric_limits<double>::infinity()};
      double hard_clearance{std::numeric_limits<double>::infinity()};
      double trigger_distance{0.0};
      double conflict_time{0.0};
      double prediction_query_time{0.0};
      Eigen::Vector3d trajectory_position{Eigen::Vector3d::Zero()};
      Eigen::Vector3d obstacle_position{Eigen::Vector3d::Zero()};
      double relative_radial_velocity{0.0};
    };

    enum class CandidateKind
    {
      NOMINAL,
      SIDE_PLUS,
      SIDE_MINUS
    };

    enum class CandidateSafetyClass
    {
      ABSOLUTE_SAFE,
      INVALID
    };

    struct CandidateVisibilityReport
    {
      bool valid{false};
      int sample_count{0};
      std::array<double, 3> uav_visibility{{0.0, 0.0, 0.0}};
      double all3_visibility{0.0};
      double atleast2_visibility{0.0};
      double none_visibility{0.0};
      double min_uav_visibility{0.0};
      double max_loss_duration{0.0};
      double mean_visible_count{0.0};
      double atleast_k_visibility{0.0};
      double min_pairwise_angle_deg{0.0};
      double mean_pairwise_angle_deg{0.0};
      double diversity_score{0.0};
      double team_utility{0.0};
      // K3 hint-window 累计字段已退役(第三轮 authority 清理):K3 的 C3/K2/D3
      // 统一由 evaluateLocalK3WindowMetrics 在 LocalVisibilitySample 原语上计算,
      // 结果挂在 CandidateResult::k3_window,不再混入全局 comparator 的报告。
    };

    struct SwarmConflictInfo
    {
      bool valid{false};
      bool triggered{false};
      bool temporal_owner{false};
      int other_drone_id{-1};
      int sample_count{0};
      double min_distance{std::numeric_limits<double>::infinity()};
      double trigger_distance{0.0};
      double conflict_time{0.0};
      double window_start{0.0};
      double window_end{0.0};
    };

    struct CandidateResult
    {
      int candidate_id{-1};
      int encirclement_hypothesis_id{0};
      double encirclement_phi0{0.0};
      unsigned long encirclement_generation{0};
      bool success{false};
      CandidateKind kind{CandidateKind::NOMINAL};
      poly_traj::MinJerkOpt min_jerk_opt;
      ConstraintPoints constraint_points;
      Eigen::MatrixXd display_points;
      double optimization_cost{0.0};
      double prediction_epoch{0.0};
      unsigned long execution_revision{0};
      std::uint64_t validated_payload_hash{0};
      double checked_until{0.0};
      bool execution_target_context_valid{false};
      Eigen::Vector3d execution_target_position{Eigen::Vector3d::Zero()};
      Eigen::Vector3d execution_target_velocity{Eigen::Vector3d::Zero()};
      double execution_target_state_time{0.0};
      DynamicRiskInfo risk;
      ConflictDescriptor conflict;
      CandidateSafetyClass safety_class{CandidateSafetyClass::INVALID};
      CandidateVisibilityReport visibility;
      // K3-only 窗口指标(统一 LocalVisibilitySample 原语;仅 escalation 活跃
      // 时计算,不参与普通 comparator)。
      LocalK3WindowMetrics k3_window;
      LocalGeometryMetrics geometry;
      bool execution_prepared{false};
      double binary_camera_time{-1.0};
      bool existing_checks_passed{false};
      bool static_valid{false};
      bool dynamics_valid{false};
      bool swarm_valid{false};
      bool astar_used{false};
      // Discrete-topology provenance only. The normal current-revision hard
      // preflight still owns executability.
      bool target_side_static_topology{false};
      bool local_sfc_handoff_valid{false};
      bool local_sfc_used{false};
      std::vector<LocalSfcPlane> local_sfc_planes;
      bool feasible_initializer_fallback{false};
      // 本轮 P1：该候选所依据的真实 LOS 遮挡区间（相对本候选 activation，单位 s）。
      // enter<0 表示本次没有可用的遮挡区间证据。
      double los_occlusion_enter_time{-1.0};
      double los_occlusion_exit_time{-1.0};
      double los_side_reach_deadline{-1.0};
      bool los_observation_plane_created{false};
      bool los_reached_side{false};
      // Pre-final MINCO initializer exported to the asynchronous team quality
      // path.  This object has no execution authority; only the joint result's
      // current-revision hard preflight may create a proposal.
      bool joint_seed_valid{false};
      bool joint_seed_static_constructible{false};
      bool joint_seed_local_sfc_valid{false};
      double joint_seed_construction_ms{0.0};
      double joint_seed_dynamic_clearance{-1.0};
      poly_traj::MinJerkOpt joint_seed;
      double max_velocity{0.0};
      double max_acceleration{0.0};
      double max_jerk{0.0};
      double swarm_clearance{std::numeric_limits<double>::infinity()};
      double tracking_score{0.0};
      // Candidate-only geometry diagnostics; these do not affect selection.
      double init_nominal_max_lateral_deviation{-1.0};
      double init_nominal_conflict_lateral_deviation{-1.0};
      double final_nominal_max_lateral_deviation{-1.0};
      double final_init_max_lateral_deviation{-1.0};
      double final_nominal_conflict_lateral_deviation{-1.0};
      double final_init_conflict_lateral_deviation{-1.0};
      double local_guidance_window{-1.0};
      double local_side_conflict_progress{-1.0};
      double local_side_offset{-1.0};
      Eigen::Vector3d local_side_direction{Eigen::Vector3d::Zero()};
      double candidate_clearance_gain{-std::numeric_limits<double>::infinity()};
      bool acceptance_checked{false};
      bool acceptance_accepted{false};
      PolyTrajOptimizer::OptimizationDiagnostics minco_diagnostics;
    };

    struct CandidateSetOutput
    {
      bool generated{false};
      int hypothesis_id{0};
      double phi0{0.0};
      unsigned long encirclement_generation{0};
      int local_candidate_id{-1};
      Eigen::Vector3d relative_tracking{Eigen::Vector3d::Zero()};
      Eigen::Vector3d target_position{Eigen::Vector3d::Zero()};
      Eigen::Vector3d target_velocity{Eigen::Vector3d::Zero()};
      Eigen::Vector3d visibility_target_position{Eigen::Vector3d::Zero()};
      Eigen::Vector3d visibility_target_velocity{Eigen::Vector3d::Zero()};
      double visibility_target_epoch{0.0};
      double planning_epoch{0.0};
      double requested_evaluation_horizon{0.0};
      bool outer_loop_evaluation{false};
      unsigned int execution_history_size{0};
      double execution_history_median{0.0};
      DynamicRiskInfo nominal_risk;
      ConflictDescriptor nominal_conflict;
      int nominal_obstacle_id{-1};
      std::vector<CandidateResult> candidates;
    };

    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    void initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis = NULL);
    double geometryEvaluationHorizon() const {return geometry_policy_.horizon;}
    // Reuse the configured visibility/goal tolerance when deciding whether a
    // cooperative viewpoint still represents forward planning progress.
    double visibilityMinTargetDistance() const { return visibility_min_target_distance_; }
    bool computeInitState(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
        const Eigen::Vector3d &local_target_vel, const bool flag_polyInit,
        const bool flag_randomPolyTraj, const double &ts, poly_traj::MinJerkOpt &initMJO,
        const double time_scale = 1.0, const int side_bias = 0,
        const double side_offset = 0.7,
        const Eigen::Vector3d *side_direction_override = nullptr);
    bool buildFreshMovingInitializer(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
        const Eigen::Vector3d &local_target_vel, poly_traj::MinJerkOpt &seed,
        std::string &source, double &required_speed, double &required_acc,
        double &required_jerk, bool mission_end = false, int side_bias = 0,
        double side_offset = 0.0, double duration_scale = 1.0,
        const Eigen::Vector3d *side_direction_override = nullptr);
    bool buildStaticFeasibleInitializer(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
        const Eigen::Vector3d &local_target_vel, poly_traj::MinJerkOpt &seed,
        std::string &source, double &required_speed, double &required_acc,
        double &required_jerk, bool mission_end = false,
        const Eigen::Vector3d *tracked_target_position = nullptr);
    bool canReuseRemainingSuffix(const Eigen::Vector3d &start_pt,
                                 const Eigen::Vector3d &start_vel,
                                 const Eigen::Vector3d &start_acc,
                                 const Eigen::Vector3d &local_target_pt,
                                 const Eigen::Vector3d &local_target_vel,
                                 const poly_traj::Trajectory &remaining,
                                 std::string &reason) const;
    unsigned long initReuseCount() const { return init_reuse_count_; }
    unsigned long freshInitializerCount() const { return fresh_initializer_count_; }
    unsigned long initializerRetimedCount() const { return initializer_retimed_count_; }
    unsigned long initializerRejectedPreoptCount() const { return initializer_rejected_preopt_count_; }
    unsigned long staleShrinkingInitializerEventCount() const {
      return stale_shrinking_initializer_event_count_;
    }
    unsigned long oldRemainingTimeReusedWithNewTargetCount() const {
      return old_remaining_time_reused_with_new_target_count_;
    }
    bool reboundReplan(
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const Eigen::Vector3d &end_pt,
        const Eigen::Vector3d &end_vel, const Eigen::Vector3d &object_pt,
        const Eigen::Vector3d &object_vel, const Eigen::Quaterniond &object_q, 
        const Eigen::Vector3d &relative_track_pt, const bool flag_polyInit, 
        const bool flag_randomPolyTraj, const bool touch_goal, const bool satrt_tracking,
        const double commit_head_freshness_threshold,
        CandidateSetOutput *capture_output = nullptr,
        int encirclement_hypothesis_id = 0,
        double encirclement_phi0 = 0.0,
        unsigned long encirclement_generation = 0);
    bool planGlobalTrajWaypoints(
        const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
        const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
        const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    void getLocalTarget(
        const double planning_horizen,
        const Eigen::Vector3d &start_pt, const Eigen::Vector3d &global_end_pt,
        Eigen::Vector3d &local_target_pos, Eigen::Vector3d &local_target_vel,
        bool &touch_goal);
    void prepareFutureActivation(Eigen::Vector3d &p, Eigen::Vector3d &v, Eigen::Vector3d &a);
    bool lifecycleSuccessorDue();
    bool localActivationPending() const {
      return traj_.local_traj.traj_id > 0 &&
          (executed_traj_id_ != traj_.local_traj.traj_id || executed_generation_ != active_traj_generation_);
    }
    void ensureExecutionCoverage();
    void finishPlanningBatch(double wall_started);
    bool optionalRefinementAllowed() const;
    bool mandatoryPlanningAttemptAllowed() const;
    bool executionReserveLimited() const { return execution_reserve_limited_; }
    bool currentStateRestartActive() const { return current_state_restart_active_; }
    // Target-prediction horizon in SECONDS, owned by the moving-object
    // predictor.  Callers that need a forward time prediction must use this and
    // must never multiply by the spatial planning lookahead (metres).
    double movingObjectPredictionHorizonTime() const {
      return ploy_traj_opt_ ? ploy_traj_opt_->getMovingObjPredictionHorizon()
                            : 0.0;
    }

    /* ---- single authoritative target-state time base ---------------------- */
    // The FSM publishes the smoothed tracked-target estimate together with the
    // stamp of the measurement it belongs to.  Every consumer (raw LOS, J_vis,
    // candidate visibility, dynamic risk) must ask for the target at a world
    // time instead of keeping its own age compensation.
    void setTargetState(const Eigen::Vector3d &position,
                        const Eigen::Vector3d &velocity,
                        const double stamp_seconds) {
      target_state_position_ = position;
      target_state_velocity_ = velocity;
      target_state_stamp_ =
          std::isfinite(stamp_seconds) && stamp_seconds > 0.0 ? stamp_seconds
                                                             : 0.0;
    }
    bool targetStateValid() const { return target_state_stamp_ > 0.0; }
    double targetStateStamp() const { return target_state_stamp_; }
    Eigen::Vector3d targetPositionAt(const double world_time) const {
      if (!targetStateValid() || !std::isfinite(world_time))
        return target_state_position_;
      return target_state_position_ +
             target_state_velocity_ * (world_time - target_state_stamp_);
    }
    Eigen::Vector3d targetVelocityAt(const double /*world_time*/) const {
      return target_state_velocity_;
    }
    // Audit view of the time-base counters (used by the FSM-side reporting).
    void timebaseAuditCounts(unsigned long &mixed_target,
                             unsigned long &mixed_dynamic_risk,
                             unsigned long &ros_now_in_risk) const {
      mixed_target = mixed_target_timebase_count_;
      mixed_dynamic_risk = mixed_dynamic_risk_timebase_count_;
      ros_now_in_risk = ros_now_inside_dynamic_risk_count_;
    }
    /* traj_server 侧确认收到本机轨迹后由 FSM 通知一次；用于把
     * SIDE_BOTH_FAILED 分级链推进到 RECEIVED。只做身份比对与计数。 */
    // swarm 互不恶化解锁审计（本轮）。
    void reportSwarmUnlockAudit(bool final_report) const;
    // 本轮：soft visibility cost 与 SIDE topology 触发权分离审计。
    void visibilityTopologyAuditCounts(unsigned long &soft_nonzero,
                                       unsigned long &soft_only_side,
                                       unsigned long &body_side,
                                       unsigned long &los_side) const {
      soft_nonzero = soft_vis_nonzero_count_;
      soft_only_side = soft_vis_only_triggered_side_count_;
      body_side = body_descriptor_side_trigger_count_;
      los_side = los_descriptor_side_trigger_count_;
    }
    void losPlaneAuditCounts(unsigned long &created, unsigned long &lost,
                             unsigned long &violation) const {
      created = los_plane_created_count_;
      lost = los_plane_lost_count_;
      violation = los_plane_violation_count_;
    }
    // One consolidated, greppable time-base / side-fallback / LOS-plane audit
    // line.  Emitted periodically and once at the end of the run.
    void reportTimebaseAndFallbackAudit(bool final_report) {
      const double now = ros::Time::now().toSec();
      if (!final_report && timebase_report_last_log_ > 0.0 &&
          now - timebase_report_last_log_ < 5.0)
        return;
      if (final_report)
      {
        if (timebase_audit_final_emitted_)
          return;
        timebase_audit_final_emitted_ = true;
      }
      timebase_report_last_log_ = now;
      ROS_INFO("[timebase-audit] MIXED_TARGET_TIMEBASE_COUNT=%lu "
               "MIXED_DYNAMIC_RISK_TIMEBASE_COUNT=%lu "
               "ROS_NOW_INSIDE_DYNAMIC_RISK_COUNT=%lu",
               mixed_target_timebase_count_, mixed_dynamic_risk_timebase_count_,
               ros_now_inside_dynamic_risk_count_);
      ROS_INFO("[side-both-failed] SIDE_BOTH_FAILED_COUNT=%lu",
               side_both_failed_count_);
      ROS_INFO("[los-plane-audit] LOS_PLANE_CREATED_COUNT=%lu "
               "LOS_PLANE_LOST_COUNT=%lu LOS_PLANE_VIOLATION_COUNT=%lu",
               los_plane_created_count_, los_plane_lost_count_,
               los_plane_violation_count_);
      visibilityTopologyReportLastLog(now);
    }
    void reportVisibilityTopologyAudit(bool final_report) {
      if (final_report)
      {
        if (visibility_topology_audit_final_emitted_)
          return;
        visibility_topology_audit_final_emitted_ = true;
      }
      visibilityTopologyReportLastLog(ros::Time::now().toSec());
    }
    /* 本轮修复 4：rolling 前缀的参考进度速度。复用系统已有的目标运动速度
     * （tracking 语义下即被跟踪目标的速度），不新增固定速度阈值。目标速度
     * 尚未可用时回落到系统既有的保守速度标尺 0.55*max_vel，并始终以目标速度
     * 为上限：进度代价是"别拖慢"，不是"催着超速"。 */
    double prefixProgressReferenceSpeed() const {
      // 优先用本批次已经解析出的局部目标速度（编队几何给出的参考进度速度），
      // 它与被跟踪目标的巡航速度一致；其次用目标状态；最后才回落到系统既有
      // 的保守速度标尺。始终以 max_vel 为上界。
      const double target_speed = targetVelocityAt(0.0).norm();
      double reference = 0.0;
      if (std::isfinite(target_speed) && target_speed > 1.0e-3)
        reference = target_speed;
      else
        reference = pp_.max_vel_ > 0.0 ? 0.55 * pp_.max_vel_ : 1.0;
      if (pp_.max_vel_ > 0.0)
        reference = std::min(reference, pp_.max_vel_);
      return reference;
    }
    /* 真实物理风险越高，前向进度目标越应让位于避障。这里只复用系统已有的
     * BODY 风险证据（是否越过避让裕度、是否真实重叠），不做任何硬约束。 */
    double prefixProgressRiskAttenuation(const double prediction_epoch) const {
      if (traj_.local_traj.traj.getPieceNum() <= 0)
        return 1.0;
      const DynamicRiskInfo risk =
          evaluateDynamicRisk(traj_.local_traj.traj, prediction_epoch, false);
      constexpr double kMinimumAttenuation = 0.15;
      if (!risk.valid)
        return 1.0;
      if (risk.hard_collision)
        return kMinimumAttenuation;
      const double trigger_distance = std::max(0.30, risk.trigger_distance);
      if (risk.triggered)
        return kMinimumAttenuation;
      const double distance = risk.min_distance;
      if (!std::isfinite(distance))
        return 1.0;
      const double normalized =
          std::max(0.0, std::min(1.0, (distance - trigger_distance) /
                                          std::max(0.30, 0.50 * trigger_distance)));
      return kMinimumAttenuation + (1.0 - kMinimumAttenuation) * normalized;
    }
    void visibilityTopologyReportLastLog(const double now) {
      if (!visibility_topology_audit_final_emitted_ &&
          visibility_topology_report_last_log_ > 0.0 &&
          now - visibility_topology_report_last_log_ < 5.0)
        return;
      visibility_topology_report_last_log_ = now;
      ROS_INFO("[visibility-topology-audit] SOFT_VIS_NONZERO_COUNT=%lu "
               "SOFT_VIS_ONLY_TRIGGERED_SIDE_COUNT=%lu "
               "BODY_DESCRIPTOR_SIDE_TRIGGER_COUNT=%lu "
               "LOS_DESCRIPTOR_SIDE_TRIGGER_COUNT=%lu "
               "SIDE_WITHOUT_DESCRIPTOR_NOMINAL_FALLBACK_COUNT=%lu",
               soft_vis_nonzero_count_, soft_vis_only_triggered_side_count_,
               body_descriptor_side_trigger_count_,
               los_descriptor_side_trigger_count_,
               side_without_descriptor_nominal_fallback_count_);
      ROS_INFO("[los-current-state-audit] CURRENT_RAW_LOS_BLOCKED_COUNT=%lu "
               "CURRENT_LOS_BLOCKED_WITH_AUTHORITY_COUNT=%lu "
               "CURRENT_LOS_BLOCKED_WITHOUT_AUTHORITY_COUNT=%lu",
               current_raw_los_blocked_count_,
               current_los_blocked_with_authority_count_,
               current_los_blocked_without_authority_count_);
      ROS_INFO("[side-both-failed-audit] SIDE_BOTH_FAILED_COUNT=%lu "
               "SIDE_BOTH_FAILED_NO_SUCCESSOR_COUNT=%lu",
               side_both_failed_count_, side_both_failed_no_successor_count_);
      // Feedback117: candidate lifecycle + A*->MINCO chain + LOS soft
      // authority run-time evidence (INVARIANT 1: LOS_HARD_REJECT stays 0).
      ROS_INFO("[side-lifecycle-audit] SIDE_HARD_VALID=%lu "
               "SIDE_HARD_PREFLIGHT_FAILED=%lu SIDE_VALID_NOT_SELECTED=%lu "
               "SIDE_SELECTED=%lu SIDE_COMMITTED=%lu SIDE_ACTIVATED=%lu",
               side_hard_valid_count_, side_hard_preflight_failed_count_,
               side_valid_not_selected_count_, side_selected_count_,
               side_committed_count_, side_activated_count_);
      ROS_INFO("[astar-to-minco-audit] ASTAR_ATTEMPTS=%lu "
               "ASTAR_RAW_PATH_SAFE=%lu CORRIDOR_BUILD_SUCCESS=%lu "
               "MINCO_SEED_STATIC_SAFE=%lu ASTAR_SAFE_BUT_MINCO_UNSAFE=%lu "
               "ASTAR_TO_MINCO_REPAIR_ATTEMPT=%lu "
               "ASTAR_TO_MINCO_REPAIR_SUCCESS=%lu",
               astar_attempt_count_, astar_raw_path_safe_count_,
               corridor_build_success_count_, minco_seed_static_safe_count_,
               astar_safe_but_minco_unsafe_count_,
               astar_to_minco_repair_attempt_count_,
               astar_to_minco_repair_success_count_);
      // Feedback119:前端一次性收敛合同(A* admission / corridor / SCP)。
      ROS_INFO("[corridor-contract-audit] ASTAR_REQUIRED=%lu "
               "ASTAR_NO_PATH=%lu ASTAR_STATIC_SAFE=%lu "
               "SFC_BUILD_SUCCESS=%lu SFC_BUILD_FAILED=%lu "
               "SFC_NOT_REQUIRED=%lu CORRIDOR_BUILD_FAILED=%lu "
               "CORRIDOR_SCP_ATTEMPT=%lu CORRIDOR_SCP_SUCCESS=%lu "
               "CORRIDOR_SCP_INFEASIBLE=%lu "
               "CONTINUOUS_CORRIDOR_VIOLATION=%lu",
               astar_required_count_, astar_no_path_count_,
               astar_raw_path_safe_count_, corridor_build_success_count_,
               corridor_build_failed_count_, sfc_not_required_count_,
               corridor_build_failed_count_, corridor_scp_attempt_count_,
               corridor_scp_success_count_, corridor_scp_infeasible_count_,
               corridor_continuous_violation_count_);
      ROS_INFO("[los-soft-authority-audit] LOS_SOFT_QP_ATTEMPTS=%lu "
               "LOS_SOFT_QP_SOLVED=%lu LOS_SOFT_SLACK_NONZERO=%lu "
               "LOS_HARD_REJECT_COUNT=%lu K3_SAME_CYCLE_EVENT_CREATED=%lu "
               "K3_SAME_CYCLE_DISPATCH=%lu",
               los_soft_qp_attempt_count_, los_soft_qp_solved_count_,
               los_soft_qp_slack_nonzero_count_, los_hard_reject_count_,
               k3_same_cycle_event_created_count_,
               k3_same_cycle_dispatch_count_);
      {
        std::string side_cause_line;
        for (const auto &cause : side_failure_cause_count_)
        {
          if (!side_cause_line.empty())
            side_cause_line += " ";
          side_cause_line += cause.first + "=" + std::to_string(cause.second);
        }
        ROS_INFO("[side-construction-causes]%s",
                 side_cause_line.empty() ? " NONE"
                                         : (" " + side_cause_line).c_str());
      }
      ROS_INFO("[los-occlusion-audit] LOS_KNOWN_OCCLUSION_WITHOUT_PLANE_COUNT=%lu "
               "LOS_WINDOW_LIMITED_COUNT=%lu LOS_RECOVERY_NOT_REACHED_COUNT=%lu "
               "LOS_PLANE_CREATED_COUNT=%lu LOS_PLANE_LOST_COUNT=%lu "
               "LOS_PLANE_VIOLATION_COUNT=%lu",
               los_known_occlusion_without_plane_count_, los_window_limited_count_,
               los_recovery_not_reached_count_, los_plane_created_count_,
               los_plane_lost_count_, los_plane_violation_count_);
      ROS_INFO("[candidate-feasibility-audit] NO_EXECUTABLE_SUCCESSOR_COUNT=%lu",
               no_executable_successor_count_);
    }
    void setGeometryTarget(const Eigen::Vector3d &p,const Eigen::Vector3d &v,
                           double stamp, std::uint64_t source_identity) {
      geometry_target_p_=p;geometry_target_v_=v;geometry_target_stamp_=stamp;
      geometry_target_source_identity_=source_identity;
    }
    LocalGeometryMetrics evaluateLocalGeometry(const poly_traj::Trajectory &candidate,
        double candidate_start,double begin,double horizon) const;
    void logExecutionGeometry(const char *event, const LocalGeometryMetrics *baseline=nullptr);

    void invalidatePendingTopology(const char *reason);
    bool checkActiveHandoff(const poly_traj::Trajectory &trajectory, double activation,
                            const char *new_source, bool log = true) const;
    /* REBUILD contract for foreign payloads only (Team relay anchor probe,
     * defensive commit of a not-yet-prepared candidate): re-attaches the
     * polynomial to the given activation boundary by rebuilding it.  Every
     * caller must follow with its own explicit validation (checkActiveHandoff
     * / dynamics / SFC).  The ordinary Local N/L/R production contract NEVER
     * calls this — after generation the optimizer owns the polynomial and no
     * post-optimization mutation authority exists (frozen-activation
     * contract; see validateLocalHandoffWindow).  若 activation 发生变化，
     * 函数内部会把 local_sfc_planes 中世界时间锚定的 LOS 平面从旧锚点换算
     * 到新锚点，保证遮挡事件的世界时间语义不漂移。 */
    bool rebuildLocalCandidateAtActivation(poly_traj::MinJerkOpt &opt, double &activation,
                             std::string *reason = nullptr,
                             std::vector<LocalSfcPlane> *local_sfc_planes = nullptr,
                             bool reanchor_to_now = true);
    /* Validation-only handoff scheduling check for the frozen-activation
     * contract: verifies the batch's frozen activation is still an executable
     * boundary (not missed by a late pipeline, not beyond predecessor
     * validated coverage).  Continuity itself is checkActiveHandoff's
     * verdict.  This function never touches the polynomial. */
    bool validateLocalHandoffWindow(double activation, std::string *reason) const;
    bool validateActivePrefixUntil(double until, std::string &reason) const;
    bool validateCommittedPrefixUntil(double until, std::string &reason) const;

    bool checkCollision(int drone_id);
    bool checkTrajectoryDynamics(const poly_traj::Trajectory &traj, double &max_vel,
                                 double &max_acc, double &max_jer) const;
    bool checkTrajectoryStaticSafety(const poly_traj::Trajectory &traj,
                                     const bool touch_goal, std::string &reason) const;
    bool checkTrajectorySwarmSafety(const poly_traj::Trajectory &traj, const double start_time,
                                    const bool touch_goal, std::string &reason) const;
    SwarmConflictInfo evaluateTrajectorySwarmConflict(
        const poly_traj::Trajectory &traj, double start_time,
        bool touch_goal, bool adjustable_only) const;
    bool setLocalTrajFromOpt(
        const poly_traj::MinJerkOpt &opt, const bool touch_goal,
        double explicit_start_time = std::numeric_limits<double>::quiet_NaN(),
        const traj_utils::TeamTrajectorySolution *team = nullptr);
    bool validateExecutionTrajectory(const poly_traj::Trajectory &trajectory,
        double activation, bool touch_goal, std::string &reason,
        const traj_utils::TeamTrajectorySolution *team = nullptr) const;
    // Backward-compatible probe/contract entry point.  Callers that do not
    // carry explicit mission-terminal state are rolling callers by contract.
    bool validateExecutionTrajectory(const poly_traj::Trajectory &trajectory,
        double activation, std::string &reason,
        const traj_utils::TeamTrajectorySolution *team = nullptr) const
    {
      return validateExecutionTrajectory(trajectory, activation, false, reason, team);
    }
    void annotateExecutionSource(traj_utils::PolyTraj &message) const;
    bool validateRetimedLocalSfc(const CandidateResult &source,
        const poly_traj::Trajectory &final_trajectory) const;
    bool validatePreviousRemainingTrajectory(const double prediction_epoch,
                                             const bool touch_goal,
                                             std::string &reason,
                                             double &min_dynamic_clearance) const;
    bool buildWarmStartFromAccepted(const Eigen::Vector3d &start_pt,
                                    const Eigen::Vector3d &start_vel,
                                    const Eigen::Vector3d &start_acc,
                                    const Eigen::Vector3d &local_target_pt,
                                    const Eigen::Vector3d &local_target_vel,
                                    poly_traj::MinJerkOpt &warm_mjo,
                                    double &age,
                                    double &remaining_duration,
                                    int &old_piece_num,
                                    int &new_piece_num,
                                    std::string &reason) const;
    DynamicRiskInfo evaluateDynamicRisk(const poly_traj::Trajectory &traj,
                                        double prediction_epoch,
                                        bool touch_goal) const;
    // A rolling local endpoint is only authoritative for the bounded prefix
    // that the optimizer repairs.  Mission-terminal trajectories retain full
    // duration validation.  All hard validators use this single horizon.
    double executionAuthorityHorizon(double duration, bool touch_goal) const;
    double hardCheckedUntil(const poly_traj::Trajectory &trajectory,
                            double activation, bool touch_goal) const;
    double dynamicHardClearanceForObject(int object_id) const;
    CandidateVisibilityReport evaluateCandidateVisibility(
        const poly_traj::Trajectory &candidate_traj,
        double candidate_start_time,
        const Eigen::Vector3d &target_position,
        const Eigen::Vector3d &target_velocity,
        double target_prediction_epoch,
        bool encirclement_active) const;
    /* K3 统一 visibility sample 原语(§4):唯一一份 binary(含 FOV)+ 风险分量
     * + margin。所有查询复用既有权威函数;target/observer/障碍全部由调用方
     * 按 world_time 采样,本原语不自带第二套锚定。 */
    LocalVisibilitySample evaluateLocalVisibilitySampleAtWorldTime(
        const Eigen::Vector3d &observer, double observer_yaw,
        const Eigen::Vector3d &target, double world_time) const;
    /* K3 窗口指标:C3/K2(三机统一 binary)+ D3(self margin)在相同
     * world_time/target/obstacle/camera 契约下逐样本计算。 */
    LocalK3WindowMetrics evaluateLocalK3WindowMetrics(
        const poly_traj::Trajectory &self_traj, double self_start_world,
        const Eigen::Vector3d &target_at_epoch,
        const Eigen::Vector3d &target_velocity, double target_epoch,
        double window_begin, double window_end,
        double required_margin) const;
    bool consumeStaleHeadReplanRequest(Eigen::Vector3d &odom_pos,
                                       Eigen::Vector3d &odom_vel);
    enum class TopologyProcessStatus
    {
      NONE,
      PENDING,
      COMMITTED,
      RETAINED_PREVIOUS,
      FAILED
    };
    bool topologyCoordinationPending() const;
    bool teamTrajectorySampleHoldActive(double now) const;
    bool jointTopologyCoordinationEnabled() const
    {
      return enable_joint_topology_coordination_;
    }
    bool backgroundTeamQualityAdmissible(double now) const;
    bool stageCapturedTopologyCoordination(
        const std::vector<CandidateSetOutput> &sets, int current_hypothesis_id,
        bool touch_goal, double commit_head_freshness_threshold);
    TopologyProcessStatus finalizeCapturedCandidates(std::vector<CandidateSetOutput> &sets,
        bool touch_goal,double head_threshold,int &selected_set);
    TopologyProcessStatus finalizeCapturedLocal(
        const CandidateSetOutput &set, bool touch_goal,
        double commit_head_freshness_threshold);
    TopologyProcessStatus processPendingTopologyCoordination();
    void setFovSoftScale(const double scale);
    inline double getSwarmClearance(void) const { return ploy_traj_opt_->get_swarm_clearance_(); }
    inline int getCpsNumPrePiece(void) { return ploy_traj_opt_->get_cps_num_prePiece_(); }
    // inline PtsChk_t getPtsCheck(void) { return ploy_traj_opt_->get_pts_check_(); }

    PlanParameters pp_;
    GridMap::Ptr grid_map_;
    fast_planner::ObjPredictor::Ptr obj_predictor_;
    TrajContainer traj_;

  private:
    // Optional, read-only audit sink; never consumed by planning decisions.
    void traceTopologyAudit134(const char *stage, const CandidateResult *candidate,
        const poly_traj::Trajectory *trajectory = nullptr,
        const std::string &reason = "", int related_id = -1,
        double comparison_begin = 0.0, double comparison_end = 0.0) const;
    trajectory_lifecycle::State currentStateAt(double stamp) const;
    LocalGeometryPolicy geometry_policy_;
    unsigned long active_reference_id_{0};
    std::string active_reference_reason_{"NO_TEAM_REFERENCE"};
    bool local_geometry_active_{false};
    Eigen::Vector3d geometry_target_p_{Eigen::Vector3d::Zero()},geometry_target_v_{Eigen::Vector3d::Zero()};
    double geometry_target_stamp_{0.0};
    std::uint64_t geometry_target_source_identity_{0};
    ExecutionPlanningBudget execution_budget_;
    bool execution_reserve_limited_{false};
    double execution_margin_{0.15}, uninterruptible_budget_{0.05};
    double planning_deadline_wall_{std::numeric_limits<double>::infinity()};
    double validated_coverage_end_{std::numeric_limits<double>::infinity()};
    double planning_deadline_ros_{std::numeric_limits<double>::infinity()};
    double validated_coverage_remaining_{std::numeric_limits<double>::infinity()};
    double planning_budget_remaining_{std::numeric_limits<double>::infinity()};
    bool validated_handoff_this_batch_{false};
    bool executable_candidate_seen_this_batch_{false};
    int planning_predecessor_id_{-1};
    bool current_state_restart_active_{false};
    int expired_predecessor_id_{-1};
    bool current_state_valid_{false};
    Eigen::Vector3d current_state_p_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d current_state_v_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d current_state_a_{Eigen::Vector3d::Zero()};
    double current_state_stamp_{0.0};
    double local_activation_time_{0.0};
    double local_planning_started_{0.0};
    double local_planning_budget_{0.08};
    double local_activation_margin_{0.10};
    double lifecycle_last_validation_{0.0};
    double lifecycle_validation_expiry_{0.0};
    bool lifecycle_invalidated_{false};
    double execution_safe_until_{0.0};
    std::uint64_t active_validation_revision_{0};
    LocalTrajData scheduled_predecessor_;
    ros::Subscriber execution_adoption_sub_;
    int executed_traj_id_{-1};
    unsigned long executed_generation_{0};

    /* =====================================================================
     * Feedback093 authoritative executed history.
     *
     * The Local Planner's lineage authority is the trajectory the executor has
     * CONFIRMED ACTIVE (`/trajectory_execution/activated`), never a Team future
     * that has merely been REALIZED / CERTIFIED / PREPARED / COMMITTED.
     * `predecessor_history_` resolves a trajectory identity to the exact
     * polynomial, so a successor can never be re-anchored on some other
     * trajectory.  `declared_predecessor_id_` is the identity written into
     * every published payload; the executor resolves exactly that identity.
     * ===================================================================== */
    struct PredecessorRecord
    {
      LocalTrajData data;
      uint64_t generation{0};
      std::string source;
      uint64_t team_solution_id{0};
      bool team_speculative{false};
    };
    static constexpr int kTeamSpeculativeTrajectoryIdBase = 1000000;
    std::deque<PredecessorRecord> predecessor_history_;
    LocalTrajData authoritative_predecessor_;
    bool authoritative_predecessor_valid_{false};
    int authoritative_predecessor_id_{-1};
    uint64_t authoritative_predecessor_generation_{0};
    std::string authoritative_predecessor_source_;
    int declared_predecessor_id_{-1};
    uint64_t declared_predecessor_generation_{0};
    unsigned long authority_switch_count_{0};
    unsigned long authority_lookup_miss_count_{0};
    unsigned long silent_predecessor_fallback_count_{0};
    const PredecessorRecord *findPredecessorRecord(const int traj_id) const;
    void recordPredecessorRecord(const LocalTrajData &data,
                                 const uint64_t generation,
                                 const std::string &source,
                                 const uint64_t team_solution_id,
                                 const bool team_speculative);
    const LocalTrajData &predecessorAuthority() const
    {
      return authoritative_predecessor_valid_ ? authoritative_predecessor_
                                              : traj_.local_traj;
    }
    int predecessorAuthorityId() const
    {
      return authoritative_predecessor_valid_ ? authoritative_predecessor_.traj_id
                                              : traj_.local_traj.traj_id;
    }
    uint64_t predecessorAuthorityGeneration() const
    {
      return authoritative_predecessor_valid_
                 ? authoritative_predecessor_generation_
                 : active_traj_generation_;
    }
    void reportLineageAudit(bool final_report) const;
    int pending_quality_upgrade_traj_id_{-1};
    int pending_quality_upgrade_candidate_id_{-1};
    std::string lifecycle_state_{"CURRENTLY_VALID"};
    double handoff_position_tolerance_{0.02};
    double handoff_velocity_tolerance_{0.05};
    double handoff_acceleration_tolerance_{0.10};
    double candidate_ready_time_{0.0}, commit_time_{0.0}, last_pending_duration_{0.0};
    std::vector<LocalSfcPlane> active_local_sfc_planes_;
    PlanningVisualization::Ptr visualization_;

    PolyTrajOptimizer::Ptr ploy_traj_opt_;
    // Reuse the planner's inflated GridMap for local SIDE reference repair.
    // This is kept separate from PolyTrajOptimizer's collision-repair A* so
    // candidate generation never changes the optimizer's internal state.
    AStar::Ptr side_a_star_;

    bool enable_risk_triggered_candidates_{false};
    bool enable_candidate_region_constraint_{true};
    bool enable_side_local_astar_repair_{true};
    double risk_sample_dt_{0.10};
    // Radius of the UAV footprint used by the inflated grid map.  Dynamic
    // obstacle radii are taken from the predictor's live marker scale.
    // Horizontal hard-collision footprint of the deployed iris model; this
    // is intentionally independent from grid-map occupancy inflation.
    double dynamic_body_radius_{0.384};
    double candidate_trigger_margin_{0.15};
    double temporal_swarm_trigger_margin_{0.08};
    double temporal_swarm_sample_dt_{0.05};
    double temporal_swarm_conflict_half_window_{0.60};
    double team_visibility_preferred_separation_deg_{25.0};
    double encirclement_spread_saturation_angle_deg_{60.0};
    bool enable_joint_topology_coordination_{false};
    bool enable_team_visibility_optimizer_{false};
    double topology_coordination_timeout_{0.12};
    double team_coordination_timeout_{0.35};
    double topology_result_max_age_{0.30};
    double topology_min_active_remaining_{0.35};
    double visibility_sample_dt_{0.10};
    double visibility_min_target_distance_{0.20};
    double visibility_max_target_distance_{8.0};
    double visibility_occlusion_margin_{0.08};
    CandidateKind last_candidate_kind_{CandidateKind::NOMINAL};
    int last_candidate_obstacle_id_{-1};
    CandidateKind active_candidate_kind_{CandidateKind::NOMINAL};
    int active_candidate_obstacle_id_{-1};
    bool candidate_observation_frame_valid_{false};
    Eigen::Vector3d candidate_observation_side_direction_{Eigen::Vector3d::Zero()};
    bool active_candidate_metadata_valid_{false};
    // Preserve the execution semantics of the currently installed trajectory.
    // Rolling local targets are validated only over their bounded authority
    // prefix; mission-terminal trajectories retain full-duration validation.
    bool active_execution_touch_goal_{false};
    unsigned long active_traj_generation_{0};
    std::string active_execution_source_{"OTHER"};
    unsigned long active_execution_team_id_{0};
    unsigned long active_execution_view_generation_{0};
    int active_execution_hypothesis_{0};
    bool active_execution_safety_validated_{false};
    int non_risk_count_{0};
    int continous_failures_count_{0};
    unsigned long risk_triggered_count_{0};
    unsigned long raw_static_los_blocked_count_{0};
    unsigned long raw_dynamic_los_blocked_count_{0};
    unsigned long los_witness_created_count_{0};
    unsigned long los_descriptor_count_{0};
    unsigned long los_detection_miss_count_{0};
    /* 本轮修复 4：rolling execution prefix 的前向进度软目标。
     * span  [s]   —— 真正可能被执行的 rolling 前缀长度（定义为 0.0 时由系统
     *                已有的 local_activation_margin + execution_margin 推出）；
     * weight[-]   —— 无量纲软权重（代价 = w * (deficit/v_ref)^2 的时间积分）。
     * 两者都不构成任何硬约束。 */
    /* 本轮修复 1/2：合围软方位恢复权重（相对 wei_tracking_ 的倍数）与前缀加强。
     * 参考方位来自稳定编队种子，不随当前位置漂移；纯 soft cost。 */
    double encirclement_bearing_recovery_weight_{0.08};
    double encirclement_bearing_prefix_boost_{2.0};
    /* 本轮：连续 visibility cost（J_vis）只能进入连续优化，不得把任意非零
     * 代价升级为 SIDE topology 权限。 下面这些计数器只做审计。 */
    mutable unsigned long soft_vis_nonzero_count_{0};
    mutable unsigned long soft_vis_only_triggered_side_count_{0};
    mutable unsigned long side_without_descriptor_nominal_fallback_count_{0};
    /* 本轮 P1：真实 LOS 遮挡区间（相对本批次 planning_prediction_epoch，单位 s）。
     * 由 raw LOS truth 扫描逐帧得到：blocker 第一次/最后一次与
     * target(t) → nominal_traj(t) 线段相交的时刻。它是 LOS 观察面有效窗口的
     * 唯一依据，不再沿用固定 ±0.45 s 的 BODY 指导窗。 */
    double raw_los_occlusion_enter_rel_{-1.0};
    double raw_los_occlusion_exit_rel_{-1.0};
    bool raw_los_occlusion_interval_observed_{false};
    /* 本轮修复 4：当前帧（当前世界时刻）的真实 LOS 是否被挡。
     * 只要它为真，本轮就必须持续拥有 LOS recovery authority——不能因为
     * "之前预测的那次遮挡事件已经过去" 就停止恢复（预测事件结束 != 实际已恢复）。 */
    bool current_raw_los_blocked_{false};
    int current_raw_los_blocker_id_{-1};
    bool current_raw_los_blocker_dynamic_{false};
    /* 当前帧 LOS 判定所用的观测者来源：true=本机当前 odom（真实执行世界状态），
     * false=退回标称轨迹 0 时刻。用于验证 current LOS recovery 不以未来候选为准。 */
    bool current_los_observer_source_is_odom_{false};
    mutable unsigned long current_raw_los_blocked_count_{0};
    mutable unsigned long current_los_blocked_with_authority_count_{0};
    mutable unsigned long current_los_blocked_without_authority_count_{0};
    /* 本轮 P1 审计：已知未来遮挡却没有生成 LOS 平面 / 观察面窗口不可达。 */
    mutable unsigned long los_known_occlusion_without_plane_count_{0};
    mutable unsigned long los_window_limited_count_{0};
    mutable unsigned long los_recovery_not_reached_count_{0};
    /* 本轮 P2 审计：NOMINAL 不可执行、SIDE 绝对安全却仅因相对增益被拒。 */
    mutable unsigned long no_executable_successor_count_{0};
    mutable unsigned long body_descriptor_side_trigger_count_{0};
    mutable unsigned long los_descriptor_side_trigger_count_{0};
    double visibility_topology_report_last_log_{0.0};
    bool visibility_topology_audit_final_emitted_{false};
    unsigned long topology_planning_generation_{0};
    int topology_candidate_serial_{0};

    /* Target-state time base.  The tracked-target estimate published by the
     * FSM is a smoothed state owned by an explicit stamp; every prediction must
     * be derived from that stamp instead of from a local ros::Time::now(). */
    Eigen::Vector3d target_state_position_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d target_state_velocity_{Eigen::Vector3d::Zero()};
    double target_state_stamp_{0.0};
    mutable unsigned long mixed_target_timebase_count_{0};
    mutable unsigned long mixed_dynamic_risk_timebase_count_{0};
    mutable unsigned long ros_now_inside_dynamic_risk_count_{0};
    mutable unsigned long los_plane_created_count_{0};
    mutable unsigned long los_plane_lost_count_{0};
    mutable unsigned long los_plane_violation_count_{0};
    mutable unsigned long side_both_failed_count_{0};
    // Feedback116:SIDE 失败按真实阶段拆分,不再混在一个 SIDE_BOTH_FAILED 里。
    mutable unsigned long side_construction_failed_count_{0};
    mutable unsigned long side_hard_preflight_failed_count_{0};
    mutable unsigned long side_valid_not_selected_count_{0};
    mutable unsigned long side_selected_count_{0};
    mutable unsigned long side_activated_count_{0};
    // Feedback117(authority 整合):candidate lifecycle 全量计数与按因拆分。
    // lifecycle: CONSTRUCTION_FAILED → HARD_PREFLIGHT_FAILED → HARD_VALID →
    //            VALID_NOT_SELECTED / SELECTED → COMMITTED → ACTIVATED
    mutable unsigned long side_hard_valid_count_{0};
    mutable unsigned long side_committed_count_{0};
    // 构造/硬预检失败按因:STATIC_SEED/ASTAR/CORRIDOR/MINCO/DYNAMICS/SWARM/
    // DEADLINE/LOS_SOFT(LOS_SOFT 恒为 0 是 INVARIANT 1 的运行时证据)。
    std::map<std::string, unsigned long> side_failure_cause_count_;
    // Feedback117:§8 same-cycle dispatch(event 创建与 SIDE 授权同 revision)。
    mutable unsigned long k3_same_cycle_event_created_count_{0};
    mutable unsigned long k3_same_cycle_dispatch_count_{0};
    // Feedback117:§5 A* → corridor → MINCO 链计数(§15 目标:ASTAR_SAFE_BUT_
    // MINCO_UNSAFE 尽量为 0)。
    mutable unsigned long astar_attempt_count_{0};
    mutable unsigned long astar_raw_path_safe_count_{0};
    mutable unsigned long corridor_build_success_count_{0};
    mutable unsigned long sfc_not_required_count_{0};
    mutable unsigned long minco_seed_static_safe_count_{0};
    mutable unsigned long astar_safe_but_minco_unsafe_count_{0};
    mutable unsigned long astar_to_minco_repair_attempt_count_{0};
    mutable unsigned long astar_to_minco_repair_success_count_{0};
    // Feedback119(前端一次性收敛):ASTAR_REQUIRED/ACTUALLY_RUN/NO_PATH 三段
    // 计数 + corridor 合同计数。restore/bulge 修补链已删除,repair 计数
    // 保留但恒为 0,作为"补丁链已不存在"的运行时证据。
    mutable unsigned long astar_required_count_{0};
    mutable unsigned long astar_no_path_count_{0};
    mutable unsigned long corridor_build_failed_count_{0};
    mutable unsigned long corridor_scp_attempt_count_{0};
    mutable unsigned long corridor_scp_success_count_{0};
    mutable unsigned long corridor_scp_infeasible_count_{0};
    mutable unsigned long corridor_continuous_violation_count_{0};
    // Feedback117:§15 LOS soft 汇总(SEMI_HARD slack QP 从不 reject 候选)。
    mutable unsigned long los_soft_qp_attempt_count_{0};
    mutable unsigned long los_soft_qp_solved_count_{0};
    mutable unsigned long los_soft_qp_slack_nonzero_count_{0};
    mutable unsigned long los_hard_reject_count_{0};
    /* 本轮修复 5：L/R 全失败后仍无合格 NOMINAL（应为 0）。 */
    mutable unsigned long side_both_failed_no_successor_count_{0};
    /* 本轮修复（计数口径分级）：L/R 全失败 → NOMINAL 必须分三级观测，
     * 只证明"preselect 选中"不等于"真的进入提交/执行链"。
     *   SELECTED  : 本轮把 selected_result 指向 NOMINAL
     *   SUBMITTED : 该 NOMINAL 真的被交给 commit 链并成功写回 traj_
     *   ACTIVATED : traj_server/生命周期真的安装并激活了这条轨迹
     * 三个计数应当单调递减，且 SUBMITTED <= SELECTED。 */
    /* 分级链必须绑定"同一条 trajectory identity"（traj_id + start_time），
     * 不能只凭"本轮处于 L/R 全失败语义"就记 SUBMITTED/RECEIVED/ACTIVATED。 */
    /* 本批次是否处于"L/R 全失败 → NOMINAL 回落"语义（提交链使用）。 */
    /* 已提交、等待真正安装/激活的 NOMINAL 回落轨迹。 */
    // Two independent throttles: the consistency witness is paced on planning
    // epochs, the periodic audit report on wall time.  Sharing one variable
    // made the audit report silently stop after the first sample.
    mutable unsigned long swarm_mutual_worsening_reject_count_{0};
    mutable unsigned long swarm_mutual_non_worsening_accept_count_{0};
    mutable unsigned long swarm_absolute_reject_count_{0};
    mutable double swarm_unlock_last_report_{0.0};
    mutable bool swarm_unlock_final_emitted_{false};
    double timebase_consistency_last_log_{0.0};
    double timebase_report_last_log_{0.0};
    bool timebase_audit_final_emitted_{false};
    struct PendingTopologyState
    {
      bool active{false};
      bool execution_commit_performed{false};
      unsigned long transaction_id{0};
      unsigned long planning_generation{0};
      unsigned long active_generation{0};
      int expected_trajectory_id{-1};
      unsigned long static_map_revision{0};
      unsigned long visibility_model_version{0};
      double target_prediction_epoch{0.0};
      double dynamic_prediction_epoch{0.0};
      unsigned long target_snapshot_identity{0};
      double target_snapshot_epoch{0.0};
      unsigned long dynamic_prediction_identity{0};
      double dynamic_prediction_valid_from{0.0};
      double dynamic_prediction_valid_to{0.0};
      double earliest_activation{0.0};
      double validated_end{0.0};
      double planning_epoch{0.0};
      double bundle_stamp{0.0};
      double evaluation_start_time{0.0};
      double evaluation_horizon{0.0};
      bool outer_loop_evaluation{false};
      unsigned int execution_history_size{0};
      double execution_history_median{0.0};
      double deadline{0.0};
      int local_candidate_id{-1};
      int local_index{-1};
      std::vector<CandidateResult> candidates;
      DynamicRiskInfo nominal_risk;
      int nominal_obstacle_id{-1};
      bool touch_goal{false};
      double commit_head_freshness_threshold{0.5};
    } pending_topology_;
    traj_utils::TopologyCoordination latest_topology_result_;
    bool latest_topology_result_valid_{false};
    ros::Publisher topology_bundle_pub_;
    ros::Subscriber topology_result_sub_;
    ros::Publisher team_trajectory_ack_pub_;
    ros::Subscriber team_trajectory_solution_sub_;
    ros::Subscriber team_reference_schedule_sub_;
    ros::Subscriber task_reference_sub_;
    ros::Subscriber topology_escalation_sub_;
    std::uint64_t task_reference_generation_{0};
    double team_solution_max_age_{0.30};
    double team_solution_min_activation_lead_{0.015};
    double team_min_execution_time_{0.12};
    double team_execution_hold_until_{0.0};
    unsigned long team_execution_hold_solution_id_{0};
    int team_execution_hold_traj_id_{-1};
    struct PendingTeamTrajectory
    {
      bool accepted{false};
      bool prepare_received{false};
      bool commit_received{false};
      unsigned long team_solution_id{0};
      unsigned long coordination_generation{0};
      unsigned long planning_generation{0};
      int source_candidate_id{-1};
      int source_candidate_index{-1};
      double activation_time{0.0};
      poly_traj::MinJerkOpt trajectory;
      traj_utils::MINCOTraj realized_message;
      traj_utils::TeamTrajectorySolution solution;
      bool target_centered_reference{false};
      /* True only when this commit is the result of a relay handoff-contract
       * refinement.  The Feedback096 adoption re-check applies to that case
       * only; an ordinary Team reference is never re-checked against the relay
       * contract and must keep its existing transaction semantics. */
      bool handoff_refinement{false};
      traj_utils::TeamReferenceSchedule reference_schedule;
    } pending_team_trajectory_;

    /* =====================================================================
     * Feedback096 — Team refinement scheduling against high-rate Local rolling.
     *
     * The Local planner rolls every ~12 ms while a Team T/PT refinement is
     * solved and then carried through PREPARE/READY/COMMIT.  A rolling update
     * is NOT by itself a reason to discard a refinement result: the result is
     * re-checked against the truly current execution anchor, the latest peers
     * and the latest world snapshot at the moment it would be adopted.  Only
     * physical incompatibility, a vanished requirement, or a missed activation
     * may discard it.
     *
     * Single-flight: at most one refinement may be open (frozen .. terminal).
     * Further schedule messages coalesce into `latest_snapshot_id_` instead of
     * starting a competing solve.
     * ===================================================================== */
    struct TeamRefinementSnapshot
    {
      std::uint8_t repair_k{0};
      std::uint64_t refinement_id{0};
      std::uint64_t contract_id{0};
      std::uint64_t team_solution_id{0};
      double activation{0.0};
      double acquire_by{0.0};
      double preserve_until{0.0};
      double contract_expire{0.0};
      double critical_begin{0.0};   // activation-relative window
      double critical_end{0.0};
      int outgoing{-1};
      int incoming{-1};
      int stable{-1};
      double required_margin{0.0};
      double m2_before{0.0};
      double anchor_dp{0.0};
      double anchor_dv{0.0};
      double anchor_da{0.0};
      // Baseline identities this refinement was solved from.
      std::array<int, 3> source_trajectory_ids{{0, 0, 0}};
      std::array<std::uint64_t, 3> source_generations{{0, 0, 0}};
      std::uint64_t start_snapshot_id{0};
      std::uint64_t finish_snapshot_id{0};
      std::uint64_t updates_during{0};
      /* Feedback100: M2 only DETECTS the risk.  The repair target set decides
       * who is actually repaired.  These are read straight from the published
       * schedule so every node derives the same priority order without adding
       * a protocol round trip. */
      std::array<int, 2> repair_candidates{{-1, -1}};
      std::array<double, 2> repair_candidate_margins{{0.0, 0.0}};
      std::array<double, 3> forecast_margins{{0.0, 0.0, 0.0}};
      int repair_priority_uav{-1};
      double t_cross{0.0};
      double t_recover{0.0};
      double t_star{0.0};
      double forecast_created_world{0.0};
      double detect_wall{0.0};
      double request_wall{0.0};
      double receive_wall{0.0};
      bool repair_declared_candidate{false};
      bool repair_seed_deficient{false};
      bool repair_eligible{false};
      double seed_window_min{-std::numeric_limits<double>::infinity()};
      double trial_m2_after{-std::numeric_limits<double>::infinity()};
      double trial_deformation_cost{std::numeric_limits<double>::infinity()};
      bool trial_feasible{false};
      std::string trial_reject_reason{"NONE"};
      int trial_selected_uav{-1};
      std::string trial_selection_reason{"NONE"};
      double scp_begin_wall{0.0};
      double scp_end_wall{0.0};
    };
    TeamRefinementSnapshot team_refinement_;
    bool team_refinement_open_{false};
    std::uint64_t team_refinement_serial_{0};
    std::uint64_t latest_team_prediction_snapshot_id_{0};
    /* Feedback098 Planner-side forecast configuration and identity. */
    /* Feedback101 short-horizon Team visibility reserve.  One new parameter
     * only; both repair candidates use the same weight. */
    double team_reserve_weight_{0.0};
    std::uint64_t team_reserve_activation_count_{0};
    std::uint64_t team_reserve_clear_count_{0};
    double team_reserve_trace_forecast_margin_{
        -std::numeric_limits<double>::infinity()};
    double team_reserve_trace_baseline_before_{
        -std::numeric_limits<double>::infinity()};
    double visibility_forecast_horizon_{2.0};
    double visibility_forecast_dt_{0.1};
    bool team_k3_repair_enabled_{false};
    std::uint64_t visibility_forecast_serial_{0};
    /* Latest relay-contract state seen on the schedule stream.  Used by the
     * adoption re-check to decide whether the obligation this refinement was
     * solved for is still outstanding (Check 2).  A contract that has been
     * released or replaced makes the result simply unnecessary. */
    std::uint64_t latest_handoff_contract_id_{0};
    bool latest_handoff_contract_active_{false};
    std::uint8_t latest_handoff_repair_k_{0};
    int latest_handoff_priority_uav_{-1};
    std::uint64_t latest_handoff_world_revision_{0};
    bool latest_handoff_contract_seen_{false};
    /* Result of the last solver attempt, consumed by the adoption re-check. */
    bool team_refinement_solver_pass_{false};
    /* Feedback098: a zero-modification acceptance is not a refinement and must
     * never obtain improvement authority. */
    bool team_refinement_zero_modification_{false};
    /* Margin the SCP claimed AFTER its refinement, used to decide whether a
     * measurable Team benefit exists before granting improvement authority. */
    double team_result_margin_after_{-std::numeric_limits<double>::infinity()};
    double team_result_dp_norm_{0.0};
    double team_result_dt_norm_{0.0};
    /* Feedback100: mode and pre-refinement margin of the last Team-SCP trial,
     * kept as members so the repair-selected log can report them outside the
     * solver's scope. */
    int team_result_mode_t_only_{0};
    double team_result_margin_before_{-std::numeric_limits<double>::infinity()};
    std::string team_refinement_solver_reason_;
    double team_refinement_solve_ms_{0.0};
    double team_refinement_total_ms_{0.0};
    std::vector<double> team_ref_solve_ms_samples_;
    std::vector<double> team_ref_total_ms_samples_;
    std::vector<double> team_ref_updates_samples_;
    std::vector<double> team_ref_pt_solve_ms_samples_;
    std::vector<double> team_ref_pt_total_ms_samples_;
    std::vector<double> team_ref_t_solve_ms_samples_;
    std::vector<double> team_ref_t_total_ms_samples_;

    /* Counters for the adoption funnel and for the two protections. */
    unsigned long team_ref_started_{0};
    unsigned long team_ref_solver_pass_{0};
    unsigned long team_ref_adopted_{0};
    unsigned long team_ref_activated_{0};
    unsigned long team_ref_coalesced_{0};
    unsigned long team_ref_no_longer_needed_{0};
    unsigned long team_ref_stale_anchor_{0};
    unsigned long team_ref_latest_recheck_reject_{0};
    unsigned long team_ref_too_late_{0};
    unsigned long team_ref_solver_infeasible_{0};
    unsigned long local_quality_overwrite_blocked_{0};
    unsigned long local_safety_override_team_{0};
    unsigned long team_ref_zero_modification_{0};
    unsigned long team_ref_current_baseline_safe_{0};
    unsigned long team_ref_execution_samples_{0};
    double team_ref_execution_duration_sum_{0.0};
    std::vector<double> team_ref_execution_durations_;

    /* A Team-refined future that has actually been adopted carries a light
     * certificate describing the visibility improvement it was accepted for.
     * It is not a state machine: it expires automatically as soon as the world
     * obligation it refers to is gone. */
    struct TeamImprovementCertificate
    {
      bool valid{false};
      std::uint64_t team_solution_id{0};
      int trajectory_id{-1};
      double activation{0.0};
      double critical_begin{0.0};
      double critical_end{0.0};
      double m2_before{0.0};
      double required_margin{0.0};
      std::uint64_t world_revision{0};
    } team_improvement_certificate_;

    /* True when the pending Local successor is required by safety or liveness
     * rather than being an ordinary quality update.  Reuses the existing
     * authority of the Local lifecycle; it is not a new heuristic. */
    bool localSuccessorIsSafetyRequired(std::string &reason) const;
    void reportTeamRefinementAudit(bool final_report) const;
    /* Feedback100: emit the full forecast->activation latency chain for the
     * current refinement on EVERY terminal outcome (adopted / infeasible /
     * not-selected / zero-modification), so a repair event that finds no
     * feasible candidate is still fully measurable. */
    void logRelayLatency(const char *outcome) const;
    void topologyResultCallback(
        const traj_utils::TopologyCoordinationConstPtr &message);
    bool teamProposalBoundaryMatches(const poly_traj::Trajectory &proposed,
        const poly_traj::Trajectory &source_suffix,double activation,
        bool require_source_tail=true) const;
    void teamTrajectorySolutionCallback(
        const traj_utils::TeamTrajectorySolutionConstPtr &message);
    void teamReferenceScheduleCallback(
        const traj_utils::TeamReferenceScheduleConstPtr &message);
    /* 阶段 B：CooperativeTaskReference 消费（identity 复核 + 日志）。 */
    void taskReferenceCallback(
        const traj_utils::CooperativeTaskReferenceConstPtr &message);
    /* Legacy wire telemetry only. Local K3 authority is created from its own
     * margin crossing and current raw visibility witness. */
    void topologyEscalationCallback(
        const traj_utils::TopologyEscalationConstPtr &message);
    // K3 Local escalation bookkeeping (telemetry + cross-cycle side
    // preference only; it never gates or alters hard admission).
    struct K3EscalationEventState
    {
      bool active{false};
      std::uint64_t escalation_id{0};
      std::uint64_t contract_id{0};
      ros::Time window_begin;
      ros::Time window_end;
      ros::WallTime received_wall;
      double required_margin{0.0};
      std::uint8_t detect_limiter{0};
      double margin_at_detect{0.0};
      double k2_at_detect{0.0};
      double k3_at_detect{0.0};
      std::uint64_t target_prediction_revision{0};
      bool outside_normal_authority{false};
      int cycle_count{0};
      int grant_count{0};
      int side_trigger_count{0};
      int side_both_failed_count{0};
      int progress_candidate_count{0};
      int progress_selected_count{0};
      int progress_committed_count{0};
      // Feedback116:窗口指标改善的 cycle 计数(仅遥测)。
      int c3_improved_cycles{0};
      int d3_improved_cycles{0};
      int progress_activated_count{0};
      bool result_emitted{false};
      // Cross-cycle light side preference: the last committed side and its
      // window deficit for the SAME event.  A tie in C3/D3 keeps this side;
      // any strictly better side still wins (no hard lock).
      int last_committed_side{-1};  // CandidateKind as int; -1 = none
      double last_committed_d3{0.0};
      // First executed binary self-visibility recovery inside the window.
      bool recovered{false};
      double recovery_world_time{0.0};
      int recovery_observed_traj_id{0};
      // Pending progress commit waiting for activation confirmation on the
      // next cycle (traj_id match against the executed trajectory).
      bool pending_activation_check{false};
      int pending_traj_id{0};
      int pending_candidate_id{-1};
      double pending_c3{0.0};
      double pending_d3{0.0};
      double pending_k2{0.0};
      double pending_c3_before{0.0};
      double pending_d3_before{0.0};
      bool activated_progress{false};
      double activated_k2{0.0};
      double activated_c3_before{0.0};
      double activated_c3_after{0.0};
      double activated_d3_before{0.0};
      double activated_d3_after{0.0};
      bool blocker_bound{false};
      bool blocker_dynamic{false};
      int blocker_id{-1};
      // 第三轮 authority 清理:事件由 Local 自主检测创建/刷新。window_begin =
      // margin 首次下穿 trigger 的世界时间(不再等 binary loss);窗口每 cycle
      // 重预测,直到 predicted_recover + 一个 sample cell。旧 tail_* 机制由
      // "margin 锚定窗口 + 每 cycle 刷新"取代,字段退役。
      bool local_origin{false};
      double predicted_binary_loss_time{0.0};
      double predicted_recover_time{0.0};
      bool m2_preempted{false};
    };
    K3EscalationEventState k3_event_;
    bool k3_local_escalation_enabled_{false};
    double k3_margin_trigger_{0.20};
    std::uint64_t k3_local_event_serial_{0};
    int k3_team_escalation_merged_count_{0};
    // 逐事件结果行闭合 + 窗口内新鲜执行里程计的二值恢复检查(仅遥测簿记)。
    void closeK3EscalationEvent(const char *close_reason);
    void checkK3EscalationRecovery();
    void confirmK3LocalActivation();
    // Local 自主 K3 检测/刷新(§6):每个 rolling cycle 在已执行基线 horizon 上
    // 用统一 visibility sample 扫描 margin 首次下穿;不依赖 Team escalation。
    void updateLocalK3RecoveryEvent();
    // Cached baseline window metrics for the current finalize batch (computed
    // once per batch from the executed trajectory, reused by the comparator).
    bool k3_baseline_valid_{false};
    double k3_baseline_window_k2_{0.0};
    double k3_baseline_window_d3_{0.0};
    double k3_baseline_window_c3_{0.0};
    std::uint64_t latestTaskReferenceGeneration() const
    {
      return task_reference_generation_;
    }
    void publishTeamTrajectoryAck(
        const traj_utils::TeamTrajectorySolution &solution,
        unsigned long planning_generation, int source_candidate_id,
        bool accepted, const std::string &reason);
    struct TeamReferenceSideCertificate
    {
      bool active{false};
      Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
      Eigen::Vector3d direction{Eigen::Vector3d::Zero()};
      int sign{0};
      double offset{0.0};
      double conflict_progress{0.5};
      double guidance_window{1.0};
    };
    void publishTeamReferenceAck(
        const traj_utils::TeamReferenceSchedule &schedule,
        unsigned long planning_generation, int source_candidate_id,
        int topology_kind, bool accepted, const std::string &reason,
        const poly_traj::MinJerkOpt *realized = nullptr,
        double realization_latency_ms = 0.0,
        const TeamReferenceSideCertificate *side_certificate = nullptr);
    void publishTeamCommitRevoke(const std::string &reason);
    bool hasCommittedPendingTeamTail() const;
    bool revalidateCommittedPendingTeamTail(std::string &reason) const;
    void displayTrajectoryAuthority(const poly_traj::Trajectory &trajectory,
                                    double start_time, double validated_end,
                                    int marker_id);
    bool stageTopologyCoordination(
        CandidateResult &nominal, CandidateResult &plus, CandidateResult &minus,
        const CandidateResult &local_selected,
        const Eigen::Vector3d &target_position,
        const Eigen::Vector3d &target_velocity,
        const Eigen::Vector3d &relative_tracking,
        bool touch_goal, double commit_head_freshness_threshold,
        double planning_epoch);
    TopologyProcessStatus finalizeTopologyCandidate(
        CandidateResult candidate, const DynamicRiskInfo &nominal_risk,
        int nominal_obstacle_id, bool touch_goal,
        double commit_head_freshness_threshold, double planning_epoch,
        bool coordinated, const char *selection_reason);
    void fillCandidateBinaryVisibility(const poly_traj::Trajectory &trajectory,
        double trajectory_start_time,double horizon,const Eigen::Vector3d &target_position,
        const Eigen::Vector3d &target_velocity,double target_state_time,
        traj_utils::TopologyCandidate &message) const;
    void fillTopologyCandidateMessage(
        const CandidateResult &candidate, double trajectory_start_time,
        double target_state_time, double evaluation_horizon,
        const Eigen::Vector3d &target_position,
        const Eigen::Vector3d &target_velocity,
        traj_utils::TopologyCandidate &message) const;
    double planningToCommitMedian() const;
    void recordPlanningToCommitDelay(double delay);

    struct VisibilityOdomState
    {
      bool valid{false};
      bool header_stamp_valid{false};
      double stamp{0.0};
      Eigen::Vector3d position{Eigen::Vector3d::Zero()};
      Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
      double yaw{0.0};
      double yaw_rate{0.0};
    };
    std::array<VisibilityOdomState, 3> visibility_odom_;
    std::array<ros::Subscriber, 3> visibility_odom_subs_;
    multi_uav_formation::TrackingCameraContract tracking_camera_;
    void visibilityOdomCallback(const nav_msgs::OdometryConstPtr &msg, int drone_id);

    ros::NodeHandle node_;
    traj_utils::AblationConfig ablation_config_;
    bool formal_ablation_mode_{false};
    bool stale_head_replan_requested_{false};
    Eigen::Vector3d stale_head_odom_pos_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d stale_head_odom_vel_{Eigen::Vector3d::Zero()};

    // Cross-cycle accepted nonlinear seed.  This cache is populated only
    // after the selected trajectory passes post-checks and is committed to
    // the execution container.  It is intentionally separate from the
    // KEEP_PREVIOUS_SAFE fallback state.
    bool accepted_state_cache_valid_{false};
    poly_traj::Trajectory accepted_state_cache_traj_;
    double accepted_state_cache_time_{0.0};
    CandidateKind accepted_state_cache_kind_{CandidateKind::NOMINAL};
    CandidateSafetyClass accepted_state_cache_safety_class_{
        CandidateSafetyClass::INVALID};
    int accepted_state_cache_obstacle_id_{-1};
    unsigned long accepted_state_cache_generation_{0};
    unsigned long accepted_state_cache_id_{0};
    std::deque<double> planning_to_commit_history_;
    mutable unsigned long init_reuse_count_{0};
    mutable unsigned long fresh_initializer_count_{0};
    mutable unsigned long initializer_retimed_count_{0};
    mutable unsigned long initializer_rejected_preopt_count_{0};
    mutable unsigned long old_remaining_time_reused_with_new_target_count_{0};
    mutable unsigned long stale_shrinking_initializer_event_count_{0};
    mutable unsigned long target_side_astar_count_{0};
    mutable unsigned long outer_side_astar_count_{0};
    mutable unsigned long unbiased_astar_count_{0};

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner

#endif
