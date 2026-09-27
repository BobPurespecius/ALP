#include <traj_utils/trajectory_lifecycle.h>
#include <fstream>
#include <iomanip>
#include <plan_manage/local_sfc_boundary_scan.h>
#include <plan_manage/local_static_geometry_contract.h>
#include <plan_manage/local_visibility_preference.h>
#include <plan_manage/planner_manager.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <numeric>
#include <nav_msgs/Odometry.h>
#include <ros/topic.h>
#include <sstream>
#include <thread>
#include <boost/bind.hpp>
#include "visualization_msgs/Marker.h" // zx-todo

namespace ego_planner
{

  namespace
  {
    constexpr double kCandidateSideOffset = 0.7;

    std::uint64_t trajectoryPayloadHash(const poly_traj::Trajectory &trajectory) {
      if (trajectory.getPieceNum() <= 0) return 0;
      std::uint64_t hash = 1469598103934665603ULL;
      const auto mix = [&hash](const double value) {
        std::uint64_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        hash ^= bits;
        hash *= 1099511628211ULL;
      };
      mix(static_cast<double>(trajectory.getPieceNum()));
      for (int piece = 0; piece < trajectory.getPieceNum(); ++piece) {
        mix(trajectory[piece].getDuration());
        const auto &coefficients = trajectory[piece].getCoeffMat();
        for (int row = 0; row < coefficients.rows(); ++row)
          for (int col = 0; col < coefficients.cols(); ++col)
            mix(coefficients(row, col));
      }
      return hash == 0 ? 1 : hash;
    }

    // The same finite initializer serves fresh, N, and SIDE routes.  Every
    // correction regenerates MINCO with the unchanged real boundary P/V/A;
    // the dilation ratio is only a proposed next duration, never a safety
    // certificate for the regenerated polynomial.
    bool initializePvaTiming(
        const Eigen::Matrix3d &head, const Eigen::Matrix3d &tail,
        const Eigen::MatrixXd &inner, Eigen::VectorXd &durations,
        const double velocity_limit, const double acceleration_limit,
        const double jerk_limit,
        const std::function<bool(const poly_traj::Trajectory &, double &,
                                 double &, double &)> &evaluate,
        poly_traj::MinJerkOpt &seed, double &max_velocity,
        double &max_acceleration, double &max_jerk, int &iterations) {
      iterations = 0;
      if (!head.allFinite() || !tail.allFinite() || !inner.allFinite() ||
          durations.size() < 1 || inner.rows() != 3 ||
          inner.cols() != durations.size() - 1 ||
          !durations.allFinite() || (durations.array() <= 0.0).any() ||
          !std::isfinite(velocity_limit) || velocity_limit <= 0.0 ||
          !std::isfinite(acceleration_limit) || acceleration_limit <= 0.0 ||
          !std::isfinite(jerk_limit) || jerk_limit <= 0.0 ||
          head.col(1).norm() > velocity_limit + 1.0e-6 ||
          tail.col(1).norm() > velocity_limit + 1.0e-6 ||
          head.col(2).norm() > acceleration_limit + 1.0e-6 ||
          tail.col(2).norm() > acceleration_limit + 1.0e-6)
        return false;
      // The existing fresh-route floor prevents a 0.1 s guide subdivision
      // from producing a singular PVA interpolation.  Piece geometry and
      // SFC ownership remain unchanged.
      durations = durations.array().max(0.10).matrix();
      constexpr int kMaxIterations = 8;
      for (int attempt = 0; attempt < kMaxIterations; ++attempt) {
        iterations = attempt + 1;
        if (durations.sum() > 120.0 || !durations.allFinite()) return false;
        seed.reset(head, tail, durations.size());
        seed.generate(inner, durations);
        const poly_traj::Trajectory trajectory = seed.getTraj();
        if (trajectory.getPieceNum() != durations.size() ||
            !std::isfinite(trajectory.getTotalDuration()) ||
            trajectory.getTotalDuration() <= 1.0e-6) return false;
        const bool feasible = evaluate(trajectory, max_velocity,
                                       max_acceleration, max_jerk);
        if (!std::isfinite(max_velocity) || !std::isfinite(max_acceleration) ||
            !std::isfinite(max_jerk)) return false;
        if (feasible) return true;
        const double scale = std::max({1.05,
            max_velocity / velocity_limit,
            std::sqrt(std::max(0.0, max_acceleration / acceleration_limit)),
            std::cbrt(std::max(0.0, max_jerk / jerk_limit))});
        durations *= std::min(3.0, 1.05 * scale);
      }
      return false;
    }

    std::uint64_t stableSpatialContractIdentity(
        const Eigen::Vector3d &origin, const Eigen::Vector3d &size,
        const double resolution)
    {
      std::uint64_t hash = 1469598103934665603ULL;
      const auto mix = [&hash](const double value) {
        const std::int64_t quantized = static_cast<std::int64_t>(
            std::llround(value * 1.0e6));
        hash ^= static_cast<std::uint64_t>(quantized);
        hash *= 1099511628211ULL;
      };
      for (int axis = 0; axis < 3; ++axis)
      {
        mix(origin(axis));
        mix(size(axis));
      }
      mix(resolution);
      return hash == 0 ? 1 : hash;
    }

    // Return an exact polynomial reparameterization q(u) = p(offset + u)
    // for a sub-interval of one quintic Piece.  This is used only to retain
    // the already published trajectory from the current execution time; it
    // does not alter any planner or optimizer mathematics.
    poly_traj::Piece shiftPiece(const poly_traj::Piece &piece,
                                const double offset, const double duration)
    {
      const auto &src = piece.getCoeffMat();
      poly_traj::CoefficientMat dst = poly_traj::CoefficientMat::Zero();
      for (int n = 0; n <= 5; ++n)
      {
        const Eigen::Vector3d coeff = src.col(5 - n);
        for (int k = 0; k <= n; ++k)
        {
          double binom = 1.0;
          for (int r = 1; r <= k; ++r)
            binom *= static_cast<double>(n - (k - r)) / static_cast<double>(r);
          dst.col(5 - k) += coeff * binom * std::pow(offset, n - k);
        }
      }
      return poly_traj::Piece(duration, dst);
    }

    bool sliceTrajectory(const poly_traj::Trajectory &source, const double elapsed,
                         poly_traj::Trajectory &remaining)
    {
      remaining.clear();
      const double total = source.getTotalDuration();
      if (source.getPieceNum() <= 0 || !std::isfinite(elapsed) ||
          elapsed < 0.0 || elapsed >= total - 1.0e-6)
        return false;

      double skip = elapsed;
      for (int i = 0; i < source.getPieceNum(); ++i)
      {
        const poly_traj::Piece &piece = source[i];
        const double dur = piece.getDuration();
        if (skip >= dur - 1.0e-9)
        {
          skip -= dur;
          continue;
        }
        const double offset = std::max(0.0, skip);
        const double piece_remaining = dur - offset;
        if (piece_remaining > 1.0e-9)
          remaining.emplace_back(shiftPiece(piece, offset, piece_remaining));
        skip = 0.0;
      }
      return remaining.getPieceNum() > 0 && remaining.getTotalDuration() > 1.0e-6;
    }

    // Feedback126: prefixTrajectory was removed — its only consumer was the
    // legacy external Team polynomial proposal path, which is deleted.

    void trajectoryToMINCOMessage(const poly_traj::Trajectory &trajectory,
                                  const int drone_id, const int trajectory_id,
                                  const double start_time,
                                  traj_utils::MINCOTraj &message)
    {
      const int piece_count = trajectory.getPieceNum();
      message.drone_id = drone_id;
      message.traj_id = trajectory_id;
      message.start_time = ros::Time(start_time);
      message.order = 5;
      message.duration.resize(piece_count);
      const Eigen::VectorXd durations = trajectory.getDurations();
      Eigen::Vector3d value = trajectory.getPos(0.0);
      for (int axis = 0; axis < 3; ++axis)
        message.start_p[axis] = value(axis);
      value = trajectory.getVel(0.0);
      for (int axis = 0; axis < 3; ++axis)
        message.start_v[axis] = value(axis);
      value = trajectory.getAcc(0.0);
      for (int axis = 0; axis < 3; ++axis)
        message.start_a[axis] = value(axis);
      const double duration = trajectory.getTotalDuration();
      value = trajectory.getPos(duration);
      for (int axis = 0; axis < 3; ++axis)
        message.end_p[axis] = value(axis);
      value = trajectory.getVel(duration);
      for (int axis = 0; axis < 3; ++axis)
        message.end_v[axis] = value(axis);
      value = trajectory.getAcc(duration);
      for (int axis = 0; axis < 3; ++axis)
        message.end_a[axis] = value(axis);
      message.inner_x.resize(std::max(0, piece_count - 1));
      message.inner_y.resize(std::max(0, piece_count - 1));
      message.inner_z.resize(std::max(0, piece_count - 1));
      const Eigen::MatrixXd positions = trajectory.getPositions();
      for (int piece = 0; piece < piece_count - 1; ++piece)
      {
        message.inner_x[piece] = positions(0, piece + 1);
        message.inner_y[piece] = positions(1, piece + 1);
        message.inner_z[piece] = positions(2, piece + 1);
      }
      for (int piece = 0; piece < piece_count; ++piece)
        message.duration[piece] = durations(piece);
    }

    bool mincoMessageToOptimizer(const traj_utils::MINCOTraj &message,
                                 poly_traj::MinJerkOpt &optimizer)
    {
      if (message.order != 5 || message.duration.empty() ||
          message.duration.size() != message.inner_x.size() + 1 ||
          message.inner_x.size() != message.inner_y.size() ||
          message.inner_x.size() != message.inner_z.size())
        return false;
      const int pieces = static_cast<int>(message.duration.size());
      Eigen::Matrix3d head;
      Eigen::Matrix3d tail;
      head << message.start_p[0], message.start_v[0], message.start_a[0],
          message.start_p[1], message.start_v[1], message.start_a[1],
          message.start_p[2], message.start_v[2], message.start_a[2];
      tail << message.end_p[0], message.end_v[0], message.end_a[0],
          message.end_p[1], message.end_v[1], message.end_a[1],
          message.end_p[2], message.end_v[2], message.end_a[2];
      Eigen::MatrixXd points(3, pieces - 1);
      Eigen::VectorXd durations(pieces);
      for (int piece = 0; piece < pieces - 1; ++piece)
        points.col(piece) << message.inner_x[piece], message.inner_y[piece],
            message.inner_z[piece];
      for (int piece = 0; piece < pieces; ++piece)
      {
        durations(piece) = message.duration[piece];
        if (!std::isfinite(durations(piece)) || durations(piece) <= 1.0e-6)
          return false;
      }
      if (!head.allFinite() || !tail.allFinite() || !points.allFinite())
        return false;
      optimizer.reset(head, tail, pieces);
      optimizer.generate(points, durations);
      return optimizer.getTraj().getPieceNum() == pieces &&
             std::isfinite(optimizer.getTraj().getTotalDuration());
    }

// 本轮修复（动态 LOS world-time 语义）：把"世界时间锚定平面"的相对时间区间
// 从一个世界时间锚点换算到另一个锚点。
//   t_rel_new = (t_rel_old + anchor_old) - anchor_new
// 普通（静态进度型）平面不受影响。若换算后区间完全落到新原点之前，则返回 false
// 表示该平面在新锚点下已无有效作用区间。
static bool rebaseWorldTimeAnchoredPlane(LocalSfcPlane &plane,
                                         const double new_anchor_time)
{
  if (!plane.world_time_anchored)
    return true;
  if (!std::isfinite(plane.world_anchor_time) ||
      !std::isfinite(new_anchor_time))
    return false;
  if (std::abs(new_anchor_time - plane.world_anchor_time) <= 1.0e-9)
    return true;
  const double start_world = plane.world_anchor_time + plane.active_start;
  const double end_world = plane.world_anchor_time + plane.active_end;
  plane.active_start = start_world - new_anchor_time;
  plane.active_end = end_world - new_anchor_time;
  plane.world_anchor_time = new_anchor_time;
  if (plane.active_end <= 1.0e-6)
    return false;
  if (plane.active_start < 0.0)
    plane.active_start = 0.0;
  return plane.active_end > plane.active_start + 1.0e-6;
}

    const char *candidateKindName(const EGOPlannerManager::CandidateKind kind)
    {
      switch (kind)
      {
      case EGOPlannerManager::CandidateKind::SIDE_PLUS:
        return "SIDE_PLUS";
      case EGOPlannerManager::CandidateKind::SIDE_MINUS:
        return "SIDE_MINUS";
      default:
        return "NOMINAL";
      }
    }

    const char *candidateSafetyClassName(
        const EGOPlannerManager::CandidateSafetyClass safety_class)
    {
      switch (safety_class)
      {
      case EGOPlannerManager::CandidateSafetyClass::ABSOLUTE_SAFE:
        return "ABSOLUTE_SAFE";
      default:
        return "INVALID";
      }
    }

    bool segmentIntersectsVerticalCylinder(const Eigen::Vector3d &start,
                                           const Eigen::Vector3d &end,
                                           const Eigen::Vector3d &center,
                                           const double radius,
                                           const double height)
    {
      const Eigen::Vector3d delta = end - start;
      const double sx = start.x() - center.x();
      const double sy = start.y() - center.y();
      const double radial_a = delta.x() * delta.x() + delta.y() * delta.y();
      const double radial_b = 2.0 * (sx * delta.x() + sy * delta.y());
      const double radial_c = sx * sx + sy * sy - radius * radius;
      double radial_t0 = 0.0;
      double radial_t1 = 1.0;
      if (radial_a < 1.0e-9)
      {
        if (radial_c > 0.0)
          return false;
      }
      else
      {
        const double discriminant = radial_b * radial_b -
                                    4.0 * radial_a * radial_c;
        if (discriminant < 0.0)
          return false;
        const double root = std::sqrt(std::max(0.0, discriminant));
        radial_t0 = std::max(0.0, (-radial_b - root) / (2.0 * radial_a));
        radial_t1 = std::min(1.0, (-radial_b + root) / (2.0 * radial_a));
        if (radial_t0 > radial_t1)
          return false;
      }

      const double z_min = center.z() - 0.5 * height;
      const double z_max = center.z() + 0.5 * height;
      double z_t0 = 0.0;
      double z_t1 = 1.0;
      if (std::abs(delta.z()) < 1.0e-9)
      {
        if (start.z() < z_min || start.z() > z_max)
          return false;
      }
      else
      {
        z_t0 = std::max(0.0, std::min((z_min - start.z()) / delta.z(),
                                     (z_max - start.z()) / delta.z()));
        z_t1 = std::min(1.0, std::max((z_min - start.z()) / delta.z(),
                                     (z_max - start.z()) / delta.z()));
        if (z_t0 > z_t1)
          return false;
      }
      return std::max(radial_t0, z_t0) <= std::min(radial_t1, z_t1);
    }

    // Keeps all optimizer calls in one candidate batch on one absolute
    // prediction epoch, and clears the override on every return path.
    class MovingObjPredictionEpochScope
    {
    public:
      MovingObjPredictionEpochScope(PolyTrajOptimizer *optimizer, const double epoch,
                                     const bool enabled = true)
          : optimizer_(enabled ? optimizer : nullptr)
      {
        if (optimizer_ != nullptr)
          optimizer_->setMovingObjPredictionEpoch(epoch);
      }

      ~MovingObjPredictionEpochScope()
      {
        if (optimizer_ != nullptr)
          optimizer_->clearMovingObjPredictionEpoch();
      }

      MovingObjPredictionEpochScope(const MovingObjPredictionEpochScope &) = delete;
      MovingObjPredictionEpochScope &operator=(const MovingObjPredictionEpochScope &) = delete;

    private:
      PolyTrajOptimizer *optimizer_;
    };

    class CandidateSideScope
    {
    public:
      CandidateSideScope(PolyTrajOptimizer *optimizer,
                         const Eigen::Vector3d &start,
                         const Eigen::Vector3d &end,
                         const int side,
                         const double offset,
                         const double conflict_progress = 0.5,
                         const double guidance_window = 1.0,
                         const bool enable_region_constraint = true,
                         const Eigen::Vector3d *reference_direction = nullptr)
          : optimizer_(optimizer)
      {
        if (optimizer_ != nullptr)
        {
          if (reference_direction != nullptr)
            optimizer_->setCandidateSideBiasReference(
                start, end, *reference_direction, side, offset,
                conflict_progress, guidance_window, enable_region_constraint);
          else
            optimizer_->setCandidateSideBias(start, end, side, offset,
                                              conflict_progress, guidance_window,
                                              enable_region_constraint);
        }
      }

      ~CandidateSideScope()
      {
        if (optimizer_ != nullptr)
          optimizer_->clearCandidateSideBias();
      }

      CandidateSideScope(const CandidateSideScope &) = delete;
      CandidateSideScope &operator=(const CandidateSideScope &) = delete;

    private:
      PolyTrajOptimizer *optimizer_;
    };

  }

  // SECTION interfaces for setup and query

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() { std::cout << "des manager" << std::endl; }

  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    node_ = nh;
    ablation_config_ = traj_utils::loadAblationConfig(
        "planner_manager", &formal_ablation_mode_);
    if (formal_ablation_mode_)
    {
      bool encirclement_enabled = false;
      bool cooperative_reference_enabled = false;
      nh.param("optimization/enable_encirclement_tracking",
               encirclement_enabled, false);
      nh.param("fsm/enable_cooperative_viewpoint_reference",
               cooperative_reference_enabled, false);
      if (!encirclement_enabled)
        throw std::invalid_argument(
            "ENCIRCLEMENT_MUST_REMAIN_ENABLED_FOR_FORMAL_ABLATION");
      if (!cooperative_reference_enabled)
        throw std::invalid_argument(
            "COOPERATIVE_REFERENCE_MUST_REMAIN_ENABLED_FOR_FORMAL_ABLATION");
    }
    execution_adoption_sub_=nh.subscribe<traj_utils::PolyTraj>("/trajectory_execution/activated",30,
        [this](const traj_utils::PolyTrajConstPtr &message) {
          if(message->drone_id==pp_.drone_id && message->generation>=executed_generation_)
          {
            executed_traj_id_=message->traj_id; executed_generation_=message->generation;
            // Feedback093: ONLY a confirmed executor ACTIVATION may move the
            // Local Planner's authoritative predecessor.  A Team COMMIT never
            // reaches this point, so a speculative Team future can no longer
            // become the predecessor of the Local rolling chain.
            const PredecessorRecord *record=findPredecessorRecord(message->traj_id);
            const int previous_authority=authoritative_predecessor_id_;
            if(record)
            {
              authoritative_predecessor_=record->data;
              authoritative_predecessor_valid_=true;
              authoritative_predecessor_id_=record->data.traj_id;
              authoritative_predecessor_generation_=record->generation;
              authoritative_predecessor_source_=record->source;
              ++authority_switch_count_;
              // A Team-refined future reaches the executor through the 2PC
              // channel under the disjoint Team speculative id namespace, so
              // the confirmation must be matched on BOTH namespaces.
              const bool certificate_activated =
                  team_improvement_certificate_.valid &&
                  (message->traj_id==team_improvement_certificate_.trajectory_id ||
                   message->traj_id==kTeamSpeculativeTrajectoryIdBase+
                       static_cast<int>(
                           team_improvement_certificate_.team_solution_id));
              if(certificate_activated)
              {
                ++team_ref_activated_;
                team_ref_execution_durations_.push_back(
                    std::max(0.0,ros::Time::now().toSec()-
                        team_improvement_certificate_.activation));
                ROS_INFO("[TEAM_REFINEMENT_ACTIVATED] drone=%d "
                         "team_solution_id=%lu trajectory_id=%d "
                         "activation=%.9f certificate_valid=1",
                    pp_.drone_id,
                    static_cast<unsigned long>(
                        team_improvement_certificate_.team_solution_id),
                    message->traj_id,
                    team_improvement_certificate_.activation);
              }
              ROS_INFO("[planner-authority-switch] drone=%d old_predecessor=%d new_predecessor=%d reason=EXECUTOR_CONFIRMED_ACTIVATION source=%s generation=%lu lineage_class=%s",
                  pp_.drone_id,previous_authority,message->traj_id,
                  message->trajectory_source.c_str(),
                  static_cast<unsigned long>(message->generation),
                  record->team_speculative?"TEAM":"LOCAL");
            }
            else
            {
              ++authority_lookup_miss_count_;
              ROS_WARN("[planner-authority-switch] drone=%d old_predecessor=%d new_predecessor=%d reason=EXECUTOR_CONFIRMED_ACTIVATION_BUT_LOCAL_COPY_MISSING source=%s lookup_miss_total=%lu",
                  pp_.drone_id,previous_authority,message->traj_id,
                  message->trajectory_source.c_str(),
                  static_cast<unsigned long>(authority_lookup_miss_count_));
            }
            if(message->traj_id==pending_quality_upgrade_traj_id_)
            {
              ROS_INFO("[coverage-quality] event=BETTER_CANDIDATE_ACTIVATED drone=%d trajectory_id=%d candidate=%d generation=%lu",
                  pp_.drone_id,message->traj_id,pending_quality_upgrade_candidate_id_,
                  static_cast<unsigned long>(message->generation));
              pending_quality_upgrade_traj_id_=-1;
              pending_quality_upgrade_candidate_id_=-1;
            }
          }
        });
    // Feedback098: the Planner owns its own future; the forecast it publishes
    // must cover a horizon the Coordinator can fuse without extrapolation.
    nh.param("manager/visibility_forecast_horizon",
             visibility_forecast_horizon_, 2.0);
    nh.param("manager/visibility_forecast_dt", visibility_forecast_dt_, 0.1);
    nh.param("/alp/team_k3_repair_enabled", team_k3_repair_enabled_, false);
    // K3 Local escalation(2026-09-25 第二轮):唯一 canonical 开关。OFF 时
    // escalation-specific early authority 与 C3/D3 comparator 全部关闭,
    // 普通 LOS N/L/R、M2 Team repair 与 J_vis 不受影响。
    nh.param("manager/k3_local_escalation_enabled",
             k3_local_escalation_enabled_, false);
    // K3 margin 触发线:与 coordinator 的 predictive_relay_required_margin 同名
    // 同源(全局参数,同一常量,不新增第二套阈值)。
    nh.param("predictive_relay_required_margin", k3_margin_trigger_, 0.20);
    if (!std::isfinite(k3_margin_trigger_) || k3_margin_trigger_ < -1.0 ||
        k3_margin_trigger_ > 1.0)
      k3_margin_trigger_ = 0.20;
    ROS_INFO("[k3-local-escalation-config] drone=%d enabled=%d "
             "margin_trigger=%.3f",
             pp_.drone_id, static_cast<int>(k3_local_escalation_enabled_),
             k3_margin_trigger_);
    // Feedback101: ONE new parameter.  Both repair candidates use the same
    // weight; no other cost weight (J_track / encirclement / time) is touched.
    nh.param("manager/team_reserve_weight", team_reserve_weight_, 20.0);
    if (!std::isfinite(team_reserve_weight_) || team_reserve_weight_ < 0.0)
      team_reserve_weight_ = 0.0;
    ROS_INFO("[team-reserve-config] drone=%d team_reserve_weight=%.3f",
             pp_.drone_id, team_reserve_weight_);
    if (!std::isfinite(visibility_forecast_horizon_) ||
        visibility_forecast_horizon_ <= 0.0)
      visibility_forecast_horizon_ = 2.0;
    if (!std::isfinite(visibility_forecast_dt_) ||
        visibility_forecast_dt_ <= 1.0e-3)
      visibility_forecast_dt_ = 0.1;
    ROS_INFO("[relay-forecast-config] drone=%d horizon=%.3f dt=%.3f "
             "authority=PLANNER_LOCAL_BASELINE no_polynomial_extrapolation=1",
        pp_.drone_id, visibility_forecast_horizon_, visibility_forecast_dt_);
    nh.param("/trajectory_lifecycle/handoff_position_tolerance", handoff_position_tolerance_, 0.02);
    nh.param("/trajectory_lifecycle/handoff_velocity_tolerance", handoff_velocity_tolerance_, 0.05);
    nh.param("/trajectory_lifecycle/handoff_acceleration_tolerance", handoff_acceleration_tolerance_, 0.10);
    nh.param("manager/local_activation_margin", local_activation_margin_, 0.10);
    if (handoff_position_tolerance_ <= 0 || handoff_velocity_tolerance_ <= 0 ||
        handoff_acceleration_tolerance_ <= 0 || local_activation_margin_ < 0.10)
      throw std::invalid_argument("Invalid trajectory lifecycle safety configuration");
    nh.param("/trajectory_lifecycle/execution_margin", execution_margin_, 0.15);
    // 本轮修复 1/2：合围软方位恢复（相对 tracking 权重的倍数）与前缀加强倍数。
    nh.param("manager/encirclement_bearing_recovery_weight",
             encirclement_bearing_recovery_weight_, 0.08);
    nh.param("manager/encirclement_bearing_prefix_boost",
             encirclement_bearing_prefix_boost_, 2.0);
    if (encirclement_bearing_prefix_boost_ < 1.0)
      encirclement_bearing_prefix_boost_ = 1.0;
    double min_deg=25,max_deg=170;
    nh.param("/encirclement_geometry/min_angular_separation_deg",min_deg,25.0);
    nh.param("/encirclement_geometry/max_circular_gap_deg",max_deg,170.0);
    geometry_policy_.minimum=min_deg*M_PI/180.0;geometry_policy_.maximum=max_deg*M_PI/180.0;
    nh.param("/encirclement_geometry/local_horizon",geometry_policy_.horizon,1.5);
    nh.param("/encirclement_geometry/local_dt",geometry_policy_.dt,0.10);
    nh.param("/encirclement_geometry/peer_fresh_age",
             geometry_policy_.peer_fresh_age, 0.45);
    geometry_policy_.validate();
    nh.param("/trajectory_lifecycle/planning_budget_initial",execution_budget_.initial,1.10);
    nh.param("/trajectory_lifecycle/planning_budget_min",execution_budget_.minimum,0.20);
    nh.param("/trajectory_lifecycle/planning_budget_max",execution_budget_.maximum,2.50);
    nh.param("/trajectory_lifecycle/planning_budget_factor",execution_budget_.factor,1.35);
    nh.param("/trajectory_lifecycle/planning_budget_half_life",execution_budget_.half_life,60.0);
    nh.param("/trajectory_lifecycle/execution_margin",execution_margin_,0.15);
    nh.param("/trajectory_lifecycle/uninterruptible_budget",uninterruptible_budget_,0.05);
    execution_budget_.validate();
    if(!std::isfinite(execution_margin_+uninterruptible_budget_) || execution_margin_<0.15 || uninterruptible_budget_<0.05)
      throw std::invalid_argument("Invalid execution reserve margins");
    /* read algorithm parameters */

    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/max_jer", pp_.max_jer_, -1.0);
    nh.param("manager/enable_risk_triggered_candidates", enable_risk_triggered_candidates_, false);
    nh.param("manager/enable_candidate_region_constraint",
             enable_candidate_region_constraint_, true);
    nh.param("manager/enable_side_local_astar_repair",
             enable_side_local_astar_repair_, true);
    nh.param("manager/risk_sample_dt", risk_sample_dt_, 0.10);
    // Keep the static occupancy inflation independent from the physical UAV
    // footprint used by the dynamic hard-collision gate.  The production iris
    // model has a 0.47 m square base and rotors at (+/-0.13,+/-0.22) m with
    // radius 0.128 m; the latter gives the conservative horizontal extent
    // sqrt(0.13^2+0.22^2)+0.128 = 0.3835 m.  This is exposed as a dedicated
    // parameter so a different deployed model can provide its measured
    // footprint without changing grid-map inflation.
    nh.param("manager/dynamic_hard_body_radius", dynamic_body_radius_, 0.384);
    dynamic_body_radius_ = std::max(0.0, dynamic_body_radius_);
    nh.param("manager/candidate_trigger_margin", candidate_trigger_margin_, 0.15);
    nh.param("manager/temporal_swarm_trigger_margin",
             temporal_swarm_trigger_margin_, 0.08);
    nh.param("manager/temporal_swarm_sample_dt",
             temporal_swarm_sample_dt_, 0.05);
    nh.param("manager/temporal_swarm_conflict_half_window",
             temporal_swarm_conflict_half_window_, 0.60);
    nh.param("manager/team_visibility_preferred_separation_deg",
             team_visibility_preferred_separation_deg_, 25.0);
    nh.param("manager/encirclement_spread_saturation_angle_deg",
             encirclement_spread_saturation_angle_deg_, 60.0);
    nh.param("manager/enable_joint_topology_coordination",
             enable_joint_topology_coordination_, false);
    nh.param("manager/enable_team_visibility_optimizer",
             enable_team_visibility_optimizer_, false);
    nh.param("manager/topology_coordination_timeout",
             topology_coordination_timeout_, 0.20);
    nh.param("manager/team_coordination_timeout",
             team_coordination_timeout_, 0.35);
    nh.param("manager/topology_result_max_age",
             topology_result_max_age_, 0.30);
    nh.param("manager/topology_min_active_remaining",
             topology_min_active_remaining_, 0.35);
    nh.param("manager/team_solution_max_age", team_solution_max_age_, 0.30);
    nh.param("manager/team_solution_min_activation_lead",
             team_solution_min_activation_lead_, 0.015);
    nh.param("manager/team_min_execution_time",
             team_min_execution_time_, 0.12);
    nh.param("manager/visibility_sample_dt", visibility_sample_dt_, 0.10);
    nh.param("manager/visibility_min_target_distance",
             visibility_min_target_distance_, 0.20);
    nh.param("manager/visibility_max_target_distance",
             visibility_max_target_distance_, 8.0);
    nh.param("manager/visibility_occlusion_margin",
             visibility_occlusion_margin_, 0.08);
    double camera_hfov_deg = 85.0;
    double camera_width = 1280.0;
    double camera_height = 720.0;
    nh.param("manager/camera_hfov_deg", camera_hfov_deg, 85.0);
    nh.param("manager/camera_image_width", camera_width, 1280.0);
    nh.param("manager/camera_image_height", camera_height, 720.0);
    tracking_camera_.horizontal_fov = camera_hfov_deg * M_PI / 180.0;
    tracking_camera_.vertical_fov =
        multi_uav_formation::verticalFovFromHorizontal(
            tracking_camera_.horizontal_fov, camera_width, camera_height);
    if (risk_sample_dt_ <= 0.0)
      risk_sample_dt_ = 0.10;
    if (candidate_trigger_margin_ < 0.0)
      candidate_trigger_margin_ = 0.0;
    visibility_sample_dt_ = std::max(0.02, visibility_sample_dt_);
    team_visibility_preferred_separation_deg_ = std::min(
        180.0, std::max(1.0, team_visibility_preferred_separation_deg_));
    encirclement_spread_saturation_angle_deg_ = std::min(
        180.0, std::max(1.0,
                        encirclement_spread_saturation_angle_deg_));
    topology_coordination_timeout_ =
        std::max(0.02, topology_coordination_timeout_);
    team_solution_max_age_ = std::max(0.05, team_solution_max_age_);
    team_solution_min_activation_lead_ =
        std::max(0.0, team_solution_min_activation_lead_);
    team_min_execution_time_ =
        std::min(0.50, std::max(0.0, team_min_execution_time_));
    team_coordination_timeout_ = std::max(
        topology_coordination_timeout_, team_coordination_timeout_);
    // A team proposal is produced only after the coordinator has evaluated
    // the joint nonlinear problem and transported the result back to all
    // planners.  Keep the local pending generation alive for one configured
    // proposal-age window beyond the ordinary coordination budget; otherwise
    // a valid proposal can arrive just after the local timeout and be
    // rejected as NO_PENDING_TOPOLOGY_GENERATION.  This changes only the
    // waiting/fallback budget, never a safety threshold or a trajectory
    // validity check.
    if (enable_team_visibility_optimizer_)
    {
      const double base_team_timeout = team_coordination_timeout_;
      team_coordination_timeout_ = base_team_timeout + team_solution_max_age_;
      ROS_INFO("[team-coordination-budget] optimizer=1 base_timeout=%.3f "
               "proposal_grace=%.3f effective_timeout=%.3f",
               base_team_timeout, team_solution_max_age_,
               team_coordination_timeout_);
    }
    topology_result_max_age_ = std::max(
        team_coordination_timeout_, topology_result_max_age_);
    topology_min_active_remaining_ = std::max(
        team_coordination_timeout_ + 0.05,
        topology_min_active_remaining_);
    visibility_min_target_distance_ =
        std::max(0.0, visibility_min_target_distance_);
    visibility_max_target_distance_ = std::max(
        visibility_min_target_distance_ + 1.0e-3,
        visibility_max_target_distance_);
    tracking_camera_.min_range = visibility_min_target_distance_;
    tracking_camera_.max_range = visibility_max_target_distance_;
    visibility_occlusion_margin_ = std::max(0.0, visibility_occlusion_margin_);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/polyTraj_piece_length", pp_.polyTraj_piece_length, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
    nh.param("manager/use_distinctive_trajs", pp_.use_distinctive_trajs, false);
    nh.param("manager/drone_id", pp_.drone_id, -1);

    for (int drone_id = 0; drone_id < 3; ++drone_id)
    {
      visibility_odom_subs_[drone_id] = nh.subscribe<nav_msgs::Odometry>(
          "/drone_" + std::to_string(drone_id) + "_visual_slam/odom",
          drone_id == pp_.drone_id ? 32 : 1,
          boost::bind(&EGOPlannerManager::visibilityOdomCallback, this, _1,
                      drone_id));
    }
    if (pp_.drone_id >= 0 && pp_.drone_id < 3)
    {
      topology_bundle_pub_ = nh.advertise<traj_utils::TopologyCandidateBundle>(
          "/topology_coordination/uav" + std::to_string(pp_.drone_id) +
              "/candidate_bundle",
          4);
      topology_result_sub_ = nh.subscribe<traj_utils::TopologyCoordination>(
          "/topology_coordination/result", 8,
          &EGOPlannerManager::topologyResultCallback, this,
          ros::TransportHints().tcpNoDelay());
      team_trajectory_ack_pub_ = nh.advertise<traj_utils::TeamTrajectoryAck>(
          "/topology_coordination/team_ack", 8);
      team_trajectory_solution_sub_ =
          nh.subscribe<traj_utils::TeamTrajectorySolution>(
              "/topology_coordination/team_solution", 8,
              &EGOPlannerManager::teamTrajectorySolutionCallback, this,
              ros::TransportHints().tcpNoDelay());
      team_reference_schedule_sub_ =
          nh.subscribe<traj_utils::TeamReferenceSchedule>(
              "/topology_coordination/team_reference_schedule", 8,
              &EGOPlannerManager::teamReferenceScheduleCallback, this,
              ros::TransportHints().tcpNoDelay());
      // 阶段 B：任务几何 authority 的消费者。Local 只用它做 identity 复核
      // 与日志，不改变 Local 的 first-safe commit 行为。
      task_reference_sub_ = nh.subscribe<traj_utils::CooperativeTaskReference>(
          "/cooperative_task_reference", 1,
          &EGOPlannerManager::taskReferenceCallback, this,
          ros::TransportHints().tcpNoDelay());
      topology_escalation_sub_ =
          nh.subscribe<traj_utils::TopologyEscalation>(
              "/topology_coordination/topology_escalation", 2,
              &EGOPlannerManager::topologyEscalationCallback, this,
              ros::TransportHints().tcpNoDelay());
    }

    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);
    side_a_star_.reset(new AStar);
    side_a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));
    ROS_INFO("[side-astar-repair] enabled=%d resolution=%.3f",
             static_cast<int>(enable_side_local_astar_repair_),
             grid_map_->getResolution());
    ROS_INFO("[trajectory-warm-start-config] enabled=1 authority=VALID_SAME_TOPOLOGY_WARM_START");
    ROS_INFO("[candidate-visibility-config] enabled=1 sample_dt=%.3f "
             "range=[%.2f,%.2f] occlusion_margin=%.3f fov=diagnostic-only",
             visibility_sample_dt_,
             visibility_min_target_distance_, visibility_max_target_distance_,
             visibility_occlusion_margin_);
    ROS_INFO("[team-visibility-config] enabled=1 N=3 K=2 "
             "preferred_separation_deg=%.3f "
             "encirclement_spread_saturation_deg=%.3f "
             "mode=K2_THEN_CAMERA_TIME",
             team_visibility_preferred_separation_deg_,
             encirclement_spread_saturation_angle_deg_);

    bool use_time_aware_moving_obj_cost = false;
    nh.param("prediction/use_time_aware_moving_obj_cost", use_time_aware_moving_obj_cost, false);
    if (use_time_aware_moving_obj_cost)
    {
      obj_predictor_.reset(new fast_planner::ObjPredictor(nh));
      obj_predictor_->init();
      ROS_INFO("[moving_obj] time-aware moving obstacle cost enabled.");
    }

    ploy_traj_opt_.reset(new PolyTrajOptimizer);
    ploy_traj_opt_->setParam(nh);
    if (use_time_aware_moving_obj_cost)
    {
      ploy_traj_opt_->setEnvironment(grid_map_, obj_predictor_);
    }
    else
      ploy_traj_opt_->setEnvironment(grid_map_);

    if (enable_risk_triggered_candidates_ && !obj_predictor_)
      ROS_WARN("[risk-shadow] requested but moving-object predictor/cost is disabled; evaluator will be skipped.");

    ROS_INFO("[risk-candidate-config] drone=%d enabled=%d predictor=%d time_aware=%d",
             pp_.drone_id, static_cast<int>(enable_risk_triggered_candidates_),
             static_cast<int>(static_cast<bool>(obj_predictor_)),
             static_cast<int>(use_time_aware_moving_obj_cost));

    visualization_ = vis;

    ploy_traj_opt_->setSwarmTrajs(&traj_.swarm_traj);
    ploy_traj_opt_->setDroneId(pp_.drone_id);
    ROS_INFO("[topology-coordination-planner-config] drone=%d enabled=%d "
             "timeout=%.3f result_max_age=%.3f min_active_remaining=%.3f "
             "candidate_bundle_topic=/topology_coordination/uav%d/candidate_bundle "
             "result_topic=/topology_coordination/result blocking=0 "
             "feature3_after_joint_selection=1",
             pp_.drone_id, static_cast<int>(enable_joint_topology_coordination_),
             topology_coordination_timeout_, topology_result_max_age_,
             topology_min_active_remaining_, pp_.drone_id);
    ROS_INFO("[team-vis-opt-planner-config] drone=%d enabled=%d "
             "max_age=%.3f min_activation_lead=%.3f min_execution_time=%.3f "
             "proposal_topic=/topology_coordination/team_solution "
             "ack_topic=/topology_coordination/team_ack default_off=1",
             pp_.drone_id,
             static_cast<int>(enable_team_visibility_optimizer_),
             team_solution_max_age_, team_solution_min_activation_lead_,
             team_min_execution_time_);
  }

  bool EGOPlannerManager::buildFreshMovingInitializer(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
      const Eigen::Vector3d &local_target_vel, poly_traj::MinJerkOpt &seed,
      std::string &source, double &required_speed, double &required_acc,
      double &required_jerk, bool mission_end, int side_bias,
      double side_offset, double duration_scale,
      const Eigen::Vector3d *side_direction_override)
  {
    source = "FRESH_MOVING_DIRECT";
    required_speed = required_acc = required_jerk = 0.0;
    if (!start_pt.allFinite() || !start_vel.allFinite() || !start_acc.allFinite() ||
        !local_target_pt.allFinite() || !local_target_vel.allFinite())
    {
      source = "FRESH_INVALID_STATE";
      ++initializer_rejected_preopt_count_;
      return false;
    }

    const double distance = (local_target_pt - start_pt).norm();
    const double piece_length = std::max(0.20, pp_.polyTraj_piece_length > 0.0
                                                   ? pp_.polyTraj_piece_length : 1.0);
    const int pieces = std::max(2, std::min(24,
        static_cast<int>(std::ceil(std::max(distance, piece_length) / piece_length))));

    Eigen::Matrix3d head, tail;
    head << start_pt, start_vel, start_acc;
    Eigen::Vector3d terminal_vel = mission_end ? Eigen::Vector3d::Zero() : local_target_vel;
    if (pp_.max_vel_ > 0.0 && terminal_vel.norm() > pp_.max_vel_)
      terminal_vel *= pp_.max_vel_ / terminal_vel.norm();
    tail << local_target_pt, terminal_vel, Eigen::Vector3d::Zero();

    Eigen::MatrixXd points(3, pieces - 1);
    Eigen::Vector3d side_dir = side_direction_override != nullptr
                                    ? *side_direction_override
                                    : (local_target_pt - start_pt).cross(Eigen::Vector3d::UnitZ());
    side_dir(2) = 0.0;
    if (side_dir.norm() > 1.0e-6)
      side_dir.normalize();
    for (int i = 1; i < pieces; ++i)
    {
      const double u = static_cast<double>(i) / static_cast<double>(pieces);
      Eigen::Vector3d q = start_pt + u * (local_target_pt - start_pt);
      if (side_bias != 0 && side_dir.norm() > 0.0)
        q += static_cast<double>(side_bias) * std::max(0.0, side_offset) *
             std::sin(M_PI * u) * side_dir;
      points.col(i - 1) = q;
    }

    // 本轮修复 3：时间分配必须由几何长度和系统已有的参考进度速度决定。
    // 之前这里对短距离额外施加 2*sqrt(max(distance,1.0)/a_limit) 的下界，但
    // 实际生效的是 1.20*max(1.0, distance/v_ref)：距离下限 1.0 m 与真实长度
    // 脱钩，于是 0.70 m 的局部目标被摊到 1.37 s（约 0.51 m/s），initializer
    // 天然生成"慢速前段"，随后被 rolling 重规划的 P/V/A 继承链锁死。这里改为
    // 复用系统既有速度标尺：优先 nominal tracking speed（local_target_vel），
    // 否则回落到 max_vel 的既有保守比例；加速度只保留"动力学不可行时再拉伸"
    // 的下界，不再主导时间分配。
    const double v_limit = pp_.max_vel_ > 0.0 ? pp_.max_vel_ : 1.0;
    const double a_limit = pp_.max_acc_ > 0.0 ? pp_.max_acc_ : 1.0;
    const double nominal_tracking_speed = local_target_vel.norm();
    double v_ref = std::isfinite(nominal_tracking_speed) &&
                           nominal_tracking_speed > 1.0e-3
                       ? nominal_tracking_speed
                       : 0.55 * v_limit;
    v_ref = std::max(0.25 * v_limit, v_ref);
    if (pp_.max_vel_ > 0.0)
      v_ref = std::min(v_ref, pp_.max_vel_);
    const double requested_scale = std::isfinite(duration_scale) &&
                                           duration_scale > 0.0
                                       ? duration_scale
                                       : 1.0;
    // T_geom = L / v_ref，只保留一个有界的余量让种子留在速度上限以内，使后续
    // MINCO 的位移不会立刻越界；它不能把"局部目标很近"变成"轨迹很慢"。
    constexpr double kFreshInitializerTimeMargin = 1.05;
    double total_time =
        std::max(0.10, kFreshInitializerTimeMargin * distance / v_ref) *
        requested_scale;
    if (pp_.max_acc_ > 0.0)
      total_time = std::max(total_time, 2.0 * std::sqrt(std::max(distance, 1.0e-3) / a_limit));
    // A fresh spatial target owns its newly allocated continuation time; do
    // not silently clamp it back to a stale short horizon.
    total_time = std::min(120.0, total_time);
    ROS_INFO_THROTTLE(0.5,
        "[fresh-time-allocation] drone=%d distance=%.6f nominal_tracking_speed=%.6f "
        "v_ref=%.6f requested_scale=%.6f total_time=%.6f length_over_duration=%.6f",
        pp_.drone_id, distance, nominal_tracking_speed, v_ref, requested_scale,
        total_time, distance / std::max(1.0e-6, total_time));
    Eigen::VectorXd durations = Eigen::VectorXd::Constant(pieces, total_time / pieces);

    double max_v = 0.0, max_a = 0.0, max_j = 0.0;
    int timing_iterations = 0;
    const bool timing_ok = initializePvaTiming(
        head, tail, points, durations, pp_.max_vel_, pp_.max_acc_,
        pp_.max_jer_,
        [&](const poly_traj::Trajectory &trajectory, double &v, double &a,
            double &j) { return checkTrajectoryDynamics(trajectory, v, a, j); },
        seed, max_v, max_a, max_j, timing_iterations);
    if (!timing_ok) {
      source = "NUMERICAL_INITIALIZATION_FAILED";
      ++initializer_rejected_preopt_count_;
      ROS_WARN("[PVA_TIMING_INIT] kind=FRESH status=%s iterations=%d",
               source.c_str(), timing_iterations);
      return false;
    }
    ++fresh_initializer_count_;
    if (timing_iterations > 1) ++initializer_retimed_count_;
    required_speed = distance / seed.getTraj().getTotalDuration();
    required_acc = max_a;
    required_jerk = max_j;
    ROS_INFO("[PVA_TIMING_INIT] kind=FRESH status=SUCCESS iterations=%d "
             "pieces=%d duration=%.6f max_v=%.6f max_a=%.6f max_j=%.6f",
             timing_iterations, pieces, seed.getTraj().getTotalDuration(),
             max_v, max_a, max_j);
    return true;
  }

  bool EGOPlannerManager::canReuseRemainingSuffix(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
      const Eigen::Vector3d &local_target_vel, const poly_traj::Trajectory &remaining,
      std::string &reason) const
  {
    reason.clear();
    const double duration = remaining.getTotalDuration();
    // A suffix shorter than the planner's minimum moving horizon is not a
    // reusable execution-time contract.  It may still be useful as geometry
    // for a fresh continuation, but it must not carry its shrunken timing.
    if (remaining.getPieceNum() <= 0 || !std::isfinite(duration) || duration <= 0.50)
    {
      reason = "HORIZON_TOO_SHORT";
      return false;
    }
    const Eigen::Vector3d end_p = remaining.getPos(duration);
    const Eigen::Vector3d end_v = remaining.getVel(duration);
    const double endpoint_error = (local_target_pt - end_p).norm();
    const double velocity_error = (local_target_vel - end_v).norm();
    const double head_p_error = (start_pt - remaining.getJuncPos(0)).norm();
    const double head_v_error = (start_vel - remaining.getJuncVel(0)).norm();
    const double head_a_error = (start_acc - remaining.getJuncAcc(0)).norm();
    const double required_speed = (local_target_pt - start_pt).norm() /
                                  std::max(1.0e-6, duration);
    if (endpoint_error > 0.60)
    {
      reason = "ENDPOINT_CHANGED";
      return false;
    }
    if (velocity_error > std::max(0.75, 0.5 * (pp_.max_vel_ > 0.0 ? pp_.max_vel_ : 1.0)))
    {
      reason = "REFERENCE_CHANGED";
      return false;
    }
    if (head_p_error > std::max(0.05, handoff_position_tolerance_ * 3.0) ||
        head_v_error > std::max(0.15, handoff_velocity_tolerance_ * 3.0) ||
        head_a_error > std::max(0.30, handoff_acceleration_tolerance_ * 3.0))
    {
      reason = "HEAD_DISCONTINUITY";
      return false;
    }
    if (pp_.max_vel_ > 0.0 && required_speed > pp_.max_vel_ * 1.15)
    {
      reason = "REQUIRED_SPEED_TOO_HIGH";
      return false;
    }
    double max_v = 0.0, max_a = 0.0, max_j = 0.0;
    if (!checkTrajectoryDynamics(remaining, max_v, max_a, max_j))
    {
      reason = "SUFFIX_DYNAMICS_INVALID";
      return false;
    }
    return true;
  }

  bool EGOPlannerManager::computeInitState(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
      const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel,
      const bool flag_polyInit, const bool flag_randomPolyTraj, const double &ts,
      poly_traj::MinJerkOpt &initMJO, const double time_scale, const int side_bias,
      const double side_offset,
      const Eigen::Vector3d *side_direction_override)
  {
    (void)flag_polyInit;
    (void)flag_randomPolyTraj;
    (void)ts;
    std::string source;
    double required_speed = 0.0, required_acc = 0.0, required_jerk = 0.0;
    return buildFreshMovingInitializer(start_pt, start_vel, start_acc,
                                       local_target_pt, local_target_vel,
                                       initMJO, source, required_speed,
                                       required_acc, required_jerk, false,
                                       side_bias, side_offset,
                                       (std::isfinite(time_scale) && time_scale > 0.0)
                                           ? time_scale : 1.0,
                                       side_direction_override);
  }

  void EGOPlannerManager::getLocalTarget(
      const double planning_horizen, const Eigen::Vector3d &start_pt,
      const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
      Eigen::Vector3d &local_target_vel, bool &touch_goal)
  {
    double t;
    touch_goal = false;

    traj_.global_traj.last_glb_t_of_lc_tgt = traj_.global_traj.glb_t_of_lc_tgt;

    double t_step = planning_horizen / 20 / pp_.max_vel_;
    // double dist_min = 9999, dist_min_t = 0.0;
    for (t = traj_.global_traj.glb_t_of_lc_tgt;
         t < (traj_.global_traj.global_start_time + traj_.global_traj.duration);
         t += t_step)
    {
      Eigen::Vector3d pos_t = traj_.global_traj.traj.getPos(t - traj_.global_traj.global_start_time);
      double dist = (pos_t - start_pt).norm();

      if (dist >= planning_horizen)
      {
        local_target_pos = pos_t;
        traj_.global_traj.glb_t_of_lc_tgt = t;
        break;
      }
    }

    if ((t - traj_.global_traj.global_start_time) >= traj_.global_traj.duration - 1e-5) // Last global point
    {
      local_target_pos = global_end_pt;
      traj_.global_traj.glb_t_of_lc_tgt = traj_.global_traj.global_start_time + traj_.global_traj.duration;
      touch_goal = true;
    }

    if ((global_end_pt - local_target_pos).norm() < (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_))
    {
      // The rolling observation waypoint is not mission completion. Preserve
      // its prescribed continuation; a true final mission goal still has v=0.
      local_target_vel = traj_.global_traj.traj.getVel(traj_.global_traj.duration);
    }
    else
    {
      local_target_vel = traj_.global_traj.traj.getVel(t - traj_.global_traj.global_start_time);
    }

    if (grid_map_ && grid_map_->getInflateOccupancy(local_target_pos) != 0)
    {
      const Eigen::Vector3d occupied_target = local_target_pos;
      const double occupied_target_time = traj_.global_traj.glb_t_of_lc_tgt;
      const double global_start_time = traj_.global_traj.global_start_time;
      bool found_free_target = false;
      for (double target_time = occupied_target_time - t_step;
           target_time >= global_start_time;
           target_time -= t_step)
      {
        const double trajectory_time = target_time - global_start_time;
        const Eigen::Vector3d candidate_target =
            traj_.global_traj.traj.getPos(trajectory_time);
        if (grid_map_->getInflateOccupancy(candidate_target) == 0)
        {
          local_target_pos = candidate_target;
          local_target_vel = traj_.global_traj.traj.getVel(trajectory_time);
          touch_goal = false;
          traj_.global_traj.glb_t_of_lc_tgt = target_time;
          traj_.global_traj.last_glb_t_of_lc_tgt = target_time;
          found_free_target = true;
          ROS_WARN_THROTTLE(
              0.5,
              "[local-target-adjust] drone=%d occupied=(%.3f %.3f %.3f) "
              "free=(%.3f %.3f %.3f) backtrack_time=%.3f",
              pp_.drone_id, occupied_target.x(), occupied_target.y(), occupied_target.z(),
              local_target_pos.x(), local_target_pos.y(), local_target_pos.z(),
              occupied_target_time - target_time);
          break;
        }
      }

      if (!found_free_target)
      {
        ROS_WARN_THROTTLE(
            0.5,
            "[local-target-adjust] drone=%d no free point behind occupied target "
            "(%.3f %.3f %.3f)",
            pp_.drone_id, occupied_target.x(), occupied_target.y(), occupied_target.z());
      }
    }
  }

  bool EGOPlannerManager::validateActivePrefixUntil(double until, std::string &reason) const
  {
    const double now=ros::Time::now().toSec();
    const auto &active=traj_.local_traj;
    if(active.traj_id<=0 || active.traj.getPieceNum()<=0) return true;
    if(now<active.start_time || until<=now) { reason="INVALID_PREFIX_TIME"; return false; }
    poly_traj::Trajectory prefix;
    double piece_start=active.start_time;
    for(int i=0;i<active.traj.getPieceNum();++i)
    {
      const double piece_end=piece_start+active.traj[i].getDuration();
      const double begin=std::max(now,piece_start), end=std::min(until,piece_end);
      if(end>begin) prefix.emplace_back(shiftPiece(active.traj[i],begin-piece_start,end-begin));
      piece_start=piece_end;
    }
    // Finite execution only: terminal hold cannot certify a future handoff.
    if(until>piece_start) { reason="MOVING_SUFFIX_EXHAUSTED"; return false; }
    return prefix.getPieceNum()>0 && validateExecutionTrajectory(prefix,now,false,reason);
  }

  bool EGOPlannerManager::validateCommittedPrefixUntil(
      const double until, std::string &reason) const
  {
    const double now = ros::Time::now().toSec();
    if (!std::isfinite(until) || until <= now)
    {
      reason = "INVALID_PREFIX_TIME";
      return false;
    }

    const auto validate_interval =
        [this, &reason](const LocalTrajData &source,
                        const double begin, const double end) -> bool
    {
      if (end <= begin + 1.0e-9)
        return true;
      if (source.traj_id <= 0 || source.traj.getPieceNum() <= 0 ||
          begin < source.start_time - 1.0e-6 ||
          end > source.end_time + 1.0e-6)
      {
        reason = "MOVING_SUFFIX_EXHAUSTED";
        return false;
      }

      poly_traj::Trajectory segment;
      double piece_start = source.start_time;
      for (int i = 0; i < source.traj.getPieceNum(); ++i)
      {
        const double piece_end = piece_start + source.traj[i].getDuration();
        const double clipped_begin = std::max(begin, piece_start);
        const double clipped_end = std::min(end, piece_end);
        if (clipped_end > clipped_begin + 1.0e-9)
          segment.emplace_back(shiftPiece(source.traj[i],
              clipped_begin - piece_start, clipped_end - clipped_begin));
        piece_start = piece_end;
      }
      if (segment.getPieceNum() <= 0)
      {
        reason = "INVALID_PREFIX_TIME";
        return false;
      }
      return validateExecutionTrajectory(segment, begin, false, reason);
    };

    const LocalTrajData &successor = traj_.local_traj;
    if (successor.traj_id <= 0 || successor.traj.getPieceNum() <= 0)
      return true;

    // The rolling container may already own a committed successor whose
    // start is still in the future.  In that interval the actual execution
    // schedule is predecessor(now..successor.start) followed by the committed
    // successor.  Validate that composed schedule instead of misclassifying
    // the future successor as an invalid active prefix.
    if (now < successor.start_time - 1.0e-6)
    {
      const LocalTrajData &predecessor = scheduled_predecessor_;
      const double predecessor_end = std::min(until, successor.start_time);
      if (!validate_interval(predecessor, now, predecessor_end))
        return false;
      if (until <= successor.start_time + 1.0e-6)
        return true;

      const double predecessor_elapsed = successor.start_time - predecessor.start_time;
      if (predecessor_elapsed < -1.0e-6 ||
          predecessor_elapsed > predecessor.duration + 1.0e-6 ||
          (predecessor.traj.getPos(predecessor_elapsed) - successor.traj.getPos(0.0)).norm() > 1.0e-5 ||
          (predecessor.traj.getVel(predecessor_elapsed) - successor.traj.getVel(0.0)).norm() > 1.0e-5 ||
          (predecessor.traj.getAcc(predecessor_elapsed) - successor.traj.getAcc(0.0)).norm() > 1.0e-5)
      {
        reason = "COMMITTED_SUCCESSOR_BOUNDARY_MISMATCH";
        return false;
      }
      return validate_interval(successor, successor.start_time, until);
    }

    return validate_interval(successor, now, until);
  }

  void EGOPlannerManager::invalidatePendingTopology(const char *reason)
  {
    const bool preserve_precommit_team_proposal =
        pending_team_trajectory_.accepted &&
        !pending_team_trajectory_.commit_received;
    if (preserve_precommit_team_proposal)
      publishTeamTrajectoryAck(pending_team_trajectory_.solution,
          pending_team_trajectory_.planning_generation, pending_team_trajectory_.source_candidate_id,
          false, std::string("LOCAL_GENERATION_INVALIDATED:")+reason);
    // A committed team tail is an authoritative three-UAV schedule.  Ordinary
    // local rolling updates must not erase it or turn into a one-sided veto.
    // Only the explicit adoption path below clears the pending identity.
    if (pending_team_trajectory_.accepted && pending_team_trajectory_.commit_received &&
        std::string(reason) != "TEAM_COMMIT_ADOPTED")
    {
      ROS_INFO("[team-commit-atomic] event=COMMITTED_TEAM_TAIL_PRESERVED "
               "drone=%d team_solution_id=%lu reason=%s",
          pp_.drone_id,
          static_cast<unsigned long>(pending_team_trajectory_.team_solution_id),
          reason);
      return;
    }
    if (preserve_precommit_team_proposal)
    {
      // The coordinator may already have settled the authoritative COMMIT
      // while this NACK is in flight.  Keep the proposal identity alive so a
      // late COMMIT can still be handled (and rejected by final PVA/safety
      // checks if the local update made it stale); an explicit team ABORT
      // clears it.  This does not block local planning or local commits.
      ROS_INFO("[team-commit-atomic] event=PRECOMMIT_PROPOSAL_PRESERVED "
               "drone=%d team_solution_id=%lu reason=%s",
          pp_.drone_id,
          static_cast<unsigned long>(pending_team_trajectory_.team_solution_id),
          reason);
      return;
    }
    if (pending_topology_.active)
    {
      last_pending_duration_ = std::max(0.0, ros::Time::now().toSec()-pending_topology_.bundle_stamp);
      ROS_INFO("[team-pending] event=TEAM_PENDING_END drone=%d generation=%lu duration=%.6f reason=%s",
          pp_.drone_id, pending_topology_.planning_generation, last_pending_duration_, reason);
    }
    pending_topology_ = PendingTopologyState();
    pending_team_trajectory_ = PendingTeamTrajectory();
    latest_topology_result_valid_ = false;
  }

  LocalGeometryMetrics EGOPlannerManager::evaluateLocalGeometry(
      const poly_traj::Trajectory &candidate,double candidate_start,
      double begin,double horizon) const
  {
    LocalGeometryMetrics metrics;
    if(!local_geometry_active_ || candidate.getPieceNum()<=0 || pp_.drone_id<0 || pp_.drone_id>=3 ||
       geometry_target_stamp_<=0 || horizon<=0 || begin<geometry_target_stamp_-0.1 ||
       begin-geometry_target_stamp_>3.0) return metrics;
    int valid=0;
    const int samples=std::max(1,int(std::ceil(horizon/geometry_policy_.dt)));
    for(int k=0;k<samples;++k)
    {
      const double world=begin+horizon*(k+0.5)/samples;
      const Eigen::Vector3d target=geometry_target_p_+geometry_target_v_*(world-geometry_target_stamp_);
      std::array<Eigen::Vector3d,3> relative;
      for(int id=0;id<3;++id)
      {
        if(id==pp_.drone_id) relative[id]=trajectory_lifecycle::sample(candidate,world-candidate_start).p-target;
        else if(id<int(traj_.swarm_traj.size()) && traj_.swarm_traj[id].drone_id==id &&
                traj_.swarm_traj[id].traj.getPieceNum()>0 && world>=traj_.swarm_traj[id].executionAt(world).start_time)
        {
          const auto &peer=traj_.swarm_traj[id].executionAt(world);
          const double expired=std::max(0.0,begin-peer.start_time-peer.duration);
          metrics.confidence=std::min(metrics.confidence,
              multi_uav_formation::peerGeometryConfidence(
                  expired,geometry_policy_.peer_fresh_age));
          relative[id]=trajectory_lifecycle::sample(peer.traj,world-peer.start_time).p-target;
        }
        else if(visibility_odom_[id].valid) {
          const double age=std::max(0.0,begin-visibility_odom_[id].stamp);
          metrics.confidence=std::min(metrics.confidence,
              multi_uav_formation::peerGeometryConfidence(
                  age,geometry_policy_.peer_fresh_age));
          // Stale odometry is conservatively held, never waited for.
          relative[id]=visibility_odom_[id].position+visibility_odom_[id].velocity*
              std::min(geometry_policy_.peer_fresh_age,
                       std::max(0.0,world-visibility_odom_[id].stamp))-target;
        }
        else return LocalGeometryMetrics();
      }
      const auto g=multi_uav_formation::circularGapGeometry(relative);
      if(!g.finite) return LocalGeometryMetrics();
      valid+=g.eligible(geometry_policy_.minimum,geometry_policy_.maximum);
      metrics.violation+=multi_uav_formation::encirclementGeometryCost(relative,
          geometry_policy_.minimum,geometry_policy_.maximum)/samples;
      metrics.maximum=std::max(metrics.maximum,g.maximum);
      metrics.minimum=std::min(metrics.minimum,g.minimum);
    }
    metrics.ratio=double(valid)/samples;
    metrics.valid=true;
    return metrics;
  }

  void EGOPlannerManager::logExecutionGeometry(const char *event,const LocalGeometryMetrics *baseline)
  {
    if(!local_geometry_active_) return;
    const double now=ros::Time::now().toSec();
    const auto &active=traj_.local_traj;
    const auto m=evaluateLocalGeometry(active.traj,active.start_time,std::max(now,active.start_time),geometry_policy_.horizon);
    const auto geometry_state=baseline ? classifyGeometry(*baseline,m,geometry_policy_) :
        (m.valid && m.ratio>=0.8 ? LocalGeometryState::NORMAL_ENCIRCLEMENT :
         LocalGeometryState::GEOMETRY_DEGRADED);
    ROS_INFO("[local-geometry-execution] event=%s drone=%d timestamp=%.9f activation=%.9f trajectory_id=%d generation=%lu source=%s state=%s valid=%d encirclement_ratio=%.6f gap_violation_integral=%.9f gap_max_deg=%.6f gap_min_deg=%.6f remaining=%.6f urgent_recovery=0 geometry_execution_gate=0 ENCIRCLEMENT_DEGRADED=%d",
        event,pp_.drone_id,now,active.start_time,active.traj_id,active_traj_generation_,active_execution_source_.c_str(),
        geometryStateName(geometry_state),int(m.valid),m.ratio,m.violation,m.maximum*180/M_PI,m.minimum*180/M_PI,
        active.start_time+active.duration-now,
        int(geometry_state!=LocalGeometryState::NORMAL_ENCIRCLEMENT));
  }

  void EGOPlannerManager::ensureExecutionCoverage()
  {
    const double now=ros::Time::now().toSec(),wall=ros::WallTime::now().toSec();
    const auto &active=traj_.local_traj;
    double coverage=std::numeric_limits<double>::infinity();
    execution_reserve_limited_=false;
    validated_handoff_this_batch_=false;
    executable_candidate_seen_this_batch_=false;
    planning_predecessor_id_=active.traj_id;
    validated_coverage_end_=std::numeric_limits<double>::infinity();
    planning_deadline_ros_=std::numeric_limits<double>::infinity();
    validated_coverage_remaining_=std::numeric_limits<double>::infinity();
    planning_budget_remaining_=std::numeric_limits<double>::infinity();
    if(active.traj_id>0 && active.traj.getPieceNum()>0)
    {
      lifecycleSuccessorDue(); // fresh safety snapshot, independent of geometry
      validated_coverage_end_=std::min(active.start_time+active.duration,
          std::max(now,execution_safe_until_));
      const ValidatedMovingCoverageWindow window{
          now,validated_coverage_end_,local_activation_margin_};
      coverage=std::max(0.0,window.validatedCoverageRemaining());
      planning_deadline_ros_=window.planningDeadline();
      validated_coverage_remaining_=coverage;
      planning_budget_remaining_=window.planningBudgetRemaining();
      ROS_INFO("[validated-moving-coverage] event=VALIDATED_MOVING_COVERAGE_REMAINING drone=%d trajectory_id=%d now=%.9f t_validated_end=%.9f t_activation_earliest=%.9f t_planning_deadline=%.9f validated_coverage_remaining=%.6f planning_budget_remaining=%.6f",
          pp_.drone_id,active.traj_id,now,validated_coverage_end_,
          window.activationEarliest(),planning_deadline_ros_,coverage,
          planning_budget_remaining_);
      if(!window.canStillHandoff())
      {
        const bool entering=!current_state_restart_active_ ||
            expired_predecessor_id_!=active.traj_id;
        current_state_restart_active_=true;
        expired_predecessor_id_=active.traj_id;
        planning_predecessor_id_=-1;
        execution_reserve_limited_=true;
        if(entering)
        {
          current_state_valid_=false;
          ROS_ERROR("[current-state-restart] event=PREDECESSOR_HANDOFF_DEADLINE_EXPIRED drone=%d expired_predecessor_id=%d now=%.9f t_validated_end=%.9f t_activation_earliest=%.9f planning_budget_remaining=%.6f coverage_failure_recorded=1 old_suffix_extended=0",
              pp_.drone_id,expired_predecessor_id_,now,
              validated_coverage_end_,window.activationEarliest(),planning_budget_remaining_);
          ROS_WARN("[current-state-restart] event=ENTER drone=%d expired_predecessor_id=%d authority=ODOMETRY_SAME_LOCAL_PLANNER",
              pp_.drone_id,expired_predecessor_id_);
        }
        // Feedback093: restart mode owns no reliable future coverage, so its
        // priority is one complete safe successor, not team refinement.  The
        // budget is derived from the measured pipeline estimate (never a
        // hard-coded number of seconds) so a restart can always afford a whole
        // normal production attempt plus one retry.
        const double pipeline_estimate=execution_budget_.estimate(wall);
        double restart_budget=movingPlanningBudget(window,pipeline_estimate);
        const double restart_floor=std::max(2.0*pipeline_estimate,
                                            uninterruptible_budget_);
        if(std::isfinite(restart_floor) && restart_floor>restart_budget)
        {
          ROS_WARN("[current-state-restart] event=MIN_BUDGET_APPLIED drone=%d "
                   "expired_predecessor_id=%d coverage_budget=%.6f "
                   "pipeline_estimate=%.6f uninterruptible=%.6f applied=%.6f",
              pp_.drone_id,expired_predecessor_id_,restart_budget,
              pipeline_estimate,uninterruptible_budget_,restart_floor);
          restart_budget=restart_floor;
        }
        planning_deadline_ros_=std::numeric_limits<double>::infinity();
        planning_budget_remaining_=restart_budget;
        planning_deadline_wall_=wall+restart_budget;
        ploy_traj_opt_->setExecutionDeadline(planning_deadline_wall_,uninterruptible_budget_);
        ROS_INFO("[current-state-restart] event=ATTEMPT drone=%d expired_predecessor_id=%d planning_wall_budget=%.6f budget_source=EXECUTION_PIPELINE_ESTIMATE old_deadline_authority=0",
            pp_.drone_id,expired_predecessor_id_,restart_budget);
        return;
      }
      if(current_state_restart_active_)
      {
        // This can only occur if a newly installed trajectory restored finite
        // handoff coverage between callbacks.  Normal commit also clears the
        // state explicitly below.
        current_state_restart_active_=false;
        current_state_valid_=false;
      }
      if(!execution_budget_.canStart(coverage,wall,local_activation_margin_,execution_margin_))
      {
        // Short coverage cuts optional work, never publishes a stop or waits
        // for reserve recovery. The current verified suffix remains executing.
        execution_reserve_limited_=true;
        ROS_WARN("[validated-moving-coverage] event=VALIDATED_COVERAGE_LOW drone=%d trajectory_id=%d t_validated_end=%.9f t_planning_deadline=%.9f validated_coverage_remaining=%.6f planning_budget_remaining=%.6f estimated_pipeline=%.6f action=FIRST_EXECUTABLE_LOCAL_SUCCESSOR",
            pp_.drone_id,active.traj_id,validated_coverage_end_,planning_deadline_ros_,
            coverage,planning_budget_remaining_,execution_budget_.estimate(wall));
        if(!window.canStillHandoff()) {
          ROS_ERROR_THROTTLE(0.2,"[validated-moving-coverage] event=VALIDATED_COVERAGE_DEADLINE_MISSED drone=%d trajectory_id=%d now=%.9f t_validated_end=%.9f t_activation_earliest=%.9f planning_budget_remaining=%.6f",
              pp_.drone_id,active.traj_id,now,validated_coverage_end_,
              window.activationEarliest(),planning_budget_remaining_);
          ROS_ERROR_THROTTLE(0.2,"[continuous-motion] event=MOVING_SUCCESSOR_STARVATION drone=%d trajectory_id=%d remaining=%.6f safe_coverage=%.6f reason=NO_ACTIVATION_BEFORE_VALIDATED_END action=URGENT_MOVING_REPLAN run_fail=1",
              pp_.drone_id,active.traj_id,active.start_time+active.duration-now,coverage);
        }
        // Never extend a wall deadline beyond the real predecessor coverage.
        // A candidate that cannot complete before this bound is not executable.
        const double fast_budget=movingPlanningBudget(
            window,execution_budget_.estimate(wall));
        planning_deadline_wall_=wall+fast_budget;
        ploy_traj_opt_->setExecutionDeadline(planning_deadline_wall_,uninterruptible_budget_);
        ROS_WARN("[execution-reserve] drone=%d action=BOUNDED_MOVING_REPLAN coverage=%.6f planning_budget=%.6f expensive_refinement=0",
            pp_.drone_id,coverage,planning_deadline_wall_-wall);
        return;
      }
    }
    const double permitted=std::max(0.0,std::min(
        execution_budget_.estimate(wall),planning_budget_remaining_));
    planning_deadline_wall_=wall+permitted;
    ploy_traj_opt_->setExecutionDeadline(planning_deadline_wall_,uninterruptible_budget_);
    ROS_INFO("[execution-reserve] drone=%d action=SOLVE_ALLOWED coverage=%.6f planning_budget=%.6f deadline_wall=%.9f planning_deadline_ros=%.9f activation_margin=%.6f execution_margin=%.6f",
        pp_.drone_id,coverage,permitted,planning_deadline_wall_,planning_deadline_ros_,local_activation_margin_,execution_margin_);
    return;
  }

  bool EGOPlannerManager::optionalRefinementAllowed() const
  {
    const bool allowed=refinementFits(planning_deadline_wall_,ros::WallTime::now().toSec(),uninterruptible_budget_);
    if(!allowed) ROS_WARN("[execution-reserve] drone=%d action=OPTIONAL_REFINEMENT_STOP deadline_wall=%.9f t_planning_deadline=%.9f validated_local_successor=%d",
        pp_.drone_id,planning_deadline_wall_,planning_deadline_ros_,
        int(validated_handoff_this_batch_));
    return allowed;
  }

  bool EGOPlannerManager::mandatoryPlanningAttemptAllowed() const
  {
    return movingPlanningAttemptAllowed(planning_deadline_wall_,
        ros::WallTime::now().toSec(),uninterruptible_budget_,
        executable_candidate_seen_this_batch_);
  }

  void EGOPlannerManager::finishPlanningBatch(double wall_started)
  {
    const double wall=ros::WallTime::now().toSec(),now=ros::Time::now().toSec();
    const double elapsed=std::max(0.0,wall-wall_started);
    if(current_state_restart_active_ && !validated_handoff_this_batch_)
      ROS_WARN_THROTTLE(0.5,"[current-state-restart] event=NO_SAFE_CANDIDATE drone=%d expired_predecessor_id=%d planning_wall_budget=%.6f hard_checks_bypassed=0 next_cycle_opportunity=1",
          pp_.drone_id,expired_predecessor_id_,planning_deadline_wall_-wall_started);
    if(planning_predecessor_id_>0 && !validated_handoff_this_batch_ &&
       std::isfinite(planning_deadline_ros_) && now>=planning_deadline_ros_)
    {
      ++no_executable_successor_count_;
      ROS_ERROR("[validated-moving-coverage] event=NO_EXECUTABLE_SUCCESSOR_BEFORE_DEADLINE drone=%d trajectory_id=%d now=%.9f t_validated_end=%.9f t_planning_deadline=%.9f candidate_seen=%d",
          pp_.drone_id,planning_predecessor_id_,now,validated_coverage_end_,
          planning_deadline_ros_,int(executable_candidate_seen_this_batch_));
      ROS_ERROR("[continuous-motion] event=MOVING_SUCCESSOR_STARVATION drone=%d trajectory_id=%d remaining=%.6f safe_coverage=%.6f reason=NO_EXECUTABLE_SUCCESSOR_BEFORE_DEADLINE action=URGENT_MOVING_REPLAN run_fail=1",
          pp_.drone_id,planning_predecessor_id_,
          std::max(0.0,validated_coverage_end_-now),
          std::max(0.0,validated_coverage_end_-now));
    }
    execution_budget_.observe(elapsed,wall);
    // Expected head epoch and worst-case execution coverage have separate jobs.
    // Finalization still reanchors and checks actual-active P/V/A.
    local_planning_budget_=std::max(0.04,std::min(0.12,0.9*local_planning_budget_+0.1*elapsed));
    ROS_INFO("[execution-planning-budget] drone=%d timestamp=%.9f planning_latency_ms=%.3f planning_budget_ms=%.3f measured_failures_included=1",
        pp_.drone_id,ros::Time::now().toSec(),elapsed*1000,execution_budget_.estimate(wall)*1000);
    planning_deadline_wall_=std::numeric_limits<double>::infinity();
    ploy_traj_opt_->setExecutionDeadline(planning_deadline_wall_,uninterruptible_budget_);
  }

  EGOPlannerManager::TopologyProcessStatus EGOPlannerManager::finalizeCapturedCandidates(
      std::vector<CandidateSetOutput> &sets,bool touch_goal,double head_threshold,int &selected_set)
  {
    selected_set=-1;
    // FROZEN-ACTIVATION contract: the batch's only activation was fixed in
    // prepareFutureActivation BEFORE any candidate was generated, and every
    // candidate already carries the matching head P/V/A, target epoch and
    // prediction epoch.  Finalization consumes it verbatim — it never moves
    // the boundary and therefore never rebuilds an optimized polynomial.
    const double activation = local_activation_time_;
    local_activation_time_=activation;
    // 阶段 E：本 batch 的唯一 activation 在此冻结。全部 N/L/R 候选共享同一
    // activation、同一 predecessor 边界、同一目标世界时原点之后才做 visibility
    // 比较与 bundle 导出（TimedTrajectoryCandidate 契约）。
    ROS_INFO("[candidate-batch-activation] drone=%d batch_activation=%.9f "
             "plan_start=%.9f planning_reserve=%.6f "
             "frozen_before_optimization=1 same_activation_contract=1",
        pp_.drone_id,activation,local_planning_started_,
        activation-local_planning_started_-local_activation_margin_);
    auto current=evaluateLocalGeometry(traj_.local_traj.traj,traj_.local_traj.start_time,
        activation,geometry_policy_.horizon);
    // ---- K3 Local recovery 比较上下文(第三轮 authority 清理)--------------
    // 事件由 updateLocalK3RecoveryEvent 自主检测/刷新/闭合(margin crossing
    // 锚定),不再依赖 Team escalation hint。OFF 时完全不进入,普通 comparator
    // 逐位不变。
    if (k3_local_escalation_enabled_)
      updateLocalK3RecoveryEvent();
    const bool k3_escalation_active =
        k3_local_escalation_enabled_ && k3_event_.active &&
        !k3_event_.m2_preempted && !k3_event_.recovered;
    const double k3_window_begin =
        k3_escalation_active ? k3_event_.window_begin.toSec() : 0.0;
    const double k3_window_end =
        k3_escalation_active ? k3_event_.window_end.toSec() : 0.0;
    const double k3_required_margin =
        k3_escalation_active ? k3_event_.required_margin : 0.0;
    // Baseline and all alternatives are compared over the same executable
    // portion of the event window.  The baseline starts before this batch's
    // activation, so including the historical prefix would compare different
    // world-time intervals and could seed yaw from the present into the past.
    const auto &k3_baseline_traj = traj_.local_traj;
    const double k3_comparison_begin = k3_escalation_active
        ? std::max(k3_window_begin, activation) : 0.0;
    const double k3_comparison_end = k3_escalation_active &&
            k3_baseline_traj.traj.getPieceNum() > 0
        ? std::min(k3_window_end, k3_baseline_traj.start_time +
                                      k3_baseline_traj.traj.getTotalDuration())
        : 0.0;
    k3_baseline_valid_ = false;
    // 已执行轨迹(基线)的窗口指标:整个 batch 只算一次,比较器与遥测共用。
    auto ensureK3Baseline = [&]() {
      if (!k3_escalation_active || k3_baseline_valid_ || sets.empty())
        return;
      const auto &baseline = traj_.local_traj;
      if (baseline.traj_id <= 0 || baseline.traj.getPieceNum() <= 0)
        return;
      const auto baseline_metrics = evaluateLocalK3WindowMetrics(
          baseline.traj, baseline.start_time, sets[0].visibility_target_position,
          sets[0].visibility_target_velocity, sets[0].visibility_target_epoch,
          k3_comparison_begin, k3_comparison_end, k3_required_margin);
      if (baseline_metrics.valid)
      {
        k3_baseline_valid_ = true;
        k3_baseline_window_k2_ = baseline_metrics.k2;
        k3_baseline_window_c3_ = baseline_metrics.c3;
        k3_baseline_window_d3_ = baseline_metrics.d3;
      }
    };
    if (k3_escalation_active)
    {
      // 基线窗口指标在 batch 开头完成(读"当前已执行"轨迹,不受本 batch 选择
      // 影响);cycle 计数供逐事件遥测。
      ensureK3Baseline();
      ++k3_event_.cycle_count;
      if (!k3_baseline_valid_)
        ROS_WARN_THROTTLE(2.0,
            "[K3_K2_GATE_UNAVAILABLE] event_id=%lu cycle=%d "
            "reason=BASELINE_WINDOW_INVALID comparison_window=[%.9f,%.9f]",
            static_cast<unsigned long>(k3_event_.escalation_id),
            k3_event_.cycle_count, k3_comparison_begin,
            k3_comparison_end);
    }
    // 逐 kind 聚合(每个 batch 汇总一次,供 [K3_SIDE_CANDIDATES])。
    struct K3SideAggregate
    {
      int safe_count{0};
      int k2_admissible_count{0};
      double k2{0.0};
      double c3{-1.0};
      double d3{std::numeric_limits<double>::infinity()};
      int best_candidate_id{-1};
    };
    std::array<K3SideAggregate, 3> k3_side_aggregates;
    std::string k3_selected_reason;
    CandidateResult *best=nullptr;
    CandidateResult *reserved_first_safe=nullptr;
    int reserved_first_safe_set=-1;
    bool deadline_stopped_quality=false;
    const auto visibility_better = [&](const CandidateVisibilityReport &lhs,
                                       const CandidateVisibilityReport &rhs) {
      const LocalVisibilityPreference lhs_preference{
          lhs.none_visibility,lhs.atleast2_visibility,lhs.atleast_k_visibility,
          lhs.mean_visible_count,lhs.all3_visibility,lhs.min_uav_visibility,
          lhs.max_loss_duration,lhs.diversity_score,lhs.min_pairwise_angle_deg,
          lhs.team_utility};
      const LocalVisibilityPreference rhs_preference{
          rhs.none_visibility,rhs.atleast2_visibility,rhs.atleast_k_visibility,
          rhs.mean_visible_count,rhs.all3_visibility,rhs.min_uav_visibility,
          rhs.max_loss_duration,rhs.diversity_score,rhs.min_pairwise_angle_deg,
          rhs.team_utility};
      return betterLocalVisibilityCandidate(lhs_preference,rhs_preference);
    };
    for(int i=0;i<int(sets.size());++i)
    {
      for(auto &candidate:sets[i].candidates)
      {
        if(reserved_first_safe && execution_reserve_limited_ &&
           !deadlineAwareAlternativeAllowed(optionalRefinementAllowed(),true,true))
        {
          ROS_WARN("[coverage-quality] event=QUALITY_SEARCH_ABORTED_FOR_DEADLINE drone=%d reserved_candidate=%d t_planning_deadline=%.9f",
              pp_.drone_id,reserved_first_safe->candidate_id,planning_deadline_ros_);
          deadline_stopped_quality=true;
          break;
        }
        if(reserved_first_safe && execution_reserve_limited_)
          ROS_INFO("[coverage-quality] event=QUALITY_SEARCH_AFTER_FIRST_SAFE drone=%d reserved_candidate=%d candidate=%d planning_budget_remaining=%.6f",
              pp_.drone_id,reserved_first_safe->candidate_id,candidate.candidate_id,
              std::max(0.0,planning_deadline_wall_-ros::WallTime::now().toSec()));
        // Construction status is not an execution verdict.  Candidates are
        // rechecked below against the current trajectory revision and all
        // hard gates; construction labels are diagnostics and must not prune
        // a candidate before that evaluation.
        if(!candidate.success) continue;
        auto prepared=candidate.min_jerk_opt;
        const auto t=prepared.getTraj();
        if(t.getPieceNum()<=0) continue;
        candidate_ready_time_=ros::Time::now().toSec();
        std::string handoff_reason;
        const double candidate_activation=activation;
        // Validation-only handoff: the polynomial arrives exactly as the
        // optimizer produced it — generation already anchored its head P/V/A,
        // target and prediction epochs to the frozen activation.  A pipeline
        // that missed the frozen slot is rejected whole; the safe incumbent
        // keeps flying and the next rolling tick refreezes a later boundary.
        const std::uint64_t payload_hash_before_handoff =
            trajectoryPayloadHash(prepared.getTraj());
        if(!validateLocalHandoffWindow(candidate_activation,&handoff_reason))
        {
          ROS_INFO("[moving-rehead-reject] drone=%d candidate=%d reason=%s",
              pp_.drone_id,candidate.candidate_id,handoff_reason.c_str());
          continue;
        }
        if(std::abs(candidate_activation-activation)>1.0e-9)
        {
          // 防御：frozen activation 契约被破坏（不应发生）。
          ROS_ERROR("[candidate-batch-activation] drone=%d candidate=%d "
                    "reason=FROZEN_ACTIVATION_MUTATED frozen=%.9f mutated=%.9f",
              pp_.drone_id,candidate.candidate_id,activation,candidate_activation);
          continue;
        }
        const std::uint64_t payload_hash_after_handoff =
            trajectoryPayloadHash(prepared.getTraj());
        if(payload_hash_after_handoff!=payload_hash_before_handoff)
        {
          // Structural tripwire: nothing between the two hashes may touch the
          // polynomial.  If this fires, post-optimization mutation authority
          // has returned somewhere and must be removed, not tolerated.
          ROS_ERROR("[candidate-batch-activation] drone=%d candidate=%d "
                    "reason=POST_OPT_PAYLOAD_MUTATED hash_before=%lu "
                    "hash_after=%lu",
              pp_.drone_id,candidate.candidate_id,
              payload_hash_before_handoff,payload_hash_after_handoff);
          continue;
        }
        // Do not select a nearly zero-duration stationary hypothesis just
        // because its optimization cost is small. The 0.2 m/s threshold is
        // the continuous-motion diagnostic threshold, not a safety limit.
        // A short MOVING successor remains eligible: rejecting it solely for
        // not covering optional planning would starve safe rolling execution.
        const double rolling_coverage = execution_budget_.minimum +
            local_planning_budget_ + local_activation_margin_ + execution_margin_;
        if(t.getTotalDuration() < rolling_coverage &&
           std::max({t.getMaxVelRate(),t.getVel(0).norm(),
                     t.getVel(t.getTotalDuration()).norm()}) < 0.2) {
          ROS_INFO("[moving-candidate-reject] drone=%d candidate=%d reason=SHORT_STATIONARY_HYPOTHESIS duration=%.6f required=%.6f",
              pp_.drone_id,candidate.candidate_id,t.getTotalDuration(),rolling_coverage);
          continue;
        }
        // Reanchoring changes the polynomial. It receives no second optimizer
        // authority: the one current-revision preflight below either accepts
        // it or the next ordinary rolling tick replans from the new state.
        std::string reason;
        const bool safe=validateRetimedLocalSfc(candidate,prepared.getTraj()) &&
            checkActiveHandoff(prepared.getTraj(),activation,candidateKindName(candidate.kind),true) &&
            validateExecutionTrajectory(prepared.getTraj(),activation,touch_goal,reason);
        if(!safe) {
          ROS_INFO("[local-geometry-candidate] drone=%d candidate=%d hypothesis=%d safe=0 reason=%s",
              pp_.drone_id,candidate.candidate_id,sets[i].hypothesis_id,reason.c_str());continue;
        }
        const double checked_until = hardCheckedUntil(
            prepared.getTraj(), activation, touch_goal);
        if (!std::isfinite(checked_until) ||
            checked_until <= activation + 1.0e-6) {
          ROS_WARN("[CANDIDATE_CERTIFICATE] candidate=%d status=UNKNOWN "
                   "reason=MISSING_MAP_PREDICTION_OR_PEER activation=%.9f",
                   candidate.candidate_id, activation);
          continue;
        }
        candidate.min_jerk_opt=std::move(prepared);candidate.prediction_epoch=activation;
        ++candidate.execution_revision;
        candidate.execution_prepared=true;
        candidate.validated_payload_hash = trajectoryPayloadHash(
            candidate.min_jerk_opt.getTraj());
        candidate.checked_until = checked_until;
        /* Feedback126 §7 freeze point: id/revision/polynomial/activation/
         * certificate are fixed HERE.  Downstream quality comparison, commit
         * and publish must consume exactly this payload — any later change
         * requires revision++ and invalidates safety, quality and
         * certificate together. */
        ROS_INFO("[FROZEN_CANDIDATE] candidate=%d revision=%lu "
                 "kind=%s payload_hash=%lu activation=%.9f "
                 "checked_until=%.9f active_end=%.9f "
                 "handoff_mutation=0 hash_before_handoff=%lu "
                 "hash_after_handoff=%lu",
                 candidate.candidate_id, candidate.execution_revision,
                 candidateKindName(candidate.kind),
                 candidate.validated_payload_hash, activation,
                 candidate.checked_until,
                 activation + candidate.min_jerk_opt.getTraj().getTotalDuration(),
                 payload_hash_before_handoff, payload_hash_after_handoff);
        candidate.execution_target_position=sets[i].visibility_target_position;
        candidate.execution_target_velocity=sets[i].visibility_target_velocity;
        candidate.execution_target_state_time=sets[i].visibility_target_epoch;
        candidate.execution_target_context_valid=
            candidate.execution_target_position.allFinite() &&
            candidate.execution_target_velocity.allFinite() &&
            std::isfinite(candidate.execution_target_state_time);
        candidate.geometry=evaluateLocalGeometry(candidate.min_jerk_opt.getTraj(),activation,activation,geometry_policy_.horizon);
        if(candidate.execution_target_context_valid)
        {
          candidate.visibility=evaluateCandidateVisibility(candidate.min_jerk_opt.getTraj(),
              activation,candidate.execution_target_position,
              candidate.execution_target_velocity,
              candidate.execution_target_state_time,
              candidate.encirclement_generation>0);
          if(k3_escalation_active)
          {
            ensureK3Baseline();
            // K3 窗口指标统一在 LocalVisibilitySample 原语上计算(含 FOV,
            // 与 executed truth 同口径;C3/K2/D3 同一 world_time/target 契约)。
            candidate.k3_window=evaluateLocalK3WindowMetrics(
                candidate.min_jerk_opt.getTraj(),activation,
                candidate.execution_target_position,
                candidate.execution_target_velocity,
                candidate.execution_target_state_time,
                k3_comparison_begin,k3_comparison_end,k3_required_margin);
            const bool candidate_k2_protected =
                candidate.k3_window.valid && k3_baseline_valid_ &&
                candidate.k3_window.k2 >= k3_baseline_window_k2_ - 1.0e-9;
            if(candidate_k2_protected &&
               (candidate.k3_window.c3 > k3_baseline_window_c3_ + 1.0e-9 ||
                candidate.k3_window.d3 < k3_baseline_window_d3_ - 1.0e-9))
            {
              if(k3_event_.active)
                ++k3_event_.progress_candidate_count;
            }
            if(candidate.k3_window.valid)
            {
              auto &aggregate=k3_side_aggregates[static_cast<size_t>(candidate.kind)];
              ++aggregate.safe_count;
              if(k3_baseline_valid_ &&
                 candidate.k3_window.k2 >= k3_baseline_window_k2_ - 1.0e-9)
                ++aggregate.k2_admissible_count;
              if(candidate.k3_window.k2 > aggregate.k2)
                aggregate.k2=candidate.k3_window.k2;
              if(candidate.k3_window.c3 > aggregate.c3)
              {
                aggregate.c3=candidate.k3_window.c3;
                aggregate.best_candidate_id=candidate.candidate_id;
              }
              if(candidate.k3_window.d3 < aggregate.d3)
                aggregate.d3=candidate.k3_window.d3;
            }
          }
        }
        traj_utils::TopologyCandidate trace;
        fillCandidateBinaryVisibility(candidate.min_jerk_opt.getTraj(),activation,geometry_policy_.horizon,
            candidate.execution_target_position,candidate.execution_target_velocity,
            candidate.execution_target_state_time,trace);
        candidate.binary_camera_time=trace.binary_visibility_valid?0.0:-1.0;
        if(trace.binary_visibility_valid)
          for(size_t k=1;k<trace.visibility_sample_offsets.size();++k)
            candidate.binary_camera_time+=(trace.visibility_sample_offsets[k]-trace.visibility_sample_offsets[k-1])*
                trace.self_visibility_samples[k-1];
        // Peer trajectories are fixed across alternatives at this epoch.
        // Selection has exactly one quality order: K2, accumulated visibility,
        // cooperative geometry, then native optimization cost.  All other
        // visibility and geometry fields remain telemetry only.
        ROS_INFO("[local-geometry-candidate] drone=%d candidate=%d hypothesis=%d safe=1 state=%s baseline_ratio=%.6f ratio=%.6f baseline_violation=%.9f violation=%.9f gap_max_deg=%.6f gap_min_deg=%.6f",
            pp_.drone_id,candidate.candidate_id,sets[i].hypothesis_id,
            geometryStateName(classifyGeometry(current,candidate.geometry,geometry_policy_)),current.ratio,
            candidate.geometry.ratio,current.violation,candidate.geometry.violation,candidate.geometry.maximum*180/M_PI,candidate.geometry.minimum*180/M_PI);
        const bool k3_side_blocked_without_baseline =
            k3_escalation_active && k3_event_.active &&
            !k3_baseline_valid_ &&
            candidate.kind != CandidateKind::NOMINAL;
        bool candidate_better = best==nullptr &&
                                !k3_side_blocked_without_baseline;
        std::string k3_decision_reason;
        if (k3_side_blocked_without_baseline)
          k3_decision_reason = "K2_BASELINE_UNAVAILABLE";
        const bool k3_k2_gate_active =
            k3_escalation_active && k3_baseline_valid_;
        const bool candidate_k2_admissible = candidate.k3_window.valid &&
            candidate.k3_window.k2 >= k3_baseline_window_k2_ - 1.0e-9;
        if(k3_k2_gate_active && !candidate_k2_admissible)
        {
          candidate_better = false;
          k3_decision_reason = candidate.k3_window.valid
              ? "K2_GATE_REJECT_CANDIDATE" : "K3_WINDOW_INVALID";
        }
        if(best)
        {
          const bool k3_pair_eligible =
              k3_escalation_active && k3_baseline_valid_ &&
              candidate.k3_window.valid && best->k3_window.valid;
          if(k3_pair_eligible)
          {
            // SAFETY FIRST:两个候选已通过同一 current-revision hard preflight
            // (上方未改动)。K2 PROTECTED:K3 事件窗口 K2 不得低于已执行基线。
            // 低于基线或窗口不完整的候选已在比较前排除;若所有候选都不满足,
            // 本 batch 保留先前执行轨迹,而不让普通 comparator 绕过 K2 门。
            const bool candidate_k2_ok =
                candidate.k3_window.k2 >= k3_baseline_window_k2_ - 1.0e-9;
            const bool best_k2_ok =
                best->k3_window.k2 >= k3_baseline_window_k2_ - 1.0e-9;
            if(k3_decision_reason.empty() &&
               candidate_k2_ok != best_k2_ok)
            {
              candidate_better = candidate_k2_ok;
              k3_decision_reason =
                  candidate_k2_ok ? "K2_GATE_PASS" : "OPPONENT_K2_GATE_PASS";
            }
            else if(k3_decision_reason.empty() && candidate_k2_ok)
            {
              if(std::abs(candidate.k3_window.c3 -
                          best->k3_window.c3) > 1.0e-9)
              {
                candidate_better =
                    candidate.k3_window.c3 > best->k3_window.c3;
                k3_decision_reason = candidate_better ? "C3_HIGHER"
                                                      : "OPPONENT_C3_HIGHER";
              }
              else if(std::abs(candidate.k3_window.d3 -
                               best->k3_window.d3) > 1.0e-9)
              {
                candidate_better =
                    candidate.k3_window.d3 <
                    best->k3_window.d3;
                k3_decision_reason = candidate_better ? "D3_LOWER"
                                                      : "OPPONENT_D3_LOWER";
              }
              else
              {
                // 跨 cycle 轻 side preference:C3/D3 全平时保留上次为同一
                // event 提交过的 side;任何严格更优的 side 仍会胜出,非
                // hard lock。
                const int preferred = k3_event_.active
                                          ? k3_event_.last_committed_side : -1;
                const int candidate_kind = static_cast<int>(candidate.kind);
                const int best_kind = static_cast<int>(best->kind);
                if(preferred >= 0 && candidate_kind == preferred &&
                   best_kind != preferred)
                {
                  candidate_better = true;
                  k3_decision_reason = "SIDE_PREFERENCE_HOLD";
                }
                else if(preferred >= 0 && best_kind == preferred &&
                        candidate_kind != preferred)
                {
                  candidate_better = false;
                  k3_decision_reason = "OPPONENT_SIDE_PREFERENCE_HOLD";
                }
              }
            }
          }
          if(k3_decision_reason.empty())
          {
          // Preserve a target-facing static homotopy once it has passed the
          // same current-revision preflight. A continuous SIDE alternative
          // must not silently switch back to the opposite obstacle side. If
          // the A* candidate fails preflight, SIDE remains the normal fallback.
          if (candidate.target_side_static_topology !=
              best->target_side_static_topology)
          {
            candidate_better = candidate.target_side_static_topology;
            if (candidate_better)
              ROS_INFO("[target-side-topology-selection] drone=%d "
                       "candidate=%d replaced=%d "
                       "reason=PRESERVE_PREFLIGHTED_TARGET_SIDE_ASTAR",
                       pp_.drone_id, candidate.candidate_id,
                       best->candidate_id);
          }
          else if(candidate.visibility.valid && best->visibility.valid)
          {
            if(visibility_better(candidate.visibility,best->visibility))
              candidate_better=true;
            else if(!visibility_better(best->visibility,candidate.visibility))
              candidate_better=geometryBetter(candidate.geometry,best->geometry,current,geometry_policy_,
                  candidate.optimization_cost,best->optimization_cost);
          }
          else
            candidate_better=geometryBetter(candidate.geometry,best->geometry,current,geometry_policy_,
                candidate.optimization_cost,best->optimization_cost);
          if(k3_escalation_active && candidate_better)
            k3_decision_reason = "ORIGINAL_ORDER";
          }
        }
        if(candidate_better)
        {
          if(!k3_decision_reason.empty())
            k3_selected_reason = k3_decision_reason;
          if(reserved_first_safe && best && &candidate!=reserved_first_safe)
            ROS_INFO("[coverage-quality] event=BETTER_CANDIDATE_FOUND_AFTER_FIRST_SAFE drone=%d reserved_candidate=%d better_candidate=%d reserved_all3=%.6f better_all3=%.6f reserved_mean_visible=%.6f better_mean_visible=%.6f reserved_k2=%.6f better_k2=%.6f",
                pp_.drone_id,reserved_first_safe->candidate_id,candidate.candidate_id,
                reserved_first_safe->visibility.all3_visibility,candidate.visibility.all3_visibility,
                reserved_first_safe->visibility.mean_visible_count,candidate.visibility.mean_visible_count,
                reserved_first_safe->visibility.atleast2_visibility,candidate.visibility.atleast2_visibility);
          best=&candidate;selected_set=i;
        }
        executable_candidate_seen_this_batch_=true;
        ROS_INFO("[validated-moving-coverage] event=CANDIDATE_CURRENT_REVISION_PREFLIGHT_PASS drone=%d candidate=%d revision=%lu activation=%.9f t_validated_end=%.9f metadata_recomputed=1",
            pp_.drone_id,candidate.candidate_id,candidate.execution_revision,activation,
            candidate.checked_until);
        if(!reserved_first_safe)
        {
          reserved_first_safe=&candidate;
          reserved_first_safe_set=i;
          ROS_WARN("[coverage-quality] event=FIRST_SAFE_RESERVED drone=%d candidate=%d hypothesis=%d revision=%lu activation=%.9f t_validated_end=%.9f planning_budget_remaining=%.6f",
              pp_.drone_id,candidate.candidate_id,sets[i].hypothesis_id,
              candidate.execution_revision,activation,candidate.checked_until,
              std::max(0.0,planning_deadline_wall_-ros::WallTime::now().toSec()));
        }
      }
      if(deadline_stopped_quality) break;
    }
    if(k3_escalation_active && k3_event_.active)
    {
      const auto &nominal = k3_side_aggregates[0];
      const auto &plus = k3_side_aggregates[1];
      const auto &minus = k3_side_aggregates[2];
      ROS_INFO(
          "[K3_SIDE_CANDIDATES] event_id=%lu cycle=%d "
          "NOMINAL={success=%d K2_admissible=%d K2=%.3f C3=%.3f D3=%.3f} "
          "PLUS={success=%d K2_admissible=%d K2=%.3f C3=%.3f D3=%.3f} "
          "MINUS={success=%d K2_admissible=%d K2=%.3f C3=%.3f D3=%.3f} "
          "selected=%d selection_reason=%s "
          "baseline={K2=%.3f C3=%.3f D3=%.3f}",
          static_cast<unsigned long>(k3_event_.escalation_id),
          k3_event_.cycle_count, nominal.safe_count,
          nominal.k2_admissible_count, nominal.k2,
          nominal.c3 < 0.0 ? 0.0 : nominal.c3,
          nominal.safe_count > 0 && std::isfinite(nominal.d3)
              ? nominal.d3 : 0.0,
          plus.safe_count, plus.k2_admissible_count, plus.k2,
          plus.c3 < 0.0 ? 0.0 : plus.c3,
          plus.safe_count > 0 && std::isfinite(plus.d3) ? plus.d3 : 0.0,
          minus.safe_count, minus.k2_admissible_count, minus.k2,
          minus.c3 < 0.0 ? 0.0 : minus.c3,
          minus.safe_count > 0 && std::isfinite(minus.d3) ? minus.d3 : 0.0,
          best ? best->candidate_id : -1,
          best ? (k3_selected_reason.empty() ? "ORIGINAL_ORDER"
                                             : k3_selected_reason.c_str())
               : "NONE",
          k3_baseline_valid_ ? k3_baseline_window_k2_ : -1.0,
          k3_baseline_valid_ ? k3_baseline_window_c3_ : -1.0,
          k3_baseline_valid_ ? k3_baseline_window_d3_ : -1.0);
    }
    if (!best && reserved_first_safe)
    {
      std::string successor_reason;
      if (localSuccessorIsSafetyRequired(successor_reason))
      {
        best = reserved_first_safe;
        selected_set = reserved_first_safe_set;
        k3_selected_reason = "SAFETY_SUCCESSOR_QUALITY_VETO_BYPASS";
        ROS_WARN("[coverage-quality] event=SAFE_SUCCESSOR_RETAINED "
                 "drone=%d candidate=%d reason=%s",
                 pp_.drone_id, best->candidate_id,
                 successor_reason.c_str());
      }
    }
    if(best)
    {
      const int reserved_candidate_id=reserved_first_safe ? reserved_first_safe->candidate_id : -1;
      const bool quality_upgrade=reserved_first_safe && best!=reserved_first_safe;
      sets[selected_set].local_candidate_id=best->candidate_id;
      const bool selected_k3_progress = k3_escalation_active &&
          k3_event_.active && k3_baseline_valid_ && best->k3_window.valid &&
          best->k3_window.k2 >= k3_baseline_window_k2_ - 1.0e-9 &&
          (best->k3_window.c3 > k3_baseline_window_c3_ + 1.0e-9 ||
           best->k3_window.d3 < k3_baseline_window_d3_ - 1.0e-9);
      if (selected_k3_progress)
      {
        ++k3_event_.progress_selected_count;
        if (best->kind != CandidateKind::NOMINAL)
          ++side_selected_count_;
        ROS_INFO("[K3_LOCAL_PROGRESS_SELECTION] event_id=%lu candidate=%d "
                 "side=%s K2=%.6f C3_before=%.6f C3_after=%.6f "
                 "D3_before=%.6f D3_after=%.6f decision=%s",
            static_cast<unsigned long>(k3_event_.escalation_id),
            best->candidate_id, candidateKindName(best->kind),
            best->k3_window.k2, k3_baseline_window_c3_,
            best->k3_window.c3, k3_baseline_window_d3_,
            best->k3_window.d3,
            k3_selected_reason.empty() ? "ORIGINAL_ORDER"
                                       : k3_selected_reason.c_str());
      }
      // Feedback116:软 LOS SIDE 逐事件因果行(hard-safe 由上方 current-revision
      // preflight 保证;slack>0 只说明尚未完全绕出,不是拒绝依据)。
      if (k3_escalation_active && k3_event_.active &&
          best->kind != CandidateKind::NOMINAL && best->k3_window.valid)
      {
        const bool hard_safe = true;  // preflight 通过才可能成为 best
        const bool c3_improved = k3_baseline_valid_ &&
            best->k3_window.c3 > k3_baseline_window_c3_ + 1.0e-9;
        const bool d3_improved = k3_baseline_valid_ &&
            best->k3_window.d3 < k3_baseline_window_d3_ - 1.0e-9;
        ROS_WARN("[K3_SIDE_SOFT_LOS] event_id=%lu candidate=%d side=%s "
                 "hard_safe=%d K2=%.3f C3_before=%.3f C3_after=%.3f "
                 "D3_before=%.3f D3_after=%.3f c3_improved=%d d3_improved=%d "
                 "selected=%d committed=0 activated=0",
            static_cast<unsigned long>(k3_event_.escalation_id),
            best->candidate_id, candidateKindName(best->kind),
            static_cast<int>(hard_safe), best->k3_window.k2,
            k3_baseline_valid_ ? k3_baseline_window_c3_ : -1.0,
            best->k3_window.c3,
            k3_baseline_valid_ ? k3_baseline_window_d3_ : -1.0,
            best->k3_window.d3, static_cast<int>(c3_improved),
            static_cast<int>(d3_improved),
            static_cast<int>(selected_k3_progress));
        if (c3_improved) ++k3_event_.c3_improved_cycles;
        if (d3_improved) ++k3_event_.d3_improved_cycles;
      }
      const auto status=finalizeTopologyCandidate(*best,sets[selected_set].nominal_risk,
          sets[selected_set].nominal_obstacle_id,touch_goal,head_threshold,sets[selected_set].planning_epoch,
          false,"LOCAL_GEOMETRY_PRESERVATION_OR_RECOVERY");
      // Feedback117 (§10): global SIDE commit stage of the candidate
      // lifecycle (K3-scoped progress counting stays separate below).
      if (status == TopologyProcessStatus::COMMITTED &&
          best->kind != CandidateKind::NOMINAL)
        ++side_committed_count_;
      if(selected_k3_progress && k3_event_.active)
      {
        const bool committed = status == TopologyProcessStatus::COMMITTED;
        if(committed)
        {
          ++k3_event_.progress_committed_count;
          k3_event_.last_committed_side = static_cast<int>(best->kind);
          k3_event_.last_committed_d3 =
              best->k3_window.valid ? best->k3_window.d3
                                    : k3_event_.last_committed_d3;
          k3_event_.pending_activation_check = true;
          k3_event_.pending_traj_id = traj_.local_traj.traj_id;
          k3_event_.pending_c3 =
              best->k3_window.valid ? best->k3_window.c3 : 0.0;
          k3_event_.pending_d3 =
              best->k3_window.valid ? best->k3_window.d3 : 0.0;
          k3_event_.pending_k2 =
              best->k3_window.valid ? best->k3_window.k2 : 0.0;
          k3_event_.pending_c3_before = k3_baseline_window_c3_;
          k3_event_.pending_d3_before = k3_baseline_window_d3_;
        }
        ROS_WARN(
            "[K3_LOCAL_PROGRESS] event_id=%lu target=%d blocker=%s "
            "selected_side=%s C3_before=%.3f C3_after=%.3f "
            "D3_before=%.3f D3_after=%.3f trajectory_id=%d "
            "selected=1 committed=%d activated=0",
            static_cast<unsigned long>(k3_event_.escalation_id),
            pp_.drone_id, "LOCAL_UNIFIED_SAMPLE",
            candidateKindName(best->kind),
            k3_baseline_valid_ ? k3_baseline_window_c3_ : -1.0,
            best->k3_window.valid ? best->k3_window.c3 : -1.0,
            k3_baseline_valid_ ? k3_baseline_window_d3_ : -1.0,
            best->k3_window.valid ? best->k3_window.d3 : -1.0,
            traj_.local_traj.traj_id, static_cast<int>(committed));
      }
      // Feedback116:软 LOS SIDE 的 commit 结果行(与上方 [K3_SIDE_SOFT_LOS]
      // 的 selected 行按 candidate_id 对齐)。
      if (k3_escalation_active && k3_event_.active &&
          best->kind != CandidateKind::NOMINAL && best->k3_window.valid)
        ROS_WARN("[K3_SIDE_SOFT_LOS_COMMIT] event_id=%lu candidate=%d "
                 "committed=%d trajectory_id=%d",
            static_cast<unsigned long>(k3_event_.escalation_id),
            best->candidate_id,
            static_cast<int>(status == TopologyProcessStatus::COMMITTED),
            traj_.local_traj.traj_id);
      if(status==TopologyProcessStatus::COMMITTED && quality_upgrade)
      {
        pending_quality_upgrade_traj_id_=traj_.local_traj.traj_id;
        pending_quality_upgrade_candidate_id_=best->candidate_id;
        ROS_INFO("[coverage-quality] event=BETTER_CANDIDATE_SELECTED_AFTER_FIRST_SAFE drone=%d reserved_candidate=%d selected_candidate=%d trajectory_id=%d",
            pp_.drone_id,reserved_candidate_id,best->candidate_id,traj_.local_traj.traj_id);
      }
      return status;
    }

    std::string reason;double clearance;
    // 本轮修复（rolling/terminal fallback 语义）：被校验对象是本机"当前已安装、
    // 正在执行"的 traj_.local_traj，它的权威时域由它自己被提交时携带的语义
    // 决定，而不是由这里硬编码的 true。rolling 轨迹会在 ~0.3 s 内被替换，用
    // terminal 的全时长语义去检查它 2.0 s 之外的尾部，会把当下完全安全的可
    // 执行前缀整条否掉（返回 FAILED 而不是 RETAINED_PREVIOUS），直接饿死
    // successor 供应。这与 PM:1975 的既有契约保持一致。
    if(validatePreviousRemainingTrajectory(ros::Time::now().toSec(),
                                           active_execution_touch_goal_,reason,
                                           clearance))
    {
      active_execution_source_="PERSISTENCE_FALLBACK";
      // 与 finalizeTopologyCandidate 的 KEEP_PREVIOUS_SAFE 出口对齐：
      // 保留即视为已通过安全复核。
      active_execution_safety_validated_=true;
      ROS_INFO("[previous-safe-final-revalidation] stage=PERSISTENCE_FALLBACK "
               "previous_id=%d generation=%lu remaining_valid=1 "
               "active_execution_touch_goal=%d min_dynamic_clearance=%.6f",
               traj_.local_traj.traj_id, active_traj_generation_,
               static_cast<int>(active_execution_touch_goal_), clearance);
      logExecutionGeometry("PERSISTENCE_REVALIDATION");
      return TopologyProcessStatus::RETAINED_PREVIOUS;
    }
    ROS_WARN("[previous-safe-final-revalidation] stage=PERSISTENCE_FALLBACK "
             "previous_id=%d generation=%lu remaining_valid=0 reason=%s",
             traj_.local_traj.traj_id, active_traj_generation_, reason.c_str());
    return TopologyProcessStatus::FAILED;
  }

  trajectory_lifecycle::State EGOPlannerManager::currentStateAt(
      const double stamp) const
  {
    trajectory_lifecycle::State state;
    const double dt=std::max(0.0,stamp-current_state_stamp_);
    state.p=current_state_p_+current_state_v_*dt+
        0.5*current_state_a_*dt*dt;
    state.v=current_state_v_+current_state_a_*dt;
    state.a=current_state_a_;
    return state;
  }

  void EGOPlannerManager::prepareFutureActivation(Eigen::Vector3d &p,
      Eigen::Vector3d &v, Eigen::Vector3d &a)
  {
    local_planning_started_ = ros::Time::now().toSec();
    const auto &active = traj_.local_traj;
    // FROZEN-ACTIVATION contract (single optimizer authority): the batch's
    // only activation is fixed HERE, before any candidate exists.  Generation,
    // target/dynamic prediction, handoff and finalization all share this one
    // world-time origin; finalization may never move it and therefore never
    // rebuilds the optimized polynomial.
    // The planning reserve is the measured execution-budget estimate — the
    // SAME authority `canStart` admitted this batch with — so the boundary is
    // conservative against real pipeline latency yet normally lands while the
    // predecessor still owns execution.  No second timing estimator exists.
    double frozen_activation =
        local_planning_started_ + local_activation_margin_;
    if (!current_state_restart_active_)
    {
      // The reserve is the measured PLANNING latency (EWMA, bounded) — pure
      // pipeline time, not the supervisory batch budget: the supervisory
      // estimate also counts Team-coordination waits and, used as a
      // scheduling reserve, once froze an activation 1.2 s out and let the
      // generator reduce every candidate to a stationary sliver inside the
      // predecessor's remaining coverage window.
      frozen_activation += local_planning_budget_;
      // Never freeze beyond the predecessor's validated handoff deadline.
      if (std::isfinite(planning_deadline_ros_))
        frozen_activation = std::min(frozen_activation, planning_deadline_ros_);
    }
    local_activation_time_ = std::max(
        frozen_activation, local_planning_started_ + local_activation_margin_);
    if(current_state_restart_active_)
    {
      // The caller supplies the production odometry P/V and its existing
      // acceleration convention.  Preserve that authority instead of
      // sampling the expired predecessor's terminal clamp.
      current_state_p_=p;
      current_state_v_=v;
      current_state_a_=a;
      current_state_stamp_=local_planning_started_;
      current_state_valid_=p.allFinite() && v.allFinite() && a.allFinite();
      if(current_state_valid_)
      {
        const auto state=currentStateAt(local_activation_time_);
        p=state.p;v=state.v;a=state.a;
      }
      ROS_WARN("[current-state-restart] event=STATE drone=%d expired_predecessor_id=%d source=ODOMETRY_PV_EXISTING_ACCELERATION_CONVENTION stamp=%.9f activation=%.9f p=(%.6f,%.6f,%.6f) v=(%.6f,%.6f,%.6f) a=(%.6f,%.6f,%.6f) valid=%d",
          pp_.drone_id,expired_predecessor_id_,
          current_state_stamp_,local_activation_time_,p.x(),p.y(),p.z(),
          v.x(),v.y(),v.z(),a.x(),a.y(),a.z(),int(current_state_valid_));
    }
    else if (active.traj_id > 0 && active.traj.getPieceNum() > 0)
    {
      const auto state = trajectory_lifecycle::sample(active.traj, local_activation_time_-active.start_time);
      p = state.p; v = state.v; a = state.a;
    }
    ROS_INFO("[future-activation] drone=%d plan_start=%.9f activation_frozen=%.9f planning_reserve=%.6f activation_margin=%.6f t_validated_end=%.9f t_planning_deadline=%.9f old_id=%d current_state_restart=%d",
        pp_.drone_id, local_planning_started_, local_activation_time_,
        local_activation_time_ - local_planning_started_ - local_activation_margin_,
        local_activation_margin_,validated_coverage_end_,
        planning_deadline_ros_, active.traj_id,int(current_state_restart_active_));
  }

  bool EGOPlannerManager::rebuildLocalCandidateAtActivation(poly_traj::MinJerkOpt &opt,
      double &activation, std::string *reason,
      std::vector<LocalSfcPlane> *local_sfc_planes, bool reanchor_to_now)
  {
    // REBUILD contract, foreign payloads only (Team relay anchor probe,
    // defensive commit of a not-yet-prepared candidate).  This function owns
    // polynomial-mutation authority; the ordinary Local N/L/R production
    // contract must never enter here — it validates through
    // validateLocalHandoffWindow + checkActiveHandoff against the frozen
    // batch activation instead.
    const double now = ros::Time::now().toSec();
    // 本函数会把 activation 重新锚定到"候选就绪时刻"。世界时间锚定的 LOS 平面
    // 必须以世界时间为不变量跟着一起换算，否则遮挡事件会随 activation 漂移。
    const double activation_before = activation;
    // 任意 return 路径都会执行：只要 activation 真的变了，就把世界时间锚定平面
    // 从旧锚点换算到新锚点。
    struct PlaneRebaseGuard
    {
      std::vector<LocalSfcPlane> *planes;
      const double old_anchor;
      double &activation_ref;
      ~PlaneRebaseGuard()
      {
        if (planes == nullptr) return;
        if (std::abs(activation_ref - old_anchor) <= 1.0e-9) return;
        for (LocalSfcPlane &plane : *planes)
          rebaseWorldTimeAnchoredPlane(plane, activation_ref);
      }
    } plane_rebase_guard{local_sfc_planes, activation_before, activation};
    if (reanchor_to_now)
    {
      // Candidate-ready time, not the stale plan-start estimate, owns the next
      // executable activation. Rebuilding below changes the polynomial and is
      // therefore followed by all current-revision hard checks at the caller.
      activation = now + local_activation_margin_;
    }
    // reanchor_to_now=false：使用调用方冻结的 batch activation（阶段 E）。
    // 全部候选由此共享同一 activation 与同一 predecessor 边界。
    const auto &active = traj_.local_traj;
    if(current_state_restart_active_)
    {
      if(!current_state_valid_)
      {
        if(reason) *reason="CURRENT_STATE_INVALID";
        return false;
      }
      const auto candidate=opt.getTraj();
      if(candidate.getPieceNum()<=0)
      {
        if(reason) *reason="INVALID_TRAJECTORY";
        return false;
      }
      const auto state=currentStateAt(activation);
      Eigen::Matrix3d head,tail;
      head<<state.p,state.v,state.a;
      const double end=candidate.getTotalDuration();
      tail<<candidate.getPos(end),candidate.getVel(end),candidate.getAcc(end);
      opt.reset(head,tail,candidate.getPieceNum());
      opt.generate(candidate.getPositions().middleCols(1,candidate.getPieceNum()-1),
          candidate.getDurations());
      if(reason) reason->clear();
      ROS_INFO("[current-state-restart] event=CANDIDATE_REANCHORED drone=%d expired_predecessor_id=%d activation=%.9f source=ODOMETRY_PROPAGATED_PVA head_p=(%.6f,%.6f,%.6f) head_v=(%.6f,%.6f,%.6f) head_a=(%.6f,%.6f,%.6f)",
          pp_.drone_id,expired_predecessor_id_,activation,
          state.p.x(),state.p.y(),state.p.z(),state.v.x(),state.v.y(),state.v.z(),
          state.a.x(),state.a.y(),state.a.z());
      return true;
    }
    if (active.traj_id <= 0 || active.traj.getPieceNum() <= 0) return true;
    double validated_end=active.start_time+active.duration;
    if(execution_safe_until_>0.0)
      validated_end=std::min(validated_end,execution_safe_until_);
    const ValidatedMovingCoverageWindow window{now,validated_end,local_activation_margin_};
    if(!window.valid() || !window.canStillHandoff())
    {
      if(reason) *reason="MOVING_SUFFIX_EXHAUSTED";
      ROS_ERROR("[validated-moving-coverage] event=EXECUTABLE_SUCCESSOR_READY_TOO_LATE drone=%d trajectory_id=%d candidate_ready=%.9f t_activation_new=%.9f t_validated_end=%.9f planning_budget_remaining=%.6f",
          pp_.drone_id,active.traj_id,now,activation,validated_end,
          window.planningBudgetRemaining());
      return false;
    }
    const auto state = trajectory_lifecycle::sample(active.traj, activation-active.start_time);
    const auto candidate = opt.getTraj();
    if (candidate.getPieceNum() <= 0) {
      if(reason) *reason="INVALID_TRAJECTORY";
      return false;
    }
    Eigen::Matrix3d head, tail;
    head << state.p, state.v, state.a;
    const double end = candidate.getTotalDuration();
    tail << candidate.getPos(end), candidate.getVel(end), candidate.getAcc(end);
    opt.reset(head, tail, candidate.getPieceNum());
    opt.generate(candidate.getPositions().middleCols(1,candidate.getPieceNum()-1), candidate.getDurations());
    if(reason) reason->clear();
    ROS_INFO("[validated-moving-coverage] event=SUCCESSOR_REANCHORED_TO_LATEST_ACTIVATION drone=%d trajectory_id=%d candidate_ready=%.9f old_activation=%.9f t_activation_new=%.9f t_validated_end=%.9f head_p=(%.6f,%.6f,%.6f) head_v=(%.6f,%.6f,%.6f) head_a=(%.6f,%.6f,%.6f)",
        pp_.drone_id,active.traj_id,now,local_activation_time_,activation,
        validated_end,state.p.x(),state.p.y(),state.p.z(),state.v.x(),state.v.y(),
        state.v.z(),state.a.x(),state.a.y(),state.a.z());
    return true;
  }

  bool EGOPlannerManager::validateLocalHandoffWindow(double activation,
      std::string *reason) const
  {
    // Validation-only handoff contract: the optimizer owns the polynomial and
    // this check owns only scheduling.  A candidate whose pipeline missed its
    // frozen activation is rejected whole (MISSED_FROZEN_ACTIVATION); the
    // safe incumbent keeps executing and the next ordinary rolling tick
    // refreezes a later boundary from the true predecessor state.  Nothing
    // here may reset, regenerate or re-anchor a candidate.
    if (current_state_restart_active_)
    {
      // Odometry authority: the expired predecessor owns no window.  The
      // restart branch of checkActiveHandoff validates continuity against the
      // odometry-propagated state instead.
      if (!current_state_valid_)
      {
        if (reason) *reason = "CURRENT_STATE_INVALID";
        return false;
      }
      if (reason) reason->clear();
      return true;
    }
    const double now = ros::Time::now().toSec();
    const auto &active = traj_.local_traj;
    if (active.traj_id > 0 && active.traj.getPieceNum() > 0)
    {
      double validated_end = active.start_time + active.duration;
      if (execution_safe_until_ > 0.0)
        validated_end = std::min(validated_end, execution_safe_until_);
      const ValidatedMovingCoverageWindow window{
          now, validated_end, local_activation_margin_};
      if (!window.activationFits(activation))
      {
        const bool missed = activation + 1.0e-9 < window.activationEarliest();
        if (reason)
          *reason = missed ? "MISSED_FROZEN_ACTIVATION"
                           : "MOVING_SUFFIX_EXHAUSTED";
        ROS_WARN("[handoff-validation] drone=%d event=FROZEN_SLOT_UNAVAILABLE "
                 "reason=%s activation=%.9f now=%.9f activation_earliest=%.9f "
                 "validated_end=%.9f plan_start=%.9f",
            pp_.drone_id, reason->c_str(), activation, now,
            window.activationEarliest(), validated_end,
            local_planning_started_);
        return false;
      }
    }
    if (reason) reason->clear();
    return true;
  }

  bool EGOPlannerManager::checkActiveHandoff(const poly_traj::Trajectory &candidate,
      double activation, const char *new_source, bool log) const
  {
    // Feedback093: for the GUARANTEED LOCAL successor the predecessor authority
    // is the executor-CONFIRMED ACTIVE trajectory, not this planner's most
    // recent local commit.  Chaining from the local commit made the planner
    // advance 76->106 while the executor was still executing 76, and every
    // successor was then rejected downstream.
    //
    // A Team realization is a different object: it is a speculative replacement
    // whose declared predecessor is the frontier owner reported to the
    // coordinator (`frontier_owner_trajectory_ids`), so it keeps anchoring on
    // this planner's committed local frontier.
    const bool guaranteed_local_successor =
        std::string(new_source) == "LOCAL_SUCCESSOR";
    const auto &active = guaranteed_local_successor ? predecessorAuthority()
                                                    : traj_.local_traj;
    if (candidate.getPieceNum() <= 0 || !std::isfinite(activation)) return false;
    if(current_state_restart_active_)
    {
      if(!current_state_valid_) return false;
      const auto expected=currentStateAt(activation);
      const auto next_state=trajectory_lifecycle::sample(candidate,0.0);
      const auto residual=trajectory_lifecycle::compare(expected,next_state);
      trajectory_lifecycle::Tolerances tolerances;
      tolerances.p=handoff_position_tolerance_;
      tolerances.v=handoff_velocity_tolerance_;
      tolerances.a=handoff_acceleration_tolerance_;
      const bool valid=residual.accepted(tolerances);
      if(log) ROS_INFO("[current-state-restart] event=CURRENT_STATE_HANDOFF drone=%d expired_predecessor_id=%d NEW_SOURCE=%s NEW_ID=%d activation=%.9f dp=%.9f dv=%.9f da=%.9f accepted=%d old_predecessor_authority=0",
          pp_.drone_id,expired_predecessor_id_,new_source,
          active.traj_id+1,activation,residual.dp,residual.dv,residual.da,int(valid));
      return valid;
    }
    if (active.traj_id <= 0 || active.traj.getPieceNum() <= 0) return true;
    double validated_end=active.start_time+active.duration;
    if(execution_safe_until_>0.0)
      validated_end=std::min(validated_end,execution_safe_until_);
    if(activation>=validated_end-1.0e-9)
    {
      if(log) ROS_ERROR("[handoff-gate] drone=%d OLD_SOURCE=%s NEW_SOURCE=%s OLD_ID=%d NEW_ID=%d ACTIVATION_TIME=%.9f t_validated_end=%.9f old_remaining=%.6f accepted=0 reason=PREDECESSOR_VALIDATED_END_EXCEEDED",
          pp_.drone_id,active_execution_source_.c_str(),new_source,active.traj_id,
          active.traj_id+1,activation,validated_end,validated_end-activation);
      return false;
    }
    const auto old_state = trajectory_lifecycle::sample(active.traj, activation-active.start_time);
    const auto next_state = trajectory_lifecycle::sample(candidate, 0.0);
    const auto residual = trajectory_lifecycle::compare(old_state, next_state);
    trajectory_lifecycle::Tolerances tolerances;
    tolerances.p=handoff_position_tolerance_;
    tolerances.v=handoff_velocity_tolerance_;
    tolerances.a=handoff_acceleration_tolerance_;
    const bool valid=residual.accepted(tolerances);
    if (log) ROS_INFO("[handoff-gate] drone=%d HANDOFF_DP=%.9f HANDOFF_DV=%.9f HANDOFF_DA=%.9f OLD_SOURCE=%s NEW_SOURCE=%s OLD_ID=%d NEW_ID=%d old_generation=%lu new_generation=%lu ACTIVATION_TIME=%.9f old_remaining=%.6f accepted=%d",
        pp_.drone_id,residual.dp,residual.dv,residual.da,active_execution_source_.c_str(),new_source,
        active.traj_id,active.traj_id+1,active_traj_generation_,active_traj_generation_+1,
        activation,active.start_time+active.duration-activation,int(valid));
    return valid;
  }

  bool EGOPlannerManager::lifecycleSuccessorDue()
  {
    const double now=ros::Time::now().toSec();
    const auto &active=traj_.local_traj;
    if(active.traj_id<=0 || active.traj.getPieceNum()<=0 || now<active.start_time) return false;
    const double remaining=active.start_time+active.duration-now;
    if(now>=lifecycle_validation_expiry_)
    {
      std::string reason; double clearance;
      // Revalidate using the semantic contract carried by the installed
      // trajectory.  A rolling local endpoint must not be upgraded to a
      // mission terminal merely because this lifecycle check runs later.
      const bool valid=remaining>0 && validatePreviousRemainingTrajectory(
          now, active_execution_touch_goal_, reason, clearance);
      logExecutionGeometry("SUFFIX_REVALIDATION");
      lifecycle_last_validation_=now;
      lifecycle_validation_expiry_=now+0.15;
      lifecycle_invalidated_=!valid;
      lifecycle_state_=valid ? "CURRENTLY_VALID" : "INVALIDATED";
      execution_safe_until_=valid
          ? std::min(active.start_time+active.duration,
              now+executionAuthorityHorizon(remaining,active_execution_touch_goal_))
          : now;
      if(!valid)
      {
        std::string prefix_reason;
        // A conflict later in the suffix does not erase the verified moving
        // time before it. Validate a prefix covering the existing conservative
        // solve budget, then shorten it if necessary. The old fixed ~0.19 s
        // prefix was shorter than the coverage budget by construction.
        const double desired=execution_budget_.estimate(ros::WallTime::now().toSec())+
            local_activation_margin_+execution_margin_;
        for(double horizon=std::min(remaining,desired);
            horizon>=local_activation_margin_;horizon*=0.5) {
          if(validateActivePrefixUntil(now+horizon,prefix_reason)) {
            execution_safe_until_=now+horizon;
            break;
          }
        }
        ROS_WARN("[execution-prefix] drone=%d trajectory_id=%d suffix_invalid=1 safe_until=%.9f prefix_valid=%d",
            pp_.drone_id,active.traj_id,execution_safe_until_,int(execution_safe_until_>now));
      }
      if(!valid)
        ROS_WARN("[execution-lifecycle] event=ACTIVE_TRAJECTORY_INVALIDATED drone=%d trajectory_id=%d generation=%lu remaining=%.6f last_validation=%.9f validation_expiry=%.9f reason=%s action=URGENT_MOVING_REPLAN",
            pp_.drone_id,active.traj_id,active_traj_generation_,remaining,now,lifecycle_validation_expiry_,reason.c_str());
    }
    const bool expiring=trajectory_lifecycle::successorDue(remaining,execution_budget_.estimate(ros::WallTime::now().toSec()),local_activation_margin_,execution_margin_);
    if(expiring && !lifecycle_invalidated_) lifecycle_state_=remaining>0 ? "EXPIRING" : "TERMINAL_HOLD";
    return lifecycle_invalidated_ || expiring;
  }



  bool EGOPlannerManager::validateRetimedLocalSfc(
      const CandidateResult &source, const poly_traj::Trajectory &trajectory) const
  {
    if (!ploy_traj_opt_ || trajectory.getPieceNum() <= 0)
      return false;
    const double violation = ploy_traj_opt_->continuousLocalSfcMaxViolation(
        trajectory, source.local_sfc_planes, nullptr);
    if (!std::isfinite(violation) || violation > 1.0e-6)
      ROS_WARN("[retime-sfc-reject] reason=STATIC_CORRIDOR_VIOLATION "
               "continuous_violation=%.9f", violation);
    return std::isfinite(violation) && violation <= 1.0e-6;
  }

  bool EGOPlannerManager::validateExecutionTrajectory(
      const poly_traj::Trajectory &trajectory, const double activation,
      const bool touch_goal, std::string &reason,
      const traj_utils::TeamTrajectorySolution *team) const
  {
    double velocity=0.0, acceleration=0.0, jerk=0.0;
    if (!std::isfinite(activation) || trajectory.getPieceNum()<=0) { reason="INVALID_TRAJECTORY"; return false; }
    if (!checkTrajectoryDynamics(trajectory,velocity,acceleration,jerk)) { reason="DYNAMICS_FAIL"; return false; }
    if (!checkTrajectoryStaticSafety(trajectory, touch_goal, reason)) return false;
    const auto risk=evaluateDynamicRisk(trajectory,activation,touch_goal);
    if (!risk.valid)
    { reason="DYNAMIC_PREDICTION_INVALID"; return false; }
    if (risk.hard_collision)
    { reason="HARD_DYNAMIC_COLLISION_FAIL"; return false; }
    if (!team)
      return checkTrajectorySwarmSafety(trajectory,activation,touch_goal,reason);
    if (team->trajectories.size()!=3 || team->drone_ids.size()!=3 ||
        std::abs(team->activation_time.toSec()-activation)>1e-6)
    { reason="TEAM_IDENTITY_FAIL"; return false; }
    // Recheck against the exact common-activation payload, not stale peers.
    for (size_t i=0;i<3;++i)
    {
      if (team->drone_ids[i]==pp_.drone_id) continue;
      poly_traj::MinJerkOpt peer;
      if (!mincoMessageToOptimizer(team->trajectories[i],peer)) { reason="TEAM_RECONSTRUCTION_FAIL"; return false; }
      const auto other=peer.getTraj();
      const double horizon=std::min(
          executionAuthorityHorizon(trajectory.getTotalDuration(), touch_goal),
          executionAuthorityHorizon(other.getTotalDuration(), touch_goal));
      for(double t=0.0;t<=horizon+1e-8;t+=0.03)
      {
        const Eigen::Vector3d d=trajectory.getPos(std::min(t,horizon))-other.getPos(std::min(t,horizon));
        if (d.head<2>().squaredNorm()+0.25*d.z()*d.z()<getSwarmClearance()*getSwarmClearance())
        { reason="TEAM_SWARM_FAIL"; return false; }
      }
    }
    return true;
  }


  double EGOPlannerManager::executionAuthorityHorizon(
      const double duration, const bool touch_goal) const
  {
    if (!std::isfinite(duration) || duration <= 0.0)
      return 0.0;
    if (touch_goal)
      return duration;
    const double configured = ploy_traj_opt_
        ? ploy_traj_opt_->getMovingObjPredictionHorizon() : 0.0;
    return configured > 1.0e-3 ? std::min(duration, configured) : duration;
  }

  double EGOPlannerManager::hardCheckedUntil(
      const poly_traj::Trajectory &trajectory, const double activation,
      const bool touch_goal) const
  {
    if (!std::isfinite(activation) || trajectory.getPieceNum() <= 0 ||
        !grid_map_ || !obj_predictor_)
      return activation;
    const double duration = trajectory.getTotalDuration();
    if (!std::isfinite(duration) || duration <= 0.0)
      return activation;
    const double checked_prefix = executionAuthorityHorizon(duration, touch_goal);
    const double static_until = activation + checked_prefix;
    const double pva_until = activation + duration;
    double dynamic_until = activation + duration;
    int predicted_objects = 0;
    for (int id = 0; id < obj_predictor_->getObjNums(); ++id) {
      if (obj_predictor_->hasObservedObject(id) &&
          !obj_predictor_->hasPrediction(id)) return activation;
      if (obj_predictor_->hasPrediction(id)) ++predicted_objects;
    }
    if (predicted_objects > 0) {
      const double prediction_horizon = ploy_traj_opt_
          ? ploy_traj_opt_->getMovingObjPredictionHorizon() : 0.0;
      if (!std::isfinite(prediction_horizon) || prediction_horizon <= 0.0)
        return activation;
      const double valid_from = obj_predictor_->commonPredictionValidFrom();
      const double valid_to = obj_predictor_->commonPredictionValidTo();
      if (!std::isfinite(valid_from) || !std::isfinite(valid_to) ||
          activation < valid_from - 1.0e-9 || valid_to <= activation)
        return activation;
      dynamic_until = std::min(activation + std::min(duration, prediction_horizon),
                               valid_to);
    }
    double swarm_until = activation + checked_prefix;
    for (int drone = 0; drone < 3; ++drone) {
      if (drone == pp_.drone_id) continue;
      // The FSM starts drones in ID order.  A higher-ID drone that has never
      // published a trajectory is not yet an executing peer.  Every earlier
      // drone, and every peer seen once, must carry a current trajectory.
      if (drone >= static_cast<int>(traj_.swarm_traj.size()) ||
          traj_.swarm_traj[drone].drone_id < 0) {
        if (drone < pp_.drone_id) return activation;
        continue;
      }
      const LocalTrajData &peer =
          traj_.swarm_traj[drone].executionAt(activation);
      const double peer_end = peer.start_time + peer.duration;
      if (peer.drone_id != drone || peer.traj.getPieceNum() <= 0 ||
          !std::isfinite(peer_end) || activation < peer.start_time - 1.0e-9)
        return activation;
      /* Feedback126 §8: an EXPIRED peer trajectory is a motion-model change,
       * not a safety-fact loss.  The executor holds the peer at its final
       * broadcast position; that hold point is a mechanical fact and is
       * verified below like any hold point.  Blocking my renewal because the
       * peer's polynomial horizon passed recreated the fleet-freeze coupling
       * (peer stops committing -> everyone's certificate UNKNOWN). */
      const bool peer_active = activation < peer_end - 1.0e-9;
      // Feedback124 (certificate renewal repair): the peer's checked_until is
      // itself defined by this same peer min, so bounding my certificate with
      // it made fleet certificates mutually recursive -- the bootstrap cap
      // propagated into every later commit and nothing could ever renew (the
      // Feedback123 run froze all three drones at one terminal instant).
      // The peer contribution is rebuilt from mechanical facts only:
      //   * swarm_until is bounded by the peer polynomial horizon peer_end,
      //     which checkTrajectorySwarmSafety actually verifies (including its
      //     beyond-end terminal-hold model);
      //   * the peer executor stop point fly_until decides WHERE the peer
      //     stops; beyond it the peer is a static hold point that is verified
      //     against my trajectory and the map below instead of being trusted
      //     as validated extent.
      // A peer without any certified prefix at all still fails closed.
      if (!std::isfinite(peer.checked_until) ||
          peer.checked_until <= peer.start_time + 1.0e-9)
        return activation;
      if (!peer_active)
      {
        /* Feedback126 §8 expired-peer hold model: the peer executor stopped
         * at its final broadcast position (mechanical fact).  Verify my
         * trajectory against that hold point over my whole verification
         * window instead of failing closed — peer data availability must
         * not masquerade as a safety certificate, and peer expiry must not
         * block my renewal (no cyclic liveness). */
        const Eigen::Vector3d peer_expired_hold =
            peer.traj.getPos(peer.duration);
        if (grid_map_->getInflateOccupancy(peer_expired_hold) != 0)
          return activation;
        const double clearance = getSwarmClearance();
        const double clearance2 = clearance * clearance;
        for (double t = 0.0; t < swarm_until - activation + 1.0e-9; t += 0.03)
        {
          const double tt = std::min(t, swarm_until - activation);
          const Eigen::Vector3d d =
              trajectory.getPos(std::min(tt, duration)) - peer_expired_hold;
          if (d.head<2>().squaredNorm() + 0.25 * d(2) * d(2) < clearance2)
            return activation;
        }
        ROS_INFO_THROTTLE(
            1.0,
            "[PEER_HOLD_MODEL] drone=%d peer=%d activation=%.9f "
            "peer_end=%.9f action=EXPIRED_PEER_HELD_VERIFIED "
            "hold=(%.3f %.3f %.3f)",
            pp_.drone_id, drone, activation, peer_end,
            peer_expired_hold.x(), peer_expired_hold.y(),
            peer_expired_hold.z());
        continue;
      }
      swarm_until = std::min(swarm_until, peer_end);
      const double fly_until =
          std::min(peer_end, std::max(peer.checked_until, activation));
      // Already-stopped peers are frozen at their executor stop point, not at
      // where the broadcast polynomial would be by now.
      double hold_local =
          (peer.checked_until > activation + 1.0e-9 ? fly_until
                                                    : peer.checked_until) -
          peer.start_time;
      hold_local = std::max(0.0, std::min(peer.duration, hold_local));
      const Eigen::Vector3d peer_hold = peer.traj.getPos(hold_local);
      const double hold_from = std::min(fly_until, swarm_until);
      if (swarm_until <= hold_from + 1.0e-6)
        continue;
      if (grid_map_->getInflateOccupancy(peer_hold) != 0)
        return activation;
      const double clearance = getSwarmClearance();
      const double clearance2 = clearance * clearance;
      ROS_INFO_THROTTLE(
          1.0,
          "[certificate-hold-model] drone=%d peer=%d activation=%.9f "
          "fly_until=%.9f peer_end=%.9f peer_checked_until=%.9f "
          "hold_verify_until=%.9f action=PEER_HELD_STATIC_VERIFY",
          pp_.drone_id, drone, activation, fly_until, peer_end,
          peer.checked_until, swarm_until);
      for (double t = hold_from; t < swarm_until + 1.0e-9; t += 0.03)
      {
        const double tt = std::min(t, swarm_until);
        const Eigen::Vector3d d =
            trajectory.getPos(std::min(tt - activation, duration)) - peer_hold;
        if (d.head<2>().squaredNorm() + 0.25 * d(2) * d(2) < clearance2)
          return activation;
      }
    }
    return std::min({static_until, dynamic_until, swarm_until, pva_until});
  }

  const EGOPlannerManager::PredecessorRecord *
  EGOPlannerManager::findPredecessorRecord(const int traj_id) const
  {
    for(auto it=predecessor_history_.rbegin();it!=predecessor_history_.rend();++it)
      if(it->data.traj_id==traj_id)
        return &(*it);
    return nullptr;
  }

  void EGOPlannerManager::recordPredecessorRecord(const LocalTrajData &data,
      const uint64_t generation,const std::string &source,
      const uint64_t team_solution_id,const bool team_speculative)
  {
    if(data.traj_id<=0 || data.traj.getPieceNum()<=0)
      return;
    for(auto it=predecessor_history_.begin();it!=predecessor_history_.end();++it)
    {
      if(it->data.traj_id==data.traj_id)
      {
        *it=PredecessorRecord{data,generation,source,team_solution_id,team_speculative};
        return;
      }
    }
    predecessor_history_.push_back(
        PredecessorRecord{data,generation,source,team_solution_id,team_speculative});
    while(predecessor_history_.size()>64)
      predecessor_history_.pop_front();
  }

  void EGOPlannerManager::reportLineageAudit(bool final_report) const
  {
    static double last_report=0.0;
    const double now=ros::Time::now().toSec();
    if(!final_report && last_report>0.0 && now-last_report<5.0)
      return;
    last_report=now;
    ROS_INFO("[planner-lineage-audit] drone=%d authoritative_predecessor_id=%d "
             "authoritative_predecessor_generation=%lu declared_predecessor_id=%d "
             "authority_switch_count=%lu authority_lookup_miss_count=%lu "
             "silent_predecessor_fallback_count=%lu local_traj_id=%d final=%d",
        pp_.drone_id,authoritative_predecessor_id_,
        static_cast<unsigned long>(authoritative_predecessor_generation_),
        declared_predecessor_id_,
        static_cast<unsigned long>(authority_switch_count_),
        static_cast<unsigned long>(authority_lookup_miss_count_),
        static_cast<unsigned long>(silent_predecessor_fallback_count_),
        traj_.local_traj.traj_id,static_cast<int>(final_report));
  }

  /* =======================================================================
   * Feedback096 — Team refinement scheduling helpers.
   * ===================================================================== */

  // A pending Local successor is *safety required* (and therefore always
  // allowed to replace a Team-refined future) when the existing Local
  // lifecycle authority already says the current coverage is not sufficient.
  // This deliberately reuses the authorities that already gate Local commits
  // instead of inventing a quality heuristic.
  bool EGOPlannerManager::localSuccessorIsSafetyRequired(
      std::string &reason) const
  {
    if (current_state_restart_active_)
    {
      reason = "CURRENT_STATE_RESTART";
      return true;
    }
    if (lifecycle_invalidated_)
    {
      reason = "LIFECYCLE_INVALIDATED";
      return true;
    }
    const double now = ros::Time::now().toSec();
    const auto &active = traj_.local_traj;
    if (active.traj_id <= 0 || active.traj.getPieceNum() <= 0)
    {
      reason = "NO_ACTIVE_FUTURE";
      return true;
    }
    const double validated_end = std::min(
        active.start_time + active.duration,
        std::isfinite(execution_safe_until_)
            ? std::max(now, execution_safe_until_)
            : active.start_time + active.duration);
    const double coverage = validated_end - now;
    if (coverage <= local_activation_margin_ + execution_margin_)
    {
      reason = "COVERAGE_EXHAUSTING";
      return true;
    }
    reason = "QUALITY_ONLY";
    return false;
  }

  // Feedback100: one uniform latency-chain emitter used by every terminal
  // outcome of a handoff-contract refinement.  Wall-clock spans are monotonic
  // and host-local; the world-time anchors stay in world time.
  void EGOPlannerManager::logRelayLatency(const char *outcome) const
  {
    const double detect_wall = team_refinement_.detect_wall;
    const double receive_wall = team_refinement_.receive_wall;
    const double scp_begin = team_refinement_.scp_begin_wall;
    const double scp_end = team_refinement_.scp_end_wall;
    const double now_wall = ros::WallTime::now().toSec();
    const double time_to_cross =
        team_refinement_.t_cross - team_refinement_.forecast_created_world;
    const double l_detect_to_here =
        (detect_wall > 0.0) ? (now_wall - detect_wall) : -1.0;
    const double slack = (detect_wall > 0.0)
        ? (time_to_cross - l_detect_to_here) : -1.0;
    ROS_INFO("[RELAY_LATENCY] drone=%d contract_id=%lu outcome=%s "
             "L_detect_to_receive_ms=%.3f L_receive_to_scp_ms=%.3f "
             "L_scp_ms=%.3f L_scp_to_outcome_ms=%.3f "
             "L_forecast_to_outcome_ms=%.3f "
             "time_to_cross_at_forecast_s=%.3f INTERVENTION_SLACK_s=%.3f "
             "t_cross=%.9f t_star=%.9f repair_selected_uav=%d",
        pp_.drone_id,
        static_cast<unsigned long>(team_refinement_.contract_id),
        (outcome != nullptr) ? outcome : "UNKNOWN",
        (receive_wall > 0.0 && detect_wall > 0.0)
            ? (receive_wall - detect_wall) * 1000.0 : -1.0,
        (scp_begin > 0.0 && receive_wall > 0.0)
            ? (scp_begin - receive_wall) * 1000.0 : -1.0,
        (scp_end > 0.0 && scp_begin > 0.0)
            ? (scp_end - scp_begin) * 1000.0 : -1.0,
        (scp_end > 0.0) ? (now_wall - scp_end) * 1000.0 : -1.0,
        l_detect_to_here * 1000.0, time_to_cross, slack,
        team_refinement_.t_cross, team_refinement_.t_star,
        team_refinement_.trial_selected_uav);
  }

  void EGOPlannerManager::reportTeamRefinementAudit(bool final_report) const
  {
    static double last_report = 0.0;
    const double now = ros::Time::now().toSec();
    if (!final_report && last_report > 0.0 && now - last_report < 5.0)
      return;
    last_report = now;
    const auto pct = [](std::vector<double> v, const double q) {
      if (v.empty())
        return 0.0;
      std::sort(v.begin(), v.end());
      const size_t index = std::min(
          v.size() - 1, static_cast<size_t>(q * static_cast<double>(v.size() - 1)));
      return v[index];
    };
    ROS_INFO("[team-refinement-audit] drone=%d started=%lu solver_pass=%lu "
             "adopted=%lu activated=%lu coalesced=%lu no_longer_needed=%lu "
             "stale_anchor=%lu latest_recheck_reject=%lu too_late=%lu "
             "solver_infeasible=%lu zero_modification=%lu "
             "current_baseline_safe=%lu quality_overwrite_blocked=%lu "
             "safety_override_team=%lu solve_ms_p50=%.3f solve_ms_p95=%.3f "
             "solve_ms_max=%.3f total_ms_p50=%.3f total_ms_p95=%.3f "
             "total_ms_max=%.3f updates_during_p50=%.1f updates_during_p95=%.1f "
             "updates_during_max=%.1f exec_duration_p50=%.3f activated_traj=%d "
             "certificate_valid=%d final=%d",
        pp_.drone_id,
        team_ref_started_, team_ref_solver_pass_, team_ref_adopted_,
        team_ref_activated_, team_ref_coalesced_, team_ref_no_longer_needed_,
        team_ref_stale_anchor_, team_ref_latest_recheck_reject_,
        team_ref_too_late_, team_ref_solver_infeasible_,
        team_ref_zero_modification_, team_ref_current_baseline_safe_,
        local_quality_overwrite_blocked_, local_safety_override_team_,
        pct(team_ref_solve_ms_samples_, 0.5), pct(team_ref_solve_ms_samples_, 0.95),
        team_ref_solve_ms_samples_.empty()
            ? 0.0 : *std::max_element(team_ref_solve_ms_samples_.begin(),
                                      team_ref_solve_ms_samples_.end()),
        pct(team_ref_total_ms_samples_, 0.5), pct(team_ref_total_ms_samples_, 0.95),
        team_ref_total_ms_samples_.empty()
            ? 0.0 : *std::max_element(team_ref_total_ms_samples_.begin(),
                                      team_ref_total_ms_samples_.end()),
        pct(team_ref_updates_samples_, 0.5), pct(team_ref_updates_samples_, 0.95),
        team_ref_updates_samples_.empty()
            ? 0.0 : *std::max_element(team_ref_updates_samples_.begin(),
                                      team_ref_updates_samples_.end()),
        pct(team_ref_execution_durations_, 0.5),
        team_improvement_certificate_.trajectory_id,
        static_cast<int>(team_improvement_certificate_.valid),
        static_cast<int>(final_report));
    if (final_report)
    {
      ROS_INFO("[team-refinement-latency] drone=%d T_only n=%zu "
               "solve_ms_p50=%.3f solve_ms_p95=%.3f solve_ms_max=%.3f "
               "total_ms_p50=%.3f total_ms_p95=%.3f total_ms_max=%.3f",
          pp_.drone_id, team_ref_t_solve_ms_samples_.size(),
          pct(team_ref_t_solve_ms_samples_, 0.5),
          pct(team_ref_t_solve_ms_samples_, 0.95),
          team_ref_t_solve_ms_samples_.empty()
              ? 0.0 : *std::max_element(team_ref_t_solve_ms_samples_.begin(),
                                        team_ref_t_solve_ms_samples_.end()),
          pct(team_ref_t_total_ms_samples_, 0.5),
          pct(team_ref_t_total_ms_samples_, 0.95),
          team_ref_t_total_ms_samples_.empty()
              ? 0.0 : *std::max_element(team_ref_t_total_ms_samples_.begin(),
                                        team_ref_t_total_ms_samples_.end()));
      ROS_INFO("[team-refinement-latency] drone=%d PT_only n=%zu "
               "solve_ms_p50=%.3f solve_ms_p95=%.3f solve_ms_max=%.3f "
               "total_ms_p50=%.3f total_ms_p95=%.3f total_ms_max=%.3f",
          pp_.drone_id, team_ref_pt_solve_ms_samples_.size(),
          pct(team_ref_pt_solve_ms_samples_, 0.5),
          pct(team_ref_pt_solve_ms_samples_, 0.95),
          team_ref_pt_solve_ms_samples_.empty()
              ? 0.0 : *std::max_element(team_ref_pt_solve_ms_samples_.begin(),
                                        team_ref_pt_solve_ms_samples_.end()),
          pct(team_ref_pt_total_ms_samples_, 0.5),
          pct(team_ref_pt_total_ms_samples_, 0.95),
          team_ref_pt_total_ms_samples_.empty()
              ? 0.0 : *std::max_element(team_ref_pt_total_ms_samples_.begin(),
                                        team_ref_pt_total_ms_samples_.end()));
    }
  }

  void EGOPlannerManager::annotateExecutionSource(traj_utils::PolyTraj &message) const
  {
    message.trajectory_source=active_execution_source_;
    // Feedback093 explicit lineage contract.  The identity is never inferred by
    // the executor; it is declared here and resolved exactly there.
    message.expected_predecessor_id=declared_predecessor_id_;
    message.expected_predecessor_generation=declared_predecessor_generation_;
    message.lineage_class="LOCAL";
    message.generation=active_traj_generation_;
    message.validation_revision=active_validation_revision_;
    message.checked_until=execution_safe_until_;
    message.team_solution_id=active_execution_team_id_;
    message.team_reference_id=active_reference_id_;
    message.team_reference_reason=active_reference_reason_;
    message.encirclement_generation=active_execution_view_generation_;
    message.viewpoint_hypothesis_id=active_execution_hypothesis_;
    message.safety_validated=active_execution_safety_validated_ &&
        (!lifecycle_invalidated_ || ros::Time::now().toSec() < execution_safe_until_);
    message.candidate_ready_time=ros::Time(candidate_ready_time_);
    message.commit_time=ros::Time(commit_time_);
    message.pending_duration=last_pending_duration_;
    message.validation_time=ros::Time(lifecycle_last_validation_);
    message.validation_expiry=ros::Time(lifecycle_validation_expiry_);
    message.lifecycle_state=lifecycle_state_;
  }

  bool EGOPlannerManager::hasCommittedPendingTeamTail() const
  {
    return pending_team_trajectory_.accepted &&
        pending_team_trajectory_.commit_received;
  }

  bool EGOPlannerManager::revalidateCommittedPendingTeamTail(
      std::string &reason) const
  {
    if (!hasCommittedPendingTeamTail())
    {
      reason = "NO_COMMITTED_TEAM_TAIL";
      return false;
    }
    return validateExecutionTrajectory(
        pending_team_trajectory_.trajectory.getTraj(),
        pending_team_trajectory_.activation_time, false, reason,
        &pending_team_trajectory_.solution);
  }

  bool EGOPlannerManager::setLocalTrajFromOpt(
      const poly_traj::MinJerkOpt &opt, const bool touch_goal,
      const double explicit_start_time, const traj_utils::TeamTrajectorySolution *team)
  {
    // Local rolling execution never waits for or leases authority to Joint.
    // A newer local commit invalidates any pending future-tail proposal below.
    poly_traj::Trajectory traj = opt.getTraj();
    const double activation=std::isfinite(explicit_start_time)?explicit_start_time:local_activation_time_;
    const bool current_state_commit=current_state_restart_active_;
    const int expired_predecessor=expired_predecessor_id_;
    // Feedback093: a Team transaction must NEVER block Local rolling.
    //
    // The previous lifecycle deferred (return false) an ordinary Local rolling
    // update while a Team tail was committed, which is exactly the coupling
    // that let a speculative Team future own the Local predecessor chain.
    // A committed Team tail is now purely *speculative*: it keeps its future
    // execution reservation, but the Local Planner always maintains its own
    // guaranteed successor.  The only thing still checked here is whether the
    // speculative tail is still hard-safe; if it is not, it is revoked as an
    // explicitly unsafe speculation and Local rolling continues regardless.
    /* =====================================================================
     * Feedback096 — Team improvement protection (quality overwrite only).
     *
     * A Team-refined future that was actually adopted carries a light
     * certificate for the visibility deficit it removed.  An ordinary
     * *quality* Local update must not immediately re-create that same deficit.
     *
     * This is deliberately NOT a new authority and NOT a state machine:
     *   - it reuses the existing coverage / restart / invalidation authority to
     *     decide whether the pending successor is safety-required;
     *   - a safety-required successor ALWAYS wins (visibility never blocks
     *     safety or liveness);
     *   - the certificate expires by itself when the contracted critical
     *     window closes, when the certified trajectory leaves the timeline, or
     *     when a safety override happens.
     * ===================================================================== */
    if (!team && team_improvement_certificate_.valid)
    {
      const double cert_now = ros::Time::now().toSec();
      const bool certificate_live =
          traj_.local_traj.traj_id ==
              team_improvement_certificate_.trajectory_id &&
          cert_now < team_improvement_certificate_.activation +
                         team_improvement_certificate_.critical_end;
      if (!certificate_live)
      {
        team_improvement_certificate_ = TeamImprovementCertificate();
      }
      else
      {
        std::string safety_reason;
        if (localSuccessorIsSafetyRequired(safety_reason))
        {
          ++local_safety_override_team_;
          ROS_WARN("[TEAM_IMPROVEMENT_OVERRIDE] drone=%d "
                   "team_solution_id=%lu trajectory_id=%d reason=%s "
                   "action=ALLOW_SAFETY_SUCCESSOR",
              pp_.drone_id,
              static_cast<unsigned long>(
                  team_improvement_certificate_.team_solution_id),
              team_improvement_certificate_.trajectory_id,
              safety_reason.c_str());
          team_improvement_certificate_ = TeamImprovementCertificate();
        }
        else if (activation < team_improvement_certificate_.activation +
                                  team_improvement_certificate_.critical_end -
                                  1.0e-6)
        {
          ++local_quality_overwrite_blocked_;
          ROS_INFO("[LOCAL_QUALITY_OVERWRITE_BLOCKED] drone=%d "
                   "team_solution_id=%lu certified_trajectory_id=%d "
                   "certified_window=[%.9f,%.9f] requested_activation=%.9f "
                   "action=KEEP_CURRENT_VALIDATED_TEAM_FUTURE",
              pp_.drone_id,
              static_cast<unsigned long>(
                  team_improvement_certificate_.team_solution_id),
              team_improvement_certificate_.trajectory_id,
              team_improvement_certificate_.activation +
                  team_improvement_certificate_.critical_begin,
              team_improvement_certificate_.activation +
                  team_improvement_certificate_.critical_end,
              activation);
          return false;
        }
      }
    }

    if (!team && hasCommittedPendingTeamTail())
    {
      std::string team_safety_reason;
      if (!revalidateCommittedPendingTeamTail(team_safety_reason))
      {
        ROS_ERROR("[team-commit-atomic] event=TEAM_COMMIT_REVOKE_REQUEST "
                  "drone=%d team_solution_id=%lu reason=%s",
            pp_.drone_id,
            static_cast<unsigned long>(pending_team_trajectory_.team_solution_id),
            team_safety_reason.c_str());
        publishTeamCommitRevoke("COMMITTED_TEAM_SAFETY_INVALIDATED");
        pending_team_trajectory_ = PendingTeamTrajectory();
      }
      else
      {
        ROS_INFO("[team-speculative] transaction_id=%lu trajectory_id=%d "
                 "state=COMMITTED_FUTURE local_guaranteed_successor_continues=1 "
                 "local_authoritative_predecessor_unchanged=1 "
                 "local_activation=%.9f",
            static_cast<unsigned long>(pending_team_trajectory_.team_solution_id),
            kTeamSpeculativeTrajectoryIdBase +
                static_cast<int>(pending_team_trajectory_.team_solution_id),
            activation);
      }
    }
    if (activation - ros::Time::now().toSec() < 0.015 ||
        !checkActiveHandoff(traj, activation, team ? "JOINT_TEAM_SOLUTION" : "LOCAL_SUCCESSOR"))
      return false;
    std::string safety_reason;
    if (!current_state_commit && !validateCommittedPrefixUntil(activation,safety_reason))
    {
      ROS_ERROR("[execution-safety-reject] drone=%d reason=PREDECESSOR_PREFIX_%s",pp_.drone_id,safety_reason.c_str());
      return false;
    }
    if(current_state_commit)
      ROS_INFO("[current-state-restart] event=EXPIRED_PREDECESSOR_PREFIX_NOT_USED drone=%d expired_predecessor_id=%d old_suffix_extended=0 old_prefix_certified=0 current_state_handoff_required=1",
          pp_.drone_id,expired_predecessor);
    if (!validateExecutionTrajectory(traj,activation,touch_goal,safety_reason,team))
    {
      ROS_ERROR("[execution-safety-reject] drone=%d reason=%s activation=%.9f",pp_.drone_id,safety_reason.c_str(),activation);
      return false;
    }
    Eigen::MatrixXd cps = opt.getInitConstraintPoints(getCpsNumPrePiece());
    PtsChk_t pts_to_check;
    bool ret = ploy_traj_opt_->computePointsToCheck(traj, ConstraintPoints::two_thirds_id(cps, touch_goal), pts_to_check, touch_goal);
    if (ret)
    {
      const double checked_until = hardCheckedUntil(traj, activation, touch_goal);
      if (!std::isfinite(checked_until) ||
          checked_until <= activation + 1.0e-6)
        return false;
      if (!team)
        invalidatePendingTopology("LOCAL_SUCCESSOR_SUPERSEDED_GENERATION");
      commit_time_ = ros::Time::now().toSec();
      lifecycle_last_validation_ = commit_time_;
      lifecycle_validation_expiry_ = activation + 0.15;
      lifecycle_invalidated_ = false;
      lifecycle_state_ = current_state_commit
          ? "CURRENT_STATE_RESTART_VALIDATED"
          : "CURRENTLY_VALID";
      execution_safe_until_=checked_until;
      validated_handoff_this_batch_=true;
      const double start_time = activation;
      // The predecessor of this successor is exactly the trajectory it was
      // validated against (the executor-confirmed authority), and that identity
      // is what downstream resolution must see.
      scheduled_predecessor_=team?traj_.local_traj:predecessorAuthority();
      declared_predecessor_id_=team?traj_.local_traj.traj_id
                                   :scheduled_predecessor_.traj_id;
      declared_predecessor_generation_=team?active_traj_generation_
                                           :predecessorAuthorityGeneration();
      traj_.setLocalTraj(traj, pts_to_check, start_time,
                         pp_.drone_id);
      traj_.local_traj.checked_until = checked_until;
      recordPredecessorRecord(traj_.local_traj,active_traj_generation_+1,
                              team?"TEAM_REALIZED_LOCAL":"LOCAL_SUCCESSOR",
                              team?team->team_solution_id:0,team!=nullptr);
      // Feedback096: a newer authoritative Local future is exactly what a Team
      // refinement must survive.  It only advances the latest-input snapshot;
      // it never cancels an in-flight refinement.
      ++latest_team_prediction_snapshot_id_;
      if(team)
      {
        // The executor assigns a Team realization an id from the disjoint Team
        // speculative namespace, derived from the shared team_solution_id.  The
        // Local Planner must file the SAME identity locally, otherwise the
        // executor-confirmed activation of that trajectory cannot be resolved
        // back to a polynomial and the authority switch is lost (observed as
        // EXECUTOR_CONFIRMED_ACTIVATION_BUT_LOCAL_COPY_MISSING, after which the
        // Local chain kept declaring the superseded predecessor).
        LocalTrajData team_copy=traj_.local_traj;
        team_copy.traj_id=kTeamSpeculativeTrajectoryIdBase+
            static_cast<int>(team->team_solution_id);
        recordPredecessorRecord(team_copy,active_traj_generation_+1,
                                "TEAM_REALIZED_LOCAL",team->team_solution_id,true);
        ROS_INFO("[team-speculative] transaction_id=%lu trajectory_id=%d "
                 "state=COMMITTED_FUTURE local_copy_filed=1 "
                 "local_authoritative_predecessor_unchanged=1",
            static_cast<unsigned long>(team->team_solution_id),team_copy.traj_id);
      }
      // setLocalTraj owns the trajectory-id increment.  Emit the handoff
      // identity only after that increment so predecessor and successor do
      // not appear to have the same ID in coverage accounting.
      ROS_INFO("[validated-moving-coverage] event=VALIDATED_COVERAGE_HANDOFF_SUCCESS drone=%d predecessor_id=%d successor_id=%d activation=%.9f predecessor_validated_end=%.9f successor_validated_end=%.9f",
          pp_.drone_id,scheduled_predecessor_.traj_id,
          traj_.local_traj.traj_id,activation,validated_coverage_end_,
          execution_safe_until_);
      ++active_traj_generation_;
      if (team)
        active_validation_revision_ = active_traj_generation_;
      active_reference_id_ = team ? team->team_solution_id : 0;
      active_reference_reason_ = team ? "JOINT_COMMIT" : "LOCAL_COMMIT";
      // Team planning is P/T only. Yaw remains the executor's ordinary
      // target-facing policy and never becomes a second trajectory authority.
      traj_.setLocalOptimizedYaw(false, {}, {});
      active_execution_safety_validated_=true;
      active_execution_touch_goal_=touch_goal;
      active_execution_source_="OTHER";
      active_execution_team_id_=0;
      active_execution_view_generation_=0;
      active_execution_hypothesis_=0;
      ROS_INFO("[planner-traj-commit] drone_id=%d trajectory_id=%d start_time=%.9f duration=%.6f piece_num=%d",
               pp_.drone_id, traj_.local_traj.traj_id, traj_.local_traj.start_time,
               traj_.local_traj.duration, traj_.local_traj.traj.getPieceNum());
      reportTeamRefinementAudit(false);
      ROS_INFO("[active-traj-lifecycle] event=COMMITTED traj_identity=start=%.9f,generation=%lu source=planner_commit start_time=%.9f duration=%.6f active=1 reason=commit_ok",
               traj_.local_traj.start_time, active_traj_generation_,
               traj_.local_traj.start_time, traj_.local_traj.duration);
      if(current_state_commit)
      {
        ROS_WARN("[current-state-restart] event=SUCCESS drone=%d expired_predecessor_id=%d new_trajectory_id=%d activation_time=%.9f successor_validated_end=%.9f full_hard_preflight=1",
            pp_.drone_id,expired_predecessor,
            traj_.local_traj.traj_id,activation,execution_safe_until_);
        ROS_INFO("[current-state-restart] event=EXIT drone=%d expired_predecessor_id=%d new_trajectory_id=%d reason=VALIDATED_MOVING_SUCCESSOR_COMMITTED",
            pp_.drone_id,expired_predecessor,traj_.local_traj.traj_id);
        current_state_restart_active_=false;
        current_state_valid_=false;
      }
    }
    else
    {
      ROS_WARN("[planner-traj-commit] drone_id=%d trajectory_id=%d commit=0 reason=POINTS_TO_CHECK_FAILED",
               pp_.drone_id, traj_.local_traj.traj_id);
    }

    return ret;
  }


  bool EGOPlannerManager::consumeStaleHeadReplanRequest(Eigen::Vector3d &odom_pos,
                                                        Eigen::Vector3d &odom_vel)
  {
    if (!stale_head_replan_requested_)
      return false;
    odom_pos = stale_head_odom_pos_;
    odom_vel = stale_head_odom_vel_;
    stale_head_replan_requested_ = false;
    return true;
  }

  bool EGOPlannerManager::topologyCoordinationPending() const
  {
    return pending_topology_.active;
  }

  bool EGOPlannerManager::backgroundTeamQualityAdmissible(
      const double now) const
  {
    if (!std::isfinite(now) || current_state_restart_active_ ||
        traj_.local_traj.traj_id <= 0 ||
        traj_.local_traj.traj.getPieceNum() <= 0)
      return false;
    const double remaining = std::min(
        execution_safe_until_,
        traj_.local_traj.start_time + traj_.local_traj.duration) - now;
    const double remaining_pipeline = std::max(
        uninterruptible_budget_, planningToCommitMedian());
    const double required = remaining_pipeline + team_solution_max_age_ +
                            local_activation_margin_;
    return remaining > required && remaining > topology_min_active_remaining_;
  }

  bool EGOPlannerManager::teamTrajectorySampleHoldActive(double now) const
  {
    return enable_team_visibility_optimizer_ &&
           team_execution_hold_solution_id_ > 0 &&
           team_execution_hold_traj_id_ == traj_.local_traj.traj_id &&
           std::isfinite(now) && now < team_execution_hold_until_;
  }

  void EGOPlannerManager::topologyResultCallback(
      const traj_utils::TopologyCoordinationConstPtr &message)
  {
    if (!enable_joint_topology_coordination_ || !message)
      return;
    if (message->drone_ids.size() != message->planning_generations.size() ||
        message->drone_ids.size() != message->selected_candidate_ids.size() ||
        message->drone_ids.size() != message->selected_candidate_kinds.size())
    {
      ROS_WARN("[topology-result-reject] drone=%d reason=MALFORMED_RESULT "
               "coordination_generation=%lu",
               pp_.drone_id,
               static_cast<unsigned long>(message->coordination_generation));
      return;
    }
    if (!message->valid)
    {
      for (size_t i=0; i<message->drone_ids.size(); ++i)
        if (message->drone_ids[i] == pp_.drone_id && pending_topology_.active &&
            message->planning_generations[i] == pending_topology_.planning_generation)
        {
          latest_topology_result_ = *message;
          latest_topology_result_valid_ = true;
          ROS_INFO("[team-transaction] event=TRANSACTION_JOINT_REJECTED "
                   "drone=%d transaction_id=%lu reason=%s "
                   "action=JOINT_DISCARD_LOCAL_UNCHANGED",
                   pp_.drone_id, pending_topology_.transaction_id,
                   message->selection_reason.c_str());
        }
      return;
    }
    latest_topology_result_ = *message;
    latest_topology_result_valid_ = true;
    ROS_INFO("[topology-result-recv] drone=%d coordination_generation=%lu "
             "snapshot_stamp=%.9f selection_reason=%s",
             pp_.drone_id,
             static_cast<unsigned long>(message->coordination_generation),
             message->snapshot_stamp.toSec(), message->selection_reason.c_str());
  }

  void EGOPlannerManager::publishTeamTrajectoryAck(
      const traj_utils::TeamTrajectorySolution &solution,
      const unsigned long planning_generation,
      const int source_candidate_id, const bool accepted,
      const std::string &reason)
  {
    if (!team_trajectory_ack_pub_)
      return;
    traj_utils::TeamTrajectoryAck ack;
    ack.phase = traj_utils::TeamTrajectoryAck::PHASE_REALIZATION;
    ack.team_solution_id = solution.team_solution_id;
    ack.team_reference_id = solution.team_solution_id;
    ack.coordination_generation = solution.coordination_generation;
    ack.drone_id = pp_.drone_id;
    ack.planning_generation = planning_generation;
    ack.source_candidate_id = source_candidate_id;
    ack.stamp = ros::Time::now();
    ack.accepted = accepted;
    ack.reason = reason;
    team_trajectory_ack_pub_.publish(ack);
    ROS_INFO("[team-solution-ack-send] drone=%d team_solution_id=%lu "
             "generation=%lu planning_generation=%lu source_candidate_id=%d "
             "accepted=%d reason=%s",
             pp_.drone_id,
             static_cast<unsigned long>(solution.team_solution_id),
             static_cast<unsigned long>(solution.coordination_generation),
             planning_generation, source_candidate_id,
             static_cast<int>(accepted), reason.c_str());
  }

  void EGOPlannerManager::publishTeamReferenceAck(
      const traj_utils::TeamReferenceSchedule &schedule,
      const unsigned long planning_generation,
      const int source_candidate_id, const int topology_kind,
      const bool accepted, const std::string &reason,
      const poly_traj::MinJerkOpt *realized,
      const double realization_latency_ms,
      const TeamReferenceSideCertificate *side_certificate)
  {
    if (!team_trajectory_ack_pub_)
      return;
    traj_utils::TeamTrajectoryAck ack;
    ack.phase = traj_utils::TeamTrajectoryAck::PHASE_REALIZATION;
    ack.team_solution_id = schedule.team_reference_id;
    ack.task_reference_generation = schedule.task_reference_generation;
    ack.team_reference_id = schedule.team_reference_id;
    ack.coordination_generation = schedule.coordination_generation;
    ack.drone_id = pp_.drone_id;
    ack.planning_generation = planning_generation;
    ack.source_candidate_id = source_candidate_id;
    ack.stamp = ros::Time::now();
    ack.accepted = accepted;
    ack.reason = reason;
    ack.target_prediction_revision = schedule.target_prediction_revision;
    ack.target_snapshot_epoch = schedule.target_snapshot_epoch;
    ack.dynamic_prediction_identity = schedule.dynamic_prediction_identity;
    ack.static_map_revision = schedule.static_map_revision;
    ack.visibility_model_version = schedule.visibility_model_version;
    ack.evaluation_start_world = schedule.evaluation_start_world;
    ack.activation_time = schedule.activation_time;
    ack.execution_horizon = schedule.execution_horizon;
    ack.topology_kind = topology_kind;
    if (pp_.drone_id >= 0 && pp_.drone_id < 3 &&
        schedule.frontier_owner_revisions.size() == 3 &&
        schedule.frontier_owner_trajectory_ids.size() == 3)
    {
      ack.frontier_owner_revision =
          schedule.frontier_owner_revisions[pp_.drone_id];
      ack.frontier_owner_trajectory_id =
          schedule.frontier_owner_trajectory_ids[pp_.drone_id];
    }
    ack.realization_valid = accepted && realized &&
                            realized->getTraj().getPieceNum() > 0;
    if (ack.realization_valid)
      trajectoryToMINCOMessage(realized->getTraj(), pp_.drone_id,
                               source_candidate_id,
                               schedule.activation_time.toSec(),
                               ack.realized_trajectory);
    ack.realization_latency_ms = realization_latency_ms;
    ack.side_contract_active = ack.realization_valid && side_certificate &&
                               side_certificate->active;
    if (ack.side_contract_active)
    {
      ack.side_contract_origin.x = side_certificate->origin.x();
      ack.side_contract_origin.y = side_certificate->origin.y();
      ack.side_contract_origin.z = side_certificate->origin.z();
      ack.side_contract_direction.x = side_certificate->direction.x();
      ack.side_contract_direction.y = side_certificate->direction.y();
      ack.side_contract_direction.z = side_certificate->direction.z();
      ack.side_contract_sign = side_certificate->sign;
      ack.side_contract_offset = side_certificate->offset;
      ack.side_contract_conflict_progress =
          side_certificate->conflict_progress;
      ack.side_contract_guidance_window =
          side_certificate->guidance_window;
    }
    team_trajectory_ack_pub_.publish(ack);
    ROS_INFO("[TEAM_REFERENCE_ACK] drone=%d reference_id=%lu topology=%d "
             "accepted=%d realization_valid=%d latency_ms=%.3f reason=%s",
        pp_.drone_id,
        static_cast<unsigned long>(schedule.team_reference_id), topology_kind,
        static_cast<int>(accepted), static_cast<int>(ack.realization_valid),
        realization_latency_ms, reason.c_str());
  }

  void EGOPlannerManager::publishTeamCommitRevoke(const std::string &reason)
  {
    if (!pending_team_trajectory_.accepted ||
        !pending_team_trajectory_.commit_received)
      return;
    traj_utils::TeamTrajectoryAck ack;
    ack.phase = traj_utils::TeamTrajectoryAck::PHASE_EXECUTION_REVOKE;
    ack.team_solution_id = pending_team_trajectory_.team_solution_id;
    ack.team_reference_id = pending_team_trajectory_.team_solution_id;
    ack.coordination_generation =
        pending_team_trajectory_.coordination_generation;
    ack.drone_id = pp_.drone_id;
    ack.planning_generation = pending_team_trajectory_.planning_generation;
    ack.source_candidate_id = pending_team_trajectory_.source_candidate_id;
    ack.stamp = ros::Time::now();
    ack.accepted = false;
    ack.reason = std::string("TEAM_COMMIT_REVOKE:") + reason;
    team_trajectory_ack_pub_.publish(ack);
    ROS_ERROR("[team-commit-atomic] event=TEAM_COMMIT_REVOKE_REQUEST "
              "drone=%d team_solution_id=%lu reason=%s",
        pp_.drone_id,
        static_cast<unsigned long>(pending_team_trajectory_.team_solution_id),
        reason.c_str());
  }

  void EGOPlannerManager::displayTrajectoryAuthority(
      const poly_traj::Trajectory &trajectory, const double start_time,
      const double validated_end, const int marker_id)
  {
    if (!visualization_ || trajectory.getPieceNum() <= 0) return;
    const double total = trajectory.getTotalDuration();
    if (!std::isfinite(total) || total <= 1.0e-6) return;
    const double valid_duration = std::max(0.0, std::min(
        total, std::isfinite(validated_end) ? validated_end - start_time : total));
    const auto sample = [&](double begin, double end) {
      std::vector<Eigen::Vector3d> points;
      const double duration = std::max(0.0, end - begin);
      const int count = std::max(2, static_cast<int>(std::ceil(duration / 0.05)) + 1);
      points.reserve(count);
      for (int i = 0; i < count; ++i)
        points.push_back(trajectory.getPos(begin + duration * i /
            static_cast<double>(count - 1)));
      return points;
    };
    visualization_->displayValidatedOptimalList(sample(0.0, valid_duration), marker_id);
    if (valid_duration + 1.0e-6 < total)
      visualization_->displayUnvalidatedFutureList(
          sample(valid_duration, total), marker_id);
    ROS_INFO("[rviz-authority] drone=%d trajectory_id=%d start=%.9f "
             "validated_end=%.9f total_end=%.9f validated_prefix_only=1",
        pp_.drone_id, traj_.local_traj.traj_id, start_time,
        start_time + valid_duration, start_time + total);
  }

  bool EGOPlannerManager::teamProposalBoundaryMatches(const poly_traj::Trajectory &proposed,
      const poly_traj::Trajectory &source_suffix,double activation,
      const bool require_source_tail) const
  {
    const auto &predecessor=activation>=traj_.local_traj.start_time?traj_.local_traj:scheduled_predecessor_;
    const double elapsed=activation-predecessor.start_time;
    if(predecessor.traj.getPieceNum()<=0 || elapsed<0 || elapsed>predecessor.duration ||
        proposed.getPieceNum()<=0 || source_suffix.getPieceNum()<=0)return false;
    const double duration=proposed.getTotalDuration(),source_duration=source_suffix.getTotalDuration();
    // Preserve the original exact boundary tolerance. The head belongs to the
    // actual/scheduled predecessor; candidate identity still fixes the tail.
    const bool head_matches =
        (proposed.getPos(0)-predecessor.traj.getPos(elapsed)).norm()<=1e-5 &&
        (proposed.getVel(0)-predecessor.traj.getVel(elapsed)).norm()<=1e-5 &&
        (proposed.getAcc(0)-predecessor.traj.getAcc(elapsed)).norm()<=1e-5;
    if (!head_matches || !require_source_tail)
      return head_matches;
    return
        (proposed.getPos(duration)-source_suffix.getPos(source_duration)).norm()<=1e-5 &&
        (proposed.getVel(duration)-source_suffix.getVel(source_duration)).norm()<=1e-5 &&
        (proposed.getAcc(duration)-source_suffix.getAcc(source_duration)).norm()<=1e-5;
  }

  void EGOPlannerManager::taskReferenceCallback(
      const traj_utils::CooperativeTaskReferenceConstPtr &message)
  {
    if (!message || !message->valid)
      return;
    if (message->generation < task_reference_generation_)
      return;  // 单调性：丢弃过期任务参考
    task_reference_generation_ = message->generation;
    ROS_INFO_THROTTLE(
        5.0, "[task-reference] consumer drone=%d generation=%lu phi0=%.6f",
        pp_.drone_id, static_cast<unsigned long>(message->generation),
        message->phi0);
  }

  void EGOPlannerManager::topologyEscalationCallback(
      const traj_utils::TopologyEscalationConstPtr &message)
  {
    if (!message || message->target_drone != pp_.drone_id)
      return;
    // 注意:Team escalation 不再闭合/替换 Local 自主事件(§7)。Local 事件
    // 只按自身闭合条件(recovered / margin 恢复 / expiry / revision stale /
    // M2 preempt)关闭。该 wire callback 仅供 legacy telemetry。
    // K3 escalation 遥测(legacy/diagnostic,第三轮 authority 清理):Team
    // escalation 不再是 K3 Local recovery 的授权源,也不在此创建事件。若
    // Local 已有自主事件且窗口/目标一致,仅做 identity merge 诊断计数。
    if (k3_local_escalation_enabled_ && message->repair_k == 3)
    {
      const double now_world = ros::Time::now().toSec();
      const double window_begin = message->window_begin_world.toSec();
      const double window_end = message->window_end_world.toSec();
      // 普通 authority horizon 同 reboundReplan 规则:滚动轨迹 ×2/3,再被
      // 动态预测 horizon 截断。
      const double normal_horizon =
          std::min(2.0 / 3.0 * std::max(0.0, traj_.local_traj.duration),
                   ploy_traj_opt_->getMovingObjPredictionHorizon());
      const bool outside_normal_authority =
          window_begin > now_world + normal_horizon;
      const char *motion = message->detect_limiter == 1 ? "STATIC"
          : message->detect_limiter == 2               ? "DYNAMIC"
                                                       : "UNKNOWN";
      if (k3_event_.active && k3_event_.local_origin)
      {
        ++k3_team_escalation_merged_count_;
        ROS_INFO("[K3_TEAM_ESCALATION_MERGED] event_id=%lu "
                 "team_escalation_id=%lu contract_id=%lu window=[%.3f,%.3f] "
                 "local_window=[%.3f,%.3f] merged_count=%d action=DIAGNOSTIC_ONLY",
            static_cast<unsigned long>(k3_event_.escalation_id),
            static_cast<unsigned long>(message->escalation_id),
            static_cast<unsigned long>(message->handoff_contract_id),
            window_begin, window_end,
            k3_event_.window_begin.toSec(), k3_event_.window_end.toSec(),
            k3_team_escalation_merged_count_);
      }
      ROS_WARN(
          "[K3_ESCALATION_EVENT] event_id=%lu target_uav=%d blocker=%s "
          "motion=%s forecast_time=%.9f time_to_cross=%.6f "
          "normal_visibility_authority_horizon=%.6f "
          "outside_normal_authority=%d window_begin=%.9f window_end=%.9f "
          "margin_at_detect=%.6f K2_at_detect=%.6f K3_at_detect=%.6f "
          "required_margin=%.6f contract_id=%lu revision=%lu "
          "authority=DIAGNOSTIC_ONLY",
          static_cast<unsigned long>(message->escalation_id),
          pp_.drone_id, "LIVE_WITNESS_MATCH", motion, window_begin,
          window_begin - now_world, normal_horizon,
          static_cast<int>(outside_normal_authority), window_begin,
          window_end, message->margin_at_detect, message->k2_at_detect,
          message->k3_at_detect, message->required_margin,
          static_cast<unsigned long>(message->handoff_contract_id),
          static_cast<unsigned long>(message->target_prediction_revision));
    }
    ROS_WARN("[topology-escalation] drone=%d escalation_id=%lu "
             "contract_id=%lu window=[%.3f, %.3f] reason=%s",
        pp_.drone_id,
        static_cast<unsigned long>(message->escalation_id),
        static_cast<unsigned long>(message->handoff_contract_id),
        message->window_begin_world.toSec(),
        message->window_end_world.toSec(), message->reason.c_str());
  }

  void EGOPlannerManager::closeK3EscalationEvent(const char *close_reason)
  {
    if (!k3_event_.active || k3_event_.result_emitted)
    {
      k3_event_ = K3EscalationEventState();
      return;
    }
    // planner 可观测的 failure_stage 优先级:恢复 > M2 抢占 > 授权/派发/进度链。
    const char *failure_stage = "RECOVERED";
    if (!k3_event_.recovered)
    {
      if (k3_event_.m2_preempted)
        failure_stage = "M2_PREEMPTED";
      else if (k3_event_.side_trigger_count <= 0)
        failure_stage = "NO_SIDE_DISPATCH";
      else if (k3_event_.side_both_failed_count > 0 &&
               k3_event_.progress_candidate_count <= 0)
        failure_stage = "SIDE_INFEASIBLE";
      else if (k3_event_.progress_candidate_count <= 0)
        failure_stage = "NO_K3_PROGRESS_CANDIDATE";
      else if (k3_event_.progress_selected_count <= 0)
        failure_stage = "COMPARATOR_REJECTED_K3_BETTER";
      else if (k3_event_.progress_committed_count <= 0)
        failure_stage = "COMMIT_NOT_REACHED";
      else if (k3_event_.pending_activation_check ||
               k3_event_.progress_activated_count <= 0)
        failure_stage = "ACTIVATION_NOT_REACHED";
      else
        failure_stage = "PROGRESS_BUT_NOT_RECOVERED";
    }
    ROS_WARN(
        "[K3_EVENT_RESULT] event_id=%lu outside_normal_authority=%d "
        "side_trigger_count=%d side_both_failed_count=%d "
        "k3_better_candidate_count=%d k3_better_selected_count=%d "
        "k3_better_committed_count=%d activated_progress_count=%d "
        "binary_k3_recovered=%d "
        "recovery_observed=%d recovery_observed_world_time=%.9f "
        "recovery_observed_traj_id=%d last_odom_stamp=%.9f "
        "time_to_recovery=%.6f failure_stage=%s close_reason=%s",
        static_cast<unsigned long>(k3_event_.escalation_id),
        static_cast<int>(k3_event_.outside_normal_authority),
        k3_event_.side_trigger_count, k3_event_.side_both_failed_count,
        k3_event_.progress_candidate_count,
        k3_event_.progress_selected_count,
        k3_event_.progress_committed_count,
        k3_event_.progress_activated_count,
        static_cast<int>(k3_event_.recovered),
        static_cast<int>(k3_event_.recovered),
        k3_event_.recovery_world_time,
        k3_event_.recovery_observed_traj_id,
        pp_.drone_id >= 0 && pp_.drone_id < 3
            ? visibility_odom_[pp_.drone_id].stamp : 0.0,
        k3_event_.recovered ? k3_event_.recovery_world_time -
                                  k3_event_.window_begin.toSec() : 0.0,
        failure_stage, close_reason);
    k3_event_ = K3EscalationEventState();
  }

  void EGOPlannerManager::checkK3EscalationRecovery()
  {
    if (!k3_event_.active || k3_event_.recovered)
      return;
    if (pp_.drone_id < 0 || pp_.drone_id >= 3)
      return;
    const double now_world = ros::Time::now().toSec();
    const VisibilityOdomState &executed = visibility_odom_[pp_.drone_id];
    if (!executed.valid || !executed.header_stamp_valid ||
        !std::isfinite(executed.stamp) ||
        !executed.position.allFinite() || !std::isfinite(executed.yaw))
      return;
    const double sample_world_time = executed.stamp;
    const double odom_age = now_world - sample_world_time;
    if (odom_age < -0.1 ||
        sample_world_time < k3_event_.window_begin.toSec() ||
        sample_world_time >= k3_event_.window_end.toSec())
      return;
    // A margin crossing can precede binary loss.  Visibility before that
    // predicted loss is not a recovery and must not close the event early.
    if (k3_event_.predicted_binary_loss_time > 0.0 &&
        sample_world_time + 1.0e-9 <
            k3_event_.predicted_binary_loss_time)
      return;
    // A queued odometry sample may be processed after a long planning
    // callback.  Its stamped time, rather than its age at this callback,
    // determines whether recovery happened inside this event's window.
    const bool m2_authority_active =
        (latest_handoff_contract_active_ && latest_handoff_repair_k_ == 2) ||
        (team_refinement_open_ && team_refinement_.repair_k == 2);
    if (m2_authority_active ||
        (k3_event_.target_prediction_revision != 0 &&
         geometry_target_source_identity_ != 0 &&
         geometry_target_source_identity_ !=
             k3_event_.target_prediction_revision))
      return;
    // Odometry carries no trajectory id.  Bind the observation to the
    // executor-confirmed trajectory, never to a future Local commit.
    if (executed_traj_id_ <= 0 || !authoritative_predecessor_valid_ ||
        authoritative_predecessor_.traj_id != executed_traj_id_ ||
        authoritative_predecessor_generation_ != executed_generation_ ||
        sample_world_time < authoritative_predecessor_.start_time ||
        sample_world_time >= authoritative_predecessor_.start_time +
                                 authoritative_predecessor_.duration)
      return;
    // Recovery is an executed-state observation, not a rescan of a planned
    // baseline with the newest yaw copied into earlier world times.  Position,
    // measured yaw, target propagation and moving-obstacle prediction all use
    // the odometry stamp as their common world-time sample.
    const Eigen::Vector3d target = localK3TargetAtWorldTime(
        geometry_target_p_, geometry_target_v_, geometry_target_stamp_,
        sample_world_time);
    const auto sample = evaluateLocalVisibilitySampleAtWorldTime(
        executed.position, executed.yaw, target, sample_world_time);
    if (sample.valid && sample.binary_visible)
    {
      k3_event_.recovered = true;
      k3_event_.recovery_world_time = sample_world_time;
      k3_event_.recovery_observed_traj_id = executed_traj_id_;
      ROS_INFO("[K3_RECOVERY_EVIDENCE] event_id=%lu uav=%d "
               "sample_world_time=%.9f window_begin=%.9f window_end=%.9f "
               "traj_id=%d binary_visible=1 latched=1 odom_age=%.6f",
          static_cast<unsigned long>(k3_event_.escalation_id), pp_.drone_id,
          sample_world_time, k3_event_.window_begin.toSec(),
          k3_event_.window_end.toSec(), executed_traj_id_, odom_age);
      ROS_INFO("[K3_BINARY_RECOVERY_OBSERVED] event_id=%lu world_time=%.9f "
               "source=EXECUTED_ODOMETRY target_epoch=%.9f "
               "dynamic_world_time=%.9f fov=1 range=1 static=1 dynamic=1",
          static_cast<unsigned long>(k3_event_.escalation_id),
          sample_world_time, geometry_target_stamp_, sample_world_time);
    }
  }

  void EGOPlannerManager::updateLocalK3RecoveryEvent()
  {
    if (!k3_local_escalation_enabled_ || pp_.drone_id < 0 ||
        pp_.drone_id >= 3)
      return;
    // ---- M2 Team 绝对优先级(§8):active 的 M2(repair_k==2)合同期间,
    // K3 事件闭合为诊断,不建立新事件、不产生 competing selection authority。
    const bool m2_authority_active =
        (latest_handoff_contract_active_ && latest_handoff_repair_k_ == 2) ||
        (team_refinement_open_ && team_refinement_.repair_k == 2);
    if (m2_authority_active)
    {
      if (k3_event_.active)
      {
        k3_event_.m2_preempted = true;
        closeK3EscalationEvent("M2_PREEMPTED");
      }
      return;
    }
    const double now_world = ros::Time::now().toSec();
    if (k3_event_.active)
    {
      // world/revision 身份失效(§7):target prediction revision 变化。
      if (k3_event_.target_prediction_revision != 0 &&
          geometry_target_source_identity_ != 0 &&
          geometry_target_source_identity_ !=
              k3_event_.target_prediction_revision)
      {
        closeK3EscalationEvent("TARGET_REVISION_STALE");
        return;
      }
    }
    confirmK3LocalActivation();
    checkK3EscalationRecovery();
    if (k3_event_.active)
    {
      if (k3_event_.recovered)
      {
        closeK3EscalationEvent("RECOVERY_OBSERVED");
        return;
      }
      // expiry(§7):窗口结束 + 1 sample cell 仍未恢复/刷新。
      if (now_world > k3_event_.window_end.toSec() + visibility_sample_dt_)
      {
        closeK3EscalationEvent("EVENT_EXPIRED");
        return;
      }
    }
    // ---- Local 自主检测扫描(§6):在已执行基线 horizon 上用统一原语找
    // margin 首次下穿 trigger;K2 healthy = 当前三机 binary 可见数 ≥ 2。
    const auto &baseline = traj_.local_traj;
    if (baseline.traj_id <= 0 || baseline.traj.getPieceNum() <= 0)
      return;
    const double scan_begin = std::max(now_world, baseline.start_time);
    const double scan_end = std::min(
        baseline.start_time +
            std::min(baseline.duration,
                     ploy_traj_opt_->getMovingObjPredictionHorizon()),
        k3_event_.active ? k3_event_.window_end.toSec() +
                               visibility_sample_dt_ : 1.0e18);
    if (scan_end <= scan_begin + 1.0e-6)
      return;
    const auto metrics = evaluateLocalK3WindowMetrics(
        baseline.traj, baseline.start_time, geometry_target_p_,
        geometry_target_v_, geometry_target_stamp_, scan_begin, scan_end,
        k3_margin_trigger_);
    ROS_INFO_THROTTLE(2.0,
        "[K3_DETECT_DEBUG] drone=%d metrics_valid=%d samples=%d margin_samples=%d "
        "c3=%.3f k2=%.3f d3=%.3f active_event=%d",
        pp_.drone_id, static_cast<int>(metrics.valid), metrics.sample_count,
        metrics.margin_sample_count, metrics.c3, metrics.k2, metrics.d3,
        static_cast<int>(k3_event_.active));
    if (!metrics.valid)
      return;
    // 逐样本找首个下穿与二值损失/恢复预测;diagnostic probe复用同一推进
    // 中的 yaw 状态,避免拿当前 odom yaw 去评价未来样本。
    const double dt = visibility_sample_dt_;
    const double probe_world = std::min(
        scan_begin + 0.5 * std::max(0.1, scan_end - scan_begin), scan_end);
    const auto &self_odom = visibility_odom_[pp_.drone_id];
    const double yaw_scan_begin = self_odom.valid
        ? std::max(scan_begin, self_odom.stamp) : scan_begin;
    const double first = std::ceil(yaw_scan_begin / dt) * dt;
    multi_uav_formation::PredictedYawState yaw_state;
    if (self_odom.valid && self_odom.stamp <= first + 1.0e-9)
    {
      yaw_state.valid = true;
      yaw_state.yaw = self_odom.yaw;
      yaw_state.yaw_rate = self_odom.yaw_rate;
    }
    double last_update = self_odom.stamp;
    double crossing_time = 0.0;
    double crossing_margin = 0.0;
    int crossing_limiter = 0;
    double binary_loss_time = -1.0;
    double binary_recover_time = -1.0;
    bool crossing_found = false;
    bool sample_breakdown_logged = false;
    for (double world_time = first; world_time <= scan_end + 1.0e-9;
         world_time += dt)
    {
      const double self_t = world_time - baseline.start_time;
      if (self_t < -1.0e-9 || self_t > baseline.traj.getTotalDuration() + 1.0e-9)
        continue;
      const Eigen::Vector3d observer = baseline.traj.getPos(std::max(0.0, self_t));
      const Eigen::Vector3d target = localK3TargetAtWorldTime(
          geometry_target_p_, geometry_target_v_, geometry_target_stamp_,
          world_time);
      if (!yaw_state.valid || world_time < last_update - 1.0e-9)
        continue;
      if (yaw_state.valid &&
          (target.head<2>() - observer.head<2>()).squaredNorm() > 1.0e-12)
      {
        yaw_state = multi_uav_formation::advanceTargetFacingYaw(
            yaw_state, std::atan2((target - observer).y(),
                                  (target - observer).x()),
            std::max(0.0, world_time - last_update));
        last_update = world_time;
      }
      const auto sample = evaluateLocalVisibilitySampleAtWorldTime(
          observer, yaw_state.valid ? yaw_state.yaw : 0.0, target, world_time);
      if (!sample.valid || !sample.continuous.margin_valid)
        continue;
      if (!sample_breakdown_logged && world_time + 1.0e-9 >= probe_world)
      {
        sample_breakdown_logged = true;
        ROS_INFO_THROTTLE(2.0,
            "[K3_SAMPLE_BREAKDOWN] drone=%d world_time=%.9f "
            "self_odom_valid=%d valid=%d range=%d static=%d dynamic=%d "
            "fov=%d binary=%d margin=%.3f limiter=%s r_static=%.3f "
            "r_dyn=%.3f r_fov=%.3f r_range=%.3f dist=%.3f",
            pp_.drone_id, world_time, static_cast<int>(self_odom.valid),
            static_cast<int>(sample.valid),
            static_cast<int>(sample.range_visible),
            static_cast<int>(sample.static_los_visible),
            static_cast<int>(sample.dynamic_los_visible),
            static_cast<int>(sample.fov_visible),
            static_cast<int>(sample.binary_visible), sample.continuous.margin,
            multi_uav_formation::visibilityMarginLimiterName(
                sample.continuous.limiter),
            sample.continuous.static_risk,
            sample.continuous.dynamic_los_risk,
            sample.continuous.fov_risk, sample.continuous.range_risk,
            (target.head<2>() - observer.head<2>()).norm());
      }
      if (!crossing_found && sample.continuous.margin < k3_margin_trigger_)
      {
        crossing_found = true;
        crossing_time = world_time;
        crossing_margin = sample.continuous.margin;
        crossing_limiter = static_cast<int>(sample.continuous.limiter);
      }
      if (crossing_found)
      {
        if (binary_loss_time < 0.0 && !sample.binary_visible)
          binary_loss_time = world_time;
        else if (binary_loss_time >= 0.0 && binary_recover_time < 0.0 &&
                 sample.binary_visible)
          binary_recover_time = world_time;
      }
    }
    // K2/K3 占比来自同一批样本的三机统一 binary(evaluateLocalK3WindowMetrics)。
    // 当前 K2 healthy:扫描窗口内三机 binary ≥2 的样本占比(与检测同源)。
    const double k2_fraction = metrics.k2;
    const double k3_fraction = metrics.c3;
    const Eigen::Vector3d target_now = localK3TargetAtWorldTime(
        geometry_target_p_, geometry_target_v_, geometry_target_stamp_,
        now_world);
    const bool peers_healthy_now = [&]() {
      int count = 0;
      for (int u = 0; u < 3; ++u)
      {
        if (u == pp_.drone_id)
          continue;
        // peer 当前状态:优先 committed swarm 轨迹(权威),odom 兜底必须
        // valid;不可用即计为不可见,不得用零向量噪声参与判定。
        Eigen::Vector3d peer = Eigen::Vector3d::Zero();
        bool peer_valid = false;
        if (u < static_cast<int>(traj_.swarm_traj.size()))
        {
          const LocalTrajData &other = traj_.swarm_traj[u].executionAt(now_world);
          const double other_t = now_world - other.start_time;
          if (other.drone_id == u && other.traj.getPieceNum() > 0 &&
              other_t >= 0.0 && other_t <= other.duration)
          {
            peer = other.traj.getPos(other_t);
            peer_valid = peer.allFinite();
          }
        }
        if (!peer_valid && visibility_odom_[u].valid)
        {
          peer = visibility_odom_[u].position +
                 visibility_odom_[u].velocity *
                     std::max(0.0, now_world - visibility_odom_[u].stamp);
          peer_valid = peer.allFinite();
        }
        if (!peer_valid)
          continue;
        const auto &peer_odom = visibility_odom_[u];
        const double yaw_age = now_world - peer_odom.stamp;
        bool yaw_known = peer_odom.valid && yaw_age >= -1.0e-6 &&
                         yaw_age <= 1.0;
        multi_uav_formation::PredictedYawState peer_yaw;
        if (yaw_known)
        {
          peer_yaw.valid = true;
          peer_yaw.yaw = peer_odom.yaw;
          peer_yaw.yaw_rate = peer_odom.yaw_rate;
          const Eigen::Vector2d to_target =
              target_now.head<2>() - peer.head<2>();
          if (to_target.squaredNorm() > 1.0e-12)
          {
            peer_yaw = multi_uav_formation::advanceTargetFacingYaw(
                peer_yaw, std::atan2(to_target.y(), to_target.x()),
                std::max(0.0, yaw_age));
          }
        }
        const auto s = evaluateLocalVisibilitySampleAtWorldTime(
            peer, yaw_known ? peer_yaw.yaw : 0.0, target_now, now_world);
        count += s.valid && yaw_known && s.binary_visible ? 1 : 0;
      }
      return count >= 2;
    }();
    if (!crossing_found)
    {
      // margin 已回到 trigger 之上(§7):margin 恢复并离开 risk band。
      if (k3_event_.active)
        closeK3EscalationEvent("MARGIN_RECOVERED");
      return;
    }
    ROS_INFO_THROTTLE(2.0,
        "[K3_DETECT_DEBUG] drone=%d crossing_found=1 cross_t=%.3f margin=%.3f "
        "limiter=%d limiter_name=%s peers_healthy=%d loss_t=%.3f recover_t=%.3f",
        pp_.drone_id, crossing_time, crossing_margin, crossing_limiter,
        multi_uav_formation::visibilityMarginLimiterName(
            static_cast<multi_uav_formation::ContinuousVisibilitySample::Limiter>(
                crossing_limiter)),
        static_cast<int>(peers_healthy_now), binary_loss_time,
        binary_recover_time);
    if (!peers_healthy_now)
      return;  // 三机非 K2 healthy:不属于 K3-only event,交回原 authority。
    const double cell = visibility_sample_dt_;
    const double window_end =
        (binary_recover_time > 0.0 ? binary_recover_time : scan_end) + cell;
    if (k3_event_.active)
    {
      // 刷新(每 cycle 重预测):窗口只延不缩,避免同一事件抖动。
      k3_event_.window_end = ros::Time(std::max(window_end,
          k3_event_.window_end.toSec()));
      k3_event_.predicted_binary_loss_time =
          binary_loss_time > 0.0 ? binary_loss_time
                                 : k3_event_.predicted_binary_loss_time;
      k3_event_.predicted_recover_time =
          binary_recover_time > 0.0 ? binary_recover_time
                                    : k3_event_.predicted_recover_time;
      k3_event_.received_wall = ros::WallTime::now();
      k3_event_.k2_at_detect = k2_fraction;
      return;
    }
    // ---- 新事件(bounded rolling context,非 FSM):margin crossing 即授权起点。
    k3_event_ = K3EscalationEventState();
    k3_event_.active = true;
    k3_event_.local_origin = true;
    k3_event_.escalation_id = ++k3_local_event_serial_;
    k3_event_.window_begin = ros::Time(crossing_time);
    k3_event_.window_end = ros::Time(window_end);
    k3_event_.received_wall = ros::WallTime::now();
    k3_event_.required_margin = k3_margin_trigger_;
    k3_event_.detect_limiter = static_cast<std::uint8_t>(
        std::max(0, crossing_limiter));
    k3_event_.margin_at_detect = crossing_margin;
    k3_event_.k2_at_detect = k2_fraction;
    k3_event_.k3_at_detect = k3_fraction;
    k3_event_.target_prediction_revision = geometry_target_source_identity_;
    k3_event_.outside_normal_authority = crossing_time > now_world +
        std::min(2.0 / 3.0 * std::max(0.0, baseline.duration),
                 ploy_traj_opt_->getMovingObjPredictionHorizon());
    k3_event_.predicted_binary_loss_time = binary_loss_time;
    k3_event_.predicted_recover_time = binary_recover_time;
    ROS_WARN(
        "[K3_LOCAL_EVENT_CREATED] event_id=%lu target_uav=%d trigger=MARGIN_CROSS "
        "margin_cross=%.9f lead_to_binary_loss=%.6f "
        "predicted_binary_loss=%.9f predicted_recover=%.9f window=[%.9f,%.9f] "
        "limiter=%d limiter_name=%s margin_at_detect=%.6f K2_healthy=%d K2_fraction=%.3f "
        "revision=%lu baseline_traj=%d",
        static_cast<unsigned long>(k3_event_.escalation_id), pp_.drone_id,
        crossing_time,
        binary_loss_time > 0.0 ? binary_loss_time - crossing_time : -1.0,
        binary_loss_time, binary_recover_time, crossing_time, window_end,
        crossing_limiter,
        multi_uav_formation::visibilityMarginLimiterName(
            static_cast<multi_uav_formation::ContinuousVisibilitySample::Limiter>(
                crossing_limiter)),
        crossing_margin, static_cast<int>(peers_healthy_now),
        k2_fraction,
        static_cast<unsigned long>(k3_event_.target_prediction_revision),
        baseline.traj_id);
    // [K3_WORLD_TIME_AUDIT](§5,每事件一条):candidate_start/planning epoch/
    // target epoch 与 anchor 误差量级;old/new 口径分歧只在此审计,不刷屏。
    ROS_INFO(
        "[K3_WORLD_TIME_AUDIT] event_id=%lu baseline_start=%.9f "
        "target_epoch=%.9f anchor_lead=%.6f target_anchor_error_max=%.6f "
        "world_time_semantics=UNIFIED_LOCAL_VISIBILITY_SAMPLE",
        static_cast<unsigned long>(k3_event_.escalation_id),
        baseline.start_time, geometry_target_stamp_,
        baseline.start_time - geometry_target_stamp_,
        (geometry_target_v_ * std::max(0.0, baseline.start_time -
                                       geometry_target_stamp_)).norm());
  }

  void EGOPlannerManager::confirmK3LocalActivation()
  {
    if (!k3_event_.active || !k3_event_.pending_activation_check)
      return;
    const auto &active = traj_.local_traj;
    if (active.traj_id != k3_event_.pending_traj_id)
    {
      ROS_WARN("[K3_LOCAL_PROGRESS_CONFIRMED] event_id=%lu trajectory_id=%d "
               "activated=0 reason=SUPERSEDED",
               static_cast<unsigned long>(k3_event_.escalation_id),
               k3_event_.pending_traj_id);
      k3_event_.pending_activation_check = false;
      k3_event_.activated_progress = false;
      return;
    }
    if (ros::Time::now().toSec() + 1.0e-9 < active.start_time)
      return;
    k3_event_.pending_activation_check = false;
    k3_event_.activated_k2 = k3_event_.pending_k2;
    k3_event_.activated_c3_before = k3_event_.pending_c3_before;
    k3_event_.activated_c3_after = k3_event_.pending_c3;
    k3_event_.activated_d3_before = k3_event_.pending_d3_before;
    k3_event_.activated_d3_after = k3_event_.pending_d3;
    k3_event_.activated_progress =
        k3_event_.activated_k2 + 1.0e-9 >= k3_event_.k2_at_detect &&
        (k3_event_.activated_c3_after > k3_event_.activated_c3_before + 1.0e-9 ||
         k3_event_.activated_d3_after < k3_event_.activated_d3_before - 1.0e-9);
    if (k3_event_.activated_progress)
      ++k3_event_.progress_activated_count;
    // Feedback117 (§10): SIDE_ACTIVATED stage — the confirmed-active
    // trajectory was committed from a SIDE topology.  The K3 activation
    // confirmation is the production witness that a committed polynomial
    // actually became the executed one.
    if (k3_event_.last_committed_side ==
            static_cast<int>(CandidateKind::SIDE_PLUS) ||
        k3_event_.last_committed_side ==
            static_cast<int>(CandidateKind::SIDE_MINUS))
      ++side_activated_count_;
    ROS_WARN("[K3_LOCAL_PROGRESS_CONFIRMED] event_id=%lu trajectory_id=%d "
             "activated=1 progress=%d C3_before=%.6f C3_after=%.6f "
             "D3_before=%.6f D3_after=%.6f K2=%.6f",
             static_cast<unsigned long>(k3_event_.escalation_id),
             active.traj_id, static_cast<int>(k3_event_.activated_progress),
             k3_event_.activated_c3_before, k3_event_.activated_c3_after,
             k3_event_.activated_d3_before, k3_event_.activated_d3_after,
             k3_event_.activated_k2);
  }

  void EGOPlannerManager::teamReferenceScheduleCallback(
      const traj_utils::TeamReferenceScheduleConstPtr &message)
  {
    if (!enable_team_visibility_optimizer_ || !message)
      return;
    // Feedback096: remember the latest relay-contract state.  This is the
    // authority the adoption re-check uses to decide whether a refinement
    // result is still needed; it is deliberately not a state machine.
    latest_handoff_contract_id_ = message->handoff_contract_id;
    latest_handoff_contract_active_ = message->handoff_contract_active;
    latest_handoff_repair_k_ = message->handoff_repair_k;
    latest_handoff_priority_uav_ = message->handoff_repair_priority_uav;
    latest_handoff_world_revision_ = message->handoff_world_revision;
    latest_handoff_contract_seen_ = true;
    ++latest_team_prediction_snapshot_id_;
    // K3 Local autonomy(第三轮 authority 清理):repair_k==3 的 Team 提案不再
    // 被 Local 接受/realize/执行 SCP。K3 recovery 的开始与执行完全归 Local;
    // coordinator 侧的 K3 检测/合同保留为遥测。M2(repair_k==2)路径零改动。
    if (k3_local_escalation_enabled_ && message->handoff_repair_k == 3)
    {
      ROS_INFO("[K3_TEAM_PATH_RETIRED] drone=%d contract_id=%lu "
               "reason=LOCAL_AUTONOMOUS_K3_RECOVERY",
          pp_.drone_id,
          static_cast<unsigned long>(message->handoff_contract_id));
      return;
    }
    // ---- Feedback101 short-horizon Team visibility reserve ----------------
    // The reserve is a soft Local cost that exists ONLY while the Coordinator
    // has predicted an M2 crossing, and only for the declared repair
    // candidates.  It is refreshed from the newest schedule on every rolling
    // step (latest-wins), so no persistent contract state machine is added.
    // With no predicted crossing the term is exactly zero.
    {
      bool reserve_for_me = false;
      for (size_t slot = 0;
           slot < message->handoff_repair_candidates.size(); ++slot)
      {
        if (static_cast<int>(message->handoff_repair_candidates[slot]) ==
            pp_.drone_id)
          reserve_for_me = true;
      }
      // One to three guide samples on the first crossing interval. A
      // single-sample crossing is valid and produces one guide.
      const double reserve_begin = message->handoff_t_cross.toSec();
      const double reserve_end = message->handoff_t_recover.toSec();
      if (message->handoff_contract_active && reserve_for_me &&
          team_reserve_weight_ > 0.0 && std::isfinite(reserve_begin) &&
          std::isfinite(reserve_end) &&
          reserve_end >= reserve_begin)
      {
        PolyTrajOptimizer::TeamVisibilityReserve reserve;
        reserve.active = true;
        reserve.contract_id = message->handoff_contract_id;
        reserve.reserve_begin_world_time = reserve_begin;
        reserve.reserve_end_world_time = reserve_end;
        // m_soft reuses the contracted requirement; no second threshold and no
        // new safety constant is introduced.
        reserve.soft_margin = message->handoff_required_visibility_margin;
        reserve.weight = team_reserve_weight_;
        reserve.activation_world_time = traj_.local_traj.start_time;
        ploy_traj_opt_->setTeamVisibilityReserve(reserve);
        ++team_reserve_activation_count_;
        ROS_INFO("[TEAM_RESERVE_ACTIVE] drone=%d contract_id=%lu "
                 "reserve_window=[%.9f,%.9f] m_soft=%.6f weight=%.3f "
                 "M2_LIMITING_UAV=%d TEAM_REPAIR_CANDIDATES=[%d,%d] "
                 "reserve_activations=%lu",
            pp_.drone_id,
            static_cast<unsigned long>(message->handoff_contract_id),
            reserve_begin, reserve_end, reserve.soft_margin, reserve.weight,
            static_cast<int>(message->handoff_limiting_uav),
            message->handoff_repair_candidates.size() > 0
                ? static_cast<int>(message->handoff_repair_candidates[0]) : -1,
            message->handoff_repair_candidates.size() > 1
                ? static_cast<int>(message->handoff_repair_candidates[1]) : -1,
            static_cast<unsigned long>(team_reserve_activation_count_));
      }
      else if (ploy_traj_opt_->teamVisibilityReserveActive())
      {
        ploy_traj_opt_->clearTeamVisibilityReserve();
        ++team_reserve_clear_count_;
        ROS_INFO("[TEAM_RESERVE_CLEARED] drone=%d contract_id=%lu "
                 "active=%d reserve_for_me=%d clears=%lu",
            pp_.drone_id,
            static_cast<unsigned long>(message->handoff_contract_id),
            static_cast<int>(message->handoff_contract_active),
            static_cast<int>(reserve_for_me),
            static_cast<unsigned long>(team_reserve_clear_count_));
      }
    }
    if (message->state == traj_utils::TeamReferenceSchedule::STATE_ABORT)
    {
      if (pending_team_trajectory_.target_centered_reference &&
          pending_team_trajectory_.team_solution_id ==
              message->team_reference_id)
      {
        ROS_INFO("[TEAM_REFERENCE_NOOP] drone=%d reference_id=%lu "
                 "reason=%s local_commit_changed=0",
            pp_.drone_id,
            static_cast<unsigned long>(message->team_reference_id),
            message->reason.c_str());
        pending_team_trajectory_ = PendingTeamTrajectory();
      }
      return;
    }
    if (message->state != traj_utils::TeamReferenceSchedule::STATE_PROPOSAL)
      return;
    // 阶段 B：identity 链 —— 过期任务参考上的 proposal 直接拒绝。
    if (message->task_reference_generation != 0 &&
        task_reference_generation_ != 0 &&
        message->task_reference_generation < task_reference_generation_)
    {
      ROS_WARN("[TEAM_REFERENCE_REJECT] drone=%d reference_id=%lu "
               "reason=STALE_TASK_REFERENCE proposal_generation=%lu "
               "local_generation=%lu",
          pp_.drone_id,
          static_cast<unsigned long>(message->team_reference_id),
          static_cast<unsigned long>(message->task_reference_generation),
          static_cast<unsigned long>(task_reference_generation_));
      publishTeamReferenceAck(*message, 0, -1, -1, false,
                              "STALE_TASK_REFERENCE");
      return;
    }
    const size_t team_size = message->drone_ids.size();
    int own_index = -1;
    for (size_t index = 0; index < team_size; ++index)
      if (message->drone_ids[index] == pp_.drone_id)
        own_index = static_cast<int>(index);
    if (own_index < 0)
      return;
    const unsigned long planning_generation =
        message->planning_generations.size() == team_size
            ? message->planning_generations[own_index] : 0;
    const int source_candidate_id =
        message->source_candidate_ids.size() == team_size
            ? message->source_candidate_ids[own_index] : -1;
    const int topology_kind =
        message->topology_kinds.size() == team_size
            ? message->topology_kinds[own_index] : -1;
    const auto reject = [&](const std::string &reason,
                            const double latency_ms = 0.0) {
      publishTeamReferenceAck(*message, planning_generation,
          source_candidate_id, topology_kind, false, reason, nullptr,
          latency_ms);
      ROS_INFO("[TEAM_REFERENCE_REALIZATION_FAILED] drone=%d "
               "reference_id=%lu topology=%d reason=%s "
               "local_commit_changed=0",
          pp_.drone_id,
          static_cast<unsigned long>(message->team_reference_id),
          topology_kind, reason.c_str());
    };

    const size_t knot_count = message->knots_per_member;
    const size_t expected_knots = team_size * knot_count;
    if (!message->valid || message->team_reference_id == 0 || team_size != 3 ||
        (message->handoff_contract_active &&
         message->handoff_repair_k != 2 && message->handoff_repair_k != 3) ||
        knot_count < 2 || message->planning_generations.size() != team_size ||
        message->source_candidate_ids.size() != team_size ||
        message->topology_kinds.size() != team_size ||
        message->frontier_owner_revisions.size() != team_size ||
        message->frontier_owner_trajectory_ids.size() != team_size ||
        message->time_shifts.size() != team_size ||
        message->knot_world_times.size() != expected_knots ||
        message->knot_radii.size() != expected_knots ||
        message->knot_bearings.size() != expected_knots ||
        message->knot_heights.size() != expected_knots)
      return reject("MALFORMED_TEAM_REFERENCE_SCHEDULE");
    const double now = ros::Time::now().toSec();
    const double age = now - message->snapshot_stamp.toSec();
    const double activation = message->activation_time.toSec();
    if (!pending_topology_.active)
      return reject("NO_PENDING_TOPOLOGY_GENERATION");
    if (!trajectory_lifecycle::currentGeneration(
            planning_generation, pending_topology_.planning_generation,
            pending_topology_.active_generation, active_traj_generation_))
      return reject("STALE_PLANNING_GENERATION");
    if (message->frontier_owner_revisions[own_index] !=
            active_traj_generation_ ||
        message->frontier_owner_trajectory_ids[own_index] !=
            traj_.local_traj.traj_id)
      return reject("TEAM_REFERENCE_CAS_STALE");
    if (message->target_prediction_revision !=
            pending_topology_.target_snapshot_identity ||
        message->static_map_revision != pending_topology_.static_map_revision ||
        message->visibility_model_version !=
            pending_topology_.visibility_model_version ||
        message->dynamic_prediction_identity !=
            pending_topology_.dynamic_prediction_identity)
      return reject("TEAM_REFERENCE_CONTEXT_IDENTITY_MISMATCH");
    if (!std::isfinite(age) || age < -0.1 || age > team_solution_max_age_ ||
        !std::isfinite(activation) || activation - now <
            team_solution_min_activation_lead_ ||
        !std::isfinite(message->execution_horizon) ||
        message->execution_horizon < 0.20 ||
        message->dynamic_prediction_valid_from.toSec() > activation + 1.0e-9 ||
        message->dynamic_prediction_valid_to.toSec() + 1.0e-9 <
            activation + message->prediction_horizon)
      return reject("TEAM_REFERENCE_TIME_CONTRACT_INVALID");
    if (message->handoff_contract_active)
    {
      const bool roles_valid = message->handoff_contract_id != 0 &&
          message->handoff_world_revision ==
              message->target_prediction_revision &&
          message->handoff_outgoing_uav >= 0 &&
          message->handoff_outgoing_uav < 3 &&
          message->handoff_incoming_uav >= 0 &&
          message->handoff_incoming_uav < 3 &&
          message->handoff_stable_observer_uav >= 0 &&
          message->handoff_stable_observer_uav < 3 &&
          message->handoff_outgoing_uav != message->handoff_incoming_uav &&
          message->handoff_outgoing_uav !=
              message->handoff_stable_observer_uav &&
          message->handoff_incoming_uav !=
              message->handoff_stable_observer_uav;
      const bool timing_valid =
          message->handoff_acquire_by_world_time.toSec() >=
              activation - 1.0e-9 &&
          message->handoff_preserve_until_world_time.toSec() >=
              message->handoff_acquire_by_world_time.toSec() +
                  message->handoff_required_overlap - 1.0e-9 &&
          message->handoff_preserve_until_world_time.toSec() <=
              activation + message->execution_horizon + 1.0e-9 &&
          message->handoff_contract_expire_time.toSec() > now &&
          message->handoff_required_visibility_margin >= -1.0 &&
          message->handoff_required_visibility_margin <= 1.0;
      if (!roles_valid || !timing_valid)
        return reject("HANDOFF_CONTRACT_IDENTITY_OR_TIME_INVALID");
      if (message->handoff_repair_k == 3 &&
          (message->handoff_repair_candidates.size() != 1 ||
           message->handoff_repair_candidates[0] !=
               message->handoff_repair_priority_uav))
        return reject("K3_REPAIR_TARGET_INVALID");
    }

    int source_index = -1;
    for (size_t index = 0; index < pending_topology_.candidates.size(); ++index)
    {
      const CandidateResult &candidate = pending_topology_.candidates[index];
      if (candidate.candidate_id == source_candidate_id &&
          static_cast<int>(candidate.kind) == topology_kind &&
          candidate.encirclement_hypothesis_id == message->hypothesis_id &&
          candidate.encirclement_generation ==
              message->encirclement_generation)
      {
        source_index = static_cast<int>(index);
        break;
      }
    }
    if (source_index < 0)
      return reject("TEAM_REFERENCE_TOPOLOGY_IDENTITY_MISMATCH");
    const CandidateResult &source = pending_topology_.candidates[source_index];
    if (!source.joint_seed_valid || !source.joint_seed_static_constructible ||
        !source.joint_seed_local_sfc_valid ||
        source.joint_seed.getTraj().getPieceNum() <= 0)
      return reject("BOUND_LOCAL_TOPOLOGY_NOT_CONSTRUCTIBLE");

    const auto wall_start = std::chrono::steady_clock::now();
    const auto &predecessor = activation >= traj_.local_traj.start_time
        ? traj_.local_traj : scheduled_predecessor_;
    const double predecessor_time = activation - predecessor.start_time;
    if (predecessor.traj.getPieceNum() <= 0 || predecessor_time < 0.0 ||
        predecessor_time > predecessor.duration)
      return reject("LOCAL_PREDECESSOR_DOES_NOT_COVER_ACTIVATION");
    const Eigen::Vector3d head_p = predecessor.traj.getPos(predecessor_time);
    const Eigen::Vector3d head_v = predecessor.traj.getVel(predecessor_time);
    const Eigen::Vector3d head_a = predecessor.traj.getAcc(predecessor_time);

    // Team realization and the extra Team-SCP share one immutable world-time
    // target/dynamic/yaw context.  Do not inherit whichever Local batch last
    // touched the optimizer.
    const Eigen::Vector3d team_target_at_activation =
        targetPositionAt(activation);
    ploy_traj_opt_->setObject(team_target_at_activation,
                              targetVelocityAt(activation),
                              Eigen::Quaterniond::Identity());
    ploy_traj_opt_->setDirectionalVisibilityGuidance(true);
    multi_uav_formation::PredictedYawState team_yaw;
    if (pp_.drone_id >= 0 && pp_.drone_id < 3 &&
        visibility_odom_[pp_.drone_id].valid)
    {
      team_yaw.valid = true;
      team_yaw.yaw = visibility_odom_[pp_.drone_id].yaw;
      team_yaw.yaw_rate = visibility_odom_[pp_.drone_id].yaw_rate;
      const Eigen::Vector3d target_delta = team_target_at_activation - head_p;
      if (target_delta.head<2>().squaredNorm() > 1.0e-12)
        team_yaw = multi_uav_formation::advanceTargetFacingYaw(
            team_yaw, std::atan2(target_delta.y(), target_delta.x()),
            std::max(1.0e-6, activation -
                visibility_odom_[pp_.drone_id].stamp));
    }
    ploy_traj_opt_->setVisibilityYawState(
        team_yaw.valid, team_yaw.valid ? team_yaw.yaw : 0.0,
        team_yaw.valid ? team_yaw.yaw_rate : 0.0,
        tracking_camera_.horizontal_fov, tracking_camera_.vertical_fov,
        tracking_camera_.min_range, tracking_camera_.max_range);
    MovingObjPredictionEpochScope team_prediction_epoch_scope(
        ploy_traj_opt_.get(), activation, static_cast<bool>(obj_predictor_));

    const size_t base = static_cast<size_t>(own_index) * knot_count;
    // Historical wire name: this is a reference phase shift, not a MINCO
    // duration decision.
    const double reference_phase_shift = message->handoff_contract_active
        ? 0.0 : message->time_shifts[own_index];
    if (message->handoff_contract_active)
      ROS_INFO("[TEAM_REFERENCE_PHASE_USED_FOR_HANDOFF] drone=%d "
               "contract_id=%lu value=0",
               pp_.drone_id,
               static_cast<unsigned long>(message->handoff_contract_id));
    const bool k3_peer = message->handoff_repair_k == 3 &&
        message->handoff_repair_priority_uav != pp_.drone_id;
    const poly_traj::Trajectory topology_seed = source.joint_seed.getTraj();
    const double topology_duration = topology_seed.getTotalDuration();
    const auto sample_reference = [&](const double world_time) -> Eigen::Vector3d {
      if (k3_peer)
        return topology_seed.getPos(std::max(0.0, std::min(
            topology_duration, world_time - activation)));
      const double phase = world_time - reference_phase_shift;
      size_t low = 0;
      size_t high = 0;
      double alpha = 0.0;
      if (phase <= message->knot_world_times[base])
        low = high = 0;
      else if (phase >= message->knot_world_times[base + knot_count - 1])
        low = high = knot_count - 1;
      else
      {
        for (size_t knot = 1; knot < knot_count; ++knot)
          if (phase <= message->knot_world_times[base + knot])
          {
            low = knot - 1;
            high = knot;
            const double span = message->knot_world_times[base + high] -
                                message->knot_world_times[base + low];
            alpha = span > 1.0e-9
                ? (phase - message->knot_world_times[base + low]) / span
                : 0.0;
            break;
          }
      }
      const double radius = (1.0 - alpha) *
          message->knot_radii[base + low] + alpha *
          message->knot_radii[base + high];
      const double bearing_delta = std::atan2(
          std::sin(message->knot_bearings[base + high] -
                   message->knot_bearings[base + low]),
          std::cos(message->knot_bearings[base + high] -
                   message->knot_bearings[base + low]));
      const double bearing = message->knot_bearings[base + low] +
                             alpha * bearing_delta;
      const double height = (1.0 - alpha) *
          message->knot_heights[base + low] + alpha *
          message->knot_heights[base + high];
      const Eigen::Vector3d target = targetPositionAt(world_time);
      return target + Eigen::Vector3d(radius * std::cos(bearing),
                                      radius * std::sin(bearing), height);
    };

    const int piece_count = std::max(2, static_cast<int>(knot_count) - 1);
    Eigen::VectorXd durations = Eigen::VectorXd::Constant(
        piece_count, message->execution_horizon /
                         static_cast<double>(piece_count));
    Eigen::MatrixXd inner_points(3, piece_count - 1);
    for (int piece = 1; piece < piece_count; ++piece)
    {
      const double ratio = static_cast<double>(piece) /
                           static_cast<double>(piece_count);
      const Eigen::Vector3d desired = sample_reference(
          activation + ratio * message->execution_horizon);
      const Eigen::Vector3d topology = topology_seed.getPos(
          ratio * topology_duration);
      const Eigen::Vector3d displacement = desired - topology;
      inner_points.col(piece - 1) = topology + displacement /
          std::max(1.0, displacement.norm() / 0.35);
    }
    const double terminal_world = activation + message->execution_horizon;
    const Eigen::Vector3d desired_tail = sample_reference(terminal_world);
    const Eigen::Vector3d topology_tail = topology_seed.getPos(topology_duration);
    const Eigen::Vector3d tail_delta = desired_tail - topology_tail;
    const Eigen::Vector3d tail_p = topology_tail + tail_delta /
        std::max(1.0, tail_delta.norm() / 0.35);
    const double velocity_dt = std::min(0.05, 0.25 * message->execution_horizon);
    const Eigen::Vector3d tail_v = velocity_dt > 1.0e-6
        ? (sample_reference(terminal_world) -
           sample_reference(terminal_world - velocity_dt)) / velocity_dt
        : target_state_velocity_;
    Eigen::Matrix3d head_state;
    Eigen::Matrix3d tail_state;
    head_state << head_p, head_v, head_a;
    tail_state << tail_p, tail_v, Eigen::Vector3d::Zero();
    poly_traj::MinJerkOpt initializer;
    initializer.reset(head_state, tail_state, piece_count);
    initializer.generate(inner_points, durations);

    std::vector<std::pair<int, int>> segments;
    std::string constraint_reason;
    if (ploy_traj_opt_->finelyCheckAndSetConstraintPoints(
            segments, initializer, true, &constraint_reason) ==
        PolyTrajOptimizer::CHK_RET::ERR)
      return reject("LOCAL_CONSTRAINT_INITIALIZATION_FAILED:" +
                    constraint_reason);
    std::vector<LocalSfcPlane> realized_sfc = source.local_sfc_planes;
    for (auto &plane : realized_sfc)
    {
      if (!plane.world_time_anchored && topology_duration > 1.0e-9)
      {
        plane.active_start = plane.active_start / topology_duration *
                             message->execution_horizon;
        plane.active_end = plane.active_end / topology_duration *
                           message->execution_horizon;
      }
    }

    // Reconstruct the exact existing SIDE authority.  This is deliberately
    // not inferred from a new handoff geometry: Team may refine P/T only
    // inside the Local candidate's original discrete side region.
    std::unique_ptr<CandidateSideScope> team_side_scope;
    TeamReferenceSideCertificate realized_side_certificate;
    if ((source.kind == CandidateKind::SIDE_PLUS ||
         source.kind == CandidateKind::SIDE_MINUS) &&
        topology_duration > 1.0e-6)
    {
      const int side = source.kind == CandidateKind::SIDE_PLUS ? 1 : -1;
      double conflict_progress = 0.5;
      if (std::isfinite(source.conflict.first_risk_world_time) &&
          source.conflict.first_risk_world_time > 0.0 &&
          std::isfinite(source.prediction_epoch))
        conflict_progress = std::max(0.05, std::min(
            0.95, (source.conflict.first_risk_world_time -
                   source.prediction_epoch) / topology_duration));
      if (message->handoff_repair_k == 3 &&
          std::isfinite(source.local_side_conflict_progress) &&
          source.local_side_conflict_progress >= 0.0 &&
          source.local_side_conflict_progress <= 1.0)
        conflict_progress = source.local_side_conflict_progress;
      Eigen::Vector3d side_direction = source.conflict.frame_right;
      side_direction.z() = 0.0;
      if (message->handoff_repair_k == 3 &&
          source.local_side_direction.allFinite() &&
          source.local_side_direction.head<2>().norm() > 1.0e-6)
        side_direction = source.local_side_direction;
      if (side_direction.norm() < 1.0e-6)
      {
        const Eigen::Vector3d chord = tail_p - head_p;
        side_direction = Eigen::Vector3d(chord.y(), -chord.x(), 0.0);
      }
      if (side_direction.norm() > 1.0e-6)
      {
        side_direction.normalize();
        // Keep the K3 SIDE band at the same normalized path location as the
        // Local candidate. A full-trajectory band can include fixed MINCO
        // endpoints outside the Local basin and create impossible hard rows.
        // M2 and ordinary Team realization retain their existing semantics.
        const bool k3_side_window = message->handoff_repair_k == 3;
        const double source_window =
            std::isfinite(source.local_guidance_window) &&
                    source.local_guidance_window > 0.0
                ? source.local_guidance_window
                : std::min(0.5, 0.45 / topology_duration);
        const double team_window = k3_side_window ? source_window : 1.0;
        const double shape = std::max(
            0.10, std::sin(M_PI * conflict_progress));
        const double signed_lateral = side *
            (topology_seed.getPos(conflict_progress * topology_duration) -
             head_p).dot(side_direction);
        const double side_offset = k3_side_window &&
                std::isfinite(source.local_side_offset) &&
                source.local_side_offset > 0.0
            ? source.local_side_offset
            : std::max(0.10, signed_lateral / shape);
        realized_side_certificate.active = true;
        realized_side_certificate.origin = head_p;
        realized_side_certificate.direction = side_direction;
        realized_side_certificate.sign = side;
        realized_side_certificate.offset = side_offset;
        realized_side_certificate.conflict_progress = conflict_progress;
        realized_side_certificate.guidance_window = team_window;
        team_side_scope.reset(new CandidateSideScope(
            ploy_traj_opt_.get(), head_p, tail_p, side, side_offset,
            conflict_progress, team_window, true, &side_direction));
        if (k3_side_window)
          ROS_INFO("[TEAM_SIDE_WINDOW] drone=%d reference_id=%lu "
                   "candidate_id=%d source_window=%.6f team_window=%.6f "
                   "conflict_progress=%.6f source_duration=%.6f "
                   "team_duration=%.6f",
                   pp_.drone_id,
                   static_cast<unsigned long>(message->team_reference_id),
                   source_candidate_id, source_window, team_window,
                   conflict_progress, topology_duration,
                   message->execution_horizon);
      }
    }
    ploy_traj_opt_->setCandidateLocalSfc(realized_sfc);
    ploy_traj_opt_->setCandidatePreservationReference(topology_seed);
    // The Team reference initializer is defined at this activation epoch.
    ploy_traj_opt_->setTeamVisibilityReserveActivation(activation);
    const Eigen::MatrixXd initial_constraints = initializer.getInitConstraintPoints(
        ploy_traj_opt_->get_cps_num_prePiece_());
    Eigen::MatrixXd optimized_points;
    double final_cost = std::numeric_limits<double>::infinity();
    const double deadline = ros::WallTime::now().toSec() + 0.045;
    ploy_traj_opt_->setExecutionDeadline(deadline, 0.002);
    // A K3 peer supplies its bound Local topology seed to the joint
    // validator.  Running an unrelated visibility optimization here can
    // change a healthy peer channel and erase the target UAV's K3 gain.
    const bool optimized = !k3_peer &&
        ploy_traj_opt_->optimizeTrajectoryWithinBudget(
            head_state, tail_state, inner_points, durations,
            optimized_points, final_cost);
    ploy_traj_opt_->setExecutionDeadline(
        std::numeric_limits<double>::infinity(), 0.0);
    if (!optimized && !message->handoff_contract_active)
    {
      const double latency = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - wall_start).count();
      return reject("LOCAL_MINCO_REALIZATION_FAILED", latency);
    }
    poly_traj::MinJerkOpt realized = optimized
        ? ploy_traj_opt_->getMinJerkOpt() : initializer;
    if (!optimized && k3_peer)
      ROS_INFO("[TEAM_K3_PEER_BASELINE] drone=%d contract_id=%lu "
               "source_candidate_id=%d action=RETAIN_BOUND_LOCAL_SEED",
               pp_.drone_id,
               static_cast<unsigned long>(message->handoff_contract_id),
               source_candidate_id);
    else if (!optimized)
      ROS_INFO("[TEAM_LOCAL_SAFE_SEED_RETAINED] drone=%d contract_id=%lu "
               "reason=LOCAL_MINCO_OPTIMIZER_FAILED action=HARD_PREFLIGHT",
               pp_.drone_id,
               static_cast<unsigned long>(message->handoff_contract_id));
    poly_traj::Trajectory realized_traj = realized.getTraj();
    double max_velocity = 0.0, max_acceleration = 0.0, max_jerk = 0.0;
    std::string hard_reason;
    DynamicRiskInfo dynamic = evaluateDynamicRisk(
        realized_traj, activation, false);
    const auto local_hard_preflight = [&](const poly_traj::Trajectory &traj,
                                          std::string &reason) {
      dynamic = evaluateDynamicRisk(traj, activation, false);
      if (!checkTrajectoryDynamics(traj, max_velocity, max_acceleration,
                                   max_jerk))
      { reason = "DYNAMICS_FAIL"; return false; }
      if (!checkTrajectoryStaticSafety(traj, false, reason)) return false;
      if (!dynamic.valid)
      { reason = "DYNAMIC_PREDICTION_INVALID"; return false; }
      if (dynamic.hard_collision)
      { reason = "HARD_DYNAMIC_COLLISION_FAIL"; return false; }
      if (!checkTrajectorySwarmSafety(traj, activation, false, reason))
        return false;
      if (!validateRetimedLocalSfc(source, traj))
      { reason = "LOCAL_SFC_FAIL"; return false; }
      return validateExecutionTrajectory(traj, activation, false, reason,
                                         nullptr);
    };
    if (!local_hard_preflight(realized_traj, hard_reason))
    {
      bool seed_retained = false;
      if (message->handoff_contract_active && optimized)
      {
        std::string seed_reason;
        const poly_traj::Trajectory seed_traj = initializer.getTraj();
        if (local_hard_preflight(seed_traj, seed_reason))
        {
          realized = initializer;
          realized_traj = seed_traj;
          hard_reason.clear();
          seed_retained = true;
          ROS_INFO("[TEAM_LOCAL_SAFE_SEED_RETAINED] drone=%d contract_id=%lu "
                   "reason=OPTIMIZED_LOCAL_HARD_PREFLIGHT_FAILED "
                   "action=USE_PREFLIGHTED_INITIALIZER",
                   pp_.drone_id,
                   static_cast<unsigned long>(message->handoff_contract_id));
        }
        else if (hard_reason.empty())
          hard_reason = seed_reason;
      }
      if (seed_retained)
      {
        // Continue into Team-SCP from the exact hard-safe Local initializer.
      }
      else
      {
      const double latency = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - wall_start).count();
      return reject("LOCAL_HARD_PREFLIGHT_FAILED:" + hard_reason, latency);
      }
    }

    if (message->handoff_contract_active && !k3_peer)
    {
      if (!ploy_traj_opt_->teamSpatiotemporalSCPEnabled())
        return reject("TEAM_SPATIOTEMPORAL_SCP_DISABLED");
      const double remaining =
          message->handoff_contract_expire_time.toSec() -
          ros::Time::now().toSec();
      if (!std::isfinite(remaining) || remaining <= 0.004)
        return reject("HANDOFF_CONTRACT_EXPIRED_BEFORE_TEAM_SCP");

      // ---- Feedback096 single-flight -----------------------------------
      // A Team refinement is open from snapshot freeze until its transaction
      // reaches a terminal state.  While it is open, further schedules only
      // refresh the latest prediction snapshot (latest-wins coalescing); they
      // never start a competing solve and they never cancel the running one.
      // A rolling Local update is deliberately NOT a reason to cancel.
      if (team_refinement_open_ || hasCommittedPendingTeamTail())
      {
        ++team_ref_coalesced_;
        ++latest_team_prediction_snapshot_id_;
        ROS_INFO_THROTTLE(0.5,
            "[TEAM_REFINEMENT_COALESCED] drone=%d contract_id=%lu "
            "refinement_id=%lu open=%d pending_transaction=%d "
            "latest_snapshot_id=%lu coalesced_total=%lu",
            pp_.drone_id,
            static_cast<unsigned long>(message->handoff_contract_id),
            static_cast<unsigned long>(team_refinement_.refinement_id),
            static_cast<int>(team_refinement_open_),
            static_cast<int>(hasCommittedPendingTeamTail()),
            static_cast<unsigned long>(latest_team_prediction_snapshot_id_),
            team_ref_coalesced_);
        return reject("TEAM_REFINEMENT_COALESCED");
      }

      // ---- Feedback096 frozen input snapshot ---------------------------
      // Everything the solver is allowed to depend on is captured here.  The
      // Local planner keeps rolling; a later update only advances
      // latest_team_prediction_snapshot_id_ and is handled by the adoption
      // re-check, never by cancelling this solve.
      {
        TeamRefinementSnapshot snapshot;
        snapshot.repair_k = message->handoff_repair_k;
        snapshot.refinement_id = ++team_refinement_serial_;
        snapshot.contract_id = message->handoff_contract_id;
        snapshot.team_solution_id = message->team_reference_id;
        snapshot.activation = activation;
        snapshot.acquire_by =
            message->handoff_acquire_by_world_time.toSec();
        snapshot.preserve_until =
            message->handoff_preserve_until_world_time.toSec();
        snapshot.contract_expire =
            message->handoff_contract_expire_time.toSec();
        snapshot.critical_begin = std::max(0.0,
            (snapshot.repair_k == 3 ? message->handoff_t_cross.toSec()
                                    : snapshot.acquire_by) - snapshot.activation);
        snapshot.critical_end = std::max(snapshot.critical_begin,
            (snapshot.repair_k == 3 ? message->handoff_t_recover.toSec()
                                    : snapshot.preserve_until) - snapshot.activation);
        snapshot.outgoing = message->handoff_outgoing_uav;
        snapshot.incoming = message->handoff_incoming_uav;
        snapshot.stable = message->handoff_stable_observer_uav;
        snapshot.required_margin =
            message->handoff_required_visibility_margin;
        snapshot.m2_before = message->handoff_m2_min;
        snapshot.start_snapshot_id = latest_team_prediction_snapshot_id_;
        // Feedback100: M2 only detects the risk.  Carry the published repair
        // target set, the earliest-crossing interval and the pipeline anchors
        // into the frozen snapshot so the trial can be judged and timed
        // without depending on anything that keeps rolling.
        snapshot.receive_wall = ros::WallTime::now().toSec();
        snapshot.repair_candidates[0] = -1;
        snapshot.repair_candidates[1] = -1;
        for (size_t slot = 0;
             slot < message->handoff_repair_candidates.size() && slot < 2;
             ++slot)
        {
          snapshot.repair_candidates[slot] =
              static_cast<int>(message->handoff_repair_candidates[slot]);
          if (slot < message->handoff_repair_candidate_margins.size())
            snapshot.repair_candidate_margins[slot] =
                message->handoff_repair_candidate_margins[slot];
        }
        for (size_t drone = 0;
             drone < message->handoff_forecast_margins.size() && drone < 3;
             ++drone)
          snapshot.forecast_margins[drone] =
              message->handoff_forecast_margins[drone];
        snapshot.repair_priority_uav =
            static_cast<int>(message->handoff_repair_priority_uav);
        snapshot.t_cross = message->handoff_t_cross.toSec();
        snapshot.t_recover = message->handoff_t_recover.toSec();
        snapshot.t_star = message->handoff_t_star.toSec();
        snapshot.forecast_created_world =
            message->handoff_forecast_created_world_time.toSec();
        snapshot.detect_wall = message->handoff_detect_wall;
        snapshot.request_wall = message->handoff_request_wall;
        snapshot.repair_declared_candidate =
            snapshot.repair_candidates[0] == pp_.drone_id ||
            snapshot.repair_candidates[1] == pp_.drone_id;
        for (int drone = 0; drone < 3; ++drone)
        {
          snapshot.source_trajectory_ids[drone] = traj_.swarm_traj[drone].traj_id;
          snapshot.source_generations[drone] =
              static_cast<std::uint64_t>(traj_.swarm_traj[drone].start_time);
        }
        team_refinement_ = snapshot;
        ++team_ref_started_;
      }
      const double refinement_wall_begin = ros::WallTime::now().toSec();

      // =====================================================================
      // Feedback098 — latest-baseline recheck (rolling request entry check).
      //
      // The forecast that triggered this request was produced for the Local
      // baseline as it existed then.  Local rolling continues, so re-evaluate
      // the CURRENT baseline at the same critical world time with the same
      // visibility authority before spending an SCP.  This is a per-request
      // entry check, not a persistent contract state machine.
      // =====================================================================
      {
        const double critical_world_time =
            message->handoff_critical_world_time.toSec();
        const auto &baseline = traj_.local_traj;
        double planner_margin = -std::numeric_limits<double>::infinity();
        int planner_limiter = 0;
        bool same_window = false;
        const bool have_planner_margin =
            baseline.traj.getPieceNum() > 0 &&
            std::isfinite(critical_world_time) &&
            critical_world_time >= baseline.start_time - 1.0e-9 &&
            critical_world_time <=
                baseline.start_time + baseline.duration + 1.0e-9 &&
            ploy_traj_opt_->evaluateVisibilityMarginAt(
                baseline.traj, baseline.start_time, critical_world_time,
                planner_margin, planner_limiter);
        same_window = have_planner_margin;
        // Feedback101 reserve trace: the chain the reserve must improve.
        team_reserve_trace_forecast_margin_ = message->handoff_limiting_margin;
        // Feedback099 (telemetry only): the SCP seed is the realized Team
        // reference, while this recheck measures the rolling Local baseline.
        // They are only comparable if both are reduced over the SAME critical
        // window, because the SCP reports margin_before as a window minimum and
        // a single point sample is always >= that minimum.  Record the rolling
        // baseline's window minimum so the two seeds can be compared directly.
        double baseline_window_min =
            -std::numeric_limits<double>::infinity();
        bool baseline_window_scanned = false;
        {
          const double window_begin =
              std::max(activation,
                       message->handoff_acquire_by_world_time.toSec());
          const double window_end = std::min(
              message->handoff_preserve_until_world_time.toSec(),
              baseline.start_time + baseline.duration);
          if (baseline.traj.getPieceNum() > 0 && have_planner_margin &&
              window_end >= window_begin - 1.0e-9)
          {
            baseline_window_scanned = true;
            baseline_window_min = std::numeric_limits<double>::infinity();
            for (double t = window_begin; t <= window_end + 1.0e-9;
                 t += visibility_forecast_dt_)
            {
              double m = -std::numeric_limits<double>::infinity();
              int limiter = 0;
              if (!ploy_traj_opt_->evaluateVisibilityMarginAt(
                      baseline.traj, baseline.start_time, t, m, limiter) ||
                  !std::isfinite(m))
              {
                baseline_window_scanned = false;
                baseline_window_min =
                    -std::numeric_limits<double>::infinity();
                break;
              }
              if (m < baseline_window_min)
                baseline_window_min = m;
            }
          }
        }
        ROS_INFO("[TEAM_REQUEST_RECHECK] drone=%d contract_id=%lu "
                 "limiting_uav=%d critical_world_time=%.9f "
                 "forecast_margin=%.6f current_baseline_margin=%.6f "
                 "baseline_window_min=%.6f baseline_window_scanned=%d "
                 "baseline_traj_id=%d same_window=%d",
            pp_.drone_id,
            static_cast<unsigned long>(message->handoff_contract_id),
            static_cast<int>(message->handoff_limiting_uav),
            critical_world_time, message->handoff_limiting_margin,
            planner_margin, baseline_window_min,
            static_cast<int>(baseline_window_scanned), baseline.traj_id,
            static_cast<int>(same_window));
        team_reserve_trace_baseline_before_ = baseline_window_min;

        // Only the UAV named by the coordinator may skip its refinement, and
        // only when the current baseline genuinely already satisfies the
        // contraction over the same window.
        const bool named_uav =
            message->handoff_repair_k == 2 &&
            message->handoff_limiting_uav == pp_.drone_id;
        bool baseline_satisfies = false;
        if (named_uav && have_planner_margin &&
            planner_margin >= message->handoff_required_visibility_margin)
        {
          baseline_satisfies = true;
          const double window_begin = std::max(
              activation, message->handoff_acquire_by_world_time.toSec());
          const double window_end = std::min(
              message->handoff_preserve_until_world_time.toSec(),
              baseline.start_time + baseline.duration);
          for (double t = window_begin;
               t <= window_end + 1.0e-9; t += visibility_forecast_dt_)
          {
            double m = -std::numeric_limits<double>::infinity();
            int limiter = 0;
            if (!ploy_traj_opt_->evaluateVisibilityMarginAt(
                    baseline.traj, baseline.start_time, t, m, limiter) ||
                m < message->handoff_required_visibility_margin)
            {
              baseline_satisfies = false;
              break;
            }
          }
        }
        if (baseline_satisfies)
        {
          ++team_ref_current_baseline_safe_;
          ROS_INFO("[TEAM_REFINEMENT_RESULT] drone=%d refinement_id=%lu "
                   "contract_id=%lu result=NOOP_CURRENT_BASELINE_SAFE "
                   "critical_world_time=%.9f planner_margin=%.6f "
                   "required=%.6f limiter=%d",
              pp_.drone_id,
              static_cast<unsigned long>(team_refinement_.refinement_id),
              static_cast<unsigned long>(message->handoff_contract_id),
              critical_world_time, planner_margin,
              message->handoff_required_visibility_margin, planner_limiter);
          return reject("TEAM_REFINEMENT_NOOP_CURRENT_BASELINE_SAFE");
        }
      }

      PolyTrajOptimizer::TeamVisibilityContract contract;
      contract.active = true;
      contract.repair_k = team_refinement_.repair_k;
      contract.contract_id = message->handoff_contract_id;
      contract.activation_world_time = activation;
      contract.acquire_by_world_time =
          message->handoff_acquire_by_world_time.toSec();
      contract.preserve_until_world_time =
          message->handoff_preserve_until_world_time.toSec();
      contract.contract_expire_world_time =
          message->handoff_contract_expire_time.toSec();
      contract.required_margin =
          message->handoff_required_visibility_margin;
      contract.outgoing_uav = message->handoff_outgoing_uav;
      contract.incoming_uav = message->handoff_incoming_uav;
      contract.stable_observer_uav =
          message->handoff_stable_observer_uav;
      contract.requested_mode = PolyTrajOptimizer::TeamSCPMode::AUTO;
      ploy_traj_opt_->setTeamVisibilityContract(contract);
      ploy_traj_opt_->setCandidatePreservationReference(realized_traj);

      const int refined_piece_count = realized_traj.getPieceNum();
      const Eigen::MatrixXd refined_positions = realized_traj.getPositions();
      const Eigen::MatrixXd refined_inner = refined_positions.block(
          0, 1, 3, refined_piece_count - 1);
      const Eigen::VectorXd refined_durations = realized_traj.getDurations();
      Eigen::Matrix3d refined_head, refined_tail;
      refined_head << realized_traj.getJuncPos(0),
          realized_traj.getJuncVel(0), realized_traj.getJuncAcc(0);
      refined_tail << realized_traj.getJuncPos(refined_piece_count),
          realized_traj.getJuncVel(refined_piece_count),
          realized_traj.getJuncAcc(refined_piece_count);
      Eigen::MatrixXd team_optimal_points;
      double team_final_cost = std::numeric_limits<double>::infinity();
      PolyTrajOptimizer::TeamSCPResult team_result;
      const double team_deadline = ros::WallTime::now().toSec() +
          std::min(0.045, std::max(0.004, remaining - 0.002));
      ploy_traj_opt_->setExecutionDeadline(team_deadline, 0.002);
      // Feedback098: SCP body timing is measured on its own span so that
      // TEAM_SCP_SOLVE_MS <= TEAM_REFINEMENT_TOTAL_MS always holds.
      // ---- Feedback100 repair-trial eligibility -------------------------
      // M2 only DETECTS the risk.  Who gets repaired is decided here from the
      // published repair target set, plus the UAV's OWN realized-seed deficit.
      // The second clause is required because the published forecast ranks the
      // PRE-realization baselines: a UAV the forecast considered healthy can
      // still realize a deficient seed (Feedback099: the one adopted repair
      // came from exactly such a UAV).  A trial costs a few milliseconds, so
      // both declared candidates always get a real solve.
      {
        double seed_window_min = std::numeric_limits<double>::infinity();
        bool seed_scanned = false;
        if (realized_traj.getPieceNum() > 0)
        {
          const double window_begin = std::max(
              activation, team_refinement_.acquire_by);
          const double window_end = std::min(
              team_refinement_.preserve_until,
              activation + realized_traj.getTotalDuration());
          if (window_end >= window_begin - 1.0e-9)
          {
            seed_scanned = true;
            for (double t = window_begin; t <= window_end + 1.0e-9;
                 t += visibility_forecast_dt_)
            {
              double m = -std::numeric_limits<double>::infinity();
              int limiter = 0;
              if (!ploy_traj_opt_->evaluateVisibilityMarginAt(
                      realized_traj, activation, t, m, limiter) ||
                  !std::isfinite(m))
              {
                seed_scanned = false;
                break;
              }
              if (m < seed_window_min)
                seed_window_min = m;
            }
          }
        }
        team_refinement_.seed_window_min =
            (seed_scanned && std::isfinite(seed_window_min))
                ? seed_window_min
                : -std::numeric_limits<double>::infinity();
        team_refinement_.repair_seed_deficient =
            std::isfinite(team_refinement_.seed_window_min) &&
            team_refinement_.seed_window_min <
                team_refinement_.required_margin;
        team_refinement_.repair_eligible =
            team_refinement_.repair_k == 3
                ? team_refinement_.repair_declared_candidate &&
                  team_refinement_.repair_priority_uav == pp_.drone_id
                : team_refinement_.repair_declared_candidate ||
                  team_refinement_.repair_seed_deficient;

        // ---- Winner rule ------------------------------------------------
        // It MUST be derivable identically on every node from PUBLISHED data.
        // An earlier version let each node use its own realized-seed margin for
        // itself and the published forecast for its peers; the two declared
        // candidates then each judged the OTHER to be the better repair target
        // and both deferred, so no repair was attempted at all.
        // A seed-deficient NON-candidate is still allowed to commit: its
        // deficiency is private information that no published rule can see,
        // and its repair is a genuine visibility improvement.
        const bool is_priority =
            (pp_.drone_id == team_refinement_.repair_priority_uav);
        if (team_refinement_.repair_declared_candidate)
        {
          team_refinement_.trial_selected_uav = is_priority
              ? pp_.drone_id : team_refinement_.repair_priority_uav;
          team_refinement_.trial_selection_reason =
              is_priority ? "DECLARED_PRIORITY_SELF"
                          : "DECLARED_PRIORITY_PEER";
        }
        else if (team_refinement_.repair_seed_deficient)
        {
          team_refinement_.trial_selected_uav = pp_.drone_id;
          team_refinement_.trial_selection_reason =
              "SEED_DEFICIENT_NON_CANDIDATE";
        }
        else
        {
          team_refinement_.trial_selected_uav = -1;
          team_refinement_.trial_selection_reason = "NOT_ELIGIBLE";
        }
        ROS_INFO("[TEAM_REPAIR_ELIGIBILITY] drone=%d contract_id=%lu "
                 "M2_LIMITING_UAV=%d TEAM_REPAIR_CANDIDATES=[%d,%d] "
                 "TEAM_REPAIR_PRIORITY_UAV=%d declared=%d seed_window_min=%.6f "
                 "seed_deficient=%d eligible=%d provisional_winner=%d "
                 "reason=%s t_cross=%.9f t_star=%.9f",
            pp_.drone_id,
            static_cast<unsigned long>(message->handoff_contract_id),
            static_cast<int>(message->handoff_limiting_uav),
            team_refinement_.repair_candidates[0],
            team_refinement_.repair_candidates[1],
            team_refinement_.repair_priority_uav,
            static_cast<int>(team_refinement_.repair_declared_candidate),
            team_refinement_.seed_window_min,
            static_cast<int>(team_refinement_.repair_seed_deficient),
            static_cast<int>(team_refinement_.repair_eligible),
            team_refinement_.trial_selected_uav,
            team_refinement_.trial_selection_reason.c_str(),
            team_refinement_.t_cross, team_refinement_.t_star);
      }
      const double scp_wall_begin = ros::WallTime::now().toSec();
      team_refinement_.scp_begin_wall = scp_wall_begin;
      const bool team_scp_ok = ploy_traj_opt_->runTeamContractSCP(
          refined_head, refined_tail, refined_inner, refined_durations,
          team_optimal_points, team_final_cost, team_result);
      const double scp_wall_end = ros::WallTime::now().toSec();
      team_refinement_.scp_end_wall = scp_wall_end;
      ploy_traj_opt_->setExecutionDeadline(
          std::numeric_limits<double>::infinity(), 0.0);
      ploy_traj_opt_->clearTeamVisibilityContract();
      if (!team_scp_ok)
      {
        const double latency = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - wall_start).count();
        ++team_ref_solver_infeasible_;
        team_refinement_solver_pass_ = false;
        team_refinement_solver_reason_ = team_result.reason;
        team_result_margin_after_ = -std::numeric_limits<double>::infinity();
        team_refinement_solve_ms_ = (scp_wall_end - scp_wall_begin) * 1000.0;
        team_refinement_total_ms_ =
            (ros::WallTime::now().toSec() - refinement_wall_begin) * 1000.0;
        ROS_WARN("[TEAM_REFINEMENT_RESULT] drone=%d refinement_id=%lu "
                 "contract_id=%lu result=SOLVER_INFEASIBLE reason=%s "
                 "solve_ms=%.3f total_ms=%.3f "
                 "updates_during_solve=%lu linearization_world_time=%.9f "
                 "linearization_component=%s",
            pp_.drone_id,
            static_cast<unsigned long>(team_refinement_.refinement_id),
            static_cast<unsigned long>(message->handoff_contract_id),
            team_result.reason.c_str(), team_refinement_solve_ms_,
            team_refinement_total_ms_,
            static_cast<unsigned long>(latest_team_prediction_snapshot_id_ -
                                       team_refinement_.start_snapshot_id),
            team_result.linearization_invalid_world_time,
            team_result.linearization_invalid_component.c_str());
        // Feedback100: an infeasible trial is still a trial.  Report it with
        // the same schema as a successful one, plus the full pipeline latency,
        // so a repair event that finds no feasible candidate is still fully
        // measurable instead of being invisible.
        team_refinement_.trial_feasible = false;
        team_refinement_.trial_m2_after =
            -std::numeric_limits<double>::infinity();
        team_refinement_.trial_reject_reason = "SOLVER_INFEASIBLE";
        if (team_refinement_.repair_k == 3)
          ROS_INFO("[TEAM_K3_TRIAL] drone=%d contract_id=%lu mode=%s "
                   "margin_before=%.6f margin_after=%.6f M2_before=%.6f "
                   "M2_after=-inf dp_norm=%.9f dT_norm=%.9f "
                   "solver_status=FAIL exact_validation=0 reason=%s",
                   pp_.drone_id,
                   static_cast<unsigned long>(message->handoff_contract_id),
                   team_result.mode == PolyTrajOptimizer::TeamSCPMode::TEAM_T_ONLY
                       ? "T" : "PT", team_result.margin_before,
                   team_result.margin_after, team_refinement_.m2_before,
                   team_result.delta_p_norm, team_result.delta_real_t_norm,
                   team_result.reason.c_str());
        ROS_INFO("[TEAM_REPAIR_TRIAL] drone=%d contract_id=%lu "
                 "M2_LIMITING_UAV=%d TEAM_REPAIR_CANDIDATES=[%d,%d] "
                 "is_declared_candidate=%d trial_feasible=0 "
                 "reject_reason=SOLVER_INFEASIBLE solver_reason=%s "
                 "margin_before=-inf margin_after=-inf required=%.6f "
                 "M2_after=-inf dp_norm=0.000000000 dt_norm=0.000000000 "
                 "deformation_cost=-1 provisional_winner=%d",
            pp_.drone_id,
            static_cast<unsigned long>(message->handoff_contract_id),
            static_cast<int>(message->handoff_limiting_uav),
            team_refinement_.repair_candidates[0],
            team_refinement_.repair_candidates[1],
            static_cast<int>(team_refinement_.repair_declared_candidate),
            team_result.reason.c_str(), team_refinement_.required_margin,
            team_refinement_.trial_selected_uav);
        logRelayLatency("SOLVER_INFEASIBLE");
        return reject("TEAM_SCP_FAILED:" + team_result.reason, latency);
      }
      {
        // Feedback098: solve_ms is the SCP body span only.  Using wall_start
        // here (function entry, before the reference realization) made
        // solve_ms exceed total_ms, which is structurally impossible.
        const double solve_ms = (scp_wall_end - scp_wall_begin) * 1000.0;
        team_refinement_solve_ms_ = solve_ms;
        team_refinement_total_ms_ =
            (ros::WallTime::now().toSec() - refinement_wall_begin) * 1000.0;
        // Feedback098: a zero-modification acceptance is NOT a refinement.  It
        // must not gain improvement authority (no certificate, no overwrite
        // protection) and must not open a speculative Team transaction.
        const bool zero_modification =
            team_result.reason == "TEAM_VISIBILITY_ALREADY_SATISFIED" ||
            (team_result.delta_p_norm <= 1.0e-9 &&
             team_result.delta_real_t_norm <= 1.0e-9);
        team_refinement_zero_modification_ = zero_modification;
        if (zero_modification)
          ++team_ref_zero_modification_;
        else
          ++team_ref_solver_pass_;
        team_refinement_solver_pass_ = !zero_modification;
        team_refinement_solver_reason_ = team_result.reason;
        team_result_margin_after_ = team_result.margin_after;
        team_result_dp_norm_ = team_result.delta_p_norm;
        team_result_dt_norm_ = team_result.delta_real_t_norm;
        team_result_mode_t_only_ =
            (team_result.mode == PolyTrajOptimizer::TeamSCPMode::TEAM_T_ONLY)
                ? 1 : 0;
        team_result_margin_before_ = team_result.margin_before;
        // ---- Feedback100 repair-trial judgement --------------------------
        // A solver status of "success" is NOT feasibility.  The trial is only
        // feasible when the exact nonlinear margin reaches the contracted
        // requirement over the critical window AND the resulting team metric
        // either recovers M2 or at least satisfies the pre-existing Team
        // acceptance rule (a strictly measurable improvement over m2_before).
        {
          double p_trust = 0.0;
          double t_trust = 0.0;
          ploy_traj_opt_->teamSCPTrustScale(p_trust, t_trust);
          team_refinement_.trial_deformation_cost =
              team_result.delta_p_norm / p_trust +
              team_result.delta_real_t_norm / t_trust;

          // M2 after this single UAV's repair, using the other two channels'
          // published forecast margins at the critical instant.
          std::array<double, 3> after_margins{{
              team_refinement_.forecast_margins[0],
              team_refinement_.forecast_margins[1],
              team_refinement_.forecast_margins[2]}};
          after_margins[pp_.drone_id] = team_result.margin_after;
          std::array<double, 3> sorted = after_margins;
          std::sort(sorted.begin(), sorted.end(),
                    [](double lhs, double rhs) { return lhs > rhs; });
          const double m2_after = sorted[1];
          team_refinement_.trial_m2_after = m2_after;

          const bool margin_recovered =
              std::isfinite(team_result.margin_after) &&
              team_result.margin_after >=
                  team_refinement_.required_margin - 1.0e-9;
          const bool m2_recovered =
              std::isfinite(m2_after) &&
              m2_after >= team_refinement_.required_margin - 1.0e-9;
          const bool existing_acceptance =
              team_refinement_.repair_seed_deficient &&
              std::isfinite(team_result.margin_after) &&
              std::isfinite(team_refinement_.seed_window_min) &&
              team_result.margin_after >
                  team_refinement_.seed_window_min + 1.0e-9;
          team_refinement_.trial_feasible =
              !zero_modification && margin_recovered &&
              (team_refinement_.repair_k == 3
                   ? m2_recovered
                   : (m2_recovered || existing_acceptance));
          if (team_refinement_.trial_feasible)
            team_refinement_.trial_reject_reason = "NONE";
          else if (zero_modification)
            team_refinement_.trial_reject_reason = "ZERO_MODIFICATION";
          else if (!margin_recovered)
            team_refinement_.trial_reject_reason =
                "REFINED_MARGIN_BELOW_REQUIRED";
          else
            team_refinement_.trial_reject_reason = "M2_NOT_RECOVERED";

          ROS_INFO("[TEAM_REPAIR_TRIAL] drone=%d contract_id=%lu "
                   "M2_LIMITING_UAV=%d TEAM_REPAIR_CANDIDATES=[%d,%d] "
                   "is_declared_candidate=%d trial_feasible=%d reject_reason=%s "
                   "mode=%s margin_before=%.6f margin_after=%.6f required=%.6f "
                   "M2_after=%.6f M2_recovered=%d existing_acceptance=%d "
                   "dp_norm=%.9f dt_norm=%.9f deformation_cost=%.6f "
                   "provisional_winner=%d",
              pp_.drone_id,
              static_cast<unsigned long>(message->handoff_contract_id),
              static_cast<int>(message->handoff_limiting_uav),
              team_refinement_.repair_candidates[0],
              team_refinement_.repair_candidates[1],
              static_cast<int>(team_refinement_.repair_declared_candidate),
              static_cast<int>(team_refinement_.trial_feasible),
              team_refinement_.trial_reject_reason.c_str(),
              team_result.mode == PolyTrajOptimizer::TeamSCPMode::TEAM_T_ONLY
                  ? "T" : "PT",
              team_result.margin_before, team_result.margin_after,
              team_refinement_.required_margin, m2_after,
              static_cast<int>(m2_recovered),
              static_cast<int>(existing_acceptance),
              team_result.delta_p_norm, team_result.delta_real_t_norm,
              team_refinement_.trial_deformation_cost,
              team_refinement_.trial_selected_uav);
          if (team_refinement_.repair_k == 3)
            ROS_INFO("[TEAM_K3_TRIAL] drone=%d contract_id=%lu mode=%s "
                     "margin_before=%.6f margin_after=%.6f M2_before=%.6f "
                     "M2_after=%.6f dp_norm=%.9f dT_norm=%.9f "
                     "solver_status=%s exact_validation=%d reason=%s",
                     pp_.drone_id,
                     static_cast<unsigned long>(message->handoff_contract_id),
                     team_result.mode == PolyTrajOptimizer::TeamSCPMode::TEAM_T_ONLY
                         ? "T" : "PT", team_result.margin_before,
                     team_result.margin_after, team_refinement_.m2_before,
                     m2_after, team_result.delta_p_norm,
                     team_result.delta_real_t_norm,
                     team_result.success ? "PASS" : "FAIL",
                     static_cast<int>(margin_recovered),
                     team_refinement_.trial_reject_reason.c_str());

          // Feedback101 reserve verification chain:
          //   forecast margin -> latest Local baseline -> SCP seed
          double baseline_window_min_after_rolling =
              -std::numeric_limits<double>::infinity();
          {
            const auto &rolling = traj_.local_traj;
            const double wb = std::max(activation, team_refinement_.acquire_by);
            const double we = std::min(team_refinement_.preserve_until,
                                       rolling.start_time + rolling.duration);
            if (rolling.traj.getPieceNum() > 0 && we >= wb - 1.0e-9)
            {
              baseline_window_min_after_rolling =
                  std::numeric_limits<double>::infinity();
              for (double t = wb; t <= we + 1.0e-9;
                   t += visibility_forecast_dt_)
              {
                double m = -std::numeric_limits<double>::infinity();
                int limiter = 0;
                if (!ploy_traj_opt_->evaluateVisibilityMarginAt(
                        rolling.traj, rolling.start_time, t, m, limiter) ||
                    !std::isfinite(m))
                {
                  baseline_window_min_after_rolling =
                      -std::numeric_limits<double>::infinity();
                  break;
                }
                if (m < baseline_window_min_after_rolling)
                  baseline_window_min_after_rolling = m;
              }
            }
          }
          ROS_INFO("[TEAM_RESERVE_TRACE] drone=%d contract_id=%lu "
                   "reserve_active=%d reserve_weight=%.3f "
                   "forecast_margin=%.6f "
                   "baseline_window_min_before_reserve=%.6f "
                   "baseline_window_min_after_rolling=%.6f "
                   "scp_seed_margin=%.6f "
                   "decay_forecast_to_seed=%.6f "
                   "decay_forecast_to_rolling=%.6f "
                   "reserve_cost=%.6f reserve_active_samples=%d guide_count=%d "
                   "reserve_activations=%lu",
              pp_.drone_id,
              static_cast<unsigned long>(message->handoff_contract_id),
              static_cast<int>(ploy_traj_opt_->teamVisibilityReserveActive()),
              team_reserve_weight_,
              team_reserve_trace_forecast_margin_,
              team_reserve_trace_baseline_before_,
              baseline_window_min_after_rolling,
              team_refinement_.seed_window_min,
              (std::isfinite(team_reserve_trace_forecast_margin_) &&
               std::isfinite(team_refinement_.seed_window_min))
                  ? (team_reserve_trace_forecast_margin_ -
                     team_refinement_.seed_window_min) : 0.0,
              (std::isfinite(team_reserve_trace_forecast_margin_) &&
               std::isfinite(baseline_window_min_after_rolling))
                  ? (team_reserve_trace_forecast_margin_ -
                     baseline_window_min_after_rolling) : 0.0,
              ploy_traj_opt_->teamVisibilityReserveCost(),
              ploy_traj_opt_->teamVisibilityReserveActiveSamples(),
              ploy_traj_opt_->teamVisibilityReserveGuideCount(),
              static_cast<unsigned long>(team_reserve_activation_count_));
        }
        if (!zero_modification)
        {
          team_ref_solve_ms_samples_.push_back(solve_ms);
          team_ref_total_ms_samples_.push_back(team_refinement_total_ms_);
        }
        if (team_result.mode == PolyTrajOptimizer::TeamSCPMode::TEAM_T_ONLY)
        {
          team_ref_t_solve_ms_samples_.push_back(solve_ms);
          team_ref_t_total_ms_samples_.push_back(team_refinement_total_ms_);
        }
        else
        {
          team_ref_pt_solve_ms_samples_.push_back(solve_ms);
          team_ref_pt_total_ms_samples_.push_back(team_refinement_total_ms_);
        }
        // Only a real (non-zero-modification) refinement is reported as a
        // refinement start/solve; a zero-modification acceptance is a NOOP.
        if (!zero_modification)
        {
          ROS_INFO("[TEAM_REFINEMENT_START] drone=%d refinement_id=%lu "
                   "contract_id=%lu mode=%s activation=%.9f "
                   "critical_window=[%.9f,%.9f] outgoing=%d incoming=%d "
                   "stable=%d required_margin=%.6f m2_before=%.6f "
                   "start_snapshot_id=%lu",
              pp_.drone_id,
              static_cast<unsigned long>(team_refinement_.refinement_id),
              static_cast<unsigned long>(message->handoff_contract_id),
              team_result.mode == PolyTrajOptimizer::TeamSCPMode::TEAM_T_ONLY
                  ? "T" : "PT",
              activation, team_refinement_.critical_begin,
              team_refinement_.critical_end, team_refinement_.outgoing,
              team_refinement_.incoming, team_refinement_.stable,
              team_refinement_.required_margin, team_result.margin_before,
              static_cast<unsigned long>(team_refinement_.start_snapshot_id));
          ROS_INFO("[TEAM_REFINEMENT_SOLVE] drone=%d refinement_id=%lu mode=%s "
                   "solve_ms=%.3f total_ms=%.3f updates_during_solve=%lu "
                   "margin=%.6f->%.6f dp=%.9f dtau=%.9f dT=%.9f",
              pp_.drone_id,
              static_cast<unsigned long>(team_refinement_.refinement_id),
              team_result.mode == PolyTrajOptimizer::TeamSCPMode::TEAM_T_ONLY
                  ? "T" : "PT",
              solve_ms, team_refinement_total_ms_,
              static_cast<unsigned long>(latest_team_prediction_snapshot_id_ -
                                         team_refinement_.start_snapshot_id),
              team_result.margin_before, team_result.margin_after,
              team_result.delta_p_norm, team_result.delta_virtual_t_norm,
              team_result.delta_real_t_norm);
        }
      }

      realized = ploy_traj_opt_->getMinJerkOpt();
      realized_traj = realized.getTraj();
      dynamic = evaluateDynamicRisk(realized_traj, activation, false);
      hard_reason.clear();
      if (!checkTrajectoryDynamics(realized_traj, max_velocity,
                                   max_acceleration, max_jerk) ||
          !checkTrajectoryStaticSafety(realized_traj, false, hard_reason) ||
          !dynamic.valid || dynamic.hard_collision ||
          !checkTrajectorySwarmSafety(realized_traj, activation, false,
                                      hard_reason) ||
          !validateRetimedLocalSfc(source, realized_traj) ||
          !validateExecutionTrajectory(realized_traj, activation, false,
                                       hard_reason, nullptr))
      {
        const double latency = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - wall_start).count();
        return reject("TEAM_SCP_FINAL_HARD_PREFLIGHT_FAILED:" + hard_reason,
                      latency);
      }
      ROS_INFO("[TEAM_SCP_REALIZED] drone=%d contract_id=%lu mode=%s "
               "margin=%.6f->%.6f dp=%.9f dt=%.9f",
               pp_.drone_id,
               static_cast<unsigned long>(message->handoff_contract_id),
               team_result.mode == PolyTrajOptimizer::TeamSCPMode::TEAM_T_ONLY
                   ? "TEAM_T_ONLY" : "TEAM_PT",
               team_result.margin_before, team_result.margin_after,
               team_result.delta_p_norm, team_result.delta_real_t_norm);
    }
    const double latency = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - wall_start).count();
    pending_team_trajectory_ = PendingTeamTrajectory();
    pending_team_trajectory_.accepted = true;
    pending_team_trajectory_.target_centered_reference = true;
    pending_team_trajectory_.team_solution_id = message->team_reference_id;
    pending_team_trajectory_.coordination_generation =
        message->coordination_generation;
    pending_team_trajectory_.planning_generation = planning_generation;
    pending_team_trajectory_.source_candidate_id = source_candidate_id;
    pending_team_trajectory_.source_candidate_index = source_index;
    pending_team_trajectory_.activation_time = activation;
    pending_team_trajectory_.trajectory = realized;
    trajectoryToMINCOMessage(realized_traj, pp_.drone_id,
        source_candidate_id, activation,
        pending_team_trajectory_.realized_message);
    pending_team_trajectory_.reference_schedule = *message;
    publishTeamReferenceAck(*message, planning_generation,
        source_candidate_id, topology_kind, true,
        message->handoff_contract_active
            ? "TEAM_SCP_AND_LOCAL_HARD_PREFLIGHT_PASS"
            : "LOCAL_MINCO_HARD_PREFLIGHT_PASS",
        &realized, latency, &realized_side_certificate);
    if (message->handoff_contract_active && !k3_peer &&
        !team_refinement_zero_modification_)
    {
      // Only a refinement that (a) actually changed the trajectory, (b) passed
      // the full repair-trial judgement, and (c) is the ONE selected repair
      // target may open the adoption re-check window and later receive an
      // improvement certificate.  A trial that solved but lost the selection
      // must not create a competing Team proposal.
      if (!team_refinement_.trial_feasible)
      {
        ROS_WARN("[TEAM_REPAIR_RESULT] drone=%d refinement_id=%lu "
                 "contract_id=%lu result=TRIAL_INFEASIBLE reason=%s "
                 "margin_after=%.6f required=%.6f M2_after=%.6f "
                 "solve_ms=%.3f total_ms=%.3f",
            pp_.drone_id,
            static_cast<unsigned long>(team_refinement_.refinement_id),
            static_cast<unsigned long>(message->handoff_contract_id),
            team_refinement_.trial_reject_reason.c_str(),
            team_result_margin_after_, team_refinement_.required_margin,
            team_refinement_.trial_m2_after,
            team_refinement_solve_ms_, team_refinement_total_ms_);
        logRelayLatency("TRIAL_INFEASIBLE");
        reportTeamRefinementAudit(false);
      }
      else if (team_refinement_.repair_eligible &&
               team_refinement_.repair_declared_candidate &&
               team_refinement_.trial_selected_uav >= 0 &&
               team_refinement_.trial_selected_uav != pp_.drone_id &&
               team_refinement_.trial_selected_uav ==
                   message->handoff_limiting_uav)
      {
        // Feedback100: suppress a declared candidate only when the PUBLISHED
        // priority target is the M2 limiting UAV itself.  The first measured
        // run showed the general rule is unsafe: in both repair events the
        // priority candidate never reached a trial (its contract-carrying
        // reference failed realization / identity validation) while the other
        // declared candidate did solve.  Deferring to an absent candidate lost
        // the only trial that could have repaired the window.
        ROS_WARN("[TEAM_REPAIR_RESULT] drone=%d refinement_id=%lu "
                 "contract_id=%lu result=NOT_SELECTED selected_uav=%d "
                 "margin_after=%.6f deformation_cost=%.6f "
                 "selection_reason=%s solve_ms=%.3f total_ms=%.3f",
            pp_.drone_id,
            static_cast<unsigned long>(team_refinement_.refinement_id),
            static_cast<unsigned long>(message->handoff_contract_id),
            team_refinement_.trial_selected_uav, team_result_margin_after_,
            team_refinement_.trial_deformation_cost,
            team_refinement_.trial_selection_reason.c_str(),
            team_refinement_solve_ms_, team_refinement_total_ms_);
        logRelayLatency("NOT_SELECTED");
        reportTeamRefinementAudit(false);
      }
      else
      {
        team_refinement_open_ = true;
        pending_team_trajectory_.handoff_refinement = true;
        ROS_INFO("[TEAM_REPAIR_SELECTED] selected_uav=%d contract_id=%lu "
                 "M2_LIMITING_UAV=%d mode=%s margin_before=%.6f "
                 "margin_after=%.6f M2_after=%.6f required=%.6f "
                 "deformation_cost=%.6f selection_reason=%s "
                 "t_cross=%.9f t_star=%.9f",
            pp_.drone_id,
            static_cast<unsigned long>(message->handoff_contract_id),
            static_cast<int>(message->handoff_limiting_uav),
            team_result_mode_t_only_ ? "T" : "PT",
            team_result_margin_before_,
            team_result_margin_after_, team_refinement_.trial_m2_after,            team_refinement_.required_margin,
            team_refinement_.trial_deformation_cost,
            team_refinement_.trial_selection_reason.c_str(),
            team_refinement_.t_cross, team_refinement_.t_star);
      }
    }
    else if (message->handoff_contract_active && !k3_peer)
    {
      ROS_INFO("[TEAM_REFINEMENT_RESULT] drone=%d refinement_id=%lu "
               "contract_id=%lu result=NOOP_ZERO_MODIFICATION reason=%s "
               "dp=%.9f dT=%.9f improvement_authority=0 "
               "solve_ms=%.3f total_ms=%.3f",
          pp_.drone_id,
          static_cast<unsigned long>(team_refinement_.refinement_id),
          static_cast<unsigned long>(message->handoff_contract_id),
          team_refinement_solver_reason_.c_str(),
          team_result_dp_norm_, team_result_dt_norm_,
          team_refinement_solve_ms_, team_refinement_total_ms_);
    }
    else
    {
      reportTeamRefinementAudit(false);
    }
    ROS_INFO("[TEAM_REFERENCE_REALIZED_LOCAL] drone=%d reference_id=%lu "
             "topology=%d activation=%.9f duration=%.6f latency_ms=%.3f "
             "max_v=%.6f max_a=%.6f max_j=%.6f dynamic=%.6f",
        pp_.drone_id,
        static_cast<unsigned long>(message->team_reference_id), topology_kind,
        activation, realized_traj.getTotalDuration(), latency, max_velocity,
        max_acceleration, max_jerk, dynamic.min_distance);
  }

  void EGOPlannerManager::teamTrajectorySolutionCallback(
      const traj_utils::TeamTrajectorySolutionConstPtr &message)
  {
    if (!enable_team_visibility_optimizer_ || !message)
      return;
    if (message->state == traj_utils::TeamTrajectorySolution::STATE_ABORT)
    {
      if (pending_team_trajectory_.accepted &&
          pending_team_trajectory_.team_solution_id ==
              message->team_solution_id)
      {
        ROS_WARN("[team-solution-abort-recv] drone=%d team_solution_id=%lu "
                 "reason=%s action=LOCAL_ROLLING_UNCHANGED",
                 pp_.drone_id,
                 static_cast<unsigned long>(message->team_solution_id),
                 message->reason.c_str());
        pending_team_trajectory_ = PendingTeamTrajectory();
      }
      return;
    }
    if (message->state == traj_utils::TeamTrajectorySolution::STATE_PREPARE ||
        message->state == traj_utils::TeamTrajectorySolution::STATE_COMMIT)
    {
      if (pending_team_trajectory_.accepted &&
          pending_team_trajectory_.target_centered_reference &&
          pending_team_trajectory_.team_solution_id ==
              message->team_solution_id &&
          pending_team_trajectory_.coordination_generation ==
              message->coordination_generation)
      {
        const auto &schedule = pending_team_trajectory_.reference_schedule;
        int own_index = -1;
        for (size_t index = 0; index < message->drone_ids.size(); ++index)
          if (message->drone_ids[index] == pp_.drone_id)
            own_index = static_cast<int>(index);
        bool same = own_index >= 0 && message->valid &&
            message->transaction_id == schedule.team_reference_id &&
            message->target_snapshot_identity ==
                schedule.target_prediction_revision &&
            message->target_snapshot_epoch == schedule.target_snapshot_epoch &&
            message->dynamic_prediction_identity ==
                schedule.dynamic_prediction_identity &&
            message->static_map_revision == schedule.static_map_revision &&
            message->visibility_model_version ==
                schedule.visibility_model_version &&
            message->activation_time == schedule.activation_time &&
            message->hypothesis_id == schedule.hypothesis_id &&
            message->encirclement_generation ==
                schedule.encirclement_generation &&
            message->planning_generations == schedule.planning_generations &&
            message->source_candidate_ids == schedule.source_candidate_ids &&
            message->source_candidate_kinds == schedule.topology_kinds &&
            message->frontier_owner_revisions ==
                schedule.frontier_owner_revisions &&
            message->frontier_owner_trajectory_ids ==
                schedule.frontier_owner_trajectory_ids &&
            message->trajectories.size() == 3;
        if (same)
        {
          // Coordinator 必须原样回传 Local ACK 中的 MINCOTraj。直接比较消息载荷
          // 可以证明没有换轨迹，也避免把 float 系数重建成高阶多项式后用采样误差
          // 冒充 identity failure。
          const traj_utils::MINCOTraj &expected =
              pending_team_trajectory_.realized_message;
          const traj_utils::MINCOTraj &actual = message->trajectories[own_index];
          same = actual.drone_id == expected.drone_id &&
                 actual.traj_id == expected.traj_id &&
                 actual.start_time == expected.start_time &&
                 actual.order == expected.order &&
                 actual.start_p == expected.start_p &&
                 actual.start_v == expected.start_v &&
                 actual.start_a == expected.start_a &&
                 actual.end_p == expected.end_p &&
                 actual.end_v == expected.end_v &&
                 actual.end_a == expected.end_a &&
                 actual.inner_x == expected.inner_x &&
                 actual.inner_y == expected.inner_y &&
                 actual.inner_z == expected.inner_z &&
                 actual.duration == expected.duration;
        }
        if (!same)
        {
          ROS_ERROR("[TEAM_REFERENCE_COMMIT_REJECT] drone=%d reference_id=%lu "
                    "reason=LOCAL_REALIZATION_PAYLOAD_CHANGED",
              pp_.drone_id,
              static_cast<unsigned long>(message->team_solution_id));
          pending_team_trajectory_ = PendingTeamTrajectory();
          return;
        }
        if (message->state == traj_utils::TeamTrajectorySolution::STATE_PREPARE)
        {
          pending_team_trajectory_.prepare_received = true;
          pending_team_trajectory_.solution = *message;
          ROS_INFO("[TEAM_EXECUTION_PREPARE_RECV] drone=%d reference_id=%lu "
                   "activation=%.9f authority=LOCAL_REALIZATION",
              pp_.drone_id,
              static_cast<unsigned long>(message->team_solution_id),
              message->activation_time.toSec());
          return;
        }
        if (!pending_team_trajectory_.prepare_received)
        {
          ROS_ERROR("[TEAM_REFERENCE_COMMIT_REJECT] drone=%d reference_id=%lu "
                    "reason=COMMIT_WITHOUT_PREPARE",
              pp_.drone_id,
              static_cast<unsigned long>(message->team_solution_id));
          pending_team_trajectory_ = PendingTeamTrajectory();
          return;
        }
        pending_team_trajectory_.commit_received = true;
        pending_team_trajectory_.solution = *message;
        ROS_INFO("[TEAM_REFERENCE_COMMIT_RECV] drone=%d reference_id=%lu "
                 "activation=%.9f authority=LOCAL_REALIZATION",
            pp_.drone_id,
            static_cast<unsigned long>(message->team_solution_id),
            message->activation_time.toSec());
        return;
      }
      if (pending_team_trajectory_.accepted &&
          pending_team_trajectory_.team_solution_id ==
              message->team_solution_id &&
          pending_team_trajectory_.coordination_generation ==
              message->coordination_generation)
      {
        const auto &accepted=pending_team_trajectory_.solution;
        bool same=message->early_joint_primary==accepted.early_joint_primary &&
            message->transaction_id==accepted.transaction_id &&
            message->team_context_generation==accepted.team_context_generation &&
            message->team_context_snapshot==accepted.team_context_snapshot &&
            message->prediction_epoch==accepted.prediction_epoch &&
            message->target_snapshot_identity==accepted.target_snapshot_identity &&
            message->target_snapshot_epoch==accepted.target_snapshot_epoch &&
            message->dynamic_prediction_identity==accepted.dynamic_prediction_identity &&
            message->dynamic_prediction_valid_from==accepted.dynamic_prediction_valid_from &&
            message->dynamic_prediction_valid_to==accepted.dynamic_prediction_valid_to &&
            message->frontier_owner_revisions==accepted.frontier_owner_revisions &&
            message->frontier_owner_trajectory_ids==accepted.frontier_owner_trajectory_ids &&
            message->static_map_revision==accepted.static_map_revision &&
            message->visibility_model_version==accepted.visibility_model_version &&
            message->activation_time==accepted.activation_time &&
            message->hypothesis_id==accepted.hypothesis_id &&
            message->encirclement_generation==accepted.encirclement_generation &&
            message->planning_generations==accepted.planning_generations &&
            message->source_candidate_ids==accepted.source_candidate_ids &&
            message->source_candidate_kinds==accepted.source_candidate_kinds &&
            message->drone_ids==accepted.drone_ids &&
            message->optimized_yaw_valid==accepted.optimized_yaw_valid &&
            message->yaw_sample_times_uav0==accepted.yaw_sample_times_uav0 &&
            message->yaw_sample_times_uav1==accepted.yaw_sample_times_uav1 &&
            message->yaw_sample_times_uav2==accepted.yaw_sample_times_uav2 &&
            message->yaw_samples_uav0==accepted.yaw_samples_uav0 &&
            message->yaw_samples_uav1==accepted.yaw_samples_uav1 &&
            message->yaw_samples_uav2==accepted.yaw_samples_uav2 &&
            message->meaningful_binary_visibility_gain ==
                accepted.meaningful_binary_visibility_gain &&
            message->adoption_gate_reason == accepted.adoption_gate_reason &&
            message->local_counterfactual_valid ==
                accepted.local_counterfactual_valid &&
            message->local_counterfactual_owner_revisions ==
                accepted.local_counterfactual_owner_revisions &&
            message->local_counterfactual_owner_trajectory_ids ==
                accepted.local_counterfactual_owner_trajectory_ids &&
            message->trajectories.size()==accepted.trajectories.size();
        if(same) for(size_t i=0;i<message->trajectories.size();++i)
        {
          const auto &a=message->trajectories[i], &b=accepted.trajectories[i];
          same=same && a.drone_id==b.drone_id && a.traj_id==b.traj_id && a.start_time==b.start_time &&
              a.order==b.order && a.start_p==b.start_p && a.start_v==b.start_v && a.start_a==b.start_a &&
              a.end_p==b.end_p && a.end_v==b.end_v && a.end_a==b.end_a && a.inner_x==b.inner_x &&
              a.inner_y==b.inner_y && a.inner_z==b.inner_z && a.duration==b.duration;
        }
        if(!same)
        {
          ROS_ERROR("[team-solution-commit-reject] drone=%d team_solution_id=%lu reason=ACK_PAYLOAD_CHANGED",pp_.drone_id,pending_team_trajectory_.team_solution_id);
          pending_team_trajectory_=PendingTeamTrajectory();
          return;
        }
        pending_team_trajectory_.commit_received = true;
        pending_team_trajectory_.solution = *message;
        ROS_INFO("[team-solution-commit-recv] drone=%d team_solution_id=%lu "
                 "activation_time=%.9f planning_generation=%lu",
                 pp_.drone_id,
                 static_cast<unsigned long>(message->team_solution_id),
                 message->activation_time.toSec(),
                 pending_team_trajectory_.planning_generation);
      }
      return;
    }
    if (message->state != traj_utils::TeamTrajectorySolution::STATE_PROPOSAL)
      return;

    // Executable polynomials originate only from Local realization of a
    // TeamReferenceSchedule.  A direct external Team polynomial proposal
    // has no Local source revision and cannot acquire execution authority.
    ROS_WARN("[team-solution-proposal-reject] drone=%d team_solution_id=%lu "
             "reason=LEGACY_EXTERNAL_POLYNOMIAL_AUTHORITY_DISABLED",
             pp_.drone_id,
             static_cast<unsigned long>(message->team_solution_id));
    return;
  }

  void EGOPlannerManager::fillTopologyCandidateMessage(
      const CandidateResult &candidate, const double trajectory_start_time,
      const double target_state_time, const double evaluation_horizon,
      const Eigen::Vector3d &target_position,
      const Eigen::Vector3d &target_velocity,
      traj_utils::TopologyCandidate &message) const
  {
    message.candidate_id = candidate.candidate_id;
    message.encirclement_hypothesis_id =
        candidate.encirclement_hypothesis_id;
    message.encirclement_phi0 = candidate.encirclement_phi0;
    message.encirclement_generation = candidate.encirclement_generation;
    switch (candidate.kind)
    {
    case CandidateKind::SIDE_PLUS:
      message.candidate_kind = traj_utils::TopologyCandidate::KIND_SIDE_PLUS;
      message.side_sign = 1;
      break;
    case CandidateKind::SIDE_MINUS:
      message.candidate_kind = traj_utils::TopologyCandidate::KIND_SIDE_MINUS;
      message.side_sign = -1;
      break;
    default:
      message.candidate_kind = traj_utils::TopologyCandidate::KIND_NOMINAL;
      message.side_sign = 0;
      break;
    }
    switch (candidate.safety_class)
    {
    case CandidateSafetyClass::ABSOLUTE_SAFE:
      message.safety_class =
          traj_utils::TopologyCandidate::SAFETY_ABSOLUTE_SAFE;
      break;
    default:
      message.safety_class = traj_utils::TopologyCandidate::SAFETY_INVALID;
      break;
    }
    message.success = candidate.success;
    message.static_valid = candidate.static_valid;
    message.dynamics_valid = candidate.dynamics_valid;
    message.swarm_valid = candidate.swarm_valid;
    message.astar_used = candidate.astar_used;
    message.local_sfc_used = candidate.local_sfc_used;
    message.feasible_initializer_fallback =
        candidate.feasible_initializer_fallback;
    message.conflict_valid = candidate.conflict.valid;
    message.conflict_reason_mask = static_cast<uint8_t>(candidate.conflict.reason_mask);
    message.conflict_obstacle_motion = static_cast<uint8_t>(candidate.conflict.obstacle_motion);
    message.conflict_obstacle_identity = candidate.conflict.obstacle_identity;
    message.conflict_primitive_type = candidate.conflict.primitive_type;
    message.conflict_obstacle_name = candidate.conflict.obstacle_name;
    message.conflict_geometry_revision = candidate.conflict.geometry_revision;
    message.conflict_first_risk_world_time = ros::Time(candidate.conflict.first_risk_world_time);
    message.conflict_risk_interval_start = ros::Time(candidate.conflict.risk_interval_start);
    message.conflict_risk_interval_end = ros::Time(candidate.conflict.risk_interval_end);
    message.conflict_observer_position.x = candidate.conflict.observer_position.x();
    message.conflict_observer_position.y = candidate.conflict.observer_position.y();
    message.conflict_observer_position.z = candidate.conflict.observer_position.z();
    message.conflict_target_position.x = candidate.conflict.target_position.x();
    message.conflict_target_position.y = candidate.conflict.target_position.y();
    message.conflict_target_position.z = candidate.conflict.target_position.z();
    message.conflict_obstacle_position.x = candidate.conflict.obstacle_position.x();
    message.conflict_obstacle_position.y = candidate.conflict.obstacle_position.y();
    message.conflict_obstacle_position.z = candidate.conflict.obstacle_position.z();
    message.conflict_frame_forward.x = candidate.conflict.frame_forward.x();
    message.conflict_frame_forward.y = candidate.conflict.frame_forward.y();
    message.conflict_frame_forward.z = candidate.conflict.frame_forward.z();
    message.conflict_frame_right.x = candidate.conflict.frame_right.x();
    message.conflict_frame_right.y = candidate.conflict.frame_right.y();
    message.conflict_frame_right.z = candidate.conflict.frame_right.z();
    message.conflict_clearance_witness.x = candidate.conflict.clearance_witness.x();
    message.conflict_clearance_witness.y = candidate.conflict.clearance_witness.y();
    message.conflict_clearance_witness.z = candidate.conflict.clearance_witness.z();
    message.conflict_clearance = candidate.conflict.clearance;
    message.conflict_radius = candidate.conflict.radius;
    const bool los_conflict_present =
        candidate.conflict.valid &&
        (candidate.conflict.reason_mask & CONFLICT_LOS_OCCLUSION) != 0;
    message.los_conflict_valid = los_conflict_present;
    message.los_obstacle_motion =
        static_cast<uint8_t>(candidate.conflict.los_obstacle_motion);
    message.los_obstacle_identity = candidate.conflict.los_obstacle_identity;
    message.los_primitive_type = candidate.conflict.los_primitive_type;
    message.los_conflict_time =
        los_conflict_present && std::isfinite(candidate.conflict.los_conflict_time) &&
                candidate.conflict.los_conflict_time >= 0.0
            ? ros::Time(candidate.conflict.los_conflict_time)
            : ros::Time(0.0);
    message.los_first_risk_world_time =
        los_conflict_present &&
                std::isfinite(candidate.conflict.los_first_risk_world_time) &&
                candidate.conflict.los_first_risk_world_time >= 0.0
            ? ros::Time(candidate.conflict.los_first_risk_world_time)
            : ros::Time(0.0);
    message.los_obstacle_position.x = candidate.conflict.los_obstacle_position.x();
    message.los_obstacle_position.y = candidate.conflict.los_obstacle_position.y();
    message.los_obstacle_position.z = candidate.conflict.los_obstacle_position.z();
    message.los_frame_right.x = candidate.conflict.frame_right.x();
    message.los_frame_right.y = candidate.conflict.frame_right.y();
    message.los_frame_right.z = candidate.conflict.frame_right.z();
    message.joint_seed_valid = candidate.joint_seed_valid;
    message.joint_seed_static_constructible =
        candidate.joint_seed_static_constructible;
    message.joint_seed_local_sfc_valid =
        candidate.joint_seed_local_sfc_valid;
    message.joint_seed_construction_ms = candidate.joint_seed_construction_ms;
    message.joint_seed_start_time = ros::Time(trajectory_start_time);
    message.joint_seed_dynamic_clearance =
        candidate.joint_seed_dynamic_clearance;
    message.trajectory_start_time = ros::Time(trajectory_start_time);
    const poly_traj::Trajectory trajectory =
        candidate.min_jerk_opt.getTraj().getPieceNum() > 0
            ? candidate.min_jerk_opt.getTraj()
            : candidate.joint_seed.getTraj();
    message.duration = trajectory.getPieceNum() > 0
                           ? trajectory.getTotalDuration()
                           : 0.0;
    const double prediction_coverage = obj_predictor_
        ? ploy_traj_opt_->getMovingObjPredictionHorizon()
        : message.duration;
    // The trace may cover beyond the requested prefix so planners whose
    // predicted commit times differ slightly can still share one global
    // interval.  TopologyCoordinatorCore is the sole authority that clips
    // ranking to the exact requested H_eval.
    message.evaluation_horizon = std::max(
        0.0, std::min(message.duration, prediction_coverage));
    message.native_cost = candidate.optimization_cost;
    message.dynamic_clearance = candidate.risk.valid
                                    ? candidate.risk.min_distance
                                    : -1.0;
    message.swarm_clearance = std::isfinite(candidate.swarm_clearance)
                                  ? candidate.swarm_clearance
                                  : -1.0;
    message.max_velocity = candidate.max_velocity;
    message.max_acceleration = candidate.max_acceleration;
    message.max_jerk = candidate.max_jerk;
    message.tracking_score = candidate.tracking_score;
    message.radial_deviation = 0.0;
    message.angular_deviation_deg = 0.0;
    message.uav1_visibility = candidate.visibility.uav_visibility[0];
    message.uav2_visibility = candidate.visibility.uav_visibility[1];
    message.uav3_visibility = candidate.visibility.uav_visibility[2];
    message.all3_visibility = candidate.visibility.all3_visibility;
    message.atleast2_visibility = candidate.visibility.atleast2_visibility;
    message.none_visibility = candidate.visibility.none_visibility;
    message.atleast_k_visibility = candidate.visibility.atleast_k_visibility;
    message.mean_visible_count = candidate.visibility.mean_visible_count;
    message.min_pairwise_view_angle_deg =
        candidate.visibility.min_pairwise_angle_deg;
    message.mean_pairwise_view_angle_deg =
        candidate.visibility.mean_pairwise_angle_deg;
    message.diversity_score = candidate.visibility.diversity_score;
    message.team_utility = candidate.visibility.team_utility;

    if (trajectory.getPieceNum() <= 0)
      return;
    const double trajectory_duration = trajectory.getTotalDuration();
    for (const LocalSfcPlane &plane : candidate.local_sfc_planes)
    {
      if (!plane.normal.allFinite() || !plane.point.allFinite() ||
          !std::isfinite(plane.clearance) ||
          !std::isfinite(plane.active_start) ||
          !std::isfinite(plane.active_end) ||
          trajectory_duration <= 1.0e-9)
        continue;
      geometry_msgs::Vector3 normal;
      normal.x = plane.normal.x();
      normal.y = plane.normal.y();
      normal.z = plane.normal.z();
      geometry_msgs::Point point;
      point.x = plane.point.x();
      point.y = plane.point.y();
      point.z = plane.point.z();
      message.local_sfc_normals.push_back(normal);
      message.local_sfc_points.push_back(point);
      message.local_sfc_clearances.push_back(plane.clearance);
      message.local_sfc_start_ratios.push_back(std::max(
          0.0, std::min(1.0, plane.active_start / trajectory_duration)));
      message.local_sfc_end_ratios.push_back(std::max(
          0.0, std::min(1.0, plane.active_end / trajectory_duration)));
      // 本轮修复（动态 LOS world-time 语义）：把平面的时间锚定语义一并导出，
      // 否则下游只能按"进度比例"处理，动态遮挡事件会被 retime 错位。
      message.local_sfc_world_time_anchored.push_back(
          plane.world_time_anchored);
    }
    if (los_conflict_present)
    {
      const size_t los_plane_count = std::count_if(
          candidate.local_sfc_planes.begin(), candidate.local_sfc_planes.end(),
          [](const LocalSfcPlane &plane) {
            return plane.source == LocalSfcPlane::LOS_OBSERVATION_SIDE;
          });
      ROS_INFO("[los-provenance-export] drone=%d candidate=%d reason_mask=%u "
               "los_blocker=%d los_time=%.6f los_planes=%zu joint_seed=%d",
               pp_.drone_id, candidate.candidate_id,
               candidate.conflict.reason_mask,
               candidate.conflict.los_obstacle_identity,
               candidate.conflict.los_conflict_time, los_plane_count,
               static_cast<int>(candidate.joint_seed_valid));
    }
    trajectoryToMINCOMessage(trajectory, pp_.drone_id,
                             candidate.candidate_id,
                             trajectory_start_time, message.trajectory);
    double horizon = message.evaluation_horizon > 1.0e-6
                         ? message.evaluation_horizon
                         : trajectory.getTotalDuration();
    if (obj_predictor_)
      horizon = std::min(horizon,
                         ploy_traj_opt_->getMovingObjPredictionHorizon());
    (void)evaluation_horizon;
    fillCandidateBinaryVisibility(trajectory,trajectory_start_time,horizon,target_position,
        target_velocity,target_state_time,message);

    if (candidate.joint_seed_valid &&
        candidate.joint_seed.getTraj().getPieceNum() > 0)
    {
      const poly_traj::Trajectory seed = candidate.joint_seed.getTraj();
      trajectoryToMINCOMessage(seed, pp_.drone_id, candidate.candidate_id,
                               trajectory_start_time, message.joint_seed);
      traj_utils::TopologyCandidate seed_trace;
      fillCandidateBinaryVisibility(
          seed, trajectory_start_time,
          std::min(horizon, seed.getTotalDuration()), target_position,
          target_velocity, target_state_time, seed_trace);
      message.joint_seed_binary_visibility_valid =
          seed_trace.binary_visibility_valid;
      message.joint_seed_visibility_sample_offsets =
          seed_trace.visibility_sample_offsets;
      message.joint_seed_self_visibility_samples =
          seed_trace.self_visibility_samples;
      const double seed_duration=seed.getTotalDuration();
      for(const LocalSfcPlane &plane:candidate.local_sfc_planes)
      {
        if(!plane.normal.allFinite() || !plane.point.allFinite() ||
           !std::isfinite(plane.clearance) ||
           !std::isfinite(plane.active_start) ||
           !std::isfinite(plane.active_end) || seed_duration<=1.0e-9)
          continue;
        geometry_msgs::Vector3 normal;
        normal.x=plane.normal.x();normal.y=plane.normal.y();normal.z=plane.normal.z();
        geometry_msgs::Point point;
        point.x=plane.point.x();point.y=plane.point.y();point.z=plane.point.z();
        message.joint_seed_local_sfc_normals.push_back(normal);
        message.joint_seed_local_sfc_points.push_back(point);
        message.joint_seed_local_sfc_clearances.push_back(plane.clearance);
        message.joint_seed_local_sfc_start_ratios.push_back(
            std::max(0.0,std::min(1.0,plane.active_start/seed_duration)));
        message.joint_seed_local_sfc_end_ratios.push_back(
            std::max(0.0,std::min(1.0,plane.active_end/seed_duration)));
        message.joint_seed_local_sfc_world_time_anchored.push_back(
            plane.world_time_anchored);
      }
      ROS_INFO("[JOINT_SEED_EXPORTED] drone=%d candidate_id=%d kind=%s "
               "hypothesis=%d generation=%lu duration=%.6f "
               "static_constructible=%d local_sfc_valid=%d "
               "dynamic_clearance=%.6f executable_authority=0",
          pp_.drone_id, candidate.candidate_id,
          candidateKindName(candidate.kind),
          candidate.encirclement_hypothesis_id,
          static_cast<unsigned long>(candidate.encirclement_generation),
          seed_duration,
          static_cast<int>(candidate.joint_seed_static_constructible),
          static_cast<int>(candidate.joint_seed_local_sfc_valid),
          candidate.joint_seed_dynamic_clearance);
    }
  }

  void EGOPlannerManager::fillCandidateBinaryVisibility(const poly_traj::Trajectory &trajectory,
      double trajectory_start_time,double horizon,const Eigen::Vector3d &target_position,
      const Eigen::Vector3d &target_velocity,double target_state_time,
      traj_utils::TopologyCandidate &message) const
  {
    multi_uav_formation::PredictedYawState yaw_state;
    if (pp_.drone_id >= 0 && pp_.drone_id < 3 &&
        visibility_odom_[pp_.drone_id].valid)
    {
      yaw_state.valid = true;
      yaw_state.yaw = visibility_odom_[pp_.drone_id].yaw;
      yaw_state.yaw_rate = visibility_odom_[pp_.drone_id].yaw_rate;
    }
    if (yaw_state.valid)
    {
      const Eigen::Vector3d observer = trajectory.getPos(0.0);
      const Eigen::Vector3d target = target_position + target_velocity *
          std::max(0.0, trajectory_start_time - target_state_time);
      const Eigen::Vector3d to_target = target - observer;
      if (to_target.head<2>().squaredNorm() > 1.0e-12)
      {
        const double desired_yaw = std::atan2(to_target.y(), to_target.x());
        yaw_state = multi_uav_formation::advanceTargetFacingYaw(
            yaw_state, desired_yaw,
            std::max(1.0e-6, trajectory_start_time -
                                    visibility_odom_[pp_.drone_id].stamp));
      }
    }
    message.binary_visibility_valid = yaw_state.valid;
    double previous_t = 0.0;
    for (double raw_t = 0.0;; raw_t += visibility_sample_dt_)
    {
      const double t = std::min(raw_t, horizon);
      const double global_time = trajectory_start_time + t;
      const Eigen::Vector3d observer = trajectory_lifecycle::sample(trajectory,t).p;
      const Eigen::Vector3d target = target_position + target_velocity *
          std::max(0.0, global_time - target_state_time);
      const Eigen::Vector3d to_target = target - observer;
      if (yaw_state.valid && to_target.head<2>().squaredNorm() > 1.0e-12)
      {
        const double desired_yaw = std::atan2(to_target.y(), to_target.x());
        yaw_state = multi_uav_formation::advanceTargetFacingYaw(
            yaw_state, desired_yaw,
            std::max(1.0e-6, t - previous_t));
      }
      double static_clearance=std::numeric_limits<double>::infinity();
      const bool static_known=ploy_traj_opt_->queryStaticLosClearance(
          observer,target,static_clearance,nullptr,nullptr);
      bool known=yaw_state.valid && static_known && observer.allFinite() && target.allFinite();
      bool los=static_known && static_clearance>=visibility_occlusion_margin_;
      if(obj_predictor_) for(int object_id=0;object_id<obj_predictor_->getObjNums();++object_id)
      {
        if(!obj_predictor_->hasPrediction(object_id)) { known=false; break; }
        const Eigen::Vector3d center=obj_predictor_->evaluateConstVel(object_id,global_time);
        const Eigen::Vector3d scale=obj_predictor_->getObjScale(object_id);
        double dynamic_clearance;
        const bool dynamic_known=multi_uav_formation::segmentVerticalCylinderClearance(
            observer,target,center,0.5*std::max(scale.x(),scale.y()),scale.z(),dynamic_clearance);
        known=known && dynamic_known;
        los=los && dynamic_known && dynamic_clearance>=visibility_occlusion_margin_;
      }
      const auto camera=multi_uav_formation::targetInTrackingCameraFov(observer,
          Eigen::Quaterniond(Eigen::AngleAxisd(yaw_state.yaw,Eigen::Vector3d::UnitZ())),target,tracking_camera_);
      known=known && std::isfinite(camera.range) && std::isfinite(camera.horizontal_angle) && std::isfinite(camera.vertical_angle);
      message.binary_visibility_valid=message.binary_visibility_valid && known;
      const bool visible=known && los && camera.valid;
      message.visibility_sample_offsets.push_back(t);
      message.self_visibility_samples.push_back(visible ? 1 : 0);
      message.predicted_yaw_sample_offsets.push_back(t);
      message.predicted_yaw_samples.push_back(
          yaw_state.valid ? yaw_state.yaw : 0.0);
      previous_t = t;
      if (t >= horizon - 1.0e-9)
        break;
    }
  }

  double EGOPlannerManager::planningToCommitMedian() const
  {
    if (planning_to_commit_history_.empty())
      return 0.0;
    std::vector<double> values(planning_to_commit_history_.begin(),
                               planning_to_commit_history_.end());
    std::sort(values.begin(), values.end());
    const size_t middle = values.size() / 2;
    return values.size() % 2
               ? values[middle]
               : 0.5 * (values[middle - 1] + values[middle]);
  }

  void EGOPlannerManager::recordPlanningToCommitDelay(const double delay)
  {
    if (!std::isfinite(delay) || delay < 0.0)
      return;
    planning_to_commit_history_.push_back(delay);
    while (planning_to_commit_history_.size() > 8)
      planning_to_commit_history_.pop_front();
    ROS_INFO("[encirclement-commit-alignment] drone=%d "
             "PLANNING_TO_COMMIT=%.6f PLANNING_TO_COMMIT_MEDIAN=%.6f "
             "history_size=%zu",
             pp_.drone_id, delay, planningToCommitMedian(),
             planning_to_commit_history_.size());
  }

  bool EGOPlannerManager::stageTopologyCoordination(
      CandidateResult &nominal, CandidateResult &plus, CandidateResult &minus,
      const CandidateResult &local_selected,
      const Eigen::Vector3d &target_position,
      const Eigen::Vector3d &target_velocity,
      const Eigen::Vector3d &relative_tracking,
      const bool touch_goal, const double commit_head_freshness_threshold,
      const double planning_epoch)
  {
    CandidateSetOutput set;
    set.generated = true;
    set.local_candidate_id = local_selected.candidate_id;
    set.relative_tracking = relative_tracking;
    set.target_position = target_position;
    set.target_velocity = target_velocity;
    set.visibility_target_position = geometry_target_p_;
    set.visibility_target_velocity = geometry_target_v_;
    set.visibility_target_epoch = geometry_target_stamp_;
    set.planning_epoch = planning_epoch;
    set.nominal_risk = nominal.risk;
    set.nominal_obstacle_id = nominal.risk.obstacle_id;
    set.candidates = {nominal, plus, minus};
    return stageCapturedTopologyCoordination({set}, 0, touch_goal,
                                             commit_head_freshness_threshold);
  }

  bool EGOPlannerManager::stageCapturedTopologyCoordination(
      const std::vector<CandidateSetOutput> &sets,
      const int current_hypothesis_id, const bool touch_goal,
      const double commit_head_freshness_threshold)
  {
    if (!enable_joint_topology_coordination_ || sets.empty() ||
        pp_.drone_id < 0 || pp_.drone_id >= 3 || !topology_bundle_pub_)
      return false;
    const double now = ros::Time::now().toSec();
    if (pending_team_trajectory_.accepted &&
        pending_team_trajectory_.commit_received)
    {
      ROS_INFO("[team-commit-atomic] event=QUALITY_STAGING_BLOCKED_BY_COMMITTED_TAIL "
               "drone=%d team_solution_id=%lu",
          pp_.drone_id,
          static_cast<unsigned long>(pending_team_trajectory_.team_solution_id));
      return false;
    }
    if (pending_topology_.active || !backgroundTeamQualityAdmissible(now))
      return false;
    const double elapsed = std::max(0.0, now - traj_.local_traj.start_time);
    const double remaining = traj_.local_traj.traj_id > 0
                                 ? traj_.local_traj.duration - elapsed : 0.0;
    if (remaining < topology_min_active_remaining_)
      return false;

    const CandidateSetOutput *current = nullptr;
    for (const auto &set : sets)
      if (set.generated && set.hypothesis_id == current_hypothesis_id)
        current = &set;
    if (current == nullptr)
      return false;

    invalidatePendingTopology("REPLACED_BUNDLE");
    pending_topology_ = PendingTopologyState();
    if (!pending_team_trajectory_.accepted ||
        !pending_team_trajectory_.commit_received)
      pending_team_trajectory_ = PendingTeamTrajectory();
    // Snapshot the already committed, hard-safe local schedule. This state is
    // input to an optional future-tail optimization, never a local reserve.
    pending_topology_.active_generation = active_traj_generation_;
    pending_topology_.expected_trajectory_id = traj_.local_traj.traj_id;
    Eigen::Vector3d map_origin = Eigen::Vector3d::Zero();
    Eigen::Vector3d map_size = Eigen::Vector3d::Zero();
    if (grid_map_)
      grid_map_->getRegion(map_origin, map_size);
    multi_uav_formation::PlanningWorldSnapshot world_snapshot;
    world_snapshot.target_snapshot_identity =
        geometry_target_source_identity_;
    world_snapshot.target_snapshot_epoch = geometry_target_stamp_;
    world_snapshot.dynamic_prediction_identity = obj_predictor_
        ? obj_predictor_->predictionIdentity() : 0;
    world_snapshot.dynamic_prediction_valid_from = obj_predictor_
        ? obj_predictor_->commonPredictionValidFrom() : now;
    world_snapshot.dynamic_prediction_valid_to = obj_predictor_
        ? obj_predictor_->commonPredictionValidTo() : now;
    world_snapshot.static_map_revision = grid_map_
        ? stableSpatialContractIdentity(
              map_origin, map_size, grid_map_->getResolution())
        : 0;
    world_snapshot.visibility_model_version =
        multi_uav_formation::trackingCameraContractVersion(tracking_camera_);
    if (!world_snapshot.complete())
    {
      ROS_WARN_THROTTLE(1.0,
          "[planning-world-snapshot] action=SKIP_INCOMPLETE "
          "target=%lu dynamic=%lu static=%lu visibility=%lu",
          static_cast<unsigned long>(
              world_snapshot.target_snapshot_identity),
          static_cast<unsigned long>(
              world_snapshot.dynamic_prediction_identity),
          static_cast<unsigned long>(world_snapshot.static_map_revision),
          static_cast<unsigned long>(
              world_snapshot.visibility_model_version));
      return false;
    }
    pending_topology_.static_map_revision =
        world_snapshot.static_map_revision;
    pending_topology_.visibility_model_version =
        world_snapshot.visibility_model_version;
    pending_topology_.target_prediction_epoch = current->planning_epoch;
    pending_topology_.target_snapshot_identity =
        world_snapshot.target_snapshot_identity;
    pending_topology_.target_snapshot_epoch =
        world_snapshot.target_snapshot_epoch;
    pending_topology_.dynamic_prediction_identity =
        world_snapshot.dynamic_prediction_identity;
    pending_topology_.dynamic_prediction_valid_from =
        world_snapshot.dynamic_prediction_valid_from;
    pending_topology_.dynamic_prediction_valid_to =
        world_snapshot.dynamic_prediction_valid_to;
    pending_topology_.dynamic_prediction_epoch =
        pending_topology_.dynamic_prediction_valid_from;
    pending_topology_.earliest_activation = now + local_activation_margin_;
    pending_topology_.planning_generation = ++topology_planning_generation_;
    pending_topology_.transaction_id = static_cast<unsigned long>(
        std::llround(pending_topology_.earliest_activation * 1.0e6));
    pending_topology_.planning_epoch = current->planning_epoch;
    pending_topology_.bundle_stamp = now;
    pending_topology_.evaluation_start_time =
        pending_topology_.earliest_activation;
    const double default_horizon = obj_predictor_
        ? ploy_traj_opt_->getMovingObjPredictionHorizon() : 2.0;
    pending_topology_.evaluation_horizon =
        current->requested_evaluation_horizon > 1.0e-6
            ? current->requested_evaluation_horizon
            : default_horizon;
    pending_topology_.outer_loop_evaluation =
        current->outer_loop_evaluation;
    pending_topology_.execution_history_size =
        current->execution_history_size;
    pending_topology_.execution_history_median =
        current->execution_history_median;
    pending_topology_.deadline = std::min(execution_safe_until_ -
        local_activation_margin_, now +
        (enable_team_visibility_optimizer_
             ? std::max(team_coordination_timeout_, team_solution_max_age_)
             : topology_coordination_timeout_));
    pending_topology_.local_candidate_id = current->local_candidate_id;
    pending_topology_.nominal_risk = current->nominal_risk;
    pending_topology_.nominal_obstacle_id = current->nominal_obstacle_id;
    pending_topology_.touch_goal = touch_goal;
    pending_topology_.commit_head_freshness_threshold =
        commit_head_freshness_threshold;

    traj_utils::TopologyCandidateBundle bundle;
    bundle.drone_id = pp_.drone_id;
    bundle.planning_generation = pending_topology_.planning_generation;
    bundle.transaction_id = pending_topology_.transaction_id;
    bundle.planning_epoch = ros::Time(current->planning_epoch);
    bundle.bundle_stamp = ros::Time(now);
    bundle.target_prediction_epoch = ros::Time(current->planning_epoch);
    bundle.dynamic_prediction_epoch = ros::Time(
        pending_topology_.dynamic_prediction_epoch);
    bundle.target_snapshot_identity =
        pending_topology_.target_snapshot_identity;
    bundle.target_snapshot_epoch = ros::Time(
        pending_topology_.target_snapshot_epoch);
    bundle.dynamic_prediction_identity =
        pending_topology_.dynamic_prediction_identity;
    bundle.dynamic_prediction_valid_from = ros::Time(
        pending_topology_.dynamic_prediction_valid_from);
    bundle.dynamic_prediction_valid_to = ros::Time(
        pending_topology_.dynamic_prediction_valid_to);
    bundle.joint_frontier_time = ros::Time(0.0);
    bundle.static_map_revision = pending_topology_.static_map_revision;
    bundle.visibility_model_version =
        pending_topology_.visibility_model_version;
    bundle.expected_execution_generation = active_traj_generation_;
    bundle.expected_trajectory_id = traj_.local_traj.traj_id;
    bundle.boundary_source_revision = active_traj_generation_;
    bundle.boundary_source_trajectory_id = traj_.local_traj.traj_id;
    bundle.committed_coverage_start = ros::Time(traj_.local_traj.start_time);
    bundle.committed_safety_validated =
        active_execution_safety_validated_ && !lifecycle_invalidated_;
    trajectoryToMINCOMessage(traj_.local_traj.traj, pp_.drone_id,
                             traj_.local_traj.traj_id,
                             traj_.local_traj.start_time,
                             bundle.committed_trajectory);
    bundle.earliest_activation = ros::Time(
        pending_topology_.earliest_activation);
    bundle.evaluation_start_time = ros::Time(
        pending_topology_.evaluation_start_time);
    bundle.requested_evaluation_horizon =
        pending_topology_.evaluation_horizon;
    bundle.outer_loop_evaluation = pending_topology_.outer_loop_evaluation;
    bundle.execution_history_size = pending_topology_.execution_history_size;
    bundle.execution_history_median =
        pending_topology_.execution_history_median;
    bundle.target_position.x = geometry_target_p_.x();
    bundle.target_position.y = geometry_target_p_.y();
    bundle.target_position.z = geometry_target_p_.z();
    bundle.target_velocity.x = geometry_target_v_.x();
    bundle.target_velocity.y = geometry_target_v_.y();
    bundle.target_velocity.z = geometry_target_v_.z();
    bundle.local_selected_candidate_id = current->local_candidate_id;
    bundle.local_selected_hypothesis_id = current_hypothesis_id;
    int executable_count = 0;
    int joint_seed_count = 0;
    for (const auto &set : sets)
    {
      for (CandidateResult candidate : set.candidates)
      {
        if (candidate.candidate_id < 0 ||
            (candidate.min_jerk_opt.getTraj().getPieceNum() <= 0 &&
             candidate.joint_seed.getTraj().getPieceNum() <= 0))
          continue;
        const double desired_distance =
            set.relative_tracking.head<2>().norm();
        const poly_traj::Trajectory trajectory =
            candidate.min_jerk_opt.getTraj().getPieceNum() > 0
                ? candidate.min_jerk_opt.getTraj()
                : candidate.joint_seed.getTraj();
        double error_sum = 0.0;
        int samples = 0;
        for (double raw_t = 0.0;; raw_t += visibility_sample_dt_)
        {
          const double t = std::min(raw_t, trajectory.getTotalDuration());
          const Eigen::Vector3d target =
              set.target_position + set.target_velocity * t;
          error_sum += std::abs(
              (trajectory.getPos(t).head<2>() - target.head<2>()).norm() -
              desired_distance);
          ++samples;
          if (t >= trajectory.getTotalDuration() - 1.0e-9)
            break;
        }
        candidate.tracking_score =
            samples ? 1.0 / (1.0 + error_sum / samples) : 0.0;
        if (candidate.success && candidate.static_valid &&
            candidate.dynamics_valid && candidate.swarm_valid &&
            candidate.safety_class == CandidateSafetyClass::ABSOLUTE_SAFE)
          ++executable_count;
        if (candidate.joint_seed_valid &&
            candidate.joint_seed_static_constructible &&
            candidate.joint_seed_local_sfc_valid &&
            candidate.joint_seed.getTraj().getPieceNum() > 0)
          ++joint_seed_count;
        if (candidate.candidate_id == current->local_candidate_id)
        {
          pending_topology_.local_index =
              static_cast<int>(pending_topology_.candidates.size());
        }
        pending_topology_.candidates.push_back(candidate);
        traj_utils::TopologyCandidate message;
        fillTopologyCandidateMessage(
                                     candidate,
                                     pending_topology_.evaluation_start_time,
                                     set.visibility_target_epoch,
                                     pending_topology_.evaluation_horizon,
                                     set.visibility_target_position,
                                     set.visibility_target_velocity,
                                     message);
        bundle.candidates.push_back(message);
      }
    }
    const double predecessor_validated_end = std::min(
        execution_safe_until_,
        traj_.local_traj.start_time + traj_.local_traj.duration);
    bundle.validated_end = ros::Time(predecessor_validated_end);
    pending_topology_.validated_end = predecessor_validated_end;
    if (joint_seed_count <= 0 || pending_topology_.local_index < 0 ||
        bundle.candidates.empty())
    {
      pending_topology_ = PendingTopologyState();
      pending_team_trajectory_ = PendingTeamTrajectory();
      latest_topology_result_valid_ = false;
      ROS_INFO("[team-pending] event=ZERO_JOINT_SEED_DOES_NOT_ENTER_PENDING drone=%d executable_count=%d joint_seed_count=%d",
          pp_.drone_id, executable_count, joint_seed_count);
      return false;
    }
    const CandidateResult &local =
        pending_topology_.candidates[pending_topology_.local_index];
    if (!local.execution_prepared ||
        local.safety_class != CandidateSafetyClass::ABSOLUTE_SAFE)
    {
      pending_topology_ = PendingTopologyState();
      ROS_WARN("[team-transaction] event=TRANSACTION_ABORT drone=%d "
               "reason=LOCAL_RESERVE_NOT_CURRENT_REVISION_SAFE", pp_.drone_id);
      return false;
    }
    bundle.local_selected_kind = static_cast<uint8_t>(local.kind);
    latest_topology_result_valid_ = false;
    pending_topology_.active = true;
    ROS_INFO("[committed-prefix-joint] event=COMMITTED_PREFIX_SNAPSHOT_PUBLISHED drone=%d "
             "joint_request_id=%lu planning_generation=%lu "
             "boundary_source_revision=%lu boundary_source_trajectory_id=%d "
             "committed_start=%.9f committed_end=%.9f target_epoch=%.9f "
             "dynamic_valid_from=%.9f dynamic_valid_to=%.9f deadline=%.9f",
             pp_.drone_id, pending_topology_.transaction_id,
             pending_topology_.planning_generation,
             pending_topology_.active_generation,
             pending_topology_.expected_trajectory_id,
             traj_.local_traj.start_time, predecessor_validated_end,
             pending_topology_.target_snapshot_epoch,
             pending_topology_.dynamic_prediction_valid_from,
             pending_topology_.dynamic_prediction_valid_to,
             pending_topology_.deadline);
    ROS_INFO("[team-pending] event=TEAM_PENDING_START drone=%d generation=%lu expected_execution_generation=%lu expected_trajectory_id=%d timestamp=%.9f mode=EARLY_JOINT_PRIMARY",
        pp_.drone_id, pending_topology_.planning_generation,
        bundle.expected_execution_generation, bundle.expected_trajectory_id,
        now);
    // =====================================================================
    // Feedback098 — Planner-side visibility forecast.
    //
    // The Planner is the only authority on where its own UAV will be: its
    // rolling candidate is replaced every few hundred milliseconds.  Publish
    // a time-indexed margin trace for the trajectory the Planner currently
    // considers its Local baseline, sampled on an absolute world-time grid by
    // the same visibility authority the Team-SCP seed uses.  The Coordinator
    // fuses three of these into M2 and never extrapolates an old polynomial.
    // =====================================================================
    bundle.visibility_forecast.valid = false;
    {
      const auto &baseline = traj_.local_traj;
      if (baseline.traj.getPieceNum() > 0 && baseline.duration > 1.0e-6)
      {
        std::vector<PolyTrajOptimizer::VisibilityTraceSample> trace;
        if (ploy_traj_opt_->evaluateVisibilityForecast(
                baseline.traj, baseline.start_time,
                visibility_forecast_horizon_, visibility_forecast_dt_, trace) &&
            !trace.empty())
        {
          bundle.visibility_forecast.valid = true;
          bundle.visibility_forecast.forecast_identity =
              ++visibility_forecast_serial_;
          bundle.visibility_forecast.trajectory_id = baseline.traj_id;
          bundle.visibility_forecast.trajectory_generation =
              active_traj_generation_;
          bundle.visibility_forecast.candidate_id =
              pending_topology_.local_candidate_id;
          bundle.visibility_forecast.source_kind =
              active_execution_source_;
          bundle.visibility_forecast.activation_time =
              ros::Time(baseline.start_time);
          bundle.visibility_forecast.created_world_time = ros::Time(now);
          bundle.visibility_forecast.start_world_time =
              ros::Time(trace.front().world_time);
          bundle.visibility_forecast.end_world_time =
              ros::Time(trace.back().world_time);
          bundle.visibility_forecast.sample_dt = visibility_forecast_dt_;
          bundle.visibility_forecast.mathematical_end_time =
              ros::Time(baseline.start_time + baseline.duration);
          double trace_min = std::numeric_limits<double>::infinity();
          int trace_limiter = 0;
          for (const auto &entry : trace)
          {
            bundle.visibility_forecast.world_times.push_back(entry.world_time);
            bundle.visibility_forecast.margins.push_back(entry.margin);
            bundle.visibility_forecast.limiters.push_back(
                static_cast<std::uint8_t>(entry.limiter));
            if (entry.valid && entry.margin < trace_min)
            {
              trace_min = entry.margin;
              trace_limiter = entry.limiter;
            }
          }
          if (team_k3_repair_enabled_)
          {
            std::vector<PolyTrajOptimizer::BinaryVisibilityTraceSample> binary;
            if (ploy_traj_opt_->evaluateBinaryVisibilityForecast(
                    baseline.traj, baseline.start_time,
                    visibility_forecast_horizon_, visibility_forecast_dt_,
                    binary))
              for (const auto &entry : binary)
              {
                bundle.visibility_forecast.k3_world_times.push_back(
                    ros::Time(entry.world_time));
                bundle.visibility_forecast.k3_binary_visible.push_back(
                    entry.visible);
              }
          }
          ROS_INFO_THROTTLE(0.5,
              "[RELAY_FORECAST] drone=%d forecast_id=%lu traj_id=%d "
              "generation=%lu source=%s activation=%.9f window=[%.9f,%.9f] "
              "dt=%.3f samples=%zu margin_min=%.6f limiter=%d",
              pp_.drone_id,
              static_cast<unsigned long>(
                  bundle.visibility_forecast.forecast_identity),
              baseline.traj_id,
              static_cast<unsigned long>(active_traj_generation_),
              active_execution_source_.c_str(), baseline.start_time,
              trace.front().world_time, trace.back().world_time,
              visibility_forecast_dt_, trace.size(), trace_min, trace_limiter);
        }
      }
    }
    topology_bundle_pub_.publish(bundle);
    ROS_INFO("[topology-bundle] drone=%d generation=%lu hypotheses=%zu "
             "candidate_count=%zu executable_candidate_count=%d joint_seed_count=%d "
             "local_hypothesis=%d local_candidate_id=%d deadline=%.9f "
             "outer_loop=%d T_EVAL_START=%.9f H_EVAL=%.6f "
             "PLANNING_TO_COMMIT_MEDIAN=%.6f T_EXEC_HISTORY_SIZE=%u "
             "T_EXEC_HISTORY_MEDIAN=%.6f",
             pp_.drone_id, pending_topology_.planning_generation, sets.size(),
             bundle.candidates.size(), executable_count, joint_seed_count,
             current_hypothesis_id, current->local_candidate_id,
             pending_topology_.deadline,
             static_cast<int>(pending_topology_.outer_loop_evaluation),
             pending_topology_.evaluation_start_time,
             pending_topology_.evaluation_horizon,
             planningToCommitMedian(),
             pending_topology_.execution_history_size,
             pending_topology_.execution_history_median);
    return joint_seed_count >= 1;
  }

  EGOPlannerManager::TopologyProcessStatus
  EGOPlannerManager::finalizeCapturedLocal(
      const CandidateSetOutput &set, const bool touch_goal,
      const double commit_head_freshness_threshold)
  {
    std::vector<CandidateSetOutput> sets{set};
    int selected=-1;
    return finalizeCapturedCandidates(sets,touch_goal,commit_head_freshness_threshold,selected);
  }

  EGOPlannerManager::TopologyProcessStatus
  EGOPlannerManager::finalizeTopologyCandidate(
      CandidateResult candidate, const DynamicRiskInfo &nominal_risk,
      const int nominal_obstacle_id, const bool touch_goal,
      const double commit_head_freshness_threshold,
      const double planning_epoch, const bool coordinated,
      const char *selection_reason)
  {
    const auto retain_previous = [&](const char *stage,
                                     const std::string &reason) {
      const double now = ros::Time::now().toSec();
      const double elapsed = std::max(0.0, now - traj_.local_traj.start_time);
      const double remaining = std::max(0.0, traj_.local_traj.duration - elapsed);
      std::string validation_reason = "EXPIRED";
      double clearance = -1.0;
      const bool valid = active_candidate_metadata_valid_ &&
                         remaining > 1.0e-6 &&
                         validatePreviousRemainingTrajectory(
                             now, touch_goal, validation_reason, clearance);
      ROS_INFO("[previous-safe-final-revalidation] stage=%s previous_id=%d "
               "generation=%lu now=%.9f age=%.6f remaining_duration=%.6f "
               "final_valid=%d reason=%s min_dynamic_clearance=%.6f",
               stage, traj_.local_traj.traj_id, active_traj_generation_, now,
               elapsed, remaining, static_cast<int>(valid),
               validation_reason.c_str(), clearance);
      if (valid)
      {
        active_execution_source_="PERSISTENCE_FALLBACK";
        logExecutionGeometry("PERSISTENCE_REVALIDATION");
        active_execution_safety_validated_=true;
        ROS_WARN("[trajectory-fallback] stage=%s selected_type=%s "
                 "action=KEEP_PREVIOUS_SAFE reason=%s",
                 stage, candidateKindName(candidate.kind), reason.c_str());
        continous_failures_count_ = 0;
      }
      return valid;
    };

    if (!candidate.success || candidate.min_jerk_opt.getTraj().getPieceNum() <= 0)
    {
      ROS_WARN("[topology-candidate-adopt] drone=%d coordinated=%d "
               "candidate_id=%d kind=%s result=REJECT reason=NO_SOLVED_CANDIDATE",
               pp_.drone_id, static_cast<int>(coordinated), candidate.candidate_id,
               candidateKindName(candidate.kind));
      return retain_previous("TOPOLOGY_NO_CANDIDATE", "NO_SOLVED_CANDIDATE")
                 ? TopologyProcessStatus::RETAINED_PREVIOUS
                 : TopologyProcessStatus::FAILED;
    }

    candidate_ready_time_ = ros::Time::now().toSec();
    poly_traj::MinJerkOpt committed = candidate.min_jerk_opt;
    double activation = candidate.execution_prepared
        ? candidate.prediction_epoch : local_activation_time_;
    if (candidate.execution_prepared &&
        (candidate.validated_payload_hash == 0 ||
         candidate.validated_payload_hash !=
             trajectoryPayloadHash(committed.getTraj()))) {
      ROS_ERROR("[EXECUTION_IDENTITY] candidate=%d revision=%lu "
                "reason=VALIDATED_POLYNOMIAL_CHANGED",
                candidate.candidate_id, candidate.execution_revision);
      return TopologyProcessStatus::FAILED;
    }
    std::string reanchor_reason;
    if (!candidate.execution_prepared &&
        !rebuildLocalCandidateAtActivation(committed, activation,&reanchor_reason,
                             &candidate.local_sfc_planes))
      return retain_previous("TOPOLOGY_READY_TOO_LATE",reanchor_reason)
          ? TopologyProcessStatus::RETAINED_PREVIOUS : TopologyProcessStatus::FAILED;
    if (!candidate.execution_prepared)
      ++candidate.execution_revision;
    local_activation_time_ = activation;
    /* Feedback126 §7 freeze contract: the polynomial is frozen (payload hash
     * verified above).  The certificate is derived from current world facts
     * at commit time and is NOT part of the frozen payload — recompute it
     * here and adopt the fresh value.  A peer committing between freeze and
     * commit must not kill an otherwise safe candidate (liveness). */
    if (candidate.execution_prepared &&
        std::abs(activation - candidate.prediction_epoch) > 1.0e-9) {
      ROS_ERROR("[EXECUTION_IDENTITY] candidate=%d revision=%lu "
                "reason=ACTIVATION_CHANGED",
                candidate.candidate_id, candidate.execution_revision);
      return TopologyProcessStatus::FAILED;
    }
    if (candidate.execution_prepared) {
      const double commit_checked_until =
          hardCheckedUntil(committed.getTraj(), activation, touch_goal);
      if (!std::isfinite(commit_checked_until) ||
          commit_checked_until <= activation + 1.0e-6) {
        ROS_WARN("[EXECUTION_IDENTITY] candidate=%d revision=%lu "
                 "reason=CERTIFICATE_UNKNOWN_AT_COMMIT activation=%.9f",
                 candidate.candidate_id, candidate.execution_revision,
                 activation);
        return TopologyProcessStatus::FAILED;
      }
      candidate.checked_until = commit_checked_until;
      ROS_INFO("[COMMIT_SAME_REVISION] candidate=%d revision=%lu "
               "checked_until=%.9f payload_hash=%lu",
               candidate.candidate_id, candidate.execution_revision,
               candidate.checked_until, candidate.validated_payload_hash);
    }
    ConstraintPoints constraints = candidate.constraint_points;
    ploy_traj_opt_->setConstraintPoints(constraints);
    const SwarmConflictInfo swarm_before = evaluateTrajectorySwarmConflict(
        committed.getTraj(), activation, touch_goal, false);
    const SwarmConflictInfo swarm_after = evaluateTrajectorySwarmConflict(
        committed.getTraj(), activation, touch_goal, false);

    if (!validateRetimedLocalSfc(candidate,committed.getTraj()))
      return retain_previous("TOPOLOGY_LOCAL_SFC", "RETIME_LOCAL_SFC_FAIL")
          ? TopologyProcessStatus::RETAINED_PREVIOUS : TopologyProcessStatus::FAILED;
    candidate.min_jerk_opt = committed;
    candidate.constraint_points = constraints;
    candidate.display_points = committed.getInitConstraintPoints(
        ploy_traj_opt_->get_cps_num_prePiece_());
    candidate.prediction_epoch = activation;
    candidate.risk = evaluateDynamicRisk(committed.getTraj(),
                                         candidate.prediction_epoch,
                                         touch_goal);
    std::string static_reason;
    candidate.dynamics_valid = checkTrajectoryDynamics(
        committed.getTraj(), candidate.max_velocity, candidate.max_acceleration,
        candidate.max_jerk);
    candidate.static_valid = checkTrajectoryStaticSafety(
        committed.getTraj(), touch_goal, static_reason);
    // Feedback126 §3: commit re-checks the SAME unified physical swarm
    // predicate as classify and the optimizer rows.  The trigger-margin
    // (0.58 m) hard gate is removed from production.
    candidate.swarm_valid =
        !ploy_traj_opt_->swarmPhysicalViolation(committed.getTraj(),
                                                activation, touch_goal,
                                                nullptr);
    candidate.existing_checks_passed =
        candidate.dynamics_valid && candidate.static_valid && candidate.swarm_valid;
    candidate.geometry=evaluateLocalGeometry(committed.getTraj(),activation,
        activation,geometry_policy_.horizon);
    if(candidate.execution_target_context_valid)
    {
      candidate.visibility=evaluateCandidateVisibility(committed.getTraj(),activation,
          candidate.execution_target_position,candidate.execution_target_velocity,
          candidate.execution_target_state_time,candidate.encirclement_generation>0);
      traj_utils::TopologyCandidate visibility_trace;
      fillCandidateBinaryVisibility(committed.getTraj(),activation,geometry_policy_.horizon,
          candidate.execution_target_position,candidate.execution_target_velocity,
          candidate.execution_target_state_time,visibility_trace);
      candidate.binary_camera_time=visibility_trace.binary_visibility_valid?0.0:-1.0;
      if(visibility_trace.binary_visibility_valid)
        for(size_t k=1;k<visibility_trace.visibility_sample_offsets.size();++k)
          candidate.binary_camera_time+=(visibility_trace.visibility_sample_offsets[k]-
              visibility_trace.visibility_sample_offsets[k-1])*
              visibility_trace.self_visibility_samples[k-1];
    }
    // The final revision has just passed the complete hard checks above.
    // Encode executability uniformly; relative improvement is retained only
    // as a diagnostic and never grants/denies execution authority.
    candidate.safety_class =
        (candidate.success && candidate.risk.valid && candidate.existing_checks_passed)
            ? CandidateSafetyClass::ABSOLUTE_SAFE
            : CandidateSafetyClass::INVALID;
    ROS_INFO("[topology-post-check] drone=%d candidate_id=%d kind=%s "
             "coordinated=%d selection_reason=%s feature3_after_joint=1 "
             "swarm_before_valid=%d swarm_before_min=%.6f "
             "swarm_after_valid=%d swarm_after_min=%.6f dynamics_valid=%d "
             "static_valid=%d dynamic_clearance=%.6f safety_class=%s",
             pp_.drone_id, candidate.candidate_id,
             candidateKindName(candidate.kind), static_cast<int>(coordinated),
             selection_reason ? selection_reason : "LOCAL_FALLBACK",
             static_cast<int>(swarm_before.valid),
             swarm_before.valid ? swarm_before.min_distance : -1.0,
             static_cast<int>(swarm_after.valid),
             swarm_after.valid ? swarm_after.min_distance : -1.0,
             static_cast<int>(candidate.dynamics_valid),
             static_cast<int>(candidate.static_valid),
             candidate.risk.valid ? candidate.risk.min_distance : -1.0,
             candidateSafetyClassName(candidate.safety_class));
    if (candidate.safety_class != CandidateSafetyClass::ABSOLUTE_SAFE)
      return retain_previous("TOPOLOGY_RERISK_RECLASS", "NOT_ABSOLUTE_SAFE")
          ? TopologyProcessStatus::RETAINED_PREVIOUS : TopologyProcessStatus::FAILED;

    executable_candidate_seen_this_batch_=true;
    ROS_INFO("[validated-moving-coverage] event=CANDIDATE_METADATA_RECOMPUTED_AFTER_REANCHOR drone=%d candidate=%d revision=%lu activation=%.9f geometry_valid=%d visibility_valid=%d risk_valid=%d dynamics_valid=%d static_valid=%d swarm_valid=%d",
        pp_.drone_id,candidate.candidate_id,candidate.execution_revision,activation,
        int(candidate.geometry.valid),int(candidate.visibility.valid),int(candidate.risk.valid),
        int(candidate.dynamics_valid),int(candidate.static_valid),int(candidate.swarm_valid));

    const double now = ros::Time::now().toSec();
    if (!checkActiveHandoff(committed.getTraj(), activation, candidateKindName(candidate.kind)))
      return TopologyProcessStatus::FAILED;

    const auto geometry_before=evaluateLocalGeometry(traj_.local_traj.traj,traj_.local_traj.start_time,activation,geometry_policy_.horizon);
    const bool had_previous = active_candidate_metadata_valid_ &&
                              traj_.local_traj.duration > 1.0e-6;
    const double previous_start = traj_.local_traj.start_time;
    const double previous_duration = traj_.local_traj.duration;
    const unsigned long previous_generation = active_traj_generation_;
    if (!setLocalTrajFromOpt(committed, touch_goal, activation))
      return retain_previous("TOPOLOGY_COMMIT", "POINTS_TO_CHECK_FAILED")
                 ? TopologyProcessStatus::RETAINED_PREVIOUS
                 : TopologyProcessStatus::FAILED;
    active_validation_revision_ = candidate.execution_revision;
    if (candidate.execution_prepared &&
        trajectoryPayloadHash(traj_.local_traj.traj) !=
            candidate.validated_payload_hash)
      ROS_ERROR("[EXECUTION_IDENTITY] candidate=%d revision=%lu "
                "reason=COMMITTED_POLYNOMIAL_MISMATCH",
                candidate.candidate_id, candidate.execution_revision);
    else
      ROS_INFO("[EXECUTION_IDENTITY] candidate=%d PRECHECK_REVISION=%lu "
               "QUALITY_REVISION=%lu COMMIT_REVISION=%lu "
               "payload_hash=%lu checked_until=%.9f",
               candidate.candidate_id, candidate.execution_revision,
               candidate.execution_revision, candidate.execution_revision,
               static_cast<unsigned long>(
                   trajectoryPayloadHash(traj_.local_traj.traj)),
               execution_safe_until_);
    recordPlanningToCommitDelay(traj_.local_traj.start_time - planning_epoch);
    if (had_previous)
      ROS_INFO("[active-traj-lifecycle] event=SUPERSEDED "
               "traj_identity=start=%.9f,generation=%lu "
               "source=topology_candidate_selection start_time=%.9f "
               "duration=%.6f active=0 reason=new_candidate_selected",
               previous_start, previous_generation, previous_start,
               previous_duration);
    active_local_sfc_planes_ = candidate.local_sfc_planes;
    ROS_INFO("[local-successor] event=LOCAL_SUCCESSOR_READY drone=%d trajectory_id=%d candidate_ready_time=%.9f commit_time=%.9f activation_time=%.9f",
        pp_.drone_id, traj_.local_traj.traj_id, candidate_ready_time_, commit_time_, activation);
    active_execution_source_=candidate.feasible_initializer_fallback ? "FEASIBLE_FALLBACK" : candidateKindName(candidate.kind);
    active_execution_view_generation_=candidate.encirclement_generation;
    active_execution_hypothesis_=candidate.encirclement_hypothesis_id;
    logExecutionGeometry("LOCAL_COMMIT",&geometry_before);
    active_candidate_kind_ = candidate.kind;
    active_candidate_obstacle_id_ =
        candidate.kind == CandidateKind::NOMINAL ? -1 : nominal_obstacle_id;
    active_candidate_metadata_valid_ = true;
    last_candidate_kind_ = candidate.kind;
    last_candidate_obstacle_id_ = nominal_obstacle_id;
    ROS_INFO("[planner-traj-selected] drone_id=%d candidate_type=%s "
             "candidate_id=%d obstacle_id=%d next_trajectory_id=%d "
             "selected_time=%.9f duration=%.6f "
             "predicted_dynamic_clearance=%.6f coordinated=%d",
             pp_.drone_id, candidateKindName(candidate.kind),
             candidate.candidate_id, active_candidate_obstacle_id_,
             traj_.local_traj.traj_id, now, committed.getTraj().getTotalDuration(),
             candidate.risk.valid ? candidate.risk.min_distance : -1.0,
             static_cast<int>(coordinated));

    if (candidate.safety_class == CandidateSafetyClass::ABSOLUTE_SAFE)
    {
      accepted_state_cache_traj_ = committed.getTraj();
      accepted_state_cache_time_ = traj_.local_traj.start_time;
      accepted_state_cache_kind_ = candidate.kind;
      accepted_state_cache_safety_class_ = candidate.safety_class;
      accepted_state_cache_obstacle_id_ = active_candidate_obstacle_id_;
      accepted_state_cache_generation_ = active_traj_generation_;
      ++accepted_state_cache_id_;
      accepted_state_cache_valid_ =
          accepted_state_cache_traj_.getPieceNum() > 0 &&
          std::isfinite(accepted_state_cache_traj_.getTotalDuration());
      ROS_INFO("[accepted-safe-cache] stored=%d traj_id=%lu type=%s class=%s "
               "generation=%lu obstacle_id=%d piece_num=%d duration=%.6f "
               "min_dynamic_clearance=%.6f source=topology_finalize",
               static_cast<int>(accepted_state_cache_valid_),
               accepted_state_cache_id_, candidateKindName(candidate.kind),
               candidateSafetyClassName(candidate.safety_class),
               accepted_state_cache_generation_, accepted_state_cache_obstacle_id_,
               accepted_state_cache_traj_.getPieceNum(),
               accepted_state_cache_traj_.getTotalDuration(),
               candidate.risk.valid ? candidate.risk.min_distance : -1.0);
    }
    displayTrajectoryAuthority(committed.getTraj(), activation,
                               execution_safe_until_, 0);
    continous_failures_count_ = 0;
    return TopologyProcessStatus::COMMITTED;
  }

  EGOPlannerManager::TopologyProcessStatus
  EGOPlannerManager::processPendingTopologyCoordination()
  {
    if (!pending_topology_.active)
      return TopologyProcessStatus::NONE;
    const double now = ros::Time::now().toSec();
    if (pending_topology_.active_generation != active_traj_generation_ &&
        !pending_team_trajectory_.accepted)
    {
      invalidatePendingTopology("STALE_LOCAL_GENERATION");
      return TopologyProcessStatus::NONE;
    }
    if (pending_team_trajectory_.accepted &&
        pending_team_trajectory_.commit_received)
    {
      const int source_index =
          pending_team_trajectory_.source_candidate_index;
      if (pending_team_trajectory_.planning_generation !=
              pending_topology_.planning_generation ||
          source_index < 0 ||
          source_index >= static_cast<int>(pending_topology_.candidates.size()))
      {
        publishTeamCommitRevoke("INTERNAL_IDENTITY_MISMATCH");
        ROS_ERROR("[team-solution-commit-reject] drone=%d "
                  "team_solution_id=%lu reason=INTERNAL_IDENTITY_MISMATCH",
                  pp_.drone_id,
                  pending_team_trajectory_.team_solution_id);
      }
      else if (now > pending_team_trajectory_.activation_time + 0.02)
      {
        publishTeamCommitRevoke("ACTIVATION_TIME_EXPIRED");
        ROS_ERROR("[team-solution-commit-reject] drone=%d "
                  "team_solution_id=%lu reason=ACTIVATION_TIME_EXPIRED "
                  "now=%.9f activation_time=%.9f",
                  pp_.drone_id,
                  pending_team_trajectory_.team_solution_id, now,
                  pending_team_trajectory_.activation_time);
      }
      else
      {
        const PendingTopologyState pending = pending_topology_;
        const PendingTeamTrajectory team = pending_team_trajectory_;
        const CandidateResult &source = pending.candidates[source_index];
        const bool had_previous = active_candidate_metadata_valid_ &&
                                  traj_.local_traj.duration > 1.0e-6;
        const double previous_start = traj_.local_traj.start_time;
        const double previous_duration = traj_.local_traj.duration;
        const unsigned long previous_generation = active_traj_generation_;
        const auto geometry_before=evaluateLocalGeometry(traj_.local_traj.traj,traj_.local_traj.start_time,
            team.activation_time,geometry_policy_.horizon);

        // =================================================================
        // Feedback096 adoption re-check.
        //
        // The solver result was produced from a frozen snapshot.  Local rolling
        // has kept running since, so the result is re-checked here against the
        // genuinely current anchor, the latest peers and the latest world
        // snapshot.  A changed trajectory generation is NOT by itself a reason
        // to reject: only physical incompatibility, a vanished requirement, or
        // a missed activation may discard the result.
        // =================================================================
        // The relay-specific re-check applies ONLY to a handoff-contract
        // refinement.  Ordinary Team reference commits keep their original
        // transaction semantics untouched.
        // ---- Feedback096 relay-specific adoption re-check ----------------
        // Applies ONLY to a handoff-contract refinement.  An ordinary
        // Team reference commit keeps its original transaction semantics.
        const bool relay_refinement = team.handoff_refinement;
        if (relay_refinement)
        {
          const double recheck_now = ros::Time::now().toSec();
          const double updates_during =
              static_cast<double>(latest_team_prediction_snapshot_id_ -
                                  team_refinement_.start_snapshot_id);
          if (team_refinement_open_)
          {
            team_refinement_.finish_snapshot_id =
                latest_team_prediction_snapshot_id_;
            team_refinement_.updates_during =
                latest_team_prediction_snapshot_id_ -
                team_refinement_.start_snapshot_id;
            team_ref_updates_samples_.push_back(updates_during);
          }

          // ---- Check 4: is there still time for PREPARE/READY/COMMIT? -------
          // Evaluated first because it is the cheapest and it is a hard
          // scheduling fact, not a geometric judgement.
          if (recheck_now > team.activation_time - local_activation_margin_)
          {
            ++team_ref_too_late_;
            publishTeamCommitRevoke("TEAM_RESULT_TOO_LATE");
            ROS_WARN("[TEAM_REFINEMENT_RESULT] drone=%d refinement_id=%lu "
                     "contract_id=%lu team_solution_id=%lu "
                     "result=TOO_LATE now=%.9f activation=%.9f "
                     "updates_during=%.0f solve_ms=%.3f total_ms=%.3f",
                pp_.drone_id,
                static_cast<unsigned long>(team_refinement_.refinement_id),
                static_cast<unsigned long>(team_refinement_.contract_id),
                static_cast<unsigned long>(team.team_solution_id), recheck_now,
                team.activation_time, updates_during,
                team_refinement_solve_ms_, team_refinement_total_ms_);
            pending_team_trajectory_ = PendingTeamTrajectory();
            team_refinement_open_ = false;
            return TopologyProcessStatus::NONE;
          }

          // ---- Check 1: activation / predecessor compatibility --------------
          // Uses the same handoff tolerance the executor uses, plus the declared
          // predecessor identity, so the Team proposal can only be accepted when
          // it can actually attach to the current execution chain.
          {
            std::string anchor_reason;
            // Re-anchor the candidate onto the real activation boundary exactly
            // as an ordinary Local successor would be, then validate continuity
            // against the authoritative predecessor with the executor tolerance.
            poly_traj::MinJerkOpt anchor_probe = team.trajectory;
            double anchor_activation = team.activation_time;
            const bool anchor_ok =
                rebuildLocalCandidateAtActivation(anchor_probe, anchor_activation,
                                    &anchor_reason, nullptr, false) &&
                checkActiveHandoff(anchor_probe.getTraj(), anchor_activation,
                                   "JOINT_TEAM_SOLUTION", true);
            if (!anchor_ok)
            {
              ++team_ref_stale_anchor_;
              publishTeamCommitRevoke("TEAM_RESULT_STALE_ANCHOR");
              ROS_WARN("[TEAM_REFINEMENT_RESULT] drone=%d refinement_id=%lu "
                       "contract_id=%lu team_solution_id=%lu "
                       "result=STALE_ANCHOR reason=%s "
                       "now=%.9f activation=%.9f updates_during=%.0f "
                       "solve_ms=%.3f total_ms=%.3f",
                  pp_.drone_id,
                  static_cast<unsigned long>(team_refinement_.refinement_id),
                  static_cast<unsigned long>(team_refinement_.contract_id),
                  static_cast<unsigned long>(team.team_solution_id),
                  anchor_reason.c_str(), recheck_now, team.activation_time,
                  updates_during, team_refinement_solve_ms_,
                  team_refinement_total_ms_);
              pending_team_trajectory_ = PendingTeamTrajectory();
              team_refinement_open_ = false;
              return TopologyProcessStatus::NONE;
            }
          }

          // ---- Check 2: is the requirement still present? -------------------
          // The refinement exists to remove an impending M2 deficit inside the
          // contracted critical window.  If the requirement has already been
          // withdrawn (the coordinator released the contract by raising M2 back
          // above the release margin) then the result is simply no longer needed;
          // that is a normal no-op, not a solver failure.
          const bool refinement_still_required =
              !latest_handoff_contract_seen_ ||
              (latest_handoff_contract_active_ &&
               latest_handoff_contract_id_ == team_refinement_.contract_id);
          if (!refinement_still_required)
          {
            ++team_ref_no_longer_needed_;
            publishTeamCommitRevoke("TEAM_RESULT_NO_LONGER_NEEDED");
            ROS_INFO("[TEAM_REFINEMENT_RESULT] drone=%d refinement_id=%lu "
                     "contract_id=%lu team_solution_id=%lu "
                     "result=NO_LONGER_NEEDED "
                     "updates_during=%.0f solve_ms=%.3f total_ms=%.3f",
                pp_.drone_id,
                static_cast<unsigned long>(team_refinement_.refinement_id),
                static_cast<unsigned long>(team_refinement_.contract_id),
                static_cast<unsigned long>(team.team_solution_id),
                updates_during, team_refinement_solve_ms_,
                team_refinement_total_ms_);
            pending_team_trajectory_ = PendingTeamTrajectory();
            team_refinement_open_ = false;
            return TopologyProcessStatus::NONE;
          }

          // ---- Check 3: recombine with the latest peers ---------------------
          // The refined polynomial is re-validated against the CURRENT swarm
          // safety, static, dynamic, dynamics and topology evidence rather than
          // the snapshot it was solved from.  Peer trajectory identity changes
          // are handled here; they are not a rejection reason by themselves.
          {
            std::string recheck_reason;
            double recheck_max_v = 0.0, recheck_max_a = 0.0, recheck_max_j = 0.0;
            const auto recheck_dynamic = evaluateDynamicRisk(
                team.trajectory.getTraj(), team.activation_time, false);
            if (!checkTrajectoryDynamics(team.trajectory.getTraj(),
                                         recheck_max_v, recheck_max_a,
                                         recheck_max_j) ||
                !checkTrajectoryStaticSafety(team.trajectory.getTraj(), false,
                                             recheck_reason) ||
                !recheck_dynamic.valid || recheck_dynamic.hard_collision ||
                !checkTrajectorySwarmSafety(team.trajectory.getTraj(),
                                            team.activation_time, false,
                                            recheck_reason) ||
                !validateExecutionTrajectory(team.trajectory.getTraj(),
                                             team.activation_time, false,
                                             recheck_reason, &team.solution))
            {
              ++team_ref_latest_recheck_reject_;
              publishTeamCommitRevoke("TEAM_RESULT_LATEST_RECHECK_REJECT");
              ROS_WARN("[TEAM_REFINEMENT_RESULT] drone=%d refinement_id=%lu "
                       "contract_id=%lu team_solution_id=%lu "
                       "result=LATEST_RECHECK_REJECT reason=%s "
                       "updates_during=%.0f solve_ms=%.3f total_ms=%.3f",
                  pp_.drone_id,
                  static_cast<unsigned long>(team_refinement_.refinement_id),
                  static_cast<unsigned long>(team_refinement_.contract_id),
                  static_cast<unsigned long>(team.team_solution_id),
                  recheck_reason.c_str(), updates_during,
                  team_refinement_solve_ms_, team_refinement_total_ms_);
              pending_team_trajectory_ = PendingTeamTrajectory();
              team_refinement_open_ = false;
              return TopologyProcessStatus::NONE;
            }
          }
        }

        if (!setLocalTrajFromOpt(team.trajectory, pending.touch_goal,
                                 team.activation_time, &team.solution))
        {
          publishTeamCommitRevoke("FINAL_PREFLIGHT_FAILED");
          ROS_ERROR("[team-solution-commit-reject] drone=%d "
                    "team_solution_id=%lu reason=POINTS_TO_CHECK_FAILED",
                    pp_.drone_id, team.team_solution_id);
          pending_team_trajectory_ = PendingTeamTrajectory();
        }
        else
        {
          active_execution_source_="EARLY_JOINT_PRIMARY";
          active_execution_team_id_=team.team_solution_id;
          active_reference_id_=team.team_solution_id;
          active_reference_reason_="JOINT_COMMIT";
          // Feedback098: the certificate asserts a MEASURABLE Team benefit.
          // A zero-modification result never reaches here (handoff_refinement
          // is only set for real refinements), and even then the improvement
          // must be real.
          const bool measurable_improvement =
              team_refinement_open_ &&
              std::isfinite(team_result_margin_after_) &&
              std::isfinite(team_refinement_.m2_before) &&
              team_result_margin_after_ >=
                  team_refinement_.required_margin - 1.0e-9 &&
              team_result_margin_after_ >
                  team_refinement_.m2_before + 1.0e-9;
          if (team_refinement_open_)
          {
            ++team_ref_adopted_;
            team_refinement_open_ = false;
            team_improvement_certificate_.valid = measurable_improvement;
            if (!measurable_improvement)
            {
              ++team_ref_zero_modification_;
              ROS_WARN("[TEAM_REFINEMENT_RESULT] drone=%d refinement_id=%lu "
                       "contract_id=%lu team_solution_id=%lu "
                       "result=NOOP_NO_MEASURABLE_IMPROVEMENT "
                       "m2_before=%.6f m2_after=%.6f required=%.6f "
                       "improvement_authority=0",
                  pp_.drone_id,
                  static_cast<unsigned long>(team_refinement_.refinement_id),
                  static_cast<unsigned long>(team_refinement_.contract_id),
                  static_cast<unsigned long>(team.team_solution_id),
                  team_refinement_.m2_before, team_result_margin_after_,
                  team_refinement_.required_margin);
            }
            team_improvement_certificate_.team_solution_id =
                team.team_solution_id;
            team_improvement_certificate_.trajectory_id =
                traj_.local_traj.traj_id;
            team_improvement_certificate_.activation = team.activation_time;
            team_improvement_certificate_.critical_begin =
                team_refinement_.critical_begin;
            team_improvement_certificate_.critical_end =
                team_refinement_.critical_end;
            team_improvement_certificate_.m2_before =
                team_refinement_.m2_before;
            team_improvement_certificate_.required_margin =
                team_refinement_.required_margin;
            team_improvement_certificate_.world_revision =
                latest_team_prediction_snapshot_id_;
            ROS_INFO("[TEAM_REFINEMENT_RESULT] drone=%d refinement_id=%lu "
                     "contract_id=%lu team_solution_id=%lu "
                     "result=ADOPTED trajectory_id=%d "
                     "updates_during=%lu solve_ms=%.3f total_ms=%.3f",
                pp_.drone_id,
                static_cast<unsigned long>(team_refinement_.refinement_id),
                // Feedback099: this line used to print team_solution_id under
                // the contract_id label, so an adopted handoff refinement was
                // reported with an id from a different id space and could not
                // be matched against its TEAM_REFINEMENT_START/SOLVE lines.
                static_cast<unsigned long>(team_refinement_.contract_id),
                static_cast<unsigned long>(team.team_solution_id),
                traj_.local_traj.traj_id,
                static_cast<unsigned long>(team_refinement_.updates_during),
                team_refinement_solve_ms_, team_refinement_total_ms_);
            logRelayLatency("ADOPTED");
          }
          active_execution_view_generation_=source.encirclement_generation;
          active_execution_hypothesis_=source.encirclement_hypothesis_id;
          logExecutionGeometry("JOINT_COMMIT",&geometry_before);
          const double executable_hold = std::min(
              team_min_execution_time_,
              std::max(0.0, traj_.local_traj.duration - 0.02));
          team_execution_hold_solution_id_ = team.team_solution_id;
          team_execution_hold_traj_id_ = traj_.local_traj.traj_id;
          team_execution_hold_until_ =
              team.activation_time + executable_hold;
          invalidatePendingTopology("TEAM_COMMIT_ADOPTED");
          recordPlanningToCommitDelay(team.activation_time -
                                      pending.planning_epoch);
          if (had_previous)
            ROS_INFO("[active-traj-lifecycle] event=SCHEDULED_SUPERSEDE "
                     "previous_start=%.9f previous_generation=%lu "
                     "previous_duration=%.6f team_solution_id=%lu "
                     "activation_time=%.9f",
                     previous_start, previous_generation, previous_duration,
                     team.team_solution_id, team.activation_time);
          active_candidate_kind_ = source.kind;
          active_candidate_obstacle_id_ =
              source.kind == CandidateKind::NOMINAL
                  ? -1
                  : pending.nominal_obstacle_id;
          active_candidate_metadata_valid_ = true;
          active_execution_touch_goal_ = pending.touch_goal;
          last_candidate_kind_ = source.kind;
          last_candidate_obstacle_id_ = pending.nominal_obstacle_id;
          accepted_state_cache_traj_ = team.trajectory.getTraj();
          accepted_state_cache_time_ = team.activation_time;
          accepted_state_cache_kind_ = source.kind;
          accepted_state_cache_safety_class_ =
              CandidateSafetyClass::ABSOLUTE_SAFE;
          accepted_state_cache_obstacle_id_ = active_candidate_obstacle_id_;
          accepted_state_cache_generation_ = active_traj_generation_;
          ++accepted_state_cache_id_;
          accepted_state_cache_valid_ = true;
          displayTrajectoryAuthority(team.trajectory.getTraj(),
                                     team.activation_time,
                                     execution_safe_until_, 0);
          continous_failures_count_ = 0;
          ROS_INFO("[team-transaction] event=TRANSACTION_COMMIT drone=%d "
                   "transaction_id=%lu TRANSACTION_FINAL_SOURCE=%s "
                   "execution_decision_count=1 execution_commit_count=1 "
                   "boundary_changed_inside_valid_transaction=0",
                   pp_.drone_id,
                   static_cast<unsigned long>(team.solution.transaction_id),
                   "EARLY_JOINT_PRIMARY");
          ROS_INFO("[team-solution-adopt] drone=%d team_solution_id=%lu "
                   "coordination_generation=%lu planning_generation=%lu "
                   "source_candidate_id=%d kind=%s activation_time=%.9f "
                   "trajectory_id=%d duration=%.6f common_activation=1 "
                   "stale_result_commit=0",
                   pp_.drone_id, team.team_solution_id,
                   team.coordination_generation,
                   team.planning_generation, team.source_candidate_id,
                   candidateKindName(source.kind), team.activation_time,
                   traj_.local_traj.traj_id,
                   traj_.local_traj.duration);
          ROS_INFO("[EARLY_JOINT_ACTIVATED] drone=%d team_solution_id=%lu "
                   "team_context_generation=%lu trajectory_id=%d "
                   "activation_time=%.9f optimized_yaw_valid=%d",
              pp_.drone_id,team.team_solution_id,
              static_cast<unsigned long>(team.solution.team_context_generation),
              traj_.local_traj.traj_id,team.activation_time,
              static_cast<int>(team.solution.optimized_yaw_valid));
          ROS_INFO("[team-solution-sample-hold] drone=%d "
                   "team_solution_id=%lu trajectory_id=%d "
                   "activation_time=%.9f hold_until=%.9f "
                   "minimum_execution_time=%.6f "
                   "normal_updates_only=1 safety_preemption_enabled=1",
                   pp_.drone_id, team.team_solution_id,
                   traj_.local_traj.traj_id, team.activation_time,
                   team_execution_hold_until_, executable_hold);
          return TopologyProcessStatus::COMMITTED;
        }
      }
    }
    std::string selection_reason = "COORDINATOR_TIMEOUT_LOCAL_FALLBACK";
    bool matching_result = false;
    if (latest_topology_result_valid_)
    {
      const double result_age = now - latest_topology_result_.snapshot_stamp.toSec();
      if (std::isfinite(result_age) && result_age >= -0.1 &&
          result_age <= topology_result_max_age_)
      {
        for (size_t index = 0; index < latest_topology_result_.drone_ids.size();
             ++index)
        {
          if (latest_topology_result_.drone_ids[index] != pp_.drone_id)
            continue;
          if (latest_topology_result_.planning_generations[index] !=
              pending_topology_.planning_generation)
          {
            ROS_INFO_THROTTLE(
                0.2,
                "[topology-result-reject] drone=%d result_generation=%lu "
                "current_generation=%lu reason=STALE_GENERATION",
                pp_.drone_id,
                static_cast<unsigned long>(
                    latest_topology_result_.planning_generations[index]),
                pending_topology_.planning_generation);
            break;
          }
          selection_reason = latest_topology_result_.selection_reason;
          matching_result = true;
          break;
        }
      }
    }
    if (!matching_result && now < pending_topology_.deadline)
      return TopologyProcessStatus::PENDING;

    const char *discard_reason = matching_result
        ? selection_reason.c_str() : "JOINT_TIMEOUT";
    ROS_INFO("[committed-prefix-joint] event=JOINT_FAILURE_LOCAL_NOOP "
             "drone=%d joint_request_id=%lu reason=%s local_commit_changed=0",
             pp_.drone_id, pending_topology_.transaction_id, discard_reason);
    invalidatePendingTopology(discard_reason);
    return TopologyProcessStatus::NONE;
  }

  bool EGOPlannerManager::buildWarmStartFromAccepted(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
      const Eigen::Vector3d &local_target_vel, poly_traj::MinJerkOpt &warm_mjo,
      double &age, double &remaining_duration, int &old_piece_num,
      int &new_piece_num, std::string &reason) const
  {
    age = -1.0;
    remaining_duration = -1.0;
    old_piece_num = 0;
    new_piece_num = 0;
    ROS_INFO_THROTTLE(1.0, "[warm-start-debug] enabled=%d cache_valid=%d cache_id=%lu "
                      "cache_class=%s cache_generation=%lu cache_pieces=%d "
                      "cache_duration=%.6f cache_time=%.6f now=%.6f",
                      1,
                      static_cast<int>(accepted_state_cache_valid_),
                      accepted_state_cache_id_,
                      candidateSafetyClassName(accepted_state_cache_safety_class_),
                      accepted_state_cache_generation_,
                      accepted_state_cache_traj_.getPieceNum(),
                      accepted_state_cache_traj_.getTotalDuration(),
                      accepted_state_cache_time_, ros::Time::now().toSec());
    if (!accepted_state_cache_valid_)
    {
      reason = "NO_ACCEPTED_STATE";
      return false;
    }
    if (accepted_state_cache_safety_class_ !=
        CandidateSafetyClass::ABSOLUTE_SAFE)
    {
      reason = "CACHE_NOT_ABSOLUTE_SAFE";
      return false;
    }
    old_piece_num = accepted_state_cache_traj_.getPieceNum();
    if (old_piece_num <= 0 || !std::isfinite(accepted_state_cache_time_))
    {
      reason = "INVALID_ACCEPTED_STATE";
      return false;
    }
    age = local_activation_time_ - accepted_state_cache_time_;
    if (!std::isfinite(age) || age < -1.0e-3 || age > 8.0 ||
        accepted_state_cache_traj_.getTotalDuration() <= 1.0e-3)
    {
      reason = age > 8.0 ? "EXPIRED" : "INVALID_CACHE_AGE";
      return false;
    }

    poly_traj::Trajectory remaining;
    if (!sliceTrajectory(accepted_state_cache_traj_, std::max(0.0, age), remaining))
    {
      reason = "EXPIRED";
      return false;
    }
    remaining_duration = remaining.getTotalDuration();
    new_piece_num = remaining.getPieceNum();
    if (new_piece_num <= 0 || remaining_duration <= 0.50 ||
        !std::isfinite(remaining_duration))
    {
      reason = "HORIZON_TOO_SHORT";
      return false;
    }
    if (!canReuseRemainingSuffix(start_pt, start_vel, start_acc,
                                 local_target_pt, local_target_vel,
                                 remaining, reason))
    {
      ROS_INFO("[trajectory-warm-start] cache_id=%lu action=FRESH_MOVING_INIT reason=%s "
               "remaining=%.6f target_distance=%.6f",
               accepted_state_cache_id_, reason.c_str(), remaining_duration,
               (local_target_pt - start_pt).norm());
      return false;
    }
    // Measure the accepted slice before replacing its head with the
    // authoritative current state.  This is diagnostic only.
    const double head_p_error = (start_pt - remaining.getJuncPos(0)).norm();
    const double head_v_error = (start_vel - remaining.getJuncVel(0)).norm();
    const double head_a_error = (start_acc - remaining.getJuncAcc(0)).norm();
    ROS_INFO("[trajectory-warm-start-head] cache_id=%lu age=%.6f remaining=%.6f head_p_error=%.9f head_v_error=%.9f head_a_error=%.9f",
             accepted_state_cache_id_, age, remaining_duration,
             head_p_error, head_v_error, head_a_error);

    // Preserve the remaining piece partition and physical durations.  The
    // authoritative planning-start P/V/A is still supplied by the FSM; only
    // the interior geometry and timing come from the accepted trajectory.
    Eigen::Matrix3d head_state, tail_state;
    head_state << start_pt, start_vel, start_acc;
    tail_state << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();
    const Eigen::VectorXd durations = remaining.getDurations();
    Eigen::MatrixXd positions = remaining.getPositions();
    if (positions.cols() != new_piece_num + 1)
    {
      reason = "INVALID_REMAINING_GEOMETRY";
      return false;
    }
    Eigen::MatrixXd inner_points = positions.block(0, 1, 3, new_piece_num - 1);
    warm_mjo.reset(head_state, tail_state, new_piece_num);
    warm_mjo.generate(inner_points, durations);
    const poly_traj::Trajectory generated = warm_mjo.getTraj();
    if (generated.getPieceNum() != new_piece_num ||
        !std::isfinite(generated.getTotalDuration()) ||
        generated.getTotalDuration() <= 1.0e-3)
    {
      reason = "WARM_GENERATE_INVALID";
      return false;
    }
    ++init_reuse_count_;
    ROS_INFO("[trajectory-warm-start] cache_id=%lu event=INITIALIZER_REUSED remaining=%.6f",
             accepted_state_cache_id_, remaining_duration);
    reason = "USED_PREVIOUS_ABSOLUTE_SAFE";
    return true;
  }

  bool EGOPlannerManager::validatePreviousRemainingTrajectory(
      const double prediction_epoch, const bool touch_goal, std::string &reason,
      double &min_dynamic_clearance) const
  {
    min_dynamic_clearance = -1.0;
    if (!active_candidate_metadata_valid_ || traj_.local_traj.start_time < 1.0e9 ||
        traj_.local_traj.duration <= 1.0e-6 || traj_.local_traj.traj.getPieceNum() <= 0)
    {
      reason = "NO_PREVIOUS_ACTIVE";
      ROS_WARN("[active-traj-lifecycle] event=INVALIDATED traj_identity=start=%.9f,generation=%lu source=persistence_validator start_time=%.9f duration=%.6f active=0 reason=%s",
               traj_.local_traj.start_time, active_traj_generation_,
               traj_.local_traj.start_time, traj_.local_traj.duration, reason.c_str());
      return false;
    }

    const double elapsed = prediction_epoch - traj_.local_traj.start_time;
    poly_traj::Trajectory remaining;
    if (!sliceTrajectory(traj_.local_traj.traj, elapsed, remaining))
    {
      reason = "EXPIRED";
      ROS_WARN("[active-traj-lifecycle] event=EXPIRED traj_identity=start=%.9f,generation=%lu source=persistence_validator start_time=%.9f duration=%.6f active=0 reason=%s",
               traj_.local_traj.start_time, active_traj_generation_,
               traj_.local_traj.start_time, traj_.local_traj.duration, reason.c_str());
      return false;
    }

    double max_vel = 0.0, max_acc = 0.0, max_jerk = 0.0;
    if (!checkTrajectoryDynamics(remaining, max_vel, max_acc, max_jerk))
    {
      std::ostringstream oss;
      oss << "DYNAMICS vel=" << max_vel << "/" << pp_.max_vel_
          << " acc=" << max_acc << "/" << pp_.max_acc_
          << " jerk=" << max_jerk << "/" << pp_.max_jer_;
      reason = oss.str();
      ROS_WARN("[active-traj-lifecycle] event=INVALIDATED traj_identity=start=%.9f,generation=%lu source=persistence_validator start_time=%.9f duration=%.6f active=0 reason=%s",
               traj_.local_traj.start_time, active_traj_generation_,
               traj_.local_traj.start_time, traj_.local_traj.duration, reason.c_str());
      return false;
    }

    if (!checkTrajectoryStaticSafety(remaining, touch_goal, reason))
    {
      if (reason.empty())
        reason = "STATIC";
      ROS_WARN("[active-traj-lifecycle] event=INVALIDATED traj_identity=start=%.9f,generation=%lu source=persistence_validator start_time=%.9f duration=%.6f active=0 reason=%s",
               traj_.local_traj.start_time, active_traj_generation_,
               traj_.local_traj.start_time, traj_.local_traj.duration, reason.c_str());
      return false;
    }

    const DynamicRiskInfo risk = evaluateDynamicRisk(remaining, prediction_epoch, touch_goal);
    if (!risk.valid)
    {
      reason = "DYNAMIC_RISK_INVALID";
      ROS_WARN("[active-traj-lifecycle] event=INVALIDATED traj_identity=start=%.9f,generation=%lu source=persistence_validator start_time=%.9f duration=%.6f active=0 reason=%s",
               traj_.local_traj.start_time, active_traj_generation_,
               traj_.local_traj.start_time, traj_.local_traj.duration, reason.c_str());
      return false;
    }
    else
    {
      min_dynamic_clearance = risk.min_distance;
      // Persistence remains executable when it violates only the preferred
      // 1.1 m margin. Physical body overlap is still a hard rejection.
      if (risk.hard_collision)
      {
        std::ostringstream oss;
        oss << "HARD_DYNAMIC_COLLISION clearance=" << risk.min_distance
            << " hard_clearance=" << risk.hard_clearance;
        reason = oss.str();
        ROS_WARN("[active-traj-lifecycle] event=INVALIDATED traj_identity=start=%.9f,generation=%lu source=persistence_validator start_time=%.9f duration=%.6f active=0 reason=%s",
                 traj_.local_traj.start_time, active_traj_generation_,
                 traj_.local_traj.start_time, traj_.local_traj.duration, reason.c_str());
        return false;
      }
    }

    if (!checkTrajectorySwarmSafety(remaining, prediction_epoch, touch_goal, reason))
    {
      if (reason.empty())
        reason = "SWARM";
      ROS_WARN("[active-traj-lifecycle] event=INVALIDATED traj_identity=start=%.9f,generation=%lu source=persistence_validator start_time=%.9f duration=%.6f active=0 reason=%s",
               traj_.local_traj.start_time, active_traj_generation_,
               traj_.local_traj.start_time, traj_.local_traj.duration, reason.c_str());
      return false;
    }
    if (!validateExecutionTrajectory(remaining,prediction_epoch,touch_goal,reason)) return false;
    ROS_INFO("[active-traj-lifecycle] event=VALIDATED traj_identity=start=%.9f,generation=%lu source=persistence_validator start_time=%.9f duration=%.6f active=1 reason=remaining_safe",
             traj_.local_traj.start_time, active_traj_generation_,
             traj_.local_traj.start_time, traj_.local_traj.duration);
    return true;
  }

  EGOPlannerManager::DynamicRiskInfo EGOPlannerManager::evaluateDynamicRisk(
      const poly_traj::Trajectory &traj, const double prediction_epoch,
      const bool touch_goal) const
  {
    DynamicRiskInfo risk;
    if (traj.getPieceNum()<=0) return risk;
    if (!obj_predictor_ || obj_predictor_->getObjNums()<=0)
    { risk.valid=true; return risk; }
    // 【本轮修复】原先只要有一个配置槽位还没有预测就整体判为 invalid
    // （reason=DYNAMIC_PREDICTION_INVALID），导致"场景里动态障碍数少于
    // 配置槽位数"或"预测短暂缺失"时，所有候选轨迹乃至首条轨迹都被拒绝，
    // 无人机根本无法起飞。改为：无预测的槽位直接跳过，风险只由有预测的
    // 障碍决定。这样"没有障碍被评估"与"障碍不可见"不再互相污染。
    int predicted_object_count = 0;
    for (int id=0;id<obj_predictor_->getObjNums();++id)
      if (obj_predictor_->hasPrediction(id)) ++predicted_object_count;
    if (predicted_object_count <= 0)
      return risk;

    // Dynamic optimization and hard admission share the same bounded rolling
    // authority interval.  A local target is not mission termination; its
    // polynomial suffix remains forecast-only until a later rolling solve.
    double check_end = executionAuthorityHorizon(
        traj.getTotalDuration(), touch_goal);
    check_end=std::max(0.0,check_end);
    const double trigger_distance = ploy_traj_opt_->getMovingObjClearance() +
                                    candidate_trigger_margin_;
    risk.trigger_distance = trigger_distance;

    const double epsilon = 1.0e-6;
    for (double t = 0.0;; t += risk_sample_dt_)
    {
      const double sample_time = std::min(t, check_end);
      const Eigen::Vector3d trajectory_position = traj.getPos(sample_time);
      const Eigen::Vector3d trajectory_velocity = traj.getVel(sample_time);
      const double query_time = prediction_epoch + sample_time;
      for (int id = 0; id < obj_predictor_->getObjNums(); ++id)
      {
        if (!obj_predictor_->hasPrediction(id))
          continue;
        const Eigen::Vector3d obstacle_position =
            obj_predictor_->evaluateConstVel(id, query_time);
        if (!obstacle_position.allFinite())
          return DynamicRiskInfo();
        const Eigen::Vector3d obstacle_velocity =
            obj_predictor_->evaluateConstVelVelocity(id, query_time);
        const Eigen::Vector3d relative_position = trajectory_position - obstacle_position;
        const double distance = relative_position.norm();
        const double hard_clearance = dynamicHardClearanceForObject(id);
        if (std::isfinite(hard_clearance))
        {
          risk.hard_clearance = std::min(risk.hard_clearance,
                                         distance - hard_clearance);
          if (distance < hard_clearance)
            risk.hard_collision = true;
        }
        if (!risk.valid || distance < risk.min_distance)
        {
          risk.valid = true;
          risk.obstacle_id = id;
          risk.min_distance = distance;
          risk.conflict_time = sample_time;
          risk.prediction_query_time = query_time;
          risk.trajectory_position = trajectory_position;
          risk.obstacle_position = obstacle_position;
          risk.relative_radial_velocity =
              relative_position.dot(trajectory_velocity - obstacle_velocity) /
              std::max(distance, epsilon);
        }
      }
      if (sample_time >= check_end - epsilon)
        break;
    }

    if (risk.valid)
      risk.triggered = risk.min_distance < risk.trigger_distance;
    return risk;
  }

  double EGOPlannerManager::dynamicHardClearanceForObject(const int object_id) const
  {
    if (!obj_predictor_ || object_id < 0 ||
        object_id >= obj_predictor_->getObjNums())
      return std::numeric_limits<double>::quiet_NaN();
    const Eigen::Vector3d scale = obj_predictor_->getObjScale(object_id);
    if (!scale.allFinite() || scale.x() <= 0.0 || scale.y() <= 0.0)
      return std::numeric_limits<double>::quiet_NaN();
    // Marker scale is the cylinder diameter; combine its live radius with
    // the UAV footprint radius used by the inflated grid map.
    return dynamic_body_radius_ + 0.5 * std::max(scale.x(), scale.y());
  }

  void EGOPlannerManager::visibilityOdomCallback(
      const nav_msgs::OdometryConstPtr &msg, const int drone_id)
  {
    if (!msg || drone_id < 0 || drone_id >= 3)
      return;
    VisibilityOdomState state;
    state.header_stamp_valid = !msg->header.stamp.isZero();
    state.stamp = msg->header.stamp.isZero()
                      ? ros::Time::now().toSec()
                      : msg->header.stamp.toSec();
    state.position << msg->pose.pose.position.x,
        msg->pose.pose.position.y, msg->pose.pose.position.z;
    state.velocity << msg->twist.twist.linear.x,
        msg->twist.twist.linear.y, msg->twist.twist.linear.z;
    const auto &q = msg->pose.pose.orientation;
    Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
    if (orientation.coeffs().allFinite() && orientation.norm() > 1.0e-12)
    {
      orientation.normalize();
      state.yaw = std::atan2(
          2.0 * (orientation.w() * orientation.z() +
                 orientation.x() * orientation.y()),
          1.0 - 2.0 * (orientation.y() * orientation.y() +
                       orientation.z() * orientation.z()));
      state.yaw_rate = msg->twist.twist.angular.z;
    }
    state.valid = std::isfinite(state.stamp) && state.position.allFinite() &&
                  state.velocity.allFinite() && std::isfinite(state.yaw) &&
                  std::isfinite(state.yaw_rate);
    visibility_odom_[drone_id] = state;
    if (drone_id == pp_.drone_id)
      checkK3EscalationRecovery();
  }

  EGOPlannerManager::CandidateVisibilityReport
  EGOPlannerManager::evaluateCandidateVisibility(
      const poly_traj::Trajectory &candidate_traj,
      const double candidate_start_time,
      const Eigen::Vector3d &target_position,
      const Eigen::Vector3d &target_velocity,
      const double target_prediction_epoch,
      const bool encirclement_active) const
  {
    CandidateVisibilityReport report;
    if (!grid_map_ || !obj_predictor_ ||
        pp_.drone_id < 0 || pp_.drone_id >= 3 ||
        candidate_traj.getPieceNum() <= 0 ||
        !std::isfinite(candidate_start_time) ||
        !std::isfinite(target_prediction_epoch) ||
        !target_position.allFinite() || !target_velocity.allFinite())
      return report;

    double horizon = candidate_traj.getTotalDuration();
    horizon = std::min(horizon, ploy_traj_opt_->getMovingObjPredictionHorizon());
    if (!std::isfinite(horizon) || horizon < 1.0e-6)
      return report;

    // 与 evaluateDynamicRisk 同理：缺预测的槽位跳过，不再整体作废可见性报告。

    const auto static_los_clear = [&](const Eigen::Vector3d &observer,
                                      const Eigen::Vector3d &target,
                                      bool &query_valid) {
      double clearance = std::numeric_limits<double>::infinity();
      query_valid = ploy_traj_opt_->queryStaticLosClearance(
          observer, target, clearance, nullptr, nullptr);
      return query_valid && clearance > visibility_occlusion_margin_;
    };

    const auto dynamic_los_clear = [&](const Eigen::Vector3d &observer,
                                       const Eigen::Vector3d &target,
                                       const double query_time,
                                       bool &query_valid) {
      for (int id = 0; id < obj_predictor_->getObjNums(); ++id)
      {
        const Eigen::Vector3d center = obj_predictor_->evaluateConstVel(id, query_time);
        const Eigen::Vector3d scale = obj_predictor_->getObjScale(id);
        if (!center.allFinite() || !scale.allFinite() ||
            scale.x() <= 0.0 || scale.y() <= 0.0 || scale.z() <= 0.0)
        {
          query_valid = false;
          return false;
        }
        const double radius = 0.5 * std::max(scale.x(), scale.y()) +
                              visibility_occlusion_margin_;
        if (segmentIntersectsVerticalCylinder(observer, target, center,
                                              radius, scale.z()))
          return false;
      }
      return true;
    };

    std::array<int, 3> visible_count{{0, 0, 0}};
    std::array<int, 3> current_loss{{0, 0, 0}};
    std::array<int, 3> max_loss{{0, 0, 0}};
    int all3_count = 0;
    int atleast2_count = 0;
    int atleast_k_count = 0;
    int none_count = 0;
    int total_visible_count = 0;
    double min_pairwise_angle = std::numeric_limits<double>::infinity();
    double pairwise_angle_sum = 0.0;
    int pairwise_angle_count = 0;
    double diversity_score_sum = 0.0;
    bool query_valid = true;
    const double epsilon = 1.0e-9;
    for (double t = 0.0;; t += visibility_sample_dt_)
    {
      const double sample_t = std::min(t, horizon);
      const double absolute_time = candidate_start_time + sample_t;
      // The candidate, peers, target and moving obstacles are all sampled at
      // the same world time.  This matches fillCandidateBinaryVisibility().
      const Eigen::Vector3d target = target_position + target_velocity *
          std::max(0.0, absolute_time - target_prediction_epoch);
      std::array<bool, 3> visible{{false, false, false}};
      std::array<Eigen::Vector3d, 3> observers;
      for (int drone_id = 0; drone_id < 3; ++drone_id)
      {
        Eigen::Vector3d observer;
        bool observer_valid = false;
        if (drone_id == pp_.drone_id)
        {
          observer = candidate_traj.getPos(sample_t);
          observer_valid = observer.allFinite();
        }
        else if (drone_id < static_cast<int>(traj_.swarm_traj.size()))
        {
          const LocalTrajData &other = traj_.swarm_traj[drone_id].executionAt(absolute_time);
          const double other_t = absolute_time - other.start_time;
          if (other.drone_id == drone_id && other.traj.getPieceNum() > 0 &&
              other_t >= 0.0 && other_t <= other.duration)
          {
            observer = other.traj.getPos(other_t);
            observer_valid = observer.allFinite();
          }
        }
        if (!observer_valid && visibility_odom_[drone_id].valid &&
            candidate_start_time - visibility_odom_[drone_id].stamp <= 1.0)
        {
          const double odom_dt = std::max(0.0, absolute_time -
                                                   visibility_odom_[drone_id].stamp);
          observer = visibility_odom_[drone_id].position +
                     visibility_odom_[drone_id].velocity * odom_dt;
          observer_valid = observer.allFinite();
        }
        if (!observer_valid)
        {
          query_valid = false;
          break;
        }
        observers[drone_id] = observer;

        const double target_distance =
            (target.head<2>() - observer.head<2>()).norm();
        const bool range_ok = target_distance >= visibility_min_target_distance_ &&
                              target_distance <= visibility_max_target_distance_;
        const bool static_ok = range_ok &&
                               static_los_clear(observer, target, query_valid);
        const bool dynamic_ok = static_ok && dynamic_los_clear(
                                                 observer, target, absolute_time,
                                                 query_valid);
        visible[drone_id] = range_ok && static_ok && dynamic_ok;
        if (!query_valid)
          break;
      }
      if (!query_valid)
        return CandidateVisibilityReport();

      const int team_visible = static_cast<int>(visible[0]) +
                               static_cast<int>(visible[1]) +
                               static_cast<int>(visible[2]);
      double sample_min_pairwise_angle = std::numeric_limits<double>::infinity();
      for (int first = 0; first < 3; ++first)
      {
        const Eigen::Vector3d first_view = observers[first] - target;
        const double first_norm = first_view.norm();
        if (first_norm <= 1.0e-6)
          return CandidateVisibilityReport();
        for (int second = first + 1; second < 3; ++second)
        {
          const Eigen::Vector3d second_view = observers[second] - target;
          const double second_norm = second_view.norm();
          if (second_norm <= 1.0e-6)
            return CandidateVisibilityReport();
          const double cosine = std::min(
              1.0, std::max(-1.0, first_view.dot(second_view) /
                                      (first_norm * second_norm)));
          const double angle = std::acos(cosine);
          sample_min_pairwise_angle = std::min(
              sample_min_pairwise_angle, angle);
          min_pairwise_angle = std::min(min_pairwise_angle, angle);
          pairwise_angle_sum += angle;
          ++pairwise_angle_count;
        }
      }
      const double preferred_separation_rad =
          (encirclement_active
               ? encirclement_spread_saturation_angle_deg_
               : team_visibility_preferred_separation_deg_) *
          M_PI / 180.0;
      diversity_score_sum += std::min(
          1.0, sample_min_pairwise_angle /
                   std::max(1.0e-6, preferred_separation_rad));
      ++report.sample_count;
      all3_count += static_cast<int>(team_visible == 3);
      atleast2_count += static_cast<int>(team_visible >= 2);
      atleast_k_count += static_cast<int>(team_visible >= 2);
      none_count += static_cast<int>(team_visible == 0);
      total_visible_count += team_visible;
      for (int drone_id = 0; drone_id < 3; ++drone_id)
      {
        visible_count[drone_id] += static_cast<int>(visible[drone_id]);
        if (visible[drone_id])
          current_loss[drone_id] = 0;
        else
          max_loss[drone_id] = std::max(max_loss[drone_id],
                                        ++current_loss[drone_id]);
      }
      if (sample_t >= horizon - epsilon)
        break;
    }

    if (report.sample_count <= 0)
      return CandidateVisibilityReport();
    const double inv_count = 1.0 / static_cast<double>(report.sample_count);
    for (int drone_id = 0; drone_id < 3; ++drone_id)
      report.uav_visibility[drone_id] = visible_count[drone_id] * inv_count;
    report.all3_visibility = all3_count * inv_count;
    report.atleast2_visibility = atleast2_count * inv_count;
    report.none_visibility = none_count * inv_count;
    report.mean_visible_count = total_visible_count * inv_count;
    report.atleast_k_visibility = atleast_k_count * inv_count;
    report.min_pairwise_angle_deg =
        min_pairwise_angle * 180.0 / M_PI;
    report.mean_pairwise_angle_deg = pairwise_angle_count > 0
        ? pairwise_angle_sum / pairwise_angle_count * 180.0 / M_PI
        : 0.0;
    report.diversity_score = diversity_score_sum * inv_count;
    report.team_utility = report.atleast2_visibility +
                          report.mean_visible_count / 3.0;
    report.min_uav_visibility = *std::min_element(
        report.uav_visibility.begin(), report.uav_visibility.end());
    report.max_loss_duration = *std::max_element(max_loss.begin(), max_loss.end()) *
                               visibility_sample_dt_;
    report.valid = true;
    return report;
  }

  LocalVisibilitySample
  EGOPlannerManager::evaluateLocalVisibilitySampleAtWorldTime(
      const Eigen::Vector3d &observer, const double observer_yaw,
      const Eigen::Vector3d &target, const double world_time) const
  {
    LocalVisibilitySample s;
    if (!observer.allFinite() || !target.allFinite())
      return s;
    // ---- range(任务跟踪带,与 evaluateCandidateVisibility 同源)------------
    const double range2d = (target.head<2>() - observer.head<2>()).norm();
    if (!std::isfinite(range2d) || range2d <= 1.0e-9)
      return s;
    s.range_visible = range2d >= visibility_min_target_distance_ &&
                      range2d <= visibility_max_target_distance_;
    // ---- static LOS(唯一查询权威:queryStaticLosClearance)----------------
    double static_clearance = std::numeric_limits<double>::infinity();
    const bool static_known = ploy_traj_opt_->queryStaticLosClearance(
        observer, target, static_clearance, nullptr, nullptr);
    s.static_los_visible =
        static_known && static_clearance >= visibility_occlusion_margin_;
    std::array<multi_uav_formation::VisibilityRiskComponent, 4> components;
    components[0].limiter =
        multi_uav_formation::ContinuousVisibilitySample::LIMITER_STATIC;
    components[1].limiter =
        multi_uav_formation::ContinuousVisibilitySample::LIMITER_DYNAMIC_LOS;
    components[2].limiter =
        multi_uav_formation::ContinuousVisibilitySample::LIMITER_FOV;
    components[3].limiter =
        multi_uav_formation::ContinuousVisibilitySample::LIMITER_RANGE;
    if (static_known)
    {
      // 与 visibilitySampleAt 同一契约:query 权威可能用 +∞ 表示"附近无遮挡",
      // directionalClearanceRisk 只接受有限输入 —— 无穷距离即零风险。
      if (std::isfinite(static_clearance))
      {
        const auto risk = multi_uav_formation::directionalClearanceRisk(
            static_clearance, visibility_occlusion_margin_,
            visibility_occlusion_margin_ +
                ploy_traj_opt_->visibilityLosSmoothing());
        if (risk.valid)
        {
          components[0].valid = true;
          components[0].risk = risk.value;
        }
      }
      else
      {
        components[0].valid = true;
        components[0].risk = 0.0;
      }
    }
    // ---- dynamic LOS(同一 predictor + cylinder 查询权威)------------------
    // 与 static 同一初始化语义:乐观起点,仅在查询失败或真实遮挡时清除。
    s.dynamic_los_visible = true;
    bool dynamic_known = true;
    if (obj_predictor_)
    {
      for (int id = 0; id < obj_predictor_->getObjNums(); ++id)
      {
        if (!obj_predictor_->hasPrediction(id))
        {
          dynamic_known = false;
          break;
        }
        const Eigen::Vector3d center =
            obj_predictor_->evaluateConstVel(id, world_time);
        const Eigen::Vector3d scale = obj_predictor_->getObjScale(id);
        double clearance = 0.0;
        if (!center.allFinite() || !scale.allFinite() ||
            scale.x() <= 0.0 || scale.y() <= 0.0 || scale.z() <= 0.0 ||
            !multi_uav_formation::segmentVerticalCylinderClearance(
                observer, target, center,
                0.5 * std::max(scale.x(), scale.y()), scale.z(), clearance))
        {
          dynamic_known = false;
          break;
        }
        if (clearance < visibility_occlusion_margin_)
          s.dynamic_los_visible = false;
        // +∞ clearance = 段不进入该障碍竖直平板:零风险,不喂给有限输入契约。
        if (std::isfinite(clearance))
        {
          const auto risk = multi_uav_formation::directionalClearanceRisk(
              clearance, visibility_occlusion_margin_,
              visibility_occlusion_margin_ +
                  ploy_traj_opt_->visibilityLosSmoothing());
          if (!risk.valid)
          {
            dynamic_known = false;
            break;
          }
          if (risk.value > components[1].risk + 1.0e-12)
            components[1].risk = risk.value;
        }
      }
    }
    else
      dynamic_known = false;
    components[1].valid = dynamic_known;
    s.dynamic_los_visible = s.dynamic_los_visible && dynamic_known;
    // ---- FOV / camera range(与 fillCandidateBinaryVisibility 同源)--------
    const multi_uav_formation::TrackingCameraContract camera = tracking_camera_;
    const Eigen::Quaterniond world_from_body(
        Eigen::AngleAxisd(observer_yaw, Eigen::Vector3d::UnitZ()));
    const auto fov = multi_uav_formation::targetInTrackingCameraFov(
        observer, world_from_body, target, camera);
    s.fov_visible = fov.valid;
    const auto fov_risk = multi_uav_formation::trackingCameraFovDirectionalRisk(
        observer, observer_yaw, target, camera);
    if (fov_risk.valid)
    {
      components[2].valid = true;
      components[2].risk = fov_risk.risk;
    }
    // ---- range risk(camera 带,与 visibilitySampleAt 同源)----------------
    const double range = (target - observer).norm();
    const auto near_risk = multi_uav_formation::directionalClearanceRisk(
        range - camera.min_range, 0.0,
        ploy_traj_opt_->visibilityRangeSmoothing());
    const auto far_risk = multi_uav_formation::directionalClearanceRisk(
        camera.max_range - range, 0.0,
        ploy_traj_opt_->visibilityRangeSmoothing());
    if (near_risk.valid && far_risk.valid)
    {
      const auto &risk = near_risk.value >= far_risk.value ? near_risk : far_risk;
      components[3].valid = true;
      components[3].risk = risk.value;
    }
    // margin 有效性与"目标是否在 FOV 内"(二值)是两个量:fov_risk.valid 是
    // 风险分量可计算性,binary fov_visible 才决定 binary_visible。
    s.valid = static_known && dynamic_known && fov_risk.valid &&
              components[0].valid && components[1].valid &&
              components[2].valid && components[3].valid;
    multi_uav_formation::composeVisibilityMargin(components, s.continuous);
    s.binary_visible = s.valid && s.range_visible && s.static_los_visible &&
                       s.dynamic_los_visible && s.fov_visible;
    return s;
  }

  LocalK3WindowMetrics
  EGOPlannerManager::evaluateLocalK3WindowMetrics(
      const poly_traj::Trajectory &self_traj, const double self_start_world,
      const Eigen::Vector3d &target_at_epoch,
      const Eigen::Vector3d &target_velocity, const double target_epoch,
      const double window_begin, const double window_end,
      const double required_margin) const
  {
    LocalK3WindowMetrics m;
    if (self_traj.getPieceNum() <= 0 || !std::isfinite(self_start_world) ||
        !std::isfinite(window_begin) || !std::isfinite(window_end) ||
        window_end <= window_begin || !std::isfinite(required_margin))
      return m;
    const double trajectory_end = self_start_world + self_traj.getTotalDuration();
    // K3 candidate, baseline and K2 comparisons must cover the same requested
    // world-time window.  Silently clipping a short candidate would compare a
    // different sample set and could report artificial C3/D3 progress.
    if (window_begin < self_start_world - 1.0e-6 ||
        window_end > trajectory_end + 1.0e-6)
      return m;
    const double scan_begin = window_begin;
    const double scan_end = window_end;
    // 每 UAV 一份推进中的目标朝向 yaw 状态(与 fillCandidateBinaryVisibility
    // 同一推进模型)。没有该 world-time yaw 的 UAV 不计为可见，避免把缺失 FOV
    // 状态乐观地当作 K2/C3 可见。
    std::array<multi_uav_formation::PredictedYawState, 3> yaw{};
    std::array<bool, 3> yaw_valid{{false, false, false}};
    std::array<double, 3> yaw_last_update{{0.0, 0.0, 0.0}};
    double previous_world = scan_begin;
    for (int u = 0; u < 3; ++u)
    {
      if (visibility_odom_[u].valid &&
          visibility_odom_[u].stamp <= scan_begin + 1.0e-9)
      {
        yaw[u].valid = true;
        yaw[u].yaw = visibility_odom_[u].yaw;
        yaw[u].yaw_rate = visibility_odom_[u].yaw_rate;
        yaw_valid[u] = true;
        yaw_last_update[u] = visibility_odom_[u].stamp;
      }
    }
    const double dt = visibility_sample_dt_;
    const double first = std::ceil(scan_begin / dt) * dt;
    for (double world_time = first; world_time <= scan_end + 1.0e-9;
         world_time += dt)
    {
      const double self_t = world_time - self_start_world;
      if (self_t < -1.0e-9 || self_t > self_traj.getTotalDuration() + 1.0e-9)
      {
        previous_world = world_time;
        continue;
      }
      const Eigen::Vector3d target = localK3TargetAtWorldTime(
          target_at_epoch, target_velocity, target_epoch, world_time);
      std::array<bool, 3> visible{{false, false, false}};
      bool sample_known = true;
      for (int u = 0; u < 3 && sample_known; ++u)
      {
        Eigen::Vector3d observer;
        bool observer_valid = false;
        if (u == pp_.drone_id)
        {
          observer = self_traj.getPos(std::max(0.0, self_t));
          observer_valid = observer.allFinite();
        }
        else if (u < static_cast<int>(traj_.swarm_traj.size()))
        {
          const LocalTrajData &other =
              traj_.swarm_traj[u].executionAt(world_time);
          const double other_t = world_time - other.start_time;
          if (other.drone_id == u && other.traj.getPieceNum() > 0 &&
              other_t >= 0.0 && other_t <= other.duration)
          {
            observer = other.traj.getPos(other_t);
            observer_valid = observer.allFinite();
          }
        }
        if (!observer_valid && visibility_odom_[u].valid &&
            world_time - visibility_odom_[u].stamp <= 1.0)
        {
          observer = visibility_odom_[u].position +
                     visibility_odom_[u].velocity *
                         std::max(0.0, world_time - visibility_odom_[u].stamp);
          observer_valid = observer.allFinite();
        }
        if (!observer_valid)
        {
          sample_known = false;
          break;
        }
        if (yaw_valid[u] && world_time < yaw_last_update[u] - 1.0e-9)
          yaw_valid[u] = false;
        if (!yaw_valid[u] && visibility_odom_[u].valid &&
            world_time + 1.0e-9 >= visibility_odom_[u].stamp)
        {
          yaw[u].valid = true;
          yaw[u].yaw = visibility_odom_[u].yaw;
          yaw[u].yaw_rate = visibility_odom_[u].yaw_rate;
          yaw_valid[u] = true;
          yaw_last_update[u] = visibility_odom_[u].stamp;
        }
        if (u == pp_.drone_id && !yaw_valid[u])
        {
          sample_known = false;
          break;
        }
        if (yaw_valid[u] &&
            (target.head<2>() - observer.head<2>()).squaredNorm() > 1.0e-12)
        {
          yaw[u] = multi_uav_formation::advanceTargetFacingYaw(
              yaw[u], std::atan2((target - observer).y(),
                                 (target - observer).x()),
              std::max(0.0, world_time - yaw_last_update[u]));
          yaw_last_update[u] = world_time;
        }
        const auto sample = evaluateLocalVisibilitySampleAtWorldTime(
            observer, yaw_valid[u] ? yaw[u].yaw : 0.0, target, world_time);
        if (!sample.valid)
        {
          sample_known = false;
          break;
        }
        visible[u] = yaw_valid[u] && sample.binary_visible;
        if (u == pp_.drone_id && sample.continuous.margin_valid &&
            std::isfinite(sample.continuous.margin))
        {
          ++m.margin_sample_count;
          m.d3 += std::max(0.0, required_margin - sample.continuous.margin) *
                 std::max(0.0, world_time - previous_world);
        }
      }
      if (sample_known)
      {
        ++m.sample_count;
        const int count = static_cast<int>(visible[0]) +
                          static_cast<int>(visible[1]) +
                          static_cast<int>(visible[2]);
        m.c3 += count == 3 ? 1.0 : 0.0;
        m.k2 += count >= 2 ? 1.0 : 0.0;
      }
      previous_world = world_time;
    }
    if (m.sample_count > 0)
    {
      const double inv = 1.0 / static_cast<double>(m.sample_count);
      m.c3 *= inv;
      m.k2 *= inv;
    }
    m.valid = m.sample_count > 0 && m.margin_sample_count > 0;
    return m;
  }

  bool EGOPlannerManager::checkTrajectoryDynamics(const poly_traj::Trajectory &traj,
                                                  double &max_vel, double &max_acc,
                                                  double &max_jer) const
  {
    max_vel = traj.getMaxVelRate();
    max_acc = traj.getMaxAccRate();
    // Exact per-piece analytic maximum of |jerk|.  Uniform sampling misses
    // interior jerk peaks; the same payload then oscillates between
    // optimizer-side acceptance (sampled) and this kernel's rejection until
    // both sides evaluate the same exact quantity.
    max_jer = traj.getMaxJerRate();

    const double tol = 1.0 + std::max(0.0, pp_.feasibility_tolerance_);
    const bool vel_ok = pp_.max_vel_ <= 0.0 || max_vel <= pp_.max_vel_ * tol;
    const bool acc_ok = pp_.max_acc_ <= 0.0 || max_acc <= pp_.max_acc_ * tol;
    const bool jer_ok = pp_.max_jer_ <= 0.0 || max_jer <= pp_.max_jer_ * tol;
    return vel_ok && acc_ok && jer_ok;
  }

  bool EGOPlannerManager::checkTrajectoryStaticSafety(const poly_traj::Trajectory &traj,
                                                      const bool touch_goal,
                                                      std::string &reason) const
  {
    if (!grid_map_)
      return true;

    const double duration = traj.getTotalDuration();
    const double dt = std::max(
        0.02, std::min(0.05, grid_map_->getResolution() / std::max(0.1, pp_.max_vel_)));

    const double virtual_ceil_height = grid_map_->getVirtualCeilHeight();
    if (virtual_ceil_height > -0.5)
    {
      for (double t = 0.0; t < duration + 1.0e-6; t += dt)
      {
        const double tt = std::min(t, duration);
        const Eigen::Vector3d pos = traj.getPos(tt);
        if (!pos.allFinite() || pos(2) >= virtual_ceil_height)
        {
          std::ostringstream oss;
          oss << "virtual ceiling t=" << tt << " z=" << pos(2)
              << "/" << virtual_ceil_height;
          reason = oss.str();
          return false;
        }
      }

    }

    const double check_end = executionAuthorityHorizon(duration, touch_goal);
    for (double t = 0.0; t < check_end + 1.0e-6; t += dt)
    {
      const double tt = std::min(t, check_end);
      const Eigen::Vector3d pos = traj.getPos(tt);
      const int occupancy = pos.allFinite() ? grid_map_->getInflateOccupancy(pos) : -1;
      if (occupancy != 0)
      {
        std::ostringstream oss;
        oss << "static map t=" << tt << " occupancy=" << occupancy
            << " pos=" << pos.transpose();
        reason = oss.str();
        return false;
      }
    }

    const Eigen::Vector3d end_pos = traj.getPos(check_end);
    const int end_occupancy = end_pos.allFinite() ? grid_map_->getInflateOccupancy(end_pos) : -1;
    if (end_occupancy != 0)
    {
      std::ostringstream oss;
      oss << "static map t=" << check_end << " occupancy=" << end_occupancy
          << " pos=" << end_pos.transpose();
      reason = oss.str();
      return false;
    }
    return true;
  }

 EGOPlannerManager::SwarmConflictInfo
  EGOPlannerManager::evaluateTrajectorySwarmConflict(
      const poly_traj::Trajectory &traj, const double start_time,
      const bool touch_goal, const bool adjustable_only) const
  {
    SwarmConflictInfo info;
    if (traj_.swarm_traj.empty() || pp_.drone_id < 0 ||
        traj.getPieceNum() <= 0 || !std::isfinite(start_time))
      return info;

    const double clearance = getSwarmClearance();
    info.trigger_distance = clearance + temporal_swarm_trigger_margin_;
    const double duration = traj.getTotalDuration();
    const double check_end = executionAuthorityHorizon(duration, touch_goal);
    if (!std::isfinite(check_end) || check_end <= 0.0)
      return info;

    for (double t = 0.0; t < check_end + 1.0e-9;
         t += temporal_swarm_sample_dt_)
    {
      const double tt = std::min(t, check_end);
      const Eigen::Vector3d position = traj.getPos(tt);
      if (!position.allFinite())
        continue;
      const double global_time = start_time + tt;
      for (size_t id = 0; id < traj_.swarm_traj.size(); ++id)
      {
        const LocalTrajData &other = traj_.swarm_traj.at(id).executionAt(global_time);
        if (other.drone_id < 0 || other.drone_id == pp_.drone_id ||
            other.duration <= 0.0 || other.traj.getPieceNum() <= 0 ||
            (adjustable_only && other.drone_id >= pp_.drone_id))
          continue;

        double other_t = global_time - other.start_time;
        Eigen::Vector3d other_position;
        if (other_t <= 0.0)
        {
          other_position = other.traj.getPos(0.0);
        }
        else if (other_t <= other.duration)
        {
          other_position = other.traj.getPos(other_t);
        }
        else
        {
          other_position = trajectory_lifecycle::sample(other.traj, other_t).p;
        }
        if (!other_position.allFinite())
          continue;
        const double distance = (position - other_position).norm();
        ++info.sample_count;
        if (distance < info.min_distance)
        {
          info.min_distance = distance;
          info.conflict_time = tt;
          info.other_drone_id = other.drone_id;
        }
      }
    }

    info.valid = info.sample_count > 0 && std::isfinite(info.min_distance);
    info.triggered = info.valid &&
                     info.min_distance < info.trigger_distance;
    info.temporal_owner = info.valid && pp_.drone_id > info.other_drone_id;
    info.window_start = std::max(
        0.0, info.conflict_time - temporal_swarm_conflict_half_window_);
    info.window_end = std::min(
        check_end, info.conflict_time + temporal_swarm_conflict_half_window_);
    if (info.triggered)
    {
      const LocalTrajData &peer =
          (info.other_drone_id >= 0 &&
           info.other_drone_id < static_cast<int>(traj_.swarm_traj.size()))
              ? traj_.swarm_traj.at(info.other_drone_id).executionAt(
                    start_time + info.conflict_time)
              : traj_.local_traj;
      ROS_WARN("[swarm-hard-conflict] drone=%d other_drone=%d candidate_start=%.9f "
               "candidate_duration=%.6f check_end=%.6f conflict_time=%.6f "
               "min_distance=%.6f trigger_distance=%.6f peer_traj_id=%d "
               "peer_start=%.9f peer_duration=%.6f peer_end=%.9f "
               "peer_has_predecessor=%d touch_goal=%d adjustable_only=%d",
               pp_.drone_id, info.other_drone_id, start_time, duration, check_end,
               info.conflict_time, info.min_distance, info.trigger_distance,
               peer.traj_id, peer.start_time, peer.duration,
               peer.start_time + peer.duration,
               static_cast<int>(peer.predecessor != nullptr),
               static_cast<int>(touch_goal), static_cast<int>(adjustable_only));
    }
    return info;
  }

  bool EGOPlannerManager::checkTrajectorySwarmSafety(const poly_traj::Trajectory &traj,
                                                     const double start_time,
                                                     const bool touch_goal,
                                                     std::string &reason) const
  {
    if (traj_.swarm_traj.empty() || pp_.drone_id < 0)
      return true;

    // The optimizer uses larger 1.25/1.5 buffers. Post-check enforces the
    // paper's actual Cw safety constraint, not the optimizer's soft buffer.
    const double clearance = getSwarmClearance();
    const double clearance2 = clearance * clearance;
    const double duration = traj.getTotalDuration();
    const double check_end = executionAuthorityHorizon(duration, touch_goal);
    constexpr double inv_vertical_axis2 = 0.25;

    // Physical clearance is the single hard swarm predicate. A trajectory
    // inside that clearance is not certified safe by comparing it to another
    // already-violating trajectory.

    const auto elliptical_distance2 = [&](const Eigen::Vector3d &a,
                                          const Eigen::Vector3d &b) {
      const Eigen::Vector3d d = a - b;
      return d.head<2>().squaredNorm() + d(2) * d(2) * inv_vertical_axis2;
    };

    const auto peer_position_at = [&](const LocalTrajData &other,
                                      const double global_time,
                                      Eigen::Vector3d &other_pos) {
      const double other_t = global_time - other.start_time;
      if (other_t <= other.duration)
      {
        other_pos = other.traj.getPos(std::max(0.0, other_t));
        return true;
      }
      other_pos = trajectory_lifecycle::sample(other.traj, other_t).p;
      return true;
    };

    for (double t = 0.0; t < check_end + 1.0e-6; t += 0.03)
    {
      const double tt = std::min(t, check_end);
      const Eigen::Vector3d pos = traj.getPos(tt);
      const double global_time = start_time + tt;

      for (size_t id = 0; id < traj_.swarm_traj.size(); ++id)
      {
        const LocalTrajData &other = traj_.swarm_traj.at(id).executionAt(global_time);
        if (other.drone_id < 0 || other.drone_id == pp_.drone_id || other.duration <= 0.0)
          continue;

        Eigen::Vector3d other_pos;
        peer_position_at(other, global_time, other_pos);

        const double ellip_dist2 = elliptical_distance2(pos, other_pos);
        if (ellip_dist2 < clearance2)
        {
          ++swarm_absolute_reject_count_;
          ROS_WARN_THROTTLE(
              1.0,
              "[swarm-mutual-clearance] drone=%d other_drone=%d "
              "action=REJECT_PHYSICAL_CLEARANCE candidate_min_distance=%.6f "
              "trigger_distance=%.6f conflict_time=%.6f peer_traj_id=%d",
              pp_.drone_id, other.drone_id,
              std::sqrt(ellip_dist2),
              clearance, tt, other.traj_id);
          std::ostringstream oss;
          oss << "swarm drone=" << other.drone_id << " t=" << tt
              << " ellip_dist2=" << ellip_dist2;
          reason = oss.str();
          return false;
        }
      }

    }

    return true;
  }

  void EGOPlannerManager::reportSwarmUnlockAudit(bool final_report) const
  {
    const double now = ros::Time::now().toSec();
    if (!final_report && swarm_unlock_last_report_ > 0.0 &&
        now - swarm_unlock_last_report_ < 5.0)
      return;
    if (final_report)
    {
      if (swarm_unlock_final_emitted_)
        return;
      swarm_unlock_final_emitted_ = true;
    }
    swarm_unlock_last_report_ = now;
    ROS_INFO("[swarm-unlock-audit] drone=%d SWARM_MUTUAL_WORSENING_REJECT_COUNT=%lu "
             "SWARM_MUTUAL_NON_WORSENING_ACCEPT_COUNT=%lu "
             "SWARM_ABSOLUTE_REJECT_COUNT=%lu",
             pp_.drone_id,
             static_cast<unsigned long>(swarm_mutual_worsening_reject_count_),
             static_cast<unsigned long>(swarm_mutual_non_worsening_accept_count_),
             static_cast<unsigned long>(swarm_absolute_reject_count_));
  }

 bool EGOPlannerManager::buildStaticFeasibleInitializer(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
      const Eigen::Vector3d &local_target_vel, poly_traj::MinJerkOpt &seed,
      std::string &source, double &required_speed, double &required_acc,
      double &required_jerk, bool mission_end,
      const Eigen::Vector3d *tracked_target_position)
  {
    if (!buildFreshMovingInitializer(
            start_pt, start_vel, start_acc, local_target_pt,
            local_target_vel, seed, source, required_speed, required_acc,
            required_jerk, mission_end))
      return false;

    std::string direct_reason;
    if (checkTrajectoryStaticSafety(seed.getTraj(), mission_end, direct_reason))
      return true;

    if (!grid_map_ || !side_a_star_)
    {
      source = "FRESH_DIRECT_STATIC_INFEASIBLE";
      return false;
    }

    const double resolution = grid_map_->getResolution() > 1.0e-6
                                  ? grid_map_->getResolution()
                                  : 0.1;
    const double velocity_limit = pp_.max_vel_ > 0.0 ? pp_.max_vel_ : 1.0;
    const double nominal_speed = local_target_vel.norm();
    Eigen::Vector3d target_side_point = 0.5 * (start_pt + local_target_pt);
    Eigen::Vector3d target_side_normal =
        (local_target_pt - start_pt).cross(Eigen::Vector3d::UnitZ());
    target_side_normal.z() = 0.0;
    const bool target_side_geometry_available =
        tracked_target_position != nullptr &&
        tracked_target_position->allFinite() &&
        target_side_normal.head<2>().norm() > 1.0e-6;
    bool target_side_preference = target_side_geometry_available;
    if (target_side_geometry_available)
    {
      target_side_normal.normalize();
      Eigen::Vector3d target_direction =
          *tracked_target_position - target_side_point;
      target_direction.z() = 0.0;
      const double alignment = target_direction.dot(target_side_normal);
      target_side_preference = std::abs(alignment) > 1.0e-6;
      if (alignment < 0.0)
        target_side_normal = -target_side_normal;
    }
    target_side_preference = target_side_preference &&
                             ablation_config_.target_facing_astar;
    side_a_star_->clearVisibilityCostContext();
    if (target_side_preference)
    {
      ++target_side_astar_count_;
      traj_utils::incrementAblationCounter(
          traj_utils::AblationCounter::TARGET_FACING_ASTAR_ATTEMPT,
          "planner_manager", ablation_config_);
    }
    else
    {
      ++unbiased_astar_count_;
    }
    ASTAR_RET result = side_a_star_->AstarSearch(
        resolution, start_pt, local_target_pt,
        target_side_preference ? &target_side_point : nullptr,
        target_side_preference ? &target_side_normal : nullptr,
        false);  // topology affects order/cost, never static feasibility
    bool target_side_fallback = false;
    if (result != ASTAR_RET::SUCCESS && target_side_preference)
    {
      target_side_fallback = true;
      ++unbiased_astar_count_;
      result = side_a_star_->AstarSearch(
          resolution, start_pt, local_target_pt);
    }
    side_a_star_->clearVisibilityCostContext();
    if (result != ASTAR_RET::SUCCESS)
    {
      source = "FRESH_ASTAR_FAILED";
      ROS_WARN("[local-static-initializer] drone=%d direct_reason=%s "
               "astar=FAILED start=(%.3f,%.3f,%.3f) target=(%.3f,%.3f,%.3f) "
               "target_side_preference=%d target_side_fallback=%d",
               pp_.drone_id, direct_reason.c_str(), start_pt.x(), start_pt.y(),
               start_pt.z(), local_target_pt.x(), local_target_pt.y(),
               local_target_pt.z(), static_cast<int>(target_side_preference),
               static_cast<int>(target_side_fallback));
      return false;
    }

    std::vector<Eigen::Vector3d> raw = side_a_star_->getPath();
    if (raw.size() < 2)
    {
      source = "FRESH_ASTAR_EMPTY";
      return false;
    }
    raw.front() = start_pt;
    raw.back() = local_target_pt;
    bool outer_side_path = false;
    if (target_side_geometry_available && raw.size() > 2)
    {
      for (std::size_t index = 1; index + 1 < raw.size(); ++index)
      {
        if (target_side_normal.dot(raw[index] - target_side_point) <
            -0.5 * resolution)
        {
          outer_side_path = true;
          break;
        }
      }
      if (outer_side_path)
        ++outer_side_astar_count_;
    }
    ROS_INFO("[astar-topology-counter] drone=%d mode=%s "
             "TARGET_SIDE_ASTAR_COUNT=%lu OUTER_SIDE_ASTAR_COUNT=%lu "
             "UNBIASED_ASTAR_COUNT=%lu current_outer=%d",
             pp_.drone_id, traj_utils::toString(ablation_config_.mode),
             target_side_astar_count_, outer_side_astar_count_,
             unbiased_astar_count_, static_cast<int>(outer_side_path));

    const auto segment_free = [&](const Eigen::Vector3d &from,
                                  const Eigen::Vector3d &to) {
      return voxelSegmentFree(*grid_map_, from, to, resolution);
    };

    std::vector<Eigen::Vector3d> path;
    path.push_back(raw.front());
    size_t current = 0;
    while (current + 1 < raw.size())
    {
      size_t next = current + 1;
      for (size_t candidate = raw.size() - 1; candidate > current; --candidate)
      {
        if (segment_free(raw[current], raw[candidate]))
        {
          next = candidate;
          break;
        }
      }
      if (next <= current)
      {
        source = "FRESH_ASTAR_SIMPLIFY_FAILED";
        return false;
      }
      path.push_back(raw[next]);
      current = next;
    }
    if (path.size() < 3 || path.size() > 25)
    {
      source = "FRESH_ASTAR_PATH_SIZE_INVALID";
      return false;
    }

    const int pieces = static_cast<int>(path.size()) - 1;
    Eigen::Matrix3d head, tail;
    head << start_pt, start_vel, start_acc;
    Eigen::Vector3d terminal_velocity =
        mission_end ? Eigen::Vector3d::Zero() : local_target_vel;
    if (pp_.max_vel_ > 0.0 && terminal_velocity.norm() > pp_.max_vel_)
      terminal_velocity *= pp_.max_vel_ / terminal_velocity.norm();
    tail << local_target_pt, terminal_velocity, Eigen::Vector3d::Zero();
    Eigen::MatrixXd inner(3, pieces - 1);
    Eigen::VectorXd durations(pieces);
    const double acceleration_limit = pp_.max_acc_ > 0.0 ? pp_.max_acc_ : 1.0;
    const double reference_speed = std::max(
        0.25 * velocity_limit,
        std::min(velocity_limit,
                 nominal_speed > 1.0e-3 ? nominal_speed
                                        : 0.55 * velocity_limit));
    double path_length = 0.0;
    for (int piece = 0; piece < pieces; ++piece)
    {
      const double length = (path[piece + 1] - path[piece]).norm();
      path_length += length;
      durations(piece) = std::max(0.10, 1.05 * length / reference_speed);
      durations(piece) = std::max(
          durations(piece),
          1.5 * std::sqrt(std::max(1.0e-3, length) / acceleration_limit));
      if (piece + 1 < pieces)
        inner.col(piece) = path[piece + 1];
    }

    double max_velocity = 0.0;
    double max_acceleration = 0.0;
    double max_jerk = 0.0;
    int timing_iterations = 0;
    const bool timing_ok = initializePvaTiming(
        head, tail, inner, durations, pp_.max_vel_, pp_.max_acc_,
        pp_.max_jer_,
        [&](const poly_traj::Trajectory &trajectory, double &v, double &a,
            double &j) { return checkTrajectoryDynamics(trajectory, v, a, j); },
        seed, max_velocity, max_acceleration, max_jerk, timing_iterations);
    if (!timing_ok) {
      source = "NUMERICAL_INITIALIZATION_FAILED";
      ++initializer_rejected_preopt_count_;
      ROS_WARN("[PVA_TIMING_INIT] kind=N status=%s iterations=%d",
               source.c_str(), timing_iterations);
      return false;
    }
    source = target_side_preference && !target_side_fallback
                 ? "FRESH_ASTAR_TARGET_SIDE_LOCAL_CONSTRAINTS"
                 : "FRESH_ASTAR_LOCAL_CONSTRAINTS";
    if (timing_iterations > 1) ++initializer_retimed_count_;
    required_speed = path_length / seed.getTraj().getTotalDuration();
    required_acc = max_acceleration;
    required_jerk = max_jerk;
    ROS_INFO("[PVA_TIMING_INIT] kind=N status=SUCCESS iterations=%d "
             "pieces=%d duration=%.6f max_v=%.6f max_a=%.6f max_j=%.6f",
             timing_iterations, pieces, seed.getTraj().getTotalDuration(),
             max_velocity, max_acceleration, max_jerk);
    return true;
  }

  bool EGOPlannerManager::reboundReplan(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
      const Eigen::Vector3d &local_target_vel, const Eigen::Vector3d &object_pt,
      const Eigen::Vector3d &object_vel,  const Eigen::Quaterniond &object_q,
      const Eigen::Vector3d &relative_track_pt, const bool flag_polyInit,
      const bool flag_randomPolyTraj, const bool touch_goal, const bool satrt_tracking,
      const double commit_head_freshness_threshold,
      CandidateSetOutput *capture_output,
      const int encirclement_hypothesis_id,
      const double encirclement_phi0,
      const unsigned long encirclement_generation)
  {

    // The caller hands over the raw smoothed target state and the stamp it
    // belongs to.  From here on this batch uses exactly one target time base:
    // targetPositionAt(world_time) with world_time = planning_prediction_epoch
    // + relative time.  Nothing below may re-derive the target from
    // ros::Time::now().
    if (targetStateValid() && (target_state_position_ - object_pt).norm() > 1.0e-6)
      ++mixed_target_timebase_count_;
    if (!targetStateValid())
      setTargetState(object_pt, object_vel, local_activation_time_);
    const double planning_prediction_epoch = local_activation_time_;
    // Authoritative target position for the whole batch.  Every target/dynamic
    // query below uses this single world-time-derived value so that a late CPU
    // slice cannot move the target between two evaluations of one candidate.
    const Eigen::Vector3d target_at_prediction_epoch =
        targetPositionAt(planning_prediction_epoch);
    // Every dynamic-risk evaluation of this batch must use the batch's own
    // planning epoch.  A different epoch would move the obstacle prediction
    // between two evaluations of the same candidate and is audited here.
    const auto dynamic_risk_at_epoch = [&](const poly_traj::Trajectory &traj,
                                           const double epoch,
                                           const bool touch) {
      if (!std::isfinite(epoch) ||
          std::abs(epoch - planning_prediction_epoch) > 1.0e-9)
        ++mixed_dynamic_risk_timebase_count_;
      return evaluateDynamicRisk(traj, epoch, touch);
    };

    const bool optional_allowed=optionalRefinementAllowed();
    const bool mandatory_allowed=mandatoryPlanningAttemptAllowed();
    ROS_INFO("[local-planning-batch] timestamp=%.9f drone=%d "
             "active_trajectory_id=%d deadline_allowed=%d "
             "mandatory_supply_allowed=%d current_state_restart=%d",
        ros::Time::now().toSec(),pp_.drone_id,traj_.local_traj.traj_id,
        int(optional_allowed),int(mandatory_allowed),
        int(current_state_restart_active_));
    if(!optional_allowed && !mandatory_allowed) return false;
    static int count = 0;
    printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n",
           pp_.drone_id, count++);
    // cout.precision(3);
    // cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
    //      << endl;

    ploy_traj_opt_->setIfTouchGoal(touch_goal);
    ploy_traj_opt_->setObject(target_at_prediction_epoch, object_vel, object_q);
    ploy_traj_opt_->setRelativeTrackingP(relative_track_pt);
    ploy_traj_opt_->setStartTracking(satrt_tracking);
    // 阶段 C：Local visibility guidance 的门 = ablation_config.local_visibility
    // + 有效 visibility context（目标状态有限且跟踪模式已配置），与 Team 的
    // encirclement_generation 完全解耦。软合围参考仍由 production tracking
    // mode 决定（encirclement 是 formal invariant，恒开）。
    const bool visibility_context_valid =
        target_at_prediction_epoch.allFinite() && object_vel.allFinite() &&
        relative_track_pt.allFinite() &&
        ploy_traj_opt_->isEncirclementTrackingConfigured();
    ploy_traj_opt_->setSoftEncirclementReference(true);
    ploy_traj_opt_->setDirectionalVisibilityGuidance(
        ablation_config_.local_visibility && visibility_context_valid);
    local_geometry_active_ = visibility_context_valid;

    // Cooperative geometry remains one soft objective. It has no separate
    // recovery/endpoint authority and cannot reject a hard-safe candidate.
    {
      const double execution_prefix = local_activation_margin_ + execution_margin_;
      const int cps_per_piece = std::max(1, ploy_traj_opt_->get_cps_num_prePiece_());
      const double bearing_sample_dt =
          (pp_.max_vel_ > 0.0 && pp_.polyTraj_piece_length > 0.0)
              ? pp_.polyTraj_piece_length /
                    (static_cast<double>(cps_per_piece) * pp_.max_vel_)
              : 0.05;
      ploy_traj_opt_->setEncirclementBearingRecovery(
          encirclement_bearing_recovery_weight_,
          encirclement_bearing_prefix_boost_, execution_prefix,
          bearing_sample_dt);
      ROS_INFO_THROTTLE(
          0.5,
          "[bearing-recovery-setup] drone=%d w_bearing=%.6f prefix_boost=%.3f "
          "prefix_span=%.6f sample_dt=%.6f",
          pp_.drone_id, encirclement_bearing_recovery_weight_,
          encirclement_bearing_prefix_boost_, execution_prefix,
          bearing_sample_dt);
    }
    ploy_traj_opt_->setVisibilityContextIdentity(
        encirclement_generation, encirclement_hypothesis_id);
    const bool visibility_yaw_fresh =
        pp_.drone_id >= 0 && pp_.drone_id < 3 &&
        visibility_odom_[pp_.drone_id].valid &&
        ros::Time::now().toSec() - visibility_odom_[pp_.drone_id].stamp >=
            -0.10 &&
        ros::Time::now().toSec() - visibility_odom_[pp_.drone_id].stamp <=
            1.0;
    ploy_traj_opt_->setVisibilityYawState(
        visibility_yaw_fresh,
        visibility_yaw_fresh ? visibility_odom_[pp_.drone_id].yaw : 0.0,
        visibility_yaw_fresh ? visibility_odom_[pp_.drone_id].yaw_rate : 0.0,
        tracking_camera_.horizontal_fov, tracking_camera_.vertical_fov,
        tracking_camera_.min_range, tracking_camera_.max_range);
    // The initializer and its target state use planning_prediction_epoch.
    // The previous active trajectory's start_time is an older time origin.
    ploy_traj_opt_->setTeamVisibilityReserveActivation(
        planning_prediction_epoch);

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      // continous_failures_count_++;
      // return false;
    }

    ros::Time t_start = ros::Time::now();

    MovingObjPredictionEpochScope prediction_epoch_scope(
        ploy_traj_opt_.get(), planning_prediction_epoch,
        enable_risk_triggered_candidates_ && static_cast<bool>(obj_predictor_));
    /* Feedback126 §5 pairwise ownership: for any conflicted pair the drone
     * with the HIGHER id is the OWNER (keeps its committed trajectory as the
     * fixed pair reference); the LOWER id drone is the ADJUSTER and must
     * satisfy the hard swarm constraint against the owner's committed
     * future.  Exactly one adjuster exists per pair per conflict snapshot —
     * this kills the symmetric "both sides avoid each other at once" dance.
     * Deterministic from IDs alone, so both endpoints agree without any new
     * protocol message. */
    std::vector<int> swarm_adjuster_peers;
    std::vector<int> swarm_owner_peers;
    for (size_t id = 0; id < traj_.swarm_traj.size(); ++id)
    {
      const LocalTrajData &peer = traj_.swarm_traj[id].executionAt(
          planning_prediction_epoch);
      if (peer.drone_id < 0 || peer.drone_id == pp_.drone_id ||
          peer.duration <= 0.0 || peer.traj.getPieceNum() <= 0)
        continue;
      if (peer.drone_id > pp_.drone_id)
        swarm_adjuster_peers.push_back(peer.drone_id);
      else
        swarm_owner_peers.push_back(peer.drone_id);
    }
    ploy_traj_opt_->setSwarmAdjusterPeers(swarm_adjuster_peers);
    if (!swarm_adjuster_peers.empty() || !swarm_owner_peers.empty())
    {
      std::string adj = "[", own = "[";
      for (size_t k = 0; k < swarm_adjuster_peers.size(); ++k)
        adj += (k ? "," : "") + std::to_string(swarm_adjuster_peers[k]);
      for (size_t k = 0; k < swarm_owner_peers.size(); ++k)
        own += (k ? "," : "") + std::to_string(swarm_owner_peers[k]);
      ROS_INFO_THROTTLE(2.0,
          "[SWARM_OWNER_ASSIGNMENT] drone=%d adjuster_peers=%s] "
          "owner_peers=%s] rule=HIGHER_ID_OWNS_PAIR",
          pp_.drone_id, adj.c_str(), own.c_str());
    }
    ros::Duration t_init, t_opt;

    /*** STEP 1: INIT ***/
    double ts = pp_.polyTraj_piece_length / pp_.max_vel_;

    poly_traj::MinJerkOpt initMJO;
    bool target_side_static_initializer = false;
    bool warm_start_used = false;
    double warm_age = -1.0, warm_remaining_duration = -1.0;
    int warm_old_piece_num = 0, warm_new_piece_num = 0;
    std::string warm_reason;
    // Hypothesis zero is the single authoritative NOMINAL objective and keeps
    // the normal local target. Initializer choice is local to this Planner:
    // a current accepted NOMINAL suffix may warm-start it; otherwise the same
    // call constructs a direct or A*/Local-SFC static-feasible initializer.
    const bool nominal_baseline = (encirclement_hypothesis_id == 0);
    // Target semantics and numerical initialization are independent.  The
    // NOMINAL target remains the normal local target, but a previously
    // accepted NOMINAL trajectory is a valid numerical warm start when it is
    // current, absolute-safe, and no team/guide geometry is active.  Never
    // reuse an alternative (SIDE/team/recovery) cache for hypothesis zero.
    const bool nominal_warm_cache =
        nominal_baseline && accepted_state_cache_kind_ == CandidateKind::NOMINAL;
    const bool warm_start_allowed = nominal_warm_cache && !flag_polyInit;
    ROS_INFO("[nominal-baseline-semantics] drone=%d nominal=%d hypothesis=%d generation=%lu "
             "warm_start_allowed=%d source=AUTHORITATIVE_PVA_TO_LOCAL_TARGET",
             pp_.drone_id, static_cast<int>(nominal_baseline),
             encirclement_hypothesis_id,
             static_cast<unsigned long>(encirclement_generation),
             static_cast<int>(warm_start_allowed));
    if (warm_start_allowed &&
        buildWarmStartFromAccepted(
                             start_pt, start_vel, start_acc, local_target_pt,
                             local_target_vel, initMJO, warm_age,
                             warm_remaining_duration, warm_old_piece_num,
                             warm_new_piece_num, warm_reason))
    {
      warm_start_used = true;
      ROS_INFO("[trajectory-warm-start] previous_traj_id=%lu age=%.6f "
               "current_piece=%d remaining_duration=%.6f old_piece_num=%d "
               "new_piece_num=%d topology=%s head_p_error=0 head_v_error=0 "
               "head_a_error=0 init_max_jerk=%.6f init_min_dynamic_clearance=NA "
               "action=USED_PREVIOUS_SAFE",
               accepted_state_cache_id_, warm_age, 0,
               warm_remaining_duration, warm_old_piece_num, warm_new_piece_num,
               candidateKindName(accepted_state_cache_kind_), -1.0);
    }
    else
    {
      if (warm_reason.empty())
        warm_reason = "NO_ACCEPTED_STATE";
      ROS_INFO("[trajectory-warm-start] previous_traj_id=%lu age=%.6f "
               "current_piece=-1 remaining_duration=-1 old_piece_num=%d "
               "new_piece_num=-1 topology=%s head_p_error=NA head_v_error=NA "
               "head_a_error=NA init_max_jerk=NA init_min_dynamic_clearance=NA "
               "action=%s",
               accepted_state_cache_id_, warm_age, warm_old_piece_num,
               candidateKindName(accepted_state_cache_kind_), warm_reason.c_str());
      std::string fresh_source;
      double fresh_speed = 0.0, fresh_acc = 0.0, fresh_jerk = 0.0;
      const bool seed_ok = buildStaticFeasibleInitializer(
          start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
          initMJO, fresh_source, fresh_speed, fresh_acc, fresh_jerk,
          touch_goal, &target_at_prediction_epoch);
      target_side_static_initializer =
          seed_ok &&
          fresh_source == "FRESH_ASTAR_TARGET_SIDE_LOCAL_CONSTRAINTS";
      ROS_INFO("[fresh-initializer-selection] drone=%d source=%s ok=%d "
               "required_speed=%.6f required_acc=%.6f required_jerk=%.6f",
               pp_.drone_id, fresh_source.c_str(), static_cast<int>(seed_ok),
               fresh_speed, fresh_acc, fresh_jerk);
      if (!seed_ok)
        return false;
    }

    double init_max_vel = 0.0, init_max_acc = 0.0, init_max_jerk = 0.0;
    const bool init_dynamics_valid = checkTrajectoryDynamics(
        initMJO.getTraj(), init_max_vel, init_max_acc, init_max_jerk);
    ROS_INFO("[fresh-init-snapshot] drone=%d warm_start_used=%d piece_num=%d "
             "duration=%.6f start=(%.6f %.6f %.6f) target=(%.6f %.6f %.6f) "
             "max_vel=%.6f max_acc=%.6f max_jerk=%.6f dynamics_valid=%d",
             pp_.drone_id, static_cast<int>(warm_start_used),
             initMJO.getTraj().getPieceNum(), initMJO.getTraj().getTotalDuration(),
             start_pt.x(), start_pt.y(), start_pt.z(), local_target_pt.x(),
             local_target_pt.y(), local_target_pt.z(), init_max_vel, init_max_acc,
             init_max_jerk, static_cast<int>(init_dynamics_valid));

    const poly_traj::Trajectory init_terminal_debug_traj = initMJO.getTraj();
    const Eigen::Vector3d init_terminal_debug =
        init_terminal_debug_traj.getPos(init_terminal_debug_traj.getTotalDuration());
    const double init_terminal_error = (init_terminal_debug - local_target_pt).norm();
    const int init_terminal_raw_occupancy = grid_map_->getOccupancy(init_terminal_debug);
    const int init_terminal_inflated_occupancy =
        grid_map_->getInflateOccupancy(init_terminal_debug);
    if (init_terminal_error > 1.0e-4 || init_terminal_inflated_occupancy != 0)
    {
      Eigen::Vector3i terminal_voxel;
      grid_map_->posToIndex(init_terminal_debug, terminal_voxel);
      ROS_WARN_THROTTLE(
          0.5,
          "[terminal-debug] drone=%d mode=%s phase=nominal-init candidate=NOMINAL "
          "end=(%.3f %.3f %.3f) terminal=(%.3f %.3f %.3f) error=%.6g "
          "raw_occ=%d inflated_occ=%d voxel=(%d %d %d) touch_goal=%d poly_init=%d "
          "random_init=%d init_duration=%.3f current_duration=%.3f epoch=%.6f",
          pp_.drone_id,
          enable_risk_triggered_candidates_ ? "risk-spatial" : "gradient-only",
          local_target_pt.x(), local_target_pt.y(), local_target_pt.z(),
          init_terminal_debug.x(), init_terminal_debug.y(), init_terminal_debug.z(),
          init_terminal_error, init_terminal_raw_occupancy,
          init_terminal_inflated_occupancy, terminal_voxel.x(), terminal_voxel.y(),
          terminal_voxel.z(), static_cast<int>(touch_goal), static_cast<int>(flag_polyInit),
          static_cast<int>(flag_randomPolyTraj), init_terminal_debug_traj.getTotalDuration(),
          traj_.local_traj.duration, planning_prediction_epoch);
    }

    Eigen::MatrixXd cstr_pts = initMJO.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
    vector<std::pair<int, int>> segments;
    if (ploy_traj_opt_->finelyCheckAndSetConstraintPoints(segments, initMJO, true) == PolyTrajOptimizer::CHK_RET::ERR)
    {
      return false;
    }

    t_init = ros::Time::now() - t_start;

    std::vector<Eigen::Vector3d> point_set;
    for (int i = 0; i < cstr_pts.cols(); ++i)
      point_set.push_back(cstr_pts.col(i));
    visualization_->displayInitPathList(point_set, 0.2, 0);

    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    // One authoritative geometry pass per rolling batch. All nominal Local
    // candidates reuse these fixed guides; SIDE keeps its own topology/SFC
    // authority and Team realization clears guides when it reanchors.
    ploy_traj_opt_->setGradientAuditContext("NOMINAL", -1);
    ploy_traj_opt_->prepareTeamVisibilityReserveGuides(initMJO.getTraj());
    bool flag_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;
    poly_traj::MinJerkOpt best_MJO;
    ConstraintPoints best_constraints;
    double nominal_optimizer_cost = 0.0;

    if (pp_.use_distinctive_trajs)
    {
      std::vector<ConstraintPoints> trajs = ploy_traj_opt_->distinctiveTrajs(segments);
      cout << "\033[1;33m"
           << "multi-trajs=" << trajs.size() << "\033[1;0m" << endl;

      poly_traj::Trajectory initTraj = initMJO.getTraj();
      int PN = initTraj.getPieceNum();
      Eigen::MatrixXd all_pos = initTraj.getPositions();
      Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
      Eigen::Matrix<double, 3, 3> headState, tailState;
      headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
      tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
      double final_cost, min_cost = 999999.0;
      for (int i = trajs.size() - 1; i >= 0; i--)
      {
        ploy_traj_opt_->setConstraintPoints(trajs[i]);
        if (ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                               innerPts, initTraj.getDurations(),
                                               cstr_pts, final_cost))
        {

          cout << "traj " << trajs.size() - i << " success." << endl;

          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            best_MJO = ploy_traj_opt_->getMinJerkOpt();
            best_constraints = ploy_traj_opt_->getControlPoints();
            flag_success = true;
          }

          // visualization
          Eigen::MatrixXd ctrl_pts_temp = ploy_traj_opt_->getMinJerkOpt().getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
          std::vector<Eigen::Vector3d> point_set;
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(point_set);
        }
        else
        {
          cout << "traj " << trajs.size() - i << " failed." << endl;
        }
      }

      t_opt = ros::Time::now() - t_start;
      nominal_optimizer_cost = min_cost;

      visualization_->displayMultiInitPathList(vis_trajs, 0.2); // This visuallization will take up several milliseconds.
    }
    else
    {
      poly_traj::Trajectory initTraj = initMJO.getTraj();
      int PN = initTraj.getPieceNum();
      Eigen::MatrixXd all_pos = initTraj.getPositions();
      Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
      Eigen::Matrix<double, 3, 3> headState, tailState;
      headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
      tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
      double final_cost;
      flag_success = ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                                        innerPts, initTraj.getDurations(),
                                                        cstr_pts, final_cost);
      best_MJO = ploy_traj_opt_->getMinJerkOpt();
      best_constraints = ploy_traj_opt_->getControlPoints();
      nominal_optimizer_cost = final_cost;

      t_opt = ros::Time::now() - t_start;
    }

    if (!flag_success &&
        (best_MJO.getTraj().getPieceNum() <= 0 ||
         !std::isfinite(best_MJO.getTraj().getTotalDuration()) ||
         best_MJO.getTraj().getTotalDuration() <= 1.0e-6))
    {
      // Preserve a geometrically valid guide for risk localization and SIDE
      // recovery.  It remains an INVALID nominal CandidateResult because the
      // nominal solve itself failed.
      best_MJO = initMJO;
      best_constraints = ploy_traj_opt_->getControlPoints();
      nominal_optimizer_cost = std::numeric_limits<double>::infinity();
    }

    CandidateResult nominal_result;
    nominal_result.candidate_id = ++topology_candidate_serial_;
    nominal_result.success = flag_success;
    nominal_result.kind = CandidateKind::NOMINAL;
    nominal_result.min_jerk_opt = best_MJO;
    nominal_result.target_side_static_topology =
        target_side_static_initializer;
    // Export the obstacle-aware initializer before the local nonlinear solve
    // can turn it into a finalized executable candidate.  finelyCheck above
    // established the static construction contract; dynamic/swarm safety is
    // deliberately re-established by the joint optimizer and final preflight.
    nominal_result.joint_seed = initMJO;
    nominal_result.joint_seed_valid =
        initMJO.getTraj().getPieceNum() > 0 &&
        std::isfinite(initMJO.getTraj().getTotalDuration());
    nominal_result.joint_seed_static_constructible =
        nominal_result.joint_seed_valid;
    nominal_result.joint_seed_local_sfc_valid = true;
    nominal_result.joint_seed_construction_ms = t_init.toSec() * 1000.0;
    const DynamicRiskInfo nominal_seed_risk = nominal_result.joint_seed_valid
        ? evaluateDynamicRisk(initMJO.getTraj(), planning_prediction_epoch,
                              touch_goal)
        : DynamicRiskInfo();
    nominal_result.joint_seed_dynamic_clearance = nominal_seed_risk.valid
        ? nominal_seed_risk.min_distance : -1.0;
    nominal_result.constraint_points = best_constraints;
    nominal_result.display_points = cstr_pts;
    nominal_result.optimization_cost = nominal_optimizer_cost;
    nominal_result.prediction_epoch = planning_prediction_epoch;
    // This initializer is already a continuous MinJerk trajectory, not a raw
    // grid path. If nonlinear refinement fails, retain it as an ordinary
    // candidate and let the unchanged full hard preflight decide it. SIDE
    // already has the same feasible-initializer fallback; dropping only the
    // NOMINAL route discarded the target-facing homotopy prematurely.
    if (!flag_success && ablation_config_.safe_seed_retention &&
        target_side_static_initializer &&
        initMJO.getTraj().getPieceNum() > 0 &&
        std::isfinite(initMJO.getTraj().getTotalDuration()) &&
        initMJO.getTraj().getTotalDuration() > 1.0e-6)
    {
      nominal_result.success = true;
      nominal_result.min_jerk_opt = initMJO;
      nominal_result.feasible_initializer_fallback = true;
      nominal_result.optimization_cost = initMJO.getTrajJerkCost();
      traj_utils::incrementAblationCounter(
          traj_utils::AblationCounter::SAFE_SEED_RETENTION_USED,
          "planner_manager", ablation_config_);
      ROS_WARN("[nominal-feasible-fallback] drone=%d used=1 "
               "source=TARGET_SIDE_ASTAR_MINJERK "
               "reason=NOMINAL_REFINEMENT_FAILED "
               "next=FULL_CURRENT_REVISION_PREFLIGHT",
               pp_.drone_id);
    }

    // // save and display planned results
    cout << "plan_success=" << flag_success << endl;
    if (!flag_success && !nominal_result.feasible_initializer_fallback)
    {
      visualization_->displayFailedList(cstr_pts, 0);
      continous_failures_count_++;
      ROS_WARN("[nominal-candidate] solve_success=0 class=INVALID "
               "action=CONTINUE_PREVIOUS_REVALIDATION_AND_SIDE_RECOVERY "
               "guide_piece_num=%d guide_duration=%.6f",
               nominal_result.min_jerk_opt.getTraj().getPieceNum(),
               nominal_result.min_jerk_opt.getTraj().getTotalDuration());
    }

    static double sum_time = 0;
    static int count_success = 0;
    sum_time += (t_init + t_opt).toSec();
    count_success++;
    cout << "total time:\033[42m" << (t_init + t_opt).toSec()
         << "\033[0m,init:" << t_init.toSec()
         << ",optimize:" << t_opt.toSec()
         << ",avg_time=" << sum_time / count_success << endl;

    CandidateResult plus_result;
    CandidateResult minus_result;
    CandidateResult *selected_result = &nominal_result;
    bool previous_safe_available = false;
    std::string previous_validation_reason;
    double previous_min_dynamic_clearance = -1.0;
    // Execution authority uses physical contact clearance. The configured
    // 1.1 m value remains a preferred avoidance/ranking margin.
    const double absolute_dynamic_threshold =
        ploy_traj_opt_->getMovingObjHardClearance();
    std::string selected_safety_reason = "UNSAFE_FALLBACK";
    bool candidate_safety_logged = false;
    const auto classify_candidate = [&](CandidateResult &candidate) {
      candidate.existing_checks_passed = false;
      candidate.static_valid = false;
      candidate.dynamics_valid = false;
      candidate.swarm_valid = false;
      candidate.max_velocity = 0.0;
      candidate.max_acceleration = 0.0;
      candidate.max_jerk = 0.0;
      candidate.swarm_clearance = std::numeric_limits<double>::infinity();
      if (candidate.success && candidate.min_jerk_opt.getTraj().getPieceNum() > 0)
      {
        double max_vel = 0.0;
        double max_acc = 0.0;
        double max_jerk = 0.0;
        std::string static_reason;
        candidate.dynamics_valid = checkTrajectoryDynamics(
            candidate.min_jerk_opt.getTraj(), max_vel, max_acc, max_jerk);
        if (!candidate.dynamics_valid)
        {
          candidate.risk = evaluateDynamicRisk(candidate.min_jerk_opt.getTraj(),
                                               candidate.prediction_epoch, touch_goal);
        }
        candidate.static_valid = checkTrajectoryStaticSafety(
            candidate.min_jerk_opt.getTraj(), touch_goal, static_reason);
        candidate.max_velocity = max_vel;
        candidate.max_acceleration = max_acc;
        candidate.max_jerk = max_jerk;
        /* Feedback126 §3: the classify swarm gate is the SAME unified
         * physical predicate the optimizer rows and the final preflight use
         * (elliptical clearance at the batch epoch, polynomial-then-hold peer
         * model).  The legacy trigger-margin (clearance+0.08) hard gate is
         * removed — a candidate 0.55 m from a peer is not a safety failure.
         * The trigger metric remains only as telemetry. */
        PolyTrajOptimizer::UnifiedSwarmFinding unified_swarm;
        candidate.swarm_valid =
            !ploy_traj_opt_->swarmPhysicalViolation(
                candidate.min_jerk_opt.getTraj(), candidate.prediction_epoch,
                touch_goal, &unified_swarm);
        const SwarmConflictInfo swarm = evaluateTrajectorySwarmConflict(
            candidate.min_jerk_opt.getTraj(), candidate.prediction_epoch,
            touch_goal, false);
        candidate.swarm_clearance = swarm.valid
                                          ? swarm.min_distance
                                          : std::numeric_limits<double>::infinity();
        candidate.existing_checks_passed =
            candidate.dynamics_valid && candidate.static_valid && candidate.swarm_valid;
        if(!candidate.existing_checks_passed)
          ROS_INFO("[moving-candidate-reject] drone=%d candidate=%d dynamics=%d static=%d swarm=%d max_v=%.9f max_a=%.9f max_j=%.9f static_reason=%s swarm_peer=%d swarm_t=%.3f swarm_ellip2=%.4f",
              pp_.drone_id,candidate.candidate_id,int(candidate.dynamics_valid),int(candidate.static_valid),int(candidate.swarm_valid),max_vel,max_acc,max_jerk,static_reason.c_str(),
              unified_swarm.peer_drone_id,unified_swarm.conflict_time,unified_swarm.ellip_dist2);
      }

      candidate.safety_class = CandidateSafetyClass::INVALID;
      if (!candidate.success || !candidate.risk.valid || candidate.risk.hard_collision ||
          !candidate.existing_checks_passed)
        return;
      if (candidate.risk.hard_clearance >= -1.0e-6)
        candidate.safety_class = CandidateSafetyClass::ABSOLUTE_SAFE;
    };
    ROS_INFO_THROTTLE(
        1.0,
        "[risk-candidate-dispatch] drone=%d enabled=%d predictor=%d nominal_success=%d",
        pp_.drone_id, static_cast<int>(enable_risk_triggered_candidates_),
        static_cast<int>(static_cast<bool>(obj_predictor_)),
        static_cast<int>(nominal_result.success));
    // Topology exploration is driven by the same continuous visibility-risk
    // support used by the optimizer (static LOS, dynamic LOS and FOV).  The
    // old aggregate All3/K2/blackout percentages were empirical, horizon-
    // dependent gates and could fire only after the useful detour window had
    // already closed.  A tiny epsilon is used solely as a numerical zero
    // test; no task-quality threshold is introduced here.
    const auto &nominal_visibility_diagnostics =
        ploy_traj_opt_->getLastOptimizationDiagnostics();
    const auto &nominal_visibility_diag = nominal_visibility_diagnostics.final;
    const poly_traj::Trajectory nominal_visibility_trajectory =
        nominal_result.min_jerk_opt.getTraj();
    const bool nominal_visibility_trajectory_available =
        nominal_visibility_trajectory.getPieceNum() > 0 &&
        std::isfinite(nominal_visibility_trajectory.getTotalDuration()) &&
        nominal_visibility_trajectory.getTotalDuration() > 1.0e-6;
    // Visibility diagnostics may cover the complete polynomial, while the
    // rolling execution contract only authorizes the same bounded horizon as
    // the moving-object optimizer.  Keep the diagnostic horizon intact, but
    // never turn a later LOS forecast into an immediate SIDE command.
    double visibility_authority_horizon =
        nominal_visibility_trajectory_available
            ? nominal_visibility_trajectory.getTotalDuration()
            : 0.0;
    if (!touch_goal)
      visibility_authority_horizon *= (2.0 / 3.0);
    visibility_authority_horizon = std::max(
        0.0, std::min(visibility_authority_horizon,
                       ploy_traj_opt_->getMovingObjPredictionHorizon()));

    // Raw LOS truth is sampled independently of optimizer costs, FOV, BODY
    // risk and candidate selection.  UAV, target and moving blockers all use
    // the same absolute world time.
    bool raw_static_los_blocked = false;
    bool raw_dynamic_los_blocked = false;
    double raw_static_los_time = -1.0;
    double raw_dynamic_los_time = -1.0;
    double raw_static_los_clearance = std::numeric_limits<double>::infinity();
    int raw_dynamic_los_id = -1;
    Eigen::Vector3d raw_static_observer = Eigen::Vector3d::Zero();
    Eigen::Vector3d raw_static_target = Eigen::Vector3d::Zero();
    Eigen::Vector3d raw_dynamic_observer = Eigen::Vector3d::Zero();
    Eigen::Vector3d raw_dynamic_target = Eigen::Vector3d::Zero();
    Eigen::Vector3d raw_dynamic_center = Eigen::Vector3d::Zero();
    Eigen::Vector3d raw_dynamic_scale = Eigen::Vector3d::Zero();
    StaticLosWitness raw_static_witness;
    // 本轮 P1：真实遮挡区间。对每个 blocker 记录"第一次进入视线"与"最后一次
    // 离开视线"的相对时刻；时间轴统一为 planning_prediction_epoch + relative_time，
    // 与 target/dynamic 预测完全同一世界时基，不引入第二个 epoch。
    const double raw_los_interval_scan_horizon = std::max(
        visibility_authority_horizon,
        ploy_traj_opt_->getMovingObjPredictionHorizon());
    std::vector<double> per_blocker_first_contact(
        obj_predictor_ ? std::max(1, obj_predictor_->getObjNums()) : 1,
        std::numeric_limits<double>::quiet_NaN());
    raw_los_occlusion_enter_rel_ = -1.0;
    raw_los_occlusion_exit_rel_ = -1.0;
    raw_los_occlusion_interval_observed_ = false;
    // Single authoritative target time base established at the top of this
    // batch: the raw LOS truth sampler uses the same target world position as
    // J_vis, candidate visibility and evaluateDynamicRisk.
    const double caller_target_offset =
        (object_pt - target_at_prediction_epoch).norm();
    // A non-zero offset here is the *expected* age compensation between the
    // smoothed target state and its value at the batch epoch; it is not a
    // mismatch.  Only an unusable/absent authoritative target stamp is a real
    // time-base defect for this batch.
    if (!targetStateValid())
      ++mixed_target_timebase_count_;
    else if (!std::isfinite(caller_target_offset))
      ++mixed_target_timebase_count_;
    if (nominal_visibility_trajectory_available &&
        raw_los_interval_scan_horizon > 1.0e-6)
    {
      for (double sample = 0.0;; sample += std::max(0.02, visibility_sample_dt_))
      {
        const double relative_time = std::min(sample, raw_los_interval_scan_horizon);
        const double sample_world_time = planning_prediction_epoch + relative_time;
        const Eigen::Vector3d observer =
            nominal_visibility_trajectory.getPos(
                std::min(relative_time,
                         nominal_visibility_trajectory.getTotalDuration()));
        const Eigen::Vector3d target =
            target_at_prediction_epoch + object_vel * relative_time;
        if (!raw_static_los_blocked && observer.allFinite() && target.allFinite())
        {
          StaticLosWitness witness;
          double clearance = std::numeric_limits<double>::infinity();
          const bool query_valid = ploy_traj_opt_->queryStaticLosClearance(
              observer, target, clearance, witness);
          if (query_valid && witness.valid && std::isfinite(clearance) &&
              clearance <= visibility_occlusion_margin_)
          {
            raw_static_los_blocked = true;
            raw_static_los_time = relative_time;
            raw_static_los_clearance = clearance;
            raw_static_observer = observer;
            raw_static_target = target;
            raw_static_witness = witness;
          }
        }
        if (obj_predictor_ && observer.allFinite() && target.allFinite())
        {
          for (int id = 0; id < obj_predictor_->getObjNums(); ++id)
          {
            if (!obj_predictor_->hasPrediction(id))
              continue;
            const Eigen::Vector3d center =
                obj_predictor_->evaluateConstVel(id, sample_world_time);
            const Eigen::Vector3d scale = obj_predictor_->getObjScale(id);
            if (!center.allFinite() || !scale.allFinite() ||
                scale.x() <= 0.0 || scale.y() <= 0.0 || scale.z() <= 0.0)
              continue;
            const double radius = 0.5 * std::max(scale.x(), scale.y()) +
                                  visibility_occlusion_margin_;
            if (!segmentIntersectsVerticalCylinder(observer, target, center,
                                                   radius, scale.z()))
              continue;
            // 本轮 P1：先登记"首次进入"证据（保持既有语义不变），再维护
            // blocker 的进出区间。注意这里不再 break：区间必须在整段预测
            // 视野上扫描完才能得到真正的 exit。
            if (!raw_dynamic_los_blocked)
            {
              raw_dynamic_los_blocked = true;
              raw_dynamic_los_time = relative_time;
              raw_dynamic_los_id = id;
              raw_dynamic_observer = observer;
              raw_dynamic_target = target;
              raw_dynamic_center = center;
              raw_dynamic_scale = scale;
            }
            if (id < static_cast<int>(per_blocker_first_contact.size()) &&
                !std::isfinite(per_blocker_first_contact[id]))
              per_blocker_first_contact[id] = relative_time;
            if (raw_los_occlusion_enter_rel_ < 0.0)
              raw_los_occlusion_enter_rel_ = relative_time;
            raw_los_occlusion_exit_rel_ = relative_time;
            raw_los_occlusion_interval_observed_ = true;
          }
        }
        if (relative_time >= raw_los_interval_scan_horizon - 1.0e-9)
          break;
      }
    }
    // 本轮修复 4：独立判定"当前帧"是否真的被挡。
    // 语义区分：future predicted occlusion 用于提前换侧；current already
    // blocked 用于恢复型 topology。预测事件结束不等于实际已恢复，所以只要
    // 当前帧仍被挡，本轮就必须持续产生 LOS recovery authority。
    // 本轮修复（current LOS recovery 短路）：current 判定必须与"未来扫描是否
    // 命中"完全独立。之前这里是 `if (!raw_static_los_blocked && ...)`，而
    // raw_static_los_blocked 是整段未来视野上的"是否曾经被挡"，于是只要未来
    // 某一刻会遮挡（哪怕现在完全通畅），current 采样就被短路跳过；
    // current_raw_los_blocked_ 只在"整段未来都通畅"时才有机会为真。结果
    // "当前已处于遮挡 → 本轮必须持续建立恢复 authority"这条语义无法成立。
    // 现在 current 判定无条件执行，并且观测者取当前真实执行世界状态（下方
    // 优先使用本机当前 odom，而不是未来候选/标称轨迹的 getPos(0)）。
    current_raw_los_blocked_ = false;
    current_raw_los_blocker_id_ = -1;
    current_raw_los_blocker_dynamic_ = false;
    {
      // 观测者必须是"当前真实执行世界状态"。本批次的 start_pt 正是调用方
      // 从本机 odom 传来的当前实测位置（不是未来候选的 getPos(0)，也不是
      // 标称轨迹起点），因此 current LOS 判定与未来候选激活无关。
      // 若采样点不可用则退回标称轨迹的 0 时刻，保证判定仍然执行而不是被跳过。
      const bool current_observer_from_odom = start_pt.allFinite();
      const Eigen::Vector3d current_los_observer_position =
          current_observer_from_odom
              ? start_pt
              : (nominal_visibility_trajectory_available
                     ? nominal_visibility_trajectory.getPos(0.0)
                     : Eigen::Vector3d::Constant(
                           std::numeric_limits<double>::quiet_NaN()));
      current_los_observer_source_is_odom_ = current_observer_from_odom;
      StaticLosWitness current_witness;
      double current_clearance = std::numeric_limits<double>::infinity();
      const Eigen::Vector3d current_target = target_at_prediction_epoch;
      if (current_los_observer_position.allFinite() && current_target.allFinite() &&
          ploy_traj_opt_->queryStaticLosClearance(current_los_observer_position,
                                                  current_target,
                                                  current_clearance,
                                                  current_witness) &&
          current_witness.valid && std::isfinite(current_clearance) &&
          current_clearance <= visibility_occlusion_margin_)
      {
        current_raw_los_blocked_ = true;
        current_raw_los_blocker_id_ = current_witness.primitive_index;
      }
    }
    if (!current_raw_los_blocked_ && obj_predictor_)
    {
      const Eigen::Vector3d current_los_observer_position =
          current_los_observer_source_is_odom_
              ? start_pt
              : (nominal_visibility_trajectory_available
                     ? nominal_visibility_trajectory.getPos(0.0)
                     : Eigen::Vector3d::Constant(
                           std::numeric_limits<double>::quiet_NaN()));
      const Eigen::Vector3d current_target = target_at_prediction_epoch;
      if (current_los_observer_position.allFinite() && current_target.allFinite())
      {
        for (int id = 0; id < obj_predictor_->getObjNums(); ++id)
        {
          if (!obj_predictor_->hasPrediction(id))
            continue;
          const Eigen::Vector3d center =
              obj_predictor_->evaluateConstVel(id, planning_prediction_epoch);
          const Eigen::Vector3d scale = obj_predictor_->getObjScale(id);
          if (!center.allFinite() || !scale.allFinite() ||
              scale.x() <= 0.0 || scale.y() <= 0.0 || scale.z() <= 0.0)
            continue;
          const double radius = 0.5 * std::max(scale.x(), scale.y()) +
                                visibility_occlusion_margin_;
          if (segmentIntersectsVerticalCylinder(current_los_observer_position,
                                                current_target, center, radius,
                                                scale.z()))
          {
            current_raw_los_blocked_ = true;
            current_raw_los_blocker_id_ = id;
            current_raw_los_blocker_dynamic_ = true;
            break;
          }
        }
      }
    }
    if (current_raw_los_blocked_)
    {
      // 当前已处于遮挡中：把恢复窗口起点钉在 0（现在就恢复），
      // 并保留真实 exit（若扫描视野内未见退出则覆盖到可见地平线）。
      ++current_raw_los_blocked_count_;
      raw_los_occlusion_interval_observed_ = true;
      raw_los_occlusion_enter_rel_ = 0.0;
      if (!std::isfinite(raw_los_occlusion_exit_rel_) ||
          raw_los_occlusion_exit_rel_ <= 0.0)
        raw_los_occlusion_exit_rel_ = raw_los_interval_scan_horizon;
      if (raw_dynamic_los_id < 0 && current_raw_los_blocker_dynamic_)
        raw_dynamic_los_id = current_raw_los_blocker_id_;
      ROS_WARN_THROTTLE(0.5,
          "[current-los-blocked] drone=%d blocker_id=%d dynamic=%d "
          "action=CURRENT_STATE_RECOVERY_AUTHORITY retained_until_los_clear=1",
          pp_.drone_id, current_raw_los_blocker_id_,
          static_cast<int>(current_raw_los_blocker_dynamic_));
    }
    // 本轮 P1：把首个遮挡 blocker 的区间作为候选观察面的有效窗口依据。
    if (raw_los_occlusion_interval_observed_ && raw_dynamic_los_id >= 0 &&
        raw_dynamic_los_id < static_cast<int>(per_blocker_first_contact.size()) &&
        std::isfinite(per_blocker_first_contact[raw_dynamic_los_id]))
      raw_los_occlusion_enter_rel_ = per_blocker_first_contact[raw_dynamic_los_id];
    if (raw_los_occlusion_interval_observed_ &&
        raw_los_occlusion_exit_rel_ < raw_los_occlusion_enter_rel_)
      raw_los_occlusion_exit_rel_ = raw_los_occlusion_enter_rel_;
    if (raw_los_occlusion_interval_observed_)
    {
      ROS_INFO("[los-occlusion-interval] drone=%d generation=%lu blocker_id=%d "
               "enter_rel=%.6f exit_rel=%.6f duration=%.6f dynamic_first_contact=%.6f "
               "visibility_authority_end=%.6f scan_horizon=%.6f "
               "blocker_scan_complete=%d",
               pp_.drone_id, encirclement_generation, raw_dynamic_los_id,
               raw_los_occlusion_enter_rel_, raw_los_occlusion_exit_rel_,
               raw_los_occlusion_exit_rel_ - raw_los_occlusion_enter_rel_,
               raw_dynamic_los_time, visibility_authority_horizon,
               raw_los_interval_scan_horizon,
               static_cast<int>(raw_los_occlusion_exit_rel_ <
                                raw_los_interval_scan_horizon - 1.0e-6));
    }
    if (raw_static_los_blocked)
      ++raw_static_los_blocked_count_;
    if (raw_dynamic_los_blocked)
      ++raw_dynamic_los_blocked_count_;
    if (raw_static_los_blocked || raw_dynamic_los_blocked)
      ROS_INFO("[raw-los-truth] drone=%d generation=%lu static_blocked=%d "
               "static_time=%.6f static_clearance=%.6f dynamic_blocked=%d "
               "dynamic_time=%.6f dynamic_id=%d authority_end=%.6f "
               "raw_static_count=%lu raw_dynamic_count=%lu",
               pp_.drone_id, encirclement_generation,
               static_cast<int>(raw_static_los_blocked), raw_static_los_time,
               raw_static_los_clearance,
               static_cast<int>(raw_dynamic_los_blocked), raw_dynamic_los_time,
               raw_dynamic_los_id, visibility_authority_horizon,
               raw_static_los_blocked_count_, raw_dynamic_los_blocked_count_);

    // Single time-base consistency witness.  RAW_LOS_TARGET is the target the
    // truth sampler used; the other three are what the optimizer cost, the
    // candidate visibility report and evaluateDynamicRisk receive for this same
    // batch epoch, so they must be the same world position.
    if (std::isfinite(planning_prediction_epoch) &&
        (timebase_consistency_last_log_ <= 0.0 ||
         planning_prediction_epoch - timebase_consistency_last_log_ > 0.5))
    {
      timebase_consistency_last_log_ = planning_prediction_epoch;
      const Eigen::Vector3d dynamic_risk_target =
          targetPositionAt(planning_prediction_epoch);
      ROS_INFO("[timebase-consistency] drone=%d epoch=%.6f "
               "RAW_LOS_TARGET=(%.6f,%.6f,%.6f) "
               "OPTIMIZER_TARGET=(%.6f,%.6f,%.6f) "
               "CANDIDATE_VIS_TARGET=(%.6f,%.6f,%.6f) "
               "DYNAMIC_RISK_TARGET=(%.6f,%.6f,%.6f)",
               pp_.drone_id, planning_prediction_epoch,
               target_at_prediction_epoch.x(), target_at_prediction_epoch.y(),
               target_at_prediction_epoch.z(),
               target_at_prediction_epoch.x(), target_at_prediction_epoch.y(),
               target_at_prediction_epoch.z(),
               target_at_prediction_epoch.x(), target_at_prediction_epoch.y(),
               target_at_prediction_epoch.z(), dynamic_risk_target.x(),
               dynamic_risk_target.y(), dynamic_risk_target.z());
    }

    // Raw LOS is measured for every hypothesis so its geometry remains
    // observable in diagnostics.  Only hypothesis zero represents the
    // authoritative rolling NOMINAL, though.  A cooperative/team alternative
    // may be occluded along its own trial geometry, but that must remain a
    // quality signal; it cannot manufacture a current LOS conflict and
    // recursively dispatch another executable SIDE search.
    const bool los_descriptor_authority =
        nominal_baseline && ablation_config_.los_topology;
    const bool raw_static_los_current = raw_static_los_blocked &&
        raw_static_los_time <= visibility_authority_horizon + 1.0e-6;
    const bool raw_dynamic_los_current = raw_dynamic_los_blocked &&
        raw_dynamic_los_time <= visibility_authority_horizon + 1.0e-6;
    // Directional J_vis remains a continuous P/T objective only.  N/L/R
    // authority comes exclusively from raw BODY/LOS geometry.
    const bool visibility_topology_trigger = los_descriptor_authority &&
        (raw_static_los_current || raw_dynamic_los_current);
    const uint32_t reason_mask_before_raw_los_merge =
        nominal_result.conflict.reason_mask;
    StaticLosWitness static_visibility_witness;
    bool static_visibility_witness_valid =
        los_descriptor_authority && raw_static_los_current;
    bool static_visibility_current =
        los_descriptor_authority && raw_static_los_current;
    // K3 Local recovery 授予(第三轮 authority 清理):事件由 Local 自主检测
    // (margin crossing 锚定窗口),授予只要求"当前快照的 raw witness
    // (STATIC 或 DYNAMIC LOS)与 Local 事件窗口相交"。Team escalation hint
    // 不再参与授予(M2 repair_k==2 由 updateLocalK3RecoveryEvent 抢占闭合)。
    // Feedback117(§8 same-cycle dispatch):事件检测/刷新现在发生在本轮
    // topology dispatch 读取事件之前——刚跨过 margin 的新风险可以在同一个
    // planning revision 内获得 SIDE 授权,不再"finalize 后才建事件、下一
    // cycle 才有 SIDE"。finalize 处的调用保留(update 幂等:active 事件只
    // 做窗口延长刷新,不会重复建事件)。
    bool k3_event_created_this_cycle = false;
    if (k3_local_escalation_enabled_)
    {
      const bool k3_event_active_before = k3_event_.active;
      updateLocalK3RecoveryEvent();
      k3_event_created_this_cycle =
          !k3_event_active_before && k3_event_.active;
      if (k3_event_created_this_cycle)
        ++k3_same_cycle_event_created_count_;
    }
    bool escalation_grant = false;
    const bool m2_team_authority_active =
        (latest_handoff_contract_active_ && latest_handoff_repair_k_ == 2) ||
        (team_refinement_open_ && team_refinement_.repair_k == 2);
    if (k3_local_escalation_enabled_ && k3_event_.active)
    {
      if (m2_team_authority_active)
      {
        k3_event_.m2_preempted = true;
        closeK3EscalationEvent("M2_PREEMPTED");
      }
      else
      {
        const bool target_revision_stale =
            k3_event_.target_prediction_revision != 0 &&
            geometry_target_source_identity_ != 0 &&
            geometry_target_source_identity_ !=
                k3_event_.target_prediction_revision;
        if (target_revision_stale)
          closeK3EscalationEvent("TARGET_REVISION_STALE");
        else
        {
          confirmK3LocalActivation();
          checkK3EscalationRecovery();
          if (k3_event_.recovered)
            closeK3EscalationEvent("RECOVERY_OBSERVED");
        }
      }
    }
    const bool k3_local_event_authority =
        k3_local_escalation_enabled_ && k3_event_.active &&
        !k3_event_.m2_preempted && !k3_event_.recovered &&
        !m2_team_authority_active &&
        (!team_refinement_open_ || team_refinement_.repair_k != 2);
    if (los_descriptor_authority && k3_local_event_authority &&
        (raw_static_los_blocked || raw_dynamic_los_blocked))
    {
      const double window_begin =
          k3_event_.window_begin.toSec() - 0.5;
      const double window_end =
          k3_event_.window_end.toSec() + 0.5;
      const double static_witness_world_time =
          planning_prediction_epoch + raw_static_los_time;
      const double dynamic_witness_world_time =
          planning_prediction_epoch + raw_dynamic_los_time;
      const bool static_witness_in_window =
          raw_static_los_blocked &&
          static_witness_world_time >= window_begin &&
          static_witness_world_time <= window_end;
      const bool dynamic_witness_in_window =
          raw_dynamic_los_blocked &&
          dynamic_witness_world_time >= window_begin &&
          dynamic_witness_world_time <= window_end;
      escalation_grant = static_witness_in_window || dynamic_witness_in_window;
      if (escalation_grant && k3_event_created_this_cycle)
      {
        // §8 telemetry: event creation and SIDE authorization in the SAME
        // revision (no one-cycle dispatch loss).
        ++k3_same_cycle_dispatch_count_;
        ROS_INFO("[K3_SAME_CYCLE_DISPATCH] drone=%d event_id=%lu "
                 "event_created_and_granted_same_revision=1",
                 pp_.drone_id,
                 static_cast<unsigned long>(k3_event_.escalation_id));
      }
      if (escalation_grant)
      {
        ++k3_event_.side_trigger_count;
        if (!k3_event_.blocker_bound)
        {
          k3_event_.blocker_dynamic = dynamic_witness_in_window &&
                                       !static_witness_in_window;
          k3_event_.blocker_id = k3_event_.blocker_dynamic
              ? raw_dynamic_los_id : raw_static_witness.primitive_index;
          k3_event_.blocker_bound = k3_event_.blocker_id >= 0;
        }
        ROS_INFO_THROTTLE(
            1.0,
            "[k3-local-dispatch-grant] drone=%d event_id=%lu source=LOCAL_EVENT "
            "motion=%s witness_world_time=%.3f window=[%.3f, %.3f] "
            "action=SIDE_DISPATCH",
            pp_.drone_id,
            static_cast<unsigned long>(k3_event_.escalation_id),
            dynamic_witness_in_window && !static_witness_in_window
                ? "DYNAMIC" : "STATIC",
            static_witness_in_window ? static_witness_world_time
                                     : dynamic_witness_world_time,
            window_begin, window_end);
      }
    }
    if (static_visibility_witness_valid)
      static_visibility_witness = raw_static_witness;
    const bool static_visibility_support = los_descriptor_authority &&
        std::isfinite(nominal_visibility_diag.directional_static_los_cost_max) &&
        nominal_visibility_diag.directional_static_los_cost_max > 1.0e-9;
    const Eigen::Vector3d static_query_observer =
        static_visibility_support && nominal_visibility_diag.directional_grad_max_position.allFinite()
            ? nominal_visibility_diag.directional_grad_max_position
            : nominal_visibility_diag.visibility_support_first_position;
    const Eigen::Vector3d static_query_target =
        static_visibility_support && nominal_visibility_diag.directional_grad_max_target.allFinite()
            ? nominal_visibility_diag.directional_grad_max_target
            : nominal_visibility_diag.visibility_support_first_target;
    if (!static_visibility_witness_valid && static_visibility_support &&
        static_query_observer.allFinite() &&
        static_query_target.allFinite())
    {
      double witness_clearance = std::numeric_limits<double>::infinity();
      StaticLosWitness soft_query_witness;
      const bool static_visibility_query_valid =
          ploy_traj_opt_->queryStaticLosClearance(
          static_query_observer, static_query_target,
          witness_clearance, soft_query_witness);
      // 本轮修复 1：连续 visibility cost 只进入 MINCO / 连续优化，不再获得
      // SIDE topology 权限。 之前这里把一个"方向可见性代价大于 1e-9"的软
      // 梯度极大值点查询结果直接升级成 static_visibility_witness_valid，
      // 于是 directional_static=0.02836 这类微小代价也能生成
      // CONFLICT_LOS_OCCLUSION，进而触发完整的 LEFT/RIGHT observation plane
      // 与 SIDE 搜索（约 1 m 级横移）。J_vis 本身不删除：下面的诊断继续
      // 给出该点的 clearance，供连续优化与遥测使用。
      ++soft_vis_nonzero_count_;
      if (static_visibility_query_valid && soft_query_witness.valid)
        ROS_INFO_THROTTLE(
            0.5,
            "[soft-visibility-continuous-only] drone=%d action=NO_TOPOLOGY_AUTHORITY "
            "directional_static=%.6g witness_clearance=%.6f "
            "observer=(%.3f,%.3f,%.3f) target=(%.3f,%.3f,%.3f) "
            "raw_static_los_blocked=%d",
            pp_.drone_id,
            nominal_visibility_diag.directional_static_los_cost_max,
            witness_clearance, static_query_observer.x(),
            static_query_observer.y(), static_query_observer.z(),
            static_query_target.x(), static_query_target.y(),
            static_query_target.z(), static_cast<int>(raw_static_los_blocked));
    }
    else if (static_visibility_support)
    {
      // 软 visibility 代价存在但没有真实遮挡：只作审计计数，不产生任何
      // topology 证据。
      ++soft_vis_nonzero_count_;
    }
    bool static_body_support_active = false;
    double static_body_conflict_time = 0.0;
    Eigen::Vector3d static_body_conflict_position = Eigen::Vector3d::Zero();
    if (grid_map_ && nominal_visibility_trajectory_available)
    {
      const double body_end = touch_goal
                                  ? nominal_visibility_trajectory.getTotalDuration()
                                  : std::min(nominal_visibility_trajectory.getTotalDuration(),
                                             0.80 * nominal_visibility_trajectory.getTotalDuration());
      for (double sample = 0.0;; sample += std::max(0.05, risk_sample_dt_))
      {
        const double t = std::min(sample, body_end);
        const Eigen::Vector3d p = nominal_visibility_trajectory.getPos(t);
        if (grid_map_->getInflateOccupancy(p) != 0)
        {
          static_body_support_active = true;
          static_body_conflict_time = t;
          static_body_conflict_position = p;
          break;
        }
        if (t >= body_end - 1.0e-6)
          break;
      }
    }
    if (static_visibility_witness_valid || static_body_support_active)
    {
      nominal_result.conflict.valid = true;
      nominal_result.conflict.source_snapshot_time = planning_prediction_epoch;
      nominal_result.conflict.first_risk_world_time = planning_prediction_epoch +
          (static_visibility_witness_valid
               ? (raw_static_los_blocked
                      ? raw_static_los_time
                      : nominal_visibility_diag.visibility_support_first_trajectory_time)
               : static_body_conflict_time);
      nominal_result.conflict.risk_interval_start =
          nominal_result.conflict.first_risk_world_time;
      nominal_result.conflict.risk_interval_end =
          nominal_result.conflict.first_risk_world_time + kCandidateSideOffset;
      if (static_visibility_witness_valid)
      {
        nominal_result.conflict.reason_mask |= CONFLICT_LOS_OCCLUSION;
        nominal_result.conflict.obstacle_motion = ConflictObstacleMotion::STATIC;
        nominal_result.conflict.obstacle_identity = static_visibility_witness.primitive_index;
        nominal_result.conflict.primitive_type = static_visibility_witness.primitive_type;
        nominal_result.conflict.obstacle_name = static_visibility_witness.primitive_name;
        nominal_result.conflict.observer_position = raw_static_los_blocked
                                                        ? raw_static_observer
                                                        : static_visibility_witness.hit_point;
        nominal_result.conflict.target_position = raw_static_los_blocked
                                                      ? raw_static_target
                                                      : static_visibility_witness.hit_point;
        nominal_result.conflict.obstacle_position = static_visibility_witness.primitive_center;
        nominal_result.conflict.obstacle_geometry = static_visibility_witness.primitive_center;
        nominal_result.conflict.clearance_witness = static_visibility_witness.hit_point;
        nominal_result.conflict.clearance = static_visibility_witness.clearance;
        nominal_result.conflict.radius = static_visibility_witness.effective_radius;
        nominal_result.conflict.los_conflict_time = raw_static_los_blocked
            ? raw_static_los_time
            : nominal_visibility_diag.visibility_support_first_trajectory_time;
        nominal_result.conflict.los_first_risk_world_time =
            planning_prediction_epoch + nominal_result.conflict.los_conflict_time;
        nominal_result.conflict.los_obstacle_identity =
            static_visibility_witness.primitive_index;
        nominal_result.conflict.los_obstacle_motion = ConflictObstacleMotion::STATIC;
        nominal_result.conflict.los_primitive_type =
            static_visibility_witness.primitive_type;
        nominal_result.conflict.los_obstacle_position =
            static_visibility_witness.primitive_center;
        const Eigen::Vector2d static_los_target_xy = raw_static_los_blocked
            ? Eigen::Vector2d(raw_static_target.head<2>())
            : Eigen::Vector2d(static_query_target.head<2>());
        const Eigen::Vector2d forward2 =
            static_visibility_witness.primitive_center.head<2>() -
            static_los_target_xy;
        if (forward2.norm() > 1.0e-6)
        {
          const Eigen::Vector2d e = forward2.normalized();
          nominal_result.conflict.frame_forward = Eigen::Vector3d(e.x(), e.y(), 0.0);
          nominal_result.conflict.frame_right = Eigen::Vector3d(e.y(), -e.x(), 0.0);
        }
        nominal_result.conflict.target_position = raw_static_los_blocked
                                                      ? raw_static_target
                                                      : static_query_target;
        nominal_result.conflict.observer_position = raw_static_los_blocked
                                                        ? raw_static_observer
                                                        : static_query_observer;
      }
      if (static_body_support_active)
      {
        nominal_result.conflict.reason_mask |= CONFLICT_BODY_SAFETY;
        nominal_result.conflict.body_conflict_time = static_body_conflict_time;
        nominal_result.conflict.body_first_risk_world_time =
            planning_prediction_epoch + static_body_conflict_time;
        nominal_result.conflict.body_obstacle_motion = ConflictObstacleMotion::STATIC;
        nominal_result.conflict.body_obstacle_position = static_body_conflict_position;
      }
    }
    if (visibility_topology_trigger)
    {
      // 本轮：只有两类真实证据可以获得 SIDE topology 权限——BODY 冲突描述
      // 符，或 raw LOS truth 已确认的 LOS_OCCLUSION 描述符。软 visibility
      // 代价（J_vis）不再计入这里。
      const bool descriptor_has_body_now =
          (nominal_result.conflict.reason_mask & CONFLICT_BODY_SAFETY) != 0;
      const bool descriptor_has_los_now =
          (nominal_result.conflict.reason_mask & CONFLICT_LOS_OCCLUSION) != 0;
      if (descriptor_has_body_now)
        ++body_descriptor_side_trigger_count_;
      if (descriptor_has_los_now)
        ++los_descriptor_side_trigger_count_;
      ROS_INFO("[visibility-topology-trigger] drone=%d method=RAW_LOS_DESCRIPTOR "
               "directional_static=%.6g directional_dynamic=%.6g directional_fov=%.6g",
               pp_.drone_id,
               nominal_visibility_diag.directional_static_los_cost_max,
               nominal_visibility_diag.directional_dynamic_los_cost_max,
               nominal_visibility_diag.directional_fov_cost_max);
    }
    if (enable_risk_triggered_candidates_)
    {
      nominal_result.risk = dynamic_risk_at_epoch(
          nominal_result.min_jerk_opt.getTraj(), planning_prediction_epoch, touch_goal);
      if (!obj_predictor_ && nominal_result.conflict.valid)
      {
        nominal_result.risk.valid = true;
        nominal_result.risk.min_distance = std::numeric_limits<double>::infinity();
        nominal_result.risk.trigger_distance = absolute_dynamic_threshold;
        nominal_result.risk.conflict_time = static_visibility_witness_valid
                                                 ? nominal_visibility_diag.visibility_support_first_trajectory_time
                                                 : static_body_conflict_time;
        nominal_result.risk.trajectory_position = static_visibility_witness_valid
                                                       ? nominal_visibility_diag.visibility_support_first_position
                                                       : static_body_conflict_position;
      }
      if (static_body_support_active)
      {
        nominal_result.risk.valid = true;
        nominal_result.risk.triggered = true;
        nominal_result.risk.conflict_time = static_body_conflict_time;
        nominal_result.risk.trajectory_position = static_body_conflict_position;
      }
      // Keep the hard BODY witness authoritative when the same planning
      // cycle also carries a visibility trigger.  The LOS sample is separate
      // evidence and must not move the safety SIDE window earlier or later.
      const bool body_risk_triggered = nominal_result.risk.triggered;
      const double body_risk_conflict_time = nominal_result.risk.conflict_time;
      const int body_risk_obstacle_id = nominal_result.risk.obstacle_id;
      const Eigen::Vector3d body_risk_obstacle_position =
          nominal_result.risk.obstacle_position;
      if (body_risk_triggered)
      {
        nominal_result.conflict.valid = true;
        nominal_result.conflict.reason_mask |= CONFLICT_BODY_SAFETY;
        nominal_result.conflict.body_conflict_time = body_risk_conflict_time;
        nominal_result.conflict.body_first_risk_world_time =
            planning_prediction_epoch + body_risk_conflict_time;
        nominal_result.conflict.body_obstacle_identity = body_risk_obstacle_id;
        nominal_result.conflict.body_obstacle_motion =
            (body_risk_obstacle_id >= 0 && obj_predictor_)
                ? ConflictObstacleMotion::DYNAMIC
                : (static_body_support_active ? ConflictObstacleMotion::STATIC
                                               : ConflictObstacleMotion::UNKNOWN);
        nominal_result.conflict.body_obstacle_position = body_risk_obstacle_position;
      }
      // Visibility is evidence for the ConflictDescriptor, not BODY risk.
      // In particular, a static LOS witness must not manufacture a generic
      // DynamicRiskInfo::triggered event.  BODY risk remains owned solely by
      // the physical body evaluator above; LOS dispatch is decided from the
      // descriptor and the bounded authority horizon below.
      if (visibility_topology_trigger &&
          (static_visibility_witness_valid || static_body_support_active) &&
          !body_risk_triggered)
      {
        const double visibility_time =
            nominal_visibility_diag.visibility_support_first_trajectory_time;
        if (std::isfinite(visibility_time))
        {
          ROS_INFO("[visibility-topology-witness] drone=%d conflict_time=%.6f "
                   "authority_horizon=%.6f current=%d source=LOS_DESCRIPTOR_ONLY",
                   pp_.drone_id, visibility_time, visibility_authority_horizon,
                   static_cast<int>(static_visibility_current));
        }
      }
      else if (body_risk_triggered)
      {
        ROS_INFO("[visibility-topology-witness] drone=%d conflict_time=%.6f "
                 "source=BODY_RISK_PRESERVED los_time=%.6f",
                 pp_.drone_id, body_risk_conflict_time,
                 nominal_result.conflict.los_conflict_time);
      }
      if (body_risk_triggered &&
          (nominal_result.conflict.reason_mask & CONFLICT_BODY_SAFETY) == 0)
      {
        nominal_result.conflict.valid = true;
        nominal_result.conflict.reason_mask |= CONFLICT_BODY_SAFETY;
        nominal_result.conflict.obstacle_motion =
            obj_predictor_ ? ConflictObstacleMotion::DYNAMIC
                           : ConflictObstacleMotion::UNKNOWN;
        nominal_result.conflict.obstacle_identity = nominal_result.risk.obstacle_id;
        nominal_result.conflict.obstacle_position = nominal_result.risk.obstacle_position;
        nominal_result.conflict.observer_position = nominal_result.risk.trajectory_position;
        nominal_result.conflict.first_risk_world_time = planning_prediction_epoch +
            nominal_result.risk.conflict_time;
        nominal_result.conflict.risk_interval_start = nominal_result.conflict.first_risk_world_time;
        nominal_result.conflict.risk_interval_end = nominal_result.conflict.first_risk_world_time +
            kCandidateSideOffset;
        nominal_result.conflict.body_conflict_time = nominal_result.risk.conflict_time;
        nominal_result.conflict.body_first_risk_world_time =
            nominal_result.conflict.first_risk_world_time;
        nominal_result.conflict.body_obstacle_identity = nominal_result.risk.obstacle_id;
        nominal_result.conflict.body_obstacle_motion =
            obj_predictor_ ? ConflictObstacleMotion::DYNAMIC
                           : ConflictObstacleMotion::UNKNOWN;
        nominal_result.conflict.body_obstacle_position = nominal_result.risk.obstacle_position;
      }
      const bool dynamic_visibility_witness_valid =
          los_descriptor_authority && raw_dynamic_los_current;
      if (dynamic_visibility_witness_valid)
      {
        const bool body_reason_already_present =
            (nominal_result.conflict.reason_mask & CONFLICT_BODY_SAFETY) != 0;
        const bool replace_los =
            (nominal_result.conflict.reason_mask & CONFLICT_LOS_OCCLUSION) == 0 ||
            raw_dynamic_los_time < nominal_result.conflict.los_conflict_time;
        if (replace_los)
        {
          nominal_result.conflict.valid = true;
          nominal_result.conflict.reason_mask |= CONFLICT_LOS_OCCLUSION;
          nominal_result.conflict.los_obstacle_motion = ConflictObstacleMotion::DYNAMIC;
          nominal_result.conflict.los_obstacle_identity = raw_dynamic_los_id;
          nominal_result.conflict.los_obstacle_position = raw_dynamic_center;
          nominal_result.conflict.los_primitive_type = 1;
          nominal_result.conflict.los_conflict_time = raw_dynamic_los_time;
          nominal_result.conflict.los_first_risk_world_time =
              planning_prediction_epoch + raw_dynamic_los_time;
          if (!body_reason_already_present)
          {
            nominal_result.conflict.obstacle_motion = ConflictObstacleMotion::DYNAMIC;
            nominal_result.conflict.obstacle_identity = raw_dynamic_los_id;
            nominal_result.conflict.obstacle_position = raw_dynamic_center;
            nominal_result.conflict.obstacle_geometry = raw_dynamic_scale;
            nominal_result.conflict.first_risk_world_time =
                nominal_result.conflict.los_first_risk_world_time;
          }
          nominal_result.conflict.target_position = raw_dynamic_target;
          nominal_result.conflict.observer_position = raw_dynamic_observer;
          const Eigen::Vector2d axis =
              raw_dynamic_center.head<2>() - raw_dynamic_target.head<2>();
          if (axis.norm() > 1.0e-6)
          {
            const Eigen::Vector2d e = axis.normalized();
            nominal_result.conflict.frame_forward = Eigen::Vector3d(e.x(), e.y(), 0.0);
            nominal_result.conflict.frame_right = Eigen::Vector3d(e.y(), -e.x(), 0.0);
          }
          nominal_result.conflict.radius =
              0.5 * std::max(raw_dynamic_scale.x(), raw_dynamic_scale.y()) +
              visibility_occlusion_margin_;
        }
      }
      const bool spatial_visibility_trigger =
          (static_visibility_witness_valid && static_visibility_current) ||
          dynamic_visibility_witness_valid || escalation_grant;
      const bool dynamic_body_evidence =
          body_risk_triggered && !static_body_support_active &&
          (nominal_result.conflict.body_obstacle_motion ==
               ConflictObstacleMotion::DYNAMIC ||
           (obj_predictor_ && nominal_result.risk.obstacle_id >= 0));
      const bool dynamic_body_topology_trigger =
          dynamic_body_evidence && ablation_config_.dynamic_body_topology;
      const bool body_topology_trigger =
          static_body_support_active ||
          (body_risk_triggered &&
           (!dynamic_body_evidence || dynamic_body_topology_trigger));
      if (dynamic_body_topology_trigger)
        traj_utils::incrementAblationCounter(
            traj_utils::AblationCounter::DYNAMIC_BODY_TOPOLOGY_DISPATCH,
            "planner_manager", ablation_config_);
      if (body_risk_triggered && !body_topology_trigger)
      {
        // This ablation removes only DYNAMIC BODY's authority to manufacture
        // L/R.  Prediction, soft risk and every hard dynamic preflight remain
        // unchanged and continue to reject unsafe NOMINAL trajectories.
        nominal_result.conflict.reason_mask &= ~CONFLICT_BODY_SAFETY;
        if (nominal_result.conflict.reason_mask == CONFLICT_NONE)
          nominal_result.conflict.valid = false;
        ROS_INFO("[ablation-branch] component=planner_manager mode=%s "
                 "dynamic_body_detected=1 topology_dispatch=0 "
                 "hard_dynamic_safety=UNCHANGED",
                 traj_utils::toString(ablation_config_.mode));
      }
      const bool raw_los_blocked = los_descriptor_authority &&
          (raw_static_los_current || raw_dynamic_los_current);
      const bool descriptor_has_los =
          (nominal_result.conflict.reason_mask & CONFLICT_LOS_OCCLUSION) != 0;
      if (raw_los_blocked)
      {
        ++los_witness_created_count_;
        if (descriptor_has_los)
          ++los_descriptor_count_;
        else
        {
          ++los_detection_miss_count_;
          const bool dynamic_miss = raw_dynamic_los_blocked;
          const Eigen::Vector3d &miss_observer = dynamic_miss
              ? raw_dynamic_observer : raw_static_observer;
          const Eigen::Vector3d &miss_target = dynamic_miss
              ? raw_dynamic_target : raw_static_target;
          const Eigen::Vector3d miss_blocker = dynamic_miss
              ? raw_dynamic_center : raw_static_witness.primitive_center;
          const double miss_time = dynamic_miss
              ? raw_dynamic_los_time : raw_static_los_time;
          ROS_ERROR("[los-detection-miss] uav_id=%d planning_generation=%lu "
                    "sample_world_time=%.9f relative_time=%.6f type=%s blocker_id=%d "
                    "blocker_position=(%.6f,%.6f,%.6f) target_position=(%.6f,%.6f,%.6f) "
                    "uav_position=(%.6f,%.6f,%.6f) raw_intersection=1 raw_clearance=%.6f "
                    "witness_created=1 witness_valid=1 witness_conflict_time=%.6f "
                    "authority_end=%.6f reason_mask_before_merge=%u reason_mask_after_merge=%u "
                    "drop_stage=CONFLICT_DESCRIPTOR_MERGE drop_reason=LOS_REASON_NOT_ADDED count=%lu",
                    pp_.drone_id, encirclement_generation,
                    planning_prediction_epoch + miss_time, miss_time,
                    dynamic_miss ? "DYNAMIC" : "STATIC",
                    dynamic_miss ? raw_dynamic_los_id
                                 : raw_static_witness.primitive_index,
                    miss_blocker.x(), miss_blocker.y(), miss_blocker.z(),
                    miss_target.x(), miss_target.y(), miss_target.z(),
                    miss_observer.x(), miss_observer.y(), miss_observer.z(),
                    dynamic_miss ? 0.0 : raw_static_los_clearance,
                    miss_time, visibility_authority_horizon,
                    reason_mask_before_raw_los_merge,
                    nominal_result.conflict.reason_mask,
                    los_detection_miss_count_);
        }
        ROS_INFO("[los-detection-audit] drone=%d raw_static=%d raw_dynamic=%d "
                 "witness_created=1 witness_valid=1 conflict_time=%.6f "
                 "in_current_authority=%d reason_added=%d descriptor_has_los=%d "
                 "observation_side_generated=PENDING witness_count=%lu "
                 "descriptor_count=%lu miss_count=%lu",
                 pp_.drone_id, static_cast<int>(raw_static_los_blocked),
                 static_cast<int>(raw_dynamic_los_blocked),
                 raw_dynamic_los_blocked ? raw_dynamic_los_time : raw_static_los_time,
                 static_cast<int>(spatial_visibility_trigger),
                 static_cast<int>(descriptor_has_los),
                 static_cast<int>(descriptor_has_los), los_witness_created_count_,
                 los_descriptor_count_, los_detection_miss_count_);
      }
      else if (!los_descriptor_authority &&
               (raw_static_los_blocked || raw_dynamic_los_blocked))
      {
        ROS_INFO("[raw-los-authority] drone=%d generation=%lu hypothesis=%d "
                 "raw_static=%d raw_dynamic=%d action=DIAGNOSTIC_ONLY",
                 pp_.drone_id, encirclement_generation,
                 encirclement_hypothesis_id,
                 static_cast<int>(raw_static_los_blocked),
                 static_cast<int>(raw_dynamic_los_blocked));
      }
      if (spatial_visibility_trigger && descriptor_has_los)
        traj_utils::incrementAblationCounter(
            traj_utils::AblationCounter::LOS_TOPOLOGY_DISPATCH,
            "planner_manager", ablation_config_);
      const bool task_topology_trigger =
          body_topology_trigger || spatial_visibility_trigger;
      // FOV-only and near-primitive soft support remain task-quality signals.
      // They may affect yaw/J_vis, but cannot create a spatial LEFT/RIGHT
      // candidate or enter the expensive topology branch.
      // A LOS-only event is dispatched through the descriptor, never by
      // manufacturing DynamicRiskInfo::triggered.  Future LOS remains a
      // forecast and therefore leaves NOMINAL untouched.
      if (!body_topology_trigger)
        nominal_result.risk.triggered = false;
      if (nominal_result.conflict.valid &&
          nominal_result.conflict.reason_mask != CONFLICT_NONE)
      {
        ROS_INFO("[conflict-descriptor] drone=%d reason_mask=%u "
                 "primary_motion=%d primary_id=%d body_motion=%d body_id=%d "
                 "body_time=%.6f los_motion=%d los_id=%d los_time=%.6f "
                 "primary_time=%.6f",
                 pp_.drone_id, nominal_result.conflict.reason_mask,
                 static_cast<int>(nominal_result.conflict.obstacle_motion),
                 nominal_result.conflict.obstacle_identity,
                 static_cast<int>(nominal_result.conflict.body_obstacle_motion),
                 nominal_result.conflict.body_obstacle_identity,
                 nominal_result.conflict.body_conflict_time,
                 static_cast<int>(nominal_result.conflict.los_obstacle_motion),
                 nominal_result.conflict.los_obstacle_identity,
                 nominal_result.conflict.los_conflict_time,
                 nominal_result.conflict.first_risk_world_time -
                     planning_prediction_epoch);
      }
      classify_candidate(nominal_result);
      if (active_candidate_metadata_valid_ &&
          traj_.local_traj.duration > 1.0e-6)
      {
        previous_safe_available = validatePreviousRemainingTrajectory(
            planning_prediction_epoch, touch_goal, previous_validation_reason,
            previous_min_dynamic_clearance);
      }

      // An accepted observation-side trajectory already carries the spatial
      // decision for this blocker.  Re-solving both sides on every rolling
      // cycle is unnecessary and can consume the predecessor handoff budget.
      // Keep this event-based (identity + active side + safety), never a fixed
      // dwell timer; BODY risk and a changed blocker still use both sides.
      const bool preserve_active_observation_side =
          capture_output == nullptr &&
          !body_topology_trigger && spatial_visibility_trigger &&
          active_candidate_metadata_valid_ &&
          active_candidate_kind_ != CandidateKind::NOMINAL &&
          active_candidate_obstacle_id_ == nominal_result.risk.obstacle_id &&
          previous_safe_available;
      if (preserve_active_observation_side)
      {
        nominal_result.safety_class = CandidateSafetyClass::INVALID;
        ROS_INFO("[observation-topology-persistence] drone=%d blocker=%d "
                 "side=%s action=KEEP_ACCEPTED_SIDE reason=SAME_BLOCKER_EVENT",
                 pp_.drone_id, nominal_result.risk.obstacle_id,
                 candidateKindName(active_candidate_kind_));
      }

      if (!nominal_result.risk.valid)
      {
        ++non_risk_count_;
        const bool history_reset = non_risk_count_ >= 3;
        if (history_reset)
        {
          last_candidate_kind_ = CandidateKind::NOMINAL;
          last_candidate_obstacle_id_ = -1;
          non_risk_count_ = 0;
        }
        ROS_INFO("[risk-candidate] drone=%d valid=0 selected=NOMINAL reason=no-valid-prediction "
                 "non_risk_count=%d history_reset=%d epoch=%.6f",
                 pp_.drone_id, non_risk_count_, static_cast<int>(history_reset),
                 planning_prediction_epoch);
      }
      else if (!nominal_result.risk.triggered && !task_topology_trigger &&
               nominal_result.success)
      {
        ++non_risk_count_;
        const bool history_reset = non_risk_count_ >= 3;
        if (history_reset)
        {
          last_candidate_kind_ = CandidateKind::NOMINAL;
          last_candidate_obstacle_id_ = -1;
          non_risk_count_ = 0;
        }
        ROS_INFO("[risk-candidate] drone=%d triggered=0 obs=%d nominal_dist=%.3f selected=NOMINAL "
                 "non_risk_count=%d history_reset=%d epoch=%.6f",
                 pp_.drone_id, nominal_result.risk.obstacle_id, nominal_result.risk.min_distance,
                 non_risk_count_, static_cast<int>(history_reset), planning_prediction_epoch);
      }
      else
      {
        non_risk_count_ = 0;
        ++risk_triggered_count_;
        const bool body_conflict_active =
            (nominal_result.conflict.reason_mask & CONFLICT_BODY_SAFETY) != 0 &&
            body_topology_trigger;
        ROS_INFO("[threat-dispatch] drone=%d reason=%s body=%d los=%d "
                 "conflict_time_body=%.6f conflict_time_los=%.6f "
                 "authority_horizon=%.6f future_los=%d",
                 pp_.drone_id,
                 body_conflict_active && spatial_visibility_trigger ? "BOTH" :
                     (body_conflict_active ? "BODY" :
                          (spatial_visibility_trigger ? "LOS" : "NONE")),
                 static_cast<int>(body_conflict_active),
                 static_cast<int>(spatial_visibility_trigger),
                 nominal_result.conflict.body_conflict_time,
                 nominal_result.conflict.los_conflict_time,
                 visibility_authority_horizon, 0);
        ROS_INFO("[trajectory-fallback] cycle=%lu active_traj_id=%d previous_type=%s "
                 "age=%.3f remaining_duration=%.3f static_valid=%d dynamic_valid=%d "
                 "dynamics_valid=%d min_dynamic_clearance=%.4f new_recovery_success=UNKNOWN "
                 "action=%s%s%s",
                 risk_triggered_count_, traj_.local_traj.traj_id,
                 candidateKindName(active_candidate_kind_),
                 std::max(0.0, ros::Time::now().toSec() - traj_.local_traj.start_time),
                 std::max(0.0, traj_.local_traj.end_time - ros::Time::now().toSec()),
                 static_cast<int>(previous_safe_available),
                 static_cast<int>(previous_safe_available),
                 static_cast<int>(previous_safe_available),
                 previous_min_dynamic_clearance,
                 previous_safe_available ? "PENDING_KEEP_OR_REPLACE" : "PREVIOUS_INVALID",
                 previous_safe_available ? "" : " reason=", previous_safe_available ? "" : previous_validation_reason.c_str());
        const poly_traj::Trajectory nominal_traj = nominal_result.min_jerk_opt.getTraj();
        Eigen::Vector3d tangent = nominal_traj.getVel(nominal_result.risk.conflict_time);
        tangent(2) = 0.0;
        if (tangent.norm() < 1.0e-3)
        {
          tangent = local_target_pt - start_pt;
          tangent(2) = 0.0;
        }

        bool preferred_side_valid = tangent.norm() >= 1.0e-3;
        int preferred_side = 0;
        double lateral = 0.0;
        const double lateral_deadband = 0.25 * nominal_result.risk.trigger_distance;
        if (preferred_side_valid)
        {
          tangent.normalize();
          Eigen::Vector3d local_side_direction = tangent.cross(Eigen::Vector3d::UnitZ());
          local_side_direction(2) = 0.0;
          preferred_side_valid = local_side_direction.norm() >= 1.0e-3;
          if (preferred_side_valid)
          {
            local_side_direction.normalize();
            const Eigen::Vector3d obstacle_offset =
                nominal_result.risk.obstacle_position - nominal_result.risk.trajectory_position;
            lateral = obstacle_offset.dot(local_side_direction);
            preferred_side_valid = std::abs(lateral) >= lateral_deadband;
            if (preferred_side_valid)
            {
              const Eigen::Vector3d escape_direction =
                  lateral > 0.0 ? -local_side_direction : local_side_direction;
              Eigen::Vector3d bias_plus_direction =
                  (local_target_pt - start_pt).cross(Eigen::Vector3d::UnitZ());
              bias_plus_direction(2) = 0.0;
              preferred_side_valid = bias_plus_direction.norm() >= 1.0e-3;
              if (preferred_side_valid)
              {
                bias_plus_direction.normalize();
                const double side_alignment = escape_direction.dot(bias_plus_direction);
                preferred_side_valid = std::abs(side_alignment) >= 1.0e-3;
                if (preferred_side_valid)
                  preferred_side = side_alignment > 0.0 ? 1 : -1;
              }
            }
          }
        }

        bool plus_attempted = false;
        bool minus_attempted = false;

        // Compare candidate geometry at equal normalized phases. This is
        // diagnostics only; it does not participate in candidate selection.
        const auto max_lateral_deviation = [](const poly_traj::Trajectory &lhs,
                                              const poly_traj::Trajectory &rhs) {
          double max_deviation = 0.0;
          constexpr int kSamples = 21;
          for (int i = 0; i < kSamples; ++i)
          {
            const double phase = static_cast<double>(i) / (kSamples - 1);
            const Eigen::Vector3d lhs_pos = lhs.getPos(phase * lhs.getTotalDuration());
            const Eigen::Vector3d rhs_pos = rhs.getPos(phase * rhs.getTotalDuration());
            max_deviation = std::max(max_deviation,
                                     (lhs_pos.head<2>() - rhs_pos.head<2>()).norm());
          }
          return max_deviation;
        };

        const auto conflict_pair_lateral_deviation =
            [&](const poly_traj::Trajectory &lhs, const poly_traj::Trajectory &rhs) {
              const double nominal_duration = nominal_traj.getTotalDuration();
              const double phase = nominal_duration > 1.0e-6
                                       ? std::max(0.0, std::min(1.0,
                                             nominal_result.risk.conflict_time /
                                                 nominal_duration))
                                       : 0.0;
              const Eigen::Vector3d lhs_pos = lhs.getPos(phase * lhs.getTotalDuration());
              const Eigen::Vector3d rhs_pos = rhs.getPos(phase * rhs.getTotalDuration());
              return (lhs_pos.head<2>() - rhs_pos.head<2>()).norm();
            };

        const auto conflict_lateral_deviation =
            [&](const poly_traj::Trajectory &candidate) {
              return conflict_pair_lateral_deviation(candidate, nominal_traj);
            };

        const auto sampled_trajectory_length =
            [](const poly_traj::Trajectory &trajectory) {
              if (trajectory.getPieceNum() <= 0)
                return -1.0;
              constexpr int kSamples = 40;
              double length = 0.0;
              Eigen::Vector3d previous = trajectory.getPos(0.0);
              for (int index = 1; index <= kSamples; ++index)
              {
                const double phase = static_cast<double>(index) / kSamples;
                const Eigen::Vector3d current =
                    trajectory.getPos(phase * trajectory.getTotalDuration());
                length += (current - previous).norm();
                previous = current;
              }
              return length;
            };

        // Diagnostic only: sample the already-defined side profile against the
        // same prediction epoch before MINCO can alter it. It has no effect on
        // candidate generation, selection, or optimizer inputs.
        const auto log_side_space = [&](const int side,
                                        const double conflict_progress,
                                        const double guidance_window) {
          constexpr std::array<double, 5> kAlphaSamples = {{0.0, 0.25, 0.50, 0.75, 1.0}};
          constexpr double kSideOffset = kCandidateSideOffset;
          const double nominal_duration = nominal_traj.getTotalDuration();
          Eigen::Vector3d side_direction =
              (local_target_pt - start_pt).cross(Eigen::Vector3d::UnitZ());
          side_direction.z() = 0.0;
          if (nominal_duration <= 1.0e-6 || side_direction.norm() < 1.0e-3 ||
              !obj_predictor_ ||
              !obj_predictor_->hasPrediction(nominal_result.risk.obstacle_id))
            return;
          side_direction.normalize();

          double check_end = nominal_duration;
          if (!touch_goal)
            check_end *= (2.0 / 3.0);
          check_end = std::max(0.0, std::min(
              check_end, ploy_traj_opt_->getMovingObjPredictionHorizon()));
          std::array<double, kAlphaSamples.size()> min_clearances;
          min_clearances.fill(std::numeric_limits<double>::infinity());
          for (double sample_time = 0.0;; sample_time += risk_sample_dt_)
          {
            const double t = std::min(sample_time, check_end);
            const double u = std::max(0.0, std::min(1.0, t / nominal_duration));
            const bool in_region_window =
                std::abs(u - conflict_progress) < guidance_window;
            const double profile = in_region_window ? kSideOffset * std::sin(M_PI * u) : 0.0;
            const Eigen::Vector3d obstacle_position = obj_predictor_->evaluateConstVel(
                nominal_result.risk.obstacle_id, planning_prediction_epoch + t);
            for (size_t index = 0; index < kAlphaSamples.size(); ++index)
            {
              const Eigen::Vector3d sampled_position = nominal_traj.getPos(t) +
                  static_cast<double>(side) * kAlphaSamples[index] * profile * side_direction;
              min_clearances[index] = std::min(
                  min_clearances[index], (sampled_position - obstacle_position).norm());
            }
            if (t >= check_end - 1.0e-6)
              break;
          }

          // The current soft corridor accepts 0.75..1.25 of the initialized
          // side profile. Of the requested alpha samples, 0.75 and 1.00 are
          // therefore the feasible region probes.
          constexpr size_t kFirstRegionSample = 3;
          size_t best_region_index = kFirstRegionSample;
          for (size_t index = kFirstRegionSample + 1; index < kAlphaSamples.size(); ++index)
          {
            if (min_clearances[index] > min_clearances[best_region_index])
              best_region_index = index;
          }
          const double nominal_clearance = min_clearances.front();
          const double best_region_clearance = min_clearances[best_region_index];
          const double improvement = best_region_clearance - nominal_clearance;
          ROS_INFO("[side-space] drone=%d candidate=%s obs=%d epoch=%.6f "
                   "conflict_window=[%.3f,%.3f] nominal_clearance=%.4f "
                   "alpha_clearances=0.00:%.4f,0.25:%.4f,0.50:%.4f,0.75:%.4f,1.00:%.4f "
                   "best_region_alpha=%.2f best_region_clearance=%.4f improvement=%.4f class=%s",
                   pp_.drone_id, side > 0 ? "PLUS" : "MINUS",
                   nominal_result.risk.obstacle_id, planning_prediction_epoch,
                   std::max(0.0, nominal_result.risk.conflict_time -
                                     guidance_window * nominal_duration),
                   std::min(nominal_duration, nominal_result.risk.conflict_time +
                                                  guidance_window * nominal_duration),
                   nominal_clearance,
                   min_clearances[0], min_clearances[1], min_clearances[2],
                   min_clearances[3], min_clearances[4],
                   kAlphaSamples[best_region_index], best_region_clearance,
                   improvement, improvement >= 0.10 ? "A" : "B");
        };

        const auto optimize_side = [&](const int side, CandidateResult &result,
                                       const bool mandatory_supply) {
          const auto side_work_allowed = [&]() {
            return mandatory_supply ? mandatoryPlanningAttemptAllowed()
                                    : optionalRefinementAllowed();
          };
          if(!side_work_allowed()) return;
          // 本轮审计：soft visibility cost 只能是连续优化信号，不能授予 SIDE
          // topology 权限。计数放在 attempt_side（见下），因为"本轮 NOMINAL 是否
          // 可执行"只有在 candidate 分类之后才可用。
          result.candidate_id = ++topology_candidate_serial_;
          result.kind = side > 0 ? CandidateKind::SIDE_PLUS : CandidateKind::SIDE_MINUS;
          result.prediction_epoch = planning_prediction_epoch;
          result.conflict = nominal_result.conflict;
          const bool body_hard_conflict =
              (nominal_result.conflict.reason_mask & CONFLICT_BODY_SAFETY) != 0 &&
              (static_body_support_active ||
               (nominal_result.risk.valid &&
                nominal_result.risk.min_distance < absolute_dynamic_threshold));
          const bool los_semantics_present =
              (nominal_result.conflict.reason_mask & CONFLICT_LOS_OCCLUSION) != 0;
          const bool los_observation_geometry_valid =
              los_semantics_present &&
              nominal_result.conflict.frame_right.head<2>().norm() > 1.0e-6;
          // BODY+LOS deliberately uses the BODY/path frame for seed
          // construction, but must not discard the independent LOS witness.
          // The observation half-space is attached below as a separate
          // event-scoped Local-SFC plane.
          const bool observation_frame_for_candidate =
              los_observation_geometry_valid && !body_hard_conflict;
          const bool preserve_los_observation_constraint =
              los_observation_geometry_valid;
          bool use_observation_frame = observation_frame_for_candidate;

          const double nominal_duration = nominal_traj.getTotalDuration();
          const double conflict_progress = nominal_duration > 1.0e-6
                                               ? std::max(0.0, std::min(1.0,
                                                     nominal_result.risk.conflict_time /
                                                         nominal_duration))
                                               : 0.5;
          // A short, fixed local window keeps the side basin near the actual
          // dynamic conflict without introducing another runtime parameter.
          constexpr double kGuidanceWindowSeconds = 0.45;
          const double guidance_window = nominal_duration > 1.0e-6
                                             ? std::min(0.5, std::max(0.08,
                                                   kGuidanceWindowSeconds / nominal_duration))
                                             : 0.25;
          result.local_guidance_window = guidance_window;
          double conflict_window_start = std::max(
              0.0, nominal_result.risk.conflict_time - kGuidanceWindowSeconds);
          double conflict_window_end = std::min(
              nominal_duration, nominal_result.risk.conflict_time + kGuidanceWindowSeconds);
          // 本轮 P1：LOS 遮挡是"区间事件"，不是 BODY 那种"最近接近点事件"。
          // ±kGuidanceWindowSeconds=0.45 s 的 BODY 指导窗无法表达它：实测中
          // dynamic_los_time=1.8 s 的遮挡被压成 [0,0.45]，观察面既覆盖不到遮挡
          // 区间、也来不及让候选进入所选观察侧，最终 reached_side=false →
          // WINDOW_LIMITED / RECOVERY_NOT_REACHED → los_plane_count=0。
          // 这里用 raw LOS 扫描得到的真实遮挡区间替换窗口的时间语义。
          const bool los_occlusion_window_valid =
              los_observation_geometry_valid &&
              raw_los_occlusion_interval_observed_ &&
              std::isfinite(raw_los_occlusion_enter_rel_) &&
              std::isfinite(raw_los_occlusion_exit_rel_);
          double los_occlusion_enter_rel = -1.0;
          double los_occlusion_exit_rel = -1.0;
          if (los_occlusion_window_valid)
          {
            // 遮挡区间与 SIDE 候选都以本批次 planning_prediction_epoch 为局部
            // 时间原点（见上方 "SIDE candidates use the same local time origin
            // as the nominal risk result"），因此无需再做原点平移；保留该变量
            // 只为审计输出，避免引入第二个时间基准。
            const double epoch_offset = 0.0;
            (void)planning_prediction_epoch;
            // 场景 1：进入时刻落在候选之前（当前已在遮挡中）→ 起点 clamp 到 0。
            los_occlusion_enter_rel =
                std::max(0.0, raw_los_occlusion_enter_rel_ + epoch_offset);
            los_occlusion_exit_rel = raw_los_occlusion_exit_rel_ + epoch_offset;
            // 场景 2/3：退出超出候选时长或扫描视野内未见退出 → 截到可见地平线，
            // 但绝不因此把整条观察面删除。
            const double horizon_reach = std::max(
                nominal_duration,
                raw_los_interval_scan_horizon + epoch_offset);
            if (!std::isfinite(los_occlusion_exit_rel) ||
                los_occlusion_exit_rel <= los_occlusion_enter_rel)
              los_occlusion_exit_rel = horizon_reach;
            los_occlusion_exit_rel =
                std::min(los_occlusion_exit_rel,
                         std::max(horizon_reach, los_occlusion_enter_rel));
            if (los_occlusion_exit_rel > los_occlusion_enter_rel + 1.0e-6)
            {
              conflict_window_start = los_occlusion_enter_rel;
              conflict_window_end = los_occlusion_exit_rel;
            }
          }
          // 提前动作期限：候选必须在遮挡开始前到达所选观察侧。
          const double los_side_reach_deadline =
              los_occlusion_window_valid ? los_occlusion_enter_rel
                                         : conflict_window_start;
          result.los_occlusion_enter_time = los_occlusion_enter_rel;
          result.los_occlusion_exit_time = los_occlusion_exit_rel;
          result.los_side_reach_deadline = los_side_reach_deadline;
          if (los_occlusion_window_valid)
          {
            ROS_INFO("[los-window-semantics] drone=%d candidate_type=%s "
                     "LOS_BLOCKER_ID=%d LOS_OCCLUSION_ENTER_REL=%.6f "
                     "LOS_OCCLUSION_EXIT_REL=%.6f LOS_SIDE_REACH_DEADLINE=%.6f "
                     "LOS_PLANE_ACTIVE_START=%.6f LOS_PLANE_ACTIVE_END=%.6f "
                     "raw_dynamic_los_time=%.6f epoch_offset=%.6f "
                     "fixed_guidance_window=%.3f",
                     pp_.drone_id, side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                     nominal_result.conflict.los_obstacle_identity,
                     los_occlusion_enter_rel, los_occlusion_exit_rel,
                     los_side_reach_deadline, conflict_window_start,
                     conflict_window_end, raw_dynamic_los_time, 0.0,
                     kGuidanceWindowSeconds);
          }
          log_side_space(side, conflict_progress, guidance_window);
          bool side_duration_fallback_used = false;
          const double requested_side_offset = kCandidateSideOffset;
          double accepted_side_offset = requested_side_offset;

          // SIDE candidates use the same local time origin as the nominal
          // risk result.  Keep diagnostics at this boundary because failures
          // here occur before the optimizer can emit its phase=initial log.
          const auto log_side_preinit_failure =
              [&](const char *reason, const poly_traj::Trajectory *candidate_traj) {
                const double candidate_duration = candidate_traj != nullptr
                                                      ? candidate_traj->getTotalDuration()
                                                      : -1.0;
                const double candidate_window_start = candidate_duration > 0.0
                                                          ? std::max(0.0,
                                                                     nominal_result.risk.conflict_time -
                                                                         kGuidanceWindowSeconds)
                                                          : -1.0;
                const double candidate_window_end = candidate_duration > 0.0
                                                        ? std::min(candidate_duration,
                                                                   nominal_result.risk.conflict_time +
                                                                       kGuidanceWindowSeconds)
                                                        : -1.0;
                ROS_WARN(
                    "[candidate-preinit-failure] reason=%s drone_id=%d candidate_type=%s "
                    "obstacle_id=%d piece_num=%d variable_num=%d total_duration=%.6f "
                    "reference_duration=%.6f raw_conflict_time=%.6f raw_window_start=%.6f "
                    "raw_window_end=%.6f clipped_window_start=%.6f clipped_window_end=%.6f",
                    reason, pp_.drone_id, side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                    nominal_result.risk.obstacle_id,
                    candidate_traj != nullptr ? candidate_traj->getPieceNum() : -1,
                    candidate_traj != nullptr
                        ? 3 * std::max(0, candidate_traj->getPieceNum() - 1)
                        : -1,
                    candidate_duration, -1.0, nominal_result.risk.conflict_time,
                    std::max(0.0, nominal_result.risk.conflict_time - kGuidanceWindowSeconds),
                    nominal_result.risk.conflict_time + kGuidanceWindowSeconds,
                    candidate_window_start, candidate_window_end);
                // Feedback117 (§10): construction failures are counted once,
                // per cause, in the candidate lifecycle report.  The batch-
                // level side_construction_failed_count_ keeps its F116
                // semantics (both sides failed) upstream.
                ++side_failure_cause_count_[reason];
              };

          poly_traj::MinJerkOpt side_init_mjo;
          bool side_init_valid = false;
          bool side_static_free = false;
          bool side_dynamic_valid = false;
          // A statically valid SIDE seed whose dynamic clearance is below the
          // execution threshold is eligible only for repair.  It has no
          // execution authority until the normal optimizer and final hard
          // preflight prove it safe.
          bool side_repair_only = false;
          bool side_dynamic_invalid_after_backoff = false;
          std::string side_static_reason;
          std::string side_dynamic_reason;
          // Keep the largest-offset reference that is dynamically useful even
          // when its geometry intersects the static map.  A local A* repair
          // must preserve this dynamic homotopy instead of repairing a
          // smaller backoff candidate that has already lost its gain.
          bool astar_base_valid = false;
          poly_traj::Trajectory astar_base_traj;
          DynamicRiskInfo astar_base_risk;
          double astar_base_offset = 0.0;
          const ConstraintPoints precheck_constraints = ploy_traj_opt_->getControlPoints();
          const double map_step = grid_map_ != nullptr && grid_map_->getResolution() > 1.0e-6
                                      ? grid_map_->getResolution()
                                      : 0.1;
          // Keep a meaningful SIDE homotopy: backoff may try 0.7/0.6/0.5/0.4
          // for the default request, but must not collapse the candidate onto
          // the nominal path merely to pass the static checker.
          const double min_side_offset = std::max(map_step, requested_side_offset - 0.3);
          int side_generation_attempts = 0;
          bool astar_repair_applied = false;
          bool side_warm_start_used = false;
          std::vector<LocalSfcPlane> local_sfc_planes;
          const auto logLocalSfcFrontendFinal =
              [&](const std::vector<LocalSfcPlane> &planes,
                  const char *reason, const poly_traj::Trajectory *traj,
                  const bool static_free, const bool side_valid,
                  const bool dynamic_valid) {
                if (planes.empty())
                  return;
                const double duration =
                    traj != nullptr && traj->getPieceNum() > 0 &&
                            std::isfinite(traj->getTotalDuration())
                        ? traj->getTotalDuration()
                        : -1.0;
                ROS_INFO(
                    "[side-local-sfc-final] drone_id=%d candidate_type=%s "
                    "obstacle_id=%d planes=%zu scp_success=0 final_success=0 "
                    "reason=%s duration=%.6f static_free=%d side_valid=%d "
                    "dynamic_valid=%d corridor_violation=-1.000000 "
                    "static_violation=-1.000000 v_violation=-1.000000 "
                    "a_violation=-1.000000 j_violation=-1.000000 "
                    "clearance_gain=-1.000000",
                    pp_.drone_id, side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                    nominal_result.risk.obstacle_id, planes.size(),
                    reason != nullptr ? reason : "UNKNOWN", duration,
                    static_cast<int>(static_free), static_cast<int>(side_valid),
                    static_cast<int>(dynamic_valid));
              };
          for (double trial_offset = requested_side_offset;
               trial_offset + 1.0e-6 >= min_side_offset;
               trial_offset -= map_step)
          {
            ++side_generation_attempts;
            side_duration_fallback_used = false;
            local_sfc_planes.clear();
            // Prefer the previously committed SIDE topology for the matching
            // obstacle/branch.  The inherited remaining P/T seed is used only
            // on the first trial; other offsets retain the original fresh
            // SIDE initialization and backoff behavior.
            if (!side_warm_start_used && side_generation_attempts == 1 &&
                !flag_polyInit && accepted_state_cache_valid_ &&
                accepted_state_cache_kind_ == result.kind &&
                accepted_state_cache_obstacle_id_ == nominal_result.risk.obstacle_id)
            {
              double side_warm_age = -1.0, side_warm_remaining = -1.0;
              int side_warm_old_pieces = 0, side_warm_new_pieces = 0;
              std::string side_warm_reason;
              side_init_valid = buildWarmStartFromAccepted(
                  start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
                  side_init_mjo, side_warm_age, side_warm_remaining,
                  side_warm_old_pieces, side_warm_new_pieces, side_warm_reason);
              if (side_init_valid)
              {
                side_warm_start_used = true;
                ROS_INFO("[trajectory-warm-start] previous_traj_id=%lu age=%.6f "
                         "current_piece=0 remaining_duration=%.6f old_piece_num=%d "
                         "new_piece_num=%d topology=%s head_p_error=0 head_v_error=0 "
                         "head_a_error=0 init_max_jerk=NA init_min_dynamic_clearance=NA "
                         "action=USED_PREVIOUS_SAFE_SIDE",
                         accepted_state_cache_id_, side_warm_age, side_warm_remaining,
                         side_warm_old_pieces, side_warm_new_pieces,
                         candidateKindName(accepted_state_cache_kind_));
              }
            }
            if (!side_init_valid)
            {
              side_init_valid = computeInitState(
                  start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
                  flag_polyInit, flag_randomPolyTraj, ts, side_init_mjo, 1.0, side,
                  trial_offset,
                  use_observation_frame ? &nominal_result.conflict.frame_right : nullptr);
            }
            if (!side_init_valid)
            {
              // The previous-trajectory branch can expire while the nominal
              // trajectory is being optimized.  Rebuild this SIDE candidate
              // from the same full-horizon polynomial initializer.
              log_side_preinit_failure("SIDE_INIT_FROM_PREVIOUS_FAILED", nullptr);
              side_init_valid = computeInitState(
                  start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
                  true, flag_randomPolyTraj, ts, side_init_mjo, 1.0, side,
                  trial_offset);
            }
            if (!side_init_valid)
            {
              side_static_reason = "INVALID_SIDE_INIT";
              continue;
            }

            const poly_traj::Trajectory side_init_candidate_traj = side_init_mjo.getTraj();
            if (side_init_candidate_traj.getPieceNum() <= 0 ||
                !std::isfinite(side_init_candidate_traj.getTotalDuration()) ||
                side_init_candidate_traj.getTotalDuration() <= 1.0e-6)
            {
              side_static_reason = "INVALID_SIDE_INIT_DURATION";
              continue;
            }

            if (side_init_candidate_traj.getTotalDuration() <= conflict_window_start + 1.0e-6)
            {
              side_duration_fallback_used = true;
              const double base_duration = side_init_candidate_traj.getTotalDuration();
              const double horizon_scale =
                  std::isfinite(nominal_duration) && nominal_duration > base_duration
                      ? nominal_duration / base_duration
                      : 1.0;
              ROS_INFO("[side-duration-fallback] drone_id=%d candidate_type=%s obstacle_id=%d "
                       "conflict_time=%.6f raw_window_start=%.6f raw_window_end=%.6f "
                       "original_duration=%.6f nominal_duration=%.6f rebuilt_duration=%.6f "
                       "duration_scale=%.6f clipped_window_start=%.6f clipped_window_end=%.6f",
                       pp_.drone_id, side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                       nominal_result.risk.obstacle_id, nominal_result.risk.conflict_time,
                       std::max(0.0, nominal_result.risk.conflict_time - kGuidanceWindowSeconds),
                       nominal_result.risk.conflict_time + kGuidanceWindowSeconds,
                       base_duration, nominal_duration, -1.0, horizon_scale,
                       conflict_window_start, conflict_window_end);
              side_init_valid = computeInitState(
                  start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
                  true, flag_randomPolyTraj, ts, side_init_mjo, horizon_scale, side,
                  trial_offset,
                  use_observation_frame ? &nominal_result.conflict.frame_right : nullptr);
              if (!side_init_valid)
              {
                side_static_reason = "INVALID_SIDE_INIT_FALLBACK";
                continue;
              }
              const poly_traj::Trajectory fallback_traj = side_init_mjo.getTraj();
              ROS_INFO("[side-duration-fallback] drone_id=%d candidate_type=%s obstacle_id=%d "
                       "conflict_time=%.6f raw_window_start=%.6f raw_window_end=%.6f "
                       "original_duration=%.6f nominal_duration=%.6f rebuilt_duration=%.6f "
                       "duration_scale=%.6f clipped_window_start=%.6f clipped_window_end=%.6f",
                       pp_.drone_id, side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                       nominal_result.risk.obstacle_id, nominal_result.risk.conflict_time,
                       std::max(0.0, nominal_result.risk.conflict_time - kGuidanceWindowSeconds),
                       nominal_result.risk.conflict_time + kGuidanceWindowSeconds,
                       base_duration, nominal_duration, fallback_traj.getTotalDuration(),
                       horizon_scale, conflict_window_start, conflict_window_end);
              if (fallback_traj.getPieceNum() <= 0 ||
                  !std::isfinite(fallback_traj.getTotalDuration()) ||
                  fallback_traj.getTotalDuration() <= conflict_window_start + 1.0e-6)
              {
                side_static_reason = "WINDOW_NO_OVERLAP";
                continue;
              }
            }

            // Evaluate dynamic evidence before the static precheck so a
            // collision-prone 0.7 m reference remains available as the A*
            // repair base.  Feedback119 (RC1 修复): A* 是静态几何 repair,
            // 其资格不再要求 seed 对全部动态障碍保持 preferred 软余量
            // (getMovingObjClearance=1.1 m)——那个条件曾把 90.8% 的静态
            // 不可行 SIDE 挡在 A* 之外,直接折叠成 NO_STATIC_FEASIBLE_SIDE。
            // 资格只保留真实安全阈值 hard clearance(!hard_collision);
            // dynamic collision 由后续 P/T 优化与最终 hard preflight 判定。
            DynamicRiskInfo trial_risk = dynamic_risk_at_epoch(
                side_init_mjo.getTraj(), planning_prediction_epoch, touch_goal);
            // A* repairs only static geometry. Dynamic feasibility belongs
            // to the time-parameterized candidate and final preflight.
            if (!astar_base_valid || trial_offset > astar_base_offset + 1.0e-6)
            {
              astar_base_valid = true;
              astar_base_traj = side_init_mjo.getTraj();
              astar_base_risk = trial_risk;
              astar_base_offset = trial_offset;
            }

            // Probe the exact generated reference.  The native checker may
            // populate temporary A* rays on collision, so restore its state
            // after every trial; the accepted candidate is checked formally
            // below and owns the resulting constraint points.
            std::vector<std::pair<int, int>> precheck_segments;
            std::string precheck_reason;
            const PolyTrajOptimizer::CHK_RET precheck_ret =
                ploy_traj_opt_->finelyCheckAndSetConstraintPoints(
                    precheck_segments, side_init_mjo, true, &precheck_reason);
            side_static_free = precheck_ret != PolyTrajOptimizer::CHK_RET::ERR &&
                               precheck_reason == "OBS_FREE";
            ploy_traj_opt_->setConstraintPoints(precheck_constraints);
            if (!side_static_free)
            {
              // Observation geometry is task guidance, not a reason to lose
              // the legacy safe SIDE candidate.  If its geometric seed is
              // infeasible, retry this same sign in the validated path frame;
              // the observation witness remains attached for diagnostics.
              if (use_observation_frame && side_generation_attempts == 1)
              {
                side_init_valid = computeInitState(
                    start_pt, start_vel, start_acc, local_target_pt,
                    local_target_vel, flag_polyInit, flag_randomPolyTraj, ts,
                    side_init_mjo, 1.0, side, trial_offset, nullptr);
                if (side_init_valid)
                {
                  precheck_segments.clear();
                  precheck_reason.clear();
                  const PolyTrajOptimizer::CHK_RET fallback_precheck =
                      ploy_traj_opt_->finelyCheckAndSetConstraintPoints(
                          precheck_segments, side_init_mjo, true, &precheck_reason);
                  side_static_free = fallback_precheck != PolyTrajOptimizer::CHK_RET::ERR &&
                                     precheck_reason == "OBS_FREE";
                  ploy_traj_opt_->setConstraintPoints(precheck_constraints);
                  // The fallback is a different geometric seed even when it
                  // uses the same SIDE sign and offset.  If it remains
                  // statically blocked, the subsequent A* repair must scan
                  // this path-frame trajectory rather than the rejected
                  // observation-frame trajectory.  Recompute its dynamic
                  // evidence as well; reusing the observation seed's risk
                  // would make the A* base internally inconsistent.
                  const DynamicRiskInfo fallback_risk = dynamic_risk_at_epoch(
                      side_init_mjo.getTraj(), planning_prediction_epoch, touch_goal);
                  if (fallback_risk.valid)
                  {
                    astar_base_valid = true;
                    astar_base_traj = side_init_mjo.getTraj();
                    astar_base_risk = fallback_risk;
                    astar_base_offset = trial_offset;
                  }
                  if (side_static_free)
                  {
                    use_observation_frame = false;
                    ROS_INFO("[observation-topology-fallback] drone=%d candidate=%s "
                             "reason=OBSERVATION_SEED_STATIC_INFEASIBLE action=PATH_FRAME_SIDE",
                             pp_.drone_id, side > 0 ? "SIDE_PLUS" : "SIDE_MINUS");
                  }
                }
              }
              if (!side_static_free)
              {
                side_static_reason = precheck_reason.empty() ? "STATIC_COLLISION" : precheck_reason;
                continue;
              }
            }

            // Static LOS has a local observation topology even when the body
            // path is free.  Keep this half-space event-scoped and activate it
            // only after the seed reaches the selected side; it is not a
            // global visibility execution gate.
            if (preserve_los_observation_constraint && nominal_result.conflict.valid &&
                (nominal_result.conflict.los_obstacle_position.allFinite() ||
                 nominal_result.conflict.obstacle_position.allFinite()) &&
                nominal_result.conflict.target_position.allFinite())
            {
              const Eigen::Vector2d target_xy = nominal_result.conflict.target_position.head<2>();
              const Eigen::Vector3d los_blocker_position =
                  nominal_result.conflict.los_obstacle_identity >= 0
                      ? nominal_result.conflict.los_obstacle_position
                      : nominal_result.conflict.obstacle_position;
              const Eigen::Vector2d blocker_xy = los_blocker_position.head<2>();
              const Eigen::Vector2d axis = blocker_xy - target_xy;
              const double distance = axis.norm();
              const double radius = std::max(0.0, nominal_result.conflict.radius);
              if (distance > radius + 1.0e-6)
              {
                const Eigen::Vector2d e = axis / distance;
                const Eigen::Vector2d n_right(e.y(), -e.x());
                const double alpha = std::asin(std::min(1.0, radius / distance));
                const Eigen::Vector2d m2 =
                    -std::sin(alpha) * e + static_cast<double>(side) *
                    std::cos(alpha) * n_right;
                const Eigen::Vector3d observation_normal(m2.x(), m2.y(), 0.0);
                const Eigen::Vector3d observation_point(
                    nominal_result.conflict.target_position.x(),
                    nominal_result.conflict.target_position.y(), 0.0);
                const double duration = side_init_candidate_traj.getTotalDuration();
                // 候选进入所选观察侧的最早时刻。
                double side_reached_time = -1.0;
                bool reached_side = false;
                for (int sample = 0; sample <= 40; ++sample)
                {
                  const double t = duration * static_cast<double>(sample) / 40.0;
                  const Eigen::Vector3d p = side_init_candidate_traj.getPos(t);
                  if (observation_normal.dot(p - observation_point) >= -1.0e-4)
                  {
                    side_reached_time = t;
                    reached_side = true;
                    break;
                  }
                }
                // 离开观察侧的时刻（若在候选时长内不再离开则保持 -1）。
                // 仅作为 seed 侧向行为的诊断量：约束区间已不再被它裁剪。
                double side_left_time = -1.0;
                if (reached_side)
                {
                  for (int sample = 1; sample <= 40; ++sample)
                  {
                    const double t = side_reached_time +
                        (duration - side_reached_time) *
                            static_cast<double>(sample) / 40.0;
                    const Eigen::Vector3d p = side_init_candidate_traj.getPos(t);
                    if (observation_normal.dot(p - observation_point) < -1.0e-4)
                    {
                      side_left_time = t;
                      break;
                    }
                  }
                }
                (void)side_left_time;
                // 本轮 P1：观察面必须作用在真实遮挡区间上。
                // 之前 enter/exit 都由固定 ±0.45 s 的 BODY 指导窗决定，遮挡在
                // 1.8 s 之后时窗口是 [0,0.45]，平面根本不在遮挡时段生效。
                // 现在：起点 = max(到达观察侧时刻, 遮挡进入时刻)，
                //       终点 = min(离开观察侧时刻, 遮挡退出时刻)，
                // 并在没有任何时间重叠时判定为 RECOVERY_NOT_REACHED。
                const bool occlusion_window_available =
                    los_occlusion_window_valid &&
                    los_occlusion_exit_rel > los_occlusion_enter_rel + 1.0e-6;
                const double occlusion_start = occlusion_window_available
                                                   ? los_occlusion_enter_rel
                                                   : std::max(0.0, conflict_window_start);
                const double occlusion_end = occlusion_window_available
                                                 ? los_occlusion_exit_rel
                                                 : std::min(duration, conflict_window_end);
                // 本轮修复（LOS plane 真实时间区间）：当"真实遮挡区间"已知时，
                // 约束区间就等于该区间，不再与"seed 恰好已在正确侧"的区间取交集
                // （那等于"因为初值做不到就把约束缩到初值做得到的范围"）。
                // 语义拆分：
                //   CONSTRAINT AUTHORITY : [occlusion_start, occlusion_end]
                //   SIDE_REACH_DEADLINE  : <= occlusion_start
                // 【修正我上一版的过火之处】上一版把 `reached_side` 门整个删掉了，
                // 于是即使 seed 完全在错误侧、候选也不可能在一次 MINCO 中换侧，
                // 也会强行建面并把它当硬约束——实测这会大量产出不可行候选并让
                // 最终轨迹明显变糙。现在恢复为：
                //   * 遮挡区间已知（occlusion_window_available）时：建面区间=真实
                //     遮挡区间。seed 未到正确侧时仍建面，但由后续原有 hard 校验
                //     照常拒绝，不额外放宽也不额外收紧；
                //   * 遮挡区间未知时：沿用工程既有语义，必须有 reached_side 才建面，
                //     区间取 [max(到达侧时刻, 冲突窗起点), 冲突窗终点]。
                double enter_time = occlusion_start;
                double exit_time = occlusion_end;
                if (!occlusion_window_available)
                {
                  enter_time = std::max(occlusion_start,
                                        reached_side ? side_reached_time
                                                     : occlusion_start);
                  exit_time = occlusion_end;
                  if (reached_side && side_left_time >= 0.0)
                    exit_time = std::min(exit_time, side_left_time);
                }
                const bool window_overlaps_occlusion =
                    exit_time > enter_time + 1.0e-3;
                result.los_reached_side = reached_side;
                const bool plane_window_usable =
                    occlusion_window_available ? window_overlaps_occlusion
                                               : (reached_side &&
                                                  window_overlaps_occlusion);
                if (plane_window_usable)
                {
                  LocalSfcPlane observation_plane;
                  observation_plane.source = LocalSfcPlane::LOS_OBSERVATION_SIDE;
                  observation_plane.blocker_index =
                      nominal_result.conflict.los_obstacle_identity >= 0
                          ? nominal_result.conflict.los_obstacle_identity
                          : nominal_result.conflict.obstacle_identity;
                  observation_plane.blocker_type =
                      nominal_result.conflict.los_primitive_type != 0
                          ? nominal_result.conflict.los_primitive_type
                          : nominal_result.conflict.primitive_type;
                  observation_plane.side_sign = side;
                  observation_plane.geometry_revision = nominal_result.conflict.geometry_revision;
                  observation_plane.normal = observation_normal;
                  observation_plane.point = observation_point;
                  // guide 只用于诊断关联。seed 若尚未到达正确侧，enter_time 处
                  // 的位置可能仍在错误侧，此时取它在候选时长内的最后位置。
                  observation_plane.guide =
                      side_init_candidate_traj.getPos(
                          reached_side ? enter_time : duration);
                  observation_plane.clearance = 0.0;
                  observation_plane.active_start = enter_time;
                  observation_plane.active_end = exit_time;
                  // 该区间绑定在世界时间遮挡事件上：retime 时不得按
                  // new_duration/old_duration 统一缩放（见 validateHardConstraints）。
                  observation_plane.world_time_anchored = true;
                  // 该平面区间的世界时间零点 = 本候选的 activation。
                  observation_plane.world_anchor_time = planning_prediction_epoch;
                  local_sfc_planes.push_back(observation_plane);
                  ++los_plane_created_count_;
                  if (current_raw_los_blocked_)
                    ++current_los_blocked_with_authority_count_;
                  result.los_observation_plane_created = true;
                  ROS_INFO("[los-plane-audit] drone=%d candidate_type=%s "
                           "LOS_BLOCKER_ID=%d LOS_OCCLUSION_ENTER_REL=%.6f "
                           "LOS_OCCLUSION_EXIT_REL=%.6f LOS_SIDE_REACH_DEADLINE=%.6f "
                           "LOS_PLANE_ACTIVE_START=%.6f LOS_PLANE_ACTIVE_END=%.6f "
                           "LOS_REACHED_SIDE=%d LOS_PLANE_CREATED=1 "
                           "side_reached_time=%.6f occlusion_enter=%.6f occlusion_exit=%.6f "
                           "constraint_clipped_to_seed_side=0",
                           pp_.drone_id, side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                           observation_plane.blocker_index,
                           los_occlusion_enter_rel, los_occlusion_exit_rel,
                           los_side_reach_deadline, enter_time, exit_time,
                           static_cast<int>(reached_side),
                           side_reached_time, occlusion_start, occlusion_end);
                  ROS_INFO("[observation-topology-plane] drone=%d candidate=%s blocker=%d type=%d side=%d body_conflict=%d reason_mask=%u enter=%.6f exit=%.6f normal=(%.4f,%.4f,%.4f)",
                           pp_.drone_id, side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                           observation_plane.blocker_index, observation_plane.blocker_type,
                           side, static_cast<int>(body_hard_conflict),
                           nominal_result.conflict.reason_mask, enter_time, exit_time, observation_normal.x(),
                           observation_normal.y(), observation_normal.z());
                  ROS_INFO("[los-plane-lifecycle] stage=SIDE_CREATED candidate_id=%d "
                           "candidate_type=%s reason_mask=%u blocker_id=%d "
                           "collision_plane_count=%zu los_plane_count=1 "
                           "total_plane_count=%zu "
                           "los_plane{source=LOS_OBSERVATION_SIDE blocker_id=%d "
                           "side_sign=%d normal=(%.6f,%.6f,%.6f) "
                           "point=(%.6f,%.6f,%.6f) clearance=%.6f "
                           "active_start=%.6f active_end=%.6f}",
                           nominal_result.candidate_id,
                           side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                           nominal_result.conflict.reason_mask,
                           observation_plane.blocker_index,
                           local_sfc_planes.size() - 1, local_sfc_planes.size(),
                           observation_plane.blocker_index,
                           observation_plane.side_sign,
                           observation_plane.normal.x(), observation_plane.normal.y(),
                           observation_plane.normal.z(), observation_plane.point.x(),
                           observation_plane.point.y(), observation_plane.point.z(),
                           observation_plane.clearance,
                           observation_plane.active_start,
                           observation_plane.active_end);
                }
                else
                {
                  const bool window_limited = reached_side;
                  if (window_limited)
                    ++los_window_limited_count_;
                  else
                    ++los_recovery_not_reached_count_;
                  if (los_occlusion_window_valid)
                    ++los_known_occlusion_without_plane_count_;
                  ROS_INFO("[los-plane-audit] drone=%d candidate_type=%s "
                           "LOS_BLOCKER_ID=%d LOS_OCCLUSION_ENTER_REL=%.6f "
                           "LOS_OCCLUSION_EXIT_REL=%.6f LOS_SIDE_REACH_DEADLINE=%.6f "
                           "LOS_PLANE_ACTIVE_START=%.6f LOS_PLANE_ACTIVE_END=%.6f "
                           "LOS_REACHED_SIDE=%d LOS_PLANE_CREATED=0 status=%s "
                           "side_reached_time=%.6f occlusion_enter=%.6f occlusion_exit=%.6f",
                           pp_.drone_id, side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                           nominal_result.conflict.obstacle_identity,
                           los_occlusion_enter_rel, los_occlusion_exit_rel,
                           los_side_reach_deadline, enter_time, exit_time,
                           static_cast<int>(reached_side),
                           window_limited ? "WINDOW_LIMITED" : "RECOVERY_NOT_REACHED",
                           side_reached_time, occlusion_start, occlusion_end);
                  ROS_INFO("[observation-topology-plane] drone=%d candidate=%s blocker=%d side=%d status=%s",
                           pp_.drone_id, side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                           nominal_result.conflict.obstacle_identity, side,
                           window_limited ? "WINDOW_LIMITED" : "RECOVERY_NOT_REACHED");
                }
              }
            }

            // A SIDE generated for collision avoidance must improve the
            // triggering collision clearance.  A SIDE generated for shared
            // visibility risk has a different task purpose: requiring it to
            // improve an unrelated, already-safe moving-obstacle distance
            // rejects both topologies in an otherwise empty dynamic scene.
            // It still has to pass the existing absolute dynamic-safety
            // clearance here and again in authoritative final preflight.
            const bool los_only_conflict =
                !body_conflict_active &&
                (nominal_result.conflict.reason_mask & CONFLICT_LOS_OCCLUSION) != 0;
            side_dynamic_valid = trial_risk.valid && nominal_result.risk.valid &&
                (body_conflict_active
                     ? trial_risk.min_distance + 1.0e-6 >=
                           ploy_traj_opt_->getMovingObjClearance()
                     : (los_only_conflict
                            ? !trial_risk.hard_collision
                            : !trial_risk.hard_collision));
            if (!side_dynamic_valid)
            {
              side_dynamic_reason = trial_risk.valid
                                        ? (body_conflict_active
                                               ? "DYNAMIC_SAFETY_CLEARANCE"
                                               : (los_only_conflict
                                                      ? "LOS_PHYSICAL_HARD_COLLISION"
                                                      : "INSUFFICIENT_DYNAMIC_IMPROVEMENT"))
                                        : "NO_VALID_DYNAMIC_RISK";
              side_dynamic_invalid_after_backoff = true;
              if (side_static_free && trial_risk.valid && nominal_result.risk.valid)
              {
                side_repair_only = true;
                accepted_side_offset = trial_offset;
                ROS_INFO("[side-repair-only] type=%s offset=%.3f "
                         "seed_dynamic_clearance=%.6f required=%.6f "
                         "action=ENTER_OPTIMIZER_NO_EXECUTION_AUTHORITY",
                         side > 0 ? "SIDE_PLUS" : "SIDE_MINUS", trial_offset,
                         trial_risk.min_distance,
                         ploy_traj_opt_->getMovingObjClearance());
                break;
              }
              break;
            }

            accepted_side_offset = trial_offset;
            break;
          }
          if ((!side_init_valid || !side_static_free || !side_dynamic_valid) &&
              !side_repair_only)
          {
            const char *static_aware_result = side_dynamic_invalid_after_backoff
                                                  ? "DYNAMIC_INVALID_AFTER_BACKOFF"
                                                  : "STATIC_INFEASIBLE";
            ROS_INFO("[side-static-aware] type=%s requested_offset=%.3f accepted_offset=0 "
                     "attempts=%d reference_static_free=%d dynamic_valid=0 result=%s",
                     side > 0 ? "PLUS" : "MINUS", requested_side_offset,
                     side_generation_attempts, static_cast<int>(side_static_free),
                     static_aware_result);
            ROS_INFO("[side-static-aware-failure] type=%s smallest_tested_offset=%.3f "
                     "static_reason=%s dynamic_reason=%s",
                     side > 0 ? "PLUS" : "MINUS",
                     std::max(min_side_offset, requested_side_offset -
                                                   map_step * std::max(0, side_generation_attempts - 1)),
                     side_static_free ? "OBS_FREE" :
                         (side_static_reason.empty() ? "STATIC_INFEASIBLE" :
                                                       side_static_reason.c_str()),
                     side_dynamic_reason.empty() ? "NA" : side_dynamic_reason.c_str());

            // A single local repair is the final frontend stage.  It is
            // intentionally attempted only after all cheap offset backoff
            // trials failed, and only from a reference that still has the
            // requested dynamic clearance gain.
            if (enable_side_local_astar_repair_ && astar_base_valid && grid_map_ &&
                side_a_star_ && astar_base_traj.getPieceNum() > 0)
            {
              const int astar_samples = 25;
              const double astar_duration = astar_base_traj.getTotalDuration();
              const double astar_conflict_time = astar_base_risk.valid
                                                    ? astar_base_risk.conflict_time
                                                    : nominal_result.risk.conflict_time;
              // The dynamic risk window identifies where the moving obstacle
              // matters; it is not the geometric scope of a SIDE repair.  A
              // SIDE reference is generated over its complete local horizon,
              // so inspect that whole scope for inflated static occupancy.
              const double side_scope_start = 0.0;
              const double side_scope_end = astar_duration;
              const double astar_window_start = std::max(
                  0.0, astar_conflict_time - kGuidanceWindowSeconds);
              const double astar_window_end = std::min(
                  astar_duration, astar_conflict_time + kGuidanceWindowSeconds);
              int first_collision = -1;
              int last_collision = -1;
              bool astar_logged = false;
              for (int sample = 0; sample < astar_samples; ++sample)
              {
                const double alpha = static_cast<double>(sample) /
                                     static_cast<double>(astar_samples - 1);
                const double sample_time = side_scope_start +
                                           alpha * (side_scope_end - side_scope_start);
                const int occupied = grid_map_->getInflateOccupancy(
                    astar_base_traj.getPos(sample_time));
                if (occupied != 0)
                {
                  if (first_collision < 0)
                    first_collision = sample;
                  last_collision = sample;
                }
              }
              // Feedback119: ASTAR_REQUIRED = the SIDE guide is statically
              // infeasible after all offset backoffs and the repair stage
              // was reached (regardless of anchor availability).  Paired
              // with ASTAR_ATTEMPTS (actually run) and ASTAR_NO_PATH.
              if (first_collision >= 0)
                ++astar_required_count_;

              if (first_collision < 0)
              {
                ROS_INFO(
                    "[side-astar-static-window] type=%s side_scope_start=%.6f "
                    "side_scope_end=%.6f dynamic_window_start=%.6f "
                    "dynamic_window_end=%.6f static_collision_found=0 "
                    "static_collision_start=NA static_collision_end=NA "
                    "anchor_start=NA anchor_goal=NA",
                    side > 0 ? "PLUS" : "MINUS", side_scope_start, side_scope_end,
                    astar_window_start, astar_window_end);
              }

              // Give the repaired guide enough of the original timing to
              // traverse the detour. Two existing samples on each side keep
              // the repair local while avoiding a sub-0.3 s MINCO segment for
              // a multi-voxel path.
              const int anchor_before = first_collision > 1 ? first_collision - 2 :
                                        (first_collision > 0 ? first_collision - 1 : -1);
              const int anchor_after = last_collision >= 0 &&
                                                last_collision + 2 < astar_samples
                                            ? last_collision + 2
                                            : (last_collision + 1 < astar_samples
                                                   ? last_collision + 1
                                                   : -1);
              if (first_collision >= 0 && anchor_before >= 0 && anchor_after >= 0)
              {
                const double collision_start = side_scope_start +
                                               static_cast<double>(anchor_before) *
                                                   (side_scope_end - side_scope_start) /
                                                   static_cast<double>(astar_samples - 1);
                double collision_end = side_scope_start +
                                       static_cast<double>(anchor_after) *
                                           (side_scope_end - side_scope_start) /
                                           static_cast<double>(astar_samples - 1);
                const double static_collision_start = side_scope_start +
                    static_cast<double>(first_collision) *
                        (side_scope_end - side_scope_start) /
                        static_cast<double>(astar_samples - 1);
                const double static_collision_end = side_scope_start +
                    static_cast<double>(last_collision) *
                        (side_scope_end - side_scope_start) /
                        static_cast<double>(astar_samples - 1);
                const Eigen::Vector3d anchor_start = astar_base_traj.getPos(collision_start);
                Eigen::Vector3d anchor_goal = astar_base_traj.getPos(collision_end);
                const double map_resolution = grid_map_->getResolution() > 1.0e-6
                                                  ? grid_map_->getResolution()
                                                  : 0.1;
                const int rejoin_step = std::max(1, anchor_after - anchor_before);

                ROS_INFO(
                    "[side-astar-static-window] type=%s side_scope_start=%.6f "
                    "side_scope_end=%.6f dynamic_window_start=%.6f "
                    "dynamic_window_end=%.6f static_collision_found=1 "
                    "static_collision_start=%.6f static_collision_end=%.6f "
                    "anchor_start=(%.3f,%.3f,%.3f) anchor_goal=(%.3f,%.3f,%.3f)",
                    side > 0 ? "PLUS" : "MINUS", side_scope_start, side_scope_end,
                    astar_window_start, astar_window_end, static_collision_start,
                    static_collision_end, anchor_start.x(), anchor_start.y(),
                    anchor_start.z(), anchor_goal.x(), anchor_goal.y(), anchor_goal.z());

                Eigen::Vector3d side_line = local_target_pt - start_pt;
                side_line.z() = 0.0;
                Eigen::Vector3d side_normal = side_line.cross(Eigen::Vector3d::UnitZ());
                side_normal.z() = 0.0;
                const bool side_frame_valid = side_line.norm() > 1.0e-3 &&
                                              side_normal.norm() > 1.0e-3;
                if (side_frame_valid)
                {
                  side_line.normalize();
                  side_normal.normalize();
                }

                const auto segment_free = [&](const Eigen::Vector3d &from,
                                               const Eigen::Vector3d &to) {
                  return voxelSegmentFree(*grid_map_, from, to,
                                          map_resolution);
                };

                constexpr double semantic_eps = 0.05;
                constexpr double dominant_eps = 0.05;
                constexpr double dominant_ratio_threshold = 0.60;

                // This is an intent measurement around the primary predicted
                // blocker, not an A* or safety admission rule.  A route may
                // cross the path frame while avoiding other trees.
                const auto log_side_topology_intent = [&](const std::vector<Eigen::Vector3d> &path) {
                  const int primary_blocker =
                      nominal_result.conflict.los_obstacle_identity >= 0
                          ? nominal_result.conflict.los_obstacle_identity
                          : nominal_result.risk.obstacle_id;
                  if (path.size() < 2 || !side_frame_valid || !nominal_result.risk.valid ||
                      primary_blocker != nominal_result.risk.obstacle_id ||
                      !obj_predictor_ ||
                      !obj_predictor_->hasPrediction(nominal_result.risk.obstacle_id)) {
                    ROS_INFO("[SIDE_TOPOLOGY_INTENT] candidate=%s primary_blocker=%d "
                             "requested_side=%s local_sign_at_primary=NA "
                             "path_wide_score=NA intent_consistent=UNKNOWN used_as_hard_gate=0",
                             side > 0 ? "SIDE_PLUS" : "SIDE_MINUS", primary_blocker,
                             side > 0 ? "PLUS" : "MINUS");
                    return;
                  }
                  const size_t n = path.size();
                  std::vector<double> distances(n, 0.0), laterals(n, 0.0), weights(n, 0.0);
                  const double duration = std::max(1.0e-3, nominal_traj.getTotalDuration());
                  size_t closest = 0;
                  double min_distance = std::numeric_limits<double>::infinity();
                  for (size_t i = 0; i < n; ++i) {
                    const double alpha = static_cast<double>(i) / static_cast<double>(n - 1);
                    const double t = alpha * duration;
                    const Eigen::Vector3d ref = nominal_traj.getPos(t);
                    const Eigen::Vector3d obs = obj_predictor_->evaluateConstVel(
                        nominal_result.risk.obstacle_id, planning_prediction_epoch + t);
                    if (!ref.allFinite() || !obs.allFinite()) {
                      ROS_INFO("[SIDE_TOPOLOGY_INTENT] candidate=%s primary_blocker=%d "
                               "requested_side=%s local_sign_at_primary=NA "
                               "path_wide_score=NA intent_consistent=UNKNOWN used_as_hard_gate=0",
                               side > 0 ? "SIDE_PLUS" : "SIDE_MINUS", primary_blocker,
                               side > 0 ? "PLUS" : "MINUS");
                      return;
                    }
                    distances[i] = (path[i] - obs).norm();
                    const Eigen::Vector3d tangent = nominal_traj.getVel(t);
                    Eigen::Vector3d normal = tangent.cross(Eigen::Vector3d::UnitZ());
                    normal.z() = 0.0;
                    if (normal.norm() <= 1.0e-6) {
                      ROS_INFO("[SIDE_TOPOLOGY_INTENT] candidate=%s primary_blocker=%d "
                               "requested_side=%s local_sign_at_primary=NA "
                               "path_wide_score=NA intent_consistent=UNKNOWN used_as_hard_gate=0",
                               side > 0 ? "SIDE_PLUS" : "SIDE_MINUS", primary_blocker,
                               side > 0 ? "PLUS" : "MINUS");
                      return;
                    }
                    normal.normalize();
                    laterals[i] = (path[i] - ref).dot(normal);
                    if (distances[i] < min_distance) {
                      min_distance = distances[i];
                      closest = i;
                    }
                  }
                  const double closest_lateral = laterals[closest];
                  const bool closest_ok = side > 0 ? closest_lateral >= -semantic_eps
                                                   : closest_lateral <= semantic_eps;
                  const double conflict_limit = min_distance + 1.0;
                  double weighted_lateral = 0.0, weight_sum = 0.0, correct_weight = 0.0;
                  size_t conflict_points = 0;
                  for (size_t i = 0; i < n; ++i) {
                    if (distances[i] > conflict_limit) continue;
                    ++conflict_points;
                    const double w = std::min(10.0, 1.0 / (distances[i] + 0.1));
                    weights[i] = w;
                    weighted_lateral += w * laterals[i];
                    weight_sum += w;
                    const bool correct = side > 0 ? laterals[i] >= -semantic_eps
                                                  : laterals[i] <= semantic_eps;
                    if (correct) correct_weight += w;
                  }
                  if (conflict_points == 0 || weight_sum <= 1.0e-9) {
                    ROS_INFO("[SIDE_TOPOLOGY_INTENT] candidate=%s primary_blocker=%d "
                             "requested_side=%s local_sign_at_primary=NA "
                             "path_wide_score=NA intent_consistent=UNKNOWN used_as_hard_gate=0",
                             side > 0 ? "SIDE_PLUS" : "SIDE_MINUS", primary_blocker,
                             side > 0 ? "PLUS" : "MINUS");
                    return;
                  }
                  weighted_lateral /= weight_sum;
                  const double correct_ratio = correct_weight / weight_sum;
                  const bool dominant_sign_ok = side > 0 ? weighted_lateral >= -dominant_eps
                                                         : weighted_lateral <= dominant_eps;
                  const bool dominant_ok = dominant_sign_ok &&
                                            correct_ratio >= dominant_ratio_threshold;
                  const bool valid = closest_ok && dominant_ok;
                  ROS_INFO("[SIDE_TOPOLOGY_INTENT] candidate=%s primary_blocker=%d "
                           "requested_side=%s local_sign_at_primary=%.6f "
                           "path_wide_score=%.6f intent_consistent=%d "
                           "used_as_hard_gate=0 correct_weight_ratio=%.6f",
                           side > 0 ? "SIDE_PLUS" : "SIDE_MINUS", primary_blocker,
                           side > 0 ? "PLUS" : "MINUS", closest_lateral,
                           weighted_lateral, static_cast<int>(valid), correct_ratio);
                };

                // A* admission is based on static geometry only.  SIDE
                // topology intent is recorded after path selection.
                const auto raw_path_usable = [&](const std::vector<Eigen::Vector3d> &path,
                                                 const Eigen::Vector3d &goal) {
                  if (path.size() < 2 ||
                      !segment_free(anchor_start, path.front()) ||
                      !segment_free(path.back(), goal))
                    return false;
                  for (const Eigen::Vector3d &point : path)
                  {
                    if (grid_map_->getInflateOccupancy(point) != 0)
                      return false;
                  }
                  for (size_t index = 1; index < path.size(); ++index)
                    if (!segment_free(path[index - 1], path[index]))
                      return false;
                  return true;
                };

                std::vector<Eigen::Vector3d> raw_path;
                ASTAR_RET astar_ret = ASTAR_RET::INIT_ERR;
                double astar_elapsed_ms = 0.0;
                int selected_rejoin_attempt = -1;
                int selected_rejoin_index = -1;
                bool astar_path_selected = false;
                std::vector<Eigen::Vector3d> first_success_path;
                double first_success_collision_end = collision_end;
                Eigen::Vector3d first_success_anchor_goal = anchor_goal;
                int first_success_rejoin_attempt = -1;
                int first_success_rejoin_index = -1;
                double first_success_elapsed_ms = 0.0;
                const int max_rejoin_attempts = 3;
                // A* repairs static topology only. Visibility quality is
                // evaluated once after hard preflight by the local comparator.
                side_a_star_->clearVisibilityCostContext();
                for (int rejoin_attempt = 0; rejoin_attempt < max_rejoin_attempts;
                     ++rejoin_attempt)
                {
                  if(!side_work_allowed())
                  {
                    ROS_WARN("[side-astar] drone=%d candidate=%s result=EXECUTION_COVERAGE_DEADLINE rejoin_attempt=%d",
                        pp_.drone_id,side>0?"SIDE_PLUS":"SIDE_MINUS",rejoin_attempt);
                    break;
                  }
                  const int rejoin_index = std::min(
                      astar_samples - 1, anchor_after + rejoin_attempt * rejoin_step);
                  if (rejoin_attempt > 0 && rejoin_index <= selected_rejoin_index)
                    continue;
                  const double trial_collision_end = side_scope_start +
                      static_cast<double>(rejoin_index) *
                          (side_scope_end - side_scope_start) /
                          static_cast<double>(astar_samples - 1);
                  const Eigen::Vector3d trial_anchor_goal =
                      astar_base_traj.getPos(trial_collision_end);
                  const bool trial_anchors_free =
                      grid_map_->getInflateOccupancy(anchor_start) == 0 &&
                      grid_map_->getInflateOccupancy(trial_anchor_goal) == 0;
                  ROS_INFO(
                      "[astar-rejoin] candidate_id=NA drone_id=%d side=%s "
                      "rejoin_attempt=%d rejoin_index=%d rejoin_pos=(%.3f,%.3f,%.3f)",
                      pp_.drone_id, side > 0 ? "PLUS" : "MINUS", rejoin_attempt,
                      rejoin_index, trial_anchor_goal.x(), trial_anchor_goal.y(),
                      trial_anchor_goal.z());
                  const auto astar_start_time = std::chrono::steady_clock::now();
                  ASTAR_RET trial_ret = ASTAR_RET::INIT_ERR;
                  std::vector<Eigen::Vector3d> trial_path;
                  if (trial_anchors_free)
                  {
                    ++astar_attempt_count_;
                    trial_ret = side_a_star_->AstarSearch(map_resolution,
                                                          anchor_start,
                                                          trial_anchor_goal);
                    if (trial_ret == ASTAR_RET::SUCCESS)
                      trial_path = side_a_star_->getPath();
                  }
                  const double trial_elapsed_ms =
                      std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - astar_start_time)
                          .count();
                  ROS_INFO(
                      "[astar-rejoin] candidate_id=NA drone_id=%d side=%s "
                      "rejoin_attempt=%d rejoin_index=%d astar_result=%s raw_points=%zu "
                      "elapsed_ms=%.3f",
                      pp_.drone_id, side > 0 ? "PLUS" : "MINUS", rejoin_attempt,
                      rejoin_index,
                      trial_ret == ASTAR_RET::SUCCESS ? "SUCCESS" : "FAILED",
                      trial_path.size(), trial_elapsed_ms);
                  if (trial_ret == ASTAR_RET::SUCCESS && trial_path.size() >= 2)
                  {
                    if (first_success_path.empty())
                    {
                      first_success_path = trial_path;
                      first_success_collision_end = trial_collision_end;
                      first_success_anchor_goal = trial_anchor_goal;
                      first_success_rejoin_attempt = rejoin_attempt;
                      first_success_rejoin_index = rejoin_index;
                      first_success_elapsed_ms = trial_elapsed_ms;
                    }
                    if (raw_path_usable(trial_path, trial_anchor_goal))
                    {
                      astar_ret = trial_ret;
                      raw_path = std::move(trial_path);
                      collision_end = trial_collision_end;
                      anchor_goal = trial_anchor_goal;
                      selected_rejoin_attempt = rejoin_attempt;
                      selected_rejoin_index = rejoin_index;
                      astar_elapsed_ms = trial_elapsed_ms;
                      astar_path_selected = true;
                      if (rejoin_attempt > 0)
                      {
                        ROS_INFO(
                            "[astar-rejoin-fallback] candidate_id=NA drone_id=%d side=%s "
                            "from_attempt=0 to_attempt=%d reason=DOWNSTREAM_REJOIN",
                            pp_.drone_id, side > 0 ? "PLUS" : "MINUS", rejoin_attempt);
                      }
                      break;
                    }
                  }
                  astar_ret = trial_ret;
                  astar_elapsed_ms += trial_elapsed_ms;
                }
                side_a_star_->clearVisibilityCostContext();
                // Feedback119: ASTAR_NO_PATH = the search itself returned no
                // usable path (not anchor-blocked, not downstream rejoin).
                if (astar_ret != ASTAR_RET::SUCCESS && first_success_path.empty())
                  ++astar_no_path_count_;

                // Preserve the first successful path for diagnostics when
                // every downstream alternative is also unusable.  It will be
                // checked against static geometry below.
                if (!astar_path_selected && !first_success_path.empty())
                {
                  astar_ret = ASTAR_RET::SUCCESS;
                  raw_path = std::move(first_success_path);
                  collision_end = first_success_collision_end;
                  anchor_goal = first_success_anchor_goal;
                  selected_rejoin_attempt = first_success_rejoin_attempt;
                  selected_rejoin_index = first_success_rejoin_index;
                  astar_elapsed_ms = first_success_elapsed_ms;
                }
                // Preserve the actual continuous endpoints as route vertices;
                // the grid centers alone are not the repaired trajectory's
                // head and tail. Their connecting edges are checked below.
                if (!raw_path.empty())
                {
                  if ((raw_path.front() - anchor_start).norm() > 1.0e-6)
                    raw_path.insert(raw_path.begin(), anchor_start);
                  if ((raw_path.back() - anchor_goal).norm() > 1.0e-6)
                    raw_path.push_back(anchor_goal);
                }

                std::vector<Eigen::Vector3d> simplified_path;
                int reinsertion_points = 0;
                bool path_static_free = !raw_path.empty();
                // Kept in the legacy audit field only; topology intent is
                // measured separately and cannot invalidate a static-safe path.
                bool path_side_valid = true;
                size_t raw_static_bad_count = 0;
                size_t raw_side_bad_count = 0;
                int first_raw_side_bad_index = -1;
                double first_raw_side_lateral = 0.0;
                for (size_t index = 0; index < raw_path.size(); ++index)
                {
                  const bool point_static_free =
                      grid_map_->getInflateOccupancy(raw_path[index]) == 0;
                  const bool point_side_valid = true;
                  if (!point_static_free)
                    ++raw_static_bad_count;
                  if (!point_side_valid)
                  {
                    ++raw_side_bad_count;
                    if (first_raw_side_bad_index < 0)
                    {
                      first_raw_side_bad_index = static_cast<int>(index);
                      if (side_frame_valid)
                      {
                        const Eigen::Vector3d relative = raw_path[index] - start_pt;
                        const double longitudinal = relative.dot(side_line);
                        const Eigen::Vector3d line_point =
                            start_pt + longitudinal * side_line;
                        first_raw_side_lateral =
                            static_cast<double>(side) *
                            (raw_path[index] - line_point).dot(side_normal);
                      }
                    }
                  }
                  path_static_free = path_static_free && point_static_free;
                  path_side_valid = path_side_valid && point_side_valid;
                }
                if (path_static_free && raw_path.size() >= 2)
                {
                  path_static_free = segment_free(anchor_start, raw_path.front()) &&
                      segment_free(raw_path.back(), anchor_goal);
                  for (size_t index = 1;
                       path_static_free && index < raw_path.size(); ++index)
                    path_static_free = segment_free(raw_path[index - 1],
                                                    raw_path[index]);
                }
                if (path_static_free)
                  log_side_topology_intent(raw_path);
                ROS_INFO("[side-astar-raw-audit] type=%s raw_points=%zu "
                         "static_bad=%zu side_bad=%zu first_side_bad_index=%d "
                         "first_side_lateral=%.6f path_static_free=%d "
                         "path_side_valid=%d",
                         side > 0 ? "PLUS" : "MINUS", raw_path.size(),
                         raw_static_bad_count, raw_side_bad_count,
                         first_raw_side_bad_index, first_raw_side_lateral,
                         static_cast<int>(path_static_free),
                         static_cast<int>(path_side_valid));

                // Greedy line-of-sight pruning keeps only geometric guide
                // points while rechecking every retained segment against the
                // same inflated map used by A*.
                if (path_static_free && raw_path.size() >= 2)
                {
                  size_t current = 0;
                  simplified_path.push_back(raw_path.front());
                  while (current + 1 < raw_path.size())
                  {
                    size_t furthest = current + 1;
                    // Keep local A* geometry visible to MINCO.  A long LOS
                    // shortcut can erase the turn that avoids an inflated
                    // obstacle even though the shortcut itself is collision
                    // free at map resolution.  Three raw edges is the
                    // existing local-grid scale used by this repair and does
                    // not introduce a planner tuning parameter.
                    const size_t max_shortcut_span = 3;
                    for (size_t candidate = current + 2;
                         candidate < raw_path.size(); ++candidate)
                    {
                      if (candidate - current > max_shortcut_span)
                        break;
                      if (segment_free(raw_path[current], raw_path[candidate]))
                        furthest = candidate;
                      else
                        break;
                    }
                    simplified_path.push_back(raw_path[furthest]);
                    current = furthest;
                  }

                  // Preserve pronounced direction changes even when a short
                  // LOS segment happens to be free.  This is intentionally a
                  // fixed geometric test, not a new curvature optimizer.
                  if (raw_path.size() >= 3)
                  {
                    constexpr double kTurningAngle = 20.0 * M_PI / 180.0;
                    for (size_t index = 1; index + 1 < raw_path.size(); ++index)
                    {
                      const Eigen::Vector3d incoming =
                          raw_path[index] - raw_path[index - 1];
                      const Eigen::Vector3d outgoing =
                          raw_path[index + 1] - raw_path[index];
                      if (incoming.norm() <= 1.0e-6 || outgoing.norm() <= 1.0e-6)
                        continue;
                      const double cosine = std::max(
                          -1.0, std::min(1.0, incoming.dot(outgoing) /
                                                     (incoming.norm() * outgoing.norm())));
                      if (std::acos(cosine) < kTurningAngle)
                        continue;
                      if (std::find_if(simplified_path.begin(), simplified_path.end(),
                                       [&](const Eigen::Vector3d &point) {
                                         return (point - raw_path[index]).norm() <
                                                0.5 * map_resolution;
                                       }) == simplified_path.end())
                      {
                        simplified_path.push_back(raw_path[index]);
                      }
                    }
                    std::sort(simplified_path.begin(), simplified_path.end(),
                              [&](const Eigen::Vector3d &lhs, const Eigen::Vector3d &rhs) {
                                auto raw_index = [&](const Eigen::Vector3d &point) {
                                  size_t best = 0;
                                  double best_distance = std::numeric_limits<double>::infinity();
                                  for (size_t raw_index = 0; raw_index < raw_path.size(); ++raw_index)
                                  {
                                    const double distance =
                                        (raw_path[raw_index] - point).squaredNorm();
                                    if (distance < best_distance)
                                    {
                                      best_distance = distance;
                                      best = raw_index;
                                    }
                                  }
                                  return best;
                                };
                                return raw_index(lhs) < raw_index(rhs);
                              });
                  }

                  // Keep at most two raw A* points nearest the static
                  // collision band.  These are the likely obstacle-facing
                  // turning points that a LOS shortcut can remove.  They are
                  // deliberately reinserted before MINCO reconstruction, so
                  // no second search or unbounded point growth is needed.
                  if (first_collision >= 0 && last_collision >= first_collision &&
                      raw_path.size() > simplified_path.size())
                  {
                    const double collision_mid_time =
                        0.5 * (static_collision_start + static_collision_end);
                    const Eigen::Vector3d collision_midpoint =
                        astar_base_traj.getPos(collision_mid_time);
                    std::vector<size_t> nearest_indices(raw_path.size());
                    std::iota(nearest_indices.begin(), nearest_indices.end(), 0);
                    std::sort(nearest_indices.begin(), nearest_indices.end(),
                              [&](const size_t lhs, const size_t rhs) {
                                return (raw_path[lhs] - collision_midpoint).squaredNorm() <
                                       (raw_path[rhs] - collision_midpoint).squaredNorm();
                              });
                    for (const size_t raw_index : nearest_indices)
                    {
                      if (reinsertion_points >= 2)
                        break;
                      const Eigen::Vector3d &point = raw_path[raw_index];
                      if (std::find_if(simplified_path.begin(), simplified_path.end(),
                                       [&](const Eigen::Vector3d &existing) {
                                         return (existing - point).norm() <
                                                0.5 * map_resolution;
                                       }) != simplified_path.end())
                        continue;
                      simplified_path.push_back(point);
                      ++reinsertion_points;
                    }
                    std::sort(simplified_path.begin(), simplified_path.end(),
                              [&](const Eigen::Vector3d &lhs, const Eigen::Vector3d &rhs) {
                                auto raw_index = [&](const Eigen::Vector3d &point) {
                                  size_t best = 0;
                                  double best_distance = std::numeric_limits<double>::infinity();
                                  for (size_t index = 0; index < raw_path.size(); ++index)
                                  {
                                    const double distance =
                                        (raw_path[index] - point).squaredNorm();
                                    if (distance < best_distance)
                                    {
                                      best_distance = distance;
                                      best = index;
                                    }
                                  }
                                  return best;
                                };
                                return raw_index(lhs) < raw_index(rhs);
                              });
                  }
                }

                bool repaired_static_free = false;
                bool repaired_dynamic_valid = false;
                bool repaired_side_valid = simplified_path.size() >= 2;
                std::string repaired_static_reason = "NA";
                poly_traj::Trajectory repaired_traj;
                std::vector<LocalSfcPlane> repaired_local_sfc_planes;
                bool local_sfc_build_valid = false;
                const char *local_sfc_status = "CORRIDOR_BUILD_FAILED";
                int corridor_segment_count = 0;
                int corridor_subdivision_count = 0;
                // Feedback119 (一次性重构,取代"单侧防切角 half-space"):
                // A*-repair SIDE 的几何合同是
                //   A* raw path      = 几何路线(逐点占据已审计),
                //   bounded corridor = guide 折线必要 segment 上的有界凸区域
                //                      (左右墙由 inflated map 双向边界扫描
                //                      得到,前后帽与相邻 corridor 重叠;整箱
                //                      采样审计 corridor ⊂ static free;
                //                      单段不能成域时只允许局部细分),
                //   MINCO seed       = 数值初值,不要求 static-safe,切角由
                //                      corridor hard SCP 修正。
                // 旧的 restore-raw / bulge seed 修补链(×≤3 次重建)被整体
                // 删除:一个 SIDE 至多 1 次 A* + 1 次 corridor build + 1 条
                // MINCO-SCP 优化链。
                const double corridor_scan_margin = std::max(
                    map_resolution, ploy_traj_opt_->getObsClearance());
                const double corridor_max_scan = 2.0 * corridor_scan_margin;
                // Seed 时标表:与被删除的重建循环完全相同的曲率限速常量,
                // corridor 时间窗与唯一一次 seed 生成共享同一时间轴。
                constexpr double kRepairEtaV = 0.8;
                constexpr double kRepairEtaA = 0.8;
                constexpr double kRepairEpsilon = 1.0e-6;
                constexpr double kRepairSpeedFloor = 0.10;
                constexpr double kRepairMinPieceDuration = 1.0e-3;
                int corridor_piece_num =
                    static_cast<int>(simplified_path.size()) - 1;
                std::vector<double> corridor_seg_lengths(
                    static_cast<size_t>(std::max(0, corridor_piece_num)), 0.0);
                std::vector<double> corridor_vertex_speeds(
                    simplified_path.size(),
                    std::max(kRepairSpeedFloor, kRepairEtaV * pp_.max_vel_));
                double corridor_path_length = 0.0;
                double corridor_repair_duration = 0.0;
                double corridor_max_curvature = 0.0;
                for (int seg = 0; seg < corridor_piece_num; ++seg)
                {
                  corridor_seg_lengths[seg] =
                      (simplified_path[seg + 1] - simplified_path[seg]).norm();
                  corridor_path_length += corridor_seg_lengths[seg];
                }
                for (int vertex = 1; vertex < corridor_piece_num; ++vertex)
                {
                  const Eigen::Vector3d incoming =
                      simplified_path[vertex] - simplified_path[vertex - 1];
                  const Eigen::Vector3d outgoing =
                      simplified_path[vertex + 1] - simplified_path[vertex];
                  const double incoming_length = incoming.norm();
                  const double outgoing_length = outgoing.norm();
                  if (incoming_length <= kRepairEpsilon ||
                      outgoing_length <= kRepairEpsilon)
                    continue;
                  const double cosine = std::max(
                      -1.0, std::min(1.0, incoming.dot(outgoing) /
                                              (incoming_length * outgoing_length)));
                  const double theta = std::acos(cosine);
                  const double mean_length =
                      std::max(0.5 * (incoming_length + outgoing_length),
                               kRepairEpsilon);
                  const double curvature =
                      2.0 * std::sin(0.5 * theta) / mean_length;
                  corridor_max_curvature =
                      std::max(corridor_max_curvature, curvature);
                  if (pp_.max_acc_ > 0.0)
                  {
                    const double curvature_speed = std::sqrt(
                        kRepairEtaA * pp_.max_acc_ / (curvature + kRepairEpsilon));
                    corridor_vertex_speeds[vertex] = std::max(
                        kRepairSpeedFloor,
                        std::min(corridor_vertex_speeds[vertex],
                                 curvature_speed));
                  }
                }
                std::vector<double> corridor_seg_durations(
                    static_cast<size_t>(std::max(0, corridor_piece_num)), 0.0);
                std::vector<double> corridor_seg_enter(
                    static_cast<size_t>(std::max(0, corridor_piece_num)) + 1, 0.0);
                for (int seg = 0; seg < corridor_piece_num; ++seg)
                {
                  const double segment_speed = std::max(
                      kRepairSpeedFloor,
                      std::min(corridor_vertex_speeds[seg],
                               corridor_vertex_speeds[seg + 1]));
                  double segment_duration =
                      corridor_seg_lengths[seg] / segment_speed;
                  if (!std::isfinite(segment_duration) || segment_duration <= 0.0)
                    segment_duration = kRepairMinPieceDuration;
                  corridor_seg_durations[seg] =
                      std::max(kRepairMinPieceDuration, segment_duration);
                  corridor_seg_enter[seg + 1] =
                      corridor_seg_enter[seg] + corridor_seg_durations[seg];
                }
                corridor_repair_duration = corridor_seg_enter.back();
                if (repaired_side_valid && path_static_free && grid_map_ &&
                    simplified_path.size() >= 2)
                {
                  local_sfc_build_valid = true;
                  for (const Eigen::Vector3d &point : simplified_path)
                    local_sfc_build_valid = local_sfc_build_valid && point.allFinite();

                  // Outward boundary scan of one side; the minimum free
                  // distance over all stations of the segment bounds the box.
                  const auto corridor_scan_width =
                      [&](const Eigen::Vector3d &origin,
                          const Eigen::Vector3d &direction,
                          double *lambda_out) {
                        const LocalSfcBoundaryScanResult scan =
                            scanFirstLocalSfcBoundary(
                                corridor_max_scan, map_resolution,
                                [&](const double lambda) {
                                  return grid_map_->getInflateOccupancy(
                                             origin + lambda * direction) != 0;
                                });
                        if (scan.guide_occupied)
                          return false;
                        *lambda_out = scan.transition_found
                                          ? scan.lambda_boundary
                                          : corridor_max_scan;
                        return *lambda_out >= 0.5 * map_resolution;
                      };

                  // One bounded-convex attempt over vertex range [i0, i1]
                  // with seed-time window [win_start, win_end].  Emits the
                  // found walls plus both caps (caps overlap the neighbouring
                  // corridor by map_resolution along track).
                  // try_build_segment: one bounded-convex attempt over the
                  // point pair (pa, pb) with seed-time window
                  // [win_start, win_end].  Slab coordinate
                  // u = lateral.(p - mid) must stay in
                  // [-lambda_left, +lambda_right]:
                  //   wall on the -lateral side (left):  +lateral.(p-q) >= 0,
                  //                                      q = mid - lateral*lamL
                  //   wall on the +lateral side (right): -lateral.(p-q) >= 0,
                  //                                      q = mid + lateral*lamR
                  // plus both caps overlapping the neighbouring corridor by
                  // map_resolution along track.  Whole-box audit (inset)
                  // proves corridor ⊂ static free space.
                  const auto try_build_segment =
                      [&](const Eigen::Vector3d &pa, const Eigen::Vector3d &pb,
                          const int i0, const int i1, const double win_start,
                          const double win_end) {
                        if (win_end <= win_start + 1.0e-5)
                          return false;
                        Eigen::Vector3d tangent = pb - pa;
                        tangent.z() = 0.0;
                        const double seg_len = tangent.norm();
                        // This Local-SFC is a fixed-altitude 2.5D extrusion.
                        // Sloped segments need a different certified prism.
                        if (!tangent.allFinite() || seg_len < 1.0e-6 ||
                            std::abs(pa.z() - pb.z()) > 1.0e-6)
                          return false;
                        tangent /= seg_len;
                        Eigen::Vector3d lateral = tangent.cross(Eigen::Vector3d::UnitZ());
                        lateral.z() = 0.0;
                        if (!lateral.allFinite() || lateral.norm() < 1.0e-6)
                          return false;
                        lateral.normalize();
                        double lambda_left = corridor_max_scan;
                        double lambda_right = corridor_max_scan;
                        for (int station = 0; station < 3; ++station)
                        {
                          const Eigen::Vector3d origin =
                              pa + (seg_len * station / 2.0) * tangent;
                          double scan_left = 0.0;
                          double scan_right = 0.0;
                          // lateral points to the travelling right side.
                          if (!corridor_scan_width(origin, -lateral, &scan_left))
                            return false;
                          if (!corridor_scan_width(origin, lateral, &scan_right))
                            return false;
                          lambda_left = std::min(lambda_left, scan_left);
                          lambda_right = std::min(lambda_right, scan_right);
                        }
                        // Emit exactly the inset volume that is certified.
                        // Check every grid voxel that may intersect the
                        // oriented 3D prism, including corner-touching voxels.
                        const double audit_inset = 0.25 * map_resolution;
                        const double cap_overlap = map_resolution - audit_inset;
                        const double left_extent = lambda_left - audit_inset;
                        const double right_extent = lambda_right - audit_inset;
                        const double z_extent = 0.5 * map_resolution;
                        if (left_extent <= 0.0 || right_extent <= 0.0 ||
                            cap_overlap <= 0.0 ||
                            (grid_map_->getVirtualCeilHeight() > -0.5 &&
                             pa.z() + z_extent >=
                                 grid_map_->getVirtualCeilHeight()))
                          return false;
                        Eigen::Vector3d min_corner =
                            Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
                        Eigen::Vector3d max_corner =
                            Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
                        for (const double s : {-cap_overlap, seg_len + cap_overlap})
                          for (const double w : {-left_extent, right_extent})
                            for (const double z : {-z_extent, z_extent})
                            {
                              const Eigen::Vector3d corner =
                                  pa + s * tangent + w * lateral +
                                  z * Eigen::Vector3d::UnitZ();
                              min_corner = min_corner.cwiseMin(corner);
                              max_corner = max_corner.cwiseMax(corner);
                            }
                        Eigen::Vector3i min_index, max_index;
                        if (!grid_map_->isInMap(min_corner) ||
                            !grid_map_->isInMap(max_corner))
                          return false;
                        grid_map_->posToIndex(min_corner, min_index);
                        grid_map_->posToIndex(max_corner, max_index);
                        const double voxel_t = 0.5 * map_resolution *
                            (std::abs(tangent.x()) + std::abs(tangent.y()));
                        const double voxel_w = 0.5 * map_resolution *
                            (std::abs(lateral.x()) + std::abs(lateral.y()));
                        for (int ix = min_index.x(); ix <= max_index.x(); ++ix)
                          for (int iy = min_index.y(); iy <= max_index.y(); ++iy)
                            for (int iz = min_index.z(); iz <= max_index.z(); ++iz)
                            {
                              Eigen::Vector3d center;
                              grid_map_->indexToPos(Eigen::Vector3i(ix, iy, iz), center);
                              const Eigen::Vector3d delta = center - pa;
                              const double s = delta.dot(tangent);
                              const double w = delta.dot(lateral);
                              if (s < -cap_overlap - voxel_t ||
                                  s > seg_len + cap_overlap + voxel_t ||
                                  w < -left_extent - voxel_w ||
                                  w > right_extent + voxel_w ||
                                  std::abs(delta.z()) > z_extent +
                                      0.5 * map_resolution)
                                continue;
                              if (grid_map_->getInflateOccupancy(center) != 0)
                                return false;
                            }
                        const Eigen::Vector3d seg_mid = pa + (0.5 * seg_len) * tangent;
                        LocalSfcPlane left_wall;
                        left_wall.source = LocalSfcPlane::STATIC_COLLISION_CORRIDOR;
                        left_wall.side_sign = side;
                        left_wall.normal = lateral;
                        left_wall.point = seg_mid - lateral * left_extent;
                        left_wall.guide = pa;
                        left_wall.source_segment = i0;
                        left_wall.source_guide_index = i1;
                        left_wall.clearance = 0.0;
                        left_wall.active_start = win_start;
                        left_wall.active_end = win_end;
                        LocalSfcPlane right_wall = left_wall;
                        right_wall.normal = -lateral;
                        right_wall.point = seg_mid + lateral * right_extent;
                        // Caps bound the along-track coordinate s (s = t.(p-pa),
                        // L = |pb-pa|) to [-cap_overlap, L + cap_overlap]:
                        //   front: +tangent.(p - (pa - t*cap)) = s + cap >= 0
                        //   back:  -tangent.(p - (pb + t*cap)) = L - s + cap >= 0
                        LocalSfcPlane front_cap = left_wall;
                        front_cap.normal = tangent;
                        front_cap.point = pa - tangent * cap_overlap;
                        LocalSfcPlane back_cap = left_wall;
                        back_cap.normal = -tangent;
                        back_cap.point = pb + tangent * cap_overlap;
                        LocalSfcPlane lower_cap = left_wall;
                        lower_cap.normal = Eigen::Vector3d::UnitZ();
                        lower_cap.point = pa - z_extent * Eigen::Vector3d::UnitZ();
                        LocalSfcPlane upper_cap = left_wall;
                        upper_cap.normal = -Eigen::Vector3d::UnitZ();
                        upper_cap.point = pa + z_extent * Eigen::Vector3d::UnitZ();
                        repaired_local_sfc_planes.push_back(left_wall);
                        repaired_local_sfc_planes.push_back(right_wall);
                        repaired_local_sfc_planes.push_back(front_cap);
                        repaired_local_sfc_planes.push_back(back_cap);
                        repaired_local_sfc_planes.push_back(lower_cap);
                        repaired_local_sfc_planes.push_back(upper_cap);
                        ++corridor_segment_count;
                        return true;
                      };

                  // Pass 1: which segments interact with the obstacle
                  // geometry (any wall found) and their one-segment
                  // neighbours — only these are "necessary segments" and get
                  // a bounded convex region; open-field segments keep the
                  // former free-probe semantics (no planes).
                  std::vector<bool> segment_has_wall(
                      static_cast<size_t>(std::max(0, corridor_piece_num)), false);
                  std::vector<bool> segment_necessary(
                      static_cast<size_t>(std::max(0, corridor_piece_num)), false);
                  for (int seg = 0; seg < corridor_piece_num; ++seg)
                  {
                    const Eigen::Vector3d &pa = simplified_path[seg];
                    const Eigen::Vector3d &pb = simplified_path[seg + 1];
                    Eigen::Vector3d tangent = pb - pa;
                    tangent.z() = 0.0;
                    const double seg_len = tangent.norm();
                    if (!tangent.allFinite() || seg_len < 1.0e-6)
                      continue;
                    tangent /= seg_len;
                    Eigen::Vector3d lateral = tangent.cross(Eigen::Vector3d::UnitZ());
                    lateral.z() = 0.0;
                    if (!lateral.allFinite() || lateral.norm() < 1.0e-6)
                      continue;
                    lateral.normalize();
                    for (int station = 0; station < 3 && !segment_has_wall[seg];
                         ++station)
                    {
                      const Eigen::Vector3d origin =
                          pa + (seg_len * station / 2.0) * tangent;
                      double lambda = 0.0;
                      if (!corridor_scan_width(origin, lateral, &lambda) ||
                          lambda < corridor_max_scan)
                        segment_has_wall[seg] = true;
                      else
                      {
                        lambda = 0.0;
                        if (!corridor_scan_width(origin, -lateral, &lambda) ||
                            lambda < corridor_max_scan)
                          segment_has_wall[seg] = true;
                      }
                    }
                  }
                  for (int seg = 0; seg < corridor_piece_num; ++seg)
                  {
                    if (segment_has_wall[seg])
                    {
                      segment_necessary[seg] = true;
                      if (seg > 0)
                        segment_necessary[seg - 1] = true;
                      if (seg + 1 < corridor_piece_num)
                        segment_necessary[seg + 1] = true;
                    }
                  }

                  // Pass 2: build with bounded local subdivision only.
                  // Subdivision inserts a geometric midpoint (no new search,
                  // no new planner); the corridor-local point list grows by
                  // at most depth<=2 per original segment.
                  std::vector<Eigen::Vector3d> corridor_points = simplified_path;
                  std::vector<double> corridor_point_enter(
                      corridor_points.size(), 0.0);
                  for (int seg = 0;
                       seg < static_cast<int>(corridor_points.size()) - 1; ++seg)
                    corridor_point_enter[seg + 1] = corridor_seg_enter[seg + 1];
                  struct CorridorTask
                  {
                    int i0;
                    int i1;
                    int depth;
                    double win_start;
                    double win_end;
                  };
                  for (int seg = 0;
                       local_sfc_build_valid && seg < corridor_piece_num; ++seg)
                  {
                    if (!segment_necessary[seg])
                      continue;
                    if (!side_work_allowed())
                    {
                      local_sfc_build_valid = false;
                      local_sfc_status = "EXECUTION_COVERAGE_DEADLINE";
                      break;
                    }
                    // Time overlap between adjacent corridor windows.  The
                    // spatial caps already give junction continuity; the
                    // time overlap must only cover the smoothing lag of the
                    // polynomial across the cap zone -- a full half-segment
                    // would keep walls active while the trajectory has
                    // legitimately moved on (offline check C, F119).
                    const double cap_traversal_t =
                        corridor_seg_durations[seg] * map_resolution /
                        std::max(corridor_seg_lengths[seg], kRepairEpsilon);
                    const double overlap_t =
                        std::min(0.5 * corridor_seg_durations[seg],
                                 4.0 * cap_traversal_t);
                    double win_start =
                        collision_start + corridor_seg_enter[seg] - overlap_t;
                    double win_end =
                        collision_start + corridor_seg_enter[seg + 1] + overlap_t;
                    win_start = std::max(0.0, win_start);
                    if (seg == corridor_piece_num - 1)
                      win_end = collision_start + corridor_repair_duration +
                                overlap_t;
                    std::vector<CorridorTask> tasks;
                    tasks.push_back({seg, seg + 1, 0, win_start, win_end});
                    bool segment_built = false;
                    while (!tasks.empty() && local_sfc_build_valid)
                    {
                      if (!side_work_allowed())
                      {
                        local_sfc_build_valid = false;
                        local_sfc_status = "EXECUTION_COVERAGE_DEADLINE";
                        break;
                      }
                      const CorridorTask task = tasks.back();
                      tasks.pop_back();
                      if (try_build_segment(corridor_points[task.i0],
                                            corridor_points[task.i1],
                                            task.i0, task.i1,
                                            task.win_start, task.win_end))
                      {
                        segment_built = true;
                        continue;
                      }
                      const Eigen::Vector3d &tpa = corridor_points[task.i0];
                      const Eigen::Vector3d &tpb = corridor_points[task.i1];
                      const double task_len = (tpb - tpa).norm();
                      if (task.depth < 2 && task_len > 2.0 * map_resolution)
                      {
                        // Geometric midpoint subdivision: split the window
                        // exactly at the inserted point's seed time.
                        ++corridor_subdivision_count;
                        const Eigen::Vector3d mid_point =
                            0.5 * (tpa + tpb);
                        const double mid_enter =
                            0.5 * (corridor_point_enter[task.i0] +
                                   corridor_point_enter[task.i1]);
                        const int mid_index =
                            static_cast<int>(corridor_points.size());
                        corridor_points.push_back(mid_point);
                        corridor_point_enter.push_back(mid_enter);
                        const double split_time = collision_start + mid_enter;
                        tasks.push_back({mid_index, task.i1, task.depth + 1,
                                         split_time, task.win_end});
                        tasks.push_back({task.i0, mid_index, task.depth + 1,
                                         task.win_start, split_time});
                      }
                      else
                      {
                        local_sfc_build_valid = false;
                        local_sfc_status = "CORRIDOR_LEAF_FAILED";
                      }
                    }
                    if (local_sfc_build_valid && !segment_built)
                    {
                      local_sfc_build_valid = false;
                      local_sfc_status = "CORRIDOR_BUILD_FAILED";
                      ROS_WARN("[side-local-corridor] type=%s segment=%d "
                               "result=CORRIDOR_BUILD_FAILED",
                               side > 0 ? "PLUS" : "MINUS", seg);
                    }
                  }
                  if (local_sfc_build_valid &&
                      corridor_points.size() > simplified_path.size())
                  {
                    // Commit every successful leaf into the same ordered
                    // segmentation consumed by MINCO. No geometry is emitted
                    // from a different point list than the corridor audit.
                    std::vector<int> order(corridor_points.size());
                    std::iota(order.begin(), order.end(), 0);
                    std::sort(order.begin(), order.end(), [&](int a, int b) {
                      return corridor_point_enter[a] < corridor_point_enter[b];
                    });
                    std::vector<int> rank(order.size(), -1);
                    std::vector<Eigen::Vector3d> final_points;
                    std::vector<double> final_enter;
                    for (size_t i = 0; i < order.size(); ++i)
                    {
                      if (!final_enter.empty() &&
                          corridor_point_enter[order[i]] <= final_enter.back() + 1.0e-6)
                      {
                        local_sfc_build_valid = false;
                        local_sfc_status = "CORRIDOR_SEGMENT_TIME_INVALID";
                        break;
                      }
                      rank[order[i]] = static_cast<int>(i);
                      final_points.push_back(corridor_points[order[i]]);
                      final_enter.push_back(corridor_point_enter[order[i]]);
                    }
                    if (local_sfc_build_valid)
                    {
                      for (LocalSfcPlane &plane : repaired_local_sfc_planes)
                      {
                        const int a = rank[plane.source_segment];
                        const int b = rank[plane.source_guide_index];
                        if (b != a + 1)
                        {
                          local_sfc_build_valid = false;
                          local_sfc_status = "CORRIDOR_PIECE_BINDING_INVALID";
                          break;
                        }
                        plane.source_segment = a;
                        plane.source_guide_index = b;
                      }
                    }
                    if (local_sfc_build_valid)
                    {
                      const double connection_radius =
                          0.05 * map_resolution;
                      for (int piece = 0;
                           piece + 2 < static_cast<int>(final_points.size());
                           ++piece)
                      {
                        const bool first_has_plane = std::any_of(
                            repaired_local_sfc_planes.begin(),
                            repaired_local_sfc_planes.end(),
                            [&](const LocalSfcPlane &plane) {
                              return plane.source_segment == piece;
                            });
                        const bool second_has_plane = std::any_of(
                            repaired_local_sfc_planes.begin(),
                            repaired_local_sfc_planes.end(),
                            [&](const LocalSfcPlane &plane) {
                              return plane.source_segment == piece + 1;
                            });
                        if (first_has_plane && second_has_plane &&
                            !corridorsOverlapAtJunction(
                                repaired_local_sfc_planes, piece, piece + 1,
                                final_points[piece + 1], connection_radius))
                        {
                          local_sfc_build_valid = false;
                          local_sfc_status = "SFC_OVERLAP_FAILED";
                          break;
                        }
                      }
                    }
                    if (local_sfc_build_valid)
                    {
                      simplified_path = std::move(final_points);
                      corridor_seg_enter = std::move(final_enter);
                      corridor_piece_num = static_cast<int>(simplified_path.size()) - 1;
                      corridor_seg_durations.resize(corridor_piece_num);
                      corridor_seg_lengths.resize(corridor_piece_num);
                      for (int seg = 0; seg < corridor_piece_num; ++seg)
                      {
                        corridor_seg_durations[seg] =
                            corridor_seg_enter[seg + 1] - corridor_seg_enter[seg];
                        corridor_seg_lengths[seg] =
                            (simplified_path[seg + 1] - simplified_path[seg]).norm();
                      }
                    }
                  }
                  if (local_sfc_build_valid)
                    local_sfc_status = repaired_local_sfc_planes.empty()
                                           ? "CORRIDOR_NOT_REQUIRED"
                                           : "CORRIDOR_ACTIVE";
                }
                if (!local_sfc_build_valid)
                {
                  repaired_local_sfc_planes.clear();
                  repaired_side_valid = false;
                }
                const bool local_sfc_handoff_allowed =
                    astar_base_valid && path_static_free &&
                    repaired_side_valid && local_sfc_build_valid;
                ROS_INFO("[local-sfc-handoff] candidate=%s obstacle_id=%d "
                         "astar_static_free=%d semantic_valid=%d plane_count=%zu "
                         "segments=%d subdivisions=%d status=%s handoff_allowed=%d",
                         side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                         nominal_result.risk.obstacle_id,
                         static_cast<int>(path_static_free),
                         static_cast<int>(path_side_valid),
                         repaired_local_sfc_planes.size(), corridor_segment_count,
                         corridor_subdivision_count, local_sfc_status,
                         static_cast<int>(local_sfc_handoff_allowed));
                if (repaired_side_valid)
                {
                  ROS_INFO("[side-local-sfc] type=%s astar_points=%zu guide_points=%zu "
                           "active_start=%.6f active_end=%.6f planes=%zu "
                           "corridor_subset_of_static_free=1 "
                           "adjacent_overlap=1 result=BUILD_SUCCESS",
                           side > 0 ? "PLUS" : "MINUS", raw_path.size(),
                           simplified_path.size(), collision_start, collision_end,
                           repaired_local_sfc_planes.size());
                }
                // Feedback119 (corridor contract, replaces the Feedback117
                // check-rebuild loop): the guide polyline and the bounded
                // corridor above are the geometry contract.  The MINCO seed
                // below is generated ONCE from the guide vertices -- it is a
                // numerical initial value only and is explicitly allowed to
                // cut corners; the corridor hard rows inside the single
                // MINCO-SCP-OSQP chain own static safety.  The former
                // RESTORE_RAW_WAYPOINTS / BULGE_WAYPOINT_AND_CORRIDOR_PLANE
                // repair chain and the seed-collision rejects
                // (SEED_COLLISION_EXPLICIT_FAIL) no longer exist: one SIDE
                // performs at most 1 A* search + 1 corridor build + 1
                // optimization chain.
                bool simplified_path_static_free = !simplified_path.empty();
                for (const Eigen::Vector3d &point : simplified_path)
                  simplified_path_static_free = simplified_path_static_free &&
                      grid_map_->getInflateOccupancy(point) == 0;
                if (local_sfc_build_valid)
                {
                  // Chronological guide: base junctions before the collision
                  // interval, the corridor guide vertices on the shared
                  // curvature-limited timing table, then the downstream
                  // junctions shifted by the detour time surplus.
                  std::vector<double> guide_times;
                  std::vector<Eigen::Vector3d> guide_points;
                  const int base_piece_num = astar_base_traj.getPieceNum();
                  auto append_guide = [&](const double time,
                                          const Eigen::Vector3d &point) {
                    if (guide_times.empty() || time > guide_times.back() + 1.0e-5)
                    {
                      guide_times.push_back(time);
                      guide_points.push_back(point);
                    }
                  };
                  append_guide(0.0, astar_base_traj.getJuncPos(0));
                  double base_time = 0.0;
                  for (int piece = 0; piece < base_piece_num; ++piece)
                  {
                    base_time += astar_base_traj.getDurations()(piece);
                    if (base_time < collision_start - 1.0e-5)
                      append_guide(base_time, astar_base_traj.getJuncPos(piece + 1));
                  }
                  append_guide(collision_start, anchor_start);
                  const int corridor_first_piece =
                      static_cast<int>(guide_points.size()) - 1;
                  double repaired_time = collision_start;
                  for (int seg = 0;
                       seg + 1 < static_cast<int>(simplified_path.size()); ++seg)
                  {
                    repaired_time += corridor_seg_durations[seg];
                    append_guide(repaired_time, simplified_path[seg + 1]);
                  }
                  const double repaired_end_time =
                      collision_start + corridor_repair_duration;
                  append_guide(repaired_end_time, anchor_goal);
                  if (local_sfc_build_valid)
                  {
                    for (LocalSfcPlane &plane : repaired_local_sfc_planes)
                    {
                      plane.piece_id = corridor_first_piece + plane.source_segment;
                      if (plane.piece_id < 0 ||
                          plane.piece_id + 1 >= static_cast<int>(guide_times.size()))
                      {
                        local_sfc_build_valid = false;
                        local_sfc_status = "CORRIDOR_PIECE_BINDING_INVALID";
                        break;
                      }
                      plane.piece_u_begin = 0.0;
                      plane.piece_u_end = 1.0;
                      plane.active_start = guide_times[plane.piece_id];
                      plane.active_end = guide_times[plane.piece_id + 1];
                    }
                    if (!local_sfc_build_valid)
                      repaired_local_sfc_planes.clear();
                  }
                  const double old_collision_window =
                      collision_end - collision_start;
                  const double downstream_time_shift =
                      corridor_repair_duration - old_collision_window;
                  base_time = 0.0;
                  for (int piece = 0; piece < base_piece_num; ++piece)
                  {
                    base_time += astar_base_traj.getDurations()(piece);
                    if (base_time > collision_end + 1.0e-5)
                      append_guide(base_time + downstream_time_shift,
                                   astar_base_traj.getJuncPos(piece + 1));
                  }

                  if (local_sfc_build_valid && guide_points.size() >= 3 &&
                      guide_times.size() == guide_points.size() &&
                      guide_times.back() > guide_times.front() + 1.0e-5)
                  {
                    const int repaired_piece_num =
                        static_cast<int>(guide_points.size()) - 1;
                    Eigen::MatrixXd repaired_inner_points(3,
                                                          repaired_piece_num - 1);
                    Eigen::VectorXd repaired_durations(repaired_piece_num);
                    for (int index = 0; index < repaired_piece_num; ++index)
                    {
                      repaired_durations(index) =
                          guide_times[index + 1] - guide_times[index];
                      if (repaired_durations(index) <= 1.0e-5)
                        repaired_durations(index) = 0.0;
                    }
                    for (int index = 1;
                         index < static_cast<int>(guide_points.size()) - 1;
                         ++index)
                      repaired_inner_points.col(index - 1) = guide_points[index];

                    bool durations_valid = true;
                    for (int index = 0; index < repaired_piece_num; ++index)
                      durations_valid = durations_valid &&
                                        repaired_durations(index) > 1.0e-5;
                    if (durations_valid)
                    {
                      poly_traj::MinJerkOpt repaired_mjo;
                      Eigen::Matrix3d repaired_head, repaired_tail;
                      repaired_head << astar_base_traj.getJuncPos(0),
                          astar_base_traj.getJuncVel(0),
                          astar_base_traj.getJuncAcc(0);
                      repaired_tail << astar_base_traj.getJuncPos(base_piece_num),
                          astar_base_traj.getJuncVel(base_piece_num),
                          astar_base_traj.getJuncAcc(base_piece_num);
                      repaired_mjo.reset(repaired_head, repaired_tail,
                                         repaired_piece_num);
                      repaired_mjo.generate(repaired_inner_points,
                                            repaired_durations);
                      repaired_traj = repaired_mjo.getTraj();

                      double repair_init_max_vel = 0.0;
                      double repair_init_max_acc = 0.0;
                      double repair_init_max_jerk = 0.0;
                      checkTrajectoryDynamics(repaired_traj, repair_init_max_vel,
                                              repair_init_max_acc,
                                              repair_init_max_jerk);
                      const double duration_before = repaired_traj.getTotalDuration();
                      const double v_before = repair_init_max_vel;
                      const double a_before = repair_init_max_acc;
                      const double j_before = repair_init_max_jerk;
                      ROS_INFO("[A_STAR_SEED_TIMING] candidate=%s path_length=%.6f "
                               "piece_count=%d v_before=%.6f a_before=%.6f "
                               "j_before=%.6f lambda=%.6f v_after=%.6f "
                               "a_after=%.6f j_after=%.6f duration_before=%.6f "
                               "duration_after=%.6f",
                               side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                               corridor_path_length, repaired_piece_num,
                               v_before, a_before, j_before, 1.0,
                               repair_init_max_vel, repair_init_max_acc,
                               repair_init_max_jerk, duration_before,
                               repaired_traj.getTotalDuration());
                      double min_repair_piece_duration =
                          std::numeric_limits<double>::infinity();
                      double max_repair_piece_duration = 0.0;
                      for (int seg = 0; seg < corridor_piece_num; ++seg)
                      {
                        min_repair_piece_duration =
                            std::min(min_repair_piece_duration,
                                     corridor_seg_durations[seg]);
                        max_repair_piece_duration =
                            std::max(max_repair_piece_duration,
                                     corridor_seg_durations[seg]);
                      }
                      ROS_INFO("[repair-time-init] path_length=%.6f piece_num=%d "
                               "old_window=%.6f new_duration=%.6f min_piece_T=%.6f "
                               "max_piece_T=%.6f max_curvature=%.6f initial_v=%.6f "
                               "initial_a=%.6f initial_jerk=%.6f",
                               corridor_path_length, corridor_piece_num,
                               old_collision_window, corridor_repair_duration,
                               min_repair_piece_duration,
                               max_repair_piece_duration, corridor_max_curvature,
                               repair_init_max_vel, repair_init_max_acc,
                               repair_init_max_jerk);

                      // Telemetry only -- NOT a gate.  The seed is a numerical
                      // initial value; corner cuts are repaired by the
                      // corridor hard rows in the one MINCO-SCP-OSQP chain.
                      repaired_static_free = true;
                      repaired_static_reason = "SEED_STATIC_FREE";
                      if (grid_map_ && repaired_traj.getPieceNum() > 0 &&
                          repaired_traj.getTotalDuration() > 1.0e-6)
                      {
                        constexpr int kSeedScanSamples = 64;
                        for (int sample = 0; sample < kSeedScanSamples; ++sample)
                        {
                          const double alpha =
                              static_cast<double>(sample) /
                              static_cast<double>(kSeedScanSamples - 1);
                          const double sample_time =
                              alpha * repaired_traj.getTotalDuration();
                          if (grid_map_->getInflateOccupancy(
                                  repaired_traj.getPos(sample_time)) != 0)
                          {
                            repaired_static_free = false;
                            repaired_static_reason = "SEED_CUTS_CORNER";
                            break;
                          }
                        }
                      }
                      const DynamicRiskInfo repaired_risk = evaluateDynamicRisk(
                          repaired_traj, planning_prediction_epoch, touch_goal);
                      repaired_dynamic_valid = repaired_risk.valid &&
                          !repaired_risk.hard_collision;
                      astar_base_risk = repaired_risk;

                      ROS_INFO(
                          "[side-astar-initializer] type=%s rebuilt_static_free=%d "
                          "guide_constructible=%d dynamic_valid=%d "
                          "result=SEED_BUILT_CORRIDOR_SCP_OWNS_SAFETY",
                          side > 0 ? "PLUS" : "MINUS",
                          static_cast<int>(repaired_static_free),
                          static_cast<int>(repaired_side_valid),
                          static_cast<int>(repaired_dynamic_valid));
                    }
                    else
                    {
                      repaired_side_valid = false;
                      repaired_static_reason = "SEED_TIMING_INVALID";
                    }
                  }
                  else
                  {
                    repaired_side_valid = false;
                    repaired_static_reason = "SEED_GUIDE_TOO_SHORT";
                  }
                }

                // Feedback119 (corridor contract): A* raw path static-free +
                // bounded corridor built is the whole
                // frontend contract.  The seed is an initial value; its
                // corner cut (seed_static_free=0) is repaired by the corridor
                // hard rows inside the one MINCO-SCP-OSQP chain and is NEVER
                // an accept gate.  The final authoritative checker below
                // (hard preflight, class C) is unchanged.
                if (path_static_free)
                  ++astar_raw_path_safe_count_;
                if (path_static_free && local_sfc_build_valid)
                {
                  if (repaired_local_sfc_planes.empty())
                    ++sfc_not_required_count_;
                  else
                    ++corridor_build_success_count_;
                }
                else if (path_static_free)
                  ++corridor_build_failed_count_;
                if (path_static_free && repaired_side_valid &&
                    !repaired_static_free)
                  ++astar_safe_but_minco_unsafe_count_;
                if (repaired_static_free)
                  ++minco_seed_static_safe_count_;
                ROS_INFO("[A_STAR_TO_MINCO] drone=%d candidate_type=%s "
                         "raw_path_static_free=%d simplified_path_static_free=%d "
                         "corridor_valid=%d corridor_plane_count=%zu "
                         "corridor_segments=%d corridor_subdivisions=%d "
                         "seed_static_free=%d seed_static_reason=%s "
                         "seed_piece_num=%d contract=SEED_IS_INITIAL_VALUE_ONLY "
                         "verdict=%s",
                         pp_.drone_id, side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                         static_cast<int>(path_static_free),
                         static_cast<int>(simplified_path_static_free),
                         static_cast<int>(local_sfc_build_valid),
                         repaired_local_sfc_planes.size(),
                         corridor_segment_count, corridor_subdivision_count,
                         static_cast<int>(repaired_static_free),
                         repaired_static_reason.c_str(),
                         repaired_traj.getPieceNum(),
                         !repaired_side_valid
                             ? "GUIDE_UNAVAILABLE"
                             : (repaired_static_free ? "SEED_SAFE"
                                                     : "SEED_CUT_CORNER_SCP_REPAIRS"));
                const bool astar_accept = astar_base_valid && path_static_free &&
                                          repaired_side_valid &&
                                          local_sfc_build_valid;
                double simplified_path_length = 0.0;
                for (size_t path_index = 1; path_index < simplified_path.size(); ++path_index)
                  simplified_path_length +=
                      (simplified_path[path_index] - simplified_path[path_index - 1]).norm();
                ROS_INFO(
                    "[side-astar-repair] type=%s trigger_reason=%s base_offset=%.3f "
                    "collision_start=%.6f collision_end=%.6f anchor_start=(%.3f,%.3f,%.3f) "
                    "anchor_goal=(%.3f,%.3f,%.3f) astar_raw_points=%zu "
                    "astar_simplified_points=%zu path_length=%.4f search_time_ms=%.3f "
                    "rejoin_attempt=%d rejoin_index=%d "
                    "static_free=%d static_reason=%s dynamic_valid=%d guide_constructible=%d result=%s",
                    side > 0 ? "PLUS" : "MINUS", static_aware_result, astar_base_offset,
                    collision_start, collision_end, anchor_start.x(), anchor_start.y(),
                    anchor_start.z(), anchor_goal.x(), anchor_goal.y(), anchor_goal.z(),
                    raw_path.size(), simplified_path.size(),
                    simplified_path_length,
                    astar_elapsed_ms, selected_rejoin_attempt, selected_rejoin_index,
                    static_cast<int>(repaired_static_free),
                    repaired_static_reason.c_str(),
                    static_cast<int>(repaired_dynamic_valid), static_cast<int>(repaired_side_valid),
                    astar_accept ? "ASTAR_REPAIR_ACCEPT" :
                                   (astar_ret == ASTAR_RET::SUCCESS ? "ASTAR_REPAIR_REJECT" :
                                                                       "ASTAR_SEARCH_FAILED"));
                ROS_INFO("[side-astar-guide] type=%s raw_points=%zu simplified_points=%zu "
                         "turning_points_retained=%d reinsertion_points=%d",
                         side > 0 ? "PLUS" : "MINUS", raw_path.size(),
                         simplified_path.size(),
                         std::max(0, static_cast<int>(simplified_path.size()) - 2 -
                                      reinsertion_points),
                         reinsertion_points);
                if (!astar_accept && !repaired_local_sfc_planes.empty())
                {
                  const char *frontend_reason =
                      !local_sfc_build_valid
                          ? "CORRIDOR_BUILD_FAILED"
                          : (!repaired_side_valid
                                 ? "GUIDE_UNAVAILABLE"
                                 : (!repaired_dynamic_valid
                                        ? "DYNAMIC_INVALID"
                                        : "ASTAR_REPAIR_REJECT"));
                  logLocalSfcFrontendFinal(
                      repaired_local_sfc_planes, frontend_reason, &repaired_traj,
                      repaired_static_free, repaired_side_valid,
                      repaired_dynamic_valid);
                }
                astar_logged = true;
                if (astar_accept)
                {
                  // Reconstruct once more into the live initializer.  This
                  // avoids copying MinJerkOpt (which owns a banded-system
                  // buffer) and leaves all optimizer state untouched until
                  // the normal candidate path below starts.
                  const int repaired_piece_num = repaired_traj.getPieceNum();
                  Eigen::MatrixXd repaired_inner_points =
                      repaired_traj.getPositions().block(0, 1, 3, repaired_piece_num - 1);
                  Eigen::Matrix3d repaired_head, repaired_tail;
                  repaired_head << repaired_traj.getJuncPos(0), repaired_traj.getJuncVel(0),
                      repaired_traj.getJuncAcc(0);
                  repaired_tail << repaired_traj.getJuncPos(repaired_piece_num),
                      repaired_traj.getJuncVel(repaired_piece_num),
                      repaired_traj.getJuncAcc(repaired_piece_num);
                  side_init_mjo.reset(repaired_head, repaired_tail, repaired_piece_num);
                  side_init_mjo.generate(repaired_inner_points, repaired_traj.getDurations());
                  // Two plane classes with different owners:
                  //   * collision-derived Local-SFC planes are rebuilt by the
                  //     A* repair and may legitimately be replaced;
                  //   * LOS_OBSERVATION_SIDE planes are the semantic topology
                  //     intent (blocker identity / side / blocker-time window)
                  //     and must not be destroyed by a collision rebuild.  The
                  //     replacement below used to drop them, which is the first
                  //     loss point that later showed up as
                  //     REJECT_ALTERNATIVE_LOS_PLANE_LOST at finalization.
                  std::vector<LocalSfcPlane> preserved_los_planes;
                  for (const LocalSfcPlane &plane : local_sfc_planes)
                  {
                    if (plane.source != LocalSfcPlane::LOS_OBSERVATION_SIDE)
                      continue;
                    const bool interval_usable =
                        std::isfinite(plane.active_start) &&
                        std::isfinite(plane.active_end) &&
                        plane.active_end > plane.active_start;
                    if (interval_usable)
                      preserved_los_planes.push_back(plane);
                    else
                    {
                      ++los_plane_lost_count_;
                      ROS_WARN("[los-plane-lifecycle] stage=AFTER_ASTAR "
                               "candidate_id=%d reason_mask=%u blocker_id=%d "
                               "action=LOS_PLANE_INTERVAL_INVALID "
                               "active_start=%.6f active_end=%.6f",
                               nominal_result.candidate_id,
                               nominal_result.conflict.reason_mask,
                               nominal_result.conflict.los_obstacle_identity,
                               plane.active_start, plane.active_end);
                    }
                  }
                  local_sfc_planes = repaired_local_sfc_planes;
                  local_sfc_planes.insert(local_sfc_planes.end(),
                                          preserved_los_planes.begin(),
                                          preserved_los_planes.end());
                  if (!preserved_los_planes.empty())
                    ROS_INFO("[los-plane-lifecycle] stage=AFTER_ASTAR "
                             "candidate_id=%d reason_mask=%u blocker_id=%d "
                             "los_plane_count=%zu total_plane_count=%zu "
                             "action=LOS_PLANE_PRESERVED_ACROSS_ASTAR_REBUILD",
                             nominal_result.candidate_id,
                             nominal_result.conflict.reason_mask,
                             nominal_result.conflict.los_obstacle_identity,
                             preserved_los_planes.size(),
                             local_sfc_planes.size());
                  accepted_side_offset = astar_base_offset;
                  side_static_free = true;
                  side_dynamic_valid = true;
                  side_dynamic_invalid_after_backoff = false;
                  side_init_valid = true;
                  astar_repair_applied = true;
                  ROS_INFO("[side-static-aware] type=%s requested_offset=%.3f "
                           "accepted_offset=%.3f attempts=%d reference_static_free=1 "
                           "dynamic_valid=1 result=ASTAR_REPAIR_ACCEPT",
                           side > 0 ? "PLUS" : "MINUS", requested_side_offset,
                           accepted_side_offset, side_generation_attempts);
                }
              }
              else if (first_collision >= 0)
              {
                const double static_collision_start = side_scope_start +
                    static_cast<double>(first_collision) *
                        (side_scope_end - side_scope_start) /
                        static_cast<double>(astar_samples - 1);
                const double static_collision_end = side_scope_start +
                    static_cast<double>(last_collision) *
                        (side_scope_end - side_scope_start) /
                        static_cast<double>(astar_samples - 1);
                ROS_INFO(
                    "[side-astar-static-window] type=%s side_scope_start=%.6f "
                    "side_scope_end=%.6f dynamic_window_start=%.6f "
                    "dynamic_window_end=%.6f static_collision_found=1 "
                    "static_collision_start=%.6f static_collision_end=%.6f "
                    "anchor_start=NA anchor_goal=NA",
                    side > 0 ? "PLUS" : "MINUS", side_scope_start, side_scope_end,
                    astar_window_start, astar_window_end, static_collision_start,
                    static_collision_end);
              }
              if (!astar_logged)
              {
                ROS_INFO("[side-astar-repair] type=%s trigger_reason=%s base_offset=%.3f "
                         "collision_start=NA collision_end=NA anchor_start=NA anchor_goal=NA "
                         "astar_raw_points=0 astar_simplified_points=0 path_length=0 "
                         "search_time_ms=0 static_free=0 dynamic_valid=0 "
                         "side_semantic_valid=0 result=ASTAR_NO_COLLISION_INTERVAL",
                         side > 0 ? "PLUS" : "MINUS", static_aware_result,
                         astar_base_offset);
              }
            }
          if (!astar_repair_applied)
          {
            // Feedback119: the legacy catch-all label stays for trend
            // continuity, but the physical cause is decomposed into the
            // per-cause lifecycle map.  All remaining causes are geometry
            // class (A): disabled repair, no valid side seed at all, or the
            // A*-repair chain failed (search/guide/corridor -- each already
            // carries its own verdict counter and log line).
            const char *frontend_cause =
                !enable_side_local_astar_repair_ ? "ASTAR_DISABLED"
                : (!astar_base_valid ? "NO_VALID_SIDE_SEED"
                                     : "ASTAR_REPAIR_CHAIN_FAILED");
            ++side_failure_cause_count_[frontend_cause];
            log_side_preinit_failure("NO_STATIC_FEASIBLE_SIDE", nullptr);
            return;
            }
          }
          ROS_INFO("[side-static-aware] type=%s requested_offset=%.3f accepted_offset=%.3f "
                   "attempts=%d reference_static_free=1 dynamic_valid=%d repair_only=%d result=%s",
                   side > 0 ? "PLUS" : "MINUS", requested_side_offset,
                   accepted_side_offset, side_generation_attempts,
                   static_cast<int>(side_dynamic_valid),
                   static_cast<int>(side_repair_only),
                   astar_repair_applied
                       ? "ASTAR_REPAIR_ACCEPT"
                       : (side_repair_only
                              ? "REPAIR_ONLY_TO_OPTIMIZER"
                              : (accepted_side_offset + 1.0e-6 < requested_side_offset
                                     ? "OFFSET_BACKOFF_ACCEPT" : "DIRECT_ACCEPT")));

          // Every SIDE, including an A* handoff, passes through the same
          // fixed-boundary PVA timing initializer as FRESH and N.  The route
          // and STATIC plane piece ids are immutable during this correction.
          poly_traj::Trajectory side_init_traj = side_init_mjo.getTraj();
          if (side_init_traj.getPieceNum() <= 0) {
            log_side_preinit_failure("NUMERICAL_INITIALIZATION_FAILED", nullptr);
            return;
          }
          const int timing_piece_count = side_init_traj.getPieceNum();
          Eigen::Matrix3d timing_head, timing_tail;
          timing_head << side_init_traj.getJuncPos(0),
              side_init_traj.getJuncVel(0), side_init_traj.getJuncAcc(0);
          timing_tail << side_init_traj.getJuncPos(timing_piece_count),
              side_init_traj.getJuncVel(timing_piece_count),
              side_init_traj.getJuncAcc(timing_piece_count);
          const Eigen::MatrixXd timing_inner =
              side_init_traj.getPositions().middleCols(1, timing_piece_count - 1);
          Eigen::VectorXd timing_durations = side_init_traj.getDurations();
          double timing_v = 0.0, timing_a = 0.0, timing_j = 0.0;
          int timing_iterations = 0;
          const bool timing_ok = initializePvaTiming(
              timing_head, timing_tail, timing_inner, timing_durations,
              pp_.max_vel_, pp_.max_acc_, pp_.max_jer_,
              [&](const poly_traj::Trajectory &trajectory, double &v,
                  double &a, double &j) {
                return checkTrajectoryDynamics(trajectory, v, a, j);
              },
              side_init_mjo, timing_v, timing_a, timing_j, timing_iterations);
          if (!timing_ok) {
            ROS_WARN("[PVA_TIMING_INIT] kind=SIDE status=NUMERICAL_INITIALIZATION_FAILED "
                     "iterations=%d", timing_iterations);
            log_side_preinit_failure("NUMERICAL_INITIALIZATION_FAILED",
                                     &side_init_traj);
            return;
          }
          side_init_traj = side_init_mjo.getTraj();
          const Eigen::VectorXd final_side_durations =
              side_init_traj.getDurations();
          for (LocalSfcPlane &plane : local_sfc_planes) {
            if (plane.source != LocalSfcPlane::STATIC_COLLISION_CORRIDOR)
              continue;
            if (plane.piece_id < 0 || plane.piece_id >= timing_piece_count) {
              log_side_preinit_failure("SFC_PIECE_BIND_FAILED", &side_init_traj);
              return;
            }
            plane.active_start = final_side_durations.head(plane.piece_id).sum() +
                                 plane.piece_u_begin * final_side_durations(plane.piece_id);
            plane.active_end = final_side_durations.head(plane.piece_id).sum() +
                               plane.piece_u_end * final_side_durations(plane.piece_id);
          }
          ROS_INFO("[PVA_TIMING_INIT] kind=SIDE status=SUCCESS iterations=%d "
                   "pieces=%d duration=%.6f max_v=%.6f max_a=%.6f max_j=%.6f",
                   timing_iterations, timing_piece_count,
                   side_init_traj.getTotalDuration(), timing_v, timing_a,
                   timing_j);
          double outer_init_max_vel = -1.0;
          double outer_init_max_acc = -1.0;
          double outer_init_max_jerk = -1.0;
          bool outer_init_dyn_ok = false;
          if (side_init_traj.getPieceNum() > 0)
          {
            outer_init_dyn_ok = checkTrajectoryDynamics(
                side_init_traj, outer_init_max_vel, outer_init_max_acc,
                outer_init_max_jerk);
            ROS_INFO("[dynamics-check] candidate_type=%s stage=initial fallback=%d "
                     "duration=%.6f max_vel_actual=%.6f max_vel_limit=%.6f "
                     "max_acc_actual=%.6f max_acc_limit=%.6f max_jerk_actual=%.6f "
                     "max_jerk_limit=%.6f status=%s",
                     side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                     static_cast<int>(side_duration_fallback_used),
                     side_init_traj.getTotalDuration(), outer_init_max_vel,
                     pp_.max_vel_, outer_init_max_acc, pp_.max_acc_,
                     outer_init_max_jerk, pp_.max_jer_,
                     outer_init_dyn_ok ? "PASS" : "FAIL");
          }

          PolyTrajOptimizer::CandidateDynamicsSummary authoritative_pre;
          bool authoritative_pre_valid =
              ploy_traj_opt_->evaluateCandidateDynamicsLattice(
                  side_init_traj, authoritative_pre);
          bool authoritative_dyn_ok =
              authoritative_pre_valid &&
              authoritative_pre.maxViolation() <= 1.0e-6;
          ROS_INFO("[side-authoritative-dynamics] candidate_type=%s stage=pre_retiming "
                   "sample_count=%d max_v=%.6f max_a=%.6f max_j=%.6f "
                   "v_violation=%.6f a_violation=%.6f j_violation=%.6f pass=%d",
                   side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                   authoritative_pre.sample_count,
                   authoritative_pre.max_vel_value,
                   authoritative_pre.max_acc_value,
                   authoritative_pre.max_jerk_value,
                   authoritative_pre.max_vel_violation,
                   authoritative_pre.max_acc_violation,
                   authoritative_pre.max_jerk_violation,
                   static_cast<int>(authoritative_dyn_ok));

          // The same per-piece v/a/j lattice used by final preflight is logged
          // here before the one local MINCO solve.
          const double original_side_duration =
              side_init_traj.getTotalDuration();
          constexpr bool side_retimed = false;
          DynamicRiskInfo side_init_risk = dynamic_risk_at_epoch(
              side_init_traj, planning_prediction_epoch, touch_goal);
          const double side_duration = side_init_traj.getTotalDuration();
          const double active_conflict_time =
              side_init_risk.valid ? side_init_risk.conflict_time
                                   : nominal_result.risk.conflict_time;
          const double active_conflict_progress = side_duration > 1.0e-6
                                                      ? std::max(0.0, std::min(
                                                            1.0, active_conflict_time /
                                                                     side_duration))
                                                      : conflict_progress;
          const double active_guidance_window =
              side_duration > 1.0e-6
                  ? std::min(0.5, std::max(0.08,
                                           kGuidanceWindowSeconds / side_duration))
                  : guidance_window;
          result.local_guidance_window = active_guidance_window;
          result.local_side_conflict_progress = active_conflict_progress;
          result.local_side_offset = accepted_side_offset;
          result.local_side_direction = use_observation_frame
              ? nominal_result.conflict.frame_right
              : (local_target_pt - start_pt).cross(Eigen::Vector3d::UnitZ());
          result.local_side_direction.z() = 0.0;
          if (result.local_side_direction.norm() > 1.0e-6)
            result.local_side_direction.normalize();
          conflict_window_start =
              std::max(0.0, active_conflict_time - kGuidanceWindowSeconds);
          conflict_window_end =
              std::min(side_duration, active_conflict_time + kGuidanceWindowSeconds);
          ROS_INFO("[side-retiming-risk-sync] candidate_type=%s fallback=%d retimed=%d "
                   "prediction_epoch=%.6f conflict_time=%.6f conflict_progress=%.6f "
                   "window_start=%.6f window_end=%.6f duration=%.6f risk_valid=%d "
                   "risk_distance=%.6f",
                   side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                   static_cast<int>(side_duration_fallback_used),
                   static_cast<int>(side_retimed), planning_prediction_epoch,
                   active_conflict_time, active_conflict_progress,
                   conflict_window_start, conflict_window_end, side_duration,
                   static_cast<int>(side_init_risk.valid),
                   side_init_risk.valid ? side_init_risk.min_distance : -1.0);

          Eigen::MatrixXd side_cstr_pts = side_init_mjo.getInitConstraintPoints(
              ploy_traj_opt_->get_cps_num_prePiece_());
          std::vector<std::pair<int, int>> side_segments;
          std::string retimed_static_reason;
          const PolyTrajOptimizer::CHK_RET retimed_static_check =
              ploy_traj_opt_->finelyCheckAndSetConstraintPoints(
                  side_segments, side_init_mjo, true, &retimed_static_reason);
          if (retimed_static_check == PolyTrajOptimizer::CHK_RET::ERR)
          {
            logLocalSfcFrontendFinal(
                local_sfc_planes, "SIDE_INIT_CONSTRAINT_CHECK_FAILED",
                &side_init_traj, false, side_init_valid, side_dynamic_valid);
            log_side_preinit_failure("SIDE_INIT_CONSTRAINT_CHECK_FAILED",
                                     &side_init_traj);
            return;
          }
          // Feedback119 (corridor contract): the former seed re-check gates
          // SIDE_INIT_STATIC_COLLISION and
          // SIDE_INIT_CONFLICT_WINDOW_STATIC_COLLISION are deleted.  The
          // seed is a numerical initial value; a corner cut is repaired by
          // the corridor hard rows inside the single MINCO-SCP-OSQP chain,
          // and final execution safety stays with the class-C hard
          // preflight.  Only a checker error (state corruption) aborts; a
          // cut is reported as telemetry below.
          if (retimed_static_reason != "OBS_FREE")
          {
            ROS_INFO("[side-astar-seed-contract] type=%s "
                     "recheck_static_reason=%s "
                     "action=SEED_CUT_CORRIDOR_SCP_REPAIRS",
                     side > 0 ? "PLUS" : "MINUS",
                     retimed_static_reason.c_str());
          }

          result.init_nominal_max_lateral_deviation =
              max_lateral_deviation(nominal_traj, side_init_traj);
          result.init_nominal_conflict_lateral_deviation =
              conflict_lateral_deviation(side_init_traj);
          const int side_piece_num = side_init_traj.getPieceNum();
          const Eigen::MatrixXd side_positions = side_init_traj.getPositions();
          const Eigen::MatrixXd side_inner_points =
              side_positions.block(0, 1, 3, side_piece_num - 1);
          Eigen::Matrix3d side_head_state, side_tail_state;
          side_head_state << side_init_traj.getJuncPos(0),
              side_init_traj.getJuncVel(0), side_init_traj.getJuncAcc(0);
          side_tail_state << side_init_traj.getJuncPos(side_piece_num),
              side_init_traj.getJuncVel(side_piece_num), side_init_traj.getJuncAcc(side_piece_num);

          // SIDE/A*/Local-SFC has now answered where this topology may fly,
          // but local MINCO has not yet selected its final polynomial.
          // Capture this exact boundary as the early-joint seed.
          result.joint_seed = side_init_mjo;
          result.joint_seed_valid = side_piece_num > 0 &&
              std::isfinite(side_init_traj.getTotalDuration()) &&
              side_init_valid && side_dynamic_valid;
          result.joint_seed_static_constructible = result.joint_seed_valid &&
              retimed_static_reason == "OBS_FREE";
          // Reaching this boundary means any A* repair completed its
          // Local-SFC handoff; invalid handoffs returned above.
          result.joint_seed_local_sfc_valid = result.joint_seed_valid;
          const DynamicRiskInfo joint_seed_risk = evaluateDynamicRisk(
              side_init_traj, planning_prediction_epoch, touch_goal);
          result.joint_seed_dynamic_clearance = joint_seed_risk.valid
              ? joint_seed_risk.min_distance : -1.0;
          result.joint_seed_construction_ms =
              (ros::Time::now() - t_start).toSec() * 1000.0;

          double region_violation_before = 0.0;
          double region_deviation_before = 0.0;
          double region_violation_after = -1.0;
          double final_deviation_from_region = -1.0;
          double optimization_elapsed_ms = 0.0;
          bool feasible_initializer_available = false;
          bool feasible_initializer_fallback_used = false;
          Eigen::Matrix3d feasible_initializer_head;
          Eigen::Matrix3d feasible_initializer_tail;
          Eigen::MatrixXd feasible_initializer_inner_points;
          Eigen::VectorXd feasible_initializer_durations;
          ConstraintPoints feasible_initializer_constraints;
          Eigen::MatrixXd feasible_initializer_display_points;
          DynamicRiskInfo feasible_initializer_risk;
          CandidateSafetyClass feasible_initializer_class =
              CandidateSafetyClass::INVALID;
          double feasible_initializer_local_sfc_violation =
              std::numeric_limits<double>::infinity();
          std::string minco_status = "NOT_RUN";
          // CandidateSideScope 的析构会 clearCandidateSideBias()，
          // 而它内部会 candidate_local_sfc_planes_.clear()。因此 scope 之外再调用
          // getCandidateLocalSfc() 永远得到空集——这正是"SIDE 候选带着硬平面，
          // 但平面从未被求解器施加、也没能进入 result.local_sfc_planes"的真实原因。
          // 在 scope 内、优化完成后立即快照，才是正确的取值时机。
          // 这里只是把求解器工作副本如实取出，不引入第二条求解路径。
          std::vector<LocalSfcPlane> solved_local_sfc_planes;
          {
            ploy_traj_opt_->setGradientAuditContext(
                side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                nominal_result.risk.obstacle_id);
            CandidateSideScope side_scope(ploy_traj_opt_.get(), start_pt,
                                          local_target_pt, side, accepted_side_offset,
                                          active_conflict_progress, active_guidance_window,
                                          enable_candidate_region_constraint_,
                                          use_observation_frame
                                              ? &nominal_result.conflict.frame_right
                                              : nullptr);
            // A* repair candidates carry a sparse local safe corridor. Direct
            // and offset-backoff candidates leave this vector empty and keep
            // their historical optimization path unchanged.
            const double active_time_scale =
                original_side_duration > 1.0e-6
                    ? side_init_traj.getTotalDuration() / original_side_duration
                    : 1.0;
            if (!local_sfc_planes.empty())
            {
              std::vector<LocalSfcPlane> active_local_sfc = local_sfc_planes;
              if (std::isfinite(active_time_scale) &&
                  std::abs(active_time_scale - 1.0) > 1.0e-6)
              {
                for (LocalSfcPlane &plane : active_local_sfc)
                {
                  // 本轮修复（动态 LOS world-time 语义）：静态路径进度型平面的
                  // 时间区间随轨迹时长同比缩放是正确的；但绑定在世界时间遮挡
                  // 事件上的 LOS_OBSERVATION_SIDE 平面绝不能按比例缩放——那会
                  // 把"遮挡发生在世界时刻 t"错位到另一个轨迹位置。这类平面已经
                  // 以本候选的 activation 为原点表达，retime 后保持相对时间不变
                  // 才是正确的世界时间语义。
                  if (plane.world_time_anchored)
                  {
                    // 世界时间锚定平面：不做比例缩放，按本候选 activation 换算。
                    rebaseWorldTimeAnchoredPlane(plane, planning_prediction_epoch);
                    continue;
                  }
                  plane.active_start *= active_time_scale;
                  plane.active_end *= active_time_scale;
                }
              }
              ploy_traj_opt_->setCandidateLocalSfc(active_local_sfc);
              // LOS plane provenance at the one local MINCO boundary.
              // plane must still be present with a usable blocker-time window.
              size_t los_plane_count = 0;
              for (const LocalSfcPlane &plane : active_local_sfc)
                if (plane.source == LocalSfcPlane::LOS_OBSERVATION_SIDE)
                  ++los_plane_count;
              if (los_plane_count > 0)
                ROS_INFO("[los-plane-lifecycle] stage=MINCO_INPUT "
                         "candidate_id=%d candidate_type=%s reason_mask=%u "
                         "blocker_id=%d los_plane_count=%zu "
                         "total_plane_count=%zu active_time_scale=%.6f",
                         nominal_result.candidate_id,
                         nominal_result.kind == CandidateKind::NOMINAL
                             ? "NOMINAL"
                             : (side > 0 ? "SIDE_PLUS" : "SIDE_MINUS"),
                         nominal_result.conflict.reason_mask,
                         nominal_result.conflict.los_obstacle_identity,
                         los_plane_count, active_local_sfc.size(),
                         active_time_scale);
            }
            else
            {
              ploy_traj_opt_->clearCandidateLocalSfc();
              // No plane existed for this candidate at all: nothing was lost.
              // Only report a loss when a LOS plane did exist and disappeared.
              ROS_INFO("[los-plane-lifecycle] stage=MINCO_INPUT candidate_id=%d "
                       "reason_mask=%u blocker_id=%d los_plane_count=0 "
                       "total_plane_count=0 action=NO_PLANE_FOR_CANDIDATE",
                       nominal_result.candidate_id,
                       nominal_result.conflict.reason_mask,
                       nominal_result.conflict.los_obstacle_identity);
            }
            // Candidate-aware objective terms consume the exact side-init
            // trajectory and the same local conflict window used by the
            // existing side guidance/region terms.
            ploy_traj_opt_->setCandidatePreservationReference(side_init_traj);
            ploy_traj_opt_->setCandidateRiskWindow(
                active_conflict_time, kGuidanceWindowSeconds);
            ploy_traj_opt_->evaluateCandidateSideRegion(
                side_init_traj, region_violation_before, region_deviation_before);

            PolyTrajOptimizer::CandidateDynamicsSummary initializer_dynamics;
            const bool initializer_dynamics_ok =
                ploy_traj_opt_->evaluateCandidateDynamicsLattice(
                    side_init_traj, initializer_dynamics) &&
                initializer_dynamics.maxViolation() <= 1.0e-6;
            std::string initializer_static_reason;
            const bool initializer_static_ok = checkTrajectoryStaticSafety(
                side_init_traj, touch_goal, initializer_static_reason);
            feasible_initializer_local_sfc_violation =
                ploy_traj_opt_->evaluateCandidateLocalSfcMaxViolation(
                    side_init_traj);
            const bool initializer_local_sfc_ok =
                std::isfinite(feasible_initializer_local_sfc_violation) &&
                feasible_initializer_local_sfc_violation <= 1.0e-3;
            const double initializer_prediction_epoch = planning_prediction_epoch;
            feasible_initializer_risk = dynamic_risk_at_epoch(
                side_init_traj, initializer_prediction_epoch, touch_goal);
            CandidateResult initializer_contract;
            initializer_contract.success = initializer_dynamics_ok &&
                initializer_static_ok && initializer_local_sfc_ok &&
                side_init_valid && side_dynamic_valid;
            initializer_contract.kind = result.kind;
            initializer_contract.min_jerk_opt.reset(
                side_head_state, side_tail_state, side_init_traj.getPieceNum());
            initializer_contract.min_jerk_opt.generate(
                side_inner_points, side_init_traj.getDurations());
            initializer_contract.prediction_epoch = initializer_prediction_epoch;
            initializer_contract.risk = feasible_initializer_risk;
            classify_candidate(initializer_contract);
            feasible_initializer_class = initializer_contract.safety_class;
            feasible_initializer_available =
                initializer_contract.success &&
                feasible_initializer_class == CandidateSafetyClass::ABSOLUTE_SAFE;
            if (feasible_initializer_available)
            {
              feasible_initializer_head = side_head_state;
              feasible_initializer_tail = side_tail_state;
              feasible_initializer_inner_points = side_inner_points;
              feasible_initializer_durations = side_init_traj.getDurations();
              feasible_initializer_constraints = ploy_traj_opt_->getControlPoints();
              feasible_initializer_display_points =
                  side_init_mjo.getInitConstraintPoints(
                      ploy_traj_opt_->get_cps_num_prePiece_());
            }
            ROS_INFO("[side-feasible-initializer] candidate_type=%s available=%d "
                     "static_ok=%d static_reason=%s dynamics_ok=%d dynamic_ok=%d "
                     "semantic_ok=%d local_sfc_ok=%d local_sfc_violation=%.6f "
                     "risk_valid=%d risk_distance=%.6f safety_class=%s",
                     side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                     static_cast<int>(feasible_initializer_available),
                     static_cast<int>(initializer_static_ok),
                     initializer_static_reason.empty()
                         ? "PASS" : initializer_static_reason.c_str(),
                     static_cast<int>(initializer_dynamics_ok),
                     static_cast<int>(feasible_initializer_risk.valid &&
                                      feasible_initializer_risk.min_distance >=
                                          absolute_dynamic_threshold),
                     static_cast<int>(side_init_valid && side_dynamic_valid),
                     static_cast<int>(initializer_local_sfc_ok),
                     feasible_initializer_local_sfc_violation,
                     static_cast<int>(feasible_initializer_risk.valid),
                     feasible_initializer_risk.valid
                         ? feasible_initializer_risk.min_distance : -1.0,
                     candidateSafetyClassName(feasible_initializer_class));

            // Scheduling recalibration: the batch-wide call reserve
            // (uninterruptible_budget_, 50 ms) matches the nominal solve's
            // time scale.  Measured SIDE solves converge in ~2-5 ms; with the
            // nominal reserve they were refused ENTRY whenever the batch had
            // less than 50 ms left — the common K3-event state — dying at
            // 0.05 ms without a single iteration.  This reserve covers a few
            // LBFGS iterations plus one QP; the batch wall deadline itself is
            // untouched, so predecessor coverage authority is unchanged.
            constexpr double kSideSolveCallReserve = 0.01;
            ploy_traj_opt_->setExecutionDeadline(
                planning_deadline_wall_, kSideSolveCallReserve);
            const auto optimization_start = std::chrono::steady_clock::now();
            result.success = ploy_traj_opt_->optimizeTrajectory(
                side_head_state, side_tail_state, side_inner_points,
                side_init_traj.getDurations(), side_cstr_pts,
                result.optimization_cost);
            optimization_elapsed_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - optimization_start).count();
            result.minco_diagnostics = ploy_traj_opt_->getLastOptimizationDiagnostics();
            minco_status = ploy_traj_opt_->getLastCandidateFinalStatusReason();
            // Feedback116:LOS soft-plane 遥测(SEMI_HARD 模式下 slack>0 的
            // SIDE 不再以 LOS_PLANE_SCP_FAILED 死亡,这里给出逐候选证据)。
            {
              const auto &los_tel = ploy_traj_opt_->lastLosSoftPlaneTelemetry();
              if (los_tel.attempted)
              {
                // Feedback117 (§15): aggregate LOS soft-authority counters.
                ++los_soft_qp_attempt_count_;
                if (los_tel.qp_success)
                  ++los_soft_qp_solved_count_;
                if (los_tel.slack_nonzero_count > 0)
                  ++los_soft_qp_slack_nonzero_count_;
                ROS_INFO("[LOS_SOFT_PLANE] drone=%d candidate_kind=%s "
                         "blocker=%d side=%d sample_count=%d "
                         "slack_nonzero_count=%d slack_max=%.6f "
                         "slack_mean=%.6f slack_cost=%.3f qp_status=%s "
                         "hard_safe_gate=PENDING_FINAL_PREFLIGHT",
                    pp_.drone_id, side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                    nominal_result.conflict.los_obstacle_identity, side,
                    los_tel.sample_count, los_tel.slack_nonzero_count,
                    los_tel.slack_max, los_tel.slack_mean, los_tel.slack_cost,
                    los_tel.qp_status.c_str());
              }
              // Feedback117: structurally dead code path — the LOS soft
              // authority no longer owns any reject.  The counter stays as a
              // runtime proof that INVARIANT 1 holds (must remain 0).
              if (minco_status == "LOS_PLANE_SCP_FAILED")
                ++los_hard_reject_count_;
            }
            // Feedback119: corridor-contract counters from the single
            // MINCO-SCP-OSQP chain (CORRIDOR_SCP_INFEASIBLE separates
            // trajectory-class failures from EXECUTION_DEADLINE; the
            // continuous metric is the exact quintic extremum check).
            {
              const auto &corridor_tel = ploy_traj_opt_->lastCorridorScpTelemetry();
              const bool corridor_candidate = std::any_of(
                  local_sfc_planes.begin(), local_sfc_planes.end(),
                  [](const LocalSfcPlane &plane) {
                    return plane.source ==
                           LocalSfcPlane::STATIC_COLLISION_CORRIDOR;
                  });
              if (corridor_candidate)
              {
                ++corridor_scp_attempt_count_;
                if (result.success)
                {
                  ++corridor_scp_success_count_;
                  if (corridor_tel.final_continuous_violation > 1.0e-3)
                    ++corridor_continuous_violation_count_;
                }
                else if (minco_status.find("DEADLINE") == std::string::npos)
                  ++corridor_scp_infeasible_count_;
                ROS_INFO("[corridor-scp-contract] candidate_type=%s "
                         "scp_attempted=%d scp_success=%d reason=%s "
                         "iterations=%d active_points=%d added_points=%d "
                         "continuous_violation=%.6f sampled_violation=%.6f "
                         "minco_success=%d fallback_used=%d",
                         side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                         static_cast<int>(corridor_tel.attempted),
                         static_cast<int>(corridor_tel.success),
                         corridor_tel.reason.c_str(),
                         corridor_tel.iterations,
                         corridor_tel.active_point_count,
                         corridor_tel.added_point_count,
                         corridor_tel.final_continuous_violation,
                         corridor_tel.final_sampled_violation,
                         static_cast<int>(result.success),
                         static_cast<int>(feasible_initializer_fallback_used));
              }
            }
            ROS_INFO("[side-minco] candidate_type=%s status=%s success=%d "
                     "feasible_initializer_available=%d optimization_ms=%.3f",
                     side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                     minco_status.c_str(), static_cast<int>(result.success),
                     static_cast<int>(feasible_initializer_available),
                     optimization_elapsed_ms);

            if (!result.success && feasible_initializer_available)
            {
              poly_traj::MinJerkOpt fallback_mjo;
              fallback_mjo.reset(feasible_initializer_head,
                                 feasible_initializer_tail,
                                 feasible_initializer_durations.size());
              fallback_mjo.generate(feasible_initializer_inner_points,
                                    feasible_initializer_durations);
              const poly_traj::Trajectory fallback_traj = fallback_mjo.getTraj();
              PolyTrajOptimizer::CandidateDynamicsSummary fallback_dynamics;
              const bool fallback_dynamics_ok =
                  ploy_traj_opt_->evaluateCandidateDynamicsLattice(
                      fallback_traj, fallback_dynamics) &&
                  fallback_dynamics.maxViolation() <= 1.0e-6;
              std::string fallback_static_reason;
              const bool fallback_static_ok = checkTrajectoryStaticSafety(
                  fallback_traj, touch_goal, fallback_static_reason);
              const double fallback_local_sfc_violation =
                  ploy_traj_opt_->evaluateCandidateLocalSfcMaxViolation(
                      fallback_traj);
              const bool fallback_local_sfc_ok =
                  std::isfinite(fallback_local_sfc_violation) &&
                  fallback_local_sfc_violation <= 1.0e-3;
              const double fallback_prediction_epoch = planning_prediction_epoch;
              const DynamicRiskInfo fallback_risk = dynamic_risk_at_epoch(
                  fallback_traj, fallback_prediction_epoch, touch_goal);
              CandidateResult fallback_contract;
              fallback_contract.success = fallback_dynamics_ok &&
                  fallback_static_ok && fallback_local_sfc_ok &&
                  side_init_valid && side_dynamic_valid;
              fallback_contract.kind = result.kind;
              fallback_contract.min_jerk_opt.reset(
                  feasible_initializer_head, feasible_initializer_tail,
                  feasible_initializer_durations.size());
              fallback_contract.min_jerk_opt.generate(
                  feasible_initializer_inner_points,
                  feasible_initializer_durations);
              fallback_contract.prediction_epoch = fallback_prediction_epoch;
              fallback_contract.risk = fallback_risk;
              classify_candidate(fallback_contract);
              const bool fallback_usable = fallback_contract.success &&
                  fallback_contract.safety_class ==
                      CandidateSafetyClass::ABSOLUTE_SAFE;
              ROS_INFO("[side-feasible-fallback] candidate_type=%s used=%d "
                       "reason=%s static_ok=%d dynamics_ok=%d local_sfc_ok=%d "
                       "local_sfc_violation=%.6f risk_valid=%d risk_distance=%.6f "
                       "safety_class=%s",
                       side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                       static_cast<int>(fallback_usable),
                       minco_status.c_str(),
                       static_cast<int>(fallback_static_ok),
                       static_cast<int>(fallback_dynamics_ok),
                       static_cast<int>(fallback_local_sfc_ok),
                       fallback_local_sfc_violation,
                       static_cast<int>(fallback_risk.valid),
                       fallback_risk.valid ? fallback_risk.min_distance : -1.0,
                       candidateSafetyClassName(fallback_contract.safety_class));
              if (fallback_usable)
              {
                // Preserve the SIDE frontend/geometry diagnostics already
                // accumulated in result.  The fallback changes only the
                // executable trajectory and its freshly revalidated safety
                // metadata; replacing the whole CandidateResult would erase
                // the A*/SFC/corridor lifecycle evidence.
                result.success = true;
                // Round-5 measurement: applying the PVA timing initializer to
                // the raw fallback seed did NOT reduce all-fail K3 batches
                // (26 vs 25) — the dilated seeds survived preflight elsewhere
                // and displaced better-optimized payloads during episodes
                // (ALL3 89.3 -> 87.8).  The fallback contract therefore keeps
                // shipping the raw seed; the hard preflight remains its gate.
                result.min_jerk_opt.reset(
                    feasible_initializer_head, feasible_initializer_tail,
                    feasible_initializer_durations.size());
                result.min_jerk_opt.generate(
                    feasible_initializer_inner_points,
                    feasible_initializer_durations);
                result.prediction_epoch = fallback_prediction_epoch;
                result.risk = fallback_risk;
                result.safety_class = fallback_contract.safety_class;
                result.existing_checks_passed =
                    fallback_contract.existing_checks_passed;
                result.constraint_points = feasible_initializer_constraints;
                result.display_points = feasible_initializer_display_points;
                result.optimization_cost =
                    result.min_jerk_opt.getTrajJerkCost();
                result.minco_diagnostics =
                    ploy_traj_opt_->getLastOptimizationDiagnostics();
                feasible_initializer_fallback_used = true;
              }
            }
            else
            {
              ROS_INFO("[side-feasible-fallback] candidate_type=%s used=0 "
                       "reason=%s",
                       side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                       result.success ? "MINCO_SUCCESS" : "NO_FEASIBLE_INITIALIZER");
            }

            const poly_traj::Trajectory minco_final_traj =
                feasible_initializer_fallback_used
                    ? result.min_jerk_opt.getTraj()
                    : ploy_traj_opt_->getMinJerkOpt().getTraj();
            double minco_final_max_vel = -1.0;
            double minco_final_max_acc = -1.0;
            double minco_final_max_jerk = -1.0;
            if (minco_final_traj.getPieceNum() > 0)
            {
              const bool minco_final_dyn_ok =
                  checkTrajectoryDynamics(minco_final_traj,
                                          minco_final_max_vel,
                                          minco_final_max_acc,
                                          minco_final_max_jerk);
              ROS_INFO("[dynamics-check] candidate_type=%s stage=final fallback=%d "
                       "duration=%.6f max_vel_actual=%.6f max_vel_limit=%.6f "
                       "max_acc_actual=%.6f max_acc_limit=%.6f max_jerk_actual=%.6f "
                       "max_jerk_limit=%.6f status=%s",
                      side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                       static_cast<int>(side_duration_fallback_used),
                       minco_final_traj.getTotalDuration(), minco_final_max_vel, pp_.max_vel_,
                       minco_final_max_acc, pp_.max_acc_, minco_final_max_jerk, pp_.max_jer_,
                       minco_final_dyn_ok ? "PASS" : "FAIL");
            }
            DynamicRiskInfo minco_final_risk;
            if (minco_final_traj.getPieceNum() > 0)
            {
              minco_final_risk =
                  evaluateDynamicRisk(minco_final_traj, planning_prediction_epoch, touch_goal);
            }
            const std::string candidate_final_reason =
                feasible_initializer_fallback_used
                    ? std::string("FEASIBLE_INITIALIZER_FALLBACK:") +
                          minco_status
                    : (result.success ? "MINCO_OK" : minco_status);
            ROS_INFO("[candidate-final-status] drone_id=%d candidate_type=%s obstacle_id=%d "
                     "scp_success=%d final_success=%d reason=%s duration=%.6f "
                     "max_vel=%.6f max_acc=%.6f max_jerk=%.6f dynamic_clearance=%.6f "
                     "static_clearance=NA corridor_violation=%.6f",
                     pp_.drone_id, side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                     nominal_result.risk.obstacle_id,
                     static_cast<int>(result.success ||
                                      feasible_initializer_fallback_used),
                     static_cast<int>(result.success),
                     candidate_final_reason.c_str(),
                     minco_final_traj.getPieceNum() > 0 ? minco_final_traj.getTotalDuration() : -1.0,
                     minco_final_max_vel, minco_final_max_acc, minco_final_max_jerk,
                     minco_final_risk.valid ? minco_final_risk.min_distance : -1.0,
                     -1.0);
            if (result.success)
            {
              // On the MINCO success path, result.min_jerk_opt has not yet
              // been populated.  Install the optimizer result before any
              // diagnostic reads it; otherwise getTraj() observes an
              // uninitialized piece count in the default CandidateResult.
              if (!feasible_initializer_fallback_used)
                result.min_jerk_opt = ploy_traj_opt_->getMinJerkOpt();
              ploy_traj_opt_->evaluateCandidateSideRegion(
                  result.min_jerk_opt.getTraj(),
                  region_violation_after, final_deviation_from_region);
            }
            // scope 仍存活：此刻取出的才是"求解器实际工作过的平面集"。
            solved_local_sfc_planes = ploy_traj_opt_->getCandidateLocalSfc();
          }
          ploy_traj_opt_->clearGradientAuditContext();
          if (result.success)
          {
            const poly_traj::Trajectory side_final_traj = result.min_jerk_opt.getTraj();
            result.final_nominal_max_lateral_deviation =
                max_lateral_deviation(nominal_traj, side_final_traj);
            result.final_init_max_lateral_deviation =
                max_lateral_deviation(side_init_traj, side_final_traj);
            result.final_nominal_conflict_lateral_deviation =
                conflict_lateral_deviation(side_final_traj);
            result.final_init_conflict_lateral_deviation =
                conflict_pair_lateral_deviation(side_final_traj, side_init_traj);
            if (!feasible_initializer_fallback_used)
            {
              result.constraint_points = ploy_traj_opt_->getControlPoints();
              result.display_points = result.min_jerk_opt.getInitConstraintPoints(
                  ploy_traj_opt_->get_cps_num_prePiece_());
            }
            // Same batch epoch as the NOMINAL risk and the raw LOS witness: a
            // 20 ms CPU delay must never move the obstacle prediction.
            result.risk = dynamic_risk_at_epoch(
                result.min_jerk_opt.getTraj(), planning_prediction_epoch, touch_goal);
          }
          result.astar_used = astar_repair_applied;
          // An empty plane set means the Local-SFC scan proved that no plane
          // is required; it is still a successful A* -> Local-SFC handoff.
          result.local_sfc_handoff_valid = astar_repair_applied;
          result.local_sfc_used = !local_sfc_planes.empty();
          // Keep the frontend-generated planes even when MINCO rejects the
          // repaired polynomial.  The optimizer may clear its working copy
          // during a failed retry; dropping the local vector here would erase
          // the exact A* -> Local-SFC handoff from production telemetry.
          // 求解器工作副本已在上面的 scope 内快照；这里优先使用快照，
          // 缺失时才退回前端平面集。绝不使用 scope 析构后必然为空的副本。
          result.local_sfc_planes =
              !solved_local_sfc_planes.empty()
                  ? solved_local_sfc_planes
                  : ((astar_repair_applied && !result.success &&
                      !local_sfc_planes.empty())
                         ? local_sfc_planes
                         : local_sfc_planes);
          // The optimizer owns a working copy of the corridor and may clear
          // it while retrying MINCO. Never let that retry erase the
          // independent LOS observation constraint from a BODY+LOS seed.
          // Preserve the frontend LOS plane (and its blocker/side geometry)
          // when the optimizer result no longer contains one.
          const bool frontend_has_los_plane = std::any_of(
              local_sfc_planes.begin(), local_sfc_planes.end(),
              [](const LocalSfcPlane &plane) {
                return plane.source == LocalSfcPlane::LOS_OBSERVATION_SIDE;
              });
          const bool result_has_los_plane = std::any_of(
              result.local_sfc_planes.begin(), result.local_sfc_planes.end(),
              [](const LocalSfcPlane &plane) {
                return plane.source == LocalSfcPlane::LOS_OBSERVATION_SIDE;
              });
          // 段数为 0 的候选没有任何几何可检查（历史上曾在此处 SIGSEGV）。
          const bool result_traj_usable =
              result.min_jerk_opt.getTraj().getPieceNum() > 0 &&
              std::isfinite(result.min_jerk_opt.getTraj().getTotalDuration()) &&
              result.min_jerk_opt.getTraj().getTotalDuration() > 1.0e-6;
          if (frontend_has_los_plane && !result_has_los_plane)
          {
            // 【崩溃修复】候选求解失败时 result.min_jerk_opt 是空轨迹（0 段）。
            // poly_traj::Trajectory::locatePieceIdx() 在段数为 0 时会返回 -1，
            // getPos()/getVel() 随即访问 pieces[-1] 造成 SIGSEGV（实测三个规划器
            // 进程同时崩在 poly_traj_utils.hpp:530 ← planner_manager.cpp 本处）。
            // 段数为 0 的候选没有任何几何可检查，直接判定为不满足该 LOS 平面，
            // 走下面的 else 分支丢弃该候选；绝不采样空轨迹。
            bool final_satisfies_los_plane =
                result.success && result_traj_usable;
            for (const LocalSfcPlane &plane : local_sfc_planes)
            {
              if (!final_satisfies_los_plane)
                break;
              if (plane.source != LocalSfcPlane::LOS_OBSERVATION_SIDE)
                continue;
              if (!std::isfinite(plane.active_start) ||
                  !std::isfinite(plane.active_end) ||
                  !(plane.active_end > plane.active_start))
                continue;
              const double plane_active_start =
                  std::max(0.0, plane.active_start);
              const double plane_active_end = std::min(
                  plane.active_end,
                  result.min_jerk_opt.getTraj().getTotalDuration());
              if (!(plane_active_end > plane_active_start))
                continue;
              const Eigen::Vector3d normal = plane.normal.normalized();
              for (int sample = 0; sample <= 8; ++sample)
              {
                const double t = plane_active_start +
                    (plane_active_end - plane_active_start) * sample / 8.0;
                if (normal.dot(result.min_jerk_opt.getTraj().getPos(t) -
                               plane.point) + 1.0e-6 <
                    plane.clearance)
                {
                  final_satisfies_los_plane = false;
                  break;
                }
              }
            }
            if (final_satisfies_los_plane)
            {
              for (const LocalSfcPlane &plane : local_sfc_planes)
                if (plane.source == LocalSfcPlane::LOS_OBSERVATION_SIDE)
                  result.local_sfc_planes.push_back(plane);
              ROS_WARN("[los-provenance-preserved] drone=%d candidate=%d "
                       "reason_mask=%u action=RESTORE_FRONTEND_OBSERVATION_PLANE",
                       pp_.drone_id, result.candidate_id,
                       result.conflict.reason_mask);
            }
            else
            {
              // Feedback116:LOS observation violation 不再是 candidate reject
              // 条件——"安全、方向正确、本周期尚未完全绕出遮挡"的轨迹必须
              // 活到 C3/D3 比较层。执行资格仍完全由 STATIC/DYNAMIC BODY、
              // SWARM、动力学与真实 collision SFC 的 current-revision 硬检查
              // 决定;LOS 偏差只降级为质量/进度信号(遥测)。
              ++los_plane_violation_count_;
              ROS_WARN("[los-provenance-preserved] drone=%d candidate=%d "
                       "reason_mask=%u action=KEEP_ALTERNATIVE_LOS_PLANE_SOFT "
                       "los_violation=NONZERO authority=HARD_SAFETY_ONLY",
                       pp_.drone_id, result.candidate_id,
                       result.conflict.reason_mask);
            }
          }
          result.feasible_initializer_fallback =
              feasible_initializer_fallback_used;

          // Hard feasibility has one authority.  Clearance gain remains
          // telemetry only; it cannot remove a fully preflighted SIDE from
          // the executable set.  The single visibility comparator below owns
          // quality selection among all such candidates.
          const bool side_absolute_safe_now =
              result.success &&
              result.safety_class == CandidateSafetyClass::ABSOLUTE_SAFE &&
              result_traj_usable && result.risk.valid &&
              nominal_result.risk.valid;
          result.acceptance_checked = true;
          result.candidate_clearance_gain =
              result.success && result.risk.valid && nominal_result.risk.valid
                  ? result.risk.min_distance - nominal_result.risk.min_distance
                  : -std::numeric_limits<double>::infinity();
          result.acceptance_accepted = side_absolute_safe_now;
          ROS_INFO("[clearance-gain-decision] drone=%d candidate_type=%s "
                   "SIDE_ABSOLUTE_SAFE=%d clearance_gain=%.6f "
                   "accepted=%d decision=TELEMETRY_ONLY",
                   pp_.drone_id, side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                   static_cast<int>(side_absolute_safe_now),
                   result.candidate_clearance_gain,
                   static_cast<int>(result.acceptance_accepted));

          const auto &diag = result.minco_diagnostics;
          const auto &diag_init = diag.initial;
          const auto &diag_final = diag.final;
          const double clearance_improvement =
              result.success && result.risk.valid
                  ? result.risk.min_distance - nominal_result.risk.min_distance
                  : 0.0;
          const auto local_reference_deviation =
              [&](const poly_traj::Trajectory &candidate) {
                if (!result.success || candidate.getPieceNum() <= 0)
                  return -1.0;
                double max_deviation = 0.0;
                constexpr int kSamples = 21;
                for (int sample = 0; sample < kSamples; ++sample)
                {
                  const double phase = static_cast<double>(sample) / (kSamples - 1);
                  if (std::abs(phase - active_conflict_progress) >= active_guidance_window)
                    continue;
                  const Eigen::Vector3d ref_pos = side_init_traj.getPos(
                      phase * side_init_traj.getTotalDuration());
                  const Eigen::Vector3d candidate_pos = candidate.getPos(
                      phase * candidate.getTotalDuration());
                  max_deviation = std::max(max_deviation,
                                           (candidate_pos - ref_pos).norm());
                }
                return max_deviation;
              };
          const double preserve_initial_deviation = 0.0;
          const double preserve_final_deviation =
              result.success ? local_reference_deviation(result.min_jerk_opt.getTraj()) : -1.0;
          ROS_INFO("[minco-candidate-diag] drone=%d kind=%s obs=%d valid=%d evals=%d "
                   "side_cost_init=%.4g side_cost_final=%.4g "
                   "side_grad_init=%.4g side_grad_final=%.4g "
                   "moving_cost_init=%.4g moving_cost_final=%.4g "
                   "moving_grad_init=%.4g moving_grad_final=%.4g "
                   "static_cost_final=%.4g static_grad_final=%.4g "
                   "tracking_cost_final=%.4g tracking_grad_final=%.4g "
                   "smooth_cost_final=%.4g feasibility_cost_final=%.4g "
                   "variance_cost_final=%.4g time_cost_final=%.4g total_cost_final=%.4g "
                   "local_guidance_window=%.3f init_nominal_dev=%.3f "
                   "init_conflict_dev=%.3f final_nominal_dev=%.3f "
                   "final_conflict_dev=%.3f "
                   "clearance_improvement=%.4f solver_status=%s",
                   pp_.drone_id,
                   side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                   nominal_result.risk.obstacle_id,
                   static_cast<int>(diag.valid), diag.evaluations,
                   diag_init.side_cost, diag_final.side_cost,
                   diag_init.side_grad_norm, diag_final.side_grad_norm,
                   diag_init.moving_cost, diag_final.moving_cost,
                   diag_init.moving_grad_norm, diag_final.moving_grad_norm,
                   diag_final.static_cost, diag_final.static_grad_norm,
                   diag_final.tracking_cost, diag_final.tracking_grad_norm,
                   diag_final.smoothness_cost, diag_final.feasibility_cost,
                   diag_final.variance_cost, diag_final.time_cost, diag_final.total_cost,
                   result.local_guidance_window,
                   result.init_nominal_max_lateral_deviation,
                   result.init_nominal_conflict_lateral_deviation,
                   result.final_nominal_max_lateral_deviation,
                   result.final_nominal_conflict_lateral_deviation,
                   clearance_improvement,
                   result.success ? "SUCCESS" : "FAILED");
          ROS_INFO("[side-region] drone=%d candidate=%s conflict_window=[%.3f,%.3f] "
                   "region_violation_before=%.4f region_violation_after=%.4f "
                   "final_deviation_from_region=%.4f nominal_clearance=%.4f "
                   "candidate_clearance=%.4f optimization_ms=%.3f success=%d",
                   pp_.drone_id, side > 0 ? "PLUS" : "MINUS",
                   conflict_window_start, conflict_window_end,
                   region_violation_before, region_violation_after,
                   final_deviation_from_region,
                   nominal_result.risk.min_distance,
                   result.success && result.risk.valid ? result.risk.min_distance : -1.0,
                   optimization_elapsed_ms, static_cast<int>(result.success));
          const double region_width = std::max(
              0.10, 0.25 * std::abs(accepted_side_offset *
                                    std::sin(M_PI * active_conflict_progress)));
          ROS_INFO("[candidate-region] drone=%d candidate_type=%s obstacle_id=%d "
                   "region_type=%s "
                   "conflict_window=[%.3f,%.3f] region_width=%.4f "
                   "initial_region_violation=%.4f final_region_violation=%.4f "
                   "region_cost=%.6g final_clearance=%.4f final_conflict_deviation=%.4f "
                   "enabled=%d",
                   pp_.drone_id, side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                   nominal_result.risk.obstacle_id,
                   ploy_traj_opt_->getCandidateRegionType().c_str(),
                   conflict_window_start, conflict_window_end, region_width,
                   region_violation_before, region_violation_after,
                   diag_final.side_region_cost,
                   result.success && result.risk.valid ? result.risk.min_distance : -1.0,
                   result.final_nominal_conflict_lateral_deviation,
                   static_cast<int>(enable_candidate_region_constraint_));
          ROS_INFO("[candidate-preserve] drone=%d candidate_type=%s window=[%.3f,%.3f] "
                   "initial_deviation=%.4f final_deviation=%.4f preserve_cost=%.6g "
                   "enabled=%d",
                   pp_.drone_id, side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                   conflict_window_start, conflict_window_end,
                   preserve_initial_deviation, preserve_final_deviation,
                   diag_final.preserve_cost,
                   static_cast<int>(diag_final.preserve_cost > 0.0));
          ROS_INFO("[moving-risk] drone=%d candidate_type=%s time=%.4f distance=%.4f "
                   "weight=%.4f cost_contribution=%.6g",
                   pp_.drone_id, side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                   diag_final.moving_risk_time, diag_final.moving_risk_distance,
                   diag_final.moving_risk_weight, diag_final.moving_risk_cost);
          const double initial_clearance =
              side_init_risk.valid ? side_init_risk.min_distance : -1.0;
          const double final_clearance =
              result.success && result.risk.valid ? result.risk.min_distance : -1.0;
          const double initial_length = sampled_trajectory_length(side_init_traj);
          const double final_length = result.success
                                          ? sampled_trajectory_length(result.min_jerk_opt.getTraj())
                                          : -1.0;
          const double initial_smoothness = side_init_mjo.getTrajJerkCost();
          const double final_smoothness = result.success
                                              ? result.min_jerk_opt.getTrajJerkCost()
                                              : -1.0;
          ROS_INFO("[candidate-ground-truth] candidate_type=%s obstacle_id=%d "
                   "conflict_time=%.3f initial_min_clearance=%.4f "
                   "final_min_clearance=%.4f collision_count=UNAVAILABLE "
                   "trajectory_length_initial=%.4f trajectory_length_final=%.4f "
                   "smoothness_metric_initial=%.4f smoothness_metric_final=%.4f "
                   "optimization_success=%d solver_status=%s",
                   side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                   nominal_result.risk.obstacle_id,
                   nominal_result.risk.conflict_time,
                   initial_clearance, final_clearance,
                   initial_length, final_length,
                   initial_smoothness, final_smoothness,
                   static_cast<int>(result.success),
                   result.success ? "SUCCESS" : "FAILED");
          ROS_INFO("[candidate-acceptance] candidate_type=%s obstacle_id=%d "
                   "nominal_clearance=%.4f initial_candidate_clearance=%.4f "
                   "final_candidate_clearance=%.4f clearance_gain=%.4f "
                   "accepted=%d decision=HARD_FEASIBILITY enabled=1",
                   side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                   nominal_result.risk.obstacle_id,
                   nominal_result.risk.valid ? nominal_result.risk.min_distance : -1.0,
                   side_init_risk.valid ? side_init_risk.min_distance : -1.0,
                   result.success && result.risk.valid ? result.risk.min_distance : -1.0,
                   result.candidate_clearance_gain,
                   static_cast<int>(result.acceptance_accepted));
        };


        const auto candidate_is_accepted = [&](const CandidateResult &candidate) {
          return candidate.acceptance_accepted;
        };

        const auto executable_local = [&](const CandidateResult &candidate) {
          return candidate.success &&
              candidate.safety_class==CandidateSafetyClass::ABSOLUTE_SAFE &&
              candidate_is_accepted(candidate);
        };


        const auto attempt_side = [&](const int side, const bool mandatory_supply) {
          if (preserve_active_observation_side)
            return;
          // Budget-aware skip: in the low-coverage band the whole batch wall
          // budget is a few ms; measured side solves entered with <10 ms left
          // and died at the entry checkpoint (optimization_ms P50 = 0.08)
          // after nominal's warm solve had already consumed the window.
          // Skipping here ends a doomed batch immediately so the next
          // rolling tick refreezes with a full budget — the same
          // no-blocking, refreeze-later contract as MISSED_FROZEN_ACTIVATION.
          const double wall_remaining =
              planning_deadline_wall_ - ros::WallTime::now().toSec();
          if (std::isfinite(planning_deadline_wall_) && wall_remaining < 0.02)
          {
            if (side > 0)
              plus_attempted = true;
            else
              minus_attempted = true;
            ROS_INFO("[side-budget-skip] drone=%d side=%s wall_remaining=%.4f "
                     "reason=PIPELINE_BUDGET_EXHAUSTED",
                pp_.drone_id, side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                wall_remaining);
            return;
          }
          // 本轮审计：区分两种"没有 BODY/LOS 描述符证据"的 SIDE 尝试。
          //   * MISSING_EXECUTABLE_NOMINAL —— 本轮 NOMINAL 不可执行，SIDE 只是
          //     安全供给兜底，与 visibility 无关（既有冻结语义）；
          //   * SOFT_VISIBILITY_AUTHORITY —— 仅凭软 visibility 支撑就发起 SIDE
          //     搜索；修复 1 之后必须恒为 0。
          {
            const bool descriptor_backed =
                (nominal_result.conflict.reason_mask & CONFLICT_BODY_SAFETY) != 0 ||
                (nominal_result.conflict.reason_mask & CONFLICT_LOS_OCCLUSION) != 0;
            if (!descriptor_backed)
            {
              if (!executable_local(nominal_result))
              {
                ++side_without_descriptor_nominal_fallback_count_;
                ROS_INFO_THROTTLE(0.5,
                    "[side-without-descriptor] drone=%d side=%s cause="
                    "MISSING_EXECUTABLE_NOMINAL action=SAFETY_SUPPLY_FALLBACK "
                    "reason_mask=%u",
                    pp_.drone_id, side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                    nominal_result.conflict.reason_mask);
              }
              if (static_visibility_support)
              {
                ++soft_vis_only_triggered_side_count_;
                ROS_WARN("[soft-visibility-topology-violation] drone=%d side=%s "
                         "cause=SOFT_VISIBILITY_AUTHORITY reason_mask=%u "
                         "directional_static=%.6g raw_static_los_blocked=%d",
                         pp_.drone_id, side > 0 ? "SIDE_PLUS" : "SIDE_MINUS",
                         nominal_result.conflict.reason_mask,
                         nominal_visibility_diag.directional_static_los_cost_max,
                         static_cast<int>(raw_static_los_blocked));
              }
            }
          }
          if (side > 0)
          {
            if (!plus_attempted)
            {
              plus_attempted = true;
              optimize_side(side, plus_result, mandatory_supply);
            }
          }
          else if (!minus_attempted)
          {
            minus_attempted = true;
            optimize_side(side, minus_result, mandatory_supply);
          }
        };
        if(execution_reserve_limited_)
        {
          // Phase 1 finds the first safe local supply.  Phase 2 retains that
          // candidate while the same real deadline still permits the missing
          // observation topology to be solved.  The optimizer checkpoints and
          // the existing uninterruptible reserve prevent quality work from
          // consuming the final handoff budget.
          if(!executable_local(nominal_result))
          {
            const int first=preferred_side_valid ? preferred_side : +1;
            attempt_side(first, true);
            classify_candidate(first>0 ? plus_result : minus_result);
            if(!executable_local(first>0 ? plus_result : minus_result) &&
               mandatoryPlanningAttemptAllowed())
            {
              attempt_side(-first, true);
              classify_candidate(first>0 ? minus_result : plus_result);
            }
          }
          const bool first_safe_available=executable_local(nominal_result) ||
              executable_local(plus_result) || executable_local(minus_result);
          if(first_safe_available)
          {
            const CandidateResult *first_safe=executable_local(nominal_result) ? &nominal_result :
                (executable_local(plus_result) ? &plus_result : &minus_result);
            ROS_WARN("[coverage-quality] event=FIRST_SAFE_SUPPLY_HELD drone=%d candidate=%d kind=%s planning_budget_remaining=%.6f",
                pp_.drone_id,first_safe->candidate_id,candidateKindName(first_safe->kind),
                std::max(0.0,planning_deadline_wall_-ros::WallTime::now().toSec()));
            const int order[2]={preferred_side_valid ? preferred_side : +1,
                                preferred_side_valid ? -preferred_side : -1};
            for(const int side:order)
            {
              const bool attempted=side>0 ? plus_attempted : minus_attempted;
              if(attempted) continue;
              const bool deadline_available=optionalRefinementAllowed();
              if(!deadlineAwareAlternativeAllowed(deadline_available,true,true))
              {
                ROS_WARN("[coverage-quality] event=QUALITY_SEARCH_ABORTED_FOR_DEADLINE drone=%d reserved_candidate=%d phase=SIDE_GENERATION t_planning_deadline=%.9f",
                    pp_.drone_id,first_safe->candidate_id,planning_deadline_ros_);
                break;
              }
              ROS_INFO("[coverage-quality] event=QUALITY_SEARCH_AFTER_FIRST_SAFE drone=%d reserved_candidate=%d phase=SIDE_GENERATION side=%s planning_budget_remaining=%.6f",
                  pp_.drone_id,first_safe->candidate_id,side>0 ? "SIDE_PLUS" : "SIDE_MINUS",
                  std::max(0.0,planning_deadline_wall_-ros::WallTime::now().toSec()));
              attempt_side(side, false);
              classify_candidate(side>0 ? plus_result : minus_result);
            }
          }
          ROS_WARN("[execution-reserve] drone=%d action=FIRST_EXECUTABLE_LOCAL_SELECTION nominal_ready=%d plus_ready=%d minus_ready=%d plus_attempted=%d minus_attempted=%d",
              pp_.drone_id,int(executable_local(nominal_result)),
              int(executable_local(plus_result)),int(executable_local(minus_result)),
              int(plus_attempted),int(minus_attempted));
        }
        else if (!preferred_side_valid)
        {
          attempt_side(+1, false);
          attempt_side(-1, false);
        }
        else
        {
          attempt_side(preferred_side, false);
          // Preferred side is only an attempt-order hint. Always solve the
          // opposite side so the complete N/L/R hard-safe set reaches the one
          // current-revision visibility-first comparator.
          attempt_side(-preferred_side, false);
        }

        classify_candidate(plus_result);
        classify_candidate(minus_result);

        const auto visibility_report_for = [&](CandidateResult &candidate) {
          if (!candidate.success || candidate.min_jerk_opt.getTraj().getPieceNum() <= 0)
            return;
          candidate.visibility = evaluateCandidateVisibility(
              candidate.min_jerk_opt.getTraj(), planning_prediction_epoch,
              geometry_target_p_, geometry_target_v_,
              geometry_target_stamp_, encirclement_generation > 0);
        };
        visibility_report_for(nominal_result);
        visibility_report_for(plus_result);
        visibility_report_for(minus_result);

        CandidateVisibilityReport previous_visibility;
        if (active_candidate_metadata_valid_ &&
            traj_.local_traj.traj.getPieceNum() > 0)
        {
          poly_traj::Trajectory previous_remaining;
          const double previous_elapsed = std::max(
              0.0, planning_prediction_epoch - traj_.local_traj.start_time);
          if (sliceTrajectory(traj_.local_traj.traj, previous_elapsed,
                              previous_remaining))
            previous_visibility = evaluateCandidateVisibility(
                previous_remaining, planning_prediction_epoch,
                geometry_target_p_, geometry_target_v_,
                geometry_target_stamp_, encirclement_generation > 0);
        }

        const auto log_visibility = [&](const char *type,
                                        const CandidateSafetyClass safety_class,
                                        const double clearance,
                                        const CandidateVisibilityReport &visibility) {
          ROS_INFO("[candidate-visibility] candidate_type=%s safety_class=%s "
                   "dynamic_clearance=%.6f sample_count=%d "
                   "uav1_visibility=%.6f uav2_visibility=%.6f "
                   "uav3_visibility=%.6f all3_visibility=%.6f "
                   "atleast2_visibility=%.6f none_visibility=%.6f "
                   "atleast_k_visibility=%.6f mean_visible_count=%.6f "
                   "min_pairwise_angle_deg=%.6f mean_pairwise_angle_deg=%.6f "
                   "diversity_score=%.6f team_utility=%.6f "
                   "max_loss_duration=%.6f visibility_valid=%d "
                   "fov_mode=DIAGNOSTIC_UNAVAILABLE",
                   type, candidateSafetyClassName(safety_class), clearance,
                   visibility.sample_count, visibility.uav_visibility[0],
                   visibility.uav_visibility[1], visibility.uav_visibility[2],
                   visibility.all3_visibility, visibility.atleast2_visibility,
                   visibility.none_visibility, visibility.atleast_k_visibility,
                   visibility.mean_visible_count,
                   visibility.min_pairwise_angle_deg,
                   visibility.mean_pairwise_angle_deg,
                   visibility.diversity_score, visibility.team_utility,
                   visibility.max_loss_duration,
                   static_cast<int>(visibility.valid));
        };
        log_visibility("NOMINAL", nominal_result.safety_class,
                       nominal_result.risk.valid ? nominal_result.risk.min_distance : -1.0,
                       nominal_result.visibility);
        log_visibility("SIDE_PLUS", plus_result.safety_class,
                       plus_result.risk.valid ? plus_result.risk.min_distance : -1.0,
                       plus_result.visibility);
        log_visibility("SIDE_MINUS", minus_result.safety_class,
                       minus_result.risk.valid ? minus_result.risk.min_distance : -1.0,
                       minus_result.visibility);
        log_visibility(active_candidate_kind_ == CandidateKind::SIDE_PLUS
                           ? "PREVIOUS_SIDE_PLUS"
                           : active_candidate_kind_ == CandidateKind::SIDE_MINUS
                                 ? "PREVIOUS_SIDE_MINUS"
                                 : "PREVIOUS_NOMINAL",
                       previous_safe_available
                           ? CandidateSafetyClass::ABSOLUTE_SAFE
                           : CandidateSafetyClass::INVALID,
                       previous_min_dynamic_clearance, previous_visibility);

        const bool plus_good =
            plus_result.safety_class != CandidateSafetyClass::INVALID &&
            candidate_is_accepted(plus_result);
        const bool minus_good =
            minus_result.safety_class != CandidateSafetyClass::INVALID &&
            candidate_is_accepted(minus_result);
        // Feedback117 (§10): explicit SIDE lifecycle counting after the hard
        // classification (never before it — Feedback115 showed that reading
        // the default INVALID earlier fabricated failures):
        //   success && classify INVALID → SIDE_HARD_PREFLIGHT_FAILED
        //   success && accepted         → SIDE_HARD_VALID
        // and below, after the commit-chain pick:
        //   hard-valid but not picked → SIDE_VALID_NOT_SELECTED
        //   picked                    → SIDE_SELECTED (pre-commit).
        const auto count_side_post_classify =
            [&](const bool attempted, const CandidateResult &candidate) {
              if (!attempted || !candidate.success)
                return;
              if (candidate.safety_class == CandidateSafetyClass::INVALID)
                ++side_hard_preflight_failed_count_;
              else
                ++side_hard_valid_count_;
            };
        count_side_post_classify(plus_attempted, plus_result);
        count_side_post_classify(minus_attempted, minus_result);
        // Construction does not rank N/L/R.  It only exports the complete
        // hard-feasible set.  Current-revision preflight and the one shared
        // visibility-first comparator below own the final decision.
        if (nominal_result.safety_class == CandidateSafetyClass::ABSOLUTE_SAFE)
          selected_result = &nominal_result;
        else if (plus_good)
          selected_result = &plus_result;
        else if (minus_good)
          selected_result = &minus_result;

        const auto count_side_selection =
            [&](const bool attempted, const CandidateResult &candidate) {
              if (!attempted || !candidate.success ||
                  candidate.safety_class == CandidateSafetyClass::INVALID ||
                  !candidate_is_accepted(candidate))
                return;
              if (selected_result == &candidate)
                ++side_selected_count_;
              else
                ++side_valid_not_selected_count_;
            };
        count_side_selection(plus_attempted, plus_result);
        count_side_selection(minus_attempted, minus_result);

        // If both observation-side alternatives fail, no special authority is
        // created. A hard-safe NOMINAL remains in the same executable set;
        // otherwise the validated predecessor is retained and this batch fails
        // closed.
        // Feedback116:side 双侧失败判定移到 classify 之后才可靠。这里先记录
        // 构造阶段事实(success=MINCO/优化产物存在),不再在 safety_class 分类
        // 前读默认 INVALID 当作几何失败(Feedback115:56/108 对被误报)。
        const bool side_construction_failed =
            (!plus_result.success && !minus_result.success) &&
            (plus_attempted && minus_attempted);
        if (side_construction_failed)
          ++side_construction_failed_count_;
        if ((plus_attempted || minus_attempted) && !plus_result.success &&
            !minus_result.success)
          ROS_WARN("[side-construction] drone=%d PLUS_attempted=%d "
                   "MINUS_attempted=%d PLUS_success=%d MINUS_success=%d "
                   "PLUS_status=%s MINUS_status=%s action=NOMINAL_FALLBACK",
                   pp_.drone_id, static_cast<int>(plus_attempted),
                   static_cast<int>(minus_attempted),
                   static_cast<int>(plus_result.success),
                   static_cast<int>(minus_result.success),
                   ploy_traj_opt_->getLastCandidateFinalStatusReason().c_str(),
                   ploy_traj_opt_->getLastCandidateFinalStatusReason().c_str());

        ROS_INFO("[visibility-selection] stage=CONSTRUCTION candidate_set=N,L,R "
                 "decision=DEFER_TO_CURRENT_REVISION_PREFLIGHT_AND_ONE_COMPARATOR");

        const auto candidate_clearance = [&](const CandidateResult &candidate) {
          return candidate.risk.min_distance - nominal_result.risk.min_distance;
        };
        const auto candidate_status = [&](const CandidateResult &candidate,
                                          const bool attempted) {
          if (!attempted)
            return std::string("not-needed");
          if (!candidate.success)
            return std::string("FAILED");
          if (!candidate.risk.valid)
            return std::string("INVALID");
          std::ostringstream status;
          status << candidate.risk.min_distance << ":imp=" << candidate_clearance(candidate);
          return status.str();
        };
        const char *geometry = preferred_side_valid
                                   ? (preferred_side > 0 ? "preferred-plus" : "preferred-minus")
                                   : "ambiguous";
        const char *selected_name = selected_result->kind == CandidateKind::SIDE_PLUS
                                        ? "SIDE_PLUS"
                                        : selected_result->kind == CandidateKind::SIDE_MINUS
                                              ? "SIDE_MINUS"
                                              : "NOMINAL";
        const int side_attempts = static_cast<int>(plus_attempted) +
                                   static_cast<int>(minus_attempted);
        const auto candidate_clearance_or_invalid = [&](const CandidateResult &candidate,
                                                         const bool attempted) {
          return attempted && candidate.risk.valid ? candidate.risk.min_distance : -1.0;
        };
        const char *selection_reason = "DEFERRED_TO_FINAL_COMPARATOR";
        if (selected_result->safety_class == CandidateSafetyClass::ABSOLUTE_SAFE)
          selected_safety_reason = "NEW_ABSOLUTE_SAFE";
        else if (previous_safe_available)
          selected_safety_reason = "KEEP_PREVIOUS_ABSOLUTE_SAFE";
        else
          selected_safety_reason = "UNSAFE_FALLBACK";
        ROS_INFO("[side-bilateral] preferred=%d plus_attempted=%d minus_attempted=%d "
                 "plus_good=%d minus_good=%d plus_clearance=%.6f minus_clearance=%.6f "
                 "selected=%s selection_reason=%s",
                 preferred_side_valid ? preferred_side : 0,
                 static_cast<int>(plus_attempted), static_cast<int>(minus_attempted),
                 static_cast<int>(plus_good), static_cast<int>(minus_good),
                 candidate_clearance_or_invalid(plus_result, plus_attempted),
                 candidate_clearance_or_invalid(minus_result, minus_attempted),
                 selected_name, selection_reason);
        ROS_INFO("[candidate-safety-class] type=NOMINAL min_dynamic_distance=%.6f "
                 "nominal_distance=%.6f absolute_threshold=%.6f class=%s "
                 "selection_reason=%s",
                 nominal_result.risk.valid ? nominal_result.risk.min_distance : -1.0,
                 nominal_result.risk.valid ? nominal_result.risk.min_distance : -1.0,
                 absolute_dynamic_threshold,
                 candidateSafetyClassName(nominal_result.safety_class),
                 selected_result == &nominal_result ? selected_safety_reason.c_str()
                                                    : "NOT_SELECTED");
        ROS_INFO("[candidate-safety-class] type=SIDE_PLUS min_dynamic_distance=%.6f "
                 "nominal_distance=%.6f absolute_threshold=%.6f class=%s "
                 "selection_reason=%s",
                 plus_result.risk.valid ? plus_result.risk.min_distance : -1.0,
                 nominal_result.risk.min_distance, absolute_dynamic_threshold,
                 candidateSafetyClassName(plus_result.safety_class),
                 selected_result == &plus_result ? selected_safety_reason.c_str()
                                                 : "NOT_SELECTED");
        ROS_INFO("[candidate-safety-class] type=SIDE_MINUS min_dynamic_distance=%.6f "
                 "nominal_distance=%.6f absolute_threshold=%.6f class=%s "
                 "selection_reason=%s",
                 minus_result.risk.valid ? minus_result.risk.min_distance : -1.0,
                 nominal_result.risk.min_distance, absolute_dynamic_threshold,
                 candidateSafetyClassName(minus_result.safety_class),
                 selected_result == &minus_result ? selected_safety_reason.c_str()
                                                  : "NOT_SELECTED");
        candidate_safety_logged = true;
        ROS_INFO("[risk-candidate] drone=%d triggered=1 obs=%d geometry=%s nominal=%.3f plus=%s minus=%s "
                 "provisional=%s side_attempts=%d final_decision=DEFERRED "
                 "plus_init_dev=%.3f plus_final_nom_dev=%.3f plus_final_init_dev=%.3f "
                 "plus_init_conf=%.3f plus_final_nom_conf=%.3f plus_final_init_conf=%.3f "
                 "minus_init_dev=%.3f minus_final_nom_dev=%.3f minus_final_init_dev=%.3f "
                 "minus_init_conf=%.3f minus_final_nom_conf=%.3f minus_final_init_conf=%.3f "
                 "epoch=%.6f",
                 pp_.drone_id, nominal_result.risk.obstacle_id, geometry,
                 nominal_result.risk.min_distance,
                 candidate_status(plus_result, plus_attempted).c_str(),
                 candidate_status(minus_result, minus_attempted).c_str(),
                 selected_name, side_attempts,
                 plus_result.init_nominal_max_lateral_deviation,
                 plus_result.final_nominal_max_lateral_deviation,
                 plus_result.final_init_max_lateral_deviation,
                 plus_result.init_nominal_conflict_lateral_deviation,
                 plus_result.final_nominal_conflict_lateral_deviation,
                 plus_result.final_init_conflict_lateral_deviation,
                 minus_result.init_nominal_max_lateral_deviation,
                 minus_result.final_nominal_max_lateral_deviation,
                 minus_result.final_init_max_lateral_deviation,
                 minus_result.init_nominal_conflict_lateral_deviation,
                 minus_result.final_nominal_conflict_lateral_deviation,
                 minus_result.final_init_conflict_lateral_deviation,
                 planning_prediction_epoch);

        // Feedback116:双侧真实终止审计移到 classify/preflight 之后的语义点,
        // 用构造阶段事实(success)+ 分类结果,不再提前读默认 INVALID。
        {
          const bool plus_alive = plus_result.success &&
              plus_result.min_jerk_opt.getTraj().getPieceNum() > 0;
          const bool minus_alive = minus_result.success &&
              minus_result.min_jerk_opt.getTraj().getPieceNum() > 0;
          if ((plus_attempted || minus_attempted) && !plus_alive && !minus_alive)
          {
            if (!nominal_result.success ||
                nominal_result.min_jerk_opt.getTraj().getPieceNum() <= 0)
              ++side_both_failed_no_successor_count_;
            if (k3_event_.active)
              ++k3_event_.side_both_failed_count;
            ROS_WARN("[side-both-failed-fallback] drone=%d generation=%lu "
                     "nominal_id=%d L_status=%s R_status=%s selected=%s "
                     "reason=SIDE_BOTH_FAILED_FALLBACK_NOMINAL",
                     pp_.drone_id,
                     static_cast<unsigned long>(active_traj_generation_),
                     nominal_result.candidate_id,
                     candidate_status(plus_result, plus_attempted).c_str(),
                     candidate_status(minus_result, minus_attempted).c_str(),
                     candidateKindName(selected_result->kind));
          }
        }

        }
      }

    if (enable_risk_triggered_candidates_ && obj_predictor_)
    {
      if (!candidate_safety_logged)
      {
        if (nominal_result.safety_class == CandidateSafetyClass::ABSOLUTE_SAFE)
          selected_safety_reason = "NEW_ABSOLUTE_SAFE";
        else if (previous_safe_available)
          selected_safety_reason = "KEEP_PREVIOUS_ABSOLUTE_SAFE";
        else
          selected_safety_reason = "UNSAFE_FALLBACK";
        ROS_INFO("[candidate-safety-class] type=NOMINAL min_dynamic_distance=%.6f "
                 "nominal_distance=%.6f absolute_threshold=%.6f class=%s "
                 "selection_reason=%s",
                 nominal_result.risk.valid ? nominal_result.risk.min_distance : -1.0,
                 nominal_result.risk.valid ? nominal_result.risk.min_distance : -1.0,
                 absolute_dynamic_threshold,
                 candidateSafetyClassName(nominal_result.safety_class),
                 selected_safety_reason.c_str());
      }
      if (active_candidate_metadata_valid_ && traj_.local_traj.duration > 1.0e-6)
      {
        ROS_INFO("[candidate-safety-class] type=PREVIOUS_%s "
                 "min_dynamic_distance=%.6f nominal_distance=%.6f "
                 "absolute_threshold=%.6f class=%s selection_reason=%s",
                 candidateKindName(active_candidate_kind_),
                 previous_min_dynamic_clearance,
                 nominal_result.risk.valid ? nominal_result.risk.min_distance : -1.0,
                 absolute_dynamic_threshold,
                 previous_safe_available ? "ABSOLUTE_SAFE" : "INVALID",
                 "TELEMETRY_ONLY_FINAL_DECISION_PENDING");
      }
    }

    for (CandidateResult *candidate :
         std::array<CandidateResult *, 3>{{&nominal_result, &plus_result,
                                           &minus_result}})
    {
      candidate->encirclement_hypothesis_id = encirclement_hypothesis_id;
      candidate->encirclement_phi0 = encirclement_phi0;
      candidate->encirclement_generation = encirclement_generation;
    }

    if (capture_output != nullptr)
    {
      *capture_output = CandidateSetOutput();
      capture_output->generated = true;
      capture_output->hypothesis_id = encirclement_hypothesis_id;
      capture_output->phi0 = encirclement_phi0;
      capture_output->encirclement_generation = encirclement_generation;
      capture_output->local_candidate_id = selected_result->candidate_id;
      capture_output->relative_tracking = relative_track_pt;
      capture_output->target_position = object_pt;
      capture_output->target_velocity = object_vel;
      capture_output->visibility_target_position = geometry_target_p_;
      capture_output->visibility_target_velocity = geometry_target_v_;
      capture_output->visibility_target_epoch = geometry_target_stamp_;
      capture_output->planning_epoch = t_start.toSec();
      capture_output->nominal_risk = nominal_result.risk;
      capture_output->nominal_conflict = nominal_result.conflict;
      capture_output->nominal_obstacle_id = nominal_result.risk.obstacle_id;
      for (const CandidateResult *candidate :
           std::array<const CandidateResult *, 3>{{&nominal_result, &plus_result,
                                                    &minus_result}})
        if (candidate->candidate_id >= 0 &&
            (candidate->min_jerk_opt.getTraj().getPieceNum() > 0 ||
             candidate->astar_used || candidate->local_sfc_handoff_valid))
          capture_output->candidates.push_back(*candidate);
      ROS_INFO("[encirclement-hypothesis-candidates] drone=%d generation=%lu "
               "hypothesis=%d phi0=%.6f candidate_count=%zu "
               "local_candidate_id=%d real_minco=1 deferred_commit=1",
               pp_.drone_id, encirclement_generation,
               encirclement_hypothesis_id, encirclement_phi0,
               capture_output->candidates.size(),
               capture_output->local_candidate_id);
      continous_failures_count_ = 0;
      return !capture_output->candidates.empty();
    }

    CandidateSetOutput local_set;
    local_set.nominal_risk=nominal_result.risk;
    local_set.nominal_conflict=nominal_result.conflict;
    local_set.nominal_obstacle_id=nominal_result.risk.obstacle_id;
    local_set.planning_epoch=t_start.toSec();local_set.local_candidate_id=selected_result->candidate_id;
    local_set.target_position=object_pt;local_set.target_velocity=object_vel;
    local_set.visibility_target_position=geometry_target_p_;
    local_set.visibility_target_velocity=geometry_target_v_;
    local_set.visibility_target_epoch=geometry_target_stamp_;
    local_set.candidates={nominal_result,plus_result,minus_result};
    std::vector<CandidateSetOutput> local_sets{std::move(local_set)};
    int selected_set=-1;
    const auto status=finalizeCapturedCandidates(local_sets,touch_goal,commit_head_freshness_threshold,selected_set);
    return status==TopologyProcessStatus::COMMITTED || status==TopologyProcessStatus::RETAINED_PREVIOUS;
  }

  void EGOPlannerManager::setFovSoftScale(const double scale)
  {
    ploy_traj_opt_->setFovSoftScale(scale);
  }

  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    if (traj_.local_traj.start_time < 1e9) // It means my first planning has not started
      return false;

    const double my_start = traj_.local_traj.start_time;
    const double other_start = traj_.swarm_traj[drone_id].start_time;
    const double t_start = std::max(my_start, other_start);
    const double t_end = std::min(my_start + traj_.local_traj.duration * 2.0 / 3.0,
                                   other_start + traj_.swarm_traj[drone_id].duration);
    for (double t = t_start; t < t_end; t += 0.03)
      if ((traj_.local_traj.traj.getPos(t - my_start) -
           traj_.swarm_traj[drone_id].traj.getPos(t - other_start)).norm() <
          getSwarmClearance()) return true;
    return false;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
      const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    poly_traj::MinJerkOpt globalMJO;
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << start_pos, start_vel, start_acc;
    tailState << waypoints.back(), end_vel, end_acc;
    Eigen::MatrixXd innerPts;

    if (waypoints.size() > 1)
    {

      innerPts.resize(3, waypoints.size() - 1);
      for (int i = 0; i < (int)waypoints.size() - 1; ++i)
      {
        innerPts.col(i) = waypoints[i];
      }
    }
    else
    {
      if (innerPts.size() != 0)
      {
        ROS_ERROR("innerPts.size() != 0");
      }
    }

    globalMJO.reset(headState, tailState, waypoints.size());

    double des_vel = pp_.max_vel_ / 1.5;
    Eigen::VectorXd time_vec(waypoints.size());

    for (int j = 0; j < 2; ++j)
    {
      for (size_t i = 0; i < waypoints.size(); ++i)
      {
        time_vec(i) = (i == 0) ? (waypoints[0] - start_pos).norm() / des_vel
                               : (waypoints[i] - waypoints[i - 1]).norm() / des_vel;
      }

      globalMJO.generate(innerPts, time_vec);

      if (globalMJO.getTraj().getMaxVelRate() < pp_.max_vel_ ||
          start_vel.norm() > pp_.max_vel_ ||
          end_vel.norm() > pp_.max_vel_)
      {
        break;
      }

      if (j == 2)
      {
        ROS_WARN("Global traj MaxVel = %f > set_max_vel", globalMJO.getTraj().getMaxVelRate());
        cout << "headState=" << endl
             << headState << endl;
        cout << "tailState=" << endl
             << tailState << endl;
      }

      des_vel /= 1.5;
    }

    auto time_now = ros::Time::now();
    traj_.setGlobalTraj(globalMJO.getTraj(), time_now.toSec());

    return true;
  }

} // namespace ego_planner
