#include <traj_utils/trajectory_lifecycle.h>
#include "optimizer/poly_traj_optimizer.h"

#include <multi_uav_formation/tracking_visibility_geometry.h>
#include <multi_uav_formation/encirclement_geometry.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <sstream>
#include <vector>
// using namespace std;

namespace ego_planner
{
  namespace
  {
    bool staticPlaneInterval(const LocalSfcPlane &plane,
                             const Eigen::VectorXd &durations,
                             double &begin, double &end)
    {
      if (plane.source != LocalSfcPlane::STATIC_COLLISION_CORRIDOR ||
          plane.piece_id < 0 || plane.piece_id >= durations.size() ||
          !durations.allFinite() ||
          (durations.array() <= 0.0).any() ||
          !std::isfinite(plane.piece_u_begin) ||
          !std::isfinite(plane.piece_u_end) ||
          plane.piece_u_begin < 0.0 ||
          plane.piece_u_end > 1.0 ||
          plane.piece_u_end <= plane.piece_u_begin)
        return false;
      const double piece_begin = durations.head(plane.piece_id).sum();
      begin = piece_begin + plane.piece_u_begin * durations(plane.piece_id);
      end = piece_begin + plane.piece_u_end * durations(plane.piece_id);
      return std::isfinite(begin) && std::isfinite(end) && end > begin;
    }

    bool directionalVisibilityCostEnabled(
        const bool tracking_configured,
        const bool guidance_active,
        const double weight)
    {
      return tracking_configured && guidance_active &&
             std::isfinite(weight) && weight > 0.0;
    }
  }

  bool PolyTrajOptimizer::forEachCandidateDynamicsSample(
      const poly_traj::Trajectory &traj,
      const CandidateDynamicsSampleVisitor &visitor) const
  {
    const int piece_count = traj.getPieceNum();
    const Eigen::VectorXd durations = traj.getDurations();
    if (piece_count <= 0 || durations.size() != piece_count ||
        !durations.allFinite() || (durations.array() <= 0.0).any() || !visitor)
      return false;

    const int samples_per_piece = std::max(1, cps_num_prePiece_);
    for (int piece = 0; piece < piece_count; ++piece)
    {
      const double duration = durations(piece);
      for (int sample = 0; sample <= samples_per_piece; ++sample)
      {
        if (piece > 0 && sample == 0)
          continue;
        const double alpha = static_cast<double>(sample) /
                             static_cast<double>(samples_per_piece);
        const double piece_time = duration * alpha;
        const Eigen::Vector3d vel = traj[piece].getVel(piece_time);
        const Eigen::Vector3d acc = traj[piece].getAcc(piece_time);
        const Eigen::Vector3d jerk = traj[piece].getJer(piece_time);
        if (!vel.allFinite() || !acc.allFinite() || !jerk.allFinite())
          return false;
        visitor(piece, sample, samples_per_piece, alpha, piece_time,
                vel, acc, jerk);
      }
      // Jerk peaks live between uniform samples.  Adding the exact stationary
      // points of |jerk|^2 keeps the QP rows, the SCP acceptance summaries and
      // the final hard kernel predicate on one and the same physical truth;
      // without them a payload could pass the solver lattice and still fail
      // the kernel by double-digit percent on the same quantity.
      for (const double alpha : traj[piece].getJerStationaryAlphas())
      {
        const double piece_time = duration * alpha;
        const Eigen::Vector3d vel = traj[piece].getVel(piece_time);
        const Eigen::Vector3d acc = traj[piece].getAcc(piece_time);
        const Eigen::Vector3d jerk = traj[piece].getJer(piece_time);
        if (!vel.allFinite() || !acc.allFinite() || !jerk.allFinite())
          return false;
        visitor(piece, -1, samples_per_piece, alpha, piece_time,
                vel, acc, jerk);
      }
    }
    return true;
  }

  bool PolyTrajOptimizer::evaluateCandidateDynamicsLattice(
      const poly_traj::Trajectory &traj,
      CandidateDynamicsSummary &summary) const
  {
    summary = CandidateDynamicsSummary();
    const bool valid = forEachCandidateDynamicsSample(
        traj,
        [&](const int, const int, const int, const double, const double,
            const Eigen::Vector3d &vel, const Eigen::Vector3d &acc,
            const Eigen::Vector3d &jerk) {
          ++summary.sample_count;
          summary.max_vel_value = std::max(summary.max_vel_value, vel.norm());
          summary.max_acc_value = std::max(summary.max_acc_value, acc.norm());
          summary.max_jerk_value = std::max(summary.max_jerk_value, jerk.norm());
          if (max_vel_ > 0.0 && std::isfinite(max_vel_))
          {
            ++summary.velocity_constraint_count;
            summary.max_vel_violation = std::max(
                summary.max_vel_violation, vel.norm() - max_vel_);
          }
          if (max_acc_ > 0.0 && std::isfinite(max_acc_))
          {
            ++summary.acceleration_constraint_count;
            summary.max_acc_violation = std::max(
                summary.max_acc_violation, acc.norm() - max_acc_);
          }
          if (max_jer_ > 0.0 && std::isfinite(max_jer_))
          {
            ++summary.jerk_constraint_count;
            summary.max_jerk_violation = std::max(
                summary.max_jerk_violation, jerk.norm() - max_jer_);
          }
        });
    summary.valid = valid && summary.sample_count > 0;
    summary.max_vel_violation = std::max(0.0, summary.max_vel_violation);
    summary.max_acc_violation = std::max(0.0, summary.max_acc_violation);
    summary.max_jerk_violation = std::max(0.0, summary.max_jerk_violation);
    return summary.valid;
  }

  double PolyTrajOptimizer::realTimeJacobianAt(const double virtual_t)
  {
    if (virtual_t > 0.0)
      return virtual_t + 1.0;
    const double denominator = (0.5 * virtual_t - 1.0) * virtual_t + 1.0;
    return (1.0 - virtual_t) / (denominator * denominator);
  }

  Eigen::RowVectorXd PolyTrajOptimizer::mincoSampleGradientWrtDecision(
      const int piece_index, const double piece_time,
      const Eigen::Vector3d &grad_sample, const int derivative_order,
      const Eigen::VectorXd &ds_dT,
      const Eigen::VectorXd &current_virtual_t)
  {
    const int position_dim = 3 * std::max(0, piece_num_ - 1);
    const int dim = position_dim + piece_num_;
    Eigen::RowVectorXd row = Eigen::RowVectorXd::Zero(dim);
    if (piece_num_ <= 0 || piece_index < 0 || piece_index >= piece_num_ ||
        !std::isfinite(piece_time) || !grad_sample.allFinite() ||
        ds_dT.size() != piece_num_ || !ds_dT.allFinite() ||
        current_virtual_t.size() != piece_num_ ||
        !current_virtual_t.allFinite())
      return row;

    const double s1 = piece_time;
    const double s2 = s1 * s1;
    const double s3 = s2 * s1;
    const double s4 = s2 * s2;
    const double s5 = s4 * s1;
    Eigen::Matrix<double, 6, 1> beta;
    switch (derivative_order)
    {
    case 0:
      beta << 1.0, s1, s2, s3, s4, s5;
      break;
    case 1:
      beta << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
      break;
    case 2:
      beta << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
      break;
    case 3:
      beta << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
      break;
    default:
      return row;
    }

    jerkOpt_.get_gdC().setZero();
    jerkOpt_.get_gdC().block<6, 3>(6 * piece_index, 0) =
        beta * grad_sample.transpose();
    Eigen::VectorXd gradT = Eigen::VectorXd::Zero(piece_num_);
    Eigen::MatrixXd gradP = Eigen::MatrixXd::Zero(3, piece_num_ - 1);
    Eigen::Matrix<double, 6, 1> next_beta;
    switch (derivative_order)
    {
    case 0:
      next_beta << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
      break;
    case 1:
      next_beta << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
      break;
    case 2:
      next_beta << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
      break;
    case 3:
      next_beta << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1;
      break;
    default:
      next_beta.setZero();
      break;
    }
    const Eigen::Vector3d explicit_time_derivative =
        jerkOpt_.get_b().block<6, 3>(6 * piece_index, 0).transpose() * next_beta;
    for (int i = 0; i < piece_num_; ++i)
      gradT(i) += grad_sample.dot(explicit_time_derivative) * ds_dT(i);
    jerkOpt_.getGrad2TP(gradT, gradP);
    if (gradP.rows() == 3 && gradP.cols() == piece_num_ - 1 &&
        gradP.allFinite())
    {
      for (int index = 0; index < position_dim; ++index)
        row(index) = gradP.data()[index];
    }
    if (gradT.allFinite())
    {
      for (int i = 0; i < piece_num_; ++i)
        row(position_dim + i) =
            gradT(i) * realTimeJacobianAt(current_virtual_t(i));
    }
    return row;
  }

  double PolyTrajOptimizer::evaluateCandidateLocalSfcMaxViolation(
      const poly_traj::Trajectory &traj) const
  {
    if (traj.getPieceNum() <= 0 || !std::isfinite(traj.getTotalDuration()))
      return std::numeric_limits<double>::infinity();
    double max_violation = 0.0;
    constexpr int samples = 25;
    for (const LocalSfcPlane &plane : candidate_local_sfc_planes_)
    {
      // Feedback117 (authority consolidation): only STATIC_COLLISION_CORRIDOR
      // planes are spatial-safety geometry.  LOS_OBSERVATION_SIDE planes are
      // the visibility authority and must not gate the initializer or the
      // verified-fallback contract through this violation metric; otherwise
      // an unfinished-occlusion detour (slack > 0) could still be rejected
      // here as a third hidden LOS reject path.
      if (plane.source == LocalSfcPlane::LOS_OBSERVATION_SIDE)
        continue;
      const double normal_norm = plane.normal.norm();
      double active_start = 0.0, active_end = 0.0;
      if (!plane.normal.allFinite() || !plane.point.allFinite() ||
          !std::isfinite(plane.clearance) || normal_norm < 1.0e-6 ||
          !staticPlaneInterval(plane, traj.getDurations(),
                               active_start, active_end))
        return std::numeric_limits<double>::infinity();
      const Eigen::Vector3d normal = plane.normal / normal_norm;
      for (int sample = 0; sample < samples; ++sample)
      {
        const double alpha = static_cast<double>(sample) /
                             static_cast<double>(samples - 1);
        const double t = active_start + alpha * (active_end - active_start);
        const double margin = normal.dot(traj.getPos(t) - plane.point) -
                              plane.clearance;
        max_violation = std::max(max_violation, -margin);
      }
    }
    return std::max(0.0, max_violation);
  }

  std::vector<double> PolyTrajOptimizer::quarticRealRoots(
      const Eigen::Matrix<double, 5, 1> &c, const double lo,
      const double hi)
  {
    std::vector<double> roots;
    if (!c.allFinite() || !std::isfinite(lo) || !std::isfinite(hi) ||
        !(lo < hi))
      return roots;
    // Isolate real roots by monotone intervals. Critical points are recursively
    // included, so even-multiplicity roots are retained without a complex
    // imaginary-part heuristic. Coefficients are ascending in x.
    const std::vector<double> coefficients(c.data(), c.data() + 5);
    std::function<std::vector<double>(std::vector<double>)> isolate =
        [&](std::vector<double> a) -> std::vector<double> {
      double scale = 0.0;
      for (const double v : a)
        scale = std::max(scale, std::abs(v));
      if (!(scale > 0.0))
        return {};
      for (double &v : a)
        v /= scale;
      while (a.size() > 1 && std::abs(a.back()) <= 1.0e-14)
        a.pop_back();
      if (a.size() == 1)
        return {};
      const auto eval = [&](const double x) {
        double value = 0.0;
        for (auto it = a.rbegin(); it != a.rend(); ++it)
          value = value * x + *it;
        return value;
      };
      std::vector<double> critical;
      if (a.size() > 2)
      {
        std::vector<double> derivative(a.size() - 1);
        for (size_t k = 1; k < a.size(); ++k)
          derivative[k - 1] = static_cast<double>(k) * a[k];
        critical = isolate(derivative);
      }
      std::vector<double> cuts{lo};
      for (const double x : critical)
        if (x > lo && x < hi)
          cuts.push_back(x);
      cuts.push_back(hi);
      std::sort(cuts.begin(), cuts.end());
      std::vector<double> found;
      const auto add = [&](const double x) {
        const double residual = std::abs(eval(x));
        double magnitude = 1.0;
        double power = 1.0;
        for (const double v : a)
        {
          magnitude += std::abs(v) * power;
          power *= std::max(1.0, std::abs(x));
        }
        if (residual <= 1.0e-9 * magnitude &&
            (found.empty() || std::abs(found.back() - x) > 1.0e-8))
          found.push_back(x);
      };
      for (size_t i = 0; i < cuts.size(); ++i)
      {
        add(cuts[i]);
        if (i + 1 == cuts.size())
          break;
        double left = cuts[i], right = cuts[i + 1];
        double fleft = eval(left), fright = eval(right);
        if (fleft * fright >= 0.0)
          continue;
        for (int iteration = 0; iteration < 80; ++iteration)
        {
          const double mid = 0.5 * (left + right);
          const double fm = eval(mid);
          if (fleft * fm <= 0.0)
          {
            right = mid;
            fright = fm;
          }
          else
          {
            left = mid;
            fleft = fm;
          }
          if (right - left <= 1.0e-12 * std::max(1.0, std::abs(mid)))
            break;
        }
        add(0.5 * (left + right));
      }
      std::sort(found.begin(), found.end());
      return found;
    };
    roots = isolate(coefficients);
    return roots;
  }

  double PolyTrajOptimizer::continuousLocalSfcMaxViolation(
      const poly_traj::Trajectory &traj,
      std::vector<CorridorViolationPoint> *worst_per_plane) const
  {
    return continuousLocalSfcMaxViolation(
        traj, candidate_local_sfc_planes_, worst_per_plane);
  }

  double PolyTrajOptimizer::continuousLocalSfcMaxViolation(
      const poly_traj::Trajectory &traj,
      const std::vector<LocalSfcPlane> &planes,
      std::vector<CorridorViolationPoint> *worst_per_plane) const
  {
    if (traj.getPieceNum() <= 0 || !std::isfinite(traj.getTotalDuration()) ||
        traj.getTotalDuration() <= 0.0)
      return std::numeric_limits<double>::infinity();
    if (worst_per_plane != nullptr)
      worst_per_plane->clear();
    const int piece_num = traj.getPieceNum();
    const Eigen::VectorXd durations = traj.getDurations();
    double max_violation = 0.0;
    for (size_t plane_index = 0; plane_index < planes.size();
         ++plane_index)
    {
      const LocalSfcPlane &plane = planes[plane_index];
      // Feedback119: LOS planes never enter any corridor metric; only the
      // STATIC_COLLISION_CORRIDOR geometry owns this measurement.
      if (plane.source == LocalSfcPlane::LOS_OBSERVATION_SIDE)
        continue;
      const double normal_norm = plane.normal.norm();
      double active_start = 0.0, active_end = 0.0;
      if (!plane.normal.allFinite() || !plane.point.allFinite() ||
          !std::isfinite(plane.clearance) || normal_norm < 1.0e-6 ||
          !staticPlaneInterval(plane, durations,
                               active_start, active_end))
        return std::numeric_limits<double>::infinity();
      const Eigen::Vector3d normal = plane.normal / normal_norm;
      const double plane_begin = active_start;
      const double plane_end = active_end;
      if (plane_end <= plane_begin)
        continue;
      double plane_worst = 0.0;
      double plane_worst_time = plane_begin;
      double t_piece_start = 0.0;
      for (int piece = 0; piece < piece_num; ++piece)
      {
        const double piece_duration = durations(piece);
        const double t_piece_end = t_piece_start + piece_duration;
        const double lo = std::max(plane_begin, t_piece_start);
        const double hi = std::min(plane_end, t_piece_end);
        if (hi <= lo + 1.0e-12)
        {
          t_piece_start = t_piece_end;
          continue;
        }
        // f(s) = sum_k c_k s^k with s in piece-local time; c_k = n·b_k,
        // c_0 additionally carries -n·point-clearance.
        // getCoeffMat() is DESCENDING (col(5) = constant term), so the
        // ascending coefficient of t^k lives in col(5-k).
        const auto coeff_mat = traj[piece].getCoeffMat();
        Eigen::Matrix<double, 6, 1> c;
        for (int k = 0; k < 6; ++k)
          c(k) = normal.dot(coeff_mat.col(5 - k));
        c(0) -= normal.dot(plane.point) + plane.clearance;
        auto evaluate = [&](const double s) {
          double value = 0.0;
          for (int k = 5; k >= 0; --k)
            value = value * s + c(k);
          return value;
        };
        // Candidate extrema: interval endpoints + real roots of f'(s)=0.
        std::vector<double> candidates;
        candidates.push_back(lo - t_piece_start);
        candidates.push_back(hi - t_piece_start);
        const double interval_lo = lo - t_piece_start;
        const double interval_hi = hi - t_piece_start;
        // Normalize the quartic to u in [0,1] over this clipped interval.
        // This avoids powers of a very short physical piece duration.
        Eigen::Matrix<double, 5, 1> derivative;
        double time_power = 1.0;
        for (int k = 0; k < 5; ++k)
        {
          derivative(k) = (k + 1) * c(k + 1) * time_power;
          time_power *= piece_duration;
        }
        for (const double u : quarticRealRoots(
                 derivative, interval_lo / piece_duration,
                 interval_hi / piece_duration))
        {
          const double root = u * piece_duration;
          if (root > interval_lo + 1.0e-9 &&
              root < interval_hi - 1.0e-9)
            candidates.push_back(root);
        }
        t_piece_start = t_piece_end;
        for (const double s : candidates)
        {
          // Violation is negative margin: f < 0 means outside the corridor.
          const double violation = -evaluate(s);
          if (violation > plane_worst)
          {
            plane_worst = violation;
            plane_worst_time = t_piece_start + s;
          }
        }
      }
      if (worst_per_plane != nullptr && plane_worst > 0.0)
        worst_per_plane->push_back(
            {plane_index, plane_worst_time, plane_worst});
      max_violation = std::max(max_violation, plane_worst);
    }
    return max_violation;
  }

  SCPQPSolveResult PolyTrajOptimizer::solveExecutionQPWithSlack(
      const Eigen::VectorXd &g, const Eigen::MatrixXd &A,
      const Eigen::VectorXd &lo, const Eigen::VectorXd &hi,
      const double hessian_diagonal, const int soft_rows,
      const double slack_w1, const double slack_w2,
      double &slack_max, double &slack_sum) const
  {
    slack_max = 0.0;
    slack_sum = 0.0;
    if (!std::isfinite(hessian_diagonal) || hessian_diagonal <= 0.0 ||
        A.rows() == 0 || soft_rows <= 0 || (slack_w1 <= 0.0 && slack_w2 <= 0.0))
      return solveExecutionQP(g, A, lo, hi, hessian_diagonal);
    const int n = static_cast<int>(g.size());
    const int m = static_cast<int>(A.rows());
    if (n <= 0 || soft_rows > m)
      return solveExecutionQP(g, A, lo, hi, hessian_diagonal);
    // 扩展变量:x' = [x; ξ_0..ξ_{soft_rows-1}]。前 soft_rows 行的 LOS 不等式
    // 变为 a·x + ξ >= lo  →  a·x + ξ >= lo(ξ 下界 0);其余行不变。
    const int n2 = n + soft_rows;
    const int m2 = m + soft_rows;
    Eigen::MatrixXd A2 = Eigen::MatrixXd::Zero(m2, n2);
    A2.topLeftCorner(m, n) = A;
    Eigen::VectorXd lo2 = Eigen::VectorXd::Constant(m2, -1.0e20);
    Eigen::VectorXd hi2 = Eigen::VectorXd::Constant(m2, 1.0e20);
    for (int row = 0; row < m; ++row)
    {
      lo2(row) = lo(row);
      hi2(row) = hi(row);
    }
    for (int k = 0; k < soft_rows; ++k)
    {
      A2(k, n + k) = 1.0;                       // a·x + ξ_k >= lo_k
      lo2(m + k) = 0.0;                          // ξ_k >= 0
      hi2(m + k) = 1.0e20;
    }
    Eigen::VectorXd g2 = Eigen::VectorXd::Zero(n2);
    g2.head(n) = g;
    // J_slack = w1·Σξ_k + 0.5·w2·Σξ_k²:线性项进 q,二次项进对角 Hessian。
    for (int k = 0; k < soft_rows; ++k)
    {
      g2(n + k) = slack_w1;
      A2(m + k, n + k) += slack_w2;              // ξ_k >= 0 行上不动;二次项在 P
    }
    Eigen::VectorXd h2 = Eigen::VectorXd::Constant(n2, hessian_diagonal);
    for (int k = 0; k < soft_rows; ++k)
      h2(n + k) = slack_w2 > 0.0 ? slack_w2 : hessian_diagonal;
    // 无 slack 行的 Hessian 与原问题一致;slack 变量自带 w2 曲率。
    const SCPQPSolveResult qp2 = solveExecutionQP(g2, A2, lo2, hi2, h2);
    if (qp2.success && qp2.step.size() == n2)
    {
      for (int k = 0; k < soft_rows; ++k)
      {
        const double xi = std::max(0.0, qp2.step(n + k));
        slack_sum += xi;
        slack_max = std::max(slack_max, xi);
      }
    }
    SCPQPSolveResult result = qp2;
    if (qp2.success && qp2.step.size() == n2 && qp2.step.allFinite())
      result.step = qp2.step.head(n);
    else
    {
      result.success = false;
      result.step = Eigen::VectorXd::Zero(n);
    }
    return result;
  }

  bool PolyTrajOptimizer::enforceCandidateLosPlanesSCP(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      Eigen::MatrixXd &optimal_points)
  {
    std::vector<LocalSfcPlane> los_planes;
    for (const LocalSfcPlane &plane : candidate_local_sfc_planes_)
      if (plane.source == LocalSfcPlane::LOS_OBSERVATION_SIDE)
        los_planes.push_back(plane);
    if (los_planes.empty())
    {
      last_los_soft_plane_telemetry_ = LosSoftPlaneTelemetry();
      return true;
    }

    last_los_soft_plane_telemetry_ = LosSoftPlaneTelemetry();
    last_los_soft_plane_telemetry_.attempted = true;

    const poly_traj::Trajectory solved = jerkOpt_.getTraj();
    const int piece_num = solved.getPieceNum();
    auto los_max_violation = [&](const poly_traj::Trajectory &traj) {
      double result = 0.0;
      constexpr int samples = 25;
      for (const LocalSfcPlane &plane : los_planes)
      {
        const Eigen::Vector3d normal = plane.normal.normalized();
        const double start = std::max(0.0, plane.active_start);
        const double end = std::min(traj.getTotalDuration(), plane.active_end);
        if (!normal.allFinite() || !(end > start))
          return std::numeric_limits<double>::infinity();
        for (int sample = 0; sample < samples; ++sample)
        {
          const double alpha = static_cast<double>(sample) /
                               static_cast<double>(samples - 1);
          const double t = start + alpha * (end - start);
          result = std::max(result, plane.clearance -
                            normal.dot(traj.getPos(t) - plane.point));
        }
      }
      return std::max(0.0, result);
    };
    if (piece_num <= 1)
    {
      // 单段无内部控制点:LOS 无法被该 QP 改写。Feedback117:该平面属于
      // visibility authority,任何残余 violation 都只进遥测,绝不由此
      // 判死候选(最终硬安全由 current-revision preflight 独占)。
      const double violation = los_max_violation(solved);
      last_los_soft_plane_telemetry_.satisfied = violation <= 1.0e-3;
      last_los_soft_plane_telemetry_.qp_success = true;
      last_los_soft_plane_telemetry_.qp_status = last_los_soft_plane_telemetry_.satisfied
          ? "SINGLE_PIECE_SATISFIED"
          : "SINGLE_PIECE_RESIDUAL_TELEMETRY_ONLY";
      return true;
    }

    const Eigen::VectorXd durations = solved.getDurations();
    Eigen::MatrixXd inner_points =
        solved.getPositions().block(0, 1, 3, piece_num - 1);
    const int position_dim = 3 * (piece_num - 1);
    const Eigen::VectorXd fixed_virtual_t = [&]() {
      Eigen::VectorXd value(piece_num);
      RealT2VirtualT(durations, value);
      return value;
    }();
    constexpr int kSamplesPerPlane = 25;
    constexpr int kMaxIterations = 6;

    auto locate_sample = [&](const double global_t, int &piece,
                             double &piece_time) {
      double accumulated = 0.0;
      piece = piece_num - 1;
      piece_time = durations(piece);
      for (int i = 0; i < piece_num; ++i)
      {
        if (global_t <= accumulated + durations(i) || i == piece_num - 1)
        {
          piece = i;
          piece_time = std::max(0.0, std::min(durations(i),
                                               global_t - accumulated));
          return;
        }
        accumulated += durations(i);
      }
    };

    for (int iteration = 0; iteration < kMaxIterations; ++iteration)
    {
      executionCheckpoint();
      jerkOpt_.reset(iniState, finState, piece_num);
      jerkOpt_.generate(inner_points, durations);
      const poly_traj::Trajectory current = jerkOpt_.getTraj();
      std::vector<Eigen::RowVectorXd> rows;
      std::vector<double> lower_bounds;
      double max_violation = 0.0;

      for (const LocalSfcPlane &plane : los_planes)
      {
        const double normal_norm = plane.normal.norm();
        if (!plane.normal.allFinite() || !plane.point.allFinite() ||
            !std::isfinite(plane.clearance) || normal_norm < 1.0e-6)
        {
          // Feedback117:坏平面几何是 provenance 数据错误,不是安全事实。
          // visibility authority 无 reject 权:保留当前解,仅记录遥测。
          last_los_soft_plane_telemetry_.qp_status =
              "INVALID_PLANE_GEOMETRY_TELEMETRY_ONLY";
          optimal_points = jerkOpt_.getInitConstraintPoints(cps_num_prePiece_);
          return true;
        }
        const Eigen::Vector3d normal = plane.normal / normal_norm;
        const double start = std::max(0.0, plane.active_start);
        const double end = std::min(current.getTotalDuration(), plane.active_end);
        if (!(end > start))
        {
          // 空时间窗同样不能杀候选;静态部分仍由 collision 行与 final
          // preflight 全权负责。
          last_los_soft_plane_telemetry_.qp_status =
              "EMPTY_ACTIVE_WINDOW_TELEMETRY_ONLY";
          optimal_points = jerkOpt_.getInitConstraintPoints(cps_num_prePiece_);
          return true;
        }
        for (int sample = 0; sample < kSamplesPerPlane; ++sample)
        {
          const double alpha = static_cast<double>(sample) /
                               static_cast<double>(kSamplesPerPlane - 1);
          const double t = start + alpha * (end - start);
          const double margin =
              normal.dot(current.getPos(t) - plane.point) - plane.clearance;
          int piece = 0;
          double piece_time = 0.0;
          locate_sample(t, piece, piece_time);
          const Eigen::RowVectorXd full_row = mincoSampleGradientWrtDecision(
              piece, piece_time, normal, 0,
              Eigen::VectorXd::Zero(piece_num), fixed_virtual_t);
          const Eigen::RowVectorXd controllable_row =
              full_row.head(position_dim);
          // Start/end PVA are immutable.  A sample whose position has no
          // derivative with respect to an interior waypoint cannot be moved
          // by this QP and must not turn the entire LOS-side problem
          // artificially infeasible.
          if (controllable_row.norm() > 1.0e-8)
          {
            max_violation = std::max(max_violation, -margin);
            rows.push_back(controllable_row);
            lower_bounds.push_back(-margin);
          }
        }
      }

      if (max_violation <= 1.0e-3)
      {
        optimal_points = jerkOpt_.getInitConstraintPoints(cps_num_prePiece_);
        last_los_soft_plane_telemetry_.satisfied = true;
        last_los_soft_plane_telemetry_.qp_success = true;
        last_los_soft_plane_telemetry_.iteration_count = iteration + 1;
        last_los_soft_plane_telemetry_.sample_count =
            static_cast<int>(rows.size());
        last_los_soft_plane_telemetry_.qp_status = "SATISFIED";
        ROS_INFO("[los-plane-scp] candidate=%s iteration=%d rows=%zu "
                 "max_violation=%.6f result=SATISFIED",
                 gradient_audit_candidate_.c_str(), iteration, rows.size(),
                 std::max(0.0, max_violation));
        return true;
      }

      const int constraint_rows = static_cast<int>(rows.size());
      if (constraint_rows == 0)
      {
        // Feedback117:没有任何可控行意味着 LOS 在该决策子空间内无法改善,
        // 不是候选的失败原因。保持当前解并继续。
        last_los_soft_plane_telemetry_.qp_status = "NO_CONTROLLABLE_ROWS_TELEMETRY_ONLY";
        optimal_points = jerkOpt_.getInitConstraintPoints(cps_num_prePiece_);
        return true;
      }
      const double trust_region =
          std::min(1.5, std::max(0.25, max_violation + 0.10));
      Eigen::MatrixXd A(constraint_rows + position_dim, position_dim);
      Eigen::VectorXd lower(constraint_rows + position_dim);
      Eigen::VectorXd upper(constraint_rows + position_dim);
      for (int row = 0; row < constraint_rows; ++row)
      {
        A.row(row) = rows[row];
        lower(row) = lower_bounds[row];
        upper(row) = 1.0e20;
      }
      for (int i = 0; i < position_dim; ++i)
      {
        A.row(constraint_rows + i).setZero();
        A(constraint_rows + i, i) = 1.0;
        lower(constraint_rows + i) = -trust_region;
        upper(constraint_rows + i) = trust_region;
      }
      // Feedback116:LOS observation 行改为 semi-hard——允许非负 slack ξ_k,
      // 代价 w1·Σξ + 0.5·w2·Σξ²。ξ=0 时与原 hard QP 完全一致;QP 不再因
      // "当前仍在阴影内、固定端点+信任域内无法完全到观察侧"而
      // primal_infeasible。硬安全(碰撞/SFC/动力学/锁端点盒)不软化。
      double slack_max = 0.0, slack_sum = 0.0;
      const double slack_w2 = los_observation_slack_weight_;
      const double slack_w1 = 0.5 * slack_w2 * 1.0e-3;  // 微小 L1 项,推动压零
      const SCPQPSolveResult qp = solveExecutionQPWithSlack(
          Eigen::VectorXd::Zero(position_dim), A, lower, upper,
          1.0, /*soft_rows=*/constraint_rows, slack_w1, slack_w2,
          slack_max, slack_sum);
      ++last_los_soft_plane_telemetry_.iteration_count;
      last_los_soft_plane_telemetry_.sample_count += constraint_rows;
      last_los_soft_plane_telemetry_.qp_status = qp.status_text;
      ROS_INFO("[los-plane-scp] candidate=%s iteration=%d rows=%d "
               "max_violation=%.6f trust_region=%.6f qp_status=%s "
               "slack_max=%.6f slack_sum=%.6f mode=SEMI_HARD_LOS",
               gradient_audit_candidate_.c_str(), iteration, constraint_rows,
               std::max(0.0, max_violation), trust_region,
               qp.status_text.c_str(), slack_max, slack_sum);
      if (!qp.success || qp.step.size() != position_dim ||
          !qp.step.allFinite())
      {
        // Feedback117:LOS semi-hard QP 自身求解失败(数值/盒子矛盾)也不再
        // 判死候选。LOS 属 visibility authority:保留上一个已通过 LBFGS
        // 细查的迭代点,残余 deviation 只进遥测;真正的执行安全仍由
        // collision/SFC/动力学硬检查与 current-revision preflight 独占。
        last_los_soft_plane_telemetry_.qp_status = "HARD_ROWS_INFEASIBLE:" +
                                                   qp.status_text;
        jerkOpt_.reset(iniState, finState, piece_num);
        jerkOpt_.generate(inner_points, durations);
        optimal_points = jerkOpt_.getInitConstraintPoints(cps_num_prePiece_);
        ROS_INFO("[los-plane-scp] candidate=%s result=SOFT_KEEP_LAST_ITERATE "
                 "qp_status=%s action=VISIBILITY_AUTHORITY_NO_REJECT",
                 gradient_audit_candidate_.c_str(),
                 last_los_soft_plane_telemetry_.qp_status.c_str());
        return true;
      }
      last_los_soft_plane_telemetry_.slack_nonzero_count +=
          slack_max > 1.0e-6 ? 1 : 0;
      last_los_soft_plane_telemetry_.slack_max =
          std::max(last_los_soft_plane_telemetry_.slack_max, slack_max);
      last_los_soft_plane_telemetry_.slack_mean =
          (last_los_soft_plane_telemetry_.slack_mean *
               (last_los_soft_plane_telemetry_.iteration_count - 1) +
           slack_sum / std::max(1, constraint_rows)) /
          last_los_soft_plane_telemetry_.iteration_count;
      last_los_soft_plane_telemetry_.slack_cost +=
          slack_w1 * slack_sum + 0.5 * slack_w2 * slack_max * slack_max;
      Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic>> delta(
          qp.step.data(), 3, piece_num - 1);
      inner_points += delta;
    }

    jerkOpt_.reset(iniState, finState, piece_num);
    jerkOpt_.generate(inner_points, durations);
    optimal_points = jerkOpt_.getInitConstraintPoints(cps_num_prePiece_);
    const double violation = los_max_violation(jerkOpt_.getTraj());
    last_los_soft_plane_telemetry_.qp_success = true;
    // Feedback116:LOS plane violation(即使 > 0)不再是 candidate reject
    // 条件——"安全、方向正确、本周期尚未完全绕出遮挡"的轨迹必须活到
    // C3/D3 比较层。剩余 violation 只进遥测,由 MINCO 后续硬检查与
    // current-revision preflight 保留真正的执行安全。
    ROS_INFO("[los-plane-scp] candidate=%s result=SOFT_ACCEPTED "
             "final_violation=%.6f slack_max=%.6f slack_mean=%.6f "
             "slack_cost=%.3f qp_status=%s action=PROGRESS_CANDIDATE",
             gradient_audit_candidate_.c_str(), violation,
             last_los_soft_plane_telemetry_.slack_max,
             last_los_soft_plane_telemetry_.slack_mean,
             last_los_soft_plane_telemetry_.slack_cost,
             last_los_soft_plane_telemetry_.qp_status.c_str());
    return true;
  }

  void PolyTrajOptimizer::setTeamVisibilityReserve(
      const TeamVisibilityReserve &reserve)
  {
    team_visibility_reserve_ = reserve;
    team_reserve_guides_.clear();
  }

  void PolyTrajOptimizer::clearTeamVisibilityReserve(void)
  {
    team_visibility_reserve_.active = false;
    team_reserve_guides_.clear();
    team_reserve_cost_ = 0.0;
    team_reserve_active_samples_ = 0;
  }

  void PolyTrajOptimizer::setTeamVisibilityReserveActivation(
      const double activation_world_time)
  {
    if (!team_visibility_reserve_.active ||
        !std::isfinite(activation_world_time) ||
        !std::isfinite(team_visibility_reserve_.activation_world_time))
    {
      team_visibility_reserve_.activation_world_time = activation_world_time;
      return;
    }
    // Re-anchor the absolute world-time reserve window to the new
    // trajectory start so the relative timing of the contracted crossing is
    // preserved (Feedback098/101 semantics).
    const double shift = activation_world_time -
                         team_visibility_reserve_.activation_world_time;
    if (std::isfinite(shift) && std::abs(shift) > 1.0e-9)
    {
      team_visibility_reserve_.reserve_begin_world_time += shift;
      team_visibility_reserve_.reserve_end_world_time += shift;
    }
    team_visibility_reserve_.activation_world_time = activation_world_time;
  }

  void PolyTrajOptimizer::prepareTeamVisibilityReserveGuides(
      const poly_traj::Trajectory &seed)
  {
    team_reserve_guides_.clear();
    if (!team_visibility_reserve_.active || seed.getPieceNum() <= 0)
      return;
    const double begin = team_visibility_reserve_.reserve_begin_world_time;
    const double end = team_visibility_reserve_.reserve_end_world_time;
    if (!std::isfinite(begin) || !std::isfinite(end) || end <= begin)
      return;
    // One to three fixed guide samples on the contracted crossing window
    // (Feedback101: "one to three guide samples on the first crossing
    // interval").  Positions come from the Local seed; no visibility or LOS
    // query happens on this cost path.
    const double window = end - begin;
    const int sample_count = std::max(1, std::min(3,
        static_cast<int>(std::ceil(window / 0.10))));
    for (int k = 0; k < sample_count; ++k)
    {
      const double alpha = sample_count > 1
          ? static_cast<double>(k) / static_cast<double>(sample_count - 1)
          : 0.5;
      const double world_time = begin + alpha * window;
      const double local_time = world_time -
                                team_visibility_reserve_.activation_world_time;
      if (!std::isfinite(local_time) || local_time < 0.0 ||
          local_time > seed.getTotalDuration())
        continue;
      const Eigen::Vector3d position = seed.getPos(local_time);
      if (!position.allFinite())
        continue;
      TeamReserveGuide guide;
      guide.world_time = world_time;
      guide.position = position;
      team_reserve_guides_.push_back(guide);
    }
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::addTeamVisibilityReserveGradCost2CT(
      EIGENVEC &gdT, double &reserve_cost)
  {
    reserve_cost = 0.0;
    if (!team_visibility_reserve_.active || team_reserve_guides_.empty() ||
        piece_num_ <= 0)
    {
      team_reserve_active_samples_ = 0;
      return;
    }
    const double weight = team_visibility_reserve_.weight;
    if (!(weight > 0.0))
    {
      team_reserve_active_samples_ = 0;
      return;
    }
    const Eigen::VectorXd durations = jerkOpt_.get_T1();
    int active_samples = 0;
    for (const TeamReserveGuide &guide : team_reserve_guides_)
    {
      // Locate the trajectory-local time of this fixed world-time guide.
      const double local_time = guide.world_time -
                                team_visibility_reserve_.activation_world_time;
      if (!std::isfinite(local_time) || local_time < 0.0)
        continue;
      double accumulated = 0.0;
      int piece = piece_num_ - 1;
      double piece_time = durations(piece);
      for (int i = 0; i < piece_num_; ++i)
      {
        if (local_time <= accumulated + durations(i) || i == piece_num_ - 1)
        {
          piece = i;
          piece_time = std::max(0.0, std::min(durations(i),
                                              local_time - accumulated));
          break;
        }
        accumulated += durations(i);
      }
      const double duration = durations(piece);
      if (!(duration > 1.0e-9))
        continue;
      const double local_time2 = piece_time * piece_time;
      const double local_time3 = local_time2 * piece_time;
      const double local_time4 = local_time2 * local_time2;
      const double local_time5 = local_time4 * piece_time;
      Eigen::Matrix<double, 6, 1> beta0;
      Eigen::Matrix<double, 6, 1> beta1;
      beta0 << 1.0, piece_time, local_time2, local_time3, local_time4,
          local_time5;
      beta1 << 0.0, 1.0, 2.0 * piece_time, 3.0 * local_time2,
          4.0 * local_time3, 5.0 * local_time4;
      const Eigen::Matrix<double, 6, 3> &coefficients =
          jerkOpt_.get_b().block<6, 3>(piece * 6, 0);
      const Eigen::Vector3d position = coefficients.transpose() * beta0;
      const Eigen::Vector3d velocity = coefficients.transpose() * beta1;
      const Eigen::Vector3d error = position - guide.position;
      const double sample_cost = 0.5 * weight * error.squaredNorm();
      const Eigen::Vector3d sample_grad_position = weight * error;
      const double sample_grad_time = weight * error.dot(velocity);
      jerkOpt_.get_gdC().block<6, 3>(piece * 6, 0) +=
          beta0 * sample_grad_position.transpose();
      // Time gradient: the guide is fixed in world time; stretching the
      // piece (alpha fraction) or any earlier piece (1.0) moves the sample
      // along the same polynomial.
      gdT(piece) += (piece_time / duration) * sample_grad_time;
      if (piece > 0)
        gdT.head(piece).array() += sample_grad_time;
      reserve_cost += sample_cost;
      ++active_samples;
    }
    team_reserve_active_samples_ = active_samples;
    team_reserve_cost_ = reserve_cost;
  }

  template void PolyTrajOptimizer::addTeamVisibilityReserveGradCost2CT<
      Eigen::VectorXd>(Eigen::VectorXd &gdT, double &reserve_cost);

  void PolyTrajOptimizer::setTeamVisibilityContract(
      const TeamVisibilityContract &contract)
  {
    team_visibility_contract_ = contract;
    team_visibility_contract_.active = contract.active &&
        contract.contract_id != 0 &&
        std::isfinite(contract.activation_world_time) &&
        std::isfinite(contract.acquire_by_world_time) &&
        std::isfinite(contract.preserve_until_world_time) &&
        std::isfinite(contract.contract_expire_world_time) &&
        std::isfinite(contract.required_margin) &&
        contract.preserve_until_world_time >
            contract.acquire_by_world_time + 1.0e-6 &&
        contract.contract_expire_world_time >
            contract.activation_world_time;
  }

  void PolyTrajOptimizer::clearTeamVisibilityContract(void)
  {
    team_visibility_contract_ = TeamVisibilityContract();
    team_contract_scp_active_ = false;
  }

  namespace
  {
    const char *visibilityComponentName(const int index)
    {
      static const char *kNames[4] = {"STATIC", "DYNAMIC_LOS", "FOV", "RANGE"};
      if (index < 0 || index > 3)
        return "NONE";
      return kNames[index];
    }
  }

  multi_uav_formation::ContinuousVisibilitySample
  PolyTrajOptimizer::visibilitySampleAt(
      const poly_traj::Trajectory &trajectory, const double trajectory_time,
      const double activation_world_time) const
  {
      multi_uav_formation::ContinuousVisibilitySample sample;
      if (trajectory.getPieceNum() <= 0 ||
          !std::isfinite(trajectory_time) || trajectory_time < -1.0e-9 ||
          trajectory_time > trajectory.getTotalDuration() + 1.0e-9)
        return sample;
      const double t = std::max(
          0.0, std::min(trajectory.getTotalDuration(), trajectory_time));
      const Eigen::Vector3d observer = trajectory.getPos(t);
      const Eigen::Vector3d target = object_p_ + object_v_ * t;
      if (!observer.allFinite() || !target.allFinite())
        return sample;
      const Eigen::Vector3d delta = observer - target;
      const double range = delta.norm();
      if (!std::isfinite(range) || range <= 1.0e-9)
        return sample;

      std::array<multi_uav_formation::VisibilityRiskComponent, 4> components;
      components[0].limiter = multi_uav_formation::ContinuousVisibilitySample::LIMITER_STATIC;
      components[1].limiter = multi_uav_formation::ContinuousVisibilitySample::LIMITER_DYNAMIC_LOS;
      components[2].limiter = multi_uav_formation::ContinuousVisibilitySample::LIMITER_FOV;
      components[3].limiter = multi_uav_formation::ContinuousVisibilitySample::LIMITER_RANGE;

      bool binary_valid = true;
      bool binary_los_clear = true;
      double clearance = std::numeric_limits<double>::infinity();
      Eigen::Vector3d clearance_gradient = Eigen::Vector3d::Zero();
      if (static_los_geometry_.querySegmentClearance(
              observer, target, clearance, &clearance_gradient, nullptr,
              nullptr))
      {
        const auto risk = multi_uav_formation::directionalClearanceRisk(
            clearance, static_los_margin_,
            static_los_margin_ + team_visibility_los_smoothing_);
        if (risk.valid)
        {
          components[0].valid = true;
          components[0].risk = risk.value;
          components[0].gradient_position =
              risk.derivative_clearance * clearance_gradient;
          binary_los_clear = clearance >= static_los_margin_;
        }
      }
      else
        binary_valid = false;

      components[1].valid = true;
      const double world_time =
          activation_world_time + t;
      if (moving_objs_)
      {
        for (int id = 0; id < moving_objs_->getObjNums(); ++id)
        {
          if (!moving_objs_->hasPrediction(id))
          {
            components[1].valid = false;
            binary_valid = false;
            break;
          }
          const Eigen::Vector3d center =
              moving_objs_->evaluateConstVel(id, world_time);
          const Eigen::Vector3d scale = moving_objs_->getObjScale(id);
          if (!center.allFinite() || !scale.allFinite())
          {
            components[1].valid = false;
            binary_valid = false;
            break;
          }
          const auto los = multi_uav_formation::
              segmentVerticalCylinderClearanceGradient(
                  observer, target, center,
                  0.5 * std::max(scale.x(), scale.y()), scale.z());
          if (!los.valid)
          {
            components[1].valid = false;
            binary_valid = false;
            break;
          }
          // The shared cylinder query uses +infinity to represent a valid
          // no-overlap result when the observer-target segment never enters
          // this obstacle's vertical slab.  The coordinator already treats
          // that result as zero dynamic-LOS risk.  Do the same here instead
          // of passing +infinity to directionalClearanceRisk(), whose finite
          // input contract would otherwise turn harmless non-overlap into a
          // TEAM_VISIBILITY_LINEARIZATION_INVALID failure.
          if (!std::isfinite(los.clearance))
            continue;
          const auto risk = multi_uav_formation::directionalClearanceRisk(
              los.clearance, static_los_margin_,
              static_los_margin_ + team_visibility_los_smoothing_);
          if (!risk.valid)
          {
            components[1].valid = false;
            break;
          }
          binary_los_clear = binary_los_clear &&
                             los.clearance >= static_los_margin_;
          if (risk.value > components[1].risk + 1.0e-12)
          {
            components[1].risk = risk.value;
            components[1].gradient_position =
                risk.derivative_clearance * los.gradient_start;
          }
        }
      }

      multi_uav_formation::PredictedYawState yaw_state;
      yaw_state.valid = visibility_yaw_valid_;
      yaw_state.yaw = visibility_yaw_;
      yaw_state.yaw_rate = visibility_yaw_rate_;
      if (yaw_state.valid && t > 1.0e-9)
      {
        const Eigen::Vector2d bearing = (target - observer).head<2>();
        if (bearing.norm() > 1.0e-6)
          yaw_state = multi_uav_formation::advanceTargetFacingYaw(
              yaw_state, std::atan2(bearing.y(), bearing.x()), t);
      }
      multi_uav_formation::TrackingCameraContract camera;
      camera.horizontal_fov = visibility_camera_hfov_;
      camera.vertical_fov = visibility_camera_vfov_;
      camera.min_range = visibility_camera_min_range_;
      camera.max_range = visibility_camera_max_range_;
      const auto fov = multi_uav_formation::trackingCameraFovDirectionalRisk(
          observer, yaw_state.yaw, target, camera);
      if (yaw_state.valid && fov.valid)
      {
        components[2].valid = true;
        components[2].risk = fov.risk;
        components[2].gradient_position = fov.gradient_body_position;
        components[2].gradient_yaw = fov.gradient_yaw;
      }

      const Eigen::Vector3d range_gradient = delta / range;
      const auto near_risk = multi_uav_formation::directionalClearanceRisk(
          range - camera.min_range, 0.0, team_visibility_range_smoothing_);
      const auto far_risk = multi_uav_formation::directionalClearanceRisk(
          camera.max_range - range, 0.0,
          team_visibility_range_smoothing_);
      if (near_risk.valid && far_risk.valid)
      {
        const bool near_limits = near_risk.value >= far_risk.value;
        const auto &risk = near_limits ? near_risk : far_risk;
        components[3].valid = true;
        components[3].risk = risk.value;
        const Eigen::Vector3d clearance_gradient =
            near_limits ? Eigen::Vector3d(range_gradient)
                        : Eigen::Vector3d(-range_gradient);
        components[3].gradient_position =
            risk.derivative_clearance * clearance_gradient;
      }
      team_margin_invalid_component_ = -1;
      for (int component_index = 0; component_index < 4; ++component_index)
      {
        const auto &component = components[component_index];
        if (!component.valid || !std::isfinite(component.risk) ||
            !component.gradient_position.allFinite() ||
            !std::isfinite(component.gradient_yaw))
        {
          team_margin_invalid_component_ = component_index;
          break;
        }
      }
      multi_uav_formation::composeVisibilityMargin(components, sample);
      const Eigen::Quaterniond world_from_body(
          Eigen::AngleAxisd(yaw_state.yaw, Eigen::Vector3d::UnitZ()));
      const auto binary_fov = multi_uav_formation::targetInTrackingCameraFov(
          observer, world_from_body, target, camera);
      sample.binary_valid = binary_valid && yaw_state.valid &&
                            std::isfinite(binary_fov.range);
      sample.binary_visible = sample.binary_valid && binary_los_clear &&
                              binary_fov.valid;
      sample.valid = sample.margin_valid;
      sample.value = sample.margin_valid
                         ? std::max(0.0, std::min(1.0,
                               0.5 * (sample.margin + 1.0)))
                         : 0.0;
      return sample;
  }

  bool PolyTrajOptimizer::evaluateVisibilityForecast(
      const poly_traj::Trajectory &trajectory, const double activation_world_time,
      const double horizon, const double sample_dt,
      std::vector<VisibilityTraceSample> &trace) const
  {
    trace.clear();
    if (trajectory.getPieceNum() <= 0 ||
        !std::isfinite(activation_world_time) || !std::isfinite(horizon) ||
        horizon <= 0.0)
      return false;
    // Never extrapolate a polynomial past its own mathematical end.
    const double usable = std::min(horizon, trajectory.getTotalDuration());
    if (!std::isfinite(usable) || usable <= 1.0e-6)
      return false;
    const double dt =
        std::isfinite(sample_dt) && sample_dt > 1.0e-3 ? sample_dt : 0.05;
    const int count = std::max(
        2, static_cast<int>(std::ceil(usable / dt)) + 1);
    trace.reserve(static_cast<size_t>(count));
    for (int index = 0; index < count; ++index)
    {
      const double alpha = static_cast<double>(index) /
                           static_cast<double>(count - 1);
      const double t = usable * alpha;
      const auto sample = visibilitySampleAt(trajectory, t, activation_world_time);
      VisibilityTraceSample entry;
      entry.world_time = activation_world_time + t;
      entry.margin = sample.margin;
      entry.limiter = static_cast<int>(sample.limiter);
      entry.valid = sample.margin_valid && std::isfinite(sample.margin);
      trace.push_back(entry);
    }
    return true;
  }

  bool PolyTrajOptimizer::evaluateBinaryVisibilityForecast(
      const poly_traj::Trajectory &trajectory,
      const double activation_world_time, const double horizon,
      const double sample_dt,
      std::vector<BinaryVisibilityTraceSample> &trace) const
  {
    trace.clear();
    if (trajectory.getPieceNum() <= 0 ||
        !std::isfinite(activation_world_time) || !std::isfinite(horizon) ||
        !std::isfinite(sample_dt) || horizon <= 0.0 || sample_dt <= 0.0)
      return false;
    const double end = activation_world_time +
        std::min(horizon, trajectory.getTotalDuration());
    // Unlike the legacy margin trace, this grid is aligned to absolute world
    // time.  All three UAVs therefore report actual binary evaluations at
    // exactly the same instants without binary interpolation.
    const double first = std::ceil(activation_world_time / sample_dt) * sample_dt;
    for (double world_time = first; world_time <= end + 1.0e-9;
         world_time += sample_dt)
    {
      const auto sample = visibilitySampleAt(
          trajectory, world_time - activation_world_time,
          activation_world_time);
      if (!sample.valid || !sample.binary_valid || !sample.margin_valid)
      {
        trace.clear();
        return false;
      }
      trace.push_back({world_time, sample.binary_visible});
    }
    return !trace.empty();
  }

  bool PolyTrajOptimizer::evaluateVisibilityMarginAt(
      const poly_traj::Trajectory &trajectory, const double activation_world_time,
      const double world_time, double &margin, int &limiter) const
  {
    margin = -std::numeric_limits<double>::infinity();
    limiter = 0;
    if (trajectory.getPieceNum() <= 0 || !std::isfinite(world_time))
      return false;
    const auto sample = visibilitySampleAt(
        trajectory, world_time - activation_world_time, activation_world_time);
    if (!sample.margin_valid || !std::isfinite(sample.margin))
      return false;
    margin = sample.margin;
    limiter = static_cast<int>(sample.limiter);
    return true;
  }

  bool PolyTrajOptimizer::runTeamContractSCP(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
      Eigen::MatrixXd &optimal_points, double &final_cost,
      TeamSCPResult &result)
  {
    team_scp_result_ = TeamSCPResult();
    team_scp_result_.attempted = true;
    result = team_scp_result_;
    if (!team_spatiotemporal_scp_enabled_)
    {
      result.reason = "TEAM_SPATIOTEMPORAL_SCP_DISABLED";
      return false;
    }
    if (!team_visibility_contract_.active)
    {
      result.reason = "TEAM_HANDOFF_CONTRACT_INVALID";
      return false;
    }
    poly_traj::MinJerkOpt before;
    before.reset(iniState, finState, static_cast<int>(initT.size()));
    before.generate(initInnerPts, initT);
    CandidateDynamicsSummary before_dynamics;
    evaluateCandidateDynamicsLattice(before.getTraj(), before_dynamics);
    team_scp_result_.max_velocity_before = before_dynamics.max_vel_value;
    team_scp_result_.max_acceleration_before = before_dynamics.max_acc_value;
    team_scp_result_.max_jerk_before = before_dynamics.max_jerk_value;

    team_contract_scp_active_ = true;
    bool success = runCandidateHardCorridorSCP(
        iniState, finState, initInnerPts, initT, optimal_points, final_cost);

    // AUTO deliberately starts in the smallest useful decision subspace.
    // Failure of TEAM_T_ONLY is not evidence that the fixed Local topology is
    // infeasible: spatial P columns have not been available yet.  Retry once
    // in TEAM_PT from the exact original Local seed, under the same contract
    // and execution deadline.  runCandidateHardCorridorSCP() regenerates the
    // MINCO state from initInnerPts/initT, so no failed T iterate leaks into
    // the P/T projection.
    if (!success &&
        team_scp_result_.mode == TeamSCPMode::TEAM_T_ONLY &&
        team_visibility_contract_.requested_mode == TeamSCPMode::AUTO &&
        executionBudgetAvailable())
    {
      const TeamSCPResult t_only_result = team_scp_result_;
      ROS_INFO("[TEAM_SCP_T_FAIL] contract_id=%lu drone=%d reason=%s "
               "action=RETRY_TEAM_PT TEAM_SCP_DP_NORM=%.9f "
               "TEAM_SCP_DTAU_NORM=%.9f TEAM_SCP_DT_NORM=%.9f "
               "TEAM_SCP_LINEAR_MARGIN_PRED=%.9f "
               "TEAM_SCP_NONLINEAR_MARGIN=%.9f "
               "TEAM_SCP_MODEL_AGREEMENT=%.9f",
               static_cast<unsigned long>(
                   team_visibility_contract_.contract_id),
               drone_id_, last_candidate_final_status_reason_.c_str(),
               t_only_result.delta_p_norm,
               t_only_result.delta_virtual_t_norm,
               t_only_result.delta_real_t_norm,
               t_only_result.linear_margin_prediction,
               t_only_result.margin_after,
               t_only_result.model_agreement);
      const TeamSCPMode requested_mode =
          team_visibility_contract_.requested_mode;
      team_visibility_contract_.requested_mode = TeamSCPMode::TEAM_PT;
      team_scp_result_ = TeamSCPResult();
      team_scp_result_.attempted = true;
      team_scp_result_.max_velocity_before = before_dynamics.max_vel_value;
      team_scp_result_.max_acceleration_before = before_dynamics.max_acc_value;
      team_scp_result_.max_jerk_before = before_dynamics.max_jerk_value;
      ROS_INFO("[TEAM_SCP_T_TO_PT_ESCALATION] contract_id=%lu drone=%d "
               "reason=T_SUBSPACE_FAILED local_seed_unchanged=1",
               static_cast<unsigned long>(
                   team_visibility_contract_.contract_id),
               drone_id_);
      success = runCandidateHardCorridorSCP(
          iniState, finState, initInnerPts, initT, optimal_points, final_cost);
      team_visibility_contract_.requested_mode = requested_mode;
    }
    team_contract_scp_active_ = false;
    team_scp_result_.success = success;
    // Feedback098: the early-accept path already recorded a precise reason; the
    // generic candidate-status field must not clobber it, otherwise the audit
    // line reported reason="" for a TEAM_VISIBILITY_ALREADY_SATISFIED result.
    if (team_scp_result_.reason != "TEAM_VISIBILITY_ALREADY_SATISFIED")
      team_scp_result_.reason = last_candidate_final_status_reason_;
    if (jerkOpt_.getTraj().getPieceNum() > 0)
    {
      const poly_traj::Trajectory after = jerkOpt_.getTraj();
      const Eigen::MatrixXd before_positions = before.getTraj().getPositions();
      const Eigen::MatrixXd after_positions = after.getPositions();
      if (before_positions.rows() == after_positions.rows() &&
          before_positions.cols() == after_positions.cols())
        team_scp_result_.delta_p_norm =
            (after_positions - before_positions).norm();
      const Eigen::VectorXd after_t = after.getDurations();
      if (after_t.size() == initT.size())
      {
        team_scp_result_.delta_real_t_norm = (after_t - initT).norm();
        Eigen::VectorXd before_virtual(initT.size());
        Eigen::VectorXd after_virtual(after_t.size());
        RealT2VirtualT(initT, before_virtual);
        RealT2VirtualT(after_t, after_virtual);
        team_scp_result_.delta_virtual_t_norm =
            (after_virtual - before_virtual).norm();
      }
      CandidateDynamicsSummary after_dynamics;
      evaluateCandidateDynamicsLattice(after, after_dynamics);
      team_scp_result_.max_velocity_after = after_dynamics.max_vel_value;
      team_scp_result_.max_acceleration_after = after_dynamics.max_acc_value;
      team_scp_result_.max_jerk_after = after_dynamics.max_jerk_value;
    }
    result = team_scp_result_;
    const char *mode = result.mode == TeamSCPMode::TEAM_T_ONLY
                           ? "T"
                           : (result.mode == TeamSCPMode::TEAM_PT ? "PT"
                                                                  : "AUTO");
    ROS_INFO("[TEAM_SCP_%s_%s] contract_id=%lu drone=%d reason=%s "
             "TEAM_SCP_DP_NORM=%.9f TEAM_SCP_DTAU_NORM=%.9f "
             "TEAM_SCP_DT_NORM=%.9f TEAM_SCP_LINEAR_MARGIN_PRED=%.9f "
             "TEAM_SCP_NONLINEAR_MARGIN=%.9f TEAM_SCP_MODEL_AGREEMENT=%.9f "
             "max_v=%.6f->%.6f max_a=%.6f->%.6f max_j=%.6f->%.6f",
             mode, success ? "PASS" : "FAIL",
             static_cast<unsigned long>(team_visibility_contract_.contract_id),
             drone_id_, result.reason.c_str(), result.delta_p_norm,
             result.delta_virtual_t_norm, result.delta_real_t_norm,
             result.linear_margin_prediction, result.margin_after,
             result.model_agreement, result.max_velocity_before,
             result.max_velocity_after, result.max_acceleration_before,
             result.max_acceleration_after, result.max_jerk_before,
             result.max_jerk_after);
    return success;
  }

  namespace
  {
    /* A one-piece trajectory has no interior spatial decision variable: the
     * spatial hard-corridor SCP would run with a zero-width P block (the
     * Team T-only path may still refine durations).  Returns whether the
     * given piece count provides interior waypoints for spatial refinement. */
    bool candidateHardCorridorSpatialRefinementSupported(const int piece_count)
    {
      return piece_count >= 2;
    }
  }

  bool PolyTrajOptimizer::runCandidateHardCorridorSCP(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
      Eigen::MatrixXd &optimal_points, double &final_cost)
  {
    const bool team_scp = team_contract_scp_active_ &&
                          team_visibility_contract_.active;
    // Feedback119: corridor-contract telemetry for the manager counters
    // (CORRIDOR_SCP_INFEASIBLE / CONTINUOUS_CORRIDOR_VIOLATION).  Filled by
    // the terminal logger below on every exit path.
    last_corridor_scp_telemetry_ = CorridorScpTelemetry();
    last_corridor_scp_telemetry_.attempted = true;
    if (team_scp)
    {
      team_scp_result_.attempted = true;
      team_scp_result_.success = false;
      team_scp_result_.reason = "TEAM_SCP_RUNNING";
      ROS_INFO("[TEAM_SCP_START] contract_id=%lu drone=%d "
               "activation=%.9f acquire_by=%.9f preserve_until=%.9f "
               "required_margin=%.6f role=%s",
               static_cast<unsigned long>(team_visibility_contract_.contract_id),
               drone_id_, team_visibility_contract_.activation_world_time,
               team_visibility_contract_.acquire_by_world_time,
               team_visibility_contract_.preserve_until_world_time,
               team_visibility_contract_.required_margin,
               drone_id_ == team_visibility_contract_.incoming_uav
                   ? "INCOMING"
                   : (drone_id_ == team_visibility_contract_.outgoing_uav
                          ? "OUTGOING"
                          : "STABLE"));
    }
    const auto log_preinitial_failure =
        [&](const char *reason, const double total_duration,
            const double reference_duration, const double raw_window_start,
            const double raw_window_end, const double clipped_window_start,
            const double clipped_window_end) {
          ROS_WARN(
              "[scp-hard-corridor-preinitial] reason=%s candidate_type=%s "
              "obstacle_id=%d piece_num=%d variable_num=%d total_duration=%.6f "
              "reference_duration=%.6f raw_conflict_time=%.6f raw_window_start=%.6f "
              "raw_window_end=%.6f clipped_window_start=%.6f clipped_window_end=%.6f",
              reason, gradient_audit_candidate_.c_str(), gradient_audit_obstacle_id_,
              static_cast<int>(initT.size()),
              initT.size() > 0 ? 3 * std::max(0, static_cast<int>(initT.size()) - 1) : -1,
              total_duration, reference_duration, candidate_risk_conflict_time_,
              raw_window_start, raw_window_end, clipped_window_start,
              clipped_window_end);
        };

    // Pre-initial failures occur before the regular SCP terminal logger is
    // available.  Keep the Local-SFC lifecycle complete by emitting a
    // terminal record for those paths as well.
    const auto logLocalSfcPreinitial = [&](const char *reason,
                                           const double duration) {
      if (candidate_local_sfc_planes_.empty())
        return;
      ROS_INFO(
          "[side-local-sfc-final] drone_id=%d candidate_type=%s obstacle_id=%d "
          "planes=%zu scp_success=0 final_success=0 reason=%s duration=%.6f "
          "static_free=-1 side_valid=-1 dynamic_valid=-1 corridor_violation=-1.000000 "
          "static_violation=-1.000000 v_violation=-1.000000 a_violation=-1.000000 "
          "j_violation=-1.000000 clearance_gain=-1.000000",
          drone_id_, gradient_audit_candidate_.c_str(),
          gradient_audit_obstacle_id_, candidate_local_sfc_planes_.size(),
          reason != nullptr ? reason : "UNKNOWN", duration);
    };

    const double input_total_duration =
        initT.size() > 0 && initT.allFinite() ? initT.sum() : -1.0;
    const double reference_duration =
        candidate_preserve_reference_.getTotalDuration();
    const bool valid_reference = candidate_preserve_reference_valid_ &&
                                 candidate_preserve_reference_.getPieceNum() > 0 &&
                                 std::isfinite(reference_duration) &&
                                 reference_duration > 1.0e-6;
    const bool valid_input = initT.size() > 0 && initT.allFinite() &&
                             (initT.array() > 0.0).all() && initInnerPts.rows() == 3 &&
                             initInnerPts.cols() == initT.size() - 1 &&
                             initInnerPts.allFinite();
    if (!valid_input)
    {
      last_candidate_final_status_reason_ = "INVALID_INIT_T_OR_INNER_POINTS";
      log_preinitial_failure("INVALID_INIT_T_OR_INNER_POINTS",
                             input_total_duration, reference_duration, -1.0, -1.0,
                             -1.0, -1.0);
      logLocalSfcPreinitial("INVALID_INIT_T_OR_INNER_POINTS",
                            input_total_duration);
      return false;
    }

    // A one-piece trajectory has no interior spatial decision variable.
    // Its duration has already passed the dedicated time-only feasibility
    // correction in the SIDE frontend, and the frontend has independently
    // checked dynamics, static, dynamic and Local-SFC safety.  Entering the
    // spatial hard-corridor SCP with a zero-width P block is therefore not a
    // refinement and used to reach Eigen code that assumes an interior
    // waypoint.  Return a classified non-success so the existing verified
    // initializer fallback is rechecked by the current-revision final
    // preflight and remains eligible for immediate local rolling supply.
    if (!team_scp &&
        !candidateHardCorridorSpatialRefinementSupported(initT.size()))
    {
      last_candidate_final_status_reason_ =
          "SINGLE_PIECE_NO_SPATIAL_REFINEMENT";
      final_cost = std::numeric_limits<double>::infinity();
      optimal_points = jerkOpt_.getTraj().getPositions();
      ROS_INFO("[scp-hard-corridor-preinitial] reason=%s candidate_type=%s "
               "obstacle_id=%d piece_num=1 spatial_variable_num=0 "
               "action=VERIFIED_INITIALIZER_FALLBACK",
               last_candidate_final_status_reason_.c_str(),
               gradient_audit_candidate_.c_str(),
               gradient_audit_obstacle_id_);
      logLocalSfcPreinitial(last_candidate_final_status_reason_.c_str(),
                            input_total_duration);
      return false;
    }

    piece_num_ = static_cast<int>(initT.size());
    const int scp_candidate_id = ++scp_candidate_sequence_;
    const int position_dim = 3 * (piece_num_ - 1);
    const bool optimize_time = true;
    const int virtual_time_dim = piece_num_;
    variable_num_ = position_dim + virtual_time_dim;
    iter_num_ = 0;
    jerkOpt_.reset(iniState, finState, piece_num_);
    Eigen::MatrixXd current_points = initInnerPts;
    Eigen::VectorXd current_virtual_t = Eigen::VectorXd::Zero(piece_num_);
    if (optimize_time)
      RealT2VirtualT(initT, current_virtual_t);
    Eigen::VectorXd current_durations = initT;
    jerkOpt_.generate(current_points, current_durations);
    const double initial_total_duration = current_durations.sum();
    double total_duration = initial_total_duration;
    // Native cost terms (notably the distance-variance gradient) index the
    // constraint-point lattice with the current MINCO piece count.  A SIDE
    // candidate can arrive here carrying the previous nominal lattice, so
    // synchronize only the sampled point storage when its dimensions differ.
    // Existing separating rays are retained whenever their lattice is still
    // compatible with this candidate.
    if (cps_num_prePiece_ <= 0)
    {
      log_preinitial_failure("INVALID_CONSTRAINT_SAMPLE_COUNT", total_duration,
                             reference_duration, -1.0, -1.0, -1.0, -1.0);
      logLocalSfcPreinitial("INVALID_CONSTRAINT_SAMPLE_COUNT", total_duration);
      return false;
    }
    const int expected_cp_count = piece_num_ * cps_num_prePiece_ + 1;
    if (cps_.points.rows() != 3 || cps_.points.cols() != expected_cp_count ||
        cps_.cp_size != expected_cp_count)
    {
      const Eigen::MatrixXd synced_points =
          jerkOpt_.getInitConstraintPoints(cps_num_prePiece_);
      cps_.resize_cp(expected_cp_count);
      cps_.points = synced_points;
      ROS_INFO("[scp-constraint-state-sync] candidate_type=%s piece_num=%d "
               "cp_count=%d", gradient_audit_candidate_.c_str(), piece_num_,
               expected_cp_count);
    }

    // elastic risk profile 已随 elastic 语义退出 production;SCP 体内不读取该 profile,调用为空操作。


    if (!std::isfinite(total_duration) || total_duration <= 1.0e-6)
    {
      last_candidate_final_status_reason_ = "INVALID_DURATION";
      log_preinitial_failure("INVALID_DURATION", total_duration,
                             reference_duration, -1.0, -1.0, -1.0, -1.0);
      logLocalSfcPreinitial("INVALID_DURATION", total_duration);
      return false;
    }

    double window_start = 0.0;
    double window_end = total_duration;
    double raw_window_start = 0.0;
    double raw_window_end = total_duration;
    if (candidate_risk_window_valid_)
    {
      raw_window_start = candidate_risk_conflict_time_ - candidate_risk_half_window_;
      raw_window_end = candidate_risk_conflict_time_ + candidate_risk_half_window_;
      window_start = std::max(0.0, raw_window_start);
      window_end = std::min(total_duration, raw_window_end);
    }
    else
    {
      raw_window_start = (candidate_side_conflict_progress_ -
                          candidate_side_guidance_window_) * total_duration;
      raw_window_end = (candidate_side_conflict_progress_ +
                        candidate_side_guidance_window_) * total_duration;
      window_start = std::max(0.0, raw_window_start);
      window_end = std::min(total_duration, raw_window_end);
    }
    if (window_end <= window_start + 1.0e-6)
    {
      last_candidate_final_status_reason_ =
          window_end <= 0.0 || window_start >= total_duration
              ? "WINDOW_NO_OVERLAP"
              : "WINDOW_COLLAPSED_AFTER_CLIP";
      log_preinitial_failure(
          window_end <= 0.0 || window_start >= total_duration
              ? "WINDOW_NO_OVERLAP"
              : "WINDOW_COLLAPSED_AFTER_CLIP",
          total_duration, reference_duration, raw_window_start, raw_window_end,
          window_start, window_end);
      logLocalSfcPreinitial(
          window_end <= 0.0 || window_start >= total_duration
              ? "WINDOW_NO_OVERLAP"
              : "WINDOW_COLLAPSED_AFTER_CLIP",
          total_duration);
      return false;
    }

    const int sample_count = std::max(
        5, team_scp
               ? static_cast<int>(std::ceil(
                     std::max(0.05, team_visibility_contract_.preserve_until_world_time -
                                        team_visibility_contract_.activation_world_time) /
                     0.05)) + 1
               : candidate_hard_corridor_scp_samples_);
    const int dim = variable_num_;
    const double radius = std::max(
        0.05, team_scp ? team_scp_p_trust_
                       : candidate_hard_corridor_scp_radius_);
    // Keep the configured starting radii, but adapt the spatial and virtual
    // time blocks independently.  The envelope is deliberately derived from
    // the configured start value so this change does not retune the planner;
    // it only prevents a trust box from making an otherwise feasible linear
    // model artificially infeasible.
    double trust_p = std::max(
        1.0e-3, team_scp ? team_scp_p_trust_
                         : candidate_hard_corridor_scp_trust_region_);
    // The SCP variable remains virtual_T, but its trust block is expressed in
    // physical duration space.  We initialize the relative duration radius
    // from the previous virtual_T=0.15 envelope at the current iterate, so
    // this is a parameterization change rather than a trust enlargement.
    const double virtual_trust_initial = std::max(
        1.0e-3, candidate_hard_corridor_scp_time_trust_region_);
    std::vector<double> equivalent_time_ratios;
    equivalent_time_ratios.reserve(piece_num_);
    for (int i = 0; i < piece_num_; ++i)
    {
      const double duration = current_durations(i);
      const double jacobian = std::abs(realTimeJacobianAt(current_virtual_t(i)));
      if (std::isfinite(duration) && duration > 1.0e-6 &&
          std::isfinite(jacobian) && jacobian > 1.0e-9)
      {
        equivalent_time_ratios.push_back(virtual_trust_initial * jacobian /
                                         duration);
      }
    }
    double physical_time_trust_ratio = team_scp
        ? std::max(1.0e-3, team_scp_t_trust_ratio_)
        : virtual_trust_initial;
    if (!team_scp && !equivalent_time_ratios.empty())
    {
      std::sort(equivalent_time_ratios.begin(), equivalent_time_ratios.end());
      physical_time_trust_ratio =
          equivalent_time_ratios[equivalent_time_ratios.size() / 2];
    }
    if (!std::isfinite(physical_time_trust_ratio) ||
        physical_time_trust_ratio <= 1.0e-6)
      physical_time_trust_ratio = virtual_trust_initial;
    double trust_t = physical_time_trust_ratio;
    const double trust_p_initial = trust_p;
    const double trust_t_initial = trust_t;
    const double trust_p_min = std::max(1.0e-3, trust_p_initial * 0.25);
    const double trust_t_min = std::max(1.0e-3, trust_t_initial * 0.25);
    const double trust_p_max = std::max(trust_p_initial, trust_p_initial * 4.0);
    const double trust_t_max = std::max(trust_t_initial, trust_t_initial * 4.0);
    constexpr double kTrustGrowFactor = 1.25;
    constexpr double kTrustShrinkFactor = 0.5;
    constexpr double kTrustBoundaryRatio = 0.8;
    // The QP hard rows and the nonlinear MINCO regenerate do not share a
    // common merit function, so a textbook objective-reduction ratio is not
    // well-defined here.  Use one normalized model-agreement scale instead.
    // 0.05/0.10/0.20 correspond to small, noticeable, and large errors as a
    // fraction of the project's physical v/a/j limits.
    constexpr double kModelAgreementGood = 0.05;
    constexpr double kModelAgreementAcceptable = 0.10;
    constexpr double kModelAgreementPoor = 0.20;
    constexpr int kMaxTrustExpandAttempts = 8;
    constexpr int kMaxTrustShrinkAttempts = 8;
    int trust_expand_attempts = 0;
    int trust_shrink_attempts = 0;
    bool saw_qp_max_iter = false;
    std::string adaptive_failure_reason;
    const Eigen::VectorXd initial_virtual_t = current_virtual_t;
    const Eigen::VectorXd initial_durations = current_durations;
    double max_virtual_time_step = 0.0;
    double max_real_time_step = 0.0;
    double max_position_step = 0.0;

    // Model-agreement state for the current trial.  It is intentionally kept
    // local to the SCP loop: accepted steps update current_points/current_T
    // below, and the next iteration rebuilds all MINCO derivatives at that
    // new base point.
    double trial_model_error = std::numeric_limits<double>::quiet_NaN();
    double trial_model_error_v = std::numeric_limits<double>::quiet_NaN();
    double trial_model_error_a = std::numeric_limits<double>::quiet_NaN();
    double trial_model_error_j = std::numeric_limits<double>::quiet_NaN();
    bool trial_model_unreliable = false;
    bool trial_model_accurate_but_infeasible = false;
    const char *trial_agreement_class = "UNKNOWN";

    const auto formatVector = [](const Eigen::VectorXd &values) {
      std::ostringstream stream;
      stream << "[";
      for (int index = 0; index < values.size(); ++index)
      {
        if (index > 0)
          stream << ",";
        stream << values(index);
      }
      stream << "]";
      return stream.str();
    };

    // Candidate windows and Local-SFC intervals are stored by the frontend in
    // trajectory seconds.  Keep their normalized location so that every
    // virtual-time update moves the same spatial interval to the new time
    // axis instead of leaving stale absolute bounds behind.
    const double initial_window_start_ratio =
        std::max(0.0, std::min(1.0, window_start / initial_total_duration));
    const double initial_window_end_ratio =
        std::max(initial_window_start_ratio,
                 std::min(1.0, window_end / initial_total_duration));
    std::vector<std::pair<double, double>> local_sfc_ratios;
    local_sfc_ratios.reserve(candidate_local_sfc_planes_.size());
    double local_sfc_start_before = std::numeric_limits<double>::infinity();
    double local_sfc_end_before = 0.0;
    for (const LocalSfcPlane &plane : candidate_local_sfc_planes_)
    {
      local_sfc_start_before = std::min(local_sfc_start_before, plane.active_start);
      local_sfc_end_before = std::max(local_sfc_end_before, plane.active_end);
      local_sfc_ratios.emplace_back(
          std::max(0.0, std::min(1.0, plane.active_start / initial_total_duration)),
          std::max(0.0, std::min(1.0, plane.active_end / initial_total_duration)));
    }

    const auto refreshTimeState = [&]() -> bool {
      if (current_virtual_t.size() != piece_num_ ||
          !current_virtual_t.allFinite())
        return false;
      VirtualT2RealT(current_virtual_t, current_durations);
      if (current_durations.size() != piece_num_ ||
          !current_durations.allFinite() ||
          (current_durations.array() <= 1.0e-6).any())
        return false;
      total_duration = current_durations.sum();
      if (!std::isfinite(total_duration) || total_duration <= 1.0e-6)
        return false;
      window_start = initial_window_start_ratio * total_duration;
      window_end = initial_window_end_ratio * total_duration;
      return window_end > window_start + 1.0e-6;
    };
    if (!refreshTimeState())
    {
      last_candidate_final_status_reason_ = "INVALID_DURATION";
      log_preinitial_failure("INVALID_DURATION", total_duration,
                             reference_duration, raw_window_start, raw_window_end,
                             window_start, window_end);
      logLocalSfcPreinitial("INVALID_DURATION", total_duration);
      return false;
    }

    // The reference tube is a seed preference, not a physical predicate.
    // Trust bounds below remain the SCP numerical locality constraint.
    auto maxViolation = [&](const poly_traj::Trajectory &traj) {
      (void)traj;
      return 0.0;
    };

    // Nonlinear dynamic-body violation used by SCP trial acceptance.  The
    // final execution checker remains authoritative; this only prevents the
    // SCP loop from accepting a step that improves the side corridor while
    // silently undoing the moving-obstacle repair.
    const auto maxDynamicBodyViolation = [&](const poly_traj::Trajectory &traj) {
      if (!use_time_aware_moving_obj_cost_ || !moving_objs_ ||
          moving_objs_->getObjNums() <= 0 || traj.getPieceNum() <= 0)
        return 0.0;
      double check_end = traj.getTotalDuration();
      if (moving_obj_prediction_horizon_ > 1.0e-3)
        check_end = std::min(check_end, moving_obj_prediction_horizon_);
      double violation = 0.0;
      constexpr double kDt = 0.03;
      for (double t = 0.0; t < check_end + 1.0e-9; t += kDt)
      {
        const double tt = std::min(t, check_end);
        const Eigen::Vector3d p = traj.getPos(tt);
        const double query_time = t_now_ + tt;
        for (int id = 0; id < moving_objs_->getObjNums(); ++id)
        {
          if (!moving_objs_->hasPrediction(id))
            continue;
          const Eigen::Vector3d obstacle =
              moving_objs_->evaluateConstVel(id, query_time);
          if (!obstacle.allFinite() || !p.allFinite())
            continue;
          // Feedback126: per-object clearance, same as preflight.
          violation = std::max(violation,
                               movingObjHardClearanceForObject(id) -
                                   (p - obstacle).norm());
        }
      }
      return std::max(0.0, violation);
    };

    // Feedback126: swarm violation from the unified physical predicate.
    // Used by the SCP trial-acceptance chain (monotone) and the terminal
    // exit gate (absolute) exactly like the dynamic-body term above.
    const auto maxSwarmViolation = [&](const poly_traj::Trajectory &traj) {
      UnifiedSwarmFinding finding;
      if (!swarmPhysicalViolation(traj, t_now_, touch_goal_, &finding))
        return 0.0;
      return std::max(0.0, swarm_clearance_ - std::sqrt(finding.ellip_dist2));
    };

    const auto maxLocalSfcViolation = [&](const poly_traj::Trajectory &traj) {
      double max_violation = 0.0;
      const int samples = std::max(5, candidate_hard_corridor_scp_samples_);
      for (size_t plane_index = 0;
           plane_index < candidate_local_sfc_planes_.size(); ++plane_index)
      {
        const LocalSfcPlane &plane = candidate_local_sfc_planes_[plane_index];
        if (plane.source == LocalSfcPlane::LOS_OBSERVATION_SIDE)
          continue;
        const double normal_norm = plane.normal.norm();
        double active_start = 0.0, active_end = 0.0;
        if (!plane.normal.allFinite() || !plane.point.allFinite() ||
            !std::isfinite(plane.clearance) || normal_norm < 1.0e-6 ||
            !staticPlaneInterval(plane, traj.getDurations(),
                                 active_start, active_end))
          return std::numeric_limits<double>::infinity();
        const Eigen::Vector3d normal = plane.normal / normal_norm;
        for (int sample = 0; sample < samples; ++sample)
        {
          const double alpha = static_cast<double>(sample) /
                               static_cast<double>(samples - 1);
          const double t = active_start + alpha * (active_end - active_start);
          const Eigen::Vector3d p = traj.getPos(t);
          max_violation = std::max(max_violation,
              plane.clearance - normal.dot(p - plane.point));
        }
      }
      return std::max(0.0, max_violation);
    };

    const auto sampledDynamics = [](const poly_traj::Trajectory &traj,
                                    double &max_vel, double &max_acc,
                                    double &max_jer) {
      max_vel = traj.getMaxVelRate();
      max_acc = traj.getMaxAccRate();
      max_jer = 0.0;
      const double duration = traj.getTotalDuration();
      if (!std::isfinite(duration) || duration <= 0.0)
        return;
      const double dt = std::max(0.02, std::min(0.05, duration / 100.0));
      for (double t = 0.0; t < duration + 1.0e-6; t += dt)
      {
        const double tt = std::min(t, duration);
        max_jer = std::max(max_jer, traj.getJer(tt).norm());
      }
      max_jer = std::max(max_jer, traj.getJer(duration).norm());
    };

    struct ScpDynamicsSummary
    {
      int constraint_count{0};
      int velocity_constraint_count{0};
      int acceleration_constraint_count{0};
      int jerk_constraint_count{0};
      // Values below are evaluated on exactly the same per-piece sample
      // lattice as the hard dynamics rows.  They are diagnostic values only;
      // the existing violation fields and limits remain unchanged.
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

    constexpr int SAMPLE_POSITION = 0;
    constexpr int SAMPLE_VELOCITY = 1;
    constexpr int SAMPLE_ACCELERATION = 2;
    constexpr int SAMPLE_JERK = 3;

    const auto locatePieceTime = [&](const double t_query,
                                     int &piece_index,
                                     double &piece_time) -> bool {
      if (current_durations.size() <= 0 || !std::isfinite(t_query))
        return false;
      double remaining = std::min(std::max(0.0, t_query), total_duration);
      for (int piece = 0; piece < current_durations.size(); ++piece)
      {
        const double duration = current_durations(piece);
        if (duration <= 0.0 || !std::isfinite(duration))
          return false;
        if (remaining <= duration || piece == current_durations.size() - 1)
        {
          piece_index = piece;
          piece_time = std::min(std::max(0.0, remaining), duration);
          return true;
        }
        remaining -= duration;
      }
      return false;
    };

    // One visibility-margin implementation feeds both the Team hard rows and
    // their exact nonlinear postcheck.  The world-time sample is fixed; a
    // duration update changes only which piece/local-time is evaluated.  It
    // never shifts the target or obstacle snapshot to a different time.
    // Thin wrapper: the margin authority itself now lives in
    // visibilitySampleAt() so the Planner-side forecast and the Team-SCP seed
    // evaluation are guaranteed to use exactly the same quantity.
    const auto teamMarginAt = [&](const poly_traj::Trajectory &trajectory,
                                  const double trajectory_time) {
      if (!team_scp)
        return multi_uav_formation::ContinuousVisibilitySample();
      return visibilitySampleAt(
          trajectory, trajectory_time,
          team_visibility_contract_.activation_world_time);
    };

    const auto minimumTeamMargin = [&](const poly_traj::Trajectory &trajectory,
                                       bool *binary_ok = nullptr) {
      double minimum = std::numeric_limits<double>::infinity();
      bool all_binary = true;
      if (!team_scp)
        return minimum;
      const double start = drone_id_ == team_visibility_contract_.incoming_uav
          ? std::max(0.0,
                team_visibility_contract_.acquire_by_world_time -
                    team_visibility_contract_.activation_world_time)
          : 0.0;
      const double end = std::min(
          trajectory.getTotalDuration(),
          team_visibility_contract_.preserve_until_world_time -
              team_visibility_contract_.activation_world_time);
      if (end < start - 1.0e-9)
        return -std::numeric_limits<double>::infinity();
      const int count = std::max(
          2, static_cast<int>(std::ceil((end - start) / 0.05)) + 1);
      for (int sample_index = 0; sample_index < count; ++sample_index)
      {
        const double alpha = count > 1
                                 ? static_cast<double>(sample_index) /
                                       static_cast<double>(count - 1)
                                 : 0.0;
        const auto sample = teamMarginAt(
            trajectory, start + alpha * (end - start));
        if (!sample.margin_valid)
          return -std::numeric_limits<double>::infinity();
        minimum = std::min(minimum, sample.margin);
        all_binary = all_binary && sample.binary_valid &&
                     sample.binary_visible;
      }
      if (binary_ok != nullptr)
        *binary_ok = all_binary;
      return minimum;
    };

    const auto maxTeamSideViolation = [&](const poly_traj::Trajectory &trajectory) {
      if (!team_scp || !candidate_side_bias_enabled_ ||
          !candidate_side_region_enabled_ || trajectory.getPieceNum() <= 0)
        return 0.0;
      double violation = 0.0;
      constexpr int kSamples = 41;
      for (int sample_index = 0; sample_index < kSamples; ++sample_index)
      {
        const double progress = static_cast<double>(sample_index) /
                                static_cast<double>(kSamples - 1);
        violation = std::max(
            violation,
            std::abs(candidateSideRegionViolation(
                progress,
                trajectory.getPos(progress * trajectory.getTotalDuration()))));
      }
      return violation;
    };

    // Static audit of the Local-SFC association.  The optimizer uses one
    // local sample lattice per plane; expose that exact scope, plus overlap
    // on the common time axis, without changing any constraint rows.
    if (!candidate_local_sfc_planes_.empty())
    {
      std::vector<std::pair<double, double>> audit_intervals;
      audit_intervals.reserve(candidate_local_sfc_planes_.size());
      for (size_t plane_index = 0;
           plane_index < candidate_local_sfc_planes_.size(); ++plane_index)
      {
        const LocalSfcPlane &plane = candidate_local_sfc_planes_[plane_index];
        const double start = !plane.world_time_anchored &&
                                     plane_index < local_sfc_ratios.size()
                                 ? local_sfc_ratios[plane_index].first * total_duration
                                 : plane.active_start;
        const double end = !plane.world_time_anchored &&
                                   plane_index < local_sfc_ratios.size()
                               ? local_sfc_ratios[plane_index].second * total_duration
                               : plane.active_end;
        audit_intervals.emplace_back(start, end);
        int active_sample_count = 0;
        for (int sample = 0; sample < sample_count; ++sample)
        {
          const double alpha = static_cast<double>(sample) /
                               static_cast<double>(sample_count - 1);
          const double t = start + alpha * (end - start);
          int piece_index = -1;
          double piece_time = 0.0;
          if (locatePieceTime(t, piece_index, piece_time))
            ++active_sample_count;
        }
        const Eigen::Vector3d normal = plane.normal.norm() > 1.0e-6
                                           ? plane.normal.normalized()
                                           : Eigen::Vector3d::Zero();
        const double guide_margin =
            normal.allFinite() && plane.guide.allFinite() && plane.point.allFinite()
                ? normal.dot(plane.guide - plane.point) - plane.clearance
                : std::numeric_limits<double>::quiet_NaN();
        const double progress_start = total_duration > 1.0e-6
                                          ? start / total_duration : -1.0;
        const double progress_end = total_duration > 1.0e-6
                                        ? end / total_duration : -1.0;
        ROS_INFO(
            "[local-sfc-plane-audit] candidate_id=%d plane_id=%zu source_segment=%d "
            "source_guide_index=%d progress_start=%.6f progress_end=%.6f "
            "time_start=%.6f time_end=%.6f guide_margin=%.6f active_sample_count=%d",
            scp_candidate_id, plane_index, plane.source_segment,
            plane.source_guide_index, progress_start, progress_end, start, end,
            guide_margin, active_sample_count);
      }

      std::vector<int> overlap_counts(sample_count, 0);
      std::vector<double> overlap_min_margin(
          sample_count, std::numeric_limits<double>::infinity());
      for (int sample = 0; sample < sample_count; ++sample)
      {
        const double alpha = static_cast<double>(sample) /
                             static_cast<double>(sample_count - 1);
        const double t = alpha * total_duration;
        const Eigen::Vector3d p = jerkOpt_.getTraj().getPos(t);
        for (size_t plane_index = 0; plane_index < audit_intervals.size(); ++plane_index)
        {
          const double start = audit_intervals[plane_index].first;
          const double end = audit_intervals[plane_index].second;
          if (t + 1.0e-9 < start || t - 1.0e-9 > end)
            continue;
          ++overlap_counts[sample];
          const LocalSfcPlane &plane = candidate_local_sfc_planes_[plane_index];
          if (plane.normal.norm() > 1.0e-6 && p.allFinite())
          {
            const double margin = plane.normal.normalized().dot(p - plane.point) -
                                  plane.clearance;
            overlap_min_margin[sample] = std::min(overlap_min_margin[sample], margin);
          }
        }
        ROS_INFO("[local-sfc-sample-audit] candidate_id=%d sample_id=%d "
                 "progress=%.6f active_planes=%d min_margin=%.6f",
                 scp_candidate_id, sample,
                 total_duration > 1.0e-6 ? t / total_duration : -1.0,
                 overlap_counts[sample],
                 std::isfinite(overlap_min_margin[sample])
                     ? overlap_min_margin[sample] : 0.0);
      }
    }

    const auto mincoSampleGradientWrtX =
        [&](const int piece_index, const double piece_time,
            const Eigen::Vector3d &grad_sample,
            const int derivative_order,
            const Eigen::VectorXd &ds_dT) -> Eigen::RowVectorXd {
      return mincoSampleGradientWrtDecision(
          piece_index, piece_time, grad_sample, derivative_order, ds_dT,
          current_virtual_t);
    };

    // Reuse the native MINCO obstacle rays populated by
    // finelyCheckAndSetConstraintPoints().  obstacleGradCostP() defines the
    // same separating quantity as
    //   dist = (p - base_point) dot direction,
    //   violation = obs_clearance_ - dist.
    // This helper evaluates that native representation on the current
    // trajectory; no second map, SDF, or corridor generator is introduced.
    auto maxNativeStaticViolation = [&](const poly_traj::Trajectory &traj) {
      double max_violation = 0.0;
      if (cps_.cp_size <= 0 || cps_num_prePiece_ <= 0 || piece_num_ <= 0)
        return max_violation;
      for (int cp = 0; cp < cps_.cp_size; ++cp)
      {
        int piece = std::min(cp / cps_num_prePiece_, piece_num_ - 1);
        const int local_index = cp % cps_num_prePiece_;
        double t = 0.0;
        for (int i = 0; i < piece; ++i)
          t += current_durations(i);
        t += static_cast<double>(local_index) /
             static_cast<double>(cps_num_prePiece_) * current_durations(piece);
        if (cp == cps_.cp_size - 1)
          t = total_duration;
        const Eigen::Vector3d p = traj.getPos(std::min(total_duration, t));
        for (size_t j = 0; j < cps_.direction[cp].size(); ++j)
        {
          const Eigen::Vector3d &normal = cps_.direction[cp][j];
          const Eigen::Vector3d &base = cps_.base_point[cp][j];
          if (!normal.allFinite() || !base.allFinite() || !p.allFinite())
            continue;
          max_violation = std::max(
              max_violation, obs_clearance_ - (p - base).dot(normal));
        }
      }
      return std::max(0.0, max_violation);
    };

    const auto appendLinearUpperBound =
        [](const Eigen::RowVectorXd &row, const double ub,
           std::vector<Eigen::RowVectorXd> &rows_out,
           std::vector<double> &upper_out) -> bool {
      if (!std::isfinite(ub) || !row.allFinite())
        return false;
      const double row_norm = row.norm();
      if (row_norm < 1.0e-10)
      {
        if (ub >= 0.0)
          return false;
        rows_out.emplace_back(row);
        upper_out.push_back(ub);
        return true;
      }
      rows_out.emplace_back(row / row_norm);
      upper_out.push_back(ub / row_norm);
      return true;
    };

    // Build the native obstacle-ray set once before the first SCP subproblem.
    // The final checker remains authoritative; this call only exposes its
    // existing separating geometry to the hard SCP rows below.
    std::vector<std::pair<int, int>> initial_static_segments;
    const CHK_RET initial_static_check =
        finelyCheckAndSetConstraintPoints(initial_static_segments, jerkOpt_, true);
    if (initial_static_check == CHK_RET::ERR)
    {
      last_candidate_final_status_reason_ = "STATIC_QP_FAILURE";
      ROS_WARN("[scp-static-failure] reason=STATIC_QP_FAILURE piece=-1 time=NA "
               "static_violation=NA corridor_violation=%.6f trust_region=%.6f "
               "solver_status=constraint_generation_failed",
               maxViolation(jerkOpt_.getTraj()), trust_p);
      logLocalSfcPreinitial("STATIC_QP_FAILURE", total_duration);
      return false;
    }
    bool reference_static_collision = false;
    if (valid_reference && grid_map_)
    {
      const int ref_samples = std::max(5, sample_count);
      for (int sample = 0; sample < ref_samples; ++sample)
      {
        const double alpha = static_cast<double>(sample) /
                             static_cast<double>(ref_samples - 1);
        const double t = window_start + alpha * (window_end - window_start);
        const double rt = std::min(
            reference_duration,
            std::max(0.0, t / total_duration * reference_duration));
        if (grid_map_->getInflateOccupancy(
                candidate_preserve_reference_.getPos(rt)))
        {
          reference_static_collision = true;
          break;
        }
      }
    }
    int initial_static_rows = 0;
    for (int cp = 0; cp < cps_.cp_size; ++cp)
      initial_static_rows += static_cast<int>(cps_.direction[cp].size());
    ROS_INFO("[scp-static-constraint] candidate_type=%s sample_count=%d "
             "static_constraint_rows=%d reference_static_collision=%d "
             "max_static_violation_before=%.6f max_static_violation_after=%.6f",
             gradient_audit_candidate_.c_str(), sample_count, initial_static_rows,
             static_cast<int>(reference_static_collision),
             maxNativeStaticViolation(jerkOpt_.getTraj()),
             maxNativeStaticViolation(jerkOpt_.getTraj()));

    const auto appendDynamicsConstraints =
        [&](std::vector<Eigen::RowVectorXd> *rows_out,
            std::vector<double> *upper_out) -> ScpDynamicsSummary {
      ScpDynamicsSummary summary;
      const bool append = rows_out != nullptr && upper_out != nullptr;
      const bool sampled = forEachCandidateDynamicsSample(
          jerkOpt_.getTraj(),
          [&](const int piece, const int sample,
              const int dynamic_samples_per_piece, const double local_alpha,
              const double s1, const Eigen::Vector3d &vel,
              const Eigen::Vector3d &acc, const Eigen::Vector3d &jerk) {
          const auto appendSquaredNormLimit =
              [&](const Eigen::Vector3d &value, const double limit,
                  const int derivative_order, double &max_violation) {
            if (limit <= 0.0 || !std::isfinite(limit) || !value.allFinite())
              return;
            const double norm = value.norm();
            if (derivative_order == SAMPLE_VELOCITY)
              summary.max_vel_value = std::max(summary.max_vel_value, norm);
            else if (derivative_order == SAMPLE_ACCELERATION)
              summary.max_acc_value = std::max(summary.max_acc_value, norm);
            else if (derivative_order == SAMPLE_JERK)
              summary.max_jerk_value = std::max(summary.max_jerk_value, norm);
            max_violation = std::max(max_violation, norm - limit);
            if (append)
            {
              Eigen::VectorXd ds_dT = Eigen::VectorXd::Zero(piece_num_);
              ds_dT(piece) = local_alpha;
              const bool appended = appendLinearUpperBound(
                  mincoSampleGradientWrtX(piece, s1, 2.0 * value,
                                          derivative_order, ds_dT),
                  limit * limit - value.squaredNorm(),
                  *rows_out, *upper_out);
              if (appended)
              {
                ++summary.constraint_count;
                if (derivative_order == SAMPLE_VELOCITY)
                  ++summary.velocity_constraint_count;
                else if (derivative_order == SAMPLE_ACCELERATION)
                  ++summary.acceleration_constraint_count;
                else if (derivative_order == SAMPLE_JERK)
                  ++summary.jerk_constraint_count;
              }
            }
          };

          appendSquaredNormLimit(vel, max_vel_, SAMPLE_VELOCITY,
                                 summary.max_vel_violation);
          appendSquaredNormLimit(acc, max_acc_, SAMPLE_ACCELERATION,
                                 summary.max_acc_violation);
          appendSquaredNormLimit(jerk, max_jer_, SAMPLE_JERK,
                                 summary.max_jerk_violation);
        });
      if (!sampled)
      {
        summary.max_vel_violation = std::numeric_limits<double>::infinity();
        summary.max_acc_violation = std::numeric_limits<double>::infinity();
        summary.max_jerk_violation = std::numeric_limits<double>::infinity();
        return summary;
      }
      summary.max_vel_violation = std::max(0.0, summary.max_vel_violation);
      summary.max_acc_violation = std::max(0.0, summary.max_acc_violation);
      summary.max_jerk_violation = std::max(0.0, summary.max_jerk_violation);
      return summary;
    };

    // Predict the same sampled dynamics quantities that the QP rows model.
    // Each row is a first-order model of squared norm:
    //   ||d(x)||^2 + grad(||d(x)||^2)^T * delta_x.
    // This helper intentionally uses the same sample lattice, MINCO analytic
    // Jacobian, virtual_T mapping, and local duration derivative convention as
    // appendDynamicsConstraints().  It is audit-only and does not alter rows,
    // solver settings, or acceptance logic.
    const auto predictDynamicsForStep =
        [&](const Eigen::VectorXd &step) {
          ScpDynamicsSummary prediction;
          const int dynamic_samples_per_piece = std::max(1, cps_num_prePiece_);
          if (step.size() != dim)
            return prediction;
          // mincoSampleGradientWrtX() uses the optimizer's scratch gdC
          // buffer.  Preserve it so this diagnostics-only pass cannot alter
          // any subsequent native cost/constraint evaluation.
          const Eigen::MatrixXd gdC_before_prediction = jerkOpt_.get_gdC();
          for (int piece = 0; piece < piece_num_; ++piece)
          {
            const double duration = current_durations(piece);
            if (duration <= 0.0 || !std::isfinite(duration))
              continue;
            const Eigen::Matrix<double, 6, 3> coeff =
                jerkOpt_.get_b().block<6, 3>(piece * 6, 0);
            for (int sample = 0; sample <= dynamic_samples_per_piece; ++sample)
            {
              if (piece > 0 && sample == 0)
                continue;
              const double s1 = duration * static_cast<double>(sample) /
                                static_cast<double>(dynamic_samples_per_piece);
              const double s2 = s1 * s1;
              const double s3 = s2 * s1;
              const double s4 = s2 * s2;
              Eigen::Matrix<double, 6, 1> beta1, beta2, beta3;
              beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
              beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
              beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
              const Eigen::Vector3d vel = coeff.transpose() * beta1;
              const Eigen::Vector3d acc = coeff.transpose() * beta2;
              const Eigen::Vector3d jerk = coeff.transpose() * beta3;
              const double local_alpha =
                  static_cast<double>(sample) /
                  static_cast<double>(dynamic_samples_per_piece);
              Eigen::VectorXd ds_dT = Eigen::VectorXd::Zero(piece_num_);
              ds_dT(piece) = local_alpha;

              const auto predicted_norm =
                  [&](const Eigen::Vector3d &value, const double limit,
                      const int derivative_order) {
                    if (limit <= 0.0 || !std::isfinite(limit) ||
                        !value.allFinite())
                      return 0.0;
                    const Eigen::RowVectorXd grad_sq =
                        mincoSampleGradientWrtX(
                            piece, s1, 2.0 * value, derivative_order, ds_dT);
                    const double predicted_sq = value.squaredNorm() +
                                                 grad_sq.dot(step);
                    const double predicted =
                        std::sqrt(std::max(0.0, predicted_sq));
                    if (derivative_order == SAMPLE_VELOCITY)
                      prediction.max_vel_value =
                          std::max(prediction.max_vel_value, predicted);
                    else if (derivative_order == SAMPLE_ACCELERATION)
                      prediction.max_acc_value =
                          std::max(prediction.max_acc_value, predicted);
                    else if (derivative_order == SAMPLE_JERK)
                      prediction.max_jerk_value =
                          std::max(prediction.max_jerk_value, predicted);
                    return predicted;
                  };

              const double pred_vel =
                  predicted_norm(vel, max_vel_, SAMPLE_VELOCITY);
              const double pred_acc =
                  predicted_norm(acc, max_acc_, SAMPLE_ACCELERATION);
              const double pred_jerk =
                  predicted_norm(jerk, max_jer_, SAMPLE_JERK);
              if (max_vel_ > 0.0)
                prediction.max_vel_violation = std::max(
                    prediction.max_vel_violation, pred_vel - max_vel_);
              if (max_acc_ > 0.0)
                prediction.max_acc_violation = std::max(
                    prediction.max_acc_violation, pred_acc - max_acc_);
              if (max_jer_ > 0.0)
                prediction.max_jerk_violation = std::max(
                    prediction.max_jerk_violation, pred_jerk - max_jer_);
            }

            // Mirror the row lattice: the jerk stationary points of |jerk|^2
            // are part of the dynamics rows and of the authoritative trial
            // check, so the step prediction must see the same peaks.
            {
              poly_traj::CoefficientMat piece_coeff;
              for (int k = 0; k < 6; ++k)
                piece_coeff.col(k) = coeff.row(5 - k);
              const poly_traj::Piece jerk_piece(duration, piece_coeff);
              for (const double alpha_s : jerk_piece.getJerStationaryAlphas())
              {
                const double s1 = duration * alpha_s;
                const double s2 = s1 * s1;
                Eigen::Matrix<double, 6, 1> beta3;
                beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
                const Eigen::Vector3d jerk = coeff.transpose() * beta3;
                if (max_jer_ <= 0.0 || !jerk.allFinite())
                  continue;
                Eigen::VectorXd ds_dT = Eigen::VectorXd::Zero(piece_num_);
                ds_dT(piece) = alpha_s;
                const Eigen::RowVectorXd grad_sq = mincoSampleGradientWrtX(
                    piece, s1, 2.0 * jerk, SAMPLE_JERK, ds_dT);
                const double predicted_sq =
                    jerk.squaredNorm() + grad_sq.dot(step);
                const double pred_jerk =
                    std::sqrt(std::max(0.0, predicted_sq));
                prediction.max_jerk_value =
                    std::max(prediction.max_jerk_value, pred_jerk);
                prediction.max_jerk_violation = std::max(
                    prediction.max_jerk_violation, pred_jerk - max_jer_);
              }
            }
          }
          prediction.max_vel_violation =
              std::max(0.0, prediction.max_vel_violation);
          prediction.max_acc_violation =
              std::max(0.0, prediction.max_acc_violation);
          prediction.max_jerk_violation =
              std::max(0.0, prediction.max_jerk_violation);
          jerkOpt_.get_gdC() = gdC_before_prediction;
          return prediction;
        };

    const auto logModelAgreement =
        [&](const int iteration, const int attempt, const char *qp_status,
            const ScpDynamicsSummary &predicted,
            const ScpDynamicsSummary &actual, const double error_v,
            const double error_a, const double error_j,
            const double model_error, const char *agreement_class,
            const double max_dp, const double max_dt_real,
            const char *trust_decision, const bool accepted) {
          ROS_INFO(
              "[scp-model-agreement] candidate_id=%d scp_iter=%d attempt=%d "
              "qp_status=%s trust_p=%.6g trust_t_physical=%.6g max_dp=%.6g "
              "max_dt_real=%.6g pred_v=%.6f actual_v=%.6f "
              "pred_a=%.6f actual_a=%.6f pred_j=%.6f actual_j=%.6f "
              "model_error_v=%.6f model_error_a=%.6f model_error_j=%.6f "
              "model_error=%.6f agreement_class=%s trust_decision=%s "
              "trial_result=%s",
              scp_candidate_id, iteration, attempt,
              qp_status != nullptr ? qp_status : "unknown", trust_p, trust_t,
              max_dp, max_dt_real, predicted.max_vel_value,
              actual.max_vel_value, predicted.max_acc_value,
              actual.max_acc_value, predicted.max_jerk_value,
              actual.max_jerk_value, error_v, error_a, error_j, model_error,
              agreement_class != nullptr ? agreement_class : "UNKNOWN",
              trust_decision != nullptr ? trust_decision : "KEEP",
              accepted ? "ACCEPT" : "REJECT");
        };

    const auto logFinalStatus = [&](const char *reason, const bool scp_success,
                                    const bool final_success,
                                    const poly_traj::Trajectory &traj,
                                    const double corridor_violation) {
      double max_vel = -1.0;
      double max_acc = -1.0;
      double max_jer = -1.0;
      if (traj.getPieceNum() > 0 && std::isfinite(traj.getTotalDuration()) &&
          traj.getTotalDuration() > 0.0)
      {
        sampledDynamics(traj, max_vel, max_acc, max_jer);
      }
      last_candidate_final_status_reason_ = reason != nullptr ? reason : "UNKNOWN";
      // Feedback119: record the exact exit on the corridor-contract telemetry
      // so the manager can count CORRIDOR_SCP_INFEASIBLE vs deadline exits.
      last_corridor_scp_telemetry_.success = final_success;
      last_corridor_scp_telemetry_.reason = last_candidate_final_status_reason_;
      ROS_INFO("[scp-hard-corridor-final] drone_id=%d candidate_type=%s obstacle_id=%d "
               "scp_success=%d final_success=%d reason=%s duration=%.6f "
               "max_vel=%.6f max_acc=%.6f max_jerk=%.6f dynamic_clearance=NA "
               "static_clearance=NA corridor_violation=%.6f",
               drone_id_, gradient_audit_candidate_.c_str(),
               gradient_audit_obstacle_id_, static_cast<int>(scp_success),
               static_cast<int>(final_success), last_candidate_final_status_reason_.c_str(),
               traj.getPieceNum() > 0 ? traj.getTotalDuration() : -1.0,
               max_vel, max_acc, max_jer, corridor_violation);
      if (!candidate_local_sfc_planes_.empty())
      {
        const double local_sfc_violation = maxLocalSfcViolation(traj);
        const double static_violation = maxNativeStaticViolation(traj);
        const bool static_free = static_violation <= 1.0e-3;
        const bool dynamics_valid =
            traj.getPieceNum() > 0 && std::isfinite(max_vel) &&
            std::isfinite(max_acc) && std::isfinite(max_jer) &&
            (max_vel_ <= 0.0 || max_vel <= max_vel_ * 1.0001) &&
            (max_acc_ <= 0.0 || max_acc <= max_acc_ * 1.0001) &&
            (max_jer_ <= 0.0 || max_jer <= max_jer_ * 1.0001);
        const double vel_violation =
            max_vel_ > 0.0 ? std::max(0.0, max_vel - max_vel_) : -1.0;
        const double acc_violation =
            max_acc_ > 0.0 ? std::max(0.0, max_acc - max_acc_) : -1.0;
        const double jerk_violation =
            max_jer_ > 0.0 ? std::max(0.0, max_jer - max_jer_) : -1.0;
        ROS_INFO(
            "[side-local-sfc-final] drone_id=%d candidate_type=%s obstacle_id=%d "
            "planes=%zu scp_success=%d final_success=%d reason=%s duration=%.6f "
            "static_free=%d side_valid=-1 dynamic_valid=%d "
            "corridor_violation=%.6f static_violation=%.6f "
            "v_violation=%.6f a_violation=%.6f j_violation=%.6f "
            "clearance_gain=-1.000000",
            drone_id_, gradient_audit_candidate_.c_str(),
            gradient_audit_obstacle_id_, candidate_local_sfc_planes_.size(),
            static_cast<int>(scp_success), static_cast<int>(final_success),
            last_candidate_final_status_reason_.c_str(),
            traj.getPieceNum() > 0 ? traj.getTotalDuration() : -1.0,
            static_cast<int>(static_free), static_cast<int>(dynamics_valid),
            std::max(corridor_violation, local_sfc_violation),
            static_violation, vel_violation, acc_violation, jerk_violation);

        // Keep free-time and Local-SFC lifecycle diagnostics complete for
        // rejected candidates as well as accepted ones.  The trajectory state
        // is still live here; later cleanup must not erase the final time map.
        const double final_duration = current_durations.size() == piece_num_
                                          ? current_durations.sum()
                                          : traj.getTotalDuration();
        const std::string initial_duration_vector = formatVector(initial_durations);
        const std::string initial_virtual_vector = formatVector(initial_virtual_t);
        const std::string final_duration_vector =
            current_durations.size() == piece_num_
                ? formatVector(current_durations)
                : std::string("[]");
        const std::string final_virtual_vector = formatVector(current_virtual_t);
        ROS_INFO(
            "[local-sfc-free-time-final] candidate_id=%d candidate_type=%s "
            "piece_num=%d P_dim=%d virtual_T_dim=%d total_dim=%d "
            "initial_real_T=%s initial_virtual_T=%s final_real_T=%s "
            "final_virtual_T=%s max_abs_delta_P=%.6g "
            "max_abs_delta_virtual_T=%.6g",
            scp_candidate_id, gradient_audit_candidate_.c_str(), piece_num_,
            position_dim, virtual_time_dim, dim, initial_duration_vector.c_str(),
            initial_virtual_vector.c_str(), final_duration_vector.c_str(),
            final_virtual_vector.c_str(), max_position_step,
            max_virtual_time_step);
        const double local_sfc_start_after =
            std::isfinite(local_sfc_start_before) &&
                    initial_total_duration > 1.0e-6
                ? local_sfc_start_before / initial_total_duration * final_duration
                : -1.0;
        const double local_sfc_end_after =
            initial_total_duration > 1.0e-6
                ? local_sfc_end_before / initial_total_duration * final_duration
                : -1.0;
        const double progress_start =
            initial_total_duration > 1.0e-6
                ? local_sfc_start_before / initial_total_duration
                : -1.0;
        const double progress_end =
            initial_total_duration > 1.0e-6
                ? local_sfc_end_before / initial_total_duration
                : -1.0;
        ROS_INFO(
            "[local-sfc-time-map] candidate_id=%d candidate_type=%s "
            "sfc_active_progress_start=%.6f sfc_active_progress_end=%.6f "
            "sfc_active_time_before=%.6f sfc_active_time_after=%.6f "
            "sfc_active_end_before=%.6f sfc_active_end_after=%.6f "
            "duration_before=%.6f duration_after=%.6f",
            scp_candidate_id, gradient_audit_candidate_.c_str(), progress_start,
            progress_end, local_sfc_start_before, local_sfc_start_after,
            local_sfc_end_before, local_sfc_end_after, initial_total_duration,
            final_duration);
      }
    };

    int local_sfc_plane_count = 0;
    for (const LocalSfcPlane &plane : candidate_local_sfc_planes_)
    {
      // Feedback117: LOS_OBSERVATION_SIDE planes are enforced exclusively by
      // the semi-hard slack QP; a malformed LOS plane must never fail this
      // collision-corridor validation (and with it the candidate).
      if (plane.source == LocalSfcPlane::LOS_OBSERVATION_SIDE)
        continue;
      const double normal_norm = plane.normal.norm();
      double active_start = 0.0, active_end = 0.0;
      if (!plane.normal.allFinite() || !plane.point.allFinite() ||
          !std::isfinite(plane.clearance) || normal_norm < 1.0e-6 ||
          !staticPlaneInterval(plane, current_durations,
                               active_start, active_end))
      {
        logFinalStatus("LOCAL_SFC_INVALID_PLANE", false, false,
                       jerkOpt_.getTraj(), maxViolation(jerkOpt_.getTraj()));
        return false;
      }
      ++local_sfc_plane_count;
    }
    // Feedback119(corridor active-set 协议):Local-SFC 硬行的初始采样集
    // 只包含"必要"点——每平面 active 区间的端点+中点,再加上 MINCO seed
    // 的连续 violation argmax。seed 允许切角进入本函数;真实违反点由
    // continuousLocalSfcMaxViolation(五次多项式端点+f'=0 实根)解析定位,
    // 之后每轮 SCP 再把新出现的 t* 追加进下一轮行集。禁止 plane×25 均匀
    // 暴力扩张。
    const int static_sfc_plane_count = local_sfc_plane_count;
    std::vector<std::vector<double>> sfc_active_times(
        candidate_local_sfc_planes_.size());
    for (size_t plane_index = 0;
         plane_index < candidate_local_sfc_planes_.size(); ++plane_index)
    {
      const LocalSfcPlane &plane = candidate_local_sfc_planes_[plane_index];
      if (plane.source == LocalSfcPlane::LOS_OBSERVATION_SIDE)
        continue;
      double begin = 0.0, end = 0.0;
      if (!staticPlaneInterval(plane, current_durations, begin, end))
        continue;
      if (end <= begin)
        continue;
      sfc_active_times[plane_index].push_back(plane.piece_u_begin);
      sfc_active_times[plane_index].push_back(
          0.5 * (plane.piece_u_begin + plane.piece_u_end));
      sfc_active_times[plane_index].push_back(plane.piece_u_end);
    }
    int corridor_active_points_added = 0;
    int corridor_scp_iterations_run = 0;
    double corridor_continuous_violation = 0.0;
    if (static_sfc_plane_count > 0)
    {
      std::vector<CorridorViolationPoint> seed_worst;
      corridor_continuous_violation =
          continuousLocalSfcMaxViolation(jerkOpt_.getTraj(), &seed_worst);
      for (const CorridorViolationPoint &point : seed_worst)
      {
        const LocalSfcPlane &plane = candidate_local_sfc_planes_[point.plane_index];
        double begin = 0.0, end = 0.0;
        if (!staticPlaneInterval(plane, current_durations, begin, end))
          continue;
        const double u = plane.piece_u_begin +
            (point.time - begin) / current_durations(plane.piece_id);
        auto &times = sfc_active_times[point.plane_index];
        bool duplicate = false;
        for (const double existing : times)
          duplicate = duplicate ||
                      std::abs(existing - u) < 1.0e-3;
        if (!duplicate && times.size() < 12)
        {
          times.push_back(std::max(plane.piece_u_begin,
              std::min(plane.piece_u_end, u)));
          ++corridor_active_points_added;
        }
      }
      ROS_INFO(
          "[corridor-scp-active-set] candidate_id=%d phase=initial "
          "static_planes=%d initial_points=%zu seed_worst_points=%zu "
          "seed_continuous_violation=%.6f",
          scp_candidate_id, static_sfc_plane_count,
          [&]() {
            size_t total = 0;
            for (const auto &times : sfc_active_times)
              total += times.size();
            return total;
          }(),
          seed_worst.size(), corridor_continuous_violation);
    }
    if (local_sfc_plane_count > 0)
      ROS_INFO("[side-local-sfc] type=%s astar_points=NA guide_points=NA "
               "active_start=%.6f active_end=%.6f planes=%d "
               "astar_guide_inside=1 side_intersection_valid=1 "
               "initial_minco_static_free=%d result=ACTIVE",
               gradient_audit_candidate_.c_str(), window_start, window_end,
               local_sfc_plane_count,
               static_cast<int>(maxNativeStaticViolation(jerkOpt_.getTraj()) <= 1.0e-3));

    double native_cost_before = std::numeric_limits<double>::quiet_NaN();

    ROS_INFO("[scp-hard-corridor-time] candidate=%s initial_duration=%.6f "
             "virtual_time_dim=%d optimize_time=%d",
             gradient_audit_candidate_.c_str(), initial_durations.sum(),
             virtual_time_dim, static_cast<int>(optimize_time));

    const double initial_violation = maxViolation(jerkOpt_.getTraj());
    ROS_INFO("[scp-hard-corridor] candidate=%s obstacle_id=%d phase=initial "
             "constraint_samples=%d radius=%.4f max_violation=%.6f",
             gradient_audit_candidate_.c_str(), gradient_audit_obstacle_id_,
             sample_count, radius, initial_violation);

    bool accepted_any = false;
    bool saw_qp_success = false;
    bool saw_qp_failure = false;
    for (int iteration = 0;
         executionBudgetAvailable() &&
             iteration < std::max(
                 1, team_scp ? team_scp_max_iterations_
                             : candidate_hard_corridor_scp_max_iterations_);
         ++iteration)
    {
      // Failure classification is local to this major SCP iteration.  A
      // rejected trial must not leak its reason into a later accepted step or
      // into an unrelated retry exhaustion path.
      adaptive_failure_reason.clear();
      corridor_scp_iterations_run = iteration + 1;
      Eigen::VectorXd x(dim), gradient(dim);
      if (!refreshTimeState())
      {
        logFinalStatus("SCP_INVALID_DURATION", false, false,
                       jerkOpt_.getTraj(), maxViolation(jerkOpt_.getTraj()));
        return false;
      }
      Eigen::Map<Eigen::MatrixXd>(x.data(), 3, piece_num_ - 1) = current_points;
      if (optimize_time)
        x.segment(position_dim, virtual_time_dim) = current_virtual_t;
      const double current_native_cost =
          costFunctionCallback(this, x.data(), gradient.data(), dim);
      executionCheckpoint();
      if (iteration == 0)
        native_cost_before = current_native_cost;
      if (optimize_time)
      {
        const Eigen::VectorXd native_grad_t =
            gradient.segment(position_dim, virtual_time_dim);
        ROS_INFO("[scp-free-time-gradient] candidate_id=%d candidate_type=%s "
                 "iteration=%d grad_T_norm=%.6g grad_T_max_abs=%.6g "
                 "duration=%.6f",
                 scp_candidate_id, gradient_audit_candidate_.c_str(), iteration,
                 native_grad_t.norm(),
                 native_grad_t.size() > 0 ? native_grad_t.cwiseAbs().maxCoeff() : 0.0,
                 total_duration);
      }
      if (!refreshTimeState())
      {
        logFinalStatus("SCP_INVALID_DURATION", false, false,
                       jerkOpt_.getTraj(), maxViolation(jerkOpt_.getTraj()));
        return false;
      }
      if (!gradient.allFinite())
      {
        last_candidate_final_status_reason_ = "SCP_GRADIENT_INVALID";
        logFinalStatus("SCP_GRADIENT_INVALID", false, false, jerkOpt_.getTraj(),
                       maxViolation(jerkOpt_.getTraj()));
        return false;
      }

      std::vector<std::pair<int, int>> static_segments;
      const CHK_RET static_check =
          finelyCheckAndSetConstraintPoints(static_segments, jerkOpt_, true);
      if (static_check == CHK_RET::ERR)
      {
        last_candidate_final_status_reason_ = "STATIC_QP_FAILURE";
        ROS_WARN("[scp-static-failure] reason=STATIC_QP_FAILURE piece=-1 time=NA "
                 "static_violation=NA corridor_violation=%.6f trust_region=%.6f "
                 "solver_status=constraint_generation_failed",
                 maxViolation(jerkOpt_.getTraj()), trust_p);
        logFinalStatus("STATIC_QP_FAILURE", false, false, jerkOpt_.getTraj(),
                       maxViolation(jerkOpt_.getTraj()));
        return false;
      }

      // Build the linearized corridor.  Its rows are mapped to the MINCO
      // position variables through the native coefficient-adjoint path
      // (gdC -> getGrad2TP -> gradP), not by finite-differencing P.
      std::vector<Eigen::RowVectorXd> rows;
      std::vector<double> upper;
      int corridor_constraint_count = 0;
      const size_t corridor_rows_begin = rows.size();
      // No hard rows may be generated from candidate_preserve_reference_.

      // Local SFC rows preserve the sparse A* repair geometry over its active
      // time interval. They are linearized through the same analytic MINCO
      // position Jacobian as the SIDE corridor and native static rows.
      // Feedback117 (authority consolidation): LOS_OBSERVATION_SIDE planes
      // are a visibility-quality authority.  They keep guiding the solution
      // through enforceCandidateLosPlanesSCP()'s semi-hard slack QP and are
      // never allowed to re-enter this collision-corridor hard row set,
      // otherwise a primal-infeasible LOS row could still kill a candidate
      // through this dormant second authority (INVARIANT 1/2 of the
      // hard/geometry/visibility split).
      int los_hard_rows_skipped = 0;
      int local_sfc_constraint_count = 0;
      const size_t local_sfc_rows_begin = rows.size();
      for (size_t plane_index = 0;
           plane_index < candidate_local_sfc_planes_.size(); ++plane_index)
      {
        const LocalSfcPlane &raw_plane = candidate_local_sfc_planes_[plane_index];
        if (raw_plane.source == LocalSfcPlane::LOS_OBSERVATION_SIDE)
        {
          ++los_hard_rows_skipped;
          continue;
        }
        const double normal_norm = raw_plane.normal.norm();
        if (!std::isfinite(normal_norm) || normal_norm < 1.0e-6)
          continue;
        const Eigen::Vector3d normal = raw_plane.normal / normal_norm;
        double active_start = 0.0, active_end = 0.0;
        if (!staticPlaneInterval(raw_plane, current_durations,
                                 active_start, active_end))
          continue;
        if (active_end <= active_start + 1.0e-6)
          continue;
        // Feedback119(corridor active-set 协议):行只在 active-set 时间点
        // 上生成——初始为端点+中点+seed 连续违反 argmax,之后每轮 SCP 把
        // 当前迭代的真实违反点 t* 追加进 sfc_active_times(见下方
        // current_local_sfc_violation 处),下一轮生效。不再做 plane×25
        // 均匀暴力扩张。
        for (const double u : plane_index < sfc_active_times.size()
                                  ? sfc_active_times[plane_index]
                                  : std::vector<double>{})
        {
          if (u < raw_plane.piece_u_begin - 1.0e-9 ||
              u > raw_plane.piece_u_end + 1.0e-9)
            continue;
          const int piece_index = raw_plane.piece_id;
          const double piece_time = u * current_durations(piece_index);
          const double t = current_durations.head(piece_index).sum() + piece_time;
          if (t < active_start - 1.0e-9 || t > active_end + 1.0e-9)
            continue;
          const Eigen::Vector3d p = jerkOpt_.getTraj().getPos(t);
          const double h = raw_plane.clearance - normal.dot(p - raw_plane.point);
          Eigen::VectorXd ds_dT = Eigen::VectorXd::Zero(piece_num_);
          if (optimize_time)
            ds_dT(piece_index) = u;
          const Eigen::RowVectorXd grad_h =
              -mincoSampleGradientWrtX(piece_index, piece_time, normal,
                                       SAMPLE_POSITION, ds_dT);
          if (appendLinearUpperBound(grad_h, -h, rows, upper))
            ++local_sfc_constraint_count;
        }
      }

      // Zero-plane SIDE candidates still need a discrete-side authority when
      // Team PT is allowed to move P.  Reuse exactly the existing
      // candidateSideRegion frame and sinusoidal band; this is its hard-row
      // form, not a fourth topology representation.
      int side_region_constraint_count = 0;
      const size_t side_region_rows_begin = rows.size();
      if (team_scp && candidate_side_bias_enabled_ &&
          candidate_side_region_enabled_)
      {
        const int side_samples = std::max(5, sample_count);
        for (int sample_index = 0; sample_index < side_samples; ++sample_index)
        {
          const double progress = static_cast<double>(sample_index) /
                                  static_cast<double>(side_samples - 1);
          const double half_window = std::min(
              1.0, std::max(1.0e-3, candidate_side_guidance_window_));
          if (std::abs(progress - candidate_side_conflict_progress_) >=
              half_window)
            continue;
          const double t = progress * total_duration;
          int piece_index = -1;
          double piece_time = 0.0;
          if (!locatePieceTime(t, piece_index, piece_time))
            continue;
          const double desired = candidate_side_offset_ *
                                 std::sin(M_PI * progress);
          const double half_width = std::max(0.10, 0.25 * desired);
          const double lower_side = std::max(0.0, desired - half_width);
          const double upper_side = desired + half_width;
          const Eigen::Vector3d p = jerkOpt_.getTraj().getPos(t);
          const double signed_lateral = candidate_side_sign_ *
              (p - candidate_side_origin_).dot(candidate_side_direction_);
          Eigen::VectorXd ds_dT = Eigen::VectorXd::Zero(piece_num_);
          for (int duration_index = 0; duration_index < piece_num_;
               ++duration_index)
            ds_dT(duration_index) = progress -
                (duration_index < piece_index ? 1.0 : 0.0);
          const Eigen::RowVectorXd signed_row =
              mincoSampleGradientWrtX(
                  piece_index, piece_time,
                  candidate_side_sign_ * candidate_side_direction_,
                  SAMPLE_POSITION, ds_dT);
          if (appendLinearUpperBound(-signed_row,
                                     signed_lateral - lower_side,
                                     rows, upper))
            ++side_region_constraint_count;
          if (appendLinearUpperBound(signed_row,
                                     upper_side - signed_lateral,
                                     rows, upper))
            ++side_region_constraint_count;
        }
      }

      int team_visibility_constraint_count = 0;
      const size_t team_visibility_rows_begin = rows.size();
      double team_margin_current = std::numeric_limits<double>::infinity();
      Eigen::RowVectorXd critical_margin_row =
          Eigen::RowVectorXd::Zero(dim);
      Eigen::VectorXd team_soft_gradient = Eigen::VectorXd::Zero(dim);
      multi_uav_formation::ContinuousVisibilitySample::Limiter
          team_margin_limiter =
              multi_uav_formation::ContinuousVisibilitySample::LIMITER_NONE;
      if (team_scp)
      {
        const double relative_start =
            drone_id_ == team_visibility_contract_.incoming_uav
                ? std::max(0.0,
                      team_visibility_contract_.acquire_by_world_time -
                          team_visibility_contract_.activation_world_time)
                : 0.0;
        const double relative_end = std::min(
            total_duration,
            team_visibility_contract_.preserve_until_world_time -
                team_visibility_contract_.activation_world_time);
        if (relative_end < relative_start - 1.0e-9)
        {
          logFinalStatus("TEAM_CONTRACT_OUTSIDE_TRAJECTORY", false, false,
                         jerkOpt_.getTraj(), maxViolation(jerkOpt_.getTraj()));
          return false;
        }
        const int visibility_samples = std::max(
            2, static_cast<int>(std::ceil(
                   (relative_end - relative_start) / 0.05)) + 1);
        bool rows_valid = true;
        for (int sample_index = 0; sample_index < visibility_samples;
             ++sample_index)
        {
          const double alpha = visibility_samples > 1
                                   ? static_cast<double>(sample_index) /
                                         static_cast<double>(visibility_samples - 1)
                                   : 0.0;
          const double t = relative_start +
                           alpha * (relative_end - relative_start);
          int piece_index = -1;
          double piece_time = 0.0;
          if (!locatePieceTime(t, piece_index, piece_time))
          {
            rows_valid = false;
            team_scp_result_.linearization_invalid_world_time =
                team_visibility_contract_.activation_world_time + t;
            team_scp_result_.linearization_invalid_component =
                "LOCATE_PIECE_TIME";
            break;
          }
          const auto margin = teamMarginAt(jerkOpt_.getTraj(), t);
          if (!margin.margin_valid)
          {
            rows_valid = false;
            // Record the exact missing-gradient evidence (world time plus the
            // component whose continuous representation was unavailable) so
            // the failure can be diagnosed without guessing.
            team_scp_result_.linearization_invalid_world_time =
                team_visibility_contract_.activation_world_time + t;
            team_scp_result_.linearization_invalid_component =
                visibilityComponentName(team_margin_invalid_component_);
            break;
          }
          Eigen::VectorXd ds_dT = Eigen::VectorXd::Zero(piece_num_);
          // t is a fixed world-time sample.  Only preceding durations alter
          // its local piece time; there is no progress/time rescaling term.
          for (int duration_index = 0; duration_index < piece_num_;
               ++duration_index)
            ds_dT(duration_index) =
                duration_index < piece_index ? -1.0 : 0.0;
          const Eigen::RowVectorXd margin_row =
              mincoSampleGradientWrtX(
                  piece_index, piece_time,
                  margin.gradient_margin_position, SAMPLE_POSITION, ds_dT);
          if (appendLinearUpperBound(
                  -margin_row,
                  margin.margin - team_visibility_contract_.required_margin,
                  rows, upper))
            ++team_visibility_constraint_count;
          team_soft_gradient -=
              (team_scp_soft_weight_ /
               static_cast<double>(visibility_samples)) * margin_row.transpose();
          if (margin.margin < team_margin_current)
          {
            team_margin_current = margin.margin;
            critical_margin_row = margin_row;
            team_margin_limiter = margin.limiter;
          }
        }
        if (!rows_valid)
        {
          // Genuinely no continuous linearization: fail closed.
          logFinalStatus("TEAM_VISIBILITY_LINEARIZATION_INVALID", false,
                         false, jerkOpt_.getTraj(),
                         maxViolation(jerkOpt_.getTraj()));
          return false;
        }
        if (team_visibility_constraint_count <= 0)
        {
          // Every sampled point already satisfies the contracted margin, so no
          // visibility row was needed.  That is NOT a linearization failure:
          // the seed already discharges the obligation and a zero modification
          // is the correct refinement result.  Reporting this as invalid used
          // to reject the whole Team reference.  All downstream exact nonlinear
          // validation still runs, so nothing is bypassed.
          team_scp_result_.success = true;
          team_scp_result_.reason = "TEAM_VISIBILITY_ALREADY_SATISFIED";
          team_scp_result_.visibility_rows = 0;
          // Feedback098: this early return happens before the iteration-0 block
          // that normally records margin_before, so without this the trace
          // reported m2_before=-inf for a seed whose margin was in fact known.
          team_scp_result_.margin_before = team_margin_current;
          team_scp_result_.margin_after = team_margin_current;
          team_scp_result_.linear_margin_prediction = team_margin_current;
          ROS_INFO("[TEAM_VISIBILITY_ALREADY_SATISFIED] drone_id=%d "
                   "contract_id=%lu window=[%.9f,%.9f] margin_min=%.6f "
                   "required=%.6f action=ACCEPT_SEED_UNCHANGED",
              drone_id_,
              static_cast<unsigned long>(team_visibility_contract_.contract_id),
              team_visibility_contract_.acquire_by_world_time,
              team_visibility_contract_.preserve_until_world_time,
              team_margin_current,
              team_visibility_contract_.required_margin);
          return true;
        }

        if (iteration == 0)
        {
          const double deficit = std::max(
              0.0, team_visibility_contract_.required_margin -
                       team_margin_current);
          double t_gain_bound = 0.0;
          double t_gradient_norm = 0.0;
          for (int time_index = 0; time_index < virtual_time_dim;
               ++time_index)
          {
            const double jacobian = std::abs(
                realTimeJacobianAt(current_virtual_t(time_index)));
            const double physical_trust = team_scp_t_trust_ratio_ *
                                          current_durations(time_index);
            const double virtual_bound = jacobian > 1.0e-9
                ? physical_trust / jacobian : 0.0;
            const double derivative =
                critical_margin_row(position_dim + time_index);
            t_gain_bound += std::abs(derivative) * virtual_bound;
            t_gradient_norm += derivative * derivative;
          }
          t_gradient_norm = std::sqrt(t_gradient_norm);
          TeamSCPMode mode = team_visibility_contract_.requested_mode;
          if (mode == TeamSCPMode::AUTO)
          {
            mode = t_gradient_norm > 1.0e-8 &&
                           deficit <= team_scp_t_gain_ratio_ * t_gain_bound
                       ? TeamSCPMode::TEAM_T_ONLY
                       : TeamSCPMode::TEAM_PT;
          }
          team_scp_result_.mode = mode;
          team_scp_result_.margin_before = team_margin_current;
          ROS_INFO("[TEAM_SCP_MODE=%s] contract_id=%lu drone=%d "
                   "deficit=%.9f grad_T_norm=%.9f T_gain_bound=%.9f "
                   "gain_ratio=%.3f limiter=%s "
                   "TEAM_REFERENCE_PHASE_USED_FOR_HANDOFF=0",
                   mode == TeamSCPMode::TEAM_T_ONLY ? "T" : "PT",
                   static_cast<unsigned long>(
                       team_visibility_contract_.contract_id),
                   drone_id_, deficit, t_gradient_norm, t_gain_bound,
                   team_scp_t_gain_ratio_,
                   multi_uav_formation::visibilityMarginLimiterName(
                       team_margin_limiter));
        }
        team_scp_result_.visibility_rows =
            team_visibility_constraint_count;
        // Team SCP is a minimum-deformation projection.  The native Local
        // objective already produced this feasible realization and does not
        // regain planner authority here; only the contract soft descent is
        // kept as a secondary linear merit.
        gradient = team_soft_gradient;

        // TEAM_T_ONLY is a real subspace, not a tiny P trust box.  Equality is
        // represented by the same upper-bound row adapter in both signs, so
        // adaptive trust probes cannot accidentally unlock P.
        if (team_scp_result_.mode == TeamSCPMode::TEAM_T_ONLY)
        {
          for (int column = 0; column < position_dim; ++column)
          {
            Eigen::RowVectorXd lock = Eigen::RowVectorXd::Zero(dim);
            lock(column) = 1.0;
            appendLinearUpperBound(lock, 0.0, rows, upper);
            appendLinearUpperBound(-lock, 0.0, rows, upper);
          }
        }
      }

      const size_t team_visibility_rows_end = rows.size();
      // Add the native MINCO separating constraints.  cps_ was refreshed at
      // the current iterate above, so these sparse rows follow exactly the
      // same base points, directions, and clearance convention as the
      // original obstacle cost/checker.
      int static_constraint_count = 0;
      const size_t static_rows_begin = rows.size();
      for (int cp = 0; cp < cps_.cp_size; ++cp)
      {
        const int piece = std::min(cp / cps_num_prePiece_, piece_num_ - 1);
        const int local_index = cp % cps_num_prePiece_;
        double sample_time = 0.0;
        for (int i = 0; i < piece; ++i)
          sample_time += current_durations(i);
        sample_time += static_cast<double>(local_index) /
                       static_cast<double>(cps_num_prePiece_) * current_durations(piece);
        if (cp == cps_.cp_size - 1)
          sample_time = total_duration;
        int located_piece = piece;
        double piece_time = 0.0;
        if (!locatePieceTime(sample_time, located_piece, piece_time))
          continue;
        const Eigen::Vector3d p = jerkOpt_.getTraj().getPos(sample_time);
        for (size_t j = 0; j < cps_.direction[cp].size(); ++j)
        {
          const Eigen::Vector3d &normal = cps_.direction[cp][j];
          const Eigen::Vector3d &base = cps_.base_point[cp][j];
          if (!normal.allFinite() || !base.allFinite() || !p.allFinite())
            continue;
          const double dist = (p - base).dot(normal);
          Eigen::VectorXd ds_dT = Eigen::VectorXd::Zero(piece_num_);
          if (optimize_time)
          {
            const double local_alpha =
                cp == cps_.cp_size - 1
                    ? 1.0
                    : static_cast<double>(local_index) /
                          static_cast<double>(cps_num_prePiece_);
            ds_dT(located_piece) = local_alpha;
          }
          const Eigen::RowVectorXd grad_dist =
              mincoSampleGradientWrtX(located_piece, piece_time, normal,
                                       SAMPLE_POSITION, ds_dT);
          // dist + grad_dist * dP >= clearance  <=>
          // -grad_dist * dP <= dist - clearance.
          if (appendLinearUpperBound(-grad_dist, dist - obs_clearance_, rows,
                                     upper))
            ++static_constraint_count;
        }
      }

      // Dynamic body clearance is a repair constraint, not an execution
      // admission shortcut.  Linearize the same center-distance predicate
      // used by checkMovingObjSafety() at the current MINCO iterate.  The
      // position Jacobian comes from MINCO; the obstacle velocity term is
      // added explicitly so virtual-T steps move the query in world time as
      // well as moving the trajectory along its local time axis.
      int dynamic_constraint_count = 0;
      const size_t dynamic_rows_begin = rows.size();
      if (use_time_aware_moving_obj_cost_ && moving_objs_ &&
          moving_objs_->getObjNums() > 0)
      {
        const double dynamic_horizon =
            moving_obj_prediction_horizon_ > 1.0e-3
                ? std::min(total_duration, moving_obj_prediction_horizon_)
                : total_duration;
        const double activation_margin =
            std::max(0.0, elastic_moving_risk_margin_);
        for (int sample = 0; sample < sample_count; ++sample)
        {
          const double alpha = static_cast<double>(sample) /
                               static_cast<double>(sample_count - 1);
          const double t = alpha * dynamic_horizon;
          int piece_index = -1;
          double piece_time = 0.0;
          if (!locatePieceTime(t, piece_index, piece_time))
            continue;
          const Eigen::Vector3d p = jerkOpt_.getTraj().getPos(t);
          if (!p.allFinite())
            continue;
          const double dt_dT =
              (moving_obj_prediction_horizon_ <= 1.0e-3 ||
               total_duration < moving_obj_prediction_horizon_)
                  ? alpha : 0.0;
          Eigen::VectorXd ds_dT = Eigen::VectorXd::Zero(piece_num_);
          if (optimize_time)
          {
            for (int duration_index = 0; duration_index < piece_num_; ++duration_index)
              ds_dT(duration_index) = dt_dT -
                                      (duration_index < piece_index ? 1.0 : 0.0);
          }
          const double query_time = t_now_ + t;
          for (int id = 0; id < moving_objs_->getObjNums(); ++id)
          {
            if (!moving_objs_->hasPrediction(id))
              continue;
            const Eigen::Vector3d obstacle =
                moving_objs_->evaluateConstVel(id, query_time);
            const Eigen::Vector3d obstacle_velocity =
                moving_objs_->evaluateConstVelVelocity(id, query_time);
            if (!obstacle.allFinite() || !obstacle_velocity.allFinite())
              continue;
            const Eigen::Vector3d delta = p - obstacle;
            const double distance = delta.norm();
            if (!std::isfinite(distance) || distance <= 1.0e-6 ||
                distance > moving_obj_clearance_ + activation_margin)
              continue;
            const Eigen::Vector3d normal = delta / distance;
            Eigen::RowVectorXd grad_distance = mincoSampleGradientWrtX(
                piece_index, piece_time, normal, SAMPLE_POSITION, ds_dT);
            if (optimize_time)
            {
              const double obstacle_time_derivative =
                  -normal.dot(obstacle_velocity);
              for (int duration_index = 0; duration_index < piece_num_; ++duration_index)
                grad_distance(position_dim + duration_index) +=
                    obstacle_time_derivative * dt_dT *
                    realTimeJacobianAt(current_virtual_t(duration_index));
            }
            // Enforce only physical contact avoidance in the hard row.  The
            // 1.1 m value remains the preferred-margin objective/trigger.
            // Feedback126: per-object clearance (this object's live radius +
            // body radius) — the same predicate the final preflight uses,
            // never the fleet-max approximation.
            const double clearance_margin =
                distance - movingObjHardClearanceForObject(id);
            if (appendLinearUpperBound(-grad_distance, clearance_margin,
                                       rows, upper))
              ++dynamic_constraint_count;
          }
        }
      }

      /* Feedback126 §2A/§4: swarm hard rows from the unified physical
       * predicate (elliptical dxy^2 + 0.25*dz^2 >= swarm_clearance_^2,
       * polynomial-then-hold@end peer model, world time at the batch epoch).
       * §5 ownership: rows are appended only for ADJUSTER-role peers —
       * against an OWNER-role peer this drone must not add opposite
       * avoidance.  With no ownership information every violating peer gets
       * rows (conservative default). */
      int swarm_constraint_count = 0;
      if (swarm_trajs_ != nullptr && !swarm_trajs_->empty())
      {
        const double swarm_horizon =
            moving_obj_prediction_horizon_ > 1.0e-3
                ? std::min(total_duration, moving_obj_prediction_horizon_)
                : total_duration;
        const double swarm_activation_margin = 0.30;
        for (int sample = 0; sample < sample_count; ++sample)
        {
          const double alpha = static_cast<double>(sample) /
                               static_cast<double>(sample_count - 1);
          const double t = alpha * swarm_horizon;
          int piece_index = -1;
          double piece_time = 0.0;
          if (!locatePieceTime(t, piece_index, piece_time))
            continue;
          const Eigen::Vector3d p = jerkOpt_.getTraj().getPos(t);
          if (!p.allFinite())
            continue;
          const double dt_dT =
              (moving_obj_prediction_horizon_ <= 1.0e-3 ||
               total_duration < moving_obj_prediction_horizon_)
                  ? alpha : 0.0;
          Eigen::VectorXd ds_dT = Eigen::VectorXd::Zero(piece_num_);
          if (optimize_time)
          {
            for (int duration_index = 0; duration_index < piece_num_; ++duration_index)
              ds_dT(duration_index) = dt_dT -
                                      (duration_index < piece_index ? 1.0 : 0.0);
          }
          const double query_time = t_now_ + t;
          for (size_t id = 0; id < swarm_trajs_->size(); ++id)
          {
            const LocalTrajData &peer =
                swarm_trajs_->at(id).executionAt(query_time);
            if (peer.drone_id < 0 || peer.drone_id == drone_id_ ||
                peer.duration <= 0.0 || peer.traj.getPieceNum() <= 0)
              continue;
            const std::vector<int> &adjusters = swarm_adjuster_peers_;
            if (!adjusters.empty() &&
                std::find(adjusters.begin(), adjusters.end(),
                          peer.drone_id) == adjusters.end())
              continue;  // OWNER-role peer: no opposite-avoidance rows.
            const double peer_t = query_time - peer.start_time;
            const Eigen::Vector3d peer_pos =
                peer.traj.getPos(std::max(0.0, std::min(peer_t, peer.duration)));
            const Eigen::Vector3d peer_vel =
                peer_t < peer.duration
                    ? peer.traj.getVel(std::max(0.0, peer_t))
                    : Eigen::Vector3d::Zero();
            if (!peer_pos.allFinite())
              continue;
            const Eigen::Vector3d delta = p - peer_pos;
            // Linearize the elliptical separation around the current
            // iterate: dist_ellip(p) ≈ dist_ellip(p0) + grad·Δp with
            // grad = (dx, dy, 0.25*dz) / dist_ellip.  Row:
            // dist_ellip + grad·Δp >= swarm_clearance_.
            const double dist_ellip =
                std::sqrt(delta.head<2>().squaredNorm() +
                          0.25 * delta(2) * delta(2));
            if (!std::isfinite(dist_ellip) || dist_ellip <= 1.0e-6 ||
                dist_ellip > swarm_clearance_ + swarm_activation_margin)
              continue;
            Eigen::Vector3d grad_ellip(
                delta(0) / dist_ellip, delta(1) / dist_ellip,
                0.25 * delta(2) / dist_ellip);
            Eigen::RowVectorXd grad_distance = mincoSampleGradientWrtX(
                piece_index, piece_time, grad_ellip, SAMPLE_POSITION, ds_dT);
            if (optimize_time)
            {
              // World-time motion of the peer shifts the query point; the
              // separation derivative w.r.t. the world time of the sample is
              // -grad_ellip · peer_velocity (same convention as the dynamic
              // body rows above).
              for (int duration_index = 0; duration_index < piece_num_; ++duration_index)
                grad_distance(position_dim + duration_index) +=
                    -grad_ellip.dot(peer_vel) * dt_dT *
                    realTimeJacobianAt(current_virtual_t(duration_index));
            }
            const double clearance_margin = dist_ellip - swarm_clearance_;
            if (appendLinearUpperBound(-grad_distance, clearance_margin,
                                       rows, upper))
              ++swarm_constraint_count;
          }
        }
      }
      if (swarm_constraint_count > 0 && iteration == 0)
      {
        ROS_INFO("[SCP_SWARM_HARD_ROWS] candidate_type=%s drone=%d "
                 "swarm_rows=%d adjuster_peers=%zu",
                 gradient_audit_candidate_.c_str(), drone_id_,
                 swarm_constraint_count, swarm_adjuster_peers_.size());
      }

      const ScpDynamicsSummary current_dynamics =
          appendDynamicsConstraints(&rows, &upper);
      const size_t dynamics_rows_begin = rows.size() -
                                         static_cast<size_t>(current_dynamics.constraint_count);

      const auto maxAbsTimeJacobian = [&](const size_t begin, const size_t end) {
        double maximum = 0.0;
        if (!optimize_time || begin >= end || end > rows.size())
          return maximum;
        for (size_t row_index = begin; row_index < end; ++row_index)
          maximum = std::max(maximum,
                             rows[row_index].segment(position_dim, virtual_time_dim).cwiseAbs().maxCoeff());
        return maximum;
      };
      const double corridor_max_jt =
          maxAbsTimeJacobian(corridor_rows_begin, local_sfc_rows_begin);
      const double local_sfc_max_jt =
          maxAbsTimeJacobian(local_sfc_rows_begin, static_rows_begin);
      const double static_max_jt =
          maxAbsTimeJacobian(static_rows_begin, dynamic_rows_begin);
      const double dynamic_max_jt =
          maxAbsTimeJacobian(dynamic_rows_begin, dynamics_rows_begin);
      const double velocity_max_jt =
          maxAbsTimeJacobian(dynamics_rows_begin,
                             dynamics_rows_begin + current_dynamics.velocity_constraint_count);
      const double acceleration_max_jt =
          maxAbsTimeJacobian(dynamics_rows_begin + current_dynamics.velocity_constraint_count,
                             dynamics_rows_begin + current_dynamics.velocity_constraint_count +
                                 current_dynamics.acceleration_constraint_count);
      const double jerk_max_jt =
          maxAbsTimeJacobian(dynamics_rows_begin + current_dynamics.velocity_constraint_count +
                                 current_dynamics.acceleration_constraint_count,
                             rows.size());

      // Diagnostic-only feasibility decomposition.  This deliberately reuses
      // the rows built above and the production OSQP adapter, but replaces the
      // native objective with a tiny positive quadratic.  It therefore answers
      // which constraint group first removes a feasible dx=[dP,dVirtualT]
      // without changing the production SCP step or its trust-region policy.
      // Production adaptive-trust probes use the exact full hard-row set
      // assembled above.  They differ only in whether the P and/or virtual-T
      // box rows are present; no safety row is removed from these probes.
      const auto runAdaptiveProbe = [&](const bool add_p_trust,
                                        const bool add_t_trust,
                                        const char *probe_name) {
        const int trust_rows = (add_p_trust ? position_dim : 0) +
                               (add_t_trust ? virtual_time_dim : 0);
        const size_t total_rows = rows.size() + static_cast<size_t>(trust_rows);
        SCPQPSolveResult result;
        if (rows.empty())
        {
          result.status_text = "no_hard_rows";
          ROS_INFO("[adaptive-tr-probe] candidate_id=%d iteration=%d probe=%s "
                   "status=%s feasible=0 p_trust=%.6f t_trust=%.6f",
                   scp_candidate_id, iteration, probe_name,
                   result.status_text.c_str(), trust_p, trust_t);
          return result;
        }
        Eigen::MatrixXd probe_A(static_cast<int>(total_rows), dim);
        Eigen::VectorXd probe_lower(static_cast<int>(total_rows));
        Eigen::VectorXd probe_upper(static_cast<int>(total_rows));
        for (size_t row = 0; row < rows.size(); ++row)
        {
          probe_A.row(static_cast<int>(row)) = rows[row];
          probe_lower(static_cast<int>(row)) = -1.0e20;
          probe_upper(static_cast<int>(row)) = upper[row];
        }
        size_t output_row = rows.size();
        for (int column = 0; column < dim; ++column)
        {
          const bool is_time = optimize_time && column >= position_dim;
          const bool constrained = (is_time && add_t_trust) ||
                                   (!is_time && add_p_trust);
          if (!constrained)
            continue;
          probe_A.row(static_cast<int>(output_row)).setZero();
          double limit = trust_p;
          if (is_time)
          {
            const int time_index = column - position_dim;
            probe_A(static_cast<int>(output_row), column) =
                realTimeJacobianAt(current_virtual_t(time_index));
            limit = trust_t * current_durations(time_index);
          }
          else
          {
            probe_A(static_cast<int>(output_row), column) = 1.0;
          }
          probe_lower(static_cast<int>(output_row)) = -limit;
          probe_upper(static_cast<int>(output_row)) = limit;
          ++output_row;
        }
        const Eigen::VectorXd zero_gradient = Eigen::VectorXd::Zero(dim);
        result = solveExecutionQP(zero_gradient, probe_A, probe_lower,
                                      probe_upper, Eigen::VectorXd::Ones(dim));
        ROS_INFO("[adaptive-tr-probe] candidate_id=%d iteration=%d probe=%s "
                 "status=%s feasible=%d p_trust=%d t_trust=%d p_radius=%.6f "
                 "t_radius=%.6f rows=%zu",
                 scp_candidate_id, iteration, probe_name,
                 result.status_text.c_str(), static_cast<int>(result.success),
                 static_cast<int>(add_p_trust), static_cast<int>(add_t_trust),
                 trust_p, trust_t, total_rows);
        return result;
      };

      if (los_hard_rows_skipped > 0 && iteration == 0)
      {
        // Visibility authority can only be soft (Feedback117): record every
        // LOS plane excluded from the collision hard row set.
        ROS_INFO("[los-soft-authority] candidate_id=%d candidate_type=%s "
                 "los_planes_excluded_from_hard_rows=%d "
                 "owner=SEMI_HARD_LOS_SLACK_QP",
                 scp_candidate_id, gradient_audit_candidate_.c_str(),
                 los_hard_rows_skipped);
      }

      if (!candidate_local_sfc_planes_.empty() && iteration == 0)
      {
        ROS_INFO("[local-sfc-free-time] candidate_id=%d candidate_type=%s piece_num=%d "
                 "P_dim=%d virtual_T_dim=%d total_dim=%d duration_before=%.6f "
                 "duration_current=%.6f side_rows=%d side_max_JT=%.6g "
                 "sfc_rows=%d sfc_max_JT=%.6g static_rows=%d static_max_JT=%.6g "
                 "dynamic_rows=%zu dynamic_max_JT=%.6g "
                 "v_rows=%d v_max_JT=%.6g a_rows=%d a_max_JT=%.6g "
                 "j_rows=%d j_max_JT=%.6g native_cost_before=%.6g "
                 "time_trust_alpha=%.6g median_real_time_trust=%.6g",
                 scp_candidate_id, gradient_audit_candidate_.c_str(), piece_num_,
                 position_dim, virtual_time_dim, dim, initial_durations.sum(),
                 total_duration, corridor_constraint_count, corridor_max_jt,
                 local_sfc_constraint_count, local_sfc_max_jt, static_constraint_count,
                 static_max_jt, dynamics_rows_begin - dynamic_rows_begin,
                 dynamic_max_jt, current_dynamics.velocity_constraint_count,
                 velocity_max_jt, current_dynamics.acceleration_constraint_count,
                 acceleration_max_jt, current_dynamics.jerk_constraint_count,
                 jerk_max_jt, current_native_cost, trust_t,
                 trust_t * current_durations.size() > 0
                     ? trust_t * current_durations.mean()
                     : 0.0);
      }

      if (rows.empty())
      {
        last_candidate_final_status_reason_ = "SCP_LINEARIZATION_EMPTY";
        logFinalStatus("SCP_LINEARIZATION_EMPTY", false, false, jerkOpt_.getTraj(),
                       maxViolation(jerkOpt_.getTraj()));
        return false;
      }
      Eigen::MatrixXd A(static_cast<int>(rows.size()) + dim, dim);
      Eigen::VectorXd lower(A.rows()), upper_bound(A.rows());
      for (size_t row = 0; row < rows.size(); ++row)
      {
        A.row(static_cast<int>(row)) = rows[row];
        lower(static_cast<int>(row)) = -1.0e20;
        upper_bound(static_cast<int>(row)) = upper[row];
      }
      for (int column = 0; column < dim; ++column)
      {
        A.row(static_cast<int>(rows.size()) + column).setZero();
        double column_trust = trust_p;
        if (optimize_time && column >= position_dim)
        {
          const int time_index = column - position_dim;
          A(static_cast<int>(rows.size()) + column, column) =
              realTimeJacobianAt(current_virtual_t(time_index));
          column_trust = trust_t * current_durations(time_index);
        }
        else
        {
          A(static_cast<int>(rows.size()) + column, column) = 1.0;
        }
        lower(static_cast<int>(rows.size()) + column) = -column_trust;
        upper_bound(static_cast<int>(rows.size()) + column) = column_trust;
      }

      if (iteration == 0)
      {
        const auto percentile = [](std::vector<double> values,
                                   const double quantile) {
          if (values.empty())
            return 0.0;
          std::sort(values.begin(), values.end());
          const double location =
              quantile * static_cast<double>(values.size() - 1);
          const size_t lower_index = static_cast<size_t>(location);
          const size_t upper_index = std::min(values.size() - 1, lower_index + 1);
          const double fraction = location - static_cast<double>(lower_index);
          return values[lower_index] * (1.0 - fraction) +
                 values[upper_index] * fraction;
        };
        std::vector<double> row_norms;
        std::vector<double> p_column_norms(static_cast<size_t>(position_dim), 0.0);
        std::vector<double> t_column_norms(static_cast<size_t>(virtual_time_dim), 0.0);
        row_norms.reserve(rows.size());
        for (const Eigen::RowVectorXd &row : rows)
        {
          row_norms.push_back(row.norm());
          for (int column = 0; column < position_dim; ++column)
            p_column_norms[static_cast<size_t>(column)] = std::max(
                p_column_norms[static_cast<size_t>(column)],
                std::abs(row(column)));
          for (int column = 0; column < virtual_time_dim; ++column)
            t_column_norms[static_cast<size_t>(column)] = std::max(
                t_column_norms[static_cast<size_t>(column)],
                std::abs(row(position_dim + column)));
        }
        std::vector<double> grad_p_abs;
        std::vector<double> grad_t_abs;
        grad_p_abs.reserve(static_cast<size_t>(position_dim));
        grad_t_abs.reserve(static_cast<size_t>(virtual_time_dim));
        for (int column = 0; column < position_dim; ++column)
          grad_p_abs.push_back(std::abs(gradient(column)));
        for (int column = 0; column < virtual_time_dim; ++column)
          grad_t_abs.push_back(std::abs(gradient(position_dim + column)));
        std::vector<double> durations;
        durations.reserve(static_cast<size_t>(current_durations.size()));
        for (int index = 0; index < current_durations.size(); ++index)
          durations.push_back(current_durations(index));
        ROS_INFO(
            "[scp-pt-scale] candidate_id=%d iteration=%d P_value_max=%.6g "
            "virtual_T_value_max=%.6g duration_median=%.6g "
            "gradP_abs_median=%.6g gradP_abs_p95=%.6g gradP_abs_max=%.6g "
            "gradVT_abs_median=%.6g gradVT_abs_p95=%.6g gradVT_abs_max=%.6g "
            "row_norm_median=%.6g row_norm_p95=%.6g row_norm_max=%.6g "
            "P_col_abs_max=%.6g T_col_abs_max=%.6g "
            "time_trust_alpha=%.6g physical_time_trust_median=%.6g",
            scp_candidate_id, iteration,
            current_points.size() > 0 ? current_points.cwiseAbs().maxCoeff() : 0.0,
            current_virtual_t.size() > 0 ? current_virtual_t.cwiseAbs().maxCoeff() : 0.0,
            percentile(durations, 0.5), percentile(grad_p_abs, 0.5),
            percentile(grad_p_abs, 0.95), percentile(grad_p_abs, 1.0),
            percentile(grad_t_abs, 0.5), percentile(grad_t_abs, 0.95),
            percentile(grad_t_abs, 1.0), percentile(row_norms, 0.5),
            percentile(row_norms, 0.95), percentile(row_norms, 1.0),
            percentile(p_column_norms, 1.0), percentile(t_column_norms, 1.0),
            trust_t, trust_t * percentile(durations, 0.5));
      }

      bool accepted = false;
      Eigen::MatrixXd trial_points = current_points;
      double trial_violation = std::numeric_limits<double>::infinity();
      double trial_local_sfc_violation = std::numeric_limits<double>::infinity();
      double trial_static_violation = std::numeric_limits<double>::infinity();
      bool trial_static_free = false;
      ScpDynamicsSummary trial_dynamics;
      const double current_violation = maxViolation(jerkOpt_.getTraj());
      const double current_dynamic_violation =
          maxDynamicBodyViolation(jerkOpt_.getTraj());
      const double current_swarm_violation =
          maxSwarmViolation(jerkOpt_.getTraj());
      // Feedback119: continuous corridor violation on the current iterate.
      // Any real violating point t* is appended to the plane's active set and
      // enters the hard row set on the NEXT SCP round ("加入下一轮 SCP").
      double corridor_iteration_violation_before = 0.0;
      if (static_sfc_plane_count > 0)
      {
        std::vector<CorridorViolationPoint> iterate_worst;
        corridor_iteration_violation_before = continuousLocalSfcMaxViolation(
            jerkOpt_.getTraj(), &iterate_worst);
        int added_this_iteration = 0;
        for (const CorridorViolationPoint &point : iterate_worst)
        {
          const LocalSfcPlane &plane = candidate_local_sfc_planes_[point.plane_index];
          double begin = 0.0, end = 0.0;
          if (!staticPlaneInterval(plane, current_durations, begin, end))
            continue;
          const double u = plane.piece_u_begin +
              (point.time - begin) / current_durations(plane.piece_id);
          auto &times = sfc_active_times[point.plane_index];
          bool duplicate = false;
          for (const double existing : times)
            duplicate = duplicate || std::abs(existing - u) < 1.0e-3;
          if (!duplicate && times.size() < 12)
          {
            times.push_back(std::max(plane.piece_u_begin,
                std::min(plane.piece_u_end, u)));
            ++added_this_iteration;
            ++corridor_active_points_added;
          }
        }
        if (added_this_iteration > 0 || iteration == 0)
          ROS_INFO("[corridor-scp-active-set] candidate_id=%d iteration=%d "
                   "added=%d total_points=%zu continuous_violation=%.6f",
                   scp_candidate_id, iteration, added_this_iteration,
                   [&]() {
                     size_t total = 0;
                     for (const auto &times : sfc_active_times)
                       total += times.size();
                     return total;
                   }(),
                   corridor_iteration_violation_before);
      }
      const double current_local_sfc_violation =
          static_sfc_plane_count > 0
              ? continuousLocalSfcMaxViolation(jerkOpt_.getTraj(), nullptr)
              : maxLocalSfcViolation(jerkOpt_.getTraj());
      const double current_static_violation =
          maxNativeStaticViolation(jerkOpt_.getTraj());
      const double current_team_margin = team_scp
          ? minimumTeamMargin(jerkOpt_.getTraj())
          : std::numeric_limits<double>::infinity();
      const double current_team_side_violation =
          maxTeamSideViolation(jerkOpt_.getTraj());
      const ScpDynamicsSummary current_dynamics_snapshot = current_dynamics;
      const Eigen::MatrixXd base_points = current_points;
      const Eigen::VectorXd base_virtual_t = current_virtual_t;
      const Eigen::VectorXd base_durations = current_durations;
      const double base_total_duration = total_duration;
      std::string last_qp_status = "unknown";
      int accepted_attempt = -1;
      ScpDynamicsSummary accepted_predicted_dynamics;
      ScpDynamicsSummary accepted_trial_dynamics;
      const auto shrinkAgreementTrust = [&](const bool shrink_p,
                                            const bool shrink_t) {
        bool changed = false;
        if (trust_shrink_attempts < kMaxTrustShrinkAttempts && shrink_p &&
            trust_p > trust_p_min + 1.0e-9)
        {
          trust_p = std::max(trust_p_min, trust_p * kTrustShrinkFactor);
          changed = true;
        }
        if (trust_shrink_attempts < kMaxTrustShrinkAttempts && shrink_t &&
            optimize_time && trust_t > trust_t_min + 1.0e-9)
        {
          trust_t = std::max(trust_t_min, trust_t * kTrustShrinkFactor);
          changed = true;
        }
        if (changed)
          ++trust_shrink_attempts;
        return changed;
      };
      for (int attempt = 0; attempt < 5 && !accepted && executionBudgetAvailable(); ++attempt)
      {
        for (int column = 0; column < dim; ++column)
        {
          const double column_trust =
              optimize_time && column >= position_dim
              ? trust_t * current_durations(column - position_dim)
                  : trust_p;
          lower(static_cast<int>(rows.size()) + column) = -column_trust;
          upper_bound(static_cast<int>(rows.size()) + column) = column_trust;
        }
        Eigen::VectorXd hessian = Eigen::VectorXd::Ones(dim);
        if (optimize_time)
          hessian.segment(position_dim, virtual_time_dim).setConstant(4.0);
        const SCPQPSolveResult qp = solveExecutionQP(
            gradient, A, lower, upper_bound, hessian);
        last_qp_status = qp.status_text;
        ROS_INFO("[scp-hard-corridor] iteration=%d constraint_num=%d "
                 "corridor_constraints=%d local_sfc_constraints=%d static_constraints=%d dynamics_constraints=%d "
                 "max_violation=%.6f dyn_vel_violation=%.6f "
                 "dyn_acc_violation=%.6f dyn_jerk_violation=%.6f "
                 "trust_region=%.6f time_trust_region=%.6f solver_status=%s",
                 iteration, static_cast<int>(rows.size()),
                 corridor_constraint_count,
                 local_sfc_constraint_count,
                 static_constraint_count,
                 current_dynamics.constraint_count,
                 current_violation,
                 current_dynamics.max_vel_violation,
                 current_dynamics.max_acc_violation,
                 current_dynamics.max_jerk_violation,
                 trust_p, trust_t,
                 qp.status_text.c_str());
        ROS_INFO("[scp-osqp-diagnostics] candidate_id=%d iteration=%d "
                 "status=%s iter=%d pri_res=%.6g dua_res=%.6g "
                 "rho_updates=%d rho_estimate=%.6g",
                 scp_candidate_id, iteration, qp.status_text.c_str(),
                 qp.iterations, qp.primal_residual, qp.dual_residual,
                 qp.rho_updates, qp.rho_estimate);
        ROS_INFO("[scp-free-time-step] candidate_id=%d candidate_type=%s "
                 "iteration=%d attempt=%d solver_status=%s qp_step_T_max_abs=%.6g "
                 "qp_step_P_max_abs=%.6g",
                 scp_candidate_id, gradient_audit_candidate_.c_str(), iteration,
                 attempt, qp.status_text.c_str(),
                 (qp.success && optimize_time && qp.step.size() == dim &&
                  virtual_time_dim > 0)
                     ? qp.step.segment(position_dim, virtual_time_dim)
                           .cwiseAbs()
                           .maxCoeff()
                     : 0.0,
                 (qp.success && qp.step.size() == dim && position_dim > 0)
                     ? qp.step.head(position_dim).cwiseAbs().maxCoeff()
                     : 0.0);
        if (!qp.success || qp.step.size() != dim)
        {
          saw_qp_failure = true;
          const double trust_p_before = trust_p;
          const double trust_t_before = trust_t;
          int probe_no_p_tr = -1;
          int probe_no_t_tr = -1;
          int probe_no_pt_tr = -1;
          std::string decision = "KEEP";
          const bool primal_infeasible =
              qp.status_text.find("primal_infeasible") != std::string::npos;
          if (primal_infeasible)
          {
            // A primal-infeasible full box does not imply that the hard
            // constraints are infeasible.  Remove one trust block at a time,
            // using the same rows, before changing either radius.
            const SCPQPSolveResult no_p_probe =
                runAdaptiveProbe(false, optimize_time, "NO_P_TRUST");
            probe_no_p_tr = static_cast<int>(no_p_probe.success);
            if (no_p_probe.success)
            {
              if (trust_p < trust_p_max - 1.0e-9 &&
                  trust_expand_attempts < kMaxTrustExpandAttempts)
              {
                trust_p = std::min(trust_p_max, trust_p * kTrustGrowFactor);
                ++trust_expand_attempts;
                decision = "P_GROW";
                adaptive_failure_reason = "P_TRUST_TOO_SMALL";
              }
              else
              {
                decision = "TR_MAX_REACHED";
                adaptive_failure_reason = "P_TRUST_MAX_REACHED";
              }
            }
            else
            {
              const SCPQPSolveResult no_t_probe =
                  runAdaptiveProbe(true, false, "NO_T_TRUST");
              probe_no_t_tr = static_cast<int>(no_t_probe.success);
              if (no_t_probe.success)
              {
                if (optimize_time && trust_t < trust_t_max - 1.0e-9 &&
                    trust_expand_attempts < kMaxTrustExpandAttempts)
                {
                  trust_t = std::min(trust_t_max, trust_t * kTrustGrowFactor);
                  ++trust_expand_attempts;
                  decision = "T_GROW";
                  adaptive_failure_reason = "T_TRUST_TOO_SMALL";
                }
                else
                {
                  decision = "TR_MAX_REACHED";
                  adaptive_failure_reason = "T_TRUST_MAX_REACHED";
                }
              }
              else
              {
                const SCPQPSolveResult no_pt_probe =
                    runAdaptiveProbe(false, false, "NO_PT_TRUST");
                probe_no_pt_tr = static_cast<int>(no_pt_probe.success);
                if (no_pt_probe.success)
                {
                  if ((trust_p < trust_p_max - 1.0e-9 ||
                       (!optimize_time || trust_t < trust_t_max - 1.0e-9)) &&
                      trust_expand_attempts < kMaxTrustExpandAttempts)
                  {
                    trust_p = std::min(trust_p_max, trust_p * kTrustGrowFactor);
                    if (optimize_time)
                      trust_t = std::min(trust_t_max, trust_t * kTrustGrowFactor);
                    ++trust_expand_attempts;
                    decision = "PT_GROW";
                    adaptive_failure_reason = "P_T_TRUST_TOO_SMALL";
                  }
                  else
                  {
                    decision = "TR_MAX_REACHED";
                    adaptive_failure_reason = "P_T_TRUST_MAX_REACHED";
                  }
                }
                else
                {
                  if (team_scp &&
                      team_visibility_contract_.repair_k == 3 &&
                      iteration == 0)
                  {
                    // A fixed MINCO boundary can make a misplaced hard SIDE
                    // row impossible.  Keep this cheap provenance log after
                    // the diagnostic subset-QP experiment; no extra QPs run
                    // on the production failure path.
                    int zero_visibility = 0;
                    int zero_other = 0;
                    size_t first_zero_other = rows.size();
                    for (size_t row = 0; row < rows.size(); ++row)
                      if (rows[row].norm() < 1.0e-10 && upper[row] < 0.0)
                      {
                        const bool is_visibility =
                            row >= team_visibility_rows_begin &&
                            row < team_visibility_rows_end;
                        if (is_visibility) ++zero_visibility;
                        else
                        {
                          ++zero_other;
                          if (first_zero_other == rows.size())
                            first_zero_other = row;
                        }
                      }
                    const char *zero_other_group = "NONE";
                    if (first_zero_other < local_sfc_rows_begin)
                      zero_other_group = "CORRIDOR";
                    else if (first_zero_other < side_region_rows_begin)
                      zero_other_group = "LOCAL_SFC";
                    else if (first_zero_other < team_visibility_rows_begin)
                      zero_other_group = "SIDE";
                    else if (first_zero_other < static_rows_begin)
                      zero_other_group = "VISIBILITY";
                    else if (first_zero_other < dynamic_rows_begin)
                      zero_other_group = "STATIC";
                    else if (first_zero_other < dynamics_rows_begin)
                      zero_other_group = "DYNAMIC";
                    else if (first_zero_other < rows.size())
                      zero_other_group = "DYNAMICS";
                    ROS_INFO("[TEAM_K3_HARD_ROW_DIAG] contract_id=%lu "
                             "drone=%d visibility_rows=%zu other_rows=%zu "
                             "zero_visibility=%d zero_other=%d "
                             "first_zero_other_group=%s first_zero_other_ub=%.6f",
                             static_cast<unsigned long>(
                                 team_visibility_contract_.contract_id),
                             drone_id_,
                             team_visibility_rows_end -
                                 team_visibility_rows_begin,
                             rows.size() -
                                 (team_visibility_rows_end -
                                  team_visibility_rows_begin),
                             zero_visibility, zero_other, zero_other_group,
                             first_zero_other < rows.size()
                                 ? upper[first_zero_other] : 0.0);
                  }
                  const bool all_linearized_probes_infeasible =
                      no_p_probe.status_text.find("primal_infeasible") !=
                          std::string::npos &&
                      no_t_probe.status_text.find("primal_infeasible") !=
                          std::string::npos &&
                      no_pt_probe.status_text.find("primal_infeasible") !=
                          std::string::npos;
                  decision = all_linearized_probes_infeasible
                      ? "QP_PRIMAL_INFEASIBLE" : "NUMERICAL_FAILURE";
                  adaptive_failure_reason = decision;
                }
              }
            }
          }
          else
          {
            // Numerical/iteration failures are not infeasibility evidence.
            // Retry with a smaller box only to improve numerical conditioning,
            // and keep the classification visible in the adaptive log.
            saw_qp_max_iter = saw_qp_max_iter || qp.status_text == "max_iter";
            if (trust_shrink_attempts < kMaxTrustShrinkAttempts &&
                (trust_p > trust_p_min + 1.0e-9 ||
                 (!optimize_time || trust_t > trust_t_min + 1.0e-9)))
            {
              trust_p = std::max(trust_p_min, trust_p * kTrustShrinkFactor);
              if (optimize_time)
                trust_t = std::max(trust_t_min, trust_t * kTrustShrinkFactor);
              ++trust_shrink_attempts;
              decision = "PT_SHRINK";
              adaptive_failure_reason =
                  qp.status_text == "max_iter" ? "QP_MAX_ITER" : "QP_NUMERICAL_FAILURE";
            }
            else
            {
              decision = "TRUST_RETRY_EXHAUSTED";
              adaptive_failure_reason =
                  qp.status_text == "max_iter" ? "QP_MAX_ITER_EXHAUSTED"
                                                : "QP_NUMERICAL_FAILURE";
            }
          }
          ROS_INFO("[adaptive-tr] candidate_id=%d iter=%d qp_status=%s "
                   "trust_p_before=%.6f trust_t_before=%.6f "
                   "probe_no_p_tr=%d probe_no_t_tr=%d probe_no_pt_tr=%d "
                   "decision=%s trust_p_after=%.6f trust_t_after=%.6f "
                   "max_dp=0.0 max_dvt=0.0",
                   scp_candidate_id, iteration, qp.status_text.c_str(),
                   trust_p_before, trust_t_before, probe_no_p_tr, probe_no_t_tr,
                   probe_no_pt_tr, decision.c_str(), trust_p, trust_t);
          const std::string current_duration_text =
              formatVector(current_durations);
          const std::string current_virtual_text =
              formatVector(current_virtual_t);
          const std::string zero_delta_text =
              formatVector(Eigen::VectorXd::Zero(piece_num_));
          ROS_INFO("[scp-time-trust] candidate_id=%d iteration=%d attempt=%d "
                   "trust_alpha=%.6g current_real_T=%s current_virtual_T=%s "
                   "delta_virtual_T=%s predicted_delta_real_T=%s actual_delta_real_T=%s "
                   "predicted_max=0 actual_max=0 actual_relative_max=0 "
                   "qp_status=%s pri_res=%.6g dua_res=%.6g",
                   scp_candidate_id, iteration, attempt, trust_t,
                   current_duration_text.c_str(), current_virtual_text.c_str(),
                   zero_delta_text.c_str(), zero_delta_text.c_str(),
                   zero_delta_text.c_str(), qp.status_text.c_str(),
                   qp.primal_residual, qp.dual_residual);
          if (decision == "QP_PRIMAL_INFEASIBLE" ||
              decision == "NUMERICAL_FAILURE" ||
              decision == "TR_MAX_REACHED" ||
              decision == "TRUST_RETRY_EXHAUSTED")
          {
            current_points = base_points;
            current_virtual_t = base_virtual_t;
            current_durations = base_durations;
            total_duration = base_total_duration;
            window_start = initial_window_start_ratio * total_duration;
            window_end = initial_window_end_ratio * total_duration;
            jerkOpt_.generate(base_points, base_durations);
            logFinalStatus(adaptive_failure_reason.c_str(), false, false,
                           jerkOpt_.getTraj(), maxViolation(jerkOpt_.getTraj()));
            return false;
          }
          continue;
      }
      saw_qp_success = true;
        const ScpDynamicsSummary predicted_dynamics =
          predictDynamicsForStep(qp.step);
      const double predicted_team_margin = team_scp
          ? team_margin_current + critical_margin_row.dot(qp.step)
          : std::numeric_limits<double>::infinity();
      Eigen::Map<const Eigen::MatrixXd> step_points(
          qp.step.data(), 3, piece_num_ - 1);
        // TEAM_T_ONLY keeps the Local spatial decision exactly unchanged.
        // OSQP equality rows are still retained in the subproblem, but their
        // numerical feasibility tolerance must not leak into exact MINCO
        // regeneration as a position update.
        if (team_scp && team_scp_result_.mode == TeamSCPMode::TEAM_T_ONLY)
          trial_points = base_points;
        else
          trial_points = base_points + step_points;
        Eigen::VectorXd trial_virtual_t = base_virtual_t;
        if (optimize_time)
          trial_virtual_t += qp.step.segment(position_dim, virtual_time_dim);
        Eigen::VectorXd trial_durations = base_durations;
        if (optimize_time)
          VirtualT2RealT(trial_virtual_t, trial_durations);
        Eigen::VectorXd predicted_delta_real_t =
            Eigen::VectorXd::Zero(piece_num_);
        Eigen::VectorXd actual_delta_real_t =
            Eigen::VectorXd::Zero(piece_num_);
        double predicted_delta_real_t_max = 0.0;
        double actual_delta_real_t_max = 0.0;
        double actual_delta_real_t_relative_max = 0.0;
        if (optimize_time && trial_virtual_t.size() == piece_num_)
        {
          for (int index = 0; index < piece_num_; ++index)
          {
            predicted_delta_real_t(index) =
                realTimeJacobianAt(base_virtual_t(index)) *
                (trial_virtual_t(index) - base_virtual_t(index));
            predicted_delta_real_t_max = std::max(
                predicted_delta_real_t_max,
                std::abs(predicted_delta_real_t(index)));
          }
        }
        if (trial_durations.size() != piece_num_ ||
            !trial_durations.allFinite() ||
            (trial_durations.array() <= 1.0e-6).any())
        {
          const double trust_p_before = trust_p;
          const double trust_t_before = trust_t;
          const bool can_shrink_t = optimize_time &&
                                    trust_t > trust_t_min + 1.0e-9 &&
                                    trust_shrink_attempts < kMaxTrustShrinkAttempts;
          if (can_shrink_t)
          {
            trust_t = std::max(trust_t_min, trust_t * kTrustShrinkFactor);
            ++trust_shrink_attempts;
            adaptive_failure_reason = "INVALID_DURATION";
          }
          else
          {
            adaptive_failure_reason = "DURATION_INVALID_TRUST_RETRY_EXHAUSTED";
          }
          ROS_INFO("[adaptive-tr] candidate_id=%d iter=%d qp_status=%s "
                   "trust_p_before=%.6f trust_t_before=%.6f "
                   "probe_no_p_tr=-1 probe_no_t_tr=-1 probe_no_pt_tr=-1 "
                   "decision=%s trust_p_after=%.6f trust_t_after=%.6f "
                   "max_dp=0.0 max_dvt=0.0",
                   scp_candidate_id, iteration, qp.status_text.c_str(),
                   trust_p_before, trust_t_before,
                   can_shrink_t ? "T_SHRINK" : "TRUST_RETRY_EXHAUSTED",
                   trust_p, trust_t);
          current_points = base_points;
          current_virtual_t = base_virtual_t;
          current_durations = base_durations;
          total_duration = base_total_duration;
          window_start = initial_window_start_ratio * total_duration;
          window_end = initial_window_end_ratio * total_duration;
          jerkOpt_.generate(base_points, base_durations);
          ROS_INFO(
              "[dynamics-prediction-audit] candidate_id=%d candidate_type=%s "
              "iter=%d attempt=%d qp_status=%s "
              "pred_vmax=%.6f actual_vmax=NA err_v=NA rel_err_v=NA "
              "pred_amax=%.6f actual_amax=NA err_a=NA rel_err_a=NA "
              "pred_jmax=%.6f actual_jmax=NA err_j=NA rel_err_j=NA "
              "trust_p=%.6g physical_time_trust=%.6g max_dp=%.6g "
              "max_actual_dt=NA trial_accepted=0 reject_reason=INVALID_DURATION",
              scp_candidate_id, gradient_audit_candidate_.c_str(), iteration,
              attempt, qp.status_text.c_str(), predicted_dynamics.max_vel_value,
              predicted_dynamics.max_acc_value, predicted_dynamics.max_jerk_value,
              trust_p, trust_t,
              qp.step.size() == dim && position_dim > 0
                  ? qp.step.head(position_dim).cwiseAbs().maxCoeff()
                  : 0.0);
          if (!can_shrink_t)
          {
            logFinalStatus(adaptive_failure_reason.c_str(), false, false,
                           jerkOpt_.getTraj(), maxViolation(jerkOpt_.getTraj()));
            return false;
          }
          continue;
        }
        current_durations = trial_durations;
        current_virtual_t = trial_virtual_t;
        total_duration = current_durations.sum();
        if (trial_durations.size() == piece_num_ &&
            base_durations.size() == piece_num_)
        {
          actual_delta_real_t = trial_durations - base_durations;
          for (int index = 0; index < piece_num_; ++index)
          {
            actual_delta_real_t_max = std::max(
                actual_delta_real_t_max,
                std::abs(actual_delta_real_t(index)));
            if (base_durations(index) > 1.0e-6)
              actual_delta_real_t_relative_max = std::max(
                  actual_delta_real_t_relative_max,
                  std::abs(actual_delta_real_t(index)) /
                      base_durations(index));
          }
          max_real_time_step = std::max(max_real_time_step,
                                        actual_delta_real_t_max);
        }
        window_start = initial_window_start_ratio * total_duration;
        window_end = initial_window_end_ratio * total_duration;
        jerkOpt_.generate(trial_points, trial_durations);
        trial_violation = maxViolation(jerkOpt_.getTraj());
        const double trial_dynamic_violation =
            maxDynamicBodyViolation(jerkOpt_.getTraj());
        const double trial_swarm_violation =
            maxSwarmViolation(jerkOpt_.getTraj());
        // Feedback119: trial gate uses the EXACT continuous corridor
        // violation (endpoints + f'=0 roots), not a 25-point lattice that
        // can hide between-sample penetrations.
        trial_local_sfc_violation =
            static_sfc_plane_count > 0
                ? continuousLocalSfcMaxViolation(jerkOpt_.getTraj(), nullptr)
                : maxLocalSfcViolation(jerkOpt_.getTraj());
        trial_static_violation = maxNativeStaticViolation(jerkOpt_.getTraj());
        bool trial_team_binary_ok = true;
        const double trial_team_margin = team_scp
            ? minimumTeamMargin(jerkOpt_.getTraj(), &trial_team_binary_ok)
            : std::numeric_limits<double>::infinity();
        const double trial_team_side_violation =
            maxTeamSideViolation(jerkOpt_.getTraj());
        std::vector<std::pair<int, int>> trial_static_segments;
        std::string trial_static_reason;
        // The native checker appends separating rays to cps_ even when
        // flag_first_init is false.  A trial trajectory must not leak those
        // temporary rays into the next SCP linearization.
        const ConstraintPoints constraints_before_trial = cps_;
        // Reuse the native inflated-map checker as a trial feasibility gate.
        // ConstraintPoints remain the source of separating rows when present;
        // this gate prevents an SCP step from entering a previously unseen
        // occupied cell that could not have generated a native ray at the
        // linearization point.
        trial_static_free =
            finelyCheckAndSetConstraintPoints(trial_static_segments, jerkOpt_,
                                              false, &trial_static_reason) ==
            CHK_RET::OBS_FREE;
        cps_ = constraints_before_trial;
        trial_dynamics = appendDynamicsConstraints(nullptr, nullptr);
        Eigen::VectorXd trial_x(dim), trial_gradient(dim);
        Eigen::Map<Eigen::MatrixXd>(trial_x.data(), 3, piece_num_ - 1) =
            trial_points;
        if (optimize_time)
          trial_x.segment(position_dim, virtual_time_dim) = trial_virtual_t;
        const ConstraintPoints constraints_before_cost = cps_;
        const double trial_native_cost =
            costFunctionCallback(this, trial_x.data(), trial_gradient.data(), dim);
        executionCheckpoint();
        cps_ = constraints_before_cost;
        current_durations = trial_durations;
        current_virtual_t = trial_virtual_t;
        total_duration = trial_durations.sum();
        window_start = initial_window_start_ratio * total_duration;
        window_end = initial_window_end_ratio * total_duration;
        jerkOpt_.generate(trial_points, trial_durations);
        const auto normalizedModelError = [](const double actual,
                                             const double predicted,
                                             const double limit) {
          if (limit <= 0.0 || !std::isfinite(limit) ||
              !std::isfinite(actual) || !std::isfinite(predicted))
            return std::numeric_limits<double>::quiet_NaN();
          return std::abs(actual - predicted) / limit;
        };
        trial_model_error_v = normalizedModelError(
            trial_dynamics.max_vel_value, predicted_dynamics.max_vel_value,
            max_vel_);
        trial_model_error_a = normalizedModelError(
            trial_dynamics.max_acc_value, predicted_dynamics.max_acc_value,
            max_acc_);
        trial_model_error_j = normalizedModelError(
            trial_dynamics.max_jerk_value, predicted_dynamics.max_jerk_value,
            max_jer_);
        trial_model_error = 0.0;
        bool have_model_error = false;
        for (const double error : {trial_model_error_v, trial_model_error_a,
                                   trial_model_error_j})
        {
          if (std::isfinite(error))
          {
            trial_model_error = std::max(trial_model_error, error);
            have_model_error = true;
          }
        }
        if (!have_model_error)
          trial_model_error = std::numeric_limits<double>::quiet_NaN();
        trial_model_unreliable = !std::isfinite(trial_model_error) ||
                                 trial_model_error > kModelAgreementAcceptable;
        const bool model_mismatch =
            !std::isfinite(trial_model_error) ||
            trial_model_error > kModelAgreementPoor;
        const bool actual_dynamics_infeasible =
            trial_dynamics.maxViolation() > 1.0e-3;
        trial_model_accurate_but_infeasible =
            actual_dynamics_infeasible && !trial_model_unreliable;
        if (trial_model_accurate_but_infeasible)
          trial_agreement_class = "MODEL_ACCURATE_BUT_INFEASIBLE";
        else if (model_mismatch)
          trial_agreement_class = "MODEL_MISMATCH";
        else if (trial_model_error > kModelAgreementAcceptable)
          trial_agreement_class = "POOR";
        else if (trial_model_error > kModelAgreementGood)
          trial_agreement_class = "ACCEPTABLE";
        else
          trial_agreement_class = "GOOD";
        const double current_merit =
            current_native_cost + 1000.0 *
                (current_violation + current_local_sfc_violation +
                 current_static_violation + current_dynamics_snapshot.maxViolation() +
                 current_dynamic_violation);
        const double trial_merit =
            trial_native_cost + 1000.0 *
                (trial_violation + trial_local_sfc_violation +
                 trial_static_violation + trial_dynamics.maxViolation() +
                 trial_dynamic_violation);
        const bool objective_ok = team_scp ||
            (std::isfinite(trial_merit) &&
             trial_merit <= current_merit +
                 1.0e-4 * std::max(1.0, std::abs(current_merit)));
        const bool team_margin_ok = !team_scp ||
            (std::isfinite(trial_team_margin) && trial_team_binary_ok &&
             (trial_team_margin >=
                  team_visibility_contract_.required_margin - 2.0e-3 ||
              trial_team_margin > current_team_margin + 1.0e-4));
        const bool team_side_ok = !team_scp ||
            trial_team_side_violation <=
                std::max(2.0e-3, current_team_side_violation + 1.0e-5);
        const bool static_restoration_step =
            static_check != CHK_RET::OBS_FREE && !trial_static_free &&
            (trial_static_violation < current_static_violation - 1.0e-4 ||
             trial_local_sfc_violation <
                 current_local_sfc_violation - 1.0e-4);
        accepted =
            trial_violation <= std::max(1.0e-3, current_violation + 1.0e-5) &&
            // Feedback119: monotone improvement, like every other term in
            // this predicate.  The old absolute `<= 2.0e-3` made a
            // corner-cutting MINCO seed unfixable: the first accepted step
            // had to be corridor-clean before the active set ever grew.
            // Absolute cleanliness is enforced once at the exit
            // (LOCAL_SFC_FINAL_VIOLATION on the continuous metric).
            trial_local_sfc_violation <=
                std::max(2.0e-3, current_local_sfc_violation + 1.0e-5) &&
            trial_static_violation <=
                std::max(1.0e-3, current_static_violation + 1.0e-5) &&
            (trial_static_free || static_restoration_step) &&
            trial_dynamics.maxViolation() <=
                std::max(1.0e-3, current_dynamics_snapshot.maxViolation() + 1.0e-5) &&
            trial_dynamic_violation <=
                std::max(1.0e-3, current_dynamic_violation + 1.0e-5) &&
            trial_swarm_violation <=
                std::max(1.0e-3, current_swarm_violation + 1.0e-5) &&
            objective_ok && team_margin_ok && team_side_ok;
        if (team_scp)
        {
          team_scp_result_.linear_margin_prediction =
              predicted_team_margin;
          team_scp_result_.margin_after = trial_team_margin;
          team_scp_result_.model_agreement =
              std::isfinite(predicted_team_margin) &&
                      std::isfinite(trial_team_margin)
                  ? std::abs(predicted_team_margin - trial_team_margin)
                  : std::numeric_limits<double>::infinity();
          ROS_INFO("[TEAM_SCP_MARGIN_TRIAL] contract_id=%lu drone=%d "
                   "iteration=%d attempt=%d linear=%.9f nonlinear=%.9f "
                   "required=%.9f binary=%d side_violation=%.9f "
                   "model_error=%.9f accepted=%d",
                   static_cast<unsigned long>(
                       team_visibility_contract_.contract_id),
                   drone_id_, iteration, attempt, predicted_team_margin,
                   trial_team_margin,
                   team_visibility_contract_.required_margin,
                   static_cast<int>(trial_team_binary_ok),
                   trial_team_side_violation,
                   team_scp_result_.model_agreement,
                   static_cast<int>(accepted));
        }
        const bool dynamics_bad_for_audit =
            trial_dynamics.maxViolation() >
            std::max(1.0e-3, current_dynamics_snapshot.maxViolation() + 1.0e-5);
        const bool spatial_bad_for_audit =
            !trial_static_free ||
            trial_static_violation >
                std::max(1.0e-3, current_static_violation + 1.0e-5) ||
            trial_local_sfc_violation >
                std::max(2.0e-3, current_local_sfc_violation + 1.0e-5) ||
            trial_violation > std::max(1.0e-3, current_violation + 1.0e-5) ||
            !team_margin_ok || !team_side_ok;
        const char *dynamics_audit_reason =
            accepted ? "ACCEPTED"
                     : (dynamics_bad_for_audit && !spatial_bad_for_audit
                            ? "DYNAMICS"
                            : (spatial_bad_for_audit && !dynamics_bad_for_audit
                                   ? "SPATIAL"
                                   : (!objective_ok ? "OBJECTIVE" : "MULTIPLE")));
        const auto relativeError = [](const double actual, const double predicted) {
          return std::isfinite(actual) && std::isfinite(predicted) &&
                         std::abs(predicted) > 1.0e-9
                     ? (actual - predicted) / std::abs(predicted)
                     : std::numeric_limits<double>::quiet_NaN();
        };
        ROS_INFO(
            "[dynamics-prediction-audit] candidate_id=%d candidate_type=%s "
            "iter=%d attempt=%d qp_status=%s "
            "pred_vmax=%.6f actual_vmax=%.6f err_v=%.6f rel_err_v=%.6f "
            "pred_amax=%.6f actual_amax=%.6f err_a=%.6f rel_err_a=%.6f "
            "pred_jmax=%.6f actual_jmax=%.6f err_j=%.6f rel_err_j=%.6f "
            "trust_p=%.6g physical_time_trust=%.6g max_dp=%.6g "
            "max_actual_dt=%.6g trial_accepted=%d reject_reason=%s",
            scp_candidate_id, gradient_audit_candidate_.c_str(), iteration,
            attempt, qp.status_text.c_str(), predicted_dynamics.max_vel_value,
            trial_dynamics.max_vel_value,
            trial_dynamics.max_vel_value - predicted_dynamics.max_vel_value,
            relativeError(trial_dynamics.max_vel_value,
                          predicted_dynamics.max_vel_value),
            predicted_dynamics.max_acc_value, trial_dynamics.max_acc_value,
            trial_dynamics.max_acc_value - predicted_dynamics.max_acc_value,
            relativeError(trial_dynamics.max_acc_value,
                          predicted_dynamics.max_acc_value),
            predicted_dynamics.max_jerk_value, trial_dynamics.max_jerk_value,
            trial_dynamics.max_jerk_value - predicted_dynamics.max_jerk_value,
            relativeError(trial_dynamics.max_jerk_value,
                          predicted_dynamics.max_jerk_value),
            trust_p, trust_t,
            qp.step.size() == dim && position_dim > 0
                ? qp.step.head(position_dim).cwiseAbs().maxCoeff()
                : 0.0,
            actual_delta_real_t_max, static_cast<int>(accepted),
            dynamics_audit_reason);
        if (optimize_time)
        {
          const double trial_delta_t =
              (trial_virtual_t - base_virtual_t).cwiseAbs().maxCoeff();
          ROS_INFO("[scp-free-time-step] candidate_id=%d candidate_type=%s "
                   "iteration=%d attempt=%d solver_status=%s "
                   "trial_delta_virtual_T_max_abs=%.6g "
                   "predicted_delta_real_T_max_abs=%.6g "
                   "actual_delta_real_T_max_abs=%.6g "
                   "actual_delta_real_T_relative_max=%.6g time_trust_alpha=%.6g "
                   "trial_duration=%.6f trial_static_violation=%.6f "
                   "trial_dynamics_violation=%.6f accepted=%d",
                   scp_candidate_id, gradient_audit_candidate_.c_str(), iteration,
                   attempt, qp.status_text.c_str(), trial_delta_t,
                   predicted_delta_real_t_max, actual_delta_real_t_max,
                   actual_delta_real_t_relative_max, trust_t,
                   trial_durations.sum(), trial_static_violation,
                   trial_dynamics.maxViolation(), static_cast<int>(accepted));
        }
        if (!accepted)
        {
          const double step_p_inf =
              qp.step.size() == dim && position_dim > 0
                  ? qp.step.head(position_dim).cwiseAbs().maxCoeff()
                  : 0.0;
          const double step_t_inf =
              qp.step.size() == dim && optimize_time && virtual_time_dim > 0
                  ? qp.step.segment(position_dim, virtual_time_dim)
                        .cwiseAbs()
                        .maxCoeff()
                  : 0.0;
          const bool spatial_bad =
              !trial_static_free ||
              trial_static_violation >
                  std::max(1.0e-3, current_static_violation + 1.0e-5) ||
              trial_local_sfc_violation >
                  std::max(2.0e-3, current_local_sfc_violation + 1.0e-5) ||
              trial_violation > std::max(1.0e-3, current_violation + 1.0e-5);
          const bool dynamics_bad =
              trial_dynamics.maxViolation() >
              std::max(1.0e-3, current_dynamics_snapshot.maxViolation() + 1.0e-5);
          const double trust_p_before = trust_p;
          const double trust_t_before = trust_t;
          std::string decision;
          const bool p_step_at_boundary =
              step_p_inf >= kTrustBoundaryRatio * trust_p;
          const bool t_step_at_boundary =
              optimize_time &&
              actual_delta_real_t_relative_max >=
                  kTrustBoundaryRatio * trust_t;
          if (dynamics_bad &&
              (trial_model_accurate_but_infeasible || trial_model_unreliable))
          {
            // A rejected dynamics trial is a step-level failure, regardless of
            // whether the model was accurate or mismatched.  Keep the current
            // base fixed and shrink the active trust block(s).  In particular,
            // MODEL_ACCURATE_BUT_INFEASIBLE must not terminate the candidate:
            // the next inner attempt re-solves the same local model at the same
            // base with a smaller trust region.
            const bool shrink_p = p_step_at_boundary || !t_step_at_boundary;
            const bool shrink_t = t_step_at_boundary || !p_step_at_boundary;
            if (shrink_p && !shrink_t)
              decision = "P_SHRINK";
            else if (shrink_t && !shrink_p)
              decision = "T_SHRINK";
            else
              decision = "PT_SHRINK";
            if (!shrinkAgreementTrust(shrink_p, shrink_t))
            {
              decision = "TRUST_RETRY_EXHAUSTED";
              adaptive_failure_reason = trial_model_accurate_but_infeasible
                                             ? "DYNAMICS_TRUST_RETRY_EXHAUSTED"
                                             : "DYNAMICS_MODEL_MISMATCH_TRUST_EXHAUSTED";
            }
            else
            {
              adaptive_failure_reason = trial_model_accurate_but_infeasible
                                             ? "DYNAMICS_STEP_REJECTED_TRUST_SHRINK"
                                             : "DYNAMICS_MODEL_MISMATCH";
            }
          }
          else if (spatial_bad && !dynamics_bad)
          {
            decision = "P_SHRINK";
            if (trust_shrink_attempts < kMaxTrustShrinkAttempts &&
                trust_p > trust_p_min + 1.0e-9)
            {
              trust_p = std::max(trust_p_min, trust_p * kTrustShrinkFactor);
              ++trust_shrink_attempts;
              adaptive_failure_reason = "SPATIAL_TRIAL_REJECT";
            }
            else
            {
              decision = "TRUST_RETRY_EXHAUSTED";
              adaptive_failure_reason = "SPATIAL_TRUST_RETRY_EXHAUSTED";
            }
          }
          else
          {
            decision = "PT_SHRINK";
            if (trust_shrink_attempts < kMaxTrustShrinkAttempts &&
                (trust_p > trust_p_min + 1.0e-9 ||
                 (!optimize_time || trust_t > trust_t_min + 1.0e-9)))
            {
              trust_p = std::max(trust_p_min, trust_p * kTrustShrinkFactor);
              if (optimize_time)
                trust_t = std::max(trust_t_min, trust_t * kTrustShrinkFactor);
              ++trust_shrink_attempts;
              adaptive_failure_reason = "TRIAL_REJECT";
            }
            else
            {
              decision = "TRUST_RETRY_EXHAUSTED";
              adaptive_failure_reason = "TRUST_RETRY_EXHAUSTED";
            }
          }
          logModelAgreement(
              iteration, attempt, qp.status_text.c_str(), predicted_dynamics,
              trial_dynamics, trial_model_error_v, trial_model_error_a,
              trial_model_error_j, trial_model_error, trial_agreement_class,
              step_p_inf, actual_delta_real_t_max, decision.c_str(), accepted);
          ROS_INFO("[adaptive-tr] candidate_id=%d iter=%d qp_status=%s "
                   "trust_p_before=%.6f trust_t_before=%.6f "
                   "probe_no_p_tr=-1 probe_no_t_tr=-1 probe_no_pt_tr=-1 "
                   "decision=%s trust_p_after=%.6f trust_t_after=%.6f "
                   "max_dp=%.6g max_dvt=%.6g",
                   scp_candidate_id, iteration, qp.status_text.c_str(),
                   trust_p_before, trust_t_before, decision.c_str(), trust_p,
                   trust_t, step_p_inf, step_t_inf);
          const std::string base_duration_text = formatVector(base_durations);
          const std::string base_virtual_text = formatVector(base_virtual_t);
          const std::string delta_virtual_text =
              formatVector(trial_virtual_t - base_virtual_t);
          const std::string predicted_delta_real_text =
              formatVector(predicted_delta_real_t);
          const std::string actual_delta_real_text =
              formatVector(actual_delta_real_t);
          ROS_INFO("[scp-time-trust] candidate_id=%d iteration=%d attempt=%d "
                   "trust_alpha=%.6g current_real_T=%s current_virtual_T=%s "
                   "delta_virtual_T=%s predicted_delta_real_T=%s actual_delta_real_T=%s "
                   "predicted_max=%.6g actual_max=%.6g actual_relative_max=%.6g "
                   "qp_status=%s pri_res=%.6g dua_res=%.6g",
                   scp_candidate_id, iteration, attempt, trust_t,
                   base_duration_text.c_str(), base_virtual_text.c_str(),
                   delta_virtual_text.c_str(), predicted_delta_real_text.c_str(),
                   actual_delta_real_text.c_str(), predicted_delta_real_t_max,
                   actual_delta_real_t_max, actual_delta_real_t_relative_max,
                   qp.status_text.c_str(), qp.primal_residual, qp.dual_residual);
          if (!trial_static_free)
          {
            ROS_WARN("[scp-static-failure] reason=STATIC_TRUST_REGION_EXHAUSTED "
                     "piece=-1 time=NA static_violation=%.6f "
                     "corridor_violation=%.6f trust_region=%.6f solver_status=%s",
                     trial_static_violation, trial_violation, trust_p,
                     trial_static_reason.empty() ? "occupied" :
                                                    trial_static_reason.c_str());
          }
          current_points = base_points;
          current_virtual_t = base_virtual_t;
          current_durations = base_durations;
          total_duration = base_total_duration;
          window_start = initial_window_start_ratio * total_duration;
          window_end = initial_window_end_ratio * total_duration;
          jerkOpt_.generate(base_points, base_durations);
          if (decision == "TRUST_RETRY_EXHAUSTED")
          {
            logFinalStatus(adaptive_failure_reason.c_str(), false, false,
                           jerkOpt_.getTraj(), maxViolation(jerkOpt_.getTraj()));
            return false;
          }
          ROS_INFO(
              "[scp-trust-retry] candidate_id=%d scp_iter=%d attempt=%d "
              "decision=%s base_preserved=1 retry_same_base=1 "
              "trust_p=%.6g trust_t_physical=%.6g",
              scp_candidate_id, iteration, attempt, decision.c_str(), trust_p,
              trust_t);
        }
        else
        {
          accepted_attempt = attempt;
          accepted_predicted_dynamics = predicted_dynamics;
          accepted_trial_dynamics = trial_dynamics;
        }
      }
      if (!accepted)
      {
        current_points = base_points;
        current_virtual_t = base_virtual_t;
        current_durations = base_durations;
        total_duration = base_total_duration;
        window_start = initial_window_start_ratio * total_duration;
        window_end = initial_window_end_ratio * total_duration;
        jerkOpt_.generate(base_points, base_durations);
        const bool dynamics_blocked =
            saw_qp_success &&
            trial_violation <= std::max(1.0e-3, current_violation + 1.0e-5) &&
            trial_dynamics.maxViolation() >
                std::max(1.0e-3, current_dynamics.maxViolation() + 1.0e-5);
        const std::string fallback_reason =
            !adaptive_failure_reason.empty()
                ? adaptive_failure_reason
                : (saw_qp_success
                       ? (dynamics_blocked ? "SCP_DYNAMICS_TRUST_REGION_EXHAUSTED"
                                           : "SCP_TRUST_REGION_EXHAUSTED")
                       : (saw_qp_failure
                              ? (saw_qp_max_iter ? "SCP_QP_MAX_ITER"
                                                 : "SCP_QP_FAILURE")
                              : "SCP_QP_FAILURE"));
        const std::string terminal_reason =
            fallback_reason == "DYNAMICS_STEP_REJECTED_TRUST_SHRINK"
                ? "DYNAMICS_TRUST_RETRY_EXHAUSTED"
                : fallback_reason;
        logFinalStatus(terminal_reason.c_str(),
                       false, false, jerkOpt_.getTraj(), maxViolation(jerkOpt_.getTraj()));
        return false;
      }

      const double step_norm = (trial_points - base_points).norm();
      if (trial_points.size() > 0)
        max_position_step = std::max(
            max_position_step,
            (trial_points - base_points).cwiseAbs().maxCoeff());
      if (optimize_time)
        max_virtual_time_step = std::max(
            max_virtual_time_step,
            (current_virtual_t - base_virtual_t).cwiseAbs().maxCoeff());
      current_points = trial_points;
      accepted_any = true;
      const double accepted_step_p =
          (trial_points - base_points).size() > 0
              ? (trial_points - base_points).cwiseAbs().maxCoeff()
              : 0.0;
      const double accepted_step_t_virtual =
          optimize_time && (current_virtual_t - base_virtual_t).size() > 0
              ? (current_virtual_t - base_virtual_t).cwiseAbs().maxCoeff()
              : 0.0;
      double accepted_step_t_real_relative = 0.0;
      double accepted_step_t_real_abs = 0.0;
      if (optimize_time && current_durations.size() == piece_num_ &&
          base_durations.size() == piece_num_)
      {
        for (int index = 0; index < piece_num_; ++index)
        {
          const double delta =
              std::abs(current_durations(index) - base_durations(index));
          accepted_step_t_real_abs = std::max(accepted_step_t_real_abs, delta);
          if (base_durations(index) > 1.0e-6)
            accepted_step_t_real_relative = std::max(
                accepted_step_t_real_relative, delta / base_durations(index));
        }
      }
      const double trust_p_before = trust_p;
      const double trust_t_before = trust_t;
      std::string trust_decision = "KEEP";
      const bool p_step_at_boundary =
          accepted_step_p >= kTrustBoundaryRatio * trust_p;
      const bool t_step_at_boundary =
          optimize_time &&
          accepted_step_t_real_relative >= kTrustBoundaryRatio * trust_t;
      if (trial_model_accurate_but_infeasible)
      {
        // An improving but still infeasible trial is an intermediate iterate,
        // not a final candidate.  Keep the radii conservative and force the
        // next major iteration to regenerate and relinearize at this base.
        trust_decision = "KEEP";
        ROS_INFO(
            "[scp-intermediate-accept] candidate_id=%d scp_iter=%d "
            "accepted_infeasible=1 next_action=RELINEARIZE",
            scp_candidate_id, iteration);
      }
      else if (trial_model_unreliable)
      {
        // Even a hard-feasible accepted trial should not immediately enlarge
        // a radius whose local dynamics model was unreliable.  The next SCP
        // iteration will relinearize at this accepted state with the smaller
        // block(s), preserving multi-step continuation.
        const bool shrink_p = p_step_at_boundary && !t_step_at_boundary;
        const bool shrink_t = t_step_at_boundary && !p_step_at_boundary;
        if (shrinkAgreementTrust(shrink_p || (!shrink_p && !shrink_t),
                                 shrink_t || (!shrink_p && !shrink_t)))
        {
          trust_decision = shrink_p ? "P_SHRINK" :
                           (shrink_t ? "T_SHRINK" : "PT_SHRINK");
        }
      }
      else if (p_step_at_boundary &&
               trust_p < trust_p_max - 1.0e-9)
      {
        trust_p = std::min(trust_p_max, trust_p * kTrustGrowFactor);
        trust_decision = "P_GROW";
      }
      if (!trial_model_accurate_but_infeasible && !trial_model_unreliable &&
          optimize_time && t_step_at_boundary &&
          trust_t < trust_t_max - 1.0e-9)
      {
        trust_t = std::min(trust_t_max, trust_t * kTrustGrowFactor);
        trust_decision = trust_decision == "KEEP" ? "T_GROW" : "PT_GROW";
      }
      logModelAgreement(iteration, accepted_attempt, last_qp_status.c_str(),
                        accepted_predicted_dynamics, accepted_trial_dynamics,
                        trial_model_error_v,
                        trial_model_error_a, trial_model_error_j,
                        trial_model_error, trial_agreement_class,
                        accepted_step_p, accepted_step_t_real_abs,
                        trust_decision.c_str(), true);
      ROS_INFO("[adaptive-tr] candidate_id=%d iter=%d qp_status=%s "
               "trust_p_before=%.6f trust_t_before=%.6f "
               "probe_no_p_tr=-1 probe_no_t_tr=-1 probe_no_pt_tr=-1 "
               "decision=%s trust_p_after=%.6f trust_t_after=%.6f "
               "max_dp=%.6g max_dvt=%.6g max_dT_abs=%.6g max_dT_relative=%.6g",
               scp_candidate_id, iteration, last_qp_status.c_str(),
               trust_p_before, trust_t_before, trust_decision.c_str(), trust_p,
               trust_t, accepted_step_p, accepted_step_t_virtual,
               accepted_step_t_real_abs, accepted_step_t_real_relative);
      jerkOpt_.generate(current_points, current_durations);
      const double actual_violation = maxViolation(jerkOpt_.getTraj());
      const ScpDynamicsSummary actual_dynamics =
          appendDynamicsConstraints(nullptr, nullptr);
      if ((actual_violation <= 1.0e-3 &&
           actual_dynamics.maxViolation() <= 1.0e-3 &&
           continuousLocalSfcMaxViolation(jerkOpt_.getTraj(), nullptr) <=
               1.0e-3 &&
           maxNativeStaticViolation(jerkOpt_.getTraj()) <= 1.0e-3) ||
          (step_norm <= 1.0e-5 && !trial_model_accurate_but_infeasible))
        break;
    }

    if (!refreshTimeState())
    {
      logFinalStatus("SCP_INVALID_DURATION", false, false,
                     jerkOpt_.getTraj(), 0.0);
      return false;
    }
    jerkOpt_.generate(current_points, current_durations);
    const double final_violation = maxViolation(jerkOpt_.getTraj());
    // Feedback119: the exit gate is the EXACT continuous corridor violation.
    const double final_local_sfc_violation =
        static_sfc_plane_count > 0
            ? continuousLocalSfcMaxViolation(jerkOpt_.getTraj(), nullptr)
            : maxLocalSfcViolation(jerkOpt_.getTraj());
    last_corridor_scp_telemetry_.iterations =
        corridor_scp_iterations_run;
    {
      size_t total_active = 0;
      for (const auto &times : sfc_active_times)
        total_active += times.size();
      last_corridor_scp_telemetry_.active_point_count =
          static_cast<int>(total_active);
    }
    last_corridor_scp_telemetry_.added_point_count = corridor_active_points_added;
    last_corridor_scp_telemetry_.final_continuous_violation =
        final_local_sfc_violation;
    last_corridor_scp_telemetry_.final_sampled_violation =
        maxLocalSfcViolation(jerkOpt_.getTraj());
    bool final_team_binary_ok = true;
    const double final_team_margin = team_scp
        ? minimumTeamMargin(jerkOpt_.getTraj(), &final_team_binary_ok)
        : std::numeric_limits<double>::infinity();
    const double final_team_side_violation =
        maxTeamSideViolation(jerkOpt_.getTraj());
    const ScpDynamicsSummary final_scp_dynamics =
        appendDynamicsConstraints(nullptr, nullptr);
    ROS_INFO("[scp-hard-corridor] candidate=%s obstacle_id=%d phase=final "
             "max_violation=%.6f dyn_vel_violation=%.6f "
             "dyn_acc_violation=%.6f dyn_jerk_violation=%.6f "
             "accepted_update=%d",
             gradient_audit_candidate_.c_str(), gradient_audit_obstacle_id_,
             final_violation, final_scp_dynamics.max_vel_violation,
             final_scp_dynamics.max_acc_violation,
             final_scp_dynamics.max_jerk_violation,
             static_cast<int>(accepted_any));
    if (!accepted_any || final_violation > 2.0e-3 || final_local_sfc_violation > 2.0e-3)
    {
      logFinalStatus(!accepted_any ? "SCP_NO_ACCEPTED_UPDATE"
                                   : (final_local_sfc_violation > 2.0e-3
                                          ? "LOCAL_SFC_FINAL_VIOLATION"
                                          : "SCP_FINAL_CORRIDOR_VIOLATION"),
                     false, false, jerkOpt_.getTraj(), final_violation);
      return false;
    }
    if (team_scp &&
        (!std::isfinite(final_team_margin) || !final_team_binary_ok ||
         final_team_margin <
             team_visibility_contract_.required_margin - 2.0e-3 ||
         final_team_side_violation > 2.0e-3))
    {
      team_scp_result_.margin_after = final_team_margin;
      const char *reason =
          final_team_side_violation > 2.0e-3
              ? "TEAM_FINAL_SIDE_SIGN_VIOLATION"
              : (!final_team_binary_ok
                     ? "TEAM_FINAL_BINARY_VISIBILITY_REJECT"
                     : "TEAM_FINAL_MARGIN_VIOLATION");
      logFinalStatus(reason, false, false, jerkOpt_.getTraj(),
                     final_violation);
      return false;
    }
    if (final_scp_dynamics.maxViolation() > 2.0e-3)
    {
      const char *reason =
          final_scp_dynamics.max_vel_violation > 2.0e-3
              ? "SCP_FINAL_DYNAMICS_VIOLATION:VELOCITY_LIMIT"
              : (final_scp_dynamics.max_acc_violation > 2.0e-3
                     ? "SCP_FINAL_DYNAMICS_VIOLATION:ACCELERATION_LIMIT"
                     : "SCP_FINAL_DYNAMICS_VIOLATION:JERK_LIMIT");
      logFinalStatus(reason, false, false, jerkOpt_.getTraj(),
                     final_violation);
      return false;
    }

    const double final_dynamic_violation =
        maxDynamicBodyViolation(jerkOpt_.getTraj());
    if (final_dynamic_violation > 2.0e-3)
    {
      last_candidate_final_status_reason_ = "SCP_FINAL_DYNAMIC_VIOLATION";
      logFinalStatus(last_candidate_final_status_reason_.c_str(), false, false,
                     jerkOpt_.getTraj(), final_violation);
      return false;
    }

    // Feedback126: the constrained chain exits only on the unified physical
    // swarm predicate — the same one classify and the final preflight use.
    const double final_swarm_violation =
        maxSwarmViolation(jerkOpt_.getTraj());
    if (final_swarm_violation > 2.0e-3)
    {
      last_candidate_final_status_reason_ = "SCP_FINAL_SWARM_VIOLATION";
      logFinalStatus(last_candidate_final_status_reason_.c_str(), false, false,
                     jerkOpt_.getTraj(), final_violation);
      return false;
    }

    double final_max_vel = -1.0;
    double final_max_acc = -1.0;
    double final_max_jer = -1.0;
    sampledDynamics(jerkOpt_.getTraj(), final_max_vel, final_max_acc,
                    final_max_jer);
    const bool vel_ok =
        max_vel_ <= 0.0 || final_max_vel <= max_vel_ * 1.0001;
    const bool acc_ok =
        max_acc_ <= 0.0 || final_max_acc <= max_acc_ * 1.0001;
    const bool jerk_ok =
        max_jer_ <= 0.0 || final_max_jer <= max_jer_ * 1.0001;
    if (!vel_ok || !acc_ok || !jerk_ok)
    {
      const char *reason = !vel_ok ? "VELOCITY_LIMIT"
                           : (!acc_ok ? "ACCELERATION_LIMIT" : "JERK_LIMIT");
      logFinalStatus(reason, true, false, jerkOpt_.getTraj(),
                     final_violation);
      return false;
    }

    std::vector<std::pair<int, int>> segments;
    std::string final_constraint_reason;
    if (finelyCheckAndSetConstraintPoints(segments, jerkOpt_, false,
                                          &final_constraint_reason) !=
        CHK_RET::OBS_FREE)
    {
      const std::string combined_reason =
          std::string("FINAL_CONSTRAINT_CHECK_FAILED:") +
          (final_constraint_reason.empty() ? "UNKNOWN" : final_constraint_reason);
      logFinalStatus(combined_reason.c_str(), true, false,
                     jerkOpt_.getTraj(), final_violation);
      return false;
    }

    Eigen::VectorXd final_gradient(dim);
    Eigen::VectorXd final_x(dim);
    Eigen::Map<Eigen::MatrixXd>(final_x.data(), 3, piece_num_ - 1) =
        current_points;
    if (optimize_time)
      final_x.segment(position_dim, virtual_time_dim) = current_virtual_t;
    final_cost = costFunctionCallback(this, final_x.data(),
                                      final_gradient.data(), dim);
    logDirectionalVisibilityCost();
    logDirectionalVisibilityCost();
    // elastic telemetry 已随 elastic 语义退出 production(空操作)。
    optimal_points = cps_.points;
    last_lbfgs_status_ = lbfgs::LBFGS_CONVERGENCE;
    ROS_INFO("[scp-hard-corridor-time] candidate=%s final_duration=%.6f "
             "duration_delta=%.6f max_virtual_time_step=%.6f "
             "max_real_time_step=%.6f time_trust_alpha_initial=%.6g",
             gradient_audit_candidate_.c_str(), current_durations.sum(),
             current_durations.sum() - initial_durations.sum(),
             max_virtual_time_step, max_real_time_step, trust_t_initial);
    if (!candidate_local_sfc_planes_.empty())
    {
      const double local_sfc_start_after =
          std::isfinite(local_sfc_start_before) && initial_total_duration > 1.0e-6
              ? local_sfc_start_before / initial_total_duration * current_durations.sum()
              : -1.0;
      const std::string initial_duration_vector = formatVector(initial_durations);
      const std::string initial_virtual_vector = formatVector(initial_virtual_t);
      const std::string final_duration_vector = formatVector(current_durations);
      const std::string final_virtual_vector = formatVector(current_virtual_t);
      ROS_INFO("[local-sfc-free-time] candidate_id=%d candidate_type=%s piece_num=%d "
               "P_dim=%d virtual_T_dim=%d total_dim=%d initial_real_T=%s "
               "initial_virtual_T=%s final_real_T=%s final_virtual_T=%s "
               "max_abs_delta_P=%.6g max_abs_delta_virtual_T=%.6g "
               "native_cost_before=%.6g native_cost_after=%.6g",
               scp_candidate_id, gradient_audit_candidate_.c_str(), piece_num_,
               position_dim, virtual_time_dim, dim, initial_duration_vector.c_str(),
               initial_virtual_vector.c_str(), final_duration_vector.c_str(),
               final_virtual_vector.c_str(), max_position_step,
               max_virtual_time_step, native_cost_before, final_cost);
      const double progress_start =
          initial_total_duration > 1.0e-6 ? local_sfc_start_before / initial_total_duration : -1.0;
      const double progress_end =
          initial_total_duration > 1.0e-6 ? local_sfc_end_before / initial_total_duration : -1.0;
      ROS_INFO("[local-sfc-time-map] candidate_id=%d candidate_type=%s "
               "sfc_active_progress_start=%.6f sfc_active_progress_end=%.6f "
               "sfc_active_time_before=%.6f sfc_active_time_after=%.6f "
               "duration_before=%.6f duration_after=%.6f",
               scp_candidate_id, gradient_audit_candidate_.c_str(), progress_start,
               progress_end, local_sfc_start_before, local_sfc_start_after,
               initial_total_duration, current_durations.sum());
    }
    logFinalStatus("SCP_FINAL_OK", true, true, jerkOpt_.getTraj(),
                   final_violation);
    if (team_scp)
    {
      team_scp_result_.margin_after = final_team_margin;
      team_scp_result_.success = true;
    }
    return true;
  }

  // std::vector<std::pair<int, int>> PolyTrajOptimizer::finelyCheckConstraintPointsOnly(
  //     Eigen::MatrixXd &init_points)
  // {
  //   vector<std::pair<int, int>> segment_ids;
  //   constexpr int ENOUGH_INTERVAL = 2;
  //   double step_size = grid_map_->getResolution() / ((init_points.col(0) - init_points.rightCols(1)).norm() / (init_points.cols() - 1)) / 1.5;
  //   int in_id = -1, out_id = -1;
  //   int same_occ_state_times = ENOUGH_INTERVAL + 1;
  //   bool occ, last_occ = false;
  //   bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
  //   int i_end = ConstraintPoints::two_thirds_id(init_points); // only check closed 2/3 points.

  //   for (int i = 1; i <= i_end; ++i)
  //   {
  //     for (double a = 1.0; a > 0.0; a -= step_size)
  //     {
  //       occ = grid_map_->getInflateOccupancy(a * init_points.col(i - 1) + (1 - a) * init_points.col(i));

  //       if (occ && !last_occ)
  //       {
  //         if (same_occ_state_times > ENOUGH_INTERVAL || i == 1)
  //         {
  //           in_id = i - 1;
  //           flag_got_start = true;
  //         }
  //         same_occ_state_times = 0;
  //         flag_got_end_maybe = false; // terminate in advance
  //       }
  //       else if (!occ && last_occ)
  //       {
  //         out_id = i;
  //         flag_got_end_maybe = true;
  //         same_occ_state_times = 0;
  //       }
  //       else
  //       {
  //         ++same_occ_state_times;
  //       }

  //       if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == (int)init_points.cols() - 1)))
  //       {
  //         flag_got_end_maybe = false;
  //         flag_got_end = true;
  //       }

  //       last_occ = occ;

  //       if (flag_got_start && flag_got_end)
  //       {
  //         flag_got_start = false;
  //         flag_got_end = false;
  //         if (in_id < 0 || out_id < 0)
  //         {
  //           ROS_ERROR("Should not happen! in_id=%d, out_id=%d", in_id, out_id);
  //           vector<std::pair<int, int>> blank_ret;
  //           return blank_ret;
  //         }
  //         segment_ids.push_back(std::pair<int, int>(in_id, out_id));
  //       }
  //     }
  //   }

  //   return segment_ids;
  // }


  /* main planning API */
  bool PolyTrajOptimizer::optimizeTrajectory(
      const Eigen::MatrixXd &iniState,const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts,const Eigen::VectorXd &initT,
      Eigen::MatrixXd &optimal_points,double &final_cost)
  {
    try {
      executionCheckpoint();
      // 审计位按每次求解重置，绝不让上一次候选的结论泄漏到本次。
      const bool success=optimizeTrajectoryWithinBudget(iniState,finState,initInnerPts,initT,optimal_points,final_cost);
      if (team_visibility_reserve_.active)
      {
        const bool reserve_applies =
            gradient_audit_candidate_ != "SIDE_PLUS" &&
            gradient_audit_candidate_ != "SIDE_MINUS";
        ROS_INFO("[TEAM_RESERVE_GUIDE_RESULT] drone=%d contract_id=%lu "
                 "guide_count=%zu guide_cost=%.6f reserve_active=%d solve_success=%d",
                 drone_id_, static_cast<unsigned long>(team_visibility_reserve_.contract_id),
                 reserve_applies ? team_reserve_guides_.size() : 0,
                 reserve_applies ? team_reserve_cost_ : 0.0,
                 static_cast<int>(reserve_applies),
                 static_cast<int>(success));
      }
      executionCheckpoint();
      return success;
    } catch(const ExecutionDeadlineExceeded &) {
      // Discard the WHOLE unfinished integral/QP. A partially evaluated cost
      // or gradient is never accepted, and an enormous trial polynomial must
      // not leak into the subsequent risk/fallback validation loops.
      if(initT.size()>0 && initT.allFinite() && (initT.array()>0).all() &&
         initInnerPts.cols()==initT.size()-1) {
        jerkOpt_.reset(iniState,finState,initT.size());jerkOpt_.generate(initInnerPts,initT);
        optimal_points=jerkOpt_.getTraj().getPositions();
      }
      final_cost=std::numeric_limits<double>::infinity();
      force_stop_type_=STOP_FOR_ERROR;last_lbfgs_status_=lbfgs::LBFGSERR_CANCELED;
      last_candidate_final_status_reason_="EXECUTION_DEADLINE";
      ROS_WARN("[execution-reserve] drone=%d action=OBJECTIVE_EVALUATION_CANCELLED deadline_wall=%.9f now_wall=%.9f initializer_restored=1 partial_cost_accepted=0",
          drone_id_,execution_deadline_,ros::WallTime::now().toSec());
      return false;
    }
  }

  bool PolyTrajOptimizer::optimizeTrajectoryWithinBudget(
      const Eigen::MatrixXd &iniState, const Eigen::MatrixXd &finState,
      const Eigen::MatrixXd &initInnerPts, const Eigen::VectorXd &initT,
      Eigen::MatrixXd &optimal_points, double &final_cost)
  {
    if(!executionBudgetAvailable()) return false;
    if (initInnerPts.cols() != (initT.size() - 1))
    {
      ROS_ERROR("initInnerPts.cols() != (initT.size()-1)");
      return false;
    }

    piece_num_ = initT.size();

    optimization_diagnostics_ = OptimizationDiagnostics();
    current_cost_snapshot_ = CostGradientSnapshot();
    current_static_grad_sq_ = 0.0;
    current_moving_grad_sq_ = 0.0;
    current_side_grad_sq_ = 0.0;
    current_side_region_grad_sq_ = 0.0;
    current_tracking_grad_sq_ = 0.0;
    current_smoothness_grad_sq_ = 0.0;
    current_variance_grad_sq_ = 0.0;
    current_visibility_grad_sq_ = 0.0;
    gradient_audit_middle_logged_ = false;
    last_candidate_final_status_reason_.clear();
    last_lbfgs_status_ = lbfgs::LBFGSERR_UNKNOWNERROR;
    jerkOpt_.reset(iniState, finState, piece_num_);
    t_now_ = prediction_epoch_override_enabled_
                 ? prediction_epoch_override_
                 : ros::Time::now().toSec();
    jerkOpt_.generate(initInnerPts, initT);

    // 阶段 F(2026-09-22 恢复):P/T 联合 hard SCP 语义回归。SIDE 候选在
    // flag 打开时不再走 LBFGS,而是直接进入联合 [ΔP;Δτ] QP(corridor/SFC/
    // static/v-a-j/动态行 + 双信任盒),这是本项目的核心优化语义之一。
    // 恢复自 09-18 快照 src_20260918_2027.tar.gz,默认仍为 flag-off,
    // 与历史行为完全一致;验证另行安排。
    const bool side_candidate =
        gradient_audit_candidate_ == "SIDE_PLUS" ||
        gradient_audit_candidate_ == "SIDE_MINUS";
    // Feedback119(corridor 合同):携带 STATIC_COLLISION_CORRIDOR 平面的
    // A*-repair SIDE 候选必须走 corridor hard SCP——A* 已证明的安全几何以
    // 硬约束形式进入优化,MINCO seed 只是初值。
    bool candidate_has_static_corridor = false;
    for (const LocalSfcPlane &plane : candidate_local_sfc_planes_)
      if (plane.source == LocalSfcPlane::STATIC_COLLISION_CORRIDOR)
      {
        candidate_has_static_corridor = true;
        break;
      }
    /* Feedback126 §4 risk-driven dispatch: a candidate whose SEED carries a
     * real hard risk (static corridor / dynamic body / swarm / PVAJ) goes
     * straight into the constrained SCP chain instead of relying on soft
     * LBFGS and letting the final preflight discover the violation.  §5
     * ownership: a swarm risk against an OWNER-role peer (this drone wins
     * the deterministic ID-order tie for that pair) must NOT trigger
     * opposite avoidance — the adjuster peer moves instead; the incumbent
     * persists until its broadcast arrives.  Swarm risk against an
     * ADJUSTER-role peer (or with no ownership info) escalates. */
    const double dispatch_start =
        prediction_epoch_override_enabled_ ? prediction_epoch_override_
                                           : t_now_;
    UnifiedSwarmFinding dispatch_swarm_finding;
    bool swarm_risk_adjuster = false;
    bool swarm_risk_owner = false;
    if (swarmPhysicalViolation(jerkOpt_.getTraj(), dispatch_start,
                               touch_goal_, &dispatch_swarm_finding))
    {
      const std::vector<int> &adjusters = swarm_adjuster_peers_;
      const bool ownership_known = !adjusters.empty();
      const bool i_adjust_this_pair =
          !ownership_known ||
          std::find(adjusters.begin(), adjusters.end(),
                    dispatch_swarm_finding.peer_drone_id) != adjusters.end();
      if (i_adjust_this_pair)
        swarm_risk_adjuster = true;
      else
        swarm_risk_owner = true;
      ROS_INFO(
          "[PREDICTED_SWARM_HARD_CONFLICT] context=%s drone=%d peer=%d "
          "role=%s t=%.3f ellip_dist2=%.4f clearance2=%.4f "
          "dispatch=%s",
          side_candidate ? gradient_audit_candidate_.c_str() : "NOMINAL",
          drone_id_, dispatch_swarm_finding.peer_drone_id,
          i_adjust_this_pair ? "ADJUSTER" : "OWNER",
          dispatch_swarm_finding.conflict_time,
          dispatch_swarm_finding.ellip_dist2,
          swarm_clearance_ * swarm_clearance_,
          swarm_risk_adjuster ? "CONSTRAINED_SCP" : "KEEP_INCUMBENT_NO_ESCALATION");
    }
    int dispatch_dynamic_object = -1;
    const bool dynamic_risk =
        dynamicPhysicalViolation(jerkOpt_.getTraj(), dispatch_start,
                                 &dispatch_dynamic_object);
    if (dynamic_risk)
    {
      ROS_INFO("[PVAJ_DYNAMIC_DISPATCH] context=%s drone=%d "
               "predicate=DYNAMIC_HARD object=%d dispatch=CONSTRAINED_SCP",
               side_candidate ? gradient_audit_candidate_.c_str() : "NOMINAL",
               drone_id_, dispatch_dynamic_object);
    }
    double dispatch_v = 0.0, dispatch_a = 0.0, dispatch_j = 0.0;
    const bool pvaj_risk =
        pvaPhysicalViolation(jerkOpt_.getTraj(), &dispatch_v,
                             &dispatch_a, &dispatch_j);
    if (pvaj_risk)
    {
      ROS_INFO("[PVAJ_HARD_RISK] context=%s drone=%d seed_v=%.3f seed_a=%.3f "
               "seed_j=%.3f dispatch=CONSTRAINED_SCP",
               side_candidate ? gradient_audit_candidate_.c_str() : "NOMINAL",
               drone_id_, dispatch_v, dispatch_a, dispatch_j);
    }
    const bool risk_dispatch =
        candidate_has_static_corridor || dynamic_risk || pvaj_risk ||
        swarm_risk_adjuster;
    if (risk_dispatch)
    {
      ROS_INFO(
          "[SWARM_CONSTRAINED_SOLVE_ATTEMPT] context=%s drone=%d "
          "static_corridor=%d dynamic_risk=%d pvaj_risk=%d swarm_risk=%d "
          "owner_conflict_deferred=%d",
          side_candidate ? gradient_audit_candidate_.c_str() : "NOMINAL",
          drone_id_, static_cast<int>(candidate_has_static_corridor),
          static_cast<int>(dynamic_risk), static_cast<int>(pvaj_risk),
          static_cast<int>(swarm_risk_adjuster),
          static_cast<int>(swarm_risk_owner));
      // The constrained chain uses the same free-time MINCO parameterization
      // with hard rows built from the unified physical predicates.
      const bool scp_success = runCandidateHardCorridorSCP(
          iniState, finState, initInnerPts, initT, optimal_points, final_cost);
      ROS_INFO(
          "[SWARM_CONSTRAINED_SOLVE_%s] context=%s drone=%d success=%d "
          "reason=%s",
          scp_success ? "SUCCESS" : "FAILED",
          side_candidate ? gradient_audit_candidate_.c_str() : "NOMINAL",
          drone_id_, static_cast<int>(scp_success),
          last_candidate_final_status_reason_.c_str());
      // LOS observation planes stay a soft authority on the SCP path too:
      // the semi-hard slack QP adjusts the converged control points and
      // never rejects (Feedback117 invariant, same as the LBFGS path).
      if (scp_success && !candidate_local_sfc_planes_.empty())
        enforceCandidateLosPlanesSCP(iniState, finState, optimal_points);
      return scp_success;
    }

    variable_num_ = 4 * (piece_num_ - 1) + 1;

    ros::Time t0 = ros::Time::now(), t1, t2;
    int restart_nums = 0, rebound_times = 0;
    bool flag_force_return, flag_still_occ, flag_success;

    Eigen::MatrixXd current_inner_points = initInnerPts;
    Eigen::VectorXd current_durations = initT;
    std::vector<double> x_init;
    auto rebuild_initial_vector = [&]() {
      variable_num_ = 4 * (piece_num_ - 1) + 1;
      x_init.assign(variable_num_, 0.0);
      if (current_inner_points.size() > 0)
      {
        std::memcpy(x_init.data(), current_inner_points.data(),
                    current_inner_points.size() * sizeof(double));
      }
      Eigen::Map<Eigen::VectorXd> virtual_t(
          x_init.data() + current_inner_points.size(), current_durations.size());
      RealT2VirtualT(current_durations, virtual_t);
    };
    rebuild_initial_vector();

    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;
    lbfgs_params.max_iterations = 200;
    lbfgs_params.min_step = 1e-32;
    // lbfgs_params.abs_curv_cond = 0;
    lbfgs_params.past = 3;
    lbfgs_params.delta = 1.0e-4;
    do
    {
      /* ---------- prepare ---------- */
      iter_num_ = 0;
      flag_force_return = false;
      force_stop_type_ = DONT_STOP;
      flag_still_occ = false;
      flag_success = false;
      t_now_ = prediction_epoch_override_enabled_
                   ? prediction_epoch_override_
                   : ros::Time::now().toSec();

      /* ---------- optimize ---------- */
      t1 = ros::Time::now();
      int result = lbfgs::lbfgs_optimize(
          variable_num_,
          x_init.data(),
          &final_cost,
          PolyTrajOptimizer::costFunctionCallback,
          NULL,
          PolyTrajOptimizer::earlyExitCallback,
          this,
          &lbfgs_params,
          NULL);
      last_lbfgs_status_ = result;
      executionCheckpoint(); // LBFGS buffers have been released before cancellation

      t2 = ros::Time::now();
      double time_ms = (t2 - t1).toSec() * 1000;
      double total_time_ms = (t2 - t0).toSec() * 1000;

      /* ---------- get result and check collision ---------- */
      if (result == lbfgs::LBFGS_CONVERGENCE ||
          result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
          result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
          result == lbfgs::LBFGS_STOP)
      {
        flag_force_return = false;

        /* Feedback126 unified acceptance gate: the LBFGS exit must satisfy
         * the SAME physical predicates the final preflight applies.  The
         * legacy 1.25x swarm soft-buffer acceptance and the implicit "PVAJ
         * is free" behaviour are removed — a solve that violates the unified
         * swarm/PVAJ predicate is not a success, it retries (bounded) and
         * then reports failure so the risk-driven constrained path can own
         * it. */
        const double unified_start =
            prediction_epoch_override_enabled_ ? prediction_epoch_override_
                                               : t_now_;
        UnifiedSwarmFinding unified_swarm_finding;
        const bool swarm_hard_violated =
            swarmPhysicalViolation(jerkOpt_.getTraj(), unified_start,
                                   touch_goal_, &unified_swarm_finding);
        double unified_v = 0.0, unified_a = 0.0, unified_j = 0.0;
        const bool pva_hard_violated =
            pvaPhysicalViolation(jerkOpt_.getTraj(), &unified_v, &unified_a,
                                 &unified_j);
        if (swarm_hard_violated)
        {
          ROS_INFO_THROTTLE(0.5,
              "[UNIFIED_GATE_REJECT] context=%s drone=%d predicate=SWARM "
              "peer=%d t=%.3f ellip_dist2=%.4f clearance2=%.4f "
              "action=RETRY_WITHIN_REBOUND_LOOP",
              candidate_side_bias_enabled_ ? "SIDE" : "NOMINAL", drone_id_,
              unified_swarm_finding.peer_drone_id,
              unified_swarm_finding.conflict_time,
              unified_swarm_finding.ellip_dist2, swarm_clearance_ * swarm_clearance_);
        }
        if (pva_hard_violated)
        {
          ROS_INFO_THROTTLE(0.5,
              "[UNIFIED_GATE_REJECT] context=%s drone=%d predicate=PVAJ "
              "v=%.3f a=%.3f j=%.3f limits=(%.3f %.3f %.3f) "
              "action=RETRY_WITHIN_REBOUND_LOOP",
              candidate_side_bias_enabled_ ? "SIDE" : "NOMINAL", drone_id_,
              unified_v, unified_a, unified_j, max_vel_, max_acc_, max_jer_);
        }
        /* double check: fine collision check */
        std::vector<std::pair<int, int>> segments_nouse;
        if (!swarm_hard_violated && !pva_hard_violated &&
            finelyCheckAndSetConstraintPoints(segments_nouse, jerkOpt_, false) == CHK_RET::OBS_FREE)
        {
          flag_success = true;
          printf("\033[32miter=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms, total_time_ms, final_cost);
        }
        else
        {
          // A not-blank return value means collision to obstales
          flag_still_occ = true;
          restart_nums++;
          printf("\033[32miter=%d,time(ms)=%5.3f, fine check collided, keep optimizing\n\033[0m", iter_num_, time_ms);
        }
      }
      else if (result == lbfgs::LBFGSERR_CANCELED)
      {
        flag_force_return = true;
        rebound_times++;
        cout << "iter=" << iter_num_ << ",time(ms)=" << time_ms << ",rebound." << endl;
      }
      else
      {
        cout << "iter=" << iter_num_ << ",time(ms)=" << time_ms << ",error." << endl;
        const char *optimizer_context =
            candidate_side_bias_enabled_ ? "SIDE" : "NOMINAL";
        ROS_WARN("[lbfgs-error] context=%s drone_id=%d return_code=%d",
                 optimizer_context, drone_id_, result);
        ROS_WARN("Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
      }

    } while (executionBudgetAvailable() && (
        (flag_still_occ && restart_nums < 3) ||
        (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20)));

    if (gradient_audit_enabled_ && optimization_diagnostics_.valid)
    {
      const CostGradientSnapshot &snapshot = optimization_diagnostics_.final;
      ROS_INFO("[minco-gradient-audit] candidate_type=%s obstacle_id=%d phase=final "
               "evaluation=%d region_cost=%.6g moving_cost=%.6g tracking_cost=%.6g "
               "smoothness_cost=%.6g static_cost=%.6g variance_cost=%.6g total_cost=%.6g "
               "region_grad=%.6g moving_grad=%.6g tracking_grad=%.6g "
               "smoothness_grad=%.6g static_grad=%.6g variance_grad=%.6g total_grad=%.6g",
               gradient_audit_candidate_.c_str(), gradient_audit_obstacle_id_,
               optimization_diagnostics_.evaluations - 1,
               snapshot.side_region_cost, snapshot.moving_cost,
               snapshot.tracking_cost, snapshot.smoothness_cost,
               snapshot.static_cost, snapshot.variance_cost,
               snapshot.total_cost, snapshot.side_region_grad_norm,
               snapshot.moving_grad_norm,
               snapshot.tracking_grad_norm,
               snapshot.smoothness_grad_norm, snapshot.static_grad_norm,
               snapshot.variance_grad_norm, snapshot.total_grad_norm);
    }

    logDirectionalVisibilityCost();
    optimal_points = cps_.points;
    if (flag_success && !candidate_local_sfc_planes_.empty() &&
        !enforceCandidateLosPlanesSCP(iniState, finState, optimal_points))
    {
      flag_success = false;
      last_candidate_final_status_reason_ = "LOS_PLANE_SCP_FAILED";
    }

    return flag_success;
  }


  bool PolyTrajOptimizer::computePointsToCheck(
      poly_traj::Trajectory &traj,
      int id_cps_end, PtsChk_t &pts_check, bool full_trajectory)
  {
    return samplePointsToCheck(traj,id_cps_end,pts_check,full_trajectory,
        grid_map_->getResolution(),max_vel_,cps_num_prePiece_);
  }

  bool PolyTrajOptimizer::samplePointsToCheck(poly_traj::Trajectory &traj,
      int id_cps_end,PtsChk_t &pts_check,bool full_trajectory,
      double resolution,double max_velocity,int samples_per_piece)
  {
    pts_check.clear();
    if(id_cps_end<=0 || traj.getPieceNum()<=0 || samples_per_piece<=0 ||
       !std::isfinite(resolution) || resolution<=0 || !std::isfinite(max_velocity) || max_velocity<=0)
      return false;
    pts_check.resize(id_cps_end);
    const double RES = resolution, RES_2 = RES / 2;
    // const double DURATION = traj.getDurations().sum();
    Eigen::VectorXd durations = traj.getDurations();
    Eigen::VectorXd t_seg_start(durations.size() + 1);
    t_seg_start(0) = 0;
    for (int i = 0; i < durations.size(); ++i)
      t_seg_start(i + 1) = t_seg_start(i) + durations(i);
    const double DURATION = durations.sum();
    double t = 0.0, t_step = RES / max_velocity;
    Eigen::Vector3d pt_last = traj.getPos(0.0);
    // pts_check[0].push_back(pt_last);
    int id_cps_curr = 0, id_piece_curr = 0;

    while (true)
    {
      if (t > DURATION)
      {
        if (full_trajectory && pts_check.size() > 0)
        {
          while (!pts_check.empty() && pts_check.back().empty())
          {
            pts_check.pop_back();
          }

          if (pts_check.empty())
          {
            ROS_ERROR("Failed to get points list to check (0x02). pts_check.size()=%d", (int)pts_check.size());
            return false;
          }

          return true;
        }
        else
        {
          ROS_ERROR("Failed to get points list to check. full_trajectory=%d, pts_check.size()=%d", full_trajectory, (int)pts_check.size());
          pts_check.clear();
          return false;
        }

        // Eigen::Vector3d last_pt = pts_check[0][0].second;
        // for (size_t i = 0; i < pts_check.size(); ++i)
        // {
        //   cout << "--------------------" << endl;
        //   for (size_t j = 0; j < pts_check[i].size(); ++j)
        //   {
        //     cout << pts_check[i][j].first << " @ " << pts_check[i][j].second.transpose() << " " << (pts_check[i][j].second - last_pt).transpose() << endl;
        //     last_pt = pts_check[i][j].second;
        //   }
        // }
      }

      const double next_t_stp = t_seg_start(id_piece_curr) + durations(id_piece_curr) / samples_per_piece * ((id_cps_curr + 1) - samples_per_piece * id_piece_curr);
      if (t >= next_t_stp)
      {
        if (id_cps_curr + 1 >= samples_per_piece * (id_piece_curr + 1))
        {
          ++id_piece_curr;
          // cout << "id_piece_curr=" << id_piece_curr << endl;
          // cout << "traj.getPieceNum()=" << traj.getPieceNum() << endl;
        }
        if (++id_cps_curr >= id_cps_end)
        {
          break;
        }
      }

      // cout << "pts_check.size()" << pts_check.size() << " id_cps_curr=" << id_cps_curr << endl;
      Eigen::Vector3d pt = traj.getPos(t);
      if (t < 1e-5 || pts_check[id_cps_curr].empty() ||
          (pt - pt_last).cwiseAbs().maxCoeff() > RES_2)
      {
        pts_check[id_cps_curr].emplace_back(std::pair<double, Eigen::Vector3d>(t, pt));
        pt_last = pt;
      }

      t += t_step;
    }
    // ROS_ERROR("C");

    return true;
  }

  /* check collision and set {p,v} pairs to constrain points */
  PolyTrajOptimizer::CHK_RET PolyTrajOptimizer::finelyCheckAndSetConstraintPoints(
      std::vector<std::pair<int, int>> &segments,
      const poly_traj::MinJerkOpt &pt_data,
      const bool flag_first_init /*= true*/,
      std::string *reason)
  {
    if (reason)
      reason->clear();

    Eigen::MatrixXd init_points = pt_data.getInitConstraintPoints(cps_num_prePiece_);
    poly_traj::Trajectory traj = pt_data.getTraj();

    if (flag_first_init)
    {
      cps_.resize_cp(init_points.cols());
      cps_.points = init_points;
    }

    /*** Segment the initial trajectory according to obstacles ***/
    vector<std::pair<int, int>> segment_ids;
    constexpr int ENOUGH_INTERVAL = 2;
    // double step_size = grid_map_->getResolution() / ((init_points.col(0) - init_points.rightCols(1)).norm() / (init_points.cols() - 1)) / 1.5;
    int in_id = -1, out_id = -1;
    int same_occ_state_times = ENOUGH_INTERVAL + 1;
    bool occ, last_occ = false;
    bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
    int i_end = ConstraintPoints::two_thirds_id(init_points, touch_goal_); // only check closed 2/3 points.

    PtsChk_t pts_check;
    if (!computePointsToCheck(traj, i_end, pts_check, touch_goal_))
    {
      if (reason)
        *reason = "PTS_CHECK_BUILD_FAILED";
      return CHK_RET::ERR;
    }

    for (int i = 0; i < i_end; ++i)
    {
      for (size_t j = 0; j < pts_check[i].size(); ++j)
      {
        occ = grid_map_->getInflateOccupancy(pts_check[i][j].second);

        if (occ && !last_occ)
        {
          if (same_occ_state_times > ENOUGH_INTERVAL || i == 0)
          {
            in_id = i;
            flag_got_start = true;
          }
          same_occ_state_times = 0;
          flag_got_end_maybe = false; // terminate in advance
        }
        else if (!occ && last_occ)
        {
          out_id = i + 1;
          flag_got_end_maybe = true;
          same_occ_state_times = 0;
        }
        else
        {
          ++same_occ_state_times;
        }

        if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == i_end - 1)))
        {
          flag_got_end_maybe = false;
          flag_got_end = true;
        }

        last_occ = occ;

        if (flag_got_start && flag_got_end)
        {
          flag_got_start = false;
          flag_got_end = false;
          if (in_id < 0 || out_id < 0)
          {
            ROS_ERROR("Should not happen! in_id=%d, out_id=%d", in_id, out_id);
            if (reason)
              *reason = "SEGMENT_ENDPOINT_INVALID";
            return CHK_RET::ERR;
          }
          segment_ids.push_back(std::pair<int, int>(in_id, out_id));
        }
      }
    }

    /* Collision free and return in advance */
    if (segment_ids.size() == 0)
    {

      // if (!flag_first_init)
      // {
      //   pts_check_ = pts_check; // collision free and "flag_first_init == 0" means in the test procedure after planning
      // }

      if (reason)
        *reason = "OBS_FREE";
      return CHK_RET::OBS_FREE;
    }

    // A non-empty segment means that the trajectory intersects the inflated
    // static map.  The caller may use FINISH to request a corridor rebuild,
    // but for final candidate validation this is an explicit collision rather
    // than an unclassified failure.
    if (reason)
      *reason = "STATIC_COLLISION";

    /*** a star search ***/
    vector<vector<Eigen::Vector3d>> a_star_pathes;
    for (size_t i = 0; i < segment_ids.size(); ++i)
    {
      // Search from back to head
      Eigen::Vector3d in(init_points.col(segment_ids[i].second)), out(init_points.col(segment_ids[i].first));
      ASTAR_RET ret = a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ grid_map_->getResolution(), in, out);
      if (ret == ASTAR_RET::SUCCESS)
      {
        a_star_pathes.push_back(a_star_->getPath());
      }
      else if (ret == ASTAR_RET::SEARCH_ERR && i + 1 < segment_ids.size()) // connect the next segment
      {
        segment_ids[i].second = segment_ids[i + 1].second;
        segment_ids.erase(segment_ids.begin() + i + 1);
        --i;
        ROS_WARN("A conor case 2, I have never exeam it.");
      }
      else
      {
        ROS_ERROR("A-star error, force return!");
        if (reason)
          *reason = "A_STAR_ERROR";
        return CHK_RET::ERR;
      }
    }

    /*** calculate bounds ***/
    int id_low_bound, id_up_bound;
    vector<std::pair<int, int>> bounds(segment_ids.size());
    for (size_t i = 0; i < segment_ids.size(); i++)
    {

      if (i == 0) // first segment
      {
        id_low_bound = 1;
        if (segment_ids.size() > 1)
        {
          id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
        }
        else
        {
          id_up_bound = init_points.cols() - 2;
        }
      }
      else if (i == segment_ids.size() - 1) // last segment, i != 0 here
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = init_points.cols() - 2;
      }
      else
      {
        id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
        id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2);  // id_up_bound : -1.0f fix()
      }

      bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
    }

    /*** Adjust segment length ***/
    vector<std::pair<int, int>> adjusted_segment_ids(segment_ids.size());
    constexpr double MINIMUM_PERCENT = 0.0; // Each segment is guaranteed to have sufficient points to generate sufficient force
    int minimum_points = round(init_points.cols() * MINIMUM_PERCENT), num_points;
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      /*** Adjust segment length ***/
      num_points = segment_ids[i].second - segment_ids[i].first + 1;
      //cout << "i = " << i << " first = " << segment_ids[i].first << " second = " << segment_ids[i].second << endl;
      if (num_points < minimum_points)
      {
        double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

        adjusted_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first
                                            ? segment_ids[i].first - add_points_each_side
                                            : bounds[i].first;

        adjusted_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second
                                             ? segment_ids[i].second + add_points_each_side
                                             : bounds[i].second;
      }
      else
      {
        adjusted_segment_ids[i].first = segment_ids[i].first;
        adjusted_segment_ids[i].second = segment_ids[i].second;
      }
    }

    for (size_t i = 1; i < adjusted_segment_ids.size(); i++) // Avoid overlap
    {
      if (adjusted_segment_ids[i - 1].second >= adjusted_segment_ids[i].first)
      {
        double middle = (double)(adjusted_segment_ids[i - 1].second + adjusted_segment_ids[i].first) / 2.0;
        adjusted_segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
        adjusted_segment_ids[i].first = static_cast<int>(middle + 1.1);
      }
    }

    // Used for return
    vector<std::pair<int, int>> final_segment_ids;

    /*** Assign data to each segment ***/
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
      // step 1
      for (int j = adjusted_segment_ids[i].first; j <= adjusted_segment_ids[i].second; ++j)
        cps_.flag_temp[j] = false;

      // step 2
      int got_intersection_id = -1;
      for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
      {
        Eigen::Vector3d ctrl_pts_law(init_points.col(j + 1) - init_points.col(j - 1)), intersection_point;
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law), init_val = val;
        while (true)
        {

          last_Astar_id = Astar_id;

          if (val >= 0)
          {
            ++Astar_id; // Previous Astar search from back to head
            if (Astar_id >= (int)a_star_pathes[i].size())
            {
              break;
            }
          }
          else
          {
            --Astar_id;
            if (Astar_id < 0)
            {
              break;
            }
          }

          val = (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law);

          if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
          {
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(init_points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                );

            got_intersection_id = j;
            break;
          }
        }

        if (got_intersection_id >= 0)
        {
          double length = (intersection_point - init_points.col(j)).norm();
          if (length > 1e-5)
          {
            cps_.flag_temp[j] = true;
            for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
            {
              bool occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * init_points.col(j));

              if (occ || a < grid_map_->getResolution())
              {
                if (occ)
                  a += grid_map_->getResolution();
                cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * init_points.col(j));
                cps_.direction[j].push_back((intersection_point - init_points.col(j)).normalized());
                break;
              }
            }
          }
          else
          {
            got_intersection_id = -1;
          }
        }
      }

      /* Corner case: the segment length is too short. Here the control points may outside the A* path, leading to opposite gradient direction. So I have to take special care of it */
      if (segment_ids[i].second - segment_ids[i].first == 1)
      {
        Eigen::Vector3d ctrl_pts_law(init_points.col(segment_ids[i].second) - init_points.col(segment_ids[i].first)), intersection_point;
        Eigen::Vector3d middle_point = (init_points.col(segment_ids[i].second) + init_points.col(segment_ids[i].first)) / 2;
        int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
        double val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law), init_val = val;
        while (true)
        {

          last_Astar_id = Astar_id;

          if (val >= 0)
          {
            ++Astar_id; // Previous Astar search from back to head
            if (Astar_id >= (int)a_star_pathes[i].size())
            {
              break;
            }
          }
          else
          {
            --Astar_id;
            if (Astar_id < 0)
            {
              break;
            }
          }

          val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law);

          if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
          {
            intersection_point =
                a_star_pathes[i][Astar_id] +
                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                 (ctrl_pts_law.dot(middle_point - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                );

            if ((intersection_point - middle_point).norm() > 0.01) // 1cm.
            {
              cps_.flag_temp[segment_ids[i].first] = true;
              cps_.base_point[segment_ids[i].first].push_back(init_points.col(segment_ids[i].first));
              cps_.direction[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

              got_intersection_id = segment_ids[i].first;
            }
            break;
          }
        }
      }

      //step 3
      if (got_intersection_id >= 0)
      {
        for (int j = got_intersection_id + 1; j <= adjusted_segment_ids[i].second; ++j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
            cps_.direction[j].push_back(cps_.direction[j - 1].back());
          }

        for (int j = got_intersection_id - 1; j >= adjusted_segment_ids[i].first; --j)
          if (!cps_.flag_temp[j])
          {
            cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
            cps_.direction[j].push_back(cps_.direction[j + 1].back());
          }

        final_segment_ids.push_back(adjusted_segment_ids[i]);
      }
      else
      {
        // Just ignore, it does not matter ^_^.
        // ROS_ERROR("Failed to generate direction! segment_id=%d", i);
      }
    }

    segments = final_segment_ids;
    return CHK_RET::FINISH;
  }

  bool PolyTrajOptimizer::roughlyCheckConstraintPoints(void)
  {

    // int end_idx = cps_.cp_size - 1;

    /*** Check and segment the initial trajectory according to obstacles ***/
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    bool flag_new_obs_valid = false;
    int i_end = ConstraintPoints::two_thirds_id(cps_.points, touch_goal_); // only check closed 2/3 points.
    for (int i = 1; i <= i_end; ++i)
    {

      bool occ = grid_map_->getInflateOccupancy(cps_.points.col(i));

      /*** check if the new collision will be valid ***/
      if (occ)
      {
        for (size_t k = 0; k < cps_.direction[i].size(); ++k)
        {
          if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < 1 * grid_map_->getResolution()) // current point is outside all the collision_points.
          {
            occ = false;
            break;
          }
        }
      }

      if (occ)
      {
        flag_new_obs_valid = true;

        int j;
        for (j = i - 1; j >= 0; --j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));
          if (!occ)
          {
            in_id = j;
            break;
          }
        }
        if (j < 0) // fail to get the obs free point
        {
          ROS_ERROR("ERROR! the drone is in obstacle. It means a crash in real-world.");
          in_id = 0;
        }

        for (j = i + 1; j < cps_.cp_size; ++j)
        {
          occ = grid_map_->getInflateOccupancy(cps_.points.col(j));

          if (!occ)
          {
            out_id = j;
            break;
          }
        }
        if (j >= cps_.cp_size) // fail to get the obs free point
        {
          ROS_WARN("WARN! terminal point of the current trajectory is in obstacle, skip this planning.");

          force_stop_type_ = STOP_FOR_ERROR;
          return false;
        }

        i = j + 1;

        segment_ids.push_back(std::pair<int, int>(in_id, out_id));
      }
    }

    if (flag_new_obs_valid)
    {
      vector<vector<Eigen::Vector3d>> a_star_pathes;
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        /*** a star search ***/
        Eigen::Vector3d in(cps_.points.col(segment_ids[i].second)), out(cps_.points.col(segment_ids[i].first));
        ASTAR_RET ret = a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ grid_map_->getResolution(), in, out);
        if (ret == ASTAR_RET::SUCCESS)
        {
          a_star_pathes.push_back(a_star_->getPath());
        }
        else if (ret == ASTAR_RET::SEARCH_ERR && i + 1 < segment_ids.size()) // connect the next segment
        {
          segment_ids[i].second = segment_ids[i + 1].second;
          segment_ids.erase(segment_ids.begin() + i + 1);
          --i;
          ROS_WARN("A conor case 2, I have never exeam it.");
        }
        else
        {
          ROS_ERROR("A-star error");
          segment_ids.erase(segment_ids.begin() + i);
          --i;
        }
      }

      for (size_t i = 1; i < segment_ids.size(); i++) // Avoid overlap
      {
        if (segment_ids[i - 1].second >= segment_ids[i].first)
        {
          double middle = (double)(segment_ids[i - 1].second + segment_ids[i].first) / 2.0;
          segment_ids[i - 1].second = static_cast<int>(middle - 0.1);
          segment_ids[i].first = static_cast<int>(middle + 1.1);
        }
      }

      /*** Assign parameters to each segment ***/
      for (size_t i = 0; i < segment_ids.size(); ++i)
      {
        // step 1
        for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
          cps_.flag_temp[j] = false;

        // step 2
        int got_intersection_id = -1;
        for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
        {
          Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
          int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
          double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), init_val = val;
          while (true)
          {

            last_Astar_id = Astar_id;

            if (val >= 0)
            {
              ++Astar_id; // Previous Astar search from back to head
              if (Astar_id >= (int)a_star_pathes[i].size())
              {
                break;
              }
            }
            else
            {
              --Astar_id;
              if (Astar_id < 0)
              {
                break;
              }
            }

            val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

            if (val * init_val <= 0 && (abs(val) > 0 || abs(init_val) > 0)) // val = init_val = 0.0 is not allowed
            {
              intersection_point =
                  a_star_pathes[i][Astar_id] +
                  ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                   (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                  );

              got_intersection_id = j;
              break;
            }
          }

          if (got_intersection_id >= 0)
          {
            double length = (intersection_point - cps_.points.col(j)).norm();
            if (length > 1e-5)
            {
              cps_.flag_temp[j] = true;
              for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
              {
                bool occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));

                if (occ || a < grid_map_->getResolution())
                {
                  if (occ)
                    a += grid_map_->getResolution();
                  cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                  cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                  break;
                }
              }
            }
            else
            {
              got_intersection_id = -1;
            }
          }
        }

        //step 3
        if (got_intersection_id >= 0)
        {
          for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
              cps_.direction[j].push_back(cps_.direction[j - 1].back());
            }

          for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
            if (!cps_.flag_temp[j])
            {
              cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
              cps_.direction[j].push_back(cps_.direction[j + 1].back());
            }
        }
        else
          ROS_WARN("Failed to generate direction. It doesn't matter.");
      }

      force_stop_type_ = STOP_FOR_REBOUND;
      return true;
    }

    return false;
  }

  /* multi-topo support */
  std::vector<ConstraintPoints> PolyTrajOptimizer::distinctiveTrajs(vector<std::pair<int, int>> segments)
  {
    if (segments.size() == 0) // will be invoked again later.
    {
      std::vector<ConstraintPoints> oneSeg;
      oneSeg.push_back(cps_);
      return oneSeg;
    }

    constexpr int MAX_TRAJS = 8;
    constexpr int VARIS = 2;
    int seg_upbound = std::min((int)segments.size(), static_cast<int>(floor(log(MAX_TRAJS) / log(VARIS))));
    std::vector<ConstraintPoints> control_pts_buf;
    control_pts_buf.reserve(MAX_TRAJS);
    const double RESOLUTION = grid_map_->getResolution();
    const double CTRL_PT_DIST = (cps_.points.col(0) - cps_.points.col(cps_.cp_size - 1)).norm() / (cps_.cp_size - 1);

    // Step 1. Find the opposite vectors and base points for every segment.
    std::vector<std::pair<ConstraintPoints, ConstraintPoints>> RichInfoSegs;
    for (int i = 0; i < seg_upbound; i++)
    {
      std::pair<ConstraintPoints, ConstraintPoints> RichInfoOneSeg;
      ConstraintPoints RichInfoOneSeg_temp;
      cps_.segment(RichInfoOneSeg_temp, segments[i].first, segments[i].second);
      RichInfoOneSeg.first = RichInfoOneSeg_temp;
      RichInfoOneSeg.second = RichInfoOneSeg_temp;
      RichInfoSegs.push_back(RichInfoOneSeg);
    }

    for (int i = 0; i < seg_upbound; i++)
    {

      // 1.1 Find the start occupied point id and the last occupied point id
      if (RichInfoSegs[i].first.cp_size > 1)
      {
        int occ_start_id = -1, occ_end_id = -1;
        Eigen::Vector3d occ_start_pt, occ_end_pt;
        for (int j = 0; j < RichInfoSegs[i].first.cp_size - 1; j++)
        {
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j + 1)).norm() / 2;
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j + 1));
            if (grid_map_->getInflateOccupancy(pt))
            {
              occ_start_id = j;
              occ_start_pt = pt;
              goto exit_multi_loop1;
            }
          }
        }
      exit_multi_loop1:;
        for (int j = RichInfoSegs[i].first.cp_size - 1; j >= 1; j--)
        {
          ;
          double step_size = RESOLUTION / (RichInfoSegs[i].first.points.col(j) - RichInfoSegs[i].first.points.col(j - 1)).norm();
          for (double a = 1; a > 0; a -= step_size)
          {
            Eigen::Vector3d pt(a * RichInfoSegs[i].first.points.col(j) + (1 - a) * RichInfoSegs[i].first.points.col(j - 1));
            if (grid_map_->getInflateOccupancy(pt))
            {
              occ_end_id = j;
              occ_end_pt = pt;
              goto exit_multi_loop2;
            }
          }
        }
      exit_multi_loop2:;

        // double check
        if (occ_start_id == -1 || occ_end_id == -1)
        {
          // It means that the first or the last control points of one segment are in obstacles, which is not allowed.
          // ROS_WARN("What? occ_start_id=%d, occ_end_id=%d", occ_start_id, occ_end_id);

          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;

          continue;
        }

        // 1.2 Reverse the vector and find new base points from occ_start_id to occ_end_id.
        for (int j = occ_start_id; j <= occ_end_id; j++)
        {
          Eigen::Vector3d base_pt_reverse, base_vec_reverse;
          if (RichInfoSegs[i].first.base_point[j].size() != 1)
          {
            cout << "RichInfoSegs[" << i << "].first.base_point[" << j << "].size()=" << RichInfoSegs[i].first.base_point[j].size() << endl;
            ROS_ERROR("Wrong number of base_points!!! Should not be happen!.");

            cout << setprecision(5);
            cout << "cps_" << endl;
            cout << " clearance=" << obs_clearance_ << " cps.size=" << cps_.cp_size << endl;
            for (int temp_i = 0; temp_i < cps_.cp_size; temp_i++)
            {
              if (cps_.base_point[temp_i].size() > 1 && cps_.base_point[temp_i].size() < 1000)
              {
                ROS_ERROR("Should not happen!!!");
                cout << "######" << cps_.points.col(temp_i).transpose() << endl;
                for (size_t temp_j = 0; temp_j < cps_.base_point[temp_i].size(); temp_j++)
                  cout << "      " << cps_.base_point[temp_i][temp_j].transpose() << " @ " << cps_.direction[temp_i][temp_j].transpose() << endl;
              }
            }

            std::vector<ConstraintPoints> blank;
            return blank;
          }

          base_vec_reverse = -RichInfoSegs[i].first.direction[j][0];

          // The start and the end case must get taken special care of.
          if (j == occ_start_id)
          {
            base_pt_reverse = occ_start_pt;
          }
          else if (j == occ_end_id)
          {
            base_pt_reverse = occ_end_pt;
          }
          else
          {
            base_pt_reverse = RichInfoSegs[i].first.points.col(j) + base_vec_reverse * (RichInfoSegs[i].first.base_point[j][0] - RichInfoSegs[i].first.points.col(j)).norm();
          }

          if (grid_map_->getInflateOccupancy(base_pt_reverse)) // Search outward.
          {
            double l_upbound = 5 * CTRL_PT_DIST; // "5" is the threshold.
            double l = RESOLUTION;
            for (; l <= l_upbound; l += RESOLUTION)
            {
              Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
              if (!grid_map_->getInflateOccupancy(base_pt_temp))
              {
                RichInfoSegs[i].second.base_point[j][0] = base_pt_temp;
                RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
                break;
              }
            }
            if (l > l_upbound)
            {
              ROS_WARN("Can't find the new base points at the opposite within the threshold. i=%d, j=%d", i, j);

              segments.erase(segments.begin() + i);
              RichInfoSegs.erase(RichInfoSegs.begin() + i);
              seg_upbound--;
              i--;

              goto exit_multi_loop3; // break "for (int j = 0; j < RichInfoSegs[i].first.size; j++)"
            }
          }
          else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(j)).norm() >= RESOLUTION) // Unnecessary to search.
          {
            RichInfoSegs[i].second.base_point[j][0] = base_pt_reverse;
            RichInfoSegs[i].second.direction[j][0] = base_vec_reverse;
          }
          else
          {
            ROS_WARN("base_point and control point are too close!");
            cout << "base_point=" << RichInfoSegs[i].first.base_point[j][0].transpose() << " control point=" << RichInfoSegs[i].first.points.col(j).transpose() << endl;

            segments.erase(segments.begin() + i);
            RichInfoSegs.erase(RichInfoSegs.begin() + i);
            seg_upbound--;
            i--;

            goto exit_multi_loop3; // break "for (int j = 0; j < RichInfoSegs[i].first.size; j++)"
          }
        }

        // 1.3 Assign the base points to control points within [0, occ_start_id) and (occ_end_id, RichInfoSegs[i].first.size()-1].
        if (RichInfoSegs[i].second.cp_size)
        {
          for (int j = occ_start_id - 1; j >= 0; j--)
          {
            RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_start_id][0];
            RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_start_id][0];
          }
          for (int j = occ_end_id + 1; j < RichInfoSegs[i].second.cp_size; j++)
          {
            RichInfoSegs[i].second.base_point[j][0] = RichInfoSegs[i].second.base_point[occ_end_id][0];
            RichInfoSegs[i].second.direction[j][0] = RichInfoSegs[i].second.direction[occ_end_id][0];
          }
        }

      exit_multi_loop3:;
      }
      else
      {
        Eigen::Vector3d base_vec_reverse = -RichInfoSegs[i].first.direction[0][0];
        Eigen::Vector3d base_pt_reverse = RichInfoSegs[i].first.points.col(0) + base_vec_reverse * (RichInfoSegs[i].first.base_point[0][0] - RichInfoSegs[i].first.points.col(0)).norm();

        if (grid_map_->getInflateOccupancy(base_pt_reverse)) // Search outward.
        {
          double l_upbound = 5 * CTRL_PT_DIST; // "5" is the threshold.
          double l = RESOLUTION;
          for (; l <= l_upbound; l += RESOLUTION)
          {
            Eigen::Vector3d base_pt_temp = base_pt_reverse + l * base_vec_reverse;
            //cout << base_pt_temp.transpose() << endl;
            if (!grid_map_->getInflateOccupancy(base_pt_temp))
            {
              RichInfoSegs[i].second.base_point[0][0] = base_pt_temp;
              RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
              break;
            }
          }
          if (l > l_upbound)
          {
            ROS_WARN("Can't find the new base points at the opposite within the threshold, 2. i=%d", i);

            segments.erase(segments.begin() + i);
            RichInfoSegs.erase(RichInfoSegs.begin() + i);
            seg_upbound--;
            i--;
          }
        }
        else if ((base_pt_reverse - RichInfoSegs[i].first.points.col(0)).norm() >= RESOLUTION) // Unnecessary to search.
        {
          RichInfoSegs[i].second.base_point[0][0] = base_pt_reverse;
          RichInfoSegs[i].second.direction[0][0] = base_vec_reverse;
        }
        else
        {
          ROS_WARN("base_point and control point are too close!, 2");
          cout << "base_point=" << RichInfoSegs[i].first.base_point[0][0].transpose() << " control point=" << RichInfoSegs[i].first.points.col(0).transpose() << endl;

          segments.erase(segments.begin() + i);
          RichInfoSegs.erase(RichInfoSegs.begin() + i);
          seg_upbound--;
          i--;
        }
      }
    }

    // Step 2. Assemble each segment to make up the new control point sequence.
    if (seg_upbound == 0) // After the erase operation above, segment legth will decrease to 0 again.
    {
      std::vector<ConstraintPoints> oneSeg;
      oneSeg.push_back(cps_);
      return oneSeg;
    }

    // cout << "A4" << endl;

    std::vector<int> selection(seg_upbound);
    std::fill(selection.begin(), selection.end(), 0);
    selection[0] = -1; // init
    int max_traj_nums = static_cast<int>(pow(VARIS, seg_upbound));
    for (int i = 0; i < max_traj_nums; i++)
    {
      // 2.1 Calculate the selection table.
      int digit_id = 0;
      selection[digit_id]++;
      while (digit_id < seg_upbound && selection[digit_id] >= VARIS)
      {
        selection[digit_id] = 0;
        digit_id++;
        if (digit_id >= seg_upbound)
        {
          ROS_ERROR("Should not happen!!! digit_id=%d, seg_upbound=%d", digit_id, seg_upbound);
        }
        selection[digit_id]++;
      }

      // 2.2 Assign params according to the selection table.
      ConstraintPoints cpsOneSample;
      cpsOneSample.resize_cp(cps_.cp_size);
      int cp_id = 0, seg_id = 0, cp_of_seg_id = 0;
      while (/*seg_id < RichInfoSegs.size() ||*/ cp_id < cps_.cp_size)
      {

        if (seg_id >= seg_upbound || cp_id < segments[seg_id].first || cp_id > segments[seg_id].second)
        {
          cpsOneSample.points.col(cp_id) = cps_.points.col(cp_id);
          cpsOneSample.base_point[cp_id] = cps_.base_point[cp_id];
          cpsOneSample.direction[cp_id] = cps_.direction[cp_id];
        }
        else if (cp_id >= segments[seg_id].first && cp_id <= segments[seg_id].second)
        {
          if (!selection[seg_id]) // zx-todo
          {
            cpsOneSample.points.col(cp_id) = RichInfoSegs[seg_id].first.points.col(cp_of_seg_id);
            cpsOneSample.base_point[cp_id] = RichInfoSegs[seg_id].first.base_point[cp_of_seg_id];
            cpsOneSample.direction[cp_id] = RichInfoSegs[seg_id].first.direction[cp_of_seg_id];
            cp_of_seg_id++;
          }
          else
          {
            if (RichInfoSegs[seg_id].second.cp_size)
            {
              cpsOneSample.points.col(cp_id) = RichInfoSegs[seg_id].second.points.col(cp_of_seg_id);
              cpsOneSample.base_point[cp_id] = RichInfoSegs[seg_id].second.base_point[cp_of_seg_id];
              cpsOneSample.direction[cp_id] = RichInfoSegs[seg_id].second.direction[cp_of_seg_id];
              cp_of_seg_id++;
            }
            else
            {
              // Abandon this trajectory.
              goto abandon_this_trajectory;
            }
          }

          if (cp_id == segments[seg_id].second)
          {
            cp_of_seg_id = 0;
            seg_id++;
          }
        }
        else
        {
          ROS_ERROR("Shold not happen!!!!, cp_id=%d, seg_id=%d, segments.front().first=%d, segments.back().second=%d, segments[seg_id].first=%d, segments[seg_id].second=%d",
                    cp_id, seg_id, segments.front().first, segments.back().second, segments[seg_id].first, segments[seg_id].second);
        }

        cp_id++;
      }

      control_pts_buf.push_back(cpsOneSample);

    abandon_this_trajectory:;
    }

    return control_pts_buf;
  }

  /* mappings between real world time and unconstrained virtual time */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT)
  {
    for (int i = 0; i < RT.size(); ++i)
    {
      VT(i) = RT(i) > 1.0 ? (sqrt(2.0 * RT(i) - 1.0) - 1.0)
                          : (1.0 - sqrt(2.0 / RT(i) - 1.0));
    }
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
  }

  template <typename EIGENVEC, typename EIGENVECGD>
  void PolyTrajOptimizer::VirtualTGradCost(
      const Eigen::VectorXd &RT, const EIGENVEC &VT,
      const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
      double &costT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      double gdVT2Rt;
      if (VT(i) > 0)
      {
        gdVT2Rt = VT(i) + 1.0;
      }
      else
      {
        double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
        gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
      }

      gdVT(i) = (gdRT(i) + wei_time_) * gdVT2Rt;
    }

    costT = RT.sum() * wei_time_;
  }

  /* gradient and cost evaluation functions */
  template <typename EIGENVEC>
  void PolyTrajOptimizer::initAndGetSmoothnessGradCost2PT(EIGENVEC &gdT, double &cost)
  {
    jerkOpt_.initGradCost(gdT, cost);
  }

 PolyTrajOptimizer::LiveVisibilityCost
  PolyTrajOptimizer::directionalVisibilityGradCostP(
      const double t, const Eigen::Vector3d &p, const Eigen::Vector3d &v,
      const bool predicted_yaw_valid, const double predicted_yaw,
      const double predicted_yaw_rate) const
  {
    LiveVisibilityCost result;
    if (!ablation_config_.local_visibility ||
        !directionalVisibilityCostEnabled(
            encirclement_tracking_configured_,
            encirclement_visibility_guidance_active_, weight_visibility_) ||
        !std::isfinite(t) ||
        !p.allFinite() || !v.allFinite() || !object_p_.allFinite() ||
        !object_v_.allFinite())
      return result;

    const Eigen::Vector3d target = object_p_ + object_v_ * t;
    bool component_valid = false;
    using VisibilityClock = std::chrono::steady_clock;

    const auto static_begin = VisibilityClock::now();
    if (static_los_geometry_.valid())
    {
      double clearance = std::numeric_limits<double>::infinity();
      Eigen::Vector3d gradient_observer = Eigen::Vector3d::Zero();
      Eigen::Vector3d gradient_target = Eigen::Vector3d::Zero();
      int witness = -1;
      if (static_los_geometry_.querySegmentClearance(
              p, target, clearance, &gradient_observer, &gradient_target,
              &witness))
      {
        const auto risk = multi_uav_formation::directionalClearanceRisk(
            clearance, static_los_margin_,
            static_los_margin_ + directional_static_risk_transition_,
            directional_visibility_deep_risk_kappa_,
            directional_visibility_deep_risk_beta_);
        if (risk.valid)
        {
          component_valid = true;
          result.static_los_cost =
              weight_visibility_ * risk.value * risk.value;
          const double factor = 2.0 * weight_visibility_ * risk.value *
                                risk.derivative_clearance;
          const Eigen::Vector3d gradient_position =
              factor * gradient_observer;
          const Eigen::Vector3d gradient_target_position =
              factor * gradient_target;
          result.static_clearance = clearance;
          result.static_risk = risk.value;
          result.static_witness = witness;
          result.static_gradient_position = gradient_position;
          result.static_time_derivative =
              gradient_position.dot(v) +
              gradient_target_position.dot(object_v_);
          result.gradient_position += gradient_position;
          result.gradient_time += result.static_time_derivative;
          result.gradient_previous_time +=
              gradient_target_position.dot(object_v_);
          result.static_los_gradient_active =
              result.static_los_cost > 1.0e-12 &&
              gradient_position.norm() > 1.0e-9;
        }
      }
    }
    result.static_compute_ms =
        std::chrono::duration<double, std::milli>(
            VisibilityClock::now() - static_begin).count();

    const auto dynamic_begin = VisibilityClock::now();
    double best_dynamic_risk = 0.0;
    Eigen::Vector3d best_dynamic_gradient_position =
        Eigen::Vector3d::Zero();
    Eigen::Vector3d best_dynamic_gradient_target = Eigen::Vector3d::Zero();
    Eigen::Vector3d best_dynamic_gradient_center = Eigen::Vector3d::Zero();
    Eigen::Vector3d best_dynamic_velocity = Eigen::Vector3d::Zero();
    double best_dynamic_clearance = std::numeric_limits<double>::infinity();
    int best_dynamic_witness = -1;
    if (moving_objs_ && moving_objs_->getObjNums() > 0 &&
        (moving_obj_prediction_horizon_ <= 1.0e-3 ||
         t <= moving_obj_prediction_horizon_))
    {
      const double query_time = t_now_ + t;
      for (int id = 0; id < moving_objs_->getObjNums(); ++id)
      {
        if (!moving_objs_->hasPrediction(id))
          continue;
        const Eigen::Vector3d center =
            moving_objs_->evaluateConstVel(id, query_time);
        const Eigen::Vector3d center_velocity =
            moving_objs_->evaluateConstVelVelocity(id, query_time);
        const Eigen::Vector3d scale = moving_objs_->getObjScale(id);
        if (!center.allFinite() || !center_velocity.allFinite() ||
            !scale.allFinite() || scale.x() <= 0.0 || scale.y() <= 0.0 ||
            scale.z() <= 0.0)
          continue;
        const auto clearance =
            multi_uav_formation::segmentVerticalCylinderClearanceGradient(
                p, target, center,
                0.5 * std::max(scale.x(), scale.y()), scale.z());
        if (!clearance.valid || !std::isfinite(clearance.clearance))
          continue;
        const auto risk = multi_uav_formation::directionalClearanceRisk(
            clearance.clearance, static_los_margin_,
            static_los_margin_ + directional_moving_risk_transition_,
            directional_visibility_deep_risk_kappa_,
            directional_visibility_deep_risk_beta_);
        if (!risk.valid)
          continue;
        component_valid = true;
        if (risk.value > best_dynamic_risk)
        {
          best_dynamic_risk = risk.value;
          const double factor = 2.0 * weight_visibility_ * risk.value *
                                risk.derivative_clearance;
          best_dynamic_gradient_position = factor * clearance.gradient_start;
          best_dynamic_gradient_target = factor * clearance.gradient_end;
          best_dynamic_gradient_center = factor * clearance.gradient_center;
          best_dynamic_velocity = center_velocity;
          best_dynamic_clearance = clearance.clearance;
          best_dynamic_witness = id;
        }
      }
    }
    result.dynamic_los_cost =
        weight_visibility_ * best_dynamic_risk * best_dynamic_risk;
    result.dynamic_clearance = best_dynamic_clearance;
    result.dynamic_risk = best_dynamic_risk;
    result.dynamic_witness = best_dynamic_witness;
    result.dynamic_gradient_position = best_dynamic_gradient_position;
    result.gradient_position += best_dynamic_gradient_position;
    result.dynamic_time_derivative =
        best_dynamic_gradient_position.dot(v) +
        best_dynamic_gradient_target.dot(object_v_) +
        best_dynamic_gradient_center.dot(best_dynamic_velocity);
    result.gradient_time += result.dynamic_time_derivative;
    result.gradient_previous_time +=
        best_dynamic_gradient_target.dot(object_v_) +
        best_dynamic_gradient_center.dot(best_dynamic_velocity);
    result.dynamic_los_gradient_active =
        result.dynamic_los_cost > 1.0e-12 &&
        best_dynamic_gradient_position.norm() > 1.0e-9;
    result.dynamic_compute_ms =
        std::chrono::duration<double, std::milli>(
            VisibilityClock::now() - dynamic_begin).count();

    const auto fov_begin = VisibilityClock::now();
    if (predicted_yaw_valid && std::isfinite(predicted_yaw) &&
        std::isfinite(predicted_yaw_rate) &&
        std::isfinite(visibility_camera_hfov_) &&
        std::isfinite(visibility_camera_vfov_))
    {
      multi_uav_formation::TrackingCameraContract camera;
      camera.horizontal_fov = visibility_camera_hfov_;
      camera.vertical_fov = visibility_camera_vfov_;
      camera.min_range = visibility_camera_min_range_;
      camera.max_range = visibility_camera_max_range_;
      const auto fov =
          multi_uav_formation::trackingCameraFovDirectionalRisk(
              p, predicted_yaw, target, camera);
      if (fov.valid)
      {
        component_valid = true;
        result.fov_cost = weight_visibility_ * fov.risk * fov.risk;
        const double factor = 2.0 * weight_visibility_ * fov.risk;
        const Eigen::Vector3d gradient_position =
            factor * fov.gradient_body_position;
        const Eigen::Vector3d gradient_target_position =
            factor * fov.gradient_target_position;
        result.horizontal_fov_margin = fov.horizontal_margin;
        result.vertical_fov_margin = fov.vertical_margin;
        result.fov_risk = fov.risk;
        result.fov_gradient_position = gradient_position;
        result.fov_time_derivative =
            gradient_position.dot(v) +
            gradient_target_position.dot(object_v_) +
            factor * fov.gradient_yaw * predicted_yaw_rate;
        result.gradient_position += gradient_position;
        result.gradient_time += result.fov_time_derivative;
        result.gradient_previous_time +=
            gradient_target_position.dot(object_v_) +
            factor * fov.gradient_yaw * predicted_yaw_rate;
        result.fov_gradient_active = result.fov_cost > 1.0e-12 &&
                                     gradient_position.norm() > 1.0e-9;
      }
    }
    result.fov_compute_ms =
        std::chrono::duration<double, std::milli>(
            VisibilityClock::now() - fov_begin).count();

    result.valid = component_valid;
    result.weighted_cost = result.static_los_cost +
                           result.dynamic_los_cost + result.fov_cost;
    // 阶段 D：LOCAL_VIS 真实 runtime 证据。COST_EVAL 只证明进入了函数；
    // 这三个计数器证明"门开 + 真实采样 + 非零 cost/gradient"。NO_LOCAL_VIS
    // 模式下门在上层关闭，三个计数器必须保持 0。
    ++local_vis_active_sample_count_;
    if (result.weighted_cost > 1.0e-12) ++local_vis_nonzero_cost_count_;
    if (result.gradient_position.norm() > 1.0e-12) ++local_vis_nonzero_grad_count_;
    const std::chrono::steady_clock::time_point now_steady =
        std::chrono::steady_clock::now();
    const double now_steady_s =
        std::chrono::duration<double>(now_steady.time_since_epoch()).count();
    if (now_steady_s - local_vis_last_log_time_s_ > 5.0)
    {
      local_vis_last_log_time_s_ = now_steady_s;
      ROS_INFO("[local-vis-counters] LOCAL_VIS_ACTIVE_SAMPLE_COUNT=%lu "
               "LOCAL_VIS_NONZERO_COST_COUNT=%lu "
               "LOCAL_VIS_NONZERO_GRAD_COUNT=%lu",
               static_cast<unsigned long>(local_vis_active_sample_count_),
               static_cast<unsigned long>(local_vis_nonzero_cost_count_),
               static_cast<unsigned long>(local_vis_nonzero_grad_count_));
    }
    return result;
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::addDirectionalVisibilityGradCost2CT(
      EIGENVEC &gdT, double &visibility_cost)
  {
    const auto visibility_begin = std::chrono::steady_clock::now();
    visibility_cost = 0.0;
    if (!ablation_config_.local_visibility)
      return;

    // Count entry into the local J_vis evaluation module.  The existing
    // guidance/context predicate below still owns whether this particular
    // candidate has a non-zero directional objective.  Keeping those two
    // facts separate lets the ablation prove that NO_LOCAL_VIS bypassed the
    // module without changing FULL's established candidate semantics.
    traj_utils::incrementAblationCounter(
        traj_utils::AblationCounter::LOCAL_VIS_COST_EVAL,
        "poly_traj_optimizer", ablation_config_);

    if (!directionalVisibilityCostEnabled(
            encirclement_tracking_configured_,
            encirclement_visibility_guidance_active_, weight_visibility_) ||
        piece_num_ <= 0 ||
        directional_visibility_sample_dt_ <= 0.0)
      return;

    // Evaluate every quadrature point from the same measured yaw state.  The
    // former sequential rollout made the objective (and its gradient) depend
    // on the number and ordering of quadrature samples.  A direct horizon
    // rollout is sample-order invariant and its returned yaw rate is exactly
    // d(yaw)/d(trajectory_time) away from the predictor's branch boundaries.
    multi_uav_formation::PredictedYawState initial_yaw_state;
    initial_yaw_state.valid = visibility_yaw_valid_ &&
                              std::isfinite(visibility_yaw_) &&
                              std::isfinite(visibility_yaw_rate_);
    initial_yaw_state.yaw = visibility_yaw_;
    initial_yaw_state.yaw_rate = visibility_yaw_rate_;
    double elapsed = 0.0;
    int previous_static_witness = -1;
    int previous_dynamic_witness = -1;
    std::vector<double> witness_switch_gradients;
    for (int piece = 0; piece < piece_num_; ++piece)
    {
      const double duration = jerkOpt_.get_T1()(piece);
      const int intervals = std::max(
          1, static_cast<int>(std::ceil(duration /
                                       directional_visibility_sample_dt_)));
      const double step = duration / static_cast<double>(intervals);
      const Eigen::Matrix<double, 6, 3> &coefficients =
          jerkOpt_.get_b().block<6, 3>(piece * 6, 0);
      for (int sample = 0; sample <= intervals; ++sample)
      {
        if((sample & 63)==0) executionCheckpoint();
        const double local_time = step * sample;
        const double local_time2 = local_time * local_time;
        const double local_time3 = local_time2 * local_time;
        const double local_time4 = local_time2 * local_time2;
        const double local_time5 = local_time4 * local_time;
        Eigen::Matrix<double, 6, 1> beta0;
        Eigen::Matrix<double, 6, 1> beta1;
        beta0 << 1.0, local_time, local_time2, local_time3,
            local_time4, local_time5;
        beta1 << 0.0, 1.0, 2.0 * local_time, 3.0 * local_time2,
            4.0 * local_time3, 5.0 * local_time4;
        const Eigen::Vector3d position = coefficients.transpose() * beta0;
        const Eigen::Vector3d velocity = coefficients.transpose() * beta1;
        const double trajectory_time = elapsed + local_time;

        multi_uav_formation::PredictedYawState sample_yaw_state =
            initial_yaw_state;
        if (sample_yaw_state.valid && trajectory_time > 1.0e-9)
        {
          const Eigen::Vector3d target =
              object_p_ + object_v_ * trajectory_time;
          const Eigen::Vector2d bearing =
              (target - position).head<2>();
          if (bearing.norm() > 1.0e-6)
          {
            sample_yaw_state =
                multi_uav_formation::advanceTargetFacingYaw(
                    initial_yaw_state,
                    std::atan2(bearing.y(), bearing.x()), trajectory_time);
          }
        }

        const LiveVisibilityCost sample_cost =
            directionalVisibilityGradCostP(
                trajectory_time, position, velocity,
                sample_yaw_state.valid, sample_yaw_state.yaw,
                sample_yaw_state.yaw_rate);
        if (!sample_cost.valid)
          continue;

        CostGradientSnapshot &snapshot = current_cost_snapshot_;
        const int previous_samples = snapshot.directional_visibility_sample_count;
        const int samples = previous_samples + 1;
        const auto update_mean = [&](double &mean, const double value) {
          mean = (mean * previous_samples + value) /
                 static_cast<double>(samples);
        };
        snapshot.directional_visibility_sample_count = samples;
        update_mean(snapshot.directional_visibility_cost_mean,
                    sample_cost.weighted_cost);
        snapshot.directional_visibility_cost_max = std::max(
            snapshot.directional_visibility_cost_max,
            sample_cost.weighted_cost);
        update_mean(snapshot.directional_static_los_cost_mean,
                    sample_cost.static_los_cost);
        snapshot.directional_static_los_cost_max = std::max(
            snapshot.directional_static_los_cost_max,
            sample_cost.static_los_cost);
        update_mean(snapshot.directional_dynamic_los_cost_mean,
                    sample_cost.dynamic_los_cost);
        snapshot.directional_dynamic_los_cost_max = std::max(
            snapshot.directional_dynamic_los_cost_max,
            sample_cost.dynamic_los_cost);
        update_mean(snapshot.directional_fov_cost_mean,
                    sample_cost.fov_cost);
        snapshot.directional_fov_cost_max = std::max(
            snapshot.directional_fov_cost_max, sample_cost.fov_cost);
        if ((sample_cost.static_los_cost > 1.0e-12 ||
             sample_cost.dynamic_los_cost > 1.0e-12 ||
             sample_cost.fov_cost > 1.0e-12) &&
            trajectory_time <
                snapshot.visibility_support_first_trajectory_time)
        {
          snapshot.visibility_support_first_trajectory_time = trajectory_time;
          snapshot.visibility_support_first_position = position;
          snapshot.visibility_support_first_target =
              object_p_ + object_v_ * trajectory_time;
        }
        const double gradient_norm = sample_cost.gradient_position.norm();
        update_mean(snapshot.directional_visibility_grad_mean,
                    gradient_norm);
        snapshot.directional_visibility_grad_max = std::max(
            snapshot.directional_visibility_grad_max, gradient_norm);
        const bool directional_active =
            sample_cost.weighted_cost > 1.0e-12 && gradient_norm > 1.0e-9;
        snapshot.directional_visibility_activations +=
            directional_active ? 1 : 0;
        snapshot.static_los_visibility_grad_active_samples +=
            sample_cost.static_los_gradient_active ? 1 : 0;
        snapshot.dynamic_los_visibility_grad_active_samples +=
            sample_cost.dynamic_los_gradient_active ? 1 : 0;
        snapshot.fov_visibility_grad_active_samples +=
            sample_cost.fov_gradient_active ? 1 : 0;
        snapshot.directional_static_compute_ms +=
            sample_cost.static_compute_ms;
        snapshot.directional_dynamic_compute_ms +=
            sample_cost.dynamic_compute_ms;
        snapshot.directional_fov_compute_ms += sample_cost.fov_compute_ms;
        const bool static_switch =
            sample_cost.static_witness >= 0 &&
            previous_static_witness >= 0 &&
            sample_cost.static_witness != previous_static_witness;
        const bool dynamic_switch =
            sample_cost.dynamic_witness >= 0 &&
            previous_dynamic_witness >= 0 &&
            sample_cost.dynamic_witness != previous_dynamic_witness;
        if (static_switch)
          ++snapshot.static_witness_switch_count;
        if (dynamic_switch)
          ++snapshot.dynamic_witness_switch_count;
        if (static_switch || dynamic_switch)
          witness_switch_gradients.push_back(gradient_norm);
        if (sample_cost.static_witness >= 0)
          previous_static_witness = sample_cost.static_witness;
        if (sample_cost.dynamic_witness >= 0)
          previous_dynamic_witness = sample_cost.dynamic_witness;

        if (gradient_norm >= snapshot.directional_visibility_grad_max - 1.0e-12)
        {
          snapshot.directional_grad_max_piece = piece;
          snapshot.directional_grad_max_sample = sample;
          snapshot.directional_grad_max_local_time = local_time;
          snapshot.directional_grad_max_trajectory_time = trajectory_time;
          snapshot.directional_grad_max_global_time = t_now_ + trajectory_time;
          snapshot.directional_grad_max_position = position;
          snapshot.directional_grad_max_target =
              object_p_ + object_v_ * trajectory_time;
          snapshot.directional_grad_max_yaw = sample_yaw_state.yaw;
          snapshot.directional_grad_max_static_clearance =
              sample_cost.static_clearance;
          snapshot.directional_grad_max_dynamic_clearance =
              sample_cost.dynamic_clearance;
          snapshot.directional_grad_max_horizontal_margin =
              sample_cost.horizontal_fov_margin;
          snapshot.directional_grad_max_vertical_margin =
              sample_cost.vertical_fov_margin;
          snapshot.directional_grad_max_static_risk = sample_cost.static_risk;
          snapshot.directional_grad_max_dynamic_risk = sample_cost.dynamic_risk;
          snapshot.directional_grad_max_fov_risk = sample_cost.fov_risk;
          snapshot.directional_grad_max_static_component =
              sample_cost.static_gradient_position.norm();
          snapshot.directional_grad_max_dynamic_component =
              sample_cost.dynamic_gradient_position.norm();
          snapshot.directional_grad_max_fov_component =
              sample_cost.fov_gradient_position.norm();
          snapshot.directional_grad_max_static_time_derivative =
              sample_cost.static_time_derivative;
          snapshot.directional_grad_max_dynamic_time_derivative =
              sample_cost.dynamic_time_derivative;
          snapshot.directional_grad_max_fov_time_derivative =
              sample_cost.fov_time_derivative;
          snapshot.directional_grad_max_static_witness =
              sample_cost.static_witness;
          snapshot.directional_grad_max_dynamic_witness =
              sample_cost.dynamic_witness;
        }

        const double trapezoid_weight =
            (sample == 0 || sample == intervals) ? 0.5 : 1.0;
        const double alpha = static_cast<double>(sample) /
                             static_cast<double>(intervals);
        jerkOpt_.get_gdC().block<6, 3>(piece * 6, 0) +=
            trapezoid_weight * step *
            (beta0 * sample_cost.gradient_position.transpose());
        gdT(piece) += trapezoid_weight *
                      (sample_cost.weighted_cost /
                           static_cast<double>(intervals) +
                       step * alpha * sample_cost.gradient_time);
        if (piece > 0)
          gdT.head(piece).array() += trapezoid_weight * step *
                                     sample_cost.gradient_previous_time;
        visibility_cost +=
            trapezoid_weight * step * sample_cost.weighted_cost;
        current_visibility_grad_sq_ +=
            trapezoid_weight * step *
            sample_cost.gradient_position.squaredNorm();
      }
      elapsed += duration;
    }
    if (!witness_switch_gradients.empty())
    {
      std::sort(witness_switch_gradients.begin(),
                witness_switch_gradients.end());
      const size_t p95_index = static_cast<size_t>(std::ceil(
          0.95 * static_cast<double>(witness_switch_gradients.size()))) - 1;
      current_cost_snapshot_.witness_switch_grad_p95 =
          witness_switch_gradients[std::min(
              p95_index, witness_switch_gradients.size() - 1)];
      current_cost_snapshot_.witness_switch_grad_max =
          witness_switch_gradients.back();
    }
    current_cost_snapshot_.directional_visibility_integral = visibility_cost;
    current_cost_snapshot_.directional_visibility_compute_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - visibility_begin).count();
  }

  void PolyTrajOptimizer::logDirectionalVisibilityCost(void) const
  {
    if (!encirclement_visibility_guidance_active_ ||
        !optimization_diagnostics_.valid)
      return;
    const CostGradientSnapshot &snapshot = optimization_diagnostics_.final;
    ROS_INFO(
        "[directional-visibility-cost] drone=%d candidate_type=%s "
        "weight_visibility=%.6f samples=%d "
        "VISIBILITY_DIRECTIONAL_COST_ACTIVATIONS=%d "
        "J_VIS_TOTAL_MEAN=%.9f J_VIS_TOTAL_MAX=%.9f "
        "J_VIS_STATIC_LOS_MEAN=%.9f J_VIS_STATIC_LOS_MAX=%.9f "
        "J_VIS_DYNAMIC_LOS_MEAN=%.9f J_VIS_DYNAMIC_LOS_MAX=%.9f "
        "J_VIS_FOV_MEAN=%.9f J_VIS_FOV_MAX=%.9f "
        "J_VIS_SAMPLE_TOTAL_MAX=%.9f J_VIS_SAMPLE_STATIC_MAX=%.9f "
        "J_VIS_SAMPLE_DYNAMIC_MAX=%.9f J_VIS_SAMPLE_FOV_MAX=%.9f "
        "VIS_GRAD_NORM_MEAN=%.9f VIS_GRAD_NORM_MAX=%.9f "
        "VIS_COST_RATIO=%.9f VIS_GRAD_RATIO=%.9f "
        "VIS_COST_TIME_MS=%.6f STATIC_VIS_TIME_MS=%.6f "
        "DYNAMIC_VIS_TIME_MS=%.6f FOV_VIS_TIME_MS=%.6f "
        "STATIC_WITNESS_SWITCH_COUNT=%d DYNAMIC_WITNESS_SWITCH_COUNT=%d "
        "WITNESS_SWITCH_GRAD_P95=%.9f WITNESS_SWITCH_GRAD_MAX=%.9f "
        "STATIC_LOS_VIS_GRAD_ACTIVE_SAMPLES=%d "
        "DYNAMIC_LOS_VIS_GRAD_ACTIVE_SAMPLES=%d "
        "FOV_VIS_GRAD_ACTIVE_SAMPLES=%d J_VIS_INTEGRAL=%.9f "
        "guidance_valid=1",
        drone_id_, visibility_cost_candidate_.c_str(), weight_visibility_,
        snapshot.directional_visibility_sample_count,
        snapshot.directional_visibility_activations,
        snapshot.directional_visibility_cost_mean,
        snapshot.directional_visibility_cost_max,
        snapshot.directional_static_los_cost_mean,
        snapshot.directional_static_los_cost_max,
        snapshot.directional_dynamic_los_cost_mean,
        snapshot.directional_dynamic_los_cost_max,
        snapshot.directional_fov_cost_mean,
        snapshot.directional_fov_cost_max,
        snapshot.directional_visibility_cost_max,
        snapshot.directional_static_los_cost_max,
        snapshot.directional_dynamic_los_cost_max,
        snapshot.directional_fov_cost_max,
        snapshot.directional_visibility_grad_mean,
        snapshot.directional_visibility_grad_max,
        snapshot.directional_visibility_cost_ratio,
        snapshot.directional_visibility_grad_ratio,
        snapshot.directional_visibility_compute_ms,
        snapshot.directional_static_compute_ms,
        snapshot.directional_dynamic_compute_ms,
        snapshot.directional_fov_compute_ms,
        snapshot.static_witness_switch_count,
        snapshot.dynamic_witness_switch_count,
        snapshot.witness_switch_grad_p95,
        snapshot.witness_switch_grad_max,
        snapshot.static_los_visibility_grad_active_samples,
        snapshot.dynamic_los_visibility_grad_active_samples,
        snapshot.fov_visibility_grad_active_samples,
        snapshot.directional_visibility_integral);
    ROS_INFO(
        "[directional-visibility-max-sample] drone=%d candidate_type=%s "
        "planning_generation=%lu hypothesis_id=%d candidate_id=-1 "
        "piece=%d sample=%d local_time=%.9f trajectory_time=%.9f "
        "global_time=%.9f position=(%.9f,%.9f,%.9f) "
        "target=(%.9f,%.9f,%.9f) predicted_yaw=%.9f "
        "static_clearance=%.9f dynamic_clearance=%.9f "
        "horizontal_fov_margin=%.9f vertical_fov_margin=%.9f "
        "rho_static=%.9f rho_dynamic=%.9f rho_fov=%.9f "
        "grad_static=%.9f grad_dynamic=%.9f grad_fov=%.9f "
        "dt_static=%.9f dt_dynamic=%.9f dt_fov=%.9f "
        "static_witness=%d dynamic_witness=%d",
        drone_id_, visibility_cost_candidate_.c_str(),
        visibility_context_generation_, visibility_context_hypothesis_id_,
        snapshot.directional_grad_max_piece,
        snapshot.directional_grad_max_sample,
        snapshot.directional_grad_max_local_time,
        snapshot.directional_grad_max_trajectory_time,
        snapshot.directional_grad_max_global_time,
        snapshot.directional_grad_max_position.x(),
        snapshot.directional_grad_max_position.y(),
        snapshot.directional_grad_max_position.z(),
        snapshot.directional_grad_max_target.x(),
        snapshot.directional_grad_max_target.y(),
        snapshot.directional_grad_max_target.z(),
        snapshot.directional_grad_max_yaw,
        snapshot.directional_grad_max_static_clearance,
        snapshot.directional_grad_max_dynamic_clearance,
        snapshot.directional_grad_max_horizontal_margin,
        snapshot.directional_grad_max_vertical_margin,
        snapshot.directional_grad_max_static_risk,
        snapshot.directional_grad_max_dynamic_risk,
        snapshot.directional_grad_max_fov_risk,
        snapshot.directional_grad_max_static_component,
        snapshot.directional_grad_max_dynamic_component,
        snapshot.directional_grad_max_fov_component,
        snapshot.directional_grad_max_static_time_derivative,
        snapshot.directional_grad_max_dynamic_time_derivative,
        snapshot.directional_grad_max_fov_time_derivative,
        snapshot.directional_grad_max_static_witness,
        snapshot.directional_grad_max_dynamic_witness);
  }

  template <typename EIGENVEC>
  void PolyTrajOptimizer::addPVAGradCost2CT(EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K)
  {
    //
    int N = gdT.size();
    Eigen::Vector3d pos, vel, acc, jer, snap;
    Eigen::Vector3d gradp, gradv, grada, gradj;
    double costp, costv, costa, costj;
    Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
    double s1, s2, s3, s4, s5;
    double step, alpha;
    Eigen::Matrix<double, 6, 3> gradViolaPc, gradViolaVc, gradViolaAc, gradViolaJc;
    double gradViolaPt, gradViolaVt, gradViolaAt, gradViolaJt;
    double omg;
    int i_dp = 0;
    costs.setZero();
    // Keep moving-obstacle telemetry separate from costs(3), which is a
    // legacy aggregate bucket shared by moving/side/region/corridor terms.
    // The optimizer itself has always consumed the moving term below; these
    // accumulators make that fact observable without changing its weighting.
    double moving_cost_accum = 0.0;
    double moving_grad_sq_accum = 0.0;
    // Eigen::MatrixXd constraint_pts(3, N * K + 1);

        // cout << "drone_id=" << drone_id_ << " ====================================================" << endl;

    // printf("A\n");

    // int innerLoop;
    double t = 0;
    // 本轮修复（prefix 真实时间语义）：执行前缀不能再由"前几个采样点"近似。
    // 每个积分点的绝对轨迹时间 = 前面所有 piece 的真实时长之和 + 该 piece 内
    // 的局部时间，因此 T 被优化时前缀归属会自动跟着移动，而不需要任何样本
    // 计数。execution_prefix_elapsed 在推进到下一段时累加。
    double execution_prefix_elapsed = 0.0;
    for (int i = 0; i < N; ++i)
    {
      executionCheckpoint();

      const Eigen::Matrix<double, 6, 3> &c = jerkOpt_.get_b().block<6, 3>(i * 6, 0);
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;
      // innerLoop = K;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
        beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
        beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1;
        alpha = 1.0 / K * j;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;
        snap = c.transpose() * beta4;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        cps_.points.col(i_dp) = pos;

        // collision
        if (obstacleGradCostP(i_dp, pos, gradp, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          gdT(i) += omg * (costp / K + step * gradViolaPt);
          costs(0) += omg * step * costp;
        }

        // swarm
        double gradt, grad_prev_t;
        if (swarmGradCostP(i_dp, t + step * j, pos, vel, gradp, gradt, grad_prev_t, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradt;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          gdT(i) += omg * (costp / K + step * gradViolaPt);
          if (i > 0)
          {
            gdT.head(i).array() += omg * step * grad_prev_t;
          }
          costs(1) += omg * step * costp;
        }

        // time-aware moving obstacle cost
        double gradt_moving, grad_prev_t_moving;
        if (movingObjGradCostP(i_dp, t + step * j, pos, vel, gradp, gradt_moving, grad_prev_t_moving, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradt_moving;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          gdT(i) += omg * (costp / K + step * gradViolaPt);
          if (i > 0)
          {
            gdT.head(i).array() += omg * step * grad_prev_t_moving;
          }
          costs(3) += omg * step * costp;
          moving_cost_accum += omg * step * costp;
          moving_grad_sq_accum += omg * step * gradp.squaredNorm();
        }

        const double candidate_progress =
            (static_cast<double>(i) + alpha) / static_cast<double>(N);
        if (candidateSideGradCostP(candidate_progress, pos, gradp, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          gdT(i) += omg * (costp / K + step * gradViolaPt);
          costs(3) += omg * step * costp;
          current_cost_snapshot_.side_cost += omg * step * costp;
          current_side_grad_sq_ += omg * step * gradp.squaredNorm();
        }

        if (candidateSideRegionGradCostP(candidate_progress, pos, gradp, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          gdT(i) += omg * (costp / K + step * gradViolaPt);
          costs(3) += omg * step * costp;
          current_cost_snapshot_.side_region_cost += omg * step * costp;
          current_side_region_grad_sq_ += omg * step * gradp.squaredNorm();
        }

        if (candidatePreservationGradCostP(candidate_progress, pos, gradp, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          gdT(i) += omg * (costp / K + step * gradViolaPt);
          costs(3) += omg * step * costp;
          current_cost_snapshot_.preserve_cost += omg * step * costp;
          current_preserve_grad_sq_ += omg * step * gradp.squaredNorm();
        }

        // tracking
        double gradt_tracking, grad_prev_t_tracking;
        if (trackingGradCostP(i_dp, t + step * j, pos, vel, gradp, gradt_tracking, grad_prev_t_tracking, costp,
                              execution_prefix_elapsed + step * j))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradt_tracking;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          gdT(i) += omg * (costp / K + step * gradViolaPt);
          if (i > 0)
          {
            gdT.head(i).array() += omg * step * grad_prev_t_tracking;
          }
          costs(1) += omg * step * costp;
          current_cost_snapshot_.tracking_cost += omg * step * costp;
          current_tracking_grad_sq_ +=
              omg * step * gradp.squaredNorm();
        }

        // 本轮修复 4：rolling execution prefix 的前向进度软代价。rolling 规划
        // 真正有执行权的只有前缀（后续轨迹会很快替换当前轨迹），所以优化器
        // 必须意识到"加速要尽早发生"。这里只加软代价，不改变起点头状态、不
        // 引入任何硬速度下限；物理风险高时有效权重已被调用方衰减。
        // 本轮修复 4 的进度软代价改为"前缀末端单点"形式（见本函数末尾），
        // 不再逐样本铺满前缀：逐样本版本实测会让 LBFGS 线搜索全线失去可行性。
        double gradt_angular, grad_prev_t_angular;
        if (angularGradCostP(i_dp, t + step * j, pos, vel, gradp, gradt_angular, grad_prev_t_angular, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradt_angular;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaPc;
          gdT(i) += omg * (costp / K + step * gradViolaPt);
          if (i > 0)
          {
            gdT.head(i).array() += omg * step * grad_prev_t_angular;
          }
          costs(1) += omg * step * costp;
        }

        double gradt_yaw, grad_prev_t_yaw;
        if (yawGradCostPV(i_dp, t + step * j, pos, vel, acc, gradp, gradv, gradt_yaw, grad_prev_t_yaw, costp))
        {
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaVc = beta1 * gradv.transpose();
          gradViolaPt = alpha * gradt_yaw;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * (gradViolaPc + gradViolaVc);
          gdT(i) += omg * (costp / K + step * gradViolaPt);
          if (i > 0)
          {
            gdT.head(i).array() += omg * step * grad_prev_t_yaw;
          }
          costs(1) += omg * step * costp;
        }

        // feasibility
        if (feasibilityGradCostV(vel, gradv, costv))
        {
          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaVc;
          gdT(i) += omg * (costv / K + step * gradViolaVt);
          costs(2) += omg * step * costv;
        }

        if (feasibilityGradCostA(acc, grada, costa))
        {
          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaAc;
          gdT(i) += omg * (costa / K + step * gradViolaAt);
          costs(2) += omg * step * costa;
        }

        if (enable_fov_costs_ && feasibilityGradCostJ(jer, gradj, costj))
        {
          gradViolaJc = beta3 * gradj.transpose();
          gradViolaJt = alpha * gradj.transpose() * snap;
          jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * step * gradViolaJc;
          gdT(i) += omg * (costj / K + step * gradViolaJt);
          costs(2) += omg * step * costj;
        }

        // printf("L\n");

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }

      t += jerkOpt_.get_T1()(i);
      execution_prefix_elapsed = t;
    }

   // quratic variance
    Eigen::MatrixXd gdp;
    double var;
    // lengthVarianceWithGradCost2p(cps_.points, K, gdp, var);
    distanceSqrVarianceWithGradCost2p(cps_.points, gdp, var);
    // cout << "var=" << var << endl;

    i_dp = 0;
    for (int i = 0; i < N; ++i)
    {
      step = jerkOpt_.get_T1()(i) / K;
      s1 = 0.0;

      for (int j = 0; j <= K; ++j)
      {
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        beta0 << 1.0, s1, s2, s3, s4, s5;
        beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
        alpha = 1.0 / K * j;
        vel = jerkOpt_.get_b().block<6, 3>(i * 6, 0).transpose() * beta1;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;

        gradViolaPc = beta0 * gdp.col(i_dp).transpose();
        gradViolaPt = alpha * gdp.col(i_dp).transpose() * vel;
        jerkOpt_.get_gdC().block<6, 3>(i * 6, 0) += omg * gradViolaPc;
        gdT(i) += omg * (gradViolaPt);

        s1 += step;
        if (j != K || (j == K && i == N - 1))
        {
          ++i_dp;
        }
      }
    }

    costs(4) += var;
    current_cost_snapshot_.moving_cost = moving_cost_accum;
    current_moving_grad_sq_ = moving_grad_sq_accum;

        // cout << "drone_id=" << drone_id_ << " ====================================================end" << endl;
  }

  bool PolyTrajOptimizer::obstacleGradCostP(const int i_dp,
                                            const Eigen::Vector3d &p,
                                            Eigen::Vector3d &gradp,
                                            double &costp)
  {
    if (i_dp == 0 || i_dp > ConstraintPoints::two_thirds_id(cps_.points, touch_goal_)) // only apply to first 2/3
      return false;

    bool ret = false;

    gradp.setZero();
    costp = 0;

    // Obatacle cost
    for (size_t j = 0; j < cps_.direction[i_dp].size(); ++j)
    {
      Eigen::Vector3d ray = (p - cps_.base_point[i_dp][j]);
      double dist = ray.dot(cps_.direction[i_dp][j]);
      double dist_err = obs_clearance_ - dist;
      double dist_err_soft = obs_clearance_soft_ - dist;
      Eigen::Vector3d dist_grad = cps_.direction[i_dp][j];

      if (dist_err > 0)
      {
        ret = true;
        costp += wei_obs_ * pow(dist_err, 3);
        gradp += -wei_obs_ * 3.0 * dist_err * dist_err * dist_grad;
      }

      if (dist_err_soft > 0)
      {
        ret = true;
        double r = 0.05;
        double rsqr = r * r;
        double term = sqrt(1.0 + dist_err_soft * dist_err_soft / rsqr);
        costp += wei_obs_soft_ * rsqr * (term - 1.0);
        gradp += -wei_obs_soft_ * dist_err_soft / term * dist_grad;
      }
    }

    return ret;
  }

  bool PolyTrajOptimizer::swarmGradCostP(const int i_dp,
                                         const double t,
                                         const Eigen::Vector3d &p,
                                         const Eigen::Vector3d &v,
                                         Eigen::Vector3d &gradp,
                                         double &gradt,
                                         double &grad_prev_t,
                                         double &costp)
  {
    if (i_dp <= 0 || i_dp > ConstraintPoints::two_thirds_id(cps_.points, touch_goal_)) // only apply to first 2/3
      return false;

    bool ret = false;

    gradp.setZero();
    gradt = 0;
    grad_prev_t = 0;
    costp = 0;

    const double CLEARANCE2 = (swarm_clearance_ * 1.5) * (swarm_clearance_ * 1.5);
    constexpr double a = 2.0, b = 1.0, inv_a2 = 1 / a / a, inv_b2 = 1 / b / b;

    double pt_time = t_now_ + t;

    for (size_t id = 0; id < swarm_trajs_->size(); id++)
    {
      if ((swarm_trajs_->at(id).executionAt(pt_time).drone_id < 0) || swarm_trajs_->at(id).executionAt(pt_time).drone_id == drone_id_)
      {
        continue;
      }

      double traj_i_satrt_time = swarm_trajs_->at(id).executionAt(pt_time).start_time;

      Eigen::Vector3d swarm_p, swarm_v;
      if (pt_time < traj_i_satrt_time + swarm_trajs_->at(id).executionAt(pt_time).duration)
      {
        swarm_p = swarm_trajs_->at(id).executionAt(pt_time).traj.getPos(std::max(0.0,pt_time - traj_i_satrt_time));
        swarm_v = swarm_trajs_->at(id).executionAt(pt_time).traj.getVel(std::max(0.0,pt_time - traj_i_satrt_time));
      }
      else
      {
        swarm_v.setZero();
        swarm_p = trajectory_lifecycle::sample(swarm_trajs_->at(id).executionAt(pt_time).traj, pt_time-traj_i_satrt_time).p;
      }
      Eigen::Vector3d dist_vec = p - swarm_p;
      double ellip_dist2 = dist_vec(2) * dist_vec(2) * inv_a2 + (dist_vec(0) * dist_vec(0) + dist_vec(1) * dist_vec(1)) * inv_b2;
      double dist2_err = CLEARANCE2 - ellip_dist2;
      double dist2_err2 = dist2_err * dist2_err;
      double dist2_err3 = dist2_err2 * dist2_err;

      if (dist2_err3 > 0)
      {
        ret = true;

        costp += wei_swarm_ * dist2_err3;

        Eigen::Vector3d dJ_dP = wei_swarm_ * 3 * dist2_err2 * (-2) * Eigen::Vector3d(inv_b2 * dist_vec(0), inv_b2 * dist_vec(1), inv_a2 * dist_vec(2));
        gradp += dJ_dP;
        gradt += dJ_dP.dot(v - swarm_v);
        grad_prev_t += dJ_dP.dot(-swarm_v);
      }

      if (min_ellip_dist2_ > ellip_dist2)
      {
        min_ellip_dist2_ = ellip_dist2;
      }
    }

    return ret;
  }

  bool PolyTrajOptimizer::movingObjGradCostP(const int i_dp,
                                             const double t,
                                             const Eigen::Vector3d &p,
                                             const Eigen::Vector3d &v,
                                             Eigen::Vector3d &gradp,
                                             double &gradt,
                                             double &grad_prev_t,
                                             double &costp)
  {
    if (!use_time_aware_moving_obj_cost_ || !moving_objs_ || moving_objs_->getObjNums() <= 0)
      return false;
    if (i_dp <= 0 || i_dp > ConstraintPoints::two_thirds_id(cps_.points, touch_goal_))
      return false;
    if (moving_obj_prediction_horizon_ > 1.0e-3 && t > moving_obj_prediction_horizon_)
      return false;

    bool ret = false;
    gradp.setZero();
    gradt = 0.0;
    grad_prev_t = 0.0;
    costp = 0.0;

    double time_weight = 1.0;
    double dtime_weight = 0.0;
    if (moving_obj_time_decay_tau_ > 1.0e-3)
    {
      time_weight = std::exp(-t / moving_obj_time_decay_tau_);
      dtime_weight = -time_weight / moving_obj_time_decay_tau_;
    }

    const double query_time = t_now_ + t;
    for (int id = 0; id < moving_objs_->getObjNums(); ++id)
    {
      if (!moving_objs_->hasPrediction(id))
        continue;

      const Eigen::Vector3d obj_p = moving_objs_->evaluateConstVel(id, query_time);
      if (!obj_p.allFinite())
        continue;
      const Eigen::Vector3d obj_v = moving_objs_->evaluateConstVelVelocity(id, query_time);
      // Match v1 calcMovingObjCost: Euclidean clearance with a quadratic hinge.
      const Eigen::Vector3d delta = p - obj_p;
      const double dist = std::max(1.0e-3, delta.norm());
      const double dist_err = moving_obj_clearance_ - dist;
      if (dist_err < 0.0)
        continue;

      ret = true;
      const double dist_err2 = dist_err * dist_err;
      Eigen::Vector3d raw_gradp = -2.0 * time_weight * dist_err * delta / dist;
      const double grad_norm = raw_gradp.norm();
      if (moving_obj_max_grad_ > 1.0e-3 && grad_norm > moving_obj_max_grad_)
      {
        raw_gradp = raw_gradp / grad_norm * moving_obj_max_grad_;
      }

      const double raw_cost = time_weight * dist_err2;
      const double raw_time_weight_grad = dtime_weight * dist_err2;

      costp += moving_obj_lambda_ * raw_cost;
      gradp += moving_obj_lambda_ * raw_gradp;
      gradt += moving_obj_lambda_ * (raw_gradp.dot(v - obj_v) + raw_time_weight_grad);
      grad_prev_t += moving_obj_lambda_ * (raw_gradp.dot(-obj_v) + raw_time_weight_grad);
    }

    return ret;
  }

  bool PolyTrajOptimizer::candidateSideGradCostP(
      const double progress, const Eigen::Vector3d &p,
      Eigen::Vector3d &gradp, double &costp)
  {
    if (!candidate_side_bias_enabled_)
      return false;

    const double u = std::min(1.0, std::max(0.0, progress));
    const double desired = candidate_side_sign_ * candidate_side_offset_ *
                           std::sin(M_PI * u);
    const double error =
        (p - candidate_side_origin_).dot(candidate_side_direction_) - desired;
    costp = candidate_side_weight_ * error * error;
    gradp = 2.0 * candidate_side_weight_ * error * candidate_side_direction_;
    return true;
  }

  double PolyTrajOptimizer::candidateSideRegionViolation(
      const double progress, const Eigen::Vector3d &p) const
  {
    if (!candidate_side_bias_enabled_ || !candidate_side_region_enabled_)
      return 0.0;

    const double u = std::min(1.0, std::max(0.0, progress));
    const double half_window = std::min(
        1.0, std::max(1.0e-3, candidate_side_guidance_window_));
    if (std::abs(u - candidate_side_conflict_progress_) >= half_window)
      return 0.0;

    const double desired = candidate_side_offset_ * std::sin(M_PI * u);
    const double half_width = std::max(0.10, 0.25 * desired);
    const double lower = std::max(0.0, desired - half_width);
    const double upper = desired + half_width;
    const double signed_lateral = candidate_side_sign_ *
        (p - candidate_side_origin_).dot(candidate_side_direction_);
    if (signed_lateral < lower)
      return signed_lateral - lower;
    if (signed_lateral > upper)
      return signed_lateral - upper;
    return 0.0;
  }

  bool PolyTrajOptimizer::candidateSideRegionGradCostP(
      const double progress, const Eigen::Vector3d &p,
      Eigen::Vector3d &gradp, double &costp) const
  {
    const double violation = candidateSideRegionViolation(progress, p);
    if (std::abs(violation) <= 1.0e-12)
      return false;

    const double abs_violation = std::abs(violation);
    double dcost_dd = 0.0;
    if (candidate_region_type_ == "barrier")
    {
      constexpr double kBarrierSlope = 8.0;
      const double z = kBarrierSlope * abs_violation;
      const double softplus = z > 40.0 ? z : std::log1p(std::exp(z));
      costp = candidate_region_weight_ *
          (softplus - std::log(2.0)) / kBarrierSlope;
      const double sigmoid = z > 40.0 ? 1.0 : 1.0 / (1.0 + std::exp(-z));
      dcost_dd = candidate_region_weight_ * sigmoid;
    }
    else if (candidate_region_type_ == "hinge4")
    {
      const double squared = violation * violation;
      costp = candidate_region_weight_ * squared * squared;
      dcost_dd = 4.0 * candidate_region_weight_ * violation * squared;
    }
    else
    {
      costp = candidate_region_weight_ * violation * violation;
      dcost_dd = 2.0 * candidate_region_weight_ * violation;
    }

    const double sign = violation >= 0.0 ? 1.0 : -1.0;
    gradp = (candidate_region_type_ == "barrier" ? dcost_dd * sign : dcost_dd) *
        candidate_side_sign_ * candidate_side_direction_;
    return true;
  }

  bool PolyTrajOptimizer::candidatePreservationGradCostP(
      const double progress, const Eigen::Vector3d &p,
      Eigen::Vector3d &gradp, double &costp) const
  {
    if (!candidate_side_bias_enabled_ ||
        !candidate_preserve_reference_valid_ ||
        candidate_preserve_reference_.getPieceNum() <= 0)
      return false;
    const double u = std::min(1.0, std::max(0.0, progress));
    const double half_window = std::min(
        1.0, std::max(1.0e-3, candidate_side_guidance_window_));
    const double distance = std::abs(u - candidate_side_conflict_progress_);
    if (distance >= half_window)
      return false;
    const double taper = 0.5 * (1.0 + std::cos(M_PI * distance / half_window));
    const Eigen::Vector3d reference = candidate_preserve_reference_.getPos(
        u * candidate_preserve_reference_.getTotalDuration());
    const Eigen::Vector3d error = p - reference;
    costp = taper * candidate_preserve_weight_ * error.squaredNorm();
    gradp = 2.0 * taper * candidate_preserve_weight_ * error;
    return true;
  }

  bool PolyTrajOptimizer::trackingGradCostP(const int i_dp,
                                            const double t,
                                            const Eigen::Vector3d &p,
                                            const Eigen::Vector3d &v,
                                            Eigen::Vector3d &gradp,
                                            double &gradt,
                                            double &grad_prev_t,
                                            double &costp,
                                            const double elapsed_t)
  {
    if ( !satrt_tracking_ )
      return false;
    if (i_dp <= 0)
      return false;
    const bool in_tracking_prefix = i_dp <= ConstraintPoints::two_thirds_id(cps_.points, touch_goal_);
    const bool use_fov_height_cost = enable_fov_costs_ && fov_lock_tracking_height_ && fov_height_weight_ > 0.0;
    if (!in_tracking_prefix && !use_fov_height_cost)
      return false;
    if (object_p_(0) < -9999 || object_p_(1) < -9999 || object_p_(2) < -9999) // Have no initial object data
      return false;
    if (relative_tracking_p_(0) < -9999 || relative_tracking_p_(1) < -9999 || relative_tracking_p_(2) < -9999) // Have no initial posi data
      return false;

    bool ret = false;

    gradp.setZero();
    gradt = 0;
    grad_prev_t = 0;
    costp = 0;

    Eigen::Vector3d object_p_t, object_v_t;
    object_v_t = object_v_;
    const Eigen::Matrix3d object_rotation =
        enable_fov_costs_ ? object_q_.normalized().matrix() : object_q_.matrix();
    object_p_t = object_p_ + object_v_t * t + object_rotation * relative_tracking_p_;

    // if ((cps_.points.col(0) - object_p_).dot(object_v_t.normalized()) > -2)
    // {
    if (in_tracking_prefix)
    {
      ret = true;
      const Eigen::Vector3d tracking_error = p - object_p_t;
      if (encirclement_tracking_active_)
      {
        // Visibility-selected goal supplies the MINCO seed and endpoint.
        // Along the trajectory only radius/height bands restore; no bearing prior.
        const Eigen::Vector3d q = p - object_p_ - object_v_t * t;
        const Eigen::Vector3d seed = object_rotation * relative_tracking_p_;
        const double radius = q.head<2>().norm();
        const double desired_radius = seed.head<2>().norm();
        const auto band_error = [](double e, double width) {
          return std::copysign(std::max(0.0, std::abs(e)-width), e);
        };
        const double er = band_error(radius-desired_radius, observation_radius_band_);
        const double ez = band_error(q.z()-seed.z(), observation_height_band_);
        // 本轮修复（cost/gradient 权重重复）：径向/高度项与方位项是两个不同的
        // 目标，必须各自"裸"累加，再在最外层各乘一次自己的权重。之前把方位项
        // 预先写进了 wei_tracking_，随后 gradp/costp 又整体乘了一次
        // wei_tracking_，于是方位梯度相对它自己的代价被多乘了一个
        // wei_tracking_（J_phi = W*w*e^2 却给出 W^2*w*2e*de/dp），LBFGS 的
        // 线搜索因此长期不可信。禁止再用"把 bearing weight 从 0.30 调到 0.08"
        // 掩盖这个公式错误。
        Eigen::Vector3d radial_height_grad(0.0,0.0,2.0*ez);
        if (radius > 1e-8) radial_height_grad.head<2>() = 2.0*er*q.head<2>()/radius;
        double bearing_cost = 0.0;
        double bearing_explicit_time_gradient = 0.0;
        Eigen::Vector3d bearing_grad_p = Eigen::Vector3d::Zero();
        // 本轮修复 1：补回弱方位恢复。
        // 之前这里只有径向项，切向梯度恒为 0，且 |Δr|≤0.35 时径向梯度也为 0，
        // 于是 UAV 可以保持正确半径却沿目标周围漂移一百多度。这里加入一个
        // 软方位项：参考方位 phi_star 来自稳定的编队种子（object_rotation *
        // relative_tracking_p_），绝不是当前 UAV 的方位，因此不会产生
        // reference drift。风险松弛直接复用 ConflictDescriptor 已有时间窗，
        // 不再维护另一套 Elastic 风险场：无威胁为 20°，真实冲突中心最多
        // 再放宽 25°。SIDE 临时绕行仍由 hard-safe set 与统一 comparator 决定。
        // 这是 soft tracking 目标，任何 hard safety 仍然优先。
        if (encirclement_bearing_recovery_weight_ > 0.0 &&
            radius > 1.0e-8 && seed.head<2>().norm() > 1.0e-8)
        {
          const double phi = std::atan2(q.y(), q.x());
          const double phi_star = std::atan2(seed.y(), seed.x());
          double e_phi = phi - phi_star;
          while (e_phi > M_PI) e_phi -= 2.0 * M_PI;
          while (e_phi < -M_PI) e_phi += 2.0 * M_PI;
          double threat_risk = 0.0;
          if (candidate_risk_window_valid_ &&
              candidate_risk_half_window_ > 1.0e-6)
          {
            threat_risk = std::max(
                0.0, 1.0 - std::abs(t - candidate_risk_conflict_time_) /
                               candidate_risk_half_window_);
          }
          constexpr double kThreatAngularSlack = 25.0 * M_PI / 180.0;
          const double angular_band =
              encirclement_base_angular_slack_ +
              threat_risk * kThreatAngularSlack;
          const double e_phi_eff =
              std::max(0.0, std::abs(e_phi) - angular_band);
          if (e_phi_eff > 0.0)
          {
            const double sign_phi = e_phi > 0.0 ? 1.0 : -1.0;
            // dphi/dq = (-q_y/r^2, q_x/r^2, 0)：真实切向梯度。
            const Eigen::Vector2d dphi_dq(-q.y() / (radius * radius),
                                          q.x() / (radius * radius));
            // 本轮修复（prefix 真实时间语义）：真正会被执行的是轨迹前缀，所以
            // 前缀内的方位恢复必须更明确。判定条件用该积分点的真实绝对时间
            // (execution_prefix_elapsed + step*j) 与真实前缀时长比较，不再用
            // "第几个采样点"。这样 T 被优化时"哪些点属于前缀"会自动跟着变，
            // 也不再依赖构造 setter 时估出来的 sample_dt。
            const bool in_execution_prefix =
                prefix_execution_span_ > 0.0 &&
                (elapsed_t >= 0.0 ? elapsed_t : t) <=
                    prefix_execution_span_ + 1.0e-9;
            // 裸权重：只含配置的方位权重与前缀加强，绝不含 wei_tracking_
            // （wei_tracking_ 只在最外层统一乘一次）。
            const double w_phi = encirclement_bearing_recovery_weight_ *
                                 (in_execution_prefix ? prefix_bearing_recovery_boost_ : 1.0);
            bearing_cost = w_phi * e_phi_eff * e_phi_eff;
            bearing_grad_p.head<2>() =
                w_phi * (2.0 * sign_phi * e_phi_eff) * dphi_dq;
            if (threat_risk > 0.0 &&
                std::abs(t - candidate_risk_conflict_time_) > 1.0e-9)
            {
              const double risk_prime =
                  (t < candidate_risk_conflict_time_ ? 1.0 : -1.0) /
                  candidate_risk_half_window_;
              bearing_explicit_time_gradient =
                  -2.0 * w_phi * e_phi_eff *
                  kThreatAngularSlack * risk_prime;
            }
          }
        }
        // cost 与 gradient 必须是同一个数学目标的真实导数：
        //   J = W*(er^2+ez^2) + w_phi*e_phi^2
        //   dJ/dp = W*radial_height_grad + bearing_grad_p
        costp += wei_tracking_*(er*er+ez*ez) + bearing_cost;
        gradp += wei_tracking_*radial_height_grad + bearing_grad_p;
        gradt += (wei_tracking_*radial_height_grad + bearing_grad_p).dot(v-object_v_t);
        gradt += bearing_explicit_time_gradient;
        grad_prev_t +=
            (wei_tracking_*radial_height_grad + bearing_grad_p).dot(-object_v_t);
        grad_prev_t += bearing_explicit_time_gradient;

        // Geometry must reach the rolling prefix, not only a distant endpoint:
        // an initially same-side team cannot pass normal team eligibility yet.
        // This is the SAME empty-sector cost as the joint optimizer, using
        // frozen peer polynomials at the sample's world time. No bearing prior.
        if (ablation_config_.local_gap &&
            !(visibility_context_generation_>0 && visibility_context_hypothesis_id_==0) &&
            encirclement_local_gap_weight_>0.0 && swarm_trajs_ && drone_id_>=0 && drone_id_<3)
        {
          traj_utils::incrementAblationCounter(
              traj_utils::AblationCounter::LOCAL_GAP_COST_EVAL,
              "poly_traj_optimizer", ablation_config_);
          std::array<Eigen::Vector3d,3> relative, velocities, gap_gradient;
          std::array<bool,3> ready{{false,false,false}};
          relative[drone_id_]=q;velocities[drone_id_]=v;ready[drone_id_]=true;
          for(const auto &scheduled:*swarm_trajs_)
          {
            const auto &peer=scheduled.executionAt(t_now_+t);
            const int id=peer.drone_id;
            const double elapsed=t_now_+t-peer.start_time;
            if(id<0 || id>=3 || id==drone_id_ || peer.traj.getPieceNum()<=0 ||
               peer.duration<=0.0 || elapsed<0.0) continue;
            const auto state=trajectory_lifecycle::sample(peer.traj,elapsed);
            relative[id]=state.p-object_p_-object_v_*t;
            velocities[id]=state.v;ready[id]=true;
          }
          if(std::all_of(ready.begin(),ready.end(),[](bool value){return value;}) &&
             multi_uav_formation::circularGapGeometry(relative).finite)
          {
            const double gap_cost=multi_uav_formation::encirclementGeometryCost(
                relative,encirclement_min_gap_,encirclement_max_gap_,&gap_gradient);
            double explicit_time=0.0;
            for(int id=0;id<3;++id)
            {
              explicit_time+=gap_gradient[id].dot(-object_v_);
              if(id!=drone_id_) explicit_time+=gap_gradient[id].dot(velocities[id]);
            }
            ROS_INFO_THROTTLE(1.0,"[encirclement-local-recovery] drone=%d world_time=%.9f gap_cost=%.6f gradient_norm=%.6f",
                drone_id_,t_now_+t,gap_cost,gap_gradient[drone_id_].norm());
            costp+=encirclement_local_gap_weight_*gap_cost;
            gradp+=encirclement_local_gap_weight_*gap_gradient[drone_id_];
            gradt+=encirclement_local_gap_weight_*(explicit_time+gap_gradient[drone_id_].dot(v));
            grad_prev_t+=encirclement_local_gap_weight_*explicit_time;
          }
        }
      }
      else
      {
        const Eigen::Vector3d dJ_dp = 2.0 * tracking_error;
        costp += wei_tracking_ * tracking_error.squaredNorm();
        gradp += wei_tracking_ * dJ_dp;
        gradt += wei_tracking_ * dJ_dp.dot(v - object_v_t);
        grad_prev_t += wei_tracking_ * dJ_dp.dot(-object_v_t);
      }
    }

      if (enable_fov_costs_)
      {
        if (in_tracking_prefix && fov_distance_weight_ > 0.0)
        {
          Eigen::Vector3d object_center_t = object_p_ + object_v_ * t;
          if (fov_lock_tracking_height_)
            object_center_t(2) = fov_target_height_;

          const Eigen::Vector3d delta_to_target = p - object_center_t;
          const Eigen::Vector2d delta_xy = delta_to_target.head<2>();
          const double target_distance = fov_lock_tracking_height_
                                             ? std::max(delta_xy.norm(), 1.0e-3)
                                             : std::max(delta_to_target.norm(), 1.0e-3);
          double distance_error = 0.0;
          double distance_sign = 0.0;
          if (target_distance < fov_min_target_distance_)
          {
            distance_error = fov_min_target_distance_ - target_distance;
            distance_sign = -1.0;
          }
          else if (target_distance > fov_max_target_distance_)
          {
            distance_error = target_distance - fov_max_target_distance_;
            distance_sign = 1.0;
          }

          if (distance_error > 0.0)
          {
            Eigen::Vector3d grad_distance = Eigen::Vector3d::Zero();
            if (fov_lock_tracking_height_)
            {
              grad_distance.head<2>() = fov_distance_weight_ * 2.0 * distance_error * distance_sign *
                                        delta_xy / target_distance;
            }
            else
            {
              grad_distance = fov_distance_weight_ * 2.0 * distance_error * distance_sign *
                              delta_to_target / target_distance;
            }
            const Eigen::Vector3d object_v_for_distance =
                fov_lock_tracking_height_ ? Eigen::Vector3d(object_v_t(0), object_v_t(1), 0.0) : object_v_t;

            costp += fov_distance_weight_ * distance_error * distance_error;
            gradp += grad_distance;
            gradt += grad_distance.dot(v - object_v_for_distance);
            grad_prev_t += grad_distance.dot(-object_v_for_distance);
          }
        }

        if (use_fov_height_cost)
        {
          const double height_error = p(2) - (fov_target_height_ + relative_tracking_p_(2));
          Eigen::Vector3d grad_height = Eigen::Vector3d::Zero();
          grad_height(2) = 2.0 * fov_height_weight_ * height_error;
          costp += fov_height_weight_ * height_error * height_error;
          gradp += grad_height;
          gradt += grad_height.dot(v);
          ret = true;
        }
      }
    // }

    // printf("id=%d, t=%.2f, p=%.1f,%.1f, object_p_=%.1f,%.1f, object_p_t=%.1f,%.1f, costp=%.2f, gradp=%.2f,%.2f, gradt=%.2f\n", drone_id_, t, p(0), p(1), object_p_(0), object_p_(1), object_p_t(0), object_p_t(1), costp, gradp(0), gradp(1), gradt);

    return ret;
  }

  double PolyTrajOptimizer::fovObstacleSafetyScale(const Eigen::Vector3d &p) const
  {
    if (!grid_map_ || !grid_map_->isInMap(p))
      return 0.0;
    if (grid_map_->getInflateOccupancy(p) != 0)
      return 0.0;

    const double resolution = std::max(0.05, grid_map_->getResolution());
    const double inner_radius = std::max(1.60, obs_clearance_soft_ + 8.0 * resolution);
    const double outer_radius = std::max(inner_radius + 0.60, inner_radius + 6.0 * resolution);
    double nearest_occupied = outer_radius;
    bool found_occupied = false;

    const double radius_step = std::max(0.15, 2.0 * resolution);
    for (double radius = radius_step; radius <= outer_radius + 1.0e-6; radius += radius_step)
    {
      const int angle_count =
          std::max(16, static_cast<int>(std::ceil(2.0 * M_PI * radius / std::max(0.15, 2.0 * resolution))));
      for (int i = 0; i < angle_count; ++i)
      {
        const double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(angle_count);
        Eigen::Vector3d sample = p;
        sample(0) += radius * std::cos(theta);
        sample(1) += radius * std::sin(theta);
        if (!grid_map_->isInMap(sample) || grid_map_->getInflateOccupancy(sample) != 0)
        {
          nearest_occupied = radius;
          found_occupied = true;
          break;
        }
      }
      if (found_occupied)
        break;
    }

    if (!found_occupied || nearest_occupied >= outer_radius)
      return 1.0;
    if (nearest_occupied <= inner_radius)
      return 0.0;

    const double ratio = (nearest_occupied - inner_radius) / (outer_radius - inner_radius);
    return ratio * ratio * (3.0 - 2.0 * ratio);
  }

  bool PolyTrajOptimizer::angularGradCostP(const int i_dp,
                                           const double t,
                                           const Eigen::Vector3d &p,
                                           const Eigen::Vector3d &v,
                                           Eigen::Vector3d &gradp,
                                           double &gradt,
                                            double &grad_prev_t,
                                            double &costp)
  {
    const double base_angular_weight = fov_angular_weight_ * fov_soft_scale_;
    if (!enable_fov_costs_ || !satrt_tracking_ || base_angular_weight <= 0.0)
      return false;
    const double safety_scale = fovObstacleSafetyScale(p);
    if (safety_scale <= 0.0)
      return false;
    const double angular_weight = base_angular_weight * safety_scale;
    if (i_dp <= 0 || i_dp > ConstraintPoints::two_thirds_id(cps_.points, touch_goal_))
      return false;
    if (object_p_(0) < -9999 || object_p_(1) < -9999 || object_p_(2) < -9999)
      return false;
    if (swarm_trajs_ == NULL)
      return false;

    bool ret = false;
    gradp.setZero();
    gradt = 0.0;
    grad_prev_t = 0.0;
    costp = 0.0;

    const Eigen::Vector3d object_center = object_p_ + object_v_ * t;
    Eigen::Vector2d a = p.head<2>() - object_center.head<2>();
    const double a_norm = a.norm();
    if (a_norm < 1.0e-3)
      return false;

    const double pt_time = t_now_ + t;

    for (size_t id = 0; id < swarm_trajs_->size(); ++id)
    {
      if (swarm_trajs_->at(id).executionAt(pt_time).drone_id < 0 || swarm_trajs_->at(id).executionAt(pt_time).drone_id == drone_id_)
        continue;

      const double cos_desired_angle = std::cos(fov_angular_min_angle_);

      const double traj_start_time = swarm_trajs_->at(id).executionAt(pt_time).start_time;
      Eigen::Vector3d other_p, other_v;
      if (pt_time < traj_start_time + swarm_trajs_->at(id).executionAt(pt_time).duration)
      {
        other_p = swarm_trajs_->at(id).executionAt(pt_time).traj.getPos(std::max(0.0,pt_time - traj_start_time));
        other_v = swarm_trajs_->at(id).executionAt(pt_time).traj.getVel(std::max(0.0,pt_time - traj_start_time));
      }
      else
      {
        other_v.setZero();
        other_p = trajectory_lifecycle::sample(swarm_trajs_->at(id).executionAt(pt_time).traj, pt_time-traj_start_time).p;
      }

      Eigen::Vector2d b = other_p.head<2>() - object_center.head<2>();
      const double b_norm = b.norm();
      if (b_norm < 1.0e-3)
        continue;

      const double dot_ab = a.dot(b);
      const double cos_phi = dot_ab / (a_norm * b_norm);
      const double violation = cos_phi - cos_desired_angle;
      if (violation <= 0.0)
        continue;

      ret = true;
      costp += angular_weight * violation * violation;

      const Eigen::Vector2d dcos_da = b / (a_norm * b_norm) - cos_phi * a / (a_norm * a_norm);
      const Eigen::Vector2d dcos_db = a / (a_norm * b_norm) - cos_phi * b / (b_norm * b_norm);
      Eigen::Vector3d grad_current = Eigen::Vector3d::Zero();
      grad_current.head<2>() = 2.0 * angular_weight * violation * dcos_da;
      gradp += grad_current;

      Eigen::Vector2d grad_object_xy = -2.0 * angular_weight * violation * (dcos_da + dcos_db);
      Eigen::Vector2d grad_other_xy = 2.0 * angular_weight * violation * dcos_db;

      gradt += grad_current.head<2>().dot(v.head<2>()) + grad_other_xy.dot(other_v.head<2>()) + grad_object_xy.dot(object_v_.head<2>());
      grad_prev_t += grad_other_xy.dot(other_v.head<2>()) + grad_object_xy.dot(object_v_.head<2>());
    }

    return ret;
  }

  bool PolyTrajOptimizer::yawGradCostPV(const int i_dp,
                                        const double t,
                                        const Eigen::Vector3d &p,
                                        const Eigen::Vector3d &v,
                                        const Eigen::Vector3d &a,
                                        Eigen::Vector3d &gradp,
                                        Eigen::Vector3d &gradv,
                                        double &gradt,
                                            double &grad_prev_t,
                                            double &costp)
  {
    const double base_yaw_weight = fov_yaw_weight_ * fov_soft_scale_;
    if (!enable_fov_costs_ || !satrt_tracking_ || base_yaw_weight <= 0.0)
      return false;
    const double safety_scale = fovObstacleSafetyScale(p);
    if (safety_scale <= 0.0)
      return false;
    const double yaw_weight = base_yaw_weight * safety_scale;
    if (i_dp <= 0 || i_dp > ConstraintPoints::two_thirds_id(cps_.points, touch_goal_))
      return false;
    if (object_p_(0) < -9999 || object_p_(1) < -9999 || object_p_(2) < -9999)
      return false;

    Eigen::Vector2d vel_xy(v(0), v(1));
    const double speed = vel_xy.norm();
    if (speed < fov_yaw_speed_min_)
      return false;

    const Eigen::Vector3d object_center = object_p_ + object_v_ * t;
    Eigen::Vector2d los_xy = object_center.head<2>() - p.head<2>();
    const double los_norm = los_xy.norm();
    if (los_norm < 1.0e-3)
      return false;

    const double cos_angle = std::max(-1.0, std::min(1.0, vel_xy.dot(los_xy) / (speed * los_norm)));
    const double violation = std::cos(fov_half_angle_) - cos_angle;
    if (violation <= 0.0)
      return false;

    gradp.setZero();
    gradv.setZero();
    gradt = 0.0;
    grad_prev_t = 0.0;
    costp = yaw_weight * violation * violation;

    const Eigen::Vector2d dcos_dvel = los_xy / (speed * los_norm) - cos_angle * vel_xy / (speed * speed);
    const Eigen::Vector2d dcos_dlos = vel_xy / (speed * los_norm) - cos_angle * los_xy / (los_norm * los_norm);
    const double scale = -2.0 * yaw_weight * violation;

    gradv.head<2>() = scale * dcos_dvel;
    gradp.head<2>() = -scale * dcos_dlos;
    gradt = gradp.dot(v) + gradv.dot(a) + scale * dcos_dlos.dot(object_v_.head<2>());
    grad_prev_t = scale * dcos_dlos.dot(object_v_.head<2>());
    return true;
  }

  bool PolyTrajOptimizer::feasibilityGradCostV(const Eigen::Vector3d &v,
                                               Eigen::Vector3d &gradv,
                                               double &costv)
  {
    double vpen = v.squaredNorm() - max_vel_ * max_vel_;
    if (vpen > 0)
    {
      gradv = wei_feas_ * 6 * vpen * vpen * v;
      costv = wei_feas_ * vpen * vpen * vpen;
      return true;
    }
    return false;
  }

  bool PolyTrajOptimizer::feasibilityGradCostA(const Eigen::Vector3d &a,
                                               Eigen::Vector3d &grada,
                                               double &costa)
  {
    double apen = a.squaredNorm() - max_acc_ * max_acc_;
    if (apen > 0)
    {
      grada = wei_feas_ * 6 * apen * apen * a;
      costa = wei_feas_ * apen * apen * apen;
      return true;
    }
    return false;
  }

  bool PolyTrajOptimizer::feasibilityGradCostJ(const Eigen::Vector3d &j,
                                               Eigen::Vector3d &gradj,
                                               double &costj)
  {
    if (max_jer_ <= 0.0)
      return false;

    double jpen = j.squaredNorm() - max_jer_ * max_jer_;
    if (jpen > 0)
    {
      gradj = wei_feas_ * 6 * jpen * jpen * j;
      costj = wei_feas_ * jpen * jpen * jpen;
      return true;
    }
    return false;
  }

  void PolyTrajOptimizer::distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                                            Eigen::MatrixXd &gdp,
                                                            double &var)
  {
    int N = ps.cols() - 1;
    Eigen::MatrixXd dps = ps.rightCols(N) - ps.leftCols(N);
    Eigen::VectorXd dsqrs = dps.colwise().squaredNorm().transpose();
    // double dsqrsum = dsqrs.sum();
    double dquarsum = dsqrs.squaredNorm();
    // double dsqrmean = dsqrsum / N;
    double dquarmean = dquarsum / N;
    var = wei_sqrvar_ * (dquarmean);
    gdp.resize(3, N + 1);
    gdp.setZero();
    for (int i = 0; i <= N; i++)
    {
      if (i != 0)
      {
        gdp.col(i) += wei_sqrvar_ * (4.0 * (dsqrs(i - 1)) / N * dps.col(i - 1));
      }
      if (i != N)
      {
        gdp.col(i) += wei_sqrvar_ * (-4.0 * (dsqrs(i)) / N * dps.col(i));
      }
    }
    return;
  }

  void PolyTrajOptimizer::lengthVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                                       const int n,
                                                       Eigen::MatrixXd &gdp,
                                                       double &var)
  {
    int N = ps.cols() - 1;
    int M = N / n;
    Eigen::MatrixXd dps = ps.rightCols(N) - ps.leftCols(N);
    Eigen::VectorXd ds = dps.colwise().norm().transpose();
    Eigen::VectorXd ls(M), lsqrs(M);
    for (int i = 0; i < M; i++)
    {
      ls(i) = ds.segment(i * n, n).sum();
      lsqrs(i) = ls(i) * ls(i);
    }
    double lm = ls.mean();
    double lsqrm = lsqrs.mean();
    var = wei_sqrvar_ * (lsqrm - lm * lm) + 250.0 * M * lm;
    Eigen::VectorXd gdls = wei_sqrvar_ * 2.0 / M * (ls.array() - lm) + 250.0;
    Eigen::MatrixXd gdds = dps.colwise().normalized();
    gdp.resize(3, N + 1);
    gdp.setZero();
    for (int i = 0; i < M; i++)
    {
      gdp.block(0, i * n, 3, n) -= gdls(i) * gdds.block(0, i * n, 3, n);
      gdp.block(0, i * n + 1, 3, n) += gdls(i) * gdds.block(0, i * n, 3, n);
    }
    return;
  }

  /* helper functions */
  void PolyTrajOptimizer::setParam(ros::NodeHandle &nh)
  {
    ablation_config_ = traj_utils::loadAblationConfig(
        "poly_traj_optimizer", &formal_ablation_mode_);
    nh.param("optimization/constraint_points_perPiece", cps_num_prePiece_, -1);
    nh.param("optimization/weight_obstacle", wei_obs_, -1.0);
    nh.param("optimization/weight_obstacle_soft", wei_obs_soft_, -1.0);
    nh.param("optimization/weight_swarm", wei_swarm_, -1.0);
    nh.param("optimization/weight_tracking", wei_tracking_, -1.0);
    nh.param("optimization/enable_encirclement_tracking",
             encirclement_tracking_configured_, false);
    if (formal_ablation_mode_ && !encirclement_tracking_configured_)
      throw std::invalid_argument(
          "ENCIRCLEMENT_MUST_REMAIN_ENABLED_FOR_FORMAL_ABLATION");
    double encirclement_max_gap_deg=170.0,encirclement_min_gap_deg=25.0;
    nh.param("/encirclement_geometry/min_angular_separation_deg",encirclement_min_gap_deg,25.0);
    encirclement_min_gap_=encirclement_min_gap_deg*M_PI/180.0;
    nh.param("/encirclement_geometry/max_circular_gap_deg",encirclement_max_gap_deg,170.0);
    nh.param("/encirclement_geometry/local_gap_weight",encirclement_local_gap_weight_,100.0);
    encirclement_max_gap_=encirclement_max_gap_deg*M_PI/180.0;
    multi_uav_formation::validateEncirclementLimits(encirclement_min_gap_,encirclement_max_gap_);
    if(!std::isfinite(encirclement_local_gap_weight_) || encirclement_local_gap_weight_<0.0)
      throw std::invalid_argument("Invalid local encirclement gap weight");
    ROS_INFO("[encirclement-local-contract] max_gap_deg=%.3f local_gap_weight=%.3f bearing_prior=0",
        encirclement_max_gap_deg,encirclement_local_gap_weight_);
    nh.param("optimization/observation_radius_band", observation_radius_band_, 0.35);
    nh.param("optimization/observation_height_band", observation_height_band_, 0.20);
    observation_radius_band_ = std::max(0.0, observation_radius_band_);
    observation_height_band_ = std::max(0.0, observation_height_band_);
    double encirclement_base_angular_slack_deg = 20.0;
    nh.param("optimization/encirclement_base_angular_slack_deg",
             encirclement_base_angular_slack_deg, 20.0);
    nh.param("optimization/directional_visibility_deep_risk_kappa",
             directional_visibility_deep_risk_kappa_, 1.0);
    nh.param("optimization/directional_visibility_deep_risk_beta",
             directional_visibility_deep_risk_beta_, 2.0);
    encirclement_base_angular_slack_deg = std::max(
        0.0, std::min(180.0, encirclement_base_angular_slack_deg));
    encirclement_base_angular_slack_ =
        encirclement_base_angular_slack_deg * M_PI / 180.0;
    directional_static_risk_transition_ =
        std::max(1.0e-3, directional_static_risk_transition_);
    directional_moving_risk_transition_ =
        std::max(1.0e-3, directional_moving_risk_transition_);
    directional_visibility_deep_risk_kappa_ =
        std::max(1.0e-6, directional_visibility_deep_risk_kappa_);
    directional_visibility_deep_risk_beta_ =
        std::max(1.0e-6, directional_visibility_deep_risk_beta_);
    nh.param("optimization/weight_feasibility", wei_feas_, -1.0);
    nh.param("optimization/weight_sqrvariance", wei_sqrvar_, -1.0);
    nh.param("optimization/weight_time", wei_time_, -1.0);
    nh.param("optimization/obstacle_clearance", obs_clearance_, -1.0);
    nh.param("optimization/obstacle_clearance_soft", obs_clearance_soft_, -1.0);
    nh.param("optimization/swarm_clearance", swarm_clearance_, -1.0);
    nh.param("optimization/moving_obj_clearance", moving_obj_clearance_, 1.1);
    // Dynamic hard collision uses the production UAV collision footprint,
    // not the (deliberately smaller) static grid-map inflation.  For the
    // deployed iris model this is 0.3835 m (rotor outer extent); the live
    // moving-obstacle radius still comes from ObjPredictor marker scale.
    nh.param("optimization/moving_obj_hard_body_radius",
             moving_obj_hard_body_radius_, 0.384);
    moving_obj_hard_body_radius_ = std::max(0.0, moving_obj_hard_body_radius_);
    nh.param("optimization/moving_obj_lambda", moving_obj_lambda_, 0.5);
    nh.param("prediction/moving_obj_prediction_horizon", moving_obj_prediction_horizon_, 2.0);
    nh.param("optimization/moving_obj_time_decay_tau", moving_obj_time_decay_tau_, 1.8);
    nh.param("optimization/moving_obj_max_grad", moving_obj_max_grad_, 3.0);
    nh.param("prediction/use_time_aware_moving_obj_cost", use_time_aware_moving_obj_cost_, false);
    if (use_time_aware_moving_obj_cost_)
      ROS_INFO("[moving_obj] optimizer params: preferred_clearance=%.2f, hard_body_radius=%.3f, lambda=%.2f, horizon=%.2f, decay_tau=%.2f, max_grad=%.2f",
               moving_obj_clearance_, moving_obj_hard_body_radius_,
               moving_obj_lambda_, moving_obj_prediction_horizon_,
               moving_obj_time_decay_tau_, moving_obj_max_grad_);
    nh.param("optimization/weight_visibility", weight_visibility_, 20.0);
    // 2026-09-22 恢复:P/T 联合 hard SCP 参数(默认关闭,语义回归)。
    nh.param("optimization/enable_candidate_hard_corridor_scp",
             candidate_hard_corridor_scp_enabled_, false);
    nh.param("optimization/candidate_hard_corridor_scp_radius",
             candidate_hard_corridor_scp_radius_, 0.5);
    nh.param("optimization/candidate_hard_corridor_scp_trust_region",
             candidate_hard_corridor_scp_trust_region_, 0.25);
    nh.param("optimization/candidate_hard_corridor_scp_time_trust_region",
             candidate_hard_corridor_scp_time_trust_region_, 0.15);
    nh.param("optimization/candidate_hard_corridor_scp_max_iterations",
             candidate_hard_corridor_scp_max_iterations_, 8);
    nh.param("optimization/candidate_hard_corridor_scp_samples",
             candidate_hard_corridor_scp_samples_, 25);
    nh.param("optimization/enable_team_spatiotemporal_scp",
             team_spatiotemporal_scp_enabled_, true);
    nh.param("optimization/team_scp_p_trust", team_scp_p_trust_, 0.30);
    nh.param("optimization/team_scp_t_trust_ratio",
             team_scp_t_trust_ratio_, 0.15);
    nh.param("optimization/team_scp_max_iterations",
             team_scp_max_iterations_, 6);
    nh.param("optimization/team_scp_soft_weight",
             team_scp_soft_weight_, 10.0);
    nh.param("optimization/team_scp_t_gain_ratio",
             team_scp_t_gain_ratio_, 0.80);
    nh.param("optimization/team_visibility_range_smoothing",
             team_visibility_range_smoothing_, 0.25);
    nh.param("optimization/team_visibility_los_smoothing",
             team_visibility_los_smoothing_, 0.15);
    // LOS observation-plane slack 权重(Feedback116,唯一新参数)。LOS plane
    // violation 的典型量级为 O(0.1~3)m;trust-region 盒约束把 Δp 限制在
    // O(0.1~1.5)m,原 QP 的单位 Hessian 下走一步的代价量级 O(1)。w2=1e3 使
    // 1 cm slack 与 1 m 走步同量级、1 m slack 被强抑制(≈1e3),既允许"尚未
    // 绕出"的渐进解存在,又让 optimizer 优先把 violation 压到 0。
    nh.param("optimization/los_observation_slack_weight",
             los_observation_slack_weight_, 1.0e3);
    if (!std::isfinite(los_observation_slack_weight_) ||
        los_observation_slack_weight_ < 0.0)
      los_observation_slack_weight_ = 1.0e3;
    team_scp_p_trust_ = std::max(1.0e-3, team_scp_p_trust_);
    team_scp_t_trust_ratio_ = std::max(1.0e-3, team_scp_t_trust_ratio_);
    team_scp_max_iterations_ = std::max(1, team_scp_max_iterations_);
    team_scp_soft_weight_ = std::max(0.0, team_scp_soft_weight_);
    team_scp_t_gain_ratio_ = std::max(
        0.1, std::min(1.0, team_scp_t_gain_ratio_));
    team_visibility_range_smoothing_ =
        std::max(1.0e-3, team_visibility_range_smoothing_);
    team_visibility_los_smoothing_ =
        std::max(1.0e-3, team_visibility_los_smoothing_);
    ROS_INFO("[TEAM_SPATIOTEMPORAL_SCP_ENABLED] value=%d "
             "candidate_hard_policy=%d p_trust=%.3f t_trust_ratio=%.3f "
             "max_iterations=%d soft_weight=%.3f t_gain_ratio=%.3f",
             static_cast<int>(team_spatiotemporal_scp_enabled_),
             static_cast<int>(candidate_hard_corridor_scp_enabled_),
             team_scp_p_trust_, team_scp_t_trust_ratio_,
             team_scp_max_iterations_, team_scp_soft_weight_,
             team_scp_t_gain_ratio_);
    nh.param("optimization/elastic_moving_risk_margin",
             elastic_moving_risk_margin_, 0.80);
    nh.param("optimization/static_los_margin", static_los_margin_, 0.08);
    nh.param("optimization/static_los_scene_file",
             static_los_scene_file_, std::string(""));
    weight_visibility_ = std::max(0.0, weight_visibility_);
    static_los_margin_ = std::max(0.0, static_los_margin_);
    std::string static_los_reason;
    const bool static_los_loaded =
        static_los_geometry_.loadScene(static_los_scene_file_, static_los_reason);
    ROS_INFO("[static-los-contract] loaded=%d scene=%s cylinders=%zu margin=%.3f "
             "geometry=SOLID_VERTICAL_CYLINDERS reason=%s",
             static_cast<int>(static_los_loaded), static_los_scene_file_.c_str(),
             static_los_geometry_.size(), static_los_margin_,
             static_los_reason.c_str());
    ROS_INFO("[directional-visibility-config] sample_dt=%.3f "
             "weight_visibility=%.6f target_model=CONST_VEL_FULL_TRAJECTORY "
             "static_geometry_valid=%d",
             directional_visibility_sample_dt_,
             weight_visibility_,
             static_cast<int>(static_los_geometry_.valid()));
    nh.param("optimization/max_vel", max_vel_, -1.0);
    nh.param("optimization/max_acc", max_acc_, -1.0);
    nh.param("optimization/max_jer", max_jer_, -1.0);
    // Feedback126 unified safety kernel: the same feasibility tolerance the
    // manager checker applies, so generation and validation share one PVAJ
    // definition (limit × (1 + tolerance)).
    nh.param("manager/feasibility_tolerance", feasibility_tolerance_, 0.0);
    ROS_INFO("[planner-dynamics-limits] velocity_limit=%.6f acceleration_limit=%.6f jerk_limit=%.6f feasibility_tolerance=%.4f",
             max_vel_, max_acc_, max_jer_, feasibility_tolerance_);
    nh.param("optimization/candidate_preserve_weight",
             candidate_preserve_weight_, 2.0);
    nh.param("optimization/enable_candidate_region_constraint",
             candidate_region_constraint_enabled_, true);
    nh.param("optimization/candidate_region_weight",
             candidate_region_weight_, 2.0);
    nh.param("optimization/candidate_region_type",
             candidate_region_type_, std::string("quadratic"));
    ROS_INFO("[candidate-optimizer-config] preservation=1 region=%d corridor=REMOVED",
             static_cast<int>(candidate_region_constraint_enabled_));
    bool legacy_use_fov_tracking = false;
    nh.param("optimization/use_fov_tracking", legacy_use_fov_tracking, false);
    nh.param("optimization/use_fov_costs", enable_fov_costs_, legacy_use_fov_tracking);
    nh.param("optimization/fov_lock_tracking_height", fov_lock_tracking_height_, true);
    nh.param("optimization/fov_min_target_distance", fov_min_target_distance_, 0.7);
    nh.param("optimization/fov_max_target_distance", fov_max_target_distance_, 3.0);
    nh.param("optimization/fov_distance_weight", fov_distance_weight_, 50.0);
    nh.param("optimization/fov_height_weight", fov_height_weight_, 500.0);
    nh.param("optimization/fov_target_height", fov_target_height_, 1.2);
    nh.param("optimization/fov_angular_weight", fov_angular_weight_, 20.0);
    double fov_angular_min_angle_deg = 45.0;
    nh.param("optimization/fov_angular_min_angle_deg", fov_angular_min_angle_deg, 45.0);
    fov_angular_min_angle_ = std::max(0.0, std::min(179.0, fov_angular_min_angle_deg)) * M_PI / 180.0;
    nh.param("optimization/fov_yaw_weight", fov_yaw_weight_, 20.0);
    double fov_half_angle_deg = 42.5;
    nh.param("optimization/fov_half_angle_deg", fov_half_angle_deg, 42.5);
    fov_half_angle_ = std::max(1.0, std::min(179.0, fov_half_angle_deg)) * M_PI / 180.0;
    nh.param("optimization/fov_yaw_speed_min", fov_yaw_speed_min_, 0.15);
  }

  double PolyTrajOptimizer::getMovingObjHardClearance(void) const
  {
    if (!moving_objs_ || moving_objs_->getObjNums() <= 0)
      return moving_obj_hard_body_radius_;
    double maximum = 0.0;
    for (int id = 0; id < moving_objs_->getObjNums(); ++id)
    {
      const Eigen::Vector3d scale = moving_objs_->getObjScale(id);
      if (!scale.allFinite() || scale.x() <= 0.0 || scale.y() <= 0.0)
        return moving_obj_hard_body_radius_ + 0.5;
      maximum = std::max(maximum, 0.5 * std::max(scale.x(), scale.y()));
    }
    return moving_obj_hard_body_radius_ + maximum;
  }

  double PolyTrajOptimizer::movingObjHardClearanceForObject(const int object_id) const
  {
    if (!moving_objs_ || object_id < 0 || object_id >= moving_objs_->getObjNums())
      return moving_obj_hard_body_radius_;
    const Eigen::Vector3d scale = moving_objs_->getObjScale(object_id);
    if (!scale.allFinite() || scale.x() <= 0.0 || scale.y() <= 0.0)
      return moving_obj_hard_body_radius_;
    // Per-object physical contact distance: this object's live radius plus
    // the UAV footprint.  Identical definition to the manager's
    // dynamicHardClearanceForObject — generation and validation share it.
    return moving_obj_hard_body_radius_ + 0.5 * std::max(scale.x(), scale.y());
  }

  bool PolyTrajOptimizer::swarmPhysicalViolation(
      const poly_traj::Trajectory &traj, const double start_time,
      const bool touch_goal, UnifiedSwarmFinding *finding) const
  {
    if (finding != nullptr)
    {
      finding->violated = false;
      finding->peer_drone_id = -1;
      finding->conflict_time = 0.0;
      finding->ellip_dist2 = 0.0;
    }
    if (traj.getPieceNum() <= 0 || swarm_trajs_ == nullptr || swarm_trajs_->empty())
      return false;
    const double duration = traj.getTotalDuration();
    if (!std::isfinite(duration) || duration <= 0.0 || !std::isfinite(start_time))
      return false;
    // Same authority horizon semantics as the manager-side validation:
    // rolling prefixes are bounded by the moving-object prediction horizon.
    const double check_end =
        touch_goal ? duration
                   : (moving_obj_prediction_horizon_ > 1.0e-3
                          ? std::min(duration, moving_obj_prediction_horizon_)
                          : duration);
    const double clearance = swarm_clearance_;
    const double clearance2 = clearance * clearance;
    constexpr double inv_vertical_axis2 = 0.25;
    for (double t = 0.0; t < check_end + 1.0e-6; t += 0.03)
    {
      const double tt = std::min(t, check_end);
      const Eigen::Vector3d pos = traj.getPos(tt);
      if (!pos.allFinite())
        continue;
      const double global_time = start_time + tt;
      for (size_t id = 0; id < swarm_trajs_->size(); ++id)
      {
        const LocalTrajData &other = swarm_trajs_->at(id).executionAt(global_time);
        if (other.drone_id < 0 || other.drone_id == drone_id_ ||
            other.duration <= 0.0 || other.traj.getPieceNum() <= 0)
          continue;
        const double other_t = global_time - other.start_time;
        // Peer model identical to validation: polynomial while alive,
        // terminal position hold afterwards (never extrapolate velocity).
        const Eigen::Vector3d other_pos =
            other.traj.getPos(std::max(0.0, std::min(other_t, other.duration)));
        if (!other_pos.allFinite())
          continue;
        const Eigen::Vector3d d = pos - other_pos;
        const double ellip_dist2 =
            d.head<2>().squaredNorm() + d(2) * d(2) * inv_vertical_axis2;
        if (ellip_dist2 < clearance2)
        {
          if (finding != nullptr)
          {
            finding->violated = true;
            finding->peer_drone_id = other.drone_id;
            finding->conflict_time = tt;
            finding->ellip_dist2 = ellip_dist2;
          }
          return true;
        }
      }
    }
    return false;
  }

  bool PolyTrajOptimizer::pvaPhysicalViolation(const poly_traj::Trajectory &traj,
                                               double *max_v, double *max_a,
                                               double *max_j) const
  {
    if (max_v != nullptr) *max_v = 0.0;
    if (max_a != nullptr) *max_a = 0.0;
    if (max_j != nullptr) *max_j = 0.0;
    if (traj.getPieceNum() <= 0)
      return false;
    const double duration = traj.getTotalDuration();
    if (!std::isfinite(duration) || duration <= 0.0)
      return false;
    // Same sampling and same limit × (1 + feasibility_tolerance) definition
    // as the manager-side checkTrajectoryDynamics — one PVAJ predicate.
    const double dt = std::max(0.02, std::min(0.05, duration / 100.0));
    const double tol = 1.0 + std::max(0.0, feasibility_tolerance_);
    const double v_limit = max_vel_ * tol;
    const double a_limit = max_acc_ * tol;
    const double j_limit = max_jer_ * tol;
    double worst_v = 0.0, worst_a = 0.0, worst_j = 0.0;
    for (double t = 0.0; t < duration + 1.0e-6; t += dt)
    {
      const double tt = std::min(t, duration);
      worst_v = std::max(worst_v, traj.getVel(tt).norm());
      worst_a = std::max(worst_a, traj.getAcc(tt).norm());
      worst_j = std::max(worst_j, traj.getJer(tt).norm());
    }
    worst_j = std::max(worst_j, traj.getJer(duration).norm());
    if (max_v != nullptr) *max_v = worst_v;
    if (max_a != nullptr) *max_a = worst_a;
    if (max_j != nullptr) *max_j = worst_j;
    const bool vel_ok = max_vel_ <= 0.0 || worst_v <= v_limit;
    const bool acc_ok = max_acc_ <= 0.0 || worst_a <= a_limit;
    const bool jer_ok = max_jer_ <= 0.0 || worst_j <= j_limit;
    return !(vel_ok && acc_ok && jer_ok);
  }

  bool PolyTrajOptimizer::dynamicPhysicalViolation(
      const poly_traj::Trajectory &traj, const double start_time,
      int *violating_object) const
  {
    if (violating_object != nullptr) *violating_object = -1;
    if (traj.getPieceNum() <= 0 || !moving_objs_ ||
        moving_objs_->getObjNums() <= 0)
      return false;
    const double duration = traj.getTotalDuration();
    if (!std::isfinite(duration) || duration <= 0.0 || !std::isfinite(start_time))
      return false;
    const double check_end =
        moving_obj_prediction_horizon_ > 1.0e-3
            ? std::min(duration, moving_obj_prediction_horizon_)
            : duration;
    for (double t = 0.0; t < check_end + 1.0e-6; t += 0.05)
    {
      const double tt = std::min(t, check_end);
      const Eigen::Vector3d pos = traj.getPos(tt);
      if (!pos.allFinite())
        continue;
      const double query_time = start_time + tt;
      for (int id = 0; id < moving_objs_->getObjNums(); ++id)
      {
        if (!moving_objs_->hasPrediction(id))
          continue;
        const Eigen::Vector3d obstacle =
            moving_objs_->evaluateConstVel(id, query_time);
        if (!obstacle.allFinite())
          continue;
        const double hard_clearance = movingObjHardClearanceForObject(id);
        if ((pos - obstacle).norm() < hard_clearance)
        {
          if (violating_object != nullptr) *violating_object = id;
          return true;
        }
      }
    }
    return false;
  }

  void PolyTrajOptimizer::setEnvironment(const GridMap::Ptr &map)
  {
    grid_map_ = map;
    moving_objs_.reset();

    a_star_.reset(new AStar);
    a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));
  }

  void PolyTrajOptimizer::setEnvironment(const GridMap::Ptr &map, const fast_planner::ObjPredictor::Ptr &moving_objs)
  {
    grid_map_ = map;
    moving_objs_ = moving_objs;

    a_star_.reset(new AStar);
    a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));
  }

  void PolyTrajOptimizer::setControlPoints(const Eigen::MatrixXd &points)
  {
    cps_.points = points;
  }

  void PolyTrajOptimizer::setFovSoftScale(const double scale)
  {
    fov_soft_scale_ = std::max(0.0, std::min(1.0, scale));
  }

  void PolyTrajOptimizer::setEncirclementBearingRecovery(
      const double weight, const double prefix_boost,
      const double prefix_span_seconds, const double sample_time_step_seconds)
  {
    encirclement_bearing_recovery_weight_ =
        std::isfinite(weight) && weight > 0.0 ? weight : 0.0;
    prefix_bearing_recovery_boost_ =
        std::isfinite(prefix_boost) && prefix_boost >= 1.0 ? prefix_boost : 1.0;
    prefix_execution_span_ =
        std::isfinite(prefix_span_seconds) && prefix_span_seconds > 0.0
            ? prefix_span_seconds : 0.0;
    // 前缀归属完全由积分点的真实绝对时间决定（见 trackingGradCostP），
    // 这里不再把时长换算成"采样点个数"。
    (void)sample_time_step_seconds;
  }

 void PolyTrajOptimizer::setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr) { swarm_trajs_ = swarm_trajs_ptr; }

  void PolyTrajOptimizer::setDroneId(const int drone_id) { drone_id_ = drone_id; }

  void PolyTrajOptimizer::setIfTouchGoal(const bool touch_goal) { touch_goal_ = touch_goal; }

  void PolyTrajOptimizer::setObject(const Eigen::Vector3d &object_pt, const Eigen::Vector3d &object_vel, const Eigen::Quaterniond &object_q)
  {
    object_p_ = object_pt, object_v_ = object_vel, object_q_ = object_q;
  }

  void PolyTrajOptimizer::setRelativeTrackingP(const Eigen::Vector3d &rela_track_p) { relative_tracking_p_ = rela_track_p; }

  void PolyTrajOptimizer::setSoftEncirclementReference(const bool enabled)
  {
    // The configured mode owns observation radius/height bands on every
    // tracking/recovery path. Neither the seed bearing nor a 120-degree slot
    // restores P/T. enabled = production tracking mode / cooperative task
    // valid; the launch switch remains the authority.
    encirclement_tracking_active_ = encirclement_tracking_configured_ && enabled;
  }

  void PolyTrajOptimizer::setDirectionalVisibilityGuidance(const bool enabled)
  {
    // Local directional J_vis gate: launch switch AND the caller-declared
    // "ablation local_visibility + valid visibility context" verdict.  It must
    // never be derived from a Team encirclement_generation value (阶段 C:
    // production 曾经因此让 Local J_vis 恒为死路径).
    encirclement_visibility_guidance_active_ =
        encirclement_tracking_configured_ && enabled;
  }

  void PolyTrajOptimizer::setVisibilityYawState(
      const bool valid, const double yaw, const double yaw_rate,
      const double horizontal_fov, const double vertical_fov,
      const double min_range, const double max_range)
  {
    visibility_yaw_valid_ = valid && std::isfinite(yaw) &&
                            std::isfinite(yaw_rate) &&
                            std::isfinite(horizontal_fov) &&
                            std::isfinite(vertical_fov) &&
                            std::isfinite(min_range) &&
                            std::isfinite(max_range) &&
                            horizontal_fov > 1.0e-6 &&
                            vertical_fov > 1.0e-6 && min_range >= 0.0 &&
                            max_range > min_range;
    visibility_yaw_ = visibility_yaw_valid_ ? yaw : 0.0;
    visibility_yaw_rate_ = visibility_yaw_valid_ ? yaw_rate : 0.0;
    if (visibility_yaw_valid_)
    {
      visibility_camera_hfov_ = horizontal_fov;
      visibility_camera_vfov_ = vertical_fov;
      visibility_camera_min_range_ = min_range;
      visibility_camera_max_range_ = max_range;
    }
  }

  void PolyTrajOptimizer::setVisibilityContextIdentity(
      const unsigned long generation, const int hypothesis_id)
  {
    visibility_context_generation_ = generation;
    visibility_context_hypothesis_id_ = hypothesis_id;
  }

  void PolyTrajOptimizer::setStartTracking(const bool satrt_tracking) { satrt_tracking_ = satrt_tracking; }

  void PolyTrajOptimizer::setConstraintPoints(ConstraintPoints cps) { cps_ = cps; }

  /* 单点解析梯度探针：严格按生产调用顺序
   *   reset(head,tail,N) -> generate(inner,T) -> getInitConstraintPoints(K)
   * 重建轨迹与约束点，再取指定约束点上的解析 cost/grad。 */
  bool PolyTrajOptimizer::trackingGradientProbeAtPoint(
      const Eigen::Matrix3d &head, const Eigen::Matrix3d &tail,
      const Eigen::MatrixXd &inner_points, const Eigen::VectorXd &durations,
      const int cps_num_pre_piece, const int point_index,
      double &cost, Eigen::Vector3d &grad_p, double &grad_t,
      double &grad_prev_t, double &point_time)
  {
    const int pieces = static_cast<int>(durations.size());
    if (pieces <= 0 || inner_points.cols() != pieces - 1 ||
        cps_num_pre_piece <= 0)
      return false;
    poly_traj::MinJerkOpt opt;
    opt.reset(head, tail, pieces);
    opt.generate(inner_points, durations);
    const int K = cps_num_pre_piece;
    const int cp_count = pieces * K + 1;
    if (point_index < 0 || point_index >= cp_count)
      return false;
    ConstraintPoints cps;
    cps.resize_cp(cp_count);
    cps.points = opt.getInitConstraintPoints(K);
    cps_ = cps;
    piece_num_ = pieces;

    int index = 0;
    double t = 0.0;
    for (int i = 0; i < pieces; ++i)
    {
      const double step = opt.get_T1()(i) / static_cast<double>(K);
      const Eigen::Matrix<double, 6, 3> &c = opt.get_b().block<6, 3>(i * 6, 0);
      for (int j = 0; j <= K; ++j)
      {
        if (index == point_index)
        {
          const double s1 = step * j;
          const double s2 = s1 * s1, s3 = s2 * s1, s4 = s2 * s2, s5 = s4 * s1;
          Eigen::Matrix<double, 6, 1> beta0, beta1;
          beta0 << 1.0, s1, s2, s3, s4, s5;
          beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
          const Eigen::Vector3d p = c.transpose() * beta0;
          const Eigen::Vector3d v = c.transpose() * beta1;
          point_time = t + step * j;
          grad_p.setZero();
          grad_t = 0.0;
          grad_prev_t = 0.0;
          cost = 0.0;
          const bool evaluated = trackingGradCostP(
              index, point_time, p, v, grad_p, grad_t, grad_prev_t, cost,
              point_time);
          return evaluated;
        }
        if (j != K || i == pieces - 1)
          ++index;
      }
      t += opt.get_T1()(i);
    }
    return false;
  }

  void PolyTrajOptimizer::rebuildTrackingGradientState(
      const Eigen::Matrix3d &head, const Eigen::Matrix3d &tail,
      const Eigen::MatrixXd &inner_points, const Eigen::VectorXd &durations,
      const int cps_num_pre_piece)
  {
    const int pieces = static_cast<int>(durations.size());
    jerkOpt_.reset(head, tail, pieces);
    jerkOpt_.generate(inner_points, durations);
    piece_num_ = pieces;
    const int K = std::max(1, cps_num_pre_piece);
    const int cp_count = pieces * K + 1;
    ConstraintPoints cps;
    cps.resize_cp(cp_count);
    cps.points = jerkOpt_.getInitConstraintPoints(K);
    cps_ = cps;
  }

  bool PolyTrajOptimizer::trackingGradientAtPoint(
      const int point_index, double &cost, Eigen::Vector3d &grad_p,
      double &grad_t, double &grad_prev_t, double &point_time)
  {
    const int pieces = piece_num_;
    const int K = std::max(1, cps_num_prePiece_);
    if (pieces <= 0 || point_index < 0 || point_index >= pieces * K + 1)
      return false;
    int index = 0;
    double t = 0.0;
    for (int i = 0; i < pieces; ++i)
    {
      const double step = jerkOpt_.get_T1()(i) / static_cast<double>(K);
      const Eigen::Matrix<double, 6, 3> &c =
          jerkOpt_.get_b().block<6, 3>(i * 6, 0);
      for (int j = 0; j <= K; ++j)
      {
        if (index == point_index)
        {
          const double s1 = step * j;
          const double s2 = s1 * s1, s3 = s2 * s1, s4 = s2 * s2, s5 = s4 * s1;
          Eigen::Matrix<double, 6, 1> beta0, beta1;
          beta0 << 1.0, s1, s2, s3, s4, s5;
          beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
          const Eigen::Vector3d p = c.transpose() * beta0;
          const Eigen::Vector3d v = c.transpose() * beta1;
          point_time = t + step * j;
          grad_p.setZero();
          grad_t = 0.0;
          grad_prev_t = 0.0;
          cost = 0.0;
          return trackingGradCostP(index, point_time, p, v, grad_p, grad_t,
                                   grad_prev_t, cost, point_time);
        }
        if (j != K || i == pieces - 1)
          ++index;
      }
      t += jerkOpt_.get_T1()(i);
    }
    return false;
  }

  bool PolyTrajOptimizer::trackingCostAtPoint(const int point_index,
                                              const Eigen::Vector3d &p,
                                              const Eigen::Vector3d &v,
                                              const double t, double &cost)
  {
    Eigen::Vector3d gradp = Eigen::Vector3d::Zero();
    double gradt = 0.0, grad_prev_t = 0.0;
    cost = 0.0;
    return trackingGradCostP(point_index, t, p, v, gradp, gradt, grad_prev_t,
                             cost, t);
  }

  void PolyTrajOptimizer::configureTrackingGradientTest(
      const Eigen::Vector3d &object_p, const Eigen::Vector3d &object_v,
      const Eigen::Vector3d &relative_tracking, const double wei_tracking,
      const int cps_num_pre_piece)
  {
    object_p_ = object_p;
    object_v_ = object_v;
    object_q_ = Eigen::Quaterniond::Identity();
    relative_tracking_p_ = relative_tracking;
    wei_tracking_ = wei_tracking;
    cps_num_prePiece_ = std::max(1, cps_num_pre_piece);
    satrt_tracking_ = true;
    touch_goal_ = false;
    encirclement_tracking_active_ = true;
    encirclement_tracking_configured_ = true;
    enable_fov_costs_ = false;
    t_now_ = 0.0;
  }

  PolyTrajOptimizer::TrackingGradientCheck
  PolyTrajOptimizer::bearingGradientFiniteDifferenceCheck(
      const double bearing_weight, const double prefix_span_seconds,
      const double prefix_boost, const double perturbation)
  {
    TrackingGradientCheck check;
    check.weight = bearing_weight;
    check.wei_tracking = wei_tracking_;
    check.prefix_span = prefix_span_seconds;
    check.boost = prefix_boost;
    if (!satrt_tracking_ || !encirclement_tracking_active_ ||
        cps_num_prePiece_ <= 0)
    {
      check.reason = "TEST_ENVIRONMENT_NOT_CONFIGURED";
      return check;
    }
    if (!(perturbation > 0.0) || !std::isfinite(perturbation))
    {
      check.reason = "INVALID_PERTURBATION";
      return check;
    }

    // 保存生产状态，函数返回前完整恢复。
    const poly_traj::MinJerkOpt saved_jerk = jerkOpt_;
    const ConstraintPoints saved_cps = cps_;
    const double saved_weight = encirclement_bearing_recovery_weight_;
    const double saved_boost = prefix_bearing_recovery_boost_;
    const double saved_span = prefix_execution_span_;
    const double saved_t_now = t_now_;
    const int saved_piece_num = piece_num_;

    const int pieces = 2;
    const int K = cps_num_prePiece_;
    Eigen::Matrix3d head, tail;
    head.setZero();
    tail.setZero();
    head.col(0) = Eigen::Vector3d(0.0, 0.0, 1.5);
    head.col(1) = Eigen::Vector3d(0.9, 0.2, 0.0);
    tail.col(0) = Eigen::Vector3d(3.4, 0.5, 1.5);
    tail.col(1) = Eigen::Vector3d(0.9, -0.1, 0.0);
    // 内点：故意让实际方位显著偏离种子方位，使方位项处于活动区（e_phi_eff>0）。
    Eigen::MatrixXd inner(3, pieces - 1);
    inner << 1.6,
             1.1,
             1.5;
    Eigen::VectorXd durations(pieces);
    durations << 0.55, 0.52;

    encirclement_bearing_recovery_weight_ = bearing_weight;
    prefix_bearing_recovery_boost_ =
        std::isfinite(prefix_boost) && prefix_boost >= 1.0 ? prefix_boost : 1.0;
    prefix_execution_span_ =
        std::isfinite(prefix_span_seconds) && prefix_span_seconds > 0.0
            ? prefix_span_seconds
            : 0.0;

    // 正确的不变量：在同一 (p, v, t) 点上，trackingGradCostP 返回的 gradp
    // 必须等于它自己返回的 costp 对 p 的偏导数。这直接把"方位项相对自己的
    // 代价被多乘一次权重"这类 cost/gradient 不同源错误暴露出来，而且不受
    // MINCO 约束点之间的耦合影响（那是 getGrad2TP 的职责，不是本项检查）。
    rebuildTrackingGradientState(head, tail, inner, durations, K);
    const int cp_count = pieces * K + 1;
    check.pieces = pieces;
    check.samples = cp_count;

    std::vector<Eigen::Vector3d> grad_point(cp_count, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> pos_point(cp_count, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> vel_point(cp_count, Eigen::Vector3d::Zero());
    std::vector<double> elapsed_point(cp_count, 0.0);
    std::vector<double> cost_point(cp_count, 0.0);
    {
      int index = 0;
      double t = 0.0;
      for (int i = 0; i < pieces; ++i)
      {
        const double step = jerkOpt_.get_T1()(i) / static_cast<double>(K);
        const Eigen::Matrix<double, 6, 3> &c =
            jerkOpt_.get_b().block<6, 3>(i * 6, 0);
        for (int j = 0; j <= K; ++j)
        {
          const double s1 = step * j;
          const double s2 = s1 * s1, s3 = s2 * s1, s4 = s2 * s2, s5 = s4 * s1;
          Eigen::Matrix<double, 6, 1> beta0, beta1;
          beta0 << 1.0, s1, s2, s3, s4, s5;
          beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
          pos_point[index] = c.transpose() * beta0;
          vel_point[index] = c.transpose() * beta1;
          elapsed_point[index] = t + step * j;
          // 与 ConstraintPoints::getInitConstraintPoints 完全一致的"接缝去重"：
          // 段末点仅保留在最后一段，避免相邻段重复占用同一索引。
          if (j != K || i == pieces - 1)
            ++index;
        }
        t += jerkOpt_.get_T1()(i);
      }
    }
    for (int p = 0; p < cp_count; ++p)
    {
      double cost = 0.0;
      Eigen::Vector3d gp = Eigen::Vector3d::Zero();
      double gt = 0.0, gpt = 0.0, tt = 0.0;
      if (trackingGradientAtPoint(p, cost, gp, gt, gpt, tt))
      {
        grad_point[p] = gp;
        cost_point[p] = cost;
      }
    }
    check.prefix_points = 0;
    check.cost = 0.0;
    for (int p = 0; p < cp_count; ++p)
    {
      if (prefix_execution_span_ > 0.0 &&
          elapsed_point[p] <= prefix_execution_span_ + 1.0e-9)
        ++check.prefix_points;
      const int piece = std::min(pieces - 1, p / K);
      const int j = p - piece * K;
      const double step = durations(piece) / static_cast<double>(K);
      const double omg = (j == 0 || j == K) ? 0.5 : 1.0;
      check.cost += omg * step * cost_point[p];
    }

    // 位置自由度：gradp ?= ∂costp/∂p（中心差分，同一 v 与 t）
    double worst_pos = 0.0;
    for (int p = 0; p < cp_count; ++p)
    {
      for (int axis = 0; axis < 3; ++axis)
      {
        Eigen::Vector3d plus = pos_point[p], minus = pos_point[p];
        plus(axis) += perturbation;
        minus(axis) -= perturbation;
        double c_plus = 0.0, c_minus = 0.0;
        trackingCostAtPoint(p, plus, vel_point[p], elapsed_point[p], c_plus);
        trackingCostAtPoint(p, minus, vel_point[p], elapsed_point[p], c_minus);
        const double numeric = (c_plus - c_minus) / (2.0 * perturbation);
        const double analytic = grad_point[p](axis);
        const double abs_err = std::abs(numeric - analytic);
        if (abs_err > check.position_error_max)
        {
          check.position_error_max = abs_err;
          check.worst_position_index = axis;
          worst_pos = check.position_error_max;
        }
        const double denom =
            std::max(1.0e-6, std::abs(numeric) + std::abs(analytic));
        check.position_error_relative_max =
            std::max(check.position_error_relative_max, abs_err / denom);
        if (p == 1)
        {
          check.numeric_position[axis] = numeric;
          check.analytic_position[axis] = analytic;
        }
      }
    }
    (void)worst_pos;

    // 时间自由度：对每个积分点核对
    //     gradt == ∂costp/∂t|_(p,v 固定) + gradp·v
    // 其中 ∂costp/∂t 是"仅由目标/障碍预测沿世界时间前进"引起的显式偏导，
    // 可以用固定 (p, v) 的 t 中心差分精确测出；gradp·v 是采样点沿轨迹滑动
    // 的链式项。这条不变量正是 MINCO 时间自由度真正的 ∂J/∂T 展开式中的
    // "积分点显式项"，因此它对 T 梯度是否可信具有直接的判定力。
    double worst_time_abs = 0.0;
    for (int p = 0; p < cp_count; ++p)
    {
      double c_plus = 0.0, c_minus = 0.0;
      trackingCostAtPoint(p, pos_point[p], vel_point[p],
                          elapsed_point[p] + perturbation, c_plus);
      trackingCostAtPoint(p, pos_point[p], vel_point[p],
                          elapsed_point[p] - perturbation, c_minus);
      const double explicit_part = (c_plus - c_minus) / (2.0 * perturbation);
      Eigen::Vector3d gp = Eigen::Vector3d::Zero();
      double gt = 0.0, gpt = 0.0, tt = 0.0, cost = 0.0;
      if (!trackingGradientAtPoint(p, cost, gp, gt, gpt, tt))
        continue;
      const double predicted = explicit_part + gp.dot(vel_point[p]);
      const double abs_err = std::abs(gt - predicted);
      if (abs_err > check.time_error_max)
      {
        check.time_error_max = abs_err;
        check.worst_time_index = p;
        worst_time_abs = abs_err;
      }
      if (p == 1)
      {
        check.numeric_time[0] = explicit_part;
        check.analytic_time[0] = gt;
        check.numeric_time[1] = predicted;
        check.analytic_time[1] = gt;
      }
    }
    (void)worst_time_abs;

    // 恢复生产状态
    jerkOpt_ = saved_jerk;
    cps_ = saved_cps;
    encirclement_bearing_recovery_weight_ = saved_weight;
    prefix_bearing_recovery_boost_ = saved_boost;
    prefix_execution_span_ = saved_span;
    t_now_ = saved_t_now;
    piece_num_ = saved_piece_num;

    check.valid = true;
    check.reason = "OK";
    return check;
  }

  void PolyTrajOptimizer::setMovingObjPredictionEpoch(const double epoch)
  {
    if (!std::isfinite(epoch))
    {
      clearMovingObjPredictionEpoch();
      return;
    }
    prediction_epoch_override_ = epoch;
    prediction_epoch_override_enabled_ = true;
  }

  void PolyTrajOptimizer::clearMovingObjPredictionEpoch(void)
  {
    prediction_epoch_override_enabled_ = false;
  }

  void PolyTrajOptimizer::setMovingObjPredictionHorizon(const double horizon)
  {
    moving_obj_prediction_horizon_ = std::max(0.0, horizon);
  }

  void PolyTrajOptimizer::setCandidateSideBias(
      const Eigen::Vector3d &start, const Eigen::Vector3d &end,
      const int side, const double offset, const double conflict_progress,
      const double guidance_window, const bool enable_region_constraint)
  {
    Eigen::Vector3d direction = (end - start).cross(Eigen::Vector3d::UnitZ());
    direction.z() = 0.0;
    if (side == 0 || direction.norm() < 1.0e-6 || offset <= 0.0 ||
        !std::isfinite(offset))
    {
      clearCandidateSideBias();
      return;
    }
    candidate_side_origin_ = start;
    candidate_side_direction_ = direction.normalized();
    candidate_side_sign_ = side > 0 ? 1.0 : -1.0;
    candidate_side_offset_ = offset;
    candidate_side_conflict_progress_ =
        std::min(1.0, std::max(0.0, conflict_progress));
    candidate_side_guidance_window_ =
        std::min(1.0, std::max(1.0e-3, guidance_window));
    candidate_side_region_enabled_ = enable_region_constraint;
    candidate_side_bias_enabled_ = true;
  }

  void PolyTrajOptimizer::setCandidateSideBiasReference(
      const Eigen::Vector3d &start, const Eigen::Vector3d &end,
      const Eigen::Vector3d &reference_direction, const int side,
      const double offset, const double conflict_progress,
      const double guidance_window, const bool enable_region_constraint)
  {
    Eigen::Vector3d direction = reference_direction;
    direction.z() = 0.0;
    if (side == 0 || direction.norm() < 1.0e-6 || offset <= 0.0 ||
        !std::isfinite(offset))
    {
      setCandidateSideBias(start, end, side, offset, conflict_progress,
                           guidance_window, enable_region_constraint);
      return;
    }
    candidate_side_origin_ = start;
    candidate_side_direction_ = direction.normalized();
    candidate_side_sign_ = side > 0 ? 1.0 : -1.0;
    candidate_side_offset_ = offset;
    candidate_side_conflict_progress_ =
        std::min(1.0, std::max(0.0, conflict_progress));
    candidate_side_guidance_window_ =
        std::min(1.0, std::max(1.0e-3, guidance_window));
    candidate_side_region_enabled_ = enable_region_constraint;
    candidate_side_bias_enabled_ = true;
  }

  void PolyTrajOptimizer::setCandidatePreservationReference(
      const poly_traj::Trajectory &reference)
  {
    candidate_preserve_reference_ = reference;
    candidate_preserve_reference_valid_ =
        reference.getPieceNum() > 0 &&
        std::isfinite(reference.getTotalDuration()) &&
        reference.getTotalDuration() > 1.0e-6;
  }

  void PolyTrajOptimizer::setCandidateRiskWindow(const double conflict_time,
                                                 const double half_window)
  {
    candidate_risk_conflict_time_ = std::max(0.0, conflict_time);
    candidate_risk_half_window_ = std::max(0.0, half_window);
    candidate_risk_window_valid_ = candidate_risk_half_window_ > 1.0e-3;
  }

  void PolyTrajOptimizer::setCandidateLocalSfc(
      const std::vector<LocalSfcPlane> &planes)
  {
    candidate_local_sfc_planes_ = planes;
  }

  void PolyTrajOptimizer::clearCandidateLocalSfc(void)
  {
    candidate_local_sfc_planes_.clear();
  }

  void PolyTrajOptimizer::clearCandidateSideBias(void)
  {
    candidate_side_bias_enabled_ = false;
    candidate_side_region_enabled_ = false;
    candidate_side_origin_.setZero();
    candidate_side_direction_.setZero();
    candidate_side_sign_ = 0.0;
    candidate_side_offset_ = 0.0;
    candidate_side_conflict_progress_ = 0.5;
    candidate_side_guidance_window_ = 1.0;
    candidate_preserve_reference_valid_ = false;
    candidate_preserve_reference_.clear();
    candidate_risk_window_valid_ = false;
    candidate_risk_conflict_time_ = 0.0;
    candidate_risk_half_window_ = 0.0;
    candidate_local_sfc_planes_.clear();
  }

  void PolyTrajOptimizer::setGradientAuditContext(
      const std::string &candidate_type, const int obstacle_id)
  {
    gradient_audit_candidate_ = candidate_type;
    gradient_audit_obstacle_id_ = obstacle_id;
    gradient_audit_enabled_ = !candidate_type.empty();
    gradient_audit_middle_logged_ = false;
    visibility_cost_candidate_ = candidate_type.empty() ? "NOMINAL" : candidate_type;
  }

  void PolyTrajOptimizer::clearGradientAuditContext(void)
  {
    gradient_audit_enabled_ = false;
    gradient_audit_candidate_.clear();
    gradient_audit_obstacle_id_ = -1;
    gradient_audit_middle_logged_ = false;
    visibility_cost_candidate_ = "NOMINAL";
  }

  bool PolyTrajOptimizer::queryStaticLosClearance(
      const Eigen::Vector3d &observer, const Eigen::Vector3d &target,
      double &clearance, Eigen::Vector3d *gradient_observer,
      Eigen::Vector3d *gradient_target) const
  {
    return static_los_geometry_.querySegmentClearance(
        observer, target, clearance, gradient_observer, gradient_target, nullptr);
  }

  bool PolyTrajOptimizer::queryStaticLosClearance(
      const Eigen::Vector3d &observer, const Eigen::Vector3d &target,
      double &clearance, StaticLosWitness &witness,
      Eigen::Vector3d *gradient_observer,
      Eigen::Vector3d *gradient_target) const
  {
    return static_los_geometry_.querySegmentClearance(
        observer, target, clearance, gradient_observer, gradient_target,
        nullptr, &witness);
  }

  void PolyTrajOptimizer::evaluateCandidateSideRegion(
      const poly_traj::Trajectory &traj, double &rms_violation,
      double &max_violation) const
  {
    rms_violation = 0.0;
    max_violation = 0.0;
    if (!candidate_side_bias_enabled_ || !candidate_side_region_enabled_ ||
        traj.getPieceNum() <= 0 || traj.getTotalDuration() <= 1.0e-6)
      return;
    constexpr int kSamples = 41;
    double squared_sum = 0.0;
    int count = 0;
    for (int i = 0; i < kSamples; ++i)
    {
      const double progress = static_cast<double>(i) / (kSamples - 1);
      const double violation = std::abs(candidateSideRegionViolation(
          progress, traj.getPos(progress * traj.getTotalDuration())));
      if (violation > 0.0)
      {
        squared_sum += violation * violation;
        max_violation = std::max(max_violation, violation);
        ++count;
      }
    }
    if (count > 0)
      rms_violation = std::sqrt(squared_sum / count);
  }

  /* callbacks by the L-BFGS optimizer */
  double PolyTrajOptimizer::costFunctionCallback(void *func_data, const double *x, double *grad, const int n)
  {
    try { return costFunctionWithinBudget(func_data,x,grad,n); }
    catch(const ExecutionDeadlineExceeded &) {
      // LBFGS uses manually allocated work buffers. Return an invalid full
      // objective so its normal cleanup runs; never unwind through LBFGS.
      auto *opt=reinterpret_cast<PolyTrajOptimizer *>(func_data);
      opt->force_stop_type_=STOP_FOR_ERROR;
      if(grad && n>0) std::fill(grad,grad+n,0.0);
      return std::numeric_limits<double>::infinity();
    }
  }

  double PolyTrajOptimizer::costFunctionWithinBudget(void *func_data, const double *x, double *grad, const int n)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    opt->executionCheckpoint();
    const int position_dim = 3 * (opt->piece_num_ - 1);
    const int expected_dim = position_dim + opt->piece_num_;
    if (opt->piece_num_ <= 0 || n != expected_dim || grad == nullptr ||
        x == nullptr)
    {
      if (grad != nullptr && n > 0)
        std::fill(grad, grad + n, 0.0);
      return std::numeric_limits<double>::infinity();
    }

    opt->min_ellip_dist2_ = std::numeric_limits<double>::max();
    opt->current_cost_snapshot_ = CostGradientSnapshot();
    opt->current_static_grad_sq_ = 0.0;
    opt->current_moving_grad_sq_ = 0.0;
    opt->current_side_grad_sq_ = 0.0;
    opt->current_side_region_grad_sq_ = 0.0;
    opt->current_preserve_grad_sq_ = 0.0;
    opt->current_tracking_grad_sq_ = 0.0;
    opt->current_smoothness_grad_sq_ = 0.0;
    opt->current_variance_grad_sq_ = 0.0;
    opt->current_visibility_grad_sq_ = 0.0;

    Eigen::Map<const Eigen::MatrixXd> P(x, 3, opt->piece_num_ - 1);
    Eigen::Map<Eigen::MatrixXd> gradP(grad, 3, opt->piece_num_ - 1);
    Eigen::VectorXd T(opt->piece_num_);
    Eigen::Map<const Eigen::VectorXd> t(x + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    opt->VirtualT2RealT(t, T);
    if (!T.allFinite() || (T.array() <= 1.0e-6).any())
    {
      std::fill(grad, grad + n, 0.0);
      return std::numeric_limits<double>::infinity();
    }

    Eigen::VectorXd gradT(opt->piece_num_);
    double smoo_cost = 0, time_cost = 0;
    Eigen::VectorXd obs_swarm_feas_qvar_costs(5);

    opt->jerkOpt_.generate(P, T);

    // Native cost integration writes one sampled position for every
    // piece/K lattice entry.  The optimizer instance is reused across
    // nominal, SIDE, and piece-refined candidates, so a previous candidate
    // may have left a lattice with a different piece count.  Synchronize the
    // storage before addPVAGradCost2CT() performs its indexed writes.  This
    // is a state-layout repair only; separating rays are retained whenever
    // their lattice is still compatible and are otherwise rebuilt by the
    // normal static checker.
    if (opt->cps_num_prePiece_ <= 0)
    {
      std::fill(grad, grad + n, 0.0);
      return std::numeric_limits<double>::infinity();
    }
    const int expected_cp_count =
        opt->piece_num_ * opt->cps_num_prePiece_ + 1;
    const bool cp_shape_valid =
        opt->cps_.points.rows() == 3 &&
        opt->cps_.points.cols() == expected_cp_count &&
        opt->cps_.cp_size == expected_cp_count &&
        static_cast<int>(opt->cps_.base_point.size()) == expected_cp_count &&
        static_cast<int>(opt->cps_.direction.size()) == expected_cp_count &&
        static_cast<int>(opt->cps_.flag_temp.size()) == expected_cp_count;
    if (!cp_shape_valid)
    {
      const Eigen::MatrixXd synced_points =
          opt->jerkOpt_.getInitConstraintPoints(opt->cps_num_prePiece_);
      opt->cps_.resize_cp(expected_cp_count);
      opt->cps_.points = synced_points;
      ROS_INFO("[minco-constraint-state-sync] piece_num=%d cp_count=%d",
               opt->piece_num_, expected_cp_count);
    }

    opt->initAndGetSmoothnessGradCost2PT(gradT, smoo_cost); // Smoothness cost
    opt->current_cost_snapshot_.smoothness_cost = smoo_cost;
    opt->current_smoothness_grad_sq_ =
        opt->jerkOpt_.get_gdC().squaredNorm() + gradT.squaredNorm();

    opt->addPVAGradCost2CT(gradT, obs_swarm_feas_qvar_costs, opt->cps_num_prePiece_); // Time int cost
    double visibility_cost = 0.0;
    const Eigen::MatrixXd directional_gdC_before =
        opt->jerkOpt_.get_gdC();
    const Eigen::VectorXd directional_gdT_before = gradT;
    double directional_visibility_cost = 0.0;
    opt->addDirectionalVisibilityGradCost2CT(
        gradT, directional_visibility_cost);
    double reserve_cost = 0.0;
    opt->addTeamVisibilityReserveGradCost2CT(gradT, reserve_cost);
    visibility_cost += reserve_cost;
    const Eigen::MatrixXd directional_gdC =
        opt->jerkOpt_.get_gdC() - directional_gdC_before;
    Eigen::VectorXd directional_gdT = gradT - directional_gdT_before;
    visibility_cost += directional_visibility_cost;
    opt->current_cost_snapshot_.visibility_cost = visibility_cost;

    if (opt->iter_num_ > 3 && smoo_cost / opt->piece_num_ < 10.0) // 10.0 is an experimental value that indicates the trajectory is smooth enough.
    {
      opt->roughlyCheckConstraintPoints();
    }

    opt->jerkOpt_.getGrad2TP(gradT, gradP);
    Eigen::MatrixXd directional_gradP =
        Eigen::MatrixXd::Zero(3, opt->piece_num_ - 1);
    opt->jerkOpt_.propagateGrad2TP(
        directional_gdC, directional_gdT, directional_gradP);
    // time_cost += opt->rho_ * T(0) * piece_nums;  // same t
    // grad[n - 1] = (gradT.sum() + opt->rho_ * piece_nums) * gdT2t(x[n - 1]);  // same t

    Eigen::Map<Eigen::VectorXd> gradt(grad + (3 * (opt->piece_num_ - 1)), opt->piece_num_);
    opt->VirtualTGradCost(T, t, gradT, gradt, time_cost);

    Eigen::VectorXd directional_x_gradient = Eigen::VectorXd::Zero(n);
    if (directional_gradP.size() > 0)
      Eigen::Map<Eigen::MatrixXd>(directional_x_gradient.data(), 3,
                                  opt->piece_num_ - 1) = directional_gradP;
    for (int i = 0; i < opt->piece_num_; ++i)
    {
      double physical_time_derivative = 0.0;
      if (t(i) > 0.0)
        physical_time_derivative = t(i) + 1.0;
      else
      {
        const double denominator = (0.5 * t(i) - 1.0) * t(i) + 1.0;
        physical_time_derivative =
            (1.0 - t(i)) / (denominator * denominator);
      }
      directional_x_gradient(position_dim + i) =
          directional_gdT(i) * physical_time_derivative;
    }

    opt->current_cost_snapshot_.time_cost = time_cost;
    opt->current_cost_snapshot_.total_cost =
        smoo_cost + obs_swarm_feas_qvar_costs.sum() + time_cost + visibility_cost;
    opt->current_cost_snapshot_.directional_visibility_cost_ratio =
        opt->current_cost_snapshot_.total_cost > 1.0e-12
            ? directional_visibility_cost /
                  opt->current_cost_snapshot_.total_cost
            : 0.0;
    opt->current_cost_snapshot_.static_grad_norm =
        std::sqrt(std::max(0.0, opt->current_static_grad_sq_));
    opt->current_cost_snapshot_.moving_grad_norm =
        std::sqrt(std::max(0.0, opt->current_moving_grad_sq_));
    opt->current_cost_snapshot_.side_grad_norm =
        std::sqrt(std::max(0.0, opt->current_side_grad_sq_));
    opt->current_cost_snapshot_.side_region_grad_norm =
        std::sqrt(std::max(0.0, opt->current_side_region_grad_sq_));
    opt->current_cost_snapshot_.preserve_grad_norm =
        std::sqrt(std::max(0.0, opt->current_preserve_grad_sq_));
    opt->current_cost_snapshot_.tracking_grad_norm =
        std::sqrt(std::max(0.0, opt->current_tracking_grad_sq_));
    opt->current_cost_snapshot_.smoothness_grad_norm =
        std::sqrt(std::max(0.0, opt->current_smoothness_grad_sq_));
    opt->current_cost_snapshot_.variance_grad_norm =
        std::sqrt(std::max(0.0, opt->current_variance_grad_sq_));
    opt->current_cost_snapshot_.visibility_grad_norm =
        std::sqrt(std::max(0.0, opt->current_visibility_grad_sq_));
    opt->current_cost_snapshot_.total_grad_norm =
        Eigen::Map<const Eigen::VectorXd>(grad, n).norm();
    const Eigen::VectorXd full_x_gradient =
        Eigen::Map<const Eigen::VectorXd>(grad, n);
    const double existing_x_gradient_norm =
        (full_x_gradient - directional_x_gradient).norm();
    opt->current_cost_snapshot_.directional_visibility_grad_ratio =
        directional_x_gradient.norm() /
        std::max(1.0e-12, existing_x_gradient_norm);
    if (!opt->optimization_diagnostics_.valid)
    {
      opt->optimization_diagnostics_.initial = opt->current_cost_snapshot_;
      opt->optimization_diagnostics_.valid = true;
    }
    opt->optimization_diagnostics_.final = opt->current_cost_snapshot_;
    const int evaluation = opt->optimization_diagnostics_.evaluations;
    opt->optimization_diagnostics_.evaluations += 1;

    if (opt->gradient_audit_enabled_ &&
        (evaluation == 0 || (!opt->gradient_audit_middle_logged_ && evaluation >= 20)))
    {
      const char *phase = evaluation == 0 ? "initial" : "middle";
      ROS_INFO("[minco-gradient-audit] candidate_type=%s obstacle_id=%d phase=%s "
               "evaluation=%d region_cost=%.6g moving_cost=%.6g tracking_cost=%.6g "
               "smoothness_cost=%.6g static_cost=%.6g variance_cost=%.6g total_cost=%.6g "
               "region_grad=%.6g moving_grad=%.6g tracking_grad=%.6g "
               "smoothness_grad=%.6g static_grad=%.6g variance_grad=%.6g total_grad=%.6g",
               opt->gradient_audit_candidate_.c_str(), opt->gradient_audit_obstacle_id_,
               phase, evaluation,
               opt->current_cost_snapshot_.side_region_cost,
               opt->current_cost_snapshot_.moving_cost,
               opt->current_cost_snapshot_.tracking_cost,
               opt->current_cost_snapshot_.smoothness_cost,
               opt->current_cost_snapshot_.static_cost,
               opt->current_cost_snapshot_.variance_cost,
               opt->current_cost_snapshot_.total_cost,
               opt->current_cost_snapshot_.side_region_grad_norm,
               opt->current_cost_snapshot_.moving_grad_norm,
               opt->current_cost_snapshot_.tracking_grad_norm,
               opt->current_cost_snapshot_.smoothness_grad_norm,
               opt->current_cost_snapshot_.static_grad_norm,
               opt->current_cost_snapshot_.variance_grad_norm,
               opt->current_cost_snapshot_.total_grad_norm);
      if (evaluation >= 20)
        opt->gradient_audit_middle_logged_ = true;
    }

    // for ( int i=0; i<n; ++i )
    // {
    //   printf("%.2f,", grad[i]);
    // }
    // printf("\n");

    opt->iter_num_ += 1;
    return smoo_cost + obs_swarm_feas_qvar_costs.sum() + time_cost +
           visibility_cost;
  }

  int PolyTrajOptimizer::earlyExitCallback(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
  {
    PolyTrajOptimizer *opt = reinterpret_cast<PolyTrajOptimizer *>(func_data);

    return (!opt->executionBudgetAvailable() || opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
  }

  bool PolyTrajOptimizer::checkMovingObjSafety(const poly_traj::Trajectory &traj,
                                               const double prediction_start_time,
                                               std::string &reason) const
  {
    if (!use_time_aware_moving_obj_cost_ || !moving_objs_ ||
        moving_objs_->getObjNums() <= 0 || traj.getPieceNum() <= 0)
      return true;

    double check_end = traj.getTotalDuration();
    if (moving_obj_prediction_horizon_ > 1.0e-3)
      check_end = std::min(check_end, moving_obj_prediction_horizon_);

    constexpr double sample_dt = 0.03;
    for (double t = 0.0; t < check_end + 1.0e-6; t += sample_dt)
    {
      const double tt = std::min(t, check_end);
      const Eigen::Vector3d pos = traj.getPos(tt);
      for (int id = 0; id < moving_objs_->getObjNums(); ++id)
      {
        if (!moving_objs_->hasPrediction(id))
          continue;
        const Eigen::Vector3d obj_p =
            moving_objs_->evaluateConstVel(id, prediction_start_time + tt);
        if (!obj_p.allFinite())
          continue;
        const double distance = (pos - obj_p).norm();
        // Feedback126: per-object physical clearance — the same predicate as
        // the manager's dynamicHardClearanceForObject and the SCP rows.
        const double required_hard = movingObjHardClearanceForObject(id);
        if (distance < required_hard)
        {
          std::ostringstream oss;
          oss << "moving obstacle=" << id << " t=" << tt
              << " distance=" << distance
              << " required_hard=" << required_hard;
          reason = oss.str();
          return false;
        }
      }
    }
    return true;
  }

} // namespace ego_planner
