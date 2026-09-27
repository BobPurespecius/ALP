#ifndef MULTI_UAV_FORMATION_REALIZED_TEAM_VALIDATOR_H_
#define MULTI_UAV_FORMATION_REALIZED_TEAM_VALIDATOR_H_

/* 阶段 F：RealizedTeamValidator —— Team 建议的最终执行守门人。
 *
 * 职责边界（任务书第十五节，必须严格遵守）：
 *  - 只接收三条【已经由 Local 生成并通过本机 hard preflight】的 realized
 *    polynomial（poly_traj::Trajectory），加上与 proposal 相同的 identity。
 *  - 不优化、不修改任何轨迹；不是 shadow planner。
 *  - 只计算/验证：
 *      1) identity：三条 realized 必须绑定同一 task reference generation、
 *         同一 world snapshot identity、同一 activation、同一 horizon；
 *      2) Local certificates：realization_valid 全 true；
 *      3) Team interaction：三条真实 realized 轨迹的两两最小间距
 *         （0.03 s 采样），阈值 = coordinator 既有 swarm_clearance，
 *         不新发明第二套 safety threshold；
 *      4) realized visibility：对真实 polynomial 采样（yaw 用 executor 同一
 *         advanceTargetFacingYaw contract 前向 rollout，不做瞬时对准），
 *         计算 realized Q2 / ACC / redundancy / encirclement；
 *      5) baseline comparison：baseline = 同一 activation / horizon 下当前
 *         实际 committed Local triple，同一套指标；字典序比较
 *         REALIZED_TEAM_BENEFICIAL。
 *  - 只有 PASS 才允许生成 execution transaction；Team reference optimizer
 *    自己的 before/after（REFERENCE_METRIC）不能决定执行。
 */

#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <optimizer/poly_traj_utils.hpp>
#include <multi_uav_formation/encirclement_geometry.h>
#include <multi_uav_formation/tracking_visibility_geometry.h>

namespace multi_uav_formation
{

struct RealizedTeamMetrics
{
  bool valid{false};
  double q2{0.0};            // 至少两机可见的连续量均值
  double accumulated{0.0};   // 平均连续可见性
  double redundancy{0.0};    // 1 - multiviewQuality（冗余/聚集度，越低越好）
  double encirclement{0.0};  // 圆周间隔代价（25°/170°，越低越好）
  double min_pairwise_distance{std::numeric_limits<double>::infinity()};
  double max_pairwise_distance{0.0};
  // Predictive relay uses the second-largest signed visibility margin of the
  // three actually realized trajectories.  Positive means that at least two
  // observers satisfy the shared static/dynamic-LOS/FOV/range contract.
  double min_m2{std::numeric_limits<double>::infinity()};
  double min_m2_world_time{0.0};
  bool binary_visibility_valid{true};
  std::vector<int> binary_visible_counts;  // -1 means invalid authority sample
  /* 逐采样连续 Q2(与 binary_visible_counts 同索引对齐;NaN 表示该采样
   * authority 无效)。用于 repair_k==3 的逐采样 Q2 底座:临界窗的定义
   * (另两机可见)使容斥式对第三机零敏感(v1=v2=1 时 dQ2/dv0=0),全时域
   * 均值口径把 K3 收益结构性记为零、却把修复在其它采样点的微小代价全额
   * 计入(run 20260925_021921:均值仅降 1.19e-4 > 1e-4 被拒,而逐采样
   * 最大降幅 ~2.5e-6)。 */
  std::vector<double> q2_samples;
};

struct RealizedTeamMember
{
  bool valid{false};
  poly_traj::Trajectory trajectory;  // realized polynomial（或 baseline）
  // 轨迹自身的绝对起始时刻：realized = activation；baseline = 各机 committed
  // 轨迹的 start_time。采样时按 t - start_time 截断。
  double start_time{0.0};
  // activation 时刻由 Local candidate 预测链提供的执行器 yaw 状态。
  // Validator 只能用共享 advanceTargetFacingYaw 向前滚动，禁止把每个采样点
  // 瞬时对准目标。
  bool initial_yaw_valid{false};
  double initial_yaw{0.0};
  double initial_yaw_rate{0.0};
};

struct RealizedTeamSideContract
{
  bool active{false};
  Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
  Eigen::Vector3d direction{Eigen::Vector3d::Zero()};
  int side_sign{0};
  double offset{0.0};
  double conflict_progress{0.5};
  double guidance_window{1.0};
  poly_traj::Trajectory seed_trajectory;
};

struct RealizedTeamValidationInput
{
  // ---- identity（必须与 proposal 完全一致）----
  std::uint64_t team_reference_id{0};
  std::uint64_t task_reference_generation{0};
  std::uint64_t target_prediction_revision{0};
  std::uint64_t dynamic_prediction_identity{0};
  std::uint64_t static_map_revision{0};
  std::uint64_t visibility_model_version{0};
  double activation_time{0.0};
  double execution_horizon{0.0};

  std::array<RealizedTeamMember, 3> realized;
  std::array<RealizedTeamMember, 3> baseline;
  std::array<RealizedTeamSideContract, 3> side_contracts;

  // 任务几何（CooperativeTaskReference 的 slot 映射，用于 encirclement
  // 指标的 bearing 排序无关性之外的语义锚定与建议反解）。
  std::array<int, 3> slot_for_uav{{0, 1, 2}};
  std::array<double, 3> desired_bearing{{0.0, 0.0, 0.0}};

  // Optional persistent make-before-break contract.  These are world-time
  // deadlines, not trajectory-local ratios.
  bool handoff_contract_active{false};
  std::uint64_t handoff_contract_id{0};
  int outgoing_uav{-1};
  int incoming_uav{-1};
  int stable_observer_uav{-1};
  double acquire_by_world_time{0.0};
  double preserve_until_world_time{0.0};
  double required_overlap{0.0};
  double required_visibility_margin{0.0};
  std::uint8_t repair_k{0};
  int repair_target_uav{-1};
  double critical_begin_world_time{0.0};
  double critical_end_world_time{0.0};
};

struct RealizedTeamValidationResult
{
  bool pass{false};
  std::string reason;
  RealizedTeamMetrics realized;
  RealizedTeamMetrics baseline;
  // 与任务书第三十二节对应的决策字段。
  bool identity_ok{false};
  bool certificates_ok{false};
  bool interaction_ok{false};
  bool handoff_ok{false};
  bool side_topology_ok{false};
  double realized_overlap{0.0};
  int critical_k3_before{0};
  int critical_k3_after{0};
  int total_k3_before{0};
  int total_k3_after{0};
  int k2_regression_samples{0};
};

class RealizedTeamValidator
{
 public:
  // 可见性求值回调：与 coordinator 的 continuousVisibility 同一实现
  // （drone, observer_p, target_p, world_time, yaw -> 连续可见性 v∈[0,1]）。
  using VisibilityFn = std::function<ContinuousVisibilitySample(
      int drone, const Eigen::Vector3d &observer,
      const Eigen::Vector3d &target, double world_time, double yaw)>;
  using TargetFn = std::function<Eigen::Vector3d(double world_time)>;

  struct Params
  {
    double sample_dt{0.03};          // 与 planner 采样步长一致
    double swarm_min_separation{0.50}; // 既有 swarm_clearance，不新发明阈值
    double q2_epsilon{1.0e-4};       // 字典序比较容差
    double acc_epsilon{1.0e-4};
    double yaw_rate_limit{2.0 * M_PI};   // executor 同一 target-facing contract
    double yaw_accel_limit{5.0 * M_PI};
    double yaw_dt{0.05};             // yaw rollout 步长
  };

  RealizedTeamValidationResult validate(
      const RealizedTeamValidationInput &input, const Params &params,
      const VisibilityFn &visibility, const TargetFn &target_at) const;

 private:
  static RealizedTeamMetrics evaluateMetrics(
      const std::array<RealizedTeamMember, 3> &members,
      const std::array<int, 3> &slot_for_uav, double activation,
      double horizon, const Params &params, const VisibilityFn &visibility,
      const TargetFn &target_at);
};

}  // namespace multi_uav_formation

#endif  // MULTI_UAV_FORMATION_REALIZED_TEAM_VALIDATOR_H_
