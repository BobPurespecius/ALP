// =============================================================================
// 方位恢复项 cost / gradient 一致性核对（中心有限差分）
//
// 这是本轮"最小数学验证"，不是大规模 unit test。
// 它直接调用生产代码 PolyTrajOptimizer::trackingGradCostP()：
//   * 解析梯度 = 生产代码返回的 gradp / gradt / grad_prev_t
//   * 数值梯度 = 完全相同的 cost 函数在扰动后的 MINCO 参数上重算
// 两者必须一致。修复前方位项相对它自己的代价多乘了一个 wei_tracking_，
// 这里会立刻表现为显著误差。
//
// 同时核对前缀进度软代价对时间自由度的解析 / 数值梯度。
//
// 编译：见 scripts/tests/build_and_run_bearing_fd.sh
// =============================================================================
#include <optimizer/poly_traj_optimizer.h>

#include <cstdio>
#include <string>
#include <vector>

using ego_planner::PolyTrajOptimizer;

namespace
{
int failures = 0;

void report(const char *name, const PolyTrajOptimizer::TrackingGradientCheck &check,
            const double tolerance)
{
  std::printf("[%s] valid=%d reason=%s pieces=%d samples=%d prefix_points=%d\n",
              name, static_cast<int>(check.valid), check.reason.c_str(),
              check.pieces, check.samples, check.prefix_points);
  std::printf("[%s] cost=%.9f weight=%.6f boost=%.3f prefix_span=%.3f wei_tracking=%.6f\n",
              name, check.cost, check.weight, check.boost, check.prefix_span,
              check.wei_tracking);
  std::printf("[%s] BEARING_POSITION_GRADIENT_FINITE_DIFF_ERROR_MAX=%.3e "
              "RELATIVE=%.3e\n",
              name, check.position_error_max, check.position_error_relative_max);
  std::printf("[%s] BEARING_TIME_GRADIENT_FINITE_DIFF_ERROR_MAX=%.3e "
              "RELATIVE=%.3e\n",
              name, check.time_error_max, check.time_error_relative_max);
  for (int i = 0; i < 3; ++i)
  {
    std::printf("[%s]   pos[%d] numeric=%12.6f analytic=%12.6f\n", name, i,
                check.numeric_position[i], check.analytic_position[i]);
  }
  std::printf("[%s]   time point1: explicit_dcost_dt=%12.6f  predicted_gradt=%12.6f"
              "  analytic_gradt=%12.6f\n",
              name, check.numeric_time[0], check.numeric_time[1],
              check.analytic_time[0]);
  std::printf("[%s] worst_pos_index=%d worst_time_index=%d\n", name,
              check.worst_position_index, check.worst_time_index);
  // 判据只针对 cost/gradient 同源性（位置项）。时间项的输出是信息性的：
  // gradt 还包含 ∂J/∂p·(v - v_obj) 的链式项，不能直接与纯 ∂J/∂t 比较。
  const bool ok = check.valid && check.position_error_max <= tolerance &&
                  check.time_error_max <= tolerance;
  std::printf("[%s] VERDICT=%s (tolerance=%.1e)\n\n", name,
              ok ? "PASS" : "FAIL", tolerance);
  if (!ok)
    ++failures;
}
}  // namespace

int main(int argc, char **argv)
{
  // 分段可执行：默认只跑 bearing 一致性（Phase A 核心判据）。
  // --with-prefix 才跑 prefix 进度项核对。
  bool with_prefix = false;
  for (int i = 1; i < argc; ++i)
    if (std::string(argv[i]) == "--with-prefix") with_prefix = true;
  PolyTrajOptimizer optimizer;
  // 权重取生产默认的 0.08；前缀加强取生产默认的 2.0。
  // 只读测试环境：目标匀速 +x，编队种子在目标右后侧 1.7 m 附近。
  optimizer.configureTrackingGradientTest(
      Eigen::Vector3d(-6.0, 0.2, 1.5),   // object_p
      Eigen::Vector3d(0.928, 0.0, 0.0),  // object_v
      Eigen::Vector3d(-1.5, 0.85, 0.0),  // relative_tracking (编队种子)
      1.0,                               // wei_tracking
      5);                                // cps_num_pre_piece

  const double tolerance = 1.0e-4;

  // 1) 前缀加强生效（prefix_span = 0.6 s，轨迹总时长约 1.57 s）
  report("PREFIX_BOOSTED",
         optimizer.bearingGradientFiniteDifferenceCheck(0.08, 0.6, 2.0, 1.0e-6),
         tolerance);

  // 2) 无前缀加强（prefix_span = 0，boost 退化 1.0）
  report("NO_PREFIX",
         optimizer.bearingGradientFiniteDifferenceCheck(0.08, 0.0, 2.0, 1.0e-6),
         tolerance);

  // 3) 大权重：修复前 W^2 放大误差会随权重线性放大，这里必须同样成立
  report("LARGE_WEIGHT",
         optimizer.bearingGradientFiniteDifferenceCheck(0.30, 0.6, 2.0, 1.0e-6),
         tolerance);

  if (!with_prefix)
  {
    std::printf("BEARING_COST_GRADIENT_WEIGHT_FIXED=%s\n",
                failures == 0 ? "YES" : "NO");
    return failures == 0 ? 0 : 1;
  }

  // ---- 前缀进度软代价的时间梯度核对 ----
  const auto prefix_check =
      optimizer.prefixProgressTimeGradientCheck(8.0, 1.30, 3.00, 1.0e-6);
  std::printf("[PREFIX_PROGRESS] valid=%d reason=%s cost=%.9f span=%.3f weight=%.3f\n",
              static_cast<int>(prefix_check.valid), prefix_check.reason.c_str(),
              prefix_check.prefix_progress_cost,
              prefix_check.prefix_progress_span,
              prefix_check.prefix_progress_weight);
  std::printf("[PREFIX_PROGRESS] PREFIX_PROGRESS_TIME_GRADIENT_NUMERIC=%.9f "
              "ANALYTIC=%.9f ERROR=%.3e\n",
              prefix_check.prefix_progress_numeric_time_gradient,
              prefix_check.prefix_progress_analytic_time_gradient,
              prefix_check.prefix_progress_time_gradient_error);
  // 非循环核对：用生产 getGrad2TP 装配 (P,T) 梯度，与决策变量中心差分比较。
  const auto prefix_full =
      optimizer.prefixProgressFullGradientCheck(8.0, 1.30, 3.00, 1.0e-6);
  std::printf("[PREFIX_FULL] valid=%d reason=%s cost=%.9f\n",
              static_cast<int>(prefix_full.valid), prefix_full.reason.c_str(),
              prefix_full.prefix_progress_cost);
  std::printf("[PREFIX_FULL] POSITION_MAX_ABS_ERR=%.3e TIME_MAX_ABS_ERR=%.3e\n",
              prefix_full.position_error_max, prefix_full.time_error_max);
  std::printf("[PREFIX_FULL] NUMERIC_TOTAL=%.9f ANALYTIC_TOTAL=%.9f\n",
              prefix_full.prefix_progress_numeric_time_gradient,
              prefix_full.prefix_progress_analytic_time_gradient);
  const bool prefix_ok = prefix_full.valid &&
                         prefix_full.position_error_max <= 1.0e-3 &&
                         prefix_full.time_error_max <= 1.0e-3 &&
                         prefix_full.prefix_progress_cost > 1.0e-9;
  // 若该配置下前缀进度代价本身不活跃（速度亏欠<=0 => cost==0），
  // 梯度必然为 0，此时本项不构成证据，标记 NOT_APPLICABLE 而不是误判 FAIL。
  const bool prefix_applicable = prefix_full.valid &&
                                 prefix_full.prefix_progress_cost > 1.0e-9;
  if (!prefix_applicable)
  {
    std::printf("[PREFIX_PROGRESS] PREFIX_TIME_GRADIENT_FIXED=NOT_APPLICABLE "
                "(cost=%.3e, 该配置下前缀速度亏欠<=0)\n",
                prefix_full.prefix_progress_cost);
    ++failures;
  }
  else
  {
    std::printf("[PREFIX_PROGRESS] PREFIX_TIME_GRADIENT_FIXED=%s\n",
                prefix_ok ? "YES" : "NO");
    if (!prefix_ok)
      ++failures;
  }

  std::printf("BEARING_COST_GRADIENT_WEIGHT_FIXED=%s\n",
              failures == 0 ? "YES" : "NO");
  return failures == 0 ? 0 : 1;
}
