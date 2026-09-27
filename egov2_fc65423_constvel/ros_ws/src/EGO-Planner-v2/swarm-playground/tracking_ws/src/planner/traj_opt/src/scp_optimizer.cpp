#include "optimizer/scp_optimizer.h"

#include <osqp.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <vector>

namespace ego_planner
{
namespace
{
void freeProblem(OSQPData *data)
{
  if (data == nullptr)
    return;
  if (data->P != nullptr)
    csc_spfree(data->P);
  if (data->A != nullptr)
    csc_spfree(data->A);
  if (data->q != nullptr)
    c_free(data->q);
  if (data->l != nullptr)
    c_free(data->l);
  if (data->u != nullptr)
    c_free(data->u);
}

std::string statusText(const int status)
{
  switch (status)
  {
  case OSQP_SOLVED:
    return "solved";
  case OSQP_SOLVED_INACCURATE:
    return "solved_inaccurate";
  case OSQP_PRIMAL_INFEASIBLE:
    return "primal_infeasible";
  case OSQP_PRIMAL_INFEASIBLE_INACCURATE:
    return "primal_infeasible_inaccurate";
  case OSQP_MAX_ITER_REACHED:
    return "max_iter";
  default:
    return "osqp_status_" + std::to_string(status);
  }
}
}  // namespace

SCPQPSolveResult SCPOptimizer::solve(const Eigen::VectorXd &gradient,
                                     const Eigen::MatrixXd &constraint_matrix,
                                     const Eigen::VectorXd &lower_bound,
                                     const Eigen::VectorXd &upper_bound,
                                     const Eigen::VectorXd &hessian_diagonal, double wall_time_limit)
{
  SCPQPSolveResult result;
  const int n = static_cast<int>(gradient.size());
  const int m = static_cast<int>(constraint_matrix.rows());
  if (n <= 0 || m <= 0 || constraint_matrix.cols() != n ||
      lower_bound.size() != m || upper_bound.size() != m ||
      !gradient.allFinite() || !constraint_matrix.allFinite() ||
      !lower_bound.allFinite() || !upper_bound.allFinite() ||
      (lower_bound.array() > upper_bound.array()).any() ||
      hessian_diagonal.size() != n || !hessian_diagonal.allFinite() ||
      (hessian_diagonal.array() <= 0.0).any())
  {
      result.status_text = "invalid_qp";
    return result;
  }

  std::vector<c_int> a_col_ptr(static_cast<size_t>(n) + 1, 0);
  std::vector<c_int> a_row_idx;
  std::vector<c_float> a_values;
  a_row_idx.reserve(static_cast<size_t>(std::max(n, m)));
  a_values.reserve(static_cast<size_t>(std::max(n, m)));
  constexpr double kSparseEps = 1.0e-12;
  for (int col = 0; col < n; ++col)
  {
    a_col_ptr[static_cast<size_t>(col)] =
        static_cast<c_int>(a_values.size());
    for (int row = 0; row < m; ++row)
    {
      const double value = constraint_matrix(row, col);
      if (std::abs(value) <= kSparseEps)
        continue;
      a_row_idx.push_back(static_cast<c_int>(row));
      a_values.push_back(static_cast<c_float>(value));
    }
  }
  a_col_ptr[static_cast<size_t>(n)] = static_cast<c_int>(a_values.size());

  const auto osqpBound = [](const double value) -> c_float {
#ifdef OSQP_INFTY
    if (value <= -1.0e19)
      return static_cast<c_float>(-OSQP_INFTY);
    if (value >= 1.0e19)
      return static_cast<c_float>(OSQP_INFTY);
#endif
    return static_cast<c_float>(value);
  };

  OSQPData data{};
  data.n = n;
  data.m = m;
  data.P = csc_spalloc(n, n, n, 1, 0);
  data.A = csc_spalloc(m, n, std::max<c_int>(1, a_col_ptr.back()), 1, 0);
  data.q = static_cast<c_float *>(c_malloc(sizeof(c_float) * n));
  data.l = static_cast<c_float *>(c_malloc(sizeof(c_float) * m));
  data.u = static_cast<c_float *>(c_malloc(sizeof(c_float) * m));
  if (data.P == nullptr || data.A == nullptr || data.q == nullptr ||
      data.l == nullptr || data.u == nullptr)
  {
    freeProblem(&data);
    result.status_text = "allocation_failure";
    return result;
  }

  // The SCP model uses a diagonal positive definite Hessian.  Keeping P
  // diagonal also makes the adapter independent of Eigen's sparse formats.
  for (int col = 0; col <= n; ++col)
    data.P->p[col] = col;
  for (int i = 0; i < n; ++i)
  {
    data.P->i[i] = i;
    data.P->x[i] = static_cast<c_float>(hessian_diagonal(i));
    data.q[i] = static_cast<c_float>(gradient(i));
  }

  for (int col = 0; col < n; ++col)
  {
    data.A->p[col] = a_col_ptr[static_cast<size_t>(col)];
  }
  data.A->p[n] = a_col_ptr[static_cast<size_t>(n)];
  for (size_t nz = 0; nz < a_values.size(); ++nz)
  {
    data.A->i[nz] = a_row_idx[nz];
    data.A->x[nz] = a_values[nz];
  }
  for (int row = 0; row < m; ++row)
  {
    data.l[row] = osqpBound(lower_bound(row));
    data.u[row] = osqpBound(upper_bound(row));
  }

  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = 0;
  settings.polish = 1;
  // Keep OSQP's built-in numerical safeguards explicit.  Scaling and adaptive
  // rho are solver conditioning controls only; the nonlinear SCP recheck
  // remains authoritative for every hard sampled constraint.  The previous
  // 1e-4 tolerances were tighter than this vendor's default and caused
  // avoidable max-iteration exits on redundant corridor/dynamics rows.
  settings.scaling = 10;
  settings.adaptive_rho = 1;
  settings.adaptive_rho_interval = 0;
  settings.adaptive_rho_tolerance = 5.0;
  settings.check_termination = 25;
  settings.scaled_termination = 0;
  settings.warm_start = 1;
  settings.eps_abs = 1.0e-3;
  settings.eps_rel = 1.0e-3;
  settings.max_iter = 2000;
#ifdef PROFILING
  if(wall_time_limit>0.0) settings.time_limit=wall_time_limit;
#endif

  OSQPWorkspace *work = nullptr;
  const c_int setup_status = osqp_setup(&work, &data, &settings);
  if (setup_status != 0 || work == nullptr)
  {
    result.status = setup_status;
    result.status_text = "setup_failure";
    freeProblem(&data);
    return result;
  }

  const c_int solve_status = osqp_solve(work);
  result.status = work->info != nullptr ? work->info->status_val : solve_status;
  result.status_text = statusText(result.status);
  if (work->info != nullptr)
  {
    result.iterations = static_cast<int>(work->info->iter);
    result.primal_residual = static_cast<double>(work->info->pri_res);
    result.dual_residual = static_cast<double>(work->info->dua_res);
#if EMBEDDED != 1
    result.rho_updates = static_cast<int>(work->info->rho_updates);
    result.rho_estimate = static_cast<double>(work->info->rho_estimate);
#endif
  }
  if ((result.status == OSQP_SOLVED || result.status == OSQP_SOLVED_INACCURATE) &&
      work->solution != nullptr && work->solution->x != nullptr)
  {
    result.step.resize(n);
    for (int i = 0; i < n; ++i)
      result.step(i) = work->solution->x[i];
    result.success = result.step.allFinite();
  }

  osqp_cleanup(work);
  freeProblem(&data);
  return result;
}

SCPQPSolveResult SCPOptimizer::solve(const Eigen::VectorXd &gradient,
                                     const Eigen::MatrixXd &constraint_matrix,
                                     const Eigen::VectorXd &lower_bound,
                                     const Eigen::VectorXd &upper_bound,
                                     const double hessian_diagonal)
{
  if (!std::isfinite(hessian_diagonal) || hessian_diagonal <= 0.0)
  {
    SCPQPSolveResult result;
    result.status_text = "invalid_qp";
    return result;
  }
  return solve(gradient, constraint_matrix, lower_bound, upper_bound,
               Eigen::VectorXd::Constant(gradient.size(), hessian_diagonal));
}

}  // namespace ego_planner
