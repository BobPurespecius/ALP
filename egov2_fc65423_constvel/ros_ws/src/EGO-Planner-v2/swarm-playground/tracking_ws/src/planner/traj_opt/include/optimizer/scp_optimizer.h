#ifndef EGO_PLANNER_SCP_OPTIMIZER_H_
#define EGO_PLANNER_SCP_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <string>

namespace ego_planner
{

struct SCPQPSolveResult
{
  bool success{false};
  int status{0};
  Eigen::VectorXd step;
  std::string status_text;
  int iterations{0};
  double primal_residual{0.0};
  double dual_residual{0.0};
  int rho_updates{0};
  double rho_estimate{0.0};
};

/** A small, solver-specific adapter for the convex QP used by one SCP step. */
class SCPOptimizer
{
public:
  static SCPQPSolveResult solve(const Eigen::VectorXd &gradient,
                                const Eigen::MatrixXd &constraint_matrix,
                                const Eigen::VectorXd &lower_bound,
                                const Eigen::VectorXd &upper_bound,
                                const Eigen::VectorXd &hessian_diagonal, double wall_time_limit=0.0);

  static SCPQPSolveResult solve(const Eigen::VectorXd &gradient,
                                const Eigen::MatrixXd &constraint_matrix,
                                const Eigen::VectorXd &lower_bound,
                                const Eigen::VectorXd &upper_bound,
                                double hessian_diagonal);
};

}  // namespace ego_planner

#endif  // EGO_PLANNER_SCP_OPTIMIZER_H_
