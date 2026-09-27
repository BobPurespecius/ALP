#include <plan_manage/local_sfc_boundary_scan.h>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace
{

bool near(const double lhs, const double rhs)
{
  return std::abs(lhs - rhs) <= 1.0e-12;
}

ego_planner::LocalSfcBoundaryScanResult scanSequence(
    const std::vector<bool> &occupied, const double step)
{
  const double max_distance = step * static_cast<double>(occupied.size() - 1);
  return ego_planner::scanFirstLocalSfcBoundary(
      max_distance, step, [&](const double lambda) {
        const size_t index = static_cast<size_t>(
            std::llround(lambda / step));
        return occupied.at(std::min(index, occupied.size() - 1));
      });
}

}  // namespace

int main()
{
  const auto case_a = scanSequence({false, false, false, false}, 0.1);
  const auto case_b = scanSequence({false, false, true, true}, 0.1);
  const auto case_c = scanSequence({false, true, true, false}, 0.1);
  const auto case_d = scanSequence({false, true}, 0.1);
  double last_lambda = -1.0;
  const auto endpoint_case = ego_planner::scanFirstLocalSfcBoundary(
      0.25, 0.1, [&](const double lambda) {
        last_lambda = lambda;
        return lambda >= 0.25 - 1.0e-12;
      });

  const bool pass_a = case_a.valid_input && !case_a.transition_found;
  const bool pass_b = case_b.transition_found &&
                      near(case_b.lambda_free, 0.1) &&
                      near(case_b.lambda_occupied, 0.2) &&
                      near(case_b.lambda_boundary, 0.15);
  const bool pass_c = case_c.transition_found &&
                      near(case_c.lambda_free, 0.0) &&
                      near(case_c.lambda_occupied, 0.1) &&
                      near(case_c.lambda_boundary, 0.05);
  const bool pass_d = case_d.transition_found &&
                      near(case_d.lambda_free, 0.0) &&
                      near(case_d.lambda_occupied, 0.1) &&
                      near(case_d.lambda_boundary, 0.05);
  const bool pass_endpoint = endpoint_case.transition_found &&
                             near(last_lambda, 0.25) &&
                             near(endpoint_case.lambda_free, 0.2) &&
                             near(endpoint_case.lambda_occupied, 0.25) &&
                             near(endpoint_case.lambda_boundary, 0.225);

  std::cout << "LOCAL_SFC_BOUNDARY_SCAN_CONTRACT="
            << (pass_a && pass_b && pass_c && pass_d && pass_endpoint
                    ? "PASS" : "FAIL")
            << " case_a_no_plane=" << pass_a
            << " case_b_first_transition=" << pass_b
            << " case_c_terminal_free_still_found=" << pass_c
            << " case_d_first_sample_occupied=" << pass_d
            << " endpoint_explicitly_covered=" << pass_endpoint << std::endl;
  return pass_a && pass_b && pass_c && pass_d && pass_endpoint
             ? EXIT_SUCCESS : EXIT_FAILURE;
}
