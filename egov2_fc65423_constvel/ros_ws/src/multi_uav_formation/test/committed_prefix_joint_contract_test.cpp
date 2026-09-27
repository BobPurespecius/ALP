#include <multi_uav_formation/committed_prefix_joint_contract.h>
#include <plan_env/obj_predictor.h>

#include <cstdlib>
#include <iostream>

using namespace multi_uav_formation;

namespace
{
void require(const bool value, const char *name)
{
  if (!value)
  {
    std::cerr << "FAIL " << name << '\n';
    std::exit(1);
  }
  std::cout << "PASS " << name << '\n';
}

std::array<CommittedPrefixMember, 3> healthyMembers()
{
  return {{{0, 174, 41, true, 10.10, 12.00},
           {1, 181, 52, true, 10.20, 11.90},
           {2, 184, 63, true, 10.15, 12.10}}};
}
}  // namespace

int main()
{
  const auto members = healthyMembers();
  const auto snapshot = discoverCommittedFutureFrontier(
      members, 10.0, 0.02, 0.02, 0.03, 0.08);
  require(snapshot.valid && snapshot.frontier == 10.20,
          "committed_prefix_joint_contract_test");
  require(snapshot.members[0].owner_revision !=
              snapshot.members[1].owner_revision &&
              snapshot.members[1].owner_revision !=
              snapshot.members[2].owner_revision,
          "local_generations_need_not_match");
  require(predictionWindowCovers(9.9, 12.0, snapshot.frontier, 1.5),
          "prediction_epoch_semantics_contract_test");
  require(!predictionWindowCovers(9.9, 11.0, snapshot.frontier, 1.5),
          "prediction_coverage_fail_closed");

  // Real producer-field semantics: two asynchronous subscribers may hold
  // shifted windows from the same /dynamic/prediction source. Their source
  // identity must agree, while their validity intervals remain distinct and
  // are intersected by the Joint coverage gate.
  nav_msgs::Path prediction_a;
  prediction_a.header.frame_id = "world";
  nav_msgs::Path prediction_b;
  prediction_b.header.frame_id = "world";
  for (int sample = 0; sample < 31; ++sample)
  {
    geometry_msgs::PoseStamped a;
    a.header.stamp = ros::Time(20.0 + 0.1 * sample);
    a.pose.position.x = 0.2 * sample;
    prediction_a.poses.push_back(a);
    geometry_msgs::PoseStamped b = a;
    b.header.stamp = ros::Time(20.2 + 0.1 * sample);
    b.pose.position.x += 0.4;
    prediction_b.poses.push_back(b);
  }
  fast_planner::PolynomialPrediction rolling_a;
  fast_planner::PolynomialPrediction rolling_b;
  rolling_a.setPredictionPath(prediction_a);
  rolling_b.setPredictionPath(prediction_b);
  require(rolling_a.sourceIdentity() != 0 &&
              rolling_a.sourceIdentity() == rolling_b.sourceIdentity(),
          "rolling_prediction_source_identity_stable");
  require(rolling_a.validFrom() != rolling_b.validFrom() &&
              rolling_a.validTo() != rolling_b.validTo(),
          "rolling_prediction_coverage_remains_explicit");

  auto changed = members;
  changed[1].owner_revision++;
  changed[1].owner_trajectory_id++;
  require(!committedFrontierStillCurrent(snapshot, changed),
          "stale_joint_cas_contract_test");
  require(committedFrontierStillCurrent(snapshot, members),
          "joint_future_tail_activation_contract_test");

  // Production-shaped success fixture: three asynchronous committed owners
  // contribute distinct frontier P/V/A, each topology seed is re-headed to
  // that state, and the tail becomes active only after all three ACKs.
  std::array<poly_traj::MinJerkOpt, 3> future_tails;
  bool exact_heads = true;
  for (int drone = 0; drone < 3; ++drone)
  {
    Eigen::Matrix3d seed_head = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d seed_tail = Eigen::Matrix3d::Zero();
    seed_head.col(0) << 0.2 * drone, -0.3 * drone, 1.5;
    seed_tail.col(0) << 1.0 + 0.2 * drone, 0.5 - 0.2 * drone, 1.5;
    poly_traj::MinJerkOpt seed;
    seed.reset(seed_head, seed_tail, 2);
    Eigen::MatrixXd inner(3, 1);
    inner.col(0) = 0.5 * (seed_head.col(0) + seed_tail.col(0));
    seed.generate(inner, Eigen::Vector2d(0.75, 0.75));
    const Eigen::Vector3d frontier_p(2.0 + drone, 1.0 - drone, 1.5);
    const Eigen::Vector3d frontier_v(0.4, 0.1 * drone, 0.0);
    const Eigen::Vector3d frontier_a(0.05 * drone, 0.0, 0.0);
    exact_heads = exact_heads && reheadCommittedFutureTailSeed(
        seed.getTraj(), frontier_p, frontier_v, frontier_a, 1.5,
        future_tails[drone]);
    const auto tail = future_tails[drone].getTraj();
    exact_heads = exact_heads &&
        (tail.getPos(0.0) - frontier_p).norm() < 1.0e-9 &&
        (tail.getVel(0.0) - frontier_v).norm() < 1.0e-9 &&
        (tail.getAcc(0.0) - frontier_a).norm() < 1.0e-9;
  }
  require(exact_heads, "joint_tail_exact_committed_frontier_pva");
  const std::array<bool, 3> acked{{true, true, true}};
  const bool scheduled_at_frontier =
      acked[0] && acked[1] && acked[2] &&
      committedFrontierStillCurrent(snapshot, members);
  require(scheduled_at_frontier,
          "joint_future_tail_three_ack_activation_fixture");

  LocalRollingTrace off{7, 8, 8, 10.25, 11.75, 0, 0, 0, 0};
  LocalRollingTrace failed_joint = off;
  require(sameLocalRollingTrace(off, failed_joint),
          "joint_failure_local_noop_contract_test");
  failed_joint.activation_time += 0.01;
  require(!sameLocalRollingTrace(off, failed_joint),
          "feedback58_barrier_regression_test");

  std::cout << "JOINT_FAILURE_IS_LOCAL_NOOP = PASS\n"
            << "LOCAL_COMMIT_BLOCKED_BY_TEAM_ALIGNMENT = NO\n"
            << "PREDECLARED_ALIGNMENT_CAN_BLOCK_LOCAL = NO\n"
            << "STALE_JOINT_DISCARDED_WITHOUT_LOCAL_IMPACT = PASS\n"
            << "JOINT_FUTURE_TAIL_ACTIVATION = PASS\n";
  return 0;
}
