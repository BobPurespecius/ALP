#include <plan_manage/planner_manager.h>
#include <cassert>
#include <cmath>
#include <iostream>

using namespace ego_planner;

static poly_traj::Trajectory makeSegment(const Eigen::Vector3d &a,
                                         const Eigen::Vector3d &b,
                                         double duration)
{
  Eigen::Matrix3d head = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d tail = Eigen::Matrix3d::Zero();
  head.col(0) = a;
  tail.col(0) = b;
  poly_traj::MinJerkOpt opt;
  opt.reset(head, tail, 2);
  Eigen::MatrixXd inner(3, 1);
  inner.col(0) = 0.5 * (a + b);
  opt.generate(inner, Eigen::Vector2d::Constant(duration * 0.5));
  return opt.getTraj();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fresh_moving_initializer_contract_test");
  ros::NodeHandle nh("/fresh_initializer_contract");
  nh.setParam("manager/max_vel", 3.0);
  nh.setParam("manager/max_acc", 6.0);
  nh.setParam("manager/max_jer", 20.0);
  nh.setParam("manager/polyTraj_piece_length", 1.0);
  nh.setParam("manager/drone_id", 0);
  nh.setParam("optimization/constraint_points_perPiece", 5);

  EGOPlannerManager manager;
  manager.initPlanModules(nh, PlanningVisualization::Ptr(new PlanningVisualization(nh)));
  const Eigen::Vector3d p0(0.0, 0.0, 1.2);
  const Eigen::Vector3d old_endpoint(0.4, 0.0, 1.2);
  const Eigen::Vector3d distant_target(4.0, 0.0, 1.2);
  std::string reason;

  const auto short_suffix = makeSegment(p0, old_endpoint, 0.35);
  assert(!manager.canReuseRemainingSuffix(
      p0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), distant_target,
      Eigen::Vector3d::Zero(), short_suffix, reason));
  std::cout << "OLD_SUFFIX_NEW_TARGET_REJECTED=PASS reason=" << reason << std::endl;

  poly_traj::MinJerkOpt fresh;
  std::string source;
  double required_speed = 0.0, required_acc = 0.0, required_jerk = 0.0;
  const bool fresh_ok = manager.buildFreshMovingInitializer(
      p0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), distant_target,
      Eigen::Vector3d(0.2, 0.0, 0.0), fresh, source, required_speed,
      required_acc, required_jerk);
  assert(fresh_ok);
  assert(fresh.getTraj().getTotalDuration() > 0.5);
  assert(required_speed < 3.0 * 1.15);
  assert(std::isfinite(required_acc) && std::isfinite(required_jerk));
  assert(fresh.getTraj().getVel(fresh.getTraj().getTotalDuration()).norm() > 0.01);
  std::cout << "FRESH_RETIMED_INITIALIZER=PASS duration="
            << fresh.getTraj().getTotalDuration() << " required_speed="
            << required_speed << std::endl;

  const auto valid_suffix = makeSegment(p0, old_endpoint, 2.0);
  assert(manager.canReuseRemainingSuffix(
      p0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), old_endpoint,
      Eigen::Vector3d::Zero(), valid_suffix, reason));
  std::cout << "SAME_ENDPOINT_SUFFIX_REUSE=PASS" << std::endl;

  poly_traj::MinJerkOpt mission_end;
  assert(manager.buildFreshMovingInitializer(
      p0, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), old_endpoint,
      Eigen::Vector3d(0.2, 0.0, 0.0), mission_end, source, required_speed,
      required_acc, required_jerk, true));
  assert(mission_end.getTraj().getVel(mission_end.getTraj().getTotalDuration()).norm() < 1e-6);
  std::cout << "MISSION_END_ZERO_TERMINAL_VELOCITY=PASS" << std::endl;
  return 0;
}
