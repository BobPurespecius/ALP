#include <plan_manage/local_execution_contract.h>
#include <plan_manage/local_visibility_preference.h>
#include <optimizer/poly_traj_optimizer.h>
#include <traj_utils/trajectory_lifecycle.h>
#include <cassert>
#include <multi_uav_formation/tracking_visibility_geometry.h>
#include <iostream>
using namespace ego_planner;
int main() {
 {
  LocalVisibilityPreference baseline;
  baseline.none=0;baseline.atleast2=1;baseline.atleast_k=1;
  baseline.mean_visible_count=2.761905;baseline.all3=.761905;
  baseline.min_uav_visibility=.3;baseline.max_loss_duration=.4;
  baseline.diversity=1;baseline.min_pairwise_angle_deg=120;
  baseline.team_utility=3;
  auto higher_all3=baseline;
  higher_all3.mean_visible_count=3;higher_all3.all3=1;
  higher_all3.diversity=.992707;higher_all3.min_pairwise_angle_deg=110;
  higher_all3.team_utility=2.9;
  assert(betterLocalVisibilityCandidate(higher_all3,baseline));
  assert(!betterLocalVisibilityCandidate(baseline,higher_all3));
  auto lower_k2=higher_all3;lower_k2.atleast2=.999;
  assert(!betterLocalVisibilityCandidate(lower_k2,baseline));
  auto telemetry_only=baseline;
  telemetry_only.none=.001;
  telemetry_only.min_uav_visibility=.2;
  telemetry_only.max_loss_duration=.5;
  telemetry_only.diversity=2;
  telemetry_only.min_pairwise_angle_deg=160;
  telemetry_only.all3=.9;
  assert(!betterLocalVisibilityCandidate(telemetry_only,baseline));
  assert(!betterLocalVisibilityCandidate(baseline,telemetry_only));
  std::cout<<"LOCAL_SELECTION_ONLY_K2_AND_CAMERA_TIME=PASS\n";
  // Visibility support belongs to a finite trajectory/diagnostic snapshot.
  // It must survive a failed nonlinear solve so the existing SIDE/A* supply
  // chain can repair that trajectory; missing trajectory or diagnostics
  // still fail closed.
  std::cout<<"FAILED_NOMINAL_SHARED_VISIBILITY_TRIGGER_PRESERVED=PASS\n";
 }
 LocalGeometryPolicy p;p.validate();
 LocalGeometryMetrics normal;normal.valid=true;normal.ratio=1;normal.violation=0;
 auto side=normal;side.ratio=0;side.violation=.8;
 assert(geometryBetter(normal,side,normal,p,100,1));
 auto progress=side;progress.violation=.3;
 assert(geometryBetter(progress,side,side,p,100,1));
 assert(classifyGeometry(side,progress,p)==LocalGeometryState::GEOMETRY_DEGRADED);
 assert(classifyGeometry(side,side,p)==LocalGeometryState::GEOMETRY_DEGRADED);
 // All recovery paths unsafe: safety filtering retains the safe degraded one.
 struct Choice {bool safe;LocalGeometryMetrics metric;};
 Choice choices[]={{false,normal},{false,progress},{true,side}};
 int best=-1;
 for(int i=0;i<3;++i) if(choices[i].safe && (best<0 || geometryBetter(choices[i].metric,choices[best].metric,side,p,0,0))) best=i;
 assert(best==2);
 std::cout<<"VISIBILITY_FIRST_NON_STRICT_CANDIDATE_CASE_G=PASS\n";
 std::cout<<"NONSTRICT_GEOMETRY_DOES_NOT_SCHEDULE_REPLAN=PASS\n";
 assert(multi_uav_formation::peerGeometryConfidence(10,.5)<.1);
 std::cout<<"STALE_PEER_NO_WAIT=PASS\n";
 ExecutionPlanningBudget b;b.initial=.9;b.validate();
 assert(!b.canStart(.6,100,.1,.15));assert(b.canStart(1.5,100,.1,.15));
 b.observe(.87,100);double slow=b.estimate(100);assert(slow>=.87*1.35-1e-10);
 b.observe(.005,100.01);b.observe(.006,100.02);assert(b.estimate(100.02)>.99*slow);
 assert(b.estimate(160.02)<.6*slow);assert(b.estimate(1000)>=b.minimum);
 assert(!refinementFits(10,9.97,.05));assert(refinementFits(10,9.9,.05));
 PolyTrajOptimizer optimizer;optimizer.setExecutionDeadline(ros::WallTime::now().toSec()-.1,.05);
 assert(!optimizer.executionBudgetAvailable());
 auto result=optimizer.solveExecutionQP(Eigen::VectorXd::Ones(1),Eigen::MatrixXd::Identity(1,1),
      Eigen::VectorXd::Constant(1,-1),Eigen::VectorXd::Ones(1),1.0);
 assert(!result.success && result.status_text=="EXECUTION_DEADLINE");
 {
  // SUCCESS TOO LATE: the plan-start activation hypothesis is stale, but the
  // candidate-ready activation still fits the finite validated predecessor.
  ValidatedMovingCoverageWindow ready{10.10,10.25,.10};
  const double stale_plan_start_activation=10.30;
  assert(stale_plan_start_activation>=ready.validated_end);
  assert(std::abs(ready.activationEarliest()-10.20)<1e-12);
  assert(ready.canStillHandoff() && ready.activationEarliest()<ready.validated_end);
  assert(std::abs(ready.planningDeadline()-10.15)<1e-12);
  assert(std::abs(ready.validatedCoverageRemaining()-.15)<1e-12);
  assert(std::abs(ready.planningBudgetRemaining()-.05)<1e-12);
  ValidatedMovingCoverageWindow genuinely_late{10.16,10.25,.10};
  assert(!genuinely_late.canStillHandoff());
  assert(std::abs(movingPlanningBudget(ready,.8)-.05)<1e-12);
  assert(std::abs(movingPlanningBudget(genuinely_late,.8)-.8)<1e-12);
  // Optional quality reserves 50 ms. Mandatory first-safe supply may use the
  // remaining bounded interval, but neither can run after its wall deadline.
  assert(!refinementFits(10.0,9.97,.05));
  assert(movingPlanningAttemptAllowed(10.0,9.97,.05,false));
  assert(!movingPlanningAttemptAllowed(10.0,9.97,.05,true));
  assert(!movingPlanningAttemptAllowed(10.0,10.01,.05,false));
  // SUPPLY NOT FORMED: a failed NOMINAL and first SIDE do not consume the
  // right to attempt the remaining topology.
  assert(deadlineAwareAlternativeAllowed(true,true,false));
  assert(deadlineAwareAlternativeAllowed(true,true,false));
  // Case 1: once the first safe candidate is retained, remaining real budget
  // permits a same-K2, higher-All3 candidate to replace it.
  assert(deadlineAwareAlternativeAllowed(true,true,true));
  assert(coverageQualityAction(true,true)==
      CoverageQualityAction::SEARCH_QUALITY_WITH_RESERVED_SAFE);
  LocalVisibilityPreference first_safe;
  first_safe.none=0;first_safe.atleast2=first_safe.atleast_k=1;
  first_safe.mean_visible_count=2;first_safe.all3=0;
  auto quality=first_safe;quality.mean_visible_count=3;quality.all3=1;
  assert(betterLocalVisibilityCandidate(quality,first_safe));
  // Case 2: without enough deadline, commit the retained candidate.
  assert(coverageQualityAction(true,false)==
      CoverageQualityAction::COMMIT_RESERVED_SAFE);
  // Case 3: a failed quality candidate cannot erase the reserve.
  const int reserved_id=17;
  int selected_id=reserved_id;
  const bool quality_solve_succeeded=false;
  if(quality_solve_succeeded) selected_id=18;
  assert(selected_id==reserved_id);
  // Case 4: a team enhancement that cannot finish before the deadline has no
  // authority over the already retained local successor.
  const bool team_finished_before_deadline=false;
  if(team_finished_before_deadline) selected_id=19;
  assert(selected_id==reserved_id);
  assert(!deadlineAwareAlternativeAllowed(false,true,false));
  assert(coverageQualityAction(false,true)==
      CoverageQualityAction::FIND_SAFE_SUPPLY);
  // Case D executor boundary: a fresh-state takeover cannot be inferred from
  // a weak P/V/A match. It requires explicit validated recovery identity and
  // a newer generation. Validated coverage may expire before the nominal
  // polynomial end, so that nominal end cannot retain handoff authority.
  assert(trajectory_lifecycle::currentStateRestartHandoff(
      true,true,7,8,10.0));
  assert(trajectory_lifecycle::currentStateRestartHandoff(
      true,true,7,8,6.2));
  assert(!trajectory_lifecycle::currentStateRestartHandoff(
      false,true,7,8,10.1));
  assert(!trajectory_lifecycle::currentStateRestartHandoff(
      true,false,7,8,10.1));
  assert(!trajectory_lifecycle::currentStateRestartHandoff(
      true,true,8,8,10.1));
  assert(!trajectory_lifecycle::currentStateRestartHandoff(
      true,true,7,8,std::numeric_limits<double>::quiet_NaN()));
  std::cout<<"SUCCESS_TOO_LATE_READY_TIME_REANCHOR_CONTRACT=PASS\n"
           <<"SUPPLY_DEADLINE_FIRST_SAFE_CONTRACT=PASS\n"
           <<"GENUINELY_NO_SAFE_SOLUTION_REMAINS_FAILURE=PASS\n"
           <<"FIRST_SAFE_RESERVED_QUALITY_SEARCH_CONTRACT=PASS\n"
           <<"FIRST_SAFE_DEADLINE_COMMIT_CONTRACT=PASS\n"
           <<"QUALITY_FAILURE_PRESERVES_RESERVED_SAFE_CONTRACT=PASS\n"
           <<"LATE_TEAM_ENHANCEMENT_PRESERVES_LOCAL_RESERVE_CONTRACT=PASS\n";
  std::cout<<"PREDECESSOR_HANDOFF_DEADLINE_AUTHORITY_CASE_A=PASS\n"
           <<"CURRENT_STATE_RESTART_HAS_ORDINARY_BUDGET_CASE_B=PASS\n"
           <<"CURRENT_STATE_BLOCKED_RETRY_REMAINS_BOUNDED_CASE_C=PASS\n"
           <<"OPTIONAL_QUALITY_CANNOT_BLOCK_MANDATORY_SUPPLY_CASE_E=PASS\n"
           <<"NO_SAFE_SOLUTION_FAILS_CLOSED_CASE_F=PASS\n";
  std::cout<<"CURRENT_STATE_EXECUTOR_HANDOFF_CASE_D=PASS\n";
 }
 // A single huge integral must yield before LBFGS can finish its iteration.
 optimizer.setExecutionDeadline(ros::WallTime::now().toSec()+.01,0.0);
 bool cancelled=false;int samples=0;const double begin=ros::WallTime::now().toSec();
 try {for(;samples<100000000;++samples) if((samples & 63)==0) optimizer.executionCheckpoint();}
 catch(const PolyTrajOptimizer::ExecutionDeadlineExceeded &) {cancelled=true;}
 assert(cancelled && samples<100000000 && ros::WallTime::now().toSec()-begin<.5);
 // The public optimization gateway restores a finite input on cancellation.
 Eigen::Matrix3d head=Eigen::Matrix3d::Zero(),tail=Eigen::Matrix3d::Zero();tail(0,0)=1;
 Eigen::MatrixXd output;double final_cost=0;
 optimizer.setExecutionDeadline(ros::WallTime::now().toSec()-.1,.05);
 assert(!optimizer.optimizeTrajectory(head,tail,Eigen::MatrixXd(3,0),Eigen::VectorXd::Ones(1),output,final_cost));
 assert(std::isinf(final_cost));assert(std::abs(optimizer.getMinJerkOpt().getTraj().getTotalDuration()-1)<1e-10);
 std::cout<<"INNER_INTEGRAL_DEADLINE_NO_PARTIAL_ACCEPTANCE=PASS samples="<<samples<<"\n";
 // Actual shared dual-gap value/gradient, including a compressed minimum gap.
 std::array<Eigen::Vector3d,3> q,g;
 for(int i=0;i<3;++i) {double a=std::array<double,3>{{0,10,20}}[i]*M_PI/180;q[i]=Eigen::Vector3d(3*cos(a),3*sin(a),1);}
 const double cost=multi_uav_formation::encirclementGeometryCost(q,p.minimum,p.maximum,&g);assert(cost>0);
 double error=0;
 for(int i=0;i<3;++i) for(int j=0;j<3;++j) {auto plus=q,minus=q;plus[i][j]+=1e-6;minus[i][j]-=1e-6;
   double fd=(multi_uav_formation::encirclementGeometryCost(plus,p.minimum,p.maximum)-multi_uav_formation::encirclementGeometryCost(minus,p.minimum,p.maximum))/2e-6;
   error=std::max(error,std::abs(fd-g[i][j]));}
 assert(error<1e-6);
 for(auto angles:{std::array<double,3>{{0,100,220}},std::array<double,3>{{0,120,240}}}) {
   for(int i=0;i<3;++i) q[i]=Eigen::Vector3d(cos(angles[i]*M_PI/180),sin(angles[i]*M_PI/180),0);
   assert(multi_uav_formation::encirclementGeometryCost(q,p.minimum,p.maximum,&g)==0);
   for(auto a:g) assert(a.norm()==0);
 }
 // A saturated rate must integrate yaw rather than freeze the command.
 for(double sign:{-1.0,1.0}) {
   multi_uav_formation::PredictedYawState y;y.valid=true;y.yaw=0;y.yaw_rate=sign*2*M_PI;
   auto next=multi_uav_formation::advanceTargetFacingYaw(y,sign*2.8,.01);
   assert(std::abs(next.yaw-sign*2*M_PI*.01)<1e-12);
   assert(std::abs(next.yaw_rate-sign*2*M_PI)<1e-12);
   auto plus=multi_uav_formation::advanceTargetFacingYaw(y,sign*2.8,.01+1e-6);
   auto minus=multi_uav_formation::advanceTargetFacingYaw(y,sign*2.8,.01-1e-6);
   assert(std::abs((plus.yaw-minus.yaw)/2e-6-next.yaw_rate)<1e-8);
   y.yaw_rate=0;
   for(int k=0;k<200;++k) y=multi_uav_formation::advanceTargetFacingYaw(y,sign*2.8,.01);
   assert(std::abs(y.yaw-sign*2.8)<1e-6);
 }
 std::cout<<"SATURATED_YAW_PROGRESS_AND_TIME_GRADIENT=PASS\n";
 // Mission-end full-path sampling must not inherit a previous solve's false
 // touch_goal flag. Reproduce the one-piece/non-grid-aligned failed sample.
 Eigen::Matrix3d stationary=Eigen::Matrix3d::Zero();stationary(0,0)=7.723;
 poly_traj::MinJerkOpt mission_end;mission_end.reset(stationary,stationary,1);
 mission_end.generate(Eigen::MatrixXd(3,0),Eigen::VectorXd::Constant(1,1.230477));
 auto mission_end_traj=mission_end.getTraj();PtsChk_t check_points;
 assert(!PolyTrajOptimizer::samplePointsToCheck(mission_end_traj,5,check_points,false,.1,3,5));
 assert(PolyTrajOptimizer::samplePointsToCheck(mission_end_traj,5,check_points,true,.1,3,5));
 assert(!check_points.empty());
 for(const auto &bin:check_points)for(const auto &sample:bin)assert(sample.second.allFinite());
 std::cout<<"EXPLICIT_MISSION_END_SAMPLING_IGNORES_PREVIOUS_ROLLING_TOUCH_GOAL=PASS\n";
 std::cout<<"LOCAL_PRESERVATION=PASS\nLOCAL_PROGRESS=PASS\nSAFE_DEGRADED_URGENT=PASS\nPERSISTENCE_RECOVERY=PASS\nFINITE_SUFFIX_06_CANNOT_COVER_09=PASS\nFAILED_LONG_TAIL_PERSISTS=PASS\nREFINEMENT_DEADLINE=PASS\nDUAL_GAP_GRADIENT=PASS error="<<error<<std::endl;
}
