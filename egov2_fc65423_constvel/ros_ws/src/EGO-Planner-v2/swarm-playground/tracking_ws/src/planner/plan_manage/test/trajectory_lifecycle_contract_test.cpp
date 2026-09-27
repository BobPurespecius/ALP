#include <traj_utils/trajectory_lifecycle.h>
#include <optimizer/poly_traj_utils.hpp>
#include <iostream>
#include <stdexcept>
using namespace trajectory_lifecycle;
void require(bool ok,const char *name) { if(!ok) throw std::runtime_error(name); std::cout<<name<<"=PASS\n"; }
int main() {
  require(localReplanAllowed(true),"PENDING_LOCAL_REPLAN_CONTINUES");
  require(!canStage(0) && canStage(1),"ZERO_EXECUTABLE_DOES_NOT_ENTER_PENDING");
  poly_traj::CoefficientMat cm=poly_traj::CoefficientMat::Zero();
  cm(0,4)=1.4;cm(0,3)=0.2;cm(1,2)=0.03;
  poly_traj::Trajectory old({3.0},{cm});
  const double future=0.37;
  const State head=sample(old,future);
  Eigen::Matrix3d h,t;h<<head.p,head.v,head.a;
  t<<Eigen::Vector3d(4,1,0),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero();
  poly_traj::MinJerkOpt next;next.reset(h,t,2);
  Eigen::MatrixXd inner(3,1);inner<<2.0,0.4,0.0;
  next.generate(inner,Eigen::Vector2d(1.5,1.5));
  Tolerances tol;
  const auto error=compare(head,sample(next.getTraj(),0.0));
  require(error.dp<1e-9 && error.dv<1e-9 && error.da<1e-9,"FUTURE_ACTIVATION_PVA_ALIGNMENT");
  State bad=head;bad.v.x()+=0.06;
  require(!compare(head,bad).accepted(tol),"VELOCITY_MISMATCH_REJECTED");
  bad=head;bad.a.z()+=0.11;
  require(!compare(head,bad).accepted(tol),"ACCELERATION_MISMATCH_REJECTED");
  require(!compare(sample(old,0.0),head).accepted(tol),"PLANNING_TIME_HEAD_REJECTED");
  require(successorDue(0.6,0.08,0.10,0.65) && !successorDue(2.0,0.08,0.10,0.65),"PERSISTENCE_EXPIRY_ADVANCE_RESERVE");
  const auto end=sample(old,4.0);
  require((end.p-old.getPos(3.0)).norm()<1e-12 && end.v.norm()==0 && end.a.norm()==0,"PEER_EXPIRY_MATCHES_TERMINAL_HOLD");
  require(currentGeneration(7,7,11,11) && !currentGeneration(7,7,11,12) && !currentGeneration(6,7,11,11),"STALE_TEAM_CANNOT_OVERRIDE_LOCAL");
  const auto traj=next.getTraj();
  require((traj[0].getPos(1.5)-traj[1].getPos(0)).norm()<1e-8 &&
      (traj[0].getVel(1.5)-traj[1].getVel(0)).norm()<1e-8 &&
      (traj[0].getAcc(1.5)-traj[1].getAcc(0)).norm()<1e-8,"REANCHORED_MINCO_C2");
}
