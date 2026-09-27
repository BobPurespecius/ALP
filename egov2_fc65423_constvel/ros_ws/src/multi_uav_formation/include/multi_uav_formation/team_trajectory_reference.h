#pragma once
#include <optimizer/poly_traj_utils.hpp>
#include <array>
#include <cstdint>
#include <limits>

namespace multi_uav_formation {
// A committed joint payload is a timed soft reference, never execution authority.
// Its identity/epoch survive local generations. No timestamp renewal or slot state.
struct TeamTrajectoryReference {
  std::uint64_t id{0};
  double epoch{0}, end{0};
  std::array<poly_traj::Trajectory,3> trajectories;
  std::array<std::vector<double>,3> yaw_times,yaws;
  bool usable(double world) const {
    return id && std::isfinite(world) && world>=epoch && world<end;
  }
  template<class Message> bool accept(const Message &m) {
    if(!m.valid || m.state!=Message::STATE_COMMIT || !m.team_solution_id ||
       m.team_solution_id<=id || m.drone_ids.size()!=3 || m.trajectories.size()!=3)return false;
    TeamTrajectoryReference next;next.id=m.team_solution_id;next.epoch=m.activation_time.toSec();
    next.end=std::numeric_limits<double>::infinity();std::array<bool,3> seen{};
    if(!std::isfinite(next.epoch))return false;
    for(size_t k=0;k<3;++k) {
      const int i=m.drone_ids[k];const auto &p=m.trajectories[k];
      if(i<0 || i>=3 || seen[i] || p.drone_id!=i || p.order!=5 ||
         std::abs(p.start_time.toSec()-next.epoch)>1e-6 || p.duration.empty() ||
         p.inner_x.size()+1!=p.duration.size() || p.inner_y.size()!=p.inner_x.size() ||
         p.inner_z.size()!=p.inner_x.size())return false;
      seen[i]=true;const int n=p.duration.size();Eigen::Matrix3d h,t;
      for(int a=0;a<3;++a) {h.row(a)<<p.start_p[a],p.start_v[a],p.start_a[a];t.row(a)<<p.end_p[a],p.end_v[a],p.end_a[a];}
      Eigen::MatrixXd inner(3,n-1);Eigen::VectorXd durations(n);
      for(int j=0;j<n;++j) {durations[j]=p.duration[j];if(!std::isfinite(durations[j]) || durations[j]<=0)return false;}
      for(int j=0;j<n-1;++j)inner.col(j)<<p.inner_x[j],p.inner_y[j],p.inner_z[j];
      if(!h.allFinite() || !t.allFinite() || !inner.allFinite())return false;
      poly_traj::MinJerkOpt opt;opt.reset(h,t,n);opt.generate(inner,durations);next.trajectories[i]=opt.getTraj();
      next.end=std::min(next.end,next.epoch+durations.sum());
    }
    if(m.optimized_yaw_valid) {
      next.yaw_times={{m.yaw_sample_times_uav0,m.yaw_sample_times_uav1,m.yaw_sample_times_uav2}};
      next.yaws={{m.yaw_samples_uav0,m.yaw_samples_uav1,m.yaw_samples_uav2}};
      for(int i=0;i<3;++i) {
        const auto &ts=next.yaw_times[i];const auto &ys=next.yaws[i];
        if(ts.size()<2 || ts.size()!=ys.size())return false;
        for(size_t j=0;j<ts.size();++j)if(!std::isfinite(ts[j]+ys[j]) || (j && ts[j]<=ts[j-1]))return false;
      }
    }
    *this=next;return true;
  }
  // New MINCO head is the true active/scheduled predecessor at world, not the old head.
  bool seed(int i,double world,const Eigen::Matrix3d &head,poly_traj::MinJerkOpt &out,int pieces=0) const {
    if(i<0 || i>=3 || !usable(world) || end-world<0.3)return false;
    const auto &tr = trajectories[i];
    const double start = world - epoch, finish = end - epoch;
    // Preserve original junction times. Uniformly resampling only positions
    // changes even an exactly followed MINCO reference and can introduce new
    // acceleration/jerk before the rolling optimization has done any work.
    std::vector<double> knots{start};
    double junction = 0.0;
    for (int j = 0; j < tr.getPieceNum(); ++j) {
      junction += tr[j].getDuration();
      if (junction > start + 1e-6 && junction < finish - 1e-6)
        knots.push_back(junction);
    }
    knots.push_back(finish);
    const int required = static_cast<int>(knots.size()) - 1;
    // Joint warm starts have a fixed decision dimension. If it cannot retain
    // the old knots, decline this optional seed and use the fresh local seed.
    if (pieces > 0 && pieces < required) return false;
    const int count = pieces > 0 ? pieces : std::max(2, required);
    while (static_cast<int>(knots.size()) - 1 < count) {
      size_t longest = 0;
      for (size_t j = 1; j + 1 < knots.size(); ++j)
        if (knots[j+1] - knots[j] > knots[longest+1] - knots[longest]) longest = j;
      knots.insert(knots.begin() + longest + 1,
                   0.5 * (knots[longest] + knots[longest+1]));
    }
    Eigen::Matrix3d tail;
    tail << tr.getPos(finish), tr.getVel(finish), tr.getAcc(finish);
    Eigen::MatrixXd inner(3,count-1);
    Eigen::VectorXd durations(count);
    for (int j = 0; j < count; ++j) {
      durations[j] = knots[j+1] - knots[j];
      if (j + 1 < count) inner.col(j) = tr.getPos(knots[j+1]);
    }
    out.reset(head,tail,count);
    out.generate(inner,durations);
    return true;
  }
  double cost(const Eigen::Vector3d &p,int i,double world,Eigen::Vector3d &g,double &gt) const {
    g.setZero();gt=0;if(i<0 || i>=3 || !usable(world))return 0;
    const auto &tr=trajectories[i];const double t=world-epoch;
    const Eigen::Vector3d e=p-tr.getPos(t);g=2*e;gt=-g.dot(tr.getVel(t));return e.squaredNorm();
  }
  double yaw(int i,double world) const {
    const auto &ts=yaw_times[i];const auto &ys=yaws[i];const double t=world-epoch;
    if(ts.empty())return 0;if(t<=ts.front())return ys.front();if(t>=ts.back())return ys.back();
    const size_t j=std::upper_bound(ts.begin(),ts.end(),t)-ts.begin();
    const double d=std::atan2(std::sin(ys[j]-ys[j-1]),std::cos(ys[j]-ys[j-1]));
    return ys[j-1]+d*(t-ts[j-1])/(ts[j]-ts[j-1]);
  }
};
// The bundle's selected candidate is rebound to the post-check committed local
// polynomial before publication. It and pending.active_generation form one
// snapshot; a separate ROS execution callback can arrive one callback later.
// Newer execution feedback invalidates that snapshot; older feedback cannot
// replace its head. No waiting for the callback queue and no new protocol.
template<class Bundle> auto committedPredecessorCandidate(const Bundle &bundle,double latest_start)
    -> decltype(&bundle.candidates.front()) {
  for(const auto &candidate:bundle.candidates) {
    if(candidate.candidate_id!=bundle.local_selected_candidate_id)continue;
    const double start=candidate.trajectory_start_time.toSec();
    if(!candidate.success || !candidate.static_valid || !candidate.dynamics_valid || !candidate.swarm_valid ||
       candidate.safety_class!=candidate.SAFETY_ABSOLUTE_SAFE ||
       std::abs(start-bundle.evaluation_start_time.toSec())>1e-6 || latest_start>start+1e-6)return nullptr;
    return &candidate;
  }
  return nullptr;
}
} // namespace multi_uav_formation
