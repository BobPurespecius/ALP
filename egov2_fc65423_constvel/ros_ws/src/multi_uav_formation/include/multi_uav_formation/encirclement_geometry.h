#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace multi_uav_formation
{
using TeamViewOffsets = std::array<Eigen::Vector3d, 3>;

// Exact interval accounting shared by local/team telemetry and the Team P/T
// acceptance path. It carries no optimization or execution authority.
struct BinaryCameraInterval
{
  explicit BinaryCameraInterval(const int visible_count, const double dt)
      : camera_time(std::max(0, std::min(3, visible_count)) * dt),
        k2_time(visible_count >= 2 ? dt : 0.0),
        all3_time(visible_count >= 3 ? dt : 0.0),
        none_time(visible_count <= 0 ? dt : 0.0)
  {
  }
  double camera_time{0.0};
  double k2_time{0.0};
  double all3_time{0.0};
  double none_time{0.0};
};

inline double peerGeometryConfidence(const double age,
                                     const double half_life)
{
  if (!std::isfinite(age) || !std::isfinite(half_life) || half_life <= 0.0)
    return 0.0;
  return std::exp2(-std::max(0.0, age) / half_life);
}

struct MultiviewPolicy
{
  double bad{25.0*M_PI/180.0}, good{60.0*M_PI/180.0};
  void validate() const {
    if(!std::isfinite(bad+good) || bad<0 || good<=bad || good>M_PI)
      throw std::invalid_argument("Invalid multiview separation limits");
  }
};
template<class Node> inline void loadMultiviewPolicy(Node &n,MultiviewPolicy &p) {
  double bad,good;
  n.param("/encirclement_geometry/multiview_bad_sep_deg",bad,25.0);
  n.param("/encirclement_geometry/multiview_good_sep_deg",good,60.0);
  p.bad=bad*M_PI/180.0;p.good=good*M_PI/180.0;p.validate();
}
struct MultiviewQuality
{
  bool finite{false};
  std::array<double,3> separation{}; // pairs 01,12,20: MINOR angles, not gaps
  double quality{0}, penalty{1};
};
// Saturating, permutation invariant view complementarity. No slots, sorting,
// 120-degree prior or execution eligibility. The optional derivative is dJ_dir/dp.
inline MultiviewQuality multiviewQuality(const std::array<Eigen::Vector3d,3> &q,
    const MultiviewPolicy &p={},std::array<Eigen::Vector3d,3> *gradient=nullptr)
{
  if(gradient)for(auto &g:*gradient)g.setZero();
  MultiviewQuality result;std::array<double,3> theta;
  for(int i=0;i<3;++i) {
    if(!q[i].allFinite() || q[i].head<2>().squaredNorm()<1e-12)return result;
    theta[i]=std::atan2(q[i].y(),q[i].x());
  }
  result.finite=true;result.penalty=0;
  for(int i=0;i<3;++i) {
    const int j=(i+1)%3;
    const double wrapped=std::atan2(std::sin(theta[i]-theta[j]),std::cos(theta[i]-theta[j]));
    const double delta=std::abs(wrapped);result.separation[i]=delta;
    const double u=std::max(0.0,std::min(1.0,(delta-p.bad)/(p.good-p.bad)));
    const double quality=u*u*(3-2*u);
    result.quality+=quality/3;result.penalty+=(1-quality)*(1-quality)/3;
    if(gradient && delta>p.bad && delta<p.good) {
      const double d=-2*(1-quality)*6*u*(1-u)/(3*(p.good-p.bad))*(wrapped>0?1:-1);
      for(int k:{i,j})(*gradient)[k]+=(k==i?d:-d)*
          Eigen::Vector3d(-q[k].y(),q[k].x(),0)/q[k].head<2>().squaredNorm();
    }
  }
  return result;
}
// Geometry only; visibility remains defined by tracking_visibility_geometry.h.
struct CircularGapGeometry
{
  bool finite{false};
  std::array<double, 3> bearings{}, gaps{};
  std::array<int, 3> order{{0, 1, 2}};
  double minimum{0.0}, maximum{2.0*M_PI};
  bool eligible(double minimum_angle, double maximum_gap) const
  {
    return finite && minimum + 1e-10 >= minimum_angle &&
        maximum <= maximum_gap + 1e-10;
  }
};

inline void validateEncirclementLimits(double minimum, double maximum,
                                      double ratio = 0.8)
{
  if (!std::isfinite(minimum) || !std::isfinite(maximum) ||
      !std::isfinite(ratio) || minimum < 0.0 || minimum > 2.0*M_PI/3.0 ||
      maximum < 2.0*M_PI/3.0 || maximum >= M_PI || ratio < 0.0 || ratio > 1.0)
    throw std::invalid_argument("Invalid encirclement min/gap/ratio limits");
}

inline CircularGapGeometry circularGapGeometry(
    const std::array<Eigen::Vector3d, 3> &relative)
{
  CircularGapGeometry result;
  for (int i=0; i<3; ++i)
  {
    if (!relative[i].allFinite() || relative[i].head<2>().squaredNorm()<1e-12)
      return result; // bearing at the target is undefined, never eligible
    result.bearings[i]=std::atan2(relative[i].y(),relative[i].x());
  }
  std::stable_sort(result.order.begin(),result.order.end(),[&](int a,int b) {
    return result.bearings[a]<result.bearings[b];
  });
  for (int j=0;j<3;++j)
    result.gaps[j]=result.bearings[result.order[(j+1)%3]]-
        result.bearings[result.order[j]]+(j==2 ? 2.0*M_PI : 0.0);
  result.minimum=*std::min_element(result.gaps.begin(),result.gaps.end());
  result.maximum=*std::max_element(result.gaps.begin(),result.gaps.end());
  result.finite=true;
  return result;
}

// Piecewise differentiable maximum circular gap. Each gradient evaluation
// fixes the sorted cyclic branch for the QP linearization; every trial is
// sorted again. At equal maxima a deterministic valid subgradient is used.
// There is no bearing/120-degree prior anywhere in this cost.
inline double encirclementGapCost(const std::array<Eigen::Vector3d,3> &relative,
    double maximum_gap, std::array<Eigen::Vector3d,3> *gradient=nullptr)
{
  if (gradient) for (auto &g:*gradient) g.setZero();
  const auto geometry=circularGapGeometry(relative);
  if (!geometry.finite) return std::numeric_limits<double>::infinity();
  const double error=std::max(0.0,geometry.maximum-maximum_gap);
  if (gradient && error>0.0)
  {
    const int gap=std::max_element(geometry.gaps.begin(),geometry.gaps.end())-geometry.gaps.begin();
    const int a=geometry.order[gap], b=geometry.order[(gap+1)%3];
    for (int i:{a,b})
      (*gradient)[i]=(i==a ? -2.0 : 2.0)*error*
          Eigen::Vector3d(-relative[i].y(),relative[i].x(),0.0)/relative[i].head<2>().squaredNorm();
  }
  return error*error;
}

// Both bounds use the same sorted circular gaps, with no equiangular reward.
inline double encirclementGeometryCost(const std::array<Eigen::Vector3d,3> &relative,
    double minimum_angle, double maximum_gap,
    std::array<Eigen::Vector3d,3> *gradient=nullptr)
{
  if (gradient) for(auto &g:*gradient) g.setZero();
  const auto geometry=circularGapGeometry(relative);
  if (!geometry.finite) return std::numeric_limits<double>::infinity();
  double cost=0.0;
  for(int j=0;j<3;++j) {
    const double hi=std::max(0.0,geometry.gaps[j]-maximum_gap);
    const double lo=std::max(0.0,minimum_angle-geometry.gaps[j]);
    cost+=hi*hi+lo*lo;
    const int a=geometry.order[j],b=geometry.order[(j+1)%3];
    if(gradient) for(int i:{a,b}) (*gradient)[i]+=(i==a?-2.0:2.0)*(hi-lo)*
        Eigen::Vector3d(-relative[i].y(),relative[i].x(),0.0)/relative[i].head<2>().squaredNorm();
  }
  return cost;
}

inline double gapQuantile(std::vector<double> values, double fraction)
{
  if (values.empty()) return std::numeric_limits<double>::infinity();
  std::sort(values.begin(),values.end());
  const double index=fraction*(values.size()-1);
  const size_t lo=static_cast<size_t>(index), hi=std::min(lo+1,values.size()-1);
  return values[lo]+(index-lo)*(values[hi]-values[lo]);
}
} // namespace multi_uav_formation
