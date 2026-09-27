#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>

namespace ego_planner {

// Closed segment versus a closed axis-aligned map voxel.  Touching an
// inflated occupied voxel is conservatively treated as a collision.
inline bool segmentIntersectsVoxel(const Eigen::Vector3d &a,
                                   const Eigen::Vector3d &b,
                                   const Eigen::Vector3d &center,
                                   const double resolution) {
  if (!a.allFinite() || !b.allFinite() || !center.allFinite() ||
      !std::isfinite(resolution) || resolution <= 0.0)
    return true;
  double enter = 0.0, exit = 1.0;
  const Eigen::Vector3d delta = b - a;
  const double half = 0.5 * resolution;
  for (int axis = 0; axis < 3; ++axis) {
    const double lo = center(axis) - half;
    const double hi = center(axis) + half;
    if (std::abs(delta(axis)) <= 1.0e-12) {
      if (a(axis) < lo || a(axis) > hi) return false;
      continue;
    }
    double t0 = (lo - a(axis)) / delta(axis);
    double t1 = (hi - a(axis)) / delta(axis);
    if (t0 > t1) std::swap(t0, t1);
    enter = std::max(enter, t0);
    exit = std::min(exit, t1);
    if (enter > exit) return false;
  }
  return true;
}

template <class Map>
inline bool voxelSegmentFree(Map &map, const Eigen::Vector3d &a,
                             const Eigen::Vector3d &b,
                             const double resolution) {
  if (!a.allFinite() || !b.allFinite() || !std::isfinite(resolution) ||
      resolution <= 0.0 || !map.isInMap(a) || !map.isInMap(b)) return false;
  Eigen::Vector3i min_index, max_index;
  const Eigen::Vector3d padding = Eigen::Vector3d::Constant(resolution);
  const Eigen::Vector3d lo = a.cwiseMin(b) - padding;
  const Eigen::Vector3d hi = a.cwiseMax(b) + padding;
  if (!map.isInMap(lo) || !map.isInMap(hi)) return false;
  map.posToIndex(lo, min_index);
  map.posToIndex(hi, max_index);
  for (int ix = min_index.x(); ix <= max_index.x(); ++ix)
    for (int iy = min_index.y(); iy <= max_index.y(); ++iy)
      for (int iz = min_index.z(); iz <= max_index.z(); ++iz) {
        const Eigen::Vector3i index(ix, iy, iz);
        Eigen::Vector3d center;
        map.indexToPos(index, center);
        if (segmentIntersectsVoxel(a, b, center, resolution) &&
            map.getInflateOccupancy(center) != 0) return false;
      }
  return true;
}

// An open ball around the common guide point is inside both convex prisms
// when every normalized half-space has at least this much signed clearance.
template <class PlaneContainer>
inline bool corridorsOverlapAtJunction(const PlaneContainer &planes,
                                        const int first_piece,
                                        const int second_piece,
                                        const Eigen::Vector3d &junction,
                                        const double required_radius) {
  if (!junction.allFinite() || !std::isfinite(required_radius) ||
      required_radius <= 0.0) return false;
  bool first_seen = false, second_seen = false;
  for (const auto &plane : planes) {
    if (plane.source_segment != first_piece &&
        plane.source_segment != second_piece) continue;
    const double norm = plane.normal.norm();
    if (!plane.normal.allFinite() || !plane.point.allFinite() ||
        !std::isfinite(plane.clearance) || norm <= 1.0e-12 ||
        plane.normal.dot(junction - plane.point) / norm - plane.clearance <
            required_radius) return false;
    first_seen = first_seen || plane.source_segment == first_piece;
    second_seen = second_seen || plane.source_segment == second_piece;
  }
  return first_seen && second_seen;
}

}  // namespace ego_planner
