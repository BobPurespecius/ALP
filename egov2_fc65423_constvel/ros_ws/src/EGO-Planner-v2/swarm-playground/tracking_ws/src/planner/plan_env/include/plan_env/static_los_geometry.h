#ifndef PLAN_ENV_STATIC_LOS_GEOMETRY_H
#define PLAN_ENV_STATIC_LOS_GEOMETRY_H

#include <Eigen/Core>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace ego_planner
{

struct StaticLosCylinder
{
  std::string name;
  Eigen::Vector2d center{Eigen::Vector2d::Zero()};
  double radius{0.0};
  double z_min{0.0};
  double z_max{0.0};
};

struct StaticLosWall
{
  std::string name;
  Eigen::Vector2d center{Eigen::Vector2d::Zero()};
  Eigen::Vector2d half_size{Eigen::Vector2d::Zero()};
  double yaw{0.0};
  double z_min{0.0};
  double z_max{0.0};
};

// Witness returned by the authoritative static LOS query.  This is kept
// separate from the planner's dynamic-object risk record: a target point or a
// primitive array index must never be used as a substitute for the blocker
// identity that generated the LOS event.
struct StaticLosWitness
{
  bool valid{false};
  int primitive_index{-1};
  int primitive_type{0}; // 1=cylinder, 2=wall
  std::string primitive_name;
  double ray_parameter{0.0};
  double clearance{std::numeric_limits<double>::infinity()};
  Eigen::Vector3d hit_point{Eigen::Vector3d::Zero()};
  Eigen::Vector3d horizontal_normal{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d primitive_center{Eigen::Vector3d::Zero()};
  double effective_radius{0.0};
};

// Authoritative static-LOS geometry for the scene-driven simulator.  The
// global point cloud samples solid primitives, while visibility is defined by
// the same cylinders and yaw-oriented wall prisms in the scene JSON.  Both
// diagnostic LOS evaluation and the MINCO soft objective use this class.
class StaticLosGeometry
{
public:
  bool loadScene(const std::string &scene_file, std::string &reason)
  {
    cylinders_.clear();
    walls_.clear();
    scene_file_.clear();
    if (scene_file.empty())
    {
      reason = "EMPTY_SCENE_FILE";
      return false;
    }

    try
    {
      boost::property_tree::ptree root;
      boost::property_tree::read_json(scene_file, root);
      const double default_height =
          root.get<double>("egoPlanner.obstacleHeight", 3.0);
      const auto obstacles = root.get_child_optional("obstacleData");
      if (!obstacles)
      {
        reason = "MISSING_OBSTACLE_DATA";
        return false;
      }

      for (const auto &entry : *obstacles)
      {
        const boost::property_tree::ptree &node = entry.second;
        const auto center_node = node.get_child_optional("centerENU");
        if (!center_node)
          continue;
        std::vector<double> center_values;
        for (const auto &value : *center_node)
          center_values.push_back(value.second.get_value<double>());
        if (center_values.size() < 2)
          continue;

        StaticLosCylinder cylinder;
        cylinder.name = entry.first;
        cylinder.center = Eigen::Vector2d(center_values[0], center_values[1]);
        cylinder.radius = node.get<double>("radius", 0.5);
        cylinder.z_min = node.get<double>("zMin", 0.0);
        cylinder.z_max =
            cylinder.z_min + node.get<double>("height", default_height);
        if (!cylinder.center.allFinite() || !std::isfinite(cylinder.radius) ||
            !std::isfinite(cylinder.z_min) || !std::isfinite(cylinder.z_max) ||
            cylinder.radius <= 0.0 || cylinder.z_max <= cylinder.z_min)
          continue;
        cylinders_.push_back(cylinder);
      }

      const auto walls = root.get_child_optional("wallData");
      if (walls)
      {
        for (const auto &entry : *walls)
        {
          const boost::property_tree::ptree &node = entry.second;
          const auto center_node = node.get_child_optional("centerENU");
          const auto size_node = node.get_child_optional("sizeENU");
          if (!center_node || !size_node)
            continue;
          std::vector<double> center_values;
          std::vector<double> size_values;
          for (const auto &value : *center_node)
            center_values.push_back(value.second.get_value<double>());
          for (const auto &value : *size_node)
            size_values.push_back(value.second.get_value<double>());
          if (center_values.size() < 2 || size_values.size() < 2)
            continue;

          StaticLosWall wall;
          wall.name = entry.first;
          wall.center = Eigen::Vector2d(center_values[0], center_values[1]);
          wall.half_size =
              0.5 * Eigen::Vector2d(size_values[0], size_values[1]);
          wall.yaw = node.get<double>("yawRad", 0.0);
          wall.z_min = node.get<double>("zMin", 0.0);
          const double height = size_values.size() >= 3
                                    ? size_values[2]
                                    : node.get<double>("height", default_height);
          wall.z_max = wall.z_min + height;
          if (!wall.center.allFinite() || !wall.half_size.allFinite() ||
              !std::isfinite(wall.yaw) || !std::isfinite(wall.z_min) ||
              !std::isfinite(wall.z_max) ||
              wall.half_size.minCoeff() <= 0.0 || wall.z_max <= wall.z_min)
            continue;
          walls_.push_back(wall);
        }
      }
    }
    catch (const std::exception &error)
    {
      reason = std::string("JSON_PARSE_ERROR:") + error.what();
      cylinders_.clear();
      walls_.clear();
      return false;
    }

    if (cylinders_.empty() && walls_.empty())
    {
      reason = "NO_VALID_STATIC_PRIMITIVES";
      return false;
    }
    scene_file_ = scene_file;
    reason = "OK";
    return true;
  }

  void setCylinders(const std::vector<StaticLosCylinder> &cylinders)
  {
    cylinders_ = cylinders;
    walls_.clear();
    scene_file_ = "<in-memory>";
  }

  void setWalls(const std::vector<StaticLosWall> &walls)
  {
    cylinders_.clear();
    walls_ = walls;
    scene_file_ = "<in-memory>";
  }

  bool valid() const { return !cylinders_.empty() || !walls_.empty(); }
  size_t size() const { return cylinders_.size() + walls_.size(); }
  size_t cylinderSize() const { return cylinders_.size(); }
  size_t wallSize() const { return walls_.size(); }
  const std::string &sceneFile() const { return scene_file_; }

  bool querySegmentClearance(const Eigen::Vector3d &observer,
                             const Eigen::Vector3d &target,
                             double &clearance,
                             Eigen::Vector3d *gradient_observer = nullptr,
                             Eigen::Vector3d *gradient_target = nullptr,
                             int *cylinder_index = nullptr,
                             StaticLosWitness *witness = nullptr) const
  {
    clearance = std::numeric_limits<double>::infinity();
    if (gradient_observer != nullptr)
      gradient_observer->setZero();
    if (gradient_target != nullptr)
      gradient_target->setZero();
    if (cylinder_index != nullptr)
      *cylinder_index = -1;
    if (witness != nullptr)
      *witness = StaticLosWitness();
    if (!valid() || !observer.allFinite() || !target.allFinite())
      return false;

    const Eigen::Vector3d delta = target - observer;
    bool found = false;
    for (size_t index = 0; index < cylinders_.size(); ++index)
    {
      const StaticLosCylinder &cylinder = cylinders_[index];
      double s_min = 0.0;
      double s_max = 1.0;
      if (std::abs(delta.z()) < 1.0e-9)
      {
        if (observer.z() < cylinder.z_min || observer.z() > cylinder.z_max)
          continue;
      }
      else
      {
        const double first = (cylinder.z_min - observer.z()) / delta.z();
        const double second = (cylinder.z_max - observer.z()) / delta.z();
        s_min = std::max(0.0, std::min(first, second));
        s_max = std::min(1.0, std::max(first, second));
        if (s_min > s_max)
          continue;
      }

      const Eigen::Vector2d start_xy = observer.head<2>();
      const Eigen::Vector2d delta_xy = delta.head<2>();
      const double denominator = delta_xy.squaredNorm();
      double s = s_min;
      if (denominator > 1.0e-12)
      {
        s = -(start_xy - cylinder.center).dot(delta_xy) / denominator;
        s = std::max(s_min, std::min(s_max, s));
      }
      const Eigen::Vector2d closest = start_xy + s * delta_xy;
      const Eigen::Vector2d radial = closest - cylinder.center;
      const double radial_distance = radial.norm();
      const double candidate_clearance = radial_distance - cylinder.radius;
      if (!found || candidate_clearance < clearance)
      {
        found = true;
        clearance = candidate_clearance;
        Eigen::Vector2d normal;
        if (radial_distance > 1.0e-9)
          normal = radial / radial_distance;
        else if ((start_xy - cylinder.center).norm() > 1.0e-9)
          normal = (start_xy - cylinder.center).normalized();
        else if ((target.head<2>() - cylinder.center).norm() > 1.0e-9)
          normal = (target.head<2>() - cylinder.center).normalized();
        else
          normal = Eigen::Vector2d::UnitX();

        if (gradient_observer != nullptr)
        {
          gradient_observer->setZero();
          gradient_observer->head<2>() = (1.0 - s) * normal;
        }
        if (gradient_target != nullptr)
        {
          gradient_target->setZero();
          gradient_target->head<2>() = s * normal;
        }
        if (cylinder_index != nullptr)
          *cylinder_index = static_cast<int>(index);
        if (witness != nullptr)
        {
          witness->valid = true;
          witness->primitive_index = static_cast<int>(index);
          witness->primitive_type = 1;
          witness->primitive_name = cylinder.name;
          witness->ray_parameter = s;
          witness->clearance = candidate_clearance;
          witness->hit_point = Eigen::Vector3d(closest.x(), closest.y(),
                                               observer.z() + s * delta.z());
          witness->horizontal_normal = Eigen::Vector3d(normal.x(), normal.y(), 0.0);
          witness->primitive_center = Eigen::Vector3d(cylinder.center.x(),
                                                      cylinder.center.y(),
                                                      0.5 * (cylinder.z_min + cylinder.z_max));
          witness->effective_radius = cylinder.radius;
        }
      }
    }

    for (size_t index = 0; index < walls_.size(); ++index)
    {
      const StaticLosWall &wall = walls_[index];
      double s_min = 0.0;
      double s_max = 1.0;
      if (std::abs(delta.z()) < 1.0e-9)
      {
        if (observer.z() < wall.z_min || observer.z() > wall.z_max)
          continue;
      }
      else
      {
        const double first = (wall.z_min - observer.z()) / delta.z();
        const double second = (wall.z_max - observer.z()) / delta.z();
        s_min = std::max(0.0, std::min(first, second));
        s_max = std::min(1.0, std::max(first, second));
        if (s_min > s_max)
          continue;
      }

      const double cosine = std::cos(wall.yaw);
      const double sine = std::sin(wall.yaw);
      Eigen::Matrix2d world_from_local;
      world_from_local << cosine, -sine, sine, cosine;
      const Eigen::Vector2d local_start =
          world_from_local.transpose() * (observer.head<2>() - wall.center);
      const Eigen::Vector2d local_delta =
          world_from_local.transpose() * delta.head<2>();

      std::vector<double> breaks{s_min, s_max};
      for (int axis = 0; axis < 2; ++axis)
      {
        if (std::abs(local_delta(axis)) <= 1.0e-12)
          continue;
        for (const double sign : {-1.0, 1.0})
        {
          const double crossing =
              (sign * wall.half_size(axis) - local_start(axis)) /
              local_delta(axis);
          if (crossing > s_min && crossing < s_max)
            breaks.push_back(crossing);
        }
      }
      std::sort(breaks.begin(), breaks.end());
      breaks.erase(std::unique(breaks.begin(), breaks.end(),
                               [](const double first, const double second) {
                                 return std::abs(first - second) < 1.0e-12;
                               }),
                   breaks.end());

      double best_s = s_min;
      double best_squared = std::numeric_limits<double>::infinity();
      const auto evaluate = [&](const double raw_s) {
        const double s = std::max(s_min, std::min(s_max, raw_s));
        const Eigen::Vector2d point = local_start + s * local_delta;
        const Eigen::Vector2d closest = point.cwiseMax(-wall.half_size)
                                              .cwiseMin(wall.half_size);
        const double squared = (point - closest).squaredNorm();
        if (squared < best_squared)
        {
          best_squared = squared;
          best_s = s;
        }
      };
      for (const double value : breaks)
        evaluate(value);
      for (size_t interval = 0; interval + 1 < breaks.size(); ++interval)
      {
        const double lower = breaks[interval];
        const double upper = breaks[interval + 1];
        const double middle = 0.5 * (lower + upper);
        const Eigen::Vector2d point = local_start + middle * local_delta;
        Eigen::Vector2d residual = Eigen::Vector2d::Zero();
        Eigen::Vector2d slope = Eigen::Vector2d::Zero();
        for (int axis = 0; axis < 2; ++axis)
        {
          if (point(axis) < -wall.half_size(axis))
          {
            residual(axis) = local_start(axis) + wall.half_size(axis);
            slope(axis) = local_delta(axis);
          }
          else if (point(axis) > wall.half_size(axis))
          {
            residual(axis) = local_start(axis) - wall.half_size(axis);
            slope(axis) = local_delta(axis);
          }
        }
        const double denominator = slope.squaredNorm();
        if (denominator > 1.0e-15)
          evaluate(std::max(lower, std::min(upper,
                                           -residual.dot(slope) / denominator)));
        else
          evaluate(middle);
      }

      const Eigen::Vector2d local_point = local_start + best_s * local_delta;
      const Eigen::Vector2d local_closest =
          local_point.cwiseMax(-wall.half_size).cwiseMin(wall.half_size);
      const Eigen::Vector2d local_residual = local_point - local_closest;
      const double candidate_clearance = std::sqrt(std::max(0.0, best_squared));
      if (!found || candidate_clearance < clearance)
      {
        found = true;
        clearance = candidate_clearance;
        Eigen::Vector2d local_normal;
        if (candidate_clearance > 1.0e-9)
          local_normal = local_residual / candidate_clearance;
        else
        {
          const double x_face = wall.half_size.x() - std::abs(local_point.x());
          const double y_face = wall.half_size.y() - std::abs(local_point.y());
          if (x_face <= y_face)
            local_normal = Eigen::Vector2d(local_point.x() >= 0.0 ? 1.0 : -1.0, 0.0);
          else
            local_normal = Eigen::Vector2d(0.0, local_point.y() >= 0.0 ? 1.0 : -1.0);
        }
        const Eigen::Vector2d normal = world_from_local * local_normal;
        if (gradient_observer != nullptr)
        {
          gradient_observer->setZero();
          gradient_observer->head<2>() = (1.0 - best_s) * normal;
        }
        if (gradient_target != nullptr)
        {
          gradient_target->setZero();
          gradient_target->head<2>() = best_s * normal;
        }
        if (cylinder_index != nullptr)
          *cylinder_index = static_cast<int>(cylinders_.size() + index);
        if (witness != nullptr)
        {
          witness->valid = true;
          witness->primitive_index = static_cast<int>(cylinders_.size() + index);
          witness->primitive_type = 2;
          witness->primitive_name = wall.name;
          witness->ray_parameter = best_s;
          witness->clearance = candidate_clearance;
          witness->hit_point = Eigen::Vector3d(
              observer.x() + best_s * delta.x(),
              observer.y() + best_s * delta.y(),
              observer.z() + best_s * delta.z());
          witness->horizontal_normal = Eigen::Vector3d(normal.x(), normal.y(), 0.0);
          witness->primitive_center = Eigen::Vector3d(wall.center.x(), wall.center.y(),
                                                      0.5 * (wall.z_min + wall.z_max));
          witness->effective_radius = wall.half_size.norm();
        }
      }
    }
    return found;
  }

  bool querySoftPenalty(const Eigen::Vector3d &observer,
                        const Eigen::Vector3d &target,
                        const double margin,
                        const double lambda,
                        double &weighted_cost,
                        double &raw_cost,
                        double &clearance,
                        Eigen::Vector3d &gradient_observer,
                        Eigen::Vector3d &gradient_target) const
  {
    weighted_cost = 0.0;
    raw_cost = 0.0;
    gradient_observer.setZero();
    gradient_target.setZero();
    Eigen::Vector3d clearance_gradient_observer;
    Eigen::Vector3d clearance_gradient_target;
    if (!querySegmentClearance(observer, target, clearance,
                               &clearance_gradient_observer,
                               &clearance_gradient_target, nullptr))
      return false;

    const double violation = std::max(0.0, margin - clearance);
    if (violation <= 0.0 || lambda <= 0.0)
      return true;
    raw_cost = violation * violation;
    weighted_cost = lambda * raw_cost;
    gradient_observer =
        -2.0 * lambda * violation * clearance_gradient_observer;
    gradient_target =
        -2.0 * lambda * violation * clearance_gradient_target;
    return weighted_cost >= 0.0 && std::isfinite(weighted_cost) &&
           gradient_observer.allFinite() && gradient_target.allFinite();
  }

  // Signed point clearance to the same finite scene primitives used by the
  // LOS query.  Positive is outside, zero is on a surface and negative is
  // inside.  This lets the optional team refinement retain a local hard body
  // safety model before every UAV independently performs its authoritative
  // inflated GridMap acceptance check.
  bool queryPointClearance(const Eigen::Vector3d &point,
                           double &clearance,
                           Eigen::Vector3d *gradient = nullptr,
                           int *primitive_index = nullptr) const
  {
    clearance = std::numeric_limits<double>::infinity();
    if (gradient != nullptr)
      gradient->setZero();
    if (primitive_index != nullptr)
      *primitive_index = -1;
    if (!valid() || !point.allFinite())
      return false;

    bool found = false;
    const auto update = [&](const double candidate,
                            const Eigen::Vector3d &candidate_gradient,
                            const int index) {
      if (!std::isfinite(candidate) || !candidate_gradient.allFinite())
        return;
      if (!found || candidate < clearance)
      {
        found = true;
        clearance = candidate;
        if (gradient != nullptr)
          *gradient = candidate_gradient;
        if (primitive_index != nullptr)
          *primitive_index = index;
      }
    };

    for (size_t index = 0; index < cylinders_.size(); ++index)
    {
      const StaticLosCylinder &cylinder = cylinders_[index];
      const Eigen::Vector2d radial_vector = point.head<2>() - cylinder.center;
      const double radial = radial_vector.norm();
      const double center_z = 0.5 * (cylinder.z_min + cylinder.z_max);
      const double half_height = 0.5 * (cylinder.z_max - cylinder.z_min);
      const Eigen::Vector2d q(radial - cylinder.radius,
                              std::abs(point.z() - center_z) - half_height);
      const Eigen::Vector2d outside = q.cwiseMax(0.0);
      const double signed_distance = outside.norm() +
          std::min(std::max(q.x(), q.y()), 0.0);
      Eigen::Vector3d candidate_gradient = Eigen::Vector3d::Zero();
      Eigen::Vector2d radial_normal = Eigen::Vector2d::UnitX();
      if (radial > 1.0e-9)
        radial_normal = radial_vector / radial;
      const double z_sign = point.z() >= center_z ? 1.0 : -1.0;
      if (outside.norm() > 1.0e-9)
      {
        const Eigen::Vector2d q_gradient = outside / outside.norm();
        candidate_gradient.head<2>() = q_gradient.x() * radial_normal;
        candidate_gradient.z() = q_gradient.y() * z_sign;
      }
      else if (q.x() >= q.y())
        candidate_gradient.head<2>() = radial_normal;
      else
        candidate_gradient.z() = z_sign;
      update(signed_distance, candidate_gradient,
             static_cast<int>(index));
    }

    for (size_t index = 0; index < walls_.size(); ++index)
    {
      const StaticLosWall &wall = walls_[index];
      const double cosine = std::cos(wall.yaw);
      const double sine = std::sin(wall.yaw);
      Eigen::Matrix2d world_from_local;
      world_from_local << cosine, -sine, sine, cosine;
      Eigen::Vector3d local;
      local.head<2>() = world_from_local.transpose() *
                        (point.head<2>() - wall.center);
      const double center_z = 0.5 * (wall.z_min + wall.z_max);
      local.z() = point.z() - center_z;
      const Eigen::Vector3d half_size(wall.half_size.x(), wall.half_size.y(),
                                      0.5 * (wall.z_max - wall.z_min));
      const Eigen::Vector3d q = local.cwiseAbs() - half_size;
      const Eigen::Vector3d outside = q.cwiseMax(0.0);
      const double signed_distance = outside.norm() +
          std::min(std::max(q.x(), std::max(q.y(), q.z())), 0.0);
      Eigen::Vector3d local_gradient = Eigen::Vector3d::Zero();
      if (outside.norm() > 1.0e-9)
      {
        local_gradient = outside / outside.norm();
        for (int axis = 0; axis < 3; ++axis)
          local_gradient(axis) *= local(axis) >= 0.0 ? 1.0 : -1.0;
      }
      else
      {
        Eigen::Index axis = 0;
        q.maxCoeff(&axis);
        local_gradient(axis) = local(axis) >= 0.0 ? 1.0 : -1.0;
      }
      Eigen::Vector3d candidate_gradient;
      candidate_gradient.head<2>() =
          world_from_local * local_gradient.head<2>();
      candidate_gradient.z() = local_gradient.z();
      update(signed_distance, candidate_gradient,
             static_cast<int>(cylinders_.size() + index));
    }
    return found;
  }

private:
  std::vector<StaticLosCylinder> cylinders_;
  std::vector<StaticLosWall> walls_;
  std::string scene_file_;
};

} // namespace ego_planner

#endif
