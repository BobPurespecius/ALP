#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <XmlRpcValue.h>
#include <boost/bind.hpp>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace fov_tracking
{
constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(value, high));
}

double wrapPi(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double yawFromQuaternion(const geometry_msgs::Quaternion& q)
{
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

Eigen::Matrix3d rotationZ(double yaw)
{
  Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  rot(0, 0) = c;
  rot(0, 1) = -s;
  rot(1, 0) = s;
  rot(1, 1) = c;
  return rot;
}

Eigen::Vector3d pointToVector(const geometry_msgs::Point& point)
{
  return Eigen::Vector3d(point.x, point.y, point.z);
}

Eigen::Vector3d vectorToEigen(const geometry_msgs::Vector3& vector)
{
  return Eigen::Vector3d(vector.x, vector.y, vector.z);
}

std::vector<std::string> split(const std::string& text, char delimiter)
{
  std::vector<std::string> parts;
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, delimiter))
  {
    if (!item.empty())
    {
      parts.push_back(item);
    }
  }
  return parts;
}

bool xmlRpcNumber(const XmlRpc::XmlRpcValue& value, double& out)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble)
  {
    out = static_cast<double>(value);
    return true;
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    out = static_cast<int>(value);
    return true;
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeString)
  {
    try
    {
      out = std::stod(static_cast<std::string>(value));
      return true;
    }
    catch (const std::exception&)
    {
      return false;
    }
  }
  return false;
}

bool xmlRpcInt(const XmlRpc::XmlRpcValue& value, int& out)
{
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt)
  {
    out = static_cast<int>(value);
    return true;
  }
  double number = 0.0;
  if (xmlRpcNumber(value, number))
  {
    out = static_cast<int>(number);
    return true;
  }
  return false;
}

std::string formatTopicTemplate(std::string text, int drone_id)
{
  const std::string value = std::to_string(drone_id);
  const std::vector<std::string> keys = { "{id}", "{index}" };
  for (const auto& key : keys)
  {
    std::size_t pos = 0;
    while ((pos = text.find(key, pos)) != std::string::npos)
    {
      text.replace(pos, key.size(), value);
      pos += value.size();
    }
  }
  return text;
}

std::vector<int> parseDroneIds(ros::NodeHandle& pnh)
{
  XmlRpc::XmlRpcValue value;
  std::vector<int> ids;
  if (!pnh.getParam("drone_ids", value))
  {
    return { 0 };
  }

  if (value.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (int i = 0; i < value.size(); ++i)
    {
      int id = 0;
      if (xmlRpcInt(value[i], id))
      {
        ids.push_back(id);
      }
    }
  }
  else if (value.getType() == XmlRpc::XmlRpcValue::TypeString)
  {
    for (const auto& part : split(static_cast<std::string>(value), ','))
    {
      ids.push_back(std::stoi(part));
    }
  }
  else
  {
    int id = 0;
    if (xmlRpcInt(value, id))
    {
      ids.push_back(id);
    }
  }

  if (ids.empty())
  {
    ids.push_back(0);
  }
  return ids;
}

Eigen::Vector3d parseVector3String(const std::string& text)
{
  std::vector<std::string> parts = split(text, ',');
  Eigen::Vector3d out = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < std::min<std::size_t>(3, parts.size()); ++i)
  {
    out(static_cast<int>(i)) = std::stod(parts[i]);
  }
  return out;
}

std::vector<Eigen::Vector3d> parseRelativePositions(ros::NodeHandle& pnh, std::size_t drone_count,
                                                     double default_distance)
{
  XmlRpc::XmlRpcValue value;
  std::vector<Eigen::Vector3d> rels;
  if (pnh.getParam("relative_positions", value))
  {
    if (value.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int i = 0; i < value.size(); ++i)
      {
        if (value[i].getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
          continue;
        }
        Eigen::Vector3d rel = Eigen::Vector3d::Zero();
        const int limit = std::min(3, value[i].size());
        for (int dim = 0; dim < limit; ++dim)
        {
          double number = 0.0;
          if (xmlRpcNumber(value[i][dim], number))
          {
            rel(dim) = number;
          }
        }
        rels.push_back(rel);
      }
    }
    else if (value.getType() == XmlRpc::XmlRpcValue::TypeString)
    {
      for (const auto& item : split(static_cast<std::string>(value), ';'))
      {
        rels.push_back(parseVector3String(item));
      }
    }
  }

  if (rels.empty())
  {
    if (drone_count <= 1)
    {
      rels.push_back(Eigen::Vector3d(-default_distance, 0.0, 0.0));
    }
    else
    {
      for (std::size_t idx = 0; idx < drone_count; ++idx)
      {
        const double angle = kPi + 2.0 * kPi * static_cast<double>(idx) / static_cast<double>(drone_count);
        rels.emplace_back(default_distance * std::cos(angle), default_distance * std::sin(angle), 0.0);
      }
    }
  }

  while (rels.size() < drone_count)
  {
    rels.push_back(rels.back());
  }
  rels.resize(drone_count);
  return rels;
}

std::vector<double> parseDoubleList(ros::NodeHandle& pnh, const std::string& name,
                                    const std::vector<double>& fallback)
{
  XmlRpc::XmlRpcValue value;
  std::vector<double> out;
  if (!pnh.getParam(name, value))
  {
    return fallback;
  }

  if (value.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (int i = 0; i < value.size(); ++i)
    {
      double number = 0.0;
      if (xmlRpcNumber(value[i], number))
      {
        out.push_back(number);
      }
    }
  }
  else if (value.getType() == XmlRpc::XmlRpcValue::TypeString)
  {
    for (const auto& item : split(static_cast<std::string>(value), ','))
    {
      out.push_back(std::stod(item));
    }
  }

  return out.empty() ? fallback : out;
}

struct Obstacle
{
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  double radius = 0.0;
  double z_min = -std::numeric_limits<double>::infinity();
  double z_max = std::numeric_limits<double>::infinity();
};

double structNumber(XmlRpc::XmlRpcValue& value, const std::string& name, double fallback)
{
  if (!value.hasMember(name))
  {
    return fallback;
  }
  double out = fallback;
  xmlRpcNumber(value[name], out);
  return out;
}

std::vector<Obstacle> parseObstacles(ros::NodeHandle& pnh)
{
  XmlRpc::XmlRpcValue value;
  std::vector<Obstacle> obstacles;
  if (!pnh.getParam("obstacles", value))
  {
    return obstacles;
  }

  if (value.getType() == XmlRpc::XmlRpcValue::TypeString)
  {
    for (const auto& item : split(static_cast<std::string>(value), ';'))
    {
      const auto parts = split(item, ',');
      if (parts.size() < 3)
      {
        continue;
      }
      Obstacle obstacle;
      obstacle.center = Eigen::Vector2d(std::stod(parts[0]), std::stod(parts[1]));
      obstacle.radius = std::stod(parts[2]);
      if (parts.size() > 3)
      {
        obstacle.z_min = std::stod(parts[3]);
      }
      if (parts.size() > 4)
      {
        obstacle.z_max = std::stod(parts[4]);
      }
      obstacles.push_back(obstacle);
    }
    return obstacles;
  }

  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    return obstacles;
  }

  for (int i = 0; i < value.size(); ++i)
  {
    Obstacle obstacle;
    if (value[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
    {
      obstacle.center = Eigen::Vector2d(structNumber(value[i], "x", 0.0), structNumber(value[i], "y", 0.0));
      obstacle.radius = structNumber(value[i], "radius", 0.0);
      obstacle.z_min = structNumber(value[i], "z_min", obstacle.z_min);
      obstacle.z_max = structNumber(value[i], "z_max", obstacle.z_max);
      obstacles.push_back(obstacle);
    }
    else if (value[i].getType() == XmlRpc::XmlRpcValue::TypeArray && value[i].size() >= 3)
    {
      double x = 0.0;
      double y = 0.0;
      double radius = 0.0;
      xmlRpcNumber(value[i][0], x);
      xmlRpcNumber(value[i][1], y);
      xmlRpcNumber(value[i][2], radius);
      obstacle.center = Eigen::Vector2d(x, y);
      obstacle.radius = radius;
      if (value[i].size() > 3)
      {
        xmlRpcNumber(value[i][3], obstacle.z_min);
      }
      if (value[i].size() > 4)
      {
        xmlRpcNumber(value[i][4], obstacle.z_max);
      }
      obstacles.push_back(obstacle);
    }
  }
  return obstacles;
}

double segmentCircleDistance(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& center)
{
  const Eigen::Vector2d ab = b - a;
  const double denom = ab.squaredNorm();
  if (denom <= 1.0e-9)
  {
    return (a - center).norm();
  }
  const double t = clamp((center - a).dot(ab) / denom, 0.0, 1.0);
  return (a + t * ab - center).norm();
}

struct TrajSample
{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d jerk = Eigen::Vector3d::Zero();
};

TrajSample minJerkState(const Eigen::Vector3d& p0, const Eigen::Vector3d& v0, const Eigen::Vector3d& a0,
                        const Eigen::Vector3d& p1, const Eigen::Vector3d& v1, const Eigen::Vector3d& a1,
                        double duration, double sample_t)
{
  const double T = std::max(duration, 1.0e-3);
  const double t = clamp(sample_t, 0.0, T);
  Eigen::Matrix3d mat;
  mat << std::pow(T, 3), std::pow(T, 4), std::pow(T, 5),
         3.0 * std::pow(T, 2), 4.0 * std::pow(T, 3), 5.0 * std::pow(T, 4),
         6.0 * T, 12.0 * std::pow(T, 2), 20.0 * std::pow(T, 3);

  TrajSample sample;
  for (int dim = 0; dim < 3; ++dim)
  {
    const double c0 = p0(dim);
    const double c1 = v0(dim);
    const double c2 = 0.5 * a0(dim);
    Eigen::Vector3d rhs;
    rhs << p1(dim) - (c0 + c1 * T + c2 * T * T),
           v1(dim) - (c1 + 2.0 * c2 * T),
           a1(dim) - (2.0 * c2);
    const Eigen::Vector3d coeff = mat.colPivHouseholderQr().solve(rhs);
    const double c3 = coeff(0);
    const double c4 = coeff(1);
    const double c5 = coeff(2);

    sample.pos(dim) = c0 + c1 * t + c2 * std::pow(t, 2) + c3 * std::pow(t, 3) +
                      c4 * std::pow(t, 4) + c5 * std::pow(t, 5);
    sample.vel(dim) = c1 + 2.0 * c2 * t + 3.0 * c3 * std::pow(t, 2) +
                      4.0 * c4 * std::pow(t, 3) + 5.0 * c5 * std::pow(t, 4);
    sample.acc(dim) = 2.0 * c2 + 6.0 * c3 * t + 12.0 * c4 * std::pow(t, 2) +
                      20.0 * c5 * std::pow(t, 3);
    sample.jerk(dim) = 6.0 * c3 + 24.0 * c4 * t + 60.0 * c5 * std::pow(t, 2);
  }
  return sample;
}

struct DroneRuntime
{
  nav_msgs::Odometry odom;
  bool has_odom = false;
  ros::Subscriber odom_sub;
  ros::Publisher command_pub;
};

struct State
{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  double yaw = 0.0;
};

struct Goal
{
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_vel = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  bool visible = false;
  bool team_visible = false;
  double score = std::numeric_limits<double>::infinity();
};

class FovTrackingPlannerNode
{
public:
  FovTrackingPlannerNode() : pnh_("~")
  {
    loadParams();

    target_sub_ = nh_.subscribe(object_odom_topic_, 1, &FovTrackingPlannerNode::targetCallback, this);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 10);

    for (const int drone_id : drone_ids_)
    {
      DroneRuntime& runtime = drones_[drone_id];
      const std::string odom_topic = formatTopicTemplate(odom_topic_template_, drone_id);
      const std::string command_topic = formatTopicTemplate(command_topic_template_, drone_id);
      runtime.odom_sub = nh_.subscribe<nav_msgs::Odometry>(
          odom_topic, 1, boost::bind(&FovTrackingPlannerNode::odomCallback, this, _1, drone_id));
      runtime.command_pub = nh_.advertise<quadrotor_msgs::PositionCommand>(command_topic, 10);
      trajectory_ids_[drone_id] = 0;
      ROS_INFO("FOV planner drone %d: odom=%s command=%s", drone_id, odom_topic.c_str(), command_topic.c_str());
    }

    const double period = 1.0 / std::max(rate_hz_, 1.0);
    timer_ = nh_.createTimer(ros::Duration(period), &FovTrackingPlannerNode::timerCallback, this);
    ROS_WARN("FOV C++ tracking planner ready: object=%s marker=%s frame=%s",
             object_odom_topic_.c_str(), marker_topic_.c_str(), frame_id_.c_str());
  }

private:
  void loadParams()
  {
    pnh_.param<std::string>("frame_id", frame_id_, "world");
    pnh_.param<std::string>("object_odom_topic", object_odom_topic_, "/object_odom");
    pnh_.param<std::string>("odom_topic_template", odom_topic_template_, "/drone_{id}_visual_slam/odom");
    pnh_.param<std::string>("command_topic_template", command_topic_template_, "/drone_{id}_planning/pos_cmd");
    pnh_.param<std::string>("marker_topic", marker_topic_, "/fov_tracking/markers");

    drone_ids_ = parseDroneIds(pnh_);

    pnh_.param("rate", rate_hz_, 30.0);
    pnh_.param("prediction_dt", prediction_dt_, 0.25);
    pnh_.param("prediction_steps", prediction_steps_, 8);
    pnh_.param("lookahead_time", lookahead_time_, 1.0);
    pnh_.param("plan_duration", plan_duration_, 1.6);
    pnh_.param("command_time_forward", command_time_forward_, 0.25);

    pnh_.param("tracking_distance", tracking_distance_, 1.2);
    relative_positions_ = parseRelativePositions(pnh_, drone_ids_.size(), tracking_distance_);
    pnh_.param("min_target_distance", min_target_distance_, 0.7);
    pnh_.param("max_target_distance", max_target_distance_, 3.0);
    pnh_.param("min_uav_distance", min_uav_distance_, 0.8);
    double fov_half_angle_deg = 45.0;
    pnh_.param("fov_half_angle_deg", fov_half_angle_deg, 45.0);
    fov_half_angle_ = fov_half_angle_deg * kPi / 180.0;
    pnh_.param("enable_candidate_goal", enable_candidate_goal_, false);
    pnh_.param("candidate_count", candidate_count_, 33);
    double candidate_angle_span_deg = 220.0;
    pnh_.param("candidate_angle_span_deg", candidate_angle_span_deg, 220.0);
    candidate_angle_span_ = candidate_angle_span_deg * kPi / 180.0;
    candidate_radius_samples_ = parseDoubleList(pnh_, "candidate_radius_samples", { 0.85, 1.0, 1.2 });

    pnh_.param("max_vel_xy", max_vel_xy_, 1.0);
    pnh_.param("max_vel_z", max_vel_z_, 0.45);
    pnh_.param("max_acc", max_acc_, 1.8);
    pnh_.param("target_height", target_height_, 1.2);
    pnh_.param("lock_tracking_height", lock_tracking_height_, true);

    pnh_.param("desired_weight", desired_weight_, 1.0);
    pnh_.param("smooth_weight", smooth_weight_, 0.2);
    pnh_.param("distance_weight", distance_weight_, 0.6);
    pnh_.param("occlusion_weight", occlusion_weight_, 3.0);
    pnh_.param("shared_visibility_weight", shared_visibility_weight_, 0.8);
    pnh_.param("lost_visibility_weight", lost_visibility_weight_, 6.0);
    pnh_.param("inter_uav_weight", inter_uav_weight_, 4.0);
    obstacles_ = parseObstacles(pnh_);
  }

  void targetCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    target_odom_ = *msg;
    has_target_ = true;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, int drone_id)
  {
    DroneRuntime& runtime = drones_[drone_id];
    runtime.odom = *msg;
    runtime.has_odom = true;
  }

  bool targetState(State& state) const
  {
    if (!has_target_)
    {
      return false;
    }
    state.pos = pointToVector(target_odom_.pose.pose.position);
    state.vel = vectorToEigen(target_odom_.twist.twist.linear);
    state.yaw = yawFromQuaternion(target_odom_.pose.pose.orientation);
    if (state.vel.head<2>().norm() > 1.0e-3)
    {
      state.yaw = std::atan2(state.vel.y(), state.vel.x());
    }
    if (lock_tracking_height_)
    {
      state.pos.z() = target_height_;
      state.vel.z() = 0.0;
    }
    return true;
  }

  bool droneState(int drone_id, State& state) const
  {
    const auto it = drones_.find(drone_id);
    if (it == drones_.end() || !it->second.has_odom)
    {
      return false;
    }
    state.pos = pointToVector(it->second.odom.pose.pose.position);
    state.vel = vectorToEigen(it->second.odom.twist.twist.linear);
    state.yaw = yawFromQuaternion(it->second.odom.pose.pose.orientation);
    return true;
  }

  State predictedTarget(const State& target, double t) const
  {
    State pred = target;
    pred.pos = target.pos + target.vel * std::max(0.0, t);
    if (target.vel.head<2>().norm() > 1.0e-3)
    {
      pred.yaw = std::atan2(target.vel.y(), target.vel.x());
    }
    if (lock_tracking_height_)
    {
      pred.pos.z() = target_height_;
      pred.vel.z() = 0.0;
    }
    return pred;
  }

  Eigen::Vector3d desiredTrackingPoint(std::size_t drone_index, const State& target) const
  {
    Eigen::Vector3d desired = target.pos + rotationZ(target.yaw) * relative_positions_[drone_index];
    if (lock_tracking_height_)
    {
      desired.z() = target_height_ + relative_positions_[drone_index].z();
    }
    return desired;
  }

  bool lineOfSightClear(const Eigen::Vector3d& candidate, const Eigen::Vector3d& target) const
  {
    const Eigen::Vector2d a = candidate.head<2>();
    const Eigen::Vector2d b = target.head<2>();
    const double z_min = std::min(candidate.z(), target.z());
    const double z_max = std::max(candidate.z(), target.z());
    for (const auto& obstacle : obstacles_)
    {
      if (obstacle.z_max < z_min || obstacle.z_min > z_max)
      {
        continue;
      }
      if (segmentCircleDistance(a, b, obstacle.center) <= obstacle.radius)
      {
        return false;
      }
    }
    return true;
  }

  bool visibleFrom(const Eigen::Vector3d& observer_pos, const Eigen::Vector3d& target_pos,
                   double observer_yaw, bool use_yaw) const
  {
    const double distance = (observer_pos - target_pos).norm();
    if (distance < min_target_distance_ || distance > max_target_distance_)
    {
      return false;
    }
    if (!lineOfSightClear(observer_pos, target_pos))
    {
      return false;
    }
    if (use_yaw)
    {
      const double bearing = std::atan2(target_pos.y() - observer_pos.y(), target_pos.x() - observer_pos.x());
      if (std::abs(wrapPi(bearing - observer_yaw)) > fov_half_angle_)
      {
        return false;
      }
    }
    return true;
  }

  bool teamHasVisibility(const Eigen::Vector3d& target_pos) const
  {
    for (const int drone_id : drone_ids_)
    {
      State state;
      if (!droneState(drone_id, state))
      {
        continue;
      }
      if (visibleFrom(state.pos, target_pos, state.yaw, true))
      {
        return true;
      }
    }
    return false;
  }

  std::vector<Eigen::Vector3d> candidatePoints(const Eigen::Vector3d& desired, const Eigen::Vector3d& target_pos,
                                               double target_yaw, const Eigen::Vector3d& rel) const
  {
    if (!enable_candidate_goal_)
    {
      return { desired };
    }

    const Eigen::Vector2d rel_xy = desired.head<2>() - target_pos.head<2>();
    const double base_angle = rel_xy.norm() > 1.0e-3 ? std::atan2(rel_xy.y(), rel_xy.x()) : target_yaw + kPi;
    const double base_radius = std::max(rel.head<2>().norm(), tracking_distance_);
    const int count = std::max(1, candidate_count_);
    std::vector<Eigen::Vector3d> points;
    points.reserve(1 + candidate_radius_samples_.size() * static_cast<std::size_t>(count));
    points.push_back(desired);

    for (const double scale : candidate_radius_samples_)
    {
      const double radius = base_radius * scale;
      for (int idx = 0; idx < count; ++idx)
      {
        const double alpha = count == 1 ? 0.5 : static_cast<double>(idx) / static_cast<double>(count - 1);
        const double offset = -0.5 * candidate_angle_span_ + alpha * candidate_angle_span_;
        const double angle = base_angle + offset;
        points.emplace_back(target_pos.x() + radius * std::cos(angle),
                            target_pos.y() + radius * std::sin(angle),
                            desired.z());
      }
    }
    return points;
  }

  double interUavCost(const Eigen::Vector3d& candidate, int drone_id) const
  {
    double cost = 0.0;
    for (const auto& item : last_goals_)
    {
      if (item.first == drone_id)
      {
        continue;
      }
      const double distance = (candidate - item.second).norm();
      if (distance < min_uav_distance_)
      {
        cost += inter_uav_weight_ * std::pow(min_uav_distance_ - distance, 2);
      }
    }
    return cost;
  }

  Goal selectGoal(int drone_id, std::size_t drone_index, const State& drone, const State& target) const
  {
    const State pred = predictedTarget(target, lookahead_time_);
    const Eigen::Vector3d desired = desiredTrackingPoint(drone_index, pred);
    const Eigen::Vector3d rel = relative_positions_[drone_index];
    const bool team_visible = teamHasVisibility(target.pos);

    Goal best;
    bool has_best = false;
    for (const auto& candidate : candidatePoints(desired, pred.pos, pred.yaw, rel))
    {
      const double distance = (candidate - pred.pos).norm();
      if (distance < min_target_distance_ || distance > max_target_distance_)
      {
        continue;
      }

      const double yaw = std::atan2(pred.pos.y() - candidate.y(), pred.pos.x() - candidate.x());
      const bool visible = visibleFrom(candidate, pred.pos, yaw, true);
      const double desired_cost = desired_weight_ * (candidate - desired).squaredNorm();
      const double smooth_cost = smooth_weight_ * (candidate - drone.pos - drone.vel * plan_duration_).squaredNorm();
      const double distance_cost = distance_weight_ * std::pow(distance - tracking_distance_, 2);
      const double obstacle_cost = lineOfSightClear(candidate, pred.pos) ? 0.0 : occlusion_weight_;
      const double visibility_cost = visible ? 0.0 : (team_visible ? shared_visibility_weight_ : lost_visibility_weight_);
      const double score = desired_cost + smooth_cost + distance_cost + obstacle_cost + visibility_cost +
                           interUavCost(candidate, drone_id);

      if (!has_best || score < best.score)
      {
        has_best = true;
        best.point = candidate;
        best.yaw = yaw;
        best.visible = visible;
        best.team_visible = team_visible;
        best.score = score;
        best.target_vel = pred.vel;
      }
    }

    if (!has_best)
    {
      best.point = desired;
      best.yaw = std::atan2(pred.pos.y() - desired.y(), pred.pos.x() - desired.x());
      best.visible = false;
      best.team_visible = team_visible;
      best.target_vel = pred.vel;
    }
    return best;
  }

  Eigen::Vector3d boundedVelocity(Eigen::Vector3d vel) const
  {
    const double xy_norm = vel.head<2>().norm();
    if (xy_norm > max_vel_xy_ && max_vel_xy_ > 1.0e-6)
    {
      vel.head<2>() *= max_vel_xy_ / xy_norm;
    }
    vel.z() = clamp(vel.z(), -max_vel_z_, max_vel_z_);
    return vel;
  }

  void publishCommand(int drone_id, const State& drone, const Goal& goal)
  {
    Eigen::Vector3d goal_vel = goal.target_vel;
    const double goal_xy_norm = goal_vel.head<2>().norm();
    if (goal_xy_norm > max_vel_xy_ && max_vel_xy_ > 1.0e-6)
    {
      goal_vel.head<2>() *= max_vel_xy_ / goal_xy_norm;
    }
    if (lock_tracking_height_)
    {
      goal_vel.z() = 0.0;
    }

    TrajSample sample = minJerkState(drone.pos, drone.vel, Eigen::Vector3d::Zero(),
                                     goal.point, goal_vel, Eigen::Vector3d::Zero(),
                                     plan_duration_, command_time_forward_);
    sample.vel = boundedVelocity(sample.vel);
    for (int dim = 0; dim < 3; ++dim)
    {
      sample.acc(dim) = clamp(sample.acc(dim), -max_acc_, max_acc_);
    }

    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = frame_id_;
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd.trajectory_id = ++trajectory_ids_[drone_id];
    cmd.position.x = sample.pos.x();
    cmd.position.y = sample.pos.y();
    cmd.position.z = sample.pos.z();
    cmd.velocity.x = sample.vel.x();
    cmd.velocity.y = sample.vel.y();
    cmd.velocity.z = sample.vel.z();
    cmd.acceleration.x = sample.acc.x();
    cmd.acceleration.y = sample.acc.y();
    cmd.acceleration.z = sample.acc.z();
    cmd.yaw = goal.yaw;
    cmd.yaw_dot = 0.0;

    drones_[drone_id].command_pub.publish(cmd);
  }

  visualization_msgs::Marker makeMarker(int id, int type, const std::string& ns,
                                         double r, double g, double b, double a,
                                         double sx, double sy, double sz) const
  {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id_;
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0.4);
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.scale.x = sx;
    marker.scale.y = sy;
    marker.scale.z = sz;
    marker.pose.orientation.w = 1.0;
    return marker;
  }

  void publishMarkers(const Eigen::Vector3d& target_pos)
  {
    visualization_msgs::MarkerArray array;
    visualization_msgs::Marker target = makeMarker(0, visualization_msgs::Marker::SPHERE, "target",
                                                   1.0, 0.15, 0.05, 0.9,
                                                   0.25, 0.25, 0.25);
    target.pose.position.x = target_pos.x();
    target.pose.position.y = target_pos.y();
    target.pose.position.z = target_pos.z();
    array.markers.push_back(target);

    int marker_id = 1;
    for (const auto& item : last_goals_)
    {
      const int drone_id = item.first;
      const Eigen::Vector3d& goal = item.second;
      const bool visible = last_visibility_.count(drone_id) > 0 && last_visibility_[drone_id];
      const double r = visible ? 0.0 : 1.0;
      const double g = visible ? 0.85 : 0.65;
      const double b = visible ? 0.25 : 0.0;

      visualization_msgs::Marker point = makeMarker(marker_id++, visualization_msgs::Marker::SPHERE, "fov_goal",
                                                    r, g, b, 0.9, 0.18, 0.18, 0.18);
      point.pose.position.x = goal.x();
      point.pose.position.y = goal.y();
      point.pose.position.z = goal.z();
      array.markers.push_back(point);

      visualization_msgs::Marker line = makeMarker(marker_id++, visualization_msgs::Marker::LINE_STRIP, "fov_los",
                                                   r, g, b, 0.9, 0.025, 0.0, 0.0);
      geometry_msgs::Point p0;
      p0.x = goal.x();
      p0.y = goal.y();
      p0.z = goal.z();
      geometry_msgs::Point p1;
      p1.x = target_pos.x();
      p1.y = target_pos.y();
      p1.z = target_pos.z();
      line.points.push_back(p0);
      line.points.push_back(p1);
      array.markers.push_back(line);
    }

    marker_pub_.publish(array);
  }

  void timerCallback(const ros::TimerEvent&)
  {
    State target;
    if (!targetState(target))
    {
      ROS_WARN_THROTTLE(2.0, "FOV planner waiting for target odom on %s", object_odom_topic_.c_str());
      return;
    }

    for (std::size_t idx = 0; idx < drone_ids_.size(); ++idx)
    {
      const int drone_id = drone_ids_[idx];
      State drone;
      if (!droneState(drone_id, drone))
      {
        ROS_WARN_THROTTLE(2.0, "FOV planner waiting for drone %d odom", drone_id);
        continue;
      }

      const Goal goal = selectGoal(drone_id, idx, drone, target);
      last_goals_[drone_id] = goal.point;
      last_visibility_[drone_id] = goal.visible;
      publishCommand(drone_id, drone, goal);
    }

    publishMarkers(target.pos);
    int visible_count = 0;
    for (const auto& item : last_visibility_)
    {
      if (item.second)
      {
        ++visible_count;
      }
    }
    ROS_INFO_THROTTLE(1.0, "FOV planner target=(%.2f %.2f %.2f) visible=%d/%zu",
                      target.pos.x(), target.pos.y(), target.pos.z(), visible_count, drone_ids_.size());
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber target_sub_;
  ros::Publisher marker_pub_;
  ros::Timer timer_;

  std::string frame_id_;
  std::string object_odom_topic_;
  std::string odom_topic_template_;
  std::string command_topic_template_;
  std::string marker_topic_;
  std::vector<int> drone_ids_;
  std::map<int, DroneRuntime> drones_;
  nav_msgs::Odometry target_odom_;
  bool has_target_ = false;

  double rate_hz_ = 30.0;
  double prediction_dt_ = 0.25;
  int prediction_steps_ = 8;
  double lookahead_time_ = 1.0;
  double plan_duration_ = 1.6;
  double command_time_forward_ = 0.25;

  double tracking_distance_ = 1.2;
  std::vector<Eigen::Vector3d> relative_positions_;
  double min_target_distance_ = 0.7;
  double max_target_distance_ = 3.0;
  double min_uav_distance_ = 0.8;
  double fov_half_angle_ = kPi / 4.0;
  bool enable_candidate_goal_ = false;
  int candidate_count_ = 33;
  double candidate_angle_span_ = 220.0 * kPi / 180.0;
  std::vector<double> candidate_radius_samples_;
  double max_vel_xy_ = 1.0;
  double max_vel_z_ = 0.45;
  double max_acc_ = 1.8;
  double target_height_ = 1.2;
  bool lock_tracking_height_ = true;

  double desired_weight_ = 1.0;
  double smooth_weight_ = 0.2;
  double distance_weight_ = 0.6;
  double occlusion_weight_ = 3.0;
  double shared_visibility_weight_ = 0.8;
  double lost_visibility_weight_ = 6.0;
  double inter_uav_weight_ = 4.0;
  std::vector<Obstacle> obstacles_;

  std::map<int, Eigen::Vector3d> last_goals_;
  std::map<int, bool> last_visibility_;
  std::map<int, uint32_t> trajectory_ids_;
};
}  // namespace fov_tracking

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fov_tracking_planner");
  fov_tracking::FovTrackingPlannerNode node;
  ros::spin();
  return 0;
}
