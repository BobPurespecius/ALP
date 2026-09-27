#include <multi_uav_formation/cooperative_viewpoint_core.h>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <traj_utils/CooperativeTaskReference.h>
#include <traj_utils/TaskReferenceAdjustment.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/bind/bind.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <string>

namespace
{

// This process owns one thing only: the cooperative soft encirclement
// reference. It never generates a trajectory, a recovery guide, or an
// alternative endpoint bundle. Local N/L/R and Team P/T consume the same
// published reference through their normal soft objectives.
class CooperativeViewpointManager
{
public:
  CooperativeViewpointManager() : private_nh_("~")
  {
    private_nh_.param("rate", rate_hz_, 10.0);
    private_nh_.param("visibility_horizon", horizon_, 2.0);
    private_nh_.param("visibility_sample_dt", sample_dt_, 0.1);
    private_nh_.param("min_target_distance", min_range_, 0.2);
    private_nh_.param("max_target_distance", max_range_, 8.0);
    private_nh_.param("static_los_margin", los_margin_, 0.08);
    private_nh_.param("reference_static_clearance", reference_clearance_, 0.0);
    private_nh_.param("visibility_bad_hold", visibility_bad_hold_, 0.5);
    private_nh_.param("viewpoint_trigger_tracking_error",
                      trigger_tracking_error_, 0.5);
    private_nh_.param("theta_sep_min_deg", theta_sep_min_deg_, 25.0);
    private_nh_.param("min_visibility_improvement",
                      min_visibility_improvement_, 0.10);
    private_nh_.param("k2_near_best_tolerance",
                      k2_near_best_tolerance_, 0.01);
    private_nh_.param("omega_ref_max", max_angular_rate_, 0.25);
    private_nh_.param("viewpoint_arrive_distance", arrive_distance_, 0.5);
    private_nh_.param("viewpoint_arrive_hold", arrive_hold_, 0.5);
    private_nh_.param("target_timeout", target_timeout_, 0.5);
    private_nh_.param("odom_timeout", odom_timeout_, 0.5);
    private_nh_.param("camera_hfov_deg", camera_hfov_deg_, 85.0);
    private_nh_.param("camera_image_width", camera_image_width_, 1280.0);
    private_nh_.param("camera_image_height", camera_image_height_, 720.0);
    private_nh_.param("camera_extrinsic_x", camera_extrinsic_x_, 0.0);
    private_nh_.param("camera_extrinsic_y", camera_extrinsic_y_, 0.0);
    private_nh_.param("camera_extrinsic_z", camera_extrinsic_z_, 0.0);
    private_nh_.param("camera_extrinsic_roll", camera_extrinsic_roll_, 0.0);
    private_nh_.param("camera_extrinsic_pitch", camera_extrinsic_pitch_, 0.0);
    private_nh_.param("camera_extrinsic_yaw", camera_extrinsic_yaw_, 0.0);
    private_nh_.param<std::string>("target_topic", target_topic_,
                                   "/object_odom");
    private_nh_.param<std::string>("scene_file", scene_file_, "");
    private_nh_.param<std::string>("frame_id", frame_id_, "world");

    std::array<Eigen::Vector3d, 3> offsets;
    for (int index = 0; index < 3; ++index)
    {
      private_nh_.param("offset" + std::to_string(index) + "_x",
                        offsets[index].x(), index == 1 ? -1.7 : -1.5);
      private_nh_.param("offset" + std::to_string(index) + "_y",
                        offsets[index].y(), index == 0 ? -0.85 :
                                             (index == 2 ? 0.85 : 0.0));
      private_nh_.param("offset" + std::to_string(index) + "_z",
                        offsets[index].z(), 0.0);
    }
    core_.setOffsets(offsets);
    core_.setEncirclementTracking(true);
    core_.configure(
        horizon_, sample_dt_, min_range_, max_range_, los_margin_,
        reference_clearance_, visibility_bad_hold_, trigger_tracking_error_,
        theta_sep_min_deg_ * M_PI / 180.0, min_visibility_improvement_,
        max_angular_rate_, arrive_distance_, arrive_hold_,
        k2_near_best_tolerance_);

    multi_uav_formation::TrackingCameraContract camera;
    camera.horizontal_fov = camera_hfov_deg_ * M_PI / 180.0;
    camera.vertical_fov = multi_uav_formation::verticalFovFromHorizontal(
        camera.horizontal_fov, camera_image_width_, camera_image_height_);
    camera.min_range = min_range_;
    camera.max_range = max_range_;
    camera.translation_body = Eigen::Vector3d(
        camera_extrinsic_x_, camera_extrinsic_y_, camera_extrinsic_z_);
    camera.body_from_camera =
        Eigen::AngleAxisd(camera_extrinsic_yaw_, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(camera_extrinsic_pitch_, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(camera_extrinsic_roll_, Eigen::Vector3d::UnitX());
    core_.setCameraContract(camera);

    std::string geometry_reason;
    if (!core_.loadScene(scene_file_, geometry_reason))
      ROS_ERROR("[cooperative-soft-reference] static_geometry_loaded=0 "
                "reason=%s scene=%s", geometry_reason.c_str(),
                scene_file_.c_str());

    target_sub_ = nh_.subscribe(
        target_topic_, 1, &CooperativeViewpointManager::targetCallback, this,
        ros::TransportHints().tcpNoDelay());
    motion_start_sub_ = nh_.subscribe(
        "/dynamic/motion_start_time", 1,
        &CooperativeViewpointManager::motionStartCallback, this);
    visible_count_sub_ = nh_.subscribe(
        "/native_egov2/visibility/visible_count", 10,
        &CooperativeViewpointManager::visibleCountCallback, this);
    for (int index = 0; index < 3; ++index)
    {
      odom_subs_[index] = nh_.subscribe<nav_msgs::Odometry>(
          "/drone_" + std::to_string(index) + "_visual_slam/odom", 1,
          boost::bind(&CooperativeViewpointManager::odomCallback, this,
                      boost::placeholders::_1, index),
          ros::VoidPtr(), ros::TransportHints().tcpNoDelay());
      reference_pubs_[index] = nh_.advertise<nav_msgs::Odometry>(
          "/cooperative_viewpoint/uav" + std::to_string(index) +
              "/reference", 2);
    }
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/cooperative_viewpoint/markers", 2);
    // 阶段 B：本进程是 CooperativeTaskReference 的唯一 producer。
    // latched，使后启动的 consumer 也能拿到最近一次任务几何。
    task_reference_pub_ = nh_.advertise<traj_utils::CooperativeTaskReference>(
        "/cooperative_task_reference", 1, true);
    // Team -> Cooperative 的任务级 phi0 建议（Team 不是 authority，本进程
    // 仍通过 core 内的 rate-limited transition 决定是否接受）。
    adjustment_sub_ = nh_.subscribe(
        "/cooperative_task_adjustment", 1,
        &CooperativeViewpointManager::adjustmentCallback, this);
    timer_ = nh_.createTimer(
        ros::Duration(1.0 / std::max(1.0, rate_hz_)),
        &CooperativeViewpointManager::timerCallback, this);

    ROS_INFO("[cooperative-soft-reference] active=1 planner_calls=0 "
             "trajectory_authority=0 adaptive_hypotheses=0 "
             "reference_model=SOFT_120_SLOTS");
  }

private:
  void targetCallback(const nav_msgs::OdometryConstPtr &message)
  {
    const Eigen::Vector3d position(message->pose.pose.position.x,
                                   message->pose.pose.position.y,
                                   message->pose.pose.position.z);
    const Eigen::Vector3d velocity(message->twist.twist.linear.x,
                                   message->twist.twist.linear.y,
                                   message->twist.twist.linear.z);
    if (!position.allFinite() || !velocity.allFinite())
      return;
    target_position_ = position;
    target_velocity_ = velocity;
    target_stamp_ = message->header.stamp.isZero() ? ros::Time::now()
                                                   : message->header.stamp;
    have_target_ = true;
  }

  void motionStartCallback(const std_msgs::Float64ConstPtr &message)
  {
    if (message && std::isfinite(message->data))
    {
      dynamic_motion_start_ = message->data;
      have_dynamic_motion_start_ = true;
    }
  }

  void visibleCountCallback(const std_msgs::UInt8ConstPtr &message)
  {
    observed_visible_count_ =
        std::max(0, std::min(3, static_cast<int>(message->data)));
    observed_visible_count_stamp_ = ros::Time::now();
  }

  void odomCallback(const nav_msgs::OdometryConstPtr &message,
                    const int index)
  {
    const Eigen::Vector3d position(message->pose.pose.position.x,
                                   message->pose.pose.position.y,
                                   message->pose.pose.position.z);
    if (!position.allFinite())
      return;
    odom_positions_[index] = position;
    odom_stamps_[index] = message->header.stamp.isZero() ? ros::Time::now()
                                                         : message->header.stamp;
    have_odom_[index] = true;

    const auto &orientation = message->pose.pose.orientation;
    Eigen::Quaterniond quaternion(orientation.w, orientation.x,
                                  orientation.y, orientation.z);
    const double yaw_rate = message->twist.twist.angular.z;
    if (quaternion.coeffs().allFinite() && quaternion.norm() > 1.0e-12 &&
        std::isfinite(yaw_rate))
    {
      quaternion.normalize();
      observer_yaw_states_[index].valid = true;
      observer_yaw_states_[index].yaw = std::atan2(
          2.0 * (quaternion.w() * quaternion.z() +
                 quaternion.x() * quaternion.y()),
          1.0 - 2.0 * (quaternion.y() * quaternion.y() +
                       quaternion.z() * quaternion.z()));
      observer_yaw_states_[index].yaw_rate = yaw_rate;
    }
    else
      observer_yaw_states_[index].valid = false;
  }

  static bool publishable(const multi_uav_formation::ViewpointStep &step)
  {
    for (int drone = 0; drone < 3; ++drone)
      if (!step.reference_valid[drone] ||
          !step.positions[drone].allFinite() ||
          !step.velocities[drone].allFinite())
        return false;
    return true;
  }

  void publishReferences(const multi_uav_formation::ViewpointStep &step,
                         const ros::Time &stamp)
  {
    const std::uint32_t generation = ++generation_;
    for (int index = 0; index < 3; ++index)
    {
      nav_msgs::Odometry reference;
      reference.header.seq = generation;
      reference.header.stamp = stamp;
      reference.header.frame_id = frame_id_;
      reference.child_frame_id = "cooperative_soft_encirclement_reference";
      reference.pose.pose.position.x = step.positions[index].x();
      reference.pose.pose.position.y = step.positions[index].y();
      reference.pose.pose.position.z = step.positions[index].z();
      reference.pose.pose.orientation.w =
          std::cos(0.5 * step.commanded_angles[index]);
      reference.pose.pose.orientation.z =
          std::sin(0.5 * step.commanded_angles[index]);
      reference.pose.covariance[0] = 0.0;
      reference.twist.twist.linear.x = step.velocities[index].x();
      reference.twist.twist.linear.y = step.velocities[index].y();
      reference.twist.twist.linear.z = step.velocities[index].z();
      reference.twist.twist.angular.z = step.angular_rates[index];
      reference_pubs_[index].publish(reference);
    }
  }

  void publishMarkers(const multi_uav_formation::ViewpointStep &step,
                      const ros::Time &stamp)
  {
    visualization_msgs::MarkerArray array;
    const std::array<std::array<float, 3>, 3> colors{{
        {{0.2f, 0.8f, 1.0f}}, {{0.3f, 1.0f, 0.3f}},
        {{1.0f, 0.6f, 0.2f}}}};
    for (int index = 0; index < 3; ++index)
    {
      visualization_msgs::Marker marker;
      marker.header.stamp = stamp;
      marker.header.frame_id = frame_id_;
      marker.ns = "cooperative_soft_reference";
      marker.id = index;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = step.positions[index].x();
      marker.pose.position.y = step.positions[index].y();
      marker.pose.position.z = step.positions[index].z();
      marker.pose.orientation.w = 1.0;
      marker.scale.x = marker.scale.y = marker.scale.z = 0.20;
      marker.color.r = colors[index][0];
      marker.color.g = colors[index][1];
      marker.color.b = colors[index][2];
      marker.color.a = 0.9;
      marker.lifetime = ros::Duration(0.3);
      array.markers.push_back(marker);
    }
    marker_pub_.publish(array);
  }

  void adjustmentCallback(
      const traj_utils::TaskReferenceAdjustmentConstPtr &message)
  {
    if (!message || !std::isfinite(message->recommended_phi0))
      return;
    // Team 只是建议者：core 内部执行 rate-limited、带静态复验的转换契约，
    // 本进程不直接改任何几何状态。
    core_.requestJointSelectedPhi0(message->recommended_phi0);
    ROS_INFO(
        "[cooperative-task-reference] adjustment accepted_for_review "
        "recommended_phi0=%.6f reference_generation=%lu "
        "team_transaction_id=%lu reason=%s",
        message->recommended_phi0,
        static_cast<unsigned long>(message->reference_generation),
        static_cast<unsigned long>(message->team_transaction_id),
        message->reason.c_str());
  }

  void publishTaskReference(const multi_uav_formation::ViewpointStep &step,
                            const ros::Time &stamp)
  {
    traj_utils::CooperativeTaskReference reference;
    reference.header.stamp = stamp;
    reference.header.frame_id = frame_id_;
    reference.generation = ++task_reference_generation_;
    reference.valid = true;
    reference.phi0 = step.phi0_current;
    for (int index = 0; index < 3; ++index)
    {
      reference.slot_for_uav[index] = step.slot_assignment[index];
      // 名义观察几何取 target-centered 坐标（与 Team baseline 同一语义）。
      const Eigen::Vector3d delta =
          step.positions[index] - target_position_;
      reference.desired_radius[index] =
          std::max(0.05, delta.head<2>().norm());
      reference.desired_bearing[index] = std::atan2(delta.y(), delta.x());
      reference.desired_height[index] = delta.z();
    }
    reference.reason =
        multi_uav_formation::viewpointStateName(step.state);
    task_reference_pub_.publish(reference);
  }

  void timerCallback(const ros::TimerEvent &)
  {
    const ros::Time now = ros::Time::now();
    if (!have_target_ || (now - target_stamp_).toSec() > target_timeout_)
      return;

    std::array<bool, 3> odom_valid{{false, false, false}};
    auto yaw_states = observer_yaw_states_;
    for (int index = 0; index < 3; ++index)
    {
      odom_valid[index] = have_odom_[index] &&
          (now - odom_stamps_[index]).toSec() <= odom_timeout_;
      yaw_states[index].valid = yaw_states[index].valid && odom_valid[index];
    }
    if (!std::all_of(odom_valid.begin(), odom_valid.end(),
                     [](const bool valid) { return valid; }))
      return;

    core_.setObserverYawStates(yaw_states);
    core_.setObservedVisibleCount(
        observed_visible_count_, !observed_visible_count_stamp_.isZero() &&
        (now - observed_visible_count_stamp_).toSec() <= target_timeout_);
    core_.setDynamicMotionElapsed(
        have_dynamic_motion_start_
            ? std::max(0.0, now.toSec() - dynamic_motion_start_)
            : 0.0);
    const double dt = last_update_.isZero() ? 0.0
                                            : (now - last_update_).toSec();
    last_update_ = now;
    const auto step = core_.step(target_position_, target_velocity_,
                                 odom_positions_, odom_valid,
                                 now.toSec(), dt);
    if (!publishable(step))
    {
      ROS_WARN_THROTTLE(1.0,
          "[cooperative-soft-reference] publish=0 reason=INVALID_REFERENCE");
      return;
    }
    publishReferences(step, now);
    publishMarkers(step, now);
    publishTaskReference(step, now);
    ROS_INFO_THROTTLE(
        1.0,
        "[cooperative-soft-reference] publish=1 state=%s phi0=%.6f "
        "slot0=%d slot1=%d slot2=%d k2=%.6f mean_visible=%.6f "
        "all3_metric=%.6f blackout_metric=%.6f",
        multi_uav_formation::viewpointStateName(step.state),
        step.phi0_current, step.slot_assignment[0], step.slot_assignment[1],
        step.slot_assignment[2], step.metrics.atleast2,
        step.metrics.mean_visible_count, step.metrics.all3,
        step.metrics.none);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber target_sub_;
  ros::Subscriber motion_start_sub_;
  ros::Subscriber visible_count_sub_;
  std::array<ros::Subscriber, 3> odom_subs_;
  std::array<ros::Publisher, 3> reference_pubs_;
  ros::Publisher marker_pub_;
  ros::Publisher task_reference_pub_;
  ros::Subscriber adjustment_sub_;
  ros::Timer timer_;
  multi_uav_formation::CooperativeViewpointCore core_;

  double rate_hz_{10.0};
  double horizon_{2.0};
  double sample_dt_{0.1};
  double min_range_{0.2};
  double max_range_{8.0};
  double los_margin_{0.08};
  double reference_clearance_{0.0};
  double visibility_bad_hold_{0.5};
  double trigger_tracking_error_{0.5};
  double theta_sep_min_deg_{25.0};
  double min_visibility_improvement_{0.10};
  double k2_near_best_tolerance_{0.01};
  double max_angular_rate_{0.25};
  double arrive_distance_{0.5};
  double arrive_hold_{0.5};
  double target_timeout_{0.5};
  double odom_timeout_{0.5};
  double camera_hfov_deg_{85.0};
  double camera_image_width_{1280.0};
  double camera_image_height_{720.0};
  double camera_extrinsic_x_{0.0};
  double camera_extrinsic_y_{0.0};
  double camera_extrinsic_z_{0.0};
  double camera_extrinsic_roll_{0.0};
  double camera_extrinsic_pitch_{0.0};
  double camera_extrinsic_yaw_{0.0};
  std::string target_topic_{"/object_odom"};
  std::string scene_file_;
  std::string frame_id_{"world"};

  Eigen::Vector3d target_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d target_velocity_{Eigen::Vector3d::Zero()};
  std::array<Eigen::Vector3d, 3> odom_positions_;
  std::array<ros::Time, 3> odom_stamps_;
  std::array<bool, 3> have_odom_{{false, false, false}};
  std::array<multi_uav_formation::PredictedYawState, 3>
      observer_yaw_states_;
  ros::Time target_stamp_;
  ros::Time observed_visible_count_stamp_;
  ros::Time last_update_;
  bool have_target_{false};
  bool have_dynamic_motion_start_{false};
  double dynamic_motion_start_{0.0};
  int observed_visible_count_{0};
  std::atomic<std::uint32_t> generation_{0};
  std::uint64_t task_reference_generation_{0};
};

}  // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cooperative_viewpoint_manager");
  CooperativeViewpointManager manager;
  ros::spin();
  return 0;
}
