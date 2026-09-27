#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <optimizer/poly_traj_optimizer.h>
#include <plan_env/grid_map.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/GoalSet.h>
#include <traj_utils/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/MINCOTraj.h>

using std::vector;

namespace ego_planner
{

class EGOReplanFSM
{
public:
  EGOReplanFSM() {}
  ~EGOReplanFSM();

  void init(ros::NodeHandle &nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  /* ---------- flag ---------- */
  enum FSM_EXEC_STATE
  {
    INIT,
    WAIT_TARGET,
    GEN_NEW_TRAJ,
    REPLAN_TRAJ,
    EXEC_TRAJ,
    EMERGENCY_STOP,
    SEQUENTIAL_START
  };
  // A published successor that is still waiting for executor activation is NOT
  // a completed replan.  Conflating the two makes the FSM treat "nothing new
  // was planned" as success and re-enter REPLAN_TRAJ inside the same timer
  // tick, re-running the whole MINCO/candidate pipeline in a tight loop.
  enum class ReplanOutcome
  {
    SUCCESS,
    PENDING,
    FAILED
  };
  enum TARGET_TYPE
  {
    MANUAL_TARGET = 1,
    PRESET_TARGET = 2,
    REFENCE_PATH = 3
  };

  /* planning utils */
  EGOPlannerManager::Ptr planner_manager_;
  PlanningVisualization::Ptr visualization_;
  traj_utils::DataDisp data_disp_;

  /* parameters */
  int target_type_; // 1 mannual select, 2 hard code
  double no_replan_thresh_, replan_thresh_;
  double tracking_error_replan_thresh_, tracking_velocity_error_replan_thresh_;
  double waypoints_[50][3];
  int waypoint_num_, wpt_id_;
  double planning_horizen_;
  double emergency_time_;
  double preset_start_delay_step_{0.0};
  bool flag_realworld_experiment_;
  bool enable_fail_safe_;
    bool enable_ground_height_measurement_; 
  bool flag_escape_emergency_;
  Eigen::Vector3d relative_tracking_p_{Eigen::Vector3d(-10000, -10000, -10000)};
  Eigen::Vector3d active_relative_tracking_p_{Eigen::Vector3d(-10000, -10000, -10000)};
  Eigen::Vector3d object_p_{Eigen::Vector3d(-10000, -10000, -10000)};
  Eigen::Vector3d object_v_{Eigen::Vector3d(-10000, -10000, -10000)};
  // Stamp of the smoothed target estimate in object_p_/object_v_.  It is the
  // single authoritative time base for every target prediction.
  double object_stamp_{0.0};
  Eigen::Quaterniond object_q_{Eigen::Quaterniond(1, 0, 0, 0)};
  bool enable_fov_tracking_{false};
  bool fov_lock_tracking_height_{true};
  bool fov_enable_candidate_goal_{false};
  bool fov_enable_visible_safe_region_{true};
  bool fov_enable_goal_safe_projection_{false};
  bool fov_use_los_check_{true};
  bool fov_use_angle_check_{true};
  int fov_candidate_count_{33};
  int fov_frontend_max_waypoints_{4};
  double fov_lookahead_time_{1.0};
  double fov_tracking_distance_{1.2};
  double fov_min_target_distance_{0.7};
  double fov_max_target_distance_{3.0};
  double fov_candidate_angle_span_{3.839724354387525};
  double fov_desired_weight_{1.0};
  double fov_smooth_weight_{0.2};
  double fov_distance_weight_{0.6};
  double fov_half_angle_{0.7853981633974483};
  double fov_weight_{8.0};
  double fov_occlusion_weight_{20.0};
  double fov_los_sample_step_{0.12};
  double fov_target_height_{1.2};
  double fov_safe_region_clearance_{0.12};
  double fov_frontend_grid_step_{0.2};
  double fov_frontend_search_margin_{2.0};
  double fov_frontend_path_weight_{0.3};
  bool fov_enable_soft_retry_{true};
  double fov_soft_retry_scale_{1.0};
  double fov_retry_distance_scale_{1.25};
  double object_velocity_deadband_{0.5};
  bool enable_cooperative_viewpoint_reference_{false};
  double cooperative_viewpoint_reference_timeout_{0.5};
  std::string cooperative_viewpoint_reference_topic_;
  Eigen::Vector3d cooperative_viewpoint_p_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d cooperative_viewpoint_v_{Eigen::Vector3d::Zero()};
  ros::Time cooperative_viewpoint_stamp_;
  bool cooperative_viewpoint_received_{false};
  bool cooperative_viewpoint_valid_{false};
  mutable std::mutex cooperative_viewpoint_mutex_;
  Eigen::Vector3d cooperative_goal_velocity_{Eigen::Vector3d::Zero()};
  bool cooperative_goal_velocity_pending_{false};
  bool mission_started_{false};
  bool pretrack_goal_logged_{false};

  bool have_trigger_, have_target_, have_odom_, have_new_target_, have_recv_pre_agent_, touch_goal_, mandatory_stop_, satrt_tracking_;
  ros::Time preset_start_ready_time_;
  FSM_EXEC_STATE exec_state_;
  int continously_called_times_{0};

  Eigen::Vector3d start_pt_, start_vel_, start_acc_;   // start state
  Eigen::Vector3d end_pt_;                             // goal state
  Eigen::Vector3d local_target_pt_, local_target_vel_; // local target state
  Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;     // odometry state
  bool force_odom_resync_next_replan_{false};
  std::vector<Eigen::Vector3d> wps_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_;
  ros::NodeHandle heartbeat_node_;
  ros::CallbackQueue heartbeat_callback_queue_;
  ros::Timer heartbeat_timer_;
  std::unique_ptr<ros::AsyncSpinner> heartbeat_spinner_;
  ros::NodeHandle cooperative_viewpoint_node_;
  ros::CallbackQueue cooperative_viewpoint_callback_queue_;
  std::unique_ptr<ros::AsyncSpinner> cooperative_viewpoint_spinner_;
  ros::Time heartbeat_last_publish_time_;
  ros::WallTime heartbeat_last_diagnostic_time_;
  double heartbeat_period_sum_{0.0};
  double heartbeat_period_max_{0.0};
  uint64_t heartbeat_period_count_{0};
  ros::Subscriber waypoint_sub_, odom_sub_, trigger_sub_, broadcast_ploytraj_sub_, mandatory_stop_sub_, object_sub_, cooperative_viewpoint_sub_, target_start_sub_;
  ros::Publisher poly_traj_pub_, data_disp_pub_, broadcast_ploytraj_pub_, heartbeat_pub_, goal_pub_, ground_height_pub_;
  // Publication identity guard.  Revalidation/retained-previous planning may
  // revisit the same LocalTrajData; it must not rebroadcast the same
  // trajectory identity.  A changed id or start epoch is a real successor.
  int last_published_traj_id_{-1};
  double last_published_start_time_{-std::numeric_limits<double>::infinity()};
  std::uint64_t duplicate_publication_suppressed_count_{0};

  /* replan cadence / pending-successor bookkeeping */
  // 上一次真正进入 planning pipeline 的 wall time。
  ros::WallTime last_replan_start_wall_;
  std::uint64_t replan_total_count_{0};
  std::uint64_t replan_interval_lt_10ms_count_{0};
  std::uint64_t replan_interval_lt_5ms_count_{0};
  ReplanOutcome last_replan_outcome_{ReplanOutcome::FAILED};
  // 【本轮】同一个 planning tick 内最多一次真实 planning attempt。
  // 该标记在每个 timer tick 进入 execFSMCallback 时复位，
  // 只用于限制同 tick 重复求解，不限制下一个正常 tick。
  bool planning_attempted_this_tick_{false};
  std::uint64_t same_tick_duplicate_replan_suppressed_count_{0};
  // pending 期间仍在规划的次数：证明 pending 不再是 planning 入口 gate。
  std::uint64_t planning_tick_while_local_pending_count_{0};
  std::uint64_t planning_tick_while_joint_pending_count_{0};
  // 逐机 replan 间隔样本（ms），用于 P50/P95/MAX 验收。
  std::vector<double> replan_intervals_ms_;
  ros::WallTime last_replan_cadence_report_wall_;
  bool replan_cadence_final_emitted_{false};

  /* rolling-target progress guard bookkeeping (units: metres / seconds) */
  std::vector<double> rolling_target_extension_distances_;
  std::uint64_t rolling_target_extension_count_{0};
  double rolling_target_extension_time_used_{0.0};
  double rolling_target_extension_distance_max_{0.0};
  ros::WallTime last_rolling_extension_report_wall_;
  bool rolling_extension_final_emitted_{false};

  /* state machine functions */
  void heartbeatTimerCallback(const ros::TimerEvent &e);
  void execFSMCallback(const ros::TimerEvent &e);
  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
  void printFSMExecState();
  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();

  /* safety */
  void checkCollisionCallback(const ros::TimerEvent &e);

  /* local planning */
  ReplanOutcome callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);
  bool planFromGlobalTraj(const int trial_times = 1);
  bool planFromLocalTraj(const int trial_times = 1);
  void recordReplanStart();
  void reportReplanCadence(bool final_report);
  void reportRollingTargetExtension(bool final_report);

  /* global trajectory */
  void waypointCallback(const quadrotor_msgs::GoalSetPtr &msg);
  // void planGlobalTrajbyGivenWps();
  void readGivenWpsAndPlan();
  bool planNextWaypoint(const Eigen::Vector3d next_wp);

  /* input-output */
  void mandatoryStopCallback(const std_msgs::Empty &msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
  void objectCallback(const nav_msgs::OdometryConstPtr &msg);
  void targetStartCallback(const std_msgs::Float64ConstPtr &msg);
  void cooperativeViewpointCallback(const nav_msgs::OdometryConstPtr &msg);
  bool cooperativeViewpointSnapshot(const ros::Time &now,
                                    Eigen::Vector3d &position,
                                    Eigen::Vector3d &velocity,
                                    double &age) const;
  void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
  void RecvBroadcastMINCOTrajCallback(const traj_utils::MINCOTrajConstPtr &msg);
  void polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg);
    void publishCurrentTrajectory(const char *source,
                                  bool publish_to_executor = true);
  Eigen::Vector3d computeFOVRelativeTrackingPoint();
  Eigen::Vector3d computeFOVVisibleSafeGoal(const Eigen::Vector3d &nominal_goal,
                                            const Eigen::Vector3d &target_pred);
  bool fovPointSafe(const Eigen::Vector3d &point, const double clearance);
  bool fovSegmentClear(const Eigen::Vector3d &start, const Eigen::Vector3d &end);
  bool fovProjectToNearestSafePoint(const Eigen::Vector3d &point, Eigen::Vector3d &safe_point,
                                    const bool allow_without_safe_region = false);
  bool fovApplySafePlanningStart(Eigen::Vector3d &start_pt, Eigen::Vector3d &start_vel,
                                 Eigen::Vector3d &start_acc);
  bool fovBuildSafeCorridorWaypoints(const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                                     std::vector<Eigen::Vector3d> &waypoints);
  bool fovLineOfSightClear(const Eigen::Vector3d &observer, const Eigen::Vector3d &target);
  double fovOcclusionCost(const Eigen::Vector3d &observer, const Eigen::Vector3d &target);
  double fovAngleViolation(const Eigen::Vector3d &observer, const Eigen::Vector3d &target,
                           const Eigen::Vector3d &reference_observer);

    /* ground height measurement */
    bool measureGroundHeight(double &height); 
};

} // namespace ego_planner

#endif
