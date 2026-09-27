
#include <plan_manage/ego_replan_fsm.h>
#include <traj_utils/poly_payload_identity.h>

#include <cmath>
#include <algorithm>
#include <limits>
#include <numeric>

namespace ego_planner
{

  EGOReplanFSM::~EGOReplanFSM()
  {
    reportReplanCadence(true);
    reportRollingTargetExtension(true);
    if (planner_manager_)
    {
      planner_manager_->reportTimebaseAndFallbackAudit(true);
      planner_manager_->reportVisibilityTopologyAudit(true);
      planner_manager_->reportSwarmUnlockAudit(true);
    }
    heartbeat_timer_.stop();
    if (heartbeat_spinner_)
      heartbeat_spinner_->stop();
    if (cooperative_viewpoint_spinner_)
      cooperative_viewpoint_spinner_->stop();
  }

  // Replan-cadence instrumentation.  A replan that enters the planning pipeline
  // back-to-back with the previous one within a few milliseconds is the
  // signature of the pending-successor busy loop this round removes.
  void EGOReplanFSM::recordReplanStart()
  {
    const ros::WallTime now = ros::WallTime::now();
    ++replan_total_count_;
    if (!last_replan_start_wall_.isZero())
    {
      const double interval = (now - last_replan_start_wall_).toSec();
      if (std::isfinite(interval) && interval >= 0.0)
      {
        replan_intervals_ms_.push_back(interval * 1000.0);
        if (replan_intervals_ms_.size() > 8192)
          replan_intervals_ms_.erase(replan_intervals_ms_.begin());
        if (interval < 0.010)
          ++replan_interval_lt_10ms_count_;
        if (interval < 0.005)
          ++replan_interval_lt_5ms_count_;
      }
    }
    last_replan_start_wall_ = now;
  }

  void EGOReplanFSM::reportReplanCadence(bool final_report)
  {
    const ros::WallTime now = ros::WallTime::now();
    if (!final_report && !last_replan_cadence_report_wall_.isZero() &&
        (now - last_replan_cadence_report_wall_).toSec() < 5.0)
      return;
    if (final_report)
    {
      if (replan_cadence_final_emitted_)
        return;
      replan_cadence_final_emitted_ = true;
    }
    last_replan_cadence_report_wall_ = now;
    double p50 = 0.0, p95 = 0.0, max_interval = 0.0;
    if (!replan_intervals_ms_.empty())
    {
      std::vector<double> sorted = replan_intervals_ms_;
      std::sort(sorted.begin(), sorted.end());
      const size_t last = sorted.size() - 1;
      p50 = sorted[static_cast<size_t>(std::floor(0.50 * last))];
      p95 = sorted[static_cast<size_t>(std::floor(0.95 * last))];
      max_interval = sorted.back();
    }
    ROS_INFO("[replan-cadence-per-uav] drone=%d REPLAN_TOTAL_COUNT=%lu "
             "REPLAN_INTERVAL_P50_MS=%.3f REPLAN_INTERVAL_P95_MS=%.3f "
             "REPLAN_INTERVAL_MAX_MS=%.3f REPLAN_INTERVAL_LT_10MS_COUNT=%lu "
             "REPLAN_INTERVAL_LT_5MS_COUNT=%lu",
             planner_manager_ ? planner_manager_->pp_.drone_id : -1,
             static_cast<unsigned long>(replan_total_count_), p50, p95,
             max_interval,
             static_cast<unsigned long>(replan_interval_lt_10ms_count_),
             static_cast<unsigned long>(replan_interval_lt_5ms_count_));
    ROS_INFO("[planning-gate-audit] drone=%d "
             "PLANNING_TICK_WHILE_LOCAL_PENDING_COUNT=%lu "
             "PLANNING_TICK_WHILE_JOINT_PENDING_COUNT=%lu "
             "SAME_TICK_DUPLICATE_REPLAN_SUPPRESSED_COUNT=%lu",
             planner_manager_ ? planner_manager_->pp_.drone_id : -1,
             static_cast<unsigned long>(planning_tick_while_local_pending_count_),
             static_cast<unsigned long>(planning_tick_while_joint_pending_count_),
             static_cast<unsigned long>(
                 same_tick_duplicate_replan_suppressed_count_));
  }

  void EGOReplanFSM::reportRollingTargetExtension(bool final_report)
  {
    const ros::WallTime now = ros::WallTime::now();
    if (!final_report && !last_rolling_extension_report_wall_.isZero() &&
        (now - last_rolling_extension_report_wall_).toSec() < 5.0)
      return;
    if (final_report)
    {
      if (rolling_extension_final_emitted_)
        return;
      rolling_extension_final_emitted_ = true;
    }
    last_rolling_extension_report_wall_ = now;
    double p50 = 0.0;
    double p95 = 0.0;
    if (!rolling_target_extension_distances_.empty())
    {
      std::vector<double> sorted = rolling_target_extension_distances_;
      std::sort(sorted.begin(), sorted.end());
      const size_t last = sorted.size() - 1;
      p50 = sorted[static_cast<size_t>(std::floor(0.50 * last))];
      p95 = sorted[static_cast<size_t>(std::floor(0.95 * last))];
    }
    ROS_INFO("[rolling-target-extension] ROLLING_TARGET_EXTENSION_COUNT=%lu "
             "ROLLING_TARGET_EXTENSION_TIME_USED=%.6f EXTENSION_DISTANCE_P50=%.6f "
             "EXTENSION_DISTANCE_P95=%.6f EXTENSION_DISTANCE_MAX=%.6f "
             "MAX_LOCAL_TARGET_FORWARD_JUMP=%.6f",
             static_cast<unsigned long>(rolling_target_extension_count_),
             rolling_target_extension_time_used_, p50, p95,
             rolling_target_extension_distance_max_,
             rolling_target_extension_distance_max_);
  }

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;
    mandatory_stop_ = false;
    satrt_tracking_ = false;
    mission_started_ = false;
    pretrack_goal_logged_ = false;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/preset_start_delay_step", preset_start_delay_step_, 0.0);
    nh.param("fsm/tracking_error_replan_threshold", tracking_error_replan_thresh_, 0.5);
    nh.param("fsm/tracking_velocity_error_replan_threshold", tracking_velocity_error_replan_thresh_, 0.5);
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);
    nh.param("fsm/relative_tracking_x", relative_tracking_p_(0), -10000.0);
    nh.param("fsm/relative_tracking_y", relative_tracking_p_(1), -10000.0);
    nh.param("fsm/relative_tracking_z", relative_tracking_p_(2), -10000.0);
    active_relative_tracking_p_ = relative_tracking_p_;
    nh.param("fsm/ground_height_measurement", enable_ground_height_measurement_, true);
    nh.param("fsm/use_fov_tracking", enable_fov_tracking_, false);
    nh.param("fsm/fov_lock_tracking_height", fov_lock_tracking_height_, true);
    nh.param("fsm/fov_enable_candidate_goal", fov_enable_candidate_goal_, false);
    nh.param("fsm/fov_enable_visible_safe_region", fov_enable_visible_safe_region_, true);
    nh.param("fsm/fov_enable_goal_safe_projection", fov_enable_goal_safe_projection_, false);
    nh.param("fsm/fov_use_los_check", fov_use_los_check_, true);
    nh.param("fsm/fov_use_angle_check", fov_use_angle_check_, true);
    nh.param("fsm/fov_candidate_count", fov_candidate_count_, 33);
    nh.param("fsm/fov_frontend_max_waypoints", fov_frontend_max_waypoints_, 4);
    nh.param("fsm/fov_lookahead_time", fov_lookahead_time_, 1.0);
    nh.param("fsm/fov_tracking_distance", fov_tracking_distance_, std::max(1.0, relative_tracking_p_.head<2>().norm()));
    nh.param("fsm/fov_min_target_distance", fov_min_target_distance_, 0.7);
    nh.param("fsm/fov_max_target_distance", fov_max_target_distance_, 3.0);
    double fov_candidate_angle_span_deg = 220.0;
    nh.param("fsm/fov_candidate_angle_span_deg", fov_candidate_angle_span_deg, 220.0);
    fov_candidate_angle_span_ = fov_candidate_angle_span_deg * M_PI / 180.0;
    nh.param("fsm/fov_desired_weight", fov_desired_weight_, 4.0);
    nh.param("fsm/fov_smooth_weight", fov_smooth_weight_, 0.05);
    nh.param("fsm/fov_distance_weight", fov_distance_weight_, 1.0);
    double fov_half_angle_deg = 45.0;
    nh.param("fsm/fov_half_angle_deg", fov_half_angle_deg, 45.0);
    fov_half_angle_ = std::max(0.0, std::min(179.0, fov_half_angle_deg)) * M_PI / 180.0;
    nh.param("fsm/fov_weight", fov_weight_, 8.0);
    nh.param("fsm/fov_occlusion_weight", fov_occlusion_weight_, 20.0);
    nh.param("fsm/fov_los_sample_step", fov_los_sample_step_, 0.12);
    nh.param("fsm/fov_target_height", fov_target_height_, 1.2);
    nh.param("fsm/fov_safe_region_clearance", fov_safe_region_clearance_, 0.12);
    nh.param("fsm/fov_frontend_grid_step", fov_frontend_grid_step_, 0.2);
    nh.param("fsm/fov_frontend_search_margin", fov_frontend_search_margin_, 2.0);
    nh.param("fsm/fov_frontend_path_weight", fov_frontend_path_weight_, 0.3);
    nh.param("fsm/fov_enable_soft_retry", fov_enable_soft_retry_, true);
    nh.param("fsm/fov_soft_retry_scale", fov_soft_retry_scale_, 1.0);
    fov_soft_retry_scale_ = std::max(0.0, std::min(1.0, fov_soft_retry_scale_));
    nh.param("fsm/fov_retry_distance_scale", fov_retry_distance_scale_, 1.25);
    fov_retry_distance_scale_ = std::max(1.0, std::min(2.0, fov_retry_distance_scale_));
    nh.param("fsm/object_velocity_deadband", object_velocity_deadband_, 0.5);
    nh.param("fsm/enable_cooperative_viewpoint_reference",
             enable_cooperative_viewpoint_reference_, false);
    nh.param("fsm/cooperative_viewpoint_reference_timeout",
             cooperative_viewpoint_reference_timeout_, 0.5);
    nh.param<std::string>("fsm/cooperative_viewpoint_reference_topic",
                          cooperative_viewpoint_reference_topic_, "");

    have_trigger_ = !flag_realworld_experiment_;

    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    heartbeat_node_ = nh;
    heartbeat_node_.setCallbackQueue(&heartbeat_callback_queue_);
    heartbeat_pub_ = heartbeat_node_.advertise<std_msgs::Empty>("planning/heartbeat", 10);
    heartbeat_timer_ = heartbeat_node_.createTimer(
        ros::Duration(0.01), &EGOReplanFSM::heartbeatTimerCallback, this);
    heartbeat_spinner_.reset(new ros::AsyncSpinner(1, &heartbeat_callback_queue_));
    heartbeat_spinner_->start();

    odom_sub_ = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this);
    object_sub_ = nh.subscribe("object", 1, &EGOReplanFSM::objectCallback, this);
    target_start_sub_ = nh.subscribe(
        "/target_tracking/target_start_time", 1,
        &EGOReplanFSM::targetStartCallback, this);
    if (cooperative_viewpoint_reference_topic_.empty())
      cooperative_viewpoint_reference_topic_ =
          "/cooperative_viewpoint/uav" +
          std::to_string(planner_manager_->pp_.drone_id) + "/reference";
    // Keep the lightweight reference callback alive while the main callback
    // queue is occupied by synchronous planning.  This preserves the existing
    // timestamp/validity contract without extending its timeout.
    cooperative_viewpoint_node_ = nh;
    cooperative_viewpoint_node_.setCallbackQueue(
        &cooperative_viewpoint_callback_queue_);
    cooperative_viewpoint_sub_ = cooperative_viewpoint_node_.subscribe(
        cooperative_viewpoint_reference_topic_, 1,
        &EGOReplanFSM::cooperativeViewpointCallback, this,
        ros::TransportHints().tcpNoDelay());
    cooperative_viewpoint_spinner_.reset(
        new ros::AsyncSpinner(1, &cooperative_viewpoint_callback_queue_));
    cooperative_viewpoint_spinner_->start();
    mandatory_stop_sub_ = nh.subscribe("mandatory_stop", 1, &EGOReplanFSM::mandatoryStopCallback, this);

    /* Use MINCO trajectory to minimize the message size in wireless communication */
    broadcast_ploytraj_pub_ = nh.advertise<traj_utils::MINCOTraj>("planning/broadcast_traj_send", 10);
    broadcast_ploytraj_sub_ = nh.subscribe<traj_utils::MINCOTraj>("planning/broadcast_traj_recv", 100,
                                                                  &EGOReplanFSM::RecvBroadcastMINCOTrajCallback,
                                                                  this,
                                                                  ros::TransportHints().tcpNoDelay());

    poly_traj_pub_ = nh.advertise<traj_utils::PolyTraj>("planning/trajectory", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);
    goal_pub_ = nh.advertise<quadrotor_msgs::GoalSet>("/goal", 10);
    ground_height_pub_ = nh.advertise<std_msgs::Float64>("/ground_height_measurement", 10); 

    ROS_INFO("Wait for 2 second.");
    ROS_INFO("[cooperative-viewpoint-planner-contract] drone_id=%d enabled=%d topic=%s timeout=%.3f true_target_preserved=1 safety_pipeline_unchanged=1",
             planner_manager_->pp_.drone_id,
             static_cast<int>(enable_cooperative_viewpoint_reference_),
             cooperative_viewpoint_reference_topic_.c_str(),
             cooperative_viewpoint_reference_timeout_);
    int count = 0;
    while (ros::ok() && count++ < 2000)
    {
      ros::spinOnce();
      ros::Duration(0.001).sleep();
    }

    trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this);
    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = nh.subscribe("/goal", 1, &EGOReplanFSM::waypointCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {

      // ROS_INFO("Waiting for trigger from RC");

      // while (ros::ok() && (!have_odom_ || !have_trigger_))
      // {
      //   ros::spinOnce();
      //   ros::Duration(0.001).sleep();
      // }

      readGivenWpsAndPlan();
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::heartbeatTimerCallback(const ros::TimerEvent &e)
  {
    reportReplanCadence(false);
    reportRollingTargetExtension(false);
    planner_manager_->reportTimebaseAndFallbackAudit(false);
    planner_manager_->reportSwarmUnlockAudit(false);
    planner_manager_->reportVisibilityTopologyAudit(false);
    const ros::Time now = ros::Time::now();
    if (!heartbeat_last_publish_time_.isZero())
    {
      const double period = (now - heartbeat_last_publish_time_).toSec();
      if (std::isfinite(period) && period >= 0.0)
      {
        heartbeat_period_sum_ += period;
        heartbeat_period_max_ = std::max(heartbeat_period_max_, period);
        ++heartbeat_period_count_;
      }
    }
    heartbeat_last_publish_time_ = now;

    std_msgs::Empty heartbeat_msg;
    heartbeat_pub_.publish(heartbeat_msg);

    const ros::WallTime wall_now = ros::WallTime::now();
    if (heartbeat_last_diagnostic_time_.isZero())
      heartbeat_last_diagnostic_time_ = wall_now;
    if ((wall_now - heartbeat_last_diagnostic_time_).toSec() >= 5.0)
    {
      const double mean_period = heartbeat_period_count_ > 0
                                     ? heartbeat_period_sum_ /
                                           static_cast<double>(heartbeat_period_count_)
                                     : 0.0;
      ROS_INFO("[planner-heartbeat] node=%s callback_queue=dedicated "
               "period_samples=%llu period_mean=%.6f period_max=%.6f",
               ros::this_node::getName().c_str(),
               static_cast<unsigned long long>(heartbeat_period_count_),
               mean_period, heartbeat_period_max_);
      heartbeat_last_diagnostic_time_ = wall_now;
    }
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage

    // 【本轮】每个 timer tick 进入时复位"本 tick 已尝试规划"标记：
    // 本 tick 内最多一次真实 planning attempt，下一个正常 tick 自动允许再次规划。
    planning_attempted_this_tick_ = false;

    const EGOPlannerManager::TopologyProcessStatus topology_status =
        planner_manager_->processPendingTopologyCoordination();
    if (topology_status == EGOPlannerManager::TopologyProcessStatus::COMMITTED)
    {
      if (planner_manager_->teamTrajectorySampleHoldActive(
              ros::Time::now().toSec()))
      {
        // The team commit replaces the candidate that put the FSM in
        // REPLAN_TRAJ.  Return to execution so the next ordinary FSM tick
        // cannot immediately build a successor before the common activation.
        // Safety callbacks remain independent and may still preempt it.
        changeFSMExecState(EXEC_TRAJ, "TEAM_SOLUTION");
      }
      // Executor 已经在 PREPARE 阶段缓存同一份 Local realization，并在
      // coordinator 的 COMMIT 后原子提升。这里 Planner 只更新自己的 rolling
      // state 并向 peer 广播 MINCO；再次发布 PolyTraj 会把同一 team identity
      // 第二次送入本机 executor 队列，破坏 one commit / one activation。
      publishCurrentTrajectory("topology_coordination", false);
      exec_timer_.start();
      return;
    }
    if (topology_status ==
        EGOPlannerManager::TopologyProcessStatus::RETAINED_PREVIOUS)
    {
      exec_timer_.start();
      return;
    }
    else if (topology_status == EGOPlannerManager::TopologyProcessStatus::FAILED)
      changeFSMExecState(REPLAN_TRAJ, "TOPOLOGY_FALLBACK");

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 500)
    {
      fsm_num = 0;
      printFSMExecState();
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return; // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)
        goto force_return; // return;
      if (preset_start_ready_time_.isZero())
        preset_start_ready_time_ = ros::Time::now();
      const double start_delay = std::max(0.0, preset_start_delay_step_) *
                                 std::max(0, planner_manager_->pp_.drone_id);
      if (target_type_ == TARGET_TYPE::PRESET_TARGET && start_delay > 0.0 &&
          (ros::Time::now() - preset_start_ready_time_).toSec() < start_delay)
      {
        ROS_INFO_THROTTLE(1.0, "EGO drone %d waits %.1fs before preset route start.",
                          planner_manager_->pp_.drone_id,
                          start_delay - (ros::Time::now() - preset_start_ready_time_).toSec());
        goto force_return;
      }
      else
      {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm or single drone with drone_id = 0
    {
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        if (planning_attempted_this_tick_)
        {
          ++same_tick_duplicate_replan_suppressed_count_;
          break;
        }
        planning_attempted_this_tick_ = true;
        bool success = planFromGlobalTraj(10); // zx-todo
        if (success)
        {
          changeFSMExecState(EXEC_TRAJ, "FSM");
        }
        else
        {
          ROS_WARN("Failed to generate the first trajectory, keep trying");
          changeFSMExecState(SEQUENTIAL_START, "FSM"); // "changeFSMExecState" must be called each time planned
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {
      if (planning_attempted_this_tick_)
      {
        ++same_tick_duplicate_replan_suppressed_count_;
        break;
      }
      planning_attempted_this_tick_ = true;
      bool success = planFromGlobalTraj(10); // zx-todo
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // "changeFSMExecState" must be called each time planned
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      // 同 tick 保护：本 tick 已经尝试过一次规划就不再重复求解。
      // 这是 busy-loop 的唯一防线，且只在同一个 tick 内生效。
      if (planning_attempted_this_tick_)
      {
        ++same_tick_duplicate_replan_suppressed_count_;
        break;
      }
      planning_attempted_this_tick_ = true;
      const int previous_id=planner_manager_->traj_.local_traj.traj_id;
      const bool success=planFromLocalTraj(1);
      const bool successor=planner_manager_->traj_.local_traj.traj_id!=previous_id;
      if (success && successor)
      {
        changeFSMExecState(EXEC_TRAJ, "MOVING_ROLLING");
        break;
      }
      // 本 tick 的同一 Local Planner 没有安全 successor：executor 继续当前
      // validated predecessor；下一正常 tick 从最新状态重跑同一条主链。
      changeFSMExecState(REPLAN_TRAJ, "MOVING_ROLLING_RETRY");
      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->traj_.local_traj;
      double t_cur = ros::Time::now().toSec() - info->start_time;
      if (t_cur < 0.0)
      {
        ROS_INFO_THROTTLE(
            0.5,
            "EGO drone %d waits %.3f s for coordinated trajectory activation.",
            planner_manager_->pp_.drone_id, -t_cur);
        break;
      }
      t_cur = min(info->duration, t_cur);

      Eigen::Vector3d pos = info->traj.getPos(t_cur);
      const Eigen::Vector3d nominal_vel = info->traj.getVel(t_cur);
      const double tracking_error = (odom_pos_ - pos).norm();
      const double tracking_velocity_error = (odom_vel_ - nominal_vel).norm();
      bool touch_the_goal = ((local_target_pt_ - end_pt_).norm() < 1e-2);

      const bool lifecycle_due=planner_manager_->lifecycleSuccessorDue();
      // Lifecycle/revalidation is state-only.  Republishing the current
      // PolyTraj here every 100 ms reuses the same trajectory_id/start_time
      // and makes every receiver correctly report it as duplicate/out of
      // order.  A real successor is published exactly once at commit/adoption
      // (or by topology coordination); revalidation must not rebroadcast it.
      if (lifecycle_due)
      {
        changeFSMExecState(REPLAN_TRAJ, "LIFECYCLE_RESERVE");
      }
      else if ((tracking_error_replan_thresh_ > 0.0 && tracking_error > tracking_error_replan_thresh_) ||
          (enable_fov_tracking_ && tracking_velocity_error_replan_thresh_ > 0.0 &&
           tracking_velocity_error > tracking_velocity_error_replan_thresh_))
      {
        ROS_WARN_THROTTLE(0.5,
                          "EGO drone %d tracking error pos=%.3f vel=%.3f; replan from measured state.",
                          planner_manager_->pp_.drone_id, tracking_error, tracking_velocity_error);
        changeFSMExecState(REPLAN_TRAJ, "TRACKING");
      }
      else if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
          (wpt_id_ < waypoint_num_ - 1) &&
          (end_pt_ - pos).norm() < no_replan_thresh_)
      {
        wpt_id_++;
        planNextWaypoint(wps_[wpt_id_]);
      }
      else if ((t_cur > info->duration - 1e-2) && touch_the_goal) // local target close to the global target
      {
        have_target_ = false;
        // have_trigger_ = false;

        if (target_type_ == TARGET_TYPE::PRESET_TARGET)
        {
          wpt_id_ = 0;
          planNextWaypoint(wps_[wpt_id_]);
        }

        /* The navigation task completed */
        changeFSMExecState(WAIT_TARGET, "FSM");
        goto force_return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        if (planner_manager_->grid_map_->getInflateOccupancy(end_pt_))
        {
          ROS_ERROR("The goal is in obstacles; retry moving candidates.");
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          goto force_return;
        }
        else
        {
          // pass;
        }
      }
      else if (t_cur > replan_thresh_ ||
               (!touch_the_goal &&
                !planner_manager_->traj_.local_traj.pts_chk.empty() &&
                !planner_manager_->traj_.local_traj.pts_chk.back().empty() &&
                planner_manager_->traj_.local_traj.pts_chk.back().back().first - t_cur < emergency_time_))
      {
        // A scheduled joint tail is not a lease on local rolling authority.
        // Local keeps replanning; a newer local commit invalidates the joint
        // proposal through the frontier-owner CAS contract.
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {
      // External mandatory failsafe only. Never generate a planner stop poly.
      ROS_FATAL_ONCE("[continuous-motion] event=EMERGENCY_STOP external_mandatory_stop=1 run_fail=1");
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;
    exec_timer_.start();
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    // static int last_printed_state = -1, dot_nums = 0;

    // if (exec_state_ != last_printed_state)
    //   dot_nums = 0;
    // else
    //   dot_nums++;

    cout << "\r[FSM]: state: " + state_str[int(exec_state_)];

    // last_printed_state = exec_state_;

    // some warnings
    if (!have_odom_)
    {
      cout << ", waiting for odom";
    }
    if (!have_target_)
    {
      cout << ", waiting for target";
    }
    if (!have_trigger_)
    {
      cout << ", waiting for trigger";
    }
    if (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_)
    {
      cout << ", haven't receive traj from previous drone";
    }

    cout << endl;

    // cout << string(dot_nums, '.');

    // fflush(stdout);
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    // check ground height by the way
    if ( enable_ground_height_measurement_ )
    {
      double height;
      measureGroundHeight(height);
    }

    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;
    double t_cur = ros::Time::now().toSec() - info->start_time;
    PtsChk_t pts_chk = info->pts_chk;

    if (exec_state_ == WAIT_TARGET || info->traj_id <= 0)
      return;
    if (t_cur < 0.0)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      ROS_ERROR("Depth Lost! EMERGENCY_STOP");
      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }

    // bool close_to_the_end_of_safe_segment = (pts_chk.back().back().first - t_cur) < emergency_time_;
    // // bool close_to_goal = (info->traj.getPos(info->duration) - end_pt_).norm() < 1e-5;
    // if (close_to_the_end_of_safe_segment)
    // {
    //   changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    //   return;

    //   // if (!close_to_goal)
    //   // {
    //   //   // ROS_INFO("current position is close to the safe segment end.");
    //   //   changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    //   //   return;
    //   // }
    //   // else
    //   // {
    //   //   double t_step = map->getResolution() / planner_manager_->pp_.max_vel_;
    //   //   for (double t = pts_chk.back().back().first; t < info->duration; t += t_step)
    //   //   {
    //   //     if (map->getInflateOccupancy(info->traj.getPos(t)))
    //   //     {
    //   //       if ((odom_pos_ - end_pt_).norm() < no_replan_thresh_)
    //   //       {
    //   //         ROS_ERROR("Dense obstacles close to the goal, stop planning.");
    //   //         have_target_ = false;
    //   //         changeFSMExecState(WAIT_TARGET, "SAFETY");
    //   //         return;
    //   //       }
    //   //       else
    //   //       {
    //   //         changeFSMExecState(REPLAN_TRAJ, "SAFETY");
    //   //         return;
    //   //       }
    //   //     }
    //   //   }
    //   // }
    // }

    /* ---------- check trajectory ---------- */
    const double CLEARANCE = 0.8 * planner_manager_->getSwarmClearance();
    auto id_ratio = info->traj.locatePieceIdxWithRatio(t_cur);

    // cout << "t_cur=" << t_cur << " info->duration=" << info->duration << endl;

    size_t i_start = floor((id_ratio.first + id_ratio.second) * planner_manager_->getCpsNumPrePiece());
    if (i_start >= pts_chk.size())
    {
      // ROS_ERROR("i_start >= pts_chk.size()");
      return;
    }
    size_t j_start = 0;
    // cout << "i_start=" << i_start << " pts_chk.size()=" << pts_chk.size() << " pts_chk[i_start].size()=" << pts_chk[i_start].size() << endl;
    for (; i_start < pts_chk.size(); ++i_start)
    {
      for (j_start = 0; j_start < pts_chk[i_start].size(); ++j_start)
      {
        if (pts_chk[i_start][j_start].first > t_cur)
        {
          goto find_ij_start;
        }
      }
    }
  find_ij_start:;

    // Eigen::Vector3d last_pt = pts_chk[0][0].second;
    // for (size_t i = 0; i < pts_chk.size(); ++i)
    // {
    //   cout << "--------------------" << endl;
    //   for (size_t j = 0; j < pts_chk[i].size(); ++j)
    //   {
    //     cout << pts_chk[i][j].first << " @ " << pts_chk[i][j].second.transpose() << " @ " << (pts_chk[i][j].second - last_pt).transpose() << " @ " << map->getInflateOccupancy(pts_chk[i][j].second) << endl;
    //     last_pt = pts_chk[i][j].second;
    //   }
    // }

    // cout << "pts_chk[i_start][j_start].first - t_cur = " << pts_chk[i_start][j_start].first - t_cur << endl;
    // cout << "devi = " << (pts_chk[i_start][j_start].second - info->traj.getPos(t_cur)).transpose() << endl;

    // cout << "pts_chk.size()=" << pts_chk.size() << " i_start=" << i_start << endl;
    // Eigen::Vector3d p_last = pts_chk[i_start][j_start].second;
    const bool touch_the_end = ((local_target_pt_ - end_pt_).norm() < 1e-2);
    size_t i_end = touch_the_end ? pts_chk.size() : pts_chk.size() * 3 / 4;
    for (size_t i = i_start; i < i_end; ++i)
    {
      for (size_t j = j_start; j < pts_chk[i].size(); ++j)
      {

        double t = pts_chk[i][j].first;
        Eigen::Vector3d p = pts_chk[i][j].second;
        // if ( (p - p_last).cwiseAbs().maxCoeff() > planner_manager_->grid_map_->getResolution() * 1.05 )
        // {
        //   ROS_ERROR("BBBBBBBBBBBBBBBBBBBBBBBBBBB");
        //   cout << "p=" << p.transpose() << " p_last=" << p_last.transpose() << " dist=" << (p - p_last).cwiseAbs().maxCoeff() << endl;
        // }
        // p_last = p;

        // cout << "t=" << t << " @ "
        //      << "p=" << p.transpose() << endl;
        // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        // if (t_cur < t_2_3 && t >= t_2_3)
        //   break;

        bool dangerous = false;
        dangerous |= map->getInflateOccupancy(p);

        // cout << "p=" << p.transpose() << endl;

        // if (occ)
        // {
        //   ROS_WARN("AAAAAAAAAAAAAAAAAAA");
        //   cout << "pts_chk[i_start].size()=" << pts_chk[i_start].size() << endl;
        //   cout << "i=" << i << " j=" << j << " i_start=" << i_start << " j_start=" << j_start << endl;
        //   cout << "pts_chk.size()=" << pts_chk.size() << endl;
        //   cout << "t=" << t << endl;
        //   cout << "from t=" << info->traj.getPos(t).transpose() << endl;
        //   cout << "from rec=" << p.transpose() << endl;
        // }

        for (size_t id = 0; id < planner_manager_->traj_.swarm_traj.size(); id++)
        {
          if ((planner_manager_->traj_.swarm_traj.at(id).drone_id != (int)id) ||
              (planner_manager_->traj_.swarm_traj.at(id).drone_id == planner_manager_->pp_.drone_id))
          {
            continue;
          }

          double t_X = t + (info->start_time - planner_manager_->traj_.swarm_traj.at(id).start_time);
          if (t_X > 0 && t_X < planner_manager_->traj_.swarm_traj.at(id).duration)
          {
            Eigen::Vector3d swarm_pridicted = planner_manager_->traj_.swarm_traj.at(id).traj.getPos(t_X);
            double dist = (p - swarm_pridicted).norm();

            if (dist < CLEARANCE)
            {
              ROS_WARN("swarm distance between drone %d and drone %d is %f, too close!",
                       planner_manager_->pp_.drone_id, (int)id, dist);
              dangerous = true;
              break;
            }
          }
        }

        if (dangerous)
        {
          /* Handle the collided case immediately */
          if (planFromLocalTraj()) // Make a chance
          {
            ROS_INFO("Plan success when detect collision. %f", t / info->duration);
            changeFSMExecState(EXEC_TRAJ, "SAFETY");
            return;
          }
          else
          {
            ROS_WARN("Current trajectory risk; urgent moving replan, no stop fallback.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
            return;
          }
          break;
        }
      }
      j_start = 0;
    }
  }



  void EGOReplanFSM::publishCurrentTrajectory(const char *source,
                                               const bool publish_to_executor)
  {
    traj_utils::PolyTraj poly_msg;
    traj_utils::MINCOTraj minco_msg;
    polyTraj2ROSMsg(poly_msg, minco_msg);
    const double identity_start = poly_msg.start_time.toSec();
    // Feedback093: this guard tracks what the EXECUTOR has actually been given,
    // not what this planner has merely committed locally.
    //
    // A Team 2PC adoption deliberately skips the direct publish (the executor
    // promotes the PREPARE-cached realization through the Team transaction), but
    // the bookkeeping below used to be updated anyway.  The guaranteed Local
    // successor then looked "already published" and its real delivery was
    // suppressed, so the executor ran out of coverage while the planner still
    // held a perfectly valid successor.  Real production evidence:
    //   [planner-traj-publish] drone_id=2 trajectory_id=27
    //     executor_publish=0 reason=EXECUTOR_2PC_ALREADY_COMMITTED
    //   -> 0 executor RECEIVE for that identity
    //   -> TERMINAL_HOLD on the previous trajectory 0.6 s later.
    // A Local guaranteed successor must always reach the executor.
    if (publish_to_executor &&
        poly_msg.traj_id == last_published_traj_id_ &&
        std::isfinite(identity_start) &&
        std::abs(identity_start - last_published_start_time_) <= 1.0e-9)
    {
      ++duplicate_publication_suppressed_count_;
      ROS_INFO_THROTTLE(1.0,
                        "[planner-traj-publish] suppressed duplicate identity "
                        "drone_id=%d trajectory_id=%d start_time=%.9f source=%s count=%llu",
                        planner_manager_->pp_.drone_id, poly_msg.traj_id,
                        identity_start, source ? source : "unknown",
                        static_cast<unsigned long long>(duplicate_publication_suppressed_count_));
      return;
    }
    if (publish_to_executor)
    {
      last_published_traj_id_ = poly_msg.traj_id;
      last_published_start_time_ = identity_start;
    }
    const double duration =
        std::accumulate(poly_msg.duration.begin(), poly_msg.duration.end(), 0.0);
    ROS_INFO("[planner-traj-publish] drone_id=%d trajectory_id=%d "
             "publish_time=%.9f start_time=%.9f duration=%.6f piece_num=%zu "
             "source=%s",
             planner_manager_->pp_.drone_id, poly_msg.traj_id,
             ros::Time::now().toSec(), poly_msg.start_time.toSec(), duration,
             poly_msg.duration.size(), source ? source : "unknown");
    ROS_INFO("[active-traj-lifecycle] event=PUBLISHED "
             "traj_identity=start=%.9f,trajectory_id=%d source=%s "
             "start_time=%.9f duration=%.6f active=1 reason=message_published",
             poly_msg.start_time.toSec(), poly_msg.traj_id,
             source ? source : "planner_publish", poly_msg.start_time.toSec(),
             duration);
    if (publish_to_executor)
      poly_traj_pub_.publish(poly_msg);
    else
      ROS_INFO("[planner-traj-publish] drone_id=%d trajectory_id=%d "
               "executor_publish=0 reason=EXECUTOR_2PC_ALREADY_COMMITTED",
               planner_manager_->pp_.drone_id, poly_msg.traj_id);
    broadcast_ploytraj_pub_.publish(minco_msg);
  }

  EGOReplanFSM::ReplanOutcome EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {
    // 【本轮修复】这里曾经用 localActivationPending() 作为 planning 入口 gate：
    // 只要执行器还没回报 adoption，后续每个 timer tick 都会在入口被跳过，
    // 等价于"有 pending 就停止 rolling replanning"。
    // 正确语义只有一句：限制同一个 planning tick 内的重复求解，
    // 绝不限制下一个正常 planning tick 继续规划。
    // 因此 pending 不再是入口条件；pending 期间照常产生候选，
    // 若新候选通过现有 predecessor P/V/A、generation、current revision、
    // hard preflight、activation timing 检查，就走现有 supersede 路径替换旧 future。
    // localActivationPending() 现在只用于状态/遥测（下面这条审计计数）。
    if (planner_manager_->localActivationPending())
    {
      ++planning_tick_while_local_pending_count_;
      ROS_INFO_THROTTLE(1.0,
                        "[planning-gate] drone=%d active_trajectory_id=%d "
                        "local_pending=1 action=PLAN_ANYWAY "
                        "count=%lu",
                        planner_manager_->pp_.drone_id,
                        planner_manager_->traj_.local_traj.traj_id,
                        static_cast<unsigned long>(
                            planning_tick_while_local_pending_count_));
    }
    if (planner_manager_->topologyCoordinationPending())
      ++planning_tick_while_joint_pending_count_;
    recordReplanStart();
    planner_manager_->ensureExecutionCoverage();
    struct BatchObservation {
      EGOPlannerManager *manager; double begin;
      ~BatchObservation() { manager->finishPlanningBatch(begin); }
    } observation{planner_manager_.get(),ros::WallTime::now().toSec()};
    if(planner_manager_->currentStateRestartActive())
    {
      // The expired polynomial no longer owns either the planning deadline or
      // the start state.  Reuse the production global-replan authority:
      // measured odometry P/V and the existing zero-acceleration convention.
      start_pt_=odom_pos_;
      start_vel_=odom_vel_;
      start_acc_.setZero();
      fovApplySafePlanningStart(start_pt_,start_vel_,start_acc_);
      ROS_WARN("[current-state-restart] event=START_AUTHORITY drone=%d source=ODOMETRY_PV_ZERO_ACCELERATION p=(%.6f,%.6f,%.6f) v=(%.6f,%.6f,%.6f) a=(%.6f,%.6f,%.6f)",
          planner_manager_->pp_.drone_id,start_pt_.x(),start_pt_.y(),start_pt_.z(),
          start_vel_.x(),start_vel_.y(),start_vel_.z(),start_acc_.x(),start_acc_.y(),start_acc_.z());
    }
    planner_manager_->prepareFutureActivation(start_pt_, start_vel_, start_acc_);
    ROS_INFO("[rolling-replan] event=PLANNING_START drone=%d timestamp=%.9f pending=%d",
        planner_manager_->pp_.drone_id, ros::Time::now().toSec(), int(planner_manager_->topologyCoordinationPending()));

	    planner_manager_->getLocalTarget(
	        planning_horizen_, start_pt_, end_pt_,
	        local_target_pt_, local_target_vel_,
	        touch_goal_);

        // getLocalTarget reaches the end of the current global rolling
        // segment on every moving-target update.  That local endpoint is not
        // mission termination: only non-tracking plans may request full-tail
        // terminal admission.  Keep the endpoint and target velocity intact,
        // but bound all rolling hard checks to the shared execution horizon.
        if (satrt_tracking_ && touch_goal_)
        {
          ROS_INFO_THROTTLE(
              1.0,
              "[rolling-target-semantics] drone=%d local_endpoint=1 "
              "mission_terminal=0 action=BOUNDED_AUTHORITY_HORIZON",
              planner_manager_->pp_.drone_id);
          touch_goal_ = false;
        }

        // A rolling target callback can occasionally deliver the endpoint of
        // a just-finished global segment at the current UAV position.  Do not
        // turn that zero-progress sample into a sub-second executable
        // trajectory: continue along the already authoritative target
        // velocity for one normal rolling horizon.  This is target prediction
        // only; it does not alter mission-terminal semantics or add a safety
        // gate.
        //
        // UNITS: planning_horizen_ is a SPATIAL lookahead in metres (7.5 m in
        // this scene; getLocalTarget() walks the global trajectory until the
        // point is that far from the UAV).  It must never be multiplied by a
        // velocity.  The forward extension is a TIME prediction, so it uses the
        // planner's own target-prediction horizon in seconds.
        if (satrt_tracking_ &&
            (local_target_pt_ - start_pt_).norm() < 0.5)
        {
          // getLocalTarget may report zero velocity at the end of its current
          // global segment even though the tracked object is still moving.
          // In that case the object estimate is the authoritative rolling
          // prediction velocity; a zero vector must not disable this guard.
          Eigen::Vector3d progress_velocity = local_target_vel_;
          const char *velocity_source = "LOCAL_TARGET";
          if (!progress_velocity.allFinite() ||
              progress_velocity.norm() <= 1.0e-3)
          {
            progress_velocity = object_v_;
            velocity_source = "TRACKED_OBJECT";
          }
          if (progress_velocity.allFinite() &&
              progress_velocity.norm() > 1.0e-3)
          {
          // prediction_horizon_time_s is exactly the horizon the moving-object
          // predictor owns (2.0 s in this scenario), never the 7.5 m spatial
          // lookahead.  The fallback is a named seconds constant.
          constexpr double kRollingPredictionHorizonTimeFallbackS = 1.0;
          double prediction_horizon_time_s =
              planner_manager_->movingObjectPredictionHorizonTime();
          if (!std::isfinite(prediction_horizon_time_s) ||
              prediction_horizon_time_s <= 0.0)
            prediction_horizon_time_s = kRollingPredictionHorizonTimeFallbackS;
          const double planning_horizon_distance_m = planning_horizen_;
          const double extension_time_s = prediction_horizon_time_s;
          const Eigen::Vector3d original_local_target = local_target_pt_;
          local_target_pt_ += progress_velocity * extension_time_s;
          local_target_vel_ = progress_velocity;
          const double extension_distance_m =
              (local_target_pt_ - original_local_target).norm();
          ++rolling_target_extension_count_;
          rolling_target_extension_time_used_ = extension_time_s;
          rolling_target_extension_distance_max_ =
              std::max(rolling_target_extension_distance_max_, extension_distance_m);
          rolling_target_extension_distances_.push_back(extension_distance_m);
          if (rolling_target_extension_distances_.size() > 4096)
            rolling_target_extension_distances_.erase(
                rolling_target_extension_distances_.begin());
          ROS_WARN("[rolling-target-progress-guard] drone=%d distance=%.6f "
                   "extension_time_s=%.6f extension_distance_m=%.6f "
                   "planning_horizon_distance_m=%.6f velocity_source=%s "
                   "original=(%.6f,%.6f,%.6f) "
                   "extended=(%.6f,%.6f,%.6f)",
                   planner_manager_->pp_.drone_id,
                   (original_local_target - start_pt_).norm(), extension_time_s,
                   extension_distance_m, planning_horizon_distance_m,
                   velocity_source,
                   original_local_target.x(), original_local_target.y(),
                   original_local_target.z(), local_target_pt_.x(),
                   local_target_pt_.y(), local_target_pt_.z());
          }
        }

	    // Keep the normal rolling target authoritative.  The cooperative
	    // viewpoint is one soft reference consumed by this same Local Planner;
	    // it never becomes a second endpoint or execution authority.
	    const Eigen::Vector3d normal_local_target = local_target_pt_;
	    const Eigen::Vector3d normal_local_target_vel = local_target_vel_;
	    Eigen::Vector3d replan_relative_tracking = active_relative_tracking_p_;

	    auto try_replan = [&](const Eigen::Vector3d &relative_tracking, const double fov_soft_scale) {
	      planner_manager_->setFovSoftScale(fov_soft_scale);
	      return planner_manager_->reboundReplan(
	          start_pt_, start_vel_, start_acc_,
	          local_target_pt_, local_target_vel_,
	          object_p_, object_v_, object_q_, relative_tracking,
	          (have_new_target_ || flag_use_poly_init),
	          flag_randomPolyTraj, touch_goal_, satrt_tracking_,
              tracking_error_replan_thresh_);
	    };

        const auto consume_stale_head_replan = [&]() {
          Eigen::Vector3d fresh_odom_pos;
          Eigen::Vector3d fresh_odom_vel;
          if (!planner_manager_->consumeStaleHeadReplanRequest(fresh_odom_pos,
                                                               fresh_odom_vel))
            return false;
          odom_pos_ = fresh_odom_pos;
          odom_vel_ = fresh_odom_vel;
          odom_acc_.setZero();
          force_odom_resync_next_replan_ = true;
          ROS_WARN("EGO drone %d rejected stale trajectory head; next replan starts from odom.",
                   planner_manager_->pp_.drone_id);
          return true;
        };

        // Local first-safe owns rolling execution. Joint planning is a
        // background quality pass over the already committed future prefix;
        // failure, timeout, or stale input must be a local no-op.
        auto finish_captured = [&](std::vector<EGOPlannerManager::CandidateSetOutput> &sets) {
              int selected_set=-1;
              // Never reserve or defer local authority for a team operation.
              auto status = planner_manager_->finalizeCapturedCandidates(
                  sets,touch_goal_,tracking_error_replan_thresh_,selected_set);
              have_new_target_ = false;
              const bool success =
                  status == EGOPlannerManager::TopologyProcessStatus::COMMITTED ||
                  status == EGOPlannerManager::TopologyProcessStatus::RETAINED_PREVIOUS;
              if (success)
              {
                publishCurrentTrajectory("local_first_safe_direct");
                ROS_INFO("[committed-prefix-joint] event=LOCAL_DIRECT_COMMIT "
                         "drone=%d local_candidate=%d local_gate_by_joint=0",
                         planner_manager_->pp_.drone_id,
                         selected_set >= 0 ? sets[selected_set].local_candidate_id : -1);
                // Export a snapshot only after the local hard-safe trajectory
                // has become the committed execution authority.
                if (selected_set >= 0 &&
                    planner_manager_->jointTopologyCoordinationEnabled())
                {
                  planner_manager_->stageCapturedTopologyCoordination(
                      sets, sets[selected_set].hypothesis_id, touch_goal_,
                      tracking_error_replan_thresh_);
                }
              }
              return success ? ReplanOutcome::SUCCESS : ReplanOutcome::FAILED;
        };
        if (planner_manager_->jointTopologyCoordinationEnabled())
        {
          // Team P/T consumes exactly the same already-safe Local candidate
          // set that would otherwise commit directly.  Cooperative geometry
          // is the single soft reference upstream; there is no second
          // viewpoint-hypothesis endpoint authority.
          EGOPlannerManager::CandidateSetOutput output;
          planner_manager_->setFovSoftScale(1.0);
          const bool generated = planner_manager_->reboundReplan(
              start_pt_, start_vel_, start_acc_, normal_local_target,
              normal_local_target_vel, object_p_, object_v_, object_q_,
              active_relative_tracking_p_,
              (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj,
              touch_goal_, satrt_tracking_, tracking_error_replan_thresh_,
              &output, 0, 0.0, 0);
          if (generated && output.generated)
          {
            std::vector<EGOPlannerManager::CandidateSetOutput> sets;
            sets.push_back(std::move(output));
            const ReplanOutcome captured_outcome = finish_captured(sets);
            last_replan_outcome_ = captured_outcome;
            return captured_outcome;
          }
        }

	    bool plan_success = try_replan(replan_relative_tracking, 1.0);

        if (!plan_success && consume_stale_head_replan())
          return ReplanOutcome::FAILED;

    /* Feedback126 §6/§9: the optional FOV/quality whole-batch retry is
     * removed from production.  A failed batch keeps the hard-safe
     * incumbent (last_hard_safe_incumbent semantics) and the next rolling
     * tick replans normally; quality/FOV must never re-run the whole batch
     * with a mutated target or a different soft scale. */

    have_new_target_ = false;

    if (plan_success)
    {
      publishCurrentTrajectory("planner_publish");
      last_replan_outcome_ = ReplanOutcome::SUCCESS;
      return ReplanOutcome::SUCCESS;
    }

    last_replan_outcome_ = ReplanOutcome::FAILED;
    return ReplanOutcome::FAILED;
  }

  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) //zx-todo
  {

    force_odom_resync_next_replan_ = false;

    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();
    fovApplySafePlanningStart(start_pt_, start_vel_, start_acc_);

    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
      flag_random_poly_init = false;
    else
      flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++)
    {
      const ReplanOutcome outcome = callReboundReplan(true, flag_random_poly_init);
      if (outcome == ReplanOutcome::SUCCESS)
      {
        return true;
      }
      if (outcome == ReplanOutcome::PENDING || force_odom_resync_next_replan_)
        return false;
    }
    return false;
  }

  bool EGOReplanFSM::planFromLocalTraj(const int trial_times /*=1*/)
  {

    if (force_odom_resync_next_replan_)
    {
      ROS_WARN_THROTTLE(0.5,
                        "EGO drone %d performs requested stale-head replan from odom.",
                        planner_manager_->pp_.drone_id);
      return planFromGlobalTraj(std::max(1, trial_times));
    }

    LocalTrajData *info = &planner_manager_->traj_.local_traj;
    double t_cur = ros::Time::now().toSec() - info->start_time;

    if (t_cur < 0.0)
    {
      ROS_INFO_THROTTLE(
          0.5,
          "EGO drone %d keeps scheduled coordinated trajectory before activation.",
          planner_manager_->pp_.drone_id);
      return true;
    }

    if (info->traj_id <= 0 || t_cur > info->duration - 1e-2)
    {
      ROS_WARN_THROTTLE(1.0,
                        "EGO drone %d local trajectory expired, replan from odom.",
                        planner_manager_->pp_.drone_id);
      return planFromGlobalTraj(std::max(1, trial_times));
    }

    start_pt_ = info->traj.getPos(t_cur);
    start_vel_ = info->traj.getVel(t_cur);
    start_acc_ = info->traj.getAcc(t_cur);
    const double tracking_error = (odom_pos_ - start_pt_).norm();
    const double tracking_velocity_error = (odom_vel_ - start_vel_).norm();
    if ((tracking_error_replan_thresh_ > 0.0 && tracking_error > tracking_error_replan_thresh_) ||
        (enable_fov_tracking_ && tracking_velocity_error_replan_thresh_ > 0.0 &&
         tracking_velocity_error > tracking_velocity_error_replan_thresh_))
    {
      ROS_WARN_THROTTLE(0.5,
                        "EGO drone %d replans from odom due to tracking error pos=%.3f vel=%.3f.",
                        planner_manager_->pp_.drone_id, tracking_error, tracking_velocity_error);
      return planFromGlobalTraj(std::max(1, trial_times));
    }
    fovApplySafePlanningStart(start_pt_, start_vel_, start_acc_);

    ReplanOutcome outcome = callReboundReplan(false, false);

    if (outcome == ReplanOutcome::PENDING)
      return false;
    if (outcome != ReplanOutcome::SUCCESS)
    {
      if (force_odom_resync_next_replan_)
        return false;
      outcome = callReboundReplan(true, false);
      if (outcome == ReplanOutcome::PENDING)
        return false;
      if (outcome != ReplanOutcome::SUCCESS)
      {
        if (force_odom_resync_next_replan_)
          return false;
        for (int i = 0; i < trial_times; i++)
        {
          outcome = callReboundReplan(true, true);
          if (outcome == ReplanOutcome::SUCCESS)
            break;
          if (outcome == ReplanOutcome::PENDING ||
              force_odom_resync_next_replan_)
            return false;
        }
        if (outcome != ReplanOutcome::SUCCESS)
        {
          if (enable_fov_tracking_)
          {
            ROS_WARN_THROTTLE(1.0,
                              "EGO drone %d local replan failed, retry from odom.",
                              planner_manager_->pp_.drone_id);
            return planFromGlobalTraj(std::max(1, trial_times));
          }
          return false;
        }
      }
    }

    return true;
  }

  bool EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    bool success = false;
    std::vector<Eigen::Vector3d> one_pt_wps;
    if (enable_fov_tracking_ && fov_enable_visible_safe_region_ && fov_lock_tracking_height_)
    {
      std::vector<Eigen::Vector3d> corridor_wps;
      if (fovBuildSafeCorridorWaypoints(odom_pos_, next_wp, corridor_wps) && !corridor_wps.empty())
      {
        const double base_min_progress = std::max(0.45, 2.0 * fov_frontend_grid_step_);
        const double goal_distance = (next_wp - odom_pos_).norm();
        const double min_progress =
            goal_distance > 2.5 ? std::max(base_min_progress, std::min(1.2, 0.25 * goal_distance))
                                : base_min_progress;
        size_t first_idx = 0;
        while (first_idx + 1 < corridor_wps.size() &&
               (corridor_wps[first_idx] - odom_pos_).norm() < min_progress)
          ++first_idx;

        const size_t last_idx = corridor_wps.size() - 1;
        const size_t max_wps = std::max(1, fov_frontend_max_waypoints_);
        one_pt_wps.push_back(corridor_wps[first_idx]);
        if (max_wps > 1 && first_idx < last_idx)
        {
          size_t last_added = first_idx;
          for (size_t slot = 1; slot < max_wps && last_added < last_idx; ++slot)
          {
            const double alpha = static_cast<double>(slot) / static_cast<double>(max_wps - 1);
            size_t idx = first_idx + static_cast<size_t>(
                                         std::round(alpha * static_cast<double>(last_idx - first_idx)));
            idx = std::max(idx, last_added + 1);
            idx = std::min(idx, last_idx);
            one_pt_wps.push_back(corridor_wps[idx]);
            last_added = idx;
          }
        }

	        const Eigen::Vector3d local_corridor_goal = one_pt_wps.front();
	        Eigen::Vector3d target_pred = object_p_ + object_v_ * std::max(0.0, fov_lookahead_time_);
	        if (fov_lock_tracking_height_)
	          target_pred(2) = fov_target_height_;
	        active_relative_tracking_p_ = object_q_.normalized().inverse() * (next_wp - target_pred);
	        if (fov_lock_tracking_height_)
	          active_relative_tracking_p_(2) = relative_tracking_p_(2);
        ROS_INFO_THROTTLE(1.0,
                          "EGO-FOV drone %d uses %zu corridor waypoints, local=(%.2f %.2f %.2f), final=(%.2f %.2f %.2f)",
                          planner_manager_->pp_.drone_id,
                          one_pt_wps.size(),
                          local_corridor_goal(0), local_corridor_goal(1), local_corridor_goal(2),
                          one_pt_wps.back()(0), one_pt_wps.back()(1), one_pt_wps.back()(2));
      }
    }

    if (one_pt_wps.empty())
    {
      Eigen::Vector3d fallback_wp = next_wp;
      Eigen::Vector3d safe_wp;
      if (enable_fov_tracking_ && fov_enable_visible_safe_region_ &&
          fovProjectToNearestSafePoint(next_wp, safe_wp) &&
          (safe_wp - next_wp).norm() > 1e-3)
      {
        ROS_WARN_THROTTLE(1.0,
                          "EGO-FOV drone %d projects unsafe waypoint by %.2f m.",
                          planner_manager_->pp_.drone_id, (safe_wp - next_wp).norm());
        fallback_wp = safe_wp;
      }
      one_pt_wps.push_back(fallback_wp);
    }

    Eigen::Vector3d global_start = odom_pos_;
    Eigen::Vector3d global_start_vel = odom_vel_;
    Eigen::Vector3d global_start_acc = Eigen::Vector3d::Zero();
	    fovApplySafePlanningStart(global_start, global_start_vel, global_start_acc);
	    const Eigen::Vector3d goal_velocity = cooperative_goal_velocity_pending_ ?
	        cooperative_goal_velocity_ : (satrt_tracking_ ? object_v_ : Eigen::Vector3d::Zero());
	    success = planner_manager_->planGlobalTrajWaypoints(
	        global_start, global_start_vel, global_start_acc,
	        one_pt_wps, goal_velocity, Eigen::Vector3d::Zero());

    // visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      end_pt_ = one_pt_wps.back();

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_.global_traj.duration / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->traj_.global_traj.traj.getPos(i * step_size_t);
      }

      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ != WAIT_TARGET)
      {
        // Target callbacks only request the next rolling solve. This applies
        // to both tracking objective modes; never recursively spin/wait for
        // EXEC_TRAJ while a failed moving solve needs the same callback queue.
        if (exec_state_ == EXEC_TRAJ && !mandatory_stop_)
          changeFSMExecState(REPLAN_TRAJ, "TRIG");

      }

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }

    return success;
  }

  void EGOReplanFSM::waypointCallback(const quadrotor_msgs::GoalSetPtr &msg)
  {
    if (msg->drone_id != planner_manager_->pp_.drone_id || msg->goal[2] < -0.1)
      return;

    // ROS_INFO("Received goal: %f, %f, %f", msg->goal[0], msg->goal[1], msg->goal[2]);

    Eigen::Vector3d end_wp(msg->goal[0], msg->goal[1], msg->goal[2]);
    planNextWaypoint(end_wp);
    cooperative_goal_velocity_pending_ = false;
  }

  void EGOReplanFSM::readGivenWpsAndPlan()
  {
    if (waypoint_num_ <= 0)
    {
      ROS_ERROR("Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];
    }

    // bool success = planner_manager_->planGlobalTrajWaypoints(
    //   odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    //   wps_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    // plan first global waypoint
    wpt_id_ = 0;
    planNextWaypoint(wps_[wpt_id_]);
  }

  void EGOReplanFSM::mandatoryStopCallback(const std_msgs::Empty &msg)
  {
    mandatory_stop_ = true;
    ROS_ERROR("Received a mandatory stop command!");
    changeFSMExecState(EMERGENCY_STOP, "Mandatory Stop");
    enable_fail_safe_ = false;
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    have_odom_ = true;
  }

  void EGOReplanFSM::objectCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    // Raw timestamped target state for common-world-time geometry only.
    const std::string target_frame = msg->header.frame_id.empty()
        ? "world" : msg->header.frame_id;
    const std::uint64_t target_source_identity =
        multi_uav_formation::stablePredictionSourceIdentity(
            "/object_odom", target_frame, "CONSTANT_VELOCITY");
    planner_manager_->setGeometryTarget(
        Eigen::Vector3d(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z),
        Eigen::Vector3d(msg->twist.twist.linear.x,msg->twist.twist.linear.y,msg->twist.twist.linear.z),
        msg->header.stamp.isZero()?ros::Time::now().toSec():msg->header.stamp.toSec(),
        target_source_identity);
    static std::list<nav_msgs::Odometry> msgs;
    msgs.push_back(*msg);
    while (msgs.size() > 10)
    {
      msgs.pop_front();
    }

    constexpr double INTERVAL = 0.25;
    static bool is_init = false;
    static nav_msgs::Odometry last_msg;
    static ros::Time last_t = ros::Time(0.0);
    ros::Time t_now = ros::Time::now();
    if ((t_now - last_t).toSec() > INTERVAL)
    {
      nav_msgs::Odometry avg_msg;
      avg_msg.pose.pose.position.x = 0;
      avg_msg.pose.pose.position.y = 0;
      avg_msg.pose.pose.position.z = 0;
      avg_msg.pose.pose.orientation.w = 0;
      avg_msg.pose.pose.orientation.x = 0;
      avg_msg.pose.pose.orientation.y = 0;
      avg_msg.pose.pose.orientation.z = 0;
      avg_msg.twist.twist.linear.x = 0;
      avg_msg.twist.twist.linear.y = 0;
      avg_msg.twist.twist.linear.z = 0;
      int i = 0;
      for (auto it = msgs.begin(); it != msgs.end(); ++it)
      {
        ++i;
        avg_msg.pose.pose.position.x += it->pose.pose.position.x;
        avg_msg.pose.pose.position.y += it->pose.pose.position.y;
        avg_msg.pose.pose.position.z += it->pose.pose.position.z;
        avg_msg.pose.pose.orientation.w += it->pose.pose.orientation.w;
        avg_msg.pose.pose.orientation.x += it->pose.pose.orientation.x;
        avg_msg.pose.pose.orientation.y += it->pose.pose.orientation.y;
        avg_msg.pose.pose.orientation.z += it->pose.pose.orientation.z;
        avg_msg.twist.twist.linear.x += it->twist.twist.linear.x;
        avg_msg.twist.twist.linear.y += it->twist.twist.linear.y;
        avg_msg.twist.twist.linear.z += it->twist.twist.linear.z;
      }
      avg_msg.pose.pose.position.x /= i;
      avg_msg.pose.pose.position.y /= i;
      avg_msg.pose.pose.position.z /= i;
      avg_msg.pose.pose.orientation.w /= i;
      avg_msg.pose.pose.orientation.x /= i;
      avg_msg.pose.pose.orientation.y /= i;
      avg_msg.pose.pose.orientation.z /= i;
      avg_msg.twist.twist.linear.x /= i;
      avg_msg.twist.twist.linear.y /= i;
      avg_msg.twist.twist.linear.z /= i;

      if (!is_init)
      {
        is_init = true;
        last_msg = avg_msg;
      }

      last_t = t_now;

      object_p_(0) = avg_msg.pose.pose.position.x;
      object_p_(1) = avg_msg.pose.pose.position.y;
      object_p_(2) = avg_msg.pose.pose.position.z;
      const Eigen::Vector3d measured_v((avg_msg.pose.pose.position.x - last_msg.pose.pose.position.x) / INTERVAL,
                                       (avg_msg.pose.pose.position.y - last_msg.pose.pose.position.y) / INTERVAL,
                                       (avg_msg.pose.pose.position.z - last_msg.pose.pose.position.z) / INTERVAL);
      const Eigen::Vector3d twist_v(avg_msg.twist.twist.linear.x,
                                    avg_msg.twist.twist.linear.y,
                                    avg_msg.twist.twist.linear.z);
      // Target velocity belongs to target prediction, independently of which
      // visibility objective is enabled. A fixed-INTERVAL position difference
      // is not a substitute for the supplied odometry velocity under jitter.
      object_v_ = twist_v.allFinite() && twist_v.norm() > object_velocity_deadband_ ? twist_v : measured_v;
      object_q_.w() = avg_msg.pose.pose.orientation.w;
      object_q_.x() = avg_msg.pose.pose.orientation.x;
      object_q_.y() = avg_msg.pose.pose.orientation.y;
      object_q_.z() = avg_msg.pose.pose.orientation.z;

      // The averaged target state is a filtered estimate of the samples it was
      // built from, so its own time is the average of their stamps.  Publishing
      // it together with that stamp is what makes one authoritative
      // targetPositionAt(world_time) possible; without it every consumer would
      // have to invent its own age compensation.
      double stamp_sum = 0.0;
      int stamp_count = 0;
      for (auto it = msgs.begin(); it != msgs.end(); ++it)
      {
        const double stamp = it->header.stamp.isZero()
                                 ? t_now.toSec()
                                 : it->header.stamp.toSec();
        if (std::isfinite(stamp) && stamp > 0.0)
        {
          stamp_sum += stamp;
          ++stamp_count;
        }
      }
      object_stamp_ =
          stamp_count > 0 ? stamp_sum / static_cast<double>(stamp_count)
                          : t_now.toSec();
      planner_manager_->setTargetState(object_p_, object_v_, object_stamp_);

      last_msg = avg_msg;

      static ros::Time last_moving_t = ros::Time(0);
      if ( object_v_.norm() < object_velocity_deadband_ )
      {
        object_v_.setZero();
      }
      else
      {
        last_moving_t = t_now;
      }

      // if ( object_v_.norm() < 1.0 )
      //   return;

      active_relative_tracking_p_ =
          (enable_fov_tracking_ && fov_enable_candidate_goal_) ? computeFOVRelativeTrackingPoint() : relative_tracking_p_;

      const double goal_lookahead = enable_fov_tracking_ ? std::max(0.0, fov_lookahead_time_) : 5.0;
	      Eigen::Vector3d target_pred = object_p_ + object_v_ * goal_lookahead;
	      if (enable_fov_tracking_ && fov_lock_tracking_height_)
	      {
	        target_pred(2) = fov_target_height_;
	      }
	      const Eigen::Matrix3d object_rotation =
	          enable_fov_tracking_ ? object_q_.normalized().matrix() : object_q_.matrix();
	      Eigen::Vector3d goal_p = object_p_ + object_v_ * goal_lookahead +
	                               object_rotation * active_relative_tracking_p_;
	      Eigen::Vector3d cooperative_viewpoint_position =
	          Eigen::Vector3d::Zero();
	      Eigen::Vector3d cooperative_viewpoint_velocity =
	          Eigen::Vector3d::Zero();
	      double cooperative_viewpoint_age = -1.0;
	      const bool use_cooperative_viewpoint = cooperativeViewpointSnapshot(
	          t_now, cooperative_viewpoint_position,
	          cooperative_viewpoint_velocity, cooperative_viewpoint_age);
	      if (use_cooperative_viewpoint)
	      {
	        // The cooperative layer replaces only the tracking reference.  The
	        // real object_p_/object_v_/object_q_ remain untouched and continue
	        // to drive prediction, dynamic risk, target-facing yaw and metrics.
	        // Preserve the existing target lookahead without linearly
	        // extrapolating the rotational component for five seconds.  The
	        // published viewpoint velocity is used as the global tail state.
	        goal_p = cooperative_viewpoint_position + object_v_ * goal_lookahead;
	        cooperative_goal_velocity_ = cooperative_viewpoint_velocity;
	        cooperative_goal_velocity_pending_ = true;
	        active_relative_tracking_p_ =
	            object_rotation.inverse() * (goal_p - target_pred);
	      }
	      else if (enable_cooperative_viewpoint_reference_)
	      {
	        cooperative_goal_velocity_pending_ = false;
	        active_relative_tracking_p_ = relative_tracking_p_;
	        ROS_WARN_THROTTLE(
	            1.0,
	            "[cooperative-viewpoint-planner] drone_id=%d result=FALLBACK_FIXED_FORMATION reason=REFERENCE_INVALID_OR_STALE topic=%s",
	            planner_manager_->pp_.drone_id,
	            cooperative_viewpoint_reference_topic_.c_str());
	      }
	      if (enable_fov_tracking_ && fov_lock_tracking_height_)
	      {
	        goal_p(2) = fov_target_height_ + active_relative_tracking_p_(2);
	      }
	      if (enable_fov_tracking_ && fov_enable_visible_safe_region_ && fov_enable_candidate_goal_)
	      {
	        goal_p = computeFOVVisibleSafeGoal(goal_p, target_pred);
	        active_relative_tracking_p_ = object_q_.normalized().inverse() * (goal_p - target_pred);
	        if (fov_lock_tracking_height_)
	          active_relative_tracking_p_(2) = relative_tracking_p_(2);
	      }
	      if (enable_fov_tracking_ && fov_enable_goal_safe_projection_)
	      {
	        Eigen::Vector3d safe_goal;
	        if (fovProjectToNearestSafePoint(goal_p, safe_goal, true) &&
	            (safe_goal - goal_p).norm() > 1e-3)
	        {
	          ROS_WARN_THROTTLE(1.0,
	                            "EGO-FOV drone %d projects unsafe tracking goal by %.2f m.",
	                            planner_manager_->pp_.drone_id, (safe_goal - goal_p).norm());
	          goal_p = safe_goal;
	          active_relative_tracking_p_ = object_q_.normalized().inverse() * (goal_p - target_pred);
	          if (fov_lock_tracking_height_)
	            active_relative_tracking_p_(2) = relative_tracking_p_(2);
	        }
	      }
	      ROS_INFO_THROTTLE(
	          1.0,
	          "[cooperative-viewpoint-planner] drone_id=%d enabled=%d reference_used=%d goal=(%.3f,%.3f,%.3f) true_target=(%.3f,%.3f,%.3f) reference_age=%.3f",
	          planner_manager_->pp_.drone_id,
	          static_cast<int>(enable_cooperative_viewpoint_reference_),
	          static_cast<int>(use_cooperative_viewpoint), goal_p.x(), goal_p.y(),
	          goal_p.z(), object_p_.x(), object_p_.y(), object_p_.z(),
	          cooperative_viewpoint_age);

      quadrotor_msgs::GoalSet goal_msg;
      goal_msg.drone_id = planner_manager_->pp_.drone_id;
      goal_msg.goal[0] = goal_p(0);
      goal_msg.goal[1] = goal_p(1);
      goal_msg.goal[2] = goal_p(2);

      // ROS_ERROR("obj_p, obj_v");
      // cout << object_p_.transpose() << " @@ " << object_v_.transpose() << endl;
      // ROS_ERROR("goal_p, odom_pos_, dis");
      // cout << goal_p.transpose() << " @@ " << odom_pos_.transpose() << " @@ " << (goal_p - odom_pos_).norm() << endl;
      // cout << "(t_now - last_moving_t).toSec()=" << (t_now - last_moving_t).toSec() << endl;

      if ( (t_now - last_moving_t).toSec() > 3.0 ) // 3.0 second
      {
        satrt_tracking_ = false;
      }

      if ( (goal_p - odom_pos_).norm() > 1.0 ) // higher priority
      {
        satrt_tracking_ = true;
      }

      // During PRETRACK the target is intentionally stationary.  Publish its
      // formation goal even when the UAV starts within the normal 1 m motion
      // trigger, so the planner can create an initial executable PolyTraj
      // before the target clock is opened.  Once mission start is published,
      // the original moving-target/final-stop behavior remains unchanged.
      if (!mission_started_ && !have_target_)
      {
        satrt_tracking_ = true;
        if (!pretrack_goal_logged_)
        {
          ROS_INFO("[startup-pretrack-goal] drone_id=%d target_available=1 "
                   "mission_started=0 goal=(%.3f,%.3f,%.3f)",
                   planner_manager_->pp_.drone_id, goal_p.x(), goal_p.y(),
                   goal_p.z());
          pretrack_goal_logged_ = true;
        }
      }

      if (satrt_tracking_)
      {
        goal_pub_.publish(goal_msg);
      }

    }

    //   cout << msg->pose.pose.orientation.w << " " << msg->pose.pose.orientation.x << " " << msg->pose.pose.orientation.y << " " << msg->pose.pose.orientation.z << endl;
	  }

  void EGOReplanFSM::targetStartCallback(const std_msgs::Float64ConstPtr &msg)
  {
    if (!std::isfinite(msg->data) || mission_started_)
      return;
    mission_started_ = true;
    ROS_INFO("[startup-mission-start-observed] drone_id=%d start_time=%.9f",
             planner_manager_->pp_.drone_id, msg->data);
  }

	  void EGOReplanFSM::cooperativeViewpointCallback(
	      const nav_msgs::OdometryConstPtr &msg)
	  {
	    std::lock_guard<std::mutex> lock(cooperative_viewpoint_mutex_);
	    cooperative_viewpoint_received_ = true;
	    cooperative_viewpoint_stamp_ = msg->header.stamp;
	    cooperative_viewpoint_p_ = Eigen::Vector3d(
	        msg->pose.pose.position.x, msg->pose.pose.position.y,
	        msg->pose.pose.position.z);
	    cooperative_viewpoint_v_ = Eigen::Vector3d(
	        msg->twist.twist.linear.x, msg->twist.twist.linear.y,
	        msg->twist.twist.linear.z);
	    cooperative_viewpoint_valid_ =
	        !msg->header.stamp.isZero() && msg->pose.covariance[0] >= 0.0 &&
	        cooperative_viewpoint_p_.allFinite() &&
	        cooperative_viewpoint_v_.allFinite() &&
	        msg->header.frame_id == "world";
	  }

	  bool EGOReplanFSM::cooperativeViewpointSnapshot(
	      const ros::Time &now, Eigen::Vector3d &position,
	      Eigen::Vector3d &velocity, double &age) const
	  {
	    std::lock_guard<std::mutex> lock(cooperative_viewpoint_mutex_);
	    position = cooperative_viewpoint_p_;
	    velocity = cooperative_viewpoint_v_;
	    age = cooperative_viewpoint_received_ &&
	              !cooperative_viewpoint_stamp_.isZero()
	        ? std::max(0.0, (now - cooperative_viewpoint_stamp_).toSec())
	        : -1.0;
	    if (!enable_cooperative_viewpoint_reference_ ||
	        !cooperative_viewpoint_received_ || !cooperative_viewpoint_valid_ ||
	        cooperative_viewpoint_stamp_.isZero())
	      return false;
	    const double signed_age = (now - cooperative_viewpoint_stamp_).toSec();
	    age = std::max(0.0, signed_age);
	    return std::isfinite(signed_age) && signed_age >= -0.1 &&
	           signed_age <=
	               std::max(0.05, cooperative_viewpoint_reference_timeout_);
	  }
	
	  bool EGOReplanFSM::fovPointSafe(const Eigen::Vector3d &point, const double clearance)
	  {
	    if (!planner_manager_ || !planner_manager_->grid_map_)
	      return true;

	    const double r = std::max(0.0, clearance);
	    const std::vector<Eigen::Vector3d> offsets{
	        Eigen::Vector3d(0.0, 0.0, 0.0),
	        Eigen::Vector3d(r, 0.0, 0.0), Eigen::Vector3d(-r, 0.0, 0.0),
	        Eigen::Vector3d(0.0, r, 0.0), Eigen::Vector3d(0.0, -r, 0.0),
	        Eigen::Vector3d(0.707 * r, 0.707 * r, 0.0),
	        Eigen::Vector3d(0.707 * r, -0.707 * r, 0.0),
	        Eigen::Vector3d(-0.707 * r, 0.707 * r, 0.0),
	        Eigen::Vector3d(-0.707 * r, -0.707 * r, 0.0)};

	    for (const auto &offset : offsets)
	    {
	      if (planner_manager_->grid_map_->getInflateOccupancy(point + offset) != 0)
	        return false;
	    }
	    return true;
	  }

	  bool EGOReplanFSM::fovSegmentClear(const Eigen::Vector3d &start, const Eigen::Vector3d &end)
	  {
	    if (!planner_manager_ || !planner_manager_->grid_map_)
	      return true;

	    const Eigen::Vector3d delta = end - start;
	    const double length = delta.norm();
	    if (length < 1e-3)
	      return fovPointSafe(end, fov_safe_region_clearance_);

	    const double sample_step = std::max(0.05, fov_los_sample_step_);
	    const int steps = std::max(2, static_cast<int>(std::ceil(length / sample_step)));
	    for (int i = 0; i <= steps; ++i)
	    {
	      const double ratio = static_cast<double>(i) / static_cast<double>(steps);
	      if (!fovPointSafe(start + ratio * delta, fov_safe_region_clearance_))
	        return false;
	    }
	    return true;
	  }

	  bool EGOReplanFSM::fovProjectToNearestSafePoint(const Eigen::Vector3d &point,
	                                                  Eigen::Vector3d &safe_point,
	                                                  const bool allow_without_safe_region)
	  {
	    safe_point = point;
	    if (!enable_fov_tracking_ || (!fov_enable_visible_safe_region_ && !allow_without_safe_region) ||
	        !planner_manager_ || !planner_manager_->grid_map_)
	      return false;

	    if (fovPointSafe(point, fov_safe_region_clearance_))
	      return true;

	    const double step = std::max(0.1, fov_frontend_grid_step_);
	    const double max_radius = std::max(0.9, 4.0 * std::max(step, fov_safe_region_clearance_));
	    const int max_cells = std::max(1, static_cast<int>(std::ceil(max_radius / step)));
	    double best_score = std::numeric_limits<double>::infinity();
	    bool found = false;

	    const bool have_hint = std::isfinite(end_pt_(0)) && std::isfinite(end_pt_(1)) &&
	                           std::abs(end_pt_(0)) < 9999.0 && std::abs(end_pt_(1)) < 9999.0;

	    const int max_z_cells = fov_lock_tracking_height_ ? 0 : max_cells;
	    for (int radius = 1; radius <= max_cells; ++radius)
	    {
	      for (int dx = -radius; dx <= radius; ++dx)
	      {
	        for (int dy = -radius; dy <= radius; ++dy)
	        {
	          const int z_radius = std::min(radius, max_z_cells);
	          for (int dz = -z_radius; dz <= z_radius; ++dz)
	          {
	            if (std::max(std::max(std::abs(dx), std::abs(dy)), std::abs(dz)) != radius)
	              continue;

	            Eigen::Vector3d candidate = point + Eigen::Vector3d(dx * step, dy * step, dz * step);
	            if (fov_lock_tracking_height_)
	              candidate(2) = point(2);
	            if (!fovPointSafe(candidate, fov_safe_region_clearance_))
	              continue;

	            const double distance_score = (candidate - point).norm();
	            const double hint_score = have_hint ? 0.05 * (candidate - end_pt_).norm() : 0.0;
	            const double score = distance_score + hint_score;
	            if (score < best_score)
	            {
	              best_score = score;
	              safe_point = candidate;
	              found = true;
	            }
	          }
	        }
	      }

	      if (found)
	        return true;
	    }

	    return false;
	  }

	  bool EGOReplanFSM::fovApplySafePlanningStart(Eigen::Vector3d &start_pt,
	                                               Eigen::Vector3d &start_vel,
	                                               Eigen::Vector3d &start_acc)
	  {
	    Eigen::Vector3d safe_start;
	    if (!fovProjectToNearestSafePoint(start_pt, safe_start))
	      return false;

	    const double shift = (safe_start - start_pt).norm();
	    if (shift < 1e-3)
	      return false;

	    ROS_WARN_THROTTLE(1.0,
	                      "EGO-FOV drone %d projects unsafe planning start by %.2f m.",
	                      planner_manager_->pp_.drone_id, shift);
	    const Eigen::Vector3d shift_vec = safe_start - start_pt;
	    start_pt = safe_start;
	    if (shift > std::max(0.35, 2.0 * fov_safe_region_clearance_))
	    {
	      start_vel.setZero();
	    }
	    else
	    {
	      const Eigen::Vector3d safe_dir = shift_vec / shift;
	      const double unsafe_speed = start_vel.dot(safe_dir);
	      if (unsafe_speed < 0.0)
	        start_vel -= unsafe_speed * safe_dir;
	    }
		    start_acc.setZero();
		    return true;
		  }

	  bool EGOReplanFSM::fovBuildSafeCorridorWaypoints(const Eigen::Vector3d &start,
	                                                   const Eigen::Vector3d &goal,
	                                                   std::vector<Eigen::Vector3d> &waypoints)
	  {
	    waypoints.clear();
	    if (!enable_fov_tracking_ || !fov_enable_visible_safe_region_ ||
	        !planner_manager_ || !planner_manager_->grid_map_)
	      return false;

	    if (fovSegmentClear(start, goal))
	    {
	      waypoints.push_back(goal);
	      return true;
	    }

	    const double z = goal(2);
	    const double base_step = std::max(0.12, fov_frontend_grid_step_);
	    const double margin = std::max(0.5, fov_frontend_search_margin_);
	    const double min_x = std::min(start(0), goal(0)) - margin;
	    const double max_x = std::max(start(0), goal(0)) + margin;
	    const double min_y = std::min(start(1), goal(1)) - margin;
	    const double max_y = std::max(start(1), goal(1)) + margin;
	    const int max_cells = 90;
	    const double range_x = std::max(base_step, max_x - min_x);
	    const double range_y = std::max(base_step, max_y - min_y);
	    const double step = std::max(base_step, std::max(range_x, range_y) / static_cast<double>(max_cells));
	    const int nx = std::max(3, static_cast<int>(std::ceil(range_x / step)) + 1);
	    const int ny = std::max(3, static_cast<int>(std::ceil(range_y / step)) + 1);

	    auto to_id = [ny](const int ix, const int iy) { return ix * ny + iy; };
	    auto clamp = [](const int v, const int lo, const int hi) { return std::max(lo, std::min(hi, v)); };
	    auto to_grid = [&](const Eigen::Vector3d &p) {
	      const int ix = clamp(static_cast<int>(std::round((p(0) - min_x) / step)), 0, nx - 1);
	      const int iy = clamp(static_cast<int>(std::round((p(1) - min_y) / step)), 0, ny - 1);
	      return Eigen::Vector2i(ix, iy);
	    };
	    auto to_pos = [&](const int ix, const int iy) {
	      return Eigen::Vector3d(min_x + ix * step, min_y + iy * step, z);
	    };
	    auto cell_free = [&](const int ix, const int iy) {
	      return fovPointSafe(to_pos(ix, iy), fov_safe_region_clearance_);
	    };
	    auto nearest_free = [&](Eigen::Vector2i idx) {
	      if (cell_free(idx(0), idx(1)))
	        return idx;
	      for (int radius = 1; radius <= 8; ++radius)
	      {
	        for (int dx = -radius; dx <= radius; ++dx)
	        {
	          for (int dy = -radius; dy <= radius; ++dy)
	          {
	            if (std::max(std::abs(dx), std::abs(dy)) != radius)
	              continue;
	            const int ix = idx(0) + dx;
	            const int iy = idx(1) + dy;
	            if (ix < 0 || ix >= nx || iy < 0 || iy >= ny)
	              continue;
	            if (cell_free(ix, iy))
	              return Eigen::Vector2i(ix, iy);
	          }
	        }
	      }
	      return Eigen::Vector2i(-1, -1);
	    };

	    const Eigen::Vector2i start_idx = nearest_free(to_grid(start));
	    const Eigen::Vector2i goal_idx = nearest_free(to_grid(goal));
	    if (start_idx(0) < 0 || goal_idx(0) < 0)
	    {
	      ROS_WARN_THROTTLE(1.0, "EGO-FOV drone %d cannot seed visible safe corridor.",
	                        planner_manager_->pp_.drone_id);
	      return false;
	    }

	    struct Node
	    {
	      double g{std::numeric_limits<double>::infinity()};
	      int parent{-1};
	      bool closed{false};
	    };
	    struct QueueNode
	    {
	      double f;
	      int id;
	      bool operator<(const QueueNode &other) const { return f > other.f; }
	    };

	    std::vector<Node> nodes(nx * ny);
	    std::priority_queue<QueueNode> open_set;
	    const int start_id = to_id(start_idx(0), start_idx(1));
	    const int goal_id = to_id(goal_idx(0), goal_idx(1));
	    auto heuristic = [&](const int ix, const int iy) {
	      return std::hypot(static_cast<double>(ix - goal_idx(0)), static_cast<double>(iy - goal_idx(1)));
	    };

	    nodes[start_id].g = 0.0;
	    open_set.push({heuristic(start_idx(0), start_idx(1)), start_id});
	    const int dirs[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
	    bool found = false;

	    while (!open_set.empty())
	    {
	      const QueueNode top = open_set.top();
	      open_set.pop();
	      if (nodes[top.id].closed)
	        continue;
	      nodes[top.id].closed = true;
	      if (top.id == goal_id)
	      {
	        found = true;
	        break;
	      }

	      const int cx = top.id / ny;
	      const int cy = top.id % ny;
		      for (const auto &dir : dirs)
		      {
		        const int nx_i = cx + dir[0];
		        const int ny_i = cy + dir[1];
		        if (nx_i < 0 || nx_i >= nx || ny_i < 0 || ny_i >= ny || !cell_free(nx_i, ny_i))
		          continue;
	        if (dir[0] != 0 && dir[1] != 0 &&
	            (!cell_free(cx + dir[0], cy) || !cell_free(cx, cy + dir[1])))
	          continue;

		        const int nid = to_id(nx_i, ny_i);
		        if (nodes[nid].closed)
		          continue;
	        const double move_cost = std::hypot(static_cast<double>(dir[0]), static_cast<double>(dir[1]));
	        const double tentative_g = nodes[top.id].g + move_cost;
	        if (tentative_g < nodes[nid].g)
	        {
	          nodes[nid].g = tentative_g;
	          nodes[nid].parent = top.id;
	          open_set.push({tentative_g + heuristic(nx_i, ny_i), nid});
	        }
	      }
	    }

	    if (!found)
	    {
	      ROS_WARN_THROTTLE(1.0, "EGO-FOV drone %d found no visible safe corridor.",
	                        planner_manager_->pp_.drone_id);
	      return false;
	    }

	    std::vector<Eigen::Vector3d> raw_path;
	    for (int id = goal_id; id >= 0; id = nodes[id].parent)
	    {
	      raw_path.push_back(to_pos(id / ny, id % ny));
	      if (id == start_id)
	        break;
	    }
		    std::reverse(raw_path.begin(), raw_path.end());
		    if (raw_path.size() < 2)
		      return false;
	    if (fovPointSafe(start, fov_safe_region_clearance_))
	      raw_path.front() = Eigen::Vector3d(start(0), start(1), z);
	    if (fovPointSafe(goal, fov_safe_region_clearance_))
	      raw_path.back() = goal;

	    size_t anchor = 0;
	    while (anchor + 1 < raw_path.size())
	    {
	      size_t next = anchor + 1;
	      for (size_t j = raw_path.size() - 1; j > anchor; --j)
	      {
	        if (fovSegmentClear(raw_path[anchor], raw_path[j]))
	        {
	          next = j;
	          break;
	        }
	      }
	      waypoints.push_back(raw_path[next]);
	      anchor = next;
	    }

	    ROS_INFO_THROTTLE(1.0, "EGO-FOV drone %d front-end corridor waypoints=%zu raw=%zu",
	                      planner_manager_->pp_.drone_id, waypoints.size(), raw_path.size());
	    return !waypoints.empty();
	  }

	  Eigen::Vector3d EGOReplanFSM::computeFOVVisibleSafeGoal(const Eigen::Vector3d &nominal_goal,
	                                                          const Eigen::Vector3d &target_pred)
	  {
	    if (!enable_fov_tracking_ || !fov_enable_visible_safe_region_ ||
	        relative_tracking_p_(0) < -9999 || relative_tracking_p_(1) < -9999 || relative_tracking_p_(2) < -9999)
	      return nominal_goal;
	    if (!fov_lock_tracking_height_)
	      return nominal_goal;

		    const Eigen::Vector2d nominal_delta = nominal_goal.head<2>() - target_pred.head<2>();
		    const double nominal_side = nominal_delta(1);
		    auto keeps_nominal_observation_side = [&](const Eigen::Vector3d &candidate) {
		      if (std::abs(nominal_side) < 0.2)
		        return true;
		      const double candidate_side = candidate(1) - target_pred(1);
		      return candidate_side * nominal_side >= -0.05;
		    };
		
		    auto in_visible_safe_region = [&](const Eigen::Vector3d &candidate, const bool enforce_nominal_side) {
		      if (enforce_nominal_side && !keeps_nominal_observation_side(candidate))
		        return false;
		      const double distance_xy = (candidate.head<2>() - target_pred.head<2>()).norm();
		      if (distance_xy < fov_min_target_distance_ || distance_xy > fov_max_target_distance_)
		        return false;
	      if (!fovPointSafe(candidate, fov_safe_region_clearance_))
	        return false;
	      if (fovOcclusionCost(candidate, target_pred) > 1e-9)
	        return false;
	      return true;
	    };

		    const double base_angle = nominal_delta.norm() > 1e-3 ? std::atan2(nominal_delta(1), nominal_delta(0)) : M_PI;
	    const double base_radius = std::max(fov_tracking_distance_, std::max(0.1, relative_tracking_p_.head<2>().norm()));
	    const std::vector<double> radius_scales{0.85, 1.0, 1.15, 1.3};
	    const int candidate_count = std::max(8, fov_candidate_count_);
	    const double angle_span = std::max(30.0 * M_PI / 180.0, std::min(fov_candidate_angle_span_, 2.0 * M_PI));

	    struct VisibleSafeCandidate
	    {
	      Eigen::Vector3d point;
	      double score;
	    };
	    std::vector<VisibleSafeCandidate> candidates;

	    auto eval_candidate = [&](const Eigen::Vector3d &candidate, const bool enforce_nominal_side) {
	      if (!in_visible_safe_region(candidate, enforce_nominal_side))
	        return;

	      const double distance_xy = (candidate.head<2>() - target_pred.head<2>()).norm();
	      const double direct_path = (candidate - odom_pos_).norm();
	      const double corridor_penalty = fovSegmentClear(odom_pos_, candidate) ? 0.0 : 1.0;
	      const double desired_cost = fov_desired_weight_ * (candidate - nominal_goal).squaredNorm();
	      const double smooth_cost = fov_smooth_weight_ *
	                                 (candidate - odom_pos_ - odom_vel_ * std::max(0.1, fov_lookahead_time_)).squaredNorm();
	      const double distance_cost = fov_distance_weight_ * std::pow(distance_xy - fov_tracking_distance_, 2);
	      const double path_cost = fov_frontend_path_weight_ * direct_path + 2.0 * corridor_penalty;
	      const double score = desired_cost + smooth_cost + distance_cost + path_cost;
	      candidates.push_back({candidate, score});
	    };

	    auto sample_candidates = [&](const bool enforce_nominal_side) {
	      eval_candidate(nominal_goal, enforce_nominal_side);
	      for (double radius_scale : radius_scales)
	      {
	        const double radius = base_radius * radius_scale;
	        for (int idx = 0; idx < candidate_count; ++idx)
	        {
	          const double alpha = candidate_count == 1 ? 0.5 : static_cast<double>(idx) / static_cast<double>(candidate_count - 1);
	          const double angle = base_angle - 0.5 * angle_span + alpha * angle_span;
	          Eigen::Vector3d candidate(target_pred(0) + radius * std::cos(angle),
	                                    target_pred(1) + radius * std::sin(angle),
	                                    nominal_goal(2));
	          eval_candidate(candidate, enforce_nominal_side);
	        }
	      }
	    };
	
	    sample_candidates(true);
	    if (candidates.empty() && std::abs(nominal_side) >= 0.2)
	    {
	      ROS_WARN_THROTTLE(1.0,
	                        "EGO-FOV drone %d relaxes observation side in narrow passage.",
	                        planner_manager_->pp_.drone_id);
	      sample_candidates(false);
	    }

	    if (candidates.empty())
	    {
	      ROS_WARN_THROTTLE(1.0,
	                        "EGO-FOV drone %d found no visible safe region, keep nominal goal.",
	                        planner_manager_->pp_.drone_id);
	      return nominal_goal;
	    }

	    std::sort(candidates.begin(), candidates.end(),
	              [](const VisibleSafeCandidate &a, const VisibleSafeCandidate &b) {
	                return a.score < b.score;
	              });

    Eigen::Vector3d best = candidates.front().point;
    double best_score = candidates.front().score;
    bool have_corridor_best = false;
    for (size_t i = 0; i < candidates.size(); ++i)
    {
      std::vector<Eigen::Vector3d> corridor;
      if (fovBuildSafeCorridorWaypoints(odom_pos_, candidates[i].point, corridor))
	      {
	        best = candidates[i].point;
	        best_score = candidates[i].score;
	        have_corridor_best = true;
	        break;
	      }
	    }

	    if (!have_corridor_best)
	    {
	      ROS_WARN_THROTTLE(1.0,
	                        "EGO-FOV drone %d found visible region but no front-end corridor, keep best visible point.",
	                        planner_manager_->pp_.drone_id);
	    }

	    ROS_INFO_THROTTLE(1.0,
	                      "EGO-FOV drone %d selected visible safe goal=(%.2f %.2f %.2f), score=%.2f",
	                      planner_manager_->pp_.drone_id, best(0), best(1), best(2), best_score);
	    return best;
	  }
	
	  bool EGOReplanFSM::fovLineOfSightClear(const Eigen::Vector3d &observer, const Eigen::Vector3d &target)
	  {
	    return fovOcclusionCost(observer, target) <= 1e-9;
	  }

  double EGOReplanFSM::fovOcclusionCost(const Eigen::Vector3d &observer, const Eigen::Vector3d &target)
  {
    if (!fov_use_los_check_ || !planner_manager_ || !planner_manager_->grid_map_)
      return 0.0;

    const Eigen::Vector3d delta = target - observer;
    const double distance = delta.norm();
    if (distance < 1e-3)
      return 0.0;

    const double sample_step = std::max(0.03, fov_los_sample_step_);
    const int steps = std::max(2, static_cast<int>(std::ceil(distance / sample_step)));
    int occupied_samples = 0;
    for (int i = 1; i < steps; ++i)
    {
      const double ratio = static_cast<double>(i) / static_cast<double>(steps);
      Eigen::Vector3d sample = observer + ratio * delta;
      const int occ = planner_manager_->grid_map_->getInflateOccupancy(sample);
      if (occ > 0)
        ++occupied_samples;
    }

    if (occupied_samples == 0)
      return 0.0;

    const double occupied_ratio = static_cast<double>(occupied_samples) / static_cast<double>(std::max(1, steps - 1));
    return fov_occlusion_weight_ * (0.5 + occupied_ratio);
  }

  double EGOReplanFSM::fovAngleViolation(const Eigen::Vector3d &observer, const Eigen::Vector3d &target,
                                         const Eigen::Vector3d &reference_observer)
  {
    if (!fov_use_angle_check_)
      return 0.0;

    Eigen::Vector2d los = target.head<2>() - observer.head<2>();
    Eigen::Vector2d reference_los = target.head<2>() - reference_observer.head<2>();
    const double los_norm = los.norm();
    const double ref_norm = reference_los.norm();
    if (los_norm < 1e-3 || ref_norm < 1e-3)
      return 0.0;

    const double cos_angle = std::max(-1.0, std::min(1.0, los.dot(reference_los) / (los_norm * ref_norm)));
    const double angle = std::acos(cos_angle);
    return std::max(0.0, angle - fov_half_angle_);
  }

  Eigen::Vector3d EGOReplanFSM::computeFOVRelativeTrackingPoint()
  {
    if (relative_tracking_p_(0) < -9999 || relative_tracking_p_(1) < -9999 || relative_tracking_p_(2) < -9999)
      return relative_tracking_p_;

    Eigen::Quaterniond object_q = object_q_.normalized();
    Eigen::Vector3d target_pred = object_p_ + object_v_ * std::max(0.0, fov_lookahead_time_);
    if (fov_lock_tracking_height_)
      target_pred(2) = fov_target_height_;

    Eigen::Vector3d desired = target_pred + object_q.matrix() * relative_tracking_p_;
    if (fov_lock_tracking_height_)
      desired(2) = fov_target_height_ + relative_tracking_p_(2);

    auto candidate_occupied = [&](const Eigen::Vector3d &candidate) {
      if (!planner_manager_ || !planner_manager_->grid_map_)
        return false;
      return planner_manager_->grid_map_->getInflateOccupancy(candidate) > 0;
    };

    auto candidate_clear = [&](const Eigen::Vector3d &candidate) {
      const double distance_xy = (candidate.head<2>() - target_pred.head<2>()).norm();
      if (distance_xy < fov_min_target_distance_ || distance_xy > fov_max_target_distance_)
        return false;
      if (candidate_occupied(candidate))
        return false;
      if (fovOcclusionCost(candidate, target_pred) > 1e-9)
        return false;
      if (fovAngleViolation(candidate, target_pred, desired) > 1e-6)
        return false;
      return true;
    };

    if (candidate_clear(desired))
    {
      Eigen::Vector3d active_relative = object_q.inverse() * (desired - target_pred);
      if (fov_lock_tracking_height_)
        active_relative(2) = relative_tracking_p_(2);
      return active_relative;
    }

    const Eigen::Vector2d desired_delta = desired.head<2>() - target_pred.head<2>();
    const double base_angle = desired_delta.norm() > 1e-3 ? std::atan2(desired_delta(1), desired_delta(0)) : M_PI;
    const double base_radius = std::max(fov_tracking_distance_, std::max(0.1, relative_tracking_p_.head<2>().norm()));
    const std::vector<double> radius_scales{0.85, 1.0, 1.2};
    const int candidate_count = std::max(1, fov_candidate_count_);
    const double min_angle_span = 30.0 * M_PI / 180.0;
    const double angle_span = std::max(min_angle_span, std::min(fov_candidate_angle_span_, 2.0 * M_PI));

    bool have_clear_best = false;
    Eigen::Vector3d clear_best = desired;
    double clear_best_score = std::numeric_limits<double>::infinity();

    auto eval_candidate = [&](const Eigen::Vector3d &candidate) {
      const double distance_xy = (candidate.head<2>() - target_pred.head<2>()).norm();
      if (distance_xy < fov_min_target_distance_ || distance_xy > fov_max_target_distance_)
        return;

      const bool occupied = candidate_occupied(candidate);
      const double occupancy_cost = occupied ? fov_occlusion_weight_ : 0.0;
      const double occlusion_cost = fovOcclusionCost(candidate, target_pred);
      const double fov_violation = fovAngleViolation(candidate, target_pred, desired);
      const double desired_cost = fov_desired_weight_ * (candidate - desired).squaredNorm();
      const double smooth_cost = fov_smooth_weight_ *
                                 (candidate - odom_pos_ - odom_vel_ * std::max(0.1, fov_lookahead_time_)).squaredNorm();
      const double distance_cost = fov_distance_weight_ * std::pow(distance_xy - fov_tracking_distance_, 2);
      const double fov_cost = fov_weight_ * fov_violation * fov_violation;
      const double score = desired_cost + smooth_cost + distance_cost + fov_cost + occupancy_cost + occlusion_cost;
      const bool clear = !occupied && occlusion_cost <= 1e-9 && fov_violation <= 1e-6;

      if (clear && (!have_clear_best || score < clear_best_score))
      {
        have_clear_best = true;
        clear_best_score = score;
        clear_best = candidate;
      }
    };

    eval_candidate(desired);
    for (double radius_scale : radius_scales)
    {
      const double radius = base_radius * radius_scale;
      for (int idx = 0; idx < candidate_count; ++idx)
      {
        const double alpha = candidate_count == 1 ? 0.5 : static_cast<double>(idx) / static_cast<double>(candidate_count - 1);
        const double angle = base_angle - 0.5 * angle_span + alpha * angle_span;
        Eigen::Vector3d candidate(target_pred(0) + radius * std::cos(angle),
                                  target_pred(1) + radius * std::sin(angle),
                                  desired(2));
        eval_candidate(candidate);
      }
    }

    if (!have_clear_best)
    {
      ROS_WARN_THROTTLE(1.0,
                        "EGO-FOV drone %d found no clear candidate, keep nominal relative tracking point.",
                        planner_manager_->pp_.drone_id);
      return relative_tracking_p_;
    }

    Eigen::Vector3d active_relative = object_q.inverse() * (clear_best - target_pred);
    if (fov_lock_tracking_height_)
      active_relative(2) = relative_tracking_p_(2);

    ROS_INFO_THROTTLE(1.0,
                      "EGO-FOV selected candidate relative=(%.2f %.2f %.2f), score=%.2f",
                      active_relative(0), active_relative(1), active_relative(2), clear_best_score);
    return active_relative;
  }

  void EGOReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
  }

  void EGOReplanFSM::RecvBroadcastMINCOTrajCallback(const traj_utils::MINCOTrajConstPtr &msg)
  {
    const size_t recv_id = (size_t)msg->drone_id;
    if ((int)recv_id == planner_manager_->pp_.drone_id) // myself
      return;

    if (msg->drone_id < 0)
    {
      ROS_ERROR("drone_id < 0 is not allowed in a swarm system!");
      return;
    }
    if (msg->order != 5)
    {
      ROS_ERROR("Only support trajectory order equals 5 now!");
      return;
    }
    if (msg->duration.empty() ||
        msg->duration.size() != (msg->inner_x.size() + 1) ||
        msg->inner_x.size() != msg->inner_y.size() ||
        msg->inner_x.size() != msg->inner_z.size())
    {
      ROS_ERROR("WRONG trajectory parameters.");
      return;
    }
    for (const double duration : msg->duration)
    {
      if (!std::isfinite(duration) || duration <= 0.0)
      {
        ROS_ERROR("WRONG trajectory duration.");
        return;
      }
    }
    for (size_t i = 0; i < msg->inner_x.size(); ++i)
    {
      if (!std::isfinite(msg->inner_x[i]) ||
          !std::isfinite(msg->inner_y[i]) ||
          !std::isfinite(msg->inner_z[i]))
      {
        ROS_ERROR("WRONG non-finite trajectory junction.");
        return;
      }
    }
    for (size_t axis = 0; axis < 3; ++axis)
    {
      if (!std::isfinite(msg->start_p[axis]) ||
          !std::isfinite(msg->start_v[axis]) ||
          !std::isfinite(msg->start_a[axis]) ||
          !std::isfinite(msg->end_p[axis]) ||
          !std::isfinite(msg->end_v[axis]) ||
          !std::isfinite(msg->end_a[axis]))
      {
        ROS_ERROR("WRONG non-finite trajectory boundary state.");
        return;
      }
    }
    if (planner_manager_->traj_.swarm_traj.size() > recv_id &&
        planner_manager_->traj_.swarm_traj[recv_id].drone_id == (int)recv_id &&
        msg->start_time.toSec() - planner_manager_->traj_.swarm_traj[recv_id].start_time <= 0)
    {
      ROS_WARN("Received drone %d's trajectory out of order or duplicated, abandon it.", (int)recv_id);
      return;
    }

    ros::Time t_now = ros::Time::now();
    if (abs((t_now - msg->start_time).toSec()) > 0.25)
    {

      if (abs((t_now - msg->start_time).toSec()) < 10.0) // 10 seconds offset, more likely to be caused by unsynced system time.
      {
        ROS_WARN("Time stamp diff: Local - Remote Agent %d = %fs",
                 msg->drone_id, (t_now - msg->start_time).toSec());
      }
      else
      {
        ROS_ERROR("Time stamp diff: Local - Remote Agent %d = %fs, swarm time seems not synchronized, abandon!",
                  msg->drone_id, (t_now - msg->start_time).toSec());
        return;
      }
    }

    /* Fill up the buffer */
    if (planner_manager_->traj_.swarm_traj.size() <= recv_id)
    {
      for (size_t i = planner_manager_->traj_.swarm_traj.size(); i <= recv_id; i++)
      {
        LocalTrajData blank;
        blank.drone_id = -1;
        planner_manager_->traj_.swarm_traj.push_back(blank);
      }
    }

    // Future successor does not erase the peer still executing before activation.
    auto &slot=planner_manager_->traj_.swarm_traj[recv_id];
    if(slot.traj.getPieceNum()>0) {
      auto previous=std::make_shared<LocalTrajData>(slot);
      previous->predecessor.reset();slot.predecessor=previous;
    }
    /* Store data */
    planner_manager_->traj_.swarm_traj[recv_id].drone_id = recv_id;
    planner_manager_->traj_.swarm_traj[recv_id].traj_id = msg->traj_id;
    planner_manager_->traj_.swarm_traj[recv_id].start_time = msg->start_time.toSec();

    int piece_nums = msg->duration.size();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << msg->start_p[0], msg->start_v[0], msg->start_a[0],
        msg->start_p[1], msg->start_v[1], msg->start_a[1],
        msg->start_p[2], msg->start_v[2], msg->start_a[2];
    tailState << msg->end_p[0], msg->end_v[0], msg->end_a[0],
        msg->end_p[1], msg->end_v[1], msg->end_a[1],
        msg->end_p[2], msg->end_v[2], msg->end_a[2];
    Eigen::MatrixXd innerPts(3, piece_nums - 1);
    Eigen::VectorXd durations(piece_nums);
    for (int i = 0; i < piece_nums - 1; i++)
      innerPts.col(i) << msg->inner_x[i], msg->inner_y[i], msg->inner_z[i];
    for (int i = 0; i < piece_nums; i++)
      durations(i) = msg->duration[i];
    poly_traj::MinJerkOpt MJO;
    MJO.reset(headState, tailState, piece_nums);
    MJO.generate(innerPts, durations);

    poly_traj::Trajectory trajectory = MJO.getTraj();
    planner_manager_->traj_.swarm_traj[recv_id].traj = trajectory;

    planner_manager_->traj_.swarm_traj[recv_id].duration = trajectory.getTotalDuration();
    planner_manager_->traj_.swarm_traj[recv_id].checked_until =
        msg->checked_until;
    planner_manager_->traj_.swarm_traj[recv_id].start_pos = trajectory.getPos(0.0);

    /* Check Collision */
    if (planner_manager_->checkCollision(recv_id))
    {
      changeFSMExecState(REPLAN_TRAJ, "SWARM_CHECK");
    }

    /* Check if receive agents have lower drone id */
    if (!have_recv_pre_agent_)
    {
      if ((int)planner_manager_->traj_.swarm_traj.size() >= planner_manager_->pp_.drone_id)
      {
        bool all_previous_agents_received = true;
        for (int i = 0; i < planner_manager_->pp_.drone_id; ++i)
        {
          if (planner_manager_->traj_.swarm_traj[i].drone_id != i)
          {
            all_previous_agents_received = false;
            break;
          }
        }
        have_recv_pre_agent_ = all_previous_agents_received;
      }
    }
  }

  void EGOReplanFSM::polyTraj2ROSMsg(traj_utils::PolyTraj &poly_msg, traj_utils::MINCOTraj &MINCO_msg)
  {

    auto data = &planner_manager_->traj_.local_traj;
    Eigen::VectorXd durs = data->traj.getDurations();
    int piece_num = data->traj.getPieceNum();

    poly_msg.drone_id = planner_manager_->pp_.drone_id;
    poly_msg.traj_id = data->traj_id;
    poly_msg.start_time = ros::Time(data->start_time);
    poly_msg.order = 5; // todo, only support order = 5 now.
    poly_msg.duration.resize(piece_num);
    poly_msg.coef_x.resize(6 * piece_num);
    poly_msg.coef_y.resize(6 * piece_num);
    poly_msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i)
    {
      poly_msg.duration[i] = durs(i);

      poly_traj::CoefficientMat cMat = data->traj.getPiece(i).getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++)
      {
        poly_msg.coef_x[i6 + j] = cMat(0, j);
        poly_msg.coef_y[i6 + j] = cMat(1, j);
        poly_msg.coef_z[i6 + j] = cMat(2, j);
      }
    }
    planner_manager_->annotateExecutionSource(poly_msg);
    poly_msg.payload_hash = traj_utils::polyPayloadHash(poly_msg);
    ROS_INFO("[EXECUTION_IDENTITY] stage=PUBLISHED trajectory_id=%d "
             "ACK_REVISION=%lu payload_hash=%lu checked_until=%.9f",
             poly_msg.traj_id,
             static_cast<unsigned long>(poly_msg.validation_revision),
             static_cast<unsigned long>(poly_msg.payload_hash),
             poly_msg.checked_until);
    poly_msg.optimized_yaw_valid = data->optimized_yaw_valid;
    poly_msg.yaw_sample_times = data->optimized_yaw_sample_times;
    poly_msg.yaw_samples = data->optimized_yaw_samples;

    MINCO_msg.drone_id = planner_manager_->pp_.drone_id;
    MINCO_msg.traj_id = data->traj_id;
    MINCO_msg.start_time = ros::Time(data->start_time);
    MINCO_msg.order = 5; // todo, only support order = 5 now.
    MINCO_msg.duration.resize(piece_num);
    Eigen::Vector3d vec;
    vec = data->traj.getPos(0);
    MINCO_msg.start_p[0] = vec(0), MINCO_msg.start_p[1] = vec(1), MINCO_msg.start_p[2] = vec(2);
    vec = data->traj.getVel(0);
    MINCO_msg.start_v[0] = vec(0), MINCO_msg.start_v[1] = vec(1), MINCO_msg.start_v[2] = vec(2);
    vec = data->traj.getAcc(0);
    MINCO_msg.start_a[0] = vec(0), MINCO_msg.start_a[1] = vec(1), MINCO_msg.start_a[2] = vec(2);
    vec = data->traj.getPos(data->duration);
    MINCO_msg.end_p[0] = vec(0), MINCO_msg.end_p[1] = vec(1), MINCO_msg.end_p[2] = vec(2);
    vec = data->traj.getVel(data->duration);
    MINCO_msg.end_v[0] = vec(0), MINCO_msg.end_v[1] = vec(1), MINCO_msg.end_v[2] = vec(2);
    vec = data->traj.getAcc(data->duration);
    MINCO_msg.end_a[0] = vec(0), MINCO_msg.end_a[1] = vec(1), MINCO_msg.end_a[2] = vec(2);
    MINCO_msg.inner_x.resize(piece_num - 1);
    MINCO_msg.inner_y.resize(piece_num - 1);
    MINCO_msg.inner_z.resize(piece_num - 1);
    Eigen::MatrixXd pos = data->traj.getPositions();
    for (int i = 0; i < piece_num - 1; i++)
    {
      MINCO_msg.inner_x[i] = pos(0, i + 1);
      MINCO_msg.inner_y[i] = pos(1, i + 1);
      MINCO_msg.inner_z[i] = pos(2, i + 1);
    }
    for (int i = 0; i < piece_num; i++)
      MINCO_msg.duration[i] = durs[i];
    MINCO_msg.checked_until = poly_msg.checked_until;
  }

  bool EGOReplanFSM::measureGroundHeight(double &height)
  {
    if ( planner_manager_->traj_.local_traj.pts_chk.size() < 3 ) // means planning have not started
    {
      return false;
    } 

    auto traj = &planner_manager_->traj_.local_traj;
    auto map = planner_manager_->grid_map_;
    ros::Time t_now = ros::Time::now();

    double forward_t = 2.0 / planner_manager_->pp_.max_vel_; //2.0m
    double traj_t = (t_now.toSec() - traj->start_time) + forward_t;
    if (traj_t <= traj->duration)
    {
      Eigen::Vector3d forward_p = traj->traj.getPos(traj_t);

      double reso = map->getResolution();
      for (;; forward_p(2) -= reso)
      {
        int ret = map->getOccupancy(forward_p);
        if ( ret == -1 ) // reach map bottom
        {
          return false;
        }
        if ( ret == 1 ) // reach the ground
        {
          height = forward_p(2);

          std_msgs::Float64 height_msg;
          height_msg.data = height;
          ground_height_pub_.publish(height_msg);
          
          return true;
        }
      }
    }

    return false;
  }
} // namespace ego_planner
