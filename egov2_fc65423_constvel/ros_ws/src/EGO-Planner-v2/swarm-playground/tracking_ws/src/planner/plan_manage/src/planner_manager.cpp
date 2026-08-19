// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <thread>
#include "visualization_msgs/Marker.h" // zx-todo

namespace ego_planner
{

  // SECTION interfaces for setup and query

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() { std::cout << "des manager" << std::endl; }

  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */

    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/max_jer", pp_.max_jer_, -1.0);
    nh.param("manager/enable_post_checks", enable_post_checks_, false);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/polyTraj_piece_length", pp_.polyTraj_piece_length, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
    nh.param("manager/use_distinctive_trajs", pp_.use_distinctive_trajs, false);
    nh.param("manager/drone_id", pp_.drone_id, -1);

    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    bool use_time_aware_moving_obj_cost = false;
    nh.param("prediction/use_time_aware_moving_obj_cost", use_time_aware_moving_obj_cost, false);
    if (use_time_aware_moving_obj_cost)
    {
      obj_predictor_.reset(new fast_planner::ObjPredictor(nh));
      obj_predictor_->init();
      ROS_INFO("[moving_obj] time-aware moving obstacle cost enabled.");
    }

    ploy_traj_opt_.reset(new PolyTrajOptimizer);
    ploy_traj_opt_->setParam(nh);
    if (use_time_aware_moving_obj_cost)
      ploy_traj_opt_->setEnvironment(grid_map_, obj_predictor_);
    else
      ploy_traj_opt_->setEnvironment(grid_map_);

    visualization_ = vis;

    ploy_traj_opt_->setSwarmTrajs(&traj_.swarm_traj);
    ploy_traj_opt_->setDroneId(pp_.drone_id);
  }

  bool EGOPlannerManager::computeInitState(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
      const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel,
      const bool flag_polyInit, const bool flag_randomPolyTraj, const double &ts,
      poly_traj::MinJerkOpt &initMJO)
  {

    static bool flag_first_call = true;

    if (flag_first_call || flag_polyInit) /*** case 1: polynomial initialization ***/
    {
      flag_first_call = false;

      /* basic params */
      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs;
      Eigen::VectorXd piece_dur_vec;
      int piece_nums;
      constexpr double init_of_init_totaldur = 2.0;
      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

      /* determined or random inner point */
      if (!flag_randomPolyTraj)
      {
        if (innerPs.cols() != 0)
        {
          ROS_ERROR("innerPs.cols() != 0");
        }

        piece_nums = 1;
        piece_dur_vec.resize(1);
        piece_dur_vec(0) = init_of_init_totaldur;
      }
      else
      {
        Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
        Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
        innerPs.resize(3, 1);
        innerPs = (start_pt + local_target_pt) / 2 +
                  (((double)rand()) / RAND_MAX - 0.5) *
                      (start_pt - local_target_pt).norm() *
                      horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                  (((double)rand()) / RAND_MAX - 0.5) *
                      (start_pt - local_target_pt).norm() *
                      vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);

        piece_nums = 2;
        piece_dur_vec.resize(2);
        piece_dur_vec = Eigen::Vector2d(init_of_init_totaldur / 2, init_of_init_totaldur / 2);
      }

      /* generate the init of init trajectory */
      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
      poly_traj::Trajectory initTraj = initMJO.getTraj();

      /* generate the real init trajectory */
      piece_nums = round((headState.col(0) - tailState.col(0)).norm() / pp_.polyTraj_piece_length);
      if (piece_nums < 2)
        piece_nums = 2;
      double piece_dur = init_of_init_totaldur / (double)piece_nums;
      piece_dur_vec.resize(piece_nums);
      piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, ts);
      innerPs.resize(3, piece_nums - 1);
      int id = 0;
      double t_s = piece_dur, t_e = init_of_init_totaldur - piece_dur / 2;
      for (double t = t_s; t < t_e; t += piece_dur)
      {
        innerPs.col(id++) = initTraj.getPos(t);
      }
      if (id != piece_nums - 1)
      {
        ROS_ERROR("Should not happen! x_x");
        return false;
      }
      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
    }
    else /*** case 2: initialize from previous optimal trajectory ***/
    {
      if (traj_.global_traj.last_glb_t_of_lc_tgt < 0.0)
      {
        ROS_ERROR("You are initialzing a trajectory from a previous optimal trajectory, but no previous trajectories up to now.");
        return false;
      }

      /* the trajectory time system is a little bit complicated... */
      double passed_t_on_lctraj = ros::Time::now().toSec() - traj_.local_traj.start_time;
      double t_to_lc_end = traj_.local_traj.duration - passed_t_on_lctraj;
      if ( t_to_lc_end < 0 )
      {
        ROS_INFO("t_to_lc_end < 0, exit and wait for another call.");
        return false;
      }
      double t_to_lc_tgt = t_to_lc_end +
                           (traj_.global_traj.glb_t_of_lc_tgt - traj_.global_traj.last_glb_t_of_lc_tgt);
      int piece_nums = ceil((start_pt - local_target_pt).norm() / pp_.polyTraj_piece_length);
      if (piece_nums < 2)
        piece_nums = 2;

      Eigen::Matrix3d headState, tailState;
      Eigen::MatrixXd innerPs(3, piece_nums - 1);
      Eigen::VectorXd piece_dur_vec = Eigen::VectorXd::Constant(piece_nums, t_to_lc_tgt / piece_nums);
      headState << start_pt, start_vel, start_acc;
      tailState << local_target_pt, local_target_vel, Eigen::Vector3d::Zero();

      double t = piece_dur_vec(0);
      for (int i = 0; i < piece_nums - 1; ++i)
      {
        if (t < t_to_lc_end)
        {
          innerPs.col(i) = traj_.local_traj.traj.getPos(t + passed_t_on_lctraj);
        }
        else if (t <= t_to_lc_tgt)
        {
          double glb_t = t - t_to_lc_end + traj_.global_traj.last_glb_t_of_lc_tgt - traj_.global_traj.global_start_time;
          innerPs.col(i) = traj_.global_traj.traj.getPos(glb_t);
        }
        else
        {
          ROS_ERROR("Should not happen! x_x 0x88 t=%.2f, t_to_lc_end=%.2f, t_to_lc_tgt=%.2f", t, t_to_lc_end, t_to_lc_tgt);
        }

        t += piece_dur_vec(i + 1);
      }

      initMJO.reset(headState, tailState, piece_nums);
      initMJO.generate(innerPs, piece_dur_vec);
    }

    return true;
  }

  void EGOPlannerManager::getLocalTarget(
      const double planning_horizen, const Eigen::Vector3d &start_pt,
      const Eigen::Vector3d &global_end_pt, Eigen::Vector3d &local_target_pos,
      Eigen::Vector3d &local_target_vel, bool &touch_goal)
  {
    double t;
    touch_goal = false;

    traj_.global_traj.last_glb_t_of_lc_tgt = traj_.global_traj.glb_t_of_lc_tgt;

    double t_step = planning_horizen / 20 / pp_.max_vel_;
    // double dist_min = 9999, dist_min_t = 0.0;
    for (t = traj_.global_traj.glb_t_of_lc_tgt;
         t < (traj_.global_traj.global_start_time + traj_.global_traj.duration);
         t += t_step)
    {
      Eigen::Vector3d pos_t = traj_.global_traj.traj.getPos(t - traj_.global_traj.global_start_time);
      double dist = (pos_t - start_pt).norm();

      if (dist >= planning_horizen)
      {
        local_target_pos = pos_t;
        traj_.global_traj.glb_t_of_lc_tgt = t;
        break;
      }
    }

    if ((t - traj_.global_traj.global_start_time) >= traj_.global_traj.duration - 1e-5) // Last global point
    {
      local_target_pos = global_end_pt;
      traj_.global_traj.glb_t_of_lc_tgt = traj_.global_traj.global_start_time + traj_.global_traj.duration;
      touch_goal = true;
    }

    if ((global_end_pt - local_target_pos).norm() < (pp_.max_vel_ * pp_.max_vel_) / (2 * pp_.max_acc_))
    {
      local_target_vel = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel = traj_.global_traj.traj.getVel(t - traj_.global_traj.global_start_time);
    }
  }

  bool EGOPlannerManager::setLocalTrajFromOpt(const poly_traj::MinJerkOpt &opt, const bool touch_goal)
  {
    poly_traj::Trajectory traj = opt.getTraj();
    Eigen::MatrixXd cps = opt.getInitConstraintPoints(getCpsNumPrePiece());
    PtsChk_t pts_to_check;
    bool ret = ploy_traj_opt_->computePointsToCheck(traj, ConstraintPoints::two_thirds_id(cps, touch_goal), pts_to_check);
    if (ret)
      traj_.setLocalTraj(traj, pts_to_check, ros::Time::now().toSec());

    return ret;
  }

  bool EGOPlannerManager::checkTrajectoryDynamics(const poly_traj::Trajectory &traj,
                                                  double &max_vel, double &max_acc,
                                                  double &max_jer) const
  {
    max_vel = traj.getMaxVelRate();
    max_acc = traj.getMaxAccRate();
    max_jer = 0.0;

    const double duration = traj.getTotalDuration();
    const double dt = std::max(0.02, std::min(0.05, duration / 100.0));
    for (double t = 0.0; t < duration + 1.0e-6; t += dt)
    {
      const double tt = std::min(t, duration);
      max_jer = std::max(max_jer, traj.getJer(tt).norm());
    }
    max_jer = std::max(max_jer, traj.getJer(duration).norm());

    const double tol = 1.0 + std::max(0.0, pp_.feasibility_tolerance_);
    const bool vel_ok = pp_.max_vel_ <= 0.0 || max_vel <= pp_.max_vel_ * tol;
    const bool acc_ok = pp_.max_acc_ <= 0.0 || max_acc <= pp_.max_acc_ * tol;
    const bool jer_ok = pp_.max_jer_ <= 0.0 || max_jer <= pp_.max_jer_ * tol;
    return vel_ok && acc_ok && jer_ok;
  }

  bool EGOPlannerManager::checkTrajectoryStaticSafety(const poly_traj::Trajectory &traj,
                                                      const bool touch_goal,
                                                      std::string &reason) const
  {
    if (!grid_map_)
      return true;

    const double duration = traj.getTotalDuration();
    const double dt = std::max(
        0.02, std::min(0.05, grid_map_->getResolution() / std::max(0.1, pp_.max_vel_)));

    const double virtual_ceil_height = grid_map_->getVirtualCeilHeight();
    if (virtual_ceil_height > -0.5)
    {
      for (double t = 0.0; t < duration + 1.0e-6; t += dt)
      {
        const double tt = std::min(t, duration);
        const Eigen::Vector3d pos = traj.getPos(tt);
        if (!pos.allFinite() || pos(2) >= virtual_ceil_height)
        {
          std::ostringstream oss;
          oss << "virtual ceiling t=" << tt << " z=" << pos(2)
              << "/" << virtual_ceil_height;
          reason = oss.str();
          return false;
        }
      }
    }

    const double check_end = touch_goal ? duration : duration * 2.0 / 3.0;
    for (double t = 0.0; t < check_end + 1.0e-6; t += dt)
    {
      const double tt = std::min(t, check_end);
      const Eigen::Vector3d pos = traj.getPos(tt);
      const int occupancy = pos.allFinite() ? grid_map_->getInflateOccupancy(pos) : -1;
      if (occupancy != 0)
      {
        std::ostringstream oss;
        oss << "static map t=" << tt << " occupancy=" << occupancy
            << " pos=" << pos.transpose();
        reason = oss.str();
        return false;
      }
    }

    const Eigen::Vector3d end_pos = traj.getPos(check_end);
    const int end_occupancy = end_pos.allFinite() ? grid_map_->getInflateOccupancy(end_pos) : -1;
    if (end_occupancy != 0)
    {
      std::ostringstream oss;
      oss << "static map t=" << check_end << " occupancy=" << end_occupancy
          << " pos=" << end_pos.transpose();
      reason = oss.str();
      return false;
    }
    return true;
  }

  bool EGOPlannerManager::retryTrajectoryDynamics(poly_traj::MinJerkOpt &opt,
                                                  ConstraintPoints &constraints,
                                                  const bool touch_goal,
                                                  std::string &reason)
  {
    double max_vel, max_acc, max_jer;
    if (checkTrajectoryDynamics(opt.getTraj(), max_vel, max_acc, max_jer))
      return true;

    const double base_feasibility_weight = ploy_traj_opt_->getFeasibilityWeight();
    const double base_time_weight = ploy_traj_opt_->getTimeWeight();
    constexpr int max_retries = 3;
    constexpr double weight_scale = 10.0;

    for (int retry = 1; retry <= max_retries; ++retry)
    {
      const poly_traj::Trajectory previous_traj = opt.getTraj();
      const int piece_num = previous_traj.getPieceNum();
      if (piece_num <= 0)
        break;

      double duration_scale = 1.0;
      if (pp_.max_vel_ > 0.0)
        duration_scale = std::max(duration_scale, max_vel / pp_.max_vel_);
      if (pp_.max_acc_ > 0.0)
        duration_scale = std::max(duration_scale, std::sqrt(max_acc / pp_.max_acc_));
      if (pp_.max_jer_ > 0.0)
        duration_scale = std::max(duration_scale, std::cbrt(max_jer / pp_.max_jer_));
      if (!std::isfinite(duration_scale))
        break;

      // The longer MINCO allocation is only a warm start. The retry still jointly
      // optimizes spatial waypoints and time with the paper's increased penalty.
      duration_scale = std::min(2.0, std::max(1.02, duration_scale * 1.02));
      Eigen::VectorXd retry_durations = previous_traj.getDurations() * duration_scale;
      Eigen::MatrixXd all_positions = previous_traj.getPositions();
      Eigen::MatrixXd inner_points = all_positions.block(0, 1, 3, piece_num - 1);
      Eigen::Matrix3d head_state, tail_state;
      head_state << previous_traj.getJuncPos(0), previous_traj.getJuncVel(0), previous_traj.getJuncAcc(0);
      tail_state << previous_traj.getJuncPos(piece_num), previous_traj.getJuncVel(piece_num),
          previous_traj.getJuncAcc(piece_num);

      std::vector<std::pair<int, int>> static_segments;
      if (ploy_traj_opt_->finelyCheckAndSetConstraintPoints(static_segments, opt, true) ==
          PolyTrajOptimizer::CHK_RET::ERR)
      {
        ROS_WARN("[post_check] Static-map retry %d/%d could not build a full-trajectory constraint corridor.",
                 retry, max_retries);
        continue;
      }
      constraints = ploy_traj_opt_->getControlPoints();
      ploy_traj_opt_->setConstraintPoints(constraints);
      ploy_traj_opt_->setFeasibilityWeight(
          base_feasibility_weight * std::pow(weight_scale, retry));
      ploy_traj_opt_->setTimeWeight(
          base_time_weight / std::pow(weight_scale, retry));

      Eigen::MatrixXd retry_points;
      double retry_cost = 0.0;
      const bool retry_success = ploy_traj_opt_->optimizeTrajectory(
          head_state, tail_state, inner_points, retry_durations, retry_points, retry_cost);
      if (!retry_success)
      {
        ROS_WARN("[post_check] Dynamics retry %d/%d optimization failed "
                 "(feas_weight=%.3g, time_weight=%.3g, time_scale=%.3f).",
                 retry, max_retries, ploy_traj_opt_->getFeasibilityWeight(),
                 ploy_traj_opt_->getTimeWeight(), duration_scale);
        continue;
      }

      opt = ploy_traj_opt_->getMinJerkOpt();
      constraints = ploy_traj_opt_->getControlPoints();
      const bool dynamics_ok = checkTrajectoryDynamics(opt.getTraj(), max_vel, max_acc, max_jer);
      ROS_WARN("[post_check] Dynamics retry %d/%d: feas_weight=%.3g time_weight=%.3g time_scale=%.3f "
               "vel=%.6f/%.6f acc=%.6f/%.6f jer=%.6f/%.6f accepted=%d",
               retry, max_retries, ploy_traj_opt_->getFeasibilityWeight(),
               ploy_traj_opt_->getTimeWeight(), duration_scale,
               max_vel, pp_.max_vel_, max_acc, pp_.max_acc_, max_jer, pp_.max_jer_,
               static_cast<int>(dynamics_ok));
      if (dynamics_ok)
      {
        ploy_traj_opt_->setFeasibilityWeight(base_feasibility_weight);
        ploy_traj_opt_->setTimeWeight(base_time_weight);
        return true;
      }
    }

    ploy_traj_opt_->setFeasibilityWeight(base_feasibility_weight);
    ploy_traj_opt_->setTimeWeight(base_time_weight);
    std::ostringstream oss;
    oss << "dynamics after retries vel=" << max_vel << "/" << pp_.max_vel_
        << " acc=" << max_acc << "/" << pp_.max_acc_
        << " jer=" << max_jer << "/" << pp_.max_jer_;
    reason = oss.str();
    return false;
  }

  bool EGOPlannerManager::retryTrajectoryStaticSafety(poly_traj::MinJerkOpt &opt,
                                                      ConstraintPoints &constraints,
                                                      const bool touch_goal,
                                                      std::string &reason)
  {
    if (checkTrajectoryStaticSafety(opt.getTraj(), touch_goal, reason))
      return true;

    const double base_obstacle_weight = ploy_traj_opt_->getObstacleWeight();
    const double base_obstacle_soft_weight = ploy_traj_opt_->getObstacleSoftWeight();
    constexpr int max_retries = 3;
    constexpr double weight_scale = 10.0;

    // Match the optimizer's executable-prefix contract. Only terminal local
    // trajectories require full-trajectory static-map safety.
    ploy_traj_opt_->setIfTouchGoal(touch_goal);
    for (int retry = 1; retry <= max_retries; ++retry)
    {
      const poly_traj::Trajectory previous_traj = opt.getTraj();
      const int piece_num = previous_traj.getPieceNum();
      if (piece_num <= 0)
        break;

      const Eigen::MatrixXd all_positions = previous_traj.getPositions();
      const Eigen::MatrixXd inner_points = all_positions.block(0, 1, 3, piece_num - 1);
      Eigen::Matrix3d head_state, tail_state;
      head_state << previous_traj.getJuncPos(0), previous_traj.getJuncVel(0), previous_traj.getJuncAcc(0);
      tail_state << previous_traj.getJuncPos(piece_num), previous_traj.getJuncVel(piece_num),
          previous_traj.getJuncAcc(piece_num);

      std::vector<std::pair<int, int>> static_segments;
      if (ploy_traj_opt_->finelyCheckAndSetConstraintPoints(static_segments, opt, true) ==
          PolyTrajOptimizer::CHK_RET::ERR)
      {
        ROS_WARN("[post_check] Static-map retry %d/%d could not rebuild prefix constraints.",
                 retry, max_retries);
        continue;
      }
      constraints = ploy_traj_opt_->getControlPoints();
      ploy_traj_opt_->setConstraintPoints(constraints);
      ploy_traj_opt_->setObstacleWeight(base_obstacle_weight * std::pow(weight_scale, retry));
      ploy_traj_opt_->setObstacleSoftWeight(base_obstacle_soft_weight * std::pow(weight_scale, retry));

      Eigen::MatrixXd retry_points;
      double retry_cost = 0.0;
      const bool retry_success = ploy_traj_opt_->optimizeTrajectory(
          head_state, tail_state, inner_points, previous_traj.getDurations(), retry_points, retry_cost);
      if (!retry_success)
      {
        ROS_WARN("[post_check] Static-map retry %d/%d optimization failed (weight=%.3g, soft=%.3g).",
                 retry, max_retries, ploy_traj_opt_->getObstacleWeight(),
                 ploy_traj_opt_->getObstacleSoftWeight());
        continue;
      }

      opt = ploy_traj_opt_->getMinJerkOpt();
      constraints = ploy_traj_opt_->getControlPoints();
      std::string dynamics_reason;
      if (!retryTrajectoryDynamics(opt, constraints, touch_goal, dynamics_reason))
      {
        reason = dynamics_reason;
        ROS_WARN("[post_check] Static-map retry %d/%d failed dynamics: %s",
                 retry, max_retries, dynamics_reason.c_str());
        continue;
      }

      const bool accepted = checkTrajectoryStaticSafety(opt.getTraj(), touch_goal, reason);
      ROS_WARN("[post_check] Static-map retry %d/%d: weight=%.3g soft=%.3g accepted=%d%s%s",
               retry, max_retries, ploy_traj_opt_->getObstacleWeight(),
               ploy_traj_opt_->getObstacleSoftWeight(), static_cast<int>(accepted),
               accepted ? "" : " reason=", accepted ? "" : reason.c_str());
      if (accepted)
      {
        ploy_traj_opt_->setObstacleWeight(base_obstacle_weight);
        ploy_traj_opt_->setObstacleSoftWeight(base_obstacle_soft_weight);
        ploy_traj_opt_->setIfTouchGoal(touch_goal);
        return true;
      }
    }

    ploy_traj_opt_->setObstacleWeight(base_obstacle_weight);
    ploy_traj_opt_->setObstacleSoftWeight(base_obstacle_soft_weight);
    ploy_traj_opt_->setIfTouchGoal(touch_goal);
    return false;
  }

  bool EGOPlannerManager::checkTrajectorySwarmSafety(const poly_traj::Trajectory &traj,
                                                     const double start_time,
                                                     const bool touch_goal,
                                                     std::string &reason) const
  {
    if (traj_.swarm_traj.empty() || pp_.drone_id < 0)
      return true;

    // The optimizer uses larger 1.25/1.5 buffers. Post-check enforces the
    // paper's actual Cw safety constraint, not the optimizer's soft buffer.
    const double clearance = getSwarmClearance();
    const double clearance2 = clearance * clearance;
    const double duration = traj.getTotalDuration();
    const double check_end = touch_goal ? duration : duration * 2.0 / 3.0;
    constexpr double inv_vertical_axis2 = 0.25;

    for (double t = 0.0; t < check_end + 1.0e-6; t += 0.03)
    {
      const double tt = std::min(t, check_end);
      const Eigen::Vector3d pos = traj.getPos(tt);
      const double global_time = start_time + tt;

      for (size_t id = 0; id < traj_.swarm_traj.size(); ++id)
      {
        const LocalTrajData &other = traj_.swarm_traj.at(id);
        if (other.drone_id < 0 || other.drone_id == pp_.drone_id || other.duration <= 0.0)
          continue;

        double other_t = global_time - other.start_time;
        Eigen::Vector3d other_pos;
        if (other_t <= other.duration)
        {
          other_t = std::max(0.0, other_t);
          other_pos = other.traj.getPos(other_t);
        }
        else
        {
          const Eigen::Vector3d end_vel = other.traj.getVel(other.duration);
          other_pos = other.traj.getPos(other.duration) + (other_t - other.duration) * end_vel;
        }

        const Eigen::Vector3d delta = pos - other_pos;
        const double ellip_dist2 = delta.head<2>().squaredNorm() +
                                   delta(2) * delta(2) * inv_vertical_axis2;
        if (ellip_dist2 < clearance2)
        {
          std::ostringstream oss;
          oss << "swarm drone=" << other.drone_id << " t=" << tt
              << " ellip_dist2=" << ellip_dist2;
          reason = oss.str();
          return false;
        }
      }
    }

    return true;
  }

  bool EGOPlannerManager::retryTrajectorySwarm(poly_traj::MinJerkOpt &opt,
                                               ConstraintPoints &constraints,
                                               const bool touch_goal,
                                               std::string &reason)
  {
    if (checkTrajectorySwarmSafety(opt.getTraj(), ros::Time::now().toSec(), touch_goal, reason))
      return true;

    const double base_swarm_weight = ploy_traj_opt_->getSwarmWeight();
    constexpr int max_retries = 3;
    constexpr double weight_scale = 10.0;

    for (int retry = 1; retry <= max_retries; ++retry)
    {
      const poly_traj::Trajectory previous_traj = opt.getTraj();
      const int piece_num = previous_traj.getPieceNum();
      if (piece_num <= 0)
        break;

      const Eigen::MatrixXd all_positions = previous_traj.getPositions();
      const Eigen::MatrixXd inner_points = all_positions.block(0, 1, 3, piece_num - 1);
      Eigen::Matrix3d head_state, tail_state;
      head_state << previous_traj.getJuncPos(0), previous_traj.getJuncVel(0), previous_traj.getJuncAcc(0);
      tail_state << previous_traj.getJuncPos(piece_num), previous_traj.getJuncVel(piece_num),
          previous_traj.getJuncAcc(piece_num);

      ploy_traj_opt_->setConstraintPoints(constraints);
      ploy_traj_opt_->setSwarmWeight(base_swarm_weight * std::pow(weight_scale, retry));

      Eigen::MatrixXd retry_points;
      double retry_cost = 0.0;
      const bool retry_success = ploy_traj_opt_->optimizeTrajectory(
          head_state, tail_state, inner_points, previous_traj.getDurations(), retry_points, retry_cost);
      if (!retry_success)
      {
        ROS_WARN("[post_check] Swarm retry %d/%d optimization failed (weight=%.3g).",
                 retry, max_retries, ploy_traj_opt_->getSwarmWeight());
        continue;
      }

      opt = ploy_traj_opt_->getMinJerkOpt();
      constraints = ploy_traj_opt_->getControlPoints();
      if (!retryTrajectoryDynamics(opt, constraints, touch_goal, reason) ||
          !retryTrajectoryStaticSafety(opt, constraints, touch_goal, reason))
      {
        ROS_WARN("[post_check] Swarm retry %d/%d failed another safety constraint: %s",
                 retry, max_retries, reason.c_str());
        continue;
      }

      const bool accepted = checkTrajectorySwarmSafety(
          opt.getTraj(), ros::Time::now().toSec(), touch_goal, reason);
      ROS_WARN("[post_check] Swarm retry %d/%d: weight=%.3g accepted=%d%s%s",
               retry, max_retries, ploy_traj_opt_->getSwarmWeight(), static_cast<int>(accepted),
               accepted ? "" : " reason=", accepted ? "" : reason.c_str());
      if (accepted)
      {
        ploy_traj_opt_->setSwarmWeight(base_swarm_weight);
        return true;
      }
    }

    ploy_traj_opt_->setSwarmWeight(base_swarm_weight);
    return false;
  }

  bool EGOPlannerManager::reboundReplan(
      const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const Eigen::Vector3d &local_target_pt,
      const Eigen::Vector3d &local_target_vel, const Eigen::Vector3d &object_pt,
      const Eigen::Vector3d &object_vel,  const Eigen::Quaterniond &object_q, 
      const Eigen::Vector3d &relative_track_pt, const bool flag_polyInit, 
      const bool flag_randomPolyTraj, const bool touch_goal, const bool satrt_tracking)
  {

    static int count = 0;
    printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n",
           pp_.drone_id, count++);
    // cout.precision(3);
    // cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
    //      << endl;

    ploy_traj_opt_->setIfTouchGoal(touch_goal);
    ploy_traj_opt_->setObject(object_pt, object_vel, object_q);
    ploy_traj_opt_->setRelativeTrackingP(relative_track_pt);
    ploy_traj_opt_->setStartTracking(satrt_tracking);

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      // continous_failures_count_++;
      // return false;
    }

    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt;

    /*** STEP 1: INIT ***/
    double ts = pp_.polyTraj_piece_length / pp_.max_vel_;

    poly_traj::MinJerkOpt initMJO;
    if (!computeInitState(start_pt, start_vel, start_acc, local_target_pt, local_target_vel,
                          flag_polyInit, flag_randomPolyTraj, ts, initMJO))
    {
      return false;
    }

    Eigen::MatrixXd cstr_pts = initMJO.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
    vector<std::pair<int, int>> segments;
    if (ploy_traj_opt_->finelyCheckAndSetConstraintPoints(segments, initMJO, true) == PolyTrajOptimizer::CHK_RET::ERR)
    {
      return false;
    }

    t_init = ros::Time::now() - t_start;

    std::vector<Eigen::Vector3d> point_set;
    for (int i = 0; i < cstr_pts.cols(); ++i)
      point_set.push_back(cstr_pts.col(i));
    visualization_->displayInitPathList(point_set, 0.2, 0);

    t_start = ros::Time::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_success = false;
    vector<vector<Eigen::Vector3d>> vis_trajs;
    poly_traj::MinJerkOpt best_MJO;
    ConstraintPoints best_constraints;

    if (pp_.use_distinctive_trajs)
    {
      std::vector<ConstraintPoints> trajs = ploy_traj_opt_->distinctiveTrajs(segments);
      cout << "\033[1;33m"
           << "multi-trajs=" << trajs.size() << "\033[1;0m" << endl;

      poly_traj::Trajectory initTraj = initMJO.getTraj();
      int PN = initTraj.getPieceNum();
      Eigen::MatrixXd all_pos = initTraj.getPositions();
      Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
      Eigen::Matrix<double, 3, 3> headState, tailState;
      headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
      tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
      double final_cost, min_cost = 999999.0;
      for (int i = trajs.size() - 1; i >= 0; i--)
      {
        ploy_traj_opt_->setConstraintPoints(trajs[i]);
        if (ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                               innerPts, initTraj.getDurations(),
                                               cstr_pts, final_cost))
        {

          cout << "traj " << trajs.size() - i << " success." << endl;

          if (final_cost < min_cost)
          {
            min_cost = final_cost;
            best_MJO = ploy_traj_opt_->getMinJerkOpt();
            best_constraints = ploy_traj_opt_->getControlPoints();
            flag_success = true;
          }

          // visualization
          Eigen::MatrixXd ctrl_pts_temp = ploy_traj_opt_->getMinJerkOpt().getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
          std::vector<Eigen::Vector3d> point_set;
          for (int j = 0; j < ctrl_pts_temp.cols(); j++)
          {
            point_set.push_back(ctrl_pts_temp.col(j));
          }
          vis_trajs.push_back(point_set);
        }
        else
        {
          cout << "traj " << trajs.size() - i << " failed." << endl;
        }
      }

      t_opt = ros::Time::now() - t_start;

      visualization_->displayMultiInitPathList(vis_trajs, 0.2); // This visuallization will take up several milliseconds.
    }
    else
    {
      poly_traj::Trajectory initTraj = initMJO.getTraj();
      int PN = initTraj.getPieceNum();
      Eigen::MatrixXd all_pos = initTraj.getPositions();
      Eigen::MatrixXd innerPts = all_pos.block(0, 1, 3, PN - 1);
      Eigen::Matrix<double, 3, 3> headState, tailState;
      headState << initTraj.getJuncPos(0), initTraj.getJuncVel(0), initTraj.getJuncAcc(0);
      tailState << initTraj.getJuncPos(PN), initTraj.getJuncVel(PN), initTraj.getJuncAcc(PN);
      double final_cost;
      flag_success = ploy_traj_opt_->optimizeTrajectory(headState, tailState,
                                                        innerPts, initTraj.getDurations(),
                                                        cstr_pts, final_cost);
      best_MJO = ploy_traj_opt_->getMinJerkOpt();
      best_constraints = ploy_traj_opt_->getControlPoints();

      t_opt = ros::Time::now() - t_start;
    }

    // // save and display planned results
    cout << "plan_success=" << flag_success << endl;
    if (!flag_success)
    {
      visualization_->displayFailedList(cstr_pts, 0);
      continous_failures_count_++;
      return false;
    }

    static double sum_time = 0;
    static int count_success = 0;
    sum_time += (t_init + t_opt).toSec();
    count_success++;
    cout << "total time:\033[42m" << (t_init + t_opt).toSec()
         << "\033[0m,init:" << t_init.toSec()
         << ",optimize:" << t_opt.toSec()
         << ",avg_time=" << sum_time / count_success << endl;

    if (enable_post_checks_)
    {
      std::string post_check_reason;
      if (!retryTrajectoryDynamics(best_MJO, best_constraints, touch_goal, post_check_reason))
      {
        ROS_WARN("[post_check] Reject unsafe trajectory: %s", post_check_reason.c_str());
        visualization_->displayFailedList(cstr_pts, 0);
        continous_failures_count_++;
        return false;
      }

      if (!retryTrajectoryStaticSafety(best_MJO, best_constraints, touch_goal, post_check_reason))
      {
        ROS_WARN("[post_check] Reject unsafe trajectory: %s", post_check_reason.c_str());
        visualization_->displayFailedList(cstr_pts, 0);
        continous_failures_count_++;
        return false;
      }

      if (!retryTrajectorySwarm(best_MJO, best_constraints, touch_goal, post_check_reason))
      {
        ROS_WARN("[post_check] Reject unsafe trajectory: %s", post_check_reason.c_str());
        visualization_->displayFailedList(cstr_pts, 0);
        continous_failures_count_++;
        return false;
      }

      cstr_pts = best_MJO.getInitConstraintPoints(ploy_traj_opt_->get_cps_num_prePiece_());
    }

    setLocalTrajFromOpt(best_MJO, touch_goal);
    visualization_->displayOptimalList(cstr_pts, 0);

    // success. YoY
    continous_failures_count_ = 0;
    return true;
  }

  void EGOPlannerManager::setFovSoftScale(const double scale)
  {
    ploy_traj_opt_->setFovSoftScale(scale);
  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    auto ZERO = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << stop_pos, ZERO, ZERO;
    tailState = headState;
    poly_traj::MinJerkOpt stopMJO;
    stopMJO.reset(headState, tailState, 2);
    stopMJO.generate(stop_pos, Eigen::Vector2d(1.0, 1.0));

    setLocalTrajFromOpt(stopMJO, false);

    return true;
  }

  bool EGOPlannerManager::checkCollision(int drone_id)
  {
    if (traj_.local_traj.start_time < 1e9) // It means my first planning has not started
      return false;

    double my_traj_start_time = traj_.local_traj.start_time;
    double other_traj_start_time = traj_.swarm_traj[drone_id].start_time;

    double t_start = max(my_traj_start_time, other_traj_start_time);
    double t_end = min(my_traj_start_time + traj_.local_traj.duration * 2 / 3,
                       other_traj_start_time + traj_.swarm_traj[drone_id].duration);

    for (double t = t_start; t < t_end; t += 0.03)
    {
      if ((traj_.local_traj.traj.getPos(t - my_traj_start_time) -
           traj_.swarm_traj[drone_id].traj.getPos(t - other_traj_start_time))
              .norm() < getSwarmClearance())
      {
        return true;
      }
    }

    return false;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(
      const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel,
      const Eigen::Vector3d &start_acc, const std::vector<Eigen::Vector3d> &waypoints,
      const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    poly_traj::MinJerkOpt globalMJO;
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << start_pos, start_vel, start_acc;
    tailState << waypoints.back(), end_vel, end_acc;
    Eigen::MatrixXd innerPts;

    if (waypoints.size() > 1)
    {

      innerPts.resize(3, waypoints.size() - 1);
      for (int i = 0; i < (int)waypoints.size() - 1; ++i)
      {
        innerPts.col(i) = waypoints[i];
      }
    }
    else
    {
      if (innerPts.size() != 0)
      {
        ROS_ERROR("innerPts.size() != 0");
      }
    }

    globalMJO.reset(headState, tailState, waypoints.size());

    double des_vel = pp_.max_vel_ / 1.5;
    Eigen::VectorXd time_vec(waypoints.size());

    for (int j = 0; j < 2; ++j)
    {
      for (size_t i = 0; i < waypoints.size(); ++i)
      {
        time_vec(i) = (i == 0) ? (waypoints[0] - start_pos).norm() / des_vel
                               : (waypoints[i] - waypoints[i - 1]).norm() / des_vel;
      }

      globalMJO.generate(innerPts, time_vec);

      if (globalMJO.getTraj().getMaxVelRate() < pp_.max_vel_ ||
          start_vel.norm() > pp_.max_vel_ ||
          end_vel.norm() > pp_.max_vel_)
      {
        break;
      }

      if (j == 2)
      {
        ROS_WARN("Global traj MaxVel = %f > set_max_vel", globalMJO.getTraj().getMaxVelRate());
        cout << "headState=" << endl
             << headState << endl;
        cout << "tailState=" << endl
             << tailState << endl;
      }

      des_vel /= 1.5;
    }

    auto time_now = ros::Time::now();
    traj_.setGlobalTraj(globalMJO.getTraj(), time_now.toSec());

    return true;
  }

} // namespace ego_planner
