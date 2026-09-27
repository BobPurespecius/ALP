from pathlib import Path
import shutil
base=Path('ros_ws/src/EGO-Planner-v2/swarm-playground/tracking_ws/src/planner/plan_manage')
p=base/'src/planner_manager.cpp'; h=base/'include/plan_manage/planner_manager.h'
s=Path('feedback/artifacts/feedback134_before_planner_manager.cpp').read_text(); hs=Path('feedback/artifacts/feedback134_before_planner_manager.h').read_text()
def rep(old,new,n=1):
 global s
 assert s.count(old)==n,(old[:100],s.count(old),n)
 s=s.replace(old,new)
hs=hs.replace('  private:\n    trajectory_lifecycle::State currentStateAt', '''  private:
    // Optional, read-only audit sink; never consumed by planning decisions.
    void traceTopologyAudit134(const char *stage, const CandidateResult *candidate,
        const poly_traj::Trajectory *trajectory = nullptr,
        const std::string &reason = "", int related_id = -1,
        double comparison_begin = 0.0, double comparison_end = 0.0) const;
    trajectory_lifecycle::State currentStateAt''')
helper=r'''
  // Feedback134: opt-in diagnostic payload capture. No geometry evaluation,
  // solver calls, new gates, or writes to planner/candidate state occur here.
  void EGOPlannerManager::traceTopologyAudit134(
      const char *stage, const CandidateResult *candidate,
      const poly_traj::Trajectory *trajectory, const std::string &reason,
      int related_id, double comparison_begin, double comparison_end) const
  {
    static const char *directory = std::getenv("ALP_TOPOLOGY_AUDIT_DIR");
    if (!directory || !*directory || geometry_target_v_.norm() < 0.1) return;
    static std::ofstream output(std::string(directory) + "/candidate_drone_" +
        std::to_string(pp_.drone_id) + ".jsonl", std::ios::app);
    if (!output) return;
    const auto started = std::chrono::steady_clock::now();
    static double cumulative_us = 0.0;
    std::ostringstream o;
    o << std::setprecision(17);
    const auto number = [&](double x) { if (std::isfinite(x)) o << x; else o << "null"; };
    const auto vector = [&](const Eigen::Vector3d &v) {
      o << '['; for (int i=0;i<3;++i) { if(i) o<<',';number(v(i)); } o << ']';
    };
    o << "{\"stage\":" << std::quoted(stage) << ",\"reason\":" << std::quoted(reason)
      << ",\"drone\":" << pp_.drone_id << ",\"world\":";
    number(ros::Time::now().toSec()); o << ",\"batch\":";number(local_planning_started_);
    o << ",\"activation\":";number(local_activation_time_);
    o << ",\"wall_remaining\":";number(planning_deadline_wall_-ros::WallTime::now().toSec());
    o << ",\"event\":" << k3_event_.escalation_id << ",\"event_active\":" << k3_event_.active
      << ",\"cycle\":" << k3_event_.cycle_count << ",\"related_id\":" << related_id;
    o << ",\"comparison_begin\":";number(comparison_begin);
    o << ",\"comparison_end\":";number(comparison_end);
    o << ",\"self_yaw\":";number(visibility_odom_[pp_.drone_id].yaw);
    o << ",\"self_yaw_rate\":";number(visibility_odom_[pp_.drone_id].yaw_rate);
    o << ",\"self_odom_stamp\":";number(visibility_odom_[pp_.drone_id].stamp);
    o << ",\"target_epoch\":";number(geometry_target_stamp_);
    o << ",\"target\":";vector(geometry_target_p_);o << ",\"target_v\":";vector(geometry_target_v_);
    o << ",\"baseline_valid\":" << k3_baseline_valid_ << ",\"baseline_k2\":";number(k3_baseline_window_k2_);
    o << ",\"baseline_c3\":";number(k3_baseline_window_c3_);o << ",\"baseline_d3\":";number(k3_baseline_window_d3_);
    o << ",\"writer_cumulative_us\":";number(cumulative_us);
    if (candidate) {
      const auto &c=*candidate; const auto &f=c.conflict;
      o << ",\"candidate\":" << c.candidate_id << ",\"kind\":" << int(c.kind)
        << ",\"success\":" << c.success << ",\"revision\":" << c.execution_revision
        << ",\"hash\":" << c.validated_payload_hash << ",\"fallback\":" << c.feasible_initializer_fallback
        << ",\"class\":" << int(c.safety_class) << ",\"static_valid\":" << c.static_valid
        << ",\"dynamics_valid\":" << c.dynamics_valid << ",\"swarm_valid\":" << c.swarm_valid
        << ",\"astar\":" << c.astar_used << ",\"sfc\":" << c.local_sfc_used
        << ",\"mask\":" << f.reason_mask << ",\"blocker\":" << f.obstacle_identity
        << ",\"motion\":" << int(f.obstacle_motion) << ",\"los_blocker\":" << f.los_obstacle_identity
        << ",\"los_motion\":" << int(f.los_obstacle_motion);
      o << ",\"blocker_p\":";vector(f.obstacle_position);
      o << ",\"los_blocker_p\":";vector(f.los_obstacle_position);
      o << ",\"frame_forward\":";vector(f.frame_forward);o << ",\"frame_right\":";vector(f.frame_right);
      o << ",\"witness_target\":";vector(f.target_position);o << ",\"witness_observer\":";vector(f.observer_position);
      o << ",\"los_first_risk_world\":";number(f.los_first_risk_world_time);
      o << ",\"checked_until\":";number(c.checked_until);
      o << ",\"k3_valid\":" << c.k3_window.valid << ",\"K2\":";number(c.k3_window.k2);
      o << ",\"C3\":";number(c.k3_window.c3);o << ",\"D3\":";number(c.k3_window.d3);
      o << ",\"max_v\":";number(c.max_velocity);o << ",\"max_a\":";number(c.max_acceleration);
      o << ",\"max_j\":";number(c.max_jerk);
    }
    if (trajectory) {
      o << ",\"poly_hash\":" << trajectoryPayloadHash(*trajectory) << ",\"pieces\":[";
      for(int i=0;i<trajectory->getPieceNum();++i) {
        if(i) o<<',';
        o<<'[';number((*trajectory)[i].getDuration());
        const auto &a=(*trajectory)[i].getCoeffMat();
        for(int row=0;row<3;++row) for(int col=0;col<6;++col) {o<<',';number(a(row,col));}
        o<<']';
      }
      o << ']';
    }
    o << "}\n";
    output << o.str();
    cumulative_us += std::chrono::duration<double,std::micro>(
        std::chrono::steady_clock::now()-started).count();
  }

'''
rep('  void EGOPlannerManager::finishPlanningBatch(double wall_started)',helper+'  void EGOPlannerManager::finishPlanningBatch(double wall_started)')
# Complete finalized input list even if a later loop breaks for deadline.
rep('    selected_set=-1;\n    // FROZEN-ACTIVATION', '''    selected_set=-1;
    for (const auto &audit_set : sets) for (const auto &audit_candidate : audit_set.candidates) {
      if (audit_candidate.success) {
        const auto audit_traj = audit_candidate.min_jerk_opt.getTraj();
        traceTopologyAudit134("FINAL_INPUT", &audit_candidate, &audit_traj);
      } else traceTopologyAudit134("FINAL_INPUT_FAILED", &audit_candidate);
      if (audit_candidate.joint_seed_valid) {
        const auto audit_seed = audit_candidate.joint_seed.getTraj();
        traceTopologyAudit134("JOINT_SEED", &audit_candidate, &audit_seed);
      }
    }
    // FROZEN-ACTIVATION''')
rep('          deadline_stopped_quality=true;','          traceTopologyAudit134("FINALIZE_DEADLINE_STOP", &candidate, nullptr, "QUALITY_SEARCH_ABORTED_FOR_DEADLINE");\n          deadline_stopped_quality=true;')
rep('          ROS_INFO("[moving-rehead-reject] drone=%d candidate=%d reason=%s",','          traceTopologyAudit134("HANDOFF_REJECT", &candidate, nullptr, handoff_reason);\n          ROS_INFO("[moving-rehead-reject] drone=%d candidate=%d reason=%s",')
rep('          ROS_INFO("[moving-candidate-reject] drone=%d candidate=%d reason=SHORT_STATIONARY_HYPOTHESIS', '          traceTopologyAudit134("STATIONARY_REJECT", &candidate);\n          ROS_INFO("[moving-candidate-reject] drone=%d candidate=%d reason=SHORT_STATIONARY_HYPOTHESIS')
rep('        if(!safe) {\n          ROS_INFO("[local-geometry-candidate]', '        if(!safe) {\n          traceTopologyAudit134("PREFLIGHT_REJECT", &candidate, nullptr, reason);\n          ROS_INFO("[local-geometry-candidate]')
rep('          ROS_WARN("[CANDIDATE_CERTIFICATE] candidate=%d status=UNKNOWN ', '          traceTopologyAudit134("CERTIFICATE_REJECT", &candidate);\n          ROS_WARN("[CANDIDATE_CERTIFICATE] candidate=%d status=UNKNOWN ')
rep('        if(candidate_better)\n        {','''        traceTopologyAudit134(candidate_better ? "COMPARATOR_WIN" : "COMPARATOR_LOSS",
            &candidate, nullptr, k3_decision_reason, best ? best->candidate_id : -1,
            k3_comparison_begin, k3_comparison_end);
        if(candidate_better)
        {''')
rep('      const auto status=finalizeTopologyCandidate(*best,sets[selected_set].nominal_risk,','''      traceTopologyAudit134("SELECTED", best, nullptr, k3_selected_reason, -1,
          k3_comparison_begin, k3_comparison_end);
      const auto status=finalizeTopologyCandidate(*best,sets[selected_set].nominal_risk,''')
rep('          false,"LOCAL_GEOMETRY_PRESERVATION_OR_RECOVERY");','''          false,"LOCAL_GEOMETRY_PRESERVATION_OR_RECOVERY");
      traceTopologyAudit134(status == TopologyProcessStatus::COMMITTED ? "COMMITTED" : "COMMIT_FAILED",
          best, nullptr, "", traj_.local_traj.traj_id, k3_comparison_begin, k3_comparison_end);''')
# Conflict snapshot before dispatch and raw seeds before PVA timing.
rep('          result.conflict = nominal_result.conflict;','''          result.conflict = nominal_result.conflict;
          traceTopologyAudit134("SIDE_DISPATCH", &result);''')
rep('              [&](const char *reason, const poly_traj::Trajectory *candidate_traj) {','''              [&](const char *reason, const poly_traj::Trajectory *candidate_traj) {
                traceTopologyAudit134("PREINIT_REJECT", &result, candidate_traj, reason);''')
rep('            const poly_traj::Trajectory side_init_candidate_traj = side_init_mjo.getTraj();','''            const poly_traj::Trajectory side_init_candidate_traj = side_init_mjo.getTraj();
            traceTopologyAudit134("RAW_SEED_ATTEMPT", &result, &side_init_candidate_traj,
                use_observation_frame ? "OBSERVATION_FRAME" : "PATH_FRAME");''')
rep('          poly_traj::Trajectory side_init_traj = side_init_mjo.getTraj();','''          poly_traj::Trajectory side_init_traj = side_init_mjo.getTraj();
          traceTopologyAudit134("PRE_TIMING_SEED", &result, &side_init_traj);''')
rep('          side_init_traj = side_init_mjo.getTraj();','''          side_init_traj = side_init_mjo.getTraj();
          traceTopologyAudit134("TIMED_SEED", &result, &side_init_traj);''')
rep('            minco_status = ploy_traj_opt_->getLastCandidateFinalStatusReason();','''            minco_status = ploy_traj_opt_->getLastCandidateFinalStatusReason();
            {
              const auto audit_solver_traj = ploy_traj_opt_->getMinJerkOpt().getTraj();
              traceTopologyAudit134(result.success ? "SOLVER_OK" : "SOLVER_FAILED",
                  &result, &audit_solver_traj, minco_status);
            }''')
rep('        const auto attempt_side = [&](const int side, const bool mandatory_supply) {','''        const auto audit_nominal_traj = nominal_result.min_jerk_opt.getTraj();
        traceTopologyAudit134("NOMINAL_CONTEXT", &nominal_result, &audit_nominal_traj);
        const auto attempt_side = [&](const int side, const bool mandatory_supply) {''')
rep('          if (std::isfinite(planning_deadline_wall_) && wall_remaining < 0.02)\n          {','''          if (std::isfinite(planning_deadline_wall_) && wall_remaining < 0.02)
          {
            traceTopologyAudit134("SIDE_BUDGET_SKIP", &nominal_result, nullptr,
                side > 0 ? "SIDE_PLUS" : "SIDE_MINUS");''')
rep('            if (feasible_initializer_available)\n            {','''            traceTopologyAudit134("SEED_ELIGIBILITY", &result, nullptr,
                std::string("static=") + std::to_string(initializer_static_ok) +
                ";dynamics=" + std::to_string(initializer_dynamics_ok) +
                ";sfc=" + std::to_string(initializer_local_sfc_ok) +
                ";class=" + candidateSafetyClassName(feasible_initializer_class),
                static_cast<int>(feasible_initializer_available));
            if (feasible_initializer_available)
            {''')
p.write_text(s);h.write_text(hs)
print('Telemetry only edits written',p,h)
