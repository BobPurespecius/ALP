#include <traj_utils/trajectory_lifecycle.h>
#include <nav_msgs/Odometry.h>
#include <traj_utils/PolyTraj.h>
#include <traj_utils/poly_payload_identity.h>
#include <traj_utils/TeamTrajectoryAck.h>
#include <traj_utils/TeamTrajectorySolution.h>
#include <optimizer/poly_traj_utils.hpp>
#include <multi_uav_formation/tracking_visibility_geometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sstream>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <mutex>

using namespace Eigen;

ros::Publisher pos_cmd_pub;
ros::Publisher tracking_ready_pub;
ros::Publisher execution_source_pub, execution_adoption_pub;
ros::Publisher team_trajectory_ack_pub;
traj_utils::PolyTraj active_source_, scheduled_source_;
int source_reported_traj_id_=-1;
std::string reported_source_;
bool reported_safety_validated_=false;


quadrotor_msgs::PositionCommand cmd;
// double pos_gain[3] = {0, 0, 0};
// double vel_gain[3] = {0, 0, 0};

bool receive_traj_ = false;
boost::shared_ptr<poly_traj::Trajectory> traj_;
double traj_duration_ = 0.0;
ros::Time start_time_;
int traj_id_ = -1;
boost::shared_ptr<poly_traj::Trajectory> scheduled_traj_;
double scheduled_traj_duration_ = 0.0;
ros::Time scheduled_start_time_;
int scheduled_traj_id_ = -1;
bool scheduled_traj_valid_ = false;
struct ScheduledTrajectoryEntry
{
  traj_utils::PolyTraj source;
  boost::shared_ptr<poly_traj::Trajectory> traj;
  ros::Time start_time;
  double duration = 0.0;
  double checked_until = 0.0;
  int traj_id = -1;
  bool optimized_yaw_valid = false;
  std::vector<double> yaw_sample_times;
  std::vector<double> yaw_samples;
  // Feedback093 lineage contract carried by this future reservation.
  int expected_predecessor_id = -1;
  uint64_t expected_predecessor_generation = 0;
  // True == this entry is a *speculative* Team replacement.  Speculative
  // entries may be discarded by a pre-activation Team ABORT; Local guaranteed
  // entries may never be discarded by any Team transaction.
  bool team_speculative = false;
};
std::deque<ScheduledTrajectoryEntry> scheduled_traj_queue_;
ScheduledTrajectoryEntry prepared_team_trajectory_;
bool prepared_team_trajectory_valid_ = false;

// Forward declarations: the admission helpers are defined next to the lineage
// registry further down but are used by the Team callbacks above them.
void clearScheduledSlot();
void normalizeFutureTimeline();

/* ===========================================================================
 * Feedback093 trajectory lifecycle: three distinct authorities.
 *
 *   A. executable polynomial producer ... Local Planner (planner_manager)
 *   B. future reservation authority ..... Team execution transaction
 *   C. authoritative executed history ... traj_server ACTIVATE (this file)
 *
 * Only (C) decides who the authoritative predecessor is.  A Team trajectory
 * that is REALIZED / CERTIFIED / PREPARED / COMMITTED_FUTURE is *speculative*:
 * it holds a future execution slot but is nobody's predecessor until it is
 * actually activated.  `lineage_records_` is the only place an identity may be
 * resolved, and it separates "known but invalidated" from "never seen", so an
 * identity error is never again reported as a geometry (PVA) error.
 * ======================================================================== */
constexpr int kTeamSpeculativeTrajectoryIdBase = 1000000;
constexpr size_t kLineageHistoryLimit = 96;
constexpr const char *kLineageClassLocal = "LOCAL";
constexpr const char *kLineageClassTeam = "TEAM";

enum class LineageResolution
{
  FOUND = 0,
  INVALIDATED = 1,
  NOT_AVAILABLE = 2
};

struct LineageRecord
{
  int traj_id = -1;
  uint64_t generation = 0;
  std::string source;
  uint64_t team_solution_id = 0;
  bool team_speculative = false;
  bool live = false;
  bool invalidated = false;
  bool retired = false;
  double start_time = 0.0;
  double duration = 0.0;
  double checked_until = 0.0;
  boost::shared_ptr<poly_traj::Trajectory> traj;
};
std::deque<LineageRecord> lineage_records_;
uint64_t silent_predecessor_fallback_count_ = 0;
uint64_t explicit_predecessor_resolve_count_ = 0;
uint64_t team_speculative_abort_count_ = 0;
uint64_t lineage_identity_error_count_ = 0;

LineageRecord *findLineageRecord(int traj_id)
{
  for (auto &record : lineage_records_)
    if (record.traj_id == traj_id)
      return &record;
  return nullptr;
}

void noteLineageRecord(const traj_utils::PolyTraj &source,
                       const boost::shared_ptr<poly_traj::Trajectory> &traj,
                       const bool live)
{
  if (source.traj_id < 0)
    return;
  LineageRecord *existing = findLineageRecord(source.traj_id);
  if (existing)
  {
    existing->generation = static_cast<uint64_t>(source.generation);
    existing->source = source.trajectory_source;
    existing->team_solution_id = static_cast<uint64_t>(source.team_solution_id);
    existing->team_speculative = source.lineage_class == kLineageClassTeam;
    existing->start_time = source.start_time.toSec();
    existing->duration =
        traj ? traj->getTotalDuration() : existing->duration;
    existing->checked_until = source.checked_until;
    existing->traj = traj;
    existing->live = live;
    if (live)
      existing->invalidated = false;
    return;
  }
  LineageRecord record;
  record.traj_id = source.traj_id;
  record.generation = static_cast<uint64_t>(source.generation);
  record.checked_until = source.checked_until;
  record.source = source.trajectory_source;
  record.team_solution_id = static_cast<uint64_t>(source.team_solution_id);
  record.team_speculative = source.lineage_class == kLineageClassTeam;
  record.live = live;
  record.start_time = source.start_time.toSec();
  record.duration = traj ? traj->getTotalDuration() : 0.0;
  record.traj = traj;
  lineage_records_.push_back(record);
  while (lineage_records_.size() > kLineageHistoryLimit)
    lineage_records_.pop_front();
}

void markLineageInvalidated(const int traj_id)
{
  LineageRecord *record = findLineageRecord(traj_id);
  if (!record)
    return;
  record->live = false;
  record->invalidated = true;
}

// A trajectory that has been superseded as ACTIVE was nevertheless real
// history, so payloads that declare it as their predecessor stay resolvable at
// admission: rejecting them there would break the ordinary A -> L chain, where
// every rolling revision is anchored to the same still-covering active
// trajectory.  Whether such a payload may actually take over is a *physical*
// question answered at ACTIVATION by the live-continuity check, which is
// reported as ACTIVATION_HANDOFF_DISCONTINUITY rather than as an identity
// error.  Retirement is therefore telemetry, not invalidation.
void retireLineageRecord(const int traj_id)
{
  LineageRecord *record = findLineageRecord(traj_id);
  if (!record)
    return;
  record->retired = true;
}

LineageResolution resolveLineage(const int traj_id, LineageRecord **out)
{
  LineageRecord *record = findLineageRecord(traj_id);
  if (record && record->live && record->traj && record->traj->getPieceNum() > 0)
  {
    if (out)
      *out = record;
    return LineageResolution::FOUND;
  }
  if (out)
    *out = nullptr;
  return record ? LineageResolution::INVALIDATED
                : LineageResolution::NOT_AVAILABLE;
}

const char *lineageResolutionName(const LineageResolution resolution)
{
  switch (resolution)
  {
    case LineageResolution::FOUND: return "FOUND";
    case LineageResolution::INVALIDATED: return "INVALIDATED";
    default: return "NOT_AVAILABLE";
  }
}

void publishLineage(const char *state, const int traj_id,
                    const int expected_predecessor_id, const char *lineage_class,
                    const uint64_t team_solution_id)
{
  ROS_INFO("[trajectory-lineage] trajectory_id=%d expected_predecessor_id=%d "
           "source=%s transaction_id=%lu state=%s",
           traj_id, expected_predecessor_id, lineage_class,
           static_cast<unsigned long>(team_solution_id), state);
}

bool optimized_yaw_valid_ = false;
bool optimized_yaw_execution_reported_ = false;
std::vector<double> optimized_yaw_sample_times_;
std::vector<double> optimized_yaw_samples_;
bool scheduled_optimized_yaw_valid_ = false;
std::vector<double> scheduled_yaw_sample_times_;
std::vector<double> scheduled_yaw_samples_;
int scheduled_expected_predecessor_id_ = -1;
uint64_t scheduled_expected_predecessor_generation_ = 0;
bool scheduled_team_speculative_ = false;
ros::Time heartbeat_time_(0);
bool heartbeat_stale_ = false;
bool terminal_hold_active_ = false;
double terminal_hold_started_ = 0.0;
trajectory_lifecycle::Tolerances handoff_tolerances_;
Eigen::Vector3d last_pos_(Eigen::Vector3d::Zero());
bool odom_valid_ = false;
bool tracking_ready_ = false;
std::mutex trajectory_state_mutex_;

/* ---- 运动连续性遥测状态（本轮第二验收标准） ----
 * low-speed 判据：代码库中没有可直接复用的 stall/low-speed 阈值，
 * 因此使用具名常量 0.10 m/s；该阈值只用于遥测，不影响任何控制逻辑。 */
constexpr double kMotionContinuityLowSpeedThreshold = 0.10;  // [m/s]
int trajectory_drone_id_ = -1;
double measured_speed_ = -1.0;
ros::Time measured_speed_stamp_;
double motion_moving_duration_ = 0.0;
ros::Time motion_last_sample_time_;
std::vector<double> motion_low_speed_episodes_;
double motion_current_low_speed_start_ = -1.0;
double motion_last_fast_speed_ = 0.0;
double motion_speed_before_event_ = 0.0;
double motion_min_speed_in_episode_ = 1.0e9;
bool motion_first_event_reported_ = false;
ros::WallTime motion_last_report_wall_;
bool motion_final_emitted_ = false;
std::vector<double> activation_gap_samples_;
double last_activation_time_ = -1.0;
double max_successor_gap_ = 0.0;
void recordMotionContinuity(const double speed);
void reportMotionContinuity(bool final_report);
void reportFirstStopAndGoIfNeeded();

ros::WallTime last_cmd_callback_wall_;
ros::WallTime last_cmd_publish_wall_;
ros::WallTime last_execution_summary_wall_;
double max_cmd_callback_gap_ = 0.0;
double max_cmd_publish_gap_ = 0.0;
uint64_t cmd_callback_count_ = 0;
uint64_t cmd_publish_count_ = 0;
constexpr double EXECUTION_GAP_WARN_SEC = 0.20;

// yaw control
double last_yaw_, last_yawdot_, slowly_flip_yaw_target_, slowly_turn_to_center_target_;
double time_forward_;
bool enable_target_facing_yaw_ = true;
bool target_odom_valid_ = false;
Eigen::Vector3d target_position_(Eigen::Vector3d::Zero());

bool executionHandoffGate(const poly_traj::Trajectory &next,
                          const traj_utils::PolyTraj &source,
                          const char *stage);
void polyTrajCallback(traj_utils::PolyTrajPtr msg);

bool mincoToPreparedSource(const traj_utils::MINCOTraj &message,
                           const traj_utils::TeamTrajectorySolution &solution,
                           const size_t member_index,
                           ScheduledTrajectoryEntry &entry)
{
  if (message.order != 5 || message.duration.empty() ||
      message.duration.size() != message.inner_x.size() + 1 ||
      message.inner_x.size() != message.inner_y.size() ||
      message.inner_x.size() != message.inner_z.size() ||
      member_index >= solution.frontier_owner_revisions.size() ||
      member_index >= solution.frontier_owner_trajectory_ids.size())
    return false;
  const int pieces = static_cast<int>(message.duration.size());
  Eigen::Matrix3d head, tail;
  head << message.start_p[0], message.start_v[0], message.start_a[0],
      message.start_p[1], message.start_v[1], message.start_a[1],
      message.start_p[2], message.start_v[2], message.start_a[2];
  tail << message.end_p[0], message.end_v[0], message.end_a[0],
      message.end_p[1], message.end_v[1], message.end_a[1],
      message.end_p[2], message.end_v[2], message.end_a[2];
  Eigen::MatrixXd inner(3, pieces - 1);
  Eigen::VectorXd durations(pieces);
  for (int piece = 0; piece < pieces - 1; ++piece)
    inner.col(piece) << message.inner_x[piece], message.inner_y[piece],
        message.inner_z[piece];
  for (int piece = 0; piece < pieces; ++piece)
  {
    durations(piece) = message.duration[piece];
    if (!std::isfinite(durations(piece)) || durations(piece) <= 1.0e-6)
      return false;
  }
  if (!head.allFinite() || !tail.allFinite() || !inner.allFinite())
    return false;
  poly_traj::MinJerkOpt minco;
  minco.reset(head, tail, pieces);
  minco.generate(inner, durations);
  const poly_traj::Trajectory trajectory = minco.getTraj();
  if (trajectory.getPieceNum() != pieces ||
      !std::isfinite(trajectory.getTotalDuration()))
    return false;

  entry = ScheduledTrajectoryEntry();
  entry.traj.reset(new poly_traj::Trajectory(trajectory));
  entry.start_time = solution.activation_time;
  entry.duration = trajectory.getTotalDuration();
  // Feedback093: a Team speculative trajectory gets its own id namespace.
  // Deriving it as `frontier_owner_trajectory_ids[i] + 1` collided with the id
  // the Local Planner independently allocates for its own next rolling
  // successor (Feedback092: two different trajectories were both id 77 with
  // different activation times).  The id is deterministic on both sides, being
  // a pure function of the shared team_solution_id.
  entry.traj_id = kTeamSpeculativeTrajectoryIdBase +
      static_cast<int>(solution.team_solution_id);
  entry.source.drone_id = trajectory_drone_id_;
  entry.source.traj_id = entry.traj_id;
  entry.source.start_time = solution.activation_time;
  entry.source.order = 5;
  entry.source.trajectory_source = "TEAM_REALIZED_LOCAL";
  entry.source.generation = solution.frontier_owner_revisions[member_index] + 1;
  entry.source.team_solution_id = solution.team_solution_id;
  entry.source.team_prepare_only = true;
  entry.source.encirclement_generation = solution.encirclement_generation;
  entry.source.viewpoint_hypothesis_id = solution.hypothesis_id;
  entry.source.safety_validated = true;
  entry.source.candidate_ready_time = ros::Time::now();
  entry.source.commit_time = ros::Time(0.0);
  entry.source.validation_time = ros::Time::now();
  entry.source.validation_expiry = ros::Time(
      solution.activation_time.toSec() + trajectory.getTotalDuration());
  entry.source.lifecycle_state = "TEAM_EXECUTION_PREPARED";
  entry.source.team_reference_id = solution.team_solution_id;
  entry.source.team_reference_reason = "REALIZED_TEAM_VALIDATED";
  // The coordinator reports the Local Planner frontier owner this realization
  // was built to continue.  The executor validates it against its own
  // confirmed ACTIVE identity before admitting the speculation.
  entry.source.expected_predecessor_id =
      solution.frontier_owner_trajectory_ids[member_index];
  entry.source.expected_predecessor_generation =
      static_cast<uint64_t>(solution.frontier_owner_revisions[member_index]);
  entry.source.lineage_class = kLineageClassTeam;
  entry.source.validation_revision = entry.source.generation;
  entry.source.checked_until = solution.activation_time.toSec() +
      std::min(trajectory.getTotalDuration(), solution.evaluation_horizon);
  if (!std::isfinite(entry.source.checked_until) ||
      entry.source.checked_until <= solution.activation_time.toSec())
    return false;
  entry.source.validation_expiry = ros::Time(entry.source.checked_until);
  entry.expected_predecessor_id = entry.source.expected_predecessor_id;
  entry.expected_predecessor_generation = entry.source.expected_predecessor_generation;
  entry.team_speculative = true;
  for (int piece = 0; piece < pieces; ++piece)
  {
    entry.source.duration.push_back(trajectory[piece].getDuration());
    const auto &coeff = trajectory[piece].getCoeffMat();
    for (int coefficient = 0; coefficient < 6; ++coefficient)
    {
      entry.source.coef_x.push_back(coeff(0, coefficient));
      entry.source.coef_y.push_back(coeff(1, coefficient));
      entry.source.coef_z.push_back(coeff(2, coefficient));
    }
  }
  entry.source.payload_hash = traj_utils::polyPayloadHash(entry.source);
  return true;
}

void teamSolutionCallback(
    const traj_utils::TeamTrajectorySolutionConstPtr &message)
{
  if (!message)
    return;
  if (message->state == traj_utils::TeamTrajectorySolution::STATE_PREPARE)
  {
    int member_index = -1;
    for (size_t index = 0; index < message->drone_ids.size(); ++index)
      if (message->drone_ids[index] == trajectory_drone_id_)
        member_index = static_cast<int>(index);
    ScheduledTrajectoryEntry prepared;
    bool accepted = message->valid && member_index >= 0 &&
        static_cast<size_t>(member_index) < message->trajectories.size() &&
        mincoToPreparedSource(message->trajectories[member_index], *message,
                              member_index, prepared);
    std::string reason = accepted ? "EXECUTOR_PREPARED" :
                                    "EXECUTOR_PREPARE_PAYLOAD_INVALID";
    if (accepted)
    {
      std::lock_guard<std::mutex> lock(trajectory_state_mutex_);
      accepted = ros::Time::now() < prepared.start_time &&
          executionHandoffGate(*prepared.traj, prepared.source, "PREPARE");
      if (accepted)
      {
        prepared_team_trajectory_ = prepared;
        prepared_team_trajectory_valid_ = true;
      }
      else
        reason = "EXECUTOR_PREPARE_HANDOFF_REJECTED";
    }
    traj_utils::TeamTrajectoryAck ack;
    ack.phase = traj_utils::TeamTrajectoryAck::PHASE_EXECUTOR_READY;
    ack.team_solution_id = message->team_solution_id;
    ack.team_reference_id = message->team_solution_id;
    ack.coordination_generation = message->coordination_generation;
    ack.drone_id = trajectory_drone_id_;
    if (member_index >= 0 &&
        static_cast<size_t>(member_index) < message->planning_generations.size())
      ack.planning_generation = message->planning_generations[member_index];
    if (member_index >= 0 &&
        static_cast<size_t>(member_index) < message->source_candidate_ids.size())
      ack.source_candidate_id = message->source_candidate_ids[member_index];
    ack.stamp = ros::Time::now();
    ack.accepted = accepted;
    ack.reason = reason;
    ack.target_prediction_revision = message->target_snapshot_identity;
    ack.target_snapshot_epoch = message->target_snapshot_epoch;
    ack.dynamic_prediction_identity = message->dynamic_prediction_identity;
    ack.static_map_revision = message->static_map_revision;
    ack.visibility_model_version = message->visibility_model_version;
    ack.activation_time = message->activation_time;
    ack.execution_horizon = message->evaluation_horizon;
    team_trajectory_ack_pub.publish(ack);
    ROS_INFO("[TEAM_EXECUTOR_READY] drone=%d team_solution_id=%lu accepted=%d "
             "activation=%.9f reason=%s",
             trajectory_drone_id_,
             static_cast<unsigned long>(message->team_solution_id),
             static_cast<int>(accepted), message->activation_time.toSec(),
             reason.c_str());
    return;
  }

  if (message->state == traj_utils::TeamTrajectorySolution::STATE_COMMIT)
  {
    traj_utils::PolyTrajPtr committed;
    {
      std::lock_guard<std::mutex> lock(trajectory_state_mutex_);
      const bool already_committed =
          (scheduled_traj_valid_ &&
           scheduled_source_.team_solution_id == message->team_solution_id &&
           scheduled_start_time_ == message->activation_time) ||
          std::any_of(
              scheduled_traj_queue_.begin(), scheduled_traj_queue_.end(),
              [&](const ScheduledTrajectoryEntry &entry) {
                return entry.source.team_solution_id ==
                           message->team_solution_id &&
                       entry.start_time == message->activation_time;
              }) ||
          (receive_traj_ &&
           active_source_.team_solution_id == message->team_solution_id &&
           start_time_ == message->activation_time);
      // Coordinator 会在 common activation 前重发同一个 COMMIT 以抵抗消息丢失。
      // 第一次 COMMIT 已把 PREPARED 晋升到 future queue 后，后续相同消息必须是
      // 幂等 no-op；它既不是新的 enqueue，也不是缺少 prepared payload 的错误。
      if (!prepared_team_trajectory_valid_ && already_committed)
      {
        ROS_INFO_THROTTLE(
            0.5, "[TEAM_EXECUTION_COMMIT_DUPLICATE] drone=%d "
                 "team_solution_id=%lu activation=%.9f action=IDEMPOTENT_NOOP",
            trajectory_drone_id_,
            static_cast<unsigned long>(message->team_solution_id),
            message->activation_time.toSec());
        return;
      }
      if (!prepared_team_trajectory_valid_ ||
          prepared_team_trajectory_.source.team_solution_id !=
              message->team_solution_id ||
          prepared_team_trajectory_.start_time != message->activation_time)
      {
        ROS_ERROR("[TEAM_EXECUTION_COMMIT_REJECT] drone=%d team_solution_id=%lu "
                  "reason=NO_MATCHING_PREPARED_PAYLOAD",
                  trajectory_drone_id_,
                  static_cast<unsigned long>(message->team_solution_id));
        return;
      }
      committed.reset(new traj_utils::PolyTraj(prepared_team_trajectory_.source));
      committed->team_prepare_only = false;
      committed->commit_time = ros::Time::now();
      committed->lifecycle_state = "TEAM_EXECUTION_COMMITTED";
      prepared_team_trajectory_valid_ = false;
      prepared_team_trajectory_ = ScheduledTrajectoryEntry();
    }
    polyTrajCallback(committed);
    ROS_INFO("[TEAM_EXECUTION_COMMIT] drone=%d team_solution_id=%lu "
             "activation=%.9f",
             trajectory_drone_id_,
             static_cast<unsigned long>(message->team_solution_id),
             message->activation_time.toSec());
    return;
  }

  if (message->state != traj_utils::TeamTrajectorySolution::STATE_ABORT)
    return;
  std::lock_guard<std::mutex> lock(trajectory_state_mutex_);
  const ros::Time now = ros::Time::now();
  if (prepared_team_trajectory_valid_ &&
      prepared_team_trajectory_.source.team_solution_id ==
          message->team_solution_id)
  {
    ROS_WARN("[team-speculative-abort] transaction_id=%lu trajectory_id=%d "
             "state=ABORTED reason=%s "
             "local_authoritative_predecessor_unchanged=1 prepared_only=1",
             static_cast<unsigned long>(message->team_solution_id),
             prepared_team_trajectory_.traj_id, message->reason.c_str());
    prepared_team_trajectory_valid_ = false;
    prepared_team_trajectory_ = ScheduledTrajectoryEntry();
  }
  // Feedback093: a Team abort may only retract TEAM SPECULATIVE entries.  It
  // must never remove a Local guaranteed successor and must never touch the
  // Planner's authoritative predecessor (which only ACTIVATION can move).
  bool removed_speculation = false;
  if (scheduled_traj_valid_ && scheduled_traj_ &&
      scheduled_team_speculative_ &&
      static_cast<uint64_t>(scheduled_source_.team_solution_id) ==
          static_cast<uint64_t>(message->team_solution_id) &&
      now < scheduled_start_time_)
  {
    ROS_WARN("[team-speculative-abort] transaction_id=%lu trajectory_id=%d "
             "state=ABORTED reason=%s "
             "local_authoritative_predecessor_unchanged=1 "
             "active_trajectory_id=%d",
             static_cast<unsigned long>(message->team_solution_id),
             scheduled_traj_id_, message->reason.c_str(), traj_id_);
    markLineageInvalidated(scheduled_traj_id_);
    clearScheduledSlot();
    removed_speculation = true;
  }
  const size_t before = scheduled_traj_queue_.size();
  scheduled_traj_queue_.erase(
      std::remove_if(scheduled_traj_queue_.begin(), scheduled_traj_queue_.end(),
                     [&](const ScheduledTrajectoryEntry &entry) {
                       if (!entry.team_speculative)
                         return false;   // Local guaranteed future is protected
                       if (static_cast<uint64_t>(entry.source.team_solution_id) !=
                           static_cast<uint64_t>(message->team_solution_id))
                         return false;
                       ROS_WARN("[team-speculative-abort] transaction_id=%lu "
                                "trajectory_id=%d state=ABORTED reason=%s "
                                "local_authoritative_predecessor_unchanged=1 "
                                "active_trajectory_id=%d",
                           static_cast<unsigned long>(message->team_solution_id),
                           entry.traj_id, message->reason.c_str(), traj_id_);
                       markLineageInvalidated(entry.traj_id);
                       return true;
                     }),
      scheduled_traj_queue_.end());
  if (scheduled_traj_queue_.size() != before)
    removed_speculation = true;
  if (removed_speculation)
  {
    ++team_speculative_abort_count_;
    normalizeFutureTimeline();
  }
  const bool already_active = receive_traj_ &&
      active_source_.team_solution_id == message->team_solution_id &&
      now >= start_time_;
  ROS_WARN("[traj-server-team-cancel] node=%s team_solution_id=%lu "
           "reason=%s action=%s",
           ros::this_node::getName().c_str(),
           static_cast<unsigned long>(message->team_solution_id),
           message->reason.c_str(),
           already_active ? "ACTIVE_INVALIDATED" : "CANCEL_ALL_FUTURE");
}

void targetOdomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  const auto &p = msg->pose.pose.position;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
    return;
  std::lock_guard<std::mutex> lock(trajectory_state_mutex_);
  target_position_ = Eigen::Vector3d(p.x, p.y, p.z);
  target_odom_valid_ = true;
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  const auto &p = msg->pose.pose.position;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
    return;
  const auto &v = msg->twist.twist.linear;
  const bool velocity_finite =
      std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
  const double speed =
      velocity_finite ? std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z) : -1.0;
  {
    std::lock_guard<std::mutex> lock(trajectory_state_mutex_);
    odom_valid_ = true;
    measured_speed_ = speed;
    measured_speed_stamp_ =
        msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  }
  recordMotionContinuity(speed);
  reportMotionContinuity(false);
}

void heartbeatCallback(std_msgs::EmptyPtr msg)
{
  const ros::Time now = ros::Time::now();
  bool recovered = false;
  int traj_id = -1;
  double remaining_duration = 0.0;
  {
    std::lock_guard<std::mutex> lock(trajectory_state_mutex_);
    heartbeat_time_ = now;
    recovered = heartbeat_stale_;
    if (receive_traj_ && traj_)
      remaining_duration = std::max(0.0, traj_duration_ - (now - start_time_).toSec());
    traj_id = traj_id_;
    heartbeat_stale_ = false;
  }
  if (recovered)
  {
    ROS_INFO("[traj-server-heartbeat-recovered] node=%s traj_id=%d "
             "remaining_duration=%.6f",
             ros::this_node::getName().c_str(), traj_id, remaining_duration);
  }
}

// Reason of the most recent rejected admission; the caller reports it verbatim
// so an identity error can never be mislabelled as a geometry error.
std::string last_handoff_reject_reason_ = "NONE";

ScheduledTrajectoryEntry makeEntry(const traj_utils::PolyTraj &msg,
                                   const boost::shared_ptr<poly_traj::Trajectory> &traj)
{
  ScheduledTrajectoryEntry entry;
  entry.source = msg;
  entry.traj = traj;
  entry.start_time = msg.start_time;
  entry.duration = traj ? traj->getTotalDuration() : 0.0;
  entry.traj_id = msg.traj_id;
  entry.optimized_yaw_valid = msg.optimized_yaw_valid;
  entry.yaw_sample_times = msg.yaw_sample_times;
  entry.yaw_samples = msg.yaw_samples;
  entry.expected_predecessor_id = msg.expected_predecessor_id;
  entry.expected_predecessor_generation = msg.expected_predecessor_generation;
  entry.team_speculative = msg.lineage_class == kLineageClassTeam;
  return entry;
}

void clearScheduledSlot()
{
  scheduled_traj_valid_ = false;
  scheduled_traj_.reset();
  scheduled_traj_duration_ = 0.0;
  scheduled_traj_id_ = -1;
  scheduled_optimized_yaw_valid_ = false;
  scheduled_yaw_sample_times_.clear();
  scheduled_yaw_samples_.clear();
  scheduled_expected_predecessor_id_ = -1;
  scheduled_expected_predecessor_generation_ = 0;
  scheduled_team_speculative_ = false;
}

void setScheduledSlot(const ScheduledTrajectoryEntry &entry)
{
  scheduled_source_ = entry.source;
  scheduled_traj_ = entry.traj;
  scheduled_start_time_ = entry.start_time;
  scheduled_traj_duration_ = entry.duration;
  scheduled_traj_id_ = entry.traj_id;
  scheduled_optimized_yaw_valid_ = entry.optimized_yaw_valid;
  scheduled_yaw_sample_times_ = entry.yaw_sample_times;
  scheduled_yaw_samples_ = entry.yaw_samples;
  scheduled_expected_predecessor_id_ = entry.expected_predecessor_id;
  scheduled_expected_predecessor_generation_ = entry.expected_predecessor_generation;
  scheduled_team_speculative_ = entry.team_speculative;
  scheduled_traj_valid_ = true;
}

// The pending-future timeline is ONE ordered sequence keyed by activation time.
// Feedback092: a Local guaranteed successor queued behind a Team speculative
// entry whose activation was 0.48 s later could never activate at all, because
// the activation timer only ever consulted the scheduled slot.  Promoting the
// earliest pending entry unconditionally removes that starvation.  Ties prefer
// Local: a guaranteed successor outranks a speculative replacement.
bool futureEntryEarlier(const ScheduledTrajectoryEntry &a,
                        const ScheduledTrajectoryEntry &b)
{
  const double lhs = a.start_time.toSec(), rhs = b.start_time.toSec();
  if (std::abs(lhs - rhs) > 1.0e-9)
    return lhs < rhs;
  if (a.team_speculative != b.team_speculative)
    return !a.team_speculative;
  return a.source.generation < b.source.generation;
}

void normalizeFutureTimeline()
{
  if (scheduled_traj_valid_ && scheduled_traj_)
  {
    ScheduledTrajectoryEntry current;
    current.source = scheduled_source_;
    current.traj = scheduled_traj_;
    current.start_time = scheduled_start_time_;
    current.duration = scheduled_traj_duration_;
    current.traj_id = scheduled_traj_id_;
    current.optimized_yaw_valid = scheduled_optimized_yaw_valid_;
    current.yaw_sample_times = scheduled_yaw_sample_times_;
    current.yaw_samples = scheduled_yaw_samples_;
    current.expected_predecessor_id = scheduled_expected_predecessor_id_;
    current.expected_predecessor_generation =
        scheduled_expected_predecessor_generation_;
    current.team_speculative = scheduled_team_speculative_;
    scheduled_traj_queue_.push_back(current);
    clearScheduledSlot();
  }
  std::stable_sort(scheduled_traj_queue_.begin(), scheduled_traj_queue_.end(),
                   futureEntryEarlier);
  if (!scheduled_traj_queue_.empty())
  {
    ScheduledTrajectoryEntry head = scheduled_traj_queue_.front();
    scheduled_traj_queue_.pop_front();
    setScheduledSlot(head);
  }
}

// Admit a future reservation.  Local and Team reservations share the activation
// timeline but are tagged, so a Team transaction can only ever retract its own
// speculative entries.
void admitFutureEntry(const ScheduledTrajectoryEntry &entry)
{
  const double act = entry.start_time.toSec();
  if (scheduled_traj_valid_ && scheduled_traj_ &&
      std::abs(act - scheduled_start_time_.toSec()) <= 1.0e-6 &&
      entry.source.generation > scheduled_source_.generation)
  {
    const int replaced_id = scheduled_traj_id_;
    setScheduledSlot(entry);
    ROS_INFO("[traj-server-scheduled-supersede] node=%s trajectory_id=%d "
             "activation_time=%.9f replaced_trajectory_id=%d "
             "reason=SAME_ACTIVATION_NEW_REVISION",
             ros::this_node::getName().c_str(), entry.traj_id, act, replaced_id);
    return;
  }
  for (auto &queued : scheduled_traj_queue_)
  {
    if (std::abs(act - queued.start_time.toSec()) <= 1.0e-6 &&
        entry.source.generation > queued.source.generation)
    {
      const int replaced_id = queued.traj_id;
      queued = entry;
      ROS_INFO("[traj-server-scheduled-supersede] node=%s trajectory_id=%d "
               "activation_time=%.9f replaced_trajectory_id=%d "
               "reason=SAME_ACTIVATION_NEW_REVISION",
               ros::this_node::getName().c_str(), entry.traj_id, act,
               replaced_id);
      normalizeFutureTimeline();
      return;
    }
  }
  if (!scheduled_traj_valid_ && scheduled_traj_queue_.empty())
  {
    setScheduledSlot(entry);
    ROS_INFO("[traj-server-scheduled] node=%s trajectory_id=%d "
             "receive_time=%.9f activation_time=%.9f lead=%.6f duration=%.6f "
             "current_trajectory_id=%d lineage_class=%s",
             ros::this_node::getName().c_str(), entry.traj_id,
             ros::Time::now().toSec(), act, act - ros::Time::now().toSec(),
             entry.duration, traj_id_,
             entry.team_speculative ? kLineageClassTeam : kLineageClassLocal);
    return;
  }
  scheduled_traj_queue_.push_back(entry);
  normalizeFutureTimeline();
  ROS_INFO("[traj-server-scheduled-queue] node=%s trajectory_id=%d "
           "receive_time=%.9f activation_time=%.9f lead=%.6f duration=%.6f "
           "predecessor_trajectory_id=%d queue_depth=%zu lineage_class=%s",
           ros::this_node::getName().c_str(), entry.traj_id,
           ros::Time::now().toSec(), act, act - ros::Time::now().toSec(),
           entry.duration,
           scheduled_traj_valid_ ? scheduled_traj_id_ : -1,
           scheduled_traj_queue_.size(),
           entry.team_speculative ? kLineageClassTeam : kLineageClassLocal);
}

bool executionHandoffGate(const poly_traj::Trajectory &next,
                          const traj_utils::PolyTraj &source, const char *stage)
{
  last_handoff_reject_reason_ = "NONE";
  if (!receive_traj_ || !traj_) return true;
  const double activation = source.start_time.toSec();

  // ---- explicit staleness / duplicate guards (never implicit) -------------
  // A payload identical to the confirmed ACTIVE identity is a replay, not a
  // successor.  A payload that has already fully expired cannot become history.
  if (source.traj_id == traj_id_ && source.start_time == start_time_ &&
      !(source.lifecycle_state == "CURRENT_STATE_RESTART_VALIDATED"))
  {
    last_handoff_reject_reason_ = "DUPLICATE_ACTIVE_IDENTITY";
    return false;
  }
  if (!next.getTotalDuration() || activation + next.getTotalDuration() <=
          ros::Time::now().toSec() + 1.0e-6)
  {
    last_handoff_reject_reason_ = "EXPIRED_PAYLOAD";
    return false;
  }

  // Odometry re-anchor: the planner explicitly declares that the predecessor
  // authority is the *measured* state, not any polynomial.  This is the only
  // path allowed to bypass predecessor resolution, and it is never entered by
  // an ordinary Team commit or an ordinary Team abort.
  const bool current_state_restart = trajectory_lifecycle::currentStateRestartHandoff(
      source.lifecycle_state == "CURRENT_STATE_RESTART_VALIDATED",
      source.safety_validated, active_source_.generation, source.generation,
      activation);

  // ---- Feedback093: exact predecessor identity, no implicit inference -----
  // The payload names the trajectory it was built to continue.  We resolve that
  // exact identity.  We never substitute the queued / scheduled / active
  // trajectory, because doing so silently converts an identity error into a
  // geometry error (Feedback092: planner OLD_ID=106 DP=0 while the executor
  // compared against OLD_ID=76 and reported ACTIVE_PVA_MISMATCH).
  LineageRecord *predecessor_record = nullptr;
  LineageResolution resolution = LineageResolution::FOUND;
  const int expected_id = source.expected_predecessor_id;
  const bool bootstrap_predecessor = expected_id <= 0;
  if (!bootstrap_predecessor)
  {
    ++explicit_predecessor_resolve_count_;
    resolution = resolveLineage(expected_id, &predecessor_record);
  }
  else
  {
    // Bootstrap: the only case where "no declared predecessor" is legal is a
    // payload explicitly marked as such by the producer.
    resolution = LineageResolution::FOUND;
  }

  const bool identity_ok =
      bootstrap_predecessor || resolution == LineageResolution::FOUND;
  if (!identity_ok)
  {
    ++lineage_identity_error_count_;
    last_handoff_reject_reason_ =
        resolution == LineageResolution::INVALIDATED
            ? "EXPECTED_PREDECESSOR_INVALIDATED"
            : "EXPECTED_PREDECESSOR_NOT_AVAILABLE";
    std::ostringstream err;
    err.precision(17);
    err << "[execution-handoff] stage=" << stage << " drone=" << source.drone_id
        << " OLD_SOURCE=UNRESOLVED NEW_SOURCE=" << source.trajectory_source
        << " OLD_ID=" << expected_id << " NEW_ID=" << source.traj_id
        << " expected_predecessor_id=" << expected_id
        << " expected_predecessor_generation="
        << static_cast<unsigned long>(source.expected_predecessor_generation)
        << " lineage_class=" << source.lineage_class
        << " team_solution_id=" << source.team_solution_id
        << " HANDOFF_DP=-1 HANDOFF_DV=-1 HANDOFF_DA=-1"
        << " ACTIVATION_TIME=" << activation
        << " actual_time=" << ros::Time::now().toSec()
        << " authority=EXPECTED_PREDECESSOR"
        << " resolution=" << lineageResolutionName(resolution)
        << " accepted=0";
    ROS_INFO_STREAM(err.str());
    ROS_ERROR("[traj-server-handoff-reject] node=%s trajectory_id=%d "
              "reason=%s expected_predecessor_id=%d",
              ros::this_node::getName().c_str(), source.traj_id,
              resolution == LineageResolution::INVALIDATED
                  ? "EXPECTED_PREDECESSOR_INVALIDATED"
                  : "EXPECTED_PREDECESSOR_NOT_AVAILABLE",
              expected_id);
    return false;
  }

  const poly_traj::Trajectory *predecessor =
      bootstrap_predecessor ? traj_.get() : predecessor_record->traj.get();
  const double predecessor_start =
      bootstrap_predecessor ? start_time_.toSec() : predecessor_record->start_time;
  const double predecessor_duration =
      bootstrap_predecessor ? traj_duration_ : predecessor_record->duration;
  const std::string predecessor_source =
      bootstrap_predecessor ? active_source_.trajectory_source
                            : predecessor_record->source;
  const int predecessor_id =
      bootstrap_predecessor ? traj_id_ : predecessor_record->traj_id;
  const double old_end = predecessor_start + predecessor_duration;
  const double predecessor_checked_until = bootstrap_predecessor
      ? active_source_.checked_until : predecessor_record->checked_until;

  const auto old_state = trajectory_lifecycle::sample(*predecessor, activation - predecessor_start);
  const auto new_state = trajectory_lifecycle::sample(next, 0.0);
  const auto residual = trajectory_lifecycle::compare(old_state, new_state);
  bool valid = current_state_restart ||
      (activation < predecessor_checked_until - 1.0e-9 &&
       residual.accepted(handoff_tolerances_));

  // ---- activation-stage physical continuity --------------------------------
  // Admission is identity-exact: a payload must name a resolvable predecessor.
  // That is a statement about *provenance* and says nothing about whether the
  // declared predecessor is still what the vehicle is executing at the switch
  // instant.  A speculative Team future can therefore stay provenance-valid
  // while the Local guaranteed successor has already taken over in between.
  // Handing over to it would be a real discontinuity, so at ACTIVATION we also
  // require continuity with the trajectory that is actually being superseded.
  // This is an explicitly named physical check, never a silent predecessor
  // substitution: the declared identity above is untouched and still reported.
  double live_dp = -1.0;
  bool live_continuous = true;
  if (valid && !current_state_restart && std::string(stage) == "ACTIVATION" &&
      !bootstrap_predecessor && predecessor_id != traj_id_)
  {
    const auto live_old =
        trajectory_lifecycle::sample(*traj_, activation - start_time_.toSec());
    const auto live_residual = trajectory_lifecycle::compare(live_old, new_state);
    live_dp = live_residual.dp;
    live_continuous = live_residual.accepted(handoff_tolerances_);
    if (!live_continuous)
    {
      valid = false;
      last_handoff_reject_reason_ = "ACTIVATION_HANDOFF_DISCONTINUITY";
      ROS_ERROR("[execution-handoff-discontinuity] stage=ACTIVATION "
                "drone=%d trajectory_id=%d lineage_class=%s "
                "declared_predecessor_id=%d declared_dp=%.9f "
                "live_active_id=%d live_dp=%.9f live_dv=%.9f live_da=%.9f "
                "activation=%.9f action=REJECT_STALE_SPECULATION",
                source.drone_id, source.traj_id, source.lineage_class.c_str(),
                expected_id, residual.dp, traj_id_, live_residual.dp,
                live_residual.dv, live_residual.da, activation);
    }
  }
  std::ostringstream data;
  data.precision(17);
  data << "[execution-handoff] stage=" << stage << " drone=" << source.drone_id
       << " OLD_SOURCE=" << predecessor_source << " NEW_SOURCE=" << source.trajectory_source
       << " OLD_ID=" << predecessor_id << " NEW_ID=" << source.traj_id
       << " expected_predecessor_id=" << expected_id
       << " predecessor_match=" << (bootstrap_predecessor ? 1 : (predecessor_id == expected_id ? 1 : 0))
       << " lineage_class=" << source.lineage_class
       << " old_generation=" << (bootstrap_predecessor ? active_source_.generation : predecessor_record->generation)
       << " new_generation=" << source.generation
       << " team_solution_id=" << source.team_solution_id
       << " HANDOFF_DP=" << residual.dp << " HANDOFF_DV=" << residual.dv << " HANDOFF_DA=" << residual.da
       << " ACTIVATION_TIME=" << activation << " actual_time=" << ros::Time::now().toSec()
       << " old_remaining=" << old_end-activation
       << " pending_duration=" << source.pending_duration
       << " candidate_ready_time=" << source.candidate_ready_time.toSec()
       << " commit_time=" << source.commit_time.toSec()
       << " authority=" << (current_state_restart ? "CURRENT_STATE" : "EXPECTED_PREDECESSOR")
       << " resolution=" << lineageResolutionName(resolution)
       << " live_active_id=" << traj_id_ << " live_dp=" << live_dp
       << " live_continuous=" << static_cast<int>(live_continuous)
       << " accepted=" << valid;
  for(int i=0;i<3;++i) data << " old_p" << i << "=" << old_state.p[i] << " old_v" << i << "=" << old_state.v[i]
      << " old_a" << i << "=" << old_state.a[i] << " new_p" << i << "=" << new_state.p[i]
      << " new_v" << i << "=" << new_state.v[i] << " new_a" << i << "=" << new_state.a[i];
  ROS_INFO_STREAM(data.str());
  if(current_state_restart)
    ROS_WARN("[current-state-restart] event=EXECUTOR_HANDOFF_ACCEPTED stage=%s node=%s expired_predecessor_id=%d new_trajectory_id=%d old_end=%.9f activation=%.9f old_generation=%lu new_generation=%lu planner_safety_validated=1 predecessor_pva_authority=0",
        stage,ros::this_node::getName().c_str(),predecessor_id,source.traj_id,
        old_end,activation,static_cast<unsigned long>(active_source_.generation),
        static_cast<unsigned long>(source.generation));
  if(valid && std::string(stage)=="ACTIVATION") {
    double previous_min=old_state.v.norm(),next_min=new_state.v.norm();
    for(double dt=0;dt<=0.300001;dt+=0.01) {
      previous_min=std::min(previous_min,trajectory_lifecycle::sample(*predecessor,activation-predecessor_start-dt).v.norm());
      next_min=std::min(next_min,trajectory_lifecycle::sample(next,dt).v.norm());
    }
    ROS_INFO("[continuous-motion-activation] drone=%d source=%s trajectory_id=%d activation_time=%.9f previous_source=%s speed_before_activation=%.9f speed_after_activation=%.9f current_trajectory_end_speed=%.9f remaining_time_before_activation=%.9f minimum_speed_previous_03_reference=%.9f minimum_speed_next_03_reference=%.9f",
        source.drone_id,source.trajectory_source.c_str(),source.traj_id,activation,
        predecessor_source.c_str(),old_state.v.norm(),new_state.v.norm(),
        next.getVel(next.getTotalDuration()).norm(),start_time_.toSec()+traj_duration_-activation,previous_min,next_min);
    if(source.team_solution_id>0 &&
       source.trajectory_source=="TEAM_REALIZED_LOCAL")
      ROS_INFO("[team-transaction] event=TRANSACTION_ACTIVATED "
               "TEAM_TRANSACTION_ID=%lu team_solution_id=%lu drone=%d "
               "TRANSACTION_FINAL_SOURCE=%s trajectory_id=%d "
               "activation=%.9f",
          static_cast<unsigned long>(source.team_solution_id),
          static_cast<unsigned long>(source.team_solution_id),
          source.drone_id,source.trajectory_source.c_str(),source.traj_id,
          activation);
  }
  return valid;
}
void exitTerminalHold(double now)
{
  if(terminal_hold_active_) ROS_WARN("[execution-lifecycle] event=TERMINAL_HOLD_EXIT node=%s trajectory_id=%d timestamp=%.9f duration=%.9f",
      ros::this_node::getName().c_str(),traj_id_,now,now-terminal_hold_started_);
  terminal_hold_active_=false;
}

void polyTrajCallback(traj_utils::PolyTrajPtr msg)
{
  const std::uint64_t observed_hash = traj_utils::polyPayloadHash(*msg);
  if (msg->payload_hash == 0 || msg->payload_hash != observed_hash ||
      msg->validation_revision == 0 ||
      !std::isfinite(msg->checked_until) ||
      msg->checked_until <= msg->start_time.toSec())
  {
    ROS_ERROR("[EXECUTION_IDENTITY] stage=RECEIVE trajectory_id=%d "
              "reason=PAYLOAD_OR_CERTIFICATE_MISMATCH declared_hash=%lu "
              "observed_hash=%lu revision=%lu checked_until=%.9f",
              msg->traj_id, static_cast<unsigned long>(msg->payload_hash),
              static_cast<unsigned long>(observed_hash),
              static_cast<unsigned long>(msg->validation_revision),
              msg->checked_until);
    return;
  }
  ROS_INFO("[EXECUTION_IDENTITY] stage=RECEIVE trajectory_id=%d "
           "ACK_REVISION=%lu payload_hash=%lu checked_until=%.9f",
           msg->traj_id,
           static_cast<unsigned long>(msg->validation_revision),
           static_cast<unsigned long>(observed_hash), msg->checked_until);
  {
    std::lock_guard<std::mutex> lock(trajectory_state_mutex_);
    if (receive_traj_ && msg->generation == active_source_.generation &&
        msg->traj_id == traj_id_ && msg->start_time == start_time_)
    {
      // Lease/source updates cannot replace coefficients or rewind execution.
      if (msg->validation_time >= active_source_.validation_time)
      {
        active_source_.trajectory_source=msg->trajectory_source;
        active_source_.validation_time=msg->validation_time;
        active_source_.validation_expiry=msg->validation_expiry;
        active_source_.lifecycle_state=msg->lifecycle_state;
        active_source_.safety_validated=msg->safety_validated;
        active_source_.checked_until=msg->checked_until;
        active_source_.validation_revision=msg->validation_revision;
      }
      return;
    }
    if(scheduled_traj_valid_ && msg->generation == scheduled_source_.generation &&
       msg->traj_id == scheduled_traj_id_ && msg->start_time == scheduled_start_time_) return;
    if(receive_traj_ && msg->generation <= active_source_.generation) return;
    if(scheduled_traj_valid_ && msg->generation <= scheduled_source_.generation) return;
  }
  if (!msg->safety_validated || msg->generation == 0)
  {
    ROS_ERROR("[traj-server-safety-reject] trajectory_id=%d reason=UNVALIDATED_SOURCE",msg->traj_id);
    return;
  }
  if (msg->order != 5)
  {
    ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  const size_t expected_coefficient_count =
      msg->duration.size() * static_cast<size_t>(msg->order + 1);
  if (msg->duration.empty() ||
      expected_coefficient_count != msg->coef_x.size() ||
      expected_coefficient_count != msg->coef_y.size() ||
      expected_coefficient_count != msg->coef_z.size())
  {
    ROS_ERROR("[traj_server] WRONG trajectory dimensions.");
    return;
  }

  for (const double duration : msg->duration)
  {
    if (!std::isfinite(duration) || duration <= 0.0)
    {
      ROS_ERROR("[traj_server] WRONG trajectory duration.");
      return;
    }
  }
  for (size_t index = 0; index < expected_coefficient_count; ++index)
  {
    if (!std::isfinite(msg->coef_x[index]) ||
        !std::isfinite(msg->coef_y[index]) ||
        !std::isfinite(msg->coef_z[index]))
    {
      ROS_ERROR("[traj_server] WRONG non-finite trajectory coefficient.");
      return;
    }
  }
  if (msg->optimized_yaw_valid &&
      (msg->yaw_sample_times.size() < 2 ||
       msg->yaw_sample_times.size() != msg->yaw_samples.size()))
  {
    ROS_ERROR("[traj-server] INVALID optimized yaw dimensions; fallback target-facing yaw");
    msg->optimized_yaw_valid = false;
  }
  if (msg->optimized_yaw_valid)
  {
    double previous_rate = 0.0;
    double previous_dt = 0.0;
    bool have_previous_rate = false;
    if (std::abs(msg->yaw_sample_times.front()) > 1.0e-6)
    {
      ROS_ERROR("[traj-server] INVALID optimized yaw activation origin; fallback target-facing yaw");
      msg->optimized_yaw_valid = false;
    }
    for (size_t index = 0; index < msg->yaw_sample_times.size(); ++index)
    {
      if (!std::isfinite(msg->yaw_sample_times[index]) ||
          !std::isfinite(msg->yaw_samples[index]) ||
          (index > 0 && msg->yaw_sample_times[index] <=
              msg->yaw_sample_times[index - 1]))
      {
        ROS_ERROR("[traj-server] INVALID optimized yaw values; fallback target-facing yaw");
        msg->optimized_yaw_valid = false;
        break;
      }
      if (index == 0 || !msg->optimized_yaw_valid)
        continue;
      const double dt = msg->yaw_sample_times[index] -
                        msg->yaw_sample_times[index - 1];
      double difference = msg->yaw_samples[index] - msg->yaw_samples[index - 1];
      while (difference > M_PI) difference -= 2.0 * M_PI;
      while (difference < -M_PI) difference += 2.0 * M_PI;
      const double rate = difference / dt;
      if (std::abs(rate) > 2.0 * M_PI + 1.0e-6)
      {
        ROS_ERROR("[traj-server] INVALID optimized yaw rate; fallback target-facing yaw");
        msg->optimized_yaw_valid = false;
        break;
      }
      if (have_previous_rate)
      {
        const double dt_acc = 0.5 * (previous_dt + dt);
        if (std::abs((rate - previous_rate) / dt_acc) >
            5.0 * M_PI + 1.0e-6)
        {
          ROS_ERROR("[traj-server] INVALID optimized yaw acceleration; fallback target-facing yaw");
          msg->optimized_yaw_valid = false;
          break;
        }
      }
      previous_rate = rate;
      previous_dt = dt;
      have_previous_rate = true;
    }
  }

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
        msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
        msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    dura[i] = msg->duration[i];
  }

  boost::shared_ptr<poly_traj::Trajectory> new_traj(
      new poly_traj::Trajectory(dura, cMats));
  const ros::Time start_time = msg->start_time;
  const double traj_duration = new_traj->getTotalDuration();
  const int traj_id = msg->traj_id;

  {
    std::lock_guard<std::mutex> lock(trajectory_state_mutex_);
    const ros::Time now = ros::Time::now();
    if (!executionHandoffGate(*new_traj,*msg,"RECEIVE"))
    {
      ROS_ERROR("[traj-server-handoff-reject] node=%s trajectory_id=%d "
                "reason=%s expected_predecessor_id=%d lineage_class=%s",
                ros::this_node::getName().c_str(),traj_id,
                last_handoff_reject_reason_.c_str(),msg->expected_predecessor_id,
                msg->lineage_class.c_str());
      return;
    }
    if ((start_time - now).toSec() > 0.0 && receive_traj_ && traj_)
    {
      const ScheduledTrajectoryEntry entry = makeEntry(*msg, new_traj);
      noteLineageRecord(*msg, new_traj, true);
      admitFutureEntry(entry);
      publishLineage("COMMITTED_FUTURE", traj_id, msg->expected_predecessor_id,
                     msg->lineage_class.c_str(), msg->team_solution_id);
      return;
    }
    scheduled_traj_valid_ = false;
    scheduled_traj_.reset();
    scheduled_optimized_yaw_valid_ = false;
    scheduled_yaw_sample_times_.clear();
    scheduled_yaw_samples_.clear();
    if(receive_traj_ && !executionHandoffGate(*new_traj,*msg,"ACTIVATION"))
    {
      ROS_ERROR("[traj-server-handoff-reject] node=%s trajectory_id=%d "
                "reason=%s expected_predecessor_id=%d",
                ros::this_node::getName().c_str(),traj_id,
                last_handoff_reject_reason_.c_str(),msg->expected_predecessor_id);
      return;
    }
    exitTerminalHold(now.toSec());
    const int previous_active_id = traj_id_;
    active_source_ = *msg;
    traj_ = new_traj;
    start_time_ = start_time;
    traj_duration_ = traj_duration;
    traj_id_ = traj_id;
    receive_traj_ = true;
    if (previous_active_id >= 0 && previous_active_id != traj_id_)
      retireLineageRecord(previous_active_id);
    noteLineageRecord(*msg, new_traj, true);
    ROS_INFO("[execution-activated] trajectory_id=%d previous_trajectory_id=%d "
             "source=%s transaction_id=%lu expected_predecessor_id=%d "
             "lineage_class=%s activation_time=%.9f",
             traj_id_, previous_active_id, msg->trajectory_source.c_str(),
             static_cast<unsigned long>(msg->team_solution_id),
             msg->expected_predecessor_id, msg->lineage_class.c_str(),
             start_time.toSec());
    terminal_hold_active_ = false;
    optimized_yaw_valid_ = msg->optimized_yaw_valid;
    optimized_yaw_execution_reported_ = false;
    optimized_yaw_sample_times_ = msg->yaw_sample_times;
    optimized_yaw_samples_ = msg->yaw_samples;
  }

  ROS_INFO("[traj-server-receive] node=%s trajectory_id=%d receive_time=%.9f start_time=%.9f duration=%.6f piece_num=%d",
           ros::this_node::getName().c_str(), traj_id, ros::Time::now().toSec(),
           start_time.toSec(), traj_duration, piece_nums);
  ROS_INFO("[active-traj-lifecycle] event=ACTIVATED traj_identity=start=%.9f,trajectory_id=%d source=traj_server_receive start_time=%.9f duration=%.6f active=1 reason=trajectory_received",
           start_time.toSec(), traj_id, start_time.toSec(), traj_duration);
}

// ---------------------------------------------------------------------------
// 运动连续性遥测（本轮第二验收标准：用户可见的"走走停停"）。
// low-speed 判据：代码库中没有可直接复用的 stall/low-speed 阈值，
// 因此使用具名常量 0.10 m/s；阈值只在遥测里使用，不影响任何控制逻辑。
// 判据：实测速度从 > 阈值跌到 <= 阈值开始记一次 episode，回到 > 阈值结束。
// ---------------------------------------------------------------------------
void recordMotionContinuity(const double speed)
{
  if (!std::isfinite(speed) || speed < 0.0)
    return;
  const ros::Time now = ros::Time::now();
  double dt = 0.0;
  if (!motion_last_sample_time_.isZero())
  {
    dt = (now - motion_last_sample_time_).toSec();
    if (!std::isfinite(dt) || dt < 0.0 || dt > 0.5)
      dt = 0.0;
  }
  motion_last_sample_time_ = now;
  if (speed > kMotionContinuityLowSpeedThreshold)
    motion_moving_duration_ += dt;
  if (speed <= kMotionContinuityLowSpeedThreshold)
  {
    if (motion_current_low_speed_start_ < 0.0)
    {
      motion_current_low_speed_start_ = now.toSec();
      motion_speed_before_event_ = motion_last_fast_speed_;
      motion_min_speed_in_episode_ = speed;
      // 第一条 episode 的完整因果链快照（只打印一次）
      reportFirstStopAndGoIfNeeded();
    }
    motion_min_speed_in_episode_ = std::min(motion_min_speed_in_episode_, speed);
  }
  else
  {
    motion_last_fast_speed_ = speed;
    if (motion_current_low_speed_start_ >= 0.0)
    {
      const double duration = now.toSec() - motion_current_low_speed_start_;
      if (std::isfinite(duration) && duration >= 0.0)
        motion_low_speed_episodes_.push_back(duration);
      motion_current_low_speed_start_ = -1.0;
    }
  }
}

void reportMotionContinuity(bool final_report)
{
  const ros::WallTime now = ros::WallTime::now();
  if (!final_report && !motion_last_report_wall_.isZero() &&
      (now - motion_last_report_wall_).toSec() < 5.0)
    return;
  if (final_report)
  {
    if (motion_final_emitted_)
      return;
    motion_final_emitted_ = true;
  }
  motion_last_report_wall_ = now;
  double p50 = 0.0, p95 = 0.0, max_episode = 0.0;
  if (!motion_low_speed_episodes_.empty())
  {
    std::vector<double> sorted = motion_low_speed_episodes_;
    std::sort(sorted.begin(), sorted.end());
    const size_t last = sorted.size() - 1;
    p50 = sorted[static_cast<size_t>(std::floor(0.50 * last))];
    p95 = sorted[static_cast<size_t>(std::floor(0.95 * last))];
    max_episode = sorted.back();
  }
  double gap_p50 = 0.0, gap_p95 = 0.0, gap_max = 0.0;
  if (!activation_gap_samples_.empty())
  {
    std::vector<double> sorted = activation_gap_samples_;
    std::sort(sorted.begin(), sorted.end());
    const size_t last = sorted.size() - 1;
    gap_p50 = sorted[static_cast<size_t>(std::floor(0.50 * last))];
    gap_p95 = sorted[static_cast<size_t>(std::floor(0.95 * last))];
    gap_max = sorted.back();
  }
  ROS_INFO("[motion-continuity] drone=%d MOVING_DURATION=%.3f "
           "LOW_SPEED_EPISODE_COUNT=%lu LOW_SPEED_EPISODE_P50=%.4f "
           "LOW_SPEED_EPISODE_P95=%.4f LOW_SPEED_EPISODE_MAX=%.4f "
           "TRAJECTORY_ACTIVATION_GAP_P50=%.4f TRAJECTORY_ACTIVATION_GAP_P95=%.4f "
           "TRAJECTORY_ACTIVATION_GAP_MAX=%.4f MAX_SUCCESSOR_GAP=%.4f "
           "LOW_SPEED_THRESHOLD=%.3f",
           trajectory_drone_id_, motion_moving_duration_,
           static_cast<unsigned long>(motion_low_speed_episodes_.size()),
           p50, p95, max_episode, gap_p50, gap_p95, gap_max,
           max_successor_gap_, kMotionContinuityLowSpeedThreshold);
}

// 第一条 low-speed episode 的完整因果链快照（只打印一次）。
void reportFirstStopAndGoIfNeeded()
{
  if (motion_first_event_reported_)
    return;
  motion_first_event_reported_ = true;
  ROS_WARN("[first-stop-and-go] drone=%d timestamp=%.9f speed_before=%.6f "
           "min_speed=%.6f active_trajectory_id=%d active_generation=%lu "
           "remaining_duration=%.6f odom_speed=%.6f",
           trajectory_drone_id_, ros::Time::now().toSec(), motion_speed_before_event_,
           motion_min_speed_in_episode_, traj_id_,
           static_cast<unsigned long>(active_source_.generation),
           std::max(0.0, traj_duration_ - (ros::Time::now() - start_time_).toSec()),
           measured_speed_);
}

void recordExecutionPeriod(const ros::WallTime &now, ros::WallTime &last,
                           double &maximum, uint64_t &count,
                           const char *stage)
{
  if (!last.isZero())
  {
    const double gap = (now - last).toSec();
    if (std::isfinite(gap) && gap >= 0.0)
    {
      maximum = std::max(maximum, gap);
      if (gap > EXECUTION_GAP_WARN_SEC)
      {
        ROS_WARN_THROTTLE(
            1.0,
            "[execution-gap] node=%s stage=%s gap=%.6f threshold=%.3f",
            ros::this_node::getName().c_str(), stage, gap,
            EXECUTION_GAP_WARN_SEC);
      }
    }
  }
  last = now;
  ++count;
}

void maybeReportExecutionTiming(const ros::WallTime &now)
{
  if (last_execution_summary_wall_.isZero())
  {
    last_execution_summary_wall_ = now;
    return;
  }
  if ((now - last_execution_summary_wall_).toSec() < 30.0)
    return;

  last_execution_summary_wall_ = now;
  ROS_INFO("[execution-timing-summary] node=%s "
           "traj_server_command_timer_max_gap=%.6f "
           "position_command_publish_max_gap=%.6f "
           "command_timer_count=%llu command_publish_count=%llu",
           ros::this_node::getName().c_str(), max_cmd_callback_gap_,
           max_cmd_publish_gap_,
           static_cast<unsigned long long>(cmd_callback_count_),
           static_cast<unsigned long long>(cmd_publish_count_));
}

void updateTrackingReady(const ros::Time &now)
{
  bool changed = false;
  bool ready = false;
  bool odom_valid = false;
  bool target_valid = false;
  int traj_id = -1;
  double remaining_duration = 0.0;
  {
    std::lock_guard<std::mutex> lock(trajectory_state_mutex_);
    const double certified_duration = std::max(0.0, std::min(
        traj_duration_, active_source_.checked_until - start_time_.toSec()));
    if (receive_traj_ && traj_)
      remaining_duration = std::max(0.0, certified_duration -
          (now - start_time_).toSec());
    const double t_cur = (now - start_time_).toSec();
    const bool active_trajectory =
        receive_traj_ && traj_ && t_cur >= 0.0 &&
        t_cur < certified_duration;
    ready = odom_valid_ && target_odom_valid_ && active_trajectory;
    odom_valid = odom_valid_;
    target_valid = target_odom_valid_;
    changed = ready != tracking_ready_;
    tracking_ready_ = ready;
    traj_id = traj_id_;
  }
  if (!changed)
    return;

  std_msgs::Bool msg;
  msg.data = ready;
  tracking_ready_pub.publish(msg);
  ROS_INFO("[traj-server-tracking-ready] node=%s ready=%d traj_id=%d "
           "remaining_duration=%.6f odom_valid=%d target_valid=%d",
           ros::this_node::getName().c_str(), ready ? 1 : 0, traj_id,
           remaining_duration, odom_valid ? 1 : 0, target_valid ? 1 : 0);
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt)
{
  constexpr double YAW_DOT_MAX_PER_SEC = 2 * M_PI;
  constexpr double YAW_DOT_DOT_MAX_PER_SEC = 5 * M_PI;
  std::pair<double, double> yaw_yawdot(0, 0);

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                            ? traj_->getPos(t_cur + time_forward_) - pos
                            : traj_->getPos(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1
                        ? atan2(dir(1), dir(0))
                        : last_yaw_;

  double yawdot = 0;
  double d_yaw = yaw_temp - last_yaw_;
  if (d_yaw >= M_PI)
  {
    d_yaw -= 2 * M_PI;
  }
  if (d_yaw <= -M_PI)
  {
    d_yaw += 2 * M_PI;
  }

  const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
  const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
  double d_yaw_max;
  if (fabs(last_yawdot_ + dt * YDDM) <= fabs(YDM))
  {
    // yawdot = last_yawdot_ + dt * YDDM;
    d_yaw_max = last_yawdot_ * dt + 0.5 * YDDM * dt * dt;
  }
  else
  {
    // yawdot = YDM;
    double t1 = (YDM - last_yawdot_) / YDDM;
    d_yaw_max = ((dt - t1) + dt) * (YDM - last_yawdot_) / 2.0;
  }

  if (fabs(d_yaw) > fabs(d_yaw_max))
  {
    d_yaw = d_yaw_max;
  }
  yawdot = d_yaw / dt;

  double yaw = last_yaw_ + d_yaw;
  if (yaw > M_PI)
    yaw -= 2 * M_PI;
  if (yaw < -M_PI)
    yaw += 2 * M_PI;
  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  last_yaw_ = yaw_yawdot.first;
  last_yawdot_ = yaw_yawdot.second;

  return yaw_yawdot;
}

std::pair<double, double> calculate_target_facing_yaw(const Eigen::Vector3d &pos, double dt)
{
  if (!target_odom_valid_ || dt <= 1.0e-6)
    return std::make_pair(last_yaw_, 0.0);

  const Eigen::Vector3d delta = target_position_ - pos;
  if (delta.head<2>().norm() <= 0.1)
    return std::make_pair(last_yaw_, 0.0);
  multi_uav_formation::PredictedYawState state;
  state.valid = true;
  state.yaw = last_yaw_;
  state.yaw_rate = last_yawdot_;
  state = multi_uav_formation::advanceTargetFacingYaw(
      state, std::atan2(delta.y(), delta.x()), dt, 2.0 * M_PI,
      5.0 * M_PI);
  last_yaw_ = state.yaw;
  last_yawdot_ = state.yaw_rate;
  return std::make_pair(state.yaw, state.yaw_rate);
}

std::pair<double, double> optimized_yaw_at(const double t_cur)
{
  if (!optimized_yaw_valid_ || optimized_yaw_sample_times_.size() < 2 ||
      optimized_yaw_sample_times_.size() != optimized_yaw_samples_.size())
    return std::make_pair(last_yaw_, last_yawdot_);
  const auto upper = std::lower_bound(optimized_yaw_sample_times_.begin(),
                                      optimized_yaw_sample_times_.end(), t_cur);
  if (upper == optimized_yaw_sample_times_.begin())
    return std::make_pair(optimized_yaw_samples_.front(), 0.0);
  if (upper == optimized_yaw_sample_times_.end())
  {
    const size_t n = optimized_yaw_samples_.size();
    const double dt = optimized_yaw_sample_times_[n - 1] -
                      optimized_yaw_sample_times_[n - 2];
    return std::make_pair(optimized_yaw_samples_.back(),
                          dt > 1.0e-6 ? (optimized_yaw_samples_[n - 1] -
                              optimized_yaw_samples_[n - 2]) / dt : 0.0);
  }
  const size_t high = static_cast<size_t>(upper - optimized_yaw_sample_times_.begin());
  const size_t low = high - 1;
  const double dt = optimized_yaw_sample_times_[high] -
                    optimized_yaw_sample_times_[low];
  if (dt <= 1.0e-6)
    return std::make_pair(optimized_yaw_samples_[low], 0.0);
  const double alpha = (t_cur - optimized_yaw_sample_times_[low]) / dt;
  double difference = optimized_yaw_samples_[high] - optimized_yaw_samples_[low];
  while (difference > M_PI) difference -= 2.0 * M_PI;
  while (difference < -M_PI) difference += 2.0 * M_PI;
  double yaw = optimized_yaw_samples_[low] + alpha * difference;
  while (yaw > M_PI) yaw -= 2.0 * M_PI;
  while (yaw < -M_PI) yaw += 2.0 * M_PI;
  return std::make_pair(yaw, difference / dt);
}

void publish_cmd(Vector3d p, Vector3d v, Vector3d a, Vector3d j, double y, double yd)
{

  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.yaw = y;
  cmd.yaw_dot = yd;
  pos_cmd_pub.publish(cmd);
  recordExecutionPeriod(ros::WallTime::now(), last_cmd_publish_wall_,
                        max_cmd_publish_gap_, cmd_publish_count_,
                        "POSITION_COMMAND_PUBLISH");

  last_pos_ = p;
}

void cmdCallback(const ros::TimerEvent &e)
{
  const ros::WallTime callback_wall = ros::WallTime::now();
  recordExecutionPeriod(callback_wall, last_cmd_callback_wall_,
                        max_cmd_callback_gap_, cmd_callback_count_,
                        "TRAJ_SERVER_COMMAND_TIMER");
  maybeReportExecutionTiming(callback_wall);

  std::unique_lock<std::mutex> lock(trajectory_state_mutex_);
  const ros::Time time_now = ros::Time::now();
  // ---- single activation timeline -----------------------------------------
  // Promote the earliest pending reservation whose activation time has arrived,
  // regardless of which slot it lives in.  Feedback092 defect: the timer only
  // consulted the scheduled slot, so a Local guaranteed successor queued behind
  // a later-activating Team speculative entry never activated at all.
  normalizeFutureTimeline();
  if (scheduled_traj_valid_ && scheduled_traj_ &&
      time_now + ros::Duration(1.0e-9) >= scheduled_start_time_)
  {
    if(!executionHandoffGate(*scheduled_traj_,scheduled_source_,"ACTIVATION"))
    {
      ROS_ERROR("[traj-server-handoff-reject] node=%s trajectory_id=%d "
                "reason=%s expected_predecessor_id=%d",
                ros::this_node::getName().c_str(),scheduled_traj_id_,
                last_handoff_reject_reason_.c_str(),
                scheduled_expected_predecessor_id_);
      const bool was_team = scheduled_team_speculative_;
      const uint64_t aborted_transaction =
          static_cast<uint64_t>(scheduled_source_.team_solution_id);
      markLineageInvalidated(scheduled_traj_id_);
      clearScheduledSlot();
      if (was_team)
      {
        ++team_speculative_abort_count_;
        ROS_ERROR("[team-speculative-abort] transaction_id=%lu "
                  "trajectory_id=%d reason=ACTIVATION_PREDECESSOR_%s "
                  "local_authoritative_predecessor_unchanged=1",
                  static_cast<unsigned long>(aborted_transaction),
                  scheduled_traj_id_, last_handoff_reject_reason_.c_str());
      }
      normalizeFutureTimeline();
      return;
    }
    exitTerminalHold(time_now.toSec());
    const int previous_traj_id = traj_id_;
    const bool activated_team = scheduled_team_speculative_;
    const uint64_t activated_transaction =
        static_cast<uint64_t>(scheduled_source_.team_solution_id);
    traj_ = scheduled_traj_;
    traj_duration_ = scheduled_traj_duration_;
    start_time_ = scheduled_start_time_;
    traj_id_ = scheduled_traj_id_;
    receive_traj_ = true;
    terminal_hold_active_ = false;
    optimized_yaw_valid_ = scheduled_optimized_yaw_valid_;
    optimized_yaw_execution_reported_ = false;
    active_source_ = scheduled_source_;
    optimized_yaw_sample_times_ = scheduled_yaw_sample_times_;
    optimized_yaw_samples_ = scheduled_yaw_samples_;
    clearScheduledSlot();
    if (previous_traj_id >= 0 && previous_traj_id != traj_id_)
      retireLineageRecord(previous_traj_id);
    noteLineageRecord(active_source_, traj_, true);
    normalizeFutureTimeline();
    ROS_INFO("[EXECUTION_IDENTITY] stage=ACTIVATED trajectory_id=%d "
             "ACTIVATION_REVISION=%lu payload_hash=%lu "
             "checked_until=%.9f",
             traj_id_,
             static_cast<unsigned long>(active_source_.validation_revision),
             static_cast<unsigned long>(active_source_.payload_hash),
             active_source_.checked_until);
    ROS_INFO("[traj-server-scheduled-activate] node=%s trajectory_id=%d "
             "previous_trajectory_id=%d activation_time=%.9f actual_time=%.9f "
             "activation_error=%.6f duration=%.6f lineage_class=%s "
             "queue_depth=%zu",
             ros::this_node::getName().c_str(), traj_id_, previous_traj_id,
             start_time_.toSec(), time_now.toSec(),
             (time_now - start_time_).toSec(), traj_duration_,
             activated_team ? kLineageClassTeam : kLineageClassLocal,
             scheduled_traj_queue_.size());
    ROS_INFO("[execution-activated] trajectory_id=%d previous_trajectory_id=%d "
             "source=%s transaction_id=%lu expected_predecessor_id=%d "
             "lineage_class=%s activation_time=%.9f",
             traj_id_, previous_traj_id, active_source_.trajectory_source.c_str(),
             static_cast<unsigned long>(activated_transaction),
             active_source_.expected_predecessor_id,
             activated_team ? kLineageClassTeam : kLineageClassLocal,
             start_time_.toSec());
    if (activated_team)
      ROS_INFO("[team-speculative] transaction_id=%lu trajectory_id=%d "
               "state=ACTIVE local_authoritative_predecessor_unchanged=0",
               static_cast<unsigned long>(activated_transaction), traj_id_);
    ROS_INFO("[active-traj-lifecycle] event=ACTIVATED "
             "traj_identity=start=%.9f,trajectory_id=%d "
             "source=traj_server_scheduled_activation start_time=%.9f "
             "duration=%.6f active=1 reason=COMMON_TEAM_ACTIVATION",
             start_time_.toSec(), traj_id_, start_time_.toSec(),
             traj_duration_);
  }
  if (!receive_traj_ || !traj_)
  {
    lock.unlock();
    updateTrackingReady(time_now);
    return;
  }

  const double t_cur = (time_now - start_time_).toSec();
  const double certified_duration = std::max(0.0, std::min(
      traj_duration_, active_source_.checked_until - start_time_.toSec()));
  const double remaining_duration = std::max(0.0, certified_duration - t_cur);
  const bool trajectory_executable =
      t_cur >= 0.0 && t_cur < certified_duration;
  const bool heartbeat_received = heartbeat_time_.toSec() > 1e-5;
  const bool heartbeat_stale =
      !heartbeat_received || (time_now - heartbeat_time_).toSec() > 0.5;

  if (heartbeat_stale && !heartbeat_stale_)
  {
    const char *action = trajectory_executable
                             ? "CONTINUE_ACTIVE_TRAJECTORY"
                             : (t_cur >= certified_duration
                                    ? "TERMINAL_HOLD"
                                    : "WAIT_FOR_TRAJECTORY_START");
    ROS_WARN("[traj-server-heartbeat-stale] node=%s traj_id=%d "
             "remaining_duration=%.6f action=%s",
             ros::this_node::getName().c_str(), traj_id_,
             remaining_duration, action);
    heartbeat_stale_ = true;
  }

  if (last_activation_time_ > 0.0)
  {
    const double gap = start_time_.toSec() - last_activation_time_;
    if (std::isfinite(gap) && gap >= 0.0)
    {
      activation_gap_samples_.push_back(gap);
      max_successor_gap_ = std::max(max_successor_gap_, gap);
    }
  }
  last_activation_time_ = start_time_.toSec();

  if (t_cur >= certified_duration)
  {
    const Eigen::Vector3d terminal_pos =
        trajectory_lifecycle::sample(*traj_, certified_duration).p;
    if (!terminal_hold_active_)
    {
      ROS_WARN("[traj-server-terminal-hold] node=%s traj_id=%d "
               "remaining_duration=0 reason=CERTIFICATE_OR_TRAJECTORY_EXPIRED_NO_REPLACEMENT",
               ros::this_node::getName().c_str(), traj_id_);
      terminal_hold_active_ = true;
      terminal_hold_started_=time_now.toSec();
      ROS_ERROR("[execution-lifecycle] event=TERMINAL_HOLD_ENTER node=%s trajectory_id=%d generation=%lu timestamp=%.9f end_time=%.9f source=%s",
          ros::this_node::getName().c_str(),traj_id_,static_cast<unsigned long>(active_source_.generation),time_now.toSec(),start_time_.toSec()+certified_duration,active_source_.trajectory_source.c_str());
      ROS_ERROR("[continuous-motion] event=TRAJECTORY_END_BEFORE_NEXT_ACTIVATION node=%s trajectory_id=%d run_fail=1",ros::this_node::getName().c_str(),traj_id_);
      active_source_.lifecycle_state="TERMINAL_HOLD";
      active_source_.safety_validated=false;
      std_msgs::String hold_source;
      std::ostringstream hs;
      hs << "{\"drone_id\":" << active_source_.drone_id << ",\"trajectory_id\":" << traj_id_
         << ",\"generation\":" << active_source_.generation << ",\"team_solution_id\":" << active_source_.team_solution_id
         << ",\"encirclement_generation\":" << active_source_.encirclement_generation
         << ",\"hypothesis_id\":" << active_source_.viewpoint_hypothesis_id
         << ",\"trajectory_source\":\"TERMINAL_HOLD\",\"safety_validated\":false}";
      hold_source.data=hs.str();execution_source_pub.publish(hold_source);
      last_pos_ = terminal_pos;
    }
    publish_cmd(terminal_pos, Vector3d::Zero(), Vector3d::Zero(),
                Vector3d::Zero(), last_yaw_, 0.0);
    lock.unlock();
    updateTrackingReady(time_now);
    return;
  }

  if (t_cur < 0.0)
  {
    lock.unlock();
    updateTrackingReady(time_now);
    return;
  }

  if (source_reported_traj_id_ != traj_id_ || reported_source_ != active_source_.trajectory_source || reported_safety_validated_ != active_source_.safety_validated)
  {
    const bool new_polynomial=source_reported_traj_id_ != traj_id_;
    source_reported_traj_id_=traj_id_;
    reported_source_=active_source_.trajectory_source;
    reported_safety_validated_=active_source_.safety_validated;
    std::ostringstream stream;
    stream.precision(17);
    stream << "{\"drone_id\":" << active_source_.drone_id
           << ",\"trajectory_id\":" << traj_id_
           << ",\"generation\":" << active_source_.generation
           << ",\"team_reference_id\":" << active_source_.team_reference_id
           << ",\"team_reference_reason\":\"" << active_source_.team_reference_reason << "\""
           << ",\"team_solution_id\":" << active_source_.team_solution_id
           << ",\"encirclement_generation\":" << active_source_.encirclement_generation
           << ",\"hypothesis_id\":" << active_source_.viewpoint_hypothesis_id
           << ",\"trajectory_source\":\"" << active_source_.trajectory_source
           << "\",\"timestamp\":" << time_now.toSec()
           << ",\"activation_time\":" << start_time_.toSec()
           << ",\"safety_validated\":" << (active_source_.safety_validated ? "true" : "false") << "}";
    std_msgs::String source; source.data=stream.str();
    execution_source_pub.publish(source);
    if(new_polynomial) execution_adoption_pub.publish(active_source_);
    ROS_INFO("[executed-trajectory-source] %s",source.data.c_str());
  }

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero());
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
  pos = traj_->getPos(t_cur);
  vel = traj_->getVel(t_cur);
  acc = traj_->getAcc(t_cur);
  jer = traj_->getJer(t_cur);

  /*** calculate yaw ***/
  const double yaw_dt = (time_now - time_last).toSec();
  if (optimized_yaw_valid_ && (active_source_.team_reference_id==0 || t_cur<=optimized_yaw_sample_times_.back()))
  {
    yaw_yawdot = optimized_yaw_at(t_cur);
    if (!optimized_yaw_execution_reported_)
    {
      ROS_INFO("[optimized-yaw-executed] node=%s trajectory_id=%d "
               "OPTIMIZED_YAW_VALID=1 OPTIMIZED_YAW_EXECUTED=1 "
               "sample_count=%zu activation_time=%.9f",
               ros::this_node::getName().c_str(), traj_id_,
               optimized_yaw_samples_.size(), start_time_.toSec());
      optimized_yaw_execution_reported_ = true;
    }
  }
  else if (enable_target_facing_yaw_)
    yaw_yawdot = calculate_target_facing_yaw(pos, yaw_dt);
  else
    yaw_yawdot = std::make_pair(0.0, 0.0);
  /*** calculate yaw ***/

  time_last = time_now;
  last_yaw_ = yaw_yawdot.first;
  last_yawdot_ = yaw_yawdot.second;
  last_pos_ = pos;

  slowly_flip_yaw_target_ = yaw_yawdot.first + M_PI;
  if (slowly_flip_yaw_target_ > M_PI)
    slowly_flip_yaw_target_ -= 2 * M_PI;
  if (slowly_flip_yaw_target_ < -M_PI)
    slowly_flip_yaw_target_ += 2 * M_PI;
  constexpr double CENTER[2] = {0.0, 0.0};
  slowly_turn_to_center_target_ = atan2(CENTER[1] - pos(1), CENTER[0] - pos(0));

  // publish
  if (!enable_target_facing_yaw_ && !optimized_yaw_valid_)
  {
    yaw_yawdot.first = 0.0;
    yaw_yawdot.second = 0.0;
  }
  publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  lock.unlock();
  updateTrackingReady(time_now);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");
  nh.param("/trajectory_lifecycle/handoff_position_tolerance",handoff_tolerances_.p,0.02);
  nh.param("/trajectory_lifecycle/handoff_velocity_tolerance",handoff_tolerances_.v,0.05);
  nh.param("/trajectory_lifecycle/handoff_acceleration_tolerance",handoff_tolerances_.a,0.10);

  ros::Subscriber poly_traj_sub = nh.subscribe("planning/trajectory", 10, polyTrajCallback);
  ros::Subscriber team_solution_sub = nh.subscribe<traj_utils::TeamTrajectorySolution>(
      "/topology_coordination/team_solution", 8, teamSolutionCallback,
      ros::TransportHints().tcpNoDelay());
  team_trajectory_ack_pub = nh.advertise<traj_utils::TeamTrajectoryAck>(
      "/topology_coordination/team_ack", 8);
  ros::Subscriber heartbeat_sub = nh.subscribe("heartbeat", 10, heartbeatCallback);
  ros::Subscriber target_odom_sub = nh.subscribe("/object_odom", 10, targetOdomCallback);
  ros::Subscriber odom_sub = nh.subscribe("odom", 10, odomCallback,
                                          ros::TransportHints().tcpNoDelay());

  execution_adoption_pub = nh.advertise<traj_utils::PolyTraj>("/trajectory_execution/activated",30,false);
  execution_source_pub = nh.advertise<std_msgs::String>("/trajectory_execution/source", 30, false);
  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  tracking_ready_pub = nh.advertise<std_msgs::Bool>("tracking_ready", 1, true);

  std_msgs::Bool initial_ready;
  initial_ready.data = false;
  tracking_ready_pub.publish(initial_ready);

  ros::CallbackQueue execution_callback_queue;
  ros::NodeHandle execution_nh(nh);
  execution_nh.setCallbackQueue(&execution_callback_queue);
  ros::Timer cmd_timer = execution_nh.createTimer(ros::Duration(0.01), cmdCallback);
  ros::AsyncSpinner execution_spinner(1, &execution_callback_queue);

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  nh.param("traj_server/enable_target_facing_yaw", enable_target_facing_yaw_, true);
  // 运动连续性遥测：本进程只服务一架 UAV，drone_id 从私有参数读取。
  nh.param("/trajectory_lifecycle/drone_id", trajectory_drone_id_, -1);
  if (trajectory_drone_id_ < 0)
  {
    const std::string node_name = ros::this_node::getName();
    const size_t first = node_name.find("drone_");
    if (first != std::string::npos)
      trajectory_drone_id_ = std::atoi(node_name.substr(first + 6, 1).c_str());
  }
  last_yaw_ = 0.0;
  last_yawdot_ = 0.0;

  ROS_INFO("[alp-camera-yaw] enable_target_facing_yaw=%d hfov_deg=85.0 max_range_m=8.0 yaw_rate_limit_rad_s=%.3f yaw_acc_limit_rad_s2=%.3f camera_extrinsic_yaw_rad=0.0",
           enable_target_facing_yaw_ ? 1 : 0, 2.0 * M_PI, 5.0 * M_PI);

  ros::Duration(1.0).sleep();

  ROS_INFO("[Traj server]: ready.");

  execution_spinner.start();
  ros::spin();
  execution_spinner.stop();
  reportMotionContinuity(true);

  ROS_INFO("[execution-timing-summary] node=%s "
           "traj_server_command_timer_max_gap=%.6f "
           "position_command_publish_max_gap=%.6f "
           "command_timer_count=%llu command_publish_count=%llu",
           ros::this_node::getName().c_str(), max_cmd_callback_gap_,
           max_cmd_publish_gap_,
           static_cast<unsigned long long>(cmd_callback_count_),
           static_cast<unsigned long long>(cmd_publish_count_));

  return 0;
}
