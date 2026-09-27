#pragma once

#include <atomic>
#include <cstdint>
#include <string>

#include <ros/ros.h>

#include <traj_utils/ablation_config.h>

namespace traj_utils
{

enum class AblationCounter
{
  LOCAL_VIS_COST_EVAL = 0,
  TEAM_PT_ATTEMPT,
  LOS_TOPOLOGY_DISPATCH,
  DYNAMIC_BODY_TOPOLOGY_DISPATCH,
  TARGET_FACING_ASTAR_ATTEMPT,
  SAFE_SEED_RETENTION_USED,
  LOCAL_GAP_COST_EVAL,
  COUNT
};

inline const char *counterName(const AblationCounter counter)
{
  switch (counter)
  {
    case AblationCounter::LOCAL_VIS_COST_EVAL:
      return "LOCAL_VIS_COST_EVAL_COUNT";
    case AblationCounter::TEAM_PT_ATTEMPT:
      return "TEAM_PT_ATTEMPT_COUNT";
    case AblationCounter::LOS_TOPOLOGY_DISPATCH:
      return "LOS_TOPOLOGY_DISPATCH_COUNT";
    case AblationCounter::DYNAMIC_BODY_TOPOLOGY_DISPATCH:
      return "DYNAMIC_BODY_TOPOLOGY_DISPATCH_COUNT";
    case AblationCounter::TARGET_FACING_ASTAR_ATTEMPT:
      return "TARGET_FACING_ASTAR_ATTEMPT_COUNT";
    case AblationCounter::SAFE_SEED_RETENTION_USED:
      return "SAFE_SEED_RETENTION_USED_COUNT";
    case AblationCounter::LOCAL_GAP_COST_EVAL:
      return "LOCAL_GAP_COST_EVAL_COUNT";
    case AblationCounter::COUNT: break;
  }
  return "INVALID_ABLATION_COUNTER";
}

inline std::atomic<std::uint64_t> *ablationCounterStorage()
{
  static std::atomic<std::uint64_t> counters[
      static_cast<std::size_t>(AblationCounter::COUNT)];
  return counters;
}

inline void logAblationCounterZeros(const char *component,
                                    const AblationConfig &config)
{
  ROS_INFO("[ablation-counters] component=%s mode=%s "
           "LOCAL_VIS_COST_EVAL_COUNT=0 TEAM_PT_ATTEMPT_COUNT=0 "
           "LOS_TOPOLOGY_DISPATCH_COUNT=0 "
           "DYNAMIC_BODY_TOPOLOGY_DISPATCH_COUNT=0 "
           "TARGET_FACING_ASTAR_ATTEMPT_COUNT=0 "
           "SAFE_SEED_RETENTION_USED_COUNT=0 LOCAL_GAP_COST_EVAL_COUNT=0",
           component, toString(config.mode));
}

inline std::uint64_t incrementAblationCounter(const AblationCounter counter,
                                              const char *component,
                                              const AblationConfig &config)
{
  const std::size_t index = static_cast<std::size_t>(counter);
  const std::uint64_t value = ++ablationCounterStorage()[index];
  // Log the first hit and powers of two: enough for branch proof without
  // turning a hot objective loop into high-volume telemetry.
  if (value == 1 || (value & (value - 1)) == 0)
    ROS_INFO("[ablation-counter] component=%s mode=%s %s=%lu",
             component, toString(config.mode), counterName(counter),
             static_cast<unsigned long>(value));
  return value;
}

inline AblationConfig loadAblationConfig(const char *component,
                                         bool *formal_mode = nullptr)
{
  std::string mode_name;
  const bool formal = ros::param::get("/alp/ablation_mode", mode_name);
  if (!formal)
    mode_name = "full";
  const AblationConfig config =
      makeAblationConfig(parseAblationMode(mode_name));
  if (formal_mode != nullptr)
    *formal_mode = formal;
  ROS_INFO("[ablation-config] component=%s mode=%s formal=%d "
           "local_visibility=%d team_pt=%d los_topology=%d "
           "dynamic_body_topology=%d target_facing_astar=%d "
           "safe_seed_retention=%d local_gap=%d encirclement=%d "
           "cooperative_reference=%d physical_safety=%d joint_yaw=%d",
           component, toString(config.mode), static_cast<int>(formal),
           static_cast<int>(config.local_visibility),
           static_cast<int>(config.team_pt),
           static_cast<int>(config.los_topology),
           static_cast<int>(config.dynamic_body_topology),
           static_cast<int>(config.target_facing_astar),
           static_cast<int>(config.safe_seed_retention),
           static_cast<int>(config.local_gap),
           static_cast<int>(config.encirclement),
           static_cast<int>(config.cooperative_reference),
           static_cast<int>(config.physical_safety),
           static_cast<int>(config.joint_yaw));
  logAblationCounterZeros(component, config);
  return config;
}

}  // namespace traj_utils
