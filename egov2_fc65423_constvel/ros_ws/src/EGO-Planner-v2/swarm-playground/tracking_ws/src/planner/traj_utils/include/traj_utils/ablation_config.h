#pragma once

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>
#include <vector>

namespace traj_utils
{

enum class AblationMode
{
  FULL,
  NO_LOCAL_VIS,
  NO_TEAM_PT,
  NO_LOS_TOPOLOGY,
  NO_DYNAMIC_BODY_TOPOLOGY,
  NO_TARGET_FACING_ASTAR,
  NO_SAFE_SEED_RETENTION,
  NO_LOCAL_GAP
};

struct AblationConfig
{
  AblationMode mode{AblationMode::FULL};

  bool local_visibility{true};
  bool team_pt{true};
  bool los_topology{true};
  bool dynamic_body_topology{true};
  bool target_facing_astar{true};
  bool safe_seed_retention{true};
  bool local_gap{true};

  // Formal experiment invariants.  These are deliberately not ablation
  // variables and therefore never vary between the eight modes.
  bool encirclement{true};
  bool cooperative_reference{true};
  bool physical_safety{true};
  bool joint_yaw{false};
};

inline std::string normalizeAblationModeName(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(),
                 [](const unsigned char ch) {
                   return static_cast<char>(std::toupper(ch));
                 });
  std::replace(value.begin(), value.end(), '-', '_');
  return value;
}

inline const char *toString(const AblationMode mode)
{
  switch (mode)
  {
    case AblationMode::FULL: return "FULL";
    case AblationMode::NO_LOCAL_VIS: return "NO_LOCAL_VIS";
    case AblationMode::NO_TEAM_PT: return "NO_TEAM_PT";
    case AblationMode::NO_LOS_TOPOLOGY: return "NO_LOS_TOPOLOGY";
    case AblationMode::NO_DYNAMIC_BODY_TOPOLOGY:
      return "NO_DYNAMIC_BODY_TOPOLOGY";
    case AblationMode::NO_TARGET_FACING_ASTAR:
      return "NO_TARGET_FACING_ASTAR";
    case AblationMode::NO_SAFE_SEED_RETENTION:
      return "NO_SAFE_SEED_RETENTION";
    case AblationMode::NO_LOCAL_GAP: return "NO_LOCAL_GAP";
  }
  return "INVALID";
}

inline const char *toCliString(const AblationMode mode)
{
  switch (mode)
  {
    case AblationMode::FULL: return "full";
    case AblationMode::NO_LOCAL_VIS: return "no_local_vis";
    case AblationMode::NO_TEAM_PT: return "no_team_pt";
    case AblationMode::NO_LOS_TOPOLOGY: return "no_los_topology";
    case AblationMode::NO_DYNAMIC_BODY_TOPOLOGY:
      return "no_dynamic_body_topology";
    case AblationMode::NO_TARGET_FACING_ASTAR:
      return "no_target_facing_astar";
    case AblationMode::NO_SAFE_SEED_RETENTION:
      return "no_safe_seed_retention";
    case AblationMode::NO_LOCAL_GAP: return "no_local_gap";
  }
  return "invalid";
}

inline bool parseAblationMode(const std::string &value, AblationMode &mode)
{
  const std::string normalized = normalizeAblationModeName(value);
  if (normalized == "FULL") mode = AblationMode::FULL;
  else if (normalized == "NO_LOCAL_VIS") mode = AblationMode::NO_LOCAL_VIS;
  else if (normalized == "NO_TEAM_PT") mode = AblationMode::NO_TEAM_PT;
  else if (normalized == "NO_LOS_TOPOLOGY")
    mode = AblationMode::NO_LOS_TOPOLOGY;
  else if (normalized == "NO_DYNAMIC_BODY_TOPOLOGY")
    mode = AblationMode::NO_DYNAMIC_BODY_TOPOLOGY;
  else if (normalized == "NO_TARGET_FACING_ASTAR")
    mode = AblationMode::NO_TARGET_FACING_ASTAR;
  else if (normalized == "NO_SAFE_SEED_RETENTION")
    mode = AblationMode::NO_SAFE_SEED_RETENTION;
  else if (normalized == "NO_LOCAL_GAP")
    mode = AblationMode::NO_LOCAL_GAP;
  else
    return false;
  return true;
}

inline AblationMode parseAblationMode(const std::string &value)
{
  AblationMode mode = AblationMode::FULL;
  if (!parseAblationMode(value, mode))
    throw std::invalid_argument("Unknown ALP formal ablation mode: " + value);
  return mode;
}

inline AblationConfig makeAblationConfig(const AblationMode mode)
{
  AblationConfig config;
  config.mode = mode;
  switch (mode)
  {
    case AblationMode::FULL: break;
    case AblationMode::NO_LOCAL_VIS: config.local_visibility = false; break;
    case AblationMode::NO_TEAM_PT: config.team_pt = false; break;
    case AblationMode::NO_LOS_TOPOLOGY: config.los_topology = false; break;
    case AblationMode::NO_DYNAMIC_BODY_TOPOLOGY:
      config.dynamic_body_topology = false;
      break;
    case AblationMode::NO_TARGET_FACING_ASTAR:
      config.target_facing_astar = false;
      break;
    case AblationMode::NO_SAFE_SEED_RETENTION:
      config.safe_seed_retention = false;
      break;
    case AblationMode::NO_LOCAL_GAP: config.local_gap = false; break;
  }
  return config;
}

inline std::vector<AblationMode> formalAblationModes()
{
  return {
      AblationMode::FULL,
      AblationMode::NO_LOCAL_VIS,
      AblationMode::NO_TEAM_PT,
      AblationMode::NO_LOS_TOPOLOGY,
      AblationMode::NO_DYNAMIC_BODY_TOPOLOGY,
      AblationMode::NO_TARGET_FACING_ASTAR,
      AblationMode::NO_SAFE_SEED_RETENTION,
      AblationMode::NO_LOCAL_GAP};
}

}  // namespace traj_utils
