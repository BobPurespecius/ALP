#include <traj_utils/ablation_config.h>

#include <array>
#include <cstdlib>
#include <iostream>

namespace
{

std::array<bool, 7> scientificSwitches(const traj_utils::AblationConfig &c)
{
  return {{c.local_visibility, c.team_pt, c.los_topology,
           c.dynamic_body_topology, c.target_facing_astar,
           c.safe_seed_retention, c.local_gap}};
}

}  // namespace

int main()
{
  const auto modes = traj_utils::formalAblationModes();
  if (modes.size() != 8)
    return EXIT_FAILURE;
  const auto full = traj_utils::makeAblationConfig(
      traj_utils::AblationMode::FULL);
  const auto full_switches = scientificSwitches(full);
  for (const bool enabled : full_switches)
    if (!enabled)
      return EXIT_FAILURE;

  std::cout << "Mode LocalVis TeamPT LOSTopo DynBodyTopo TargetA* "
               "SeedRetain LocalGap\n";
  for (std::size_t row = 0; row < modes.size(); ++row)
  {
    const auto config = traj_utils::makeAblationConfig(modes[row]);
    const auto switches = scientificSwitches(config);
    std::size_t differences = 0;
    for (std::size_t column = 0; column < switches.size(); ++column)
      differences += switches[column] != full_switches[column] ? 1 : 0;
    const std::size_t expected = row == 0 ? 0 : 1;
    if (differences != expected || !config.encirclement ||
        !config.cooperative_reference || !config.physical_safety ||
        config.joint_yaw)
      return EXIT_FAILURE;
    if (row > 0 && switches[row - 1])
      return EXIT_FAILURE;
    std::cout << traj_utils::toString(config.mode);
    for (const bool enabled : switches)
      std::cout << ' ' << static_cast<int>(enabled);
    std::cout << '\n';
  }

  for (const auto mode : modes)
  {
    traj_utils::AblationMode parsed = traj_utils::AblationMode::FULL;
    if (!traj_utils::parseAblationMode(traj_utils::toCliString(mode), parsed) ||
        parsed != mode)
      return EXIT_FAILURE;
  }
  traj_utils::AblationMode invalid = traj_utils::AblationMode::FULL;
  if (traj_utils::parseAblationMode("no_local_vis,no_team_pt", invalid))
    return EXIT_FAILURE;

  std::cout << "ABLATION_CONFIG_CONTRACT=PASS\n";
  return EXIT_SUCCESS;
}
