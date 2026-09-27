#include <traj_utils/ablation_config.h>

#include <cstdlib>
#include <iostream>
#include <string>

namespace
{

const char *onOff(const bool enabled) { return enabled ? "ON" : "OFF"; }
const char *boolean(const bool enabled) { return enabled ? "true" : "false"; }

void printManifest(const traj_utils::AblationConfig &c)
{
  std::cout << "========================================\n"
            << "ALP FORMAL ABLATION\n"
            << "========================================\n"
            << "mode                       = " << traj_utils::toString(c.mode) << "\n\n"
            << "encirclement               = " << onOff(c.encirclement) << "   [LOCKED]\n"
            << "cooperative_reference      = " << onOff(c.cooperative_reference) << "   [LOCKED]\n"
            << "physical_safety            = " << onOff(c.physical_safety) << "   [LOCKED]\n"
            << "joint_yaw                  = " << onOff(c.joint_yaw) << "  [LOCKED]\n\n"
            << "local_visibility           = " << onOff(c.local_visibility) << "\n"
            << "team_pt                    = " << onOff(c.team_pt) << "\n"
            << "los_topology               = " << onOff(c.los_topology) << "\n"
            << "dynamic_body_topology      = " << onOff(c.dynamic_body_topology) << "\n"
            << "target_facing_astar        = " << onOff(c.target_facing_astar) << "\n"
            << "safe_seed_retention        = " << onOff(c.safe_seed_retention) << "\n"
            << "local_gap                  = " << onOff(c.local_gap) << "\n"
            << "========================================\n";
}

void printShell(const traj_utils::AblationConfig &c)
{
  std::cout << "ABLATION_CANONICAL_MODE=" << traj_utils::toString(c.mode) << '\n'
            << "ABLATION_CLI_MODE=" << traj_utils::toCliString(c.mode) << '\n'
            << "ABLATION_LOCAL_VIS=" << boolean(c.local_visibility) << '\n'
            << "ABLATION_TEAM_PT=" << boolean(c.team_pt) << '\n'
            << "ABLATION_LOS_TOPOLOGY=" << boolean(c.los_topology) << '\n'
            << "ABLATION_DYNAMIC_BODY_TOPOLOGY=" << boolean(c.dynamic_body_topology) << '\n'
            << "ABLATION_TARGET_FACING_ASTAR=" << boolean(c.target_facing_astar) << '\n'
            << "ABLATION_SAFE_SEED_RETENTION=" << boolean(c.safe_seed_retention) << '\n'
            << "ABLATION_LOCAL_GAP=" << boolean(c.local_gap) << '\n'
            << "ABLATION_ENCIRCLEMENT=" << boolean(c.encirclement) << '\n'
            << "ABLATION_COOPERATIVE_REFERENCE=" << boolean(c.cooperative_reference) << '\n'
            << "ABLATION_PHYSICAL_SAFETY=" << boolean(c.physical_safety) << '\n'
            << "ABLATION_JOINT_YAW=" << boolean(c.joint_yaw) << '\n';
}

}  // namespace

int main(int argc, char **argv)
{
  if (argc == 2 && std::string(argv[1]) == "--list")
  {
    for (const auto mode : traj_utils::formalAblationModes())
      std::cout << traj_utils::toCliString(mode) << '\n';
    return EXIT_SUCCESS;
  }
  if (argc == 2 && std::string(argv[1]) == "--matrix")
  {
    std::cout << "Mode LocalVis TeamPT LOSTopo DynBodyTopo TargetA* SeedRetain LocalGap\n";
    for (const auto mode : traj_utils::formalAblationModes())
    {
      const auto c = traj_utils::makeAblationConfig(mode);
      std::cout << traj_utils::toString(mode) << ' '
                << c.local_visibility << ' ' << c.team_pt << ' '
                << c.los_topology << ' ' << c.dynamic_body_topology << ' '
                << c.target_facing_astar << ' ' << c.safe_seed_retention << ' '
                << c.local_gap << '\n';
    }
    return EXIT_SUCCESS;
  }
  if (argc != 3)
  {
    std::cerr << "usage: ablation_config_cli --list|--matrix|--manifest MODE|--shell MODE\n";
    return EXIT_FAILURE;
  }
  try
  {
    const auto config = traj_utils::makeAblationConfig(
        traj_utils::parseAblationMode(argv[2]));
    const std::string command(argv[1]);
    if (command == "--manifest") printManifest(config);
    else if (command == "--shell") printShell(config);
    else return EXIT_FAILURE;
  }
  catch (const std::exception &error)
  {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
