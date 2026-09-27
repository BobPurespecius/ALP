#include <plan_env/static_los_geometry.h>

#include <Eigen/Core>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace
{
void require(const bool condition, const char *message)
{
  if (!condition)
  {
    std::cerr << "FAIL: " << message << std::endl;
    std::exit(1);
  }
}
} // namespace

int main(int argc, char **argv)
{
  if (argc >= 2)
  {
    ego_planner::StaticLosGeometry scene_geometry;
    std::string reason;
    require(scene_geometry.loadScene(argv[1], reason),
            "generated B scene must load into authoritative static LOS");
    require(scene_geometry.cylinderSize() == 11,
            "generated B scene cylinder count");
    require(scene_geometry.wallSize() == 8,
            "generated B scene wall count");
  }

  ego_planner::StaticLosWall wall;
  wall.name = "contract_wall";
  wall.center = Eigen::Vector2d::Zero();
  wall.half_size = Eigen::Vector2d(1.0, 0.1);
  wall.yaw = 0.0;
  wall.z_min = 0.0;
  wall.z_max = 5.0;

  ego_planner::StaticLosGeometry geometry;
  geometry.setWalls(std::vector<ego_planner::StaticLosWall>{wall});
  require(geometry.valid(), "wall-only static LOS geometry must be valid");
  require(geometry.cylinderSize() == 0 && geometry.wallSize() == 1,
          "wall-only primitive counts");

  double clearance = 0.0;
  require(geometry.querySegmentClearance(Eigen::Vector3d(0.0, -1.0, 1.5),
                                         Eigen::Vector3d(0.0, 1.0, 1.5),
                                         clearance),
          "crossing query must be valid");
  require(std::abs(clearance) < 1.0e-9,
          "crossing segment must have zero clearance");

  Eigen::Vector3d gradient_observer;
  Eigen::Vector3d gradient_target;
  require(geometry.querySegmentClearance(Eigen::Vector3d(-2.0, 0.5, 1.5),
                                         Eigen::Vector3d(2.0, 0.5, 1.5),
                                         clearance,
                                         &gradient_observer,
                                         &gradient_target),
          "parallel query must be valid");
  require(std::abs(clearance - 0.4) < 1.0e-8,
          "parallel segment clearance must match wall thickness");
  require(gradient_observer.allFinite() && gradient_target.allFinite(),
          "wall gradients must remain finite");

  wall.yaw = 0.5 * M_PI;
  geometry.setWalls(std::vector<ego_planner::StaticLosWall>{wall});
  require(geometry.querySegmentClearance(Eigen::Vector3d(-1.0, 0.0, 1.5),
                                         Eigen::Vector3d(1.0, 0.0, 1.5),
                                         clearance),
          "rotated query must be valid");
  require(std::abs(clearance) < 1.0e-9,
          "rotated wall must block in world coordinates");

  std::cout << "static LOS wall contract: PASS" << std::endl;
  return 0;
}
