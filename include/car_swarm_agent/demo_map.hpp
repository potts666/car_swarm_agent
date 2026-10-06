#ifndef CAR_SWARM_AGENT_DEMO_MAP_HPP
#define CAR_SWARM_AGENT_DEMO_MAP_HPP

#include <cmath>
#include <stdexcept>
#include "car_swarm_agent/hybrid_astar_planner.hpp"

namespace car_swarm_agent {

// U opens toward negative x; the walls leave 23.5 m of free width.
inline void addUShapedObstacle(OccupancyGrid & map)
{
  if (map.width <= 0 || map.height <= 0 ||
      !std::isfinite(map.resolution) || map.resolution <= 0.0 ||
      map.cells.size() != static_cast<std::size_t>(map.width) *
        static_cast<std::size_t>(map.height) ||
      !std::isfinite(map.origin_x) || !std::isfinite(map.origin_y) ||
      map.origin_x > -10.0 || map.origin_y > -12.0 ||
      map.origin_x + map.width * map.resolution < 14.5 ||
      map.origin_y + map.height * map.resolution < 12.5) {
    throw std::invalid_argument("Demo map must contain the complete U-shaped obstacle");
  }

  for (int row = 0; row < map.height; ++row) {
    for (int col = 0; col < map.width; ++col) {
      const double x = map.origin_x + col * map.resolution;
      const double y = map.origin_y + row * map.resolution;
      // Occupy every cell overlapping a wall, including on coarser maps.
      const bool side = x + map.resolution > -10.0 && x < 14.5 &&
        ((y + map.resolution > -12.0 && y < -11.5) ||
        (y + map.resolution > 12.0 && y < 12.5));
      const bool back = x + map.resolution > 14.0 && x < 14.5 &&
        y + map.resolution > -12.0 && y < 12.5;
      if (side || back) {
        map.cells[static_cast<std::size_t>(row) * map.width + col] = 100;
      }
    }
  }
}

}  // namespace car_swarm_agent

#endif
