#ifndef CAR_SWARM_AGENT_VEHICLE_GEOMETRY_HPP
#define CAR_SWARM_AGENT_VEHICLE_GEOMETRY_HPP

#include <cmath>
#include "car_swarm_agent/hybrid_astar_planner.hpp"

namespace car_swarm_agent {

// Same rear-axle origin and padded rectangle used by the spatial planner.
struct VehicleRectangle
{
  double center_x, center_y;
  double half_length, half_width;
  double c, s;
};

inline VehicleRectangle vehicleRectangle(
  const Pose & pose, const PlannerConfig & config, double extra_margin = 0.0)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  const double offset = config.vehicle_length * 0.5 - config.rear_overhang;
  return {pose.x + offset * c, pose.y + offset * s,
    config.vehicle_length * 0.5 + config.collision_margin + extra_margin,
    config.vehicle_width * 0.5 + config.collision_margin + extra_margin, c, s};
}

inline bool vehicleRectanglesOverlap(
  const VehicleRectangle & a, const VehicleRectangle & b)
{
  const double dx = b.center_x - a.center_x;
  const double dy = b.center_y - a.center_y;
  const auto separated = [&](double x, double y) {
      const double radius_a = a.half_length * std::abs(a.c * x + a.s * y) +
        a.half_width * std::abs(-a.s * x + a.c * y);
      const double radius_b = b.half_length * std::abs(b.c * x + b.s * y) +
        b.half_width * std::abs(-b.s * x + b.c * y);
      return std::abs(dx * x + dy * y) > radius_a + radius_b + 1e-9;
    };
  // SAT on both rectangles' longitudinal and lateral axes; contact counts.
  return !separated(a.c, a.s) && !separated(-a.s, a.c) &&
         !separated(b.c, b.s) && !separated(-b.s, b.c);
}

}  // namespace car_swarm_agent
#endif
