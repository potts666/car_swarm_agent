#ifndef CAR_SWARM_AGENT_REEDS_SHEPP_HPP
#define CAR_SWARM_AGENT_REEDS_SHEPP_HPP

#include <limits>
#include "car_swarm_agent/hybrid_astar_planner.hpp"

namespace car_swarm_agent::detail {
// Geometry adapted from OMPL 1.6.0; BSD notice retained in reeds_shepp.cpp.
class ReedsSheppStateSpace {
public:
    enum ReedsSheppPathSegmentType { RS_NOP, RS_LEFT, RS_STRAIGHT, RS_RIGHT };
    static const ReedsSheppPathSegmentType reedsSheppPathType[18][5];
    struct ReedsSheppPath {
        explicit ReedsSheppPath(const ReedsSheppPathSegmentType * type = reedsSheppPathType[0],
            double t = std::numeric_limits<double>::infinity(), double u = 0,
            double v = 0, double w = 0, double x = 0);
        double length() const { return totalLength_; }
        const ReedsSheppPathSegmentType * type_;
        double length_[5]; // Signed lengths in units of turning radius.
        double totalLength_;
    };
};
ReedsSheppStateSpace::ReedsSheppPath shortestReedsShepp(
    const Pose & from, const Pose & to, double turning_radius);
Pose integrateArc(const Pose & from, double signed_distance, double curvature);
}  // namespace car_swarm_agent::detail
#endif
