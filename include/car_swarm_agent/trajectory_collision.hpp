#ifndef CAR_SWARM_AGENT_TRAJECTORY_COLLISION_HPP
#define CAR_SWARM_AGENT_TRAJECTORY_COLLISION_HPP

#include <cstdint>
#include <optional>
#include "car_swarm_agent/hybrid_astar_planner.hpp"
#include "car_swarm_agent/msg/predicted_trajectory.hpp"

namespace car_swarm_agent {

enum class CollisionCheckStatus { NoConflictAtSamples, Conflict, NoCommonTime, Expired };

struct CollisionCheckOptions
{
  std::int64_t sample_step_ns{100000000};  // 0.1 s
  // Optional caller-supplied clock time: never infer freshness from wall time.
  std::optional<std::int64_t> not_before_ns;
};

struct TrajectoryConflict
{
  std::int64_t absolute_time_ns;
  Pose first_pose;   // Rear-axle location and heading, not a contact point.
  Pose second_pose;
};

struct CollisionCheckResult
{
  CollisionCheckStatus status{CollisionCheckStatus::NoCommonTime};
  std::optional<TrajectoryConflict> first_conflict;
  std::optional<std::int64_t> checked_start_ns;
  std::optional<std::int64_t> checked_end_ns;
  std::size_t samples_checked{0};
};

// Same interpolation as the detector. Empty/out-of-range trajectories return
// nullopt; malformed points throw std::invalid_argument. No extrapolation.
std::optional<Pose> interpolateTrajectoryPose(
  const msg::PredictedTrajectory & trajectory, std::int64_t absolute_time_ns);

// Pure sampled check: header.stamp + time_from_start, no callbacks or mutation.
// Linear x/y and shortest-angle yaw interpolation; no extrapolation.
// Requires equal nonempty frame IDs and finite planar poses with strictly
// increasing nonnegative offsets. Invalid inputs throw std::invalid_argument.
// Both endpoints are sampled. No continuous-time collision guarantee.
CollisionCheckResult detectTrajectoryCollision(
  const msg::PredictedTrajectory & first,
  const msg::PredictedTrajectory & second,
  const PlannerConfig & first_geometry = {},
  const PlannerConfig & second_geometry = {},
  const CollisionCheckOptions & options = {});

}  // namespace car_swarm_agent
#endif
