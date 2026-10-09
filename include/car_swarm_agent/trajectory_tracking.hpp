#ifndef CAR_SWARM_AGENT_TRAJECTORY_TRACKING_HPP
#define CAR_SWARM_AGENT_TRAJECTORY_TRACKING_HPP

#include <optional>
#include <vector>
#include "car_swarm_agent/continuous_trajectory.hpp"

namespace car_swarm_agent {

struct BicycleState
{
  Pose pose;
  double signed_speed{0};
  double steering{0};
};

struct TrackingCommand
{
  double acceleration{0}; // Signed longitudinal acceleration.
  double steering{0};     // Front wheel angle, rad.
  int gear{1};
};

struct TrackingOptions
{
  double time_step{0.01};
  double settle_seconds{2.0};
  double max_speed{2.0};
  double max_acceleration{1.0};
  double max_steering_rate{1.0};
  double max_total_acceleration{2.0};
  double speed_gain{3.0};
  double longitudinal_gain{0.5};
  double lateral_gain{0.6};
  double heading_gain{2.0};
  // Nonzero defaults make simulated motion visibly distinct from plan playback.
  double initial_lateral_offset{0.25};
  double initial_heading_offset{0.04};
  double initial_speed_scale{0.9};
};

struct BicycleStep
{
  BicycleState state;
  double signed_distance, curvature, acceleration, moving_time;
};

// A discrete kinematic bicycle: steering-rate-limited mean steering per step,
// constant acceleration and exact arc integration for that numerical step.
// Gear reversal first reaches zero; it cannot jump directly between signs.
BicycleStep stepBicycle(const BicycleState & state, const TrackingCommand & command,
  double dt, const PlannerConfig & geometry, const TrackingOptions & options);

TrackingCommand trackReference(const BicycleState & actual, const ContinuousState & reference,
  const PlannerConfig & geometry, const TrackingOptions & options);

struct TrackingError
{
  double position, longitudinal, lateral, heading, speed;
};

TrackingError trackingError(const BicycleState & actual, const ContinuousState & reference);

struct TrackingSample
{
  double time;
  BicycleState actual;
  TrackingCommand command;
  TrackingError error;
};

struct TrackingSimulation
{
  std::vector<TrackingSample> samples;
  std::optional<ContinuousTrajectory> actual_trajectory;
  std::optional<ContinuousCheckResult> actual_verification;
  double rms_position{0};
  double max_position{0};
  double max_heading{0};
  double max_speed_error{0};
  bool stopped{false};
};

inline bool allowTrackingPlayback(const TrackingSimulation & simulation)
{
  return simulation.stopped && simulation.actual_trajectory && simulation.actual_verification &&
         simulation.actual_verification->status == ContinuousCheckStatus::Safe &&
         simulation.actual_verification->static_map_checked;
}

// Offline closed-loop simulation, with feedback from the integrated model state.
// Requires a map and re-verifies the planned trajectory before simulating.
// No real vehicle commands. Actual model trajectory is verified separately.
TrackingSimulation simulateTracking(const ContinuousTrajectory & planned,
  const msg::PredictedTrajectory & peer,
  const PlannerConfig & geometry, const PlannerConfig & peer_geometry,
  const TrackingOptions & options = {});

}  // namespace car_swarm_agent
#endif
