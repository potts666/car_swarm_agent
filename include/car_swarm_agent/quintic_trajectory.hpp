#ifndef CAR_SWARM_AGENT_QUINTIC_TRAJECTORY_HPP
#define CAR_SWARM_AGENT_QUINTIC_TRAJECTORY_HPP

#include <array>
#include "car_swarm_agent/trajectory_tracking.hpp"

namespace car_swarm_agent {
// Coefficients use normalized local time u=t/duration, in ascending order.
// A gear segment contains equal-duration pieces; gear=0 is an explicit hold.
struct QuinticPiece
{
  double duration;
  int gear;
  double stopped_yaw;
  std::array<double, 6> x, y;
};

class QuinticTrajectory
{
public:
  QuinticTrajectory(std::vector<QuinticPiece> pieces, std_msgs::msg::Header header);
  ContinuousState evaluate(double t, bool left_limit = false) const;
  std::array<double, 2> jerk(double t) const;
  double duration() const;
  const std_msgs::msg::Header & header() const {return header_;}
  const std::vector<QuinticPiece> & pieces() const {return pieces_;}
private:
  std::vector<QuinticPiece> pieces_;
  std_msgs::msg::Header header_;
};

// Research comparison only: fine sampled plan check, not a continuous safety
// certificate. Does not authorize demo playback. Actual motion is independently
// checked by the existing continuous arc verifier.
TrackingSimulation simulateTrackingExperiment(const QuinticTrajectory & planned,
  const msg::PredictedTrajectory & peer, const PlannerConfig & geometry,
  const PlannerConfig & peer_geometry, const TrackingOptions & options = {});
}  // namespace car_swarm_agent
#endif
