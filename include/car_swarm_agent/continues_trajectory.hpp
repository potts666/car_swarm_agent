#ifndef CAR_SWARM_AGENT_CONTINUOUS_TRAJECTORY_HPP
#define CAR_SWARM_AGENT_CONTINUOUS_TRAJECTORY_HPP

#include <vector>
#include "car_swarm_agent/trajectory_collision.hpp"

namespace car_swarm_agent {

struct STMotionPiece
{
  double t0, duration, s0, v0, acceleration, moving_time;
};

struct ContinuousState
{
  double s;
  double speed;
  double signed_speed;
  double longitudinal_acceleration;

  Pose pose;

  double vx, vy, ax, ay;
  double yaw_rate;
};

class ContinuousTrajectory
{
public:
  ContinuousTrajectory(
    std::vector<Pose> path,
    std_msgs::msg::Header header,
    std::vector<STMotionPiece> pieces);

  // t 相对于 header.stamp，单位秒。
  ContinuousState evaluate(
    double t, bool left_limit = false) const;

  double duration() const;

  const std_msgs::msg::Header & header() const
  {
    return header_;
  }

  const std::vector<Pose> & path() const
  {
    return path_;
  }

  const std::vector<STMotionPiece> & pieces() const
  {
    return pieces_;
  }

private:
  std::vector<Pose> path_;
  std_msgs::msg::Header header_;
  std::vector<STMotionPiece> pieces_;
};

enum class ContinuousCheckStatus
{
  Safe,
  CollisionOrStaticBlocked,
  DynamicsViolation,
  InformationInsufficient,
  Unresolved
};

struct ContinuousCheckOptions
{
  double max_speed{2.0};
  double max_longitudinal_acceleration{1.0};
  double max_total_acceleration{2.0};

  double min_interval{1e-5};
  int max_depth{24};
  std::size_t max_intervals{200000};
};

struct ContinuousCheckResult
{
  ContinuousCheckStatus status{
    ContinuousCheckStatus::Unresolved};

  std::size_t intervals_checked{0};
  bool static_map_checked{false};
};

ContinuousCheckResult verifyContinuousTrajectory(
  const ContinuousTrajectory & own,
  const msg::PredictedTrajectory & peer,
  const PlannerConfig & own_geometry,
  const PlannerConfig & peer_geometry,
  const ContinuousCheckOptions & options = {});

}  // namespace car_swarm_agent

#endif