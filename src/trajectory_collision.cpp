#include "car_swarm_agent/trajectory_collision.hpp"
#include "car_swarm_agent/vehicle_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace car_swarm_agent {
namespace {
constexpr std::int64_t kSecond = 1000000000LL;
constexpr double kTwoPi = 6.28318530717958647692;

template<typename Time>
std::int64_t nanoseconds(const Time & time)
{
  if (time.nanosec >= kSecond) {
    throw std::invalid_argument("Time nanosec must be less than 1e9");
  }
  return static_cast<std::int64_t>(time.sec) * kSecond + time.nanosec;
}

void validateGeometry(const PlannerConfig & config)
{
  if (!std::isfinite(config.vehicle_length) || config.vehicle_length <= 0.0 ||
    !std::isfinite(config.vehicle_width) || config.vehicle_width <= 0.0 ||
    !std::isfinite(config.rear_overhang) || config.rear_overhang < 0.0 ||
    config.rear_overhang > config.vehicle_length ||
    !std::isfinite(config.collision_margin) || config.collision_margin < 0.0)
  {
    throw std::invalid_argument("Invalid vehicle geometry");
  }
}

struct TimedPose { std::int64_t time; Pose pose; };

std::vector<TimedPose> timedPoses(const msg::PredictedTrajectory & trajectory)
{
  const auto stamp = nanoseconds(trajectory.header.stamp);
  std::vector<TimedPose> result;
  result.reserve(trajectory.points.size());
  for (const auto & point : trajectory.points) {
    const auto offset = nanoseconds(point.time_from_start);
    const auto & p = point.pose.position;
    const auto & q = point.pose.orientation;
    const double norm = std::hypot(q.z, q.w);
    if (offset < 0 || (!result.empty() && stamp + offset <= result.back().time) ||
      !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(q.x) || !std::isfinite(q.y) ||
      !std::isfinite(q.z) || !std::isfinite(q.w) ||
      std::abs(q.x) > 1e-9 || std::abs(q.y) > 1e-9 ||
      !std::isfinite(norm) || norm < 1e-12)
    {
      throw std::invalid_argument("Expected increasing offsets and finite planar poses");
    }
    // Normalize z/w; reverse motion does not flip the vehicle heading.
    const double z = q.z / norm;
    const double w = q.w / norm;
    result.push_back({stamp + offset,
      {p.x, p.y, std::atan2(2.0 * z * w, w * w - z * z)}});
  }
  return result;
}

Pose interpolate(const std::vector<TimedPose> & points, std::int64_t time,
  std::size_t & segment)
{
  while (segment + 1 < points.size() && points[segment + 1].time <= time) {
    ++segment;
  }
  const auto & a = points[segment];
  if (a.time == time || segment + 1 == points.size()) {
    return a.pose;
  }
  const auto & b = points[segment + 1];
  const double fraction = static_cast<double>(time - a.time) / (b.time - a.time);
  return {a.pose.x + fraction * (b.pose.x - a.pose.x),
    a.pose.y + fraction * (b.pose.y - a.pose.y),
    std::remainder(a.pose.yaw + fraction *
      std::remainder(b.pose.yaw - a.pose.yaw, kTwoPi), kTwoPi)};
}
}  // namespace

std::optional<Pose> interpolateTrajectoryPose(
  const msg::PredictedTrajectory & trajectory, std::int64_t absolute_time_ns)
{
  const auto points = timedPoses(trajectory);
  if (points.empty() || absolute_time_ns < points.front().time ||
    absolute_time_ns > points.back().time)
  {
    return std::nullopt;
  }
  std::size_t segment = 0;
  return interpolate(points, absolute_time_ns, segment);
}

CollisionCheckResult detectTrajectoryCollision(
  const msg::PredictedTrajectory & first, const msg::PredictedTrajectory & second,
  const PlannerConfig & first_geometry, const PlannerConfig & second_geometry,
  const CollisionCheckOptions & options)
{
  if (options.sample_step_ns <= 0) {
    throw std::invalid_argument("Sample step must be positive");
  }
  if (first.header.frame_id.empty() || first.header.frame_id != second.header.frame_id) {
    throw std::invalid_argument("Trajectories must share a nonempty coordinate frame");
  }
  validateGeometry(first_geometry);
  validateGeometry(second_geometry);
  const auto a = timedPoses(first);
  const auto b = timedPoses(second);
  CollisionCheckResult result;
  if (a.empty() || b.empty()) {return result;}
  auto start = std::max(a.front().time, b.front().time);
  const auto end = std::min(a.back().time, b.back().time);
  if (start > end) {return result;}
  if (options.not_before_ns) {
    start = std::max(start, *options.not_before_ns);
    if (start > end) {
      result.status = CollisionCheckStatus::Expired;
      return result;
    }
  }
  result.checked_start_ns = start;
  result.checked_end_ns = end;
  result.status = CollisionCheckStatus::NoConflictAtSamples;
  std::size_t segment_a = 0, segment_b = 0;
  for (auto time = start; ; ) {
    const auto pose_a = interpolate(a, time, segment_a);
    const auto pose_b = interpolate(b, time, segment_b);
    ++result.samples_checked;
    if (vehicleRectanglesOverlap(vehicleRectangle(pose_a, first_geometry),
      vehicleRectangle(pose_b, second_geometry)))
    {
      result.status = CollisionCheckStatus::Conflict;
      result.first_conflict = TrajectoryConflict{time, pose_a, pose_b};
      return result;
    }
    if (time == end) {break;}
    time += std::min(options.sample_step_ns, end - time);
  }
  return result;
}
}  // namespace car_swarm_agent
