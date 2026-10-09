#include "car_swarm_agent/trajectory_tracking.hpp"
#include "car_swarm_agent/demo_map.hpp"
#include "car_swarm_agent/quintic_trajectory.hpp"
#include "car_swarm_agent/vehicle_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace car_swarm_agent {
namespace {
constexpr double kTwoPi = 6.28318530717958647692;

void validate(const PlannerConfig & g, const TrackingOptions & o)
{
  for (double x : {g.wheel_base, g.max_steer_angle, o.time_step, o.settle_seconds,
      o.max_speed, o.max_acceleration, o.max_steering_rate, o.max_total_acceleration,
      o.speed_gain, o.longitudinal_gain, o.lateral_gain, o.heading_gain,
      o.initial_lateral_offset, o.initial_heading_offset, o.initial_speed_scale})
  {
    if (!std::isfinite(x)) {throw std::invalid_argument("Nonfinite tracking settings");}
  }
  if (g.wheel_base <= 0 || g.max_steer_angle <= 0 || g.max_steer_angle >= 1.5 ||
    o.time_step <= 0 || o.time_step > 0.1 || o.settle_seconds < 0 ||
    o.max_speed <= 0 || o.max_acceleration <= 0 || o.max_steering_rate <= 0 ||
    o.max_total_acceleration <= 0 || o.speed_gain <= 0 || o.longitudinal_gain < 0 ||
    o.lateral_gain < 0 || o.heading_gain < 0 || o.initial_speed_scale < 0)
  {
    throw std::invalid_argument("Invalid tracking settings");
  }
}

Pose integrate(const Pose & p, double d, double k)
{
  if (std::abs(k) < 1e-12) {
    return {p.x + d * std::cos(p.yaw), p.y + d * std::sin(p.yaw), p.yaw, d, k};
  }
  const double yaw = p.yaw + k * d;
  return {p.x + (std::sin(yaw) - std::sin(p.yaw)) / k,
    p.y + (std::cos(p.yaw) - std::cos(yaw)) / k,
    std::remainder(yaw, kTwoPi), d, k};
}
}  // namespace

BicycleStep stepBicycle(const BicycleState & state, const TrackingCommand & command,
  double dt, const PlannerConfig & g, const TrackingOptions & o)
{
  validate(g, o);
  if (!std::isfinite(dt) || dt <= 0 || dt > 0.1 ||
    !std::isfinite(state.pose.x) || !std::isfinite(state.pose.y) ||
    !std::isfinite(state.pose.yaw) || !std::isfinite(state.signed_speed) ||
    !std::isfinite(state.steering) || !std::isfinite(command.acceleration) ||
    !std::isfinite(command.steering) || (command.gear != 1 && command.gear != -1) ||
    std::abs(state.signed_speed) > o.max_speed + 1e-8 ||
    std::abs(state.steering) > g.max_steer_angle + 1e-8)
  {
    throw std::invalid_argument("Invalid bicycle state/command/step");
  }
  const double requested = std::clamp(command.steering, -g.max_steer_angle, g.max_steer_angle);
  const double steering = state.steering + std::clamp(requested - state.steering,
    -o.max_steering_rate * dt, o.max_steering_rate * dt);
  const double k = std::tan(0.5 * (state.steering + steering)) / g.wheel_base;
  const double v0 = state.signed_speed;
  double a = std::clamp(command.acceleration, -o.max_acceleration, o.max_acceleration);
  if (v0 * command.gear < -1e-8) {
    a = -std::copysign(o.max_acceleration, v0);
  }
  if (v0 == 0 && a * command.gear < 0) {a = 0;}
  double moving = dt;
  double v1 = v0 + a * dt;
  if (v0 * v1 < 0) {
    moving = -v0 / a;
    v1 = 0; // Stay stopped for the remainder; reversal requires another step.
  } else {
    v1 = std::clamp(v1, -o.max_speed, o.max_speed);
    a = (v1 - v0) / dt;
  }
  const double d = v0 * moving + 0.5 * a * moving * moving;
  return {{integrate(state.pose, d, k), v1, steering}, d, k, a, moving};
}

TrackingCommand trackReference(const BicycleState & actual, const ContinuousState & ref,
  const PlannerConfig & g, const TrackingOptions & o)
{
  validate(g, o);
  const double dx = ref.pose.x - actual.pose.x;
  const double dy = ref.pose.y - actual.pose.y;
  const double c = std::cos(actual.pose.yaw), s = std::sin(actual.pose.yaw);
  const double longitudinal = c * dx + s * dy;
  const double lateral = -s * dx + c * dy;
  const double heading = std::remainder(ref.pose.yaw - actual.pose.yaw, kTwoPi);
  const double desired = ref.speed < 1e-8 ? 0.0 : ref.gear * std::clamp(
    ref.speed + ref.gear * o.longitudinal_gain * longitudinal, 0.0, o.max_speed);
  double a = ref.longitudinal_acceleration + o.speed_gain * (desired - actual.signed_speed);
  // Waiting/terminal stop is an explicit brake request, not a positional teleport.
  if (ref.speed < 1e-8 && std::abs(ref.longitudinal_acceleration) < 1e-8 &&
    std::abs(actual.signed_speed) > 0)
  {
    a = -std::copysign(o.max_acceleration, actual.signed_speed);
  }
  const double steering = std::atan(g.wheel_base * ref.curvature) +
    o.lateral_gain * lateral + ref.gear * o.heading_gain * heading;
  return {std::clamp(a, -o.max_acceleration, o.max_acceleration),
    std::clamp(steering, -g.max_steer_angle, g.max_steer_angle), ref.gear};
}

TrackingError trackingError(const BicycleState & actual, const ContinuousState & ref)
{
  const double dx = actual.pose.x - ref.pose.x, dy = actual.pose.y - ref.pose.y;
  const double c = std::cos(ref.pose.yaw), s = std::sin(ref.pose.yaw);
  return {std::hypot(dx, dy), c * dx + s * dy, -s * dx + c * dy,
    std::remainder(actual.pose.yaw - ref.pose.yaw, kTwoPi),
    actual.signed_speed - ref.signed_speed};
}

template<class Reference>
TrackingSimulation integrateTracking(const Reference & planned,
  const msg::PredictedTrajectory & peer, const PlannerConfig & g,
  const PlannerConfig & peer_g, const TrackingOptions & o)
{
  ContinuousCheckOptions check;
  check.max_speed = o.max_speed;
  check.max_longitudinal_acceleration = o.max_acceleration;
  check.max_total_acceleration = o.max_total_acceleration;
  const double end = planned.duration() + o.settle_seconds;
  if (end / o.time_step > 200000) {
    throw std::invalid_argument("Tracking simulation step budget exceeded");
  }
  const auto start = planned.evaluate(0);
  BicycleState actual{start.pose, start.signed_speed * o.initial_speed_scale, 0};
  actual.pose.x -= std::sin(start.pose.yaw) * o.initial_lateral_offset;
  actual.pose.y += std::cos(start.pose.yaw) * o.initial_lateral_offset;
  actual.pose.yaw = std::remainder(actual.pose.yaw + o.initial_heading_offset, kTwoPi);
  if (std::abs(actual.signed_speed) > o.max_speed) {
    throw std::invalid_argument("Initial model speed exceeds limit");
  }
  std::vector<Pose> actual_path{actual.pose};
  std::vector<STMotionPiece> actual_pieces;
  TrackingSimulation result;
  double squared_error = 0, t = 0, s = 0;
  const auto append = [&](double time, const TrackingCommand & command) {
      const auto error = trackingError(actual, planned.evaluate(std::min(time, planned.duration())));
      result.samples.push_back({time, actual, command, error});
      squared_error += error.position * error.position;
      result.max_position = std::max(result.max_position, error.position);
      result.max_heading = std::max(result.max_heading, std::abs(error.heading));
      result.max_speed_error = std::max(result.max_speed_error, std::abs(error.speed));
    };
  append(0, {});
  while (t < end) {
    const double dt = std::min(o.time_step, end - t);
    const auto ref = planned.evaluate(std::min(t, planned.duration()));
    auto command = trackReference(actual, ref, g, o);
    const auto step = stepBicycle(actual, command, dt, g, o);
    const int gear = actual.signed_speed != 0 ? (actual.signed_speed > 0 ? 1 : -1) :
      (step.signed_distance != 0 ? (step.signed_distance > 0 ? 1 : -1) : command.gear);
    actual_pieces.push_back({t, dt, s, std::abs(actual.signed_speed),
      gear * step.acceleration, step.moving_time});
    s += std::abs(step.signed_distance);
    if (step.signed_distance != 0) {actual_path.push_back(step.state.pose);}
    actual = step.state;
    command.acceleration = step.acceleration;
    t += dt;
    append(t, command);
  }
  result.rms_position = std::sqrt(squared_error / result.samples.size());
  result.stopped = std::abs(actual.signed_speed) < 1e-8;
  if (result.stopped && actual_path.size() >= 2) {
    result.actual_trajectory.emplace(std::move(actual_path), planned.header(), std::move(actual_pieces));
    result.actual_verification = verifyContinuousTrajectory(
      *result.actual_trajectory, peer, g, peer_g, check);
  }
  return result;
}

TrackingSimulation simulateTracking(const ContinuousTrajectory & planned,
  const msg::PredictedTrajectory & peer, const PlannerConfig & g,
  const PlannerConfig & peer_g, const TrackingOptions & o)
{
  validate(g, o);
  if (!hasStaticMap(g.map)) {
    throw std::invalid_argument("Tracking simulation: 静态碰撞未验证; map required");
  }
  ContinuousCheckOptions check;
  check.max_speed = o.max_speed;
  check.max_longitudinal_acceleration = o.max_acceleration;
  check.max_total_acceleration = o.max_total_acceleration;
  const auto verified = verifyContinuousTrajectory(planned, peer, g, peer_g, check);
  if (verified.status != ContinuousCheckStatus::Safe || !verified.static_map_checked) {
    throw std::invalid_argument("Tracking requires a fully verified planned trajectory");
  }
  return integrateTracking(planned, peer, g, peer_g, o);
}

TrackingSimulation simulateTrackingExperiment(const QuinticTrajectory & planned,
  const msg::PredictedTrajectory & peer, const PlannerConfig & g,
  const PlannerConfig & peer_g, const TrackingOptions & o)
{
  validate(g, o);
  if (!hasStaticMap(g.map) || planned.header().frame_id != peer.header.frame_id) {
    throw std::invalid_argument("Polynomial experiment needs map and common frame");
  }
  HybridAStarPlanner planner(g);
  const auto origin = static_cast<int64_t>(planned.header().stamp.sec) * 1000000000LL +
    planned.header().stamp.nanosec;
  // Include both limits of every knot as well as a finer independent grid.
  std::vector<double> times{0, planned.duration()};
  for (double t = 0; t < planned.duration(); t += 0.01) {times.push_back(t);}
  double knot = 0;
  for (const auto & p : planned.pieces()) {
    knot += p.duration; times.push_back(knot);
    times.push_back(std::max(0.0, knot - 1e-5));
    if (knot < planned.duration()) {times.push_back(knot + 1e-5);}
  }
  for (double t : times) {
    const auto ref = planned.evaluate(t);
    const auto other = interpolateTrajectoryPose(peer, origin + std::llround(t * 1e9));
    if (!other || !planner.isPoseCollisionFree(ref.pose) ||
      vehicleRectanglesOverlap(vehicleRectangle(ref.pose, g), vehicleRectangle(*other, peer_g)) ||
      ref.speed > o.max_speed + 1e-5 ||
      std::abs(ref.longitudinal_acceleration) > o.max_acceleration + 1e-5 ||
      std::hypot(ref.ax, ref.ay) > o.max_total_acceleration + 1e-5 ||
      std::abs(std::atan(g.wheel_base * ref.curvature)) > g.max_steer_angle + 1e-5)
    {
      throw std::invalid_argument("Polynomial failed independent fine sampled check at t=" +
        std::to_string(t) + ", speed=" + std::to_string(ref.speed) +
        ", steering=" + std::to_string(std::atan(g.wheel_base * ref.curvature)) +
        ", acceleration=" + std::to_string(ref.longitudinal_acceleration));
    }
  }
  return integrateTracking(planned, peer, g, peer_g, o);
}

}  // namespace car_swarm_agent
