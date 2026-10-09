#include <gtest/gtest.h>
#include <cmath>
#include <stdexcept>
#include "car_swarm_agent/trajectory_tracking.hpp"
#include "car_swarm_agent/demo_map.hpp"

using namespace car_swarm_agent;
namespace {
std_msgs::msg::Header header()
{
  std_msgs::msg::Header h; h.frame_id = "map"; h.stamp.sec = 100; return h;
}

PlannerConfig geometry()
{
  PlannerConfig g; g.map = makeUShapedDemoMap(); return g;
}

msg::PredictedTrajectory peer()
{
  msg::PredictedTrajectory p; p.header = header(); p.vehicle_id = "other";
  msg::TrajectoryPoint a, b;
  a.pose.position.x = b.pose.position.x = -24;
  a.pose.position.y = b.pose.position.y = -24;
  a.pose.orientation.w = b.pose.orientation.w = 1;
  b.time_from_start.sec = 300;
  p.points = {a, b}; return p;
}

ContinuousTrajectory straight()
{
  return ContinuousTrajectory({{0, 0, 0}, {8, 0, 0, 8, 0}}, header(), {
    {0, 7.5, 0, 1, 0, 7.5}, {7.5, 1, 7.5, 1, -1, 1}});
}

TEST(BicycleModel, IntegratesCommandWithoutCopyingReferencePose)
{
  TrackingOptions o;
  BicycleState state{{0, 0, 0}, 1, 0};
  const auto step = stepBicycle(state, {1, 0, 1}, 0.1, geometry(), o);
  EXPECT_NEAR(step.state.pose.x, 0.105, 1e-12);
  EXPECT_NEAR(step.state.signed_speed, 1.1, 1e-12);
  EXPECT_NEAR(step.state.pose.y, 0, 1e-12);
}

TEST(BicycleModel, SteeringAndAccelerationAreBounded)
{
  const auto g = geometry(); TrackingOptions o;
  const auto step = stepBicycle({{0, 0, 0}, 1, 0}, {100, 100, 1}, 0.01, g, o);
  EXPECT_LE(std::abs(step.acceleration), o.max_acceleration + 1e-10);
  EXPECT_LE(std::abs(step.state.steering), g.max_steer_angle);
  EXPECT_LE(std::abs(step.state.steering), o.max_steering_rate * 0.01 + 1e-10);
  EXPECT_LE(std::abs(step.state.signed_speed), o.max_speed);
}

TEST(BicycleModel, ReverseCommandFirstBrakesToZero)
{
  TrackingOptions o;
  const auto g = geometry();
  BicycleState state{{0, 0, 0}, 0.05, 0};
  const auto stopped = stepBicycle(state, {-1, 0, -1}, 0.1, g, o);
  EXPECT_DOUBLE_EQ(stopped.state.signed_speed, 0);
  EXPECT_NEAR(stopped.signed_distance, 0.00125, 1e-12);
  const auto reversed = stepBicycle(stopped.state, {-1, 0, -1}, 0.1, g, o);
  EXPECT_LT(reversed.state.signed_speed, 0);
  EXPECT_LT(reversed.signed_distance, 0);
}

TEST(TrackingController, FeedbackReducesStraightPathErrorsAndStops)
{
  const auto simulation = simulateTracking(straight(), peer(), geometry(), geometry());
  ASSERT_GT(simulation.samples.size(), 100u);
  EXPECT_NEAR(simulation.samples.front().error.lateral, 0.25, 1e-12);
  EXPECT_NEAR(simulation.samples.front().error.heading, 0.04, 1e-12);
  EXPECT_NEAR(simulation.samples.front().error.speed, -0.1, 1e-12);
  EXPECT_LT(std::abs(simulation.samples.back().error.lateral), 0.03);
  EXPECT_LT(std::abs(simulation.samples.back().error.heading), 0.02);
  EXPECT_LT(simulation.samples.back().error.position, 0.15);
  EXPECT_TRUE(simulation.stopped);
  EXPECT_TRUE(allowTrackingPlayback(simulation));
  ASSERT_TRUE(simulation.actual_verification);
  EXPECT_EQ(simulation.actual_verification->status, ContinuousCheckStatus::Safe)
    << simulation.actual_verification->detail;
  EXPECT_GT(simulation.rms_position, 0.01); // Actual state is not copied from reference.
}

TEST(TrackingController, CurvedReferenceUsesSteeringAndRemainsVerified)
{
  constexpr double pi = 3.14159265358979323846;
  const double length = 4 * pi;
  ContinuousTrajectory curve({{0, 0, 0}, {8, 8, pi / 2, length, 0.125}}, header(), {
    {0, length - 0.5, 0, 1, 0, length - 0.5},
    {length - 0.5, 1, length - 0.5, 1, -1, 1}});
  const auto simulation = simulateTracking(curve, peer(), geometry(), geometry());
  ASSERT_TRUE(simulation.actual_verification);
  EXPECT_EQ(simulation.actual_verification->status, ContinuousCheckStatus::Safe)
    << simulation.actual_verification->detail;
  EXPECT_LT(simulation.max_position, 0.5);
  EXPECT_LT(simulation.samples.back().error.position, 0.2);
  bool steering_used = false;
  for (const auto & sample : simulation.samples) {
    if (std::abs(sample.actual.steering) > 0.1) {steering_used = true;}
    EXPECT_LE(std::abs(sample.actual.signed_speed), 2.0 + 1e-8);
    EXPECT_LE(std::abs(sample.actual.steering), 0.5 + 1e-8);
  }
  EXPECT_TRUE(steering_used);
}

TEST(TrackingController, ReferenceGearChangeHasActualZeroSpeedBeforeReverse)
{
  ContinuousTrajectory ref({{0, 0, 0}, {1, 0, 0, 1, 0}, {0, 0, 0, -1, 0}}, header(), {
    {0, 1, 0, 0, 1, 1}, {1, 1, 0.5, 1, -1, 1},
    {2, 1, 1, 0, 1, 1}, {3, 1, 1.5, 1, -1, 1}});
  TrackingOptions o; o.initial_lateral_offset = 0.1;
  const auto simulation = simulateTracking(ref, peer(), geometry(), geometry(), o);
  bool forward = false, zero_after_forward = false, reverse = false;
  for (const auto & sample : simulation.samples) {
    if (sample.actual.signed_speed > 1e-8) {forward = true;}
    if (forward && sample.actual.signed_speed == 0) {zero_after_forward = true;}
    if (sample.actual.signed_speed < -1e-8) {
      EXPECT_TRUE(zero_after_forward);
      reverse = true;
    }
  }
  EXPECT_TRUE(forward); EXPECT_TRUE(reverse); EXPECT_TRUE(simulation.stopped);
  ASSERT_TRUE(simulation.actual_verification);
  EXPECT_EQ(simulation.actual_verification->status, ContinuousCheckStatus::Safe)
    << simulation.actual_verification->detail;
}

TEST(TrackingController, MissingMapAndBlockedPlanCannotStartSimulation)
{
  auto g = geometry(); g.map = {};
  EXPECT_THROW(simulateTracking(straight(), peer(), g, geometry()), std::invalid_argument);
  ContinuousTrajectory blocked({{0, 0, 0}, {24, 0, 0, 24, 0}}, header(), {
    {0, 23.5, 0, 1, 0, 23.5}, {23.5, 1, 23.5, 1, -1, 1}});
  EXPECT_THROW(simulateTracking(blocked, peer(), geometry(), geometry()), std::invalid_argument);
}

TEST(TrackingController, DisturbedActualMotionDoesNotInheritPlannedCertificate)
{
  TrackingOptions o; o.initial_lateral_offset = 12;
  const auto simulation = simulateTracking(straight(), peer(), geometry(), geometry(), o);
  ASSERT_TRUE(simulation.actual_verification);
  EXPECT_NE(simulation.actual_verification->status, ContinuousCheckStatus::Safe);
  EXPECT_FALSE(allowTrackingPlayback(simulation));
}
}  // namespace
