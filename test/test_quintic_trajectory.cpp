#include <gtest/gtest.h>
#include "car_swarm_agent/quintic_trajectory.hpp"
#include "car_swarm_agent/demo_map.hpp"
#include <cmath>

using namespace car_swarm_agent;
namespace {
std_msgs::msg::Header header()
{
  std_msgs::msg::Header h; h.frame_id = "map"; h.stamp.sec = 100; return h;
}
QuinticPiece forward()
{
  return {4, 1, 0, {0, 0, 0, 10, -15, 6}, {0, 0, 0, 0, 0, 0}};
}
TEST(QuinticTrajectory, AnalyticDerivativesAndTerminalRightLimit)
{
  QuinticTrajectory plan({forward()}, header());
  const auto middle = plan.evaluate(2);
  EXPECT_NEAR(middle.pose.x, 0.5, 1e-12);
  EXPECT_NEAR(middle.speed, 1.875 / 4, 1e-12);
  EXPECT_NEAR(middle.ax, 0, 1e-12);
  EXPECT_NEAR(plan.jerk(2)[0], -30.0 / 64, 1e-12);
  EXPECT_NEAR(plan.evaluate(4).speed, 0, 1e-12);
  EXPECT_NEAR(plan.evaluate(4).longitudinal_acceleration, 0, 1e-12);
  EXPECT_THROW(plan.evaluate(4.1), std::out_of_range);
}
TEST(QuinticTrajectory, ReversalKeepsCarHeadingAndZeroSpeedBoundary)
{
  auto reverse = forward(); reverse.gear = -1;
  reverse.x = {1, 0, 0, -10, 15, -6};
  QuinticTrajectory plan({forward(), reverse}, header());
  EXPECT_NEAR(plan.evaluate(4).speed, 0, 1e-12);
  EXPECT_EQ(plan.evaluate(4).gear, -1);
  EXPECT_EQ(plan.evaluate(4, true).gear, 1);
  EXPECT_LT(plan.evaluate(6).signed_speed, 0);
  EXPECT_NEAR(plan.evaluate(6).pose.yaw, 0, 1e-12);
}
TEST(QuinticTrajectory, RejectsNonzeroGearChangeAndPositionJump)
{
  auto next = forward(); next.x[0] = 1.1;
  EXPECT_THROW(QuinticTrajectory({forward(), next}, header()), std::invalid_argument);
  next.x = {1, -1, 0, 0, 0, 0}; next.gear = -1;
  EXPECT_THROW(QuinticTrajectory({forward(), next}, header()), std::invalid_argument);
  next = forward(); next.gear = 0;
  EXPECT_THROW(QuinticTrajectory({next}, header()), std::invalid_argument);
  next.x = {1, 0, 0, 0, 0, 0}; next.y = {}; next.stopped_yaw = 1.5707963267948966;
  EXPECT_THROW(QuinticTrajectory({forward(), next}, header()), std::invalid_argument);
}
TEST(QuinticTrajectory, ComputesFiniteSteeringLimitAtStoppedCurvedStart)
{
  // Tangent acceleration/jerk, with lateral snap: curvature tends to 0.05.
  QuinticPiece first{2, 1, 0, {0, 0, 0.2, 0, 0.01, 0}, {0, 0, 0, 0, 0.001, 0}};
  QuinticPiece second{2, 1, 0, {0.21, 0.44, 0.26, 4.48, -7.55, 3.16},
    {0.001, 0.004, 0.006, 0.048, -0.085, 0.036}};
  QuinticTrajectory plan({first, second}, header());
  EXPECT_NEAR(plan.evaluate(0).curvature, 0.05, 1e-12);
  EXPECT_NEAR(plan.evaluate(1e-6).curvature, 0.05, 1e-8);
}
TEST(QuinticTrajectory, ExperimentUsesSameModelAndChecksPeerCoverage)
{
  PlannerConfig g; g.map = makeUShapedDemoMap();
  msg::PredictedTrajectory peer; peer.header = header(); peer.vehicle_id = "peer";
  msg::TrajectoryPoint a, b;
  a.pose.position.x = b.pose.position.x = -24;
  a.pose.position.y = b.pose.position.y = -24;
  a.pose.orientation.w = b.pose.orientation.w = 1;
  b.time_from_start.sec = 30; peer.points = {a, b};
  auto piece = forward(); piece.duration = 16;
  for (auto & c : piece.x) {c *= 8;}
  QuinticTrajectory plan({piece}, header());
  const auto simulation = simulateTrackingExperiment(plan, peer, g, g);
  ASSERT_FALSE(simulation.samples.empty());
  EXPECT_NEAR(simulation.samples.front().error.lateral, 0.25, 1e-12);
  EXPECT_LT(simulation.samples.back().error.position, 0.15);
  EXPECT_TRUE(simulation.stopped);
  ASSERT_TRUE(simulation.actual_verification);
  // Numerical experiment still retains the independent actual certificate.
  // The fixed-path verifier can reject very small final steps near its stop tolerance.
  EXPECT_EQ(allowTrackingPlayback(simulation),
    simulation.actual_verification->status == ContinuousCheckStatus::Safe &&
    simulation.actual_verification->static_map_checked && simulation.stopped);
  peer.points.back().time_from_start.sec = 2;
  EXPECT_THROW(simulateTrackingExperiment(plan, peer, g, g), std::invalid_argument);
  g.map = {};
  EXPECT_THROW(simulateTrackingExperiment(plan, peer, g, g), std::invalid_argument);
}
}  // namespace
