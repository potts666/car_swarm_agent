#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <stdexcept>
#include "car_swarm_agent/trajectory_collision.hpp"

namespace {
using namespace car_swarm_agent;
constexpr std::int64_t kSecond = 1000000000LL;
constexpr double kPi = 3.14159265358979323846;
using Trajectory = msg::PredictedTrajectory;

msg::TrajectoryPoint point(double seconds, double x, double y, double yaw = 0.0,
  int direction = msg::TrajectoryPoint::FORWARD)
{
  msg::TrajectoryPoint p;
  const auto ns = std::llround(seconds * kSecond);
  p.time_from_start.sec = static_cast<std::int32_t>(ns / kSecond);
  p.time_from_start.nanosec = static_cast<std::uint32_t>(ns % kSecond);
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  p.direction = static_cast<std::int8_t>(direction);
  return p;
}

Trajectory trajectory(std::int64_t stamp, std::initializer_list<msg::TrajectoryPoint> points)
{
  Trajectory t;
  t.header.frame_id = "map";
  t.header.stamp.sec = static_cast<std::int32_t>(stamp / kSecond);
  t.header.stamp.nanosec = static_cast<std::uint32_t>(stamp % kSecond);
  t.points = points;
  return t;
}

PlannerConfig smallBody()
{
  PlannerConfig c;
  c.vehicle_length = 0.2;
  c.vehicle_width = 0.2;
  c.rear_overhang = 0.1;
  c.collision_margin = 0.0;
  return c;
}

CollisionCheckResult check(const Trajectory & a, const Trajectory & b,
  const CollisionCheckOptions & options = {})
{
  return detectTrajectoryCollision(a, b, smallBody(), smallBody(), options);
}

TEST(TrajectoryCollision, CrossingPathsAtDifferentAbsoluteTimesDoNotConflict)
{
  const auto a = trajectory(100 * kSecond, {point(0, -5, 0), point(10, 5, 0)});
  // Identical local crossing time t=5, but B arrives three seconds later.
  const auto b = trajectory(103 * kSecond,
    {point(0, 0, -5, kPi / 2), point(10, 0, 5, kPi / 2)});
  const auto result = check(a, b);
  EXPECT_EQ(result.status, CollisionCheckStatus::NoConflictAtSamples);
  EXPECT_FALSE(result.first_conflict);
  EXPECT_EQ(result.checked_start_ns, 103 * kSecond);
  EXPECT_EQ(result.checked_end_ns, 110 * kSecond);
  EXPECT_EQ(result.samples_checked, 71u);
}

TEST(TrajectoryCollision, CrossingAtSameAbsoluteTimeConflictsDespiteDifferentLocalTimes)
{
  const auto a = trajectory(100 * kSecond, {point(0, -5, 0), point(10, 5, 0)});
  // Local t=2 for B equals local t=5 for A: both reach origin at 105 s.
  const auto b = trajectory(103 * kSecond,
    {point(0, 0, -2, kPi / 2), point(4, 0, 2, kPi / 2)});
  const auto a_before = a;
  const auto b_before = b;
  const auto result = check(a, b);
  ASSERT_EQ(result.status, CollisionCheckStatus::Conflict);
  ASSERT_TRUE(result.first_conflict);
  // Body contact occurs before rear-axle centers reach the crossing.
  EXPECT_EQ(result.first_conflict->absolute_time_ns, 104800000000LL);
  EXPECT_NEAR(result.first_conflict->first_pose.x, -0.2, 1e-12);
  EXPECT_NEAR(result.first_conflict->second_pose.y, -0.2, 1e-12);
  EXPECT_EQ(a, a_before);
  EXPECT_EQ(b, b_before);
}

TEST(TrajectoryCollision, WaitingVehicleRemainsOccupied)
{
  const auto waiting = trajectory(100 * kSecond,
    {point(0, 0, 0, 0, msg::TrajectoryPoint::STOP),
      point(10, 0, 0, 0, msg::TrajectoryPoint::STOP)});
  const auto moving = trajectory(100 * kSecond, {point(0, -5, 0), point(10, 5, 0)});
  const auto result = check(waiting, moving);
  ASSERT_TRUE(result.first_conflict);
  EXPECT_EQ(result.first_conflict->absolute_time_ns, 104800000000LL);
  EXPECT_DOUBLE_EQ(result.first_conflict->first_pose.x, 0.0);
}

TEST(TrajectoryCollision, ReverseKeepsVehicleHeading)
{
  const auto reversing = trajectory(100 * kSecond,
    {point(0, 5, 0), point(10, -5, 0, 0, msg::TrajectoryPoint::REVERSE)});
  const auto parked = trajectory(100 * kSecond, {point(0, 0, 0), point(10, 0, 0)});
  const auto result = check(reversing, parked);
  ASSERT_TRUE(result.first_conflict);
  EXPECT_EQ(result.first_conflict->absolute_time_ns, 104800000000LL);
  EXPECT_DOUBLE_EQ(result.first_conflict->first_pose.yaw, 0.0);
  EXPECT_NEAR(result.first_conflict->first_pose.x, 0.2, 1e-12);
}

TEST(TrajectoryCollision, ExpiredCoverageIsNotReportedAsSafe)
{
  const auto t = trajectory(100 * kSecond, {point(0, 0, 0), point(2, 0, 0)});
  CollisionCheckOptions options;
  options.not_before_ns = 103 * kSecond;
  const auto result = check(t, t, options);
  EXPECT_EQ(result.status, CollisionCheckStatus::Expired);
  EXPECT_FALSE(result.first_conflict);
  EXPECT_EQ(result.samples_checked, 0u);
}

TEST(TrajectoryCollision, NotBeforeSkipsHistoricalConflicts)
{
  const auto a = trajectory(100 * kSecond, {point(0, 0, 0), point(10, 10, 0)});
  const auto b = trajectory(100 * kSecond, {point(0, 0, 0), point(10, 0, 0)});
  CollisionCheckOptions options;
  options.not_before_ns = 102 * kSecond;
  const auto result = check(a, b, options);
  EXPECT_EQ(result.status, CollisionCheckStatus::NoConflictAtSamples);
  EXPECT_EQ(result.checked_start_ns, 102 * kSecond);
}

TEST(TrajectoryCollision, DisjointCoverageAndEmptyTrajectoryAreNotReportedAsSafe)
{
  const auto a = trajectory(100 * kSecond, {point(0, 0, 0), point(1, 0, 0)});
  const auto b = trajectory(102 * kSecond, {point(0, 0, 0), point(1, 0, 0)});
  EXPECT_EQ(check(a, b).status, CollisionCheckStatus::NoCommonTime);
  EXPECT_EQ(check(a, trajectory(100 * kSecond, {})).status,
    CollisionCheckStatus::NoCommonTime);
}

TEST(TrajectoryCollision, NonzeroFirstOffsetAndNanosecondStampAreAlignedExactly)
{
  const auto a = trajectory(1700000000123456789LL, {point(2, 0, 0)});
  const auto b = trajectory(1700000002123456789LL, {point(0, 0, 0)});
  const auto result = check(a, b);
  ASSERT_TRUE(result.first_conflict);
  EXPECT_EQ(result.first_conflict->absolute_time_ns, 1700000002123456789LL);
  EXPECT_EQ(result.samples_checked, 1u);
}

TEST(TrajectoryCollision, SamplesFinalEndpointEvenWhenStepDoesNotDivideInterval)
{
  const auto a = trajectory(100 * kSecond, {point(0, 0, 0), point(0.25, 0, 0)});
  const auto b = trajectory(100 * kSecond, {point(0, 0.45, 0), point(0.25, 0.2, 0)});
  const auto result = check(a, b);
  ASSERT_TRUE(result.first_conflict);
  EXPECT_EQ(result.first_conflict->absolute_time_ns, 100250000000LL);
  EXPECT_EQ(result.samples_checked, 4u);
}

TEST(TrajectoryCollision, YawInterpolatesAcrossPiByShortestAngle)
{
  const auto a = trajectory(0, {point(0, 0, 0, 179 * kPi / 180),
      point(2, 0, 0, -179 * kPi / 180)});
  const auto b = trajectory(kSecond, {point(0, 0, 0)});
  const auto result = check(a, b);
  ASSERT_TRUE(result.first_conflict);
  EXPECT_NEAR(std::abs(result.first_conflict->first_pose.yaw), kPi, 1e-12);
}

TEST(TrajectoryCollision, UsesRearAxleOffsetAndIndependentVehicleDimensions)
{
  PlannerConfig large;
  large.collision_margin = 0.0;
  const auto a = trajectory(0, {point(0, 0, 0)});
  const auto b = trajectory(0, {point(0, 3.6, 0, kPi / 2)});
  EXPECT_EQ(detectTrajectoryCollision(a, b, large, smallBody()).status,
    CollisionCheckStatus::Conflict);  // Front edge at 3.5 touches B's edge.
  const auto behind = trajectory(0, {point(0, -1.3, 0)});
  EXPECT_EQ(detectTrajectoryCollision(a, behind, large, smallBody()).status,
    CollisionCheckStatus::NoConflictAtSamples);
  large.collision_margin = 0.2;
  EXPECT_EQ(detectTrajectoryCollision(a, behind, large, smallBody()).status,
    CollisionCheckStatus::Conflict);
}

TEST(TrajectoryCollision, RotatedRectanglesCanSeparateDespiteOverlappingBoundingBoxes)
{
  auto geometry = smallBody();
  geometry.vehicle_length = 4.0;
  geometry.vehicle_width = 0.2;
  geometry.rear_overhang = 2.0;
  const auto a = trajectory(0, {point(0, 0, 0, kPi / 4)});
  const auto b = trajectory(0, {point(0, -0.5, 0.5, kPi / 4)});
  EXPECT_EQ(detectTrajectoryCollision(a, b, geometry, geometry).status,
    CollisionCheckStatus::NoConflictAtSamples);
}

TEST(TrajectoryCollision, RejectsInvalidInputs)
{
  const auto valid = trajectory(0, {point(0, 0, 0), point(1, 1, 0)});
  auto invalid = valid;
  invalid.header.frame_id = "odom";
  EXPECT_THROW(check(valid, invalid), std::invalid_argument);
  invalid = valid;
  invalid.points[1].time_from_start = invalid.points[0].time_from_start;
  EXPECT_THROW(check(valid, invalid), std::invalid_argument);
  invalid = valid;
  invalid.points[0].time_from_start.sec = -1;
  EXPECT_THROW(check(valid, invalid), std::invalid_argument);
  invalid = valid;
  invalid.header.stamp.nanosec = 1000000000u;
  EXPECT_THROW(check(valid, invalid), std::invalid_argument);
  invalid = valid;
  invalid.points[0].pose.orientation.w = 0.0;
  EXPECT_THROW(check(valid, invalid), std::invalid_argument);
  invalid = valid;
  invalid.points[0].pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(check(valid, invalid), std::invalid_argument);
  CollisionCheckOptions options;
  options.sample_step_ns = 0;
  EXPECT_THROW(check(valid, valid, options), std::invalid_argument);
  auto geometry = smallBody();
  geometry.vehicle_width = -1;
  EXPECT_THROW(detectTrajectoryCollision(valid, valid, geometry), std::invalid_argument);
}

TEST(TrajectoryCollision, CoarseSamplingCanMissCollisionBetweenSamples)
{
  const auto a = trajectory(0, {point(0, -1, 0), point(1, 1, 0)});
  const auto b = trajectory(0, {point(0, 0, -1), point(1, 0, 1)});
  CollisionCheckOptions options;
  options.sample_step_ns = kSecond;
  EXPECT_EQ(check(a, b, options).status, CollisionCheckStatus::NoConflictAtSamples);
  options.sample_step_ns = kSecond / 10;
  EXPECT_EQ(check(a, b, options).status, CollisionCheckStatus::Conflict);
}

TEST(TrajectoryCollision, PlaybackInterpolationHoldsStopThenResumesWithoutExtrapolation)
{
  const auto t = trajectory(100 * kSecond,
    {point(0, 0, 0), point(2, 2, 0), point(5, 2, 0, 0, msg::TrajectoryPoint::STOP),
      point(7, 4, 0)});
  const auto waiting = interpolateTrajectoryPose(t, 104 * kSecond);
  ASSERT_TRUE(waiting);
  EXPECT_DOUBLE_EQ(waiting->x, 2.0);
  const auto resumed = interpolateTrajectoryPose(t, 106 * kSecond);
  ASSERT_TRUE(resumed);
  EXPECT_DOUBLE_EQ(resumed->x, 3.0);
  const auto endpoint = interpolateTrajectoryPose(t, 107 * kSecond);
  ASSERT_TRUE(endpoint);
  EXPECT_DOUBLE_EQ(endpoint->x, 4.0);
  EXPECT_FALSE(interpolateTrajectoryPose(t, 99 * kSecond));
  EXPECT_FALSE(interpolateTrajectoryPose(t, 108 * kSecond));
  EXPECT_FALSE(interpolateTrajectoryPose(trajectory(100 * kSecond, {}), 100 * kSecond));
}
}  // namespace
