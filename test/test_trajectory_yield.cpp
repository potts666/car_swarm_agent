#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include "car_swarm_agent/trajectory_yield.hpp"

namespace {
using namespace car_swarm_agent;
using Trajectory = msg::PredictedTrajectory;
using Point = msg::TrajectoryPoint;
constexpr std::int64_t kSecond = 1000000000LL;
constexpr std::int64_t kStamp = 100 * kSecond;
constexpr double kPi = 3.14159265358979323846;

std::int64_t offset(const Point & p)
{
  return static_cast<std::int64_t>(p.time_from_start.sec) * kSecond +
         p.time_from_start.nanosec;
}

Point point(double seconds, double x, double y, double yaw = 0.0,
  int direction = Point::FORWARD)
{
  Point p;
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

Trajectory trajectory(const std::string & id, std::initializer_list<Point> points,
  std::int64_t stamp = kStamp)
{
  Trajectory t;
  t.vehicle_id = id;
  t.header.frame_id = "map";
  t.header.stamp.sec = static_cast<std::int32_t>(stamp / kSecond);
  t.header.stamp.nanosec = static_cast<std::uint32_t>(stamp % kSecond);
  t.points = points;
  if (!t.points.empty()) {t.points.front().direction = Point::STOP;}
  return t;
}

PlannerConfig body()
{
  PlannerConfig c;
  c.vehicle_length = c.vehicle_width = 0.2;
  c.rear_overhang = 0.1;
  c.collision_margin = 0.0;
  return c;
}

Trajectory crossingOwn()
{
  return trajectory("car_2", {point(0, 0, -2, kPi / 2),
      point(2, 0, 0, kPi / 2), point(4, 0, 2, kPi / 2)});
}

Trajectory crossingNeighbor()
{
  return trajectory("car_1", {point(0, -2, 0), point(2, 0, 0),
      point(4, 2, 0), point(10, 8, 0)});
}

YieldResult plan(const Trajectory & own, const Trajectory & neighbor,
  const YieldOptions & options = {}, std::int64_t now = kStamp)
{
  return planFixedPathYield(own, neighbor, now, body(), body(), options);
}

TEST(TrajectoryYield, SimultaneousCrossingClearsAfterFirstFeasibleWait)
{
  const auto own = crossingOwn();
  const auto neighbor = crossingNeighbor();
  const auto own_before = own;
  const auto neighbor_before = neighbor;
  const auto result = plan(own, neighbor);
  ASSERT_EQ(result.status, YieldStatus::Yielded);
  ASSERT_TRUE(result.trajectory);
  EXPECT_EQ(result.wait_ns, kSecond / 2);
  ASSERT_TRUE(result.stop_point_index);
  EXPECT_EQ(*result.stop_point_index, 0u);
  const auto & adjusted = *result.trajectory;
  ASSERT_EQ(adjusted.points.size(), own.points.size() + 1);
  EXPECT_EQ(adjusted.header, own.header);
  EXPECT_EQ(adjusted.vehicle_id, own.vehicle_id);
  EXPECT_EQ(offset(adjusted.points.front()), 0);
  EXPECT_EQ(adjusted.points[1].direction, Point::STOP);
  EXPECT_EQ(adjusted.points[1].pose, own.points.front().pose);
  EXPECT_EQ(offset(adjusted.points[1]), kSecond / 2);
  EXPECT_EQ(adjusted.points[0], own.points[0]);
  for (std::size_t i = 1; i < own.points.size(); ++i) {
    EXPECT_EQ(adjusted.points[i + 1].pose, own.points[i].pose);
    EXPECT_EQ(adjusted.points[i + 1].direction, own.points[i].direction);
    EXPECT_EQ(offset(adjusted.points[i + 1]), offset(own.points[i]) + result.wait_ns);
  }
  for (std::size_t i = 1; i < adjusted.points.size(); ++i) {
    EXPECT_GT(offset(adjusted.points[i]), offset(adjusted.points[i - 1]));
  }
  ASSERT_TRUE(result.verification);
  EXPECT_EQ(result.verification->status, CollisionCheckStatus::NoConflictAtSamples);
  EXPECT_EQ(result.verification->checked_start_ns, kStamp);
  EXPECT_EQ(result.verification->checked_end_ns, kStamp + 4500000000LL);
  EXPECT_EQ(detectTrajectoryCollision(adjusted, neighbor, body(), body()).status,
    CollisionCheckStatus::NoConflictAtSamples);
  EXPECT_EQ(own, own_before);
  EXPECT_EQ(neighbor, neighbor_before);
}

TEST(TrajectoryYield, AlreadySeparatedInAbsoluteTimeRemainsExactlyUnchanged)
{
  const auto own = crossingOwn();
  // Neighbor crosses at absolute 100 s, own at 102 s; same spatial paths.
  const auto neighbor = trajectory("car_1", {point(0, -2, 0), point(2, 0, 0),
      point(4, 2, 0), point(10, 8, 0)}, kStamp - 2 * kSecond);
  const auto result = plan(own, neighbor);
  EXPECT_EQ(result.status, YieldStatus::Unchanged);
  ASSERT_TRUE(result.trajectory);
  EXPECT_EQ(*result.trajectory, own);
  EXPECT_EQ(result.wait_ns, 0);
  EXPECT_EQ(result.candidates_tried, 0u);
}

TEST(TrajectoryYield, UnsafeParkingPositionRejectsAllWaits)
{
  const auto own = crossingOwn();
  // Neighbor sweeps through the only eligible parking point during every wait,
  // then crosses the moving original trajectory at t=2.
  const auto neighbor = trajectory("car_1", {point(0, -1, -2), point(0.5, 0, -2),
      point(0.8, 0, -2, 0, Point::STOP), point(1, -1, 0), point(2, 0, 0),
      point(3, 1, 0), point(10, 8, 0)});
  YieldOptions options;
  options.max_wait_ns = 2 * kSecond;
  const auto result = plan(own, neighbor, options);
  EXPECT_EQ(result.status, YieldStatus::NoFeasiblePlan);
  EXPECT_FALSE(result.trajectory);
  EXPECT_FALSE(result.verification);
  EXPECT_EQ(result.candidates_tried, 4u);
  ASSERT_TRUE(result.original_conflict);
  EXPECT_EQ(result.original_conflict->absolute_time_ns, kStamp + 1800000000LL);
}

TEST(TrajectoryYield, SmallerIdDoesNotAlsoWaitOrReturnConflictingOriginal)
{
  const auto result = plan(crossingNeighbor(), crossingOwn());
  // Full coverage is required for the original interval before priority logic.
  EXPECT_EQ(result.status, YieldStatus::InformationInsufficient);
  EXPECT_FALSE(result.trajectory);
  auto own = crossingOwn();
  auto neighbor = crossingNeighbor();
  own.vehicle_id = "car_1";
  neighbor.vehicle_id = "car_2";
  const auto priority = plan(own, neighbor);
  EXPECT_EQ(priority.status, YieldStatus::AwaitingPeerYield);
  EXPECT_FALSE(priority.trajectory);
  EXPECT_EQ(priority.candidates_tried, 0u);
}

TEST(TrajectoryYield, NoCommonTimeAndExpiredNeighborMeanInsufficientInformation)
{
  auto neighbor = crossingNeighbor();
  neighbor.header.stamp.sec += 20;
  EXPECT_EQ(plan(crossingOwn(), neighbor).status, YieldStatus::InformationInsufficient);
  const auto expired = plan(crossingOwn(), crossingNeighbor(), {}, kStamp + 11 * kSecond);
  EXPECT_EQ(expired.status, YieldStatus::InformationInsufficient);
  EXPECT_FALSE(expired.trajectory);
  const auto past_neighbor = trajectory("car_1", {point(0, -2, 0), point(1, -1, 0)},
    kStamp - 2 * kSecond);
  EXPECT_EQ(plan(crossingOwn(), past_neighbor).status, YieldStatus::InformationInsufficient);
}

TEST(TrajectoryYield, PartialOriginalCoverageCannotBeAcceptedAsSafe)
{
  const auto own = crossingOwn();
  const auto neighbor = trajectory("car_1", {point(0, 10, 10), point(1, 10, 10)});
  EXPECT_EQ(detectTrajectoryCollision(own, neighbor, body(), body()).status,
    CollisionCheckStatus::NoConflictAtSamples);
  const auto result = plan(own, neighbor);
  EXPECT_EQ(result.status, YieldStatus::InformationInsufficient);
  EXPECT_FALSE(result.trajectory);
}

TEST(TrajectoryYield, WaitingBeyondNeighborHorizonIsNotAResolution)
{
  const auto neighbor = trajectory("car_1", {point(0, -2, 0), point(2, 0, 0),
      point(4, 2, 0)});
  YieldOptions options;
  options.max_wait_ns = kSecond;
  const auto result = plan(crossingOwn(), neighbor, options);
  EXPECT_EQ(result.status, YieldStatus::InformationInsufficient);
  EXPECT_FALSE(result.trajectory);
  EXPECT_EQ(result.candidates_tried, 2u);
}

TEST(TrajectoryYield, WholeCandidateRecheckRejectsLaterConflict)
{
  const auto own = crossingOwn();
  // A 0.5 s wait clears the crossing, but neighbor then meets the delayed
  // destination at 4.5 s. Parking itself remains clear.
  const auto neighbor = trajectory("car_1", {point(0, -2, 0), point(2, 0, 0),
      point(3, 1, 0), point(4.5, 0, 2), point(8, 0, 2)});
  YieldOptions options;
  options.max_wait_ns = options.wait_step_ns;
  const auto result = plan(own, neighbor, options);
  EXPECT_EQ(result.status, YieldStatus::NoFeasiblePlan);
  EXPECT_FALSE(result.trajectory);
  EXPECT_EQ(result.candidates_tried, 1u);
}

TEST(TrajectoryYield, FallsBackToEarlierParkingPointWhenNearestOneIsUnsafe)
{
  auto own = crossingOwn();
  own.points.insert(own.points.begin() + 1, point(1, 0, -1, kPi / 2));
  const auto neighbor = trajectory("car_1", {point(0, -2, 0), point(1, -1, -1),
      point(1.5, 0, -1), point(2, 0, 0), point(4, 2, 0), point(10, 8, 0)});
  const auto result = plan(own, neighbor);
  ASSERT_EQ(result.status, YieldStatus::Yielded);
  ASSERT_TRUE(result.stop_point_index);
  EXPECT_EQ(*result.stop_point_index, 0u);
  EXPECT_GT(result.candidates_tried, 1u);
}

TEST(TrajectoryYield, LatestSafeParkingPointKeepsEarlierPrefixUnchanged)
{
  auto own = crossingOwn();
  own.points.insert(own.points.begin() + 1, point(1, 0, -1, kPi / 2));
  const auto result = plan(own, crossingNeighbor());
  ASSERT_EQ(result.status, YieldStatus::Yielded);
  ASSERT_TRUE(result.stop_point_index);
  EXPECT_EQ(*result.stop_point_index, 1u);
  EXPECT_EQ(result.wait_ns, kSecond / 2);
  ASSERT_TRUE(result.trajectory);
  const auto & candidate = *result.trajectory;
  EXPECT_EQ(candidate.points[0], own.points[0]);
  EXPECT_EQ(candidate.points[1], own.points[1]);
  EXPECT_EQ(candidate.points[2].pose, own.points[1].pose);
  EXPECT_EQ(candidate.points[2].direction, Point::STOP);
  EXPECT_EQ(offset(candidate.points[2]), 1500000000LL);
  for (std::size_t i = 2; i < own.points.size(); ++i) {
    EXPECT_EQ(candidate.points[i + 1].pose, own.points[i].pose);
    EXPECT_EQ(candidate.points[i + 1].direction, own.points[i].direction);
    EXPECT_EQ(offset(candidate.points[i + 1]), offset(own.points[i]) + result.wait_ns);
  }
}

TEST(TrajectoryYield, ReverseSegmentDirectionsAndHeadingArePreserved)
{
  const auto own = trajectory("car_2", {point(0, 0, -2, -kPi / 2),
      point(2, 0, 0, -kPi / 2, Point::REVERSE),
      point(4, 0, 2, -kPi / 2, Point::REVERSE)});
  const auto result = plan(own, crossingNeighbor());
  ASSERT_EQ(result.status, YieldStatus::Yielded);
  ASSERT_TRUE(result.trajectory);
  EXPECT_EQ(result.trajectory->points[2].direction, Point::REVERSE);
  EXPECT_EQ(result.trajectory->points[2].pose, own.points[1].pose);
}

TEST(TrajectoryYield, DoesNotInsertWaitBeforeCurrentTime)
{
  const auto result = plan(crossingOwn(), crossingNeighbor(), {}, kStamp + kSecond);
  EXPECT_EQ(result.status, YieldStatus::NoFeasiblePlan);
  EXPECT_FALSE(result.trajectory);
  EXPECT_EQ(result.candidates_tried, 0u);
}

TEST(TrajectoryYield, InitialCollisionCannotBeFixedByWaitingAtThatLocation)
{
  const auto own = trajectory("car_2", {point(0, 0, 0), point(2, 2, 0)});
  const auto neighbor = trajectory("car_1", {point(0, 0, 0), point(10, 0, 10)});
  const auto result = plan(own, neighbor);
  EXPECT_EQ(result.status, YieldStatus::NoFeasiblePlan);
  EXPECT_FALSE(result.trajectory);
  EXPECT_EQ(result.candidates_tried, 0u);
}

TEST(TrajectoryYield, EmptyInputMeansInsufficientInformation)
{
  const auto result = plan(trajectory("car_2", {}), crossingNeighbor());
  EXPECT_EQ(result.status, YieldStatus::InformationInsufficient);
  EXPECT_FALSE(result.trajectory);
}

TEST(TrajectoryYield, DurationOverflowRejectsCandidateWithoutWrappingTime)
{
  auto own = crossingOwn();
  auto neighbor = crossingNeighbor();
  own.points.back().time_from_start.sec = std::numeric_limits<std::int32_t>::max();
  own.points.back().time_from_start.nanosec = 999999999u;
  neighbor.points.pop_back();
  neighbor.points.back().time_from_start = own.points.back().time_from_start;
  YieldOptions options;
  options.max_wait_ns = options.wait_step_ns;
  const auto result = plan(own, neighbor, options);
  EXPECT_EQ(result.status, YieldStatus::NoFeasiblePlan);
  EXPECT_FALSE(result.trajectory);
  EXPECT_EQ(result.candidates_tried, 1u);
}

TEST(TrajectoryYield, RejectsInvalidIdsStartOffsetAndWaitOptions)
{
  auto own = crossingOwn();
  own.vehicle_id = "car_1";
  EXPECT_THROW(plan(own, crossingNeighbor()), std::invalid_argument);
  own.vehicle_id.clear();
  EXPECT_THROW(plan(own, crossingNeighbor()), std::invalid_argument);
  own = crossingOwn();
  own.points.front().time_from_start.nanosec = 1;
  EXPECT_THROW(plan(own, crossingNeighbor()), std::invalid_argument);
  own = crossingOwn();
  own.points.front().direction = Point::FORWARD;
  EXPECT_THROW(plan(own, crossingNeighbor()), std::invalid_argument);
  YieldOptions options;
  options.wait_step_ns = 0;
  EXPECT_THROW(plan(crossingOwn(), crossingNeighbor(), options), std::invalid_argument);
  options.wait_step_ns = kSecond;
  options.max_wait_ns = -1;
  EXPECT_THROW(plan(crossingOwn(), crossingNeighbor(), options), std::invalid_argument);
  options.max_wait_ns = kSecond;
  options.sample_step_ns = 0;
  EXPECT_THROW(plan(crossingOwn(), crossingNeighbor(), options), std::invalid_argument);
}
}  // namespace


namespace st_speed_test {
using namespace car_swarm_agent;
constexpr std::int64_t kSecond = 1000000000LL;
constexpr double kPi = 3.14159265358979323846;

std_msgs::msg::Header header()
{
  std_msgs::msg::Header h;
  h.frame_id = "map";
  h.stamp.sec = 100;
  return h;
}

msg::TrajectoryPoint point(int seconds, double x, double y)
{
  msg::TrajectoryPoint p;
  p.time_from_start.sec = seconds;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.w = 1;
  p.direction = seconds == 0 ? msg::TrajectoryPoint::STOP : msg::TrajectoryPoint::FORWARD;
  return p;
}

msg::PredictedTrajectory neighbor(bool crossing = false)
{
  msg::PredictedTrajectory t;
  t.header = header();
  t.vehicle_id = "car_1";
  t.points = crossing ? std::vector<msg::TrajectoryPoint>{
    point(0, -10, 0), point(10, 0, 0), point(60, 50, 0)} :
    std::vector<msg::TrajectoryPoint>{point(0, 100, 100), point(60, 100, 100)};
  return t;
}

PlannerConfig smallBody()
{
  PlannerConfig c;
  c.vehicle_length = c.vehicle_width = 0.2;
  c.rear_overhang = 0.1;
  c.collision_margin = 0;
  return c;
}

void checkDynamics(const STResult & result, const STOptions & o)
{
  ASSERT_EQ(result.status, STStatus::Success);
  ASSERT_TRUE(result.trajectory);
  ASSERT_EQ(result.samples.size(), result.trajectory->points.size());
  ASSERT_FALSE(result.samples.empty());
  EXPECT_DOUBLE_EQ(result.samples.front().speed, o.initial_speed);
  EXPECT_EQ(result.samples.front().time_from_start_ns, 0);
  EXPECT_NEAR(result.samples.back().speed, 0, 1e-8);
  for (std::size_t i = 1; i < result.samples.size(); ++i) {
    const auto & a = result.samples[i - 1];
    const auto & b = result.samples[i];
    ASSERT_GT(b.time_from_start_ns, a.time_from_start_ns);
    const double dt = (b.time_from_start_ns - a.time_from_start_ns) / 1e9;
    EXPECT_GE(b.s + 1e-8, a.s);
    EXPECT_GE(b.speed, 0);
    EXPECT_LE(b.speed, o.max_speed + 1e-8);
    EXPECT_LE(std::abs(b.speed - a.speed) / dt, o.max_acceleration + 1e-6);
    EXPECT_NEAR(b.s - a.s, 0.5 * (a.speed + b.speed) * dt, 1e-7);
    const auto & p = result.trajectory->points[i];
    EXPECT_EQ(static_cast<std::int64_t>(p.time_from_start.sec) * kSecond +
      p.time_from_start.nanosec, b.time_from_start_ns);
  }
}

TEST(STSpeedPlanner, CrossingSlowsOrWaitsThenResumesAndChecksWholeTrajectory)
{
  // Same crossing scene and full-size bodies as yield_demo_node.
  const std::vector<Pose> path{{0, -10, kPi / 2}, {0, -6, kPi / 2, 4, 0},
    {0, 0, kPi / 2, 6, 0}, {0, 10, kPi / 2, 10, 0}};
  STOptions o;
  o.max_speed = o.initial_speed = 1;
  const auto other = neighbor(true);
  const auto before = other;
  const auto result = planSTSpeed(path, other, header(), "car_2", {}, {}, o);
  checkDynamics(result, o);
  ASSERT_TRUE(result.trajectory);
  EXPECT_EQ(result.trajectory->header, header());
  EXPECT_NEAR(result.samples.back().s, 20, 1e-8);
  const auto slow = std::find_if(result.samples.begin(), result.samples.end(),
    [](const STSample & s) {return s.s < 10 && s.speed < 0.9;});
  ASSERT_NE(slow, result.samples.end());
  EXPECT_TRUE(std::any_of(slow, result.samples.end(),
    [](const STSample & s) {return s.speed > 0.9;}));
  EXPECT_EQ(detectTrajectoryCollision(*result.trajectory, other).status,
    CollisionCheckStatus::NoConflictAtSamples);
  EXPECT_EQ(other, before);
}

TEST(STSpeedPlanner, HighInitialSpeedCannotInstantlyStopBeforeObstacle)
{
  const std::vector<Pose> path{{0, 0, 0}, {10, 0, 0, 10, 0}};
  auto other = neighbor();
  other.points = {point(0, 1, 0), point(10, 1, 0)};
  STOptions o;
  o.initial_speed = o.max_speed = 3;
  o.max_acceleration = 1;
  // Braking distance is 4.5 m; obstacle contact begins around s=0.8 m.
  const auto result = planSTSpeed(path, other, header(), "car_2", smallBody(), smallBody(), o);
  EXPECT_EQ(result.status, STStatus::NoFeasiblePlan);
  EXPECT_FALSE(result.trajectory);
  EXPECT_TRUE(result.samples.empty());
  EXPECT_FALSE(result.verification);
}

TEST(STSpeedPlanner, UncoveredFutureIsUnknownNotSafe)
{
  const std::vector<Pose> path{{0, 0, 0}, {10, 0, 0, 10, 0}};
  auto other = neighbor();
  other.points.back().time_from_start.sec = 1;
  const auto result = planSTSpeed(path, other, header(), "car_2", smallBody(), smallBody());
  EXPECT_EQ(result.status, STStatus::InformationInsufficient);
  EXPECT_FALSE(result.trajectory);
}

TEST(STSpeedPlanner, ExpiredOrNotYetCoveredStartIsUnknown)
{
  const std::vector<Pose> path{{0, 0, 0}, {2, 0, 0, 2, 0}};
  auto other = neighbor();
  other.header.stamp.sec = 10;
  EXPECT_EQ(planSTSpeed(path, other, header(), "car_2").status,
    STStatus::InformationInsufficient);
  other.header.stamp.sec = 101;
  EXPECT_EQ(planSTSpeed(path, other, header(), "car_2").status,
    STStatus::InformationInsufficient);
}

TEST(STSpeedPlanner, ReverseAccumulatesPositiveDistanceAndShiftsAtZeroSpeed)
{
  const std::vector<Pose> path{{0, 0, 0}, {2, 0, 0, 2, 0}, {1, 0, 0, -1, 0}};
  STOptions o;
  o.initial_speed = 0;
  const auto result = planSTSpeed(path, neighbor(), header(), "car_2", smallBody(), smallBody(), o);
  checkDynamics(result, o);
  ASSERT_TRUE(result.trajectory);
  EXPECT_NEAR(result.samples.back().s, 3, 1e-8);
  EXPECT_NEAR(result.trajectory->points.back().pose.position.x, 1, 1e-8);
  bool saw_zero_at_cusp = false, saw_reverse = false;
  for (std::size_t i = 0; i < result.samples.size(); ++i) {
    const auto & s = result.samples[i];
    if (std::abs(s.s - 2) < 1e-8 && s.speed < 1e-8) {saw_zero_at_cusp = true;}
    if (result.trajectory->points[i].direction == msg::TrajectoryPoint::REVERSE) {
      saw_reverse = true;
      EXPECT_TRUE(saw_zero_at_cusp);
    }
  }
  EXPECT_TRUE(saw_reverse);
}

TEST(STSpeedPlanner, ArcGeometryUsesArcLengthRatherThanChord)
{
  const std::vector<Pose> path{{0, 0, 0}, {1, 1, kPi / 2, kPi / 2, 1}};
  STOptions o;
  o.initial_speed = 0;
  const auto result = planSTSpeed(path, neighbor(), header(), "car_2", smallBody(), smallBody(), o);
  checkDynamics(result, o);
  ASSERT_TRUE(result.trajectory);
  EXPECT_NEAR(result.samples.back().s, kPi / 2, 1e-8);
  for (std::size_t i = 0; i < result.samples.size(); ++i) {
    EXPECT_NEAR(result.trajectory->points[i].pose.position.x,
      std::sin(result.samples[i].s), 1e-7);
    EXPECT_NEAR(result.trajectory->points[i].pose.position.y,
      1 - std::cos(result.samples[i].s), 1e-7);
  }
}

TEST(STSpeedPlanner, SearchBudgetExhaustionIsExplicit)
{
  const std::vector<Pose> path{{0, 0, 0}, {10, 0, 0, 10, 0}};
  STOptions o;
  o.max_nodes = 2;
  EXPECT_EQ(planSTSpeed(path, neighbor(), header(), "car_2", {}, {}, o).status,
    STStatus::SearchLimit);
}

TEST(STSpeedPlanner, InvalidInputsThrow)
{
  const std::vector<Pose> path{{0, 0, 0}, {2, 0, 0, 2, 0}};
  STOptions o;
  o.initial_speed = 3;
  EXPECT_THROW(planSTSpeed(path, neighbor(), header(), "car_2", {}, {}, o), std::invalid_argument);
  auto broken = path;
  broken[1].signed_distance = 1;
  EXPECT_THROW(planSTSpeed(broken, neighbor(), header(), "car_2"), std::invalid_argument);
  o = {};
  o.sample_step_ns = 0;
  EXPECT_THROW(planSTSpeed(path, neighbor(), header(), "car_2", {}, {}, o), std::invalid_argument);
}

TEST(STSpeedPlanner, UsesExistingHybridAStarOutputWithIncomingArcMetadata)
{
  auto geometry = smallBody();
  geometry.max_iterations = 20000;
  geometry.step_size = 0.5;
  geometry.goal_tolerance = 1e-6;
  geometry.goal_yaw_tolerance = 1e-6;
  HybridAStarPlanner planner(geometry);
  PlanningStats stats;
  const auto path = planner.plan({0, 0, 0}, {3, 0, 0}, &stats);
  ASSERT_TRUE(stats.reached_goal);
  ASSERT_FALSE(path.empty());
  const auto original_path = path;
  STOptions o;
  const auto result = planSTSpeed(path, neighbor(), header(), "car_2", geometry, geometry, o);
  checkDynamics(result, o);
  ASSERT_TRUE(result.trajectory);
  EXPECT_NEAR(result.samples.back().s, stats.path_length, 1e-8);
  EXPECT_NEAR(result.trajectory->points.back().pose.position.x, path.back().x, 1e-8);
  EXPECT_NEAR(result.trajectory->points.back().pose.position.y, path.back().y, 1e-8);
  for (std::size_t i = 0; i < path.size(); ++i) {
    EXPECT_DOUBLE_EQ(path[i].signed_distance, original_path[i].signed_distance);
    EXPECT_DOUBLE_EQ(path[i].curvature, original_path[i].curvature);
  }
}
}  // namespace
