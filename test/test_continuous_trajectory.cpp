#include <gtest/gtest.h>
#include <cmath>
#include <stdexcept>

#include "car_swarm_agent/continuous_trajectory.hpp"
#include "car_swarm_agent/demo_map.hpp"
#include "car_swarm_agent/demo_validation.hpp"

using namespace car_swarm_agent;

namespace {

std_msgs::msg::Header header()
{
  std_msgs::msg::Header h;
  h.frame_id = "map";
  h.stamp.sec = 100;
  return h;
}

PlannerConfig body()
{
  PlannerConfig g;
  g.vehicle_length = g.vehicle_width = 0.2;
  g.rear_overhang = 0.1;
  g.collision_margin = 0;
  return g;
}

msg::TrajectoryPoint point(int t, double x, double y)
{
  msg::TrajectoryPoint p;
  p.time_from_start.sec = t;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.w = 1;
  return p;
}

msg::PredictedTrajectory peer(
  double x0 = 100, double y0 = 100,
  double x1 = 100, double y1 = 100)
{
  msg::PredictedTrajectory p;
  p.header = header();
  p.vehicle_id = "other";
  p.points = {
    point(0, x0, y0),
    point(2, x1, y1)
  };
  return p;
}

ContinuousTrajectory straight()
{
  // 初速 2，减速度 1，2 秒后在 s=2 停车。
  return ContinuousTrajectory(
    {{0, 0, 0}, {2, 0, 0, 2, 0}},
    header(),
    {{0, 2, 0, 2, -1, 2}});
}

TEST(ContinuousTrajectory, AnalyticMotionAndLimits)
{
  const auto c = straight();
  const auto q = c.evaluate(0.5);

  EXPECT_NEAR(q.s, 0.875, 1e-12);
  EXPECT_NEAR(q.speed, 1.5, 1e-12);
  EXPECT_NEAR(q.ax, -1, 1e-12);
  EXPECT_NEAR(c.evaluate(2).speed, 0, 1e-12);

  EXPECT_EQ(
    verifyContinuousTrajectory(
      c, peer(), body(), body()).status,
    ContinuousCheckStatus::Safe);

  EXPECT_THROW(c.evaluate(-0.1), std::out_of_range);
  EXPECT_THROW(c.evaluate(2.1), std::out_of_range);
}

TEST(ContinuousTrajectory, CollisionBetweenCoarseSamples)
{
  const auto other = peer(0.875, -1, 0.875, 3);

  msg::PredictedTrajectory coarse;
  coarse.header = header();
  coarse.points = {
    point(0, 0, 0),
    point(2, 2, 0)
  };

  CollisionCheckOptions sampling;
  sampling.sample_step_ns = 1000000000;

  // 粗采样没有发现碰撞。
  EXPECT_EQ(
    detectTrajectoryCollision(
      coarse, other, body(), body(), sampling).status,
    CollisionCheckStatus::NoConflictAtSamples);

  // 真实恒减速运动在 t=0.5 时相撞。
  EXPECT_EQ(
    verifyContinuousTrajectory(
      straight(), other, body(), body()).status,
    ContinuousCheckStatus::CollisionOrStaticBlocked);
}

TEST(ContinuousTrajectory, GearChangeRequiresZeroSpeed)
{
  ContinuousTrajectory c(
    {{0, 0, 0}, {2, 0, 0, 2, 0}, {1, 0, 0, -1, 0}},
    header(),
    {
      {0, 2, 0, 1, 0, 2},
      {2, 2, 2, 1, -0.5, 2}
    });

  auto other = peer();
  other.points.back().time_from_start.sec = 4;

  EXPECT_EQ(
    verifyContinuousTrajectory(
      c, other, body(), body()).status,
    ContinuousCheckStatus::DynamicsViolation);
}

TEST(ContinuousTrajectory, ZeroSpeedGearChangeAndAccelerationSides)
{
  ContinuousTrajectory c(
    {{0, 0, 0}, {1, 0, 0, 1, 0}, {0, 0, 0, -1, 0}},
    header(),
    {
      {0, 1, 0, 0, 1, 1},
      {1, 1, 0.5, 1, -1, 1},
      {2, 1, 1, 0, 1, 1},
      {3, 1, 1.5, 1, -1, 1}
    });

  auto other = peer();
  other.points.back().time_from_start.sec = 4;

  EXPECT_EQ(
    verifyContinuousTrajectory(
      c, other, body(), body()).status,
    ContinuousCheckStatus::Safe);

  EXPECT_NEAR(c.evaluate(2).speed, 0, 1e-12);
  EXPECT_LT(c.evaluate(2.5).signed_speed, 0);

  EXPECT_NEAR(c.evaluate(1, true).ax, 1, 1e-12);
  EXPECT_NEAR(c.evaluate(1).ax, -1, 1e-12);
}

TEST(ContinuousTrajectory, TotalAccelerationIncludesCurvature)
{
  ContinuousTrajectory c(
    {
      {0, 0, 0},
      {std::sin(1.0), 1 - std::cos(1.0), 1, 1, 1}
    },
    header(),
    {
      {0, 1, 0, 0, 1, 1},
      {1, 1, 0.5, 1, -1, 1}
    });

  ContinuousCheckOptions o;
  o.max_total_acceleration = 1.1;

  EXPECT_EQ(
    verifyContinuousTrajectory(
      c, peer(), body(), body(), o).status,
    ContinuousCheckStatus::DynamicsViolation);

  const auto q = c.evaluate(0.5);
  EXPECT_NEAR(q.pose.x, std::sin(q.s), 1e-12);
  EXPECT_NEAR(
    std::hypot(q.ax, q.ay),
    std::hypot(1.0, q.speed * q.speed),
    1e-12);
}

TEST(ContinuousTrajectory, CoverageAndUnresolvedAreNotSafe)
{
  auto other = peer();
  other.points.back().time_from_start.sec = 1;

  EXPECT_EQ(
    verifyContinuousTrajectory(
      straight(), other, body(), body()).status,
    ContinuousCheckStatus::InformationInsufficient);

  ContinuousCheckOptions o;
  o.min_interval = 10;

  EXPECT_EQ(
    verifyContinuousTrajectory(
      straight(), peer(-100, 3, 100, 3),
      body(), body(), o).status,
    ContinuousCheckStatus::Unresolved);
}

TEST(ContinuousTrajectory, StaticSweepRejectsObstacleBetweenEndpoints)
{
  auto g = body();
  g.map.resolution = 0.1;
  g.map.origin_x = -1;
  g.map.origin_y = -1;
  g.map.width = 40;
  g.map.height = 20;
  g.map.cells.assign(800, 0);

  // 路径中部的障碍栅格。
  g.map.cells[10 * 40 + 20] = 100;

  const auto r = verifyContinuousTrajectory(
    straight(), peer(), g, body());

  EXPECT_TRUE(r.static_map_checked);
  EXPECT_EQ(
    r.status,
    ContinuousCheckStatus::CollisionOrStaticBlocked);
}

}  // namespace

namespace {

PlannerConfig demoGeometry()
{
  PlannerConfig g;
  g.map = makeUShapedDemoMap();
  g.max_iterations = 200000;
  g.allow_reverse = true;
  g.analytic_expansion = true;
  return g;
}

msg::PredictedTrajectory distantPeer()
{
  auto p = peer(-24, -24, -24, -24);
  p.points.back().time_from_start.sec = 300;
  return p;
}

ContinuousTrajectory cruiseThenStop(const std::vector<Pose> & path)
{
  std::vector<double> stops;
  double length = 0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (i > 1 && (path[i].signed_distance > 0) != (path[i - 1].signed_distance > 0)) {
      stops.push_back(length);
    }
    length += std::abs(path[i].signed_distance);
  }
  stops.push_back(length);
  // Deterministic timing fixture: accelerate/cruise/brake, stopping at every cusp.
  std::vector<STMotionPiece> pieces;
  double t = 0, s = 0, v = 0;
  const auto add = [&](double dt, double a) {
      pieces.push_back({t, dt, s, v, a, dt});
      s += v * dt + 0.5 * a * dt * dt;
      v = std::max(0.0, v + a * dt);
      t += dt;
    };
  for (double stop : stops) {
    const double distance = stop - s;
    const double ramp = std::min(1.0, std::sqrt(distance));
    add(ramp, 1);
    if (distance > 1.0) {add(distance - 1.0, 0);}
    add(ramp, -1);
  }
  return ContinuousTrajectory(path, header(), std::move(pieces));
}

STResult playbackCandidate(ContinuousTrajectory continuous,
  const PlannerConfig & geometry)
{
  STResult result;
  result.status = STStatus::Success;
  msg::PredictedTrajectory output;
  output.header = continuous.header();
  output.vehicle_id = "car_2";
  double previous_s = 0;
  for (double t = 0; ; t = std::min(t + 0.05, continuous.duration())) {
    const auto state = continuous.evaluate(t);
    const auto ns = std::llround(t * 1e9);
    auto p = point(0, state.pose.x, state.pose.y);
    p.time_from_start.sec = static_cast<std::int32_t>(ns / 1000000000);
    p.time_from_start.nanosec = static_cast<std::uint32_t>(ns % 1000000000);
    p.pose.orientation.z = std::sin(state.pose.yaw / 2);
    p.pose.orientation.w = std::cos(state.pose.yaw / 2);
    p.direction = t == 0 || state.s <= previous_s + 1e-12 ? 0 : state.gear;
    output.points.push_back(p);
    result.samples.push_back({ns, state.s, state.speed});
    previous_s = state.s;
    if (t == continuous.duration()) {break;}
  }
  result.trajectory = std::move(output);
  result.verification = detectTrajectoryCollision(*result.trajectory, distantPeer(), geometry, geometry);
  result.continuous_verification = verifyContinuousTrajectory(
    continuous, distantPeer(), geometry, geometry);
  result.continuous = std::move(continuous);
  return result;
}

TEST(DemoAfterAcceptance, BodySweepsWallBetweenFreeEndpointsBlocksAfter)
{
  const auto g = demoGeometry();
  HybridAStarPlanner planner(g);
  const std::vector<Pose> path{{0, 0, 0}, {24, 0, 0, 24, 0}};
  ASSERT_TRUE(planner.isPoseCollisionFree(path.front()));
  ASSERT_TRUE(planner.isPoseCollisionFree(path.back()));

  const auto candidate = playbackCandidate(cruiseThenStop(path), g);
  ASSERT_TRUE(candidate.continuous_verification->static_map_checked);
  EXPECT_EQ(candidate.continuous_verification->status,
    ContinuousCheckStatus::CollisionOrStaticBlocked);
  EXPECT_EQ(assessAfterPlayback(candidate), AfterPlaybackDecision::Blocked);

  // The real S-T entry must also reject it before expanding any timing nodes.
  const auto searched = planSTSpeed(path, distantPeer(), header(), "car_2", g, g);
  EXPECT_EQ(searched.status, STStatus::NoFeasiblePlan);
  EXPECT_EQ(searched.nodes_expanded, 0u);
  EXPECT_FALSE(searched.trajectory);
  EXPECT_EQ(assessAfterPlayback(searched), AfterPlaybackDecision::Blocked);
}

TEST(DemoAfterAcceptance, CompleteDetourAroundSameUWallsAllowsAfter)
{
  const auto g = demoGeometry();
  HybridAStarPlanner planner(g);
  PlanningStats stats;
  const auto path = planner.plan({0, 0, 0}, {24, 0, 0}, &stats);
  ASSERT_TRUE(stats.reached_goal);
  ASSERT_GT(path.size(), 2u);
  EXPECT_FALSE(g.map.isSegmentFree(path.front().x, path.front().y,
    path.back().x, path.back().y));

  const auto candidate = playbackCandidate(cruiseThenStop(path), g);
  ASSERT_EQ(candidate.continuous_verification->status, ContinuousCheckStatus::Safe);
  ASSERT_TRUE(candidate.continuous_verification->static_map_checked);
  EXPECT_EQ(assessAfterPlayback(candidate), AfterPlaybackDecision::StaticAndDynamicVerified);
  EXPECT_NEAR(candidate.continuous->evaluate(candidate.continuous->duration()).speed, 0, 1e-8);
}

TEST(DemoAfterAcceptance, MissingMapExplicitlyReportsStaticCollisionUnverified)
{
  auto g = demoGeometry();
  g.map = {};
  const auto candidate = playbackCandidate(
    cruiseThenStop({{0, 0, 0}, {24, 0, 0, 24, 0}}), g);
  ASSERT_EQ(candidate.continuous_verification->status, ContinuousCheckStatus::Safe);
  ASSERT_FALSE(candidate.continuous_verification->static_map_checked);
  EXPECT_EQ(assessAfterPlayback(candidate), AfterPlaybackDecision::DynamicOnly);
  EXPECT_NE(std::string(afterPlaybackMessage(assessAfterPlayback(candidate))).find(
    "静态碰撞未验证"), std::string::npos);
}

}  // namespace
