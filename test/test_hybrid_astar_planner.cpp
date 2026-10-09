#include <gtest/gtest.h>
#include <cmath>
#include "car_swarm_agent/hybrid_astar_planner.hpp"
#include "car_swarm_agent/demo_map.hpp"
#include "car_swarm_agent/reeds_shepp.hpp"
#include <algorithm>

TEST(HybridAStarPlannerTest, ReachesGoalWithinTolerance)
{
  car_swarm_agent::PlannerConfig config;
  config.step_size = 1.0;
  config.wheel_base = 2.7;
  config.max_steer_angle = 0.5;
  config.steering_samples = 3;
  config.goal_tolerance = 0.5;
  config.max_iterations = 10000;

  car_swarm_agent::HybridAStarPlanner planner(config);

  const car_swarm_agent::Pose start{1.0, -2.0, 0.0};
  const car_swarm_agent::Pose goal{7.0, 2.0, 0.0};

  const std::vector<car_swarm_agent::Pose> path =
    planner.plan(start, goal);

  ASSERT_FALSE(path.empty());

  EXPECT_DOUBLE_EQ(path.front().x, start.x);
  EXPECT_DOUBLE_EQ(path.front().y, start.y);

  const car_swarm_agent::Pose & last_pose = path.back();

  const double distance_to_goal = std::hypot(
    last_pose.x - goal.x,
    last_pose.y - goal.y);

  EXPECT_LE(distance_to_goal, config.goal_tolerance);
  EXPECT_LE(std::abs(std::remainder(last_pose.yaw - goal.yaw, 2.0 * std::acos(-1.0))),
    config.goal_yaw_tolerance);
}


TEST(HybridAStarPlannerTest, PropagatesStraight)
{
  car_swarm_agent::PlannerConfig config;
  config.step_size = 1.0;
  config.wheel_base = 2.5;
  config.max_steer_angle = 0.5;

  car_swarm_agent::HybridAStarPlanner planner(config);

  const car_swarm_agent::Pose pose{0.0, 0.0, 0.0};
  const auto next = planner.propagate(pose, 0.0);

  EXPECT_NEAR(next.x, 1.0, 1e-9);
  EXPECT_NEAR(next.y, 0.0, 1e-9);
  EXPECT_NEAR(next.yaw, 0.0, 1e-9);
}

TEST(HybridAStarPlannerTest, LimitsSteeringAngle)
//第二个测试故意传入 1.0 弧度的过大转向角；因为配置最大允许 0.5，它验证 std::clamp() 确实限制了转向
{
  car_swarm_agent::PlannerConfig config;
  config.step_size = 1.0;
  config.wheel_base = 2.0;
  config.max_steer_angle = 0.5;

  car_swarm_agent::HybridAStarPlanner planner(config);

  const car_swarm_agent::Pose pose{0.0, 0.0, 0.0};
  const auto next = planner.propagate(pose, 1.0);

  const double expected_yaw = 0.5 * std::tan(0.5);

  EXPECT_NEAR(next.x, 1.0, 1e-9);
  EXPECT_NEAR(next.y, 0.0, 1e-9);
  EXPECT_NEAR(next.yaw, expected_yaw, 1e-9);
}

TEST(HybridAStarPlannerTest, GeneratesLeftStraightRightSuccessors)
{
  car_swarm_agent::PlannerConfig config;
  config.step_size = 1.0;
  config.wheel_base = 2.0;
  config.max_steer_angle = 0.5;
  config.steering_samples = 3;

  car_swarm_agent::HybridAStarPlanner planner(config);

  const car_swarm_agent::Pose pose{0.0, 0.0, 0.0};
  const auto successors = planner.generateSuccessors(pose);

  ASSERT_EQ(successors.size(), 3u);

  const double expected_yaw = 0.5 * std::tan(0.5);

  EXPECT_NEAR(successors[0].yaw, -expected_yaw, 1e-9);
  EXPECT_NEAR(successors[1].yaw, 0.0, 1e-9);
  EXPECT_NEAR(successors[2].yaw, expected_yaw, 1e-9);
}

TEST(SearchNodeTest, AddsActualAndEstimatedCosts)
{
  car_swarm_agent::SearchNode node{
    car_swarm_agent::Pose{1.0, 2.0, 0.0},
    2.5,
    1.5,
    0};

  EXPECT_DOUBLE_EQ(node.totalCost(), 4.0);
}

TEST(HybridAStarPlannerTest, RejectsStartBodyCollision)
{
  car_swarm_agent::PlannerConfig config;
  config.max_iterations = 1000;

  config.map.resolution = 1.0;
  config.map.origin_x = -5.0;
  config.map.origin_y = -5.0;
  config.map.width = 20;
  config.map.height = 20;
  config.map.cells.assign(20 * 20, 0);

  const car_swarm_agent::Pose start{1.0, -2.0, 0.0};
  const car_swarm_agent::Pose goal{7.0, 2.0, 0.0};

  // 无障碍时起终点车身均在地图内，排除边界造成的失败。
  car_swarm_agent::HybridAStarPlanner clear_planner(config);
  ASSERT_TRUE(clear_planner.isPoseCollisionFree(start));
  ASSERT_TRUE(clear_planner.isPoseCollisionFree(goal));

  // (2, -2) 对应第 3 行、第 7 列，障碍位于起点车身内。
  config.map.cells[3 * 20 + 7] = 100;

  car_swarm_agent::HybridAStarPlanner planner(config);

  EXPECT_FALSE(config.map.isOccupied(start.x, start.y));
  EXPECT_FALSE(planner.isPoseCollisionFree(start));
  EXPECT_TRUE(planner.isPoseCollisionFree(goal));
  EXPECT_TRUE(planner.plan(start, goal).empty());
}

TEST(OccupancyGridTest, DetectsCollisionBetweenFreeEndpoints)
{
  car_swarm_agent::OccupancyGrid map;
  map.resolution = 1.0;
  map.origin_x = 0.0;
  map.origin_y = -5.0;
  map.width = 20;
  map.height = 15;
  map.cells.assign(20 * 15, 0);

  // 障碍格：x 属于 [4, 5)，y 属于 [-1, 0)
  map.cells[4 * 20 + 4] = 1;

  const double x0 = 4.8180660823;
  const double y0 = -1.0116153540;
  const double x1 = 5.6394274335;
  const double y1 = -0.4412071398;

  EXPECT_FALSE(map.isOccupied(x0, y0));
  EXPECT_FALSE(map.isOccupied(x1, y1));
  EXPECT_FALSE(map.isSegmentFree(x0, y0, x1, y1));
}//两个端点都在空地，但连接它们的线段短暂穿过障碍格。之前的间隔取样可能漏掉它；新方法应检出

TEST(HybridAStarPlannerTest, ExitsUThroughOpeningAndReachesGoal)
{
  car_swarm_agent::PlannerConfig config;
  config.max_iterations = 200000;
  config.map.resolution = 0.5;
  config.map.origin_x = -30.0;
  config.map.origin_y = -30.0;
  config.map.width = 120;
  config.map.height = 120;
  config.map.cells.assign(120 * 120, 0);
  car_swarm_agent::addUShapedObstacle(config.map);

  const car_swarm_agent::Pose start{0.0, 0.0, 0.0};
  const car_swarm_agent::Pose goal{24.0, 0.0, 0.0};
  EXPECT_FALSE(config.map.isSegmentFree(start.x, start.y, goal.x, goal.y));
  car_swarm_agent::HybridAStarPlanner planner(config);
  const auto path = planner.plan(start, goal);
  ASSERT_GE(path.size(), 2u);
  EXPECT_DOUBLE_EQ(path.front().x, start.x);
  EXPECT_DOUBLE_EQ(path.front().y, start.y);
  EXPECT_LE(std::hypot(path.back().x - goal.x, path.back().y - goal.y),
    config.goal_tolerance);

  EXPECT_LE(std::abs(std::remainder(path.back().yaw - goal.yaw, 2.0 * std::acos(-1.0))),
    config.goal_yaw_tolerance);

  bool exited_opening = false;
  for (std::size_t i = 1; i < path.size(); ++i) {
    EXPECT_TRUE(planner.isMotionCollisionFree(path[i - 1], path[i]))
      << "Body collision on segment " << i;
    if (path[i].x < -10.0 && path[i].y > -11.5 && path[i].y < 12.0) {
      exited_opening = true;
    }
  }
  EXPECT_TRUE(exited_opening);
}

TEST(HybridAStarPlannerTest, ReachesGoalAroundObstacle)
{
  car_swarm_agent::PlannerConfig config;
  config.step_size = 1.0;
  config.wheel_base = 2.7;
  config.max_steer_angle = 0.5;
  config.steering_samples = 3;
  config.goal_tolerance = 0.5;
  config.max_iterations = 1000;
  config.grid_resolution = 0.5;
  config.yaw_resolution = 0.08726646259971647;

  config.map.resolution = 0.5;
  config.map.origin_x = -10.0;
  config.map.origin_y = -10.0;
  config.map.width = 60;
  config.map.height = 60;
  config.map.cells.assign(60 * 60, 0);

  // 保持障碍世界坐标 x=[3.5,4.0)、y=[-1.5,-1.0)。
  config.map.cells[17 * 60 + 27] = 100;

  car_swarm_agent::HybridAStarPlanner planner(config);
  const car_swarm_agent::Pose start{-4.0, -2.0, 0.0};
  const car_swarm_agent::Pose goal{7.0, 2.0, 0.0};

  const auto path = planner.plan(start, goal);

  ASSERT_GE(path.size(), 2u);

  EXPECT_LE(
    std::hypot(path.back().x - goal.x, path.back().y - goal.y),
    config.goal_tolerance);

  for (std::size_t i = 1; i < path.size(); ++i) {
    EXPECT_TRUE(planner.isMotionCollisionFree(
        path[i - 1], path[i]))
        << "Body collision on segment " << i;
  }
}

namespace {
//这叫“匿名命名空间”。

//它的作用是：

//把里面的东西放在一个“局部的、没有名字的命名空间”里
//这样这些函数/变量不会污染全局作用域
//也不会和其他文件里的同名符号冲突
  car_swarm_agent::PlannerConfig makeBodyTestConfig()
  {
    car_swarm_agent::PlannerConfig config;
    //“给后面的车辆碰撞测试准备一份默认配置。”
    config.max_iterations = 1000;
    config.vehicle_length = 4.5;
    config.vehicle_width = 1.8;
    config.rear_overhang = 1.0;
    config.collision_margin = 0.1;
    config.collision_check_step = 0.1;

    config.map.resolution = 0.5;
    config.map.origin_x = -10.0;
    config.map.origin_y = -10.0;
    config.map.width = 40;
    config.map.height = 40;
    config.map.cells.assign(40 * 40, 0);

    return config;
  }

  void occupyCell(
    car_swarm_agent::OccupancyGrid & map,
    double x,
    double y)
  {
    const int col = static_cast<int>(
      std::floor((x - map.origin_x) / map.resolution));

    const int row = static_cast<int>(
      std::floor((y - map.origin_y) / map.resolution));

    map.cells.at(
      static_cast<std::size_t>(row) * map.width + col) = 100;
  }

}  // namespace

TEST(VehicleCollisionTest, RejectsObstacleInsideBody)
//这个测试属于 VehicleCollisionTest 这个测试套件
//测试名称是 RejectsObstacleInsideBody
//中文可以理解为：
//“拒绝/识别车身内部有障碍物的情况”
{
    auto config = makeBodyTestConfig();

    // 障碍在车身内部；后轴中心和车身四角均不在此格。
    occupyCell(config.map, 2.1, 0.1);

    const car_swarm_agent::Pose pose{0.0, 0.0, 0.0};

    EXPECT_FALSE(config.map.isOccupied(pose.x, pose.y));

    car_swarm_agent::HybridAStarPlanner planner(config);
    EXPECT_FALSE(planner.isPoseCollisionFree(pose));
}

TEST(VehicleCollisionTest, RejectsSideCollision)
{
    auto config = makeBodyTestConfig();

    // y=[0.5, 1.0) 的障碍格与车身侧边相交。
    occupyCell(config.map, 1.1, 0.6);

    const car_swarm_agent::Pose pose{0.0, 0.0, 0.0};

    EXPECT_FALSE(config.map.isOccupied(pose.x, pose.y));
    //如果障碍物在车身内部，函数应该返回 false
    //测试期望它返回 false

    car_swarm_agent::HybridAStarPlanner planner(config);
    EXPECT_FALSE(planner.isPoseCollisionFree(pose));
}

TEST(VehicleCollisionTest, RejectsBodyOutsideMap)
{
    auto config = makeBodyTestConfig();
    car_swarm_agent::HybridAStarPlanner planner(config);

    // 后轴仍在地图内，但车头已越过 x=10。
    const car_swarm_agent::Pose pose{8.0, 0.0, 0.0};

    EXPECT_FALSE(config.map.isOccupied(pose.x, pose.y));
    EXPECT_FALSE(planner.isPoseCollisionFree(pose));
}

TEST(VehicleCollisionTest, DetectsCollisionDuringRotation)
//碰撞检查函数的几何测试，原地旋转不是当前车辆的搜索动作
{
    auto config = makeBodyTestConfig();

    // 横放、竖放均不碰撞，但旋转到约 45° 时车头扫过此格。
    occupyCell(config.map, 2.1, 2.1);

    car_swarm_agent::HybridAStarPlanner planner(config);

    const car_swarm_agent::Pose from{0.0, 0.0, 0.0};
    const car_swarm_agent::Pose to{
        0.0, 0.0, 1.5707963267948966};

    EXPECT_TRUE(planner.isPoseCollisionFree(from));
    EXPECT_TRUE(planner.isPoseCollisionFree(to));
    EXPECT_FALSE(planner.isMotionCollisionFree(from, to));
}

TEST(VehicleCollisionTest, DetectsObstacleBetweenSamples)
{
    auto config = makeBodyTestConfig();

    // 用小车和粗检查步长，专门验证保守膨胀能覆盖采样间隙。
    config.vehicle_length = 0.1;
    config.vehicle_width = 0.1;
    config.rear_overhang = 0.05;
    config.collision_margin = 0.0;
    config.collision_check_step = 1.0;

    config.map.resolution = 0.1;
    config.map.origin_x = -1.0;
    config.map.origin_y = -1.0;
    config.map.width = 40;
    config.map.height = 20;
    config.map.cells.assign(40 * 20, 0);

    occupyCell(config.map, 0.55, 0.05);

    car_swarm_agent::HybridAStarPlanner planner(config);

    const car_swarm_agent::Pose from{0.0, 0.05, 0.0};
    const car_swarm_agent::Pose to{1.0, 0.05, 0.0};

    EXPECT_TRUE(planner.isPoseCollisionFree(from));
    EXPECT_TRUE(planner.isPoseCollisionFree(to));
    EXPECT_FALSE(planner.isMotionCollisionFree(from, to));
}

TEST(VehicleCollisionTest, AcceptsClearMotion)
{
    const auto config = makeBodyTestConfig();
    car_swarm_agent::HybridAStarPlanner planner(config);

    EXPECT_TRUE(planner.isMotionCollisionFree(
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.2}));
}

TEST(HybridAStarHeuristicTest, ChecksHeadingAndWrapsAngles)
{
  car_swarm_agent::PlannerConfig config;
  config.max_iterations = 1;
  const double pi = std::acos(-1.0);
  for (auto mode : {car_swarm_agent::HeuristicMode::Euclidean,
      car_swarm_agent::HeuristicMode::Obstacle, car_swarm_agent::HeuristicMode::Dual}) {
    config.heuristic_mode = mode;
    car_swarm_agent::HybridAStarPlanner planner(config);
    EXPECT_TRUE(planner.plan({0, 0, 0}, {0, 0, pi}).empty());
    EXPECT_EQ(planner.plan({0, 0, pi - 0.01}, {0, 0, -pi + 0.01}).size(), 1u);
  }
}

TEST(HybridAStarHeuristicTest, BuildsOncePerPlanAndHandlesDisconnectedMap)
{
  auto config = makeBodyTestConfig();
  config.max_iterations = 10000;
  for (auto mode : {car_swarm_agent::HeuristicMode::Euclidean,
      car_swarm_agent::HeuristicMode::Obstacle, car_swarm_agent::HeuristicMode::Dual}) {
    config.heuristic_mode = mode;
    car_swarm_agent::HybridAStarPlanner planner(config);
    for (double x : {4.0, 5.0}) {
      car_swarm_agent::PlanningStats stats;
      ASSERT_FALSE(planner.plan({-5, 0, 0}, {x, 0, 0}, &stats).empty());
      EXPECT_TRUE(stats.reached_goal);
      EXPECT_DOUBLE_EQ(stats.path_length, x + 5.0);
      EXPECT_EQ(stats.obstacle_table_builds, mode == car_swarm_agent::HeuristicMode::Euclidean ? 0u : 1u);
      EXPECT_GT(stats.expanded_nodes, 1u);
      EXPECT_GE(stats.planning_time_ms, 0.0);
    }
  }
  for (int row = 0; row < config.map.height; ++row) {
    config.map.cells[row * config.map.width + 20] = 100;
  }
  car_swarm_agent::HybridAStarPlanner blocked(config);
  car_swarm_agent::PlanningStats stats;
  EXPECT_TRUE(blocked.plan({-5, 0, 0}, {5, 0, 0}, &stats).empty());
  EXPECT_FALSE(stats.reached_goal);
  EXPECT_EQ(stats.obstacle_table_builds, 1u);
}

TEST(HybridAStarHeuristicTest, ZeroSteeringCannotMoveSidewaysOrChangeHeading)
{
  car_swarm_agent::PlannerConfig config;
  config.max_steer_angle = 0.0;
  config.max_iterations = 100;
  car_swarm_agent::HybridAStarPlanner planner(config);
  EXPECT_FALSE(planner.plan({0, 0, 0}, {5, 0, 0}).empty());
  EXPECT_TRUE(planner.plan({0, 0, 0}, {5, 2, 0}).empty());
  EXPECT_TRUE(planner.plan({0, 0, 0}, {5, 0, 1}).empty());
}

TEST(HybridAStarHeuristicTest, PreservesOriginalUGoalWithRelaxedHeading)
{
  car_swarm_agent::PlannerConfig config;
  config.max_iterations = 200000;
  config.goal_yaw_tolerance = std::acos(-1.0);
  config.heuristic_mode = car_swarm_agent::HeuristicMode::Euclidean;
  config.map.resolution = 0.5;
  config.map.origin_x = config.map.origin_y = -30.0;
  config.map.width = config.map.height = 120;
  config.map.cells.assign(120 * 120, 0);
  car_swarm_agent::addUShapedObstacle(config.map);
  car_swarm_agent::HybridAStarPlanner planner(config);
  const auto path = planner.plan({0, 0, 0}, {18, 0, 0});
  ASSERT_FALSE(path.empty());
  for (std::size_t i = 1; i < path.size(); ++i) {
    EXPECT_TRUE(planner.isMotionCollisionFree(path[i - 1], path[i]));
  }
}

TEST(HybridAStarDiagnosticTest, ReportsStopReasons)
{
  car_swarm_agent::PlannerConfig c;
  c.max_iterations = 1;
  car_swarm_agent::PlanningStats s;
  car_swarm_agent::HybridAStarPlanner limited(c);
  EXPECT_TRUE(limited.plan({0,0,0}, {10,0,0}, &s).empty());
  EXPECT_EQ(s.stop_reason, car_swarm_agent::StopReason::IterationLimit);
  EXPECT_EQ(s.iterations, 1u);
  EXPECT_GT(s.open_nodes_remaining, 0u);
  EXPECT_DOUBLE_EQ(s.closest_position_error, 9.0);
  auto blocked = makeBodyTestConfig();
  blocked.map.cells[20 * blocked.map.width + 20] = 100;
  car_swarm_agent::HybridAStarPlanner invalid(blocked);
  EXPECT_TRUE(invalid.plan({-5,0,0}, {0,0,0}, &s).empty());
  EXPECT_EQ(s.stop_reason, car_swarm_agent::StopReason::InvalidGoal);
  EXPECT_FALSE(s.goal_body_free);
  EXPECT_TRUE(s.start_body_free);
  EXPECT_TRUE(invalid.plan({0,0,0}, {-5,0,0}, &s).empty());
  EXPECT_EQ(s.stop_reason, car_swarm_agent::StopReason::InvalidStart);
  c.allow_reverse = true;
  c.max_steer_angle = 0;
  c.max_iterations = 1000;
  auto finite = makeBodyTestConfig();
  c.map = finite.map;
  car_swarm_agent::HybridAStarPlanner exhausted(c);
  EXPECT_TRUE(exhausted.plan({-5,0,0}, {0,4,0}, &s).empty());
  EXPECT_EQ(s.stop_reason, car_swarm_agent::StopReason::OpenExhausted);
  EXPECT_EQ(s.open_nodes_remaining, 0u);
}

TEST(ReedsSheppPlannerTest, GeneratesSignedReverseArcs)
{
  car_swarm_agent::PlannerConfig c;
  c.allow_reverse = true;
  car_swarm_agent::HybridAStarPlanner planner(c);
  const auto reverse = planner.propagate({0,0,0}, 0.5, -1);
  const double k = std::tan(0.5) / c.wheel_base;
  EXPECT_NEAR(reverse.x, -std::sin(k) / k, 1e-10);
  EXPECT_NEAR(reverse.y, (1-std::cos(k)) / k, 1e-10);
  EXPECT_NEAR(reverse.yaw, -k, 1e-10);
  EXPECT_DOUBLE_EQ(reverse.signed_distance, -1.0);
  EXPECT_EQ(planner.generateSuccessors({0,0,0}).size(), 6u);
}

TEST(ReedsSheppPlannerTest, ConnectsReverseStraightWithActualLength)
{
  car_swarm_agent::PlannerConfig c;
  c.allow_reverse = true;
  c.analytic_expansion = true;
  c.max_iterations = 1;
  car_swarm_agent::HybridAStarPlanner planner(c);
  car_swarm_agent::PlanningStats s;
  auto path = planner.plan({0,0,0}, {-5,0,0}, &s);
  ASSERT_FALSE(path.empty());
  EXPECT_EQ(s.stop_reason, car_swarm_agent::StopReason::GoalReached);
  EXPECT_EQ(s.analytic_successes, 1u);
  EXPECT_NEAR(s.path_length, 5.0, 1e-9);
  EXPECT_GT(s.reverse_segments, 0u);
  EXPECT_EQ(s.path_points, path.size());
  EXPECT_NEAR(path.back().x, -5.0, 1e-9);
  double sum = 0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    EXPECT_LT(path[i].signed_distance, 0);
    sum += std::abs(path[i].signed_distance);
  }
  EXPECT_NEAR(sum, s.path_length, 1e-9);
}

TEST(ReedsSheppPlannerTest, RejectsAnalyticConnectionThroughWall)
{
  auto c = makeBodyTestConfig();
  c.allow_reverse = true;
  c.analytic_expansion = true;
  c.max_iterations = 1;
  for (int row = 0; row < c.map.height; ++row) { c.map.cells[row * c.map.width + 20] = 100; }
  car_swarm_agent::HybridAStarPlanner planner(c);
  car_swarm_agent::PlanningStats s;
  EXPECT_TRUE(planner.plan({-5,0,0}, {5,0,0}, &s).empty());
  EXPECT_EQ(s.analytic_attempts, 1u);
  EXPECT_EQ(s.analytic_successes, 0u);
  EXPECT_EQ(s.stop_reason, car_swarm_agent::StopReason::IterationLimit);
}

TEST(ReedsSheppPlannerTest, ReachesOriginalUGoalWithHeadingAndBodySafety)
{
  car_swarm_agent::PlannerConfig c;
  c.allow_reverse = true;
  c.analytic_expansion = true;
  c.max_iterations = 200000;
  c.map.resolution = .5;
  c.map.origin_x = c.map.origin_y = -30;
  c.map.width = c.map.height = 120;
  c.map.cells.assign(120 * 120, 0);
  car_swarm_agent::addUShapedObstacle(c.map);
  car_swarm_agent::HybridAStarPlanner planner(c);
  car_swarm_agent::PlanningStats s;
  const car_swarm_agent::Pose goal{18,0,0};
  ASSERT_TRUE(planner.isPoseCollisionFree(goal));
  auto path = planner.plan({0,0,0}, goal, &s);
  ASSERT_FALSE(path.empty());
  EXPECT_EQ(s.stop_reason, car_swarm_agent::StopReason::GoalReached);
  EXPECT_EQ(s.analytic_successes, 1u);
  EXPECT_GT(s.reverse_segments, 0u);
  EXPECT_NEAR(path.back().x, goal.x, 1e-6);
  EXPECT_NEAR(path.back().y, goal.y, 1e-6);
  EXPECT_NEAR(std::remainder(path.back().yaw-goal.yaw, 2*std::acos(-1.0)), 0, 1e-6);
  double length = 0;
  bool exited = false;
  for (std::size_t i = 1; i < path.size(); ++i) {
    EXPECT_TRUE(planner.isPoseCollisionFree(path[i]));
    EXPECT_TRUE(planner.isMotionCollisionFree(path[i-1], path[i]));
    const double ds = path[i].signed_distance;
    ASSERT_NE(ds, 0.0);
    const double k = std::remainder(path[i].yaw-path[i-1].yaw, 2*std::acos(-1.0)) / ds;
    EXPECT_NEAR(k, path[i].curvature, 1e-9);
    EXPECT_TRUE(planner.isArcCollisionFree(path[i-1], ds, path[i].curvature));
    EXPECT_LE(std::abs(k), std::tan(c.max_steer_angle)/c.wheel_base + 1e-9);
    if (path[i].x < -10) { exited = true; }
    length += std::abs(ds);
  }
  EXPECT_TRUE(exited);
  EXPECT_NEAR(length, s.path_length, 1e-8);
}

TEST(ReedsSheppGeometryTest, SignedSegmentsReconstructEndpointsAndSymmetricLengths)
{
  using Geometry = car_swarm_agent::detail::ReedsSheppStateSpace;
  const double radius = 5.0;
  for (int i = 0; i < 300; ++i) {
    const car_swarm_agent::Pose from{std::sin(i)*3, std::cos(i)*2, std::sin(i*2)*3};
    const car_swarm_agent::Pose goal{std::cos(i*3)*12, std::sin(i*4)*12, std::cos(i*5)*3};
    const auto solution = car_swarm_agent::detail::shortestReedsShepp(from, goal, radius);
    const auto reversed = car_swarm_agent::detail::shortestReedsShepp(goal, from, radius);
    EXPECT_NEAR(solution.length(), reversed.length(), 1e-9);
    auto current = from;
    for (int segment = 0; segment < 5; ++segment) {
      const double k = solution.type_[segment] == Geometry::RS_LEFT ? 1/radius :
        solution.type_[segment] == Geometry::RS_RIGHT ? -1/radius : 0;
      current = car_swarm_agent::detail::integrateArc(current, solution.length_[segment]*radius, k);
    }
    EXPECT_NEAR(current.x, goal.x, 1e-8);
    EXPECT_NEAR(current.y, goal.y, 1e-8);
    EXPECT_NEAR(std::remainder(current.yaw-goal.yaw, 2*std::acos(-1.0)), 0, 1e-8);
  }
}

TEST(ReedsSheppPlannerTest, ArcSweepDetectsCollisionWithFreeEndpoints)
{
  auto c = makeBodyTestConfig();
  c.vehicle_length = c.vehicle_width = 0.1;
  c.rear_overhang = 0.05;
  c.collision_margin = 0;
  occupyCell(c.map, 3.6, 1.4);
  car_swarm_agent::HybridAStarPlanner planner(c);
  const car_swarm_agent::Pose start{0,0,0};
  const double length = 5.0 * std::acos(-1.0) / 2;
  const auto end = car_swarm_agent::detail::integrateArc(start, length, 0.2);
  ASSERT_TRUE(planner.isPoseCollisionFree(start));
  ASSERT_TRUE(planner.isPoseCollisionFree(end));
  EXPECT_FALSE(planner.isArcCollisionFree(start, length, 0.2));
}

TEST(ReedsSheppPlannerTest, InPlaceHeadingConnectionHasCuspsAndNonzeroLength)
{
  car_swarm_agent::PlannerConfig c;
  c.allow_reverse = c.analytic_expansion = true;
  c.max_iterations = 1;
  car_swarm_agent::HybridAStarPlanner planner(c);
  car_swarm_agent::PlanningStats s;
  auto path = planner.plan({0,0,0}, {0,0,std::acos(-1.0)}, &s);
  ASSERT_FALSE(path.empty());
  EXPECT_GT(s.path_length, 0);
  EXPECT_GT(s.reverse_segments, 0u);
  bool forward = false, reverse = false;
  for (const auto & pose : path) {
    forward |= pose.signed_distance > 0;
    reverse |= pose.signed_distance < 0;
  }
  EXPECT_TRUE(forward);
  EXPECT_TRUE(reverse);
  EXPECT_NEAR(path.back().x, 0, 1e-8);
  EXPECT_NEAR(path.back().y, 0, 1e-8);
}
