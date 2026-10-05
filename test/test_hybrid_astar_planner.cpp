#include <gtest/gtest.h>
#include <cmath>
#include "car_swarm_agent/hybrid_astar_planner.hpp"

TEST(HybridAStarPlannerTest, ReachesGoalWithinTolerance)
{
  car_swarm_agent::PlannerConfig config;
  config.step_size = 1.0;
  config.wheel_base = 2.7;
  config.max_steer_angle = 0.5;
  config.steering_samples = 3;
  config.goal_tolerance = 0.5;
  config.max_iterations = 1000;

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

TEST(HybridAStarPlannerTest, RejectsBlockedFirstStep)
{
  car_swarm_agent::PlannerConfig config;

  config.map.resolution = 1.0;
  config.map.origin_x = 0.0;
  config.map.origin_y = -3.0;
  config.map.width = 10;
  config.map.height = 10;
  config.map.cells.assign(100, 0);

  // (2, -2) 对应第 1 行、第 2 列
  config.map.cells[1 * 10 + 2] = 1;

  car_swarm_agent::HybridAStarPlanner planner(config);

  const car_swarm_agent::Pose start{1.0, -2.0, 0.0};
  const car_swarm_agent::Pose goal{7.0, 2.0, 0.0};

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
  config.map.origin_x = 0.0;
  config.map.origin_y = -5.0;
  config.map.width = 40;
  config.map.height = 30;
  config.map.cells.assign(40 * 30, 0);

  // 第 7 行、第 7 列：x=[3.5,4.0), y=[-1.5,-1.0)
  config.map.cells[7 * 40 + 7] = 1;

  car_swarm_agent::HybridAStarPlanner planner(config);
  const car_swarm_agent::Pose start{1.0, -2.0, 0.0};
  const car_swarm_agent::Pose goal{7.0, 2.0, 0.0};

  const auto path = planner.plan(start, goal);

  ASSERT_GE(path.size(), 2u);

  EXPECT_LE(
    std::hypot(path.back().x - goal.x, path.back().y - goal.y),
    config.goal_tolerance);

  for (std::size_t i = 1; i < path.size(); ++i) {
    EXPECT_TRUE(config.map.isSegmentFree(
      path[i - 1].x, path[i - 1].y,
      path[i].x, path[i].y)) << "Collision on segment " << i;
  }
}