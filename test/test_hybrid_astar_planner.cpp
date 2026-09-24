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