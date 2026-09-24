#include <gtest/gtest.h>
#include <cmath>
#include "car_swarm_agent/hybrid_astar_planner.hpp"

TEST(HybridAStarPlannerTest, GeneratesExpectedPath)
{
  car_swarm_agent::HybridAStarPlanner planner(
    car_swarm_agent::PlannerConfig{4});// num_segments = 4
    //其他成员：使用它们各自的默认值

  const car_swarm_agent::Pose start{1.0, -2.0, 0.0};
  const car_swarm_agent::Pose goal{7.0, 2.0, 0.0};

  const auto path = planner.plan(start, goal);

  ASSERT_EQ(path.size(), 5u);

  EXPECT_DOUBLE_EQ(path.front().x, 1.0);
  EXPECT_DOUBLE_EQ(path.front().y, -2.0);

  EXPECT_DOUBLE_EQ(path[2].x, 4.0);
  EXPECT_DOUBLE_EQ(path[2].y, 0.0);

  EXPECT_DOUBLE_EQ(path.back().x, 7.0);
  EXPECT_DOUBLE_EQ(path.back().y, 2.0);
}

TEST(HybridAStarPlannerTest, ClampsZeroSegmentsToOne)
{
  car_swarm_agent::HybridAStarPlanner planner(
    car_swarm_agent::PlannerConfig{0});

  const car_swarm_agent::Pose start{0.0, 0.0, 0.0};
  const car_swarm_agent::Pose goal{2.0, 0.0, 0.0};

  const auto path = planner.plan(start, goal);

  ASSERT_EQ(path.size(), 2u);

  EXPECT_DOUBLE_EQ(path.front().x, 0.0);
  EXPECT_DOUBLE_EQ(path.back().x, 2.0);
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