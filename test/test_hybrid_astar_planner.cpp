#include <gtest/gtest.h>

#include "car_swarm_agent/hybrid_astar_planner.hpp"

TEST(HybridAStarPlannerTest, GeneratesExpectedPath)
{
  car_swarm_agent::HybridAStarPlanner planner(4);

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