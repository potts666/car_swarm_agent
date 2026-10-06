#include <gtest/gtest.h>
#include <cmath>
#include "car_swarm_agent/hybrid_astar_planner.hpp"
#include "car_swarm_agent/demo_map.hpp"
#include <algorithm>

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
  const car_swarm_agent::Pose goal{18.0, 0.0, 0.0};
  EXPECT_FALSE(config.map.isSegmentFree(start.x, start.y, goal.x, goal.y));
  car_swarm_agent::HybridAStarPlanner planner(config);
  const auto path = planner.plan(start, goal);
  ASSERT_GE(path.size(), 2u);
  EXPECT_DOUBLE_EQ(path.front().x, start.x);
  EXPECT_DOUBLE_EQ(path.front().y, start.y);
  EXPECT_LE(std::hypot(path.back().x - goal.x, path.back().y - goal.y),
    config.goal_tolerance);

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
