#include "car_swarm_agent/hybrid_astar_planner.hpp"
#include "car_swarm_agent/demo_map.hpp"
#include <cmath>
#include <iostream>

using namespace car_swarm_agent;

// Identical vehicle/search settings across modes; timings include table construction
// and output-path collision validation. Compile with -O2 for useful timing comparisons.
int main()
{
    std::cout << "scenario,mode,reached,length_m,points,expanded,time_ms,table_builds,body_safe,yaw_error\n";
    for (int scenario = 0; scenario < 3; ++scenario) {
        for (auto mode : {HeuristicMode::Euclidean, HeuristicMode::Obstacle, HeuristicMode::Dual}) {
            const bool u = scenario != 0;
            PlannerConfig config;
            config.max_iterations = u ? 200000 : 10000;
            config.heuristic_mode = mode;
            config.map.resolution = 0.5;
            config.map.origin_x = config.map.origin_y = u ? -30.0 : -10.0;
            config.map.width = config.map.height = u ? 120 : 60;
            config.map.cells.assign(config.map.width * config.map.height, 0);
            if (u) { addUShapedObstacle(config.map); }
            else { config.map.cells[17 * 60 + 27] = 100; }
            const Pose start = u ? Pose{0, 0, 0} : Pose{-4, -2, 0};
            const Pose goal = u ? Pose{scenario == 1 ? 18.0 : 24.0, 0, 0} : Pose{7, 2, 0};
            HybridAStarPlanner planner(config);
            PlanningStats stats;
            const auto path = planner.plan(start, goal, &stats);
            bool safe = !path.empty();
            for (const auto & pose : path) { safe &= planner.isPoseCollisionFree(pose); }
            for (std::size_t i = 1; i < path.size(); ++i) {
                safe &= planner.isMotionCollisionFree(path[i - 1], path[i]);
            }
            const double yaw_error = path.empty() ? -1.0 :
                std::abs(std::remainder(path.back().yaw - goal.yaw, 2.0 * std::acos(-1.0)));
            const char * name = mode == HeuristicMode::Euclidean ? "euclidean" :
                mode == HeuristicMode::Obstacle ? "obstacle" : "dual";
            std::cout << (u ? (scenario == 1 ? "u_original_goal" : "u_heading_goal") : "single")
                << ',' << name << ',' << stats.reached_goal << ',' << stats.path_length
                << ',' << path.size() << ',' << stats.expanded_nodes << ',' << stats.planning_time_ms
                << ',' << stats.obstacle_table_builds << ',' << safe << ',' << yaw_error << '\n';
            if (!path.empty() && (!safe || yaw_error > config.goal_yaw_tolerance)) { return 1; }
            if (scenario != 1 && path.empty()) { return 1; }
        }
    }
}
