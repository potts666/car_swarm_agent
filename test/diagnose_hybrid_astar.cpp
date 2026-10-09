#include "car_swarm_agent/hybrid_astar_planner.hpp"
#include "car_swarm_agent/demo_map.hpp"
#include <iostream>
#include <cmath>
#include <string>
using namespace car_swarm_agent;

int main(int argc, char ** argv)
{
  const bool rs = argc == 2 && std::string(argv[1]) == "--rs";
  if (argc > 1 && !rs) { std::cerr << "Usage: diagnose_hybrid_astar [--rs]\n"; return 2; }
  std::cout << "case,start_free,goal_free,stop,iterations,expanded,open,closest_x,closest_y,closest_yaw,position_error,yaw_error_deg,best_yaw_near_goal_deg,length_m,points,reverse_segments,analytic_attempts,analytic_successes,final_position_error,final_yaw_error_deg,time_ms\n";
  for (int scenario = 0; scenario < (rs ? 3 : 5); ++scenario) {
    PlannerConfig c;
    c.max_iterations = rs ? 200000 : 1000000;
    c.map.resolution = .5;
    c.map.origin_x = c.map.origin_y = -30;
    c.map.width = c.map.height = 120;
    c.map.cells.assign(120 * 120, 0);
    addUShapedObstacle(c.map);
    const char * name;
    if (rs) {
      const char * names[] = {"forward", "reverse_only", "reverse_rs"};
      name = names[scenario];
      c.allow_reverse = scenario != 0;
      c.analytic_expansion = scenario == 2;
    } else {
      const char * names[] = {"original", "half_step", "half_yaw_bin", "half_position_bin", "five_steerings"};
      name = names[scenario];
      if (scenario == 1) { c.step_size = .5; }
      if (scenario == 2) { c.yaw_resolution = .04363323129985824; }
      if (scenario == 3) { c.grid_resolution = .25; }
      if (scenario == 4) { c.steering_samples = 5; }
    }
    HybridAStarPlanner planner(c);
    PlanningStats s;
    const Pose goal{18, 0, 0};
    const auto path = planner.plan({0,0,0}, goal, &s);
    const double final_position = path.empty() ? -1.0 : std::hypot(path.back().x-goal.x, path.back().y-goal.y);
    const double final_yaw = path.empty() ? -1.0 : std::abs(std::remainder(path.back().yaw-goal.yaw, 2*std::acos(-1.0))) * 180 / std::acos(-1.0);
    bool safe = !path.empty();
    for (std::size_t i = 1; i < path.size(); ++i) { safe &= planner.isMotionCollisionFree(path[i-1], path[i]); }
    if (!path.empty() && !safe) { return 1; }
    std::cout << name << ',' << s.start_body_free << ',' << s.goal_body_free
      << ',' << stopReasonName(s.stop_reason) << ',' << s.iterations << ',' << s.expanded_nodes
      << ',' << s.open_nodes_remaining << ',' << s.closest_pose.x << ',' << s.closest_pose.y
      << ',' << s.closest_pose.yaw << ',' << s.closest_position_error
      << ',' << s.closest_yaw_error * 180 / std::acos(-1.0)
      << ',' << s.best_yaw_error_near_goal * 180 / std::acos(-1.0)
      << ',' << s.path_length << ',' << path.size() << ',' << s.reverse_segments
      << ',' << s.analytic_attempts << ',' << s.analytic_successes
      << ',' << final_position << ',' << final_yaw << ',' << s.planning_time_ms << std::endl;
  }
}
