// Reproducible numerical experiment; no ROS node, publication or playback.
#include <array>
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include "car_swarm_agent/demo_map.hpp"
#include "car_swarm_agent/quintic_trajectory.hpp"
#include "car_swarm_agent/trajectory_yield.hpp"

using namespace car_swarm_agent;
namespace {
msg::TrajectoryPoint point(int seconds, double x, double y, double yaw)
{
  msg::TrajectoryPoint p;
  p.time_from_start.sec = seconds; p.pose.position.x = x; p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2); p.pose.orientation.w = std::cos(yaw / 2);
  p.direction = seconds ? p.FORWARD : p.STOP; return p;
}
template<class Reference>
void writePlan(const std::string & file, const Reference & plan)
{
  std::ofstream out(file); out << std::setprecision(17);
  out << "t,x,y,yaw,vx,vy,ax,ay,gear,speed,curvature\n";
  const int n = static_cast<int>(std::ceil(plan.duration() / 0.01));
  for (int i = 0; i <= n; ++i) {
    double t = std::min(i * 0.01, plan.duration()); const auto r = plan.evaluate(t);
    out << t << ',' << r.pose.x << ',' << r.pose.y << ',' << r.pose.yaw << ',' <<
      r.vx << ',' << r.vy << ',' << r.ax << ',' << r.ay << ',' << r.gear << ',' <<
      r.speed << ',' << r.curvature << '\n';
  }
}
void writeActual(const std::string & prefix, const TrackingSimulation & simulation,
  const PlannerConfig & g)
{
  std::ofstream out(prefix + ".csv"); out << std::setprecision(17);
  out << "t,x,y,yaw,speed,steering,acceleration,position_error\n";
  for (const auto & s : simulation.samples) {
    out << s.time << ',' << s.actual.pose.x << ',' << s.actual.pose.y << ',' <<
      s.actual.pose.yaw << ',' << s.actual.signed_speed << ',' << s.actual.steering << ',' <<
      s.command.acceleration << ',' << s.error.position << '\n';
  }
  std::ofstream metrics(prefix + ".json");
  metrics << std::setprecision(17) << "{\"rms_position\":" << simulation.rms_position <<
    ",\"max_position\":" << simulation.max_position << ",\"stopped\":" <<
    (simulation.stopped ? "true" : "false") << ",\"continuous_actual_safe\":" <<
    (allowTrackingPlayback(simulation) ? "true" : "false") <<
    ",\"actual_verification_detail\":" << std::quoted(simulation.actual_verification ?
      simulation.actual_verification->detail : "No stopped actual trajectory") <<
    ",\"wheel_base\":" << g.wheel_base << "}\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    if (argc < 3) {throw std::invalid_argument("Usage: compare_spatial_temporal --export DIR [reverse] | --evaluate DIR");}
    const std::string mode = argv[1], dir = argv[2];
    std::filesystem::create_directories(dir);
    PlannerConfig g; g.map = makeUShapedDemoMap(); g.max_iterations = 200000;
    g.allow_reverse = g.analytic_expansion = true; g.step_size = 0.5;
    g.goal_tolerance = g.goal_yaw_tolerance = 1e-6;
    std_msgs::msg::Header header; header.frame_id = "map"; header.stamp.sec = 100;
    msg::PredictedTrajectory peer; peer.header = header; peer.vehicle_id = "car_1";
    peer.points = {point(0, -10, 0, 0), point(10, 0, 0, 0),
      point(20, 10, 0, 0), point(120, 10, 0, 0)};
    TrackingOptions tracking; tracking.max_speed = 1;
    if (mode == "--export") {
      bool reverse = argc > 3 && std::string(argv[3]) == "reverse";
      HybridAStarPlanner planner(g);
      auto path = reverse ? planner.plan({0, -8, 1.5707963267948966},
        {0, -4, -1.5707963267948966}) : planner.plan({0, -8, 1.5707963267948966},
        {0, 8, 1.5707963267948966});
      const bool gear_smoke = argc > 3 && std::string(argv[3]) == "gear-smoke";
      if (gear_smoke) {
        // Deterministic forward/reverse/forward regression from three native
        // Hybrid A* legs. Preserve incoming arc metadata at both cusp points.
        path = planner.plan({0, -8, 1.5707963267948966}, {0, -2, 1.5707963267948966});
        for (auto endpoints : {std::array<double, 2>{-2, -4}, std::array<double, 2>{-4, 8}}) {
          const auto leg = planner.plan({0, endpoints[0], 1.5707963267948966},
            {0, endpoints[1], 1.5707963267948966});
          if (leg.size() < 2) {throw std::runtime_error("Gear regression leg failed");}
          path.insert(path.end(), leg.begin() + 1, leg.end());
        }
      }
      STOptions options; options.max_speed = 1; options.initial_speed = reverse || gear_smoke ? 0 : 1;
      options.max_steps = 200; options.max_nodes = 200000;
      const auto seed = planSTSpeed(path, peer, header, "car_2", g, g, options);
      if (seed.status != STStatus::Success || !seed.continuous) {
        throw std::runtime_error("Hybrid A* / S-T seed failed");
      }
      const auto & plan = *seed.continuous;
      writePlan(dir + "/before.csv", plan);
      std::ofstream segments(dir + "/segments.csv"); segments << std::setprecision(17);
      segments << "start,end,gear\n";
      // Retain every stop/hold/gear boundary, coalesce consecutive same-gear motion.
      double start = 0, end = 0; int previous = 9;
      for (const auto & p : plan.pieces()) {
        const double moving_end = p.t0 + p.moving_time;
        for (int part = 0; part < 2; ++part) {
          double a = part == 0 ? p.t0 : moving_end;
          double b = part == 0 ? moving_end : p.t0 + p.duration;
          if (b - a < 1e-9) {continue;}
          int gear = part == 1 || (p.v0 < 1e-8 && std::abs(p.acceleration) < 1e-8) ?
            0 : plan.evaluate((a + b) / 2).gear;
          const bool stopped = plan.evaluate(a).speed < 1e-8;
          if (gear != previous || (gear != 0 && stopped && a > start + 1e-8)) {
            if (previous != 9) {segments << start << ',' << end << ',' << previous << '\n';}
            start = a; previous = gear;
          }
          end = b;
        }
      }
      segments << start << ',' << end << ',' << previous << '\n';
      std::ofstream map(dir + "/obstacles.csv"); map << "x,y,width,height\n";
      for (int row = 0; row < g.map.height; ++row) {
        for (int col = 0; col < g.map.width; ++col) {
          if (g.map.cells[row * g.map.width + col] > 0) {
            map << g.map.origin_x + col * g.map.resolution << ',' <<
              g.map.origin_y + row * g.map.resolution << ",0.5,0.5\n";
          }
        }
      }
      std::ofstream neighbor(dir + "/peer.csv"); neighbor << "t,x,y,yaw\n";
      for (int t : {0, 10, 20, 120}) {
        auto p = interpolateTrajectoryPose(peer, 100000000000LL + t * 1000000000LL);
        neighbor << t << ',' << p->x << ',' << p->y << ',' << p->yaw << '\n';
      }
      writeActual(dir + "/before_actual", simulateTracking(plan, peer, g, g, tracking), g);
      std::cout << "Exported Hybrid A* + S-T seed, " << plan.duration() << " seconds\n";
    } else if (mode == "--evaluate") {
      std::ifstream input(dir + "/coefficients.txt");
      std::vector<QuinticPiece> pieces; QuinticPiece p;
      std::string line;
      while (std::getline(input, line)) {
        if (line.empty()) {continue;}
        std::istringstream row(line);
        if (!(row >> p.duration >> p.gear >> p.stopped_yaw)) {
          throw std::runtime_error("Bad polynomial metadata");
        }
        for (auto & x : p.x) {if (!(row >> x)) {throw std::runtime_error("Bad x coefficients");}}
        for (auto & y : p.y) {if (!(row >> y)) {throw std::runtime_error("Bad y coefficients");}}
        std::string extra;
        if (row >> extra) {throw std::runtime_error("Unexpected coefficient fields");}
        pieces.push_back(p);
      }
      const QuinticTrajectory plan(pieces, header);
      writePlan(dir + "/after.csv", plan);
      writeActual(dir + "/after_actual", simulateTrackingExperiment(plan, peer, g, g, tracking), g);
      std::cout << "Evaluated quintic plan, " << plan.duration() << " seconds\n";
    } else {throw std::invalid_argument("Unknown mode");}
    return 0;
  } catch (const std::exception & e) {std::cerr << e.what() << '\n'; return 1;}
}
