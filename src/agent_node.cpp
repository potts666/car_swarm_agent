#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "car_swarm_agent/hybrid_astar_planner.hpp"

using namespace std::chrono_literals;

class CarSwarmAgent : public rclcpp::Node
{
public:
  CarSwarmAgent()
  : Node("car_swarm_agent"), planner_()
  {
    car_swarm_agent::PlannerConfig config;

    config.step_size =
      declare_parameter<double>("step_size", 1.0);
    config.wheel_base =
      declare_parameter<double>("wheel_base", 2.7);
    config.max_steer_angle =
      declare_parameter<double>("max_steer_angle", 0.5);
    config.steering_samples =
      static_cast<int>(declare_parameter<int>("steering_samples", 3));
    config.goal_tolerance =
      declare_parameter<double>("goal_tolerance", 0.5);
    config.max_iterations =
      static_cast<int>(declare_parameter<int>("max_iterations", 1000));
    config.grid_resolution =
      declare_parameter<double>("grid_resolution", 0.5);
    config.yaw_resolution =
      declare_parameter<double>("yaw_resolution", 0.08726646259971647);

    config.map.resolution =
      declare_parameter<double>("map_resolution", 0.5);
    config.map.origin_x =
      declare_parameter<double>("map_origin_x", 0.0);
    config.map.origin_y =
      declare_parameter<double>("map_origin_y", -5.0);
    config.map.width =
      static_cast<int>(declare_parameter<int>("map_width", 40));
    config.map.height =
      static_cast<int>(declare_parameter<int>("map_height", 30));

    if (!std::isfinite(config.map.resolution) ||
        config.map.resolution <= 0.0 ||
        config.map.width <= 0 || config.map.width > 1000 ||
        config.map.height <= 0 || config.map.height > 1000) {
      throw std::invalid_argument("Invalid demo map configuration");
    }

    config.map.cells.assign(
      static_cast<std::size_t>(config.map.width) *
        static_cast<std::size_t>(config.map.height),
      0);

    const bool obstacle_enabled =
      declare_parameter<bool>("obstacle_enabled", true);
    const int obstacle_col =
      static_cast<int>(declare_parameter<int>("obstacle_col", 7));
    const int obstacle_row =
      static_cast<int>(declare_parameter<int>("obstacle_row", 7));

    if (obstacle_enabled) {
      if (obstacle_col < 0 || obstacle_col >= config.map.width ||
          obstacle_row < 0 || obstacle_row >= config.map.height) {
        throw std::invalid_argument("Obstacle cell is outside demo map");
      }

      const auto index =
        static_cast<std::size_t>(obstacle_row) *
          static_cast<std::size_t>(config.map.width) +
        static_cast<std::size_t>(obstacle_col);

      config.map.cells[index] = 100;
    }

    planner_ = car_swarm_agent::HybridAStarPlanner(config);

    const double start_x =
      declare_parameter<double>("start_x", 0.0);
    const double start_y =
      declare_parameter<double>("start_y", 0.0);
    const double goal_x =
      declare_parameter<double>("goal_x", 5.0);
    const double goal_y =
      declare_parameter<double>("goal_y", 3.0);

    const car_swarm_agent::Pose start{start_x, start_y, 0.0};
    const car_swarm_agent::Pose goal{goal_x, goal_y, 0.0};
    const std::vector<car_swarm_agent::Pose> path =
      planner_.plan(start, goal);

    map_publisher_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>("demo_map", 10);

    map_message_.header.frame_id = "map";
    map_message_.info.resolution =
      static_cast<float>(config.map.resolution);
    map_message_.info.width =
      static_cast<std::uint32_t>(config.map.width);
    map_message_.info.height =
      static_cast<std::uint32_t>(config.map.height);
    map_message_.info.origin.position.x = config.map.origin_x;
    map_message_.info.origin.position.y = config.map.origin_y;
    map_message_.info.origin.orientation.w = 1.0;

    map_message_.data.assign(
      config.map.cells.begin(), config.map.cells.end());//地图发布器

    path_publisher_ =
    create_publisher<nav_msgs::msg::Path>("planned_path", 10);

    path_message_.header.frame_id = "map";

    for (const auto & planner_pose : path) {
      geometry_msgs::msg::PoseStamped path_pose;

      path_pose.header.frame_id = "map";
      path_pose.pose.position.x = planner_pose.x;
      path_pose.pose.position.y = planner_pose.y;
      path_pose.pose.position.z = 0.0;

      path_pose.pose.orientation.z =
      std::sin(planner_pose.yaw / 2.0);
      path_pose.pose.orientation.w =
      std::cos(planner_pose.yaw / 2.0);

      path_message_.poses.push_back(path_pose);
    }

    RCLCPP_INFO(
      get_logger(),
      "Planner generated %zu path points.",
      path.size());

    for (std::size_t i = 0; i < path.size(); ++i) {
      const auto & pose = path[i];

      RCLCPP_INFO(
        get_logger(),
        "point %zu: x=%.2f, y=%.2f, yaw=%.2f",
        i, pose.x, pose.y, pose.yaw);
    }

    publisher_ =
      create_publisher<std_msgs::msg::String>("agent_status", 10);

    timer_ = create_wall_timer(
      1s,
      [this]() {
        auto message = std_msgs::msg::String();
        message.data = "agent is working";
        publisher_->publish(message);
        publishPath();
      });
  }

private:
  void publishPath()
  {
    const auto now = get_clock()->now();

    path_message_.header.stamp = now;

    for (auto & path_pose : path_message_.poses) {
      path_pose.header.stamp = now;
    }

    map_message_.header.stamp = now;
    map_publisher_->publish(map_message_);
    path_publisher_->publish(path_message_);
  }

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  nav_msgs::msg::Path path_message_;
  car_swarm_agent::HybridAStarPlanner planner_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  nav_msgs::msg::OccupancyGrid map_message_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CarSwarmAgent>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
