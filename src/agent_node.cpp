#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "car_swarm_agent/hybrid_astar_planner.hpp"

using namespace std::chrono_literals;

class CarSwarmAgent : public rclcpp::Node
{
public:
  CarSwarmAgent()
  : Node("car_swarm_agent"), planner_(10) 
  // 创建节点时，同时创建一个“分成 10 段”的规划器。
  // 节点只负责“何时调用规划器、如何展示或发布结果”；规划器负责“如何生成路径”
  {
    car_swarm_agent::Pose start{0.0, 0.0, 0.0};
    car_swarm_agent::Pose goal{5.0, 3.0, 0.0};

    const std::vector<car_swarm_agent::Pose> path =
      planner_.plan(start, goal);

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
      });
  }

private:
  car_swarm_agent::HybridAStarPlanner planner_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CarSwarmAgent>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}