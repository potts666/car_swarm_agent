/*新增通信节点：发布自己，接收邻车，再显示收到的数据
刻意让车辆停在起点，每秒重新发布同一个测试计划，并更新预测基准时间。它验证的是轨迹消息通信，不代表车辆已经执行计划。
前进和倒车重叠的直线路径，在 RViz 中看起来是一条线；方向字段需要用 topic echo 检查，不能靠画面判断。*/
#include <chrono>
#include <memory>
#include <string>
#include "tf2_ros/static_transform_broadcaster.h"
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include "car_swarm_agent/demo_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "car_swarm_agent/hybrid_astar_planner.hpp"
#include "car_swarm_agent/msg/predicted_trajectory.hpp"
#include "car_swarm_agent/msg/trajectory_point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class TrajectoryCommNode : public rclcpp::Node
{
public:
  using Trajectory = car_swarm_agent::msg::PredictedTrajectory;
  using Point = car_swarm_agent::msg::TrajectoryPoint;
  TrajectoryCommNode()
  : Node("trajectory_comm")
  {
    vehicle_id_ = declare_parameter<std::string>("vehicle_id", "car_1");
    neighbor_topic_ = declare_parameter<std::string>(
      "neighbor_topic", "/car_2/predicted_trajectory");
    lane_y_ = declare_parameter<double>("lane_y", 0.0);
    buildPlannedTrajectory();
    // 相对话题名：由 launch 中的 namespace 区分两辆车。
    const auto qos = rclcpp::QoS(10).reliable();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "demo_map", qos);
    trajectory_pub_ = create_publisher<Trajectory>(
      "predicted_trajectory", qos);

    own_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "trajectory_path", qos);

    neighbor_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "received_neighbor_path", qos);
//发布器负责发布本车轨迹、供显示的本车路径，以及转换后的邻车路径

    // 邻车话题使用参数指定的绝对名称。
    neighbor_sub_ = create_subscription<Trajectory>(
      neighbor_topic_, qos,
      [this](Trajectory::ConstSharedPtr msg) {
        if (msg->vehicle_id == vehicle_id_) {
          return;
        }
//create_subscription() 返回的是订阅者的共享指针，不是收到的路径；收到的路径通过回调参数 message 交给你。
        RCLCPP_INFO(
          get_logger(),
          "received vehicle=%s stamp=%d.%09u points=%zu frame=%s",
          msg->vehicle_id.c_str(),
          static_cast<int>(msg->header.stamp.sec),
          static_cast<unsigned int>(msg->header.stamp.nanosec),
          msg->points.size(),
          msg->header.frame_id.c_str());
//           ROS 2 中用于记录普通信息日志的 C++ 宏。它的第一个参数应是 rclcpp::Logger 类型的日志器，后面依次提供日志格式字符串和对应的参数。

//           例如，代码中的 get_logger() 会取得当前节点的日志器；宏会用它的名称标记日志，并按照格式字符串输出车辆 ID、时间戳、轨迹点数量和坐标系等信息。宏还会在编译时检查日志器类型是否正确。


        if (msg->header.frame_id != "map") {
          RCLCPP_WARN(get_logger(), "Expected trajectory in map frame");
          return;
        }
        // 显示确实经过订阅接收的邻车轨迹。
        neighbor_path_pub_->publish(toPath(*msg));
      });
// 订阅器的回调函数定义了收到邻车消息后的处理流程：先过滤本车消息，再记录消息信息，接着检查轨迹是否处于 map 坐标系
// 检查通过后，调用 toPath() 将轨迹转换成可视化路径并发布
    // 本实验车辆停在起点，TF 不沿预测轨迹移动。
    tf_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
// std::make_shared<A>(B);
// 意思就是：
// 创建一个 A 对象，把 B 传给 A 的构造函数，然后返回一个管理这个 A 对象的 shared_ptr。
// 放到你的 ROS 2 代码里：
// std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
// 就是：
// 创建一个 StaticTransformBroadcaster，把当前 TrajectoryCommNode 节点 this 交给它，然后用 shared_ptr 保存这个 broadcaster。
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = "map";
    tf.child_frame_id = vehicle_id_ + "/base_link";
    tf.transform.translation.y = lane_y_;
    tf.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(tf);

    timer_ = create_wall_timer(1s, [this]() {
      publishTrajectory();
    });
  }
//构造函数负责初始化节点及其通信资源。它先读取车辆 ID、邻车话题和车道位置等参数
// 构造函数还创建静态 TF 广播器，将车辆的 base_link 固定关联到 map 坐标系，并设置一个每秒触发一次的定时器来发布轨迹。
private:
  // 缓存规划结果转换后的轨迹点。
  // 每秒发布时只更新整条轨迹的时间基准。
  Trajectory planned_trajectory_;
  nav_msgs::msg::OccupancyGrid map_message_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
  map_pub_;
  nav_msgs::msg::Path toPath(const Trajectory & trajectory)
  {
    nav_msgs::msg::Path path;
    path.header = trajectory.header;

    for (const auto & point : trajectory.points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = trajectory.header.frame_id;

      const auto arrival_time =
        rclcpp::Time(trajectory.header.stamp) +
        rclcpp::Duration(point.time_from_start);

      pose.header.stamp =
        static_cast<builtin_interfaces::msg::Time>(arrival_time);

      pose.pose = point.pose;
      path.poses.push_back(pose);
    }

    return path;
//     一个私有成员函数。
// 它有两个特点：
// 第一，它是“成员函数”，因为它属于这个类。
// 第二，它是“私有的”，因为它写在 private: 下面，所以一般只能在这个类自己的其他成员函数里调用，类外面不能直接调用。
  }
  // 私有辅助函数 toPath() 负责数据格式转换：它创建一个 nav_msgs::msg::Path，
  // 逐点生成 PoseStamped，复制每个点的姿态，并根据轨迹起始时间和点的预计用时计算时间戳。
  // 整体上，构造函数负责搭建 ROS 通信与定时机制，回调函数负责处理接收的消息，toPath() 负责把轨迹数据转换为显示路径。

  void publishTrajectory()
  {
    const auto stamp = now();

    planned_trajectory_.header.stamp = stamp;
    map_message_.header.stamp = stamp;

    map_pub_->publish(map_message_);
    trajectory_pub_->publish(planned_trajectory_);
    own_path_pub_->publish(toPath(planned_trajectory_));
  }

  std::string vehicle_id_;
  std::string neighbor_topic_;
  double lane_y_{0.0};

  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr own_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr neighbor_path_pub_;
  rclcpp::Subscription<Trajectory>::SharedPtr neighbor_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  /// @brief 负责生成规划路径，并将其转换成节点内部缓存的 planned_trajectory_。
  void buildPlannedTrajectory()
  {
    const double goal_x =
      declare_parameter<double>("goal_x", 6.0);

    const double forward_speed =
      declare_parameter<double>("forward_speed", 1.0);

    const double reverse_speed =
      declare_parameter<double>("reverse_speed", 0.5);
// 先读取目标位置、前进速度和倒车速度参数
    if (!std::isfinite(goal_x) || !std::isfinite(lane_y_) ||
        !std::isfinite(forward_speed) || forward_speed <= 0.0 ||
        !std::isfinite(reverse_speed) || reverse_speed <= 0.0)
    {
      throw std::invalid_argument(
        "Goal/lane must be finite; speeds must be finite and positive");
    }
// 再检查目标位置与车道位置是否为有限数值、速度是否为有限正数。参数不合法时会抛出 std::invalid_argument，避免后续规划使用无效数据
    car_swarm_agent::PlannerConfig config;

    // 注意：库的默认搜索上限是 1，必须显式设置。
    config.max_iterations = 200000;
    config.allow_reverse = true;
    config.analytic_expansion = true;
// 函数配置 Hybrid A* 规划器：允许倒车、启用解析扩展，并显式设置最大搜索迭代次数
    config.map.resolution = 0.5;
    config.map.origin_x = -30.0;
    config.map.origin_y = -30.0;
    config.map.width = 120;
    config.map.height = 120;

    config.map.cells.assign(
      static_cast<std::size_t>(config.map.width) *
      static_cast<std::size_t>(config.map.height),
      0);

    car_swarm_agent::addUShapedObstacle(config.map);

    // 保存一份 ROS 地图消息，供 RViz 显示。
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
      config.map.cells.begin(), config.map.cells.end());

    car_swarm_agent::HybridAStarPlanner planner(config);

    const car_swarm_agent::Pose start{0.0, lane_y_, 0.0};
    const car_swarm_agent::Pose goal{goal_x, lane_y_, 0.0};

    car_swarm_agent::PlanningStats stats;
    const auto path = planner.plan(start, goal, &stats);
// 先填充地图和障碍，再构造规划器。 规划器接收的是配置副本，构造之后再改外面的 config 不会更新它。
// 两车这次使用相同的地图，仍分别独立规划。
    if (!stats.reached_goal || path.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "Planning failed: stop=%s points=%zu",
        car_swarm_agent::stopReasonName(stats.stop_reason),
        path.size());

      throw std::runtime_error("Cannot publish a failed plan");
    }
// 如果规划未到达目标或路径为空，函数会记录错误并抛出异常，因此失败的路径不会被当作有效计划继续处理。
    planned_trajectory_.header.frame_id = "map";
    planned_trajectory_.vehicle_id = vehicle_id_;
    planned_trajectory_.points.clear();
    planned_trajectory_.points.reserve(path.size());
// 规划成功后，函数设置轨迹的坐标系为 map，记录车辆 ID，清空旧轨迹点并为新路径预留空间。
    double elapsed_seconds = 0.0;
    std::int64_t previous_ns = 0;

    for (std::size_t i = 0; i < path.size(); ++i) {
      const auto & planner_pose = path[i];
      Point point;

      point.pose.position.x = planner_pose.x;
      point.pose.position.y = planner_pose.y;
      point.pose.position.z = 0.0;

      // yaw 表示车头朝向，倒车时也直接使用规划器的 yaw。
      point.pose.orientation.z = std::sin(planner_pose.yaw / 2.0);
      point.pose.orientation.w = std::cos(planner_pose.yaw / 2.0);
//随后逐个把规划器位姿转换成 Point：位置直接复制，二维偏航角 yaw 转换为四元数的 z 和 w 分量
      if (i == 0) {
        point.direction = Point::STOP;
      } else {
        const double distance = planner_pose.signed_distance;

        // 本阶段不包含停留点，后续运动段必须有有效长度。
        if (!std::isfinite(distance) || distance == 0.0) {
          throw std::runtime_error(
            "Expected a finite, nonzero incoming segment");
        }

        point.direction =
          distance > 0.0 ? Point::FORWARD : Point::REVERSE;

        const double speed =
          distance > 0.0 ? forward_speed : reverse_speed;

        elapsed_seconds += std::abs(distance) / speed;
      }
// 第一个点标记为 STOP；其余点根据该点对应线段的有符号距离标记为前进或倒车
// 这里的 signed_distance 必须有限且非零，否则函数会报错。
// 时间增量按“距离绝对值除以对应速度”累加
      // 转成整数纳秒，处理小数秒和进位。
      // 本次实验轨迹很短；这里同时限制 Duration 的秒数范围。
      if (!std::isfinite(elapsed_seconds) ||
          elapsed_seconds >= 2147483647.0)
      {
        throw std::runtime_error("Trajectory duration is out of range");
      }

      const std::int64_t total_ns =
        std::llround(elapsed_seconds * 1e9);

      if (i > 0 && total_ns <= previous_ns) {
        throw std::runtime_error(
          "Arrival time offsets must strictly increase");
      }

      point.time_from_start.sec =
        static_cast<std::int32_t>(total_ns / 1000000000LL);

      point.time_from_start.nanosec =
        static_cast<std::uint32_t>(total_ns % 1000000000LL);

      previous_ns = total_ns;
      planned_trajectory_.points.push_back(point);
    }
// 每个点的累计到达时间会转换成整数纳秒，再拆分为 ROS 消息中的秒和纳秒字段

    std::size_t forward_segments = 0;
    std::size_t reverse_segments = 0;
    std::size_t direction_changes = 0;

    int previous_direction = Point::STOP;

    for (std::size_t i = 1;
        i < planned_trajectory_.points.size(); ++i)
    {
      const auto & point = planned_trajectory_.points[i];
      const int direction = point.direction;

      if (direction == Point::FORWARD) {
        ++forward_segments;
      } else if (direction == Point::REVERSE) {
        ++reverse_segments;
      } else {
        throw std::runtime_error(
          "Expected FORWARD or REVERSE for a moving segment");
      }

      if (previous_direction != Point::STOP &&
          direction != previous_direction)
      {
        ++direction_changes;

        // direction 表示进入本点的段方向，
        // 所以换向发生在前一个点。
        const auto & switch_point =
          planned_trajectory_.points[i - 1];

        const double switch_time =
          switch_point.time_from_start.sec +
          switch_point.time_from_start.nanosec * 1e-9;

        RCLCPP_INFO(
          get_logger(),
          "Switch vehicle=%s at_point=%zu t=%.3f s "
          "position=(%.3f, %.3f) direction=%d -> %d",
          vehicle_id_.c_str(),
          i - 1,
          switch_time,
          switch_point.pose.position.x,
          switch_point.pose.position.y,
          previous_direction,
          direction);
      }

      previous_direction = direction;
    }

    RCLCPP_INFO(
      get_logger(),
      "Trajectory vehicle=%s forward=%zu reverse=%zu switches=%zu",
      vehicle_id_.c_str(),
      forward_segments,
      reverse_segments,
      direction_changes);

    if (forward_segments == 0 ||
        reverse_segments == 0 ||
        direction_changes == 0)
    {
      RCLCPP_WARN(
        get_logger(),
        "This plan does not cover the mixed-direction test");
    }
// 统计的是转换后的消息，可以确认方向信息确实保留到了通信数据中
    RCLCPP_INFO(
      get_logger(),
      "Planned vehicle=%s points=%zu length=%.3f m "
      "reverse_segments=%zu duration=%.3f s",
      vehicle_id_.c_str(),
      path.size(),
      stats.path_length,
      stats.reverse_segments,
      elapsed_seconds);
// 最后，所有点加入缓存轨迹，并用信息日志输出车辆 ID、点数、路径长度、倒车段数和预计总时长。
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryCommNode>());
  rclcpp::shutdown();
  return 0;
}