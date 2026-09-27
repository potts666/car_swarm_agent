#include <chrono>
#include <cstddef>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class VehicleTfBroadcaster : public rclcpp::Node
{
public:
  VehicleTfBroadcaster()
  : Node("vehicle_tf_broadcaster"),
    static_broadcaster_(
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this)),
    dynamic_broadcaster_(
      std::make_shared<tf2_ros::TransformBroadcaster>(this))
  {
    publishStaticTransforms();

    path_subscription_ =
      create_subscription<nav_msgs::msg::Path>(
        "planned_path",
        10,
        [this](nav_msgs::msg::Path::SharedPtr message) {
          if (message->poses.empty()) {
            RCLCPP_WARN(get_logger(), "Received an empty path.");
            return;
          }

          const bool received_first_path = path_.empty();

          path_ = message->poses;

          if (current_point_index_ >= path_.size()) {
            current_point_index_ = 0;
          }

          if (received_first_path) {
            RCLCPP_INFO(
              get_logger(),
              "Received path with %zu points.",
              path_.size());
          }
        });

    timer_ = create_wall_timer(
      500ms,
      [this]() {
        publishOdomToBaseLink();
      });

    RCLCPP_INFO(
      get_logger(),
      "Vehicle TF broadcaster started.");
  }

private:
  void publishStaticTransforms()
  {
    const auto now = get_clock()->now();

    geometry_msgs::msg::TransformStamped map_to_odom;
    map_to_odom.header.stamp = now;
    map_to_odom.header.frame_id = "map";
    map_to_odom.child_frame_id = "odom";
    map_to_odom.transform.rotation.w = 1.0;

    geometry_msgs::msg::TransformStamped base_to_laser;
    base_to_laser.header.stamp = now;
    base_to_laser.header.frame_id = "base_link";
    base_to_laser.child_frame_id = "laser";
    base_to_laser.transform.translation.x = 0.8;
    base_to_laser.transform.translation.z = 0.3;
    base_to_laser.transform.rotation.w = 1.0;

    static_broadcaster_->sendTransform(
      std::vector<geometry_msgs::msg::TransformStamped>{
        map_to_odom,
        base_to_laser});
  }

  void publishOdomToBaseLink()
  {
    if (path_.empty()) {
      return;
    }

    const auto & target_pose = path_[current_point_index_];

    geometry_msgs::msg::TransformStamped odom_to_base;
    odom_to_base.header.stamp = get_clock()->now();
    odom_to_base.header.frame_id = "odom";
    odom_to_base.child_frame_id = "base_link";

    odom_to_base.transform.translation.x =
      target_pose.pose.position.x;
    odom_to_base.transform.translation.y =
      target_pose.pose.position.y;
    odom_to_base.transform.translation.z =
      target_pose.pose.position.z;
    odom_to_base.transform.rotation =
      target_pose.pose.orientation;

    dynamic_broadcaster_->sendTransform(odom_to_base);

    current_point_index_ =
      (current_point_index_ + 1) % path_.size();
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster>
    static_broadcaster_;

  std::shared_ptr<tf2_ros::TransformBroadcaster>
    dynamic_broadcaster_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr
    path_subscription_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<geometry_msgs::msg::PoseStamped> path_;
  std::size_t current_point_index_{0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}