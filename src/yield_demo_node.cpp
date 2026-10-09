// Plan playback or an offline closed-loop bicycle simulation. No real commands.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <optional>

#include "car_swarm_agent/trajectory_yield.hpp"
#include "car_swarm_agent/vehicle_geometry.hpp"
#include "car_swarm_agent/demo_map.hpp"
#include "car_swarm_agent/demo_validation.hpp"
#include "car_swarm_agent/trajectory_tracking.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace {
constexpr std::int64_t kSecond = 1000000000LL;
constexpr double kPi = 3.14159265358979323846;
using Trajectory = car_swarm_agent::msg::PredictedTrajectory;
using Point = car_swarm_agent::msg::TrajectoryPoint;
using Pose = car_swarm_agent::Pose;
using Marker = visualization_msgs::msg::Marker;
using Clock = std::chrono::steady_clock;

template<typename Time>
std::int64_t ns(const Time & time)
{
  return static_cast<std::int64_t>(time.sec) * kSecond + time.nanosec;
}

Point point(int seconds, double x, double y, double yaw)
{
  Point p;
  p.time_from_start.sec = seconds;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  p.direction = seconds == 0 ? Point::STOP : Point::FORWARD;
  return p;
}

std_msgs::msg::ColorRGBA color(float r, float g, float b, float alpha = 1.0f)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r; c.g = g; c.b = b; c.a = alpha;
  return c;
}
}  // namespace

class YieldDemoNode : public rclcpp::Node
{
public:
  YieldDemoNode() : Node("yield_demo")
  {
    rate_ = declare_parameter<double>("playback_rate", 2.0);
    show_before_ = declare_parameter<bool>("show_before", true);
    loop_ = declare_parameter<bool>("loop", true);
    const auto mode = declare_parameter<std::string>("planner_mode", "st");
    if (mode != "st" && mode != "wait") {
      throw std::invalid_argument("planner_mode must be st or wait");
    }
    use_st_ = mode == "st";
    const auto execution = declare_parameter<std::string>("execution_mode", "playback");
    if (execution != "playback" && execution != "tracking") {
      throw std::invalid_argument("execution_mode must be playback or tracking");
    }
    tracking_requested_ = execution == "tracking";
    tracking_options_.time_step = declare_parameter<double>("model_time_step", 0.01);
    tracking_options_.initial_lateral_offset = declare_parameter<double>("model_lateral_offset", 0.25);
    tracking_options_.initial_heading_offset = declare_parameter<double>("model_heading_offset", 0.04);
    tracking_options_.initial_speed_scale = declare_parameter<double>("model_speed_scale", 0.9);
    const bool map_enabled = declare_parameter<bool>("static_map_enabled", true);
    if (map_enabled) {geometry_.map = car_swarm_agent::makeUShapedDemoMap();}
    speed_options_.initial_speed = declare_parameter<double>("initial_speed", 1.0);
    speed_options_.max_speed = declare_parameter<double>("max_speed", 1.0);
    speed_options_.max_acceleration = declare_parameter<double>("max_acceleration", 1.0);
    if (!std::isfinite(rate_) || rate_ <= 0.0) {
      throw std::invalid_argument("playback_rate must be finite and positive");
    }
    const auto stamp = now();
    original_.header.stamp = stamp;
    original_.header.frame_id = "map";
    original_.vehicle_id = "car_2";
    // Generate the fixed spatial path with the existing Hybrid A* planner.
    // Preserve its incoming arc metadata for the speed search.
    geometry_.max_iterations = 200000;
    // True arc metadata is required by continuous timing and bicycle tracking.
    geometry_.allow_reverse = true;
    geometry_.analytic_expansion = true;
    geometry_.step_size = 0.5;
    geometry_.goal_tolerance = 1e-6;
    geometry_.goal_yaw_tolerance = 1e-6;
    car_swarm_agent::HybridAStarPlanner planner(geometry_);
    car_swarm_agent::PlanningStats stats;
    const auto spatial_path = planner.plan({0, -8, kPi / 2}, {0, 8, kPi / 2}, &stats);
    if (!stats.reached_goal || spatial_path.empty()) {
      throw std::runtime_error("Hybrid A* failed to produce the fixed demo path");
    }
    double distance = 0.0;
    for (std::size_t i = 0; i < spatial_path.size(); ++i) {
      distance += std::abs(spatial_path[i].signed_distance);
      auto p = point(0, spatial_path[i].x, spatial_path[i].y, spatial_path[i].yaw);
      const auto time = std::llround(distance * kSecond);  // Original reference: 1 m/s.
      p.time_from_start.sec = static_cast<std::int32_t>(time / kSecond);
      p.time_from_start.nanosec = static_cast<std::uint32_t>(time % kSecond);
      p.direction = i == 0 ? Point::STOP :
        (spatial_path[i].signed_distance > 0 ? Point::FORWARD : Point::REVERSE);
      original_.points.push_back(p);
    }
    neighbor_.header = original_.header;
    neighbor_.vehicle_id = "car_1";
    // Stop before the U's back wall; prediction covers the full AFTER interval.
    neighbor_.points = {point(0, -10, 0, 0), point(10, 0, 0, 0),
      point(20, 10, 0, 0), point(120, 10, 0, 0)};

    car_swarm_agent::YieldOptions options;
    options.sample_step_ns = 50000000;
    car_swarm_agent::CollisionCheckOptions check;
    check.sample_step_ns = options.sample_step_ns;
    const auto original_check = car_swarm_agent::detectTrajectoryCollision(
      original_, neighbor_, geometry_, geometry_, check);
    if (!original_check.first_conflict) {
      throw std::runtime_error("Demo requires a conflict in the original timing");
    }
    if (use_st_) {
      speed_result_ = car_swarm_agent::planSTSpeed(
        spatial_path, neighbor_, original_.header, original_.vehicle_id,
        geometry_, geometry_, speed_options_);

      // 这就是“放行检查”：全部通过后才能继续回放。
      playback_decision_ = car_swarm_agent::assessAfterPlayback(speed_result_);
      if (playback_decision_ == car_swarm_agent::AfterPlaybackDecision::Blocked)
      {
        throw std::runtime_error(
          "S-T planning or continuous verification failed; "
          "no AFTER playback");
      }

      yielded_ = *speed_result_.trajectory;
      findSpeedLandmark();

      RCLCPP_INFO(
        get_logger(),
        "%s; duration=%.3f s; static map checked=%s",
        car_swarm_agent::afterPlaybackMessage(playback_decision_),
        speed_result_.continuous->duration(),
        speed_result_.continuous_verification->static_map_checked ?
          "true" : "false");
    } else {
      // The legacy timing baseline still needs a static sweep before AFTER.
      if (car_swarm_agent::hasStaticMap(geometry_.map)) {
        for (std::size_t i = 1; i < spatial_path.size(); ++i) {
          if (!planner.isArcCollisionFree(spatial_path[i - 1],
            spatial_path[i].signed_distance, spatial_path[i].curvature))
          {
            throw std::runtime_error("Static sweep blocked; no AFTER playback");
          }
        }
      }
      const auto result = car_swarm_agent::planFixedPathYield(
        original_, neighbor_, stamp.nanoseconds(), geometry_, geometry_, options);
      if (result.status != car_swarm_agent::YieldStatus::Yielded ||
        !result.trajectory || !result.stop_point_index)
      {
        throw std::runtime_error("Demo has no fully verified yielding plan");
      }
      yielded_ = *result.trajectory;
      landmark_pose_ = original_.points[*result.stop_point_index].pose;
      wait_start_ = ns(original_.points[*result.stop_point_index].time_from_start) / 1e9;
      wait_end_ = wait_start_ + result.wait_ns / 1e9;
      has_wait_ = true;
    }
    conflict_seconds_ = (original_check.first_conflict->absolute_time_ns - ns(original_.header.stamp)) /
      static_cast<double>(kSecond);
    end_seconds_ = ns(yielded_.points.back().time_from_start) / static_cast<double>(kSecond);

    if (tracking_requested_) {
      if (!use_st_) {
        throw std::invalid_argument("Tracking requires planner_mode=st continuous output");
      }
      if (playback_decision_ != car_swarm_agent::AfterPlaybackDecision::StaticAndDynamicVerified) {
        RCLCPP_WARN(get_logger(), "Tracking disabled: 静态碰撞未验证; retaining plan playback only");
      } else {
        tracking_options_.max_speed = speed_options_.max_speed;
        tracking_options_.max_acceleration = speed_options_.max_acceleration;
        tracking_options_.max_total_acceleration = speed_options_.continuous_check.max_total_acceleration;
        tracking_ = car_swarm_agent::simulateTracking(*speed_result_.continuous,
          neighbor_, geometry_, geometry_, tracking_options_);
        if (!car_swarm_agent::allowTrackingPlayback(*tracking_)) {
          throw std::runtime_error(
            "Actual model trajectory failed verification or terminal stop; "
            "no tracking AFTER playback; planned certificate does not cover actual motion");
        }
        end_seconds_ = tracking_->actual_trajectory->duration();
        const bool actual_verified = tracking_->actual_verification &&
          tracking_->actual_verification->status == car_swarm_agent::ContinuousCheckStatus::Safe;
        RCLCPP_INFO(get_logger(),
          "Closed-loop bicycle simulation: RMS position=%.4f m, max=%.4f m, final=%.4f m, "
          "max heading=%.4f rad, max speed error=%.4f m/s; actual model verification=%s; no real commands",
          tracking_->rms_position, tracking_->max_position, tracking_->samples.back().error.position,
          tracking_->max_heading, tracking_->max_speed_error, actual_verified ? "passed" : "NOT passed");
      }
    }

    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("yield_demo/markers", latched);
    status_pub_ = create_publisher<std_msgs::msg::String>("yield_demo/status", latched);
    original_path_pub_ = create_publisher<nav_msgs::msg::Path>("yield_demo/original_path", latched);
    yielded_path_pub_ = create_publisher<nav_msgs::msg::Path>("yield_demo/yielded_path", latched);
    neighbor_path_pub_ = create_publisher<nav_msgs::msg::Path>("yield_demo/neighbor_path", latched);
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("yield_demo/map", latched);
    actual_path_pub_ = create_publisher<nav_msgs::msg::Path>("yield_demo/actual_path", latched);
    actual_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("yield_demo/model_odometry", 10);
    tracking_metrics_pub_ = create_publisher<std_msgs::msg::String>("yield_demo/tracking_metrics", 10);
    if (car_swarm_agent::hasStaticMap(geometry_.map)) {
      nav_msgs::msg::OccupancyGrid map;
      map.header = original_.header;
      map.info.resolution = static_cast<float>(geometry_.map.resolution);
      map.info.width = geometry_.map.width;
      map.info.height = geometry_.map.height;
      map.info.origin.position.x = geometry_.map.origin_x;
      map.info.origin.position.y = geometry_.map.origin_y;
      map.info.origin.orientation.w = 1.0;
      map.data.assign(geometry_.map.cells.begin(), geometry_.map.cells.end());
      map_pub_->publish(map);
    }
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    pause_sub_ = create_subscription<std_msgs::msg::Bool>("yield_demo/pause", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr message) {paused_ = message->data;});
    reset_sub_ = create_subscription<std_msgs::msg::Empty>("yield_demo/reset", 10,
      [this](std_msgs::msg::Empty::ConstSharedPtr) {elapsed_ = 0.0; last_tick_ = Clock::now();});
    original_path_pub_->publish(path(original_));
    yielded_path_pub_->publish(path(yielded_));
    neighbor_path_pub_->publish(path(neighbor_));
    if (tracking_) {
      nav_msgs::msg::Path actual_path;
      actual_path.header = original_.header;
      for (const auto & sample : tracking_->samples) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = actual_path.header;
        pose.header.stamp = rclcpp::Time(original_.header.stamp) +
          rclcpp::Duration::from_seconds(sample.time);
        pose.pose.position.x = sample.actual.pose.x;
        pose.pose.position.y = sample.actual.pose.y;
        pose.pose.orientation.z = std::sin(sample.actual.pose.yaw / 2);
        pose.pose.orientation.w = std::cos(sample.actual.pose.yaw / 2);
        actual_path.poses.push_back(pose);
      }
      actual_path_pub_->publish(actual_path);
    }
    last_tick_ = Clock::now();
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {tick();});
    RCLCPP_INFO(get_logger(),
      "RViz %s playback: original conflict at %.2f s; car_2 STOP %.2f -> %.2f s; "
      "candidate duration %.2f s; Hybrid A* points=%zu",
      mode.c_str(), conflict_seconds_, wait_start_, wait_end_, end_seconds_, spatial_path.size());
  }

private:
  void findSpeedLandmark()
  {
    const auto & samples = speed_result_.samples;
    landmark_pose_ = yielded_.points.front().pose;
    for (std::size_t i = 1; i + 1 < samples.size(); ++i) {
      if (samples[i].s < samples.back().s - 1e-8 &&
        samples[i].speed < 1e-8 && samples[i + 1].speed < 1e-8 &&
        std::abs(samples[i + 1].s - samples[i].s) < 1e-8)
      {
        landmark_pose_ = yielded_.points[i].pose;
        wait_start_ = samples[i].time_from_start_ns / 1e9;
        std::size_t j = i + 1;
        while (j + 1 < samples.size() && samples[j + 1].speed < 1e-8 &&
          std::abs(samples[j + 1].s - samples[i].s) < 1e-8) {++j;}
        wait_end_ = samples[j].time_from_start_ns / 1e9;
        has_wait_ = true;
        return;
      }
    }
    for (std::size_t i = 1; i < samples.size(); ++i) {
      if (samples[i].speed < samples[i - 1].speed - 1e-8) {
        landmark_pose_ = yielded_.points[i - 1].pose;
        return;
      }
    }
  }

  std::pair<double, double> speedAt(double seconds) const
  {
    const auto & trajectory = *speed_result_.continuous;

    const double query_time = std::clamp(
      seconds, 0.0, trajectory.duration());

    const auto state = trajectory.evaluate(query_time);

    // 当前演示显示的是速率和速率变化率，
    // 前进、倒车都用同一个非负速率定义。
    const double speed_acceleration =
      std::abs(state.signed_speed) > 1e-8 ?
      (state.signed_speed > 0.0 ?
        state.longitudinal_acceleration :
        -state.longitudinal_acceleration) :
      0.0;

    return {state.speed, speed_acceleration};
  }

  nav_msgs::msg::Path path(const Trajectory & trajectory) const
  {
    nav_msgs::msg::Path p;
    p.header = trajectory.header;
    for (const auto & point : trajectory.points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.header.stamp = rclcpp::Time(trajectory.header.stamp) +
        rclcpp::Duration(point.time_from_start);
      pose.pose = point.pose;
      p.poses.push_back(pose);
    }
    return p;
  }

  Marker marker(const std::string & group, int id, int type) const
  {
    Marker m;
    m.header.frame_id = "map";
    m.header.stamp = now();
    m.ns = group;
    m.id = id;
    m.type = type;
    m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    return m;
  }

  Marker text(int id, double x, double y, const std::string & value,
    const std_msgs::msg::ColorRGBA & c, double height = 0.65) const
  {
    auto m = marker("labels", id, Marker::TEXT_VIEW_FACING);
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = 1.0;
    m.scale.z = height;
    m.color = c;
    m.text = value;
    return m;
  }

  void vehicle(visualization_msgs::msg::MarkerArray & scene, int id, const Pose & pose,
    const std_msgs::msg::ColorRGBA & c, const std::string & label)
  {
    const auto rectangle = car_swarm_agent::vehicleRectangle(pose, geometry_);
    auto body = marker("body", id, Marker::CUBE);
    body.pose.position.x = rectangle.center_x;
    body.pose.position.y = rectangle.center_y;
    body.pose.position.z = 0.35;
    body.pose.orientation.z = std::sin(pose.yaw / 2);
    body.pose.orientation.w = std::cos(pose.yaw / 2);
    body.scale.x = geometry_.vehicle_length;
    body.scale.y = geometry_.vehicle_width;
    body.scale.z = 0.7;
    body.color = c;
    body.color.a = 0.8;
    scene.markers.push_back(body);

    auto margin = marker("safety_margin", id, Marker::LINE_STRIP);
    margin.pose = body.pose;
    margin.pose.position.z = 0.05;
    margin.scale.x = 0.08;
    margin.color = c;
    for (const auto & corner : std::vector<std::pair<double, double>>{
        {-rectangle.half_length, -rectangle.half_width},
        {rectangle.half_length, -rectangle.half_width},
        {rectangle.half_length, rectangle.half_width},
        {-rectangle.half_length, rectangle.half_width},
        {-rectangle.half_length, -rectangle.half_width}})
    {
      geometry_msgs::msg::Point p;
      p.x = corner.first; p.y = corner.second;
      margin.points.push_back(p);
    }
    scene.markers.push_back(margin);

    auto arrow = marker("heading", id, Marker::ARROW);
    arrow.pose.position.x = pose.x;
    arrow.pose.position.y = pose.y;
    arrow.pose.position.z = 0.9;
    arrow.pose.orientation = body.pose.orientation;
    arrow.scale.x = 2.5; arrow.scale.y = 0.2; arrow.scale.z = 0.3;
    arrow.color = color(1, 1, 1);
    scene.markers.push_back(arrow);
    scene.markers.push_back(text(id, rectangle.center_x, rectangle.center_y + 2.0, label, c));

    geometry_msgs::msg::TransformStamped transform;
    transform.header = body.header;
    transform.child_frame_id = "yield_demo/car_" + std::to_string(id) + "/base_link";
    transform.transform.translation.x = pose.x;
    transform.transform.translation.y = pose.y;
    transform.transform.rotation = body.pose.orientation;
    tf_->sendTransform(transform);
  }

  void tick()
  {
    const auto current = Clock::now();
    if (!paused_) {elapsed_ += std::chrono::duration<double>(current - last_tick_).count() * rate_;}
    last_tick_ = current;
    const double before_length = show_before_ ? conflict_seconds_ + 2.0 : 0.0;
    const double cycle_length = before_length + end_seconds_ + 2.0;
    if (loop_ && elapsed_ >= cycle_length) {elapsed_ = std::fmod(elapsed_, cycle_length);}
    const bool before = elapsed_ < before_length;
    const double time = before ? std::min(elapsed_, conflict_seconds_) :
      std::min(std::max(0.0, elapsed_ - before_length), end_seconds_);
    const auto & own = before ? original_ : yielded_;
    const auto absolute = ns(own.header.stamp) + std::llround(time * kSecond);
    std::optional<car_swarm_agent::Pose> pose_own;

    if (use_st_ && !before) {
      // AFTER 的 S-T 回放使用已通过复核的连续轨迹。
      const double query_time = std::clamp(
        time, 0.0, speed_result_.continuous->duration());

      const auto state =
        speed_result_.continuous->evaluate(query_time);

      pose_own = state.pose;
    } else {
      // BEFORE 和原有等待模式保持原来的回放方式。
      pose_own =
        car_swarm_agent::interpolateTrajectoryPose(own, absolute);
    }
    const auto pose_neighbor = car_swarm_agent::interpolateTrajectoryPose(neighbor_, absolute);
    if (!pose_own || !pose_neighbor) {throw std::runtime_error("Playback exceeded prediction coverage");}
    const bool overlap = car_swarm_agent::vehicleRectanglesOverlap(
      car_swarm_agent::vehicleRectangle(*pose_own, geometry_),
      car_swarm_agent::vehicleRectangle(*pose_neighbor, geometry_));
    const auto motion = use_st_ && !before ? speedAt(time) : std::make_pair(1.0, 0.0);
    const bool waiting = !before && time < end_seconds_ &&
      (use_st_ ? motion.first < 1e-8 : (has_wait_ && time >= wait_start_ && time < wait_end_));
    const auto blue = color(0.15, 0.65, 1.0);
    const auto orange = color(1.0, 0.55, 0.15);
    const auto red = color(1.0, 0.15, 0.15);
    const auto yellow = color(1.0, 0.9, 0.1);
    visualization_msgs::msg::MarkerArray scene;
    vehicle(scene, 1, *pose_neighbor, overlap ? red : blue, "car_1 (priority)");
    vehicle(scene, 2, *pose_own, overlap ? red : (waiting ? yellow : orange),
      waiting ? "car_2 STOP / waiting" : (tracking_ && !before ? "car_2 PLAN" : "car_2 (yields)"));

    std::optional<car_swarm_agent::TrackingError> model_error;
    if (tracking_ && !before) {
      const auto actual = tracking_->actual_trajectory->evaluate(
        std::min(time, tracking_->actual_trajectory->duration()));
      const auto planned = speed_result_.continuous->evaluate(
        std::min(time, speed_result_.continuous->duration()));
      model_error = car_swarm_agent::trackingError({actual.pose, actual.signed_speed, 0}, planned);
      const bool model_verified = tracking_->actual_verification &&
        tracking_->actual_verification->status == car_swarm_agent::ContinuousCheckStatus::Safe;
      vehicle(scene, 5, actual.pose, model_verified ? color(0.2, 1.0, 0.4) : red,
        "car_2 BICYCLE MODEL (feedback control)");
      nav_msgs::msg::Odometry odom;
      odom.header = original_.header;
      odom.header.stamp = rclcpp::Time(original_.header.stamp) + rclcpp::Duration::from_seconds(time);
      odom.child_frame_id = "yield_demo/car_5/base_link";
      odom.pose.pose.position.x = actual.pose.x;
      odom.pose.pose.position.y = actual.pose.y;
      odom.pose.pose.orientation.z = std::sin(actual.pose.yaw / 2);
      odom.pose.pose.orientation.w = std::cos(actual.pose.yaw / 2);
      odom.twist.twist.linear.x = actual.signed_speed;
      odom.twist.twist.angular.z = actual.yaw_rate;
      actual_odom_pub_->publish(odom);
      std::ostringstream metrics;
      metrics << std::setprecision(10) << "{\"time\":" << time
        << ",\"position_m\":" << model_error->position
        << ",\"longitudinal_m\":" << model_error->longitudinal
        << ",\"lateral_m\":" << model_error->lateral
        << ",\"heading_rad\":" << model_error->heading
        << ",\"speed_mps\":" << model_error->speed
        << ",\"rms_position_m\":" << tracking_->rms_position
        << ",\"max_position_m\":" << tracking_->max_position
        << ",\"actual_model_verified\":" << (model_verified ? "true" : "false") << "}";
      std_msgs::msg::String message; message.data = metrics.str();
      tracking_metrics_pub_->publish(message);
    }

    const auto & stop_pose = landmark_pose_;
    auto stop = marker("parking", 0, Marker::CYLINDER);
    stop.pose.position = stop_pose.position;
    stop.pose.position.z = 0.03;
    stop.scale.x = stop.scale.y = 1.0;
    stop.scale.z = 0.05;
    stop.color = yellow;
    scene.markers.push_back(stop);
    scene.markers.push_back(text(3, stop_pose.position.x + 4.5, stop_pose.position.y,
      has_wait_ ? "STOP point" : "DECEL point", yellow));

    std::ostringstream status;
    status << (before ? "BEFORE: original conflict demo" :
      (use_st_ ? "AFTER: S-T speed search" : "AFTER: fixed-path waiting baseline"))
      << "\nt=" << std::fixed << std::setprecision(2) << time << " s   "
      << (overlap ? "BODY CONFLICT (incl. margin)" : (waiting ? "car_2 WAITING" :
        (time >= end_seconds_ ? "DONE" : (use_st_ && motion.second < -0.01 ? "BRAKING" :
          (use_st_ && motion.second > 0.01 ? "ACCELERATING" : "DRIVING")))));
    if (use_st_ && !before) {
      status << "\nv=" << motion.first << " m/s  a=" << motion.second << " m/s^2"
        << " (limits " << speed_options_.max_speed << ", +/-" << speed_options_.max_acceleration << ")";
    }
    if (has_wait_) {
      status << "\nSTOP " << wait_start_ << " -> " << wait_end_ << " s";
    }
    if (use_st_) {
      status << "\n" << car_swarm_agent::afterPlaybackMessage(playback_decision_);
    } else {
      status << "\nNeighbor collision: no conflict at 0.05 s samples"
        << (car_swarm_agent::hasStaticMap(geometry_.map) ?
        "\nStatic path sweep checked" : "\n静态碰撞未验证 (static collision NOT verified)");
    }
    if (tracking_) {
      status << "\nClosed-loop BICYCLE MODEL simulation; no real vehicle commands";
      if (model_error) {
        status << "\nActual-plan: position=" << model_error->position
          << " m, lateral=" << model_error->lateral << " m, heading=" << model_error->heading
          << " rad, speed=" << model_error->speed << " m/s"
          << "\nWhole simulation: RMS=" << tracking_->rms_position
          << " m, max=" << tracking_->max_position << " m";
      }
      status << ((tracking_->actual_verification &&
        tracking_->actual_verification->status == car_swarm_agent::ContinuousCheckStatus::Safe) ?
        "\nActual numerical model trajectory verification passed" :
        "\nActual model trajectory NOT verified; planned certificate does not cover actual motion");
    } else {
      status << "\nPlan playback only; playback is NOT control; no vehicle commands";
      if (tracking_requested_) {status << "\nTracking disabled: 静态碰撞未验证";}
    }
    status << (paused_ ? " [PAUSED]" : "");
    scene.markers.push_back(text(4, 2.0, 17.0, status.str(), overlap ? red : color(0.9, 0.95, 1.0)));
    markers_pub_->publish(scene);
    std_msgs::msg::String message;
    message.data = status.str();
    status_pub_->publish(message);
  }

  Trajectory original_, neighbor_, yielded_;
  car_swarm_agent::PlannerConfig geometry_;
  car_swarm_agent::STOptions speed_options_;
  car_swarm_agent::STResult speed_result_;
  car_swarm_agent::TrackingOptions tracking_options_;
  std::optional<car_swarm_agent::TrackingSimulation> tracking_;
  bool tracking_requested_{false};
  car_swarm_agent::AfterPlaybackDecision playback_decision_{
    car_swarm_agent::AfterPlaybackDecision::Blocked};
  geometry_msgs::msg::Pose landmark_pose_;
  double rate_{2.0}, elapsed_{0.0}, conflict_seconds_{0.0};
  double wait_start_{0.0}, wait_end_{0.0}, end_seconds_{0.0};
  bool show_before_{true}, loop_{true}, paused_{false}, use_st_{true}, has_wait_{false};
  Clock::time_point last_tick_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr original_path_pub_, yielded_path_pub_, neighbor_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr actual_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr actual_odom_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tracking_metrics_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<YieldDemoNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("yield_demo"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
