#include "car_swarm_agent/trajectory_yield.hpp"
#include "car_swarm_agent/vehicle_geometry.hpp"
#include "car_swarm_agent/demo_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <tuple>
#include <stdexcept>
#include <utility>

namespace car_swarm_agent {
namespace {
constexpr std::int64_t kSecond = 1000000000LL;
constexpr std::int64_t kMaxDuration =
  static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()) * kSecond +
  kSecond - 1;

template<typename Time>
std::int64_t ns(const Time & time)
{
  return static_cast<std::int64_t>(time.sec) * kSecond + time.nanosec;
}

void setOffset(msg::TrajectoryPoint & point, std::int64_t time)
{
  point.time_from_start.sec = static_cast<std::int32_t>(time / kSecond);
  point.time_from_start.nanosec = static_cast<std::uint32_t>(time % kSecond);
}

bool fullyCovered(const msg::PredictedTrajectory & trajectory,
  const msg::PredictedTrajectory & neighbor)
{
  const auto start = ns(trajectory.header.stamp) + ns(trajectory.points.front().time_from_start);
  const auto end = ns(trajectory.header.stamp) + ns(trajectory.points.back().time_from_start);
  const auto neighbor_start = ns(neighbor.header.stamp) +
    ns(neighbor.points.front().time_from_start);
  const auto neighbor_end = ns(neighbor.header.stamp) +
    ns(neighbor.points.back().time_from_start);
  return neighbor_start <= start && neighbor_end >= end;
}

msg::PredictedTrajectory insertWait(const msg::PredictedTrajectory & original,
  std::size_t stop_index, std::int64_t wait)
{
  auto candidate = original;
  auto endpoint = original.points[stop_index];
  setOffset(endpoint, ns(endpoint.time_from_start) + wait);
  endpoint.direction = msg::TrajectoryPoint::STOP;
  candidate.points.insert(candidate.points.begin() + stop_index + 1, endpoint);
  for (std::size_t i = stop_index + 2; i < candidate.points.size(); ++i) {
    setOffset(candidate.points[i], ns(candidate.points[i].time_from_start) + wait);
  }
  return candidate;
}
}  // namespace

YieldResult planFixedPathYield(
  const msg::PredictedTrajectory & own, const msg::PredictedTrajectory & neighbor,
  std::int64_t now_ns, const PlannerConfig & own_geometry,
  const PlannerConfig & neighbor_geometry, const YieldOptions & options)
{
  if (own.vehicle_id.empty() || neighbor.vehicle_id.empty() ||
    own.vehicle_id == neighbor.vehicle_id)
  {
    throw std::invalid_argument("Yield planning requires distinct nonempty vehicle IDs");
  }
  if (options.wait_step_ns <= 0 || options.max_wait_ns < 0 ||
    options.max_wait_ns > kMaxDuration)
  {
    throw std::invalid_argument("Invalid wait search bounds");
  }
  CollisionCheckOptions check_options;
  check_options.sample_step_ns = options.sample_step_ns;
  // Validate all input geometry, poses, frame IDs and offsets through the
  // existing detector, and retain the complete original check (including past).
  const auto initial = detectTrajectoryCollision(
    own, neighbor, own_geometry, neighbor_geometry, check_options);
  YieldResult result;
  result.original_conflict = initial.first_conflict;
  if (!own.points.empty() &&
    (ns(own.points.front().time_from_start) != 0 ||
    own.points.front().direction != msg::TrajectoryPoint::STOP))
  {
    throw std::invalid_argument("Own trajectory must start at offset zero with STOP");
  }
  for (const auto & point : own.points) {
    if (point.direction != msg::TrajectoryPoint::STOP &&
      point.direction != msg::TrajectoryPoint::FORWARD &&
      point.direction != msg::TrajectoryPoint::REVERSE)
    {
      throw std::invalid_argument("Invalid own trajectory direction");
    }
  }
  if (own.points.empty() || neighbor.points.empty() ||
    ns(own.header.stamp) + ns(own.points.back().time_from_start) < now_ns ||
    ns(neighbor.header.stamp) + ns(neighbor.points.back().time_from_start) < now_ns ||
    !fullyCovered(own, neighbor))
  {
    return result;
  }
  if (initial.status == CollisionCheckStatus::NoConflictAtSamples) {
    result.status = YieldStatus::Unchanged;
    result.trajectory = own;
    result.verification = initial;
    return result;
  }
  if (initial.status != CollisionCheckStatus::Conflict) {return result;}
  if (own.vehicle_id < neighbor.vehicle_id) {
    result.status = YieldStatus::AwaitingPeerYield;
    return result;
  }

  const auto stamp = ns(own.header.stamp);
  const auto conflict_time = initial.first_conflict->absolute_time_ns;
  bool incomplete_candidate = false;
  result.status = YieldStatus::NoFeasiblePlan;
  // Waiting duration is the outer loop: first feasible candidate has the
  // shortest tested delay, with the latest eligible parking point as tie-break.
  for (auto wait = options.wait_step_ns; wait <= options.max_wait_ns; ) {
    for (std::size_t i = own.points.size(); i-- > 0; ) {
      const auto stop_time = stamp + ns(own.points[i].time_from_start);
      if (stop_time >= conflict_time || stop_time < now_ns) {continue;}
      ++result.candidates_tried;
      if (ns(own.points.back().time_from_start) > kMaxDuration - wait) {continue;}
      auto candidate = insertWait(own, i, wait);
      if (!fullyCovered(candidate, neighbor)) {
        incomplete_candidate = true;
        continue;
      }
      // Check the parking interval itself, including both endpoints, then the
      // entire candidate. A clear crossing alone is insufficient.
      auto parking = candidate;
      parking.points = {candidate.points[i], candidate.points[i + 1]};
      const auto parking_check = detectTrajectoryCollision(
        parking, neighbor, own_geometry, neighbor_geometry, check_options);
      if (parking_check.status != CollisionCheckStatus::NoConflictAtSamples) {continue;}
      const auto verified = detectTrajectoryCollision(
        candidate, neighbor, own_geometry, neighbor_geometry, check_options);
      if (verified.status != CollisionCheckStatus::NoConflictAtSamples) {continue;}
      result.status = YieldStatus::Yielded;
      result.trajectory = std::move(candidate);
      result.verification = verified;
      result.stop_point_index = i;
      result.wait_ns = wait;
      return result;
    }
    if (options.max_wait_ns - wait < options.wait_step_ns) {break;}
    wait += options.wait_step_ns;
  }
  if (incomplete_candidate) {result.status = YieldStatus::InformationInsufficient;}
  return result;
}
}  // namespace car_swarm_agent


namespace car_swarm_agent {
namespace {
constexpr std::int64_t kSTSecond = 1000000000LL;
constexpr double kEpsilon = 1e-8;
constexpr double kTwoPi = 6.28318530717958647692;
using Trajectory = msg::PredictedTrajectory;
using Point = msg::TrajectoryPoint;

template<typename Time>
std::int64_t stNanoseconds(const Time & t)
{
  return static_cast<std::int64_t>(t.sec) * kSTSecond + t.nanosec;
}

Pose advance(const Pose & from, double distance, double curvature)
{
  if (std::abs(curvature) < 1e-12) {
    return {from.x + distance * std::cos(from.yaw),
      from.y + distance * std::sin(from.yaw), from.yaw};
  }
  const double yaw = from.yaw + distance * curvature;
  return {from.x + (std::sin(yaw) - std::sin(from.yaw)) / curvature,
    from.y + (std::cos(from.yaw) - std::cos(yaw)) / curvature,
    std::remainder(yaw, kTwoPi)};
}

struct SpatialPath
{
  const std::vector<Pose> & points;
  std::vector<double> lengths{0.0};
  std::vector<double> stops;

  explicit SpatialPath(const std::vector<Pose> & path) : points(path)
  {
    if (path.size() < 2) {throw std::invalid_argument("Path needs at least two points");}
    for (std::size_t i = 0; i < path.size(); ++i) {
      const auto & p = path[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw) ||
        !std::isfinite(p.signed_distance) || !std::isfinite(p.curvature))
      {
        throw std::invalid_argument("Path data must be finite");
      }
      if (i == 0) {continue;}
      if (p.signed_distance == 0.0) {throw std::invalid_argument("Zero-length path segment");}
      const auto expected = advance(path[i - 1], p.signed_distance, p.curvature);
      if (std::hypot(expected.x - p.x, expected.y - p.y) > 1e-5 ||
        std::abs(std::remainder(expected.yaw - p.yaw, kTwoPi)) > 1e-5)
      {
        throw std::invalid_argument("Path poses do not match incoming arc metadata");
      }
      if (i > 1 && (p.signed_distance > 0) != (path[i - 1].signed_distance > 0)) {
        stops.push_back(lengths.back());
      }
      const double length = lengths.back() + std::abs(p.signed_distance);
      if (!std::isfinite(length) || length > 1e6) {
        throw std::invalid_argument("Path is too long for this small search");
      }
      lengths.push_back(length);
    }
    stops.push_back(lengths.back());  // Stop at the final goal, too.
  }

  std::size_t segment(double s) const
  {
    const auto it = std::upper_bound(lengths.begin(), lengths.end(), s);
    return std::min<std::size_t>(std::max<std::size_t>(1, it - lengths.begin()), points.size() - 1);
  }

  Pose pose(double s) const
  {
    if (s >= lengths.back() - kEpsilon) {return points.back();}
    const auto i = segment(s);
    const double distance = (points[i].signed_distance > 0 ? 1.0 : -1.0) *
      (s - lengths[i - 1]);
    return advance(points[i - 1], distance, points[i].curvature);
  }

  std::size_t nextStop(double s) const
  {
    return std::upper_bound(stops.begin(), stops.end(), s + kEpsilon) - stops.begin();
  }
};

Point trajectoryPoint(const Pose & pose, std::int64_t time, int direction)
{
  Point p;
  p.time_from_start.sec = static_cast<std::int32_t>(time / kSTSecond);
  p.time_from_start.nanosec = static_cast<std::uint32_t>(time % kSTSecond);
  p.pose.position.x = pose.x;
  p.pose.position.y = pose.y;
  p.pose.orientation.z = std::sin(pose.yaw / 2.0);
  p.pose.orientation.w = std::cos(pose.yaw / 2.0);
  p.direction = static_cast<std::int8_t>(direction);
  return p;
}

struct Node
{
  double s, v;
  int step;
  std::size_t parent;
  double acceleration, moving_time;  // Incoming edge, then zero-speed waiting.
};

struct Edge
{
  Trajectory trajectory;
  std::vector<STSample> samples;
};

Edge makeEdge(const SpatialPath & path, const Node & from, const Node & to,
  const std_msgs::msg::Header & header, const std::string & id, const STOptions & options)
{
  Edge edge;
  edge.trajectory.header = header;
  edge.trajectory.vehicle_id = id;
  const auto duration = options.time_step_ns;
  const auto start = static_cast<std::int64_t>(from.step) * duration;
  std::vector<std::int64_t> times{0, duration};
  for (auto t = options.sample_step_ns; t < duration; t += options.sample_step_ns) {
    times.push_back(t);
  }
  // Include the exact braking/STOP transition and crossed spatial knots.
  const auto stopped_at = std::llround(to.moving_time * kSTSecond);
  if (stopped_at > 0 && stopped_at < duration) {times.push_back(stopped_at);}
  for (const double s : path.lengths) {
    if (s <= from.s + kEpsilon || s >= to.s - kEpsilon) {continue;}
    const double distance = s - from.s;
    const double end_v = std::sqrt(std::max(0.0,
      from.v * from.v + 2.0 * to.acceleration * distance));
    const double denominator = from.v + end_v;
    if (denominator > kEpsilon) {
      const auto t = std::llround(2.0 * distance / denominator * kSTSecond);
      if (t > 0 && t < duration) {times.push_back(t);}
    }
  }
  std::sort(times.begin(), times.end());
  times.erase(std::unique(times.begin(), times.end()), times.end());
  double previous_s = from.s;
  for (const auto t : times) {
    const double moving = std::min(t / static_cast<double>(kSTSecond), to.moving_time);
    double s = from.s + from.v * moving + 0.5 * to.acceleration * moving * moving;
    double speed = std::max(0.0, from.v + to.acceleration * moving);
    if (t == duration) {s = to.s; speed = to.v;}
    const auto i = path.segment(0.5 * (previous_s + s));
    const int direction = t == 0 || s <= previous_s + 1e-12 ? Point::STOP :
      (path.points[i].signed_distance > 0 ? Point::FORWARD : Point::REVERSE);
    edge.trajectory.points.push_back(trajectoryPoint(path.pose(s), start + t, direction));
    edge.samples.push_back({start + t, s, speed});
    previous_s = s;
  }
  return edge;
}

void validateSTOptions(const STOptions & o)
{
  if (!std::isfinite(o.initial_speed) || o.initial_speed < 0 ||
    !std::isfinite(o.max_speed) || o.max_speed <= 0 || o.initial_speed > o.max_speed ||
    !std::isfinite(o.max_acceleration) || o.max_acceleration <= 0 ||
    o.time_step_ns <= 0 || o.time_step_ns > 10 * kSTSecond ||
    o.sample_step_ns <= 0 || o.sample_step_ns > o.time_step_ns ||
    o.time_step_ns / o.sample_step_ns > 10000 ||
    o.max_steps <= 0 || o.max_steps > 10000 || o.max_nodes < 2 ||
    !std::isfinite(o.s_resolution) || o.s_resolution < 1e-6 ||
    !std::isfinite(o.v_resolution) || o.v_resolution < 1e-6 || o.max_speed > 1000)
  {
    throw std::invalid_argument("Invalid S-T search options");
  }
}
}  // namespace

STResult planSTSpeed(const std::vector<Pose> & poses, const Trajectory & neighbor,
  const std_msgs::msg::Header & header, const std::string & id,
  const PlannerConfig & own_geometry, const PlannerConfig & neighbor_geometry,
  const STOptions & options)
{
  validateSTOptions(options);
  if (id.empty()) {throw std::invalid_argument("Vehicle ID must be nonempty");}
  const SpatialPath path(poses);
  STResult result;
  Trajectory initial;
  initial.header = header;
  initial.vehicle_id = id;
  initial.points.push_back(trajectoryPoint(path.pose(0), 0, Point::STOP));
  CollisionCheckOptions check;
  check.sample_step_ns = options.sample_step_ns;
  // Also validates time fields, common frame and both body geometries.
  const auto first = detectTrajectoryCollision(initial, neighbor, own_geometry, neighbor_geometry, check);
  const auto stamp = stNanoseconds(header.stamp);
  if (neighbor.points.empty() || first.status == CollisionCheckStatus::NoCommonTime) {
    result.status = STStatus::InformationInsufficient;
    return result;
  }
  if (first.status != CollisionCheckStatus::NoConflictAtSamples) {return result;}
  // Timing cannot repair a fixed spatial path which sweeps into a wall.
  // Reject it before expanding the S-T tree, with a diagnostic for the demo gate.
  if (hasStaticMap(own_geometry.map)) {
    HybridAStarPlanner planner(own_geometry);
    for (std::size_t i = 1; i < poses.size(); ++i) {
      if (!planner.isArcCollisionFree(poses[i - 1],
        poses[i].signed_distance, poses[i].curvature))
      {
        ContinuousCheckResult blocked;
        blocked.status = ContinuousCheckStatus::CollisionOrStaticBlocked;
        blocked.static_map_checked = true;
        result.continuous_verification = blocked;
        return result;
      }
    }
  }
  const auto neighbor_end = stNanoseconds(neighbor.header.stamp) + stNanoseconds(neighbor.points.back().time_from_start);
  const double dt = options.time_step_ns / static_cast<double>(kSTSecond);
  if (options.initial_speed * options.initial_speed / (2 * options.max_acceleration) >
    path.stops.front() + kEpsilon)
  {
    return result;  // Even maximum braking cannot stop at the first mandatory boundary.
  }
  std::vector<Node> nodes{{0, options.initial_speed, 0, 0, 0, 0}};
  using Key = std::tuple<int, long long, long long, std::size_t, bool>;
  std::set<Key> visited;
  bool unknown = false;
  // FIFO expansion gives increasing arrival time. Physical s/v are never snapped
  // to bins; bins only suppress similar states, so completeness is not claimed.
  for (std::size_t index = 0; index < nodes.size(); ++index) {
    const Node current = nodes[index];  // Copy: vector growth invalidates references.
    ++result.nodes_expanded;
    if (current.s >= path.lengths.back() - kEpsilon && current.v <= kEpsilon) {
      std::vector<std::size_t> chain;
      for (auto i = index; i != 0; i = nodes[i].parent) {chain.push_back(i);}
      std::reverse(chain.begin(), chain.end());

      // 保留搜索边的运动方程，避免从离散消息反推加速度。
      std::vector<STMotionPiece> pieces;
      for (const auto i : chain) {
        const auto & from = nodes[nodes[i].parent];
        const auto & to = nodes[i];

        pieces.push_back({
          from.step * dt,
          dt,
          from.s,
          from.v,
          to.acceleration,
          to.moving_time
        });
      }

      ContinuousTrajectory continuous(poses, header, std::move(pieces));

      auto continuous_options = options.continuous_check;
      continuous_options.max_speed = options.max_speed;
      continuous_options.max_longitudinal_acceleration =
        options.max_acceleration;

      const auto continuous_verified = verifyContinuousTrajectory(
        continuous, neighbor, own_geometry, neighbor_geometry,
        continuous_options);

      result.continuous_verification = continuous_verified;

      // 包括 Unresolved 在内，任何非 Safe 结果都不放行。
      // 继续搜索其他候选，不能回退执行这条候选。
      if (continuous_verified.status != ContinuousCheckStatus::Safe) {
        continue;
      }

      auto output = initial;
      std::vector<STSample> samples{{0, 0, options.initial_speed}};
      for (const auto i : chain) {
        const auto edge = makeEdge(path, nodes[nodes[i].parent], nodes[i], header, id, options);
        output.points.insert(output.points.end(), edge.trajectory.points.begin() + 1,
          edge.trajectory.points.end());
        samples.insert(samples.end(), edge.samples.begin() + 1, edge.samples.end());
      }
      const auto verified = detectTrajectoryCollision(output, neighbor, own_geometry, neighbor_geometry, check);
      if (verified.status != CollisionCheckStatus::NoConflictAtSamples) {continue;}
      result.continuous = std::move(continuous);
      result.status = STStatus::Success;
      result.trajectory = std::move(output);
      result.samples = std::move(samples);
      result.verification = verified;
      return result;
    }
    if (current.step >= options.max_steps) {continue;}
    const auto next_time = static_cast<std::int64_t>(current.step + 1) * options.time_step_ns;
    if (stamp + next_time > neighbor_end) {unknown = true; continue;}
    const auto stop_id = path.nextStop(current.s);
    if (stop_id >= path.stops.size()) {continue;}
    const double boundary = path.stops[stop_id];
    const double remaining = boundary - current.s;
    std::vector<double> actions{options.max_acceleration, 0, -options.max_acceleration};
    actions.push_back((options.max_speed - current.v) / dt);
    if (current.v > kEpsilon) {
      // Constant braking that arrives at this boundary with exactly zero speed.
      actions.push_back(-current.v * current.v / (2 * remaining));
    }
    for (const double acceleration : actions) {
      if (std::abs(acceleration) > options.max_acceleration + 1e-12) {continue;}
      const double moving_time = acceleration < 0 ? std::min(dt, current.v / -acceleration) : dt;
      double v = current.v + acceleration * moving_time;
      if (moving_time < dt || v < kEpsilon) {v = 0;}
      if (v > options.max_speed + kEpsilon) {continue;}
      double s = current.s + current.v * moving_time +
        0.5 * acceleration * moving_time * moving_time;
      if (s > boundary + kEpsilon) {continue;}
      if (std::abs(s - boundary) <= kEpsilon) {
        if (v > kEpsilon) {continue;}
        s = boundary;
      }
      if (v * v / (2 * options.max_acceleration) > boundary - s + kEpsilon) {continue;}
      Node next{s, v, current.step + 1, index, acceleration, moving_time};
      const Key key{next.step, std::llround(s / options.s_resolution),
        std::llround(v / options.v_resolution), path.nextStop(s), v == 0.0};
      if (visited.count(key)) {continue;}
      const auto edge = makeEdge(path, current, next, header, id, options);
      const auto checked = detectTrajectoryCollision(edge.trajectory, neighbor,
        own_geometry, neighbor_geometry, check);
      if (checked.status != CollisionCheckStatus::NoConflictAtSamples) {continue;}
      if (nodes.size() >= options.max_nodes) {
        result.status = STStatus::SearchLimit;
        return result;
      }
      visited.insert(key);
      nodes.push_back(next);
    }
  }
  if (unknown) {result.status = STStatus::InformationInsufficient;}
  return result;
}
}  // namespace car_swarm_agent

namespace car_swarm_agent {
namespace {

double movingS(const STMotionPiece & p, double u)
{
  u = std::min(u, p.moving_time);
  return p.s0 + p.v0 * u +
         0.5 * p.acceleration * u * u;
}

double bodyRadius(const PlannerConfig & g)
{
  // 含原有 collision_margin 的车身最远角点到后轴的距离。
  return std::hypot(
    std::max(g.rear_overhang,
      g.vehicle_length - g.rear_overhang) + g.collision_margin,
    g.vehicle_width * 0.5 + g.collision_margin);
}

}  // namespace

ContinuousTrajectory::ContinuousTrajectory(
  std::vector<Pose> path,
  std_msgs::msg::Header header,
  std::vector<STMotionPiece> pieces)
: path_(std::move(path)),
  header_(std::move(header)),
  pieces_(std::move(pieces))
{
  const SpatialPath spatial(path_);

  if (header_.frame_id.empty() ||
    header_.stamp.nanosec >= kSTSecond ||
    pieces_.empty())
  {
    throw std::invalid_argument(
      "Invalid continuous trajectory header/pieces");
  }

  // 消除路径节点数值误差导致的位姿跳变。
  // 保留每段 signed_distance 和 curvature，
  // 用真实圆弧积分重新得到端点。
  for (std::size_t i = 1; i < path_.size(); ++i) {
    const auto exact = advance(
      path_[i - 1],
      path_[i].signed_distance,
      path_[i].curvature);

    if (std::hypot(
        exact.x - path_[i].x,
        exact.y - path_[i].y) > 1e-5 ||
      std::abs(std::remainder(
        exact.yaw - path_[i].yaw, kTwoPi)) > 1e-5)
    {
      throw std::invalid_argument(
        "Accumulated path endpoint drift");
    }

    path_[i].x = exact.x;
    path_[i].y = exact.y;
    path_[i].yaw = exact.yaw;
  }

  double time = 0;
  double s = 0;
  double v = pieces_.front().v0;

  for (auto & p : pieces_) {
    for (double x : {
        p.t0, p.duration, p.s0, p.v0,
        p.acceleration, p.moving_time})
    {
      if (!std::isfinite(x)) {
        throw std::invalid_argument("Nonfinite motion piece");
      }
    }

    const double vm =
      p.v0 + p.acceleration * p.moving_time;

    if (p.duration <= 0 ||
      p.moving_time < 0 ||
      p.moving_time > p.duration ||
      std::abs(p.t0 - time) > 1e-9 ||
      std::abs(p.s0 - s) > 1e-7 ||
      std::abs(p.v0 - v) > 1e-7 ||
      p.v0 < 0 ||
      vm < -1e-8 ||
      (p.moving_time < p.duration &&
      std::abs(vm) > 1e-8))
    {
      throw std::invalid_argument(
        "Discontinuous/nonphysical motion pieces");
    }

    // 只消除已通过容差检查的连续性数值残差。
    p.t0 = time;
    p.s0 = s;
    p.v0 = v;

    s = movingS(p, p.duration);
    v = p.moving_time < p.duration ?
      0.0 : std::max(0.0, vm);
    time = p.t0 + p.duration;

    if (!std::isfinite(time) ||
      s > spatial.lengths.back() + 1e-7)
    {
      throw std::invalid_argument(
        "Motion exceeds path/time range");
    }
  }

  if (std::abs(s - spatial.lengths.back()) > 1e-7 ||
    v > 1e-8)
  {
    throw std::invalid_argument(
      "Trajectory must finish at path end with zero speed");
  }
}

double ContinuousTrajectory::duration() const
{
  return pieces_.back().t0 + pieces_.back().duration;
}

ContinuousState ContinuousTrajectory::evaluate(
  double t, bool left_limit) const
{
  if (!std::isfinite(t) || t < 0 || t > duration()) {
    throw std::out_of_range(
      "Continuous trajectory time out of range");
  }

  const auto it = std::upper_bound(
    pieces_.begin(), pieces_.end(), t,
    [](double x, const STMotionPiece & p) {
      return x < p.t0;
    });

  std::size_t j =
    static_cast<std::size_t>(it - pieces_.begin() - 1);

  if (left_limit && j > 0 && t == pieces_[j].t0) {
    --j;
  }

  const auto & p = pieces_[j];
  const double u = t - p.t0;
  const double s = movingS(p, u);

  const double speed = u > p.moving_time ? 0.0 :
    std::max(0.0,
      p.v0 + p.acceleration *
      std::min(u, p.moving_time));

  const double a =
    u < p.moving_time ||
    (left_limit && u == p.moving_time) ?
    p.acceleration : 0.0;

  const SpatialPath spatial(path_);
  auto i = spatial.segment(s);

  if (left_limit && i > 1 &&
    std::abs(s - spatial.lengths[i - 1]) < 1e-10)
  {
    --i;
  }

  const double gear =
    path_[i].signed_distance > 0 ? 1.0 : -1.0;
  const double k = path_[i].curvature;

  const Pose pose = advance(
    path_[i - 1],
    gear * (s - spatial.lengths[i - 1]),
    k);

  const double c = std::cos(pose.yaw);
  const double sn = std::sin(pose.yaw);

  return {
    s,
    speed,
    gear * speed,
    gear * a,
    pose,
    gear * speed * c,
    gear * speed * sn,
    gear * a * c - k * speed * speed * sn,
    gear * a * sn + k * speed * speed * c,
    gear * k * speed,
    static_cast<int>(gear),
    k
  };
}

ContinuousCheckResult verifyContinuousTrajectory(
  const ContinuousTrajectory & own,
  const Trajectory & peer,
  const PlannerConfig & own_g,
  const PlannerConfig & peer_g,
  const ContinuousCheckOptions & o)
{
  if (!std::isfinite(o.max_speed) ||
    o.max_speed <= 0 ||
    !std::isfinite(o.max_longitudinal_acceleration) ||
    o.max_longitudinal_acceleration <= 0 ||
    !std::isfinite(o.max_total_acceleration) ||
    o.max_total_acceleration <= 0 ||
    !std::isfinite(o.min_interval) ||
    o.min_interval <= 0 ||
    o.max_depth < 0 ||
    o.max_depth > 60 ||
    o.max_intervals == 0)
  {
    throw std::invalid_argument(
      "Invalid continuous check options");
  }

  // 借用现有函数校验消息、坐标系、时间和车身参数。
  // 此处不把它的采样结果作为连续安全证书。
  Trajectory probe;
  probe.header = own.header();
  probe.points.push_back(
    trajectoryPoint(own.evaluate(0).pose, 0, Point::STOP));

  (void)detectTrajectoryCollision(
    probe, peer, own_g, peer_g);

  ContinuousCheckResult result;

  if (peer.points.size() < 2) {
    result.status =
      ContinuousCheckStatus::InformationInsufficient;
    return result;
  }

  struct PeerKnot
  {
    double t;
    Pose pose;
  };

  std::vector<PeerKnot> knots;
  const auto origin = stNanoseconds(own.header().stamp);

  for (const auto & p : peer.points) {
    const auto ns =
      stNanoseconds(peer.header.stamp) +
      stNanoseconds(p.time_from_start);

    knots.push_back({
      (ns - origin) / 1e9,
      *interpolateTrajectoryPose(peer, ns)
    });
  }

  // 必须覆盖本车完整执行区间，禁止外推邻车。
  if (knots.front().t > 0 ||
    knots.back().t < own.duration())
  {
    result.status =
      ContinuousCheckStatus::InformationInsufficient;
    return result;
  }

  const SpatialPath spatial(own.path());
  double own_rate = 0;

  for (const auto & p : own.pieces()) {
    const double end_s = movingS(p, p.duration);
    const double end_v = std::max(
      0.0, p.v0 + p.acceleration * p.moving_time);

    // 恒加速度段速度为仿射函数，极值在端点。
    const double vmax = std::max(p.v0, end_v);
    const double a = p.moving_time > 0 ?
      std::abs(p.acceleration) : 0;

    double kmax = 0;

    for (std::size_t i = 1; i < own.path().size(); ++i) {
      if (spatial.lengths[i] >= p.s0 &&
        spatial.lengths[i - 1] <= end_s)
      {
        kmax = std::max(
          kmax, std::abs(own.path()[i].curvature));
      }
    }

    // 整段运动限制的保守上界，不只检查离散节点。
    if (vmax > o.max_speed + 1e-8 ||
      a > o.max_longitudinal_acceleration + 1e-8 ||
      std::hypot(a, kmax * vmax * vmax) >
      o.max_total_acceleration + 1e-8)
    {
      result.status =
        ContinuousCheckStatus::DynamicsViolation;
      result.detail = "Speed or acceleration limit exceeded";
      return result;
    }

    // 换向点和终点都是强制停车边界。
    for (const double stop : spatial.stops) {
      if (std::abs(p.s0 - stop) <= 1e-7 &&
        p.v0 > 1e-8)
      {
        result.status =
          ContinuousCheckStatus::DynamicsViolation;
        result.detail = "Nonzero speed at mandatory stop boundary";
        return result;
      }

      if (stop > p.s0 + 1e-7 &&
        stop <= end_s + 1e-7)
      {
        // 不能以非零速度穿过停车边界。
        // 必须在此段运动结束时停在该边界。
        if (end_v > 1e-8 ||
          std::abs(end_s - stop) > 1e-7)
        {
          result.status =
            ContinuousCheckStatus::DynamicsViolation;
          result.detail = "Motion crosses mandatory stop without stopping";
          return result;
        }
      }
    }

    // 任意车身点速度上界：
    // 后轴平移速度 + 最远角点半径 × 角速度。
    own_rate = std::max(
      own_rate,
      vmax * (1 + bodyRadius(own_g) * kmax));
  }

  // 固定路径上的静态扫掠覆盖行驶和停车位姿。
  result.static_map_checked =
    !(own_g.map.width == 0 &&
    own_g.map.height == 0 &&
    own_g.map.cells.empty());

  if (result.static_map_checked) {
    HybridAStarPlanner planner(own_g);

    for (std::size_t i = 1; i < own.path().size(); ++i) {
      if (!planner.isArcCollisionFree(
          own.path()[i - 1],
          own.path()[i].signed_distance,
          own.path()[i].curvature))
      {
        result.status =
          ContinuousCheckStatus::CollisionOrStaticBlocked;
        return result;
      }
    }
  }

  double peer_rate = 0;

  for (std::size_t i = 1; i < knots.size(); ++i) {
    const auto & a = knots[i - 1];
    const auto & b = knots[i];

    const double yaw_delta = std::remainder(
      b.pose.yaw - a.pose.yaw, kTwoPi);

    peer_rate = std::max(
      peer_rate,
      (std::hypot(
        b.pose.x - a.pose.x,
        b.pose.y - a.pose.y) +
      bodyRadius(peer_g) * std::abs(yaw_delta)) /
      (b.t - a.t));
  }

  if (!std::isfinite(own_rate) ||
    !std::isfinite(peer_rate))
  {
    return result;  // Unresolved。
  }

  const auto peerPose = [&](double t) {
      auto it = std::upper_bound(
        knots.begin(), knots.end(), t,
        [](double x, const PeerKnot & p) {
          return x < p.t;
        });

      const auto i = std::min<std::size_t>(
        it - knots.begin() - 1,
        knots.size() - 2);

      const auto & a = knots[i];
      const auto & b = knots[i + 1];
      const double f = (t - a.t) / (b.t - a.t);

      return Pose{
        a.pose.x + f * (b.pose.x - a.pose.x),
        a.pose.y + f * (b.pose.y - a.pose.y),
        a.pose.yaw + f * std::remainder(
          b.pose.yaw - a.pose.yaw, kTwoPi)
      };
    };

  struct Interval
  {
    double a, b;
    int depth;
  };

  std::vector<Interval> pending{
    {0, own.duration(), 0}
  };

  // 检查完整时间区间的两个端点。
  for (double t : {0.0, own.duration()}) {
    if (vehicleRectanglesOverlap(
        vehicleRectangle(own.evaluate(t).pose, own_g),
        vehicleRectangle(peerPose(t), peer_g)))
    {
      result.status =
        ContinuousCheckStatus::CollisionOrStaticBlocked;
      return result;
    }
  }

  while (!pending.empty()) {
    if (result.intervals_checked >= o.max_intervals) {
      return result;
    }

    const auto interval = pending.back();
    pending.pop_back();
    ++result.intervals_checked;

    const double middle =
      0.5 * (interval.a + interval.b);
    const double half =
      0.5 * (interval.b - interval.a);

    const Pose a = own.evaluate(middle).pose;
    const Pose b = peerPose(middle);

    // 原始车身相交：已找到实际模型碰撞。
    if (vehicleRectanglesOverlap(
        vehicleRectangle(a, own_g),
        vehicleRectangle(b, peer_g)))
    {
      result.status =
        ContinuousCheckStatus::CollisionOrStaticBlocked;
      return result;
    }

    // 中点车身按运动上界膨胀后，
    // 分别覆盖各自在整个区间内的扫掠范围。
    // 两个包络分离，才证明整个区间安全。
    if (!vehicleRectanglesOverlap(
        vehicleRectangle(
          a, own_g, own_rate * half + 1e-7),
        vehicleRectangle(
          b, peer_g, peer_rate * half + 1e-7)))
    {
      continue;
    }

    if (interval.depth >= o.max_depth ||
      interval.b - interval.a <= o.min_interval ||
      middle == interval.a ||
      middle == interval.b)
    {
      return result;  // Unresolved，绝不能当作 Safe。
    }

    pending.push_back({
      middle, interval.b, interval.depth + 1
    });
    pending.push_back({
      interval.a, middle, interval.depth + 1
    });
  }

  result.status = ContinuousCheckStatus::Safe;
  return result;
}

}  // namespace car_swarm_agent
