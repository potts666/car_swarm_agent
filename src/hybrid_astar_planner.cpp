#include "car_swarm_agent/hybrid_astar_planner.hpp"
#include "car_swarm_agent/reeds_shepp.hpp"
#include "car_swarm_agent/vehicle_geometry.hpp"
#include <stdexcept>
#include <chrono>
#include <limits>
#include <algorithm>
#include <cmath>
#include <vector>
#include <cstddef>
#include <queue>
#include <map>
#include <set>
#include <tuple>

namespace car_swarm_agent {
    namespace
    {

        constexpr double kPi = 3.14159265358979323846;

        double normalizeAngle(double angle)// 将yaw角度控制在[-pi ,pi )之间
        {
            while (angle > kPi) {
                angle -= 2.0 * kPi;
            }

            while (angle <= -kPi) {
                angle += 2.0 * kPi;
            }

        return angle;
        }

        bool segmentIntersectsBox(
            double x0, double y0, double x1, double y1,
            double left, double bottom, double right, double top)
        {
            const double dx = x1 - x0;
            const double dy = y1 - y0;
            double enter = 0.0;
            double leave = 1.0;

            const auto clip = [&](double p, double q) {
                if (std::abs(p) < 1e-12) {
                    return q >= 0.0;
                }

                const double t = q / p;

                if (p < 0.0) {
                    enter = std::max(enter, t);
                } else {
                    leave = std::min(leave, t);
                }

                return enter <= leave;
            };

            return clip(-dx, x0 - left) &&
                   clip( dx, right - x0) &&
                   clip(-dy, y0 - bottom) &&
                   clip( dy, top - y0);
        }

        using GridKey = std::tuple<int, int, int>; // 用三个离散编号代表一个搜索状态

        constexpr double kCostEpsilon = 1e-9;
        //constexpr 表示它在编译期就确定，属于常量表达式；也就是说，它不会在运行时变化，而且通常会被编译器优化成直接常量
        GridKey makeGridKey(
            const Pose & pose, // 通过引用读取原对象，避免复制，同时不修改它
            const PlannerConfig & config)
            {
                const int x_bin = static_cast<int>(
                    std::floor(pose.x / config.grid_resolution));
                    //调用 std::floor 把结果向下取整，最后强制转换成 int，得到一个整数索引 x_bin
                const int y_bin = static_cast<int>(
                    std::floor(pose.y / config.grid_resolution));

                const int yaw_bins = static_cast<int>(
                    std::ceil(2.0 * kPi / config.yaw_resolution));

                const double yaw = normalizeAngle(pose.yaw);

                const int raw_yaw_bin = static_cast<int>(
                    std::floor((yaw + kPi) / config.yaw_resolution));

                const int yaw_bin = std::clamp(
                    raw_yaw_bin,
                    0,
                    yaw_bins - 1);
                    //但这里还有一个关键步骤：std::clamp。它保证 raw_yaw_bin 一定落在合法的范围内：
                    //最小不能小于 0
                    //最大不能超过 yaw_bins - 1
                return GridKey{x_bin, y_bin, yaw_bin};
            }

        struct SearchNodeCompare
        // 它的作用是比较两个 SearchNode：
        //- left.totalCost() > right.totalCost() 为真，表示 left 比 right 更差；
        //- 所以以后“总代价更小”的节点会排在前面，最先被取出扩展；
        //- 这个看似反直觉的 > 是为了配合 C++ 的 priority_queue：默认它是“大数优先”，我们用这个比较器把它改成“小总代价优先”。
        {
            bool operator()(const SearchNode & left, const SearchNode & right) const
            {
                return left.totalCost() > right.totalCost();
            }
        };

    }  // namespace 匿名命名空间

    const char * stopReasonName(StopReason reason)
    {
        switch (reason) {
            case StopReason::GoalReached: return "goal_reached";
            case StopReason::IterationLimit: return "iteration_limit";
            case StopReason::OpenExhausted: return "open_exhausted";
            case StopReason::InvalidStart: return "invalid_start";
            case StopReason::InvalidGoal: return "invalid_goal";
            case StopReason::PathCollision: return "path_collision";
            default: return "not_started";
        }
    }

    bool OccupancyGrid::isOccupied(double x, double y) const
    {
        // 默认空地图：沿用现有的无障碍演示
        if (width == 0 && height == 0 && cells.empty()) {
            return false;
        }

        if (width <= 0 || height <= 0 ||
            !std::isfinite(resolution) || resolution <= 0.0 ||
            cells.size() !=
            static_cast<std::size_t>(width) *
            static_cast<std::size_t>(height)) {
            return true; // 地图无效时按不可通行处理
        }

        const double column = (x - origin_x) / resolution;
        const double row = (y - origin_y) / resolution;

        if (!std::isfinite(column) || !std::isfinite(row) ||
            column < 0.0 || row < 0.0 ||
            column >= width || row >= height) {
            return true; // 地图外不可通行
        }

        const auto col = static_cast<std::size_t>(std::floor(column));
        const auto r = static_cast<std::size_t>(std::floor(row));

        return cells[r * static_cast<std::size_t>(width) + col] != 0;
    }
    // 这些辅助函数、常量、类型、GridKey 等，只在这个 .cpp 文件中可见，不会暴露给其他文件。
    // 这样能减少命名污染，避免多个文件里有同名函数时发生链接错误

    bool OccupancyGrid::isSegmentFree(
        double x0, double y0, double x1, double y1) const
    {
        if (isOccupied(x0, y0) || isOccupied(x1, y1)) {
            return false;
        }

        if (width == 0 && height == 0) {
            return true;
        }

        const double col0 = (x0 - origin_x) / resolution;
        const double col1 = (x1 - origin_x) / resolution;
        const double row0 = (y0 - origin_y) / resolution;
        const double row1 = (y1 - origin_y) / resolution;

        const int first_col = std::max(
            0, static_cast<int>(std::floor(std::min(col0, col1))) - 1);
        const int last_col = std::min(
            width - 1,
            static_cast<int>(std::floor(std::max(col0, col1))) + 1);
        const int first_row = std::max(
            0, static_cast<int>(std::floor(std::min(row0, row1))) - 1);
        const int last_row = std::min(
            height - 1,
            static_cast<int>(std::floor(std::max(row0, row1))) + 1);

        for (int row = first_row; row <= last_row; ++row) {
            for (int col = first_col; col <= last_col; ++col) {
                const auto index =
                    static_cast<std::size_t>(row) *
                    static_cast<std::size_t>(width) +
                    static_cast<std::size_t>(col);

                if (cells[index] == 0) {
                    continue;
                }

                const double left = origin_x + col * resolution;
                const double bottom = origin_y + row * resolution;

                if (segmentIntersectsBox(
                        x0, y0, x1, y1,
                        left, bottom,
                        left + resolution, bottom + resolution)) {
                    return false;
                }
            }
        }

        return true;
    }

    HybridAStarPlanner::HybridAStarPlanner(PlannerConfig config)
    : config_(config)//构造函数
    //创建 HybridAStarPlanner 对象时，先把传入的 config 赋给成员变量 config_，然后再做参数修正和初始化
    //构造函数就是“对象出生时的初始化流程”
    {
        if (!std::isfinite(config_.max_steer_angle) || config_.max_steer_angle < 0.0 ||
            config_.max_steer_angle >= kPi / 2.0 ||
            !std::isfinite(config_.analytic_max_distance) || config_.analytic_max_distance <= 0.0 ||
            (config_.analytic_expansion && !config_.allow_reverse)) {
            throw std::invalid_argument("Invalid steering or Reeds-Shepp configuration");
        }
        config_.analytic_expansion_interval = std::max(1, config_.analytic_expansion_interval);
        config_.step_size = std::max(0.001, config_.step_size);
        config_.wheel_base = std::max(0.001, config_.wheel_base);
        config_.max_steer_angle = std::max(0.0, config_.max_steer_angle);
        config_.steering_samples = std::max(2, config_.steering_samples);
        config_.goal_tolerance = std::max(0.001, config_.goal_tolerance);
        config_.goal_yaw_tolerance = std::clamp(
            config_.goal_yaw_tolerance, 0.001, kPi);
        config_.max_iterations = std::max(1, config_.max_iterations);
        //至少保留“左转、右转”两个候选，后面不会出现除以零。
        config_.grid_resolution = std::max(
        0.001,
        config_.grid_resolution);// 把连续 x, y 归入一个离散格子

        config_.yaw_resolution = std::clamp(
            config_.yaw_resolution,
            0.01,
            2.0 * kPi);// 把连续朝向归入一个角度格子
            if (!std::isfinite(config_.vehicle_length) ||
            !std::isfinite(config_.vehicle_width) ||
            !std::isfinite(config_.rear_overhang) ||
            !std::isfinite(config_.collision_margin) ||
            !std::isfinite(config_.collision_check_step) ||
            config_.vehicle_length <= 0.0 ||
            config_.vehicle_width <= 0.0 ||
            config_.rear_overhang < 0.0 ||
            config_.rear_overhang >= config_.vehicle_length ||
            config_.collision_margin < 0.0 ||
            config_.collision_check_step <= 0.0)
            {
                throw std::invalid_argument(
                    "Invalid vehicle collision configuration");
            }
    }
    //段数至少是 1；
    //步长至少是 0.001 m；
    //轴距至少是 0.001 m，避免除以零；
    //最大转角不能是负数。
    bool HybridAStarPlanner::isFootprintFree(
        const Pose & pose,
        double extra_margin) const
    {
        if (!std::isfinite(pose.x) ||
            !std::isfinite(pose.y) ||
            !std::isfinite(pose.yaw) ||
            !std::isfinite(extra_margin) ||
            extra_margin < 0.0)
        {
            return false;
        }

        const OccupancyGrid & map = config_.map;

        // 默认空地图：保留无障碍规划行为。
        if (map.width == 0 &&
            map.height == 0 &&
            map.cells.empty())
        {
            return true;
        }

        if (map.width <= 0 ||
            map.height <= 0 ||
            !std::isfinite(map.resolution) ||
            map.resolution <= 0.0 ||
            !std::isfinite(map.origin_x) ||
            !std::isfinite(map.origin_y) ||
            map.cells.size() !=
                static_cast<std::size_t>(map.width) *
                static_cast<std::size_t>(map.height))
        {
            return false;
        }

        const auto body = vehicleRectangle(pose, config_, extra_margin);
        const double half_length = body.half_length;
        const double half_width = body.half_width;
        const double c = body.c;
        const double s = body.s;
        const double center_x = body.center_x;
        const double center_y = body.center_y;

        // 旋转车身矩形在世界坐标下的轴对齐包围盒。
        const double extent_x =
            half_length * std::abs(c) +
            half_width * std::abs(s);

        const double extent_y =
            half_length * std::abs(s) +
            half_width * std::abs(c);

        const double min_x = center_x - extent_x;
        const double max_x = center_x + extent_x;
        const double min_y = center_y - extent_y;
        const double max_y = center_y + extent_y;

        const double map_right =
            map.origin_x + map.width * map.resolution;

        const double map_top =
            map.origin_y + map.height * map.resolution;

        if (!std::isfinite(min_x) ||
            !std::isfinite(max_x) ||
            !std::isfinite(min_y) ||
            !std::isfinite(max_y) ||
            !std::isfinite(map_right) ||
            !std::isfinite(map_top))
        {
            return false;
        }

        // 接触或越过地图外边界，按不可通行处理。
        if (min_x <= map.origin_x ||
            max_x >= map_right ||
            min_y <= map.origin_y ||
            max_y >= map_top)
        {
            return false;
        }

        // 多检查一圈，包含恰好接触车身包围盒边界的障碍格。
        const int first_col = std::max(
            0,
            static_cast<int>(
                std::floor((min_x - map.origin_x) /
                        map.resolution)) - 1);

        const int last_col = std::min(
            map.width - 1,
            static_cast<int>(
                std::floor((max_x - map.origin_x) /
                        map.resolution)) + 1);

        const int first_row = std::max(
            0,
            static_cast<int>(
                std::floor((min_y - map.origin_y) /
                        map.resolution)) - 1);

        const int last_row = std::min(
            map.height - 1,
            static_cast<int>(
                std::floor((max_y - map.origin_y) /
                        map.resolution)) + 1);

        const double cell_half = map.resolution * 0.5;
        constexpr double epsilon = 1e-9;

        for (int row = first_row; row <= last_row; ++row) {
            for (int col = first_col; col <= last_col; ++col) {
                const auto index =
                    static_cast<std::size_t>(row) *
                    static_cast<std::size_t>(map.width) +
                    static_cast<std::size_t>(col);

                if (map.cells[index] == 0) {
                    continue;
                }

                const double cell_x =
                    map.origin_x +
                    (col + 0.5) * map.resolution;

                const double cell_y =
                    map.origin_y +
                    (row + 0.5) * map.resolution;

                const double dx = cell_x - center_x;
                const double dy = cell_y - center_y;

                // SAT：世界 x 轴。 这里的 SAT 会检查整个矩形，因此能够检测“障碍在车身内部，但四个角都在空地”的情况。
                if (std::abs(dx) >
                    extent_x + cell_half + epsilon)
                {
                    continue;
                }

                // SAT：世界 y 轴。
                if (std::abs(dy) >
                    extent_y + cell_half + epsilon)
                {
                    continue;
                }

                // 栅格矩形在车身纵向、横向轴上的投影半径。
                const double cell_projection =
                    cell_half * (std::abs(c) + std::abs(s));

                // SAT：车身纵向轴。
                const double longitudinal = dx * c + dy * s;

                if (std::abs(longitudinal) >
                    half_length + cell_projection + epsilon)
                {
                    continue;
                }

                // SAT：车身横向轴。
                const double lateral = -dx * s + dy * c;

                if (std::abs(lateral) >
                    half_width + cell_projection + epsilon)
                {
                    continue;
                }

                // 四个轴都无法分离：车身与障碍格相交。
                return false;
            }
        }

        return true;
    }

    bool HybridAStarPlanner::isPoseCollisionFree(
        const Pose & pose) const
    {
        return isFootprintFree(pose, 0.0);
    }

    bool HybridAStarPlanner::isMotionCollisionFree(
        const Pose & from,
        const Pose & to) const
    {
        if (!isPoseCollisionFree(from) ||
            !isPoseCollisionFree(to))
        {
            return false;
        }

        const OccupancyGrid & map = config_.map;

        if (map.width == 0 &&
            map.height == 0 &&
            map.cells.empty())
        {
            return true;
        }

        const double dx = to.x - from.x;
        const double dy = to.y - from.y;

        // 已检查位姿有限；差值仍可能溢出。
        const double raw_yaw_delta = to.yaw - from.yaw;

        if (!std::isfinite(dx) ||
            !std::isfinite(dy) ||
            !std::isfinite(raw_yaw_delta))
        {
            return false;
        }

        const double yaw_delta =
            std::remainder(raw_yaw_delta, 2.0 * kPi);

        const double distance = std::hypot(dx, dy);

        // 包含安全余量的车身，离后轴中心最远的角点半径。
        const double longitudinal_radius =
            std::max(
                config_.rear_overhang,
                config_.vehicle_length - config_.rear_overhang) +
            config_.collision_margin;

        const double lateral_radius =
            config_.vehicle_width * 0.5 +
            config_.collision_margin;

        const double radius =
            std::hypot(longitudinal_radius, lateral_radius);

        // 任意车身点在整段插值中的位移上界：
        // 后轴平移距离 + 旋转角度 × 角点半径。
        const double motion_bound =
            distance + radius * std::abs(yaw_delta);

        const double required_samples = std::ceil(
            motion_bound / config_.collision_check_step);

        // 异常大输入按不可通行处理，避免溢出或无界循环。
        if (!std::isfinite(required_samples) ||
            required_samples > 1000000.0)
        {
            return false;
        }

        const int samples = std::max(
            1, static_cast<int>(required_samples));

        // 任意时刻距最近采样时刻最多半个间隔。
        // 用这个上界膨胀采样车身，覆盖采样间的运动。
        const double sweep_margin =
            motion_bound / (2.0 * samples) + 1e-9;

        for (int i = 0; i <= samples; ++i) {
            const double t = static_cast<double>(i) / samples;

            const Pose sample{
                from.x + t * dx,
                from.y + t * dy,
                from.yaw + t * yaw_delta};

            if (!isFootprintFree(sample, sweep_margin)) {
                return false;
            }
        }

        return true;
    }
    std::vector<double> HybridAStarPlanner::buildObstacleDistances(
        const Pose & goal) const
    {
        const auto & map = config_.map;
        if (map.cells.empty()) { return {}; }
        const double infinity = std::numeric_limits<double>::infinity();
        std::vector<double> distances(map.cells.size(), infinity);
        using Entry = std::pair<double, std::size_t>;
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
        const int gx = static_cast<int>(std::floor((goal.x - map.origin_x) / map.resolution));
        const int gy = static_cast<int>(std::floor((goal.y - map.origin_y) / map.resolution));
        const auto goal_index = static_cast<std::size_t>(gy) * map.width + gx;
        distances[goal_index] = 0.0;
        queue.emplace(0.0, goal_index);
        while (!queue.empty()) {
            const auto [cost, index] = queue.top();
            queue.pop();
            if (cost > distances[index]) { continue; }
            const int x = static_cast<int>(index % map.width);
            const int y = static_cast<int>(index / map.width);
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) { continue; }
                    const int nx = x + dx, ny = y + dy;
                    if (nx < 0 || ny < 0 || nx >= map.width || ny >= map.height) { continue; }
                    const auto next = static_cast<std::size_t>(ny) * map.width + nx;
                    if (map.cells[next] != 0) { continue; }
                    // Diagonal moves cannot cut through occupied corners.
                    if (dx != 0 && dy != 0 &&
                        (map.cells[static_cast<std::size_t>(y) * map.width + nx] != 0 ||
                         map.cells[static_cast<std::size_t>(ny) * map.width + x] != 0)) { continue; }
                    const double candidate = cost + map.resolution * std::hypot(dx, dy);
                    if (candidate < distances[next]) {
                        distances[next] = candidate;
                        queue.emplace(candidate, next);
                    }
                }
            }
        }
        return distances;
    }

    double HybridAStarPlanner::turnHeuristic(const Pose & from, const Pose & goal) const
    {
        if (config_.allow_reverse) {
            const double curvature = std::tan(config_.max_steer_angle) / config_.wheel_base;
            if (curvature > 1e-12) {
                return detail::shortestReedsShepp(from, goal, 1.0 / curvature).length() / curvature;
            }
            return std::hypot(goal.x - from.x, goal.y - from.y);
        }
        // Forward-only Dubins relaxation: six combinations of maximum-curvature
        // arcs and straight segments, with no obstacle or footprint constraints.
        const double curvature = std::tan(config_.max_steer_angle) / config_.wheel_base;
        const double dx = goal.x - from.x, dy = goal.y - from.y;
        const double distance = std::hypot(dx, dy);
        if (distance <= config_.goal_tolerance &&
            std::abs(normalizeAngle(goal.yaw - from.yaw)) <= config_.goal_yaw_tolerance) { return 0.0; }
        if (curvature <= 1e-12) {
            const double lateral = -dx * std::sin(from.yaw) + dy * std::cos(from.yaw);
            return std::abs(lateral) < 1e-9 &&
                dx * std::cos(from.yaw) + dy * std::sin(from.yaw) >= 0.0 &&
                std::abs(normalizeAngle(goal.yaw - from.yaw)) <= config_.goal_yaw_tolerance
                ? distance : std::numeric_limits<double>::infinity();
        }
        // Close to the goal, Euler discretization and the accepted goal region
        // can turn an exact Dubins connection into an unnecessary full loop.
        // Use a model-consistent relaxed bound in this region instead.
        if (distance < 2.0 / curvature) {
            const double yaw_error = std::max(0.0,
                std::abs(normalizeAngle(goal.yaw - from.yaw)) - config_.goal_yaw_tolerance);
            const double lateral = std::max(0.0,
                std::abs(-dx * std::sin(from.yaw) + dy * std::cos(from.yaw)) - config_.goal_tolerance);
            double bound = std::max({std::max(0.0, distance - config_.goal_tolerance),
                yaw_error / curvature, std::sqrt(2.0 * lateral / curvature)});
            if (dx * std::cos(from.yaw) + dy * std::sin(from.yaw) < -config_.goal_tolerance) {
                bound = std::max(bound, kPi / (2.0 * curvature));
            }
            return bound;
        }
        const auto mod = [](double v) { v = std::fmod(v, 2.0 * kPi); return v < 0.0 ? v + 2.0 * kPi : v; };
        const double theta = std::atan2(dy, dx);
        const double a = mod(from.yaw - theta), b = mod(goal.yaw - theta);
        const double d = distance * curvature;
        const double sa = std::sin(a), sb = std::sin(b), ca = std::cos(a), cb = std::cos(b);
        const double cab = std::cos(a - b);
        double best = std::numeric_limits<double>::infinity();
        const auto accept = [&](double t, double p, double q) { best = std::min(best, mod(t) + p + mod(q)); };
        double p2 = 2.0 + d*d - 2.0*cab + 2.0*d*(sa-sb);
        if (p2 >= 0.0) { const double t = std::atan2(cb-ca, d+sa-sb); accept(-a+t, std::sqrt(p2), b-t); }
        p2 = 2.0 + d*d - 2.0*cab + 2.0*d*(sb-sa);
        if (p2 >= 0.0) { const double t = std::atan2(ca-cb, d-sa+sb); accept(a-t, std::sqrt(p2), -b+t); }
        p2 = -2.0 + d*d + 2.0*cab + 2.0*d*(sa+sb);
        if (p2 >= 0.0) { const double p = std::sqrt(p2); const double t = std::atan2(-ca-cb,d+sa+sb)-std::atan2(-2.0,p); accept(-a+t,p,-b+t); }
        p2 = d*d - 2.0 + 2.0*cab - 2.0*d*(sa+sb);
        if (p2 >= 0.0) { const double p = std::sqrt(p2); const double t = std::atan2(ca+cb,d-sa-sb)-std::atan2(2.0,p); accept(a-t,p,b-t); }
        double v = (6.0-d*d+2.0*cab+2.0*d*(sa-sb))/8.0;
        if (std::abs(v) <= 1.0) { const double p = mod(2.0*kPi-std::acos(v)); const double t = mod(a-std::atan2(ca-cb,d-sa+sb)+p/2.0); accept(t,p,a-b-t+p); }
        v = (6.0-d*d+2.0*cab+2.0*d*(-sa+sb))/8.0;
        if (std::abs(v) <= 1.0) { const double p = mod(2.0*kPi-std::acos(v)); const double t = mod(-a-std::atan2(ca-cb,d+sa-sb)+p/2.0); accept(t,p,b-a-t+p); }
        return best / curvature;
    }

    double HybridAStarPlanner::heuristic(
        const Pose & from, const Pose & goal,
        const std::vector<double> & obstacle_distances) const
    {
        const double euclidean = std::hypot(goal.x - from.x, goal.y - from.y);
        if (config_.heuristic_mode == HeuristicMode::Euclidean) { return euclidean; }
        double obstacle = euclidean;
        if (!obstacle_distances.empty()) {
            const auto & map = config_.map;
            const int x = static_cast<int>(std::floor((from.x - map.origin_x) / map.resolution));
            const int y = static_cast<int>(std::floor((from.y - map.origin_y) / map.resolution));
            obstacle = obstacle_distances[static_cast<std::size_t>(y) * map.width + x];
        }
        return config_.heuristic_mode == HeuristicMode::Dual
            ? std::max(obstacle, turnHeuristic(from, goal)) : obstacle;
    }
   
    bool HybridAStarPlanner::isGoalReached(
        const Pose & pose,
        const Pose & goal) const
    {
        return std::hypot(pose.x - goal.x, pose.y - goal.y) <= config_.goal_tolerance &&
            std::abs(normalizeAngle(pose.yaw - goal.yaw)) <= config_.goal_yaw_tolerance;
    }

    SearchNode HybridAStarPlanner::makeChildNode(
        const SearchNode & parent,
        const Pose & child_pose,
        const Pose & goal,
        const std::vector<double> & obstacle_distances,
        std::size_t parent_index) const
    {
        const double step_cost = std::abs(child_pose.signed_distance);

        return SearchNode{
            child_pose,
            parent.g_cost + step_cost,
            heuristic(child_pose, goal, obstacle_distances),
            parent_index,
            0};
    }

    Pose HybridAStarPlanner::propagate(
        //基本运动学代码化 下一时刻的x坐标 y坐标 yaw坐标计算
        const Pose & pose,
        double steering_angle, int direction) const
    {
        const double steering = std::clamp(
            steering_angle,
            -config_.max_steer_angle,
            config_.max_steer_angle);

        if (direction != 1 && direction != -1) { throw std::invalid_argument("Direction must be +1 or -1"); }
        if (direction < 0 && !config_.allow_reverse) { throw std::invalid_argument("Reverse is disabled"); }
        if (config_.allow_reverse) {
            return detail::integrateArc(pose, direction * config_.step_size,
                std::tan(steering) / config_.wheel_base);
        }
        Pose next = pose;
        next.signed_distance = config_.step_size;
        next.curvature = 0.0; // Historical Euler segment is checked as an interpolated motion.

        next.x += config_.step_size * std::cos(pose.yaw);
        next.y += config_.step_size * std::sin(pose.yaw);
        next.yaw = normalizeAngle(
            pose.yaw +
            config_.step_size / config_.wheel_base * std::tan(steering));

        return next;
    }

    std::vector<Pose> HybridAStarPlanner::generateSuccessors(
        //i = 0 → steering = -max_steer_angle
        //i = 1 → steering = 0
        //i = 2 → steering = +max_steer_angle
        const Pose & pose) const
    {
        std::vector<Pose> successors;
        successors.reserve(
            static_cast<std::size_t>(config_.steering_samples));

        for (int i = 0; i < config_.steering_samples; ++i) {
            const double ratio =
            static_cast<double>(i) / (config_.steering_samples - 1);

            const double steering =
            -config_.max_steer_angle +
            ratio * 2.0 * config_.max_steer_angle;

            successors.push_back(propagate(pose, steering));
            if (config_.allow_reverse) { successors.push_back(propagate(pose, steering, -1)); }
        }

        return successors;
    }

    bool HybridAStarPlanner::isArcCollisionFree(
        const Pose & from, double signed_distance, double curvature) const
    {
        const double radius = std::hypot(
            std::max(config_.rear_overhang, config_.vehicle_length - config_.rear_overhang) + config_.collision_margin,
            config_.vehicle_width / 2.0 + config_.collision_margin);
        const double bound = std::abs(signed_distance) * (1.0 + radius * std::abs(curvature));
        const double required = std::ceil(bound / config_.collision_check_step);
        if (!std::isfinite(required) || required > 1000000.0) { return false; }
        const int samples = std::max(1, static_cast<int>(required));
        // True arc sweep bound between samples; retain the rectangle SAT checker.
        const double margin = bound / (2.0 * samples) + 1e-9;
        for (int i = 0; i <= samples; ++i) {
            if (!isFootprintFree(detail::integrateArc(from,
                    signed_distance * i / samples, curvature), margin)) { return false; }
        }
        return true;
    }

    bool HybridAStarPlanner::tryAnalyticConnection(
        const Pose & from, const Pose & goal, std::vector<Pose> & connection) const
    {
        connection.clear();
        const double curvature = std::tan(config_.max_steer_angle) / config_.wheel_base;
        if (curvature <= 1e-12) { return false; }
        const double radius = 1.0 / curvature;
        const auto solution = detail::shortestReedsShepp(from, goal, radius);
        if (!std::isfinite(solution.length())) { return false; }
        Pose current = from;
        for (int segment = 0; segment < 5; ++segment) {
            const double length = solution.length_[segment] * radius;
            if (std::abs(length) < 1e-10) { continue; }
            const auto type = solution.type_[segment];
            const double k = type == detail::ReedsSheppStateSpace::RS_LEFT ? curvature :
                type == detail::ReedsSheppStateSpace::RS_RIGHT ? -curvature : 0.0;
            const double required = std::ceil(std::abs(length) /
                std::min(config_.step_size, config_.collision_check_step));
            if (!std::isfinite(required) || required > 1000000.0) { connection.clear(); return false; }
            const int count = std::max(1, static_cast<int>(required));
            const double ds = length / count;
            for (int i = 0; i < count; ++i) {
                const auto next = detail::integrateArc(current, ds, k);
                if (!isArcCollisionFree(current, ds, k) || !isMotionCollisionFree(current, next)) {
                    connection.clear();
                    return false;
                }
                connection.push_back(next);
                current = next;
            }
        }
        // Reject any numerical/geometry mismatch; never append an unchecked straight snap.
        if (std::hypot(current.x - goal.x, current.y - goal.y) > 1e-6 ||
            std::abs(normalizeAngle(current.yaw - goal.yaw)) > 1e-6) {
            connection.clear();
            return false;
        }
        return true;
    }

    std::vector<Pose> HybridAStarPlanner::plan(
        const Pose & start,
        const Pose & goal, PlanningStats * stats) const
        {
        PlanningStats local_stats;
        if (!stats) { stats = &local_stats; }
        *stats = {};
        struct Timer {
            PlanningStats * stats;
            std::chrono::steady_clock::time_point start{std::chrono::steady_clock::now()};
            ~Timer() { stats->planning_time_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count(); }
        } timer{stats};
        stats->start_body_free = isPoseCollisionFree(start);
        stats->goal_body_free = isPoseCollisionFree(goal);
        if (!stats->start_body_free || !stats->goal_body_free) {
            stats->stop_reason = stats->start_body_free ? StopReason::InvalidGoal : StopReason::InvalidStart;
            return {};
        }
        const auto record_pose = [&](const Pose & pose) {
            const double position_error = std::hypot(pose.x - goal.x, pose.y - goal.y);
            const double yaw_error = std::abs(normalizeAngle(pose.yaw - goal.yaw));
            if (position_error < stats->closest_position_error ||
                (position_error == stats->closest_position_error && yaw_error < stats->closest_yaw_error)) {
                stats->closest_position_error = position_error;
                stats->closest_yaw_error = yaw_error;
                stats->closest_pose = pose;
            }
            if (position_error <= config_.goal_tolerance && yaw_error < stats->best_yaw_error_near_goal) {
                stats->best_yaw_error_near_goal = yaw_error;
                stats->best_yaw_pose_near_goal = pose;
            }
        };
        record_pose(start);
        const auto obstacle_distances = config_.heuristic_mode == HeuristicMode::Euclidean
            ? std::vector<double>{} : buildObstacleDistances(goal);
        stats->obstacle_table_builds = obstacle_distances.empty() ? 0 : 1;
        std::vector<SearchNode> all_nodes;
        //all_nodes 不是 Open 或 Closed。它是“所有被接受节点的档案库”，专门保存 parent_index；找到目标后才能从末尾一路追溯到起点，得到完整路径
        all_nodes.reserve(
            static_cast<std::size_t>(config_.max_iterations) + 1);

        std::priority_queue<
            SearchNode,// 优先队列里存放的元素类型，代表一个搜索状态。
            std::vector<SearchNode>, // 底层数据结构使用 std::vector 来保存这些节点
            SearchNodeCompare> open_set; // 按 f 排序的候选节点

        std::set<GridKey> closed_set;//创建一个保存 GridKey 的集合。集合里的键不会重复，用来记录已展开的离散状态
        std::map<GridKey, double> best_g_cost;//处理“尚未展开但重复发现”的情况

        SearchNode start_node{
            start,
            0.0,
            heuristic(start, goal, obstacle_distances),
            0,
            0};

        const GridKey start_key = makeGridKey(start, config_);

        all_nodes.push_back(start_node);
        open_set.push(start_node);
        best_g_cost[start_key] = 0.0;

        std::size_t goal_index = 0;
        bool goal_found = false;
        std::vector<Pose> analytic_connection;

        for (
            int iteration = 0;
            iteration < config_.max_iterations && !open_set.empty();
            ++iteration)
        {
            ++stats->iterations;
            const SearchNode current = open_set.top();
            open_set.pop();

            const GridKey current_key =
                makeGridKey(current.pose, config_);
                //根据当前搜索节点 current 的位姿，生成一个离散状态标识 current_key

            const auto best_current =
                best_g_cost.find(current_key);
                //在 best_g_cost 这个映射表中，用当前状态的网格键 current_key 查找对应的记录
                //如果找到该状态，迭代器指向对应的键值对；如果没找到，则返回 best_g_cost.end()。
                // 找到时，可以用 best_current->second 读取记录的代价。
                //auto 的意思是“让编译器根据右边的表达式推断变量类型”，这样就不用手动写出较长的类型名称。
            if (
                best_current == best_g_cost.end() ||
                current.g_cost >
                best_current->second + kCostEpsilon)
            {
                continue;
            }//priority_queue 不能原地修改旧节点的优先级

            if (closed_set.find(current_key) != closed_set.end()) {
                continue;
            }

            closed_set.insert(current_key);
            ++stats->expanded_nodes;

            if (isGoalReached(current.pose, goal)) {
                goal_index = current.node_index;
                goal_found = true;
                break;
            }// 到达目标判断

            if (config_.analytic_expansion &&
                (stats->expanded_nodes - 1) % config_.analytic_expansion_interval == 0 &&
                std::hypot(current.pose.x - goal.x, current.pose.y - goal.y) <= config_.analytic_max_distance) {
                ++stats->analytic_attempts;
                if (tryAnalyticConnection(current.pose, goal, analytic_connection)) {
                    ++stats->analytic_successes;
                    goal_index = current.node_index;
                    goal_found = true;
                    break;
                }
            }
            const std::vector<Pose> successors =
                generateSuccessors(current.pose);

            for (const Pose & successor_pose : successors) {
                const double curvature = successor_pose.curvature;
                if ((config_.allow_reverse && !isArcCollisionFree(current.pose,
                        successor_pose.signed_distance, curvature)) || !isMotionCollisionFree(
                        current.pose,
                        successor_pose))
                {
                    continue;
                }
                record_pose(successor_pose); // Include every collision-free generated pose, even if merged.
                const GridKey child_key =
                    makeGridKey(successor_pose, config_);


                SearchNode child = makeChildNode(
                    current,
                    successor_pose,
                    goal,
                    obstacle_distances,
                    current.node_index);

                const auto known_child =
                    best_g_cost.find(child_key);

                if (
                    known_child != best_g_cost.end() &&
                    child.g_cost >=
                    known_child->second - kCostEpsilon)
                {
                    continue;
                }//若新路线没有更便宜，就不浪费搜索资源

                closed_set.erase(child_key); // Reopen when an inconsistent heuristic finds a better g.
                child.node_index = all_nodes.size();

                all_nodes.push_back(child);
                best_g_cost[child_key] = child.g_cost;
                open_set.push(child);
            }
        }

        stats->open_nodes_remaining = open_set.size();
        if (!goal_found) {
            stats->stop_reason = open_set.empty() ? StopReason::OpenExhausted : StopReason::IterationLimit;
            return {};
        }

        std::vector<Pose> path;

        for (
            std::size_t index = goal_index;
            ;
            index = all_nodes[index].parent_index)
        {
            path.push_back(all_nodes[index].pose);

            if (index == 0) {
                break;
            }
        }

        std::reverse(path.begin(), path.end());
        path.front().signed_distance = 0.0;
        path.front().curvature = 0.0;
        path.insert(path.end(), analytic_connection.begin(), analytic_connection.end());
        for (const Pose & pose : path) {
            if (!isPoseCollisionFree(pose)) {
                stats->stop_reason = StopReason::PathCollision;
                return {};
            }
        }

        for (std::size_t i = 1; i < path.size(); ++i) {
            if (!isMotionCollisionFree(path[i - 1], path[i])) {
                stats->stop_reason = StopReason::PathCollision;
                return {};
            }
        }

        stats->stop_reason = StopReason::GoalReached;
        stats->reached_goal = true;
        for (std::size_t i = 1; i < path.size(); ++i) {
            stats->path_length += std::abs(path[i].signed_distance);
            if (path[i].signed_distance < 0.0) { ++stats->reverse_segments; }
        }
        stats->path_points = path.size();
        return path;
    }

}
