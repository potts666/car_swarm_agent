#include "car_swarm_agent/hybrid_astar_planner.hpp"
#include <stdexcept>
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
        config_.step_size = std::max(0.001, config_.step_size);
        config_.wheel_base = std::max(0.001, config_.wheel_base);
        config_.max_steer_angle = std::max(0.0, config_.max_steer_angle);
        config_.steering_samples = std::max(2, config_.steering_samples);
        config_.goal_tolerance = std::max(0.001, config_.goal_tolerance);
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

        const double padding =
            config_.collision_margin + extra_margin;

        // 车身矩形的半长、半宽。
        const double half_length =
            config_.vehicle_length * 0.5 + padding;

        const double half_width =
            config_.vehicle_width * 0.5 + padding;

        const double c = std::cos(pose.yaw);
        const double s = std::sin(pose.yaw);

        // 后轴中心到车身几何中心的纵向距离。
        const double center_offset =
            config_.vehicle_length * 0.5 -
            config_.rear_overhang;

        const double center_x = pose.x + center_offset * c;
        const double center_y = pose.y + center_offset * s;

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
    double HybridAStarPlanner::heuristic(
        //计算的是两点的直线距离
        const Pose & from,
        const Pose & goal) const
    {
        return std::hypot(goal.x - from.x, goal.y - from.y);
    }
   
    bool HybridAStarPlanner::isGoalReached(
        const Pose & pose,
        const Pose & goal) const
    {
        return heuristic(pose, goal) <= config_.goal_tolerance;
    }

    SearchNode HybridAStarPlanner::makeChildNode(
        const SearchNode & parent,
        const Pose & child_pose,
        const Pose & goal,
        std::size_t parent_index) const
    {
        const double step_cost = std::hypot(
            child_pose.x - parent.pose.x,
            child_pose.y - parent.pose.y);

        return SearchNode{
            child_pose,
            parent.g_cost + step_cost,
            heuristic(child_pose, goal),
            parent_index,
            0};
    }

    Pose HybridAStarPlanner::propagate(
        //基本运动学代码化 下一时刻的x坐标 y坐标 yaw坐标计算
        const Pose & pose,
        double steering_angle) const
    {
        const double steering = std::clamp(
            steering_angle,
            -config_.max_steer_angle,
            config_.max_steer_angle);

        Pose next = pose;

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
        }

        return successors;
    }

    std::vector<Pose> HybridAStarPlanner::plan(
        const Pose & start,
        const Pose & goal) const
        {
        if (!isPoseCollisionFree(start) ||
            !isPoseCollisionFree(goal))
        {
            return {};
        }
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
            heuristic(start, goal),
            0,
            0};

        const GridKey start_key = makeGridKey(start, config_);

        all_nodes.push_back(start_node);
        open_set.push(start_node);
        best_g_cost[start_key] = 0.0;

        std::size_t goal_index = 0;
        bool goal_found = false;

        for (
            int iteration = 0;
            iteration < config_.max_iterations && !open_set.empty();
            ++iteration)
        {
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

            if (isGoalReached(current.pose, goal)) {
                goal_index = current.node_index;
                goal_found = true;
                break;
            }// 到达目标判断

            const std::vector<Pose> successors =
                generateSuccessors(current.pose);

            for (const Pose & successor_pose : successors) {
                if (!isMotionCollisionFree(
                        current.pose,
                        successor_pose))
                {
                    continue;
                }
                const GridKey child_key =
                    makeGridKey(successor_pose, config_);

                if (closed_set.find(child_key) != closed_set.end()) {
                    continue;
                }

                SearchNode child = makeChildNode(
                    current,
                    successor_pose,
                    goal,
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

                child.node_index = all_nodes.size();

                all_nodes.push_back(child);
                best_g_cost[child_key] = child.g_cost;
                open_set.push(child);
            }
        }

        if (!goal_found) {
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
        for (const Pose & pose : path) {
            if (!isPoseCollisionFree(pose)) {
                return {};
            }
        }

        for (std::size_t i = 1; i < path.size(); ++i) {
            if (!isMotionCollisionFree(path[i - 1], path[i])) {
                return {};
            }
        }

        return path;
    }

}
