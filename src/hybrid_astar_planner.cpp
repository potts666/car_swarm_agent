#include "car_swarm_agent/hybrid_astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <vector>
#include <cstddef>
#include <queue>

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

    }  // namespace

    HybridAStarPlanner::HybridAStarPlanner(PlannerConfig config)
    : config_(config)//构造函数
    {
        config_.step_size = std::max(0.001, config_.step_size);
        config_.wheel_base = std::max(0.001, config_.wheel_base);
        config_.max_steer_angle = std::max(0.0, config_.max_steer_angle);
        config_.steering_samples = std::max(2, config_.steering_samples);
        config_.goal_tolerance = std::max(0.001, config_.goal_tolerance);
        config_.max_iterations = std::max(1, config_.max_iterations);
        //至少保留“左转、右转”两个候选，后面不会出现除以零。
    }
    //段数至少是 1；
    //步长至少是 0.001 m；
    //轴距至少是 0.001 m，避免除以零；
    //最大转角不能是负数。

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
            parent_index};
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
        std::vector<SearchNode> all_nodes;
        all_nodes.reserve(
            static_cast<std::size_t>(config_.max_iterations) + 1);

        std::priority_queue<
            SearchNode,
            std::vector<SearchNode>,
            SearchNodeCompare> open_set;

        SearchNode start_node{
            start,
            0.0,
            heuristic(start, goal),
            0,
            0};

        all_nodes.push_back(start_node);
        open_set.push(start_node);

        std::size_t best_index = 0;

        for (
            int iteration = 0;
            iteration < config_.max_iterations && !open_set.empty();
            ++iteration)
        {
            const SearchNode current = open_set.top();
            open_set.pop();

            if (current.h_cost < all_nodes[best_index].h_cost) {
                best_index = current.node_index;
            }

            if (isGoalReached(current.pose, goal)) {
                best_index = current.node_index;
                break;
            }

            const std::vector<Pose> successors =
            generateSuccessors(current.pose);

            for (const Pose & successor_pose : successors) {
                const std::size_t child_index = all_nodes.size();

                SearchNode child = makeChildNode(
                    current,
                    successor_pose,
                    goal,
                    current.node_index);

                child.node_index = child_index;

                all_nodes.push_back(child);
                open_set.push(child);
            }
        }

        std::vector<Pose> path;

        for (
            std::size_t index = best_index;
            ;
            index = all_nodes[index].parent_index)
        {
            path.push_back(all_nodes[index].pose);

            if (index == 0) {
                break;
            }
        }

        std::reverse(path.begin(), path.end());

        return path;
    }

}