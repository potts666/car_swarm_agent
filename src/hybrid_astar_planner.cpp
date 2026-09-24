#include "car_swarm_agent/hybrid_astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

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

    }  // namespace
    HybridAStarPlanner::HybridAStarPlanner(PlannerConfig config)
    : config_(config)//构造函数
    {
        config_.num_segments = std::max(1, config_.num_segments);
        config_.step_size = std::max(0.001, config_.step_size);
        config_.wheel_base = std::max(0.001, config_.wheel_base);
        config_.max_steer_angle = std::max(0.0, config_.max_steer_angle);
        config_.steering_samples = std::max(2, config_.steering_samples);
        //至少保留“左转、右转”两个候选，后面不会出现除以零。
    }
    //段数至少是 1；
    //步长至少是 0.001 m；
    //轴距至少是 0.001 m，避免除以零；
    //最大转角不能是负数。

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
    std::vector<Pose> HybridAStarPlanner::plan(const Pose& start, const Pose& goal) const {
        std::vector<Pose> path;
        path.reserve(static_cast<std::size_t>(config_.num_segments) + 1);// 预留空间，避免多次分配 static_cast含义是将num_segments_转换为size_t类型，确保在调用reserve时不会发生类型不匹配或溢出问题

        // 计算每个分段的增量
        double dx = (goal.x - start.x) ;
        double dy = (goal.y - start.y) ;
        double yaw = std::atan2(dy, dx) ;

        // 生成路径点
        for (int i = 0; i <= config_.num_segments; ++i) {
            const double t = static_cast<double>(i) / config_.num_segments;

            path.push_back(Pose{
                start.x + t * dx,
                start.y + t * dy,
                yaw
            });
        }

        return path;
    }

}