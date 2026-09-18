#include "car_swarm_agent/hybrid_astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace car_swarm_agent {

    HybridAStarPlanner::HybridAStarPlanner(int num_segments)
        : num_segments_(std::max(1, num_segments)) {} 
        // num_segments_(std::max(1, num_segments)) 叫“成员初始化列表”：
        // 把传进来的段数保存到成员变量 num_segments_ 中；同时强制最小为 1，避免后面 ... / num_segments_ 出现除以零

    std::vector<Pose> HybridAStarPlanner::plan(const Pose& start, const Pose& goal) const {
        std::vector<Pose> path;
        path.reserve(static_cast<std::size_t>(num_segments_) + 1);// 预留空间，避免多次分配 static_cast含义是将num_segments_转换为size_t类型，确保在调用reserve时不会发生类型不匹配或溢出问题

        // 计算每个分段的增量
        double dx = (goal.x - start.x) ;
        double dy = (goal.y - start.y) ;
        double yaw = std::atan2(dy, dx) ;

        // 生成路径点
        for (int i = 0; i <= num_segments_; ++i) {
            const double t = static_cast<double>(i) / num_segments_;

            path.push_back(Pose{
                start.x + t * dx,
                start.y + t * dy,
                yaw
            });
        }

        return path;
    }
}