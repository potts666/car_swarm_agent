#ifndef CAR_SWARM_AGENT_HYBRID_ASTAR_PLANNER_HPP
#define CAR_SWARM_AGENT_HYBRID_ASTAR_PLANNER_HPP

#include <vector>

namespace car_swarm_agent {

    struct Pose{
        
        double x;
        double y;
        double yaw;
    };

    class HybridAStarPlanner {
    public:
        explicit HybridAStarPlanner(int num_segments = 10);// 构造函数，默认分段数为 10

        std::vector<Pose> plan(const Pose& start, const Pose& goal) const;// 规划函数，输入起点和终点，返回路径点的向量

    private:
        int num_segments_;
    };
}

#endif // CAR_SWARM_AGENT_HYBRID_ASTAR_PLANNER_HPP
// 这一份 .hpp 是“声明书”：只告诉其他代码，这个规划器有什么数据、能做什么；具体怎样生成路径，下一步写进 .cpp