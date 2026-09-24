#ifndef CAR_SWARM_AGENT_HYBRID_ASTAR_PLANNER_HPP
#define CAR_SWARM_AGENT_HYBRID_ASTAR_PLANNER_HPP

#include <vector>

namespace car_swarm_agent {

    struct Pose{
        
        double x;
        double y;
        double yaw;
    };

    struct PlannerConfig
    {
    int num_segments{10};
    double step_size{1.0};
    double wheel_base{2.7};
    double max_steer_angle{0.5};
    int steering_samples{3};
    };

    class HybridAStarPlanner {
    public:
        explicit HybridAStarPlanner(PlannerConfig config = {});// 构造函数，默认分段数为 10

        std::vector<Pose> plan(const Pose& start, const Pose& goal) const;// 规划函数，输入起点和终点，返回路径点的向量
        Pose propagate(const Pose & pose, double steering_angle) const;// 给当前车辆姿态 pose 和前轮转向角 steering_angle，
        //根据 PlannerConfig 中的 step_size、wheel_base、max_steer_angle，向前模拟一步，得到下一个姿态
        std::vector<Pose> generateSuccessors(const Pose & pose) const;
    private:
        PlannerConfig config_;
    };
}

#endif // CAR_SWARM_AGENT_HYBRID_ASTAR_PLANNER_HPP
// 这一份 .hpp 是“声明书”：只告诉其他代码，这个规划器有什么数据、能做什么；具体怎样生成路径，下一步写进 .cpp