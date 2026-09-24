#ifndef CAR_SWARM_AGENT_HYBRID_ASTAR_PLANNER_HPP
#define CAR_SWARM_AGENT_HYBRID_ASTAR_PLANNER_HPP

#include <vector>
#include <cstddef>
namespace car_swarm_agent {

    struct Pose{
        
        double x;
        double y;
        double yaw;
    };

    struct PlannerConfig
    {
    double step_size{1.0};
    double wheel_base{2.7};
    double max_steer_angle{0.5};
    int steering_samples{3};
    double goal_tolerance{0.5};// 车辆离终点小于 0.5 m，就认为已经到达
    int max_iterations{1000};
    };

    struct SearchNode
    {
        Pose pose;
        double g_cost{0.0};
        double h_cost{0.0};
        std::size_t parent_index{0}; // 我是从哪个节点扩展来的
        std::size_t node_index{0}; // 我自己在全部节点数组中的编号

        double totalCost() const
        {
            return g_cost + h_cost;
        }
    };

    class HybridAStarPlanner {
    public:
        explicit HybridAStarPlanner(PlannerConfig config = {});// 构造函数

        std::vector<Pose> plan(const Pose& start, const Pose& goal) const;// 规划函数，输入起点和终点，返回路径点的向量
        Pose propagate(const Pose & pose, double steering_angle) const;// 给当前车辆姿态 pose 和前轮转向角 steering_angle，
        //根据 PlannerConfig 中的 step_size、wheel_base、max_steer_angle，向前模拟一步，得到下一个姿态
        std::vector<Pose> generateSuccessors(const Pose & pose) const;
    private:
        double heuristic(const Pose & from, const Pose & goal) const;
        bool isGoalReached(const Pose & pose, const Pose & goal) const;
        SearchNode makeChildNode(
            const SearchNode & parent, // 当前被拓展的旧节点
            const Pose & child_pose, // 车辆向前走一步后得到的新姿态
            const Pose & goal, // 终点 用来计算新节点的h_cost
            std::size_t parent_index) const; // 父节点在全部节点数组中的编号
        PlannerConfig config_;
    };
}

#endif // CAR_SWARM_AGENT_HYBRID_ASTAR_PLANNER_HPP
// 这一份 .hpp 是“声明书”：只告诉其他代码，这个规划器有什么数据、能做什么；具体怎样生成路径，下一步写进 .cpp