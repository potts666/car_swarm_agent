#ifndef CAR_SWARM_AGENT_HYBRID_ASTAR_PLANNER_HPP
#define CAR_SWARM_AGENT_HYBRID_ASTAR_PLANNER_HPP

#include <vector>
#include <cstddef>
#include <limits>
namespace car_swarm_agent {

    struct Pose{
        
        double x;
        double y;
        double yaw;
        double signed_distance{0.0}; // Incoming segment: forward +, reverse -, metres.
        double curvature{0.0}; // Incoming arc curvature, 1/metre; zero for straight segments.
    };

    struct OccupancyGrid
    {
        double resolution{1.0};
        double origin_x{0.0};
        double origin_y{0.0};
        int width{0};
        int height{0};
        std::vector<unsigned char> cells;

        bool isOccupied(double x, double y) const;
        bool isSegmentFree(double x0, double y0, double x1, double y1) const;
    };

    enum class HeuristicMode { Euclidean, Obstacle, Dual };

    enum class StopReason { NotStarted, GoalReached, IterationLimit, OpenExhausted,
        InvalidStart, InvalidGoal, PathCollision };
    const char * stopReasonName(StopReason reason);

    struct PlanningStats {
        StopReason stop_reason{StopReason::NotStarted};
        std::size_t iterations{0};
        std::size_t open_nodes_remaining{0};
        bool start_body_free{false};
        bool goal_body_free{false};
        Pose closest_pose{};
        double closest_position_error{std::numeric_limits<double>::infinity()};
        double closest_yaw_error{std::numeric_limits<double>::infinity()};
        double best_yaw_error_near_goal{std::numeric_limits<double>::infinity()};
        Pose best_yaw_pose_near_goal{};
        bool reached_goal{false};
        double path_length{0.0};
        std::size_t expanded_nodes{0};
        std::size_t obstacle_table_builds{0};
        std::size_t analytic_attempts{0};
        std::size_t analytic_successes{0};
        std::size_t reverse_segments{0};
        std::size_t path_points{0};
        double planning_time_ms{0.0};
    };

    struct PlannerConfig
    {
    double step_size{1.0};
    double wheel_base{2.7};
    double max_steer_angle{0.5};
    int steering_samples{3};
    double goal_tolerance{0.5};// 车辆离终点小于 0.5 m，就认为已经到达
    double goal_yaw_tolerance{0.17453292519943295}; // 10 degrees
    bool allow_reverse{false};
    bool analytic_expansion{false}; // Requires allow_reverse; leaves historical mode reproducible.
    double analytic_max_distance{15.0};
    int analytic_expansion_interval{10};
    HeuristicMode heuristic_mode{HeuristicMode::Dual};
    int max_iterations{1}; // 以后不会意外把“搜索未完成”当成“规划成功”
    double grid_resolution{0.5};
    double yaw_resolution{0.08726646259971647}; // 5 度，单位 rad
    OccupancyGrid map{};
    // Pose 的位置代表后轴中心。
    double vehicle_length{4.5};
    double vehicle_width{1.8};
    double rear_overhang{1.0};
    double collision_margin{0.1};

    // 车身任意点在相邻检查姿态之间的位移上界，单位 m。
    double collision_check_step{0.1};
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
        bool isPoseCollisionFree(const Pose & pose) const;

        bool isMotionCollisionFree(
            const Pose & from,
            const Pose & to) const;
        // Checks the true straight/arc sweep, with signed travel in metres.
        bool isArcCollisionFree(const Pose & from, double signed_distance, double curvature) const;
        explicit HybridAStarPlanner(PlannerConfig config = {});// 构造函数

        std::vector<Pose> plan(const Pose& start, const Pose& goal, PlanningStats * stats = nullptr) const;// 规划函数，输入起点和终点，返回路径点的向量
        Pose propagate(const Pose & pose, double steering_angle, int direction = 1) const;// 给当前车辆姿态 pose 和前轮转向角 steering_angle，
        //根据 PlannerConfig 中的 step_size、wheel_base、max_steer_angle，向前模拟一步，得到下一个姿态
        std::vector<Pose> generateSuccessors(const Pose & pose) const;
    private:
        bool tryAnalyticConnection(const Pose & from, const Pose & goal,
            std::vector<Pose> & connection) const;
        bool isFootprintFree(
            const Pose & pose,
            double extra_margin) const;
        double heuristic(const Pose & from, const Pose & goal,
            const std::vector<double> & obstacle_distances) const;
        std::vector<double> buildObstacleDistances(const Pose & goal) const;
        double turnHeuristic(const Pose & from, const Pose & goal) const;
        bool isGoalReached(const Pose & pose, const Pose & goal) const;
        SearchNode makeChildNode(
            const SearchNode & parent, // 当前被拓展的旧节点
            const Pose & child_pose, // 车辆向前走一步后得到的新姿态
            const Pose & goal, // 终点 用来计算新节点的h_cost
            const std::vector<double> & obstacle_distances,
            std::size_t parent_index) const; // 父节点在全部节点数组中的编号
        PlannerConfig config_;
    };
}

#endif // CAR_SWARM_AGENT_HYBRID_ASTAR_PLANNER_HPP
// 这一份 .hpp 是“声明书”：只告诉其他代码，这个规划器有什么数据、能做什么；具体怎样生成路径，下一步写进 .cpp
