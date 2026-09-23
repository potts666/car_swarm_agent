# car_swarm_agent

一个用于学习 ROS 2、C++ 和路径规划工程结构的小型项目。

当前版本展示了如何把纯 C++ 路径规划类与 ROS 2 节点分开：

```text
ROS 2 Node
    ↓
HybridAStarPlanner
    ↓
std::vector<Pose>
```

## 当前功能

- 使用 ROS 2 Jazzy 和 C++17；
- `HybridAStarPlanner` 是独立的 C++ 库；
- `agent_node` 创建规划器并调用 `plan()`；
- 输入起点 `(0, 0)` 和终点 `(5, 3)`；
- 输出 11 个路径点；
- 当前规划算法是直线插值，用于验证软件结构。

> 注意：当前版本不是完整的 Hybrid A* 搜索算法；后续会逐步加入车辆运动学、碰撞检测与真实的搜索过程。

## 项目结构

```text
car_swarm_agent/
├── include/car_swarm_agent/
│   └── hybrid_astar_planner.hpp
├── src/
│   ├── agent_node.cpp
│   └── hybrid_astar_planner.cpp
├── CMakeLists.txt
└── package.xml
```

## 构建与运行

将这个 package 放进 ROS 2 workspace 的 `src` 目录后：

```bash
cd ~/car_swarm_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select car_swarm_agent
source install/setup.bash
ros2 run car_swarm_agent agent_node
```
## ROS 参数

规划起点、终点和路径段数可在运行时传入，无需修改代码：

```bash
ros2 run car_swarm_agent agent_node --ros-args \
  -p start_x:=1.0 \
  -p start_y:=-2.0 \
  -p goal_x:=7.0 \
  -p goal_y:=2.0 \
  -p num_segments:=4
```

上述示例从 `(1, -2)` 规划至 `(7, 2)`，生成 5 个路径点。

## Launch 启动

也可以使用 launch 文件一次性启动节点和参数：

```bash
ros2 launch car_swarm_agent planner_demo.launch.py
```

`launch/planner_demo.launch.py` 当前配置：

- 起点：`(1, -2)`；
- 终点：`(7, 2)`；
- 路径段数：`4`；
- 输出路径点数量：`5`。

## 自动测试

规划器的核心逻辑可脱离 ROS 节点独立测试：

```bash
cd ~/car_swarm_ws
colcon test --packages-select car_swarm_agent --ctest-args -R test_hybrid_astar_planner
colcon test-result --verbose
```

当前测试验证路径点数量、起点、中间点和终点是否符合预期。

## 技术标签

- ROS 2 Jazzy
- C++17
- CMake
- 路径规划