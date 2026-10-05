# car_swarm_agent

一个用于学习 ROS 2、C++、车辆运动学和路径规划的小型项目，使用 ROS 2 Jazzy 和 C++17。

规划器是独立的 C++ 库，ROS 节点负责读取参数、生成演示地图、调用规划器和发布结果：

```text
agent_node → HybridAStarPlanner → std::vector<Pose>
     ↓
/planned_path + /demo_map
     ↓
vehicle_tf_broadcaster → TF → RViz
```

## 当前功能

- 基础 Hybrid A* 搜索：用车辆运动学生成左转、直行、右转候选状态。
- 使用优先队列按 `f = g + h` 搜索，`g` 为累计路径长度，`h` 为到目标的欧氏距离。
- 将 `(x, y, yaw)` 离散为状态键，通过 Closed 集合和最优 `g` 代价记录减少重复搜索。
- 到达目标位置容差后，沿父节点回溯生成路径；未找到目标时返回空路径。
- 支持栅格地图，拒绝占用的起终点、地图外位置和无效地图。
- 检查相邻路径点之间的整条线段是否与障碍格相交，接触障碍边界或角落也视为碰撞，避免间隔采样漏检。
- 发布路径和演示地图，并通过 TF 循环展示车辆沿路径点移动。

当前是基础规划演示：碰撞检测针对车辆参考点的连接线段，尚未考虑车身轮廓；仅支持前进，没有倒车、路径平滑或目标朝向约束。TF 跟随是路径点播放，尚未实现车辆控制器。

## 项目结构

```text
car_swarm_agent/
├── include/car_swarm_agent/hybrid_astar_planner.hpp
├── src/
│   ├── agent_node.cpp
│   ├── hybrid_astar_planner.cpp
│   └── vehicle_tf_broadcaster.cpp
├── config/planner_params.yaml
├── launch/
│   ├── planner_demo.launch.py
│   └── car_swarm_demo.launch.py
├── rviz/car_swarm_demo.rviz
├── test/test_hybrid_astar_planner.cpp
├── CMakeLists.txt
└── package.xml
```

## 构建与运行

将 package 放入工作区的 `src` 目录后：

```bash
cd ~/car_swarm_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select car_swarm_agent
source install/setup.bash
```

启动使用 YAML 配置的规划演示：

```bash
ros2 launch car_swarm_agent planner_demo.launch.py
```

当前配置从 `(1, -2)` 规划到 `(7, 2)`，起始朝向为 `0 rad`，默认启用一个障碍格：第 7 行、第 7 列（索引从 0 开始），对应 `x=[3.5, 4.0)`、`y=[-1.5, -1.0)`。路径点数量由搜索结果决定。

也可以直接启动节点并在启动时覆盖参数。例如关闭障碍，复现无障碍演示：

```bash
ros2 run car_swarm_agent agent_node --ros-args \
  -p start_x:=1.0 \
  -p start_y:=-2.0 \
  -p goal_x:=7.0 \
  -p goal_y:=2.0 \
  -p obstacle_enabled:=false
```

上述无障碍配置已验证生成 **9 个路径点**。直接运行节点而不传参数时，默认起点为 `(0, 0)`，终点为 `(5, 3)`；launch 则加载 YAML 中的起终点。

参数在节点启动时读取，规划也在启动时执行一次。修改 YAML 后需要重新构建并重启演示。

## 路径跟随与 RViz

启动规划节点和 TF 广播节点：

```bash
ros2 launch car_swarm_agent car_swarm_demo.launch.py
```

另开终端启动 RViz：

```bash
cd ~/car_swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
rviz2 -d src/car_swarm_agent/rviz/car_swarm_demo.rviz
```

已有 RViz 配置显示 `/planned_path` 和 TF，Fixed Frame 为 `map`。显示障碍地图时，在 RViz 中添加 **Map** 显示项，Topic 设置为 `/demo_map`。

| 话题 / 坐标关系 | 内容 |
| --- | --- |
| `/planned_path` | `nav_msgs/msg/Path`，每秒发布一次 |
| `/demo_map` | `nav_msgs/msg/OccupancyGrid`，每秒发布一次 |
| `/agent_status` | `std_msgs/msg/String`，每秒发布一次 |
| `map → odom → base_link → laser` | TF 坐标链，`base_link` 每 0.5 秒移到下一个路径点，并循环播放 |

地图由节点内的参数生成，当前未接入外部地图订阅或 RViz 交互目标。

## ROS 参数

以下数值对应 `config/planner_params.yaml`：

| 参数 | 配置值 | 含义 |
| --- | --- | --- |
| `start_x`, `start_y` | `1.0`, `-2.0` | 起点位置，单位 m |
| `goal_x`, `goal_y` | `7.0`, `2.0` | 目标位置，单位 m |
| `step_size` | `1.0` | 每次运动扩展的步长，单位 m |
| `wheel_base` | `2.7` | 车辆轴距，单位 m |
| `max_steer_angle` | `0.5` | 最大前轮转向角，单位 rad |
| `steering_samples` | `3` | 转向采样数量 |
| `goal_tolerance` | `0.5` | 到达目标的位置容差，单位 m |
| `max_iterations` | `1000` | 搜索循环次数上限 |
| `grid_resolution` | `0.5` | 搜索位置离散分辨率，单位 m |
| `yaw_resolution` | `0.08726646259971647` | 搜索朝向离散分辨率，约 5° |
| `map_resolution` | `0.5` | 地图每格边长，单位 m |
| `map_origin_x`, `map_origin_y` | `0.0`, `-5.0` | 地图原点，单位 m |
| `map_width`, `map_height` | `40`, `30` | 地图列数与行数 |
| `obstacle_enabled` | `true` | 是否放置演示障碍 |
| `obstacle_col`, `obstacle_row` | `7`, `7` | 障碍格列号与行号，从 0 开始 |

`grid_resolution` 用于搜索状态去重，`map_resolution` 用于障碍地图，两者用途不同。旧版本的 `num_segments` 已不再使用。

直接使用 C++ 库时，默认空地图表示无障碍；当前 `PlannerConfig::max_iterations` 默认值为 `1`，正常搜索应显式设置为 `1000` 或其他合适值。ROS 节点默认使用 `1000`。

## 自动测试

规划器核心逻辑可脱离 ROS 节点独立测试：

```bash
cd ~/car_swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select car_swarm_agent \
  --ctest-args -R test_hybrid_astar_planner --output-on-failure
colcon test-result --verbose
```

当前包含 8 个 GoogleTest 测试，覆盖：

- 到达目标容差和保留路径起点。
- 直行运动、转向角限制及左转/直行/右转后继生成。
- 搜索节点总代价计算。
- 堵住第一步时返回空路径。
- 两端点均为空闲时，仍能检测连接线段穿过障碍格。
- 绕过障碍到达目标，并检查路径的每一条线段无碰撞。

目前堵住第一步的测试使用默认 `max_iterations=1`，其通过本身不能区分碰撞阻断与迭代上限导致的失败；线段相交和绕障碍测试提供额外验证。
