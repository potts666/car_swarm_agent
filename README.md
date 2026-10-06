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
- 检查完整矩形车身与障碍的相交，并在 RViz 显示车身和安全余量。

当前采用以后轴中心为参考点的矩形车身，通过 SAT 检查车身与障碍栅格的相交。相邻路径点之间按位置线性插值、朝向最短角度插值检查运动，并加入保守膨胀量覆盖采样间运动；搜索前检查起终点车身，输出前再次验证全路径。

这个安全检查对应上述插值模型，尚未验证真实车辆的圆弧运动和跟踪误差。当前仅支持前进，没有倒车、路径平滑或目标朝向约束。TF 跟随是路径点播放，尚未实现车辆控制器。

## 项目结构

```text
car_swarm_agent/
├── include/car_swarm_agent/
│   ├── hybrid_astar_planner.hpp
│   └── demo_map.hpp
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
colcon build --packages-select car_swarm_agent \
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
```

启动使用 YAML 配置的规划演示：

```bash
ros2 launch car_swarm_agent planner_demo.launch.py
```

当前默认使用宽通道 U 形障碍，从内部 `(0, 0)`、朝向 `0 rad` 出发，目标为封闭端外侧 `(18, 0)`。U 向左开口，车辆需要向前转弯，经左侧开口退出，再绕到右侧目标。

地图范围为 `[-30, 30) × [-30, 30)`，分辨率为 `0.5 m`。三面墙的位置：

- 下墙：`x=[-10, 14.5)`，`y=[-12, -11.5)`。
- 上墙：`x=[-10, 14.5)`，`y=[12, 12.5)`。
- 右墙：`x=[14, 14.5)`，`y=[-12, 12.5)`。

内部净宽为 **23.5 m**，为当前约 **4.9 m** 的最小转弯半径留出空间。路径点数量由搜索结果决定，不作为测试的固定要求。此场景用于检验死胡同绕行，暂不测试狭窄通道贴边通过。

加入车身碰撞检查后的当前配置已验证生成 **77 个路径点**，路径经左侧开口退出并绕到目标。规划器测试验证各运动段符合车身碰撞约束；运行检查已确认路径、车身 Marker 和 TF 正常发布。RViz 显示用于观察，不能替代碰撞测试。

当前仍使用欧氏距离启发式，搜索上限设为 `200000`。后续可保持地图、车辆参数和起终点相同，与考虑障碍物的二维启发式比较。

也可以直接启动节点并在启动时覆盖参数。例如关闭障碍，复现无障碍演示：

```bash
ros2 run car_swarm_agent agent_node --ros-args \
  -p start_x:=1.0 \
  -p start_y:=-2.0 \
  -p goal_x:=7.0 \
  -p goal_y:=2.0 \
  -p obstacle_enabled:=false
```

上述旧场景的无障碍配置已验证生成 **9 个路径点**。直接运行节点和 launch 的默认起终点均为 `(0, 0)` 与 `(18, 0)`。

保留单格障碍模式：设置 `obstacle_shape:=single`，并通过 `obstacle_col`、`obstacle_row` 指定障碍格。两者为从 0 开始的栅格索引，改变地图原点后其世界坐标也会变化。

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

已有 RViz 配置显示 `/demo_map`、`/planned_path`、`/vehicle_footprint` 和 TF，Fixed Frame 为 `map`，视野已扩大以观察 U 形绕行。蓝色半透明矩形为车身，黄色外框为包含安全余量的碰撞矩形。两者使用与规划器相同的车辆参数，跟随 `base_link` 移动。黄色框不包含运动检查时额外使用的保守膨胀量。

当前车辆按路径点跳转并循环播放，到终点后回到起点；这个循环跳转不是规划路径的一部分，也不是连续驾驶仿真。如果使用自己的 RViz 配置，添加 **Map**、**Path** 和 **MarkerArray** 显示项，分别选择 `/demo_map`、`/planned_path` 和 `/vehicle_footprint`。

| 话题 / 坐标关系 | 内容 |
| --- | --- |
| `/planned_path` | `nav_msgs/msg/Path`，每秒发布一次 |
| `/demo_map` | `nav_msgs/msg/OccupancyGrid`，每秒发布一次 |
| `/agent_status` | `std_msgs/msg/String`，每秒发布一次 |
| `/vehicle_footprint` | `visualization_msgs/msg/MarkerArray`，有效路径下每秒发布一次 |
| `map → odom → base_link → laser` | TF 坐标链，`base_link` 每 0.5 秒移到下一个路径点，并循环播放 |

地图由节点内的参数生成，当前未接入外部地图订阅或 RViz 交互目标。

## ROS 参数

以下数值对应 `config/planner_params.yaml`：

| 参数 | 配置值 | 含义 |
| --- | --- | --- |
| `start_x`, `start_y` | `0.0`, `0.0` | 起点位置，单位 m |
| `goal_x`, `goal_y` | `18.0`, `0.0` | 目标位置，单位 m |
| `step_size` | `1.0` | 每次运动扩展的步长，单位 m |
| `wheel_base` | `2.7` | 车辆轴距，单位 m |
| `max_steer_angle` | `0.5` | 最大前轮转向角，单位 rad |
| `steering_samples` | `3` | 转向采样数量 |
| `goal_tolerance` | `0.5` | 到达目标的位置容差，单位 m |
| `max_iterations` | `200000` | 搜索循环次数上限 |
| `grid_resolution` | `0.5` | 搜索位置离散分辨率，单位 m |
| `yaw_resolution` | `0.08726646259971647` | 搜索朝向离散分辨率，约 5° |
| `map_resolution` | `0.5` | 地图每格边长，单位 m |
| `map_origin_x`, `map_origin_y` | `-30.0`, `-30.0` | 地图原点，单位 m |
| `map_width`, `map_height` | `120`, `120` | 地图列数与行数 |
| `obstacle_enabled` | `true` | 是否放置演示障碍 |
| `obstacle_shape` | `u` | `u` 为 U 形障碍，`single` 为单格障碍 |
| `obstacle_col`, `obstacle_row` | `7`, `7` | 单格模式的列号与行号，从 0 开始 |
| `vehicle_length`, `vehicle_width` | `4.5`, `1.8` | 车身长度与宽度，单位 m |
| `rear_overhang` | `1.0` | 后轴中心到车尾的距离，单位 m |
| `collision_margin` | `0.1` | 车身四周的安全余量，单位 m |
| `collision_check_step` | `0.1` | 相邻检查姿态间车身点位移的上界，单位 m |

`grid_resolution` 用于搜索状态去重，`map_resolution` 用于障碍地图，两者用途不同。旧版本的 `num_segments` 已不再使用。

U 形墙的位置固定在世界坐标中；地图必须完整容纳三面墙。其他分辨率下，会将与墙相交的栅格标为占用，实际通道尺寸可能改变。

直接使用 C++ 库时，默认空地图表示无障碍；当前 `PlannerConfig::max_iterations` 默认值为 `1`，正常搜索应显式设置合适值。ROS 节点默认使用 `200000`。

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

当前包含 15 个 GoogleTest 测试，覆盖：

- 到达目标容差和保留路径起点。
- 直行运动、转向角限制及左转/直行/右转后继生成。
- 搜索节点总代价计算。
- 起点车身与障碍相交时返回空路径。
- 两端点均为空闲时，仍能检测连接线段穿过障碍格。
- 绕过障碍到达目标，并检查路径的每一条线段无碰撞。
- 从 U 形内部经开口退出，绕过封闭端到达外侧目标，并检查整条路径无碰撞。

另外覆盖障碍位于车身内部、车侧碰撞、车身越界、旋转期间碰撞、采样间碰撞及无障碍运动。U 形与单障碍绕行测试逐段验证完整车身运动无碰撞。

本轮构建和 15 个规划器测试均已通过。双启发式尚未实现；下一步将以当前车身检查和 U 形场景作为基线，比较复杂地图中的搜索效率。

## VS Code 头文件诊断

如果终端构建成功，但编辑器提示找不到 ROS 消息头文件，将 C/C++ 扩展的 `C_Cpp.default.compileCommands` 设置为工作区生成的 `build/car_swarm_agent/compile_commands.json` 的绝对路径，再运行 **C/C++: Reset IntelliSense Database**。个人 `.vscode` 设置不随仓库提交。

如果 `rclcpp::Node` 无法识别，先确认节点文件包含 `#include "rclcpp/rclcpp.hpp"`，并以实际构建结果判断是否存在编译错误。
