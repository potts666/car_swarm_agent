> 论文基线：*Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments*，主线为 Hybrid A* 搜索与路径后处理。S–T、轨迹通信、轨迹复核、跟踪仿真、多项式优化和 MPC/DMPC 属于项目扩展，不能归为该论文的下一节或第 IV 节复现。多项式优化与严格连续安全证明现暂停推进。

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

- Hybrid A* 搜索：生成前进及倒车的左转、直行、右转候选，支持 Reeds–Shepp 解析连接。
- 使用优先队列按 `f = g + h` 搜索，`g` 为累计路径长度，`h` 默认取 `max(h_obstacle, h_turn)`，可切换欧氏距离或只开二维绕障。
- 将 `(x, y, yaw)` 离散为状态键，通过 Closed 集合和最优 `g` 代价记录减少重复搜索。
- 同时满足目标位置和车头朝向容差后，沿父节点回溯生成路径；未找到目标时返回空路径。
- 支持栅格地图，拒绝占用的起终点、地图外位置和无效地图。
- 检查相邻路径点之间的整条线段是否与障碍格相交，接触障碍边界或角落也视为碰撞，避免间隔采样漏检。
- 发布路径和演示地图，并通过 TF 循环展示车辆沿路径点移动。
- 检查完整矩形车身与障碍的相交，并在 RViz 显示车身和安全余量。

当前采用以后轴中心为参考点的矩形车身，通过 SAT 检查车身与障碍栅格的相交。相邻路径点之间按位置线性插值、朝向最短角度插值检查运动，并加入保守膨胀量覆盖采样间运动；搜索前检查起终点车身，输出前再次验证全路径。

启用倒车时，后继采用精确自行车圆弧积分，并逐段检查真实圆弧的车身扫掠；Reeds–Shepp 连接也做同样检查。历史前进模式保留 Euler 更新，便于复现基线。输出路径另外通过位置/朝向插值碰撞复检。尚未考虑跟踪误差或实现路径平滑，TF 跟随仍是路径点播放，尚未实现车辆控制器。

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

当前默认使用宽通道 U 形障碍，从内部 `(0, 0)`、朝向 `0 rad` 出发，目标为封闭端外侧 `(18, 0, 0)`。U 向左开口，车辆通过前进和倒车，经左侧开口退出，再绕到右侧目标。

地图范围为 `[-30, 30) × [-30, 30)`，分辨率为 `0.5 m`。三面墙的位置：

- 下墙：`x=[-10, 14.5)`，`y=[-12, -11.5)`。
- 上墙：`x=[-10, 14.5)`，`y=[12, 12.5)`。
- 右墙：`x=[14, 14.5)`，`y=[-12, 12.5)`。

内部净宽为 **23.5 m**，为当前约 **4.9 m** 的最小转弯半径留出空间。路径点数量由搜索结果决定，不作为测试的固定要求。此场景用于检验死胡同绕行，暂不测试狭窄通道贴边通过。

当前倒车与 Reeds–Shepp 配置生成 **203 个路径点**，实际累计行驶距离约 **64.105 m**，路径经左侧开口退出并绕到目标。规划器测试验证各运动段符合车身碰撞约束；运行检查已确认路径、车身 Marker 和 TF 正常发布。RViz 显示用于观察，不能替代碰撞测试。

默认使用双启发式、倒车和 Reeds–Shepp 解析连接，搜索上限为 `200000`，演示已恢复原目标 `(18, 0, 0)`。前进模式失败的诊断、参数逐项实验以及倒车/解析连接对照见 [诊断记录](test/reeds_shepp_diagnosis.md)。前一轮使用 `(24, 0, 0)` 的前进启发式对比保留在历史测试记录中。

也可以直接启动节点并在启动时覆盖参数。例如关闭障碍，复现无障碍演示：

```bash
ros2 run car_swarm_agent agent_node --ros-args \
  -p start_x:=1.0 \
  -p start_y:=-2.0 \
  -p goal_x:=7.0 \
  -p goal_y:=2.0 \
  -p obstacle_enabled:=false
```

上述旧场景的无障碍配置可生成路径。直接运行节点和 launch 的默认起终点均为 `(0, 0)` 与 `(18, 0, 0)`。

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

参数表适用于原空间规划演示；两车让行回放使用下面的独立启动入口。

### RViz2 两车让行过程回放

```bash
cd ~/car_swarm_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select car_swarm_agent --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 launch car_swarm_agent yield_demo.launch.py
```

启动会自动打开 RViz2，并加载 `rviz/yield_demo.rviz` 的俯视配置。
本车固定空间路径由现有 Hybrid A* 生成，邻车沿交叉直线通过。
默认在 `trajectory_yield` 模块内做 S–T 速度搜索，把邻车作为动态
障碍；画面是轨迹回放，不驱动车辆。

让行演示现在默认使用空间演示的同一张 U 形静态地图：0.5 m 分辨率、
原点 `(-30,-30)`、120×120 栅格，同一个 `geometry_.map` 用于 Hybrid A*、
连续复核和 `/yield_demo/map` 发布。交叉场景本车从 `(0,-8,pi/2)` 到
`(0,8,pi/2)`，保证终点完整车身留在上墙以内；邻车到 `(10,0)` 后停车，
不再穿过 U 的背墙。规划器启用已有的真实圆弧模式，而非历史 Euler 前进模式。

AFTER 准入由 `assessAfterPlayback()` 统一判断。S–T 结果、连续轨迹、
连续复核缺失或复核非 `Safe` 都禁止进入 AFTER。静态路径扫墙会在
S–T 扩展前拒绝，不能靠改时间绕过墙。连续复核采用带运动上界的区间
包络，超出细分预算返回 `Unresolved`，不会作为通过。

`static_map_enabled:=false` 用于复现缺少地图的情况：允许观察仅动态
验证的计划回放，但状态明确显示 **“静态碰撞未验证”**。这不是全面
安全结论；缺少地图时闭环模型仿真不会启动。

默认循环播放两个阶段：

1. **BEFORE**：按原时间行驶，首次检出车身安全矩形接触时，两车变红，
   停留 2 个回放秒。这段只是展示原冲突，不是可执行轨迹。
2. **AFTER**：`car_2` 按加速度约束减速，停车后变黄等待，蓝色 `car_1` 先通过，
   随后 `car_2` 恢复行驶；结束后停留 2 个回放秒，再重播。

蓝色车和路径表示优先通过的 `car_1`；橙色车和路径表示让行的
`car_2`；黄色圆盘是停车点，黄色车身表示 `STOP`。白色箭头表示
车头朝向，车身外的矩形线包含检测使用的 0.1 m 安全余量。
上方文字显示阶段、轨迹时间、速度、加速度、等待区间和检测精度。路径形状在让行
前后相同，区别通过车辆的时间回放表现。

默认 S–T 例子：初始速度和最高速度为 1 m/s，加减速度上限为
1 m/s²。原轨迹在 `t=5.40 s` 首次检出冲突；`car_2` 减速后在
`t=5.50 s` 停车，到 `t=11.50 s` 恢复；调整后的全程到
`t=27.50 s`，由 0.05 s 取样检测复检通过。这些时间都是相对于同一
轨迹发布时间的回放时间，和墙上时钟不同。

可选启动参数：

```bash
# 只看让行后过程，按原速度播放一次；结束后画面保持
ros2 launch car_swarm_agent yield_demo.launch.py \
  show_before:=false playback_rate:=1.0 loop:=false

# 只运行回放节点，用已打开的 RViz 查看
ros2 launch car_swarm_agent yield_demo.launch.py rviz:=false

# 对比之前的纯等待基线（不含加速度约束）
ros2 launch car_swarm_agent yield_demo.launch.py planner_mode:=wait

# 修改 S–T 搜索中的速度和加速度约束
ros2 launch car_swarm_agent yield_demo.launch.py \
  planner_mode:=st initial_speed:=1.0 max_speed:=2.0 max_acceleration:=0.5
```

默认 `playback_rate=2.0`，即两倍速。另一个已 source 的终端可以控制：

```bash
# 暂停 / 继续
ros2 topic pub --once /yield_demo/pause std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /yield_demo/pause std_msgs/msg/Bool '{data: false}'

# 回到播放起点；暂停状态会保留
ros2 topic pub --once /yield_demo/reset std_msgs/msg/Empty '{}'
```

若手动配置 RViz，将 **Fixed Frame** 设为 `map`，添加 **MarkerArray**
选择 `/yield_demo/markers`，再添加两个 **Path**，分别选择
`/yield_demo/neighbor_path` 和 `/yield_demo/yielded_path`。
这些话题的 **Durability Policy** 设为 **Transient Local**，便于
较晚打开 RViz 时收到已发布的静态路径。`/yield_demo/original_path`
提供原路径对照，其空间形状与让行后路径重合。

`/yield_demo/status` 可用 `ros2 topic echo` 查看。TF 子坐标系为
`yield_demo/car_1/base_link` 和 `yield_demo/car_2/base_link`。
默认模式仅回放计划，不发布车辆控制命令或 `predicted_trajectory`。
没有通过准入的结果时节点报错退出，不回退成 AFTER 原冲突轨迹。
旧 `planner_mode:=wait` 仍只有动态采样检测，状态单独标注其精度。

### 车辆模型与闭环跟踪仿真

```bash
# 独立数值车辆模型；先复核计划，再运行反馈控制并展示仿真结果。
ros2 launch car_swarm_agent yield_demo.launch.py \
  execution_mode:=tracking show_before:=false loop:=false playback_rate:=1.0

# 无图形环境：同样能查看复核结果和误差统计。
ros2 launch car_swarm_agent yield_demo.launch.py \
  rviz:=false execution_mode:=tracking show_before:=false loop:=false

# 对照：仅回放计划，并明确显示静态碰撞未验证。
ros2 launch car_swarm_agent yield_demo.launch.py \
  rviz:=false static_map_enabled:=false
```

`execution_mode:=playback` 是默认值；回放本身不是控制。
`tracking` 使用独立的运动学自行车状态：控制器读取计划和模型状态，
输出加速度和前轮转角，再由模型积分得到下一状态，不会把实际位姿
直接赋值成计划位姿。纵向采用加速度前馈、速度和纵向位置反馈；
转向采用曲率前馈、横向及朝向反馈，倒车时调整朝向反馈符号。
限制速度、加速度、转角及转角变化率，实际前进/倒车切换也必须先到零。

模型默认步长 0.01 s、初始横向偏差 0.25 m、朝向偏差 0.04 rad、
初速为计划初速的 0.9 倍。这些偏差用于验证反馈在工作；可用
`model_time_step`、`model_lateral_offset`、`model_heading_offset`、
`model_speed_scale` 改变。步长独立于 RViz 的 0.05 s 刷新周期与播放倍速。
模型每步以限速后的平均转角计算曲率，恒加速度/恒曲率解析积分；
它是离散运动学模型，不包含轮胎滑移或真实执行器延迟。

橙色为计划，绿色为模型积分得到的轨迹；模型轨迹复核非 `Safe`、
未覆盖完整时间区间或末端未停车时，禁止进入模型 AFTER 回放，
不能继承计划的安全结论。
仿真先在节点初始化时完成闭环积分，然后显示缓存的仿真结果；
这不是实时硬件控制，也没有发布真实控制命令。

| 话题 | 内容 |
| --- | --- |
| `/yield_demo/map` | 与规划和复核一致的静态占用栅格 |
| `/yield_demo/actual_path` | 独立车辆模型的完整运动路径 |
| `/yield_demo/model_odometry` | 仿真位姿、带符号速度和角速度 |
| `/yield_demo/tracking_metrics` | JSON：同一计划时间的总位置、纵向、横向、朝向、速度误差；全程 RMS 和最大位置误差；模型复核结果 |

朝向误差使用最短角差；位置误差在计划坐标系分解；RMS 是固定
仿真步长样本的均方根，包含初始偏差和末端停车阶段。可查看：

```bash
ros2 topic echo /yield_demo/tracking_metrics
```

验收测试包括：同一 U 地图中车身途中扫墙禁止 AFTER、完整绕障
轨迹允许 AFTER、缺图状态明确标注；另有实际积分、命令限幅、
实际换向零速、直线/圆弧误差收敛、缺图/冲墙禁止启动仿真，以及
扰动后的实际运动必须独立复核。

```bash
ctest --test-dir build/car_swarm_agent \
  -R '^test_(continuous_trajectory|trajectory_tracking)$' --output-on-failure
```

### 项目扩展：双车分散式 MPC 滚动闭环

```bash
source /opt/ros/jazzy/setup.bash
python3 src/car_swarm_agent/test/decentralized_mpc.py
python3 src/car_swarm_agent/test/test_decentralized_mpc.py
```

该入口在同一 U 形地图与双车交叉场景中执行两车闭环数值仿真。
两车分别优化未来 4.8 s 的加速度和前轮转角，控制周期 0.4 s，
自行车模型积分与约束检查间隔 0.1 s。各车从自己的实际积分状态重新
求解，只执行第一步控制，再发布含绝对时间的预测。时间采用共同仿真
纪元 100 s，不是当前墙钟。car_1 的参考是原场景横向路线，car_2 的
参考来自原生 Hybrid A* + S–T（包含等待）；参考不会覆盖执行状态。
静态占用栅格与原 C++ 场景一致，使用带每车 0.1 m 余量的矩形车身。

每轮冻结上一轮的两份预测，双方各自独立求解后同时更新状态与预测。
邻车在对应绝对时刻的车身构成约束；预测尾部增加 0.4 s 的恒速恒转角
模型预测，以覆盖下一轮视界，接收端拒绝时间覆盖不足的数据。
约束包括速度、加速度、转角、离散控制变化率、总加速度、静态车身
间隙与邻车车身间隙。SLSQP 使用居中差分；移位初值求解失败时，显式
记录并用制动初值重试一次（该初值不会直接执行）。最终求解成功且
约束残差不超过 1e-5 才执行；一方
失败即结束实验并报告，不静默改成参考回放或宣称到达。

这是**考虑邻车预测的分散式 MPC 基线**，不是协同 DMPC。当前预测
通过进程内数据包同步交换，未接入 ROS 通信节点或实车；没有同一控制
周期内的双方反复交换、优先权协商或联合收敛规则。实现采用 Python
中点自行车模型，与原 C++ 跟踪模型独立；计划和执行约束都是采样检查。
这里不开展严格连续安全证明。转角变化率指离散命令差/控制周期，模型
在周期内采用该步转角，不含转向执行器动态。

结果输出到 `log/decentralized_mpc/`：`summary.json` 记录两车是否到达、
最小采样车身距离、逐项约束违背、求解耗时及控制周期超时数；
`solves.json` 保留每车每轮的求解状态、收到预测的时间戳及第一步控制；
`predictions.jsonl` 保留每次发布的完整绝对时间预测，`actual.csv` 保留
实际执行状态。到达判据为距目标小于 0.3 m 且速度小于 0.05 m/s。
历史运行结果见 [双车 MPC 实验记录](test/decentralized_mpc_results.md)。

### 项目扩展：空间–时间联合优化数值实验（暂停推进）

新增离线入口直接复用同一 U 形双车交叉场景、Hybrid A*、`planSTSpeed()`、
`trackReference()` 和 `stepBicycle()`。它输出优化前计划、优化后计划及两份独立
闭环模型轨迹，不需要启动 RViz 或 ROS 节点。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select car_swarm_agent --cmake-args -DBUILD_TESTING=ON
python3 src/car_swarm_agent/test/optimize_spatial_temporal.py
# 查看可比较的指标和完整轨迹
cat log/spatial_temporal/comparison.json
```

运行依赖为 NumPy 和 SciPy（已在 `package.xml` 声明）。可通过 `--binary`、
`--output`、`--time-weight` 和 `--iterations` 指定可执行文件、输出目录、
时间权重及每轮迭代预算。`--gear-smoke` 在同一地图和邻车预测下，用三段
原生 Hybrid A* 路径做前进/倒车/前进的停车边界回归；`--reverse` 是曲率
更紧的掉头压力场景。两者不是演示准入条件。

后端见 [optimize_spatial_temporal.py](test/optimize_spatial_temporal.py)，
轨迹查询见 `quintic_trajectory.hpp/.cpp`。每个运动段用二维分片五次多项式
表示，段内各片等时长；内部节点的位置、速度、加速度和各段的对数片时长
一起优化。Hermite 参数化直接保证段内 C2 连续。保留初值的停车/等待及
换向位姿为段边界，固定边界位置和速度，禁止非零速度换向；停车处切向
加速度取 0.35 m/s²，并约束切向 jerk 与有限曲率极限，避免直接在零速
套用曲率除法。停靠边界允许有限加速度跳变，精确 jerk 积分仅计段内部分。

目标为精确的段内平方 jerk 积分加 `time_weight × duration`。惩罚目标使用
0.2 s 固定物理时间步长，并额外检查片边界。约束包括速度、纵向/总加速度、
带前进倒车符号的前轮转角、地图所有占用栅格与地图边界、同一绝对时刻的
邻车矩形车身距离。邻车预测必须覆盖整个优化执行区间，禁止以末点外推
补足信息。静态图来自 C++ 规划器使用的同一地图。

这是项目自建的离线研究扩展，不是 Dolgov 论文第 IV 节的复现。使用平方 hinge
惩罚及 SLSQP 数值梯度；优化使用矩形 SAT
有符号分离间隙作为保守距离约束。
固定点数的约束精修按 101/301/901 点逐级加密；0.01 s 独立网格复查与
C++ 检查（含停车前后极限）都通过才运行模型实验。失败保留诊断 JSON，
不输出可执行的优化系数，也不回退成原冲突轨迹。

`comparison.json` 包含时长、采样 jerk 能量、最小车间车身距离、实际跟踪
RMS/最大误差和模型独立连续复核结果；CSV 保留全轨迹。距离是带每车
0.1 m 余量的矩形之间的欧氏距离，按样本计算。计划的跟踪误差不填零，
只对实际模型相对其自身参考计算。模型时长包含相同的 2 s 末端停车阶段。
采样 jerk 能量按相邻二维加速度差/实际样本时间间隔积分，包含停车处的
加速度跳变；通常步长为 0.01 s，末区间不足 0.01 s 时使用其实际长度。
此指标依赖采样间隔，与另行记录的精确段内 jerk 积分有不同含义。

五次计划目前只有独立采样检查，**没有连续时间安全证书**；数值模型实际
轨迹仍由原连续圆弧验证器独立复核。实验结果不会接入 AFTER 或实车发布。
结果及已知限制见 [实验记录](test/spatial_temporal_results.md)。

```bash
ctest --test-dir build/car_swarm_agent \
  -R '^test_(spatial_temporal_math|quintic_trajectory|trajectory_tracking)$' \
  --output-on-failure
```

### 原空间规划参数

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
| `goal_yaw_tolerance` | `0.17453292519943295` | 目标朝向容差，10°，单位 rad |
| `start_yaw`, `goal_yaw` | `0.0`, `0.0` | 起终点朝向，单位 rad |
| `allow_reverse` | `true` | 前进/倒车圆弧后继及 Reeds–Shepp 转弯启发式 |
| `analytic_expansion` | `true` | 启用 Reeds–Shepp 终点连接，要求允许倒车 |
| `analytic_max_distance` | `15.0` | 当前节点与目标欧氏距离不超过此值时尝试连接，单位 m |
| `analytic_expansion_interval` | `10` | 每隔此展开次数尝试连接 |
| `heuristic_mode` | `dual` | `euclidean`、`obstacle`、`dual` |
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

### 固定路径 S–T 速度搜索

搜索接口 `planSTSpeed()`、`STOptions` 和 `STResult` 直接放在现有
`trajectory_yield.hpp/.cpp` 中，沿用 `trajectory_yield` 库及
`test_trajectory_yield` 测试目标；不另建项目或独立速度规划库。
RViz 的原 `yield_demo_node` 调用同一接口显示结果。

输入是 Hybrid A* 的 `std::vector<Pose>`、邻车预测、规划开始时刻的
Header 和当前实际速度。保留每段 `signed_distance` / `curvature`，
以其绝对距离累计 `s`，倒车同样增加；圆弧按原弧长解析求位姿。
`header.stamp + time_from_start` 仍是每个样本的实际时间。

每个时间步选择加速、匀速、减速或零速等待；换挡处和最终目标必须
以零速到达。在一步内提前刹停时，先按允许的减速度积分到零速，
剩余时间保持停车，不会直接把非零初速改为零。每个候选运动交给
现有车身检测器检查，成功拼接后再对全程复检。

```cpp
#include "car_swarm_agent/trajectory_yield.hpp"

car_swarm_agent::STOptions options;
options.initial_speed = measured_speed;  // 速度的绝对值，不能为可行而改成零
options.max_speed = 2.0;
options.max_acceleration = 1.0;
const auto result = car_swarm_agent::planSTSpeed(
  remaining_path, neighbor_trajectory, planning_header, "car_2",
  own_geometry, neighbor_geometry, options);
if (result.status == car_swarm_agent::STStatus::Success) {
  // result.trajectory 是全程复检通过的轨迹。
  // result.samples 与轨迹点一一对应，包含 s、时间、速度。
}
```

`remaining_path` 必须从当前规划起点开始，且保留 Hybrid A* 的原始
运动段信息。首个消息点的方向 `STOP` 表示无上一段，实际初速在
`samples.front().speed` 中，不能用方向字段推断速度为零。
调用方仍按 ID 选择让行车辆；此搜索器自身不做双车协商。

返回状态为 `Success`、`NoFeasiblePlan`、`InformationInsufficient`
或 `SearchLimit`，只有成功时携带轨迹。邻车过期、未覆盖规划起点、
或搜索遇到预测末端之外的未来时段，都不能作为安全；若没有成功
计划且探索遇到未知未来，返回信息不足。节点预算不足单独返回
`SearchLimit`。时间步数上限或状态剪枝可能漏掉可行方案，
`NoFeasiblePlan` 表示本轮设置下未找到计划，不是搜索完备性的证明。

默认搜索步长 0.5 s，输出/旧检测间隔 0.05 s。成功结果现在还包含
`continuous` 和 `continuous_verification`：真实圆弧与分段运动方程的
全区间复核同时检查速度、纵向及含向心项的总加速度，以及换向零速。
不限制 jerk。旧 ROS 消息仍是离散位姿；旧线性插值与圆弧连续模型
不同，不能直接继承连续模型的安全结论。跟踪仿真查询 `continuous`。

新增 9 项速度搜索测试放在原让行测试文件中，覆盖交叉减速再通过、
高速来不及停车、未知未来、过期/未覆盖起点、换挡零速、倒车累计
距离、真实圆弧、预算及无效输入，并验证现有 Hybrid A* 输出可直接
作为输入。运行现有测试目标即可同时检查两个时间规划基线：

```bash
./build/car_swarm_agent/test_trajectory_yield
# 只运行新增速度搜索测试
./build/car_swarm_agent/test_trajectory_yield --gtest_filter='STSpeedPlanner.*'
```

### 固定空间路径的让行基线

`trajectory_yield` 库的纯函数 `planFixedPathYield()` 在现有冲突检测之上
尝试插入等待，不调用或修改 Hybrid A*，不修改输入轨迹，也不接入 ROS
订阅和发布。它是速度规划前的简单时间调整基线，不是完整 S–T 优化。

输入为本车与邻车轨迹、显式传入的当前绝对时间 `now_ns`、两车几何和
`YieldOptions`。车辆 ID 必须非空且不同，按字符串字典序较大的车辆
让行，例如 `car_2` 让 `car_1`。若使用多位数字，请统一补零，例如
`car_02` 与 `car_10`，避免字典序与数值大小不一致。首点必须为
`time_from_start=0` 且方向为 `STOP`。

先检查原轨迹；全程未检出冲突时原样返回。存在冲突时，较大的 ID
按 `wait_step_ns` 递增尝试等待时长，直到 `max_wait_ns`（默认每次
增加 0.5 s，最多 10 s）。每个等待时长下，从首次检出冲突之前的
最近已有路径点向前搜索停车点，只允许选择绝对时间不早于 `now_ns`
的点。复制停车点为等待结束点，方向设为 `STOP`，后续原点的时间
统一延后；保持原来的位姿、方向及 `header.stamp`，首点仍为零，所有
时间严格递增。候选必须同时通过停车等待期间和调整后整条轨迹的检测，
才能返回第一条可行候选。停车点只限于原有点，不额外生成空间路径。

邻车预测必须覆盖本车整条原轨迹，以及候选的整个延长区间。
邻车预测结束后不假设其离场、也不外推末点停车。时间段不相交、
轨迹已过期或覆盖不完整均为信息不足，避免把等待到预测区间之外
误判为解除冲突。`now_ns` 和双方轨迹必须使用同一时钟；函数不读取
系统时钟。当前基线保守地检查整条轨迹，历史冲突不会通过修改过去
的时间点消除。

| 返回状态 | 含义 | `trajectory` |
| --- | --- | --- |
| `Unchanged` | 原轨迹全程按取样精度未检出冲突 | 原轨迹 |
| `Yielded` | 找到等待后的全程无检出冲突候选 | 调整后的轨迹 |
| `AwaitingPeerYield` | 本车 ID 较小，等待对方提供让行结果 | 空 |
| `InformationInsufficient` | 过期、无共同区间或预测覆盖不足 | 空 |
| `NoFeasiblePlan` | 本轮已测试候选无可行让行方案 | 空 |

只有前两种状态包含可供后续发布接入使用的结果。失败状态不返回原
冲突轨迹，调用方不得在 `trajectory` 为空时回退发布原轨迹。较小 ID
也不能在对方尚未确认让行时把原冲突轨迹当作已经验证的安全结果。
返回结果同时记录原首次冲突、选中的停车点、等待时长、尝试次数和
最终全程检测结果。无效输入抛出 `std::invalid_argument`。

```cpp
#include "car_swarm_agent/trajectory_yield.hpp"

car_swarm_agent::YieldOptions options;
options.wait_step_ns = 500000000;  // 0.5 s
options.max_wait_ns = 5000000000LL;  // 5 s
const auto result = car_swarm_agent::planFixedPathYield(
  own_trajectory, neighbor_trajectory, now_ns, own_geometry, neighbor_geometry, options);
if (result.trajectory) {
  // 此处才可把 result.trajectory 交给未来的发布接入层。
}
```

该基线没有制动/加速度约束或执行控制。与原检测器一样，仅能称为
“按取样精度未检出冲突”，不能证明取样间绝对无碰撞。现有
`trajectory_comm_node` 仍是通信演示，本次按约定未修改其发布逻辑，
尚未使用这里的让行结果。

让行基线测试不需要启动 ROS 节点或 RViz：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select car_swarm_agent --cmake-args -DBUILD_TESTING=ON
./build/car_swarm_agent/test_trajectory_yield
```

测试包含同刻交叉等待后解除、错时通过原样返回、停车位置不安全拒绝，
以及优先级、过期/无共同时间段、部分预测覆盖、延迟越过预测末端、
后续第二次冲突、回退更早停车点、倒车、时间溢出和无效输入。

### 两车轨迹的同刻车身碰撞检测

`trajectory_collision` 库提供纯函数 `detectTrajectoryCollision()`，声明位于
`include/car_swarm_agent/trajectory_collision.hpp`。输入为两条
`PredictedTrajectory`、两车各自的 `PlannerConfig` 几何参数以及取样选项；
不依赖订阅回调、不读取时钟、不修改轨迹，也不执行 S–T 速度规划。

每个点的绝对时间为 `header.stamp + time_from_start`，用整数纳秒计算。
检测范围是两条轨迹从首点到末点的共同闭区间；首点可有非零时间偏移。
在该区间按 `sample_step_ns` 取样（默认 0.1 s），并始终检查区间末端。
点间后轴中心 x/y 线性插值，车头 yaw 按最短角差插值；倒车不翻转车头，
相同位姿的不同时间点表示停车等待。该插值是当前离散消息的近似，不能
恢复稀疏点间的真实圆弧。区间外不外推，也不默认车辆在末点永久停留。

几何检测与空间规划器共用后轴偏移和旋转矩形构造，使用两车矩形的
四个分离轴判断重叠，边界接触也算冲突。每车矩形四周包含各自
`collision_margin`，因此默认结果包含安全余量；仅检查物理车身时将
两车的 `collision_margin` 均设为 `0.0`。

返回状态为 `Conflict`、`NoConflictAtSamples`、`NoCommonTime` 或 `Expired`。
首次冲突包含绝对时间 `absolute_time_ns` 及两车后轴中心位姿
`first_pose` / `second_pose`（位置不是车身接触点）。结果还包含检测
区间和实际取样次数。若需要过滤历史轨迹，由调用方设置
`options.not_before_ns` 为与消息时间基准相同的当前时间；共同区间
完全早于该时刻返回 `Expired`，否则仅检查剩余区间。默认检查全部
共同时间段，便于离线重放。

两条轨迹必须位于同一非空坐标系，调用方负责保证时钟基准一致；
函数不做 TF 或时钟同步。时间偏移必须非负且严格递增，位姿必须有限，
四元数必须有效且仅表示平面 yaw。无效输入抛出 `std::invalid_argument`，
空轨迹或时间段不相交返回 `NoCommonTime`，这些状态不能当成安全结论。

```cpp
#include "car_swarm_agent/trajectory_collision.hpp"

car_swarm_agent::PlannerConfig car_a, car_b;
car_a.collision_margin = car_b.collision_margin = 0.0;
car_swarm_agent::CollisionCheckOptions options;
options.sample_step_ns = 50000000;  // 0.05 s
const auto result = car_swarm_agent::detectTrajectoryCollision(
  trajectory_a, trajectory_b, car_a, car_b, options);
```

此层只称为“按取样精度检测”：`NoConflictAtSamples` 不保证取样之间绝对
无碰撞。测试包含粗取样漏检、细取样检出的例子，明确该限制。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select car_swarm_agent --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select car_swarm_agent \
  --ctest-args -R 'test_(trajectory_collision|hybrid_astar_planner)' --output-on-failure
colcon test-result --verbose
```

新增测试首先区分“路径交叉、相对时间相同但发布时间不同”与
“发布时间不同、相对时间不同但实际同刻经过”，随后覆盖停车等待、倒车、
过期轨迹、无共同时间段、纳秒时间对齐、端点取样、yaw 跨 ±π、车身偏移、
异尺寸几何、安全余量、旋转矩形以及无效输入。

规划器核心逻辑可脱离 ROS 节点独立测试：

```bash
cd ~/car_swarm_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select car_swarm_agent \
  --ctest-args -R test_hybrid_astar_planner --output-on-failure
colcon test-result --verbose
```

当前包含 27 个 GoogleTest 测试，覆盖：

- 到达目标容差和保留路径起点。
- 直行运动、转向角限制及左转/直行/右转后继生成。
- 搜索节点总代价计算。
- 起点车身与障碍相交时返回空路径。
- 两端点均为空闲时，仍能检测连接线段穿过障碍格。
- 绕过障碍到达目标，并检查路径的每一条线段无碰撞。
- 从 U 形内部经开口退出，绕过封闭端到达外侧目标，并检查整条路径无碰撞。

另外覆盖障碍位于车身内部、车侧碰撞、车身越界、旋转期间碰撞、采样间碰撞及无障碍运动。U 形与单障碍绕行测试逐段验证完整车身运动无碰撞。

双启发式实现与对比见 [测试记录](test/heuristic_benchmark.md)。二维启发式每次 `plan()` 从目标做一次八邻域反向 Dijkstra（禁止对角穿过障碍角），搜索节点直接查表，空地图退回欧氏距离。距离表属于单次调用，不会跨目标复用旧结果。

转弯启发式忽略障碍，使用最小转弯半径 `wheel_base / tan(max_steer_angle)` 的前进 Dubins 六类路径；目标两倍转弯半径以内使用与离散前进模型一致的距离、朝向修正、横向位移及目标在后方的转弯下界，避免精确连接强迫绕整圈。Dubins 公式可参照 [OMPL 官方实现](https://ompl.kavrakilab.org/DubinsStateSpace_8cpp_source.html)。此段描述历史前进模式（`allow_reverse=false`）；启用倒车时转弯启发式改用 Reeds–Shepp 距离，前进和倒车后继均采用精确圆弧。Reeds–Shepp 几何适配自 [OMPL 1.6.0](https://github.com/ompl/ompl/blob/1.6.0/src/ompl/base/spaces/src/ReedsSheppStateSpace.cpp)，BSD 许可保留于源码及 [第三方声明](THIRD_PARTY_NOTICES.md)，无需安装 OMPL。

八邻域距离表、连续 Dubins 与离散 Euler 模型以及终点容差存在差异，因此此实现不承诺启发式严格可采纳或全局最短路径。发现更小 `g` 时允许重新展开状态。`PlanningStats` 提供停止原因、循环次数、剩余 Open 数、起终点完整车身是否合法、最近的碰撞合法搜索姿态及其位置/朝向误差、位置容差内最小朝向误差、解析连接尝试/成功次数、倒车段数、路径点数和累计实际行驶长度。`Pose::signed_distance` 表示进入该点的段距离，前进为正、倒车为负；`path_length` 累加绝对值，圆弧按弧长而非弦长统计。统计耗时包括距离表构建、搜索及最终路径复检。最近搜索姿态统计不含解析连接的采样点，解析连接末端应单独检查。

## VS Code 头文件诊断

如果终端构建成功，但编辑器提示找不到 ROS 消息头文件，将 C/C++ 扩展的 `C_Cpp.default.compileCommands` 设置为工作区生成的 `build/car_swarm_agent/compile_commands.json` 的绝对路径，再运行 **C/C++: Reset IntelliSense Database**。个人 `.vscode` 设置不随仓库提交。

如果 `rclcpp::Node` 无法识别，先确认节点文件包含 `#include "rclcpp/rclcpp.hpp"`，并以实际构建结果判断是否存在编译错误。

诊断前进模式和原目标：

```bash
build/car_swarm_agent/diagnose_hybrid_astar
build/car_swarm_agent/diagnose_hybrid_astar --rs
```

前一条依次改变步长、角度分箱、位置分箱和转向采样，预算100万次；后一条在20万次预算下对比前进、仅倒车、倒车加解析连接。独立编译时需要同时编译 `src/reeds_shepp.cpp`。复现前进模式需设置 `allow_reverse:=false` 和 `analytic_expansion:=false`；仅倒车设置 `allow_reverse:=true`、`analytic_expansion:=false`。

解析扩展仅尝试当前位姿到目标的最短 Reeds–Shepp 几何曲线；若曲线碰撞则继续栅格搜索，不保证尝试所有更长连接。成功连接后按原父链加解析曲线输出，并保留换向点。`nav_msgs/Path` 只发布位姿，倒车方向保存在规划器的 `Pose::signed_distance` 中。原空间节点 TF 仍作位姿播放；独立车辆模型与跟踪控制仿真通过 `yield_demo` 的 `execution_mode:=tracking` 启用，尚未接真实车辆。
