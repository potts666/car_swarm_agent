# E1 ROS 2 Flag 与约束接口

实际工程：`car_swarm_switch_ws/src/car_swarm_single_lane/examples/head_on_mpc`，为 Git 独立 worktree 中的 ROS 2 `ament_cmake` 包。历史 `results/*/source`、`final_source` 不作为开发入口。新增结果独立保存至 `results/flag_e1_001`。

从包目录运行：

```bash
source /opt/ros/jazzy/setup.bash
OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_flag_e1.py'
ROS_DOMAIN_ID=91 OPENBLAS_NUM_THREADS=1 python3 scripts/run_flag_e1.py --output results/flag_e1_rerun
```

输出目录必须不存在。使用系统 Python 的 rclpy、std_msgs、NumPy 和 SciPy，无新增 pip 依赖。

三个实际 ROS 2 节点为 `e1_channel`、`e1_vehicle_a`、`e1_vehicle_b`。单进程执行器通过 DDS 实际投递，不启用进程内消息直传；仿真在消息传递和检查期间暂停，不模拟网络延迟。控制状态和原语仿真由同一协调循环推进；不是三个独立进程的车辆部署。

| Topic | ROS 类型 | JSON 内容 | 发布 / 订阅 |
|---|---|---|---|
| `/e1/channel/request` | std_msgs/String | vehicle_id | 车辆 / 通道 |
| `/e1/channel/state` | std_msgs/String | occupied、owner_id、version、publish_cycle | 通道 / 两车 |
| `/e1/channel/ready` | std_msgs/String | vehicle_id | 停靠完成车辆 / 通道 |
| `/e1/channel/release` | std_msgs/String | vehicle_id、checks（passed、both_future_goals 等原验收结果） | 当前 owner / 通道 |

均使用 reliable QoS，状态额外使用 transient local。理想通信场景以控制周期内接收作为请求时间，同周期请求按 ID 排序。首次授权允许接近车辆通行，之后的等待车辆必须报告原机动已停靠就绪。当前 owner 及完整未来检查结果匹配才释放，释放后自动从保留的申请中授权，不重复申请。无消息到达时最多等待 10 秒，随后中止并保存已收日志。

`Channel` 只存在于通道端；车辆约束的输入来自 `on_state` 订阅结果。`run_policy(..., transport=None)` 保留原行为；E1 注入 `RosFlag`。原 `S1/messages.jsonl` 仍为旧逻辑诊断，实际 DDS 发布/接收证据在顶层 `ros_messages.jsonl`，按 sequence 和单调墙钟记录顺序。

`flag_constraints.construct(car, g, state, remaining_commands, phase, scene)` 返回固定剩余参考上的 `0 <= s <= s_max_m`、末端停车标记、道路区域、车身禁入保护区和车辆能力。s 是后轴二维路径累计弧长，原点每次构造置为当前状态。等待参考来自既有制动、接近及入位原语，整车扫掠遇保护区时上界截断到上一个采样；停驻时 s_max=0。获准时切换原返回及通行参考，上界扩展到原任务终点。已释放车虽 g=0，仍可沿保护区外的剩余原参考完成原目标。

`accepts` 在执行前验收整段预测的进度和旋转车身保护区排除；原道路、双方间距、执行子采样、原目标及完整未来释放验收继续生效。该上界仅对所给固定参考有效，E2 的优化轨迹须沿同一参考并在阶段切换时保持进度一致。道路并集及保护区排除没有转换成 E3 的凸走廊矩阵，E3 还须按机动阶段及车身支撑量构造走廊。

`constraints.jsonl` 保存实际收到的状态版本、receive_cycle、effective_cycle、g_i 和具体约束；`constraint_transition.json` 摘录 B 停驻前后变化。通信完成、约束接口完成、实际优化闭环未完成分别写入 summary.json；当前没有调用 QP，不能称为优化闭环结果。
