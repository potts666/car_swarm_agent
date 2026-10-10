# ROS 双节点离线闭环对比

2026-10-10，项目扩展。复用 `decentralized_mpc.py` 的同一 U 形交叉场景、
Hybrid A* + S–T 参考、障碍、自行车执行模型及 MPC 参数。控制周期 0.4 s，
视界 4.8 s；仿真时间按闭环轮次推进，不以墙钟超时丢弃已经算出的控制。
五种实验依次运行，设置 `OPENBLAS_NUM_THREADS=1`；耗时受机器负载影响。

每辆车由独立 Python 进程中的 rclpy 节点负责接收邻车预测、求解、发布控制和
下一轮预测。第三个 ROS 节点只提供测量状态并积分车辆模型，不代替车辆求解或
传输邻车预测。通过 `/mpc_benchmark_<PID>/car_<ID>/prediction` 直接交换原生
`PredictedTrajectory`，ROS reliable + transient-local QoS。控制和仿真测量通过
`std_msgs/String` JSON 传送，属于离线实验接口。安装后可用 `ros2 run
car_swarm_agent ros_decentralized_mpc.py` 启动实验及两个车辆进程。

消息扩展：`TrajectoryPoint` 增加速度和转角；`PredictedTrajectory` 增加轮次。
初始预测轮次 -1，第 0 轮输出虽然与初始预测同为 100 s，仍不会覆盖对方应使用
的初始预测。每轮只取序号小于本轮的最新预测，检查精确上一轮序号、车辆 ID、
map 坐标系、点数、有限状态、单调时刻、年龄及完整求解视界覆盖。消息共 53 点。
共享绝对仿真纪元是 100 s；`header.stamp + time_from_start` 是点的绝对仿真时刻，
不是 Unix 时间。消息年龄以本节点接到的测量仿真时刻减 header 时间戳计算；
接收日志可能仍处于上一轮，控制日志记录本轮使用时的年龄，正常为 0.4 s。

缺失消息最多等待墙钟 0.15 s；年龄超过 0.45 s、未来时间戳、轮次缺失、
覆盖不足或求解拒绝时，保留当前转角并以最大 1 m/s² 减速，速度降到零后
保持停车。禁止用缺失消息补成空障碍或外推过期预测继续正常 MPC。
`optimizer_accepted` 与控制输出的 `accepted` 分开记录，制动不算优化成功。
故障注入分别抑制 car_2 初始预测、抑制第 2 轮预测、将第 2 轮时间戳倒退 1 s。
后两项均在第 3 轮触发 car_1 的过期处理；每种实验恰好一次保守输出。

| 场景 | 接收至控制平均 / P95 / 最大 ms | 单车输出 / 整轮超 0.4 s 次数 | 保守输出 | 采样间最小车身距离 m | 距离不足 0.08 m 的加密点数 |
|---|---:|---:|---:|---:|---:|
| 进程内基线 | 82.7 / 261.2 / 441.7 | 1 / 1 | 0 | 0.161083 | 0 |
| ROS 正常 | 101.4 / 288.1 / 422.4 | 1 / 1 | 0 | 0.161083 | 0 |
| ROS 初始缺失 | 104.4 / 285.9 / 472.9 | 1 / 1 | 1 | 0.056731 | 9 |
| ROS 单轮丢失 | 139.5 / 356.7 / 546.6 | 4 / 4 | 1 | 0.056849 | 15 |
| ROS 时间戳过期 | 147.4 / 437.8 / 1084.5 | 7 / 6 | 1 | 0.056849 | 15 |

五种场景均在 20.8 / 25.6 s 达到 car_1 / car_2 的终点条件。
正常 ROS 的执行状态与进程内基线通过逐字段数值对比；场景参考和障碍导出逐字节
一致。原 0.1 s 采样矩形车身距离最小值正常为 0.161651 m。采样间距离用每轮
第一条实际控制重新以 0.01 s 中点积分计算，包含每车 0.1 m 外扩，不是对距离
标量插值；不同积分步长会产生少量数值差异，仍不是连续时间证明。

发现的限制：三种故障实验的原 0.1 s 采样约束都通过，但 0.01 s 距离检查低于
0.08 m 余量，最小约 0.057 m。此次没有检测到车身重叠；有限采样不能排除
更小间隔的碰撞。制动策略没有未知邻车运动的可达集或连续时间碰撞保证。
`success` 只表示到达且原采样约束通过；新增 `inter_sample_clearance_target_met`
在故障实验明确为 false，并保存不足余量的点数与最大短缺。正常 ROS 仍有整轮
超过控制周期，不能称为已经满足实时调度。

接收至控制耗时使用 `perf_counter`，从该条预测接收回调开始，到控制命令准备好、
即将发布为止；包含接收后等待与求解。初始消息完全缺失时此指标为 null，另存
`request_to_control_seconds`，避免把请求延迟冒充接收延迟。ROS 整轮超周期以仿真
节点发出两个请求到收齐两个控制的实际墙钟耗时计数。基线整轮以两车顺序求解
耗时之和计数；没有 ROS 请求/传输。`solves.json` 保存每轮单车耗时与整轮耗时，
`car_*_exchange.jsonl` 记录车辆 ID、绝对时间戳、轮次、消息年龄、点数及拒绝原因。

复现（在工作区根目录）：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select car_swarm_agent
source install/setup.bash
export OPENBLAS_NUM_THREADS=1
python3 src/car_swarm_agent/test/decentralized_mpc.py --output log/ros_comparison/in_process
python3 src/car_swarm_agent/test/ros_decentralized_mpc.py --output log/ros_comparison/ros_nominal
for fault in missing drop stale; do
  python3 src/car_swarm_agent/test/ros_decentralized_mpc.py --fault "$fault" --output "log/ros_comparison/ros_$fault"
done
python3 src/car_swarm_agent/test/check_ros_mpc_results.py
ctest --test-dir build/car_swarm_agent -R 'test_(decentralized_mpc|ros_decentralized_mpc|spatial_temporal_math)$' --output-on-failure
```

构建、3 个相关 CTest（ROS 协议包含 4 个单元测试）和实跑日志复核均通过。
完整数据在 `log/ros_comparison/<场景>/`：`summary.json`、`solves.json`、
`predictions.jsonl`、`actual.csv`、`body_distances.csv` 及车辆交换日志。
随代码保存五种实验摘要 `test/ros_decentralized_mpc_summary.json`。
