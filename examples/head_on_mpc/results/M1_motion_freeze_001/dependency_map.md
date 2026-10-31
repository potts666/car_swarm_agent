# M2前的信息依赖审计

当前全部车辆由同一个Controller对象持有；三个ROS节点提供的是请求、许可与释放通信。没有车辆间预测交换，也没有独立局部优化器。下表描述真实代码读写，不是已实现的M2接口。

## 输入与持有者

| 信息 | 当前持有者/读取位置 | 当前用途 | M2归属与缺口 |
|---|---|---|---|
| 本车z=[x,y,psi,v,delta] | Controller.agents[c].z；linearized_joint.predict/subproblem/commit | 名义初态、动力学、约束和积分提交 | 车辆本地；需把测量时间和计划初态时间显式绑定 |
| cycle、仿真时刻 | run_joint_e2.run_b1中的now；规划日志state_timestamp_s | 两车同一冻结时刻 | 本地时钟/状态时间；当前Agent没有独立估计时间戳 |
| s、index、mode、previous、stop_steps、steering、ready | longitudinal_joint.Agent.advance/parameters；BicycleController.commit | 目标/约束/切段、制动与停车转向 | 本地行为状态，邻车所需部分须消息化；不允许外部模式覆盖状态 |
| 道路、车型、地图、初态和目标 | scene；routes(scene)、pose、freeze_corridors | 原固定直线/圆弧、车辆能力、道路矩形/保护区 | 可共享静态地图及能力；每车自己的目标/参考由本车持有，不能由通道返回全车轨迹 |
| 本车有效许可与版本 | RosFlag.received[c]；run_b1读取owner_id | 进入保护区/阶段约束，记录receive_cycle和state_version | 通道发布；车辆必须知道授权属于自己，M2需明确有效性/版本语义 |
| owner、pending、ready、released | ros_flag.Channel | 原请求排序、占用、就绪门、释放 | 通道资源协议；不是运动QP变量，车辆不能单方改owner |
| 两车当前状态/阶段 | Controller.agents全字典；longitudinal_joint.build_qp | 参考层阶段参数、双方分离约束 | 本车本地＋邻车消息或约束承诺；当前直接内存读取没有年龄检查 |
| 联合参考加速度/参数 | longitudinal_joint.solve → Controller.predict | 联合64变量求解；生成两车参考输入、稠密轨迹和几何跟踪目标 | 当前集中式专有，M2需重写每车局部问题与耦合输入；不能只切数组 |
| 两车参考/target/desired/linear center | BicycleController.predict内部字典 | 线性化、跟踪、信赖域、固定走廊 | 各车自有部分可本地化；邻车参考是否可读、由谁发布未定义 |
| 邻车状态、预测或承诺 | 当前不经车车消息；均来自联合内存与联合变量 | 双车同一QP中的位置分离 | M2必须设计消息schema、预测版本/年龄及兼容约束，当前没有承诺机制 |
| 固定走廊与分离轴 | freeze_corridors读取双方reference、permissions及B.ready/mode；subproblem装配pair行 | 车身包络与双方分离半空间 | 道路走廊可本地；阶段依赖和分离轴的共同选取需明确双方一致性 |
| 参考热启动缓存 | Controller.reference_previous按A/B存储；remember_reference | 上次提交参考、移位、段/模式失效 | 每车自身缓存可本地；当前是集中式字典，不能把它当车车共享预测 |
| 全车未来预演 | Controller.future_witness深拷贝全部agents/缓存，授予B未来许可 | 从当前状态滚动直到双方目标以允许释放 | 当前集中式对照专有；M2需单独声明协议/验收变更及替代验证，不能偷偷删掉 |

## 输出、提交与反馈

| 输出 | 产生及消费位置 | M2需要保留的含义 |
|---|---|---|
| 两车候选N步输入 | subproblem → BicycleController.predict → run_b1 | 每车只能优化自身输入；发布/执行的版本必须明确 |
| 线性状态及稠密非线性预测 | condensed/subproblem、bicycle_core.rollout | 邻车接收的应是哪一版预测、包含停车修复与否，须明确 |
| accepted、message、原QP状态/迭代/残差 | 两级solve及predict；run_b1拒绝出口 | 不把未收敛改成功，按参考/模型/修复后约束/阶段/物理/预算分类 |
| mapping_errors、trust_replay_passed、安全检查 | BicycleController.predict及外层audit | 模型误差不是道路跟踪误差，也不是求解器残差 |
| stop_input_adjustments | subproblem，候选检查前 | 原输入/修复输入与实际预测一致；M2修复若改变已发布预测，需重新协调 |
| 实际提交、更新进度及缓存 | BicycleController.commit、remember_reference | 只提交首周期；状态由积分产生；拒绝候选不污染缓存 |
| 阶段反馈与ready/完成事件 | 下一周期prepare、run_b1、RosFlag.ready | 优化结果可影响本地后续模式，但当前没有通用反馈框架或安全备用行为 |
| 释放/未来见证、完整周期日志 | run_b1/future_witness | 不能把停止仿真当作运动中的备用处置 |

## 四个集中式断点

1. 参考层同时优化A/B的64个加速度变量，耦合行属于同一个问题。
2. 自行车层同时优化128个输入，两车位置映射在同一pair约束中出现。
3. 走廊分离轴读取双方阶段和B.ready/mode；当前依赖一致共享内存。
4. 释放检查复制双方全部未来，具有全局状态、参考及许可信息。

将向量切成两个长度相同的片段、把节点拆成两个进程或改名DMPC，都不解决上述断点。M1只冻结集中式参考，不做这些架构改动。
