# 单通道双车相向通行：最小 MPC 案例

本目录是可独立复制、运行与提交的 ROS2 案例包，聚焦“进入旁侧暂停区”与“倒车到入口退让区”两种固定让行策略，并保留“只在主通道停车”的负对照。

[本次模拟日志总报告](results/run_001/report.md) · [参数](config/scenarios.json) · [原始数据与版本](results/run_001/provenance.json)

| 策略报告 | 可播放视频 |
|---|---|
| [旁侧暂停区让行](results/run_001/head_on_pause_bay/report.md) | [H.264 MP4](results/run_001/head_on_pause_bay/replay.mp4) |
| [倒车退让](results/run_001/head_on_reverse/report.md) | [H.264 MP4](results/run_001/head_on_reverse/replay.mp4) |
| [主通道内停车对照](results/run_001/head_on_inline_stop/report.md) | [H.264 MP4](results/run_001/head_on_inline_stop/replay.mp4) |

MP4可以下载后用系统播放器/VLC播放。GitHub是否内嵌显示由其页面支持决定；视频文件本身无需ROS。

每一次模拟包含七项：提交版本、地图与初值、控制周期、通信设置、原始日志与图片、RViz2回放、可播放视频。代码只保留本案例需要的仿真、导出、回放与验证，独立于父工程的规划器和旧演示。


## 自动检测、释放与原任务返回（独立增量实验）

[实现协议与证据边界](recovery_protocol.md) · [E0–E4八组对照报告](results/recovery_001/report.md) · [独立日志验收](results/recovery_001/validation.json)

入口 `scripts/run_recovery.py` 复用已有QP、完整退出候选和实际通过/释放流程，增加持续互阻检测、锁定选角、返回双方原任务，以及可选联合切换见证。旧入口和历史报告保留。完整运行、验证、报告及测试命令见新报告末尾；当前结果是离线完整预演，未声称在线实时或递归可行证明。

## 当前研究：无占用权调度的双车死锁

新增独立入口 `scripts/run_deadlock.py`，不使用原来固定 A 优先的让行状态机。配置见 [deadlock.json](config/deadlock.json)，已生成结果见 [复现报告](results/deadlock_001/report.md) 和 [轨迹图](results/deadlock_001/head_on_deadlock/overview.png)。旧三种策略仍由 `run_study.py` 运行。

本阶段从两车已在单通道内相遇并停车的状态开始：A 位于 `x=-2.6`，目标 `x=18`；B 位于 `x=2.6`，目标 `x=-18`。主通道宽 2.8 m、车宽 2 m，没有沿途避让区。两车中心间距 5.2 m，车身间隙 0.5 m。尚未模拟接近和刹车过程，也未建立入口外等待区的几何；入口占用权调度属于下一阶段。

两车 MPC 均设置 `allow_reverse=True`，在保持前方最终目标时也允许正负速度，不设置等待位置或 A/B 优先级。每车针对对方上一周期的预测求解。对方预测保持停车时，前进受间距约束，后退使自己的目标误差增大；因此双方的局部最优都是停车。“等待对方倒车”是该策略的行为解释，当前没有显式协商报文。

另做两次互不执行的反事实求解：让 A 或 B 单独以身后位置为目标、对方保持停车，验证倒车轨迹可行。检测器只在双方未到目标、持续至少 5 秒位置变化不超过 0.01 m、速度不超过 0.01 m/s，且双方倒车探测均可行时标记 `deadlock`。默认配置在 5.2 秒检测到，随后记录到 12 秒；所有 60 次局部求解均被接受。该判据用于本场景复现，不构成任意场景的死锁证明。

重跑（输出目录下的 `head_on_deadlock` 子目录必须不存在）：

```bash
cd ~/car_swarm_switch_ws/src/car_swarm_single_lane/examples/head_on_mpc
OPENBLAS_NUM_THREADS=1 python3 scripts/run_deadlock.py --output results/rerun_deadlock_001
# 可加 --video 导出 H.264 视频；默认只生成日志、图片与报告。
```

查看仓库自带结果：先按下文方法独立构建并 source 安装环境，然后运行：

```bash
ros2 launch head_on_mpc replay.launch.py \
  case:=head_on_deadlock \
  study_dir:="$(ros2 pkg prefix --share head_on_mpc)/results/deadlock_001" \
  loop:=false
```

回放中的 `mutual_wait` 表示双方等待，`deadlock_detected` 表示已满足检测条件。标题最后显示 `deadlock`。车辆从开始就静止是本实验的明确初始条件；优化器每周期仍在求解，没有人为将控制量覆盖为零。

无 GUI 验证回放消息（`--run` 指向新结果可验证重跑）：

```bash
ROS_DOMAIN_ID=83 python3 tests/check_ros_replay.py \
  --run results/deadlock_001 --case head_on_deadlock
```

所有离线测试：`OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_*.py'`。新增测试覆盖实际双向速度可行性、双方互等、对方后退后能够前进，以及到达目标后的正常停车不会被判为死锁。

## 代码与工件维护

完成与中止分支共用 `pass_release_core.write_branch_logs()` 写出原始工件；未进入保持阶段时，`hold_prechecks.json` 为空数组。独立验证器按实际通行权授予事件检查后续阶段，EXIT 或 PARKED_READY 失败仍检查已执行轨迹的动力学、道路与间距，不要求不存在的通过状态。

报告中的几何、车长、释放裕度、保持时长和控制时序取自本轮配置及保存的 `geometry`，不另维护固定默认值。历史结果目录保持原样；源码变化后需要使用新目录重跑，不能为通过哈希检查而改写旧 `provenance.json`。

## 最新：CLEARED_HOLD 保持约束补齐

[旧新完整重跑对照](results/pass_release_hold_002/hold_comparison.md) · [本轮逐初值阶段报告](results/pass_release_hold_002/report.md) · [旧首控制量核对](results/pass_release_hold_002/original_acceleration_checks.json)

旧六次拒绝已核对为通行车首加速度使下一0.4 s段速度幅值从约0.023411升到0.026484 m/s。新可选模式在通行车CLEARED_HOLD优化内加入全预测低速硬约束 `|v|≤0.025 m/s` 与通过目标±0.08 m位置区间；保留道路、间距、目标不越界、原速度/加速度和预测末端停车条件。进入条件、外部严格阈值、0.8 s确认时间均未改变。

切换前两次QP诊断检查配对完整保持预测，失败则不进入可保持状态。另用“首周期制动到零、随后零加速度”的序列检查动力学、目标不越界、位置区间、道路、间距和完整路径，仅保存诊断，不作替代执行。不开启 `--hold-constraints` 时保留旧默认行为。

```bash
cd examples/head_on_mpc
OPENBLAS_NUM_THREADS=1 python3 scripts/run_pass_release.py --hold-constraints --output results/rerun_hold
OPENBLAS_NUM_THREADS=1 python3 scripts/verify_pass_release.py --run results/rerun_hold
OPENBLAS_NUM_THREADS=1 python3 scripts/compare_hold_update.py \
  --old results/pass_release_001 --new results/rerun_hold
```

新八组完整重跑中固定A与筛选均EXIT/PASS/释放8/8；原六次保持拒绝消失，未观察到新增返回验收或保持预检查失败。16条候选与16个分支在进入保持前的实际日志均与旧版一致，进入时刻不变。16次配对预检查与16条制动候选完整检查通过；额外32次QP诊断，零重试。实际保持样本最大速度约0.0249999964 m/s，位置误差最大约0.01794 m。

这属于保持约束补齐后的数值和采样验证；仍是离线模拟预演，不构成递归可行性、连续时间安全或最终恢复理论证明，不代表退让车回归或双方任务完成。

## 阶段历史：完整退出、实际通过与通行权释放

[完整闭环实验报告](results/pass_release_001/report.md) · [协议与几何定义](pass_release_protocol.md) · [逐策略指标](results/pass_release_001/summary.csv)

新增独立入口 `scripts/run_pass_release.py`，在原八组安全静止初值上，先按完整EXIT候选冻结固定A/筛选角色，再继承退出的实际积分状态和控制器上下文，运行 `PARKED_READY → PASS → CLEARED_HOLD → RELEASED/FAILED`。不修改旧复现入口和退出实验默认行为；不实现退让车回归，不标记双方任务完成。

```bash
cd examples/head_on_mpc
OPENBLAS_NUM_THREADS=1 python3 scripts/run_pass_release.py --output results/rerun_pass_release
OPENBLAS_NUM_THREADS=1 python3 scripts/verify_pass_release.py --run results/rerun_pass_release
# 需要已有 requirements-qp.txt 中的可选OSQP依赖。
```

保护区域R为原单通道主路矩形，释放裕度显式为0.5 m。通过目标沿用原 `goals_x`，既要整车离开R也要在原道路内停驻；禁止移动边界。通过车实际到目标附近、速度≤0.025 m/s并持续确认0.8 s后才释放。阶段切换不重置 x/v/y、全局时间、上一控制或warm controls。退让车保持参考取实际退出终态，始终继续求解和积分。两基线使用相同模型、约束、容差和预算，PASS失败不改选角色。

本次EXIT两基线均8/8；固定A完整通过/释放6/8，筛选4/8。6条失败分支均已实际整车清空R，但后续持续低速确认的积分段检查失败，未提交该段状态；这是停驻/释放条件未完成，不是无法穿过通道的几何证明。较短退出并不保证本模型下后续停驻与释放成功。完整原始日志、失败原因、共同成功集合的配对比较与计算时间见报告。

仍是离线完整预演，没有在线等待补偿；不声称实时预算、连续时间安全、转向可达性、递归可行性或双方任务完成已获证明。

## 退出阶段历史：分项验收、等价 QP 与预演耗时

[数值核查总报告](results/exit_numerics_003/report.md) · [分项残差与计时数据](results/exit_numerics_003/analysis.json) · [SLSQP 完整重跑](results/exit_audited_003/report.md) · [OSQP 完整重跑](results/exit_qp_003/report.md)

当前 `run_exit_screening.py` 采用独立于收敛标志的验收：检查有限值、独立逐子步积分与凝聚状态的动力学偏差、速度、加速度、终端速度、纵向道路、邻车与目标不越界，再检查配对完整路径、实际执行与退出终态。局部约束残差验收容差为 1e−5；求解器 status 保留为诊断，未收敛但全部检查通过的控制可以继续预演。分项原始值见每条候选的 `solves.json`、`path_checks.json` 和 `summary.json`。原有 DMPC/四组对照入口仍使用原先的 success 验收规则。

代码确认每个固定邻车/掩码/目标的纵向子问题为严格凸 QP；`build_longitudinal_qp()` 为 SLSQP 和 OSQP 共用实际矩阵。OSQP 将终端双向不等式合并为等式，约束行与目标使用正数缩放，保持数学等价。全道路区域并集、横向占用与配对完整轨迹仍在外部验收，并不将整个预演称为凸 QP。参考 [OSQP 问题形式](https://osqp.org/docs/solver/index.html) 与 [Python 接口](https://osqp.org/docs/interfaces/python.html)。

```bash
cd examples/head_on_mpc
# 在自己的 Python 环境安装可选依赖：python3 -m pip install -r requirements-qp.txt
OPENBLAS_NUM_THREADS=1 python3 scripts/run_exit_screening.py --output results/rerun_audited --qp-crosscheck
OPENBLAS_NUM_THREADS=1 python3 scripts/run_exit_screening.py --output results/rerun_qp --solver osqp
OPENBLAS_NUM_THREADS=1 python3 scripts/analyze_exit_numerics.py \
  --runs results/rerun_audited results/rerun_qp --output results/rerun_numerics
```

最新完整 QP 重跑中，A退与B退均 8/8，固定A与筛选均 8/8，成功率收益消失。固定A平均退让 14.10 m、退出 17.1 s；筛选平均 11.10 m、退出 15.8 s。SLSQP 新验收重跑接受了 6 次未收敛但检查通过的解，随后仍拒绝 4 次确实违反终端速度/目标边界的返回解；对应等价 QP 均通过。下方旧 6/8 表属于历史版本，不应作为当前成功率结论。

**方法定位为离线完整闭环模拟预演。** 阻塞状态快照上先串行生成两条直到退出的候选，然后选角；不等同于在线 DMPC 的单次有限时域决策。最终QP运行每组两候选生成与选角等待约 0.549–1.244 s（含审计与日志写入，不含进程启动），均超过 0.4 s 控制周期；8组共1368次QP主求解，零重试。交叉验证另一次运行有1186次SLSQP与1186次QP诊断，其延迟另报。不能称为满足实时预算的在线方法。共享候选是离线公平比较安排，真实固定角色部署可省去另一条预演。尚未模拟后续恢复与最终通行，不报告恢复时间收益。

## 安全静止初值下的完整退出候选与筛选

新增 [完整退出入口](scripts/run_exit_screening.py)、[终端与硬路径约束清单](constraints_exit.md) 和 [本次报告](results/exit_screening_001/report.md)。此入口固定初速为零，在相同安全阻塞初值下独立生成“A退/B等”与“B退/A等”，每条候选滚动求解到入口外退让区并完成横向进入、停车。等待车同样实际求解并积分，要求始终留在原位。退让车先对静止对方求解，等待车再对该候选求解，两车均保留邻车硬约束；属于离线候选生成，不是原 DMPC 的上一周期预测通信实验。

```bash
cd examples/head_on_mpc  # 从仓库根目录出发
OPENBLAS_NUM_THREADS=1 python3 scripts/run_exit_screening.py --output results/rerun_exit_screening
```

检查初始安全、每轮整个 6.4 s 预测、每 0.1 s 执行采样及退出终态；不仅检查端点。任一求解拒绝、路径违规或退出超时均使整条候选不通过。固定角色基线始终选 A；筛选基线只在完整退出检查通过的候选里按退让距离选角色，两条失败则不分配角色。两基线共享同一对候选与生成预算，不中途切换，也不执行已拒绝候选。

本次 8 组静止初值、16 条候选中，12 条完整退出通过；固定 A 6/8，筛选 8/8。4 条失败均为求解器 success 标志拒绝，最后线性约束残差仍小于数值容差，**不是物理或数学不可行证明**。报告保留具体原因；这里的改善只表示避开当前数值求解未认证的退出分支。退出不包含后续交会通行与回归。全部轨迹、预测、分段检查和求解记录均在报告同目录下。

## 四组统一对照与参数扫描

新增 [实验入口](scripts/run_comparison.py)、[扫描配置](config/comparison.json) 和 [64 次运行报告](results/comparison_001/report.md)。四组为 `dmpc`（保持前方目标）、`fixed_A`（固定 A 退让）、`distance`（选择较短退让距离）和 `checked`（在相同选择上增加退让预测可行性、恢复几何与切换检查）。设置 `selection_metric="time"` 可将后两组切换为估计时间选择：当前速度制动时间加退让距离/限速；它是启发式估计。`switch_improvement` 的单位随选择指标为米或秒。

```bash
cd examples/head_on_mpc  # 从仓库根目录出发
OPENBLAS_NUM_THREADS=1 python3 scripts/run_comparison.py --output results/rerun_comparison
# 加 --smoke 仅执行第一个初值的四种策略；输出目录必须不存在。
```

扫描相遇位置、初始速度、通道长度和对称加减速能力，默认 16 个初值共 64 次运行。初始中心间距为车长 + 安全裕度 + 双车总制动距离，改变速度/制动时会相应改变间距。每个初值四组共享完全相同的道路、状态、目标、模型、代价权重和约束，控制周期 0.4 s、预测 16 步、每步 4 子采样、SLSQP 每次最多 120 次迭代。每周期所有版本均执行两次退让探测和两次控制求解；前三组忽略探测结果。共同增加预测末端零速度与纵向道路边界约束，旧死锁复现入口保持默认约束。

每端入口外增加统一退让区和通行净空，通道内部没有避让区。退让车到区内停车后以现有预设横向占用曲线进入，再让对方通行，净空满足后回到主路出发。检查版用停车对方的预测探测退让轨迹，并检查退让区车身容纳、横向净空和对方终点是否允许完成超越。该检查只覆盖本模型的几何恢复条件，不是完整转向或多阶段可达性证明。角色切换仅在退让阶段、双方低速、新旧退让探测均通过、恢复几何可行、驻留时间和收益阈值均满足时允许；进入退让区后锁定角色。

输出 `summary.csv/json` 记录最小采样间距、执行/探测求解拒绝、任务超时、周期计算超时、停滞时间、恢复时间、任务完成时间、恢复失败、角色切换和两车倒车距离。每组保留实际轨迹、求解记录、预测和事件。失败或未恢复/完成的时间为 null，避免当作零时间收益。检查范围为采样安全与纵向动力学，不包含转向可达性或连续时间安全保证。

本次结果：DMPC 0/16、固定 A 6/16、距离选择 8/16、检查版 8/16 完成。8 个非零初速组合均因共同的初始匀速预测导致首次执行求解拒绝，须与任务超时分别理解。检查版尚未显示进一步收益，全部角色切换数为零，不能据此宣称切换检查有效。完整原始日志在报告旁 `raw_logs.tar.gz`；更详细的限制与指标定义见报告。

## 运行范围

状态为车身中心x和有符号vx，控制为加速度。纵向双积分模型、0.4s控制周期、6.4s预测时域、0.1s采样。两车分别求解并交换上一轮预测；高层监督器固定A优先，策略由案例名指定。

**横向进出暂停区是预设平滑占用变化，尚未验证自行车转向可达性。** 因此结论限于纵向空间与通行时序。视频从执行日志绘制，不是RViz录屏。`blocked`负对照表示未完成通行，不能写成任务成功。

## 环境

已验证环境见`results/run_001/provenance.json`。离线仿真/图片/视频需要Python3、NumPy、SciPy、Matplotlib、OpenCV；RViz回放需要ROS2 Jazzy和RViz2。Ubuntu可使用系统包：

```bash
sudo apt install python3-numpy python3-scipy python3-matplotlib python3-opencv
```

H.264编码必须由OpenCV的FFmpeg后端支持。本机系统OpenCV已通过实际编码/逐帧解码验证；部分pip OpenCV wheel不提供H.264编码器。`requirements.txt`列出Python依赖范围，不保证每种wheel都含编码器。已有结果的MP4播放不依赖Python环境。

## 独立构建与 RViz2 回放

在本目录运行，构建位置在目录外，不混入提交工件：

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/head_on_log build --base-paths . \
  --packages-select head_on_mpc \
  --build-base /tmp/head_on_build --install-base /tmp/head_on_install
source /tmp/head_on_install/setup.bash
ros2 launch head_on_mpc replay.launch.py case:=head_on_pause_bay
```

修改`case:=head_on_reverse`观看倒车，`case:=head_on_inline_stop`观看负对照。可加`playback_rate:=2.0`、`loop:=false`；默认循环播放。安装包自带本次日志，运行不依赖原工作区路径。

RViz显示地图/暂停区、A蓝车/B橙车、车头白条、已执行轨迹（粗线）、当前预测（细线）和阶段/速度/加速度；相机随`case_view`移动，固定坐标系`map`。B朝西但vx为正表示倒车。回放不运行优化器，也不发布车辆控制。

## 重跑一次完整模拟日志总结

仍在本目录运行，输出目录必须不存在：

```bash
OPENBLAS_NUM_THREADS=1 python3 scripts/run_study.py --output results/rerun_001
```

入口先运行4项相关测试，再仿真三种策略并生成原始日志、PNG、H.264 MP4、视频验证、版本哈希和报告。可以用`--config`传入另一份同结构参数。仅修改参数时不需要改优化器代码。

随后验证新结果的ROS回放并更新报告：

```bash
ROS_DOMAIN_ID=81 python3 tests/check_ros_replay.py --run results/rerun_001
python3 scripts/run_study.py --output results/rerun_001 --refresh-report
python3 scripts/verify_bundle.py --run results/rerun_001
```

ROS集成检查使用`rviz:=false`，验证发布消息、相机TF和最终位置，并非GUI画面检查。查看新结果时使用`study_dir:=/absolute/path/to/results/rerun_001`。`.gitignore`默认忽略个人`rerun*`目录；本次正式工件`run_001`随案例提交。

## 代码与日志对应

| 文件 | 职责 |
|---|---|
| `config/scenarios.json` | 地图、初值、控制参数和三个策略配置 |
| `scripts/head_on_core.py` | 预测模型、本车优化、让行状态机、闭环日志 |
| `scripts/recording.py` | RViz和视频共用的日志读取/显示插值 |
| `scripts/rviz_replay.py` | 仅发布回放可视化消息 |
| `scripts/render.py` | PNG及H.264视频导出与解码验证 |
| `scripts/run_study.py` | 版本记录、测试、仿真、报告与校验清单 |
| `scripts/verify_bundle.py` | 独立校验动力学、地图、车身间隙、通信、预测与视频 |
| `tests/` | 离线行为测试和ROS发布订阅集成检查 |

每个策略目录保存`config.json`、`actual.csv`、`solves.json`、`predictions.jsonl`、`events.json`、`summary.json`、`overview.png`、`replay.mp4`、`video.json`、视频预览图及`report.md`。`manifest.json`校验源码和本次工件，自身不参与哈希。

本案例从父工程相向通行实现提取并精简，沿用Apache-2.0许可证。Git提交号记录的是运行时父仓库HEAD；新增代码尚未提交时，以源码逐文件哈希识别实际执行版本。用户最终提交后，该提交同时包含源码与本次结果，无需伪造提前未知的提交号。

## GitHub 提交范围

当前目录位于父仓库的`examples/head_on_mpc`。最终提交由用户完成；只加入本目录，避免把父工程其他尚未整理的改动一起提交：

```bash
git add examples/head_on_mpc
git diff --cached --stat
```

审阅后按自己的流程提交并推送。这里只准备第一个方向，第二个方向不混入本次提交范围。

## 入口互斥预防与相遇后恢复：同构自行车模型单组对照

本轮独立入口采用后轴五状态运动学自行车及加速度/转角速率输入，
通过受限转向和圆弧机动实际积分，不调用旧预设横向日程。
两策略共享配置、车身、控制器与原目标，只比较入口协调与遇阻恢复。
原窄暂停区上的所测机动失败被保留，新公共转弯区地图单独保存；
不能把结果归为旧地图复现或DMPC优越性证据。

[变量表](entry_bicycle_variables.md) · [实测报告](results/entry_bicycle_001/report.md) ·
[独立验收](results/entry_bicycle_001/validation.json) · [运动动画](results/entry_bicycle_001/motion.gif)

在本目录执行，输出目录必须不存在：

```bash
OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p test_entry_bicycle.py -v
OPENBLAS_NUM_THREADS=1 python3 scripts/run_entry_bicycle.py --output results/entry_bicycle_rerun
OPENBLAS_NUM_THREADS=1 python3 scripts/verify_entry_bicycle.py --run results/entry_bicycle_rerun
```

新增模块：`bicycle_core.py`负责车辆积分、共同运动原语和旋转矩形验收；
`run_entry_bicycle.py`负责单车门禁、两条主运行、完整退出候选、许可与释放；
`verify_entry_bicycle.py`重读轨迹独立积分和重算指标；
`report_entry_bicycle.py`生成报告、轨迹图、时间线、控制界曲线及动画。
只需已有NumPy、Matplotlib及其Pillow依赖；复用旧BlockingDetector及候选排序规则，
旧E3/E4横移执行层不能沿用，新版释放另用双方完整未来轨迹见证。
离线模拟在预演计算时冻结，未验证连续时间安全、实时调度或实车动力学。
