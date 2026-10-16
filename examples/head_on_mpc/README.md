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
