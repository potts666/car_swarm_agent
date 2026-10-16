# A/B跟驰中的单C车merging：最小MPC案例

本目录是可独立复制、运行和提交的ROS2案例包。主道上A在B前方行驶，邻道C请求插入两者之间。用相同初值比较配合开间距和不配合两种配置，成功后的顺序为A-C-B。

[本次模拟日志总报告](results/run_001/report.md) · [地图与参数](config/scenarios.json) · [执行源码与依赖版本](results/run_001/provenance.json)

| 模拟报告 | 可播放视频 |
|---|---|
| [配合开间距](results/run_001/merge_cooperative/report.md) | [H.264 MP4](results/run_001/merge_cooperative/replay.mp4) |
| [不配合对照](results/run_001/merge_no_cooperation/report.md) | [H.264 MP4](results/run_001/merge_no_cooperation/replay.mp4) |

MP4可下载后用系统播放器/VLC播放，无需ROS；GitHub是否内嵌显示取决于其页面支持。每次模拟按七项标准保存：提交版本、地图与初值、控制周期、通信设置、原始日志与图片、RViz2案例回放、可播放视频。

## 模型与结论范围

三车初始(x,vx)：A=(14,1.5)，B=(0,1.5)，C=(7,1.5)。主道y=0、邻道y=3.5m，三车均朝东。采用纵向双积分和加速度控制，0.4s控制周期、16步/6.4s预测、0.1s执行/约束采样。只有单辆C；本最小案例不引入额外车队规模。

t=2s请求插入。协作配置激活虚拟前车链：C跟随A，B跟随C；速度/间距代价及约束促使B减速开间距。集中许可检查完整6秒动作时域，满足后执行预设横向占用变化，并继续观察速度恢复与A-C-B顺序。不配合配置维持巡航，不满足许可时C保持邻道。

**横向占用用五次平滑函数预设，未优化转角或验证自行车换道可达性。** 因此结果支持纵向空间/时序验证；不是自主顺序协商、一般可行性证明、连续安全证明或实车实验。负对照的`blocked`表示未完成并线。视频从原始执行日志绘制，不是RViz录屏。

## 环境

离线部分需要Python3、NumPy、SciPy、Matplotlib、OpenCV；实际测试版本在`provenance.json`。ROS回放使用ROS2 Jazzy与RViz2。Ubuntu可使用系统依赖：

```bash
sudo apt install python3-numpy python3-scipy python3-matplotlib python3-opencv
```

H.264编码需要OpenCV的FFmpeg后端支持。本机系统OpenCV已通过实际编码/逐帧解码；部分pip wheel不带H.264编码器，`requirements.txt`只声明依赖范围。已有MP4的播放不依赖Python环境。

## 独立构建与 RViz2 回放

在本目录执行，构建产物放到目录外：

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/merging_log build --base-paths . \
  --packages-select merging_mpc \
  --build-base /tmp/merging_build --install-base /tmp/merging_install
source /tmp/merging_install/setup.bash
ros2 launch merging_mpc replay.launch.py case:=merge_cooperative
```

改成`case:=merge_no_cooperation`看负对照。可加`playback_rate:=2.0`、`loop:=false`；默认循环并在结尾停留两秒仿真时间。安装包自带本次日志，无需原工程、旧规划器或原工作区绝对路径。

蓝A、橙B、绿C；白条是车头；粗线是执行历史、细线是当前保存的预测。显示时间、阶段、vx与加速度，俯视相机跟随`merging_view`，固定坐标系`map`。话题为`/merging_replay/markers`，回放不执行优化器、不发布车辆控制。

## 重跑七项标准模拟日志

在本目录执行，输出目录须不存在：

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 \
  python3 scripts/run_study.py --output results/rerun_001
```

先运行六项离线测试，再仿真两个配置，生成CSV/JSON/JSONL、PNG、H.264视频、解码验证、版本和报告。`--config`可指定同结构参数文件；预测时域必须覆盖完整横向动作。

安装ROS包后，验证新结果的真实发布订阅并封存报告：

```bash
ROS_DOMAIN_ID=84 python3 tests/check_ros_replay.py --run results/rerun_001
python3 scripts/verify_bundle.py --run results/rerun_001 --write-summary
python3 scripts/run_study.py --output results/rerun_001 --refresh-report
python3 scripts/verify_bundle.py --run results/rerun_001
```

ROS检查用`rviz:=false`，验证三车位置、状态、预测/执行轨迹与相机TF，不代表已检查GUI渲染。打开新结果时传`study_dir:=/absolute/path/to/results/rerun_001`。`.gitignore`忽略个人`rerun*`目录，正式`run_001`随案例提交。源码改变后应生成新run，不能用`--refresh-report`改写旧run的版本身份。

## 文件对应

| 文件 | 用途 |
|---|---|
| `config/scenarios.json` | 地图、初值、控制与许可参数、两种配置 |
| `scripts/merging_core.py` | 预测模型、各车优化、插入许可、闭环与日志 |
| `scripts/recording.py` | RViz和视频共用的状态读取/显示插值 |
| `scripts/rviz_replay.py` | ROS可视化回放 |
| `scripts/render.py` | 结果图、跟随视角视频和解码验证 |
| `scripts/run_study.py` | 版本记录、测试、运行、报告和清单封存 |
| `scripts/verify_bundle.py` | 独立审计动力学、地图、间隙、前车、包龄、预测、全部许可和视频 |
| `tests/` | 六项离线测试和ROS发布订阅检查 |

每个配置目录保存`config.json`、`actual.csv`、`solves.json`、`predictions.jsonl`、`admission_checks.json`、`events.json`、`summary.json`、`overview.png`、`replay.mp4`、`video.json`、预览图和`report.md`。其中`solves.json`记录每轮指定前车，`admission_checks.json`记录两对间距的最差预测残差。

来源沿用父仓库相关实现及Apache-2.0许可证。本目录独立保留merging需要的代码；没有倒车/暂停区状态机、旧地图规划器、C++二进制或旧实验副本。执行版本由运行时父提交和源码逐文件SHA-256共同标识；新增案例最终Git提交由用户完成。

## GitHub 提交范围

在父仓库中只加入本目录，避免混入其他未整理改动：

```bash
git add examples/merging_mpc
git diff --cached --stat
```

审阅后由用户提交和推送。本次不修改已提交的`examples/head_on_mpc`内容。
