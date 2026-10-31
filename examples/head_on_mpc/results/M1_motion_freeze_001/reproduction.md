# 冻结副本的复现与证据复用

本M1未新增运动实验、未重新积分、未重跑回归；17个本地运行依赖、完整配置、原验收及已声明依赖/设备/线程与关闭版相符，复用两轮已通过结果。新做的只是材料审计与`--help`/资源布局导入检查，见packaging_check.json。

## 当前可直接使用的Linux环境

在当前git工作区根目录执行。需要ROS2 Jazzy、Python3.12.3、NumPy1.26.4、SciPy1.11.4，以及原OSQP1.1.3/joblib1.4.2。当前/tmp/t1_qp_deps已存在；临时目录会被清理，不是项目长期依赖目录。Windows不可直接执行这些命令。

```bash
source /opt/ros/jazzy/setup.bash
export OPENBLAS_NUM_THREADS=1
export PYTHONPATH=/tmp/t1_qp_deps:${PYTHONPATH:-}
python3 -c 'import numpy,scipy,osqp; assert (numpy.__version__,scipy.__version__,osqp.__version__)==("1.26.4","1.11.4","1.1.3")'
python3 examples/head_on_mpc/results/M1_motion_freeze_001/source/run_m0.py --help
```

如果只缺OSQP/joblib，可按原已核对清单恢复到临时目录，不修改系统。此步骤需要网络；若系统已有准确版本可跳过。

```bash
python3 - <<'PY'
import hashlib,json,urllib.request,zipfile
from pathlib import Path
manifest=Path('examples/head_on_mpc/results/bicycle_e3_002/dependency_download_manifest.json')
target=Path('/tmp/t1_qp_deps');target.mkdir(exist_ok=True)
for item in json.loads(manifest.read_text()):
    wheel=Path('/tmp')/item['filename']
    if not wheel.exists() or hashlib.sha256(wheel.read_bytes()).hexdigest()!=item['sha256']:
        with urllib.request.urlopen(item['url']) as response:wheel.write_bytes(response.read())
    assert hashlib.sha256(wheel.read_bytes()).hexdigest()==item['sha256']
    with zipfile.ZipFile(wheel) as archive:archive.extractall(target)
PY
```

ROS2/NumPy/SciPy由声明的环境提供，不使用这段脚本随意升级。历史记录未给rclpy/DDS具体补丁构建，M1只核对已声明的ROS2 Jazzy环境，不声称未记录的二进制身份也得到证明。

## 按需完整复现，不是本轮已执行的步骤

以下使用本目录source中的已有run_m0入口，不依赖将来可能修改的维护代码。它会执行原入口定义的冷/热两臂各两轮；所选B2-C对应输出中的B2_sequence_warm_r1/r2。冷启动臂仅作额外诊断，并不改变已冻结选择。没有为M1新增控制器包装层或算法选项。

必须在原git工作区内调用，因为原入口记录git HEAD/diff；即便从冻结source运行，也不把当前git HEAD当成冻结运动版本的替代。本目录config和两份数学文档按原ROOT布局提供给入口。

```bash
source /opt/ros/jazzy/setup.bash
export OPENBLAS_NUM_THREADS=1
export PYTHONPATH=/tmp/t1_qp_deps:${PYTHONPATH:-}
export ROS_DOMAIN_ID=99
export ROS_LOG_DIR=/tmp/m1_reproduction_ros
# 输出目录必须尚不存在；已存在时换新的递增编号。
python3 examples/head_on_mpc/results/M1_motion_freeze_001/source/run_m0.py --output examples/head_on_mpc/results/M1_reproduction_001
```

运行器会对每条新日志执行原独立验收、ROS和未来执行一致性检查，结果见各运行目录。需要单独重读时：

```bash
python3 examples/head_on_mpc/results/M1_motion_freeze_001/source/verify_m0.py --run examples/head_on_mpc/results/M1_reproduction_001/B2_sequence_warm_r1
```

读取independent_validation.json.passed、ros_validation.json和future_execution_consistency.json，而不是只看脚本退出码或summary.both_completed。该入口的独立验收CLI没有用非零退出码代替验收字段。保留所有拒绝和超时；新轨迹如与冻结证据不同，先定位，不自动继承M1关闭或改名为新基线。80.4 s只是已有观测，不是新环境的硬性性能承诺。

## 已有证据入口

- ../M0_closure_001/closed_loop/repaired_r1、repaired_r2：选定候选原始轨迹、控制、求解、授权、独立验收及全部周期计时。
- ../M0_closure_001/closed_loop/unrepaired_r1、unrepaired_r2：到达目标但未通过原跨零契约的保留对照。
- ../M0_closure_001/hand_cases.json、classification.json、frozen/、local_paired/：人工语义测试、状态来源和局部机制。
- ../bicycle_e3_002/B1：历史完整完成诊断，非同口径性能对手或在线备用。
- ../B1&B2-T1：按哈希定位的T1同源材料；../M0_001/mainline_plan_v1.txt：主线边界。

source_manifest.json列出原始工件逐文件SHA256，不重复复制四条大型日志；没有runs/新运行目录，因为本M1没有新闭环。
