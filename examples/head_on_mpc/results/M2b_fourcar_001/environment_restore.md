# 环境与恢复入口

实际运行环境及完整命令逐组保存在C-FIFO/L-FIFO/L-ALT/environment.json。Python3.12.3、NumPy1.26.4、SciPy1.11.4、OSQP1.1.3，ROS2 Jazzy/rclpy；OPENBLAS_NUM_THREADS=1，实际五个ROS节点同进程、可靠DDS，隔离域121/122/123。OSQP所需joblib使用1.4.2。车辆能力是离线模型假设，不是硬件标定。

原临时依赖路径为/tmp/t1_qp_deps；会话环境重置后该目录可能消失，并不表示实验未运行。重新执行数学/控制检查前，在自己的依赖环境恢复相同版本，不能只复制PYTHONPATH命令就假定目录仍在。有pip的环境可安装到临时目录，不升级系统NumPy/SciPy：

```bash
python3 -m pip install --target /tmp/t1_qp_deps --no-deps osqp==1.1.3 joblib==1.4.2
source /opt/ros/jazzy/setup.bash
export PYTHONPATH=/tmp/t1_qp_deps:$PYTHONPATH
export OPENBLAS_NUM_THREADS=1
export ROS_LOG_DIR=/tmp/m2b_ros
```

若系统Python没有pip，不应为此升级系统包；使用已准备的同版本Python环境或按照既有平台流程恢复wheel。运行器输出目录须不存在，新的整套正式实验须另起编号，不能覆盖本包。

只检查已归档文件完整性无需上述数值依赖：

```bash
python3 M2b_fourcar_001/verify_artifact_hashes.py
```

完整数学/协议验收在维护仓库用scripts/verify_m2b.py，原理是独立重积分而非读取summary为true。补充种子、完整周期口径和归档工具见analysis_source及相应JSON；调用维护目录m2b_timing_post.py可在原报告生成后补回包含诊断整理的完整提交周期间隔。包内source快照是证据，长期维护仍在examples/head_on_mpc/scripts。

诊断正文共享对象必须随包保留。恢复命令见report.md；不能把引用中的SHA当作正文。恢复的逻辑JSONL字节与归档前一致，gzip封装自身的元数据/字节不要求相同。2份中断时空索引及21个空对象的恢复来源、逐内容ID匹配与损坏副本见archive_checkpoint_recovery.json及interrupted_archive_files/；没有从新一次求解猜测或重造矩阵。

打包采用tar保留同内容计划文件硬链接；普通复制展开硬链接会增加物理占用。不能在硬链接的冻结计划文件上原地编辑；重新试验使用新输出目录。
