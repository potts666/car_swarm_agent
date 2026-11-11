# M2a复现与交接

正式设置是design/design_v4.md、settings.json、message_schema.json及source_manifest.json；design.md保留最初v1，不能误当最终v4。每次运行旁的*_source保存完整运行源码及版本/线程记录。历史M1/M0目录未修改，C-old直接复用原已验收证据。

从仓库根运行。需要ROS2 Jazzy/rclpy/std_msgs、Python3.12.3、NumPy1.26.4、SciPy1.11.4、OSQP1.1.3、joblib1.4.2。OSQP/joblib原安装到临时目录/tmp/t1_qp_deps；重启或清理后须恢复相同版本，或将PYTHONPATH改为同版本的实际安装目录。续跑期间确实发生临时目录丢失，恢复版本后补测；没有因此重写正式运行数据。

```bash
source /opt/ros/jazzy/setup.bash
export OPENBLAS_NUM_THREADS=1
export PYTHONPATH=/tmp/t1_qp_deps:$PYTHONPATH
export ROS_LOG_DIR=/tmp/m2a_ros
export ROS_DOMAIN_ID=109
python3 examples/head_on_mpc/scripts/run_m2a.py --arm L-new --output /tmp/M2a_L_new_repeat
python3 examples/head_on_mpc/scripts/verify_m2a.py --run /tmp/M2a_L_new_repeat
# 仅在在线运行结束后执行旧中央未来诊断
python3 examples/head_on_mpc/scripts/audit_m2a_future.py --run /tmp/M2a_L_new_repeat
```

输出目录必须未存在。C-new使用同一入口、--arm C-new和另一个新目录；两臂顺序运行、线程环境相同。读取independent_validation.json的actual_full_acceptance，不能用进程退出码代替验收。未来诊断结果单独保存在centralized_future_diagnostic.json，不能算成局部未来证明或回写运动。正式四次运行已完成，不因文档改动重复跑控制。

```bash
python3 -m unittest discover -s examples/head_on_mpc/tests -p test_m2a.py -v
python3 examples/head_on_mpc/results/M1_motion_freeze_001/verify_freeze.py
python3 examples/head_on_mpc/results/M2a_local_dmpc_001/verify_delivery.py
```

本轮新增11项协议/局部QP测试；M1的34项原运动/停车回归复用已冻结证据，并核对维护源码哈希，未将它们冒充本次新跑。新独立验收实际重积分四次正式完整轨迹及最终预测，覆盖本轮运动影响。

M2b进入条件已在当前双车固定场景满足。后续必须另行设计多车邻接、有限轮协调及信息边界，再评估四/六车与批次；本轮不包含这些扩展、延迟或在线备用。三个ROS节点在同一进程中使用可靠DDS，不代表独立硬件部署。局部每车32/64变量由代码和日志核对，不能仅以节点数认定分布式。

周期最大约524 ms，多个周期超过200 ms，计算期间状态冻结，未满足硬实时要求。新释放只是6.4 s有限预测兼容，不能代替旧全未来契约或证明递归可行。采样验收及两次确定性复现也不构成连续安全证明。
