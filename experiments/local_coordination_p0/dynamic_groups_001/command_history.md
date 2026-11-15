# 实际命令与证据索引

以下命令均在本目录执行，数值阶段统一设置OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps。原始stdout/stderr不裁剪；阶段exit_code和耗时另见results/*.stage.json及results/commands.jsonl。

- `python3 precheck.py`：results/precheck.attempt_1.log至attempt_5.log。首次初始化失败、第二次活动行零梯度扫描被主动中断、之后补发现身份和实际权限接线，最终721/722窗口冻结，始终0调用。第一条误用相对目录的启动输出在results/precheck.initial.log。
- `python3 check_protocol.py`：results/protocol.attempt_1.log（发现身份缺失）、attempt_2/3.log；最终14项零调用通过，stage B原始两条完整拒绝轨迹保存results/rejection/stageB_*。
- `python3 check_generation.py`：results/generation_regression.attempt_1.log，7类真实源回归；0调用。
- `python3 check_terminal.py`：results/terminal_hold.attempt_1.log，813拍到目标、829拍接收/零K；0调用。
- `python3 stage_driver.py`：results/stage_driver.log首次F-S完成4调用后，独立验收发现served快照；v2.log为归档路径检查修正前0新增调用失败；v3.log完整继续独立验收与F-P/D-P剩余8调用。原源码和逐拍执行日志保留；F-S源字段重建清单在其结果目录。
- `python3 stage_driver.py`再运行：results/stage_reuse_check.log，六阶段全部内容核对复用，账本仍12。
- `python3 delivery_tools/postsolve_rejections.py`：results/rejection/postsolve.log，已受理真实候选的15项分支；0新调用，失败车/块回退同时保留另一车新候选。
- `python3 delivery_tools/accept_baseline.py`：results/baseline_independent.log；首次为避开数值计时主动中断的完整日志与部分轨迹保留*.interrupted.*，最终完整基线独立通过。
- `python3 delivery_tools/accept_forks.py`：results/rejection/independent_acceptance.log；两条实际回退组合各2081拍独立通过。
- `python3 delivery_tools/timing_replay.py`：results/timing_replay.log；六轮短协议分项零调用复测。
- `python3 delivery_tools/summarize.py`：results/final_summary.log，逐项从内容对象/唯一绝对拍号重算统计；0新调用。
- `python3 delivery_tools/package_delivery.py`：最初输出results/package.initial.log，最终输出放/tmp以避免清单与正在写入日志递归依赖。
- `python3 verify_delivery.py`及搬移后的同一入口：结果在verification/。`--fresh-run`仅准备临时新目录/空账本，不运行求解。

依赖：系统缺clarabel且pip也不存在；先下载官方pip.pyz到/tmp，再`python3 /tmp/dmpc-pip.pyz install --target /tmp/dmpc-p0-deps clarabel==0.11.1 osqp==1.1.3 numpy==1.26.4 scipy==1.11.4`恢复历史固定版本。没有改系统环境；实际环境版本在environment.json。安装工具的临时setuptools与系统colcon版本提示不影响此隔离数值运行，未运行ROS构建。
