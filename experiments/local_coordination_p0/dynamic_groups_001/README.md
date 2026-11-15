# dynamic_groups_001

本轮独立目录；历史003/002只读。主线依据mainline_checkpoint_2026-11-14_v2和DG-DMPC-theory-v1，原文在mainline_checkpoint.md及theory/。实际结果见report.md、GATE.json；源与历史身份在source_inventory.json，调用预算在solver_calls.json。

执行环境恢复到/tmp/dmpc-p0-deps，固定Clarabel0.11.1、NumPy1.26.4、SciPy1.11.4、OSQP1.1.3。未改系统Python。数值环境需设置OPENBLAS_NUM_THREADS=1、OMP_NUM_THREADS=1、MKL_NUM_THREADS=1。

运行阶段入口：
```
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 stage_driver.py
```
已完成阶段按内容复用。账本登记过的数值事件不得重跑；失败日志保留。默认复核、完整重放与搬移入口见verify_delivery.py。所有模块只在本目录写入，新内容对象使用SHA256去重及压缩。

F-S早期逐拍source.served字段存在可变列表快照问题，原日志未修改；单独重建4处源字段并保存逐项变更与两份哈希。数值源事件、实际命令、物理状态、history和期限均保持。原源码在source_archive/，原验收失败在results/F-S_accept.stage.log。后续源复制该列表，不再发生此问题。

不提交Git，不修改论文；合并/拆分本轮为标明fork的标签迁移。逻辑h=.2s和实际计算墙钟分别报告。

默认零调用内容核对与关键前缀独立重算：
```
OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 verify_delivery.py
```
完整三臂独立物理重放（零求解）：同一命令加`--full-replay`。`--fresh-run`仅在/tmp准备新目录和空账本，不运行数值，并标明需另设预算；本轮12次不能重复使用。分项补充计时在timing_components.csv中明确标为零调用重放。

SHA256SUMS覆盖冻结代码、原始证据及报告；verification/保存清单冻结后生成的派生复核结果，避免结果自引用。这些派生结果由最终tar.gz整体SHA256覆盖。实际命令与保留失败见command_history.md。
