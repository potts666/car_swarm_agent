# M2b_transition_parallel_001

先读 report.md、comparison.csv、determinism.json、delivery_status.json。驱动未结束时以 driver.log / study_status.json / driver_failure.json 为准，前缀不是完整任务通过。

- reconstruction/：94 个历史重建案例、case_matrix.csv、计数、debug_v1 停车状态与冻结 QP 矩阵诊断；debug_v2 缺失项明确记录。
- debug_serial_80、debug_parallel_6、debug_parallel_200：正式前有界调试，停止原因 prefix_limit，保留原日志。最早 6 周期调试早于完整轮号审计元数据，不作为完整验收。
- rename_prefix、position_prefix：两种在看结果前确定的小变体前缀，均经独立验收。
- S-J_1、P-J_1、P-J_2、S-J_2：同一冻结算法、交错运行，含原始 gzip 日志、配置、完整预测内容、来源引用、独立验收。
- rename_full、position_full：小变体完整检查（只有实际完成并验收后才算通过）。
- sources/<current_source_version>/：冻结脚本、测试、夹具、配置及 manifest。
- supplemental_sources/：独立复现发现的5个遗漏传递依赖，单独哈希，未覆写冻结主 manifest；补录后完整31项测试通过。首次失败日志仍保留。
- design.md / literature_conditions.md / metric_dictionary.csv：周期时序、检查层次、论文版本与条件、指标口径。
- *_concurrency.csv / *_prediction_changes.csv / cycle_p95.png / parallel_overlap.png：从原始日志派生的并发和时间对齐预测指标。
- *.checkpoint.json：驱动在独立验收后记录的所有阶段文件 SHA256。恢复时须源码、内容全部匹配，不能凭目录存在就跳过。

复现环境：Linux Python3.12、ROS2 Jazzy、numpy1.26.4/scipy1.11.4/osqp1.1.3/joblib1.4.2，数值线程固定为1。依赖安装与wheel来源沿用 M2b_stage1_001/test_reproduction/requirements.txt 及该目录上级的wheel URL/SHA材料；/tmp/m2b_stage1_repro_deps 是本机临时依赖位置，不是持久交付依赖。

先把 sources/<version> 复制到一个新的工作目录，再将 supplemental_sources/scripts 下的5个文件覆盖复制到其 scripts/（只补不存在的文件）。核对两个 manifest 的所有 SHA256。隔离测试入口：

```
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=/path/to/numerical_deps /usr/bin/python3 -m unittest discover -s /path/to/copied_snapshot/tests -p 'test_m2b*.py' -q
```

独立单次运行（输出目录必须不存在；先 source /opt/ros/jazzy/setup.bash 并设置相同数值环境）：

```
/usr/bin/python3 /path/to/copied_snapshot/scripts/run_m2b_jacobi.py --backend P-J --max-rounds 1 --output /path/to/new_run
/usr/bin/python3 /path/to/copied_snapshot/scripts/verify_m2b.py --run /path/to/new_run/r1
```

--backend S-J 只改变执行后端；--variant rename 或 position 使用任务内冻结的小变体；--max-cycles 80 只做有界前缀。全部固定初态、目标、原物理门限不另改。

当前维护仓库驱动：scripts/run_m2b_transition_study.py --root results/M2b_transition_parallel_001。准确仓库相对入口见 HANDOFF.md，不要在已有驱动仍运行时另起同名驱动。报告生成器 make_delivery.py 与 prediction_metrics.py 只读结果，不回写验收或运行日志。

原始消息采用单份计划引用，可用 m2b_recording.lines 恢复，并检查原ID和应用层字节数。引用内容完整保存在 plans.jsonl.gz，非仅有哈希。


收尾版本：current_source_version.txt 专指正式冻结版本；maintenance_source_version.txt 为只加固释放报文消费的维护版本。post_review_change.json 列唯一源码差异；post_review_validation.json 记录新源码260周期功能前缀独立通过且计划ID不变。release_adapter_replay.json 为六次保存运行的1002次资源准备重放和96次坏报文拒绝。test_release_adapter.py 是对应测试源码；post_review_regression_tests.log 为维护版本31项回归。

六次完整运行不重新标成硬化版本。复现正式数据请用 current_source_version 对应快照加 supplemental_sources；复现最终维护代码请用 maintenance_source_version 对应快照加相同补录依赖。现有driver已经完成，维护源码版本变化后向原目录resume会正确拒绝checkpoint/source不一致；新实验必须使用新输出目录。post_review_prefix260只作受影响路径验证，不计入正式性能组。

交付包还含 reconstruction/baseline_trace.jsonl.gz（实际输入、采样状态、公开快照和参考记录的最小恢复材料）。可执行 verify_reconstruction_trace.py，从包内材料恢复882周期队列并匹配94案例，无需读取历史完整预测正文。依赖版本与原wheel来源补录在 dependencies/，第三方二进制和ROS不内置。
