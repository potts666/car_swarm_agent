# 短交接：M2b第一阶段

目标：保护历史结果，解释疑问，定位灰/黄开销，优化重复本端检查；先验收再进入并行/连续转向。原四车场景、能力、门限、FIFO、单步导数、3次更新、0.5秒预测预算不变。

维护修改：scripts/m2b_protocol.py 的端点私有计划检查缓存；run_m2b.py 的缓存接线/计时/前缀入口；新增m2b_timing.py。源码快照和哈希在本目录 sources/。控制QP、物理模型和场景源文件保持历史哈希。

证据：prefix_results.json 的3个6周期ROS2前缀均独立验收通过，实际轨迹哈希一致；21项受影响测试通过。questions.md解释16项疑问；literature_notes.md记录原文核查，未声称并行/连续转向已实现。

已完成唯一完整L-FIFO由 run_full_validation.py 自动推进与独立验收。只读状态：full_status.json；进度：full_execution.log；结果：full_L_FIFO/r1/summary.json及independent_validation.json。不要再次启动该脚本覆盖已有目录。完整独立验收通过，176.4秒、882周期，逻辑实际轨迹与历史r1一致。完整在线543.774秒，P50/P95/P99/max=598.20/710.15/896.09/1350.92毫秒，仍全部超过200毫秒。21测试及人工失败出口检查通过。

原数值依赖已按官方wheel SHA恢复至/tmp/m2b_stage1_repro_deps（osqp1.1.3/joblib1.4.2），没有系统pip。环境入口：source /opt/ros/jazzy/setup.bash；OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONPATH=/tmp/m2b_stage1_repro_deps:$PYTHONPATH。

第一阶段汇总/灰黄采样/报告已完成，见report.md、comparison.csv、measurements.json。未完成：第二阶段真实并行、第三阶段连续转向受控验证。当前密集计时开销可见，不作为免费观测；6周期不是全任务性能证据。若运行失败保留反例，不改成功标签。

精确测试入口（无需完整仿真）：PYTHONPATH=/tmp/m2b_stage1_repro_deps:examples/head_on_mpc/tests:$PYTHONPATH OPENBLAS_NUM_THREADS=1 python3 -m unittest test_m2b_stage1 test_m2b_protocol test_m2b_control test_stopping_acceptance test_m2b_scene。

包内独立测试入口：PYTHONPATH=/tmp/m2b_stage1_repro_deps OPENBLAS_NUM_THREADS=1 python3 test_reproduction/run_tests.py（从本结果目录执行）；该快照为交付补录，复核日志另存test_snapshot_validation.log。完整运行与B_no_profiler的cache_hits为null/未测，不能解读为零命中。
