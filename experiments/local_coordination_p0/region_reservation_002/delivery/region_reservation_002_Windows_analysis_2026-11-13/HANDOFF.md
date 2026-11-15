目标：用户队列1→2→3，本轮先完成最关键的两车重叠连续两拍优化。
状态：2用例×2新求解全部采用；新接收820/K820，两个2802拍独立验收、12个拒绝旧尾续行通过。
限制：名义同步模型；历史和论文保留；不做批量。
源码与证据：executed_source_versions.json、shifted_receiver_certificate.json、GATE.json、results/*/independent_acceptance.json。
恢复：OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 run_stage.py
复核：OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 verify_delivery.py
后续：work_queue.json，优先2倒车SOCP，再3更多组合。
