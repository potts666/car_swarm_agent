目标：C0两接口诊断及实际接续；四次求解完成，证书策略接续已通过，严格Solved仍失败。
约束：A/B/C/P0及论文保留；不推进后续阶段。
源版本：ebc3d29f5d7caf0b77b6d11887f82252e9a151dc
证据：GATE.json、results/metrics.json、两份independent_acceptance.json、comparison.csv。
恢复：OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 run_validation.py
交付复核：OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 verify_delivery.py
