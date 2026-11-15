目标：两个既有接口连续两轮重新优化及实际接续。
状态：4次新求解完成，各2次受理/首拍改变，两个240拍独立验收与10个拒绝续行分支通过；严格Solved未通过。
约束：不改历史/P0/论文；不推进预约、多活动车、C2或批量。
源提交：ebc3d29f5d7caf0b77b6d11887f82252e9a151dc
证据：GATE.json、comparison.csv、metrics.json、event_index.json、results/*/independent_acceptance.json。
恢复：OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 run_stage.py
分发复核：OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 verify_delivery.py
