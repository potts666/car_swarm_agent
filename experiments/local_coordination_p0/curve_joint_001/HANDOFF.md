# Completed boundary
Goal: curve_joint_001 four fair arms, two actual rounds and full completion.
Constraints: 16 native attempts consumed, no further solves authorized in this task; no Git commits.
Source: executed_source_versions.json; source_archive/numerical_v1.
Evidence: GATE.json, report.md, results/independent_acceptance.json, solver_calls.json.
Remaining: no experiment issue; default verification and full physical replay passed. Archive/portable-read check is the final delivery step.
Resume: OPENBLAS_NUM_THREADS=1 PYTHONPATH=/tmp/dmpc-p0-deps python3 experiments/local_coordination_p0/curve_joint_001/verify_delivery.py --full-replay
