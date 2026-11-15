# C-stage local results

Read report.md and STATUS.md. This delivery is intentionally not an all-pass gate: C0 exhausted the two registered variants/three settings without Solved; C1 passed; C2 was not started.

C0_comparison.csv distinguishes solver state from original-row/quantization diagnostics. Passing a diagnostic prediction is not execution. C2_comparison.csv explicitly records not_run. Full P/q/A/b and large diagnostic rows are content-addressed in objects, with frozen B dependencies under base_b.

The exact C1 replay is self-contained in analytic_c1.py and results/C1_001. Reconstructing the two saved cases with independent() does not rerun an optimizer. The scientific Python dependency versions are unchanged from base_b/requirements-development.txt.

selected_repair.json is the last probe input only; repair_selection.json has selected=null. No C2 execution program is provided or authorized by this failed C0 gate. Do not use a low residual or an original-row passing point to bypass the required Solved status.
