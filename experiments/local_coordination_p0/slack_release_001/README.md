# slack_release_001

Read report.md, GATE.json, release_theory.md, method_spec.md. All8 native attempts consumed; do not rerun numeric stages.

`python3 verify_delivery.py` validates SHA, raw adoption, events, Phi/B and actual history.
`python3 verify_delivery.py --full-replay` independently replays every unique complete main/backup/mixed/recovery policy, zero new solves. Verification uses standard Python only; optional compiler reconstruction uses requirements.txt and /tmp/dmpc-p0-deps.

Source numerical_v1 is immutable. Analytic_v0 and all evaluator/guard failures retained. SHA excludes mutable .log/stage-event logs and the checksum file itself; archive checksum protects the whole package.
