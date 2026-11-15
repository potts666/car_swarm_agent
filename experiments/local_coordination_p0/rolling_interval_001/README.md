# rolling_interval_001

Read report.md and GATE.json. Verify: `python3 verify_delivery.py`; physical actual full replay: `python3 verify_delivery.py --full-replay`. Both release zero new solves. Main budget is budget_ledger.json; historical solver_calls.json is empty and not the new budget.

Numerical entry: rolling.py; budget_ledger.json is authoritative. engine.py supplies inherited analytic/release routines; its legacy numerical settings are not the rolling experiment configuration. Frozen execution sources: rolling_v1/v2/v3. Independent judge/report sources: acceptance_v0/v1/v2. Source isolation certificate records strict white-listed regeneration before historical policy comparison.
