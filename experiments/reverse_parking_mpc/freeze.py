#!/usr/bin/env python3
"""Freeze only after all single-car runs and same-constraint witness pass."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent
CORE = ['mpc.py', 'run.py', 'config.json', 'reference_A_controls.json',
        'reference_B_controls.json', '../parking_conflict/frozen/optimize_spatial_temporal.py']


def freeze():
    config = json.loads((ROOT / 'config.json').read_text())
    manifest = json.loads((ROOT / 'single_acceptance/manifest.json').read_text())
    hashes = {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in CORE}
    for name in CORE[:-1]:
        if manifest['files_sha256'][name] != hashes[name]:
            raise RuntimeError('Single-car acceptance source changed: ' + name)
    reports = []
    for seed in config['seeds']:
        report = json.loads((ROOT / f'single_acceptance/single_{seed}_summary.json').read_text())
        if not report['success'] or report['maximum_actual_sampled_violation'] > config['mpc']['constraint_tolerance']:
            raise RuntimeError('Single-car gate failed: ' + str(seed))
        reports.append(report)
    witness = json.loads((ROOT / 'witness_results/witness_validation.json').read_text())
    if not witness['pass_all_declared_sampled_constraints']:
        raise RuntimeError('Same-model sampled witness did not pass')
    result = dict(single_car_gate_passed=True, single_car_runs=reports,
                  sampled_counterfactual_passed=True, core_sha256=hashes,
                  original_scenario_sha256=config['source_scenario_sha256'],
                  declaration='Frozen after single-car acceptance and before any two-car baseline run. Physical limits and geometry unchanged; signed timed control references and explicit stop/gear modes added. No continuous-time proof.')
    (ROOT / 'frozen_baseline.json').write_text(json.dumps(result, indent=2) + '\n')
    print('Frozen signed-speed MPC baseline: 3/3 single-car acceptance, sampled witness passed')


if __name__ == '__main__':
    freeze()
