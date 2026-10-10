#!/usr/bin/env python3
"""Plot and tabulate the fixed local problem without changing solve results."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import diagnose as d

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--results', type=Path, default=ROOT / 'results')
    result = parser.parse_args().results
    problem = json.loads((result / 'problem.json').read_text())
    audit = json.loads((result / 'constraint_policy_audit.json').read_text())
    control_reference = np.array(json.loads((d.BASE / 'reference_A_controls.json').read_text()))
    reference = d.mpc.reference(problem['config']['starts'][0], control_reference, problem['config'])
    gear = reference['gear']
    segments = []
    start = 0
    for index in range(1, len(gear) + 1):
        if index == len(gear) or gear[index] != gear[start]:
            segments.append(dict(start_s=start * .4, end_s=index * .4, mode=int(gear[start])))
            start = index
    stop = gear == 0
    stop[:-1] |= gear[1:] == 0
    stop[:-1] |= gear[:-1] * gear[1:] < 0
    audit['whole_A_reference_mode_intervals'] = segments
    audit['whole_A_reference_forced_stop_endpoints_s'] = ((np.flatnonzero(stop) + 1) * .4).tolist()
    audit['reference_tail_behavior'] = 'window() uses gear=0 and zero reference controls after reference exhaustion; forcing zero speed at every endpoint prevents a stopped car from catching up to a missed goal. Policy restriction, not vehicle requirement.'
    (result / 'full_reference_policy_audit.json').write_text(json.dumps(audit, indent=2) + '\n')
    candidates = json.loads((result / 'candidates.json').read_text())
    refined = json.loads((result / 'objective_from_feasible.json').read_text())
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    times = problem['time_s'] + np.arange(1, 49) * .1
    rows = []
    for name, record in candidates.items():
        prediction = np.array(record['prediction'])
        gap = np.array(record['constraints']['predicted_peer_body_gap']['values']) + .08
        axes[0].plot(times, gap, label=name)
        axes[1].plot(times, prediction[1:, 3])
        axes[2].plot(times, prediction[1:, 4])
        stop = np.flatnonzero(abs(prediction[:, 3]) <= 1e-10)
        stop_time = problem['time_s'] + stop[0] * .1 if len(stop) else None
        rows.append([name, record['feasible_same_local_constraints'],
                     f"{record['maximum_violation']:.9g}",
                     f"{record['min_sampled_peer_sat_gap_m']:.6f}",
                     '—' if stop_time is None else f'{stop_time:.1f}',
                     f"{record['objective']:.6f}"])
        with (result / f'{name}_predictions.csv').open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(['t', 'x', 'y', 'yaw', 'signed_speed', 'steering'])
            writer.writerows(np.column_stack((np.r_[problem['time_s'], times], prediction)))
    axes[0].axhline(.08, color='black', linestyle='--', label='required SAT gap')
    axes[0].set_ylabel('Predicted padded SAT gap / m')
    axes[0].legend(fontsize=8)
    axes[1].set_ylabel('Signed speed / m/s')
    axes[2].set_ylabel('Steering / rad')
    axes[2].set_xlabel('Absolute relative-simulation time / s')
    fig.tight_layout()
    fig.savefig(result / 'local_candidates.png', dpi=160)
    plt.close(fig)
    table = '| Candidate | Same local constraints | Max violation | Min peer SAT gap (m) | Stops at (s) | Objective |\n'
    table += '| --- | --- | --- | --- | --- | --- |\n'
    table += '\n'.join('| ' + ' | '.join(map(str, row)) + ' |' for row in rows) + '\n'
    table += '\n| Original objective from Phase I output | Accepted | Max violation | Objective |\n'
    table += '| --- | --- | --- | --- |\n'
    table += '\n'.join(f"| {row['initialization']} | {row['accepted']} | {row['validation']['maximum_violation']:.9g} | {row['objective']:.6f} |" for row in refined) + '\n'
    (result / 'comparison.md').write_text(table)
    phase = json.loads((result / 'phase_one.json').read_text())
    with (result / 'phase_one_metrics.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['initialization', 'optimizer_success', 'slack', 'max_violation', 'actually_feasible'])
        writer.writerows([row['initialization'], row['optimizer_success'], row['slack'],
                          row['validation']['maximum_violation'], row['validation']['feasible_same_local_constraints']] for row in phase)


if __name__ == '__main__':
    main()
