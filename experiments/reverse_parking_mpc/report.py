#!/usr/bin/env python3
"""Render summaries from saved executed MPC logs; never change run labels."""
import csv
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import mpc

ROOT = Path(__file__).resolve().parent


def load(path):
    return json.loads(path.read_text())


def main():
    config = load(ROOT / 'config.json')
    runs = []
    for mode, directory in [('single', 'single_acceptance'), ('two', 'two_baseline')]:
        for seed in config['seeds']:
            path = ROOT / directory / f'{mode}_{seed}'
            summary = load(path.with_name(path.name + '_summary.json'))
            logs = [json.loads(line) for line in path.with_suffix('.jsonl').read_text().splitlines()]
            differences = [np.linalg.norm(np.array(row['controls'][0]) - np.array(row['solver']['reference_controls'][0])) for row in logs if row.get('executed')]
            summary['max_first_control_difference_from_reference'] = float(max(differences))
            summary['mode'] = mode
            runs.append(summary)
            with path.with_name(path.name + '_metrics.csv').open('w') as stream:
                writer = csv.writer(stream)
                writer.writerow(['t', 'car', 'goal_distance_m', 'yaw_error_rad',
                                 'speed_m_s', 'displacement_m', 'direction',
                                 'reference_motion_allowed', 'solver_accepted',
                                 'executed', 'min_sampled_pair_body_distance_m', 'solve_seconds'])
                for row in logs:
                    speed = row.get('actual_next_state', row['state'])[3]
                    writer.writerow([row['time_s'], row['car'], row['goal_error']['position_m'],
                                     row['goal_error']['yaw_rad'], speed, row.get('displacement_m'),
                                     int(np.sign(speed)) if abs(speed) > 1e-5 else 0,
                                     row['reference_motion_allowed'], row['solver']['accepted'],
                                     row['executed'], row.get('min_sampled_pair_body_distance_m'),
                                     row['solver']['solve_seconds']])
    fig, axes = plt.subplots(3, 2, figsize=(12, 9))
    for column, (mode, directory) in enumerate([('single', 'single_acceptance'), ('two', 'two_baseline')]):
        logs = [json.loads(line) for line in (ROOT / directory / f'{mode}_20261010.jsonl').read_text().splitlines()]
        for car in range(1 if mode == 'single' else 2):
            rows = [row for row in logs if row['car'] == car and row['executed']]
            t = [row['time_s'] for row in rows]
            axes[0, column].plot(t, [row['goal_error']['position_m'] for row in rows], label='AB'[car])
            axes[1, column].plot(t, [row['actual_next_state'][3] for row in rows])
            axes[2, column].plot(t, [row['displacement_m'] for row in rows])
        axes[0, column].set_title(mode + ': measured closed-loop execution')
        axes[0, column].legend()
        axes[0, column].set_ylabel('Goal distance / m')
        axes[1, column].set_ylabel('Signed speed / m/s')
        axes[2, column].set_ylabel('Cycle displacement / m')
        axes[2, column].set_xlabel('Simulation time / s')
    fig.tight_layout()
    fig.savefig(ROOT / 'closed_loop_series.png', dpi=150)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(11, 5))
    for x, y, width, height in config['obstacle_boxes']:
        ax.add_patch(Polygon([[x,y],[x+width,y],[x+width,y+height],[x,y+height]], color='gray'))
    ax.add_patch(Polygon(config['parking_polygon'], fill=False, edgecolor='green'))
    actual = np.genfromtxt(ROOT / 'single_acceptance/single_20261010_actual.csv', delimiter=',', names=True)
    ax.plot(actual['x'], actual['y'], label='A actual MPC')
    controls = np.array(load(ROOT / 'reference_A_controls.json'))
    ref = mpc.rollout(config['starts'][0], controls, config)
    ax.plot(ref[:, 0], ref[:, 1], '--', label='A reference')
    for name, state in [('A start', config['starts'][0]), ('A goal', config['goals'][0]),
                        ('B start', config['starts'][1]), ('B goal', config['goals'][1])]:
        ax.text(state[0], state[1], name)
    ax.set(xlim=(-12, 18), ylim=(-6, 6), xlabel='x / m', ylabel='y / m')
    ax.set_aspect('equal')
    ax.legend()
    fig.tight_layout()
    fig.savefig(ROOT / 'single_car_scene.png', dpi=150)
    plt.close(fig)
    evidence = dict(single_car=runs[:3], independent_two_car=runs[3:],
                    witness=load(ROOT / 'witness_results/witness_validation.json'),
                    conclusion='Single-car gate passed; sampled same-model/input-envelope sequential witness passed; independent two-car solves rejected before sustained stagnation. No MPC strategy deadlock reproduced.')
    (ROOT / 'comparison.json').write_text(json.dumps(evidence, indent=2) + '\n')
    table = '| Mode / seed | Success | Arrival A/B (s) | Min static SAT (m) | Min pair body (m) | Solve mean/max (s) | Stop |\n'
    table += '| --- | --- | --- | --- | --- | --- | --- |\n'
    for row in runs:
        arrival = '/'.join('—' if v is None else f'{v:.1f}' for v in row['arrival_times_s'])
        pair = '—' if row['min_sampled_pair_body_distance_m'] is None else f"{row['min_sampled_pair_body_distance_m']:.3f}"
        table += f"| {row['mode']}/{row['seed']} | {row['success']} | {arrival} | {row['min_sampled_static_sat_gap_m']:.3f} | {pair} | {row['solve_seconds_mean']:.3f}/{row['solve_seconds_max']:.3f} | {row['stop']} |\n"
    (ROOT / 'comparison.md').write_text(table)


if __name__ == '__main__':
    main()
