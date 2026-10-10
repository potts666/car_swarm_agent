#!/usr/bin/env python3
"""Derive comparison and measured cycle CSVs from saved immutable run logs."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import experiment as e


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=e.ROOT / 'results')
    out = parser.parse_args().output
    comparison = json.loads((out / 'comparison.json').read_text())
    rows = []
    for report in comparison['closed_loop']:
        name = f"{report['policy']}_{report['seed']}"
        logs = [json.loads(line) for line in (out / f'{name}.jsonl').read_text().splitlines()]
        with (out / f'{name}_metrics.csv').open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(['cycle', 't', 'car', 'goal_distance_m', 'yaw_error_rad',
                             'displacement_m', 'speed_m_s', 'direction', 'permitted',
                             'optimizer_accepted', 'sampled_pair_body_distance_m',
                             'solve_seconds'])
            for cycle in sorted({row['cycle'] for row in logs}):
                pair = [row for row in logs if row['cycle'] == cycle]
                if all(row.get('executed') for row in pair):
                    dense = [e.simulate(row['state'], np.array(row['controls'])[:1]) for row in pair]
                    gap = float(np.min(e.body_distance(dense[0][:, :2], dense[0][:, 2],
                                                      dense[1][:, :2], dense[1][:, 2])))
                else:
                    gap = None
                for row in pair:
                    writer.writerow([cycle, row['t'], row['car'],
                                     row['goal_errors']['distance_m'],
                                     row['goal_errors']['yaw_error_rad'],
                                     row.get('displacement_m'),
                                     row.get('actual_next_state', row['state'])[3],
                                     row.get('direction'), row['permission'],
                                     row['solver']['accepted'], gap,
                                     row['solver']['solve_seconds']])
        rows.append([name, '0/1', '—', f"{report['minimum_sampled_sat_gap_m']:.3f} (SAT)",
                     '0', f"{report['solve_seconds_mean']:.3f}/{report['solve_seconds_max']:.3f}",
                     report['classification'], report['stop']])
    for report in comparison['counterfactuals']:
        if report['name'] == 'uncoordinated_conflict':
            continue
        rows.append([report['name'] + ' (physical envelope, scheduled controls)',
                     '1/1' if report['physical_envelope_pass'] else '0/1',
                     '/'.join(f'{value:.2f}' for value in report['completion_time_s']),
                     f"{report['minimum_sampled_padded_body_distance_m']:.3f}",
                     '/'.join(map(str, report['reverse_entries'])),
                     'N/A: no optimizer',
                     'normal waiting' if report['physical_envelope_pass'] else 'candidate rejected',
                     'complete' if report['physical_envelope_pass'] else 'static collision'])
    headings = ['Run', 'Success', 'A/B finish (s)', 'Min padded gap (m)',
                'A/B reverse entries', 'Solve mean/max (s)', 'Category', 'Stop']
    text = '| ' + ' | '.join(headings) + ' |\n'
    text += '| ' + ' | '.join(['---'] * len(headings)) + ' |\n'
    text += '\n'.join('| ' + ' | '.join(row) + ' |' for row in rows) + '\n'
    (out / 'comparison.md').write_text(text)


if __name__ == '__main__':
    main()
