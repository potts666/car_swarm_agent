#!/usr/bin/env python3
"""Validate recorded real ROS runs against the same offline baseline."""
import argparse
import json
from pathlib import Path
import numpy as np


def check(root):
    summaries = {}
    for mode in ('in_process', 'ros_nominal', 'ros_missing', 'ros_drop', 'ros_stale'):
        directory = root / mode
        summary = json.loads((directory / 'summary.json').read_text())
        records = json.loads((directory / 'solves.json').read_text())
        assert summary['success'], (mode, summary['failure'], summary['arrived'])
        assert summary['max_realized_sampled_constraint_violation'] <= 1e-5, mode
        distances = np.genfromtxt(directory / 'body_distances.csv', names=True, delimiter=',')
        assert abs(np.min(distances['body_distance_m']) - summary['min_inter_sample_body_distance_m']) < 1e-10
        assert np.min(distances['body_distance_m']) > 0., mode
        below = distances['body_distance_m'] < .08 - 1e-5
        assert summary['inter_sample_clearance_below_margin_samples'] == int(np.sum(below))
        assert summary['inter_sample_clearance_target_met'] == (not np.any(below))
        if mode != 'in_process':
            assert len({r['worker_pid'] for r in records}) == 2
            assert all(r['peer_vehicle_id'] == f"car_{3-r['car']}" for r in records)
            for r in records:
                if not r['conservative']:
                    assert r['peer_sequence'] == r['cycle'] - 1, r
                    assert r['point_count'] == 53, r
                    assert abs(r['message_age_s'] - (0. if r['cycle'] == 0 else .4)) < 1e-8
            assert summary['individual_outputs_exceeding_period'] == sum(
                r['receive_to_control_seconds'] is not None and r['receive_to_control_seconds'] > .4 for r in records)
            assert summary['ros_rounds_exceeding_control_period'] == sum(
                records[j]['round_request_to_outputs_seconds'] > .4 for j in range(0, len(records), 2))
            for car in (1, 2):
                events = [json.loads(line) for line in (directory / f'car_{car}_exchange.jsonl').read_text().splitlines()]
                received = [e for e in events if e['event'] == 'receive']
                assert received and all(e['vehicle_id'] == f'car_{3-car}' for e in received)
                for e in received:
                    assert abs(e['message_age_s'] - (e['receiver_absolute_s'] - e['stamp_absolute_s'])) < 1e-8
            conservative = [r for r in records if r['conservative']]
            if mode == 'ros_nominal':
                assert not conservative
            else:
                target_cycle = 0 if mode == 'ros_missing' else 3
                target = [r for r in conservative if r['cycle'] == target_cycle and r['car'] == 1]
                assert target, (mode, conservative)
                assert target[0]['peer_status'] == ('missing' if mode == 'ros_missing' else 'expired')
                assert target[0]['executed_first_control'][0] <= 0
        summaries[mode] = summary
    baseline = np.genfromtxt(root / 'in_process' / 'actual.csv', names=True, delimiter=',')
    nominal = np.genfromtxt(root / 'ros_nominal' / 'actual.csv', names=True, delimiter=',')
    for key in baseline.dtype.names:
        np.testing.assert_allclose(nominal[key], baseline[key], atol=5e-4, rtol=0)
    assert summaries['ros_nominal']['arrival_time_s'] == summaries['in_process']['arrival_time_s']
    # Reference exports must also be byte-identical, including static scene.
    for name in ('before.csv', 'obstacles.csv'):
        expected = (root / 'in_process' / 'seed' / name).read_bytes()
        for mode in summaries:
            assert (root / mode / 'seed' / name).read_bytes() == expected
    print('Validated identical scene, separate worker PIDs, previous-round ROS predictions, timings, distances and fault braking.')
    return summaries


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path, nargs='?', default=Path('log/ros_comparison'))
    args = parser.parse_args()
    check(args.root)
