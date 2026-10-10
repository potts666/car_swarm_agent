#!/usr/bin/env python3
"""Sampled witness validation using the new baseline's plant and constraints."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import mpc
import run

ROOT = Path(__file__).resolve().parent


def validate(config, controls, output):
    length = max(map(len, controls))
    padded = [np.vstack((control, np.zeros((length - len(control), 2)))) for control in controls]
    refs = [mpc.reference(state, control, config) for state, control in zip(config['starts'], padded)]
    states = np.array(config['starts'], dtype=float)
    violations = dict(pair=0., prediction_constraints=0., stop_equalities=0.)
    minimum_pair = float('inf')
    minimum_static = float('inf')
    arrived_at = [None, None]
    samples = []
    hold_history = []
    dt = config['time']['control_dt_s']
    steps = config['time']['validation_substeps']
    # Same residual evaluator as MPC, with witness-specific driving modes.
    # Candidate changes time coordination, not any physical/input constraint.
    for i in range(2):
        residual, equality = mpc.residuals(np.array(config['starts'][i]), padded[i], config,
                                         refs[i]['gear'], refs[1-i]['states'][1:])
        violations['prediction_constraints'] = max(violations['prediction_constraints'], max(0., -float(np.min(residual))))
        violations['stop_equalities'] = max(violations['stop_equalities'], float(np.max(abs(equality))) if len(equality) else 0.)
    for cycle in range(length):
        motions = []
        for i in range(2):
            motion, measured = run.executed_checks(states[i], padded[i][cycle], config, refs[i]['gear'][cycle])
            motions.append(motion)
            for key, value in measured.items():
                violations[key] = max(violations.get(key, 0.), value)
            minimum_static = min(minimum_static, float(np.min(mpc.static_gaps(motion[:, :2], motion[:, 2], np.array(config['obstacle_boxes'])))))
        gap = float(np.min(mpc.pair_gap(*motions)))
        violations['pair'] = max(violations['pair'], config['limits']['pair_sat_gap_m'] - gap)
        distances = mpc.body_distance(motions[0][:, :2], motions[0][:, 2], motions[1][:, :2], motions[1][:, 2])
        minimum_pair = min(minimum_pair, float(np.min(distances)))
        hold_history.append([all(run.arrived(sample, config['goals'][i], config) for sample in motion) for i, motion in enumerate(motions)])
        for i, motion in enumerate(motions):
            for j, state in enumerate(motion):
                samples.append([cycle * dt + j * dt / steps, i, *state])
            states[i] = mpc.rollout(states[i], padded[i][cycle:cycle+1], config)[-1]
            if run.arrived(states[i], config['goals'][i], config) and arrived_at[i] is None:
                arrived_at[i] = (cycle + 1) * dt
    final_ok = all(run.arrived(s, g, config) for s, g in zip(states, config['goals']))
    hold_count = round(config['acceptance']['hold_seconds'] / dt)
    hold_pass = len(hold_history) >= hold_count and bool(np.all(hold_history[-hold_count:]))
    report = dict(pass_all_declared_sampled_constraints=bool(max(violations.values()) <= config['mpc']['constraint_tolerance'] and final_ok and hold_pass),
                  final_hold_pass=hold_pass, hold_seconds=config['acceptance']['hold_seconds'],
                  input_limits=config['limits'], vehicle=config['vehicle'], initial_states=config['starts'], goals=config['goals'],
                  violations=violations, completion_times_s=arrived_at, total_duration_s=length * dt,
                  min_sampled_padded_body_distance_m=minimum_pair,
                  min_sampled_static_sat_gap_m=minimum_static,
                  final_errors=[run.error(s, g) for s, g in zip(states, config['goals'])],
                  continuous_time_certified=False, sampling_dt_s=dt/steps,
                  driving_mode_policy='Witness references encode B waiting before forward motion; baseline references are uncoordinated. Same physical/input/gear-switch constraints, different planned timing.',
                  validation_method='Per-control interval dense midpoint reintegration from actual coarse-model states; SAT gap and exact rectangle edge/vertex distances at samples only.')
    output.mkdir(parents=True, exist_ok=True)
    run.write(output / 'witness_validation.json', report)
    for i in range(2):
        run.write(output / f'witness_{i}_controls.json', padded[i].tolist())
        run.write(output / f'witness_{i}_gear.json', refs[i]['gear'].tolist())
    np.savetxt(output / 'witness_trajectories.csv', samples, delimiter=',', header='t,car,x,y,yaw,speed,steering', comments='')
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=ROOT / 'witness_results')
    args = parser.parse_args()
    config = json.loads((ROOT / 'config.json').read_text())
    a, b = [np.array(json.loads((ROOT / f'reference_{car}_controls.json').read_text())) for car in 'AB']
    # B stays at its original safe pose until A's full reference, including
    # standstill steering reset and hold, has completed.
    wait = np.vstack((np.zeros((len(a), 2)), b))
    report = validate(config, [a, wait], args.output)
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report['pass_all_declared_sampled_constraints'] else 1)


if __name__ == '__main__':
    main()
