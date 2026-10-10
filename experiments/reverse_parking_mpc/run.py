#!/usr/bin/env python3
"""Single-car gate first; frozen-config two-car evidence follows only on success."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import mpc

ROOT = Path(__file__).resolve().parent


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def error(state, goal):
    return dict(position_m=float(np.linalg.norm(state[:2] - goal[:2])),
                yaw_rad=float(abs(mpc.wrap(state[2] - goal[2]))), speed_m_s=float(abs(state[3])))


def arrived(state, goal, config):
    measured = error(state, goal)
    criteria = config['criteria']
    return (measured['position_m'] <= criteria['position_m'] and
            measured['yaw_rad'] <= criteria['yaw_rad'] and
            measured['speed_m_s'] <= criteria['speed_m_s'])


def executed_checks(state, control, config, gear):
    # Validate each actual control interval, not a separately accumulated replay.
    substeps = config['time']['validation_substeps']
    dense = mpc.rollout(state, np.array([control]), config, substeps)
    limits = config['limits']
    dt = config['time']['control_dt_s']
    lateral = dense[:, 3] ** 2 * np.tan(control[1]) / config['vehicle']['wheelbase_m']
    violations = dict(
        static=max(0., limits['static_sat_gap_m'] - float(np.min(mpc.static_gaps(dense[:, :2], dense[:, 2], np.array(config['obstacle_boxes']))))),
        speed=max(0., limits['speed_m_s'][0] - float(np.min(dense[:, 3])), float(np.max(dense[:, 3])) - limits['speed_m_s'][1]),
        acceleration=max(0., abs(float(control[0])) - limits['acceleration_m_s2']),
        steering=max(0., abs(float(control[1])) - limits['steering_rad']),
        steering_rate=max(0., abs(float(control[1]) - state[4]) / dt - limits['steering_rate_rad_s']),
        total_acceleration=max(0., float(np.max(np.hypot(control[0], lateral))) - limits['total_acceleration_m_s2']),
        gear=max(0., -float(np.min(dense[:, 3] * gear))) if gear else float(np.max(abs(dense[:, 3]))))
    return dense, violations


def run(config, controls, output, seed, two=False):
    count = 2 if two else 1
    states = np.array(config['starts'][:count], dtype=float)
    refs = [mpc.reference(state, control, config) for state, control in zip(states, controls)]
    n = config['mpc']['horizon_steps']
    dt = config['time']['control_dt_s']
    substeps = config['time']['model_substeps']
    predictions = [np.tile(state, (n * substeps + substeps + 1, 1)) for state in states]
    warm = [None] * count
    duration = config['acceptance']['two_car_max_seconds' if two else 'single_car_max_seconds']
    name = f'{"two" if two else "single"}_{seed}'
    actual = [[0., i, *state] for i, state in enumerate(states)]
    held = 0
    finish = [None] * count
    minimum_static = float('inf')
    minimum_pair = float('inf')
    maximum_violation = 0.
    solve_seconds = []
    history = [states.copy()]
    permission_history = []
    stopped = 'observation_limit'
    with (output / f'{name}.jsonl').open('w') as stream:
        for cycle in range(round(duration / dt)):
            frozen_predictions = [p.copy() for p in predictions]
            outputs = []
            for i in range(count):
                # Cycle zero predictions start at t=0; subsequent previous plans
                # start one control interval earlier. Tail covers next horizon.
                offset = 1 if cycle == 0 else substeps + 1
                peer = frozen_predictions[1 - i][offset:offset + n * substeps] if two else None
                u, report = mpc.solve(states[i], cycle, refs[i], config, warm[i], peer)
                solve_seconds.append(report['solve_seconds'])
                row = dict(cycle=cycle, time_s=cycle * dt, car=i, seed=seed, permission=True,
                           state=states[i].tolist(), goal_error=error(states[i], config['goals'][i]),
                           warm_input=None if warm[i] is None else warm[i].tolist(),
                           peer_input=None if peer is None else peer.tolist(),
                           reference_motion_allowed=report['gear'][0] != 0,
                           controls=u.tolist(), prediction=mpc.rollout(states[i], u, config).tolist(), solver=report)
                outputs.append((u, row))
            if not all(row['solver']['accepted'] for _, row in outputs):
                for _, row in outputs:
                    row['executed'] = False
                    stream.write(json.dumps(row) + '\n')
                stopped = 'solver_rejected_no_execution'
                break
            dense = []
            for i, (u, row) in enumerate(outputs):
                motion, violations = executed_checks(states[i], u[0], config, row['solver']['gear'][0])
                dense.append(motion)
                minimum_static = min(minimum_static, float(np.min(mpc.static_gaps(motion[:, :2], motion[:, 2], np.array(config['obstacle_boxes'])))))
                maximum_violation = max(maximum_violation, *violations.values())
                row['actual_violations'] = violations
            if two:
                gap = float(np.min(mpc.pair_gap(*dense)))
                distance = float(np.min(mpc.body_distance(dense[0][:, :2], dense[0][:, 2], dense[1][:, :2], dense[1][:, 2])))
                minimum_pair = min(minimum_pair, distance)
                maximum_violation = max(maximum_violation, config['limits']['pair_sat_gap_m'] - gap)
            for i, (u, row) in enumerate(outputs):
                next_motion = mpc.rollout(states[i], u[:1], config)
                row.update(executed=True, actual_next_state=next_motion[-1].tolist(),
                           displacement_m=float(np.linalg.norm(next_motion[-1, :2] - states[i, :2])),
                           min_sampled_pair_body_distance_m=distance if two else None)
                stream.write(json.dumps(row) + '\n')
                for j, s in enumerate(next_motion[1:], 1):
                    actual.append([cycle * dt + j * dt / substeps, i, *s])
                states[i] = next_motion[-1]
                warm[i] = np.vstack((u[1:], u[-1]))
                # Tail holds last speed/steering, with zero longitudinal input.
                predictions[i] = mpc.rollout(row['state'], np.vstack((u, [0., u[-1, 1]])), config)
                if arrived(states[i], config['goals'][i], config) and finish[i] is None:
                    finish[i] = (cycle + 1) * dt
            history.append(states.copy())
            permission_history.append([row['reference_motion_allowed'] for _, row in outputs])
            if maximum_violation > config['mpc']['constraint_tolerance']:
                stopped = 'actual_sampled_constraint_violation'
                break
            at_goal = all(arrived(s, g, config) for s, g in zip(states, config['goals']))
            full_interval_at_goal = at_goal and all(
                all(arrived(sample, config['goals'][i], config) for sample in motion)
                for i, motion in enumerate(dense))
            held = held + 1 if full_interval_at_goal else 0
            if held * dt >= config['acceptance']['hold_seconds']:
                stopped = 'goals_reached_and_held'
                break
            if cycle % 10 == 0:
                print(name, cycle, [error(s, g) for s, g in zip(states, config['goals'])], flush=True)
    success = stopped == 'goals_reached_and_held'
    window = round(config['criteria']['stagnation_window_s'] / dt)
    stalled = False
    if two:
        for j in range(window, len(history)):
            segment = np.array(history[j-window:j+1])
            extent = np.max(np.linalg.norm(segment[:, :, :2] - segment[0, :, :2], axis=2), axis=0)
            progress = [np.linalg.norm(segment[0, i, :2] - config['goals'][i][:2]) - np.linalg.norm(segment[-1, i, :2] - config['goals'][i][:2]) for i in range(count)]
            if (np.all(permission_history[j-window:j]) and np.all(extent < config['criteria']['max_extent_m']) and
                    np.all(np.array(progress) < config['criteria']['min_progress_m']) and
                    all(not arrived(s, g, config) for s, g in zip(segment[-1], config['goals']))):
                stalled = True
                break
    summary = dict(success=success, stop=stopped, seed=seed, cycles=len(history)-1,
                   arrival_times_s=finish, final_errors=[error(s, g) for s, g in zip(states, config['goals'])],
                   maximum_actual_sampled_violation=maximum_violation,
                   min_sampled_static_sat_gap_m=minimum_static if np.isfinite(minimum_static) else None,
                   min_sampled_pair_body_distance_m=minimum_pair if two and np.isfinite(minimum_pair) else None,
                   solve_seconds_mean=float(np.mean(solve_seconds)), solve_seconds_max=max(solve_seconds),
                   classification='experimental_sustained_stagnation' if stalled else 'normal_completion' if success else 'stagnation_not_reproduced',
                   continuous_time_certified=False, validation_dt_s=dt/config['time']['validation_substeps'])
    write(output / f'{name}_summary.json', summary)
    np.savetxt(output / f'{name}_actual.csv', actual, delimiter=',', header='t,car,x,y,yaw,speed,steering', comments='')
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=ROOT / 'development_results')
    parser.add_argument('--mode', choices=['single', 'two'], default='single')
    parser.add_argument('--seed', type=int, default=20261010)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    config = json.loads((ROOT / 'config.json').read_text())
    if args.mode == 'two':
        freeze = json.loads((ROOT / 'frozen_baseline.json').read_text())
        if not freeze['single_car_gate_passed']:
            parser.error('Single-car gate has not passed')
        for path, digest in freeze['core_sha256'].items():
            if hashlib.sha256((ROOT / path).read_bytes()).hexdigest() != digest:
                parser.error('Frozen baseline hash mismatch: ' + path)
    controls = [np.array(json.loads((ROOT / f'reference_{car}_controls.json').read_text())) for car in 'AB']
    write(args.output / 'manifest.json', dict(config=config,
          files_sha256={p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in ROOT.glob('*') if p.is_file()}))
    result = run(config, controls, args.output, args.seed, args.mode == 'two')
    print(json.dumps(result, indent=2), flush=True)
    raise SystemExit(0 if result['success'] else 1)


if __name__ == '__main__':
    main()
