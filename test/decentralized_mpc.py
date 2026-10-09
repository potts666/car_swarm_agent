#!/usr/bin/env python3
"""Project extension: synchronous, previous-prediction decentralized MPC.

Offline numerical closed loop; no ROS/vehicle command publication. Absolute
simulation timestamps share an epoch of 100 s. Neither cooperative DMPC nor a
continuous-time safety proof. Native Hybrid A*/S-T exports supply references.
"""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import time

import numpy as np
from scipy.optimize import minimize
from optimize_spatial_temporal import load, rectangle_gap, static_gaps, body_distance

EPOCH = 100.0
DT = 0.4
SUBSTEPS = 4
N = 12
MARGIN = 0.08


def rollout(state, controls, tail=False):
    """Midpoint bicycle model, controls [acceleration, front-wheel angle].

    The last column stores applied steering. No clipping hides violations.
    A published constant-speed/steering tail covers the next solve's horizon.
    """
    state = np.array(state, dtype=float).copy()
    states = [state.copy()]
    commands = np.asarray(controls).reshape(-1, 2)
    if tail:
        commands = np.vstack((commands, [0.0, commands[-1, 1]]))
    h = DT / SUBSTEPS
    for acc, steer in commands:
        for _ in range(SUBSTEPS):
            v_mid = state[3] + acc * h / 2
            omega = v_mid * np.tan(steer) / 2.7
            yaw_mid = state[2] + omega * h / 2
            state[:2] += h * v_mid * np.array([np.cos(yaw_mid), np.sin(yaw_mid)])
            state[2] += h * omega
            state[3] += acc * h
            state[4] = steer
            states.append(state.copy())
    return np.asarray(states)


def packet(now, states):
    return {"stamp": float(now), "times": now + np.arange(len(states)) * DT / SUBSTEPS,
            "states": np.array(states, copy=True)}


def receive(prediction, times):
    if (not np.all(np.isfinite(prediction['states'])) or
            not np.all(np.diff(prediction['times']) > 0) or
            abs(prediction['times'][0] - prediction['stamp']) > 1e-9 or
            times[0] < prediction['times'][0] - 1e-8 or
            times[-1] > prediction['times'][-1] + 1e-8):
        raise ValueError('Invalid or uncovered absolute-time neighbor prediction')
    data = prediction['states']
    return np.column_stack([np.interp(times, prediction['times'],
                                     np.unwrap(data[:, i]) if i == 2 else data[:, i])
                            for i in range(5)])


def reference(data, absolute_times):
    t = absolute_times - EPOCH
    return np.column_stack([np.interp(t, data['t'], data[k]) for k in ('x', 'y', 'yaw', 'speed')])


def neighbor_gap(states, peer):
    center = peer[:, :2] + 1.25 * np.column_stack((np.cos(peer[:, 2]), np.sin(peer[:, 2])))
    return rectangle_gap(states[:, :2], states[:, 2], center, peer[:, 2],
                         np.tile([2.35, 1.0], (len(states), 1)))


def solve(state, now, prediction, ref, obstacles, warm):
    # Current state is measured; optimize future nodes including substeps.
    times = now + np.arange(1, N * SUBSTEPS + 1) * DT / SUBSTEPS
    peer = receive(prediction, times)
    target = reference(ref, times)

    def objective(z):
        u = z.reshape(N, 2)
        future = rollout(state, u)[1:]
        heading = np.arctan2(np.sin(future[:, 2] - target[:, 2]),
                             np.cos(future[:, 2] - target[:, 2]))
        error = future[:, :2] - target[:, :2]
        change = np.diff(np.vstack(([0, state[4]], u)), axis=0)
        return float(3 * np.sum(error ** 2) + 2 * np.sum(heading ** 2) +
                     np.sum((future[:, 3] - target[:, 3]) ** 2) +
                     0.05 * np.sum(u ** 2) + 0.15 * np.sum(change ** 2) +
                     8 * np.sum(error[-1] ** 2))

    def constraints(z):
        u = z.reshape(N, 2)
        future = rollout(state, u)[1:]
        steer_change = np.diff(np.r_[state[4], u[:, 1]])
        lateral = future[:, 3] ** 2 * np.tan(future[:, 4]) / 2.7
        return np.r_[future[:, 3], 1 - future[:, 3],
                     DT - np.abs(steer_change),
                     2 - np.hypot(np.repeat(u[:, 0], SUBSTEPS), lateral),
                     neighbor_gap(future, peer) - MARGIN,
                     static_gaps(future[:, :2], future[:, 2], obstacles) - 0.03]

    start = time.perf_counter()
    attempts = []
    initial = warm.copy()
    for attempt in range(2):
        result = minimize(objective, initial.ravel(), method='SLSQP', jac='3-point',
                          bounds=[(-1, 1), (-0.5, 0.5)] * N,
                          constraints=[{'type': 'ineq', 'fun': constraints}],
                          options={'maxiter': 80, 'ftol': 1e-5})
        violation = max(0.0, -float(np.min(constraints(result.x))))
        accepted = bool(result.success and np.all(np.isfinite(result.x)) and violation <= 1e-5)
        attempts.append({'initialization': 'shifted_plan' if attempt == 0 else 'braking',
                         'success': bool(result.success), 'accepted': accepted,
                         'message': str(result.message), 'violation': violation})
        if accepted:
            break
        # A failed nonconvex solve gets one explicit physically stopped initial
        # guess. This is only an optimization seed, never a fallback command.
        initial = np.zeros((N, 2))
        initial[:, 1] = state[4]
        speed = max(0., state[3])
        for j in range(N):
            initial[j, 0] = -min(1., speed / DT)
            speed += initial[j, 0] * DT
    seconds = time.perf_counter() - start
    return result.x.reshape(N, 2), {'solver_success': bool(result.success),
        'accepted': accepted, 'message': str(result.message), 'iterations': int(result.nit),
        'solve_seconds': seconds, 'prediction_constraint_violation': violation,
        'peer_stamp_absolute_s': prediction['stamp'], 'attempts': attempts}



def run(directory, binary, max_seconds=60):
    directory.mkdir(parents=True, exist_ok=True)
    seed = directory / 'seed'
    subprocess.run([str(binary.resolve()), '--export', str(seed.resolve())], check=True)
    before = load(seed / 'before.csv')
    # Both cars solve controls. Car 1's straight spatial route is the original
    # scene; car 2 uses the native Hybrid A*/S-T reference including its wait.
    # Smooth endpoint deceleration; do not keep demanding 1 m/s after arrival.
    ts = np.arange(0, 60.1, 0.1)
    straight = np.zeros(len(ts), dtype=[(k, float) for k in ('t', 'x', 'y', 'yaw', 'speed')])
    straight['t'] = ts
    straight['x'] = np.where(ts <= 19.5, -10 + ts,
        np.where(ts < 20.5, 9.5 + (ts - 19.5) - 0.5 * (ts - 19.5) ** 2, 10))
    straight['speed'] = np.where(ts <= 19.5, 1, np.maximum(0, 20.5 - ts))
    refs = [straight, before]
    boxes = load(seed / 'obstacles.csv')
    # Same occupied cells, merged into horizontal runs for faster SAT checks.
    runs = []
    for y in np.unique(boxes['y']):
        row = np.sort(boxes[boxes['y'] == y], order='x')
        x, end = row[0]['x'], row[0]['x'] + row[0]['width']
        for b in row[1:]:
            if abs(b['x'] - end) < 1e-8:
                end += b['width']
            else:
                runs.append([x, y, end - x, .5]); x, end = b['x'], b['x'] + b['width']
        runs.append([x, y, end - x, .5])
    obstacles = np.array(runs)
    states = [np.array([-10, 0, 0, 1, 0.]), np.array([0, -8, np.pi / 2, 1, 0.])]
    goals = [np.array([10, 0]), np.array([0, 8])]
    warm = [np.zeros((N, 2)), np.zeros((N, 2))]
    predictions = [packet(EPOCH, rollout(s, u, tail=True)) for s, u in zip(states, warm)]
    records, actual, publications = [], [], []
    min_distance, actual_violation, failure = float('inf'), 0., None
    reached_at = [None, None]
    maxima = {k: 0.0 for k in ('neighbor_gap_m', 'static_gap_m', 'speed_m_s',
                              'steering_rad', 'acceleration_m_s2',
                              'steering_rate_rad_s', 'total_acceleration_m_s2')}
    for cycle in range(int(np.ceil(max_seconds / DT))):
        now = EPOCH + cycle * DT
        # Freeze both inboxes before either solve: no Gauss-Seidel coordination.
        controls, reports = [], []
        for i in range(2):
            u, report = solve(states[i], now, predictions[1-i], refs[i], obstacles, warm[i])
            controls.append(u); reports.append(report)
            records.append({'cycle': cycle, 'car': i + 1, 'absolute_time_s': now, **report})
        if not all(r['accepted'] for r in reports):
            failure = f'cycle {cycle}: rejected optimization; no controls executed'
            break
        new_predictions = [packet(now, rollout(s, u, tail=True)) for s, u in zip(states, controls)]
        for i, pred in enumerate(new_predictions):
            publications.append({'cycle': cycle, 'car': i + 1, 'stamp_absolute_s': pred['stamp'],
                                 'times_absolute_s': pred['times'].tolist(), 'states': pred['states'].tolist()})
        # Execute only the first control of each accepted plan, simultaneously.
        executed = [rollout(s, u[:1]) for s, u in zip(states, controls)]
        a, b = executed
        distances = body_distance(a[:, :2], a[:, 2], b[:, :2], b[:, 2])
        min_distance = min(min_distance, float(np.min(distances)))
        measured = dict.fromkeys(maxima, 0.0)
        measured['neighbor_gap_m'] = max(0., MARGIN - float(np.min(neighbor_gap(a, b))))
        for i, motion in enumerate(executed):
            acc, steer = controls[i][0]
            lateral = motion[:, 3] ** 2 * np.tan(steer) / 2.7
            checks = {
                'static_gap_m': max(0., .03 - float(np.min(static_gaps(motion[:, :2], motion[:, 2], obstacles)))),
                'speed_m_s': max(0., -float(np.min(motion[:, 3])), float(np.max(motion[:, 3])) - 1),
                'steering_rad': max(0., abs(steer) - .5),
                'acceleration_m_s2': max(0., abs(acc) - 1),
                'steering_rate_rad_s': max(0., abs(steer - states[i][4]) / DT - 1),
                'total_acceleration_m_s2': max(0., float(np.max(np.hypot(acc, lateral))) - 2)}
            for key, value in checks.items():
                measured[key] = max(measured[key], value)
            records[-2 + i]['executed_first_control'] = controls[i][0].tolist()
            for j, s in enumerate(motion[1:], 1):
                actual.append([now + j * DT / SUBSTEPS, i + 1, *s, *controls[i][0]])
            states[i] = motion[-1]
            warm[i] = np.vstack((controls[i][1:], controls[i][-1]))
            if np.linalg.norm(states[i][:2] - goals[i]) < .3 and abs(states[i][3]) < .05:
                if reached_at[i] is None:
                    reached_at[i] = now + DT - EPOCH
        for key, value in measured.items():
            maxima[key] = max(maxima[key], value)
        for r in records[-2:]:
            r['realized_pair_violation_by_constraint'] = measured.copy()
        actual_violation = max(actual_violation, *measured.values())
        predictions = new_predictions
        if actual_violation > 1e-5:
            failure = f'cycle {cycle}: realized sampled constraint violation'
            break
        if all(np.linalg.norm(s[:2] - g) < .3 and abs(s[3]) < .05 for s, g in zip(states, goals)):
            break
        if cycle % 10 == 0:
            print(f'cycle={cycle} t={now-EPOCH:.1f}s min_body_distance={min_distance:.3f}m', flush=True)
    with (directory / 'actual.csv').open('w') as out:
        writer = csv.writer(out)
        writer.writerow(['absolute_time_s', 'car', 'x', 'y', 'yaw', 'speed', 'steering', 'acceleration', 'command_steering'])
        writer.writerows(actual)
    (directory / 'solves.json').write_text(json.dumps(records, indent=2) + '\n')
    with (directory / 'predictions.jsonl').open('w') as out:
        for pub in publications:
            out.write(json.dumps(pub) + '\n')
    elapsed = [r['solve_seconds'] for r in records]
    arrived = [bool(np.linalg.norm(s[:2] - g) < .3 and abs(s[3]) < .05) for s, g in zip(states, goals)]
    report = {'method': 'previous-prediction decentralized MPC baseline',
        'cooperative_dmpc': False, 'transport': 'synchronous in-process prediction packets',
        'model': 'midpoint kinematic bicycle, substep 0.1 s; independent of native C++ tracking model',
        'absolute_epoch_s': EPOCH, 'control_dt_s': DT, 'horizon_s': N * DT,
        'arrival_position_tolerance_m': .3, 'arrival_speed_tolerance_m_s': .05,
        'arrived': arrived, 'arrival_time_s': reached_at, 'final_states': [s.tolist() for s in states],
        'min_sampled_body_distance_m': None if not np.isfinite(min_distance) else min_distance,
        'max_realized_sampled_constraint_violation': actual_violation,
        'realized_violation_by_constraint': maxima,
        'violation_tolerance': 1e-5, 'body_padding_per_car_m': .1,
        'required_neighbor_sat_gap_m': MARGIN, 'required_static_sat_gap_m': .03,
        'max_prediction_constraint_violation': max((r['prediction_constraint_violation'] for r in records), default=0),
        'rejected_solves': sum(not r['accepted'] for r in records), 'solve_count': len(records),
        'braking_initialization_retries': sum(len(r['attempts']) > 1 for r in records),
        'solve_seconds_mean': float(np.mean(elapsed)) if elapsed else None,
        'solve_seconds_p95': float(np.percentile(elapsed, 95)) if elapsed else None,
        'solve_seconds_max': max(elapsed, default=0),
        'cycles_exceeding_control_period': sum(sum(r['solve_seconds'] for r in records if r['cycle'] == c) > DT
                                               for c in set(r['cycle'] for r in records)),
        'failure': failure, 'continuous_safety_certified': False,
        'success': all(arrived) and failure is None}
    (directory / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2), flush=True)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, default=Path('build/car_swarm_agent/compare_spatial_temporal'))
    parser.add_argument('--output', type=Path, default=Path('log/decentralized_mpc'))
    parser.add_argument('--max-seconds', type=float, default=60)
    args = parser.parse_args()
    if not np.isfinite(args.max_seconds) or args.max_seconds <= 0:
        parser.error('--max-seconds must be finite and positive')
    result = run(args.output, args.binary, args.max_seconds)
    raise SystemExit(0 if result['success'] else 1)
