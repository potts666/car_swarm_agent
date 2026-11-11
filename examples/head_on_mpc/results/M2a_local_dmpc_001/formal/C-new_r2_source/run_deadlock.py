#!/usr/bin/env python3
"""Reproduce a reverse-capable MPC standoff without priority or side bays."""
import argparse
from collections import deque
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from head_on_core import Settings, body_gap, inside_road, integrate, matrices, prediction, receive, solve, write_json

ROOT = Path(__file__).resolve().parents[1]


def retreat_probe(states, cfg, scene, now):
    """Counterfactual only: can each car retreat while its peer holds still?"""
    evidence = {}
    for car, other in (('A', 'B'), ('B', 'A')):
        peer = np.tile([*states[other], 0.], (cfg.horizon*cfg.substeps, 1))
        side = 'front' if states[other][0] >= states[car][0] else 'rear'
        u, report = solve(states[car], 0., np.zeros(cfg.horizon), now,
                          scene['retreat_probe_targets_x'][car], cfg=cfg,
                          allow_reverse=True, **{side: peer})
        packet = prediction(states[car], u, [0., 0., now, 1.], now, cfg)
        motion = np.asarray(packet['states'][:-1])
        ticks = np.arange(1, cfg.horizon*cfg.substeps+1)*cfg.dt/cfg.substeps
        # Check all fine prediction samples, not only control knots.
        pmat, vmat = matrices(ticks, cfg)
        sampled = np.column_stack((states[car][0]+ticks*states[car][1]+pmat@u,
                                   states[car][1]+vmat@u, np.zeros(len(ticks))))
        sign = -1 if car == 'A' else 1
        possible = (report['accepted'] and sign*u[0] > .01 and
                    sign*(motion[-1, 0]-states[car][0]) > .1 and
                    bool(np.all(inside_road(sampled, scene['road'], cfg))) and
                    float(np.min(body_gap(sampled, peer, cfg))) >= scene['sampled_gap_min_m'])
        evidence[car] = {'feasible': bool(possible), 'target_x': scene['retreat_probe_targets_x'][car],
                         'first_acceleration': float(u[0]),
                         'predicted_displacement_m': float(motion[-1, 0]-states[car][0]),
                         'accepted': report['accepted']}
    return evidence


def run_deadlock(output, scene):
    cfg = Settings(**scene['settings'])
    if (cfg.dt <= 0 or cfg.horizon < 1 or cfg.substeps < 1 or
            scene['deadlock_window_s'] <= 0 or scene['duration_s'] < scene['deadlock_window_s']+cfg.dt):
        raise ValueError('Require a positive control period/window and a recording longer than the window')
    if scene['road'].get('bays'):
        raise ValueError('The deadlock reproduction must have no side bays')
    states = {car: np.array(scene['initial_states_x_v'][car], dtype=float) for car in ('A', 'B')}
    if not states['A'][0] < states['B'][0]:
        raise ValueError('A must start to the left of B')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    config = {**scene, 'case': 'head_on_deadlock', 'origin': 'vehicle centre; A yaw=0, B yaw=pi',
              'model': 'longitudinal double integrator; y=0, both velocity signs allowed',
              'supervisor': 'none; both retain their forward destinations; no hold targets',
              'initial_condition': 'already stopped at an encounter inside the channel; no approach simulation',
              'communication': 'synchronous previous-cycle prediction packets; no negotiation',
              'capability_probe': 'counterfactual retreat target, never executed by the actual policy'}
    write_json(output/'config.json', config)
    warm = {car: np.zeros(cfg.horizon) for car in states}
    previous = dict.fromkeys(states, 0.)
    lanes = {car: [0., 0., cfg.epoch, 1.] for car in states}
    packets = {car: prediction(s, warm[car], lanes[car], cfg.epoch, cfg) for car, s in states.items()}
    initial_probe = retreat_probe(states, cfg, scene, cfg.epoch)
    events, records, actual, publications = [], [], [], []
    history = deque()
    minimum_gap, failure, detected = float('inf'), None, None
    phase = 'mutual_wait'

    def record(now, motions, accelerations):
        nonlocal minimum_gap, failure
        minimum_gap = min(minimum_gap, float(np.min(body_gap(motions['A'], motions['B'], cfg))))
        if minimum_gap < scene['sampled_gap_min_m']-cfg.tolerance:
            failure = 'sampled executed separation violation'
        for car, motion in motions.items():
            if (not np.all(np.isfinite(motion)) or not np.all(inside_road(motion, scene['road'], cfg)) or
                    np.max(np.abs(motion[:, 1])) > cfg.max_speed+cfg.tolerance or
                    abs(accelerations[car]) > cfg.max_acceleration+cfg.tolerance):
                failure = f'{car}: executed model/road/limit violation'
            for j, s in enumerate(motion):
                actual.append([round(now+j*cfg.dt/cfg.substeps, 10), car, *s,
                               accelerations[car], phase, s[1] if car == 'A' else -s[1]])

    record(cfg.epoch, {c: np.array([[*s, 0.]]) for c, s in states.items()}, previous)
    history.append((cfg.epoch, {c: s.copy() for c, s in states.items()}))
    for cycle in range(int(scene['duration_s']/cfg.dt)):
        if failure:
            break
        now = cfg.epoch+cycle*cfg.dt
        ticks = now+np.arange(1, cfg.horizon*cfg.substeps+1)*cfg.dt/cfg.substeps
        peers = {c: receive(packet, ticks) for c, packet in packets.items()}
        controls, reports = {}, {}
        for car, other in (('A', 'B'), ('B', 'A')):
            side = 'front' if states[other][0] >= states[car][0] else 'rear'
            u, report = solve(states[car], previous[car], warm[car], now, scene['goals_x'][car],
                              cfg=cfg, allow_reverse=True, **{side: peers[other]})
            report.update({'cycle': cycle, 'car': car, 'phase': phase, 'controls': u.tolist(),
                           'allow_reverse': True, 'peer_stamps': {other: packets[other]['stamp']}})
            records.append(report)
            controls[car], reports[car] = u, report
        if not all(r['accepted'] for r in reports.values()):
            failure = 'rejected optimization; no controls executed'
            break
        outgoing = {c: prediction(states[c], controls[c], lanes[c], now, cfg) for c in states}
        publications.extend({'cycle': cycle, 'car': c, **p} for c, p in outgoing.items())
        relative = np.arange(cfg.substeps+1)*cfg.dt/cfg.substeps
        motions = {c: np.column_stack((integrate(states[c], controls[c][0], relative),
                                       np.zeros(len(relative)))) for c in states}
        record(now, motions, {c: float(u[0]) for c, u in controls.items()})
        for c in states:
            states[c] = motions[c][-1, :2]
            previous[c] = float(controls[c][0])
            warm[c] = np.r_[controls[c][1:], controls[c][-1]]
        packets = outgoing
        stamp = now+cfg.dt
        history.append((stamp, {c: s.copy() for c, s in states.items()}))
        while len(history) > 1 and history[1][0] <= stamp-scene['deadlock_window_s']+1e-8:
            history.popleft()
        stalled = stamp-history[0][0] >= scene['deadlock_window_s']-1e-8
        for c in states:
            samples = np.array([entry[1][c] for entry in history])
            stalled &= (np.ptp(samples[:, 0]) <= scene['deadlock_position_tolerance_m'] and
                        np.max(np.abs(samples[:, 1])) <= scene['deadlock_speed_tolerance_m_s'] and
                        abs(states[c][0]-scene['goals_x'][c]) > scene['arrival_position_tolerance_m'])
        if stalled and detected is None and not failure:
            probes = retreat_probe(states, cfg, scene, stamp)
            if all(p['feasible'] for p in probes.values()):
                detected = round(stamp-cfg.epoch, 10)
                events.append({'time_s': detected, 'from': phase, 'to': 'deadlock_detected',
                               'detail': 'Both unfinished and stationary over the window; each has a feasible retreat',
                               'wait_for': {'A': 'B', 'B': 'A'}, 'retreat_probes': probes})
                phase = 'deadlock_detected'
    status = 'failed' if failure else 'deadlock' if detected is not None else 'inconclusive'
    summary = {'case': 'head_on_deadlock', 'status': status, 'expected_status': 'deadlock',
               'expectation_met': status == 'deadlock', 'failure': failure, 'final_phase': phase,
               'simulation_seconds': round(actual[-1][0]-cfg.epoch, 10),
               'deadlock_detected_at_s': detected, 'initial_retreat_probes': initial_probe,
               'minimum_sampled_sat_gap_m': minimum_gap,
               'final_states_x_v': {c: s.tolist() for c, s in states.items()},
               'solve_count': len(records), 'rejected_solves': sum(not r['accepted'] for r in records),
               'continuous_safety_certified': False, 'autonomous_strategy_selection': False,
               'supervisor_is_centralized': False,
               'interpretation': 'Operational mutual-wait criterion, not a general deadlock-freedom theorem'}
    with (output/'actual.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['absolute_time_s', 'car', 'x', 'vx', 'y', 'acceleration', 'phase', 'body_signed_speed'])
        writer.writerows(actual)
    write_json(output/'solves.json', records)
    write_json(output/'events.json', events)
    write_json(output/'summary.json', summary)
    with (output/'predictions.jsonl').open('w') as stream:
        for publication in publications:
            stream.write(json.dumps(publication, allow_nan=False)+'\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config/deadlock.json')
    parser.add_argument('--output', type=Path, default=ROOT/'results/deadlock_001')
    parser.add_argument('--video', action='store_true', help='also export H.264 video')
    args = parser.parse_args()
    output = args.output.resolve()
    summary = run_deadlock(output/'head_on_deadlock', json.loads(args.config.read_text()))
    from render import plot, export_video
    from verify_bundle import audit_case
    directory = output/'head_on_deadlock'
    plot(directory)
    if args.video:
        export_video(directory)
    audit = audit_case(directory, video=args.video)
    write_json(output/'validation.json', audit)
    sources = sorted({*ROOT.glob('scripts/*.py'), *ROOT.glob('tests/*.py'),
                      ROOT/'launch/replay.launch.py', ROOT/'config/deadlock.json',
                      ROOT/'rviz/replay.rviz', ROOT/'README.md', args.config.resolve()})
    write_json(output/'provenance.json', {'source_sha256': {
        str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p):
        hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        'config': json.loads(args.config.read_text())})
    (output/'report.md').write_text(
        '# 双向行驶能力下的互等死锁复现\n\n'
        f'结果：`{summary["status"]}`；{summary["deadlock_detected_at_s"]} 秒触发检测，'
        f'记录时长 {summary["simulation_seconds"]} 秒，最小采样车身间隙 '
        f'{summary["minimum_sampled_sat_gap_m"]} m。\n\n'
        '无沿途避让区、无优先权。两车从通道内相遇后停车的状态开始，保持各自前方目标，'
        'MPC 允许正负速度。对方预测停车时，本车无法继续前进，后退会增大自身目标误差，'
        '因此持续选择停车。每周期仍实际求解 MPC，没有将控制强制覆盖为零。\n\n'
        '双方反事实倒车求解均可行，但实际策略不执行这些探测结果。持续无进展、未到目标、'
        '低速以及可行倒车共同构成检测证据，见 `head_on_deadlock/events.json`。'
        '初始接近过程、入口外等待区及占用权调度尚未实现。\n\n'
        '数据校验见 `validation.json`，原始轨迹见 `head_on_deadlock/actual.csv`，'
        '图见 `head_on_deadlock/overview.png`。ROS 回放验证单独运行。\n')
    print(json.dumps(summary, indent=2))
    return 0 if summary['expectation_met'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
