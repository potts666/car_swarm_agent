#!/usr/bin/env python3
"""Paired head-on policy ablation with identical physical and solver budgets."""
import argparse
import csv
import hashlib
import itertools
import json
from pathlib import Path
import time

import numpy as np

from head_on_core import (Settings, body_gap, inside_road, integrate, lane_y,
                          matrices, prediction, receive, solve, write_json)

ROOT = Path(__file__).resolve().parents[1]
POLICIES = ('dmpc', 'fixed_A', 'distance', 'checked')


def make_scene(base, encounter, speed, length, braking):
    """Channel endpoints have external parking bays; there are no interior bays."""
    scene = json.loads(json.dumps(base))
    cfg = Settings(**scene['settings'])
    scene['settings']['max_acceleration'] = braking
    left, right = -length/2, length/2
    # Bay centre is one body length outside each channel endpoint.
    bays = {'A': left-cfg.length, 'B': right+cfg.length}
    extent = length/2+4*cfg.length
    scene.update(channel={'xmin': left, 'xmax': right}, retreat_x=bays,
                 road={'xmin': -extent, 'xmax': extent, 'ymin': -1.4, 'ymax': 1.4,
                       'bays': [{'xmin': x-cfg.length/2-.2, 'xmax': x+cfg.length/2+.2,
                                 'ymax': scene['bay_y']+cfg.width/2+.2} for x in bays.values()]},
                 goals_x={'A': right+2.5*cfg.length, 'B': left-2.5*cfg.length})
    # Extra gap covers symmetric stopping distances at the swept initial speed.
    separation = cfg.length+cfg.margin+speed*speed/braking
    scene['initial_states_x_v'] = {'A': [encounter-separation/2, speed],
                                      'B': [encounter+separation/2, -speed]}
    if any(abs(s[0])+cfg.length/2 > length/2 for s in scene['initial_states_x_v'].values()):
        raise ValueError('Initial bodies must fit inside the channel')
    scene['factors'] = dict(encounter_x_m=encounter, initial_speed_m_s=speed,
                           channel_length_m=length, braking_m_s2=braking)
    return scene


def choose_role(states, scene, cfg, metric='distance', eligible=None):
    eligible = ('A', 'B') if eligible is None else eligible
    scores = {}
    for c in eligible:
        distance = abs(states[c][0]-scene['retreat_x'][c])
        # Transparent heuristic, not a trajectory-derived arrival-time guarantee.
        scores[c] = (distance if metric == 'distance' else
                     abs(states[c][1])/cfg.max_acceleration + distance/cfg.max_speed)
    return min(scores, key=lambda c: (scores[c], c)) if scores else None, scores


def recovery_feasible(car, scene, cfg):
    """Geometric recovery certificate in this prescribed-occupancy model only."""
    bay = scene['retreat_x'][car]
    other = 'B' if car == 'A' else 'A'
    side = -1 if car == 'A' else 1
    parked = np.array([[bay, 0., scene['bay_y']]])
    return bool(np.all(inside_road(parked, scene['road'], cfg)) and
                scene['bay_y'] >= cfg.width+scene['sampled_gap_min_m'] and
                side*(scene['goals_x'][other]-bay) >= cfg.length+cfg.margin+.3)


def switching_allowed(now, last, old_score, new_score, scene):
    return (now-last >= scene['minimum_role_dwell_s'] and
            old_score-new_score >= scene['switch_improvement'])


def run_trial(output, scene, policy):
    if policy not in POLICIES:
        raise ValueError(policy)
    cfg = Settings(**scene['settings'])
    if cfg.dt <= 0 or cfg.horizon < 1 or cfg.substeps < 1 or cfg.max_acceleration <= 0:
        raise ValueError('Invalid model settings')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    write_json(output/'config.json', {**scene, 'policy': policy,
               'budget': '2 retreat probes + 2 executed MPC solves per cycle for every policy',
               'model': 'double integrator; prescribed lateral occupancy at external bays'})
    states = {c: np.array(s, float) for c, s in scene['initial_states_x_v'].items()}
    warm = {c: np.zeros(cfg.horizon) for c in states}
    previous = dict.fromkeys(states, 0.)
    lanes = {c: [0., 0., cfg.epoch, cfg.lateral_seconds] for c in states}
    packets = {c: prediction(s, warm[c], lanes[c], cfg.epoch, cfg) for c, s in states.items()}
    records, actual, events, publications = [], [], [], []
    role, phase, last_switch, switches = None, 'unassigned', -1e9, 0
    recovery, complete, failure, stall_at = None, None, None, None
    history = []
    reverse = dict.fromkeys(states, 0.)
    min_gap, elapsed = float('inf'), 0.
    clearance_at = None

    def stopped(c, target):
        return (abs(states[c][0]-target) <= scene['arrival_position_tolerance_m'] and
                abs(states[c][1]) <= scene['arrival_speed_tolerance_m_s'])

    def log_motion(now, motions, controls):
        nonlocal min_gap, failure
        min_gap = min(min_gap, float(np.min(body_gap(motions['A'], motions['B'], cfg))))
        if min_gap < scene['sampled_gap_min_m']-cfg.tolerance:
            failure = 'executed_separation_violation'
        for c, motion in motions.items():
            if (not np.all(np.isfinite(motion)) or not np.all(inside_road(motion, scene['road'], cfg)) or
                    np.max(np.abs(motion[:, 1])) > cfg.max_speed+cfg.tolerance or
                    abs(controls[c]) > cfg.max_acceleration+cfg.tolerance):
                failure = 'executed_model_road_limit_violation'
            sign = 1 if c == 'A' else -1
            reverse[c] += float(np.sum(np.maximum(0., -sign*np.diff(motion[:, 0]))))
            for j, s in enumerate(motion):
                actual.append([round(now+j*cfg.dt/cfg.substeps, 10), c, *s, controls[c], phase, sign*s[1]])

    log_motion(cfg.epoch, {c: np.array([[*s, 0.]]) for c, s in states.items()}, previous)
    for cycle in range(int(scene['duration_s']/cfg.dt)):
        if failure:
            break
        started = time.perf_counter()
        now = cfg.epoch+cycle*cfg.dt
        ticks = now+np.arange(1, cfg.horizon*cfg.substeps+1)*cfg.dt/cfg.substeps
        peers = {c: receive(packets[c], ticks) for c in states}
        probes = {}
        # Same extra optimization budget, even for policies that ignore probes.
        for c, other in (('A', 'B'), ('B', 'A')):
            peer = np.tile([*states[other], lane_y(lanes[other], [now])[0]], (len(ticks), 1))
            side = 'front' if states[other][0] >= states[c][0] else 'rear'
            u, report = solve(states[c], previous[c], warm[c], now, scene['retreat_x'][c],
                              cfg=cfg, allow_reverse=True, road=scene['road'], terminal_stop=True, **{side: peer})
            p, v = matrices(ticks-now, cfg)
            sampled = np.column_stack((states[c][0]+(ticks-now)*states[c][1]+p@u,
                                       states[c][1]+v@u, np.zeros(len(ticks))))
            sign = -1 if c == 'A' else 1
            feasible = (report['accepted'] and np.all(inside_road(sampled, scene['road'], cfg)) and
                        np.min(body_gap(sampled, peer, cfg)) >= scene['sampled_gap_min_m'] and
                        sign*(sampled[-1, 0]-states[c][0]) > .1)
            report.update(cycle=cycle, car=c, purpose='probe', feasible=bool(feasible))
            records.append(report)
            probes[c] = bool(feasible)
        if phase == 'unassigned':
            selected, scores = choose_role(states, scene, cfg, scene['selection_metric'],
                                            [c for c in states if probes[c] and recovery_feasible(c, scene, cfg)] if policy == 'checked' else None)
            if policy == 'fixed_A':
                selected = 'A'
            if policy == 'dmpc':
                selected = None
            if selected:
                role, phase, last_switch = selected, 'retreat', cycle*cfg.dt
                events.append(dict(time_s=last_switch, kind='assignment', role=role, scores=scores))
        elif policy == 'checked' and phase == 'retreat':
            selected, scores = choose_role(states, scene, cfg, scene['selection_metric'],
                                            [c for c in states if probes[c] and recovery_feasible(c, scene, cfg)])
            # Switch only before lateral entry, while both cars are stopped and
            # the new retreat and geometric recovery have passed checks.
            all_scores = choose_role(states, scene, cfg, scene['selection_metric'])[1]
            if (selected and selected != role and probes[role] and
                    all(abs(s[1]) <= scene['arrival_speed_tolerance_m_s'] for s in states.values()) and
                    switching_allowed(cycle*cfg.dt, last_switch, all_scores[role], all_scores[selected], scene)):
                old, role, last_switch = role, selected, cycle*cfg.dt
                switches += 1
                events.append(dict(time_s=last_switch, kind='role_switch', old=old, role=role, scores=scores))
        if role:
            passer = 'B' if role == 'A' else 'A'
            bay = scene['retreat_x'][role]
            if phase == 'retreat' and stopped(role, bay):
                lanes[role] = [0., scene['bay_y'], now, cfg.lateral_seconds]
                phase = 'enter_bay'
            elif phase == 'enter_bay' and now >= lanes[role][2]+cfg.lateral_seconds-1e-8:
                phase, clearance_at = 'pass', cycle*cfg.dt
            elif phase == 'pass' and abs(states[passer][0]-bay) >= cfg.length+cfg.margin+.3 and (
                    states[passer][0] < bay if role == 'A' else states[passer][0] > bay):
                lanes[role] = [scene['bay_y'], 0., now, cfg.lateral_seconds]
                phase = 'leave_bay'
            elif phase == 'leave_bay' and now >= lanes[role][2]+cfg.lateral_seconds-1e-8:
                phase = 'depart'
        controls = {}
        for c, other in (('A', 'B'), ('B', 'A')):
            target = scene['goals_x'][c]
            if role and c == role and phase != 'depart':
                target = scene['retreat_x'][c]
            elif role and c != role and phase in ('retreat', 'enter_bay'):
                # Hold the initial position while the yielding car clears.
                # Moving starts brake towards its natural stopping point.
                x0, v0 = scene['initial_states_x_v'][c]
                target = x0+v0*abs(v0)/(2*cfg.max_acceleration)
            overlap = np.abs(lane_y(lanes[c], ticks)-peers[other][:, 2]) < cfg.width+scene['sampled_gap_min_m']
            neighbor = peers[other].copy()
            front = states[other][0] >= states[c][0]
            neighbor[~overlap, 0] = 1e4 if front else -1e4
            u, report = solve(states[c], previous[c], warm[c], now, target,
                              cfg=cfg, allow_reverse=True, road=scene['road'], terminal_stop=True,
                              **{('front' if front else 'rear'): neighbor})
            report.update(cycle=cycle, car=c, purpose='execute', phase=phase, role=role,
                          controls=u.tolist(), cycle_seconds=None)
            records.append(report)
            controls[c] = u
        cycle_seconds = time.perf_counter()-started
        records[-1]['cycle_seconds'] = cycle_seconds
        if not all(r['accepted'] for r in records[-2:]):
            failure = 'optimization_rejected'
            break
        outgoing = {c: prediction(states[c], controls[c], lanes[c], now, cfg) for c in states}
        publications.extend(dict(cycle=cycle, car=c, **packet) for c, packet in outgoing.items())
        relative = np.arange(cfg.substeps+1)*cfg.dt/cfg.substeps
        motions = {c: np.column_stack((integrate(states[c], controls[c][0], relative),
                                       lane_y(lanes[c], now+relative))) for c in states}
        log_motion(now, motions, {c: controls[c][0] for c in states})
        for c in states:
            states[c] = motions[c][-1, :2]
            previous[c] = float(controls[c][0])
            warm[c] = np.r_[controls[c][1:], controls[c][-1]]
        packets, elapsed = outgoing, (cycle+1)*cfg.dt
        history.append((elapsed, {c: s.copy() for c, s in states.items()}))
        while len(history) > 1 and history[1][0] <= elapsed-scene['deadlock_window_s']+1e-8:
            history.pop(0)
        if (stall_at is None and elapsed-history[0][0] >= scene['deadlock_window_s']-1e-8 and
                all(np.ptp([h[1][c][0] for h in history]) <= .01 and
                    max(abs(h[1][c][1]) for h in history) <= .01 and
                    not stopped(c, scene['goals_x'][c]) for c in states)):
            stall_at = elapsed
        # Recovery means priority car actually passes the parked yielding car.
        if clearance_at is not None and recovery is None and phase in ('leave_bay', 'depart'):
            recovery = elapsed
        if all(stopped(c, scene['goals_x'][c]) for c in states) and phase not in ('enter_bay', 'leave_bay'):
            complete = elapsed
            break
    executed = [r for r in records if r['purpose'] == 'execute']
    status = 'failed' if failure else 'completed' if complete is not None else 'timeout'
    summary = dict(policy=policy, **scene['factors'], status=status, failure=failure,
                   minimum_sampled_gap_m=min_gap, simulation_seconds=elapsed,
                   rejected_execution_solves=sum(not r['accepted'] for r in executed),
                   rejected_probe_solves=sum(not r['accepted'] for r in records if r['purpose'] == 'probe'),
                   infeasible_probe_count=sum(not r['feasible'] for r in records if r['purpose'] == 'probe'),
                   optimization_rejected=failure == 'optimization_rejected',
                   timeout=status == 'timeout', stall_detected_at_s=stall_at,
                   recovery_time_s=recovery, task_completion_time_s=complete,
                   recovery_failed=role is not None and recovery is None,
                   role_switch_count=switches, reverse_distance_m=reverse,
                   spontaneous_reverse=policy == 'dmpc' and max(reverse.values()) > .1,
                   final_role=role, final_phase=phase, solve_count=len(records),
                   cycles_over_control_period=sum(r.get('cycle_seconds', 0.) > cfg.dt
                       for r in records if r.get('cycle_seconds') is not None),
                   continuous_safety_certified=False, steering_feasibility_tested=False)
    write_json(output/'summary.json', summary)
    write_json(output/'solves.json', records)
    write_json(output/'events.json', events)
    with (output/'actual.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['absolute_time_s', 'car', 'x', 'vx', 'y', 'acceleration', 'phase', 'body_signed_speed'])
        writer.writerows(actual)
    with (output/'predictions.jsonl').open('w') as stream:
        for p in publications:
            stream.write(json.dumps(p, allow_nan=False)+'\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config/comparison.json')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--smoke', action='store_true', help='first factor combination, all four policies')
    args = parser.parse_args()
    base = json.loads(args.config.read_text())
    if base['selection_metric'] not in ('distance', 'time'):
        raise ValueError('selection_metric must be distance or time')
    args.output.mkdir(parents=True, exist_ok=False)
    write_json(args.output/'study_config.json', base)
    rows = []
    grid = itertools.product(*(base['sweep'][k] for k in
                               ('encounter_x_m', 'initial_speed_m_s', 'channel_length_m', 'braking_m_s2')))
    for i, factors in enumerate(grid):
        scene = make_scene(base, *factors)
        for policy in POLICIES:
            result = run_trial(args.output/f'case_{i:03d}'/policy, scene, policy)
            rows.append(result)
            print(f'{i:03d} {policy}: {result["status"]}, gap={result["minimum_sampled_gap_m"]:.3f}', flush=True)
        if args.smoke:
            break
    write_json(args.output/'summary.json', rows)
    columns = [k for k in rows[0] if k != 'reverse_distance_m']+['A_reverse_m', 'B_reverse_m']
    with (args.output/'summary.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for r in rows:
            writer.writerow({**{k: v for k, v in r.items() if k != 'reverse_distance_m'},
                             'A_reverse_m': r['reverse_distance_m']['A'], 'B_reverse_m': r['reverse_distance_m']['B']})
    sources = [Path(__file__), ROOT/'scripts/head_on_core.py', args.config]
    write_json(args.output/'provenance.json', {'source_sha256': {
        str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        'numpy': np.__version__, 'smoke': args.smoke})
    lines = ['# 双车死锁对照实验', '', '每个初值依次执行四种策略；详见 summary.csv 与各组原始日志。', '',
             '|策略|完成/总数|执行求解拒绝|任务超时|恢复失败|角色切换|最小采样间隙 m|',
             '|---|---|---|---|---|---|---|']
    for policy in POLICIES:
        group = [r for r in rows if r['policy'] == policy]
        lines.append(f'|{policy}|{sum(r["status"] == "completed" for r in group)}/{len(group)}|'
                     f'{sum(r["optimization_rejected"] for r in group)}|{sum(r["timeout"] for r in group)}|'
                     f'{sum(r["recovery_failed"] for r in group)}|{sum(r["role_switch_count"] for r in group)}|'
                     f'{min(r["minimum_sampled_gap_m"] for r in group):.4f}|')
    lines += ['', '未完成任务的时间保留为空；失败运行的最小间隙仅覆盖实际执行片段。',
              '横向占用为预设曲线，未验证转向可达性；安全检查为离散采样。',
              '制动能力通过对称加速度上限改变。时间选择为启发式估计。',
              '相同上限：每周期两次探测、两次执行求解；总墙钟耗时包含探测和策略检查。']
    (args.output/'report.md').write_text('\n'.join(lines)+'\n')


if __name__ == '__main__':
    main()
