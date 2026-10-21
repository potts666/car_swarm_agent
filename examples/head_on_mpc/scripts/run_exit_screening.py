#!/usr/bin/env python3
"""Generate and audit complete A-retreat/B-wait and B-retreat/A-wait exits."""
import argparse
import csv
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np

from head_on_core import (Settings, body_gap, inside_road, integrate, lane_y,
                          matrices, solve, write_json)
from run_comparison import ROOT, make_scene


def check_path(motions, accelerations, scene, cfg, waiter, waiting_x):
    """Check every supplied sample; never infer path feasibility from endpoints."""
    reasons = []
    gap = body_gap(motions['A'], motions['B'], cfg)
    if not all(np.all(np.isfinite(m)) for m in motions.values()):
        reasons.append('nonfinite_path')
    if np.min(gap) < cfg.margin-cfg.tolerance:
        reasons.append('separation')
    for c, motion in motions.items():
        if not np.all(inside_road(motion, scene['road'], cfg)):
            reasons.append(c+':road_body_containment')
        if np.max(np.abs(motion[:, 1])) > cfg.max_speed+cfg.tolerance:
            reasons.append(c+':speed')
        if np.max(np.abs(accelerations[c])) > cfg.max_acceleration+cfg.tolerance:
            reasons.append(c+':acceleration')
    if (np.max(np.abs(motions[waiter][:, 0]-waiting_x)) > .01 or
            np.max(np.abs(motions[waiter][:, 1])) > .01 or
            np.max(np.abs(motions[waiter][:, 2])) > cfg.tolerance):
        reasons.append('waiter_moved')
    return {'passed': not reasons, 'reasons': reasons,
            'minimum_sampled_gap_m': float(np.min(gap)), 'samples_per_car': len(gap)}


def validate_start(scene, cfg):
    states = {c: np.array([[*s, 0.]], float) for c, s in scene['initial_states_x_v'].items()}
    if any(abs(s[1]) > 1e-10 for s in scene['initial_states_x_v'].values()):
        raise ValueError('Exit screening requires two stationary cars')
    if not states['A'][0, 0] < states['B'][0, 0]:
        raise ValueError('A must be left of B')
    if not check_path(states, {'A': [0.], 'B': [0.]}, scene, cfg, 'B', states['B'][0, 0])['passed']:
        raise ValueError('Initial state must satisfy road, limits and padded separation')


def generate_exit(directory, scene, role):
    """Roll out to the external bay, then enter it fully, with peer stopped."""
    cfg = Settings(**scene['settings'])
    validate_start(scene, cfg)
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    waiter = 'B' if role == 'A' else 'A'
    states = {c: np.array(s, float) for c, s in scene['initial_states_x_v'].items()}
    waiting_x = float(states[waiter][0])
    warm = {c: np.zeros(cfg.horizon) for c in states}
    previous = dict.fromkeys(states, 0.)
    lane = [0., 0., cfg.epoch, cfg.lateral_seconds]
    phase, failure, end_time, complete = 'retreat', None, 0., False
    records, rows, audits, predictions = [], [], [], []
    initial = {c: np.array([[*s, 0.]]) for c, s in states.items()}
    audits.append(check_path(initial, {'A': [0.], 'B': [0.]}, scene, cfg, waiter, waiting_x))
    for c, m in initial.items():
        rows.append([cfg.epoch, c, *m[0], 0., phase])
    for cycle in range(int(scene['duration_s']/cfg.dt)):
        now = cfg.epoch+cycle*cfg.dt
        at_bay = (abs(states[role][0]-scene['retreat_x'][role]) <= scene['arrival_position_tolerance_m'] and
                  abs(states[role][1]) <= scene['arrival_speed_tolerance_m_s'])
        if phase == 'retreat' and at_bay:
            phase, lane = 'enter_bay', [0., scene['bay_y'], now, cfg.lateral_seconds]
        if phase == 'enter_bay' and now >= lane[2]+cfg.lateral_seconds-1e-8 and at_bay:
            complete = True
            break
        ticks = np.arange(1, cfg.horizon*cfg.substeps+1)*cfg.dt/cfg.substeps
        pmat, vmat = matrices(ticks, cfg)
        controls, predicted = {}, {}
        # Generate retreat first against the stationary peer, then solve the
        # waiting car against that candidate. Both retain hard peer constraints.
        for c in (role, waiter):
            target = scene['retreat_x'][role] if c == role else waiting_x
            kwargs = {}
            if c == role:
                peer = np.tile([waiting_x, 0., 0.], (len(ticks), 1))
                kwargs['front' if c == 'A' else 'rear'] = peer
            else:
                peer = predicted[role].copy()
                front = states[role][0] >= states[c][0]
                peer[np.abs(peer[:, 2]) >= cfg.width+cfg.margin, 0] = 1e4 if front else -1e4
                kwargs['front' if front else 'rear'] = peer
            u, report = solve(states[c], previous[c], warm[c], now, target, cfg=cfg,
                              allow_reverse=True, terminal_stop=True, road=scene['road'], **kwargs)
            report.update(cycle=cycle, car=c, phase=phase, controls=u.tolist())
            records.append(report)
            controls[c] = u
            predicted[c] = np.column_stack((states[c][0]+ticks*states[c][1]+pmat@u,
                                            states[c][1]+vmat@u,
                                            lane_y(lane, now+ticks) if c == role else np.zeros(len(ticks))))
        audit = check_path(predicted, controls, scene, cfg, waiter, waiting_x)
        audit.update(cycle=cycle, kind='whole_prediction')
        audits.append(audit)
        predictions.append(dict(cycle=cycle, times=(now+ticks).tolist(),
                                states={c: p.tolist() for c, p in predicted.items()}))
        if not all(r['accepted'] for r in records[-2:]):
            failure = 'optimization_rejected'
            break
        if not audit['passed']:
            failure = 'prediction_path_rejected'
            break
        relative = np.arange(cfg.substeps+1)*cfg.dt/cfg.substeps
        motions = {c: np.column_stack((integrate(states[c], controls[c][0], relative),
                    lane_y(lane, now+relative) if c == role else np.zeros(len(relative)))) for c in states}
        audit = check_path(motions, {c: [u[0]] for c, u in controls.items()}, scene, cfg, waiter, waiting_x)
        audit.update(cycle=cycle, kind='executed_segment')
        audits.append(audit)
        for c, motion in motions.items():
            for j, sample in enumerate(motion):
                rows.append([round(now+relative[j], 10), c, *sample, float(controls[c][0]), phase])
            states[c] = motion[-1, :2]
            previous[c] = float(controls[c][0])
            warm[c] = np.r_[controls[c][1:], controls[c][-1]]
        end_time = (cycle+1)*cfg.dt
        if not audit['passed']:
            failure = 'executed_path_rejected'
            break
    terminal = {c: np.array([[*s, lane_y(lane, [cfg.epoch+end_time])[0] if c == role else 0.]])
                for c, s in states.items()}
    terminal_audit = check_path(terminal, {'A': [0.], 'B': [0.]}, scene, cfg, waiter, waiting_x)
    terminal_ok = bool(terminal_audit['passed'] and
        abs(states[role][0]-scene['retreat_x'][role]) <= scene['arrival_position_tolerance_m'] and
        abs(states[role][1]) <= scene['arrival_speed_tolerance_m_s'] and
        abs(terminal[role][0, 2]-scene['bay_y']) <= cfg.tolerance)
    feasible = bool(complete and terminal_ok and all(a['passed'] for a in audits) and not failure)
    summary = dict(role=role, waiter=waiter, feasible=feasible, terminal_passed=terminal_ok,
                   failure=failure or (None if complete else 'exit_timeout'),
                   exit_time_s=round(end_time, 10) if feasible else None,
                   attempted_time_s=round(end_time, 10),
                   minimum_sampled_gap_m=min(a['minimum_sampled_gap_m'] for a in audits),
                   full_path_checks=len(audits), solve_count=len(records),
                   rejected_solves=sum(not r['accepted'] for r in records),
                   final_states_x_v={c: s.tolist() for c, s in states.items()},
                   terminal_check=terminal_audit, continuous_safety_certified=False)
    write_json(directory/'config.json', {**scene, 'role': role, 'waiter': waiter})
    write_json(directory/'summary.json', summary)
    write_json(directory/'path_checks.json', audits)
    write_json(directory/'solves.json', records)
    with (directory/'predictions.jsonl').open('w') as stream:
        for entry in predictions:
            stream.write(json.dumps(entry, allow_nan=False)+'\n')
    with (directory/'actual.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['absolute_time_s', 'car', 'x', 'vx', 'y', 'acceleration', 'phase'])
        writer.writerows(rows)
    return summary


def select_baselines(candidates, scene):
    # Both policies receive the same two generated candidates and total budget.
    # Fixed A ignores feasibility; screening removes failed complete exits.
    eligible = [c for c in ('A', 'B') if candidates[c]['feasible']]
    chosen = min(eligible, key=lambda c: (abs(scene['initial_states_x_v'][c][0]-scene['retreat_x'][c]), c)) if eligible else None
    return {'fixed_A': {'role': 'A', 'feasible': candidates['A']['feasible']},
            'screened': {'role': chosen, 'feasible': chosen is not None}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config/comparison.json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    base = json.loads(args.config.read_text())
    args.output.mkdir(parents=True, exist_ok=False)
    write_json(args.output/'study_config.json', base)
    results = []
    grid = itertools.product(*(base['sweep'][k] for k in ('encounter_x_m', 'channel_length_m', 'braking_m_s2')))
    for i, (encounter, length, braking) in enumerate(grid):
        scene = make_scene(base, encounter, 0., length, braking)
        candidates = {c: generate_exit(args.output/f'case_{i:03d}'/f'{c}_retreat', scene, c) for c in ('A', 'B')}
        policies = select_baselines(candidates, scene)
        result = dict(case=i, factors=scene['factors'], candidates=candidates, baselines=policies,
                      shared_generation_solve_count=sum(c['solve_count'] for c in candidates.values()))
        results.append(result)
        print(f'{i:03d}: A={candidates["A"]["feasible"]}, B={candidates["B"]["feasible"]}, screened={policies["screened"]["role"]}', flush=True)
    write_json(args.output/'summary.json', results)
    with (args.output/'summary.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['case', 'encounter_x_m', 'channel_length_m', 'braking_m_s2', 'A_exit_feasible', 'B_exit_feasible',
                         'A_exit_time_s', 'B_exit_time_s', 'fixed_A_feasible', 'screened_role', 'screened_feasible'])
        for r in results:
            f, c, b = r['factors'], r['candidates'], r['baselines']
            writer.writerow([r['case'], f['encounter_x_m'], f['channel_length_m'], f['braking_m_s2'],
                             c['A']['feasible'], c['B']['feasible'], c['A']['exit_time_s'], c['B']['exit_time_s'],
                             b['fixed_A']['feasible'], b['screened']['role'], b['screened']['feasible']])
    sources = [Path(__file__), ROOT/'scripts/head_on_core.py', ROOT/'scripts/run_comparison.py', args.config]
    write_json(args.output/'provenance.json', {'source_sha256': {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}})
    lines = ['# 安全静止阻塞：双候选完整退出检查', '',
             '退出定义：退让车从原位倒车到入口外退让区，并完成横向占用变化、低速停在区内；等待车全程保持原位。退出不包括后续对方通行、回归和双方任务完成。', '',
             '|初值|A退/B等|B退/A等|固定A可行|筛选角色|', '|---|---|---|---|---|']
    for r in results:
        c, b = r['candidates'], r['baselines']
        lines.append(f'|{r["case"]}|{c["A"]["feasible"]}|{c["B"]["feasible"]}|{b["fixed_A"]["feasible"]}|{b["screened"]["role"]}|')
    lines += ['', '每个候选保存 actual.csv、全部预测 predictions.jsonl、逐段与整段预测检查 path_checks.json、solves.json、summary.json。',
              '两个基线共享两条候选和相同生成预算。固定 A 忽略筛选结果；筛选版在完整退出检查通过的候选中选择退让距离较短者，两条均不通过时不分配角色。没有执行已被判为失败的候选。',
              '检查覆盖初始状态、每周期完整 6.4 s 预测、每个实际执行子步和最终停驻状态；间隔 0.1 s。失败或超时不能因端点安全而通过。',
              '所有样本均通过不等于连续时间证明。横向位置沿既有预设曲线变化，不包含转向动力学。',
              '数值求解拒绝不等于严格不可行证明；此处 feasible 表示给定预算下已生成且通过检查的完整退出。',
              '所有候选若都可行，本组数据只支持距离选择的比较，不能证明筛选减少不可行；针对端点安全但中途违规的拒绝行为另有自动测试。',
              '', '终端与硬路径约束见 [约束清单](../../constraints_exit.md)。']
    (args.output/'report.md').write_text('\n'.join(lines)+'\n')


if __name__ == '__main__':
    main()
