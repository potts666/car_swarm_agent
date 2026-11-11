#!/usr/bin/env python3
"""Generate and audit complete A-retreat/B-wait and B-retreat/A-wait exits."""
import argparse
import csv
import hashlib
import itertools
import json
import time
from pathlib import Path

import numpy as np

from head_on_core import (Settings, body_gap, inside_road, integrate, lane_y,
                          matrices, solve, write_json)
from run_comparison import ROOT, make_scene
from trajectory_acceptance import audit_controls, crosscheck_qp, road_residual, solve_qp


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
    residuals = {
        'road_body_m': max(road_residual(m, scene['road'], cfg) for m in motions.values()),
        'neighbor_gap_m': float(max(0., cfg.margin-np.min(gap))),
        'speed_m_s': max(float(max(0., np.max(np.abs(m[:, 1]))-cfg.max_speed)) for m in motions.values()),
        'acceleration_m_s2': max(float(max(0., np.max(np.abs(a))-cfg.max_acceleration)) for a in accelerations.values()),
        'waiter_position_m': float(np.max(np.abs(motions[waiter][:, 0]-waiting_x))),
        'waiter_velocity_m_s': float(np.max(np.abs(motions[waiter][:, 1]))),
        'waiter_lateral_m': float(np.max(np.abs(motions[waiter][:, 2])))}
    return {'passed': not reasons, 'reasons': reasons, 'residuals': residuals,
            'minimum_sampled_gap_m': float(np.min(gap)), 'samples_per_car': len(gap)}


def validate_start(scene, cfg, speed_tolerance=1e-10):
    states = {c: np.array([[*s, 0.]], float) for c, s in scene['initial_states_x_v'].items()}
    if any(abs(s[1]) > speed_tolerance for s in scene['initial_states_x_v'].values()):
        raise ValueError('Exit screening requires two stationary cars')
    if not states['A'][0, 0] < states['B'][0, 0]:
        raise ValueError('A must be left of B')
    if not check_path(states, {'A': [0.], 'B': [0.]}, scene, cfg, 'B', states['B'][0, 0])['passed']:
        raise ValueError('Initial state must satisfy road, limits and padded separation')


def generate_exit(directory, scene, role, qp_crosscheck=False, solver_backend="slsqp", checkpoint=None):
    """Roll out to the external bay, then enter it fully, with peer stopped."""
    generation_started = time.perf_counter()
    cfg = Settings(**scene['settings'])
    validate_start(scene, cfg, speed_tolerance=cfg.tolerance if checkpoint is not None else 1e-10)
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    waiter = 'B' if role == 'A' else 'A'
    states = {c: np.array(s, float) for c, s in scene['initial_states_x_v'].items()}
    waiting_x = float(states[waiter][0])
    warm = {c: np.zeros(cfg.horizon) for c in states}
    previous = dict.fromkeys(states, 0.)
    start_time = cfg.epoch
    if checkpoint is not None:
        start_time = checkpoint['absolute_time_s']
        for c in states:
            np.testing.assert_array_equal(states[c], checkpoint['states'][c][:2])
            previous[c] = checkpoint['previous_acceleration'][c]
            warm[c] = np.array(checkpoint['warm_controls'][c], float)
    lane = [0., 0., start_time, cfg.lateral_seconds]
    phase, failure, end_time, complete = 'retreat', None, 0., False
    records, rows, audits, predictions = [], [], [], []
    initial = {c: np.array([[*s, 0.]]) for c, s in states.items()}
    audits.append(check_path(initial, {'A': [0.], 'B': [0.]}, scene, cfg, waiter, waiting_x))
    for c, m in initial.items():
        rows.append([start_time, c, *m[0], previous[c], phase])
    for cycle in range(int(scene['duration_s']/cfg.dt)):
        now = start_time+cycle*cfg.dt
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
            if solver_backend == 'osqp':
                u, report = solve_qp(states[c], previous[c], warm[c], now, target, cfg, scene['road'], **kwargs)
            else:
                u, report = solve(states[c], previous[c], warm[c], now, target, cfg=cfg,
                                  allow_reverse=True, terminal_stop=True, road=scene['road'], **kwargs)
                report['solver'] = 'SLSQP'
            acceptance = audit_controls(states[c], u, target, cfg, scene['road'], **kwargs)
            report['optimizer_accepted'] = report['accepted']
            report['independent_acceptance'] = acceptance
            report['accepted'] = acceptance['passed']
            if qp_crosscheck:
                report['qp_crosscheck'] = crosscheck_qp(states[c], previous[c], warm[c], now, target,
                                                       cfg, scene['road'], u, **kwargs)
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
    terminal = {c: np.array([[*s, lane_y(lane, [start_time+end_time])[0] if c == role else 0.]])
                for c, s in states.items()}
    terminal_audit = check_path(terminal, {'A': [0.], 'B': [0.]}, scene, cfg, waiter, waiting_x)
    terminal_audit['exit_residuals'] = {
        'position_m': abs(float(states[role][0]-scene['retreat_x'][role])),
        'velocity_m_s': abs(float(states[role][1])),
        'lateral_m': abs(float(terminal[role][0, 2]-scene['bay_y']))}
    terminal_ok = bool(terminal_audit['passed'] and
        abs(states[role][0]-scene['retreat_x'][role]) <= scene['arrival_position_tolerance_m'] and
        abs(states[role][1]) <= scene['arrival_speed_tolerance_m_s'] and
        abs(terminal[role][0, 2]-scene['bay_y']) <= cfg.tolerance)
    feasible = bool(complete and terminal_ok and all(a['passed'] for a in audits) and not failure)
    compute_seconds = time.perf_counter()-generation_started
    summary = dict(role=role, waiter=waiter, feasible=feasible, terminal_passed=terminal_ok,
                   failure=failure or (None if complete else 'exit_timeout'),
                   exit_time_s=round(end_time, 10) if feasible else None,
                   attempted_time_s=round(end_time, 10),
                   minimum_sampled_gap_m=min(a['minimum_sampled_gap_m'] for a in audits),
                   full_path_checks=len(audits), solve_count=len(records),
                   generation_compute_wall_seconds=compute_seconds,
                   primary_solve_calls=len(records), qp_crosscheck_calls=sum('qp_crosscheck' in r for r in records),
                   solver_backend=solver_backend, retry_count=0, acceptance_rule='finite and all independent residual checks and full path; optimizer status diagnostic',
                   nonconverged_accepted_solves=sum(not r['solver_success'] and r['accepted'] for r in records),
                   retreat_distance_m=abs(scene['initial_states_x_v'][role][0]-states[role][0]),
                   rejected_solves=sum(not r['accepted'] for r in records),
                   final_states_x_v={c: s.tolist() for c, s in states.items()},
                   terminal_check=terminal_audit, continuous_safety_certified=False)
    write_json(directory/'config.json', {**scene, 'role': role, 'waiter': waiter,
               'qp_crosscheck': qp_crosscheck, 'acceptance_rule': summary['acceptance_rule'], 'initial_checkpoint': checkpoint})
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
    summary['generation_total_wall_seconds'] = time.perf_counter()-generation_started
    write_json(directory/'summary.json', summary)
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
    parser.add_argument('--qp-crosscheck', action='store_true', help='diagnostic OSQP solve for every primary call; not fallback')
    parser.add_argument('--solver', choices=('slsqp', 'osqp'), default='slsqp')
    args = parser.parse_args()
    base = json.loads(args.config.read_text())
    args.output.mkdir(parents=True, exist_ok=False)
    write_json(args.output/'study_config.json', base)
    results = []
    grid = itertools.product(*(base['sweep'][k] for k in ('encounter_x_m', 'channel_length_m', 'braking_m_s2')))
    for i, (encounter, length, braking) in enumerate(grid):
        scene = make_scene(base, encounter, 0., length, braking)
        planning_started = time.perf_counter()
        candidates = {c: generate_exit(args.output/f'case_{i:03d}'/f'{c}_retreat', scene, c, args.qp_crosscheck, args.solver) for c in ('A', 'B')}
        generation_seconds = time.perf_counter()-planning_started
        selection_started = time.perf_counter()
        policies = select_baselines(candidates, scene)
        selection_seconds = time.perf_counter()-selection_started
        decision_wait = time.perf_counter()-planning_started
        for b in policies.values():
            c = candidates[b['role']] if b['role'] else None
            b.update(exit_time_s=c['exit_time_s'] if c else None,
                     retreat_distance_m=c['retreat_distance_m'] if c else None,
                     decision_wait_wall_seconds=decision_wait)
        result = dict(case=i, factors=scene['factors'], candidates=candidates, baselines=policies,
                      shared_generation_solve_count=sum(c['solve_count'] for c in candidates.values()),
                      qp_crosscheck_calls=sum(c['qp_crosscheck_calls'] for c in candidates.values()),
                      retry_count=0, candidate_generation_total_wall_seconds=generation_seconds,
                      selection_wall_seconds=selection_seconds, decision_wait_wall_seconds=decision_wait,
                      control_period_s=scene['settings']['dt'],
                      decision_within_control_period=decision_wait <= scene['settings']['dt'],
                      method='offline full closed-loop rehearsal before role selection')
        results.append(result)
        print(f'{i:03d}: A={candidates["A"]["feasible"]}, B={candidates["B"]["feasible"]}, screened={policies["screened"]["role"]}', flush=True)
    write_json(args.output/'summary.json', results)
    with (args.output/'summary.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['case', 'encounter_x_m', 'channel_length_m', 'braking_m_s2', 'A_exit_feasible', 'B_exit_feasible',
                         'A_exit_time_s', 'B_exit_time_s', 'fixed_A_feasible', 'screened_role', 'screened_feasible',
                         'candidate_generation_wall_s', 'primary_calls', 'qp_calls', 'retries', 'decision_wait_wall_s',
                         'fixed_retreat_m', 'screened_retreat_m'])
        for r in results:
            f, c, b = r['factors'], r['candidates'], r['baselines']
            writer.writerow([r['case'], f['encounter_x_m'], f['channel_length_m'], f['braking_m_s2'],
                             c['A']['feasible'], c['B']['feasible'], c['A']['exit_time_s'], c['B']['exit_time_s'],
                             b['fixed_A']['feasible'], b['screened']['role'], b['screened']['feasible'],
                             r['candidate_generation_total_wall_seconds'], r['shared_generation_solve_count'],
                             r['qp_crosscheck_calls'], r['retry_count'], r['decision_wait_wall_seconds'],
                             b['fixed_A']['retreat_distance_m'], b['screened']['retreat_distance_m']])
    sources = [Path(__file__), ROOT/'scripts/head_on_core.py', ROOT/'scripts/run_comparison.py', ROOT/'scripts/trajectory_acceptance.py', args.config]
    write_json(args.output/'provenance.json', {'source_sha256': {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}})
    lines = [f'主求解器：{args.solver}；独立验收；QP诊断：{args.qp_crosscheck}。', '', '# 安全静止阻塞：双候选完整退出检查', '',
             '退出定义：退让车从原位倒车到入口外退让区，并完成横向占用变化、低速停在区内；等待车全程保持原位。退出不包括后续对方通行、回归和双方任务完成。', '',
             '|初值|A退/B等|B退/A等|固定A可行|筛选角色|', '|---|---|---|---|---|']
    for r in results:
        c, b = r['candidates'], r['baselines']
        lines.append(f'|{r["case"]}|{c["A"]["feasible"]}|{c["B"]["feasible"]}|{b["fixed_A"]["feasible"]}|{b["screened"]["role"]}|')
    lines += ['', '每个候选保存 actual.csv、全部预测 predictions.jsonl、逐段与整段预测检查 path_checks.json、solves.json、summary.json。',
              '两个基线共享两条候选和相同生成预算。固定 A 忽略筛选结果；筛选版在完整退出检查通过的候选中选择退让距离较短者，两条均不通过时不分配角色。没有执行已被判为失败的候选。',
              '检查覆盖初始状态、每周期完整 6.4 s 预测、每个实际执行子步和最终停驻状态；间隔 0.1 s。失败或超时不能因端点安全而通过。',
              '所有样本均通过不等于连续时间证明。横向位置沿既有预设曲线变化，不包含转向动力学。',
              '接受规则独立于 solver_success：有限控制、独立分项残差、配对全预测、实际执行和退出终态均通过；两个基线共用该新规则，所有候选完整重跑。',
              '定位为离线完整闭环模拟预演。先串行生成 A/B 两条完整候选，之后选角；不是在线 DMPC 的一次有限时域决策。',
              'summary.csv/json 报告两候选总墙钟时间（含审计、可选QP交叉验证、日志写入）、主求解调用、QP诊断调用、重试次数、选角等待时间。无失败重试、无QP替代控制。',
              '退出模拟时间不等于计算墙钟时间。退出不包括后续恢复与双车任务完成；本实验不能报告恢复时间收益。',
              '所有候选若都可行，本组数据只支持距离选择的比较，不能证明筛选减少不可行；针对端点安全但中途违规的拒绝行为另有自动测试。',
              '', '终端与硬路径约束见 [约束清单](../../constraints_exit.md)。']
    for policy in ('fixed_A', 'screened'):
        group = [r['baselines'][policy] for r in results]
        lines.append(f'\n{policy}: 成功 {sum(b["feasible"] for b in group)}/{len(group)}；平均退让距离 {np.mean([b["retreat_distance_m"] for b in group if b["feasible"]]):.4f} m；平均退出模拟时间 {np.mean([b["exit_time_s"] for b in group if b["feasible"]]):.4f} s。')
    lines.append(f'\n角色选择等待墙钟范围 {min(r["decision_wait_wall_seconds"] for r in results):.6f}–{max(r["decision_wait_wall_seconds"] for r in results):.6f} s；控制周期为 0.4 s。本机单次计时不构成实时保证。')
    (args.output/'report.md').write_text('\n'.join(lines)+'\n')


if __name__ == '__main__':
    main()
