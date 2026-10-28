#!/usr/bin/env python3
"""Offline EXIT -> PARKED_READY -> PASS -> CLEARED_HOLD -> RELEASED study."""
import argparse
import csv
import hashlib
import itertools
import json
from pathlib import Path
import platform
import shutil
import time

import numpy as np
import scipy

from head_on_core import Settings, write_json
from pass_release_core import geometry, run_branch, write_branch_logs
from run_comparison import ROOT, make_scene
from run_exit_screening import generate_exit, select_baselines


def failed_exit_branch(directory, source, scene, policy, role, wait):
    directory.mkdir(parents=True, exist_ok=False)
    result = json.loads((source/'summary.json').read_text())
    with (source/'actual.csv').open() as stream:
        original=list(csv.DictReader(stream))
    rows=[dict(absolute_time_s=float(r['absolute_time_s']),car=r['car'],x=float(r['x']),vx=float(r['vx']),
               y=float(r['y']),acceleration=float(r['acceleration']),stage='EXIT',exit_subphase=r['phase'],
               role=role or '',passer=('B' if role=='A' else 'A') if role else '',right_holder='') for r in original]
    stamp=max(r['absolute_time_s'] for r in rows)
    for c in ('A','B'):
        rows.append({**next(r for r in reversed(rows) if r['car']==c),'stage':'FAILED'})
    paired={}
    for row in rows:paired.setdefault(row['absolute_time_s'],{})[row['car']]=row
    cfg=Settings(**scene['settings'])
    executed=min(max(abs(p['A']['x']-p['B']['x'])-cfg.length,abs(p['A']['y']-p['B']['y'])-cfg.width) for p in paired.values())
    checks=json.loads((source/'path_checks.json').read_text())
    predicted=[a['minimum_sampled_gap_m'] for a in checks if a.get('kind')=='whole_prediction']
    summary = dict(policy=policy, role=role, status='FAILED', failure=('no_accepted_exit_candidate' if role is None else 'exit_candidate_failed:'+str(result['failure'])),
                   failure_phase='EXIT', exit_success=False, pass_success=False, released=False, overall_success=False,
                   both_tasks_completed=False, selection_wait_wall_s=wait, simulation_terminated=True,
                   verified_physical_safe_stop=False, role_switch_count=0, exit_simulation_s=result['attempted_time_s'],
                   pass_simulation_s=None, initial_to_release_simulation_s=None, retreat_distance_m=result['retreat_distance_m'],
                   passing_distance_m=0., minimum_executed_gap_m=executed, minimum_predicted_gap_m=min(predicted) if predicted else None,
                   phase_compute_wall_s={'EXIT': result['generation_compute_wall_seconds'],'PASS':0.,'CLEARED_HOLD':0.},
                   phase_primary_calls={'EXIT':result['primary_solve_calls'],'PASS':0,'CLEARED_HOLD':0},
                   phase_diagnostic_calls={'EXIT':0,'PASS':0,'CLEARED_HOLD':0},phase_retry_counts={'EXIT':0,'PASS':0,'CLEARED_HOLD':0},diagnostic_calls=0,retries=0)
    records=[{**r, 'stage': 'EXIT'} for r in json.loads((source/'solves.json').read_text())]
    paths=[{**r, 'stage': 'EXIT'} for r in checks]
    predictions=[{**json.loads(line), 'stage': 'EXIT', 'role': role}
                 for line in (source/'predictions.jsonl').read_text().splitlines()]
    events=[dict(kind='failure', absolute_time_s=stamp, failure_phase='EXIT', failure=summary['failure'],
                 states={c: [paired[stamp][c][key] for key in ('x', 'vx', 'y')] for c in ('A', 'B')},
                 simulation_terminated=True, verified_physical_safe_stop=False)]
    write_branch_logs(directory, config=dict(scene=scene, policy=policy, role=role), actual=rows,
                      solves=records, path_checks=paths, predictions=predictions, events=events)
    write_json(directory/'summary.json', summary)
    return summary


def validate_options(options, cfg):
    for k, v in options.items():
        if not np.isfinite(v) or v <= 0:
            raise ValueError('All timeout, hold and margin options must be finite and positive: '+k)
    if options['total_timeout_s'] < cfg.dt:
        raise ValueError('Total timeout must allow a full control step')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, default=ROOT/'config/comparison.json')
    p.add_argument('--options', type=Path, default=ROOT/'config/pass_release.json')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--hold-constraints', action='store_true', help='hard hold speed/position band and paired precheck before CLEARED_HOLD')
    args = p.parse_args()
    base = json.loads(args.config.read_text())
    options = json.loads(args.options.read_text())
    validate_options(options, Settings(**base['settings']))
    args.output.mkdir(parents=True, exist_ok=False)
    write_json(args.output/'study_config.json', dict(base=base, options=options, hold_constraints=args.hold_constraints))
    results = []
    grid = itertools.product(*(base['sweep'][k] for k in ('encounter_x_m', 'channel_length_m', 'braking_m_s2')))
    for i, factors in enumerate(grid):
        directory = args.output/f'case_{i:03d}'
        scene = make_scene(base, factors[0], 0., factors[1], factors[2])
        # No map or target adjustment is allowed to manufacture feasibility.
        geo = geometry(scene, Settings(**scene['settings']), options)
        scene['duration_s'] = min(scene['duration_s'], options['exit_timeout_s'], options['total_timeout_s'])
        begun = time.perf_counter()
        candidates = {c: generate_exit(directory/'candidates'/f'{c}_retreat', scene, c, solver_backend='osqp') for c in ('A','B')}
        generation_wall = time.perf_counter()-begun
        selected = select_baselines(candidates, scene) # EXIT results only; PASS has not run.
        wait = time.perf_counter()-begun
        # Persist/freeze selection before either continuation exists.
        write_json(directory/'selection.json', dict(candidates=candidates, selected=selected,
                   selection_wait_wall_s=wait, candidate_generation_wall_s=generation_wall,
                   primary_calls=sum(c['primary_solve_calls'] for c in candidates.values()),
                   diagnostic_calls=0, retries=0, used_pass_results=False, role_switch_allowed=False))
        branches = {}
        for policy in ('fixed_A','screened'):
            role = selected[policy]['role']
            # No eligible exit: screened fails without a role assignment.
            if role is None:
                branches[policy]=failed_exit_branch(directory/policy,directory/'candidates'/'A_retreat',scene,policy,None,wait)
            elif not candidates[role]['feasible']:
                branches[policy]=failed_exit_branch(directory/policy,directory/'candidates'/f'{role}_retreat',scene,policy,role,wait)
            else:
                branches[policy]=run_branch(directory/policy,directory/'candidates'/f'{role}_retreat',scene,policy,options,wait,hold_constraints=args.hold_constraints)
            r=branches[policy]
            print(f'{i:03d} {policy} role={role}: {r["status"]}, failure={r["failure"]}',flush=True)
        same_role_check=None
        if selected['fixed_A']['role']==selected['screened']['role'] and selected['fixed_A']['role']:
            # Actual trajectories/events should be identical; elapsed computation can differ.
            hashes={p:hashlib.sha256((directory/p/'actual.csv').read_bytes()).hexdigest() for p in branches}
            same_role_check=dict(actual_log_sha256=hashes,identical=len(set(hashes.values()))==1)
        results.append(dict(case=i, factors=scene['factors'], geometry=geo, selection_wait_wall_s=wait,
                            candidate_generation_wall_s=generation_wall,branches=branches,same_role_check=same_role_check))
    write_json(args.output/'summary.json',results)
    columns=['case','policy','role','exit_success','pass_success','released','overall_success','status','failure_phase','failure',
             'exit_simulation_s','pass_simulation_s','initial_to_release_simulation_s','retreat_distance_m','passing_distance_m',
             'selection_wait_wall_s','minimum_executed_gap_m','minimum_predicted_gap_m']
    with (args.output/'summary.csv').open('w') as stream:
        w=csv.DictWriter(stream,fieldnames=columns);w.writeheader()
        for r in results:
            for policy,b in r['branches'].items():
                w.writerow({k:(r['case'] if k=='case' else b.get(k)) for k in columns})
    import osqp
    sources=[ROOT/'scripts'/n for n in ('pass_release_core.py','run_pass_release.py','run_exit_screening.py','trajectory_acceptance.py','head_on_core.py','run_comparison.py','verify_pass_release.py')]+[args.config,args.options,ROOT/'pass_release_protocol.md',ROOT/'tests/test_pass_release.py',ROOT/'tests/test_hold_constraints.py',ROOT/'tests/test_pass_release_artifacts.py',ROOT/'scripts/compare_hold_update.py']
    write_json(args.output/'provenance.json',dict(source_sha256={str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        python=platform.python_version(),numpy=np.__version__,scipy=scipy.__version__,osqp=osqp.__version__,openblas_threads='1 requested by command',
        model='longitudinal double integrator and prescribed lateral occupancy; offline full rehearsal',
        command_solver='OSQP, max_iter=20000, no retries; optional holding precheck diagnostics separately counted'))
    report(args.output,results)


def report(output, results):
    study=json.loads((output/'study_config.json').read_text())
    options=study['options']
    cfg=Settings(**study['base']['settings'])
    holding_enabled=study.get('hold_constraints', False)
    lines=[f'保持QP硬约束与进入前预检查：{holding_enabled}。', '', '# 退出停驻 → 对方实际通过 → 释放通行权', '',
      '完整离线闭环模拟预演。本次只验证一辆退让、另一辆通过、释放指定冲突区域的占用权；不启动退让车回归，不标记双方任务完成。', '',
      '## 几何、目标与释放', '',
      'R为原单通道主路矩形，几何与停驻目标按各初值保存的geometry列于下表。沿用原道路和退让区，不移动边界。',
      f'车长 {cfg.length:g} m；释放裕度配置为 {options["release_margin_m"]:g} m。'
      f'A朝+x通过要求x−{cfg.length/2:g}≥R.xmax+{options["release_margin_m"]:g}，'
      f'B朝−x通过要求x+{cfg.length/2:g}≤R.xmin−{options["release_margin_m"]:g}。',
      f'实际到达表中目标容差且低速后，连续保持 {options["cleared_hold_s"]:g} s；'
      '退让车保持实际停驻参考，配对实际与最新全预测通过，预测未来也不会重新进入R，通行权持有者匹配。', '',
      '|初值|通道x区间 m|R的x/y区间 m|A/B停驻目标 m|释放裕度 m|位置/速度容差 m、m/s|',
      '|---|---|---|---|---|---|']
    for r in results:
        g=r['geometry'];channel=g['channel'];region=g['conflict_region'];targets=g['passing_stop_targets_x']
        lines.append(f'|{r["case"]:03d}|[{channel["xmin"]:g}, {channel["xmax"]:g}]|'
                     f'[{region["xmin"]:g}, {region["xmax"]:g}] / [{region["ymin"]:g}, {region["ymax"]:g}]|'
                     f'{targets["A"]:g} / {targets["B"]:g}|{g["release_margin_m"]:g}|'
                     f'{g["arrival_position_tolerance_m"]:g} / {g["arrival_speed_tolerance_m_s"]:g}|')
    lines+=['',
      '## 逐初值、逐基线', '',
      '|初值|相遇中心 m|通道长 m|加减速 m/s²|策略|退让角色|EXIT|PASS|RELEASED|失败|',
      '|---|---|---|---|---|---|---|---|---|---|']
    for r in results:
        f=r['factors']
        for policy,b in r['branches'].items():
            lines.append(f'|{r["case"]:03d}|{f["encounter_x_m"]}|{f["channel_length_m"]}|{f["braking_m_s2"]}|{policy}|{b["role"]}|{b["exit_success"]}|{b["pass_success"]}|{b["released"]}|{b["failure"]}|')
    lines+=['','## 实现文件','',
        '|文件|作用|','|---|---|',
        '|scripts/pass_release_core.py|从实际EXIT日志重建检查点；停驻与连续性检查；PASS、CLEARED_HOLD、RELEASED状态机与完整日志|',
        '|scripts/run_pass_release.py|八组初值两候选生成、冻结退出选角、独立运行两基线后续阶段、结果和计时报告|',
        '|config/pass_release.json|释放裕度、停驻确认时长、停驻保持阈值、EXIT/PASS/总超时|',
        '|scripts/verify_pass_release.py|独立从日志核验实际动力学、真实车身清空、分项验收、阶段连续性与角色一致性|',
        '|tests/test_pass_release.py|中途违规、漂移、预测提前清空、车尾未清空、超时与不改选等失效机制测试|',
        '|pass_release_protocol.md、README.md|几何、状态、日志、运行与结论说明|',
        '原复现入口与退出默认行为保持不变，QP增加可选保持约束。历史报告未改写，新增结果使用独立目录。']
    lines+=['','## 汇总与配对比较','','|策略|EXIT成功|退出后PASS成功|释放成功|完整链成功|平均退出 s|平均通过 s|平均阻塞至释放 s|平均退让 m|平均通过 m|','|---|---|---|---|---|---|---|---|---|']
    for policy in ('fixed_A','screened'):
        group=[r['branches'][policy] for r in results];good=[b for b in group if b['overall_success']];exit_good=[b for b in group if b['exit_success']]
        def mean(key):
            v=[b[key] for b in good if b.get(key) is not None];return f'{np.mean(v):.4f}' if v else 'NA'
        lines.append(f'|{policy}|{sum(b["exit_success"] for b in group)}/{len(group)}|{sum(b["pass_success"] for b in exit_good)}/{len(exit_good)}|{sum(b["released"] for b in group)}/{len(group)}|{len(good)}/{len(group)}|'+ '|'.join(mean(k) for k in ('exit_simulation_s','pass_simulation_s','initial_to_release_simulation_s','retreat_distance_m','passing_distance_m'))+'|')
    common=[r for r in results if all(b['overall_success'] for b in r['branches'].values())]
    lines+=['','上述性能均值的样本集合为各自完整链成功运行；PASS成功率以退出成功样本为分母，释放与整体成功率以全部初值为分母。',f'共同成功初值：{[r["case"] for r in common]}。']
    for key in ('retreat_distance_m','pass_simulation_s','initial_to_release_simulation_s'):
        diff=[r['branches']['screened'][key]-r['branches']['fixed_A'][key] for r in common]
        lines.append(f'共同成功集合 screened−fixed_A 的平均 {key} 差：{np.mean(diff) if diff else None}。')
    failures=[(r['case'],p,b) for r in results for p,b in r['branches'].items() if not b['overall_success']]
    lines+=['','## 失败与原始证据','']
    if not failures:lines.append('本组初值没有运行失败，也未出现退出成功但PASS失败。')
    for i,p,b in failures:
        lines.append(f'- case_{i:03d}/{p}: {b["failure_phase"]} / {b["failure"]}；见该目录 summary.json、solves.json、path_checks.json、events.jsonl。模拟中止不是已验证的物理安全停车。')
    lines+=['','退出成功但未完成PASS停驻确认的运行数：'+str(sum(b['exit_success'] and not b['pass_success'] for r in results for b in r['branches'].values()))+'。',
       ('各失败原因由分项验收日志定位；拒绝段未提交到模拟状态，不由安全端点或预测清空覆盖。' if failures else '本次未观察到保持拒绝或释放失败；结论限于当前模型和本组初值。'),
       '实际最小间隙只覆盖actual.csv中的已提交状态；拒绝执行的积分段与未执行预测不计入minimum_executed_gap_m。', '',
       '## 逐初值计算量', '',
       '|初值|选角前墙钟 s|两EXIT生成墙钟 s|两EXIT公共主调用|策略|EXIT归属主调用|PASS主调用|HOLD主调用|PASS计算墙钟 s|HOLD计算墙钟 s|诊断/重试|',
       '|---|---|---|---|---|---|---|---|---|---|---|']
    for r in results:
        selection=json.loads((output/f'case_{r["case"]:03d}'/'selection.json').read_text())
        for name,b in r['branches'].items():
            calls=b['phase_primary_calls'];wall=b['phase_compute_wall_s']
            lines.append(f'|{r["case"]:03d}|{r["selection_wait_wall_s"]:.6f}|{r["candidate_generation_wall_s"]:.6f}|{selection["primary_calls"]}|{name}|{calls.get("EXIT",0)}|{calls.get("PASS",0)}|{calls.get("CLEARED_HOLD",0)}|{wall.get("PASS",0):.6f}|{wall.get("CLEARED_HOLD",0):.6f}|{b.get("diagnostic_calls",0)}/{b.get("retries",0)}|')
    lines+=['','同角色案例的actual.csv完全一致：'+str({r['case']:r['same_role_check']['identical'] for r in results if r['same_role_check']})+'。']
    lines+=['','每个策略目录保存 EXIT 至终态的 actual.csv、predictions.jsonl、solves.json、path_checks.json、events.jsonl、summary.json。选择结果在任何PASS模拟前写入 case_*/selection.json；只依据完整退出成功和退让距离。相同角色的两基线独立继续积分，actual.csv哈希一致性见summary.json的same_role_check。',
       '两条EXIT来自同初值的独立候选；所选候选实际执行日志作为各策略EXIT前缀，后续从该日志终态继承时间、x/v/y、上一实际加速度及warm controls。没有重置状态、覆盖速度或控制。通过期间退让参考取实际终态，非理想退让点。',
       f'邻车掩码按实际横向占用逐样本重算并记录，激活阈值为车宽+裕度={cfg.width+cfg.margin:g} m；外部配对车身检查始终生效。',
       '局部独立分项验收、配对全预测、实际执行子步、阶段完成条件均必须通过。预测提前清空不产生释放；实际车尾清空时间另记，停驻确认和释放更晚。', '',
       '## 时间、预算与边界','',
       f'选角前墙钟等待范围 {min(r["selection_wait_wall_s"] for r in results):.6f}–{max(r["selection_wait_wall_s"] for r in results):.6f} s，包含两条完整EXIT预演与日志写入，不含进程启动。控制周期{cfg.dt:g} s、预测{cfg.horizon*cfg.dt:g} s、检查间隔{cfg.dt/cfg.substeps:g} s；每次主求解预算见solves.json；保持进入前诊断调用合计 {sum(b.get("diagnostic_calls",0) for r in results for b in r["branches"].values())}，无重试。',
       f'EXIT/PASS/整个模拟超时分别{options["exit_timeout_s"]}/{options["pass_timeout_s"]}/{options["total_timeout_s"]} s；PASS超时包括CLEARED_HOLD。超时终止模拟，未实现实时deadline中断或物理备用停车。各阶段计算墙钟时间和主调用详见每分支summary.json，候选生成公共成本见selection.json，不能把公共生成成本与每条分支的EXIT重计为实际CPU总量。',
       '模拟时间与计算墙钟时间分别报告，未相加成部署恢复时间。方法仍为离线预演，未补偿真实车辆等待期间的环境变化，不能声称满足在线实时预算。',
       '数值残差仅支持返回解在给定容差下通过独立验收；采样安全仅支持本组实际执行样本通过模型道路与间距检查；阶段结果只支持本组初值、当前模型下的一次完整退出/通过/释放。未证明连续时间安全、转向可达性、递归可行性或双方任务完成。']
    (output/'report.md').write_text('\n'.join(lines)+'\n')


if __name__=='__main__':main()
