#!/usr/bin/env python3
"""Independent incremental recovery study; original experiment CLIs unchanged."""
import argparse
import copy
import csv
import hashlib
import itertools
import json
import platform
import sys
from pathlib import Path
import time

import numpy as np

from head_on_core import Settings, body_gap, write_json
from pass_release_core import CARS, run_branch, snapshot, write_branch_logs
from recovery_core import BlockingDetector, append_cycle, motion_cycle
from run_comparison import ROOT, make_scene
from run_exit_screening import generate_exit
from switch_checks import TransitionGuard


def run_navigation(scene, options, recovery, directory):
    cfg=Settings(**scene['settings'])
    states={c:np.array(scene['initial_states_x_v'][c],float) for c in CARS}
    lanes={c:[0.,0.,cfg.epoch,cfg.lateral_seconds] for c in CARS}
    cp=snapshot(states,lanes,cfg.epoch,dict.fromkeys(CARS,0.),{c:np.zeros(cfg.horizon) for c in CARS})
    rows=[dict(absolute_time_s=cfg.epoch,car=c,x=float(s[0]),vx=float(s[1]),y=0.,acceleration=0.,
               stage='NAVIGATE',exit_subphase='',role='',passer='',right_holder='') for c,s in states.items()]
    records=[];paths=[];predictions=[];events=[];detections=[]
    detector=BlockingDetector(scene,recovery['detector']);failure=None;begun=time.perf_counter();trigger=None
    for cycle in range(int(recovery['navigation_timeout_s']/cfg.dt)):
        now=cp['absolute_time_s'];plan=motion_cycle(cp,scene,options,scene['goals_x'],stage='NAVIGATE',cycle=cycle)
        blocked={};blocking_measurements={}
        if plan['passed']:
            for c,other in (('A','B'),('B','A')):
                direction=np.sign(scene['goals_x'][c]-cp['states'][c][0])
                points=plan['predictions'];slack=direction*(points[other][:,0]-points[c][:,0])-cfg.length-cfg.margin
                beyond=direction*(scene['goals_x'][c]-cp['states'][other][0])>0
                active=plan['masks'][c]
                blocked[c]=bool(beyond and np.any(active & (slack<=recovery['detector']['active_neighbor_slack_m'])))
                blocking_measurements[c]=dict(goal_beyond_peer=bool(beyond),
                    overlap_sample_count=int(np.sum(active)),
                    minimum_active_neighbor_slack_m=float(np.min(slack[active])) if np.any(active) else None)
        observed=detector.observe(now,cp['states'],plan['predictions'],plan['passed'],blocked=blocked)
        observed['planning_stamp_s']=now
        observed['current_blocking_measurements']=blocking_measurements
        observed['current_blocked']=blocked
        detections.append(observed)
        if observed['triggered']:
            trigger=observed
            plan['checks'][-1]['applied_to_simulation']=False
        append_cycle(plan,cp,scene,'','','NAVIGATE',rows,records,paths,predictions,execute=trigger is None)
        if not plan['passed']:
            failure='solver_failure' if plan['failure']=='solver_failure' else plan['failure'];break
        if trigger:
            events.append(dict(kind='mutual_blocking_detected',absolute_time_s=now,evidence=observed,strict_deadlock_proof=False));break
        cp=plan['next']
        if all(abs(cp['states'][c][0]-scene['goals_x'][c])<=scene['arrival_position_tolerance_m'] and
               abs(cp['states'][c][1])<=scene['arrival_speed_tolerance_m_s'] for c in CARS):break
    completed=all(abs(cp['states'][c][0]-scene['goals_x'][c])<=scene['arrival_position_tolerance_m'] and
                  abs(cp['states'][c][1])<=scene['arrival_speed_tolerance_m_s'] for c in CARS)
    if completed:
        events.append(dict(kind='both_tasks_completed',absolute_time_s=cp['absolute_time_s'],states=cp['states']))
    elif trigger is None and failure is None:
        failure='navigation_timeout'
    directory.mkdir(parents=True,exist_ok=False)
    write_branch_logs(directory,config=dict(scene=scene,options=options),actual=rows,solves=records,
                      path_checks=paths,predictions=predictions,events=events)
    with (directory/'detections.jsonl').open('w') as stream:
        for d in detections:stream.write(json.dumps(d)+'\n')
    metrics=dict(compute_wall_s=time.perf_counter()-begun,primary_calls=len(records),failure=failure,
                 trigger=trigger,final_checkpoint=cp)
    write_json(directory/'summary.json',metrics)
    return metrics


def candidate_gap_metrics(directory):
    with (directory/'actual.csv').open() as stream:rows=list(csv.DictReader(stream))
    config=json.loads((directory/'config.json').read_text());cfg=Settings(**config['settings'])
    paired={}
    for r in rows:paired.setdefault(float(r['absolute_time_s']),{})[r['car']]=[float(r[k]) for k in ('x','vx','y')]
    executed=min(float(body_gap(np.array([p['A']]),np.array([p['B']]),cfg)[0]) for p in paired.values())
    predicted=[]
    for line in (directory/'predictions.jsonl').read_text().splitlines():
        p=json.loads(line);predicted.append(float(np.min(body_gap(np.array(p['states']['A']),np.array(p['states']['B']),cfg))))
    return dict(minimum_executed_gap_m=executed,minimum_predicted_gap_m=min(predicted) if predicted else None)


def assemble_execution(directory, scene, result):
    """One global-time log chain; candidate rehearsals remain separate."""
    parts=[directory/'navigation']
    if (directory/'execution').exists():parts.append(directory/'execution')
    rows=[];solves=[];checks=[];predictions=[];events=[]
    for part in parts:
        with (part/'actual.csv').open() as stream:rows.extend(csv.DictReader(stream))
        solves.extend(json.loads((part/'solves.json').read_text()))
        checks.extend(json.loads((part/'path_checks.json').read_text()))
        for name,out in (('predictions.jsonl',predictions),('events.jsonl',events)):
            out.extend(json.loads(line) for line in (part/name).read_text().splitlines())
    if result['failure'] and not any(e.get('failure')==result['failure'] for e in events):
        events.append(dict(kind='failure',failure=result['failure'],stage=result['status'],simulation_terminated=True,
                           verified_physical_safe_stop=False,
                           absolute_time_s=max(float(r['absolute_time_s']) for r in rows)))
    write_branch_logs(directory,config=dict(scene=scene,variant=result['variant'],original_goals=scene['goals_x']),
                      actual=rows,solves=solves,path_checks=checks,predictions=predictions,events=events)
    cfg=Settings(**scene['settings']);paired={}
    for row in rows:paired.setdefault(float(row['absolute_time_s']),{})[row['car']]=[float(row[k]) for k in ('x','vx','y')]
    actual_min=min(float(body_gap(np.array([p['A']]),np.array([p['B']]),cfg)[0]) for p in paired.values())
    prediction_min=min((float(np.min(body_gap(np.array(p['states']['A']),np.array(p['states']['B']),cfg))) for p in predictions),default=None)
    result.update(minimum_executed_gap_m=actual_min,minimum_predicted_gap_m=prediction_min,
                  minimum_executed_safety_margin_m=actual_min-cfg.margin,
                  minimum_predicted_safety_margin_m=prediction_min-cfg.margin if prediction_min is not None else None,
                  gap_definition=f'axis-aligned SAT body gap; safety margin = gap minus fixed {cfg.margin:g}m requirement',
                  continuous_safety_certified=False,simulation_terminated=bool(result['failure']))


def run_variant(directory, scene, options, recovery, variant):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=False);started=time.perf_counter()
    nav=run_navigation(scene,options,recovery,directory/'navigation')
    result=dict(variant=variant,detected=nav['trigger'] is not None,
                detected_at_s=nav['trigger']['absolute_time_s']-scene['settings']['epoch'] if nav['trigger'] else None,
                released=False,mutual_blocking_resolved=False,both_tasks_completed=False,exit_success=False,
                pass_success=False,role=None,role_switch_count=0,failure=nav['failure'],
                status='NAVIGATION_STOPPED',primary_calls=nav['primary_calls'],diagnostic_calls=0,retries=0,
                navigation_compute_wall_s=nav['compute_wall_s'],candidate_compute_wall_s=0.,switch_check_wall_s=0.)
    if nav['trigger'] is None or variant=='E0':
        result['status']='FAILED' if nav['failure'] else 'MUTUAL_BLOCKING_UNRESOLVED' if nav['trigger'] else 'NAVIGATION_STOPPED'
        final=nav['final_checkpoint']['states']
        if not nav['failure'] and all(abs(final[c][0]-scene['goals_x'][c])<=scene['arrival_position_tolerance_m'] and
                                     abs(final[c][1])<=scene['arrival_speed_tolerance_m_s'] for c in CARS):
            result.update(status='BOTH_TASKS_COMPLETED',both_tasks_completed=True)
    else:
        cp=copy.deepcopy(nav['final_checkpoint']);frozen=copy.deepcopy(cp)
        candidate_scene=copy.deepcopy(scene)
        candidate_scene['initial_states_x_v']={c:cp['states'][c][:2] for c in CARS}
        candidate_scene['duration_s']=min(scene['duration_s'],options['exit_timeout_s'],
            max(0.,recovery['total_timeout_s']-(cp['absolute_time_s']-scene['settings']['epoch'])))
        candidates={};begun=time.perf_counter()
        for c in (('A',) if variant=='E1' else CARS):
            path=directory/'candidates'/f'{c}_retreat'
            try:
                candidate=generate_exit(path,candidate_scene,c,solver_backend='osqp',checkpoint=copy.deepcopy(cp))
            except ValueError as error:
                # Unsupported start conditions are a recorded rejected attempt,
                # never a reason to reset measured velocity or change the map.
                path.mkdir(parents=True,exist_ok=True)
                candidate=dict(role=c,feasible=False,failure='candidate_input_rejected:'+str(error),
                               exit_time_s=None,retreat_distance_m=0.,primary_solve_calls=0,
                               solver_status='not_run_input_validation',physical_infeasibility_proved=False)
                initial=[dict(absolute_time_s=cp['absolute_time_s'],car=k,x=v[0],vx=v[1],y=v[2],
                    acceleration=cp['previous_acceleration'][k],stage='EXIT') for k,v in cp['states'].items()]
                write_branch_logs(path,config={**candidate_scene,'initial_checkpoint':cp},actual=initial,
                    solves=[],path_checks=[],predictions=[],events=[dict(kind='candidate_rejected',reason=candidate['failure'])])
                write_json(path/'summary.json',candidate)
            candidate.update(candidate_gap_metrics(path))
            candidates[c]=candidate
            result['primary_calls']+=candidate['primary_solve_calls']
        result['candidate_compute_wall_s']=time.perf_counter()-begun
        assert cp==frozen,'Counterfactual candidate generation changed actual state'
        eligible=[c for c,v in candidates.items() if v['feasible']]
        # Complete-exit duration asc, retreat distance asc, then fixed car ID.
        role=min(eligible,key=lambda c:(round(candidates[c]['exit_time_s'],9),round(candidates[c]['retreat_distance_m'],6),c)) if eligible else None
        write_json(directory/'selection.json',dict(role=role,candidates=candidates,
            ranking='accepted complete exits only; exit time asc (1e-9 s), retreat distance asc (1e-6 m), car ID asc',
            actual_checkpoint_before=frozen,actual_checkpoint_after=cp,
            candidate_tests_mutated_actual_state=False,used_pass_results=False,
            candidate_wall_s=result['candidate_compute_wall_s'],role_locked_until_release=True))
        result['role']=role
        if role is None:
            result.update(status='FAILED',failure='no_accepted_candidate')
        else:
            guard=TransitionGuard(candidate_scene,{**options,**recovery}) if variant=='E4' else None
            branch=run_branch(directory/'execution',directory/'candidates'/f'{role}_retreat',candidate_scene,variant,
                              {**options,**{k:recovery[k] for k in ('return_timeout_s','total_timeout_s')}},
                              result['candidate_compute_wall_s'],hold_constraints=True,return_to_tasks=variant in ('E3','E4'),transition_guard=guard)
            if guard is not None:
                with (directory/'switch_checks.jsonl').open('w') as stream:
                    for check in guard.records:stream.write(json.dumps(check)+'\n')
                result['switch_check_wall_s']=guard.wall_s
                result['switch_solver_calls']=guard.solver_calls
            result.update({k:branch[k] for k in ('status','failure','exit_success','pass_success','released','both_tasks_completed')})
            result['mutual_blocking_resolved']=branch['first_actual_body_cleared_time_s'] is not None
            result['primary_calls']+=sum(v for k,v in branch['phase_primary_calls'].items() if k!='EXIT')+branch.get('return_primary_calls',0)
            result['diagnostic_calls']=branch['diagnostic_calls']+(guard.solver_calls if guard is not None else 0)
            result['phase_compute_wall_s']=branch['phase_compute_wall_s']
            result['phase_primary_calls']=branch['phase_primary_calls']
            result['phase_diagnostic_calls']=branch['phase_diagnostic_calls']
            result['phase_retry_counts']=branch['phase_retry_counts']
            result['retreat_distance_m']=branch['retreat_distance_m']
            result['passing_distance_m']=branch['passing_distance_m']
            result['failure_phase']=branch.get('failure_phase')
            result['selection_wait_wall_s']=result['candidate_compute_wall_s']
            result['return_compute_wall_s']=branch.get('return_compute_wall_s',0.)
            result['initial_to_release_simulation_s']=branch['initial_to_release_simulation_s']
            result['initial_to_completion_simulation_s']=branch.get('initial_to_completion_simulation_s')
            result['exit_simulation_s']=branch['exit_simulation_s']
            result['pass_simulation_s']=branch['pass_simulation_s']
            result['return_simulation_s']=branch.get('return_simulation_s')
    assemble_execution(directory,scene,result)
    if not (directory/'switch_checks.jsonl').exists():(directory/'switch_checks.jsonl').write_text('')
    result['switch_guard_enabled']=variant=='E4'
    result['total_compute_wall_s']=time.perf_counter()-started
    write_json(directory/'summary.json',result)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=ROOT/'config/comparison.json')
    parser.add_argument('--options',type=Path,default=ROOT/'config/pass_release.json')
    parser.add_argument('--recovery',type=Path,default=ROOT/'config/recovery.json')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--variants',nargs='+',choices=('E0','E1','E2','E3','E4'),default=['E0','E1','E2','E3'])
    parser.add_argument('--smoke',action='store_true')
    args=parser.parse_args()
    base=json.loads(args.config.read_text());options=json.loads(args.options.read_text());recovery=json.loads(args.recovery.read_text())
    args.output.mkdir(parents=True,exist_ok=False)
    write_json(args.output/'study_config.json',dict(base=base,options=options,recovery=recovery,variants=args.variants))
    results=[]
    grid=itertools.product(*(base['sweep'][k] for k in ('encounter_x_m','channel_length_m','braking_m_s2')))
    for i,(encounter,length,braking) in enumerate(grid):
        scene=make_scene(base,encounter,0.,length,braking)
        for variant in args.variants:
            summary=run_variant(args.output/f'case_{i:03d}'/variant,scene,options,recovery,variant)
            results.append(dict(case=i,factors=scene['factors'],**summary))
            print(i,variant,summary['status'],summary['failure'],flush=True)
        if args.smoke:break
    write_json(args.output/'summary.json',results)
    columns=sorted(set(k for r in results for k,v in r.items() if not isinstance(v,dict)))
    with (args.output/'summary.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=columns);writer.writeheader()
        writer.writerows({k:r.get(k) for k in columns} for r in results)
    import scipy
    import osqp
    sources=list((ROOT/'scripts').glob('*.py'))+list((ROOT/'tests').glob('test_*.py'))+[args.config,args.options,args.recovery]
    write_json(args.output/'provenance.json',dict(source_sha256={str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        runtime=dict(python=sys.version,numpy=np.__version__,scipy=scipy.__version__,osqp=osqp.__version__,platform=platform.platform()),
        command=sys.argv,scope='offline full-rehearsal adaptation; not a Chen algorithm reproduction or theorem'))


if __name__=='__main__':main()
