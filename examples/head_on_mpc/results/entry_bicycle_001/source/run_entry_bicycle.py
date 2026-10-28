#!/usr/bin/env python3
"""One shared-scene offline prevention/recovery pair, with bicycle execution."""
import argparse
from collections import deque
import copy
import csv
import hashlib
import json
import platform
from pathlib import Path
import subprocess
import time

import numpy as np

from bicycle_core import (CARS, SIGN, arbitrate, audit, bay_entry, clipped_area,
    completed, corners, drive, return_plan, rollout, shift, shift_advance)
from recovery_core import BlockingDetector

ROOT = Path(__file__).resolve().parents[1]


def write(path, value):
    path.write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n')


def jsonl(path, values):
    with path.open('w') as stream:
        for v in values:
            stream.write(json.dumps(v,ensure_ascii=False,allow_nan=False)+'\n')


def rows_for(motions, controls, start, scene, owner=None, fallback=None):
    rows = []
    n = round(scene['h']/scene['sample_s'])
    for c,motion in motions.items():
        for j,z in enumerate(motion):
            if j == 0 and start > scene['epoch_s']+1e-8:
                continue
            command = controls[c][min(max(0,(j-1)//n),len(controls[c])-1)] if controls[c] else (0.,0.,fallback or 'HOLD')
            rows.append(dict(time_s=round(start+j*scene['sample_s'],10),car=c,
                **dict(zip(('x','y','psi','v','delta'),map(float,z))),
                a=float(command[0]) if j else 0.,omega=float(command[1]) if j else 0.,
                stage=command[2],owner=owner or ''))
    return sorted(rows,key=lambda r:(r['time_s'],r['car']))


def save_rows(path, rows):
    with path.open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)


def single_vehicle(scene, directory):
    directory.mkdir()
    cases = {
        'approach_stop': (scene['initial']['A'],drive(6.,scene,'APPROACH')),
        'waiting_entry': ([-30.,0.,0.,0.,0.],shift(scene)),
        'reverse_exit': ([0.,0.,0.,0.,0.],drive(-30.,scene,'RETREAT')),
    }
    parked=rollout(cases['waiting_entry'][0],cases['waiting_entry'][1],scene)[-1]
    cases['waiting_return']=(parked,shift(scene,returning=True,stage='RETURN'))
    checks={}
    for name,(z,commands) in cases.items():
        motions={'A':rollout(z,commands,scene)}
        check=audit(motions,scene)
        check['final_state']=motions['A'][-1].tolist()
        check['duration_s']=len(commands)*scene['h']
        check['controls_within_limits']=all(scene['vehicle']['a_min']-1e-10<=a<=scene['vehicle']['a_max']+1e-10 and
            abs(w)<=scene['vehicle']['omega_max']+1e-10 for a,w,_ in commands)
        check['passed'] &= check['controls_within_limits']
        checks[name]=check
        save_rows(directory/f'{name}.csv',rows_for(motions,{'A':commands},scene['epoch_s'],scene))
    # Retain a concrete rejected maneuver on the original map; this is not an impossibility proof.
    from run_comparison import make_scene
    base=json.loads((ROOT/'config/scenarios.json').read_text())
    original=make_scene(base,0.,0.,20.,1.)
    road=original['road'];rects=[[road['xmin'],road['xmax'],road['ymin'],road['ymax']]]
    rects += [[b['xmin'],b['xmax'],road['ymin'],b['ymax']] for b in road['bays']]
    old=copy.deepcopy(scene);old['road_rectangles']=rects
    rear=original['retreat_x']['A']-(scene['vehicle']['L']+scene['vehicle']['front']-scene['vehicle']['rear'])/2
    z=[rear-shift_advance(scene),0.,0.,0.,0.]
    motions={'A':rollout(z,shift(scene),old)}
    rejected=audit(motions,old)
    rejected.update(evidence='this stop/steer two-arc primitive leaves the original road; no exhaustive search, no physical impossibility proof',
                    original_scene=original,initial_state=z)
    write(directory/'original_map_rejected_maneuver.json',rejected)
    save_rows(directory/'original_map_rejected_maneuver.csv',rows_for(motions,{'A':shift(scene)},scene['epoch_s'],old))
    write(directory/'checks.json',checks)
    return all(v['passed'] for v in checks.values())


def detector_for(scene):
    adapter=dict(settings=dict(dt=scene['h'],horizon=scene['N'],substeps=1),
        goals_x={c:g[0] for c,g in scene['goals'].items()},
        arrival_position_tolerance_m=scene['terminal']['position_m'],
        arrival_speed_tolerance_m_s=scene['terminal']['speed_m_s'])
    return BlockingDetector(adapter,scene['detector'])


def full_pair(states, queues, scene):
    count=max(map(len,queues.values()))
    return {c:rollout(states[c],queues[c],scene,cycles=count) for c in CARS}


def candidate(states, car, scene, directory):
    directory.mkdir(parents=True)
    started=time.perf_counter()
    commands=bay_entry(states[car],car,scene,recovery=True)
    motions=full_pair(states,{c:commands if c==car else [] for c in CARS},scene)
    check=audit(motions,scene)
    expected=np.array([-SIGN[car]*(scene['waiting_anchor_abs_x']-shift_advance(scene)),
                       SIGN[car]*scene['bay_offset_local_y'],scene['initial'][car][2],0.,0.])
    check['exit_terminal_error']=float(np.abs(motions[car][-1]-expected).max())
    check['passed'] &= check['exit_terminal_error']<=scene['tolerance']
    distance=float(np.sum(np.linalg.norm(np.diff(motions[car][:,:2],axis=0),axis=1)))
    result=dict(car=car,accepted=bool(check['passed']),exit_time_s=len(commands)*scene['h'],
                retreat_distance_m=float(np.sum(np.maximum(0.,-motions[car][1:,3]))*scene['sample_s']),
                distance_m=distance,checks=check,initial={c:z.tolist() for c,z in states.items()},
                final={c:z[-1].tolist() for c,z in motions.items()},commands=commands,
                solver_calls=0,retries=0)
    save_rows(directory/'actual.csv',rows_for(motions,{c:commands if c==car else [(0.,0.,'WAIT_PEER')]*len(commands)
              for c in CARS},scene['epoch_s'],scene))
    result['wall_s']=time.perf_counter()-started
    write(directory/'summary.json',result)
    return result


def run_policy(scene, policy, directory):
    directory.mkdir()
    started=time.perf_counter();states={c:np.array(scene['initial'][c],float) for c in CARS}
    events=[];messages=[];forecasts=[];checks=[];planning=[];rows=[];detections=[]
    owner=None;requested=set();done={};role=None;mode='NAVIGATE';failure=None
    selection_wall=0.;selection=None;detector=detector_for(scene)
    midpoint=(states['A'][0]+states['B'][0])/2
    collision_targets={'A':midpoint-(scene['vehicle']['L']+scene['vehicle']['front'])-scene['navigation_gap_m']/2,
                       'B':midpoint+(scene['vehicle']['L']+scene['vehicle']['front'])+scene['navigation_gap_m']/2}
    queues={c:deque(drive(SIGN[c]*((scene['goals'][c][0] if policy=='S1' else collision_targets[c])-states[c][0]),scene,'NAVIGATE')) for c in CARS}
    motion_build_wall=time.perf_counter()-started
    status={c:'NAVIGATE' for c in CARS};reserved_s=0.;now=scene['epoch_s']

    def event(kind, **details):
        events.append(dict(time_s=round(now,10),kind=kind,**details))

    def message(sender,receiver,kind,payload):
        packet=dict(sender=sender,receiver=receiver,kind=kind,payload=payload)
        encoded=json.dumps(packet,separators=(',',':'),ensure_ascii=False).encode('utf-8')
        messages.append(dict(time_s=round(now,10),**packet,payload_bytes=len(encoded),
                             measurement='single-process logical protocol estimate; UTF-8 compact JSON, no transport headers'))

    def grant_requests(requests):
        nonlocal owner
        owner,grants=arbitrate(owner,requests)
        for stamp,c in requests:
            event('request',car=c,request_stamp_s=stamp)
            message(c,'manager','request',dict(car=c,stamp_s=stamp))
            event('grant' if grants[c] else 'deny',car=c,owner=owner)
            message('manager',c,'grant' if grants[c] else 'deny',dict(owner=owner))
            if not grants[c]:
                queues[c]=deque(bay_entry(states[c],c,scene));status[c]='QUEUE_MANEUVER'
            elif status[c]=='MUTEX_WAIT':
                queues[c]=deque(return_plan(states[c],c,scene));status[c]='RETURNING'

    for cycle in range(round(scene['timeout_s']/scene['h'])):
        now=scene['epoch_s']+cycle*scene['h'];cycle_started=time.perf_counter()
        if policy=='S1':
            requests=[(now,c) for c in CARS if c not in requested and SIGN[c]*states[c][0]>=-scene['request_rear_axle_abs_x']-1e-8]
            requested.update(c for _,c in requests)
            if requests:
                grant_requests(requests)
        for c in CARS:
            if not queues[c] and completed(states[c],scene['goals'][c],scene) and c not in done:
                done[c]=now-scene['epoch_s'];status[c]='COMPLETE';event('car_completed',car=c)
            elif not queues[c] and status[c]=='QUEUE_MANEUVER':
                status[c]='MUTEX_WAIT';event('waiting_ready',car=c,pose=states[c].tolist())
        if policy=='S2' and mode=='EXIT' and not queues[role]:
            status[role]='RETREAT_PARK';passer='B' if role=='A' else 'A'
            queues[passer]=deque(drive(SIGN[passer]*(scene['goals'][passer][0]-states[passer][0]),scene,'PASS'))
            mode='PASS';status[passer]='PASS';event('exit_complete',car=role);event('pass_started',car=passer)
        holder=owner if policy=='S1' else ('B' if role=='A' else 'A') if mode=='PASS' else None
        release_guard=(holder is not None and SIGN[holder]*states[holder][0]-scene['vehicle']['rear'] >=
                       scene['release_plane_abs_x']+scene['release_margin_m'])
        if release_guard:
            waiting='B' if holder=='A' else 'A'
            future={c:list(queues[c]) for c in CARS}
            if status[waiting] in ('MUTEX_WAIT','RETREAT_PARK'):
                future[waiting]=return_plan(states[waiting],waiting,scene)
            diagnostic_started=time.perf_counter()
            witness=full_pair(states,future,scene)
            proof=audit(witness,scene)
            # After the full maneuver, each car stays at its distinct original goal.
            proof['both_future_goals']=all(completed(witness[c][-1],scene['goals'][c],scene) for c in CARS)
            proof['passed'] &= proof['both_future_goals']
            checks.append(dict(time_s=now,kind='complete_future_release_witness',wall_s=time.perf_counter()-diagnostic_started,**proof))
            if proof['passed']:
                event('release',car=holder,actual_body_tail_x=float(states[holder][0]-SIGN[holder]*scene['vehicle']['rear']),
                      release_plane_abs_x=scene['release_plane_abs_x'],future_check_index=len(checks)-1,
                      future_assumption='execute accepted full remaining paths then stay at distinct goals; no extra reversal')
                jsonl(directory/f'release_witness_{holder}.jsonl',[dict(states={c:states[c].tolist() for c in CARS},commands=future,
                    final={c:witness[c][-1].tolist() for c in CARS},check=proof)])
                if policy=='S1':
                    message(holder,'manager','cleared',dict(car=holder));owner=None
                    message('manager',waiting,'release',dict(previous_owner=holder))
                    if status[waiting]=='MUTEX_WAIT':
                        grant_requests([(now,waiting)])
                else:
                    mode='RETURN';queues[role]=deque(future[role]);status[role]='RETURNING'
                    for c in CARS:
                        message('manager',c,'release_and_return',dict(role=role))
                if status[waiting]=='RETURNING':
                    event('return_started',car=waiting)
        if len(done)==2:
            event('both_tasks_completed');break
        controls={c:list(queues[c])[:scene['N']] for c in CARS}
        for c in CARS:
            fallback=status[c] if status[c]!='NAVIGATE' else 'BLOCKED_HOLD'
            controls[c]+=[(0.,0.,fallback)]*max(0,scene['N']-len(controls[c]))
        forecast={c:rollout(states[c],controls[c],scene,cycles=scene['N']) for c in CARS}
        plan_wall=time.perf_counter()-cycle_started
        check_started=time.perf_counter();check=audit(forecast,scene)
        check_wall=time.perf_counter()-check_started
        checks.append(dict(time_s=now,kind='dense_prediction',wall_s=check_wall,**check))
        n=round(scene['h']/scene['sample_s'])
        forecasts.append(dict(time_s=now,states={c:states[c].tolist() for c in CARS},
            commands=controls,predicted_states={c:forecast[c][n::n].tolist() for c in CARS},owner=owner))
        if not check['passed']:
            failure='prediction_acceptance_rejected';event('failure',reason=failure);break
        if policy=='S2' and mode=='NAVIGATE':
            current={c:np.array([states[c][0],SIGN[c]*states[c][3]]) for c in CARS}
            predicted={c:np.column_stack((forecast[c][n::n,0],SIGN[c]*forecast[c][n::n,3],forecast[c][n::n,1])) for c in CARS}
            blocked={c:check['minimum_gap_m'] <= scene['navigation_gap_m']+scene['tolerance'] and
                SIGN[c]*(scene['goals'][c][0]-states['B' if c=='A' else 'A'][0])>0 for c in CARS}
            observation=detector.observe(now,current,predicted,True,blocked=blocked)
            observation['planning_stamp_s']=now;detections.append(observation)
            if observation['triggered']:
                event('mutual_blocking_detected',evidence=observation)
                frozen={c:z.copy() for c,z in states.items()};begun=time.perf_counter()
                candidates={c:candidate(copy.deepcopy(states),c,scene,directory/'candidates'/c) for c in CARS}
                selection_wall=time.perf_counter()-begun
                assert all(np.array_equal(states[c],frozen[c]) for c in CARS)
                eligible=[c for c in CARS if candidates[c]['accepted']]
                role=min(eligible,key=lambda c:(round(candidates[c]['exit_time_s'],9),round(candidates[c]['retreat_distance_m'],6),c)) if eligible else None
                selection=dict(role=role,candidates=candidates,selection_wait_wall_s=selection_wall,
                    simulation_frozen_during_selection=True,checkpoint_unchanged=True,
                    checkpoint={c:z.tolist() for c,z in states.items()},time_s=now,
                    ranking='accepted complete exit duration, reverse distance, car ID; same rule as run_recovery E3/E4')
                write(directory/'selection.json',selection)
                if role is None:
                    failure='no_accepted_candidate';event('failure',reason=failure);break
                event('role_selected',role=role,selection_wait_wall_s=selection_wall)
                for c in CARS:
                    message('manager',c,'role_assignment',dict(retreat=role,locked_until_release=True))
                queues[role]=deque(candidates[role]['commands']);queues['B' if role=='A' else 'A']=deque()
                status[role]='EXIT';status['B' if role=='A' else 'A']='WAIT_PEER';mode='EXIT'
                # Same simulation instant; generate fresh candidate-mode prediction below.
                controls={c:list(queues[c])[:scene['N']] for c in CARS}
                for c in CARS:
                    controls[c]+=[(0.,0.,status[c])]*max(0,scene['N']-len(controls[c]))
                forecast={c:rollout(states[c],controls[c],scene,cycles=scene['N']) for c in CARS}
                fresh_started=time.perf_counter();fresh=audit(forecast,scene)
                checks.append(dict(time_s=now,kind='post_selection_prediction',wall_s=time.perf_counter()-fresh_started,**fresh))
                forecasts.append(dict(time_s=now,states={c:states[c].tolist() for c in CARS},commands=controls,
                    predicted_states={c:forecast[c][n::n].tolist() for c in CARS},owner=None))
                if not fresh['passed']:
                    failure='post_selection_prediction_rejected';event('failure',reason=failure);break
        if policy=='S2':
            for c in CARS:
                message(c,'manager','current_prediction',dict(stamp_s=now,states=forecast[c][n::n].tolist()))
        motions={c:forecast[c][:n+1] for c in CARS}
        check_started=time.perf_counter();executed=audit(motions,scene)
        checks.append(dict(time_s=now,kind='executed_segment',wall_s=time.perf_counter()-check_started,**executed))
        if not executed['passed']:
            failure='actual_acceptance_rejected';event('failure',reason=failure);break
        if policy=='S1':
            for c in CARS:
                if owner!=c:
                    unauthorized=any(clipped_area(body,[-scene['admission_protection_abs_x'],scene['admission_protection_abs_x'],-scene['W']/2,scene['W']/2])>scene['tolerance']
                                     for body in corners(motions[c],scene['vehicle']))
                    # After a documented release the former holder continues monotonically away.
                    released=any(e['kind']=='release' and e['car']==c for e in events)
                    if unauthorized and not released:
                        failure='unauthorized_protection_entry';event('failure',reason=failure,car=c);break
        if failure:
            break
        rows.extend(rows_for(motions,{c:[controls[c][0]] for c in CARS},now,scene,owner))
        for c in CARS:
            states[c]=motions[c][-1]
            if queues[c]:
                previous_stage=queues[c].popleft()[2]
                if queues[c] and queues[c][0][2]!=previous_stage:
                    event('motion_stage_change',car=c,previous=previous_stage,next=queues[c][0][2],effective_time_s=now+scene['h'])
        reserved_s+=scene['h']*(owner is not None)
        planning.append(dict(time_s=now,planning_wall_s=plan_wall,acceptance_wall_s=check_wall,
            total_cycle_wall_s=time.perf_counter()-cycle_started,optimization_solver_calls=0,retries=0))
    else:
        failure='timeout';now=scene['epoch_s']+scene['timeout_s'];event('failure',reason=failure)
    if not rows:
        rows=rows_for({c:np.array([states[c]]) for c in CARS},{c:[] for c in CARS},scene['epoch_s'],scene)
    save_rows(directory/'actual.csv',rows)
    jsonl(directory/'events.jsonl',events);jsonl(directory/'predictions.jsonl',forecasts)
    jsonl(directory/'checks.jsonl',checks);jsonl(directory/'messages.jsonl',messages);jsonl(directory/'detections.jsonl',detections)
    write(directory/'planning.json',planning);write(directory/'config.json',dict(scene=scene,policy=policy))
    summary=dict(policy=policy,status='both_completed' if len(done)==2 and failure is None else failure or 'incomplete',
        both_completed=len(done)==2 and failure is None,T_complete_sim_s=max(done.values()) if len(done)==2 and failure is None else None,
        completion_per_car_s=done,stop_time_sim_s=now-scene['epoch_s'],failure=failure,role=role,
        selection_wait_wall_s=selection_wall,simulation_frozen_during_selection=True,
        initial_motion_build_wall_s=motion_build_wall,channel_reserved_s=reserved_s,
        main_planning_calls=len(planning),optimization_solver_calls=0,retries=0,
        candidate_count=2 if selection else 0,diagnostic_calls=len(checks),
        minimum_actual_gap_m=min((v['minimum_gap_m'] for v in checks if v['kind']=='executed_segment'),default=None),
        minimum_predicted_gap_m=min((v['minimum_gap_m'] for v in checks if 'prediction' in v['kind']),default=None),
        continuous_time_safety_proved=False,computation_mode='offline',
        total_policy_wall_s=time.perf_counter()-started)
    write(directory/'summary.json',summary)
    return summary


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=ROOT/'config/entry_bicycle.json')
    args=parser.parse_args();begun=time.perf_counter()
    scene=json.loads(args.config.read_text());args.output.mkdir(parents=True,exist_ok=False)
    write(args.output/'scene.json',scene)
    (args.output/'variables.md').write_text((ROOT/'entry_bicycle_variables.md').read_text())
    sources=[*Path(__file__).parent.glob('*bicycle*.py'),ROOT/'tests/test_entry_bicycle.py',args.config,
             Path(__file__).with_name('recovery_core.py')]
    snapshot=args.output/'source';snapshot.mkdir()
    hashes={}
    for path in sources:
        data=path.read_bytes();(snapshot/path.name).write_bytes(data);hashes[str(path.relative_to(ROOT))]=hashlib.sha256(data).hexdigest()
    git=subprocess.run(['git','rev-parse','HEAD'],capture_output=True,text=True,check=True).stdout.strip()
    write(args.output/'provenance.json',dict(git_head=git,sha256=hashes,python=platform.python_version(),numpy=np.__version__,
        command='python3 scripts/run_entry_bicycle.py --output '+str(args.output),hardware_calibrated=False))
    initial=audit({c:np.array([scene['initial'][c]]) for c in CARS},scene)
    write(args.output/'initial_checks.json',initial)
    if not initial['passed'] or not single_vehicle(scene,args.output/'single_vehicle'):
        write(args.output/'summary.json',dict(status='single_vehicle_gate_failed',performance_conclusions_allowed=False))
        raise SystemExit('single-vehicle gate failed; no paired performance claims')
    results={}
    for policy in ('S1','S2'):
        print('Running',policy,flush=True)
        results[policy]=run_policy(scene,policy,args.output/policy)
        print(json.dumps(results[policy]),flush=True)
    write(args.output/'summary.json',dict(results=results,total_process_wall_s=time.perf_counter()-begun))
    from verify_entry_bicycle import verify
    verification=verify(args.output)
    write(args.output/'validation.json',verification)
    from report_entry_bicycle import render_report
    render_report(args.output)
    print('Independent verification:',verification['passed'],flush=True)
    if not verification['passed']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
