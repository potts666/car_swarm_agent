#!/usr/bin/env python3
"""Re-read logs, independently replay component equations, recompute pair metrics."""
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from bicycle_core import CARS, SIGN, audit, clipped_area, completed, corners


def read_jsonl(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def load_rows(path):
    rows=list(csv.DictReader(path.open()))
    return {c:[r for r in rows if r['car']==c] for c in sorted({r['car'] for r in rows})}


def vectors(rows):
    return np.array([[float(r[k]) for k in ('x','y','psi','v','delta')] for r in rows])


def replay(z, inputs, dt, vehicle):
    """Independent scalar-component RK4 with two half steps, not the execution integrator."""
    def rhs(q,a,w):
        return np.array([q[3]*np.cos(q[2]),q[3]*np.sin(q[2]),
                         q[3]*np.tan(q[4])/vehicle['L'],a,w])
    q=z.copy();out=[q.copy()]
    for a,w in inputs:
        for _ in range(2):
            ds=dt/2
            k1=rhs(q,a,w);k2=rhs(q+ds*k1/2,a,w)
            k3=rhs(q+ds*k2/2,a,w);k4=rhs(q+ds*k3,a,w)
            q=q+ds*(k1+2*k2+2*k3+k4)/6
        out.append(q.copy())
    return np.array(out)


def check_series(rows, scene):
    z=vectors(rows);times=np.array([float(r['time_s']) for r in rows]);dt=scene['sample_s']
    inputs=np.array([[float(r['a']),float(r['omega'])] for r in rows[1:]])
    # Restart each interval at the recorded state so a reset cannot hide in accumulated errors.
    p=scene['vehicle'];pred=[]
    for q,u in zip(z[:-1],inputs):
        pred.append(replay(q,[u],dt,p)[-1])
    error=np.max(np.abs(np.array(pred)-z[1:]),axis=0)
    residual=dict(zip(('x_m','y_m','psi_rad','v_m_s','delta_rad'),map(float,error)))
    residual.update(time_s=float(np.max(np.abs(np.diff(times)-dt))),
        acceleration_m_s2=float(max(0.,inputs[:,0].max()-p['a_max'],p['a_min']-inputs[:,0].min())),
        steering_rate_rad_s=float(max(0.,np.abs(inputs[:,1]).max()-p['omega_max'])))
    sign_jump=bool(np.any(z[:-1,3]*z[1:,3] < -scene['tolerance']**2))
    body=corners(z,p)
    lengths=np.linalg.norm(body[:,1]-body[:,0],axis=1)
    widths=np.linalg.norm(body[:,2]-body[:,1],axis=1)
    residual['body_length_m']=float(np.max(np.abs(lengths-p['length'])))
    residual['body_width_m']=float(np.max(np.abs(widths-p['width'])))
    return dict(passed=all(v<=scene['tolerance'] for v in residual.values()) and not sign_jump,
                residuals=residual,gear_sign_jump=sign_jump)


def metrics(rows, scene, policy):
    per={};motions={c:vectors(rows[c]) for c in CARS};dt=scene['sample_s']
    for c in CARS:
        z=motions[c];v=(z[:-1,3]+z[1:,3])/2
        stages=[r['stage'] for r in rows[c][1:]]
        parked=(np.abs(z[1:,3])<=scene['terminal']['speed_m_s'])
        categories={'mutex_queue':{'MUTEX_WAIT','QUEUE_BRAKE','QUEUE_APPROACH'} if policy=='S1' else set(),
                    'mutual_blocking_detection':{'BLOCKED_HOLD'},
                    'retreat_park':{'RETREAT_PARK'},'recovery_peer_hold':{'WAIT_PEER'},
                    'maneuver_stops':{'ENTRY','RETURN'}}
        waiting={name:float(sum(stop and stage in names for stop,stage in zip(parked,stages))*dt)
                 for name,names in categories.items()}
        zone=([-28.,-18.,1.5,5.] if c=='A' else [18.,28.,-5.,-1.5])
        per[c]=dict(distance_m=float(np.linalg.norm(np.diff(z[:,:2],axis=0),axis=1).sum()),
            longitudinal_distance_m=float(np.abs(np.diff(z[:,0])).sum()),
            reverse_distance_m=float(np.maximum(0.,-v).sum()*dt),waiting_sim_s=waiting,
            stationary_per_stage_s={stage:float(sum(stop and s==stage for stop,s in zip(parked,stages))*dt) for stage in sorted(set(stages))},
            waiting_region_occupancy_s=float(sum(clipped_area(b,zone)>scene['tolerance'] for b in corners(z[1:],scene['vehicle']))*dt))
    channel=[-scene['D']/2,scene['D']/2,-scene['W']/2,scene['W']/2]
    occupied=np.zeros(len(motions['A'])-1,bool);entry={}
    for c,z in motions.items():
        present=np.array([clipped_area(b,channel)>scene['tolerance'] for b in corners(z[1:],scene['vehicle'])])
        occupied|=present
        indices=np.flatnonzero(present)
        entry[c]=float(rows[c][indices[0]+1]['time_s'])-scene['epoch_s'] if len(indices) else None
    return dict(per_car=per,total_distance_m=sum(v['distance_m'] for v in per.values()),
        total_reverse_distance_m=sum(v['reverse_distance_m'] for v in per.values()),
        channel_physically_occupied_s=float(occupied.sum()*dt),
        waiting_sim_s=sum(sum(v['waiting_sim_s'].values()) for v in per.values()),
        first_body_channel_entry_sim_s=entry,A_enters_first=entry['A'] is not None and entry['B'] is not None and entry['A']<entry['B'])


def verify_policy(directory, scene):
    rows=load_rows(directory/'actual.csv');motions={c:vectors(rows[c]) for c in CARS}
    check=audit(motions,scene)
    series={c:check_series(rows[c],scene) for c in CARS}
    summary=json.loads((directory/'summary.json').read_text());events=read_jsonl(directory/'events.jsonl')
    predictions=read_jsonl(directory/'predictions.jsonl');n=round(scene['h']/scene['sample_s'])
    prediction_passed=True;minimum_prediction_gap=math.inf;prediction_max_error=0.
    actual_at={c:{round(float(r['time_s']),8):q for r,q in zip(rows[c],motions[c])} for c in CARS}
    for f in predictions:
        future={}
        for c in CARS:
            stamp=round(f['time_s'],8)
            prediction_passed &= stamp in actual_at[c] and np.max(np.abs(np.array(f['states'][c])-actual_at[c].get(stamp,np.full(5,np.inf))))<=scene['tolerance']
            dense=np.repeat(np.array(f['commands'][c],dtype=object)[:,:2].astype(float),n,axis=0)
            prediction_passed &= bool(np.isfinite(dense).all() and dense[:,0].max()<=scene['vehicle']['a_max']+scene['tolerance'] and dense[:,0].min()>=scene['vehicle']['a_min']-scene['tolerance'] and np.abs(dense[:,1]).max()<=scene['vehicle']['omega_max']+scene['tolerance'])
            future[c]=replay(np.array(f['states'][c]),dense,scene['sample_s'],scene['vehicle'])
            prediction_max_error=max(prediction_max_error,float(np.max(np.abs(future[c][n::n]-f['predicted_states'][c]))))
        a=audit(future,scene);prediction_passed &= a['passed']
        minimum_prediction_gap=min(minimum_prediction_gap,a['minimum_gap_m'])
    owner=None;protocol_errors=[];authorization_passed=True;release_passed=True
    for e in events:
        if e['kind']=='grant':
            if owner is not None:
                protocol_errors.append('grant while already owned')
            owner=e['car']
        elif e['kind']=='release':
            if summary['policy']=='S1':
                if owner!=e['car']:
                    protocol_errors.append('release from non-owner')
                owner=None
            stamp=e['time_s'];c=e['car']
            match=[r for r in rows[c] if abs(float(r['time_s'])-stamp)<1e-7]
            if not match:
                release_passed=False;continue
            z=vectors(match)[0]
            release_passed &= SIGN[c]*z[0]-scene['vehicle']['rear'] >= scene['release_plane_abs_x']+scene['release_margin_m']-scene['tolerance']
            release_passed &= abs(z[1])<=scene['tolerance'] and abs(z[2]-scene['initial'][c][2])<=scene['tolerance']
            witnesses=read_jsonl(directory/f'release_witness_{c}.jsonl')
            w=witnesses[0];count=max(len(v) for v in w['commands'].values());future={}
            for vehicle in CARS:
                commands=w['commands'][vehicle]+[[0.,0.,'HOLD']]*max(0,count-len(w['commands'][vehicle]))
                inputs=np.repeat(np.array(commands,dtype=object)[:,:2].astype(float),n,axis=0)
                future[vehicle]=replay(np.array(w['states'][vehicle]),inputs,scene['sample_s'],scene['vehicle'])
            release_passed &= audit(future,scene)['passed'] and all(completed(future[c][-1],scene['goals'][c],scene) for c in CARS)
    if summary['policy']=='S1':
        # Each sample's owner is the one in force during the preceding integrated interval.
        protection=[-scene['admission_protection_abs_x'],scene['admission_protection_abs_x'],-scene['W']/2,scene['W']/2]
        for c in CARS:
            bodies=corners(motions[c],scene['vehicle'])
            for r,b in zip(rows[c][1:],bodies[1:]):
                if r['owner']!=c and clipped_area(b,protection)>scene['tolerance']:
                    authorization_passed=False
    terminals={c:completed(motions[c][-1],scene['goals'][c],scene) for c in CARS}
    reported_success_consistent=summary['both_completed']==all(terminals.values())
    initial_same=all(np.max(np.abs(motions[c][0]-scene['initial'][c]))<=scene['tolerance'] for c in CARS)
    paired_times=np.array([float(r['time_s']) for r in rows['A']])-np.array([float(r['time_s']) for r in rows['B']])
    m=metrics(rows,scene,summary['policy'])
    messages=read_jsonl(directory/'messages.jsonl')
    message_bytes_valid=all(len(json.dumps({k:r[k] for k in ('sender','receiver','kind','payload')},
        separators=(',',':'),ensure_ascii=False).encode('utf-8'))==r['payload_bytes'] for r in messages)
    m['communication']=dict(messages=len(messages),payload_bytes=sum(r['payload_bytes'] for r in messages),by_type={},
        definition='estimated logical protocol messages; compact UTF-8 JSON envelope+payload; no actual network measurement')
    for r in messages:
        bucket=m['communication']['by_type'].setdefault(r['kind'],dict(count=0,payload_bytes=0))
        bucket['count']+=1;bucket['payload_bytes']+=r['payload_bytes']
    planning=json.loads((directory/'planning.json').read_text());checks=read_jsonl(directory/'checks.jsonl')
    timing={}
    for name,values in [('main_planning',[r['planning_wall_s'] for r in planning]),
                        ('cycle_total',[r['total_cycle_wall_s'] for r in planning]),
                        ('diagnostic',[r['wall_s'] for r in checks])]:
        timing[name]=dict(count=len(values),total_s=sum(values),mean_s=float(np.mean(values)) if values else 0.,
            max_s=max(values,default=0.),over_h=sum(v>scene['h'] for v in values))
    selection_path=directory/'selection.json';candidate_passed=True
    if selection_path.exists():
        s=json.loads(selection_path.read_text());values=[v['wall_s'] for v in s['candidates'].values()]
        timing['candidate']=dict(count=len(values),total_s=sum(values),mean_s=float(np.mean(values)),max_s=max(values),over_h=sum(v>scene['h'] for v in values))
        for c,original in s['candidates'].items():
            candidate_rows=load_rows(directory/'candidates'/c/'actual.csv')
            actual={vehicle:vectors(candidate_rows[vehicle]) for vehicle in CARS}
            accepted=audit(actual,scene)['passed'] and all(check_series(candidate_rows[k],scene)['passed'] for k in CARS)
            candidate_passed &= bool(accepted)==original['accepted']
            candidate_passed &= all(np.max(np.abs(actual[k][0]-s['checkpoint'][k]))<scene['tolerance'] and abs(float(candidate_rows[k][0]['time_s'])-s['time_s'])<scene['tolerance'] for k in CARS)
        eligible=[c for c,v in s['candidates'].items() if v['accepted']]
        expected=min(eligible,key=lambda c:(round(s['candidates'][c]['exit_time_s'],9),round(s['candidates'][c]['retreat_distance_m'],6),c)) if eligible else None
        candidate_passed &= expected==s['role']
    m['timing']=timing
    m['channel_reserved_s_recomputed']=sum(bool(r['owner']) for r in rows['A'][1:])*scene['sample_s']
    reservation_consistent=abs(m['channel_reserved_s_recomputed']-summary['channel_reserved_s'])<=scene['tolerance']
    candidate_passed &= all(abs(d['planning_stamp_s']-d['absolute_time_s'])<1e-8 for d in read_jsonl(directory/'detections.jsonl') if 'absolute_time_s' in d)
    completion={c:next((float(r['time_s'])-scene['epoch_s'] for r in rows[c] if completed(vectors([r])[0],scene['goals'][c],scene)),None) for c in CARS}
    # Events use queue exhaustion as the actual task-completion tick, bounded by common tolerances.
    completion_consistent=all(abs(float(next(e['time_s'] for e in events if e['kind']=='car_completed' and e['car']==c))-scene['epoch_s']-summary['completion_per_car_s'][c])<=1e-7 for c in summary['completion_per_car_s'])
    m['first_terminal_tolerance_entry_s']=completion
    if summary['both_completed']:
        completion_consistent &= abs(summary['T_complete_sim_s']-max(summary['completion_per_car_s'].values()))<=scene['tolerance']
    results=dict(passed=bool(check['passed'] and all(v['passed'] for v in series.values()) and prediction_passed and
        prediction_max_error<=scene['tolerance'] and initial_same and not protocol_errors and authorization_passed and release_passed and
        candidate_passed and reported_success_consistent and reservation_consistent and message_bytes_valid and np.max(np.abs(paired_times))<=1e-8 and completion_consistent),
        trajectory=check,component_equations=series,predictions_passed=bool(prediction_passed),
        minimum_replayed_prediction_gap_m=minimum_prediction_gap,prediction_state_max_error=prediction_max_error,
        initial_same=initial_same,protocol_errors=protocol_errors,unauthorized_entry_absent=authorization_passed,
        release_full_body_and_future_passed=bool(release_passed),candidate_replay_passed=bool(candidate_passed),
        reported_success_consistent=reported_success_consistent,terminal=terminals,message_bytes_valid=message_bytes_valid,metrics=m)
    (directory/'recomputed_metrics.json').write_text(json.dumps(m,indent=2)+'\n')
    return results


def verify(directory):
    directory=Path(directory);scene=json.loads((directory/'scene.json').read_text())
    singles={}
    for path in (directory/'single_vehicle').glob('*.csv'):
        if path.name.startswith('original_map'):
            continue
        rows=load_rows(path)['A'];singles[path.stem]=dict(equations=check_series(rows,scene),road=audit({'A':vectors(rows)},scene))
    policies={p:verify_policy(directory/p,scene) for p in ('S1','S2')}
    equal=all(json.loads((directory/p/'config.json').read_text())['scene']==scene for p in ('S1','S2'))
    passed=equal and all(v['passed'] for v in policies.values()) and all(v['equations']['passed'] and v['road']['passed'] for v in singles.values())
    return dict(passed=passed,serialized_scene_identical=equal,single_vehicle=singles,policies=policies,
                scope='dense 0.02s sampled replay; no continuous-time safety proof; successful completion is a separate outcome')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,required=True);args=parser.parse_args()
    result=verify(args.run);(args.run/'validation.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(passed=result['passed'],policies={k:v['passed'] for k,v in result['policies'].items()})))
    raise SystemExit(0 if result['passed'] else 1)
