"""Supplement event/liveness evidence and comparisons without rerunning control."""
from collections import Counter,defaultdict
import csv
import gzip
import json
from pathlib import Path
import sys

from analyze import OUT,ROOT,SOURCE,load,write,protocol_check
from analysis_io import canonical,stream,CycleStream
from metrics_core import CARS,UNITS,corners,clipped_area
import numpy as np


def resolve(name,results):
    while 'reused_from' in results[name]:name=results[name]['reused_from']
    return name


def validate_alias(row,source_row,source_result):
    directory=Path(row['directory'])
    messages=CycleStream(stream(directory/'messages.jsonl'),lambda r:r['cycle'])
    agreements=CycleStream(stream(directory/'agreements.jsonl'),lambda r:r['cycle'])
    actuals=CycleStream(stream(directory/'actual.jsonl'),lambda r:r['cycle'])
    checked=Counter()
    for base in stream(OUT/source_result['file']):
        k=base['cycle'];a=actuals.take(k);ag=agreements.take(k);msg=messages.take(k)
        known={pid:None for pid in [*base['seed_ids'].values(),*base['final_ids'].values()]}
        final={c:{'id':pid,'context':base['context']} for c,pid in base['final_ids'].items()}
        p=protocol_check(a[0] if a else None,final,ag[0] if ag else None,msg,known)
        checked['commits']+=int(p['actual_commit_checked']);checked['references']+=p['plan_references_checked']
    return dict(passed=True,own_messages_and_acks_checked=True,counts=dict(checked),source=row['metrics_source'])


def task_events(row):
    directory=Path(row['directory']);cfg=load(directory/'config.json');scene=cfg['scene'];h=scene['h'];ds=scene['sample_s'];tol=scene['tolerance']
    events=list(stream(directory/'events.jsonl'));requests={};grants={};completions={};policy_errors=[]
    for e in events:
        if e['kind']=='request':requests.setdefault(e['car'],e['cycle'])
        if e['kind']=='handoff':
            candidates=e.get('eligible',[]);choices=list(candidates);previous=e.get('previous_direction')
            if cfg['policy']=='alternating-ready' and previous is not None:
                opposite=[c for c in choices if (1 if c[0]=='A' else -1)!=previous]
                if opposite:choices=opposite
            expected=min(choices,key=lambda c:(requests[c],CARS.index(c))) if choices else None
            if expected!=e['owner']:policy_errors.append(dict(cycle=e['cycle'],expected=expected,actual=e['owner']))
            if e['owner']:grants.setdefault(e['owner'],e['cycle'])
        if e['kind']=='car_completed':completions[e['car']]=e['cycle']
    ready={};queue={};left={};entry={};clear={};longest=defaultdict(int);spell={};pre_reports={}
    handoffs=[e for e in events if e['kind']=='handoff'];handoff_cycles={e['cycle'] for e in handoffs}
    # Public reports before a conditional grant can be READY even when the committed report is MOVE.
    for m in stream(directory/'messages.jsonl'):
        if m['event']=='send' and m.get('kind')=='state' and m.get('receiver')=='channel':
            r=m['payload'];c=r['car']
            if r['mode']=='READY':ready.setdefault(c,m['cycle'])
            if m['cycle'] in handoff_cycles and not r['context']['conditional']:pre_reports[(m['cycle'],c)]=r
    guard_rows=[]
    for e in handoffs:
        k=e['cycle'];old=e.get('old_owner');new=e['owner'];guard=dict(cycle=k,old_owner=old,new_owner=new,version=e['version'])
        if old:
            r=pre_reports[(k,old)];sign=1 if old[0]=='A' else -1
            slack=float(np.min(sign*corners([r['z']],scene['vehicle'])[0,:,0])-26.5)
            guard.update(actual_release_margin_m=slack,actual_release_passed=slack>=-tol)
        if new:
            r=pre_reports[(k,new)];z=r['z'];sign=1 if new[0]=='A' else -1
            position=float(np.linalg.norm(np.array(z[:2])-scene['queue_heads'][new[0]]))
            yaw=abs(float(np.arctan2(np.sin(z[2]-(0 if sign==1 else np.pi)),np.cos(z[2]-(0 if sign==1 else np.pi)))))
            area=clipped_area(corners([z],scene['vehicle'])[0],[-18,18,-1.4,1.4])
            guard.update(preproposal_mode=r['mode'],position_error_m=position,wrapped_yaw_error_rad=yaw,actual_speed_m_s=abs(z[3]),steering_rad=abs(z[4]),protection_area_m2=area,
                ready_guard_passed=bool(r['mode']=='READY' and r['steering_complete'] and abs(z[3])<=1e-12 and abs(z[4])<=1e-7 and position<=.005 and yaw<=.003 and area<=tol),
                preproposal_permission_version=r['context']['permission_version'])
        guard_rows.append(guard)
    previous_actual=None;segment_rows=[];routes=load(directory.parent/'routes.json')
    for a in stream(directory/'actual.jsonl'):
        k=a['cycle']
        for c in CARS:
            report=a['reports'][c];mode=report['mode'];z=np.array(a['states'][c]);body=corners(z,scene['vehicle']);sign=1 if c[0]=='A' else -1
            if previous_actual and report['index']>previous_actual['reports'][c]['index']:
                prior=previous_actual['reports'][c];segment=routes[c]['segments'][prior['index']]
                progress=prior['progress_m']+h*prior['z'][3]+.5*h*h*previous_actual['controls'][c][0]
                pose_error=float(np.linalg.norm(z[0,:2]-np.array(segment['end'])[:2]))
                segment_rows.append(dict(car=c,cycle=k,old_index=prior['index'],new_index=report['index'],progress_end_slack_m=progress-segment['length'],speed_m_s=abs(float(z[0,3])),
                    original_progress_speed_guard_passed=bool(prior['mode']=='MOVE' and progress>=segment['length']-1e-6 and abs(z[0,3])<=1e-12),
                    observed_pose_endpoint_error_m=pose_error,pose_is_explicit_advance_guard=False,
                    state_continuity_max_component=float(np.max(abs(z[0]-np.array(previous_actual['states'][c])[-1])))))
            if report.get('queue_arrived'):queue.setdefault(c,k)
            if report.get('left_queue'):left.setdefault(c,k)
            if c not in clear:
                hit=np.flatnonzero((sign*body[:,:,0]).min(axis=1)>=26.5-tol)
                if len(hit):clear[c]=k*h+int(hit[0])*ds
            if c not in entry:
                lo=body.min(axis=1);hi=body.max(axis=1)
                candidates=np.flatnonzero((hi[:,0]>=-18)&(lo[:,0]<=18)&(hi[:,1]>=-1.4)&(lo[:,1]<=1.4))
                for i in candidates:
                    if clipped_area(body[i],[-18,18,-1.4,1.4])>tol:entry[c]=k*h+int(i)*ds;break
            key=c+':'+mode
            for v in z[1:,3]:
                if abs(v)<=scene['terminal']['speed_m_s']:
                    if spell.get(c,(None,0))[0]==mode:spell[c]=(mode,spell[c][1]+1)
                    else:spell[c]=(mode,1)
                    longest[key]=max(longest[key],spell[c][1])
                else:spell[c]=(None,0)
        previous_actual=a
    return dict(policy_recomputed=True,policy_violations=policy_errors,
                per_car={c:dict(request_elapsed_s=requests[c]*h if c in requests else None,
                    ready_elapsed_s=ready[c]*h if c in ready else None,
                    grant_elapsed_s=grants[c]*h if c in grants else None,
                    request_to_grant_s=(grants[c]-requests[c])*h if c in grants and c in requests else None,
                    ready_to_grant_s=(grants[c]-ready[c])*h if c in grants and c in ready else None,
                    completion_elapsed_s=completions[c]*h if c in completions else None,
                    queue_arrived_elapsed_s=queue[c]*h if c in queue else None,
                    actual_left_queue_elapsed_s=left[c]*h if c in left else None,
                    first_protection_entry_elapsed_s=entry.get(c),first_actual_body_clear_elapsed_s=clear.get(c)) for c in CARS},
                handoff_guards=guard_rows,segment_transition_guards=segment_rows,
                longest_stationary_by_mode_s={k:v*ds for k,v in longest.items()},
                stationary_threshold_m_s=scene['terminal']['speed_m_s'],
                unexplained_stop_diagnosis='no new deadlock detector applied; mode-labelled spells are observations, not a deadlock guarantee')


def detailed_summary(name,result):
    summary=dict(worst_by_unit={},reoptimization_by_event={},partial_phase_failures=[],partial_terminal_failures=[],dynamics_checks=Counter(),violating_samples=Counter(),progress_decreases=[],terminal_zero_worst={},sampled_violation_intervals=[])
    previous={};previous_stop={};summary['reference_stop_activations']=[]
    summary['seed_phase_initialization_failures']=0;summary['seed_phase_shifted_failures']=0
    for row in stream(OUT/result['file']):
        k=row['cycle']
        for layer,safe in row['safety'].items():
            for c,r in safe['cars'].items():
                for metric,v in r['violations'].items():
                    key=layer+':'+metric;old=summary['worst_by_unit'].get(key)
                    if old is None or v['value']>old['value']:summary['worst_by_unit'][key]=dict(value=v['value'],car=c,cycle=k,sample_time_s=row['time_s']+v['lead_s'],lead_s=v['lead_s'])
                    summary['violating_samples'][key]+=v['violating_samples']
                    if layer=='actual' and v['violating_samples']:
                        summary['sampled_violation_intervals'].append(dict(car=c,cycle=k,metric=metric,count=v['violating_samples'],scope='cycle enclosing offending samples; exact duration not inferred from a maximum'))
                for metric,value in r['input_violations'].items():
                    key=layer+':'+metric;old=summary['worst_by_unit'].get(key)
                    if old is None or value>old['value']:summary['worst_by_unit'][key]=dict(value=value,car=c,cycle=k,sample_time_s=row['time_s'])
        for c,cm in row['cars'].items():
            cp=cm['cumulative_route_progress_m']
            if c in previous and cp<previous[c]-1e-5:summary['progress_decreases'].append(dict(car=c,cycle=k,change_m=cp-previous[c]))
            previous[c]=cp
            active='reference_terminal_progress_m' in cm['seed_phase']
            if active and not previous_stop.get(c,False):summary['reference_stop_activations'].append(dict(car=c,cycle=k,time_s=row['time_s'],events=row['events']))
            previous_stop[c]=active
            for prop in ['checked_phase_rows_passed','reference_terminal_rows_passed']:
                if cm['seed_phase'].get(prop) is False:
                    key='partial_phase_failures' if prop.startswith('checked') else 'partial_terminal_failures'
                    summary[key].append(dict(car=c,cycle=k,time_s=row['time_s'],events=row['events'],diagnostic=cm['seed_phase']))
                    if key=='partial_phase_failures':summary['seed_phase_initialization_failures' if k==0 else 'seed_phase_shifted_failures']+=1
            for prop in ['seed_replay_error','final_replay_error','execution_error','shift_initial_error']:
                if prop in cm:
                    summary['dynamics_checks'][prop+':checked']+=1
                    if all(v<=1e-12 for v in cm[prop]['maximum'].values()):summary['dynamics_checks'][prop+':within_1e-12_per_reported_unit']+=1
                    for unit,value in cm[prop]['maximum'].items():
                        key=prop+':'+unit;old=summary['worst_by_unit'].get(key)
                        if old is None or value>old['value']:summary['worst_by_unit'][key]=dict(value=value,car=c,cycle=k,sample_time_s=row['time_s'],lead_s=cm[prop]['argmax_lead_s'][unit])
            if 'zero_tail' in cm:
                for metric in ['terminal_speed_m_s','zero_tail_displacement_m']:
                    old=summary['terminal_zero_worst'].get(metric)
                    if old is None or cm['zero_tail'][metric]>old['value']:summary['terminal_zero_worst'][metric]=dict(value=cm['zero_tail'][metric],cycle=k,car=c)
            if 'successive_final' in cm:
                own_switch=any(e.startswith(c+':') for e in row['events'])
                permission_switch=any(e in ['initial_grant','owner_handoff','release_to_none'] for e in row['events'])
                group='own_mode_or_segment_switch' if own_switch else 'permission_switch' if permission_switch else 'other_car_switch' if row['events']!=['same_mode_permission'] else 'same_mode_permission'
                d=summary['reoptimization_by_event'].setdefault(group,dict(samples=0,worst={}))
                d['samples']+=1
                for unit,v in cm['successive_final']['maximum'].items():
                    old=d['worst'].get(unit)
                    if old is None or v>old['value']:d['worst'][unit]=dict(value=v,cycle=k,car=c,lead_s=cm['successive_final']['argmax_lead_s'][unit])
    summary['dynamics_checks']=dict(summary['dynamics_checks']);summary['violating_samples']=dict(summary['violating_samples'])
    return summary


if __name__=='__main__':
    inventory=load(OUT/'input_identity.json');results=load(OUT/'analysis_results.json');by_name={r['run']:r for r in inventory}
    details={};events={};aliases={};comparison=[]
    for row in inventory:
        name=row['run'];result=results[name]
        if 'error' in result:continue
        source=resolve(name,results);base=results[source]
        if source not in details:details[source]=detailed_summary(source,base)
        events[name]=task_events(row)
        if name!=source:aliases[name]=validate_alias(row,by_name[source],base)
        d=details[source];directory=Path(row['directory']);old=load(directory/'independent_validation.json') if (directory/'independent_validation.json').exists() else {}
        summary=load(directory/'summary.json');planning=list(stream(directory/'planning.jsonl'))
        periods=[b['start_monotonic_s']-a['start_monotonic_s'] for a,b in zip(planning,planning[1:]) if a['committed']]
        values=np.array(periods)
        counts=base['counts'];w=base['worst']
        comparison.append(dict(run=name,classification=row['classification'],motion_group=row['motion_group'],metrics_source=source,
            physical_scenario=row['physical_scenario'],recorded_full_acceptance=old.get('actual_full_acceptance'),
            completion_s=summary.get('completion_sim_s'),actual_cycles=counts.get('actual_cycles',0),
            sampled_actual_gap_margin_m=-w['actual_gap_deficit_m']['value'] if 'actual_gap_deficit_m' in w else None,
            sampled_final_gap_margin_m=-w['final_gap_deficit_m']['value'] if 'final_gap_deficit_m' in w else None,
            seed_physical_checked=counts.get('seed_complete',0),seed_physical_passed=counts.get('seed_physical_protocol_passed',0),
            partial_phase_checked=counts.get('seed_phase_checked',0),partial_phase_passed=counts.get('seed_phase_passed',0),
            partial_terminal_checked=counts.get('seed_terminal_rows_checked',0),partial_terminal_passed=counts.get('seed_terminal_rows_passed',0),
            successive_position_change_m=w.get('successive_final_position_m',{}).get('value'),
            full_ocp_candidate_checked=False,lyapunov_guarantee_evaluable=False,robust_domain_identified=False,
            cycle_count=len(values),cycle_p50_ms=float(np.percentile(values,50)*1000) if len(values) else None,
            cycle_p95_ms=float(np.percentile(values,95)*1000) if len(values) else None,
            cycle_p99_ms=float(np.percentile(values,99)*1000) if len(values) else None,
            cycle_max_ms=float(values.max()*1000) if len(values) else None,cycles_over_200ms=int(np.sum(values>.2)),
            application_bytes=sum(r['application_bytes'] for r in planning),online_wall_s=summary.get('online_wall_s')))
    write(OUT/'detailed_results.json',details);write(OUT/'task_events.json',events);write(OUT/'alias_protocol_validation.json',aliases)
    write(OUT/'comparison.json',comparison)
    if comparison:
        with (OUT/'comparison.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=list(comparison[0]));w.writeheader();w.writerows(comparison)
    with (OUT/'seed_failure_events.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['metrics_source','kind','car','cycle','time_s','events','diagnostic'])
        for source,d in details.items():
            for kind in ['partial_phase_failures','partial_terminal_failures']:
                for r in d[kind]:w.writerow([source,kind,r['car'],r['cycle'],r['time_s'],canonical(r['events']),canonical(r['diagnostic'])])
    for directory in ['cycle_metrics','event_metrics']:
        with (OUT/directory/'index.csv').open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['run','metrics_source','file','equality_basis','own_alias_protocol_verified'])
            for row in inventory:
                name=row['run']
                if 'error' in results[name]:continue
                source=resolve(name,results);key=source.replace('/','__')
                w.writerow([name,source,key+'.jsonl.gz','actual logical hash + complete plan bytes + scenario identity',aliases.get(name,{}).get('passed',name==source)])
    print('Supplemental summaries and per-alias protocol validation complete.',flush=True)
