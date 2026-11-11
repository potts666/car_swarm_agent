"""Read-only, streaming theory-linked diagnostics of existing runs. No optimizer calls."""
from collections import Counter,defaultdict
import csv
import gzip
import hashlib
import json
from pathlib import Path
import sys
import time

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[3]
RESULTS=ROOT/'examples/head_on_mpc/results'
# Use the frozen stage1 source, not an editable controller implementation.
FROZEN=json.loads((RESULTS/'M2b_stage1_001/full_L_FIFO/r1/config.json').read_text())['source_version']
SOURCE=RESULTS/'M2b_stage1_001/sources'/FROZEN
sys.path.insert(0,str(SOURCE))
sys.path.insert(0,str(OUT))
import numpy as np
from analysis_io import canonical,digest,file_sha,physical,stream,restored_plans,CycleStream,logical_motion,run_paths
from metrics_core import CARS,UNITS,differences,aligned_difference,layer_safety,phase_diagnostic,terminal_zero_check,corners,cumulative_progress
from bicycle_core import rollout


def write(path,value):path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def load(path):return json.loads(path.read_text())


def inventory():
    rows=[];hashes={};known={};old_manifest=load(RESULTS/'M2b_fourcar_001/artifact_manifest.json')
    stage_manifest=load(RESULTS/'M2b_stage1_001/artifact_manifest.json')['files']
    for name,directory,kind in run_paths(RESULTS):
        try:
            cfg=load(directory/'config.json');summary=load(directory/'summary.json');top=directory.parent.parent
            scene={k:v for k,v in cfg['scene'].items() if k not in ('source_version','log_format')}
            motion,count=logical_motion(directory)
            files={}
            for filename in ['config.json','summary.json','actual.jsonl','plans.jsonl','messages.jsonl','agreements.jsonl','events.jsonl','solves.jsonl','planning.jsonl']:
                p=physical(directory/filename);stat=p.stat();inode=(stat.st_dev,stat.st_ino,stat.st_size,stat.st_mtime_ns)
                if inode not in hashes:hashes[inode]=file_sha(p)
                value=hashes[inode];relative=str(p.relative_to(top))
                expected=old_manifest.get(relative) if top.name=='M2b_fourcar_001' else stage_manifest.get(relative,{}).get('sha256')
                if expected is not None and expected!=value:raise ValueError('input manifest mismatch '+str(p))
                files[filename]={'path':str(p.relative_to(ROOT)),'sha256':value,'bytes':stat.st_size,'manifest_match':True if expected else None}
            key=(motion,files['plans.jsonl']['sha256'],digest(scene))
            representative=known.setdefault(key,name)
            source=top/'sources'/str(cfg.get('source_version'))
            source_checks={}
            if source.exists():
                for f in ['bicycle_core.py','m2b_scene.py','verify_entry_bicycle.py']:
                    source_checks[f]=file_sha(source/f)==file_sha(SOURCE/f)
            if source_checks and not all(source_checks.values()):raise ValueError('incompatible physical validator '+name)
            row=dict(run=name,directory=str(directory),classification=('debug_complete' if summary['all_completed'] else 'natural_failure') if kind=='natural_debug' else kind,arm=cfg['arm'],policy=cfg['policy'],
                     all_completed=summary['all_completed'],committed_cycles=count,motion_group=motion,
                     physical_scenario=digest(scene),prediction_file_sha256=files['plans.jsonl']['sha256'],metrics_source=representative,
                     source_version=cfg.get('source_version'),source_snapshot_available=source.exists(),physical_source_matches=source_checks,
                     failure=summary.get('failure'),failure_layer=summary.get('failure_layer'),files=files)
            rows.append(row)
        except Exception as e:
            rows.append(dict(run=name,directory=str(directory),classification=kind,inventory_error=type(e).__name__+': '+str(e)))
    write(OUT/'input_identity.json',rows)
    with (OUT/'run_inventory.csv').open('w',newline='') as f:
        keys=['run','classification','arm','policy','all_completed','committed_cycles','physical_scenario','motion_group','metrics_source','source_version','source_snapshot_available','failure','failure_layer']
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows({k:r.get(k) for k in keys} for r in rows)
    return rows


def protocol_check(actual,final,agreement,messages,known):
    sent={};receives=set();refs=0
    for m in messages:
        if m['event']=='send':
            if m['sequence'] in sent:raise ValueError('repeated send sequence')
            sent[m['sequence']]=m
            if m['kind'] in ('seed','final'):
                payload=m['payload'];pid=payload.get('$plan',payload.get('id'))
                if pid not in known:raise ValueError('missing message plan body')
                if '$plan' in payload and set(payload)!={'$plan'}:raise ValueError('invalid message reference')
                refs+=1
        elif m['event']=='receive':
            s=sent.get(m['sequence'])
            if s is None or s['receiver']!=m['receiver']:raise ValueError('orphan or wrong receive')
            receives.add(m['sequence'])
    if set(sent)!=receives:raise ValueError('missing delivery')
    if actual is None:return dict(plan_references_checked=refs,actual_commit_checked=False)
    if agreement is None or set(final)!=set(CARS):raise ValueError('missing committed final/ACK set')
    ids={c:final[c]['id'] for c in CARS};packet=agreement['packet'];context=final[CARS[0]]['context']
    if ids!=actual['plans'] or ids!=agreement['plans'] or ids!=packet['token']['plans']:raise ValueError('mixed final plan set')
    acks=agreement['acks']
    if set(acks)!=set(CARS):raise ValueError('missing four ACKs')
    for c,a in acks.items():
        if a['car']!=c or a['context']!=context or a['plans']!=ids or not a['accepted']:raise ValueError('invalid ACK binding')
    if packet['effective']['owner']!=actual['owner'] or packet['effective']['permission_version']!=actual['permission_version']:raise ValueError('execution authorization mismatch')
    if context['owner']!=actual['owner'] or context['permission_version']!=actual['permission_version']:raise ValueError('conditional final not activated')
    if context['conditional'] and packet['activated_proposal_id']!=context['proposal_id']:raise ValueError('unactivated proposal')
    return dict(plan_references_checked=refs,actual_commit_checked=True,ack_count=4,plans=ids,permission_version=actual['permission_version'])


def classify(previous,reports,context,old_context):
    changes=[]
    for c,r in reports.items():
        old=previous.get(c)
        if old and r['mode']!=old['mode']:changes.append(c+':mode:'+old['mode']+'>'+r['mode'])
        if old and r['index']!=old['index']:changes.append(c+':segment')
        for key in ['queue_arrived','left_queue']:
            if old and r.get(key)!=old.get(key):changes.append(c+':'+key)
    if old_context and context.get('permission_version')!=old_context.get('permission_version'):
        old=old_context['owner'];new=context['owner']
        changes.append('initial_grant' if old is None and new else 'release_to_none' if new is None else 'owner_handoff')
    return changes or ['same_mode_permission']


def compute_run(row):
    name=row['run'];directory=Path(row['directory']);cfg=load(directory/'config.json');scene=cfg['scene'];routes=load(directory.parent/'routes.json')
    ds=scene['sample_s'];h=scene['h'];n=round(h/ds);N=scene['N'];tol=scene['tolerance']
    if abs(n*ds-h)>1e-10:raise ValueError('unaligned sample periods')
    actuals=CycleStream(stream(directory/'actual.jsonl'),lambda r:r['cycle'])
    plans=CycleStream(restored_plans(directory/'plans.jsonl'),lambda r:r['context']['cycle'])
    messages=CycleStream(stream(directory/'messages.jsonl'),lambda r:r['cycle'])
    agreements=CycleStream(stream(directory/'agreements.jsonl'),lambda r:r['cycle'])
    solves=CycleStream(stream(directory/'solves.jsonl'),lambda r:r['cycle'])
    previous_final={};previous_reports={};old_context=None;horizon={};counts=Counter();worst={};episodes={};open_episodes={};mode_duration=Counter()
    run_key=name.replace('/','__');dest=OUT/'cycle_metrics'/f'{run_key}.jsonl.gz';dest.parent.mkdir(exist_ok=True)
    event_rows=[];event_channel=list(stream(directory/'events.jsonl'));event_cycles={r['cycle'] for r in event_channel}
    windows={};compact_cycles={}
    def retain_worst(key,value,cycle,**where):
        if key not in worst or value>worst[key]['value']:worst[key]=dict(value=float(value),cycle=cycle,time_s=scene['epoch_s']+cycle*h,**where)
    with gzip.open(dest,'wt') as output:
        for planning in stream(directory/'planning.jsonl'):
            k=planning['cycle'];a=actuals.take(k);actual=a[0] if a else None
            ps=plans.take(k);by_id={p['id']:p for p in ps};seed={p['car']:p for p in ps if p['kind']=='seed'};final={p['car']:p for p in ps if p['kind']=='final'}
            msg=messages.take(k);ag=agreements.take(k);sr=solves.take(k)
            context=(ps[0]['context'] if ps else planning.get('context'))
            if context is None:
                if planning['outcome']=='completed':
                    counts['completion_only_cycles']+=1
                    terminal_reports={m['sender']:m['payload'] for m in msg if m['event']=='send' and m.get('kind')=='state' and m.get('receiver')=='channel'}
                    compact_cycles[k]=dict(cycle=k,time_s=scene['epoch_s']+k*h,available=True,cycle_role='completion_only_no_input_or_plan',modes={c:r['mode'] for c,r in terminal_reports.items()},reports=terminal_reports,plans={},acks={})
                    continue
                raise ValueError('cycle has no usable context')
            reports={c:p['report'] for c,p in {**seed,**final}.items()}
            changes=classify(previous_reports,reports,context,old_context)
            if changes!=['same_mode_permission']:event_cycles.add(k)
            protocol=protocol_check(actual,final,ag[0] if ag else None,msg,by_id)
            metrics=dict(run=name,cycle=k,time_s=scene['epoch_s']+k*h,elapsed_s=k*h,context=context,
                         outcome=planning['outcome'],events=changes,protocol=protocol,seed_ids={c:p['id'] for c,p in seed.items()},
                         final_ids={c:p['id'] for c,p in final.items()},cars={},safety={})
            for layer,selection in [('seed',seed),('final',final)]:
                safe=layer_safety(selection,context,scene);metrics['safety'][layer]=safe
                if safe['complete']:
                    counts[layer+'_complete']+=1;counts[layer+'_physical_protocol_passed']+=int(safe['physical_protocol_passed'])
                for pair,v in safe['pairs'].items():retain_worst(layer+'_gap_deficit_m',-v['margin_m'],k,pair=pair,lead_s=v['lead_s'])
            if actual:
                selection={c:dict(states=actual['states'][c],commands=[actual['controls'][c]]) for c in CARS}
                metrics['safety']['actual']=layer_safety(selection,context,scene);counts['actual_cycles']+=1
                for pair,v in metrics['safety']['actual']['pairs'].items():retain_worst('actual_gap_deficit_m',-v['margin_m'],k,pair=pair,lead_s=v['lead_s'])
            records=sr[0] if sr else {};local=records.get('local');central=records.get('central')
            for c,p in seed.items():
                z=np.array(p['states']);old=previous_final.get(c);car_metrics=dict(mode=p['report']['mode'],index=p['report']['index'])
                r=p['report'];car_metrics['cumulative_route_progress_m']=cumulative_progress(r,routes[c])
                if actual:mode_duration[c+':'+r['mode']]+=h
                if old is None:
                    if k!=0 or p['shifted_from'] is not None:raise ValueError('missing previous final for seed')
                    car_metrics['seed_initialization_error']=differences(z,np.tile(scene['initial'][c],(len(z),1)),ds)[0]
                else:
                    expected=old['commands'][1:]+[[0.,0.,'SHIFT_TAIL']]
                    if p['shifted_from']!=old['id'] or p['commands']!=expected:raise ValueError('implemented shift differs from submitted input shift')
                    car_metrics['shift_initial_error']=differences([old['states'][n]],[r['z']],ds)[0]
                    counts['shift_candidates']+=1
                replayed=rollout(np.array(r['z']),p['commands'],scene)
                car_metrics['seed_replay_error']=differences(z,replayed,ds)[0]
                counts['seed_replays']+=1
                record=central if central else (local or {}).get(c)
                parameters=(record or {}).get('reference_record',{}).get('parameters',{}).get(c)
                diagnostic=phase_diagnostic(p,routes[c],scene,parameters,final[c]['commands'] if c in final else None)
                car_metrics['seed_phase']=diagnostic
                checked=diagnostic.get('checked_phase_rows_passed')
                if checked is not None:counts['seed_phase_checked']+=1;counts['seed_phase_passed']+=int(checked)
                terminal=diagnostic.get('reference_terminal_rows_passed')
                if terminal is not None:counts['seed_terminal_rows_checked']+=1;counts['seed_terminal_rows_passed']+=int(terminal)
                if 'discrete_stop_margin_m' in diagnostic:retain_worst('discrete_stop_deficit_m',-diagnostic['discrete_stop_margin_m'],k,car=c,mode=r['mode'])
                if c in final:
                    f=final[c];fz=np.array(f['states']);fr=rollout(np.array(f['report']['z']),f['commands'],scene)
                    car_metrics['final_replay_error']=differences(fz,fr,ds)[0]
                    car_metrics['seed_to_final']=differences(z,fz,ds)[0]
                    car_metrics['zero_tail']=terminal_zero_check(f,scene);counts['final_replays']+=1
                    if old:
                        change,values=aligned_difference(old['states'],fz,h,ds);car_metrics['successive_final']=change
                        for unit,value in change['maximum'].items():
                            retain_worst('successive_final_'+unit,value,k,car=c,event=changes,lead_s=change['argmax_lead_s'][unit])
                            key=c+':'+unit
                            horizon[key]=np.maximum(horizon.get(key,np.zeros(len(values[unit]))),values[unit])
                    if actual:
                        car_metrics['execution_error']=differences(fz[:n+1],actual['states'][c],ds)[0]
                    for key in ['seed_replay_error','final_replay_error','execution_error','shift_initial_error']:
                        if key in car_metrics:
                            for unit,value in car_metrics[key]['maximum'].items():retain_worst(key+'_'+unit,value,k,car=c)
                if record:
                    car_metrics['solver_diagnostics']=dict(accepted=record['accepted'],failure_layer=record.get('failure_layer'),mapping_errors=record.get('mapping_errors'),deadline_exceeded=record.get('deadline_exceeded'),reference_message=record.get('reference_record',{}).get('message'),reference_variables=record.get('reference_record',{}).get('variables'),bicycle_variables=[q['variables'] for q in record.get('subproblems',[])],reference_iterations=record.get('reference_record',{}).get('iterations'),bicycle_iterations=[q['iterations'] for q in record.get('subproblems',[])])
                metrics['cars'][c]=car_metrics
            output.write(canonical(metrics)+'\n');counts['analyzed_plan_cycles']+=1
            compact_cycles[k]=dict(cycle=k,time_s=metrics['time_s'],owner=context['owner'],permission_version=context['permission_version'],
                                    reports=reports,modes={c:r['mode'] for c,r in reports.items()},indices={c:r['index'] for c,r in reports.items()},
                                    plans=metrics['final_ids'],acks=(ag[0]['acks'] if ag else {}),events=changes,
                                    gap_margin_m={layer:v['gap_margin_m'] for layer,v in metrics['safety'].items()},
                                    seed_phase={c:v['seed_phase'] for c,v in metrics['cars'].items()},
                                    reoptimization={c:v.get('successive_final') for c,v in metrics['cars'].items()})
            previous_final=final or previous_final;previous_reports=reports;old_context=context
            if k and k%200==0:print(name,'analyzed cycle',k,flush=True)
    for event in event_channel:
        if event['kind']=='handoff':
            k=event['cycle'];entry=compact_cycles.get(k);old=event.get('old_owner')
            if old:
                z=entry['reports'][old]['z'];body=corners([z],scene['vehicle'])[0];sign=1 if old[0]=='A' else -1
                slack=float(np.min(sign*body[:,0])-26.5)
                event=dict(event,recomputed_rotated_body_release_margin_m=slack,recomputed_release_passed=slack>=-tol)
            if event.get('owner'):
                r=entry['reports'][event['owner']];z=r['z'];head=np.array(scene['queue_heads'][event['owner'][0]])
                event=dict(event,recomputed_ready_position_error_m=float(np.linalg.norm(np.array(z[:2])-head)),recomputed_ready_speed_m_s=abs(z[3]),mode=r['mode'])
        event_rows.append(event)
    with gzip.open(OUT/'event_metrics'/f'{run_key}.jsonl.gz','wt') as output:
        for k in sorted(event_cycles):
            windows=[compact_cycles.get(k+i,dict(cycle=k+i,available=False)) for i in [-1,0,1]]
            output.write(canonical(dict(run=name,event_cycle=k,channel_events=[r for r in event_rows if r['cycle']==k],window=windows))+'\n')
    with (OUT/'horizon_envelopes'/f'{run_key}.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['run','car','metric','lead_s','observed_maximum'])
        for key,values in horizon.items():
            c,unit=key.split(':')
            for i,v in enumerate(values):w.writerow([name,c,unit,i*ds,float(v)])
    return dict(counts=dict(counts),worst=worst,mode_duration_s=dict(mode_duration),event_windows=len(event_cycles),
                file=str(dest.relative_to(OUT)),ocp_feasibility_checked=False,geometric_independence=False)


def main():
    if (OUT/'status.json').exists():raise SystemExit('Refuse to overwrite an analysis run; use a new numbered output')
    for name in ['event_metrics','horizon_envelopes']: (OUT/name).mkdir(exist_ok=True)
    write(OUT/'status.json',dict(stage='input_inventory',started_unix=time.time()))
    inv=inventory();write(OUT/'parameters.json',dict(horizon_source='per-run config',prediction_alignment='old[h/sample:] vs new[:common_length]',
        replay_tolerance=1e-12,phase_tolerance='per-run scene tolerance',new_controls_or_closed_loops=False,
        frozen_analysis_source=str(SOURCE),physical_source_sha256={n:file_sha(SOURCE/n) for n in ['bicycle_core.py','m2b_scene.py']},
        candidate_scope='physical/protocol and selected phase/terminal rows; full QP/trust/corridor NOT reconstructed'))
    results={}
    for row in inv:
        name=row['run'];write(OUT/'status.json',dict(stage='analyzing',run=name,finished=list(results)))
        if 'inventory_error' in row:
            results[name]=dict(error=row['inventory_error'],evidence_status='input identity failed; analysis not attempted')
            write(OUT/'analysis_results.json',results)
            continue
        print('START',name,'source',row['metrics_source'],flush=True)
        if row['metrics_source']!=name:
            results[name]=dict(reused_from=row['metrics_source'],equality_basis='exact logical actual hash, entire compressed plan bytes, scenario hash')
        else:
            try:results[name]=compute_run(row)
            except Exception as e:
                results[name]=dict(error=type(e).__name__+': '+str(e),evidence_status='not verified; preserve partial output')
                print('FAILED',name,results[name]['error'],flush=True)
        write(OUT/'analysis_results.json',results)
    write(OUT/'status.json',dict(stage='data_analysis_complete',finished=list(results),errors={k:v['error'] for k,v in results.items() if 'error' in v},ended_unix=time.time()))
    print('DATA ANALYSIS COMPLETE',flush=True)


if __name__=='__main__':main()
