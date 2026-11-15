#!/usr/bin/env python3
"""One of the 15 gated, development-only four-car runs. No formal-batch mode."""
import argparse
import copy
from fractions import Fraction as F
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import time

from moving_objects import content_id
from pilot_control import (CARS,H,load_model,initial_state,object_by_id,initial_plan,
    optimize,commit_token,execute,shift_plan,state_in_node,pose_rows)


def write(path,value):path.write_text(json.dumps(value,default=str,ensure_ascii=False,indent=2)+'\n')


def run(args):
    out=args.output;out.mkdir(parents=True)
    gate=json.loads(args.gate.read_text())
    assert gate['p0_passed'] and gate['development_pilot_allowed'] and not gate['formal_batch_allowed']
    here=Path(__file__).resolve().parent
    for name,digest in gate['repository_dependencies_sha256'].items():
        assert hashlib.sha256((here.parents[1]/name).read_bytes()).hexdigest()==digest,'repository dependency changed: '+name
    for name,digest in gate['algorithm_sha256'].items():
        assert hashlib.sha256((here/name).read_bytes()).hexdigest()==digest,'algorithm changed after P0: '+name
        shutil.copy2(here/name,out/name)
    model_path=args.rebuild/'work/M2b_首段自由度与递归可行性/route_zoh_certificate.py'
    assert hashlib.sha256(model_path.read_bytes()).hexdigest()==gate['interval_model_sha256']
    case=args.objects/args.case;cfg=json.loads((case/'config.json').read_text())
    common=json.loads((case/'coordination.json').read_text())
    assert hashlib.sha256((case/'coordination.json').read_bytes()).hexdigest()==gate['case_coordination_sha256'][args.case]
    assert hashlib.sha256((case/'config.json').read_bytes()).hexdigest()==gate['case_config_sha256'][args.case]
    write(out/'config.json',dict(case=args.case,method=args.method,scene=cfg,physical_dt_s=str(H),
        execution='single-process synchronous ideal protocol, frozen physical clock during computation',
        goal='configured position/yaw bounds, exact v=delta=0, then invariant zero-input hold',
        initial_policy_sha256=content_id(common['initial_policy']),initial_U_sha256=content_id(common['initial_U']),
        physical_Tmax_s=str(H*common['initial_U'][common['initial']]),
        timeout_source='recomputed common feasible strategy; not legacy timeout_s=180',
        preparation_objects=str(args.objects.resolve()),source_rebuild=str(args.rebuild.resolve()),
        gate_sha256=hashlib.sha256(args.gate.read_bytes()).hexdigest(),
        formal_holdout=False,communication_scope='in-process barrier; no DDS/network traffic measurement'))
    model=load_model(args.rebuild);state=initial_state(model,cfg)
    U=copy.deepcopy(common['initial_U']);policy=copy.deepcopy(common['initial_policy'])
    node=common['initial'];budget=U[node];initial_budget=budget;cycle=0;version=0;done={}
    stopped=args.method.startswith('S');lookahead=args.method in ('S-T','M-T','M-R')
    optimize_enabled=args.method!='M-R';energy=F(0);change_energy=F(0)
    component_energy=[F(0),F(0)]
    previous={c:(F(0),F(0)) for c in CARS};counts=dict(changed=0,accepted=0,rejected=0,native_solves=0,inherited_invalid=0,program_errors=0)
    message_count=message_bytes=0
    cycles=[];events=[];minimum_road=100*model.Q;minimum_pair=100*model.Q
    travel={c:F(0) for c in CARS};waiting={c:F(0) for c in CARS};handoff_stops=0;moving_joins=[]
    failure=None;began=time.perf_counter();objects={}
    with gzip.open(out/'execution.jsonl.gz','wt',encoding='utf8') as stream:
        try:
            while node!=common['goal']:
                connection_start=time.perf_counter()
                before_node_budget=budget;old_U=copy.deepcopy(U)
                choices=[e for e in common['options'] if e['source']==node and (not stopped or e['kind']=='S')
                         and e['ticks']+U[e['target']]<=U[node]]
                assert choices,'no certified replacement and no default continuation'
                edge=min(choices,key=lambda e:((e['ticks']+U[e['target']]) if lookahead else e['ticks'],
                                               e['actor'],e['target'],e['object']))
                policy[node]=edge
                for s in common['topological_order']:
                    if s==common['goal']:U[s]=0
                    else:
                        e=policy[s];U[s]=e['ticks']+U[e['target']]
                assert all(U[s]<=old_U[s] for s in U)
                budget=edge['ticks']+U[edge['target']]
                assert budget<=before_node_budget
                version+=1;cert=object_by_id(args.objects,edge['object']);objects[edge['object']]=cert
                assert state_in_node(state,cfg,node,cert['stage_radii'])
                plan=initial_plan(model,state,cert,edge,stopped=stopped,
                    progress=dict(version=version,policy=policy,U=U,source_allowance=before_node_budget))
                event=dict(event='connection_commit',cycle=cycle,version=version,edge=edge,
                    budget_before=before_node_budget,budget_after=budget,U_version=version,
                    continuation_policy=policy,continuation_U=U)
                # Snapshot policy versions; later updates must not change old evidence.
                event=copy.deepcopy(event);events.append(event)
                for phase in range(cert['ticks']):
                    start=connection_start if phase==0 else time.perf_counter();old=plan;c=cert['car']
                    others=[x for x in CARS if x!=c]
                    groups=[[c,others[0]],[others[1]],[others[2]]] if phase%64==0 else \
                           [[c,others[0],others[1]],[others[2]]] if phase%64==1 else [[c],[others[0],others[1]],[others[2]]]
                    if optimize_enabled:proposal,record=optimize(model,state,old,cert,phase,cfg,stopped)
                    else:proposal=old;record=dict(accepted=True,changed=False,solver_called=False,status='certified_reference_only')
                    staged=copy.deepcopy(state);v,d=state['v'][c],state['delta'][c]
                    validation_start=time.perf_counter()
                    try:actual=execute(model,staged,c,proposal['controls'][0],cfg,cert['source']>0)
                    except ValueError as error:
                        if proposal['id']==old['id']:raise
                        record.update(accepted=False,changed=False,failure_layer='new_candidate',rejection='first-step certificate: '+str(error))
                        proposal=old;staged=copy.deepcopy(state)
                        actual=execute(model,staged,c,proposal['controls'][0],cfg,cert['source']>0)
                    validation_wall=time.perf_counter()-validation_start
                    token=commit_token(proposal,state,cert,phase,cycle,version,groups,old,cfg,model)
                    assert token['context']['physical_budget']==budget
                    message_count+=token['application_messages'];message_bytes+=token['application_bytes']
                    executable=time.perf_counter();state=staged
                    assert budget>0;before=budget;budget-=1
                    u=proposal['controls'][0];energy+=H*sum(x*x for x in u)
                    for j in (0,1):component_energy[j]+=H*u[j]*u[j]
                    current={x:u if x==c else (F(0),F(0)) for x in CARS}
                    change_energy+=sum((x-y)**2 for car in CARS for x,y in zip(current[car],previous[car]));previous=current
                    travel[c]+=H*(v+state['v'][c])/2
                    for car in CARS:
                        if car not in done and (car!=c or v==state['v'][c]==0):waiting[car]+=H
                    if v>0 and state['v'][c]==0 and not (phase+1==cert['ticks'] and edge['target'][CARS.index(c)]=='3'):handoff_stops+=1
                    if phase+1 in [j['cut'] for j in cert['joins'][:-1]]:
                        moving_joins.append(dict(cycle=cycle+1,actor=c,v=str(state['v'][c]),delta=str(state['delta'][c])))
                    minimum_road=min(minimum_road,actual['minimum_road_lower_Q'])
                    minimum_pair=min(minimum_pair,actual['minimum_pair_lower_Q'])
                    counts['accepted']+=int(optimize_enabled and record['accepted']);counts['rejected']+=int(optimize_enabled and not record['accepted'])
                    counts['changed']+=int(record['accepted'] and record['changed']);counts['native_solves']+=int(record.get('solver_called',False))
                    counts['inherited_invalid']+=int(record.get('failure_layer')=='inherited_anchor')
                    counts['program_errors']+=int(record.get('failure_layer')=='optimizer_program_error')
                    short={k:v for k,v in record.items() if k not in ('scaled_numerical_solution','coefficients')}
                    row=dict(cycle=cycle,phase=phase,version=version,object=edge['object'],source=edge['source'],target=edge['target'],
                        actor=c,groups=groups,actual=actual,budget_before=before,budget_after=budget,
                        receiver_phase=token['context']['receiver_phase'],remaining_deadline=token['context']['remaining_deadline'],
                        receiver_time_value=token['context']['receiver_time_value'],policy_version=token['context']['policy_version'],policy_sha256=token['context']['policy_sha256'],
                        application_messages=token['application_messages'],application_bytes=token['application_bytes'],message_digest=token['message_digest'],
                        committed_plan=proposal['id'],parent_plan=old['id'],history=state['history'],
                        optimizer=short,cycle_wall_s=executable-start,first_step_validation_wall_s=validation_wall)
                    if record['accepted'] and record['changed']:
                        row['accepted_controls']=proposal['controls']
                        row['cone_proof']=proposal['proof']
                    stream.write(json.dumps(row,default=str,separators=(',',':'))+'\n')
                    cycles.append(executable-start);cycle+=1
                    assert cycle<=initial_budget,'physical completion exceeded initial witnessed budget'
                    if phase+1<cert['ticks']:plan=shift_plan(model,proposal,cert,phase,v,d)
                assert state_in_node(state,cfg,edge['target'],cert['stage_radii'])
                assert budget==U[edge['target']]
                node=edge['target']
                if node[CARS.index(c)]=='3' and c not in done:done[c]=str(cycle*H)
            assert budget==0 and len(done)==4
            assert all(state['v'][c]==state['delta'][c]==0 for c in CARS)
        except Exception as error:failure=repr(error)
    import numpy as np
    summary=dict(passed=failure is None and counts['inherited_invalid']==counts['program_errors']==0,error=failure,case=args.case,method=args.method,
        cycles=cycle,all_goals_reached=len(done)==4,arrival_s=done,
        T_all_s=str(cycle*H) if failure is None else None,
        continuous_road_lower_m=float(F(minimum_road,model.Q)),
        continuous_pair_lower_m=float(F(minimum_pair,model.Q)),initial_budget_ticks=initial_budget,
        remaining_budget=budget,optimizer_counts=counts,input_energy=float(energy),input_change_energy=float(change_energy),
        acceleration_energy=float(component_energy[0]),steering_rate_energy=float(component_energy[1]),
        application_messages=message_count,application_bytes=message_bytes,
        communication_scope='serialized reliable ideal JSON messages; excludes network/DDS headers',
        offline_driver_wall_s=time.perf_counter()-began,
        path_length_m={c:float(x) for c,x in travel.items()},waiting_before_arrival_s={c:str(x) for c,x in waiting.items()},
        handoff_stops=handoff_stops,physical_waypoint_joins=moving_joins,
        cycle_wall_s={k:float(np.quantile(cycles,q)) for k,q in [('p50',.5),('p95',.95),('p99',.99)]} if cycles else {},
        cycle_max_s=max(cycles,default=0),over_200ms_fraction=sum(t>.2 for t in cycles)/len(cycles) if cycles else None,
        invariant_hold='all final v/delta exactly zero, zero input leaves pose fixed',
        local_time_search=False,secondary_objective='sum normalized squared acceleration/steering-rate inputs',
        accepted_rationalized_plan_optimality_proven=False,
        independent_acceptance_pending=True,development_only=True,formal_batch=False)
    write(out/'summary.json',summary);write(out/'coordination_events.json',events)
    write(out/'referenced_objects.json',{sha:str((args.objects/'objects'/(sha+'.json')).resolve()) for sha in objects})
    print(json.dumps(summary,indent=2),flush=True)
    return 0 if summary['passed'] else 1


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('rebuild','objects','gate','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--case',choices=['asymmetric_baseline','tight_curve','exit_merge_dependency'],required=True)
    p.add_argument('--method',choices=['S-L','S-T','M-L','M-T','M-R'],required=True)
    args=p.parse_args()
    if args.output.exists():p.error('use a new output directory')
    return run(args)


if __name__=='__main__':raise SystemExit(main())
