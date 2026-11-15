#!/usr/bin/env python3
"""P0: one moving connector, real fallback execution, grouping and conic checks."""
import argparse
import copy
from fractions import Fraction as F
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import time

from pilot_control import (CARS,H,load_model,initial_state,object_by_id,initial_plan,
    optimize,commit_token,execute,shift_plan,state_in_node,pose_rows)


def write(path,value):path.write_text(json.dumps(value,default=str,ensure_ascii=False,indent=2)+'\n')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--rebuild',type=Path,required=True);p.add_argument('--objects',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--stopped',action='store_true')
    a=p.parse_args()
    if a.output.exists():p.error('use a new output directory')
    a.output.mkdir(parents=True)
    for name in ('check_runtime.py','pilot_control.py','socp_anchor.py','moving_objects.py'):
        shutil.copy2(Path(__file__).with_name(name),a.output/name)
    case=a.objects/'asymmetric_baseline';cfg=json.loads((case/'config.json').read_text())
    coord=json.loads((case/'coordination.json').read_text());seed=coord['initial_policy'][coord['initial']]
    edge=next(e for e in coord['options'] if e['kind']==('S' if a.stopped else 'M') and e['motion']==seed['motion'] and e['source']==seed['source'])
    cert=object_by_id(a.objects,edge['object']);model=load_model(a.rebuild);state=initial_state(model,cfg)
    write(a.output/'config.json',dict(scene=cfg,case='asymmetric_baseline',method='P0-S' if a.stopped else 'P0-M',
        source_rebuild=str(a.rebuild.resolve()),objects=str(a.objects.resolve()),scope='one-connector P0, not full task'))
    assert state_in_node(state,cfg,edge['source'],cert['stage_radii'])
    progress=dict(version=1,policy=coord['initial_policy'],U=coord['initial_U'],source_allowance=coord['initial_U'][edge['source']])
    plan=initial_plan(model,state,cert,edge,stopped=a.stopped,progress=progress)
    budget=edge['ticks']+coord['initial_U'][edge['target']]
    assert budget<=coord['initial_U'][edge['source']]
    source_budget=budget;rejected=[];changed=accepted=0;minroad=minpair=None;versions=[]
    failure=None;began=time.perf_counter()
    with gzip.open(a.output/'execution.jsonl.gz','wt',encoding='utf8') as log:
        try:
            for phase in range(cert['ticks']):
                groups=[['A1','A2'],['B1'],['B2']] if phase==0 else [['A1','A2','B1'],['B2']] if phase==1 else [['A1'],['A2','B1'],['B2']]
                old=plan
                if phase<7:
                    bad=copy.deepcopy(old)
                    kind=['velocity_seam','moving_zero_tail','outside_receiver_prediction','stale_history','wrong_certificate','steering_seam','expired_phase'][phase]
                    if phase==0:bad['v0']+=F(1,10)
                    if phase==1:bad['controls'][-1]=(F(0),F(0))
                    if phase==2:bad['poses'][-1]=tuple(x+100 for x in bad['poses'][-1])
                    if phase==3:bad['history']='expired'
                    if phase==4:bad['object']='wrong-owner-or-domain-certificate'
                    if phase==5:bad['delta0']+=F(1,10)
                    if phase==6:bad['phase']-=1
                    try:commit_token(bad,state,cert,phase,phase,phase+1,groups,old,cfg,model)
                    except ValueError as err:rejected.append(dict(case=kind,reason=str(err)))
                    else:raise AssertionError('illegal proposal accepted: '+kind)
                    assert plan is old
                    proposal=old;record=dict(accepted=False,changed=False,rejection=kind,fallback='committed branch executed')
                else:
                    proposal,record=optimize(model,state,old,cert,phase,cfg,a.stopped)
                token=commit_token(proposal,state,cert,phase,phase,phase+1,groups,old,cfg,model)
                assert token['context']['physical_budget']==budget
                assert len(token['plans'])==4
                v,d=state['v'][cert['car']],state['delta'][cert['car']]
                interval=execute(model,state,cert['car'],proposal['controls'][0],cfg,cert['source']>0)
                if phase+1 in [j['cut'] for j in cert['joins'][:-1]]:
                    assert state['v'][cert['car']]==0 if a.stopped else state['v'][cert['car']]>0
                accepted+=int(record['accepted']);changed+=int(record['accepted'] and record['changed'])
                minroad=interval['minimum_road_lower_Q'] if minroad is None else min(minroad,interval['minimum_road_lower_Q'])
                minpair=interval['minimum_pair_lower_Q'] if minpair is None else min(minpair,interval['minimum_pair_lower_Q'])
                before=budget;budget-=1
                assert before==budget+1 and budget>=0
                row=dict(phase=phase,actual=interval,budget_before=before,budget_after=budget,
                    committed_plan=proposal['id'],parent_plan=old['id'],history=state['history'],groups=groups,
                    receiver_phase=token['context']['receiver_phase'],optimizer=record)
                if record.get('failure_layer') in ('inherited_anchor','optimizer_program_error'):
                    raise AssertionError('invalid P0 candidate inheritance: '+str(record))
                if record.get('changed'):
                    row['accepted_controls']=proposal['controls'];row['cone_proof']=proposal['proof']
                log.write(json.dumps(row,default=str,separators=(',',':'))+'\n')
                if phase in (0,1,2):versions.append(token)
                if phase+1<cert['ticks']:
                    plan=shift_plan(model,proposal,cert,phase,v,d)
                    assert plan['history']==state['history']
            assert len(rejected)==7
            assert state_in_node(state,cfg,edge['target'],cert['stage_radii'])
            assert budget==coord['initial_U'][edge['target']]
            # Only after actual arrival is another owner admitted. Every default
            # S successor is still feasible under the same witnessed U version.
            next_edge=coord['initial_policy'][edge['target']]
            assert next_edge['source']==edge['target'] and next_edge['actor']!=edge['actor']
            newcert=object_by_id(a.objects,next_edge['object'])
            next_plan=initial_plan(model,state,newcert,next_edge,stopped=True,
                progress=dict(version=2,policy=coord['initial_policy'],U=coord['initial_U'],source_allowance=budget))
            handoff=commit_token(next_plan,state,newcert,0,cert['ticks'],cert['ticks']+1,
                         [['A1','A2','B1'],['B2']])
            before_handoff=budget;nv,nd=state['v'][newcert['car']],state['delta'][newcert['car']]
            actual=execute(model,state,newcert['car'],next_plan['controls'][0],cfg,newcert['source']>0)
            budget-=1
            shifted=shift_plan(model,next_plan,newcert,0,nv,nd)
            commit_token(shifted,state,newcert,1,cert['ticks']+1,cert['ticks']+1,
                         [['A1','A2','B1'],['B2']])
            log.write(json.dumps(dict(phase=0,actor=newcert['car'],object=next_edge['object'],actual=actual,
                budget_before=before_handoff,budget_after=budget,committed_plan=next_plan['id'],parent_plan=next_plan['id'],
                history=state['history'],groups=[['A1','A2','B1'],['B2']],receiver_phase=handoff['context']['receiver_phase'],
                optimizer=dict(accepted=True,changed=False,solver_called=False,status='new-owner-certified-default')),
                default=str,separators=(',',':'))+'\n')
        except Exception as err:
            failure=repr(err)
    result=dict(passed=failure is None,error=failure,elapsed_wall_s=time.perf_counter()-began,
        first_connector_ticks=cert['ticks'],source=edge['source'],receiver=edge['target'],edge=edge,stopped=a.stopped,
        handoff_edge=next_edge if failure is None else None,actual_steps=cert['ticks']+1 if failure is None else None,
        source_budget=source_budget,remaining_budget=budget,rejections=rejected,
        accepted_optimizations=accepted,changed_optimizations=changed,
        minimum_continuous_road_lower_m=float(F(minroad or 0,model.Q)),
        minimum_continuous_pair_lower_m=float(F(minpair or 0,model.Q)),
        first_group_commits=versions,
        actual_final_state={c:dict(pose_Q=pose_rows(state['poses'][c]),v=str(state['v'][c]),delta=str(state['delta'][c])) for c in CARS},
        full_fourcar_task_run=False,formal_batch_started=False,
        independent_full_task_acceptance_run=False)
    write(a.output/'summary.json',result)
    print(json.dumps({k:v for k,v in result.items() if k not in ('first_group_commits','actual_final_state','rejections')},indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
