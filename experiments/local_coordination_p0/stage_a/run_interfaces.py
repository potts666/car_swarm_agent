#!/usr/bin/env python3
"""Two bounded fixed-duration interface runs. Never launch a full experiment batch."""
import argparse
import copy
from fractions import Fraction as F
import gzip
import json
from pathlib import Path
import shutil
import time

from region_certificates import Certificates,CARS,controls,write,read,sha,in_box,in_flow,content_id
from region_control import initial_plan,construct,commit,shift,seal,groups,window
import pilot_control as legacy


def run(args):
    out=args.output;out.mkdir(parents=True)
    data=Certificates(args.inputs);model=legacy.load_model(args.rebuild);state=legacy.initial_state(model,data.cfg)
    cert=data.cert;c=cert['car'];old=initial_plan(data,state)
    assert legacy.state_in_node(state,data.cfg,data.edge['source'],cert['stage_radii'])
    endpoint=37 if args.interface=='curve' else 42;start=endpoint-32
    source_budget=old['budget'];budget=source_budget;rejections=[];receipt=None;shifts=[]
    for f in Path(__file__).parent.glob('*.py'):shutil.copy2(f,out/f.name)
    write(out/'config.json',dict(interface=args.interface,endpoint=endpoint,prefix_start=start,prefix_ticks=32,
        inputs=str(args.inputs.resolve()),source_rebuild=str(args.rebuild.resolve()),manifest_sha256=sha(args.inputs/'manifest.json'),
        edge=data.edge,witness=data.witness_id,source_budget=source_budget,source_budget_kind='current accepted W and same-version actual suffix; not C/R',
        duration_ticks=cert['ticks'],solver_derivatives_connected=False,construction_only=True,
        objective='same normalized prefix input energy',groups='unchanged phase%64 rule',permission='unchanged parked inter-car handoff',formal_batch=False))
    failure=None;minroad=minpair=100*model.Q;began=time.perf_counter();actual_steps=0;box_state=reset_state=None
    with gzip.open(out/'execution.jsonl.gz','wt') as stream:
        try:
            for phase in range(cert['ticks']):
                rejected=None;proposal=old;proof=None
                if phase<5 or phase==endpoint:
                    legal,_=construct(model,data,state,old,phase);bad=copy.deepcopy(legal)
                    kind=('K','phase','object','domain','deadline')[phase] if phase<5 else 'K_after_box'
                    if kind in ('K','K_after_box'):bad['pending_reset']['input']=(F(bad['pending_reset']['input'][0])+F(1,100),F(bad['pending_reset']['input'][1]))
                    if kind=='phase':bad['receiver']['phase']-=1
                    if kind=='object':bad['receiver']['domain_object']='wrong-object'
                    if kind=='domain':bad['receiver']['domain']['position_l1_radius']='1'
                    if kind=='deadline':bad['receiver']['deadline']-=1
                    bad=seal(bad)
                    try:commit(model,data,state,old,bad,phase,phase)
                    except ValueError as error:rejected=dict(kind=kind,reason=str(error),proposal=bad,old_plan_id=old['id'],old_first_input=window(old)[0])
                    else:raise AssertionError('damaged proposal accepted: '+kind)
                    rejections.append({k:v for k,v in rejected.items() if k!='proposal'})
                if phase==start:
                    proposal,proof=construct(model,data,state,old,phase);receipt=copy.deepcopy(proposal['pending_reset'])
                token=commit(model,data,state,old,proposal,phase,phase)
                assert token['context']['budget']==budget
                if phase==endpoint:
                    assert in_box(model,state,c,data.row(endpoint));box_state=dict(pose_Q=legacy.pose_rows(state['poses'][c]),v=str(state['v'][c]),delta=str(state['delta'][c]))
                    assert state['v'][c]!=F(data.row(endpoint)['nominal_v']) and state['delta'][c]!=F(data.row(endpoint)['nominal_delta'])
                actual=legacy.execute(model,state,c,window(proposal)[0],data.cfg,cert['source']>0)
                if phase==endpoint:
                    assert in_flow(model,state,cert,endpoint+1);reset_state=actual
                before=budget;budget-=1;actual_steps+=1
                minroad=min(minroad,actual['minimum_road_lower_Q']);minpair=min(minpair,actual['minimum_pair_lower_Q'])
                if rejected:assert actual['executed_input']==list(map(str,rejected['old_first_input']))
                next_plan,transition=shift(data,proposal,state)
                assert next_plan['budget']==budget
                if transition['kind']=='saved_state_dependent_K':shifts.append(transition)
                row=dict(cycle=phase,phase=phase,kind='stage_a',actor=c,plan=proposal,old_plan_id=old['id'],rejected=rejected,
                    acceptance_proof=proof,token=token,actual=actual,history=state['history'],budget_before=before,budget_after=budget,shift=transition)
                stream.write(json.dumps(row,default=str,separators=(',',':'))+'\n');old=next_plan
            assert legacy.state_in_node(state,data.cfg,data.edge['target'],cert['stage_radii'])
            assert budget==data.suffix_ticks and len(rejections)==6 and box_state and reset_state and len(shifts)==1
            # The existing parked-owner protocol is used without any new motion permission.
            edge=data.witness['chain'][1];nextcert=data.objects[edge['object']]
            assert edge['source']==data.edge['target'] and edge['actor']!=c
            assert legacy.state_in_node(state,data.cfg,edge['source'],nextcert['stage_radii'])
            policy={e['source']:e for e in data.witness['chain']}
            plan=legacy.initial_plan(model,state,nextcert,edge,stopped=True,progress=dict(version=data.witness['version'],policy=policy,U=data.witness['U'],source_allowance=budget))
            token=legacy.commit_token(plan,state,nextcert,0,actual_steps,data.witness['version']+1,groups(0,edge['actor']))
            actual=legacy.execute(model,state,edge['actor'],plan['controls'][0],data.cfg,nextcert['source']>0)
            stream.write(json.dumps(dict(kind='parked_handoff',cycle=actual_steps,phase=0,actor=edge['actor'],edge=edge,
                actual=actual,history=state['history'],budget_before=budget,budget_after=budget-1,token=token),default=str,separators=(',',':'))+'\n')
            budget-=1;actual_steps+=1
        except Exception as error:failure=repr(error)
    summary=dict(passed=failure is None,error=failure,interface=args.interface,endpoint=endpoint,actual_steps=actual_steps,
        source_budget=source_budget,remaining_budget=budget,coarse_connection_ticks=cert['ticks'],duration_changed=False,
        accepted_box_candidates=1 if receipt else 0,constructed_candidate=True,optimizer_called=False,
        actual_box_state=box_state,actual_K_reset_state=reset_state,nonzero_terminal_v_delta=box_state is not None,
        shift_appended_saved_K=shifts,corrupt_proposal_rejections=rejections,
        minimum_online_road_m=float(F(minroad,model.Q)),minimum_online_pair_m=float(F(minpair,model.Q)),
        parked_handoff_executed=failure is None,wall_s=time.perf_counter()-began,
        full_fourcar_task=False,formal_batch=False,independent_acceptance_pending=True)
    write(out/'summary.json',summary);print(json.dumps(summary,default=str,indent=2));return 0 if summary['passed'] else 1


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('inputs','rebuild','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--interface',choices=('curve','straight_acceleration'),required=True)
    a=p.parse_args()
    if a.output.exists():p.error('preserve existing results; use a new directory')
    return run(a)


if __name__=='__main__':raise SystemExit(main())
