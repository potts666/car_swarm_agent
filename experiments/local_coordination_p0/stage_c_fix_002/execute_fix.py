#!/usr/bin/env python3
"""Fixed-duration B runs, real solver provenance, rejection fallback and K replay."""
import argparse
import time
from fix_common import *
from derivatives import build,verify
from fix_socp import solve_current
from validate_fix import check
from damage_checks import run_damage


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--index',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--interface',choices=('curve','straight_acceleration'),required=True)
    p.add_argument('--method',choices=('anchor','B-ZG','B-ZL','B-JL'),required=True);p.add_argument('--shift-recert',action='store_true')
    args=p.parse_args()
    if args.output.exists():p.error('preserve previous evidence')
    args.output.mkdir(parents=True);data,m=setup();item=read(args.index)[args.interface]
    shared_anchor=get(HERE/'objects',item['anchor_id']);shared_cert=get(HERE/'objects',item['derivative_id']);start=shared_anchor['phase']
    state=legacy.initial_state(m,data.cfg);old=A.initial_plan(data,state);budget=old['budget'];c=data.cert['car']
    write(args.output/'config.json',dict(interface=args.interface,method=args.method,shared_anchor_id=item['anchor_id'],shared_derivative_id=item['derivative_id'],shift_recert=args.shift_recert,input_manifest_sha256=sha(HERE/'inputs/manifest.json'),prefix_start=start,endpoint=start+32,witness=data.witness_id,source_budget=budget,duration_ticks=239,formal_batch=False))
    failure=None;accepted=numeric_rejected=0;events=[];resets=[];bad_receipts=[];K_shifts=0;count=0;began=time.perf_counter();prefix=[];first_anchor=first_cert=None
    with gzip.open(args.output/'execution.jsonl.gz','wt') as stream:
        try:
            for phase in range(data.cert['ticks']):
                proposal=old;event=None;damaged=None;candidate_id=None
                if phase in (start,start+1):
                    round_number=phase-start+1;anchor=anchor_for(data,m,state,old);anchor_id=put(HERE/'objects',anchor)
                    build_start=time.perf_counter();cert=build(data,m,anchor,32);verify(data,m,cert,anchor,recompute=False);derivative_id=put(HERE/'objects',cert)
                    binding=dict(run_id='two_updates_001',interface=args.interface,round=round_number,phase=phase)
                    raw_id,policy_id,result=solve_current(data,m,state,old,anchor,cert,binding,args.output/('round_'+str(round_number)))
                    check(data,m,anchor,cert,result,recompute=False)
                    event=dict(**binding,history=state['history'],parent_plan_id=old['id'],anchor_id=anchor_id,derivative_id=derivative_id,assembly_id=result['assembly_id'],raw_result_id=raw_id,policy_result_id=policy_id,accepted=result['accepted'],accepted_plan_id=result.get('accepted_proposal',{}).get('id'),commit_id=None,derivative_and_solve_and_cert_wall_s=time.perf_counter()-build_start,failure_layer=result.get('failure_layer'),old_pending_reset=old['pending_reset'])
                    if result['accepted']:
                        proposal=result['accepted_proposal'];accepted+=1;candidate_id=policy_id
                        assert proposal['parent']==old['id'] and proposal['remaining'][33:]==old['remaining'][33:]
                        event['new_pending_reset']=proposal['pending_reset'];event['old_K_disposition']='entire accepted prefix/receiver/K replaces old obligation atomically' if old['pending_reset'] else 'first box contract'
                    else:
                        numeric_rejected+=1;event['new_pending_reset']=old['pending_reset'];event['old_K_disposition']='old actual tail and pending K retained'
                    if round_number==1:first_anchor,first_cert=anchor,cert
                    else:
                        assert old['parent']==events[0]['accepted_plan_id'] if events[0]['accepted'] else True
                        if old['pending_reset']:
                            run_damage(data,m,state,old,anchor,cert,get(HERE/'objects',raw_id),result,first_anchor,first_cert,prefix,args.output/'damage_forks')
                    events.append(event)
                pending=old['pending_reset']
                if pending is not None and pending['phase']==phase:
                    # Damage the saved contract while its real K is due; its ID must reject.
                    bad=copy.deepcopy(old);bad['pending_reset']['input']=[str(F(bad['pending_reset']['input'][0])+F(1,100)),str(bad['pending_reset']['input'][1])]
                    try:A.commit(m,data,state,old,bad,phase,phase)
                    except ValueError as error:damaged=dict(kind='K_fixture_while_numerical_old_K_is_due',proposal=bad,rejection=str(error),old_first=A.window(old)[0],old_plan_id=old['id'])
                    else:raise AssertionError('damaged K proposal accepted')
                    bad_receipts.append(dict(phase=phase,old_plan_id=old['id']))
                commit_start=time.perf_counter();token=A.commit(m,data,state,old,proposal,phase,phase);commit_wall=time.perf_counter()-commit_start;number=validate_commit_token(data,state,old,proposal,token,phase)
                if event:event['commit_id']=number;event['selected_plan_id']=proposal['id'];event['actual_selected_first']=list(map(str,A.window(proposal)[0]));event['old_first']=list(map(str,A.window(old)[0]))
                if event and not event['accepted']:assert proposal['id']==old['id']
                active_reset=proposal['pending_reset']
                if active_reset and phase==active_reset['phase']:
                    assert in_box(m,state,c,data.row(phase));assert A.window(proposal)[0]==reset_input(state['v'][c],state['delta'][c],data.row(phase))
                actual=legacy.execute(m,state,c,A.window(proposal)[0],data.cfg,data.cert['source']>0)
                prefix.append(tuple(map(F,actual['executed_input'])))
                if damaged:assert actual['executed_input']==list(map(str,damaged['old_first']))
                if event:
                    event['actual_output']=actual['executed_input'];event['actual_successor_history']=state['history'];event['actual_first_changed']=event['actual_output']!=event['old_first']
                if active_reset and phase==active_reset['phase']:
                    assert in_flow(m,state,data.cert,phase+1);resets.append(dict(phase=phase,actual=actual))
                next_plan,transition=A.shift(data,proposal,state);K_shifts+=transition['kind']=='saved_state_dependent_K'
                stream.write(json.dumps(dict(kind='B',cycle=phase,phase=phase,actor=c,plan=proposal,old_plan_id=old['id'],solver_event=event,accepted_result_id=candidate_id,damaged_proposal=damaged,token=token,commit_id=number,commit_wall_s=commit_wall,actual=actual,history=state['history'],budget_before=budget,budget_after=budget-1,shift=transition),default=str,separators=(',',':'))+'\n')
                budget-=1;count+=1;old=next_plan
            assert legacy.state_in_node(state,data.cfg,data.edge['target'],data.cert['stage_radii']) and budget==1040
            edge=data.witness['chain'][1];cert=data.objects[edge['object']];assert legacy.state_in_node(state,data.cfg,edge['source'],cert['stage_radii'])
            policy={e['source']:e for e in data.witness['chain']}
            new=legacy.initial_plan(m,state,cert,edge,stopped=True,progress=dict(version=1,policy=policy,U=data.witness['U'],source_allowance=budget))
            token=legacy.commit_token(new,state,cert,0,count,2,A.groups(0,edge['actor']))
            actual=legacy.execute(m,state,edge['actor'],new['controls'][0],data.cfg,cert['source']>0)
            stream.write(json.dumps(dict(kind='parked_handoff',cycle=count,phase=0,actor=edge['actor'],edge=edge,token=token,actual=actual,history=state['history'],budget_before=budget,budget_after=budget-1),default=str,separators=(',',':'))+'\n');budget-=1;count+=1
        except Exception as error:failure=repr(error)
    summary=dict(passed=failure is None,error=failure,interface=args.interface,method=args.method,shift_recert=args.shift_recert,actual_steps=count,remaining_budget=budget,accepted_numeric_candidates=accepted,numeric_proposals_rejected=numeric_rejected,solver_events=events,K_executions=resets,saved_K_shifts=K_shifts,rejected_K_fixtures=bad_receipts,all_numeric_candidate_inputs_from_Clarabel=True,full_fourcar_task=False,formal_batch=False,wall_s=time.perf_counter()-began)
    write(args.output/'events.json',events);write(args.output/'summary.json',summary);print(json.dumps(summary,default=str,indent=2));return 0 if summary['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
