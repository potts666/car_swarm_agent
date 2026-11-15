#!/usr/bin/env python3
"""Fixed-duration B runs, real solver provenance, rejection fallback and K replay."""
import argparse
import time
from common import *
from derivatives import build,verify
from socp import solve,diagnostic,rows_for


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
    failure=None;accepted=numeric_rejected=0;events=[];resets=[];bad_receipts=[];K_shifts=0;count=0;began=time.perf_counter()
    with gzip.open(args.output/'execution.jsonl.gz','wt') as stream:
        try:
            for phase in range(data.cert['ticks']):
                proposal=old;event=None;damaged=None;candidate_id=None
                if phase==start and args.method!='anchor' or args.shift_recert and phase==start+1:
                    anchor=anchor_for(data,m,state,old);anchor_id=put(HERE/'objects',anchor)
                    if phase==start:
                        assert anchor_id==item['anchor_id'];cert=shared_cert;derivative_id=item['derivative_id'];attempt_subs=[cert['subdivisions']]
                    else:attempt_subs=[8,16,32]
                    records=[]
                    for sub in attempt_subs:
                        if phase!=start:
                            cert=build(data,m,anchor,sub);verify(data,m,cert,anchor);derivative_id=put(HERE/'objects',cert)
                        result=solve(data,m,anchor,cert,args.method);result['derivative_id']=derivative_id
                        result_id=put(HERE/'objects',result);records.append(dict(result_id=result_id,derivative_id=derivative_id,subdivisions=sub,status=result['status'],accepted=result['accepted']))
                        if result['accepted'] or result['status']=='free_box_temporarily_unavailable':break
                    event=dict(phase=phase,anchor_id=anchor_id,attempts=records,accepted=result['accepted'],rejection_layer=result.get('failure_layer'))
                    if result['accepted']:
                        proposal=result['accepted_proposal'];accepted+=1;candidate_id=result_id
                        assert result['candidate_source'].startswith('Clarabel')
                    else:numeric_rejected+=1
                    events.append(event)
                    if phase==start:
                        fixture=get(HERE/'objects',item['A_reference_id']);diag=diagnostic(result['rows'],fixture['dA'])
                        write(args.output/'A_direction_diagnostic.json',diag)
                pending=old['pending_reset']
                if pending is not None and pending['phase']==phase:
                    # A valid fixture over the numerical old tail is corrupted;
                    # its rejection must leave that already accepted K executable.
                    bad=copy.deepcopy(old);rr=data.row(phase+32);a,w=map(F,old['remaining'][32])
                    bad.update(branch='box_candidate',parent=old['id'],receiver={**A.receiver(data.cert,phase),'component':'box','row_sha256':content_id(rr),'domain':rr['domain']},pending_reset=dict(phase=phase+32,next_phase=phase+33,row_sha256=content_id(rr),input=(a+F(1,100),w)))
                    bad=A.seal(bad)
                    try:A.commit(m,data,state,old,bad,phase,phase)
                    except ValueError as error:damaged=dict(kind='K_fixture_while_numerical_old_K_is_due',proposal=bad,rejection=str(error),old_first=A.window(old)[0],old_plan_id=old['id'])
                    else:raise AssertionError('damaged K proposal accepted')
                    bad_receipts.append(dict(phase=phase,old_plan_id=old['id']))
                commit_start=time.perf_counter();token=A.commit(m,data,state,old,proposal,phase,phase);commit_wall=time.perf_counter()-commit_start
                if event and not event['accepted']:assert proposal['id']==old['id']
                active_reset=proposal['pending_reset']
                if active_reset and phase==active_reset['phase']:
                    assert in_box(m,state,c,data.row(phase));assert A.window(proposal)[0]==reset_input(state['v'][c],state['delta'][c],data.row(phase))
                actual=legacy.execute(m,state,c,A.window(proposal)[0],data.cfg,data.cert['source']>0)
                if damaged:assert actual['executed_input']==list(map(str,damaged['old_first']))
                if active_reset and phase==active_reset['phase']:
                    assert in_flow(m,state,data.cert,phase+1);resets.append(dict(phase=phase,actual=actual))
                next_plan,transition=A.shift(data,proposal,state);K_shifts+=transition['kind']=='saved_state_dependent_K'
                stream.write(json.dumps(dict(kind='B',cycle=phase,phase=phase,actor=c,plan=proposal,old_plan_id=old['id'],solver_event=event,accepted_result_id=candidate_id,damaged_proposal=damaged,token=token,commit_wall_s=commit_wall,actual=actual,history=state['history'],budget_before=budget,budget_after=budget-1,shift=transition),default=str,separators=(',',':'))+'\n')
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
    write(args.output/'summary.json',summary);print(json.dumps(summary,default=str,indent=2));return 0 if summary['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
