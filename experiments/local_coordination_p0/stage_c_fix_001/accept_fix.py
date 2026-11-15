#!/usr/bin/env python3
"""External B replay, numerical provenance and flexible multi-update acceptance."""
import argparse
import time
from fix_common import *
from validate_fix import check


def accept(run):
    data,m=setup();cfg=read(run/'config.json');summary=read(run/'summary.json');audit=Replay(m,data.cfg);audit.node(data.edge['source'],data.cert['stage_radii'])
    initial=A.initial_plan(data,legacy.initial_state(m,data.cfg));expected=json.loads(json.dumps(initial,default=str));budget=1279;accepted=resets=rejected=0;pending=None;seen=set();recert=[];count=0;start=time.perf_counter()
    def validate_new(plan,old,phase):
        assert A.seal(plan)['id']==plan['id'] and plan['parent']==old['id']
        r=plan['receiver'];row=data.row(phase+32);reset=plan['pending_reset'];c=data.cert['car']
        assert r['component']=='box' and r['domain_object']==data.cert['_object_id'] and r['phase']==phase+32 and r['deadline']==32
        assert r['domain']==row['domain'] and r['row_sha256']==content_id(row)
        assert reset['phase']==phase+32 and reset['next_phase']==phase+33 and reset['row_sha256']==content_id(row)
        commands=[tuple(map(F,u)) for u in plan['remaining']];base=[tuple(map(F,u)) for u in old['remaining']]
        assert len(commands)==len(base) and commands[33:]==base[33:]
        assert sum(a*a+w*w for a,w in commands[:32])<=sum(a*a+w*w for a,w in base[:32])
        v=audit.v[c]+H*sum(a for a,w in commands[:32]);d=audit.d[c]+H*sum(w for a,w in commands[:32]);domain=row['domain']
        K=(F(domain['feedback_a_ref'])-(v-F(domain['center_v']))/H,F(domain['feedback_omega_ref'])-(d-F(domain['center_delta']))/H)
        assert commands[32]==tuple(map(F,reset['input']))==K,'independent live K mismatch'
        cloned=audit.clone()
        for j,u in enumerate(commands[:33]):
            if j==32:cloned.box(c,row)
            cloned.step(c,u,data.cert['source']>0)
        cloned.flow(c,data.cert,phase+33)
    with gzip.open(run/'execution.jsonl.gz','rt') as f:
        for row in map(json.loads,f):
            assert row['cycle']==count
            if row['kind']=='parked_handoff':
                assert count==239 and row['edge']==data.witness['chain'][1]
                audit.node(data.edge['target'],data.cert['stage_radii']);edge=row['edge'];cert=data.objects[edge['object']]
                assert row['token']['context']['physical_budget']==budget
                audit.step(edge['actor'],tuple(map(F,cert['inputs'][0])),cert['source']>0,row['actual'])
            else:
                phase=row['phase'];assert phase==count and row['old_plan_id']==expected['id'];plan=row['plan'];c=data.cert['car'];event=row['solver_event']
                if event and event.get('kind')=='actual_shift_recertification':
                    from derivatives import verify
                    from socp import rows_for,evaluate
                    anchor=anchor_for(data,m,dict(poses=audit.p4,v=audit.v,delta=audit.d,history=audit.history),expected)
                    assert content_id(anchor)==event['anchor_id']
                    cert=get(HERE/'objects',event['derivative_id']);verify(data,m,cert,anchor)
                    rows=rows_for(data,m,anchor,cert,'B-JL');zero=evaluate(rows,[F(0)]*64)
                    assert content_id(zero)==content_id(event['free_box_zero_check'])
                    assert event['inherited_plan_id']==expected['id'] and event['remaining_budget']==budget
                    trial=audit.clone()
                    for j,u in enumerate(A.window(expected)):
                        if expected['pending_reset'] and phase+j==expected['pending_reset']['phase']:trial.box(c,data.row(phase+j))
                        trial.step(c,u,data.cert['source']>0)
                    trial.flow(c,data.cert,phase+32)
                    recert.append(dict(phase=phase,new_anchor_id=content_id(anchor),different_from_initial=content_id(anchor)!=cfg['shared_anchor_id'],inherited_actual_tail_verified=True,free_box_available=zero['passed']))
                    assert plan['id']==expected['id'] and row['accepted_result_id'] is None
                elif event:
                    anchor=anchor_for(data,m,dict(poses=audit.p4,v=audit.v,delta=audit.d,history=audit.history),expected)
                    assert event['anchor_id']==content_id(anchor)
                    if phase==cfg['prefix_start']:assert event['anchor_id']==cfg['shared_anchor_id']
                    else:recert.append(dict(phase=phase,new_anchor_id=event['anchor_id'],different_from_initial=event['anchor_id']!=cfg['shared_anchor_id']))
                    result=None
                    for attempt in event['attempts']:
                        result=get(HERE/'objects',attempt['result_id']);cert=get(HERE/'objects',attempt['derivative_id'])
                        assert result['derivative_id']==attempt['derivative_id'] and result['accepted']==attempt['accepted']
                        check(data,m,anchor,cert,result,plan if result['accepted'] else None,recompute=attempt['derivative_id'] not in seen);seen.add(attempt['derivative_id'])
                    assert result['accepted']==event['accepted']
                    if result['accepted']:
                        assert row['accepted_result_id']==event['attempts'][-1]['result_id'];validate_new(plan,expected,phase);accepted+=1
                    else:assert plan['id']==expected['id'] and row['accepted_result_id'] is None
                else:assert plan['id']==expected['id']
                if plan['id']==expected['id']:assert plan==expected
                assert plan['object']==data.cert['_object_id'] and plan['phase']==phase and plan['source_history']==audit.history
                assert plan['source_pose_Q']==pose_rows(audit.p4[c]) and F(plan['v0'])==audit.v[c] and F(plan['delta0'])==audit.d[c]
                assert plan['budget']==budget and plan['witness']==data.witness_id and plan['suffix_ticks']==1040
                if row['damaged_proposal']:
                    bad=row['damaged_proposal'];assert bad['old_plan_id']==expected['id']
                    try:validate_new(bad['proposal'],expected,phase)
                    except (AssertionError,ValueError):rejected+=1
                    else:raise AssertionError('damaged proposal independently accepted')
                    assert tuple(map(F,bad['old_first']))==A.window(expected)[0]==A.window(plan)[0]
                    assert expected['pending_reset']['phase']==phase,'not an actual saved-K fallback'
                token=row['token'];ctx=token['context'];assert ctx['history']==audit.history and ctx['owner']==c and ctx['groups']==A.groups(phase,c)
                assert ctx['pair_contracts']==json.loads(json.dumps(__import__('moving_objects').pair_partition(A.groups(phase,c))))
                assert ctx['receiver']==plan['receiver'] and ctx['pending_reset']==plan['pending_reset'] and ctx['budget']==budget and ctx['witness']==data.witness_id and ctx['plan_id']==plan['id']
                assert set(token['plans'])==set(CARS) and token['application_messages']==40
                pending=plan['pending_reset']
                if pending and pending['phase']==phase:
                    audit.box(c,data.row(phase));d=data.row(phase)['domain']
                    live=(F(d['feedback_a_ref'])-(audit.v[c]-F(d['center_v']))/H,F(d['feedback_omega_ref'])-(audit.d[c]-F(d['center_delta']))/H)
                    assert A.window(plan)[0]==live
                audit.step(c,A.window(plan)[0],data.cert['source']>0,row['actual'])
                if pending and pending['phase']==phase:audit.flow(c,data.cert,phase+1);resets+=1
                # Rebuild the exact actual tail independently of the controller's shift function.
                nextplan=copy.deepcopy(plan);nextphase=phase+1;end=min(nextphase+32,239)
                nextplan.update(branch='locked_derived_suffix' if plan['ancestry'] else 'locked_inherited',phase=nextphase,parent=plan['id'],remaining=plan['remaining'][1:],budget=budget-1,source_history=audit.history,source_pose_Q=pose_rows(audit.p4[c]),v0=audit.v[c],delta0=audit.d[c],receiver=dict(component='flow' if end<239 else 'coarse_stopped',phase=end,domain_object=data.cert['_object_id'],deadline=end-nextphase))
                if pending and pending['phase']<nextphase:nextplan['pending_reset']=None
                nextplan=A.seal(nextplan);assert row['shift']['new_id']==nextplan['id']
                tail=tuple(map(F,plan['remaining'][32])) if len(plan['remaining'])>32 else (F(0),F(0))
                assert tuple(map(F,row['shift']['appended_input']))==tail and A.window(nextplan)==A.window(plan)[1:]+[tail]
                expected=json.loads(json.dumps(nextplan,default=str))
            assert row['history']==audit.history and row['budget_before']==budget and row['budget_after']==budget-1
            budget-=1;count+=1
    assert summary['passed'] and count==summary['actual_steps']==240 and budget==summary['remaining_budget']==1039
    assert accepted==summary['accepted_numeric_candidates'] and resets==len(summary['K_executions'])
    assert rejected==len(summary['rejected_K_fixtures'])
    return dict(passed=True,actual_steps=count,accepted_numeric_candidates=accepted,actual_K_executions=resets,damaged_K_proposals_rejected_with_actual_old_K=rejected,remaining_budget=budget,reconstructed_completion_witness=data.witness_id,shift_recertifications=recert,minimum_road_lower_m=float(F(audit.road,m.Q)),minimum_pair_lower_m=float(F(audit.pair,m.Q)),wall_s=time.perf_counter()-start,shared=['interval arithmetic, trigonometry, propagation','certified variational generator and derivative replay','A identity helpers'],independent=['terminal Taylor parameter derivation','raw-to-rational input provenance','bodies, road, six pairs, permissions','actual boxes/K/flow, shift, completion witness and budget'])


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();dest=a.run/'independent_acceptance.json'
    if dest.exists():p.error('preserve acceptance evidence')
    try:value=accept(a.run)
    except Exception as error:write(dest,dict(passed=False,error=repr(error)));raise
    write(dest,value);print(json.dumps(value,indent=2))


if __name__=='__main__':main()
