"""Independent event-ID provenance, two actual anchors, atomic contracts and fork replay."""
import argparse,time
from fix_common import *
from validate_fix import check


def normal(value):return json.loads(json.dumps(value,default=str))


def shifted(plan,audit,data,phase):
    new=copy.deepcopy(plan);nextphase=phase+1;end=min(nextphase+32,239);c=data.cert['car'];pending=plan['pending_reset']
    new.update(branch='locked_derived_suffix' if plan['ancestry'] else 'locked_inherited',phase=nextphase,parent=plan['id'],remaining=plan['remaining'][1:],budget=plan['budget']-1,source_history=audit.history,source_pose_Q=pose_rows(audit.p4[c]),v0=audit.v[c],delta0=audit.d[c],receiver=dict(component='flow' if end<239 else 'coarse_stopped',phase=end,domain_object=data.cert['_object_id'],deadline=end-nextphase))
    if pending and pending['phase']<nextphase:new['pending_reset']=None
    return normal(A.seal(new))


def validate_token(data,audit,plan,old,token,phase):
    ctx=token['context'];c=data.cert['car']
    expected=dict(cycle=phase,history=audit.history,state_time_s=str(phase*H),owner=c,permission_version=data.witness['version'],joint_source=data.edge['source'],joint_target=data.edge['target'],groups=A.groups(phase,c),pair_contracts=__import__('moving_objects').pair_partition(A.groups(phase,c)),receiver=plan['receiver'],pending_reset=plan['pending_reset'],witness=plan['witness'],witness_version=plan['witness_version'],budget=plan['budget'],plan_id=plan['id'])
    assert normal(expected)==ctx,'independent mixed commit context'
    kind='locked_inherited' if plan['id']==old['id'] else 'box_prefix_and_actual_K';ids={}
    for car in CARS:
        ids[car]=content_id(dict(car=car,context=ctx,controls=A.window(plan) if car==c else [(F(0),F(0))]*32,certificate=dict(plan_id=plan['id'],witness=plan['witness'],kind=kind)))
    assert ids==token['plans'] and token['application_messages']==40
    return content_id(dict(context=ctx,plans=ids))


def validate_new(data,m,audit,plan,old,phase):
    assert A.seal(plan)['id']==plan['id'] and plan['parent']==old['id']
    r=plan['receiver'];row=data.row(phase+32);reset=plan['pending_reset'];c=data.cert['car']
    assert r['component']=='box' and r['domain_object']==data.cert['_object_id'] and r['phase']==phase+32 and r['deadline']==32
    assert r['domain']==row['domain'] and r['row_sha256']==content_id(row)
    assert reset['phase']==phase+32 and reset['next_phase']==phase+33 and reset['row_sha256']==content_id(row)
    commands=[tuple(map(F,u)) for u in plan['remaining']];base=[tuple(map(F,u)) for u in old['remaining']]
    assert len(commands)==len(base) and commands[33:]==base[33:]
    assert H*sum(a*a+w*w for a,w in commands[:32])<=H*sum(a*a+w*w for a,w in base[:32])
    v=audit.v[c]+H*sum(a for a,w in commands[:32]);d=audit.d[c]+H*sum(w for a,w in commands[:32]);domain=row['domain']
    K=(F(domain['feedback_a_ref'])-(v-F(domain['center_v']))/H,F(domain['feedback_omega_ref'])-(d-F(domain['center_delta']))/H)
    assert commands[32]==tuple(map(F,reset['input']))==K,'independent live K mismatch'
    assert plan['budget']==len(commands)+plan['suffix_ticks']<=old['budget']
    cloned=audit.clone()
    for j,u in enumerate(commands[:33]):
        if j==32:cloned.box(c,row)
        cloned.step(c,u,data.cert['source']>0)
    cloned.flow(c,data.cert,phase+33)


def replay_step(data,m,audit,expected,row):
    phase=row['phase'];plan=row['plan'];c=data.cert['car'];budget=expected['budget']
    assert plan['object']==data.cert['_object_id'] and plan['phase']==phase and plan['source_history']==audit.history
    assert A.seal(plan)['id']==plan['id'] and plan['source_pose_Q']==pose_rows(audit.p4[c]) and F(plan['v0'])==audit.v[c] and F(plan['delta0'])==audit.d[c]
    assert plan['budget']==budget and plan['witness']==data.witness_id and plan['witness_version']==data.witness['version'] and plan['suffix_ticks']==data.suffix_ticks
    assert row['commit_id']==validate_token(data,audit,plan,expected,row['token'],phase)
    pending=plan['pending_reset']
    if pending and phase==pending['phase']:
        audit.box(c,data.row(phase));domain=data.row(phase)['domain']
        K=(F(domain['feedback_a_ref'])-(audit.v[c]-F(domain['center_v']))/H,F(domain['feedback_omega_ref'])-(audit.d[c]-F(domain['center_delta']))/H)
        assert A.window(plan)[0]==K
    audit.step(c,A.window(plan)[0],data.cert['source']>0,row['actual'])
    if pending and phase==pending['phase']:audit.flow(c,data.cert,phase+1)
    nextplan=shifted(plan,audit,data,phase);tail=tuple(map(F,plan['remaining'][32])) if len(plan['remaining'])>32 else (F(0),F(0))
    assert row['shift']['new_id']==nextplan['id'] and tuple(map(F,row['shift']['appended_input']))==tail
    assert A.window(nextplan)==A.window(plan)[1:]+[tail]
    assert row['history']==audit.history and row['budget_before']==budget and row['budget_after']==budget-1
    return nextplan,bool(pending and phase==pending['phase'])


def accept_damage(run,data,m,source_audit,anchor,old):
    summary=read(run/'damage_forks/summary.json');assert summary['passed'];c=data.cert['car'];verified=[]
    assert {case['kind'] for case in summary['cases']}=={'stale_anchor','stale_derivative','old_phase','damaged_K','mixed_commit_context'}
    for case in summary['cases']:
        req=case['request'];kind=case['kind'];assert req['kind']==kind and case['source_anchor_id']==content_id(anchor) and case['old_plan_id']==old['id']
        assert content_id(case['old_pending_reset'])==content_id(old['pending_reset']) and case['invalid_commits']==case['invalid_executions']==0
        if kind=='stale_anchor':assert content_id(get(HERE/'objects',req['anchor_id']))!=content_id(anchor)
        elif kind=='stale_derivative':assert get(HERE/'objects',req['derivative_id'])['anchor_id']!=content_id(anchor)
        elif kind=='old_phase':assert get(HERE/'objects',req['proposal_id'])['phase']!=old['phase']
        elif kind=='damaged_K':
            bad=get(HERE/'objects',req['proposal_id']);domain=data.row(old['phase']+32)['domain'];u=A.window(bad)
            v=source_audit.v[c]+H*sum(a for a,w in u);d=source_audit.d[c]+H*sum(w for a,w in u)
            K=(F(domain['feedback_a_ref'])-(v-F(domain['center_v']))/H,F(domain['feedback_omega_ref'])-(d-F(domain['center_delta']))/H)
            assert tuple(map(F,bad['pending_reset']['input']))!=K
        else:
            template=get(HERE/'objects',req['template_plan_id'])
            assert req['token']['context']['plan_id']!=template['id']
        audit=source_audit.clone();expected=copy.deepcopy(old);count=K_count=0
        with gzip.open(run/'damage_forks'/case['log_file'],'rt') as stream:
            for row in map(json.loads,stream):
                assert row['phase']==old['phase']+count and row['plan']==normal(expected)
                expected,Kdone=replay_step(data,m,audit,expected,row);K_count+=Kdone;count+=1
                if count==1:assert row['actual']['executed_input']==case['old_first']==case['first_actual_input']
        assert K_count==1 and count==case['actual_steps'] and audit.history==case['final_history']
        assert expected['budget']==case['budget_after']==old['budget']-count
        verified.append(dict(kind=kind,passed=True,actual_steps=count,actual_old_K=K_count))
    return dict(passed=True,cases=verified)


def accept(run):
    data,m=setup();cfg=read(run/'config.json');summary=read(run/'summary.json');audit=Replay(m,data.cfg);audit.node(data.edge['source'],data.cert['stage_radii'])
    expected=normal(A.initial_plan(data,legacy.initial_state(m,data.cfg)));budget=1279;events=[];count=accepted=resets=damaged_due=0;start=time.perf_counter();forks=None
    with gzip.open(run/'execution.jsonl.gz','rt') as stream:
        for row in map(json.loads,stream):
            assert row['cycle']==count
            if row['kind']=='parked_handoff':
                assert count==239 and row['edge']==data.witness['chain'][1];audit.node(data.edge['target'],data.cert['stage_radii'])
                edge=row['edge'];cert=data.objects[edge['object']];assert row['token']['context']['physical_budget']==budget
                audit.step(edge['actor'],tuple(map(F,cert['inputs'][0])),cert['source']>0,row['actual'])
                assert row['history']==audit.history and row['budget_before']==budget and row['budget_after']==budget-1
            else:
                phase=row['phase'];assert phase==count and row['old_plan_id']==expected['id'];plan=row['plan'];event=row['solver_event']
                if event:
                    anchor=anchor_for(data,m,dict(poses=audit.p4,v=audit.v,delta=audit.d,history=audit.history),expected)
                    assert event['anchor_id']==content_id(anchor) and event['history']==audit.history and event['parent_plan_id']==expected['id']
                    assert event['run_id']=='two_updates_001' and event['interface']==cfg['interface'] and event['round']==len(events)+1 and event['phase']==phase
                    cert=get(HERE/'objects',event['derivative_id']);raw=get(HERE/'objects',event['raw_result_id']);result=get(HERE/'objects',event['policy_result_id'])
                    assert result['raw_result_id']==event['raw_result_id'] and result['assembly_id']==event['assembly_id'] and result['accepted']==event['accepted']
                    for key in ('run_id','interface','round','phase','anchor_id','derivative_id','history','parent_plan_id'):assert result[key]==event[key]
                    check(data,m,anchor,cert,result,plan if result['accepted'] else None,recompute=True)
                    assert raw['anchor_id']==content_id(anchor)
                    if len(events)==1:
                        assert phase==events[0]['phase']+1 and anchor['plan']['parent']==events[0]['accepted_plan_id']
                        assert event['anchor_id']!=events[0]['anchor_id'] and event['derivative_id']!=events[0]['derivative_id'] and event['assembly_id']!=events[0]['assembly_id']
                        forks=accept_damage(run,data,m,audit.clone(),anchor,expected)
                    if result['accepted']:
                        assert row['accepted_result_id']==event['policy_result_id'] and event['accepted_plan_id']==plan['id'];validate_new(data,m,audit,plan,expected,phase);accepted+=1
                        assert content_id(event['new_pending_reset'])==content_id(plan['pending_reset'])
                    else:assert plan==expected and row['accepted_result_id'] is None
                    assert event['selected_plan_id']==plan['id'] and event['commit_id']==row['commit_id'] and event['actual_output']==row['actual']['executed_input']
                    assert event['old_first']==list(map(str,A.window(expected)[0])) and event['actual_first_changed']==(event['old_first']!=event['actual_output'])
                    assert content_id(event['old_pending_reset'])==content_id(expected['pending_reset']);events.append(event)
                else:assert plan==expected
                if row['damaged_proposal']:
                    bad=row['damaged_proposal'];assert bad['old_plan_id']==expected['id'] and A.seal(bad['proposal'])['id']!=bad['proposal']['id']
                    assert expected['pending_reset']['phase']==phase and bad['old_first']==list(map(str,A.window(expected)[0]));damaged_due+=1
                expected,Kdone=replay_step(data,m,audit,expected,row);resets+=Kdone
                if event:assert event['actual_successor_history']==audit.history
            budget-=1;count+=1
    assert summary['passed'] and count==summary['actual_steps']==240 and budget==summary['remaining_budget']==1039
    assert accepted==summary['accepted_numeric_candidates'] and resets==len(summary['K_executions']) and damaged_due==len(summary['rejected_K_fixtures'])
    assert len(events)==2 and [e['phase'] for e in events]==[cfg['prefix_start'],cfg['prefix_start']+1]
    assert events==read(run/'events.json') and forks and forks['passed']
    return dict(passed=True,actual_steps=count,solver_events=len(events),accepted_numeric_candidates=accepted,continuous_two_updates_accepted=accepted==2,actual_K_executions=resets,damaged_K_due_rejections=damaged_due,damage_forks=forks,remaining_budget=budget,reconstructed_completion_witness=data.witness_id,minimum_road_lower_m=float(F(audit.road,m.Q)),minimum_pair_lower_m=float(F(audit.pair,m.Q)),wall_s=time.perf_counter()-start,shared=['interval arithmetic/trigonometry/propagation','certified derivative generator','identity and window helpers'],independent=['actual anchors and per-event provenance','terminal rows and raw-to-rational commands','four public plans/atomic contexts','all continuous road/six-pair envelopes','K, inherited suffix shifts, budgets and controlled rejection forks'])


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);args=p.parse_args();dest=args.run/'independent_acceptance.json';assert not dest.exists()
    try:result=accept(args.run)
    except Exception as error:write(dest,dict(passed=False,error=repr(error)));raise
    write(dest,result);print(json.dumps(result,indent=2))
if __name__=='__main__':main()
