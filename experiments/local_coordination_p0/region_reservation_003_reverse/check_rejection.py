"""Corrupted copied proposals cannot commit; execute the same actual old tail to old K."""
from common import *
from protocol import prepare,commit,select_for_execution
from witness import plan_id
from run_case import advance
from accept_case import Audit


def corrupted_witness(proposed,kind):
    new=copy.deepcopy(get(HERE/'objects',proposed));raw=copy.deepcopy(get(HERE/'objects',new['optimization_origin']));anchor=copy.deepcopy(get(HERE/'objects',raw['anchor_id']));actor=anchor['actor'];k=anchor['phase']
    if kind=='stale_source':anchor['state']['history']='stale:'+anchor['state']['history'];raw['anchor_id']=put(HERE/'objects',anchor)
    elif kind=='wrong_frame':anchor['direction_sign']*=-1;raw['anchor_id']=put(HERE/'objects',anchor)
    elif kind=='wrong_signed_speed':anchor['state']['v'][actor]=str(-F(anchor['state']['v'][actor]));raw['anchor_id']=put(HERE/'objects',anchor)
    elif kind=='missing_negative_bound':
        model=copy.deepcopy(get(HERE/'objects',raw['assembly_id']));index=next(i for i,r in enumerate(model['physical_rows']) if r['name']=='state_0_0_-1');model['physical_rows'][index]['g_upper']=str(F(model['physical_rows'][index]['g_upper'])+1);raw['assembly_id']=put(HERE/'objects',model)
    elif kind=='damaged_derivative':
        cert=copy.deepcopy(get(HERE/'objects',raw['derivative_id']));cert['direction_sign']*=-1;raw['derivative_id']=put(HERE/'objects',cert)
    elif kind=='damaged_K':new['controls'][actor][k+32][0]=str(F(new['controls'][actor][k+32][0])+F(1,10**12))
    else:raise ValueError(kind)
    new['optimization_origin']=put(HERE/'objects',raw);return put(HERE/'objects',new)


def check(side):
    run=HERE/'results'/side;events=read(run/'events.json');event=next(e for e in events if e['kind']=='local_socp');assert event['accepted'],'fault copying needs an admitted numerical proposal'
    anchor=get(HERE/'objects',event['anchor_id']);source=restore(m,anchor['state']);state=copy.deepcopy(source);old=state['witness'];proposed=event['accepted_witness'];parent=plan_id(state,old);out=run/'rejection';assert not out.exists();out.mkdir();records=[]
    req=prepare(state,proposed,parent)
    for kind in ('stale_source','wrong_frame','wrong_signed_speed','missing_negative_bound','damaged_derivative','damaged_K','stale_phase','mixed_commit'):
        before=normal(state_record(state));evidence=None
        try:
            if kind=='stale_phase':
                bad=copy.deepcopy(req);bad['cycle']-=1;evidence=put(HERE/'objects',bad);commit(state,bad)
            elif kind=='mixed_commit':
                token=commit(state,req);token['plans']['A1']='damaged';evidence=put(HERE/'objects',token);select_for_execution(state,token)
            else:
                bad_wid=corrupted_witness(proposed,kind);evidence=bad_wid;bad=prepare(state,bad_wid,parent);commit(state,bad)
        except (AssertionError,ValueError) as error:records.append(dict(kind=kind,rejected=True,reason=repr(error),evidence_id=evidence,state_unchanged=before==normal(state_record(state))))
        else:raise AssertionError('invalid proposal admitted: '+kind)
        assert before==normal(state_record(state))
    # All corruptions refer to the same immutable checkpoint. One real old-tail fork
    # proves continuation after the rejected proposals, without duplicate identical runs.
    w=get(HERE/'objects',old);actor=anchor['actor'];k=source['cycle'];target=min(int(t) for t,ev in w['events'].items() if int(t)>k and [actor,'G_in'] in ev);steps=target+1-k
    audit=Audit(read(run/'config.json')['initial_witness'])
    with gzip.open(run/'execution.jsonl.gz','rt') as stream:
        for row in map(json.loads,stream):
            if row['cycle']==k:break
            audit.frame(row)
    assert normal(state_record(audit.state))==normal(state_record(source))
    with gzip.open(out/'execution.jsonl.gz','wt') as stream:
        for j in range(steps):
            row=advance(state,old);audit.frame(normal(row));stream.write(json.dumps(row,default=str,separators=(',',':'))+'\n')
    assert audit.state['history']==state['history'] and state['v'][actor]==F(1,2) and state['delta'][actor]==0
    assert source['v'][actor]<0 and all(r['state_unchanged'] for r in records)
    result=dict(passed=True,side=side,source=state_record(source),old_witness=old,proposed_witness=proposed,rejections=records,invalid_commits=0,invalid_selections=0,valid_token_built_for_mixed_token_test=1,actual_fallback_branches=1,actual_steps=steps,old_G_in_phase=target,old_K_phase=target,old_K_command=w['controls'][actor][target],final=state_record(state),budget_before=2802-k,budget_after=2802-state['cycle'],independent_8_subsegments=True,minimum_road=str(audit.road),minimum_pair=str(audit.pair))
    write(out/'summary.json',result);return dict(side=side,passed=True,rejections=len(records),actual_steps=steps)
if __name__=='__main__':
    result=[check(side) for side in ('A','B')];write(HERE/'results/rejection_checks.json',result);print(json.dumps(result))
