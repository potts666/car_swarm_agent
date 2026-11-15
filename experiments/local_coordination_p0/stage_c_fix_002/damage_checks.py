"""Copied second-round forks: reject damage, then execute the already accepted tail."""
from fix_common import *
from fix_socp import admit_current


def run_damage(data,m,state,old,anchor,cert,raw,policy,first_anchor,first_cert,prefix,output):
    assert old['pending_reset'] is not None,'first update did not establish a saved K'
    output.mkdir();c=data.cert['car'];template=policy.get('accepted_proposal')
    if template is None:
        template=numerical_plan(data,state,old,A.window(old));A.validate_candidate(m,data,state,old,template,old['phase'])
    audits=Replay(m,data.cfg)
    for command in prefix:audits.step(c,command,data.cert['source']>0)
    assert audits.history==state['history']
    cases=[]
    for kind in ('stale_anchor','stale_derivative','old_phase','damaged_K','mixed_commit_context'):
        staged=copy.deepcopy(state);saved=copy.deepcopy(old);request=dict(kind=kind,anchor_id=content_id(anchor),derivative_id=content_id(cert),raw_result_id=put(HERE/'objects',raw),template_plan_id=put(HERE/'objects',template))
        try:
            if kind=='stale_anchor':
                request['anchor_id']=put(HERE/'objects',first_anchor);admit_current(data,m,staged,saved,first_anchor,cert,raw)
            elif kind=='stale_derivative':
                request['derivative_id']=put(HERE/'objects',first_cert);admit_current(data,m,staged,saved,anchor,first_cert,raw)
            elif kind in ('old_phase','damaged_K'):
                bad=copy.deepcopy(template)
                if kind=='old_phase':bad['phase']-=1
                else:bad['pending_reset']['input']=[str(F(bad['pending_reset']['input'][0])+F(1,100)),str(bad['pending_reset']['input'][1])]
                bad=A.seal(bad);request['proposal_id']=put(HERE/'objects',bad);A.commit(m,data,staged,saved,bad,saved['phase'],saved['phase'])
            else:
                # A complete old commit cannot select the new receiver/permission combination.
                token=A.commit(m,data,staged,saved,saved,saved['phase'],saved['phase']);request['token']=token
                validate_commit_token(data,staged,saved,template,token,saved['phase'])
        except (ValueError,AssertionError) as error:rejection=str(error)
        else:raise AssertionError('damaged request was allowed to execute: '+kind)
        assert content_id(state_record(staged))==content_id(state_record(state)) and content_id(saved)==content_id(old)
        audit=audits.clone();pending=copy.deepcopy(saved['pending_reset']);stop=pending['phase']+1;records=[]
        for phase in range(saved['phase'],stop):
            proposal=saved;token=A.commit(m,data,staged,saved,proposal,phase,phase);number=validate_commit_token(data,staged,saved,proposal,token,phase)
            if phase==pending['phase']:assert in_box(m,staged,c,data.row(phase));audit.box(c,data.row(phase))
            actual=legacy.execute(m,staged,c,A.window(proposal)[0],data.cfg,data.cert['source']>0);audit.step(c,A.window(proposal)[0],data.cert['source']>0,actual)
            if phase==pending['phase']:
                assert in_flow(m,staged,data.cert,phase+1);audit.flow(c,data.cert,phase+1)
                assert actual['executed_input']==list(map(str,pending['input']))
            saved,shift=A.shift(data,proposal,staged)
            records.append(dict(phase=phase,plan=proposal,token=token,commit_id=number,actual=actual,history=staged['history'],budget_before=proposal['budget'],budget_after=saved['budget'],shift=shift))
        path=output/(kind+'.jsonl.gz')
        with gzip.open(path,'wt') as stream:
            for record in records:stream.write(json.dumps(record,default=str,separators=(',',':'))+'\n')
        cases.append(dict(kind=kind,request=request,rejection=rejection,invalid_commits=0,invalid_executions=0,source_anchor_id=content_id(anchor),old_plan_id=old['id'],old_pending_reset=pending,first_actual_input=records[0]['actual']['executed_input'],old_first=list(map(str,A.window(old)[0])),actual_steps=len(records),actual_old_K_phase=pending['phase'],final_history=staged['history'],budget_before=old['budget'],budget_after=saved['budget'],log_file=path.name,passed=True))
    summary=dict(passed=True,cases=cases,scope='copied controlled forks at actual second-round state; excluded from main performance and solver counts')
    write(output/'summary.json',summary);return summary
