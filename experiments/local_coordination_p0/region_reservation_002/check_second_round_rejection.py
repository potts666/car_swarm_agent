"""Copied actual round-two sources: reject damage, execute the same round-one tail."""
from common import *
from protocol import prepare,commit
from witness import plan_id
from run_case import advance
from accept_case import Audit


def main():
    summaries=[]
    for case in ('opposite','same_direction'):
        run=HERE/'results'/case;events=[e for e in read(run/'events.json') if e['kind']=='local_socp'];assert len(events)==2 and all(e['accepted'] for e in events)
        first,second=events;a1=get(HERE/'objects',first['anchor_id']);a2=get(HERE/'objects',second['anchor_id']);old=a2['state']['witness'];source=restore(m,a2['state']);raw=get(HERE/'objects',second['raw_result_id']);valid=get(HERE/'objects',second['accepted_witness']);actor=second['actor']
        assert source['cycle']==788 and old==first['accepted_witness']
        audit0=Audit(read(run/'config.json')['initial_witness']);audit0.state=restore(m,a1['state']);audit0.p8=copy.deepcopy(audit0.state['poses']);audit0.validate_witness(audit0.state['witness']);audit0.numeric_update(first)
        with gzip.open(run/'execution.jsonl.gz','rt') as stream:firstrow=next(r for r in map(json.loads,stream) if r['cycle']==787)
        audit0.frame(firstrow);assert normal(state_record(audit0.state))==a2['state']
        folder=run/'second_round_rejections';assert not folder.exists();folder.mkdir();cases=[]
        for kind in ('stale_anchor','stale_derivative','old_phase','stale_peer_pose','damaged_K','mixed_permission_version'):
            trial=copy.deepcopy(source);bad=copy.deepcopy(valid);modified_raw=copy.deepcopy(raw);request=None
            if kind=='stale_anchor':modified_raw['anchor_id']=first['anchor_id']
            elif kind=='stale_derivative':modified_raw['derivative_id']=first['derivative_id']
            elif kind=='old_phase':modified_raw['phase']=787
            elif kind=='stale_peer_pose':
                stale=copy.deepcopy(a2);stale['state']['poses_Q']['A1']=copy.deepcopy(a1['state']['poses_Q']['A1']);modified_raw['anchor_id']=put(HERE/'objects',stale)
            elif kind=='damaged_K':bad['controls'][actor][820][0]=str(F(bad['controls'][actor][820][0])+F(1,100))
            if kind not in ('damaged_K','mixed_permission_version'):bad['optimization_origin']=put(HERE/'objects',modified_raw)
            bid=put(HERE/'objects',bad)
            try:
                request=prepare(trial,bid,plan_id(trial,old))
                if kind=='mixed_permission_version':request['permission_version']+=1
                commit(trial,request)
            except (AssertionError,ValueError) as error:reason=repr(error)
            else:raise AssertionError('damaged second-round proposal committed: '+kind)
            assert normal(state_record(trial))==a2['state'];audit=audit0.clone();count=0
            with gzip.open(folder/(kind+'.jsonl.gz'),'wt') as stream:
                for j in range(33):
                    row=advance(trial,old);audit.frame(row);stream.write(json.dumps(row,default=str,separators=(',',':'))+'\n');count+=1
                    if j==0:assert row['actual']['commands']=={c:list(map(str,get(HERE/'objects',old)['controls'][c][788])) for c in CARS}
                    if row['cycle']==819:assert trial['v'][actor]==F(1,2) and trial['delta'][actor]==0
            assert trial['history']==audit.state['history'] and trial['cycle']==821
            cases.append(dict(kind=kind,passed=True,bad_witness=bid,rejection=reason,invalid_commits=0,actual_steps=count,old_witness=old,old_K_phase=819,actual_old_K_executed=True,final=state_record(trial),independent_eight_subsegment_replay=True))
        summary=dict(passed=True,case=case,actual_round_two_source=a2['state'],first_accepted_witness=old,second_accepted_witness=second['accepted_witness'],cases=cases,scope='copied branches excluded from main time and solver counts');write(folder/'summary.json',summary);summaries.append(summary)
    write(HERE/'second_round_rejection_checks.json',dict(passed=True,cases=summaries));print('All twelve damaged round-two proposals rejected; same old tails and K actually executed')
if __name__=='__main__':main()
