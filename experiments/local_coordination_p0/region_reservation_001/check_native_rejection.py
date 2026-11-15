"""A damaged new K must leave the overlapping old joint tail executable."""
from common import *
from protocol import prepare,commit
from run_case import advance
from accept_case import Audit


def main():
    cases=[]
    for case in ('opposite','same_direction'):
        run=HERE/'results'/case;event=next(e for e in read(run/'events.json') if e['kind']=='local_socp');raw=get(HERE/'objects',event['raw_result_id']);anchor=get(HERE/'objects',event['anchor_id']);state=restore(m,anchor['state']);old=state['witness'];new=copy.deepcopy(get(HERE/'objects',event['accepted_witness']));actor=event['actor'];k=state['cycle'];new['controls'][actor][k+32][0]=str(F(new['controls'][actor][k+32][0])+F(1,100));bad=put(HERE/'objects',new);dest=run/'native_K_rejection';assert not dest.exists();dest.mkdir()
        try:request=prepare(state,bad,content_id(dict(witness=old,cycle=k,history=state['history'],remaining=2802-k)));commit(state,request)
        except (AssertionError,ValueError) as error:reason=repr(error)
        else:raise AssertionError('damaged K committed')
        assert normal(state_record(state))==anchor['state'];audit=Audit(old);audit.state=copy.deepcopy(state);audit.p8=copy.deepcopy(state['poses'])
        vv=state['v'][actor]+H*sum(F(u[0]) for u in new['controls'][actor][k:k+32]);dd=state['delta'][actor]+H*sum(F(u[1]) for u in new['controls'][actor][k:k+32]);assert tuple(map(F,new['controls'][actor][k+32]))!=((F(1,2)-vv)/H,-dd/H)
        with gzip.open(dest/'execution.jsonl.gz','wt') as stream:
            for j in range(33):
                row=advance(state,old);audit.frame(row);stream.write(json.dumps(row,default=str,separators=(',',':'))+'\n')
        assert audit.state['history']==state['history'] and state['v'][actor]==F(1,2) and state['delta'][actor]==0
        result=dict(passed=True,case=case,source_anchor_id=event['anchor_id'],bad_witness=bad,old_witness=old,rejection=reason,invalid_commits=0,actual_steps=33,old_K_phase=k+32,old_K_exact_reset=True,final=state_record(state),independent_eight_segment_replay=True,scope='auxiliary copied source at phase787, excluded from main timing/solver counts');write(dest/'summary.json',result);cases.append(result)
    write(HERE/'native_rejection_checks.json',dict(passed=True,cases=cases));print('Both damaged-new-K proposals rejected; actual 33-step old tails independently accepted')
if __name__=='__main__':main()
