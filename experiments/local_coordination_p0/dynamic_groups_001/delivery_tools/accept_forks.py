"""Independent replay of two recorded rejection suffixes from certified actual source."""
import sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent.parent;sys.path.insert(0,str(ROOT))
from common import *
from audit_base import Audit

def main():
    result=[]
    for name in ('anchor_execution','layer2_execution'):
        path=ROOT/'results/rejection'/(name+'.jsonl.gz');began=time.perf_counter();audit=None;n=0
        with gzip.open(path,'rt') as stream:
            for row in map(json.loads,stream):
                if audit is None:
                    source=row['source'];audit=Audit(source['witness']);audit.state=restore(m,source);audit.p8=copy.deepcopy(audit.state['poses'])
                s=audit.state;k=s['cycle'];assert row['cycle']==k;assert row['source']==normal(state_record(s));w=get(ROOT/'objects',row['witness_id']);owner=w['owners'][k]
                if owner!=s['owner']:
                    if s['owner']:audit.domain(s['owner'],'G_out')
                    if owner:audit.domain(owner,'W_in');s['served'].append(owner)
                    s['permission_version']+=1
                s.update(witness=row['witness_id'],owner=owner)
                for c,kind in w['events'].get(str(k),[]):
                    if kind.startswith('generated:'):
                        audit.domain(c,kind);from receivers import feedback
                        assert tuple(map(F,w['controls'][c][k]))==feedback(s,c,get(ROOT/'objects',kind.split(':')[1]))
                audit.step({c:tuple(map(F,w['controls'][c][k])) for c in CARS},owner,row['actual'])
                for c,kind in w['events'].get(str(k+1),[]):audit.domain(c,kind)
                assert row['B_after']==max(2802-s['cycle'],0);n+=1
        for c in CARS:audit.domain(c,'target')
        result.append(dict(branch=name,passed=True,steps=n,minimum_road_m=float(audit.road),minimum_pair_m=float(audit.pair),final_history=audit.state['history'],wall_s=time.perf_counter()-began))
    write(ROOT/'results/rejection/independent_acceptance.json',dict(passed=True,new_calls=0,branches=result));print(json.dumps(result,indent=2))
if __name__=='__main__':main()
