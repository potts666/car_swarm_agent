"""Same generator rejects insufficient old launch and preserves old-source feasibility."""
from common import *
from proposals import candidate
from contracts import build,bind
from generation import generate
from protocol import reference_state

def main():
    assert read(HERE/'solver_calls.json')==[];r=read(HERE/'pre_registration.json');actual=restore(m,r['actual_source']);old=get(HERE/'objects',actual['witness']);wrong=candidate(old,actual,r['shifts'],repair=False);s=copy.deepcopy(actual);s['witness']=put(HERE/'objects',wrong);cid=build(s,r['obligations'],base=r['new_contract']);bind(s,cid);g,gid=generate(s,r['added'][0]);assert not g['passed'] and 'progress' in g['failure'],'uncorrected launch unexpectedly met eta';rows=[dict(case='unrepaired_launch_deficit',expected='zero_anchor progress failure',generation_id=gid,failure=g['failure'],passed=True)]
    source=reference_state(actual,r['old_contract'])
    for c in (r['added'][0],next(x for x in r['active'] if x not in r['added'])):
        g,gid=generate(source,c);assert g['passed'],g;rows.append(dict(case='old_contract_source',actor=c,generation_id=gid,passed=True))
    write(HERE/'results/generation_regression.json',dict(passed=True,new_calls=0,cases=rows));print(dict(passed=True,cases=len(rows),new_calls=0),flush=True)
if __name__=='__main__':main()
