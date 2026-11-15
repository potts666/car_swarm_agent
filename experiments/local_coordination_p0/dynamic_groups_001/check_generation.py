"""Historical signed and shifted states use exactly the same contract/generator."""
from common import *
from contracts import build,bind
from generation import generate
from physics import step
from receivers import feedback,check_generated

def main():
    assert read(HERE/'solver_calls.json')==[];rows=[]
    for entry in read(HERE/'regression_sources.json'):
        s=restore(m,entry['source']);s['contract_id']=None;cid=build(s);bind(s,cid);g,gid=generate(s,entry['actor']);result=dict(kind=entry['kind'],case=entry['case'],actor=entry['actor'],source_id=entry['source_id'],contract_id=cid,generation_id=gid,passed=g['passed'],failure=g.get('failure'));rows.append(result);write(HERE/'results/generation_regression.progress.json',rows)
        assert g['passed'],result
        anchor=get(HERE/'objects',g['anchor_id']);trial=copy.deepcopy(s);w=get(HERE/'objects',s['witness']);k=s['cycle'];c=entry['actor'];cert=get(HERE/'objects',g['receiver']['certificate_id'])
        for t in range(k,k+33):
            if t==k+32:check_generated(trial,c,g['receiver']['domain']);assert feedback(trial,c,cert)==tuple(map(F,w['controls'][c][t]))
            step(trial,{a:tuple(map(F,w['controls'][a][t])) for a in CARS},w['owners'][t],4)
        result['zero_prefix_and_K_replayed']=True
    write(HERE/'results/generation_regression.json',dict(passed=True,new_calls=0,cases=rows));print(dict(passed=True,cases=len(rows),calls=0),flush=True)
if __name__=='__main__':main()
