"""Content-checked defaults plus new independent key-prefix checks; no solver calls."""
import argparse,gzip,hashlib,json,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT))
from common import *
from audit_base import Audit
from rules import quantize
from socp_rows import evaluate

def verify(full=False):
    began=time.perf_counter();checked=0
    for line in (ROOT/'SHA256SUMS').read_text().splitlines():
        digest,relative=line.split('  ',1);assert sha(ROOT/relative)==digest,relative;checked+=1
    for p in (ROOT/'objects').glob('*.json.gz'):
        value=json.loads(gzip.decompress(p.read_bytes()));assert content_id(value)==p.name.split('.')[0],p.name
    calls=read(ROOT/'solver_calls.json');assert len(calls)==12 and len({x['event'] for x in calls})==12 and all(x['completed'] for x in calls)
    gate=read(ROOT/'GATE.json');prefixes=[]
    for arm in ('F-S','F-P','D-P'):
        out=ROOT/'results'/arm;summary=read(out/'summary.json');acceptance=read(out/'independent_acceptance.json');assert summary['passed'] and acceptance['passed']
        for event in summary['events']:
            s=restore(m,event['source']);new=get(ROOT/'objects',event['selected_witness']);k=s['cycle'];audit=Audit(summary['events'][0]['source']['witness']);audit.state=copy.deepcopy(s);audit.p8=copy.deepcopy(s['poses']);start_history=s['history']
            for admission in event['admissions']:
                raw=get(ROOT/'objects',admission['raw_result_id']);job=admission['job'];anchor=get(ROOT/'objects',job['anchor_id']);g=get(ROOT/'objects',job['generation_id']);assert anchor['state']==event['source'] and raw['source']==event['source'];rows=get(ROOT/'objects',g['physical_rows_id']);delta=[F(x)-F(y) for u,v in zip(admission['candidate']['controls'][:32],get(ROOT/'objects',s['witness'])['controls'][job['actor']][k:k+32]) for x,y in zip(u,v)]
                assert evaluate(rows,delta)['passed'] and any((lambda z:z[1]['passed'] and z[0]==delta)(quantize(raw['x'],a,anchor['contracts'])) for a in (F(1),F(1,2),F(1,4)))
            for t in range(k,k+33):
                for c,kind in new['events'].get(str(t),[]):
                    if kind.startswith('generated:'):
                        audit.domain(c,kind);from receivers import feedback
                        assert tuple(map(F,new['controls'][c][t]))==feedback(audit.state,c,get(ROOT/'objects',kind.split(':')[1]))
                audit.step({c:tuple(map(F,new['controls'][c][t])) for c in CARS},new['owners'][t])
                for c,kind in new['events'].get(str(t+1),[]):audit.domain(c,kind)
            prefixes.append(dict(arm=arm,round=event['round'],source_history=start_history,steps=33,passed=True))
        if full:
            import accept_case as module
            # Fresh conclusions remain in memory; original acceptance evidence stays immutable.
            original_write=module.write
            try:
                module.write=lambda path,value: None
                result=module.accept(arm)
            finally:module.write=original_write
            assert result['passed']
    assert gate['passed'];return dict(passed=True,manifest_files=checked,independent_prefixes=prefixes,full_replay=full,new_calls=0,wall_s=time.perf_counter()-began)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--full-replay',action='store_true');p.add_argument('--fresh-run',action='store_true');a=p.parse_args()
    if a.fresh_run:
        import shutil,tempfile
        target=Path(tempfile.mkdtemp(prefix='dynamic_groups_001_fresh_'))
        for name in ('inputs','sources','objects','theory'):
            shutil.copytree(ROOT/name,target/name)
        for name in ('common.py','physics.py','rules.py','derivatives.py','socp_rows.py','receivers.py','generation.py','contracts.py','conic.py','certify_domains.py','witness.py','domain_checks.json','rational_inputs.json','frozen_config.json','method_spec.md','execution_plan.md'):
            shutil.copy2(ROOT/name,target/name)
        (target/'solver_calls.json').write_text('[]\n')
        (target/'NEW_BUDGET_REQUIRED.json').write_text(json.dumps(dict(allowed_new_calls=0,reason='original version exhausted its 12-call budget; allocate a new version and budget before numerical execution'),indent=2)+'\n')
        print(json.dumps(dict(created=str(target),empty_solver_ledger=True,new_calls=0,status='prepared; separate numerical budget required'),indent=2));raise SystemExit(0)
    print(json.dumps(verify(a.full_replay),ensure_ascii=False,indent=2))
