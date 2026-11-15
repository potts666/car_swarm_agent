"""Re-certify the common contract-based receiver at unchanged physical proposals."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from common import *
from protocol import reference_state
from generation import generate
from contracts import build
from execution import advance
r=read(HERE/'pre_registration.json');write(HERE/'results/pre_registration.receiver_v1.json',r);actual=restore(m,r['actual_source']);ref=reference_state(actual,r['new_contract']);checks=[]
for c in CARS:
 g,gid=generate(ref,c);assert g['passed'],g;checks.append(dict(actor=c,generation_id=gid,passed=True,failure=None))
s=copy.deepcopy(actual);advance(s,r['candidate_witness']);s['contract_id']=None;cid=build(s,r['obligations'],base=r['new_contract']);ref2=reference_state(s,cid);second=[]
for c in r['active']:
 g,gid=generate(ref2,c);assert g['passed'],g;second.append(dict(actor=c,generation_id=gid,passed=True,failure=None))
r.update(zero_checks=checks,second_baseline_checks=second,second_baseline_source=state_record(ref2),receiver_rule='contract_private_reference_v2');write(HERE/'pre_registration.json',r);print(dict(passed=True,first=len(checks),second=len(second),new_calls=0),flush=True)
