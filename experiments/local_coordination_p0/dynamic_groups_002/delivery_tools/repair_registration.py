"""New preregistered construction version after retained C-v1 diagnostic failures."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from common import *
from contracts import build,physical_diff
from protocol import reference_state
from generation import generate
from execution import advance
oldreg=read(HERE/'results/pre_registration.C_v1.json');assert len(read(HERE/'solver_calls.json'))==6
actual=restore(m,oldreg['actual_source']);oldid=build(actual);reference=copy.deepcopy(actual);reference['witness']=oldreg['candidate_witness'];cid=build(reference,oldreg['obligations']);c=copy.deepcopy(get(HERE/'objects',cid));diff=physical_diff(oldid,cid);c['transition']=dict(actual_source=state_record(actual),old_contract=oldid,closure=diff['closure'],old_faces_id=get(HERE/'objects',oldid)['faces_id'],old_calendar_id=get(HERE/'objects',oldid)['owner_calendar_id']);cid=put(HERE/'objects',c);reference=reference_state(actual,cid);checks=[]
for car in CARS:
 g,gid=generate(reference,car);assert g['passed'],g;checks.append(dict(actor=car,generation_id=gid,passed=True,failure=None))
s=copy.deepcopy(actual);advance(s,oldreg['candidate_witness']);s['contract_id']=None;cid2=build(s,oldreg['obligations'],base=cid);ref2=reference_state(s,cid2);second=[]
for car in oldreg['active']:
 g,gid=generate(ref2,car);assert g['passed'],g;second.append(dict(actor=car,generation_id=gid,passed=True,failure=None))
r=copy.deepcopy(oldreg);r.update(version='construction_v3_after_C_diagnostic',old_contract=oldid,new_contract=cid,anchor_source=state_record(reference),zero_checks=checks,second_baseline_checks=second,second_baseline_source=state_record(ref2),calls_at_freeze=6,C_v1_registration='results/pre_registration.C_v1.json',fixes=['maximize available certified separation axes instead of accepting a barely feasible axis','compile zero-radius receiver endpoint equalities into common quantization blocks'],comparability_to_C_v1='different physical faces and redundant equality encoding; controlled numerical comparison pending')
write(HERE/'pre_registration.json',r);write(HERE/'physical_contract_diff.json',physical_diff(oldid,cid));print(dict(passed=True,unchanged_source_and_physical_anchor=True,old_cap=get(HERE/'objects',oldreg['new_contract'])['cap'],new_cap=get(HERE/'objects',cid)['cap'],native_calls_used=6),flush=True)
