"""Prove the advanced reference actually violates a physical old separation face."""
import sys,re
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from common import *
from contracts import build,PAIRS,support
from physics import step,predicted_nodes,body
from protocol import reference_state
from rules import contracts_for,assert_step
r=read(HERE/'pre_registration.json');actual=restore(m,r['actual_source']);s=copy.deepcopy(actual);s['witness']=r['candidate_witness']
try:build(s,{},base=r['old_contract'])
except AssertionError as error:
 reason=str(error);match=re.search(r"'physical separation face', (\d+), (\d+), '([^']+)', '([^']+)', '([^']+)'",reason);assert match,reason;t,j,a,b,slack=match.groups();t,j=int(t),int(j)
else:raise AssertionError('new anchor also fits every old separation face; do not claim geometric release')
w=get(HERE/'objects',s['witness'])
while s['cycle']<t:step(s,{c:tuple(map(F,w['controls'][c][s['cycle']])) for c in CARS},w['owners'][s['cycle']],4)
nodes=predicted_nodes(s,{c:tuple(map(F,w['controls'][c][t])) for c in CARS},4);pairno=PAIRS.index((a,b));old=get(HERE/'objects',r['old_contract']);new=get(HERE/'objects',r['new_contract']);oldrow=get(HERE/'objects',old['faces_id'])[str(t)][j*6+pairno];newrow=get(HERE/'objects',new['faces_id'])[str(t)][j*6+pairno];n=tuple(map(F,oldrow[2:4]));beta=F(oldrow[4]);pad=TUBE+F(15)*(H/4)**2/8;aa=body(nodes[a][j],a)+body(nodes[a][j+1],a);bb=body(nodes[b][j],b)+body(nodes[b][j+1],b);al,ah=support(aa,n);bl,bh=support(bb,n);sides=dict(upper_slack=beta-SAFE/2-ah-pad,lower_slack=bl-pad-beta-SAFE/2);assert min(sides.values())<0
oldstate=copy.deepcopy(actual);oldstate['contract_id']=r['old_contract'];newstate=reference_state(actual,r['new_contract']);actor=r['added'][0];oldbox=contracts_for(oldstate,actor)[0];newbox=contracts_for(newstate,actor)[0];cmd=tuple(map(F,get(HERE/'objects',r['candidate_witness'])['controls'][actor][actual['cycle']]))
try:assert_step(actual['v'][actor],actual['delta'][actor],cmd,oldbox)
except ValueError as error:phase_reason=str(error)
else:raise AssertionError('old signed permission does not exclude the new first command')
write(HERE/'old_contract_exclusion.json',normal(dict(passed=True,new_calls=0,actor=actor,old_contract=r['old_contract'],new_contract=r['new_contract'],first_old_face_violation=dict(cycle=t,sub=j,pair=[a,b],old_plane=oldrow,new_plane=newrow,old_side_slacks=sides,old_projected_upper=ah,old_projected_lower=bl,required_SAFE=SAFE,tube_and_chord=pad,source_at_violation=state_record(s)),first_command_permission=dict(command=cmd,old_box=oldbox,new_box=newbox,old_rejection=phase_reason),new_reference_has_fresh_full_contract=True)));print(dict(passed=True,first_old_face_violation=t,pair=[a,b],slack=slack,new_calls=0),flush=True)
