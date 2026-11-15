"""Zero-call encounter damage, closed occupancy, release and receiving checks."""
import copy
from common import *
from plan_io import load,save,source,record
from encounter import deadline_check,approve_joint_requests,active_pairs
from discovery import certify
from transaction import check_binding,select,closure_required
from local_mpc import prepare,admit
from physics import step
from execution import gate

def rejects(fn):
 try:fn()
 except (AssertionError,ValueError):return True
 raise AssertionError('damaged proposal accepted')

def run():
 event=read(HERE/'event_registration.json');k=event['source_cycle'];old=load('results/R_old.json.gz');new=load('results/E_anchor.json.gz');state=source(old,k);checks=[]
 assert content_id(record(state))==event['source_id']
 a,b=event['actors'];intervals=[list(map(F,i['resource_interval'])) for i in event['event']['intents']];assert max(x[0] for x in intervals)<=min(x[1] for x in intervals)
 # Individually valid requests overlap the exclusive key; combining approvals
 # without a common owner cannot be admitted.
 rejects(lambda:approve_joint_requests(event['event']['intents']))
 discovered,_=active_pairs(old,state);assert content_id(discovered[0])==content_id(event['event'])
 checks.append({'name':'two independent approvals conflict','overlap':event['event']['overlap'],'joint_admitted':False})
 rejects(lambda:select(old,new,[a,b],[a,b],{}));checks.append({'name':'resource endpoints omitted','required':sorted(closure_required(old,new))})
 oldjob=prepare(old,state,b);rejects(lambda:check_binding(oldjob,new,state,b));checks.append({'name':'old physical calendar / new anchor mixed rejected'})
 for c in CARS:
  j=prepare(new,state,c);check_binding(j,new,state,c,j['receiving_certificate'])
  for field in ('parent','phase','K_id','deadline'):
   bad=copy.deepcopy(j['receiving_certificate']);bad[field]='damaged';rejects(lambda:check_binding(j,new,state,c,bad))
 checks.append({'name':'receiving/K and exact equalities retained'})
 # Missing brake/recovery changes actual endpoint velocity or target identity,
 # not just a label. Reject broken copies through physical execution checks.
 broken=copy.deepcopy(new);end=broken['per_car_policies'][b]['approach_end'];broken['controls'][b][end-1]=['0','0']
 def check_stop():
  s=source(broken,end);assert s['v'][b]==s['delta'][b]==0,'waiting domain not reached'
 rejects(check_stop)
 missing=copy.deepcopy(new);start=missing['per_car_policies'][b]['service_start'];finish=missing['per_car_policies'][b]['goal'];missing['controls'][b][start:finish]=[['0','0']]*(finish-start)
 def check_goal():
  s=source(missing,finish);assert sum((s['poses'][b][j]-F(TASKS[b]['goal'][j])).abs_upper() for j in (0,1))<F(2,25),'resume connection missing'
 rejects(check_goal);checks.append({'name':'brake endpoint / recovery missing rejected','stop_phase':end,'service_start':start})
 reverse=load(f'results/proposals/{b}_passes_{a}_existing_stop.json.gz');limits=deadline_check(old,reverse,k);assert limits['block_passed'] and not limits['per_car_passed'];rejects(lambda:select(old,reverse,CARS,CARS,{}));checks.append({'name':'individual deadline violation rejected','limits':limits})
 assert select(old,new,CARS,[c for c in CARS if c!=b],{})[0] is old;checks.append({'name':'missing ACK returns complete old strategy'})
 idx=certify(old,new,participants=CARS,cycle=k);save('results/event_occupancy.json.gz',idx)
 # Actual physical outside-core gate, not an unconditional cycle flag.
 out=new['encounter']['first_out'];at=source(new,out);assert gate(new,at)['body_resource_clear'];bad=copy.deepcopy(at);bad['poses'][a]=(m.I(0),m.I(0),m.I(0));rejects(lambda:gate(new,bad));checks.append({'name':'actual exit-domain release required','release_phase':out})
 # The two other cars retain their complete compatible early movement.
 others=[c for c in CARS if c not in event['actors']]
 assert all(new['controls'][c][:new['per_car_policies'][c]['approach_end']]==old['controls'][c][:old['per_car_policies'][c]['approach_end']] for c in others)
 checks.append({'name':'other cars compatible movement preserved','actors':others})
 write(HERE/'results/protocol_precheck.json',{'passed':True,'checks':checks,'native_calls':0,'initial_block':list(CARS),'discovered_encounter_group':event['actors'],'edit_closure':sorted(closure_required(old,new)),'natural_commit_size_changed':False})
 print(json.dumps({'passed':True,'checks':len(checks),'native_calls':0}))
if __name__=='__main__':run()
