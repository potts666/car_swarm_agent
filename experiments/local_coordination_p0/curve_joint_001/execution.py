"""Runtime source-domain release guard and full branch execution."""
import gzip
from common import *
from physics import step,body
from plan_io import load,record

def gate(plan,state):
 event=plan.get('encounter');k=state['cycle']
 if not event or k!=event['first_out']:return None
 c,d=event['passing'],event['yielding'];phase=event['gate_path_phase'];ref=plan['paths'][c]['poses_Q'][phase];p=state['poses'][c]
 assert sum((p[j]-m.I.raw(*ref[j])).abs_upper() for j in (0,1))<F(1,1000000)
 assert (p[2]-m.I.raw(*ref[2])).abs_upper()<F(1,100000000) and state['v'][c]==state['delta'][c]==0
 box=m.axis_box(body(p,c));clear=F(box[0],m.Q)-TUBE>16 if direction(c)>0 else F(box[1],m.Q)+TUBE<-16
 assert clear and state['v'][d]==state['delta'][d]==0
 assert plan['owners'][k-1]==c and plan['owners'][k]==d
 return {'cycle':k,'event':'actual_exit_domain_and_body_clear_release','passing':c,'yielding':d,'actual_source':record(state),'body_resource_clear':clear,'gate_phase':phase,'new_owner':d}

def run_branch(name,plan):
 state=initial_state();events=[]
 with gzip.open(HERE/f'results/{name}.jsonl.gz','wt') as f:
  for k in range(plan['B0']):
   event=gate(plan,state)
   if event:events.append(event)
   row=step(state,{c:plan['controls'][c][k] for c in CARS},plan['owners'][k],4);row['cycle']=state['cycle'];f.write(json.dumps(row,default=str)+'\n')
 write(HERE/f'results/{name}_events.json',events)
 return state
if __name__=='__main__':
 for n,p in [('R_old','results/R_old.json.gz'),('E_anchor','results/E_anchor.json.gz')]:run_branch(n,load(p))
