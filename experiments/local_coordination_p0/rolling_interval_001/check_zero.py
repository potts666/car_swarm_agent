"""Zero native calls: Clear negatives, full analytic audits and two window prechecks."""
import copy,gzip,traceback
from common import *
from plan_io import load,save,restore,record
from curve_model import generate,evaluate
from contracts import binding,continuation
from local_mpc import rollout,check_receiver
from physics import step
from release import clear,verify_clear
from schedules import propose,deadlines,validate_prefix,intervals_valid
from transport import exchange
from audit import audit

def source_models(plan,state,actors,stem,transport=True):
 from itertools import combinations
 k=state['cycle'];nextstate=copy.deepcopy(state);rows=[]
 for phase in range(2):
  if phase:step(nextstate,{c:plan['controls'][c][k] for c in CARS},plan['owners'][k],8)
  for c in actors:
   assert nextstate['v'][c]!=0;job,cert=generate(plan,nextstate,c,'O-E');binding(job,plan,nextstate);assert evaluate(job,[F(0)]*64)['passed'];ref=list(map(F,job['reference']));scores=[sum(F(a)*b for a,b in zip(col,ref)) for col in job['lattice']['T']];i=max(range(len(scores)),key=lambda j:abs(scores[j]));assert scores[i]!=0,'no energy descent in exact lattice';a=F(-1 if scores[i]>0 else 1,10**7);delta=[a*F(x) for x in job['lattice']['T'][i]];assert evaluate(job,delta)['passed'],'grid descent fails original rows';cmd=[[ref[2*j]+delta[2*j],ref[2*j+1]+delta[2*j+1]] for j in range(32)];r=rollout(job,nextstate,cmd);check_receiver(job,r);base=rollout(job,nextstate,[[ref[2*j],ref[2*j+1]] for j in range(32)]);assert r['energy']<base['energy']
   row={'actor':c,'cycle':nextstate['cycle'],'model_id':job['model_id'],'analytic_delta':delta,'strict_energy_improvement':base['energy']-r['energy'],'receiver':job['receiver']}
   if transport:
    path=f'results/preflight/{stem}_p{phase}_{c}';save(path+'_job.json.gz',job);save(path+'_derivatives.json.gz',cert);raw,meta=exchange(job,'assemble',HERE/path);row.update(assembly=raw['assembly'],transport=meta)
   rows.append(row)
 return rows

def negatives():
 results=[];e=read(HERE/'results/E-anchor_events.json')[0];p=load(e['old_plan']);s=restore(load(e['checkpoint']));c=e['meta']['outgoing']
 def expect(name,fn):
  try:fn()
  except (AssertionError,ValueError,KeyError) as err:results.append({'fixture':name,'rejected':True,'reason':repr(err),'commits':0,'executions':0})
  else:raise AssertionError(('fixture not rejected',name))
 bad=copy.deepcopy(s);bad['poses'][c]=(m.I(-F(TASKS[c]['goal'][0])),s['poses'][c][1],s['poses'][c][2]);expect('wrong_geometric_release_side',lambda:assert_clear(p,bad,c))
 # Explicit physically possible reverse suffix into the core, despite clear now.
 badp=copy.deepcopy(p);k=s['cycle'];newgoal=k+160;badp['per_car_policies'][c]['goal']=newgoal;badp['controls'][c][k:newgoal]=[['-1','0']]*5+[['0','0']]*155;expect('future_reentry',lambda:assert_clear(badp,s,c));future=clear(badp,s,c);assert not future['future_clear'];results[-1]['actual_future_certificate']=future['actual_future']
 bad=copy.deepcopy(e['clear']);bad['source']['K_id']='missing';expect('missing_receiver_K',lambda:verify_clear(p,s,bad))
 new=load(e['candidate']);bad=copy.deepcopy(new);bad['per_car_policies'][c]['goal']=p['per_car_policies'][c]['goal']+1;expect('deadline_increase',lambda:deadlines(p,bad,k))
 bad=copy.deepcopy(new);bad['controls'][c][0]=['0','0'];expect('executed_prefix_modified',lambda:validate_prefix(p,bad,k))
 bad=copy.deepcopy(new);bad['release_cuts'][c]=bad['per_car_policies'][e['meta']['incoming']]['service_start']+1;expect('half_open_owner_endpoint_overlap',lambda:intervals_valid(bad))
 bad=copy.deepcopy(p);incoming=e['meta']['incoming'];bad['controls'][incoming][k]=['0','1/10'];expect('illegal_delete_stopped_steering_or_K',lambda:propose(bad,s,c,k))
 after=copy.deepcopy(s);step(after,{d:p['controls'][d][k] for d in CARS},p['owners'][k],8);assert clear(p,after,c)['full_Clear'],'inherited parking flow must allow next-boundary retry';bad=clear(p,s,c);expect('stale_actual_history',lambda:verify_clear(p,after,bad))
 # Closed occupancy temporal boundary remains a collision candidate at equality.
 from discovery import Directory
 directory=Directory();directory.register('A1',[F(0),F(1),F(0),F(1)],0,1,'intent');directory.register('B1',[F(0),F(1),F(0),F(1)],1,2,'permanent_goal');q=directory.query('A1',[F(0),F(1),F(0),F(1)],0,1);assert 'B1' in q['owners'];results.append({'fixture':'closed_occupancy_boundary','passed':True,'discovered_at_equal_endpoint':True})
 from frames import validate_publication,validate_discovery,discover
 frame=read(HERE/p['frame_file']);damaged=copy.deepcopy(frame);damaged['permanent_goals'].remove(incoming);expect('omitted_permanent_goal',lambda:validate_publication(damaged))
 damaged=copy.deepcopy(frame);damaged['outer_slots'][c]='stale';expect('mixed_outer_inner_identity',lambda:validate_publication(damaged))
 event2=read(HERE/'results/E-anchor_events.json')[1];old2,new2=load(event2['old_plan']),load(event2['candidate']);expected=discover(old2,new2,read(HERE/old2['frame_file']),read(HERE/new2['frame_file']),event2['cycle']);damaged=copy.deepcopy(expected);damaged['participants'].pop();expect('omitted_closure_member',lambda:validate_discovery(damaged,expected))
 write(HERE/'results/negative_fixtures.json',{'passed':True,'fixtures':results,'next_boundary_retry_with_nonzero_delta':str(after['delta'][c]),'new_optimizer_calls':0});return results

def assert_clear(plan,state,c):assert clear(plan,state,c)['full_Clear'],'Clear(actual future/Q/geometry) failed'
def run():
 assert not read(HERE/'solver_calls.json');negatives();out=[]
 for name in ('R-fixed','E-anchor'):
  p=load(f'results/{name}_final_plan.json.gz');result=audit(p,trace=str(HERE/f'results/{name}_actual.jsonl.gz'),output=HERE/f'results/{name}_independent.json');assert result['passed']
 summary=read(HERE/'results/E-anchor_summary.json');events=[e for e in summary['events'] if e['transaction']['accepted']];assert len(events)>=2;rows={}
 with gzip.open(HERE/'results/E-anchor_actual.jsonl.gz','rt') as f:
  for text in f:r=json.loads(text);rows[r['cycle']]=r
 previous_end=-1
 for eventindex,event in enumerate(events[:2],1):
  p=load(event['candidate']);chosen=None
  for k in range(event['cycle'],min(e['cycle'] for e in events if e['cycle']>event['cycle']) if any(e['cycle']>event['cycle'] for e in events) else p['B0']):
   if k<=previous_end:continue
   s=restore(rows[k]);moving=[c for c in CARS if s['v'][c]!=0 and abs(H*(2*s['v'][c]+H*F(p['controls'][c][k][0]))/2)>=F(1,50)]
   for actors in __import__('itertools').combinations(moving,2):
    try:models=source_models(p,s,actors,f'e{eventindex}_k{k}')
    except (AssertionError,ValueError) as err:out.append({'event':eventindex,'cycle':k,'actors':actors,'skipped':repr(err)});continue
    chosen={'event':eventindex,'cycle':k,'actors':list(actors),'models':models,'source':record(s),'forecast_only':eventindex==2,'plan':event['candidate']};break
   if chosen:break
  assert chosen,('no two-round window',eventindex);out.append(chosen);previous_end=chosen['cycle']+1;print('PASS window',eventindex,chosen['cycle'],chosen['actors'],flush=True)
 write(HERE/'results/preflight_certificate.json',{'passed':True,'windows':out,'new_optimizer_calls':0,'source_rule':'earliest actual active pair with two-phase nonzero exact-grid energy descent and receiver; actual successor regenerated before every native'})
if __name__=='__main__':
 try:run()
 except Exception as e:write(HERE/'results/preflight_failure.json',{'layer':'zero_preflight','error':repr(e),'traceback':traceback.format_exc(),'new_optimizer_calls':0});raise
