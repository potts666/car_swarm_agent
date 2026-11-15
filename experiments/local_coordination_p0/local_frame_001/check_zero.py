"""Zero-solve actual worker assembly, source freedom, own-ID and fallback fixtures."""
import copy,traceback
from common import *
from plan_io import load,save,source,record
from curve_model import generate,evaluate
from contract import validate_binding,patch,replay_suffix
from local_mpc import rollout,check_receiver
from frame import validate_frame_plan,source_valid,node_inclusion
from protocol import Wire,choose,check_dependency
from transport import exchange

def expect(name,fn,fixtures):
 try:fn()
 except (AssertionError,KeyError,ValueError) as e:fixtures.append({'fixture':name,'rejected':True,'reason':repr(e),'commits':0,'executions':0,'optimizer_calls':0})
 else:raise AssertionError(('fixture not rejected',name))
def run():
 assert not read(HERE/'solver_calls.json');frame=read(HERE/'outer_frame.json');sel=read(HERE/'results/source_selection.json');old=load('results/old.json.gz');new=load('results/new.json.gz');k=sel['cycle'];D=sel['D'];X=sel['X'];actors=D+[X];models=[]
 wire=Wire('initialization');acks=[]
 for c in CARS:
  msg=wire.send('frame_directory',c,'establish_frame',{'outer_frame_id':frame['outer_frame_id'],'certificate':content_id(read(HERE/'results/outer_frame_certificate.json')),'own_occupancy':'old/new branch union + allowed .025 corridor + receiver/K/permanent goals','lease':next(q for q in frame['leases'].values() if c in q['members']),'deadline':frame['deadlines'][c]},'initialization');acks.append(wire.send(c,'frame_directory','ACK_frame',msg,'initialization'))
 assert len(acks)==4;write(HERE/'results/initialization_ACK.json',acks)
 for phase in (k,k+1):
  state=source(old,phase)
  for c in actors:
   job,cert=generate(new,state,c,'O-E');validate_binding(job,new,state);label=f'p{phase}_{c}';save(f'results/preflight/{label}_job.json.gz',job);save(f'results/preflight/{label}_derivatives.json.gz',cert);raw,meta=exchange(job,'assemble',HERE/f'results/preflight/{label}');r=rollout(job,state,[[F(x) for x in row] for row in new['controls'][c][phase:phase+32]]);check_receiver(job,r);assert job['lattice']['rank_free_dimension']>0
   # Analytic energy descent: choose an exact E/T acceleration column direction.
   ref=list(map(F,job['reference']));scores=[sum(ref[t]*F(col[t]) for t in range(64)) for col in job['lattice']['T']];idx=max(range(len(scores)),key=lambda t:abs(scores[t]));assert scores[idx]!=0;col=list(map(F,job['lattice']['T'][idx]));step=F(-1 if scores[idx]>0 else 1,10**7);delta=[step*x for x in col];assert evaluate(job,delta)['passed'];commands=[[ref[2*j]+delta[2*j],ref[2*j+1]+delta[2*j+1]] for j in range(32)];actual=rollout(job,state,commands);check_receiver(job,actual);assert actual['energy']<r['energy'];models.append({'phase':phase,'actor':c,'model_id':job['model_id'],'assembly':raw['assembly'],'transport':meta,'free_dimension':job['lattice']['rank_free_dimension'],'analytic_descent':r['energy']-actual['energy'],'receiver':job['receiver']});print('PASS zero model',phase,c,flush=True)
 # X model and ACK depend on fixed outer frame and its own inputs, not D hash.
 s=source(old,k);a,_=generate(old,s,X,'O-E');b,_=generate(new,s,X,'O-E');assert a==b,'external model invalidated by D private change';fixtures=[]
 bad=copy.deepcopy(new);bad['outer_frame_id']='old';expect('stale_outer_frame_id',lambda:validate_frame_plan(bad,frame),fixtures)
 expect('expired_frame',lambda:source_valid(frame,{'cycle':frame['valid_through']+1}),fixtures)
 damaged=copy.deepcopy(frame);damaged['permanent_goals'].remove(X);expect('omitted_permanent_goal',lambda:require_goals(damaged),fixtures)
 outside=copy.deepcopy(s);outside['poses'][X]=(outside['poses'][X][0]+1,*outside['poses'][X][1:]);expect('outside_occupancy_envelope',lambda:node_inclusion(new,outside),fixtures)
 damaged=copy.deepcopy(new);damaged['per_car_policies'][X]['goal']+=1;expect('deadline_increase',lambda:validate_frame_plan(damaged,frame),fixtures)
 expect('owner_endpoint_overlap',lambda:intervals([(26,475),(406,855)]),fixtures)
 expect('mixed_internal_commit_id',lambda:one_bundle(['version1','version2']),fixtures)
 from contract import guard
 state=source(new,new['encounter']['first_out']);state['poses']['A1']=(state['poses']['A1'][0]-1,*state['poses']['A1'][1:]);expect('wrong_release_guard',lambda:guard(new,state),fixtures)
 # Legacy global-parent binding would invalidate X even though own model is equal.
 write(HERE/'results/old_identity_failure.json',{'old_global_parent_changes':content_id(old)!=content_id(new),'own_X_model_equal':a==b,'old_D_calendar':[26,475,924],'new_D_calendar':[26,406,855,924],'repair':'own-plan worker parent; outer-only external dependency; D atomic inner version','zero_calls':True})
 chosen,event=choose(old,new,s,frame,D,X,'L',Wire('preflight_missing_ack'),acks=[D[0],X],phase='fixture');assert not event['D_committed'] and event['X_committed'];assert chosen['controls'][X]==new['controls'][X] and chosen['owners']==old['owners']
 write(HERE/'results/preflight_certificate.json',{'passed':True,'models':models,'forecast_second_only':True,'actual_second_regenerated_before_native':True,'D':D,'X':X,'outer_frame_id':frame['outer_frame_id'],'external_model_equal_across_D_private_change':True,'fixtures':fixtures,'solver_calls':0});print('PASS preflight all; releases0',flush=True)
def require_goals(frame):assert set(frame['permanent_goals'])==set(CARS),'permanent goal omitted'
def intervals(rr):assert all(a[1]<=b[0] for a,b in zip(rr,rr[1:])),'half-open lease endpoints overlap'
def one_bundle(ids):assert len(set(ids))==1,'mixed internal transaction numbers'
if __name__=='__main__':
 try:run()
 except Exception as e:write(HERE/'results/preflight_failure.json',{'layer':'zero_preflight','error':repr(e),'traceback':traceback.format_exc(),'optimizer_calls':0});raise
