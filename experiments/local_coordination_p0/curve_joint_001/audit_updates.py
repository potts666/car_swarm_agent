"""Independent post-run raw/model, actual objective, two-round and full-plan audit."""
import copy,time,traceback,hashlib
from common import *
from plan_io import load,save,source,restore,record
from audit import audit
from physics import propagate
from progression import coordinate
from curve_bounds import sqrt_upper

def own_check(job,ad,state):
 if not ad['accepted']:return {'accepted':False,'anchor_retained':True}
 delta=list(map(F,ad['delta']));T=job['lattice']['T'];ints=ad['quantization']['integer_coordinates'];rebuilt=[sum(F(col[j])*F(n,10**12) for col,n in zip(T,ints)) for j in range(64)];assert rebuilt==delta
 assert all(sum(F(e)*x for e,x in zip(row,delta))==0 for row in job['lattice']['E']);assert sum(x*x for x in delta)<=F(job['rho'])**2
 n=sqrt_upper(sum(x*x for x in delta));minslack=F(100)
 for row in job['rows']:
  val=F(row['g'])+sum(F(a)*d for a,d in zip(row['a'],delta))+F(row['epsilon'])*n+F(row['H'])*n*n/2;assert val<=0,(row['name'],val);minslack=min(minslack,-val)
 c=job['actor'];p=state['poses'][c];v=state['v'][c];d=state['delta'][c];ref=list(map(F,job['reference']));cmd=[[ref[2*j]+delta[2*j],ref[2*j+1]+delta[2*j+1]] for j in range(32)];assert cmd==[[F(x) for x in row] for row in ad['commands']]
 for a,w in cmd:
  nv,nd=v+H*a,d+H*w;assert -1<=nv<=2 and abs(nd)<=F(1,2) and max(abs(a),abs(w))<=1;p=propagate(p,v,nv,d,nd,direction(c),64)[-1];v,d=nv,nd
 r=job['receiver'];ep=sum((p[j]-F(r['center_pose'][j])).abs_upper() for j in (0,1));et=(p[2]-F(r['center_pose'][2])).abs_upper();assert ep<=F(r['rP']) and et<=F(r['rPsi']) and v==F(r['center_v']) and d==F(r['center_delta'])
 ds=coordinate(p,job['progression']['terminal_chart'])-m.I.raw(*job['progression']['source_s_Q']);JE=H*sum(a*a+w*w for a,w in cmd);Fhi=JE-(F(ds.l,m.Q) if job['objective']['kind']=='progress' else 0);assert Fhi<=F(ad['objective']['SOCP_F_upper'])+F(1,10**12) and Fhi<F(ad['baseline_objective']['true_F_lower'])-F(1,10**10)
 if job['arm'][0]=='F':assert all(x==0 for x in delta[1::2])
 return {'accepted':True,'exact_ET_quantization':True,'all_rows_passed':True,'min_slack':minslack,'receiver_position_error':ep,'receiver_yaw_error':et,'JE':JE,'actual_progress_Q':[ds.l,ds.u],'true_F_upper':Fhi,'own_replay_subdivisions':64,'first_delta_a':delta[0],'first_delta_omega':delta[1]}

def run():
 assert read(HERE/'results/numerical_complete.json')['passed'];ledger=read(HERE/'solver_calls.json');assert len(ledger)==16;events=read(HERE/'results/events.json');result=[];previous={};matrices={}
 for event in events:
  label=event['label'];arm=event['arm'];plan=load(f'results/{label}_committed_plan.json.gz');check=load(f'results/{label}_checkpoint.json.gz');state=restore(check['state']);assert content_id(plan)==event['new_plan'] and record(state)==restore_record(event['source']);assert content_id(check['plan'])==event['source_plan']
  if event['round']==2:assert event['source']==previous[arm]['successor'] and content_id(check['plan'])==previous[arm]['new_plan']
  own=[]
  for worker in event['workers']:
   c=worker['actor'];job=load(f'results/{label}_{c}_job.json.gz');ad=read(HERE/f'results/{label}_{c}_admission.json');assert job['model_id']==content_id({k:v for k,v in job.items() if k!='model_id'}) and job['parent']==event['source_plan'] and job['source']['history']==event['source']['history'] and job['source']['poses_Q']==event['source']['poses_Q'][c]
   stem=HERE/f'results/workers/{label}_{c}';raw=read(str(stem)+'_response.json');request=read(str(stem)+'_request.json');assert request['job']==job and raw['model_id']==job['model_id'] and raw['request_sha']==sha(str(stem)+'_request.json');assert len(raw['x'])==65 and len(raw['s'])==len(raw['z'])==raw['assembly']['shape'][0];own.append({'actor':c,'raw_status':raw['status'],'gap':raw['obj_val']-raw['obj_val_dual'],'checks':own_check(job,ad,state)})
   # Matrix evidence uses the executed compiler, but never invokes a solver.
   from local_mpc import assemble
   P,q,A,b,cones,settings=assemble(job)
   def sparse(x):return {'shape':list(x.shape),'indptr':x.indptr.tolist(),'indices':x.indices.tolist(),'data':x.data.tolist()}
   matrix={'P':sparse(P),'q':q.tolist(),'A':sparse(A),'b':b.tolist(),'cones':[str(x) for x in cones]};assert content_id(matrix)==raw['assembly']['matrix_id'];save(f'results/matrices/{job["model_id"]}.json.gz',matrix)
  full=audit(plan,output=HERE/f'results/independent/{label}_full.json');assert full['passed'];entry={'label':label,'workers':own,'full':full};result.append(entry);write(HERE/'results/independent/progress.json',result);previous[arm]=event;print('PASS independent',label,flush=True)
 # Two actual second-source corrupt proposals: transaction retains first-round policy.
 from contract import validate_binding,replay_suffix
 cp=load('results/O-P_r2_checkpoint.json.gz');st=restore(cp['state']);old=cp['plan'];base=load('results/O-P_r2_A1_job.json.gz');rejections=[]
 for case in ('invalid_certificate','missing_ACK'):
  if case=='invalid_certificate':
   bad=copy.deepcopy(base);bad['receiver']['K_id']='stale';rejected=False
   try:validate_binding(bad,old,st)
   except AssertionError:rejected=True
   assert rejected;chosen=old;reason='invalid/absent worker certificate -> within-contract anchor'
  else:
   participants=set(CARS);acks=set(CARS)-{'A1'};assert acks!=participants;chosen=old;reason='incomplete closure ACK -> prior jointly accepted plan'
  suffix=replay_suffix(chosen,st,f'rejections/{case}');independent=audit(chosen,output=HERE/f'results/independent/rejection_{case}.json');rejections.append({'case':case,'reason':reason,'source':record(st),'illegal_commits':0,'illegal_executions':0,'fallback_plan':content_id(chosen),'continuation':suffix,'independent':independent});print('PASS rejection',case,flush=True)
 write(HERE/'results/independent_acceptance.json',{'passed':True,'rounds':result,'rejections':rejections,'optimizer_calls':0,'attempts':16,'all_actual_second_sources_bound':True})

def restore_record(r):return record(restore(r))
if __name__=='__main__':
 try:run()
 except Exception as e:write(HERE/'results/independent_failure.json',{'layer':'independent_acceptance','error':repr(e),'traceback':traceback.format_exc(),'optimizer_calls':0});raise
