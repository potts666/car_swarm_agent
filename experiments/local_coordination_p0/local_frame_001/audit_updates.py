"""Independent raw/row/quant/objective and complete mixed physical audit."""
import copy,time,traceback,gzip
from common import *
from plan_io import load,save,source,restore,record
from audit import audit
from physics import propagate,body,predicted_nodes
from progression import coordinate
from curve_bounds import sqrt_upper
from frame import physical_id
from receivers import reference

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


def full(plan):
 key=physical_id(plan);path=HERE/f'results/independent/physical/{key}.json'
 if path.exists():r=read(path);assert r['physical_id']==key;return {'reused_content_checked_physical_id':key,'certificate':str(path.relative_to(HERE)),'result':r}
 result=audit(plan);result['physical_id']=key;write(path,result);return {'certificate':str(path.relative_to(HERE)),'result':result}

def trace_inclusion(plan,certificate):
 ref=reference(plan);count=0;maximum=F(0)
 with gzip.open(HERE/certificate['trace'],'rt') as f:
  for text in f:
   row=json.loads(text);k=row['cycle'];assert row['commands']=={c:list(map(str,map(F,plan['controls'][c][k-1]))) for c in CARS}
   for c in CARS:
    p=[m.I.raw(*q) for q in row['poses_Q'][c]];z=[m.I.raw(*q) for q in ref[k]['poses_Q'][c]];error=sum((p[j]-z[j]).abs_upper() for j in (0,1))+RB*(p[2]-z[2]).abs_upper();assert error<=F(1,40),('independent frame inclusion',c,k,error);maximum=max(maximum,error)
   count+=1
 assert count==plan['B0']-certificate['source'];return {'passed':True,'nodes':count,'max_body_domain_error':maximum,'continuous_between_nodes':'per-corner physical acceleration bound15; frame published original tube+.025+15*h²/8'}

def actual_successor(event,plan):
 state=restore(event['source']);k=state['cycle'];commands={c:tuple(map(F,plan['controls'][c][k])) for c in CARS};nodes=predicted_nodes(state,commands,16);nextstate=event.get('successor',event.get('actual_successor'))
 for c in CARS:
  for a,b in zip(nodes[c][-1],nextstate['poses_Q'][c]):assert max(a.l,b[0])<=min(a.u,b[1]),('actual first source integration',c,k)
  assert F(nextstate['v'][c])==state['v'][c]+H*commands[c][0] and F(nextstate['delta'][c])==state['delta'][c]+H*commands[c][1]
 assert nextstate['history']==content_id({'parent':state['history'],'commands':commands,'owner':plan['owners'][k]});return True

def run():
 assert read(HERE/'results/numerical_complete.json')['passed'];calls=read(HERE/'solver_calls.json');assert len(calls)==12;events=read(HERE/'results/events.json');previous={};rounds=[];mix_reports=[]
 for e in events:
  label=e['label'];org=e['organization'];cp=load(f'results/{label}_checkpoint.json.gz');state=restore(cp['state']);plan=load(f'results/{label}_committed.json.gz');assert content_id(plan)==e['new_id'] and content_id(cp['anchor'])==e['anchor_id'];assert e['outer_frame_unchanged'];actual_successor(e,plan)
  if e['round']==2:assert e['source']==previous[org]['successor'] and content_id(cp['old'])==previous[org]['new_id']
  own=[]
  for c in read(HERE/'pre_registration.json')['actors']:
   job=load(f'results/{label}_{c}_job.json.gz');ad=read(HERE/f'results/{label}_{c}_admission.json');raw=read(HERE/f'results/workers/{label}_{c}_response.json');request=HERE/f'results/workers/{label}_{c}_request.json';assert read(request)['job']==job and raw['request_sha']==sha(request);assert job['model_id']==content_id({k:v for k,v in job.items() if k!='model_id'}) and raw['model_id']==job['model_id'];assert job['source']['poses_Q']==e['source']['poses_Q'][c] and job['source']['history']==e['source']['history'];assert job['parent']==content_id(cp['anchor']['controls'][c]);assert job['receiver']['K_id']==content_id(cp['anchor']['controls'][c][state['cycle']+32:]);own.append({'actor':c,'status':raw['status'],'gap':raw['obj_val']-raw['obj_val_dual'],'own_check':own_check(job,ad,state)})
   from local_mpc import assemble
   P,q,A,b,cones,settings=assemble(job)
   def sparse(x):return {'shape':list(x.shape),'indptr':x.indptr.tolist(),'indices':x.indices.tolist(),'data':x.data.tolist()}
   matrix={'P':sparse(P),'q':q.tolist(),'A':sparse(A),'b':b.tolist(),'cones':[str(c) for c in cones]};assert content_id(matrix)==raw['assembly']['matrix_id'];save(f'results/matrices/{job["model_id"]}.json.gz',matrix)
   if org=='L':assert job==load(f'results/C_r{e["round"]}_{c}_job.json.gz')
  for mix in e['mixes']:
   p=load(mix['plan']);cert=read(HERE/mix['certificate']);result=full(p);inc=trace_inclusion(p,cert);assert result['result']['passed'];mix_reports.append({'label':label,'D_new':mix['D_new'],'X_new':mix['X_new'],'plan':mix['plan'],'physical':result,'inclusion':inc});print('PASS independent mix',label,int(mix['D_new']),int(mix['X_new']),flush=True)
  check=full(plan);rounds.append({'label':label,'own':own,'physical':check,'actual_successor_checked':True,'outer_frame_id':plan['outer_frame_id']});write(HERE/'results/independent/progress.json',{'rounds':rounds,'mixes':mix_reports});previous[org]=e
 # Post branches are independent zero-call work; required before final acceptance.
 post=read(HERE/'results/post_branches.json');rejections=[];extensions=[]
 for e in post['rejections']:
  p=load(e['plan']);result=full(p);inc=trace_inclusion(p,read(HERE/e['certificate']));actual_successor(e,p);assert e['external_new_first_executed'] and e['external_changed_output_ticks'];assert e['transaction']['X_committed'] and e['transaction']['external_ack_still_valid'];rejections.append({'name':e['name'],'passed':True,'physical':result,'inclusion':inc,'X_new_first':True});print('PASS independent rejection',e['name'],flush=True)
 for e in post['extension']['branches']:
  p=load(e['plan']);result=full(p);inc=trace_inclusion(p,read(HERE/e['certificate']));extensions.append({'name':e['name'],'passed':True,'physical':result,'inclusion':inc,'new_frame_committed':e['new_frame_committed']});print('PASS independent extension',e['name'],flush=True)
 assert set(post['extension']['closure']['participants'])==set(CARS);assert post['extension']['post_frame_dependency_discovery']['private_changes_need_no_external_reACK'];write(HERE/'results/independent_acceptance.json',{'passed':True,'rounds':rounds,'mixes':mix_reports,'rejections':rejections,'extensions':extensions,'all_actual_sources':True,'all_C_L_models_equal':True,'original':full(load('results/old.json.gz')),'native_attempts':12,'new_optimizer_calls':0})
if __name__=='__main__':
 try:run()
 except Exception as e:write(HERE/'results/independent_failure.json',{'layer':'independent_acceptance','error':repr(e),'traceback':traceback.format_exc(),'optimizer_calls':0});raise
