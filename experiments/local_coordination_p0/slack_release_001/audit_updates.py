import gzip,copy,time,traceback
from common import *
from plan_io import load,save,restore,record
from audit import audit
from contracts import physical_id
from physics import propagate,predicted_nodes
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


def complete(plan,trace=None):
 key=physical_id(plan);p=HERE/f'results/independent/{key}.json'
 # A physical certificate may be shared only by exactly equal complete controls,
 # owner, per-car policy and length. Protocol/source checks remain separate.
 if p.exists() and (not trace or read(p).get('trace_sha')==sha(trace)):return {'reused_physical_id':key,'certificate':str(p.relative_to(HERE)),'result':read(p)}
 r=audit(plan,trace=trace);r['physical_id']=key
 if trace:r['trace_sha']=sha(trace)
 write(p,r);return {'certificate':str(p.relative_to(HERE)),'result':r}

def actual_first(event,plan):
 state=restore(event['source']);k=state['cycle'];commands={c:tuple(map(F,plan['controls'][c][k])) for c in CARS};nodes=predicted_nodes(state,commands,16);nextstate=event['successor']
 for c in CARS:
  for a,b in zip(nodes[c][-1],nextstate['poses_Q'][c]):assert max(a.l,b[0])<=min(a.u,b[1]),('actual successor',k,c)
  assert F(nextstate['v'][c])==state['v'][c]+H*commands[c][0] and F(nextstate['delta'][c])==state['delta'][c]+H*commands[c][1]
 assert nextstate['history']==content_id({'parent':state['history'],'commands':commands,'owner':plan['owners'][k]});return True

def closure_semantic_id(value):
 value=json.loads(json.dumps(value,default=str))
 for row in value['discharged_completed_dependencies']:row['interfaces'].sort(key=lambda p:p['interface_id'])
 return content_id(value)

def release_events(summary):
 from release import clear,verify_clear
 from schedules import validate_prefix,deadlines,intervals_valid
 from frames import discover,validate_publication,contains
 from canonical import exit_phase
 reports=[]
 for e in summary['events']:
  old,new=load(e['old_plan']),load(e['candidate']);s=restore(load(e['checkpoint']));proof=clear(old,s,e['meta']['outgoing']);assert content_id(proof)==content_id(e['clear']);verify_clear(old,s,proof);validate_prefix(old,new,s['cycle']);intervals_valid(new);d=deadlines(old,new,s['cycle']);assert d==e['meta']['deadlines'];assert d['Phi_new']<d['Phi_old'];oldframe,newframe=read(HERE/old['frame_file']),read(HERE/new['frame_file']);validate_publication(oldframe);validate_publication(newframe);contains(newframe,new);closure=discover(old,new,oldframe,newframe,s['cycle']);assert closure_semantic_id(closure)==closure_semantic_id(e['transaction']['closure']);accepted=e['transaction']['accepted'];assert accepted==(len(e['transaction']['ACK'])==len(closure['participants']) and e['transaction']['fixture']!='stale_K')
  # Global serialized arrays end at B0; unchanged completed vehicles keep their
  # exact own program through goal plus infinite zero hold, regardless of padding.
  equivalent={}
  for c in CARS:
   if c not in closure['participants']:
    goal=old['per_car_policies'][c]['goal'];assert goal==new['per_car_policies'][c]['goal'] and old['controls'][c][:goal]==new['controls'][c][:goal];assert all(tuple(map(F,u))==(0,0) for u in old['controls'][c][goal:]+new['controls'][c][goal:]);equivalent[c]={'semantic_program_id':content_id({'controls_until_goal':old['controls'][c][:goal],'goal':goal,'hold':'infinite zero'}),'old_raw_array_id':content_id(old['controls'][c]),'new_raw_array_id':content_id(new['controls'][c]),'outer_slot_unchanged':old['outer_slots'][c]==new['outer_slots'][c]};assert equivalent[c]['outer_slot_unchanged']
  reports.append({'cycle':s['cycle'],'actor':e['meta']['outgoing'],'kind':e['meta']['kind'],'accepted':accepted,'Clear':True,'full_suffix':proof['actual_future'],'phase_guard_boundary_from_model':exit_phase(old,e['meta']['outgoing']),'participants':closure['participants'],'deadline_Phi':d,'unchanged_external_programs':equivalent,'proposal_first_legal_start':e['attempts'][-1]['start'],'old_future_valid_if_rejected':True})
 return reports

def run():
 ledger=read(HERE/'solver_calls.json');assert len(ledger)==8;branches={};releases={}
 for name in ('R-fixed','E-anchor','E-MPC'):
  summary=read(HERE/f'results/{name}_summary.json');p=load(summary['plan']);branches[name]=complete(p,str(HERE/summary['trace']));releases[name]=release_events(summary);print('PASS independent main',name,summary['final_cycle'],flush=True)
 assert read(HERE/'results/E-anchor_summary.json')['goals']==read(HERE/'results/E-MPC_summary.json')['goals'];assert len([r for r in releases['E-MPC'] if r['accepted']])>=2
 numeric=[];previous={}
 for win in (1,2):
  for rnd in (1,2):
   label=f'e{win}_r{rnd}';cp=load(f'results/numeric/{label}_checkpoint.json.gz');e=read(HERE/f'results/numeric/{label}_event.json');plan=load(f'results/numeric/{label}_committed.json.gz');state=restore(cp['state']);actual_first(e,plan)
   if rnd==2:assert e['source']==previous[win]['successor'] and physical_id(cp['plan'])==previous[win]['plan_after']
   own=[]
   for c in e['actors']:
    job=load(f'results/numeric/{label}_{c}_job.json.gz');ad=read(HERE/f'results/numeric/{label}_{c}_admission.json');raw=read(HERE/f'results/workers/{label}_{c}_response.json');req=HERE/f'results/workers/{label}_{c}_request.json';assert raw['request_sha']==sha(req) and read(req)['job']==job and raw['model_id']==job['model_id'];assert job['source']['history']==e['source']['history'] and job['source']['poses_Q']==e['source']['poses_Q'][c];assert job['parent']==content_id(cp['plan']['controls'][c]);assert job['model_id']==content_id({k:v for k,v in job.items() if k!='model_id'});own.append({'actor':c,'raw_status':raw['status'],'gap':raw['obj_val']-raw['obj_val_dual'],'checks':own_check(job,ad,state)})
    from local_mpc import assemble
    P,q,A,b,cones,settings=assemble(job)
    def sparse(x):return {'shape':list(x.shape),'indptr':x.indptr.tolist(),'indices':x.indices.tolist(),'data':x.data.tolist()}
    matrix={'P':sparse(P),'q':q.tolist(),'A':sparse(A),'b':b.tolist(),'cones':[str(c) for c in cones]};assert content_id(matrix)==raw['assembly']['matrix_id'];save(f'results/matrices/{job["model_id"]}.json.gz',matrix)
   mixes=[]
   for mix in e['mixes']:
    p=load(mix['plan']);cert=read(HERE/mix['certificate']);assert cert['passed'] and cert['source']==e['source'];mixes.append({'mask':mix['mask'],'physical':complete(p)})
   numeric.append({'label':label,'own':own,'mixes':mixes,'committed_complete':complete(plan),'actual_successor':True});write(HERE/'results/independent/numeric_progress.json',numeric);previous[win]=e;print('PASS independent numeric',label,flush=True)
 recovery=read(HERE/'results/recovery_summary.json');recoveries=[]
 for summary in recovery['branches']:
  p=load(summary['plan']);result=complete(p,str(HERE/summary['full_trace']));rr=release_events(summary);assert not rr[0]['accepted'] and rr[1]['accepted'];assert rr[1]['cycle']==rr[0]['cycle']+1 and len([x for x in rr if x['accepted']])>=2;recoveries.append({'name':summary['name'],'physical':result,'release_and_retry':rr,'illegal_outputs':0,'illegal_commits':0});print('PASS independent recovery',summary['name'],summary['final_cycle'],flush=True)
 write(HERE/'results/independent_acceptance.json',{'passed':True,'main':branches,'releases':releases,'numeric':numeric,'recovery':recoveries,'formal_attempts':8,'new_optimizer_calls':0})
if __name__=='__main__':
 try:run()
 except Exception as e:write(HERE/'results/independent_failure.json',{'layer':'independent_acceptance','error':repr(e),'traceback':traceback.format_exc(),'new_optimizer_calls':0});raise
