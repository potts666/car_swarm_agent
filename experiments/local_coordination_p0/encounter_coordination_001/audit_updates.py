"""Independent exact ZOH/quantization/source audit of accepted numerical updates."""
from collections import Counter
import gzip
from common import *
from physics import propagate
from plan_io import load,restore,record,source as rebuild_source

def end(state,c,commands):
 p=state['poses'][c];v=state['v'][c];d=state['delta'][c]
 for a,w in commands:
  nv,nd=v+H*a,d+H*w;assert max(abs(a),abs(w))<=1 and -1<=nv<=2 and abs(nd)<=F(1,2)
  p=propagate(p,v,nv,d,nd,direction(c),8)[-1];v,d=nv,nd
 return p,v,d

def verify(write_output=True):
 base=load('results/R_old.json.gz');anchor=load('results/E_anchor.json.gz');event=read(HERE/'event_registration.json');calls=read(HERE/'solver_calls.json');assert len(calls)==8 and all(c['native_entered'] for c in calls)
 trace=[json.loads(s) for s in gzip.open(HERE/'results/actual.jsonl.gz','rt')];rows=[];first_changed=[];prefix_changed=[]
 for r in (0,1):
  k=event['source_cycle']+r
  parent=load(f'results/round_{r}_plan.json.gz');committed=load(f'results/round_{r}_committed.json.gz');source=read(HERE/f'results/round_{r}_source.json');state=restore(source);pc=fc=0
  assert source['cycle']==k
  if r==0:assert content_id(record(rebuild_source(base,k)))==content_id(record(state))==event['source_id']
  else:
   prev=trace[k-1];assert source['poses_Q']==prev['poses_Q'] and source['v']==prev['v'] and source['delta']==prev['delta'] and source['history']==prev['history']
   assert content_id(parent)==content_id(load(f'results/round_{r-1}_committed.json.gz'))
  for c in CARS:
   stem=f'results/round_{r}_{c}';job=read(HERE/(stem+'_model.json'));raw=read(HERE/(stem+'_raw.json'));a=read(HERE/(stem+'_admission.json'));meta=read(HERE/(stem+'_transport.json'));j=next(t for t in calls if t['round']==r and t['actor']==c)
   assert j['model_id']==raw['model_id']==job['model_id']==a['model_id'] and raw['status'] in ('Solved','AlmostSolved') and a['accepted']
   assert job['model_id']==content_id({key:value for key,value in job.items() if key!='model_id'})
   assert j['native_entering_durable_at']<=raw['native_entered_at']<=raw['native_finished_at']
   request=Path(HERE/(stem+'_request.json')).read_bytes();assert hashlib.sha256(request).hexdigest()==raw['request_sha']==meta['request_sha']
   zero=read(HERE/(stem+'_zero_response.json'));assert zero['assembly']==raw['assembly'] and zero['solve_called'] is False
   assert job['parent']==content_id(parent) and job['cycle']==k
   assert job['state']=={'poses_Q':source['poses_Q'][c],'v':source['v'][c],'delta':source['delta'][c]}
   ref=[F(x) for u in parent['controls'][c][k:k+32] for x in u];assert list(map(F,job['reference']))==ref
   delta=list(map(F,a['delta']));q=[F(x)+F(y) for x,y in zip(ref,delta)];cmd=[[q[2*t],q[2*t+1]] for t in range(32)]
   assert cmd==[[F(x),F(y)] for x,y in a['commands']]==[[F(x),F(y)] for x,y in committed['controls'][c][k:k+32]]
   assert trace[k]['commands'][c]==list(map(str,cmd[0]))
   assert sum(x*x for x in delta)<=F(1,1000)**2
   assert sum(x*x for x in q)<sum(x*x for x in ref)
   assert all(sum(F(e[t])*delta[t] for t in range(64))==0 for e in job['E'])
   assert all(sum(F(e[t])*delta[t] for t in range(64))<=F(b) for e,b in zip(job['rows'],job['bounds']))
   # Re-derive all physical linear rows directly from signed straight ZOH.
   vv=state['v'][c];expected=[];bounds=[];labels=[]
   for t in range(64):
    unit=[F(i==t) for i in range(64)];expected.extend([unit,[-x for x in unit]]);bounds.extend([1-ref[t],1+ref[t]]);labels.extend([f'input+:{t}',f'input-:{t}'])
   for t in range(32):
    vv+=H*ref[2*t];velocity_row=[H if z%2==0 and z//2<=t else F(0) for z in range(64)];s=[H*H*(F(t-z//2)+F(1,2)) if z%2==0 and z//2<=t else F(0) for z in range(64)]
    expected.extend([velocity_row,[-x for x in velocity_row],s,[-x for x in s]]);bounds.extend([2-vv,vv+1,F(1,100),F(1,100)]);labels.extend([f'v+:{t}',f'v-:{t}',f'physical_x+:{t}',f'physical_x-:{t}'])
   assert expected==[[F(x) for x in row] for row in job['rows']] and bounds==list(map(F,job['bounds'])) and labels==job['labels']
   n=min(32,parent['per_car_policies'][c]['approach_end']-k);first=1 if k==0 else 0
   eq=[]
   for t in range(32):
    eq.append(tuple(F(z==2*t+1) for z in range(64)))
    if t>=n or (k==0 and t==0):eq.append(tuple(F(z==2*t) for z in range(64)))
   eq.extend(tuple(F((z//2)**power) if z%2==0 and z//2<n else F(0) for z in range(64)) for power in (0,1))
   assert Counter(eq)==Counter(tuple(map(F,e)) for e in job['E'])
   T=[]
   for t in range(first,n-2):T.append([1 if z==2*t or z==2*t+4 else -2 if z==2*t+2 else 0 for z in range(64)])
   assert T==job['T_columns']
   alpha=F(a['alpha']);z=[alpha*sum(F(t-i+1)*F(str(raw['x'][2*(first+i)])) for i in range(t+1)) for t in range(len(T))]
   ints=[round(x*10**12) for x in z];assert ints==a['integer_coordinates']
   assert delta==[sum(F(T[t][i]*ints[t],10**12) for t in range(len(T))) for i in range(64)]
   projected=[sum(T[t][i]*z[t] for t in range(len(T))) for i in range(64)];quant_error=max(abs(x-y) for x,y in zip(delta,projected));assert quant_error<=F(1,500000000000)
   cert=job['receiving_certificate'];assert cert['parent']==content_id(parent) and cert['phase']==k+32 and cert['K_id']==content_id(parent['controls'][c][k+32:])
   assert cert['deadline']==parent['per_car_policies'][c]['goal'] and committed['controls'][c][k+32:]==parent['controls'][c][k+32:]
   assert committed['per_car_policies'][c]['goal']<=base['per_car_policies'][c]['goal']
   assert cert['phase_domain_id']==content_id(read(HERE/'results/phase_receivers.json')[c])
   ep,ev,ed=end(state,c,cmd);rp,rv,rd=end(state,c,[[ref[2*t],ref[2*t+1]] for t in range(32)])
   assert ev==rv and ed==rd and sum((ep[t]-rp[t]).abs_upper() for t in (0,1))<F(1,1000000) and (ep[2]-rp[2]).abs_upper()<F(1,100000000)
   changed=cmd!=[[ref[2*t],ref[2*t+1]] for t in range(32)];assert changed;pc+=changed;fc+=cmd[0]!=[ref[0],ref[1]]
   progress=abs(H*(2*state['v'][c]+H*cmd[0][0])/2);assert progress>=F(1,50)
   rows.append({'round':r,'cycle':k,'actor':c,'passed':True,'model_id':job['model_id'],'actual_source_bound':True,'native_status':raw['status'],'new_prefix':changed,'first_changed':cmd[0]!=[ref[0],ref[1]],'progress':progress,'phase':k+32,'K_id':cert['K_id'],'endpoint_v':ev,'endpoint_delta':ed,'quantization_error':quant_error,'input_cost_decrease':sum(x*x for x in ref)-sum(x*x for x in q)})
  prefix_changed.append(pc);first_changed.append(fc)
 out={'passed':True,'rows':rows,'accepted':[4,4],'new_prefix_by_round':prefix_changed,'first_input_changed_by_round':first_changed,'real_sources':True,'receivers_and_K':True,'exact_grid_equalities':True,'native_calls_checked':8,'new_native_calls':0}
 if write_output:write(HERE/'results/independent_updates.json',out)
 print(json.dumps({'passed':True,'accepted':[4,4],'new_prefix':prefix_changed,'first_changed':first_changed}))
if __name__=='__main__':verify()
