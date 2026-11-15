"""Real-candidate damage and same-source full continuation without new solves."""
import copy,gzip
from common import *
from plan_io import load,save,record,source
from transaction import check_binding,select
from local_mpc import admit
from physics import step
from execution import gate
from audit import audit
from check_zero import rejects as expect_rejection

def run():
 before=load('results/round_1_plan.json.gz');accepted=load('results/round_1_committed.json.gz');checkpoint=read(HERE/'results/round_1_source.json');k0=checkpoint['cycle'];job=read(HERE/'results/round_1_A1_model.json');raw=read(HERE/'results/round_1_A1_raw.json');results=read(HERE/'results/rounds.json')[1]['admissions'];assert results['A1']['accepted']
 bad=copy.deepcopy(raw);bad['model_id']='old-source-model';expect_rejection(lambda:admit(job,bad))
 state=source(before,k0);assert content_id(record(state))==content_id(checkpoint)
 tests=[]
 for key in ('phase','parent','deadline','K_id'):
  damaged=copy.deepcopy(job['receiving_certificate']);damaged[key]='damaged';expect_rejection(lambda:check_binding(job,before,state,'A1',damaged));tests.append(key+' rejected')
 delayed=copy.deepcopy(accepted);delayed['per_car_policies']['A1']['goal']+=1;expect_rejection(lambda:select(before,delayed,CARS,CARS,results));tests.append('deadline increase rejected')
 mixed=copy.deepcopy(accepted);mixed['controls']['A1'][k0:k0+32]=before['controls']['A1'][k0:k0+32];failed=copy.deepcopy(results);failed['A1']={'accepted':False,'reason':'fixture corrupted real model binding / worker_error'}
 first,layer1=select(before,mixed,CARS,CARS,failed);assert first is mixed
 second,layer2=select(before,accepted,CARS,['A1','A2','B1'],results);assert second is before
 branches=[]
 unrelated=copy.deepcopy(before);unrelated['controls']['A2'][k0:k0+32]=accepted['controls']['A2'][k0:k0+32];private,private_layer=select(before,unrelated,['A2'],['A2'],{'A2':results['A2']});assert private is unrelated
 for name,plan,layer in [('worker_error',first,layer1),('missing_ACK',second,layer2),('unrelated_group',private,private_layer)]:
  save(f'results/rejection_{name}_plan.json.gz',plan);s=initial_state();path=HERE/f'results/rejection_{name}.jsonl.gz'
  with gzip.open(path,'wt') as f:
   for k in range(plan['B0']):
    release=gate(plan,s)
    row=step(s,{c:plan['controls'][c][k] for c in CARS},plan['owners'][k],4);row['cycle']=s['cycle'];f.write(json.dumps(row,default=str)+'\n')
    if k+1==k0:assert content_id(record(s))==content_id(checkpoint)
  proof=audit(plan,path,HERE/f'results/rejection_{name}_acceptance.json');branches.append({'name':name,'scope':'copied real round1 candidate fixture','layer':layer,'source_id':content_id(checkpoint),'same_actual_checkpoint':True,'receiver_phase':k0+32,'all_four_goals':proof['passed'],'steps':plan['B0'],'new_native_calls':0})
 write(HERE/'results/rejection_checks.json',{'passed':True,'real_candidate_source':raw['model_id'],'tests':tests,'branches':branches,'new_native_calls':0});print(json.dumps({'passed':True,'branches':3,'native_calls':0}))
if __name__=='__main__':run()
