"""Zero-call damaged branches and actual safe fallback suffixes."""
import copy,gzip
from common import *
from discovery import Directory
from initialize import init_plan
from map_domain import graph_cycle
from local_mpc import prepare
from transaction import check_binding,select
from physics import step
from audit import audit

def load(n):return json.loads(gzip.decompress((HERE/n).read_bytes()))
def expect_rejection(fn):
 try:fn()
 except AssertionError:return True
 raise AssertionError('damaged fixture was accepted')
def run(new_name='results/repair_v2/anchor.json.gz',out='results/repair_v2'):
 old=load('results/init_0.json.gz');new=load(new_name);state=initial_state();tests=[]
 def record(name,detail):tests.append({'name':name,'passed':True,'scope':'fixture/fork unless specified','detail':detail})
 cycle=graph_cycle(CARS,[('A2','B2'),('B2','A2')]);assert cycle==['A2','B2','A2'];assert 3<len(CARS)
 damaged_graph=copy.deepcopy(read(HERE/'results/map_radius_6.json'));damaged_graph['edges']=[('A2','B2'),('B2','A2')];damaged_graph['DAG_passed']=False
 expect_rejection(lambda:init_plan(read(HERE/'map_config.json'),CFG,TASKS,damaged_graph,read(HERE/'requests.json')[0]));record('cycle_and_insufficient_buffers',{'cycle':cycle,'free_non_goal':3,'required':4})
 directory=Directory();goalbox=[F(42),F(50),F(1),F(5)];directory.register('A1',goalbox,500,float('inf'),'permanent_goal')
 assert directory.query('B1',goalbox,2000,2001)['owners']==['A1'];missing=Directory();assert missing.query('B1',goalbox,2000,2001)['owners']==[];record('omitted_permanent_goal_misses_late_pair',{'late_time':2000,'target':'A1'})
 directory.register('A1',goalbox,0,10,'intent');snap=directory.query('A1',goalbox,0,10);directory.register('B1',goalbox,0,10,'intent');assert not directory.commit(snap);snap=directory.query('A1',goalbox,0,10);assert directory.commit(snap);record('simultaneous_intent_version_change',directory.events)
 expect_rejection(lambda:select(old,new,['A1','A2','B1'],['A1','A2','B1'],{}));record('resource_endpoint_omission','B2 missing from changed reservation closure')
 job=prepare(old,state,'A1');expect_rejection(lambda:check_binding(job,new,state,'A1'));record('new_contract_old_anchor','parent/reference identity rejected')
 job=prepare(new,state,'A1');certificate={'parent':job['parent'],'phase':32,'deadline':new['per_car_policies']['A1']['goal'],'K_id':content_id(new['controls']['A1'][32:])};assert check_binding(job,new,state,'A1',certificate)
 for key in ('parent','phase','deadline','K_id'):
  bad=copy.deepcopy(certificate);bad[key]='damaged';expect_rejection(lambda:check_binding(job,new,state,'A1',bad))
 record('receiver_K_phase_parent_deadline_damage','four separate rejection checks')
 q=[round(F(1,3)*10**12)/F(10**12)]*3;assert sum(q)!=1
 bad=copy.deepcopy(job);idx=bad['equality_labels'].index('terminal_v_exact');bad['equality_labels'].pop(idx);bad['E'].pop(idx);bad['model_id']=content_id({k:v for k,v in bad.items() if k!='model_id'});expect_rejection(lambda:check_binding(bad,new,state,'A1'));record('zero_radius_quantization_equality_omitted',{'absolute_grid_sum':str(sum(q)),'required':'1','affine_increment_grid':'E T = 0 and anchor origin checked in every real model'})
 failed={c:{'accepted':False,'reason':'fixture worker_error'} for c in CARS};selected,layer=select(old,new,CARS,CARS,failed);assert selected is new;record('worker_error_first_layer',layer)
 delayed=copy.deepcopy(new);delayed['per_car_policies']['A1']['goal']=old['per_car_policies']['A1']['goal']+1;expect_rejection(lambda:select(old,delayed,CARS,CARS,failed));record('individual_deadline_increase','A1+1 rejected')
 selected,layer=select(old,new,CARS,['A1','A2','B1'],failed);assert selected is old;record('missing_ACK_second_layer',layer)
 # Actual execution starts from the same checkpoint: new-C at phase 1 (all
 # source commands a=1), or pretransaction old-C at phase 0. Never splice poses.
 branches=[]
 for name,plan,phase in [('worker_error',new,1),('missing_ACK',old,0)]:
  s=initial_state();path=HERE/f'{out}/rejection_{name}.jsonl.gz'
  with gzip.open(path,'wt') as f:
   for k in range(plan['B0']):
    row=step(s,{c:plan['controls'][c][k] for c in CARS},plan['owners'][k],4);row['cycle']=s['cycle'];f.write(json.dumps(row,default=str)+'\n')
    if k+1==phase:
     checkpoint={'poses_Q':{c:pose_rows(s['poses'][c]) for c in CARS},'v':s['v'],'delta':s['delta'],'cycle':phase}
  result=audit(plan,path,HERE/f'{out}/rejection_{name}_acceptance.json');branches.append({'name':name,'phase':phase,'plan_id':content_id(plan),'passed':result['passed'],'checkpoint':checkpoint if phase else 'actual initial state','steps_to_goal':plan['B0']-phase})
 write(HERE/f'{out}/protocol_checks.json',{'passed':True,'tests':tests,'branches':branches,'optimizer_calls':0})
 print(json.dumps({'passed':True,'tests':len(tests),'branches':len(branches),'optimizer_calls':0}))
if __name__=='__main__':run()
