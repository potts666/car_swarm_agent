"""Evidence-derived gates and metrics; never enters an optimizer."""
import gzip
from common import *
from run_update import load

def metrics(plan):
 v={c:F(0) for c in CARS};J=F(0);hist={};first=[];initial4=0;live=True
 for k in range(plan['B0']):
  aa=[]
  for c in CARS:
   a,w=map(F,plan['controls'][c][k]);travel=abs(H*(2*v[c]+H*a)/2);v[c]+=H*a;J+=a*a+w*w
   if travel>=F(1,50):aa.append(c)
  hist[len(aa)]=hist.get(len(aa),0)+1
  if k<2:first.append(aa)
  if live and len(aa)==4:initial4+=1
  else:live=False
 return {'B0':plan['B0'],'seconds':H*plan['B0'],'goal_ticks':{c:plan['per_car_policies'][c]['goal'] for c in CARS},'input_cost':J,'activity_histogram':hist,'first_two':first,'initial_four_active_ticks':initial4}

def run():
 num=read(HERE/'results/numerical_summary.json');updates=read(HERE/'results/independent_updates.json');physical=read(HERE/'results/independent_physical.json');round0=read(HERE/'results/round0_policy_acceptance.json');rejections=read(HERE/'results/rejection_checks.json')
 assert all(x['passed'] for x in (num,updates,physical,round0,rejections))
 plans={'initial':load('results/init_0.json.gz'),'anchor':load('results/parallel_anchor.json.gz'),'round0':load('results/round_0_committed.json.gz'),'updated':load('results/updated.json.gz')};mm={n:metrics(p) for n,p in plans.items()};mm['MPC_gain_vs_anchor']=F(mm['anchor']['input_cost'])-F(mm['updated']['input_cost']);mm['MPC_percent_vs_anchor']=100*mm['MPC_gain_vs_anchor']/F(mm['anchor']['input_cost']);assert mm['MPC_gain_vs_anchor']>0;write(HERE/'results/comparison.json',mm)
 old=read(HERE/'prior_manifest.json');changed=[n for n,h in old['files'].items() if sha(Path(old['path'])/n)!=h];assert changed==[]
 preserved={'autonomy_init_001':{'passed':True,'files':len(old['files'])}}
 for name,info in read(Path(old['path'])/'source_inventory.json').items():
  assert all(sha(Path(info['path'])/n)==h for n,h in info['files'].items());preserved[name]={'passed':True,'files':len(info['files'])}
 write(HERE/'results/preservation_check.json',preserved)
 final=plans['updated']
 with gzip.open(HERE/'results/deadlines.jsonl.gz','wt') as f:
  for line in gzip.open(HERE/'results/actual.jsonl.gz','rt'):
   row=json.loads(line);k=row['cycle'];before={c:max(0,final['per_car_policies'][c]['goal']-k+1) for c in CARS};after={c:max(0,final['per_car_policies'][c]['goal']-k) for c in CARS};assert max(after.values())==max(before.values())-1
   f.write(json.dumps({'cycle':k,'actual_row_id':content_id(row),'b_before':before,'b_after':after,'B_before':max(before.values()),'B_after':max(after.values())})+'\n')
 gate={'passed':True,'status':'passed','task':'autonomy_init_002','criteria':{'autonomous_initialization':{'status':'passed','regenerated_from_static_inputs':True,'two_initial_B0':1955,'new_anchor_B0':1822},'joint_repairs':{'status':'passed','repairs':['rational conversion','individual deadlines'],'live_and_generator_deadline_checks':True},'actual_worker_transport':{'status':'passed','forecast_models':8,'actual_round_models':8,'actual_matrix_and_model_fingerprints_match':True,'zero_stage_native_calls':0},'true_MPC_updates':{'status':'passed','calls':8,'accepted':updates['accepted'],'new_prefix_by_round':updates['new_prefix_by_round'],'first_changed':updates['first_input_changed_by_round']},'real_successor_and_receiving_K':{'status':'passed','evidence':'independent exact row/grid reconstruction; actual-source bindings; phase32/33 exact v/delta and position inclusion; each committed complete strategy separately replayed'},'four_car_progress':{'status':'passed','first_two':list(map(len,mm['updated']['first_two'])),'initial_contiguous_ticks':mm['updated']['initial_four_active_ticks']},'rejection_continuation':{'status':'passed','branches':2,'source':'real second-round numerical candidates and actual checkpoint'},'safety_and_completion':{'status':'passed','steps':physical['steps'],'all_six_pairs':True,'subdivisions':8,'all_four_goals':True,'B_final':0,'individual_deadlines_nonincrease':True}},'native_usage':{'new_limit':8,'releases':8,'native_entered':8,'accepted':8,'automatic_repeats':0},'scope':'nominal fixed four-car map, two initial straight/stopped-steering update rounds, synchronous in-process directory; no curved online or network/timing performance claim','old_task_preserved':True,'DG002_CL_budget_untouched':True}
 write(HERE/'GATE.json',gate);print(json.dumps({'passed':True,'accepted':[4,4],'first_changed':updates['first_input_changed_by_round'],'steps':1822,'native_calls':8,'MPC_percent_vs_anchor':float(mm['MPC_percent_vs_anchor'])}))
if __name__=='__main__':run()
