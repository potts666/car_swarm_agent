"""Independent execution and obligation DAG judge; does not release any solve."""
import copy,gzip,time
from common import *
from plan_io import load,restore
from rolling_recover import nodes,objects,plan_at,run_all
from audit import audit
from curve_model import evaluate
from local_mpc import rollout,check_receiver,values,assemble
from quantization import quantize
from contracts import physical_id,binding
from rolling_model import classify
from canonical import reference

def run():
 started=time.monotonic();summary=read(HERE/'results/R-rolling_summary.json');ledger=read(HERE/'budget_ledger.json');ledger_sha=sha(HERE/'budget_ledger.json');ns=nodes();assert len(ns)==summary['cycles'];assert [n['cycle'] for n in ns]==list(range(ns[0]['cycle'],ns[0]['cycle']+len(ns)));p=plan_at(ns[0]['cycle']);parent=content_id(p);counts={};rejected=[];obligations=[]
 for n in ns:
  assert n['parent']==parent;assert content_id({k:v for k,v in n.items() if k!='node_id'})==n['node_id'];parent=n['node_id'];k=n['cycle'];state=restore(n['source'])
  if n['release'] and n['release']['transaction']['accepted']:p=load(n['release']['candidate'])
  assert physical_id(p)==n['plan_before'];new=copy.deepcopy(p)
  for c,row in n['rows'].items():
   counts[row['classification']]=counts.get(row['classification'],0)+1
   if 'model_object' not in row:rejected.append({'cycle':k,'car':c,'reason':row['reason']});continue
   job=objects(row['model_object']);binding(job,p,state);refs=reference(p)
   if F(refs[k]['v'][c])==0:assert state['v'][c]==0 and state['delta'][c]==F(refs[k]['delta'][c]),('independent phase source guard',c,k)
   assert evaluate(job,[F(0)]*64)['passed'];assert all(sum(F(a)*F(b) for a,b in zip(eq,col))==0 for eq in job['lattice']['E'] for col in job['lattice']['T']);kind,proof=classify(p,state,c,job)
   if row['classification']!='fallback_after_failure':assert kind==row['classification']
   if row['adopted']:
    raw=objects(row['raw_object']);ad=objects(row['admission_object']);delta,quant=quantize(raw['x'][:64],F(ad['alpha']),job['lattice']);assert list(map(F,ad['delta']))==delta;assert evaluate(job,delta)['passed'];assert raw['model_id']==job['model_id'];assert raw['status'] in ('Solved','AlmostSolved');ref=list(map(F,job['reference']));commands=[[ref[2*j]+delta[2*j],ref[2*j+1]+delta[2*j+1]] for j in range(32)];assert [[F(x) for x in q] for q in ad['commands']]==commands;r=rollout(job,state,commands,64);check_receiver(job,r)
    for phase,node in enumerate(r['nodes'],k+1):
     if F(refs[phase]['v'][c])==0:assert F(node['v'])==0 and F(node['delta'])==F(refs[phase]['delta'][c]),('independent phase seam',c,phase)
    base_energy=H*sum(u*u for u in ref);assert r['energy']<base_energy;ob=objects(row['obligation_object']);assert ob['passed'] and ob['source_history']==state['history'] and ob['old_K_id']==job['receiver']['K_id'] and ob['tail_controls_id']==content_id(p['controls'][c][k+32:]);target=reference(p)[k+32];end=[m.I.raw(*q) for q in r['pose_Q']];nom=[m.I.raw(*q) for q in target['poses_Q'][c]];ep=sum((end[j]-nom[j]).abs_upper() for j in (0,1));et=(end[2]-nom[2]).abs_upper();bound=ep+(F(job['receiver']['L_remaining'])+RB)*et;assert bound<F(1,40);obligations.append({'cycle':k,'actor':c,'full_bound':bound,'raw_exact_adoption':True,'receiver':True});new['controls'][c][k:k+32]=commands
   elif row['classification']=='fallback_after_failure':rejected.append({'cycle':k,'car':c,'reason':row.get('reason')})
  assert physical_id(new)==n['plan_after'];assert all(new['controls'][c][k]==n['commands'][c] or list(map(F,new['controls'][c][k]))==list(map(F,n['commands'][c])) for c in CARS);p=new
  if (k-ns[0]['cycle'])%8==0:write(HERE/'results/audit_progress.json',{'cycle':k,'cycles_checked':k-ns[0]['cycle']+1,'total_cycles':len(ns),'accepted_obligations_checked':len(obligations),'seconds':time.monotonic()-started})
 assert len(ledger)<=2048 and len({(r['cycle'],r['actor'],r['source_version']) for r in ledger})==len(ledger);assert len(ns)<=512
 for name in ('E-anchor','R-rolling'):
  audit(load(f'results/{name}_final_plan.json.gz'),trace=str(HERE/f'results/{name}_actual.jsonl.gz'),output=HERE/f'results/{name}_rolling_independent.json')
 run_all();recovery=read(HERE/'results/rolling_recovery.json')
 for b in recovery['branches']:audit(load(b['plan']),trace=str(HERE/b['full_trace']),output=HERE/f'results/{b["name"]}_independent.json')
 assert sha(HERE/'budget_ledger.json')==ledger_sha
 write(HERE/'results/rolling_independent.json',{'passed':True,'cycles':len(ns),'car_cycles':4*len(ns),'all_actual_raw_adoption_and_ET_exact':True,'obligations':obligations,'counts':counts,'fallbacks':rejected,'full_trajectories':['E-anchor','R-rolling']+[b['name'] for b in recovery['branches']],'original_eight_subdivision_six_pair_physical_replay':True,'actual_prefix_receiver_subdivisions':64,'shared_algorithms':['original interval arithmetic and ZOH','original original-row evaluator/grid basis','independent audit traversal and actual full physical replay'],'new_optimizer_calls':0,'wall_seconds':time.monotonic()-started})
 print('PASS independent full trajectories and obligation DAG',len(obligations),flush=True)
if __name__=='__main__':run()
