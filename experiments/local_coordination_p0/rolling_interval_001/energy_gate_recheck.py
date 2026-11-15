"""Exact original energy needs no arbitrary positive improvement floor."""
from common import *
from rolling_recover import nodes,objects,plan_at
from local_mpc import admit
from plan_io import restore,save

def run():
 ledger=sha(HERE/'budget_ledger.json');proofs=[]
 for n in nodes():
  if n['cycle'] not in (521,522,523):continue
  row=n['rows']['B1'];job=objects(row['model_object']);raw=objects(row['raw_object']);ad=admit(job,raw,restore(n['source']));assert ad['accepted'];gain=F(ad['objective_improvement_lower']);assert 0<gain<F(1,10**10);key=content_id(ad);save(f'results/energy_gate_regression/{key}.json.gz',ad);proofs.append({'cycle':n['cycle'],'raw_id':row['raw_object'],'admission_id':key,'exact_gain':gain,'gain_UJ_display':float(gain),'not_retroactively_adopted':True})
 assert len(proofs)==3 and sha(HERE/'budget_ledger.json')==ledger;write(HERE/'results/energy_gate_regression.json',{'passed':True,'samples':proofs,'new_optimizer_calls':0,'argument':'JE of exact rational commands is exact. Require strict JE_new<JE_anchor. Retain positive comparison budget for progress objective intervals. Physical rows/grid/true Q/K/full tail unchanged.'});print('PASS exact-JE gate from all3 saved raw; no new solves',flush=True)
if __name__=='__main__':run()
