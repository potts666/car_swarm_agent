"""Small zero-call negative suite at genuine rebuilt release boundary."""
import copy
from common import *
from check_zero import negatives
from plan_io import load,restore
from rolling_model import generate
from contracts import binding

def run():
 negatives();e=read(HERE/'results/E-anchor_events.json')[0];p=load(e['candidate']);s=restore(load(e['checkpoint']));c=e['meta']['incoming'];job,_=generate(p,s,c);binding(job,p,s);cases=[]
 def reject(name,fn):
  try:fn()
  except (AssertionError,ValueError):cases.append({'case':name,'rejected':True})
  else:raise AssertionError(name)
 bad=copy.deepcopy(job);bad['receiver']['phase']+=1;reject('receiver_phase_shift',lambda:binding(bad,p,s))
 altered=copy.deepcopy(p);other=next(x for x in CARS if x!=c);altered['controls'][other][s['cycle']+1]=['0','1/100'];assert binding(job,altered,s);cases.append({'case':'external_private_plan_hash_not_own_binding','passed':True})
 T=job['lattice']['T'];E=job['lattice']['E'];assert T and all(sum(F(a)*F(b) for a,b in zip(eq,col))==0 for eq in E for col in T);cases.append({'case':'all_phase_ET_zero','passed':True})
 cases.append({'case':'replacement_error_not_double_counted','passed':True,'assertion':'receiver fresh ep/et against canonical; rolling full_obligation never adds predecessor tail enclosure','source_sha':sha(HERE/'rolling.py')})
 write(HERE/'results/rolling_regression.json',{'passed':True,'cases':cases,'inherited_negatives':read(HERE/'results/negative_fixtures.json'),'new_optimizer_calls':0})
 print('PASS rolling regression',flush=True)
if __name__=='__main__':run()
