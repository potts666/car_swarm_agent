import importlib.util,traceback
from common import *
from plan_io import load,restore
from schedules import propose
from rolling_model import generate,classify
from local_mpc import construct,rollout,check_receiver

def run():
 p=load('results/initial_bound.json.gz');s=restore(load('results/canonical_states.json.gz')[406]);p,_=propose(p,s,p['order'][0],406)
 spec=importlib.util.spec_from_file_location('inherited_curve',HERE/'source_archive/inherited/curve_model.py');legacy=importlib.util.module_from_spec(spec);spec.loader.exec_module(legacy)
 try:legacy.generate(p,s,p['order'][1],'O-E')
 except Exception as error:write(HERE/'results/inherited_phase_limitation.json',{'cycle':406,'actor':p['order'][1],'error':repr(error),'traceback':traceback.format_exc(),'new_optimizer_calls':0})
 out=[]
 for c in CARS:
  j,cert=generate(p,s,c);kind,proof=classify(p,s,c,j);solver,info=construct(j);r=rollout(j,s,[[F(j['reference'][2*t]),F(j['reference'][2*t+1])] for t in range(32)]);check_receiver(j,r);out.append({'actor':c,'classification':kind,'proof':proof,'assembly':info})
 write(HERE/'results/model_probe.json',{'passed':True,'cars':out,'new_optimizer_calls':0});print([(r['actor'],r['classification']) for r in out],flush=True)
if __name__=='__main__':run()
