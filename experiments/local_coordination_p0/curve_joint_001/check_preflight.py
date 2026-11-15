"""Bounded zero-solve source/model/lattice certification; never releases native."""
import time,traceback
from common import *
from plan_io import load,save,source,record
from curve_model import generate,evaluate
from quantization import quantize
from local_mpc import rollout,check_receiver,values
from transport import exchange

def run():
 p=load('results/R_original.json.gz');sel=read(HERE/'results/source_probe.json');k=sel['cycle'];s=source(p,k);actors=sel['actors'];out=[]
 for phase in (k,k+1):
  st=source(p,phase)
  for arm in ('F-E','O-E','F-P','O-P'):
   for c in actors:
    label=f'{arm}_{phase}_{c}';beg=time.monotonic();job,cert=generate(p,st,c,arm);save(f'results/preflight/{label}_job.json.gz',job)
    raw,meta=exchange(job,'assemble',HERE/f'results/preflight/{label}');base=rollout(job,st,[[F(x) for x in row] for row in p['controls'][c][phase:phase+32]]);recv=check_receiver(job,base);entry={'label':label,'model_id':job['model_id'],'assembly':raw['assembly'],'receiver':recv,'wall':time.monotonic()-beg}
    if c==actors[0] and arm=='O-P' and phase==k:
     probe=read(HERE/f'results/probe_{c}.json');direction=list(map(F,probe['direction']));alpha=F(1,100000);delta,q=quantize([float(alpha*x) for x in direction],F(1),job['lattice']);ev=evaluate(job,delta);assert ev['passed'],[r for r in ev['rows'] if r['slack']<0]
     refs=list(map(F,job['reference']));cmd=[[refs[2*j]+delta[2*j],refs[2*j+1]+delta[2*j+1]] for j in range(32)];actual=rollout(job,st,cmd);check_receiver(job,actual);v=values(job,actual,delta);b=values(job,base,[F(0)]*64);assert delta[1]!=0 and v['true_F_upper']<b['true_F_lower'] and v['SOCP_F_upper']<b['true_F_lower'];entry['positive_joint_direction']={'alpha':alpha,'delta':delta,'quantization':q,'c_cert':probe['c_cert'],'values':v,'baseline':b}
    out.append(entry);write(HERE/'results/preflight/progress.json',out);print('PASS',label,'wall',round(entry['wall'],2),flush=True)
 write(HERE/'results/preflight/zero_model_certificate.json',{'passed':True,'forecast_phase_only':True,'actual_second_source_requires_regeneration':True,'models':out,'solver_calls':0})
if __name__=='__main__':
 try:run()
 except Exception as e:
  write(HERE/'results/preflight/failure.json',{'passed':False,'layer':'zero_model_preflight','error':repr(e),'traceback':traceback.format_exc(),'solver_calls':len(read(HERE/'solver_calls.json'))});raise
