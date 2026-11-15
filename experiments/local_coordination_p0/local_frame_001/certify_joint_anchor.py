from common import *
from plan_io import load,source,save
from contract import patch,replay_suffix

def run():
 p=load('results/new.json.gz');sel=read(HERE/'results/source_selection.json');k=sel['cycle'];s=source(p,k);jobs=[];ads=[]
 for c in sel['D']+[sel['X']]:
  job=load(f'results/preflight/p{k}_{c}_job.json.gz');ref=list(map(F,job['reference']));T=job['lattice']['T'];scores=[sum(x*F(y) for x,y in zip(ref,col)) for col in T];i=max(range(len(T)),key=lambda i:abs(scores[i]));alpha=F(-1 if scores[i]>0 else 1,10**7);delta=[alpha*F(x) for x in T[i]];commands=[[ref[2*j]+delta[2*j],ref[2*j+1]+delta[2*j+1]] for j in range(32)];jobs.append(job);ads.append({'accepted':True,'model_id':job['model_id'],'commands':commands})
 trial=patch(p,s,jobs,ads);save('results/zero_joint_direction_plan.json.gz',trial);r=replay_suffix(trial,s,'zero_joint_direction');assert r['passed'];print('PASS zero-call joint direction complete',flush=True)
if __name__=='__main__':run()
