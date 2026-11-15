"""Assemble and construct every repaired native model, never call solve."""
from common import *
from local_mpc import assemble

def run():
 import clarabel,numpy as np
 results=[]
 for k in (0,1):
  for c in CARS:
   job=read(HERE/f'results/round_{k}_{c}_model.json');P,q,A,b,cones,settings=assemble(job)
   assert P.shape==(64,64) and A.shape[1]==64 and len(q)==64 and len(b)==A.shape[0]
   assert np.all(np.isfinite(A.data)) and np.all(np.isfinite(b)) and np.all(np.isfinite(q))
   e=len(job['E']);nn=len(job['bounds']);assert np.all(b[:e]==0) and np.all(b[e:e+nn]>=0)
   assert b[e+nn]==.001 and np.all(b[e+nn+1:]==0)
   solver=clarabel.DefaultSolver(P,q,A,b,cones,settings)
   results.append({'round':k,'actor':c,'passed':True,'shape':A.shape,'model_id':job['model_id'],'constructed':True,'solve_called':False})
 write(HERE/'results/adapter_repair_regression.json',{'passed':True,'models':results,'native_solver_calls':0,'budgeted_worker_attempts_still':len(read(HERE/'solver_calls.json'))})
if __name__=='__main__':run()
