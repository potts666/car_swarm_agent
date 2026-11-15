"""Synthetic nullspace vectors test real admission; no optimizer result claim."""
from common import *
from local_mpc import admit

def run():
 rows=[]
 for k in (0,1):
  for c in CARS:
   job=read(HERE/f'results/repair_v2/round_{k}_{c}_model.json');ref=list(map(F,job['reference']));T=job['T_columns']
   scores=[sum(ref[j]*t[j] for j in range(64)) for t in T];idx=max(range(len(T)),key=lambda x:abs(scores[x]));assert scores[idx]!=0
   sign=-1 if scores[idx]>0 else 1;eps=F('0.00000314159265358979');x=[float(sign*eps*t) for t in T[idx]]
   raw={'status':'Solved','model_id':job['model_id'],'x':x,'fixture':True};result=admit(job,raw);assert result['accepted'] and result['all_equalities_exact']
   error=max(abs(F(str(a))-F(b)) for a,b in zip(x,result['delta']));assert error<=F(1,500000000000)
   rows.append({'round':k,'actor':c,'scope':'synthetic fixture, no native result','passed':True,'maximum_component_quantization_error':error,'exact_terminal_equations':True,'model_id':job['model_id']})
 write(HERE/'results/repair_v2/quantization_fixture.json',{'passed':True,'rows':rows,'native_attempts':0})
if __name__=='__main__':run()
