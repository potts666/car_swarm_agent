"""Required repaired deadline/source/equality checks before native release."""
import copy
from common import *
from transaction import select,check_binding
from local_mpc import prepare
from run_update import load
from check_protocol import expect_rejection

def run():
 base=load('results/init_0.json.gz');plan=load('results/parallel_anchor.json.gz');state=initial_state();checks=[]
 bad=copy.deepcopy(plan);bad['per_car_policies']['A1']['goal']=base['per_car_policies']['A1']['goal']+1
 expect_rejection(lambda:select(base,bad,CARS,CARS,{}));checks.append('A1 deadline+1 rejected')
 oldjob=prepare(base,state,'A1');expect_rejection(lambda:check_binding(oldjob,plan,state,'A1'));checks.append('old parent anchor rejected')
 for c in CARS:
  j=prepare(plan,state,c);check_binding(j,plan,state,c,j['receiving_certificate'])
  for key in ('phase','deadline','K_id','parent'):
   cert=copy.deepcopy(j['receiving_certificate']);cert[key]='corrupt';expect_rejection(lambda:check_binding(j,plan,state,c,cert))
  bad=copy.deepcopy(j);i=bad['equality_labels'].index('terminal_v_exact');bad['equality_labels'].pop(i);bad['E'].pop(i);bad['model_id']=content_id({k:v for k,v in bad.items() if k!='model_id'});expect_rejection(lambda:check_binding(bad,plan,state,c));checks.append(c+' receiving/K/equality checks')
 assert select(base,plan,CARS,['A1','A2','B1'],{})[0] is base
 expect_rejection(lambda:select(base,plan,['A1','A2','B1'],['A1','A2','B1'],{}));checks.append('missing ACK/closure')
 write(HERE/'results/protocol_precheck.json',{'passed':True,'checks':checks,'native_calls':0})
if __name__=='__main__':run()
