"""Rebuild original analytic policy from task inputs, compare history afterward."""
from common import *
from initialize import run as initialize
from preflight import simultaneous
from encounter import active_pairs,propose
from plan_io import save,source
from audit import audit

def run():
 initialize();import gzip
 initial=json.loads(gzip.decompress((HERE/'results/init_0.json.gz').read_bytes()));parallel=simultaneous(initial,True)
 for k in range(parallel['B0']):
  state=source(parallel,k);events,active=active_pairs(parallel,state)
  if events:break
 a,b=events[0]['actors'];plan=propose(parallel,state,a,b)
 prior=HERE.parent/'encounter_coordination_001';old=json.loads(gzip.decompress((prior/'results/E_anchor.json.gz').read_bytes()))
 for key in ('controls','owners','per_car_policies'):
  assert json.loads(json.dumps(plan[key],default=str))==old[key],key
 save('results/R_original.json.gz',plan);write(HERE/'results/reference_relation.json',{'passed':True,'new_id':content_id(plan),'prior_analytic_id':content_id(old),'controls_calendar_goals_identical':True,'metadata_difference':'new locally refined interval integration source SHA; physical map and actual model unchanged','prior_numeric_result_input':False,'ticks':plan['B0']})
 audit(plan,output=HERE/'results/R_original_acceptance.json')
 print(json.dumps({'passed':True,'ticks':plan['B0'],'new_solver_calls':0}))
if __name__=='__main__':run()
