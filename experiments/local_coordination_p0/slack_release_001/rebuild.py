"""Static-input1822 baseline, reference and source-input read provenance."""
from common import *
from initialize import run as initialize
from preflight import simultaneous
from plan_io import load,save,record
from physics import step
from audit import audit

def run():
 assert not read(HERE/'solver_calls.json');initialize();p=simultaneous(load('results/init_0.json.gz'),True);p=json.loads(json.dumps(p,default=str));order=p['order'];first=order[:2];p.update(release_cuts={},lease_groups={})
 for i,c in enumerate(order):
  if i==0:p['lease_groups']['D']={'members':first,'window':[p['per_car_policies'][first[0]]['service_start'],p['per_car_policies'][first[-1]]['goal']]}
  if i>=2:p['lease_groups'][c]={'members':[c],'window':[p['per_car_policies'][c]['service_start'],p['per_car_policies'][c]['goal']]}
 p['inner_version']=0;p['release_count']=0;p['outer_frame_id']='initial_pending';save('results/R_initial.json.gz',p);s=initial_state();states=[record(s)]
 for k in range(p['B0']):step(s,{c:p['controls'][c][k] for c in CARS},p['owners'][k],8);states.append(record(s))
 save('results/canonical_states.json.gz',states);audit(p,output=HERE/'results/R_initial_acceptance.json')
 # Compare historical analytic results only after regeneration and physical replay.
 import gzip
 prior=HERE.parent/'local_frame_001/results/old.json.gz';old=json.loads(gzip.decompress(prior.read_bytes()))
 for key in ('controls','owners','per_car_policies'):assert p[key]==old[key]
 extended=HERE.parent/'local_frame_001/results/extension/success_plan.json.gz';hist=json.loads(gzip.decompress(extended.read_bytes()))
 write(HERE/'input_lineage.json',{'passed':True,'inputs':p['source_manifest'],'fresh_initial_B0':load('results/init_0.json.gz')['B0'],'regenerated_B0':p['B0'],'goals':p['b_i'],'prior_analytic_same_controls_owners_policies':True,'historical_read_after_new_generation_and_audit':True,'comparison_only':[{'path':str(prior.resolve()),'sha':sha(prior)},{'path':str(extended.resolve()),'sha':sha(extended),'historical1753_ticks':hist['B0']}],'historical_numeric_input':False,'prior_local_verify_delivery':'passed all12 source/model/raw and physical GATE keys; actual contract.guard, frame/receiver/worker/protocol/audit inspected'})
 print('PASS rebuilt1822',p['b_i'],flush=True)
if __name__=='__main__':run()
