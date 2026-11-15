"""Freeze joint rational/deadline repaired source and8-call preregistration."""
import shutil
from common import *
from run_update import load
from transaction import select

def run():
 assert read(HERE/'solver_calls.json')==[]
 assert all(read(HERE/f'results/{name}.json')['passed'] for name in ('preparation','worker_precheck','protocol_precheck'))
 base=load('results/init_0.json.gz');plan=load('results/parallel_anchor.json.gz');assert select(base,plan,CARS,CARS,{})[0] is plan
 sources={p.name:sha(p) for p in sorted(HERE.glob('*.py'))};archive=HERE/'source_archive/numerical_v1';archive.mkdir(parents=True,exist_ok=False)
 for name in sources:shutil.copyfile(HERE/name,archive/name)
 reg={'passed':True,'task':'autonomy_init_002','limit':8,'rounds':[0,1],'actors':[list(CARS),list(CARS)],'selected':content_id(plan),'initialization':content_id(base),'B0':plan['B0'],'variables_per_actor':64,'eta':'1/50','trust_l2':'1/1000','quantum':'1/1000000000000','first_input_note':'round0 a0=1 fixed by eta=.02 at zero speed; accept changed numerical tails and report first-input changes separately','repairs':['Fraction before float in shared actual assembly','first car service without max-prep wait','per-car deadlines checked in generator and live selector'],'worker_gate':'actual JSON bytes and full construct for8 forecast models; exact actual-round models rechecked before solve','native_entry':'durable ready/parent release handshake before solve','executed_sources':sources,'precheck_id':content_id(read(HERE/'results/worker_precheck.json')),'failures_counted':True,'automatic_repeats':False}
 write(HERE/'pre_registration.json',reg);write(HERE/'executed_source_versions.json',sources);write(HERE/'frozen_config.json',{'physical':CFG,'map':read(HERE/'map_config.json'),'eta':'1/50','rho':'1/1000','quantum':'1/1000000000000','N':32,'numerical_scope':'initial straight / stopped steering with exact affine ZOH and fixed steering perturbations'})
 print(json.dumps({'ready':True,'budget':8,'calls':0,'source_count':len(sources)}))
if __name__=='__main__':run()
