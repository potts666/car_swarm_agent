"""Freeze joint rational/deadline repaired source and8-call preregistration."""
import shutil
from common import *
from plan_io import load
from transaction import select

def run():
 assert read(HERE/'solver_calls.json')==[]
 assert all(read(HERE/f'results/{name}.json')['passed'] for name in ('event_scan','worker_precheck','protocol_precheck'))
 base=load('results/R_old.json.gz');plan=load('results/E_anchor.json.gz');event=read(HERE/'event_registration.json');assert content_id(base)==event['old_id'] and content_id(plan)==event['new_id']
 assert any(r['id']==event['selected'] and r['eligible'] for r in read(HERE/'results/proposal_table.json'))
 assert read(HERE/'results/R_old_acceptance.json')['passed'] and read(HERE/'results/E_anchor_acceptance.json')['passed']
 acks={c:{'actor':c,'old_contract':content_id(base),'new_contract':content_id(plan),'own_inputs':content_id(plan['controls'][c]),'deadline':plan['per_car_policies'][c]['goal'],'accepted':plan['per_car_policies'][c]['goal']<=base['per_car_policies'][c]['goal']} for c in CARS}
 assert all(m['accepted'] for m in acks.values());write(HERE/'results/anchor_acks.json',acks)
 assert select(base,plan,CARS,list(acks),{})[0] is plan
 sources={p.name:sha(p) for p in sorted(HERE.glob('*.py'))};archive=HERE/'source_archive/numerical_v1';archive.mkdir(parents=True,exist_ok=False)
 for name in sources:shutil.copyfile(HERE/name,archive/name)
 reg={'passed':True,'task':'encounter_coordination_001','event_id':content_id(event),'source_cycle':event['source_cycle'],'encounter_actors':event['actors'],'limit':8,'rounds':[event['source_cycle'],event['source_cycle']+1],'actors':[list(CARS),list(CARS)],'selected':content_id(plan),'initialization':content_id(base),'B0':plan['B0'],'variables_per_actor':64,'eta':'1/50','trust_l2':'1/1000','quantum':'1/1000000000000','first_input_note':'actual moving encounter source, retain all active quotas; report numeric first-input changes separately','repairs':['Fraction before float in shared actual assembly','first car service without max-prep wait','per-car deadlines checked in generator and live selector'],'worker_gate':'actual JSON bytes and full construct for8 forecast models; exact actual-round models rechecked before solve','native_entry':'durable ready/parent release handshake before solve','executed_sources':sources,'precheck_id':content_id(read(HERE/'results/worker_precheck.json')),'failures_counted':True,'automatic_repeats':False}
 write(HERE/'pre_registration.json',reg);write(HERE/'executed_source_versions.json',sources);write(HERE/'frozen_config.json',{'physical':CFG,'map':read(HERE/'map_config.json'),'eta':'1/50','rho':'1/1000','quantum':'1/1000000000000','N':32,'numerical_scope':'initial straight / stopped steering with exact affine ZOH and fixed steering perturbations'})
 print(json.dumps({'ready':True,'budget':8,'calls':0,'source_count':len(sources)}))
if __name__=='__main__':run()
