"""Regenerate autonomous initialization and certify the fixed legal candidate."""
import gzip
from common import *
from initialize import run as initialize
from preflight import simultaneous,save
from audit import audit
from transaction import select

def load(p):return json.loads(gzip.decompress(Path(p).read_bytes()))
def run():
 old=HERE.parent/'autonomy_init_001';initialize();reuse=[]
 for k in (0,1):
  current=load(HERE/f'results/init_{k}.json.gz');prior=load(old/f'results/init_{k}.json.gz');assert current==prior
  acceptance=read(old/f'results/init_{k}_acceptance.json');assert acceptance['passed']
  reuse.append({'plan':k,'identity':content_id(current),'prior_file_sha':sha(old/f'results/init_{k}.json.gz'),'current_sha':sha(HERE/f'results/init_{k}.json.gz'),'prior_acceptance':acceptance})
 base=load(HERE/'results/init_0.json.gz');plan=simultaneous(base,True);previous=load(old/'results/repair_v2/anchor.json.gz');assert content_id(plan)==content_id(previous)
 assert select(base,plan,CARS,CARS,{})[0] is plan
 save('results/parallel_anchor.json.gz',plan);audit(plan,output=HERE/'results/anchor_acceptance.json')
 write(HERE/'input_lineage.json',{'prior_task':str(old),'fresh_init_input_manifest':base['source_manifest'],'regenerated_initializations_bit_identical':reuse,'fixed_anchor_id':content_id(plan),'fixed_anchor_prior_id':content_id(previous),'historical_complete_witness_input':False,'source':'map/config/local tasks + finite certified graph; prior plans accessed afterward for content comparison only'})
 write(HERE/'results/preparation.json',{'passed':True,'B_initial':base['B0'],'B_anchor':plan['B0'],'deadlines':{c:plan['per_car_policies'][c]['goal'] for c in CARS},'independent_anchor_replay':True,'native_calls':0})
if __name__=='__main__':run()
