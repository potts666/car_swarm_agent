"""Portable content verification and zero-solve independent physical replays."""
import argparse,gzip
from common import *
from audit import audit

def plan(name):return json.loads(gzip.decompress((HERE/name).read_bytes()))
def run(full=False):
 hashes=read(HERE/'SHA256SUMS.json')
 for name,digest in hashes.items():assert sha(HERE/name)==digest,('changed',name)
 for name,digest in read(HERE/'executed_source_versions.numerical_v1.json').items():assert sha(HERE/'source_archive/numerical_v1'/name)==digest,('executed archive changed',name)
 for name,digest in read(HERE/'current_source_versions.repair_v2.json').items():assert sha(HERE/name)==digest,('current repair source changed',name)
 calls=read(HERE/'solver_calls.json');assert len(calls)==8 and [c['id'] for c in calls]==list(range(1,9));assert all(c['status']=='worker_error' for c in calls)
 for k in (0,1):
  for c in CARS:
   job=read(HERE/f'results/round_{k}_{c}_model.json');raw=read(HERE/f'results/round_{k}_{c}_raw.json');result=read(HERE/f'results/round_{k}_{c}_admission.json')
   assert raw['status']=='worker_error' and 'could not convert string to float' in raw['error'] and not result['accepted']
   assert all(F(b)>=0 for b in job['bounds'])
   assert all(sum(F(e[j])*col[j] for j in range(64))==0 for e in job['E'] for col in job['T_columns'])
 assert read(HERE/'results/adapter_repair_regression.json')['passed']
 assert read(HERE/'results/repair_v2/plan_summary.json')['all_absolute_deadlines_nonincrease']
 assert read(HERE/'results/repair_v2/protocol_checks.json')['passed']
 if full:
  for name in ('init_0','init_1','parallel_anchor','updated'):
   p=plan(f'results/{name}.json.gz');trace=HERE/'results/actual.jsonl.gz' if name=='updated' else None;assert audit(p,trace)['passed']
  p=plan('results/repair_v2/anchor.json.gz');assert audit(p,HERE/'results/repair_v2/actual.jsonl.gz')['passed']
  for prefix,newname in (('results','results/parallel_anchor.json.gz'),('results/repair_v2','results/repair_v2/anchor.json.gz')):
   for name,base in (('worker_error',newname),('missing_ACK','results/init_0.json.gz')):assert audit(plan(base),HERE/f'{prefix}/rejection_{name}.jsonl.gz')['passed']
 gate=read(HERE/'GATE.json');print(json.dumps({'integrity_passed':True,'files':len(hashes),'full_replay':full,'new_solver_calls':0,'experiment_passed':gate['passed'],'experiment_status':gate['status'],'numerical_budgeted_attempts':len(calls),'accepted_numerical_candidates':0}))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--full-replay',action='store_true');run(p.parse_args().full_replay)
