"""Portable integrity/default exact audits and optional full physical replay."""
import argparse,gzip
from common import *
from audit import audit
from run_update import load

def run(full=False):
 hashes=read(HERE/'SHA256SUMS.json')
 for name,h in hashes.items():assert sha(HERE/name)==h,('changed',name)
 for name,h in read(HERE/'executed_source_versions.json').items():assert sha(HERE/name)==h and sha(HERE/'source_archive/numerical_v1'/name)==h
 calls=read(HERE/'solver_calls.json');assert len(calls)==8 and [c['id'] for c in calls]==list(range(1,9)) and all(c['native_entered'] and c['status'] in ('Solved','AlmostSolved') for c in calls)
 from audit_updates import verify
 verify(write_output=False)
 if full:
  for name,trace in [('parallel_anchor',None),('round_0_committed',None),('updated','results/actual.jsonl.gz'),('rejection_worker_error_plan','results/rejection_worker_error.jsonl.gz'),('rejection_missing_ACK_plan','results/rejection_missing_ACK.jsonl.gz')]:assert audit(load(f'results/{name}.json.gz'),HERE/trace if trace else None)['passed']
 assert read(HERE/'GATE.json')['passed']
 print(json.dumps({'integrity_passed':True,'experiment_passed':True,'files':len(hashes),'full_replay':full,'new_native_calls':0,'recorded_native_calls':8}))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--full-replay',action='store_true');run(p.parse_args().full_replay)
