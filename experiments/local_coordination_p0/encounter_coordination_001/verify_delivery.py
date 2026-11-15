"""Portable identity/exact-update verification and zero-solve full replays."""
import argparse
from common import *
from plan_io import load
from audit import audit

def run(full=False):
 files=read(HERE/'SHA256SUMS.json')
 for name,h in files.items():assert sha(HERE/name)==h,('changed',name)
 for name,h in read(HERE/'executed_source_versions.json').items():
  assert sha(HERE/'source_archive/numerical_v1'/name)==h
  if name not in ('audit_updates.py','verify_delivery.py'):assert sha(HERE/name)==h
 for name,h in read(HERE/'verification_source_versions.json').items():assert sha(HERE/name)==h
 calls=read(HERE/'solver_calls.json');assert len(calls)==8 and all(c['native_entered'] and c['status'] in ('Solved','AlmostSolved') for c in calls)
 from audit_updates import verify
 verify(write_output=False)
 if full:
  for name,plan,trace in [('R','R_old','R_old'),('E','E_anchor','E_anchor'),('round0','round_0_committed',None),('actual','updated','actual')]+[(n,'rejection_'+n+'_plan','rejection_'+n) for n in ('worker_error','missing_ACK','unrelated_group')]:
   assert audit(load(f'results/{plan}.json.gz'),HERE/f'results/{trace}.jsonl.gz' if trace else None)['passed']
 assert read(HERE/'GATE.json')['passed'];print(json.dumps({'integrity_passed':True,'experiment_passed':True,'full_replay':full,'files':len(files),'native_calls_recorded':8,'new_native_calls':0}))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--full-replay',action='store_true');run(p.parse_args().full_replay)
