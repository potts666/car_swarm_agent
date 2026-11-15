"""Zero-call actual numerical-candidate failures and independent-block continuation."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from common import *
from protocol import *
from local_update import admit
from strategy import certify_combination

ledger_before=sha(HERE/'solver_calls.json');reg=read(HERE/'pre_registration.json');events=read(HERE/'results/L-new/summary.json')['events'];out=HERE/'results/rejection';cases=[]
e=events[0];actual=restore(m,e['actual_source']);cid=e['contract_id'];reference=reference_state(actual,cid);versions={c:0 for c in CARS};failed=reg['added'][0];plans=copy.deepcopy(e['plans']);job=next(j for j in e['jobs'] if j['actor']==failed)
for status in ('Exception','Timeout'):
 fallback,a=admit(reference,job,dict(status=status,logical_protocol_branch=True));assert not a['accepted'];mixed=copy.deepcopy(plans);mixed[failed]=fallback;q=prepare(actual,mixed,CARS,cid,versions);token=commit(actual,q,versions);wid,choice,_=select_for_execution(actual,[token],versions,cid);assert choice[failed]=='candidate_contract_anchor' and all(choice[c]=='new_candidate' for c in reg['active'] if c!=failed)
 if status=='Exception':
  trace=out/'numeric_worker_failure_execution.jsonl.gz';proof=certify_combination(actual,wid,trace);proof_id=put(HERE/'objects',proof)
 cases.append(dict(name='worker_'+status,layer=1,source=state_record(actual),witness=wid,choice=choice,trace='numeric_worker_failure_execution.jsonl.gz',proof_id=proof_id,other_numerical_candidates_kept=True,fork=True))
q=prepare(actual,plans,CARS,cid,versions)
try:commit(actual,q,versions,missing_ack=failed)
except ValueError as error:reason=str(error)
else:raise AssertionError('missing ACK committed')
wid,choice,_=select_for_execution(actual,[],versions,cid,fail=True);assert wid==actual['witness'];proof=certify_combination(actual,wid,out/'numeric_physical_transaction_rejection.jsonl.gz');cases.append(dict(name='missing_ACK_physical_migration',layer=2,source=state_record(actual),witness=wid,choice=choice,reason=reason,trace='numeric_physical_transaction_rejection.jsonl.gz',proof_id=put(HERE/'objects',proof),fork=True))
# After the first migration, external physical faces are fixed and single blocks can fail.
e=events[1];actual=restore(m,e['actual_source']);cid=e['contract_id'];versions={c:1 for c in CARS};plans=e['plans'];tokens=[]
for c in reg['active']:
 q=prepare(actual,plans,[c],cid,versions)
 if c==failed:
  try:commit(actual,q,versions,missing_ack=c)
  except ValueError:pass
  else:raise AssertionError('missing local ACK passed')
 else:tokens.append(commit(actual,q,versions))
wid,choice,_=select_for_execution(actual,tokens,versions,cid);assert choice[failed]=='old_contract_old_plan' and all(choice[c]=='new_candidate' for c in reg['active'] if c!=failed);proof=certify_combination(actual,wid,out/'numeric_independent_block_rejection.jsonl.gz');cases.append(dict(name='missing_ACK_independent_block',layer=2,source=state_record(actual),witness=wid,choice=choice,trace='numeric_independent_block_rejection.jsonl.gz',proof_id=put(HERE/'objects',proof),other_numerical_candidates_kept=True,fork=True))
assert ledger_before==sha(HERE/'solver_calls.json');write(out/'postsolve_checks.json',dict(passed=True,new_calls=0,cases=cases,ledger_sha256=ledger_before));print(dict(passed=True,cases=len(cases),new_calls=0),flush=True)
