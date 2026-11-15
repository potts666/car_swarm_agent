"""Fresh source whitelist replay; historical whole-policy reads blocked until both audits."""
import hashlib,json,os,shutil,subprocess,sys
from pathlib import Path
R=Path(__file__).resolve().parent
SEED=R/'results/source_isolation'
RUNNER=r'''
import sys,json,gzip,traceback
from pathlib import Path
R=Path(__file__).resolve().parent
phase='fresh_generation';opened=[];denied=[]
class HistoricalReadDenied(RuntimeError):pass
def hook(event,args):
 if phase!='fresh_generation':return
 if event in ('subprocess.Popen','os.system','os.posix_spawn'):raise HistoricalReadDenied('no subprocesses/solver workers during isolated generation')
 if event=='open' and isinstance(args[0],(str,bytes)):
  p=Path(args[0]).resolve()
  if '/experiments/local_coordination_p0/' in str(p):
   if not p.is_relative_to(R):denied.append(str(p));raise HistoricalReadDenied('outside isolated source whitelist: '+str(p))
   opened.append(str(p.relative_to(R)))
sys.addaudithook(hook)
from common import *
from plan_io import load,save
import local_mpc
def forbidden(*a,**kw):raise AssertionError('optimizer entry forbidden in source check')
local_mpc.native=forbidden
import rebuild
try:rebuild.run()
except HistoricalReadDenied:pass
else:raise AssertionError('legacy comparison was not blocked by whitelist')
assert read(R/'results/R_initial_acceptance.json')['passed']
from engine import run
from frames import certify,slot_descriptors,bind
p=load('results/R_initial.json.gz');frame=certify(p,slot_descriptors(p,True));p=bind(p,frame);save('results/initial_bound.json.gz',p);result=run('E-anchor','anchor')
from audit import audit
q=load(result['plan']);independent=audit(q,trace=str(R/result['trace']),output=R/'results/E-anchor_independent.json');assert independent['passed']
assert read(R/'solver_calls.json')==[]
phase='after_both_independent_replays'
# First whole-history comparisons of this child occur only after both fresh audits.
actual=R.parent.parent
checks=[]
for fresh,prior in [(load('results/R_initial.json.gz'),actual/'results/R_initial.json.gz'),(q,actual/'results/E-anchor_final_plan.json.gz')]:
 old=json.loads(gzip.decompress(prior.read_bytes()));assert all(fresh[k]==old[k] for k in ('controls','owners','per_car_policies'));checks.append({'compared':str(prior),'sha':sha(prior),'controls_owners_policies_equal':True})
write(R/'source_certificate.json',{'passed':True,'fresh_initial':1822,'fresh_anchor':result['final_cycle'],'historical_whole_policy_reads_blocked_until_both_audits':True,'blocked_legacy_comparison':denied,'project_open_paths_before_comparison':sorted(set(opened)),'comparisons':checks,'new_optimizer_calls':0})
print('PASS source-isolated initial and E-anchor; zero new solves',flush=True)
'''
def main():
 before=hashlib.sha256((R/'budget_ledger.json').read_bytes()).hexdigest();SEED.mkdir(exist_ok=False)
 for p in (R/'source_archive/rolling_v1').rglob('*'):
  if p.is_file():
   target=SEED/p.relative_to(R/'source_archive/rolling_v1');target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,target)
 for name in ('map_config.json','vehicle_configs.json','local_tasks.json','requests.json','event_selection_rule.json'):shutil.copy2(R/name,SEED/name)
 (SEED/'results').mkdir();shutil.copy2(R/'results/map_radius_6.json',SEED/'results/map_radius_6.json');(SEED/'solver_calls.json').write_text('[]\n');(SEED/'source_runner.py').write_text(RUNNER)
 env=os.environ.copy();env.update(PYTHONPATH='/tmp/dmpc-p0-deps',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
 with (R/'results/source_isolation.log').open('w') as log:code=subprocess.run([sys.executable,str(SEED/'source_runner.py')],stdout=log,stderr=subprocess.STDOUT,env=env,timeout=3600).returncode
 assert code==0,('isolated source generation failure',code,str(R/'results/source_isolation.log'));assert before==hashlib.sha256((R/'budget_ledger.json').read_bytes()).hexdigest();cert=json.loads((SEED/'source_certificate.json').read_text());(R/'results/source_isolation_certificate.json').write_text(json.dumps(cert,indent=2)+'\n');print('PASS source isolation and identical policies; no new native calls',flush=True)
if __name__=='__main__':main()
