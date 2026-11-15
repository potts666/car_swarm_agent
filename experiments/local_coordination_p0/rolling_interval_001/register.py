"""Freeze method/settings only after full zero-call coverage; no solver release."""
import sys,shutil
from common import *
from plan_io import load

def run():
 assert not read(HERE/'budget_ledger.json');assert read(HERE/'results/rolling_preflight.json')['passed']
 import clarabel,numpy,scipy
 files={str(p.relative_to(HERE)):sha(p) for p in HERE.rglob('*.py') if 'source_archive' not in p.parts and 'instructions' not in p.parts and 'results' not in p.parts}
 archive=HERE/'source_archive/rolling_v1';archive.mkdir(parents=True,exist_ok=False)
 for name in files:
  target=archive/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(HERE/name,target)
 write(HERE/'executed_source_versions.json',files)
 write(HERE/'pre_registration.json',{'task':'rolling_interval_001','passed':True,'interval':read(HERE/'interval_registration.json'),'preflight_sha':sha(HERE/'results/rolling_preflight.json'),'constants':{'h':'1/5','N':32,'rho':'1/1000','eta':'1/50','Delta':'1/1000000000000','R':[1,1],'S':[1,1],'objective':'original JE=h sum(a²+omega²)','units':'normalized input squared times second (U_J), not joule','report_F_lambda_UJ_per_m':1},'settings':{'clarabel':clarabel.__version__,'numpy':numpy.__version__,'scipy':scipy.__version__,'python':sys.version,'max_iter':200,'tol_gap_abs':1e-10,'tol_gap_rel':1e-10,'tol_feas':1e-10,'time_limit':'infinity (original default)','pipe_timeout_seconds':30,'worker_order':list(CARS),'worker_execution':'sequential independent spawned processes; common sealed logical source'},'budget':2048,'native_retries':0,'all_abnormal_releases_count':True,'first_stage':32,'source_hashes':files,'source_inventory_sha':sha(HERE/'source_inventory.json')})
 print('REGISTERED rolling_v1; zero attempts',flush=True)
if __name__=='__main__':run()
