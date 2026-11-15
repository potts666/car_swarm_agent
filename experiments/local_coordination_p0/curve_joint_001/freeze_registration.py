"""Release gate: shared physics and source freeze after all zero-call certificates."""
import shutil,sys,csv,platform
from common import *
from plan_io import load,source,record

def run():
 assert not read(HERE/'solver_calls.json');assert read(HERE/'results/preflight/zero_model_certificate.json')['passed'];assert read(HERE/'results/contract_certificate.json')['passed'];domain=read(HERE/'results/reference_domain_audit.json');assert domain['passed'] and F(domain['minimum']['resource'])>F(1,40)
 sel=read(HERE/'results/source_probe.json');p=load('results/R_original.json.gz');st=source(p,sel['cycle']);models=[load(f'results/preflight/{arm}_{sel["cycle"]}_{c}_job.json.gz') for arm in ('F-E','O-E','F-P','O-P') for c in sel['actors']]
 for c in sel['actors']:
  own=[j for j in models if j['actor']==c];assert len({content_id(j['rows']) for j in own})==1 and len({content_id(j['receiver']) for j in own})==1
 for name in ('A1','A2'):
  cert=load(f'results/derivative_probe_{name}.json.gz');assert max(F(x['Ac']) for x in cert['bounds'])<15
 rows=read(HERE/'results/source_scan.initial.json')+[{'cycle':sel['cycle'],'reason':'first fully certified mixed-curve joint source','selected':True}]
 with (HERE/'source_scan.csv').open('w') as f:
  w=csv.DictWriter(f,fieldnames=['cycle','reason','selected']);w.writeheader();w.writerows(rows)
 bounds={c:load(f'results/derivative_probe_{c}.json.gz')['bounds'] for c in sel['actors']};write(HERE/'preflight_certificate.json',{'passed':True,'source':record(st),'actors':sel['actors'],'bounds_per_node':bounds,'zero_model_checks':'results/preflight/zero_model_certificate.json','direction_quantized_and_complete':'results/contract_certificate.json','reference_envelope':'results/reference_domain_audit.json','nonzero_direction':'results/preflight/progress.json','all_selected_first_source_conditions':True,'calls':0})
 sources={str(q.relative_to(HERE)):sha(q) for q in sorted(HERE.rglob('*.py')) if 'source_archive' not in q.parts};archive=HERE/'source_archive/numerical_v1';assert not archive.exists();archive.mkdir(parents=True)
 for n in sources:
  target=archive/n;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(HERE/n,target)
 write(HERE/'executed_source_versions.json',sources)
 import clarabel,numpy,scipy
 reg={'passed':True,'task':'curve_joint_001','limit':16,'source_cycle':sel['cycle'],'source':record(st),'actors':sel['actors'],'arms':['F-E','O-E','F-P','O-P'],'rounds':2,'lambda':1,'eta':'1/50','R':[1,1],'S':[1,1],'rho':'1/1000','h':'1/5','N':32,'quantum':'1/1000000000000','objective_multiplier':5,'failures_counted':True,'automatic_retry':False,'source_hashes':sources,'python':sys.version,'platform':platform.platform(),'dependencies':{'clarabel':clarabel.__version__,'numpy':numpy.__version__,'scipy':scipy.__version__},'solver_settings':{'max_iter':200,'tol_gap_abs':1e-10,'tol_gap_rel':1e-10,'tol_feas':1e-10,'time_limit':'infinity (existing default)','worker_channel_timeout_seconds':30},'old_reference':content_id(p),'fairness':'same first-source rows/receivers sealed and checked; freeze/energy-progress flags are sole arm differences; round2 actual own successors regenerated'};write(HERE/'pre_registration.json',reg);print('PASS frozen; budget 16; releases 0',flush=True)
if __name__=='__main__':run()
